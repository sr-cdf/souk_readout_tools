"""Bounded, thread-safe storage for complete readout stream frames.

The receiver only copies raw words into a ring. Parsing and readout correction
run on a detached snapshot, outside the receiver lock, using the same parser
as saved captures.
"""

import copy
import math
import operator
import threading
import time
from collections.abc import Mapping

import numpy as np

def _tone_indices(tones, num_tones):
    """Validate a tone selection; return its indices (first occurrence order)."""
    if isinstance(tones, (str, int, np.integer)):
        tones = [tones]
    indices = []
    for tone in tones:
        try:
            index = int(tone) if isinstance(tone, str) else operator.index(tone)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid tone index: {tone!r}") from exc
        if not 0 <= index < num_tones:
            raise ValueError(f"Tone {tone!r} is not present in this stream")
        if index not in indices:
            indices.append(index)
    return np.asarray(indices, dtype=int)


class StreamBuffer:
    """Keep a rolling interval of stream data in a preallocated raw ring.

    Parameters
    ----------
    history_seconds : float
        Retention duration, fixed for the lifetime of the buffer. The sample
        rate and number of tones are discovered from the first frame.
    max_bytes : int
        Maximum persistent ring allocation, including gap masks. Detached
        snapshots and analysis arrays need additional temporary memory.

    Notes
    -----
    ``append_frame`` is a ``receive_stream(frame_callback=...)`` callback.
    Missing accumulation counts occupy real time slots and appear as NaN I/Q
    in snapshots. Counter wrap is handled modulo 2**32. A backwards, reset,
    or repeated counter inserts one marked discontinuity slot because its
    elapsed duration cannot be inferred. Such slots also count towards
    ``missing_samples``. Sample indices always increase from stream start.
    """

    def __init__(self, history_seconds=60, max_bytes=512 * 1024**2):
        history_seconds = float(history_seconds)
        if not math.isfinite(history_seconds) or history_seconds <= 0:
            raise ValueError("history_seconds must be positive and finite")
        try:
            max_bytes = operator.index(max_bytes)
        except TypeError as exc:
            raise ValueError("max_bytes must be a positive integer") from exc
        if max_bytes <= 0:
            raise ValueError("max_bytes must be a positive integer")
        self._history_seconds = history_seconds
        self._max_bytes = max_bytes
        self._lock = threading.Lock()
        self._sample_rate = None
        self._capacity = 0
        self._raw = None
        self._missing = None
        self._discontinuity = None
        self._position = 0
        self._size = 0
        self._next_sample = 0
        self._last_counter = None
        self._total_received = 0
        self._missing_samples = 0
        self._num_tones = None
        self._info = None
        self._date = None
        self._updates = []

    @property
    def history_seconds(self):
        """Configured retention interval in seconds."""
        return self._history_seconds

    @property
    def sample_rate(self):
        """Sample rate in Hz, or None before the first frame."""
        with self._lock:
            return self._sample_rate

    @property
    def total_received(self):
        """Number of actual frames accepted, including evicted frames."""
        with self._lock:
            return self._total_received

    @property
    def missing_samples(self):
        """Cumulative missing time slots, including discontinuity markers."""
        with self._lock:
            return self._missing_samples

    @property
    def capacity(self):
        """Maximum retained sample slots, or zero before initialization."""
        with self._lock:
            return self._capacity

    @property
    def nbytes(self):
        """Bytes held by the persistent raw ring and its masks."""
        with self._lock:
            if self._raw is None:
                return 0
            return (self._raw.nbytes + self._missing.nbytes
                    + self._discontinuity.nbytes)

    def _initialize(self, words, sample_rate, metadata):
        """Allocate atomically, while the caller holds the lock."""
        slots = self._history_seconds * sample_rate
        bytes_per_slot = words.nbytes + 2  # raw frame and two bool masks
        if not math.isfinite(slots):
            raise MemoryError("Live history is too large; reduce history_seconds")
        capacity = max(1, math.ceil(slots))
        required = capacity * bytes_per_slot
        if required > self._max_bytes:
            supported = (self._max_bytes // bytes_per_slot) / sample_rate
            raise MemoryError(
                f"Live history requires {required / 1024**2:.1f} MiB for "
                f"{(words.size - 10) // 2} tones at {sample_rate:g} Hz, "
                f"exceeding max_bytes={self._max_bytes / 1024**2:.1f} MiB. "
                f"Reduce history_seconds to at most {supported:.6g}, stream "
                "fewer tones, or explicitly increase max_bytes. Display tone "
                "selection does not reduce raw history storage.")
        raw = np.empty((capacity, words.size), dtype='<i4')
        missing = np.empty(capacity, dtype=bool)
        discontinuity = np.empty(capacity, dtype=bool)
        info = copy.deepcopy(metadata.get('info', {}))
        self._raw, self._missing = raw, missing
        self._discontinuity = discontinuity
        self._capacity = capacity
        self._sample_rate = sample_rate
        self._num_tones = (words.size - 10) // 2
        self._info = info
        self._date = metadata.get('date', time.strftime('%Y-%m-%d %H:%M:%S UTC%z'))

    def _append_missing(self, count, discontinuity=False):
        """Insert gaps in O(min(count, capacity)) work, without large lists."""
        retained = min(count, self._capacity)
        if count >= self._capacity:
            # No previous samples survive. Reset physical layout so filling
            # even a years-long counter gap costs only one ring's storage.
            self._position = 0
            self._size = 0
        first = min(retained, self._capacity - self._position)
        consumed = 0
        for start, length in ((self._position, first), (0, retained - first)):
            if not length:
                continue
            section = slice(start, start + length)
            self._raw[section] = 0
            self._missing[section] = True
            self._discontinuity[section] = discontinuity
            first_counter = self._last_counter + count - retained + consumed + 1
            counters = np.arange(length, dtype=np.uint64) + first_counter
            self._raw[section, -2] = (counters & 0xFFFFFFFF).astype('<u4').view('<i4')
            consumed += length
        self._position = (self._position + retained) % self._capacity
        self._size = min(self._capacity, self._size + retained)
        self._next_sample += count
        self._missing_samples += count

    def append_frame(self, payload, metadata):
        """Copy one complete little-endian IQ frame and its ten-word trailer.

        ``metadata`` must contain a positive ``sample_rate`` and may contain
        ``info`` and ``date`` as supplied by ``receive_stream``. The actual
        payload, rather than a potentially stale metadata tone count,
        determines the ring width. A changed rate or frame width raises
        ``ValueError`` so an existing time base is never silently reused.
        """
        if not isinstance(metadata, Mapping):
            raise ValueError("Stream metadata must contain sample_rate")
        try:
            sample_rate = float(metadata['sample_rate'])
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError("Stream metadata must contain a positive sample_rate") from exc
        if not math.isfinite(sample_rate) or sample_rate <= 0:
            raise ValueError("Stream sample_rate must be positive and finite")
        try:
            payload = memoryview(payload)
            if payload.nbytes < 48 or payload.nbytes % 8:
                raise ValueError("Invalid stream frame: expected IQ pairs and ten trailer words")
            words = np.frombuffer(payload, dtype='<i4')
        except (TypeError, BufferError) as exc:
            raise ValueError("Stream payload must be a contiguous bytes-like frame") from exc
        counter = int(words[-2:].view('<u4')[0])
        with self._lock:
            if self._raw is None:
                self._initialize(words, sample_rate, metadata)
            elif words.size != self._raw.shape[1]:
                raise ValueError(
                    "Stream frame shape changed; start a new viewer for the new tone count")
            elif sample_rate != self._sample_rate:
                raise ValueError(
                    "Stream sample_rate changed; start a new viewer for the new sample rate")
            if self._last_counter is not None:
                delta = (counter - self._last_counter) & 0xFFFFFFFF
                if delta == 0 or delta >= 0x80000000:
                    self._append_missing(1, discontinuity=True)
                elif delta > 1:
                    self._append_missing(delta - 1)
            self._raw[self._position] = words
            self._missing[self._position] = False
            self._discontinuity[self._position] = False
            self._position = (self._position + 1) % self._capacity
            self._size = min(self._size + 1, self._capacity)
            self._next_sample += 1
            self._last_counter = counter
            self._total_received += 1

    def append_update(self, record):
        """Keep an ordered, sample-indexed modulation announcement."""
        if not isinstance(record, Mapping):
            raise ValueError('Stream update must be a record')
        payload = record.get('payload')
        if not isinstance(payload, Mapping):
            raise ValueError('Stream update record needs a payload')
        if payload.get('type') not in ('SNAPSHOT', 'TONE_UPDATE'):
            return
        if payload.get('op') == 'tracking_dry_run':
            return
        entry = copy.deepcopy(record)
        entry['stream_frame_index'] = int(entry['sample_index'])
        with self._lock:
            # Receiver callbacks arrive in stream order. Map the IQ-frame
            # count onto this ring's time slots, which also include gaps.
            entry['sample_index'] = self._next_sample
            self._updates.append(entry)

    def _ordered_copy(self, array, columns=None):
        """Copy the ring oldest-first, optionally keeping only ``columns``."""
        start = (self._position - self._size) % self._capacity
        stop = start + self._size
        parts = ([array[start:stop]] if stop <= self._capacity
                 else [array[start:], array[:stop - self._capacity]])
        if columns is not None:
            parts = [part[:, columns] for part in parts]
        return np.concatenate(parts)

    def snapshot(self, tones=None, *, apply_readout_correction=True):
        """Return an independent, chronologically ordered parsed timestream.

        ``tones`` may be one index/key or a sequence. Returned I/Q keys keep
        their original tone numbers and ``info`` retains the complete tone
        metadata, as expected by the timeseries extraction helpers.

        Additional fields are absolute ``sample_index``, ``time_s`` (seconds
        from first received frame), ``packet_missing`` and
        ``packet_discontinuity`` masks. Missing I/Q is NaN. Telescope ticks
        preserve uint64 precision; missing ticks are zero and must be masked
        with ``packet_missing``. Returns None until a frame has arrived.
        """
        # Keep importing the plotting package safe in server-only installs.
        from ..client.readout_client import ReadoutClient

        with self._lock:
            if not self._size:
                return None
            num_tones = self._num_tones
            indices = None if tones is None else _tone_indices(tones, num_tones)
            columns = None
            if indices is not None:
                # Only the selected tones' I/Q columns plus the ten trailer
                # words: the copy (made under the lock) and the parse scale
                # with the displayed tones, not the streamed ones.
                columns = np.r_[np.stack((2 * indices, 2 * indices + 1), axis=1).ravel(),
                                np.arange(2 * num_tones, 2 * num_tones + 10)]
            raw = self._ordered_copy(self._raw, columns)
            missing = self._ordered_copy(self._missing)
            discontinuity = self._ordered_copy(self._discontinuity)
            first_sample = self._next_sample - self._size
            sample_rate = self._sample_rate
            info = self._info       # set once in _initialize, never mutated
            date = self._date
            # Retain the last announcement before the ring and all later
            # announcements. Each new typed record carries full state.
            baseline = next((i for i in range(len(self._updates) - 1, -1, -1)
                             if self._updates[i]['sample_index'] <= first_sample), None)
            if baseline is not None and baseline > 0:
                del self._updates[:baseline]
            updates = copy.deepcopy(self._updates)
        info = copy.deepcopy(info)

        unsigned = raw.view('<u4')
        telescope_time = ((unsigned[:, -4].astype(np.uint64) << 32)
                          + unsigned[:, -3].astype(np.uint64))
        width = (raw.shape[1] - 10) // 2
        i_data, q_data = raw[:, :2 * width:2], raw[:, 1:2 * width:2]
        if np.any(missing):
            i_data, q_data = i_data.astype(float), q_data.astype(float)
            i_data[missing], q_data[missing] = np.nan, np.nan
        result = ReadoutClient._parsed_timestream(
            i_data, q_data, unsigned[:, -2].astype(np.int64),
            unsigned[:, -1].copy(), telescope_time, unsigned[:, -10:-4],
            sample_rate, info, date, apply_readout_correction,
            tone_indices=indices, total_tones=num_tones)
        result['stream_flags'] = {key: values.copy()
                                  for key, values in result['stream_flags'].items()}
        result['sample_index'] = first_sample + np.arange(len(raw), dtype=np.int64)
        result['time_s'] = result['sample_index'] / sample_rate
        result['packet_missing'] = missing
        result['packet_discontinuity'] = discontinuity
        result['tone_updates'] = updates
        # The over-voltage state recorded in the stream's info (as parse_samples).
        from souk_readout_tools.stream_file import capture_adc_overvoltage
        result['adc_overvoltage'], result['adc_overvoltage_events'] = \
            capture_adc_overvoltage({'info': info}, result['packet_counter'])
        return result
