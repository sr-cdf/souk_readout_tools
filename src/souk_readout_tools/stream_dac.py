"""
Real-time stream -> analog DAC output for KID frequency-shift readout.

This module supports the ``souk-stream-to-dac`` client script
(:mod:`souk_readout_tools.client.client_scripts.stream_to_dac`): it receives
the readout server's continuous TCP stream, converts a chosen tone's IQ into
a scalar (magnitude, phase, or frequency shift via this package's resonator
calibrations) and drives an analog output on a USB DAC (LabJack T-series) so
external instruments (e.g. the TOPTICA cw-THz control software) can record a
live voltage proportional to the KID response.

Signal chain::

    readout server --TCP stream--> reader thread --bounded queue--> output
    thread --IQ->x conversion--> x->volts mapping --> DAC backend

Contents
--------
- Frame handling: :func:`unpack_frame`, :class:`FrameAssembler` (the
  length-prefixed TCP framing used by ``ReadoutClient.receive_stream``).
- Frame sources: :class:`SocketFrameSource` (live server),
  :class:`MockFrameSource` (``ReadoutClient(mock=True)``),
  :class:`ReplayFrameSource` (recorded stream file + JSON sidecar).
- Readout correction: :class:`ReadoutCorrection` applies the same software
  readout-flattening factors as ``parse_samples(apply_readout_correction=True)``
  so stream IQ matches the units of parsed sweep data used for calibrations.
- Converters: :class:`MagnitudeConverter`, :class:`PhaseConverter`,
  :class:`DfModelFreeConverter`, :class:`DfCalibratedConverter` (and an
  :class:`FfmConverter` placeholder for the future fast-frequency-modulation
  demodulation mode; see ``doc/frequency_modulation.md``).
- DAC backends: :class:`DummyDac` (default, no hardware) and
  :class:`LabJackDac` (optional ``labjack-ljm`` dependency).
- Calibration persistence: :func:`save_calibrations`,
  :func:`load_calibrations`, :func:`calibrations_from_sweep`.
- The application: :class:`StreamToDac`.

See ``doc/stream_to_dac.md`` for wiring, configuration and the calibration
workflow.
"""

import json
import os
import socket
import struct
import threading
import time
from collections import deque

import numpy as np

# Number of int32 trailer words at the end of every stream frame:
# flag0..flag5, tt_msb, tt_lsb, cnt, err.
STREAM_TRAILER_WORDS = 10

# Stream frame types carried in the top byte of the 4-byte length prefix.
# Data frames are <= ~16 KB so the top byte is always 0x00 for legacy frames;
# nonzero values are reserved for typed (JSON) frames from newer servers.
FRAME_TYPE_DATA = 0x00


class StreamFrame:
    """One parsed stream frame: per-tone IQ plus the trailer words.

    Attributes
    ----------
    iq : numpy.ndarray
        Complex128 array of the active tones' IQ, in user order, raw
        accumulator units (no readout correction applied).
    flags : numpy.ndarray
        The six int32 flag words.
    tt : int
        64-bit PTP telescope timestamp.
    cnt : int
        32-bit frame counter (use to detect dropped frames).
    err : int
        Error word.
    point, settling, revision : int
        Fast-modulation tag decoded from flag5 (0/0/0 when not modulated):
        1-based active probe point, settling marker, config revision.
    host_time : float
        ``time.monotonic()`` at parse time (for watchdog/latency accounting).
    """

    __slots__ = ('iq', 'flags', 'tt', 'cnt', 'err',
                 'point', 'settling', 'revision', 'host_time')

    def __init__(self, iq, flags, tt, cnt, err, point, settling, revision,
                 host_time):
        self.iq = iq
        self.flags = flags
        self.tt = tt
        self.cnt = cnt
        self.err = err
        self.point = point
        self.settling = settling
        self.revision = revision
        self.host_time = host_time


def unpack_frame(payload):
    """Parse one raw stream-frame payload into a :class:`StreamFrame`.

    The payload layout matches ``ReadoutClient.parse_samples``: interleaved
    little-endian int32 I,Q pairs for the active tones in user order,
    followed by 10 trailer words (flags 0-5, tt_msb, tt_lsb, cnt, err).
    flag5 is decoded unsigned: bits 0-15 modulation point (1-based, 0 = off),
    bit 16 settling, bits 17-31 config revision.

    Parameters
    ----------
    payload : bytes
        One complete frame payload (no length prefix).

    Returns
    -------
    StreamFrame
    """
    words = np.frombuffer(payload, dtype='<i4')
    num_tones = (len(words) - STREAM_TRAILER_WORDS) // 2
    if num_tones < 0 or len(words) != 2 * num_tones + STREAM_TRAILER_WORDS:
        raise ValueError(f'bad frame payload length {len(payload)} bytes')
    iq = words[0:2 * num_tones:2].astype(np.float64) \
        + 1j * words[1:2 * num_tones:2].astype(np.float64)
    tail_u = words[-STREAM_TRAILER_WORDS:].view('<u4')
    flags = words[-STREAM_TRAILER_WORDS:-4].copy()
    f5 = int(tail_u[5])
    tt = (int(tail_u[6]) << 32) + int(tail_u[7])
    cnt = int(tail_u[8])
    err = int(words[-1])
    return StreamFrame(iq, flags, tt, cnt, err,
                       point=f5 & 0xFFFF,
                       settling=(f5 >> 16) & 0x1,
                       revision=(f5 >> 17) & 0x7FFF,
                       host_time=time.monotonic())


class FrameAssembler:
    """Incremental parser for the length-prefixed TCP stream framing.

    Feed it byte chunks as they arrive from the socket (any sizes, including
    partial frames) and it yields complete ``(frame_type, payload)`` tuples.
    Each frame on the wire is a 4-byte big-endian prefix followed by the
    payload. Legacy data frames put the payload length in all 32 bits (the
    top byte is always 0x00 at current frame sizes); typed frames from newer
    servers set a nonzero type in the top byte with the payload length in the
    low 24 bits. Zero-length keepalive frames are dropped.
    """

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data):
        """Add received bytes; return a list of complete (type, payload)."""
        self._buf.extend(data)
        frames = []
        while True:
            if len(self._buf) < 4:
                break
            prefix = struct.unpack_from('>I', self._buf, 0)[0]
            frame_type = (prefix >> 24) & 0xFF
            datalen = prefix if frame_type == FRAME_TYPE_DATA \
                else prefix & 0xFFFFFF
            if datalen == 0:
                # keepalive
                del self._buf[:4]
                continue
            if len(self._buf) < 4 + datalen:
                break
            frames.append((frame_type, bytes(self._buf[4:4 + datalen])))
            del self._buf[:4 + datalen]
        return frames


class StreamEnded(Exception):
    """Raised by a frame source when the stream has ended (EOF / disconnect)."""


class SocketFrameSource:
    """Live TCP stream frames from the readout server.

    Parameters
    ----------
    address : str
        Stream server address.
    port : int
        Stream server port.
    timeout : float, optional
        Socket receive timeout in seconds; :meth:`read` returns ``None``
        when nothing arrives within it so the caller can poll for shutdown.
    subscribe_updates : bool, optional
        Opt in to the server's typed frames (default False): on connect a
        one-line JSON subscribe message is sent, and the server then
        interleaves SNAPSHOT / TONE_UPDATE JSON frames (nonzero type in the
        length-prefix top byte) with the data stream. Decoded typed frames
        are appended to :attr:`typed_frames` for the consumer to drain
        (e.g. :class:`StreamToDac` logs them to a JSONL sidecar). Legacy
        servers simply never send any, so this is safe to enable against
        either.
    """

    def __init__(self, address, port, timeout=0.5, subscribe_updates=False):
        self.address = address
        self.port = int(port)
        self.timeout = float(timeout)
        self.subscribe_updates = bool(subscribe_updates)
        self.typed_frames = deque()   # decoded (type, dict) typed frames
        self._sock = None
        self._assembler = FrameAssembler()
        self._pending = deque()

    def open(self):
        """Connect to the stream server (and subscribe, if requested)."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._sock.settimeout(self.timeout)
        self._sock.connect((self.address, self.port))
        if self.subscribe_updates:
            self._sock.sendall(
                json.dumps({'subscribe': ['tone_updates']}).encode() + b'\n')

    def read(self):
        """Return the next data-frame payload, or ``None`` on timeout.

        Typed frames encountered along the way are decoded and queued on
        :attr:`typed_frames` rather than returned.

        Raises
        ------
        StreamEnded
            When the server closes the connection.
        """
        while not self._pending:
            try:
                chunk = self._sock.recv(65536)
            except socket.timeout:
                return None
            if not chunk:
                raise StreamEnded('stream server closed the connection')
            for frame_type, payload in self._assembler.feed(chunk):
                if frame_type == FRAME_TYPE_DATA:
                    self._pending.append(payload)
                else:
                    try:
                        self.typed_frames.append(
                            (frame_type, json.loads(payload.decode())))
                    except Exception as exc:
                        print(f'WARNING: undecodable typed frame '
                              f'(type {frame_type}): {exc}')
        return self._pending.popleft()

    def close(self):
        """Close the socket."""
        if self._sock is not None:
            try:
                self._sock.close()
            finally:
                self._sock = None


class MockFrameSource:
    """Frames generated by ``ReadoutClient(mock=True)``'s emulated server.

    Uses the mock server's ``stream_frame()`` generator (the same synthetic
    frames its ``receive_stream`` writes) throttled to the sample rate.

    Parameters
    ----------
    client : ReadoutClient
        A client constructed with ``mock=True``.
    num_tones : int or None, optional
        Tones per frame; ``None`` uses the mock's active tone count.
    """

    def __init__(self, client, num_tones=None):
        if not getattr(client, 'mock', False):
            raise ValueError('MockFrameSource needs a ReadoutClient(mock=True)')
        self._server = client._mock_server
        self._num_tones = num_tones
        self._period = 1.0 / client.get_sample_rate()
        self._next_time = None

    def open(self):
        """No-op (kept for interface symmetry)."""
        self._next_time = time.monotonic()

    def read(self):
        """Return the next synthetic frame payload, paced at the sample rate."""
        now = time.monotonic()
        if self._next_time is None:
            self._next_time = now
        if now < self._next_time:
            time.sleep(self._next_time - now)
        self._next_time += self._period
        return self._server.stream_frame(self._num_tones)

    def close(self):
        """No-op (kept for interface symmetry)."""


class ReplayFrameSource:
    """Re-emit frames from a recorded stream file at their true rate.

    Reads a binary recording produced by ``ReadoutClient.receive_stream`` (or
    by :class:`StreamToDac` itself) together with its JSON metadata sidecar
    (``<filename>.json``), and emits one frame payload per sample period.
    Replay plus the dummy DAC backend gives a full end-to-end test with zero
    hardware.

    Parameters
    ----------
    filename : str
        Path to the binary recording; ``filename + '.json'`` must exist.
    speed : float, optional
        Playback speed multiplier (default 1.0 = true rate; 0 = as fast
        as possible).
    loop : bool, optional
        Restart from the beginning at EOF instead of ending (default False).
    """

    def __init__(self, filename, speed=1.0, loop=False):
        self.filename = filename
        with open(filename + '.json') as f:
            self.metadata = json.load(f)
        self.num_tones = int(self.metadata['num_tones'])
        self.sample_rate = float(self.metadata['sample_rate'])
        self.info = self.metadata.get('info', {})
        self.frame_bytes = self.num_tones * 2 * 4 + STREAM_TRAILER_WORDS * 4
        self.speed = float(speed)
        self.loop = bool(loop)
        self._file = None
        self._next_time = None

    def open(self):
        """Open the recording."""
        self._file = open(self.filename, 'rb')
        self._next_time = time.monotonic()

    def read(self):
        """Return the next recorded frame payload, paced at the sample rate.

        Raises
        ------
        StreamEnded
            At end of file (unless ``loop``).
        """
        payload = self._file.read(self.frame_bytes)
        if len(payload) < self.frame_bytes:
            if self.loop and self._file.tell() > 0:
                self._file.seek(0)
                payload = self._file.read(self.frame_bytes)
            if len(payload) < self.frame_bytes:
                raise StreamEnded(f'end of recording {self.filename}')
        if self.speed > 0:
            now = time.monotonic()
            if now < self._next_time:
                time.sleep(self._next_time - now)
            self._next_time += (1.0 / self.sample_rate) / self.speed
        return payload

    def close(self):
        """Close the recording."""
        if self._file is not None:
            self._file.close()
            self._file = None


class ReadoutCorrection:
    """Software readout-flattening factors for stream IQ.

    Raw stream int32 IQ is in accumulator units; parsed data used to build
    calibrations has the per-(point, tone) software readout-flattening
    factors applied (``parse_samples(apply_readout_correction=True)``; the
    filterbank compensation's RX half cannot land in hardware). This class
    applies the same factors to stream frames, selected per sample via the
    flag5 modulation-point tag, so converted values match the calibration's
    units. It is the identity when no modulation state (and hence no
    correction) is present -- i.e. for plain unmodulated streams.

    Parameters
    ----------
    info : dict
        A ``get_info('all')`` payload (or the recording sidecar's ``info``).
    num_tones : int
        Active tone count (frame width).
    """

    def __init__(self, info, num_tones):
        from .modulation import readout_correction_factors
        self.num_tones = int(num_tones)
        self.factors = readout_correction_factors(info, num_tones)

    def apply(self, iq, point):
        """Return corrected IQ for one frame tagged with modulation ``point``."""
        if self.factors is None:
            return iq
        if 1 <= point <= self.factors.shape[0]:
            return iq * self.factors[point - 1, :len(iq)]
        return iq


# ---------------------------------------------------------------------------
# Converters: averaged complex IQ -> scalar x
# ---------------------------------------------------------------------------

class StreamConverter:
    """Base interface: convert one tone's (averaged) complex IQ to a scalar.

    Subclasses set ``units`` and implement :meth:`convert`. The output
    thread's df->volts mapping is applied afterwards, so converters return
    physical (or raw) values, not volts.
    """

    units = 'arb'

    def convert(self, iq):
        """Return the scalar x for complex ``iq`` (scalar or array)."""
        raise NotImplementedError


class MagnitudeConverter(StreamConverter):
    """``x = |IQ|`` in raw accumulator counts. Zero-calibration mode."""

    units = 'counts'

    def convert(self, iq):
        return np.abs(iq)


class PhaseConverter(StreamConverter):
    """``x = arg(IQ) - arg(IQ_ref)`` in radians, wrapped to (-pi, pi].

    Parameters
    ----------
    reference_iq : complex
        Reference IQ point (captured at startup or supplied in config).
    """

    units = 'rad'

    def __init__(self, reference_iq):
        reference_iq = complex(reference_iq)
        if reference_iq == 0:
            raise ValueError('reference_iq must be nonzero')
        self._ref_conj = np.conj(reference_iq) / abs(reference_iq)

    def convert(self, iq):
        return np.angle(iq * self._ref_conj)


class DfModelFreeConverter(PhaseConverter):
    """``x = phase / dphi_df`` in Hz: model-free frequency shift.

    Parameters
    ----------
    reference_iq : complex
        Reference IQ point (see :class:`PhaseConverter`).
    dphi_df : float
        Phase-vs-frequency slope at the operating point, in rad/Hz
        (e.g. from :func:`souk_readout_tools.modulation.demodulate`'s
        ``dphi_df`` or a sweep's local phase slope).
    """

    units = 'Hz'

    def __init__(self, reference_iq, dphi_df):
        super().__init__(reference_iq)
        dphi_df = float(dphi_df)
        if dphi_df == 0:
            raise ValueError('dphi_df must be nonzero')
        self._dphi_df = dphi_df

    def convert(self, iq):
        return super().convert(iq) / self._dphi_df


class DfCalibratedConverter(StreamConverter):
    """Calibrated Mobius conversion to probe detuning (Hz) or dissipation.

    Wraps ``ResonatorCalibration.tone_converter(f_tone)``: raw IQ is
    deembedded, phase-centered and inverted exactly, giving probe detuning
    ``f_probe - fr`` in Hz (``output='frequency'``) or the matched
    dissipation quadrature ``Delta(1/(2*Qi))`` (``output='dissipation'``).

    Parameters
    ----------
    calibration : ResonatorCalibration
        Calibration for this resonator (see :func:`load_calibrations`).
    f_tone : float
        The tone (probe) frequency in Hz.
    output : {'frequency', 'dissipation'}, optional
        Which coordinate to return (default 'frequency').
    """

    def __init__(self, calibration, f_tone, output='frequency'):
        if output not in ('frequency', 'dissipation'):
            raise ValueError("output must be 'frequency' or 'dissipation'")
        self._converter = calibration.tone_converter(float(f_tone))
        self._output = output
        self.units = 'Hz' if output == 'frequency' else 'd(1/2Qi)'

    def convert(self, iq):
        df, dd = self._converter(iq)
        return df if self._output == 'frequency' else dd


class FfmConverter(StreamConverter):
    """Placeholder for the fast-frequency-modulation demodulation mode.

    Not implemented yet. The plan (phase 2) is to demodulate whole
    modulation cycles from the stream (grouping samples by the flag5 point
    tag, as :func:`souk_readout_tools.modulation.demodulate` does offline)
    and output the per-cycle frequency shift. Whoever implements it must
    subtract the documented ``center - fr`` operating-point baseline -- see
    ``doc/frequency_modulation.md`` and ``modulation.params_from_sweep``.
    """

    def __init__(self, *args, **kwargs):
        raise NotImplementedError(
            'FFM demodulation output is not implemented yet; see '
            'doc/stream_to_dac.md and doc/frequency_modulation.md')


# ---------------------------------------------------------------------------
# DAC backends
# ---------------------------------------------------------------------------

class DacBackend:
    """Base interface for analog-output backends.

    ``write`` takes a dict of ``{channel_name: volts}``; ``read_ain`` takes a
    list of analog-input names and returns a list of volts. Both must be
    cheap and non-blocking apart from the device transaction itself.
    """

    def write(self, values):
        """Write ``{channel_name: volts}`` to the outputs."""
        raise NotImplementedError

    def read_ain(self, channels):
        """Read the listed analog inputs; return a list of volts."""
        raise NotImplementedError

    def close(self):
        """Release the device."""


class DummyDac(DacBackend):
    """No-hardware backend: remembers (and optionally prints) writes.

    The default when ``labjack-ljm`` is not installed, and what all
    development and tests use.

    Parameters
    ----------
    print_writes : bool, optional
        Print each write (default False; the app's status line already
        reports the latest voltage).
    """

    def __init__(self, print_writes=False):
        self.print_writes = bool(print_writes)
        self.last_values = {}
        self.write_count = 0
        self.history = deque(maxlen=10000)

    def write(self, values):
        self.last_values = dict(values)
        self.write_count += 1
        self.history.append((time.monotonic(), dict(values)))
        if self.print_writes:
            text = ' '.join(f'{k}={v:+.6f}V' for k, v in values.items())
            print(f'DummyDac: {text}')

    def read_ain(self, channels):
        return [0.0] * len(channels)

    def close(self):
        pass


class LabJackDac(DacBackend):
    """LabJack T-series backend via the LJM library (optional dependency).

    Uses ``labjack-ljm``'s ``eWriteNames`` / ``eReadNames``; supports DAC0/
    DAC1 (0-5 V) and LJTick-DAC channels (e.g. ``TDAC0``, +/-10 V). USB
    command-response transactions cost ~1 ms each, so keep the update rate in
    the low hundreds of Hz.

    Parameters
    ----------
    device_type : str, optional
        LJM device type ('ANY', 'T4', 'T7', ...; default 'ANY').
    connection_type : str, optional
        LJM connection ('ANY', 'USB', 'ETHERNET', ...; default 'ANY').
    identifier : str, optional
        LJM identifier (serial number, IP, or 'ANY'; default 'ANY').
    """

    def __init__(self, device_type='ANY', connection_type='ANY',
                 identifier='ANY'):
        try:
            from labjack import ljm
        except ImportError as exc:
            raise ImportError(
                "LabJack backend needs the 'labjack-ljm' package "
                "(pip install labjack-ljm) and the LJM library") from exc
        self._ljm = ljm
        self._handle = ljm.openS(device_type, connection_type, identifier)
        dev_info = ljm.getHandleInfo(self._handle)
        print(f'LabJackDac: opened device type {dev_info[0]}, '
              f'connection {dev_info[1]}, serial {dev_info[2]}')

    def write(self, values):
        names = list(values.keys())
        self._ljm.eWriteNames(self._handle, len(names), names,
                              [float(values[n]) for n in names])

    def read_ain(self, channels):
        if not channels:
            return []
        return list(self._ljm.eReadNames(self._handle, len(channels),
                                         list(channels)))

    def close(self):
        if self._handle is not None:
            try:
                self._ljm.close(self._handle)
            finally:
                self._handle = None


def make_dac_backend(backend, labjack_options=None, print_writes=False):
    """Build a DAC backend by name ('dummy' or 'labjack').

    'labjack' falls back to :class:`DummyDac` with a warning when the LJM
    package is unavailable, so development machines run the full pipeline.
    """
    if backend == 'labjack':
        try:
            return LabJackDac(**(labjack_options or {}))
        except ImportError as exc:
            print(f'WARNING: {exc}; falling back to dummy DAC backend')
            return DummyDac(print_writes=print_writes)
    if backend == 'dummy':
        return DummyDac(print_writes=print_writes)
    raise ValueError(f"unknown DAC backend '{backend}'")


# ---------------------------------------------------------------------------
# Calibration persistence
# ---------------------------------------------------------------------------

# The ResonatorCalibration constructor arguments serialised to npz. The
# group-delay calibration table is not serialised (from_fit never sets it;
# the scalar tau carries the fitted cable delay).
_CAL_FIELDS = ('fr', 'Ql', 'tau', 'center', 'radius', 'rotation_angle',
               'gain_amplitude', 'gain_phase', 'frequency_origin_fraction',
               'anl')


def save_calibrations(filename, calibrations):
    """Save per-tone resonator calibrations to a ``.npz`` file.

    Parameters
    ----------
    filename : str
        Output path (``.npz`` appended by numpy if missing).
    calibrations : dict
        ``{tone_index: ResonatorCalibration}``.
    """
    indices = sorted(calibrations)
    arrays = {'tone_indices': np.asarray(indices, dtype=int)}
    for field in _CAL_FIELDS:
        arrays[field] = np.asarray(
            [getattr(calibrations[i], field) for i in indices])
    np.savez(filename, **arrays)


def load_calibrations(filename):
    """Load ``{tone_index: ResonatorCalibration}`` saved by
    :func:`save_calibrations`."""
    from .resonator import ResonatorCalibration
    with np.load(filename) as data:
        indices = data['tone_indices'].astype(int)
        fields = {field: data[field] for field in _CAL_FIELDS}
    calibrations = {}
    for j, tone_index in enumerate(indices):
        kwargs = {field: fields[field][j] for field in _CAL_FIELDS}
        center = complex(kwargs.pop('center'))
        calibrations[int(tone_index)] = ResonatorCalibration(
            kwargs.pop('fr'), kwargs.pop('Ql'), kwargs.pop('tau'),
            center, kwargs.pop('radius'), kwargs.pop('rotation_angle'),
            **{k: float(v) for k, v in kwargs.items()})
    return calibrations


def calibrations_from_sweep(sweep_data, tone_indices=None, **batch_fit_kwargs):
    """Fit a parsed sweep and return ``{tone_index: ResonatorCalibration}``.

    Runs :func:`souk_readout_tools.fitting.batch_fit` on the sweep-data dict
    and builds a calibration from each successful fit. Save the result with
    :func:`save_calibrations` for ``mode: df_calibrated``.

    Parameters
    ----------
    sweep_data : dict
        Parsed sweep data (see ``ReadoutClient.parse_sweep_data``).
    tone_indices : list of int or None, optional
        Restrict to these tone indices; ``None`` keeps every fitted tone.
    **batch_fit_kwargs
        Forwarded to :func:`fitting.batch_fit`.
    """
    from .fitting import batch_fit
    from .resonator import ResonatorCalibration
    fits = batch_fit(sweep_data, **batch_fit_kwargs)
    calibrations = {}
    for fit in fits:
        if tone_indices is not None and fit.tone_index not in tone_indices:
            continue
        if not np.isfinite(fit.fr) or getattr(fit, 'noise_only', False):
            continue
        calibrations[int(fit.tone_index)] = ResonatorCalibration.from_fit(fit)
    return calibrations


# ---------------------------------------------------------------------------
# Output channels and the application
# ---------------------------------------------------------------------------

class OutputChannel:
    """One DAC output channel: a tone, a converter and an x->volts mapping.

    ``volts = clip(gain * (x - x_offset) + v_offset, v_min, v_max)``.

    Parameters
    ----------
    dac_channel : str
        Backend channel name (e.g. 'DAC0', 'DAC1', 'TDAC0').
    tone_position : int
        Position of the tone in the stream frame (user order).
    converter : StreamConverter
        IQ -> x converter for this channel.
    gain, x_offset, v_offset, v_min, v_max : float
        Mapping parameters (defaults: 1, 0, 0, 0, 5).
    """

    def __init__(self, dac_channel, tone_position, converter, gain=1.0,
                 x_offset=0.0, v_offset=0.0, v_min=0.0, v_max=5.0):
        self.dac_channel = str(dac_channel)
        self.tone_position = int(tone_position)
        self.converter = converter
        self.gain = float(gain)
        self.x_offset = float(x_offset)
        self.v_offset = float(v_offset)
        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.last_x = np.nan
        self.last_volts = np.nan

    def volts(self, x):
        """Map converter output ``x`` to a clipped output voltage."""
        v = self.gain * (x - self.x_offset) + self.v_offset
        return float(np.clip(v, self.v_min, self.v_max))


class StreamToDac:
    """Reader/output threads turning stream frames into DAC voltages.

    The reader thread receives frames from ``source``, always appends the
    raw payloads to disk (same file + JSON-sidecar format as
    ``ReadoutClient.receive_stream``) and pushes parsed frames onto a
    bounded drop-oldest queue. The output thread pops the latest frames,
    applies the readout correction, boxcar-averages ``N =
    round(sample_rate / dac_update_rate)`` samples per update, converts each
    channel's IQ and writes the DAC. On any exception, stream stall
    (watchdog) or exit the DAC is driven to the idle voltage.

    Parameters
    ----------
    source : SocketFrameSource or MockFrameSource or ReplayFrameSource
        Frame source (must be opened by :meth:`run`).
    info : dict
        ``get_info('all')`` payload (or replay sidecar info) used for the
        readout correction and the recording sidecar.
    sample_rate : float
        Stream sample rate in Hz.
    num_tones : int
        Active tones per frame.
    channels : list of OutputChannel
        Output channels (at least one).
    dac : DacBackend
        DAC backend.
    dac_update_rate : float, optional
        DAC update rate in Hz (default 200).
    idle_voltage : float, optional
        Safe-state voltage driven on every channel at exit (default 0).
    queue_depth : int or None, optional
        Reader->output queue depth in frames; bounds worst-case staleness
        at ``queue_depth / sample_rate`` on top of the averaging window.
        ``None`` (default) uses ``max(8, 2 * N)`` where N is the boxcar
        length, so averaging is never starved by the bound.
    watchdog_s : float, optional
        Exit to safe state when no frame arrives for this long (default 5).
    recording_path : str or None, optional
        Basename for the raw recording (+ '.json' sidecar); ``None``
        disables recording.
    ain_channels : list of str, optional
        Analog inputs to sample each output update (default none). Each
        reading is logged to ``recording_path + '.ain.csv'`` (or
        ``ain_log_path``) with host time and the tt of the nearest frame.
    ain_log_path : str or None, optional
        Override the AIN log path.
    status_interval : float, optional
        Seconds between status lines (default 1.0; 0 disables).
    quiet : bool, optional
        Suppress the status line (default False).
    """

    def __init__(self, source, info, sample_rate, num_tones, channels, dac,
                 dac_update_rate=200.0, idle_voltage=0.0, queue_depth=None,
                 watchdog_s=5.0, recording_path=None, ain_channels=(),
                 ain_log_path=None, status_interval=1.0, quiet=False,
                 apply_readout_correction=True, max_updates=None):
        if not channels:
            raise ValueError('at least one output channel is required')
        self.source = source
        self.info = info
        self.sample_rate = float(sample_rate)
        self.num_tones = int(num_tones)
        self.channels = list(channels)
        self.dac = dac
        self.dac_update_rate = float(dac_update_rate)
        self.idle_voltage = float(idle_voltage)
        self.navg = max(1, int(round(self.sample_rate / self.dac_update_rate)))
        if queue_depth is None:
            queue_depth = max(8, 2 * self.navg)
        self.queue_depth = int(queue_depth)
        self.watchdog_s = float(watchdog_s)
        self.recording_path = recording_path
        self.ain_channels = list(ain_channels)
        self.ain_log_path = ain_log_path
        self.status_interval = float(status_interval)
        self.quiet = bool(quiet)
        self.max_updates = max_updates

        if apply_readout_correction:
            self.correction = ReadoutCorrection(info, self.num_tones)
        else:
            self.correction = None

        self._queue = deque(maxlen=self.queue_depth)
        self._stop = threading.Event()
        self._stop_reason = None
        self._last_frame_time = None
        self._last_tt = 0

        # Diagnostics
        self.frames_received = 0
        self.frames_dropped = 0     # cnt gaps on the wire
        self.queue_drops = 0        # bounded-queue drop-oldest events
        self.updates_written = 0
        self.loop_times = deque(maxlen=10000)
        self._prev_cnt = None

    # -- reader thread ------------------------------------------------------

    def _write_sidecar(self):
        metadata = {
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
            'num_tones': self.num_tones,
            'sample_rate': self.sample_rate,
            'format': '<i4',
            'index_err': 2 * self.num_tones - 1 + 10,
            'index_cnt': 2 * self.num_tones - 1 + 9,
            'index_tt_lsb': 2 * self.num_tones - 1 + 8,
            'index_tt_msb': 2 * self.num_tones - 1 + 7,
            'index_flag_5': 2 * self.num_tones - 1 + 6,
            'index_flag_4': 2 * self.num_tones - 1 + 5,
            'index_flag_3': 2 * self.num_tones - 1 + 4,
            'index_flag_2': 2 * self.num_tones - 1 + 3,
            'index_flag_1': 2 * self.num_tones - 1 + 2,
            'index_flag_0': 2 * self.num_tones - 1 + 1,
            'ordering': ('I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, '
                         'Q_tone1_sample0,..flags, tt_msb, tt_lsb, cnt, err .'),
            'info': self.info,
        }
        with open(self.recording_path + '.json', 'w') as f:
            json.dump(metadata, f, indent=4)

    def _drain_typed_frames(self, updates_log):
        """Write any typed frames the source has decoded to the JSONL log."""
        typed = getattr(self.source, 'typed_frames', None)
        if not typed:
            return
        while True:
            try:
                frame_type, payload = typed.popleft()
            except IndexError:
                break
            record = {'host_time_unix': time.time(),
                      'frame_type': frame_type, 'payload': payload}
            if updates_log is not None:
                updates_log.write(json.dumps(record) + '\n')
                updates_log.flush()
            else:
                kind = payload.get('type', f'type{frame_type}')
                print(f'\nstream update frame: {kind} '
                      f'revision={payload.get("revision")} '
                      f'op={payload.get("op")}')

    def _reader(self):
        """Reader thread: source -> disk + bounded queue."""
        recording = None
        updates_log = None
        try:
            if self.recording_path is not None:
                directory = os.path.dirname(os.path.abspath(
                    self.recording_path))
                os.makedirs(directory, exist_ok=True)
                self._write_sidecar()
                recording = open(self.recording_path, 'wb')
                print(f'Recording raw stream to {self.recording_path}')
                if getattr(self.source, 'subscribe_updates', False):
                    updates_log = open(
                        self.recording_path + '.updates.jsonl', 'w')
                    print('Logging tone-update frames to '
                          f'{self.recording_path}.updates.jsonl')
            while not self._stop.is_set():
                payload = self.source.read()
                self._drain_typed_frames(updates_log)
                if payload is None:
                    continue
                if recording is not None:
                    recording.write(payload)
                frame = unpack_frame(payload)
                if self._prev_cnt is not None:
                    gap = (frame.cnt - self._prev_cnt) & 0xFFFFFFFF
                    if gap > 1:
                        self.frames_dropped += gap - 1
                self._prev_cnt = frame.cnt
                if len(self._queue) == self._queue.maxlen:
                    self.queue_drops += 1
                self._queue.append(frame)
                self.frames_received += 1
                self._last_frame_time = frame.host_time
                self._last_tt = frame.tt
        except StreamEnded as exc:
            self._request_stop(f'stream ended: {exc}')
        except Exception as exc:
            self._request_stop(f'reader error: {exc!r}')
        finally:
            if recording is not None:
                recording.close()
            if updates_log is not None:
                updates_log.close()

    # -- output thread ------------------------------------------------------

    def _request_stop(self, reason):
        if self._stop_reason is None:
            self._stop_reason = reason
        self._stop.set()

    def _selected_iq(self, frame):
        """Corrected IQ values of the channels' tones for one frame."""
        iq = frame.iq
        if self.correction is not None:
            iq = self.correction.apply(iq, frame.point)
        return [iq[ch.tone_position] for ch in self.channels]

    def _open_ain_log(self):
        if not self.ain_channels:
            return None
        path = self.ain_log_path or (
            (self.recording_path or 'stream_to_dac') + '.ain.csv')
        f = open(path, 'w')
        f.write('host_time_unix,tt,' + ','.join(self.ain_channels) + '\n')
        print(f'Logging AIN readings to {path}')
        return f

    def capture_reference_iq(self, n_samples=100, timeout=10.0):
        """Average the first ``n_samples`` frames' IQ per channel tone.

        Used to build phase-referenced converters at startup. Must be
        called after the reader thread has started. Applies the readout
        correction. Returns a list of complex references, one per channel.
        """
        collected = [[] for _ in self.channels]
        deadline = time.monotonic() + timeout
        while len(collected[0]) < n_samples:
            try:
                frame = self._queue.popleft()
            except IndexError:
                if self._stop.is_set() or time.monotonic() > deadline:
                    # A short stream (e.g. replay) may end early: accept a
                    # partial capture rather than fail with frames in hand.
                    if collected[0]:
                        break
                    raise RuntimeError(
                        'timed out capturing reference IQ '
                        f'({len(collected[0])}/{n_samples} frames)')
                time.sleep(0.5 / self.sample_rate)
                continue
            for values, iq in zip(collected, self._selected_iq(frame)):
                values.append(iq)
        return [complex(np.mean(values)) for values in collected]

    def _output_loop(self):
        """Output thread body: queue -> average -> convert -> DAC."""
        period = 1.0 / self.dac_update_rate
        window = deque(maxlen=self.navg)
        ain_log = self._open_ain_log()
        next_update = time.monotonic()
        last_status = 0.0
        final_flush = False
        try:
            while True:
                if self._stop.is_set():
                    if final_flush:
                        break
                    # One last pass so frames queued before the stop (e.g.
                    # end of a replay) still produce a DAC update.
                    final_flush = True
                else:
                    now = time.monotonic()
                    if now < next_update:
                        time.sleep(min(next_update - now, period))
                        continue
                    next_update += period
                    # If we fell far behind, resynchronise, don't burst.
                    if now - next_update > 5 * period:
                        next_update = now + period

                t_loop = time.monotonic()
                # Drain the queue into the boxcar window.
                while True:
                    try:
                        frame = self._queue.popleft()
                    except IndexError:
                        break
                    window.append(self._selected_iq(frame))

                # Watchdog: no frames for too long -> safe state + exit.
                if self._last_frame_time is not None and \
                        time.monotonic() - self._last_frame_time > \
                        self.watchdog_s:
                    self._request_stop(
                        f'watchdog: no stream frame for {self.watchdog_s} s')
                    break

                if window:
                    mean_iq = np.mean(np.asarray(window), axis=0)
                    values = {}
                    for ch, iq in zip(self.channels, mean_iq):
                        x = float(np.real_if_close(ch.converter.convert(iq)))
                        ch.last_x = x
                        ch.last_volts = ch.volts(x)
                        values[ch.dac_channel] = ch.last_volts
                    self.dac.write(values)
                    self.updates_written += 1
                    if ain_log is not None:
                        readings = self.dac.read_ain(self.ain_channels)
                        ain_log.write(
                            f'{time.time():.6f},{self._last_tt},' +
                            ','.join(f'{v:.6f}' for v in readings) + '\n')
                self.loop_times.append(time.monotonic() - t_loop)

                if self.max_updates is not None and \
                        self.updates_written >= self.max_updates:
                    self._request_stop('max_updates reached')
                    break

                if not self.quiet and self.status_interval > 0 and \
                        time.monotonic() - last_status > self.status_interval:
                    last_status = time.monotonic()
                    self._print_status()
        except Exception as exc:
            self._request_stop(f'output error: {exc!r}')
        finally:
            if ain_log is not None:
                ain_log.close()

    def _print_status(self):
        ch = self.channels[0]
        loop_ms = 1e3 * float(np.mean(self.loop_times)) if self.loop_times \
            else 0.0
        print(f'x={ch.last_x:+.4g} {ch.converter.units}  '
              f'V={ch.last_volts:+.4f}  '
              f'frames={self.frames_received}  '
              f'updates={self.updates_written}  '
              f'dropped(frames/queue)={self.frames_dropped}/'
              f'{self.queue_drops}  loop={loop_ms:.2f} ms\r',
              end='', flush=True)

    # -- lifecycle ----------------------------------------------------------

    def safe_state(self, reason):
        """Drive every channel to the idle voltage and log ``reason``."""
        print(f'\nEntering safe state ({reason}): '
              f'driving outputs to {self.idle_voltage} V')
        try:
            self.dac.write({ch.dac_channel: self.idle_voltage
                            for ch in self.channels})
        except Exception as exc:
            print(f'WARNING: safe-state DAC write failed: {exc!r}')

    def start_reader(self):
        """Open the source and start the reader thread (for reference
        capture before :meth:`run_output`)."""
        self.source.open()
        self._last_frame_time = time.monotonic()
        self._reader_thread = threading.Thread(
            target=self._reader, name='stream-to-dac-reader', daemon=True)
        self._reader_thread.start()

    def run_output(self):
        """Run the output loop until stopped; always exits via safe state."""
        try:
            self._output_loop()
        except (KeyboardInterrupt, SystemExit):
            self._request_stop('interrupted')
        finally:
            self._stop.set()
            self.safe_state(self._stop_reason or 'exit')
            self.source.close()
            self.dac.close()
            self._print_summary()

    def run(self):
        """Start the reader and run the output loop (blocking)."""
        self.start_reader()
        self.run_output()

    def _print_summary(self):
        print(f'stream-to-dac finished: {self.frames_received} frames, '
              f'{self.updates_written} DAC updates, '
              f'{self.frames_dropped} dropped frames, '
              f'{self.queue_drops} queue drops'
              + (f' ({self._stop_reason})' if self._stop_reason else ''))
        if self.loop_times:
            times = np.asarray(self.loop_times)
            print(f'output loop: mean {1e3*times.mean():.2f} ms, '
                  f'max {1e3*times.max():.2f} ms, '
                  f'boxcar N={self.navg} '
                  f'(~{1e3*self.navg/self.sample_rate:.1f} ms window)')


# ---------------------------------------------------------------------------
# Config -> objects
# ---------------------------------------------------------------------------

def resolve_tone_position(info, tone_index=None, tone_frequency_hz=None):
    """Resolve a tone selection to its position in the stream frame.

    The stream carries active tones in user order, so a user-order
    ``tone_index`` is the position directly (validated against
    ``info['tones']['firmware_indices']``). A ``tone_frequency_hz`` selects
    the nearest entry of ``info['tones']['frequencies_hz']``.
    """
    tones = info.get('tones', {}) if isinstance(info, dict) else {}
    firmware_indices = tones.get('firmware_indices')
    n = len(firmware_indices) if firmware_indices is not None else None
    if tone_frequency_hz is not None:
        frequencies = tones.get('frequencies_hz')
        if frequencies is None:
            raise ValueError('info has no tones.frequencies_hz to resolve '
                             'tone_frequency_hz against')
        frequencies = np.asarray(frequencies, dtype=float)
        position = int(np.nanargmin(np.abs(frequencies - tone_frequency_hz)))
        offset = frequencies[position] - tone_frequency_hz
        if abs(offset) > 1.0:
            print(f'Tone {tone_frequency_hz/1e6:.6f} MHz resolved to stream '
                  f'position {position} ({frequencies[position]/1e6:.6f} MHz, '
                  f'{offset:+.1f} Hz away)')
        return position
    if tone_index is None:
        raise ValueError('give tone_index or tone_frequency_hz')
    tone_index = int(tone_index)
    if n is not None and not 0 <= tone_index < n:
        raise ValueError(f'tone_index {tone_index} out of range '
                         f'(0..{n-1} active tones)')
    return tone_index


def build_converter(mode, conversion_config, info, tone_position,
                    reference_iq=None):
    """Build a :class:`StreamConverter` from config.

    Parameters
    ----------
    mode : {'magnitude', 'phase', 'df_modelfree', 'df_calibrated', 'ffm'}
        Conversion mode.
    conversion_config : dict
        The config file's ``conversion`` section.
    info : dict
        System info (for the tone frequency in calibrated mode).
    tone_position : int
        Stream position of the tone (user order).
    reference_iq : complex or None, optional
        Startup-captured reference for the phase-based modes; overridden by
        an explicit ``conversion.reference_iq`` in the config.
    """
    conversion_config = conversion_config or {}
    configured_ref = conversion_config.get('reference_iq')
    if configured_ref is not None:
        reference_iq = complex(configured_ref[0], configured_ref[1])
    if mode == 'magnitude':
        return MagnitudeConverter()
    if mode == 'phase':
        if reference_iq is None:
            raise ValueError('phase mode needs a reference IQ')
        return PhaseConverter(reference_iq)
    if mode == 'df_modelfree':
        if reference_iq is None:
            raise ValueError('df_modelfree mode needs a reference IQ')
        dphi_df = conversion_config.get('dphi_df')
        if dphi_df is None:
            raise ValueError('df_modelfree mode needs conversion.dphi_df '
                             '(rad/Hz)')
        return DfModelFreeConverter(reference_iq, dphi_df)
    if mode == 'df_calibrated':
        calibration_file = conversion_config.get('calibration_file')
        if calibration_file is None:
            raise ValueError('df_calibrated mode needs '
                             'conversion.calibration_file')
        calibrations = load_calibrations(calibration_file)
        if tone_position not in calibrations:
            raise ValueError(f'calibration file {calibration_file} has no '
                             f'entry for tone {tone_position} '
                             f'(has {sorted(calibrations)})')
        frequencies = info.get('tones', {}).get('frequencies_hz')
        if frequencies is None:
            raise ValueError('info has no tones.frequencies_hz for the '
                             'calibrated converter tone frequency')
        f_tone = float(np.asarray(frequencies, dtype=float)[tone_position])
        return DfCalibratedConverter(
            calibrations[tone_position], f_tone,
            output=conversion_config.get('output', 'frequency'))
    if mode == 'ffm':
        return FfmConverter()
    raise ValueError(f"unknown conversion mode '{mode}'")
