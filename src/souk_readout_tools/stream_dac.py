"""
Real-time stream -> analog DAC output for KID frequency-shift readout.

This module supports the ``souk-stream-to-dac`` client script
(:mod:`souk_readout_tools.client.client_scripts.stream_to_dac`): it receives
the readout server's continuous TCP stream, converts a chosen tone's IQ into
a scalar (magnitude, phase, or any resonance quantity of
``resonator.QUANTITIES``) and drives an analog output -- a USB DAC (LabJack
T-series) or a sound card via ALSA -- so external instruments (e.g. the
TOPTICA cw-THz control software) can record a live voltage proportional to
the KID response. The conversions are ``resonator``'s own (same names, signs
and maths as offline analysis); this module only feeds them live frames.

Signal chain::

    readout server --TCP stream--> reader thread --bounded queue--> output
    thread --IQ->x conversion--> x->volts mapping --> DAC backend

Contents
--------
- Frame handling: :func:`unpack_frame`, :class:`FrameAssembler` (the
  length-prefixed TCP framing used by ``ReadoutClient.receive_stream``).
- Frame sources: :class:`SocketFrameSource` (live server),
  :class:`MockFrameSource` (``ReadoutClient(mock=True)``),
  :class:`ReplayFrameSource` (recorded stream file + JSON sidecar, with its
  tone updates replayed in place).
- Readout correction: :class:`ReadoutCorrection` applies the same software
  readout-flattening factors as ``parse_samples(apply_readout_correction=True)``,
  each revision's own once announced, so stream IQ matches the units of
  parsed sweep data used for calibrations.
- Converters: :class:`MagnitudeConverter`, :class:`PhaseConverter`,
  :class:`DfModelFreeConverter`, :class:`QuantityConverter` (fixed tone,
  calibrated) and :class:`FfmConverter` (live three-point demodulation with
  recentre-proof output; see ``doc/frequency_modulation.md``).
- DAC backends: :class:`DummyDac` (default, no hardware),
  :class:`LabJackDac` (T-series, optional ``labjack-ljm`` dependency),
  :class:`LabJackU12Dac` (U12, optional ``LabJackPython`` + Exodriver) and
  :class:`AlsaDac` (sound card, optional ``pyalsaaudio`` -- AC-coupled, so
  characterise it with :func:`alsa_selftest` before trusting a slow scan).
- The application: :class:`StreamToDac`.

Calibrations are saved and loaded with ``resonator.save_calibrations`` /
``load_calibrations`` / ``calibrations_from_sweep`` (re-exported here), or
pushed to the server with ``ReadoutClient.set_resonator_calibrations``, which
the calibrated modes pick up from the stream's info without a file.

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

from . import resonator
from .modulation import CYCLE_FIELDS, active_modulation_state
# Calibration files are resonator's; re-exported for existing scripts.
from .resonator import calibrations_from_sweep, load_calibrations, save_calibrations  # noqa: F401

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
        are appended to :attr:`typed_frames` as ``(type, payload,
        sample_index)`` -- the index being the number of data frames before
        it, as ``ReadoutClient.receive_stream`` records -- for the consumer
        to drain (e.g. :class:`StreamToDac` logs them to a JSONL sidecar).
        Legacy servers simply never send any, so this is safe to enable
        against either.
    """

    def __init__(self, address, port, timeout=0.5, subscribe_updates=False):
        self.address = address
        self.port = int(port)
        self.timeout = float(timeout)
        self.subscribe_updates = bool(subscribe_updates)
        self.typed_frames = deque()   # decoded (type, dict, sample_index)
        self._sock = None
        self._assembler = FrameAssembler()
        self._pending = deque()
        self._data_frames = 0         # data frames assembled so far

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
                    self._data_frames += 1
                else:
                    try:
                        self.typed_frames.append(
                            (frame_type, json.loads(payload.decode()),
                             self._data_frames))
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
    A recording's tone updates (``<filename>.updates.jsonl``) are replayed
    as typed frames just before the frame they preceded (their
    ``sample_index``; records without one, from older recordings, at the
    start), so recentres and per-revision readout correction replay as they
    happened. Replay plus the dummy DAC backend gives a full end-to-end test
    with zero hardware.

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
        self._updates = []
        updates_path = filename + '.updates.jsonl'
        if os.path.exists(updates_path):
            with open(updates_path) as f:
                self._updates = [json.loads(line) for line in f if line.strip()]
        self.subscribe_updates = bool(self._updates)
        self.typed_frames = deque()   # (type, dict, sample_index), as live
        self._next_update = 0
        self._frames_read = 0

    def open(self):
        """Open the recording."""
        self._file = open(self.filename, 'rb')
        self._next_time = time.monotonic()
        self._next_update = 0
        self._frames_read = 0

    def _queue_updates(self):
        """Queue the recorded typed frames that preceded the next frame."""
        while self._next_update < len(self._updates):
            record = self._updates[self._next_update]
            if int(record.get('sample_index', 0)) > self._frames_read:
                break
            self.typed_frames.append((record.get('frame_type', 0),
                                      record['payload'], self._frames_read))
            self._next_update += 1

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
                self._next_update = 0
                self._frames_read = 0
                payload = self._file.read(self.frame_bytes)
            if len(payload) < self.frame_bytes:
                raise StreamEnded(f'end of recording {self.filename}')
        self._queue_updates()
        self._frames_read += 1
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
    applies the same factors (``resonator.readout_correction_factors``) to
    stream frames, selected per sample via the flag5 modulation-point tag, so
    converted values match the calibration's units. A tracking commit or
    recentre moves the probes across the filterbank and so changes the
    factors: each revision's own are used once announced (:meth:`note_state`,
    fed from the typed tone-update frames), the startup ones otherwise. It is
    the identity when no modulation state (and hence no correction) is
    present -- i.e. for plain unmodulated streams.

    Parameters
    ----------
    info : dict
        A ``get_info('all')`` payload (or the recording sidecar's ``info``).
    num_tones : int
        Active tone count (frame width).
    """

    def __init__(self, info, num_tones):
        self.num_tones = int(num_tones)
        self.factors = resonator.readout_correction_factors(info, num_tones)
        self._by_revision = {}

    def note_state(self, revision, state):
        """Use a revision's factors, from its announced ``modulation_state``."""
        factors = resonator.readout_correction_factors(
            {'modulation': state}, self.num_tones)
        if factors is not None:
            self._by_revision[int(revision) & 0x7FFF] = factors

    def apply(self, iq, point, revision=None):
        """Return corrected IQ for one frame tagged with modulation ``point``
        (and ``revision``: its own factors when announced)."""
        factors = self._by_revision.get(revision, self.factors)
        if factors is None:
            return iq
        if 1 <= point <= factors.shape[0]:
            return iq * factors[point - 1, :len(iq)]
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
    """``x = -phase / dphi_df`` in Hz: a model-free resonance shift.

    Positive when the resonance moves up (``resonator``'s resonator-side
    sign; the probe-side ``+phase / dphi_df`` is its negative). ``dphi_df``
    must be the slope of the same raw phase -- about the I/Q origin -- at the
    operating point, e.g. from the calibration sweep. About the origin the
    phase also moves with loss, so this reads some dissipation as frequency:
    a quick look. ``df_calibrated`` and ``ffm`` separate the two.

    Parameters
    ----------
    reference_iq : complex
        Reference IQ point (see :class:`PhaseConverter`).
    dphi_df : float
        Slope of the raw phase against probe frequency at the operating
        point, in rad/Hz (negative for a notch).
    """

    units = 'Hz'

    def __init__(self, reference_iq, dphi_df):
        super().__init__(reference_iq)
        dphi_df = float(dphi_df)
        if dphi_df == 0:
            raise ValueError('dphi_df must be nonzero')
        self._dphi_df = dphi_df

    def convert(self, iq):
        return -super().convert(iq) / self._dphi_df


def _check_output(output, calibration, cycle=False):
    """The canonical name of a converter ``output``, checked against what
    the conversion can deliver for this tone.

    ``resonator.express`` is the one place that knows which quantities need a
    calibration (or its Qi), so the check is a trial call on zeros.
    """
    name = resonator.canonical_quantity(output)
    if cycle and name in CYCLE_FIELDS:
        if name.startswith('center_detuning') and calibration is None:
            raise ValueError(
                f'output {name!r} needs a calibration for the tone (about the '
                'raw I/Q origin the detuning reads 13x-100x high)')
        return name
    try:
        resonator.express(name, 0.0, 0.0, frequency_hz=1.0,
                          calibration=calibration, reference_resonance_hz=0.0)
    except ValueError as exc:
        raise ValueError(f'output {output!r}: {exc}') from None
    return name


class QuantityConverter(StreamConverter):
    """Any resonance quantity of one fixed tone, by ``resonator``'s conversions.

    ``resonator.prepare_conversion`` once, then per update
    ``resonator.convert_iq`` and ``resonator.express`` -- the functions
    ``resonator.convert_samples`` uses offline, so the DAC reads what an
    analysis of the recording would. The shift is relative to the
    calibration sweep's I/Q at the tone when the sweep is given (the
    resonator's state during the sweep), else to the fitted state.

    Parameters
    ----------
    frequency_hz : float
        The tone (probe) frequency in Hz.
    output : str, optional
        A ``resonator.QUANTITIES`` resonance quantity or alias (default
        ``'resonance_shift_hz'``, positive = resonance moved up).
    method : str, optional
        One of ``resonator.CONVERSION_METHODS`` (default ``'mobius'``, the
        exact fitted inversion; ``'linearized'`` and ``'lookup'`` need the
        sweep).
    calibration : ResonatorCalibration or None, optional
        This tone's calibration (needed by all but the sweep-only methods).
    sweep_f, sweep_z : array-like or None, optional
        This tone's calibration sweep: frequencies (Hz) and complex I/Q.
    gain : float, optional
        Data over calibration RX amplitude (``resonator.resolve_rx_gain``),
        divided out first (default 1).
    smooth_window_hz : float, optional
        Sweep smoothing for the sweep-based methods (default 1000).
    """

    def __init__(self, frequency_hz, output='resonance_shift_hz',
                 method='mobius', *, calibration=None, sweep_f=None,
                 sweep_z=None, gain=1.0, smooth_window_hz=1000):
        self.output = _check_output(output, calibration)
        self.units = resonator.QUANTITIES[self.output][1]
        self.frequency_hz = float(frequency_hz)
        self.calibration = calibration
        self.gain = float(gain)
        self._prepared = resonator.prepare_conversion(
            self.frequency_hz, method, calibration=calibration,
            sweep_f=sweep_f, sweep_z=sweep_z,
            smooth_window_hz=smooth_window_hz)

    def convert(self, iq):
        shift, dissipation = resonator.convert_iq(self._prepared, iq,
                                                  gain=self.gain)
        return resonator.express(self.output, shift, dissipation,
                                 frequency_hz=self.frequency_hz,
                                 calibration=self.calibration)


class FfmConverter(StreamConverter):
    """Live three-point (FFM) demodulation of one tone.

    Consumes the modulated stream frame by frame (``wants_frames = True``):
    samples are grouped into modulation cycles by the flag5 point tag
    (settling excluded, incomplete cycles dropped), and each cycle's point
    means go through the kernel that ``modulation.demodulate`` and the
    server's tracker use:

    - ``resonator.phase_derivatives`` (any slot order; points sharing an
      offset averaged coherently) gives a live ``dphi_df`` -- the modulation
      measures its own responsivity every cycle -- and the phase curvature,
      hence the centre detuning (resonance minus modulation centre);
    - the centre point against a reference -- its mean over the first
      ``baseline_cycles`` cycles, so the output starts from zero -- gives
      the resonance shift and matched dissipation. With a ``calibration``
      this happens in its circle-centred basis, by
      ``resonator.phase_shift_and_dissipation`` on the live slope (the
      preferred conversion); without one, on the raw I/Q, by the
      tangent/normal projection on the live gradient.

    A tracking (or client) recentre moves the probe and bumps the revision
    stamped in every frame. Each revision's centre and offsets are
    registered with :meth:`note_center` (:class:`StreamToDac` feeds it the
    SNAPSHOT / TONE_UPDATE typed frames) and the output stays the
    resonator's own motion since the reference, by the reconstruction
    identity of ``doc/tone_tracking.md``::

        resonance shift = (center[rev] - center[ref]) + shift about the reference

    The centred basis depends only on the detuning, so this is exact to
    first order in the detuning excursion, which tracking keeps small. On the
    raw I/Q the cable delay's phase also moves with the probe and reads as
    about ``2 pi tau / |dphi_df|`` hertz per hertz of recentre (well under 1%
    for typical delays and linewidths). Once typed frames are arriving, the
    cycles of a revision not yet announced are held back; without them every
    recentre reads as a step of the recentre size.

    Parameters
    ----------
    offsets_hz : array-like
        This tone's per-point probe offsets (Hz), in slot order (from
        ``info['modulation']['tones'][i]['offsets_hz']``); >= 2 distinct.
    center_hz : float or None, optional
        The modulation centre at startup; ``None`` adopts the first centre
        registered via :meth:`note_center`.
    average_cycles : int, optional
        Boxcar over the last N per-cycle outputs (default 1 = latest cycle).
    baseline_cycles : int, optional
        Cycles averaged into the reference (default 20). Outputs that need it
        are NaN (the DAC holds) until it is set.
    calibration : ResonatorCalibration or None, optional
        This tone's calibration: the centred basis, and the absolute and
        linewidth outputs.
    output : str, optional
        A ``resonator.QUANTITIES`` resonance quantity or alias (default
        ``'resonance_shift_hz'``, positive = resonance moved up), or a
        per-cycle estimate of ``modulation.CYCLE_FIELDS``
        (``'center_detuning_hz'``, ``'dphi_df'``, ...).
    demod_method : {'fast', 'accurate'}, optional
        ``'fast'`` (default, the tracker's): finite differences of the
        lowest, central and highest offsets; ``'accurate'``: a quadratic fit
        of phase against offset, every point counting.
    gain : float, optional
        Data over calibration RX amplitude (``resonator.resolve_rx_gain``),
        divided out in the centred basis (default 1).
    """

    wants_frames = True

    def __init__(self, offsets_hz, center_hz=None, average_cycles=1,
                 baseline_cycles=20, calibration=None,
                 output='resonance_shift_hz', demod_method='fast', gain=1.0):
        offsets = np.asarray(offsets_hz, dtype=float)
        if len(offsets) < 2 or np.ptp(offsets) == 0:
            raise ValueError('ffm mode needs >= 2 distinct probe offsets '
                             'for this tone (is it modulated?)')
        if demod_method not in ('fast', 'accurate'):
            raise ValueError("demod_method must be 'fast' or 'accurate'")
        self.output = _check_output(output, calibration, cycle=True)
        self.units = resonator.QUANTITIES[self.output][1]
        self.calibration = calibration
        self.hold_unannounced = False     # set once typed frames arrive
        self._kernel = 'fast' if demod_method == 'fast' else 'fit'
        self._gain = float(gain)
        self._num_points = len(offsets)
        self._states = {}                 # revision -> (centre Hz, offsets)
        self._last_state = (None if center_hz is None else float(center_hz),
                            offsets)
        self._basis_key = None            # the state _basis was built for
        self._basis = None
        self._baseline = []               # centre points for the reference
        self._baseline_key = None
        self._baseline_cycles = max(1, int(baseline_cycles))
        self._reference = None            # (centre-point I/Q, centre Hz)
        self.reference_resonance_hz = np.nan
        self._outputs = deque(maxlen=max(1, int(average_cycles)))
        # Current-cycle accumulators (per-point complex sums + counts).
        self._sums = np.zeros(self._num_points, dtype=complex)
        self._counts = np.zeros(self._num_points, dtype=int)
        self._last_point = 0
        self._revision = None
        self.cycles = 0
        self.last_dphi_df = np.nan

    def note_center(self, revision, center_hz=None, offsets_hz=None):
        """Register this tone's centre (and offsets) for a modulation
        ``revision`` (fed from SNAPSHOT / TONE_UPDATE typed frames). ``None``
        keeps the latest registered value, for a revision that moved other
        tones only."""
        center = self._last_state[0] if center_hz is None else float(center_hz)
        offsets = self._last_state[1] if offsets_hz is None \
            else np.asarray(offsets_hz, dtype=float)
        if len(offsets) != self._num_points:
            print(f'WARNING: revision {revision} has {len(offsets)} probe '
                  f'points, not {self._num_points}; restart stream-to-dac '
                  'to follow it')
            return
        state = (center, offsets)
        self._states[int(revision) & 0x7FFF] = state
        self._last_state = state

    def add_frame(self, iq, point, settling, revision):
        """Feed one stream sample (this tone's corrected IQ + flag5 tags)."""
        if point < 1 or point > self._num_points:
            return
        if point < self._last_point:
            self._finalise_cycle()
        self._last_point = point
        self._revision = revision
        if not settling:
            self._sums[point - 1] += iq
            self._counts[point - 1] += 1

    def _finalise_cycle(self):
        sums, counts = self._sums, self._counts
        self._sums = np.zeros(self._num_points, dtype=complex)
        self._counts = np.zeros(self._num_points, dtype=int)
        if np.any(counts == 0):
            return                        # incomplete cycle (drops): skip
        state = self._states.get(self._revision)
        if state is None:
            if self.hold_unannounced:
                return                    # its announcement is still in flight
            state = self._last_state
        center, offsets = state
        if center is None and (self.calibration is not None
                               or self.output not in CYCLE_FIELDS):
            return                        # no probe frequency to work at yet
        z = sums / counts
        if self.calibration is not None:
            key = (center, offsets.tobytes())
            if key != self._basis_key:
                self._basis_key = key
                self._basis = resonator.basis_coefficients(
                    self.calibration, center + offsets, 'centered_rotated',
                    gain=self._gain)
            z = self._basis[0] * z + self._basis[1]
        d = resonator.phase_derivatives(z[:, None], offsets[:, None],
                                        method=self._kernel, per_point=True)
        slope = float(d['dphi_df'][0])
        if not np.isfinite(slope) or slope == 0.0:
            return
        self.cycles += 1
        self.last_dphi_df = slope
        curvature = float(d['d2phi_df2'][0])

        if self.output in CYCLE_FIELDS:
            cycle = {'dphi_df': slope, 'd2phi_df2': curvature,
                     'curvature_over_slope': curvature / slope}
            if self.calibration is not None:
                # The calibration's inversion: right in scale for driven
                # resonators, where the curvature form reads ~2x.
                detuning_hz = float(resonator.mobius_center_detuning(
                    z, offsets, self.calibration))
                cycle['center_detuning_hz'] = detuning_hz
                cycle['center_detuning_linewidths'] = (
                    detuning_hz * self.calibration.Ql / self.calibration.fr)
            self._push(cycle[self.output])
            return

        # The centre point (its dwells averaged) and its local slope/gradient.
        mid_offset = float(d['center_offset_hz'][0])
        mid = offsets == mid_offset
        first = int(np.flatnonzero(mid)[0])
        z_mid = complex(np.mean(z[mid]))
        if self._reference is None:
            key = (center, offsets.tobytes())
            if key != self._baseline_key:   # restart across a recentre
                self._baseline, self._baseline_key = [], key
            self._baseline.append(z_mid)
            if len(self._baseline) >= self._baseline_cycles:
                reference = complex(np.mean(self._baseline))
                self._reference = (reference, center)
                self._baseline = []
                if self.calibration is not None:
                    # Where the resonance (inflection) was at the reference.
                    cal = self.calibration
                    probe_detuning = float(cal.convert_centered_iq(reference)[0])
                    self.reference_resonance_hz = (
                        center + mid_offset - probe_detuning
                        + resonator.operating_point_hz(cal) - cal.fr)
            return
        reference, reference_center = self._reference
        if self.calibration is not None:
            local_slope = float(d['local_dphi_df'][first, 0])
            if not np.isfinite(local_slope):
                local_slope = slope
            shift, dissipation_hz = resonator.phase_shift_and_dissipation(
                z_mid, reference, local_slope)
        else:
            shift, dissipation_hz = resonator.project_iq_tangent_normal(
                reference, d['dz_df'][first, 0], z_mid)
        probe_hz = center + mid_offset
        shift = float(shift) + (center - reference_center)
        self._push(float(resonator.express(
            self.output, shift, float(dissipation_hz) / probe_hz,
            frequency_hz=probe_hz, calibration=self.calibration,
            reference_resonance_hz=self.reference_resonance_hz)))

    def _push(self, value):
        if np.isfinite(value):
            self._outputs.append(value)

    def value(self):
        """Latest output (mean of the last ``average_cycles`` cycles), or
        NaN before the reference is established."""
        if not self._outputs:
            return float('nan')
        return float(np.mean(self._outputs))

    def convert(self, iq):
        """Interface compatibility: frame-consuming converters report the
        latest per-cycle value; the averaged-IQ argument is ignored."""
        return self.value()


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


class LabJackU12Dac(DacBackend):
    """LabJack U12 backend via LabJackPython + the Exodriver (Linux/macOS).

    The U12 predates the LJM library, so it uses a different driver stack from
    :class:`LabJackDac`: the ``LabJackPython`` package's ``u12`` module talking
    to the Exodriver (``liblabjackusb``, built on libusb-1.0). Outputs are the
    two 0-5 V analog outs ``AO0``/``AO1`` (``DAC0``/``DAC1`` accepted as
    aliases); inputs are ``AI0``..``AI7`` (``AIN0``.. accepted too).

    The U12's analog outs are filtered PWM and its USB transactions are
    command-response at roughly ~20 ms each, so keep ``update_rate_hz`` low
    (tens of Hz) -- well below the T-series' few-hundred Hz.

    Parameters
    ----------
    id : int, optional
        Local device id; -1 (default) opens the first U12 found.
    serial_number : int, optional
        Open a specific device by serial number (default: any).
    """

    _MAX_VOLTS = 5.0

    def __init__(self, id=-1, serial_number=None):
        try:
            import u12 as u12_module
        except ImportError as exc:
            raise ImportError(
                "LabJack U12 backend needs the 'LabJackPython' package "
                "(pip install LabJackPython) and the Exodriver (liblabjackusb; "
                "https://labjack.com/support/software/installers/exodriver)"
                ) from exc
        self._u12 = u12_module.U12(id=id, serialNumber=serial_number)
        # eAnalogOut writes both outputs at once, so remember each output's
        # last value and only change the ones a given write() names.
        self._ao = [0.0, 0.0]
        self._u12.eAnalogOut(self._ao[0], self._ao[1])
        serial = getattr(self._u12, 'serialNumber', None)
        print('LabJackU12Dac: opened U12'
              + (f' (serial {serial})' if serial else ''))

    @staticmethod
    def _output_index(name):
        """Map an output channel name to 0/1 (AO0/AO1, DAC0/DAC1 aliased)."""
        key = name.strip().upper()
        for prefix in ('AO', 'DAC'):
            rest = key[len(prefix):]
            if key.startswith(prefix) and rest.isdigit() and int(rest) in (0, 1):
                return int(rest)
        raise ValueError(f"U12 output channel must be AO0/AO1 (got '{name}')")

    @staticmethod
    def _input_index(name):
        """Map an input channel name to 0..7 (AI0.., AIN0.. aliased)."""
        key = name.strip().upper()
        for prefix in ('AIN', 'AI'):
            rest = key[len(prefix):]
            if key.startswith(prefix) and rest.isdigit() and 0 <= int(rest) <= 7:
                return int(rest)
        raise ValueError(f"U12 input channel must be AI0..AI7 (got '{name}')")

    def write(self, values):
        for name, volts in values.items():
            # Clamp to the device's 0-5 V range: a negative value is the u12
            # module's "leave this output unchanged" sentinel, so never emit one.
            self._ao[self._output_index(name)] = min(
                max(float(volts), 0.0), self._MAX_VOLTS)
        self._u12.eAnalogOut(self._ao[0], self._ao[1])

    def read_ain(self, channels):
        return [self._u12.eAnalogIn(self._input_index(name))['voltage']
                for name in channels]

    def close(self):
        if self._u12 is not None:
            try:
                self._u12.close()
            finally:
                self._u12 = None


# Peak of the signed 32-bit sample format the ALSA backend writes.
_INT32_PEAK = 2 ** 31 - 1


class AlsaDac(DacBackend):
    """Sound-card backend via ALSA (optional dependency: ``pyalsaaudio``).

    A sound card is a fast, cheap, multichannel DAC, and this backend makes
    one usable where no USB DAC is available. Two things differ from the
    LabJack backends and both matter:

    **AC coupling.** Line outputs are capacitively coupled, so they cannot
    emit a static level: DC is blocked and everything below the coupling
    corner is high-passed. The stream-to-dac output is a slowly-varying,
    near-DC control voltage, so measure the corner before trusting a slow
    scan -- :func:`alsa_selftest` does exactly that. ``idle_voltage`` is
    physically 0 V here whatever it is set to (the safe-state write is still
    honoured, it just decays away). A DC-coupled interface (Expert Sleepers
    ES-8/ES-9, MOTU) removes the limitation and needs no code change.

    **The card owns the clock.** ALSA wants a continuous sample stream, while
    :meth:`DacBackend.write` is a sparse non-blocking push. ``write``
    therefore only updates a target level; a writer thread emits one period at
    a time, ramping linearly from the previous targets to the current ones,
    and blocks in ``pcm.write`` so the card paces it. Set ``period_size`` to
    about ``rate / update_rate_hz`` and each ramp spans one update interval,
    which keeps the output smooth instead of stepped.

    Linux only -- ALSA is the Linux sound API, so on Windows and macOS the
    guarded import fails and :func:`make_dac_backend` falls back to
    :class:`DummyDac`; the LabJack backends are the portable option there.
    Needs pyalsaaudio >= 0.9 (the keyword ``PCM`` constructor).

    Parameters
    ----------
    device : str, optional
        ALSA PCM name (default 'default'). Prefer an explicit ``hw:`` device
        (e.g. 'hw:CARD=PCH,DEV=0') to bypass PipeWire/PulseAudio volume and
        resampling; :func:`alsa_selftest` lists what is available.
    rate : int, optional
        Sample rate in Hz (default 48000).
    channels : int, optional
        Audio channels to open (default 2; most cards refuse mono).
    period_size : int, optional
        Frames per ALSA period (default 256, i.e. 5.3 ms at 48 kHz).
    full_scale_volts : float, optional
        Output volts at digital full scale (default 1.0). Measure it with
        :func:`alsa_selftest`; it includes the mixer gain, so set the mixer
        once and then leave it alone.
    """

    def __init__(self, device='default', rate=48000, channels=2,
                 period_size=256, full_scale_volts=1.0):
        try:
            import alsaaudio
        except ImportError as exc:
            raise ImportError(
                "ALSA backend needs the 'pyalsaaudio' package (sudo apt "
                "install libasound2-dev; pip install pyalsaaudio)") from exc
        self._alsaaudio = alsaaudio
        self.device = str(device)
        self.rate = int(rate)
        self.nchannels = int(channels)
        self.period_size = int(period_size)
        self.full_scale_volts = float(full_scale_volts)
        self.underruns = 0
        self.write_count = 0
        self.last_values = {}
        self.writer_error = None
        self._pcm = alsaaudio.PCM(
            type=alsaaudio.PCM_PLAYBACK, mode=alsaaudio.PCM_NORMAL,
            device=self.device, channels=self.nchannels, rate=self.rate,
            format=alsaaudio.PCM_FORMAT_S32_LE, periodsize=self.period_size)
        buffer_frames = self.period_size
        try:      # pcm.info() needs a newer pyalsaaudio
            buffer_frames = int(self._pcm.info().get('buffer_size',
                                                     buffer_frames))
        except Exception:
            pass
        print(f"AlsaDac: opened '{self.device}' at {self.rate} Hz, "
              f'{self.nchannels} ch, period {self.period_size} frames'
              f' ({1e3 * self.period_size / self.rate:.1f} ms), buffer '
              f'{buffer_frames} frames ({1e3 * buffer_frames / self.rate:.1f} '
              f'ms), full scale {self.full_scale_volts} V')
        self._targets = np.zeros(self.nchannels)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._writer, name='alsa-dac',
                                        daemon=True)
        self._thread.start()

    @staticmethod
    def _channel_index(name):
        """Map an output channel name to an audio channel index.

        Accepts 'DAC0'/'DAC1' (so LabJack configs port over unchanged),
        'AO0'/'AO1', and 'CH0'..'CHn'.
        """
        key = name.strip().upper()
        for prefix in ('DAC', 'AO', 'CH'):
            rest = key[len(prefix):]
            if key.startswith(prefix) and rest.isdigit():
                return int(rest)
        raise ValueError("ALSA output channel must be DAC0../AO0../CH0.. "
                         f"(got '{name}')")

    def _writer(self):
        """Emit one period at a time, ramping between targets. The blocking
        ``pcm.write`` is what paces this thread -- the card is the clock."""
        scale = _INT32_PEAK / self.full_scale_volts
        ramp = np.linspace(0.0, 1.0, self.period_size, endpoint=False)[:, None]
        last = np.zeros(self.nchannels)
        while not self._stop.is_set():
            with self._lock:
                target = self._targets.copy()
            block = last + (target - last) * ramp
            last = target
            samples = np.clip(block * scale, -_INT32_PEAK, _INT32_PEAK)
            try:
                self._pcm.write(samples.astype('<i4').tobytes())
            except self._alsaaudio.ALSAAudioError:
                # Underrun (or a transient device error): count it and let
                # ALSA re-prepare on the next write instead of spinning hot.
                self.underruns += 1
                time.sleep(self.period_size / self.rate)
            except Exception as exc:
                # The device is gone: say so rather than freezing the output
                # at its last value with no explanation.
                self.writer_error = exc
                print(f'\nWARNING: AlsaDac writer stopped: {exc!r}; the '
                      'analog output is no longer being driven')
                return

    def write(self, values):
        """Set the target voltage of each named channel (no device I/O)."""
        with self._lock:
            for name, volts in values.items():
                index = self._channel_index(name)
                if index >= self.nchannels:
                    raise ValueError(
                        f"channel '{name}' is beyond the {self.nchannels} "
                        'audio channels opened (dac.alsa.channels)')
                self._targets[index] = float(volts)
        self.last_values = dict(values)
        self.write_count += 1

    def read_ain(self, channels):
        """Always zeros: a sound card's line-in is AC-coupled and is not
        wired into the live path (:func:`alsa_selftest` opens capture
        separately for the bench loopback)."""
        return [0.0] * len(channels)

    def close(self):
        if self._pcm is None:
            return
        with self._lock:
            self._targets[:] = 0.0
        # Let the writer ramp to zero and flush a couple of periods so the
        # output does not stop on a step.
        time.sleep(3 * self.period_size / self.rate)
        self._stop.set()
        self._thread.join(timeout=1.0)
        if self.underruns:
            print(f'AlsaDac: {self.underruns} underrun(s)')
        if self._thread.is_alive():
            print('WARNING: AlsaDac writer thread did not stop; leaving the '
                  'device open')
            return
        try:
            self._pcm.close()
        finally:
            self._pcm = None


# Sine-ladder frequencies for the low-frequency response stage of
# alsa_selftest, highest last so it can serve as the 0 dB reference.
_ALSA_LADDER_HZ = (0.2, 0.5, 1.0, 2.0, 5.0, 10.0, 20.0, 50.0, 100.0)


def _alsa_play(pcm, waveform, nchannels, period_size, capture=None):
    """Play a mono waveform (units of digital full scale) on every channel.

    Writes one period at a time, zero-padding the last one. When ``capture``
    is an open non-blocking capture PCM, drains it between periods and returns
    the recorded mono signal (first channel) in units of its own full scale;
    otherwise returns an empty array.
    """
    recorded = []
    for start in range(0, len(waveform), period_size):
        block = waveform[start:start + period_size]
        if len(block) < period_size:
            block = np.concatenate([block, np.zeros(period_size - len(block))])
        frame = np.repeat(np.clip(block, -1.0, 1.0)[:, None] * _INT32_PEAK,
                          nchannels, axis=1)
        pcm.write(frame.astype('<i4').tobytes())
        if capture is not None:
            length, data = capture.read()
            if length > 0:
                recorded.append(np.frombuffer(data, dtype='<i4'))
    if not recorded:
        return np.zeros(0)
    return np.concatenate(recorded)[::nchannels] / _INT32_PEAK


def _lockin_amplitude(signal, frequency, rate):
    """Single-bin DFT amplitude of ``signal`` at ``frequency`` (same units)."""
    if len(signal) < 2:
        return float('nan')
    t = np.arange(len(signal)) / rate
    phasor = np.exp(-2j * np.pi * frequency * t)
    return float(2.0 * np.abs(np.sum(signal * phasor)) / len(signal))


def alsa_selftest(options=None, seconds=20.0, capture_device=None,
                  droop_hz=0.1, droop_cycles=3):
    """Bench characterisation of a sound card used as the analog output.

    Three stages, printed as they run:

    1. **Devices** -- the ALSA cards and playback PCMs available, so the right
       ``dac.alsa.device`` string can be found.
    2. **Full scale** -- a 1 kHz sine at half of digital full scale, held for
       ``seconds``. Measure it with a DMM in AC volts and set
       ``dac.alsa.full_scale_volts = sqrt(2) * V_rms / 0.5``. The mixer gain is
       inside that number, so set the mixer once and then leave it alone.
    3. **Low-frequency response** -- a slow square wave (scope the output: the
       exponential droop time constant ``tau`` gives the coupling corner
       ``f_c = 1 / (2 pi tau)``), then a sine ladder from 0.2 Hz to 100 Hz.
       With ``capture_device`` set the ladder is recorded and each amplitude
       is reported in dB relative to the 100 Hz point.

    A loopback capture passes through an AC-coupled *input* as well, so the
    corner it reports is an upper bound on the output's own. A scope (or a DMM
    on the slow square wave) on the output itself is the ground truth.

    Parameters
    ----------
    options : dict or None, optional
        The config's ``dac.alsa`` block (device, rate, channels, period_size,
        full_scale_volts); missing keys take the :class:`AlsaDac` defaults.
    seconds : float, optional
        Duration of the full-scale calibration tone (default 20).
    capture_device : str or None, optional
        ALSA capture PCM for the optional loopback measurement, e.g.
        'hw:CARD=PCH,DEV=0' with line-out wired to line-in (default None).
    droop_hz, droop_cycles : float, int, optional
        Frequency and cycle count of the square wave (default 0.1 Hz, 3).

    Returns
    -------
    int
        0 on success, 1 if pyalsaaudio is not installed.
    """
    try:
        import alsaaudio
    except ImportError:
        print("selftest: the ALSA backend needs the 'pyalsaaudio' package "
              '(sudo apt install libasound2-dev; pip install pyalsaaudio)')
        return 1

    options = dict(options or {})
    device = options.get('device', 'default')
    rate = int(options.get('rate', 48000))
    nchannels = int(options.get('channels', 2))
    period_size = int(options.get('period_size', 256))

    print('--- ALSA devices ---')
    print(f'cards: {alsaaudio.cards()}')
    for name in alsaaudio.pcms(alsaaudio.PCM_PLAYBACK):
        print(f'  playback: {name}')
    print("Pick a 'hw:' device for dac.alsa.device: it bypasses "
          'PipeWire/PulseAudio volume and resampling.\n')

    pcm = alsaaudio.PCM(
        type=alsaaudio.PCM_PLAYBACK, mode=alsaaudio.PCM_NORMAL, device=device,
        channels=nchannels, rate=rate, format=alsaaudio.PCM_FORMAT_S32_LE,
        periodsize=period_size)
    capture = None
    if capture_device is not None:
        capture = alsaaudio.PCM(
            type=alsaaudio.PCM_CAPTURE, mode=alsaaudio.PCM_NONBLOCK,
            device=capture_device, channels=nchannels, rate=rate,
            format=alsaaudio.PCM_FORMAT_S32_LE, periodsize=period_size)
        print(f"loopback capture open on '{capture_device}'")

    try:
        # -- 2. full-scale calibration tone --------------------------------
        amplitude = 0.5
        print(f'--- Full scale: 1 kHz sine at {amplitude} of digital full '
              f'scale for {seconds:.0f} s ---')
        print('Measure it with a DMM in AC volts across the output, then set')
        print(f'  dac.alsa.full_scale_volts = sqrt(2) * V_rms / {amplitude}')
        print('The mixer gain is inside that number: set the mixer now and '
              'leave it alone.')
        t = np.arange(int(seconds * rate)) / rate
        _alsa_play(pcm, amplitude * np.sin(2 * np.pi * 1000.0 * t), nchannels,
                   period_size)

        # -- 3a. slow square wave: the droop measurement --------------------
        print(f'\n--- Droop: {droop_hz} Hz square wave, {droop_cycles} '
              'cycles ---')
        print('Scope the output. A flat top means DC gets through; an '
              'exponential decay')
        print('with time constant tau means the coupling corner is '
              'f_c = 1 / (2 pi tau).')
        t = np.arange(int(droop_cycles / droop_hz * rate)) / rate
        _alsa_play(pcm, amplitude * np.sign(np.sin(2 * np.pi * droop_hz * t)),
                   nchannels, period_size)

        # -- 3b. sine ladder ------------------------------------------------
        print('\n--- Low-frequency response: sine ladder ---')
        measured = {}
        for frequency in _ALSA_LADDER_HZ:
            duration = max(4.0 / frequency, 1.0)
            print(f'  {frequency:6.2f} Hz for {duration:5.1f} s', end='',
                  flush=True)
            t = np.arange(int(duration * rate)) / rate
            recorded = _alsa_play(
                pcm, amplitude * np.sin(2 * np.pi * frequency * t), nchannels,
                period_size, capture=capture)
            if capture is not None:
                # Drop the first half: the capture starts mid-transient.
                half = len(recorded) // 2
                measured[frequency] = _lockin_amplitude(
                    recorded[half:], frequency, rate)
                print(f'   captured {measured[frequency]:.4g} FS')
            else:
                print()
        if measured:
            reference = measured[_ALSA_LADDER_HZ[-1]]
            print(f'\n  relative to {_ALSA_LADDER_HZ[-1]:.0f} Hz '
                  f'({reference:.4g} FS):')
            for frequency, value in measured.items():
                ratio = value / reference if reference > 0 else float('nan')
                print(f'    {frequency:6.2f} Hz  {20 * np.log10(ratio):+6.2f} '
                      f'dB' + ('   <- -3 dB corner is near here'
                               if 0.6 < ratio < 0.79 else ''))
            print('  NOTE: the capture input is AC-coupled too, so this '
                  "corner is an upper\n        bound on the output's own -- "
                  'scope the output for the real number.')
        else:
            print('\n  No capture device configured: read the amplitudes off '
                  'a scope or a DMM\n  in AC volts, and take the -3 dB point '
                  'as the coupling corner.')
    finally:
        if capture is not None:
            capture.close()
        pcm.close()
    return 0


def make_dac_backend(backend, options=None, print_writes=False):
    """Build a DAC backend by name ('dummy', 'labjack', 'labjack_u12', 'alsa').

    ``options`` is the matching driver-options block from the config
    (``dac.labjack`` or ``dac.alsa``). The hardware backends fall back to
    :class:`DummyDac` with a warning when their driver is not installed, so
    development machines still run the full pipeline; a driver that is present
    but fails to open (e.g. a wrong ALSA device name) raises, as it should.
    """
    driver_classes = {'labjack': LabJackDac, 'labjack_u12': LabJackU12Dac,
                      'alsa': AlsaDac}
    if backend in driver_classes:
        try:
            return driver_classes[backend](**(options or {}))
        except ImportError as exc:
            print(f'WARNING: {exc}; falling back to dummy DAC backend')
            return DummyDac(print_writes=print_writes)
    if backend == 'dummy':
        return DummyDac(print_writes=print_writes)
    raise ValueError(f"unknown DAC backend '{backend}'")


# ---------------------------------------------------------------------------
# Output channels and the application
# ---------------------------------------------------------------------------

class OutputChannel:
    """One DAC output channel: a tone, a converter and an x->volts mapping.

    ``volts = clip(gain * (x - x_offset) + v_offset, v_min, v_max)``.

    With ``highpass_s`` set, ``x_offset`` is replaced by a running baseline:
    an exponential moving average of ``x`` with that time constant, started
    at the first value (so the output starts at ``v_offset``). This is the
    single-pole high-pass of an AC-coupled output, done before the clip, so
    slow resonance drift cannot push the output into the rails. Features
    slower than ``highpass_s`` are removed, as the coupling capacitor would
    do anyway if ``highpass_s`` is at or below its time constant.

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
    highpass_s : float or None, optional
        High-pass time constant in seconds (corner ``1/(2 pi highpass_s)``
        Hz). ``None`` or 0 (default) disables it and uses ``x_offset``.
    """

    def __init__(self, dac_channel, tone_position, converter, gain=1.0,
                 x_offset=0.0, v_offset=0.0, v_min=0.0, v_max=5.0,
                 highpass_s=None):
        self.dac_channel = str(dac_channel)
        self.tone_position = int(tone_position)
        self.converter = converter
        self.gain = float(gain)
        self.x_offset = float(x_offset)
        self.v_offset = float(v_offset)
        self.v_min = float(v_min)
        self.v_max = float(v_max)
        self.highpass_s = float(highpass_s) if highpass_s else None
        self._baseline = None
        self._last_time = None
        self.last_x = np.nan
        self.last_volts = np.nan

    def volts(self, x, now=None):
        """Map converter output ``x`` to a clipped output voltage.

        ``now`` (seconds, default ``time.monotonic()``) times the high-pass.
        """
        if self.highpass_s is None:
            v = self.gain * (x - self.x_offset) + self.v_offset
            return float(np.clip(v, self.v_min, self.v_max))
        now = time.monotonic() if now is None else float(now)
        if self._baseline is None:
            self._baseline = x
        else:
            # Exact single-pole step for the elapsed time, so skipped
            # updates (held NaN cycles, slow loops) do not change the corner.
            alpha = 1.0 - np.exp(-(now - self._last_time) / self.highpass_s)
            self._baseline += alpha * (x - self._baseline)
        self._last_time = now
        v = self.gain * (x - self._baseline) + self.v_offset
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
            'frame_bytes': 8 * self.num_tones + 4 * STREAM_TRAILER_WORDS,
            'tone_updates_subscribed': bool(
                getattr(self.source, 'subscribe_updates', False)),
        }
        with open(self.recording_path + '.json', 'w') as f:
            json.dump(metadata, f, indent=4)

    def _drain_typed_frames(self, updates_log):
        """Write any typed frames the source has decoded to the JSONL log.

        Records match ``ReadoutClient.receive_stream``'s, ``sample_index``
        (data frames before the update) included, so ``parse_stream`` and
        ``modulation.demodulate_stream`` place every revision change.
        """
        typed = getattr(self.source, 'typed_frames', None)
        if not typed:
            return
        while True:
            try:
                item = typed.popleft()
            except IndexError:
                break
            frame_type, payload = item[0], item[1]
            sample_index = item[2] if len(item) > 2 else self.frames_received
            record = {'host_time_unix': time.time(),
                      'sample_index': int(sample_index),
                      'frame_type': frame_type, 'payload': payload}
            if updates_log is not None:
                updates_log.write(json.dumps(record) + '\n')
                updates_log.flush()
            else:
                kind = payload.get('type', f'type{frame_type}')
                print(f'\nstream update frame: {kind} '
                      f'revision={payload.get("revision")} '
                      f'op={payload.get("op")}')
            self._note_revision(payload)

    def _note_revision(self, payload):
        """Register a SNAPSHOT / TONE_UPDATE revision: its readout-correction
        factors with the correction, and each channel tone's centre and
        offsets with the frame-consuming (FFM) converters."""
        kind = payload.get('type')
        revision = payload.get('revision')
        if kind not in ('SNAPSHOT', 'TONE_UPDATE') or revision is None \
                or payload.get('op') == 'tracking_dry_run':
            return
        state = payload.get('modulation_state')
        tones = {}
        if isinstance(state, dict) and state.get('tones'):
            if self.correction is not None:
                self.correction.note_state(revision, state)
            for tone in state['tones']:
                if tone.get('center_hz') is not None:
                    tones[int(tone['index'])] = (tone['center_hz'],
                                                 tone.get('offsets_hz'))
        else:
            # Older servers announce the centres only.
            centers = ((payload.get('modulation') or {}).get('centers_hz')
                       if kind == 'SNAPSHOT' else payload.get('centers_hz'))
            tones = {int(k): (v, None) for k, v in (centers or {}).items()}
        for ch in self.channels:
            converter = ch.converter
            if converter is None or \
                    not getattr(converter, 'wants_frames', False):
                continue
            converter.hold_unannounced = True
            converter.note_center(revision,
                                  *tones.get(ch.tone_position, (None, None)))

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
        elif reason != self._stop_reason:
            print(f'\nalso stopping: {reason}')   # e.g. an output error after EOF
        self._stop.set()

    def _selected_iq(self, frame):
        """Corrected IQ values of the channels' tones for one frame."""
        iq = frame.iq
        if self.correction is not None:
            iq = self.correction.apply(iq, frame.point, frame.revision)
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
                # Drain the queue into the boxcar window. Frame-consuming
                # converters (FFM) get every sample with its flag5 tags
                # instead of the boxcar mean.
                frame_converters = [
                    ch.converter for ch in self.channels
                    if getattr(ch.converter, 'wants_frames', False)]
                while True:
                    try:
                        frame = self._queue.popleft()
                    except IndexError:
                        break
                    selected = self._selected_iq(frame)
                    if frame_converters:
                        for ch, iq in zip(self.channels, selected):
                            if getattr(ch.converter, 'wants_frames', False):
                                ch.converter.add_frame(
                                    iq, frame.point, frame.settling,
                                    frame.revision)
                    window.append(selected)

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
                        if getattr(ch.converter, 'wants_frames', False):
                            # FFM: per-cycle output; NaN until the first
                            # complete cycle + baseline -> hold the DAC.
                            x = ch.converter.value()
                            if not np.isfinite(x):
                                continue
                        else:
                            x = float(np.real_if_close(
                                ch.converter.convert(iq)))
                        ch.last_x = x
                        ch.last_volts = ch.volts(x)
                        values[ch.dac_channel] = ch.last_volts
                    if values:
                        self.dac.write(values)
                        self.updates_written += 1
                    if values and ain_log is not None:
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


def _calibration_for(conversion_config, info, tone_position):
    """``(calibration, record, sweep)`` for one tone; Nones when there is none.

    The configured ``calibration_file`` (``resonator.save_calibrations``, with
    its sweep when saved with one), else the calibration the server holds and
    the stream carries (``info['resonators']``, pushed with
    ``ReadoutClient.set_resonator_calibrations``; left out once the server
    marks it stale).
    """
    path = conversion_config.get('calibration_file')
    if path:
        record, sweep = resonator.load_calibration_record(path)
    else:
        record, sweep = resonator.calibration_from_data({'info': info}), None
    if record is None:
        return None, None, None
    calibration = resonator.calibrations_from_record(
        record, [tone_position]).get(tone_position)
    return calibration, record, sweep


def build_converter(mode, conversion_config, info, tone_position,
                    reference_iq=None):
    """Build a :class:`StreamConverter` from config.

    Parameters
    ----------
    mode : {'magnitude', 'phase', 'df_modelfree', 'df_calibrated', 'ffm'}
        Conversion mode.
    conversion_config : dict
        The config file's ``conversion`` section: ``output`` (a
        ``resonator.QUANTITIES`` name or alias, default
        ``'resonance_shift_hz'``), ``method`` (df_calibrated: a
        ``resonator.CONVERSION_METHODS`` method, default ``'mobius'``),
        ``demod_method`` (ffm: ``'fast'`` or ``'accurate'``),
        ``calibration_file`` and the per-mode keys.
    info : dict
        System info: tone frequencies, the modulation state and, when one was
        pushed, the server's resonator calibration.
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
    output = conversion_config.get('output') or 'resonance_shift_hz'
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
        calibration, record, sweep = _calibration_for(
            conversion_config, info, tone_position)
        method = conversion_config.get('method') or 'mobius'
        sweep_f = sweep_z = None
        if sweep is not None:
            sweep_f, sweep_i, sweep_q = resonator.sweep_trace(sweep, tone_position)
            sweep_z = sweep_i + 1j * sweep_q
        if calibration is None and (sweep_f is None
                                    or method not in ('linearized', 'lookup')):
            raise ValueError(
                f'df_calibrated mode has no calibration for tone '
                f'{tone_position}: set conversion.calibration_file, or push '
                'one to the server (ReadoutClient.set_resonator_calibrations)')
        frequencies = info.get('tones', {}).get('frequencies_hz')
        if frequencies is None:
            raise ValueError('info has no tones.frequencies_hz for the '
                             'calibrated converter tone frequency')
        f_tone = float(np.asarray(frequencies, dtype=float)[tone_position])
        if output == 'frequency':
            print("Note: conversion.output 'frequency' is the resonance shift "
                  '(resonance_shift_hz, positive = resonance up); the old '
                  'probe detuning f_tone - fr is -detuning_hz')
        return QuantityConverter(
            f_tone, output, method, calibration=calibration,
            sweep_f=sweep_f, sweep_z=sweep_z,
            gain=resonator.resolve_rx_gain('auto', record, info))
    if mode == 'ffm':
        state = active_modulation_state(info)
        if not isinstance(state, dict) or not state.get('tones'):
            raise ValueError(
                'ffm mode needs frequency modulation armed: the system info '
                'carries no modulation state (enable_modulation first, or '
                'replay a modulated recording)')
        entry = next((t for t in state['tones']
                      if int(t.get('index', -1)) == tone_position), None)
        if entry is None or entry.get('offsets_hz') is None:
            raise ValueError(f'no modulation state for tone {tone_position}')
        offsets = np.asarray(entry['offsets_hz'], dtype=float)
        if not np.any(offsets != 0.0):
            raise ValueError(f'tone {tone_position} is not modulated '
                             '(all probe offsets are zero)')
        calibration, record, _ = _calibration_for(
            conversion_config, info, tone_position)
        if calibration is None:
            print(f'ffm: no calibration for tone {tone_position}, so raw I/Q '
                  'and the tangent/normal projection; push one '
                  '(set_resonator_calibrations) or set '
                  'conversion.calibration_file for the centred basis')
        converter = FfmConverter(
            offsets,
            center_hz=entry.get('center_hz'),
            average_cycles=conversion_config.get('average_cycles', 1),
            baseline_cycles=conversion_config.get('baseline_cycles', 20),
            calibration=calibration, output=output,
            demod_method=conversion_config.get('demod_method') or 'fast',
            gain=resonator.resolve_rx_gain('auto', record, info))
        # Seed the current revision so the output is anchored even before
        # the first typed frame arrives.
        revision = state.get('applied_revision')
        if revision is not None and entry.get('center_hz') is not None:
            converter.note_center(revision, entry['center_hz'], offsets)
        return converter
    raise ValueError(f"unknown conversion mode '{mode}'")
