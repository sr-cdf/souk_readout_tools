"""
Server-side FFM tone tracking: watch demodulated detunings, issue corrections.

With fast frequency modulation running, this module keeps every modulated
tone at its operating point during long observations: a lean vectorised
estimator turns whole modulation cycles from the stream into per-tone
detunings, a controller filters them, applies a deadband + confirmation
logic, and stages small centre corrections that are committed through the
server's normal ``update_modulation`` machinery (the frame producer stays
the sole owner of modulation hardware writes). Every committed correction is
revision-stamped into the stream and announced in-band as a typed
``TONE_UPDATE`` frame, so post-processing can reconstruct absolute per-tone
response exactly:

    absolute_response(t) = (center[rev(t)] - center[rev0]) + freq_shift_hz(t)

Tracking is permanent readout infrastructure (live frequency-shift serving
and array health monitoring are other plausible consumers); the estimator
(:func:`assemble_cycles` + :func:`estimate_detunings`) is deliberately
separable from the controller and from the server wiring.

Performance contract (see doc/tone_tracking.md): the only work added to the
``stream_data`` per-sample hot path is one O(1) append of the already-built
frame payload to a bounded ring (:meth:`TrackingRuntime.tap`). All parsing,
cycle assembly and estimation run in a consumer task, vectorised across all
tones, in a thread executor.

Everything above the runtime is numpy-only and server-free, so it is unit
testable off the RFSoC.
"""

import asyncio
import json
import time
import traceback
import warnings
from collections import deque

import numpy as np

# Number of int32 trailer words at the end of every stream frame
# (flags 0-5, tt_msb, tt_lsb, cnt, err) -- matches prepare_frame.
STREAM_TRAILER_WORDS = 10

# Typed stream-frame type codes, carried in the top byte of the 4-byte
# length prefix (legacy data frames always have 0x00 there -- frames are
# far below 16 MB). See doc/tone_tracking.md ("Typed stream frames").
FRAME_TYPE_DATA = 0x00
FRAME_TYPE_TONE_UPDATE = 0x01
FRAME_TYPE_SNAPSHOT = 0x02


def pack_typed_frame(frame_type, payload_dict):
    """Frame a JSON payload with a nonzero type in the length-prefix top byte."""
    body = json.dumps(payload_dict).encode()
    if len(body) > 0xFFFFFF:
        raise ValueError('typed frame payload too large')
    import struct
    return struct.pack('>I', (int(frame_type) << 24) | len(body)) + body


# ---------------------------------------------------------------------------
# Frame parsing and cycle assembly (vectorised, no Python per-tone loops)
# ---------------------------------------------------------------------------

def parse_frame_batch(payloads, num_tones):
    """Decode a batch of raw stream payloads into sample arrays.

    Parameters
    ----------
    payloads : list of bytes
        Frame payloads as built by ``prepare_frame`` (including the 4-byte
        length prefix, which is skipped).
    num_tones : int
        Active tones per frame.

    Returns
    -------
    dict
        ``z`` (n_samples, n_tones) complex128; ``point`` (1-based, 0 = not
        modulated), ``settling``, ``revision``, ``cnt``, ``tt`` -- all
        (n_samples,) arrays. Frames whose size does not match are skipped.
    """
    expected = 4 + 4 * (2 * num_tones + STREAM_TRAILER_WORDS)
    good = [p for p in payloads if len(p) == expected]
    n = len(good)
    if n == 0:
        return None
    words = np.frombuffer(b''.join(good), dtype='<i4').reshape(n, -1)
    words = words[:, 1:]                       # drop the 4-byte length prefix
    iq = words[:, :2 * num_tones]
    z = iq[:, 0::2].astype(np.float64) + 1j * iq[:, 1::2].astype(np.float64)
    tail = words[:, -STREAM_TRAILER_WORDS:].view('<u4')
    f5 = tail[:, 5].astype(np.uint32)
    return {
        'z': z,
        'point': (f5 & 0xFFFF).astype(int),
        'settling': ((f5 >> 16) & 0x1).astype(bool),
        'revision': ((f5 >> 17) & 0x7FFF).astype(int),
        'cnt': tail[:, 8].astype(np.int64),
        'tt': (tail[:, 6].astype(np.uint64) << np.uint64(32))
              + tail[:, 7].astype(np.uint64),
    }


def assemble_cycles(batch, num_points, carry=None):
    """Group parsed samples into complete modulation cycles.

    A new cycle starts whenever the point tag decreases (the 1..N sequence
    wraps). Settling-flagged samples are excluded from the per-point means.
    Incomplete leading/trailing cycles are carried between calls via
    ``carry`` so no data is lost across batches.

    Parameters
    ----------
    batch : dict
        Output of :func:`parse_frame_batch`.
    num_points : int
        Number of modulation points N.
    carry : dict or None
        Leftover samples from the previous call (returned by this function).

    Returns
    -------
    (cycles, carry) : (dict or None, dict)
        ``cycles``: ``z`` (n_cycles, N, n_tones) per-point means (NaN where
        a point had no usable samples), ``revision`` (n_cycles,) (the last
        revision seen in the cycle), ``tt`` (n_cycles,) (last sample's tt).
        ``None`` when no complete cycle is available yet.
    """
    keys = ('z', 'point', 'settling', 'revision', 'tt')
    if carry:
        batch = {k: np.concatenate([carry[k], batch[k]]) for k in keys}
    else:
        batch = {k: batch[k] for k in keys}

    point = batch['point']
    usable = point >= 1
    if not np.any(usable):
        return None, {k: batch[k][:0] for k in keys}
    # Work only on modulated samples; unmodulated (point 0) samples are rare
    # here (the tap only runs in the modulated branch) and are dropped.
    batch = {k: batch[k][usable] for k in keys}
    point = batch['point']

    # Cycle index: increments where the point sequence wraps (point decreases).
    wraps = np.zeros(len(point), dtype=int)
    wraps[1:] = (np.diff(point) < 0).astype(int)
    cycle = np.cumsum(wraps)
    n_cycles_total = int(cycle[-1]) + 1

    # The last cycle is (possibly) incomplete: carry it to the next batch.
    last_start = int(np.searchsorted(cycle, cycle[-1]))
    carry_out = {k: batch[k][last_start:] for k in keys}
    if n_cycles_total < 2:
        return None, carry_out
    n_cycles = n_cycles_total - 1
    sel = slice(0, last_start)
    z = batch['z'][sel]
    point_sel = point[sel]
    cycle_sel = cycle[sel]
    settling = batch['settling'][sel]

    # Per-(cycle, point) means over non-settling samples, vectorised with
    # bincount over a flat (cycle * N + point-1) index.
    n_tones = z.shape[1]
    flat = cycle_sel * num_points + (point_sel - 1)
    keep = ~settling & (point_sel <= num_points)
    flat_k = flat[keep]
    minlength = n_cycles * num_points
    counts = np.bincount(flat_k, minlength=minlength).astype(float)
    zk = z[keep]
    sums_r = np.zeros((minlength, n_tones))
    sums_i = np.zeros((minlength, n_tones))
    np.add.at(sums_r, flat_k, zk.real)
    np.add.at(sums_i, flat_k, zk.imag)
    with np.errstate(invalid='ignore', divide='ignore'):
        mean = (sums_r + 1j * sums_i) / counts[:, None]
    mean[counts == 0] = np.nan
    z_cycles = mean.reshape(n_cycles, num_points, n_tones)

    # Cycle-level metadata: last sample of each cycle.
    last_idx = np.searchsorted(cycle_sel, np.arange(1, n_cycles + 1)) - 1
    cycles = {
        'z': z_cycles,
        'revision': batch['revision'][sel][last_idx],
        'tt': batch['tt'][sel][last_idx],
    }
    return cycles, carry_out


def estimate_detunings(z, offsets, linewidth_hz):
    """Model-free per-cycle detuning estimate, vectorised across all tones.

    Implements the same maths as :func:`souk_readout_tools.modulation.
    demodulate`'s model-free ``method='fast'`` path (finite-difference phase
    slope and curvature; detuning from the curvature/slope ratio), as one
    set of whole-array operations -- no Python loop over tones. Numerical
    agreement with ``demodulate`` is asserted by a unit test.

    Parameters
    ----------
    z : numpy.ndarray
        ``(n_cycles, N, n_tones)`` complex per-point cycle means (from
        :func:`assemble_cycles`).
    offsets : numpy.ndarray
        ``(N, n_tones)`` per-point probe offsets in Hz.
    linewidth_hz : numpy.ndarray
        ``(n_tones,)`` per-tone resonator linewidths in Hz (NaN/0 disables a
        tone's estimate).

    Returns
    -------
    dict
        ``detuning_linewidths`` and ``detuning_hz``: ``(n_cycles, n_tones)``
        (NaN where not estimable); ``dphi_df`` the phase slope (rad/Hz).
    """
    n_cycles, N, n_tones = z.shape
    if N < 3:
        raise ValueError('tracking needs >= 3 modulation points for the '
                         'curvature-based detuning estimate')
    offsets = np.asarray(offsets, dtype=float)
    lw = np.asarray(linewidth_hz, dtype=float)

    phi = np.unwrap(np.angle(z), axis=1)             # (n_cycles, N, n_tones)
    span = offsets[-1] - offsets[0]                  # (n_tones,)
    half = span / 2.0
    mid = N // 2
    with np.errstate(divide='ignore', invalid='ignore'):
        slope = (phi[:, -1, :] - phi[:, 0, :]) / span
        d2 = (phi[:, -1, :] - 2.0 * phi[:, mid, :] + phi[:, 0, :]) / half ** 2
        detuning_linewidths = -(d2 / slope * lw) / 8.0
    bad = ~np.isfinite(detuning_linewidths)
    detuning_linewidths = np.where(bad, np.nan, detuning_linewidths)
    return {
        'detuning_linewidths': detuning_linewidths,
        'detuning_hz': detuning_linewidths * lw,
        'dphi_df': slope,
    }


# ---------------------------------------------------------------------------
# Averaging filter
# ---------------------------------------------------------------------------

class DetuningFilter:
    """Boxcar or EWMA averaging of per-cycle detunings, NaN-aware.

    The filter output is what all threshold logic sees. Frequency response
    (needed to compute the effect on the science band; see the doc page):
    boxcar of W cycles -> sinc with first null at cycle_rate / W; EWMA with
    time constant tau -> single real pole at 1 / (2*pi*tau).

    Parameters
    ----------
    mode : {'boxcar', 'ewma'}
    n_tones : int
    window : int
        Boxcar length in cycles (mode='boxcar').
    tau_s : float
        EWMA time constant in seconds (mode='ewma').
    cycle_period_s : float
        Modulation cycle period (converts tau to the per-cycle alpha).
    """

    def __init__(self, mode, n_tones, window=10, tau_s=1.0,
                 cycle_period_s=0.01):
        if mode not in ('boxcar', 'ewma'):
            raise ValueError("filter mode must be 'boxcar' or 'ewma'")
        self.mode = mode
        self.n_tones = int(n_tones)
        self.window = max(1, int(window))
        self.tau_s = float(tau_s)
        self.cycle_period_s = float(cycle_period_s)
        self._alpha = 1.0 - np.exp(-self.cycle_period_s / self.tau_s) \
            if mode == 'ewma' else None
        self._boxcar = deque(maxlen=self.window)
        self._ewma = np.full(self.n_tones, np.nan)
        self.value = np.full(self.n_tones, np.nan)
        self._cycles_seen = 0

    def reset(self):
        """Clear all state (used on external revision changes)."""
        self._boxcar.clear()
        self._ewma = np.full(self.n_tones, np.nan)
        self.value = np.full(self.n_tones, np.nan)
        self._cycles_seen = 0

    @property
    def settled(self):
        """True once the filter has seen a full window of cycles."""
        if self.mode == 'boxcar':
            return len(self._boxcar) >= self.window
        needed = max(1, int(round(self.tau_s / self.cycle_period_s)))
        return self._cycles_seen >= needed

    def update(self, detunings):
        """Feed ``(n_cycles, n_tones)`` detunings; return the filtered row."""
        detunings = np.atleast_2d(detunings)
        if self.mode == 'boxcar':
            for row in detunings:
                self._boxcar.append(row)
            with warnings.catch_warnings():
                # An all-NaN tone column (untracked tone) is expected.
                warnings.simplefilter('ignore', RuntimeWarning)
                self.value = np.nanmean(np.asarray(self._boxcar), axis=0)
        else:
            a = self._alpha
            for row in detunings:
                seed = np.isnan(self._ewma) & ~np.isnan(row)
                self._ewma[seed] = row[seed]
                ok = ~np.isnan(row) & ~np.isnan(self._ewma)
                self._ewma[ok] += a * (row[ok] - self._ewma[ok])
            self.value = self._ewma.copy()
        self._cycles_seen += len(detunings)
        return self.value


# ---------------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------------

# Update classes: 'lo' = new centre stays within armed-bin coverage, only the
# tone LO/mixer words change (seamless in-place update); 'bin' = the shift
# needs an FFT-bin / channel-map change (the needs_recenter condition) --
# slower, briefly disruptive, recenter-style re-arm.
UPDATE_CLASSES = ('lo', 'bin')


class TrackingController:
    """Decision logic for tone tracking. Pure state machine, no I/O.

    Consumes filtered detunings, applies the deadband + K-consecutive-window
    confirmation, computes clamped corrections, classifies each as ``lo`` or
    ``bin`` and accumulates them into per-class staged sets. The server
    runtime decides *when* to commit a staged set (per-class policies) and
    performs the actual commit through the normal update path.

    Parameters (all runtime-configurable via ``enable_tracking``)
    ----------
    mod_indices : list of int
        Tones being modulated (user order).
    linewidth_hz : array-like
        Per-tone linewidths, full ``(n_tones,)`` (NaN for untracked tones).
    threshold_linewidths : float
        Deadband on the *filtered* detuning (default 0.3 -- deliberately
        wider than ``demodulate``'s 0.1 ``needs_update``).
    confirm_windows : int
        Consecutive decision windows over threshold before staging (default 3).
    gain : float
        Correction gain: ``new_center = center - gain * detuning_hz``.
    max_step_linewidths : float
        Per-decision clamp on the correction step (default 0.5).
    enable_mask : array-like of int or None
        Tones allowed to receive corrections (None = all modulated tones).
    """

    def __init__(self, n_tones, mod_indices, linewidth_hz,
                 threshold_linewidths=0.3, confirm_windows=3, gain=1.0,
                 max_step_linewidths=0.5, enable_mask=None):
        self.n_tones = int(n_tones)
        self.mod_indices = [int(i) for i in mod_indices]
        self.linewidth_hz = np.asarray(linewidth_hz, dtype=float)
        if len(self.linewidth_hz) != self.n_tones:
            raise ValueError('linewidth_hz must have one entry per tone')
        self.threshold_linewidths = float(threshold_linewidths)
        self.confirm_windows = max(1, int(confirm_windows))
        self.gain = float(gain)
        self.max_step_linewidths = float(max_step_linewidths)
        mask = np.zeros(self.n_tones, dtype=bool)
        mask[self.mod_indices] = True
        if enable_mask is not None:
            allowed = np.zeros(self.n_tones, dtype=bool)
            allowed[[int(i) for i in enable_mask]] = True
            mask &= allowed
        self.enable_mask = mask

        self.over_count = np.zeros(self.n_tones, dtype=int)
        # Staged corrections per class: {tone_index: new_center_hz}. One
        # commit takes the whole set of a class -> one revision.
        self.staged = {'lo': {}, 'bin': {}}
        self.staged_detunings = {}          # tone -> filtered lw at staging
        self.corrections_staged = {'lo': 0, 'bin': 0}
        self.corrections_suppressed = {'lo': 0, 'bin': 0}
        self.last_decision_ts = None

    def reset_confirmation(self):
        """Restart the confirmation counters and drop staged corrections
        (used when an external command invalidates the decisions)."""
        self.over_count[:] = 0
        self.staged = {'lo': {}, 'bin': {}}
        self.staged_detunings = {}

    def decide(self, filtered_lw, centers_hz, lo_headroom_hz):
        """Run one decision window on the filtered detunings.

        Parameters
        ----------
        filtered_lw : numpy.ndarray
            ``(n_tones,)`` filtered detuning in linewidths.
        centers_hz : numpy.ndarray
            Current per-tone centres (Hz) the detunings were measured
            against.
        lo_headroom_hz : numpy.ndarray
            ``(n_tones,)`` per-tone |centre shift| beyond which the update
            needs a bin/map change (classification only; the commit path
            re-verifies with the prepared bundle's ``needs_recenter``).

        Returns
        -------
        list of dict
            Newly staged corrections
            (``{'tone', 'class', 'new_center_hz', 'detuning_linewidths'}``).
        """
        self.last_decision_ts = time.time()
        over = (np.abs(filtered_lw) > self.threshold_linewidths) \
            & self.enable_mask & np.isfinite(filtered_lw)
        self.over_count = np.where(over, self.over_count + 1, 0)
        ready = self.over_count >= self.confirm_windows

        staged_now = []
        for tone in np.flatnonzero(ready):
            tone = int(tone)
            lw = self.linewidth_hz[tone]
            detuning_hz = filtered_lw[tone] * lw
            step = self.gain * detuning_hz
            clamp = self.max_step_linewidths * lw
            step = float(np.clip(step, -clamp, clamp))
            base = self.staged['lo'].get(
                tone, self.staged['bin'].get(tone, centers_hz[tone]))
            new_center = float(base - step)
            total_shift = abs(new_center - centers_hz[tone])
            klass = 'lo' if total_shift <= lo_headroom_hz[tone] else 'bin'
            # A tone that grows beyond lo headroom moves to the bin set.
            self.staged['lo'].pop(tone, None)
            self.staged['bin'].pop(tone, None)
            self.staged[klass][tone] = new_center
            self.staged_detunings[tone] = float(filtered_lw[tone])
            self.corrections_staged[klass] += 1
            self.over_count[tone] = 0        # restart confirmation post-stage
            staged_now.append({'tone': tone, 'class': klass,
                               'new_center_hz': new_center,
                               'detuning_linewidths': float(filtered_lw[tone])})
        return staged_now

    def take_staged(self, classes=('lo',), tones=None):
        """Remove and return staged corrections for the given classes.

        ``tones`` optionally restricts to a tone subset. Returns
        ``{class: {tone: new_center_hz}}`` (only non-empty classes).
        """
        taken = {}
        for klass in classes:
            if klass not in self.staged:
                raise ValueError(f"unknown update class '{klass}'")
            source = self.staged[klass]
            if tones is None:
                picked = dict(source)
                source.clear()
            else:
                picked = {t: source.pop(t) for t in list(tones)
                          if t in source}
            if picked:
                taken[klass] = picked
        return taken

    def status(self):
        """Summarise controller state for ``get_info('tracking')``."""
        return {
            'threshold_linewidths': self.threshold_linewidths,
            'confirm_windows': self.confirm_windows,
            'gain': self.gain,
            'max_step_linewidths': self.max_step_linewidths,
            'enabled_tones': np.flatnonzero(self.enable_mask).tolist(),
            'staged': {k: {str(t): v for t, v in s.items()}
                       for k, s in self.staged.items()},
            'staged_counts': {k: len(s) for k, s in self.staged.items()},
            'corrections_staged': dict(self.corrections_staged),
            'corrections_suppressed': dict(self.corrections_suppressed),
            'last_decision_ts': self.last_decision_ts,
        }


# ---------------------------------------------------------------------------
# Server runtime
# ---------------------------------------------------------------------------

class TrackingRuntime:
    """Owns the ring buffer, consumer task and commit plumbing on the server.

    Construction does not touch the server's streaming behaviour; the frame
    producer only ever calls :meth:`tap` (one deque append). The consumer
    task drains the ring, runs parsing/estimation in the thread executor and
    drives the controller. Commits go through the server's normal ownership
    discipline: epoch, ``to_thread(_prepare_modulation, ...)``, epoch check,
    ``_pending_modulation`` -- never direct hardware writes.

    Parameters
    ----------
    server : ReadoutServer
    params : dict
        Resolved ``enable_tracking`` parameters (see
        ``ReadoutServer._tracking_defaults``).
    """

    def __init__(self, server, params):
        self.server = server
        self.params = params
        # Engine: whichever modulation engine is armed (they are mutually
        # exclusive). 'fw' slots and 'sw' points tag frames identically, so
        # the estimator is shared; only the commit path differs.
        if server.e_fw_modulation_enabled.is_set():
            self.engine = 'fw'
        elif server.modulation_cfg is not None:
            self.engine = 'sw'
        elif server.fw_modulation_cfg is not None:
            self.engine = 'fw'
        else:
            raise ValueError('modulation must be armed (enable_modulation, '
                             'either engine) before enable_tracking')
        cfg = self._mod_cfg()
        state = self._mod_state()
        if cfg is None or state is None:
            raise ValueError('modulation must be armed (enable_modulation) '
                             'before enable_tracking')
        if self.engine == 'fw' and cfg.get('mode') != 'auto':
            raise ValueError("fw-engine tracking needs mode='auto' (manual "
                             'slot selection never completes cycles)')
        self.num_points = int(state['num_points'])
        if self.num_points < 3:
            raise ValueError('tracking needs >= 3 modulation points (got '
                             f'{self.num_points})')
        self.n_tones = len(np.asarray(cfg['center'], dtype=float))
        self.mod_indices = [int(i) for i in cfg['mod_indices']]

        # Per-tone linewidths: enable_tracking argument wins, else the ones
        # optionally supplied to enable_modulation (params_from_sweep's
        # linewidth_hz), else error -- tracking requires them.
        linewidth = params.get('linewidth_hz')
        if linewidth is None:
            linewidth = cfg.get('linewidth_hz')
        if linewidth is None:
            raise ValueError(
                'tracking requires per-tone linewidth_hz: pass it to '
                'enable_tracking, or to enable_modulation (params_from_sweep '
                "supplies it as 'linewidth_hz')")
        self.linewidth_hz = self._expand_per_tone(linewidth)

        # Full (N, n_tones) offsets for the estimator.
        self.offsets = self._full_offsets(cfg)

        sample_rate = float(state.get('sample_rate_hz') or 0.0)
        cycle_rate = float(state.get('cycle_rate_hz') or 0.0)
        cycle_period = 1.0 / cycle_rate if cycle_rate > 0 else 0.01

        self.controller = TrackingController(
            self.n_tones, self.mod_indices, self.linewidth_hz,
            threshold_linewidths=params['threshold_linewidths'],
            confirm_windows=params['confirm_windows'],
            gain=params['gain'],
            max_step_linewidths=params['max_step_linewidths'],
            enable_mask=params.get('enable_mask'))
        self.filter = DetuningFilter(
            params['filter'], self.n_tones,
            window=params['filter_window'],
            tau_s=params['filter_tau_s'],
            cycle_period_s=cycle_period)

        # Ring buffer: the producer appends raw frame payloads; deque with
        # maxlen gives O(1) drop-oldest. Depth defaults to ~2 s of samples.
        depth = int(params['ring_depth'] or max(1024, int(2 * sample_rate)))
        self.ring = deque(maxlen=depth)
        self._carry = None

        self.enabled = True
        self.dry_run = bool(params['dry_run'])
        self.held = False
        self.hold_reason = None
        self._task = None
        self._stop = False

        # Commit bookkeeping.
        self.last_commit_ts = {'lo': None, 'bin': None}
        self.commits = {'lo': 0, 'bin': 0}
        self.commit_errors = 0
        self.decision_revision = None      # revision the current batch saw
        self.backoffs = 0
        self.cycles_seen = 0
        self.last_filtered_lw = np.full(self.n_tones, np.nan)
        self.last_estimate_ts = None

        # Per-tone running health statistics (exponentially weighted over
        # cycles, ~20-cycle memory). Written by process_batch and read by
        # tone_health()/_build_snapshot -- all in the consumer's off-loop
        # to_thread, so the sole writer is also the sole reader and there is
        # no cross-thread race. The model-free detuning estimate COMPRESSES
        # at large shifts (~x/2/(1+x^2), saturating near 0.25 linewidths),
        # so a lost resonance is detected by the phase slope collapsing
        # toward zero and by invalid (NaN) cycles -- not by big readings.
        self._stats_alpha = 0.05
        self.stat_detuning_var = np.full(self.n_tones, np.nan)  # lw^2
        self.stat_abs_slope = np.full(self.n_tones, np.nan)     # |dphi_df|
        self.stat_invalid_frac = np.full(self.n_tones, np.nan)
        self.slope_baseline = None   # |dphi_df| captured once settled

        # Cached get_info('tracking') snapshot, rebuilt off-loop by the
        # consumer each poll (see _build_snapshot / status). Primed here so a
        # health poll before the first consumer iteration still returns a
        # well-formed payload.
        self._snapshot = self._build_snapshot()

    # -- setup helpers -------------------------------------------------------

    def _mod_cfg(self):
        """The active engine's resolved modulation cfg."""
        return (self.server.fw_modulation_cfg if self.engine == 'fw'
                else self.server.modulation_cfg)

    def _mod_state(self):
        """The active engine's modulation state (get_info payload)."""
        return (self.server.fw_modulation_state if self.engine == 'fw'
                else self.server.modulation_state)

    def _mod_params(self):
        """The active engine's prepared bundle (carries 'armed')."""
        return (self.server.fw_modulation_params if self.engine == 'fw'
                else self.server.modulation_params)

    def _expand_per_tone(self, values):
        """Expand a per-modulated-tone or per-tone array to (n_tones,)."""
        values = np.asarray(values, dtype=float)
        full = np.full(self.n_tones, np.nan)
        if values.size == self.n_tones:
            full[:] = values
        elif values.size == len(self.mod_indices):
            full[self.mod_indices] = values
        elif values.size == 1:
            full[self.mod_indices] = float(values)
        else:
            raise ValueError(
                f'linewidth_hz has {values.size} entries; expected '
                f'{len(self.mod_indices)} (modulated tones) or '
                f'{self.n_tones} (all tones)')
        return full

    def _full_offsets(self, cfg):
        """(N, n_tones) probe offsets from the resolved modulation cfg.

        The per-point/per-slot offsets (``offsets`` for sw cfgs,
        ``slot_offsets`` for fw cfgs) are (N, 1) (broadcast) or
        (N, len(mod_indices)); either broadcasts onto the modulated columns.
        """
        key = 'slot_offsets' if self.engine == 'fw' else 'offsets'
        offsets = np.atleast_2d(np.asarray(cfg[key], dtype=float))
        full = np.zeros((offsets.shape[0], self.n_tones))
        full[:, self.mod_indices] = offsets
        return full

    # Conservative per-point drift bound (in channels) from an occupancy
    # class, for states that do not report numeric drift_bins (fw engine):
    # 'nearest' means < 0.5, 'second' < 1.0, 'beyond' = out of coverage.
    _OCCUPANCY_DRIFT_BOUND = {'nearest': 0.5, 'second': 1.0, 'beyond': np.inf}

    def _lo_headroom_hz(self):
        """Per-tone |centre shift| that stays within armed-bin coverage.

        Estimated from the reported per-(point, tone) ``drift_bins`` (signed
        drift from the armed bin centre in channels; |drift| > 1 is the
        ``needs_recenter`` / occupancy-'beyond' condition) and the armed TX
        bin width. States without numeric drifts (fw engine) fall back to a
        conservative bound from the occupancy class. Classification only --
        the commit path re-verifies with the freshly prepared bundle's
        ``needs_recenter`` and reclassifies.
        """
        headroom = np.full(self.n_tones, np.inf)
        params = self._mod_params()
        state = self._mod_state()
        if params is None or state is None:
            return headroom
        bin_width = float(params.get('armed', {}).get(
            'tx_bin_width_hz', 0.0) or 0.0)
        if bin_width <= 0:
            return headroom
        tones = state.get('tones') or []
        try:
            drift = np.array([tone['drift_bins'] for tone in tones],
                             dtype=float).T          # (n_points, n_tones)
            max_drift = np.nanmax(np.abs(drift), axis=0)
        except (KeyError, TypeError, ValueError):
            try:
                max_drift = np.array([
                    max(self._OCCUPANCY_DRIFT_BOUND.get(str(o), np.inf)
                        for o in tone['occupancy'])
                    for tone in tones], dtype=float)
            except (KeyError, TypeError, ValueError):
                return headroom
        if len(max_drift) != self.n_tones:
            return headroom
        headroom = np.maximum(0.0, (1.0 - max_drift)) * bin_width
        return headroom

    # -- hot path -------------------------------------------------------------

    def tap(self, payload):
        """Producer-side hook: O(1) append of one frame payload.

        Called from ``stream_data``'s modulated branch for every emitted
        frame while tracking is enabled. ``payload`` is the already-built
        frame bytes (length prefix included), so no copy is made here.
        """
        self.ring.append(payload)

    def drain(self):
        """Consumer-side: remove and return all buffered payloads."""
        payloads = []
        while True:
            try:
                payloads.append(self.ring.popleft())
            except IndexError:
                return payloads

    # -- consumer ------------------------------------------------------------

    @staticmethod
    def _ew_update(target, value, alpha):
        """In-place EW update of ``target`` with ``value`` (NaN-aware)."""
        seed = np.isnan(target) & np.isfinite(value)
        target[seed] = value[seed]
        ok = np.isfinite(target) & np.isfinite(value)
        target[ok] += alpha * (value[ok] - target[ok])

    def _update_stats(self, estimate):
        """Fold one batch's per-cycle estimates into the running health
        statistics (see ``__init__``)."""
        detunings = estimate['detuning_linewidths']
        n_cycles = detunings.shape[0]
        valid = np.isfinite(detunings)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            batch_var = np.nanvar(detunings, axis=0)
            batch_slope = np.nanmean(np.abs(estimate['dphi_df']), axis=0)
        batch_invalid = 1.0 - valid.mean(axis=0)
        # Effective alpha for a batch of n cycles at the per-cycle rate.
        alpha = 1.0 - (1.0 - self._stats_alpha) ** n_cycles
        self._ew_update(self.stat_detuning_var, batch_var, alpha)
        self._ew_update(self.stat_abs_slope, batch_slope, alpha)
        self._ew_update(self.stat_invalid_frac, batch_invalid, alpha)
        if self.slope_baseline is None and self.filter.settled:
            # On-resonance responsivity reference, captured once the
            # startup transient has averaged out. A later slope collapse
            # relative to this is the lost-resonance signature.
            self.slope_baseline = self.stat_abs_slope.copy()

    def process_batch(self, payloads):
        """Parse + assemble + estimate + filter one drained batch.

        Runs in the thread executor (numpy releases the GIL for the heavy
        parts). Returns ``None`` when no complete cycle was available, else
        a dict with the filtered detunings and batch metadata.
        """
        batch = parse_frame_batch(payloads, self.n_tones)
        if batch is None:
            return None
        cycles, self._carry = assemble_cycles(
            batch, self.num_points, self._carry)
        if cycles is None:
            return None
        revisions = cycles['revision']
        estimate = estimate_detunings(
            cycles['z'], self.offsets, self.linewidth_hz)
        filtered = self.filter.update(estimate['detuning_linewidths'])
        self._update_stats(estimate)
        self.cycles_seen += cycles['z'].shape[0]
        self.last_filtered_lw = filtered
        self.last_estimate_ts = time.time()
        return {
            'n_cycles': cycles['z'].shape[0],
            'revision_first': int(revisions[0]),
            'revision_last': int(revisions[-1]),
            'filtered_linewidths': filtered,
        }

    def _consume_batch(self, payloads, centers):
        """Off-loop consumer step: estimate + decision-gate + decide.

        Runs entirely in the ``to_thread`` executor -- the whole per-tone
        pipeline (parse, cycle assembly, estimation, filtering, running
        stats and the controller's deadband/confirmation ``decide``) is
        kept off the event loop so the stream loop is never charged for it.
        Performs **no** hardware writes and never touches
        ``_pending_modulation``; it only mutates this runtime's own
        controller/filter/stat state (the consumer is their sole writer).
        Returns ``True`` when a decision was taken, so the on-loop caller
        runs the commit -- which owns the modulation hardware writes.

        ``centers`` is snapshotted on the loop before the hop so this reads
        no live server modulation state beyond the benign, commit-revalidated
        headroom pre-check.
        """
        result = self.process_batch(payloads)
        if result is None:
            return False
        applied = self._applied_revision()
        if result['revision_last'] != applied:
            # An external command landed inside this batch; its cycles mix
            # configs. Discard and restart confirmation.
            self.backoffs += 1
            self.filter.reset()
            self.controller.reset_confirmation()
            return False
        if self.decision_revision != applied:
            # First batch against a new revision: (re)base decisions.
            self.decision_revision = applied
            self.controller.reset_confirmation()
            return False
        if not self.filter.settled:
            return False
        self.controller.decide(result['filtered_linewidths'],
                               centers, self._lo_headroom_hz())
        return True

    async def run(self):
        """Consumer task: poll the ring; estimate/decide off-loop; commit.

        The heavy per-tone work (``_consume_batch``) and the health-snapshot
        rebuild both run in the ``to_thread`` executor; only the commit --
        which must own the modulation hardware writes -- runs on the event
        loop. So neither tracking estimation nor health reporting adds
        per-tone work to the stream loop.
        """
        poll = float(self.params['poll_interval_s'])
        server = self.server
        while not self._stop:
            try:
                await asyncio.sleep(poll)
                if not self.enabled:
                    continue
                if self._interlocked():
                    # Drop buffered frames while held so stale data never
                    # feeds a decision, and restart confirmation.
                    self.drain()
                    self._carry = None
                    self.controller.reset_confirmation()
                else:
                    payloads = self.drain()
                    if payloads:
                        centers = np.asarray(self._mod_cfg()['center'],
                                             dtype=float)
                        decided = await server.to_thread(
                            self._consume_batch, payloads, centers)
                        if decided:
                            await self._auto_commit()
                # Refresh the cached health snapshot off-loop (after any
                # commit, so its staged counts are current). Keeps held /
                # liveness fields fresh even on idle/held polls.
                self._snapshot = await server.to_thread(self._build_snapshot)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f'Error in tracking loop: {e}')
                print(traceback.format_exc())
        self._task = None

    def _applied_revision(self):
        state = self._mod_state()
        return int(state.get('applied_revision', 0)) if state else 0

    def _interlocked(self):
        """Hold automatically during sweeps/retunes and while not streaming."""
        server = self.server
        if self.held:
            return True
        reason = None
        modulating = (server.e_fw_modulation_enabled.is_set()
                      if self.engine == 'fw'
                      else server.e_modulation_enabled.is_set())
        if server.sweep_state.get('state') == 'running':
            reason = 'sweep/retune in progress'
        elif not server.e_stream_enabled.is_set():
            reason = 'streaming disabled'
        elif not modulating:
            reason = 'modulation disabled'
        self.hold_reason = reason
        return reason is not None

    # -- committing ----------------------------------------------------------

    def _policy_due(self, klass):
        """Evaluate the per-class auto-commit policies (first to fire wins)."""
        p = self.params
        staged = self.controller.staged[klass]
        if not staged:
            return False
        if not p[f'{klass}_auto_commit']:
            return False
        now = time.time()
        last = self.last_commit_ts[klass]
        min_interval = p[f'{klass}_min_commit_interval_s']
        if last is not None and now - last < min_interval:
            return False
        # threshold_count: >= M tones staged in this class.
        m = p['commit_threshold_count']
        if m is not None and len(staged) >= int(m):
            return True
        # interval: commit every T seconds if anything is staged.
        t_interval = p['commit_interval_s']
        if t_interval is not None and (
                last is None or now - last >= float(t_interval)):
            return True
        # Default policy when neither knob is set: commit as soon as the
        # min-commit-interval allows.
        return m is None and t_interval is None

    async def _auto_commit(self):
        for klass in UPDATE_CLASSES:
            if self._policy_due(klass):
                await self.commit(classes=(klass,))

    async def commit(self, classes=('lo', 'bin'), tones=None):
        """Commit staged corrections (one revision per class committed).

        In dry-run mode the staged set is consumed, logged and announced
        with ``dry_run: true`` but nothing is applied. A mixed decision
        batch is split: each class commits independently and never blocks
        the other.
        """
        results = []
        for klass in classes:
            taken = self.controller.take_staged(classes=(klass,), tones=tones)
            if klass not in taken:
                continue
            corrections = taken[klass]
            if self.dry_run:
                self.server._announce_tracking_dry_run(klass, corrections,
                                                       self.controller)
                results.append({'class': klass, 'dry_run': True,
                                'tones': sorted(corrections)})
                continue
            try:
                result = await self.server._commit_tracking_update(
                    self.engine, klass, corrections,
                    self.controller.staged_detunings)
                if 'reclassified_to_bin' in result:
                    # The prepared bundle reported needs_recenter: the shift
                    # really needs a bin/map change. Move the tones to the
                    # bin staged set (committed on the bin policy instead).
                    self.controller.staged['bin'].update(
                        result['reclassified_to_bin'])
                    self.controller.corrections_suppressed['lo'] += len(
                        result['reclassified_to_bin'])
                else:
                    self.commits[klass] += 1
                    self.last_commit_ts[klass] = time.time()
                results.append(result)
            except Exception as e:
                self.commit_errors += 1
                # Put the corrections back so an on-demand commit can retry.
                self.controller.staged[klass].update(corrections)
                print(f'tracking commit ({klass}) failed: {e}')
                print(traceback.format_exc())
                results.append({'class': klass, 'error': str(e)})
        return results

    # -- lifecycle / info ------------------------------------------------------

    def start(self):
        """Start the consumer task."""
        self._task = asyncio.get_event_loop().create_task(self.run())

    def stop(self):
        """Stop the consumer task and drop buffered data."""
        self._stop = True
        self.enabled = False
        if self._task is not None:
            self._task.cancel()
        self.ring.clear()

    # -- health ---------------------------------------------------------------

    def tone_health(self):
        """Per-tone tracking health and a lock state for each tracked tone.

        Lock states:

        - ``locked``: filtered |detuning| inside the deadband, estimates
          healthy.
        - ``drifting``: filtered |detuning| over the threshold (a
          correction is confirming/staging or suppressed by config).
        - ``recenter_pending``: a bin/map correction is staged for this
          tone, awaiting its commit policy or an explicit commit.
        - ``unlocked``: the estimates are untrustworthy -- the resonance is
          likely lost and the loop would be tracking noise. Detected by the
          phase slope collapsing below ``unlock_slope_ratio`` of its
          settled baseline and/or the invalid-cycle fraction exceeding
          ``unlock_invalid_fraction`` (a far-off-resonance tone has a flat
          phase, so the reading *shrinks* rather than grows -- see the
          compression note in ``__init__``). ``unlock_detuning_linewidths``
          optionally also trips this on the reading itself.
        - ``no_data``: no usable estimate yet (startup, or held).

        Returns
        -------
        (tones, counts) : (dict, dict)
            ``tones``: ``{tone_index: {state, detuning_linewidths,
            detuning_std_linewidths, slope_ratio, invalid_fraction,
            staged_center_hz}}`` for every tracked tone. ``counts``: number
            of tones in each state.
        """
        p = self.params
        filtered = self.last_filtered_lw
        with np.errstate(invalid='ignore', divide='ignore'):
            std = np.sqrt(self.stat_detuning_var)
            slope_ratio = (self.stat_abs_slope / self.slope_baseline
                           if self.slope_baseline is not None
                           else np.full(self.n_tones, np.nan))
        unlock_lw = p.get('unlock_detuning_linewidths')
        tones = {}
        counts = {'locked': 0, 'drifting': 0, 'recenter_pending': 0,
                  'unlocked': 0, 'no_data': 0}

        def _value(x):
            return float(x) if np.isfinite(x) else None

        for i in np.flatnonzero(self.controller.enable_mask):
            i = int(i)
            f = filtered[i]
            bad_slope = (np.isfinite(slope_ratio[i])
                         and slope_ratio[i] < p['unlock_slope_ratio'])
            bad_invalid = (np.isfinite(self.stat_invalid_frac[i])
                           and self.stat_invalid_frac[i]
                           > p['unlock_invalid_fraction'])
            bad_reading = (unlock_lw is not None and np.isfinite(f)
                           and abs(f) > float(unlock_lw))
            if bad_slope or bad_invalid or bad_reading:
                state = 'unlocked'
            elif i in self.controller.staged['bin']:
                state = 'recenter_pending'
            elif np.isfinite(f) and \
                    abs(f) > self.controller.threshold_linewidths:
                state = 'drifting'
            elif np.isfinite(f):
                state = 'locked'
            else:
                state = 'no_data'
            counts[state] += 1
            staged = self.controller.staged['lo'].get(
                i, self.controller.staged['bin'].get(i))
            tones[i] = {
                'state': state,
                'detuning_linewidths': _value(f),
                'detuning_std_linewidths': _value(std[i]),
                'slope_ratio': _value(slope_ratio[i]),
                'invalid_fraction': _value(self.stat_invalid_frac[i]),
                'staged_center_hz': (None if staged is None
                                     else float(staged)),
            }
        return tones, counts

    def summary(self, counts=None):
        """Compact health summary (the lean block for monitoring tools).

        ``counts`` may be a precomputed per-state count dict (from
        :meth:`tone_health`) to avoid recomputing the O(n_tones) per-tone
        pass -- :meth:`_build_snapshot` passes it so the whole snapshot
        costs one ``tone_health`` call, not two.
        """
        if counts is None:
            _, counts = self.tone_health()
        tracked = self.controller.enable_mask
        filtered = self.last_filtered_lw[tracked]
        detuning_hz = filtered * self.linewidth_hz[tracked]
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            max_lw = np.nanmax(np.abs(filtered)) if filtered.size else np.nan
            median_lw = np.nanmedian(np.abs(filtered)) if filtered.size \
                else np.nan
            max_hz = np.nanmax(np.abs(detuning_hz)) if filtered.size \
                else np.nan
        over = np.abs(filtered) > self.controller.threshold_linewidths
        now = time.time()
        return {
            'enabled': self.enabled,
            'engine': self.engine,
            'dry_run': self.dry_run,
            'held': bool(self.held or self.hold_reason),
            'hold_reason': self.hold_reason or
                           ('held by request' if self.held else None),
            'filter_settled': self.filter.settled,
            'n_tracked': int(np.count_nonzero(tracked)),
            'n_locked': counts['locked'],
            'n_drifting': counts['drifting'],
            'n_recenter_pending': counts['recenter_pending'],
            'n_unlocked': counts['unlocked'],
            'n_no_data': counts['no_data'],
            'n_over_threshold': int(np.count_nonzero(
                over & np.isfinite(filtered))),
            'max_abs_detuning_linewidths': (None if not np.isfinite(max_lw)
                                            else float(max_lw)),
            'median_abs_detuning_linewidths': (
                None if not np.isfinite(median_lw) else float(median_lw)),
            'max_abs_detuning_hz': (None if not np.isfinite(max_hz)
                                    else float(max_hz)),
            'staged': {k: len(s) for k, s in self.controller.staged.items()},
            'commits': dict(self.commits),
            'backoffs': self.backoffs,
            'last_estimate_age_s': (None if self.last_estimate_ts is None
                                    else now - self.last_estimate_ts),
            'applied_revision': self._applied_revision(),
        }

    def _build_snapshot(self):
        """Assemble the full ``get_info('tracking')`` payload.

        This is the O(n_tones) work: it builds the per-tone health dict
        **once** (fixing the previous double ``tone_health`` compute) and
        derives ``summary`` and ``filtered_detuning_linewidths`` from that
        single pass. It is run **off the event loop** (the consumer task's
        ``to_thread`` hop) and cached in ``self._snapshot``; :meth:`status`
        just serves that cache, so a health poll never runs this on the
        stream loop. See doc/tone_tracking.md ("Health monitoring") and
        profiling/tracking_health_benchmark.py.
        """
        tones, counts = self.tone_health()
        recent = {i: t['detuning_linewidths'] for i, t in tones.items()}
        return {
            'summary': self.summary(counts=counts),
            'tones': tones,
            'enabled': self.enabled,
            'engine': self.engine,
            'dry_run': self.dry_run,
            'held': bool(self.held or self.hold_reason),
            'hold_reason': self.hold_reason or
                           ('held by request' if self.held else None),
            'params': {k: (v.tolist() if isinstance(v, np.ndarray) else v)
                       for k, v in self.params.items()},
            'num_points': self.num_points,
            'cycles_seen': self.cycles_seen,
            'filter_settled': self.filter.settled,
            'filtered_detuning_linewidths': recent,
            'controller': self.controller.status(),
            'commits': dict(self.commits),
            'commit_errors': self.commit_errors,
            'backoffs': self.backoffs,
            'last_commit_ts': dict(self.last_commit_ts),
            'last_estimate_ts': self.last_estimate_ts,
            'decision_revision': self.decision_revision,
            'applied_revision': self._applied_revision(),
            'ring_depth': self.ring.maxlen,
            'ring_fill': len(self.ring),
        }

    def status(self):
        """Return the cached ``get_info('tracking')`` snapshot (O(1)).

        The snapshot is rebuilt off-loop by the consumer task each poll
        (:meth:`_build_snapshot`), so this accessor -- called on the event
        loop by ``get_info('tracking')``, ``health_check`` and the SNAPSHOT
        stream frame -- adds no per-tone work to the loop. It is at most one
        poll (``poll_interval_s``) stale, which is far finer than any health
        cadence. Call :meth:`_build_snapshot` directly for a synchronous
        fresh build (tests)."""
        return self._snapshot
