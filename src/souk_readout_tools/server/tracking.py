"""
Server-side FFM tone tracking: watch demodulated detunings, issue corrections.

With fast frequency modulation running, this module keeps every modulated
tone at its operating point during long observations: a lean vectorised
estimator turns whole modulation cycles from the stream, circle-centred with
per-tone sweep fits, into per-tone detunings, a controller filters them, applies a deadband + confirmation
logic, and stages small centre corrections that are committed through the
server's normal ``update_modulation`` machinery (the frame producer stays
the sole owner of modulation hardware writes). Every committed correction is
revision-stamped into the stream and announced in-band as a typed
``TONE_UPDATE`` frame, so post-processing can reconstruct the resonance
exactly (``modulation.demodulate_stream`` does it):

    resonance(t) = center[rev(t)] + center_detuning(t)

Detuning follows the package convention: the resonance minus the modulation
centre, so a positive reading moves the centre up.

Tracking is permanent readout infrastructure (live frequency-shift serving
and array health monitoring are other plausible consumers); the estimator
(:func:`assemble_cycles` + :func:`estimate_detunings`) is deliberately
separable from the controller and from the server wiring.

Performance contract (see doc/tone_tracking.md): the only work added to the
per-accumulation hot path is one coherent add of the I/Q words into the
running average (:meth:`TrackingRuntime.tap`, called from ``prepare_frame``
while the data is still hot) -- 33.1 us per accumulation at 1000 tones for a
boxcar, 51.9 for an EWMA, against ~161 us of typical spare at 2500 Hz.
``frame_stride`` halves or quarters that (17.4 / 10.3 us) for a sqrt(N) noise
penalty that is negligible against thousands of averaged cycles. Estimation, the controller and the health snapshot run in a
consumer task in a thread executor, on ``(blocks, points, tones)`` arrays of a
few tens of kB.

The averaging deliberately happens in the stream task rather than in the
consumer. Handing raw frames across to another core instead moves megabytes
through the L2 that all four A53s share, which loses accumulations on **both**
pipelines -- including ones that are not tracking at all (doc/v712-notes.md).

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

from souk_readout_tools import modulation
from souk_readout_tools.resonator import (detuning_from_derivatives,
                                          phase_derivatives)

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
    # I and Q are already interleaved, so one contiguous cast plus a view beats
    # two strided gathers into complex128: 27 ms -> a few ms for a 0.2 s batch
    # at 1000 tones. float32 carries the 32-bit accumulator words exactly (they
    # fit in the 24-bit mantissa at these scales) and halves the memory traffic.
    z = words[:, :2 * num_tones].astype(np.float32).view(np.complex64)
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
    # here (the tap only runs in the modulated branch) and are dropped. The
    # mask copies every array, ~8 MB of complex z for a 0.2 s batch at 1000
    # tones, so skip it when nothing would be dropped.
    if not usable.all():
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
    zk = z if keep.all() else z[keep]
    mean = None
    if flat_k.size:
        # flat_k is nondecreasing by construction -- cycles ascend, and points
        # ascend within a cycle because a decrease is what starts a new one --
        # so each (cycle, point) group is a contiguous run and reduceat sums
        # them in one pass. np.add.at, which does not need that, is an
        # unbuffered scatter and cost 150 ms of a 200 ms batch on the ARM.
        if np.all(np.diff(flat_k) >= 0):
            starts = np.flatnonzero(np.r_[True, np.diff(flat_k) != 0])
            groups = flat_k[starts]
            if starts.size == flat_k.size:
                # One sample per point (samples_per_point=1): the rows already
                # are the per-point means, so skip the reduction and the
                # divide -- both are passes over ~8 MB.
                group_means = zk
            else:
                group_sums = np.add.reduceat(zk, starts, axis=0)
                group_counts = np.diff(np.r_[starts, flat_k.size]).astype(float)
                group_means = group_sums / group_counts[:, None]
            if groups.size == minlength and np.array_equal(groups, np.arange(minlength)):
                # Every point of every cycle present and in order, which is the
                # normal case: use the sums as they are rather than allocating
                # and scattering into an 8 MB NaN array.
                mean = group_means
            else:
                mean = np.full((minlength, n_tones), np.nan, dtype=complex)
                mean[groups] = group_means
        else:                                   # not expected; correct but slow
            counts = np.bincount(flat_k, minlength=minlength).astype(float)
            sums_r = np.zeros((minlength, n_tones))
            sums_i = np.zeros((minlength, n_tones))
            np.add.at(sums_r, flat_k, zk.real)
            np.add.at(sums_i, flat_k, zk.imag)
            with np.errstate(invalid='ignore', divide='ignore'):
                mean = (sums_r + 1j * sums_i) / counts[:, None]
            mean[counts == 0] = np.nan
    if mean is None:                            # nothing kept: all points empty
        mean = np.full((minlength, n_tones), np.nan, dtype=complex)
    z_cycles = mean.reshape(n_cycles, num_points, n_tones)

    # Cycle-level metadata: last sample of each cycle.
    last_idx = np.searchsorted(cycle_sel, np.arange(1, n_cycles + 1)) - 1
    cycles = {
        'z': z_cycles,
        'revision': batch['revision'][sel][last_idx],
        'tt': batch['tt'][sel][last_idx],
    }
    return cycles, carry_out


# Cycles behind one coherently averaged block, at most.
MAX_CYCLES_PER_BLOCK = 32



def average_cycles(z, blocks):
    """Coherently average ``(n_cycles, N, n_tones)`` cycles into ``blocks`` rows.

    The detuning estimate is nonlinear in the cycle data (phase unwrap, then a
    curvature/slope ratio), so averaging I/Q *before* it both costs far less and
    is better statistically than averaging noisy per-cycle ratios afterwards.
    A handful of blocks per batch keeps a scatter estimate for the health
    statistics; one block would leave none.

    Returns ``(min(blocks, n_cycles), N, n_tones)``. Whole cycles only: a
    remainder shorter than a block is dropped, and comes back in the next batch
    through the carry.
    """
    n_cycles = z.shape[0]
    blocks = max(1, min(int(blocks), n_cycles))
    per = n_cycles // blocks
    if per < 1:
        return z
    usable = per * blocks
    return z[:usable].reshape(blocks, per, *z.shape[1:]).mean(axis=1)


def center_iq(z, freq_hz, calibrations):
    """De-embed and circle-centre ``(..., N, n_tones)`` I/Q, one calibration per tone.

    The curvature/slope detuning needs the phase taken about the resonance-circle
    centre, where a notch is exactly ``-2 atan(2x)``. About the raw origin it
    reads ``x (1 + b + b**2)`` near resonance, ``b = 1 / (1 - Ql/Qc)``: 3x for a
    shallow dip, ~100x for a deep one.

    Parameters
    ----------
    z : numpy.ndarray
        Complex I/Q, last two axes ``(N, n_tones)``.
    freq_hz : numpy.ndarray
        ``(N, n_tones)`` absolute probe frequencies (centre + offset), Hz.
    calibrations : dict
        ``{tone_index: ResonatorCalibration}``. Tones without one come back NaN.

    Returns
    -------
    numpy.ndarray
        Centred I/Q, same shape as ``z``.
    """
    out = np.full(np.shape(z), np.nan + 0j)
    for t, cal in calibrations.items():
        out[..., t] = cal.transform_raw_iq(freq_hz[:, t], z[..., t])
    return out


def estimate_detunings(z, offsets, linewidth_hz):
    """Per-cycle centre detuning from the phase slope and curvature, all tones at once.

    ``resonator.phase_derivatives`` (``method='fast'``, the same kernel the
    demodulators use) on the lowest, central and highest probe offsets of each
    tone, whatever order the slots are in; points that share an offset are
    averaged coherently first, so the firmware default ``[-d, 0, +d, 0]``
    uses both centre dwells. Phase differences are angles of products of
    neighbouring points, so nothing is unwrapped.

    On the circle-centred phase (:func:`center_iq`) a notch is exactly
    ``-2 atan(2x)``, and ``resonator.detuning_from_derivatives`` inverts that
    from the slope and curvature alone: no linewidth needed and no
    compression (within ~1% at +/-0.1 lw offsets, ~5% low at +/-0.25). The
    linewidth only converts to Hz. Raw, uncentred I/Q reads 13x to 25x high
    or worse.

    Parameters
    ----------
    z : numpy.ndarray
        ``(n_cycles, N, n_tones)`` complex per-point means, circle-centred.
    offsets : numpy.ndarray
        ``(N, n_tones)`` per-point probe offsets in Hz.
    linewidth_hz : numpy.ndarray
        ``(n_tones,)`` per-tone resonator linewidths in Hz, for Hz.

    Returns
    -------
    dict
        ``center_detuning_linewidths`` and ``center_detuning_hz`` -- the
        resonance minus the modulation centre -- as ``(n_cycles, n_tones)``
        (NaN where not estimable, including tones with fewer than three
        distinct offsets); ``dphi_df`` the phase slope (rad/Hz).

    Examples
    --------
    A tone 0.3 linewidths above a centred arctan-phase resonance: the
    resonance is 0.3 linewidths below it.

    >>> lw = np.array([5e4]); off = np.array([-5e3, 0, 5e3, 0])[:, None]
    >>> z = np.exp(-2j * np.arctan(2 * (0.3 * lw + off) / lw))[None]
    >>> round(float(estimate_detunings(z, off, lw)['center_detuning_linewidths'][0, 0]), 3)
    -0.296
    """
    n_cycles, N, n_tones = z.shape
    if N < 3:
        raise ValueError('tracking needs >= 3 modulation points for the '
                         'curvature-based detuning estimate')
    derivatives = phase_derivatives(z, offsets, method='fast')
    detuning = detuning_from_derivatives(derivatives['dphi_df'],
                                         derivatives['d2phi_df2'])
    return {
        'center_detuning_linewidths': detuning,
        'center_detuning_hz': detuning * np.asarray(linewidth_hz, dtype=float),
        'dphi_df': derivatives['dphi_df'],
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
        Deadband on the *filtered* detuning (default 0.05, inside
        ``demodulate``'s 0.1 ``needs_update``).
    confirm_windows : int
        Consecutive decision windows over threshold before staging (default 3).
    gain : float
        Correction gain: ``new_center = center + gain * center_detuning_hz``
        (the detuning is resonance minus centre, so the centre moves toward
        the resonance).
    max_step_linewidths : float
        Per-decision clamp on the correction step (default 0.5).
    enable_mask : array-like of int or None
        Tones allowed to receive corrections (None = all modulated tones).
    """

    def __init__(self, n_tones, mod_indices, linewidth_hz,
                 threshold_linewidths=0.05, confirm_windows=3, gain=1.0,
                 max_step_linewidths=0.5, enable_mask=None,
                 min_detuning_snr=1.0):
        self.n_tones = int(n_tones)
        self.mod_indices = [int(i) for i in mod_indices]
        self.linewidth_hz = np.asarray(linewidth_hz, dtype=float)
        if len(self.linewidth_hz) != self.n_tones:
            raise ValueError('linewidth_hz must have one entry per tone')
        self.threshold_linewidths = float(threshold_linewidths)
        self.confirm_windows = max(1, int(confirm_windows))
        self.gain = float(gain)
        self.max_step_linewidths = float(max_step_linewidths)
        self.min_detuning_snr = float(min_detuning_snr)
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

    def decide(self, filtered_lw, centers_hz, lo_headroom_hz, unlocked=None,
               sigma_lw=None):
        """Run one decision window on the filtered detunings.

        Parameters
        ----------
        filtered_lw : numpy.ndarray
            ``(n_tones,)`` filtered centre detuning in linewidths (resonance
            minus centre).
        centers_hz : numpy.ndarray
            Current per-tone centres (Hz) the detunings were measured
            against.
        lo_headroom_hz : numpy.ndarray or callable
            ``(n_tones,)`` per-tone |centre shift| beyond which the update
            needs a bin/map change (classification only; the commit path
            re-verifies with the prepared bundle's ``needs_recenter``).
        unlocked : numpy.ndarray or None
            ``(n_tones,)`` bool from
            :meth:`TrackingRuntime.unlocked_mask`. These tones are skipped
            and their confirmation count reset, so a tone reading noise
            never stages a correction and re-locking starts the count again.
        sigma_lw : numpy.ndarray or None
            ``(n_tones,)`` scatter of the detuning estimate (linewidths). A
            reading over threshold must also exceed ``min_detuning_snr``
            times its own scatter to stage anything. This is what catches a
            tone -- or a whole pipeline -- that never had a resonator to lock
            to: ``unlocked`` is measured against a baseline captured once the
            filter settles, so if the tone was reading noise then, the
            baseline is noise too and the ratio never trips. Significance
            needs no baseline. A centred tone is unaffected because it is
            not over threshold in the first place.

        Returns
        -------
        list of dict
            Newly staged corrections
            (``{'tone', 'class', 'new_center_hz', 'center_detuning_linewidths'}``).
        """
        self.last_decision_ts = time.time()
        over = (np.abs(filtered_lw) > self.threshold_linewidths) \
            & self.enable_mask & np.isfinite(filtered_lw)
        if unlocked is not None:
            over &= ~np.asarray(unlocked, dtype=bool)
        if sigma_lw is not None and self.min_detuning_snr:
            sig = np.asarray(sigma_lw, dtype=float)
            over &= ~(np.isfinite(sig)
                      & (np.abs(filtered_lw) < self.min_detuning_snr * sig))
        self.over_count = np.where(over, self.over_count + 1, 0)
        ready = self.over_count >= self.confirm_windows

        staged_now = []
        self.headroom_ms = 0.0
        # The per-tone arithmetic is done for all ready tones at once; the loop
        # below only does the bookkeeping. With no resonators every tone can be
        # over threshold, and the arithmetic in Python cost 14.6 ms a batch.
        ready_idx = np.flatnonzero(ready)
        if callable(lo_headroom_hz):
            if ready_idx.size == 0:
                lo_headroom_hz = None       # nothing to classify: skip the cost
            else:
                _t0 = time.perf_counter()
                lo_headroom_hz = lo_headroom_hz()
                self.headroom_ms = 1e3 * (time.perf_counter() - _t0)
        if lo_headroom_hz is None:
            lo_headroom_hz = np.full(self.n_tones, np.inf)
        if ready_idx.size:
            lw_r = self.linewidth_hz[ready_idx]
            clamp_r = self.max_step_linewidths * lw_r
            step_r = np.clip(self.gain * filtered_lw[ready_idx] * lw_r, -clamp_r, clamp_r)
        for k, tone in enumerate(ready_idx):
            tone = int(tone)
            step = float(step_r[k])
            base = self.staged['lo'].get(
                tone, self.staged['bin'].get(tone, centers_hz[tone]))
            new_center = float(base + step)      # toward the resonance
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
                               'center_detuning_linewidths': float(filtered_lw[tone])})
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
# The per-accumulation I/Q average (the only tracking work in the stream loop)
# ---------------------------------------------------------------------------

class IQAccumulator:
    """Coherent per-(block, point) I/Q sums, fed one accumulation at a time.

    This is the tap: ``tap()`` is called for every accumulation the stream loop
    reads, with its interleaved I/Q words, and adds them into the average for
    that accumulation's modulation point. The consumer takes the finished
    window every poll and never sees a raw frame (doc/tone_tracking.md, "Data
    path and performance").

    Both averaging shapes are the real filter, not a first stage:

    - ``'boxcar'``: an int64 sum over the window, then a fresh one -- an int64
      sum *is* a boxcar. Two buffers alternate: ``take_window`` hands one to
      the consumer and taps into the other, and the consumer zeroes the one it
      was given (``TrackingRuntime.window_iq``) before the next take.
    - ``'ewma'``: ``S = (1-a) S + x`` with a per-point normaliser, so the
      average has memory set by ``alpha`` and can be read at any time.

    int64, not an int32 partial folded now and then: the RX chain's FFT shift
    schedule is biased for SNR, so accumulator words can approach full int32
    scale, and an int32 sum would then wrap after two frames -- silently, into
    plausible-looking detunings. ``frame_stride`` keeps one accumulation in N
    instead, for a sqrt(N) noise penalty.

    Parameters
    ----------
    blocks, num_points, n_tones : int
        Interleaved averages per window, modulation points, active tones.
    iq_filter : {'boxcar', 'ewma'}
    frame_stride : int
        Average one accumulation in this many.
    alpha : float
        EWMA weight per update (``'ewma'`` only).
    buffer : buffer or None
        Memory to put the sums in (the stream process passes shared memory);
        ``None`` allocates. :meth:`nbytes` gives the size needed.
    """

    def __init__(self, blocks, num_points, n_tones, iq_filter='boxcar',
                 frame_stride=1, alpha=0.0, buffer=None):
        if iq_filter not in ('boxcar', 'ewma'):
            raise ValueError("iq_filter must be 'boxcar' or 'ewma'")
        self.shape = (int(blocks), int(num_points), 2 * int(n_tones))
        self.num_points = int(num_points)
        self.iq_filter = iq_filter
        self.frame_stride = max(1, int(frame_stride))
        self.alpha = float(alpha)
        # A new buffer starts zeroed -- a bytearray, or a shared file fresh from
        # ftruncate -- so it is not written here: in the stream process that
        # would fault in every page of it at once, on the frame loop's time.
        if buffer is None:
            buffer = bytearray(self.nbytes(blocks, num_points, n_tones, iq_filter))
        if iq_filter == 'boxcar':
            self.sets = np.ndarray((2,) + self.shape, dtype=np.int64, buffer=buffer)
            self.sums = None
        else:
            self.sets = None
            self.sums = np.ndarray(self.shape, dtype=np.float64, buffer=buffer)
        self.current = 0                   # boxcar set being tapped into
        self.enabled = True
        self.revision = None
        self.frames_tapped = 0
        self.frames_skipped = 0
        self._tap_i = 0
        self._block = 0
        self._bind()

    @staticmethod
    def nbytes(blocks, num_points, n_tones, iq_filter='boxcar'):
        """Bytes of memory the sums need."""
        words = int(blocks) * int(num_points) * 2 * int(n_tones)
        return 8 * words * (2 if iq_filter == 'boxcar' else 1)

    def _bind(self):
        """Cache per-(block, point) row views and plain-int counters.

        ``tap`` runs once per accumulation, so its Python overhead is not
        negligible: indexing a 3-D array to get the row and keeping the counts
        in a numpy array cost more than the add itself. Binding the rows once
        and counting in Python ints saves ~14 us per accumulation.
        """
        b, N, _ = self.shape
        if not hasattr(self, '_row_sets'):
            # Views of every row of every buffer, made once: a window swap then
            # only picks the other set (made per swap they cost ~15 numpy
            # calls, and a swap in the stream process lands between frames).
            bases = [self.sums] if self.sums is not None else [self.sets[0], self.sets[1]]
            self._row_sets = [[[base[i, j] for j in range(N)] for i in range(b)]
                              for base in bases]
        self._rows = self._row_sets[self.current if self.sums is None else 0]
        self._counts = [[0] * N for _ in range(b)]
        self._weights = [[0.0] * N for _ in range(b)]
        self._decay = 1.0 - self.alpha

    def tap(self, data, point, settling=False, revision=0):
        """Add one accumulation's interleaved I/Q words into the average."""
        if settling or point < 1 or point > self.num_points:
            return
        self._tap_i += 1
        if self.frame_stride > 1 and self._tap_i % self.frame_stride:
            self.frames_skipped += 1
            return
        if revision != self.revision:
            self.reset(revision)
        if point == 1:
            self._block = (self._block + 1) % len(self._counts)
        b, k = self._block, point - 1
        if data.shape[0] != self.shape[2]:
            return                      # tone count changed under us
        row = self._rows[b][k]
        if self.sums is None:
            np.add(row, data, out=row, casting='unsafe')
        else:
            decay = self._decay
            row *= decay
            np.add(row, data, out=row, casting='unsafe')
            w = self._weights[b]
            w[k] = w[k] * decay + 1.0
        self._counts[b][k] += 1
        self.frames_tapped += 1

    @property
    def counts(self):
        """Per-(block, point) sample counts of the current window."""
        return np.asarray(self._counts, dtype=np.int64)

    def reset(self, revision=None):
        """Drop the part-built average (revision change, hold, tone change)."""
        if self.sums is None:
            self.sets[self.current] = 0
        else:
            self.sums[:] = 0.0
        self._bind()
        self._block = 0
        if revision is not None:
            self.revision = revision

    def take_window(self):
        """Hand the finished window over; None if nothing has been tapped.

        Returns ``('boxcar', sums, counts, revision)`` -- ``sums`` is one of the
        two buffers, which the consumer must zero before the next take -- or
        ``('ewma', sums copy, weights, counts, revision)``. In the server this
        runs on the event loop, like ``tap``, so the two never interleave and
        no lock is needed; it costs O(1) for a boxcar.
        """
        counts = self.counts
        if not counts.any():
            return None
        if self.sums is not None:
            # Always current: the consumer reads a copy and the filter keeps
            # running -- nothing is reset.
            return ('ewma', self.sums.copy(),
                    np.asarray(self._weights, dtype=float), counts, self.revision)
        window = ('boxcar', self.sets[self.current], counts, self.revision)
        self.current ^= 1
        self._bind()
        return window

    def take_window_info(self):
        """``take_window`` for a consumer in another process, which reads the
        sums from the shared buffer itself: which buffer, and the counts.

        Same hand-over as ``take_window`` (a boxcar switches buffers), but the
        sums are not copied -- in the stream process that would be a copy of
        the whole window on the frame loop's core, for nothing.
        """
        counts = self._counts              # plain lists: no numpy on this path
        if not any(any(row) for row in counts):
            return None
        info = {'mode': self.iq_filter, 'counts': counts,
                'revision': self.revision, 'frames_tapped': self.frames_tapped,
                'frames_skipped': self.frames_skipped}
        if self.sums is not None:
            info['weights'] = [list(w) for w in self._weights]
            info['counts'] = [list(c) for c in counts]   # these keep running
        else:
            info['set'] = self.current
            self.current ^= 1
            self._bind()
        return info

    async def next_window(self):
        """``take_window`` for the consumer task (the remote one awaits)."""
        return self.take_window()

    def stop(self):
        """Nothing to release in-process (the remote one tells the streamer)."""
        self.enabled = False


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
    calibrations : dict
        ``{tone_index: ResonatorCalibration}`` used to circle-centre the I/Q
        before estimating. Modulated tones without one are never corrected.
    """

    def __init__(self, server, params, calibrations):
        self.server = server
        self.params = params
        if not calibrations:
            raise ValueError('tracking needs a calibration for at least one '
                             'modulated tone to centre the I/Q')
        self.calibrations = calibrations
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
        # linewidth_hz), else fr/Ql from the calibrations.
        linewidth = params.get('linewidth_hz')
        if linewidth is None:
            linewidth = cfg.get('linewidth_hz')
        if linewidth is None:
            linewidth = [calibrations[t].fr / calibrations[t].Ql
                         if t in calibrations else np.nan
                         for t in self.mod_indices]
        self.linewidth_hz = self._expand_per_tone(linewidth)

        # Full (N, n_tones) offsets for the estimator.
        self.offsets = self._full_offsets(cfg)
        # The live plot applies the server-reported RX flattening factors
        # before calibrated demodulation; match that phase basis here.
        self.readout_correction = modulation.readout_correction_factors(
            {'modulation': state}, self.n_tones)

        sample_rate = float(state.get('sample_rate_hz') or 0.0)
        cycle_rate = float(state.get('cycle_rate_hz') or 0.0)
        cycle_period = 1.0 / cycle_rate if cycle_rate > 0 else 0.01

        self.controller = TrackingController(
            self.n_tones, self.mod_indices, self.linewidth_hz,
            threshold_linewidths=params['threshold_linewidths'],
            confirm_windows=params['confirm_windows'],
            gain=params['gain'],
            max_step_linewidths=params['max_step_linewidths'],
            enable_mask=params.get('enable_mask'),
            min_detuning_snr=params.get('min_detuning_snr', 1.0))
        # The filter is fed one row per averaged block, so its window counts
        # blocks and its EWMA alpha follows the block period, not the cycle.
        self.filter_blocks_per_batch = max(1, int(params.get('filter_blocks_per_batch') or 4))
        window_s = float(params['poll_interval_s'])
        blocks_per_s = max(1e-6, self.filter_blocks_per_batch / window_s)
        self.filter = DetuningFilter(
            params['filter'], self.n_tones,
            window=params['filter_window'],
            tau_s=params['filter_tau_s'],
            cycle_period_s=1.0 / blocks_per_s)

        # Coherent I/Q averaging, done by the stream task as it reads each
        # accumulation, while the data is still hot. A second pass over the
        # frames from another core costs BOTH pipelines dearly -- the four
        # A53s share a 1 MB L2 (doc/v712-notes.md) -- so the averaging happens
        # once, here, and the consumer only ever sees (blocks, points, tones).
        #
        # Both modes are the real filter, not a first stage:
        #   boxcar  sum over `iq_average_s`, then reset -- 33.1 us per frame
        #           at 1000 tones, because an int64 sum *is* a boxcar.
        #   ewma    S = (1-a)S + x with a per-point normaliser, so the average
        #           has memory `iq_average_s` and is readable at any time --
        #           51.9 us per frame (float64). Use `frame_stride` to trade
        #           noise for stream-loop time (stride 2 -> 17.4 / 26.8 us).
        # A *sliding* boxcar is deliberately not offered: it would need every
        # frame of the window kept (100 MB at 5 s and 2500 Hz).
        self.iq_filter = params.get('iq_filter') or 'boxcar'
        if self.iq_filter not in ('boxcar', 'ewma'):
            raise ValueError("iq_filter must be 'boxcar' or 'ewma'")
        self.iq_average_s = float(params.get('iq_average_s') or window_s)
        self.frame_stride = max(1, int(params.get('frame_stride') or 1))
        alpha = 0.0
        if self.iq_filter == 'ewma':
            # One update per point per cycle, so the decay is per cycle; with
            # `blocks` interleaved averages a given block sees every b'th cycle.
            cycle_s = self.num_points * self.frame_stride / max(1e-9, sample_rate)
            alpha = 1.0 - np.exp(-cycle_s * self.filter_blocks_per_batch
                                 / self.iq_average_s)
        spec = dict(blocks=self.filter_blocks_per_batch,
                    num_points=self.num_points, n_tones=self.n_tones,
                    iq_filter=self.iq_filter, frame_stride=self.frame_stride,
                    alpha=alpha)
        # With the stream loop in its own process the tap runs there, on
        # shared memory, and this side only takes the finished windows.
        stream_process = getattr(server, 'stream_process', None)
        self.acc = (stream_process.start_tracking(spec) if stream_process
                    else IQAccumulator(**spec))
        self.tap = self.acc.tap            # bound once: it runs per accumulation
        self.acc_peak = 0                  # largest |accumulated sum| seen
        # Wall time of each off-loop step, so the cost of tracking to the
        # stream loop is visible in get_info('tracking') instead of inferred.
        self.consume_ms = self.consume_ms_max = 0.0
        self.stage_ms = {}
        self.snapshot_ms = self.commit_ms = 0.0
        self.commit_stage_ms = {}       # prepare/apply/announce split of the last commit
        # Comparing against modulation.demodulate is a diagnostic, not free:
        # it loops over tones in Python, so at 1000 tones method='accurate'
        # costs 1064 ms and 'fast' 537 ms on this ARM -- held GIL, which loses
        # accumulations on both pipelines. Off by default; set a small tone
        # count to sample it (32 tones ~ 34 ms). The real comparison belongs
        # offline, on a recorded stream.
        self.estimator_compare_tones = int(params.get('estimator_compare_tones') or 0)
        self.estimator_diff = None
        # Longest the cached health snapshot may go unrefreshed when nobody is
        # reading it; a read marks it stale and the next poll rebuilds it.
        self.snapshot_max_age_s = float(params.get('snapshot_max_age_s') or 60.0)

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
        # Requested by update_modulation() with no arguments. Calculated from
        # the same deembedded phase basis as the Mobius live plot, off-loop.
        self.phase_calibrations = None
        self.phase_estimate_requested = False
        self.last_phase_estimate = None

        # Per-tone running health statistics (exponentially weighted over
        # cycles, ~20-cycle memory). Written by process_window and read by
        # tone_health()/_build_snapshot -- all in the consumer's off-loop
        # to_thread, so the sole writer is also the sole reader and there is
        # no cross-thread race. Far off resonance the phase is flat and the
        # detuning estimate is noise, not a reliably large reading,
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
        self._snapshot_ts = time.time()
        self._snapshot_wanted = False

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
            # .item(): NumPy 2 refuses float() of a one-element array, and a
            # single linewidth arrives from the client as [value].
            full[self.mod_indices] = values.item()
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
    # ``tap`` itself is bound in __init__ straight to ``self.acc.tap``: it runs
    # once per accumulation, so it should not pay for a second method call.

    def reset_accumulator(self, revision=None):
        """Drop the part-built average (revision change, hold, tone change)."""
        self.acc.reset(revision)

    def take_window(self):
        """Hand the finished window to the consumer (in-process accumulator)."""
        return self.acc.take_window()

    @property
    def frames_tapped(self):
        return self.acc.frames_tapped

    @property
    def frames_skipped(self):
        return self.acc.frames_skipped

    # -- consumer ------------------------------------------------------------

    @staticmethod
    def _ew_update(target, value, alpha):
        """In-place EW update of ``target`` with ``value`` (NaN-aware)."""
        seed = np.isnan(target) & np.isfinite(value)
        target[seed] = value[seed]
        ok = np.isfinite(target) & np.isfinite(value)
        target[ok] += alpha * (value[ok] - target[ok])

    def _update_stats(self, estimate, n_cycles=None):
        """Fold one batch's estimates into the running health statistics
        (see ``__init__``).

        ``estimate`` now holds one row per averaged block, so the variance is
        the scatter between blocks; ``n_cycles`` is how many cycles they stand
        for, which is what the exponential weight follows.
        """
        detunings = estimate['center_detuning_linewidths']
        if n_cycles is None:
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

    def window_iq(self, window):
        """Turn one accumulated window into ``(blocks, points, tones)`` I/Q.

        Divides the sums by the per-point sample counts and views the
        interleaved words as complex. Blocks that saw no sample come back NaN.
        A boxcar's buffer is zeroed here, whatever happens, because the tap
        will switch back to it at the next take (``IQAccumulator``).
        """
        mode = window[0]
        if mode == 'ewma':
            _, s, w, counts, revision = window
            w = w.copy()
            w[w == 0] = np.nan
            return (s / w[..., None]).view(complex), revision, int(counts.sum())
        _, acc64, counts, revision = window
        try:
            n = counts.astype(float)
            n[n == 0] = np.nan
            z = (acc64 / n[..., None]).view(complex)
            n_frames = int(counts.sum())
            self.acc_peak = max(self.acc_peak, int(np.abs(acc64).max()))
        finally:
            acc64[:] = 0
        return z, revision, n_frames

    def process_window(self, window, centers=None):
        """Estimate from the averaged I/Q. Runs off the loop.

        The coherent average is already complete -- the stream task did it
        over `iq_average_s` while the data was hot -- so this only divides
        out the counts and works on ``(blocks, points, tones)``, a few tens
        of kB whatever the sample rate.
        """
        _t = time.perf_counter
        _t0 = _t()
        blocks, revision, n_frames = self.window_iq(window)
        self.stage_ms['window_iq'] = 1e3 * (_t() - _t0); _t0 = _t()
        n_cycles = n_frames // self.num_points
        if self.phase_estimate_requested and self.phase_calibrations:
            if centers is None:
                centers = np.asarray(self._mod_cfg()['center'], dtype=float)
            indices = [t for t in self.mod_indices
                       if t in self.phase_calibrations]
            if indices:
                try:
                    local_cals = {j: self.phase_calibrations[t]
                                  for j, t in enumerate(indices)}
                    local_offsets = self.offsets[:, indices]
                    phase_blocks = blocks[..., indices]
                    if self.readout_correction is not None:
                        phase_blocks = (phase_blocks *
                                        self.readout_correction[:, indices][None, :, :])
                    phase = modulation.demodulate(
                        {'z': phase_blocks,
                         'offsets_hz': local_offsets,
                         'tone_frequency_hz': centers[indices][None, :] + local_offsets},
                        method='fast', calibration=local_cals)
                    with warnings.catch_warnings():
                        warnings.simplefilter('ignore', RuntimeWarning)
                        slope = np.nanmedian(phase['dphi_df'], axis=0)
                        curvature = np.nanmedian(phase['d2phi_df2'], axis=0)
                    self.last_phase_estimate = {
                        'revision': int(revision or 0),
                        'indices': indices,
                        'centers': np.asarray(centers, dtype=float).copy(),
                        'slope': slope,
                        'curvature': curvature,
                        'timestamp': time.time(),
                    }
                    self.phase_estimate_requested = False
                except Exception as e:
                    self.last_phase_estimate = {
                        'error': f'{type(e).__name__}: {e}'}
                    self.phase_estimate_requested = False
        if centers is None:
            centers = np.asarray(self._mod_cfg()['center'], dtype=float)
        z = blocks
        if self.readout_correction is not None:
            z = z * self.readout_correction[None, :, :]
        z = center_iq(z, np.asarray(centers, dtype=float)[None, :] + self.offsets,
                      self.calibrations)
        self.stage_ms['center'] = 1e3 * (_t() - _t0); _t0 = _t()
        estimate = estimate_detunings(z, self.offsets, self.linewidth_hz)
        self.stage_ms['estimate'] = 1e3 * (_t() - _t0); _t0 = _t()
        if self.estimator_compare_tones:
            self.estimator_diff = self._compare_estimators(z, estimate)
        filtered = self.filter.update(estimate['center_detuning_linewidths'])
        self.stage_ms['filter'] = 1e3 * (_t() - _t0); _t0 = _t()
        self._update_stats(estimate, n_cycles)
        self.stage_ms['stats'] = 1e3 * (_t() - _t0)
        self.cycles_seen += n_cycles
        self.last_filtered_lw = filtered
        self.last_estimate_ts = time.time()
        rev = int(revision or 0)
        return {
            'n_cycles': n_cycles,
            'revision_first': rev,
            'revision_last': rev,
            'filtered_linewidths': filtered,
        }

    def _compare_estimators(self, blocks, estimate):
        """Run the quadratic-fit demodulator on the same averaged I/Q and
        report the difference, without letting it drive any decision.

        ``estimate_detunings`` is ``demodulate(method='fast')`` by
        construction -- a unit test pins them to 1e-10 -- so the comparison
        worth making is against ``method='accurate'``, which fits a quadratic
        to phase against offset instead of using only the extreme points. That
        matters for a pattern like ``[0, +d, 0, -d]``, whose repeated centre
        point 'fast' does not use. At one window per ``poll_interval_s`` the
        extra cost no longer matters, so both can be carried until one is
        chosen on evidence.
        """
        try:
            n = min(self.estimator_compare_tones, self.n_tones)
            idx = np.linspace(0, self.n_tones - 1, n).astype(int)
            other = modulation.demodulate(
                {'z': blocks[..., idx], 'offsets_hz': self.offsets[:, idx]},
                method='accurate', linewidth_hz=self.linewidth_hz[idx])
            ref = np.asarray(estimate['center_detuning_linewidths'], dtype=float)[:, idx]
            alt = np.asarray(other['center_detuning_linewidths'], dtype=float)
            d = np.abs(alt - ref)
            with warnings.catch_warnings():
                warnings.simplefilter('ignore', RuntimeWarning)
                return {
                    'method': 'accurate',
                    'tones_sampled': int(n),
                    'n_compared': int(np.sum(np.isfinite(d))),
                    'median_abs_diff_lw': float(np.nanmedian(d)),
                    'max_abs_diff_lw': float(np.nanmax(d)),
                    'median_abs_ref_lw': float(np.nanmedian(np.abs(ref))),
                }
        except Exception as e:                      # never break tracking
            return {'error': f'{type(e).__name__}: {e}'}

    def _consume_window(self, window, centers):
        """Off-loop consumer step: estimate + decision-gate + decide.

        Runs entirely in the ``to_thread`` executor -- the I/Q filter,
        estimation, the estimator comparison, the detuning filter, running
        stats and the controller's deadband/confirmation ``decide`` -- so the
        stream loop is charged only for the per-frame sum in ``tap``.
        Performs **no** hardware writes and never touches
        ``_pending_modulation``; it only mutates this runtime's own
        controller/filter/stat state (the consumer is their sole writer).
        Returns ``True`` when a decision was taken, so the on-loop caller
        runs the commit -- which owns the modulation hardware writes.

        ``centers`` is snapshotted on the loop before the hop so this reads
        no live server modulation state beyond the benign, commit-revalidated
        headroom pre-check.
        """
        result = self.process_window(window, centers)
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
        # Tones reading noise never reach the controller: they would stage a
        # correction every window, which is both wrong (the estimate is
        # meaningless off resonance) and expensive.
        # Headroom is a per-tone Python loop (12.6 ms at 1000 tones) and is
        # only needed to classify tones that have passed confirmation, which
        # is normally none -- so pass the callable and let decide() call it
        # only if it has something to classify.
        _t = time.perf_counter
        _t0 = _t()
        self.controller.decide(result['filtered_linewidths'],
                               centers, self._lo_headroom_hz,
                               unlocked=self.unlocked_mask(
                                   result['filtered_linewidths']),
                               sigma_lw=np.sqrt(self.stat_detuning_var))
        self.stage_ms['decide'] = 1e3 * (_t() - _t0)
        self.stage_ms['headroom'] = round(self.controller.headroom_ms, 2)
        return True

    async def run(self):
        """Consumer task: take each finished average; estimate/decide off-loop.

        The estimation (``_consume_window``) and the health-snapshot rebuild
        run in the ``to_thread`` executor; only taking the window and the
        commit -- which owns the modulation hardware writes -- run on the
        event loop. Taking the window is O(1): the stream task and this
        coroutine share the loop and so never interleave, which is why the
        buffer swap needs no lock.
        """
        # A boxcar average is only complete after `iq_average_s`, so the
        # estimator cannot run faster than that; an EWMA is always current.
        poll = float(self.params['poll_interval_s'])
        if self.iq_filter == 'boxcar':
            poll = max(poll, self.iq_average_s)
        server = self.server
        while not self._stop:
            try:
                await asyncio.sleep(poll)
                if not self.enabled:
                    continue
                if self._interlocked():
                    # Drop the part-built average while held so stale data
                    # never feeds a decision, and restart confirmation.
                    self.reset_accumulator()
                    self.controller.reset_confirmation()
                else:
                    window = await self.acc.next_window()
                    if window is not None:
                        centers = np.asarray(self._mod_cfg()['center'],
                                             dtype=float)
                        t0 = time.perf_counter()
                        decided = await server.to_thread(
                            self._consume_window, window, centers)
                        self.consume_ms = 1e3 * (time.perf_counter() - t0)
                        self.consume_ms_max = max(self.consume_ms_max,
                                                  self.consume_ms)
                        if decided:
                            t0 = time.perf_counter()
                            await self._auto_commit()
                            self.commit_ms = 1e3 * (time.perf_counter() - t0)
                # Refresh the cached health snapshot off-loop (after any
                # commit, so its staged counts are current), but only when
                # someone is actually reading it. It builds a dict per tone --
                # 75 ms at 1000 tones on the ARM -- and that is Python holding
                # the GIL, which costs the stream loop ~190 accumulations at
                # 2500 Hz. Rebuilding it every poll for nobody was the largest
                # remaining source of tracking loss.
                if (self._snapshot_wanted or
                        time.time() - self._snapshot_ts >= self.snapshot_max_age_s):
                    t0 = time.perf_counter()
                    self._snapshot = await server.to_thread(self._build_snapshot)
                    self.snapshot_ms = 1e3 * (time.perf_counter() - t0)
                    self._snapshot_ts = time.time()
                    self._snapshot_wanted = False
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
                    self.commit_stage_ms = result.get('timing_ms', {})
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
        self.reset_accumulator()
        self.acc.stop()

    # -- health ---------------------------------------------------------------

    def unlocked_mask(self, filtered=None):
        """``(n_tones,)`` bool: tones whose reading cannot be trusted.

        One definition, used both by :meth:`tone_health` for reporting and by
        the consumer to gate corrections. A tone reading noise -- a lost
        resonator, or a whole pipeline with none -- must neither be corrected
        (the model-free estimate is meaningless there) nor cost the system
        anything, so these tones are excluded before the controller's
        deadband and confirmation ever see them.
        """
        p = self.params
        if filtered is None:
            filtered = self.last_filtered_lw
        filtered = np.asarray(filtered, dtype=float)
        if self.slope_baseline is not None:
            with np.errstate(divide='ignore', invalid='ignore'):
                ratio = self.stat_abs_slope / self.slope_baseline
        else:
            ratio = np.full(self.n_tones, np.nan)
        bad = np.isfinite(ratio) & (ratio < p['unlock_slope_ratio'])
        bad |= (np.isfinite(self.stat_invalid_frac)
                & (self.stat_invalid_frac > p['unlock_invalid_fraction']))
        unlock_lw = p.get('unlock_detuning_linewidths')
        if unlock_lw is not None:
            bad |= np.isfinite(filtered) & (np.abs(filtered) > float(unlock_lw))
        return bad

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
            ``tones``: ``{tone_index: {state, center_detuning_linewidths,
            center_detuning_std_linewidths, slope_ratio, invalid_fraction,
            staged_center_hz}}`` for every tracked tone. ``counts``: number
            of tones in each state.
        """
        filtered = np.asarray(self.last_filtered_lw, dtype=float)
        with np.errstate(invalid='ignore', divide='ignore'):
            std = np.sqrt(self.stat_detuning_var)
            slope_ratio = (self.stat_abs_slope / self.slope_baseline
                           if self.slope_baseline is not None
                           else np.full(self.n_tones, np.nan))
        staged_lo = self.controller.staged['lo']
        staged_bin = self.controller.staged['bin']

        # The states for all tones at once, first match wins; only building
        # the per-tone dicts below is Python-per-tone (75 ms at 1000 tones on
        # the ARM when the states were worked out tone by tone too).
        finite = np.isfinite(filtered)
        pending = np.zeros(self.n_tones, dtype=bool)
        pending[list(staged_bin)] = True
        over = finite & (np.abs(filtered) > self.controller.threshold_linewidths)
        names = ('unlocked', 'recenter_pending', 'drifting', 'locked')
        state = np.select([self.unlocked_mask(filtered), pending, over, finite],
                          names, 'no_data')
        idx = np.flatnonzero(self.controller.enable_mask)
        state = state[idx]
        counts = {k: int(np.count_nonzero(state == k))
                  for k in names + ('no_data',)}

        def column(values):
            """Tracked tones' values as floats, None where not finite."""
            v = values[idx]
            return [x if ok else None
                    for x, ok in zip(v.tolist(), np.isfinite(v).tolist())]

        tones = {}
        for i, s, f, sd, sr, inv in zip(idx.tolist(), state.tolist(),
                                        column(filtered), column(std),
                                        column(slope_ratio),
                                        column(self.stat_invalid_frac)):
            staged = staged_lo.get(i, staged_bin.get(i))
            tones[i] = {
                'state': s,
                'center_detuning_linewidths': f,
                'center_detuning_std_linewidths': sd,
                'slope_ratio': sr,
                'invalid_fraction': inv,
                'staged_center_hz': None if staged is None else float(staged),
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
            'max_abs_center_detuning_linewidths': (None if not np.isfinite(max_lw)
                                            else float(max_lw)),
            'median_abs_center_detuning_linewidths': (
                None if not np.isfinite(median_lw) else float(median_lw)),
            'max_abs_center_detuning_hz': (None if not np.isfinite(max_hz)
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
        derives ``summary`` and ``filtered_center_detuning_linewidths`` from that
        single pass. It is run **off the event loop** (the consumer task's
        ``to_thread`` hop) and cached in ``self._snapshot``; :meth:`status`
        just serves that cache, so a health poll never runs this on the
        stream loop. See doc/tone_tracking.md ("Health monitoring") and
        profiling/tracking_health_benchmark.py.
        """
        tones, counts = self.tone_health()
        recent = {i: t['center_detuning_linewidths'] for i, t in tones.items()}
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
            'filtered_center_detuning_linewidths': recent,
            'controller': self.controller.status(),
            'commits': dict(self.commits),
            'commit_errors': self.commit_errors,
            'backoffs': self.backoffs,
            'last_commit_ts': dict(self.last_commit_ts),
            'last_estimate_ts': self.last_estimate_ts,
            'decision_revision': self.decision_revision,
            'applied_revision': self._applied_revision(),
            'iq_filter': self.iq_filter,
            'iq_average_s': self.iq_average_s,
            'frame_stride': self.frame_stride,
            'frames_tapped': self.frames_tapped,
            'frames_skipped': self.frames_skipped,   # dropped by frame_stride
            'stage_ms': {k: round(v, 2) for k, v in self.stage_ms.items()},
            'consume_ms': round(self.consume_ms, 2),
            'consume_ms_max': round(self.consume_ms_max, 2),
            'snapshot_ms': round(self.snapshot_ms, 2),
            'commit_ms': round(self.commit_ms, 2),
            'commit_stage_ms': dict(self.commit_stage_ms),   # last commit, fw engine
            'acc_peak': self.acc_peak,               # largest |accumulated sum|
            'acc_headroom': (2**63 / self.acc_peak) if self.acc_peak else None,
            'estimator_diff': self.estimator_diff,
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
        # Asking for it marks it stale, so the next poll rebuilds it off-loop.
        self._snapshot_wanted = True
        snap = self._snapshot
        # The age must be taken now, not when the snapshot was built, or it
        # reads frozen between rebuilds (and through a long commit).
        if snap is not None and self.last_estimate_ts is not None:
            snap = dict(snap)
            snap['summary'] = dict(snap['summary'],
                                   last_estimate_age_s=time.time() - self.last_estimate_ts)
        return snap
