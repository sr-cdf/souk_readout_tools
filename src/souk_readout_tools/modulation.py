"""
Fast-frequency-modulation client toolkit (setup + demodulation).

These are **pure, transport-agnostic** functions: they operate on plain arrays /
dicts (parsed stream samples + the ``tone_modulation`` state), with no
``ReadoutClient`` or socket dependency. The conversion maths -- bases, the
three-point slope/curvature kernel, the estimators -- is ``resonator``'s; this
module groups the cycles, aligns the results to samples and handles revisions.

The workflow a consumer follows is:

0. :func:`params_from_sweep` -> turn a calibration sweep into a ready-to-use
   ``enable_modulation`` config (per-tone centres + per-tone probe deltas scaled
   to each resonator's linewidth, and the per-tone calibrations).
1. Acquire modulated samples (``ReadoutClient.get_samples`` while modulation is
   armed, or a captured continuous stream) and parse them
   (``ReadoutClient.parse_samples`` / ``parse_stream``).
2. :func:`group_cycles` -> per-tone complex measurements grouped by point and
   cycle, the per-point probe offsets and probe frequencies, and the grouped
   packet counters, telescope time, packet errors and stream flags.
   (:func:`point_timestreams` reshapes this into one ``parse_samples``-style
   timestream per probe point, for per-point PSDs with the standard plotters.)
3. :func:`demodulate` (one estimate per cycle) or
   :func:`demodulate_timestream` (every sample) -> resonance shift, matched
   dissipation, phase slope and curvature, and the centre detuning with a
   ``needs_update`` flag. :func:`demodulate_stream` does the same across a
   whole recording's revisions (tracking commits, recentres).

Conventions are ``resonator``'s: resonator-side shifts, detuning = resonance
minus the modulation centre, dissipation ``Delta(1/(2 Qi))``. A ``calibration``
chooses the **basis** (de-embedded and circle-centred per probe point, where
the phase derivatives are well conditioned); ``conversion`` chooses the
**estimator** (``'phase'``, the preferred three-point form; ``'linear'``;
``'mobius'``).

Physics caveats:
- The point we steer to is the **phase inflection** (d2phi/df2 = 0, the
  steepest point): ``fr`` for a linear resonator, ``resonator.operating_point_hz``
  for a driven one.
- The detuning ``phi'' / (2 phi' |phi'|)`` is exact for the centred notch phase
  and needs no linewidth; about the raw origin it reads 13x to 25x high.
- Dissipation comes from the radius (or the normal to the I/Q gradient), not
  from the phase.
"""

import copy
import os
import warnings

import numpy as np

from .resonator import (ResonatorCalibration, apply_readout_correction,
                        calibration_for_tone, calibration_from_data,
                        detuning_from_derivatives, is_calibration_record,
                        mobius_center_detuning, operating_point_hz, phase_derivatives,
                        phase_shift_and_dissipation, project_iq_tangent_normal,
                        readout_correction_factors, recorded_rx_gain_ratio,
                        to_basis)


def _record_gain(calibration, info):
    """RX amplitude ratio, data over calibration sweep, divided out before
    centring: from the chain states a calibration record remembers and the
    data's ``info`` (``resonator.recorded_rx_gain_ratio``); 1 when either is
    unknown or the calibration is not a record."""
    if not is_calibration_record(calibration):
        return 1.0
    ratio = recorded_rx_gain_ratio(calibration, info)
    return 1.0 if ratio is None else float(ratio)


def _reconstruct_contiguous(pc, points, settling, revisions, z, n_points, spp, n_settle):
    """
    Rebuild the sample arrays on a contiguous packet-counter axis, inserting
    NaN-IQ placeholders for missing packets.

    The modulation tag (point / settling) is periodic in the packet counter, so
    a missing packet's point and settling are inferred from the deterministic
    cadence once the phase is anchored on an observed settling rising edge (or,
    if ``n_settle == 0``, a point transition between two contiguous packets).

    Parameters
    ----------
    pc : numpy.ndarray
        Observed packet counters (int, strictly increasing but possibly gapped).
    points, settling, revisions : numpy.ndarray
        Per-sample tag arrays aligned with ``pc``.
    z : numpy.ndarray
        ``(n_samples, n_tones)`` complex I/Q aligned with ``pc``.
    n_points, spp, n_settle : int
        Cadence: points per cycle, dwell, settling samples per point.

    Returns
    -------
    (pc2, points2, settling2, revisions2, z2, present2, source_indices2) on
    success, or ``None`` if the cadence phase could not be anchored (caller then
    leaves the data unfilled).
    """
    period = max(1, int(n_points) * int(spp))
    spp = max(1, int(spp))
    # Anchor the cadence: find a contiguous pair that marks the start of a dwell.
    c0 = None
    for k in range(1, len(pc)):
        if pc[k] - pc[k - 1] != 1:
            continue
        if n_settle >= 1 and settling[k] == 1 and settling[k - 1] == 0:
            c0 = int(pc[k]) - (int(points[k]) - 1) * spp   # counter of point-1, sub-index 0
            break
        if n_settle == 0 and points[k] != points[k - 1]:
            c0 = int(pc[k]) - (int(points[k]) - 1) * spp
            break
    if c0 is None:
        return None

    lo, hi = int(pc[0]), int(pc[-1])
    full = np.arange(lo, hi + 1, dtype=np.int64)
    present = {int(c): i for i, c in enumerate(pc)}
    n_full, n_tones = len(full), z.shape[1]
    z2 = np.full((n_full, n_tones), np.nan + 1j * np.nan, dtype=complex)
    points2 = np.zeros(n_full, dtype=int)
    settling2 = np.zeros(n_full, dtype=int)
    revisions2 = np.zeros(n_full, dtype=int)
    present2 = np.zeros(n_full, dtype=bool)
    source_indices2 = np.full(n_full, -1, dtype=int)
    last_rev = int(revisions[0]) if len(revisions) else 0
    for j, c in enumerate(full):
        src = present.get(int(c))
        if src is not None:
            z2[j] = z[src]
            points2[j] = int(points[src])
            settling2[j] = int(settling[src])
            last_rev = int(revisions[src])
            revisions2[j] = last_rev
            present2[j] = True
            source_indices2[j] = int(src)
        else:
            idx = (int(c) - c0) % period            # modulation sub-index within the cycle
            points2[j] = (idx // spp) % int(n_points) + 1
            settling2[j] = 1 if (idx % spp) < n_settle else 0
            revisions2[j] = last_rev                # carry the last known revision
    return full, points2, settling2, revisions2, z2, present2, source_indices2


def _mode_int(values, default=0):
    """Return the most common integer in ``values``."""
    values = np.asarray(values, dtype=int)
    if values.size == 0:
        return int(default)
    return int(np.bincount(values).argmax())


def _first_or_default(values, default=0):
    values = np.asarray(values)
    if values.size == 0:
        return default
    return values[0]


def _regular_cycle_indices(points, settling, N, include_settling):
    """
    Sample indices of every complete cycle as an ``(n_cycles, N, dwell)``
    array, or ``None`` when the dwell is not the same at every point of every
    complete cycle (``group_cycles`` then walks the samples instead).

    Cycles are found as the walk finds them: over the non-zero points, a new
    cycle starts wherever the point index drops. Within a cycle the point
    index only rises, so a complete cycle's kept samples are already in
    point-major order and reshape straight to ``(N, dwell)``.
    """
    active = np.flatnonzero(points != 0)
    if not len(active):
        return None
    p = points[active]
    if p.min() < 1 or p.max() > N:
        return None                 # let the walk raise on bad point indices
    cycle = np.concatenate(([0], np.cumsum(p[1:] < p[:-1])))
    keep = np.ones(len(active), dtype=bool) if include_settling else settling[active] == 0
    kept, kept_cycle = active[keep], cycle[keep]
    counts = np.bincount(kept_cycle * N + p[keep] - 1,
                         minlength=(cycle[-1] + 1) * N).reshape(-1, N)
    complete = np.all(counts > 0, axis=1)
    if not complete.any() or np.any(counts[complete] != counts[complete][0, 0]):
        return None
    return kept[complete[kept_cycle]].reshape(-1, N, counts[complete][0, 0])


def _stack_or_empty(items, shape, dtype=float):
    if isinstance(items, np.ndarray):    # already stacked (group_cycles fast path)
        return items
    if items:
        return np.stack(items, axis=0)
    return np.empty(shape, dtype=dtype)


def _tone_iq(data_dict, n_tones):
    """
    Stack a parsed ``data_dict`` into a complex array ``(n_samples, n_tones)``.

    Parameters
    ----------
    data_dict : dict
        Output of ``ReadoutClient.parse_samples`` (has ``i_data`` / ``q_data``
        dicts keyed by zero-padded tone index).
    n_tones : int
        Number of tones to stack.
    """
    i_data = data_dict['i_data']
    q_data = data_dict['q_data']
    cols = []
    for t in range(n_tones):
        key = f'{t:04d}'
        cols.append(np.asarray(i_data[key], dtype=float) + 1j * np.asarray(q_data[key], dtype=float))
    return np.stack(cols, axis=1)   # (n_samples, n_tones)


def _wrap_phase(x):
    """Wrap phase differences to [-pi, pi)."""
    return (x + np.pi) % (2 * np.pi) - np.pi


def _nanmean_quiet(a, axis=None):
    """``nanmean`` without RuntimeWarnings for all-NaN slices."""
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=RuntimeWarning)
        return np.nanmean(a, axis=axis)


def _unwrap_nan_segments(phase):
    """Unwrap finite contiguous runs without bridging NaN gaps."""
    out = np.array(phase, dtype=float, copy=True)
    if out.ndim == 1:
        out = out[:, None]
        squeeze = True
    else:
        squeeze = False
    for col in range(out.shape[1]):
        finite = np.isfinite(out[:, col])
        if not np.any(finite):
            continue
        idx = np.flatnonzero(finite)
        breaks = np.where(np.diff(idx) > 1)[0] + 1
        for seg in np.split(idx, breaks):
            out[seg, col] = np.unwrap(out[seg, col])
    return out[:, 0] if squeeze else out


def _interp_nan_targets(values, targets, valid_source):
    """Linearly interpolate selected samples from valid source samples."""
    out = np.array(values, copy=True)
    targets = np.asarray(targets, dtype=bool)
    valid_source = np.asarray(valid_source, dtype=bool)
    if out.ndim == 1:
        out = out[:, None]
        squeeze = True
    else:
        squeeze = False
    x = np.arange(out.shape[0], dtype=float)
    for col in range(out.shape[1]):
        src = valid_source & np.isfinite(out[:, col])
        dst = targets
        if not np.any(dst) or not np.any(src):
            continue
        out[dst, col] = np.interp(x[dst], x[src], out[src, col])
    return out[:, 0] if squeeze else out


def _clean_indices(values, n_tones):
    """Normalise scalar, sequence or boolean-mask tone indices."""
    if values is None:
        return []
    arr = np.asarray(values)
    if arr.ndim == 0:
        candidates = [arr.item()]
    elif arr.dtype == bool and arr.size == n_tones:
        candidates = np.flatnonzero(arr)
    else:
        candidates = arr.ravel()

    out = []
    seen = set()
    for value in candidates:
        try:
            idx = int(value)
        except (TypeError, ValueError):
            continue
        if 0 <= idx < n_tones and idx not in seen:
            seen.add(idx)
            out.append(idx)
    return out


def _indices_mask(indices, n_tones):
    """Return a boolean mask for user-facing tone indices."""
    mask = np.zeros(int(n_tones), dtype=bool)
    for idx in indices:
        if 0 <= int(idx) < n_tones:
            mask[int(idx)] = True
    return mask


def active_modulation_state(info):
    """
    Return the modulation state describing a capture from a ``get_info`` payload.

    Modulation is reported under the unified ``info['modulation']`` section (with
    an ``engine`` field), whichever engine -- software or firmware-slot -- is
    active (the two are mutually exclusive). The state exposes the ``num_points``
    / per-tone ``center_hz`` / ``offsets_hz`` fields the analysis + plotting
    helpers need.
    """
    if not isinstance(info, dict):
        return None
    state = info.get('modulation')
    if isinstance(state, dict) and state.get('tones'):
        return state
    return None


def _extract_tone_metadata(container):
    """Collect tone role metadata from parsed-sample-style containers."""
    if not isinstance(container, dict):
        return {}
    metadata = {}
    top = container.get('tone_metadata')
    if isinstance(top, dict):
        metadata.update(copy.deepcopy(top))
    info = container.get('info')
    tones = info.get('tones', {}) if isinstance(info, dict) else {}
    if isinstance(tones, dict):
        metadata.update(copy.deepcopy(tones))
    return metadata


def _role_indices_from_metadata(metadata, n_tones):
    """Resolve regular/blind tone indices, using parsed metadata conventions."""
    metadata = metadata if isinstance(metadata, dict) else {}
    blind_known = 'blind_indices' in metadata
    regular_known = 'regular_indices' in metadata

    blind_indices = _clean_indices(metadata.get('blind_indices'), n_tones)
    regular_indices = _clean_indices(metadata.get('regular_indices'), n_tones)

    if not blind_known and 'is_blind' in metadata:
        blind_indices = _clean_indices(metadata.get('is_blind'), n_tones)
        blind_known = True
    if not (blind_known and regular_known) and 'tone_types' in metadata:
        try:
            tone_types = list(metadata.get('tone_types'))
        except TypeError:
            tone_types = []
        if not blind_known:
            blind_indices = [
                i for i, tone_type in enumerate(tone_types[:n_tones])
                if str(tone_type).lower() == 'blind']
            blind_known = bool(tone_types)
        if not regular_known:
            regular_indices = [
                i for i, tone_type in enumerate(tone_types[:n_tones])
                if str(tone_type).lower() == 'regular']
            regular_known = bool(tone_types)

    all_indices = list(range(n_tones))
    if blind_known and not regular_known:
        blind = set(blind_indices)
        regular_indices = [i for i in all_indices if i not in blind]
    elif regular_known and not blind_known:
        regular = set(regular_indices)
        blind_indices = [i for i in all_indices if i not in regular]
    elif not blind_known and not regular_known:
        regular_indices = all_indices
        blind_indices = []

    return blind_indices, regular_indices


def _modulated_indices_from_offsets(offsets_hz):
    """Infer modulated tones from non-zero probe offsets when no state exists."""
    offsets_hz = np.asarray(offsets_hz, dtype=float)
    if offsets_hz.ndim != 2:
        return []
    active = np.any(np.abs(offsets_hz) > 0.0, axis=0)
    return np.flatnonzero(active).astype(int).tolist()


def _annotate_tone_metadata(metadata, n_tones, modulated_indices):
    """Return parsed-compatible role metadata plus modulation-role fields."""
    metadata = copy.deepcopy(metadata) if isinstance(metadata, dict) else {}
    blind_indices, regular_indices = _role_indices_from_metadata(metadata, n_tones)
    modulated_indices = _clean_indices(modulated_indices, n_tones)
    modulated = set(modulated_indices)
    unmodulated_indices = [i for i in range(n_tones) if i not in modulated]

    tone_is_blind = _indices_mask(blind_indices, n_tones)
    tone_is_regular = _indices_mask(regular_indices, n_tones)
    tone_is_modulated = _indices_mask(modulated_indices, n_tones)

    metadata.update({
        'regular_indices': list(regular_indices),
        'blind_indices': list(blind_indices),
        'num_regular_tones': int(len(regular_indices)),
        'num_blind_tones': int(len(blind_indices)),
        'is_blind': tone_is_blind.tolist(),
        'mod_indices': list(modulated_indices),
        'modulated_indices': list(modulated_indices),
        'unmodulated_indices': list(unmodulated_indices),
        'num_modulated_tones': int(len(modulated_indices)),
        'num_unmodulated_tones': int(len(unmodulated_indices)),
        'is_modulated': tone_is_modulated.tolist(),
    })
    metadata.setdefault(
        'tone_types',
        ['blind' if tone_is_blind[i] else 'regular' for i in range(n_tones)])

    return metadata, {
        'regular_indices': list(regular_indices),
        'blind_indices': list(blind_indices),
        'mod_indices': list(modulated_indices),
        'modulated_indices': list(modulated_indices),
        'unmodulated_indices': list(unmodulated_indices),
        'tone_is_blind': tone_is_blind,
        'tone_is_regular': tone_is_regular,
        'tone_is_modulated': tone_is_modulated,
    }


def group_cycles(data_dict, tone_modulation_state, reduce='mean',
                 on_missing='notify', include_settling=False):
    """
    Group modulated samples by probe point and modulation cycle.

    Walks the sample stream, optionally drops settling samples, and collects the
    complex I/Q at each of the N probe points into successive cycles. A new
    cycle is detected when the point index wraps back down (e.g. N -> 1). The
    gap-free ``packet_counter`` guarantees the samples are contiguous so the
    grouping is unambiguous.

    Parameters
    ----------
    data_dict : dict
        Parsed samples (``ReadoutClient.parse_samples``) with ``modulation_point``
        (1..N, 0 = off), ``modulation_settling``, ``modulation_revision`` and the
        per-tone ``i_data`` / ``q_data``.
    tone_modulation_state : dict
        ``get_info('modulation')`` payload, used for ``num_points`` and the
        per-(point, tone) probe ``offsets_hz``.
    reduce : {'mean', None}, optional
        ``'mean'`` (default) averages the kept dwell samples per point per cycle
        -> complex ``z`` of shape ``(n_cycles, N, n_tones)``. ``None`` retains the
        sample axis -> ``(n_cycles, N, n_used, n_tones)`` (``n_used`` = kept
        dwell samples per point; assumed constant).
    on_missing : {'notify', 'fill', 'interpolate'}, optional
        How to handle gaps in ``packet_counter`` (dropped accumulations). A
        warning reporting the number and location of missing packets is always
        emitted when packets are missing.
        ``'notify'`` (default) proceeds without reconstruction (cycles straddling
        a gap are simply dropped, since they are incomplete).
        ``'fill'`` rebuilds the stream on a contiguous counter axis with
        **NaN-IQ placeholders** for the missing packets (their point/settling
        inferred from the deterministic cadence), so affected cycles still appear
        in the output with NaN where data was lost.
        ``'interpolate'`` reconstructs the same contiguous axis but linearly
        interpolates each missing sample's I/Q from the real samples probing the
        **same modulation point** (so the uniform cycle-rate sampling a PSD needs
        is preserved across any gap, large or small); gaps at the very start/end
        hold the nearest valid value, and a point with no real samples is left
        NaN. Interpolated samples keep their ``sample_present=False`` /
        ``packet_missing=True`` provenance so they stay identifiable.
    include_settling : bool, optional
        If ``False`` (default), drop samples with ``modulation_settling==1``.
        If ``True``, keep them in the grouped ``z`` and metadata arrays while
        preserving their ``modulation_settling`` flag. This is useful for
        sample-aligned demodulation where settling samples should remain in the
        output timestream.

    Returns
    -------
    dict
        ``z`` : grouped complex measurements (see ``reduce``);
        ``offsets_hz`` : ``(N, n_tones)`` per-point probe offsets (Hz);
        ``tone_frequency_hz`` : ``(N, n_tones)`` applied probe frequencies
        (centre + offset);
        ``revision`` : ``(n_cycles,)`` representative config revision per cycle;
        ``packet_counter`` / ``telescope_time`` / ``packet_error`` :
        per-cycle, per-point metadata. For ``reduce='mean'`` these are the first
        kept sample for timing/counters and max/OR for error-like fields; for
        ``reduce=None`` they keep the sample axis;
        ``stream_flags`` : dict of per-cycle, per-point flag arrays;
        ``cycle_packet_counter`` / ``cycle_telescope_time`` / ``cycle_packet_error``
        / ``cycle_stream_flags`` : one representative time axis per cycle, taken
        from the point nearest zero offset;
        ``tone_metadata`` plus ``blind_indices`` / ``regular_indices`` /
        ``mod_indices`` / ``modulated_indices`` / ``unmodulated_indices`` :
        parsed-compatible tone role metadata and explicit modulation roles;
        ``sample_index`` / ``sample_present`` / ``packet_missing`` : provenance
        for grouped samples, useful when ``on_missing='fill'`` inserts NaN-IQ
        placeholders;
        ``point_order`` : the point indices ``1..N``.
    """
    n_tones = int(data_dict['num_tones'])
    N = int(tone_modulation_state['num_points'])
    if N < 1:
        raise ValueError('tone_modulation_state has num_points < 1; is modulation armed?')
    if reduce not in ('mean', None):
        raise ValueError(f"reduce must be 'mean' or None, got {reduce!r}")
    if on_missing not in ('notify', 'fill', 'interpolate'):
        raise ValueError(
            f"on_missing must be 'notify', 'fill' or 'interpolate', got {on_missing!r}")

    points = np.asarray(data_dict['modulation_point'], dtype=int)
    settling = np.asarray(data_dict['modulation_settling'], dtype=int)
    revisions = np.asarray(data_dict['modulation_revision'], dtype=int)
    z = _tone_iq(data_dict, n_tones)
    sample_index = np.arange(len(points), dtype=int)
    sample_present = np.ones(len(points), dtype=bool)

    # Software readout flattening: when the filterbank compensation's RX part
    # cannot be applied in hardware (the RX LO scale word is inert in current
    # firmware -- see doc/filterbank_compensation.md), the server reports
    # per-(point, tone) linear gain factors in the state ('readout_correction')
    # and they are applied here, so downstream demod sees a flat channel. The
    # factors are ~1 over the centre half of a channel. Skipped when
    # parse_samples already applied them (marked 'readout_correction_applied').
    if not data_dict.get('readout_correction_applied', False):
        factors = readout_correction_factors(
            {'modulation': tone_modulation_state}, n_tones)
        if factors is not None and factors.shape[0] == N:
            z = apply_readout_correction(z, points, factors)

    # Detect dropped accumulations via the packet counter, and (per on_missing)
    # notify only, fill the gaps with NaN placeholders, or interpolate them
    # (their tag inferred from the cadence in either reconstruction path).
    pc = data_dict.get('packet_counter')
    source_indices = sample_index.copy()
    if pc is not None:
        pc = np.asarray(pc, dtype=np.int64)
        diffs = np.diff(pc)
        gap_at = np.where(diffs > 1)[0]
        n_missing = int(np.sum(diffs[gap_at] - 1)) if len(gap_at) else 0
        if n_missing > 0:
            locs = [(int(pc[i]), int(diffs[i] - 1)) for i in gap_at[:5]]
            warnings.warn(
                f'group_cycles: {n_missing} missing packet(s) across {len(gap_at)} gap(s) '
                f'[(after_counter, n_missing), ...]: {locs}'
                + (' ...' if len(gap_at) > 5 else ''),
                stacklevel=2)
            if on_missing in ('fill', 'interpolate'):
                filled = _reconstruct_contiguous(
                    pc, points, settling, revisions, z,
                    N, int(tone_modulation_state.get('samples_per_point', 1)),
                    int(tone_modulation_state.get('n_settle', 0)))
                if filled is None:
                    warnings.warn('group_cycles: could not anchor the cadence to fill '
                                  'gaps; proceeding without filling.', stacklevel=2)
                else:
                    pc, points, settling, revisions, z, sample_present, source_indices = filled
                    sample_index = source_indices.copy()
                    if on_missing == 'interpolate':
                        # Fill the NaN-IQ placeholders per modulation point, so a
                        # reconstructed sample is interpolated only from real
                        # samples at the same probe frequency (never across
                        # neighbouring points), keeping the cycle-rate sampling
                        # uniform across the gap.
                        inserted = ~sample_present
                        for p in range(1, N + 1):
                            targets = (points == p) & inserted
                            if np.any(targets):
                                z = _interp_nan_targets(
                                    z, targets, (points == p) & sample_present)

    n_samples = len(points)

    def _aligned_array(name, default=0, dtype=None):
        """Return a sample-aligned metadata array, filling inserted packets."""
        src = data_dict.get(name)
        if src is None:
            if dtype is None:
                dtype = int
            return np.full(n_samples, default, dtype=dtype)
        src = np.asarray(src)
        if len(src) == n_samples and np.all(source_indices >= 0):
            return src.astype(dtype, copy=False) if dtype is not None else src.copy()
        if dtype is None:
            dtype = src.dtype
        out = np.full(n_samples, default, dtype=dtype)
        valid = source_indices >= 0
        if np.any(valid):
            out[valid] = src[source_indices[valid]]
        return out

    pc_meta = (np.asarray(pc, dtype=np.int64) if pc is not None
               else np.arange(n_samples, dtype=np.int64))
    tt_meta = _aligned_array('telescope_time', default=0, dtype=np.uint64)
    err_meta = _aligned_array('packet_error', default=1, dtype=int)
    missing_meta = ~np.asarray(sample_present, dtype=bool)
    err_meta = np.asarray(err_meta, dtype=int).copy()
    err_meta[missing_meta] = 1
    flag_meta = {}
    for name, values in data_dict.get('stream_flags', {}).items():
        values = np.asarray(values)
        out = np.zeros(n_samples, dtype=values.dtype)
        valid = source_indices >= 0
        if np.any(valid):
            out[valid] = values[source_indices[valid]]
        flag_meta[name] = out

    # Per-(point,tone) probe offsets and absolute probe frequencies from the
    # modulation state (user order). The absolute frequency (centre + offset) is
    # needed to de-embed the cable delay in the calibrated demod path.
    offsets_hz = np.zeros((N, n_tones), dtype=float)
    tone_frequency_hz = np.zeros((N, n_tones), dtype=float)
    for tone in tone_modulation_state.get('tones', []):
        idx = int(tone['index'])
        if idx < n_tones:
            off = np.asarray(tone['offsets_hz'], dtype=float)
            offsets_hz[:, idx] = off
            tone_frequency_hz[:, idx] = float(tone.get('center_hz', 0.0)) + off

    mod_indices = _clean_indices(tone_modulation_state.get('mod_indices'), n_tones)
    if not mod_indices and 'mod_indices' not in tone_modulation_state:
        mod_indices = _modulated_indices_from_offsets(offsets_hz)
    tone_metadata, tone_roles = _annotate_tone_metadata(
        _extract_tone_metadata(data_dict), n_tones, mod_indices)

    # Accumulators. `cycles_*` collect one finished row per cycle (appended by
    # `_flush`). `cur`/`cur_meta` are the in-progress cycle: per probe point
    # (1..N) a list of the samples seen so far plus their aligned metadata.
    cycles_z = []
    cycles_rev = []
    cycles_sample_index = []
    cycles_sample_present = []
    cycles_packet_missing = []
    cycles_packet_counter = []
    cycles_telescope_time = []
    cycles_packet_error = []
    cycles_mod_point = []
    cycles_mod_settling = []
    cycles_mod_revision = []
    cycles_flags = {name: [] for name in flag_meta}
    cycles_used = []
    cur = {p: [] for p in range(1, N + 1)}
    cur_meta = {p: {
        'sample_index': [],
        'sample_present': [],
        'packet_missing': [],
        'packet_counter': [],
        'telescope_time': [],
        'packet_error': [],
        'modulation_point': [],
        'modulation_settling': [],
        'modulation_revision': [],
        'stream_flags': {name: [] for name in flag_meta},
    } for p in range(1, N + 1)}
    cur_rev = []
    prev_p = None

    def _flush():
        # Commit the current cycle only if every point was seen.
        if all(len(cur[p]) > 0 for p in range(1, N + 1)):
            counts = np.asarray([len(cur[p]) for p in range(1, N + 1)], dtype=int)
            if reduce == 'mean':
                # mean branch: collapse each point's dwell samples to one complex value
                arr = np.stack([np.mean(np.stack(cur[p], axis=0), axis=0)
                                for p in range(1, N + 1)], axis=0)   # (N, n_tones)
                meta_axis = None
            else:
                # retain sample axis; truncate to the common minimum dwell count
                m = min(len(cur[p]) for p in range(1, N + 1))
                arr = np.stack([np.stack(cur[p][:m], axis=0)
                                for p in range(1, N + 1)], axis=0)    # (N, n_used, n_tones)
                meta_axis = m
            cycles_z.append(arr)
            cycles_rev.append(int(np.bincount(cur_rev).argmax()) if cur_rev else 0)

            if meta_axis is None:
                # mean branch metadata: one representative value per point --
                # first sample for timing/counters, max/OR for error-like fields,
                # mode for the config revision.
                cycles_sample_index.append(np.asarray([
                    int(_first_or_default(cur_meta[p]['sample_index'], -1))
                    for p in range(1, N + 1)], dtype=int))
                cycles_sample_present.append(np.asarray([
                    bool(_first_or_default(cur_meta[p]['sample_present'], False))
                    for p in range(1, N + 1)], dtype=bool))
                cycles_packet_missing.append(np.asarray([
                    bool(np.any(cur_meta[p]['packet_missing']))
                    for p in range(1, N + 1)], dtype=bool))
                cycles_packet_counter.append(np.asarray([
                    int(_first_or_default(cur_meta[p]['packet_counter'], -1))
                    for p in range(1, N + 1)], dtype=np.int64))
                cycles_telescope_time.append(np.asarray([
                    int(_first_or_default(cur_meta[p]['telescope_time'], 0))
                    for p in range(1, N + 1)], dtype=np.uint64))
                cycles_packet_error.append(np.asarray([
                    int(np.max(cur_meta[p]['packet_error'])) if cur_meta[p]['packet_error'] else 0
                    for p in range(1, N + 1)], dtype=int))
                cycles_mod_point.append(np.arange(1, N + 1, dtype=int))
                cycles_mod_settling.append(np.asarray([
                    int(np.max(cur_meta[p]['modulation_settling'])) if cur_meta[p]['modulation_settling'] else 0
                    for p in range(1, N + 1)], dtype=int))
                cycles_mod_revision.append(np.asarray([
                    _mode_int(cur_meta[p]['modulation_revision'])
                    for p in range(1, N + 1)], dtype=int))
                for name in flag_meta:
                    cycles_flags[name].append(np.asarray([
                        int(np.max(cur_meta[p]['stream_flags'][name]))
                        if cur_meta[p]['stream_flags'][name] else 0
                        for p in range(1, N + 1)], dtype=flag_meta[name].dtype))
            else:
                # sample-axis branch: keep every retained sample's metadata,
                # truncated to the common dwell count `meta_axis` across points.
                cycles_sample_index.append(np.stack([
                    np.asarray(cur_meta[p]['sample_index'][:meta_axis], dtype=int)
                    for p in range(1, N + 1)], axis=0))
                cycles_sample_present.append(np.stack([
                    np.asarray(cur_meta[p]['sample_present'][:meta_axis], dtype=bool)
                    for p in range(1, N + 1)], axis=0))
                cycles_packet_missing.append(np.stack([
                    np.asarray(cur_meta[p]['packet_missing'][:meta_axis], dtype=bool)
                    for p in range(1, N + 1)], axis=0))
                cycles_packet_counter.append(np.stack([
                    np.asarray(cur_meta[p]['packet_counter'][:meta_axis], dtype=np.int64)
                    for p in range(1, N + 1)], axis=0))
                cycles_telescope_time.append(np.stack([
                    np.asarray(cur_meta[p]['telescope_time'][:meta_axis], dtype=np.uint64)
                    for p in range(1, N + 1)], axis=0))
                cycles_packet_error.append(np.stack([
                    np.asarray(cur_meta[p]['packet_error'][:meta_axis], dtype=int)
                    for p in range(1, N + 1)], axis=0))
                cycles_mod_point.append(np.stack([
                    np.asarray(cur_meta[p]['modulation_point'][:meta_axis], dtype=int)
                    for p in range(1, N + 1)], axis=0))
                cycles_mod_settling.append(np.stack([
                    np.asarray(cur_meta[p]['modulation_settling'][:meta_axis], dtype=int)
                    for p in range(1, N + 1)], axis=0))
                cycles_mod_revision.append(np.stack([
                    np.asarray(cur_meta[p]['modulation_revision'][:meta_axis], dtype=int)
                    for p in range(1, N + 1)], axis=0))
                for name in flag_meta:
                    cycles_flags[name].append(np.stack([
                        np.asarray(cur_meta[p]['stream_flags'][name][:meta_axis],
                                   dtype=flag_meta[name].dtype)
                        for p in range(1, N + 1)], axis=0))
            cycles_used.append(counts)
        # Reset the in-progress buffers (whether or not the cycle was committed).
        for p in range(1, N + 1):
            cur[p] = []
            for key in cur_meta[p]:
                if key == 'stream_flags':
                    for name in cur_meta[p]['stream_flags']:
                        cur_meta[p]['stream_flags'][name] = []
                else:
                    cur_meta[p][key] = []
        cur_rev.clear()

    # Fast path (reduce=None, same dwell everywhere -- the normal case): gather
    # the complete cycles directly instead of walking the samples.
    regular = (_regular_cycle_indices(points, settling, N, include_settling)
               if reduce is None else None)
    if regular is not None:
        cycles_z = z[regular]
        cycles_sample_index = sample_index[regular]
        cycles_sample_present = np.asarray(sample_present, dtype=bool)[regular]
        cycles_packet_missing = missing_meta[regular]
        cycles_packet_counter = pc_meta[regular]
        cycles_telescope_time = tt_meta[regular]
        cycles_packet_error = err_meta[regular]
        cycles_mod_point = points[regular]
        cycles_mod_settling = settling[regular]
        cycles_mod_revision = revisions[regular]
        for name in flag_meta:
            cycles_flags[name] = flag_meta[name][regular]
        cycles_used = np.full((len(regular), N), regular.shape[2], dtype=int)
        rev = revisions[regular].reshape(len(regular), -1)
        cycles_rev = [int(r[0]) if np.all(r == r[0]) else int(np.bincount(r).argmax())
                      for r in rev]
    else:
        # Walk the stream once. The point index ramps up within a cycle (1..N) and
        # wraps back down at the next cycle, which is how cycle boundaries are found.
        for k in range(len(points)):
            p = int(points[k])
            if p == 0:               # not modulating
                continue
            if prev_p is not None and p < prev_p:   # wrapped -> cycle boundary
                _flush()
            if int(settling[k]) and not include_settling:
                # settling transient: advance the point marker but don't accumulate
                prev_p = p
                continue
            # accumulate this sample into its point bucket, with aligned metadata
            cur[p].append(z[k])
            cur_meta[p]['sample_index'].append(int(sample_index[k]))
            cur_meta[p]['sample_present'].append(bool(sample_present[k]))
            cur_meta[p]['packet_missing'].append(bool(missing_meta[k]))
            cur_meta[p]['packet_counter'].append(int(pc_meta[k]))
            cur_meta[p]['telescope_time'].append(int(tt_meta[k]))
            cur_meta[p]['packet_error'].append(int(err_meta[k]))
            cur_meta[p]['modulation_point'].append(int(points[k]))
            cur_meta[p]['modulation_settling'].append(int(settling[k]))
            cur_meta[p]['modulation_revision'].append(int(revisions[k]))
            for name in flag_meta:
                cur_meta[p]['stream_flags'][name].append(flag_meta[name][k])
            cur_rev.append(int(revisions[k]))
            prev_p = p
        _flush()

    # Stack the per-cycle lists into output arrays (empty-safe): one row per
    # completed cycle, with the metadata axis matching the chosen `reduce`.
    if reduce == 'mean':
        meta_shape = (0, N)
    else:
        meta_shape = (0, N, 0)
    sample_index_out = _stack_or_empty(cycles_sample_index, meta_shape, dtype=int)
    sample_present_out = _stack_or_empty(cycles_sample_present, meta_shape, dtype=bool)
    packet_missing_out = _stack_or_empty(cycles_packet_missing, meta_shape, dtype=bool)
    packet_counter_out = _stack_or_empty(cycles_packet_counter, meta_shape, dtype=np.int64)
    telescope_time_out = _stack_or_empty(cycles_telescope_time, meta_shape, dtype=np.uint64)
    packet_error_out = _stack_or_empty(cycles_packet_error, meta_shape, dtype=int)
    mod_point_out = _stack_or_empty(cycles_mod_point, meta_shape, dtype=int)
    mod_settling_out = _stack_or_empty(cycles_mod_settling, meta_shape, dtype=int)
    mod_revision_out = _stack_or_empty(cycles_mod_revision, meta_shape, dtype=int)
    samples_used_out = _stack_or_empty(cycles_used, (0, N), dtype=int)
    stream_flags_out = {
        name: _stack_or_empty(items, meta_shape, dtype=flag_meta[name].dtype)
        for name, items in cycles_flags.items()
    }

    # Choose the representative "cycle time" point: the probe point sitting
    # nearest zero offset (averaged over the actively modulated tones), used for
    # the one-timestamp-per-cycle views below.
    active_offset_cols = np.any(np.abs(offsets_hz) > 0, axis=0)
    if np.any(active_offset_cols):
        center_point = int(np.argmin(np.mean(np.abs(offsets_hz[:, active_offset_cols]), axis=1)))
    elif len(offsets_hz):
        center_point = int(np.argmin(np.mean(np.abs(offsets_hz), axis=1)))
    else:
        center_point = 0

    # Collapse a per-(cycle, point[, sample]) array to one value per cycle:
    # `_cycle_view` reads the centre point, `_cycle_any`/`_cycle_max` reduce
    # over the point (and sample) axes for the boolean/error-like fields.
    def _cycle_view(arr, default_shape=(0,)):
        if arr.shape[0] == 0:
            return np.empty(default_shape, dtype=arr.dtype)
        if arr.ndim == 2:
            return arr[:, center_point]
        # retain-sample-axis output: use the first kept sample at the centre point
        return arr[:, center_point, 0]

    def _cycle_any(arr):
        if arr.shape[0] == 0:
            return np.empty((0,), dtype=bool)
        axes = tuple(range(1, arr.ndim))
        return np.any(arr, axis=axes)

    def _cycle_max(arr):
        if arr.shape[0] == 0:
            return np.empty((0,), dtype=arr.dtype)
        axes = tuple(range(1, arr.ndim))
        return np.max(arr, axis=axes)

    cycle_stream_flags = {
        name: _cycle_max(arr)
        for name, arr in stream_flags_out.items()
    }
    z_out_shape = (0, N, n_tones) if reduce == 'mean' else (0, N, 0, n_tones)

    return {
        'z': _stack_or_empty(cycles_z, z_out_shape, dtype=complex),
        'offsets_hz': offsets_hz,
        'tone_frequency_hz': tone_frequency_hz,
        'tone_metadata': tone_metadata,
        'blind_indices': tone_roles['blind_indices'],
        'regular_indices': tone_roles['regular_indices'],
        'mod_indices': tone_roles['mod_indices'],
        'modulated_indices': tone_roles['modulated_indices'],
        'unmodulated_indices': tone_roles['unmodulated_indices'],
        'tone_is_blind': tone_roles['tone_is_blind'],
        'tone_is_regular': tone_roles['tone_is_regular'],
        'tone_is_modulated': tone_roles['tone_is_modulated'],
        'revision': np.asarray(cycles_rev, dtype=int),
        'sample_index': sample_index_out,
        'sample_present': sample_present_out,
        'packet_missing': packet_missing_out,
        'packet_counter': packet_counter_out,
        'telescope_time': telescope_time_out,
        'packet_error': packet_error_out,
        'stream_flags': stream_flags_out,
        'modulation_point': mod_point_out,
        'modulation_settling': mod_settling_out,
        'modulation_revision': mod_revision_out,
        'samples_per_point_used': samples_used_out,
        'cycle_sample_index': _cycle_view(sample_index_out),
        'cycle_sample_present': ~_cycle_any(packet_missing_out),
        'cycle_packet_missing': _cycle_any(packet_missing_out),
        'cycle_packet_counter': _cycle_view(packet_counter_out),
        'cycle_telescope_time': _cycle_view(telescope_time_out),
        'cycle_packet_error': _cycle_max(packet_error_out),
        'cycle_stream_flags': cycle_stream_flags,
        'cycle_point_index': center_point,
        'point_order': np.arange(1, N + 1),
    }


def point_timestreams(grouped, data_dict=None, sample_rate=None, dwell='mean'):
    """
    Split grouped modulation data into one timestream per probe point.

    Each of the N probe points is visited once per modulation cycle, so its
    samples form a timestream at (a multiple of) the cycle rate. This
    returns, for every point, a dict shaped like ``parse_samples`` output
    (``i_data`` / ``q_data`` / ``sample_rate`` / ...), so the existing
    timestream tools apply directly -- e.g. per-point noise spectra with
    ``plot_timestream_psd(ts[1], format='freq_diss', sweep_data=sweep)``.
    Each dict carries its point's *probe* frequencies in
    ``info['tones']['frequencies_hz']``, so the freq/diss conversion
    references the correct location on the sweep.

    Parameters
    ----------
    grouped : dict
        Output of :func:`group_cycles` (either ``reduce`` mode). Note that
        cycles inserted as NaN placeholders by ``on_missing='fill'`` stay
        NaN here, and a Welch PSD of a stream containing NaN is all-NaN;
        for gappy captures drop or interpolate those cycles first.
    data_dict : dict or None, optional
        The parsed sample dict the grouping came from. Supplies the raw
        stream ``sample_rate`` and fallback centre frequencies for tones
        absent from the modulation state.
    sample_rate : float or None, optional
        Raw stream sample rate (Hz). Overrides ``data_dict``; one of the
        two must provide it.
    dwell : {'mean', int, None}, optional
        ``'mean'`` (default) NaN-averages the kept dwell samples of each
        point to one value per cycle -- a strictly uniform stream at the
        cycle rate (the averaging acts as a boxcar filter before the
        decimation). An integer selects that single dwell sample instead
        (uniform at the cycle rate, no filtering). ``None`` keeps every
        kept dwell sample in time order with no averaging; with more than
        one kept sample per point that stream is *bursty* -- the dwell
        samples sit at the raw rate within each cycle, then wait for the
        next cycle -- and its ``sample_rate`` is the average rate (kept
        samples per point x cycle rate), so spectral features above the
        cycle rate are smeared by the burst sampling. Settling samples
        kept by ``include_settling=True`` are averaged/kept like any
        other -- regroup with ``include_settling=False`` if that is not
        wanted.

    Returns
    -------
    dict
        ``{point_number: ts_dict}`` for point numbers ``1..N``.
        ``point_offsets_hz`` records the per-tone probe offset of that
        point.
    """
    z = np.asarray(grouped['z'])
    if z.ndim == 3:          # reduce='mean': (n_cycles, N, n_tones)
        z = z[:, :, None, :]
    elif z.ndim != 4:
        raise ValueError(f'grouped["z"] must be 3D or 4D, got {z.ndim}D')
    n_cycles, N, n_used, n_tones = z.shape
    if n_cycles < 2:
        raise ValueError('need at least two complete cycles to form a '
                         f'per-point timestream, got {n_cycles}')

    if sample_rate is None:
        if not isinstance(data_dict, dict) or 'sample_rate' not in data_dict:
            raise ValueError('pass data_dict (with sample_rate) or an '
                             'explicit raw-stream sample_rate')
        sample_rate = data_dict['sample_rate']
    # Cycle rate from the per-cycle packet counters: the counter increments
    # once per raw sample, so the median counter step between cycles is the
    # cycle length in raw samples (robust to dropped/incomplete cycles).
    cpc = np.asarray(grouped['cycle_packet_counter'], dtype=float)
    steps = np.diff(cpc)
    steps = steps[steps > 0]
    if not len(steps):
        raise ValueError('cycle_packet_counter has no positive steps; '
                         'cannot establish the cycle rate')
    cycle_rate = float(sample_rate) / float(np.median(steps))

    # Shared time origin = the globally-earliest retained sample (not a mid-cycle
    # cycle reference), so t=0 lands on the true first sample and every point's
    # axis uses the same origin. Each point is visited slightly later within a
    # cycle than the one before, so referencing the raw packet counter to this
    # origin surfaces those small per-point offsets instead of collapsing every
    # point onto the same times.
    valid_pkt = np.asarray(grouped['packet_counter'])[np.asarray(grouped['packet_counter']) >= 0]
    counter0 = float(valid_pkt.min()) if valid_pkt.size else 0.0

    if dwell is None:
        z_sel = z                       # keep every kept dwell sample
    elif dwell == 'mean':
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN dwells
            z_sel = np.nanmean(z, axis=2, keepdims=True)
    else:
        dwell_pick = int(dwell)
        if not 0 <= dwell_pick < n_used:
            raise ValueError(f'dwell index {dwell_pick} out of range for '
                             f'{n_used} kept samples per point')
        z_sel = z[:, :, dwell_pick:dwell_pick + 1, :]
    n_out = z_sel.shape[2]              # kept samples per point per cycle
    point_rate = cycle_rate * n_out

    def _point_meta(name, p):
        # Per-(cycle, point[, sample]) metadata -> flat per-point stream,
        # matching the dwell selection above.
        arr = grouped.get(name)
        if arr is None or np.ndim(arr) not in (2, 3):
            return None
        arr = np.asarray(arr)
        if arr.ndim == 2:               # mean-reduced grouping: one per cycle
            return arr[:, p].copy() if arr.shape == (n_cycles, N) else None
        if arr.shape[:2] != (n_cycles, N):
            return None
        if dwell is None:
            return arr[:, p, :].reshape(-1).copy()
        if dwell == 'mean':             # representative: first kept sample
            return arr[:, p, 0].copy()
        return arr[:, p, int(dwell)].copy()

    # Absolute probe frequency per (point, tone); tones without a modulation
    # state entry sit at 0.0 there, so fall back to the capture's centre
    # frequencies for them.
    freq_hz = np.asarray(grouped['tone_frequency_hz'], dtype=float)
    fallback_freqs = None
    if isinstance(data_dict, dict):
        info = data_dict.get('info')
        if isinstance(info, dict):
            tf = info.get('tones', {}).get('frequencies_hz')
            if tf is not None and len(tf) == n_tones:
                fallback_freqs = np.asarray(tf, dtype=float)

    out = {}
    for p in range(N):
        freqs = freq_hz[p].copy()
        missing = ~np.isfinite(freqs) | (freqs == 0.0)
        if fallback_freqs is not None:
            freqs[missing] = fallback_freqs[missing]
        # (n_cycles, n_out, n_tones) -> (n_cycles * n_out, n_tones), in time
        # order (cycle-major, dwell-minor).
        zp = z_sel[:, p, :, :].reshape(-1, n_tones)
        ts = {
            'i_data': {f'{t:04d}': zp[:, t].real.copy() for t in range(n_tones)},
            'q_data': {f'{t:04d}': zp[:, t].imag.copy() for t in range(n_tones)},
            'sample_rate': point_rate,
            'num_samples': zp.shape[0],
            'num_tones': n_tones,
            # group_cycles / parse_samples already applied the software
            # readout flattening to these samples.
            'readout_correction_applied': True,
            'point_number': p + 1,
            'point_offsets_hz': np.asarray(grouped['offsets_hz'], dtype=float)[p].copy(),
            'info': {'tones': {'frequencies_hz': freqs.tolist()}},
        }
        if grouped.get('tone_metadata') is not None:
            ts['tone_metadata'] = grouped['tone_metadata']
        for name in ('packet_counter', 'telescope_time', 'packet_error',
                     'sample_present', 'packet_missing',
                     'modulation_point', 'modulation_settling'):
            meta = _point_meta(name, p)
            if meta is not None and len(meta) == zp.shape[0]:
                ts[name] = meta
        # Real per-sample time axis from the raw packet counter (shared origin),
        # so x_axis='time' plots this point at its true sub-cycle offset rather
        # than at arange(n)/point_rate. Falls back to arange when the counter is
        # unavailable.
        pc = ts.get('packet_counter')
        if pc is not None and len(pc) == zp.shape[0]:
            ts['time_s'] = (np.asarray(pc, dtype=float) - counter0) / float(sample_rate)
        out[p + 1] = ts
    return out


def _tone_calibration(calibration, t):
    """This tone's ResonatorCalibration from a record, mapping or sequence.

    A sequence is matched on each entry's ``tone_index`` (as
    :func:`params_from_sweep` builds them) so an unordered or partial batch
    fit maps correctly, falling back to position for entries without one.
    Fit results are wrapped.
    """
    if calibration is None:
        return None
    if isinstance(calibration, dict):
        cal = calibration_for_tone(calibration, t)
    else:
        cal = None
        for position, entry in enumerate(calibration):
            if entry is None:
                continue
            idx = int(getattr(entry, 'tone_index', -1))
            if (idx if idx >= 0 else position) == t:
                cal = calibration_for_tone({t: entry}, t)
                break
    return cal if isinstance(cal, ResonatorCalibration) else None


_CONVERSIONS = ('phase', 'linear', 'mobius')


def _check_conversion(conversion, centred, t):
    """The conversion for one tone: the default, or the requested one checked."""
    conversion = conversion or ('phase' if centred else 'linear')
    if conversion not in _CONVERSIONS:
        raise ValueError(f'conversion must be one of {_CONVERSIONS}, got {conversion!r}')
    if conversion in ('phase', 'mobius') and not centred:
        raise ValueError(
            f'conversion={conversion!r} needs a calibration for tone {t} (and the '
            "probe frequencies, grouped['tone_frequency_hz'])")
    return conversion


def _convert_points(conversion, cal, z, reference, slope, gradient, frequency_hz):
    """Resonance shift (Hz) and ``Delta(1/(2 Qi))`` of samples about a reference.

    ``z`` and ``reference`` are in the working basis; ``slope`` / ``gradient``
    the local phase slope and complex I/Q gradient at each sample's point;
    ``frequency_hz`` the tone (centre) frequency the Hz-equivalent loss of the
    first-order forms is divided by.
    """
    if conversion == 'mobius':
        probe_detuning, loss = cal.convert_centered_iq(z)
        reference_detuning, reference_loss = cal.convert_centered_iq(reference)
        return reference_detuning - probe_detuning, loss - reference_loss
    if conversion == 'phase':
        shift, dissipation_hz = phase_shift_and_dissipation(z, reference, slope)
    else:
        shift, dissipation_hz = project_iq_tangent_normal(reference, gradient, z)
    with np.errstate(divide='ignore', invalid='ignore'):
        return shift, dissipation_hz / frequency_hz


def _reference_resonance_hz(cal, reference, probe_hz):
    """The resonance (inflection) of the state a centred reference I/Q shows.

    The calibration's exact inversion at the probe frequency, plus the
    operating-point offset of a driven fit. Free of the finite-difference
    bias of the three-point detuning, and a rotation error only enters at
    second order in the detuning, so joining revisions through it is exact
    for small detunings.
    """
    probe_detuning = cal.convert_centered_iq(reference)[0]
    return float(probe_hz - probe_detuning + operating_point_hz(cal) - cal.fr)


def demodulate_timestream(grouped, offsets=None, *, method='accurate',
                          calibration=None, conversion=None, reference_z=None,
                          data_dict=None, n_samples=None,
                          fill_settling=True,
                          use_settling_for_fit=False,
                          threshold_linewidths=0.1):
    """
    Convert every kept modulation sample into a sample-aligned demodulated stream.

    This is the "do not throw away integration time" companion to
    :func:`demodulate`. It uses the individual samples retained by
    ``group_cycles(..., reduce=None)`` rather than first averaging each point in
    a cycle. Each modulation point is a valid probe of the resonator:

    - the static per-point response is divided out to form ``z_demod`` /
      ``mag`` / ``phase`` as if each sample had been measured at the centre
      point;
    - each sample is converted about its point's reference through that
      point's local phase slope (or I/Q gradient) from the cycle, so every
      sample estimates the same resonance shift;
    - the cycle-level ``dphi/df``, ``d2phi/df2`` and centre detuning are held
      across all samples of that cycle.

    The conventions are ``resonator``'s: ``resonance_shift_hz`` is positive when
    the resonance moves up, ``matched_dissipation_shift`` is ``Delta(1/(2 Qi))``
    and ``center_detuning_*`` is the resonance minus the modulation centre.

    The simplest call takes the capture directly::

        ts = demodulate_timestream(data)            # parse_samples dict
        ts = demodulate_timestream('path/to/stream') # saved stream basename

    For a recording whose tracking or recentring changed the modulation, use
    :func:`demodulate_stream`, which splits it by revision and joins the
    pieces onto one physical reference.

    Parameters
    ----------
    grouped : dict or str or os.PathLike
        A parsed sample dict (``parse_samples`` / ``parse_stream``), a saved
        stream basename (read with ``ReadoutClient.parse_stream``), or the
        output of ``group_cycles(data, state, reduce=None)``.  The first two
        are grouped here with the modulation state from ``data['info']`` and
        ``include_settling=True``, and also serve as ``data_dict``; call
        ``group_cycles`` yourself for other grouping options.  Mean-reduced
        grouped data cannot recover the original per-sample stream and raises a
        ``ValueError``.
    offsets : numpy.ndarray or None, optional
        ``(N, n_tones)`` per-point probe offsets (Hz). ``None`` uses
        ``grouped['offsets_hz']``.
    method : {'accurate', 'model', 'fast'}, optional
        Phase-derivative estimator (``resonator.phase_derivatives``).
        ``'accurate'`` / ``'model'`` (the same thing) fit a quadratic of phase
        against offset, so every point counts; ``'fast'`` uses finite
        differences of the lowest, central and highest offsets. The default is
        ``'accurate'`` because repeated centre points such as
        ``[-1000, 0, +1000, 0]`` are common.
    calibration : dict or sequence or None, optional
        Per-tone calibrations (``{tone_index: ResonatorCalibration}``, fits or a
        calibration record; e.g. ``params_from_sweep(...)['calibration']``).
        It chooses the *basis*: a calibrated tone is de-embedded and
        circle-centred at each point's probe frequency, where the phase
        derivatives are well conditioned; an uncalibrated one stays raw.
    conversion : {'phase', 'linear', 'mobius'} or None, optional
        The per-sample estimator. ``'phase'`` (the default for a calibrated
        tone): centred phase and log-radius over the point's live phase slope
        (``resonator.phase_shift_and_dissipation``). ``'linear'`` (the default
        without a calibration, and the only option there): tangent/normal
        projection on the point's live complex gradient, first-order in any
        basis. ``'mobius'``: the calibration's exact inversion.
    reference_z : numpy.ndarray or None, optional
        Per-point reference response, ``(N, n_tones)``, in the working basis
        (raw, or centred for a calibrated tone). Omitted, or NaN for a tone, the
        reference is the capture mean of each point (settling excluded).
        :func:`demodulate_stream` passes it to hold one reference across a
        long recording. For a calibrated tone, ``resonance_frequency_hz`` is
        the reference state's resonance -- from the calibration's inversion of
        the reference centre point -- plus ``resonance_shift_hz``.
    data_dict : dict or None, optional
        Original parsed sample dict.  If supplied, output length is
        ``len(data_dict['modulation_point'])`` and raw per-sample metadata are
        copied through.  Otherwise ``n_samples`` is inferred from grouped sample
        indices unless supplied explicitly.
    n_samples : int or None, optional
        Explicit output length when ``data_dict`` is not supplied.
    fill_settling : bool or {'raw', 'interpolate', 'none'}, optional
        How to fill samples flagged with ``modulation_settling==1``. ``True`` /
        ``'raw'`` (default) demodulates the raw settling I/Q with the completed
        cycle's reference/slope; they remain flagged. ``'interpolate'`` fills
        them by linear interpolation between non-settling demodulated samples.
        ``False`` / ``'none'`` leaves them NaN. Incomplete cycles and missing
        packets remain NaN.
    use_settling_for_fit : bool, optional
        If ``False`` (default), settling samples are excluded from the
        per-point reference, phase-slope and curvature estimates.
    threshold_linewidths : float, optional
        ``needs_update`` trips when ``|center_detuning_linewidths|`` exceeds
        this.

    Returns
    -------
    dict
        Sample-aligned arrays, shape ``(n_samples, n_tones)``: ``z_demod``,
        ``mag``, ``phase_rad``, ``phase_unwrapped_rad``, ``resonance_shift_hz``,
        ``matched_dissipation_shift``, ``resonance_frequency_hz`` (calibrated
        tones only; NaN otherwise), ``center_detuning_hz`` /
        ``center_detuning_linewidths`` / ``needs_update`` (per cycle,
        calibrated tones only, by the calibration's inversion,
        ``resonator.mobius_center_detuning``), ``dphi_df`` (local
        point slope), ``center_dphi_df``, ``d2phi_df2`` and
        ``modulation_offset_hz``. Per tone: ``conversion`` and ``basis`` (lists),
        ``point_reference_z``, ``point_gradient`` (mean ``dz/df`` per point),
        ``reference_center_detuning_hz``,
        ``reference_resonance_frequency_hz``,
        ``reference_matched_dissipation`` (the reference's absolute
        ``Delta(1/(2 Qi))`` against the calibration's Qi; calibrated tones only)
        and ``dissipation_reference_frequency_hz``. Metadata arrays ``cycle_index``,
        ``modulation_point``, ``modulation_settling``, ``sample_present``,
        ``sample_used_in_cycle`` and packet/telescope fields are included when
        available.
    """
    if method not in ('accurate', 'model', 'fast'):
        raise ValueError(f"method must be 'accurate', 'model' or 'fast', got {method!r}")
    # Accept a stream file or a parsed capture as well as group_cycles output.
    if isinstance(grouped, (str, os.PathLike)):
        from .client.readout_client import ReadoutClient
        grouped = ReadoutClient.parse_stream(grouped)
    if 'z' not in grouped:
        state = active_modulation_state(grouped.get('info'))
        if state is None:
            raise ValueError('capture has no modulation state in its info; '
                             'was modulation enabled when it was taken?')
        if data_dict is None:
            data_dict = grouped
        grouped = group_cycles(grouped, state, reduce=None,
                               include_settling=True)
    if isinstance(fill_settling, str):
        fill_mode = fill_settling.lower()
    else:
        fill_mode = 'raw' if fill_settling else 'none'
    fill_aliases = {
        'true': 'raw',
        'cycle': 'raw',
        'raw': 'raw',
        'false': 'none',
        'none': 'none',
        'nan': 'none',
        'interpolate': 'interpolate',
        'interp': 'interpolate',
        'linear': 'interpolate',
    }
    if fill_mode not in fill_aliases:
        raise ValueError(
            "fill_settling must be bool or one of "
            "'raw', 'interpolate', 'none'")
    fill_mode = fill_aliases[fill_mode]
    kernel = 'fast' if method == 'fast' else 'fit'

    # Require the sample-retaining grouping (4-D z); mean-reduced data cannot
    # recover the per-sample stream this function rebuilds.
    z = np.asarray(grouped['z'])
    if z.ndim != 4:
        raise ValueError(
            'demodulate_timestream requires group_cycles(..., reduce=None); '
            f'grouped["z"] has {z.ndim} dimensions')
    n_cycles, N, n_used, n_tones = z.shape

    sample_index = np.asarray(grouped.get('sample_index'))
    if sample_index.shape != (n_cycles, N, n_used):
        raise ValueError(
            'grouped["sample_index"] must have shape '
            f'{(n_cycles, N, n_used)}, got {sample_index.shape}')

    if offsets is None:
        offsets = grouped['offsets_hz']
    offsets = np.asarray(offsets, dtype=float)
    if offsets.ndim == 1:
        offsets = offsets[:, None]
    if offsets.shape != (N, n_tones):
        raise ValueError(
            f'offsets must have shape {(N, n_tones)}, got {offsets.shape}')

    tone_frequency_hz = grouped.get('tone_frequency_hz')
    if tone_frequency_hz is not None:
        tone_frequency_hz = np.asarray(tone_frequency_hz, dtype=float)

    if data_dict is not None:
        n_samples_out = len(data_dict['modulation_point'])
    elif n_samples is not None:
        n_samples_out = int(n_samples)
    else:
        valid_indices = sample_index[sample_index >= 0]
        n_samples_out = int(np.max(valid_indices) + 1) if valid_indices.size else 0

    # Allocate the sample-aligned output (one row per original sample, NaN until
    # a grouped sample or raw-fill writes into it).
    sample_shape = (n_samples_out, n_tones)
    out = {
        'num_tones': int(n_tones),
        'num_samples': int(n_samples_out),
        'z_demod': np.full(sample_shape, np.nan + 1j * np.nan, dtype=complex),
        'mag': np.full(sample_shape, np.nan),
        'phase_rad': np.full(sample_shape, np.nan),
        'phase_unwrapped_rad': np.full(sample_shape, np.nan),
        'resonance_shift_hz': np.full(sample_shape, np.nan),
        'matched_dissipation_shift': np.full(sample_shape, np.nan),
        'resonance_frequency_hz': np.full(sample_shape, np.nan),
        'dphi_df': np.full(sample_shape, np.nan),
        'center_dphi_df': np.full(sample_shape, np.nan),
        'd2phi_df2': np.full(sample_shape, np.nan),
        'modulation_offset_hz': np.full(sample_shape, np.nan),
        'center_detuning_hz': np.full(sample_shape, np.nan),
        'center_detuning_linewidths': np.full(sample_shape, np.nan),
        'needs_update': np.zeros(sample_shape, dtype=bool),
        'sample_index': np.arange(n_samples_out, dtype=int),
        'cycle_index': np.full(n_samples_out, -1, dtype=int),
        'modulation_point': np.zeros(n_samples_out, dtype=int),
        'modulation_settling': np.zeros(n_samples_out, dtype=int),
        'sample_present': np.zeros(n_samples_out, dtype=bool),
        'sample_used_in_cycle': np.zeros(n_samples_out, dtype=bool),
    }
    if data_dict is not None:
        for key in ('date', 'sample_rate'):
            if key in data_dict:
                out[key] = data_dict[key]
        if 'info' in data_dict:
            out['info'] = copy.deepcopy(data_dict['info'])
        if 'sample_rate' not in out and 'sample_rate_hz' in data_dict:
            out['sample_rate'] = data_dict['sample_rate_hz']

    source_tone_metadata = _extract_tone_metadata(grouped)
    if data_dict is not None:
        source_tone_metadata.update(_extract_tone_metadata(data_dict))
    mod_indices = _clean_indices(grouped.get('modulated_indices'), n_tones)
    if not mod_indices:
        mod_indices = _clean_indices(grouped.get('mod_indices'), n_tones)
    if not mod_indices:
        mod_indices = _modulated_indices_from_offsets(offsets)
    tone_metadata, tone_roles = _annotate_tone_metadata(
        source_tone_metadata, n_tones, mod_indices)
    out['tone_metadata'] = tone_metadata
    for key, value in tone_roles.items():
        out[key] = value
    if 'info' not in out or not isinstance(out['info'], dict):
        out['info'] = {}
    tones_info = out['info'].get('tones')
    if not isinstance(tones_info, dict):
        tones_info = {}
    tones_info.update(copy.deepcopy(tone_metadata))
    out['info']['tones'] = tones_info

    # Map each grouped (cycle, point, dwell) slot back to its original sample
    # position and stamp the cycle index / point / presence onto the grid.
    meta_valid = (sample_index >= 0) & (sample_index < n_samples_out)
    if np.any(meta_valid):
        cycle_grid = np.broadcast_to(
            np.arange(n_cycles, dtype=int)[:, None, None], sample_index.shape)
        point_default = np.broadcast_to(
            np.arange(1, N + 1, dtype=int)[None, :, None], sample_index.shape)
        point_grid = np.asarray(grouped.get('modulation_point', point_default))
        present_grid = np.asarray(grouped.get(
            'sample_present', np.ones(sample_index.shape, dtype=bool)))
        idx = sample_index[meta_valid]
        out['cycle_index'][idx] = cycle_grid[meta_valid]
        out['modulation_point'][idx] = point_grid[meta_valid]
        out['sample_present'][idx] = present_grid[meta_valid]
        out['sample_used_in_cycle'][idx] = True

    # Scatter the grouped per-sample counters/flags onto the output sample grid.
    for out_key, grouped_key, default, dtype in (
            ('packet_counter', 'packet_counter', -1, np.int64),
            ('telescope_time', 'telescope_time', 0, np.uint64),
            ('packet_error', 'packet_error', 0, int),
            ('packet_missing', 'packet_missing', False, bool),
            ('modulation_revision', 'modulation_revision', 0, int)):
        if grouped_key not in grouped:
            continue
        arr = np.asarray(grouped[grouped_key])
        sample_arr = np.full(n_samples_out, default, dtype=dtype)
        valid = meta_valid & (arr.shape == sample_index.shape)
        if np.any(valid):
            sample_arr[sample_index[valid]] = arr[valid]
        out[out_key] = sample_arr

    if grouped.get('stream_flags'):
        out_flags = {}
        for name, arr in grouped['stream_flags'].items():
            arr = np.asarray(arr)
            sample_arr = np.zeros(n_samples_out, dtype=arr.dtype)
            valid = meta_valid & (arr.shape == sample_index.shape)
            if np.any(valid):
                sample_arr[sample_index[valid]] = arr[valid]
            out_flags[name] = sample_arr
        out['stream_flags'] = out_flags

    raw_z = None
    raw_points = None
    raw_settling = None
    raw_cycle_index = None
    raw_point_index = None
    if data_dict is not None:
        raw_points = np.asarray(data_dict.get(
            'modulation_point', np.zeros(n_samples_out)), dtype=int)
        raw_settling = np.asarray(data_dict.get(
            'modulation_settling', np.zeros(n_samples_out)), dtype=int)
        n_raw = min(n_samples_out, len(raw_points), len(raw_settling))
        out['modulation_point'][:n_raw] = raw_points[:n_raw]
        out['modulation_settling'][:n_raw] = raw_settling[:n_raw]
        out['sample_present'][:n_raw] = True

        for out_key, data_key, default, dtype in (
                ('packet_counter', 'packet_counter', -1, np.int64),
                ('telescope_time', 'telescope_time', 0, np.uint64),
                ('packet_error', 'packet_error', 0, int),
                ('modulation_revision', 'modulation_revision', 0, int)):
            if data_key not in data_dict:
                continue
            values = np.asarray(data_dict[data_key])
            sample_arr = np.full(n_samples_out, default, dtype=dtype)
            n_val = min(n_samples_out, len(values))
            sample_arr[:n_val] = values[:n_val]
            out[out_key] = sample_arr

        if data_dict.get('stream_flags'):
            out_flags = {}
            for name, values in data_dict['stream_flags'].items():
                values = np.asarray(values)
                sample_arr = np.zeros(n_samples_out, dtype=values.dtype)
                n_val = min(n_samples_out, len(values))
                sample_arr[:n_val] = values[:n_val]
                out_flags[name] = sample_arr
            out['stream_flags'] = out_flags

        if fill_mode == 'raw':
            # 'raw' fill: also demodulate samples that group_cycles dropped
            # (e.g. settling). Extend each retained point's [start, stop] run
            # outward over adjacent same-point samples so those dropped samples
            # inherit the completed cycle's reference/slope below.
            raw_z = _tone_iq(data_dict, n_tones)[:n_samples_out]
            raw_cycle_index = out['cycle_index'].copy()
            raw_point_index = out['modulation_point'].astype(int) - 1
            for c in range(n_cycles):
                for p in range(N):
                    kept = sample_index[c, p]
                    kept = kept[(kept >= 0) & (kept < n_samples_out)]
                    if kept.size == 0:
                        continue
                    start = int(np.min(kept))
                    stop = int(np.max(kept))
                    while start > 0 and raw_points[start - 1] == p + 1:
                        start -= 1
                    while stop + 1 < n_raw and raw_points[stop + 1] == p + 1:
                        stop += 1
                    raw_cycle_index[start:stop + 1] = c
                    raw_point_index[start:stop + 1] = p
            assigned = (raw_cycle_index >= 0) & (raw_point_index >= 0)
            out['cycle_index'][assigned] = raw_cycle_index[assigned]

    # Samples used to estimate the per-point reference and phase slope/curvature.
    # By default settling transients are excluded from those fits.
    fit_sample_mask = np.ones((n_cycles, N, n_used), dtype=bool)
    grouped_settling = grouped.get('modulation_settling')
    if not use_settling_for_fit and grouped_settling is not None:
        grouped_settling = np.asarray(grouped_settling)
        if grouped_settling.shape == fit_sample_mask.shape:
            fit_sample_mask &= grouped_settling == 0

    ref_input = None
    if reference_z is not None:
        ref_input = np.asarray(reference_z, dtype=complex)
        if ref_input.ndim == 1:
            ref_input = ref_input[:, None]
        if ref_input.shape != (N, n_tones):
            raise ValueError(
                f'reference_z must have shape {(N, n_tones)}, got {ref_input.shape}')

    point_reference_z = np.full((N, n_tones), np.nan + 1j * np.nan, dtype=complex)
    point_gradient = np.full((N, n_tones), np.nan + 1j * np.nan, dtype=complex)
    center_reference_z = np.full(n_tones, np.nan + 1j * np.nan, dtype=complex)
    center_offset_hz = np.full(n_tones, np.nan)
    center_point_index = np.zeros(n_tones, dtype=int)
    dissipation_reference_frequency_hz = np.full(n_tones, np.nan)
    reference_center_detuning = np.full(n_tones, np.nan)
    reference_resonance = np.full(n_tones, np.nan)
    reference_dissipation = np.full(n_tones, np.nan)
    conversions = [None] * n_tones
    bases = ['raw'] * n_tones

    tone_frequencies_hz = None
    info = out.get('info')
    if isinstance(info, dict):
        tones_info = info.get('tones', {})
        if isinstance(tones_info, dict) and tones_info.get('frequencies_hz') is not None:
            tone_frequencies_hz = np.asarray(tones_info.get('frequencies_hz'), dtype=float)
    gain = _record_gain(calibration, info)

    def _center_frequency_hz(t, c_point):
        """The modulation centre (Hz): the grouped probe frequency less its
        offset, else the capture's recorded tone frequency."""
        candidates = []
        if tone_frequency_hz is not None and tone_frequency_hz.shape == (N, n_tones):
            candidates.append(tone_frequency_hz[c_point, t] - offsets[c_point, t])
        if tone_frequencies_hz is not None and t < len(tone_frequencies_hz):
            candidates.append(tone_frequencies_hz[t])
        for candidate in candidates:
            try:
                candidate = float(candidate)
            except (TypeError, ValueError):
                continue
            if np.isfinite(candidate) and candidate != 0.0:
                return candidate
        return np.nan

    # Per tone: build the working basis, its per-point reference, the local
    # phase slope/curvature, then turn every sample into a shift estimate.
    for t in range(n_tones):
        f = offsets[:, t]
        zt = z[:, :, :, t]
        cal = _tone_calibration(calibration, t)
        centred = cal is not None and tone_frequency_hz is not None
        conv = _check_conversion(conversion, centred, t)
        conversions[t] = conv

        # Working basis: de-embedded and circle-centred at each point's probe
        # frequency when calibrated, otherwise the raw I/Q as captured.
        if centred:
            bases[t] = 'centered_rotated'
            z_basis = np.full_like(zt, np.nan + 1j * np.nan, dtype=complex)
            for p in range(N):
                z_basis[:, p, :] = to_basis(zt[:, p, :], cal, tone_frequency_hz[p, t],
                                            gain=gain)
        else:
            z_basis = zt

        # Per-point reference = external value, or the capture mean (settling
        # samples excluded), falling back to the all-sample mean where empty.
        z_fit_basis = np.where(fit_sample_mask, z_basis, np.nan + 1j * np.nan)
        ref = _nanmean_quiet(z_fit_basis, axis=(0, 2))
        ref_all = _nanmean_quiet(z_basis, axis=(0, 2))
        ref = np.where(np.isfinite(ref), ref, ref_all)
        if ref_input is not None and np.all(np.isfinite(ref_input[:, t])):
            ref = ref_input[:, t]
        point_reference_z[:, t] = ref

        # Locate the centre probe point (a zero offset, else the nearest one).
        zero = np.isclose(f, 0.0)
        if np.any(zero):
            ref_center = _nanmean_quiet(ref[zero])
            c_offset = 0.0
            c_point = int(np.flatnonzero(zero)[0])
        else:
            c_point = int(np.nanargmin(np.abs(f))) if len(f) else 0
            ref_center = ref[c_point]
            c_offset = float(f[c_point])
        center_reference_z[t] = ref_center
        center_offset_hz[t] = c_offset
        center_point_index[t] = c_point
        center_hz = _center_frequency_hz(t, c_point)
        dissipation_reference_frequency_hz[t] = center_hz

        # Per-cycle phase slope and curvature at the centre, and the local
        # slope and complex gradient at every point, from the point means.
        point_z = _nanmean_quiet(z_fit_basis, axis=2)      # (n_cycles, N)
        point_z_all = _nanmean_quiet(z_basis, axis=2)
        point_z = np.where(np.isfinite(point_z), point_z, point_z_all)
        derivatives = phase_derivatives(point_z[:, :, None], f[:, None],
                                        method=kernel, per_point=True)
        slope_center = derivatives['dphi_df'][:, 0]
        curvature = derivatives['d2phi_df2'][:, 0]
        local_slope = derivatives['local_dphi_df'][:, :, 0]
        local_slope = np.where(np.isfinite(local_slope), local_slope, slope_center[:, None])
        gradient = derivatives['dz_df'][:, :, 0]
        point_gradient[:, t] = _nanmean_quiet(gradient, axis=0)

        # Where the resonance is, per cycle: the calibration's inversion of the
        # centred points, whose scale holds for driven resonators too (the
        # curvature/slope form shares its zero but reads ~2x there).
        if centred:
            linewidth = cal.fr / cal.Ql
            center_lw = mobius_center_detuning(point_z, f, cal) / linewidth
        else:
            linewidth = np.nan
            center_lw = np.full(n_cycles, np.nan)
        if centred:
            reference_resonance[t] = _reference_resonance_hz(
                cal, ref_center, center_hz + c_offset)
            reference_center_detuning[t] = reference_resonance[t] - center_hz
            # The reference's absolute loss, Delta(1/(2 Qi)) from the
            # calibration's Qi: what joins revisions in demodulate_stream.
            reference_dissipation[t] = cal.convert_centered_iq(ref_center)[1]

        # z_demod: rotate/scale every sample onto the centre-point reference, so
        # each probe point reads as if measured at the centre.
        with np.errstate(divide='ignore', invalid='ignore'):
            z_demod_group = z_basis / ref[None, :, None] * ref_center

        shift_group, dissipation_group = _convert_points(
            conv, cal, z_basis, ref[None, :, None], local_slope[:, :, None],
            gradient[:, :, None], center_hz)

        # Scatter this tone's grouped per-sample results onto the output grid.
        valid = meta_valid & np.isfinite(z_basis)
        if not np.any(valid):
            continue

        idx = sample_index[valid]
        out['z_demod'][idx, t] = z_demod_group[valid]
        out['resonance_shift_hz'][idx, t] = shift_group[valid]
        out['matched_dissipation_shift'][idx, t] = dissipation_group[valid]
        out['resonance_frequency_hz'][idx, t] = (
            reference_resonance[t] + shift_group[valid] if centred else np.nan)

        offset_grid = np.broadcast_to(f[None, :, None], sample_index.shape)
        local_grid = np.broadcast_to(local_slope[:, :, None], sample_index.shape)
        center_grid = np.broadcast_to(slope_center[:, None, None], sample_index.shape)
        curv_grid = np.broadcast_to(curvature[:, None, None], sample_index.shape)
        lw_grid = np.broadcast_to(center_lw[:, None, None], sample_index.shape)
        out['modulation_offset_hz'][idx, t] = offset_grid[valid]
        out['dphi_df'][idx, t] = local_grid[valid]
        out['center_dphi_df'][idx, t] = center_grid[valid]
        out['d2phi_df2'][idx, t] = curv_grid[valid]
        out['center_detuning_linewidths'][idx, t] = lw_grid[valid]
        out['center_detuning_hz'][idx, t] = lw_grid[valid] * linewidth
        with np.errstate(invalid='ignore'):
            out['needs_update'][idx, t] = np.abs(lw_grid[valid]) > threshold_linewidths

        # 'raw' fill: demodulate the dropped samples (those not retained in a
        # cycle) using their owning cycle's reference and slope, computed above.
        if raw_z is not None:
            for p in range(N):
                sample_mask = (
                    (raw_point_index == p)
                    & (raw_cycle_index >= 0)
                    & (raw_cycle_index < n_cycles)
                    & ~out['sample_used_in_cycle']
                )
                if not np.any(sample_mask):
                    continue
                sample_idx = np.flatnonzero(sample_mask)
                cycle_idx = raw_cycle_index[sample_idx]
                raw_values = raw_z[sample_idx, t]
                if centred:
                    sample_basis = to_basis(raw_values, cal, tone_frequency_hz[p, t])
                else:
                    sample_basis = raw_values
                finite = np.isfinite(sample_basis)
                if not np.any(finite):
                    continue
                sample_idx = sample_idx[finite]
                cycle_idx = cycle_idx[finite]
                sample_basis = sample_basis[finite]

                with np.errstate(divide='ignore', invalid='ignore'):
                    out['z_demod'][sample_idx, t] = sample_basis / ref[p] * ref_center
                shift, dissipation = _convert_points(
                    conv, cal, sample_basis, ref[p], local_slope[cycle_idx, p],
                    gradient[cycle_idx, p], center_hz)
                out['resonance_shift_hz'][sample_idx, t] = shift
                out['matched_dissipation_shift'][sample_idx, t] = dissipation
                if centred:
                    out['resonance_frequency_hz'][sample_idx, t] = reference_resonance[t] + shift
                out['modulation_offset_hz'][sample_idx, t] = f[p]
                out['dphi_df'][sample_idx, t] = local_slope[cycle_idx, p]
                out['center_dphi_df'][sample_idx, t] = slope_center[cycle_idx]
                out['d2phi_df2'][sample_idx, t] = curvature[cycle_idx]
                out['center_detuning_linewidths'][sample_idx, t] = center_lw[cycle_idx]
                out['center_detuning_hz'][sample_idx, t] = center_lw[cycle_idx] * linewidth
                with np.errstate(invalid='ignore'):
                    out['needs_update'][sample_idx, t] = (
                        np.abs(center_lw[cycle_idx]) > threshold_linewidths)

    # 'interpolate' fill: bridge settling-sample gaps by linear interpolation
    # between the surrounding non-settling demodulated samples (flags untouched).
    if fill_mode == 'interpolate':
        settling_targets = (
            (out['modulation_settling'] == 1)
            & np.asarray(out['sample_present'], dtype=bool)
        )
        source = (
            (out['modulation_settling'] == 0)
            & np.asarray(out['sample_present'], dtype=bool)
        )
        out['z_demod'] = (
            _interp_nan_targets(out['z_demod'].real, settling_targets, source)
            + 1j * _interp_nan_targets(out['z_demod'].imag, settling_targets, source)
        )
        for key in (
                'resonance_shift_hz',
                'matched_dissipation_shift',
                'resonance_frequency_hz',
                'dphi_df',
                'center_dphi_df',
                'd2phi_df2',
                'center_detuning_hz',
                'center_detuning_linewidths'):
            out[key] = _interp_nan_targets(out[key], settling_targets, source)
        with np.errstate(invalid='ignore'):
            out['needs_update'][settling_targets] = (
                np.abs(out['center_detuning_linewidths'][settling_targets])
                > threshold_linewidths)

    # Derived magnitude/phase plus parsed-sample-compatible I/Q so the result
    # drops into the same plotting/analysis paths as a normal timestream.
    out['mag'] = np.abs(out['z_demod'])
    out['phase_rad'] = np.angle(out['z_demod'])
    out['phase_unwrapped_rad'] = _unwrap_nan_segments(out['phase_rad'])
    out['i_data'] = {
        f'{t:04d}': out['z_demod'][:, t].real
        for t in range(n_tones)
    }
    out['q_data'] = {
        f'{t:04d}': out['z_demod'][:, t].imag
        for t in range(n_tones)
    }
    out['modulation_demodulated'] = True
    out['conversion'] = conversions
    out['basis'] = bases
    out['point_reference_z'] = point_reference_z
    out['point_gradient'] = point_gradient
    out['center_reference_z'] = center_reference_z
    out['point_reference_phase_rad'] = np.angle(point_reference_z)
    out['center_reference_phase_rad'] = np.angle(center_reference_z)
    out['center_offset_hz'] = center_offset_hz
    out['center_point_index'] = center_point_index
    out['reference_center_detuning_hz'] = reference_center_detuning
    out['reference_resonance_frequency_hz'] = reference_resonance
    out['reference_matched_dissipation'] = reference_dissipation
    out['dissipation_reference_frequency_hz'] = dissipation_reference_frequency_hz
    return out


def demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None,
               calibration=None, conversion=None, phase_baseline=None,
               threshold_linewidths=0.1):
    """
    Demodulate grouped modulation cycles into per-tone, per-cycle quantities.

    Each cycle's probe points give the phase slope ``dphi/df`` and curvature
    ``d2phi/df2`` at the centre (``resonator.phase_derivatives``) and the
    **centre detuning**: the resonance -- its operating point, the phase
    inflection (``resonator.operating_point_hz``, which :func:`params_from_sweep`
    centres on) -- minus the modulation centre. For a calibrated tone it is the
    calibration's exact inversion of the centred points
    (``resonator.mobius_center_detuning``), right in scale for driven
    resonators too, as the tracker's default estimator is. The curvature form
    ``phi'' / (2 phi' |phi'|)`` (``resonator.detuning_from_derivatives``)
    shares its zero but is exact only for a linear notch (it read ~2x on tones
    driven to anl ~ 0.3); it is what a raw tone gets, with ``linewidth_hz``.
    The centre point's I/Q gives the resonance shift and dissipation of each
    cycle.

    Two bases:

    - **Calibrated / centred** (``calibration`` supplied, the recommended
      one): each probe point is de-embedded (cable delay removed at its
      absolute frequency) and circle-centred, where the phase is well
      conditioned and the calibration inverts each point exactly.
    - **Raw** (no ``calibration``): the phase about the origin, not the circle
      centre, reads the detuning 13x to 25x high near resonance (worse for
      very shallow dips): only its sign and rough zero mean anything, so it
      needs an explicit ``linewidth_hz`` as an opt-in, else NaN. The resonance
      shift and dissipation still come out right to first order, from the
      tangent/normal projection on the live I/Q gradient.

    Parameters
    ----------
    grouped : dict
        Output of :func:`group_cycles` (uses ``z``; ``offsets_hz`` /
        ``tone_frequency_hz`` / ``revision``). ``z`` may be
        ``(n_cycles, N, n_tones)`` (mean-reduced) or
        ``(n_cycles, N, n_used, n_tones)`` (the sample axis is averaged here).
    offsets : numpy.ndarray or None, optional
        ``(N, n_tones)`` per-point probe offsets (Hz). ``None`` uses
        ``grouped['offsets_hz']``.
    method : {'fast', 'accurate', 'model'}, optional
        Slope/curvature estimator. ``'fast'`` = finite differences between the
        lowest, most central and highest offsets (points sharing an offset are
        averaged coherently; any slot order) -- the tracker's; ``'accurate'``
        (= ``'model'``) = quadratic fit of phase against offset.
    linewidth_hz : float or array-like or None, optional
        Per-tone linewidth (Hz) for the *raw-basis* detuning. Ignored when a
        ``calibration`` is given (it supplies ``fr / Ql``).
    calibration : dict or None, optional
        Per-tone calibrations (``{tone_index: ResonatorCalibration}``, fits or a
        record; e.g. ``params_from_sweep(...)['calibration']``): chooses the
        centred basis.
    conversion : {'phase', 'linear', 'mobius'} or None, optional
        Per-cycle shift/dissipation estimator; see
        :func:`demodulate_timestream`. Default ``'phase'`` when calibrated,
        else ``'linear'``.
    phase_baseline : array-like or None, optional
        Per-tone reference phase (rad) of the centre point, in the working
        basis; ``None`` uses the mean centre point over the cycles.
    threshold_linewidths : float, optional
        ``needs_update`` trips when ``|center_detuning_linewidths|`` exceeds
        this (default 0.1).

    Returns
    -------
    dict
        Per-tone, per-cycle arrays of shape ``(n_cycles, n_tones)``:
        ``z_center_point`` (the centre point's raw I/Q), ``dphi_df``,
        ``d2phi_df2``, ``center_detuning_linewidths``, ``center_detuning_hz``,
        ``needs_update`` (bool), ``resonance_shift_hz`` and
        ``matched_dissipation_shift`` (relative to the mean cycle), and
        ``resonance_frequency_hz`` (calibrated tones only); plus ``revision``
        ``(n_cycles,)`` and cycle-aligned metadata copied from
        :func:`group_cycles` when available: ``packet_counter``,
        ``telescope_time``, ``packet_error``, ``packet_missing``,
        ``sample_index``, ``stream_flags``.
    """
    z = np.asarray(grouped['z'])
    if z.ndim == 4:                      # (n_cycles, N, n_used, n_tones) -> average dwell
        z = z.mean(axis=2)
    if z.ndim != 3:
        raise ValueError(f'grouped["z"] must be 3D or 4D, got {z.ndim}D')
    n_cycles, N, n_tones = z.shape
    kernel = 'fast' if method == 'fast' else 'fit'

    if offsets is None:
        offsets = grouped['offsets_hz']
    offsets = np.broadcast_to(np.asarray(offsets, dtype=float).reshape(N, -1),
                              (N, n_tones))
    tone_frequency_hz = grouped.get('tone_frequency_hz')   # absolute probe freqs

    lw = None if linewidth_hz is None else np.broadcast_to(
        np.asarray(linewidth_hz, dtype=float), (n_tones,))

    shape = (n_cycles, n_tones)
    out = {
        'z_center_point': np.zeros(shape, dtype=complex),
        'dphi_df': np.full(shape, np.nan),
        'd2phi_df2': np.full(shape, np.nan),
        'resonance_shift_hz': np.full(shape, np.nan),
        'matched_dissipation_shift': np.full(shape, np.nan),
        'resonance_frequency_hz': np.full(shape, np.nan),
        'center_detuning_linewidths': np.full(shape, np.nan),
        'center_detuning_hz': np.full(shape, np.nan),
        'needs_update': np.zeros(shape, dtype=bool),
        'revision': np.asarray(grouped.get('revision', np.zeros(n_cycles)), dtype=int),
    }
    passthrough = {
        'packet_counter': 'cycle_packet_counter',
        'telescope_time': 'cycle_telescope_time',
        'packet_error': 'cycle_packet_error',
        'packet_missing': 'cycle_packet_missing',
        'sample_present': 'cycle_sample_present',
        'sample_index': 'cycle_sample_index',
        'stream_flags': 'cycle_stream_flags',
    }
    for out_key, grouped_key in passthrough.items():
        if grouped_key in grouped:
            out[out_key] = grouped[grouped_key]
    if 'cycle_point_index' in grouped:
        out['cycle_point_index'] = grouped['cycle_point_index']
    if n_cycles == 0:
        return out
    gain = _record_gain(calibration, grouped.get('info'))

    for t in range(n_tones):
        f = offsets[:, t]                        # (N,) probe offsets for this tone
        zt = z[:, :, t]                          # (n_cycles, N)
        ci = int(np.argmin(np.abs(f)))           # the point nearest the centre
        out['z_center_point'][:, t] = zt[:, ci]

        cal = _tone_calibration(calibration, t)
        centred = cal is not None and tone_frequency_hz is not None
        conv = _check_conversion(conversion, centred, t)
        if centred:
            # Calibrated basis: de-embed + circle-centre each point at its own
            # probe frequency; the phase is then well conditioned.
            probe = np.asarray(tone_frequency_hz, dtype=float)[:, t]
            zb = to_basis(zt, cal, probe[None, :], gain=gain)   # (n_cycles, N)
            linewidth = cal.fr / cal.Ql
            center_hz = probe[ci] - f[ci]
        else:
            zb = zt
            linewidth = np.nan if lw is None else lw[t]
            center_hz = np.nan
            if tone_frequency_hz is not None:
                center_hz = float(np.asarray(tone_frequency_hz, dtype=float)[ci, t] - f[ci])

        derivatives = phase_derivatives(zb[:, :, None], f[:, None], method=kernel,
                                        per_point=True)
        slope = derivatives['dphi_df'][:, 0]
        curvature = derivatives['d2phi_df2'][:, 0]
        out['dphi_df'][:, t] = slope
        out['d2phi_df2'][:, t] = curvature

        # Resonance minus centre: the calibration's inversion for a calibrated
        # tone (right in scale for driven resonators too); the curvature/slope
        # form on the raw phase only with an explicit linewidth.
        center_lw = None
        if centred:
            center_lw = mobius_center_detuning(zb, f, cal) / linewidth
        elif N >= 3 and np.isfinite(linewidth):
            center_lw = detuning_from_derivatives(slope, curvature)
        if center_lw is not None:
            out['center_detuning_linewidths'][:, t] = center_lw
            out['center_detuning_hz'][:, t] = center_lw * linewidth
            with np.errstate(invalid='ignore'):
                out['needs_update'][:, t] = np.abs(center_lw) > threshold_linewidths

        # The centre point's shift and dissipation about the mean cycle (or
        # the supplied baseline phase).
        reference = _nanmean_quiet(zb[:, ci])
        if phase_baseline is not None:
            reference = np.abs(reference) * np.exp(
                1j * np.asarray(phase_baseline, dtype=float)[t])
        local_slope = derivatives['local_dphi_df'][:, ci, 0]
        local_slope = np.where(np.isfinite(local_slope), local_slope, slope)
        shift, dissipation = _convert_points(
            conv, cal, zb[:, ci], reference, local_slope,
            derivatives['dz_df'][:, ci, 0], center_hz)
        out['resonance_shift_hz'][:, t] = shift
        out['matched_dissipation_shift'][:, t] = dissipation
        if centred:
            out['resonance_frequency_hz'][:, t] = shift + _reference_resonance_hz(
                cal, reference, center_hz + f[ci])

    return out


# ---------------------------------------------------------------------------
# Long recordings: demodulate across modulation revisions
# ---------------------------------------------------------------------------

# Per-cycle fields held across each cycle's samples (and spectra at the cycle
# rate, not the sample rate).
CYCLE_FIELDS = ('center_detuning_hz', 'center_detuning_linewidths', 'dphi_df',
                'd2phi_df2', 'curvature_over_slope')


class StreamDemodulator:
    """Demodulate a modulated stream across its revisions, on one reference.

    Tracking commits and recentres change the modulation state mid-stream;
    each change is a revision stamped into every frame and announced by a
    ``TONE_UPDATE`` record. This splits the stream at those revisions,
    demodulates each piece with its own state (offsets, readout-correction
    factors) and joins them onto **one physical reference**, so a probe move
    never looks like a resonator move:

    - a calibrated tone is joined through its absolute resonance frequency,
      ``f_res = f_centre + d_centre`` (the reconstruction identity), with the
      centre detuning of each revision's reference from the calibration's
      inversion (exact, unlike the finite-difference detuning), and its
      dissipation through the same inversion's absolute loss. Nothing is
      matched across a join: a real step at a commit stays, and so does a
      calibration error (as a step proportional to the move);
    - an uncalibrated tone carries its per-point reference I/Q across each
      move along the measured I/Q gradient (first order in the move, which
      tracking keeps small).

    The first usable per-point reference of each revision is held fixed, so
    repeated calls on a rolling buffer (the live viewer) stay consistent. Keep
    one instance for one stream. :func:`demodulate_stream` is the one-shot
    form for a saved recording.
    """

    def __init__(self, method='fast', settling='none', linewidth_hz=None,
                 calibration=None, conversion=None, gaps='raise'):
        if gaps not in GAP_MODES:
            raise ValueError(f'gaps must be one of {GAP_MODES}')
        self.gaps = gaps
        if method not in ('fast', 'accurate'):
            raise ValueError('demod_method must be fast or accurate')
        if settling not in ('none', 'raw'):
            raise ValueError('settling must be none or raw')
        if linewidth_hz is not None and (
                not np.all(np.isfinite(linewidth_hz)) or np.any(np.asarray(linewidth_hz) <= 0)):
            raise ValueError('linewidth_hz must be positive and finite')
        self.method = method
        self.settling = settling
        self.linewidth_hz = linewidth_hz
        self.calibration = calibration
        self.conversion = conversion
        self.revision = None
        self.signature = None
        # Per epoch and tone: (reference_z (N,), gradient (N,), probe Hz (N,)).
        self._epoch_references = {}
        self._last_reference = {}           # tone -> the latest epoch's entry
        self._baseline_resonance = {}       # tone -> f_res of the first reference
        self._baseline_dissipation = {}     # tone -> its absolute loss

    @property
    def references(self):
        """The latest per-point reference I/Q of each tone (working basis)."""
        return {tone: entry[0] for tone, entry in self._last_reference.items()}

    def __call__(self, samples, tones=None):
        if samples.get('modulation_demodulated'):
            return samples
        # The calibration given, else the one the stream carries.
        source_calibration = (self.calibration if self.calibration is not None
                              else calibration_from_data(samples))
        # An RX gain change since the calibration sweep would mis-centre the
        # I/Q and turn every recentre into a step: divide it out.
        gain_note = ''
        try:
            rx_gain = _record_gain(source_calibration, samples.get('info'))
        except ValueError as exc:          # PFB shift differs: not modelled
            rx_gain, gain_note = 1.0, f'; RX gain NOT matched ({exc})'
        if rx_gain != 1.0:
            gain_note = f'; RX gain {20 * np.log10(rx_gain):+.2f} dB vs calibration divided out'
        keys = sorted(map(int, samples['i_data'])) if tones is None else sorted(set(tones))
        if not keys:
            raise ValueError('Select at least one tone for demodulation')
        samples = _bridge_counter_gaps(samples, keys, self.gaps)
        n = int(samples['num_samples'])
        sample_index = np.asarray(samples.get('sample_index', np.arange(n)), dtype=np.int64)
        revision = np.asarray(samples['modulation_revision'], dtype=int)
        points = np.asarray(samples['modulation_point'], dtype=int)
        initial = active_modulation_state(samples.get('info'))
        events = []
        if initial is not None:
            initial_rev = initial.get('applied_revision', initial.get('revision'))
            if initial_rev is None:
                active_revs = revision[points > 0]
                initial_rev = int(active_revs[0]) if len(active_revs) else None
            events.append((-(1 << 62), None if initial_rev is None else
                           int(initial_rev) & 0x7fff, initial,
                           ('initial', initial_rev, repr(initial))))
        for record in samples.get('tone_updates', ()):
            payload = record.get('payload', {})
            if payload.get('type') not in ('SNAPSHOT', 'TONE_UPDATE'):
                continue
            if payload.get('op') == 'tracking_dry_run':
                continue
            state = payload.get('modulation_state')
            rev = payload.get('revision')
            if state is None:
                if payload.get('type') == 'SNAPSHOT':
                    # Legacy snapshots carry only a compact summary. Preserve
                    # the initial metadata for frames tagged with its revision.
                    continue
                if payload.get('op') != 'disable':
                    # Old update records lack the offsets/RX correction needed
                    # to reconstruct the next revision.
                    continue
            events.append((int(record['sample_index']),
                           None if rev is None else int(rev) & 0x7fff,
                           state, ('update', int(record['sample_index']), rev)))
        events.sort(key=lambda item: item[0])
        if not events:
            raise ValueError('Demodulation requires modulation state in stream metadata')

        fields = ('resonance_shift_hz', 'matched_dissipation_shift',
                  'resonance_frequency_hz', *CYCLE_FIELDS)
        merged = {field: np.full((n, len(keys)), np.nan) for field in fields}
        iq = {field: np.full((n, len(keys)), np.nan) for field in ('i_data', 'q_data')}
        z_demod = np.full((n, len(keys)), np.nan + 1j * np.nan)
        spectra_parts = []
        rates = []
        counts = 0
        covered = np.zeros(n, dtype=bool)
        for event_index, (begin, event_rev, state, epoch) in enumerate(events):
            if state is None:
                continue
            # Old-revision frames may still be queued after an announcement,
            # and new-revision frames arrive before it (~210 samples, 84 ms, in
            # the 2026-09-29 overnight run: the record is written after the
            # commit). The tag on each IQ frame chooses the state, so an epoch
            # opens at the previous announcement and only a later announcement
            # reusing this same revision ID ends it.
            start = events[event_index - 1][0] if event_index else begin
            stop = next((later[0] for later in events[event_index + 1:]
                         if later[1] == event_rev), 1 << 62)
            mask = ((sample_index >= start) & (sample_index < stop)
                    & (points > 0))
            if event_rev is not None:
                mask &= revision == event_rev
            if not mask.any():
                continue
            covered |= mask
            global_idx = np.flatnonzero(mask)
            lo, hi = int(global_idx[0]), int(global_idx[-1]) + 1
            local_idx = global_idx - lo
            selected = {
                key: value[lo:hi] if isinstance(value, np.ndarray)
                and value.shape[:1] == (n,) else value
                for key, value in samples.items()}
            selected['num_samples'] = hi - lo
            selected['num_tones'] = len(keys)
            selected['modulation_point'] = np.where(mask[lo:hi], points[lo:hi], 0)
            for field in ('i_data', 'q_data'):
                by_id = {int(k): v for k, v in samples[field].items()}
                selected[field] = {f'{i:04d}': by_id[t][lo:hi] / rx_gain
                                   for i, t in enumerate(keys)}
            if 'stream_flags' in samples:
                selected['stream_flags'] = {
                    key: value[lo:hi]
                    for key, value in samples['stream_flags'].items()}
            local_state = copy.deepcopy(state)
            by_id = {int(t['index']): t for t in local_state['tones']}
            # Each point's applied probe frequency, for carrying references.
            probes = {tone: float(by_id[tone]['center_hz'])
                      + np.asarray(by_id[tone]['offsets_hz'], dtype=float)
                      for tone in keys if by_id[tone].get('center_hz') is not None}
            local_state['tones'] = [dict(by_id[t], index=i)
                                    for i, t in enumerate(keys)]
            for field in ('mod_indices', 'modulated_indices'):
                if field in local_state:
                    selected_ids = set(map(int, local_state[field]))
                    local_state[field] = [i for i, t in enumerate(keys)
                                          if t in selected_ids]
            tone_info = samples['info'].get('tones', {})
            selected['info'] = {'modulation': local_state, 'tones': {}}
            if 'frequencies_hz' in tone_info:
                selected['info']['tones']['frequencies_hz'] = (
                    np.asarray(tone_info['frequencies_hz'])[keys])
            for field in ('blind_indices', 'regular_indices'):
                if field in tone_info:
                    selected['info']['tones'][field] = [
                        i for i, t in enumerate(keys) if t in tone_info[field]]
            calibration = {}
            if source_calibration is not None:
                calibration = {i: _tone_calibration(source_calibration, t)
                               for i, t in enumerate(keys)}
                calibration = {i: cal for i, cal in calibration.items() if cal is not None}
            linewidth = self.linewidth_hz
            if linewidth is not None and np.ndim(linewidth):
                linewidth = np.asarray(linewidth)[keys]

            # This epoch's per-point references: held from its first
            # demodulation; for a new epoch of an uncalibrated tone, carried
            # from the previous epoch along the I/Q gradient.
            references = self._epoch_references.setdefault(epoch, {})
            N = int(local_state['num_points'])
            reference = np.full((N, len(keys)), np.nan + 1j * np.nan)
            for i, tone in enumerate(keys):
                entry = references.get(tone)
                if entry is None and i not in calibration:
                    previous = self._last_reference.get(tone)
                    if (previous is not None and tone in probes
                            and len(previous[0]) == N == len(probes[tone])):
                        step = probes[tone] - previous[2]      # per point, Hz
                        entry = (previous[0] + previous[1] * step, previous[1],
                                 probes[tone])
                        references[tone] = entry
                if entry is not None:
                    reference[:, i] = entry[0]
            result = self._demodulate(selected, calibration or None, linewidth,
                                      reference)
            if 'point_reference_z' in result:
                for i, tone in enumerate(keys):
                    if tone not in references and np.all(
                            np.isfinite(result['point_reference_z'][:, i])):
                        references[tone] = (
                            result['point_reference_z'][:, i].copy(),
                            result['point_gradient'][:, i].copy(),
                            probes.get(tone, np.full(N, np.nan)))
                    if tone in references:
                        self._last_reference[tone] = references[tone]
                    if tone not in self._baseline_resonance and np.isfinite(
                            result['reference_resonance_frequency_hz'][i]):
                        self._baseline_resonance[tone] = float(
                            result['reference_resonance_frequency_hz'][i])
                        self._baseline_dissipation[tone] = float(
                            result['reference_matched_dissipation'][i])
            for field in ('i_data', 'q_data'):
                for i in range(len(keys)):
                    iq[field][global_idx, i] = result[field][f'{i:04d}'][local_idx]
            for field in fields:
                if field in result:
                    merged[field][global_idx] = result[field][local_idx]
            # Calibrated tones join the revisions through the absolute
            # resonance frequency and the absolute loss, each from the
            # calibration's inversion of the revision's own reference; nothing
            # is matched across the join. Carried references need nothing more.
            for i, tone in enumerate(keys):
                if i in calibration and tone in self._baseline_resonance:
                    merged['resonance_shift_hz'][global_idx, i] = (
                        result['resonance_frequency_hz'][local_idx, i]
                        - self._baseline_resonance[tone])
                    merged['matched_dissipation_shift'][global_idx, i] = (
                        result['matched_dissipation_shift'][local_idx, i]
                        + result['reference_matched_dissipation'][i]
                        - self._baseline_dissipation[tone])
            if 'z_demod' in result:
                z_demod[global_idx] = result['z_demod'][local_idx]
            spectra_parts.append(result['cycle_spectra'])
            rates.append(result['cycle_rate'])
            counts += int(result.get('complete_cycles', 0))

        output = dict(samples)
        for field in ('i_data', 'q_data'):
            output[field] = {f'{tone:04d}': iq[field][:, i]
                             for i, tone in enumerate(keys)}
        for field, values in merged.items():
            output[field] = {f'{tone:04d}': values[:, i]
                             for i, tone in enumerate(keys)}
        output['z_demod'] = z_demod
        output['modulation_demodulated'] = True
        if 'packet_filled' in samples:
            output['packet_missing'] = (np.asarray(samples['packet_missing'], dtype=bool)
                                        | samples['packet_filled'])

        if rates:
            common_rate = all(np.isclose(rate, rates[0]) for rate in rates)
            selected_parts = spectra_parts if common_rate else spectra_parts[-1:]
            output['cycle_rate'] = rates[0] if common_rate else rates[-1]
            spectra = {}
            for field in CYCLE_FIELDS:
                rows = [part[field] for part in selected_parts if field in part]
                if rows:
                    gap = np.full((1, len(keys)), np.nan)
                    combined = np.concatenate(
                        [piece for i, row in enumerate(rows)
                         for piece in ((gap, row) if i else (row,))])
                else:
                    combined = np.empty((0, len(keys)))
                spectra[field] = {f'{tone:04d}': combined[:, i]
                                  for i, tone in enumerate(keys)}
            output['cycle_spectra'] = spectra
        else:
            output['cycle_rate'] = float(samples['sample_rate'])
            output['cycle_spectra'] = {
                field: {f'{tone:04d}': np.empty(0) for tone in keys}
                for field in CYCLE_FIELDS}
        missing = int(np.count_nonzero((points > 0) & ~covered))
        output['demod_note'] = (
            f'Demodulated ({self.method}, settling={self.settling}); '
            f'{len(spectra_parts)} revision segment(s); '
            f'{missing} samples awaiting revision metadata')
        if rates and not all(np.isclose(rate, rates[0]) for rate in rates):
            output['demod_note'] += '; spectra use latest cadence'
        output['demod_note'] += gain_note
        if 'gap_fill' in samples:
            fill = samples['gap_fill']
            output['demod_note'] += (
                f"; {fill['inserted_rows'].size} lost accumulation(s) in "
                f"{len(fill['gaps'])} gap(s) filled ({fill['mode']}), rows in gap_fill")
        if source_calibration is None and self.linewidth_hz is None:
            output['demod_note'] += '; detuning needs calibrations'
        return output

    def _demodulate(self, samples, calibration, linewidth_hz, reference):
        state = active_modulation_state(samples.get('info'))
        if state is None:
            raise ValueError('Demodulation requires modulation state in stream metadata')
        points = np.asarray(samples['modulation_point'])
        n = len(points)
        tones = int(samples['num_tones'])
        N, spp = int(state['num_points']), int(state['samples_per_point'])
        settle = int(state.get('n_settle', 0))
        if N < 2 or spp < 1 or not 0 <= settle < spp:
            raise ValueError('Demodulation requires >=2 probe points and non-settling dwell samples')
        signature = (N, spp, settle)
        revisions = np.asarray(samples['modulation_revision'])
        missing = np.asarray(samples.get('packet_missing', np.zeros(n)), dtype=bool)
        active = (points > 0) & ~missing
        rev = np.unique(revisions[active])
        if len(rev) > 1:
            raise ValueError('A demodulation segment must contain one revision')
        if len(rev):
            expected = state.get('applied_revision', state.get('revision'))
            if expected is not None and (int(expected) & 0x7fff) != rev[0]:
                raise ValueError('Stream modulation revision disagrees with metadata')
            self.revision = int(rev[0])
            self.signature = signature

        # Reject partial dwells/cycles before group_cycles can truncate unequal
        # dwell lengths. Keep original indices, including gaps, for demodulation.
        length = N * spp
        starts = np.flatnonzero((points == 1) & np.r_[True, points[:-1] != 1])
        pattern = np.repeat(np.arange(1, N + 1), spp)
        settling_pattern = np.tile(np.arange(spp) < settle, N)
        flags = np.asarray(samples['modulation_settling'])
        counter = np.asarray(samples['packet_counter'], dtype=np.int64)
        discontinuity = np.asarray(samples.get('packet_discontinuity', np.zeros(n)), dtype=bool)
        complete = np.zeros(n, dtype=bool)
        # One row per candidate cycle: the sample indices it would span.
        window = starts[starts + length <= n, None] + np.arange(length)
        step_ok = np.r_[(np.diff(counter) & 0xffffffff) == 1, False]
        ok = (np.all(points[window] == pattern, axis=1)
              & np.all(flags[window] == settling_pattern, axis=1)
              & ~np.any(missing[window], axis=1)
              & np.all(step_ok[window[:, :-1]], axis=1)
              & ~np.any(discontinuity[window], axis=1))
        complete[window[ok]] = True
        clean = dict(samples)
        clean['modulation_point'] = np.where(complete, points, 0)
        # This state's RX correction, on every dwell sample (settling too).
        if not clean.get('readout_correction_applied', False):
            correction = readout_correction_factors({'modulation': state}, tones)
            if correction is not None:
                z = _tone_iq(samples, tones)
                z = apply_readout_correction(z, points, correction)
                clean['i_data'] = {f'{t:04d}': z[:, t].real for t in range(tones)}
                clean['q_data'] = {f'{t:04d}': z[:, t].imag for t in range(tones)}
                clean['readout_correction_applied'] = True
        # Time continuity is already checked above, including uint32 wrap.
        clean['packet_counter'] = np.arange(n)
        grouped = group_cycles(clean, state, reduce=None, on_missing='notify')
        count = len(grouped['z'])
        if count:
            result = demodulate_timestream(
                grouped, method=self.method, calibration=calibration,
                conversion=self.conversion, reference_z=reference, data_dict=clean,
                fill_settling=self.settling)
            cycle = demodulate(
                grouped, method=self.method, calibration=calibration,
                conversion=self.conversion, linewidth_hz=linewidth_hz)
            slope = cycle['dphi_df']
            cycle['curvature_over_slope'] = np.divide(
                cycle['d2phi_df2'], slope, out=np.full_like(slope, np.nan),
                where=np.isfinite(slope) & (slope != 0))
            # Hold the cycle estimates across their samples for the time
            # display; the spectra below keep the true cycle cadence.
            index = result['cycle_index']
            valid = complete & (index >= 0)
            if self.settling == 'none':
                valid &= flags == 0
            for field in CYCLE_FIELDS:
                result[field] = np.full((n, tones), np.nan)
                result[field][valid] = cycle[field][index[valid]]
            cycle_indices = np.asarray(grouped['cycle_sample_index'], dtype=int)
            positions = np.rint((cycle_indices - cycle_indices[0]) / length).astype(int)
            if np.any(cycle_indices - cycle_indices[0] != positions * length):
                raise ValueError('Modulation cycle indices are not on a regular cadence')
            result['cycle_spectra'] = {}
            for field in CYCLE_FIELDS:
                rows = np.full((positions[-1] + 1, tones), np.nan)
                rows[positions] = cycle[field]
                result['cycle_spectra'][field] = rows
            result['cycle_rate'] = samples['sample_rate'] / length
        else:
            result = dict(samples)
            for field in ('resonance_shift_hz', 'matched_dissipation_shift',
                          'resonance_frequency_hz', *CYCLE_FIELDS):
                result[field] = np.full((n, tones), np.nan)
            result['i_data'] = {key: np.full(n, np.nan) for key in samples['i_data']}
            result['q_data'] = {key: np.full(n, np.nan) for key in samples['q_data']}
            result['cycle_spectra'] = {field: np.empty((0, tones)) for field in CYCLE_FIELDS}
            result['cycle_rate'] = samples['sample_rate'] / length
        for field in ('sample_index', 'time_s', 'telescope_time', 'packet_counter',
                      'packet_missing', 'packet_discontinuity'):
            if field in samples:
                result[field] = samples[field]
        result['modulation_demodulated'] = True
        result['complete_cycles'] = count
        return result


GAP_MODES = ('raise', 'nan', 'interpolate')


def demodulate_stream(samples, tones=None, *, method='accurate', settling='none',
                      linewidth_hz=None, calibration=None, conversion=None,
                      gaps='raise'):
    """Demodulate a whole modulated recording across its revisions.

    The offline form of the live viewer's demodulation
    (:class:`StreamDemodulator`): ``samples`` from ``ReadoutClient.parse_stream``
    or ``stream_file.read_stream_chunk`` (with its tone-update records,
    recorded with ``subscribe_tone_updates``) or a stream basename. Every
    tracking commit or recentre is split out and joined back onto one physical
    reference, so ``resonance_shift_hz`` is the resonator's own motion across
    the whole recording. ``calibration`` defaults to the one the recording
    carries (``info['resonators']``).

    ``gaps`` says what to do with lost accumulations (steps in
    ``packet_counter``). A lost accumulation shifts the modulation cycle by a
    slot, so the cycles cannot simply be read on across it:

    - ``'raise'`` (default): refuse, saying where the gaps are;
    - ``'nan'``: insert a NaN placeholder for each lost accumulation (its
      probe point worked out from the slot cadence), so the output is evenly
      sampled and the cycles touching a gap come out NaN;
    - ``'interpolate'``: as ``'nan'``, then fill each placeholder linearly from
      the received samples at the **same probe point** (never across points),
      so every cycle demodulates. Inserted samples are marked in
      ``packet_missing`` either way.

    Returns the parsed dict (one row per accumulation, lost ones included,
    when gaps are filled) with per-tone fields added (see
    :func:`demodulate_timestream` for their meaning), ``cycle_spectra`` /
    ``cycle_rate`` for the per-cycle estimates and a ``demod_note``.
    """
    if isinstance(samples, (str, os.PathLike)):
        from .client.readout_client import ReadoutClient
        samples = ReadoutClient.parse_stream(samples, apply_readout_correction=False)
    from .resonator import warn_adc_overvoltage
    warn_adc_overvoltage(samples=samples, label='recording')
    return StreamDemodulator(method, settling, linewidth_hz, calibration,
                             conversion, gaps=gaps)(samples, tones)


def _bridge_counter_gaps(samples, tones, gaps):
    """Samples on a contiguous packet-counter axis, per ``gaps`` (see
    :func:`demodulate_stream`); ``samples`` itself when nothing is lost.

    The I/Q of the tones in ``tones`` only are kept. Inserted rows follow the
    :func:`timeseries.fill_gaps` conventions -- ``packet_error`` 1, telescope
    time 0 (unknown), stream flags -1 -- except the modulation tags, which the
    cadence determines, ``sample_index`` (the next received sample's, so
    tone-update records still apply from the right place) and ``time_s``
    (interpolated).
    """
    counter = samples.get('packet_counter')
    if counter is None or len(counter) < 2:
        return samples
    steps = np.diff(np.asarray(counter, dtype=np.int64)) % 2**32
    lost = steps - 1
    if not np.any(lost > 0):
        return samples
    if gaps == 'raise':
        where = np.flatnonzero(lost > 0)
        raise ValueError(
            f'{int(lost.sum())} accumulation(s) lost in {where.size} gap(s), the first '
            f'after sample {int(where[0])}: a lost accumulation shifts the modulation '
            "cycle. Pass gaps='nan' (placeholders; the cycles touching a gap come out "
            "NaN) or gaps='interpolate' (each lost sample filled from the same probe "
            "point's neighbours), or demodulate a gap-free range.")
    state = active_modulation_state(samples.get('info'))
    if state is None:
        raise ValueError('filling gaps needs the modulation state in the metadata')
    n = len(counter)
    pc = int(counter[0]) + np.concatenate(([0], np.cumsum(steps)))
    z = np.column_stack([np.asarray(samples['i_data'][f'{t:04d}'], dtype=float)
                         + 1j * np.asarray(samples['q_data'][f'{t:04d}'], dtype=float)
                         for t in tones])
    rebuilt = _reconstruct_contiguous(
        pc, np.asarray(samples['modulation_point'], dtype=int),
        np.asarray(samples['modulation_settling'], dtype=int),
        np.asarray(samples['modulation_revision'], dtype=int), z,
        int(state['num_points']), int(state.get('samples_per_point', 1)),
        int(state.get('n_settle', 0)))
    if rebuilt is None:
        raise ValueError('cannot fill the gaps: the modulation cadence could not be '
                         'anchored (no two consecutive received samples change point)')
    pc2, points2, settling2, revisions2, z2, present, source = rebuilt
    inserted = ~present
    if gaps == 'interpolate':
        for p in range(1, int(state['num_points']) + 1):
            targets = (points2 == p) & inserted
            if np.any(targets):
                z2 = _interp_nan_targets(z2, targets, (points2 == p) & present)

    def expand(values, fill):
        values = np.asarray(values)
        dtype = np.int64 if (fill == -1 and values.dtype.kind == 'u') else values.dtype
        out = np.full(len(pc2), fill, dtype=dtype)
        out[present] = values[source[present]]
        return out

    out = dict(samples)
    for key, value in samples.items():
        if isinstance(value, np.ndarray) and value.shape[:1] == (n,):
            out[key] = expand(value, False if value.dtype == bool else 0)
    out['i_data'] = {f'{t:04d}': z2[:, i].real for i, t in enumerate(tones)}
    out['q_data'] = {f'{t:04d}': z2[:, i].imag for i, t in enumerate(tones)}
    out['packet_counter'] = pc2 % 2**32
    if 'packet_error' in samples:
        out['packet_error'] = expand(samples['packet_error'], 1)
    if 'stream_flags' in samples:
        out['stream_flags'] = {key: expand(value, -1)
                               for key, value in samples['stream_flags'].items()}
    out['modulation_point'] = points2
    out['modulation_settling'] = settling2
    out['modulation_revision'] = revisions2
    positions = np.flatnonzero(present)
    if 'sample_index' in samples:
        received = np.asarray(samples['sample_index'])
        out['sample_index'] = received[source[positions[
            np.searchsorted(positions, np.arange(len(pc2)))]]]
    if 'time_s' in samples:
        out['time_s'] = np.interp(np.arange(len(pc2)), positions,
                                  np.asarray(samples['time_s'], dtype=float))
    # Interpolated rows are demodulated like received ones; NaN rows are
    # missing. Either way packet_filled marks them, and the demodulator puts
    # them back into packet_missing on its output.
    prior = expand(np.asarray(samples.get('packet_missing', np.zeros(n, dtype=bool)),
                              dtype=bool), False)
    out['packet_missing'] = prior if gaps == 'interpolate' else prior | inserted
    out['packet_filled'] = inserted
    out['num_samples'] = len(pc2)
    out['gap_fill'] = _gap_fill_record(gaps, inserted, z2)
    return out


def _gap_fill_record(mode, inserted, z):
    """What a gap fill did, as row indices of the filled output.

    ``inserted_rows`` are the placeholders; of those, ``interpolated_rows`` got
    values and ``nan_rows`` stayed NaN (no same-point neighbour to interpolate
    from, or ``mode='nan'``). ``gaps`` lists ``(first_inserted_row, n_lost)``.
    """
    rows = np.flatnonzero(inserted)
    finite = np.all(np.isfinite(np.atleast_2d(np.asarray(z).T).T[rows]), axis=-1) \
        if rows.size else np.zeros(0, dtype=bool)
    starts = rows[np.r_[True, np.diff(rows) > 1]] if rows.size else rows
    lengths = np.diff(np.r_[np.flatnonzero(np.r_[True, np.diff(rows) > 1]), rows.size]) \
        if rows.size else rows
    return {'mode': mode, 'inserted_rows': rows,
            'interpolated_rows': rows[finite], 'nan_rows': rows[~finite],
            'gaps': [(int(s), int(n)) for s, n in zip(starts, lengths)]}


def _measured_reference_freqs(f_tone, z_tone):
    """Return measured (model-free) feature frequencies for one raw sweep trace.

    Straight from the sweep, with no model: the magnitude minimum, the peak of
    ``|dS21/df|`` and the peak of the unwrapped-phase slope. These are reported
    for cross-checking the fitted ``fr`` and the chosen operating point; for a
    resonator with cable delay or impedance-mismatch asymmetry they generally
    sit at three different frequencies, none of which is ``fr``.

    Returns ``(f_min_s21, f_max_dz_df, f_max_dphase_df)`` (Hz).
    """
    order = np.argsort(np.asarray(f_tone, dtype=float))
    fs = np.asarray(f_tone, dtype=float)[order]
    zs = np.asarray(z_tone, dtype=complex)[order]
    f_min_s21 = float(fs[int(np.argmin(np.abs(zs)))])
    f_max_dz_df = float(fs[int(np.argmax(np.abs(np.gradient(zs, fs))))])
    phase_slope = np.gradient(np.unwrap(np.angle(zs)), fs)
    f_max_dphase_df = float(fs[int(np.argmax(np.abs(phase_slope)))])
    return f_min_s21, f_max_dz_df, f_max_dphase_df


def params_from_sweep(sweep, *, n_points=None, point_sequence=None,
                                 offset_linewidths=None,
                                 samples_per_point=1, n_settle=1,
                                 delta_linewidths=0.1, exclude_blind=True,
                                 blind_indices=None, deembed=True, fits=None):
    """
    Turn a calibration sweep into a ready-to-use ``enable_modulation`` config.

    For each resonator a centre and a linewidth are obtained, then a symmetric
    probe pattern is built with **each tone's probe spacing scaled to its own
    linewidth** (so a narrow resonator gets a smaller delta). By default this is
    the symmetric ``n_points`` pattern. Pass ``point_sequence`` to repeat or
    reorder those base points within each modulation cycle, e.g. ``(1, 2, 3, 2)``
    for ``[-delta, 0, +delta, 0]``. Or pass ``offset_linewidths`` to specify the
    expanded pattern directly in linewidth units, e.g. ``(-0.25, 0, +0.25, 0)``.
    The returned dict splats straight into :meth:`ReadoutClient.enable_modulation`.

    The centre is the **geometric steepest point** — the frequency of maximum
    frequency-shift responsivity (``argmax |dS21/df|`` of the de-embedded
    response), i.e. the best signal-to-noise operating point for reading a
    resonator frequency shift. This is **not** the fitted ``fr`` in general: it
    is displaced from ``fr`` by the impedance-mismatch asymmetry (``phi``) and,
    for a driven nonlinear fit, by the Duffing detuning plus the bistable-cliff
    skew (cable delay is excluded — it does not move with the resonator). See
    :func:`resonator.operating_point_hz`: it is the package's operational
    resonance, so the demodulators' ``center_detuning_*`` / ``needs_update``
    (measured from the centred-phase inflection) read ~0 there, and
    ``reference_freqs['offset_from_fr_hz']`` records how far it sits from the
    fitted ``fr``.

    This package does **not** fit resonators itself (so no fit options have to be
    threaded through it). With ``deembed=True`` (default) you must therefore pass
    pre-computed ``fits`` (fit the parsed sweep yourself, normally with
    :func:`souk_readout_tools.fitting.batch_fit`); a per-tone
    :class:`souk_readout_tools.resonator.ResonatorCalibration` is built from each,
    giving a model-consistent centre/linewidth **and** the de-embedding /
    phase-centring calibration that :func:`demodulate` uses for the
    well-conditioned centred basis (and dissipation). With ``deembed=False`` (or
    for any modulated tone lacking a supplied fit) the function falls back to a
    **model-free estimate** from the sweep (centre = measured peak ``|dS21/df|``,
    linewidth from the magnitude-dip FWHM) and that tone gets no calibration.

    Parameters
    ----------
    sweep : dict
        Parsed client sweep data with ``sweep_f``, ``sweep_i`` and ``sweep_q``
        arrays of shape ``(n_sweep_points, n_tones)``. The compact ``f`` / ``z``
        form is also accepted. Blind-tone indices are inferred from standard
        parsed metadata or may be carried directly as ``blind_indices``.
    n_points : int or None, optional
        Number of base probe points (>=2; 3 enables the curvature/detuning
        estimate). Default 3 when ``point_sequence`` is omitted; when
        ``point_sequence`` is supplied and ``n_points`` is None, inferred from
        the maximum point index in the sequence.
    point_sequence : array-like of int or None, optional
        1-based sequence of base point indices to repeat each cycle. For example,
        ``(1, 2, 3)`` is the ordinary three-point pattern and ``(1, 2, 3, 2)``
        repeats the centre point after the high-side point. The output
        ``offsets`` has one row per sequence entry. Mutually exclusive with
        ``offset_linewidths``.
    offset_linewidths : array-like of float or None, optional
        Direct expanded probe pattern in linewidth units. For each modulated
        tone, row ``i`` of ``offsets`` is
        ``offset_linewidths[i] * linewidth_hz[tone]``. For example,
        ``(-0.25, 0.0, 0.25, 0.0)`` gives ``[-0.25 lw, 0, +0.25 lw, 0]``.
        Mutually exclusive with ``n_points`` and ``point_sequence``.
    samples_per_point : int, optional
        Dwell (samples per point per cycle). Default 1.
    n_settle : int, optional
        Settling samples per point. Default 1.
    delta_linewidths : float, optional
        Probe half-span in units of each tone's linewidth (kept small to stay
        near the high-slope inflection). Default 0.1.
    exclude_blind : bool, optional
        Drop blind tones from ``mod_indices``. Default True.
    blind_indices : array-like or None, optional
        Blind-tone indices; overrides ``sweep['blind_indices']`` if given.
    deembed : bool, optional
        Build de-embedding / phase-centring calibrations (recommended). When True,
        ``fits`` is **required**. When False, only the model-free phase-slope
        estimate is produced (no calibration), and ``fits`` must be omitted.
        Default True.
    fits : sequence, dict, or None
        Pre-computed per-tone fits. **Required when** ``deembed=True``. Pass the
        results of :func:`souk_readout_tools.fitting.batch_fit` directly, an
        index-aligned sequence (entries may be ``None``), or
        ``{tone_index: fit}``. Each entry is a :class:`fitting.FitResult` or a
        ready :class:`resonator.ResonatorCalibration`. ``FitResult.tone_index``
        is honoured so skipped blind tones retain the correct indices. Doing the
        fit outside this package keeps the fitter's options out of the modulation
        API.

    Returns
    -------
    dict
        ``center`` : ``(n_tones,)`` per-tone centre frequencies (Hz);
        ``offsets`` : ``(n_cycle_points, len(mod_indices))`` per-tone probe
        offsets (Hz), where ``n_cycle_points`` is ``n_points`` by default or
        ``len(point_sequence)`` / ``len(offset_linewidths)`` when supplied;
        ``mod_indices`` : modulated (resonator) tone indices;
        ``point_sequence`` : 1-based base-point sequence used to build
        ``offsets`` (``None`` when ``offset_linewidths`` was supplied);
        ``offset_linewidths`` : expanded offset pattern in linewidth units;
        ``samples_per_point`` / ``n_settle`` : as requested;
        ``linewidth_hz`` : ``(len(mod_indices),)`` linewidths (for model-free demod);
        ``calibration`` : ``{tone_index: ResonatorCalibration}`` for tones with a
        supplied fit (empty if ``fits`` is None);
        ``reference_freqs`` : dict of ``(n_tones,)`` arrays for cross-checking the
        operating point — ``center_hz`` (the chosen steepest point),
        ``fitted_fr_hz`` (fitted ``fr``; NaN where no fit), ``min_s21_hz``,
        ``max_dz_df_hz`` and ``max_dphase_df_hz`` (measured magnitude-min and
        peak ``|dS21/df|`` / phase-slope frequencies), and ``offset_from_fr_hz``
        (``center - fitted_fr``);
        ``summary`` : a human-readable multi-line summary string.
    """
    sequence_requested = point_sequence is not None
    direct_offsets_requested = offset_linewidths is not None

    def _normalise_n_points(value):
        """Validate an integer-valued base point count."""
        if value is None:
            return None
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError('n_points must be an integer >= 2') from exc
        if not np.isfinite(value) or value != round(value):
            raise ValueError('n_points must be an integer >= 2')
        value = int(round(value))
        if value < 2:
            raise ValueError('n_points must be >= 2')
        return value

    def _normalise_offset_linewidths(values):
        """Validate direct linewidth-scale offsets."""
        arr = np.asarray(values)
        if arr.ndim != 1 or arr.size < 2:
            raise ValueError(
                'offset_linewidths must be a 1D sequence with at least 2 entries')
        try:
            arr = arr.astype(float)
        except (TypeError, ValueError) as exc:
            raise ValueError('offset_linewidths entries must be finite numbers') from exc
        if not np.all(np.isfinite(arr)):
            raise ValueError('offset_linewidths entries must be finite numbers')
        return arr

    if direct_offsets_requested:
        if n_points is not None or point_sequence is not None:
            raise ValueError(
                'offset_linewidths is mutually exclusive with n_points and '
                'point_sequence')
        offset_scales = _normalise_offset_linewidths(offset_linewidths)
        point_sequence = None
        n_points = None
    else:
        offset_scales = None

    if not direct_offsets_requested and point_sequence is None:
        n_points = 3 if n_points is None else _normalise_n_points(n_points)
        point_sequence = np.arange(1, n_points + 1, dtype=int)
    elif not direct_offsets_requested:
        raw_sequence = np.asarray(point_sequence)
        if raw_sequence.ndim != 1 or raw_sequence.size < 2:
            raise ValueError('point_sequence must be a 1D sequence with at least 2 entries')
        try:
            seq_float = raw_sequence.astype(float)
        except (TypeError, ValueError) as exc:
            raise ValueError('point_sequence entries must be integer point indices') from exc
        if not np.all(np.isfinite(seq_float)):
            raise ValueError('point_sequence entries must be finite integer point indices')
        if not np.array_equal(seq_float, np.rint(seq_float)):
            raise ValueError('point_sequence entries must be integer point indices')
        point_sequence = np.rint(seq_float).astype(int)
        if np.any(point_sequence < 1):
            raise ValueError('point_sequence entries must be >= 1')
        if n_points is None:
            n_points = int(np.max(point_sequence))
            if n_points < 2:
                raise ValueError(
                    'point_sequence must reference at least two base points '
                    'when n_points is omitted')
        else:
            n_points = _normalise_n_points(n_points)
        if np.any(point_sequence > n_points):
            raise ValueError(
                'point_sequence entries must be in the range 1..n_points')

    if 'f' in sweep and 'z' in sweep:
        f = np.asarray(sweep['f'], dtype=float)
        z = np.asarray(sweep['z'])
    elif all(key in sweep for key in ('sweep_f', 'sweep_i', 'sweep_q')):
        f = np.asarray(sweep['sweep_f'], dtype=float)
        z = np.asarray(sweep['sweep_i']) + 1j * np.asarray(sweep['sweep_q'])
    else:
        raise ValueError(
            "sweep must contain parsed sweep_f/sweep_i/sweep_q arrays "
            "or compact f/z arrays")
    if f.ndim == 1:
        f = f[:, None]
        z = z[:, None]
    if f.shape != z.shape:
        raise ValueError(f'sweep frequency and IQ shapes differ: {f.shape} != {z.shape}')
    n_sweep, n_tones = f.shape

    infer_blinds = blind_indices is None
    if infer_blinds:
        def _metadata_sequence(*values):
            """Return the first non-empty metadata sequence."""
            for value in values:
                if value is None:
                    continue
                try:
                    if len(value) == 0:
                        continue
                except TypeError:
                    pass
                return value
            return []

        tone_metadata = sweep.get('tone_metadata', {}) or {}
        tone_metadata = tone_metadata if isinstance(tone_metadata, dict) else {}
        info = sweep.get('info', {}) or {}
        tones = info.get('tones', {}) if isinstance(info, dict) else {}
        blind_indices = _metadata_sequence(
            sweep.get('blind_indices'),
            tone_metadata.get('blind_indices'),
            tones.get('blind_indices'),
        )
        regular_indices = _metadata_sequence(
            tone_metadata.get('regular_indices'),
            tones.get('regular_indices'),
        )
    else:
        regular_indices = []

    def _index_set(values):
        """Normalise scalar or sequence metadata indices within the tone range."""
        try:
            return {int(i) for i in values if 0 <= int(i) < n_tones}
        except TypeError:
            index = int(values)
            return {index} if 0 <= index < n_tones else set()

    blind = _index_set(blind_indices)
    regular = _index_set(regular_indices)
    if infer_blinds and regular:
        blind.update(set(range(n_tones)) - regular)
    mod_indices = [i for i in range(n_tones) if not (exclude_blind and i in blind)]

    # We never fit here. deembed=True therefore requires the caller to supply fits.
    if deembed and fits is None:
        raise ValueError(
            'deembed=True requires precomputed `fits` (fit the sweep yourself, '
            'e.g. fitting.batch_fit, and pass fits=...). Use deembed=False for '
            'the model-free phase-slope estimate.')
    if not deembed and fits is not None:
        raise ValueError('fits cannot be supplied when deembed=False')

    fit_by_tone = {}
    if isinstance(fits, dict):
        fit_by_tone = {int(tone_index): fit for tone_index, fit in fits.items()}
    elif fits is not None:
        for position, fit in enumerate(fits):
            if fit is None:
                continue
            tone_index = int(getattr(fit, 'tone_index', -1))
            tone_index = tone_index if tone_index >= 0 else position
            if tone_index in fit_by_tone:
                raise ValueError(f'fits contains duplicate tone index {tone_index}')
            fit_by_tone[tone_index] = fit

    def _provided_fit(t):
        """The caller-supplied fit/calibration for tone ``t`` (or None)."""
        fit = fit_by_tone.get(t)
        if fit is not None and hasattr(fit, 'success') and not fit.success:
            return None
        return fit

    # Magnitude-dip FWHM for the model-free linewidth (any tone without a fit).
    from souk_readout_tools.resonator import estimate_resonance_empirical
    # Only need the calibration class when fits are supplied (to wrap FitResults).
    ResonatorCalibration = None
    if fits is not None:
        from souk_readout_tools.resonator import ResonatorCalibration

    center = np.zeros(n_tones, dtype=float)
    linewidth = np.zeros(n_tones, dtype=float)
    calibration = {}
    # Measured (model-free) reference frequencies, straight from the sweep, plus
    # the fitted fr, returned for cross-checking the chosen operating point.
    fitted_fr = np.full(n_tones, np.nan)
    f_min_s21 = np.full(n_tones, np.nan)
    f_max_dz_df = np.full(n_tones, np.nan)
    f_max_dphase_df = np.full(n_tones, np.nan)
    for t in range(n_tones):
        ft = f[:, t]
        f_min_s21[t], f_max_dz_df[t], f_max_dphase_df[t] = (
            _measured_reference_freqs(ft, z[:, t]))
        provided = _provided_fit(t)
        if provided is not None:
            # Build the calibration from the supplied fit (already a calibration,
            # or a FitResult). No fitting happens here.
            cal = (provided if hasattr(provided, 'deembed_sweep')
                   else ResonatorCalibration.from_fit(provided))
            fitted_fr[t] = cal.fr
            linewidth[t] = cal.fr / cal.Ql
            # Operating point = geometric steepest point (max frequency-shift
            # SNR), not the fitted fr: see resonator.operating_point_hz.
            center[t] = operating_point_hz(cal)
            if t in mod_indices:
                calibration[t] = cal
        else:
            # Model-free: park on the steepest measured point (peak |dS21/df|,
            # the best available max-SNR estimate). Linewidth from the
            # magnitude-dip FWHM (linear power, ~ fr/Ql for a notch), matching the
            # fitted path. The raw angle-from-origin spins up near a deep dip, so
            # the old 4/|dphi/df| estimate badly underestimated the linewidth.
            center[t] = f_max_dz_df[t]
            linewidth[t] = estimate_resonance_empirical(ft, z[:, t]).linewidth_hz

    # Offset pattern in linewidth units. The hardware only needs the expanded
    # Hz offsets, but preserving the scale pattern makes configs inspectable.
    if direct_offsets_requested:
        pattern = offset_scales
    else:
        base_pattern = np.linspace(
            -delta_linewidths, delta_linewidths, n_points)
        pattern = base_pattern[point_sequence - 1]
    offsets = pattern[:, None] * linewidth[mod_indices][None, :]

    points_desc = f'{len(pattern)} points'
    if direct_offsets_requested:
        points_desc += f' (offset_linewidths={pattern.tolist()})'
    elif sequence_requested:
        points_desc += f' (sequence={point_sequence.tolist()}, base={n_points})'
    offset_desc = (
        f'offsets={pattern.tolist()} linewidths'
        if direct_offsets_requested else
        f'delta={delta_linewidths} linewidths')
    summary_lines = [
        f'params_from_sweep: {len(mod_indices)} modulated tones '
        f'({n_tones - len(mod_indices)} excluded), {points_desc}, '
        f'dwell={samples_per_point}, n_settle={n_settle}, {offset_desc}',
    ]
    # Operating point relative to the fitted resonance (in linewidths), so the
    # steepest-point offset from fr is visible and can seed the demod baseline.
    offset_from_fr = center - fitted_fr
    for t in mod_indices[:8]:
        has_fr = np.isfinite(fitted_fr[t]) and linewidth[t] > 0
        off = (f'  center-fr={offset_from_fr[t] / linewidth[t]:+.3f} lw'
               if has_fr else '  (model-free, no fr)')
        max_offset = np.nanmax(np.abs(pattern)) * linewidth[t]
        offset_summary = (
            f'offset span<={max_offset/1e3:.3f} kHz'
            if direct_offsets_requested else
            f'delta=+/-{delta_linewidths * linewidth[t]/1e3:.3f} kHz')
        summary_lines.append(
            f'  tone {t:4d}: center={center[t]/1e6:.6f} MHz  '
            f'linewidth={linewidth[t]/1e3:.2f} kHz  '
            f'{offset_summary}{off}')
    if len(mod_indices) > 8:
        summary_lines.append(f'  ... (+{len(mod_indices) - 8} more)')

    if fits is not None:
        summary_lines.append(
            f'  de-embed calibration: {len(calibration)}/{len(mod_indices)} tones (from supplied fits)')

    return {
        'center': center,
        'offsets': offsets,
        'mod_indices': mod_indices,
        'point_sequence': point_sequence,
        'offset_linewidths': pattern,
        'samples_per_point': int(samples_per_point),
        'n_settle': int(n_settle),
        'linewidth_hz': linewidth[mod_indices],
        'calibration': calibration,
        # Per-tone (n_tones,) reference frequencies for cross-checking the
        # operating point. center is the geometric steepest point (max SNR);
        # these show how far it sits from the fitted fr and the measured
        # magnitude/slope features. offset_from_fr_hz = center - fitted_fr.
        'reference_freqs': {
            'center_hz': center,
            'fitted_fr_hz': fitted_fr,
            'min_s21_hz': f_min_s21,
            'max_dz_df_hz': f_max_dz_df,
            'max_dphase_df_hz': f_max_dphase_df,
            'offset_from_fr_hz': offset_from_fr,
        },
        'summary': '\n'.join(summary_lines),
    }
