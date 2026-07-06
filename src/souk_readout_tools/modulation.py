"""
Fast-frequency-modulation client toolkit (setup + demodulation).

These are **pure, transport-agnostic** functions: they operate on plain arrays /
dicts (parsed stream samples + the ``tone_modulation`` state), with no
``ReadoutClient`` or socket dependency. That keeps them usable client-side on
received frames today and relocatable/shareable server-side later.

The workflow a consumer follows is:

0. :func:`params_from_sweep` -> turn a calibration sweep into a ready-to-use
   ``enable_modulation`` config (per-tone centres + per-tone probe deltas scaled
   to each resonator's linewidth). This is the *setup* step.
1. Acquire modulated samples (``ReadoutClient.get_samples`` while modulation is
   armed, or a captured continuous stream) and parse them
   (``ReadoutClient.parse_samples``) -> a ``data_dict`` carrying per-tone I/Q,
   ``modulation_point`` (1..N, 0 = off), ``modulation_settling`` and
   ``modulation_revision``.
2. :func:`group_cycles` -> per-tone complex measurements grouped by step and
   cycle (settling samples dropped), plus the per-point probe offsets (Hz) read
   from ``get_info('modulation')`` and the grouped packet counters,
   telescope time, packet errors and stream flags.
3. :func:`demodulate` -> per-cycle dphi/df, d2phi/df2, frequency shift and a
   detuning estimate (with a ``needs_update`` flag for the future tracking loop).

Physics caveats:
- The point we steer to is the **steepest part of the phase curve / inflection
  point** (d2phi/df2 = 0); for a non-ideal device this is *near*, not exactly,
  the physical f0 once asymmetry / cable delay / nonlinearity matter.
- ``d2phi/df2 / dphi/df`` has units 1/Hz, so a detuning expressed in linewidths
  needs a **linewidth scale** (or a calibrated model). The ``'fast'`` estimator
  here is leading-order for an ideal Lorentzian/arctan phase; calibrate per
  device with ``method='model'`` for non-ideal resonators.
- Dissipation is *not* a naive d|S21|/df: as a resonator detunes the operating
  point slides around the resonance circle, so it needs the circle calibration.
"""

import copy
import warnings

import numpy as np


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


def _stack_or_empty(items, shape, dtype=float):
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


def _linearized_iq_projection(reference_iq, gradient, s21,
                              reference_frequency_hz=None):
    """Project IQ displacement onto local frequency and matched-loss axes.

    Thin wrapper over :func:`resonator.project_iq_tangent_normal`: returns the
    tangent displacement in Hz and the matched-loss displacement as a fractional
    dissipation (``dd_hz / reference_frequency_hz``, NaN if no usable reference).
    """
    from .resonator import project_iq_tangent_normal
    df_hz, dd_hz = project_iq_tangent_normal(reference_iq, gradient, s21)
    if reference_frequency_hz is None:
        dissipation = np.full(np.shape(dd_hz), np.nan, dtype=float)
    else:
        reference_frequency_hz = float(reference_frequency_hz)
        if np.isfinite(reference_frequency_hz) and reference_frequency_hz != 0.0:
            dissipation = dd_hz / reference_frequency_hz
        else:
            dissipation = np.full(np.shape(dd_hz), np.nan, dtype=float)
    return df_hz, dissipation


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
    on_missing : {'notify', 'fill'}, optional
        How to handle gaps in ``packet_counter`` (dropped accumulations).
        ``'notify'`` (default) issues a warning reporting the number and location
        of missing packets and proceeds (cycles straddling a gap are simply
        dropped, since they are incomplete). ``'fill'`` additionally rebuilds the
        stream on a contiguous counter axis with **NaN-IQ placeholders** for the
        missing packets (their point/settling inferred from the deterministic
        cadence), so affected cycles still appear in the output with NaN where
        data was lost. A warning is always emitted when packets are missing.
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
        ``freq_hz`` : ``(N, n_tones)`` absolute probe frequencies (centre + offset);
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

    points = np.asarray(data_dict['modulation_point'], dtype=int)
    settling = np.asarray(data_dict['modulation_settling'], dtype=int)
    revisions = np.asarray(data_dict['modulation_revision'], dtype=int)
    z = _tone_iq(data_dict, n_tones)
    sample_index = np.arange(len(points), dtype=int)
    sample_present = np.ones(len(points), dtype=bool)

    # Detect dropped accumulations via the packet counter, and either notify or
    # fill the gaps with NaN placeholders (their tag inferred from the cadence).
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
            if on_missing == 'fill':
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
            elif on_missing != 'notify':
                raise ValueError(f"on_missing must be 'notify' or 'fill', got {on_missing!r}")
    elif on_missing not in ('notify', 'fill'):
        raise ValueError(f"on_missing must be 'notify' or 'fill', got {on_missing!r}")

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
    freq_hz = np.zeros((N, n_tones), dtype=float)
    for tone in tone_modulation_state.get('tones', []):
        idx = int(tone['index'])
        if idx < n_tones:
            off = np.asarray(tone['offsets_hz'], dtype=float)
            offsets_hz[:, idx] = off
            freq_hz[:, idx] = float(tone.get('center_hz', 0.0)) + off

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
    samples_used_out = (
        np.stack(cycles_used, axis=0) if cycles_used
        else np.empty((0, N), dtype=int))
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
        'z': np.stack(cycles_z, axis=0) if cycles_z else np.empty(z_out_shape, dtype=complex),
        'offsets_hz': offsets_hz,
        'freq_hz': freq_hz,
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


def demodulate_timestream(grouped, offsets=None, *, method='accurate',
                          calibration=None, reference_z=None,
                          data_dict=None, n_samples=None,
                          fill_settling=True,
                          use_settling_for_fit=False,
                          threshold_linewidths=0.1):
    """
    Convert every kept modulation sample into a sample-aligned demodulated stream.

    This is the "do not throw away integration time" companion to
    :func:`demodulate`.  It uses the individual samples retained by
    ``group_cycles(..., reduce=None)`` rather than first averaging each point in
    a cycle.  Each modulation point is treated as a valid resonator probe:

    - the static per-point response is divided out to form ``z_demod`` /
      ``mag`` / ``phase`` as if each sample had been measured at the centre
      point;
    - the phase residual at that point is divided by the local phase slope to
      give a centre-probe detuning timestream;
    - the cycle-level ``dphi/df`` and ``d2phi/df2`` estimates are held across
      all samples in that cycle so they line up with the original sample grid.

    ``freq_shift_hz`` follows the same probe-side sign convention as
    :func:`demodulate`: it is the centre probe detuning, approximately
    ``f_center - f_resonance``.  ``resonance_shift_hz`` is the opposite sign,
    useful when reading detector resonance motion.

    Parameters
    ----------
    grouped : dict
        Output from ``group_cycles(data, state, reduce=None)``.  Mean-reduced
        grouped data cannot recover the original per-sample stream and raises a
        ``ValueError``.
    offsets : numpy.ndarray or None, optional
        ``(N, n_tones)`` per-point probe offsets (Hz). ``None`` uses
        ``grouped['offsets_hz']``.
    method : {'accurate', 'model', 'fast'}, optional
        Phase-derivative estimator.  ``'accurate'`` / ``'model'`` fit a
        quadratic phase-vs-offset model when at least three distinct offsets are
        present, falling back to a line.  ``'fast'`` uses the extreme offsets
        for the centre slope and the same quadratic curvature fallback.  The
        default is ``'accurate'`` because repeated centre points such as
        ``[-1000, 0, +1000, 0]`` are common.
    calibration : dict or None, optional
        Per-tone ``{tone_index: ResonatorCalibration}``.  When supplied, raw IQ
        is de-embedded/phase-centred before forming ``z_demod`` and each sample's
        frequency/dissipation is computed with the calibrated Möbius inversion.
        The known modulation offset is then subtracted so all points estimate
        the centre-probe detuning.
    reference_z : numpy.ndarray or None, optional
        Optional static per-point reference response, shape ``(N, n_tones)``.
        If omitted, the reference is the complex mean of this capture for each
        point/tone in the selected basis (raw IQ, or centred IQ with
        ``calibration``).  Passing an external reference gives an absolute
        baseline; the default gives a capture-relative baseline.
    data_dict : dict or None, optional
        Original parsed sample dict.  If supplied, output length is
        ``len(data_dict['modulation_point'])`` and raw per-sample metadata are
        copied through.  Otherwise ``n_samples`` is inferred from grouped sample
        indices unless supplied explicitly.
    n_samples : int or None, optional
        Explicit output length when ``data_dict`` is not supplied.
    fill_settling : bool or {'raw', 'interpolate', 'none'}, optional
        How to fill samples flagged with ``modulation_settling==1`` when they
        were not retained by ``group_cycles`` or when their transient IQ should
        be replaced. ``True`` / ``'raw'`` (default) demodulates the raw settling
        IQ using the completed cycle's reference/slope. ``'interpolate'`` fills
        settling samples by linear interpolation between non-settling
        demodulated samples, leaving flags intact. ``False`` / ``'none'`` leaves
        dropped settling samples as NaN. Incomplete cycles and missing packets
        remain NaN.
    use_settling_for_fit : bool, optional
        If ``False`` (default), samples flagged with ``modulation_settling==1``
        are included in the output but excluded from the per-point reference,
        phase-slope and curvature estimates. Set ``True`` only when the settling
        flag is being used diagnostically rather than to mark transient samples.
    threshold_linewidths : float, optional
        Calibrated ``needs_update`` threshold.  Ignored without calibration.

    Returns
    -------
    dict
        Sample-aligned arrays, usually shape ``(n_samples, n_tones)``:
        ``z_demod``, ``mag``, ``phase_rad``, ``phase_unwrapped_rad``,
        ``freq_shift_hz``, ``resonance_shift_hz``, ``dphi_df`` (local point
        slope), ``center_dphi_df`` (cycle slope at zero/nearest-centre offset),
        ``d2phi_df2``, ``modulation_offset_hz``, and ``dissipation``.  Without
        calibration, ``dissipation`` is the same local linearized matched
        quadrature used by the parsed-sample plotting path, with the modulation
        points providing the local IQ gradient.  Metadata arrays ``cycle_index``,
        ``modulation_point``, ``modulation_settling``, ``sample_present``,
        ``sample_used_in_cycle`` and packet/telescope fields are included when
        available.
    """
    if method not in ('accurate', 'model', 'fast'):
        raise ValueError(f"method must be 'accurate', 'model' or 'fast', got {method!r}")
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

    freq_hz = grouped.get('freq_hz')
    if freq_hz is not None:
        freq_hz = np.asarray(freq_hz, dtype=float)

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
        'freq_shift_hz': np.full(sample_shape, np.nan),
        'resonance_shift_hz': np.full(sample_shape, np.nan),
        'dphi_df': np.full(sample_shape, np.nan),
        'center_dphi_df': np.full(sample_shape, np.nan),
        'd2phi_df2': np.full(sample_shape, np.nan),
        'modulation_offset_hz': np.full(sample_shape, np.nan),
        'dissipation': np.full(sample_shape, np.nan),
        'detuning_linewidths': np.full(sample_shape, np.nan),
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
    center_reference_z = np.full(n_tones, np.nan + 1j * np.nan, dtype=complex)
    center_offset_hz = np.full(n_tones, np.nan)
    center_point_index = np.zeros(n_tones, dtype=int)
    dissipation_reference_frequency_hz = np.full(n_tones, np.nan)

    tone_frequencies_hz = None
    info = out.get('info')
    if isinstance(info, dict):
        tones_info = info.get('tones', {})
        if isinstance(tones_info, dict) and tones_info.get('frequencies_hz') is not None:
            tone_frequencies_hz = np.asarray(tones_info.get('frequencies_hz'), dtype=float)

    def _cal_for(t):
        if calibration is None:
            return None
        if isinstance(calibration, dict):
            return calibration.get(t)
        return calibration[t] if t < len(calibration) else None

    def _reference_frequency_hz(t, c_point):
        candidates = []
        if freq_hz is not None and freq_hz.shape == (N, n_tones):
            candidates.append(freq_hz[c_point, t])
            candidates.append(_nanmean_quiet(freq_hz[:, t]))
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
    # phase slope/curvature, then turn every sample into a centre-probe estimate.
    for t in range(n_tones):
        f = offsets[:, t]
        zt = z[:, :, :, t]
        cal = _cal_for(t)

        # Working basis: calibrated (de-embedded + phase-centred) when a
        # calibration is supplied, otherwise the raw I/Q as captured.
        if cal is not None and freq_hz is not None:
            z_basis = np.full_like(zt, np.nan + 1j * np.nan, dtype=complex)
            for p in range(N):
                z_basis[:, p, :] = cal.transform_raw_iq(freq_hz[p, t], zt[:, p, :])
        else:
            z_basis = zt

        # Per-point reference = external value, or the capture mean (settling
        # samples excluded), falling back to the all-sample mean where empty.
        z_fit_basis = np.where(fit_sample_mask, z_basis, np.nan + 1j * np.nan)
        if ref_input is not None:
            ref = ref_input[:, t]
        else:
            ref = _nanmean_quiet(z_fit_basis, axis=(0, 2))
            ref_all = _nanmean_quiet(z_basis, axis=(0, 2))
            ref = np.where(np.isfinite(ref), ref, ref_all)
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
        ref_freq_hz = _reference_frequency_hz(t, c_point)
        dissipation_reference_frequency_hz[t] = ref_freq_hz

        # Per-cycle phase-vs-offset curve -> centre slope, local slope at each
        # point, and curvature. 'fast' takes the slope from the extreme offsets;
        # otherwise a quadratic (>=3 distinct offsets) / linear fit is used.
        point_z = _nanmean_quiet(z_fit_basis, axis=2)      # (n_cycles, N)
        point_z_all = _nanmean_quiet(z_basis, axis=2)
        point_z = np.where(np.isfinite(point_z), point_z, point_z_all)
        order = np.argsort(f)
        f_fit = f[order]
        phi_fit = np.unwrap(np.angle(point_z[:, order]), axis=1)
        n_unique = len(np.unique(f_fit))

        slope_center = np.full(n_cycles, np.nan)
        curvature = np.full(n_cycles, np.nan)
        local_slope = np.full((n_cycles, N), np.nan)
        complex_gradient = np.full((n_cycles, N), np.nan + 1j * np.nan, dtype=complex)

        if n_unique >= 2:
            if method == 'fast':
                lo = int(order[0])
                hi = int(order[-1])
                df_span = f[hi] - f[lo]
                if df_span != 0:
                    phi_span = _wrap_phase(np.angle(point_z[:, hi])
                                           - np.angle(point_z[:, lo]))
                    slope_center = phi_span / df_span

            deg = 2 if n_unique >= 3 else 1
            coeff = np.polyfit(f_fit, phi_fit.T, deg)
            z_coeff = np.polyfit(f_fit, point_z[:, order].T, deg)
            if deg == 1:
                fit_slope = coeff[0]
                fit_curv = np.full(n_cycles, np.nan)
                complex_gradient[:, :] = z_coeff[0][:, None]
            else:
                fit_slope = 2.0 * coeff[0] * c_offset + coeff[1]
                fit_curv = 2.0 * coeff[0]

            if method != 'fast':
                slope_center = fit_slope
            if deg == 2:
                curvature = fit_curv
                for p in range(N):
                    local_slope[:, p] = 2.0 * coeff[0] * f[p] + coeff[1]
                    complex_gradient[:, p] = 2.0 * z_coeff[0] * f[p] + z_coeff[1]
            else:
                local_slope[:, :] = fit_slope[:, None]
        local_slope = np.where(np.isfinite(local_slope), local_slope, slope_center[:, None])

        # z_demod: rotate/scale every sample onto the centre-point reference, so
        # each probe point reads as if measured at the centre.
        with np.errstate(divide='ignore', invalid='ignore'):
            z_demod_group = z_basis / ref[None, :, None] * ref_center

        # Frequency + dissipation. Calibrated: the Mobius inversion gives the
        # probe detuning directly (subtract the known offset for centre detuning).
        if cal is not None and freq_hz is not None:
            df_probe, diss = cal.convert_centered_iq(z_basis)
            freq_group = df_probe - f[None, :, None]
            diss_group = diss
            lw_t = cal.fr / cal.Ql
            with np.errstate(divide='ignore', invalid='ignore'):
                det_lw_group = freq_group / lw_t
            needs_update_group = np.abs(det_lw_group) > threshold_linewidths
        else:
            # Model-free: detuning from the phase residual / local slope, and
            # dissipation from the linearized tangent/normal IQ projection.
            phase_resid = _wrap_phase(
                np.angle(z_basis) - np.angle(ref)[None, :, None])
            with np.errstate(divide='ignore', invalid='ignore'):
                freq_group = phase_resid / local_slope[:, :, None]
            _, diss_group = _linearized_iq_projection(
                ref[None, :, None],
                complex_gradient[:, :, None],
                z_basis,
                reference_frequency_hz=ref_freq_hz)
            det_lw_group = np.full_like(freq_group, np.nan, dtype=float)
            needs_update_group = np.zeros(freq_group.shape, dtype=bool)

        # Scatter this tone's grouped per-sample results onto the output grid.
        valid = meta_valid & np.isfinite(z_basis)
        if not np.any(valid):
            continue

        idx = sample_index[valid]
        out['z_demod'][idx, t] = z_demod_group[valid]
        out['freq_shift_hz'][idx, t] = freq_group[valid]
        out['resonance_shift_hz'][idx, t] = -freq_group[valid]
        out['dissipation'][idx, t] = diss_group[valid]
        out['detuning_linewidths'][idx, t] = det_lw_group[valid]
        out['needs_update'][idx, t] = needs_update_group[valid]

        offset_grid = np.broadcast_to(f[None, :, None], sample_index.shape)
        local_grid = np.broadcast_to(local_slope[:, :, None], sample_index.shape)
        center_grid = np.broadcast_to(slope_center[:, None, None], sample_index.shape)
        curv_grid = np.broadcast_to(curvature[:, None, None], sample_index.shape)
        out['modulation_offset_hz'][idx, t] = offset_grid[valid]
        out['dphi_df'][idx, t] = local_grid[valid]
        out['center_dphi_df'][idx, t] = center_grid[valid]
        out['d2phi_df2'][idx, t] = curv_grid[valid]

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
                if cal is not None and freq_hz is not None:
                    sample_basis = cal.transform_raw_iq(freq_hz[p, t], raw_values)
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

                if cal is not None and freq_hz is not None:
                    df_probe, diss = cal.convert_centered_iq(sample_basis)
                    freq_sample = df_probe - f[p]
                    out['dissipation'][sample_idx, t] = diss
                    lw_t = cal.fr / cal.Ql
                    with np.errstate(divide='ignore', invalid='ignore'):
                        det_lw = freq_sample / lw_t
                    out['detuning_linewidths'][sample_idx, t] = det_lw
                    out['needs_update'][sample_idx, t] = (
                        np.abs(det_lw) > threshold_linewidths)
                else:
                    phase_resid = _wrap_phase(np.angle(sample_basis) - np.angle(ref[p]))
                    with np.errstate(divide='ignore', invalid='ignore'):
                        freq_sample = phase_resid / local_slope[cycle_idx, p]
                    _, diss = _linearized_iq_projection(
                        ref[p],
                        complex_gradient[cycle_idx, p],
                        sample_basis,
                        reference_frequency_hz=ref_freq_hz)
                    out['dissipation'][sample_idx, t] = diss

                out['freq_shift_hz'][sample_idx, t] = freq_sample
                out['resonance_shift_hz'][sample_idx, t] = -freq_sample
                out['modulation_offset_hz'][sample_idx, t] = f[p]
                out['dphi_df'][sample_idx, t] = local_slope[cycle_idx, p]
                out['center_dphi_df'][sample_idx, t] = slope_center[cycle_idx]
                out['d2phi_df2'][sample_idx, t] = curvature[cycle_idx]

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
                'freq_shift_hz',
                'resonance_shift_hz',
                'dphi_df',
                'center_dphi_df',
                'd2phi_df2',
                'dissipation',
                'detuning_linewidths'):
            out[key] = _interp_nan_targets(out[key], settling_targets, source)
        out['needs_update'][settling_targets] = (
            np.abs(out['detuning_linewidths'][settling_targets])
            > threshold_linewidths)

    # Derived magnitude/phase plus parsed-sample-compatible aliases so the result
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
    out['frequency_hz'] = out['resonance_shift_hz']
    out['frequency_shift_hz'] = out['resonance_shift_hz']
    out['probe_frequency_shift_hz'] = out['freq_shift_hz']
    out['modulation_demodulated'] = True
    out['point_reference_z'] = point_reference_z
    out['center_reference_z'] = center_reference_z
    out['point_reference_phase_rad'] = np.angle(point_reference_z)
    out['center_reference_phase_rad'] = np.angle(center_reference_z)
    out['center_offset_hz'] = center_offset_hz
    out['center_point_index'] = center_point_index
    out['dissipation_reference_frequency_hz'] = dissipation_reference_frequency_hz
    return out


def demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None,
               calibration=None, phase_baseline=None, threshold_linewidths=0.1):
    """
    Demodulate grouped modulation cycles into per-tone, per-cycle quantities.

    Two bases are supported:

    - **Model-free** (no ``calibration``): work on the raw I/Q phase. The local
      phase slope, curvature, frequency shift and a leading-order detuning are
      estimated directly. Detuning/`needs_update` then require ``linewidth_hz``,
      and dissipation is unavailable (NaN). The raw phase still carries the cable
      delay and an off-origin circle, so it is less well conditioned.
    - **Calibrated / centred** (``calibration`` supplied): each probe point is
      de-embedded (cable delay removed at its absolute frequency) and
      phase-centred using a per-tone
      :class:`souk_readout_tools.resonator.ResonatorCalibration` (from
      :func:`params_from_sweep`). In this basis the phase is well conditioned,
      and the **exact Möbius inversion** yields probe detuning
      ``f_probe - f_r`` and
      matched-scale ``Delta(1 / (2 * Qi))`` dissipation quadrature (no
      linewidth argument needed — the calibration carries ``fr``/``Ql``).
      For the fitted Duffing model, an
      analytic inverse maps the recovered circle coordinate back to probe
      detuning. Resonator detuning relative to the probe has the opposite sign.
      This is the recommended basis.

    Parameters
    ----------
    grouped : dict
        Output of :func:`group_cycles` (uses ``z``; ``offsets_hz`` /
        ``freq_hz`` / ``revision``). ``z`` may be ``(n_cycles, N, n_tones)``
        (mean-reduced) or ``(n_cycles, N, n_used, n_tones)`` (the sample axis is
        averaged here).
    offsets : numpy.ndarray or None, optional
        ``(N, n_tones)`` per-point probe offsets (Hz). ``None`` uses
        ``grouped['offsets_hz']``.
    method : {'fast', 'accurate', 'model'}, optional
        Slope/curvature estimator. ``'fast'`` = finite differences;
        ``'accurate'`` = (weighted) polynomial fit of phase vs offset (handles
        asymmetric offsets); ``'model'`` shares the ``'accurate'`` path here.
    linewidth_hz : float or array-like or None, optional
        Per-tone resonator linewidth (Hz) for the *model-free* detuning. Ignored
        when a ``calibration`` is given (the calibration supplies it). Without
        either, model-free ``detuning_*`` / ``needs_update`` are NaN/False.
    calibration : dict or None, optional
        Per-tone ``{tone_index: ResonatorCalibration}`` (e.g.
        ``params_from_sweep(..., deembed=True)['calibration']``). Enables the
        calibrated/centred basis with exact linear-notch frequency shift and
        matched-scale dissipation.
    phase_baseline : array-like or None, optional
        Per-tone reference phase (rad) for the *model-free* frequency-shift
        conversion; ``None`` uses each tone's mean centre-point phase.
    threshold_linewidths : float, optional
        ``needs_update`` trips when ``|detuning_linewidths|`` exceeds this
        (default 0.1).

    Returns
    -------
    dict
        Per-tone, per-cycle arrays of shape ``(n_cycles, n_tones)`` unless noted:
        ``z_center``, ``dphi_df``, ``d2phi_df2``, ``freq_shift_hz``,
        ``detuning_linewidths``, ``detuning_hz``, ``needs_update`` (bool),
        ``dissipation``; plus ``revision`` ``(n_cycles,)`` and cycle-aligned
        metadata copied from :func:`group_cycles` when available:
        ``packet_counter``, ``telescope_time``, ``packet_error``,
        ``packet_missing``, ``sample_index``, ``stream_flags``. In the
        calibrated basis ``freq_shift_hz`` / ``detuning_hz`` are the centre
        tone's probe detuning ``f_probe - f_r`` (Hz) and ``dissipation`` is
        ``Delta(1 / (2 * Qi))``.
    """
    z = np.asarray(grouped['z'])
    if z.ndim == 4:                      # (n_cycles, N, n_used, n_tones) -> average dwell
        z = z.mean(axis=2)
    if z.ndim != 3:
        raise ValueError(f'grouped["z"] must be 3D or 4D, got {z.ndim}D')
    n_cycles, N, n_tones = z.shape

    if offsets is None:
        offsets = grouped['offsets_hz']
    offsets = np.asarray(offsets, dtype=float)
    freq_hz = grouped.get('freq_hz')           # absolute probe freqs (for de-embedding)

    lw = None if linewidth_hz is None else np.broadcast_to(
        np.asarray(linewidth_hz, dtype=float), (n_tones,))

    out = {
        'z_center': np.zeros((n_cycles, n_tones), dtype=complex),
        'dphi_df': np.full((n_cycles, n_tones), np.nan),
        'd2phi_df2': np.full((n_cycles, n_tones), np.nan),
        'freq_shift_hz': np.full((n_cycles, n_tones), np.nan),
        'detuning_linewidths': np.full((n_cycles, n_tones), np.nan),
        'detuning_hz': np.full((n_cycles, n_tones), np.nan),
        'needs_update': np.zeros((n_cycles, n_tones), dtype=bool),
        'dissipation': np.full((n_cycles, n_tones), np.nan),
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

    def _cal_for(t):
        if calibration is None:
            return None
        if isinstance(calibration, dict):
            return calibration.get(t)
        return calibration[t] if t < len(calibration) else None

    for t in range(n_tones):
        f = offsets[:, t]                       # (N,) probe offsets for this tone
        zt = z[:, :, t]                          # (n_cycles, N)
        ci = int(np.argmin(np.abs(f)))           # the point nearest the centre
        out['z_center'][:, t] = zt[:, ci]

        cal = _cal_for(t)
        if cal is not None and freq_hz is not None:
            # Calibrated basis: de-embed + phase-centre, then phase is well
            # conditioned and the Mobius inversion gives df / dissipation.
            zc = cal.transform_raw_iq(freq_hz[:, t], zt)     # (n_cycles, N), centred
            phi = np.unwrap(np.angle(zc), axis=1)
            df_pts, dd_pts = cal.convert_centered_iq(zc)      # (n_cycles, N) each
            lw_t = cal.fr / cal.Ql                            # linewidth (Hz) from the fit
            # Calibrated: df_pts is the ABSOLUTE detuning of the centre probe from
            # the fitted fr, so freq_shift and detuning are the same quantity and
            # detuning-in-linewidths is just that over the linewidth.
            absolute_detuning_hz = df_pts[:, ci]
            out['freq_shift_hz'][:, t] = absolute_detuning_hz
            out['detuning_hz'][:, t] = absolute_detuning_hz
            out['detuning_linewidths'][:, t] = absolute_detuning_hz / lw_t
            out['needs_update'][:, t] = np.abs(absolute_detuning_hz / lw_t) > threshold_linewidths
            out['dissipation'][:, t] = dd_pts[:, ci]
        else:
            # Model-free basis: raw I/Q phase.
            phi = np.unwrap(np.angle(zt), axis=1)

        # Local slope / curvature from the chosen phase basis.
        if method == 'fast':
            df = f[-1] - f[0]
            slope = (phi[:, -1] - phi[:, 0]) / df if df != 0 else np.full(n_cycles, np.nan)
            if N >= 3:
                mid = N // 2
                half = (f[-1] - f[0]) / 2.0
                d2 = (phi[:, -1] - 2 * phi[:, mid] + phi[:, 0]) / (half ** 2) if half != 0 else np.full(n_cycles, np.nan)
            else:
                d2 = np.full(n_cycles, np.nan)
        else:   # 'accurate' / 'model' -> polynomial fits of phi vs f
            slope = np.full(n_cycles, np.nan)
            d2 = np.full(n_cycles, np.nan)
            if N >= 2:
                slope = np.polyfit(f, phi.T, 1)[0]           # (n_cycles,)
            if N >= 3:
                d2 = 2.0 * np.polyfit(f, phi.T, 2)[0]

        out['dphi_df'][:, t] = slope
        out['d2phi_df2'][:, t] = d2

        if cal is None or freq_hz is None:
            # Model-free: no de-embedding, so the raw phase has an unknown offset
            # (cable delay, electronics, the unknown true fr) and we cannot read an
            # absolute detuning directly. Two separate quantities are produced.

            # 1) freq_shift_hz is RELATIVE motion: the centre-point phase referenced
            #    to a baseline (mean over cycles, or supplied), divided by the local
            #    slope. It tracks how the resonance moves, NOT where it sits.
            phase_reference = (np.mean(phi[:, ci]) if phase_baseline is None
                               else np.asarray(phase_baseline, dtype=float)[t])
            with np.errstate(divide='ignore', invalid='ignore'):
                out['freq_shift_hz'][:, t] = (phi[:, ci] - phase_reference) / slope

            # 2) detuning is ABSOLUTE: the curvature d2phi/df2 crosses zero at
            #    resonance regardless of the unknown phase offset, so the ratio
            #    curvature/slope is a self-referenced detuning estimate. The -(.)/8
            #    factor and linewidth scaling assume a near-ideal Lorentzian/arctan
            #    phase, so this is approximate; pass a fit (calibrated branch above)
            #    for a model-consistent absolute detuning instead.
            if lw is not None and N >= 3:
                with np.errstate(divide='ignore', invalid='ignore'):
                    curvature_over_slope_per_hz = d2 / slope
                detuning_linewidths = -(curvature_over_slope_per_hz * lw[t]) / 8.0
                out['detuning_linewidths'][:, t] = detuning_linewidths
                out['detuning_hz'][:, t] = detuning_linewidths * lw[t]
                out['needs_update'][:, t] = np.abs(detuning_linewidths) > threshold_linewidths

    return out


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


def _max_snr_operating_point(cal, n_grid=20001, y_max=8.0):
    """Frequency (Hz) of maximum frequency-shift responsivity for one tone.

    The best signal-to-noise operating point for reading a resonator frequency
    shift is where the response moves fastest per Hz of detuning,
    ``argmax |dS21/df|`` of the resonator's own (de-embedded) response. This is
    the geometric steepest point of the resonance circle. It is displaced from
    the fitted ``fr`` by the impedance-mismatch asymmetry (``phi``) and, for a
    driven nonlinear fit, by the Duffing detuning plus the bistable-cliff skew.
    Cable delay is deliberately excluded: it does not move with the resonator,
    so it adds responsivity to a probe sweep but none to a detector frequency
    shift. Gain is a constant scale and does not move the peak.

    Linear (``anl == 0``): the peak is at ``min |D|`` of the notch denominator,
    which has the closed form ``fr * (1 - frequency_origin_fraction)`` with
    ``frequency_origin_fraction = Im(1/Qe)/2``.

    Nonlinear (``anl > 0``): the driven model is parametrised by the smooth
    Duffing drive coordinate ``y`` rather than by frequency, because the forward
    Duffing solver's branch selection is discontinuous in ``f`` (a one-sample
    jump a fine ``f`` grid would mistake for an infinite slope). On the circle
    ``|dS21/dy| = 2/(1 + 4 y**2)`` and the constant ``|1/Qe|`` drops out of the
    argmax, so ``|dS21/df| = |dS21/dy| / |df/dy|`` depends only on ``Ql`` and
    ``anl`` (``df/dy`` via the smooth rational inverse ``invert_duffing_coordinate``).
    Near a bifurcation ``df/dy -> 0`` and the peak rides up to the cliff, as the
    max-SNR criterion implies.
    """
    foj = float(getattr(cal, 'frequency_origin_fraction', 0.0))
    anl = float(getattr(cal, 'anl', 0.0))
    if anl == 0.0:
        return float(cal.fr * (1.0 - foj))
    from .resonator import invert_duffing_coordinate
    y = np.linspace(-y_max, y_max, int(n_grid))
    x = invert_duffing_coordinate(y, anl) / cal.Ql        # (f - fr) / fr
    f = cal.fr * (1.0 + x)
    dS21_dy = 2.0 / (1.0 + 4.0 * y ** 2)
    df_dy = np.abs(np.gradient(f, y))
    slope = dS21_dy / np.where(df_dy > 0.0, df_dy, np.inf)
    return float(f[int(np.argmax(slope))])


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
    :func:`_max_snr_operating_point`. Because the demod's ``freq_shift_hz`` is
    referenced to ``fr``, it reads ``center - fr`` (also returned in
    ``reference_freqs['offset_from_fr_hz']``) as a constant baseline at the
    operating point rather than ~0; subtract it in the tracking/``needs_update``
    logic.

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
        (``center - fitted_fr``, the demod ``freq_shift_hz`` baseline);
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
            # SNR), not the fitted fr: see _max_snr_operating_point.
            center[t] = _max_snr_operating_point(cal)
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
        # magnitude/slope features. offset_from_fr_hz = center - fitted_fr is the
        # expected demod freq_shift_hz baseline at the operating point.
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
