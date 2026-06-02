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
   from ``get_info('tone_modulation')``.
3. :func:`demodulate` -> per-cycle dphi/df, d2phi/df2, frequency shift and a
   detuning estimate (with a ``needs_update`` flag for the future tracking loop).

Physics caveats (kept honest):
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
    (pc2, points2, settling2, revisions2, z2) on success, or ``None`` if the
    cadence phase could not be anchored (caller then leaves the data unfilled).
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
    last_rev = int(revisions[0]) if len(revisions) else 0
    for j, c in enumerate(full):
        src = present.get(int(c))
        if src is not None:
            z2[j] = z[src]
            points2[j] = int(points[src])
            settling2[j] = int(settling[src])
            last_rev = int(revisions[src])
            revisions2[j] = last_rev
        else:
            idx = (int(c) - c0) % period            # modulation sub-index within the cycle
            points2[j] = (idx // spp) % int(n_points) + 1
            settling2[j] = 1 if (idx % spp) < n_settle else 0
            revisions2[j] = last_rev                # carry the last known revision
    return full, points2, settling2, revisions2, z2


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


def group_cycles(data_dict, tone_modulation_state, reduce='mean', on_missing='notify'):
    """
    Group modulated samples by probe point and modulation cycle.

    Walks the sample stream, drops settling samples, and collects the complex
    I/Q at each of the N probe points into successive cycles. A new cycle is
    detected when the point index wraps back down (e.g. N -> 1). The gap-free
    ``packet_counter`` guarantees the samples are contiguous so the grouping is
    unambiguous.

    Parameters
    ----------
    data_dict : dict
        Parsed samples (``ReadoutClient.parse_samples``) with ``modulation_point``
        (1..N, 0 = off), ``modulation_settling``, ``modulation_revision`` and the
        per-tone ``i_data`` / ``q_data``.
    tone_modulation_state : dict
        ``get_info('tone_modulation')`` payload, used for ``num_points`` and the
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

    Returns
    -------
    dict
        ``z`` : grouped complex measurements (see ``reduce``);
        ``offsets_hz`` : ``(N, n_tones)`` per-point probe offsets (Hz);
        ``freq_hz`` : ``(N, n_tones)`` absolute probe frequencies (centre + offset);
        ``revision`` : ``(n_cycles,)`` representative config revision per cycle;
        ``point_order`` : the point indices ``1..N``.
    """
    n_tones = int(data_dict['num_tones'])
    N = int(tone_modulation_state['num_points'])
    if N < 1:
        raise ValueError('tone_modulation_state has num_points < 1; is modulation armed?')

    points = np.asarray(data_dict['modulation_point'], dtype=int)
    settling = np.asarray(data_dict['modulation_settling'], dtype=int)
    revisions = np.asarray(data_dict['modulation_revision'], dtype=int)
    z = _tone_iq(data_dict, n_tones)

    # Detect dropped accumulations via the packet counter, and either notify or
    # fill the gaps with NaN placeholders (their tag inferred from the cadence).
    pc = data_dict.get('packet_counter')
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
                    pc, points, settling, revisions, z = filled
            elif on_missing != 'notify':
                raise ValueError(f"on_missing must be 'notify' or 'fill', got {on_missing!r}")

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

    cycles_z = []
    cycles_rev = []
    cur = {p: [] for p in range(1, N + 1)}
    cur_rev = []
    prev_p = None

    def _flush():
        # Commit the current cycle only if every point was seen.
        if all(len(cur[p]) > 0 for p in range(1, N + 1)):
            if reduce == 'mean':
                arr = np.stack([np.mean(np.stack(cur[p], axis=0), axis=0)
                                for p in range(1, N + 1)], axis=0)   # (N, n_tones)
            else:
                # retain sample axis; truncate to the common minimum dwell count
                m = min(len(cur[p]) for p in range(1, N + 1))
                arr = np.stack([np.stack(cur[p][:m], axis=0)
                                for p in range(1, N + 1)], axis=0)    # (N, n_used, n_tones)
            cycles_z.append(arr)
            cycles_rev.append(int(np.bincount(cur_rev).argmax()) if cur_rev else 0)
        for p in range(1, N + 1):
            cur[p] = []
        cur_rev.clear()

    for k in range(len(points)):
        p = int(points[k])
        if p == 0:               # not modulating
            continue
        if int(settling[k]):     # drop transient samples
            prev_p = p
            continue
        if prev_p is not None and p < prev_p:   # wrapped -> cycle boundary
            _flush()
        cur[p].append(z[k])
        cur_rev.append(int(revisions[k]))
        prev_p = p
    _flush()

    return {
        'z': np.stack(cycles_z, axis=0) if cycles_z else np.empty((0, N, n_tones), dtype=complex),
        'offsets_hz': offsets_hz,
        'freq_hz': freq_hz,
        'revision': np.asarray(cycles_rev, dtype=int),
        'point_order': np.arange(1, N + 1),
    }


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
      and the **exact Möbius inversion** yields the frequency shift and
      dissipation directly (no linewidth argument needed — the calibration
      carries ``fr``/``Ql``). This is the recommended basis.

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
        calibrated/centred basis with exact frequency-shift and dissipation.
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
        ``dissipation``; plus ``revision`` ``(n_cycles,)``. In the calibrated
        basis ``freq_shift_hz`` / ``detuning_hz`` are the centre tone's offset
        from resonance (Hz) and ``dissipation`` is the fractional loss shift.
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
            # conditioned and the exact Möbius inversion gives df / dissipation.
            zc = cal.deembed_sweep(freq_hz[:, t], zt)        # (n_cycles, N), centred
            phi = np.unwrap(np.angle(zc), axis=1)
            df_pts, dd_pts = cal.to_frequency_dissipation(zc)  # (n_cycles, N) each
            lw_t = cal.fr / cal.Ql                            # linewidth (Hz) from the fit
            out['freq_shift_hz'][:, t] = df_pts[:, ci]        # centre offset from resonance
            out['detuning_hz'][:, t] = df_pts[:, ci]
            out['detuning_linewidths'][:, t] = df_pts[:, ci] / lw_t
            out['needs_update'][:, t] = np.abs(df_pts[:, ci] / lw_t) > threshold_linewidths
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
            # Model-free frequency shift + leading-order detuning (needs linewidth).
            baseline = (np.mean(phi[:, ci]) if phase_baseline is None
                        else np.asarray(phase_baseline, dtype=float)[t])
            with np.errstate(divide='ignore', invalid='ignore'):
                out['freq_shift_hz'][:, t] = (phi[:, ci] - baseline) / slope
            # ratio d2/dphi has units 1/Hz; for an ideal Lorentzian/arctan phase
            # the detuning in linewidths is approx -(ratio * linewidth)/8.
            if lw is not None and N >= 3:
                with np.errstate(divide='ignore', invalid='ignore'):
                    ratio_per_hz = d2 / slope
                det_lw = -(ratio_per_hz * lw[t]) / 8.0
                out['detuning_linewidths'][:, t] = det_lw
                out['detuning_hz'][:, t] = det_lw * lw[t]
                out['needs_update'][:, t] = np.abs(det_lw) > threshold_linewidths

    return out


def params_from_sweep(sweep, *, n_points=3, samples_per_point=1, n_settle=1,
                                 delta_linewidths=0.25, exclude_blind=True,
                                 blind_indices=None, deembed=True, fits=None):
    """
    Turn a calibration sweep into a ready-to-use ``enable_modulation`` config.

    For each resonator a centre (the steepest / inflection operating point) and a
    linewidth are obtained, then a symmetric ``n_points`` probe pattern is built
    with **each tone's probe spacing scaled to its own linewidth** (so a narrow
    resonator gets a smaller delta). The returned dict splats straight into
    :meth:`ReadoutClient.enable_modulation`.

    This package does **not** fit resonators itself (so no fit options have to be
    threaded through it). With ``deembed=True`` (default) you must therefore pass
    pre-computed ``fits`` (fit the sweep yourself, e.g.
    :func:`souk_readout_tools.fitting.fit_resonance`); a per-tone
    :class:`souk_readout_tools.resonator.ResonatorCalibration` is built from each,
    giving a model-consistent centre/linewidth **and** the de-embedding /
    phase-centring calibration that :func:`demodulate` uses for the
    well-conditioned centred basis (and dissipation). With ``deembed=False`` (or
    for any modulated tone lacking a supplied fit) the function falls back to the
    **model-free phase-slope estimate** from the sweep (centre = steepest point,
    linewidth from the peak slope) and that tone gets no calibration.

    Parameters
    ----------
    sweep : dict
        Sweep data with ``f`` and ``z`` arrays of shape ``(n_sweep_points,
        n_tones)`` (frequency in Hz and complex S21). ``blind_indices`` may also
        be carried here.
    n_points : int, optional
        Number of probe points per cycle (>=2; 3 enables the curvature/detuning
        estimate). Default 3.
    samples_per_point : int, optional
        Dwell (samples per point per cycle). Default 1.
    n_settle : int, optional
        Settling samples per point. Default 1.
    delta_linewidths : float, optional
        Probe half-span in units of each tone's linewidth (kept small to stay
        near the high-slope inflection). Default 0.25.
    exclude_blind : bool, optional
        Drop blind tones from ``mod_indices``. Default True.
    blind_indices : array-like or None, optional
        Blind-tone indices; overrides ``sweep['blind_indices']`` if given.
    deembed : bool, optional
        Build de-embedding / phase-centring calibrations (recommended). When True,
        ``fits`` is **required**. When False, only the model-free phase-slope
        estimate is produced (no calibration). Default True.
    fits : sequence, dict, or None
        Pre-computed per-tone fits. **Required when** ``deembed=True``. A list
        (length ``n_tones``; entries ``None`` for tones to leave model-free) or
        ``{tone_index: fit}``; each entry is a :class:`fitting.FitResult` or a
        ready :class:`resonator.ResonatorCalibration`. Doing the fit outside this
        package keeps the fitter's options out of the modulation API.

    Returns
    -------
    dict
        ``center`` : ``(n_tones,)`` per-tone centre frequencies (Hz);
        ``offsets`` : ``(n_points, len(mod_indices))`` per-tone probe offsets (Hz);
        ``mod_indices`` : modulated (resonator) tone indices;
        ``samples_per_point`` / ``n_settle`` : as requested;
        ``linewidth_hz`` : ``(len(mod_indices),)`` linewidths (for model-free demod);
        ``calibration`` : ``{tone_index: ResonatorCalibration}`` for tones with a
        supplied fit (empty if ``fits`` is None);
        ``summary`` : a human-readable multi-line summary string.
    """
    if n_points < 2:
        raise ValueError('n_points must be >= 2')
    f = np.asarray(sweep['f'], dtype=float)
    z = np.asarray(sweep['z'])
    if f.ndim == 1:
        f = f[:, None]
        z = z[:, None]
    n_sweep, n_tones = f.shape

    if blind_indices is None:
        blind_indices = sweep.get('blind_indices', []) if isinstance(sweep, dict) else []
    blind = set(int(b) for b in blind_indices)
    mod_indices = [i for i in range(n_tones) if not (exclude_blind and i in blind)]

    # We never fit here. deembed=True therefore requires the caller to supply fits.
    if deembed and fits is None:
        raise ValueError(
            'deembed=True requires precomputed `fits` (fit the sweep yourself, '
            'e.g. fitting.fit_resonance, and pass fits=...). Use deembed=False for '
            'the model-free phase-slope estimate.')

    def _provided_fit(t):
        """The caller-supplied fit/calibration for tone ``t`` (or None)."""
        if fits is None:
            return None
        if isinstance(fits, dict):
            return fits.get(t)
        try:
            return fits[t]
        except (IndexError, KeyError, TypeError):
            return None

    # Only need the calibration class when fits are supplied (to wrap FitResults).
    ResonatorCalibration = None
    if fits is not None:
        from souk_readout_tools.resonator import ResonatorCalibration

    center = np.zeros(n_tones, dtype=float)
    linewidth = np.zeros(n_tones, dtype=float)
    calibration = {}
    for t in range(n_tones):
        ft = f[:, t]
        provided = _provided_fit(t)
        if provided is not None:
            # Build the calibration from the supplied fit (already a calibration,
            # or a FitResult). No fitting happens here.
            cal = (provided if hasattr(provided, 'deembed_sweep')
                   else ResonatorCalibration.from_fit(provided))
            center[t] = cal.fr
            linewidth[t] = cal.fr / cal.Ql
            if t in mod_indices:
                calibration[t] = cal
        else:
            # Model-free estimate: steepest point = inflection ~ f0; for
            # phi = -2*arctan(2(f-f0)/w) the peak slope magnitude is 4/w.
            phi = np.unwrap(np.angle(z[:, t]))
            slope = np.gradient(phi, ft)
            k = int(np.argmax(np.abs(slope)))
            center[t] = ft[k]
            smax = np.abs(slope[k])
            linewidth[t] = (4.0 / smax) if smax > 0 else (ft[-1] - ft[0])

    # Symmetric probe pattern per tone: linspace(-delta, +delta, n_points) with
    # delta scaled to each tone's linewidth.
    pattern = np.linspace(-1.0, 1.0, n_points)            # e.g. [-1,1] or [-1,0,1]
    deltas = delta_linewidths * linewidth[mod_indices]    # (len(mod_indices),)
    offsets = pattern[:, None] * deltas[None, :]          # (n_points, len(mod_indices))

    summary_lines = [
        f'params_from_sweep: {len(mod_indices)} modulated tones '
        f'({n_tones - len(mod_indices)} excluded), {n_points} points, '
        f'dwell={samples_per_point}, n_settle={n_settle}, delta={delta_linewidths} linewidths',
    ]
    for j, t in enumerate(mod_indices[:8]):
        summary_lines.append(
            f'  tone {t:4d}: center={center[t]/1e6:.6f} MHz  '
            f'linewidth={linewidth[t]/1e3:.2f} kHz  '
            f'delta=+/-{deltas[j]/1e3:.3f} kHz')
    if len(mod_indices) > 8:
        summary_lines.append(f'  ... (+{len(mod_indices) - 8} more)')

    if fits is not None:
        summary_lines.append(
            f'  de-embed calibration: {len(calibration)}/{len(mod_indices)} tones (from supplied fits)')

    return {
        'center': center,
        'offsets': offsets,
        'mod_indices': mod_indices,
        'samples_per_point': int(samples_per_point),
        'n_settle': int(n_settle),
        'linewidth_hz': linewidth[mod_indices],
        'calibration': calibration,
        'summary': '\n'.join(summary_lines),
    }
