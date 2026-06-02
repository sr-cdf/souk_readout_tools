"""
Fast-frequency-modulation demodulation helpers.

These are **pure, transport-agnostic** functions: they operate on plain arrays /
dicts (parsed stream samples + the ``tone_modulation`` state), with no
``ReadoutClient`` or socket dependency. That keeps them usable client-side on
received frames today and relocatable/shareable server-side later.

The pipeline a consumer follows is:

1. Acquire modulated samples (``ReadoutClient.get_samples`` while modulation is
   armed, or a captured continuous stream) and parse them
   (``ReadoutClient.parse_samples``) -> a ``data_dict`` carrying per-tone I/Q,
   ``modulation_point`` (1..N, 0 = off), ``modulation_settling`` and
   ``modulation_revision``.
2. :func:`group_modulation_cycles` -> per-tone complex measurements grouped by
   probe point and cycle (settling samples dropped), plus the per-point probe
   offsets (Hz) read from ``get_info('tone_modulation')``.
3. :func:`demodulate` -> per-cycle dphi/df, d2phi/df2, frequency shift and a
   detuning estimate (with a ``needs_update`` flag for the future tracking loop).

:func:`modulation_params_from_sweep` is the front of the workflow: turn a
calibration sweep into a ready-to-use ``enable_modulation`` config (per-tone
centres + per-tone probe deltas scaled to each resonator's linewidth).

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

import numpy as np


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


def group_modulation_cycles(data_dict, tone_modulation_state, reduce='mean'):
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

    Returns
    -------
    dict
        ``z`` : grouped complex measurements (see ``reduce``);
        ``offsets_hz`` : ``(N, n_tones)`` per-point probe offsets (Hz);
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

    # Per-(point,tone) probe offsets from the modulation state (user order).
    offsets_hz = np.zeros((N, n_tones), dtype=float)
    for tone in tone_modulation_state.get('tones', []):
        idx = int(tone['index'])
        if idx < n_tones:
            offsets_hz[:, idx] = np.asarray(tone['offsets_hz'], dtype=float)

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
        'revision': np.asarray(cycles_rev, dtype=int),
        'point_order': np.arange(1, N + 1),
    }


def demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None,
               circle_cal=None, phase_baseline=None, threshold_linewidths=0.1):
    """
    Demodulate grouped modulation cycles into per-tone, per-cycle quantities.

    Parameters
    ----------
    grouped : dict
        Output of :func:`group_modulation_cycles` (uses ``z`` and, if ``offsets``
        is None, ``offsets_hz`` / ``revision``). ``z`` may be ``(n_cycles, N,
        n_tones)`` (mean-reduced) or ``(n_cycles, N, n_used, n_tones)``; the
        sample axis is averaged here if present.
    offsets : numpy.ndarray or None, optional
        ``(N, n_tones)`` per-point probe offsets (Hz). ``None`` uses
        ``grouped['offsets_hz']``.
    method : {'fast', 'accurate', 'model'}, optional
        Estimator tier. ``'fast'`` uses finite differences; ``'accurate'`` uses
        a (weighted) polynomial fit of phase vs offset (handles arbitrary /
        asymmetric per-tone offsets); ``'model'`` is reserved for a calibrated
        non-ideal resonator fit (falls back to ``'accurate'`` here).
    linewidth_hz : float or array-like or None, optional
        Per-tone resonator linewidth (Hz) used to express detuning in linewidths.
        Without it, ``detuning_*`` and ``needs_update`` are NaN/False.
    circle_cal : dict or None, optional
        Per-tone resonance-circle calibration for dissipation; ``None`` -> NaN.
    phase_baseline : array-like or None, optional
        Per-tone reference phase (rad) for the frequency-shift conversion;
        ``None`` uses each tone's mean centre-point phase across cycles.
    threshold_linewidths : float, optional
        ``needs_update`` trips when ``|detuning_linewidths|`` exceeds this
        (default 0.1).

    Returns
    -------
    dict
        Per-tone, per-cycle arrays of shape ``(n_cycles, n_tones)`` unless noted:
        ``z_center``, ``dphi_df``, ``d2phi_df2``, ``freq_shift_hz``,
        ``detuning_linewidths``, ``detuning_hz``, ``needs_update`` (bool),
        ``dissipation``; plus ``revision`` ``(n_cycles,)``.
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

    for t in range(n_tones):
        f = offsets[:, t]                       # (N,) probe offsets for this tone
        zt = z[:, :, t]                          # (n_cycles, N)
        # Unwrap phase along the (few) probe points so a near-linear ramp is smooth.
        phi = np.unwrap(np.angle(zt), axis=1)    # (n_cycles, N)
        ci = int(np.argmin(np.abs(f)))           # the point nearest the centre
        out['z_center'][:, t] = zt[:, ci]

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
                c1 = np.polyfit(f, phi.T, 1)          # (2, n_cycles)
                slope = c1[0]
            if N >= 3:
                c2 = np.polyfit(f, phi.T, 2)          # (3, n_cycles)
                d2 = 2.0 * c2[0]

        out['dphi_df'][:, t] = slope
        out['d2phi_df2'][:, t] = d2

        # Frequency shift: centre-point phase deviation / local slope.
        baseline = (np.mean(phi[:, ci]) if phase_baseline is None
                    else np.asarray(phase_baseline, dtype=float)[t])
        with np.errstate(divide='ignore', invalid='ignore'):
            out['freq_shift_hz'][:, t] = (phi[:, ci] - baseline) / slope

        # Detuning needs a linewidth scale. The ratio d2/dphi has units 1/Hz;
        # for an ideal Lorentzian/arctan phase the detuning in linewidths is
        # approx -(ratio * linewidth)/8 (leading order). 'model' would calibrate
        # this per device; here it shares the leading-order form.
        if lw is not None and N >= 3:
            with np.errstate(divide='ignore', invalid='ignore'):
                ratio_per_hz = d2 / slope
            det_lw = -(ratio_per_hz * lw[t]) / 8.0
            out['detuning_linewidths'][:, t] = det_lw
            out['detuning_hz'][:, t] = det_lw * lw[t]
            out['needs_update'][:, t] = np.abs(det_lw) > threshold_linewidths

        # Dissipation requires the resonance-circle calibration (not a naive
        # |S21| derivative). Left as NaN unless a calibration is supplied.
        if circle_cal is not None:
            cal = circle_cal[t] if isinstance(circle_cal, (list, tuple)) else circle_cal.get(t)
            if cal is not None:
                centre = complex(cal.get('center', 0))
                radius = float(cal.get('radius', np.nan))
                # radial coordinate of the centre-point operating value, referred
                # to the circle -> a loss proxy (placeholder; refine with a fit).
                out['dissipation'][:, t] = np.abs(zt[:, ci] - centre) / radius

    return out


def modulation_params_from_sweep(sweep, *, n_points=3, samples_per_point=1, n_settle=1,
                                 delta_linewidths=0.25, exclude_blind=True,
                                 blind_indices=None):
    """
    Turn a calibration sweep into a ready-to-use ``enable_modulation`` config.

    For each resonator the steepest point of the phase curve (the inflection /
    operating point) and the linewidth are estimated, then a symmetric ``n_points``
    probe pattern is built with **each tone's probe spacing scaled to its own
    linewidth** (so a narrow resonator gets a smaller delta). The returned dict
    splats straight into :meth:`ReadoutClient.enable_modulation`.

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

    Returns
    -------
    dict
        ``center`` : ``(n_tones,)`` per-tone centre frequencies (Hz);
        ``offsets`` : ``(n_points, len(mod_indices))`` per-tone probe offsets (Hz);
        ``mod_indices`` : modulated (resonator) tone indices;
        ``samples_per_point`` / ``n_settle`` : as requested;
        ``linewidth_hz`` : ``(len(mod_indices),)`` estimated linewidths (for demod);
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

    center = np.zeros(n_tones, dtype=float)
    linewidth = np.zeros(n_tones, dtype=float)
    for t in range(n_tones):
        ft = f[:, t]
        phi = np.unwrap(np.angle(z[:, t]))
        slope = np.gradient(phi, ft)
        k = int(np.argmax(np.abs(slope)))     # steepest point = inflection ~ f0
        center[t] = ft[k]
        # For phi = -2*arctan(2(f-f0)/w) the peak slope magnitude is 4/w -> w = 4/|slope_max|.
        smax = np.abs(slope[k])
        linewidth[t] = (4.0 / smax) if smax > 0 else (ft[-1] - ft[0])

    # Symmetric probe pattern per tone: linspace(-delta, +delta, n_points) with
    # delta scaled to each tone's linewidth.
    pattern = np.linspace(-1.0, 1.0, n_points)            # e.g. [-1,1] or [-1,0,1]
    deltas = delta_linewidths * linewidth[mod_indices]    # (len(mod_indices),)
    offsets = pattern[:, None] * deltas[None, :]          # (n_points, len(mod_indices))

    summary_lines = [
        f'modulation_params_from_sweep: {len(mod_indices)} modulated tones '
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

    return {
        'center': center,
        'offsets': offsets,
        'mod_indices': mod_indices,
        'samples_per_point': int(samples_per_point),
        'n_settle': int(n_settle),
        'linewidth_hz': linewidth[mod_indices],
        'summary': '\n'.join(summary_lines),
    }
