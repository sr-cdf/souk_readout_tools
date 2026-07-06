"""
Timestream data plotting functions.

Supports parsed timestream data (from ReadoutClient.parse_samples()) in
multiple formats: I/Q, magnitude/phase, frequency/dissipation, and their
PSDs. Includes a debugging overlay of timestream points on the resonance
circle from sweep data, with optional deembedding or phase centering.
"""

import numpy as np
from collections.abc import Mapping

from ..noise import (fractional_frequency_and_dissipation_timestreams,
                     remove_blind_tone_common_modes,
                     remove_common_modes_svd)
from ._common import (_get_pyplot, _compute_mag_phase,
                       _apply_deembed, _apply_phase_center,
                       _resolve_label,
                       _normalise_iq, _compute_mag_phase_units, UNITS,
                       _is_calibrated_magnitude_unit,
                       _canonical_units, _validate_reference_plane)
from ._psd import compute_psd, log_bin_psd

# PTP clock rate: telescope_time counts per second
TT_CLOCK_HZ = 30720000


def _build_x_axis(ts_data, x_axis='time'):
    """Build x-axis array and label from timestream data.

    Args:
        ts_data: parsed timestream dict.
        x_axis: 'time' (seconds from sample_rate), 'sample' (sample index),
            'acc_count' (packet_counter), or 'telescope_time' (PTP seconds).

    Returns:
        (x_values, x_label) tuple.
    """
    n = ts_data.get('num_samples', len(next(iter(ts_data['i_data'].values()))))
    if x_axis == 'time':
        return np.arange(n) / ts_data['sample_rate'], 'Time (s)'
    elif x_axis == 'sample':
        return np.arange(n), 'Sample number'
    elif x_axis == 'acc_count':
        cnt = np.asarray(ts_data['packet_counter'], dtype=float)
        return cnt, 'Accumulation count'
    elif x_axis == 'telescope_time':
        tt = np.asarray(ts_data['telescope_time'], dtype=float) / TT_CLOCK_HZ
        return tt, 'Telescope time (s)'
    else:
        raise ValueError(
            f"Unknown x_axis '{x_axis}'. "
            "Use 'time', 'sample', 'acc_count', or 'telescope_time'.")


def _get_tone_data(ts_data, tones=None):
    """
    Extract I/Q arrays for selected tones from parsed timestream data.

    Args:
        ts_data: dict from parse_samples() with 'i_data', 'q_data'.
        tones: List of tone indices (int) or None for first tone only.

    Returns:
        List of (tone_key, i_array, q_array) tuples.
    """
    i_data = ts_data['i_data']
    q_data = ts_data['q_data']
    tone_keys = sorted(i_data.keys())

    if tones is None:
        tones = [0]
    selected = []
    for t in tones:
        key = tone_keys[t]
        selected.append((key, np.array(i_data[key], dtype=float),
                         np.array(q_data[key], dtype=float)))
    return selected


def _get_modulated_probe_frequencies(ts_data, tone_index, fallback_frequency,
                                     n_samples):
    """Return per-sample probe frequencies for a modulated timestream tone."""
    freqs = np.full(int(n_samples), float(fallback_frequency), dtype=float)
    info = ts_data.get('info') if isinstance(ts_data, dict) else None
    if not isinstance(info, dict):
        return freqs

    from souk_readout_tools.modulation import active_modulation_state
    state = active_modulation_state(info)   # software OR firmware-slot state
    points = ts_data.get('modulation_point') if isinstance(ts_data, dict) else None
    if not isinstance(state, dict) or points is None:
        return freqs

    try:
        points = np.asarray(points, dtype=int)
    except (TypeError, ValueError):
        return freqs
    if len(points) != len(freqs):
        return freqs

    tone_state = None
    for tone in state.get('tones', []):
        if not isinstance(tone, dict):
            continue
        try:
            if int(tone.get('index')) == int(tone_index):
                tone_state = tone
                break
        except (TypeError, ValueError):
            continue
    if tone_state is None:
        return freqs

    try:
        center = float(tone_state.get('center_hz', fallback_frequency))
        offsets = np.asarray(tone_state['offsets_hz'], dtype=float)
    except (KeyError, TypeError, ValueError):
        return freqs
    if offsets.ndim != 1 or len(offsets) == 0:
        return freqs

    freqs.fill(center)
    valid = (points > 0) & (points <= len(offsets))
    freqs[valid] = center + offsets[points[valid] - 1]
    return freqs


def _visible_modulation_sample_mask(ts_data, n_samples,
                                    hide_modulation_settling_points):
    """Return a plot mask, optionally dropping modulation settling samples."""
    mask = np.ones(int(n_samples), dtype=bool)
    if not hide_modulation_settling_points or not isinstance(ts_data, dict):
        return mask

    settling = ts_data.get('modulation_settling')
    if settling is None:
        return mask
    try:
        settling = np.asarray(settling, dtype=int)
    except (TypeError, ValueError):
        return mask
    if len(settling) != len(mask):
        return mask
    return settling == 0


def _get_sweep_trace(sweep_data, tone_index):
    """Extract the sweep frequency and I/Q trace for a timestream tone."""
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])

    is_wideband = sweep_data.get('wideband_sweep', False)
    if is_wideband or sf.shape[0] == 1:
        return sf[0], si[0].copy(), sq[0].copy()

    _, n_tones = sf.shape
    if tone_index >= n_tones:
        raise ValueError(
            f"Tone {tone_index} not in sweep data ({n_tones} tones)")
    return sf[:, tone_index], si[:, tone_index].copy(), sq[:, tone_index].copy()


def _resolve_reference_tone_frequency(ts_data, tone_idx, sweep_f,
                                       reference_tone_frequency):
    """Resolve the timestream tone frequency for freq/diss computation.

    reference_tone_frequency:
        None: look up ``ts_data['info']['tones']['frequencies_hz'][tone_idx]``.
            Falls back to ``np.mean(sweep_f)`` with a warning if unavailable.
        scalar: used directly for every tone.
        mapping: ``{tone_idx: frequency_hz}`` or ``{'0000': frequency_hz}``.
        array-like: indexed by absolute tone index.
    """
    if reference_tone_frequency is None:
        info = ts_data.get('info')
        if isinstance(info, dict):
            tf = info.get('tones', {}).get('frequencies_hz')
            if tf is not None:
                try:
                    return float(np.asarray(tf)[tone_idx])
                except (IndexError, ValueError, TypeError):
                    pass
        import warnings
        warnings.warn(
            f"Tone {tone_idx} has no frequency in ts_data['info']; "
            "falling back to the midpoint of the sweep range. Pass "
            "reference_tone_frequency to set this explicitly.",
            stacklevel=3)
        return float(np.mean(sweep_f))
    if isinstance(reference_tone_frequency, Mapping):
        for key in (tone_idx, f"{tone_idx:04d}"):
            if key in reference_tone_frequency:
                return float(reference_tone_frequency[key])
        raise ValueError(
            f"reference_tone_frequency mapping missing tone {tone_idx}")

    if np.ndim(reference_tone_frequency):
        try:
            return float(np.asarray(reference_tone_frequency, dtype=float)[tone_idx])
        except IndexError as exc:
            raise ValueError(
                "array-like reference_tone_frequency is indexed by absolute "
                f"tone index; tone {tone_idx} is missing. Use a mapping for "
                "non-contiguous selected tones."
            ) from exc

    return float(reference_tone_frequency)


def _compute_freq_diss(ts_data, tone_key, i_arr, q_arr, sweep_data,
                       reference_tone_frequency=None, smooth_window_hz=1000,
                       conversion_method='linearized', calibrations=None):
    """
    Compute fractional frequency and dissipation from timestream + sweep.

    Uses the canonical resonator conversion helpers. ``conversion_method``
    selects the historical local linear estimate or fitted-model inversion.
    """
    from ..resonator import (
        calibration_for_tone,
        interpolate_complex_trace,
        linearized_frequency_and_dissipation,
    )

    z_ts = i_arr + 1j * q_arr
    tone_idx = int(tone_key)

    sweep_f, sweep_i, sweep_q = _get_sweep_trace(sweep_data, tone_idx)
    sweep_z = sweep_i + 1j * sweep_q

    tone_freq = _resolve_reference_tone_frequency(
        ts_data, tone_idx, sweep_f, reference_tone_frequency)

    if conversion_method == 'linearized':
        frac_f, frac_d = linearized_frequency_and_dissipation(
            sweep_f, sweep_z, tone_freq, z_ts,
            smooth_window_hz=smooth_window_hz)
    elif conversion_method in ('mobius', 'circle'):
        calibration = calibration_for_tone(calibrations, tone_idx)
        if calibration is None:
            raise ValueError(
                f"conversion_method={conversion_method!r} requires a "
                f"calibration for tone {tone_idx}")
        reference_s21 = interpolate_complex_trace(sweep_f, sweep_z, tone_freq)
        df_hz, frac_d = calibration.convert_referenced_raw_iq(
            tone_freq, z_ts, reference_s21, method=conversion_method)
        frac_f = df_hz / tone_freq
    else:
        raise ValueError(
            "conversion_method must be 'linearized', 'mobius', or 'circle'")

    return frac_f, frac_d


def _has_precomputed_freq_diss(ts_data):
    """Return True when a timestream dict already carries demodulated freq/loss."""
    return any(key in ts_data for key in (
        'resonance_shift_hz',
        'frequency_shift_hz',
        'frequency_hz',
    ))


def _sample_tone_column(ts_data, key, tone_idx):
    """Extract a tone column from sample-major or tone-major arrays."""
    arr = np.asarray(ts_data[key])
    if arr.ndim == 1:
        if tone_idx != 0:
            raise ValueError(f"{key!r} is 1D and contains only tone 0")
        return arr
    if arr.ndim != 2:
        raise ValueError(f"{key!r} must be 1D or 2D, got {arr.ndim}D")

    n_samples = ts_data.get('num_samples')
    if n_samples is not None and arr.shape[0] == int(n_samples):
        return arr[:, tone_idx]
    if tone_idx < arr.shape[0]:
        return arr[tone_idx]
    if tone_idx < arr.shape[1]:
        return arr[:, tone_idx]
    raise ValueError(f"Tone {tone_idx} not present in {key!r} shape {arr.shape}")


def _precomputed_freq_diss(ts_data, tone_key, reference_tone_frequency=None):
    """Read demodulated fractional frequency and dissipation from ``ts_data``."""
    tone_idx = int(tone_key)
    freq_key = next(
        key for key in ('resonance_shift_hz', 'frequency_shift_hz', 'frequency_hz')
        if key in ts_data)
    freq_hz = _sample_tone_column(ts_data, freq_key, tone_idx)

    info = ts_data.get('info') if isinstance(ts_data, dict) else None
    tone_freq = None
    if reference_tone_frequency is None and isinstance(info, dict):
        freqs = info.get('tones', {}).get('frequencies_hz')
        if freqs is not None:
            try:
                tone_freq = float(np.asarray(freqs, dtype=float)[tone_idx])
            except (IndexError, TypeError, ValueError):
                tone_freq = None
    elif reference_tone_frequency is not None:
        if isinstance(reference_tone_frequency, Mapping):
            for key in (tone_idx, tone_key):
                if key in reference_tone_frequency:
                    tone_freq = float(reference_tone_frequency[key])
                    break
        elif np.ndim(reference_tone_frequency):
            try:
                tone_freq = float(np.asarray(reference_tone_frequency, dtype=float)[tone_idx])
            except IndexError as exc:
                raise ValueError(
                    "array-like reference_tone_frequency is indexed by absolute "
                    f"tone index; tone {tone_idx} is missing. Use a mapping for "
                    "non-contiguous selected tones."
                ) from exc
        else:
            tone_freq = float(reference_tone_frequency)

    if tone_freq is None or tone_freq == 0.0:
        raise ValueError(
            "Precomputed frequency timestreams need tone frequencies in "
            "ts_data['info']['tones']['frequencies_hz'] or an explicit "
            "reference_tone_frequency to plot fractional frequency.")

    frac_f = freq_hz / tone_freq
    if 'dissipation' in ts_data:
        frac_d = _sample_tone_column(ts_data, 'dissipation', tone_idx)
    else:
        frac_d = np.full_like(frac_f, np.nan, dtype=float)
    return frac_f, frac_d


def _dissipation_axis_label(conversion_method, *, psd=False):
    """Return a label that reflects the selected loss-coordinate convention."""
    if conversion_method in ('linearized', 'mobius'):
        label = r'Matched dissipation quadrature $\Delta(1 / 2Q_i)$'
    else:
        label = r'Radial loss proxy change $\Delta\rho$'
    return f'{label} PSD' if psd else label


def _apply_transforms(z, deembed, phase_center):
    """Apply deembed and/or phase_center to a complex array (no frequencies).

    Returns (z_out, deembed_params, phase_center_params).
    """
    d_params = None
    pc_params = None
    if deembed:
        z, d_params = _apply_deembed(None, z, deembed)
    if phase_center:
        z, pc_params = _apply_phase_center(z, phase_center)
    return z, d_params, pc_params


def _apply_tone_transforms(z, tone_idx, deembed, phase_center, *,
                           sweep_data=None, tone_frequency=None,
                           units='raw', info=None, config=None,
                           reference_plane='adc_input',
                           calibration_cache=None):
    """Apply timestream transforms, using sweep-derived params when possible."""
    d_params = None
    pc_params = None
    sweep_f = None
    z_sweep = None

    if sweep_data is not None and (deembed is True or phase_center is True):
        sweep_f, sw_i, sw_q = _get_sweep_trace(sweep_data, tone_idx)
        if tone_frequency is None:
            tone_frequency = float(np.mean(sweep_f))
        if units != 'raw':
            sweep_info = info if isinstance(info, dict) else sweep_data.get('info')
            sw_i, sw_q, _, _, _, _ = _normalise_iq(
                sw_i, sw_q, units, sweep_info, config=config,
                reference_plane=reference_plane, frequencies=sweep_f,
                calibration_cache=calibration_cache)
        z_sweep = sw_i + 1j * sw_q

    if deembed:
        if deembed is True:
            if z_sweep is None:
                raise ValueError(
                    "deembed=True for timestream data requires sweep_data "
                    "or a pre-computed deembed params dict.")
            z_sweep, d_params = _apply_deembed(sweep_f, z_sweep, True)
            z, _ = _apply_deembed(None, z, d_params, frequency=tone_frequency)
        else:
            z, d_params = _apply_deembed(
                None, z, deembed, frequency=tone_frequency)
            if z_sweep is not None and isinstance(deembed, dict):
                z_sweep, _ = _apply_deembed(
                    None, z_sweep, deembed, frequency=sweep_f)

    if phase_center:
        if phase_center is True and z_sweep is not None:
            z_sweep, pc_params = _apply_phase_center(z_sweep, True)
            z, _ = _apply_phase_center(z, pc_params)
        else:
            z, pc_params = _apply_phase_center(z, phase_center)

    return z, d_params, pc_params


def _transform_title_suffix(deembed, phase_center):
    """Return a parenthesised title suffix describing active transforms."""
    parts = []
    if deembed:
        parts.append('deembedded')
    if phase_center:
        parts.append('phase-centered')
    return f' ({", ".join(parts)})' if parts else ''


def _filter_psd_dc_point(f, p, include_dc_point):
    """Drop the zero-frequency PSD point for log-x plotting by default."""
    if include_dc_point:
        return f, p

    f = np.asarray(f)
    p = np.asarray(p)
    non_dc = f != 0
    return f[non_dc], p[non_dc]


def _prepare_psd_for_log_plot(f, p, include_dc_point, log_bin, bins_per_decade):
    """Apply PSD display filters shared by all PSD panels."""
    f, p = _filter_psd_dc_point(f, p, include_dc_point)
    if log_bin:
        f, p = log_bin_psd(f, p, bins_per_decade=bins_per_decade)
    return f, p


def _match_y_limits(*axes):
    """Set all axes to the combined current y-limits."""
    limits = [ax.get_ylim() for ax in axes]
    ymin = min(limit[0] for limit in limits)
    ymax = max(limit[1] for limit in limits)
    for ax in axes:
        ax.set_ylim(ymin, ymax)


def _compute_cleaned_freq_diss(
        ts_data, sweep_data, plotted_tones, decorrelate_modes,
        decorrelate_tones, blind_tone_modes, blind_tones,
        reference_tone_frequency, smooth_window_hz, conversion_method,
        calibrations):
    """Return reusable raw rows and sequentially cleaned freq/diss rows."""
    analysis_tones = plotted_tones
    if decorrelate_modes:
        info = ts_data.get('info')
        metadata = info.get('tones', {}) if isinstance(info, dict) else {}
        regular_tones = metadata.get('regular_indices')
        analysis_tones = decorrelate_tones
        if analysis_tones is None:
            analysis_tones = regular_tones if (
                regular_tones is not None and len(regular_tones)
            ) else None

    cleaned_ts_data = ts_data
    if blind_tone_modes:
        cleaned_ts_data, blind_info = remove_blind_tone_common_modes(
            ts_data,
            n_modes=blind_tone_modes,
            regular_tones=analysis_tones,
            blind_tones=blind_tones,
            return_info=True,
        )
        if analysis_tones is None:
            analysis_tones = blind_info['regular_tones']

    converted = fractional_frequency_and_dissipation_timestreams(
        cleaned_ts_data,
        sweep_data,
        tones=analysis_tones,
        reference_tone_frequency=reference_tone_frequency,
        smooth_window_hz=smooth_window_hz,
        method=conversion_method,
        calibrations=calibrations,
    )
    if decorrelate_modes:
        frequency = remove_common_modes_svd(
            converted['frequency'], decorrelate_modes)
        dissipation = remove_common_modes_svd(
            converted['dissipation'], decorrelate_modes)
    else:
        frequency = converted['frequency']
        dissipation = converted['dissipation']

    raw = {}
    if not blind_tone_modes:
        raw = {
            int(tone_index): (converted['frequency'][row],
                              converted['dissipation'][row])
            for row, tone_index in enumerate(converted['tone_indices'])
        }
    cleaned = {
        int(tone_index): (frequency[row], dissipation[row])
        for row, tone_index in enumerate(converted['tone_indices'])
    }
    return raw, cleaned


def plot_timestream(ts_data, format='iq_vs_t', tones=None,
                    deembed=False, phase_center=False,
                    sweep_data=None, reference_tone_frequency=None,
                    fig=None, label=None,
                    units='raw', config=None, reference_plane='adc_input',
                    x_axis='time', unwrap_phase=True,
                    smooth_window_hz=1000, conversion_method='linearized',
                    calibrations=None, **kwargs):
    """
    Plot timestream data in various formats.

    Args:
        ts_data: dict from parse_samples() with 'i_data', 'q_data',
            'sample_rate', 'num_samples'.
        format: 'iq' | 'iq_vs_t' | 'magphase' | 'freq_diss'
        tones: List of tone indices (int) to plot, or None for [0].
        deembed: bool or deembed params dict.  Applies true RF
            deembedding (baseline normalisation).  For timestream data,
            ``True`` derives params from the matching ``sweep_data`` trace;
            otherwise pass a pre-computed params dict from
            ``resonator.deembed()``.
        phase_center: bool or phase-centering params dict.  Applies
            circle centering and rotation.  For timestream data, ``True``
            derives params from the matching ``sweep_data`` trace when
            available; without ``sweep_data`` it computes from the timestream
            directly.  Pass a pre-computed params dict from
            ``resonator.phase_center()`` to apply specific params.  Applied
            after deembedding when both are set.
        sweep_data: Sweep data dict. Required for 'freq_diss' format and
            used as the parameter source for ``deembed=True`` or
            ``phase_center=True`` on I/Q and mag/phase timestream plots.
        reference_tone_frequency: Tone frequency to use for the freq/diss
            calculation (``format='freq_diss'``).  ``None`` (default) looks
            up the per-tone frequency from
            ``ts_data['info']['tones']['frequencies_hz']`` — for on-resonance
            timestreams this is the actual tone frequency.  Pass a scalar to
            apply one value to every selected tone, or a ``{tone_idx: hz}``
            dict for per-tone overrides (useful for off-resonance analyses).
        smooth_window_hz: Smoothing window passed to
            the linearized resonator conversion for ``format='freq_diss'``.
            Pass ``None`` or ``0`` to disable sweep smoothing.
        conversion_method: ``'linearized'`` (default), ``'mobius'``, or
            ``'circle'`` for ``format='freq_diss'``.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``.
        fig: Existing figure. If None, create new.
        label: Legend label. If None, uses an auto-incrementing index.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator units, no normalisation.
            'peak' - normalise to the peak magnitude of the data.
            'adc_units' / 'adc' - linear ADC units.  The firmware
                accumulator path is undone and, for non-ADC reference planes,
                the RX-chain calibration is removed.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs' - dB relative to ADC full-scale, optionally referred to
                ``reference_plane`` for magnitude plots.
            'dbm' - estimated power in dBm at ``reference_plane``.
            All options except 'raw' and 'peak' require
            'info' in ts_data.
        config: Config dict (needed when the timestream metadata does not
            include the run config, or for non-default rx_mix_scale in older
            data).
        reference_plane: Reference plane for calibrated magnitude plotting.
            Use 'adc_input' (default), 'cryostat_output', or 'detector'.
            'cryostat_output'/'detector' deembed the RX analog chain using
            calibration entries in ``config`` or ``ts_data['info']`` at each
            tone's frequency; falls back to 'adc_input' with a warning if
            that cal is not available.  ``units='raw'`` is accumulator units
            only; use ``units='adc_units'`` for a linear ADC-unit view
            referred to a detector/cryostat plane.
        x_axis: X-axis for time-domain formats.  One of:
            'time' (default) - seconds from sample rate.
            'sample' - sample index (0, 1, 2, ...).
            'acc_count' - accumulation counter (packet_counter).
            'telescope_time' - PTP telescope time in seconds.
        unwrap_phase: bool, optional.  Unwrap the phase in
            ``format='magphase'``.  Default ``True`` preserves the previous
            behaviour; pass ``False`` to show wrapped phase.
        **kwargs: Passed to matplotlib plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    custom_title = kwargs.pop('title', None)
    selected = _get_tone_data(ts_data, tones)
    sample_rate = ts_data['sample_rate']
    x_values, x_label = _build_x_axis(ts_data, x_axis)
    info = ts_data.get('info')
    units = _canonical_units(units)
    _validate_reference_plane(reference_plane)
    if units == 'raw' and reference_plane != 'adc_input':
        raise ValueError(
            "units='raw' always means accumulator units and does not have a "
            "detector/cryostat reference plane. Use units='adc_units' to "
            "undo the firmware accumulator path and refer linear ADC units "
            f"to reference_plane={reference_plane!r}.")
    calibration_cache = {}

    # Look up each selected tone's RF frequency for cal resolution.
    tone_freqs = None
    if isinstance(info, dict):
        tf = info.get('tones', {}).get('frequencies_hz')
        if tf is not None:
            tone_freqs = np.asarray(tf, dtype=float)

    def _tone_frequency(key):
        if tone_freqs is None:
            return None
        try:
            return float(tone_freqs[int(key)])
        except (IndexError, ValueError, TypeError):
            return None

    # Normalise
    if units != 'raw':
        if units != 'peak' and (info is None or not isinstance(info, dict)):
            raise ValueError("ts_data must contain 'info' for non-raw units.")
        normalised = []
        for key, i_arr, q_arr in selected:
            tone_f = _tone_frequency(key)
            ni, nq, _, _, iq_label, mag_label = _normalise_iq(
                i_arr, q_arr, units, info, config=config,
                reference_plane=reference_plane,
                frequencies=tone_f,
                calibration_cache=calibration_cache)
            normalised.append((key, ni, nq))
        selected = normalised
    else:
        iq_label = ''
        mag_label = '|RX| (dB)'

    precomputed_freq_diss = _has_precomputed_freq_diss(ts_data)
    if format == 'freq_diss' and sweep_data is None and not precomputed_freq_diss:
        raise ValueError("sweep_data is required for format='freq_diss'")

    suffix = _transform_title_suffix(deembed, phase_center)

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for key, i_arr, q_arr in selected:
            z = i_arr + 1j * q_arr
            z, _, _ = _apply_tone_transforms(
                z, int(key), deembed, phase_center,
                sweep_data=sweep_data, tone_frequency=_tone_frequency(key),
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                calibration_cache=calibration_cache)
            trace_label = _resolve_label(ax, label,
                                         suffix=f'Tone {key}' if len(selected) > 1 else None)
            ax.plot(z.real, z.imag, '.', markersize=1,
                    label=trace_label, **kwargs)
        ax.set_xlabel(f'I {iq_label}'.strip())
        ax.set_ylabel(f'Q {iq_label}'.strip())
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend(fontsize='small')
        ax.set_title(custom_title if custom_title is not None
                     else 'Timestream I vs Q' + suffix)
        plt.tight_layout()
        return fig

    # Dual-axis formats
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 5))
    else:
        ax1, ax2 = fig.axes[:2]

    for key, i_arr, q_arr in selected:
        trace_label = _resolve_label(ax1, label,
                                     suffix=f'Tone {key}' if len(selected) > 1 else None)
        z = i_arr + 1j * q_arr

        if format == 'iq_vs_t':
            z, _, _ = _apply_tone_transforms(
                z, int(key), deembed, phase_center,
                sweep_data=sweep_data, tone_frequency=_tone_frequency(key),
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                calibration_cache=calibration_cache)
            ax1.plot(x_values, z.real, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, z.imag, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(f'I {iq_label}'.strip())
            ax2.set_ylabel(f'Q {iq_label}'.strip())

        elif format == 'magphase':
            tone_f = _tone_frequency(key)
            z, _, _ = _apply_tone_transforms(
                z, int(key), deembed, phase_center,
                sweep_data=sweep_data, tone_frequency=tone_f,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                calibration_cache=calibration_cache)
            mag_db, phase = _compute_mag_phase_units(
                z, units, info=info, config=config,
                reference_plane=reference_plane, frequencies=tone_f,
                unwrap=unwrap_phase, calibration_cache=calibration_cache)
            ax1.plot(x_values, mag_db, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, phase, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(mag_label)
            ax2.set_ylabel('Phase (rad)')

        elif format == 'freq_diss':
            if precomputed_freq_diss:
                frac_f, frac_d = _precomputed_freq_diss(
                    ts_data, key,
                    reference_tone_frequency=reference_tone_frequency)
            else:
                frac_f, frac_d = _compute_freq_diss(
                    ts_data, key, i_arr, q_arr, sweep_data,
                    reference_tone_frequency=reference_tone_frequency,
                    smooth_window_hz=smooth_window_hz,
                    conversion_method=conversion_method,
                    calibrations=calibrations)
            ax1.plot(x_values, frac_f, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, frac_d, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel('Fractional resonator-frequency motion')
            ax2.set_ylabel(_dissipation_axis_label(conversion_method))

        else:
            raise ValueError(
                f"Unknown format '{format}'. "
                "Use 'iq', 'iq_vs_t', 'magphase', or 'freq_diss'.")

    ax2.set_xlabel(x_label)
    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')
    if custom_title is not None:
        title = custom_title
    else:
        title_map = {
            'iq_vs_t': 'Timestream',
            'magphase': 'Timestream',
            'freq_diss': 'Frequency & Dissipation',
        }
        title = title_map.get(format, 'Timestream')
        if format != 'freq_diss':
            title += suffix
    fig.suptitle(title)
    plt.tight_layout()
    return fig


def plot_timestream_psd(ts_data, format='iq', tones=None,
                        sweep_data=None, reference_tone_frequency=None,
                        psd_kwargs=None,
                        precomputed_psd=None, fig=None, label=None,
                        smooth_window_hz=1000, *,
                        conversion_method='linearized',
                        calibrations=None,
                        include_dc_point=False,
                        log_bin=False,
                        bins_per_decade=20,
                        decorrelate_modes=0,
                        decorrelate_plot='overlay',
                        decorrelate_tones=None,
                        blind_tone_modes=0,
                        blind_tones=None,
                        **kwargs):
    """
    Plot power spectral density of timestream data.

    Args:
        ts_data: dict from parse_samples().
        format: 'iq' | 'magphase' | 'freq_diss'
            Selects which quantity to compute PSD of.
        tones: List of tone indices. Default: [0].
        sweep_data: Required for 'freq_diss' format.
        reference_tone_frequency: Tone frequency to use for the freq/diss
            calculation (``format='freq_diss'``).  ``None`` (default) looks
            up the per-tone frequency from
            ``ts_data['info']['tones']['frequencies_hz']``.  Pass a scalar
            for a single override, or a ``{tone_idx: hz}`` dict for per-tone
            overrides.
        smooth_window_hz: Smoothing window passed to
            the linearized resonator conversion for ``format='freq_diss'``.
            Pass ``None`` or ``0`` to disable sweep smoothing.
        conversion_method: ``'linearized'`` (default), ``'mobius'``, or
            ``'circle'`` for ``format='freq_diss'``.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``.
        psd_kwargs: dict of kwargs passed to compute_psd().
        precomputed_psd: dict mapping tone_key -> (f_psd, psd_values).
            If provided, skip computation.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        include_dc_point: bool, optional. Include the zero-frequency PSD
            point. Defaults to ``False`` because the plot uses a log x-axis.
        log_bin: bool, optional. If ``True``, average PSD samples into
            logarithmically spaced positive-frequency bins before plotting.
            Default ``False`` preserves the raw Welch/periodogram frequencies.
        bins_per_decade: float, optional. Number of log bins per frequency
            decade when ``log_bin=True``. Default 10.
        decorrelate_modes: int, optional. For ``format='freq_diss'``, remove
            this many leading SVD modes across the calibrated slow-timestream
            tone rows. ``0`` (default) disables decorrelation.
        decorrelate_plot: {'overlay', 'replace'}, optional. Plot decorrelated
            spectra alongside the raw spectra or instead of them. The
            cleaned legend records each active cleaning step.
        decorrelate_tones: iterable of int or None, optional. Tones used to
            estimate the SVD common modes. ``None`` (default) uses regular
            tones from the saved metadata when available, otherwise every
            active tone. Every tone selected for plotting must be included.
        blind_tone_modes: int, optional. For ``format='freq_diss'``, fit this
            many temporal modes from simultaneous blind-tone amplitude/phase
            variations and regress them out of regular-tone I/Q before
            frequency/dissipation conversion. ``0`` (default) disables
            blind-tone cleaning. Blind sweep traces are not used. When both
            cleaning options are enabled, regular-tone SVD cleaning follows
            blind-tone subtraction and calibration.
        blind_tones: iterable of int or None, optional. Explicit blind-tone
            indices. ``None`` (default) infers them from saved timestream
            metadata. Use this for older captures without role metadata.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones)
    sample_rate = ts_data['sample_rate']
    psd_kw = psd_kwargs or {}

    precomputed_freq_diss = _has_precomputed_freq_diss(ts_data)
    if format == 'freq_diss' and sweep_data is None and not precomputed_freq_diss:
        raise ValueError("sweep_data is required for format='freq_diss'")
    if not isinstance(decorrelate_modes, (int, np.integer)):
        raise TypeError("decorrelate_modes must be an integer")
    if decorrelate_modes < 0:
        raise ValueError("decorrelate_modes must be non-negative")
    if not isinstance(blind_tone_modes, (int, np.integer)):
        raise TypeError("blind_tone_modes must be an integer")
    if blind_tone_modes < 0:
        raise ValueError("blind_tone_modes must be non-negative")
    if decorrelate_plot not in ('overlay', 'replace'):
        raise ValueError(
            "decorrelate_plot must be either 'overlay' or 'replace'")
    if (decorrelate_modes or blind_tone_modes) and format != 'freq_diss':
        raise ValueError(
            "SVD and blind-tone cleaning are supported only for "
            "format='freq_diss'")
    if ((decorrelate_modes or blind_tone_modes)
            and format == 'freq_diss' and sweep_data is None):
        raise ValueError(
            "SVD and blind-tone cleaning require sweep_data; precomputed "
            "modulation demodulation can be plotted only without these "
            "cleaning modes.")

    if format in ('iq', 'magphase'):
        if fig is None:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6),sharex=True)
        else:
            ax1, ax2 = fig.axes[:2]

        for key, i_arr, q_arr in selected:
            trace_label = _resolve_label(ax1, label,
                                         suffix=f'Tone {key}' if len(selected) > 1 else None)
            z = i_arr + 1j * q_arr

            if precomputed_psd and key in precomputed_psd:
                f1, p1 = precomputed_psd[key][:2]
                f2, p2 = precomputed_psd[key][2:4] if len(precomputed_psd[key]) > 2 else (f1, p1)
            elif format == 'iq':
                f1, p1 = compute_psd(i_arr, sample_rate, **psd_kw)
                f2, p2 = compute_psd(q_arr, sample_rate, **psd_kw)
            else:  # magphase
                mag = np.abs(z)
                phase = np.unwrap(np.angle(z))
                f1, p1 = compute_psd(mag, sample_rate, **psd_kw)
                f2, p2 = compute_psd(phase, sample_rate, **psd_kw)

            f1, p1 = _prepare_psd_for_log_plot(
                f1, p1, include_dc_point, log_bin, bins_per_decade)
            f2, p2 = _prepare_psd_for_log_plot(
                f2, p2, include_dc_point, log_bin, bins_per_decade)
            ax1.loglog(f1, p1, linewidth=0.5, label=trace_label, **kwargs)
            ax2.loglog(f2, p2, linewidth=0.5, label=trace_label, **kwargs)

        if format == 'iq':
            ax1.set_ylabel('I PSD')
            ax2.set_ylabel('Q PSD')
        else:
            ax1.set_ylabel('Magnitude PSD')
            ax2.set_ylabel('Phase PSD')

    elif format == 'freq_diss':
        if fig is None:
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6),
                                           sharex=True, sharey=True)
        else:
            ax1, ax2 = fig.axes[:2]

        raw_converted = {}
        cleaned = {}
        if decorrelate_modes or blind_tone_modes:
            raw_converted, cleaned = _compute_cleaned_freq_diss(
                ts_data,
                sweep_data,
                [int(key) for key, _, _ in selected],
                decorrelate_modes,
                decorrelate_tones,
                blind_tone_modes,
                blind_tones,
                reference_tone_frequency,
                smooth_window_hz,
                conversion_method,
                calibrations,
            )

        for key, i_arr, q_arr in selected:
            trace_label = _resolve_label(ax1, label,
                                         suffix=f'Tone {key}' if len(selected) > 1 else None)
            tone_index = int(key)
            raw_color = None
            if (not (decorrelate_modes or blind_tone_modes)
                    or decorrelate_plot == 'overlay'):
                if precomputed_freq_diss and not (decorrelate_modes or blind_tone_modes):
                    frac_f, frac_d = _precomputed_freq_diss(
                        ts_data, key,
                        reference_tone_frequency=reference_tone_frequency)
                elif ((decorrelate_modes or blind_tone_modes)
                        and tone_index in raw_converted):
                    frac_f, frac_d = raw_converted[tone_index]
                else:
                    frac_f, frac_d = _compute_freq_diss(
                        ts_data, key, i_arr, q_arr, sweep_data,
                        reference_tone_frequency=reference_tone_frequency,
                        smooth_window_hz=smooth_window_hz,
                        conversion_method=conversion_method,
                        calibrations=calibrations)
                f1, p1 = compute_psd(frac_f, sample_rate, **psd_kw)
                f2, p2 = compute_psd(frac_d, sample_rate, **psd_kw)

                f1, p1 = _prepare_psd_for_log_plot(
                    f1, p1, include_dc_point, log_bin, bins_per_decade)
                f2, p2 = _prepare_psd_for_log_plot(
                    f2, p2, include_dc_point, log_bin, bins_per_decade)
                lines = ax1.loglog(
                    f1, p1, linewidth=0.5, label=trace_label, **kwargs)
                raw_color = lines[0].get_color()
                ax2.loglog(
                    f2, p2, linewidth=0.5, label=trace_label, **kwargs)

            if decorrelate_modes or blind_tone_modes:
                if tone_index not in cleaned:
                    raise ValueError(
                        f"Tone {tone_index} selected for plotting is not in "
                        "the cleaned tone set")
                frac_f, frac_d = cleaned[tone_index]
                f1, p1 = compute_psd(frac_f, sample_rate, **psd_kw)
                f2, p2 = compute_psd(frac_d, sample_rate, **psd_kw)
                f1, p1 = _prepare_psd_for_log_plot(
                    f1, p1, include_dc_point, log_bin, bins_per_decade)
                f2, p2 = _prepare_psd_for_log_plot(
                    f2, p2, include_dc_point, log_bin, bins_per_decade)
                clean_kwargs = dict(kwargs)
                if decorrelate_plot == 'overlay':
                    clean_kwargs.setdefault('linestyle', '--')
                    clean_kwargs.setdefault('color', raw_color)
                steps = []
                if blind_tone_modes:
                    steps.append(f'Blind N={blind_tone_modes}')
                if decorrelate_modes:
                    steps.append(f'SVD N={decorrelate_modes}')
                clean_label = f'{trace_label} ({", ".join(steps)})'
                ax1.loglog(
                    f1, p1, linewidth=0.5, label=clean_label, **clean_kwargs)
                ax2.loglog(
                    f2, p2, linewidth=0.5, label=clean_label, **clean_kwargs)

        ax1.set_ylabel('Resonator-frequency noise PSD')
        ax2.set_ylabel(_dissipation_axis_label(
            conversion_method, psd=True))
        _match_y_limits(ax1, ax2)

    else:
        raise ValueError(
            f"Unknown format '{format}'. Use 'iq', 'magphase', or 'freq_diss'.")

    ax2.set_xlabel('Frequency (Hz)')
    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')
    fig.suptitle('Power Spectral Density')
    plt.tight_layout()
    return fig


def plot_timestream_on_resonance(ts_data, sweep_data, tone_index,
                                  deembed=False, phase_center=False,
                                  unwrap=False,
                                  hide_modulation_settling_points=False,
                                  fig=None, label=None,
                                  units='raw', config=None, **kwargs):
    """
    Overplot timestream I/Q points on the resonance circle from sweep data.

    Args:
        ts_data: dict from parse_samples().
        sweep_data: dict from parse_sweep_data() (per-tone).
        tone_index: int, which tone to plot.
        deembed: bool, apply true RF deembedding (cable delay removal +
            baseline normalisation) to both sweep and timestream.
        phase_center: bool, apply phase centering (circle centering +
            rotation) to both sweep and timestream.  Applied after
            deembedding when both are True.
        unwrap: bool, unwrap the phase.
        hide_modulation_settling_points: bool, optional. If True, omit
            timestream samples where ``modulation_settling`` is set. Default
            False preserves the full capture.
        fig: Existing figure.
        label: Legend label for the timestream points. If None, uses
            'Timestream'.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator codes, no normalisation.
            'peak' - normalise to the peak magnitude of the data.
            'adc_fs' - fraction of ADC full-scale (linear voltage).
        config: Config dict (needed for non-default rx_mix_scale).
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones=[tone_index])
    key, i_arr, q_arr = selected[0]

    # Extract sweep trace for this tone
    sweep_f, sw_i, sw_q = _get_sweep_trace(sweep_data, tone_index)

    # Normalise both sweep and timestream with the same units
    if _is_calibrated_magnitude_unit(units):
        raise ValueError(
            f"units='{units}' is not supported for I vs Q plots — "
            "use 'raw', 'peak', or 'adc_fs'.")
    info = ts_data.get('info') or sweep_data.get('info')
    calibration_cache = {}
    iq_label = ''
    if units != 'raw':
        tone_f = np.mean(sweep_f) if sweep_f is not None else None
        if units != 'peak' and (info is None or not isinstance(info, dict)):
            raise ValueError("data must contain 'info' for non-raw units.")
        i_arr, q_arr, _, _, iq_label, _ = _normalise_iq(
            i_arr, q_arr, units, info, config=config,
            frequencies=tone_f, calibration_cache=calibration_cache)
        sw_i, sw_q, _, _, _, _ = _normalise_iq(
            sw_i, sw_q, units, info, config=config,
            frequencies=sweep_f, calibration_cache=calibration_cache)

    z_ts = i_arr + 1j * q_arr
    z_sweep = sw_i + 1j * sw_q

    ts_info = ts_data.get('info') or {}
    ts_tone_freqs = ts_info.get('tones', {}).get('frequencies_hz')
    if ts_tone_freqs is not None:
        tone_freq = float(np.asarray(ts_tone_freqs)[tone_index])
    else:
        tone_freq = float(np.mean(sweep_f))
    ts_freq = _get_modulated_probe_frequencies(
        ts_data, tone_index, tone_freq, len(z_ts))

    # Apply transforms to sweep and timestream at their probe frequencies.
    d_params = None
    pc_params = None
    if deembed:
        z_sweep, d_params = _apply_deembed(sweep_f, z_sweep, True)
        z_ts, _ = _apply_deembed(None, z_ts, d_params, frequency=ts_freq)
    if phase_center:
        z_sweep, pc_params = _apply_phase_center(z_sweep, True)
        z_ts, _ = _apply_phase_center(z_ts, pc_params)

    visible = _visible_modulation_sample_mask(
        ts_data, len(z_ts), hide_modulation_settling_points)
    z_ts_plot = z_ts[visible]
    ts_freq_plot = ts_freq[visible]

    # Compute phase for the frequency-domain panel
    _, phase_sweep = _compute_mag_phase(z_sweep, unwrap=unwrap)
    _, phase_ts = _compute_mag_phase(z_ts_plot, unwrap=unwrap)

    if fig is None:
        fig, (ax_iq, ax_pf) = plt.subplots(1, 2, figsize=(14, 6))
    else:
        ax_iq, ax_pf = fig.axes[:2]

    ts_label = label if label is not None else 'Timestream'

    # Left panel: I vs Q resonance circle
    ax_iq.plot(z_sweep.real, z_sweep.imag, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_iq.plot(z_ts_plot.real, z_ts_plot.imag, '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_iq.set_xlabel(f'I {iq_label}'.strip())
    ax_iq.set_ylabel(f'Q {iq_label}'.strip())
    ax_iq.set_aspect('equal', adjustable='datalim')
    ax_iq.legend(fontsize='small')

    # Right panel: phase vs frequency
    ax_pf.plot(sweep_f, phase_sweep, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_pf.plot(ts_freq_plot, phase_ts,
               '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_pf.set_xlabel('Frequency (Hz)')
    ax_pf.set_ylabel('Phase (rad)')
    ax_pf.legend(fontsize='small')

    suffix = _transform_title_suffix(deembed, phase_center)
    fig.suptitle(f'Tone {tone_index} — Resonance Circle' + suffix)
    plt.tight_layout()
    return fig
