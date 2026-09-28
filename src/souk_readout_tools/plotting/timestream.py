"""
Timestream data plotting functions.

Supports parsed timestream data (from ReadoutClient.parse_samples()) in
multiple formats: I/Q, magnitude/phase, frequency/dissipation, and their
PSDs. Includes a debugging overlay of timestream points on the resonance
circle from sweep data, with optional deembedding or phase centering.
"""

import warnings

import numpy as np
from collections.abc import Mapping

from ._common import (_get_pyplot, _compute_mag_phase, _dual_panels,
                       ERRORBAR_STYLE,
                       _apply_deembed, _apply_phase_center,
                       _resolve_label, _legend,
                       _normalise_iq, _compute_mag_phase_units, UNITS,
                       _is_calibrated_magnitude_unit,
                       _canonical_units, _validate_reference_plane)
from ._psd import check_psd_units, psd_display_values
from ..resonator import sweep_trace, with_readout_correction
from ..timeseries import (FREQ_DISS_FORMATS, _compute_freq_diss, _get_tone_data,
                          _has_precomputed_freq_diss, _linewidth_scale,
                          _precomputed_freq_diss,
                          _prepare_psd_for_log_plot,
                          _warn_ignored_conversion_args, timestream_psds)

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
        # A producer (e.g. modulation.point_timestreams) may supply a real time
        # axis carrying each sample's true offset; prefer it over the uniform
        # arange so per-point streams keep their sub-cycle shifts.
        t = ts_data.get('time_s')
        if t is not None:
            t = np.asarray(t, dtype=float)
            if t.shape == (n,):
                return t, 'Time (s)'
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
        sweep_f, sw_i, sw_q = sweep_trace(sweep_data, tone_idx)
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
        elif phase_center is True:
            # A timestream covers a tiny arc of the resonance circle: a circle
            # fitted to it is noise, not the resonance's centre.
            raise ValueError(
                "phase_center=True for timestream data needs sweep_data (or a "
                "params dict from resonator.phase_center): the timestream's own "
                "short arc cannot locate the resonance circle.")
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


def _plot_psd(ax, f, p, psd_units, carrier=1.0, **kwargs):
    """Plot one PSD panel in the requested display units."""
    values = psd_display_values(p, psd_units, carrier)
    plot = ax.loglog if psd_units == 'linear' else ax.semilogx
    return plot(f, values, **kwargs)


def _psd_axis_labels(format, psd_units, conversion_method):
    """Return the two panel y-axis labels for a PSD plot."""
    if psd_units == 'dbc':
        if format == 'iq':
            return 'I noise (dBc/Hz, SSB)', 'Q noise (dBc/Hz, SSB)'
        return 'AM noise (dBc/Hz, SSB)', 'PM noise (dBc/Hz, SSB)'
    suffix = ' (dB)' if psd_units == 'db' else ''
    if format == 'linewidths':
        return ('Frequency noise PSD (linewidths$^2$/Hz)' + suffix,
                r'Dissipation PSD, $Q_r\,\Delta(1/2Q_i)$' + suffix)
    if format == 'iq':
        return 'I PSD' + suffix, 'Q PSD' + suffix
    if format == 'magphase':
        return 'Magnitude PSD' + suffix, 'Phase PSD' + suffix
    return ('Resonator-frequency noise PSD' + suffix,
            _dissipation_axis_label(conversion_method, psd=True) + suffix)


def _cleaning_label(trace_label, blind_tone_modes, decorrelate_modes):
    """Label a cleaned trace with the cleaning steps that produced it."""
    steps = []
    if blind_tone_modes:
        steps.append(f'Blind N={blind_tone_modes}')
    if decorrelate_modes:
        steps.append(f'SVD N={decorrelate_modes}')
    return f'{trace_label} ({", ".join(steps)})'


def plot_timestream(ts_data, format='iq_vs_t', tones=None,
                    deembed=False, phase_center=False,
                    sweep_data=None, reference_tone_frequency=None,
                    fig=None, label=None,
                    units='raw', config=None, reference_plane='adc_input',
                    x_axis='time', unwrap_phase=True,
                    smooth_window_hz=1000, conversion_method=None,
                    calibrations=None, apply_readout_correction=True,
                    sharex=True, sharey=False,
                    **kwargs):
    """
    Plot timestream data in various formats.

    Args:
        ts_data: dict from parse_samples() with 'i_data', 'q_data',
            'sample_rate', 'num_samples'.
        format: 'iq' | 'iq_vs_t' | 'magphase' | 'freq_diss' | 'linewidths'.
            ``'linewidths'`` is ``'freq_diss'`` times each tone's loaded Q
            (from ``calibrations``): frequency motion in linewidths, and
            dissipation ``Qr * Delta(1/2Qi)`` on the same scale.
        tones: List of tone indices (int) to plot, or None for [0].
        deembed: bool or deembed params dict.  Applies true RF
            deembedding (baseline normalisation).  For timestream data,
            ``True`` derives params from the matching ``sweep_data`` trace;
            otherwise pass a pre-computed params dict from
            ``resonator.deembed()``.
        phase_center: bool or phase-centering params dict.  Applies
            circle centering and rotation.  For timestream data, ``True``
            derives params from the matching ``sweep_data`` trace, and
            raises without one (a timestream's short arc cannot locate the
            circle).  Pass a pre-computed params dict from
            ``resonator.phase_center()`` to apply specific params.  Applied
            after deembedding when both are set.  For the fitted frame the
            tracker uses, convert with ``resonator.to_basis`` instead.
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
        conversion_method: ``'linearized'``, ``'mobius'``, or ``'circle'``
            for ``format='freq_diss'``; ``None`` (default) means
            ``'linearized'``. Ignored -- with a warning -- when ``ts_data``
            already carries demodulated freq/diss (``demodulate_timestream``
            output): there the conversion was fixed at demodulation time via
            its ``calibration`` argument.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``, and by
            ``format='linewidths'`` for each tone's Qr. Otherwise ignored for
            already-demodulated timestreams (see ``conversion_method``).
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
        apply_readout_correction: bool, plot with (True, default) or without
            (False) the filterbank compensation's software readout-flattening
            factors, for modulated captures and any sweep trace used. Toggles
            already-parsed data either way when the factors are available.
        sharex: bool, optional. Tie the two panels' x limits together in the
            dual-panel formats. Default ``True``.
        sharey: bool, optional. Tie the two panels' y limits together, so
            they can be compared by eye. Default ``False``; most useful for
            ``'iq_vs_t'`` and ``'freq_diss'``, whose panels share units.
            Ignored by ``format='iq'`` (a single panel). Plotting onto an
            existing ``fig`` can add sharing but cannot remove sharing that
            figure was created with.
        **kwargs: Passed to matplotlib plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    ts_data = with_readout_correction(ts_data, apply_readout_correction)
    sweep_data = with_readout_correction(sweep_data, apply_readout_correction)
    custom_title = kwargs.pop('title', None)
    selected = _get_tone_data(ts_data, tones)
    # Frequency/dissipation always converts the recorded I/Q, which is what the
    # sweep and calibrations describe; units only rescale the I/Q panels.
    recorded_iq = {key: (i_arr, q_arr) for key, i_arr, q_arr in selected}
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
    if (format in FREQ_DISS_FORMATS and sweep_data is None
            and not precomputed_freq_diss):
        raise ValueError(f"sweep_data is required for format={format!r}")
    if format in FREQ_DISS_FORMATS and precomputed_freq_diss:
        # 'linewidths' still reads the loaded Q from calibrations.
        _warn_ignored_conversion_args(
            conversion_method,
            calibrations if format == 'freq_diss' else None)
    if conversion_method is None:
        conversion_method = 'linearized'

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
        _legend(ax)
        ax.set_title(custom_title if custom_title is not None
                     else 'Timestream I vs Q' + suffix)
        fig.tight_layout()
        return fig

    # Dual-axis formats
    fig, ax1, ax2 = _dual_panels(fig, sharex=sharex, sharey=sharey,
                                 figsize=(10, 5))

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
            ax1.plot(x_values, z.real, label=trace_label, **kwargs)
            ax2.plot(x_values, z.imag, label=trace_label, **kwargs)
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
            ax1.plot(x_values, mag_db, label=trace_label, **kwargs)
            ax2.plot(x_values, phase,  label=trace_label, **kwargs)
            ax1.set_ylabel(mag_label)
            ax2.set_ylabel('Phase (rad)')

        elif format in FREQ_DISS_FORMATS:
            if precomputed_freq_diss:
                frac_f, frac_d = _precomputed_freq_diss(
                    ts_data, key,
                    reference_tone_frequency=reference_tone_frequency)
            else:
                frac_f, frac_d = _compute_freq_diss(
                    ts_data, key, *recorded_iq[key], sweep_data,
                    reference_tone_frequency=reference_tone_frequency,
                    smooth_window_hz=smooth_window_hz,
                    conversion_method=conversion_method,
                    calibrations=calibrations)
            if format == 'linewidths':
                scale = _linewidth_scale(calibrations, key)
                ax1.plot(x_values, scale * frac_f, label=trace_label, **kwargs)
                ax2.plot(x_values, scale * frac_d, label=trace_label, **kwargs)
                ax1.set_ylabel('Resonator-frequency motion (linewidths)')
                ax2.set_ylabel(r'Dissipation $Q_r\,\Delta(1/2Q_i)$')
            else:
                ax1.plot(x_values, frac_f, label=trace_label, **kwargs)
                ax2.plot(x_values, frac_d, label=trace_label, **kwargs)
                ax1.set_ylabel('Fractional resonator-frequency motion')
                ax2.set_ylabel(_dissipation_axis_label(conversion_method))

        else:
            raise ValueError(
                f"Unknown format '{format}'. "
                "Use 'iq', 'iq_vs_t', 'magphase', 'freq_diss' or "
                "'linewidths'.")

    ax2.set_xlabel(x_label)
    _legend(ax1)
    _legend(ax2)
    if custom_title is not None:
        title = custom_title
    else:
        title_map = {
            'iq_vs_t': 'Timestream',
            'magphase': 'Timestream',
            'freq_diss': 'Frequency & Dissipation',
            'linewidths': 'Frequency & Dissipation (linewidths)',
        }
        title = title_map.get(format, 'Timestream')
        if format not in FREQ_DISS_FORMATS:
            title += suffix
    fig.suptitle(title)
    fig.tight_layout()
    return fig


def plot_timestream_psd(ts_data, format='iq', tones=None,
                        sweep_data=None, reference_tone_frequency=None,
                        psd_kwargs=None,
                        precomputed_psd=None, fig=None, label=None,
                        smooth_window_hz=1000, *,
                        conversion_method=None,
                        calibrations=None,
                        include_dc_point=False,
                        log_bin=False,
                        bins_per_decade=20,
                        psd_units='linear',
                        decorrelate_modes=0,
                        decorrelate_plot='overlay',
                        decorrelate_tones=None,
                        blind_tone_modes=0,
                        blind_tones=None,
                        exclude_settling=False,
                        apply_readout_correction=True,
                        sharex=True,
                        sharey=None,
                        return_data=False,
                        **kwargs):
    """
    Plot power spectral density of timestream data.

    The compute half of this lives in ``souk_readout_tools.timeseries`` as
    ``timestream_psds()``: same arguments, same numbers, returned as arrays
    rather than drawn. Pass ``return_data=True`` here to get both, or see
    ``doc/timeseries_analysis.md`` for band levels, fits, on-minus-off and the
    rest of the quantitative tools.

    Args:
        ts_data: dict from parse_samples().
        format: 'iq' | 'magphase' | 'freq_diss' | 'linewidths'
            Selects which quantity to compute PSD of (``'linewidths'``: see
            ``plot_timestream``; needs ``calibrations``).
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
        conversion_method: ``'linearized'``, ``'mobius'``, or ``'circle'``
            for ``format='freq_diss'``; ``None`` (default) means
            ``'linearized'``. Ignored -- with a warning -- when ``ts_data``
            already carries demodulated freq/diss (``demodulate_timestream``
            output) and no cleaning modes are active: there the conversion
            was fixed at demodulation time via its ``calibration`` argument.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``, and by
            ``format='linewidths'`` for each tone's Qr. Otherwise ignored for
            already-demodulated timestreams (see ``conversion_method``).
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
        psd_units: {'linear', 'db', 'dbc'}, optional. Display units.
            ``'linear'`` (default) plots the PSD in its native units on a
            log-log axis. ``'db'`` plots ``10*log10(PSD)`` of the same
            quantity. ``'dbc'`` divides the raw-unit panels by the carrier
            power ``|mean(z)|**2`` and halves the one-sided density, giving
            the IEEE single-sideband ``L(f) = S(f) / 2`` in dBc/Hz: AM and PM
            noise for ``format='magphase'``, per-quadrature noise for
            ``'iq'``. Phase is already carrier-relative and is only halved.
            One carrier per tone is taken from the raw I/Q, so cleaned traces
            share the raw trace's normalisation. Not available for
            ``format='freq_diss'``, whose rows are already fractional -- use
            ``'db'`` there. Note that ``'iq'`` splits AM and PM between the
            panels by the tone's own phase, because the quadratures are not
            rotated onto the carrier; ``'magphase'`` is that rotation.
        decorrelate_modes: int, optional. Remove this many leading SVD modes
            across the slow-timestream tone rows. ``0`` (default) disables
            decorrelation. For ``format='freq_diss'`` the modes are removed
            from the calibrated frequency and dissipation rows; for
            ``'iq'``/``'magphase'`` they are fitted from the tones' own
            amplitude and unwrapped-phase variations and removed from those,
            which needs no sweep and so works on loopback data.
        decorrelate_plot: {'overlay', 'replace'}, optional. Plot decorrelated
            spectra alongside the raw spectra or instead of them. The
            cleaned legend records each active cleaning step.
        decorrelate_tones: iterable of int or None, optional. Tones used to
            estimate the SVD common modes. ``None`` (default) uses regular
            tones from the saved metadata when available, otherwise every
            active tone (every non-blind tone for ``'iq'``/``'magphase'``).
            Every tone selected for plotting must be included.
        blind_tone_modes: int, optional. Fit this many temporal modes from
            simultaneous blind-tone amplitude/phase variations and regress
            them out of the regular-tone I/Q -- before frequency/dissipation
            conversion for ``format='freq_diss'``. ``0`` (default) disables
            blind-tone cleaning. Blind sweep traces are not used. When both
            cleaning options are enabled, SVD cleaning follows blind-tone
            subtraction (and calibration, for ``'freq_diss'``).
        blind_tones: iterable of int or None, optional. Explicit blind-tone
            indices. ``None`` (default) infers them from saved timestream
            metadata. Use this for older captures without role metadata.
        exclude_settling: bool, optional. Compute the PSD from only the
            samples not flagged ``modulation_settling`` (rather than filling
            them). The kept samples must be uniformly spaced -- true when the
            dwell keeps one run of good samples per slot (e.g.
            ``dwell - n_settle == 1``) -- and the PSD is computed at the
            decimated sample rate. Raises if the flag is missing or the kept
            samples are not evenly spaced. Incompatible with SVD / blind-tone
            cleaning. Default ``False``.
        sharex: bool, optional. Tie the two panels' x limits together.
            Default ``True``.
        sharey: bool or None, optional. Tie the two panels' y limits
            together, so the top and bottom spectra can be compared by eye.
            ``None`` (default) picks per format: on for ``'freq_diss'``
            (both panels are fractional and directly comparable), off for
            ``'iq'`` and ``'magphase'`` (whose panels carry different units).
            Pass ``True`` to tie a magnitude/phase or I/Q pair anyway.
            Plotting onto an existing ``fig`` can add sharing but cannot
            remove sharing that figure was created with.
        return_data: bool, optional. Also return the computed arrays, so the
            plotted numbers can be used quantitatively. Default ``False``.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure, or ``(fig, result)`` when
        ``return_data=True``. ``result`` is the dict returned by
        :func:`souk_readout_tools.timeseries.timestream_psds`, which computes
        everything drawn here.
    """
    check_psd_units(psd_units, format)
    if decorrelate_plot not in ('overlay', 'replace'):
        raise ValueError(
            "decorrelate_plot must be either 'overlay' or 'replace'")

    result = timestream_psds(
        ts_data, format=format, tones=tones, sweep_data=sweep_data,
        reference_tone_frequency=reference_tone_frequency,
        psd_kwargs=psd_kwargs, smooth_window_hz=smooth_window_hz,
        conversion_method=conversion_method, calibrations=calibrations,
        include_dc_point=include_dc_point, log_bin=log_bin,
        bins_per_decade=bins_per_decade,
        decorrelate_modes=decorrelate_modes,
        decorrelate_tones=decorrelate_tones,
        blind_tone_modes=blind_tone_modes, blind_tones=blind_tones,
        exclude_settling=exclude_settling,
        apply_readout_correction=apply_readout_correction)

    f = result['f']
    panel1, panel2 = result['panels']
    cleaned = result['psd_cleaned']
    cleaning_on = cleaned is not None

    if sharey is None:
        # The freq/diss panels are both fractional, so one scale compares them
        # directly; I/Q and magnitude/phase panels are not in the same units.
        sharey = (format in FREQ_DISS_FORMATS)
    fig, ax1, ax2 = _dual_panels(fig, sharex=sharex, sharey=sharey)

    # The legend names tones by the capture's own dict keys ('0000', ...);
    # result['tones'] carries the same selection as integers.
    tone_keys = sorted(ts_data['i_data'])
    n_tones = len(result['tones'])

    for row, tone_index in enumerate(result['tones']):
        key = tone_keys[int(tone_index)]
        trace_label = _resolve_label(
            ax1, label, suffix=f'Tone {key}' if n_tones > 1 else None)
        raw_color = None
        # One carrier per tone, from the raw rows: cleaning preserves the
        # amplitude/phase baselines, so both traces share a normalisation.
        carrier = 1.0
        if psd_units == 'dbc':
            carrier = float(result['carrier'][row])
            if not carrier > 0:
                raise ValueError(
                    "psd_units='dbc' needs a carrier, but the mean I/Q is zero")
        # The phase panel is already carrier-relative (rad^2/Hz).
        carrier2 = carrier if format == 'iq' else 1.0

        if not cleaning_on or decorrelate_plot == 'overlay':
            f1, p1 = f, result['psd'][panel1][row]
            f2, p2 = f, result['psd'][panel2][row]
            if (precomputed_psd and format in ('iq', 'magphase')
                    and key in precomputed_psd):
                f1, p1 = precomputed_psd[key][:2]
                f2, p2 = (precomputed_psd[key][2:4]
                          if len(precomputed_psd[key]) > 2 else (f1, p1))
                f1, p1 = _prepare_psd_for_log_plot(
                    f1, p1, include_dc_point, log_bin, bins_per_decade)
                f2, p2 = _prepare_psd_for_log_plot(
                    f2, p2, include_dc_point, log_bin, bins_per_decade)
            lines = _plot_psd(ax1, f1, p1, psd_units, carrier,
                              label=trace_label, **kwargs)
            raw_color = lines[0].get_color()
            _plot_psd(ax2, f2, p2, psd_units, carrier2,
                      label=trace_label, **kwargs)

        if cleaning_on:
            clean_kwargs = dict(kwargs)
            if decorrelate_plot == 'overlay':
                clean_kwargs.setdefault('linestyle', '--')
                clean_kwargs.setdefault('color', raw_color)
            clean_label = _cleaning_label(
                trace_label, blind_tone_modes, decorrelate_modes)
            _plot_psd(ax1, f, cleaned[panel1][row], psd_units, carrier,
                      label=clean_label, **clean_kwargs)
            _plot_psd(ax2, f, cleaned[panel2][row], psd_units, carrier2,
                      label=clean_label, **clean_kwargs)

    ylabel1, ylabel2 = _psd_axis_labels(
        format, psd_units, result['conversion_method'])
    ax1.set_ylabel(ylabel1)
    ax2.set_ylabel(ylabel2)
    ax2.set_xlabel('Frequency (Hz)')
    _legend(ax1)
    _legend(ax2)
    fig.suptitle('Power Spectral Density')
    fig.tight_layout()
    return (fig, result) if return_data else fig


def plot_timestream_on_resonance(ts_data, sweep_data, tone_index,
                                  deembed=False, phase_center=False,
                                  mag_centered=None, phase_centered=None,
                                  unwrap=False,
                                  hide_modulation_settling_points=False,
                                  fig=None, label=None,
                                  units='raw', config=None,
                                  apply_readout_correction=True, **kwargs):
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
            deembedding when both are True.  Sets the default for the
            per-panel ``mag_centered``/``phase_centered`` overrides below;
            the I vs Q panel always follows this value.
        mag_centered: bool or None, per-panel override for whether the
            magnitude-vs-frequency panel is phase-centered. None (default)
            follows phase_center. Centering flattens the resonance dip, so
            pass False to keep the magnitude structure while the phase panel
            stays centered.
        phase_centered: bool or None, per-panel override for whether the
            phase-vs-frequency panel is phase-centered. None (default)
            follows phase_center.
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
        apply_readout_correction: bool, plot with (True, default) or without
            (False) the filterbank compensation's software readout-flattening
            factors, applied consistently to the timestream samples and the
            sweep trace. Toggles already-parsed data either way when the
            factors are available.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    ts_data = with_readout_correction(ts_data, apply_readout_correction)
    sweep_data = with_readout_correction(sweep_data, apply_readout_correction)
    selected = _get_tone_data(ts_data, tones=[tone_index])
    key, i_arr, q_arr = selected[0]

    # Extract sweep trace for this tone
    sweep_f, sw_i, sw_q = sweep_trace(sweep_data, tone_index)

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

    # demodulate_timestream() output is already centre-referred, and when it was
    # given a calibration it is already de-embedded + phase-centred. Re-applying
    # the transforms here would double them (IQ thrown off the circle, magnitude
    # collapsed), and every sample already reads at the centre point rather than
    # its probe offset. So for demodulated data: skip the transforms on the
    # timestream (still transform the sweep, so the two share a frame) and place
    # all samples at the centre reference frequency.
    demodulated = bool(ts_data.get('modulation_demodulated'))
    if demodulated:
        ref_f = None
        drf = ts_data.get('dissipation_reference_frequency_hz')
        if drf is not None:
            try:
                ref_f = float(np.asarray(drf)[tone_index])
            except (IndexError, TypeError, ValueError):
                ref_f = None
        if ref_f is None or not np.isfinite(ref_f):
            ref_f = tone_freq
        ts_freq = np.full(len(z_ts), ref_f, dtype=float)
        if deembed or phase_center or phase_centered or mag_centered:
            warnings.warn(
                "ts_data is a demodulated timestream (already centre-referred, "
                "and calibrated demodulation is already de-embedded/phase-"
                "centred). Applying deembed/phase-centering to the sweep only, "
                "not to the timestream, and plotting every sample at the centre "
                "frequency.",
                stacklevel=2)

    # Deembedding is shared across all panels; apply it once at each trace's
    # probe frequencies. For a demodulated timestream the transform goes on the
    # sweep only -- the timestream is already in that basis.
    if deembed:
        z_sweep, d_params = _apply_deembed(sweep_f, z_sweep, True)
        if not demodulated:
            z_ts, _ = _apply_deembed(None, z_ts, d_params, frequency=ts_freq)

    visible = _visible_modulation_sample_mask(
        ts_data, len(z_ts), hide_modulation_settling_points)
    z_ts_plot = z_ts[visible]
    ts_freq_plot = ts_freq[visible]

    # Phase centering can be enabled independently per panel; each override
    # defaults to the shared phase_center. The I vs Q panel always follows
    # phase_center.
    iq_centered = phase_center
    phase_centered = phase_center if phase_centered is None else phase_centered
    mag_centered = phase_center if mag_centered is None else mag_centered

    def _maybe_center(enable, z_s, z_t):
        """Phase-center the sweep and timestream together, or pass through.

        A demodulated timestream is already phase-centred, so only the sweep is
        transformed (into the same frame); the timestream passes through.
        """
        if not enable:
            return z_s, z_t
        z_s, pc_params = _apply_phase_center(z_s, True)
        if demodulated:
            return z_s, z_t
        z_t, _ = _apply_phase_center(z_t, pc_params)
        return z_s, z_t

    z_sweep_iq, z_ts_iq = _maybe_center(iq_centered, z_sweep, z_ts_plot)
    z_sweep_ph, z_ts_ph = _maybe_center(phase_centered, z_sweep, z_ts_plot)
    z_sweep_mg, z_ts_mg = _maybe_center(mag_centered, z_sweep, z_ts_plot)

    # Compute magnitude and phase for the frequency-domain panels
    _, phase_sweep = _compute_mag_phase(z_sweep_ph, unwrap=unwrap)
    mag_sweep, _ = _compute_mag_phase(z_sweep_mg, unwrap=unwrap)
    mag_ts, _ = _compute_mag_phase(z_ts_mg, unwrap=unwrap)
    if unwrap:
        # The timestream samples cycle through the modulation frequencies in
        # time order, so np.unwrap along the sample axis mis-reads the jump
        # back to the start of each modulation cycle as a phase wrap. Instead
        # place each sample on the sweep's unwrapped branch at its own probe
        # frequency, which does not depend on the sample ordering.
        order = np.argsort(sweep_f)
        expected = np.interp(ts_freq_plot, np.asarray(sweep_f)[order],
                             phase_sweep[order])
        wrapped_ts = np.angle(z_ts_ph)
        phase_ts = wrapped_ts + 2 * np.pi * np.round(
            (expected - wrapped_ts) / (2 * np.pi))
    else:
        _, phase_ts = _compute_mag_phase(z_ts_ph, unwrap=False)

    if fig is None:
        fig, (ax_iq, ax_pf, ax_mf) = plt.subplots(1, 3, figsize=(20, 6))
        ax_mf.sharex(ax_pf)  # both frequency-domain panels share the x axis
    else:
        ax_iq, ax_pf, ax_mf = fig.axes[:3]

    ts_label = label if label is not None else 'Timestream'

    # Left panel: I vs Q resonance circle
    ax_iq.plot(z_sweep_iq.real, z_sweep_iq.imag, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_iq.plot(z_ts_iq.real, z_ts_iq.imag, '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_iq.set_xlabel(f'I {iq_label}'.strip())
    ax_iq.set_ylabel(f'Q {iq_label}'.strip())
    ax_iq.set_aspect('equal', adjustable='datalim')
    _legend(ax_iq)

    # Right panel: phase vs frequency
    ax_pf.plot(sweep_f, phase_sweep, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_pf.plot(ts_freq_plot, phase_ts,
               '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_pf.set_xlabel('Frequency (Hz)')
    ax_pf.set_ylabel('Phase (rad)')
    _legend(ax_pf)

    # Third panel: magnitude vs frequency
    ax_mf.plot(sweep_f, mag_sweep, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_mf.plot(ts_freq_plot, mag_ts,
               '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_mf.set_xlabel('Frequency (Hz)')
    ax_mf.set_ylabel(f'Magnitude {iq_label}'.strip())
    _legend(ax_mf)

    suffix = _transform_title_suffix(deembed, phase_center)
    fig.suptitle(f'Tone {tone_index} — Resonance Circle' + suffix)
    fig.tight_layout()
    return fig


def plot_allan_deviation(data, format='freq_diss', tones=None, sweep_data=None,
                         taus=None, kind='overlapping', show_errors=True,
                         fig=None, label=None, result=None, **kwargs):
    """
    Plot the Allan deviation of a timestream, one panel per quantity.

    A thin consumer of ``timeseries.allan_deviation()``: it takes the same
    arguments, and the ``tau_min`` of each trace is marked so the useful
    integration time is readable off the figure.

    Args:
        data: parsed timestream dict, complex array, or (i, q) pair.
        format: 'iq' | 'magphase' | 'freq_diss' | 'linewidths'.
        tones: tone indices to plot. None selects tone 0.
        sweep_data: required for format='freq_diss'.
        taus: averaging times in seconds. None gives octave spacing.
        kind: 'overlapping' (default) or 'standard'.
        show_errors: draw the ``adev/sqrt(n_pairs)`` error bars. The long-tau
            end rests on very few differences, so this is worth leaving on.
        fig: existing figure to overplot onto.
        label: legend label. None auto-increments.
        result: a precomputed ``allan_deviation()`` result to plot instead of
            recomputing. Its ``format`` and ``kind`` win over the arguments.
        **kwargs: passed to the plot calls. Also accepts any
            ``allan_deviation()`` keyword (``sample_rate``, cleaning options,
            conversion options).

    Returns:
        matplotlib.figure.Figure
    """
    from ..timeseries import allan_deviation

    if result is None:
        result = allan_deviation(
            data, format=format, tones=tones, sweep_data=sweep_data,
            taus=taus, kind=kind,
            **{k: kwargs.pop(k) for k in list(kwargs)
               if k in ('sample_rate', 'reference_tone_frequency',
                        'smooth_window_hz', 'conversion_method',
                        'calibrations', 'decorrelate_modes',
                        'decorrelate_tones', 'blind_tone_modes',
                        'blind_tones')})
    panels = result['panels']
    fig, ax1, ax2 = _dual_panels(fig, sharex=True, sharey=False)
    tau = result['tau']

    for ax, panel in zip((ax1, ax2), panels):
        for row, tone in enumerate(result['tones']):
            trace_label = _resolve_label(ax1, label, suffix=f'tone {tone}')
            y = result['adev'][panel][row]
            if show_errors:
                ax.errorbar(tau, y, yerr=result['adev_err'][panel][row],
                            label=trace_label,
                            **{**ERRORBAR_STYLE, **kwargs})
            else:
                ax.plot(tau, y, '.-', label=trace_label, **kwargs)
            t_min = result['tau_min'][panel][row]
            if np.isfinite(t_min):
                ax.axvline(t_min, linestyle=':', linewidth=0.8, alpha=0.5,
                           color=ax.lines[-1].get_color() if ax.lines else None)
        ax.set_xscale('log')
        ax.set_yscale('log')
        ax.set_ylabel(f'{panel} Allan deviation')
        _legend(ax)

    ax2.set_xlabel('Averaging time (s)')
    slopes = ', '.join(
        f'{p} {np.nanmedian(result["slope"][p]):+.2f}' for p in panels)
    fig.suptitle(f'Allan deviation ({result["kind"]}) — log-log slope {slopes}')
    fig.tight_layout()
    return fig


def plot_timestream_histogram(data, format='freq_diss', tones=None,
                              sweep_data=None, bins=100, show_gaussian=True,
                              fig=None, label=None, **kwargs):
    """
    Histogram a timestream against the Gaussian its robust sigma implies.

    The visual companion to ``timeseries.timestream_stats()``: where the
    histogram departs from the overlaid Gaussian is where ``std`` and
    ``mad_std`` disagree, which is what a PSD averages away.

    Args:
        data: parsed timestream dict, complex array, or (i, q) pair.
        format: 'iq' | 'magphase' | 'freq_diss' | 'linewidths'.
        tones: tone indices. None selects tone 0.
        sweep_data: required for format='freq_diss'.
        bins: histogram bins.
        show_gaussian: overlay a Gaussian of the same median and MAD-based
            sigma. Departures from it are the point of the plot.
        fig: existing figure to overplot onto.
        label: legend label. None auto-increments.
        **kwargs: passed to the plot calls; also accepts
            ``timestream_stats()`` keywords.

    Returns:
        matplotlib.figure.Figure
    """
    from ..timeseries import _panel_time_rows, timestream_stats

    passthrough = {k: kwargs.pop(k) for k in list(kwargs)
                   if k in ('sample_rate', 'reference_tone_frequency',
                            'smooth_window_hz', 'conversion_method',
                            'calibrations', 'decorrelate_modes',
                            'decorrelate_tones', 'blind_tone_modes',
                            'blind_tones')}
    rows, panels, tone_idx, _ = _panel_time_rows(
        data, format, tones, sweep_data,
        passthrough.get('sample_rate'),
        passthrough.get('reference_tone_frequency'),
        passthrough.get('smooth_window_hz', 1000),
        passthrough.get('conversion_method'),
        passthrough.get('calibrations'),
        passthrough.get('decorrelate_modes', 0),
        passthrough.get('decorrelate_tones'),
        passthrough.get('blind_tone_modes', 0),
        passthrough.get('blind_tones'))
    stats = timestream_stats(data, format=format, tones=tones,
                             sweep_data=sweep_data, **passthrough)

    fig, ax1, ax2 = _dual_panels(fig, sharex=False, sharey=False)
    for ax, panel in zip((ax1, ax2), panels):
        for row, tone in enumerate(tone_idx):
            x = rows[panel][row]
            x = x[np.isfinite(x)]
            trace_label = _resolve_label(ax1, label, suffix=f'tone {tone}')
            ax.hist(x, bins=bins, histtype='step', density=True,
                    label=trace_label, **kwargs)
            if show_gaussian:
                med = stats['median'][panel][row]
                sig = stats['mad_std'][panel][row]
                if np.isfinite(sig) and sig > 0:
                    grid = np.linspace(x.min(), x.max(), 300)
                    ax.plot(grid,
                            np.exp(-0.5 * ((grid - med) / sig) ** 2)
                            / (sig * np.sqrt(2 * np.pi)),
                            linestyle='--', linewidth=0.8, alpha=0.7)
        ax.set_yscale('log')
        ratio = np.nanmedian(stats['std'][panel] / stats['mad_std'][panel])
        ax.set_ylabel(f'{panel} density')
        ax.set_title(f'std / mad_std = {ratio:.2f}', fontsize='small')
        _legend(ax)

    ax2.set_xlabel('Value')
    fig.suptitle('Timestream distribution vs the Gaussian its MAD implies')
    fig.tight_layout()
    return fig


def plot_coherence(data, other=None, sample_rate=None, tones=None,
                   sweep_data=None, fig=None, label=None, result=None,
                   show_bias=True, **kwargs):
    """
    Plot frequency/dissipation coherence against its own significance floor.

    A thin consumer of ``timeseries.cross_psd()``. The point of the figure is
    the floor: magnitude-squared coherence is biased up by ``1/n_eff`` even for
    completely independent channels, so a trace is only meaningful where it
    rises above the shaded band.

    Args:
        data: a ``cross_psd()`` result, a parsed timestream dict (with
            ``sweep_data``), or the first row as an array.
        other: second row, when ``data`` is an array.
        sample_rate: required for plain arrays.
        tones: tone indices.
        sweep_data: required to convert a raw capture.
        fig: existing figure to overplot onto.
        label: legend label. None auto-increments.
        result: a precomputed ``cross_psd()`` result to plot.
        show_bias: shade the bias floor and draw the 95% significance line.
        **kwargs: passed to the plot calls, and to ``cross_psd()``.

    Returns:
        matplotlib.figure.Figure
    """
    from ..timeseries import cross_psd

    if result is None:
        result = data if (isinstance(data, Mapping) and 'coherence' in data) \
            else cross_psd(data, other, sample_rate=sample_rate, tones=tones,
                           sweep_data=sweep_data,
                           **{k: kwargs.pop(k) for k in list(kwargs)
                              if k in ('psd_kwargs', 'panels',
                                       'reference_tone_frequency',
                                       'smooth_window_hz', 'conversion_method',
                                       'calibrations',
                                       'apply_readout_correction')})
    fig, ax1, ax2 = _dual_panels(fig, sharex=True, sharey=False)
    f = result['f']

    for row, tone in enumerate(result['tones']):
        trace_label = _resolve_label(ax1, label, suffix=f'tone {tone}')
        ax1.plot(f, np.atleast_2d(result['coherence'])[row],
                 linewidth=0.7, label=trace_label, **kwargs)
        ax2.plot(f, np.degrees(np.atleast_2d(result['cross_phase'])[row]),
                 linewidth=0.7, label=trace_label, **kwargs)

    if show_bias:
        bias = result.get('coherence_bias')
        c95 = result.get('coherence_95')
        if bias is not None:
            ax1.axhspan(0, float(bias), color='grey', alpha=0.2,
                        label=f'bias floor 1/n_eff = {float(bias):.3f}')
        if c95 is not None:
            ax1.axhline(float(c95), linestyle='--', linewidth=0.8,
                        color='k', alpha=0.6,
                        label=f'95% significance = {float(c95):.3f}')

    ax1.set_xscale('log')
    ax1.set_ylim(0, 1)
    ax1.set_ylabel('Magnitude-squared coherence')
    ax2.set_xscale('log')
    ax2.set_ylabel('Cross phase (deg)')
    ax2.set_xlabel('Frequency (Hz)')
    _legend(ax1)
    _legend(ax2)
    n_eff = result.get('n_eff')
    fig.suptitle(
        'Frequency/dissipation coherence'
        + (f' — n_eff = {float(n_eff):.1f}' if n_eff is not None else ''))
    fig.tight_layout()
    return fig


def plot_composite_psd(traces, panel=None, tone=None, fig=None, label=None,
                       gr_fit=None, show_gap=True, show_alias=True, **kwargs):
    """
    Plot a stitched wideband PSD with its gap and alias-affected band marked.

    A thin consumer of ``timeseries.stitch_psds()``. The shading is the
    substance: the accumulated and snapshot bands do not overlap at the usual
    ``acc_len``, and nothing is interpolated across the gap.

    Args:
        traces: a ``stitch_psds()`` result, or a sequence of
            ``timestream_psds()`` / ``snapshot_psds()`` results to stitch here.
        panel: panel name to plot. None takes the first.
        tone: absolute tone index. None plots every tone in the result.
        fig: existing figure to overplot onto.
        label: legend label. None auto-increments.
        gr_fit: a ``fit_gr_rolloff()`` result to overlay, with ``tau_qp`` and
            the fixed ``tau_res`` in the legend.
        show_gap: shade the uncovered bands.
        show_alias: shade the top of the accumulated band, where a falling
            spectrum reads low to the accumulator's alias fold.
        **kwargs: passed to the plot calls, and to ``stitch_psds()``.

    Returns:
        matplotlib.figure.Figure
    """
    from ..timeseries import _gr_model, stitch_psds

    if isinstance(traces, Mapping) and 'gaps' in traces:
        result = traces
    else:
        result = stitch_psds(
            traces, **{k: kwargs.pop(k) for k in list(kwargs)
                       if k in ('overlap', 'allow_cleaning_mismatch',
                                'panels')})
    panel = panel or result['panels'][0]
    plt = _get_pyplot()
    if fig is None:
        fig, ax = plt.subplots(figsize=(9, 5))
    else:
        ax = fig.axes[0]

    f = result['f']
    rows = np.atleast_2d(result['psd'][panel])
    tone_list = list(result['tones'])
    for row, t in enumerate(tone_list):
        if tone is not None and t != tone:
            continue
        trace_label = _resolve_label(ax, label, suffix=f'tone {t}')
        ax.loglog(f, rows[row], linewidth=0.7, label=trace_label, **kwargs)

    if show_gap:
        for n, (lo, hi) in enumerate(result['gaps']):
            ax.axvspan(lo, hi, color='grey', alpha=0.25,
                       label='no coverage' if n == 0 else None)
    if show_alias and result.get('alias_affected'):
        lo, hi = result['alias_affected']
        ax.axvspan(lo, hi, color='orange', alpha=0.15,
                   label='alias-affected (falling spectra read low)')

    if gr_fit is not None:
        gp = panel if panel in gr_fit['tau_qp'] else list(gr_fit['tau_qp'])[0]
        for row, t in enumerate(tone_list):
            if tone is not None and t != tone:
                continue
            tq = gr_fit['tau_qp'][gp][row]
            if not np.isfinite(tq):
                continue
            grid = np.logspace(np.log10(np.nanmin(f[f > 0])),
                               np.log10(np.nanmax(f)), 400)
            # tau_res may be one value per tone; take this tone's.
            tr = np.atleast_1d(np.asarray(gr_fit['tau_res'], dtype=float))
            tr = float(tr[row if tr.size > 1 else 0])
            ax.loglog(grid,
                      _gr_model(grid, gr_fit['S_0'][gp][row], tq,
                                gr_fit['white'][gp][row], tr),
                      linestyle='--', linewidth=1.0,
                      label=f'GR fit: tau_qp {tq * 1e6:.0f} us, '
                            f'tau_res {tr * 1e6:.0f} us (fixed)')

    for record in result.get('overlaps', []):
        ratio = record['ratio'].get(panel)
        if ratio is not None and np.isfinite(ratio):
            lo, hi = record['band']
            ax.annotate(f'{10 * np.log10(ratio):+.1f} dB',
                        xy=(np.sqrt(lo * hi), ax.get_ylim()[1]),
                        ha='center', va='top', fontsize='small', alpha=0.7)

    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel(f'{panel} PSD (1/Hz)')
    _legend(ax)
    fig.suptitle('Composite wideband PSD')
    fig.tight_layout()
    return fig
