"""
Sweep data plotting functions.

Supports wideband sweeps (single concatenated trace) and per-tone sweeps
(multiple tones). Formats: I vs Q, I/Q vs frequency, magnitude/phase vs
frequency, all with optional deembedding or phase centering and error bars.
"""

import numpy as np
from ._common import (_get_pyplot, _compute_mag_phase, _propagate_errors_mag,
                       _apply_deembed, _apply_phase_center,
                       ERROR_FILL_STYLE, _resolve_label,
                       _normalise_iq, _compute_mag_phase_units, UNITS)


def _extract_traces(sweep_data, tones=None):
    """
    Extract per-trace arrays from a sweep data dict.

    Returns list of (f, si, sq, ei, eq, tone_label) tuples.
    """
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])
    ei = np.atleast_2d(sweep_data.get('sweep_ei', np.zeros_like(si)))
    eq = np.atleast_2d(sweep_data.get('sweep_eq', np.zeros_like(sq)))

    is_wideband = sweep_data.get('wideband_sweep', False)

    if is_wideband or sf.shape[0] == 1:
        # Single concatenated trace: shape (1, N)
        return [(sf[0], si[0], sq[0], ei[0], eq[0], None)]

    # Per-tone: shape (N_points, N_tones)
    n_points, n_tones = sf.shape
    if tones is None:
        tones = list(range(n_tones))

    traces = []
    for t in tones:
        traces.append((sf[:, t], si[:, t], sq[:, t],
                        ei[:, t], eq[:, t], t))
    return traces


def _apply_transforms(f, z, deembed, phase_center, ei=None, eq=None,
                      group_delay_cal=None):
    """Apply deembed and/or phase_center to a complex trace and errors.

    Returns (z_out, ei_out, eq_out, deembed_params, phase_center_params).
    ei_out/eq_out are None when the input errors are None.
    """
    d_params = None
    pc_params = None
    if deembed:
        if ei is None or eq is None:
            z, d_params = _apply_deembed(f, z, deembed,
                                         group_delay_cal=group_delay_cal)
        else:
            from .. import resonator
            z_err = ei + 1j * eq
            if deembed is True:
                z, z_err, d_params = resonator.deembed(
                    f, z, group_delay_cal=group_delay_cal, s21_err=z_err)
            elif isinstance(deembed, dict):
                z, z_err = resonator.apply_deembed_params(
                    z, deembed, s21_err=z_err)
                d_params = deembed
            else:
                z, d_params = _apply_deembed(f, z, deembed,
                                             group_delay_cal=group_delay_cal)
            if d_params is not None:
                ei = z_err.real
                eq = z_err.imag
    elif group_delay_cal is not None:
        # Remove group delay without full deembedding (no baseline
        # normalisation).  This corrects the linear phase slope so that
        # phase-vs-frequency plots show the resonator response only.
        from ..resonator import remove_group_delay
        if ei is None or eq is None:
            z, _ = remove_group_delay(f, z, group_delay_cal)
        else:
            z, z_err, _ = remove_group_delay(
                f, z, group_delay_cal, s21_err=ei + 1j * eq)
            ei = z_err.real
            eq = z_err.imag
    if phase_center:
        if ei is None or eq is None:
            z, pc_params = _apply_phase_center(z, phase_center)
        else:
            from .. import resonator
            z_err = ei + 1j * eq
            if phase_center is True:
                z, z_err, pc_params = resonator.phase_center(
                    z, s21_err=z_err)
            elif isinstance(phase_center, dict):
                z, z_err = resonator.apply_phase_center_params(
                    z, phase_center, s21_err=z_err)
                pc_params = phase_center
            else:
                z, pc_params = _apply_phase_center(z, phase_center)
            if pc_params is not None:
                ei = z_err.real
                eq = z_err.imag
    return z, ei, eq, d_params, pc_params


def _transform_title_suffix(deembed, phase_center):
    """Return a parenthesised title suffix describing active transforms."""
    parts = []
    if deembed:
        parts.append('deembedded')
    if phase_center:
        parts.append('phase-centered')
    return f' ({", ".join(parts)})' if parts else ''


def plot_sweep(sweep_data, format='magphase', tones=None, deembed=False,
               phase_center=False, show_errors=None, multi_tone='overlay',
               fig=None, label=None, units='raw', config=None,
               reference_plane='adc_input', group_delay_cal=None, **kwargs):
    """
    General-purpose sweep plot.

    Args:
        sweep_data: dict with keys 'sweep_f', 'sweep_i', 'sweep_q',
            optionally 'sweep_ei', 'sweep_eq'.
        format: 'iq' | 'iq_vs_f' | 'magphase'
        tones: List of tone indices to plot, or None for all.
        deembed: bool, apply true RF deembedding (cable delay removal +
            baseline normalisation).  Off-resonance → (1, 0).
        phase_center: bool, apply phase centering (circle centering +
            rotation).  Applied after deembedding when both are True.
        show_errors: True/False/None.  None (default) draws error fills for
            per-tone sweeps but skips them for wideband sweeps, where the
            concatenated trace can be hundreds of thousands of points and
            ``fill_between`` becomes the dominant render cost.  Pass ``True``
            to force errors on (slow for large wideband sweeps) or ``False``
            to always skip.
        multi_tone: 'overlay' (shared axes) or 'grid' (one subplot per tone).
        fig: Existing figure. If None, create new.
        label: Legend label. If None, uses an auto-incrementing index.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator codes, no normalisation.
            'peak' - normalise to the peak magnitude of the data.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs' - dB relative to ADC full-scale.
            'dbm' - estimated power in dBm at ``reference_plane``.
            All options except 'raw' and 'peak' require
            'info' in sweep_data.
        config: Config dict (needed for 'dbm' and non-default rx_mix_scale).
        reference_plane: Reference plane used when ``units='dbm'``.  One of
            'adc_input' (default) or 'cryostat_output'.  The latter removes
            the RX analog chain gain using the frequency-dependent calibration
            entries in ``config['rf_frontend']`` and ``config['cryostat']``;
            falls back to 'adc_input' with a warning if that cal is not
            available.  Ignored when ``units != 'dbm'``.
        group_delay_cal: Frequency-dependent group delay calibration from
            ``measure_path_group_delay()``.  Used when ``deembed=True`` to
            remove the measured path group delay instead of auto-estimating
            a scalar cable delay.
        **kwargs: Passed to matplotlib plot/errorbar calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    custom_title = kwargs.pop('title', None)
    traces = _extract_traces(sweep_data, tones)
    info = sweep_data.get('info')

    # Normalise traces
    if units != 'raw':
        if units != 'peak' and info is None:
            raise ValueError("sweep_data must contain 'info' for non-raw units.")
        normalised = []
        for f, si, sq, ei, eq, tidx in traces:
            si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
                si, sq, units, info, config=config, ei=ei, eq=eq,
                reference_plane=reference_plane, frequencies=f)
            normalised.append((f, si, sq, ei, eq, tidx))
        traces = normalised
    else:
        iq_label = ''
        mag_label = '|S21| (dB)'

    if show_errors is None:
        # Auto: skip error fills on wideband sweeps where fill_between is slow.
        show_errors = not sweep_data.get('wideband_sweep', False)
    has_errors = show_errors and any(np.any(ei != 0) for _, _, _, ei, _, _ in traces)
    n_traces = len(traces)
    is_multi = n_traces > 1 and traces[0][5] is not None
    suffix = _transform_title_suffix(deembed, phase_center)

    # Determine subplot layout
    if is_multi and multi_tone == 'grid':
        n_cols = min(n_traces, 4)
        n_rows = int(np.ceil(n_traces / n_cols))
        if format == 'iq':
            if fig is None:
                fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 4 * n_rows))
            else:
                axes = np.array(fig.axes).reshape(n_rows, n_cols)
            axes_flat = np.atleast_1d(axes).ravel()
            for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
                ax = axes_flat[i]
                z = si + 1j * sq
                z, _, _, _, _ = _apply_transforms(f, z, deembed, phase_center,
                                                  group_delay_cal=group_delay_cal)
                trace_label = _resolve_label(ax, label, suffix=f'Tone {tidx}' if tidx is not None else None)
                ax.plot(z.real, z.imag, linewidth=0.8, label=trace_label, **kwargs)
                ax.set_aspect('equal', adjustable='datalim')
                ax.set_title(f'Tone {tidx}')
                ax.set_xlabel(f'I {iq_label}'.strip())
                ax.set_ylabel(f'Q {iq_label}'.strip())
                ax.legend(fontsize='small')
            # Hide unused axes
            for i in range(n_traces, len(axes_flat)):
                axes_flat[i].set_visible(False)
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig
        else:
            # Two rows per tone for dual-axis formats
            if fig is None:
                fig, axes = plt.subplots(n_traces, 2, figsize=(12, 3 * n_traces),
                                         squeeze=False)
            else:
                axes = np.array(fig.axes).reshape(n_traces, 2)
            for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
                trace_label = _resolve_label(axes[i, 0], label,
                                             suffix=f'Tone {tidx}' if tidx is not None else None)
                _plot_single_trace(axes[i, 0], axes[i, 1], f, si, sq, ei, eq,
                                   format, deembed, phase_center,
                                   has_errors, trace_label,
                                   units=units, info=info, config=config,
                                   reference_plane=reference_plane,
                                   iq_label=iq_label, mag_label=mag_label,
                                   group_delay_cal=group_delay_cal,
                                   **kwargs)
                axes[i, 0].legend(fontsize='small')
                axes[i, 1].legend(fontsize='small')
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig

    # Overlay or single trace
    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for f, si, sq, ei, eq, tidx in traces:
            z = si + 1j * sq
            z, _, _, _, _ = _apply_transforms(f, z, deembed, phase_center,
                                                  group_delay_cal=group_delay_cal)
            trace_label = _resolve_label(ax, label,
                                         suffix=f'Tone {tidx}' if tidx is not None else None)
            ax.plot(z.real, z.imag, linewidth=0.8, label=trace_label, **kwargs)
        ax.set_xlabel(f'I {iq_label}'.strip())
        ax.set_ylabel(f'Q {iq_label}'.strip())
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend(fontsize='small')
        ax.set_title(custom_title if custom_title is not None
                     else 'S21 Complex Plane' + suffix)
        plt.tight_layout()
        return fig

    # Dual-axis formats: magphase or iq_vs_f
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for f, si, sq, ei, eq, tidx in traces:
        trace_label = _resolve_label(ax1, label,
                                     suffix=f'Tone {tidx}' if tidx is not None else None)
        _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                           format, deembed, phase_center,
                           has_errors, trace_label,
                           units=units, info=info, config=config,
                           reference_plane=reference_plane,
                           iq_label=iq_label, mag_label=mag_label,
                           group_delay_cal=group_delay_cal,
                           **kwargs)

    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')

    if custom_title is not None:
        title = custom_title
    else:
        bw = sweep_data.get('bandwidth_hz')
        title = f'Sweep ({bw/1e6:.1f} MHz)' if bw else 'Sweep'
        title += suffix
    fig.suptitle(title)
    plt.tight_layout()
    return fig


def _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                        format, deembed, phase_center,
                        has_errors, label,
                        units='raw', info=None, config=None,
                        reference_plane='adc_input',
                        iq_label='', mag_label='|S21| (dB)',
                        group_delay_cal=None, **kwargs):
    """Plot a single trace on a pair of axes."""
    z = si + 1j * sq
    z, ei, eq, _, _ = _apply_transforms(f, z, deembed, phase_center, ei, eq,
                                        group_delay_cal=group_delay_cal)
    si, sq = z.real, z.imag

    f_mhz = f / 1e6

    if format == 'magphase':
        mag_db, phase = _compute_mag_phase_units(
            z, units, info=info, config=config,
            reference_plane=reference_plane, frequencies=f)
        if has_errors and np.any(ei != 0):
            e_mag, e_phase = _propagate_errors_mag(si, sq, ei, eq)
            line, = ax1.plot(f_mhz, mag_db, linewidth=0.8, label=label, **kwargs)
            ax1.fill_between(f_mhz, mag_db - e_mag, mag_db + e_mag,
                             color=line.get_color(), **ERROR_FILL_STYLE)
            line, = ax2.plot(f_mhz, phase, linewidth=0.8, label=label, **kwargs)
            ax2.fill_between(f_mhz, phase - e_phase, phase + e_phase,
                             color=line.get_color(), **ERROR_FILL_STYLE)
        else:
            ax1.plot(f_mhz, mag_db, linewidth=0.8, label=label, **kwargs)
            ax2.plot(f_mhz, phase, linewidth=0.8, label=label, **kwargs)
        ax1.set_ylabel(mag_label)
        ax2.set_ylabel('Phase (rad)')
        ax2.set_xlabel('Frequency (MHz)')

    elif format == 'iq_vs_f':
        if has_errors and np.any(ei != 0):
            line, = ax1.plot(f_mhz, si, linewidth=0.8, label=label, **kwargs)
            ax1.fill_between(f_mhz, si - ei, si + ei,
                             color=line.get_color(), **ERROR_FILL_STYLE)
            line, = ax2.plot(f_mhz, sq, linewidth=0.8, label=label, **kwargs)
            ax2.fill_between(f_mhz, sq - eq, sq + eq,
                             color=line.get_color(), **ERROR_FILL_STYLE)
        else:
            ax1.plot(f_mhz, si, linewidth=0.8, label=label, **kwargs)
            ax2.plot(f_mhz, sq, linewidth=0.8, label=label, **kwargs)
        ax1.set_ylabel(f'I {iq_label}'.strip())
        ax2.set_ylabel(f'Q {iq_label}'.strip())
        ax2.set_xlabel('Frequency (MHz)')

    else:
        raise ValueError(f"Unknown format '{format}'. Use 'iq', 'iq_vs_f', or 'magphase'.")


def _coerce_fit_results(fit_results):
    """Return fit results as a non-empty list."""
    if fit_results is None:
        raise ValueError('fit_results must contain at least one fit result.')
    if hasattr(fit_results, 'z_data') and hasattr(fit_results, 'z_fit'):
        fit_results = [fit_results]
    else:
        fit_results = list(fit_results)
    if not fit_results:
        raise ValueError('fit_results must contain at least one fit result.')
    return fit_results


def _fit_parameter_values(fit_results, name):
    """Return one real-valued fit parameter array."""
    if not all(hasattr(fit, name) for fit in fit_results):
        raise ValueError(f"plot_fit_params() unknown fit parameter: {name!r}")
    values = np.asarray([getattr(fit, name) for fit in fit_results])
    if np.iscomplexobj(values):
        raise ValueError(
            f"plot_fit_params() only supports real-valued fit parameters; {name!r} is complex."
        )
    return values


def _fit_param_color_values(fit_results, color):
    """Resolve color as either a literal Matplotlib color or fit parameter."""
    if color is None:
        return None, None
    if isinstance(color, str) and all(hasattr(fit, color) for fit in fit_results):
        return _fit_parameter_values(fit_results, color), color

    color_values = np.asarray(color)
    if color_values.ndim == 0:
        return None, None
    if color_values.shape != (len(fit_results),):
        return None, None
    if np.iscomplexobj(color_values):
        raise ValueError('plot_fit_params() color values must be real-valued.')
    if np.issubdtype(color_values.dtype, np.number) or color_values.dtype == bool:
        return color_values, None
    return None, None


def _fit_trace_title(fit, index):
    """Return a compact per-fit title/label suffix."""
    fr = float(getattr(fit, 'fr', np.nan))
    if np.isfinite(fr):
        return f'{fr / 1e6:.3f} MHz'
    return f'Fit {index + 1}'


def _fit_trace_label(label, fit, index, total):
    """Resolve the legend label for one fit trace."""
    suffix = _fit_trace_title(fit, index) if total > 1 else None
    if label is None:
        return suffix or 'Fit'
    return f'{label} {suffix}' if suffix else label


def _fit_has_errors(fit):
    """Return True when a fit result carries non-zero I/Q uncertainties."""
    z_err = getattr(fit, 'z_err_data', None)
    if z_err is None:
        return False
    z_err = np.asarray(z_err)
    if np.iscomplexobj(z_err):
        return np.any(z_err.real != 0) or np.any(z_err.imag != 0)
    return np.any(z_err != 0)


def _fit_error_arrays(fit, size):
    """Return sigma_I and sigma_Q arrays from a fit result."""
    z_err = getattr(fit, 'z_err_data', None)
    if z_err is None:
        return None, None
    z_err = np.asarray(z_err)
    if z_err.size != size:
        z_err = z_err.reshape(size)
    if np.iscomplexobj(z_err):
        return np.abs(z_err.real).ravel(), np.abs(z_err.imag).ravel()
    err = np.abs(z_err.astype(float, copy=False)).ravel()
    return err, err


def _normalise_fit_trace(f, z_data, z_fit, ei, eq, units='raw', info=None,
                         config=None, reference_plane='adc_input'):
    """Normalise one data/model trace pair using plot_sweep conventions."""
    if units == 'raw':
        return z_data, z_fit, ei, eq, '', '|S21| (dB)'

    if units == 'peak':
        peak = np.max(np.abs(z_data))
        if peak == 0:
            peak = 1.0
        z_data = z_data / peak
        z_fit = z_fit / peak
        if ei is not None:
            ei = ei / peak
            eq = eq / peak
        return (z_data, z_fit, ei, eq,
                '(peak-normalised)', '|S21| (dB, peak-normalised)')

    if info is None:
        raise ValueError(
            "plot_fits() requires info=... for non-raw units because "
            'FitResult does not store sweep metadata.'
        )

    si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
        z_data.real, z_data.imag, units, info, config=config,
        ei=ei, eq=eq, reference_plane=reference_plane, frequencies=f)
    fit_si, fit_sq, _, _, _, _ = _normalise_iq(
        z_fit.real, z_fit.imag, units, info, config=config,
        reference_plane=reference_plane, frequencies=f)
    return si + 1j * sq, fit_si + 1j * fit_sq, ei, eq, iq_label, mag_label


def _transform_fit_model(f, z_fit, deembed_params=None, phase_center_params=None,
                         group_delay_cal=None):
    """Apply a data-derived transform to the fitted model trace."""
    if deembed_params is not None:
        from .. import resonator
        z_fit = resonator.apply_deembed_params(z_fit, deembed_params,
                                               frequency=f)
    elif group_delay_cal is not None:
        z_fit, _, _, _, _ = _apply_transforms(
            f, z_fit, False, False, group_delay_cal=group_delay_cal)

    if phase_center_params is not None:
        from .. import resonator
        z_fit = resonator.apply_phase_center_params(z_fit, phase_center_params)
    return z_fit


def _prepare_fit_trace(fit, deembed=False, phase_center=False, units='raw',
                       info=None, config=None,
                       reference_plane='adc_input', group_delay_cal=None):
    """Return one fit trace pair in the requested plotting coordinates."""
    f = getattr(fit, 'f_data', None)
    z_data = getattr(fit, 'z_data', None)
    z_fit = getattr(fit, 'z_fit', None)
    if f is None or z_data is None or z_fit is None:
        raise ValueError(
            'Each fit result must include f_data, z_data, and z_fit arrays.'
        )

    f = np.asarray(f, float).ravel()
    z_data = np.asarray(z_data, complex).ravel()
    z_fit = np.asarray(z_fit, complex).ravel()
    if f.size != z_data.size or f.size != z_fit.size:
        raise ValueError('FitResult f_data, z_data, and z_fit must have matching lengths.')

    ei, eq = _fit_error_arrays(fit, f.size)
    z_data, z_fit, ei, eq, iq_label, mag_label = _normalise_fit_trace(
        f, z_data, z_fit, ei, eq, units=units, info=info, config=config,
        reference_plane=reference_plane)

    z_data, ei, eq, deembed_params, phase_center_params = _apply_transforms(
        f, z_data, deembed, phase_center, ei, eq,
        group_delay_cal=group_delay_cal)
    z_fit = _transform_fit_model(
        f, z_fit, deembed_params=deembed_params,
        phase_center_params=phase_center_params,
        group_delay_cal=group_delay_cal)
    return f, z_data, z_fit, ei, eq, iq_label, mag_label


def _base_fit_styles(kwargs, data_kwargs=None, fit_kwargs=None):
    """Build default marker/line styles for plot_fits()."""
    data_specific = {} if data_kwargs is None else dict(data_kwargs)
    fit_specific = {} if fit_kwargs is None else dict(fit_kwargs)

    data_style = dict(kwargs)
    fit_style = dict(kwargs)
    data_style.update(data_specific)
    fit_style.update(fit_specific)

    if 'linestyle' not in data_specific:
        data_style['linestyle'] = 'None'
    if 'marker' not in data_specific:
        data_style['marker'] = 'o'
    data_style.setdefault('markersize', 3)

    if 'marker' not in fit_specific:
        fit_style['marker'] = None
    fit_style.setdefault('linestyle', '-')
    fit_style.setdefault('linewidth', 0.8)
    return data_style, fit_style


def _pair_trace_styles(ax, data_style, fit_style):
    """Assign a shared color to the data markers and fit line."""
    data_style = dict(data_style)
    fit_style = dict(fit_style)
    color = fit_style.get('color', data_style.get('color'))
    if color is None:
        color = ax._get_lines.get_next_color()
    data_style.setdefault('color', color)
    fit_style.setdefault('color', color)
    return data_style, fit_style


def _plot_single_fit_trace(ax1, ax2, fit, format,
                           deembed=False, phase_center=False,
                           has_errors=False, data_label=None,
                           fit_label='_nolegend_', units='raw',
                           info=None, config=None,
                           reference_plane='adc_input',
                           group_delay_cal=None,
                           data_style=None, fit_style=None):
    """Plot one fit result as markers for data and a line for the model."""
    f, z_data, z_fit, ei, eq, iq_label, mag_label = _prepare_fit_trace(
        fit, deembed=deembed, phase_center=phase_center, units=units,
        info=info, config=config, reference_plane=reference_plane,
        group_delay_cal=group_delay_cal)
    data_style, fit_style = _pair_trace_styles(ax1, data_style, fit_style)

    if format == 'iq':
        ax1.plot(z_fit.real, z_fit.imag, label=fit_label, **fit_style)
        ax1.plot(z_data.real, z_data.imag, label=data_label, **data_style)
        ax1.set_xlabel(f'I {iq_label}'.strip())
        ax1.set_ylabel(f'Q {iq_label}'.strip())
        ax1.set_aspect('equal', adjustable='datalim')
        return

    f_mhz = f / 1e6
    if format == 'magphase':
        fit_mag, fit_phase = _compute_mag_phase_units(
            z_fit, units, info=info, config=config,
            reference_plane=reference_plane, frequencies=f)
        data_mag, data_phase = _compute_mag_phase_units(
            z_data, units, info=info, config=config,
            reference_plane=reference_plane, frequencies=f)
        ax1.plot(f_mhz, fit_mag, label=fit_label, **fit_style)
        ax2.plot(f_mhz, fit_phase, label=fit_label, **fit_style)
        if has_errors and ei is not None and np.any(ei != 0):
            e_mag, e_phase = _propagate_errors_mag(
                z_data.real, z_data.imag, ei, eq)
            ax1.fill_between(
                f_mhz, data_mag - e_mag, data_mag + e_mag,
                color=data_style['color'], **ERROR_FILL_STYLE)
            ax2.fill_between(
                f_mhz, data_phase - e_phase, data_phase + e_phase,
                color=data_style['color'], **ERROR_FILL_STYLE)
        ax1.plot(f_mhz, data_mag, label=data_label, **data_style)
        ax2.plot(f_mhz, data_phase, label=data_label, **data_style)
        ax1.set_ylabel(mag_label)
        ax2.set_ylabel('Phase (rad)')
        ax2.set_xlabel('Frequency (MHz)')
        return

    if format == 'iq_vs_f':
        ax1.plot(f_mhz, z_fit.real, label=fit_label, **fit_style)
        ax2.plot(f_mhz, z_fit.imag, label=fit_label, **fit_style)
        if has_errors and ei is not None and np.any(ei != 0):
            ax1.fill_between(
                f_mhz, z_data.real - ei, z_data.real + ei,
                color=data_style['color'], **ERROR_FILL_STYLE)
            ax2.fill_between(
                f_mhz, z_data.imag - eq, z_data.imag + eq,
                color=data_style['color'], **ERROR_FILL_STYLE)
        ax1.plot(f_mhz, z_data.real, label=data_label, **data_style)
        ax2.plot(f_mhz, z_data.imag, label=data_label, **data_style)
        ax1.set_ylabel(f'I {iq_label}'.strip())
        ax2.set_ylabel(f'Q {iq_label}'.strip())
        ax2.set_xlabel('Frequency (MHz)')
        return

    raise ValueError(f"Unknown format '{format}'. Use 'iq', 'iq_vs_f', or 'magphase'.")


def plot_fits(fit_results, format='magphase', deembed=False,
              phase_center=False, show_errors=None,
              multi_tone='overlay', fig=None, label=None,
              units='raw', info=None, config=None,
              reference_plane='adc_input', group_delay_cal=None,
              show_fit_legend=False, data_kwargs=None, fit_kwargs=None,
              **kwargs):
    """Plot one or more FitResult traces using the sweep plotting styles.

    Args:
        fit_results: One FitResult or an iterable of FitResult objects.
        format: 'iq' | 'iq_vs_f' | 'magphase'
        deembed: Apply RF deembedding before plotting.
        phase_center: Apply phase centering before plotting.
        show_errors: True/False/None. ``None`` draws error bands when any fit
            result carries ``z_err_data``.
        multi_tone: 'overlay' (shared axes) or 'grid' (one subplot per fit).
        fig: Existing figure. If None, create a new one.
        label: Optional legend label prefix.
        units: Same units as plot_sweep(). For non-raw units, pass ``info``.
        info: Sweep metadata needed for non-raw units. FitResult does not
            retain the original sweep_data dict.
        config: Plotting config used for units such as 'dbm'.
        reference_plane: Reference plane used when ``units='dbm'``.
        group_delay_cal: Group delay calibration forwarded to sweep transforms.
        show_fit_legend: If True, add separate legend entries for fit lines.
        data_kwargs: Matplotlib keyword overrides for the data markers only.
        fit_kwargs: Matplotlib keyword overrides for the fit lines only.
        **kwargs: Base matplotlib keyword arguments applied to both data and
            fit traces before the marker/line defaults are enforced.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    custom_title = kwargs.pop('title', None)
    fit_results = _coerce_fit_results(fit_results)
    if units != 'raw' and units != 'peak' and info is None:
        raise ValueError(
            "plot_fits() requires info=... for non-raw units because "
            'FitResult does not store sweep metadata.'
        )

    if show_errors is None:
        show_errors = any(_fit_has_errors(fit) for fit in fit_results)

    n_traces = len(fit_results)
    is_multi = n_traces > 1
    suffix = _transform_title_suffix(deembed, phase_center)
    data_style_base, fit_style_base = _base_fit_styles(
        kwargs, data_kwargs=data_kwargs, fit_kwargs=fit_kwargs)

    if is_multi and multi_tone == 'grid':
        if format == 'iq':
            n_cols = min(n_traces, 4)
            n_rows = int(np.ceil(n_traces / n_cols))
            if fig is None:
                fig, axes = plt.subplots(
                    n_rows, n_cols, figsize=(4 * n_cols, 4 * n_rows))
            else:
                axes = np.array(fig.axes).reshape(n_rows, n_cols)
            axes_flat = np.atleast_1d(axes).ravel()
            for i, fit in enumerate(fit_results):
                ax = axes_flat[i]
                trace_label = _fit_trace_label(label, fit, i, n_traces)
                fit_label = (
                    f'{trace_label} fit' if show_fit_legend else '_nolegend_'
                )
                _plot_single_fit_trace(
                    ax, None, fit, format, deembed=deembed,
                    phase_center=phase_center, has_errors=show_errors,
                    data_label=trace_label, fit_label=fit_label,
                    units=units, info=info, config=config,
                    reference_plane=reference_plane,
                    group_delay_cal=group_delay_cal,
                    data_style=data_style_base, fit_style=fit_style_base)
                ax.set_title(_fit_trace_title(fit, i))
                ax.legend(fontsize='small')
            for i in range(n_traces, len(axes_flat)):
                axes_flat[i].set_visible(False)
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig

        if fig is None:
            fig, axes = plt.subplots(n_traces, 2, figsize=(12, 3 * n_traces),
                                     squeeze=False)
        else:
            axes = np.array(fig.axes).reshape(n_traces, 2)
        for i, fit in enumerate(fit_results):
            trace_label = _fit_trace_label(label, fit, i, n_traces)
            fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
            _plot_single_fit_trace(
                axes[i, 0], axes[i, 1], fit, format, deembed=deembed,
                phase_center=phase_center, has_errors=show_errors,
                data_label=trace_label, fit_label=fit_label,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                data_style=data_style_base, fit_style=fit_style_base)
            axes[i, 0].legend(fontsize='small')
            axes[i, 1].legend(fontsize='small')
            axes[i, 0].set_title(_fit_trace_title(fit, i))
        if custom_title is not None:
            fig.suptitle(custom_title)
        plt.tight_layout()
        return fig

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for i, fit in enumerate(fit_results):
            trace_label = _fit_trace_label(label, fit, i, n_traces)
            fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
            _plot_single_fit_trace(
                ax, None, fit, format, deembed=deembed,
                phase_center=phase_center, has_errors=show_errors,
                data_label=trace_label, fit_label=fit_label,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                data_style=data_style_base, fit_style=fit_style_base)
        ax.legend(fontsize='small')
        ax.set_title(custom_title if custom_title is not None
                     else 'Fitted Sweeps' + suffix)
        plt.tight_layout()
        return fig

    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for i, fit in enumerate(fit_results):
        trace_label = _fit_trace_label(label, fit, i, n_traces)
        fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
        _plot_single_fit_trace(
            ax1, ax2, fit, format, deembed=deembed,
            phase_center=phase_center, has_errors=show_errors,
            data_label=trace_label, fit_label=fit_label,
            units=units, info=info, config=config,
            reference_plane=reference_plane,
            group_delay_cal=group_delay_cal,
            data_style=data_style_base, fit_style=fit_style_base)

    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')
    fig.suptitle(custom_title if custom_title is not None
                 else 'Fitted Sweeps' + suffix)
    plt.tight_layout()
    return fig


def plot_fit_params(fit_results, x='fr', y='Ql', color=None, fig=None,
                    label=None, colorbar=True, colorbar_label=None,
                    **kwargs):
    """Plot one fitted parameter against another.

    Args:
        fit_results: One FitResult or an iterable of FitResult objects.
        x: Attribute name for the x-axis.
        y: Attribute name for the y-axis.
        color: Either a literal Matplotlib color, a fit-parameter name to map
            to point color with a colorbar, or a numeric array with one entry
            per fit result.
        fig: Existing figure. If None, create a new one.
        label: Optional legend label.
        colorbar: When True, draw a colorbar for parameter/numeric coloring.
        colorbar_label: Optional colorbar label. Defaults to the parameter name
            when ``color`` is a fit parameter.
        **kwargs: Passed to matplotlib plot() for literal colors and to
            matplotlib scatter() for parameter-based coloring. Defaults to
            marker-only styling.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    fit_results = _coerce_fit_results(fit_results)

    x_values = _fit_parameter_values(fit_results, x)
    y_values = _fit_parameter_values(fit_results, y)
    color_values, color_name = _fit_param_color_values(fit_results, color)

    if fig is None:
        fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    else:
        ax = fig.gca()

    if color_values is None:
        kwargs.setdefault('linestyle', 'None')
        kwargs.setdefault('marker', 'o')
        if color is not None:
            kwargs.setdefault('color', color)
        ax.plot(x_values, y_values, label=label, **kwargs)
    else:
        scatter_kwargs = dict(kwargs)
        scatter_kwargs.pop('linestyle', None)
        scatter_kwargs.setdefault('marker', 'o')
        artist = ax.scatter(x_values, y_values, c=color_values,
                            label=label, **scatter_kwargs)
        if colorbar:
            fig.colorbar(
                artist, ax=ax,
                label=colorbar_label or color_name,
            )
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    if label is not None:
        ax.legend(fontsize='small')
    plt.tight_layout()
    return fig


# Convenience wrappers

def plot_sweep_iq(sweep_data, **kwargs):
    """Plot sweep as I vs Q in the complex plane. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='iq', **kwargs)


def plot_sweep_magphase(sweep_data, **kwargs):
    """Plot sweep as magnitude/phase vs frequency. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='magphase', **kwargs)


def plot_sweep_iq_vs_f(sweep_data, **kwargs):
    """Plot sweep as I and Q vs frequency. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='iq_vs_f', **kwargs)
