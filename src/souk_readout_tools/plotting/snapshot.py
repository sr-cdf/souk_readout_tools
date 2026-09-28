"""
Snapshot data plotting functions.

Handles single-tone snapshots (multiple repetitions) and batch snapshots
(multiple tones). Supports time-domain, complex plane, and PSD views with
options for per-repetition, averaged, and concatenated display.

Snapshot layout is always one axis group per tone (grid), never multi-tone
overlay on shared axes.
"""

import numpy as np
from ._common import (_get_pyplot, _compute_mag_phase, _dual_panels,
                       ERRORBAR_STYLE, _resolve_label, _legend,
                       _apply_deembed, _apply_phase_center,
                       _normalise_iq, _compute_mag_phase_units, UNITS)
from ._psd import check_psd_units, psd_display_values
from ..timeseries import (FREQ_DISS_FORMATS, _linewidth_scale, carrier_power,
                          compute_psd, compute_psd_averaged,
                          compute_psd_concatenated)


def _snapshot_acc_len(snapshot_data, sweep_data):
    """Resolve acc_len for matching pre-accumulation snapshots to sweep I/Q."""
    info = (sweep_data.get('info') if isinstance(sweep_data, dict) else None) \
           or snapshot_data.get('info')
    if isinstance(info, dict):
        acc_len = info.get('pipeline', {}).get('acc_len')
        if acc_len is not None:
            return acc_len
    raise ValueError(
        "Cannot determine acc_len for freq_diss scaling. "
        "Provide sweep_data with 'info.pipeline.acc_len' set.")


def _single_tone_snapshot(snapshot_data, func_name):
    """
    Return the single-tone snapshot dict these plots expect.

    ``batch_snapshots()`` returns one snapshot dict per tone under
    ``'results'``, so passing the batch dict (or its ``'results'`` mapping)
    here is an easy slip.  Unwrap it when it holds a single tone, and
    otherwise say which tone to pick.
    """
    if 'snapshots' in snapshot_data:
        return snapshot_data

    results = snapshot_data.get('results', snapshot_data)
    tones = sorted(tidx for tidx, snap in results.items()
                   if isinstance(snap, dict) and 'snapshots' in snap) \
        if isinstance(results, dict) else []
    if len(tones) == 1:
        return results[tones[0]]
    if tones:
        raise ValueError(
            f"{func_name}() plots one tone, but was given batch snapshot data "
            f"for tones {tones}. Pass a single tone, e.g. "
            f"batch_data['results'][{tones[0]}], or use "
            "plot_batch_snapshots() for all of them.")
    raise KeyError(
        "'snapshots' — expected snapshot data from "
        "get_accumulator_snapshots() or batch_snapshots().")


def _plot_snapshots_freq_diss(snapshot_data, snapshots, sample_rate, tone_index,
                               n_snap, *, repetitions, sweep_data,
                               reference_tone_frequency, smooth_window_hz,
                               conversion_method, calibrations,
                               deembed, phase_center, units, fig, label,
                               sharex=False, sharey=False, linewidths=False,
                               **kwargs):
    """Time-domain fractional frequency/dissipation panel for snapshots."""
    if sweep_data is None:
        raise ValueError("sweep_data is required for format='freq_diss'")
    if units != 'raw':
        raise ValueError(
            "format='freq_diss' produces dimensionless quantities; "
            "use units='raw'.")
    if deembed or phase_center:
        raise ValueError(
            "deembed/phase_center are not applicable to format='freq_diss'.")

    from .timestream import _compute_freq_diss, _dissipation_axis_label
    acc_len = _snapshot_acc_len(snapshot_data, sweep_data)
    ts_like = {'info': snapshot_data.get('info')}

    scale = _linewidth_scale(calibrations, tone_index) if linewidths else 1.0

    def _ff_fd(z):
        """Frequency/dissipation (fractional, or in linewidths) for trace ``z``."""
        ff, fd = _compute_freq_diss(
            ts_like, tone_index, z.real, z.imag, sweep_data,
            reference_tone_frequency=reference_tone_frequency,
            smooth_window_hz=smooth_window_hz,
            conversion_method=conversion_method,
            calibrations=calibrations)
        return scale * ff, scale * fd

    # Snapshots are pre-accumulation; scale to match summed sweep I/Q.
    if repetitions == 'concatenate':
        z = snapshots.ravel() * acc_len
        traces = [(_ff_fd(z), None)]
    elif repetitions == 'overlay':
        traces = []
        for i in range(n_snap):
            z = snapshots[i] * acc_len
            traces.append((_ff_fd(z), f'Rep {i}'))
    elif repetitions == 'mean':
        z = np.mean(snapshots, axis=0) * acc_len
        traces = [(_ff_fd(z), None)]
    else:
        raise ValueError(
            f"Unknown repetitions mode '{repetitions}'. "
            "Use 'concatenate', 'overlay', or 'mean'.")

    fig, ax1, ax2 = _dual_panels(fig, sharex=sharex, sharey=sharey)

    for (ff, fd), rep_label in traces:
        t = np.arange(len(ff)) / sample_rate * 1e6
        trace_label = _resolve_label(ax1, label, suffix=rep_label)
        ax1.plot(t, ff, linewidth=0.5, label=trace_label, **kwargs)
        ax2.plot(t, fd, linewidth=0.5, label=trace_label, **kwargs)

    if linewidths:
        ax1.set_ylabel('Resonator-frequency motion (linewidths)')
        ax2.set_ylabel(r'Dissipation $Q_r\,\Delta(1/2Q_i)$')
    else:
        ax1.set_ylabel('Fractional resonator-frequency motion')
        ax2.set_ylabel(_dissipation_axis_label(conversion_method))
    ax2.set_xlabel('Time (µs)')
    _legend(ax1)
    _legend(ax2)
    fig.suptitle(f'Tone {tone_index} — {n_snap} snapshots ({repetitions})')
    fig.tight_layout()
    return fig


def plot_snapshots(snapshot_data, format='iq_vs_t', repetitions='concatenate',
                   deembed=False, phase_center=False,
                   sweep_data=None, reference_tone_frequency=None,
                   fig=None, label=None,
                   units='raw', config=None, system_info=None,
                   unwrap_phase=True, smooth_window_hz=1000,
                   conversion_method='linearized', calibrations=None,
                   sharex=True, sharey=False,
                   **kwargs):
    """
    Plot snapshot data for a single tone.

    Args:
        snapshot_data: dict from get_accumulator_snapshots() with keys
            'snapshots' (N_snap, 1024), 'sample_rate', 'tone_index'.
            A single-tone batch_snapshots() dict is also accepted; for
            several tones pass one of them, e.g.
            ``batch_data['results'][3]``.
        format: 'iq' | 'iq_vs_t' | 'magphase' | 'freq_diss' | 'linewidths'
            (``'freq_diss'`` times Qr from ``calibrations``; see
            ``plot_timestream``).
        repetitions: How to handle multiple snapshots:
            'concatenate': join all end-to-end
            'overlay': plot each snapshot as a separate trace
            'mean': plot mean across snapshots
        deembed: bool or deembed params dict.  Applies true RF
            deembedding (baseline normalisation).
        phase_center: bool or phase-centering params dict.  Applies
            circle centering and rotation.  Applied after deembedding
            when both are set.
        sweep_data: Sweep data dict. Required for ``format='freq_diss'``.
        reference_tone_frequency: Tone frequency for the freq/diss
            calculation (``format='freq_diss'``).  ``None`` (default) uses
            ``snapshot_data['info']['tones']['frequencies_hz'][tone_index]``.
            Pass a scalar to override.
        smooth_window_hz: Smoothing window passed to
            the linearized resonator conversion for ``format='freq_diss'``.
            Pass ``None`` or ``0`` to disable sweep smoothing.
        conversion_method: ``'linearized'`` (default), ``'mobius'``, or
            ``'circle'`` for ``format='freq_diss'``.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator codes, no normalisation.
            'peak' - normalise to the peak magnitude of the data.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs' - dB relative to ADC full-scale.
            'dbm' - estimated ADC input power in dBm.
            All options except 'raw' and 'peak' require system_info.
            Ignored for ``format='freq_diss'`` (must be 'raw').
        config: Config dict (needed for 'dbm' and non-default rx_mix_scale).
        system_info: structured info dict.  Looked up from
            snapshot_data['info'] if not provided.
        unwrap_phase: bool, optional.  Unwrap the phase in
            ``format='magphase'``.  Default ``True`` preserves the previous
            behaviour; pass ``False`` to show wrapped phase.
        sharex, sharey: bool, optional. Tie the two panels' x / y limits
            together in the dual-panel formats. ``sharex`` defaults to
            ``True`` (both panels are the same time axis); ``sharey``
            defaults to ``False`` and is most useful for ``'iq_vs_t'`` and
            ``'freq_diss'``, whose panels share units. Ignored by
            ``format='iq'`` (a single panel). Plotting onto an existing
            ``fig`` can add sharing but cannot remove sharing that figure
            was created with.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    snapshot_data = _single_tone_snapshot(snapshot_data, 'plot_snapshots')
    snapshots = snapshot_data['snapshots']
    sample_rate = snapshot_data['sample_rate']
    tone_index = snapshot_data['tone_index']
    n_snap, n_samples = snapshots.shape
    info = system_info or snapshot_data.get('info')
    calibration_cache = {}

    if format in FREQ_DISS_FORMATS:
        return _plot_snapshots_freq_diss(
            snapshot_data, snapshots, sample_rate, tone_index, n_snap,
            repetitions=repetitions, sweep_data=sweep_data,
            reference_tone_frequency=reference_tone_frequency,
            smooth_window_hz=smooth_window_hz,
            conversion_method=conversion_method,
            calibrations=calibrations,
            deembed=deembed, phase_center=phase_center,
            units=units, fig=fig, label=label,
            sharex=sharex, sharey=sharey,
            linewidths=(format == 'linewidths'), **kwargs)

    # Normalise I/Q label setup
    if units != 'raw':
        if units != 'peak' and info is None:
            raise ValueError("system_info (or snapshot_data['info']) "
                             "is required for non-raw units.")
        # Probe labels from a dummy call
        _, _, _, _, iq_label, mag_label = _normalise_iq(
            np.zeros(1), np.zeros(1), units, info, config=config,
            pre_accumulation=True, calibration_cache=calibration_cache)
    else:
        iq_label = ''
        mag_label = '|RX| (dB)'

    # Prepare data based on repetitions mode
    if repetitions == 'concatenate':
        z_list = [snapshots.ravel()]
        labels = [None]
    elif repetitions == 'overlay':
        z_list = [snapshots[i] for i in range(n_snap)]
        labels = [f'Rep {i}' for i in range(n_snap)]
    elif repetitions == 'mean':
        z_list = [np.mean(snapshots, axis=0)]
        labels = [None]
    else:
        raise ValueError(
            f"Unknown repetitions mode '{repetitions}'. "
            "Use 'concatenate', 'overlay', or 'mean'.")

    if deembed:
        z_list = [_apply_deembed(None, z, deembed)[0] for z in z_list]
    if phase_center:
        z_list = [_apply_phase_center(z, phase_center)[0] for z in z_list]

    # Apply normalisation to complex arrays
    if units != 'raw':
        normalised = []
        for z in z_list:
            ni, nq, _, _, _, _ = _normalise_iq(
                z.real.copy(), z.imag.copy(), units, info, config=config,
                pre_accumulation=True, calibration_cache=calibration_cache)
            normalised.append(ni + 1j * nq)
        z_list = normalised

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for z, rep_label in zip(z_list, labels):
            trace_label = _resolve_label(ax, label,
                                         suffix=rep_label)
            ax.plot(z.real, z.imag, '.', markersize=1, label=trace_label, **kwargs)
        ax.set_xlabel(f'I {iq_label}'.strip())
        ax.set_ylabel(f'Q {iq_label}'.strip())
        ax.set_aspect('equal', adjustable='datalim')
        _legend(ax)
        ax.set_title(f'Tone {tone_index} — I vs Q')
        fig.tight_layout()
        return fig

    # Dual-axis formats
    fig, ax1, ax2 = _dual_panels(fig, sharex=sharex, sharey=sharey)

    for z, rep_label in zip(z_list, labels):
        n = len(z)
        t = np.arange(n) / sample_rate * 1e6  # microseconds
        trace_label = _resolve_label(ax1, label, suffix=rep_label)

        if format == 'iq_vs_t':
            ax1.plot(t, z.real, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(t, z.imag, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(f'I {iq_label}'.strip())
            ax2.set_ylabel(f'Q {iq_label}'.strip())
            ax2.set_xlabel('Time (µs)')

        elif format == 'magphase':
            mag_db, phase = _compute_mag_phase_units(
                z, units, info=info, config=config, unwrap=unwrap_phase,
                calibration_cache=calibration_cache)
            ax1.plot(t, mag_db, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(t, phase, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(mag_label)
            ax2.set_ylabel('Phase (rad)')
            ax2.set_xlabel('Time (µs)')

        else:
            raise ValueError(
                f"Unknown format '{format}'. Use 'iq', 'iq_vs_t', or 'magphase'.")

    _legend(ax1)
    _legend(ax2)
    fig.suptitle(f'Tone {tone_index} — {n_snap} snapshots ({repetitions})')
    fig.tight_layout()
    return fig


def plot_snapshots_psd(snapshot_data, format='iq', method='averaged',
                       sweep_data=None, reference_tone_frequency=None,
                       psd_kwargs=None,
                       show_errors=True, fig=None, label=None,
                       smooth_window_hz=1000,
                       conversion_method='linearized', calibrations=None,
                       psd_units='linear',
                       sharex=True, sharey=None,
                       **kwargs):
    """
    Plot PSD of snapshot data.

    The compute half lives in ``souk_readout_tools.timeseries`` as
    ``snapshot_psds()``, which also reports the true spectral resolution, the
    bin oversampling and the block-to-block drift. See
    ``doc/timeseries_analysis.md``.

    Args:
        snapshot_data: dict from get_accumulator_snapshots(). A single-tone
            batch_snapshots() dict is also accepted; for several tones pass
            one of them, e.g. ``batch_data['results'][3]``.
        format: 'iq' | 'magphase' | 'freq_diss' | 'linewidths'
            Selects which quantity to compute PSD of.  Two panels:
            I/Q, magnitude/phase, or fractional frequency/dissipation.
        method: 'averaged' (per-rep PSD then mean, with error bars from
                std dev) or 'concatenated' (concat reps, single PSD).
        sweep_data: Required for 'freq_diss' format.
        reference_tone_frequency: Tone frequency for the freq/diss
            calculation (``format='freq_diss'``).  ``None`` (default) uses
            ``snapshot_data['info']['tones']['frequencies_hz'][tone_index]``.
            Pass a scalar to override.
        smooth_window_hz: Smoothing window passed to
            the linearized resonator conversion for ``format='freq_diss'``.
            Pass ``None`` or ``0`` to disable sweep smoothing.
        conversion_method: ``'linearized'`` (default), ``'mobius'``, or
            ``'circle'`` for ``format='freq_diss'``.
        calibrations: Per-tone resonator calibrations or fit results. Required
            by ``conversion_method='mobius'`` and ``'circle'``.
        psd_kwargs: dict passed to compute_psd().
        show_errors: bool. For 'averaged', show std-dev error bars. In dB the
            band is asymmetric, and both edges are converted.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        psd_units: {'linear', 'db', 'dbc'}, optional. Display units, as in
            ``plot_timestream_psd()``: ``'dbc'`` gives carrier-referenced
            dBc/Hz in the IEEE single-sideband convention. Because that
            divides out the raw units, snapshot and accumulator-timestream
            spectra can be overlaid on one figure in dBc/Hz despite the
            pre-accumulation scale difference. Not available for
            ``format='freq_diss'`` -- use ``'db'``.
        sharex: bool, optional. Tie the two panels' x limits together.
            Default ``True``.
        sharey: bool or None, optional. Tie the two panels' y limits
            together, as in ``plot_timestream_psd()``: ``None`` (default) is
            on for ``'freq_diss'`` and off for ``'iq'`` / ``'magphase'``.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    snapshot_data = _single_tone_snapshot(snapshot_data, 'plot_snapshots_psd')
    snapshots = snapshot_data['snapshots']
    sample_rate = snapshot_data['sample_rate']
    tone_index = snapshot_data['tone_index']
    psd_kw = psd_kwargs or {}
    n_snap = snapshots.shape[0]

    if format in FREQ_DISS_FORMATS and sweep_data is None:
        raise ValueError(f"sweep_data is required for format={format!r}")
    check_psd_units(psd_units, format)

    if format == 'iq':
        data_top = snapshots.real
        data_bot = snapshots.imag
    elif format == 'magphase':
        data_top = np.abs(snapshots)
        data_bot = np.unwrap(np.angle(snapshots), axis=-1)
    elif format in FREQ_DISS_FORMATS:
        from .timestream import _compute_freq_diss
        # Snapshots are pre-accumulation; sweep I/Q are sums over acc_len
        # samples. Scale up so both are in the same units.
        acc_len = _snapshot_acc_len(snapshot_data, sweep_data)
        scaled = snapshots * acc_len

        ts_like = {'info': snapshot_data.get('info')}
        frac_f_list, frac_d_list = [], []
        for i in range(n_snap):
            ff, fd = _compute_freq_diss(
                ts_like, tone_index,
                scaled[i].real, scaled[i].imag, sweep_data,
                reference_tone_frequency=reference_tone_frequency,
                smooth_window_hz=smooth_window_hz,
                conversion_method=conversion_method,
                calibrations=calibrations)
            frac_f_list.append(ff)
            frac_d_list.append(fd)
        scale = (_linewidth_scale(calibrations, tone_index)
                 if format == 'linewidths' else 1.0)
        data_top = scale * np.array(frac_f_list)
        data_bot = scale * np.array(frac_d_list)
    else:
        raise ValueError(
            f"Unknown format '{format}'. "
            "Use 'iq', 'magphase', 'freq_diss' or 'linewidths'.")

    if sharey is None:
        # The freq/diss panels are both fractional, so one scale compares them
        # directly; I/Q and magnitude/phase panels are not in the same units.
        sharey = (format in FREQ_DISS_FORMATS)
    fig, ax1, ax2 = _dual_panels(fig, sharex=sharex, sharey=sharey)

    trace_label = _resolve_label(ax1, label)
    # One carrier for the whole capture; the phase and freq/diss panels are
    # already carrier-relative.
    carrier = carrier_power(snapshots) if psd_units == 'dbc' else 1.0
    carrier_bot = carrier if format == 'iq' else 1.0

    def _plot_panel(ax, data2d, panel_carrier):
        """Draw one PSD panel from a (snapshots x samples) array on ``ax``."""
        if method == 'averaged':
            f, p_mean, p_std = compute_psd_averaged(
                data2d, sample_rate, **psd_kw)
            values = psd_display_values(p_mean, psd_units, panel_carrier)
            if show_errors and p_std is not None:
                if psd_units == 'linear':
                    yerr = p_std
                else:
                    # A symmetric linear band is asymmetric in dB, so convert
                    # each edge and give errorbar the two halves.
                    low = psd_display_values(
                        np.maximum(p_mean - p_std, np.finfo(float).tiny),
                        psd_units, panel_carrier)
                    high = psd_display_values(
                        p_mean + p_std, psd_units, panel_carrier)
                    yerr = np.vstack([values - low, high - values])
                ax.errorbar(f, values, yerr=yerr, linewidth=0.5,
                            label=trace_label,
                            **{**ERRORBAR_STYLE, **kwargs})
            else:
                ax.plot(f, values, linewidth=0.5,
                        label=trace_label, **kwargs)
        elif method == 'concatenated':
            f, p = compute_psd_concatenated(
                data2d, sample_rate, **psd_kw)
            ax.plot(f, psd_display_values(p, psd_units, panel_carrier),
                    linewidth=0.5, label=trace_label, **kwargs)
        else:
            raise ValueError(
                f"Unknown method '{method}'. "
                "Use 'averaged' or 'concatenated'.")
        ax.set_xscale('log')
        ax.set_yscale('log' if psd_units == 'linear' else 'linear')

    _plot_panel(ax1, data_top, carrier)
    _plot_panel(ax2, data_bot, carrier_bot)

    from .timestream import _psd_axis_labels
    y_top, y_bot = _psd_axis_labels(format, psd_units, conversion_method)
    ax1.set_ylabel(y_top)
    ax2.set_ylabel(y_bot)
    ax2.set_xlabel('Frequency (Hz)')
    _legend(ax1)
    _legend(ax2)
    title_method = 'Averaged' if method == 'averaged' else 'Concatenated'
    fig.suptitle(
        f'Tone {tone_index} — {title_method} PSD ({n_snap} reps)')
    fig.tight_layout()
    return fig


def plot_batch_snapshots(batch_data, format='iq_vs_t', repetitions='concatenate',
                         psd=False, psd_method='averaged', psd_kwargs=None,
                         fig=None, label=None, **kwargs):
    """
    Plot batch snapshot data (multiple tones). One subplot row per tone.

    Args:
        batch_data: dict from batch_snapshots() with keys 'results'
            (dict tone_idx -> snapshot dict), 'sample_rate',
            'tone_frequencies'.
        format: 'iq' | 'iq_vs_t' | 'magphase'
        repetitions: 'concatenate' | 'overlay' | 'mean'
        psd: If True, add a PSD subplot column.
        psd_method: 'averaged' or 'concatenated'.
        psd_kwargs: dict passed to compute_psd().
        fig: Existing figure.
        label: Optional legend label for the plotted traces (auto if None).
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    results = batch_data['results']
    freqs = batch_data['tone_frequencies']
    n_tones = len(results)
    psd_kw = psd_kwargs or {}

    n_cols = 2 if psd else 1
    if format == 'iq':
        # IQ is single axis; PSD also single axis
        if fig is None:
            fig, axes = plt.subplots(n_tones, n_cols,
                                     figsize=(6 * n_cols, 3 * n_tones),
                                     squeeze=False)
        else:
            axes = np.array(fig.axes).reshape(n_tones, n_cols)

        for row, (tidx, snap) in enumerate(sorted(results.items())):
            snapshots = snap['snapshots']
            sample_rate = snap['sample_rate']

            # Time-domain / IQ axis
            ax = axes[row, 0]
            if repetitions == 'concatenate':
                z = snapshots.ravel()
            elif repetitions == 'mean':
                z = np.mean(snapshots, axis=0)
            else:
                z = snapshots[0]  # first rep for IQ scatter

            ax.plot(z.real, z.imag, '.', markersize=1,
                    label=_resolve_label(ax, label), **kwargs)
            ax.set_aspect('equal', adjustable='datalim')
            ax.set_ylabel(f'Tone {tidx}\n({freqs[tidx]/1e6:.3f} MHz)')
            _legend(ax)
            if row == n_tones - 1:
                ax.set_xlabel('I')

            # PSD column
            if psd:
                ax_psd = axes[row, 1]
                if psd_method == 'averaged':
                    f, psd_mean, psd_std = compute_psd_averaged(
                        snapshots, sample_rate, **psd_kw)
                    ax_psd.plot(f, 10 * np.log10(psd_mean), linewidth=0.5)
                else:
                    f, p = compute_psd_concatenated(
                        snapshots, sample_rate, **psd_kw)
                    ax_psd.plot(f, 10 * np.log10(p), linewidth=0.5)
                ax_psd.set_xscale('log')
                ax_psd.set_ylabel('PSD (dB/Hz)')
                if row == n_tones - 1:
                    ax_psd.set_xlabel('Frequency (Hz)')

    else:
        # Dual-axis formats: 2 columns for time-domain, optionally +1 for PSD
        td_cols = 2
        total_cols = td_cols + (1 if psd else 0)
        if fig is None:
            fig, axes = plt.subplots(n_tones, total_cols,
                                     figsize=(4 * total_cols, 3 * n_tones),
                                     squeeze=False)
        else:
            axes = np.array(fig.axes).reshape(n_tones, total_cols)

        for row, (tidx, snap) in enumerate(sorted(results.items())):
            snapshots = snap['snapshots']
            sample_rate = snap['sample_rate']
            n_snap, n_samp = snapshots.shape

            ax1, ax2 = axes[row, 0], axes[row, 1]

            # Prepare data
            if repetitions == 'concatenate':
                z = snapshots.ravel()
            elif repetitions == 'mean':
                z = np.mean(snapshots, axis=0)
            elif repetitions == 'overlay':
                z = snapshots[0]  # first rep; overlay handled below

            t = np.arange(len(z)) / sample_rate * 1e6

            if format == 'iq_vs_t':
                if repetitions == 'overlay':
                    for s in range(n_snap):
                        ts = np.arange(n_samp) / sample_rate * 1e6
                        rep_label = _resolve_label(ax1, label, suffix=f'Rep {s}')
                        ax1.plot(ts, snapshots[s].real, linewidth=0.3, alpha=0.5, label=rep_label)
                        ax2.plot(ts, snapshots[s].imag, linewidth=0.3, alpha=0.5, label=rep_label)
                else:
                    trace_label = _resolve_label(ax1, label)
                    ax1.plot(t, z.real, linewidth=0.5, label=trace_label, **kwargs)
                    ax2.plot(t, z.imag, linewidth=0.5, label=trace_label, **kwargs)
                ax1.set_ylabel('I')
                ax2.set_ylabel('Q')

            elif format == 'magphase':
                if repetitions == 'overlay':
                    for s in range(n_snap):
                        mag_db, phase = _compute_mag_phase(snapshots[s])
                        ts = np.arange(n_samp) / sample_rate * 1e6
                        rep_label = _resolve_label(ax1, label, suffix=f'Rep {s}')
                        ax1.plot(ts, mag_db, linewidth=0.3, alpha=0.5, label=rep_label)
                        ax2.plot(ts, phase, linewidth=0.3, alpha=0.5, label=rep_label)
                else:
                    trace_label = _resolve_label(ax1, label)
                    mag_db, phase = _compute_mag_phase(z)
                    ax1.plot(t, mag_db, linewidth=0.5, label=trace_label, **kwargs)
                    ax2.plot(t, phase, linewidth=0.5, label=trace_label, **kwargs)
                ax1.set_ylabel('|RX| (dB)')
                ax2.set_ylabel('Phase (rad)')

            # Row label + legends
            ax1.set_title(f'Tone {tidx} ({freqs[tidx]/1e6:.3f} MHz)',
                          fontsize='small')
            _legend(ax1)
            _legend(ax2)
            if row == n_tones - 1:
                ax1.set_xlabel('Time (µs)')
                ax2.set_xlabel('Time (µs)')

            # PSD column
            if psd:
                ax_psd = axes[row, td_cols]
                if psd_method == 'averaged':
                    f, psd_mean, psd_std = compute_psd_averaged(
                        snapshots, sample_rate, **psd_kw)
                    psd_db = 10 * np.log10(psd_mean)
                    ax_psd.plot(f, psd_db, linewidth=0.5)
                else:
                    f, p = compute_psd_concatenated(
                        snapshots, sample_rate, **psd_kw)
                    ax_psd.plot(f, 10 * np.log10(p), linewidth=0.5)
                ax_psd.set_xscale('log')
                ax_psd.set_ylabel('PSD (dB/Hz)')
                if row == n_tones - 1:
                    ax_psd.set_xlabel('Frequency (Hz)')

    fig.suptitle(f'Batch Snapshots — {n_tones} tones, {repetitions}')
    fig.tight_layout()
    return fig
