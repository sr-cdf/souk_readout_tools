"""
Timestream data plotting functions.

Supports parsed timestream data (from ReadoutClient.parse_samples()) in
multiple formats: I/Q, magnitude/phase, frequency/dissipation, and their
PSDs. Includes a debugging overlay of timestream points on the resonance
circle from sweep data.
"""

import numpy as np
from ._common import (_get_pyplot, _compute_mag_phase, _apply_deembedding,
                       ERRORBAR_STYLE, _resolve_label,
                       _normalise_iq, _compute_mag_phase_units, UNITS)
from ._psd import compute_psd

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


def _compute_freq_diss(ts_data, tone_key, i_arr, q_arr, sweep_data):
    """
    Compute fractional frequency and dissipation from timestream + sweep.

    Uses ReadoutClient.calculate_frequency_and_dissipation_noise as a
    static method.
    """
    from souk_readout_tools.client.readout_client import ReadoutClient

    z_ts = i_arr + 1j * q_arr
    tone_idx = int(tone_key)

    # Extract per-tone sweep data
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])

    is_wideband = sweep_data.get('wideband_sweep', False)
    if is_wideband or sf.shape[0] == 1:
        sweep_f = sf[0]
        sweep_z = si[0] + 1j * sq[0]
    else:
        # Per-tone: column index
        n_pts, n_tones = sf.shape
        if tone_idx >= n_tones:
            raise ValueError(
                f"Tone {tone_idx} not in sweep data ({n_tones} tones)")
        sweep_f = sf[:, tone_idx]
        sweep_z = si[:, tone_idx] + 1j * sq[:, tone_idx]

    # Tone frequency: middle of sweep range for this tone
    tone_freq = np.mean(sweep_f)

    frac_f, frac_d, *_ = ReadoutClient.calculate_frequency_and_dissipation_noise(
        sweep_f, sweep_z, tone_freq, z_ts)

    return frac_f, frac_d


def plot_timestream(ts_data, format='iq_vs_t', tones=None,
                    deembed=False, sweep_data=None, fig=None, label=None,
                    units='raw', config=None, reference_plane='adc_input',
                    x_axis='time', **kwargs):
    """
    Plot timestream data in various formats.

    Args:
        ts_data: dict from parse_samples() with 'i_data', 'q_data',
            'sample_rate', 'num_samples'.
        format: 'iq' | 'iq_vs_t' | 'magphase' | 'freq_diss'
        tones: List of tone indices (int) to plot, or None for [0].
        deembed: bool or deembed params dict. For 'iq' format, applies
            deembedding. Requires sweep_data.
        sweep_data: Sweep data dict. Required for 'freq_diss' format
            and for deembed=True.
        fig: Existing figure. If None, create new.
        label: Legend label. If None, uses an auto-incrementing index.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator codes, no normalisation.
            'peak' - normalise to the peak magnitude of the data.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs' - dB relative to ADC full-scale.
            'dbm' - estimated power in dBm at ``reference_plane``.
            All options except 'raw' and 'peak' require
            'system_information' in ts_data.
        config: Config dict (needed for 'dbm' and non-default rx_mix_scale).
        reference_plane: Reference plane used when ``units='dbm'``.  One of
            'adc_input' (default), 'cryostat_output', or 'detector'.
            'cryostat_output'/'detector' deembed the RX analog chain using
            the calibration entries in ``config['rf_frontend']`` and
            ``config['cryostat']`` at each tone's frequency; falls back to
            'adc_input' with a warning if that cal is not available.
        x_axis: X-axis for time-domain formats.  One of:
            'time' (default) - seconds from sample rate.
            'sample' - sample index (0, 1, 2, ...).
            'acc_count' - accumulation counter (packet_counter).
            'telescope_time' - PTP telescope time in seconds.
        **kwargs: Passed to matplotlib plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones)
    sample_rate = ts_data['sample_rate']
    x_values, x_label = _build_x_axis(ts_data, x_axis)
    info = ts_data.get('system_information')

    # Look up each selected tone's RF frequency for cal resolution.
    tone_freqs = None
    if isinstance(info, dict):
        tf = info.get('tone_frequencies')
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
            raise ValueError("ts_data must contain 'system_information' for non-raw units.")
        normalised = []
        for key, i_arr, q_arr in selected:
            tone_f = _tone_frequency(key)
            ni, nq, _, _, iq_label, mag_label = _normalise_iq(
                i_arr, q_arr, units, info, config=config,
                reference_plane=reference_plane,
                frequencies=tone_f)
            normalised.append((key, ni, nq))
        selected = normalised
    else:
        iq_label = ''
        mag_label = '|S21| (dB)'

    if format == 'freq_diss' and sweep_data is None:
        raise ValueError("sweep_data is required for format='freq_diss'")

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for key, i_arr, q_arr in selected:
            z = i_arr + 1j * q_arr
            if deembed:
                z, _ = _apply_deembedding(None, z, deembed)
            trace_label = _resolve_label(ax, label,
                                         suffix=f'Tone {key}' if len(selected) > 1 else None)
            ax.plot(z.real, z.imag, '.', markersize=1,
                    label=trace_label, **kwargs)
        ax.set_xlabel(f'I {iq_label}'.strip())
        ax.set_ylabel(f'Q {iq_label}'.strip())
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend(fontsize='small')
        title = 'Timestream I vs Q'
        if deembed:
            title += ' (deembedded)'
        ax.set_title(title)
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
            if deembed:
                z, _ = _apply_deembedding(None, z, deembed)
            ax1.plot(x_values, z.real, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, z.imag, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(f'I {iq_label}'.strip())
            ax2.set_ylabel(f'Q {iq_label}'.strip())

        elif format == 'magphase':
            if deembed:
                z, _ = _apply_deembedding(None, z, deembed)
            mag_db, phase = _compute_mag_phase_units(z, units, info=info, config=config)
            ax1.plot(x_values, mag_db, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, phase, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel(mag_label)
            ax2.set_ylabel('Phase (rad)')

        elif format == 'freq_diss':
            frac_f, frac_d = _compute_freq_diss(ts_data, key, i_arr, q_arr,
                                                 sweep_data)
            ax1.plot(x_values, frac_f, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(x_values, frac_d, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel('Fractional frequency shift')
            ax2.set_ylabel('Fractional dissipation shift')

        else:
            raise ValueError(
                f"Unknown format '{format}'. "
                "Use 'iq', 'iq_vs_t', 'magphase', or 'freq_diss'.")

    ax2.set_xlabel(x_label)
    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')
    title_map = {
        'iq_vs_t': 'Timestream',
        'magphase': 'Timestream',
        'freq_diss': 'Frequency & Dissipation',
    }
    title = title_map.get(format, 'Timestream')
    if deembed and format != 'freq_diss':
        title += ' (deembedded)'
    fig.suptitle(title)
    plt.tight_layout()
    return fig


def plot_timestream_psd(ts_data, format='iq', tones=None,
                        sweep_data=None, psd_kwargs=None,
                        precomputed_psd=None, fig=None, label=None, **kwargs):
    """
    Plot power spectral density of timestream data.

    Args:
        ts_data: dict from parse_samples().
        format: 'iq' | 'magphase' | 'freq_diss'
            Selects which quantity to compute PSD of.
        tones: List of tone indices. Default: [0].
        sweep_data: Required for 'freq_diss' format.
        psd_kwargs: dict of kwargs passed to compute_psd().
        precomputed_psd: dict mapping tone_key -> (f_psd, psd_values).
            If provided, skip computation.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones)
    sample_rate = ts_data['sample_rate']
    psd_kw = psd_kwargs or {}

    if format == 'freq_diss' and sweep_data is None:
        raise ValueError("sweep_data is required for format='freq_diss'")

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
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
        else:
            ax1, ax2 = fig.axes[:2]

        for key, i_arr, q_arr in selected:
            trace_label = _resolve_label(ax1, label,
                                         suffix=f'Tone {key}' if len(selected) > 1 else None)
            frac_f, frac_d = _compute_freq_diss(ts_data, key, i_arr, q_arr,
                                                 sweep_data)
            f1, p1 = compute_psd(frac_f, sample_rate, **psd_kw)
            f2, p2 = compute_psd(frac_d, sample_rate, **psd_kw)

            ax1.loglog(f1, p1, linewidth=0.5, label=trace_label, **kwargs)
            ax2.loglog(f2, p2, linewidth=0.5, label=trace_label, **kwargs)

        ax1.set_ylabel('Frequency noise PSD')
        ax2.set_ylabel('Dissipation noise PSD')

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
                                  deembed=False, fig=None, label=None,
                                  units='raw', config=None, **kwargs):
    """
    Overplot timestream I/Q points on the resonance circle from sweep data.

    Args:
        ts_data: dict from parse_samples().
        sweep_data: dict from parse_sweep_data() (per-tone).
        tone_index: int, which tone to plot.
        deembed: bool, deembed both sweep and timestream.
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
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])

    is_wideband = sweep_data.get('wideband_sweep', False)
    if is_wideband or sf.shape[0] == 1:
        sweep_f = sf[0]
        sw_i, sw_q = si[0].copy(), sq[0].copy()
    else:
        sweep_f = sf[:, tone_index]
        sw_i, sw_q = si[:, tone_index].copy(), sq[:, tone_index].copy()

    # Normalise both sweep and timestream with the same units
    if units in ('dbfs', 'dbm'):
        raise ValueError(
            f"units='{units}' is not supported for I vs Q plots — "
            "use 'raw', 'peak', or 'adc_fs'.")
    info = ts_data.get('system_information') or sweep_data.get('system_information')
    iq_label = ''
    if units != 'raw':
        tone_f = np.mean(sweep_f) if sweep_f is not None else None
        if units != 'peak' and (info is None or not isinstance(info, dict)):
            raise ValueError("data must contain 'system_information' for non-raw units.")
        i_arr, q_arr, _, _, iq_label, _ = _normalise_iq(
            i_arr, q_arr, units, info, config=config,
            frequencies=tone_f)
        sw_i, sw_q, _, _, _, _ = _normalise_iq(
            sw_i, sw_q, units, info, config=config,
            frequencies=sweep_f)

    z_ts = i_arr + 1j * q_arr
    z_sweep = sw_i + 1j * sw_q

    deembed_params = None
    if deembed:
        z_sweep, deembed_params = _apply_deembedding(sweep_f, z_sweep, True)
        z_ts = _apply_deembedding(None, z_ts, deembed_params)[0]

    # Compute phase for the frequency-domain panel
    _, phase_sweep = _compute_mag_phase(z_sweep)
    _, phase_ts = _compute_mag_phase(z_ts)
    ts_info = ts_data.get('system_information') or {}
    ts_tone_freqs = ts_info.get('tone_frequencies')
    if ts_tone_freqs is not None:
        tone_freq = float(np.asarray(ts_tone_freqs)[tone_index])
    else:
        tone_freq = np.mean(sweep_f)

    if fig is None:
        fig, (ax_iq, ax_pf) = plt.subplots(1, 2, figsize=(14, 6))
    else:
        ax_iq, ax_pf = fig.axes[:2]

    ts_label = label if label is not None else 'Timestream'

    # Left panel: I vs Q resonance circle
    ax_iq.plot(z_sweep.real, z_sweep.imag, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_iq.plot(z_ts.real, z_ts.imag, '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_iq.set_xlabel(f'I {iq_label}'.strip())
    ax_iq.set_ylabel(f'Q {iq_label}'.strip())
    ax_iq.set_aspect('equal', adjustable='datalim')
    ax_iq.legend(fontsize='small')

    # Right panel: phase vs frequency
    ax_pf.plot(sweep_f, phase_sweep, '-', linewidth=1.5,
               color='C0', label='Sweep', zorder=2)
    ax_pf.plot(np.full_like(phase_ts, tone_freq), phase_ts,
               '.', markersize=1, alpha=0.3,
               color='C1', label=ts_label, zorder=1, **kwargs)
    ax_pf.set_xlabel('Frequency (Hz)')
    ax_pf.set_ylabel('Phase (rad)')
    ax_pf.legend(fontsize='small')

    title = f'Tone {tone_index} — Resonance Circle'
    if deembed:
        title += ' (deembedded)'
    fig.suptitle(title)
    plt.tight_layout()
    return fig
