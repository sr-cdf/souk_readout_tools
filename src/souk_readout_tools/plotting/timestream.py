"""
Timestream data plotting functions.

Supports parsed timestream data (from ReadoutClient.parse_samples()) in
multiple formats: I/Q, magnitude/phase, frequency/dissipation, and their
PSDs. Includes a debugging overlay of timestream points on the resonance
circle from sweep data.
"""

import numpy as np
from ._common import (_get_pyplot, _compute_mag_phase, _apply_deembedding,
                       ERRORBAR_STYLE)
from ._psd import compute_psd


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
                    deembed=False, sweep_data=None, fig=None, **kwargs):
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
        **kwargs: Passed to matplotlib plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones)
    sample_rate = ts_data['sample_rate']
    n_samples = ts_data.get('num_samples', len(selected[0][1]))
    t_axis = np.arange(n_samples) / sample_rate

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
            ax.plot(z.real, z.imag, '.', markersize=1,
                    label=f'Tone {key}', **kwargs)
        ax.set_xlabel('I')
        ax.set_ylabel('Q')
        ax.set_aspect('equal')
        if len(selected) > 1:
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
        label = f'Tone {key}' if len(selected) > 1 else None
        z = i_arr + 1j * q_arr

        if format == 'iq_vs_t':
            if deembed:
                z, _ = _apply_deembedding(None, z, deembed)
            ax1.plot(t_axis, z.real, linewidth=0.5, label=label, **kwargs)
            ax2.plot(t_axis, z.imag, linewidth=0.5, label=label, **kwargs)
            ax1.set_ylabel('I')
            ax2.set_ylabel('Q')

        elif format == 'magphase':
            if deembed:
                z, _ = _apply_deembedding(None, z, deembed)
            mag_db, phase = _compute_mag_phase(z)
            ax1.plot(t_axis, mag_db, linewidth=0.5, label=label, **kwargs)
            ax2.plot(t_axis, phase, linewidth=0.5, label=label, **kwargs)
            ax1.set_ylabel('|S21| (dB)')
            ax2.set_ylabel('Phase (rad)')

        elif format == 'freq_diss':
            frac_f, frac_d = _compute_freq_diss(ts_data, key, i_arr, q_arr,
                                                 sweep_data)
            ax1.plot(t_axis, frac_f, linewidth=0.5, label=label, **kwargs)
            ax2.plot(t_axis, frac_d, linewidth=0.5, label=label, **kwargs)
            ax1.set_ylabel('Fractional frequency shift')
            ax2.set_ylabel('Fractional dissipation shift')

        else:
            raise ValueError(
                f"Unknown format '{format}'. "
                "Use 'iq', 'iq_vs_t', 'magphase', or 'freq_diss'.")

    ax2.set_xlabel('Time (s)')
    if len(selected) > 1:
        ax1.legend(fontsize='small')
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
                        precomputed_psd=None, fig=None, **kwargs):
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
            fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
        else:
            ax1, ax2 = fig.axes[:2]

        for key, i_arr, q_arr in selected:
            label = f'Tone {key}' if len(selected) > 1 else None
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

            ax1.loglog(f1, p1, linewidth=0.5, label=label, **kwargs)
            ax2.loglog(f2, p2, linewidth=0.5, label=label, **kwargs)

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
            label = f'Tone {key}' if len(selected) > 1 else None
            frac_f, frac_d = _compute_freq_diss(ts_data, key, i_arr, q_arr,
                                                 sweep_data)
            f1, p1 = compute_psd(frac_f, sample_rate, **psd_kw)
            f2, p2 = compute_psd(frac_d, sample_rate, **psd_kw)

            ax1.loglog(f1, p1, linewidth=0.5, label=label, **kwargs)
            ax2.loglog(f2, p2, linewidth=0.5, label=label, **kwargs)

        ax1.set_ylabel('Frequency noise PSD')
        ax2.set_ylabel('Dissipation noise PSD')

    else:
        raise ValueError(
            f"Unknown format '{format}'. Use 'iq', 'magphase', or 'freq_diss'.")

    ax2.set_xlabel('Frequency (Hz)')
    if len(selected) > 1:
        ax1.legend(fontsize='small')
    fig.suptitle('Power Spectral Density')
    plt.tight_layout()
    return fig


def plot_timestream_on_resonance(ts_data, sweep_data, tone_index,
                                  deembed=False, fig=None, **kwargs):
    """
    Overplot timestream I/Q points on the resonance circle from sweep data.

    Args:
        ts_data: dict from parse_samples().
        sweep_data: dict from parse_sweep_data() (per-tone).
        tone_index: int, which tone to plot.
        deembed: bool, deembed both sweep and timestream.
        fig: Existing figure.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    selected = _get_tone_data(ts_data, tones=[tone_index])
    key, i_arr, q_arr = selected[0]
    z_ts = i_arr + 1j * q_arr

    # Extract sweep trace for this tone
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])

    is_wideband = sweep_data.get('wideband_sweep', False)
    if is_wideband or sf.shape[0] == 1:
        sweep_f = sf[0]
        z_sweep = si[0] + 1j * sq[0]
    else:
        sweep_f = sf[:, tone_index]
        z_sweep = si[:, tone_index] + 1j * sq[:, tone_index]

    deembed_params = None
    if deembed:
        z_sweep, deembed_params = _apply_deembedding(sweep_f, z_sweep, True)
        z_ts = _apply_deembedding(None, z_ts, deembed_params)[0]

    if fig is None:
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))
    else:
        ax = fig.gca()

    # Sweep circle
    ax.plot(z_sweep.real, z_sweep.imag, '-', linewidth=1.5,
            color='C0', label='Sweep', zorder=2)
    # Timestream scatter
    ax.plot(z_ts.real, z_ts.imag, '.', markersize=1, alpha=0.3,
            color='C1', label='Timestream', zorder=1, **kwargs)

    ax.set_xlabel('I')
    ax.set_ylabel('Q')
    ax.set_aspect('equal')
    ax.legend()
    title = f'Tone {tone_index} — Resonance Circle'
    if deembed:
        title += ' (deembedded)'
    ax.set_title(title)
    plt.tight_layout()
    return fig
