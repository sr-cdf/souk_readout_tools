"""
Snapshot data plotting functions.

Handles single-tone snapshots (multiple repetitions) and batch snapshots
(multiple tones). Supports time-domain, complex plane, and PSD views with
options for per-repetition, averaged, and concatenated display.

Snapshot layout is always one axis group per tone (grid), never multi-tone
overlay on shared axes.
"""

import numpy as np
from ._common import _get_pyplot, _compute_mag_phase, ERRORBAR_STYLE, _resolve_label
from ._psd import compute_psd, compute_psd_averaged, compute_psd_concatenated


def plot_snapshots(snapshot_data, format='iq_vs_t', repetitions='concatenate',
                   deembed=False, fig=None, label=None, **kwargs):
    """
    Plot snapshot data for a single tone.

    Args:
        snapshot_data: dict from get_accumulator_snapshots() with keys
            'snapshots' (N_snap, 1024), 'sample_rate', 'tone_index'.
        format: 'iq' | 'iq_vs_t' | 'magphase'
        repetitions: How to handle multiple snapshots:
            'concatenate': join all end-to-end
            'overlay': plot each snapshot as a separate trace
            'mean': plot mean across snapshots
        deembed: bool or deembed params dict.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    snapshots = snapshot_data['snapshots']
    sample_rate = snapshot_data['sample_rate']
    tone_index = snapshot_data['tone_index']
    n_snap, n_samples = snapshots.shape

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
        from ._common import _apply_deembedding
        z_list = [_apply_deembedding(None, z, deembed)[0] for z in z_list]

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        for z, rep_label in zip(z_list, labels):
            trace_label = _resolve_label(ax, label,
                                         suffix=rep_label)
            ax.plot(z.real, z.imag, '.', markersize=1, label=trace_label, **kwargs)
        ax.set_xlabel('I')
        ax.set_ylabel('Q')
        ax.set_aspect('equal')
        ax.legend(fontsize='small')
        ax.set_title(f'Tone {tone_index} — I vs Q')
        plt.tight_layout()
        return fig

    # Dual-axis formats
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for z, rep_label in zip(z_list, labels):
        n = len(z)
        t = np.arange(n) / sample_rate * 1e6  # microseconds
        trace_label = _resolve_label(ax1, label, suffix=rep_label)

        if format == 'iq_vs_t':
            ax1.plot(t, z.real, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(t, z.imag, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel('I')
            ax2.set_ylabel('Q')
            ax2.set_xlabel('Time (µs)')

        elif format == 'magphase':
            mag_db, phase = _compute_mag_phase(z)
            ax1.plot(t, mag_db, linewidth=0.5, label=trace_label, **kwargs)
            ax2.plot(t, phase, linewidth=0.5, label=trace_label, **kwargs)
            ax1.set_ylabel('|S21| (dB)')
            ax2.set_ylabel('Phase (rad)')
            ax2.set_xlabel('Time (µs)')

        else:
            raise ValueError(
                f"Unknown format '{format}'. Use 'iq', 'iq_vs_t', or 'magphase'.")

    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')
    fig.suptitle(f'Tone {tone_index} — {n_snap} snapshots ({repetitions})')
    plt.tight_layout()
    return fig


def plot_snapshots_psd(snapshot_data, method='averaged', psd_kwargs=None,
                       show_errors=True, fig=None, label=None, **kwargs):
    """
    Plot PSD of snapshot data.

    Args:
        snapshot_data: dict from get_accumulator_snapshots().
        method: 'averaged' (per-rep PSD then mean, with error bars from
                std dev) or 'concatenated' (concat reps, single PSD).
        psd_kwargs: dict passed to compute_psd().
        show_errors: bool. For 'averaged', show std-dev error bars.
        fig: Existing figure.
        label: Legend label. If None, uses an auto-incrementing index.
        **kwargs: Passed to plot calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    snapshots = snapshot_data['snapshots']
    sample_rate = snapshot_data['sample_rate']
    tone_index = snapshot_data['tone_index']
    psd_kw = psd_kwargs or {}

    if fig is None:
        fig, ax = plt.subplots(1, 1, figsize=(10, 5))
    else:
        ax = fig.gca()

    if method == 'averaged':
        f, psd_mean, psd_std = compute_psd_averaged(
            snapshots, sample_rate, **psd_kw)
        psd_db = 10 * np.log10(psd_mean)
        trace_label = _resolve_label(ax, label)
        if show_errors and psd_std is not None:
            # Error in dB: propagate from linear
            e_db = 10 / (psd_mean * np.log(10)) * psd_std
            ax.errorbar(f, psd_db, yerr=e_db, linewidth=0.5, label=trace_label,
                        **{**ERRORBAR_STYLE, **kwargs})
        else:
            ax.plot(f, psd_db, linewidth=0.5, label=trace_label, **kwargs)
        ax.set_title(f'Tone {tone_index} — Averaged PSD '
                     f'({snapshots.shape[0]} reps)')

    elif method == 'concatenated':
        f, psd = compute_psd_concatenated(snapshots, sample_rate, **psd_kw)
        psd_db = 10 * np.log10(psd)
        trace_label = _resolve_label(ax, label)
        ax.plot(f, psd_db, linewidth=0.5, label=trace_label, **kwargs)
        ax.set_title(f'Tone {tone_index} — Concatenated PSD')

    else:
        raise ValueError(
            f"Unknown method '{method}'. Use 'averaged' or 'concatenated'.")

    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('PSD (dB/Hz)')
    ax.set_xscale('log')
    ax.legend(fontsize='small')
    plt.tight_layout()
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
            ax.set_aspect('equal')
            ax.set_ylabel(f'Tone {tidx}\n({freqs[tidx]/1e6:.3f} MHz)')
            ax.legend(fontsize='small')
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
                ax1.set_ylabel('|S21| (dB)')
                ax2.set_ylabel('Phase (rad)')

            # Row label + legends
            ax1.set_title(f'Tone {tidx} ({freqs[tidx]/1e6:.3f} MHz)',
                          fontsize='small')
            ax1.legend(fontsize='small')
            ax2.legend(fontsize='small')
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
    plt.tight_layout()
    return fig
