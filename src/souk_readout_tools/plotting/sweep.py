"""
Sweep data plotting functions.

Supports wideband sweeps (single concatenated trace) and per-tone sweeps
(multiple tones). Formats: I vs Q, I/Q vs frequency, magnitude/phase vs
frequency, all with optional deembedding and error bars.
"""

import numpy as np
from ._common import (_get_pyplot, _compute_mag_phase, _propagate_errors_mag,
                       _apply_deembedding, ERROR_FILL_STYLE)


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


def plot_sweep(sweep_data, format='magphase', tones=None, deembed=False,
               show_errors=True, multi_tone='overlay', fig=None, **kwargs):
    """
    General-purpose sweep plot.

    Args:
        sweep_data: dict with keys 'sweep_f', 'sweep_i', 'sweep_q',
            optionally 'sweep_ei', 'sweep_eq'.
        format: 'iq' | 'iq_vs_f' | 'magphase'
        tones: List of tone indices to plot, or None for all.
        deembed: bool, apply deembedding (cable delay + centering + rotation).
        show_errors: bool, show error bars (line only, no caps).
        multi_tone: 'overlay' (shared axes) or 'grid' (one subplot per tone).
        fig: Existing figure. If None, create new.
        **kwargs: Passed to matplotlib plot/errorbar calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    traces = _extract_traces(sweep_data, tones)
    has_errors = show_errors and any(np.any(ei != 0) for _, _, _, ei, _, _ in traces)
    n_traces = len(traces)
    is_multi = n_traces > 1 and traces[0][5] is not None

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
                if deembed:
                    z, _ = _apply_deembedding(f, z, deembed)
                ax.plot(z.real, z.imag, linewidth=0.8, **kwargs)
                ax.set_aspect('equal')
                ax.set_title(f'Tone {tidx}')
                ax.set_xlabel('I')
                ax.set_ylabel('Q')
            # Hide unused axes
            for i in range(n_traces, len(axes_flat)):
                axes_flat[i].set_visible(False)
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
                _plot_single_trace(axes[i, 0], axes[i, 1], f, si, sq, ei, eq,
                                   format, deembed, has_errors, tidx, **kwargs)
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
            if deembed:
                z, _ = _apply_deembedding(f, z, deembed)
            label = f'Tone {tidx}' if tidx is not None else None
            ax.plot(z.real, z.imag, linewidth=0.8, label=label, **kwargs)
        ax.set_xlabel('I')
        ax.set_ylabel('Q')
        ax.set_aspect('equal')
        if is_multi:
            ax.legend(fontsize='small')
        title = 'S21 Complex Plane'
        if deembed:
            title += ' (deembedded)'
        ax.set_title(title)
        plt.tight_layout()
        return fig

    # Dual-axis formats: magphase or iq_vs_f
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for f, si, sq, ei, eq, tidx in traces:
        _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                           format, deembed, has_errors, tidx, **kwargs)

    if is_multi:
        ax1.legend(fontsize='small')

    bw = sweep_data.get('bandwidth_hz')
    title = f'Sweep ({bw/1e6:.1f} MHz)' if bw else 'Sweep'
    if deembed:
        title += ' (deembedded)'
    fig.suptitle(title)
    plt.tight_layout()
    return fig


def _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                        format, deembed, has_errors, tidx, **kwargs):
    """Plot a single trace on a pair of axes."""
    z = si + 1j * sq
    if deembed:
        z, _ = _apply_deembedding(f, z, deembed)
        si, sq = z.real, z.imag

    f_mhz = f / 1e6
    label = f'Tone {tidx}' if tidx is not None else None

    if format == 'magphase':
        mag_db, phase = _compute_mag_phase(z)
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
        ax1.set_ylabel('|S21| (dB)')
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
        ax1.set_ylabel('I')
        ax2.set_ylabel('Q')
        ax2.set_xlabel('Frequency (MHz)')

    else:
        raise ValueError(f"Unknown format '{format}'. Use 'iq', 'iq_vs_f', or 'magphase'.")


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
