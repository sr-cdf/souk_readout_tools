"""
Shared utilities for the plotting subpackage.
"""

import numpy as np


def _get_pyplot():
    """Lazy import of matplotlib.pyplot."""
    import matplotlib.pyplot as plt
    return plt


def _compute_mag_phase(z):
    """
    Compute log magnitude (dB) and unwrapped phase from complex S21.

    Args:
        z: Complex array.

    Returns:
        log_mag: 20*log10(|z|) in dB.
        phase: Unwrapped phase in radians.
    """
    log_mag = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z))
    return log_mag, phase


def _propagate_errors_mag(si, sq, ei, eq):
    """
    Propagate I/Q errors to log-magnitude and phase errors.

    Matches the convention used in wideband_sweep.py.

    Args:
        si, sq: I and Q signal arrays.
        ei, eq: I and Q standard error arrays.

    Returns:
        e_logmag: Error on 20*log10(|S21|) in dB.
        e_phase: Error on phase in radians.
    """
    mag_sq = si ** 2 + sq ** 2
    mag = np.sqrt(mag_sq)
    # Error on linear magnitude
    e_mag = np.sqrt((si * ei) ** 2 + (sq * eq) ** 2) / mag
    # Error on log magnitude
    e_logmag = 20 / (mag * np.log(10)) * e_mag
    # Error on phase
    e_phase = np.sqrt((sq * ei) ** 2 + (si * eq) ** 2) / mag_sq
    return e_logmag, e_phase


def _apply_deembedding(frequencies, z, deembed, params=None):
    """
    Apply deembedding if requested.

    Args:
        frequencies: 1D frequency array (Hz).
        z: 1D complex S21 array.
        deembed: bool or dict. If True, compute deembedding. If dict,
                 use as pre-computed params.
        params: Pre-computed deembed params (alternative to passing as
                deembed argument). If provided, applies these params.

    Returns:
        z_out: Processed complex array.
        deembed_params: dict of deembedding parameters, or None.
    """
    if params is not None:
        from .. import resonator
        return resonator.apply_deembed_params(z, params), params

    if deembed is True:
        from .. import resonator
        return resonator.deembed(frequencies, z)
    elif isinstance(deembed, dict):
        from .. import resonator
        return resonator.apply_deembed_params(z, deembed), deembed

    return z, None


# Standard error bar style (legacy, kept for non-sweep plots)
ERRORBAR_STYLE = dict(fmt='.', capsize=0, ecolor='red', markersize=2)

# Fill style for fast error-band rendering on large traces
ERROR_FILL_STYLE = dict(alpha=0.25)


def _resolve_label(ax, label=None, suffix=None):
    """Resolve the label for a plot line.

    If label and suffix both given, returns '{label} {suffix}'.
    If only label, returns label.
    If only suffix, returns str(suffix).
    If neither, returns the next integer index based on existing lines on ax.
    """
    if label is not None:
        return f'{label} {suffix}' if suffix else label
    if suffix is not None:
        return str(suffix)
    return str(len(ax.get_lines()))
