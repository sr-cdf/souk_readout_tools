"""
Resonator data transforms for MKID S21 analysis.

Provides deembedding operations (cable delay removal, circle centering,
rotation) as pure numerical transforms on complex S21 data.

These transforms are used by the plotting library when deembed=True,
but can also be used independently for resonator characterisation.
"""

import numpy as np


def remove_cable_delay(frequencies, s21, tau=None):
    """
    Remove electrical delay (cable delay) from S21 data.

    Multiplies S21 by exp(+j * 2 * pi * f * tau) to unwind the linear
    phase slope caused by cable length.

    Args:
        frequencies: 1D array of frequencies in Hz.
        s21: 1D complex array of S21 values.
        tau: Cable delay in seconds. If None, estimated from the
             median gradient of the unwrapped phase.

    Returns:
        s21_corrected: Complex array with cable delay removed.
        tau: The delay that was removed (seconds).
    """
    frequencies = np.asarray(frequencies, dtype=float)
    s21 = np.asarray(s21, dtype=complex)

    if tau is None:
        phase = np.unwrap(np.angle(s21))
        # Median of phase gradient is robust to resonance dips
        dphase_df = np.gradient(phase, frequencies)
        tau = -np.median(dphase_df) / (2 * np.pi)

    s21_corrected = s21 * np.exp(1j * 2 * np.pi * frequencies * tau)
    return s21_corrected, tau


def center_circle(s21):
    """
    Translate the resonance circle so its algebraic center is at the origin.

    Uses the Kasa algebraic circle fit to find the center and radius
    of the resonance circle in the complex plane.

    Args:
        s21: 1D complex array (cable delay should be removed first).

    Returns:
        s21_centered: Complex array with center subtracted.
        center: Complex number, the fitted center.
        radius: Float, the fitted radius.
    """
    s21 = np.asarray(s21, dtype=complex)
    x = s21.real
    y = s21.imag

    # Kasa algebraic circle fit
    # Minimises sum of (x^2 + y^2 - 2*xc*x - 2*yc*y + xc^2 + yc^2 - R^2)^2
    n = len(x)
    sum_x = np.sum(x)
    sum_y = np.sum(y)
    sum_x2 = np.sum(x ** 2)
    sum_y2 = np.sum(y ** 2)
    sum_xy = np.sum(x * y)
    sum_x3 = np.sum(x ** 3)
    sum_y3 = np.sum(y ** 3)
    sum_x2y = np.sum(x ** 2 * y)
    sum_xy2 = np.sum(x * y ** 2)

    A = np.array([
        [sum_x2, sum_xy, sum_x],
        [sum_xy, sum_y2, sum_y],
        [sum_x, sum_y, n]
    ])
    b = np.array([
        sum_x3 + sum_xy2,
        sum_x2y + sum_y3,
        sum_x2 + sum_y2
    ])

    try:
        params = np.linalg.solve(A, b)
    except np.linalg.LinAlgError:
        # Fallback: use mean as center estimate
        center = np.mean(s21)
        radius = np.mean(np.abs(s21 - center))
        return s21 - center, center, radius

    xc = params[0] / 2
    yc = params[1] / 2
    radius = np.sqrt(params[2] + xc ** 2 + yc ** 2)
    center = xc + 1j * yc

    s21_centered = s21 - center
    return s21_centered, center, radius


def rotate_to_real_axis(s21, s21_at_resonance=None):
    """
    Rotate S21 so the resonance point lies on the negative real axis.

    Args:
        s21: 1D complex array (should already be centered).
        s21_at_resonance: Complex value at resonance. If None, uses
                          the point with minimum distance from the origin
                          (deepest dip on the centered circle).

    Returns:
        s21_rotated: Complex array.
        angle: Rotation angle applied (radians).
    """
    s21 = np.asarray(s21, dtype=complex)

    if s21_at_resonance is None:
        # Minimum magnitude point on centered circle = resonance
        idx = np.argmin(np.abs(s21))
        s21_at_resonance = s21[idx]

    # Rotate so this point is on the negative real axis
    angle = np.pi - np.angle(s21_at_resonance)
    s21_rotated = s21 * np.exp(1j * angle)
    return s21_rotated, angle


def deembed(frequencies, s21, tau=None):
    """
    Full deembedding pipeline: cable delay removal, circle centering, rotation.

    Args:
        frequencies: 1D array of frequencies in Hz.
        s21: 1D complex array of S21 values.
        tau: Cable delay in seconds. If None, auto-estimated.

    Returns:
        s21_deembedded: Complex array after full deembedding.
        params: dict with keys:
            'tau': cable delay removed (seconds)
            'center': complex circle center
            'radius': float circle radius
            'rotation_angle': float rotation applied (radians)
    """
    s21_nodelay, tau = remove_cable_delay(frequencies, s21, tau=tau)
    s21_centered, center, radius = center_circle(s21_nodelay)
    s21_rotated, angle = rotate_to_real_axis(s21_centered)

    params = {
        'tau': tau,
        'center': center,
        'radius': radius,
        'rotation_angle': angle,
    }
    return s21_rotated, params


def apply_deembed_params(s21, params):
    """
    Apply previously computed deembedding parameters to new data.

    Useful for applying sweep-derived deembedding to timestream data.

    Args:
        s21: Complex array (or single complex value) to transform.
        params: dict from deembed() with keys 'tau', 'center',
                'rotation_angle'. Note: 'tau' is not applied here
                since timestream data is at a single frequency.

    Returns:
        s21_deembedded: Transformed complex array.
    """
    s21 = np.asarray(s21, dtype=complex)
    s21_centered = s21 - params['center']
    s21_rotated = s21_centered * np.exp(1j * params['rotation_angle'])
    return s21_rotated
