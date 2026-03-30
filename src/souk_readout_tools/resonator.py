"""
Resonator data transforms for MKID S21 analysis.

Provides deembedding operations (cable delay removal, circle centering,
rotation) as pure numerical transforms on complex S21 data, and a
ResonatorCalibration class for vectorized IQ ↔ frequency/dissipation
conversion.

These transforms are used by the plotting library when deembed=True,
but can also be used independently for resonator characterisation and
real-time readout.
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


class ResonatorCalibration:
    """
    Cached calibration for vectorized IQ ↔ frequency/dissipation conversion.

    Stores the deembedding parameters (cable delay, circle center, rotation)
    along with the resonator model parameters (fr, Ql) needed to convert
    between raw IQ and physical quantities.

    On the deembedded circle (centered at origin, resonance on negative
    real axis), the exact Möbius inversion gives:

        x = (f - fr) / fr = Re[-j * (z + r) / (2 * Ql * (r - z))]

    This is valid for arbitrary detuning, not just small perturbations.

    Construct from a FitResult or from raw sweep data:

        cal = ResonatorCalibration.from_fit(fit_result)
        cal = ResonatorCalibration.from_sweep(f, z, fr=..., Ql=...)

    For real-time readout at a fixed tone frequency, use a ToneConverter
    to reduce per-sample work to one complex multiply + one add + the
    Möbius inversion:

        convert = cal.tone_converter(f_tone)
        df, dd = convert(z)     # z is raw IQ, scalar or array
    """

    __slots__ = ('fr', 'Ql', 'tau', 'center', 'radius',
                 'rotation_angle', '_rotation_phasor')

    def __init__(self, fr, Ql, tau, center, radius, rotation_angle):
        self.fr = float(fr)
        self.Ql = float(Ql)
        self.tau = float(tau)
        self.center = complex(center)
        self.radius = float(radius)
        self.rotation_angle = float(rotation_angle)
        self._rotation_phasor = np.exp(1j * rotation_angle)

    @classmethod
    def from_fit(cls, fit_result):
        """
        Build from a fitting.FitResult.

        Derives the deembedding geometry (center, radius, rotation) from
        the fitted model parameters rather than from a Kasa circle fit,
        so the calibration is fully consistent with the resonator model.
        """
        fr = fit_result.fr
        Ql = fit_result.Ql
        tau = fit_result.tau
        center = fit_result.iq_center
        radius = fit_result.iq_radius
        # Rotation angle: put the resonance point on the negative real axis.
        # In the centered frame, the resonance point is at angle (pi + alpha + phi).
        rotation_angle = -(fit_result.alpha + fit_result.phi)
        return cls(fr, Ql, tau, center, radius, rotation_angle)

    @classmethod
    def from_sweep(cls, frequencies, s21, fr=None, Ql=None):
        """
        Build from raw sweep data using the deembed pipeline.

        Args:
            frequencies: 1D frequency array (Hz).
            s21: 1D complex S21 array.
            fr: Resonance frequency (Hz). If None, estimated from
                the minimum |S21| point.
            Ql: Loaded quality factor. If None, estimated from the
                3 dB bandwidth.
        """
        s21_deembedded, params = deembed(frequencies, s21)

        if fr is None:
            fr = float(frequencies[np.argmin(np.abs(s21))])
        if Ql is None:
            mag = np.abs(s21)
            min_mag = np.min(mag)
            max_mag = np.max(mag)
            half = (min_mag + max_mag) / 2
            below = np.where(mag < half)[0]
            if len(below) > 1:
                bw = max(frequencies[below[-1]] - frequencies[below[0]],
                         np.median(np.diff(frequencies)))
            else:
                bw = (frequencies[-1] - frequencies[0]) / 10
            Ql = float(fr / bw)

        return cls(fr, Ql, params['tau'], params['center'],
                   params['radius'], params['rotation_angle'])

    @property
    def deembed_params(self):
        """Return a dict compatible with apply_deembed_params()."""
        return {
            'tau': self.tau,
            'center': self.center,
            'radius': self.radius,
            'rotation_angle': self.rotation_angle,
        }

    # -- Deembedding ---------------------------------------------------------

    def deembed_sweep(self, frequencies, s21):
        """
        Deembed sweep data (frequency-dependent cable delay removal).

        Args:
            frequencies: 1D frequency array (Hz).
            s21: Complex S21 array.

        Returns:
            Complex array, fully deembedded.
        """
        s21 = np.asarray(s21, dtype=complex)
        z = s21 * np.exp(1j * 2 * np.pi * np.asarray(frequencies) * self.tau)
        return (z - self.center) * self._rotation_phasor

    def deembed_timestream(self, s21):
        """
        Deembed timestream data (no cable delay removal — fixed tone).

        For timestream IQ at a fixed tone frequency, cable delay is a
        constant phase that is absorbed into the center/rotation. Use
        tone_converter() for the full pipeline including cable delay.

        Args:
            s21: Complex array of timestream IQ samples.

        Returns:
            Complex array, centered and rotated.
        """
        s21 = np.asarray(s21, dtype=complex)
        return (s21 - self.center) * self._rotation_phasor

    # -- Conversions ---------------------------------------------------------

    def to_phase_amplitude(self, z_deembedded):
        """
        Convert deembedded IQ to (phase, normalised amplitude).

        Args:
            z_deembedded: Complex array on the deembedded circle
                          (output of deembed_sweep or deembed_timestream).

        Returns:
            phase: Angle on the resonance circle (rad). Zero at the
                   off-resonance point (+real axis), ±pi at resonance.
            amplitude: |z| / radius. Unity on the model circle;
                       deviations indicate dissipation changes.
        """
        phase = np.angle(z_deembedded)
        amplitude = np.abs(z_deembedded) / self.radius
        return phase, amplitude

    def to_frequency_dissipation(self, z_deembedded):
        """
        Convert deembedded IQ to (frequency shift, dissipation shift).

        Uses the exact Möbius inversion of the resonance circle, valid
        for arbitrary detuning (not just small perturbations).

        On the deembedded circle the model is:
            z = r * (-1 + 2j*Ql*x) / (1 + 2j*Ql*x),  x = (f-fr)/fr

        Inverting:
            x = Re[-j * (z + r) / (2*Ql * (r - z))]

        Args:
            z_deembedded: Complex array on the deembedded circle.

        Returns:
            df: Frequency shift from resonance (Hz).
            dd: Fractional dissipation shift, (|z|/r - 1).
                Zero on the model circle; positive = increased loss.
        """
        r = self.radius
        z = np.asarray(z_deembedded, dtype=complex)
        x = np.real(-1j * (z + r) / (2.0 * self.Ql * (r - z)))
        df = x * self.fr
        dd = np.abs(z) / r - 1.0
        return df, dd

    # -- Fixed-tone converter ------------------------------------------------

    def tone_converter(self, f_tone):
        """
        Return a ToneConverter for a fixed tone frequency.

        Pre-combines cable delay + centering + rotation into a single
        complex multiply and add, minimising per-sample work for
        real-time readout or tracking loops.

        Args:
            f_tone: Tone frequency (Hz).

        Returns:
            ToneConverter callable: df, dd = converter(z_raw)
        """
        return ToneConverter(self, f_tone)


class ToneConverter:
    """
    Optimised IQ → (df, dd) converter for a single fixed tone frequency.

    All per-tone constants are pre-computed so that each call is:
        z_d = z * _multiply - _offset      (1 complex mul + 1 complex sub)
        x   = Re[-j*(z_d + r) / (2*Ql*(r - z_d))]  (Möbius inversion)
        df  = x * fr

    Construct via ResonatorCalibration.tone_converter(f_tone).
    """

    __slots__ = ('_multiply', '_offset', '_fr', '_Ql', '_radius')

    def __init__(self, cal, f_tone):
        # Combine cable delay and rotation into a single phasor
        self._multiply = np.exp(1j * 2 * np.pi * f_tone * cal.tau) * cal._rotation_phasor
        self._offset = cal.center * cal._rotation_phasor
        self._fr = cal.fr
        self._Ql = cal.Ql
        self._radius = cal.radius

    def __call__(self, z):
        """
        Convert raw IQ to (df, dd).

        Args:
            z: Raw complex IQ data (scalar or array).

        Returns:
            df: Frequency shift from resonance (Hz).
            dd: Fractional dissipation shift.
        """
        z_d = z * self._multiply - self._offset
        r = self._radius
        x = np.real(-1j * (z_d + r) / (2.0 * self._Ql * (r - z_d)))
        df = x * self._fr
        dd = np.abs(z_d) / r - 1.0
        return df, dd

    def deembed(self, z):
        """
        Deembed raw IQ without converting to frequency/dissipation.

        Useful when you want the deembedded circle for plotting.

        Args:
            z: Raw complex IQ data (scalar or array).

        Returns:
            Complex deembedded IQ.
        """
        return z * self._multiply - self._offset
