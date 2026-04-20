"""
Resonator data transforms for MKID S21 analysis.

Provides two independent categories of transform on complex S21 data:

**Deembedding** (``deembed``) — the RF procedure for representing a
notch resonator.  Removes cable delay and normalises by the
off-resonance environment response so that the off-resonance point
sits at (1, 0) and the on-resonance point lies near zero on the
positive real axis (overcoupled case).

**Phase centering** (``phase_center``) — translates the resonance
circle so its algebraic center sits at the origin and rotates it so
that the resonance point lies on the negative real axis (zero phase
off-resonance).  Phase centering can be applied to raw *or*
deembedded data.

Also provides a ``ResonatorCalibration`` class for vectorized
IQ - frequency/dissipation conversion that relies on the
phase-centered representation.
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


def _resolve_group_delay_cal(group_delay_cal, frequencies):
    """Resolve a group delay calibration to a per-frequency tau array in seconds.

    Accepts the same formats as ``rf_frontend.path_group_delay_ns`` in the
    config:

    - scalar (float) — constant delay in nanoseconds.
    - ``[[freq_hz, tau_ns], ...]`` — frequency-dependent pairs; linearly
      interpolated to *frequencies*.
    - result dict from ``measure_path_group_delay()`` — uses the
      ``'frequencies'`` and ``'tau_ns'`` keys.

    Returns:
        tau_s: 1-D array of delay values in **seconds**, same length as
        *frequencies*.
    """
    frequencies = np.asarray(frequencies, dtype=float)

    if isinstance(group_delay_cal, dict):
        cal_f = np.asarray(group_delay_cal['frequencies'], dtype=float).ravel()
        cal_tau_ns = np.asarray(group_delay_cal['tau_ns'], dtype=float).ravel()
    elif np.ndim(group_delay_cal) == 0:
        return np.full(frequencies.shape, float(group_delay_cal) * 1e-9)
    else:
        arr = np.asarray(group_delay_cal, dtype=float)
        cal_f = arr[:, 0]
        cal_tau_ns = arr[:, 1]

    tau_ns = np.interp(frequencies.ravel(), cal_f, cal_tau_ns)
    return tau_ns.reshape(frequencies.shape) * 1e-9


def remove_group_delay(frequencies, s21, group_delay_cal):
    """
    Remove frequency-dependent group delay from S21 data.

    Like ``remove_cable_delay`` but uses a frequency-dependent calibration
    instead of a single scalar delay.  The calibration is typically produced
    by ``ReadoutClient.measure_path_group_delay()``.

    Args:
        frequencies: 1-D array of frequencies in Hz.
        s21: 1-D complex array of S21 values.
        group_delay_cal: Group delay calibration.  Accepted formats:
            - scalar (ns) — constant delay.
            - ``[[freq_hz, tau_ns], ...]`` — frequency-dependent pairs.
            - result dict from ``measure_path_group_delay()`` with keys
              ``'frequencies'`` and ``'tau_ns'``.

    Returns:
        s21_corrected: Complex array with group delay removed.
        tau_s: 1-D array of delay values removed (seconds).
    """
    frequencies = np.asarray(frequencies, dtype=float)
    s21 = np.asarray(s21, dtype=complex)
    tau_s = _resolve_group_delay_cal(group_delay_cal, frequencies)
    s21_corrected = s21 * np.exp(1j * 2 * np.pi * frequencies * tau_s)
    return s21_corrected, tau_s


def center_circle(s21):
    """
    Translate the resonance circle so its algebraic center is at the origin.

    This is the first step of *phase centering*.  Uses the Kasa algebraic
    circle fit to find the center and radius of the resonance circle in
    the complex plane.

    Args:
        s21: 1D complex array (may be raw or cable-delay-corrected).

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

    This is the second step of *phase centering* (after ``center_circle``).
    The result places the off-resonance point near 0 rad and the
    on-resonance point near ±π rad.

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


def _estimate_baseline(s21, n_edge=5):
    """Estimate the off-resonance baseline from sweep edge points.

    Averages ``n_edge`` points from each end of the sweep (assumed to be
    far from resonance) to obtain a single complex baseline value.
    """
    s21 = np.asarray(s21, dtype=complex)
    n = max(1, min(n_edge, len(s21) // 4))
    edge_pts = np.concatenate([s21[:n], s21[-n:]])
    return np.mean(edge_pts)


# -- True RF deembedding ----------------------------------------------------

def deembed(frequencies, s21, tau=None, group_delay_cal=None):
    """
    True RF deembedding for a notch resonator.

    Removes cable delay and normalises by the off-resonance baseline so
    that the off-resonance point sits at (1, 0) and the on-resonance
    point lies near zero on the positive real axis (overcoupled case).

    Args:
        frequencies: 1D array of frequencies in Hz.
        s21: 1D complex array of S21 values.
        tau: Cable delay in seconds. If None, auto-estimated.
            Ignored when ``group_delay_cal`` is provided.
        group_delay_cal: Frequency-dependent group delay calibration
            (from ``measure_path_group_delay()``).  When provided, used
            instead of a scalar ``tau``.  Accepted formats: scalar (ns),
            ``[[freq_hz, tau_ns], ...]``, or result dict.

    Returns:
        s21_deembedded: Complex array in the standard notch resonator
            representation.
        params: dict with keys:
            'tau': cable delay removed (seconds; scalar or array)
            'baseline': complex off-resonance baseline value
            'group_delay_cal': the calibration used, or None
    """
    if group_delay_cal is not None:
        s21_no_delay, tau = remove_group_delay(frequencies, s21, group_delay_cal)
    else:
        s21_no_delay, tau = remove_cable_delay(frequencies, s21, tau=tau)
    baseline = _estimate_baseline(s21_no_delay)
    s21_deembedded = s21_no_delay / baseline

    params = {
        'tau': tau,
        'baseline': baseline,
        'group_delay_cal': group_delay_cal,
    }
    return s21_deembedded, params


def apply_deembed_params(s21, params, frequency=None):
    """
    Apply deembed parameters to new data (e.g. timestream).

    Divides by the off-resonance baseline.  For timestream data at a
    known tone frequency, also removes the cable delay at that frequency.

    Args:
        s21: Complex array (or single complex value) to transform.
        params: dict from ``deembed()`` with keys 'tau', 'baseline'.
        frequency: Tone frequency in Hz.  When provided, cable delay is
            removed at this frequency before baseline normalisation.
            When None, only baseline normalisation is applied.

    Returns:
        s21_deembedded: Deembedded complex array.
    """
    s21 = np.asarray(s21, dtype=complex)
    if frequency is not None:
        group_delay_cal = params.get('group_delay_cal')
        if group_delay_cal is not None:
            tau_s = _resolve_group_delay_cal(group_delay_cal, np.atleast_1d(frequency))
            s21 = s21 * np.exp(1j * 2 * np.pi * np.asarray(frequency, dtype=float) * tau_s)
        elif params.get('tau') is not None:
            s21 = s21 * np.exp(
                1j * 2 * np.pi * np.asarray(frequency, dtype=float) * params['tau'])
    return s21 / params['baseline']


# -- Phase centering --------------------------------------------------------

def phase_center(s21):
    """
    Phase-center S21 data: circle centering followed by rotation.

    Translates the resonance circle so its algebraic center is at the
    origin and rotates so that the resonance point lies on the negative
    real axis (off-resonance near 0 rad, on-resonance near ±π rad).

    Can be applied to raw, cable-delay-corrected, or deembedded data.

    Args:
        s21: 1D complex array.

    Returns:
        s21_centered: Phase-centered complex array.
        params: dict with keys:
            'center': complex circle center
            'radius': float circle radius
            'rotation_angle': float rotation applied (radians)
    """
    s21_centered, center, radius = center_circle(s21)
    s21_rotated, angle = rotate_to_real_axis(s21_centered)

    params = {
        'center': center,
        'radius': radius,
        'rotation_angle': angle,
    }
    return s21_rotated, params


def apply_phase_center_params(s21, params):
    """
    Apply phase-centering parameters to new data.

    Subtracts the circle center and applies the rotation.  Useful for
    applying sweep-derived phase centering to timestream data.

    Args:
        s21: Complex array (or single complex value) to transform.
        params: dict from ``phase_center()`` with keys 'center',
                'rotation_angle'.

    Returns:
        s21_centered: Phase-centered complex array.
    """
    s21 = np.asarray(s21, dtype=complex)
    s21_centered = s21 - params['center']
    return s21_centered * np.exp(1j * params['rotation_angle'])


class ResonatorCalibration:
    """
    Cached calibration for vectorized IQ - frequency/dissipation conversion.

    Stores the phase-centering parameters (cable delay, circle center,
    rotation) along with the resonator model parameters (fr, Ql) needed
    to convert between raw IQ and physical quantities.

    On the phase-centered circle (centered at origin, resonance on
    negative real axis), the exact Möbius inversion gives:

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

        Derives the phase-centering geometry (center, radius, rotation)
        from the fitted model parameters rather than from a Kasa circle
        fit, so the calibration is fully consistent with the resonator
        model.
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
    def from_sweep(cls, frequencies, s21, fr=None, Ql=None, group_delay_cal=None):
        """
        Build from raw sweep data using cable delay removal + phase centering.

        Args:
            frequencies: 1D frequency array (Hz).
            s21: 1D complex S21 array.
            fr: Resonance frequency (Hz). If None, estimated from
                the minimum |S21| point.
            Ql: Loaded quality factor. If None, estimated from the
                3 dB bandwidth.
            group_delay_cal: Frequency-dependent group delay calibration
                (from ``measure_path_group_delay()``).  When provided,
                used instead of auto-estimated scalar cable delay.
        """
        if group_delay_cal is not None:
            s21_no_delay, tau = remove_group_delay(frequencies, s21, group_delay_cal)
        else:
            s21_no_delay, tau = remove_cable_delay(frequencies, s21)
        _, pc_params = phase_center(s21_no_delay)

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

        return cls(fr, Ql, tau, pc_params['center'],
                   pc_params['radius'], pc_params['rotation_angle'])

    @property
    def phase_center_params(self):
        """Return a phase-centering params dict for apply_phase_center_params()."""
        return {
            'center': self.center,
            'radius': self.radius,
            'rotation_angle': self.rotation_angle,
        }

    # -- Phase centering -----------------------------------------------------

    def deembed_sweep(self, frequencies, s21):
        """
        Phase-center sweep data (cable delay removal + centering + rotation).

        Args:
            frequencies: 1D frequency array (Hz).
            s21: Complex S21 array.

        Returns:
            Complex array, phase-centered.
        """
        s21 = np.asarray(s21, dtype=complex)
        z = s21 * np.exp(1j * 2 * np.pi * np.asarray(frequencies) * self.tau)
        return (z - self.center) * self._rotation_phasor

    def deembed_timestream(self, s21):
        """
        Phase-center timestream data (centering + rotation, no cable delay).

        For timestream IQ at a fixed tone frequency, cable delay is a
        constant phase that is absorbed into the center/rotation. Use
        tone_converter() for the full pipeline including cable delay.

        Args:
            s21: Complex array of timestream IQ samples.

        Returns:
            Complex array, phase-centered (centered and rotated).
        """
        s21 = np.asarray(s21, dtype=complex)
        return (s21 - self.center) * self._rotation_phasor

    # -- Conversions ---------------------------------------------------------

    def to_phase_amplitude(self, z_centered):
        """
        Convert phase-centered IQ to (phase, normalised amplitude).

        Args:
            z_centered: Complex array on the phase-centered circle
                        (output of deembed_sweep or deembed_timestream).

        Returns:
            phase: Angle on the resonance circle (rad). Zero at the
                   off-resonance point (+real axis), ±pi at resonance.
            amplitude: |z| / radius. Unity on the model circle;
                       deviations indicate dissipation changes.
        """
        phase = np.angle(z_centered)
        amplitude = np.abs(z_centered) / self.radius
        return phase, amplitude

    def to_frequency_dissipation(self, z_centered):
        """
        Convert phase-centered IQ to (frequency shift, dissipation shift).

        Uses the exact Möbius inversion of the resonance circle, valid
        for arbitrary detuning (not just small perturbations).

        On the phase-centered circle the model is:
            z = r * (-1 + 2j*Ql*x) / (1 + 2j*Ql*x),  x = (f-fr)/fr

        Inverting:
            x = Re[-j * (z + r) / (2*Ql * (r - z))]

        Args:
            z_centered: Complex array on the phase-centered circle.

        Returns:
            df: Frequency shift from resonance (Hz).
            dd: Fractional dissipation shift, (|z|/r - 1).
                Zero on the model circle; positive = increased loss.
        """
        r = self.radius
        z = np.asarray(z_centered, dtype=complex)
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

    Pre-combines cable delay removal and phase centering into a single
    complex multiply and add, so each call is:
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
        Phase-center raw IQ without converting to frequency/dissipation.

        Applies cable delay removal + centering + rotation in a single
        step.  Useful when you want the phase-centered circle for
        plotting.

        Args:
            z: Raw complex IQ data (scalar or array).

        Returns:
            Complex phase-centered IQ.
        """
        return z * self._multiply - self._offset
