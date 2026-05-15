"""
Resonator data transforms for MKID S21 analysis.

Provides two independent categories of transform on complex S21 data:

**Deembedding** (``deembed``) — the RF procedure for representing a
notch resonator.  Removes cable delay and normalises by the
off-resonance environment response so that the off-resonance point
sits at (1, 0) and the on-resonance point lies near zero on the
positive real axis (overcoupled case).

**Phase centering** (``phase_center``) — translates the resonance
circle so its algebraic center sits at the origin, then rotates the
centered circle so that the off-resonance point lies on the negative
real axis and the resonance point lies near zero phase.  Phase centering
can be applied to raw *or* deembedded data.

Also provides a ``ResonatorCalibration`` class for vectorized
IQ - frequency/dissipation conversion that relies on the
phase-centered representation.

Uncertainty propagation:
- Public transform functions accept optional ``s21_err`` arrays.
- A complex error array means real=sigma_I and imag=sigma_Q, matching
  the readout-server sweep_ei/sweep_eq convention.
- Complex multiplications, including cable-delay rotation, baseline
  division, and phase rotation, propagate independent I/Q uncertainties.
- Translation by a circle center does not change per-point uncertainty.
- Uncertainty in fitted transform parameters themselves is not included.
"""

import numpy as np


def _coerce_s21_error(s21_err, shape=None):
    """Return I/Q standard uncertainties with the requested shape.

    Error arrays in this package follow the readout-server convention:
    a complex error means ``real == sigma_I`` and ``imag == sigma_Q``.
    A real error is interpreted as the same standard uncertainty for both
    quadratures, and a ``(sigma_I, sigma_Q)`` tuple is also accepted.
    This helper only normalises shape and sign; it deliberately does not
    decide whether the input represents a standard deviation or standard
    error of the mean.
    """
    if s21_err is None:
        return None

    if isinstance(s21_err, (tuple, list)) and len(s21_err) == 2:
        err_i, err_q = s21_err
    else:
        err = np.asarray(s21_err)
        if np.iscomplexobj(err):
            err_i = err.real
            err_q = err.imag
        else:
            err_i = err
            err_q = err

    if shape is None:
        err_i = np.abs(err_i).astype(float, copy=True)
        err_q = np.abs(err_q).astype(float, copy=True)
        return err_i, err_q

    err_i = np.broadcast_to(np.abs(err_i), shape).astype(float, copy=True)
    err_q = np.broadcast_to(np.abs(err_q), shape).astype(float, copy=True)
    return err_i, err_q


def transform_s21_error(s21_err, multiplier):
    """Propagate I/Q uncertainties through ``z_out = multiplier * z``.

    The calculation assumes independent I and Q errors and does not include
    uncertainty in the transform parameters themselves (for example the
    fitted baseline, delay, circle center, or rotation angle).  Translation
    does not change per-point uncertainty; complex multiplication rotates
    and scales it.  This matters when sigma_I != sigma_Q because a pure
    90-degree rotation swaps the quadrature uncertainties:

        I' = Re(c) I - Im(c) Q
        Q' = Im(c) I + Re(c) Q

    Args:
        s21_err: Real, complex, or ``(sigma_I, sigma_Q)`` error array.
        multiplier: Complex scalar or array multiplying the S21 data.

    Returns:
        Complex error array with real=sigma_I and imag=sigma_Q.
    """
    multiplier = np.asarray(multiplier, dtype=complex)
    err_i, err_q = _coerce_s21_error(s21_err)
    shape = np.broadcast_shapes(multiplier.shape, err_i.shape, err_q.shape)
    multiplier = np.broadcast_to(multiplier, shape)
    err_i = np.broadcast_to(err_i, shape)
    err_q = np.broadcast_to(err_q, shape)
    c_re = multiplier.real
    c_im = multiplier.imag
    out_i = np.sqrt((c_re * err_i) ** 2 + (c_im * err_q) ** 2)
    out_q = np.sqrt((c_im * err_i) ** 2 + (c_re * err_q) ** 2)
    return out_i + 1j * out_q


def remove_cable_delay(frequencies, s21, tau=None, s21_err=None):
    """
    Remove electrical delay (cable delay) from S21 data.

    Multiplies S21 by exp(+j * 2 * pi * f * tau) to unwind the linear
    phase slope caused by cable length.

    Args:
        frequencies: 1D array of frequencies in Hz.
        s21: 1D complex array of S21 values.
        tau: Cable delay in seconds. If None, estimated from the
             median gradient of the unwrapped phase.
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.

    Returns:
        If ``s21_err`` is None: ``(s21_corrected, tau)``.
        Otherwise: ``(s21_corrected, s21_err_corrected, tau)``.
    """
    frequencies = np.asarray(frequencies, dtype=float)
    s21 = np.asarray(s21, dtype=complex)

    if tau is None:
        phase = np.unwrap(np.angle(s21))
        # Median of phase gradient is robust to resonance dips
        dphase_df = np.gradient(phase, frequencies)
        tau = -np.median(dphase_df) / (2 * np.pi)

    multiplier = np.exp(1j * 2 * np.pi * frequencies * tau)
    s21_corrected = s21 * multiplier
    if s21_err is None:
        return s21_corrected, tau
    s21_err_corrected = transform_s21_error(s21_err, multiplier)
    return s21_corrected, s21_err_corrected, tau


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

    if isinstance(group_delay_cal, str):
        raise TypeError(
            'group_delay_cal does not accept config path strings directly; '
            'load the calibration first with '
            'ReadoutClient.load_path_group_delay_calibration().')

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


def _integrate_group_delay(group_delay_cal, frequencies):
    """Integrate a group delay calibration to recover cumulative phase.

    Group delay is the derivative of phase: τ(f) = -1/(2π) dφ/df.
    Recovering the phase requires integration.
    For a constant delay the integral reduces to 2πfτ..

    Args:
        group_delay_cal: Group delay calibration (same formats as
            ``_resolve_group_delay_cal``).
        frequencies: 1-D array of frequencies in Hz.

    Returns:
        phase: 1-D array of cumulative phase in radians (same shape as
        *frequencies*).
    """
    frequencies = np.asarray(frequencies, dtype=float)

    # Scalar delay: integral is 2π·f·τ exactly
    if not isinstance(group_delay_cal, dict) and np.ndim(group_delay_cal) == 0:
        tau_s = float(group_delay_cal) * 1e-9
        return 2 * np.pi * frequencies * tau_s

    # Extract calibration grid
    if isinstance(group_delay_cal, dict):
        cal_f = np.asarray(group_delay_cal['frequencies'], dtype=float).ravel()
        cal_tau_ns = np.asarray(group_delay_cal['tau_ns'], dtype=float).ravel()
    else:
        arr = np.asarray(group_delay_cal, dtype=float)
        cal_f = arr[:, 0]
        cal_tau_ns = arr[:, 1]

    order = np.argsort(cal_f)
    cal_f = cal_f[order]
    cal_tau_s = cal_tau_ns[order] * 1e-9

    # Cumulative trapezoidal integration on the calibration grid.
    df = np.diff(cal_f)
    avg_tau = (cal_tau_s[:-1] + cal_tau_s[1:]) / 2
    cum_phase = np.zeros(len(cal_f))
    cum_phase[1:] = np.cumsum(2 * np.pi * avg_tau * df)

    # Re-zero so that the phase is 0 at the first target frequency.
    # This avoids inventing phase below the calibration grid and keeps
    # the correction relative to the start of the data being corrected.
    f0 = float(frequencies.ravel()[0])
    phase_at_f0 = float(np.interp(f0, cal_f, cum_phase))
    cum_phase -= phase_at_f0

    # Interpolate cumulative phase to target frequencies, with linear
    # extrapolation using the edge group delay values
    flat = frequencies.ravel()
    phase = np.interp(flat, cal_f, cum_phase)
    below = flat < cal_f[0]
    above = flat > cal_f[-1]
    if np.any(below):
        phase[below] = cum_phase[0] + 2 * np.pi * cal_tau_s[0] * (flat[below] - cal_f[0])
    if np.any(above):
        phase[above] = cum_phase[-1] + 2 * np.pi * cal_tau_s[-1] * (flat[above] - cal_f[-1])

    return phase.reshape(frequencies.shape)


def remove_group_delay(frequencies, s21, group_delay_cal, s21_err=None):
    """
    Remove frequency-dependent group delay from S21 data.

    Like ``remove_cable_delay`` but uses a frequency-dependent calibration
    instead of a single scalar delay.  The calibration is typically produced
    by ``ReadoutClient.measure_path_group_delay()``.

    The phase accumulated by a frequency-dependent group delay τ(f) is
    φ(f) = 2π ∫ τ(f') df', *not* 2πf·τ(f).  This function integrates
    the calibration to obtain the correct phase correction.

    Args:
        frequencies: 1-D array of frequencies in Hz.
        s21: 1-D complex array of S21 values.
        group_delay_cal: Group delay calibration.  Accepted formats:
            - scalar (ns) — constant delay.
            - ``[[freq_hz, tau_ns], ...]`` — frequency-dependent pairs.
            - result dict from ``measure_path_group_delay()`` with keys
              ``'frequencies'`` and ``'tau_ns'``.
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.

    Returns:
        If ``s21_err`` is None: ``(s21_corrected, tau_s)``.
        Otherwise: ``(s21_corrected, s21_err_corrected, tau_s)``.
    """
    frequencies = np.asarray(frequencies, dtype=float)
    s21 = np.asarray(s21, dtype=complex)
    tau_s = _resolve_group_delay_cal(group_delay_cal, frequencies)
    phase = _integrate_group_delay(group_delay_cal, frequencies)
    multiplier = np.exp(1j * phase)
    s21_corrected = s21 * multiplier
    if s21_err is None:
        return s21_corrected, tau_s
    s21_err_corrected = transform_s21_error(s21_err, multiplier)
    return s21_corrected, s21_err_corrected, tau_s


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


def rotate_to_real_axis(s21, s21_off_resonance=None, n_edge=5):
    """
    Rotate centered S21 so off resonance lies on the negative real axis.

    This is the second step of *phase centering* (after ``center_circle``).
    The result places the off-resonance point near ±π rad and the
    on-resonance point near 0 rad.

    Args:
        s21: 1D complex array (should already be centered).
        s21_off_resonance: Centered complex value off resonance. If None,
            uses the average of the centered sweep edge points.
        n_edge: Number of points from each edge to average when estimating
            the off-resonance point.

    Returns:
        s21_rotated: Complex array.
        angle: Rotation angle applied (radians).
    """
    s21 = np.asarray(s21, dtype=complex)

    if s21_off_resonance is None:
        s21_off_resonance = _estimate_baseline(s21, n_edge=n_edge)

    # Rotate so the off-resonance point is on the negative real axis.
    angle = np.pi - np.angle(s21_off_resonance)
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

def deembed(frequencies, s21, tau=None, group_delay_cal=None, s21_err=None):
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
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.

    Returns:
        If ``s21_err`` is None: ``(s21_deembedded, params)``.
        Otherwise: ``(s21_deembedded, s21_err_deembedded, params)``.

        The propagated errors include the pointwise delay rotation and
        baseline division, but not uncertainty in the estimated tau or
        baseline themselves. ``params`` has keys:
        'tau', 'baseline', and 'group_delay_cal'.
    """
    if group_delay_cal is not None:
        if s21_err is None:
            s21_no_delay, tau = remove_group_delay(
                frequencies, s21, group_delay_cal)
            err_no_delay = None
        else:
            s21_no_delay, err_no_delay, tau = remove_group_delay(
                frequencies, s21, group_delay_cal, s21_err=s21_err)
    else:
        if s21_err is None:
            s21_no_delay, tau = remove_cable_delay(
                frequencies, s21, tau=tau)
            err_no_delay = None
        else:
            s21_no_delay, err_no_delay, tau = remove_cable_delay(
                frequencies, s21, tau=tau, s21_err=s21_err)
    baseline = _estimate_baseline(s21_no_delay)
    s21_deembedded = s21_no_delay / baseline
    if err_no_delay is not None:
        s21_err_deembedded = transform_s21_error(err_no_delay, 1.0 / baseline)

    params = {
        'tau': tau,
        'baseline': baseline,
        'group_delay_cal': group_delay_cal,
    }
    if s21_err is not None:
        return s21_deembedded, s21_err_deembedded, params
    return s21_deembedded, params


def apply_deembed_params(s21, params, frequency=None, s21_err=None):
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
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.

    Returns:
        If ``s21_err`` is None: deembedded complex array.
        Otherwise: ``(s21_deembedded, s21_err_deembedded)``.
    """
    s21 = np.asarray(s21, dtype=complex)
    multiplier = 1.0 / params['baseline']
    if frequency is not None:
        group_delay_cal = params.get('group_delay_cal')
        if group_delay_cal is not None:
            phase = _integrate_group_delay(group_delay_cal, np.atleast_1d(frequency))
            multiplier = np.exp(1j * phase) / params['baseline']
        elif params.get('tau') is not None:
            multiplier = (np.exp(
                1j * 2 * np.pi * np.asarray(frequency, dtype=float) * params['tau'])
                / params['baseline'])
    s21_deembedded = s21 * multiplier
    if s21_err is None:
        return s21_deembedded
    s21_err_deembedded = transform_s21_error(s21_err, multiplier)
    return s21_deembedded, s21_err_deembedded


# -- Phase centering --------------------------------------------------------

def phase_center(s21, s21_err=None, n_edge=5):
    """
    Phase-center S21 data: circle centering followed by rotation.

    Translates the resonance circle so its algebraic center is at the
    origin and rotates so that the off-resonance point lies on the
    negative real axis (off-resonance near ±π rad, on-resonance near 0 rad).

    Can be applied to raw, cable-delay-corrected, or deembedded data.

    Args:
        s21: 1D complex array.
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.
        n_edge: Number of points from each sweep edge to average when
            estimating the off-resonance point for rotation.

    Returns:
        If ``s21_err`` is None: ``(s21_centered, params)``.
        Otherwise: ``(s21_centered, s21_err_centered, params)``.
        Translation by the fitted center does not change per-point
        uncertainty; rotation propagates I/Q uncertainties with
        ``transform_s21_error``.
    """
    s21_centered, center, radius = center_circle(s21)
    s21_rotated, angle = rotate_to_real_axis(s21_centered, n_edge=n_edge)
    if s21_err is not None:
        s21_err_rotated = transform_s21_error(s21_err, np.exp(1j * angle))

    params = {
        'center': center,
        'radius': radius,
        'rotation_angle': angle,
    }
    if s21_err is not None:
        return s21_rotated, s21_err_rotated, params
    return s21_rotated, params


def apply_phase_center_params(s21, params, s21_err=None):
    """
    Apply phase-centering parameters to new data.

    Subtracts the circle center and applies the rotation.  Useful for
    applying sweep-derived phase centering to timestream data.

    Args:
        s21: Complex array (or single complex value) to transform.
        params: dict from ``phase_center()`` with keys 'center',
                'rotation_angle'.
        s21_err: Optional S21 uncertainty. A complex array is interpreted
            as real=sigma_I and imag=sigma_Q.

    Returns:
        If ``s21_err`` is None: phase-centered complex array.
        Otherwise: ``(s21_centered, s21_err_centered)``.
    """
    s21 = np.asarray(s21, dtype=complex)
    s21_centered = s21 - params['center']
    multiplier = np.exp(1j * params['rotation_angle'])
    s21_rotated = s21_centered * multiplier
    if s21_err is None:
        return s21_rotated
    s21_err_rotated = transform_s21_error(s21_err, multiplier)
    return s21_rotated, s21_err_rotated


class ResonatorCalibration:
    """
    Cached calibration for vectorized IQ - frequency/dissipation conversion.

    Stores the phase-centering parameters (cable delay, gain phase, circle
    center, rotation) along with the resonator model parameters (fr, Ql)
    needed to convert between raw IQ and physical quantities.

    On the phase-centered circle (centered at origin, resonance on
    positive real axis), the exact Möbius inversion gives:

        x = (f - fr) / fr = Re[-j * (r - z) / (2 * Ql * (r + z))]

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

    __slots__ = ('fr', 'Ql', 'tau', 'center', 'radius', 'gain_amplitude',
                 'gain_phase', 'rotation_angle', '_deembed_scale',
                 '_rotation_phasor')

    def __init__(self, fr, Ql, tau, center, radius, rotation_angle,
                 gain_amplitude=1.0, gain_phase=0.0):
        self.fr = float(fr)
        self.Ql = float(Ql)
        self.tau = float(tau)
        self.center = complex(center)
        self.radius = float(radius)
        self.gain_amplitude = float(gain_amplitude)
        self.gain_phase = float(gain_phase)
        self.rotation_angle = float(rotation_angle)
        self._deembed_scale = np.exp(-1j * self.gain_phase) / self.gain_amplitude
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
        # The public fit_result.iq_center/iq_radius describe the raw IQ plot.
        # Calibration removes fitted gain and delay before centering, so use
        # the exact deembedded circle geometry from the model.
        center = fit_result.iq_center_deembed
        radius = fit_result.iq_radius_deembed
        # Rotation angle: put the off-resonance point on the negative real
        # axis, leaving the resonance point on the positive real axis.
        # FitResult.alpha is already in the public absolute-frequency phase
        # convention used with exp(-j*2*pi*f*tau), and is removed by
        # _deembed_scale before this rotation is applied.
        # With Qe = Qc*(1 + j*tan(phi)), the coupling term has phase -phi,
        # so after removing alpha the resonance point is at angle pi - phi.
        rotation_angle = getattr(
            fit_result, 'phase_center_rotation_angle', fit_result.phi - np.pi)
        return cls(fr, Ql, tau, center, radius, rotation_angle,
                   gain_amplitude=fit_result.a, gain_phase=fit_result.alpha)

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
            'gain_amplitude': self.gain_amplitude,
            'gain_phase': self.gain_phase,
            'rotation_angle': self.rotation_angle,
        }

    # -- Phase centering -----------------------------------------------------

    def deembed_sweep(self, frequencies, s21, s21_err=None):
        """
        Phase-center sweep data (cable delay removal + centering + rotation).

        Args:
            frequencies: 1D frequency array (Hz).
            s21: Complex S21 array.
            s21_err: Optional S21 uncertainty. A complex array is interpreted
                as real=sigma_I and imag=sigma_Q.

        Returns:
            If ``s21_err`` is None: complex array, phase-centered.
            Otherwise: ``(z_centered, z_err_centered)``.
        """
        s21 = np.asarray(s21, dtype=complex)
        deembed_scale = (
            np.exp(1j * 2 * np.pi * np.asarray(frequencies) * self.tau)
            * self._deembed_scale)
        multiplier = deembed_scale * self._rotation_phasor
        z = (s21 * deembed_scale - self.center) * self._rotation_phasor
        if s21_err is None:
            return z
        return z, transform_s21_error(s21_err, multiplier)

    def deembed_timestream(self, s21, s21_err=None):
        """
        Phase-center timestream data (centering + rotation, no cable delay).

        For timestream IQ at a fixed tone frequency, cable delay is a
        constant phase that is absorbed into the center/rotation. Use
        tone_converter() for the full pipeline including cable delay.

        Args:
            s21: Complex array of timestream IQ samples.
            s21_err: Optional S21 uncertainty. A complex array is interpreted
                as real=sigma_I and imag=sigma_Q.

        Returns:
            If ``s21_err`` is None: complex array, phase-centered.
            Otherwise: ``(z_centered, z_err_centered)``.
        """
        s21 = np.asarray(s21, dtype=complex)
        z = (s21 * self._deembed_scale - self.center) * self._rotation_phasor
        if s21_err is None:
            return z
        return z, transform_s21_error(
            s21_err, self._deembed_scale * self._rotation_phasor)

    # -- Conversions ---------------------------------------------------------

    def to_phase_amplitude(self, z_centered):
        """
        Convert phase-centered IQ to (phase, normalised amplitude).

        Args:
            z_centered: Complex array on the phase-centered circle
                        (output of deembed_sweep or deembed_timestream).

        Returns:
            phase: Angle on the resonance circle (rad). Zero at the
                   resonance point (+real axis), ±pi off resonance.
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
            z = r * (1 - 2j*Ql*x) / (1 + 2j*Ql*x),  x = (f-fr)/fr

        Inverting:
            x = Re[-j * (r - z) / (2*Ql * (r + z))]

        Args:
            z_centered: Complex array on the phase-centered circle.

        Returns:
            df: Frequency shift from resonance (Hz).
            dd: Fractional dissipation shift, (|z|/r - 1).
                Zero on the model circle; positive = increased loss.
        """
        r = self.radius
        z = np.asarray(z_centered, dtype=complex)
        x = np.real(-1j * (r - z) / (2.0 * self.Ql * (r + z)))
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
        x   = Re[-j*(r - z_d) / (2*Ql*(r + z_d))]  (Möbius inversion)
        df  = x * fr

    Construct via ResonatorCalibration.tone_converter(f_tone).
    """

    __slots__ = ('_multiply', '_offset', '_fr', '_Ql', '_radius')

    def __init__(self, cal, f_tone):
        # Combine cable delay, gain removal, and rotation into one phasor.
        self._multiply = (np.exp(1j * 2 * np.pi * f_tone * cal.tau)
                          * cal._deembed_scale * cal._rotation_phasor)
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
        x = np.real(-1j * (r - z_d) / (2.0 * self._Ql * (r + z_d)))
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
