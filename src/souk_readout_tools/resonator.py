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

from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np
from scipy import signal


@dataclass
class EmpiricalResonanceEstimate:
    """Measured dip estimates that do not depend on an optimizer."""
    fr: float = np.nan
    linewidth_hz: float = np.nan
    left_width_hz: float = np.nan
    right_width_hz: float = np.nan
    skew: float = np.nan
    Ql: float = np.nan
    Qc: float = np.nan
    Qi: float = np.nan
    dip_depth_db: float = np.nan
    peak_index: int = -1
    marker_log_mag_db: float = np.nan
    baseline_log_mag_db: float = np.nan


@dataclass(frozen=True)
class DeembeddingCalibration:
    """Frequency-dependent raw-IQ envelope removal."""
    scale: complex = 1.0 + 0.0j
    tau: float = 0.0
    group_delay_cal: object = None
    group_delay_reference_frequency: float = None

    def multiplier(self, frequency):
        """Return the complex multiplier that removes the fitted environment."""
        frequency = np.asarray(frequency, dtype=float)
        if self.group_delay_cal is not None:
            phase = _integrate_group_delay(
                self.group_delay_cal,
                frequency,
                reference_frequency=self.group_delay_reference_frequency,
            )
        else:
            phase = 2.0 * np.pi * frequency * self.tau
        return np.exp(1j * phase) * self.scale

    def apply(self, frequency, s21, s21_err=None):
        """Remove the environment response from raw IQ at ``frequency``."""
        if frequency is None:
            raise ValueError("frequency is required when deembedding raw IQ")
        multiplier = self.multiplier(frequency)
        z = np.asarray(s21, dtype=complex) * multiplier
        if s21_err is None:
            return z
        return z, transform_s21_error(s21_err, multiplier)


@dataclass(frozen=True)
class PhaseCenterCalibration:
    """Circle-center subtraction and optional reference-frame rotation."""
    center: complex
    radius: float
    rotation_angle: float

    def apply(self, s21, *, center=True, rotate=True, s21_err=None):
        """Apply the requested phase-centering stages to complex IQ."""
        z = np.asarray(s21, dtype=complex)
        if center:
            z = z - self.center
        multiplier = np.exp(1j * self.rotation_angle) if rotate else 1.0 + 0.0j
        z = z * multiplier
        if s21_err is None:
            return z
        return z, transform_s21_error(s21_err, multiplier)


def estimate_resonance_empirical(frequencies, s21, peak_index=None):
    """Estimate quick-look resonance values directly from complex S21 data.

    Parameters
    ----------
    frequencies : array-like
        Sweep frequencies (Hz).
    s21 : array-like of complex
        Complex S21 at each frequency.
    peak_index : int or None, optional
        Index to treat as the resonance; ``None`` (default) uses the deepest
        finite dip in the log-magnitude trace.
    """
    f = np.asarray(frequencies, dtype=float).ravel()
    z = np.asarray(s21, dtype=complex).ravel()
    if f.size != z.size:
        raise ValueError("frequencies and s21 must have the same length.")
    if f.size == 0:
        return EmpiricalResonanceEstimate()

    # Work in log magnitude; the resonance marker is the supplied point or the
    # deepest finite dip in the trace.
    mag_db = 20.0 * np.log10(np.abs(z) + 1e-300)
    good = np.isfinite(f) & np.isfinite(mag_db)
    if not np.any(good):
        return EmpiricalResonanceEstimate()
    if peak_index is None or not (0 <= int(peak_index) < f.size) or not good[int(peak_index)]:
        peak_index = int(np.nanargmin(np.where(good, mag_db, np.nan)))
    else:
        peak_index = int(peak_index)

    fr = float(f[peak_index])
    step = float(np.nanmedian(np.abs(np.diff(f[good])))) if np.count_nonzero(good) > 1 else 1.0
    step = step if np.isfinite(step) and step > 0.0 else 1.0
    dip_db = float(mag_db[peak_index])

    # Estimate the local off-resonance level from a line between small edge
    # medians.  For centered sweeps this is close to averaging the endpoints,
    # while a sloped through-line is not counted as dip depth.
    good_indices = np.flatnonzero(good)
    n_edge = max(1, min(5, good_indices.size // 4))
    left_edge = good_indices[:n_edge]
    right_edge = good_indices[-n_edge:]
    left_baseline_db = float(np.nanmedian(mag_db[left_edge]))
    right_baseline_db = float(np.nanmedian(mag_db[right_edge]))
    f_left_edge = float(np.nanmedian(f[left_edge]))
    f_right_edge = float(np.nanmedian(f[right_edge]))
    if (
        np.isfinite(f_left_edge)
        and np.isfinite(f_right_edge)
        and f_left_edge != f_right_edge
    ):
        t = (fr - f_left_edge) / (f_right_edge - f_left_edge)
        baseline_db = left_baseline_db + t * (right_baseline_db - left_baseline_db)
    else:
        baseline_db = 0.5 * (left_baseline_db + right_baseline_db)
    shoulder_db = float(
        min(
            np.nanmax(mag_db[: peak_index + 1]),
            np.nanmax(mag_db[peak_index:]),
        )
    )

    depth_db = max(0.0, baseline_db - dip_db)

    # Width at half the dip depth in *linear power* |S21|^2 (not dB). For a
    # symmetric notch this half-power FWHM is the loaded linewidth fr/Ql exactly
    # (the crossing is at 2*Ql*x = 1); the half-depth in dB instead crosses at
    # 2*Ql*x = depth^(1/4), underestimating the width by 10**(-depth_db/40) --
    # badly for deep dips. The baseline is bounded by the weaker shoulder so one
    # high edge of a sloped sweep cannot stretch the width.
    power = np.abs(z) ** 2
    dip_power = float(power[peak_index])
    width_baseline_power = max(
        dip_power, min(10.0 ** (shoulder_db / 10.0), 10.0 ** (baseline_db / 10.0)))
    half_power = 0.5 * (dip_power + width_baseline_power)

    f_left = f_right = np.nan
    for j in range(peak_index - 1, -1, -1):
        if power[j] >= half_power and np.isfinite(power[j + 1]):
            denom = power[j + 1] - power[j]
            frac = 0.0 if denom == 0.0 else (half_power - power[j]) / denom
            f_left = float(f[j] + frac * (f[j + 1] - f[j]))
            break
    for j in range(peak_index + 1, f.size):
        if power[j] >= half_power and np.isfinite(power[j - 1]):
            denom = power[j] - power[j - 1]
            frac = 0.0 if denom == 0.0 else (half_power - power[j - 1]) / denom
            f_right = float(f[j - 1] + frac * (f[j] - f[j - 1]))
            break

    left_width = max(fr - f_left, 0.0) if np.isfinite(f_left) else np.nan
    right_width = max(f_right - fr, 0.0) if np.isfinite(f_right) else np.nan
    linewidth = left_width + right_width if np.isfinite(left_width + right_width) else step
    linewidth = linewidth if linewidth > 0.0 else step
    skew = (
        (right_width - left_width) / linewidth
        if np.isfinite(left_width + right_width)
        else np.nan
    )
    Ql = abs(fr / linewidth) if linewidth > 0.0 else np.inf
    Qc = Ql / (1.0 - 10.0 ** (-depth_db / 20.0)) if depth_db > 0.0 and np.isfinite(Ql) else np.inf
    denom = 1.0 / Ql - 1.0 / Qc if Ql != 0.0 and Qc != 0.0 else 0.0
    Qi = 1.0 / denom if denom != 0.0 else np.inf

    return EmpiricalResonanceEstimate(
        fr=fr,
        linewidth_hz=float(linewidth),
        left_width_hz=float(left_width),
        right_width_hz=float(right_width),
        skew=float(skew),
        Ql=float(Ql),
        Qc=float(Qc),
        Qi=float(Qi),
        dip_depth_db=float(depth_db),
        peak_index=peak_index,
        marker_log_mag_db=dip_db,
        baseline_log_mag_db=float(baseline_db),
    )


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


def _integrate_group_delay(group_delay_cal, frequencies,
                           reference_frequency=None):
    """Integrate a group delay calibration to recover cumulative phase.

    Group delay is the derivative of phase: τ(f) = -1/(2π) dφ/df.
    Recovering the phase requires integration.
    For a constant delay the integral reduces to 2πfτ..

    Args:
        group_delay_cal: Group delay calibration (same formats as
            ``_resolve_group_delay_cal``).
        frequencies: 1-D array of frequencies in Hz.
        reference_frequency: Frequency where the returned phase correction is
            defined to be zero. ``None`` uses the first target frequency.

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

    # Re-zero at an explicit shared reference when reusing calibration
    # parameters on new data. Falling back to the first target preserves the
    # historical standalone behaviour.
    f0 = (
        float(frequencies.ravel()[0])
        if reference_frequency is None else float(reference_frequency)
    )
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


def remove_group_delay(frequencies, s21, group_delay_cal, s21_err=None,
                       reference_frequency=None):
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
        reference_frequency: Frequency where the integrated phase correction is
            zero. ``None`` uses the first corrected frequency.

    Returns:
        If ``s21_err`` is None: ``(s21_corrected, tau_s)``.
        Otherwise: ``(s21_corrected, s21_err_corrected, tau_s)``.
    """
    frequencies = np.asarray(frequencies, dtype=float)
    s21 = np.asarray(s21, dtype=complex)
    tau_s = _resolve_group_delay_cal(group_delay_cal, frequencies)
    phase = _integrate_group_delay(
        group_delay_cal, frequencies, reference_frequency=reference_frequency)
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
    group_delay_reference_frequency = None
    if group_delay_cal is not None:
        group_delay_reference_frequency = float(
            np.asarray(frequencies, dtype=float).ravel()[0])
        if s21_err is None:
            s21_no_delay, tau = remove_group_delay(
                frequencies, s21, group_delay_cal,
                reference_frequency=group_delay_reference_frequency)
            err_no_delay = None
        else:
            s21_no_delay, err_no_delay, tau = remove_group_delay(
                frequencies, s21, group_delay_cal, s21_err=s21_err,
                reference_frequency=group_delay_reference_frequency)
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
        'group_delay_reference_frequency': group_delay_reference_frequency,
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
            phase = _integrate_group_delay(
                group_delay_cal,
                np.atleast_1d(frequency),
                reference_frequency=params.get(
                    'group_delay_reference_frequency'),
            )
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


def apply_phase_center_stages(s21, params, *, center=True, rotate=True,
                              s21_err=None):
    """Apply circle-center subtraction and rotation as separately selectable stages."""
    calibration = PhaseCenterCalibration(
        center=complex(params['center']),
        radius=float(params.get('radius', np.nan)),
        rotation_angle=float(params['rotation_angle']),
    )
    return calibration.apply(
        s21, center=center, rotate=rotate, s21_err=s21_err)


def model_deembed(frequencies, s21, gain_amplitude, gain_phase, tau,
                  s21_err=None):
    """Remove the fitted scalar gain and cable-delay envelope from raw IQ."""
    calibration = DeembeddingCalibration(
        scale=np.exp(-1j * float(gain_phase)) / float(gain_amplitude),
        tau=float(tau),
    )
    return calibration.apply(frequencies, s21, s21_err=s21_err)


def model_phase_center_geometry(Ql, Qe):
    """Return the exact deembedded circle geometry for the notch model."""
    center = 1.0 - 0.5 * float(Ql) / complex(Qe)
    radius = float(abs(float(Ql) / complex(Qe)) / 2.0)
    rotation_angle = float(np.pi - np.angle(1.0 - center))
    rotation_angle = (rotation_angle + np.pi) % (2.0 * np.pi) - np.pi
    return PhaseCenterCalibration(center, radius, rotation_angle)


def rotate_around_point(s21, angle, point=0.0, s21_err=None):
    """Rotate IQ around ``point`` while propagating optional I/Q errors."""
    multiplier = np.exp(1j * float(angle))
    z = point + (np.asarray(s21, dtype=complex) - point) * multiplier
    if s21_err is None:
        return z
    return z, transform_s21_error(s21_err, multiplier)


def invert_duffing_coordinate(y, anl):
    """Return the linear Duffing drive coordinate ``y0`` for a measured ``y``.

    The forward model solves

        4*y**3 - 4*y0*y**2 + y - y0 - anl = 0.

    Collecting the terms in ``y0`` gives a direct inverse. The inverse is
    single-valued even where the forward model is bistable because a measured
    point already selects one branch through its value of ``y``.
    """
    y = np.asarray(y, dtype=float)
    return (4.0 * y ** 3 + y - float(anl)) / (4.0 * y ** 2 + 1.0)


def mobius_coordinate(z_centered, radius, Ql):
    """Return the complex matched-scale coordinate of centered-circle IQ.

    For the linear notch model, the real part is the circle frequency
    coordinate and the negative imaginary part is ``Delta(1 / (2 * Qi))``.
    """
    z = np.asarray(z_centered, dtype=complex)
    return -1j * (float(radius) - z) / (
        2.0 * float(Ql) * (float(radius) + z))


def physical_frequency_fraction(x_circle, Ql, *, anl=0.0,
                                frequency_origin_fraction=0.0,
                                matched_dissipation=0.0, method='mobius'):
    """Convert a Mobius circle coordinate to fractional probe detuning."""
    x_circle = np.asarray(x_circle, dtype=float)
    if method == 'circle':
        return x_circle - float(frequency_origin_fraction)
    if method != 'mobius':
        raise ValueError("method must be 'mobius' or 'circle'")
    if float(anl) == 0.0:
        return x_circle - float(frequency_origin_fraction)
    gamma = 1.0 / float(Ql) + 2.0 * np.asarray(
        matched_dissipation, dtype=float)
    y = x_circle / gamma
    return gamma * invert_duffing_coordinate(y, anl)


def interpolate_complex_trace(frequencies, s21, reference_frequency):
    """Interpolate one complex sweep trace at ``reference_frequency``."""
    frequencies = np.asarray(frequencies, dtype=float)
    z = np.asarray(s21, dtype=complex)
    if frequencies.ndim != 1 or z.ndim != 1 or frequencies.size != z.size:
        raise ValueError("frequencies and s21 must be matching 1-D arrays")
    order = np.argsort(frequencies)
    f_sorted = frequencies[order]
    z_sorted = z[order]
    return complex(
        np.interp(reference_frequency, f_sorted, z_sorted.real)
        + 1j * np.interp(reference_frequency, f_sorted, z_sorted.imag)
    )


@dataclass(frozen=True)
class LinearizedResonatorCalibration:
    """Small-signal tangent/normal projection around one probe frequency."""
    reference_frequency: float
    reference_iq: complex
    gradient: complex

    @classmethod
    def from_sweep(cls, frequencies, s21, reference_frequency,
                   smooth_window_hz=1000):
        """Build a local linearized calibration from a complex sweep."""
        frequencies = np.asarray(frequencies, dtype=float)
        z = np.asarray(s21, dtype=complex)
        if frequencies.ndim != 1 or z.ndim != 1 or frequencies.size != z.size:
            raise ValueError("frequencies and s21 must be matching 1-D arrays")
        if frequencies.size < 2:
            raise ValueError("at least two sweep points are required")
        if smooth_window_hz:
            step_hz = float(np.median(np.abs(np.diff(frequencies))))
            window = max(3, int(float(smooth_window_hz) / step_hz))
            window = min(window, frequencies.size)
            if window % 2 == 0:
                window -= 1
            if window >= 3:
                z = (
                    signal.savgol_filter(z.real, window, 1)
                    + 1j * signal.savgol_filter(z.imag, window, 1)
                )
        gradient = np.gradient(z, frequencies)
        order = np.argsort(frequencies)
        f_sorted = frequencies[order]
        z_sorted = z[order]
        gradient_sorted = gradient[order]
        reference_iq = (
            np.interp(reference_frequency, f_sorted, z_sorted.real)
            + 1j * np.interp(reference_frequency, f_sorted, z_sorted.imag)
        )
        reference_gradient = (
            np.interp(reference_frequency, f_sorted, gradient_sorted.real)
            + 1j * np.interp(reference_frequency, f_sorted, gradient_sorted.imag)
        )
        return cls(
            float(reference_frequency),
            complex(reference_iq),
            complex(reference_gradient),
        )

    def convert_raw_iq(self, s21, *, fractional=True):
        """Project raw IQ displacement onto local frequency/dissipation axes."""
        z = np.asarray(s21, dtype=complex)
        delta = self.reference_iq - z
        divisor = abs(self.gradient) ** 2
        if divisor == 0.0:
            shape = np.broadcast_shapes(z.shape, ())
            return np.full(shape, np.nan), np.full(shape, np.nan)
        df = (
            delta.real * self.gradient.real
            + delta.imag * self.gradient.imag
        ) / divisor
        dd_hz = (
            delta.imag * self.gradient.real
            - delta.real * self.gradient.imag
        ) / divisor
        if fractional:
            return df / self.reference_frequency, dd_hz / self.reference_frequency
        return df, dd_hz


def linearized_frequency_and_dissipation(
        sweep_frequencies, sweep_s21, reference_frequency, timestream_s21,
        smooth_window_hz=1000, *, fractional=True, return_calibration=False):
    """Convert raw IQ with the historical local tangent/normal approximation."""
    calibration = LinearizedResonatorCalibration.from_sweep(
        sweep_frequencies,
        sweep_s21,
        reference_frequency,
        smooth_window_hz=smooth_window_hz,
    )
    frequency, dissipation = calibration.convert_raw_iq(
        timestream_s21, fractional=fractional)
    if return_calibration:
        return frequency, dissipation, calibration
    return frequency, dissipation


def calibration_for_tone(calibrations, tone_index):
    """Resolve one per-tone calibration from a mapping, sequence, or fit."""
    if calibrations is None:
        return None
    if isinstance(calibrations, Mapping):
        calibration = calibrations.get(
            tone_index, calibrations.get(f"{tone_index:04d}"))
    else:
        try:
            calibration = calibrations[tone_index]
        except (IndexError, KeyError, TypeError):
            calibration = None
    if calibration is not None and not isinstance(
            calibration, ResonatorCalibration):
        calibration = ResonatorCalibration.from_fit(calibration)
    return calibration


class ResonatorCalibration:
    """
    Cached calibration for vectorized IQ - frequency/dissipation conversion.

    Stores the phase-centering parameters (cable delay, gain phase, circle
    center, rotation) along with the resonator model parameters (fr, Ql)
    needed to convert between raw IQ and physical quantities.

    On the phase-centered circle (centered at origin, circle-coordinate
    reference on the positive real axis), the exact Möbius inversion gives
    a fractional frequency coordinate:

        x_circle = Re[-j * (r - z) / (2 * Ql * (r + z))]

    The calibration stores the value of ``x_circle`` at its reference
    frequency ``fr`` and subtracts it during conversion, so the returned
    ``df`` is zero at ``fr``. This matters for asymmetric resonators, where
    the fitted complex coupling shifts the circle coordinate slightly.

    The returned frequency coordinate is probe detuning relative to the fitted
    resonance:

        probe_detuning = f_probe - f_r

    It is positive when the probe is above resonance. The resonator detuning
    relative to a fixed probe is the opposite sign, ``f_r - f_probe``. For a
    linear notch model the probe detuning is recovered exactly, not just for
    small perturbations. For the nonlinear driven model used by
    :mod:`souk_readout_tools.fitting`, the Duffing relation is also inverted
    analytically after the circle inversion.

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
                 'gain_phase', 'rotation_angle', 'frequency_origin_fraction',
                 'anl', 'group_delay_cal', 'group_delay_reference_frequency',
                 '_deembed_scale', '_rotation_phasor')

    def __init__(self, fr, Ql, tau, center, radius, rotation_angle,
                 gain_amplitude=1.0, gain_phase=0.0,
                 frequency_origin_fraction=0.0, anl=0.0,
                 group_delay_cal=None, group_delay_reference_frequency=None):
        self.fr = float(fr)
        self.Ql = float(Ql)
        self.tau = float(tau)
        self.center = complex(center)
        self.radius = float(radius)
        self.gain_amplitude = float(gain_amplitude)
        self.gain_phase = float(gain_phase)
        self.rotation_angle = float(rotation_angle)
        self.frequency_origin_fraction = float(frequency_origin_fraction)
        self.anl = float(anl)
        self.group_delay_cal = group_delay_cal
        self.group_delay_reference_frequency = group_delay_reference_frequency
        self._deembed_scale = np.exp(-1j * self.gain_phase) / self.gain_amplitude
        self._rotation_phasor = np.exp(1j * rotation_angle)

    @classmethod
    def from_fit(cls, fit_result):
        """
        Build from a fitting.FitResult ``fit_result``.

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
        # axis, leaving the circle-coordinate reference on the positive axis.
        # FitResult.alpha is already in the public absolute-frequency phase
        # convention used with exp(-j*2*pi*f*tau), and is removed by
        # _deembed_scale before this rotation is applied.
        # With Qe = Qc*(1 + j*tan(phi)), the coupling term has phase -phi.
        rotation_angle = getattr(
            fit_result, 'phase_center_rotation_angle', fit_result.phi - np.pi)
        # The centered-circle Möbius coordinate includes the imaginary part of
        # the complex coupling. Subtract its value at the fitted fr so df=0
        # there, including for asymmetric resonators (phi != 0).
        anl = float(getattr(fit_result, 'anl', 0.0))
        if anl == 0.0:
            frequency_origin_fraction = np.imag(1.0 / fit_result.Qe) / 2.0
        else:
            # Preserve a zero-at-fr diagnostic circle coordinate. The default
            # physical conversion analytically inverts Duffing instead.
            from .fitting import s21_model
            z_fr = s21_model(
                np.array([fr]), fr, fit_result.Qi, fit_result.Qc,
                fit_result.phi, 1.0, 0.0, 0.0, anl=anl,
                sweep_direction=getattr(fit_result, 'sweep_direction', 'up'),
            )[0]
            z_fr = (z_fr - center) * np.exp(1j * rotation_angle)
            frequency_origin_fraction = np.real(
                -1j * (radius - z_fr) / (2.0 * Ql * (radius + z_fr)))
        return cls(fr, Ql, tau, center, radius, rotation_angle,
                   gain_amplitude=fit_result.a, gain_phase=fit_result.alpha,
                   frequency_origin_fraction=frequency_origin_fraction,
                   anl=anl)

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
        frequencies = np.asarray(frequencies, dtype=float)
        s21 = np.asarray(s21, dtype=complex)
        group_delay_reference_frequency = None
        if group_delay_cal is not None:
            group_delay_reference_frequency = float(frequencies.ravel()[0])
            s21_no_delay, _ = remove_group_delay(
                frequencies, s21, group_delay_cal,
                reference_frequency=group_delay_reference_frequency)
            tau = 0.0
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

        order = np.argsort(frequencies)
        z_fr = (
            np.interp(fr, np.asarray(frequencies)[order], s21_no_delay.real[order])
            + 1j * np.interp(fr, np.asarray(frequencies)[order], s21_no_delay.imag[order])
        )
        z_fr = ((z_fr - pc_params['center'])
                * np.exp(1j * pc_params['rotation_angle']))
        r = pc_params['radius']
        frequency_origin_fraction = np.real(
            -1j * (r - z_fr) / (2.0 * Ql * (r + z_fr)))

        return cls(fr, Ql, tau, pc_params['center'], r,
                   pc_params['rotation_angle'],
                   frequency_origin_fraction=frequency_origin_fraction,
                   group_delay_cal=group_delay_cal,
                   group_delay_reference_frequency=(
                       group_delay_reference_frequency))

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

    @property
    def deembedding_calibration(self):
        """Return the raw-IQ deembedding calibration."""
        return DeembeddingCalibration(
            scale=self._deembed_scale,
            tau=self.tau,
            group_delay_cal=self.group_delay_cal,
            group_delay_reference_frequency=self.group_delay_reference_frequency,
        )

    @property
    def phase_center_calibration(self):
        """Return the circle-centering and rotation calibration."""
        return PhaseCenterCalibration(
            self.center, self.radius, self.rotation_angle)

    # -- Phase centering -----------------------------------------------------

    def deembed_raw_iq(self, frequency, s21, s21_err=None):
        """Remove the fitted frequency-dependent environment from raw IQ."""
        return self.deembedding_calibration.apply(
            frequency, s21, s21_err=s21_err)

    def phase_center_iq(self, s21, s21_err=None):
        """Subtract the resonance-circle center and rotate its reference axis."""
        return self.phase_center_calibration.apply(s21, s21_err=s21_err)

    def transform_raw_iq(self, frequency, s21, s21_err=None):
        """Deembed and phase-center raw IQ measured at ``frequency``."""
        if s21_err is None:
            return self.phase_center_iq(self.deembed_raw_iq(frequency, s21))
        z, z_err = self.deembed_raw_iq(frequency, s21, s21_err=s21_err)
        return self.phase_center_iq(z, s21_err=z_err)

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
        return self.transform_raw_iq(frequencies, s21, s21_err=s21_err)

    def deembed_timestream(self, s21, s21_err=None, *, frequency=None):
        """
        Phase-center timestream data measured at an explicit tone frequency.

        Args:
            s21: Complex array of timestream IQ samples.
            frequency: Tone frequency (Hz).
            s21_err: Optional S21 uncertainty. A complex array is interpreted
                as real=sigma_I and imag=sigma_Q.

        Returns:
            If ``s21_err`` is None: complex array, phase-centered.
            Otherwise: ``(z_centered, z_err_centered)``.
        """
        if frequency is None:
            raise ValueError(
                "frequency is required when phase-centering raw timestream IQ")
        return self.transform_raw_iq(frequency, s21, s21_err=s21_err)

    # -- Conversions ---------------------------------------------------------

    def to_phase_amplitude(self, z_centered):
        """
        Convert phase-centered IQ to (phase, normalised amplitude).

        Args:
            z_centered: Complex array on the phase-centered circle
                        (output of deembed_sweep or deembed_timestream).

        Returns:
            phase: Angle on the resonance circle (rad). Zero at the
                   positive-real circle point, ±pi off resonance.
            amplitude: |z| / radius. Unity on the model circle. Deviations
                       give a radial loss diagnostic, not a normalized
                       ``Delta(1 / (2 * Qi))`` quadrature.
        """
        phase = np.angle(z_centered)
        amplitude = np.abs(z_centered) / self.radius
        return phase, amplitude

    def circle_coordinate_fraction(self, z_centered):
        """Return the Möbius coordinate on the centered resonance circle."""
        return self.complex_mobius_coordinate(z_centered).real

    def complex_mobius_coordinate(self, z_centered):
        """Return probe-side circle and matched-loss coordinates in one value."""
        return mobius_coordinate(z_centered, self.radius, self.Ql)

    def radial_loss_proxy(self, z_centered):
        """Return signed fractional radius change, ``abs(z) / radius - 1``."""
        return np.abs(np.asarray(z_centered, dtype=complex)) / self.radius - 1.0

    def convert_centered_iq(self, z_centered, *, method='mobius'):
        """
        Convert phase-centered IQ to probe-detuning and loss coordinates.

        ``method='mobius'`` is the default analysis path. It uses exact Möbius
        inversion of the resonance circle and also inverts the Duffing relation
        analytically when the calibration came from a nonlinear fit.

        ``method='circle'`` is a diagnostic path. It returns the effective
        circle coordinate, zeroed at the fitted ``fr``, without the Duffing
        inverse. Its second result is the raw radial loss proxy rather than a
        matched physical loss coordinate.

        On the phase-centered circle the model is:
            z = r * (1 - 2j*Ql*x_circle) / (1 + 2j*Ql*x_circle)

        Inverting:
            x_circle = Re[-j * (r - z) / (2*Ql * (r + z))]

        For a linear fit, or ``method='circle'``:
            df = (x_circle - x_circle_at_fr) * fr

        For a Duffing fit, the default method analytically maps ``x_circle``
        back to probe detuning before scaling by ``fr``.

        Args:
            z_centered: Complex scalar or array on the phase-centered circle,
                usually returned by :meth:`transform_raw_iq`.
            method: ``'mobius'`` (default) or ``'circle'``. Use ``'mobius'``
                for fitted frequency/dissipation analysis. Use ``'circle'``
                only to inspect the intermediate circle coordinate and radial
                loss proxy.

        Returns:
            df: Probe detuning relative to the fitted resonance,
                ``f_probe - fr`` (Hz). Resonator detuning relative to the probe
                has the opposite sign.
            dd: ``Delta(1 / (2 * Qi))`` for ``method='mobius'``. For
                ``method='circle'``, the diagnostic radial circle-departure
                proxy ``|z| / r - 1`` instead.
        """
        z = np.asarray(z_centered, dtype=complex)
        coordinate = self.complex_mobius_coordinate(z)
        matched_dissipation = -coordinate.imag
        x = physical_frequency_fraction(
            coordinate.real,
            self.Ql,
            anl=self.anl,
            frequency_origin_fraction=self.frequency_origin_fraction,
            matched_dissipation=matched_dissipation,
            method=method,
        )
        df = x * self.fr
        dd = (
            self.radial_loss_proxy(z)
            if method == 'circle' else matched_dissipation
        )
        return df, dd

    def to_frequency_dissipation(self, z_centered):
        """Compatibility wrapper for :meth:`convert_centered_iq`."""
        return self.convert_centered_iq(z_centered)

    def convert_raw_iq(self, frequency, s21, *, method='mobius'):
        """
        Convert measured IQ to fitted probe-detuning and loss coordinates.

        Both fitted methods return a probe-side frequency coordinate relative
        to the fitted resonance: positive means ``f_probe > fr``. The default
        returns ``(probe_detuning_hz, Delta(1 / (2 * Qi)))``. The diagnostic
        ``method='circle'`` path returns
        ``(circle_probe_detuning_hz, abs(z_centered) / radius - 1)``.

        The input IQ is deembedded and phase-centered internally using the
        calibration derived from the resonance fit.

        Args:
            frequency: Probe frequency in Hz. Pass a scalar for a fixed-tone
                timestream, or an array broadcastable with ``s21`` when samples
                were measured at different probe frequencies, such as a sweep
                or modulation cycle.
            s21: Raw measured complex IQ sample or array, ``I + 1j * Q``, before
                deembedding or phase centering. Its shape must broadcast with
                ``frequency``.
            method: Fitted conversion method:

                - ``'mobius'`` (default): exact fitted-model conversion. Returns
                  probe detuning ``f_probe - fr`` in Hz and matched dissipation
                  ``Delta(1 / (2 * Qi))``. For a nonlinear calibration, the
                  fitted Duffing relation is inverted analytically.
                - ``'circle'``: diagnostic intermediate conversion. Returns
                  the effective circle probe-detuning coordinate in Hz and the
                  raw radial loss proxy ``abs(z_centered) / radius - 1``. It
                  does not apply the Duffing inverse.

                The historical local tangent/normal approximation is not a
                value of ``method``. Use
                :func:`linearized_frequency_and_dissipation` for that path.

        Returns:
            probe_detuning_hz: Scalar or array with the broadcast input shape.
                For both methods, positive values place the probe above the
                fitted resonance. Resonator detuning relative to a fixed probe
                has the opposite sign.
            loss: Scalar or array with the broadcast input shape. Its meaning
                depends on ``method`` as described above.

        See Also:
            convert_referenced_raw_iq: Convert fixed-tone IQ changes into the
                reference-subtracted resonator-motion convention commonly used
                for noise timestreams.
        """
        return self.convert_centered_iq(
            self.transform_raw_iq(frequency, s21), method=method)

    def convert_referenced_raw_iq(self, frequency, s21, reference_s21, *,
                                  method='mobius'):
        """Convert fixed-tone IQ motion relative to a reference IQ sample.

        ``convert_raw_iq`` returns absolute probe detuning relative to the
        fitted resonance, ``f_probe - f_r``. Resonator detuning relative to a
        fixed probe is ``f_r - f_probe`` and has the opposite sign. This method
        subtracts a reference IQ sample and returns resonator motion plus the
        corresponding loss change.

        For ``method='circle'``, the loss result is ``Delta rho`` rather than
        the absolute radial proxy ``rho = abs(z_centered) / radius - 1``.

        Args:
            frequency: Fixed probe frequency in Hz.
            s21: Raw measured complex IQ sample or array.
            reference_s21: Raw complex IQ reference sample at ``frequency``.
                A sweep interpolated at the probe tone is a suitable reference.
            method: ``'mobius'`` (default) or diagnostic ``'circle'``; see
                :meth:`convert_raw_iq` for the returned loss coordinate.

        Returns:
            resonator_motion_hz: Reference-subtracted resonator motion in Hz,
                with the familiar resonator-side sign.
            delta_loss: Reference-subtracted loss coordinate. For ``'mobius'``
                this is ``Delta(1 / (2 * Qi))``; for ``'circle'`` it is
                ``Delta rho``.
        """
        df, dd = self.convert_raw_iq(frequency, s21, method=method)
        reference_df, reference_dd = self.convert_raw_iq(
            frequency, reference_s21, method=method)
        return reference_df - df, dd - reference_dd

    # -- Fixed-tone converter ------------------------------------------------

    def tone_converter(self, f_tone, *, method='mobius'):
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
        return ToneConverter(self, f_tone, method=method)


class ToneConverter:
    """
    Optimised IQ to probe-detuning/loss coordinates for one fixed tone.

    Pre-combines cable delay removal and phase centering into a single
    complex multiply and add, so each call is:
        z_d = z * _multiply - _offset      (1 complex mul + 1 complex sub)
        x   = Re[-j*(r - z_d) / (2*Ql*(r + z_d))]  (Möbius inversion)
        df  = (x - x_at_fr) * fr

    Construct via ResonatorCalibration.tone_converter(f_tone).
    """

    __slots__ = ('_multiply', '_offset', '_fr', '_Ql', '_radius',
                 '_frequency_origin_fraction', '_anl', '_method')

    def __init__(self, cal, f_tone, method='mobius'):
        # Combine cable delay, gain removal, and rotation into one phasor.
        self._multiply = (
            cal.deembedding_calibration.multiplier(f_tone)
            * cal._rotation_phasor)
        self._offset = cal.center * cal._rotation_phasor
        self._fr = cal.fr
        self._Ql = cal.Ql
        self._radius = cal.radius
        self._frequency_origin_fraction = cal.frequency_origin_fraction
        self._anl = cal.anl
        if method not in ('mobius', 'circle'):
            raise ValueError("method must be 'mobius' or 'circle'")
        self._method = method

    def __call__(self, z):
        """
        Convert raw IQ to (df, dd).

        Args:
            z: Raw complex IQ data (scalar or array).

        Returns:
            df: Probe detuning relative to the fitted resonance,
                ``f_probe - fr`` (Hz). Resonator detuning relative to the probe
                has the opposite sign.
            dd: ``Delta(1 / (2 * Qi))`` for ``method='mobius'`` or the radial
                circle-departure proxy for ``method='circle'``.
        """
        z_d = z * self._multiply - self._offset
        r = self._radius
        coordinate = mobius_coordinate(z_d, r, self._Ql)
        matched_dissipation = -coordinate.imag
        x = physical_frequency_fraction(
            coordinate.real,
            self._Ql,
            anl=self._anl,
            frequency_origin_fraction=self._frequency_origin_fraction,
            matched_dissipation=matched_dissipation,
            method=self._method,
        )
        df = x * self._fr
        dd = (
            np.abs(z_d) / r - 1.0
            if self._method == 'circle' else matched_dissipation
        )
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
