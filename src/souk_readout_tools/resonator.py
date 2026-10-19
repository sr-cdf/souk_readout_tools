"""
Resonator data transforms and the package's one I/Q -> physics conversion.

Every consumer -- timeseries, plotting, the live viewer, the demodulators,
the DAC streamer and the server tracker -- takes its conversions from here,
so a quantity has one name, one sign and one definition wherever it appears.
``doc/conversions.md`` has the maths and a worked example for saved data.

============================================  =================================
I want to...                                  Use
============================================  =================================
see the list of named quantities, units       ``QUANTITIES``, ``canonical_quantity``
convert a parsed capture to named quantities  ``convert_samples``
convert one tone's I/Q (plain arrays)         ``prepare_conversion`` + ``convert_iq``
express a shift in Hz/fractional/linewidths   ``express``
move raw I/Q into a de-embedded/centred basis ``basis_coefficients``, ``to_basis``
phase slope/curvature from modulation points  ``phase_derivatives``
detuning (resonance - tone) from those        ``detuning_from_derivatives``
where the resonance is (phase inflection)     ``operating_point_hz``
build/save/load a calibration record          ``calibration_record``,
                                              ``save_calibrations``,
                                              ``load_calibration_record``
a fit's frequency error on its own sweep      ``sweep_frequency_correction``
check calibrations, pick the good tones       ``calibration_quality``
fit a sweep straight to calibrations          ``calibrations_from_sweep``
the calibration a recording carries           ``calibration_from_data``
toggle the filterbank readout correction      ``with_readout_correction``,
                                              ``readout_correction_factors``,
                                              ``apply_readout_correction``
match RX gain between sweep and timestream    ``rx_gain_ratio``,
                                              ``recorded_rx_gain_ratio``
============================================  =================================

Conventions, everywhere:

- Shifts are **resonator-side**: positive when the resonance moves up.
- **Detuning is resonance minus tone** (positive: resonance above the tone).
  For demodulated modulation data the tone is the modulation centre, which
  every sample is referred to.
- Dissipation is the matched coordinate ``Delta(1 / (2 Qi))``: the factor 1/2
  is already applied, and it is positive for more loss.
- The **resonance** is the operational one: the inflection of the de-embedded,
  circle-centred phase (zero curvature, steepest slope). For a linear
  resonator that is the fitted ``fr``; a driven (Duffing) one sits at
  ``operating_point_hz``, displaced from ``fr``.

The transforms underneath fall in two independent categories:

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

import copy
import functools
import json
import time
import warnings
from collections.abc import Mapping
from dataclasses import dataclass

import numpy as np


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

    left_width = abs(fr - f_left) if np.isfinite(f_left) else np.nan
    right_width = abs(f_right - fr) if np.isfinite(f_right) else np.nan
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


def project_iq_tangent_normal(reference_iq, gradient, s21):
    """Project an IQ displacement onto local tangent/normal axes.

    Given a reference point ``reference_iq``, the local complex sweep gradient
    ``dS21/df`` and a measured ``s21``, return the displacement resolved along
    the tangent (local frequency, ``df`` in Hz) and normal (matched-loss,
    ``dd`` in Hz) directions. Fully vectorized; div-by-zero where the gradient
    vanishes yields NaN per element.
    """
    delta = np.asarray(reference_iq, dtype=complex) - np.asarray(s21, dtype=complex)
    gradient = np.asarray(gradient, dtype=complex)
    divisor = np.abs(gradient) ** 2
    with np.errstate(divide='ignore', invalid='ignore'):
        df = (delta.real * gradient.real + delta.imag * gradient.imag) / divisor
        dd = (delta.imag * gradient.real - delta.real * gradient.imag) / divisor
    return df, dd


def smooth_sweep(frequencies, s21, smooth_window_hz=1000):
    """Savitzky-Golay-smooth a complex sweep over ``smooth_window_hz``.

    First order, so a straight line passes unchanged; the window is the
    number of sweep steps in ``smooth_window_hz`` (odd, at least 3). A falsy
    window, or one covering fewer than three points, returns the input.
    """
    z = np.asarray(s21, dtype=complex)
    if not smooth_window_hz or z.size < 3:
        return z
    step_hz = float(np.median(np.abs(np.diff(np.asarray(frequencies, dtype=float)))))
    if not step_hz > 0.0:
        return z
    window = min(max(3, int(float(smooth_window_hz) / step_hz)), z.size)
    if window % 2 == 0:
        window -= 1
    if window < 3:
        return z
    from scipy import signal
    return (signal.savgol_filter(z.real, window, 1)
            + 1j * signal.savgol_filter(z.imag, window, 1))


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
        z = smooth_sweep(frequencies, z, smooth_window_hz)
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
        df, dd_hz = project_iq_tangent_normal(self.reference_iq, self.gradient, s21)
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


class PhaseLookupCalibration:
    """Fit-free frequency readout by inverting a sweep's phase-vs-frequency curve.

    Where :class:`LinearizedResonatorCalibration` linearises about one point (so
    it saturates for large excursions), this uses the *whole* measured sweep as
    a lookup table. The loop is moved into the centred, rotated basis, where
    the phase ``theta(f)`` is monotonic through resonance, and the inverse
    ``f(theta)`` maps a measured phase straight back to the probe frequency at
    which the swept resonator showed that phase. That is exact across the
    swept span -- it never linearises -- so it follows large signals and is the
    natural fallback when a tone will not fit but its sweep still resolves a
    resonance.

    Pass ``calibration`` (a :class:`ResonatorCalibration`) to :meth:`from_sweep`
    to centre in the same de-embedded frame as every other conversion; without
    one the loop is centred with a Kasa circle fit of the raw sweep, which is
    model-free but a slightly different frame (no cable-delay removal).

    :meth:`convert_raw_iq` follows the package convention: a resonator-side
    shift (positive when the resonance moves up) and the matched dissipation
    ``Delta(1 / (2 Qi))``, both relative to the sweep at ``reference_frequency``.
    The dissipation compares the measured radius with the swept loop's radius at
    the same phase, scaled by the local phase slope, so it too holds past the
    linear range. Phases beyond the swept range come back NaN rather than
    clamped: keep the sweep wider than the timestream's frequency motion.

    **Noise on the steep part matters.** The inverse is read out right where
    ``theta(f)`` is steepest, so ``from_sweep`` smooths the complex sweep
    (Savitzky-Golay over ``smooth_window_hz``); raise it for a noisy sweep.
    """

    def __init__(self, reference_frequency, basis_multiply, basis_offset, sign,
                 phase_lookup, frequency_lookup, radius_lookup, slope_lookup,
                 reference_phase):
        self.reference_frequency = float(reference_frequency)
        # z_c = z * basis_multiply + basis_offset at the reference frequency.
        self.basis_multiply = complex(basis_multiply)
        self.basis_offset = complex(basis_offset)
        # +1 or -1: whether the centred phase increases or decreases with
        # frequency. Folded into the stored lookup so it ascends.
        self.sign = float(sign)
        # Strictly ascending (folded) phase, and at each phase the sweep
        # frequency, the loop radius |z_c| and the folded slope dtheta/df.
        self.phase_lookup = np.asarray(phase_lookup, dtype=float)
        self.frequency_lookup = np.asarray(frequency_lookup, dtype=float)
        self.radius_lookup = np.asarray(radius_lookup, dtype=float)
        self.slope_lookup = np.asarray(slope_lookup, dtype=float)
        # Folded phase at the parked tone; measured angles are folded onto the
        # branch nearest this so the inversion stays single-valued.
        self.reference_phase = float(reference_phase)

    @classmethod
    def from_sweep(cls, frequencies, s21, reference_frequency,
                   smooth_window_hz=1000, min_phase_progress=0.5,
                   calibration=None):
        """Build a phase lookup from one complex sweep trace.

        Pass a **targeted, single-resonance** sweep (the same trace you would
        fit). ``reference_frequency`` is the parked-tone frequency the timestream
        is read out at. ``smooth_window_hz`` sets the Savitzky-Golay smoothing
        (0/None disables it). ``min_phase_progress`` is the minimum ratio of net
        phase travel to total variation required to accept the loop as
        invertible (~1 for a clean resonance, ~0 for noise). ``calibration``
        centres the loop in its de-embedded frame (see the class docstring).
        """
        frequencies = np.asarray(frequencies, dtype=float).ravel()
        z = np.asarray(s21, dtype=complex).ravel()
        if frequencies.size != z.size:
            raise ValueError("frequencies and s21 must be matching 1-D arrays")
        if frequencies.size < 5:
            raise ValueError("at least five sweep points are required")
        order = np.argsort(frequencies)
        f = frequencies[order]
        z = z[order]
        # A collapsed sweep (frequency never advances for most points) has no
        # invertible phase curve; reject it up front with the same criterion the
        # fitter uses, rather than letting circle-fit noise sneak through.
        if np.median(np.diff(f)) <= 0.0:
            raise ValueError(
                "degenerate sweep: frequency did not advance across the majority "
                "of points; cannot build a phase lookup")
        # Smooth before taking angles: noise on the steep part of theta(f) is
        # what limits a lookup inverter.
        z = smooth_sweep(f, z, smooth_window_hz)
        if calibration is not None:
            z_centered = to_basis(z, calibration, f, 'centered_rotated')
            multiply, offset = basis_coefficients(
                calibration, float(reference_frequency), 'centered_rotated')
        else:
            # Model-free: centre (Kasa fit) then rotate off-resonance to +/-pi.
            _, center, _ = center_circle(z)
            _, rotation_angle = rotate_to_real_axis(z - center)
            multiply = np.exp(1j * rotation_angle)
            offset = -center * multiply
            z_centered = z * multiply + offset
        theta_f = np.unwrap(np.angle(z_centered))  # phase in frequency order
        # The inverse f(theta) only exists if theta advances monotonically
        # through resonance. Judge that by net travel vs total variation (~1 for
        # a clean loop, ~0 for noise); a backward-step count would wrongly reject
        # the flat, noise-dominated off-resonance tails of a real sweep.
        dtheta = np.diff(theta_f)
        total_variation = float(np.sum(np.abs(dtheta)))
        net_travel = float(abs(np.sum(dtheta)))
        if total_variation <= 0.0 or net_travel < min_phase_progress * total_variation:
            raise ValueError(
                "sweep phase is not monotonic through resonance; cannot build a "
                "phase lookup (loop too noisy or no resolved resonance)")
        sign = 1.0 if np.sum(dtheta) >= 0.0 else -1.0
        theta_f = sign * theta_f
        slope_f = np.gradient(theta_f, f)
        reference_phase = float(np.interp(float(reference_frequency), f, theta_f))
        # Sort by ascending phase and drop ties so np.interp stays single-valued.
        p_order = np.argsort(theta_f)
        phase_lookup = theta_f[p_order]
        keep = np.concatenate(([True], np.diff(phase_lookup) > 0.0))
        return cls(
            reference_frequency, complex(np.squeeze(multiply)),
            complex(np.squeeze(offset)), sign, phase_lookup[keep],
            f[p_order][keep], np.abs(z_centered)[p_order][keep],
            slope_f[p_order][keep], reference_phase,
        )

    def convert_raw_iq(self, s21, *, fractional=True):
        """Convert raw timestream IQ at ``reference_frequency`` to shift and loss.

        Returns ``(resonance_shift, dissipation)`` relative to the sweep at
        ``reference_frequency``: the resonator-side shift in Hz and the matched
        dissipation in Hz (``fr * Delta(1 / (2 Qi))``), or both divided by
        ``reference_frequency`` when ``fractional`` (the default), which makes
        the second ``Delta(1 / (2 Qi))`` itself. NaN beyond the swept range.
        """
        z_centered = np.asarray(s21, dtype=complex) * self.basis_multiply + self.basis_offset
        theta = self.sign * np.angle(z_centered)
        # Fold each measured angle onto the 2*pi branch nearest the parked
        # reference phase. This tracks the whole resonance loop and, unlike
        # folding into a fixed range, stays correct when a cable-delay ramp makes
        # the sweep span exceed 2*pi.
        theta = theta + 2.0 * np.pi * np.round(
            (self.reference_phase - theta) / (2.0 * np.pi))
        inside = ((theta >= self.phase_lookup[0])
                  & (theta <= self.phase_lookup[-1]))
        f_equiv = np.interp(theta, self.phase_lookup, self.frequency_lookup)
        # The resonator moved by +d if the tone now shows the phase the swept
        # resonator showed d below the tone.
        shift = self.reference_frequency - f_equiv
        radius = np.interp(theta, self.phase_lookup, self.radius_lookup)
        slope = self.sign * np.interp(theta, self.phase_lookup, self.slope_lookup)
        with np.errstate(divide='ignore', invalid='ignore'):
            dissipation = np.log(np.abs(z_centered) / radius) / slope
        shift = np.where(inside, shift, np.nan)
        dissipation = np.where(inside, dissipation, np.nan)
        if fractional:
            return shift / self.reference_frequency, dissipation / self.reference_frequency
        return shift, dissipation


def phase_lookup_frequency_and_dissipation(
        sweep_frequencies, sweep_s21, reference_frequency, timestream_s21,
        smooth_window_hz=1000, *, fractional=True, return_calibration=False,
        calibration=None):
    """Convert raw IQ by inverting the sweep's phase curve (fit-free fallback).

    Mirrors :func:`linearized_frequency_and_dissipation` but inverts the whole
    resonance loop instead of linearising about the parked tone, so it follows
    large signals far past where the linear readout saturates. Returns
    ``(resonance_shift, dissipation)``; see :class:`PhaseLookupCalibration`.
    """
    lookup = PhaseLookupCalibration.from_sweep(
        sweep_frequencies,
        sweep_s21,
        reference_frequency,
        smooth_window_hz=smooth_window_hz,
        calibration=calibration,
    )
    frequency, dissipation = lookup.convert_raw_iq(
        timestream_s21, fractional=fractional)
    if return_calibration:
        return frequency, dissipation, lookup
    return frequency, dissipation


def calibration_for_tone(calibrations, tone_index):
    """Resolve one tone's calibration from a record, mapping, sequence or fit.

    ``calibrations`` may be a calibration record (:func:`calibration_record`),
    a ``{tone_index: ...}`` mapping (``'0003'``-style keys too) or a sequence
    indexed by tone. Fit results are wrapped in :class:`ResonatorCalibration`;
    any other calibration (linearized, phase lookup) comes back unchanged.
    Returns ``None`` when the tone has none.
    """
    if calibrations is None:
        return None
    if is_calibration_record(calibrations):
        tone_index = int(tone_index)
        if tone_index not in [int(t) for t in calibrations['tone_indices']]:
            return None
        return calibrations_from_record(calibrations, tones=[tone_index])[tone_index]
    if isinstance(calibrations, Mapping):
        calibration = calibrations.get(
            tone_index, calibrations.get(f"{tone_index:04d}"))
    else:
        try:
            calibration = calibrations[tone_index]
        except (IndexError, KeyError, TypeError):
            calibration = None
    if calibration is not None and _is_fit_result(calibration):
        calibration = ResonatorCalibration.from_fit(calibration)
    return calibration


def _is_fit_result(obj):
    """True for a fitting.FitResult (or anything shaped like one)."""
    return hasattr(obj, 'iq_center_deembed') and hasattr(obj, 'alpha')


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

    For readout at a fixed tone, use :func:`prepare_conversion` once and
    :func:`convert_iq` per sample: the conversion contract's signs, RX gain
    handling and sweep reference.
    """

    __slots__ = ('fr', 'Ql', 'Qi', 'tau', 'center', 'radius', 'gain_amplitude',
                 'gain_phase', 'rotation_angle', 'frequency_origin_fraction',
                 'anl', 'group_delay_cal', 'group_delay_reference_frequency',
                 'frequency_correction', '_deembed_scale', '_rotation_phasor')

    def __init__(self, fr, Ql, tau, center, radius, rotation_angle,
                 gain_amplitude=1.0, gain_phase=0.0,
                 frequency_origin_fraction=0.0, anl=0.0,
                 group_delay_cal=None, group_delay_reference_frequency=None,
                 Qi=np.nan, frequency_correction=None):
        self.fr = float(fr)
        self.Ql = float(Ql)
        # Internal Q of the calibration state: only for absolute loss
        # (internal_loss); NaN when the calibration did not come from a fit.
        self.Qi = float(Qi)
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
        # The sweep table (sweep_frequency_correction) as (knots, correction),
        # both probe detunings from fr in Hz; None reads the model alone.
        self.frequency_correction = (
            None if frequency_correction is None
            else tuple(np.asarray(v, dtype=float) for v in frequency_correction))
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
        # Offset that puts df = 0 at the fitted fr. The linear model's
        # denominator carries only the real part of 1/Qe, so its circle
        # coordinate is already zero at fr for any phi and no offset is needed;
        # only the driven model displaces it.
        anl = float(getattr(fit_result, 'anl', 0.0))
        if anl == 0.0:
            frequency_origin_fraction = 0.0
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
                   anl=anl, Qi=getattr(fit_result, 'Qi', np.nan))

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

    def correct_probe_detuning(self, probe_detuning_hz):
        """Move the model's probe detuning ``f - fr`` (Hz) onto the frequency
        axis of the calibration's own sweep, through its sweep table
        (:func:`sweep_frequency_correction`). Unchanged without a table; past
        the table's ends its end corrections hold."""
        if self.frequency_correction is None:
            return probe_detuning_hz
        knots, correction = self.frequency_correction
        return probe_detuning_hz + np.interp(probe_detuning_hz, knots, correction)

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

        A calibration with a sweep table (a record made with its sweep) then
        reads ``df`` off the sweep's own frequency axis
        (:meth:`correct_probe_detuning`): the fit's shape error near the
        operating point, about 2% of the detuning on real resonators, is taken
        out. The loss coordinate keeps the model's scale.

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
        if method == 'mobius':
            df = self.correct_probe_detuning(df)
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


# ===========================================================================
# The conversion contract
# ===========================================================================
#
# Everything below is the one place raw I/Q becomes a named physical
# quantity. The signs and definitions are in the module docstring; the maths
# is in doc/conversions.md.

# ---------------------------------------------------------------------------
# Quantities
# ---------------------------------------------------------------------------

QUANTITIES = {
    # I/Q views, in whichever basis was asked for (raw unless stated).
    'i': ('I', 'counts'),
    'q': ('Q', 'counts'),
    'amplitude': ('Amplitude', 'counts'),
    'log_amplitude': ('Log amplitude', 'dB'),
    'phase': ('Phase', 'rad'),
    'unwrapped_phase': ('Unwrapped phase', 'rad'),
    # The same views in a fixed basis, whatever basis was asked for.
    'centered_phase': ('Centred phase', 'rad'),
    'centered_rotated_phase': ('Centred, rotated phase', 'rad'),
    'centered_radius': ('Centred radius, |z_c| / r', '1'),
    # Resonator response: need a sweep or calibration, or demodulated data.
    'resonance_shift_hz': ('Resonance shift', 'Hz'),
    'fractional_resonance_shift': ('Fractional resonance shift', '1'),
    'resonance_shift_linewidths': ('Resonance shift', 'linewidths'),
    'resonance_frequency_hz': ('Resonance frequency', 'Hz'),
    'detuning_hz': ('Detuning, resonance - tone', 'Hz'),
    'fractional_detuning': ('Detuning / tone frequency', '1'),
    'detuning_linewidths': ('Detuning, resonance - tone', 'linewidths'),
    'center_detuning_hz': ('Detuning, resonance - modulation centre', 'Hz'),
    'center_detuning_linewidths': ('Detuning, resonance - modulation centre',
                                   'linewidths'),
    'matched_dissipation_shift': ('Dissipation, Δ(1/2Qi)', '1'),
    'matched_dissipation_hz': ('Dissipation, fr·Δ(1/2Qi)', 'Hz'),
    'matched_dissipation_linewidths': ('Dissipation, Qr·Δ(1/2Qi)', '1'),
    'internal_loss': ('Internal loss, 1/Qi', '1'),
    'internal_loss_shift': ('Internal loss shift, Δ(1/Qi)', '1'),
    'fractional_internal_loss_shift': ('Internal loss shift, Qi·Δ(1/Qi)', '1'),
    # Modulation diagnostics, one estimate per cycle.
    'dphi_df': ('Phase slope', 'rad/Hz'),
    'd2phi_df2': ('Phase curvature', 'rad/Hz²'),
    'curvature_over_slope': ('Phase curvature / slope', '1/Hz'),
}

# Earlier and shorter names, resolved by canonical_quantity.
QUANTITY_ALIASES = {
    'amp': 'amplitude', 'mag': 'amplitude', 'magnitude': 'amplitude',
    'logmag': 'log_amplitude', 'log_mag': 'log_amplitude',
    'log_magnitude': 'log_amplitude',
    'centred_phase': 'centered_phase',
    'centred_rotated_phase': 'centered_rotated_phase',
    'centred_radius': 'centered_radius', 'radius': 'centered_radius',
    'frequency': 'resonance_shift_hz', 'freq': 'resonance_shift_hz',
    'frequency_shift': 'resonance_shift_hz', 'resonance_shift': 'resonance_shift_hz',
    'fractional_frequency': 'fractional_resonance_shift',
    'fractional': 'fractional_resonance_shift',
    'frac_frequency': 'fractional_resonance_shift',
    'frequency_linewidths': 'resonance_shift_linewidths',
    'freq_lw': 'resonance_shift_linewidths',
    'dissipation': 'matched_dissipation_shift',
    'diss': 'matched_dissipation_shift',
    'dissipation_linewidths': 'matched_dissipation_linewidths',
    'diss_lw': 'matched_dissipation_linewidths',
    'detuning': 'detuning_hz',
    'responsivity': 'dphi_df', 'phase_slope': 'dphi_df',
    'curvature': 'd2phi_df2', 'phase_curvature': 'd2phi_df2',
}

_IQ_VIEWS = ('i', 'q', 'amplitude', 'log_amplitude', 'phase', 'unwrapped_phase')
_FIXED_BASIS_VIEWS = {'centered_phase': 'centered',
                      'centered_rotated_phase': 'centered_rotated',
                      'centered_radius': 'centered'}
_DEMOD_ONLY = ('center_detuning_hz', 'center_detuning_linewidths',
               'dphi_df', 'd2phi_df2', 'curvature_over_slope')


def canonical_quantity(name):
    """Resolve a quantity name or alias to its ``QUANTITIES`` key.

    Case, spaces and hyphens are forgiven (``'Centred phase'`` works).

    Examples
    --------
    >>> canonical_quantity('frequency_shift')
    'resonance_shift_hz'
    >>> QUANTITIES[canonical_quantity('diss')]
    ('Dissipation, Δ(1/2Qi)', '1')
    """
    if not isinstance(name, str):
        raise ValueError('quantity names must be strings')
    key = name.strip().lower().replace(' ', '_').replace('-', '_')
    key = QUANTITY_ALIASES.get(key, key)
    if key not in QUANTITIES:
        raise ValueError(f'Unknown quantity {name!r}; choose from '
                         + ', '.join(QUANTITIES))
    return key


# ---------------------------------------------------------------------------
# Where the resonance is
# ---------------------------------------------------------------------------

# Duffing bifurcation threshold of the fitted model: above it the swept
# response is bistable and the steepest point is a cliff edge.
BIFURCATION_ANL = 4.0 * np.sqrt(3.0) / 9.0


def operating_point_hz(calibration, n_grid=20001, y_max=8.0):
    """Frequency (Hz) of the operational resonance: the phase inflection.

    The steepest point of the de-embedded, centred response -- maximum
    ``|dS21/df|``, zero phase curvature -- which is the best signal-to-noise
    point for a frequency readout and the point tracking steers to.

    Linear (``anl == 0``): ``fr`` itself for any asymmetry ``phi`` (the fitted
    model carries only ``Re(1/Qe)`` in its denominator). Driven (``anl > 0``):
    displaced from ``fr`` by the Duffing detuning plus the bistable skew. The
    driven model is parametrised by the smooth Duffing coordinate ``y`` rather
    than frequency, because the forward solver's branch selection jumps in
    ``f``; on the circle ``|dS21/dy| = 2 / (1 + 4 y**2)`` and ``df/dy`` comes
    from the rational inverse :func:`invert_duffing_coordinate`. Past
    ``BIFURCATION_ANL`` the peak rides up to the cliff edge, where a small
    shift jumps the branch: :func:`calibration_record` flags such tones.

    A driven fit's grid solve costs a few ms, so results are remembered per
    parameter set (a pure function of them: it cannot go stale); the
    demodulators, the tracker and stream-to-DAC call it every cycle.

    With a sweep table (:func:`sweep_frequency_correction`) the model's
    inflection is placed at the frequency the calibration sweep showed it at.
    """
    foj = float(getattr(calibration, 'frequency_origin_fraction', 0.0))
    anl = float(getattr(calibration, 'anl', 0.0))
    if anl == 0.0:
        op = float(calibration.fr * (1.0 - foj))
    else:
        op = _driven_operating_point_hz(float(calibration.fr), float(calibration.Ql), anl,
                                        int(n_grid), float(y_max))
    if getattr(calibration, 'frequency_correction', None) is None:
        return op
    return float(calibration.fr + calibration.correct_probe_detuning(op - calibration.fr))


@functools.lru_cache(maxsize=8192)
def _driven_operating_point_hz(fr, Ql, anl, n_grid, y_max):
    """:func:`operating_point_hz` for a driven fit (``anl > 0``)."""
    y = np.linspace(-y_max, y_max, n_grid)
    x = invert_duffing_coordinate(y, anl) / Ql     # (f - fr) / fr
    f = fr * (1.0 + x)
    dS21_dy = 2.0 / (1.0 + 4.0 * y ** 2)
    df_dy = np.abs(np.gradient(f, y))
    slope = dS21_dy / np.where(df_dy > 0.0, df_dy, np.inf)
    return float(f[int(np.argmax(slope))])


# A sweep table spans the operating point +-FREQUENCY_TABLE_LINEWIDTHS in
# FREQUENCY_TABLE_KNOTS knots (1/20 of a linewidth apart), and needs the fit to
# map its sweep one-to-one out to +-FREQUENCY_TABLE_MIN_LINEWIDTHS.
FREQUENCY_TABLE_LINEWIDTHS = 2.0
FREQUENCY_TABLE_MIN_LINEWIDTHS = 0.5
FREQUENCY_TABLE_KNOTS = 81


def _table_frequency_scale(knots, correction, linewidth):
    """Swept over modelled frequency change across +-1/4 linewidth of a
    table's centre (its operating point): 1 when the fit reads its sweep right."""
    centre, half = 0.5 * (knots[0] + knots[-1]), 0.25 * linewidth
    change = (np.interp(centre + half, knots, correction)
              - np.interp(centre - half, knots, correction))
    return float(1.0 + change / (2.0 * half))


def sweep_frequency_correction(calibration, frequencies, s21):
    """Tabulate a calibration's frequency error on its own sweep.

    Each sweep point is read back through the fitted model -- the Mobius
    inversion, and the Duffing inverse for a driven fit -- and the frequency
    it was swept at is compared with the frequency the model gives. Near the
    operating point real fits are off by about 2% of the detuning (10% or more
    on some driven tones), and that is what put a step in the tracked
    resonance at every tracking commit: the step is the error times the move.

    A calibration carrying the table (``frequency_correction``) reads its
    frequencies off the sweep instead: its probe detuning is the model's plus
    the table's correction at that detuning
    (:meth:`ResonatorCalibration.correct_probe_detuning`), while the model
    still separates frequency from loss -- which the sweep alone cannot: it
    only shows what a change of frequency looks like, not a change of loss.
    :func:`calibration_record` builds one per tone when given the sweep.

    The table spans the operating point +-2 linewidths, 1/20 of a linewidth
    between knots, and is cut at the first sweep point either side where the
    model runs backwards (a glitch in the sweep, or the fit breaking down);
    past its ends the end corrections hold.

    Parameters
    ----------
    calibration : ResonatorCalibration
        The tone's calibration, fitted to this sweep. Any table it already
        carries is ignored.
    frequencies, s21 : array_like
        The tone's calibration sweep: probe frequencies (Hz) and raw complex
        I/Q, as fitted.

    Returns
    -------
    dict
        ``knots_hz``: the model's probe detunings ``f - fr`` (Hz), evenly
        spaced; ``correction_hz``: swept minus modelled frequency at each
        (Hz); ``frequency_scale``: swept over modelled frequency change across
        +-1/4 linewidth of the operating point (1 for a perfect fit);
        ``linewidths``: how far the one-to-one stretch reaches below and
        above the operating point.

    Raises
    ------
    ValueError
        When the model does not map the sweep one-to-one out to
        +-0.5 linewidths of its operating point: the fit is unusable there.

    Examples
    --------
    A fit whose ``Ql`` is 3% low reads its sweep 3% wide; the table puts it
    back:

    >>> fr, Ql = 1e9, 2e4
    >>> f = fr + fr / Ql * np.linspace(-5, 5, 201)
    >>> s21 = 1 - 0.5 / (1 + 2j * Ql * (f - fr) / fr)
    >>> cal = ResonatorCalibration(fr, 0.97 * Ql, 0.0, 0.75 + 0j, 0.25, np.pi)
    >>> round(float(cal.convert_raw_iq(fr + 2e3, 1 - 0.5 / (1 + 2j * Ql * 2e3 / fr))[0]))
    2062
    >>> table = sweep_frequency_correction(cal, f, s21)
    >>> round(table['frequency_scale'], 3)
    0.97
    >>> cal.frequency_correction = (table['knots_hz'], table['correction_hz'])
    >>> round(float(cal.convert_raw_iq(fr + 2e3, 1 - 0.5 / (1 + 2j * Ql * 2e3 / fr))[0]))
    2000
    """
    model = copy.copy(calibration)
    model.frequency_correction = None
    f = np.asarray(frequencies, dtype=float).ravel()
    z = np.asarray(s21, dtype=complex).ravel()
    keep = np.isfinite(f) & np.isfinite(z)
    order = np.argsort(f[keep])
    f, z = f[keep][order], z[keep][order]
    linewidth = model.fr / model.Ql
    op = operating_point_hz(model) - model.fr
    swept = f - model.fr
    with np.errstate(divide='ignore', invalid='ignore'):
        modelled = model.convert_raw_iq(f, z)[0]
    # Out from the sweep point nearest the operating point, while the model
    # keeps running forwards.
    near = np.flatnonzero(np.abs(swept - op) <= FREQUENCY_TABLE_LINEWIDTHS * linewidth)
    if near.size < 3:
        raise ValueError('the sweep has too few points near the operating point')
    lo = hi = near[np.argmin(np.abs(swept[near] - op))]
    while lo > near[0] and modelled[lo - 1] < modelled[lo]:
        lo -= 1
    while hi < near[-1] and modelled[hi + 1] > modelled[hi]:
        hi += 1
    reach = ((swept[lo] - op) / linewidth, (swept[hi] - op) / linewidth)
    if not reach[0] <= -FREQUENCY_TABLE_MIN_LINEWIDTHS <= FREQUENCY_TABLE_MIN_LINEWIDTHS <= reach[1]:
        raise ValueError(
            f'the fit does not map its sweep one-to-one within '
            f'{FREQUENCY_TABLE_MIN_LINEWIDTHS:g} linewidths of its operating point '
            f'(only {reach[0]:+.2f} to {reach[1]:+.2f})')
    modelled, swept = modelled[lo:hi + 1], swept[lo:hi + 1]
    knots = op + linewidth * np.linspace(-FREQUENCY_TABLE_LINEWIDTHS, FREQUENCY_TABLE_LINEWIDTHS,
                                         FREQUENCY_TABLE_KNOTS)
    correction = np.interp(knots, modelled, swept - modelled)
    return {'knots_hz': knots, 'correction_hz': correction,
            'frequency_scale': _table_frequency_scale(knots, correction, linewidth),
            'linewidths': (float(reach[0]), float(reach[1]))}


# ---------------------------------------------------------------------------
# Calibration records: how a calibration travels
# ---------------------------------------------------------------------------

CALIBRATION_SCHEMA = 'souk-resonator-calibration/1'

# Per-tone numbers of a record and the ResonatorCalibration argument each
# feeds; the complex circle centre is split so a record is plain JSON.
_RECORD_FIELDS = (
    ('fr_hz', 'fr'), ('Ql', 'Ql'), ('Qi', 'Qi'), ('tau_s', 'tau'),
    ('circle_radius', 'radius'), ('rotation_rad', 'rotation_angle'),
    ('gain_amplitude', 'gain_amplitude'), ('gain_phase_rad', 'gain_phase'),
    ('frequency_origin_fraction', 'frequency_origin_fraction'), ('anl', 'anl'),
)
# The keys of the .npz files the old stream_dac.save_calibrations wrote.
_LEGACY_FIELDS = ('fr', 'Ql', 'tau', 'center', 'radius', 'rotation_angle',
                  'gain_amplitude', 'gain_phase', 'frequency_origin_fraction', 'anl')
_SWEEP_KEYS = ('sweep_f', 'sweep_i', 'sweep_q', 'sweep_ei', 'sweep_eq')


def is_calibration_record(obj):
    """True for a calibration record (see :func:`calibration_record`)."""
    return isinstance(obj, Mapping) and 'tone_indices' in obj and 'fr_hz' in obj


def _json_safe(value):
    """``json.dumps`` default for numpy scalars and arrays."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError(f'{type(value).__name__} is not JSON serialisable')


def _software_version():
    try:
        from importlib.metadata import version
        return version('souk_readout_tools')
    except Exception:
        return 'unknown'


def _calibration_items(calibrations):
    """``[(tone, ResonatorCalibration, source)]`` from fits or calibrations."""
    if isinstance(calibrations, Mapping):
        entries = [(int(key), value) for key, value in calibrations.items()]
    else:
        entries = []
        for position, entry in enumerate(calibrations):
            if entry is None:
                continue
            index = int(getattr(entry, 'tone_index', -1))
            entries.append((index if index >= 0 else position, entry))
    items = []
    for tone, entry in sorted(entries, key=lambda item: item[0]):
        if entry is None:
            continue
        if _is_fit_result(entry):
            # Same acceptance as the server's own fits: a finite resonance,
            # not a noise-only trace, and a usable loaded Q.
            if (getattr(entry, 'noise_only', False)
                    or not np.isfinite(getattr(entry, 'fr', np.nan))):
                continue
            calibration, source = ResonatorCalibration.from_fit(entry), 'fit'
        elif isinstance(entry, ResonatorCalibration):
            calibration, source = entry, 'calibration'
        else:
            raise TypeError(
                f'tone {tone}: cannot record a {type(entry).__name__}; pass fit '
                'results or ResonatorCalibration objects')
        if np.isfinite(calibration.fr) and np.isfinite(calibration.Ql) and calibration.Ql > 0:
            items.append((tone, calibration, source))
    return items


def _group_delay_to_json(table):
    if table is None:
        return None
    if isinstance(table, Mapping):
        return {'frequencies': np.asarray(table['frequencies'], dtype=float).tolist(),
                'tau_ns': np.asarray(table['tau_ns'], dtype=float).tolist()}
    if np.ndim(table) == 0:
        return float(table)
    return np.asarray(table, dtype=float).tolist()


def _group_delay_from_json(table):
    if table is None or np.ndim(table) == 0 and not isinstance(table, Mapping):
        return table
    if isinstance(table, Mapping):
        return {key: np.asarray(value, dtype=float) for key, value in table.items()}
    return np.asarray(table, dtype=float)


def _sweep_provenance(sweep_data):
    """What a record keeps of the sweep it came from (metadata only)."""
    info = sweep_data.get('info') if isinstance(sweep_data, Mapping) else None
    gain_db, fftshift = _rx_chain_gain_db(info)
    return {
        'sweep_date': sweep_data.get('date') if isinstance(sweep_data, Mapping) else None,
        'rx_chain_gain_db': gain_db,
        'pfb_fftshift': json.loads(json.dumps(fftshift, default=_json_safe)),
        'acc_len': _acc_len(info),
        'readout_correction_applied': bool(
            isinstance(sweep_data, Mapping)
            and sweep_data.get('readout_correction_applied', False)),
    }


def _has_sweep_traces(sweep_data):
    return (isinstance(sweep_data, Mapping)
            and all(sweep_data.get(key) is not None for key in ('sweep_f', 'sweep_i', 'sweep_q')))


def _set_frequency_tables(record, tables):
    """Write per-tone sweep tables -- ``(knots, correction)`` or None -- into
    a record, as ``frequency_correction_start_hz`` / ``_step_hz`` / ``_hz``
    (NaN without one), with the operating points they move."""
    starts, steps, values = [], [], []
    for table in tables:
        if table is None:
            starts.append(np.nan)
            steps.append(np.nan)
            values.append([np.nan] * FREQUENCY_TABLE_KNOTS)
        else:
            knots, correction = table
            starts.append(float(knots[0]))
            steps.append(float((knots[-1] - knots[0]) / (len(knots) - 1)))
            values.append([float(v) for v in correction])
    record['frequency_correction_start_hz'] = starts
    record['frequency_correction_step_hz'] = steps
    record['frequency_correction_hz'] = values
    record['operating_point_hz'] = [
        operating_point_hz(cal) for cal in calibrations_from_record(record).values()]


def _sweep_tables(record, sweep_data):
    """Each record tone's sweep table from ``sweep_data`` (None if refused)."""
    calibrations = calibrations_from_record(record)
    tables = []
    for tone in record['tone_indices']:
        try:
            f, i, q = sweep_trace(sweep_data, int(tone))
            table = sweep_frequency_correction(
                calibrations[int(tone)], f, np.asarray(i, float) + 1j * np.asarray(q, float))
            tables.append((table['knots_hz'], table['correction_hz']))
        except (ValueError, IndexError):
            tables.append(None)
    return tables


def calibration_record(calibrations, *, sweep_data=None, **provenance):
    """Collect per-tone resonator calibrations into one plain, JSON-safe record.

    The record is how a calibration travels: pushed to the server with
    ``ReadoutClient.set_resonator_calibrations`` (which puts it in every
    stream's metadata as ``info['resonators']``), saved with
    :func:`save_calibrations`, and accepted anywhere ``calibrations=`` is.

    Parameters
    ----------
    calibrations : mapping, sequence or record
        Fit results (``fitting.batch_fit`` output) or
        :class:`ResonatorCalibration` objects, as ``{tone_index: ...}`` or a
        sequence (an entry's ``tone_index`` is used when it has one). Noise-only
        and non-finite fits are left out. A record comes back as a copy.
    sweep_data : dict or None, optional
        The sweep the calibrations were fitted to. Its traces give every tone
        a sweep table (:func:`sweep_frequency_correction`), so conversions
        read frequencies off the sweep itself rather than the fitted shape;
        tones the fit does not map one-to-one near the operating point get
        none (:func:`calibration_quality` flags them). Of the rest, only its
        metadata is kept: date, RX chain gain and PFB shift (for
        :func:`recorded_rx_gain_ratio`) and whether its filterbank readout
        correction was applied.
    **provenance
        Anything else to keep with the record (JSON-safe values).

    Returns
    -------
    dict
        ``schema``; ``tone_indices``; per-tone lists ``fr_hz``, ``Ql``, ``Qi``,
        ``tau_s``, ``circle_center_re``, ``circle_center_im``,
        ``circle_radius``, ``rotation_rad``, ``gain_amplitude``,
        ``gain_phase_rad``, ``frequency_origin_fraction``, ``anl``,
        ``operating_point_hz`` (the phase inflection), ``linewidth_hz``
        (``fr / Ql``), ``bifurcated`` and ``source`` (``'fit'`` or
        ``'calibration'``); ``group_delay_cal`` (the shared path-delay table,
        or None) with per-tone ``group_delay_reference_frequency_hz``; and a
        ``provenance`` dict (creation time, software version, sweep metadata).
        With sweep tables, per-tone ``frequency_correction_hz`` (the table's
        corrections) at knots ``frequency_correction_start_hz +
        frequency_correction_step_hz * k`` of model probe detuning (NaN
        without a table).

    Examples
    --------
    >>> cal = ResonatorCalibration(1e9, 2e4, 0.0, 0.5 + 0.0j, 0.5, 0.0)
    >>> record = calibration_record({3: cal})
    >>> record['tone_indices'], record['linewidth_hz']
    ([3], [50000.0])
    >>> calibrations_from_record(record)[3].Ql
    20000.0
    """
    if is_calibration_record(calibrations):
        record = json.loads(json.dumps(dict(calibrations), default=_json_safe))
        record.setdefault('provenance', {}).update(provenance)
        if _has_sweep_traces(sweep_data):
            _set_frequency_tables(record, _sweep_tables(record, sweep_data))
        return record
    items = _calibration_items(calibrations)
    cals = [calibration for _, calibration, _ in items]
    record = {'schema': CALIBRATION_SCHEMA,
              'tone_indices': [int(tone) for tone, _, _ in items]}
    for field, attribute in _RECORD_FIELDS:
        record[field] = [float(getattr(cal, attribute)) for cal in cals]
    record['circle_center_re'] = [float(cal.center.real) for cal in cals]
    record['circle_center_im'] = [float(cal.center.imag) for cal in cals]
    record['operating_point_hz'] = [operating_point_hz(cal) for cal in cals]
    record['linewidth_hz'] = [cal.fr / cal.Ql for cal in cals]
    record['bifurcated'] = [bool(cal.anl >= BIFURCATION_ANL) for cal in cals]
    record['source'] = [source for _, _, source in items]
    tables = [cal.group_delay_cal for cal in cals if cal.group_delay_cal is not None]
    record['group_delay_cal'] = _group_delay_to_json(tables[0]) if tables else None
    record['group_delay_reference_frequency_hz'] = [
        None if cal.group_delay_reference_frequency is None
        else float(cal.group_delay_reference_frequency) for cal in cals]
    if _has_sweep_traces(sweep_data):
        _set_frequency_tables(record, _sweep_tables(record, sweep_data))
    elif any(cal.frequency_correction is not None for cal in cals):
        _set_frequency_tables(record, [cal.frequency_correction for cal in cals])
    info = {'created_unix': time.time(), 'software_version': _software_version()}
    if sweep_data is not None:
        info.update(_sweep_provenance(sweep_data))
    info.update(provenance)
    record['provenance'] = json.loads(json.dumps(info, default=_json_safe))
    bifurcated = [tone for tone, flag in zip(record['tone_indices'], record['bifurcated'])
                  if flag]
    if bifurcated:
        warnings.warn(
            f'{len(bifurcated)} tone(s) fitted past the Duffing bifurcation '
            f'(anl >= {BIFURCATION_ANL:.3f}): their steepest point is a cliff '
            f'edge, not a smooth inflection; lower the tone power before '
            f'tracking them. Tones: {bifurcated[:10]}', stacklevel=2)
    return record


def calibrations_from_record(record, tones=None):
    """Rebuild ``{tone_index: ResonatorCalibration}`` from a record.

    ``tones`` restricts to those tone indices.
    """
    wanted = None if tones is None else {int(t) for t in tones}
    table = _group_delay_from_json(record.get('group_delay_cal'))
    n = len(record['tone_indices'])
    references = record.get('group_delay_reference_frequency_hz') or [None] * n
    starts = record.get('frequency_correction_start_hz') or [None] * n
    out = {}
    for j, tone in enumerate(record['tone_indices']):
        tone = int(tone)
        if wanted is not None and tone not in wanted:
            continue
        kwargs = {attribute: float(record[field][j])
                  for field, attribute in _RECORD_FIELDS if field in record}
        center = complex(record['circle_center_re'][j], record['circle_center_im'][j])
        reference = references[j]
        if reference is not None and not np.isfinite(reference):
            reference = None
        correction = None
        if starts[j] is not None and np.isfinite(starts[j]):
            values = np.asarray(record['frequency_correction_hz'][j], dtype=float)
            knots = starts[j] + record['frequency_correction_step_hz'][j] * np.arange(values.size)
            correction = (knots, values)
        out[tone] = ResonatorCalibration(
            kwargs.pop('fr'), kwargs.pop('Ql'), kwargs.pop('tau'), center,
            kwargs.pop('radius'), kwargs.pop('rotation_angle'),
            group_delay_cal=table, group_delay_reference_frequency=reference,
            frequency_correction=correction, **kwargs)
    return out


def calibration_record_subset(record, tones):
    """A calibration record cut to ``tones``: its per-tone lists keep only
    those tones; schema, provenance and the shared group-delay table stay."""
    keep = {int(t) for t in tones}
    rows = [j for j, t in enumerate(record['tone_indices']) if int(t) in keep]
    n = len(record['tone_indices'])
    out = {}
    for key, value in record.items():
        if (key not in ('schema', 'provenance', 'group_delay_cal')
                and isinstance(value, (list, tuple)) and len(value) == n):
            out[key] = [value[j] for j in rows]
        else:
            out[key] = value
    return out


def stale_calibration_tones(record, tone_frequencies_hz, max_linewidths=5.0):
    """``{tone: reason}`` for the tones a calibration record no longer covers.

    A tone is stale when it is not active (no finite frequency at its index
    in ``tone_frequencies_hz``) or is tuned more than ``max_linewidths`` of its
    linewidths from its calibrated operating point: its resonance, and so its
    calibration, has moved on. Per tone -- the rest of the record stays good.
    """
    freqs = (None if tone_frequencies_hz is None
             else np.asarray(tone_frequencies_hz, dtype=float))
    stale = {}
    for j, tone in enumerate(record['tone_indices']):
        tone = int(tone)
        if freqs is None or not 0 <= tone < freqs.size or not np.isfinite(freqs[tone]):
            stale[tone] = 'not active'
        elif (abs(freqs[tone] - record['operating_point_hz'][j])
              > max_linewidths * record['linewidth_hz'][j]):
            stale[tone] = (f'{abs(freqs[tone] - record["operating_point_hz"][j]) / 1e3:.1f} kHz '
                           f'(over {max_linewidths:g} linewidths) from its calibrated resonance')
    return stale


def save_calibrations(filename, calibrations, sweep_data=None, **provenance):
    """Save calibrations -- and the sweep they came from -- to one ``.npz``.

    Loads without pickle (plain ``np.load`` works). Holds the record's
    per-tone arrays, its provenance as JSON and, when ``sweep_data`` is given,
    the sweep traces ``sweep_f`` / ``sweep_i`` / ``sweep_q`` (and errors), so a
    linearized, phase or lookup conversion can be redone from the file alone.
    ``calibrations`` and ``provenance`` are as for :func:`calibration_record`.
    """
    record = calibration_record(calibrations, sweep_data=sweep_data, **provenance)
    arrays = {'schema': np.asarray(record['schema']),
              'provenance_json': np.asarray(json.dumps(record['provenance'])),
              'group_delay_json': np.asarray(json.dumps(record['group_delay_cal']))}
    for key, value in record.items():
        if key in ('schema', 'provenance', 'group_delay_cal'):
            continue
        if key == 'group_delay_reference_frequency_hz':
            value = [np.nan if v is None else v for v in value]
        arrays[key] = np.asarray(value)
    if sweep_data is not None:
        for key in _SWEEP_KEYS:
            if sweep_data.get(key) is not None:
                arrays[key] = np.asarray(sweep_data[key], dtype=float)
        arrays['sweep_readout_correction_applied'] = np.asarray(
            bool(sweep_data.get('readout_correction_applied', False)))
        arrays['sweep_wideband'] = np.asarray(bool(sweep_data.get('wideband_sweep', False)))
    np.savez(filename, **arrays)


def load_calibration_record(filename):
    """Load ``(record, sweep_data)`` from :func:`save_calibrations`.

    ``sweep_data`` is None when the file holds no sweep. Also reads the older
    files ``stream_dac.save_calibrations`` wrote (parameters only).
    """
    with np.load(filename, allow_pickle=False) as handle:
        arrays = {key: handle[key] for key in handle.files}
    sweep = None
    if 'sweep_f' in arrays:
        sweep = {key: arrays.pop(key) for key in _SWEEP_KEYS if key in arrays}
        sweep['readout_correction_applied'] = bool(
            arrays.pop('sweep_readout_correction_applied', False))
        sweep['wideband_sweep'] = bool(arrays.pop('sweep_wideband', False))
    if 'schema' not in arrays:
        # Legacy layout: the ResonatorCalibration arguments, one array each.
        legacy = {}
        for j, tone in enumerate(arrays['tone_indices'].astype(int)):
            legacy[int(tone)] = ResonatorCalibration(
                float(arrays['fr'][j]), float(arrays['Ql'][j]), float(arrays['tau'][j]),
                complex(arrays['center'][j]), float(arrays['radius'][j]),
                float(arrays['rotation_angle'][j]),
                gain_amplitude=float(arrays['gain_amplitude'][j]),
                gain_phase=float(arrays['gain_phase'][j]),
                frequency_origin_fraction=float(arrays['frequency_origin_fraction'][j]),
                anl=float(arrays['anl'][j]))
        return calibration_record(legacy, legacy_file=True), sweep
    record = {'schema': str(arrays.pop('schema')),
              'provenance': json.loads(str(arrays.pop('provenance_json'))),
              'group_delay_cal': json.loads(str(arrays.pop('group_delay_json')))}
    for key, value in arrays.items():
        record[key] = value.tolist()
    record['group_delay_reference_frequency_hz'] = [
        None if v is None or not np.isfinite(v) else v
        for v in record.get('group_delay_reference_frequency_hz', [])]
    return record, sweep


def load_calibrations(filename):
    """Load ``{tone_index: ResonatorCalibration}`` from :func:`save_calibrations`."""
    return calibrations_from_record(load_calibration_record(filename)[0])


def calibrations_from_sweep(sweep_data, tone_indices=None, **batch_fit_kwargs):
    """Fit a parsed sweep and return ``{tone_index: ResonatorCalibration}``.

    Runs :func:`souk_readout_tools.fitting.batch_fit` (``batch_fit_kwargs``
    are forwarded) and keeps each tone with a finite, not noise-only fit.
    ``tone_indices`` restricts the result. Save with :func:`save_calibrations`,
    or push to the server with ``ReadoutClient.set_resonator_calibrations``.
    """
    from .fitting import batch_fit
    fits = batch_fit(sweep_data, **batch_fit_kwargs)
    calibrations = {}
    for tone, calibration, _ in _calibration_items(list(fits)):
        if tone_indices is None or tone in tone_indices:
            calibrations[tone] = calibration
    return calibrations


def calibration_from_data(data):
    """The calibration record a capture carries, or None.

    A capture saved with ``receive_stream(..., calibrations=...)`` carries it as
    ``data['resonator_calibration']``; any capture recorded while the server
    held a pushed calibration carries it in ``data['info']['resonators']``.
    A calibration the server has marked stale is not returned.
    """
    if not isinstance(data, Mapping):
        return None
    record = data.get('resonator_calibration')
    if is_calibration_record(record):
        return record
    info = data.get('info')
    section = info.get('resonators') if isinstance(info, Mapping) else None
    if (isinstance(section, Mapping) and section.get('valid', True)
            and is_calibration_record(section.get('record'))):
        return section['record']
    return None


# Calibration fit beyond this range: re-sweep and push again.
CALIBRATION_FIT_RANGE = (0.85, 1.15)


def calibration_fit(z_centered, calibration):
    """How well a calibration's circle still fits a tone's data: 1 when it does.

    The median of ``|z_c| / r`` over ``z_centered``, the tone's I/Q in the
    calibration's centred basis (:func:`to_basis`, each sample at its own probe
    frequency). Every point of a resonance the calibration describes lies on
    its circle, whatever the detuning, so this leaves 1 when the resonance
    changes shape since the calibration sweep (``Qi``, coupling, drive -- a
    cooldown does all three) or the data changes scale (TX or RX levels).
    Calibrated conversions go wrong with it: on krm4 a tone at 0.45 read its
    own tracking moves 1.37x too large. Beyond about 0.85-1.15, re-sweep and
    push again. The tracker and stream-to-DAC report it per tone.
    """
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        return float(np.nanmedian(np.abs(np.asarray(z_centered, dtype=complex)))
                     / float(calibration.radius))


def calibration_quality(calibrations, sweep_data=None, samples=None, *,
                        max_linewidth_ratio=2.0, max_neighbour_ratio=3.0, neighbours=10,
                        max_anl=0.6, min_dip_depth_db=3.0, fit_range=CALIBRATION_FIT_RANGE,
                        max_frequency_scale_error=0.2):
    """Per-tone checks of a calibration, and the tones that fail them.

    For choosing the tones to track, ``enable_tracking(enable_mask=result['good'])``,
    or to push. Each check is skipped when its data is not given:

    - **fitted vs measured linewidth** (``sweep_data``): ``fr / Ql`` against the
      model-free FWHM of the same trace beyond ``max_linewidth_ratio``: the fit
      did not describe the dip;
    - **linewidth odd for its neighbourhood**: ``fr / Ql`` beyond
      ``max_neighbour_ratio`` of the median of its ``neighbours`` nearest tones
      in frequency (linewidths vary across a band, so not the global median);
    - **near bifurcation**: ``anl > max_anl`` (the cliff is at ``BIFURCATION_ANL``);
    - **shallow dip** (``sweep_data``): shallower than ``min_dip_depth_db``;
    - **sweep table** (a record made with its sweep, or ``sweep_data``): the
      fit must map its own sweep one-to-one within 0.5 linewidths of its
      operating point (:func:`sweep_frequency_correction`), and there read
      frequencies within ``max_frequency_scale_error`` of the sweep's
      (``frequency_scale``; the table corrects smaller errors, but a fit that
      far out has the shape wrong for the loss coordinate too);
    - **calibration fit** (``samples``, a capture taken since the calibration
      sweep): :func:`calibration_fit` outside ``fit_range``.

    Parameters
    ----------
    calibrations : record, fits or ResonatorCalibration objects
        Anything :func:`calibration_record` takes, e.g. the record of
        :func:`load_calibration_record`.
    sweep_data : dict, optional
        The calibration sweep (a parsed sweep, or the one the file holds).
    samples : dict, optional
        A parsed capture (``parse_samples`` / ``parse_stream``), modulated or
        not; its RX gain change since the sweep is divided out.

    Returns
    -------
    dict
        Lists aligned with ``tone_indices``: ``linewidth_hz``,
        ``measured_linewidth_hz``, ``neighbour_linewidth_hz``, ``anl``,
        ``dip_depth_db``, ``frequency_scale`` and ``fit`` (NaN where not
        measured); ``reasons``, ``{tone: [reason, ...]}`` for the tones that
        fail; ``poor`` and ``good``, sorted tone indices.

    Examples
    --------
    >>> record, sweep = load_calibration_record('cal.npz')              # doctest: +SKIP
    >>> capture = client.parse_samples(client.get_samples(10000))       # doctest: +SKIP
    >>> quality = calibration_quality(record, sweep, capture)           # doctest: +SKIP
    >>> client.enable_tracking(dry_run=False, enable_mask=quality['good'])  # doctest: +SKIP
    """
    record = (calibrations if is_calibration_record(calibrations)
              else calibration_record(calibrations))
    cals = calibrations_from_record(record)
    tones = [int(t) for t in record['tone_indices'] if int(t) in cals]
    n = len(tones)
    fr = np.array([cals[t].fr for t in tones])
    lw = np.array([cals[t].fr / cals[t].Ql for t in tones])
    anl = np.array([cals[t].anl for t in tones])
    measured, depth, fit = np.full(n, np.nan), np.full(n, np.nan), np.full(n, np.nan)
    if sweep_data is not None:
        for j, t in enumerate(tones):
            try:
                f, i, q = sweep_trace(sweep_data, t)
                e = estimate_resonance_empirical(
                    np.asarray(f, dtype=float), np.asarray(i, float) + 1j * np.asarray(q, float))
            except (ValueError, IndexError):
                continue
            measured[j], depth[j] = e.linewidth_hz, e.dip_depth_db
    scale, no_table = np.full(n, np.nan), {}
    for j, t in enumerate(tones):
        if cals[t].frequency_correction is not None:
            scale[j] = _table_frequency_scale(*cals[t].frequency_correction, lw[j])
        elif sweep_data is not None:
            try:
                f, i, q = sweep_trace(sweep_data, t)
                scale[j] = sweep_frequency_correction(
                    cals[t], f, np.asarray(i, float) + 1j * np.asarray(q, float))['frequency_scale']
            except ValueError as error:
                no_table[t] = str(error)
            except IndexError:
                pass
        elif 'frequency_correction_hz' in record:
            no_table[t] = 'the fit does not map its sweep one-to-one near its operating point'
    neighbour = np.full(n, np.nan)
    order = np.argsort(fr)
    for rank, j in enumerate(order):
        lo = max(0, min(rank - neighbours // 2, n - neighbours - 1))
        near = [k for k in order[lo:lo + neighbours + 1] if k != j]
        if near:
            neighbour[j] = np.median(lw[near])
    if samples is not None:
        try:
            gain = recorded_rx_gain_ratio(record, samples.get('info')) or 1.0
        except ValueError:
            gain = 1.0
        settling = np.asarray(samples.get('modulation_settling', ()), dtype=bool)
        for j, t in enumerate(tones):
            try:
                z = (np.asarray(tone_column(samples, 'i_data', t), dtype=float)
                     + 1j * np.asarray(tone_column(samples, 'q_data', t), dtype=float)) / gain
                probe = probe_frequencies(samples, t, resolve_tone_frequency(samples, t))
            except ValueError:
                continue
            keep = ~settling if settling.size == z.size else np.ones(z.size, dtype=bool)
            fit[j] = calibration_fit(to_basis(z[keep], cals[t], probe[keep]), cals[t])
    reasons = {}
    with np.errstate(divide='ignore', invalid='ignore'):
        for j, t in enumerate(tones):
            why = []
            ratio = lw[j] / measured[j]
            if np.isfinite(ratio) and not 1 / max_linewidth_ratio <= ratio <= max_linewidth_ratio:
                why.append(f'fitted linewidth {lw[j] / 1e3:.1f} kHz, measured {measured[j] / 1e3:.1f} kHz')
            ratio = lw[j] / neighbour[j]
            if np.isfinite(ratio) and not 1 / max_neighbour_ratio <= ratio <= max_neighbour_ratio:
                why.append(f'linewidth {lw[j] / 1e3:.1f} kHz, neighbours {neighbour[j] / 1e3:.1f} kHz')
            if anl[j] > max_anl:
                why.append(f'anl {anl[j]:.2f} (bifurcation {BIFURCATION_ANL:.2f})')
            if np.isfinite(depth[j]) and depth[j] < min_dip_depth_db:
                why.append(f'dip {depth[j]:.1f} dB deep')
            if t in no_table:
                why.append(f'no sweep table: {no_table[t]}')
            if np.isfinite(scale[j]) and abs(scale[j] - 1.0) > max_frequency_scale_error:
                why.append(f'fit reads its sweep {100 * (1 / scale[j] - 1):+.0f}% in '
                           f'frequency at the operating point')
            if np.isfinite(fit[j]) and not fit_range[0] <= fit[j] <= fit_range[1]:
                why.append(f'calibration fit {fit[j]:.2f}')
            if why:
                reasons[t] = why
    return {'tone_indices': tones, 'linewidth_hz': lw.tolist(),
            'measured_linewidth_hz': measured.tolist(),
            'neighbour_linewidth_hz': neighbour.tolist(), 'anl': anl.tolist(),
            'dip_depth_db': depth.tolist(), 'frequency_scale': scale.tolist(),
            'fit': fit.tolist(), 'reasons': reasons,
            'poor': sorted(reasons), 'good': [t for t in tones if t not in reasons]}


# ---------------------------------------------------------------------------
# Tones and sweeps
# ---------------------------------------------------------------------------

def sweep_trace(sweep_data, tone_index):
    """One tone's sweep as ``(frequencies, I, Q)`` from targeted or wideband data."""
    sweep_f = np.atleast_2d(sweep_data['sweep_f'])
    sweep_i = np.atleast_2d(sweep_data['sweep_i'])
    sweep_q = np.atleast_2d(sweep_data['sweep_q'])
    if sweep_data.get('wideband_sweep', False) or sweep_f.shape[0] == 1:
        return sweep_f[0], sweep_i[0], sweep_q[0]
    n_tones = sweep_f.shape[1]
    if not 0 <= tone_index < n_tones:
        raise ValueError(f"Tone {tone_index} not in sweep data ({n_tones} tones)")
    return sweep_f[:, tone_index], sweep_i[:, tone_index], sweep_q[:, tone_index]


def resolve_tone_frequency(data, tone_index, override=None, sweep_f=None):
    """The frequency (Hz) a capture's tone was read out at.

    ``override`` wins: a scalar for every tone, a ``{tone: Hz}`` mapping
    (``'0003'`` keys too) or an array indexed by absolute tone index. Otherwise
    ``data['info']['tones']['frequencies_hz'][tone_index]``; failing that the
    middle of ``sweep_f``, with a warning; failing that, ``ValueError``.
    """
    tone_index = int(tone_index)
    if override is not None:
        if isinstance(override, Mapping):
            for key in (tone_index, f'{tone_index:04d}'):
                if key in override:
                    return float(override[key])
            raise ValueError(f'reference_tone_frequency mapping missing tone {tone_index}')
        if np.ndim(override):
            try:
                return float(np.asarray(override, dtype=float)[tone_index])
            except IndexError as exc:
                raise ValueError(
                    'array-like reference_tone_frequency is indexed by absolute '
                    f'tone index; tone {tone_index} is missing. Use a mapping for '
                    'non-contiguous selected tones.') from exc
        return float(override)
    info = data.get('info') if isinstance(data, Mapping) else None
    tones = info.get('tones', {}) if isinstance(info, Mapping) else {}
    frequencies = tones.get('frequencies_hz') if isinstance(tones, Mapping) else None
    if frequencies is not None:
        try:
            return float(np.asarray(frequencies, dtype=float)[tone_index])
        except (IndexError, TypeError, ValueError):
            pass
    if sweep_f is not None:
        warnings.warn(
            f"Tone {tone_index} has no frequency in the capture's info; falling "
            'back to the middle of the sweep range. Pass reference_tone_frequency '
            'to set it explicitly.', stacklevel=3)
        return float(np.mean(sweep_f))
    raise ValueError(f'tone {tone_index} needs a frequency: none in the capture '
                     'info; pass reference_tone_frequency')


# ---------------------------------------------------------------------------
# Readout (filterbank) correction and RX gain
# ---------------------------------------------------------------------------

def readout_correction_factors(info, num_tones):
    """The ``(num_points, num_tones)`` software readout-flattening factors.

    From a ``get_info`` payload's modulation state, or ``None`` when it
    carries none (compensation off, or nothing armed). Tones without a
    reported correction get 1. These are the linear gains the filterbank
    compensation cannot apply in hardware (the RX scale word is inert);
    ``ReadoutClient.parse_samples`` applies them to modulated captures by
    default. See ``doc/filterbank_compensation.md``.
    """
    state = info.get('modulation') if isinstance(info, Mapping) else None
    if not (isinstance(state, Mapping) and state.get('tones')):
        return None
    n_points = int(state.get('num_points', 0) or 0)
    if n_points < 1:
        return None
    factors = np.ones((n_points, int(num_tones)), dtype=float)
    found = False
    for tone in state.get('tones', []):
        if not isinstance(tone, Mapping):
            continue
        rc = tone.get('readout_correction')
        try:
            idx = int(tone.get('index'))
        except (TypeError, ValueError):
            continue
        if rc is None or len(rc) != n_points or not 0 <= idx < num_tones:
            continue
        factors[:, idx] = np.asarray(rc, dtype=float)
        found = True
    return factors if found else None


def apply_readout_correction(z, points, factors, *, invert=False):
    """Scale each modulated sample by its ``(point, tone)`` readout factor.

    ``z`` is ``(n_samples, n_tones)`` I/Q (real or complex), ``points`` the
    1-based point tag of each sample (0 = not modulated, left alone) and
    ``factors`` from :func:`readout_correction_factors`. ``invert=True``
    divides instead, undoing an earlier application. Returns a new array.
    """
    z = np.array(z, dtype=np.result_type(np.asarray(z).dtype, float), copy=True)
    points = np.asarray(points, dtype=int)
    factors = np.asarray(factors, dtype=float)
    valid = (points >= 1) & (points <= factors.shape[0])
    gain = factors[points[valid] - 1][:, :z.shape[1]]
    z[valid] = z[valid] / gain if invert else z[valid] * gain
    return z


def sample_readout_factors(data, num_tones=None):
    """Per-sample ``(n_samples, num_tones)`` readout factors of a parsed capture.

    Each modulated sample gets the factor of its point, from the modulation
    state in ``data['info']`` -- or, in a recording whose tracking or
    recentring changed the state, from its own revision's factors
    (``readout_correction_by_revision``, collected by ``parse_stream``).
    ``None`` when the capture is unmodulated or reports no factors.
    """
    points = np.asarray(data.get('modulation_point', ()), dtype=int)
    if not points.size or not np.any(points > 0):
        return None
    if num_tones is None:
        num_tones = max([int(data.get('num_tones', 0) or 0)]
                        + [int(key) + 1 for key in data.get('i_data', {})])
    initial = readout_correction_factors(data.get('info'), num_tones)
    by_revision = {int(rev): np.asarray(factors, dtype=float) for rev, factors in
                   (data.get('readout_correction_by_revision') or {}).items()}
    if initial is None and not by_revision:
        return None
    revisions = np.asarray(data.get('modulation_revision', np.zeros(len(points))), dtype=int)
    gains = np.ones((len(points), int(num_tones)), dtype=float)
    for revision in np.unique(revisions[points > 0]):
        factors = by_revision.get(int(revision), initial)
        if factors is None:
            continue
        rows = (revisions == revision) & (points >= 1) & (points <= factors.shape[0])
        width = min(factors.shape[1], gains.shape[1])
        gains[rows, :width] = factors[points[rows] - 1][:, :width]
    return gains


def with_readout_correction(data, apply=True):
    """A sweep or parsed capture with the software readout correction on or off.

    The filterbank compensation's RX half cannot be applied in hardware, so
    the server reports per-(point, tone) factors: a parsed sweep carries them
    (``readout_correction``), a parsed capture takes them from its modulation
    state (:func:`sample_readout_factors`). Both carry a
    ``readout_correction_applied`` flag, so the correction can be toggled
    either way. Returns the input itself when it is already in the requested
    state or has no factors (compensation off, older or unmodulated data).
    """
    if not isinstance(data, dict):
        return data
    want = bool(apply)
    if want == bool(data.get('readout_correction_applied', False)):
        return data
    if 'sweep_i' in data:
        factors = data.get('readout_correction')
        if factors is None:
            return data
        factors = np.asarray(factors, dtype=float)
        if factors.shape != np.shape(data['sweep_i']):
            return data
        gain = factors if want else 1.0 / factors
        out = dict(data)
        for key in ('sweep_i', 'sweep_q', 'sweep_ei', 'sweep_eq'):
            if out.get(key) is not None:
                out[key] = np.asarray(out[key], dtype=float) * gain
        out['readout_correction_applied'] = want
        return out
    if 'i_data' not in data:
        return data
    gains = sample_readout_factors(data)
    if gains is None:
        return data
    if not want:
        gains = 1.0 / gains

    def scaled(columns):
        out = {}
        for key, values in columns.items():
            values = np.asarray(values)
            out[key] = (values.astype(float) * gains[:, int(key)]
                        if len(values) == len(gains) else values)
        return out

    out = dict(data)
    out['i_data'] = scaled(data['i_data'])
    out['q_data'] = scaled(data['q_data'])
    out['readout_correction_applied'] = want
    return out


def _rx_chain_gain_db(info):
    """Analog RX gain in dB from a capture's recorded chain state, or None.

    Only the analog part -- the RF frontend total (attenuator and bypass amp
    together) and the ADC DSA, which sits at the converter. The PFB fftshift
    is returned separately: it is downstream of the ADC and its per-stage
    convention has bitten this codebase before, so a mismatch there is raised
    rather than silently modelled.
    """
    if not isinstance(info, Mapping):
        return None, None
    frontend = info.get('rf_frontend')
    rfdc = info.get('rfdc')
    if not isinstance(frontend, Mapping) or not isinstance(rfdc, Mapping):
        return None, None
    try:
        total = float(frontend['rx_total_gain_db'])
        # A latched ADC over-voltage trip holds the DSA at 15 dB, which the
        # programmed 'dsa' does not show (firmware_lib.effective_adc_dsa_db).
        dsa = float(rfdc.get('dsa_effective_db', rfdc.get('dsa', 0.0)))
    except (KeyError, TypeError, ValueError):
        return None, None
    pipeline = info.get('pipeline')
    digital = pipeline.get('pfb_fftshift') if isinstance(pipeline, Mapping) else None
    return total - dsa, digital


def rx_gain_ratio(sweep_data, ts_data, method="carrier", *, tones=None):
    """Amplitude ratio to put a calibration sweep on the timestream's RX scale.

    A sweep and a timestream taken at different ADC drive -- different RX
    attenuation or DSA -- differ by one scale factor on the I/Q plane. That is
    recoverable, and recovering it is what lets the ADC fill be raised for a
    science capture while the calibration sweep is taken cold enough not to
    clip. Note this is *RX* gain only: a sweep taken at a different **tone
    power** cannot be rescaled to serve a timestream, because the drive changes
    ``anl``, ``Qi`` and ``fr`` and the resonance is a different shape.

    Parameters
    ----------
    sweep_data : dict
        Calibration sweep.
    ts_data : dict
        Parsed timestream whose scale the sweep should be matched to.
    method : {'carrier', 'chain'}, optional
        ``'carrier'`` (default) measures the ratio as the timestream's median
        ``|z|`` over the sweep's ``|z|`` at the same tone frequency. Model-free,
        but it assumes the resonator did not move between the captures -- a
        real shift in loading is absorbed as a fake gain change. ``'chain'``
        computes it from the recorded RF frontend gain and ADC DSA, and raises
        if the digital (PFB) settings differ.
    tones : iterable of int or None, optional
        Tone indices; ``None`` (default) does every tone in ``ts_data``.

    Returns
    -------
    dict
        ``ratio`` (per tone: timestream amplitude over sweep amplitude, so
        divide the timestream by it to reach the sweep's scale), ``ratio_db``,
        ``tone_indices``, ``valid``, ``method``.

    Examples
    --------
    >>> import numpy as np
    >>> f = np.linspace(0.999e9, 1.001e9, 201)
    >>> z = 1.0 - 0.5 / (1.0 + 2j * 5e4 * (f - 1e9) / 1e9)
    >>> sweep = {"sweep_f": f[:, None], "sweep_i": z.real[:, None],
    ...          "sweep_q": z.imag[:, None]}
    >>> ts = {"i_data": {"0000": 2.0 * z.real[:3]},
    ...       "q_data": {"0000": 2.0 * z.imag[:3]},
    ...       "info": {"tones": {"frequencies_hz": [f[1]]}}}
    >>> round(float(rx_gain_ratio(sweep, ts)["ratio"][0]), 2)
    2.0
    """
    import operator

    method = str(method).lower()
    if method not in ("carrier", "chain"):
        raise ValueError("method must be 'carrier' or 'chain'")
    tone_keys = sorted(ts_data["i_data"])
    if tones is None:
        tones = range(len(tone_keys))
    indices = [operator.index(t) for t in tones]

    if method == "chain":
        ratio = recorded_rx_gain_ratio(sweep_data, ts_data.get("info"))
        if ratio is None:
            raise ValueError(
                "method='chain' needs rf_frontend and rfdc sections in both "
                "captures' info; use method='carrier' instead.")
        ratio = np.full(len(indices), ratio)
        valid = np.ones(len(indices), dtype=bool)
    else:
        ratio = np.full(len(indices), np.nan)
        valid = np.zeros(len(indices), dtype=bool)
        outside = []
        for row, tone_index in enumerate(indices):
            tone_key = tone_keys[tone_index]
            sweep_f, sweep_i, sweep_q = sweep_trace(sweep_data, tone_index)
            tone_frequency = resolve_tone_frequency(ts_data, tone_index, sweep_f=sweep_f)
            lo, hi = np.nanmin(sweep_f), np.nanmax(sweep_f)
            if not (lo <= tone_frequency <= hi):
                outside.append(tone_index)
                continue
            sweep_mag = np.interp(tone_frequency, sweep_f, np.abs(sweep_i + 1j * sweep_q))
            ts_mag = np.median(np.abs(
                np.asarray(ts_data["i_data"][tone_key], dtype=float)
                + 1j * np.asarray(ts_data["q_data"][tone_key], dtype=float)))
            if sweep_mag > 0 and np.isfinite(ts_mag):
                ratio[row] = float(ts_mag / sweep_mag)
                valid[row] = True
        if outside:
            warnings.warn(
                f"{len(outside)} tone(s) sit outside their sweep range, so the "
                "carrier ratio is undefined for them (an off-resonance capture "
                "parked beyond the sweep span does this). Pass method='chain' or "
                "an explicit ratio.", stacklevel=2)

    with np.errstate(divide="ignore", invalid="ignore"):
        ratio_db = 20.0 * np.log10(np.where(ratio > 0, ratio, np.nan))
    return {"ratio": ratio, "ratio_db": ratio_db,
            "tone_indices": np.asarray(indices, dtype=int), "valid": valid,
            "method": method}


def warn_adc_overvoltage(info=None, samples=None, label='data'):
    """Warn about an ADC over-voltage trip in a capture's recorded state.

    ``info``: latched when recorded, so the effective DSA (15 dB) is used in
    place of the programmed one. ``samples``: a parsed capture whose
    ``adc_overvoltage`` mask changes part-way, so no one RX scale fits it.
    """
    rfdc = info.get('rfdc') if isinstance(info, Mapping) else None
    if isinstance(rfdc, Mapping) and rfdc.get('adc_overvoltage_latched'):
        note = (' (the trip\'s effect above 15 dB is unmeasured)'
                if rfdc.get('dsa_effective_uncertain') else '')
        warnings.warn(
            f"ADC over-voltage protection was engaged for the {label}: its DSA was "
            f"{rfdc.get('dsa_effective_db')} dB, not the programmed {rfdc.get('dsa')} dB; "
            f"the RX scale uses the effective value{note}.", stacklevel=3)
    mask = samples.get('adc_overvoltage') if isinstance(samples, Mapping) else None
    if mask is not None and len(mask):
        mask = np.asarray(mask, dtype=bool)
        if mask.any() and not mask.all():
            warnings.warn(
                f'ADC over-voltage protection engaged for {int(mask.sum())} of {len(mask)} '
                f'samples of the {label} (samples["adc_overvoltage"]): the ADC DSA was held '
                'at 15 dB there, so those samples are on a different RX scale from the rest.',
                stacklevel=3)


def recorded_rx_gain_ratio(calibration, data_info):
    """RX amplitude ratio, data over calibration, from recorded chain states.

    ``calibration`` is a calibration record (its provenance keeps the sweep's
    chain state) or any capture with an ``info`` -- a sweep, or another parsed
    stream, to compare two streams. Two parts, each used when both sides
    record it:

    - the analog chain gain (RF frontend and ADC DSA);
    - the accumulation length: the accumulator sums ``acc_len`` spectra, so
      the I/Q scale with it, and data at acc_len 480 are twice as large as a
      sweep at 240.

    Returns ``None`` when neither part is known on both sides (records made
    before acc_len was kept have only the analog part); raises when the PFB
    shifts differ (its per-stage gain is not modelled). Divide the data's I/Q
    by the ratio to put it on the calibration's scale.
    """
    if is_calibration_record(calibration):
        provenance = calibration.get('provenance') or {}
        cal_db, cal_shift = provenance.get('rx_chain_gain_db'), provenance.get('pfb_fftshift')
        cal_acc = provenance.get('acc_len')
    elif isinstance(calibration, Mapping):
        cal_db, cal_shift = _rx_chain_gain_db(calibration.get('info'))
        cal_acc = _acc_len(calibration.get('info'))
    else:
        return None
    warn_adc_overvoltage(data_info, label='data')
    if isinstance(calibration, Mapping) and not is_calibration_record(calibration):
        warn_adc_overvoltage(calibration.get('info'), label='calibration')
    data_db, data_shift = _rx_chain_gain_db(data_info)
    data_acc = _acc_len(data_info)
    analog_known = cal_db is not None and data_db is not None
    acc_known = cal_acc is not None and data_acc is not None
    if not analog_known and not acc_known:
        return None
    if analog_known and (json.dumps(cal_shift, default=_json_safe)
                         != json.dumps(data_shift, default=_json_safe)):
        raise ValueError(
            f'PFB fftshift differs between the calibration and the data '
            f'({cal_shift} vs {data_shift}); its per-stage gain is not modelled. '
            'Match the RX scale explicitly (match_rx_gain=<ratio>).')
    ratio = 10.0 ** ((float(data_db) - float(cal_db)) / 20.0) if analog_known else 1.0
    if acc_known:
        ratio *= float(data_acc) / float(cal_acc)
    return float(ratio)


def _acc_len(info):
    """The accumulation length a capture's info records, or None."""
    pipeline = info.get('pipeline') if isinstance(info, Mapping) else None
    try:
        acc_len = float(pipeline['acc_len'])
    except (KeyError, TypeError, ValueError):
        return None
    return acc_len if acc_len > 0 else None


def resolve_rx_gain(match_rx_gain, calibration, data_info):
    """The data/calibration RX amplitude ratio a conversion divides out.

    ``match_rx_gain``: ``'auto'`` uses :func:`recorded_rx_gain_ratio` when both
    chain states are known and differ (with a warning), else 1; ``False`` or
    ``None`` gives 1; a number is taken as the ratio.
    """
    if match_rx_gain is False or match_rx_gain is None:
        return 1.0
    if match_rx_gain == 'auto':
        ratio = recorded_rx_gain_ratio(calibration, data_info)
        if ratio is None or np.isclose(ratio, 1.0, rtol=1e-6):
            return 1.0
        warnings.warn(
            f'RX scale (chain gain and acc_len) differs by {20 * np.log10(ratio):+.2f} dB '
            'between the calibration and the data; the data I/Q is rescaled to the '
            "calibration's scale (match_rx_gain=False to disable).", stacklevel=3)
        return ratio
    return float(match_rx_gain)


# ---------------------------------------------------------------------------
# Bases
# ---------------------------------------------------------------------------

BASES = ('raw', 'deembedded', 'centered', 'centered_rotated')


def basis_coefficients(calibration, frequency_hz, basis='centered_rotated', gain=1.0):
    """Affine coefficients ``(A, B)`` taking raw I/Q into a basis: ``A * z + B``.

    ``'raw'`` leaves the I/Q alone; ``'deembedded'`` removes the fitted gain
    and cable delay at ``frequency_hz`` (the probe frequency each sample was
    measured at); ``'centered'`` also subtracts the fitted circle centre;
    ``'centered_rotated'`` also rotates the off-resonance point onto the
    negative real axis, so the resonance sits at phase 0. ``gain`` (data over
    calibration amplitude, see :func:`recorded_rx_gain_ratio`) is divided out
    first. Precompute once per tone and probe frequency: applying is one
    complex multiply-add, cheap enough for the RFSoC's ARM.

    Examples
    --------
    >>> cal = ResonatorCalibration(1e9, 2e4, 0.0, 0.5 + 0.0j, 0.5, np.pi)
    >>> A, B = basis_coefficients(cal, 1e9)
    >>> complex(np.round(A * 0.0 + B, 12))   # the resonance point lands at +r
    (0.5-0j)
    """
    if basis not in BASES:
        raise ValueError(f'basis must be one of {BASES}, got {basis!r}')
    frequency_hz = np.asarray(frequency_hz, dtype=float)
    scale = 1.0 / gain
    if basis == 'raw' or calibration is None:
        if basis != 'raw':
            raise ValueError(f"basis={basis!r} needs a calibration")
        return (np.full(frequency_hz.shape, scale, dtype=complex),
                np.zeros(frequency_hz.shape, dtype=complex))
    multiply = calibration.deembedding_calibration.multiplier(frequency_hz) * scale
    if basis == 'deembedded':
        return multiply, np.zeros(multiply.shape, dtype=complex)
    rotation = calibration._rotation_phasor if basis == 'centered_rotated' else 1.0
    return multiply * rotation, np.full(multiply.shape, -calibration.center * rotation)


def to_basis(z, calibration, frequency_hz, basis='centered_rotated', gain=1.0):
    """Raw I/Q measured at ``frequency_hz`` in a basis (see :func:`basis_coefficients`)."""
    A, B = basis_coefficients(calibration, frequency_hz, basis, gain)
    return np.asarray(z, dtype=complex) * A + B


# ---------------------------------------------------------------------------
# Three-point modulation: slope, curvature, detuning
# ---------------------------------------------------------------------------

def phase_derivatives(z_points, offsets, method='fast', per_point=False):
    """Phase slope and curvature from the I/Q at each modulation probe point.

    Parameters
    ----------
    z_points : array_like of complex, shape (..., N, n_tones)
        One (averaged) I/Q value per probe point -- cycle means, or blocks of
        cycles averaged coherently. The phase differentiated is that of the
        basis they are in: use the centred basis (:func:`to_basis`) for
        detuning, since about the raw origin it reads 13x to 100x high.
    offsets : array_like, shape (N, n_tones) or (N,)
        Probe offset of each point in Hz, in any slot order.
    method : {'fast', 'fit'}, optional
        ``'fast'`` (default, the tracker's): finite differences between the
        lowest, central and highest distinct offsets, points sharing an offset
        averaged coherently first; phase differences are angles of products,
        so nothing is unwrapped. ``'fit'``: least-squares quadratic of the
        unwrapped phase against offset (a line with two distinct offsets), so
        every point counts -- e.g. both centre dwells of ``[0, +d, 0, -d]``.
    per_point : bool, optional
        Also return the complex I/Q gradient and the phase slope at every
        point, for converting the samples there. These are exact for a
        response that is a Mobius function of frequency -- a resonance
        circle, in any affine basis -- whatever ``method``: with the
        difference quotients ``D_ab = (z_b - z_a) / (f_b - f_a)`` of the
        lowest, central and highest offsets' mean I/Q,
        ``dz/df = D_lm D_mh / D_lh`` at the centre (the harmonic mean of the
        two one-sided quotients), ``D_lm D_lh / D_mh`` at the lowest offset
        and ``D_mh D_lh / D_lm`` at the highest; linear in offset between
        them, and the chord ``D_lh`` with only two distinct offsets. A central
        difference would read the circle's chord for its tangent, 4% low at
        +/-0.1 linewidth offsets and 20% at +/-0.25, scaling every
        converted shift by as much.

    Returns
    -------
    dict
        ``dphi_df`` (rad/Hz) and ``d2phi_df2`` (rad/Hz**2), shape
        ``(..., n_tones)``, at ``center_offset_hz`` (the offset nearest the
        middle of the span, ``(n_tones,)``). NaN where not estimable; the
        curvature needs three distinct offsets. These are the ``method``'s
        finite-difference or fitted estimates, the inputs of
        :func:`detuning_from_derivatives`: robust, but low by
        ``atan(u) / u``, ``u = 2 * offset / linewidth``, on resonance (1.3%
        at +/-0.1 linewidth). With ``per_point``: ``dz_df`` and
        ``local_dphi_df = Im(dz_df / z)`` (the exact phase slope about the
        basis origin), shape ``(..., N, n_tones)``.

    Examples
    --------
    A centred notch phase ``-2 atan(2x)``, the tone 0.3 linewidths below the
    resonance, on the firmware default pattern:

    >>> lw = 5e4; off = np.array([-5e3, 0.0, 5e3, 0.0])
    >>> z = np.exp(-2j * np.arctan(2 * (-0.3 * lw + off) / lw))[:, None]
    >>> d = phase_derivatives(z, off)
    >>> round(float(detuning_from_derivatives(d['dphi_df'], d['d2phi_df2'])[0]), 3)
    0.296
    """
    z = np.asarray(z_points, dtype=complex)
    N, n_tones = z.shape[-2], z.shape[-1]
    offsets = np.broadcast_to(np.asarray(offsets, dtype=float).reshape(N, -1),
                              (N, n_tones))
    f_lo = offsets.min(axis=0)
    f_hi = offsets.max(axis=0)
    f_mid = offsets[np.argmin(np.abs(offsets - 0.5 * (f_lo + f_hi)), axis=0),
                    np.arange(n_tones)]
    three = (f_lo < f_mid) & (f_mid < f_hi)
    out = {'center_offset_hz': f_mid}

    def mean_at(f):
        """Coherent mean of the points at offset ``f`` (per tone)."""
        w = (offsets == f).astype(float)
        return np.einsum('...nt,nt->...t', z, w / w.sum(axis=0))

    z_lo, z_mid, z_hi = mean_at(f_lo), mean_at(f_mid), mean_at(f_hi)
    if method == 'fast':
        d_lo = np.angle(z_mid * np.conj(z_lo))          # phi_mid - phi_lo
        d_hi = np.angle(z_hi * np.conj(z_mid))          # phi_hi - phi_mid
        with np.errstate(divide='ignore', invalid='ignore'):
            slope = (d_lo + d_hi) / (f_hi - f_lo)
            # Second difference for (possibly) unequal spacing; for a symmetric
            # pattern it is (phi_hi - 2 phi_mid + phi_lo) / half**2.
            curvature = 2.0 * (d_hi / (f_hi - f_mid) - d_lo / (f_mid - f_lo)) / (f_hi - f_lo)
        slope = np.where(f_hi > f_lo, slope, np.nan)
        curvature = np.where(three, curvature, np.nan)
    elif method == 'fit':
        lead = z.shape[:-2]
        flat = z.reshape(-1, N, n_tones)
        slope = np.full((flat.shape[0], n_tones), np.nan)
        curvature = np.full_like(slope, np.nan)
        for t in range(n_tones):
            x = offsets[:, t] - f_mid[t]
            order = np.argsort(x)
            distinct = len(np.unique(x))
            if distinct < 2:
                continue
            deg = 2 if distinct >= 3 else 1
            phi = np.unwrap(np.angle(flat[:, order, t]), axis=1)
            c = np.polyfit(x[order], phi.T, deg)          # highest power first
            slope[:, t] = c[-2]
            if deg == 2:
                curvature[:, t] = 2.0 * c[0]
        slope = slope.reshape(lead + (n_tones,))
        curvature = curvature.reshape(lead + (n_tones,))
    else:
        raise ValueError(f"method must be 'fast' or 'fit', got {method!r}")
    out['dphi_df'] = slope
    out['d2phi_df2'] = curvature

    if per_point:
        with np.errstate(divide='ignore', invalid='ignore'):
            d_lm = (z_mid - z_lo) / (f_mid - f_lo)
            d_mh = (z_hi - z_mid) / (f_hi - f_mid)
            d_lh = (z_hi - z_lo) / (f_hi - f_lo)
            # Exact for a circle (see per_point above); the chord otherwise.
            g_lo = np.where(three, d_lm * d_lh / d_mh, d_lh)[..., None, :]
            g_mid = np.where(three, d_lm * d_mh / d_lh, d_lh)[..., None, :]
            g_hi = np.where(three, d_mh * d_lh / d_lm, d_lh)[..., None, :]
            below = g_lo + (g_mid - g_lo) * (offsets - f_lo) / (f_mid - f_lo)
            above = g_mid + (g_hi - g_mid) * (offsets - f_mid) / (f_hi - f_mid)
            dz_df = np.where(three, np.where(offsets <= f_mid, below, above),
                             d_lh[..., None, :])
            out['dz_df'] = dz_df
            out['local_dphi_df'] = np.imag(dz_df / z)
    return out


def detuning_from_derivatives(dphi_df, d2phi_df2):
    """Detuning, resonance minus tone, in linewidths, from phase slope and curvature.

    On the circle-centred phase a notch is exactly ``phi_0 - 2 atan(2 x)``,
    ``x = (f_tone - f_res) / linewidth``, which inverts to
    ``x = -phi'' / (2 phi' |phi'|)``: the detuning in linewidths from the two
    derivatives alone, whatever the phase offset (a rotation error cancels)
    and without a linewidth. This returns ``-x``, the resonance-minus-tone
    sign: positive means the resonance is above the tone, so the tone should
    move up. Exact for the ideal linear notch's derivatives; the finite
    differences of :func:`phase_derivatives` bias it by ~1% at +/-0.1
    linewidth offsets and ~5% at +/-0.25. For a driven (Duffing) response only
    its zero -- the inflection -- is exact. Multiply by a linewidth for Hz.
    """
    slope = np.asarray(dphi_df, dtype=float)
    curvature = np.asarray(d2phi_df2, dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        detuning = curvature / (2.0 * slope * np.abs(slope))
    return np.where(np.isfinite(detuning), detuning, np.nan)


def mobius_center_detuning(z_points, offsets, calibration):
    """Resonance minus modulation centre (Hz) by the calibration's exact inversion.

    The default detuning estimate for calibrated tones -- the tracker's, the
    live viewer's and the demodulators'. Each probe point's centred I/Q (the
    ``centered_rotated`` basis, :func:`to_basis`) is inverted with the fitted
    model -- the circle's Mobius map and, for a driven fit, the analytic
    Duffing inverse -- into that probe's detuning from the fitted resonance;
    less the point's offset it is the centre's, averaged over the points,
    plus the operating point's offset from ``fr``, so zero means the centre
    sits on the phase inflection (:func:`operating_point_hz`).

    Its scale is right for driven resonators too. :func:`detuning_from_derivatives`
    shares its zero but not its scale there: on tones driven to ``anl ~ 0.3``
    it read 2x (3-3.5x by the fit), enough to make a gain-1 tracker jump from
    side to side of the resonance. The price is leaning on the fitted rotation
    and nonlinearity, which the curvature estimate does not. A calibration
    with a sweep table (a record made with its sweep) reads each probe's
    detuning off the sweep's own frequency axis instead of the fitted shape
    (:func:`sweep_frequency_correction`), which is what keeps the tracked
    resonance continuous across tracking commits.

    Parameters
    ----------
    z_points : array_like of complex, shape (..., N)
        Centred I/Q at each of the tone's N probe points (e.g. cycle means).
    offsets : array_like, shape (N,)
        Each point's probe offset from the centre (Hz); a single zero offset
        (no modulation) works too.
    calibration : ResonatorCalibration

    Returns
    -------
    numpy.ndarray, shape (...)
        Resonance minus centre in Hz; divide by ``fr / Ql`` for linewidths.

    Examples
    --------
    >>> cal = ResonatorCalibration(1e9, 2e4, 0.0, 0.75 + 0j, 0.25, np.pi)
    >>> off = np.array([-5e3, 0.0, 5e3, 0.0])
    >>> s21 = 1 - 0.5 / (1 + 2j * 2e4 * (1e9 + off - (1e9 + 3e3)) / 1e9)
    >>> round(float(mobius_center_detuning(to_basis(s21, cal, 1e9 + off), off, cal)))
    3000
    """
    # Points that saw no sample are NaN: they drop out quietly.
    with np.errstate(divide='ignore', invalid='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)
        probe_detuning = calibration.convert_centered_iq(
            np.asarray(z_points, dtype=complex))[0]
        centre = np.nanmean(np.asarray(offsets, dtype=float) - probe_detuning, axis=-1)
    return centre + operating_point_hz(calibration) - calibration.fr


def phase_shift_and_dissipation(z, reference, dphi_df):
    """Resonance shift and matched dissipation, both in Hz, from centred I/Q.

    With ``z`` and ``reference`` in the circle-centred basis and ``dphi_df``
    the local slope of its phase with probe frequency, a resonator
    perturbation moves the point along the circle (frequency) and radially
    (loss) on the same scale -- the map is conformal -- so with
    ``w = ln(z / reference)``::

        resonance shift        = -Im(w) / dphi_df
        fr * Delta(1 / (2 Qi)) =  Re(w) / dphi_df

    Linear in the centred phase and log-radius, first-order exact at any point
    on the circle, and needing nothing of the fit but the centring: the
    preferred three-point conversion, with ``dphi_df`` measured live by the
    modulation. About the raw origin the phase mixes loss into frequency, so
    do not feed it uncentred I/Q.
    """
    with np.errstate(divide='ignore', invalid='ignore'):
        w = np.log(np.asarray(z, dtype=complex) / np.asarray(reference, dtype=complex))
        slope = np.asarray(dphi_df, dtype=float)
        return -w.imag / slope, w.real / slope


# ---------------------------------------------------------------------------
# Fixed-tone conversion: prepare once, convert many
# ---------------------------------------------------------------------------

CONVERSION_METHODS = ('linearized', 'phase', 'mobius', 'lookup', 'circle')


def centered_phase_slope(calibration, frequency_hz, sweep_f=None, sweep_z=None,
                         smooth_window_hz=1000):
    """``dphi/df`` (rad/Hz) of the circle-centred phase at ``frequency_hz``.

    Measured from the sweep when one is given (smoothed, then
    differentiated); otherwise from the calibration's model circle, which is
    only done for a linear (``anl == 0``) calibration.
    """
    if sweep_f is not None:
        f = np.asarray(sweep_f, dtype=float)
        order = np.argsort(f)
        f = f[order]
        z = smooth_sweep(f, np.asarray(sweep_z, dtype=complex)[order], smooth_window_hz)
        phase = np.unwrap(np.angle(to_basis(z, calibration, f, 'centered_rotated')))
        return float(np.interp(frequency_hz, f, np.gradient(phase, f)))
    if calibration.anl != 0.0:
        raise ValueError('a driven (Duffing) calibration needs the sweep to '
                         'measure its phase slope; pass sweep_data')
    x = frequency_hz / calibration.fr - 1.0 + calibration.frequency_origin_fraction
    return float(-4.0 * calibration.Ql / calibration.fr
                 / (1.0 + (2.0 * calibration.Ql * x) ** 2))


def prepare_conversion(frequency_hz, method='linearized', *, calibration=None,
                       sweep_f=None, sweep_z=None, smooth_window_hz=1000):
    """Precompute a fixed-tone conversion: everything except the samples.

    Returns a plain dict for :func:`convert_iq`; keep it when converting the
    same tone again (a live display does, every refresh). Every method gives
    the resonator-side shift (Hz) and the matched dissipation
    ``Delta(1 / (2 Qi))`` relative to the sweep's I/Q at the tone -- the
    resonator's state during the sweep -- or, with no sweep, to the fitted
    state:

    ``'linearized'``  tangent/normal projection on the sweep's local complex
                      gradient. Sweep only; first order; any basis.
    ``'phase'``       centred phase and log-radius over the centred phase
                      slope (:func:`phase_shift_and_dissipation`): the
                      fixed-tone form of the three-point conversion.
                      Calibration for the centring; slope and reference from
                      the sweep, else from the model (linear only).
    ``'mobius'``      exact fitted inversion, Duffing included. Calibration.
    ``'lookup'``      inverts the sweep's whole centred phase curve; follows
                      large signals, NaN beyond the swept span. Sweep; the
                      calibration (optional) sets the centring.
    ``'circle'``      diagnostic: the circle coordinate, and the radial proxy
                      ``Delta(|z_c| / r)`` in place of dissipation.
    """
    if method not in CONVERSION_METHODS:
        raise ValueError(f'method must be one of {CONVERSION_METHODS}, got {method!r}')
    frequency_hz = float(frequency_hz)
    have_sweep = sweep_f is not None and sweep_z is not None
    if have_sweep:
        sweep_f = np.asarray(sweep_f, dtype=float)
        sweep_z = np.asarray(sweep_z, dtype=complex)
    prepared = {'method': method, 'frequency_hz': frequency_hz}
    if method in ('linearized', 'lookup') and not have_sweep:
        raise ValueError(f"method={method!r} needs the calibration sweep")
    if method in ('phase', 'mobius', 'circle') and calibration is None:
        raise ValueError(f"method={method!r} needs a calibration for the tone")
    if method == 'linearized':
        prepared['calibration'] = LinearizedResonatorCalibration.from_sweep(
            sweep_f, sweep_z, frequency_hz, smooth_window_hz=smooth_window_hz)
    elif method == 'lookup':
        prepared['lookup'] = PhaseLookupCalibration.from_sweep(
            sweep_f, sweep_z, frequency_hz, smooth_window_hz=smooth_window_hz,
            calibration=calibration)
    elif method == 'phase':
        A, B = basis_coefficients(calibration, frequency_hz, 'centered_rotated')
        prepared['A'], prepared['B'] = complex(A), complex(B)
        if have_sweep:
            reference = interpolate_complex_trace(sweep_f, sweep_z, frequency_hz)
            prepared['reference'] = complex(A * reference + B)
        else:
            # The model circle's point at the tone (linear calibrations).
            x = frequency_hz / calibration.fr - 1.0 + calibration.frequency_origin_fraction
            prepared['reference'] = complex(
                calibration.radius * np.exp(-2j * np.arctan(2.0 * calibration.Ql * x)))
        prepared['dphi_df'] = centered_phase_slope(
            calibration, frequency_hz, sweep_f if have_sweep else None,
            sweep_z if have_sweep else None, smooth_window_hz)
    else:
        # De-embedding, centring and rotation at this tone are one complex
        # multiply-add, so convert_iq only has to invert the circle.
        A, B = basis_coefficients(calibration, frequency_hz, 'centered_rotated')
        prepared['A'], prepared['B'] = complex(A), complex(B)
        prepared['calibration'] = calibration
        prepared['reference'] = None
        if have_sweep:
            reference = interpolate_complex_trace(sweep_f, sweep_z, frequency_hz)
            prepared['reference'] = calibration.convert_centered_iq(
                A * reference + B, method=method)
    return prepared


def convert_iq(prepared, z, gain=1.0):
    """Convert one tone's raw I/Q with a :func:`prepare_conversion` dict.

    ``gain`` (data over calibration amplitude, see
    :func:`recorded_rx_gain_ratio`) is divided out of ``z`` first, bringing the
    data onto the calibration's scale. Returns ``(resonance_shift_hz,
    matched_dissipation_shift)``: resonator-side Hz and ``Delta(1 / (2 Qi))``
    (the radial proxy for ``'circle'``), relative to the prepared reference.

    Examples
    --------
    >>> f = np.linspace(0.999e9, 1.001e9, 4001)
    >>> sweep = 1 - 0.5 / (1 + 2j * 2e4 * (f - 1e9) / 1e9)
    >>> moved = 1 - 0.5 / (1 + 2j * 2e4 * (1e9 - (1e9 + 100.0)) / 1e9)
    >>> p = prepare_conversion(1e9, 'linearized', sweep_f=f, sweep_z=sweep,
    ...                        smooth_window_hz=0)
    >>> shift, _ = convert_iq(p, moved)
    >>> round(float(shift))
    100
    """
    z = np.asarray(z, dtype=complex) / gain
    method = prepared['method']
    frequency_hz = prepared['frequency_hz']
    if method == 'linearized':
        shift, dissipation_hz = prepared['calibration'].convert_raw_iq(z, fractional=False)
        return shift, dissipation_hz / frequency_hz
    if method == 'lookup':
        shift, dissipation_hz = prepared['lookup'].convert_raw_iq(z, fractional=False)
        return shift, dissipation_hz / frequency_hz
    if method == 'phase':
        shift, dissipation_hz = phase_shift_and_dissipation(
            z * prepared['A'] + prepared['B'], prepared['reference'], prepared['dphi_df'])
        return shift, dissipation_hz / frequency_hz
    calibration = prepared['calibration']
    probe_detuning, loss = calibration.convert_centered_iq(
        z * prepared['A'] + prepared['B'], method=method)
    reference = prepared['reference']
    if reference is not None:
        # Relative to the sweep's I/Q at the tone, resonator-side sign.
        return reference[0] - probe_detuning, loss - reference[1]
    # No sweep: relative to the fitted state, where the probe detuning is
    # f - fr and the loss coordinate is zero.
    return (frequency_hz - calibration.fr) - probe_detuning, loss


def frequency_and_dissipation(z, frequency_hz, method='linearized', *, calibration=None,
                              sweep_f=None, sweep_z=None, smooth_window_hz=1000,
                              gain=1.0):
    """One-shot :func:`prepare_conversion` + :func:`convert_iq` for one tone."""
    prepared = prepare_conversion(
        frequency_hz, method, calibration=calibration, sweep_f=sweep_f,
        sweep_z=sweep_z, smooth_window_hz=smooth_window_hz)
    return convert_iq(prepared, z, gain=gain)


# ---------------------------------------------------------------------------
# Expressing a shift in other units
# ---------------------------------------------------------------------------

def express(quantity, resonance_shift_hz, matched_dissipation_shift, *,
            frequency_hz=None, calibration=None, reference_resonance_hz=None):
    """Turn a (resonance shift, matched dissipation) pair into a named quantity.

    ``frequency_hz`` is the tone frequency the pair was measured at (the
    modulation centre for demodulated data). Linewidth units use the
    calibration's *reference* linewidth ``fr / Ql`` -- fixed, never an
    instantaneous estimate, so a changing loss cannot reshape the signal.
    Absolute quantities (``resonance_frequency_hz``, ``detuning_*``,
    ``internal_loss``) need the resonance at the shift's reference:
    ``reference_resonance_hz``, by default the calibration's operating point
    (right when the reference is the calibration's own sweep).

    Examples
    --------
    >>> cal = ResonatorCalibration(1e9, 2e4, 0.0, 0.5 + 0.0j, 0.5, 0.0, Qi=4e4)
    >>> float(express('resonance_shift_linewidths', 5e3, 0.0, frequency_hz=1e9,
    ...               calibration=cal))
    0.1
    >>> float(express('detuning_hz', 5e3, 0.0, frequency_hz=1e9 + 1e3,
    ...               calibration=cal))
    4000.0
    """
    name = canonical_quantity(quantity)
    shift = np.asarray(resonance_shift_hz, dtype=float)
    dissipation = np.asarray(matched_dissipation_shift, dtype=float)
    if name == 'resonance_shift_hz':
        return shift
    if name == 'matched_dissipation_shift':
        return dissipation
    if name == 'internal_loss_shift':
        return 2.0 * dissipation
    if frequency_hz is None and name not in (
            'resonance_shift_linewidths', 'internal_loss',
            'fractional_internal_loss_shift', 'resonance_frequency_hz'):
        raise ValueError(f'{name!r} needs a frequency: pass the tone frequency '
                         '(reference_tone_frequency)')
    if name == 'fractional_resonance_shift':
        return shift / frequency_hz
    if name == 'matched_dissipation_hz':
        return dissipation * frequency_hz
    known = ('resonance_shift_linewidths', 'matched_dissipation_linewidths',
             'internal_loss', 'fractional_internal_loss_shift',
             'resonance_frequency_hz', 'detuning_hz', 'fractional_detuning',
             'detuning_linewidths')
    if name not in known:
        raise ValueError(f'{name!r} is not a resonance shift or loss quantity')
    if calibration is None:
        raise ValueError(f'{name!r} needs a calibration (a fit) for the tone')
    linewidth = calibration.fr / calibration.Ql
    if name == 'resonance_shift_linewidths':
        return shift / linewidth
    if name == 'matched_dissipation_linewidths':
        return dissipation * frequency_hz / linewidth
    if name in ('internal_loss', 'fractional_internal_loss_shift'):
        if not np.isfinite(calibration.Qi):
            raise ValueError(f'{name!r} needs Qi: build the calibration from a fit')
        if name == 'internal_loss':
            return 1.0 / calibration.Qi + 2.0 * dissipation
        return 2.0 * dissipation * calibration.Qi
    reference = (operating_point_hz(calibration) if reference_resonance_hz is None
                 else reference_resonance_hz)
    resonance = reference + shift
    if name == 'resonance_frequency_hz':
        return resonance
    detuning = resonance - frequency_hz
    if name == 'detuning_hz':
        return detuning
    if name == 'fractional_detuning':
        return detuning / frequency_hz
    return detuning / linewidth


# ---------------------------------------------------------------------------
# Parsed captures
# ---------------------------------------------------------------------------

def capture_tone_keys(samples, tones=None):
    """The ``i_data`` keys of a capture for absolute tone IDs (``None``: all)."""
    available = samples.get('i_data', {})
    keys_by_id = {int(key): key for key in available}
    if len(keys_by_id) != len(available):
        raise ValueError('i_data contains duplicate tone IDs')
    if tones is None or (isinstance(tones, str) and tones == 'all'):
        return [keys_by_id[tone] for tone in sorted(keys_by_id)]
    if isinstance(tones, (int, np.integer, str)):
        tones = [tones]
    keys = []
    for tone in tones:
        try:
            key = keys_by_id[int(tone)]
        except (ValueError, KeyError, TypeError) as exc:
            raise ValueError(f'Tone {tone!r} is not present in the samples') from exc
        if key not in keys:
            keys.append(key)
    return keys


def tone_column(samples, field, tone):
    """One tone's column of a per-tone field in a parsed or demodulated capture.

    The field may be a ``{tone_key: array}`` dict or an array shaped
    ``(n_samples, n_tones)`` / ``(n_tones, n_samples)`` / ``(n_samples,)``
    (tone 0 only).
    """
    tone = int(tone)
    if field not in samples:
        raise ValueError(f'{field!r} is not in the capture')
    values = samples[field]
    if isinstance(values, Mapping):
        for key in (tone, f'{tone:04d}', str(tone)):
            if key in values:
                return np.asarray(values[key])
        raise ValueError(f'{field} is missing tone {tone}')
    arr = np.asarray(values)
    if arr.ndim == 1:
        if tone != 0:
            raise ValueError(f"{field!r} is 1D and contains only tone 0")
        return arr
    if arr.ndim != 2:
        raise ValueError(f"{field!r} must be 1D or 2D, got {arr.ndim}D")
    n_samples = samples.get('num_samples')
    if n_samples is not None and arr.shape[0] == int(n_samples):
        return arr[:, tone]
    if tone < arr.shape[0]:
        return arr[tone]
    return arr[:, tone]


def _unwrap_finite(phase):
    """Unwrap each finite run separately, so a gap cannot poison its tail."""
    phase = np.asarray(phase, dtype=float)
    out = np.full(phase.shape, np.nan)
    valid = np.isfinite(phase)
    edges = np.flatnonzero(np.diff(np.r_[False, valid, False]))
    for start, stop in zip(edges[::2], edges[1::2]):
        out[start:stop] = np.unwrap(phase[start:stop])
    return out


def probe_frequencies(samples, tone, tone_frequency_hz):
    """Per-sample probe frequency (Hz) of one tone in a parsed capture.

    Unmodulated samples sit at ``tone_frequency_hz``. Modulated ones sit at
    their revision's centre plus their point's offset, from the modulation
    state in ``samples['info']`` and, when present, the per-revision centres
    ``parse_stream`` collects from tone updates.
    """
    points = np.asarray(samples.get('modulation_point', ()), dtype=int)
    n = int(samples.get('num_samples', len(points)))
    info = samples.get('info') if isinstance(samples.get('info'), Mapping) else {}
    state = info.get('modulation') if isinstance(info, Mapping) else None
    frequencies = np.full(n, float(tone_frequency_hz))
    if not points.size or not np.any(points > 0) or not isinstance(state, Mapping):
        return frequencies
    entry = next((t for t in state.get('tones', ())
                  if isinstance(t, Mapping) and int(t.get('index', -1)) == int(tone)), None)
    if entry is None or entry.get('offsets_hz') is None:
        return frequencies
    offsets = np.asarray(entry['offsets_hz'], dtype=float)
    centers = np.full(n, float(entry.get('center_hz', tone_frequency_hz)))
    by_revision = samples.get('tone_centers_by_revision') or {}
    if by_revision and 'modulation_revision' in samples:
        revisions = np.asarray(samples['modulation_revision'], dtype=int)
        for revision, centre_map in by_revision.items():
            centre = centre_map.get(str(int(tone)), centre_map.get(int(tone)))
            if centre is not None:
                centers[revisions == int(revision)] = float(centre)
    valid = (points >= 1) & (points <= len(offsets))
    frequencies[valid] = centers[valid] + offsets[points[valid] - 1]
    return frequencies


def convert_samples(samples, quantities, tones=None, *, method='linearized',
                    basis='raw', calibrations=None, sweep_data=None,
                    reference_tone_frequency=None, smooth_window_hz=1000,
                    match_rx_gain='auto', cache=None):
    """Named quantities, per tone, from a parsed capture.

    The one entry point for ``parse_samples`` / ``parse_stream`` dicts, raw or
    demodulated: pick quantities from ``QUANTITIES`` and get back
    ``{quantity: {tone_key: array}}``.

    Parameters
    ----------
    samples : dict
        ``parse_samples`` / ``parse_stream`` output, or a demodulated capture
        (``modulation.demodulate_timestream`` / ``demodulate_stream``), whose
        shifts, dissipation and detunings are used as they are. For
        demodulated data the tone of ``detuning_*`` is the modulation centre.
    quantities : str or sequence of str
        Names or aliases from ``QUANTITIES``.
    tones : int, sequence of int or None, optional
        Absolute tone indices; ``None`` does every tone.
    method : str, optional
        Fixed-tone conversion, see :func:`prepare_conversion`
        (``'linearized'`` by default). Ignored for demodulated data.
    basis : str, optional
        Basis of the plain I/Q views (``i``, ``q``, ``amplitude``,
        ``log_amplitude``, ``phase``, ``unwrapped_phase``); see
        :func:`basis_coefficients`. The ``centered_*`` views use their own.
    calibrations : record, mapping or None, optional
        Per-tone calibrations or fits. ``None`` uses the calibration the
        capture carries (:func:`calibration_from_data`), if any.
    sweep_data : dict or None, optional
        The calibration sweep, for ``'linearized'`` and ``'lookup'`` and the
        reference and slope of ``'phase'``. ``None`` uses the sweep a capture
        saved with its calibration, if any.
    reference_tone_frequency : scalar, mapping, array or None, optional
        Tone frequency override (:func:`resolve_tone_frequency`).
    smooth_window_hz : float, optional
        Sweep smoothing for the sweep-based methods.
    match_rx_gain : {'auto', False} or float, optional
        ``'auto'`` (default) rescales the capture's I/Q onto the calibration's
        RX scale when both recorded chain states are known and differ (with a
        warning); ``False`` never; a number is the data/calibration ratio.
    cache : dict or None, optional
        Pass the same dict on every call to reuse prepared conversions.

    Returns
    -------
    dict
        ``{quantity: {tone_key: numpy.ndarray}}``, each array one per sample.

    Examples
    --------
    >>> t = np.arange(4)
    >>> samples = {'i_data': {'0000': 1.0 + 0 * t}, 'q_data': {'0000': 0.5 + 0 * t},
    ...            'num_samples': 4}
    >>> out = convert_samples(samples, ['amplitude', 'phase'])
    >>> round(float(out['amplitude']['0000'][0]), 4)
    1.118
    """
    if isinstance(quantities, str):
        quantities = [quantities]
    names = list(dict.fromkeys(canonical_quantity(q) for q in quantities))
    if basis not in BASES:
        raise ValueError(f'basis must be one of {BASES}, got {basis!r}')
    cache = {} if cache is None else cache
    if calibrations is None:
        calibrations = calibration_from_data(samples)
    if sweep_data is None:
        sweep_data = samples.get('calibration_sweep')
    # Precomputed shifts (a demodulator's output) are used as they are.
    demodulated = (bool(samples.get('modulation_demodulated'))
                   or 'resonance_shift_hz' in samples)
    modulated = np.any(np.asarray(samples.get('modulation_point', ()), dtype=int) > 0)
    gains = []

    def gain():
        # Resolved once, and only if a conversion against the calibration
        # needs it (raw I/Q views never do).
        if not gains:
            source = calibrations if is_calibration_record(calibrations) else sweep_data
            gains.append(1.0 if demodulated else
                         resolve_rx_gain(match_rx_gain, source, samples.get('info')))
        return gains[0]

    out = {name: {} for name in names}
    for key in capture_tone_keys(samples, tones):
        tone = int(key)
        z = (np.asarray(samples['i_data'][key], dtype=float)
             + 1j * np.asarray(samples['q_data'][key], dtype=float))
        calibration = calibration_for_tone(calibrations, tone)
        sweep_f = sweep_z = None
        if sweep_data is not None:
            sweep_f, sweep_i, sweep_q = sweep_trace(sweep_data, tone)
            sweep_z = np.asarray(sweep_i) + 1j * np.asarray(sweep_q)
        state = {}

        def frequency():
            if 'frequency' not in state:
                state['frequency'] = resolve_tone_frequency(
                    samples, tone, reference_tone_frequency, sweep_f)
            return state['frequency']

        def in_basis(which):
            if which not in state:
                if demodulated and which != 'raw':
                    raise ValueError(
                        'demodulated I/Q is already in the basis it was demodulated '
                        f"in ({samples.get('basis', 'raw')!r}); use basis='raw'")
                if which == 'raw':
                    state[which] = z          # counts as recorded: no gain
                else:
                    if not isinstance(calibration, ResonatorCalibration):
                        raise ValueError(f"basis {which!r} needs a calibration for tone {tone}")
                    state[which] = to_basis(z, calibration, probe_frequencies(
                        samples, tone, frequency()), which, gain())
            return state[which]

        def shifts():
            if 'shifts' not in state:
                if demodulated:
                    shift = tone_column(samples, 'resonance_shift_hz', tone).astype(float)
                    dissipation = np.full(shift.shape, np.nan)
                    # 'dissipation' in results saved before the rename.
                    for field in ('matched_dissipation_shift', 'dissipation'):
                        if field in samples:
                            dissipation = tone_column(samples, field, tone)
                            break
                    state['shifts'] = (shift, np.asarray(dissipation, dtype=float))
                else:
                    if modulated:
                        raise ValueError(
                            'frequency and dissipation of modulated I/Q need '
                            'demodulating first (modulation.demodulate_stream)')
                    if (method in ('phase', 'mobius', 'circle')
                            and not isinstance(calibration, ResonatorCalibration)):
                        raise ValueError(
                            f'method={method!r} requires a calibration for tone {tone}')
                    cache_key = ('prepared', tone, method, frequency(), id(calibrations),
                                 id(sweep_data), smooth_window_hz)
                    if cache_key not in cache:
                        cache[cache_key] = prepare_conversion(
                            frequency(), method, calibration=calibration,
                            sweep_f=sweep_f, sweep_z=sweep_z,
                            smooth_window_hz=smooth_window_hz)
                    state['shifts'] = convert_iq(cache[cache_key], z, gain())
            return state['shifts']

        for name in names:
            if name in _IQ_VIEWS:
                zb = in_basis(basis)
                value = {'i': zb.real, 'q': zb.imag, 'amplitude': np.abs(zb)}.get(name)
                if name == 'log_amplitude':
                    with np.errstate(divide='ignore', invalid='ignore'):
                        value = 20.0 * np.log10(np.abs(zb))
                elif name == 'phase':
                    value = np.angle(zb)
                elif name == 'unwrapped_phase':
                    value = _unwrap_finite(np.angle(zb))
            elif name in _FIXED_BASIS_VIEWS:
                zb = in_basis(_FIXED_BASIS_VIEWS[name])
                value = (np.abs(zb) / calibration.radius if name == 'centered_radius'
                         else _unwrap_finite(np.angle(zb)))
            elif name in _DEMOD_ONLY or (demodulated and name in (
                    'detuning_hz', 'detuning_linewidths', 'fractional_detuning')):
                if not demodulated:
                    raise ValueError(f'{name} needs demodulated modulation data')
                field = {'detuning_hz': 'center_detuning_hz',
                         'detuning_linewidths': 'center_detuning_linewidths'}.get(name, name)
                if name == 'fractional_detuning':
                    value = tone_column(samples, 'center_detuning_hz', tone) / frequency()
                else:
                    value = tone_column(samples, field, tone)
            elif demodulated and name == 'resonance_frequency_hz':
                value = tone_column(samples, 'resonance_frequency_hz', tone)
            else:
                shift, dissipation = shifts()
                try:
                    tone_hz = frequency()
                except ValueError:
                    tone_hz = None      # express says so if this quantity needs it
                value = express(name, shift, dissipation, frequency_hz=tone_hz,
                                calibration=calibration if isinstance(
                                    calibration, ResonatorCalibration) else None)
            out[name][key] = np.asarray(value, dtype=float)
    return out
