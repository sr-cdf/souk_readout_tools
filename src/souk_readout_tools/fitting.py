"""Resonator fitting for complex S21 sweeps.

The public fitters in this module fit complex MKID transmission data to a
notch resonator model with optional Duffing-style nonlinearity. Parameters are
reported in physical coordinates:

    fr, Qi, Qc, phi, a, alpha, tau[, anl]

where ``fr`` is the resonance frequency, ``Qi`` is the internal quality
factor, ``Qc`` is the real coupling quality factor, ``phi`` is the impedance
mismatch angle, ``a`` is the complex-gain amplitude, ``alpha`` is the
complex-gain phase, ``tau`` is the cable delay, and ``anl`` is the optional
Duffing nonlinearity parameter. The model functions are ``s21_model`` and
``s21_model_centered_delay``; the latter is used internally because centering
the cable-delay phase near the sweep frequency improves optimizer
conditioning.

Q conventions
-------------

The fitted coupling parameter is the real ``Qc`` in
``Qe = Qc * (1 + 1j * tan(phi))``. The loaded quality factor is
``1 / Ql = 1 / Qi + real(1 / Qe)``. Use ``Qc`` in parameter dictionaries such
as ``initial_guess``, ``param_bounds`` and ``param_fixed``. ``Qe`` and
``Qc_abs = abs(Qe)`` are returned for reporting; ``Qc_abs`` is not itself a
fit parameter.

Fitting workflow
----------------

``fit_resonance`` prepares one targeted sweep by masking non-finite samples,
sorting by frequency, building a robust initial guess from the magnitude dip
and phase slope, then running ``scipy.optimize.least_squares`` with scaled
parameters. ``fit_resonance(..., nonlinear=True)`` delegates to
``fit_resonance_nonlinear``.

``fit_resonance_nonlinear`` first performs a linear fit on the full sweep to
seed the Duffing fit. The nonlinear ``anl`` parameter is optimized internally
as ``log(anl)`` for stable per-decade sensitivity, then converted back to
linear units in the returned result. For strongly bistable/high-drive shapes,
``try_harder`` and ``try_even_harder`` probe extra ``(fr, Qi, anl)`` seeds and
return the lowest-cost candidate. ``autodetect_high_anl=True`` runs the small
probe automatically when the measured dip has a clear cliff/shoulder shape and
the first nonlinear refinement appears to be in the wrong basin.

``batch_fit`` is the server-sweep adapter. It accepts a sweep-data dictionary
from the readout server, fits each targeted trace by default, skips tones
marked as blind monitors unless requested otherwise, and records each returned
fit's original ``tone_index``. It can also fit user-supplied resonance windows
or auto-find/window resonances in a concatenated full sweep.

``fit_sweep_stack`` is the array-stack adapter. It accepts already-windowed
frequency/S21/error arrays, treats each row or column as one independent fit,
and returns a list of ``FitResult`` objects in stack order. Stack rows are fit
independently today: the helper does not carry one row's result into the next
row as an ``initial_guess``. For ordered repeat measurements of the same
resonator, you can call ``fit_resonance`` / ``fit_resonance_nonlinear`` in a
loop and pass the previous ``FitResult`` as ``initial_guess``.

Both collection helpers support process-based parallelism via ``n_jobs``:
``1`` is serial, ``-1`` uses all visible CPUs, and ``-2`` uses all but one.
``verbose=True`` prints compact progress and throughput; ``verbose=2`` prints
one line per completed fit. Returned lists are kept in input order even though
parallel progress is completion-order.

Sub-sampling
------------

For long or densely sampled nonlinear sweeps, ``subsample=True`` can reduce
runtime substantially. It keeps every point near the resonator dip and samples
the tails with density weighted by phase-gradient, so the optimizer sees the
informative part of the sweep without carrying every smooth baseline point.
The fit is evaluated back on the full input arrays in the returned
``FitResult``.

Uncertainties and weights
-------------------------

``z_err`` is the measurement uncertainty used for covariance, parameter
uncertainties, weighted RMS, and reduced chi-square. Parsed server sweep
data stores ``sweep_ei``/``sweep_eq`` as the standard error of the averaged
point already, so fitting helpers consume those values as-is; do not scale
them again by ``samples_per_point``. The optimizer itself is unweighted by
default. Pass ``optimizer_z_err=z_err`` or
``use_error_weights=True`` only when the measurement uncertainty is also a
good numerical weight for the least-squares objective. ``error_weight_power``
can soften those optimizer weights, for example ``0.5``.

Useful result fields
--------------------

Each fit returns a ``FitResult`` with fitted parameters, derived quantities,
diagnostic arrays, optimizer results, and timing/count fields. For nonlinear
fits with fallback probes, ``nonlinear_nfev`` and
``nonlinear_fit_duration_s`` include all nonlinear attempts, not just the
winning candidate. ``nfev`` is the total of the linear seed and all nonlinear
refinements. The ``empirical_*`` fields are measured dip estimates computed
before optimisation; they are useful sanity checks and fallback diagnostics,
not least-squares fit parameters. If ``min_dip_depth_db`` is set and the
empirical dip depth falls below it, the fitters return ``success=False`` with
``noise_only=True`` and leave ``nfev=0`` so batch/power-sweep tools can exclude
the row cleanly.

Examples
--------

Fit one targeted sweep:

    from souk_readout_tools.fitting import fit_resonance, evaluate_fit

    z = sweep_i + 1j * sweep_q
    e = sweep_ei + 1j * sweep_eq
    fit = fit_resonance(
        f, z, z_err=e, nonlinear=True, use_error_weights=True,
        error_weight_power=0.5, subsample=True,
        param_bounds={"Qi": (1e3, 1e8), "phi": (-0.5, 0.5)},
        param_fixed={"tau": 0.0},
    )
    z_model = evaluate_fit(f, fit)

Fit already-windowed traces for the same KID over many powers, independently:

    from souk_readout_tools.fitting import fit_sweep_stack, extract_parameters

    files = [
        "sweep_-10dB.npy", "sweep_-8dB.npy", "sweep_-6dB.npy",
        "sweep_-4dB.npy", "sweep_-2dB.npy", "sweep_0dB.npy",
    ]
    kid_index = 0

    f_stack, z_stack, e_stack = [], [], []
    for filename in files:
        data = np.load(filename, allow_pickle=True).item()
        z_all = data["sweep_i"] + 1j * data["sweep_q"]
        e_all = data["sweep_ei"] + 1j * data["sweep_eq"]
        f_stack.append(data["sweep_f"][:, kid_index])
        z_stack.append(z_all[:, kid_index])
        e_stack.append(e_all[:, kid_index])

    fits = fit_sweep_stack(
        np.asarray(f_stack), np.asarray(z_stack),
        z_err_stack=np.asarray(e_stack), nonlinear=True, n_jobs=-1,
        verbose=True, use_error_weights=True, error_weight_power=0.5,
        subsample=True,
    )
    params = extract_parameters(fits)

Auto-find and fit resonances in a full sweep dictionary:

    fits = batch_fit(
        sweep_data, nonlinear=True, n_jobs=-1, verbose=True,
        window_fwhm=10.0, use_error_weights=True, subsample=True,
        find_resonances=True,
    )
"""

from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import dataclass, field
import os
import time

# These fits run many small least-squares problems. Let the outer
# n_jobs/process pool own CPU parallelism instead of nested numeric worker
# pools inside every fit.
for _cpu_pool_env_var in (
    "OMP_NUM_THREADS",
    "OPENBLAS_NUM_THREADS",
    "MKL_NUM_THREADS",
    "NUMEXPR_NUM_THREADS",
    "VECLIB_MAXIMUM_THREADS",
):
    os.environ.setdefault(_cpu_pool_env_var, "1")

import numpy as np
from scipy.optimize import OptimizeResult, least_squares

from .resonator import (
    estimate_resonance_empirical,
    model_deembed,
    model_phase_center_geometry,
    rotate_around_point,
)


LINEAR_NAMES = ("fr", "Qi", "Qc", "phi", "a", "alpha", "tau")
NONLINEAR_NAMES = LINEAR_NAMES + ("anl",)
DERIVED_PARAMETER_NAMES = {"Qc_abs"}
FIT_SUMMARY_KEYS = (
    "fr",
    "Ql",
    "Qi",
    "Qc",
    "Qc_abs",
    "phi",
    "a",
    "alpha",
    "tau",
    "anl",
    "nonlinear_detuning_hz",
    "empirical_fr",
    "empirical_linewidth_hz",
    "empirical_Ql",
    "empirical_Qc",
    "empirical_Qi",
    "empirical_dip_depth_db",
    "empirical_skew",
    "noise_only",
    "residual_rms",
    "weighted_rms",
    "reduced_chi2",
    "success",
    "nfev",
    "fit_duration_s",
)
FIT_UNCERTAINTY_KEYS = ("fr", "Ql", "Qi", "Qc", "phi", "a", "alpha", "tau", "anl")
# Per-parameter normalisation floor (order: fr, Qi, Qc, phi, a, alpha, tau);
# raising an entry makes the optimiser take coarser steps in that parameter,
# lowering it takes finer steps (more sensitive, but slower and noisier).
SCALE_FLOORS = np.array([1.0, 1e3, 1e3, 1.0, 1e-3, 1.0, 1e-8])
# Per-parameter finite-difference step for the numeric-Jacobian path; larger
# gives a coarser, more noise-robust gradient, smaller a more precise gradient
# that can lock onto noise-induced local minima.
DIFF_STEPS = np.array([1e-7, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4])

# Whether linear fits use the closed-form Jacobian (faster, see
# _s21_linear_jacobian); set False to fall back to finite differences if a
# pathological case ever needs it. Nonlinear (Duffing) fits always use finite
# differences -- differentiating through the bistable branch selection is not
# worth the risk near saddle nodes.
_USE_ANALYTIC_LINEAR_JAC = True

# Lower clamp on anl before it is reparameterised as log(anl); raise it to treat
# weaker nonlinearity as effectively linear, lower it to let the optimiser chase
# anl values that sit below the noise floor of any real measurement.
_ANL_MIN = 1e-8

# Dip-shape thresholds that trigger the high-anl probe. Raising a value makes the
# probe fire less often (fewer extra fits, but more risk of missing a strongly
# bistable resonator); lowering it fires the probe more readily.
_HIGH_ANL_WIDTH_RATIO = 30.0      # smooth-shoulder:sharp-cliff width ratio counted as a strong cliff
_HIGH_ANL_ASYMMETRY = 0.94        # dip asymmetry (0..1) counted as a strong cliff
_MODERATE_ANL_WIDTH_RATIO = 12.0  # width ratio for a "moderate" cliff (probes only if the first fit is also poor)
_MODERATE_ANL_ASYMMETRY = 0.88    # asymmetry for a "moderate" cliff
# First-fit residual, as a fraction of the dip feature size, above which a
# moderate-cliff fit is judged poor enough to justify the probe; raise to probe
# less, lower to probe more.
_AUTODETECT_HIGH_ANL_BAD_FIT_FRACTION = 0.02

# Seed grids for the high-anl probe: each (anl, Qi, fr-perturbation) triple is
# one extra fit, so adding entries widens the basin search at a linear cost in
# time. The small grid runs automatically; the exhaustive grid only when
# try_even_harder=True.
_HIGH_ANL_PROBE_ANL_SEEDS = (0.5, 5.0, 20.0)
_HIGH_ANL_PROBE_QI_SEEDS = (1e5, 5e5, 5e6)
_HIGH_ANL_PROBE_FR_PERTURB_KHZ = (0.0,)
_HIGH_ANL_EXHAUSTIVE_ANL_SEEDS = (0.5, 2.0, 5.0, 10.0, 20.0)
_HIGH_ANL_EXHAUSTIVE_QI_SEEDS = (1e5, 5e5, 1e6, 5e6)
_HIGH_ANL_EXHAUSTIVE_FR_PERTURB_KHZ = (-1.0, 0.0, 1.0)

# Thresholds that let nonlinear='auto' accept the linear fit as final (see
# _linear_fit_is_sufficient). Tuned on a real -115..-85 dBm power sweep where
# linear-sufficient fits sat at dip_excess ~1.0 and genuinely nonlinear ones at
# ~2.4+. Raising either threshold keeps more fits linear (faster, but risks
# missing weak nonlinearity); lowering either runs the Duffing fit more often
# (safer, slower).
_AUTO_DIP_EXCESS_MAX = 1.5  # accept linear when the dip is misfit < this multiple of the baseline
_AUTO_ASYMMETRY_MAX = 0.15  # accept linear only when the dip asymmetry (0..1) is below this

@dataclass
class FitResult:
    """Container for one fitted resonance and the arrays used to make it."""
    tone_index: int = -1
    is_blind: bool = False
    fr: float = np.nan
    Ql: float = np.nan
    Qi: float = np.nan
    Qc: float = np.nan
    phi: float = np.nan
    a: float = np.nan
    alpha: float = np.nan
    tau: float = np.nan
    Qe: complex = complex(np.nan, np.nan)
    Qc_abs: float = np.nan
    empirical_fr: float = np.nan
    empirical_linewidth_hz: float = np.nan
    empirical_Ql: float = np.nan
    empirical_Qc: float = np.nan
    empirical_Qi: float = np.nan
    empirical_dip_depth_db: float = np.nan
    empirical_skew: float = np.nan
    noise_only: bool = False
    noise_reason: str = ""
    iq_center: complex = complex(np.nan, np.nan)
    iq_radius: float = np.nan
    iq_center_deembed: complex = complex(np.nan, np.nan)
    iq_radius_deembed: float = np.nan
    residual_rms: float = np.nan
    weighted_rms: float = np.nan
    optimizer_weighted_rms: float = np.nan
    reduced_chi2: float = np.nan
    success: bool = False
    message: str = ""
    # Populated only by nonlinear='auto': the linear-sufficiency diagnostics
    # (dip_excess, asymmetry, ...) and whether the Duffing fit was skipped.
    linear_sufficiency: object = field(default=None, repr=False)
    anl: float = 0.0
    sweep_direction: str = "up"
    nonlinear_detuning_hz: float = 0.0
    fr_minus_nonlinear_detuning: float = np.nan
    nfev: int = 0
    linear_nfev: int = 0
    nonlinear_nfev: int = 0
    fit_duration_s: float = np.nan
    linear_fit_duration_s: float = np.nan
    nonlinear_fit_duration_s: float = np.nan
    optimizer_cost: float = np.nan
    linear_cost: float = np.nan
    nonlinear_cost: float = np.nan
    f_reference: float = np.nan
    parameter_names: tuple = field(default_factory=tuple)
    parameter_covariance: object = field(default=None, repr=False)
    parameter_uncertainties: object = field(default=None, repr=False)
    p: object = field(default=None, repr=False)
    initial_guess: object = field(default=None, repr=False)
    opt: object = field(default=None, repr=False)
    opt_linear: object = field(default=None, repr=False)
    opt_nonlinear: object = field(default=None, repr=False)
    optimizer_result: object = field(default=None, repr=False)
    optimizer_result_linear: object = field(default=None, repr=False)
    optimizer_result_nonlinear: object = field(default=None, repr=False)
    linear_fit_result: object = field(default=None, repr=False)
    f_data: object = field(default=None, repr=False)
    z_data: object = field(default=None, repr=False)
    z_err_data: object = field(default=None, repr=False)
    optimizer_z_err_data: object = field(default=None, repr=False)
    z_fit: object = field(default=None, repr=False)
    z_data_deembed: object = field(default=None, repr=False)
    z_fit_deembed: object = field(default=None, repr=False)
    z_data_deembed_rotated: object = field(default=None, repr=False)
    z_fit_deembed_rotated: object = field(default=None, repr=False)
    z_data_deembed_phase_centered: object = field(default=None, repr=False)
    z_fit_deembed_phase_centered: object = field(default=None, repr=False)
    deembed_rotation_angle: float = np.nan
    phase_center_rotation_angle: float = np.nan


def wrap_phase(x):
    """Wrap a phase to [-pi, pi) for stable public parameters.

    ``x`` is a phase (or array of phases) in radians.
    """
    return (x + np.pi) % (2.0 * np.pi) - np.pi


def _alpha_to_centered(alpha, tau, f0):
    """Convert public alpha to the optimiser's phase-at-f0 coordinate."""
    return wrap_phase(alpha - 2.0 * np.pi * f0 * tau)


def _alpha_to_public(alpha0, tau, f0):
    """Convert phase-at-f0 alpha back to the public phase coordinate."""
    return wrap_phase(alpha0 + 2.0 * np.pi * f0 * tau)


def complex_coupling_q(Qc, phi):
    """Return complex coupling Qe from the fitted real Qc and mismatch angle.

    ``Qc`` is not ``abs(Qe)`` except when ``phi == 0``.
    """
    return Qc * (1.0 + 1j * np.tan(phi))


def loaded_q(Qi, Qc, phi):
    """Return loaded Q from internal ``Qi`` and the complex coupling.

    ``Qc`` is the fitted real coupling Q and ``phi`` the coupling mismatch
    angle (radians); see :func:`complex_coupling_q`.
    """
    return 1.0 / (1.0 / Qi + np.real(1.0 / complex_coupling_q(Qc, phi)))


def duffing_y(y0, anl, sweep_direction="up"):
    """Solve the Duffing cubic on the requested sweep branch.

    Parameters
    ----------
    y0 : array-like
        Linear detuning coordinate ``x / Qr_inv``.
    anl : array-like
        Duffing nonlinearity parameter (broadcast to ``y0``).
    sweep_direction : {'up', 'down'}, optional
        Which bistable branch to take (default ``'up'``).
    """
    y0 = np.asarray(y0, float)
    anl = np.broadcast_to(np.asarray(anl, float), y0.shape)
    A, B, C, D = 4.0, -4.0 * y0, 1.0, -y0 - anl
    D0 = B * B - 3.0 * A * C
    D1 = 2.0 * B**3 - 9.0 * A * B * C + 27.0 * A * A * D
    Ct = 0.5 * (D1 + np.emath.sqrt(D1**2 - 4.0 * D0**3))
    Ct = np.where(np.abs(Ct) < 1e-48, 1e-24 + 0j, Ct)
    Cc, xi = Ct ** (1.0 / 3.0), -0.5 + 0.5j * np.sqrt(3.0)
    r1 = -(B + Cc + D0 / Cc) / (3.0 * A)
    r2 = -(B + xi * Cc + D0 / (xi * Cc)) / (3.0 * A)
    r3 = -(B + xi * xi * Cc + D0 / (xi * xi * Cc)) / (3.0 * A)
    if sweep_direction == "up":
        return np.where(np.isclose(r1.imag, 0.0, atol=1e-12),
                        r1.real, np.maximum(r2.real, r3.real))
    if sweep_direction == "down":
        real23 = np.isclose(r2.imag, 0.0, atol=1e-12) | np.isclose(r3.imag, 0.0, atol=1e-12)
        return np.where(real23, np.maximum(r2.real, r3.real), r1.real)
    raise ValueError('sweep_direction must be "up" or "down"')


def s21_model(f, fr, Qi, Qc, phi, a, alpha, tau, anl=0.0, sweep_direction="up"):
    """Evaluate the physical S21 model in returned-parameter coordinates.

    Parameters
    ----------
    f : array-like
        Frequencies (Hz) to evaluate at.
    fr : float
        Resonance frequency (Hz).
    Qi : float
        Internal quality factor.
    Qc : float
        Real coupling quality factor (not ``abs(Qe)`` unless ``phi == 0``).
    phi : float
        Coupling impedance-mismatch angle (radians).
    a : float
        Overall gain magnitude of the cable/electronics envelope.
    alpha : float
        Overall gain phase (radians) at ``f = 0``.
    tau : float
        Cable delay (s), giving a linear phase slope ``-2*pi*f*tau``.
    anl : float, optional
        Duffing nonlinearity; ``0`` (default) gives the linear model.
    sweep_direction : {'up', 'down'}, optional
        Bistable branch used when ``anl != 0`` (default ``'up'``).
    """
    f = np.asarray(f, float)
    Qe = complex_coupling_q(Qc, phi)
    x = (f - fr) / fr
    env = a * np.exp(1j * alpha) * np.exp(-2j * np.pi * f * tau)
    if float(anl) == 0.0:
        res = 1.0 - (1.0 / Qe) / (1.0 / Qi + 1.0 / Qe + 2j * x)
    else:
        # Preserve the established nonlinear fit convention. For phi != 0,
        # this real-Qr denominator means the anl -> 0 limit is not identical
        # to the asymmetric linear branch above.
        Qr_inv = 1.0 / Qi + np.real(1.0 / Qe)
        y = duffing_y(x / Qr_inv, anl, sweep_direction)
        res = 1.0 - (1.0 / Qe) / (Qr_inv * (1.0 + 2j * y))
    return env * res


def s21_model_centered_delay(f, fr, Qi, Qc, phi, a, alpha0, tau,
                             anl=0.0, f0=0.0, sweep_direction="up"):
    """Evaluate S21 with gain phase referenced to ``f0`` for better conditioning.

    Same model as :func:`s21_model` (see it for ``f``, ``fr``, ``Qi``,
    ``Qc``, ``phi``, ``a``, ``tau``, ``anl``, ``sweep_direction``), except the
    gain phase ``alpha0`` is defined at ``f = f0`` rather than at ``f = 0``,
    which decorrelates it from ``tau`` during fitting.  ``f0`` is the
    reference frequency (Hz, usually the dip centre).
    """
    f = np.asarray(f, float)
    Qe = complex_coupling_q(Qc, phi)
    x = (f - fr) / fr
    env = a * np.exp(1j * alpha0) * np.exp(-2j * np.pi * (f - f0) * tau)
    if float(anl) == 0.0:
        res = 1.0 - (1.0 / Qe) / (1.0 / Qi + 1.0 / Qe + 2j * x)
    else:
        Qr_inv = 1.0 / Qi + np.real(1.0 / Qe)
        y = duffing_y(x / Qr_inv, anl, sweep_direction)
        res = 1.0 - (1.0 / Qe) / (Qr_inv * (1.0 + 2j * y))
    return env * res


def _s21_linear_jacobian(f, fr, Qi, Qc, phi, a, alpha0, tau, f0=0.0):
    """Analytic d(S21)/d(param) for the linear centred-delay model.

    Returns a complex array of shape ``(len(f), 7)`` whose columns are the
    derivatives of :func:`s21_model_centered_delay` (with ``anl = 0``) with
    respect to ``fr, Qi, Qc, phi, a, alpha0, tau`` in that order -- the same
    parameter order as :data:`LINEAR_NAMES`.

    Writing ``u = 1/Qe``, ``c = 1/Qi + 2j*x`` and ``g = c + u`` (so the
    resonator factor is ``res = 1 - u/g = c/g``) the derivatives are all closed
    form; the optimiser uses this instead of finite-differencing the model.
    """
    f = np.asarray(f, float)
    T = 1.0 + 1j * np.tan(phi)
    Qe = Qc * T
    u = 1.0 / Qe
    x = (f - fr) / fr
    env = a * np.exp(1j * alpha0) * np.exp(-2j * np.pi * (f - f0) * tau)
    c = 1.0 / Qi + 2j * x
    g = c + u
    g2 = g * g
    res = c / g
    s21 = env * res
    sec2 = 1.0 / np.cos(phi) ** 2
    d = np.empty((f.size, 7), complex)
    d[:, 0] = env * (2j * u / g2) * (-f / fr ** 2)   # d/d fr  (via x)
    d[:, 1] = env * (-u / (Qi * Qi * g2))            # d/d Qi
    d[:, 2] = env * (c * u / (Qc * g2))              # d/d Qc
    d[:, 3] = env * (1j * sec2 * c * u / (g2 * T))   # d/d phi
    d[:, 4] = s21 / a                                # d/d a
    d[:, 5] = 1j * s21                               # d/d alpha0
    d[:, 6] = s21 * (-2j * np.pi * (f - f0))         # d/d tau
    return d


def _parameter_dict(values):
    """Return a parameter dictionary keyed by fitted parameter name."""
    if values is None:
        return None
    if isinstance(values, FitResult):
        return {n: getattr(values, n) for n in NONLINEAR_NAMES if hasattr(values, n)}
    if not isinstance(values, dict):
        raise TypeError("parameter values must be dicts keyed by parameter name")
    derived = DERIVED_PARAMETER_NAMES & set(values)
    if derived:
        raise ValueError(
            "Qc_abs is derived from Qc and phi; use Qc in parameter dictionaries."
        )
    return dict(values)


def _select_names(values, names):
    """Keep only entries relevant to a particular linear/nonlinear fit."""
    values = _parameter_dict(values)
    return None if values is None else {k: v for k, v in values.items() if k in names}


def _prepare_z_error(z_err, shape):
    """Return positive sigma_I and sigma_Q arrays in readout-server convention.

    When ``z_err`` comes from parsed server sweep data, ``sweep_ei`` and
    ``sweep_eq`` are already the standard error of the averaged sweep point,
    matching the uncertainty of ``sweep_i``/``sweep_q`` used by the fit.
    """
    if z_err is None or z_err is False:
        return None
    if isinstance(z_err, dict):
        if "sweep_ei" in z_err and "sweep_eq" in z_err:
            ei, eq = z_err["sweep_ei"], z_err["sweep_eq"]
        elif "ei" in z_err and "eq" in z_err:
            ei, eq = z_err["ei"], z_err["eq"]
        elif "err_i" in z_err and "err_q" in z_err:
            ei, eq = z_err["err_i"], z_err["err_q"]
        elif "e" in z_err:
            z_err = z_err["e"]
            ei, eq = np.asarray(z_err).real, np.asarray(z_err).imag
        else:
            raise ValueError("z_err dict must contain sweep_ei/sweep_eq, ei/eq, err_i/err_q, or e")
    elif isinstance(z_err, (tuple, list)) and len(z_err) == 2:
        ei, eq = z_err
    else:
        err = np.asarray(z_err)
        ei, eq = (err.real, err.imag) if np.iscomplexobj(err) else (err, err)
    ei, eq = np.asarray(ei), np.asarray(eq)
    if ei.shape != shape and ei.size == int(np.prod(shape)):
        ei = ei.reshape(shape)
    if eq.shape != shape and eq.size == int(np.prod(shape)):
        eq = eq.reshape(shape)
    ei = np.broadcast_to(np.abs(ei), shape).astype(float, copy=True)
    eq = np.broadcast_to(np.abs(eq), shape).astype(float, copy=True)
    good = np.r_[ei[np.isfinite(ei) & (ei > 0.0)],
                 eq[np.isfinite(eq) & (eq > 0.0)]]
    fill = float(np.median(good)) if good.size else 1.0
    ei[~(np.isfinite(ei) & (ei > 0.0))] = fill
    eq[~(np.isfinite(eq) & (eq > 0.0))] = fill
    return ei, eq


def _pack_z_error(z_error):
    """Pack an I/Q uncertainty tuple into a complex array for result storage."""
    return None if z_error is None else z_error[0] + 1j * z_error[1]


def _resolve_optimizer_z_error(optimizer_z_err, z_error, shape):
    """Choose the separate numerical weighting used by least_squares."""
    if optimizer_z_err is None or optimizer_z_err is False:
        return None
    if isinstance(optimizer_z_err, str):
        key = optimizer_z_err.lower()
        if key in ("none", "false", "unweighted"):
            return None
        if key in ("z_err", "same", "measurement"):
            return z_error
    if optimizer_z_err is True:
        return z_error
    return _prepare_z_error(optimizer_z_err, shape)


def _error_vector(z_error, power=1.0):
    """Return concatenated I/Q weights matching the residual vector."""
    if z_error is None:
        return None
    return np.r_[z_error[0] ** power, z_error[1] ** power]


def _weighted_rms(diff, z_error, power=1.0):
    """Return RMS of I/Q residuals, optionally divided by I/Q errors."""
    if z_error is None:
        return float(np.sqrt(np.mean(np.abs(diff) ** 2)))
    r = np.r_[diff.real / (z_error[0] ** power), diff.imag / (z_error[1] ** power)]
    return float(np.sqrt(np.mean(r * r)))


class InsufficientSamplesError(ValueError):
    """Raised when a sweep trace has too few finite samples to fit.

    Subclasses ``ValueError`` so existing ``except ValueError`` callers keep
    working, while :func:`fit_resonance` / :func:`fit_resonance_nonlinear`
    catch this specific case and return a ``noise_only`` result instead of
    propagating -- so a mostly-NaN or deliberately skipped trace (e.g. a
    power-sweep step that could not be set) degrades to a failed fit rather
    than aborting a whole batch.
    """


def _prepare_arrays(f, z, z_err=None, optimizer_z_err=None):
    """Mask non-finite samples, sort by frequency, and carry errors along."""
    f = np.asarray(f, float).ravel()
    z = np.asarray(z, complex).ravel()
    if f.size != z.size:
        raise ValueError("f and z must contain the same number of samples")
    z_error = _prepare_z_error(z_err, z.shape)
    opt_error = _resolve_optimizer_z_error(optimizer_z_err, z_error, z.shape)
    good = np.isfinite(f) & np.isfinite(z.real) & np.isfinite(z.imag)
    if z_error is not None:
        good &= np.isfinite(z_error[0]) & np.isfinite(z_error[1])
    if opt_error is not None:
        good &= np.isfinite(opt_error[0]) & np.isfinite(opt_error[1])
    f, z = f[good], z[good]
    if f.size < 5:
        raise InsufficientSamplesError(
            "at least five finite samples are required for a resonator fit")
    order = np.argsort(f)
    f, z = f[order], z[order]
    if z_error is not None:
        z_error = (z_error[0][good][order], z_error[1][good][order])
    if opt_error is not None:
        opt_error = (opt_error[0][good][order], opt_error[1][good][order])
    return f, z, z_error, opt_error


def _finite_positive(value):
    try:
        value = float(value)
    except (TypeError, ValueError):
        return False
    return np.isfinite(value) and value > 0.0


def _clean_q_guess(value, fallback):
    if _finite_positive(value):
        return float(np.clip(value, 10.0, 1e8))
    return fallback


def _guess_params(f, z, empirical=None):
    """Estimate a compact, robust starting point from a targeted sweep."""
    phase = np.unwrap(np.angle(z))
    tau = -np.median(np.gradient(phase, f)) / (2.0 * np.pi)
    f0 = (
        float(empirical.fr)
        if empirical is not None and np.isfinite(empirical.fr)
        else float(f[np.argmin(np.abs(z))])
    )
    zn = z * np.exp(2j * np.pi * (f - f0) * tau)
    nedge = max(3, len(f) // 10)
    base = np.r_[zn[:nedge], zn[-nedge:]].mean()
    a = max(float(abs(base)), 1e-12)
    alpha0 = wrap_phase(np.angle(base))
    mag = np.abs(zn)
    i = int(np.argmin(mag))
    half = 0.5 * (mag[i] + np.max(mag))
    below = np.where(mag < half)[0]
    df = float(np.median(np.diff(f)))
    bw = f[below[-1]] - f[below[0]] if below.size > 1 else (f[-1] - f[0]) / 10.0
    Ql = f[i] / max(float(abs(bw)), abs(df), 1.0)
    Qc = Ql / max(1e-6, 1.0 - mag[i] / max(np.max(mag), 1e-30))
    Qi_inv = 1.0 / Ql - 1.0 / Qc
    Qi = 1.0 / Qi_inv if Qi_inv > 0.0 else 2.0 * Ql
    if empirical is not None:
        Ql = _clean_q_guess(empirical.Ql, Ql)
        Qc = _clean_q_guess(empirical.Qc, Qc)
        Qi = _clean_q_guess(empirical.Qi, Qi)
    return np.array([f0, Qi, Qc, 1e-3, a, alpha0, tau], float)


def _apply_initial_guess(p, names, initial_guess, f0):
    """Override starting values from a parameter dictionary."""
    initial_guess = _parameter_dict(initial_guess)
    if initial_guess is None:
        return p
    unknown = set(initial_guess) - set(names)
    if unknown:
        raise ValueError(f"unknown initial_guess parameter(s): {sorted(unknown)}")
    for i, name in enumerate(names):
        if name in initial_guess and name != "alpha":
            p[i] = float(initial_guess[name])
    if "alpha" in initial_guess:
        tau = p[names.index("tau")]
        p[names.index("alpha")] = _alpha_to_centered(
            float(initial_guess["alpha"]), tau, f0)
    return p


def _apply_bounds(lower, upper, names, param_bounds, f0, tau0):
    """Apply named box bounds, translating alpha to centred alpha0."""
    param_bounds = _parameter_dict(param_bounds)
    if param_bounds is None:
        return np.array(lower, float), np.array(upper, float)
    unknown = set(param_bounds) - set(names)
    if unknown:
        raise ValueError(f"unknown param_bounds parameter(s): {sorted(unknown)}")
    lower, upper = list(lower), list(upper)
    for name, pair in param_bounds.items():
        if pair is None:
            continue
        if len(pair) != 2:
            raise ValueError(f"param_bounds for {name!r} must be a (low, high) pair or None")
        i, lo, hi = names.index(name), pair[0], pair[1]
        if name == "alpha":
            if lo is not None and hi is not None and float(lo) <= -np.pi and float(hi) >= np.pi:
                continue
            lo = None if lo is None else _alpha_to_centered(float(lo), tau0, f0)
            hi = None if hi is None else _alpha_to_centered(float(hi), tau0, f0)
            if lo is not None and hi is not None and lo > hi:
                lo, hi = -np.pi, np.pi
        lower[i] = lower[i] if lo is None else float(lo)
        upper[i] = upper[i] if hi is None else float(hi)
    return np.array(lower, float), np.array(upper, float)


def nonlinear_detuning_hz(fr, Qi, Qc, anl, phi=0.0):
    """Return the Duffing nonlinear detuning converted to Hz.

    ``fr``/``Qi``/``Qc``/``phi`` are the resonator parameters (see
    :func:`s21_model`) and ``anl`` the dimensionless Duffing parameter; the
    result is ``fr * Qr_inv * anl``.
    """
    return float(fr * (1.0 / Qi + np.real(1.0 / complex_coupling_q(Qc, phi))) * anl)


def _circle_fit(z):
    """Fit a simple algebraic circle for raw-IQ diagnostic fields."""
    z = np.asarray(z, complex)
    if z.size < 3:
        return complex(np.nan, np.nan), np.nan
    x, y = z.real, z.imag
    A = np.c_[2.0 * x, 2.0 * y, np.ones_like(x)]
    b = x * x + y * y
    try:
        cx, cy, c = np.linalg.lstsq(A, b, rcond=None)[0]
        r = np.sqrt(max(c + cx * cx + cy * cy, 0.0))
        return complex(cx, cy), float(r)
    except np.linalg.LinAlgError:
        return complex(np.nan, np.nan), np.nan


def _dip_weighted_subsample(f, z, target_frac=0.5, depth_frac=0.1):
    """Return indices that mandatorily keep every point inside the magnitude
    dip and sample the rest of the sweep with density proportional to
    ``|d(phase)/df|``.

    The dip is defined by a baseline-relative depth threshold: samples with
    ``(1 - |z| / baseline) > depth_frac * max_depth`` are kept in full.
    ``depth_frac=0.1`` reaches well into the wings, so the long smooth
    shoulder of a Duffing-bistable resonator is preserved without needing
    an explicit asymmetry pad. A symmetric half-max threshold cuts the
    shoulder short and drops the optimiser into a wrong-anl basin on
    bistable data.

    Information about the resonance lives where the phase rotates: across the
    dip (kept fully) and over any nearby features such as neighbouring
    resonators (kept densely by gradient weight). Far baseline rotates slowly
    so points there are decimated heavily. ``target_frac`` is the approximate
    fraction of the original sample count to retain; the first / last samples
    are always kept to anchor the baseline.
    """
    n = len(f)
    if n < 20:
        return np.arange(n)

    mag = np.abs(z)
    edge = max(5, n // 20)
    baseline = 0.5 * (np.median(mag[:edge]) + np.median(mag[-edge:]))
    if baseline <= 0:
        return np.arange(n)
    depth = 1.0 - mag / baseline
    max_depth = float(np.max(depth))
    if max_depth <= 0.0:
        return np.arange(n)
    in_dip = depth > depth_frac * max_depth

    phase = np.unwrap(np.angle(z))
    grad = np.abs(np.gradient(phase, f))
    # Dilate the gradient so a sharp cliff (often a 1-2 sample spike)
    # spreads its weight to its neighbours, giving balanced sampling on
    # both sides of the cliff rather than just at the peak.
    k = max(2, min(7, len(f) // 50))
    pad = np.pad(grad, k, mode="edge")
    grad = np.stack([pad[i:i + len(f)] for i in range(2 * k + 1)]).max(axis=0)

    outside_idx = np.where(~in_dip)[0]
    n_target = max(int(n * target_frac), int(in_dip.sum()) + 2)
    n_keep_outside = n_target - int(in_dip.sum())

    if n_keep_outside <= 0 or outside_idx.size == 0:
        keep_idx = np.where(in_dip)[0]
    elif n_keep_outside >= outside_idx.size:
        return np.arange(n)
    else:
        # Quantile-inverse sampling on the gradient gives density proportional
        # to |d(phase)/df| outside the dip. A 10% baseline floor keeps the
        # cumulative weight from collapsing to a step function on heavily
        # skewed gradients (e.g., a sharp cliff), so the quantile picks
        # actually spread across the rest of the sweep instead of clumping.
        w = grad[outside_idx] + 0.1 * (grad.max() + 1e-12)
        cum = np.cumsum(w)
        cum = cum / cum[-1]
        targets = (np.arange(n_keep_outside) + 0.5) / n_keep_outside
        picks = np.unique(np.searchsorted(cum, targets).clip(0, outside_idx.size - 1))
        keep_idx = np.concatenate([np.where(in_dip)[0], outside_idx[picks]])

    return np.unique(np.concatenate([[0], keep_idx, [n - 1]]))


def _asymmetry_observables(f, z):
    """Measure the cliff/shoulder geometry used by high-anl seeding.

    Returns a dict of dip widths and seed values, or ``None`` when the
    magnitude dip is too weak, symmetric, or clipped by the sweep window.
    """
    f = np.asarray(f, float)
    mag = np.abs(np.asarray(z))
    n = len(f)
    if n < 11:
        return None
    edge_n = max(5, n // 20)
    baseline = 0.5 * (np.median(mag[:edge_n]) + np.median(mag[-edge_n:]))
    if baseline <= 0:
        return None
    depth = 1.0 - mag / baseline
    i_dip = int(np.argmax(depth))
    if depth[i_dip] <= 0.02 or i_dip < 2 or i_dip > n - 3:
        return None

    half = 0.5 * depth[i_dip]

    def cross(direction):
        idxs = range(i_dip - 1, -1, -1) if direction == "left" else range(i_dip + 1, n)
        prev = i_dip
        for k in idxs:
            if depth[k] < half:
                t = (depth[prev] - half) / max(depth[prev] - depth[k], 1e-12)
                return f[prev] + (f[k] - f[prev]) * t
            prev = k
        return None

    f_lo = cross("left")
    f_hi = cross("right")
    if f_lo is None or f_hi is None:
        return None
    w_lo = f[i_dip] - f_lo
    w_hi = f_hi - f[i_dip]
    if w_lo <= 0 or w_hi <= 0:
        return None

    asym = (w_lo - w_hi) / (w_lo + w_hi)
    if abs(asym) < 0.15:
        return None

    if w_hi > w_lo:
        sign, w_smooth = 1.0, w_hi
    else:
        sign, w_smooth = -1.0, w_lo

    fr_seed = float(f[i_dip] + sign * 1.35 * w_smooth)
    w_cliff = min(w_lo, w_hi)
    return {
        "f_dip": float(f[i_dip]),
        "f_lo": float(f_lo),
        "f_hi": float(f_hi),
        "max_depth": float(depth[i_dip]),
        "w_lo": float(w_lo),
        "w_hi": float(w_hi),
        "w_smooth": float(w_smooth),
        "w_cliff": float(w_cliff),
        "width_ratio": float(w_smooth / max(w_cliff, 1e-30)),
        "asymmetry": float(asym),
        "smooth_sign": float(sign),
        "fr_seed": fr_seed,
        "anl_seed": 0.5,
    }


def _is_high_anl_shape(asymmetry_observables):
    """Return True when the dip shape is a strong high-Duffing indicator."""
    if asymmetry_observables is None:
        return False
    return (
        abs(asymmetry_observables["asymmetry"]) >= _HIGH_ANL_ASYMMETRY
        and asymmetry_observables["width_ratio"] >= _HIGH_ANL_WIDTH_RATIO
    )


def _autodetect_high_anl_probe_needed(asymmetry_observables, opt, f, z, f0,
                                      sweep_direction):
    """Decide whether the automatic high-anl multiseed probe is worth its cost.

    Strong cliff/shoulder asymmetry triggers immediately. Moderate asymmetry
    only triggers when the first nonlinear fit still leaves residuals that are
    large compared with the observed dip depth.
    """
    if asymmetry_observables is None:
        return False
    if _is_high_anl_shape(asymmetry_observables):
        return True
    if (
        abs(asymmetry_observables["asymmetry"]) < _MODERATE_ANL_ASYMMETRY
        or asymmetry_observables["width_ratio"] < _MODERATE_ANL_WIDTH_RATIO
        or asymmetry_observables["max_depth"] < 0.05
    ):
        return False
    try:
        model = s21_model_centered_delay(
            f, *opt.x, f0=f0, sweep_direction=sweep_direction)
        residual = float(np.sqrt(np.mean(np.abs(model - z) ** 2)))
    except Exception:
        return False
    mag = np.abs(z)
    feature_scale = max(float(np.max(mag) - np.min(mag)), 1e-30)
    return residual > _AUTODETECT_HIGH_ANL_BAD_FIT_FRACTION * feature_scale


def _linear_fit_is_sufficient(f, z, z_error, linear_params, sweep_direction,
                              empirical, asymmetry_observables,
                              dip_excess_max=_AUTO_DIP_EXCESS_MAX,
                              asymmetry_max=_AUTO_ASYMMETRY_MAX):
    """Decide whether the linear model already explains a sweep (``anl`` unneeded).

    This backs ``nonlinear='auto'``: it inspects the *linear* fit and answers
    "would the Duffing refinement actually change anything?" so the expensive
    nonlinear path can be skipped when it cannot help. It is a
    model-adequacy test on the linear residual, designed to stay robust on real
    data where broadband systematics (cable ripple, neighbouring tones, an
    imperfectly scaled error bar) make a plain reduced-chi-square unusable -- on
    a real power sweep the linear fits sat at reduced-chi-square ~20 even when
    the resonator was perfectly linear.

    Two cheap signals are combined:

    1. ``dip_excess`` -- how badly the linear model misfits the resonance *core*
       relative to the off-resonance *baseline*. A Duffing distortion lives in
       the dip, so it inflates the dip residual but not the baseline; taking the
       ratio cancels any global mis-scaling of the noise and any broadband
       systematics shared by both regions, which is what makes the test
       transferable between datasets. The dip and baseline are located from the
       *empirical* (model-free) dip centre and linewidth, because a strong cliff
       biases the linear-fit ``fr``/``Ql`` enough to mis-place a fit-based
       window.
    2. ``asymmetry`` -- the cliff/shoulder magnitude asymmetry from
       :func:`_asymmetry_observables`. It is very specific to the Duffing shape
       but blind to weak nonlinearity, so it only ever *adds* a reason to run
       the nonlinear fit.

    The linear model is declared sufficient only when the dip residual looks
    like noise (``dip_excess < dip_excess_max``) *and* the dip is not visibly
    skewed (``asymmetry < asymmetry_max``). The decision deliberately errs
    towards running the nonlinear fit: a false "insufficient" only wastes time,
    while a false "sufficient" would return biased ``fr``/``Q``.

    Parameters
    ----------
    f, z : ndarray
        Full (un-subsampled) sweep frequencies (Hz) and complex S21.
    z_error : tuple of ndarray or None
        Prepared ``(sigma_I, sigma_Q)`` measurement uncertainties, or ``None``
        for unweighted data (then the noise is estimated from the baseline
        residual scatter).
    linear_params : sequence of float
        The seven public linear parameters ``(fr, Qi, Qc, phi, a, alpha, tau)``
        from the linear fit, i.e. ``opt_linear.x_public``.
    sweep_direction : {'up', 'down'}
        Forwarded to the model evaluation (immaterial at ``anl = 0`` but kept
        for consistency with the rest of the fitter).
    empirical : object
        The :func:`estimate_resonance_empirical` result, supplying the
        model-free dip centre (``fr``) and ``linewidth_hz`` used for masking.
    asymmetry_observables : dict or None
        Output of :func:`_asymmetry_observables` for the same sweep (``None``
        when the dip is too symmetric to measure, which counts as "not skewed").
    dip_excess_max, asymmetry_max : float, optional
        Decision thresholds (defaults :data:`_AUTO_DIP_EXCESS_MAX` and
        :data:`_AUTO_ASYMMETRY_MAX`).

    Returns
    -------
    sufficient : bool
        ``True`` when the linear model is adequate and the nonlinear
        refinement can be skipped.
    diagnostics : dict
        ``dip_excess``, ``asymmetry``, ``dip_chi``, ``base_chi``, the dip /
        baseline sample counts and the boolean decision, for logging and for
        attaching to the returned :class:`FitResult` as ``linear_sufficiency``.
    """
    # Residual magnitude of the linear model across the whole sweep.
    z_fit = s21_model(f, *linear_params, sweep_direction=sweep_direction)
    residual = np.abs(z_fit - z)

    # Model-free dip centre and width. estimate_resonance_empirical follows the
    # actual magnitude minimum, which stays put even when a Duffing cliff has
    # pulled the linear-fit fr/Ql off the true centre and would otherwise place
    # the dip window in the wrong spot.
    dip_center = (float(empirical.fr) if np.isfinite(empirical.fr)
                  else float(f[np.argmin(np.abs(z))]))
    linewidth = float(empirical.linewidth_hz)
    if not np.isfinite(linewidth) or linewidth <= 0.0:
        linewidth = (float(f[-1]) - float(f[0])) / 10.0
    offset = np.abs(f - dip_center)

    # Core of the resonance vs the off-resonance baseline.
    in_dip = offset <= 1.5 * linewidth
    in_base = offset >= 3.0 * linewidth
    # If the window is too narrow to populate both bands (e.g. a very tight
    # targeted sweep), fall back to the closest sixth of the points as the dip
    # and the farthest third as the baseline so the test still returns a value.
    if int(in_dip.sum()) < 4 or int(in_base.sum()) < 8:
        order = np.argsort(offset)
        in_dip = np.zeros(len(f), bool)
        in_dip[order[:max(6, len(f) // 6)]] = True
        in_base = np.zeros(len(f), bool)
        in_base[order[-max(8, len(f) // 3):]] = True

    # Per-point noise sigma. Prefer the measured uncertainty (quadrature sum of
    # the I and Q errors); otherwise estimate it from the scatter of the
    # baseline residual so the test still works on unweighted data.
    if z_error is not None:
        sigma = np.sqrt(z_error[0] ** 2 + z_error[1] ** 2)
    else:
        sigma = np.full(len(f), max(float(np.std(residual[in_base])), 1e-30))

    # RMS residual in units of noise, separately over dip and baseline.
    dip_chi = float(np.sqrt(np.mean((residual[in_dip] / sigma[in_dip]) ** 2)))
    base_chi = float(np.sqrt(np.mean((residual[in_base] / sigma[in_base]) ** 2)))
    # dip_excess ~ 1: the resonance core is fit as well as the baseline, so the
    # linear model is enough. dip_excess >> 1: unmodelled structure concentrated
    # in the dip -- the signature of an unmodelled Duffing term -- so refine.
    dip_excess = dip_chi / max(base_chi, 1e-9)

    asymmetry = (float(abs(asymmetry_observables["asymmetry"]))
                 if asymmetry_observables else 0.0)

    sufficient = (dip_excess < dip_excess_max) and (asymmetry < asymmetry_max)
    diagnostics = {
        "dip_excess": dip_excess,
        "asymmetry": asymmetry,
        "dip_chi": dip_chi,
        "base_chi": base_chi,
        "n_dip": int(in_dip.sum()),
        "n_base": int(in_base.sum()),
        "sufficient": bool(sufficient),
    }
    return bool(sufficient), diagnostics


def _high_anl_probe_grid(try_even_harder):
    """Return ``(anl_seeds, Qi_seeds, fr_perturb_kHz)`` for high-anl probing."""
    if try_even_harder:
        return (
            _HIGH_ANL_EXHAUSTIVE_ANL_SEEDS,
            _HIGH_ANL_EXHAUSTIVE_QI_SEEDS,
            _HIGH_ANL_EXHAUSTIVE_FR_PERTURB_KHZ,
        )
    return (
        _HIGH_ANL_PROBE_ANL_SEEDS,
        _HIGH_ANL_PROBE_QI_SEEDS,
        _HIGH_ANL_PROBE_FR_PERTURB_KHZ,
    )


def _attach_uncertainties(opt):
    """Compute the parameter covariance and uncertainties for one fit result.

    Split out of :func:`_run_optimizer` so the expensive SVD can be deferred to
    the single winning candidate of a nonlinear probe grid instead of being
    paid on every discarded candidate. Reads the inputs stashed on ``opt`` by
    :func:`_run_optimizer` (``opt._cov_inputs``) and the centred Jacobian
    ``opt.jac``; attaches ``parameter_covariance`` and
    ``parameter_uncertainties`` to ``opt`` and returns it.
    """
    optimizer_weight_vec, stat_weight_vec, reduced, fixed_mask, f0 = opt._cov_inputs
    names = opt.parameter_names
    param_fixed = opt.fixed_parameters
    try:
        model_jac = (
            opt.jac if optimizer_weight_vec is None
            else opt.jac * optimizer_weight_vec[:, None]
        )
        stat_jac = model_jac if stat_weight_vec is None else model_jac / stat_weight_vec[:, None]
        _, s, vt = np.linalg.svd(stat_jac, full_matrices=False)
        keep = s > np.finfo(float).eps * max(stat_jac.shape) * s[0]
        covariance = (vt[keep].T / s[keep]**2) @ vt[keep]
        if stat_weight_vec is None:
            covariance *= reduced
        transform = np.eye(len(names))
        if "alpha" not in param_fixed:
            transform[names.index("alpha"), names.index("tau")] = 2.0 * np.pi * f0
        covariance = transform @ covariance @ transform.T
    except Exception:
        covariance = np.full((len(names), len(names)), np.nan)
    uncertainty = {}
    for i, name in enumerate(names):
        if fixed_mask[i]:
            uncertainty[name] = 0.0
            continue
        v = covariance[i, i]
        uncertainty[name] = float(np.sqrt(v)) if np.isfinite(v) and v >= 0.0 else np.nan
    fr, Qi, Qc, phi = opt.x_public[:4]
    Ql = loaded_q(Qi, Qc, phi)
    grad = np.zeros(len(names))
    grad[names.index("Qi")] = Ql**2 / Qi**2
    grad[names.index("Qc")] = Ql**2 * np.cos(phi) ** 2 / Qc**2
    grad[names.index("phi")] = Ql**2 * np.sin(2.0 * phi) / Qc
    vql = grad @ covariance @ grad
    uncertainty["Ql"] = float(np.sqrt(vql)) if np.isfinite(vql) and vql >= 0.0 else np.nan
    opt.parameter_covariance = covariance
    opt.parameter_uncertainties = uncertainty
    opt._cov_inputs = None  # release the weight vectors; no longer needed
    return opt


def _run_optimizer(f, z, z_error, optimizer_z_error, p0, nonlinear,
                   sweep_direction, f0, max_nfev, tol, param_bounds=None,
                   param_fixed=None, return_uncertainties=True,
                   error_weight_power=1.0):
    """Run one carefully scaled least-squares optimisation.

    For nonlinear fits ``anl`` is reparameterised internally as ``log(anl)``
    so the optimiser sees uniform per-decade sensitivity across the physical
    range (~1e-6 to 1e2). The reparameterisation is invisible from the
    outside: bounds, initial guesses, ``opt.x``, ``opt.x_public`` and the
    returned covariance are all reported in linear ``anl`` units.
    """
    names = NONLINEAR_NAMES if nonlinear else LINEAR_NAMES
    param_fixed = _parameter_dict(param_fixed) or {}
    unknown = set(param_fixed) - set(names)
    if unknown:
        raise ValueError(f"unknown param_fixed parameter(s): {sorted(unknown)}")
    fixed_mask = np.array([name in param_fixed for name in names], bool)
    p0 = _apply_initial_guess(np.asarray(p0, float).copy(), names, param_fixed, f0)
    # log(anl) is naturally O(1) over ~20 units of range, so a fixed unit
    # scale floor is appropriate. The finite-difference step is set to ~0.3%
    # in anl (3e-3 in log space): smaller values give a more precise gradient
    # on noiseless data but cause the optimiser to lock into shallow noise-
    # induced local minima on real lab data, where 3e-3 reliably escapes them.
    floors = np.r_[SCALE_FLOORS, 1.0] if nonlinear else SCALE_FLOORS
    diff_steps = np.r_[DIFF_STEPS, 3e-3] if nonlinear else DIFF_STEPS
    span = max(float(f[-1] - f[0]), float(np.median(np.diff(f))), 1.0)
    eps, a0 = 1e-6, max(abs(float(p0[4])), 1e-12)

    # Broad physical defaults; param_bounds are still named parameter bounds.
    lower = [f[0] - span, 10.0, 10.0, -np.pi / 2.0 + eps,
             a0 * 0.01, -np.pi, p0[6] - 1e-6]
    upper = [f[-1] + span, 1e8, 1e8, np.pi / 2.0 - eps,
             a0 * 100.0, np.pi, p0[6] + 1e-6]
    if nonlinear:
        lower.append(0.0)
        upper.append(100.0)
    lower, upper = _apply_bounds(lower, upper, names, param_bounds, f0, p0[names.index("tau")])
    p0 = np.minimum(np.maximum(np.asarray(p0, float), lower), upper)
    p0 = _apply_initial_guess(p0, names, param_fixed, f0)
    if nonlinear and not fixed_mask[-1] and p0[-1] <= lower[-1] and upper[-1] > lower[-1]:
        p0[-1] = min(max(lower[-1] + 1e-5, 1e-5), upper[-1])

    # Public-units copies preserved before any internal reparameterisation.
    p0_physical = p0.copy()
    lower_physical = np.asarray(lower, float).copy()
    upper_physical = np.asarray(upper, float).copy()

    # Swap anl -> log(anl) in the vector the optimiser actually sees.
    if nonlinear:
        anl_min = max(float(lower_physical[-1]), _ANL_MIN)
        anl_max = max(float(upper_physical[-1]), anl_min * 10.0)
        lower = lower_physical.copy()
        upper = upper_physical.copy()
        p0 = p0_physical.copy()
        lower[-1] = np.log(anl_min)
        upper[-1] = np.log(anl_max)
        p0[-1] = np.log(max(float(p0_physical[-1]), anl_min))
    else:
        lower = lower_physical
        upper = upper_physical

    def physical_from_internal(p_internal):
        p = np.asarray(p_internal, float).copy()
        if nonlinear:
            p[-1] = np.exp(p[-1])
        for name, value in param_fixed.items():
            if name != "alpha":
                p[names.index(name)] = float(value)
        if "alpha" in param_fixed:
            p[names.index("alpha")] = _alpha_to_centered(
                float(param_fixed["alpha"]), p[names.index("tau")], f0)
        return p

    free = ~fixed_mask
    scales = np.maximum(np.abs(p0), floors)
    optimizer_weight_vec = _error_vector(optimizer_z_error, error_weight_power)

    def residual(p_scaled):
        p_internal = p0.copy()
        p_internal[free] = p_scaled * scales[free]
        p = physical_from_internal(p_internal)
        d = s21_model_centered_delay(f, *p, f0=f0, sweep_direction=sweep_direction) - z
        r = np.r_[d.real, d.imag]
        return r if optimizer_weight_vec is None else r / optimizer_weight_vec

    # Closed-form Jacobian for the linear model, mapped into the optimiser's
    # scaled/free/weighted coordinates so it can be handed straight to
    # least_squares. Only used when no parameter is held fixed via the
    # alpha<->tau coupling that physical_from_internal would introduce; the
    # nonlinear (log-anl, Duffing) model keeps finite differences.
    use_analytic_jac = (
        _USE_ANALYTIC_LINEAR_JAC and not nonlinear
        and "alpha" not in param_fixed
    )

    def jacobian(p_scaled):
        p_internal = p0.copy()
        p_internal[free] = p_scaled * scales[free]
        p = physical_from_internal(p_internal)
        dS = _s21_linear_jacobian(f, *p, f0=f0)        # (npoints, 7), per-param
        jac_full = np.vstack([dS.real, dS.imag])       # stack like residual()
        jac_free = jac_full[:, free] * scales[free]     # unscale + drop fixed cols
        if optimizer_weight_vec is not None:
            jac_free = jac_free / optimizer_weight_vec[:, None]
        return jac_free

    start = time.time()
    if np.any(free):
        least_squares_kwargs = dict(
            bounds=(lower[free] / scales[free], upper[free] / scales[free]),
            max_nfev=max_nfev, ftol=tol, xtol=tol, gtol=tol,
        )
        if use_analytic_jac:
            least_squares_kwargs["jac"] = jacobian
        else:
            least_squares_kwargs["diff_step"] = diff_steps[free]
        opt = least_squares(
            residual, p0[free] / scales[free], **least_squares_kwargs)
    else:
        r = residual(np.array([], float))
        opt = OptimizeResult(
            x=np.array([], float), fun=r, cost=0.5 * float(np.sum(r * r)),
            jac=np.zeros((r.size, 0)), nfev=1, njev=0, optimality=0.0,
            active_mask=np.array([], int), status=1,
            message="All parameters fixed.", success=True,
        )
    opt.duration_s = time.time() - start
    opt.x_scaled = np.zeros(len(names))
    opt.x_scaled[free] = opt.x.copy()
    opt.scales = scales
    opt.jac_scaled = np.zeros((opt.jac.shape[0], len(names)))
    opt.jac_scaled[:, free] = opt.jac.copy()
    x_internal = p0.copy()
    x_internal[free] = opt.x * scales[free]
    opt.x_centered = physical_from_internal(x_internal)
    opt.jac_centered = np.zeros((opt.jac.shape[0], len(names)))
    opt.jac_centered[:, free] = opt.jac / scales[free]

    # Map log(anl) back to linear anl in the public vector. Chain rule on the
    # Jacobian column: dr/d(anl) = (dr/d log anl) / anl.
    if nonlinear:
        anl_fitted = float(opt.x_centered[-1])
        if anl_fitted > 0.0:
            opt.jac_centered[:, -1] = opt.jac_centered[:, -1] / anl_fitted

    opt.x = opt.x_centered.copy()
    opt.jac = opt.jac_centered.copy()
    opt.x_public = opt.x.copy()
    opt.x_public[names.index("alpha")] = _alpha_to_public(
        opt.x[names.index("alpha")], opt.x[names.index("tau")], f0)
    opt.initial_guess_physical = p0_physical
    opt.initial_guess_public = p0_physical.copy()
    opt.initial_guess_public[names.index("alpha")] = _alpha_to_public(
        p0_physical[names.index("alpha")],
        p0_physical[names.index("tau")],
        f0,
    )
    opt.bounds_physical = (lower_physical, upper_physical)
    opt.parameter_names = names
    opt.fixed_parameters = param_fixed

    p = opt.x
    model = s21_model_centered_delay(f, *p, f0=f0, sweep_direction=sweep_direction)
    diff = model - z
    model_residual = np.r_[diff.real, diff.imag]
    dof = model_residual.size - int(np.count_nonzero(free))
    stat_weight_vec = _error_vector(z_error, 1.0)
    if stat_weight_vec is None:
        stat_residual = model_residual
        reduced = float(np.sum(stat_residual**2) / dof) if dof > 0 else np.nan
        opt.reduced_chi2 = np.nan
    else:
        stat_residual = model_residual / stat_weight_vec
        reduced = float(np.sum(stat_residual**2) / dof) if dof > 0 else np.nan
        opt.reduced_chi2 = reduced
    opt.stat_residual = stat_residual
    # Stash the inputs the covariance needs so it can be (re)computed later on
    # the winning nonlinear candidate; see _attach_uncertainties.
    opt._cov_inputs = (optimizer_weight_vec, stat_weight_vec, reduced,
                       fixed_mask, f0)

    if not return_uncertainties:
        opt.parameter_covariance = None
        opt.parameter_uncertainties = None
        return opt
    return _attach_uncertainties(opt)


def _make_noise_result(f, z, z_error, optimizer_z_error, sweep_direction, f0,
                       fit_start, empirical, min_dip_depth_db,
                       nonlinear=False):
    """Build a failed FitResult for a trace classified as unresolved noise."""
    if empirical is None:
        empirical = estimate_resonance_empirical(f, s21=z)
    raw_center, raw_radius = _circle_fit(z)
    reason = (
        f"empirical dip depth {empirical.dip_depth_db:.3g} dB < "
        f"{float(min_dip_depth_db):.3g} dB"
    )
    parameter_names = NONLINEAR_NAMES if nonlinear else LINEAR_NAMES
    return FitResult(
        fr=float(empirical.fr),
        Ql=float(empirical.Ql),
        Qi=float(empirical.Qi),
        Qc=float(empirical.Qc),
        empirical_fr=float(empirical.fr),
        empirical_linewidth_hz=float(empirical.linewidth_hz),
        empirical_Ql=float(empirical.Ql),
        empirical_Qc=float(empirical.Qc),
        empirical_Qi=float(empirical.Qi),
        empirical_dip_depth_db=float(empirical.dip_depth_db),
        empirical_skew=float(empirical.skew),
        iq_center=raw_center,
        iq_radius=raw_radius,
        success=False,
        message=f"noise-only sweep: {reason}",
        noise_only=True,
        noise_reason=reason,
        anl=np.nan,
        sweep_direction=sweep_direction,
        fit_duration_s=float(time.time() - fit_start),
        f_reference=float(f0),
        parameter_names=parameter_names,
        f_data=f,
        z_data=z,
        z_err_data=_pack_z_error(z_error),
        optimizer_z_err_data=_pack_z_error(optimizer_z_error),
        z_fit=None,
    )


def _make_insufficient_data_result(f, z, sweep_direction, fit_start, nonlinear):
    """Build a failed, ``noise_only`` FitResult for a trace with too few finite
    samples to attempt a fit (e.g. a skipped or NaN-filled power-sweep step).

    Mirrors :func:`_make_noise_result`'s ``success=False`` / ``noise_only=True``
    contract so batch and power-sweep tools exclude it exactly like a
    below-threshold dip, but carries no empirical estimate (there is not enough
    data to compute one)."""
    f_arr = np.asarray(f, float).ravel()
    z_arr = np.asarray(z, complex).ravel()
    n_finite = int(np.count_nonzero(
        np.isfinite(f_arr) & np.isfinite(z_arr.real) & np.isfinite(z_arr.imag)
    ))
    reason = f"insufficient finite samples ({n_finite} < 5)"
    parameter_names = NONLINEAR_NAMES if nonlinear else LINEAR_NAMES
    return FitResult(
        success=False,
        message=f"noise-only sweep: {reason}",
        noise_only=True,
        noise_reason=reason,
        anl=np.nan,
        sweep_direction=sweep_direction,
        fit_duration_s=float(time.time() - fit_start),
        parameter_names=parameter_names,
        f_data=f_arr,
        z_data=z_arr,
    )


def _make_result(f, z, z_error, optimizer_z_error, opt, sweep_direction, f0,
                 fit_start, empirical=None, linear_fit=None, opt_linear=None,
                 opt_nonlinear=None, store_optimizer=True):
    """Build a FitResult from optimiser output.

    ``store_optimizer=False`` keeps every fitted value, derived quantity and
    diagnostic array but drops the bulky optimiser objects (``opt``,
    ``opt_linear``, ``opt_nonlinear`` and the nested ``linear_fit_result``,
    each of which carries full Jacobian matrices). Those are debug-only fields
    -- ``power_sweep`` already strips exactly these before persisting -- and
    dropping them sharply cuts the bytes pickled back from worker processes
    when fitting thousands of resonances with ``n_jobs > 1``.
    """
    if empirical is None:
        empirical = estimate_resonance_empirical(f, s21=z)
    p = opt.x_public.copy()
    names = tuple(opt.parameter_names)
    fr, Qi, Qc, phi, a, alpha, tau = p[:7]
    anl = float(p[7]) if len(p) == 8 else 0.0
    Qe = complex_coupling_q(Qc, phi)
    Ql = float(loaded_q(Qi, Qc, phi))
    z_fit = s21_model(f, *p, sweep_direction=sweep_direction)
    z_deembed = model_deembed(f, z, a, alpha, tau)
    z_fit_deembed = model_deembed(f, z_fit, a, alpha, tau)
    phase_center_cal = model_phase_center_geometry(Ql, Qe)
    center = phase_center_cal.center
    radius = phase_center_cal.radius
    phase_rotation = phase_center_cal.rotation_angle
    z_pc = phase_center_cal.apply(z_deembed)
    z_fit_pc = phase_center_cal.apply(z_fit_deembed)
    z_res = s21_model(np.array([fr]), fr, Qi, Qc, phi, 1.0, 0.0, 0.0, anl, sweep_direction)[0]
    deembed_rotation = float(wrap_phase(np.pi - np.angle(z_res - 1.0)))
    z_rot = rotate_around_point(z_deembed, deembed_rotation, point=1.0)
    z_fit_rot = rotate_around_point(
        z_fit_deembed, deembed_rotation, point=1.0)
    raw_center, raw_radius = _circle_fit(z)
    diff = z_fit - z
    linear_nfev = (
        int(getattr(opt_linear, "nfev", 0))
        if opt_linear is not None else int(getattr(opt, "nfev", 0))
    )
    nonlinear_nfev = (
        int(getattr(
            opt_nonlinear, "total_nonlinear_nfev",
            getattr(opt_nonlinear, "nfev", 0),
        ))
        if opt_nonlinear is not None else 0
    )
    linear_duration = (
        float(getattr(opt_linear, "duration_s", np.nan))
        if opt_linear is not None
        else float(getattr(opt, "duration_s", np.nan))
    )
    nonlinear_duration = (
        float(getattr(
            opt_nonlinear, "total_nonlinear_duration_s",
            getattr(opt_nonlinear, "duration_s", np.nan),
        ))
        if opt_nonlinear is not None else np.nan
    )
    detuning = nonlinear_detuning_hz(fr, Qi, Qc, anl, phi)
    # Heavy optimiser objects are debug-only; drop them in lean mode after the
    # scalar nfev/cost/duration accounting above has already read what it needs.
    kept_opt = opt if store_optimizer else None
    kept_opt_linear = opt_linear if store_optimizer else None
    kept_opt_nonlinear = opt_nonlinear if store_optimizer else None
    kept_linear_fit = linear_fit if store_optimizer else None
    return FitResult(
        fr=float(fr), Ql=Ql, Qi=float(Qi), Qc=float(Qc), phi=float(phi),
        a=float(a), alpha=float(alpha), tau=float(tau), Qe=Qe,
        Qc_abs=float(abs(Qe)),
        empirical_fr=float(empirical.fr),
        empirical_linewidth_hz=float(empirical.linewidth_hz),
        empirical_Ql=float(empirical.Ql),
        empirical_Qc=float(empirical.Qc),
        empirical_Qi=float(empirical.Qi),
        empirical_dip_depth_db=float(empirical.dip_depth_db),
        empirical_skew=float(empirical.skew),
        iq_center=raw_center, iq_radius=raw_radius,
        iq_center_deembed=center, iq_radius_deembed=radius,
        residual_rms=float(np.sqrt(np.mean(np.abs(diff) ** 2))),
        weighted_rms=_weighted_rms(diff, z_error),
        optimizer_weighted_rms=_weighted_rms(diff, optimizer_z_error),
        reduced_chi2=float(getattr(opt, "reduced_chi2", np.nan)),
        success=bool(getattr(opt, "success", False)),
        message=str(getattr(opt, "message", "")),
        anl=anl, sweep_direction=sweep_direction,
        nonlinear_detuning_hz=detuning,
        fr_minus_nonlinear_detuning=float(fr - detuning),
        nfev=(
            linear_nfev + nonlinear_nfev
            if opt_nonlinear is not None else int(getattr(opt, "nfev", 0))
        ),
        linear_nfev=linear_nfev,
        nonlinear_nfev=nonlinear_nfev,
        fit_duration_s=float(time.time() - fit_start),
        linear_fit_duration_s=linear_duration,
        nonlinear_fit_duration_s=nonlinear_duration,
        optimizer_cost=float(getattr(opt, "cost", np.nan)),
        linear_cost=(
            float(getattr(opt_linear, "cost", np.nan))
            if opt_linear is not None else float(getattr(opt, "cost", np.nan))
        ),
        nonlinear_cost=(
            float(getattr(opt_nonlinear, "cost", np.nan))
            if opt_nonlinear is not None else np.nan
        ),
        f_reference=float(f0), parameter_names=names,
        parameter_covariance=getattr(opt, "parameter_covariance", None),
        parameter_uncertainties=getattr(opt, "parameter_uncertainties", None),
        p=p, initial_guess=getattr(opt, "initial_guess_public", None),
        opt=kept_opt, opt_linear=kept_opt_linear,
        opt_nonlinear=kept_opt_nonlinear,
        optimizer_result=kept_opt, optimizer_result_linear=kept_opt_linear,
        optimizer_result_nonlinear=kept_opt_nonlinear,
        linear_fit_result=kept_linear_fit,
        f_data=f, z_data=z, z_err_data=_pack_z_error(z_error),
        optimizer_z_err_data=_pack_z_error(optimizer_z_error),
        z_fit=z_fit, z_data_deembed=z_deembed, z_fit_deembed=z_fit_deembed,
        z_data_deembed_rotated=z_rot, z_fit_deembed_rotated=z_fit_rot,
        z_data_deembed_phase_centered=z_pc,
        z_fit_deembed_phase_centered=z_fit_pc,
        deembed_rotation_angle=deembed_rotation,
        phase_center_rotation_angle=phase_rotation,
    )


def fit_resonance(f, z, z_err=None, nonlinear=True, sweep_direction="up",
                  initial_guess=None, param_bounds=None, param_fixed=None,
                  max_nfev=1000, tol=1e-8, return_uncertainties=True,
                  optimizer_z_err=None, use_error_weights=None,
                  error_weight_power=1.0, fit_tolerance=None,
                  try_harder=False, try_even_harder=False,
                  subsample=False, autodetect_high_anl=True,
                  min_dip_depth_db=0.5, store_optimizer=True):
    """Fit one complex S21 sweep.

    ``subsample=True`` keeps every point near the dip and decimates the tails,
    cutting the fit cost roughly in half without losing fit quality. The
    optimiser runs on the subsampled points; the returned FitResult's
    f_data / z_data / z_fit arrays still match the original input length.

    ``initial_guess`` supplies starting values, ``param_bounds`` supplies
    named ``(low, high)`` box bounds, and ``param_fixed`` holds named
    parameters constant. Use the fitted parameter names listed in the module
    docstring.

    Most optional arguments are historically positional; new code should pass
    them by keyword. ``try_harder``, ``try_even_harder`` and
    ``autodetect_high_anl`` only affect nonlinear fits.

    If ``min_dip_depth_db`` is not ``None``, sweeps whose empirical dip depth
    is below the threshold return ``success=False`` and ``noise_only=True``
    instead of fitting a random fluctuation.

    Parameters
    ----------
    f : array-like
        Sweep frequencies (Hz).
    z : array-like of complex
        Complex S21 at each frequency.
    z_err : array-like of complex or None, optional
        Per-point S21 uncertainty; propagated into
        ``parameter_uncertainties`` and (optionally) the fit weights.
    nonlinear : bool or {'auto'}, optional
        How to handle the Duffing ``anl`` term (default ``True``). ``True``
        always fits ``anl`` (delegating to :func:`fit_resonance_nonlinear`),
        which is what you want whenever you need an ``anl`` estimate at every
        point -- e.g. fitting ``anl`` versus drive power, where low-power values
        matter even when small. ``False`` fits the linear model only. ``'auto'``
        runs the cheap linear fit first and only continues to the Duffing fit
        when the linear fit is judged insufficient (recording the decision on
        the result's ``linear_sufficiency`` field, and returning a linear
        ``FitResult`` with ``anl = 0`` when it keeps the linear fit); it is fast
        for bulk fitting but deliberately drops ``anl`` for resonators whose
        nonlinearity is below the single-sweep detection floor, so do not use it
        when you rely on the small-``anl`` values.
    max_nfev : int, optional
        Maximum optimiser function evaluations (default ``1000``).
    tol : float, optional
        Optimiser tolerance (default ``1e-8``); overridden by
        ``fit_tolerance`` when that is given.
    return_uncertainties : bool, optional
        Compute parameter uncertainties from the Jacobian (default ``True``).
    optimizer_z_err : array-like or bool or None, optional
        Per-point errors used to weight the optimiser residuals; ``True``
        reuses ``z_err``.  ``None`` (default) leaves the fit unweighted.
    use_error_weights : bool or None, optional
        Convenience switch: ``True`` sets ``optimizer_z_err=True``, ``False``
        disables weighting.  ``None`` (default) defers to ``optimizer_z_err``.
    error_weight_power : float, optional
        Exponent applied to the error weights (default ``1.0``).
    fit_tolerance : float or None, optional
        Alias for ``tol`` taking precedence when set (default ``None``).
    store_optimizer : bool, optional
        Keep the bulky optimiser objects (``opt``/``opt_linear``/
        ``opt_nonlinear``/``linear_fit_result``) on the result (default
        ``True``). Pass ``False`` when fitting many resonances for parameters
        only: every fitted value and diagnostic array is still returned, but
        the Jacobian-carrying optimiser objects are dropped, which avoids a
        redundant intermediate result build and sharply cuts the data pickled
        back from worker processes under ``n_jobs > 1``.
    """
    # ``nonlinear`` accepts True/False or the string 'auto'. 'auto' runs the
    # cheap linear fit first and only continues to the Duffing fit when that
    # linear fit is judged insufficient (see fit_resonance_nonlinear's
    # auto_select). The work all happens inside fit_resonance_nonlinear because
    # it already computes the linear seed there, so 'auto' reuses it for free.
    auto_select = isinstance(nonlinear, str) and nonlinear.lower() == "auto"
    if nonlinear is not True and not auto_select and nonlinear not in (False, None):
        raise ValueError("nonlinear must be True, False, or 'auto'")
    if nonlinear is True or auto_select:
        return fit_resonance_nonlinear(
            f, z, z_err=z_err, sweep_direction=sweep_direction,
            initial_guess=initial_guess, param_bounds=param_bounds,
            param_fixed=param_fixed, max_nfev=max_nfev, tol=tol,
            return_uncertainties=return_uncertainties,
            optimizer_z_err=optimizer_z_err, use_error_weights=use_error_weights,
            error_weight_power=error_weight_power,
            fit_tolerance=fit_tolerance,
            try_harder=try_harder, try_even_harder=try_even_harder,
            subsample=subsample, autodetect_high_anl=autodetect_high_anl,
            min_dip_depth_db=min_dip_depth_db, store_optimizer=store_optimizer,
            auto_select=auto_select,
        )
    fit_start = time.time()
    tol = fit_tolerance if fit_tolerance is not None else tol
    guess = {} if initial_guess is None else dict(_parameter_dict(initial_guess))
    if use_error_weights is not None:
        optimizer_z_err = True if use_error_weights else None
    try:
        f, z, z_error, opt_error = _prepare_arrays(f, z, z_err, optimizer_z_err)
    except InsufficientSamplesError:
        return _make_insufficient_data_result(
            f, z, sweep_direction, fit_start, nonlinear=False)
    empirical = estimate_resonance_empirical(f, s21=z)
    f0_full = float(np.mean(f))
    if (
        min_dip_depth_db is not None
        and np.isfinite(empirical.dip_depth_db)
        and empirical.dip_depth_db < float(min_dip_depth_db)
    ):
        return _make_noise_result(
            f, z, z_error, opt_error, sweep_direction, f0_full, fit_start,
            empirical, min_dip_depth_db, nonlinear=False,
        )
    f_full, z_full, z_error_full, opt_error_full = f, z, z_error, opt_error
    if subsample:
        keep = _dip_weighted_subsample(f, z)
        f, z = f[keep], z[keep]
        z_error = None if z_error is None else tuple(a[keep] for a in z_error)
        opt_error = None if opt_error is None else tuple(a[keep] for a in opt_error)
    f0 = float(np.mean(f))
    p0 = _apply_initial_guess(_guess_params(f, z, empirical), LINEAR_NAMES, guess, f0)
    opt = _run_optimizer(
        f, z, z_error, opt_error, p0, False, sweep_direction, f0, max_nfev, tol,
        param_bounds=_select_names(param_bounds, LINEAR_NAMES),
        param_fixed=param_fixed,
        return_uncertainties=return_uncertainties,
        error_weight_power=error_weight_power,
    )
    return _make_result(f_full, z_full, z_error_full, opt_error_full,
                        opt, sweep_direction, f0, fit_start,
                        empirical=empirical, store_optimizer=store_optimizer)


def fit_resonance_nonlinear(f, z, z_err=None, sweep_direction="up",
                            initial_guess=None, param_bounds=None,
                            param_fixed=None,
                            max_nfev=1000, tol=1e-8,
                            return_uncertainties=True, optimizer_z_err=None,
                            use_error_weights=None, error_weight_power=1.0,
                            fit_tolerance=None, try_harder=False,
                            try_even_harder=False,
                            subsample=False, autodetect_high_anl=True,
                            min_dip_depth_db=0.5, store_optimizer=True,
                            auto_select=False):
    """Fit one complex S21 sweep with a linear seed then one Duffing refinement.

    For heavily bistable resonators whose deepest basin has a narrow
    attractor, fallback paths probe a small grid of seeds and return the
    lowest-cost result. ``autodetect_high_anl=True`` runs the ``try_harder``
    grid automatically when the measured dip has a sharp cliff / smooth shoulder
    shape and the single-seed nonlinear refine appears to be in the wrong
    basin.

    - ``try_harder=True``: 3 x 3 grid over ``(anl_seed, Qi_seed)`` with the
      ``fr_seed`` derived analytically from the Duffing upper saddle-node
      relation ``f_dip - fr = fr * Qr_inv * anl ** 0.4`` (empirical fit to
      numerical Duffing sweeps, accurate to ~3% across anl in [1, 20]).
      ~9 extra fits, typically a few hundred ms.
    - ``try_even_harder=True``: 5 x 4 x 3 grid that additionally perturbs
      ``fr_seed`` around the analytical value. ~60 extra fits, ~1+ s.

    ``subsample=True`` keeps every point near the dip and decimates the tails,
    cutting per-fit cost roughly in half without losing fit quality. Compounds
    with the try_harder/try_even_harder budgets.

    ``initial_guess`` supplies starting values, ``param_bounds`` supplies
    named ``(low, high)`` box bounds, and ``param_fixed`` holds named
    parameters constant. The explicit fallback grids may override
    ``initial_guess`` while searching for a better basin, but ``param_fixed``
    remains hard. Autodetection skips the fallback grid when ``initial_guess``
    already contains ``Qi`` or ``anl``.

    For repeat measurements pass the previous ``FitResult`` as
    ``initial_guess`` instead; that path is faster and lands in the same
    basin as the original fit. The same ``min_dip_depth_db`` pre-check used by
    ``fit_resonance`` runs before the linear seed.

    ``auto_select=True`` (reached via ``fit_resonance(nonlinear='auto')``) runs
    the linear seed and then checks :func:`_linear_fit_is_sufficient`. If the
    linear model already explains the data, the linear result is returned
    immediately (``anl = 0``) with the decision diagnostics attached as
    ``linear_sufficiency``; otherwise the normal Duffing refinement proceeds and
    the same diagnostics are attached to the nonlinear result.

    The data and optimiser arguments (``f``, ``z``, ``z_err``,
    ``sweep_direction``, ``max_nfev``, ``tol``, ``return_uncertainties``,
    ``optimizer_z_err``, ``use_error_weights``, ``error_weight_power``,
    ``fit_tolerance``) mean the same as in :func:`fit_resonance`.
    """
    fit_start = time.time()
    tol = fit_tolerance if fit_tolerance is not None else tol
    guess = {} if initial_guess is None else dict(_parameter_dict(initial_guess))
    if use_error_weights is not None:
        optimizer_z_err = True if use_error_weights else None
    try:
        f, z, z_error, opt_error = _prepare_arrays(f, z, z_err, optimizer_z_err)
    except InsufficientSamplesError:
        return _make_insufficient_data_result(
            f, z, sweep_direction, fit_start, nonlinear=True)
    empirical = estimate_resonance_empirical(f, s21=z)
    f0 = float(np.mean(f))
    if (
        min_dip_depth_db is not None
        and np.isfinite(empirical.dip_depth_db)
        and empirical.dip_depth_db < float(min_dip_depth_db)
    ):
        return _make_noise_result(
            f, z, z_error, opt_error, sweep_direction, f0, fit_start,
            empirical, min_dip_depth_db, nonlinear=True,
        )
    f_full, z_full, z_error_full, opt_error_full = f, z, z_error, opt_error
    # Run seeder, _guess_params, and the linear pre-fit on the FULL data --
    # they're cheap and changing their inputs perturbs the seed enough to
    # miss knife-edge basins. Subsampling kicks in only for the expensive
    # Duffing refine and try_harder/try_even_harder grids below.
    seed_obs = _asymmetry_observables(f, z)
    seed = None if seed_obs is None else (seed_obs["fr_seed"], seed_obs["anl_seed"])
    p0_linear = _apply_initial_guess(_guess_params(f, z, empirical), LINEAR_NAMES,
                                     _select_names(guess, LINEAR_NAMES), f0)
    opt_linear = _run_optimizer(
        f, z, z_error, opt_error, p0_linear, False, sweep_direction, f0, max_nfev, tol,
        param_bounds=_select_names(param_bounds, LINEAR_NAMES),
        param_fixed=_select_names(param_fixed, LINEAR_NAMES),
        return_uncertainties=return_uncertainties,
        error_weight_power=error_weight_power,
    )
    # The linear seed's full result is only ever surfaced as
    # ``linear_fit_result``. Lean mode drops that field, so skip this redundant
    # second result build entirely; the final result still reads nfev/cost/
    # duration from ``opt_linear`` directly.
    linear_fit = _make_result(
        f, z, z_error, opt_error, opt_linear, sweep_direction, f0, fit_start,
        empirical=empirical,
    ) if store_optimizer else None

    # nonlinear='auto': stop here if the linear model already explains the data,
    # returning the linear result (anl = 0) instead of paying for the Duffing
    # refinement and any probe grid below. The decision runs on the full
    # (pre-subsample) sweep so it sees every point of the resonance; see
    # _linear_fit_is_sufficient for the dip_excess / asymmetry signals.
    sufficiency = None
    if auto_select:
        linear_sufficient, sufficiency = _linear_fit_is_sufficient(
            f, z, z_error, opt_linear.x_public, sweep_direction, empirical,
            seed_obs)
        if linear_sufficient:
            # Reuse the linear result built above, or build it now if lean mode
            # skipped it, then record why the nonlinear fit was not run.
            linear_result = linear_fit if linear_fit is not None else _make_result(
                f, z, z_error, opt_error, opt_linear, sweep_direction, f0,
                fit_start, empirical=empirical, store_optimizer=store_optimizer)
            linear_result.linear_sufficiency = sufficiency
            return linear_result

    if subsample:
        keep = _dip_weighted_subsample(f, z)
        f, z = f[keep], z[keep]
        z_error = None if z_error is None else tuple(a[keep] for a in z_error)
        opt_error = None if opt_error is None else tuple(a[keep] for a in opt_error)

    # Apply the (full-data) asymmetry seed to override the linear-pre-fit's
    # fr / anl basin. User overrides in `guess` win.
    nl_guess = dict(_select_names(guess, NONLINEAR_NAMES) or {})
    if seed is not None:
        fr_seed, anl_seed = seed
        nl_guess.setdefault("fr", fr_seed)
        nl_guess.setdefault("anl", anl_seed)

    def _refine(nl_guess_local):
        p0_local = np.r_[opt_linear.x, 1e-5]
        p0_local = _apply_initial_guess(p0_local, NONLINEAR_NAMES, nl_guess_local, f0)
        # Uncertainties are deferred: a probe grid runs up to ~60 refinements
        # but only one wins, so the SVD covariance is computed once afterwards
        # (see _attach_uncertainties below) instead of on every candidate.
        return _run_optimizer(
            f, z, z_error, opt_error, p0_local, True, sweep_direction, f0, max_nfev, tol,
            param_bounds=_select_names(param_bounds, NONLINEAR_NAMES),
            param_fixed=param_fixed,
            return_uncertainties=False,
            error_weight_power=error_weight_power,
        )

    opt = _refine(nl_guess)
    total_nonlinear_duration = float(getattr(opt, "duration_s", 0.0))
    total_nonlinear_nfev = int(getattr(opt, "nfev", 0))

    # Fallback paths for heavily bistable resonators whose deep basin has a
    # narrow attractor. Each candidate's fr_seed is derived from the Duffing
    # upper saddle-node relation (dip offset ~ fr * Qr_inv * anl^0.4) using
    # the linear-fit Qc/phi and a candidate Qi.
    user_nl = _select_names(guess, NONLINEAR_NAMES) or {}
    fixed_nl = _select_names(param_fixed, NONLINEAR_NAMES) or {}
    autodetect_probe = (
        autodetect_high_anl
        and not (try_harder or try_even_harder)
        and _autodetect_high_anl_probe_needed(seed_obs, opt, f, z, f0, sweep_direction)
        and not ({"Qi", "anl"} & set(user_nl))
        and not ({"Qi", "anl"} & set(fixed_nl))
    )
    if (try_harder or try_even_harder or autodetect_probe) and seed_obs is not None:
        f_dip = seed_obs["f_dip"]
        direction = seed_obs["smooth_sign"]
        fr_lin = float(opt_linear.x[LINEAR_NAMES.index("fr")])
        Qc_lin = float(opt_linear.x[LINEAR_NAMES.index("Qc")])
        phi_lin = float(opt_linear.x[LINEAR_NAMES.index("phi")])
        Qe_re_inv = float(np.real(1.0 / complex_coupling_q(Qc_lin, phi_lin)))

        anl_alts, qi_alts, fr_perturb_kHz = _high_anl_probe_grid(try_even_harder)

        def candidate_guess(**candidate):
            # Candidate seeds are just starts, not constraints. They override
            # initial_guess during explicit probes; param_fixed remains hard.
            alt = dict(user_nl)
            for name, value in candidate.items():
                if name not in fixed_nl:
                    alt[name] = value
            return alt

        best = opt
        for anl_alt in anl_alts:
            for qi_alt in qi_alts:
                Qr_inv = 1.0 / qi_alt + Qe_re_inv
                base_offset = direction * fr_lin * Qr_inv * anl_alt ** 0.4
                for df_kHz in fr_perturb_kHz:
                    alt_guess = candidate_guess(
                        fr=f_dip + base_offset + df_kHz * 1e3,
                        anl=anl_alt,
                        Qi=qi_alt,
                    )
                    cand = _refine(alt_guess)
                    total_nonlinear_duration += float(getattr(cand, "duration_s", 0.0))
                    total_nonlinear_nfev += int(getattr(cand, "nfev", 0))
                    if np.isfinite(cand.cost) and cand.cost < best.cost:
                        best = cand
        opt = best
    opt.total_nonlinear_duration_s = total_nonlinear_duration
    opt.total_nonlinear_nfev = total_nonlinear_nfev
    # Refinements deferred their uncertainties; compute them once on the winner.
    if return_uncertainties:
        _attach_uncertainties(opt)

    result = _make_result(f_full, z_full, z_error_full, opt_error_full,
                          opt, sweep_direction, f0, fit_start,
                          empirical=empirical,
                          linear_fit=linear_fit, opt_linear=opt_linear,
                          opt_nonlinear=opt, store_optimizer=store_optimizer)
    # Record why 'auto' decided to run the nonlinear fit (None for True/False).
    if sufficiency is not None:
        result.linear_sufficiency = sufficiency
    return result


def evaluate_fit(f, fit_result, deembed=False, phase_center=False):
    """Evaluate a FitResult at new frequencies, optionally in calibrated planes.

    Parameters
    ----------
    f : array-like
        Frequencies (Hz) at which to evaluate the fitted model.
    fit_result : FitResult
        A result from :func:`fit_resonance` / :func:`batch_fit`.
    deembed : bool, optional
        If ``True``, remove the fitted cable gain/delay envelope so the curve
        is referred to the resonator plane (default ``False``).
    phase_center : bool, optional
        If ``True``, rotate so the off-resonance point sits on the real axis
        (default ``False``).
    """
    p = np.asarray(fit_result.p if getattr(fit_result, "p", None) is not None else [
        fit_result.fr, fit_result.Qi, fit_result.Qc, fit_result.phi,
        fit_result.a, fit_result.alpha, fit_result.tau, fit_result.anl,
    ], float)
    z = s21_model(f, *p, sweep_direction=getattr(fit_result, "sweep_direction", "up"))
    if not deembed and not phase_center:
        return z
    f = np.asarray(f, float)
    z = model_deembed(
        f, z, fit_result.a, fit_result.alpha, fit_result.tau)
    if phase_center:
        phase_center_cal = model_phase_center_geometry(
            fit_result.Ql, fit_result.Qe)
        z = phase_center_cal.apply(z)
    return z


def _available_cpu_count():
    """Return the CPU count visible to this process."""
    try:
        return len(os.sched_getaffinity(0))
    except (AttributeError, OSError):
        return os.cpu_count() or 1


def _resolve_n_jobs(n_jobs, task_count):
    """Map joblib-style n_jobs values onto a bounded worker count."""
    if task_count <= 0 or n_jobs is None:
        return 1
    n_jobs = int(n_jobs)
    if n_jobs == 0:
        raise ValueError("n_jobs must be non-zero")
    if n_jobs < 0:
        workers = max(_available_cpu_count() + 1 + n_jobs, 1)
    else:
        workers = n_jobs
    return max(1, min(workers, int(task_count)))


def _verbose_level(verbose):
    """Normalize bool/int verbose values."""
    if isinstance(verbose, bool):
        return 1 if verbose else 0
    return int(verbose)


def _format_duration(seconds):
    """Format a duration compactly for progress output."""
    if seconds is None or not np.isfinite(seconds):
        return "n/a"
    seconds = float(seconds)
    if seconds < 1.0:
        return f"{seconds * 1e3:.0f} ms"
    if seconds < 60.0:
        return f"{seconds:.1f} s"
    minutes, rem = divmod(seconds, 60.0)
    return f"{int(minutes)}m{rem:04.1f}s"


def _fit_solver_duration_s(fit):
    """Return optimiser time recorded on a FitResult, excluding Python setup."""
    if fit is None:
        return np.nan
    durations = []
    for name in ("linear_fit_duration_s", "nonlinear_fit_duration_s"):
        value = float(getattr(fit, name, np.nan))
        if np.isfinite(value):
            durations.append(value)
    if durations:
        return float(np.sum(durations))
    return float(getattr(fit, "fit_duration_s", np.nan))


def _progress_stats(completed_results):
    """Summarize completed fits for concise progress logging."""
    fits = [fit for fit in completed_results if fit is not None]
    durations = np.array([
        _fit_solver_duration_s(fit) for fit in fits
    ], float)
    durations = durations[np.isfinite(durations)]
    failures = sum(1 for fit in fits if not fit.success)
    skipped = sum(1 for fit in completed_results if fit is None)
    nfev = sum(int(getattr(fit, "nfev", 0)) for fit in fits)
    mean_fit = float(np.mean(durations)) if durations.size else np.nan
    max_fit = float(np.max(durations)) if durations.size else np.nan
    return mean_fit, max_fit, nfev, failures, skipped


def _print_progress_line(message, final=False):
    """Print progress without filling the terminal with one line per fit."""
    print(message, end="\n" if final else "\r", flush=True)


def _format_progress_summary(label, completed, total, elapsed, completed_results,
                             final=False):
    """Return a compact progress/summary line."""
    mean_fit, max_fit, nfev, failures, skipped = _progress_stats(completed_results)
    verb = "Finished" if final else "Fitting"
    rate = completed / elapsed if elapsed > 0.0 else np.nan
    fev_rate = nfev / elapsed if elapsed > 0.0 else np.nan
    msg = (
        f"{verb} {completed}/{total} {label} "
        f"({100.0 * completed / max(total, 1):.0f}%), "
        f"elapsed={_format_duration(elapsed)}, "
        f"rate={rate:.2f}/s, "
        f"nfev={nfev}, fev_rate={fev_rate:.0f}/s, "
        f"mean_fit={_format_duration(mean_fit)}, "
        f"max_fit={_format_duration(max_fit)}"
    )
    extras = []
    if failures:
        extras.append(f"failed={failures}")
    if skipped:
        extras.append(f"skipped={skipped}")
    if extras:
        msg += ", " + ", ".join(extras)
    return msg


def _format_fit_progress(prefix, index, completed, total, fit,
                         item_label="row", detail=""):
    """Return one compact progress line for a completed fit."""
    item = f"{item_label} {index + 1}" if item_label else f"{index + 1}"
    if detail:
        item = f"{item}, {detail}"
    if fit is None:
        return f"  {prefix} {completed}/{total} ({item}): skipped"
    status = "OK" if fit.success else "FAILED"
    nl = (
        f", anl={fit.anl:.3g}"
        if "anl" in getattr(fit, "parameter_names", ())
        else ""
    )
    return (
        f"  {prefix} {completed}/{total} ({item}): {status}, "
        f"fr={fit.fr/1e6:.4f} MHz, Ql={fit.Ql:.0f}, "
        f"nfev={fit.nfev}, fit={_format_duration(_fit_solver_duration_s(fit))}, "
        f"wall={_format_duration(getattr(fit, 'fit_duration_s', np.nan))}{nl}"
    )


def _parallel_map(func, tasks, n_jobs, verbose=False, label="fits",
                  progress_formatter=None):
    """Run independent fit tasks serially or with a process pool."""
    tasks = list(tasks)
    total = len(tasks)
    workers = _resolve_n_jobs(n_jobs, len(tasks))
    verbose = _verbose_level(verbose)
    start = time.time()
    completed_results = []
    report_every = max(1, int(np.ceil(total / 10.0)))
    if verbose:
        if total == 0:
            print(f"No {label} to fit.")
        elif workers == 1:
            print(f"Fitting {total} {label} serially...")
        else:
            print(f"Fitting {total} {label} with {workers} processes...")
    if workers == 1:
        results = []
        for index, task in enumerate(tasks):
            result = func(task)
            results.append(result)
            completed_results.append(result)
            completed = len(results)
            if verbose >= 2 and progress_formatter is not None:
                print(progress_formatter(index, len(results), total, result))
            elif verbose == 1 and (
                completed == 1 or completed == total or completed % report_every == 0
            ):
                _print_progress_line(
                    _format_progress_summary(
                        label, completed, total, time.time() - start,
                        completed_results, final=completed == total,
                    ),
                    final=completed == total,
                )
        if verbose >= 2 and total:
            print(_format_progress_summary(
                label, total, total, time.time() - start,
                completed_results, final=True,
            ))
        return results

    results = [None] * len(tasks)
    with ProcessPoolExecutor(max_workers=workers) as pool:
        future_to_index = {
            pool.submit(func, task): index
            for index, task in enumerate(tasks)
        }
        completed = 0
        for future in as_completed(future_to_index):
            index = future_to_index[future]
            result = future.result()
            results[index] = result
            completed += 1
            completed_results.append(result)
            if verbose >= 2 and progress_formatter is not None:
                print(progress_formatter(index, completed, total, result))
            elif verbose == 1 and (
                completed == 1 or completed == total or completed % report_every == 0
            ):
                _print_progress_line(
                    _format_progress_summary(
                        label, completed, total, time.time() - start,
                        completed_results, final=completed == total,
                    ),
                    final=completed == total,
                )
    if verbose >= 2 and total:
        print(_format_progress_summary(
            label, total, total, time.time() - start,
            completed_results, final=True,
        ))
    return results


def _fit_sweep_stack_one(task):
    """Worker for one already-windowed sweep stack row."""
    f, z, e, nonlinear, sweep_direction, fit_kwargs, tone_index, is_blind = task
    # Always dispatch through fit_resonance so nonlinear True/False/'auto' is
    # interpreted in one place. Calling fit_resonance_nonlinear directly here
    # would treat the truthy string 'auto' as plain True and silently skip the
    # linear-sufficiency shortcut.
    fit = fit_resonance(
        f, z, z_err=e, nonlinear=nonlinear, sweep_direction=sweep_direction,
        **fit_kwargs)
    fit.tone_index = int(tone_index)
    fit.is_blind = bool(is_blind)
    return fit


def _batch_fit_one(task):
    """Worker for one batch_fit resonance window."""
    (i, total, fr_guess, fw, zw, ew, ow, nonlinear, sweep_direction,
     base_guess, fit_kwargs) = task
    if len(fw) < 5:
        return None

    guess = dict(base_guess or {})
    guess.setdefault("fr", fr_guess)
    kwargs = dict(fit_kwargs, initial_guess=guess, optimizer_z_err=ow)
    # Single dispatch point for nonlinear True/False/'auto' (see
    # _fit_sweep_stack_one); routing through fit_resonance keeps 'auto' working.
    fit = fit_resonance(
        fw, zw, z_err=ew, nonlinear=nonlinear, sweep_direction=sweep_direction,
        **kwargs)
    return fit


def _stack_rows(f_stack, z_stack, z_err_stack=None):
    """Turn 1D/2D sweep stacks into a list of per-resonance rows."""
    z = np.asarray(z_stack, complex)
    f = np.asarray(f_stack, float)
    e = None if z_err_stack is None else np.asarray(z_err_stack)
    if z.ndim == 1:
        return [(f.ravel(), z.ravel(), None if e is None else e.ravel())]
    rows = []
    if f.ndim == 1:
        if z.shape[-1] == f.size:
            for i in range(z.shape[0]):
                rows.append((f, z[i], None if e is None else e[i]))
        elif z.shape[0] == f.size:
            for i in range(z.shape[1]):
                rows.append((f, z[:, i], None if e is None else e[:, i]))
        else:
            raise ValueError("one axis of z_stack must match the 1D frequency array")
    elif f.shape == z.shape:
        if z.ndim != 2:
            rows.append((f.ravel(), z.ravel(), None if e is None else e.ravel()))
        elif z.shape[0] <= z.shape[1]:
            for i in range(z.shape[0]):
                rows.append((f[i], z[i], None if e is None else e[i]))
        else:
            for i in range(z.shape[1]):
                rows.append((f[:, i], z[:, i], None if e is None else e[:, i]))
    else:
        raise ValueError("f_stack and z_stack must have compatible shapes")
    return rows


def _targeted_sweep_tone_indices(sweep_data, n_tones, skip_blind=True):
    """Return original tone indices to fit for targeted sweep data."""
    def metadata_sequence(*values):
        for value in values:
            if value is None:
                continue
            try:
                if len(value) == 0:
                    continue
            except TypeError:
                pass
            return value
        return []

    indices = list(range(n_tones))
    if not skip_blind:
        return indices, set()

    info = sweep_data.get("info", {}) or {}
    tones = info.get("tones", {}) if isinstance(info, dict) else {}
    metadata = sweep_data.get("tone_metadata", {}) or {}
    blind_indices = metadata_sequence(
        metadata.get("blind_indices"),
        tones.get("blind_indices"),
    )
    try:
        blind = {int(i) for i in blind_indices if 0 <= int(i) < n_tones}
    except TypeError:
        blind = set()
    regular_indices = metadata_sequence(
        metadata.get("regular_indices"),
        tones.get("regular_indices"),
    )
    try:
        has_regular = len(regular_indices) > 0
    except TypeError:
        has_regular = bool(regular_indices)
    if has_regular:
        try:
            regular = [int(i) for i in regular_indices if 0 <= int(i) < n_tones]
        except TypeError:
            regular = []
        if regular:
            return regular, blind
    return [i for i in indices if i not in blind], blind


def fit_sweep_stack(f_stack, z_stack, z_err_stack=None, nonlinear=True,
                    sweep_direction="up", n_jobs=1, verbose=False,
                    **fit_kwargs):
    """Fit each row/column in a stack of already-windowed resonance sweeps.

    This is a low-level array helper. It does not understand server sweep
    metadata, blind-tone flags, or original tone indices; each returned
    ``FitResult.tone_index`` is the row/column number assigned by the stack
    adapter. Rows are independent fits, so even if the stack represents the
    same resonator over several powers, this function does not use the
    previous row's result as the next row's starting point.

    ``n_jobs`` follows the joblib convention: ``1`` is serial, ``-1`` uses all
    visible CPUs, and ``-2`` uses all but one. Parallel fitting uses separate
    processes. Pass ``verbose=True`` to print completion progress.
    Single-fit options, including ``initial_guess``, ``param_bounds`` and
    ``param_fixed`` and ``min_dip_depth_db``, are forwarded through
    ``**fit_kwargs``. For large runs pass ``store_optimizer=False`` to drop the
    bulky per-fit optimiser objects, which markedly shrinks the result pickled
    back from each worker process.

    Parameters
    ----------
    f_stack : array-like
        Stack of per-row frequency arrays (Hz).
    z_stack : array-like of complex
        Matching stack of complex S21 rows.
    z_err_stack : array-like of complex or None, optional
        Matching per-point S21 uncertainties forwarded as ``z_err`` to each
        row fit; ``None`` (default) fits unweighted.
    nonlinear : bool or {'auto'}, optional
        How each row handles the Duffing ``anl`` term (default ``True``; see
        :func:`fit_resonance`). ``True`` fits ``anl`` on every row, ``False``
        fits the linear model only, and ``'auto'`` keeps rows the linear model
        already explains as linear (dropping their ``anl``) and only refines
        the rest.
    sweep_direction : {'up', 'down'}, optional
        Direction the sweeps were taken in (default ``'up'``).
    """
    rows = _stack_rows(f_stack, z_stack, z_err_stack)
    tasks = [
        (f, z, e, nonlinear, sweep_direction, dict(fit_kwargs), i, False)
        for i, (f, z, e) in enumerate(rows)
    ]
    return _parallel_map(
        _fit_sweep_stack_one, tasks, n_jobs, verbose=verbose,
        label="stack rows",
        progress_formatter=lambda index, completed, total, fit:
            _format_fit_progress("Stack fit", index, completed, total, fit),
    )


def _prepare_sweep_stack(sweep_data, zerr=None):
    """Load a sweep-data dict as frequency/complex/error arrays without flattening."""
    sf = np.asarray(sweep_data["sweep_f"], float)
    si = np.asarray(sweep_data["sweep_i"], float)
    sq = np.asarray(sweep_data["sweep_q"], float)
    z = si + 1j * sq
    if zerr is None and ("sweep_ei" in sweep_data or "sweep_eq" in sweep_data):
        ei = np.asarray(sweep_data.get("sweep_ei", np.zeros_like(si)), float)
        eq = np.asarray(sweep_data.get("sweep_eq", np.zeros_like(sq)), float)
        zerr = ei + 1j * eq
    elif zerr is not None:
        ei, eq = _prepare_z_error(zerr, z.shape)
        zerr = ei + 1j * eq
    return sf, z, zerr


def _is_targeted_sweep_data(sweep_data):
    """Return True when sweep_data already contains one targeted trace per tone."""
    sf = np.asarray(sweep_data["sweep_f"], float)
    if sf.ndim == 1:
        return True
    return not sweep_data.get("wideband_sweep", False) and np.atleast_2d(sf).shape[0] != 1


def _flatten_sweep(sweep_data, zerr=None):
    """Flatten a sweep-data dict into sorted frequency, complex S21, and errors."""
    sf, z, zerr = _prepare_sweep_stack(sweep_data, zerr)
    f_all, z_all = np.atleast_2d(sf).ravel(), np.atleast_2d(z).ravel()
    e_all = None if zerr is None else np.broadcast_to(np.atleast_2d(zerr), np.atleast_2d(z).shape).ravel()
    order = np.argsort(f_all)
    return f_all[order], z_all[order], None if e_all is None else e_all[order]


def _batch_fit_finder_defaults(frequencies, filter_params=None,
                               finder_params=None):
    """Return conservative resonance-search defaults for fitting windows."""
    from .peak_finder import wideband_resonance_search_params

    return wideband_resonance_search_params(
        frequencies, filter_params=filter_params, finder_params=finder_params)


def batch_fit(sweep_data, resonances=None, data_format="log_magnitude",
              filter_params=None, finder_params=None, window_fwhm=10.0,
              nonlinear=True, sweep_direction="up", verbose=True,
              z_err=None, optimizer_z_err=None, use_error_weights=None,
              max_points=None, n_jobs=1, find_resonances=False,
              skip_blind=True,
              **fit_kwargs):
    """Fit one or more resonances from a sweep-data dict.

    This is the high-level adapter for sweep-data dictionaries returned by the
    readout server. For targeted sweeps, it fits each regular tone trace in
    that one sweep independently and preserves the original server tone index
    on each ``FitResult``. Use ``fit_sweep_stack`` instead when you already
    have raw frequency/S21 arrays rather than a sweep-data dictionary.

    ``n_jobs`` follows the joblib convention: ``1`` is serial, ``-1`` uses all
    visible CPUs, and ``-2`` uses all but one. Parallel fitting uses separate
    processes. Single-fit options, including ``initial_guess``,
    ``param_bounds``, ``param_fixed``, ``min_dip_depth_db`` and
    ``store_optimizer`` (pass ``False`` on large runs to shrink the per-fit
    result pickled back from workers), are forwarded through
    ``**fit_kwargs``. When ``resonances`` is omitted, targeted per-tone sweeps
    fit one trace per tone by default. Pass ``find_resonances=True`` to
    auto-detect resonance windows from a concatenated sweep instead. That
    auto-detection uses fitter-oriented defaults when no ``filter_params`` or
    ``finder_params`` are supplied: inner 10-90% of the frequency span,
    prominence 1-100 dB, width 1 kHz-10 MHz, minimum spacing 100 kHz,
    lowpass 0.5, highpass 0, and dip finding. By default, targeted sweeps skip
    tones marked as blind in ``sweep_data['info']['tones']`` or
    ``sweep_data['tone_metadata']``.

    Parameters
    ----------
    sweep_data : dict
        A parsed sweep-data dict from the readout client/server.
    resonances : list or None, optional
        Explicit resonance windows; ``None`` (default) fits one trace per tone
        (targeted) or auto-detects when ``find_resonances=True``.
    data_format : str, optional
        Interpretation of the sweep traces (default ``'log_magnitude'``).
    window_fwhm : float, optional
        Half-window width, in dip FWHMs, kept around each resonance for
        auto-detected windows (default ``10.0``).
    nonlinear : bool or {'auto'}, optional
        How to handle the Duffing ``anl`` term (default ``True``; see
        :func:`fit_resonance`). ``True`` fits ``anl`` on every tone, ``False``
        fits the linear model only, and ``'auto'`` fits each tone with the
        linear model and only refines the ones it cannot explain (dropping
        ``anl`` for the rest).
    sweep_direction : {'up', 'down'}, optional
        Direction the sweep was taken in (default ``'up'``).
    verbose : bool, optional
        Print per-fit progress (default ``True``).
    z_err : array-like or None, optional
        Per-point S21 uncertainty for the fits / uncertainty propagation.
    optimizer_z_err : optional
        Errors used to weight the optimiser residuals (``True`` reuses
        ``z_err``); ``None`` (default) fits unweighted.
    use_error_weights : bool or None, optional
        Convenience switch equivalent to setting ``optimizer_z_err``.
    max_points : int or None, optional
        Cap on points fed to each fit (decimated if exceeded); ``None`` keeps
        all points.
    skip_blind : bool, optional
        Skip tones flagged as blind in the sweep metadata (default ``True``).
    """
    from .peak_finder import find_mkid_resonances

    sf, z_stack, e_stack = _prepare_sweep_stack(sweep_data, z_err)
    f_all, z_all, e_all = _flatten_sweep(sweep_data, z_err)
    if use_error_weights is not None:
        optimizer_z_err = True if use_error_weights else None
    same_opt = (
        isinstance(optimizer_z_err, str)
        and optimizer_z_err in ("z_err", "same", "measurement")
    )
    none_opt = optimizer_z_err is None or optimizer_z_err is False
    opt_all = e_all if optimizer_z_err is True or same_opt else None
    opt_stack = e_stack if optimizer_z_err is True or same_opt else None
    if not (none_opt or optimizer_z_err is True or same_opt):
        _, _, opt_stack = _prepare_sweep_stack(sweep_data, optimizer_z_err)
        _, _, opt_all = _flatten_sweep(sweep_data, optimizer_z_err)
    base_guess = _parameter_dict(fit_kwargs.pop("initial_guess", None))

    if resonances is None:
        if _is_targeted_sweep_data(sweep_data) and not find_resonances:
            rows = _stack_rows(sf, z_stack, e_stack)
            tone_indices, blind_indices = _targeted_sweep_tone_indices(
                sweep_data, len(rows), skip_blind=skip_blind)
            if opt_stack is None:
                opt_rows = [None] * len(rows)
            else:
                opt_rows = [row for _, row, _ in _stack_rows(sf, opt_stack)]

            tasks = []
            for tone_index in tone_indices:
                f_row, z_row, e_row = rows[tone_index]
                opt_row = opt_rows[tone_index]
                kwargs = dict(fit_kwargs)
                if base_guess is not None:
                    kwargs["initial_guess"] = dict(base_guess)
                kwargs["optimizer_z_err"] = opt_row
                tasks.append((
                    f_row, z_row, e_row, nonlinear, sweep_direction, kwargs,
                    tone_index, tone_index in blind_indices,
                ))

            def targeted_progress(index, completed, total, fit):
                tone_index = tasks[index][6]
                return _format_fit_progress(
                    "Tone", tone_index, completed, total, fit,
                    item_label="index",
                )

            return _parallel_map(
                _fit_sweep_stack_one, tasks, n_jobs, verbose=verbose,
                label="tones",
                progress_formatter=targeted_progress,
            )

        if not find_resonances:
            raise ValueError(
                "batch_fit() requires explicit resonances for concatenated sweeps; "
                "pass find_resonances=True to auto-detect them from sweep_data."
            )
        filter_params, finder_params = _batch_fit_finder_defaults(
            f_all, filter_params=filter_params, finder_params=finder_params)
        resonances = find_mkid_resonances(
            f_all, z_all, data_format=data_format,
            filter_params=filter_params, finder_params=finder_params,
            verbose=verbose,
        )
    if resonances and isinstance(resonances[0], (int, float, np.integer, np.floating)):
        resonances = [type("R", (), {"frequency": float(fr), "fwhm": None})()
                      for fr in resonances]

    work = []
    for i, res in enumerate(resonances or []):
        fr_guess = float(getattr(res, "frequency", res))
        fwhm = getattr(res, "fwhm", None)
        if fwhm is None or fwhm <= 0:
            fwhm = fr_guess / 1000.0
        half_window = window_fwhm * fwhm / 2.0
        mask = (f_all >= fr_guess - half_window) & (f_all <= fr_guess + half_window)
        if np.count_nonzero(mask) < 10:
            mask = (f_all >= fr_guess - 3.0 * half_window) & (f_all <= fr_guess + 3.0 * half_window)
        fw, zw = f_all[mask], z_all[mask]
        ew = None if e_all is None else e_all[mask]
        ow = None if opt_all is None else opt_all[mask]
        if max_points is not None and len(fw) > max_points:
            idx = np.unique(np.linspace(0, len(fw) - 1, int(max_points)).round().astype(int))
            fw, zw = fw[idx], zw[idx]
            ew = None if ew is None else ew[idx]
            ow = None if ow is None else ow[idx]
        work.append((i, fr_guess, fw, zw, ew, ow))

    total = len(work)
    tasks = [
        (i, total, fr_guess, fw, zw, ew, ow, nonlinear, sweep_direction,
         base_guess, dict(fit_kwargs))
        for i, fr_guess, fw, zw, ew, ow in work
    ]
    def batch_progress(index, completed, total, fit):
        i, _, fr_guess, fw, *_ = tasks[index]
        detail = f"{fr_guess/1e6:.3f} MHz"
        if len(fw) < 5:
            return (
                f"  Resonance {completed}/{total} "
                f"(index {i + 1}, {detail}): skipped ({len(fw)} points)"
            )
        return _format_fit_progress(
            "Resonance", i, completed, total, fit,
            item_label="index", detail=detail,
        )

    fits = _parallel_map(
        _batch_fit_one, tasks, n_jobs, verbose=verbose,
        label="resonances", progress_formatter=batch_progress,
    )
    return [fit for fit in fits if fit is not None]


def extract_parameters(fit_results):
    """Extract common fitted parameters from one or more FitResult objects.

    ``fit_results`` is a single :class:`FitResult`, an iterable of them, or
    ``None``.  Returns a dict of arrays keyed by parameter name (one entry per
    fit), with ``anl`` set to NaN for noise-only rows.
    """
    keys = ("fr", "Ql", "Qi", "Qc", "Qc_abs", "phi", "a", "alpha",
            "tau", "anl", "nonlinear_detuning_hz", "residual_rms",
            "weighted_rms", "reduced_chi2", "success", "nfev",
            "linear_nfev", "nonlinear_nfev", "fit_duration_s",
            "linear_fit_duration_s", "nonlinear_fit_duration_s",
            "optimizer_cost", "linear_cost", "nonlinear_cost",
            "empirical_fr", "empirical_linewidth_hz", "empirical_Ql",
            "empirical_Qc", "empirical_Qi", "empirical_dip_depth_db",
            "empirical_skew", "noise_only")
    if fit_results is None:
        fit_results = ()
    elif isinstance(fit_results, FitResult):
        fit_results = (fit_results,)
    else:
        fit_results = tuple(fit_results)

    if not fit_results:
        out = {k: np.array([]) for k in keys}
        out["Qe"] = np.array([])
        return out
    out = {}
    defaults = {"noise_only": False}
    for key in keys:
        out[key] = np.array([
            getattr(r, key, defaults.get(key, np.nan))
            for r in fit_results
        ])
    noise_only = np.asarray(out["noise_only"], dtype=bool)
    if np.any(noise_only):
        out["anl"] = np.asarray(out["anl"], dtype=float)
        out["anl"][noise_only] = np.nan
    out["Qe"] = np.array([getattr(r, "Qe", complex(np.nan, np.nan)) for r in fit_results])
    return out


def fit_result_summary_row(
    fit,
    keys=FIT_SUMMARY_KEYS,
    uncertainty_keys=FIT_UNCERTAINTY_KEYS,
    include_message=True,
):
    """Return CSV-friendly scalar fields for one ``FitResult``.

    Power-sweep tooling adds sweep/tone/power columns around this generic
    fitter-level row, so the list of exported fit fields lives with the fitter
    instead of being duplicated by each workflow.

    Parameters
    ----------
    fit : FitResult
        The fit to summarise.
    keys : sequence of str, optional
        Fitted-value field names to export (default :data:`FIT_SUMMARY_KEYS`).
    uncertainty_keys : sequence of str, optional
        Field names to also export as ``<name>_err`` uncertainty columns
        (default :data:`FIT_UNCERTAINTY_KEYS`).
    include_message : bool, optional
        Include the solver ``message`` string column (default ``True``).
    """
    row = {}
    uncertainty_keys = set(uncertainty_keys or ())
    uncertainties = getattr(fit, "parameter_uncertainties", None) or {}
    for key in tuple(keys):
        if key == "noise_only":
            value = bool(getattr(fit, key, False))
        else:
            value = getattr(fit, key, np.nan)
        if isinstance(value, np.generic):
            value = value.item()
        row[key] = value
        if key in uncertainty_keys:
            row[f"{key}_err"] = float(uncertainties.get(key, np.nan))
    if include_message:
        row["message"] = getattr(fit, "message", "")
    return row
