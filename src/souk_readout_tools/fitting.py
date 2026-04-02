"""
Resonator fitting for MKID S21 data.

Provides nonlinear least-squares fitting of resonator models to
complex S21 sweep data, with batch fitting over multiple tones.

Models:
- Notch-type (Khalil 2012): S21 = a * exp(j*alpha) * exp(-2*pi*j*f*tau) *
  (1 - (Ql/|Qc|) * exp(j*phi) / (1 + 2*j*Ql*(f-fr)/fr))
- Nonlinear (Duffing): extends the notch model with a nonlinear kinetic
  inductance parameter that captures bifurcation in driven MKIDs.

Fitting workflow:
1. find_resonances() to locate resonance frequencies
2. fit_resonance() or fit_resonance_nonlinear() to fit a single resonance
3. batch_fit() to fit all resonances in a sweep

References:
    Khalil et al., J. Appl. Phys. 111, 054510 (2012)
    Probst et al., Rev. Sci. Instrum. 86, 024706 (2015)
    Swenson et al., J. Appl. Phys. 113, 104507 (2013)
"""

import numpy as np
from scipy.optimize import least_squares
from dataclasses import dataclass, field
from typing import Optional, List, Dict


@dataclass
class FitResult:
    """Result of a single resonator fit."""
    # Fitted parameters
    fr: float                      # Resonance frequency (Hz)
    Ql: float                      # Loaded quality factor
    Qc_abs: float                  # Coupling Q magnitude
    phi: float                     # Impedance mismatch angle (rad)
    a: float                       # Amplitude scaling
    alpha: float                   # Phase offset (rad)
    tau: float                     # Cable delay (s)

    # Derived quantities
    Qi: float                      # Internal quality factor
    Qc: complex                    # Complex coupling Q
    iq_center: complex             # IQ circle center (cable-delay removed)
    iq_radius: float               # IQ circle radius (cable-delay removed)

    # Fit quality
    residual_rms: float            # RMS of fit residuals
    success: bool                  # Whether the fit converged

    # Nonlinear parameter (only set by fit_resonance_nonlinear)
    anl: float = 0.0              # Nonlinear kinetic inductance parameter
    sweep_direction: str = 'up'   # Sweep direction used for nonlinear fit
    message: str = ''              # Fit status message

    # Data used for fitting
    f_data: Optional[np.ndarray] = field(default=None, repr=False)
    z_data: Optional[np.ndarray] = field(default=None, repr=False)
    z_fit: Optional[np.ndarray] = field(default=None, repr=False)


def _s21_notch(f, fr, Ql, Qc_abs, phi, a, alpha, tau):
    """
    Notch-type resonator S21 model.

    S21(f) = a * exp(j*alpha) * exp(-2*pi*j*f*tau) *
             (1 - (Ql/Qc_abs) * exp(j*phi) / (1 + 2j*Ql*(f-fr)/fr))
    """
    x = (f - fr) / fr
    environment = a * np.exp(1j * alpha) * np.exp(-2j * np.pi * f * tau)
    resonance = 1.0 - (Ql / Qc_abs) * np.exp(1j * phi) / (1.0 + 2j * Ql * x)
    return environment * resonance


def _iq_circle_params(Ql, Qc_abs, phi, a, alpha):
    """
    Compute IQ circle center and radius in the cable-delay-removed plane.

    After removing cable delay, the S21 model traces a circle:
        S21(x) = a*exp(j*alpha) * (1 - d/(1 + 2j*Ql*x))
    where d = (Ql/Qc_abs)*exp(j*phi). This is a circle with:
        center = a*exp(j*alpha) * (1 - d/2)
        radius = a * |d| / 2 = a * Ql / (2*Qc_abs)

    Returns:
        (center, radius): complex center and float radius.
    """
    d = (Ql / Qc_abs) * np.exp(1j * phi)
    center = a * np.exp(1j * alpha) * (1.0 - d / 2.0)
    radius = a * abs(d) / 2.0
    return complex(center), float(radius)


def _residuals(params, f, z_data):
    """Residual vector for least-squares fitting (real, imag interleaved)."""
    fr, Ql, Qc_abs, phi, a, alpha, tau = params
    z_model = _s21_notch(f, fr, Ql, Qc_abs, phi, a, alpha, tau)
    diff = z_model - z_data
    return np.concatenate([diff.real, diff.imag])


def _solve_duffing(y0, anl, sweep_direction='up'):
    """
    Solve the Duffing cubic: 4*y^3 - 4*y0*y^2 + y - y0 - anl = 0.

    This arises from the nonlinear kinetic inductance shift in a driven
    MKID. The three roots correspond to the bistable branches; the
    sweep direction selects which branch is followed.

    Args:
        y0: Normalized detuning array, y0 = (f-fr)/(fr * Qr_inv).
        anl: Nonlinear parameter (scalar or broadcastable).
        sweep_direction: 'up' or 'down' frequency sweep.

    Returns:
        y: Real solution array (same shape as y0).
    """
    y0 = np.asarray(y0, dtype=float)
    anl = np.broadcast_to(np.asarray(anl, dtype=float), y0.shape)

    A = 4.0
    B = -4.0 * y0
    C = 1.0
    D = -y0 - anl

    Delta0 = B * B - 3.0 * A * C
    Delta1 = 2.0 * B**3 - 9.0 * A * B * C + 27.0 * A**2 * D

    disc = np.emath.sqrt(Delta1**2 - 4.0 * Delta0**3)
    Ct = 0.5 * (Delta1 + disc)
    Ct = np.where(np.abs(Ct) < 1e-48, 1e-24 + 0j, Ct)
    Cc = Ct ** (1.0 / 3.0)

    xi = -0.5 + 0.5j * np.sqrt(3.0)
    xi2 = xi * xi
    inv3A = 1.0 / (3.0 * A)

    r1 = -(B + Cc + Delta0 / Cc) * inv3A
    r2 = -(B + xi * Cc + Delta0 / (xi * Cc)) * inv3A
    r3 = -(B + xi2 * Cc + Delta0 / (xi2 * Cc)) * inv3A

    if sweep_direction == 'up':
        y = np.where(np.isclose(r1.imag, 0, atol=1e-12),
                     r1.real, np.maximum(r2.real, r3.real))
    elif sweep_direction == 'down':
        y = np.where(np.isclose(r2.imag, 0, atol=1e-12)
                     | np.isclose(r3.imag, 0, atol=1e-12),
                     np.maximum(r2.real, r3.real), r1.real)
    else:
        raise ValueError('sweep_direction must be "up" or "down".')
    return y


def _s21_nonlinear(f, fr, Qi, Qc_abs, phi, a, alpha, tau, anl,
                   sweep_direction='up'):
    """
    Nonlinear resonator S21 model with Duffing bifurcation.

    Extends the Khalil notch model with a nonlinear kinetic inductance
    parameter. When anl ~ 0, this reduces to the standard linear model.

    The resonance part is:
        S21_res = 1 - (1/Qe) / (1/Qi + Re(1/Qe) + 2j*x_nl)
    where x_nl is the nonlinearly shifted detuning from the Duffing equation.
    """
    environment = a * np.exp(1j * alpha) * np.exp(-2j * np.pi * f * tau)

    Qe = Qc_abs / np.exp(1j * phi)
    Qr_inv = 1.0 / Qi + np.real(1.0 / Qe)

    if anl < 1e-12:
        x = (f - fr) / fr
        resonance = 1.0 - (1.0 / Qe) / (1.0 / Qi + 1.0 / Qe + 2j * x)
    else:
        x0 = (f - fr) / fr
        y0 = x0 / Qr_inv
        y = _solve_duffing(y0, anl, sweep_direction)
        resonance = 1.0 - (1.0 / Qe) / (Qr_inv * (1.0 + 2j * y))

    return environment * resonance


def _residuals_nonlinear(params, f, z_data, sweep_direction):
    """Residual vector for nonlinear least-squares fitting."""
    fr, Qi, Qc_abs, phi, a, alpha, tau, anl = params
    z_model = _s21_nonlinear(f, fr, Qi, Qc_abs, phi, a, alpha, tau, anl,
                             sweep_direction)
    diff = z_model - z_data
    return np.concatenate([diff.real, diff.imag])


def _estimate_initial_params(f, z):
    """
    Estimate initial fit parameters from data.

    Uses simple heuristics: amplitude from baseline, resonance frequency
    from minimum |S21|, Q from 3dB bandwidth estimate.
    """
    mag = np.abs(z)

    # Cable delay from phase slope
    phase = np.unwrap(np.angle(z))
    dphase_df = np.gradient(phase, f)
    tau = -np.median(dphase_df) / (2 * np.pi)

    # Remove cable delay for better estimates
    z_nodelay = z * np.exp(1j * 2 * np.pi * f * tau)

    # Amplitude and phase offset from endpoints (off-resonance)
    n_edge = max(3, len(f) // 10)
    z_baseline = np.mean(np.concatenate([z_nodelay[:n_edge], z_nodelay[-n_edge:]]))
    a = np.abs(z_baseline)
    alpha = np.angle(z_baseline)

    # Resonance frequency from minimum |S21|
    mag_nodelay = np.abs(z_nodelay)
    idx_min = np.argmin(mag_nodelay)
    fr = f[idx_min]

    # Q estimates from 3dB bandwidth
    min_mag = mag_nodelay[idx_min]
    max_mag = np.max(mag_nodelay)
    half_power = (min_mag + max_mag) / 2
    below = np.where(mag_nodelay < half_power)[0]
    if len(below) > 1:
        bandwidth = f[below[-1]] - f[below[0]]
        bandwidth = max(bandwidth, np.median(np.diff(f)))
    else:
        bandwidth = (f[-1] - f[0]) / 10

    Ql = fr / bandwidth
    # Dip depth gives Ql/Qc ratio
    dip_ratio = min_mag / max_mag
    Qc_abs = Ql / max(1e-6, 1.0 - dip_ratio)
    phi = 0.0

    return fr, Ql, Qc_abs, phi, a, alpha, tau


def fit_resonance(f, z, fr_guess=None, window_factor=5.0):
    """
    Fit a single resonance to the notch-type resonator model.

    Args:
        f: 1D frequency array (Hz).
        z: 1D complex S21 array.
        fr_guess: Approximate resonance frequency (Hz). If None,
                  estimated from the data.
        window_factor: If fr_guess is given and the data spans a wider
                       range, crop to +/- window_factor * estimated FWHM
                       around fr_guess. Set to None to use all data.

    Returns:
        FitResult with fitted parameters and derived quantities.
    """
    f = np.asarray(f, dtype=float)
    z = np.asarray(z, dtype=complex)

    # Initial parameter estimates
    p0 = list(_estimate_initial_params(f, z))

    if fr_guess is not None:
        p0[0] = fr_guess

    # Bounds
    fr0, Ql0, Qc0, phi0, a0, alpha0, tau0 = p0
    df = f[-1] - f[0]
    bounds_lower = [f[0] - df * 0.1, 1e1, 1e1, -np.pi, a0 * 0.01, -2 * np.pi, tau0 - 1e-6]
    bounds_upper = [f[-1] + df * 0.1, 1e8, 1e8, np.pi, a0 * 100, 2 * np.pi, tau0 + 1e-6]

    # Clamp initial values to bounds
    p0_clamped = [max(lo, min(hi, v)) for v, lo, hi in zip(p0, bounds_lower, bounds_upper)]

    try:
        result = least_squares(
            _residuals, p0_clamped, args=(f, z),
            bounds=(bounds_lower, bounds_upper),
            method='trf', max_nfev=5000,
        )
        success = result.success
        message = result.message
        fr, Ql, Qc_abs, phi, a, alpha, tau = result.x
    except Exception as e:
        success = False
        message = str(e)
        fr, Ql, Qc_abs, phi, a, alpha, tau = p0_clamped

    # Derived quantities
    Qc = Qc_abs * np.exp(-1j * phi)
    Qi_inv = 1.0 / Ql - np.real(1.0 / Qc)
    Qi = 1.0 / Qi_inv if Qi_inv > 0 else float('inf')
    iq_center, iq_radius = _iq_circle_params(Ql, Qc_abs, phi, a, alpha)

    z_fit = _s21_notch(f, fr, Ql, Qc_abs, phi, a, alpha, tau)
    residual_rms = float(np.sqrt(np.mean(np.abs(z_fit - z) ** 2)))

    return FitResult(
        fr=fr, Ql=Ql, Qc_abs=Qc_abs, phi=phi,
        a=a, alpha=alpha, tau=tau,
        Qi=Qi, Qc=Qc, iq_center=iq_center, iq_radius=iq_radius,
        residual_rms=residual_rms,
        success=success, message=message,
        f_data=f, z_data=z, z_fit=z_fit,
    )


def fit_resonance_nonlinear(f, z, fr_guess=None, sweep_direction='up',
                            window_factor=5.0):
    """
    Fit a single resonance with the nonlinear Duffing resonator model.

    Uses (fr, Qi, Qc_abs, phi, a, alpha, tau, anl) as fit parameters.
    A linear fit is run first to seed the initial parameter estimates.

    Args:
        f: 1D frequency array (Hz).
        z: 1D complex S21 array.
        fr_guess: Approximate resonance frequency (Hz). If None,
                  estimated from the data.
        sweep_direction: 'up' or 'down' frequency sweep direction.
        window_factor: If fr_guess is given and the data spans a wider
                       range, crop to +/- window_factor * estimated FWHM
                       around fr_guess. Set to None to use all data.

    Returns:
        FitResult with fitted parameters and derived quantities.
    """
    f = np.asarray(f, dtype=float)
    z = np.asarray(z, dtype=complex)

    # Run linear fit first to get good initial estimates
    linear_fit = fit_resonance(f, z, fr_guess=fr_guess,
                               window_factor=window_factor)

    # Seed nonlinear params from linear fit (Qi, not Ql)
    fr0 = linear_fit.fr
    Qi0 = linear_fit.Qi if np.isfinite(linear_fit.Qi) else linear_fit.Ql * 2
    Qc0 = linear_fit.Qc_abs
    phi0 = linear_fit.phi
    a0 = linear_fit.a
    alpha0 = linear_fit.alpha
    tau0 = linear_fit.tau
    anl0 = 0.0

    p0 = [fr0, Qi0, Qc0, phi0, a0, alpha0, tau0, anl0]

    df = f[-1] - f[0]
    bounds_lower = [f[0] - df * 0.1, 1e1, 1e1, -np.pi,
                    a0 * 0.01, -2 * np.pi, tau0 - 1e-6, 0.0]
    bounds_upper = [f[-1] + df * 0.1, 1e8, 1e8, np.pi,
                    a0 * 100, 2 * np.pi, tau0 + 1e-6, 1.0]

    p0_clamped = [max(lo, min(hi, v))
                  for v, lo, hi in zip(p0, bounds_lower, bounds_upper)]

    try:
        result = least_squares(
            _residuals_nonlinear, p0_clamped,
            args=(f, z, sweep_direction),
            bounds=(bounds_lower, bounds_upper),
            method='trf', max_nfev=10000,
        )
        success = result.success
        message = result.message
        fr, Qi, Qc_abs, phi, a, alpha, tau, anl = result.x
    except Exception as e:
        success = False
        message = str(e)
        fr, Qi, Qc_abs, phi, a, alpha, tau, anl = p0_clamped

    # Derived quantities
    Qc = Qc_abs * np.exp(-1j * phi)
    Ql = 1.0 / (1.0 / Qi + np.real(1.0 / Qc))
    iq_center, iq_radius = _iq_circle_params(Ql, Qc_abs, phi, a, alpha)

    z_fit = _s21_nonlinear(f, fr, Qi, Qc_abs, phi, a, alpha, tau, anl,
                           sweep_direction)
    residual_rms = float(np.sqrt(np.mean(np.abs(z_fit - z) ** 2)))

    return FitResult(
        fr=fr, Ql=Ql, Qc_abs=Qc_abs, phi=phi,
        a=a, alpha=alpha, tau=tau,
        Qi=Qi, Qc=Qc, iq_center=iq_center, iq_radius=iq_radius,
        anl=anl, sweep_direction=sweep_direction,
        residual_rms=residual_rms,
        success=success, message=message,
        f_data=f, z_data=z, z_fit=z_fit,
    )


def batch_fit(sweep_data, resonances=None, data_format='log_magnitude',
              filter_params=None, finder_params=None,
              window_fwhm=10.0, nonlinear=False, sweep_direction='up',
              verbose=True):
    """
    Fit all resonances in a sweep dataset.

    If resonances are not provided, finds them automatically using
    peak_finder.find_mkid_resonances.

    Args:
        sweep_data: Sweep data dict with 'sweep_f', 'sweep_i', 'sweep_q'.
            Can be wideband (1, N) or per-tone (N_points, N_tones).
        resonances: List of ResonanceResult objects (from find_resonances),
                    or list of frequencies in Hz. If None, finds resonances
                    automatically.
        data_format: Passed to find_mkid_resonances if finding resonances.
        filter_params: Passed to find_mkid_resonances.
        finder_params: Passed to find_mkid_resonances.
        window_fwhm: Number of estimated FWHM widths to include around
                     each resonance for fitting. Default 10.
        nonlinear: If True, use the nonlinear Duffing resonator model
                   (fit_resonance_nonlinear) instead of the linear model.
        sweep_direction: 'up' or 'down', used when nonlinear=True.
        verbose: Print progress.

    Returns:
        list of FitResult objects, one per resonance.
    """
    from .peak_finder import find_mkid_resonances, ResonanceResult

    # Get full frequency/S21 arrays
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])

    is_wideband = sweep_data.get('wideband_sweep', False)
    if is_wideband or sf.shape[0] == 1:
        f_all = sf.ravel()
        z_all = si.ravel() + 1j * sq.ravel()
    else:
        # Per-tone: concatenate all tones
        n_pts, n_tones = sf.shape
        f_all = sf.ravel()
        z_all = (si + 1j * sq).ravel()
        # Sort by frequency
        sort_idx = np.argsort(f_all)
        f_all = f_all[sort_idx]
        z_all = z_all[sort_idx]

    # Find resonances if not provided
    if resonances is None:
        resonances = find_mkid_resonances(
            f_all, z_all,
            data_format=data_format,
            filter_params=filter_params,
            finder_params=finder_params,
        )

    # Convert frequency list to resonance list if needed
    if resonances and isinstance(resonances[0], (int, float, np.floating)):
        resonances = [type('R', (), {'frequency': float(fr), 'fwhm': None})()
                      for fr in resonances]

    fit_results = []
    for i, res in enumerate(resonances):
        fr_guess = res.frequency
        # Estimate window from FWHM or default
        fwhm = getattr(res, 'fwhm', None)
        if fwhm is None or fwhm <= 0:
            fwhm = fr_guess / 1000  # Default: Q~1000

        half_window = window_fwhm * fwhm / 2
        mask = (f_all >= fr_guess - half_window) & (f_all <= fr_guess + half_window)
        if np.sum(mask) < 10:
            # Expand window if too few points
            half_window *= 3
            mask = (f_all >= fr_guess - half_window) & (f_all <= fr_guess + half_window)

        f_win = f_all[mask]
        z_win = z_all[mask]

        if len(f_win) < 5:
            if verbose:
                print(f"  Resonance {i}: {fr_guess/1e6:.3f} MHz — "
                      f"skipped (only {len(f_win)} points)")
            continue

        if verbose:
            print(f"  Fitting resonance {i+1}/{len(resonances)}: "
                  f"{fr_guess/1e6:.3f} MHz ({len(f_win)} points)...")

        if nonlinear:
            fit = fit_resonance_nonlinear(f_win, z_win, fr_guess=fr_guess,
                                         sweep_direction=sweep_direction)
        else:
            fit = fit_resonance(f_win, z_win, fr_guess=fr_guess)
        fit_results.append(fit)

        if verbose:
            status = 'OK' if fit.success else 'FAILED'
            anl_str = f', anl={fit.anl:.2e}' if nonlinear else ''
            print(f"    {status}: fr={fit.fr/1e6:.4f} MHz, "
                  f"Ql={fit.Ql:.0f}, Qi={fit.Qi:.0f}, "
                  f"Qc={fit.Qc_abs:.0f}{anl_str}, "
                  f"rms={fit.residual_rms:.2e}")

    return fit_results


def extract_parameters(fit_results):
    """
    Extract fitted parameters from a list of FitResult objects into arrays.

    Args:
        fit_results: List of FitResult objects from batch_fit().

    Returns:
        dict with keys:
            'fr': array of resonance frequencies (Hz)
            'Ql': array of loaded Q factors
            'Qi': array of internal Q factors
            'Qc_abs': array of coupling Q magnitudes
            'phi': array of impedance mismatch angles (rad)
            'tau': array of cable delays (s)
            'residual_rms': array of fit residual RMS values
            'success': array of booleans
    """
    if not fit_results:
        return {k: np.array([]) for k in
                ['fr', 'Ql', 'Qi', 'Qc_abs', 'phi', 'tau', 'anl',
                 'residual_rms', 'success']}

    return {
        'fr': np.array([r.fr for r in fit_results]),
        'Ql': np.array([r.Ql for r in fit_results]),
        'Qi': np.array([r.Qi for r in fit_results]),
        'Qc_abs': np.array([r.Qc_abs for r in fit_results]),
        'phi': np.array([r.phi for r in fit_results]),
        'tau': np.array([r.tau for r in fit_results]),
        'anl': np.array([r.anl for r in fit_results]),
        'residual_rms': np.array([r.residual_rms for r in fit_results]),
        'success': np.array([r.success for r in fit_results]),
    }
