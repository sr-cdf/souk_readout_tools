"""
Resonator fitting for MKID S21 data.

Provides nonlinear least-squares fitting of resonator models to
complex S21 sweep data, with batch fitting over multiple tones.

Models:
- Notch-type (Khalil 2012): S21 = a * exp(j*alpha) * exp(-2*pi*j*f*tau) *
  (1 - (Ql/|Qc|) * exp(j*phi) / (1 + 2*j*Ql*(f-fr)/fr))

Fitting workflow:
1. find_resonances() to locate resonance frequencies
2. fit_resonance() to fit a single resonance
3. batch_fit() to fit all resonances in a sweep

References:
    Khalil et al., J. Appl. Phys. 111, 054510 (2012)
    Probst et al., Rev. Sci. Instrum. 86, 024706 (2015)
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

    # Fit quality
    residual_rms: float            # RMS of fit residuals
    success: bool                  # Whether the fit converged
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


def _residuals(params, f, z_data):
    """Residual vector for least-squares fitting (real, imag interleaved)."""
    fr, Ql, Qc_abs, phi, a, alpha, tau = params
    z_model = _s21_notch(f, fr, Ql, Qc_abs, phi, a, alpha, tau)
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

    z_fit = _s21_notch(f, fr, Ql, Qc_abs, phi, a, alpha, tau)
    residual_rms = float(np.sqrt(np.mean(np.abs(z_fit - z) ** 2)))

    return FitResult(
        fr=fr, Ql=Ql, Qc_abs=Qc_abs, phi=phi,
        a=a, alpha=alpha, tau=tau,
        Qi=Qi, Qc=Qc,
        residual_rms=residual_rms,
        success=success, message=message,
        f_data=f, z_data=z, z_fit=z_fit,
    )


def batch_fit(sweep_data, resonances=None, data_format='log_magnitude',
              filter_params=None, finder_params=None,
              window_fwhm=10.0, verbose=True):
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

        fit = fit_resonance(f_win, z_win, fr_guess=fr_guess)
        fit_results.append(fit)

        if verbose:
            status = 'OK' if fit.success else 'FAILED'
            print(f"    {status}: fr={fit.fr/1e6:.4f} MHz, "
                  f"Ql={fit.Ql:.0f}, Qi={fit.Qi:.0f}, "
                  f"Qc={fit.Qc_abs:.0f}, rms={fit.residual_rms:.2e}")

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
                ['fr', 'Ql', 'Qi', 'Qc_abs', 'phi', 'tau',
                 'residual_rms', 'success']}

    return {
        'fr': np.array([r.fr for r in fit_results]),
        'Ql': np.array([r.Ql for r in fit_results]),
        'Qi': np.array([r.Qi for r in fit_results]),
        'Qc_abs': np.array([r.Qc_abs for r in fit_results]),
        'phi': np.array([r.phi for r in fit_results]),
        'tau': np.array([r.tau for r in fit_results]),
        'residual_rms': np.array([r.residual_rms for r in fit_results]),
        'success': np.array([r.success for r in fit_results]),
    }
