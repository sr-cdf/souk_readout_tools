"""
======================================================================================
RESONATOR FITTING PACKAGE
======================================================================================

A tool for fitting microwave/RF resonator transmission (S21) data,
including nonlinear (Duffing) resonator response with polynomial baseline correction.


FEATURES:
---------
- Linear and nonlinear (Duffing oscillator) resonator models
- Automatic data preconditioning for improved convergence
- Multi-round fitting with progressive fallback strategies
- Interactive visualization with keyboard controls
- Batch processing with summary statistics and plots
- Robust error handling and parameter bounds


PHYSICAL MODEL:
---------------
S21(f) = Baseline(f) * Resonator(f)

Where:
- Baseline(f) = 10^(mag_polynomial/20) * exp(i*phase_polynomial)
- Resonator response includes:
  * f0: resonance frequency
  * Qi: internal quality factor (energy loss)
  * Qc: coupling quality factor (port coupling strength)
  * phi: impedance mismatch angle (phase of complex Qe)
  * anl: nonlinearity parameter (Duffing coefficient)

  
DEPENDENCIES:
-------------
- numpy
- matplotlib
- lmfit


QUICK START:
------------

```python
import numpy as np
from resonator_fitter import fit_resonator, detailed_fit_plot

# Load your data (frequency in Hz, S21 as complex)
freq = np.load('frequency.npy')  # 1D array
s21_data = np.load('s21.npy')    # complex 1D array
errors = np.load('errors.npy')   # optional complex error estimates

# Fit single resonator
result, fit_curve, baseline, guess, guess_baseline, transform = fit_resonator(
    freq, s21_data, 
    errors=errors,           # optional
    mag_order=1,             # baseline polynomial order
    phase_order=1,
    sweep_direction='up',    # 'up' or 'down' for nonlinear
    use_preconditioning=True,
    max_nfev=500,
    verbose=True
)

# Plot results
fig = detailed_fit_plot(
    freq, s21_data, result, fit_curve, baseline, 
    guess, guess_baseline, transform=transform
)
fig.savefig('fit_result.png', dpi=300)

# Extract fitted parameters
f0 = result.params['f0'].value
Qi = result.params['Qi'].value
Qc = result.params['Qc'].value
print(f"f0={f0/1e9:.6f} GHz, Qi={Qi:.0f}, Qc={Qc:.0f}")


BATCH PROCESSING:

from resonator_fitter import interactive_fit_viewer, fit_summary_table, fit_summary_plot

# Load multiple resonator sweeps
f = [...]  # list of frequency arrays
z = [...]  # list of S21 arrays
e = [...]  # optional list of error arrays

# Or using souk_readout_tools.client:

sweeps = client.client.parse_sweep_data(client.get_sweep_data())
f,z,e = s['sweep_f'], s['sweep_i']+1j*s['sweep_q'], s['sweep_ei']+1j*s['sweep_eq']

fit_results = [fit_resonator(f[:,i],z[:,i],e[:,i],
                             sweep_direction='up',
                             mag_order=1,phase_order=1,
                             use_preconditioning=1,verbose=True) for i in range(len(f[0]))]


# Interactive viewer (fits on-demand)
fit_results = interactive_fit_viewer(f.T, z.T, e.T, fit_results=fit_results,show_preconditioned=1)
# Use arrow keys or A/D to navigate, F to refit, P to toggle preconditioning

# Generate summary table
table_str, summaries, filename = fit_summary_table(
    fit_results, 
    outfilename='fit_summary.txt'
)
print(table_str)

# Generate summary plots
fig, axes, plotfile = fit_summary_plot(
    fit_results,
    outfilename='fit_summary.png'
)

# Histogram analysis
fig_hist, axes_hist, histfile = fit_summary_histograms(
    fit_results,
    xscale_for={'Qi':'log', 'Qc':'log', 'Nonlinearity':'log'},
    outfilename='fit_histograms.png'
)


ADVANCED USAGE:
# Custom parameter constraints
param_controls = {
    'Qi': {'value': 1e6, 'min': 1e4, 'max': 1e7},
    'anl': {'value': 0, 'vary': False},  # fix nonlinearity at zero
    'phi': {'value': 0, 'vary': False}   # fix impedance mismatch
}

result, fit, baseline, guess, guess_bl, transform = fit_resonator(
    freq, s21_data,
    param_controls=param_controls
)

# Access covariance matrix
if result.covar is not None:
    print("Parameter correlations available")
    # result.covar contains covariance matrix


FITTING STRATEGY:
The fitter uses multiple rounds with progressive fallback:

Round 1: 'leastsq' (fast, Levenberg-Marquardt)
Round 2: 'least_squares' (more robust, trust region)
Round 3: Edge-downweighted fit (focus on resonance)
Round 4: Edge-truncated fit (avoid nearby features)
Round 5: Heavily truncated fit (extreme cases)

Each round only runs if previous rounds fail or have poor chi-squared.


INTERACTIVE VIEWER CONTROLS:
→ or D : Next resonator
← or A : Previous resonator
F      : Refit current resonator
P      : Toggle raw/preconditioned data view
G      : Toggle grid
Q/Esc  : Close viewer


TROUBLESHOOTING:

High chi-squared: Check if baseline polynomial order is sufficient
Failed fits: Try max_rounds=5 for more fallback attempts
Nearby resonators: Fitting may struggle with overlapping features
Deep nonlinearity: Ensure correct sweep_direction ('up' vs 'down')
Poor initial guess: Use param_controls to constrain parameters

    
TODO LIST:

1. Update interactive viewer to enable flagging and save flags
2. Automate procedure for fitting multiple attenutations, temperatures, optical loads.
3. Improve fallback strategies for very deep dips.


OTHER NOTES:

TBD.


"""


import numpy as np

from lmfit import Parameters, Minimizer, report_fit

import matplotlib.pyplot as plt
from matplotlib import cm, colors
from matplotlib.collections import LineCollection
from matplotlib.cm import ScalarMappable
import time
import os
import json

from datetime import datetime

# ============================================================================
# CORE RESONATOR MODEL
# ============================================================================

def resonator_s21_model(f, f0, Qi, Qc, phi, anl, mag_coef=None, phase_coef=None,
                        sweep_direction='up'):
    """
    General nonlinear resonator transmission model with polynomial baseline.

    Parameters:
    -----------
    f : array
        Frequency points
    f0 : float
        Resonance frequency
    Qi : float
        Internal quality factor
    Qc : float
        Coupling quality factor (real part)
    phi : float
        Impedance mismatch angle (radians)
    anl : float, optional
        Nonlinearity parameter (0 = linear regime)
    mag_coef : array, optional
        Polynomial coefficients for magnitude baseline (dB), highest order first
    phase_coef : array, optional
        Polynomial coefficients for phase baseline (rad), highest order first
    sweep_direction : str
        'up' or 'down' for frequency sweep direction, affects the nonlinear solution

    Returns:
    --------
    S21 : complex array
        Complex transmission coefficient
    """
    f = np.asarray(f)

    # Normalized frequency for baseline polynomials
    fc = 0.5 * (f[0] + f[-1])
    u = (f - fc) / fc

    # Baseline construction
    if mag_coef is None:
        mag_coef = np.array([0.0])
    if phase_coef is None:
        phase_coef = np.array([0.0])

    mag_db = np.polyval(mag_coef, u)
    phase_rad = np.polyval(phase_coef, u)
    baseline = 10.0**(mag_db/20.0) * np.exp(1j * phase_rad)

    # Complex external Q (includes impedance mismatch and stray inductance)
    eps = 1e-6 
    if not (-0.5*np.pi < phi < 0.5*np.pi):
        phi = np.clip(phi, -0.5*np.pi + eps, 0.5*np.pi - eps)
    Qe = Qc * (1.0 + 1j*np.tan(phi))
    
    # Loaded Q - not used - better results if we just use Qi and Qe directly
    # Qr = 1.0 / (1.0/Qi + 1.0/np.real(Qe)) # Qr = 1/(1/Qi + 1/real(Qe))

    # Resonator response
    if anl < 1e-12:
        #NOTE: tried increasing this threshold to reduce the computational load, but the minimizer doesnt like it

        # Linear regime - fast path, difference betewen 0 and 1e-6 is <0.01dB for Qi>>Qc
        x = (f - f0) / f0
        # resonator = 1.0 - (Qr / Qe) / (1.0 + 2j * Qr * x)
        resonator = 1.0 - (1.0/Qe) / (1.0/Qi + 1.0/Qe + 2j * x)
    else:
        # Nonlinear (Duffing) regime
        x0 = (f - f0) / f0
        y0 = x0 / (1.0/Qi + 1.0/np.real(Qe))
        y = _solve_duffing(y0, anl, sweep_direction)
        # resonator = 1.0 - (Qr / Qe) / (1.0 + 2j * y)
        resonator = 1.0 - (1/Qe) / ((1.0/Qi + np.real(1/Qe)) * (1 + 2j * y))
    return baseline * resonator


def _solve_duffing(y0, anl, sweep_direction='up'):
    """
    Solve 4y^3 - 4*y0*y^2 + y - y0 - anl = 0, vectorized.

    This cubic equation arises from the Duffing oscillator model of a nonlinear resonator.

    The roots of this cubic give the nonlinear frequency shift y and 
    the choice of root determines which of the bistable branches is 
    followed, based on the sweep direction.
    """
    y0, anl = np.broadcast_arrays(np.asarray(y0, float), np.asarray(anl, float))

    A = 4.0
    B = -4.0*y0
    C = 1.0
    D = -y0 - anl

    Delta0 = B*B - 3.0*A*C
    Delta1 = 2.0*B*B*B - 9.0*A*B*C + 27.0*(A*A)*D

    # complex-safe sqrt for discriminant
    disc = np.emath.sqrt(Delta1*Delta1 - 4.0*Delta0*Delta0*Delta0)
    Ct = 0.5*(Delta1 + disc)

    # avoid division by zero
    Ct = np.where((Ct.real*Ct.real + Ct.imag*Ct.imag) < 1e-48, 1e-24 + 0j, Ct)

    # complex cube root
    Cc = Ct ** (1.0/3.0)

    # cube roots of unity
    xi  = -0.5 + 0.5j*np.sqrt(3.0)
    xi2 = xi*xi

    # three roots
    inv3A = 1.0/(3.0*A)
    r1 = -(B + Cc            +  (Delta0/Cc)        )*inv3A
    r2 = -(B + xi*Cc         +  (Delta0/(xi*Cc))   )*inv3A
    r3 = -(B + xi2*Cc        +  (Delta0/(xi2*Cc))  )*inv3A

    # choose the real roots based on sweep direction
    if sweep_direction =='up':
        y = np.where(np.isclose(r1.imag, 0.0, atol=1e-12), r1.real, np.maximum(r2.real, r3.real))
    elif sweep_direction=='down':
        y = np.where(np.isclose(r2.imag, 0.0, atol=1e-12) | np.isclose(r3.imag,0) , np.maximum(r2.real, r3.real), r1.real)
    else:
        raise ValueError('Direction must be "up" or "down".')
    return y





# ===========================================================================================
# PRECONDITIONING - very roughly estimate and remove linear phase and magnitude offset
# ===========================================================================================

def precondition_data(f, z, transform=None):
    """
    Remove the magnitude offset (c) and linear phase (a*f + b).
    If transform is None, estimate it from the data.
    Returns z_pc = z / c * exp(-j*(a*f + b)) and dict of transform params
    """
    if transform == None:
        transform = get_preconditioning_transform_params(f, z)
    f = np.asarray(f)
    c, a, b = transform['c'],transform['a'], transform['b']
    z_pc = z/c * np.exp(-1j*(a*f + b))
    return z_pc, transform
    # return z, transform


def get_preconditioning_transform_params(f, z):
    """
    Estimate preconditioning parameters: mag_db offset (c) and linear phase (a*f + b).
    Returns dict with {'c':..., 'a':..., 'b':..., 'f':...}
    """
    f = np.asarray(f)
    z = np.asarray(z)

    mag = np.abs(z)
    # magc = mag[0]/2.0+mag[-1]/2.0 
    magc = np.median(mag) + 1e-12 # avoid div by zero 
    magc = np.max(mag) + 1e-12 # avoid div by zero 

    phi = np.unwrap(np.angle(z))
    pa = (phi[-1] - phi[0]) / (f[-1] - f[0] + 1e-24) # slope
    pb = phi[0] - pa*f[0] # offset

    return {'c':magc,'a':pa,'b':pb,'f':f}

def undo_preconditioning(f, z_pc, transform):
    """
    Undo the preconditioning applied to the data with the given transform.
    """
    c, a, b = transform['c'],transform['a'], transform['b']
    return (z_pc * c) * np.exp(+1j*(a*f + b))
    # return z_pc

def transform_fit_params_from_pc(params, transform):
    """
    Transform the fitted baseline coefficients in a fit parameters object
    to revert to raw coordinates from preconditioned coordinates.

    """
    
    f = np.asarray(transform['f'])
    fc = 0.5 * (f[0] + f[-1])

    c, a, b = transform['c'], transform['a'], transform['b']
    if 'm0' in params:
        params['m0'].value += 20*np.log10(c)
    if 'p0' in params:
        params['p0'].value += a*fc +b
    if 'p1' in params:
        params['p1'].value += a*fc
    
    return params

def transform_fit_values_from_pc(values_dict, transform):
    """
    Transform the fitted baseline coefficients in a fit values dict
    to revert to raw coordinates from preconditioned coordinates.
    """
    f = np.asarray(transform['f'])
    fc = 0.5 * (f[0] + f[-1])

    c, a, b = transform['c'], transform['a'], transform['b']
    if 'm0' in values_dict:
        values_dict['m0'] += 20*np.log10(c)
    if 'p0' in values_dict:
        values_dict['p0'] += a*fc + b
    if 'p1' in values_dict:
        values_dict['p1'] += a*fc

    return values_dict


def transform_fit_params_to_pc(params, transform):
    """
    Transform the fitted baseline coefficients in a fit parameters object
    to convert from raw coordinates to preconditioned coordinates.
    """
    f = np.asarray(transform['f'])
    fc = 0.5 * (f[0] + f[-1])

    c, a, b = transform['c'], transform['a'], transform['b']
    if 'm0' in params:
        params['m0'].value -= 20*np.log10(c)
    if 'p0' in params:
        params['p0'].value -= a*fc + b 
    if 'p1' in params:
        params['p1'].value -= a*fc

    return params



# ============================================================================
# PARAMETER SETUP AND FITTING
# ============================================================================

def setup_parameters(f, z, mag_order=1, phase_order=1,
                     param_controls=None):
    """
    Initialize fit parameters with optional user-specified constraints.

    Parameters:
    -----------
    f : array
        Frequency points
    z : complex array
        Complex S21 data
    mag_order : int
        Polynomial order for magnitude baseline
    phase_order : int
        Polynomial order for phase baseline
    param_controls : dict, optional
        Override parameter settings. Format:
        {'param_name': {'value': float, 'min': float, 'max': float, 'vary': bool}}
        param names: 'f0', 'Qi', 'Qc', 'phi', 'anl', 'm0', 'm1', ..., 'p0', 'p1', ...
    Returns:
    --------
    params : lmfit.Parameters
        Initial parameter set
    """
    f = np.asarray(f)
    z = np.asarray(z)
    fc = 0.5 * (f[0] + f[-1])
    u  = (f - fc) / fc
    mag_db = 20 * np.log10(np.abs(z) + 1e-18)
    phase  = np.unwrap(np.angle(z))

    # Default bounds
    eps = 1e-6
    defaults = {
        'f0': {'min': np.min(f), 'max': np.max(f)},
        'Qi': {'value': 1e5, 'min': 1e3, 'max': 1e7},
        'Qc': {'value': 1e5, 'min': 1e3, 'max': 1e7},
        'phi': {'value': 0.0, 'min': -0.5*np.pi + eps, 'max': 0.5*np.pi - eps},
        'anl': {'value': 0.00001, 'min': 0.0, 'max': 100},
    }

    # Initial parameter estimates for baseline and resonator

    # Always precondition the data internally for robust baseline weight calculation
    z_pc, _   = precondition_data(f, z)
    mag_db_pc = 20 * np.log10(np.abs(z_pc) + 1e-18)
    phase_pc  = np.unwrap(np.angle(z_pc))
    grad_pc = np.gradient(phase_pc, f)

    # Robust polynomial weights for baseline estimation
    mag_weights = _robust_weights(mag_db_pc)
    phase_weights = _robust_weights(phase_pc)
    grad_weights = _robust_weights(grad_pc)
    phase_weights *= grad_weights  # downweight steep phase regions

    # Polynomial fits for baseline with covariance, only using preconditioned data for weights
    mag_coef, mag_cov = np.polyfit(u, mag_db, mag_order, w=mag_weights, cov=True)
    phase_coef, phase_cov = np.polyfit(u, phase, phase_order, w=phase_weights, cov=True)
    
    # Remove baseline to estimate resonator parameters
    baseline = _compute_baseline(f,mag_coef,phase_coef)
    z_normalized = z / baseline

    # Find resonance
    i0 = np.argmin(np.abs(z_normalized))
    f0_est = f[i0]
    
    # Estimate Qr from phase slope near resonance, not good for unresolved or overdriven resonances
    # i1, i2 = max(i0-3, 0), min(i0+4, len(f))
    # phase_norm = np.unwrap(np.angle(z_normalized))
    # slope = np.polyfit(f[i1:i2], phase_norm[i1:i2], 1)[0]
    # Qr_est = max(1e3, 0.5 * np.abs(defaults['f0']['value'] * slope))
    
    # Estimate Qr from FWHM
    magn       = np.abs(z_normalized)
    magnsq     = magn**2
    half_power = 0.5*(magnsq.max() + magnsq.min())
    ihp        = np.abs(magnsq - half_power).argmin()
    df         = np.abs(np.diff(f))
    dfmin      = np.min(df[df>0])
    fwhm       = max(dfmin,2*np.abs( f[i0] - f[ihp] ))
    Qr_est     = f[i0]/fwhm

    # Estimate Qe and Qi
    Qe_est   = Qr_est / (1 - z_normalized[i0])
    Qc_est   = Qe_est.real
    phi_est  = 1e-3 #np.angle(Qe_est), high phi overpowers qi and anl - non zero phi makes can make very deep dips
    Qi_est   = 1.0 / (1.0/Qr_est -1/Qc_est)
    dipdepth = np.abs(1 - np.sqrt(Qi_est**2 + Qe_est.imag**2) / (Qc_est + Qi_est))
    print(20*np.log10(abs(dipdepth)))

    # Set anl default to be close to zero, but not exactly zero as we may need to use the nonlinear model
    anl_est = 0.00001

    print(fwhm,Qr_est,Qc_est,Qi_est,phi_est,20*np.log10(np.abs(dipdepth)))
    
    # Update defaults with estimates
    defaults['f0']['value'] = float(np.clip(f0_est, defaults['f0']['min'], defaults['f0']['max']))
    defaults['Qi']['value'] = float(np.clip(Qi_est, defaults['Qi']['min'], defaults['Qi']['max']))
    defaults['Qc']['value'] = float(np.clip(Qc_est, defaults['Qc']['min'], defaults['Qc']['max']))
    defaults['anl']['value'] = float(np.clip(anl_est, defaults['anl']['min'], defaults['anl']['max']))
    defaults['phi']['value'] = float(np.clip(phi_est, defaults['phi']['min'], defaults['phi']['max']))

    # Build parameter set
    params = Parameters()

    # Include controls from user if provided and update parameter set
    for pname in ['f0', 'Qi', 'Qc', 'phi', 'anl']:
        pdict = defaults[pname].copy()
        if param_controls and pname in param_controls:
            pdict.update(param_controls[pname])
        params.add(pname, **pdict)

    # Include baseline coefficients with user controls, no bounds for now, 
    for k in range(mag_order + 1):
        pname = f'm{k}'
        pdict = {'value': float(mag_coef[mag_order - k])}
        if param_controls and pname in param_controls:
            pdict.update(param_controls[pname])
        params.add(pname, **pdict)
    
    for k in range(phase_order + 1):
        pname = f'p{k}'
        pdict = {'value': float(phase_coef[phase_order - k])}
        if param_controls and pname in param_controls:
            pdict.update(param_controls[pname])
        params.add(pname, **pdict)

    return params


def _robust_weights(y):
    """
    Compute robust weights using Tukey's biweight.
    Effectively downweights any outliers and the on-resonance region in baseline fitting.
    """
    med = np.median(y)
    mad = np.median(np.abs(y - med)) + 1e-12
    u = (y - med) / (4.685 * mad)
    return np.where(np.abs(u) < 1, (1 - u*u)**2, 0.0)

def _compute_baseline(f,mag_coef,phase_coef):
    """Compute baseline from polynomial coefficients."""
    fc = 0.5 * (f[0] + f[-1])
    u = (f - fc) / fc
    mag_db = np.polyval(mag_coef, u)
    phase_rad = np.polyval(phase_coef, u)
    return 10.0**(mag_db/20.0) * np.exp(1j * phase_rad)



def fit_resonator(f, z, errors=None, 
                  mag_order=1, phase_order=1,
                  sweep_direction='up', 
                  param_controls=None,
                  use_preconditioning=True,
                  max_nfev=500, verbose=False,
                  max_rounds=5,
                  fit_kwargs={}):
    """
    Fit resonator transmission data with automatic preconditioning.

    Parameters:
    -----------
    f : array
        Frequency points
    z : complex array
        Complex S21 data
    errors : complex array, optional
        Standard deviations (real and imag independent)
    mag_order : int
        Polynomial order for magnitude baseline
    phase_order : int
        Polynomial order for phase baseline
    sweep_direction : str
        'up' or 'down'
    param_controls : dict, optional
        Parameter constraints dict of {'param_name': {'value': float, 'min': float, 'max': float, 'vary': bool}}
    use_preconditioning : bool
        Apply phase slope removal and rotation before fitting
    max_nfev : int
        Maximum function evaluations
    verbose : bool
        Print fit report
    fit_kwargs : dict
        Additional keyword arguments to pass to the fitter

    Returns:
    --------
    result : lmfit.MinimizerResult
        Fit results
    fit_curve : complex array
        Fitted S21 values
    baseline : complex array
        Fitted baseline
    guess_curve : complex array
        Initial guess for the fit
    guess_baseline : complex array
        Initial guess for the baseline
    transform : dict or None
        Preconditioning transform parameters, or None if not used
    """
    f = np.asarray(f)
    z = np.asarray(z)
    if verbose:
        print("\nfit_resonator: Starting\n")    

    # Preconditioning
    if use_preconditioning:
        z_fit, transform = precondition_data(f, z)
    else:
        z_fit, transform = z, {'c':1,'a':0,'b':0,'f':f} # identity transform
    if verbose:
        print(f'Preconditioning transform: (c,b,a) = ({transform["c"]},{transform["b"]},{transform["a"]})')

    # Setup parameters
    params = setup_parameters(f, z_fit, mag_order, phase_order, param_controls)


    # Initial non-zero values when preconditioning parameters to help convergence
    if use_preconditioning:
        params['m0'].value=1e-6
        params['m1'].value=1e-6
        params['p0'].value=1e-6
        params['p1'].value=1e-6
    params_init = params.copy()
    if verbose:
        print('Initial parameters:')
        params.pretty_print()


    # Prepare weights from measurement errors (robust to zeros/None/scalars/real/complex)
    min_sigma = 1e-12
    have_errors = not((errors is None) or (np.isscalar(errors) and errors==0) or (np.std(np.asarray(errors))==0))
    if errors is None:
        w_real = np.ones_like(f, float)
        w_imag = np.ones_like(f, float)
    else:
        errarr = np.asarray(errors) # do not modify in place as this persists between calls
        err = errarr / transform['c']  # scale errors same as data - rotating seem to worsen the residuals
        if np.iscomplexobj(err):
            w_real = 1.0 / np.maximum(np.abs(err.real), min_sigma)
            w_imag = 1.0 / np.maximum(np.abs(err.imag), min_sigma)
        else:
            s = 1.0 / np.maximum(np.asarray(err, float), min_sigma)
            w_real = s; w_imag = s    
    if verbose:
        print(f'Weight stats: real: min={w_real.min()}, max={w_real.max()}, mean={w_real.mean()}')
        print(f'Weight stats: imag: min={w_imag.min()}, max={w_imag.max()}, mean={w_imag.mean()}')


    # Define residual function to be minimised
    def residual(pars):
        mag_coef = np.array([pars[f'm{k}'].value for k in range(mag_order, -1, -1)])
        phase_coef = np.array([pars[f'p{k}'].value for k in range(phase_order, -1, -1)])

        model = resonator_s21_model(
            f, pars['f0'].value, pars['Qi'].value, pars['Qc'].value,
            pars['phi'].value, pars['anl'].value, mag_coef, phase_coef, sweep_direction
        )
        res = model - z_fit
        #real and imaginary parts weighted independently based on measurement errors
        return np.concatenate([np.real(res)*w_real, np.imag(res)*w_imag])


    # Do the fit - try 'leastsq' first, then 'least_squares' if that struggles.
    # If still struggling, try downweighting the edges of the sweep in case
    # nearby resonators are affecting the baseline fit too much.


    #Round 1: 'leastsq'
    minimizer = Minimizer(residual, params, nan_policy='propagate')
    result = minimizer.minimize(method='leastsq', max_nfev=max_nfev,**fit_kwargs)
    result.fit_resonator_round = 1
    if verbose:
        print('Round1 Report:')
        print(report_fit(result))
        print(f'Round 1 "leastsq": success={result.success} message={result.message}, redchi={result.redchi}')
    

    # Round 2: 'least_squares'
    if (max_rounds>=2) and (not result.success or (have_errors and result.redchi>10 )):
        if verbose: 
            print('Trying Round 2 with "least_squares"')
        result = Minimizer(residual,params_init.copy(),nan_policy='omit').minimize(method='least_squares', max_nfev=max_nfev,**fit_kwargs)
        result.fit_resonator_round = 2
        if verbose:
            print('Round2 Report:')
            print(report_fit(result))
            print(f'Round 2 "least_squares": success={result.success} message={result.message}, redchi={result.redchi}   ')

    # Round 3: 'least_squares' with edges downweighted
    if (max_rounds>=3) and (not result.success or (have_errors and result.redchi>20 )):
        if verbose:
            print('Trying Round 3 with down-weighted edges to focus on resonance')
        
        guess = resonator_s21_model(
                f, params_init['f0'].value, params_init['Qi'].value, params_init['Qc'].value,
                params_init['phi'].value, params_init['anl'].value, 
                np.array([params_init[f'm{k}'].value for k in range(mag_order, -1, -1)]),
                np.array([params_init[f'p{k}'].value for k in range(phase_order, -1, -1)]),
                sweep_direction)

        w=(1-(np.abs(guess)/np.max(np.abs(guess)))**2) # downweight edges, upweight center
        s=np.mean(w)
        def residual(pars):
            mag_coef = np.array([pars[f'm{k}'].value for k in range(mag_order, -1, -1)])
            phase_coef = np.array([pars[f'p{k}'].value for k in range(phase_order, -1, -1)])

            model = resonator_s21_model(
                f, pars['f0'].value, pars['Qi'].value, pars['Qc'].value,
                pars['phi'].value, pars['anl'].value, mag_coef, phase_coef, sweep_direction
            )

            res = model - z_fit
            return np.concatenate([np.real(res)*w_real*w, np.imag(res)*w_imag*w])/s
        params = params_init.copy() 
        result = Minimizer(residual,params,nan_policy='omit').minimize(method='least_squares', max_nfev=max_nfev,**fit_kwargs)
        result.fit_resonator_round = 3
        if verbose:
            print('Round3 Report:')
            print(report_fit(result))
            print(f'Round 3 "least_squares" with downweighted edges: success={result.success} message={result.message}, redchi={result.redchi}   ')


    # Round 4: 'least_squares' with edges truncated        
    if (max_rounds>=4) and (not result.success or (have_errors and result.redchi>20 )):
        if verbose:
            print('Trying Round 4 with truncated edges to avoid nearby features')
        mask = np.zeros_like(f)
        i0 = np.argmin(abs(z_fit)) #center
        ni = max(10,len(f)//(4)) # at most cut 25% off each edge leaving 50% in center
        mask[max(0,i0-ni):min(len(f),i0+ni)] = 1
        s=np.mean(mask)
        def residual(pars):
            mag_coef = np.array([pars[f'm{k}'].value for k in range(mag_order, -1, -1)])
            phase_coef = np.array([pars[f'p{k}'].value for k in range(phase_order, -1, -1)])

            model = resonator_s21_model(
                f, pars['f0'].value, pars['Qi'].value, pars['Qc'].value,
                pars['phi'].value, pars['anl'].value, mag_coef, phase_coef, sweep_direction
            )

            res = model - z_fit
            return np.concatenate([np.real(res)*w_real*mask, np.imag(res)*w_imag*mask])/s
        params = params_init.copy()
        params['m0'].vary = False
        params['m1'].vary = False
        result = Minimizer(residual,params.copy(),nan_policy='omit').minimize(method='least_squares', max_nfev=max_nfev,**fit_kwargs)
        result.fit_resonator_round = 4
        if verbose:
            print('Round 4 Report:')
            print(report_fit(result))
            print(f'Round 4 "least_squares" with truncated edges: success={result.success} message={result.message}, redchi={result.redchi}   ')

    # Round 5: 'least_squares' with edges truncated even more
    if (max_rounds>=5) and (not result.success or (have_errors and result.redchi>20 )):
        if verbose:
            print('Trying Round 5 with truncated edges to avoid nearby features')
        mask = np.zeros_like(f)
        i0 = np.argmin(abs(z_fit)) #center
        ni = max(10, int(len(f)//(3))) # at most cut 33% off each edge leaving 33% in center
        mask[max(0,i0-ni):min(len(f),i0+ni)] = 1
        s=np.mean(mask)
        def residual(pars):
            mag_coef = np.array([pars[f'm{k}'].value for k in range(mag_order, -1, -1)])
            phase_coef = np.array([pars[f'p{k}'].value for k in range(phase_order, -1, -1)])

            model = resonator_s21_model(
                f, pars['f0'].value, pars['Qi'].value, pars['Qc'].value,
                pars['phi'].value, pars['anl'].value, mag_coef, phase_coef, sweep_direction
            )

            res = model - z_fit
            return np.concatenate([np.real(res)*w_real*mask, np.imag(res)*w_imag*mask])/s
        params = params_init.copy()
        params['m0'].vary = False
        params['m1'].vary = False
        result = Minimizer(residual,params.copy(),nan_policy='omit').minimize(method='least_squares', max_nfev=max_nfev,**fit_kwargs)
        result.fit_resonator_round = 5
        if verbose:
            print('Round 5 Report:')
            print(report_fit(result))
            print(f'Round 5 "least_squares" with downweighted edges: success={result.success} message={result.message}, redchi={result.redchi}   ')

    # Round 6: Not implemented, need a more advanced physical model
    if (max_rounds>=6) and (not result.success or (have_errors and result.redchi>1000 )):
            print('Need to develop a Round 6 (not implemented yet)')
            print('Result from Round 5 returned')
            # result = Minimizer(new_residual_function,params_init.copy(),nan_policy='omit').minimize(method='least_squares', max_nfev=max_nfev,**fit_kwargs)
            # if verbose:
            #     print('Round 6 Report:')
            #     print(report_fit(result))
    

    # Re-include any missing parameters that were set fixed previously
    init_params = result.init_values.copy()
    for p in ['p1','p0','m1','m0']:
        if p not in init_params:
            init_params[p]=params[p].value
    
    # Reverse preconditioning, if applied, to get back to original coordinates for plotting later
    if use_preconditioning:
        result.params = transform_fit_params_from_pc(result.params.copy(), transform)
        init_params = transform_fit_values_from_pc(init_params, transform)
    

    # Generate fit curves in original coordinates
    mag_coef = np.array([result.params[f'm{k}'].value for k in range(mag_order, -1, -1)])
    phase_coef = np.array([result.params[f'p{k}'].value for k in range(phase_order, -1, -1)])

    fit_curve = resonator_s21_model(
        f, result.params['f0'].value, result.params['Qi'].value,
        result.params['Qc'].value, result.params['phi'].value,
        result.params['anl'].value, mag_coef, phase_coef, sweep_direction
    )

    baseline = _compute_baseline(f,mag_coef,phase_coef)

    # Generate guess curves in original coordinates
    guess_mag_coef = np.array([init_params[f'm{k}'] for k in range(mag_order, -1, -1)])
    guess_phase_coef = np.array([init_params[f'p{k}'] for k in range(phase_order, -1, -1)])

    guess_curve = resonator_s21_model(
        f, init_params['f0'], init_params['Qi'],
        init_params['Qc'], init_params['phi'],
        init_params['anl'], guess_mag_coef, guess_phase_coef, sweep_direction
    )
    guess_baseline = _compute_baseline(f,guess_mag_coef,guess_phase_coef)

#     if verbose:
#         report_fit(result)
#

    return result, fit_curve, baseline, guess_curve, guess_baseline, transform




# ============================================================================
# INTERACTIVE VISUALIZATION
# ============================================================================

def detailed_fit_plot(f, z_raw, result, fit, baseline, guess_fit, guess_baseline, show_preconditioned=False,
                     transform=None, title_prefix=''):
    """
    Create detailed plot of single fit with chi-squared colored fit lines.

    Parameters:
    -----------
    f : array
        Frequency points
    z_raw : complex array
        Raw S21 data
    result : lmfit.MinimizerResult
        Fit result
    fit : complex array
        Fitted S21 (in raw coordinates)
    baseline : complex array
        Fitted baseline (in raw coordinates)
    guess_fit : complex array
        Initial guess for the fit (in raw coordinates)
    guess_baseline : complex array
        Initial guess for the baseline (in raw coordinates)
    show_preconditioned : bool
        If True, shows preconditioned data and fits
    transform : dict
        The transform parameters used for the preconditioning
    title_prefix : str
        Additional title text
    """
    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[2, 1.4], height_ratios=[1, 1],
                          hspace=0.08, wspace=0.2)
    ax_mag = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[1, 0], sharex=ax_mag)
    ax_iq = fig.add_subplot(gs[:, 1])

    # Apply preconditioning to data and fit if requested
    if show_preconditioned:
        z_plot, t             = precondition_data(f, z_raw,transform)      # display-only
        fit_plot ,t           = precondition_data(f, fit,transform)   # display-only
        baseline_plot,t       = precondition_data(f, baseline,transform)   # display-only
        guess_plot,t          = precondition_data(f, guess_fit, transform)      # display-only
        guess_baseline_plot,t = precondition_data(f, guess_baseline, transform)   # display-only
    else:
        z_plot, fit_plot, baseline_plot, guess_plot,guess_baseline_plot = z_raw, fit, baseline, guess_fit,guess_baseline

    # Get reduced chi-squared color
    redchi = result.redchi
    cmap = plt.cm.rainbow
    norm = plt.Normalize(vmin=-1, vmax=4)  # log10(chi^2) range
    fit_color = cmap(norm(np.log10(redchi)))

    # Extract parameters
    p = result.params
    f0, Qi, Qc = p['f0'].value, p['Qi'].value, p['Qc'].value
    phi, anl = p['phi'].value, p['anl'].value

    Qe = Qc * (1 + 1j*np.tan(phi))
    avg_baseline = np.median(np.abs(z_plot))
    avg_baseline_db = 20 * np.log10(avg_baseline)

    dip_depth_fit = np.abs(1 - np.sqrt(Qi**2 + Qe.imag**2) / (Qe.real + Qi))
    dip_depth_db_fit = 20 * np.log10(dip_depth_fit) if dip_depth_fit > 0 else -np.inf

    dip_depth_data = np.abs(avg_baseline / np.min(np.abs(z_plot)))
    dip_depth_db = 20 * np.log10(dip_depth_data) if dip_depth_data > 0 else -np.inf

    data_type = ' [Preconditioned]' if show_preconditioned else ''
    title = (f"f0={f0/1e6:.4f} MHz  Qi={Qi:,.0f}  Qc={Qc:,.0f}  Phi={phi:.4f}  α={anl:.4f}\n"
            f"Baseline={avg_baseline_db:.2f} dB  Dip_fit={dip_depth_db_fit:.2f} dB  Dip_data={dip_depth_db:.2f} dB  "
            f"red_chi_sq={redchi:.4f}  nfev={result.nfev}{data_type} ")
    if title_prefix:
        title = title_prefix + '\n' + title

    data_label = 'Data'

    # Magnitude
    ax_mag.plot(f/1e6, 20*np.log10(np.abs(z_plot)), 'o', alpha=0.5, ms=3, label=data_label, color='C0')
    ax_mag.plot(f/1e6, 20*np.log10(np.abs(fit_plot)), '-', lw=2, label='Fit', color=fit_color)
    ax_mag.plot(f/1e6, 20*np.log10(np.abs(baseline_plot)), '--', lw=1.5, alpha=0.7,label='Baseline', color=fit_color)
    ax_mag.plot(f/1e6, 20*np.log10(np.abs(guess_plot)), ':', lw=1., alpha=0.5,label='Guess', color='k')
    ax_mag.plot(f/1e6, 20*np.log10(np.abs(guess_baseline_plot)), '--', lw=1., alpha=0.5,label='Guess Baseline', color='k')

    ax_mag.set_ylabel('Magnitude (dB)')
    ax_mag.legend(loc='best', fontsize=8)
    ax_mag.grid(True, alpha=0.3)

    # Phase
    ax_phase.plot(f/1e6, np.unwrap(np.angle(z_plot)), 'o', alpha=0.5, ms=3, color='C0')
    ax_phase.plot(f/1e6, np.unwrap(np.angle(fit_plot)), '-', lw=2, color=fit_color)
    ax_phase.plot(f/1e6, np.unwrap(np.angle(baseline_plot)), '--', lw=1.5, alpha=0.7, color=fit_color)
    ax_phase.plot(f/1e6, np.unwrap(np.angle(guess_plot)), ':', lw=1., alpha=0.5,label='Guess', color='k')
    ax_phase.plot(f/1e6, np.unwrap(np.angle(guess_baseline_plot)), '--', lw=1., alpha=0.5,label='Guess Baseline', color='k')
    ax_phase.set_xlabel('Frequency (MHz)')
    ax_phase.set_ylabel('Phase (rad)')
    ax_phase.grid(True, alpha=0.3)

    # IQ plane
    ax_iq.plot(z_plot.real, z_plot.imag, 'o', alpha=0.5, ms=3, label=data_label, color='C0')
    ax_iq.plot(fit_plot.real, fit_plot.imag, '-', lw=2, label='Fit', color=fit_color)
    ax_iq.plot(baseline_plot.real, baseline_plot.imag, '--', lw=1.5, alpha=0.7,label='Baseline', color=fit_color)
    ax_iq.plot(guess_plot.real,guess_plot.imag, ':', lw=1, alpha=0.5,label='Guess', color='k')
    ax_iq.plot(guess_baseline_plot.real,guess_baseline_plot.imag, ':', lw=1, alpha=0.5,label='Guess Baseline', color='k')
    ax_iq.set_xlabel('Re(S21) (arb)')
    ax_iq.set_ylabel('Im(S21) (arb)')
    ax_iq.set_aspect('equal', adjustable='datalim')
    ax_iq.legend(loc='best', fontsize=8)
    ax_iq.set_title('IQ Plane', fontsize=9)
    ax_iq.grid(True, alpha=0.3)

    fig.suptitle(title, fontsize=9)

    fig.tight_layout()
    return fig



def interactive_fit_viewer(freq_list, data_list, error_list=None,
                           fit_results=None, show_preconditioned=False, interactive = True,output_dir='.',now_string=None, **fit_kwargs):
    """
    Interactive viewer:
      - If fit_results is None, fits are computed on-demand with default parameterswhen a resonance is shown.
      - Results are cached per resonance. Press 'F' to refit just the current one.

    Keys:
        → / D : next
        ← / A : previous
        F     : refit current
        P     : toggle raw/preconditioned view
        G     : toggle grid
        Q/Esc : close
    """
    import numpy as np
    import matplotlib.pyplot as plt
    plot_datestamp = datetime.now().strftime("%Y%m%d_%H%M%S")

    n_res = len(freq_list)
    assert n_res == len(data_list), "freq_list and data_list must be same length"
    if error_list is not None:
        assert n_res == len(error_list), "error_list length must match freq_list"

    # Per-dataset display transforms for PC toggle (available even if a fit hasn't run yet)
    display_transforms = []
    for f, z in zip(freq_list, data_list):
        _, t = precondition_data(f, z)  # compute the transform
        display_transforms.append(t)

    # Fit cache (tuple: (result, fit_curve, baseline, guess_curve, guess_baseline, transform))
    if fit_results is None:
        fit_cache = [None] * n_res
    else:
        assert len(fit_results) == n_res, "fit_results length must match freq_list"
        fit_cache = list(fit_results)

    # Figure & axes 
    fig = plt.figure(figsize=(12, 7))
    gs = fig.add_gridspec(2, 2, width_ratios=[2, 1.4], height_ratios=[1, 1],
                          hspace=0.08, wspace=0.2)
    ax_mag   = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[1, 0], sharex=ax_mag)
    ax_iq    = fig.add_subplot(gs[:, 1])

    state = {'index': 0, 'grid_on': True, 'show_precond': show_preconditioned}

    def get_fit_color(redchi):
        cmap = plt.cm.rainbow
        norm = plt.Normalize(vmin=-1, vmax=5)
        return cmap(norm(np.log10(max(redchi, 1e-300))))

    def ensure_fit(i, *, force=False):
        """Run fit for index i if missing or force=True; store in cache."""
        if (fit_cache[i] is None) or force:
            f = freq_list[i]; z = data_list[i]
            e = None if error_list is None else error_list[i]
            print(f"{'Refitting' if force else 'Fitting'} {i+1}/{n_res}...")
            res = fit_resonator(f, z, e, **fit_kwargs)
            # Expected order from fit_resonator:
            # (result, fit_curve, baseline, guess_curve, guess_baseline, transform)
            fit_cache[i] = res
        return fit_cache[i]

    def get_plot_arrays(i):
        """Return (f, z_raw, result, fit_raw, baseline_raw, guess_raw, guess_baseline_raw, t_for_display)."""
        f = np.asarray(freq_list[i])
        z_raw = np.asarray(data_list[i])
        # Make sure we have a fit for current index
        try:
            result, fit_raw, baseline_raw, guess_raw, guess_baseline_raw, t_fit = ensure_fit(i)
        except Exception as exc:
            # If a fit fails, keep plotting data; provide placeholders
            print(f"[WARN] Fit failed for {i+1}/{n_res}: {exc}")
            result = None
            fit_raw = np.full_like(z_raw, np.nan+1j*np.nan)
            baseline_raw = np.full_like(z_raw, np.nan+1j*np.nan)
            guess_raw = np.full_like(z_raw, np.nan+1j*np.nan)
            guess_baseline_raw = np.full_like(z_raw, np.nan+1j*np.nan)
            t_fit = None

        t_disp = t_fit or display_transforms[i]
        return f, z_raw, result, fit_raw, baseline_raw, guess_raw, guess_baseline_raw, t_disp

    def update_plot(now_string = None):
        i = state['index']
        (f, z_raw, result, fit_raw, baseline_raw,
         guess_raw, guess_baseline_raw, t_disp) = get_plot_arrays(i)

        if state['show_precond']:
            z_plot, _              = precondition_data(f, z_raw,              t_disp)
            fit_plot, _            = precondition_data(f, fit_raw,            t_disp)
            baseline_plot, _       = precondition_data(f, baseline_raw,       t_disp)
            guess_plot, _          = precondition_data(f, guess_raw,          t_disp)
            guess_baseline_plot, _ = precondition_data(f, guess_baseline_raw, t_disp)
            mode_label = "Preconditioned"
        else:
            z_plot              = z_raw
            fit_plot            = fit_raw
            baseline_plot       = baseline_raw
            guess_plot          = guess_raw
            guess_baseline_plot = guess_baseline_raw
            mode_label = "Raw"

        # Clear axes
        ax_mag.clear(); ax_phase.clear(); ax_iq.clear()

        # Magnitude
        ax_mag.plot(f/1e6, 20*np.log10(np.abs(z_plot)), 'o', alpha=0.35, ms=3, label='Data', color='C0')
        if result is not None:
            color = get_fit_color(result.redchi)
            ax_mag.plot(f/1e6, 20*np.log10(np.abs(fit_plot)), '-', lw=2, label='Fit', color=color)
            ax_mag.plot(f/1e6, 20*np.log10(np.abs(baseline_plot)), '--', lw=1.5, alpha=0.7, label='Baseline', color=color)
            ax_mag.plot(f/1e6, 20*np.log10(np.abs(guess_plot)), ':', lw=1.0, alpha=0.6, label='Guess', color='k')
            ax_mag.plot(f/1e6, 20*np.log10(np.abs(guess_baseline_plot)), ':', lw=1.0, alpha=0.4, label='Guess Baseline', color='k')
        ax_mag.set_ylabel('Magnitude (dB)')
        ax_mag.legend(loc='best', fontsize=8)

        # Phase
        ax_phase.plot(f/1e6, np.unwrap(np.angle(z_plot)), 'o', alpha=0.35, ms=3, color='C0')
        if result is not None:
            color = get_fit_color(result.redchi)
            ax_phase.plot(f/1e6, np.unwrap(np.angle(fit_plot)), '-', lw=2, color=color)
            ax_phase.plot(f/1e6, np.unwrap(np.angle(baseline_plot)), '--', lw=1.5, alpha=0.7, color=color)
            ax_phase.plot(f/1e6, np.unwrap(np.angle(guess_plot)), ':', lw=1.0, alpha=0.6, color='k')
            ax_phase.plot(f/1e6, np.unwrap(np.angle(guess_baseline_plot)), ':', lw=1.0, alpha=0.4, color='k')
        ax_phase.set_xlabel('Frequency (MHz)')
        ax_phase.set_ylabel('Phase (rad)')

        # IQ
        ax_iq.plot(z_plot.real, z_plot.imag, 'o', alpha=0.35, ms=3, label='Data', color='C0')
        if result is not None:
            color = get_fit_color(result.redchi)
            ax_iq.plot(fit_plot.real, fit_plot.imag, '-', lw=2, label='Fit', color=color)
            ax_iq.plot(baseline_plot.real, baseline_plot.imag, '--', lw=1.5, alpha=0.7, label='Baseline', color=color)
            ax_iq.plot(guess_plot.real, guess_plot.imag, ':', lw=1.0, alpha=0.6, label='Guess', color='k')
            ax_iq.plot(guess_baseline_plot.real, guess_baseline_plot.imag, ':', lw=1.0, alpha=0.4, label='Guess Baseline', color='k')
        ax_iq.set_xlabel('Re(S21)')
        ax_iq.set_ylabel('Im(S21)')
        ax_iq.set_aspect('equal', adjustable='datalim')
        ax_iq.legend(loc='best', fontsize=8)
        ax_iq.set_title('IQ Plane', fontsize=9)

        for ax in (ax_mag, ax_phase, ax_iq):
            ax.grid(state['grid_on'], alpha=0.3)

        if result is not None:
            p   = result.params
            f0  = p['f0'].value; Qi = p['Qi'].value; Qc = p['Qc'].value
            phi = p['phi'].value; anl = p['anl'].value
            Qe  = Qc * (1 + 1j*np.tan(phi))
            avg_baseline_db = 20*np.log10(np.mean(np.abs(baseline_raw)))
            dip_depth = np.abs(1 - np.sqrt(Qi**2 + (Qe.imag**2)) / (Qe.real + Qi))
            dip_depth_db = 20*np.log10(dip_depth) if dip_depth > 0 else -np.inf
            title = (f"Resonator {i+1}/{n_res} [{mode_label}]  |  "
                     f"f0={f0/1e6:.4f} MHz  Qi={Qi:,.0f}  Qc={Qc:,.0f}  φ={phi:.4f}  α={anl:.4f}\n"
                     f"Baseline={avg_baseline_db:.2f} dB  Dip={dip_depth_db:.2f} dB  "
                     f"χ²ᵣ={result.redchi:.4f}  nfev={result.nfev} round={getattr(result, 'fit_resonator_round', '?')}")
        else:
            title = f"Resonator {i+1}/{n_res} [{mode_label}]  |  (no fit)"
        fig.suptitle(title, fontsize=9)

        fig.tight_layout()
        fig.canvas.draw_idle()
        if now_string is None:
            now = time.time()
            now_string = str(int(now))
        plt.savefig(output_dir+'/'+f'fit_plot_{now_string}_{state["index"]+1:02}_{n_res}.png')

    def on_key(event):
        k = (event.key or '').lower()
        if k in ('right', 'd'):
            state['index'] = (state['index'] + 1) % n_res
            update_plot()
        elif k in ('left', 'a'):
            state['index'] = (state['index'] - 1) % n_res
            update_plot()
        elif k == 'f':
            i = state['index']
            ensure_fit(i, force=True)
            update_plot()
        elif k == 'p':
            state['show_precond'] = not state['show_precond']
            update_plot()
        elif k == 'g':
            state['grid_on'] = not state['grid_on']
            update_plot()
        elif k in ('q', 'escape'):
            plt.close(fig)

    if interactive:       
     # First draw (will trigger fit of index 0)
     update_plot(now_string)
     fig.canvas.mpl_connect('key_press_event', on_key)
    
     plt.show()
    else:
    
      for i in range(n_res):
          state['index'] = i
          update_plot(now_string)
     
   

    return fit_cache


# ============================================================================
# FIT SUMMARY TABLES
# ============================================================================

def get_fit_summary(result, index=None):
    """
    Generate summary dictionary for a single fit result.

    Parameters:
    -----------
    result : lmfit.MinimizerResult
        Fit result
    index : int, optional
        Resonator index for identification

    Returns:
    --------
    summary : dict
        Summary statistics and parameters
    """
    p = result.params

    # Derived quantities
    Qi, Qc, phi = p['Qi'].value, p['Qc'].value, p['phi'].value
    Qe = Qc * (1 + 1j*np.tan(phi))
    Qe_mag = np.abs(Qe)
    Qr = 1.0 / ((1.0/Qi) + (1.0/Qc))
    fit_rounds = getattr(result,'fit_resonator_round',None)

    summary = {
        'index': index,
        'f0': p['f0'].value,
        'f0_err': p['f0'].stderr,
        'Qi': Qi,
        'Qi_err': p['Qi'].stderr,
        'Qc': Qc,
        'Qc_err': p['Qc'].stderr,
        'Qe_mag': Qe_mag,
        'Qr': Qr,
        'Qe_phase': phi,
        'Qe_phase_err': p['phi'].stderr,
        'anl': p['anl'].value,
        'anl_err': p['anl'].stderr,
        'redchi': result.redchi,
        'chisq': result.chisqr,
        'nfev': result.nfev,
        'ndata': result.ndata,
        'nvarys': result.nvarys,
        'success': result.success,
        'attempts': fit_rounds,
        'covar_available': result.covar is not None,
    }

    return summary

def fit_summary_write_json(fit_results, outfilename=None):

    indices = list(range(len(fit_results)))

    summaries = []
    for i, (result, _, _, _, _, _) in zip(indices, fit_results):
        summaries.append(get_fit_summary(result, index=i))

    with open(outfilename, 'w', encoding='utf-8') as f:
             json.dump(summaries, f, ensure_ascii=False, indent=4)
   

        
def fit_summary_table(fit_results, indices=None, outfilename=None):
    """
    Create formatted summary table from multiple fit results.

    Parameters:
    -----------
    fit_results : list of tuples
        List of (result, fit_curve, baseline) tuples
    indices : list of int, optional
        Resonator indices (default: 0, 1, 2, ...)

    Returns:
    --------
    table_str : str
        Formatted table string
    data : list of dict
        List of summary dictionaries
    """
    if indices is None:
        indices = list(range(len(fit_results)))

    summaries = []
    for i, (result, _, _, _, _, _) in zip(indices, fit_results):
        summaries.append(get_fit_summary(result, index=i))

    # Create header
    header = (f"{'#':>4} {'f0(Hz)':>12} {'Qi':>8} {'Qc':>8} {'Qe(mag)':>8} "
             f"{'Qe(phi)':>9} {'Qr':>8} {'anl':>9} {'attempts':>8} {'ChiSq_red':>10} {'nfev':>5}" )
    separator = "-" * len(header)

    # Create rows
    rows = [header, separator]
    for s in summaries:
        idx = s['index'] if s['index'] is not None else '?'

        row = (f"{idx:>4d} {s['f0']:>12.0f} {s['Qi']:>8.0f} {s['Qc']:>8.0f} "
              f"{s['Qe_mag']:>8.0f} {s['Qe_phase']:>9.6f} {s['Qr']:>8.0f} "
              f"{s['anl']:>9.6f} {s['attempts']:8d} {s['redchi']:>10.3f} {s['nfev']:>5d} ")
        rows.append(row)

    table_str = '\n'.join(rows)

    # Optionally write to file
    if outfilename:
        # Check if file exists and, if so, append timestamp to avoid overwriting
        if os.path.exists(outfilename):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base, ext = os.path.splitext(outfilename)
            outfilename = f"{base}_{timestamp}{ext}"
        with open(outfilename, 'w') as f:
            f.write(table_str)
        print(f"Summary table written to {outfilename}")
    
        return table_str, summaries, outfilename
    else:
        return table_str, summaries, None



# ============================================================================
# SUMMARY PLOTTING
# ============================================================================

def fit_summary_plot(fit_results,vmin_vmax=None,outfilename=None):
    """
    Plot fitted parameter vs frequency with error bars, colored by fit quality.

    Parameters:
    -----------
    freq_centers : array
        Center frequencies (Hz)
    param_values : array
        Fitted parameter values
    param_errors : array
        Parameter uncertainties
    reduced_chisq : array
        Reduced chi-squared values
    param_name : str
        Parameter name for title
    ylabel : str, optional
        Y-axis label (default: param_name)
    """
    redchi=np.array([fit_results[i][0].redchi for i in range(len(fit_results))])
    f0=np.array([fit_results[i][0].params['f0'].value for i in range(len(fit_results))])
    qi=np.array([fit_results[i][0].params['Qi'].value for i in range(len(fit_results))])
    qc=np.array([fit_results[i][0].params['Qc'].value for i in range(len(fit_results))])
    anl=np.array([fit_results[i][0].params['anl'].value for i in range(len(fit_results))])
    phi=np.array([fit_results[i][0].params['phi'].value for i in range(len(fit_results))])
    m0=np.array([fit_results[i][0].params['m0'].value for i in range(len(fit_results))] if 'm0' in fit_results[0][0].params else np.zeros(len(fit_results)))
    m1=np.array([fit_results[i][0].params['m1'].value for i in range(len(fit_results))] if 'm1' in fit_results[0][0].params else np.zeros(len(fit_results)))
    p0=np.array([fit_results[i][0].params['p0'].value for i in range(len(fit_results))] if 'p0' in fit_results[0][0].params else np.zeros(len(fit_results)))
    p1=np.array([fit_results[i][0].params['p1'].value for i in range(len(fit_results))] if 'p1' in fit_results[0][0].params else np.zeros(len(fit_results)))
    
    f0_err=np.array([fit_results[i][0].params['f0'].stderr for i in range(len(fit_results))])
    qi_err=np.array([fit_results[i][0].params['Qi'].stderr for i in range(len(fit_results))])
    qc_err=np.array([fit_results[i][0].params['Qc'].stderr for i in range(len(fit_results))])
    anl_err=np.array([fit_results[i][0].params['anl'].stderr for i in range(len(fit_results))])
    phi_err=np.array([fit_results[i][0].params['phi'].stderr for i in range(len(fit_results))])
    m0_err=np.array([fit_results[i][0].params['m0'].stderr for i in range(len(fit_results))] if 'm0' in fit_results[0][0].params else np.full(len(fit_results),np.nan))
    m1_err=np.array([fit_results[i][0].params['m1'].stderr for i in range(len(fit_results))] if 'm1' in fit_results[0][0].params else np.full(len(fit_results),np.nan))
    p0_err=np.array([fit_results[i][0].params['p0'].stderr for i in range(len(fit_results))] if 'p0' in fit_results[0][0].params else np.full(len(fit_results),np.nan))
    p1_err=np.array([fit_results[i][0].params['p1'].stderr for i in range(len(fit_results))] if 'p1' in fit_results[0][0].params else np.full(len(fit_results),np.nan))


    # Common normalization for color mapping
    logchi = np.log10(np.asarray(redchi))
    if vmin_vmax is None:
        vmin, vmax = np.min(logchi[np.isfinite(logchi)]), np.max(logchi[np.isfinite(logchi)])
    
    # Shared colormap + normalize (this is the key)
    shared_norm = colors.Normalize(vmin=vmin, vmax=vmax)
    #shared_cmap = cm.get_cmap('rainbow')
    shared_cmap =  plt.colormaps.get_cmap('rainbow')
    
    # Figure + gridspec: 2 rows × 4 plots + 1 colorbar column
    fig = plt.figure(figsize=(12, 7), constrained_layout=True)
    gs = fig.add_gridspec(2, 5, width_ratios=[1, 1, 1, 1, 0.05], wspace=0.05, hspace=0.05)
    axes = np.empty((2, 4), dtype=object)

    params = ['qi', 'qc', 'm0', 'p0', 'phi', 'anl', 'm1', 'p1']
    errmap = {
        'qi':qi_err, 'qc':qc_err, 'm0':m0_err, 'p0':p0_err,
        'phi':phi_err, 'anl':anl_err, 'm1':m1_err, 'p1':p1_err
    }
    valmap = {
        'qi': qi, 'qc': qc, 'm0': m0, 'p0': p0, 
        'phi': phi,'anl': anl, 'm1': m1, 'p1': p1
    }

    # Create a single ScalarMappable for the shared colorbar
    sm = ScalarMappable(norm=shared_norm, cmap=shared_cmap)
    sm.set_array([])

    # ylabels = ['Qi', 'Qc', 'Phi (rad)', 'Nonlinearity', 'Baseline m0', 'Baseline p0', 'Baseline m1', 'Baseline p1']
    ylabels = ['Qi', 'Qc','Baseline m0', 'Baseline p0', 'Qe_phase (rad)', 'Nonlinearity', 'Baseline m1', 'Baseline p1']

    for idx, (param,ylabel) in enumerate(zip(params,ylabels)):
        r = idx // 4
        c = idx % 4
        ax = fig.add_subplot(gs[r, c])
        axes[r, c] = ax

        vals = valmap[param]
        errs = errmap[param]
        x = f0 / 1e6

        # Scatter sharing the same norm/cmap
        sc = ax.scatter(x, vals, c=logchi, cmap=shared_cmap, norm=shared_norm, marker='.')

        # Ensure errors are numeric
        errs = np.array([float(e) if np.isfinite(e) else np.nan
                        for e in np.where(np.array(errs, dtype=object)==None, np.nan, errs)],
                        dtype=float)

        segs = [((xi, yi - (err if np.isfinite(err) else 0.0)),
                (xi, yi + (err if np.isfinite(err) else 0.0)))
        for xi, yi, err in zip(x, vals, errs)]
        lc = LineCollection(segs, cmap=shared_cmap, norm=shared_norm)
        lc.set_array(logchi)
        lc.set_linewidths(1.0)
        ax.add_collection(lc)

        # Nice caps (optional miniature horizontal ticks)
        capw = (x.max() - x.min()) * 0.003 if len(x) > 1 else 0.05
        cap_segments = [((xi - capw, yi - (err if np.isfinite(err) else 0.0)),
                         (xi + capw, yi - (err if np.isfinite(err) else 0.0)))
                        for xi, yi, err in zip(x, vals, errs)]
        cap_segments += [((xi - capw, yi + (err if np.isfinite(err) else 0.0)),
                          (xi + capw, yi + (err if np.isfinite(err) else 0.0)))
                         for xi, yi, err in zip(x, vals, errs)]
        caps = LineCollection(cap_segments, cmap=shared_cmap, norm=shared_norm)
        caps.set_array(logchi)
        caps.set_linewidths(1.0)
        ax.add_collection(caps)

        # Axis cosmetics
        if param == 'phi':
            ax.set_ylim(-np.pi/2, np.pi/2)
        if param in ('qi', 'qc'):
            ax.set_ylim(3e2, 3e7)
            ax.set_yscale('log')
        if param == 'anl':
            ax.set_ylim(1e-5, 10)
            ax.set_yscale('log')
        if param in ('m0', 'm1', 'p0', 'p1'):
            print(f"param{param}  vals: {vals}")
            select = np.abs(logchi)<2
            try:
             ax.set_ylim(np.min(vals[select])*0.8,np.max(vals[select])*1.2)
            except ValueError:  #raised if `y` is empty.
             pass   
            print(f"Setting y-limits for {param}: {ax.get_ylim()}")
        
        ax.set_xlabel('Frequency (MHz)')
        ax.set_ylabel(param if ylabel is None else ylabel)
        ax.set_title(f'Fitted "{param}" vs Frequency')
        ax.grid(True, alpha=0.3)
        # ax.autoscale_view()  # include the collections

    # Shared colorbar on the rightmost column
    cax = fig.add_subplot(gs[:, 4])
    cbar = fig.colorbar(sm, cax=cax)
    cbar.set_label('Log10 Reduced ChiSq')

    #shared axis limits for all subplots
    for ax in axes.flatten():
        ax.sharex(axes[0,0])

    # Optionally save to file
    if outfilename:
        # Ensure png
        if not outfilename.endswith(".png"):
            outfilename += ".png"

        # Check if file exists and, if so, append timestamp to avoid overwriting
        if os.path.exists(outfilename):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base, ext = os.path.splitext(outfilename)
            outfilename = f"{base}_{timestamp}{ext}"
        fig.savefig(outfilename,dpi=300)
        print(f"Summary plot written to {outfilename}")
        return fig, axes, outfilename
    else:
        return fig, axes
    
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

def fit_summary_histograms(
    fit_results,
    outfilename=None,
    *,
    xscale_for=None,   # dict, e.g. {'Qi':'log', 'Qc':'log', 'Nonlinearity':'log'}
    xranges=None,      # dict, e.g. {'Qi':(1e4,1e7)}
    bins_for=None,     # dict, e.g. {'Qi':40, 'Qc':50}
    default_bins=25
):
    """
    Plot histograms of fitted parameters across multiple resonators,
    with per-parameter control over xscale ('linear' or 'log'),
    x-range, and number of bins.
    """
    # ---- extract values ----
    redchi = np.array([fr[0].redchi for fr in fit_results])
    f0     = np.array([fr[0].params['f0'].value  for fr in fit_results])
    qi     = np.array([fr[0].params['Qi'].value  for fr in fit_results])
    qc     = np.array([fr[0].params['Qc'].value  for fr in fit_results])
    anl    = np.array([fr[0].params['anl'].value for fr in fit_results])
    phi    = np.array([fr[0].params['phi'].value for fr in fit_results])
    m0     = np.array([fr[0].params['m0'].value  for fr in fit_results]) if 'm0' in fit_results[0][0].params else np.zeros(len(fit_results))
    m1     = np.array([fr[0].params['m1'].value  for fr in fit_results]) if 'm1' in fit_results[0][0].params else np.zeros(len(fit_results))
    p0     = np.array([fr[0].params['p0'].value  for fr in fit_results]) if 'p0' in fit_results[0][0].params else np.zeros(len(fit_results))
    p1     = np.array([fr[0].params['p1'].value  for fr in fit_results]) if 'p1' in fit_results[0][0].params else np.zeros(len(fit_results))

    params = {
        'f0 (MHz)': f0 / 1e6,
        'Qi': qi,
        'Qc': qc,
        'Qr':1/(1/qc+1/qi),
        'Qe_phase (rad)': phi,
        'Nonlinearity': anl,
        'Baseline m0': m0,
        'Baseline m1': m1,
        'Baseline p0': p0,
        'Baseline p1': p1,
        'Reduced ChiSq': redchi,
    }

    # defaults
    if xscale_for is None:
        xscale_for = {'Qr':'log','Qi':'log','Qc':'log','Nonlinearity':'log','Reduced ChiSq':'log'}
    if xranges is None:
        xranges = {}
    if bins_for is None:
        bins_for = {}

    # ---- figure layout ----
    n_params = len(params)
    n_cols = 4
    n_rows = (n_params + n_cols - 1) // n_cols
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(5 * n_cols, 4 * n_rows))
    axes = axes.flatten()

    for ax in axes[n_params:]:
        ax.axis('off')

    # ---- plot each histogram ----
    for ax, (name, raw_vals) in zip(axes, params.items()):
        vals = np.asarray(raw_vals)
        vals = vals[np.isfinite(vals)]

        if vals.size == 0:
            ax.text(0.5, 0.5, "No data", ha='center', va='center', transform=ax.transAxes)
            continue

        scale = xscale_for.get(name, 'linear')
        bins  = int(bins_for.get(name, default_bins))
        xmin, xmax = xranges.get(name, (vals.min(), vals.max()))

        if scale == 'log':
            # keep positive values only
            vals = vals[vals > 0]
            xmin = max(xmin, np.min(vals)) if np.any(vals > 0) else 1e-12
            xmax = max(xmax, xmin * 10)
            edges = np.logspace(np.log10(xmin), np.log10(xmax), bins + 1)
            ax.hist(vals, bins=edges, color='C0', alpha=0.7, edgecolor='black')
            ax.set_xscale('log')
        else:
            edges = np.linspace(xmin, xmax, bins + 1)
            ax.hist(vals, bins=edges, color='C0', alpha=0.7, edgecolor='black')

        ax.set_title(f'Histogram of {name}')
        ax.set_xlabel(name)
        ax.set_ylabel('Count')
        ax.grid(True, alpha=0.3)

    fig.tight_layout()

    if outfilename:
        if not outfilename.endswith(".png"):
            outfilename += ".png"
        if os.path.exists(outfilename):
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            base, ext = os.path.splitext(outfilename)
            outfilename = f"{base}_{timestamp}{ext}"
        fig.savefig(outfilename, dpi=300)
        print(f"Summary histograms written to {outfilename}")
        return fig, axes, outfilename

    return fig, axes, None
