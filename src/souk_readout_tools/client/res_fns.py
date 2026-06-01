#!/usr/bin/env python3

############################################################################
#
# This module was added in PR #10 to support the UKKIDController OCS Agent.
# 
# We will keep it here for now until the souk_readout_tools built-in fitting 
# functions are fully tested and incorated into UKKIDController.
# 
# Then it will be deprecated and removed.
#
############################################################################

import os
import time
import numpy as np
import matplotlib.pyplot as plt
#import pickle
#import souk_readout_tools
import argparse
import pdb
#from nptdms import TdmsFile
from scipy.optimize import curve_fit

def s21_model(f,f0,Qr,Qc_real,Qc_imag=0.0):
 """Symmetric resonator S21 model.

 ``f`` is the frequency array (Hz), ``f0`` the resonance frequency (Hz),
 ``Qr`` the loaded Q, and ``Qc_real``/``Qc_imag`` the real/imaginary parts
 of the complex coupling Q.
 """
 degs = 0.0
 amp = 1.0
 prefactor = amp*np.exp(1.0j*(degs/180)*np.pi)    
 return prefactor*(1.0-(Qr/(Qc_real +1.0j*Qc_imag)) / (1.0+2.0j*Qr* ((f-f0)/f0) )) 

def abs_s21(f,f0,Qr, Qc_real,Qc_imag=0.0):
  """Magnitude (dB) of :func:`s21_model` at frequencies ``f`` with resonance
  ``f0``, loaded Q ``Qr`` and complex coupling ``Qc_real``/``Qc_imag``."""
  return 20.0*np.log10(np.abs(s21_model(f,f0,Qr,Qc_real,Qc_imag)))

def arg_s21(f,f0,Qr,Qc_real,Qc_imag=0.0):
  """Phase (radians) of :func:`s21_model` at frequencies ``f`` with resonance
  ``f0``, loaded Q ``Qr`` and complex coupling ``Qc_real``/``Qc_imag``."""
  return np.angle(s21_model(f,f0,Qr,Qc_real,Qc_imag))

def clip(x,q,i,lo,hi):
    """Restrict the ``x``/``q``/``i`` arrays to samples with ``lo <= x <= hi``."""
    indices = np.where(np.logical_and(x>=lo, x<=hi))
    x= x[indices]
    q = q[indices]
    i = i[indices]
    return (x,q,i)

def find_single_resonance(x,q,i,f_tone_approx):
    '''
    Returns the frequency at which the power is minimum (the the nearest f sample).
    Inputs:
    x: frequency array
    q: q array
    i: i array
    f_tone_approx: approximate tone frequency (unused; kept for call-site
        compatibility with the resonance-search helpers).
    (Assumes input array has one distinct resonance in it).
    returns frequency at which power is minimum within this range.
    '''
    power = 10*np.log10(i**2+q**2)
    min_freq = x[np.argmin(power)]
    return min_freq

def find_all_resonances(x,q,i,tone_guesses,window_size):
    '''
    Returns an array of exact resonance frequencies (power minima) given the frequency range.
    x: full band frequency array
    q: full band q array
    i: full band i array
    tone_guesses: array of tone guess frequencies (e.g. from VNA measurements. Will search within window_size of each to 
    find true power minimum
    window_size: Window full wide the earth in Hz
    '''
    resonances =[]
    for tone in tone_guesses:
     (x_w,q_w,i_w) = clip(x,q,i,tone - window_size /2.0 , tone + window_size /2.0)
     res = find_single_resonance(x_w,q_w,i_w,tone)
     resonances.append(float(res))

    return np.array(resonances)


def fit_resonance_symmetric(x,i,q,tone,Qr_guess,Qc_guess,window_size,do_plot,verbose,save_prefix):
    """Fit one resonance with the symmetric (real-Qc) S21 magnitude model.

    Parameters
    ----------
    x, i, q : array-like
        Full-band frequency (Hz) and I/Q arrays.
    tone : float
        Approximate resonance frequency (Hz); used as the ``f0`` guess and the
        fit-window centre.
    Qr_guess, Qc_guess : float
        Initial guesses for the loaded and coupling Q.
    window_size : float
        Width (Hz) of the fit window centred on ``tone``.
    do_plot : bool
        Save and show power/phase fit plots.
    verbose : bool
        Print guessed and fitted values.
    save_prefix : str
        Filename prefix for the saved plot PNGs.

    Returns
    -------
    popt : ndarray
        Best-fit ``[f0, Qr, Qc]``.
    """
    guess_arr = [tone,Qr_guess,Qc_guess]
    (x_w,q_w,i_w) = clip(x,q,i,tone - window_size /2.0 , tone + window_size /2.0)
    power = 10*np.log10(i_w**2+q_w**2)
    power -= np.max(power)
    phase = np.atan2(q_w,i_w)
    phase -= np.mean(phase)              
    popt, pcov,infodict,errmsg, ier = curve_fit(abs_s21, x_w, power,p0=guess_arr,maxfev=100000,full_output=True)
    if verbose:
     print('Guessed values f0= %e Qr= %e Qc= %e' % (tone,Qr_guess,Qc_guess))
     print('Fitted values f0= %e Qr= %e Qc= %e' % (popt[0],popt[1],popt[2]))
     no_evals = infodict['nfev'] 
     print('No. fn evals = ' +str(infodict['nfev']))
     
    if do_plot:
     plt.plot(x_w,power)
     plt.plot(x_w,abs_s21(x_w,*popt))
     plt.xlabel('f (Hz)')
     plt.ylabel('S21 (dB)')
     plt.title(f'Power fit: f={popt[0]/1e9:.6f} GHz, Qr ={popt[1]:.3e}, Qc ={popt[2]:.3e}\n No. fn evals = {no_evals}')
     plt.savefig(save_prefix+'power_fit_'+str(tone)+'.png')
     plt.show()
     plt.plot(x_w,phase)
     plt.plot(x_w,arg_s21(x_w,*popt))
     plt.xlabel('f (Hz)')
     plt.ylabel('arg(S21) (radians)')
     plt.title(f'Phase (from power fit): f={popt[0]/1e9:.6f} GHz, Qr ={popt[1]:.3e}, Qc ={popt[2]:.3e}\n No. fn evals = {no_evals}')          
     plt.savefig(save_prefix+'phase_fit_'+str(tone)+'.png')
     plt.show()
    return popt
   
def fit_resonance_asymmetric(x,i,q,tone,Qr_guess,Qc_real_guess,Qc_imag_guess,window_size,do_plot,verbose,save_prefix):
    """Fit one resonance with the asymmetric (complex-Qc) S21 magnitude model.

    Same as :func:`fit_resonance_symmetric` but with separate
    ``Qc_real_guess`` and ``Qc_imag_guess`` initial guesses for the complex
    coupling Q.  ``x``/``i``/``q``, ``tone``, ``Qr_guess``, ``window_size``,
    ``do_plot``, ``verbose`` and ``save_prefix`` mean the same as there.

    Returns
    -------
    popt : ndarray
        Best-fit ``[f0, Qr, Qc_real, Qc_imag]``.
    """
    guess_arr = [tone,Qr_guess,Qc_real_guess,Qc_imag_guess]
    (x_w,q_w,i_w) = clip(x,q,i,tone - window_size /2.0 , tone + window_size /2.0)
    power = 10*np.log10(i_w**2+q_w**2)
    power -= np.max(power)
    phase = np.atan2(q_w,i_w)
    phase -= np.mean(phase)              
    popt, pcov,infodict,errmsg, ier = curve_fit(abs_s21, x_w, power,p0=guess_arr,maxfev=100000,full_output=True)
    if verbose:
     print('Guessed values f0= %e Qr= %e Qc_real= %e Qc_imag= %e' % (tone,Qr_guess,Qc_real_guess, Qc_imag_guess))
     print('Fitted values f0= %e Qr= %e Qc_real= %e Qc_imag= %e' % (popt[0],popt[1],popt[2],popt[3]))
     no_evals = infodict['nfev'] 
     print('No. fn evals = ' +str(infodict['nfev']))
     
    if do_plot:
     plt.plot(x_w,power)
     plt.plot(x_w,abs_s21(x_w,*popt))
     plt.xlabel('f (Hz)')
     plt.ylabel('S21 (dB)')
     plt.title(f'Power fit: f={popt[0]/1e9:.6f} GHz, Qr ={popt[1]:.3e}, Qc_real ={popt[2]:.3e},  Qc_imag ={popt[3]:.3e}\n No. fn evals = {no_evals}',fontsize =8.0)
     plt.savefig(save_prefix+'power_fit_'+str(tone)+'.png')
     plt.show()
     plt.plot(x_w,phase)
     plt.plot(x_w,arg_s21(x_w,*popt))
     plt.xlabel('f (Hz)')
     plt.ylabel('arg(S21) (radians)')
     plt.title(f'Phase (from power fit): f={popt[0]/1e9:.6f} GHz, Qr ={popt[1]:.3e}, Qc_real ={popt[2]:.3e},  Qc_imag ={popt[3]:.3e}\n No. fn evals = {no_evals}',fontsize =8.0)          
     plt.savefig(save_prefix+'phase_fit_'+str(tone)+'.png')
     plt.show()
    return popt
   
