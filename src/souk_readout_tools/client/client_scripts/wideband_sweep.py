#!/usr/bin/env python3

"""
This script is used to sweep out the full bandwidth of the system.
"""

import sys
import os
import argparse
import signal
import numpy as np
import time

import souk_readout_tools
 

def wideband_sweep(config_file = None, bandwidth_hz = None, center_freq_hz = None, step_size_hz = 10000, num_tones = 1024, samples_per_point = 10, ignore_phase_correction = False, filename = None, filetype = 'npy', plot_data = True):
    """
    Perform a wideband sweep of the system.
    
    Args:
        config_file (str): Path to the configuration file, default is None
        bandwidth_hz (float): Total bandwidth to measure, default is 2024 MHz
        center_freq_hz (float): Center frequency of the weep, default is 3072 MHz
        step_size_hz (float): Step size of the sweep, number of actual sweep steps will equal (bandwidth / step_size / num_tones)
        num_tones (int): Number of tones to use in the sweep, default is 1024, using more tones require fewer sweep steps
        samples_per_point (int): Number of samples to integrate per sweep point, default is 10
        ignore_phase_correction (bool): Do not correct for phase jumps at filterbank channel edges. Default is False, meaning the phase correction is applied by default)
        filename (str): Filename to save the data to, default is tmp_wideband_sweep
        filetype (str): Type of file to save, default is .npy
        plot_data (bool): Plot the data after saving
    """

    client = souk_readout_tools.client.ReadoutClient(config_file)
    
    
    if config_file is None:
        #if not given, use the config that is running on the server
        client.pull_config()
    else:
        #else push the specified config to the rfsoc
        client.push_config()
    

    p = client.get_sweep_progress()
    if p==0.0:
        pass
    elif p != 1.0:
        print(f'There is already a sweep in progress ({p*100:.3f}%), please wait for it to finish before starting a new one.')
        return
    
    info = client.get_system_information()
    
    udc = client.config['rf_frontend']['connected']
    lo = client.config['rf_frontend']['tx_mixer_lo_frequency_hz']
    sb = client.config['rf_frontend']['tx_mixer_sideband']
    
    adcclk = info['adc_clk_hz']
    dacclk = adcclk
    dacduc = info['dac_duc_mixer_frequency_hz']
    dacnyq = info['nyquist_zone_dac0']
    dacint = 2
    txnfft = 8192
    rxnfft = 8192
    
    #dbbmin = -dacclk/dacint/2
    #dbbmax = +dacclk/dacint/2

    dbbmin = -dacclk/2
    dbbmax = +dacclk/2

    dacmin = min([abs(dbbmin+dacduc),abs(dbbmax+dacduc)])
    dacmax = max([abs(dbbmin+dacduc),abs(dbbmax+dacduc)])

    
    rfmin = dacmin
    rfmax = dacmax

    if udc:
        if sb==1:
            rfmin = lo + dacmin
            rfmax = lo + dacmax
        elif sb==-1:
            rfmin = lo - dacmax
            rfmax = lo - dacmin
        else:
            raise ValueError(f"Invalid sideband value {sb}, should be +1 for USB or -1 for LSB")

    if bandwidth_hz is None:
        bandwidth_hz=rfmax-rfmin

    if center_freq_hz is None:
        center_freq_hz = (rfmax+rfmin)/2


    fmin = center_freq_hz - bandwidth_hz/2
    fmax = center_freq_hz + bandwidth_hz/2

    if (fmin < rfmin) or (fmax>rfmax):
        raise ValueError(f'Attempting to sweep out of band (band = {rfmin/1e6} - {rfmax/1e6} MHz, requested {fmin/1e6} - {fmax/1e6} MHz)')
    

    freqs,spacings = np.linspace(fmin, fmax, num_tones, endpoint=False, retstep=True)
    
    if spacings <= dacclk/txnfft:
        raise ValueError(f'Tone spacing must be greater than {dacclk/txnfft} Hz but it is {spacings}. Try fewer tones or wider bandwidth.') 

    # sweep_points = 41 # not too many as its currently quite slow
    sweep_points = int(bandwidth_hz / step_size_hz / num_tones) 
    sweep_span = spacings * (sweep_points-1)/(sweep_points)

    # np.random.seed(0)
    # small_offsets = np.random.uniform(-sweep_span/sweep_points/20,+sweep_span/sweep_points/20,num_tones) # the offsets is less than 10% of the step size
    small_offsets = np.random.uniform(-sweep_span/sweep_points/2,+sweep_span/sweep_points/2,num_tones) # the largest combined offset is <= 100% of the step size
    # small_offsets = np.around(small_offsets) # round to nearest integer
    freqs += small_offsets

    
    center_freqs = freqs + np.floor(sweep_points/2)*spacings/sweep_points
    tone_amplitudes = np.ones(num_tones) # set_amplitudes to max
    tone_phases = client.generate_newman_phases(center_freqs)

    print('udc:',udc)
    print('lo:',lo)
    print('sb:',sb)
    print('dacduc:',dacduc)
    print('dacclk:',dacclk)
    print('dacnyq:',dacnyq)
    print('txnfft:',txnfft)
    print('rxnfft:',rxnfft)
    print('dacmin:',dacmin)
    print('dacmax:',dacmax)
    print('rfmin:',rfmin)
    print('rfmax:',rfmax)
    print('bandwidth:',bandwidth_hz)
    print('freqs:',freqs)
    print(center_freqs)
    print(center_freqs.min(),center_freqs.max())

    client.set_tone_frequencies(center_freqs)
    client.set_tone_amplitudes(tone_amplitudes)
    client.set_tone_phases(tone_phases)

    outps = client.check_output_saturation()
    inps  = client.check_input_saturation()
    dspof = client.check_dsp_overflow()
    
    if outps['result']:
        raise RuntimeError(f"Output saturation detected: {outps['details']}")
    if inps['result']:
        raise RuntimeError(f"Input saturation detected: {inps['details']}")
    if dspof['result']:
        raise RuntimeError(f"DSP overflow detected: {dspof['details']}")
    
    response = client.perform_sweep(center_freqs,
                                    sweep_span,
                                    points = sweep_points,
                                    samples_per_point = samples_per_point,
                                    direction = 'up')
    
    if response['status'] != 'success':
        raise RuntimeError(f"Sweep failed with message: {response['message']}")
    
    #TODO make this an option in the perform_sweep function
    while True:
        p=client.get_sweep_progress()
        print(f'Sweep progress: {100*p:.3f}%',end='\r',flush=True)
        if p==1.0: break
        else: time.sleep(1.0)

    s = client.parse_sweep_data(client.get_sweep_data(),apply_phase_correction=not ignore_phase_correction)
    f = s['sweep_f']
    z = s['sweep_i']+1j*s['sweep_q']

    #remove slope from phase
    fcat = np.ravel(f.T)
    zcat = np.ravel(z.T)
    phicat = np.angle(zcat)
    slope = np.nanmedian(np.gradient(phicat,fcat))
    zcat *= np.exp(-1j*(slope*fcat))

    s['sweep_f'] = [fcat]
    s['sweep_i'] = [np.real(zcat)]
    s['sweep_q'] = [np.imag(zcat)]
    s['sweep_ei'] = [np.ravel(s['sweep_ei'].T)]
    s['sweep_eq'] = [np.ravel(s['sweep_eq'].T)]

    if filename is None:
        filename = os.path.expanduser('~/.souk_readout_tools/tmp/tmp_wideband_sweep')
    filename = os.path.abspath(filename)
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    client.export_sweep(filename, s, filetype)
    filename = filename.replace(filetype,'')+filetype
    print('Wideband sweep exported to:',filename)

    if plot_data:
        sf = s['sweep_f'][0]
        si = s['sweep_i'][0]
        sq = s['sweep_q'][0]
        sz = si+1j*sq
        logmag = 20*np.log10(abs(sz))
        uphase = np.unwrap(np.angle(sz))

        ei = s['sweep_ei'][0] #/ np.sqrt(s['samples_per_point'])
        eq = s['sweep_eq'][0] #/ np.sqrt(s['samples_per_point'])
        emag = 1/abs(sz)*np.sqrt((si*ei)**2 + (sq*ei)**2)
        elogmag = 20/np.abs(sz)/np.log(10)*emag
        ephi = 1/(si**2+sq**2) * np.sqrt((sq*ei)**2+(si*eq)**2)
        
        import matplotlib.pyplot as plt
        fig,(s1,s2) = plt.subplots(2,1,sharex=True)
        #s1.plot(f/1e6, logmag)
        #s2.plot(f/1e6, uphase)
        s1.errorbar(sf/1e6, logmag, yerr=elogmag, fmt='.', ecolor='red')
        s2.errorbar(sf/1e6, uphase, yerr=ephi, fmt='.', ecolor='red')
        fig.supxlabel('Frequency (MHz)')
        s1.set_ylabel('Power (dB)')
        s2.set_ylabel('Phase (rad)')
        s1.set_ylim(np.min(logmag),np.max(logmag))
        s2.set_ylim(np.min(uphase),np.max(uphase))
        plt.show()

    
    return s

def main():

    parser = argparse.ArgumentParser(description='Sweep out the full bandwidth of the system eith multiple tones to save time')
    parser.add_argument('-C', '--config_file', type=str, default=None, help='Path to the configuration file, default is to search ~/.souk_readout_tools/config/default_config.lnk')
    parser.add_argument('-b', '--bandwidth_hz', type=float, default=None, help='Total bandwidth to measure, default is the bandwidth defined in the config MHz')
    parser.add_argument('-c', '--center_freq_hz', type=float, default=None, help='Center frequency of the sweep, default is the band center defined by the config')
    parser.add_argument('-s', '--step_size_hz', type=float, default=10000, help='Step size of the sweep in hz, number of actual sweep steps will equal (bandwidth / step_size / num_tones')
    parser.add_argument('-n', '--num_tones', type=int, default=1024, help='Number of tones to use in the sweep, default is 1024, using more tones require fewer sweep steps')
    parser.add_argument('-p', '--samples_per_point', type=int, default=10, help='Number of samples to integrate per sweep point, default is 10')    
    parser.add_argument('-i', '--ignore_phase_correction', action='store_true', help='Do not correct for phase jumps at filterbank channel edges, ')    
    parser.add_argument('-f', '--filename', type=str, default=None, help='Filename to save the data to, default is ~/.souk_readout_tools/tmp/tmp_wideband_sweep.npy')
    parser.add_argument('-t', '--filetype', type=str, default='npy', help='Type of file to save, default is .npy')
    parser.add_argument('-P', '--plot_data', action='store_true', help='Plot the data after saving')

    args  = parser.parse_args()

    print("Starting wideband sweep with args: ", args)

    def handle_signal(signum, frame):
        """Handles incoming signals and exits gracefully."""
        print(f"\nReceived signal {signum}. Exiting gracefully...")
        client = souk_readout_tools.client.ReadoutClient(config_file=args.config_file)
        client.cancel_all_tasks()

        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_signal)  # Handle termination


    wideband_sweep(config_file = args.config_file, 
                   bandwidth_hz = args.bandwidth_hz,
                     center_freq_hz = args.center_freq_hz,
                       step_size_hz = args.step_size_hz,
                         num_tones = args.num_tones,
                           samples_per_point = args.samples_per_point,
                             ignore_phase_correction = args.ignore_phase_correction,
                                 filename = args.filename,
                                   filetype = args.filetype,
                                     plot_data = args.plot_data)

if __name__ == "__main__":
    main()    
