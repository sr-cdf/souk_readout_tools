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
 

def wideband_sweep(config_file = None, bandwidth_hz = None, center_freq_hz = None, step_size_hz = 10000, num_tones = 1024, samples_per_point = 10, phase_correction = True, filename = None, filetype = 'npy', plot_data = True):
    """
    Perform a wideband sweep of the system.
    
    Args:
        config_file (str): Path to the configuration file, default is None
        bandwidth_hz (float): Total bandwidth to measure, default is 2024 MHz
        center_freq_hz (float): Center frequency of the weep, default is 3072 MHz
        step_size_hz (float): Step size of the sweep, number of actual sweep steps will equal (bandwidth / step_size / num_tones)
        num_tones (int): Number of tones to use in the sweep, default is 1024, using more tones require fewer sweep steps
        samples_per_point (int): Number of samples to integrate per sweep point, default is 10
        no_phase_correction (bool): Do not correct for phase jumps at filterbank channel edges
        directory (str): Directory where the file will be saved, default is ./tmp
        filename (str): Filename to save the data to, default is tmp_wideband_sweep
        filetype (str): Type of file to save, default is .npy
        plot_data (bool): Plot the data after saving
    """
    filename = 'tmp_wideband_sweep.npy' 

    if bandwidth_hz is None:
        bandwidth_hz = 2048e6
    if center_freq_hz is None:
        center_freq_hz = 2976e6
    #step_size_hz = 10000
    #num_tones = 1024,

    client = souk_readout_tools.client.ReadoutClient(config_file)
    
    
    if config_file is None:
        #if not given, use the config that is running on the server
        client.pull_config()
    else:
        #else push the specified config to the rfsoc
        client.push_config()

        ##if given, push to the rfsoc, but check first if its already running with that config
        #client_config_id = client.config['config']['config_id']    
        #client.pull_config()
        #server_config_id = client.config['config']['config_id']
        #if server_config_id != client_config_id:
        #    client.push_config()
    

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
    txnfft = 8192
    rxnfft = 8192

    dacmin = (dacduc - dacclk/2) + dacclk*(dacnyq-1)
    dacmax = (dacduc + dacclk/2) + dacclk*(dacnyq-1)

    rfmax = dacmax if not udc else max(lo+dacmax*sb, lo-dacmax*sb,lo+dacmin*sb,lo-dacmin*sb) 
    rfmin = dacmin if not udc else min(lo+dacmax*sb, lo-dacmax*sb,lo+dacmin*sb,lo-dacmin*sb)

    fmin = center_freq_hz - bandwidth_hz/2
    fmax = center_freq_hz + bandwidth_hz/2

    if (fmin < rfmin) or (fmax>rfmax):
        raise ValueError(f'Attempting to sweep out of band ({rfmin/1e6} - {rfmax/1e6} MHz)')
    

    freqs,spacings = np.linspace(fmin, fmax, num_tones, endpoint=False, retstep=True)
    
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

    s = client.parse_sweep_data(client.get_sweep_data())
    f = s['sweep_f'].T
    z = s['sweep_i'].T+1j*s['sweep_q'].T

    if phase_correction:
        #TODO: get the following constants from the sytem info or config file
        rffreqs = f
        iffreqs = sb*(rffreqs-lo)
        bbfreqs = iffreqs - dacduc
        bbbins = bbfreqs/(dacclk)*txnfft
        filterbank_bin_numbers = np.around(bbbins)
        #filterbank_bin_numbers = np.around(((4e9 - f)-1024e6)/1024e6*4096)
        z[filterbank_bin_numbers%2==1]*=np.exp(1j*np.pi)

    phi = np.unwrap(np.angle(np.ravel(z.T)))
    slope = np.nanmedian(np.gradient(phi,np.ravel(f.T)))
    z *= np.exp(-1j*(slope*f+np.pi))

    s['sweep_i'] = np.real(z).T
    s['sweep_q'] = np.imag(z).T
    
    
    if filename is None:
        filename = os.path.expanduser('~/.souk_readout_tools/tmp/tmp_wideband_sweep')
    filename = os.path.abspath(filename)
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    client.export_sweep(filename, s, filetype)

    if plot_data:
        si = s['sweep_i'].T
        sq = s['sweep_q'].T
        logmag = 20*np.log10(abs(z))
        uphase = np.unwrap(np.angle(np.ravel(z.T))).reshape(z.T.shape).T

        ei = s['sweep_ei'].T #/ np.sqrt(s['samples_per_point'])
        eq = s['sweep_eq'].T #/ np.sqrt(s['samples_per_point'])
        emag = 1/abs(z)*np.sqrt((si*ei)**2 + (sq*ei)**2)
        elogmag = 20/np.abs(z)/np.log(10)*emag
        ephi = 1/(si**2+sq**2) * np.sqrt((sq*ei)**2+(si*eq)**2)
        
        import matplotlib.pyplot as plt
        fig,(s1,s2) = plt.subplots(2,1,sharex=True)
        #s1.plot(f/1e6, logmag)
        #s2.plot(f/1e6, uphase)
        s1.errorbar(np.ravel(f)/1e6, np.ravel(logmag), yerr=np.ravel(elogmag), fmt='.', ecolor='red')
        s2.errorbar(np.ravel(f)/1e6, np.ravel(uphase), yerr=np.ravel(ephi), fmt='.', ecolor='red')
        fig.supxlabel('Frequency (MHz)')
        s1.set_ylabel('Power (dB)')
        s2.set_ylabel('Phase (rad)')
        s1.set_ylim(np.min(logmag),np.max(logmag))
        s2.set_ylim(np.min(uphase),np.max(uphase))
        plt.show()

    
    return s

def main():

    parser = argparse.ArgumentParser(description='Sweep out the full bandwidth of the system eith multiple tones to save time')
    parser.add_argument('-C', '--config_file', type=str, default=None, help='Path to the configuration file, default is None')
    parser.add_argument('-b', '--bandwidth_hz', type=float, default=2048e6, help='Total bandwidth to measure, default is 2024 MHz')
    parser.add_argument('-c', '--center_freq_hz', type=float, default=2976e6, help='Center frequency of the weep, default is 3072 MHz')
    parser.add_argument('-s', '--step_size_hz', type=float, default=10000, help='Step size of the sweep, number of actual sweep steps will equal (bandwidth / step_size / num_tones')
    parser.add_argument('-n', '--num_tones', type=int, default=1024, help='Number of tones to use in the sweep, default is 1024, using more tones require fewer sweep steps')
    parser.add_argument('-p', '--samples_per_point', type=int, default=10, help='Number of samples to integrate per sweep point, default is 10')    
    parser.add_argument('-x', '--no_phase_correction', action='store_true', help='Do not correct for phase jumps at filterbank channel edges')    
    
    parser.add_argument('-d', '--directory', type=str, default='../tmp', help='Directory where the file will be saved, default is ./tmp')
    parser.add_argument('-f', '--filename', type=str, default='tmp_wideband_sweep.npy', help='Filename to save the data to, default is tmp_wideband_sweep')
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
                             phase_correction = not args.no_phase_correction,
                                 filename = args.filename,
                                   filetype = args.filetype,
                                     plot_data = args.plot_data)

if __name__ == "__main__":
    main()    
