#!/usr/bin/env python3

"""
CLI script for performing a wideband sweep of the MKID readout system.

This script uses ReadoutClient.wideband_sweep() to perform the sweep,
then saves and optionally plots the results.

Example usage:
    souk-wideband-sweep -C config.yaml -b 500e6 -n 1024 -P
    souk-wideband-sweep --pipeline 0 --plot_data
    
Programmatic usage:
    from souk_readout_tools.client.client_scripts.wideband_sweep import wideband_sweep
    sweep_data = wideband_sweep(config_file='config.yaml', plot_data=True)
"""

import sys
import os
import argparse
import signal
import numpy as np

from souk_readout_tools.client.readout_client import ReadoutClient


def wideband_sweep(config_file=None, bandwidth_hz=None, center_freq_hz=None, 
                   step_size_hz=10000, num_tones=1024, samples_per_point=10, 
                   ignore_phase_correction=True, filename=None, filetype='npy', 
                   plot_data=True, pipeline_id=None):
    """
    Perform a wideband sweep of the system.
    
    This is a convenience wrapper that creates a client, performs the sweep,
    and saves/plots the results.
    
    Args:
        config_file (str): Path to the configuration file. Default is None (uses default config).
        bandwidth_hz (float): Total bandwidth to measure. Default is full available bandwidth.
        center_freq_hz (float): Center frequency of the sweep. Default is band center.
        step_size_hz (float): Step size of the sweep. Number of sweep steps = 
                              bandwidth / step_size / num_tones. Default is 10000.
        num_tones (int): Number of tones to use in the sweep. Default is 1024.
        samples_per_point (int): Number of samples to integrate per sweep point. Default is 10.
        ignore_phase_correction (bool): DEPRECATED. Phase correction is no longer needed 
                                        following firmware fixes. Default is True (no correction).
        filename (str): Filename to save the data to. Default is tmp_wideband_sweep in tmp dir.
        filetype (str): Type of file to save. Default is 'npy'.
        plot_data (bool): Plot the data after saving. Default is True.
        pipeline_id (int): Pipeline ID (0 or 1). If None, extracted from config. Default is None.
    
    Returns:
        dict: Sweep data dictionary with frequencies, I/Q data, errors, and metadata.
    """
    # Create client
    client = ReadoutClient(config_file=config_file, pipeline_id=pipeline_id)
    
    # Handle config: if none given, pull from server; else push ours
    if config_file is None:
        client.pull_config()
    else:
        client.push_config()
    
    # Perform the sweep using the client method
    sweep_data = client.wideband_sweep(
        bandwidth_hz=bandwidth_hz,
        center_freq_hz=center_freq_hz,
        step_size_hz=step_size_hz,
        num_tones=num_tones,
        samples_per_point=samples_per_point,
        apply_phase_correction=not ignore_phase_correction,
        verbose=True
    )
    
    # Save and optionally plot
    save_and_plot_sweep(
        client=client,
        sweep_data=sweep_data,
        filename=filename,
        filetype=filetype,
        plot_data=plot_data
    )
    
    return sweep_data


def save_and_plot_sweep(client, sweep_data, filename=None, filetype='npy', plot_data=False):
    """
    Save sweep data to file and optionally plot it.
    
    Args:
        client: ReadoutClient instance (for export_sweep method)
        sweep_data: Sweep data dictionary from wideband_sweep()
        filename: Output filename (without extension). Default is pipeline-specific tmp dir.
        filetype: File format ('npy', 'csv', etc.). Default is 'npy'.
        plot_data: Whether to display a plot. Default is False.
    """
    # Determine output filename
    if filename is None:
        filename = os.path.join(client.user_tmp_dir, 'tmp_wideband_sweep')
    filename = os.path.abspath(filename)
    
    # Ensure directory exists
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    
    # Export the data
    client.export_sweep(filename, sweep_data, filetype)
    final_filename = filename.replace(filetype, '') + filetype
    print(f'Wideband sweep exported to: {final_filename}')

    if plot_data:
        sf = sweep_data['sweep_f'][0]
        si = sweep_data['sweep_i'][0]
        sq = sweep_data['sweep_q'][0]
        sz = si + 1j * sq
        logmag = 20 * np.log10(np.abs(sz))
        uphase = np.unwrap(np.angle(sz))

        ei = sweep_data['sweep_ei'][0]
        eq = sweep_data['sweep_eq'][0]
        emag = 1 / np.abs(sz) * np.sqrt((si * ei)**2 + (sq * eq)**2)
        elogmag = 20 / np.abs(sz) / np.log(10) * emag
        ephi = 1 / (si**2 + sq**2) * np.sqrt((sq * ei)**2 + (si * eq)**2)
        
        import matplotlib.pyplot as plt
        fig, (s1, s2) = plt.subplots(2, 1, sharex=True)
        s1.errorbar(sf / 1e6, logmag, yerr=elogmag, fmt='.', ecolor='red', markersize=2)
        s2.errorbar(sf / 1e6, uphase, yerr=ephi, fmt='.', ecolor='red', markersize=2)
        fig.supxlabel('Frequency (MHz)')
        s1.set_ylabel('Power (dB)')
        s2.set_ylabel('Phase (rad)')
        s1.set_ylim(np.nanmin(logmag), np.nanmax(logmag))
        s2.set_ylim(np.nanmin(uphase), np.nanmax(uphase))
        fig.suptitle(f'Wideband Sweep ({sweep_data.get("bandwidth_hz", 0)/1e6:.1f} MHz)')
        plt.tight_layout()
        plt.show()

    return final_filename


def main():
    parser = argparse.ArgumentParser(
        description='Perform a wideband sweep of the MKID readout system using multiple tones.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('-C', '--config_file', type=str, default=None,
                        help='Path to config YAML. Default: uses default_config.lnk for the pipeline.')
    parser.add_argument('--pipeline', type=int, default=None, choices=[0, 1],
                        help='Pipeline ID (0 or 1). If omitted, extracted from config or defaults to 0.')
    parser.add_argument('-b', '--bandwidth_hz', type=float, default=None,
                        help='Total bandwidth to measure in Hz. Default: full available bandwidth.')
    parser.add_argument('-c', '--center_freq_hz', type=float, default=None,
                        help='Center frequency of the sweep in Hz. Default: band center.')
    parser.add_argument('-s', '--step_size_hz', type=float, default=10000,
                        help='Step size of the sweep in Hz.')
    parser.add_argument('-n', '--num_tones', type=int, default=1024,
                        help='Number of tones to use. More tones = fewer sweep steps.')
    parser.add_argument('-p', '--samples_per_point', type=int, default=10,
                        help='Number of samples to integrate per sweep point.')
    parser.add_argument('-i', '--ignore_phase_correction', action='store_true',
                        help='DEPRECATED (no-op). Phase correction is disabled by default '\
                             'following firmware fixes. Kept for backward compatibility.')
    parser.add_argument('--apply_phase_correction', action='store_true',
                        help='DEPRECATED. Apply legacy phase correction at filterbank channel edges. '\
                             'Not needed with current firmware.')
    parser.add_argument('-f', '--filename', type=str, default=None,
                        help='Output filename (without extension). Default: tmp_wideband_sweep in pipeline tmp dir.')
    parser.add_argument('-t', '--filetype', type=str, default='npy',
                        help='Output file format (npy, csv, etc.).')
    parser.add_argument('-P', '--plot_data', action='store_true',
                        help='Plot the data after saving.')

    args = parser.parse_args()

    print("Starting wideband sweep...")

    # Set up signal handler for graceful exit (need a client for cancel)
    # We create a temporary reference that will be set once client exists
    client_ref = [None]
    
    def handle_signal(signum, frame):
        print(f"\nReceived signal {signum}. Cancelling tasks and exiting...")
        if client_ref[0] is not None:
            try:
                client_ref[0].cancel_all_tasks()
            except Exception:
                pass
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Determine phase correction setting:
    # - Default is no correction (ignore_phase_correction=True)
    # - --apply_phase_correction overrides to apply correction
    # - -i/--ignore_phase_correction is now a no-op (kept for backward compat)
    ignore_correction = not args.apply_phase_correction  # True unless --apply_phase_correction given

    # Perform the sweep using the wrapper function
    try:
        wideband_sweep(
            config_file=args.config_file,
            bandwidth_hz=args.bandwidth_hz,
            center_freq_hz=args.center_freq_hz,
            step_size_hz=args.step_size_hz,
            num_tones=args.num_tones,
            samples_per_point=args.samples_per_point,
            ignore_phase_correction=ignore_correction,
            filename=args.filename,
            filetype=args.filetype,
            plot_data=args.plot_data,
            pipeline_id=args.pipeline
        )
    except Exception as e:
        print(f"Error during sweep: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()    
