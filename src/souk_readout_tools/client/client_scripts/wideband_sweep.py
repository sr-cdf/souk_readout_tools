#!/usr/bin/env python3

"""
CLI script for performing a wideband sweep of the MKID readout system.

This script uses ReadoutClient.wideband_sweep() to perform the sweep,
then saves and optionally plots the results.

Example usage:
    souk-wideband_sweep -C config.yaml -b 500e6 -n 1024 -P
    souk-wideband_sweep -a 10.11.11.11 --port 10000 -P

Programmatic usage:
    from souk_readout_tools.client.client_scripts.wideband_sweep import wideband_sweep
    sweep_data = wideband_sweep(config_file='config.yaml', plot_data=True)
    sweep_data = wideband_sweep(address='10.11.11.11', request_port=10000, plot_data=True)
"""

import sys
import os
import argparse
import signal
import numpy as np

from souk_readout_tools.client.readout_client import ReadoutClient


def wideband_sweep(config_file=None, address=None, request_port=None,
                   bandwidth_hz=None, center_freq_hz=None,
                   step_size_hz=10000, num_tones=1024, samples_per_point=10,
                   ignore_phase_correction=True, remove_phase_slope=True,
                   tone_powers_dbm=None,
                   filename=None, filetype='npy',
                   plot_data=True):
    """
    Perform a wideband sweep of the system.

    This is a convenience wrapper that creates a client, performs the sweep,
    and saves/plots the results.

    Connect using either a config file or an address and request port.
    When connecting by address, the config is pulled from the server.

    Args:
        config_file (str): Path to the configuration file.
        address (str): RFSoC IP address (alternative to config_file).
        request_port (int): Request port (required with address).
                            Pipeline 0: 10000, pipeline 1: 10001.
        bandwidth_hz (float): Total bandwidth to measure. Default is full available bandwidth.
        center_freq_hz (float): Center frequency of the sweep. Default is band center.
        step_size_hz (float): Step size of the sweep. Number of sweep steps =
                              bandwidth / step_size / num_tones. Default is 10000.
        num_tones (int): Number of tones to use in the sweep. Default is 1024.
        samples_per_point (int): Number of samples to integrate per sweep point. Default is 10.
        ignore_phase_correction (bool): DEPRECATED. Phase correction is no longer needed
                                        following firmware fixes. Default is True (no correction).
        remove_phase_slope (bool): Remove linear phase slope from the sweep data. Default is True.
        tone_powers_dbm (float or 'auto', optional): Output tone power in dBm applied to all tones.
                            Use 'auto' to optimise dynamic range automatically. Default is None
                            (uses unit amplitudes).
        filename (str): Filename to save the data to. Default is tmp_wideband_sweep in cwd.
        filetype (str): Type of file to save. Default is 'npy'.
        plot_data (bool): Plot the data after saving. Default is True.

    Returns:
        dict: Sweep data dictionary with frequencies, I/Q data, errors, and metadata.
    """
    # Create client
    if config_file is not None:
        client = ReadoutClient(config_file=config_file)
        client.push_config()
    elif address is not None:
        client = ReadoutClient(address=address, request_port=request_port)
        client.pull_config()
    else:
        raise ValueError(
            'Provide either config_file or address with request_port.\n'
            'Example: wideband_sweep(config_file="config.yaml")\n'
            'Example: wideband_sweep(address="10.11.11.11", request_port=10000)'
        )
    
    # Perform the sweep using the client method
    sweep_data = client.wideband_sweep(
        bandwidth_hz=bandwidth_hz,
        center_freq_hz=center_freq_hz,
        step_size_hz=step_size_hz,
        num_tones=num_tones,
        samples_per_point=samples_per_point,
        tone_powers_dbm=tone_powers_dbm,
        apply_phase_correction=not ignore_phase_correction,
        remove_phase_slope=remove_phase_slope,
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
        filename: Output filename (without extension). Default is tmp_wideband_sweep in cwd.
        filetype: File format ('npy', 'csv', etc.). Default is 'npy'.
        plot_data: Whether to display a plot. Default is False.
    """
    # Determine output filename
    if filename is None:
        filename = os.path.join(os.getcwd(), 'tmp_wideband_sweep')
    filename = os.path.abspath(filename)
    
    # Ensure directory exists
    if not os.path.exists(os.path.dirname(filename)):
        os.makedirs(os.path.dirname(filename))
    
    # Export the data
    client.export_sweep(filename, sweep_data, filetype)
    final_filename = filename.replace(filetype, '') + filetype
    print(f'Wideband sweep exported to: {final_filename}')

    if plot_data:
        from souk_readout_tools.plotting import plot_sweep_magphase
        import matplotlib.pyplot as plt
        plot_sweep_magphase(sweep_data, show_errors=True)
        plt.show()

    return final_filename


def main():
    parser = argparse.ArgumentParser(
        description='Perform a wideband sweep of the MKID readout system using multiple tones.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('-C', '--config_file', type=str, default=None,
                        help='Path to config YAML file.')
    parser.add_argument('-a', '--address', type=str, default=None,
                        help='RFSoC IP address (alternative to config file).')
    parser.add_argument('-p', '--port', type=int, default=None,
                        help='Request port (required with --address). '
                             'Pipeline 0: 10000, pipeline 1: 10001.')
    parser.add_argument('-b', '--bandwidth_hz', type=float, default=None,
                        help='Total bandwidth to measure in Hz. Default: full available bandwidth.')
    parser.add_argument('-c', '--center_freq_hz', type=float, default=None,
                        help='Center frequency of the sweep in Hz. Default: band center.')
    parser.add_argument('-s', '--step_size_hz', type=float, default=10000,
                        help='Step size of the sweep in Hz.')
    parser.add_argument('-n', '--num_tones', type=int, default=1024,
                        help='Number of tones to use. More tones = fewer sweep steps.')
    parser.add_argument('-S', '--samples_per_point', type=int, default=10,
                        help='Number of samples to integrate per sweep point.')
    parser.add_argument('-i', '--ignore_phase_correction', action='store_true',
                        help='DEPRECATED (no-op). Phase correction is disabled by default '\
                             'following firmware fixes. Kept for backward compatibility.')
    parser.add_argument('--apply_phase_correction', action='store_true',
                        help='DEPRECATED. Apply legacy phase correction at filterbank channel edges. '\
                             'Not needed with current firmware.')
    parser.add_argument('-T', '--tone_powers_dbm', type=str, default=None,
                        help='Output tone power in dBm applied to all tones. '
                             'Use "auto" to optimise dynamic range automatically. '
                             'Default: None (uses unit amplitudes).')
    parser.add_argument('-f', '--filename', type=str, default=None,
                        help='Output filename (without extension). Default: tmp_wideband_sweep in current directory.')
    parser.add_argument('-t', '--filetype', type=str, default='npy',
                        help='Output file format (npy, csv, etc.).')
    parser.add_argument('--no_remove_phase_slope', action='store_true',
                        help='Do not remove linear phase slope from the sweep data.')
    parser.add_argument('-P', '--plot_data', action='store_true',
                        help='Plot the data after saving.')

    args = parser.parse_args()

    # Validate connection arguments
    if args.config_file is None and args.address is None:
        parser.error('Provide either -C/--config_file or -a/--address (with --port).')
    if args.address is not None and args.port is None:
        parser.error('--port is required with --address (e.g. --port 10000).')

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

    # Parse tone_powers_dbm: 'auto' stays as string, numeric strings become floats
    tone_powers_dbm = args.tone_powers_dbm
    if tone_powers_dbm is not None and tone_powers_dbm != 'auto':
        tone_powers_dbm = float(tone_powers_dbm)

    # Perform the sweep using the wrapper function
    try:
        wideband_sweep(
            config_file=args.config_file,
            address=args.address,
            request_port=args.port,
            bandwidth_hz=args.bandwidth_hz,
            center_freq_hz=args.center_freq_hz,
            step_size_hz=args.step_size_hz,
            num_tones=args.num_tones,
            samples_per_point=args.samples_per_point,
            ignore_phase_correction=ignore_correction,
            remove_phase_slope=not args.no_remove_phase_slope,
            tone_powers_dbm=tone_powers_dbm,
            filename=args.filename,
            filetype=args.filetype,
            plot_data=args.plot_data,
        )
    except Exception as e:
        print(f"Error during sweep: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()    
