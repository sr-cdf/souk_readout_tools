#!/usr/bin/env python3

"""
CLI script for finding MKID resonances in sweep data.

Performs a wideband sweep (or loads existing data), finds resonances,
optionally fits them, and exports results.

Example usage:
    souk-find-resonances -C config.yaml -P
    souk-find-resonances -C config.yaml --fit -f resonances.txt -P
    souk-find-resonances --load sweep.npy --fit --prominence 3.0
"""

import sys
import argparse
import signal
import numpy as np

from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools.peak_finder import (
    find_mkid_resonances, FilterParams, PeakFinderParams
)


def main():
    parser = argparse.ArgumentParser(
        description='Find MKID resonances in sweep data.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    # Connection
    conn = parser.add_argument_group('connection')
    conn.add_argument('-C', '--config_file', type=str, default=None,
                      help='Path to config YAML file.')
    conn.add_argument('-a', '--address', type=str, default=None,
                      help='RFSoC IP address (alternative to config file).')
    conn.add_argument('-p', '--port', type=int, default=None,
                      help='Request port (required with --address).')
    conn.add_argument('--load', type=str, default=None,
                      help='Load sweep data from .npy file instead of acquiring.')

    # Sweep parameters
    sweep = parser.add_argument_group('sweep')
    sweep.add_argument('-b', '--bandwidth_hz', type=float, default=None,
                       help='Sweep bandwidth in Hz.')
    sweep.add_argument('-s', '--step_size_hz', type=float, default=10000,
                       help='Sweep step size in Hz.')
    sweep.add_argument('-n', '--num_tones', type=int, default=1024,
                       help='Number of tones for sweep.')

    # Peak finder parameters
    pf = parser.add_argument_group('peak finding')
    pf.add_argument('--prominence', type=float, default=1.0,
                    help='Minimum prominence for peak detection (dB).')
    pf.add_argument('--min_width_hz', type=float, default=100,
                    help='Minimum resonance width in Hz.')
    pf.add_argument('--max_width_hz', type=float, default=10e6,
                    help='Maximum resonance width in Hz.')
    pf.add_argument('--min_distance_hz', type=float, default=1000,
                    help='Minimum distance between resonances in Hz.')

    # Fitting
    parser.add_argument('--fit', action='store_true',
                        help='Fit each resonance to a notch-type model.')

    # Output
    parser.add_argument('-f', '--filename', type=str, default=None,
                        help='Output filename for resonance list (.txt).')
    parser.add_argument('-P', '--plot', action='store_true',
                        help='Plot the sweep with resonance markers.')

    args = parser.parse_args()

    # Validate
    if args.load is None and args.config_file is None and args.address is None:
        parser.error('Provide -C/--config_file, -a/--address, or --load.')
    if args.address is not None and args.port is None:
        parser.error('--port is required with --address.')

    # Load or acquire sweep data
    if args.load is not None:
        print(f"Loading sweep data from {args.load}...")
        sweep_data = np.load(args.load, allow_pickle=True).item()
    else:
        if args.config_file is not None:
            client = ReadoutClient(config_file=args.config_file)
        else:
            client = ReadoutClient(address=args.address, request_port=args.port)
            client.pull_config()

        def handle_signal(signum, frame):
            print(f"\nSignal {signum}. Exiting...")
            sys.exit(0)
        signal.signal(signal.SIGINT, handle_signal)
        signal.signal(signal.SIGTERM, handle_signal)

        print("Performing wideband sweep...")
        sweep_data = client.wideband_sweep(
            bandwidth_hz=args.bandwidth_hz,
            step_size_hz=args.step_size_hz,
            num_tones=args.num_tones,
            verbose=True,
        )

    # Find resonances
    f_all = np.ravel(sweep_data['sweep_f'])
    z_all = np.ravel(sweep_data['sweep_i']) + 1j * np.ravel(sweep_data['sweep_q'])

    fp = FilterParams()
    pp = PeakFinderParams(
        prominence_min=args.prominence,
        width_min=args.min_width_hz,
        width_max=args.max_width_hz,
        distance_value=args.min_distance_hz,
    )

    print("Finding resonances...")
    resonances = find_mkid_resonances(f_all, z_all, filter_params=fp, finder_params=pp)
    print(f"Found {len(resonances)} resonances.")

    # Optional fitting
    fit_results = None
    if args.fit and resonances:
        from souk_readout_tools.fitting import batch_fit
        print("Fitting resonances...")
        fit_results = batch_fit(sweep_data, resonances)
        print(f"Fitted {len(fit_results)} resonances.")

    # Print results
    print(f"\n{'Idx':>4}  {'Frequency (MHz)':>15}  {'FWHM (kHz)':>10}  "
          f"{'Ql':>8}  {'Qi':>8}  {'Qc':>8}  {'Dip (dB)':>8}")
    print("-" * 80)

    source = fit_results if fit_results else resonances
    for i, r in enumerate(source):
        fr = r.fr if fit_results else r.frequency
        fwhm = (fr / r.Ql * 1e-3) if fit_results else (r.fwhm or 0) * 1e-3
        Ql = r.Ql if fit_results else (r.q_factor or 0)
        Qi = r.Qi if fit_results else (r.qi or 0)
        Qc = r.Qc_abs if fit_results else (r.qc or 0)
        dip = (r.residual_rms if fit_results else r.dip_depth) or 0
        if fit_results:
            dip = r.residual_rms
            dip_str = f'{dip:.2e}'
        else:
            dip = r.dip_depth or 0
            dip_str = f'{dip:.2f}'
        print(f"{i:4d}  {fr/1e6:15.6f}  {fwhm:10.3f}  "
              f"{Ql:8.0f}  {Qi:8.0f}  {Qc:8.0f}  {dip_str:>8}")

    # Save results
    if args.filename:
        with open(args.filename, 'w') as fout:
            fout.write(f"# Resonances found: {len(source)}\n")
            if fit_results:
                fout.write("# Fitted with notch-type resonator model\n")
            fout.write(f"# {'Frequency_Hz':>15}\t{'FWHM_Hz':>12}\t"
                       f"{'Ql':>10}\t{'Qi':>10}\t{'Qc':>10}\t{'Dip_dB':>10}\n")
            for r in source:
                fr = r.fr if fit_results else r.frequency
                fwhm = (fr / r.Ql) if fit_results else (r.fwhm or 0)
                Ql = r.Ql if fit_results else (r.q_factor or 0)
                Qi = r.Qi if fit_results else (r.qi or 0)
                Qc = r.Qc_abs if fit_results else (r.qc or 0)
                dip = r.dip_depth if not fit_results else 0
                fout.write(f"{fr:>16.6f}\t{fwhm:>12.3f}\t"
                           f"{Ql:>10.1f}\t{Qi:>10.1f}\t{Qc:>10.1f}\t{dip:>10.3f}\n")
        print(f"\nResonances saved to {args.filename}")

    # Plot
    if args.plot:
        from souk_readout_tools.plotting import plot_sweep_magphase
        import matplotlib.pyplot as plt

        fig = plot_sweep_magphase(sweep_data, show_errors=True)
        ax = fig.axes[0]

        for r in source:
            fr = (r.fr if fit_results else r.frequency) / 1e6
            ax.axvline(fr, color='green', alpha=0.5, linewidth=0.5)

        ax.set_title(f'Wideband Sweep — {len(source)} resonances')
        plt.show()


if __name__ == "__main__":
    main()
