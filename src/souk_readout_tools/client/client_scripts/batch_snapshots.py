#!/usr/bin/env python3

"""
CLI script for acquiring pre-accumulator snapshots across multiple tones.

Iterates over selected (or all) active tones, acquiring fast pre-accumulation
snapshots for each. Results can be saved to .npz and/or plotted.

Example usage:
    souk-batch-snapshots -C config.yaml -n 20 -P
    souk-batch-snapshots -a 10.11.11.11 --port 10000 --tones 0 3 7 -n 50 -f my_snapshots -P
    souk-batch-snapshots -C config.yaml  # all tones, 10 snapshots each, save only

Programmatic usage:
    from souk_readout_tools.client.readout_client import ReadoutClient
    client = ReadoutClient(config_file='config.yaml')
    batch = client.batch_snapshots(tone_indices=[0, 1, 2], num_snapshots=20, plot=True)
"""

import sys
import argparse
import signal

from souk_readout_tools.client.readout_client import ReadoutClient


def main():
    parser = argparse.ArgumentParser(
        description='Acquire pre-accumulator snapshots for multiple tones.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    parser.add_argument('-C', '--config_file', type=str, default=None,
                        help='Path to config YAML file.')
    parser.add_argument('-a', '--address', type=str, default=None,
                        help='RFSoC IP address (alternative to config file).')
    parser.add_argument('--port', type=int, default=None,
                        help='Request port (required with --address). '
                             'Pipeline 0: 10000, pipeline 1: 10001.')
    parser.add_argument('--tones', type=int, nargs='+', default=None,
                        help='Tone indices to snapshot (user-facing ordinal). '
                             'Default: all active tones.')
    parser.add_argument('-n', '--num_snapshots', type=int, default=10,
                        help='Number of 1024-sample snapshots per tone.')
    parser.add_argument('-f', '--filename', type=str, default=None,
                        help='Output filename (without .npz extension). '
                             'Default: batch_snapshots in current directory.')
    parser.add_argument('-P', '--plot', action='store_true',
                        help='Plot time-domain and power spectrum for each tone.')
    parser.add_argument('-q', '--quiet', action='store_true',
                        help='Suppress progress output.')

    args = parser.parse_args()

    # Validate connection arguments
    if args.config_file is None and args.address is None:
        parser.error('Provide either -C/--config_file or -a/--address (with --port).')
    if args.address is not None and args.port is None:
        parser.error('--port is required with --address (e.g. --port 10000).')

    # Default output filename
    if args.filename is None:
        args.filename = 'batch_snapshots'

    # Create client
    if args.config_file is not None:
        client = ReadoutClient(config_file=args.config_file)
    else:
        client = ReadoutClient(address=args.address, request_port=args.port)
        client.pull_config()

    # Signal handler for graceful exit
    def handle_signal(signum, frame):
        print(f"\nReceived signal {signum}. Exiting...")
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    # Run batch snapshots
    try:
        batch = client.batch_snapshots(
            tone_indices=args.tones,
            num_snapshots=args.num_snapshots,
            export_file=args.filename,
            plot=args.plot,
            verbose=not args.quiet,
        )
        n_tones = len(batch['results'])
        print(f"Done: {n_tones} tone(s), {args.num_snapshots} snapshots each.")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
