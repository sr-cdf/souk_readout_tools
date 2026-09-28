#!/usr/bin/env python3

"""
This script is used to receive a continuous stream of data from the readout server and save it to a file.
"""

import sys
import os
import argparse
import signal

from souk_readout_tools.client import readout_client


def handle_signal(signum, frame):
    """Handles incoming signals and exits gracefully."""
    print(f"\nReceived signal {signum}. Exiting gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, handle_signal)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_signal)  # Handle termination

    parser = argparse.ArgumentParser(
        description=('Start saving a continuous stream of data from the readout server to a file. '
                     'Connect using either a local config file, or an address/request-port pair. '
                     'When address/request-port are used, the running config is pulled from the remote server.')
    )
    parser.add_argument('-C', '--config_file', type=str,
                        default=None,
                        help='Path to a local configuration file.')
    parser.add_argument('-a', '--address', type=str,
                        default=None,
                        help='Readout server address. Must be used with --request_port if --config_file is not given; the remote config will be pulled from this server.')
    parser.add_argument('-r', '--request_port', type=int,
                        default=None,
                        help='Readout server request port. Must be used with --address if --config_file is not given; the remote config will be pulled from this server.')
    parser.add_argument('-n', '--num_tones', type=int, default=None,
                        help='Expected number of active tones; default infers it from the first stream frame')
    parser.add_argument('-d', '--directory', type=str, default='./tmp', help='Directory where the file will be saved, default is ./tmp')
    parser.add_argument('-f', '--filename', type=str, default=None, help='Filename to save the data to, default is the first unused tmp_stream, tmp_stream_1, ...')
    parser.add_argument('--overwrite', action='store_true', help='Replace an existing capture at --filename')
    parser.add_argument('-p', '--print_data', action='store_true', help='Prints out the data to the console')

    parser.add_argument('--duration', type=float, default=None,
                        help='Capture duration in seconds; default waits until interrupted')

    parser.add_argument('--subscribe_tone_updates', action='store_true',
                        help='Save in-stream snapshots and tone updates alongside IQ data')

    args  = parser.parse_args()

    print("receiving stream with args: ", args)

    if args.config_file is not None:
        if args.address is not None or args.request_port is not None:
            parser.error('--config_file cannot be used with --address or --request_port')
        client = readout_client.ReadoutClient(config_file=args.config_file)
    elif args.address is not None and args.request_port is not None:
        # The client uses the request socket to pull the running config, then
        # reads the stream port from that remote config.
        client = readout_client.ReadoutClient(address=args.address,
                                             request_port=args.request_port)
        client.pull_config()
        client.stream_server_port = client.config['rfsoc_host']['stream_port']
    else:
        parser.error('Either --config_file or both --address and --request_port are required')

    if args.filename is None:
        filename = readout_client._unused_stream_filename(os.path.join(args.directory, 'tmp_stream'))
    else:
        filename = os.path.join(args.directory, args.filename)
    path = client.receive_stream(print_data=args.print_data,
                          num_tones=args.num_tones,
                          filename=filename,
                          duration=args.duration,
                          subscribe_tone_updates=args.subscribe_tone_updates,
                          overwrite=args.overwrite)
    print(f"Saved stream to {path}")

    

if __name__ == "__main__":
    main()    
