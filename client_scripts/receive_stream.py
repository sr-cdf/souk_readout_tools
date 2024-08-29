#!/usr/bin/env python3

"""
This script is used to receive a continuous stream of data from the readout server and save it to a file.
"""

import sys
import os
import argparse
import signal

# append the parent directory to the path so we can import the readout_client module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))


def handle_signal(signum, frame):
    """Handles incoming signals and exits gracefully."""
    print(f"\nReceived signal {signum}. Exiting gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, handle_signal)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_signal)  # Handle termination

    parser = argparse.ArgumentParser(description='Start saving a continuos stream of data from the readout server to a file')
    parser.add_argument('-n', '--num_tones', type=int, default=2048, help='Number of tones to receive, default is all 2048. Save disk space by specifying the actual number of tones')
    parser.add_argument('-d', '--directory', type=str, default='./tmp', help='Directory where the file will be saved, default is ./tmp')
    parser.add_argument('-f', '--filename', type=str, default='tmp_stream', help='Filename to save the data to, default is tmp_stream')
    parser.add_argument('-p', '--print_data', action='store_true', help='Prints out the data to the console')

    args  = parser.parse_args()

    print("receiving stream with args: ", args)

    import readout_client

    client = readout_client.ReadoutClient()

    client.receive_stream(print_data=args.print_data,
                          num_tones=args.num_tones,
                          filename=os.path.join(args.directory,args.filename))

    

if __name__ == "__main__":
    main()    