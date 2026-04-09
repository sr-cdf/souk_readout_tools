#!/usr/bin/env python3

"""
This script is used to receive a continuous stream of data from the readout server and save it to a _g3 file.
"""

import sys
import os
import argparse
import signal
import time

# append the parent directory to the path so we can import the readout_client module
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir)))

def handle_signal(signum, frame):
    """Handles incoming signals and exits gracefully."""
    print(f"\nReceived signal {signum}. Exiting gracefully...")
    sys.exit(0)

def main():
    signal.signal(signal.SIGINT, handle_signal)  # Handle Ctrl+C
    signal.signal(signal.SIGTERM, handle_signal)  # Handle termination

    parser = argparse.ArgumentParser(description='Start saving a continuous stream of data from the readout server to a file')
    parser.add_argument('-n', '--num_tones', type=int, default=2048, help='Number of tones to receive, default is all 2048. Save disk space by specifying the actual number of tones')
    parser.add_argument('-d', '--directory', type=str, default='./tmp', help='Directory where the file will be saved, default is ./tmp')
    parser.add_argument('-f', '--filename', type=str, default='tmp_stream', help='Filename to save the data to, default is tmp_stream')
    parser.add_argument('-p', '--print_data', action='store_true', help='Prints out the data to the console')
    parser.add_argument('-m', '--mock', type=str, default='F', help='Set to T or F for mocked (simulated) operation.')
    parser.add_argument('-i', '--kid_stream_id', type=str, default='UNSET', help='KID stream ID e.g. ufm_kid1 etc.')
    parser.add_argument('-t', '--time_to_stream', type=int, default=30, help='Time (in integer seconds) to run the streaming process for')

    args  = parser.parse_args()

    print("receiving stream with args: ", args)


    if args.mock == "F":
     import readout_client
    else:
     import mock_readout_client as readout_client

    client = readout_client.ReadoutClient()
    client.receive_stream_g3(print_data=args.print_data,
                          num_tones=args.num_tones,
                          filename=os.path.join(args.directory,args.filename),
                          kid_stream_id = args.kid_stream_id,
                          duration = args.time_to_stream)

    

if __name__ == "__main__":
    main()    
