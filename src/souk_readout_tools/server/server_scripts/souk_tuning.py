"""souk-enable-tuning, souk-disable-tuning, souk-tuning-status.

Board tuning for the KRM, from data/daemon/rt_tuning.sh: the CPU layout, each
stream loop in its own process at real-time priority, and the network settings.
Specific to that board (two pipelines, end0); leave it off on other platforms
such as the RFSoC 4x2. The script's header describes each setting.
"""
import argparse
import subprocess
import sys
from importlib.resources import files


def _run(*args):
    script = files('souk_readout_tools').joinpath('data', 'daemon', 'rt_tuning.sh')
    sys.exit(subprocess.call(['sudo', 'bash', str(script), *args]))


def enable():
    parser = argparse.ArgumentParser(
        description="Apply the KRM board tuning (CPU layout, stream process at "
                    "real-time priority, network) and restart both readout servers.")
    parser.add_argument(
        "--in-process", action="store_true",
        help="Keep each stream loop inside its server at normal priority (the "
             "older layout) instead of in its own real-time process.")
    parser.add_argument(
        "--journald-volatile", action="store_true",
        help="Keep the systemd journal in RAM; it is then lost on reboot.")
    args = parser.parse_args()
    opts = []
    if args.in_process:
        opts.append("--in-process")
    if args.journald_volatile:
        opts.append("--journald-volatile")
    _run("apply", *opts)


def disable():
    argparse.ArgumentParser(
        description="Remove the KRM board tuning and restart both readout "
                    "servers.").parse_args()
    _run("revert")


def status():
    argparse.ArgumentParser(
        description="Show the KRM board tuning: process placement, drop-ins, "
                    "interrupt and transmit-queue masks.").parse_args()
    _run("status")
