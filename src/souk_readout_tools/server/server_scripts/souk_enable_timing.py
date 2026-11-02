import argparse
import subprocess
import sys


def main():
    parser = argparse.ArgumentParser(
        description="Install and enable SOUK RFSoC PTP/chrony timing services."
    )
    parser.add_argument(
        "--preserve-config",
        action="store_true",
        help=(
            "Keep existing /etc timing config files and update service units only. "
            "By default the packaged timing config and service files are installed, "
            "with backups."
        ),
    )
    parser.add_argument(
        "--phc-offset",
        "--offset",
        type=float,
        default=-37.0,
        help=(
            "Chrony PHC refclock offset in seconds. Default is -37: the PHC runs "
            "on the PTP timescale (TAI), 37 s ahead of UTC. Use 0 only for a "
            "grandmaster serving the ARB timescale (ptpTimescale 0)."
        ),
    )
    parser.add_argument(
        "--ptp-lock-threshold-us",
        type=float,
        default=None,
        help=(
            "Largest |PTP master offset|, in microseconds, that the timing monitor "
            "counts as a good lock; a timed sync needs 5 good polls in a row. "
            "Default: the monitor's own 1 us. A lab grandmaster through an ordinary "
            "switch can sit near 1 us RMS; 20 is still 20x under one 2500 Hz sample. "
            "Without this flag a previous setting is removed, unless --preserve-config."
        ),
    )
    args = parser.parse_args()
    if args.ptp_lock_threshold_us is not None and not args.ptp_lock_threshold_us > 0:
        parser.error("--ptp-lock-threshold-us must be positive")

    try:
        from souk_readout_tools.server.readout_server import ensure_pipeline_dirs
        ensure_pipeline_dirs(0)
    except Exception as exc:
        print(f"Warning: could not prepare ~/.souk_readout_tools timing templates: {exc}")

    cmd = ["sudo", "/home/casper/.souk_readout_tools/daemon/install_timing_services.sh"]
    cmd.extend(["--phc-offset", str(args.phc_offset)])
    if args.preserve_config:
        cmd.append("--preserve-config")
    if args.ptp_lock_threshold_us is not None:
        cmd.extend(["--ptp-lock-threshold-us", str(args.ptp_lock_threshold_us)])
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
