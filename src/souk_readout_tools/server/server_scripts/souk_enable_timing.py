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
        default=0.0,
        help=(
            "Chrony PHC refclock offset in seconds. Default is 0 for a "
            "production UTC grandmaster; use --offset=-37 for the current lab "
            "software grandmaster."
        ),
    )
    args = parser.parse_args()

    try:
        from souk_readout_tools.server.readout_server import ensure_pipeline_dirs
        ensure_pipeline_dirs(0)
    except Exception as exc:
        print(f"Warning: could not prepare ~/.souk_readout_tools timing templates: {exc}")

    cmd = ["sudo", "/home/casper/.souk_readout_tools/daemon/install_timing_services.sh"]
    cmd.extend(["--phc-offset", str(args.phc_offset)])
    if args.preserve_config:
        cmd.append("--preserve-config")
    sys.exit(subprocess.call(cmd))


if __name__ == "__main__":
    main()
