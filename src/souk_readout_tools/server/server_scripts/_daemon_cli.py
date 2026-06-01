import argparse
import subprocess
import sys


DAEMON_DIR = "/home/casper/.souk_readout_tools/daemon"
PIPELINE_CHOICES = (0, 1)


def prepare_daemon_files(pipelines):
    """Ensure the per-pipeline daemon directories exist for each id in
    ``pipelines`` (best-effort; warns rather than raises on failure)."""
    try:
        from souk_readout_tools.server.readout_server import ensure_pipeline_dirs

        for pipeline in pipelines:
            ensure_pipeline_dirs(pipeline)
    except Exception as exc:
        print(f"Warning: could not prepare ~/.souk_readout_tools daemon files: {exc}")


def main(script_name, action, default_pipelines, default_label, help_suffix):
    """Shared entry point for the systemd daemon CLI wrappers.

    Parameters
    ----------
    script_name : str
        Name of the calling console script (used in help text).
    action : str
        The systemctl action this wrapper performs (e.g. ``'start'``,
        ``'stop'``, ``'status'``).
    default_pipelines : sequence of int
        Pipeline ids targeted when ``--pipeline`` is not given.
    default_label : str
        Default systemd unit label.
    help_suffix : str
        Extra text appended to the CLI help.
    """
    parser = argparse.ArgumentParser(
        description=f"{action.capitalize()} SOUK readout server systemd daemon(s)."
    )
    parser.add_argument(
        "-p",
        "--pipeline",
        type=int,
        nargs="+",
        default=list(default_pipelines),
        choices=PIPELINE_CHOICES,
        help=(
            f"Pipeline ID(s) to {action} (default: {default_label}). "
            f"{help_suffix}"
        ),
    )
    args = parser.parse_args()

    prepare_daemon_files(args.pipeline)

    cmd = ["sudo", f"{DAEMON_DIR}/{script_name}"]
    cmd.extend(str(pipeline) for pipeline in args.pipeline)
    sys.exit(subprocess.call(cmd))
