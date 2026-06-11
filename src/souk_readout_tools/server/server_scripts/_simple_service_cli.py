"""Shared entry point for the single-service systemd CLI wrappers.

Mirrors _daemon_cli.py (the pipeline readout-server wrappers) but for single,
non-pipeline services (the TSU strobe, the timing monitor). Each souk-<svc>-<action>
console script calls run(unit, action) here, which sudo-invokes the staged
manage_simple_service.sh.
"""
import subprocess
import sys

DAEMON_DIR = "/home/casper/.souk_readout_tools/daemon"
MANAGE_SCRIPT = f"{DAEMON_DIR}/manage_simple_service.sh"


def run(unit, action):
    """sudo-call manage_simple_service.sh <action> <unit>.

    Parameters
    ----------
    unit : str
        systemd unit name, e.g. "tsu-strobe.service".
    action : str
        install | remove | start | stop | restart | status.
    """
    if action == "install":
        # Staging the packaged unit/scripts into ~/.souk_readout_tools/daemon
        # is normally done by the readout server; ensure it on install too.
        try:
            from souk_readout_tools.server.readout_server import ensure_pipeline_dirs
            ensure_pipeline_dirs(0)
        except Exception as exc:
            print(f"Warning: could not stage daemon files: {exc}")
    cmd = ["sudo", MANAGE_SCRIPT, action, unit]
    sys.exit(subprocess.call(cmd))
