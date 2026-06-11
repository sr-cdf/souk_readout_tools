"""CLI wrappers to control the timing-monitor systemd service.

The timing monitor is installed by `souk-enable-timing` (which also installs
ptp4l/chrony config). These add symmetric start/stop/restart/status controls,
matching the strobe and readout-server daemon CLIs.

Console scripts (registered in setup.py):
  souk-timing-monitor-start
  souk-timing-monitor-stop
  souk-timing-monitor-restart
  souk-timing-monitor-status

(Install/enable stays with souk-enable-timing, which sets up the PTP/chrony
config the monitor reads; this module only does runtime control.)
"""
from . import _simple_service_cli as cli

UNIT = "timing-monitor.service"


def start():
    cli.run(UNIT, "start")


def stop():
    cli.run(UNIT, "stop")


def restart():
    cli.run(UNIT, "restart")


def status():
    cli.run(UNIT, "status")
