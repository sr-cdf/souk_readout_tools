"""CLI wrappers to control the TSU strobe systemd service (tsu-strobe.service).

Console scripts (registered in setup.py):
  souk-tsu-strobe-install   install + enable + start the service
  souk-tsu-strobe-start
  souk-tsu-strobe-stop
  souk-tsu-strobe-restart
  souk-tsu-strobe-status
  souk-tsu-strobe-remove    stop + disable + remove the unit

Each runs as a thin sudo wrapper around manage_simple_service.sh. Strobe status
(running / re-arms / tsu_sec) is also exposed via the timing monitor and
get_info('timing'); /run/tsu-strobe.status is the underlying JSON.
"""
from . import _simple_service_cli as cli

UNIT = "tsu-strobe.service"


def install():
    cli.run(UNIT, "install")


def start():
    cli.run(UNIT, "start")


def stop():
    cli.run(UNIT, "stop")


def restart():
    cli.run(UNIT, "restart")


def status():
    cli.run(UNIT, "status")


def remove():
    cli.run(UNIT, "remove")
