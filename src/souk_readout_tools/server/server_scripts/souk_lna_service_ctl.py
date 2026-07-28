"""CLI wrappers to control the LNA bias systemd service (lna-service.service).

Console scripts (registered in setup.py):
  souk-lna-service-install   install + enable + start the service
  souk-lna-service-start
  souk-lna-service-stop
  souk-lna-service-restart
  souk-lna-service-status
  souk-lna-service-remove    stop + disable + remove the unit

Install this on the one RFSoC per telescope that is wired to the LNA bias
board. Every other machine reaches it over the network; point them at it with
``lna_service.host`` in ~/.souk_readout_tools/site.yaml.

Each command is a thin sudo wrapper around manage_simple_service.sh.
"""
from . import _simple_service_cli as cli

UNIT = "lna-service.service"


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
