#!/bin/bash

# manage_simple_service.sh
# Install / control a single (non-pipeline) SOUK systemd service by unit name.
# Used by the souk-tsu-strobe-* and souk-timing-monitor-* CLI wrappers.
#
# Usage:
#   sudo ./manage_simple_service.sh <action> <unit>
#     action: install | remove | start | stop | restart | status
#     unit:   e.g. tsu-strobe.service
#
# 'install' copies the packaged unit from ~/.souk_readout_tools/daemon/<unit>
# into /etc/systemd/system/, daemon-reloads, enables and starts it. The other
# actions are thin systemctl wrappers.

set -e

BASE_DIR="/home/casper/.souk_readout_tools"
DAEMON_DIR="${BASE_DIR}/daemon"
SERVICE_DIR="/etc/systemd/system"

if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run as root."
    echo "Usage: sudo $0 <install|remove|start|stop|restart|status> <unit>"
    exit 1
fi

ACTION="$1"
UNIT="$2"

if [[ -z "$ACTION" || -z "$UNIT" ]]; then
    echo "Usage: sudo $0 <install|remove|start|stop|restart|status> <unit>"
    exit 1
fi

case "$ACTION" in
    install)
        SRC="${DAEMON_DIR}/${UNIT}"
        DEST="${SERVICE_DIR}/${UNIT}"
        if [[ ! -f "$SRC" ]]; then
            echo "Error: packaged unit '$SRC' not found."
            echo "Run the readout server once (or ensure_pipeline_dirs) to stage daemon files."
            exit 1
        fi
        echo "Installing ${UNIT} -> ${DEST}..."
        cp "$SRC" "$DEST"
        chmod 644 "$DEST"
        systemctl daemon-reload
        systemctl enable "$UNIT"
        systemctl start "$UNIT"
        systemctl status "$UNIT" --no-pager || true
        ;;
    remove)
        echo "Stopping and disabling ${UNIT}..."
        systemctl stop "$UNIT" || true
        systemctl disable "$UNIT" || true
        rm -f "${SERVICE_DIR}/${UNIT}"
        systemctl daemon-reload
        echo "Removed ${UNIT}."
        ;;
    start|stop|restart)
        echo "${ACTION}ing ${UNIT}..."
        systemctl "$ACTION" "$UNIT"
        systemctl status "$UNIT" --no-pager || true
        ;;
    status)
        systemctl status "$UNIT" --no-pager || true
        ;;
    *)
        echo "Error: unknown action '$ACTION'."
        echo "Usage: sudo $0 <install|remove|start|stop|restart|status> <unit>"
        exit 1
        ;;
esac
