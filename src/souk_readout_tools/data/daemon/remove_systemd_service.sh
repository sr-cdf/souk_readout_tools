#!/bin/bash

# remove_systemd_service.sh
# Stops, disables, and removes readout server systemd service(s).
#
# Usage:
#   sudo ./remove_systemd_service.sh           # pipeline 0 only (default)
#   sudo ./remove_systemd_service.sh 0         # pipeline 0 only
#   sudo ./remove_systemd_service.sh 1         # pipeline 1 only
#   sudo ./remove_systemd_service.sh 0 1       # both pipelines

set -e

# === Configuration ===
SERVICE_DIR="/etc/systemd/system"
# === End of Configuration ===

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run as root."
    echo "Usage: sudo $0 [pipeline_id ...]"
    exit 1
fi

# Default to pipeline 0 if no arguments given
if [[ "$#" -eq 0 ]]; then
    PIPELINES=(0)
else
    PIPELINES=("$@")
fi

for PID in "${PIPELINES[@]}"; do
    if [[ "$PID" != "0" && "$PID" != "1" ]]; then
        echo "Error: Invalid pipeline ID '$PID'. Must be 0 or 1."
        exit 1
    fi

    SERVICE_NAME="readout_server_${PID}.service"
    SERVICE_FILE_DEST="${SERVICE_DIR}/${SERVICE_NAME}"

    if [[ ! -f "$SERVICE_FILE_DEST" ]]; then
        echo "Service '${SERVICE_NAME}' is not installed. Skipping pipeline ${PID}."
        continue
    fi

    echo "=== Removing service for pipeline ${PID} ==="

    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "Stopping service '${SERVICE_NAME}'..."
        systemctl stop "$SERVICE_NAME"
    else
        echo "Service '${SERVICE_NAME}' is not running."
    fi

    if systemctl is-enabled --quiet "$SERVICE_NAME"; then
        echo "Disabling service '${SERVICE_NAME}'..."
        systemctl disable "$SERVICE_NAME"
    else
        echo "Service '${SERVICE_NAME}' is not enabled."
    fi

    echo "Removing service file '${SERVICE_FILE_DEST}'..."
    rm -f "$SERVICE_FILE_DEST"

    systemctl daemon-reload

    if systemctl is-failed --quiet "$SERVICE_NAME" 2>/dev/null; then
        systemctl reset-failed "$SERVICE_NAME"
    fi

    echo "Service '${SERVICE_NAME}' removed."
    echo ""
done

echo "Done."
