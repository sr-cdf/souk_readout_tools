#!/bin/bash

# restart_systemd_service.sh
# Restarts readout server systemd service(s) for the specified pipeline(s).
#
# Usage:
#   sudo ./restart_systemd_service.sh           # pipeline 0 only (default)
#   sudo ./restart_systemd_service.sh 0         # pipeline 0 only
#   sudo ./restart_systemd_service.sh 1         # pipeline 1 only
#   sudo ./restart_systemd_service.sh 0 1       # both pipelines

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
        echo "Service '${SERVICE_NAME}' is not installed. Run souk-enable-daemon first."
        exit 1
    fi

    echo "=== Restarting service for pipeline ${PID} ==="
    systemctl restart "$SERVICE_NAME"
    echo "Service '${SERVICE_NAME}' restarted."
    echo ""
done

echo "Done."
