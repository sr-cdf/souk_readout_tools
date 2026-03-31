#!/bin/bash

# install_systemd_service.sh
# Installs readout server systemd service(s) for the specified pipeline(s).
#
# Usage:
#   sudo ./install_systemd_service.sh           # pipeline 0 only (default)
#   sudo ./install_systemd_service.sh 0         # pipeline 0 only
#   sudo ./install_systemd_service.sh 1         # pipeline 1 only
#   sudo ./install_systemd_service.sh 0 1       # both pipelines

set -e

# === Configuration ===
BASE_DIR="/home/casper/.souk_readout_tools"
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

    # Service file lives in the pipeline directory
    SERVICE_FILE_SRC="${BASE_DIR}/pipeline_${PID}/readout_server.service"
    SERVICE_NAME="readout_server_${PID}.service"
    SERVICE_FILE_DEST="${SERVICE_DIR}/${SERVICE_NAME}"

    if [[ ! -f "$SERVICE_FILE_SRC" ]]; then
        echo "Error: Service file '$SERVICE_FILE_SRC' does not exist."
        echo "Run 'ensure_pipeline_dirs(${PID})' to generate it, e.g.:"
        echo "  python -c \"from souk_readout_tools.server.readout_server import ensure_pipeline_dirs; ensure_pipeline_dirs(${PID})\""
        exit 1
    fi

    echo "=== Installing service for pipeline ${PID} ==="

    echo "Copying to ${SERVICE_FILE_DEST}..."
    cp "$SERVICE_FILE_SRC" "$SERVICE_FILE_DEST"
    chmod 644 "$SERVICE_FILE_DEST"

    systemctl daemon-reload

    echo "Enabling service '${SERVICE_NAME}'..."
    systemctl enable "$SERVICE_NAME"

    echo "Starting service '${SERVICE_NAME}'..."
    systemctl start "$SERVICE_NAME"

    systemctl status "$SERVICE_NAME" --no-pager
    echo ""
done

echo "Done."
