#!/bin/bash

# remove_systemd_service.sh
# This script stops and disables a predefined systemd service,
# optionally removes its service file from /etc/systemd/system/,
# and reloads the systemd daemon.

# Exit immediately if a command exits with a non-zero status
set -e

# === Configuration ===

# Hardcoded service file name
SERVICE_NAME="readout_server.service"

# Destination directory for systemd service files
SERVICE_DIR="/etc/systemd/system"
SERVICE_FILE_DEST="${SERVICE_DIR}/${SERVICE_NAME}"

# === End of Configuration ===

# Function to display usage information
usage() {
    echo "Usage: sudo $0"
    echo "This script does not accept any arguments."
    echo "It stops and disables the hardcoded service: $SERVICE_NAME"
    exit 1
}

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run as root."
    usage
fi

# Ensure no arguments are provided
if [[ "$#" -ne 0 ]]; then
    echo "Error: Invalid number of arguments."
    usage
fi

# Check if the service file exists
if [[ ! -f "$SERVICE_FILE_DEST" ]]; then
    echo "Warning: Service file '$SERVICE_FILE_DEST' does not exist."
    echo "It may have already been removed."
else
    echo "=== Removing Service: $SERVICE_NAME ==="

    # Stop the service if it's running
    if systemctl is-active --quiet "$SERVICE_NAME"; then
        echo "Stopping service '$SERVICE_NAME'..."
        systemctl stop "$SERVICE_NAME"
        echo "Service '$SERVICE_NAME' stopped."
    else
        echo "Service '$SERVICE_NAME' is not running."
    fi

    # Disable the service to prevent it from starting at boot
    if systemctl is-enabled --quiet "$SERVICE_NAME"; then
        echo "Disabling service '$SERVICE_NAME'..."
        systemctl disable "$SERVICE_NAME"
        echo "Service '$SERVICE_NAME' disabled."
    else
        echo "Service '$SERVICE_NAME' is not enabled."
    fi

    # Optionally, remove the service file
    echo "Removing service file '$SERVICE_FILE_DEST'..."
    rm -f "$SERVICE_FILE_DEST"
    echo "Service file '$SERVICE_FILE_DEST' removed."

    # Reload systemd to apply changes
    echo "Reloading systemd daemon..."
    systemctl daemon-reload
    echo "Systemd daemon reloaded."

    # Optionally, reset failed state if any
    if systemctl is-failed --quiet "$SERVICE_NAME"; then
        echo "Resetting failed state for '$SERVICE_NAME'..."
        systemctl reset-failed "$SERVICE_NAME"
        echo "Failed state reset."
    fi

    echo "Service '$SERVICE_NAME' has been stopped, disabled, and removed successfully."
fi

echo "Operation completed."

