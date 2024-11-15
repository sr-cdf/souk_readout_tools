#!/bin/bash

# install_systemd_service.sh
# This script copies a predefined systemd service file to /etc/systemd/system/,
# reloads the systemd daemon, enables the service, and starts it.

# Exit immediately if a command exits with a non-zero status
set -e

# === Configuration ===

# Hardcoded service file name and source path
SERVICE_NAME="readout_server.service"
SERVICE_FILE_SRC="/home/casper/.souk_readout_tools/daemon/${SERVICE_NAME}"  # <-- Update this path

# Destination directory for systemd service files
SERVICE_DIR="/etc/systemd/system"
SERVICE_FILE_DEST="${SERVICE_DIR}/${SERVICE_NAME}"

# === End of Configuration ===

# Function to display usage information (optional, since no arguments are used)
usage() {
    echo "Usage: sudo $0"
    echo "This script does not accept any arguments."
    exit 1
}

# Check if the script is run as root
if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run as root."
    usage
fi

# Check if the service file exists and is a regular file
if [[ ! -f "$SERVICE_FILE_SRC" ]]; then
    echo "Error: Service file '$SERVICE_FILE_SRC' does not exist or is not a regular file."
    exit 1
fi

echo "Copying service file to $SERVICE_DIR..."
cp "$SERVICE_FILE_SRC" "$SERVICE_FILE_DEST"
echo "Service file copied to $SERVICE_FILE_DEST."

# Set appropriate permissions (optional but recommended)
chmod 644 "$SERVICE_FILE_DEST"
echo "Set permissions to 644 for $SERVICE_FILE_DEST."

# Reload systemd to recognize the new service
echo "Reloading systemd daemon..."
systemctl daemon-reload
echo "Systemd daemon reloaded."

# Enable the service to start on boot
echo "Enabling service '$SERVICE_NAME'..."
systemctl enable "$SERVICE_NAME"
echo "Service '$SERVICE_NAME' enabled to start on boot."

# Optionally, start the service immediately
echo "Starting service '$SERVICE_NAME'..."
systemctl start "$SERVICE_NAME"
echo "Service '$SERVICE_NAME' started."

# Check the status of the service
echo "Checking the status of '$SERVICE_NAME'..."
systemctl status "$SERVICE_NAME" --no-pager

echo "Service installation and setup completed successfully."

