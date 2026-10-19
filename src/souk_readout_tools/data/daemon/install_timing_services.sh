#!/bin/bash

# Installs SOUK timing service units and default config files.
#
# Usage:
#   sudo ./install_timing_services.sh                    # install configs and services
#   sudo ./install_timing_services.sh --phc-offset=0     # ARB-timescale GM only
#   sudo ./install_timing_services.sh --preserve-config  # update services only

set -e

BASE_DIR="/home/casper/.souk_readout_tools"
TIMING_DIR="${BASE_DIR}/timing"
SERVICE_DIR="/etc/systemd/system"
PRESERVE_CONFIG=0
PHC_OFFSET=-37

while [[ $# -gt 0 ]]; do
    case "$1" in
        --preserve-config)
            PRESERVE_CONFIG=1
            shift
            ;;
        --phc-offset|--offset)
            if [[ $# -lt 2 ]]; then
                echo "Error: $1 requires an offset value in seconds"
                exit 1
            fi
            PHC_OFFSET="$2"
            shift 2
            ;;
        --phc-offset=*|--offset=*)
            PHC_OFFSET="${1#*=}"
            shift
            ;;
        --help|-h)
            echo "Usage: sudo $0 [--phc-offset SECONDS] [--preserve-config]"
            echo "  Default: install packaged /etc timing config files and service units."
            echo "  Existing files are backed up before replacement."
            echo "  --phc-offset, --offset: chrony PHC refclock offset in seconds."
            echo "    Default is -37 (PHC on TAI). Use 0 only for an ARB-timescale GM (ptpTimescale 0)."
            echo "  --preserve-config: keep existing /etc timing config files and update service units only."
            exit 0
            ;;
        *)
            echo "Error: unknown argument '$1'"
            echo "Usage: sudo $0 [--phc-offset SECONDS] [--preserve-config]"
            exit 1
            ;;
    esac
done

if [[ "$EUID" -ne 0 ]]; then
    echo "Error: This script must be run as root."
    exit 1
fi

backup_existing() {
    local dst="$1"
    local stamp
    stamp="$(date +%Y%m%d-%H%M%S)"
    cp "$dst" "${dst}.bak.${stamp}"
    echo "Backed up existing $dst to ${dst}.bak.${stamp}"
}

install_config() {
    local src="$1"
    local dst="$2"
    local mode="$3"

    if [[ ! -f "$src" ]]; then
        echo "Error: missing source file $src"
        exit 1
    fi

    if [[ -f "$dst" && "$PRESERVE_CONFIG" -eq 1 ]]; then
        echo "Keeping existing $dst (--preserve-config)"
        return
    fi

    if [[ -f "$dst" && ! -L "$dst" ]]; then
        backup_existing "$dst"
    fi

    echo "Installing $dst"
    mkdir -p "$(dirname "$dst")"
    cp "$src" "$dst"
    chmod "$mode" "$dst"
}

apply_phc_offset() {
    local dst="$1"

    if [[ "$PRESERVE_CONFIG" -eq 1 ]]; then
        echo "Preserving existing $dst; PHC offset not changed"
        return
    fi

    if ! [[ "$PHC_OFFSET" =~ ^[-+]?[0-9]+([.][0-9]+)?$ ]]; then
        echo "Error: invalid PHC offset '$PHC_OFFSET'"
        exit 1
    fi

    sed -i -E \
        "s|^(refclock[[:space:]]+PHC[[:space:]]+/dev/ptp[^[:space:]]*[[:space:]].*)offset[[:space:]]+[^[:space:]]+|\\1offset ${PHC_OFFSET}|" \
        "$dst"
    echo "Set chrony PHC refclock offset to ${PHC_OFFSET} seconds in $dst"
}

install_service() {
    local src="$1"
    local dst="$2"
    local mode="$3"

    if [[ ! -f "$src" ]]; then
        echo "Error: missing source file $src"
        exit 1
    fi

    if [[ -f "$dst" ]] && cmp -s "$src" "$dst"; then
        echo "Service unit already up to date: $dst"
        chmod "$mode" "$dst"
        return
    fi

    if [[ -f "$dst" && ! -L "$dst" ]]; then
        backup_existing "$dst"
    fi

    echo "Installing service unit $dst"
    mkdir -p "$(dirname "$dst")"
    cp "$src" "$dst"
    chmod "$mode" "$dst"
}

# A stable /dev/ptp-end0 for end0's PHC, which the kernel renumbers on every
# ifdown/ifup. Always installed: chrony's config and the monitor rely on it.
install_service "${TIMING_DIR}/90-ptp-end0.rules" "/etc/udev/rules.d/90-ptp-end0.rules" 644
udevadm control --reload-rules
udevadm trigger --subsystem-match=ptp --action=add
udevadm settle
install_config "${TIMING_DIR}/ptp4l.conf" "/etc/linuxptp/ptp4l.conf" 644
install_config "${TIMING_DIR}/ptp-phc.conf" "/etc/chrony/conf.d/ptp-phc.conf" 644
apply_phc_offset "/etc/chrony/conf.d/ptp-phc.conf"
install_service "${BASE_DIR}/daemon/ptp4l.service" "${SERVICE_DIR}/ptp4l.service" 644
# Hold chrony until ptp4l has set the PHC, or chrony can sample the stale boot-time
# PHC, reject it as a falseticker and stay on NTP (see the file's comments).
install_service "${TIMING_DIR}/10-wait-for-ptp4l.conf" "${SERVICE_DIR}/chrony.service.d/10-wait-for-ptp4l.conf" 644
install_service "${BASE_DIR}/daemon/timing-monitor.service" "${SERVICE_DIR}/timing-monitor.service" 644

systemctl daemon-reload
systemctl enable ptp4l.service
systemctl enable timing-monitor.service

systemctl restart ptp4l.service
systemctl restart chrony.service
systemctl restart timing-monitor.service

systemctl status ptp4l.service timing-monitor.service --no-pager
