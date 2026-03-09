# Installation Guide

Detailed installation instructions for SOUK Readout Tools.

> **Most users will only need the client installation.** The server runs on the RFSoC board and should already be set up by whoever provisioned the hardware. If you are connecting to an existing RFSoC system, skip straight to [Client Installation](#client-installation).

---

## Contents

- [Prerequisites](#prerequisites)
- [Client Installation](#client-installation)
- [Server Installation](#server-installation)
- [Post-Installation Setup](#post-installation-setup)
- [Troubleshooting](#troubleshooting)

---

## Prerequisites

### Client Machine

- Python >= 3.8 (tested on 3.10, 3.12)
- Network access to the RFSoC (e.g. direct Ethernet or routed connection)
- Git (for cloning the repository)

**Supported platforms:**
- Linux (tested on Ubuntu, Fedora)
- Windows (tested on Windows 10/11 with PowerShell)
- macOS (not tested, but should work)

### RFSoC Server

- Xilinx RFSoC board with CASPER Linux image
- Python 3.8 (required for compatibility)
- `souk_mkid_readout` library from the [souk-firmware](https://github.com/realtimeradio/souk-firmware) repository
- Root/sudo access for `/dev/mem` and systemd operations

---

## Client Installation

### 1. Network Configuration

Ensure the client machine can reach the RFSoC. The default RFSoC IP is `10.11.11.11/24`.

**Linux:**
```bash
# Set static IP on the interface connected to the RFSoC
sudo ip addr add 10.11.11.1/24 dev eth0
```

**Windows:**
```
Control Panel → Network and Sharing Center → Change adapter settings
→ Right-click adapter → Properties → IPv4 → Use the following IP address
→ IP: 10.11.11.1, Subnet: 255.255.255.0
```

Verify connectivity:
```bash
ping 10.11.11.11
ssh casper@10.11.11.11
```

Optional: add a hostname entry:
```
# /etc/hosts (Linux) or C:\Windows\System32\drivers\etc\hosts (Windows)
10.11.11.11 rfsoc
```

### 2. Clone the Repository

```bash
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
```

### 3. Create a Virtual Environment

**Linux / macOS:**
```bash
python3 -m venv client_venv
source ./client_venv/bin/activate
```

**Windows PowerShell:**
```powershell
python -m venv client_venv
.\client_venv\Scripts\Activate.ps1

# If you get an execution policy error:
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

### 4. Install the Package

```bash
pip install .
```

The installer auto-detects the platform and decides which components to install. To force client installation alongside the server on the RFSoC:
```bash
INSTALL_CLIENT=true INSTALL_SERVER=true pip install .
```

### 5. Verify Installation

```bash
souk-connection-test --help
souk-mkid-finder-app --help
```

---

## Server Installation

### 1. RFSoC Initial Setup

<!-- TODO: Add instructions for initial RFSoC board setup -->
<!-- - CASPER image installation -->
<!-- - Network configuration -->
<!-- - User account setup -->

*Placeholder: Initial RFSoC board setup instructions.*

### 2. Install souk_mkid_readout

The server depends on the `souk_mkid_readout` firmware interface library.

```bash
ssh casper@rfsoc
cd /home/casper/src
git clone https://github.com/realtimeradio/souk-firmware
cd souk-firmware/software/control_sw
sudo /home/casper/py38venv/bin/pip install .
```

<!-- TODO: Add version compatibility notes -->
<!-- TODO: Add firmware bitfile installation instructions -->

*Placeholder: Firmware version compatibility and bitfile installation.*

### 3. Install souk_readout_tools

```bash
cd /home/casper/src
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
sudo /home/casper/py38venv/bin/pip install .
```

The installer will auto-detect the Xilinx platform and install the server components. To explicitly install the server (e.g. if auto-detection fails):
```bash
INSTALL_SERVER=true pip install .
```

### 4. Verify Installation

```bash
souk-readout-server --help
```

### 5. Configure the Server

<!-- TODO: Add detailed config file setup instructions -->
<!-- - Template config location -->
<!-- - Required parameters to modify -->
<!-- - Firmware config file path -->
<!-- - RFDC tile/block mapping -->

*Placeholder: Server configuration instructions.*

### 6. Enable the Daemon (Optional)

To run the server as a systemd service:

```bash
sudo souk-enable-daemon
```

Check status:
```bash
sudo systemctl status readout_server
sudo journalctl -f -xu readout_server
```

Disable:
```bash
sudo souk-disable-daemon
```

---

## Post-Installation Setup

### First Run

On first import, the package creates `~/.souk_readout_tools/` and copies template files:

```
~/.souk_readout_tools/
├── pipeline_0/
│   ├── config/
│   │   └── template_config.yaml
│   ├── calibrations/
│   └── tmp/
└── pipeline_1/
    └── ...
```

### Configuration

<!-- TODO: Add detailed configuration guide -->
<!-- - Config file sections -->
<!-- - Required vs optional parameters -->
<!-- - Example configurations for common setups -->

*Placeholder: Configuration guide.*

### Calibration Files

<!-- TODO: Add calibration file setup instructions -->
<!-- - DAC calibration -->
<!-- - RF frontend calibration -->
<!-- - Cryostat S21 measurements -->

*Placeholder: Calibration file setup.*

---

## Troubleshooting

### Client Issues

#### Connection refused

```
ConnectionRefusedError: [Errno 111] Connection refused
```

- Check that the server is running on the RFSoC
- Verify the IP address and port in your config file
- Check firewall settings

#### Module not found

```
ModuleNotFoundError: No module named 'souk_readout_tools'
```

- Ensure the virtual environment is activated
- Reinstall the package: `pip install .`

<!-- TODO: Add more client troubleshooting -->

*Placeholder: Additional client troubleshooting.*

### Server Issues

#### Permission denied for /dev/mem

```
PermissionError: [Errno 13] Permission denied: '/dev/mem'
```

- Run the server with sudo: `sudo souk-readout-server ...`

#### souk_mkid_readout not found

```
ModuleNotFoundError: No module named 'souk_mkid_readout'
```

- Install the firmware library from souk-firmware repository
- Ensure you're using the correct Python environment

<!-- TODO: Add more server troubleshooting -->

*Placeholder: Additional server troubleshooting.*

---

## Upgrading

### Client

```bash
cd souk_readout_tools
git pull
pip install .
```

### Server

```bash
ssh casper@rfsoc
cd /home/casper/src/souk_readout_tools
git pull
sudo /home/casper/py38venv/bin/pip install .
sudo systemctl restart readout_server  # if running as daemon
```

---

## Uninstalling

### Client

```bash
pip uninstall souk_readout_tools
rm -rf ~/.souk_readout_tools  # optional: remove config/data
```

### Server

```bash
sudo souk-disable-daemon  # if running as daemon
sudo pip uninstall souk_readout_tools
```
