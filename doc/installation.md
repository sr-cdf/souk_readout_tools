# Installation Guide

Detailed installation instructions for SOUK Readout Tools.

> **Most users will only need the client installation.** The server runs on the RFSoC board and should already be set up by whoever provisioned the hardware. If you are connecting to an existing RFSoC system, skip straight to [Client Installation](#client-installation).

---

## Contents

- [Prerequisites](#prerequisites)
- [Server Software Installation & Setup](#server-software-installation--setup)
  - [Writing the SD Card Image](#0-writing-the-sd-card-image)
  - [Enable Timing Services](#7-enable-timing-services-recommended)
- [Client Installation](#client-installation)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [Upgrading](#upgrading)
- [Uninstalling](#uninstalling)

---

## Prerequisites

### Client Machine

- Python >= 3.10 (tested on 3.10, 3.12)
- Network access to the RFSoC (e.g. direct Ethernet or routed connection)
- Git (for cloning the repository)

**Supported platforms:**
- Linux (tested on Ubuntu, Fedora)
- Windows (tested on Windows 10/11 with PowerShell)
- macOS (not tested, but should work)

### RFSoC Server

- Xilinx RFSoC board with CASPER Linux image
- Python 3.12 environment (the CASPER image includes a virtual environment at `/home/casper/py3.12-venv/`)
- `souk_mkid_readout` library from the [souk-firmware](https://github.com/realtimeradio/souk-firmware) repository
- Root/sudo access for `/dev/mem` and systemd operations

---

## Server Software Installation & Setup

This section covers setting up the readout server software on a new or freshly imaged RFSoC board. If your board is already running the server, skip to [Client Installation](#client-installation).

### 0. Writing the SD Card Image

Before first use, the RFSoC board needs a CASPER Linux image on its micro SD card. You will be provided with a compressed image file (e.g. `krc4700-souk-u24.zip`).

**Requirements:**
- A micro SD card (16 GB or larger recommended)
- A machine with an SD card reader
- Linux with `dd` and `parted` (or `fdisk` / `growpart`)

**Write the image:**

```bash
# Download and extract the image
unzip krc4700-souk-u24.zip            # produces krc4700-souk-u24.img

# Insert and identify the micro SD card (e.g. /dev/sdX or /dev/mmcblkN)
lsblk                      
```

> **Warning:** Double-check the device name carefully — `dd` will overwrite whatever device you point it at. Make sure it is the SD card, not your system disk.

```bash
# Write the image (replace /dev/mmcblkN with your SD card device)
sudo dd if=krc4700-souk-u24.img of=/dev/mmcblkN bs=4M status=progress conv=fsync
```

Wait for the write to complete. This may take several minutes depending on the card speed.

> **Note:** `dd` may report 100% written surprisingly quickly — this means the data has been buffered in memory, not necessarily written to the card. To monitor actual write progress, open a second terminal and run:
> ```bash
> watch -n 1 "grep -E '^(Dirty|Writeback):' /proc/meminfo"
> ```
> `Dirty` shows data waiting to be written to disk, `Writeback` shows data currently being transferred to hardware. When both drop near zero, the card is actually finished.

**Expand the root filesystem:**

The image is only ~7 GB and may not use the full capacity of the SD card. The card has two partitions — `BOOT` (partition 1) and `rootfs` (partition 2). Expand the `rootfs` partition to fill the available space so the system has room for runtime storage, logs, and calibration data.

The partition device names depend on your system — typically `mmcblk0p1`/`mmcblk0p2` for built-in card readers or `sdX1`/`sdX2` for USB adapters. Check `lsblk` to confirm.

```bash
# Verify the partition layout
sudo parted /dev/mmcblk0 print

# Resize the rootfs partition (partition 2) to fill the card
sudo parted /dev/mmcblk0 resizepart 2 100%

# Expand the filesystem to fill the resized partition
sudo e2fsck -f /dev/mmcblk0p2
sudo resize2fs /dev/mmcblk0p2
```

Alternatively, if `growpart` is available:
```bash
sudo growpart /dev/mmcblk0 2
sudo resize2fs /dev/mmcblk0p2
```

> Adjust device names as needed for your system (e.g. `/dev/sdb` and `/dev/sdb2` for a USB card reader).

Eject the card safely, insert it into the RFSoC board, and power on.

### 1. Processing System (Linux)

The RFSoC ARM processor should be running the latest CASPER Linux image with the `casper` user account and same password. The ethernet interface is configured with two addresses: one static `10.11.11.11/24` and one dynamic.

The standard way to access the OS is with SSH over Ethernet. Verify you can connect:

```bash
ssh casper@10.11.11.11
```

Network settings can be updated by modifying ```/etc/network/interfaces.d/end0``` and running `ifdown`/`ifup`.

A debug serial interface is also available via the micro-USB connector on the board. The single USB cable provides two serial ports — the first (`/dev/ttyUSB0`) is for board management, and the second (`/dev/ttyUSB1`) is the boot console and Linux terminal. This is useful for initial setup or recovering from network issues:

```bash
picocom -b 115200 /dev/ttyUSB1
```

The default Python environment on the board is `/home/casper/py3.12-venv/` and the bash shell should activate this virtual environment automatically on login, as indicated by the `(py3.12-venv) casper@host:~$` prompt. All server-side `python` and `pip install` commands should use this environment.

Note that the server requires root access for `/dev/mem` (FPGA register access). However, `sudo` does not inherit the virtual environment, so `sudo souk-readout-server ...` will fail with a "command not found" error. Use full paths to virtualenv binary instead: `sudo /home/casper/py3.12-venv/bin/souk-readout-server ...`. The systemd daemons handle this automatically (see [Enable the Daemons](#6-enable-the-daemons-recommended)).

The CASPER image includes the `souk_mkid_readout` firmware interface library from the `souk-firmware` repository. If you need to update it (e.g. after a firmware upgrade), follow the instructions at https://github.com/realtimeradio/souk-firmware.


### 2. Programmable Logic (FPGA)

The FPGA (programmable logic) does not require any manual setup. Three things happen automatically on the board:

- **Clock configuration:** The PL clocks (LMK/LMX chips) are configured on boot by the `krc-utils` service. The default source is the on-board 12.8 MHz oscillator. To use an external 10 MHz reference, set `firmware.clock_source: "external"` in the config and push it, or use `client.set_clock_source('external')`. The server also verifies that clocks are locked before programming the FPGA. See [clock_source.md](clock_source.md) for details.
- **PTP/NTP timing:** RFSoC wall-clock and telescope timestamp health are monitored separately by `ptp4l`, chrony, and `souk-timing-monitor`. See [timing.md](timing.md) for service installation, lab UTC offset caveats, and site checks.
- **Device tree overlay:** The Xilinx device tree overlay is applied by Linux at boot to expose the PL peripherals to the PS operating system.
- **Bitfile programming:** The FPGA bitfile is programmed automatically by the readout server when a client calls `ensure_ready()`. You should not need to program the FPGA manually.

However, the config files that we will use do need to reference the correct firmware config YAML, so it is worth knowing where the bitfiles live in case the firmware needs to be updated.

The `souk-firmware` repository supports multiple platforms and pipeline configurations; for SO:UK the dual-pipeline KRM firmware is used. The firmware config YAMLs are at:

```
/home/casper/souk-firmware/software/control_sw/config/
├── souk-dual-pipeline-krm.yaml       # dual-pipeline firmware config (typical)
├── souk-single-pipeline-krm.yaml     # single-pipeline firmware config
└── ...
```

Each firmware config YAML contains an `fpgfile` key that points to the actual bitfile:

```yaml
# Example firmware config YAML (souk-dual-pipeline-krm.yaml)
fpgfile: /home/casper/souk-firmware/firmware/src/souk_dual_pipeline_krm/outputs/souk_dual_pipeline_krm.fpg
```

This `fpgfile`is typically a symlink to the latest bitfile generated by the firmware build process. If switching to a different firmware version, both the `.fpg` and the `.dtbo` symlinks must be updated to point to the new build outputs.

If things get stuck and you do need to program the FPGA manually, you can use the client's `hard_reset()` method to reload the FPGA with the firmware linked in the config file; or use the `souk_mkid_readout` library directly.

The path to the firmware config YAML must go in your readout config file under `firmware.fw_config_file` (see [Create Configuration Files](#4-create-configuration-files) below).

### 3. Install souk_readout_tools server on the RFSoC

```bash
cd /home/casper
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
git submodule init && git submodule update
pip install .
```

The installer will auto-detect the Xilinx platform and install the server components. To install the server and the client at the same time (e.g. for scripted measurements on the board), use:
```bash
INSTALL_SERVER=true INSTALL_CLIENT=true pip install .
```

Verify:
```bash
souk-readout-server --help
souk-enable-timing --help
souk-timing-monitor --help
souk-test-timing-monitor --help
```

### 4. Create Configuration Files

Config files contain all the settings for a readout pipeline. Most of the config is used by clients to configure their measurements — for server setup, the key parameters are the network address, and TCP ports so the server can start and accept connections.

Use `ensure_pipeline_dirs` to create the directory structure for each pipeline, then `copy_template_config` to create a named config file for each. The config files are YAML and can be edited with any text editor:

```python
from souk_readout_tools.server.readout_server import ensure_pipeline_dirs
from souk_readout_tools.config_utils import copy_template_config

ensure_pipeline_dirs(0)
copy_template_config(
    destination='/home/casper/.souk_readout_tools/pipeline_0/config/p0_config.yaml',
    pipeline_id=0,
    config_id='souk_krm_pipeline0',
    created_by='myself',
    comments='Config for KRM4 dual-pipeline firmware p0',
)

ensure_pipeline_dirs(1)
copy_template_config(
    destination='/home/casper/.souk_readout_tools/pipeline_1/config/p1_config.yaml',
    pipeline_id=1,
    config_id='souk_krm_pipeline1',
    created_by='myself',
    comments='Config for KRM4 dual-pipeline firmware p1', 
)
```

This creates `~/.souk_readout_tools/` with the correct directory structure, template configs, example calibration files, and daemon scripts for both pipelines. The template configs are a reference — edit your named configs with your hardware-specific settings:

```
~/.souk_readout_tools/
├── daemon/
│   ├── install_systemd_service.sh
│   ├── remove_systemd_service.sh
│   ├── restart_systemd_service.sh
│   └── install_timing_services.sh
├── pipeline_0/
│   ├── config/
│   │   ├── template_config.yaml
│   │   ├── p0_config.yaml          ← edit this
│   │   └── default_config.lnk
│   ├── calibrations/
│   └── readout_server.service
└── pipeline_1/
    ├── config/
    │   ├── template_config.yaml
    │   ├── p1_config.yaml          ← edit this
    │   └── default_config.lnk
    ├── calibrations/
    └── readout_server.service
```

Edit each pipeline's config file with the appropriate hardware-specific settings:

```bash
nano ~/.souk_readout_tools/pipeline_0/config/p0_config.yaml
nano ~/.souk_readout_tools/pipeline_1/config/p1_config.yaml
```

`copy_template_config` automatically sets the pipeline-specific fields (`pipeline_id`, ports, RFDC tile/block mapping, and the SOUK mixerless-module RF channel default) based on the `pipeline_id` argument. For server setup, the parameters that matter are the address, ports, firmware config path, and any enabled RF peripheral defaults you want applied at startup:

| Parameter | Pipeline 0 | Pipeline 1 | Notes |
|-----------|-----------|-----------|-------|
| `rfsoc_host.address` | `10.11.11.11` | `10.11.11.11` | Needed for server |
| `rfsoc_host.request_port` | `10000` (auto) | `10001` (auto) | Needed for server |
| `rfsoc_host.stream_port` | `20000` (auto) | `20001` (auto) | Needed for server |
| `firmware.clock_source` | `internal` | `internal` | Board-level — must match between pipelines ([details](clock_source.md)) |
| `firmware.trigger_source_pin` | `0` | `0` | GPIO pin for external trigger input |
| `firmware.fw_config_file` | Path to firmware config YAML | Path to firmware config YAML | Needed for server |
| `firmware.pipeline_id` | `0` (auto) | `1` (auto) | Auto-set |
| `firmware.dac0_tile` / `block` | `0` / `0` (auto) | `1` / `0` (auto) | Auto-set |
| `firmware.adc_tile` / `block` | `2` / `0` (auto) | `3` / `0` (auto) | Auto-set |
| `rf_frontend.mixerless_module.rf_channel` | `0` (auto) | `1` (auto) | Auto-set from `pipeline_id`; selects mixerless-module attenuator/bypass channel |
| `cryostat.lna_bias.i2c.bus` | `0` | `0` | Linux SMBus for the LNA bias board; not pipeline-specific |

Parameters marked (auto) are set by `copy_template_config` — verify they match your firmware and RF frontend wiring. The RFDC tile/block mapping above is for v7.9+ dual-pipeline firmware. See [dual_pipeline.md](dual_pipeline.md) for full details. The remaining config sections (`cryostat`, `detector`, and RF frontend calibration values) are client-side concerns and can be configured later — see [Client Config Setup](#5-config-setup).

Point the default config link to your new file for each pipeline:

```bash
echo "p0_config.yaml" > ~/.souk_readout_tools/pipeline_0/config/default_config.lnk
echo "p1_config.yaml" > ~/.souk_readout_tools/pipeline_1/config/default_config.lnk
```

### 5. Test the Server

Start each server manually in a separate terminal/session to verify your configuration. The server runs in the foreground, so each pipeline needs its own terminal:

```bash
# Terminal 1
sudo /home/casper/py3.12-venv/bin/souk-readout-server ~/.souk_readout_tools/pipeline_0/config/p0_config.yaml
```

```bash
# Terminal 2
sudo /home/casper/py3.12-venv/bin/souk-readout-server ~/.souk_readout_tools/pipeline_1/config/p1_config.yaml
```

You can also use the `-p` flag to select a pipeline by index (the server uses `default_config.lnk` from that pipeline's directory):

```bash
sudo /home/casper/py3.12-venv/bin/souk-readout-server -p 0
```

On a freshly booted system, expect to see warnings and errors during startup — these are normal when the FPGA has not been programmed yet:

```
Could not find device: sys_board_id
Board is not programmed with valid firmware. Skipping block initialization
Warning: firmware needs programming
Warning: shared resources need initialising
Warning: pipeline resources need initialising
```

If all goes well, the server will print its listening address and port at the end:

```
Request server serving on ('0.0.0.0', 10000)
Stream server serving on ('0.0.0.0', 20000)
```

The FPGA will be programmed automatically when a client calls `ensure_ready()`. Once you have verified the server starts correctly, stop it with Ctrl-C before proceeding to step 6.

### 6. Enable the Daemons (Recommended)

To run the servers as systemd services that start on boot and auto-restart on crash:

```bash
souk-enable-daemons
```

This enables both pipelines. To enable a single pipeline, use `souk-enable-daemon -p 0` or `souk-enable-daemon -p 1`.

These commands call `sudo` internally and will prompt for a password if needed.

If these commands arent recognised, ensure the python environment is activated and the package is installed correctly. The `souk-enable-daemons` and `souk-enable-daemon` scripts should be available in the virtual environment's `bin` directory.

Check status:
```bash
systemctl status readout_server_0              # pipeline 0
systemctl status readout_server_1              # pipeline 1
```

Follow logs (Ctrl-C to stop):
```bash
sudo journalctl -f -xu readout_server_0                      # pipeline 0 only
sudo journalctl -f -xu readout_server_1                      # pipeline 1 only
sudo journalctl -f -xu readout_server_0 -u readout_server_1  # both pipelines
```

Restart:
```bash
souk-restart-daemons                          # both pipelines
souk-restart-daemon -p 0                      # pipeline 0 only
souk-restart-daemon -p 1                      # pipeline 1 only
```

Disable:
```bash
souk-disable-daemons                           # both pipelines
souk-disable-daemon -p 0                       # pipeline 0 only
souk-disable-daemon -p 1                       # pipeline 1 only
```

### 7. Enable Timing Services (Recommended)

The readout server reports timing health through `get_info("timing")`, but the
PTP/NTP timing stack runs as separate local services on the RFSoC:

- `ptp4l` disciplines the `end0` PTP Hardware Clock from the grandmaster.
- chrony disciplines Linux time from the PHC or NTP fallback.
- `souk-timing-monitor` polls `ptp4l` and chrony and exposes status on
  `/run/timing-monitor.sock`.

Install the OS packages:

```bash
sudo apt install linuxptp chrony
```

Install the packaged SOUK timing config and restart the services:

```bash
souk-enable-timing
```

For the current lab software grandmaster only, use the PHC offset workaround:

```bash
souk-enable-timing --offset=-37
```

Do not use `--offset=-37` with a production grandmaster that serves UTC
correctly.

Check the timing services:

```bash
systemctl status ptp4l chrony timing-monitor
souk-test-timing-monitor status
souk-test-timing-monitor stream 10
```

For firmware timestamp sync, the `end0` TSU/PHC PPS strobe must also be
enabled. At present this is provided by the `souk-firmware` helper:

```bash
sudo python3 ~/souk-firmware/software/rfsoc_scripts/ptp/run_strobe.py
```

See [timing.md](timing.md) for the full timing setup, standalone laptop
monitoring, site checks, lab UTC offset notes, and firmware sync caveats.

---

## Client Installation

### 1. Network Configuration

Network settings will depend on your local setup and/or institution policies. The RFSoC Ethernet interface is configured with two addresses by default:

- **Static:** `10.11.11.11/24` — for direct point-to-point connections. If using this, add a matching address on your client machine (e.g. `10.11.11.1/24`).
- **DHCP:** a second address obtained from a DHCP server, if one is available on the network.

Choose whichever is appropriate for your setup. If the board needs different network settings, these can be configured on the RFSoC by editing `/etc/network/interfaces.d/end0` — use the USB serial interface (see [Processing System](#1-processing-system-linux)) if network access is not yet available.

Verify connectivity:
```bash
ping <rfsoc_ip>
ssh casper@<rfsoc_ip>
```

Optional: add a hostname entry for convenience:
```
# /etc/hosts (Linux) or C:\Windows\System32\drivers\etc\hosts (Windows)
<rfsoc_ip> rfsoc
```

### 2. Create a Virtual Python Environment

**Linux / macOS:**
```bash
cd
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

### 3. Clone the Repository

```bash
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
git submodule init && git submodule update
```

### 4. Install the Package

```bash
pip install .
```

This is all most users need — the installer auto-detects that the platform isn't the RFSoC and installs only the client components.

**Advanced: client on the RFSoC.** If you need the client library directly on the RFSoC (e.g. for scripted measurements on the board), you can install both components in a single command:
```bash
ssh casper@rfsoc
cd souk_readout_tools
INSTALL_SERVER=true INSTALL_CLIENT=true pip install .
```
The GUI dependencies (PyQt5) are skipped automatically on the Xilinx platform since it has no display. Matplotlib will fall back to the non-interactive `Agg` backend when no display is available.

### 5. Config Setup

Config files live wherever you choose — keep them with your project or measurement data. The standard workflow is to maintain a local config file for a particular setup, connect with it, and push/pull changes to/from the firmware pipeline on the RFSoC.

> **Note:** Before creating a config, `cd` to the directory where you want to keep your project files and data — the config will be written to your current working directory:
> ```bash
> mkdir -p ~/my_mkid_project && cd ~/my_mkid_project
> ```

**Option A: Create a config from the template (standard)**

Generate a config from the bundled template, then edit it with your hardware-specific settings and push it to the server. The minimum you need to set is the **RFSoC IP address** (`rfsoc_host.address`) so the client knows where to connect:

```python
from souk_readout_tools.config_utils import copy_template_config

# Creates a config file with default settings for pipeline 0
copy_template_config(destination='my_config.yaml', pipeline_id=0,
                     config_id='krm4_pipeline0',
                     created_by='me',
                     comments='My first config')

# Edit my_config.yaml — at minimum, set rfsoc_host.address to your RFSoC IP.
# Then connect and push the config to the server:
from souk_readout_tools.client.readout_client import ReadoutClient
client = ReadoutClient(config_file='my_config.yaml')
client.push_config()
```

See [Getting Started — Preparing a Config File](getting_started.md#preparing-a-config-file) for which parameters to set.

**Option B: Pull a config from a running server (quick start)**

If the server is already configured and running (e.g. someone else set up the RFSoC), you can connect by address and pull its config. This is the easiest way to get started for first-time use — the config and any referenced calibration files are fetched in one step:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(address='10.11.11.11', request_port=10000)
client.pull_config(save_as='my_config.yaml')
# Config written to my_config.yaml, calibration files to calibrations/ alongside it
```

Specify the request port matching the pipeline you want to connect to (10000 for pipeline 0, 10001 for pipeline 1). Once saved, you have a local config file and can follow the standard workflow from then on:

```python
client = ReadoutClient(config_file='my_config.yaml')
```

### 6. Verify Installation

```bash
souk-connection-test --help
souk-mkid-finder-app --help
```

Test connectivity to a running server:
```bash
souk-connection-test -C path/to/my_config.yaml
```

Or connect by address:
```bash
souk-connection-test -a 10.11.11.11 -r 10000
``` 


### 7. End-to-End Verification

Once you have a config file and a running server, verify that commands round-trip correctly:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(config_file='my_config.yaml')
info = client.get_info()
print(info['server']['initialisation_level'])  # should be 'pipeline'
print(info['fpga'])                            # FPGA status and clock
print(info['timing']['summary']['state'])      # PTP/PHC/NTP timing state
print(info['tones']['count'])                  # number of active tones

# Quick health check
print(client.health_check())

# If RF peripherals are enabled in the config, verify hardware control:
print(info['rf_frontend'])
client.set_tx_attenuation(10.0)
print(client.get_info('rf_frontend')['tx_attenuation_db'])  # should be 10.0
```

If `get_info()` returns successfully, the client-server link is working and the firmware is accessible.

---

## Configuration Reference

### Config File Structure

The config file is YAML with five main sections:

| Section | Description | Required |
|---------|-------------|----------|
| `rfsoc_host` | IP address, ports, clock source, trigger/GPIO settings | Yes |
| `firmware` | Firmware config path, pipeline ID, RFDC mapping, DAC/ADC calibrations, default signal processing settings | Yes |
| `rf_frontend` | Attenuators, LO frequency, sideband, mixer/combiner losses, S21 measurements | If connected |
| `cryostat` | Channel info, S21 measurements, LNA settings, thermometry, optical setup | If connected |
| `detector` | Chip/channel IDs, resonance frequency and drive power file references | If connected |

The `rf_frontend`, `cryostat`, and `detector` sections each have a `connected: true/false` flag. When `false`, the section is ignored by the calibration chain.

Use `copy_template_config()` to generate a config file with all available parameters and their defaults. On the server, the template is also available at `~/.souk_readout_tools/pipeline_0/config/template_config.yaml`.

### Calibration Files

DAC power calibration maps the full-scale digital output to an absolute power level in dBm. This is used by `set_tone_powers()` / `get_tone_powers()` to convert between amplitude settings and physical power levels.

There are three ways to specify calibration values in the config file:

**Single value** (flat across frequency):
```yaml
firmware:
  dac0_dbfs_to_dbm: -6.0
```

**Frequency-power lookup pairs** (inline, linearly interpolated across frequency):
```yaml
firmware:
  dac0_dbfs_to_dbm: [[500e6, -5.9], [1000e6, -6.1], [1500e6, -6.5]]
```

**Path to a calibration file**:
```yaml
firmware:
  dac0_dbfs_to_dbm: 'calibrations/dac0.txt'
```

Calibration file format is two-column text (frequency in Hz, power in dBm at full-scale digital output):

```
# Frequency_Hz  Power_dBm
500e6  -5.9
1000e6 -6.1
1500e6 -6.5
```

Example calibration files are bundled with the package. On the server, they are copied to `~/.souk_readout_tools/pipeline_0/calibrations/` on first run.

Calibration files referenced in the config are transferred automatically: `push_config()` pushes local calibration files to the server, and `pull_config()` fetches them into memory (written to disk alongside the config when `save_config()` is called). You can also transfer individual files with `push_calibration()` and `pull_calibration()`.

Calibration file paths in the config can be absolute or relative — `push_config()` resolves them locally and rewrites them to the server-relative form. See the [calibration guide](calibration.md) for measurement conditions and file format details.

The same format and specification options apply to `dac1_dbfs_to_dbm` and `adc_dbm_to_dbfs`.

---

## Troubleshooting

### Client Issues

#### Connection refused

```
ConnectionRefusedError: [Errno 111] Connection refused
```

- Check that the server is running on the RFSoC (`ps aux | grep readout` or `sudo systemctl status readout_server_0 readout_server_1`)
- Verify the IP address and port in your config file match the server
- Check firewall settings on both machines
- Try `souk-connection-test -C path/to/config.yaml` for a quick diagnostic

#### Connection timeout

```
TimeoutError: [Errno 110] Connection timed out
```

- Verify network connectivity: `ping <rfsoc_ip>`
- Check that the client and RFSoC are on the same subnet
- Check Ethernet cable and link status

#### Module not found

```
ModuleNotFoundError: No module named 'souk_readout_tools'
```

- Ensure the virtual environment is activated
- Reinstall the package: `pip install .`

#### PyQt5 issues on Linux

If `souk-mkid-finder-app` fails to launch, you may need system Qt libraries:

```bash
# Ubuntu/Debian
sudo apt install python3-pyqt5

# Fedora/RedHat
sudo dnf install python3-qt5

```

#### Config file not found

You need to provide a config file or server address:

```python
# Connect by address and pull the config from the server:
client = ReadoutClient(address='10.11.11.11', request_port=10000)
client.pull_config(save_as='my_config.yaml')

# Or create a config from the template:
from souk_readout_tools.config_utils import copy_template_config
copy_template_config(destination='my_config.yaml', pipeline_id=0,
                     config_id="A new config", created_by="You")
```

### Server Issues

#### Permission denied for /dev/mem

```
PermissionError: [Errno 13] Permission denied: '/dev/mem'
```

- The server must run as root: `sudo souk-readout-server ...`
- The daemon runs as root automatically

#### souk_mkid_readout not found

```
ModuleNotFoundError: No module named 'souk_mkid_readout'
```

- The `souk_mkid_readout` library should already be installed on the CASPER image. If not, install from the souk-firmware repository (see [step 1](#1-rfsoc-initial-setup))
- Ensure you're using the correct Python environment: `sudo /home/casper/py3.12-venv/bin/python -c "import souk_mkid_readout"`

#### FPGA programming fails

- Check that the firmware config YAML path in your config (`firmware.fw_config_file`) is correct and the file exists on the RFSoC
- Check that the bitfile referenced by the firmware config YAML exists
- Try programming manually using `souk_mkid_readout` to isolate the issue

#### Server crashes on startup

- Check the journal for error messages: `sudo journalctl -xu readout_server_0 -u readout_server_1`
- Verify that no other server instance is using the same ports
- Ensure the config YAML is valid (no syntax errors)

#### Port already in use

```
OSError: [Errno 98] Address already in use
```

- Another server instance may be running on the same port
- Check for existing processes: `sudo lsof -i :<port>` or `ss -tlnp | grep <port>`
- Kill the existing process or use different ports in the config

---

## Upgrading

### Client

Ensure your virtual environment (e.g. `client_venv`) is activated before upgrading.

```bash
cd souk_readout_tools
git pull
git submodule update
pip install .
```

### Server

The virtual environment is automatically activated on login to the RFSoC.

```bash
ssh casper@rfsoc
cd /home/casper/souk_readout_tools
git pull
git submodule update
sudo /home/casper/py3.12-venv/bin/pip install .
souk-restart-daemons  # if running as daemons
```

---

## Uninstalling

### Client

```bash
pip uninstall souk_readout_tools
```

### Server

```bash
souk-disable-daemons  # if running as daemons
sudo /home/casper/py3.12-venv/bin/pip uninstall souk_readout_tools
```
