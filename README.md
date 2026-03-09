# SOUK Readout Tools

Tools for operating the RFSoC based MKID readout system for Simons Observatory: UK.

Uses a client-server architecture: the server runs on the RFSoC ARM processor (PS) and controls the FPGA programmable logic (PL) via the [souk_mkid_readout](https://github.com/realtimeradio/souk-firmware) firmware interface. Clients connect remotely over TCP to perform measurements and acquire data.

Supports up to two independent readout pipelines on a single RFSoC board.

## Quick Start

Assuming the server is running on the RFSoC and the client machine is configured with network access, you can install the client tools and connect as follows:

```bash
# Clone and install (client)
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
python3 -m venv client_venv && source ./client_venv/bin/activate
pip install .
```

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(config_file='path/to/my_config.yaml')
client.ensure_ready()

# Set tones and acquire data
client.set_tone_frequencies([0.800e9, 1.500e9])
raw = client.get_samples(500)
data = client.parse_samples(raw, num_tones=2)
```

See the [Getting Started Guide](doc/getting_started.md) for full details.

## Key Features

- **Tone management** - Set readout tone frequencies, amplitudes and phases with full calibration chain support (DAC through to cryostat)
- **Data acquisition** - Discrete samples, continuous streaming, and triggered streaming modes
- **Frequency sweeping** - Targeted sweeps around tone centers, and wideband sweeps covering the full RF bandwidth
- **Resonance finding** - Built-in MKID resonance detection with configurable peak-finding algorithms, plus an interactive PyQt5 GUI (`souk-mkid-finder-app`)
- **Retuning** - Automated sweep-and-retune to track resonance frequency drift
- **Dual-pipeline** - Two independent readout pipelines on one RFSoC, with 3-level initialisation to avoid cross-pipeline disruption
- **VACC multitone** (v7.9+ firmware) - Multiple tones per FFT bin via the vector accumulator
- **Power optimisation** - Automatic saturation detection and TX/RX power optimisation
- **Configuration management** - YAML-based config with `push_config()`/`pull_config()` for seamless client-server sync

## Documentation

| Document | Description |
|----------|-------------|
| [Getting Started](doc/getting_started.md) | Installation, configuration, usage guide, and feature list |
| [Dual Pipeline](doc/dual_pipeline.md) | Dual-pipeline setup, initialisation, and configuration |
| [v7.9 Multitone Notes](doc/v79-multitone-notes.md) | VACC multitone feature development notes |

## CLI Tools

**Client** (installed on your machine):

| Command | Description |
|---------|-------------|
| `souk-connection-test` | Test connectivity to the readout server |
| `souk-wideband_sweep` | Perform a wideband frequency sweep |
| `souk-mkid-finder-app` | Launch the MKID resonance finder GUI |

**Server** (installed on the RFSoC):

| Command | Description |
|---------|-------------|
| `souk-readout-server` | Start the readout server (`-p` for pipeline ID) |
| `souk-enable-daemon` | Enable the server as a systemd service |
| `souk-disable-daemon` | Disable the server systemd service |

## Configuration

YAML-based configuration with sections for `rfsoc_host`, `firmware`, `rf_frontend`, `cryostat`, and `detector`. A template config is bundled with the package and copied to `~/.souk_readout_tools/` on first run. Per-pipeline config and data are stored in separate subdirectories (`pipeline_0/`, `pipeline_1/`).

## Server Daemon

The readout server can run as a systemd service for automatic start on boot and restart on crash:

```bash
souk-enable-daemon    # Enable and start the service
souk-disable-daemon   # Stop and disable the service
```

Monitor logs with:
```bash
sudo journalctl -f -xu readout_server
```

## Installation Details

The installer auto-detects the platform:
- **Xilinx/RFSoC kernel**: Installs server components
- **Everything else**: Installs client components

Override with environment variables: `INSTALL_SERVER=true` or `INSTALL_CLIENT=true`.

Dependencies: numpy, scipy, matplotlib, pyyaml, ipython. Client additionally requires PyQt5. Server requires `souk_mkid_readout` from the [souk-firmware](https://github.com/realtimeradio/souk-firmware) repository.

Client requires Python >= 3.8. Tested on Linux (Python 3.10, 3.12) and Windows (Python 3.12).

Server requires Python == 3.8 and the `souk_mkid_readout` library. Tested on Xilinx/RFSoC Linux (Python 3.8)
