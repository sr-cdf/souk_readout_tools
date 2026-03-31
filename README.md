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

Installation is conditional and comes from `setup.py`:

- On Xilinx platforms, the package installs server components.
- On non-Xilinx platforms, the package installs client components.
- `INSTALL_SERVER=true` and/or `INSTALL_CLIENT=true` can be used to override auto-detection (both can be enabled simultaneously).
- `requirements.txt` is not the full dependency list.

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

- **Tone control** – Set frequencies, amplitudes, and phases with full TX/RX power calibration (DAC through cryostat)
- **Data acquisition** – Discrete samples, continuous streaming, and triggered streaming to multiple clients
- **Single-tone burst mode** – Fast acquisition of 1024-sample bursts at the pre-accumulator rate for a single tone
- **Frequency sweeping** – Wideband and targeted sweeps with configurable direction, span, and resolution
- **Resonance finding** – Peak detection across multiple data formats (magnitude, phase, group delay), plus interactive PyQt5 GUI
- **Automated retuning** – Sweep-and-retune workflows with max-gradient and min-magnitude methods
- **Signal level optimization** – Automatic TX/RX level maximization with saturation detection and dynamic range management
- **ADC calibration freeze** – Freeze internal ADC calibration during observations to eliminate drift noise
- **Dual-pipeline support** – Two independent pipelines per board with three-level initialization to prevent cross-pipeline disruption
- **VACC multitone** (v7.9+) – Multiple tones per FFT bin with automatic LO index management
- **Configuration sync** – YAML-based config with `push_config()`/`pull_config()` for client-server synchronization
- **Server infrastructure** – Async TCP server with systemd daemon support, multi-client streaming, and remote status monitoring

## Documentation

| Document | Description |
|----------|-------------|
| [Installation](doc/installation.md) | Detailed client and server installation instructions |
| [Getting Started](doc/getting_started.md) | Configuration, usage guide, and feature list |
| [Dual Pipeline](doc/dual_pipeline.md) | Dual-pipeline setup, initialisation, and configuration |
| [v7.9 Multitone Notes](doc/v79-multitone-notes.md) | VACC multitone feature development notes |

## CLI Tools

**Client** (installed on your machine):

| Command | Description |
|---------|-------------|
| `souk-connection-test` | Test connectivity to the readout server; accepts `-C/--config_file` or `-a/--address` |
| `souk-wideband_sweep` | Perform a wideband frequency sweep |
| `souk-mkid-finder-app` | Launch the MKID resonance finder GUI |

*More client tools are planned.*

**Server** (installed on the RFSoC):

| Command | Description |
|---------|-------------|
| `souk-readout-server` | Start the readout server (`-p` for pipeline ID) |
| `souk-enable-daemon` | Enable the server as a systemd service |
| `souk-disable-daemon` | Disable the server systemd service |

*More server tools are planned.*

## Configuration

YAML-based configuration uses sections `rfsoc_host`, `firmware`, `rf_frontend`, `cryostat`, and `detector`. A template config is bundled with the package.

**Client side:** Config files live wherever you choose. Create a config from the bundled template or pull one from a running server:

```python
from souk_readout_tools.config_utils import copy_template_config
copy_template_config('my_config.yaml', pipeline_id=0)
```

**Server side:** The server manages its own pipeline-specific directories under `~/.souk_readout_tools/` on the RFSoC:

```text
~/.souk_readout_tools/
├── daemon/
├── pipeline_0/
│   ├── config/
│   └── calibrations/
└── pipeline_1/
    ├── config/
    └── calibrations/
```

`firmware.pipeline_id` inside the config is the authoritative pipeline identifier.

## Server Daemon

The readout server can run as a systemd service for automatic start on boot and restart on crash:

```bash
souk-enable-daemon          # Enable pipeline 0 (default)
souk-enable-daemon -p 0 1   # Enable both pipelines
souk-disable-daemon         # Disable both pipelines (default)
```

Monitor logs with:
```bash
sudo journalctl -f -xu readout_server_0
```

## Installation Details

The installer auto-detects the platform:
- **Xilinx/RFSoC kernel**: Installs server components
- **Everything else**: Installs client components

Override with environment variables: `INSTALL_SERVER=true` and/or `INSTALL_CLIENT=true` (both can be enabled simultaneously, e.g. for client+server on the RFSoC).

Dependencies: numpy, scipy, matplotlib, pyyaml, ipython. Client additionally requires PyQt5 (skipped automatically on Xilinx/headless platforms). Server requires `souk_mkid_readout` from the [souk-firmware](https://github.com/realtimeradio/souk-firmware) repository.

The server package currently expects `souk_mkid_readout` to be available on the RFSoC from:

```text
/home/casper/souk-firmware/software/control_sw
```

This is hardcoded in `setup.py`.

Client requires Python >= 3.8. Tested on Linux (Python 3.10, 3.12) and Windows (Python 3.12).

Server requires Python == 3.8 and the `souk_mkid_readout` library. Tested on Xilinx/RFSoC with the CASPER Linux image (Python 3.8).
