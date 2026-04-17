# SOUK Readout Tools

## The SOUK Detector Readout System

Simons Observatory: UK (SOUK) uses Microwave Kinetic Inductance Detectors (MKIDs) to observe the cosmic microwave background across two telescopes. Each MKID is a superconducting resonator whose resonance frequency shifts in response to absorbed photons - the readout system's job is to continuously probe these resonances and record the detector responses.

The readout hardware is based on AMD/Xilinx RFSoC (Radio Frequency System-on-Chip) boards. Each board runs the [`souk-firmware`](https://github.com/realtimeradio/souk-firmware) dual-pipeline firmware providing **two independent RF readout channels**, each capable of driving and measuring up to 2048 tones simultaneously across a wide RF bandwidth. The full SOUK instrument uses **7 boards per telescope** (28 RF networks in total), with all boards managed from a single host machine.



## What This Package Does

`souk_readout_tools` is the Python control and data acquisition package for this system. It uses a **client-server architecture** with `ReadoutClient` and `ReadoutServer` classes:

- The **server** runs on each RFSoC's ARM processor. It interfaces with the FPGA firmware through the [souk_mkid_readout](https://github.com/realtimeradio/souk-firmware) library and controls RF peripheral hardware (attenuators, amplifiers, etc.) via `souk-peripherals-control`. One server instance runs per pipeline (two per board), managing hardware state, configuration, and data tranmission.
- The **client** runs on a remote machine and connects to an individual pipeline's server over TCP. It provides the high-level API for measurements, data acquisition, and basic analysis tools for inspecting data quality.

```
 Client Machine (OCS)               RFSoC Board                                            
┌─────────────────────┐            ┌──────────────────────────────────────────────────────┐
│  Python/IPython     │            │  ARM Processing System (PS)                          │
│                     │            │                                                      │
│                     │            │   ┌── Ubuntu 24.04 (CASPER image) ───────────────┐   │
│                     │            │   │                                              │   │
│    ReadoutClient A ─┼─── TCP ────┼───┼──  ReadoutServer 0 ────┐                     │   │
│                     │            │   │                        ├── souk_mkid_readout │   │
│    ReadoutClient B ─┼─── TCP ────┼───┼──  ReadoutServer 1 ────┘          │          │   │
│                     │            │   └───────────────────────────────────┼──────────┘   │
│    ReadoutClient C ─┼─           ├───────────────────────────────────────┼──────────────┤
│                     │            │  FPGA Programmable Logic (PL)         │              │
│    ...              │            │                                       │              │
│                     │            │   ┌── SOUK Firmware ──────────────────┴──────────┐   │
│    ReadoutClient N ─┼─           │   │                                              │   │
│                     │            │   │                   Pipeline 0     Pipeline 1  │   │
│                     │            │   └───────────────────────┼──────────────┼───────┘   │
│                     │            ├───────────────────────────┼──────────────┼───────────┤
│                     │            │  RF Data Converter        │              │           │
│                     │            │  (RFDC)                ┌──┴──┐        ┌──┴──┐        │
│                     │            │                      DAC0   ADC0    DAC1   ADC1      │
└─────────────────────┘            └────────────────────────┼─────┼────────┼─────┼────────┘
                                                           TX0   RX0      TX1   RX1         
```

Each `ReadoutServer` uses two TCP ports - a **request port** for JSON command/response control and a **stream port** for binary data transfer.

### Capabilities

- **Tone control** - set frequencies, amplitudes, and phases for up to 2048 readout tones per pipeline with full TX/RX power calibration through the entire signal chain (DSP → DAC → RF frontend → cryostat → detector). Tone updates happen on millisecond timescales, enabling fast resonator tracking.
- **Data acquisition** - discrete samples, continuous streaming, triggered streaming, and single tone snapshots (1024 sample bursts at the pre-accumulator rate). All data includes PTP telescope timestamps from the firmware for precise time synchronisation.
- **Frequency sweeping** - wideband survey sweeps across the full RF band and targeted sweeps around individual resonances
- **Resonance finding** - automated peak detection across multiple data formats (magnitude, phase, group delay, |dS21/df|, etc), plus an interactive PyQt5 GUI
- **Retuning** - sweep-and-retune workflows to track drifting resonances using max-derivative or min-magnitude methods
- **Power management** - automatic TX/RX level optimisation with saturation detection, dynamic range management, and calibrated power control in dBm at any reference plane in the signal chain
- **ADC calibration freeze** - freeze the RFSoC's internal ADC calibration during observations to eliminate drift noise, with periodic defrost for recalibration
- **Clock source control** - select internal (12.8 MHz) or external (10 MHz) PL reference clock with PLL lock status monitoring
- **Dual-pipeline support** - two independent pipelines per board with three-level initialisation (program FPGA → shared resources → per-pipeline resources) to prevent cross-pipeline disruption
- **VACC multitone** (v7.9+) - multiple tones per FFT bin with automatic LO index management and sparse tone index handling
- **Configuration sync** - YAML-based config with `push_config()`/`pull_config()` for client-server synchronisation, including automatic calibration file transfer
- **Server infrastructure** - async TCP server with systemd daemon support, multi-client streaming, and remote status monitoring
- **Measurement framework** - parameter space measurement tools and higher-level scripts for characterisation campaigns
- **Analysis utilities** - built-in tools for parsing raw data, plotting in various formats, fitting resonance and noise models, and extracting detector parameters.

## Quick Start (Client)

Most users only need the client. The server should already be running on the RFSoC.

Clone the repository and initialise submodules:
```bash
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
git submodule init && git submodule update
```

Create a python virtual environment and install:

```bash
python3 -m venv client_venv
source ./client_venv/bin/activate
pip install .
```

On Windows, use `python -m venv` and `.\client_venv\Scripts\Activate.ps1` instead.

Connect to a running server and pull its config:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(address='10.11.11.11', request_port=10000)
client.pull_config(save_as='my_config.yaml')
```

Or create a config from the bundled template:

```python
from souk_readout_tools.config_utils import copy_template_config
copy_template_config('my_config.yaml', pipeline_id=0)
```

Then connect, initialise, set tones, and acquire data:

```python
client = ReadoutClient(config_file='my_config.yaml')
client.ensure_ready()

client.set_tone_frequencies([0.800e9, 1.500e9]) # frequencies in Hz
client.set_tone_powers([-50, -55], reference_plane='detector') # powers in dBm

raw = client.get_samples(500)
data = client.parse_samples(raw, num_tones=2)

# Plot the timestream
from souk_readout_tools.plotting import plot_timestream
plot_timestream(data)
```

For full installation details (including server setup, SD card imaging, and daemon configuration), see the [Installation Guide](doc/installation.md).

## CLI Tools

**Client** (installed on your machine):

| Command | Description |
|---------|-------------|
| `souk-connection-test` | Test connectivity to the readout server (`-C config.yaml` or `-a address`) |
| `souk-wideband_sweep` | Perform a wideband frequency sweep |
| `souk-mkid-finder-app` | Launch the interactive MKID resonance finder GUI |
| `souk-batch-snapshots` | Acquire pre-accumulator snapshots across tones |
| `souk-find-resonances` | Find resonances from sweep data |

**Server** (installed on the RFSoC):

| Command | Description |
|---------|-------------|
| `souk-readout-server` | Start the readout server (optional config path, `-p` for pipeline) |
| `souk-enable-daemon` | Enable the server as a systemd service (`-p 0 1` for both pipelines) |
| `souk-disable-daemon` | Disable the server systemd service(s) |

## Documentation

| Document | Description |
|----------|-------------|
| [Installation Guide](doc/installation.md) | Full client and server installation, SD card setup, daemon configuration |
| [Getting Started](doc/getting_started.md) | Configuration, usage guide, and worked examples for all features |
| [Dual Pipeline](doc/dual_pipeline.md) | Dual-pipeline setup, initialisation model, and multi-server operation |
| [Clock Source](doc/clock_source.md) | PL reference clock selection (internal/external) and PLL status |
| [v7.9 Multitone Notes](doc/v79-multitone-notes.md) | VACC multitone design notes and constraints |
| [Changelog](CHANGELOG.md) | Version history, feature list, and roadmap |

## Configuration

YAML-based configuration with five sections: `rfsoc_host`, `firmware`, `rf_frontend`, `cryostat`, and `detector`. A template config with all parameters and defaults is bundled with the package. Config files live wherever you choose on the client side - keep them with your measurement data.

The standard workflow is: create or pull a config, edit hardware-/firmware-specific settings, connect with it, and use `push_config()` / `pull_config()` to synchronise with the server. Calibration files referenced in the config are transferred automatically.

See [Getting Started - Configuration](doc/getting_started.md#configuration) for details.

## Requirements

- **Client:** Python >= 3.10. Tested on Linux (Python 3.10, 3.12) and Windows (Python 3.12). Dependencies: numpy, scipy, matplotlib, pyyaml, ipython, pyqt5.
- **Server:** Python >= 3.10 on the Xilinx/RFSoC CASPER Linux image. Requires `souk_mkid_readout` from the [souk-firmware](https://github.com/realtimeradio/souk-firmware) repository.
