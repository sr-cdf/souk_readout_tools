# SOUK Readout Tools

## The SOUK Detector Readout System

Simons Observatory: UK (SOUK) uses Microwave Kinetic Inductance Detectors (MKIDs) to observe the cosmic microwave background across two telescopes. Each MKID is a superconducting resonator whose resonance frequency shifts in response to absorbed photons - the readout system's job is to continuously probe these resonances and record the detector responses.

The readout hardware is based on AMD/Xilinx RFSoC (Radio Frequency System-on-Chip) boards. Each board runs the [`souk-firmware`](https://github.com/realtimeradio/souk-firmware) dual-pipeline firmware providing **two independent RF readout channels**, each capable of driving and measuring up to 2048 tones simultaneously across a wide RF bandwidth. The full SOUK instrument uses **7 boards per telescope** (28 RF networks in total), with all boards managed from a single host machine.

> **Installing or upgrading?** See the [Installation Guide](doc/installation.md); for a firmware change, start at [Upgrading](doc/installation.md#upgrading).

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
- **Blind tone management** - add fixed off-resonance monitor tones for gain/phase tracking, with config-based and interactive helpers, sweep inclusion, calibrated power support, and retune-safe tone metadata.
- **Data acquisition** - discrete samples, continuous streaming, triggered streaming, and single tone snapshots (1024 sample bursts at the pre-accumulator rate). All data includes PTP telescope timestamps from the firmware for precise time synchronisation.
- **Stream capture** - `receive_stream` / `souk-receive-stream` record for a set duration from a background writer, never overwrite an earlier capture, and save tone updates (modulation revisions, tracking commits) alongside; `stream_file` reads captures too large for memory in chunks and gives every sample a real time.
- **High-rate streaming** - both pipelines at 2500 Hz with 1000 tones each, with modulation and tracking running, lost 0 and 7 of 4.5 M samples in 30 minutes: the stream loop in its own real-time process, DMA reads of the accumulator, and `souk-enable-tuning` on the KRM. See [Firmware v7.12 Notes](doc/v712-notes.md).
- **Live viewer** - `souk-live-plot` shows time traces, PSDs and ASDs of a live or recorded stream, including calibrated frequency/dissipation and fast-demodulation panels. See [Live Plotting](doc/live_plotting.md).
- **Stream to DAC** - `souk-stream-to-dac` turns one tone's live response into a voltage on a LabJack or a sound card, for an external analog logger. See [Stream to DAC](doc/stream_to_dac.md).
- **G3 stream output** - record continuous streams to so3g/spt3g `.g3` files (Observation, Wiring, Scan frames with `G3SuperTimestream` payloads) for downstream OCS / SO data pipelines.
- **Mock server mode** - `ReadoutClient(mock=True)` emulates the readout server in-process (synthetic resonator catalogue, sweeps, snapshots, streams) for OCS / controller integration testing without RFSoC hardware.
- **Frequency sweeping** - wideband survey sweeps across the full RF band and targeted sweeps around individual resonances
- **Resonance finding** - automated peak detection across multiple data formats (magnitude, phase, group delay, |dS21/df|, etc), plus an interactive PyQt5 GUI
- **Resonance fitting** - linear and Duffing-capable complex S21 fitting (`fit_resonance`, `batch_fit`, `fit_sweep_stack`) with extracted physical parameters (`fr`, `Qi`, `Qc`, `phi`, etc.), optional uncertainty weighting, and process-parallel fitting support. `batch_fit` consumes server sweep dictionaries and preserves tone metadata; `fit_sweep_stack` consumes already-windowed arrays and fits rows independently.
- **Resonance matching** - say which resonance in one sweep is which device in another when the lists differ in length and the frequencies have moved (`resonance_matching`). See [Resonance Matching](doc/resonance_matching.md).
- **Retuning** - sweep-and-retune workflows to track drifting resonances using max-derivative or min-magnitude methods
- **Fast frequency modulation** - step each tone through a few probe points and stream the samples tagged per point, for **real-time IQ → frequency/dissipation conversion** (live per-resonator `dφ/df` rather than a stale start-of-obs calibration). The default firmware-slot engine switches four hardware LO slots on accumulation edges (the software engine takes any number of points). Toggle on/off at any time and update centres/offsets live with no dropped frames; the `souk_readout_tools.modulation` toolkit demodulates in the de-embedded/phase-centred basis (`params_from_sweep` → `group_cycles` → `demodulate`). See [Fast Frequency Modulation](doc/frequency_modulation.md).
- **Tone tracking** - the server follows each resonance from the modulated stream and recentres its tones with seamless, revision-stamped LO-buffer commits (dry run by default), reporting per-tone lock health in `get_info('tracking')`. It uses the calibration pushed with `set_resonator_calibrations`. See [Tone Tracking](doc/tone_tracking.md).
- **I/Q → resonator quantities** - one conversion contract in `resonator.py` (`prepare_conversion` / `convert_iq`, `convert_samples`) used by the analysis, the live viewer, the tracker and stream-to-DAC, with one sign convention; calibrations travel with the data in every capture. See [Conversions](doc/conversions.md).
- **Filterbank compensation** - tones away from their bin centre (modulation riding the channel overlap) are corrected for the channel response: in hardware on TX, at parse time on RX. See [Filterbank Compensation](doc/filterbank_compensation.md).
- **Power management** - automatic TX/RX level optimisation with saturation detection, dynamic range management, and calibrated power control in dBm at any reference plane in the signal chain. ADC over-voltage trips are monitored, logged, and marked per sample in captures.
- **Drive-power tuning** - power sweeps that measure bifurcation directly (up and down sweeps), per-tone best-power choice with guards, and tone powers allocated from measured `|dz/dx|` (`power_sweep`). See [Resonator Noise Workflow](doc/resonator_noise_workflow.md) and [Tone Power Notes](doc/tone_power_notes.md).
- **ADC calibration freeze** - freeze the RFSoC's internal ADC calibration during observations to eliminate drift noise, with periodic defrost for recalibration
- **Clock source control** - select internal (12.8 MHz) or external (10 MHz) PL reference clock with PLL lock status monitoring
- **PTP/NTP timing status** - packaged `ptp4l`, chrony, and timing-monitor service templates, plus `get_info("timing")` for checking GM lock, PHC/NTP source health, and firmware sync readiness
- **Timed sync & multi-board alignment** (v7.10) - TT-targeted firmware syncs so independent boards align timestamps and accumulations to the same epoch, with on-board 1-PPS generation from the TSU strobe path. See [TSU Strobe and Timed Sync](doc/tsu_strobe_and_timed_sync.md).
- **Dual-pipeline support** - two independent pipelines per board with three-level initialisation (program FPGA → shared resources → per-pipeline resources) to prevent cross-pipeline disruption
- **RF peripherals and LNA bias** - RF module attenuators and bypass amplifiers (`cardiff_dual`, `manchester_single` and `rx_prototype` modules), and cryostat LNA bias on v1 and v2 boards through one LNA bias service per telescope. See [RF Peripherals](doc/rf_peripherals.md) and [LNA Bias](doc/lna_bias.md).
- **Board management** - read the KRM carrier's board management controller and switch board power over USB serial from the control PC (`souk-find-boards`, `souk-board-status`, `souk-board-power`). See [Board Management](doc/board_management.md).
- **VACC multitone** (v7.9+) - multiple tones per FFT bin with automatic LO index management and sparse tone index handling
- **Parameter-series measurements** - repeat any measurement across an external parameter (attenuation, temperature, bias, time, ...) with run directories, resume, retries, and live summaries; power sweeps are the worked example. See [Parameter-Series Measurements](doc/measurements.md).
- **Configuration sync** - YAML-based config with `push_config()`/`pull_config()` for client-server synchronisation, including automatic calibration file transfer
- **Server infrastructure** - async TCP server with systemd daemon support, multi-client streaming, and remote status monitoring
- **Timestream and PSD analysis** - `timeseries`: PSDs of accumulated and snapshot streams and the composite trace, cross spectra and coherence, band levels and noise-model fits, on-minus-off subtraction, combined quadratures and `beta`, `tau_qp` from roll-offs and pulses, glitch masking, Allan deviation. See [Timeseries Analysis](doc/timeseries_analysis.md).
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

**No RFSoC hardware?** Pass `mock=True` to run against a built-in in-process mock
server — no config or address needed — to explore the API or develop analysis
code without a board:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(mock=True)
client.ensure_ready()
freqs = [0.800e9, 1.500e9]
client.set_tone_frequencies(freqs)
client.set_tone_phases(client.generate_newman_phases(freqs))  # Newman phases minimise the multi-tone crest factor
data = client.parse_samples(client.get_samples(500), num_tones=2)
```

Otherwise, connect to a real running server and pull its config:

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

freqs = [0.800e9, 1.500e9]
client.set_tone_frequencies(freqs)                            # frequencies in Hz
client.set_tone_powers([-90, -95], reference_plane='detector') # powers in dBm
client.set_tone_phases(client.generate_newman_phases(freqs))  # Newman phases minimise the multi-tone crest factor

raw = client.get_samples(500)
data = client.parse_samples(raw, num_tones=2)

# Plot the timestream
from souk_readout_tools.plotting import plot_timestream
plot_timestream(data)
```

For full installation details (including server setup, SD card imaging, and daemon configuration), see the [Installation Guide](doc/installation.md). For RFSoC timing setup, see [Timing and PTP](doc/timing.md).

## CLI Tools

**Client** (installed on your machine):

| Command | Description |
|---------|-------------|
| `souk-connection-test` | Test connectivity to the readout server (`-C config.yaml` or `-a address`) |
| `souk-wideband_sweep` | Perform a wideband frequency sweep |
| `souk-mkid-finder-app` | Launch the interactive MKID resonance finder GUI (also `souk-mkid-finder`) |
| `souk-receive-stream` | Record the stream to disk (`--duration`; tone updates saved alongside; `--overwrite` to replace a capture) |
| `souk-live-plot` | [View live stream files](doc/live_plotting.md) as time/PSD/ASD panels; optionally start a receiver using hardware details |
| `souk-stream-to-dac` | [Live analog output](doc/stream_to_dac.md) of a tone's response through a LabJack or sound card |
| `souk-batch-snapshots` | Acquire pre-accumulator snapshots across tones |
| `souk-find-resonances` | Find resonances from sweep data |
| `souk-find-boards` | List attached RFSoC USB serial bridges and identify the board-management port |
| `souk-board-status` | Read and health-check the board management controller (read-only) |
| `souk-board-power` | Report or change the RFSoC board power state via the BMC |

**Server** (installed on the RFSoC):

| Command | Description |
|---------|-------------|
| `souk-readout-server` | Start the readout server (optional config path, `-p` for pipeline) |
| `souk-enable-daemon` | Enable the server as a systemd service (`-p 0 1` for both pipelines) |
| `souk-enable-daemons` | Enable both pipeline systemd services |
| `souk-disable-daemon` | Disable the server systemd service(s) |
| `souk-disable-daemons` | Disable both pipeline systemd services |
| `souk-restart-daemon` | Restart the server systemd service(s) without reinstalling them |
| `souk-restart-daemons` | Restart both pipeline systemd services without reinstalling them |
| `souk-enable-timing` | Install and restart the packaged `ptp4l`, chrony PHC, and timing-monitor setup |
| `souk-enable-tuning` | KRM only: CPU layout, real-time stream processes and network tuning for both pipelines (`souk-disable-tuning` undoes it, `souk-tuning-status` shows it) |
| `souk-timing-monitor` | Run the PTP/NTP timing monitor daemon |
| `souk-timing-monitor-start` | Start the timing-monitor service (`-stop`/`-restart`/`-status`) |
| `souk-test-timing-monitor` | Query or stream status from `/run/timing-monitor.sock` |
| `souk-tsu-strobe` | Run the TSU 1-PPS strobe loop (normally run by its service) |
| `souk-tsu-strobe-install` | Install and start the TSU strobe service (`-start`/`-stop`/`-restart`/`-status`/`-remove`) |
| `souk-find-attenuators` | Discover RUDAT and SOUK mixerless-module attenuators (`--status` for live values) |
| `souk-find-bypass-amps` | Discover SOUK mixerless-module bypass amplifiers (`--status` for bypass state) |
| `souk-find-lnas` | Discover populated LNA bias monitors (`--status` for voltage/current readings) |
| `souk-rf-peripherals-status` | Combined attenuator, bypass-amplifier, and LNA status report |
| `souk-lna-service` | Run the LNA bias service (slot-1 board of each telescope rack only) |
| `souk-lna-service-install` | Install and start the LNA bias service (`-start`/`-stop`/`-restart`/`-status`/`-remove`) |

## Documentation

| Document | Description |
|----------|-------------|
| [Installation Guide](doc/installation.md) | Full client and server installation, SD card setup, daemon configuration |
| [Getting Started](doc/getting_started.md) | Configuration, usage guide, and worked examples for all features |
| [Fast Frequency Modulation](doc/frequency_modulation.md) | Real-time IQ → frequency/dissipation conversion and resonator tracking via per-tone probe-point modulation |
| [Tone Tracking](doc/tone_tracking.md) | Server-side FFM auto-recentering: state machine, settings, staged commits, health monitoring and cost |
| [Conversions](doc/conversions.md) | I/Q to resonator quantities: conventions, conversion methods, three-point modulation and how calibrations travel with the data |
| [Timeseries Analysis](doc/timeseries_analysis.md) | Timestream and PSD tools: detector-noise workflow, estimator biases, glitches, `tau_qp`, combining quadratures |
| [Calibration](doc/calibration.md) | Power calibration model and RF signal-chain configuration |
| [Filterbank Compensation](doc/filterbank_compensation.md) | Channel-response correction for tones away from their bin centre, and how to measure it |
| [Tone Power Notes](doc/tone_power_notes.md) | Dynamic-range, VACC, crest-factor, and blind-tone power guidance |
| [Resonator Noise Workflow](doc/resonator_noise_workflow.md) | Drive-power tuning, blackbody-load directory layout, and on/off-resonance noise captures |
| [Parameter-Series Measurements](doc/measurements.md) | Repeating any measurement across an external parameter (attenuation, temperature, time, ...) with run directories, resume, and live summaries |
| [Resonance Matching](doc/resonance_matching.md) | Matching resonance lists between sweeps |
| [Stream to DAC](doc/stream_to_dac.md) | Live analog output of KID response through a LabJack or sound card |
| [Live Plotting](doc/live_plotting.md) | `souk-live-plot`: live and recorded streams as time/PSD/ASD panels, calibrated and demodulated views |
| [RF Peripherals](doc/rf_peripherals.md) | RF attenuator, bypass amplifier, and discovery tooling |
| [LNA Bias](doc/lna_bias.md) | Cryostat LNA bias control, soft-off, and status monitoring |
| [Dual Pipeline](doc/dual_pipeline.md) | Dual-pipeline setup, initialisation model, and multi-server operation |
| [Board Management](doc/board_management.md) | RFSoC board management controller (BMC) over USB serial: power on/off, board health, boot and clock settings |
| [Clock Source](doc/clock_source.md) | PL reference clock selection (internal/external) and PLL status |
| [Timing and PTP](doc/timing.md) | `ptp4l`, chrony, timing-monitor setup, standalone monitoring, and firmware-sync readiness |
| [Timing Site Checklist](doc/timing_site_checklist.md) | Site commissioning checks for PTP GM, chrony, monitor state, and holdover behaviour |
| [TSU Strobe and Timed Sync](doc/tsu_strobe_and_timed_sync.md) | 1-PPS TSU strobe, firmware timed-sync bring-up for multi-board timestamp alignment, and test scripts |
| [v7.9 Multitone Notes](doc/v79-multitone-notes.md) | VACC multitone design notes and constraints |
| [Firmware v7.12 Notes](doc/v712-notes.md) | Installing v7.12 (boot files, casperfpga, control_sw), what limits streaming, measured rates, and KRM board setup |
| [Version Compatibility](doc/compatibility.md) | Which software runs on which firmware, OS image/kernel/boot-file changes, and supported boards, by version |
| [Changelog](CHANGELOG.md) | Version history, feature list, and roadmap |

## Configuration

YAML-based configuration with five sections: `rfsoc_host`, `firmware`, `rf_frontend`, `cryostat`, and `detector`. A template config with all parameters and defaults is bundled with the package. Config files live wherever you choose on the client side - keep them with your measurement data.

The standard workflow is: create or pull a config, edit hardware-/firmware-specific settings, connect with it, and use `push_config()` / `pull_config()` to synchronise with the server. Calibration files referenced in the config are transferred automatically.

See [Getting Started - Configuration](doc/getting_started.md#configuration) for details.

## Requirements

- **Client:** Python >= 3.10. Tested on Linux (Python 3.10, 3.12) and Windows (Python 3.12). Dependencies: numpy, scipy, matplotlib, pyyaml, ipython, pyqt5, pyserial. `pip install .[g3]` adds so3g for `.g3` stream files (not available on Windows).
- **Server:** Python >= 3.10 on the Xilinx/RFSoC CASPER Linux image, with [souk-firmware](https://github.com/realtimeradio/souk-firmware) v7.12.1: its `souk_mkid_readout` (`software/control_sw`) and casperfpga installed in the board's venv, and the matching boot files. See [Version Compatibility](doc/compatibility.md).
