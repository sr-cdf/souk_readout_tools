# Getting Started with SOUK Readout Tools

SOUK Readout Tools is a Python package with tools for operating the MKID (Microwave Kinetic Inductance Detector) readout system on the RFSoC (Radio Frequency System-on-Chip) boards for Simons Observatory UK. It uses a client-server architecture: the server runs on the RFSoC ARM processor and clients connect remotely over TCP.


---

## Contents

- [System Overview](#system-overview)
- [Installation](#installation)
- [Configuration](#configuration)
- [Connecting to the RFSoC](#connecting-to-the-rfsoc)
- [Initialisation](#initialisation)
- [Setting Readout Tones](#setting-readout-tones)
- [Acquiring Data](#acquiring-data)
- [Streaming](#streaming)
- [Frequency Sweeping](#frequency-sweeping)
- [Wideband Sweep](#wideband-sweep)
- [Resonance Finding](#resonance-finding)
- [Retuning](#retuning)
- [Fast Frequency Modulation](#fast-frequency-modulation)
- [Power Calibration & Optimisation](#power-calibration--optimisation)
- [Clock Source](#clock-source)
- [Timing Status](#timing-status)
- [Pre-Accumulator Snapshots](#pre-accumulator-snapshots)
- [Plotting](#plotting)
- [Resonator Analysis](#resonator-analysis)
- [Parameter Space Measurements](#parameter-space-measurements)
- [Dual-Pipeline Operation](#dual-pipeline-operation)
- [Mock Server Mode](#mock-server-mode)
- [CLI Tools](#cli-tools)
- [Changelog & Feature List](#changelog--feature-list)
- [Future Developments](#future-developments)

---

## System Overview

### Architecture

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

Up to two independent readout pipelines can run on a single RFSoC board, each with its own server instance and client connection. Each pipeline uses separate TCP request and stream ports. The readout server runs on the ARM Processing System (PS) and communicates with the FPGA Programmable Logic (PL) through the `souk_mkid_readout` firmware interface library.

The system uses two TCP ports per pipeline:
- **Request port**: JSON command/response for control operations.
- **Stream port**: Binary data streaming for high-throughput sample transfer.

Default ports: pipeline 0 uses 10000/20000, pipeline 1 uses 10001/20001.

### Key Modules

| Module | Description |
|--------|-------------|
| `readout_client.py` | Client interface for remote control and data acquisition |
| `readout_server.py` | Async TCP server running on the RFSoC ARM processor |
| `timing.py` | Client helpers for the local RFSoC timing monitor |
| `firmware_lib.py` | FPGA firmware interface: register access, tone management, sweeps |
| `calibration.py` | RF power/amplitude calibration chain (DAC to detector) |
| `peak_finder.py` | MKID resonance detection algorithms |
| `fitting.py` | Nonlinear resonator model fitting (Khalil notch model) |
| `resonator.py` | Resonator S21 transforms: RF deembedding and phase centering |
| `plotting/` | Plotting library for sweep, timestream, and snapshot data |
| `measurement.py` | Simple repeat-measurement runner |
| `mkid_finder_app.py` | PyQt5 GUI for interactive resonance finding |
| `tone_list_tools.py` | Tone list file I/O utilities |

### Signal Chain

The readout signal chain is roughly:

```
Tone Generation (digital LO + mixer) → PSB → DAC → RF Frontend (optional upconversion) → Cryostat → Detector
Detector → Cryostat → RF Frontend (optional downconversion) → ADC → PFB → Channel Select → Mixer → Accumulator → Output
```

---

## Installation

### Server

The readout server runs on the RFSoC and should already be installed and configured. If you need to set up or reinstall the server, see the [Installation Guide](installation.md#server-software-installation--setup).

Verify the server is running:

```bash
ssh casper@10.11.11.11
sudo systemctl status readout_server_0
```

If timing services are installed on the RFSoC, they run separately from the
readout server:

```bash
systemctl status ptp4l chrony timing-monitor
souk-test-timing-monitor status
```

### Client

#### 1. Network Setup

Ensure the RFSoC (default IP `10.11.11.11/24`) is connected and on the same subnet. For example, manually set the client machine's IP address to `10.11.11.1/24` (or equivalent).

Verify connectivity:

```bash
ping 10.11.11.11
ssh casper@10.11.11.11
```

Optionally, add a hostname entry to `/etc/hosts` (Linux) or `C:\Windows\System32\drivers\etc\hosts` (Windows):

```
10.11.11.11 rfsoc
```

#### 2. Install the Package

Clone the repository and set up a virtual environment:

```bash
git clone https://github.com/sr-cdf/souk_readout_tools
cd souk_readout_tools
git submodule init && git submodule update
```

**Linux:**
```bash
python3 -m venv client_venv
source ./client_venv/bin/activate
```

**Windows PowerShell:**
```powershell
python -m venv client_venv
.\client_venv\Scripts\Activate.ps1
# If needed: Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

Install the package:
```bash
pip install .
```

The installer auto-detects the platform and installs client components on non-Xilinx machines.

---

## Configuration

Configuration is YAML-based. A template config is bundled with the package. The primary purpose of the initial config is to set the **RFSoC IP address** so the server and client know how to communicate — all other settings can be adjusted later.

### Config File Structure

The main sections are:

| Section | Description |
|---------|-------------|
| `rfsoc_host` | IP address, ports, clock source, trigger/GPIO settings |
| `firmware` | Firmware file, pipeline ID, RFDC tile/block mapping, DAC/ADC calibrations, default tone/power settings |
| `rf_frontend` | LO frequency, sideband, mixer/combiner losses, attenuators, S21 measurements |
| `cryostat` | Channel info, S21 measurements, LNA settings, thermometry, optical setup |
| `detector` | Chip/channel IDs, resonance and drive power file references |

### Config Files

On the client side, config files live wherever you choose. Keep them with your project or measurement data. The standard workflow is to maintain a local config file, connect with it, and push changes to the RFSoC.

> **Note:** Before creating a config, `cd` to the directory where you want to keep your project files and data — the config will be written to your current working directory. For example:
> ```bash
> mkdir -p ~/my_mkid_project && cd ~/my_mkid_project
> ```

Create a new config from the bundled template:

```python
from souk_readout_tools.config_utils import copy_template_config
copy_template_config(destination='my_config.yaml', pipeline_id=0)
```

The `pipeline_id` argument sets the generated config's pipeline-specific
defaults, including `firmware.pipeline_id`, TCP ports, RFDC tile/block mapping,
and `rf_frontend.mixerless_module.rf_channel`.

To create a config for **Nyquist zone 2** operation, pass `nyquist_zone=2`:

```python
copy_template_config(destination='my_config.yaml', pipeline_id=0, nyquist_zone=2)
```

This sets the `nyquist_zone` key in the config, which controls the DAC DUC and ADC DDC mixer frequencies automatically. Zone 1 uses mixer frequencies of +/-fs/4 (~1228.8 MHz) and zone 2 uses +/-3*fs/4 (~3686.4 MHz). Tone frequencies should be set in the RF band corresponding to the configured zone — e.g. for zone 2, frequencies will be in the second Nyquist band above fs/2.

### Preparing a Config File

The minimum you need to set to get started is the **RFSoC IP address** — this tells the client where to connect and the server where to listen. Everything else can be left as defaults and adjusted later.

Edit the config file with your hardware-specific settings. The most important parameters to set are:

- `rfsoc_host.address` - The RFSoC IP address (e.g. `10.11.11.11`) — **set this first**
- `rfsoc_host.request_port` / `stream_port` - TCP ports (must be unique per pipeline)
- `firmware.fw_config_file` - Path to the firmware config YAML on the RFSoC
- `firmware.pipeline_id` - Pipeline index (0 or 1)
- `firmware.defaults.nyquist_zone` - Nyquist zone (1 or 2) — set this if operating in the second Nyquist zone
- `firmware.defaults.dac_inverse_sinc_filter_enabled` - Enable DAC sinc-roll-off compensation (default `true`)
- `firmware.dac*_tile`, `dac*_block`, `adc_tile`, `adc_block` - RFDC channel mapping
- `rf_frontend.*` - Analog frontend configuration for your setup

---

## Connecting to the RFSoC

Start the readout server on the RFSoC (if not already running as a daemon):

```bash
ssh casper@rfsoc
sudo /home/casper/py3.12-venv/bin/souk-readout-server /path/to/config.yaml
```

Then, on the client machine, start an IPython session and create the client with your local config file:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(config_file='my_config.yaml')
```

> **Quick start:** If the server is already configured and running (e.g. someone else set it up), you can skip config file creation and connect by address instead. See [Pulling Configuration from the RFSoC](#pulling-configuration-from-the-rfsoc) to get a local config file from the running system.

```python
client = ReadoutClient(address='10.11.11.11', request_port=10000)
```

### How config and calibration files are managed

Config files and calibration files are always held **in memory** on the client. Pushing always writes to the server's persistent storage. Saving to the client's local disk is optional and only happens when explicitly requested.

- **`push_config()`** sends the in-memory config to the server (always persisted on the RFSoC). Any calibration file paths in the config are resolved on the client, and the referenced files are automatically pushed to the server's pipeline calibrations directory. The local config is not modified.
- **`pull_config()`** fetches the active config file and any referenced calibration files from the server **into memory**. It does not include live hardware adjustments. Nothing is written to disk unless `save_as` is provided or `save_config()` is called.
- **`save_config()`** writes the in-memory config to a local YAML file. If calibration files were pulled from the server, they are written to a `calibrations/` directory next to the config file, and the config paths are rewritten to local relative paths.

This means calibration file paths in the config are portable: `push_config` resolves local paths and transfers the files to the server, `pull_config` + `save_config` fetches them back and creates local copies.

### Pushing Configuration to the RFSoC

`push_config()` sends the client's current in-memory config dictionary to the server, which saves it, applies any hardware parameter changes, and calls `ensure_ready()` so that any initialisation steps required by the changes are performed. The pushed config becomes the new default (persists across power cycles).

Config parameters can be modified directly on the client instance before pushing:

```python
# Modify a parameter in the client's config dictionary
client.config['firmware']['acc_len'] = 2**15

# Push the modified config to the server
client.push_config()

# Optionally save the modified config back to a local file
client.save_config()                       # overwrites the original file
client.save_config('my_config_v2.yaml')    # save as a new file
```

If the config YAML file on disk is edited instead, a new client instance must be created to pick up those changes before pushing:

```python
# After editing my_config.yaml on disk:
client = ReadoutClient(config_file='my_config.yaml')
client.push_config()
```

### Pulling Configuration from the RFSoC

If you are connecting to an RFSoC that is already configured and running, you can pull its config to create a local config file. This is the easiest way to get started for first-time use — once saved, you have a local config and can follow the standard workflow from then on:

```python
client = ReadoutClient(address='10.11.11.11', request_port=10000)

# Pull into memory and save to disk (config + any referenced calibration files)
client.pull_config(save_as='my_config.yaml')

# Pick up where you left off - the system is already configured
client.get_tone_frequencies()
```

If you already have a config file and just want to refresh the in-memory config from the server:

```python
client = ReadoutClient(config_file='my_config.yaml')
client.pull_config()  # updates in memory only, nothing written to disk
```

### Syncing Live Hardware State into the Config

`sync_config_from_system()` is the inverse of `apply_config()`: rather than pushing config values to the hardware, it reads the current hardware state and writes it back into the in-memory config.  This is useful when the system has been adjusted interactively (e.g. attenuators tuned, tone frequencies retuned, DSP parameters changed) and you want to capture that state so it can be persisted with `save_config()` or `push_config()`.

Fields updated by `sync_config_from_system()`:

- `firmware.defaults` — accumulator length, sync delay, RFDC parameters (VOP, Nyquist zone, DAC inverse-sinc filter, mixer scales, QMC settings, DSA, DUC/DDC mixer frequencies)
- `firmware.defaults.frequencies/amplitudes/phases` — current regular tone state
- `firmware.defaults.blind_frequencies/blind_amplitudes/blind_phases/blind_spans` — current blind tone state when the active tone metadata still matches the config
- `rf_frontend.attenuator.tx_value_db` / `rx_value_db` — current attenuator settings
- `rf_frontend.bypass_amps.tx_amp_bypass` / `rx_amp_bypass` — current amp bypass states. Measured mixerless-module S21 and P1dB calibration values live under `rf_frontend.mixerless_module`.
- `cryostat.lna_bias` — current LNA bias setting for this pipeline when the controller is enabled

```python
# Capture the current running state into the config
client.sync_config_from_system()

# Inspect what was synced
print(client.config['firmware']['defaults'])
print(client.config['rf_frontend']['attenuator'])

# Persist locally
client.save_config()

# Or push back to the RFSoC so it survives a reboot
client.push_config()
```

`sync_config_from_system()` returns the updated config dict; the synced state
also lives on `client.config`.  It does **not** write to disk unless `save_as`
is supplied, and it does not push to the server — those are separate explicit
steps.

`pull_config()` is intentionally separate: it overwrites the client's
in-memory config with the active config file from the server only. Runtime
hardware state is available via `get_info()` / status calls, and is copied into
config only by an explicit sync/capture call.

### Requesting Information

The `get_info()` method returns structured system information organised into named sections:

```python
# Default sections (fast — server, versions, clock, timing, fpga, rfdc,
# pipeline, tones, rf_frontend, lna)
info = client.get_info()

# Specific sections only; list input returns a list in the same order
tones, rfdc = client.get_info(['tones', 'rfdc'])

# A single section name returns that section directly
pipeline = client.get_info('pipeline')

# Everything including diagnostics, config, calibrations
info = client.get_info('all')

# Each section has a 'ready' flag and section-specific keys
print(info['tones']['count'])
print(info['rfdc']['dsa'])
print(info['server']['initialisation_level'])
```

For quick periodic monitoring, use `health_check()`:

```python
health = client.health_check()
# Returns compact pass/fail bools:
#   initialisation_level, clock_locked, timing_state, timing_ready,
#   streaming, sweeping, adc_saturated, dac_saturated, dsp_overflow,
#   rts_events, tone_count, client_count, rf_frontend_available,
#   lna_available
```

---

## Initialisation

The system uses a 3-level initialisation state machine:

| Level | Scope | What it does |
|-------|-------|-------------|
| 0 - Programming | Both pipelines | Loads the FPGA bitfile. Wipes all FPGA state. |
| 1 - Shared | Both pipelines | Initialises shared resources (snapshots, LUT generator, autocorrelator, etc.) |
| 2 - Pipeline | Single pipeline | Initialises per-pipeline resources (accumulator, mixer, channel maps, etc.) |

### Normal Operation

Use `ensure_ready()` to bring the system to a usable state with minimal disruption:

```python
client.ensure_ready(level="pipeline")
```

This only performs the initialisation steps that are actually needed. If the system is already fully initialised, nothing happens. If only the pipeline needs initialising, the shared resources and firmware bitfile are left untouched.

### Hard Reset

To force a full reprogram of the FPGA (resets both pipelines):

```python
client.hard_reset()
```

After a hard reset, both pipeline servers need to run `ensure_ready()` to recover.

### Pipeline DSP Parameters

Individual DSP pipeline parameters can be read and set directly via the client:

```python
# Accumulation length (controls sample rate)
client.get_acc_len()
client.set_acc_len(2**15)

# PSB scale (global output waveform scaling)
client.get_psb_scale()
client.set_psb_scale(0)

# Internal loopback (DAC output fed back to ADC input)
client.get_internal_loopback()
client.set_internal_loopback(True)
```

These are also available in the `pipeline` section of `get_info()`:

```python
pipeline = client.get_info('pipeline')
print(pipeline['acc_len'], pipeline['psb_scale'], pipeline['internal_loopback'])
```

---

## Setting Readout Tones

### Frequencies

Set readout tone frequencies in Hz. The system automatically configures the firmware (mixer LOs, PFB/PSB channel maps) to achieve the target frequencies, accounting for the RF-DAC and any analog converters specified in the config:

```python
client.set_tone_frequencies([0.800e9, 1.500e9])

# Check what was actually set:
client.get_tone_frequencies()

# Detailed breakdown (mixer freqs, DAC output, analog output into the cryostat, etc.):
client.get_tone_frequencies(detailed_output=True)
```

### Blind Tones

Blind tones are fixed monitor tones placed away from resonances. Configure them
alongside the regular tone list:

```yaml
firmware:
  defaults:
    frequencies: [0.800e9, 1.500e9]
    blind_frequencies: [0.900e9, 1.200e9]
    blind_amplitudes: [0.2, 0.2]
    blind_phases: [0.0, 1.57]
    blind_spans: [20000, 20000]
```

Here `frequencies`, `amplitudes`, and `phases` are the regular tones.
The `blind_*` entries are appended after them. The firmware sees one combined
tone list, so `get_tone_frequencies()`,
`get_tone_amplitudes()`, `get_tone_phases()`, and power helpers return all
tones in user order. Use the metadata helpers to split them:

```python
metadata = client.get_tone_metadata()
metadata['regular_indices']
metadata['blind_indices']

blind = client.get_blind_tones()
blind['frequencies_hz']
blind['indices']
```

Sweeps include blind tones and use `blind_spans` when supplied. Retune keeps
blind tones fixed at `blind_frequencies` and only updates regular tone indices.
To generate candidate blind centers, use the helper:

```python
blind = client.suggest_blind_frequencies(
    resonance_frequencies=client.get_tone_frequencies(),
    count=8,
    band_hz=(0.75e9, 1.75e9),
    min_distance_hz=1e6,
)
```

The helper first finds grid candidates that are far enough from resonances, then
chooses tones near approximately even target positions. Those targets are
jittered by default so the blind tones are not on a perfectly regular grid,
which helps avoid intermodulation products lining up into coherent spurs. Pass
`rng=np.random.default_rng(seed)` for repeatable suggestions, or
`random_offset_fraction=0` for a deterministic regular target pattern.

During interactive setup you do not need to edit a config file first. Set the
current regular tones, then attach blind tones to the live tone state:

```python
res_freqs = np.array([...])              # from VNA or wideband sweep analysis
client.set_tone_frequencies(res_freqs)

blind_freqs = client.suggest_blind_frequencies(
    resonance_frequencies=res_freqs,
    count=8,
    band_hz=(fmin, fmax),
    min_distance_hz=1e6,
)

client.set_blind_tones(
    blind_freqs,
    powers_dbm=-65,
    spans=20e3,
    reference_plane='detector',
)

freqs = client.get_tone_frequencies()
client.set_tone_phases(client.generate_newman_phases(freqs))
client.get_blind_tones()
```

`set_blind_tones()` snapshots the current regular tones,
adds/replaces the blind tones, updates the server's in-memory tone metadata,
and immediately applies the combined tone list. If `powers_dbm` is supplied,
the server preserves the current regular tone powers and sets the blind-tone
powers in the same calibrated power call. `remove_blind_tones()` drops the
blind tones and leaves the current regular tones active.

After blind tones are attached, calls that operate on all active tones should
use the combined arrays returned by `get_tone_frequencies()`,
`get_tone_amplitudes()`, or `get_tone_phases()`. Calls that should operate on
regular tones only can use `get_regular_tone_indices()` or
`get_blind_tone_indices()` to split the returned data.

### Powers

The final output power of each tone depends on several factors in the signal chain:

1. The tone's **LO amplitude** (per-tone relative scaling, 0 to 1.0)
2. The **PSB FFT shift schedule** (controls bit growth through the filterbank - important for maintaining dynamic range and avoiding integer overflow/underflow)
3. The **global PSB scale** parameter (scales the entire output waveform going into the DAC)
4. The **RF-DAC** configuration and analog response
5. The **RF frontend** frequency response (mixer, attenuators, amplifiers)
6. The **cryogenic transmission** path losses

In practice, use the **amplitude** to set relative power levels between tones, and adjust the **global PSB scale** to maximise the waveform amplitude going into the DAC. The FFT shift schedule should be configured to maintain the highest dynamic range through the digital signal chain.

**By amplitude scale** (linear, 0 to 1.0, relative to LO scale):
```python
client.set_tone_amplitudes([0.5, 0.3])
client.get_tone_amplitudes()
```

**By power in dBm** (accounts for all of the above, using calibration values from the config):
```python
client.set_tone_powers([-20, -25], reference_plane='detector')

# Detailed power breakdown through the signal chain:
client.get_tone_powers(detailed_output=True)
```

By default, `set_tone_powers` automatically optimises the dynamic range — it maximises DAC bit utilisation, adjusts available TX RF controls (programmable attenuator and amp bypass), and lowers `psb_scale` if those controls cannot absorb enough excess power. RX handling is controlled with `rx_policy`: use `protect` to avoid ADC saturation, `compensate` to keep ADC power approximately constant when TX power changes, or `maximise` to run `maximise_rx_power()` after the TX change. To skip optimisation for faster execution (e.g. during sweeps where the analog chain is already configured), pass `optimise_dynamic_range=False`.

The tone powers can be set to the maximum level that avoids saturation of the RF chain by calling ```client.maximise_tx_power()```

Simlarly, the ADC input level can be maximised by adjusting the RX attenuators and or DSA settings with a call to ```client.maximise_rx_power()```

### Phases

Phase offsets (in radians) are applied at the firmware mixer LO stage. Use Newman's quadratic multisine phases to minimise crest factor:

```python
freqs = client.get_tone_frequencies()
phases = client.generate_newman_phases(freqs)
client.set_tone_phases(phases)
```

### Convenience: Set All at Once

```python
client.set_tones_helper(freqs=[0.8e9, 1.5e9], powers_dbm=[-50, -55], phases=[0.0, 1.57])
```

`powers_dbm` sets calibrated output power in dBm via `set_tone_powers()`. You can also pass `amps` (0 to 1.0) instead for uncalibrated amplitude control — `amps` is ignored if `powers_dbm` is provided.

---

### Dynamic range optimisation

Different combinations of internal firmware parameters and rf peripheral values can achieve the same tone powers in both the transimt and receive chains. The choice of parameters affects the dynamic range and noise performance of the system. For example, setting high per-tone amplitudes and high attenuator values reduces the impact DAC quantisation noise compared to using low amplitudes with low attenuation. The client provides methods `optimise_tx_snr()` and `optimise_rx_snr()` to automatically check and optimise the settings to provide the best SNR for a given power level. See [Power Calibration & Optimisation](#power-calibration--optimisation) for details.

## Acquiring Data

### Discrete Samples

Acquire a fixed number of samples from the accumulator output:

```python
import numpy as np
import matplotlib.pyplot as plt

# Check the sample rate
sample_rate = client.get_sample_rate()

# Collect 1 second of data
raw = client.get_samples(int(sample_rate))

# Parse into IQ data, counters, flags
num_tones = len(client.get_tone_frequencies())
data = client.parse_samples(raw, num_tones)

# Verify no dropped packets
print(np.all(np.diff(data['packet_counter']) == 1))

# Each sample includes a PTP telescope timestamp (64-bit integer)
print(data['telescope_time'])

# Plot magnitude of the first tone
t = np.arange(len(data['packet_counter'])) / sample_rate
z0 = data['i_data']['0000'] + 1j * data['q_data']['0000']
plt.plot(t, np.abs(z0))
plt.xlabel('Time (s)')
plt.ylabel('|S21|')
plt.show()

# or use the internal plotting tools
from souk_readout_tools.plotting import plot_timestream
fig = plot_timestream(data, tone_indices=[0], format = 'iq_vs_t',title='Timestream of Tone 0')
plt.show()
```

### Export and Import

```python
# Export to file (npy, json, or csv)
client.export_samples('my_data.npy', data, num_tones_to_save=2)

# Import back
data2 = client.import_samples('my_data.npy')
```

### Sample Rate

The accumulator output sample rate can be adjusted:

```python
client.get_sample_rate()   # e.g. 500.0 Hz
client.set_sample_rate(1000)
```

### Telescope Time (PTP Timestamp)

The firmware attaches a 64-bit telescope timestamp to each accumulation. This timestamp is included in every sample frame, sweep point, and stream packet. To read it on demand:

```python
tt = client.get_telescope_time()

```
To check whether the RFSoC is locked to the PTP grandmaster:

```python
timing = client.get_info("timing")
print(timing["summary"]["state"], timing["summary"]["ready_for_firmware_sync"])
print(timing["ptp"].get("healthy"), timing["chrony"].get("source_type"))
```

See [Timing and PTP](timing.md) for the RFSoC `ptp4l`, chrony, and timing-monitor setup.

The plotting tools can read the telescope time from the data:

```
plot_timestream(data,x_axis='telescope_time')
```

---

## Streaming

For continuous streams of arbitrary length, use the streaming interface.

### Start a Stream

```python
client.enable_stream()
```

### Record to Disk

In a separate terminal, run the stream receiver script:

```bash
python -m souk_readout_tools.client.client_scripts.receive_stream -p -n 2
```

The `-p` flag prints live data to the terminal. The `-n` flag limits the number of tones saved to disk (to avoid wasting space on unused channels). Use `-d` and `-f` for custom output directory and filename.

### Stop the Stream

```python
client.disable_stream()
```

The receiver script can be stopped with `Ctrl-C` or left running for the next stream.

### Parse and Plot Stream Data

```python
data = client.parse_stream('tmp_stream')
t = np.arange(len(data['packet_counter'])) / data['sample_rate']
z0 = data['i_data']['0000'] + 1j * data['q_data']['0000']
plt.plot(t, np.abs(z0))
plt.show()
```

or use the plotting tool:

```
plot_timestream(data,x_axis='telescope_time')
```

### G3 Stream Output (so3g)

For OCS / Simons Observatory data pipelines, streams can be recorded directly
into so3g/spt3g `.g3` files instead of the default binary format:

```python
client.receive_stream_g3(
    num_tones=None,             # default: all active tones from the firmware
    filename='my_stream.g3',
    duration=30,                # seconds
    kid_stream_id='UNSET',      # tag carried in Wiring frame metadata
)
```

The output file contains:

- one `Observation` frame at the start (timestamp, run metadata),
- one `Wiring` frame describing tone metadata (frequencies, indices, ID),
- a sequence of `Scan` frames carrying `G3SuperTimestream` payloads with
  I/Q data, packet counters/errors, and PTP telescope timestamps,
- one closing `Observation` frame at the end.

A standalone receiver script is provided as
`receive_stream_g3.py` (under `client/client_scripts/`); it is currently
shipped as a module rather than an installed entry point, matching the
existing `receive_stream.py` convention. It accepts either a config file
(`-C`) or an address/port pair, plus `--duration` and `--filename`.

> **Note:** PTP telescope time is currently embedded in the
> `G3SuperTimestream`, but the mapping between this package's
> `timing_monitor` state and the spt3g timing paradigms (G3 time vs.
> session time) is still being finalised. Treat G3 timestamps as
> firmware-PTP for now.

### Triggered Streaming

The server can also stream samples on external trigger pulses:

```python
client.enable_triggered_stream()

# Software trigger for testing:
client.send_fake_trigger()

# Stop:
client.disable_triggered_stream()
```

The packet counter increments every sample regardless of trigger, so trigger times can be inferred from counter gaps. Each streamed frame also includes the PTP telescope timestamp (`telescope_time`).

---

## Frequency Sweeping

Perform a targeted frequency sweep around specific tone centers:

```python
centers = [0.800e9, 1.500e9]
spans = [0.2e6, 0.2e6]        # span per tone in Hz
num_points = 101
samples_per_point = 3          # keep low for fast sweeps

client.perform_sweep(centers, spans, num_points, samples_per_point, direction='up', wait=True)

# `wait=True` blocks until the sweep completes. To dispatch without blocking,
# omit `wait` (or pass `wait=False`) and poll/await manually:
#   client.perform_sweep(centers, spans, num_points, samples_per_point, direction='up')
#   client.get_sweep_progress()  # 0.0 to 1.0
#   client.wait_for_sweep()

raw_sweep = client.get_sweep_data()
data = client.parse_sweep_data(raw_sweep)

# Plot
for k in range(data['num_tones']):
    plt.plot(data['sweep_f'][k] / 1e6,
             20 * np.log10(np.abs(data['sweep_i'][k] + 1j * data['sweep_q'][k])),
             label=f'Tone {k}')
plt.xlabel('Frequency (MHz)')
plt.ylabel('|S21| (dB)')
plt.legend()
plt.show()
```

or use the internal plotting tools:

```python
from souk_readout_tools.plotting import plot_sweep
fig = plot_sweep(data, format='magphase', multi_tone='overlay', title='Frequency Sweep')
plt.show()
```

At the end of the sweep, tones are returned to the center frequencies. 

The parsed sweep data includes a `telescope_time` array with the PTP timestamp of the first sample taken at each sweep point.

---

## Wideband Sweep

The wideband sweep covers the full RF bandwidth (or a specified sub-band) using many tones swept in parallel. This is useful for surveying the entire band to find resonances before setting individual readout tones:

```python
sweep_data = client.wideband_sweep(
    bandwidth_hz=None,               # None = defaults to 90% of full bandwidth
    center_freq_hz=None,             # None = defaults to center of band
    step_size_hz=10000,              # frequency step size
    num_tones=1024,                  # number of parallel tones
    samples_per_point=10,            # accumulation per point
    tone_powers_dbm=-50,             # per-tone power in dBm (scalar or array), or 'auto'
    reference_plane='detector',      # reference plane for tone power calibration
    optimise_tx_dynamic_range=True,  # automatically adjust TX parameters to maximise dynamic range
    optimise_rx_gain=True,           # automatically adjust RX gain/attenuation to maximise SNR without saturation

    verbose=True
)

# Plot the result
f = sweep_data['sweep_f'][0]
z = sweep_data['sweep_i'][0] + 1j * sweep_data['sweep_q'][0]
plt.plot(f / 1e9, 20 * np.log10(np.abs(z)))
plt.xlabel('Frequency (GHz)')
plt.ylabel('|S21| (dB)')
plt.show()
```
or use the internal plotting tools:

```python
from souk_readout_tools.plotting import plot_sweep
fig = plot_sweep(data, format='magphase', multi_tone='overlay', title='Frequency Sweep')
plt.show()
```

The wideband sweep automatically checks for ADC/DAC saturation before proceeding. 

A CLI tool is also available:

```bash
souk-wideband_sweep
```

---

## Resonance Finding

Use the built-in resonance finder to locate MKID dips in sweep data:

```python
# Find resonances from existing sweep data
result = client.find_resonances(sweep_data, data_format='log_magnitude')
resonances = result['all_resonances']

# Or let find_resonances perform a wideband sweep automatically
result = client.find_resonances()  # calls wideband_sweep() internally
resonances = result['all_resonances']

# Each resonance has: frequency, fwhm, q_factor, qc, qi, dip_depth
for r in resonances:
    print(f'{r.frequency/1e9:.6f} GHz, Q={r.q_factor:.0f}, depth={r.dip_depth:.1f} dB')

# Get just the frequency list
freqs = client.find_resonance_frequencies(sweep_data)
```

`find_resonances()` returns the same result object for wideband and targeted
sweeps. Use `result['all_resonances']` for the flat list, `result['per_tone']`
for per-tone/per-trace grouping, and `result['flagged_tones']` for targeted
sweeps with multiple resonances in one tone. The result is also list-like over
`all_resonances`, so existing `for r in result` style code still works.

If `sweep_data=None` (the default), `find_resonances` will call `wideband_sweep()` internally to acquire the data. Any extra keyword arguments are forwarded to `wideband_sweep()`.

### Available Data Formats

The resonance finder can operate on different representations of the S21 data:

| Format | Description |
|--------|-------------|
| `'log_magnitude'` | 20 log10(\|S21\|) - default, good for most cases |
| `'lin_magnitude'` | \|S21\| in linear units |
| `'phase'` | Phase of S21 |
| `'unwrapped_phase'` | Unwrapped phase |
| `'group_delay'` | Group delay (derivative of phase) |
| `'complex_gradient'` | Magnitude of dS21/df |

### Tuning the Finder

Pass `filter_params` and `finder_params` to control the detection sensitivity:

```python
from souk_readout_tools.peak_finder import FilterParams, PeakFinderParams

filt = FilterParams(highpass_edge=0.001, lowpass_edge=0.5, median_kernel_size=51)
peaks = PeakFinderParams(prominence=3.0, min_width=3e3, max_num_peaks=500)

result = client.find_resonances(sweep_data, filter_params=filt, finder_params=peaks)
resonances = result['all_resonances']
```

### Interactive GUI

For interactive resonance finding with visual feedback, use the MKID Finder App:

```bash
souk-mkid-finder-app
```

This launches a PyQt5 GUI where you can load sweep data, adjust filter and peak-finding parameters, visually inspect/select the detected resonances and save the results.

---

## Retuning

Retuning performs a frequency sweep, identifies the resonance frequency in each sweep window, and moves the readout tones to track the resonances:

```python
client.perform_retune(
    centers=[0.800e9, 1.500e9],
    spans=[0.2e6, 0.2e6],
    points=101,
    samples_per_point=100,
    direction='up',
    method='max_gradient',  # or 'min_mag'
    wait=True,              # block until the retune completes
)

# Tones are now placed at the detected resonance frequencies
new_freqs = client.get_tone_frequencies()
```

**Methods:**
- `'max_gradient'` - Finds the frequency with the maximum gradient of the IQ trace (recommended).
- `'min_mag'` - Finds the frequency with the minimum magnitude.

### Tracking Loop

*Not yet implemented.* A continuous tracking loop that periodically retunes tones to follow drifting resonances is planned for a future release.

---

## Fast Frequency Modulation

Fast frequency modulation rapidly dithers each tone across a few probe frequencies (2 or 3 points) and streams the result as ordinary, tagged data, so you can measure each resonator's local `dφ/df` (and, with calibration, dissipation) in real time and track its operating point. Derive the configuration from a sweep, arm it, capture or stream, and demodulate:

```python
from souk_readout_tools import modulation as mod, fitting

sweep = client.parse_sweep_data(client.get_sweep_data())
fits = fitting.batch_fit(sweep, verbose=False)
cfg = mod.params_from_sweep(sweep, n_points=3, samples_per_point=4, fits=fits)
client.enable_modulation(center=cfg['center'], offsets=cfg['offsets'],
                         mod_indices=cfg['mod_indices'],
                         samples_per_point=cfg['samples_per_point'], n_settle=cfg['n_settle'])
data = client.parse_samples(client.get_samples(3000))     # frames tagged with the cycle step
grouped = mod.group_cycles(data, client.get_modulation_state())
result = mod.demodulate(grouped, calibration=cfg['calibration'])   # centred fitted-model frequency/dissipation
client.disable_modulation()
```

See **[doc/frequency_modulation.md](frequency_modulation.md)** for the full user guide, API reference, and internals.

---

## Power Calibration & Optimisation

### Saturation Checks

Before taking data, check for ADC/DAC saturation and DSP overflow:

```python
client.check_input_saturation()    # ADC saturation
client.check_output_saturation()   # DAC saturation
client.check_dsp_overflow()        # DSP pipeline overflow
```

### Auto-Optimisation

The client provides methods to automatically optimise power levels and fix saturation:

```python
client.maximise_tx_power()     # Maximise transmit power without clipping
client.maximise_rx_power()     # Maximise receive power without clipping
client.optimise_tx_snr()       # Optimise TX signal-to-noise without changing power
client.optimise_rx_snr()       # Optimise RX signal-to-noise - minimises rx attenuation
client.fix_dac_saturation()    # Auto-fix DAC clipping
client.fix_adc_saturation()    # Auto-fix ADC clipping
```

`maximise_tx_power()` and `maximise_rx_power()` accept a `headroom_db`
parameter that sets the safety margin below saturation. The TX default is
`2.0 dB`; the RX default is `1.0 dB`:

```python
client.maximise_tx_power(headroom_db=3.0)   # 3 dB below saturation
client.maximise_rx_power(headroom_db=1.0)   # 1 dB below saturation
```

### Tone Power Queries

Query tone powers at different points in the signal chain using `reference_plane`:

```python
# TX power at different reference planes
client.get_tone_powers()                              # at detector (default)
client.get_tone_powers(reference_plane='dac')          # at DAC output
client.get_tone_powers(reference_plane='rf_output')    # at RF frontend output

# RX power prediction from current TX settings and RX calibration
client.get_tone_powers(reference_plane='adc_input')             # at ADC input
client.get_tone_powers(reference_plane='cryostat_output')       # at cryostat output
```

`set_tone_powers()` returns a result dict with `achieved_powers_dbm`, `power_error_db`, and `warnings`:

```python
result = client.set_tone_powers([-20, -25], reference_plane='dac', optimise_dynamic_range=True)
# Warnings (e.g. power clamping, compression) are printed automatically
```

### ADC Calibration Freeze

The RFSoC ADC uses internal interleaved sub-ADCs with a real-time calibration system that continuously adjusts their alignment. While this keeps the ADC performing optimally, the updating calibration introduces signal drifts and noise into the IQ data. During observations, the calibration must be frozen to eliminate this noise source:

```python
# Freeze calibration before observing
client.set_cal_freeze(True)

# ... perform observations ...

# Periodically defrost to let the ADC recalibrate and maintain performance
client.set_cal_freeze(False)
# Wait a few seconds for calibration to settle, then re-freeze
client.set_cal_freeze(True)
```

The calibration should be periodically unfrozen between observations to ensure the ADC performance remains healthy.

---

## Clock Source

The RFSoC PL clocks can be referenced to either the on-board 12.8 MHz oscillator (**internal**) or an external 10 MHz reference (**external**). This is a board-level setting that affects both pipelines.

```python
# Check current source and PLL lock status
client.get_clock_source()    # 'internal' or 'external'
client.get_clock_status()    # per-chip lock status

# Switch to external 10 MHz reference
client.set_clock_source('external')

# Switch back to internal
client.set_clock_source('internal')
```

The clock source is also applied automatically when pushing a config — set `firmware.clock_source` in the config file:

```yaml
firmware:
  clock_source: "external"
```

Clock source and PLL lock status are available via `get_info('clock')`:

```python
clock = client.get_info('clock')
print(clock['source'])      # 'internal' or 'external'
print(clock['all_locked'])  # True if all PLLs are locked
```

Since this is a shared resource, both pipeline configs should specify the same `clock_source` value. See [clock_source.md](clock_source.md) for full details, manual procedures, and troubleshooting.

---

## Timing Status

RFSoC wall-clock and firmware timestamp health are monitored by `ptp4l`,
chrony, and `souk-timing-monitor`. The readout server exposes the monitor
through the normal client API:

```python
timing = client.get_info("timing")
summary = timing["summary"]

print(summary["state"])
print(summary["active_source_type"])
print(summary["ready_for_firmware_sync"])
```

For compact periodic checks, `health_check()` includes the timing state:

```python
health = client.health_check()
print(health["timing_state"], health["timing_ready"])
```

For lower-level debugging, request the raw monitor status:

```python
raw = client.get_timing_status()
print(raw.get("ptp_port_state"), raw.get("chrony_source_type"))
```

Common states include `locked_to_gm`, `ptp_holdover`, `phc_free_run`,
`ntp_synced`, `ntp_holdover`, `free_run`, `initializing`, and `unavailable`.
`ready_for_firmware_sync` is true only when PTP is fresh, locked, and stable.

On the RFSoC, local timing checks are available with:

```bash
souk-test-timing-monitor status
souk-test-timing-monitor stream 10
```

See [timing.md](timing.md) for the `ptp4l`, chrony, timing-monitor, standalone
monitoring, and firmware sync setup.

---

## Pre-Accumulator Snapshots

For high time-resolution data on a single tone, acquire snapshots from the pre-accumulator stage. Each snapshot contains 1024 complex samples at the FFT output rate (before accumulation), which is `acc_len` times faster than the normal sample rate:

```python
snapshots = client.get_accumulator_snapshots(tone_index=0, num_snapshots=10)

# snapshots contains:
#   'snapshots'    - list of complex arrays (1024 samples each)
#   'tone_index'   - which tone was captured
#   'sample_rate'  - sample rate at the pre-accumulator stage
#   'num_snapshots' - number of snapshots acquired
#   'len_snapshot'  - samples per snapshot (1024)
```

This is useful for diagnostics, characterising noise at higher frequencies, or fast acquisition of single-tone data.

### Batch Snapshots

To acquire snapshots across multiple tones (one afer the other) in one call:

```python
# All tones, 20 snapshots each, save to file and plot
batch = client.batch_snapshots(num_snapshots=20, export_file='my_snapshots', plot=True)

# Specific tones only
batch = client.batch_snapshots(tone_indices=[0, 3, 7], num_snapshots=50)

# Access per-tone data
for tidx, snap in batch['results'].items():
    print(f"Tone {tidx}: {snap['snapshots'].shape}")
```

CLI: `souk-batch-snapshots -C config.yaml -n 20 --tones 0 3 7 -P`

For a worked example that combines snapshots with on/off-resonance
timestreams, see [Resonator Drive Tuning and Noise
Measurements](resonator_noise_workflow.md).

---

## Plotting

The `souk_readout_tools.plotting` module provides consistent visualization for all data types. All functions return matplotlib Figure objects.

### Sweep Plots

```python
from souk_readout_tools.plotting import plot_sweep, plot_sweep_iq

# Magnitude and phase vs frequency (default)
fig = plot_sweep(sweep_data, format='magphase', show_errors=True)

# I vs Q complex plane
fig = plot_sweep(sweep_data, format='iq')

# With true RF deembedding (off-resonance → (1, 0))
fig = plot_sweep(sweep_data, format='iq', deembed=True)

# With phase centering (circle centred at origin, off-resonance on negative real axis)
fig = plot_sweep(sweep_data, format='iq', phase_center=True)

# Both: deembed first, then phase-center the result
fig = plot_sweep(sweep_data, format='iq', deembed=True, phase_center=True)

# I and Q vs frequency
fig = plot_sweep(sweep_data, format='iq_vs_f')

# Per-tone grid (one subplot per tone) instead of overlay
fig = plot_sweep(per_tone_sweep, tones=[0, 1, 2], multi_tone='grid')

```

### Timestream Plots

```python
from souk_readout_tools.plotting import plot_timestream, plot_timestream_psd

# I and Q vs time
fig = plot_timestream(parsed_samples, format='iq_vs_t', tones=[0, 1])

# Historical local linear estimate (default; requires sweep data)
fig = plot_timestream(parsed_samples, format='freq_diss', sweep_data=sweep)

# Power spectral density
fig = plot_timestream_psd(parsed_samples, format='freq_diss', sweep_data=sweep)

# Overlay timestream on resonance circle (debugging)
from souk_readout_tools.plotting import plot_timestream_on_resonance
fig = plot_timestream_on_resonance(parsed_samples, sweep, tone_index=0, phase_center=True)
```

### Snapshot and Batch Snapshot Plots

```python
from souk_readout_tools.plotting import plot_snapshots, plot_snapshots_psd, plot_batch_snapshots

# Single tone: time domain (mean, concatenated, or overlay)
fig = plot_snapshots(snap_data, format='iq_vs_t', repetitions='overlay')

# Single tone: averaged PSD with error bars from repetition variance
fig = plot_snapshots_psd(snap_data, method='averaged', show_errors=True)

# Batch: one row per tone with optional PSD column
fig = plot_batch_snapshots(batch_data, format='iq_vs_t', psd=True, psd_method='averaged')
```

---

## Resonator Analysis

### Deembedding and Phase Centering

The `souk_readout_tools.resonator` module provides two independent S21 transforms:

```python
from souk_readout_tools.resonator import (
    deembed, apply_deembed_params,
    phase_center, apply_phase_center_params,
    remove_cable_delay,
)

# True RF deembedding: cable delay + baseline normalisation
# Result: off-resonance at (1, 0), on-resonance near zero positive real
z_deembedded, deembed_params = deembed(frequencies, s21_complex)
# deembed_params contains: tau, baseline

# Phase centering: circle centering + rotation
# Result: circle centred at origin, off-resonance on negative real axis,
# resonance near zero phase
z_centered, pc_params = phase_center(s21_complex)
# pc_params contains: center, radius, rotation_angle

# Apply to timestream data (pre-computed params from sweep)
z_ts_deembedded = apply_deembed_params(z_ts, deembed_params, frequency=tone_freq)
z_ts_centered = apply_phase_center_params(z_ts, pc_params)

# Propagate independent I/Q errors through the same transforms
s21_err = sigma_i + 1j * sigma_q
z_deembedded, err_deembedded, deembed_params = deembed(
    frequencies, s21_complex, s21_err=s21_err)

# Cable delay removal only
z_nodelay, tau = remove_cable_delay(frequencies, s21_complex)
```

### Resonance Fitting

The `souk_readout_tools.fitting` module fits complex S21 sweeps to a
notch/Duffing resonator model. The fitted Q convention is:

```python
Qe = Qc * (1 + 1j * np.tan(phi))
1 / Ql = 1 / Qi + np.real(1 / Qe)
```

Use `Qc` in `initial_guess`, `param_bounds`, and `param_fixed`.
`Qc_abs = abs(Qe)` is returned for reporting, but is not a fitted parameter.

```python
from souk_readout_tools.fitting import (
    fit_resonance, fit_sweep_stack, batch_fit, extract_parameters,
)

# Fit a single resonance. z_err follows the server convention:
# real=sigma_I, imag=sigma_Q.
z_tone = i_tone + 1j * q_tone
z_err = ei_tone + 1j * eq_tone
result = fit_resonance(
    f_tone, z_tone, z_err=z_err, nonlinear=True,
    use_error_weights=True, error_weight_power=0.5,
    subsample=True,
    param_bounds={"Qi": (1e3, 1e8), "phi": (-0.5, 0.5)},
)
print(
    f"fr={result.fr/1e6:.4f} MHz, Ql={result.Ql:.0f}, "
    f"Qi={result.Qi:.0f}, Qc={result.Qc:.0f}, nfev={result.nfev}")

# Fit an already-windowed array stack. Rows are independent fits; if this is
# the same KID over several powers, no previous-row result is used as a seed.
fits_by_power = fit_sweep_stack(
    f_stack, z_stack, z_err_stack=e_stack, nonlinear=True,
    n_jobs=-1, verbose=True, subsample=True,
)

# Batch fit a server sweep dictionary. Targeted sweeps preserve tone metadata
# and skip blind tones by default; full/wide sweeps can auto-detect windows.
fits = batch_fit(
    sweep_data, nonlinear=True, n_jobs=-1, verbose=True,
    use_error_weights=True, error_weight_power=0.5, subsample=True,
)

# Extract to arrays
params = extract_parameters(fits)
print(f"Mean Qi: {np.mean(params['Qi']):.0f}")
```

`verbose=True` prints compact progress with throughput and cumulative
function evaluations. Use `verbose=2` to print one line per completed fit.
Set `min_dip_depth_db` to reject noise-only traces before optimisation. The
check uses the existing empirical dip depth; rejected rows return
`success=False`, `noise_only=True`, and `nfev=0`. Use
`min_dip_depth_db=None` to force a fit.
For ordered repeat measurements where each trace is the same resonator, call
`fit_resonance()` or `fit_resonance_nonlinear()` in a loop and pass the
previous `FitResult` as `initial_guess` if you want chained starting values.

CLI: `souk-find-resonances -C config.yaml --fit -f resonances.txt -P`

### Frequency and Dissipation Conversion

Use the local linearized estimate for compatibility with older analysis, or
build fitted calibrations for exact Möbius conversion. Parse the sweep once and
use `batch_fit()` so tone indices and blind-tone metadata are preserved:

> **Frequency-sign convention.** The low-level fitted `method='mobius'` and
> diagnostic `method='circle'` conversions return probe detuning relative to
> the fitted resonance: `f_probe - f_r`. Positive values mean that the probe is
> above resonance. The more familiar resonator detuning relative to a fixed
> probe is `f_r - f_probe`, with the opposite sign. The high-level timestream
> helper below returns changes in that resonator-side convention so it can be
> compared directly with the historical linearized noise quadrature.

```python
from souk_readout_tools import fitting
from souk_readout_tools.noise import (
    fractional_frequency_and_dissipation_timestreams,
)
from souk_readout_tools.resonator import (
    ResonatorCalibration,
    interpolate_complex_trace,
)

sweep = client.parse_sweep_data(client.get_sweep_data())
fits = fitting.batch_fit(sweep, nonlinear=True, verbose=False)
calibrations = {
    fit.tone_index: ResonatorCalibration.from_fit(fit)
    for fit in fits if fit.success
}

linearized = fractional_frequency_and_dissipation_timestreams(
    parsed_samples, sweep, method='linearized')
mobius = fractional_frequency_and_dissipation_timestreams(
    parsed_samples, sweep, method='mobius', calibrations=calibrations)
circle = fractional_frequency_and_dissipation_timestreams(
    parsed_samples, sweep, method='circle', calibrations=calibrations)

# The plotting helpers expose the same switch.
fig = plot_timestream(
    parsed_samples, format='freq_diss', sweep_data=sweep,
    conversion_method='mobius', calibrations=calibrations)
```

For one fixed tone, the canonical low-level fitted conversion is:

```python
cal = calibrations[tone_index]
probe_detuning_hz, matched_dissipation = cal.convert_raw_iq(
    tone_frequency, z_ts)
tone_sweep_f = sweep['sweep_f'][:, tone_index]
tone_sweep_z = sweep['sweep_i'][:, tone_index] + 1j * sweep['sweep_q'][:, tone_index]
reference_z = interpolate_complex_trace(tone_sweep_f, tone_sweep_z, tone_frequency)
detector_df_hz, delta_matched_dissipation = cal.convert_referenced_raw_iq(
    tone_frequency, z_ts, reference_z)
```

Here `probe_detuning_hz` is the absolute fitted `f_probe - f_r` coordinate.
`detector_df_hz` is the reference-subtracted resonator motion with the opposite
sign, suitable for fixed-tone noise analysis.

The asymmetric linear notch inversion is exact. For fits with non-zero
`anl`, the default path follows the circle inversion with an analytic Duffing
inverse. Use `method='circle'` only when you explicitly want the driven-circle
coordinate and change in signed radial proxy
`Delta(abs(z_centered) / radius - 1)` for diagnostics. The high-level
timestream helper references all three methods to the sweep IQ at the fixed
probe tone, and reports detector resonance motion with the historical sign.
The default fitted and historical linearized converters return
the matched-scale dissipation quadrature `Delta(1 / (2 * Qi))`. Double that
quadrature for the commonly reported physical `Delta(1 / Qi)`.

The established nonlinear fit branch uses a real loaded-linewidth denominator,
so with non-zero `phi` its `anl -> 0` limit is not identical to the asymmetric
linear branch. Changing that convention would require a separate fit-model
migration. For the complete derivation, plots, and loss-coordinate
normalization, see
[`resonator_math_derivations.ipynb`](resonator_math_derivations.ipynb).

### Targeted Resonance Finding

For per-tone sweeps, use targeted mode to find resonances within each tone's bandwidth and flag doubles/triples:

```python
result = client.find_resonances(per_tone_sweep, mode='targeted')

print(f"Total resonances: {len(result['all_resonances'])}")
print(f"Tones with multiple resonances: {result['flagged_tones']}")

# Per-tone results
for t, ress in enumerate(result['per_tone']):
    print(f"Tone {t}: {len(ress)} resonance(s)")
```

---

## Parameter Space Measurements

The `souk_readout_tools.measurement` module records a measurement as a **run
directory**: a top-level `measurement.json` manifest plus the data, analysis,
and plot files it points at.  The manifest is plain JSON and the data model is
plain dataclasses:

- `MeasurementRun` — one run: its `kind`, `parameters`, `metadata`, and a list
  of steps, plus run-level artifacts such as the `client.get_info("all")`
  system-info captures saved at the start and end.
- `MeasurementStep` — one point on the swept axis: the requested `axis` values,
  the `readback` values read from the hardware, free-form `metadata`, and the
  artifacts saved for that step.
- `MeasurementArtifact` — a pointer to one saved file (a sweep `.npz`, a plot
  `.png`, ...), tagged with its `kind` and `role`.

`MeasurementStore` reads and writes those files; it does not drive the
measurement.  The acquisition loop lives with the measurement itself.
`run_power_sweep()` is the worked example: it steps tone power, saves one
targeted sweep per step (rewriting the manifest as it goes so an interrupted
run can still be inspected), and returns the `MeasurementRun`:

```python
from souk_readout_tools import power_sweep as ps

run = ps.run_power_sweep(
    client,
    centers=freqs,
    spans=0.5e6,
    powers_dbm=[-95, -90, -85, -80],
    output_dir="kid_power_sweep",
    follow_dips=True,  # default: pre-center first, then follow between steps
)

loaded = ps.load_power_sweep("kid_power_sweep")
analysis = ps.analyse_power_sweep(
    loaded,
    nonlinear=True,
    n_jobs=-1,
    target_anl=0.01,
)

best_power = analysis["best_power"]
```

To add a new kind of measurement, write a plain function in the same shape as
`run_power_sweep`: build a `MeasurementRun`, loop over your steps (writing the
manifest as you go), save each data product with
`MeasurementStore.save_step_npz_artifact(...)`, and return the run.  The body
of `run_power_sweep` in `power_sweep.py` is a complete template.

The artifact kinds are `sweep`, `timestream`, `accumulator_snapshot`,
`adc_snapshot`, `dac_snapshot`, `fit_results`, `summary_table`, and `plot`.
Accumulator snapshots are treated separately from timestreams because they are
pre-accumulation, high-rate captures for one tone at a time and are not valid
for tone-tone correlation analysis.

For a practical blackbody-load directory convention and an end-to-end
drive-tuning plus on/off-resonance noise workflow, see [Resonator Drive Tuning
and Noise Measurements](resonator_noise_workflow.md).

---

## Dual-Pipeline Operation

The system supports two independent readout pipelines on a single RFSoC board, allowing simultaneous readout of two RF networks. See [dual_pipeline.md](dual_pipeline.md) for full details.

### Quick Summary

1. Create two config files with different `pipeline_id` (0 and 1), different tcp ports, correct RFDC tile/block mappings, and matching default mixerless-module RF channels. Use `copy_template_config()` with `pipeline_id=0` and `pipeline_id=1`.

2. Start two server instances on the RFSoC:
   ```bash
   # Terminal 1
   sudo /home/casper/py3.12-venv/bin/souk-readout-server -p 0 /path/to/config_p0.yaml
   # Terminal 2
   sudo /home/casper/py3.12-venv/bin/souk-readout-server -p 1 /path/to/config_p1.yaml
   ```

3. Connect two clients:
   ```python
   client0 = ReadoutClient(config_file='config_p0.yaml')
   client1 = ReadoutClient(config_file='config_p1.yaml')
   ```

### RFDC Mapping (v7.9+)

| Pipeline | DAC0 (main) | DAC1 (dual-DAC only) | ADC |
|----------|-------------|----------------------|-----|
| 0 | Tile 0 / Block 0 | Tile 0 / Block 2 | Tile 2 / Block 0 |
| 1 | Tile 1 / Block 0 | Tile 1 / Block 2 | Tile 3 / Block 0 |

DAC0 is the primary DAC used in normal operation. DAC1 is only used in dual-DAC mode, which is not currently supported.

### Initialisation Considerations

- `ensure_ready()` on one pipeline will not disrupt the other unless a firmware reprogram is actually required.
- `hard_reset()` on either pipeline wipes both, so run `ensure_ready()` on both afterwards.

---

## Mock Server Mode

For OCS / controller integration testing without RFSoC hardware,
`ReadoutClient` can run against an in-process mock server:

```python
from souk_readout_tools.client.readout_client import ReadoutClient

client = ReadoutClient(mock=True)        # no config / address required
client.ensure_ready()
client.set_tone_frequencies([0.8e9, 1.5e9])
raw = client.get_samples(500)
data = client.parse_samples(raw, num_tones=2)
```

What is emulated:

- Connection setup (defaults to `127.0.0.1:10000` if no config/address is
  given), `push_config()` / `pull_config()`, and the full `get_info()`
  surface.
- A synthetic resonator catalogue with realistic `S21` responses, used to
  back wideband sweeps, targeted sweeps, snapshots, streams, and the new
  G3 stream output.
- Timing, RFDC, tone, and LNA `info` sections so OCS health-checks behave
  the same as against real hardware.

What is **not** emulated:

- Actual firmware programming, real RF behaviour, or hardware-specific
  failure modes. Mock mode is for client-side / pipeline-integration
  testing, not RF validation.

The mock implementation lives in
`souk_readout_tools.client.mock_readout` (`MockReadoutServer`); mock mode
swaps socket traffic for this in-process state object rather than
providing a separate mock client class.

---

## CLI Tools

The following command-line tools are installed with the client:

| Command | Description |
|---------|-------------|
| `souk-connection-test` | Test connectivity to the readout server |
| `souk-wideband_sweep` | Perform a wideband frequency sweep from the command line |
| `souk-batch-snapshots` | Acquire pre-accumulator snapshots across multiple tones |
| `souk-find-resonances` | Find and optionally fit resonances in sweep data |
| `souk-mkid-finder-app` | Launch the MKID resonance finder GUI |
| `souk-mkid-finder` | Same as above (GUI shortcut) |

Server-side commands (installed on the RFSoC):

| Command | Description |
|---------|-------------|
| `souk-readout-server` | Start the readout server (`-p` flag for pipeline ID) |
| `souk-enable-daemon` | Enable selected pipeline systemd daemon(s), default pipeline 0 |
| `souk-enable-daemons` | Enable both pipeline systemd daemons |
| `souk-disable-daemon` | Disable selected pipeline systemd daemon(s), default both |
| `souk-disable-daemons` | Disable both pipeline systemd daemons |
| `souk-restart-daemon` | Restart selected pipeline systemd daemon(s), default pipeline 0 |
| `souk-restart-daemons` | Restart both pipeline systemd daemons |
| `souk-enable-timing` | Install and restart the packaged `ptp4l`, chrony PHC, and timing-monitor setup |
| `souk-timing-monitor` | Run the local PTP/NTP timing monitor |
| `souk-test-timing-monitor` | Query or stream status from `/run/timing-monitor.sock` |
