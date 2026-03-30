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
- [Power Calibration & Optimisation](#power-calibration--optimisation)
- [Pre-Accumulator Snapshots](#pre-accumulator-snapshots)
- [Plotting](#plotting)
- [Resonator Analysis](#resonator-analysis)
- [Parameter Space Measurements](#parameter-space-measurements)
- [Dual-Pipeline Operation](#dual-pipeline-operation)
- [CLI Tools](#cli-tools)
- [Changelog & Feature List](#changelog--feature-list)
- [Future Developments](#future-developments)

---

## System Overview

### Architecture

```
   Client Machine                      RFSoC
┌────────────────────┐              ┌──────────────────────────────────────────────┐
│  Python/IPython    │              │  ARM Processing System (PS)                  │
│                    │              │                                              │
│  ReadoutClient 0 ──┼── request────┼─ ReadoutServer 0 ───┐                        │
│                  ──┼── stream ────┼─                    ├── firmware_lib         │
│                    │              │                     │      │                 │
│                    │              │                     │   souk_mkid_readout    │
│  ReadoutClient 1 ──┼── request────┼─ ReadoutServer 1 ───┘  (firmware interface)  │
│                  ──┼── stream ────┼─                           │                 │
│                    │              │                            │                 │
└────────────────────┘              ├────────────────────────────┼─────────────────┤
                                    │  FPGA Programmable Logic (PL)                │
                                    │                            │                 │
                                    │                    ┌───────┴────────┐        │
                                    │                    │  Pipeline 0    │        │
                                    │                    │  Pipeline 1    │        │
                                    │                    └────────────────┘        │
                                    └──────────────────────────────────────────────┘
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
| `firmware_lib.py` | FPGA firmware interface: register access, tone management, sweeps |
| `calibration.py` | RF power/amplitude calibration chain (DAC to detector) |
| `peak_finder.py` | MKID resonance detection algorithms |
| `fitting.py` | Nonlinear resonator model fitting (Khalil notch model) |
| `resonator.py` | Resonance circle deembedding (cable delay, centering, rotation) |
| `plotting/` | Plotting library for sweep, timestream, and snapshot data |
| `measurement.py` | Parameter space measurement framework |
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

The readout server runs on the RFSoC and should already be installed and configured. If you need to set up or reinstall the server, see the [Installation Guide](installation.md#server-installation--setup).

Verify the server is running:

```bash
ssh casper@10.11.11.11
sudo systemctl status readout_server_0
```

### Client

#### 1. Network Setup

Ensure the RFSoC (e.g. `10.11.11.11/24`) is connected and on the same subnet. Manually set the client machine's IP address to `10.11.11.1/24` (or equivalent).

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

Configuration is YAML-based. A template config is bundled with the package.

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

Create a new config from the bundled template:

```python
from souk_readout_tools.client.readout_client import copy_template_config
copy_template_config('my_config.yaml', pipeline_id=0)
```

### Preparing a Config File

Edit the config file with your hardware-specific settings. The most important parameters to set are:

- `rfsoc_host.address` - The RFSoC IP address
- `rfsoc_host.request_port` / `stream_port` - TCP ports (must be unique per pipeline)
- `firmware.fw_config_file` - Path to the firmware config YAML on the RFSoC
- `firmware.pipeline_id` - Pipeline index (0 or 1)
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
- **`pull_config()`** fetches the config and any referenced calibration files from the server **into memory**. Nothing is written to disk unless `save_as` is provided or `save_config()` is called.
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

### Requesting Information

Check the server status and firmware configuration:

```python
client.get_server_status()
client.get_system_information()
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
client.set_tone_powers([-20, -25])

# Detailed power breakdown through the signal chain:
client.get_tone_powers(detailed_output=True)
```

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

# Plot magnitude of the first tone
t = np.arange(len(data['packet_counter'])) / sample_rate
z0 = data['i_data']['0000'] + 1j * data['q_data']['0000']
plt.plot(t, np.abs(z0))
plt.xlabel('Time (s)')
plt.ylabel('|S21|')
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

### Parse Stream Data

```python
data = client.parse_stream('tmp_stream')
t = np.arange(len(data['packet_counter'])) / data['sample_rate']
z0 = data['i_data']['0000'] + 1j * data['q_data']['0000']
plt.plot(t, np.abs(z0))
plt.show()
```

### Triggered Streaming

The server can also stream samples on external trigger pulses:

```python
client.enable_triggered_stream()

# Software trigger for testing:
client.send_fake_trigger()

# Stop:
client.disable_triggered_stream()
```

The packet counter increments every sample regardless of trigger, so trigger times can be inferred from counter gaps.

---

## Frequency Sweeping

Perform a targeted frequency sweep around specific tone centers:

```python
centers = [0.800e9, 1.500e9]
spans = [0.2e6, 0.2e6]        # span per tone in Hz
num_points = 101
samples_per_point = 3          # keep low for fast sweeps

client.perform_sweep(centers, spans, num_points, samples_per_point, direction='up')

# Check progress
client.get_sweep_progress()  # 0.0 to 1.0

# Wait for completion and retrieve data
import time
while client.get_server_status()['latest_sweep_data_valid'] == False:
    time.sleep(1)

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

At the end of the sweep, tones are returned to the center frequencies.

---

## Wideband Sweep

The wideband sweep covers the full RF bandwidth (or a specified sub-band) using many tones swept in parallel. This is useful for surveying the entire band to find resonances before setting individual readout tones:

```python
sweep_data = client.wideband_sweep(
    bandwidth_hz=None,               # None = full bandwidth
    center_freq_hz=None,             # None = center of band
    step_size_hz=10000,              # frequency step size
    num_tones=1024,                  # number of parallel tones
    samples_per_point=10,            # accumulation per point
    tone_powers_dbm=-50,             # per-tone power in dBm (scalar or array), or 'auto'
    remove_phase_slope=True,         # remove linear phase slope
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

The wideband sweep automatically checks for ADC/DAC saturation before proceeding. A CLI tool is also available:

```bash
souk-wideband_sweep
```

---

## Resonance Finding

Use the built-in resonance finder to locate MKID dips in sweep data:

```python
# Find resonances from existing sweep data
resonances = client.find_resonances(sweep_data, data_format='log_magnitude')

# Or let find_resonances perform a wideband sweep automatically
resonances = client.find_resonances()  # calls wideband_sweep() internally

# Each resonance has: frequency, fwhm, q_factor, qc, qi, dip_depth
for r in resonances:
    print(f'{r.frequency/1e9:.6f} GHz, Q={r.q_factor:.0f}, depth={r.dip_depth:.1f} dB')

# Get just the frequency list
freqs = client.find_resonance_frequencies(sweep_data)
```

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
peaks = PeakFinderParams(prominence=3.0, min_width=3, max_num_peaks=500)

resonances = client.find_resonances(sweep_data, filter_params=filt, finder_params=peaks)
```

### Interactive GUI

For interactive resonance finding with visual feedback, use the MKID Finder App:

```bash
souk-mkid-finder-app
```

This launches a PyQt5 GUI where you can load sweep data, adjust filter and peak-finding parameters, and visually inspect the detected resonances.

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
    method='max_gradient'  # or 'min_mag'
)

# Wait for completion
while client.get_server_status()['latest_sweep_data_valid'] == False:
    time.sleep(1)

# Tones are now placed at the detected resonance frequencies
new_freqs = client.get_tone_frequencies()
```

**Methods:**
- `'max_gradient'` - Finds the frequency with the maximum gradient of the IQ trace (recommended).
- `'min_mag'` - Finds the frequency with the minimum magnitude.

### Tracking Loop

*Not yet implemented.* A continuous tracking loop that periodically retunes tones to follow drifting resonances is planned for a future release.

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
client.optimise_tx_snr()       # Optimise TX signal-to-noise
client.optimise_rx_snr()       # Optimise RX signal-to-noise
client.fix_dac_saturation()    # Auto-fix DAC clipping
client.fix_adc_saturation()    # Auto-fix ADC clipping
```

`maximise_tx_power()` and `maximise_rx_power()` accept a `headroom_db` parameter (default 2.0 dB) that sets the safety margin below saturation:

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

# RX power estimation from accumulated IQ data
client.get_rx_tone_powers()                                     # at ADC input (default)
client.get_rx_tone_powers(reference_plane='cryostat_output')    # at cryostat output
```

`set_tone_powers()` returns a result dict with `achieved_powers_dbm`, `power_error_db`, and `warnings`:

```python
result = client.set_tone_powers([-20, -25], optimise_dynamic_range=True)
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

To acquire snapshots across multiple tones in one call:

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

# With deembedding (cable delay removal, circle centering, rotation)
fig = plot_sweep(sweep_data, format='iq', deembed=True)

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

# Frequency and dissipation noise (requires sweep data for gradient)
fig = plot_timestream(parsed_samples, format='freq_diss', sweep_data=sweep)

# Power spectral density
fig = plot_timestream_psd(parsed_samples, format='freq_diss', sweep_data=sweep)

# Overlay timestream on resonance circle (debugging)
from souk_readout_tools.plotting import plot_timestream_on_resonance
fig = plot_timestream_on_resonance(parsed_samples, sweep, tone_index=0, deembed=True)
```

### Snapshot and Batch Snapshot Plots

```python
from souk_readout_tools.plotting import plot_snapshots, plot_snapshots_psd, plot_batch_snapshots

# Single tone: time domain (mean, concatenated, or overlay)
fig = plot_snapshots(snap_data, format='iq_vs_t', repetitions='mean')

# Single tone: averaged PSD with error bars from repetition variance
fig = plot_snapshots_psd(snap_data, method='averaged', show_errors=True)

# Batch: one row per tone with optional PSD column
fig = plot_batch_snapshots(batch_data, format='iq_vs_t', psd=True, psd_method='averaged')
```

---

## Resonator Analysis

### Deembedding

The `souk_readout_tools.resonator` module provides S21 deembedding transforms:

```python
from souk_readout_tools.resonator import deembed, remove_cable_delay

# Full deembedding pipeline
z_deembedded, params = deembed(frequencies, s21_complex)
# params contains: tau, center, radius, rotation_angle

# Cable delay removal only
z_nodelay, tau = remove_cable_delay(frequencies, s21_complex)
```

### Resonance Fitting

The `souk_readout_tools.fitting` module fits resonances to a notch-type (Khalil) model:

```python
from souk_readout_tools.fitting import fit_resonance, batch_fit, extract_parameters

# Fit a single resonance
result = fit_resonance(f_tone, z_tone)
print(f"fr={result.fr/1e6:.4f} MHz, Ql={result.Ql:.0f}, Qi={result.Qi:.0f}")

# Batch fit all resonances in a sweep (auto-detects resonances)
fits = batch_fit(sweep_data, verbose=True)

# Extract to arrays
params = extract_parameters(fits)
print(f"Mean Qi: {np.mean(params['Qi']):.0f}")
```

CLI: `souk-find-resonances -C config.yaml --fit -f resonances.txt -P`

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

The `souk_readout_tools.measurement` module provides tools for repeating measurements across an external parameter. The parameter is abstract — supply set/get callbacks for any controllable or monitored quantity.

### Sweeping a Controllable Parameter

```python
from souk_readout_tools.measurement import ParameterSweep

# Example: sweep TX attenuation
sweep = ParameterSweep(
    client,
    parameter_name='tx_attenuation_db',
    set_parameter=lambda v: client.set_tx_attenuation(v),
    get_parameter=lambda: client.get_rf_peripheral_status().get('result', {}).get('tx_attenuation_db'),
    settle_time=1.0,
)

results = sweep.sweep(
    values=[0, 5, 10, 15, 20],
    measure_func=lambda c: c.wideband_sweep(verbose=False),
)
```

### Timed Measurements

```python
from souk_readout_tools.measurement import TimedMeasurement

timed = TimedMeasurement(
    client,
    parameter_name='temperature_mk',
    get_parameter=lambda: read_thermometer(),  # your function
    interval_s=60.0,
)

# Take 10 measurements, one per minute
results = timed.run(
    measure_func=lambda c: c.wideband_sweep(verbose=False),
    n_points=10,
)
```

### Conditional Measurements

```python
from souk_readout_tools.measurement import ConditionalMeasurement

cond = ConditionalMeasurement(
    client,
    parameter_name='temperature_mk',
    get_parameter=lambda: read_thermometer(),
    condition=lambda t: abs(t - target_temp) < 5,  # within 5 mK
    poll_interval_s=5.0,
)

results = cond.run(
    measure_func=lambda c: c.wideband_sweep(verbose=False),
    target_values=[100, 200, 300, 400],  # target temperatures in mK
    timeout_s=3600,
)
```

---

## Dual-Pipeline Operation

The system supports two independent readout pipelines on a single RFSoC board, allowing simultaneous readout of two RF networks. See [dual_pipeline.md](dual_pipeline.md) for full details.

### Quick Summary

1. Create two config files with different `pipeline_id` (0 and 1), different ports, and correct RFDC tile/block mappings. Use `copy_template_config()` with `pipeline_id=0` and `pipeline_id=1`.

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
| `souk-enable-daemon` | Enable the server as a systemd daemon |
| `souk-disable-daemon` | Disable the server daemon |

---

## Changelog & Feature List

### v1.1.0 (Current)

**RF Peripheral Controller**
- `RFPeripheralController` with two backends: mixerless (I2C) and rudat (USB attenuators).
- `sync_config_from_system()` and `get_rf_peripheral_status()` for reading hardware state.
- Global rename: `tx_amp_s21_db` → `tx_bypass_amp_s21_db` (and rx) across the calibration chain.

**Batch Snapshots**
- `batch_snapshots()` method to acquire pre-accumulator snapshots across multiple tones.
- Exports to `.npz` with metadata including firmware channel indices.
- CLI tool: `souk-batch-snapshots`.

**Plotting Library (`souk_readout_tools.plotting`)**
- `plot_sweep()` — S21 magnitude/phase, I/Q vs frequency, or complex plane, with deembedding and error bars.
- `plot_timestream()` — I/Q, magnitude/phase, or frequency/dissipation vs time.
- `plot_timestream_psd()` — power spectral density of timestream data.
- `plot_timestream_on_resonance()` — overlay timestream points on sweep resonance circle.
- `plot_snapshots()`, `plot_snapshots_psd()`, `plot_batch_snapshots()` — snapshot visualization with per-repetition, averaged, and concatenated modes.
- All formats support optional deembedding via the `resonator` module.
- PSD utilities: `compute_psd()`, `compute_psd_averaged()`, `compute_psd_concatenated()`.

**Resonator Analysis (`souk_readout_tools.resonator`)**
- `remove_cable_delay()` — auto-estimate and remove electrical delay.
- `center_circle()` — Kasa algebraic circle fit.
- `rotate_to_real_axis()` — rotate resonance to negative real axis.
- `deembed()` — full pipeline: delay → center → rotate.
- `apply_deembed_params()` — apply sweep-derived transforms to timestream data.

**Resonance Finding Enhancements**
- `find_resonances(mode='targeted')` — per-tone resonance search with double/triple flagging.
- `flagged_tones` output for tones containing multiple resonances.

**Resonator Fitting (`souk_readout_tools.fitting`)**
- Khalil notch-type resonator model: `S21 = a*exp(jα)*exp(-2πjfτ)*(1 - Ql/|Qc|*exp(jφ)/(1+2jQlΔf/fr))`.
- `fit_resonance()` — single resonance nonlinear least-squares fit.
- `batch_fit()` — automatic detection and fitting of all resonances.
- `extract_parameters()` — extract fitted parameters into arrays.
- CLI tool: `souk-find-resonances` (with `--fit` option).

**Parameter Space Measurements (`souk_readout_tools.measurement`)**
- `ParameterSweep` — step through external parameter values with set/get callbacks.
- `TimedMeasurement` — periodic measurements at fixed time intervals.
- `ConditionalMeasurement` — measure when a monitored parameter meets a condition.
- Abstract `measure_func(client) → dict` pattern works with any readout measurement.
- `save_measurement()` for exporting results.

**System Information**
- `get_system_information()` reports software versions, git info, and RFDC RTS events.
- `check_rfdc_rts_events()` for DAC/ADC overvoltage sticky flag checking.

**Sweep Progress**
- `wait_for_sweep(progress_bar=True)` with ASCII progress bar.
- `wideband_sweep()` prints progress when `verbose=True`.

### v1.0.1

**Dual-Pipeline Support**
- Two independent readout pipelines on a single RFSoC board.
- Per-pipeline config directories, server instances, and client connections.
- 3-level initialisation state machine (programming / shared / pipeline) to avoid disrupting the other pipeline during init.
- `ensure_ready(level=...)` for safe, minimal-disruption initialisation.
- `hard_reset()` for explicit full FPGA reprogram.

**VACC Multitone (v7.9 Firmware)**
- Support for multiple tones per FFT bin via the Vector Accumulator (VACC).
- Inmap semantics for PSB channel mapping (`inmap[lo_index] = fft_bin`).
- Non-contiguous tone index allocation with minimum separation of 6 (dual-port RAM constraint).
- `compute_vacc_tone_indices()` for optimal LO index assignment.

**Wideband Sweep**
- `wideband_sweep()` method for surveying the full RF bandwidth with parallel multi-tone sweeps.
- Automatic saturation checks before sweeping.
- Optional linear phase slope removal.
- CLI tool: `souk-wideband_sweep`.

**Resonance Finding**
- `find_resonances()` and `find_resonance_frequencies()` integrated into the client.
- Multiple data format options (log magnitude, phase, group delay, complex gradient, etc.).
- Configurable filter and peak-finding parameters (`FilterParams`, `PeakFinderParams`).
- Returns `ResonanceResult` objects with frequency, FWHM, Q factor, Qc, Qi, dip depth.
- Interactive GUI: `souk-mkid-finder-app`.

**Pre-Accumulator Snapshots**
- `get_accumulator_snapshots()` for acquiring high time-resolution pre-accumulation data on a single tone (1024 samples per snapshot at FFT output rate).

**Configuration Management**
- `push_config()` / `pull_config()` for transferring configs and calibration files between client and server. Configs and calibration data are held in memory; pushed configs are always saved on the server, local saving is via `save_config()`.
- `save_config()` writes the in-memory config and any pulled calibration files to a local YAML file and `calibrations/` directory.
- `apply_config()` detects changed parameters and applies hardware changes without full reinitialisation.
- Client config files live wherever the user chooses; server uses pipeline-specific directories on the RFSoC.

**Power Calibration & Optimisation**
- `set_tone_powers()` / `get_tone_powers()` with full calibration chain and selectable `reference_plane` (`'dac'`, `'rf_output'`, `'detector'`).
- `get_rx_tone_powers()` for RX power estimation with reference planes (`'accumulator'`, `'adc_input'`, `'cryostat_output'`).
- Saturation detection: `check_input_saturation()`, `check_output_saturation()`, `check_dsp_overflow()`.
- Auto-optimisation: `maximise_tx_power(headroom_db)`, `maximise_rx_power(headroom_db)`, `optimise_tx_snr()`, `optimise_rx_snr()`.
- Auto-fix: `fix_dac_saturation()`, `fix_adc_saturation()`.

**Other**
- `generate_newman_phases()` for optimal crest factor minimisation.
- `set_tones_helper()` convenience method for setting frequencies, powers (dBm), and phases in one call.
- Triggered streaming with `enable_triggered_stream()` and `send_fake_trigger()`.
- Generic parameter access via `set_parameter()` / `get_parameter()`.
- Server daemon management: `souk-enable-daemon` / `souk-disable-daemon`.

### v1.0.0 (Initial Release)

- Client-server architecture with TCP request/stream protocol.
- Basic tone management (set/get frequencies, amplitudes, phases).
- Discrete sample acquisition (`get_samples`, `parse_samples`, `export_samples`).
- Continuous and triggered streaming.
- Frequency sweeping (`perform_sweep`) and retuning (`perform_retune`).
- YAML-based configuration.
- Cross-platform support (Linux and Windows).

---

## Future Developments

Planned for upcoming releases:

- Improved VACC tone backfilling for more efficient LO slot usage.
- Dual-DAC mode support.
- HDF5 export format support.
- Resonator tracking (continuous retune loop with drift correction).
- ADC calibration via loopback measurement.
- Automated version numbering and release workflow.
