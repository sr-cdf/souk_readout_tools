# Hardware Validation Plan — v1.1.0

This plan covers all v1.1.0 changes that touch hardware and software. Tests
are grouped into stages by what equipment is needed. Work through them in
order — each stage builds confidence for the next.

Note: this is a historical v1.1.0 validation plan. The current v1.1.1 config
uses nested `rf_frontend.attenuator`, `rf_frontend.mixerless_module`,
`rf_frontend.bypass_amps`, and `cryostat.lna_bias` backend fields; use
`template_config.yaml`, `doc/rf_peripherals.md`, and `doc/lna_bias.md` for
current config key names.

Stage 0 validates the fresh OS image install before any hardware interaction.

---

## Stage 0 — Fresh Image Installation & Smoke Test

These tests validate that the software installs and runs correctly on a
fresh RFSoC OS image (Ubuntu 24.04, Python 3.12). No hardware interaction
beyond SSH access to the board.

### 0.1  Install souk_mkid_readout

```bash
ssh casper@rfsoc
cd /home/casper/souk-firmware/software/control_sw
/home/casper/py3.12-venv/bin/pip install .
```
- [x] Installs without errors on Python 3.12
- [x] `python -c "import souk_mkid_readout"` succeeds

### 0.2  Install souk_readout_tools (server)

```bash
cd /home/casper/souk_readout_tools
git submodule init && git submodule update
/home/casper/py3.12-venv/bin/pip install .
```
- [x] Auto-detects Xilinx platform and installs server components
- [x] `smbus2` installs cleanly on Python 3.12

### 0.3  Server entry points

```bash
souk-readout-server --help
souk-enable-daemon --help
souk-disable-daemon --help
```
- [x] All three commands are registered and print help text

### 0.4  Client entry points (on client machine)

```bash
souk-connection-test --help
souk-wideband_sweep --help
souk-mkid-finder-app --help
```
- [x] All three commands are registered and print help text

### 0.5  Python imports — server side

```bash
sudo /home/casper/py3.12-venv/bin/python -c "
import souk_readout_tools
from souk_readout_tools.server.readout_server import ReadoutServer
from souk_readout_tools.server.rf_peripherals import RFPeripheralController
from souk_readout_tools.server.rudat import find_rudats, Attenuator
from souk_readout_tools.firmware_lib import needs_programming
print('All server imports OK')
"
```
- [x] All imports succeed without errors
- [x] `souk-peripherals-control` submodule classes import on Python 3.12

### 0.6  Python imports — client side

```bash
python -c "
import souk_readout_tools
from souk_readout_tools.client.readout_client import ReadoutClient, copy_template_config
from souk_readout_tools.calibration import CalibrationChain
from souk_readout_tools.peak_finder import PeakFinder
from souk_readout_tools.plotting import plot_sweep_magphase
from souk_readout_tools.resonator import remove_cable_delay
from souk_readout_tools.fitting import batch_fit
from souk_readout_tools.measurement import ParameterSweep, TimedMeasurement
print('All client imports OK')
"
```
- [FAIL ] All imports succeed without errors

CalibrationChain and PeakFinder classes not implmented, try import *:

```bash
python -c "
import souk_readout_tools
from souk_readout_tools.client.readout_client import ReadoutClient, copy_template_config
from souk_readout_tools.calibration import *
from souk_readout_tools.peak_finder import *
from souk_readout_tools.plotting import plot_sweep_magphase
from souk_readout_tools.resonator import remove_cable_delay
from souk_readout_tools.fitting import batch_fit
from souk_readout_tools.measurement import ParameterSweep, TimedMeasurement
print('All client imports OK')
"
```
- [x] All imports succeed without errors



### 0.7  First-run directory creation

```bash
sudo /home/casper/py3.12-venv/bin/souk-readout-server -p 0
# Ctrl-C after it starts
ls -la ~/.souk_readout_tools/
```
- [x] `~/.souk_readout_tools/` is created with correct structure:
  ```
  ~/.souk_readout_tools/
  ├── daemon/
  ├── pipeline_0/
  │   ├── config/
  │   │   ├── template_config.yaml
  │   │   └── default_config.lnk
  │   ├── calibrations/
  │   └── readout_server.service
  ```
- [x] `template_config.yaml` paths reference `/home/casper/souk-firmware/...` (not `/home/casper/src/...`)
- [x] `readout_server.service` references `py3.12-venv` (not `py38venv`)


### 0.8  Systemd daemon setup

```bash
souk-enable-daemon
systemctl cat readout_server_0
```
- [x] Service installs without errors
- [x] `ExecStart` path is `/home/casper/py3.12-venv/bin/souk-readout-server -p 0`
- [x] `sudo systemctl start readout_server_0` starts the server
- [x] `sudo journalctl -xu readout_server_0` shows server log output

### 0.9  RUDAT USB attenuator discovery (if devices connected)

```python
from souk_readout_tools.server.rudat import find_rudats
rudats = find_rudats()
print(rudats)
```
- [X] Enumerates connected devices without errors
- [x] Serials and bus/address info are correct
- [x] If no RUDAT connected: returns empty dict without crashing

DONE - attenuator config moved to rf_frontend level. attenuator_backend: 'i2c' (mixerless module), 'rudat' (USB), or blank (manual).

### 0.10  Template config validation

```python
from souk_readout_tools.config_utils import copy_template_config
copy_template_config(config_file='test_config.yaml', pipeline_id=0, config_id='test', created_by='test')
```
- [X] Config is created and parseable
- [x] YAML comments are preserved in the output file (not stripped by yaml.dump)
- [x] `config.creation_date` is today's date
- [x] `config.config_id` and `config.created_by` match the arguments passed
- [x] `firmware.fw_config_file` points to `/home/casper/souk-firmware/...`
- [x] `firmware.fw_config_file` references dual-pipeline config by default
- [x] `rf_frontend` section contains `attenuator_backend` key
- [x] `rf_frontend` section contains commented RUDAT serial fields
- [x] `rf_frontend.bypass_amps` subsection present with `enabled: false`
- [x] `rf_frontend.mixerless_module` subsection present with I2C settings
- [x] `cryostat.lna_bias` subsection present with `enabled: false`

---

## Stage 1 — RFSoC Connected (loopback or open, no RF frontend)

These tests need only a running server on the RFSoC with firmware loaded.
No external RF hardware required. Internal loopback is useful but not
essential.

### 1.1  Server startup and system information

1. Start the server: `souk-readout-server -p 0 <config>`
2. From the client machine:
   ```python
   c = ReadoutClient(config_file='config.yaml')
   info = c.get_info()
   ```
3. Verify the returned dict contains:
   - [x] `info['versions']['souk_readout_tools_version']` matches `1.1.0`
   - [ ] `info['versions']['souk_mkid_readout_sw_version']` is a version string (software/driver version)
   - [ ] `info['versions']['souk_mkid_readout_fw_version']` is a version string (supported firmware version)
   - [ ] `info['fpga']['fpg_file']` matches what was loaded
   - [x] `info['server']['pipeline_id']` matches config
   - [x] `info['fpga']['adc_clk_hz']` is sensible (e.g. 2.4576 GHz)
   - [x] `info['tones']['frequencies_hz']`, `['amplitudes']`, `['phases_rad']` are lists
   - [x] `info['rfdc']['rts_events']` dict is present with boolean flags
   - [ ] `info['versions']['souk_readout_tools_commit']` is a short commit hash string (or None)
   - [ ] `info['versions']['souk_firmware_commit']` is a short commit hash string (or None)

DONE - git info simplified to commit hash only from fixed paths (/home/casper/souk_readout_tools, /home/casper/souk-firmware). souk_mkid_readout_version renamed to souk_mkid_readout_sw_version. New souk_mkid_readout_fw_version from __fwversion__. firmware_version key removed.

### 1.2  RFDC RTS event detection

1. With no signal applied (or loopback off), read RTS events:
   ```python
   info = c.get_info('rfdc')
   print(info['rfdc']['rts_events'])
   ```
2. [ x] All `rts_over_*` flags should be `False` in quiescent state
3. If internal loopback is available, set tones at full scale and re-check:
   - [x] Confirm `rts_over_range` does not spuriously trigger at normal power
4. [x] Verify `rts_available` is `True` on v7.9+ firmware

DONE: RTS flag checking added to check_input_saturation — over_range treated as saturation warning, over_voltage as error

todo: investigate why check_input_saturation reports +/- 0.5 for full scale input but check_output_saturation reports +/- 1.0 — both use the same 16-bit normalisation so this may be a hardware gain/alignment issue rather than a software bug

DONE: get_adc_snapshot and get_dac_snapshot available in client, server, and firmware_lib

DONE: maximise tx power - the psbscale binary search now uses exponential ramp-up from current value instead of starting at max

DONE: maximise tx/rx power - headroom_db parameter exposed through client/server (default 2.0 dB)

DONE: optimise_tx_snr now records initial powers and verifies preservation after optimisation; optimise_rx_snr documents expected IQ level changes

DONE: set_tone_powers now returns result dict with achieved_powers_dbm, power_error_db, and warnings; client surfaces warnings

DONE: set_tone_powers with optimise_dynamic_range now follows order: maximise amplitudes → fftshift → psbscale → compute required TX level change from calibration chain → TX attenuator / TX amp bypass if available → reduce psbscale for remaining excess power

DONE: get_tone_powers now accepts reference_plane parameter ('dac', 'rf_output', 'detector')

DONE: get_tone_powers now covers the full signal chain with RX reference planes 'accumulator', 'adc_input', 'cryostat_output' in addition to TX planes; RX planes are forward-modelled with calc_accumulated_iq_level from calibration.py


### 1.3  Config sync from system

1. Set some tones and modify a firmware default:
   ```python
   c.set_tones_helper([5e9, 5.1e9])
   info = c.sync_config_from_system()
   ```
2. [x] `c.config['firmware']['defaults']` now contains updated keys
3. [x] Tone frequencies in synced config match what was set
4. [x] Calling `sync_config_from_system()` a second time is idempotent

### 1.4  Connection test CLI

```bash
souk-connection-test -C config.yaml
```
- [x] Connects, prints system information, exits cleanly

### 1.5  Push/pull config round-trip

```python
c = ReadoutClient(config_file='config.yaml')
c.push_config()
c2 = ReadoutClient(address='10.11.11.11', request_port=10000)
c2.pull_config(save_as='pulled_config.yaml')
```
- [x] `push_config()` succeeds on a freshly started server
- [x] `pull_config()` returns a valid config
- [x] Pulled config matches the pushed config (compare key fields)
- [x] `save_config()` writes a parseable YAML file
- [x] Creating a new client from the pulled config connects successfully:
  ```python
  c3 = ReadoutClient(config_file='pulled_config.yaml')
  c3.get_info('server')
  ```

DONE: souk-enable-daemon / souk-disable-daemon docs updated to not use sudo (they call sudo internally)


### 1.6  Single pre-accumulator snapshot

```python
c.set_tones_helper([1.5e9])
snap = c.get_accumulator_snapshots(tone_index=0, num_snapshots=5)
```
- [x] `snap['snapshots'].shape` is `(5, 1024)`
- [x] `snap['snapshots'].dtype` is `complex128`
- [x] `snap['sample_rate']` is consistent with `adc_clk_hz / n_fft`
- [x] Data is not all zeros (if loopback or signal present)

DONE: generate_newman_phases now returns [0] for a single tone instead of NaN from division by zero

DONE: overflow counter bug fixed — now uses unsigned 32-bit modular arithmetic to prevent negative deltas

minor asymmetry in 1.5G tone at very low level

### 1.7  Batch snapshots — single tone

```python
result = c.batch_snapshots(tone_indices=[0], num_snapshots=5, verbose=True)
```
- [x] `result['results'][0]['snapshots'].shape` is `(5, 1024)`
- [x] `result['firmware_indices']` is an array with one entry
- [x] Firmware index is not necessarily 0 (depends on VACC mapping)
- [x] Verbose output shows correct firmware channel index

DONE: maximise_tx_power now calls fix_dac_saturation() on pre-existing saturation instead of raising ValueError

DONE: new _apply_per_bin_scaling helper scales all amplitudes by worst-case bin overlap factor to prevent coherent addition saturation

### 1.8  Batch snapshots — multiple tones (VACC index mapping)

```python
c.set_tones_helper([1.5e9, 1.5001e9, 1.51e9])  # two tones near same bin + one distant
result = c.batch_snapshots(tone_indices=[0, 1, 2], num_snapshots=3, verbose=True)
```
- [x] All three tones return data
- [x] `result['firmware_indices']` shows correct sparse LO indices
- [x] For tones in the same bin: firmware indices differ by ≥ 6
- [x] `result['tone_frequencies']` matches what was set
- [x] Each tone's snapshot data is distinct (not duplicated)

### 1.9  Batch snapshots CLI

```bash
souk-batch-snapshots -C config.yaml --tones 0 1 -n 5 -f test_batch.npz
```
- [x] Runs without error
- [x] `test_batch.npz` is created and loadable with `np.load(..., allow_pickle=True)`

### 1.10  Wideband sweep (loopback)

With internal loopback enabled:
```python
sweep = c.wideband_sweep(step_size_hz=50000, verbose=True)
```
- [x] Progress display updates during sweep (new in v1.1.0)
- [x] `sweep['sweep_f'].shape[1]` matches expected number of points
- [x] Magnitude is flat-ish across band (loopback, no resonances)
- [x] Phase is smooth (phase slope removal working)
- [x] `sweep['info']` dict is populated

### 1.10b  Wideband sweep with auto power (new in v1.1.0)

```python
sweep = c.wideband_sweep(tone_powers_dbm='auto', verbose=True)
```
- [x] `maximise_tx_power()` is called before sweep
- [x] If saturation detected after maximise, auto-fix is attempted
- [x] Sweep completes successfully
- [x] Magnitude levels are higher than unit-amplitude sweep

```python
sweep = c.wideband_sweep(tone_powers_dbm=-60.0, verbose=True)
```
- [x] Tones are set to -60 dBm via `set_tone_powers()`
- [x] Sweep completes with consistent power levels

### 1.10c  RX tone powers (new in v1.1.0)

```python
# Predict received power from current TX settings and RX calibration
rx_powers = c.get_tone_powers(reference_plane='adc_input')
print(f"RX powers at ADC: {rx_powers} dBm")

rx_powers_cryo = c.get_tone_powers(reference_plane='cryostat_output')
print(f"RX powers at cryostat output: {rx_powers_cryo} dBm")
```
- [x] Returns power array matching number of active tones
- [x] `reference_plane='adc_input'` returns sensible ADC-level powers
- [x] `reference_plane='cryostat_output'` includes RX frontend corrections (when connected)
- [x] `reference_plane='accumulator'` returns modelled accumulator dB levels (no calibration)

### 1.11  Wideband sweep CLI

```bash
souk-wideband_sweep -C config.yaml -P
```
- [x] Sweep completes
- [x] Plot window appears showing mag and phase (uses new plotting library)
- [x ] Plot uses `plot_sweep_magphase()` — check axis labels are correct

TODO: plot was saved to a file but the printed log misses dot before the extension

---

## Stage 2 — RF Frontend Connected (no MKIDs)

These tests require the mixerless module or RUDAT attenuators connected
between the RFSoC and the cryostat RF lines (or a through cable for
bench testing).

### 2.1  RF peripheral status query

```python
status = c.get_rf_peripheral_status()
print(status)
```
- [x] `status['enabled']` is `True` (requires both `rf_frontend.connected: true` and `attenuator_backend` set to `'i2c'` or `'rudat'`)
- [x] `status['hardware']` is `True` (real hardware detected)
- [x] `status['attenuator_backend']` matches config (`'i2c'` or `'rudat'`)
- [x] `status['tx_attenuation_db']` and `status['rx_attenuation_db']` are floats
- [x] `status['tx_amp_bypass']` and `status['rx_amp_bypass']` are bools
- [x] `status['tx_total_gain_db']` and `status['rx_total_gain_db']` reflect current settings
- [x] `status['tx_input_1db_comp_dbm']` is a sensible value (e.g. +10 to +20 dBm)

RESOLVED: `souk-find-attenuators` CLI added (server entry point) — discovers RUDAT USB and I2C attenuators, prints serial numbers.

RESOLVED: RUDAT serial numbers now included in `rf_peripheral_status` when using RUDAT backend (`rudat_tx_serial`, `rudat_rx_serial`).

RESOLVED: `channel` field in `rf_peripheral_status` is the I2C MUX channel for the mixerless module. For RUDAT backends it defaults to `pipeline_id` and is informational only. Documented in code.

RESOLVED: `fix_dac_saturation` simplified — halves psb_scale until clear (no binary search), reduced sleep/iteration counts. Much faster.

RESOLVED: `set_tone_powers` with `optimise_dynamic_range=True` now uses a waterfall strategy: maximise digital gain first, then absorb excess power with attenuator > amp bypass > psb_scale reduction. Removed combinatorial planner.

RESOLVED: `maximise_tx_power` uses multi-resolution psb_scale ramp (6dB > 3dB > 1dB > 0.5dB steps) with DAC snapshot estimate for starting value. Removed binary search and complex PSB overflow fallback.

RESOLVED: `maximise_rx_power` simplified — no more bisection or boundary tracking. Steps attenuator/DSA directly based on snapshot headroom estimates.

RESOLVED: `optimise_rx_snr` rewritten to prefer RX attenuator over DSA for gain control (better noise performance). Transfers DSA attenuation to RX attenuator.

RESOLVED: `fix_adc_saturation` uses linear stepping (3dB attenuator, 2dB DSA) instead of binary search. Reduced sleep from 1s to 0.1s.

### 2.2  TX attenuation control

```python
for atten in [0, 5, 10, 15.5, 31.5]:
    c.set_tx_attenuation(atten)
    readback = c.get_rf_peripheral_status()
    print(f"Set {atten} dB, readback {readback['tx_attenuation_db']} dB")
```
- [x] Readback matches set value at all points
- [x] Values outside [0, 31.5] are rejected or clamped
- [x] 0.5 dB step resolution is respected (mixerless backend)
- [x] If using a spectrum analyser on the output, confirm power changes by
      the expected amount at each step

Set 0 dB, readback 0.0 dB
Set 5 dB, readback 5.0 dB
Set 10 dB, readback 10.0 dB
Set 15.5 dB, readback 15.5 dB
Set 31.5 dB, readback 30.0 dB

### 2.3  RX attenuation control

Same as 2.2 but for RX path:
```python
c.set_rx_attenuation(10.0)
```
- [x] Set/readback match
- [x] Verify with a known input signal that received power changes correctly

### 2.4  Amplifier bypass control

```python
# TX amp
c.set_tx_amp_bypass(True)
status = c.get_rf_peripheral_status()
assert status['tx_amp_bypass'] == True

c.set_tx_amp_bypass(False)
status = c.get_rf_peripheral_status()
assert status['tx_amp_bypass'] == False
```
- [ ] TX amp bypass toggles correctly
- [ ] RX amp bypass toggles correctly (repeat for RX)
- [ ] Total gain changes appropriately when amp is bypassed vs active
- [ ] `get_rf_peripheral_status()` reports live `tx_bypass_amp_s21_db` / `rx_bypass_amp_s21_db`
- [ ] `sync_config_from_system()` captures `bypass_amps.tx_amp_bypass` / `rx_amp_bypass` into a local config
- [ ] Bypass commands are only applied when `bypass_amps.enabled: true` in config

when bypass amp is not present, set_bypass_amp(True or False) both succeed but always with result=True

### 2.5  Sweep with RF frontend — power level check

1. Set known attenuation: TX=10 dB, RX=10 dB, amps active
2. Run wideband sweep through a through cable (no cryostat)
3. [x] Magnitude level is consistent with calibration chain expectations
4. Change TX attenuation to 20 dB, re-sweep:
   - [x] Magnitude drops by ~10 dB across band
5. Bypass TX amp, re-sweep:
   - [ ] Magnitude drops by amp gain amount

### 2.6  Calibration chain verification

```python
from souk_readout_tools.calibration import CalibrationChain  # or equivalent
```
- [ ] Renamed parameters `tx_bypass_amp_s21_db` / `rx_bypass_amp_s21_db` are
      used correctly in power budget calculations
- [x] Setting `rf_frontend.connected: true` in config engages the calibration
      corrections
- [x] DAC power → detector power conversion is consistent with measured values

### 2.6b  Power optimisation with RF frontend (updated in v1.1.0)

```python
# maximise_tx_power now accepts headroom_db and uses exponential ramp-up
result = c.maximise_tx_power(headroom_db=2.0)
print(result)
```
- [ ] `maximise_tx_power(headroom_db=2.0)` completes without error
- [ ] Headroom parameter is respected (2 dB margin below saturation)
- [ ] If DAC saturation exists at start, auto-fixes instead of raising ValueError
- [ ] Per-bin scaling applied when tones share FFT bins (check print output)
- [ ] PSB scale uses exponential ramp-up then binary search (not just binary search)

```python
# maximise_rx_power now optimises programmable RX attenuator before DSA
result = c.maximise_rx_power(headroom_db=2.0)
print(result)
```
- [ ] `maximise_rx_power()` returns `rx_attenuation_db` in result dict
- [ ] Programmable RX attenuator is optimised first (when RF frontend connected)
- [ ] Then ADC DSA is optimised
- [ ] Headroom parameter is respected

```python
# get_tone_powers supports reference_plane parameter
powers_dac = c.get_tone_powers(reference_plane='dac')
powers_rf = c.get_tone_powers(reference_plane='rf_output')
powers_det = c.get_tone_powers(reference_plane='detector')
print(f"DAC: {powers_dac[0]:.1f}, RF: {powers_rf[0]:.1f}, Det: {powers_det[0]:.1f} dBm")
```
- [ ] Powers decrease from DAC → RF output → detector (losses along chain)
- [ ] `detailed_output=True` works with all reference planes

```python
# set_tone_powers with optimise_dynamic_range uses improved analog adjustment
result = c.set_tone_powers([-40.0], reference_plane='detector', optimise_dynamic_range=True)
```
- [ ] Analog adjustment computes exact attenuation from calibration chain (no trial-and-error)
- [ ] `psb_scale` is reduced as the last resort when TX attenuator+amp bypass cannot absorb enough excess power
- [ ] `set_tone_powers` returns result dict with `warnings` list
- [ ] Client prints any warnings from the result

### 2.7  Attenuator backend switching

Test that the server correctly initialises with each backend:

1. Set `attenuator_backend: i2c` in config, restart server:
   - [ ] `get_rf_peripheral_status()` reports `attenuator_backend: 'i2c'`
   - [ ] Set/get attenuation works via I2C

2. Set `attenuator_backend: rudat` with valid serial numbers at `rf_frontend` level, restart server:
   ```yaml
   rf_frontend:
     connected: true
     attenuator_backend: "rudat"
     rudat_tx_serial: "12345"
     rudat_rx_serial: "12346"
   ```
   - [ ] `get_rf_peripheral_status()` reports `attenuator_backend: 'rudat'`
   - [ ] Set/get attenuation works via USB
   - [ ] RUDAT resolution and range are reported correctly

3. Set `attenuator_backend: rudat` with invalid serial:
   - [ ] Server raises a clear error at startup (not a silent fallback)

4. Set `connected: false` or remove `attenuator_backend`:
   - [ ] `get_rf_peripheral_status()` reports `enabled: False`
   - [ ] Attenuation commands are no-ops (no crash)

### 2.8  LNA bias control

```python
status = c.get_lna_controller_status()
print(status)
```
- [ ] `status['enabled']` reflects `cryostat.lna_bias.enabled` in config
- [ ] `status['lna_channel']` matches configured channel

If LNA bias board is connected and `enabled: true`:
```python
# Read bias status for this pipeline's channel
bias = c.get_lna_bias_status()
print(f"Remote V: {bias['remote_voltage_v']:.3f}, Current: {bias['bias_current_a']*1e6:.1f} uA")

# Read all 14 channels
all_bias = c.get_lna_bias_status_all()

# Set bias voltage (remote method with feedback)
result = c.set_lna_bias_voltage(0.5)
print(f"Achieved: {result['achieved_voltage_v']:.3f} V")

# Set all channels
result = c.set_lna_bias_voltage_all(0.0)
```
- [ ] `get_lna_bias_status()` returns voltage and current readings
- [ ] `get_lna_bias_status_all()` returns results for all 14 channels
- [ ] `set_lna_bias_voltage()` achieves the requested voltage (within tolerance)
- [ ] `set_lna_bias_voltage(method='local')` sets DAC directly without feedback
- [ ] `set_lna_bias_voltage(blind=True)` skips validation
- [ ] Specifying `channel=N` overrides the configured pipeline channel
- [ ] With `enabled: false` or no LNA board: status reports disabled, commands return gracefully

### 2.9  Config template validation

1. Copy the template config and fill in RF frontend section:
   ```bash
   cp $(python -c "from importlib_resources import files; print(files('souk_readout_tools.data.config') / 'template_config.yaml')") test_config.yaml
   ```
2. [ ] `rf_frontend` section contains `attenuator_backend`, `bypass_amps`, and `mixerless_module` subsections
3. [ ] `attenuator_backend` key is present with default `i2c`
4. [ ] Server starts cleanly with the filled-in config
5. [ ] `RFPeripheralController` initialises without errors

---

## Stage 3 — Through Cable or Passive Device (no MKIDs)

These tests use a known passive device (through cable, fixed attenuator,
or bandpass filter) to validate the analysis chain before going to real
resonators.

### 3.1  Plotting library — sweep formats

Run a sweep through a through cable and test all plot formats:
```python
sweep = c.wideband_sweep()

from souk_readout_tools.plotting import (
    plot_sweep, plot_sweep_iq, plot_sweep_magphase, plot_sweep_iq_vs_f
)
import matplotlib.pyplot as plt

fig = plot_sweep_magphase(sweep, show_errors=True); plt.show()
fig = plot_sweep_iq(sweep); plt.show()
fig = plot_sweep(sweep, format='iq_vs_f'); plt.show()
```
- [ ] Mag/phase plot: magnitude is flat (through cable), phase is smooth
- [ ] Error bars are visible and reasonable in magnitude
- [ ] IQ plot: single loop/arc (cable delay causes rotation)
- [ ] IQ vs freq: smooth I and Q traces
- [ ] All three formats render without errors

### 3.2  Sweep with deembedding

```python
fig = plot_sweep(sweep, format='iq', deembed=True)
plt.show()
```
- [ ] Cable delay removed: IQ trace is a tighter cluster (no large loop)
- [ ] Circle centering brings the data near the origin
- [ ] No crash or NaN in deembedded data

### 3.3  Resonator module — cable delay estimation

```python
from souk_readout_tools.resonator import remove_cable_delay, deembed
import numpy as np

f = np.ravel(sweep['sweep_f'])
z = np.ravel(sweep['sweep_i']) + 1j * np.ravel(sweep['sweep_q'])

z_corr, tau = remove_cable_delay(f, z)
print(f"Estimated cable delay: {tau*1e9:.2f} ns")
```
- [ ] `tau` is positive and physically reasonable for your cable length
      (rule of thumb: ~5 ns/m for coax)
- [ ] Phase of `z_corr` is flat (no linear slope)

### 3.4  Timestream plotting

```python
c.set_tones_helper([5e9])
ts = c.get_samples(num_samples=10000)
parsed = c.parse_samples(ts)

from souk_readout_tools.plotting import (
    plot_timestream, plot_timestream_psd, plot_timestream_on_resonance
)

fig = plot_timestream(parsed, format='iq_vs_t'); plt.show()
fig = plot_timestream(parsed, format='magphase'); plt.show()
fig = plot_timestream(parsed, format='iq'); plt.show()
fig = plot_timestream_psd(parsed, format='iq'); plt.show()
```
- [ ] IQ vs t: two traces (I and Q) vs time, stable (no large drifts)
- [ ] Magphase: magnitude and phase vs time
- [ ] IQ scatter: cluster of points (should be tight for stable signal)
- [ ] PSD: white-ish noise floor, no unexpected spurs

### 3.5  Snapshot plotting

```python
snap = c.get_accumulator_snapshots(0, 10)

from souk_readout_tools.plotting import plot_snapshots, plot_snapshots_psd

fig = plot_snapshots(snap, format='iq_vs_t', repetitions='overlay'); plt.show()
fig = plot_snapshots(snap, format='iq', repetitions='concatenate'); plt.show()
fig = plot_snapshots_psd(snap, method='averaged', show_errors=True); plt.show()
```
- [ ] Overlay mode: 10 traces visible, mutually consistent
- [ ] PSD: error bars from repetition variance are visible
- [ ] Mean repetition mode: `plot_snapshots(snap, repetitions='mean')` works

### 3.6  Batch snapshot plotting

```python
c.set_tones_helper([5e9, 5.1e9])
batch = c.batch_snapshots(num_snapshots=5, plot=True)
```
- [ ] Auto-generated plot shows one row per tone
- [ ] Alternatively, call manually:
  ```python
  from souk_readout_tools.plotting import plot_batch_snapshots
  fig = plot_batch_snapshots(batch, format='iq_vs_t', psd=True); plt.show()
  ```
- [ ] PSD column appears alongside time-domain column
- [ ] Each tone row is labelled with correct frequency

### 3.7  Batch snapshot export and reload

```python
batch = c.batch_snapshots(num_snapshots=5, export_file='test_batch')
# Reload
import numpy as np
data = np.load('test_batch.npz', allow_pickle=True)
print(list(data.keys()))
```
- [ ] File is created and loadable
- [ ] Contains expected keys for reconstruction

### 3.8  Timestream on resonance overlay

```python
# Single-tone sweep + timestream
c.set_tones_helper([5e9])
sweep_1t = c.wideband_sweep(bandwidth_hz=2e6, num_tones=1)
ts = c.get_samples(num_samples=5000)
parsed = c.parse_samples(ts)

fig = plot_timestream_on_resonance(parsed, sweep_1t, tone_index=0)
plt.show()
```
- [ ] Sweep trace (line) and timestream points (scatter) appear on same IQ axes
- [ ] Timestream points cluster on or near the sweep trace
- [ ] With `deembed=True`: circle is centred, resonance on negative real axis

---

## Stage 4 — Real MKID Resonators

These tests require a cryostat with cooled MKID devices coupled to the
readout system. They validate the full analysis pipeline.

### 4.1  Wideband sweep — resonance visibility

```python
sweep = c.wideband_sweep(step_size_hz=10000, verbose=True)
fig = plot_sweep_magphase(sweep, show_errors=True)
plt.show()
```
- [ ] Resonance dips visible in magnitude trace
- [ ] Phase wraps at resonance locations
- [ ] Error bars are small relative to dip depth
- [ ] Sweep covers the expected frequency range for your device

### 4.2  Resonance finding — wideband mode

```python
resonances = c.find_resonances(sweep_data=sweep, mode='wideband')
print(f"Found {len(resonances)} resonances")
for r in resonances[:5]:
    print(f"  {r.frequency/1e6:.3f} MHz, depth={r.dip_depth:.1f} dB, "
          f"Q={r.q_factor:.0f}")
```
- [ ] Number of resonances is consistent with expected device count
- [ ] Frequencies are within the device band
- [ ] No obvious false positives (check against the sweep plot)
- [ ] No obvious missed resonances

### 4.3  Resonance finding — targeted mode

```python
# Set tones on known resonances first
c.set_tones_helper([r.frequency for r in resonances[:5]])
targeted = c.find_resonances(sweep_data=sweep, mode='targeted')
print(f"Flagged tones: {targeted['flagged_tones']}")
for i, per_tone in enumerate(targeted['per_tone']):
    print(f"  Tone {i}: {len(per_tone)} resonances found")
```
- [ ] `per_tone` returns one list per active tone
- [ ] Most tones find exactly 1 resonance (the one they're set to)
- [ ] `flagged_tones` correctly identifies tones with 0 or >1 resonances
- [ ] `all_resonances` is the union of all per-tone results

### 4.4  Resonance finding CLI

```bash
souk-find-resonances -C config.yaml --prominence 2.0 -P
```
- [ ] Sweep runs, resonances found, table printed
- [ ] Plot shows sweep with green markers at resonance locations
- [ ] Marker positions match the resonance frequencies in the table

### 4.5  Resonance fitting

```python
from souk_readout_tools.fitting import batch_fit, extract_parameters

fit_results = batch_fit(sweep, verbose=True)
params = extract_parameters(fit_results)

print(f"Fitted {len(fit_results)} resonances")
for r in fit_results[:5]:
    print(f"  fr={r.fr/1e6:.4f} MHz, Ql={r.Ql:.0f}, "
          f"Qi={r.Qi:.0f}, Qc={r.Qc_abs:.0f}, "
          f"rms={r.residual_rms:.2e}, success={r.success}")
```
- [ ] Most fits converge (`success=True`)
- [ ] `Ql` values are physically reasonable (typically 1e3-1e6 for MKIDs)
- [ ] `Qi > Ql` (internal Q should exceed loaded Q)
- [ ] `residual_rms` is small relative to the dip depth
- [ ] Fitted `fr` values agree with peak-finder frequencies to within a linewidth
- [ ] `FitResult.iq_center` and `FitResult.iq_radius` are populated
- [ ] `FitResult.anl` is 0.0 for linear fits

### 4.5b  Nonlinear resonance fitting (new in v1.1.0)

```python
from souk_readout_tools.fitting import batch_fit

# Nonlinear (Duffing) model — useful at high drive power
nl_results = batch_fit(sweep, nonlinear=True, sweep_direction='up', verbose=True)
for r in nl_results[:5]:
    print(f"  fr={r.fr/1e6:.4f} MHz, Ql={r.Ql:.0f}, Qi={r.Qi:.0f}, "
          f"anl={r.anl:.2e}, rms={r.residual_rms:.2e}")
```
- [ ] Nonlinear fits converge
- [ ] `anl` values are small and positive (typically < 0.1 for low-power)
- [ ] Fit quality is comparable to or better than linear fits
- [ ] `sweep_direction='down'` produces different `anl` values at high power

### 4.5c  ResonatorCalibration and ToneConverter (new in v1.1.0)

```python
from souk_readout_tools.resonator import ResonatorCalibration

# Build from a fit result
cal = ResonatorCalibration.from_fit(fit_results[0])
print(f"fr={cal.fr/1e6:.4f} MHz, Ql={cal.Ql:.0f}, tau={cal.tau*1e9:.2f} ns")

# Deembed sweep data
z_de = cal.deembed_sweep(fit_results[0].f_data, fit_results[0].z_data)

# Convert to frequency/dissipation
df, dd = cal.to_frequency_dissipation(z_de)
print(f"df range: {np.min(df):.0f} to {np.max(df):.0f} Hz")

# Build a ToneConverter for real-time readout
convert = cal.tone_converter(fit_results[0].fr)
ts = c.get_samples(num_samples=100)
parsed = c.parse_samples(ts)
key = sorted(parsed['i_data'].keys())[0]
z_raw = parsed['i_data'][key] + 1j * parsed['q_data'][key]
df_ts, dd_ts = convert(z_raw)
```
- [ ] `ResonatorCalibration.from_fit()` builds without error
- [ ] `ResonatorCalibration.from_sweep()` builds from raw data
- [ ] `deembed_sweep()` produces a centered, rotated resonance circle
- [ ] `to_frequency_dissipation()` returns sensible df, dd values
- [ ] `ToneConverter` produces same results as full deembed + convert pipeline
- [ ] `deembed_params` property is compatible with `apply_deembed_params()`

### 4.6  Fit quality inspection

```python
# Plot a single fit
import matplotlib.pyplot as plt
from souk_readout_tools.fitting import _s21_notch

r = fit_results[0]
f_fine = np.linspace(r.f_data[0], r.f_data[-1], 1000)
z_model = _s21_notch(f_fine, r.fr, r.Ql, r.Qc_abs, r.phi, r.a, r.alpha, r.tau)

fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
ax1.plot(r.f_data/1e6, 20*np.log10(np.abs(r.z_data)), '.', ms=2, label='Data')
ax1.plot(f_fine/1e6, 20*np.log10(np.abs(z_model)), 'r-', label='Fit')
ax1.set_xlabel('Frequency (MHz)'); ax1.set_ylabel('|S21| (dB)')
ax1.legend()

ax2.plot(r.z_data.real, r.z_data.imag, '.', ms=2, label='Data')
ax2.plot(z_model.real, z_model.imag, 'r-', label='Fit')
ax2.set_aspect('equal'); ax2.set_xlabel('I'); ax2.set_ylabel('Q')
ax2.legend()
plt.suptitle(f'fr={r.fr/1e6:.4f} MHz, Ql={r.Ql:.0f}')
plt.tight_layout(); plt.show()
```
- [ ] Model traces through the data points in both mag and IQ views
- [ ] Resonance circle in IQ is well-described by the model
- [ ] No systematic residual pattern (would indicate model mismatch)

### 4.7  Resonance fitting CLI

```bash
souk-find-resonances -C config.yaml --fit -f resonances.txt -P
```
- [ ] Table shows fitted Ql, Qi, Qc, residual for each resonance
- [ ] `resonances.txt` is written with correct column headers
- [ ] File is tab-separated and machine-parseable
- [ ] Plot shows sweep with fitted resonance markers

### 4.8  Deembedding on real resonance data

```python
from souk_readout_tools.resonator import deembed

f = np.ravel(sweep['sweep_f'])
z = np.ravel(sweep['sweep_i']) + 1j * np.ravel(sweep['sweep_q'])

z_de, params = deembed(f, z)
print(f"Cable delay: {params['tau']*1e9:.2f} ns")
print(f"Circle center: {params['center']}")
print(f"Rotation angle: {params['rotation_angle']:.3f} rad")

fig = plot_sweep(sweep, format='iq', deembed=True)
plt.show()
```
- [ ] Cable delay is consistent with known cable length
- [ ] Deembedded IQ plot shows resonance circles centred near origin
- [ ] Resonance dips point toward negative real axis after rotation

### 4.9  Frequency and dissipation noise measurement

```python
# Set tone on a resonance
c.set_tones_helper([fit_results[0].fr])
ts = c.get_samples(num_samples=50000)
parsed = c.parse_samples(ts)

# Need a per-tone sweep around this resonance
sweep_1t = c.wideband_sweep(
    bandwidth_hz=fit_results[0].fr / fit_results[0].Ql * 20,
    center_freq_hz=fit_results[0].fr,
    num_tones=1
)

fig = plot_timestream(parsed, format='freq_diss', sweep_data=sweep_1t)
plt.show()

fig = plot_timestream_psd(parsed, format='freq_diss', sweep_data=sweep_1t)
plt.show()
```
- [ ] Fractional frequency noise trace is visible and centred near zero
- [ ] Dissipation noise trace is visible and centred near zero
- [ ] PSD shows expected 1/f + white noise shape
- [ ] No NaN or inf values in the output

### 4.10  Timestream on resonance — real resonator

```python
fig = plot_timestream_on_resonance(parsed, sweep_1t, tone_index=0, deembed=True)
plt.show()
```
- [ ] Sweep trace shows the resonance circle
- [ ] Timestream points cluster on the circle at the tone frequency
- [ ] Deembedded view: circle is centred, points are on expected arc position

### 4.11  Multi-tone operation with real resonators

```python
# Set tones on several resonances
freqs = [r.fr for r in fit_results[:10]]
c.set_tones_helper(freqs)

# Batch snapshots
batch = c.batch_snapshots(num_snapshots=10, verbose=True)
fig = plot_batch_snapshots(batch, psd=True)
plt.show()

# Targeted find
targeted = c.find_resonances(sweep_data=sweep, mode='targeted')
print(f"Flagged: {targeted['flagged_tones']}")
```
- [ ] All tones acquire data successfully
- [ ] Firmware indices are correctly mapped (no collisions)
- [ ] Batch snapshot PSDs show individual resonator noise
- [ ] Targeted find correctly identifies one resonance per tone
- [ ] Any flagged tones are genuinely problematic (collisions, weak resonances)

### 4.12  Parameter sweep — attenuation vs Q

```python
from souk_readout_tools.measurement import ParameterSweep
from souk_readout_tools.fitting import batch_fit

def set_atten(val):
    c.set_tx_attenuation(val)

def measure(client):
    sweep = client.wideband_sweep(verbose=False)
    fits = batch_fit(sweep, verbose=False)
    return {'sweep': sweep, 'fits': fits}

ps = ParameterSweep(c, 'tx_attenuation_dB', set_parameter=set_atten)
results = ps.sweep(values=[0, 5, 10, 15, 20], measure_func=measure)

# Check Q vs power
for pt in results:
    fits = pt.data['fits']
    if fits:
        print(f"Atten={pt.parameter_value} dB: "
              f"Ql={fits[0].Ql:.0f}, Qi={fits[0].Qi:.0f}")
```
- [ ] ParameterSweep runs without errors
- [ ] Attenuation is set correctly at each step (verify with `get_rf_peripheral_status()`)
- [ ] Q factors change with power as expected (Qi should increase at lower power for MKIDs)
- [ ] All measurement points contain valid sweep and fit data

### 4.13  Timed measurement — stability monitoring

```python
from souk_readout_tools.measurement import TimedMeasurement

def measure_tone(client):
    ts = client.get_samples(num_samples=1000)
    parsed = client.parse_samples(ts)
    key = sorted(parsed['i_data'].keys())[0]
    return {
        'i_mean': np.mean(parsed['i_data'][key]),
        'q_mean': np.mean(parsed['q_data'][key]),
    }

tm = TimedMeasurement(c, interval_s=10)
results = tm.run(measure_func=measure_tone, n_points=6)

# Plot drift
i_vals = [r.data['i_mean'] for r in results]
times = [r.timestamp - results[0].timestamp for r in results]
plt.plot(times, i_vals, 'o-')
plt.xlabel('Time (s)'); plt.ylabel('I mean'); plt.show()
```
- [ ] Measurements are taken at the requested interval
- [ ] Results show realistic drift/stability for the system
- [ ] `save_measurement(results, 'stability.npz')` saves and reloads correctly

### 4.14  Conditional measurement

```python
from souk_readout_tools.measurement import ConditionalMeasurement

# Example: measure whenever temperature crosses a threshold
# (replace get_temperature with your actual thermometry function)
def get_temperature():
    # Read from your cryostat monitoring system
    return read_thermometer_mK()

cm = ConditionalMeasurement(
    c, 'temperature_mK', get_temperature,
    condition=lambda t: True,  # or a real condition
    poll_interval_s=5.0
)
results = cm.run(measure_func=measure_tone, n_points=3, timeout_s=60)
```
- [ ] Polling works at the specified interval
- [ ] Condition triggers correctly
- [ ] Timeout terminates the run if condition is never met

---

## Stage 5 — Dual-Pipeline Validation

Only needed if running v7.9+ dual-pipeline firmware with two server
instances.

### 5.1  Independent server instances

1. Start two servers:
   ```bash
   souk-readout-server -p 0 config_p0.yaml
   souk-readout-server -p 1 config_p1.yaml
   ```
2. [ ] Both servers start without port conflicts
3. [ ] Each uses its own pipeline-specific calibration directory

### 5.2  Independent operation

```python
c0 = ReadoutClient(config_file='config_p0.yaml')
c1 = ReadoutClient(config_file='config_p1.yaml')

c0.set_tones_helper([5e9])
c1.set_tones_helper([6e9])

sweep0 = c0.wideband_sweep()
sweep1 = c1.wideband_sweep()
```
- [ ] Sweeps run independently without interference
- [ ] Each pipeline reports correct `pipeline_id` in system info
- [ ] RF peripheral status (if connected) is per-pipeline

### 5.3  Cross-pipeline safety

- [ ] Calling `sync_config_from_system()` on one pipeline does not affect
      the other
- [ ] Batch snapshots on pipeline 0 do not disturb pipeline 1's tones
- [ ] System information reports correct RFDC tile/block mapping per pipeline

---

## Quick Reference — What Each Test Validates

| Feature | Tests |
|---------|-------|
| Fresh install (server) | 0.1, 0.2, 0.3, 0.5 |
| Fresh install (client) | 0.4, 0.6 |
| First-run directory creation | 0.7 |
| Systemd daemon setup | 0.8 |
| RUDAT driver | 0.9 |
| Template config | 0.10, 2.9 |
| `get_info()` | 1.1, 1.10 |
| `check_rfdc_rts_events()` | 1.2 |
| `sync_config_from_system()` | 1.3 |
| `push_config()` / `pull_config()` | 1.5 |
| Pre-accumulator snapshots | 1.6, 1.7, 1.8, 1.9 |
| VACC tone index mapping | 1.8, 4.11 |
| Wideband sweep + progress | 1.10, 1.11, 4.1 |
| Wideband sweep auto power | 1.10b |
| RX tone powers | 1.10c |
| RF peripheral control | 2.1-2.5 |
| Power optimisation | 2.6b |
| Attenuator backend switching | 2.7 |
| LNA bias control | 2.8 |
| Calibration renames | 2.6 |
| Plotting: sweep formats | 3.1, 3.2 |
| Plotting: timestream | 3.4, 4.9 |
| Plotting: snapshots | 3.5, 3.6 |
| Plotting: batch snapshots | 3.6, 4.11 |
| Plotting: on-resonance | 3.8, 4.10 |
| Resonator deembedding | 3.2, 3.3, 4.8 |
| ResonatorCalibration | 4.5c |
| Resonance finding (wideband) | 4.2, 4.4 |
| Resonance finding (targeted) | 4.3, 4.11 |
| Resonance fitting (linear) | 4.5, 4.6, 4.7 |
| Resonance fitting (nonlinear) | 4.5b |
| Freq/diss noise | 4.9 |
| ParameterSweep | 4.12 |
| TimedMeasurement | 4.13 |
| ConditionalMeasurement | 4.14 |
| Dual pipeline | 5.1-5.3 |
| CLI: `souk-batch-snapshots` | 1.9 |
| CLI: `souk-find-resonances` | 4.4, 4.7 |
| CLI: `souk-wideband_sweep` | 1.11 |
| CLI: `souk-connection-test` | 1.4 |
