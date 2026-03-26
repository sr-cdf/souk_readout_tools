# Hardware Validation Plan — v1.1.0

This plan covers all v1.1.0 changes that touch hardware and software. Tests
are grouped into stages by what equipment is needed. Work through them in
order — each stage builds confidence for the next.

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
- [ ] All three commands are registered and print help text

### 0.4  Client entry points (on client machine)

```bash
souk-connection-test --help
souk-wideband_sweep --help
souk-mkid-finder-app --help
```
- [ ] All three commands are registered and print help text

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
- [ ] All imports succeed without errors
- [ ] `souk-peripherals-control` submodule classes import on Python 3.12

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
- [ ] All imports succeed without errors

### 0.7  First-run directory creation

```bash
sudo /home/casper/py3.12-venv/bin/souk-readout-server -p 0
# Ctrl-C after it starts
ls -la ~/.souk_readout_tools/
```
- [ ] `~/.souk_readout_tools/` is created with correct structure:
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
- [ ] `template_config.yaml` paths reference `/home/casper/souk-firmware/...` (not `/home/casper/src/...`)
- [ ] `readout_server.service` references `py3.12-venv` (not `py38venv`)

### 0.8  Systemd daemon setup

```bash
sudo souk-enable-daemon
systemctl cat readout_server_0
```
- [ ] Service installs without errors
- [ ] `ExecStart` path is `/home/casper/py3.12-venv/bin/souk-readout-server -p 0`
- [ ] `sudo systemctl start readout_server_0` starts the server
- [ ] `sudo journalctl -xu readout_server_0` shows server log output

### 0.9  RUDAT USB attenuator discovery (if devices connected)

```python
from souk_readout_tools.server.rudat import find_rudats
rudats = find_rudats()
print(rudats)
```
- [ ] Enumerates connected devices without errors
- [ ] Serials and bus/address info are correct
- [ ] If no RUDAT connected: returns empty dict without crashing

### 0.10  Template config validation

```python
from souk_readout_tools.client.readout_client import copy_template_config
copy_template_config('test_config.yaml', pipeline_id=0)
```
- [ ] Config is created and parseable
- [ ] `firmware.fw_config_file` points to `/home/casper/souk-firmware/...`
- [ ] `rf_frontend.mixerless_module` section contains `attenuator_backend` key
- [ ] `rf_frontend.mixerless_module` section contains commented RUDAT serial fields

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
   info = c.get_system_information()
   ```
3. Verify the returned dict contains:
   - [ ] `souk_readout_tools_version` matches `1.1.0`
   - [ ] `firmware_version` and `fpg_file` match what was loaded
   - [ ] `pipeline_id` matches config
   - [ ] `adc_clk_hz` is sensible (e.g. 2.4576 GHz)
   - [ ] `tone_frequencies`, `tone_amplitudes`, `tone_phases` are lists
   - [ ] `rts_events` dict is present with boolean flags

### 1.2  RFDC RTS event detection

1. With no signal applied (or loopback off), read RTS events:
   ```python
   info = c.get_system_information()
   print(info['rts_events'])
   ```
2. [ ] All `rts_over_*` flags should be `False` in quiescent state
3. If internal loopback is available, set tones at full scale and re-check:
   - [ ] Confirm `rts_over_range` does not spuriously trigger at normal power
4. [ ] Verify `rts_available` is `True` on v7.9+ firmware

### 1.3  Config sync from system

1. Set some tones and modify a firmware default:
   ```python
   c.set_tones([5e9, 5.1e9])
   info = c.sync_config_from_system()
   ```
2. [ ] `c.config['firmware']['defaults']` now contains updated keys
3. [ ] Tone frequencies in synced config match what was set
4. [ ] Calling `sync_config_from_system()` a second time is idempotent

### 1.4  Connection test CLI

```bash
souk-connection-test -C config.yaml
```
- [ ] Connects, prints system information, exits cleanly

### 1.5  Push/pull config round-trip

```python
c = ReadoutClient(config_file='config.yaml')
c.push_config()
c2 = ReadoutClient(address='10.11.11.11', request_port=10000)
c2.pull_config(save_as='pulled_config.yaml')
```
- [ ] `push_config()` succeeds on a freshly started server
- [ ] `pull_config()` returns a valid config
- [ ] Pulled config matches the pushed config (compare key fields)
- [ ] `save_config()` writes a parseable YAML file
- [ ] Creating a new client from the pulled config connects successfully:
  ```python
  c3 = ReadoutClient(config_file='pulled_config.yaml')
  c3.get_server_status()
  ```

### 1.6  Single pre-accumulator snapshot

```python
c.set_tones([5e9])
snap = c.get_accumulator_snapshots(tone_index=0, num_snapshots=5)
```
- [ ] `snap['snapshots'].shape` is `(5, 1024)`
- [ ] `snap['snapshots'].dtype` is `complex128`
- [ ] `snap['sample_rate']` is consistent with `adc_clk_hz / n_fft`
- [ ] Data is not all zeros (if loopback or signal present)

### 1.7  Batch snapshots — single tone

```python
result = c.batch_snapshots(tone_indices=[0], num_snapshots=5, verbose=True)
```
- [ ] `result['results'][0]['snapshots'].shape` is `(5, 1024)`
- [ ] `result['firmware_indices']` is an array with one entry
- [ ] Firmware index is not necessarily 0 (depends on VACC mapping)
- [ ] Verbose output shows correct firmware channel index

### 1.8  Batch snapshots — multiple tones (VACC index mapping)

```python
c.set_tones([5e9, 5.001e9, 5.1e9])  # two tones near same bin + one distant
result = c.batch_snapshots(tone_indices=[0, 1, 2], num_snapshots=3, verbose=True)
```
- [ ] All three tones return data
- [ ] `result['firmware_indices']` shows correct sparse LO indices
- [ ] For tones in the same bin: firmware indices differ by ≥ 6
- [ ] `result['tone_frequencies']` matches what was set
- [ ] Each tone's snapshot data is distinct (not duplicated)

### 1.9  Batch snapshots CLI

```bash
souk-batch-snapshots -C config.yaml --tones 0 1 -n 5 -f test_batch.npz
```
- [ ] Runs without error
- [ ] `test_batch.npz` is created and loadable with `np.load(..., allow_pickle=True)`

### 1.10  Wideband sweep (loopback)

With internal loopback enabled:
```python
sweep = c.wideband_sweep(step_size_hz=50000, verbose=True)
```
- [ ] Progress display updates during sweep (new in v1.1.0)
- [ ] `sweep['sweep_f'].shape[1]` matches expected number of points
- [ ] Magnitude is flat-ish across band (loopback, no resonances)
- [ ] Phase is smooth (phase slope removal working)
- [ ] `sweep['system_information']` dict is populated

### 1.11  Wideband sweep CLI

```bash
souk-wideband_sweep -C config.yaml -P
```
- [ ] Sweep completes
- [ ] Plot window appears showing mag and phase (uses new plotting library)
- [ ] Plot uses `plot_sweep_magphase()` — check axis labels are correct

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
- [ ] `status['enabled']` is `True` (config has `mixerless_module.enabled: true`)
- [ ] `status['hardware']` is `True` (real hardware detected)
- [ ] `status['attenuator_backend']` matches config (`'mixerless'` or `'rudat'`)
- [ ] `status['tx_attenuation_db']` and `status['rx_attenuation_db']` are floats
- [ ] `status['tx_amp_bypass']` and `status['rx_amp_bypass']` are bools
- [ ] `status['tx_total_gain_db']` and `status['rx_total_gain_db']` reflect current settings
- [ ] `status['tx_input_1db_comp_dbm']` is a sensible value (e.g. +10 to +20 dBm)

### 2.2  TX attenuation control

```python
for atten in [0, 5, 10, 15.5, 31.5]:
    c.send_request({'request': 'set_tx_attenuation', 'value': atten})
    readback = c.get_rf_peripheral_status()
    print(f"Set {atten} dB, readback {readback['tx_attenuation_db']} dB")
```
- [ ] Readback matches set value at all points
- [ ] Values outside [0, 31.5] are rejected or clamped
- [ ] 0.5 dB step resolution is respected (mixerless backend)
- [ ] If using a spectrum analyser on the output, confirm power changes by
      the expected amount at each step

### 2.3  RX attenuation control

Same as 2.2 but for RX path:
```python
c.send_request({'request': 'set_rx_attenuation', 'value': 10.0})
```
- [ ] Set/readback match
- [ ] Verify with a known input signal that received power changes correctly

### 2.4  Amplifier bypass control

```python
# TX amp
c.send_request({'request': 'set_tx_amp_bypass', 'bypass': True})
status = c.get_rf_peripheral_status()
assert status['tx_amp_bypass'] == True

c.send_request({'request': 'set_tx_amp_bypass', 'bypass': False})
status = c.get_rf_peripheral_status()
assert status['tx_amp_bypass'] == False
```
- [ ] TX amp bypass toggles correctly
- [ ] RX amp bypass toggles correctly (repeat for RX)
- [ ] Total gain changes appropriately when amp is bypassed vs active
- [ ] `tx_bypass_amp_s21_db` / `rx_bypass_amp_s21_db` config values are used
      in the gain calculation

### 2.5  Sweep with RF frontend — power level check

1. Set known attenuation: TX=10 dB, RX=10 dB, amps active
2. Run wideband sweep through a through cable (no cryostat)
3. [ ] Magnitude level is consistent with calibration chain expectations
4. Change TX attenuation to 20 dB, re-sweep:
   - [ ] Magnitude drops by ~10 dB across band
5. Bypass TX amp, re-sweep:
   - [ ] Magnitude drops by amp gain amount

### 2.6  Calibration chain verification

```python
from souk_readout_tools.calibration import CalibrationChain  # or equivalent
```
- [ ] Renamed parameters `tx_bypass_amp_s21_db` / `rx_bypass_amp_s21_db` are
      used correctly in power budget calculations
- [ ] Setting `rf_frontend.connected: true` in config engages the calibration
      corrections
- [ ] DAC power → detector power conversion is consistent with measured values

### 2.7  Attenuator backend switching

Test that the server correctly initialises with each backend:

1. Set `attenuator_backend: mixerless` in config, restart server:
   - [ ] `get_rf_peripheral_status()` reports `attenuator_backend: 'mixerless'`
   - [ ] Set/get attenuation works via I2C

2. Set `attenuator_backend: rudat` with valid serial numbers, restart server:
   ```yaml
   mixerless_module:
     enabled: true
     attenuator_backend: rudat
     rudat_tx_serial: "12345"
     rudat_rx_serial: "12346"
   ```
   - [ ] `get_rf_peripheral_status()` reports `attenuator_backend: 'rudat'`
   - [ ] Set/get attenuation works via USB
   - [ ] RUDAT resolution and range are reported correctly

3. Set `attenuator_backend: rudat` with invalid serial:
   - [ ] Server raises a clear error at startup (not a silent fallback)

4. Set `enabled: false`:
   - [ ] `get_rf_peripheral_status()` reports `enabled: False`
   - [ ] Attenuation commands are no-ops (no crash)

### 2.8  Config template validation

1. Copy the template config and fill in RF frontend section:
   ```bash
   cp $(python -c "from importlib_resources import files; print(files('souk_readout_tools.data.config') / 'template_config.yaml')") test_config.yaml
   ```
2. [ ] `mixerless_module` section is present with all expected keys
3. [ ] `attenuator_backend` key is present with default `mixerless`
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
c.set_tones([5e9])
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
c.set_tones([5e9, 5.1e9])
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
c.set_tones([5e9])
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
c.set_tones([r.frequency for r in resonances[:5]])
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
- [ ] `Ql` values are physically reasonable (typically 1e3–1e6 for MKIDs)
- [ ] `Qi > Ql` (internal Q should exceed loaded Q)
- [ ] `residual_rms` is small relative to the dip depth
- [ ] Fitted `fr` values agree with peak-finder frequencies to within a linewidth

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
c.set_tones([fit_results[0].fr])
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
c.set_tones(freqs)

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
    c.send_request({'request': 'set_tx_attenuation', 'value': val})

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

c0.set_tones([5e9])
c1.set_tones([6e9])

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
| Template config | 0.10, 2.8 |
| `get_system_information()` | 1.1, 1.10 |
| `check_rfdc_rts_events()` | 1.2 |
| `sync_config_from_system()` | 1.3 |
| `push_config()` / `pull_config()` | 1.5 |
| Pre-accumulator snapshots | 1.6, 1.7, 1.8, 1.9 |
| VACC tone index mapping | 1.8, 4.11 |
| Wideband sweep + progress | 1.10, 1.11, 4.1 |
| RF peripheral control | 2.1–2.5 |
| Attenuator backend switching | 2.7 |
| Calibration renames | 2.6 |
| Plotting: sweep formats | 3.1, 3.2 |
| Plotting: timestream | 3.4, 4.9 |
| Plotting: snapshots | 3.5, 3.6 |
| Plotting: batch snapshots | 3.6, 4.11 |
| Plotting: on-resonance | 3.8, 4.10 |
| Resonator deembedding | 3.2, 3.3, 4.8 |
| Resonance finding (wideband) | 4.2, 4.4 |
| Resonance finding (targeted) | 4.3, 4.11 |
| Resonance fitting | 4.5, 4.6, 4.7 |
| Freq/diss noise | 4.9 |
| ParameterSweep | 4.12 |
| TimedMeasurement | 4.13 |
| ConditionalMeasurement | 4.14 |
| Dual pipeline | 5.1–5.3 |
| CLI: `souk-batch-snapshots` | 1.9 |
| CLI: `souk-find-resonances` | 4.4, 4.7 |
| CLI: `souk-wideband_sweep` | 1.11 |
| CLI: `souk-connection-test` | 1.4 |
