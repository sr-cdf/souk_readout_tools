# Calibration Guide

How to calibrate the SOUK readout signal chain for accurate power measurements.

---

## Contents

- [Overview](#overview)
- [Signal Chain Configurations](#signal-chain-configurations)
- [DAC and ADC Calibration](#dac-and-adc-calibration)
- [Calibration File Format](#calibration-file-format)
- [Specifying Calibrations in Config](#specifying-calibrations-in-config)
- [RF Frontend Calibration](#rf-frontend-calibration)
- [RF Peripheral Control](#rf-peripheral-control)
- [Cryostat Chain](#cryostat-chain)
- [Example Workflow](#example-workflow)
- [Reference Planes](#reference-planes)

---

## Overview

The readout system converts between digital amplitude settings and physical power (dBm) at various reference planes in the signal chain. Calibration data describes the gain or loss at each stage so that `set_tone_powers()` and `get_tone_powers()` can work in absolute power units.

The digital signal chain is common to all configurations:

```
Tone amplitude (0 to 1.0)
  -> PSB FFT shift (bit growth control)
  -> PSB scale (global output scaling)
  -> DAC (14 physical bits from 16-bit word)
  -> DUC mixer (fine frequency shifting, 0P7 or 1P0 scale)
  -> VOP (variable output power current)
  -> [RF frontend — depends on module type, see below]
  -> Cryostat (cables, filters, attenuators, LNA)
  -> Detector
```

Each stage has a corresponding calibration parameter in the config file. Stages that are not connected (`rf_frontend.connected: false`, `cryostat.connected: false`) are automatically excluded from the calibration chain. The analog RF frontend varies by hardware — see [Signal Chain Configurations](#signal-chain-configurations) for the three supported module types.

---

## Signal Chain Configurations

The analog path between DAC/ADC and cryostat depends on which RF module is installed. The diagrams below are simplified representations of each module — real implementations may include additional components (bias tees, DC blocks, baluns, etc.) whose losses should be folded into the nearest S21 calibration entry. Config keys for stages not present in your configuration should be set to `0` or `None`.

Dual-DAC mode (DAC0 + DAC1 through a combiner) is not currently supported. All current configurations use a single DAC output, so `tx_combiner_loss_db` and `rx_combiner_loss_db` should be set to `0`.

### Breadboard Model (with mixer)

Uses an external mixer to up/down-convert between IF and RF frequencies.

```
TX: DAC0 ──► IF chain ──► Atten ──► Mixer ──► RF chain ──► Cryostat ──► Detector
                 |           |          |           |            |
             tx_if_s21  tx_attenuator  tx_mixer  tx_rf_s21   input_s21
               _db       _value_db    _conv_       _db         _db
                                      loss_db

RX: ADC ◄── IF chain ◄── Atten ◄── Mixer ◄── RF chain ◄── Cryostat ◄── Detector
                |           |          |           |            |
            rx_if_s21  rx_attenuator  rx_mixer  rx_rf_s21   output_s21
              _db       _value_db    _conv_       _db         _db
                                     loss_db
```

The IF and RF chains include any fixed (non-bypassable) amplifiers, filters, and cables between the DAC/ADC and mixer and the mixer and cryostat. Measure the end-to-end S21 of these paths and enter them as `tx_if_s21_db` / `rx_if_s21_db` / `tx_rf_s21_db` / `rx_rf_s21_db` .

The attenuator on the breadboard is manually set (not programmable). Enter the fixed attenuation value in the config.

Suggested config values (set unused stages to 0):

```yaml
rf_frontend:
  connected: true
  tx_combiner_loss_db: 0        # dual-DAC not supported, set to 0
  rx_combiner_loss_db: 0
  tx_attenuator_value_db: 10.0  # manual attenuator — enter the fixed setting
  rx_attenuator_value_db: 10.0
  tx_if_s21_db: -1.0            # measure IF path S21 (cables, filters)
  rx_if_s21_db: -1.0
  tx_mixer_conversion_loss_db: 7.0   # measure mixer conversion loss at LO freq
  rx_mixer_conversion_loss_db: 7.0
  tx_rf_s21_db: -2.0            # measure RF path S21 (cables, filters, amps)
  rx_rf_s21_db: -2.0
  tx_bypass_amp_s21_db: 0       # no bypass amp in breadboard
  rx_bypass_amp_s21_db: 0
  bypass_amps:
    enabled: false
```

### Direct RF Model (no mixer)

Drives the cryostat directly from the DAC output via filtering. The choice of low-pass or band-pass filter depends on whether Nyquist Zone 1 or 2 is used.

```
TX: DAC0 ──► RF chain (filters, amps, cables) ──► Cryostat ──► Detector
                         |                            |
                     tx_rf_s21                    input_s21
                       _db                          _db

RX: ADC ◄── RF chain (filters, amps, cables) ◄── Cryostat ◄── Detector
                         |                            |
                     rx_rf_s21                    output_s21
                       _db                          _db

         NyqZ 1: low-pass filter    NyqZ 2: band-pass filter
```

The RF chain S21 should include any fixed amplifiers, filters, and cables in the path. Measure end-to-end.

Suggested config values (set unused stages to 0):

```yaml
rf_frontend:
  connected: true
  tx_combiner_loss_db: 0        # dual-DAC not supported, set to 0
  rx_combiner_loss_db: 0
  tx_attenuator_value_db: 0     # no attenuator
  rx_attenuator_value_db: 0
  tx_if_s21_db: 0               # no IF chain
  rx_if_s21_db: 0
  tx_mixer_conversion_loss_db: 0     # no mixer
  rx_mixer_conversion_loss_db: 0
  tx_rf_s21_db: -1.5            # measure end-to-end RF chain S21 (filters, amps, cables)
  rx_rf_s21_db: -1.5
  tx_bypass_amp_s21_db: 0       # no bypass amp
  rx_bypass_amp_s21_db: 0
  bypass_amps:
    enabled: false
```

### RF Mixerless Module

The SOUK RF Mixerless Module adds a variable attenuator (0-31.5 dB in 0.5 dB steps) and a bypassable amplifier, both controlled via I2C. Filter selection depends on Nyquist Zone.

```
TX: DAC0 ──► RF chain (filters, amps, cables) ──► Var. Atten ──► Bypass Amp ──► Cryostat ──► Detector
                    |                                  |               |             |
                tx_rf_s21                         tx_attenuator   tx_bypass_amp    input_s21
                  _db                              _value_db        _s21_db          _db

RX: ADC ◄──  RF chain (filters, amps, cables) ◄── Var. Atten ◄── Bypass Amp ◄── Cryostat ◄── Detector
                    |                                  |               |             |
                rx_rf_s21                         rx_attenuator   rx_bypass_amp    output_s21
                  _db                              _value_db        _s21_db          _db

         NyqZ 1: low-pass filter    NyqZ 2: band-pass filter
```

Suggested config values (set unused stages to 0):

```yaml
rf_frontend:
  connected: true
  tx_combiner_loss_db: 0        # dual-DAC not supported, set to 0
  rx_combiner_loss_db: 0
  tx_attenuator_value_db: 10.0  # set by RFPeripheralController
  rx_attenuator_value_db: 10.0
  tx_if_s21_db: 0               # no IF chain
  rx_if_s21_db: 0
  tx_mixer_conversion_loss_db: 0     # no mixer
  rx_mixer_conversion_loss_db: 0
  tx_rf_s21_db: -1.5            # measure filter + cable S21
  rx_rf_s21_db: -1.5
  tx_bypass_amp_s21_db: 15.0    # auto-updated by RFPeripheralController
  rx_bypass_amp_s21_db: 15.0
  attenuator_backend: "i2c"
  bypass_amps:
    enabled: true
    tx_amp_bypass: false
    rx_amp_bypass: false
  mixerless_module:
    i2c_bus: 0
    channel: 0
```

When `rf_frontend.connected: true`, `tx_attenuator_value_db`, `rx_attenuator_value_db`, `tx_bypass_amp_s21_db`, and `rx_bypass_amp_s21_db` are automatically managed by the `RFPeripheralController`. See [RF Peripheral Control](#rf-peripheral-control) for full details and alternative attenuator backends.

---

## DAC and ADC Calibration

### Definition of full-scale output

For RFDC calibrations, full-scale output is defined under these conditions:

1. **Digital signal**: Full-scale 16-bit complex signal (I and Q both at +/-32767). Achieved with tone amplitude = 1.0, PSB FFT shift = 0b000000000000, PFBSCALE = 2.0.
2. **DUC fine mixer scale**: Set to 0P7 (not 1P0). Setting 1P0 gives ~3 dB more power but risks overflow.
3. **QMC correction**: Disabled, or gain = 1.0 and offset = 0. Maximum gain (1.999) gives ~6 dB more power with distortion.
4. **VOP current**: 20000 uA. This is the maximum rated value for gen3 devices with 2.5V DAC VTT. Higher values (up to 40500 uA) add nonlinearity and are explicitly not supported for RFSoC4x2 and KRM-4ZU47DR.
5. **Inverse sinc filters**: Disabled. Enabling may add ~1 dB near band edges.

### Measurement procedure

1. Set a single tone at a known frequency with all settings as defined above (0 dBFS signal).
2. Measure the output power at the DAC SMA connector with a spectrum analyser, subtracting cable losses.
3. Record the frequency and measured power in a calibration file.

For a broadband calibration, repeat at multiple frequencies across the band.

---

## Calibration File Format

Calibration files are two-column text files:

- Column 1: frequency in Hz
- Column 2: power in dBm corresponding to full-scale digital signal

Lines starting with `#` are comments. Include measurement conditions in comments for traceability.

Example `dac0.txt`:

```
#
# DAC0 calibration, KRM board, Cardiff lab
#
# DAC clock = 4096 MHz, Interpolation = 2x
# Effective sample rate: 2048 MHz
# DUC fine mixer = 1024 MHz, Nyquist Zone = 1
# VOP = 20000 uA, Mixer scale = 0P7
#
# Frequency_Hz  Power_dBm
500e6  -5.9
1000e6 -6.1
1500e6 -6.5
```

The same format is used for `adc_dbm_to_dbfs` (ADC calibration) and for any RF chain S21 measurement files.

---

## Specifying Calibrations in Config

There are three ways to specify calibration values:

**1. Single scalar** (flat across frequency):
```yaml
firmware:
  dac0_dbfs_to_dbm: -6.0
```

**2. Inline frequency-power pairs** (interpolated):
```yaml
firmware:
  dac0_dbfs_to_dbm: [[500e6, -5.9], [1000e6, -6.1], [1500e6, -6.5]]
```

**3. Path to a calibration file**:
```yaml
firmware:
  dac0_dbfs_to_dbm: 'calibrations/dac0.txt'
```

Calibration file paths can be absolute or relative. You do not need to worry about path differences between client and server — `push_config()` automatically resolves local paths, transfers the files to the server's pipeline calibrations directory, and rewrites the paths in the pushed config. Similarly, `pull_config()` fetches referenced calibration files into memory, and `save_config()` writes them to a local `calibrations/` directory next to your config file.

On the server, calibration files are stored in `~/.souk_readout_tools/pipeline_N/calibrations/`. On the client, they live wherever you save them (typically `calibrations/` next to the config file).

The same three options apply to all calibration parameters: `dac0_dbfs_to_dbm`, `dac1_dbfs_to_dbm`, `adc_dbm_to_dbfs`, and the RF frontend S21 fields.

---

## RF Frontend Calibration

When `rf_frontend.connected: true`, the following parameters contribute to the calibration chain. Set parameters to `0` for stages not present in your hardware (see [Signal Chain Configurations](#signal-chain-configurations)):

| Config key | Description | Sign convention | Used by |
|-----------|-------------|----------------|---------|
| `tx_combiner_loss_db` | Combiner insertion loss (TX) | Negative (loss) | None (dual-DAC not supported) |
| `tx_attenuator_value_db` | Attenuator setting (TX) | Positive = attenuation | Breadboard (fixed), Mixerless (programmable) |
| `tx_if_s21_db` | IF chain S21 (TX) | Gain +, loss - | Breadboard |
| `tx_mixer_conversion_loss_db` | Mixer conversion loss (TX) | Positive = loss | Breadboard |
| `tx_rf_s21_db` | RF chain S21 (TX) | Gain +, loss - | All |
| `tx_bypass_amp_s21_db` | Bypass amplifier S21 (TX, auto-updated) | Gain + | Mixerless module |
| `rx_combiner_loss_db` | Combiner insertion loss (RX) | Negative (loss) | None (dual-DAC not supported) |
| `rx_attenuator_value_db` | Attenuator setting (RX) | Positive = attenuation | Breadboard (fixed), Mixerless (programmable) |
| `rx_if_s21_db` | IF chain S21 (RX) | Gain +, loss - | Breadboard |
| `rx_mixer_conversion_loss_db` | Mixer conversion loss (RX) | Positive = loss | Breadboard |
| `rx_rf_s21_db` | RF chain S21 (RX) | Gain +, loss - | All |
| `rx_bypass_amp_s21_db` | Bypass amplifier S21 (RX, auto-updated) | Gain + | Mixerless module |

Each of these can be specified as a scalar, inline pairs, or a file path (same options as DAC calibration).

Measure these values with a VNA or signal source + spectrum analyser. For mixers, measure the conversion loss at the operating LO frequency and IF range.

---

## RF Peripheral Control

The `RFPeripheralController` manages programmable attenuators (and optionally a bypassable amplifier) on the TX and RX paths. It is initialised by the server on startup when `rf_frontend.connected: true`. Two attenuator backends are supported.

### Attenuator backends

The `attenuator_backend` config key (under `rf_frontend`) selects the hardware driver. If omitted, defaults to `i2c`.

#### `i2c` — SOUK RF Mixerless Module (default)

The I2C-controlled mixerless module provides a variable attenuator (0-31.5 dB, 0.5 dB steps) and a bypassable amplifier on each path.

```yaml
rf_frontend:
  connected: true
  attenuator_backend: "i2c"
  bypass_amps:
    enabled: true
    tx_amp_bypass: false   # true = amplifier bypassed
    rx_amp_bypass: false
  mixerless_module:
    i2c_bus: 0
    channel: 0             # pipeline index
```

Requires `smbus2` and the `souk-peripherals-control` submodule on the server.

#### `rudat` — Mini-Circuits RUDAT USB attenuators

Uses two standalone RUDAT USB attenuators (one TX, one RX) for bench testing without the mixerless module. Attenuator-only — no bypass amplifier (amp bypass methods are no-ops, `tx_bypass_amp_s21_db` / `rx_bypass_amp_s21_db` are always 0).

```yaml
rf_frontend:
  connected: true
  attenuator_backend: "rudat"
  rudat_tx_serial: "12345"   # serial number from find_rudats()
  rudat_rx_serial: "67890"
```

Requires `pyusb` and the `rudat` module on the server's `PYTHONPATH`. To discover connected RUDATs and their serial numbers:

```python
from rudat import find_rudats
find_rudats()
```

The RUDAT-6000-30 supports 0-30 dB in 0.25 dB steps. Attenuation limits and step size are read from the hardware automatically.

A udev rule is recommended so the server does not need root for USB access:

```
SUBSYSTEMS=="usb", ATTRS{idVendor}=="20ce", ATTRS{idProduct}=="0023", MODE="0666", GROUP="plugdev"
```

### Behaviour common to all backends

When the peripheral controller is enabled (regardless of backend):

- `tx_attenuator_value_db` and `rx_attenuator_value_db` are programmed to the hardware attenuator on config push or `apply_config()`.
- The in-memory config is updated after every hardware change via `_sync_config()`, so subsequent `get_tone_powers()` / `set_tone_powers()` calls see the correct values.
- `set_tone_powers(optimise_dynamic_range=True)` adjusts the attenuator (and amp bypass if available) automatically.
- `get_system_information()` reports the current peripheral state.
- `get_rf_peripheral_status()` returns attenuator values, bypass state, total path gain, and 1 dB compression points.

### Client API

All peripheral operations are available from the client via the server's TCP interface:

| Client method | Description |
|---|---|
| `get_rf_peripheral_status()` | Full status dict (attenuator values, bypass state, gains, 1dB comp) |
| `set_tx_attenuation(value_db)` | Set TX attenuator |
| `get_tx_attenuation()` | Read TX attenuator |
| `set_rx_attenuation(value_db)` | Set RX attenuator |
| `get_rx_attenuation()` | Read RX attenuator |
| `set_tx_amp_bypass(bypass)` | Set TX amp bypass (I2C only; no-op on RUDAT) |
| `get_tx_amp_bypass()` | Read TX amp bypass state |
| `set_rx_amp_bypass(bypass)` | Set RX amp bypass (I2C only; no-op on RUDAT) |
| `get_rx_amp_bypass()` | Read RX amp bypass state |

After changing attenuator or bypass settings from the client, call `sync_config_from_system()` to update the client's in-memory config, then `save_config()` if you want to persist the new state.

### Disabling peripheral control

For systems without programmable attenuators, leave `attenuator_backend` empty and enter the attenuator values directly in the config as fixed scalars. Keep `rf_frontend.connected: true` so that the RF frontend calibration chain is still applied — setting `connected: false` excludes the entire RF frontend from calibration.

---

## Cryostat Chain

When `cryostat.connected: true`, the `input_s21_db` and `output_s21_db` parameters account for cryostat cable losses, cold attenuators, filters and amplifiers. Measure these end-to-end with a VNA at room temperature and correct for temperature-dependent variation if needed.

---

## Example Workflow

A typical first-time calibration workflow:

1. **DAC calibration**: Measure DAC0 output power at a few frequencies with the conditions above. Create `calibrations/dac0.txt`.

2. **RF frontend calibration**: Measure S21 of each stage (IF path, mixer, RF path, combiner). Enter values in the config file.

3. **Start the server** and verify with `get_tone_powers(detailed_output=True)` that the power breakdown makes sense.

4. **Iterate**: If measured output power doesn't match the predicted value, refine the calibration entries.

```python
# On the client, verify the calibration:
powers, details = client.get_tone_powers(detailed_output=True)
print(details)  # shows per-stage power breakdown
```

---

## Reference Planes

Power can be queried at different points in the signal chain using the `reference_plane` parameter on `get_tone_powers()` and `get_rx_tone_powers()`.

### TX reference planes (`get_tone_powers`)

| `reference_plane` | Description |
|---|---|
| `'dac'` | Power at DAC output (digital full-scale converted to dBm) |
| `'rf_output'` | Power at the RF frontend output (after attenuator, amp, mixer) |
| `'detector'` | Power at the detector (after cryostat chain) — **default** |

```python
# Power at DAC output
dac_powers = client.get_tone_powers(reference_plane='dac')

# Power at detector (default)
det_powers = client.get_tone_powers(reference_plane='detector')

# Full breakdown
powers, details = client.get_tone_powers(detailed_output=True)
```

### RX reference planes (`get_rx_tone_powers`)

Estimates received tone powers from accumulated IQ data using `calibration.calc_adc_input_power()`.

| `reference_plane` | Description |
|---|---|
| `'accumulator'` | Raw accumulated IQ magnitude in dB (no calibration applied) |
| `'adc_input'` | Power at ADC input in dBm — **default** |
| `'cryostat_output'` | Power at cryostat output (before RF frontend RX chain) |

```python
# Estimated power at ADC input
rx_powers = client.get_rx_tone_powers(reference_plane='adc_input')

# Estimated power at cryostat output
cryo_powers = client.get_rx_tone_powers(reference_plane='cryostat_output')
```

This enables ADC calibration via loopback: compare `get_tone_powers(reference_plane='rf_output')` (known TX power) with `get_rx_tone_powers(reference_plane='adc_input')` (estimated RX power) to derive the ADC calibration correction.

---

## Related

- [Tone Power Notes](tone_power_notes.md) for dynamic range tuning
- [Getting Started](getting_started.md) for basic usage
- [Installation - Calibration Files](installation.md#calibration-files) for config file reference
