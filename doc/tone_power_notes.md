# Tone Power and Drive Tuning Notes

Notes on tone power dynamic range, RF resonator drive power, and related tuning considerations.

---

## Contents

- [Signal Chain Overview](#signal-chain-overview)
- [Dynamic Range Budget](#dynamic-range-budget)
- [Setting Tone Powers](#setting-tone-powers)
- [Resonator Drive Power](#resonator-drive-power)
- [Drive Power Tuning](#drive-power-tuning)

---

## Signal Chain Overview

The transmit signal chain from digital tone generation to the detector is:

```
Tone amplitude (0 to 1.0)
  → PSB FFT shift schedule (bit growth control)
  → PSB scale (global output scaling)
  → DAC (digital-to-analog, 14 physical bits from 16-bit word)
  → DUC mixer (fine frequency shifting, 0P7 or 1P0 scaling)
  → VOP (variable output power, current setting in uA)
  → RF frontend (mixer, amplifiers, attenuators, combiner)
  → Cryostat (input cable, filters, attenuators)
  → Detector
```

Each stage contributes gain or loss. The calibration chain in `calibration.py` accounts for all of these when converting between amplitude settings and physical power in dBm.

---

## Dynamic Range Budget

### DSP parameters

Three digital parameters control the signal level through the polyphase synthesis filterbank before it reaches the DAC. These affect DSP overflow (integer overflow in the FPGA), which is distinct from DAC saturation.

- **Tone amplitude** (0 to 1.0): Per-tone scaling factor applied to each LO. Sets the relative power between tones. Maximising amplitudes (scaling the largest tone to ~1.0) uses the full dynamic range of the LO.
- **PSB FFT shift**: Controls bit growth through the polyphase synthesis filterbank. Each shift stage either passes or attenuates by 2x. More shifts (more 1-bits in the shift schedule) preserve dynamic range but reduce output level. Fewer shifts increase output level but risk integer overflow in the PSB.
- **PSB scale**: A global scaling factor applied after the filterbank, before the DAC. Fine-tunes how much of the DAC's full-scale range is used.

The `maximise_tx_power()` function optimises all three in order: amplitudes first (scale up to ~1.0), then FFT shift (sweep from safest to highest power, stop at first DSP overflow), then PSB scale (exponential ramp-up then binary search, stop before DSP overflow or DAC saturation).

Both `maximise_tx_power()` and `maximise_rx_power()` accept a `headroom_db` parameter (default 2.0 dB) that sets the safety margin below the overflow/saturation point. If pre-existing DAC saturation is detected, `maximise_tx_power()` automatically calls `fix_dac_saturation()` before proceeding. `maximise_tx_power()` also accepts `compression_headroom_db`; when set, it keeps the total multitone power into the TX frontend below the input-referred 1 dB compression point by that margin.

**Per-bin coherent addition**: When multiple tones map to the same FFT bin, the vector accumulator (VACC) sums their amplitudes coherently. If the sum exceeds 1.0, the VACC overflows. The power management functions (`maximise_tx_power`, `optimise_tx_snr`, `set_tone_powers`) automatically detect shared bins and scale **all** amplitudes down by the worst-case bin overlap factor to preserve relative powers.

### Blind tones and power limits

Blind tones are ordinary firmware tones used for off-resonance gain/phase
monitoring. They count in every digital and analog power constraint:

- VACC shared-bin limits include blind tones and regular tones together.
- PAPR and DAC saturation depend on the combined waveform.
- `set_tone_powers()`, `maximise_tx_power()`, and
  `optimise_dynamic_range=True` treat blind tones as active tones.
- Total TX frontend power and compression checks include blind-tone power.

Because they share the same waveform, generate phase offsets over the full
active tone list after adding blind tones:

```python
freqs = client.get_tone_frequencies()
phases = client.generate_newman_phases(freqs)
client.set_tone_phases(phases)
```

For interactive setup, `set_blind_tones(..., powers_dbm=...)` snapshots the
current regular tones, appends/replaces the blind tones, preserves the
existing regular tone powers, and sets the requested blind-tone powers in the
same calibrated power operation. If you later call `set_tone_powers()` directly,
pass targets for all active tones, or split the arrays with
`get_tone_metadata()` / `get_blind_tone_indices()`.

Avoid placing blind tones on an exactly regular grid. Regular grids make many
intermodulation products land on the same frequencies, so the products can add
coherently into larger spurs. `client.suggest_blind_frequencies()` starts from
approximately even coverage across the band but jitters the target positions by
default while still enforcing minimum spacing from resonances and other blind
tones.

### DAC output

The DAC has 14 physical bits but is addressed with a 16-bit word. The full-scale output power depends on the VOP current setting and the DUC mixer scaling mode. DAC saturation is detected by `check_output_saturation()`, which captures DAC snapshots and checks whether the waveform approaches full-scale — this is independent of DSP overflow (checked by `check_dsp_overflow()`).

Key constraints:
- **VOP current**: Default and maximum rated value is 20000 uA for gen3 devices with 2.5V DAC VTT. Values above 20000 uA (up to the hardware max of 40500 uA) add nonlinearity and are not recommended.
- **DUC mixer scale**: Use `0P7` (not `1P0`) to avoid overflow at the mixer output. The `1P0` mode provides ~3 dB more power but risks clipping.

### Multitone power sharing

When driving N tones simultaneously, each tone's contribution to the total waveform amplitude scales as ~1/sqrt(N) on average (for random phases). The peak-to-average power ratio (PAPR, or crest factor) depends on the phase relationship between tones.

Use Newman phases (`client.generate_newman_phases(freqs)`) to minimise crest factor. This allows higher per-tone power before the composite waveform clips the DAC.

To check the PAPR for a given tone configuration, use the crest factor calculator in `firmware_lib`:

```python
from souk_readout_tools.firmware_lib import estimate_papr_db

papr = estimate_papr_db(freqs, amps, phases, sample_rate)
```

This simulates the time-domain composite waveform and returns the PAPR in dB. Useful for verifying that phase choices and tone arrangements keep the crest factor low before committing settings to hardware.

---

## Setting Tone Powers

### By amplitude (relative)

Tone amplitudes are linear scaling factors from 0 to 1.0, relative to the LO full-scale:

```python
client.set_tone_amplitudes([0.5, 0.3])
```

This sets relative power levels between tones but does not control absolute power.

### By power in dBm (absolute)

The calibrated power interface accounts for the full signal chain:

```python
# Simple mode — adjusts tone amplitudes only, keeps current PSB/analog settings
client.set_tone_powers([-90, -95], reference_plane='detector')
```

The `reference_plane` parameter controls where the target power is specified:
- `'dac'` — at the DAC output, before any analog frontend
- `'rf_output'` — at the RF frontend output, before the cryostat
- `'detector'` — at the cryogenic detector (default, full TX chain)

This requires calibration values in the config file. For mixerless modules,
put measured amp-enabled / bypassed S21 values under
`rf_frontend.mixerless_module`; these can be scalars, frequency tables, or
calibration filenames. See the [calibration guide](calibration.md) for details.

### Dynamic range optimisation

For best SNR, use the dynamic range optimisation mode. This maximises DAC bit utilisation and adjusts the analog chain to hit the target power:

```python
# Optimised mode — maximises DAC dynamic range, adjusts attenuator/amp
result = client.set_tone_powers(
    [-90, -95],
    reference_plane='detector',
    optimise_dynamic_range=True
)

print(result['warnings'])       # any limitations encountered
print(result['power_error_db']) # per-tone error vs target
```

The optimisation proceeds in steps:
1. **Amplitude ratios** are computed from the target powers so per-tone variation is preserved.
2. **PSB FFT shift** is swept from most attenuated (safe) to least, stopping at the first overflow. Per-bin scaling is applied to account for coherent addition in shared bins.
3. **PSB scale** is ramped-up from a safe starting value, stopping just before overflow or DAC saturation.
4. **TX level adjustment**: the required level change is **computed** from the
   calibration chain gains (DAC output power + known analog gains/losses). If
   RF peripherals are available, the optimiser uses the TX programmable
   attenuator and, where supported, the TX amplifier bypass. If those controls
   cannot absorb enough excess power, the remaining reduction is applied by
   lowering `psb_scale`. RFDC DSA handling in the software is for the ADC/RX
   path, not TX optimisation.
5. **Compression check**: use `maximise_tx_power(compression_headroom_db=10.0)` when you want the total power into the RF frontend kept 10 dB below the modelled 1 dB compression point.
6. **Final amplitudes** are calculated with the now-fixed analog settings.
7. **Verification** confirms achieved powers against targets using `get_tone_powers(reference_plane=...)`.

The result dict returned by `set_tone_powers` includes `achieved_powers_dbm`, `power_error_db`, and `warnings` — these are surfaced to the client automatically.

The function also checks RFDC RTS (Real-Time Status) sticky overvoltage flags during overflow detection, if available in the installed `souk_mkid_readout` version.

### Power breakdown

To see where power is gained or lost through the TX chain:

```python
powers, details = client.get_tone_powers(detailed_output=True)
print(details)  # per-stage power contributions
```

### RX tone powers

RX reference planes in `get_tone_powers()` are modelled from the current TX
tone settings and calibration. The function follows the configured TX chain to
the end of the enabled path (the API calls this the `detector` plane), then
runs a forward RX-chain model through the cryostat, RF frontend, ADC, PFB, and
accumulator.

This forward estimate is useful when the RX input can be inferred from the
preceding stages, for example with FPGA internal loopback or an external
through/loopback path represented by the config S21 terms. Configure unused
sections as disconnected or set their S21 terms to 0, and put any measured
loopback loss in the appropriate stage calibration. If a detector or resonator
is present, the returned RX powers also need the device transmission at the
tone frequency. In dB terms, apply the per-tone `S21(f_tone)` magnitude before
continuing down the RX chain.

`get_tone_powers()` does not acquire or invert measured accumulated IQ samples.
For measured data, read accumulator snapshots or stream frames and use
`calibration.calc_adc_input_power()` to walk the RX chain backwards. With only
digital/ADC parameters it returns ADC-input power; with RX frontend and
cryostat S21 terms it can refer the measured level back toward the cryostat
output. The `details` output is useful when you want to stop at an intermediate
stage such as `adc_dbm`, `rx_rf_dbm`, or `cryostat_output_dbm`.

```python
# Modelled power at ADC input
rx_powers = client.get_tone_powers(reference_plane='adc_input')

# Modelled power at cryostat output (before RX frontend)
cryo_powers = client.get_tone_powers(reference_plane='cryostat_output')

# Modelled accumulated IQ magnitude (no calibration)
accumulator_levels = client.get_tone_powers(reference_plane='accumulator')
```

See the [Calibration Guide - Reference Planes](calibration.md#reference-planes) for the full list of TX and RX reference planes.

---

## Resonator Drive Power

### Why it matters

MKID resonators are nonlinear devices. The resonance frequency, quality factor, and dip depth all depend on the readout drive power:

- **Too low**: Poor signal-to-noise ratio, amplifier or readout noise may dominate.
- **Too high**: The resonance becomes nonlinear - the dip shape distorts, the resonance frequency shifts, and the detector response becomes bifurcates.
- **Optimal**: The highest power that does not cause significant nonlinearity. This maximises SNR while keeping the detector response linear.

### Indicators of nonlinearity

- **Asymmetric dip shape**: The resonance dip becomes asymmetric, typically steeper on one side.
- **Frequency shift with power**: The resonance frequency moves as drive power changes.
- **Dip depth saturation**: Increasing drive power no longer deepens the dip, or the dip becomes shallower.
- **Bifurcation**: At very high power, the forward and reverse sweep traces no longer overlap (hysteresis). The resonance appears to "jump" at a critical frequency.

---

## Drive Power Tuning

### Manual approach

1. Perform a wideband sweep to locate resonances.
2. Set readout tones at the resonance frequencies.
3. Perform targeted sweeps at several drive power levels and compare the resonance shape.
4. Choose the highest power where the dip shape remains symmetric and the resonance parameters (frequency, Q, depth) are stable.

```python
import numpy as np

freqs = client.find_resonance_frequencies()
client.set_tone_frequencies(freqs)

# Sweep at different power levels
for power_dbm in [-30, -25, -20, -15, -10]:
    client.set_tone_powers([power_dbm] * len(freqs))
    spans = [0.5e6] * len(freqs)
    client.perform_sweep(freqs, spans, 201, 10)
    # ... wait and collect data ...
```

### Fitting to dip depth

*Not yet implemented in the codebase.* A planned approach is to fit the resonance dip depth or the nonlinearity parameter as a function of drive power, and automatically select the optimal drive level.

The general procedure would be:

1. Sweep each resonance at a range of drive powers.
2. Fit the S21 dip at each power level to extract Q_i, Q_c, and the resonance frequency.
3. Plot Q_i and resonance frequency vs drive power.
4. Identify the onset of nonlinearity as the point where Q_i or frequency begins to change rapidly with power.
5. Set the drive power just below this onset.


### Nonlinearity parameter

*Coming soon.* A quantitative nonlinearity metric based on the Duffing model could be used to automate drive power selection.

---

## Related

- [Calibration Guide](calibration.md) for measurement procedures and RF peripheral details
- [Getting Started - Power Calibration & Optimisation](getting_started.md#power-calibration--optimisation)
- [Getting Started - Setting Readout Tones](getting_started.md#setting-readout-tones)
- [Installation - Calibration Files](installation.md#calibration-files)
