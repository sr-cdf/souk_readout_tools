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

### DAC output

The DAC has 14 physical bits but is addressed with a 16-bit word. The full-scale output power depends on the VOP current setting and the DUC mixer scaling mode.

Key constraints:
- **VOP current**: Default and maximum rated value is 20000 uA for gen3 devices with 2.5V DAC VTT. Values above 20000 uA (up to the hardware max of 40500 uA) add nonlinearity and are not recommended.
- **DUC mixer scale**: Use `0P7` (not `1P0`) to avoid overflow at the mixer output. The `1P0` mode provides ~3 dB more power but risks clipping.
- **PSB FFT shift**: Controls bit growth through the polyphase synthesis filterbank. More shifts preserve dynamic range but reduce output level. Fewer shifts increase output level but risk integer overflow.

### Multitone power sharing

When driving N tones simultaneously, each tone's contribution to the total waveform amplitude scales as ~1/sqrt(N) on average (for random phases). The peak-to-average power ratio (PAPR, or crest factor) depends on the phase relationship between tones.

Use Newman phases (`client.generate_newman_phases(freqs)`) to minimise crest factor. This allows higher per-tone power before the composite waveform clips the DAC.

### PSB scale

The `psb_scale` parameter provides a global scaling factor on the output waveform going into the DAC. Use this to:
- Maximise the waveform amplitude relative to the DAC's full-scale range
- Avoid DAC clipping when the total power of all tones is high
- Fine-tune the operating point between dynamic range and headroom

The auto-optimisation methods (`client.maximise_tx_power()`, `client.optimise_tx_snr()`) adjust this parameter automatically.

---

## Setting Tone Powers

### By amplitude (relative)

Tone amplitudes are linear scaling factors from 0 to 1.0, relative to the LO full-scale:

```python
client.set_tone_amplitudes([0.5, 0.3])
```

This sets relative power levels between tones.

### By power in dBm (absolute)

The calibrated power interface accounts for the full signal chain:

```python
client.set_tone_powers([-20, -25])  # dBm at the output reference plane
```

This requires calibration values in the config file (`dac0_dbfs_to_dbm`, RF frontend S21 parameters, cryostat S21, etc.). See the [calibration section](installation.md#calibration-files) for how to specify these.

### Power breakdown

To see where power is gained or lost through the chain:

```python
client.get_tone_powers(detailed_output=True)
```

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

- [Getting Started - Power Calibration & Optimisation](getting_started.md#power-calibration--optimisation)
- [Getting Started - Setting Readout Tones](getting_started.md#setting-readout-tones)
- [Installation - Calibration Files](installation.md#calibration-files)
- Calibration measurement conditions: `~/.souk_readout_tools/pipeline_0/calibrations/readme`
