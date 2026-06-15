# Tone Power and Dynamic Range Notes

This document is a reference for tone-power controls and dynamic-range
constraints. For the practical resonator drive-tuning, retuning, and
on/off-resonance noise-recording procedure, use [Resonator Drive Tuning and
Noise Measurements](resonator_noise_workflow.md).

## Contents

- [Signal Chain](#signal-chain)
- [Digital Dynamic Range](#digital-dynamic-range)
- [DAC and RF Guardrails](#dac-and-rf-guardrails)
- [Setting Tone Powers](#setting-tone-powers)
- [RX Policies](#rx-policies)
- [Power Utility Summary](#power-utility-summary)
- [Blind Tones](#blind-tones)
- [Resonator Drive Power](#resonator-drive-power)
- [Power-Sweep Analysis](#power-sweep-analysis)

## Signal Chain

The transmit chain is:

```text
Tone amplitudes
  -> PSB FFT shift schedule
  -> PSB scale
  -> DAC and DUC mixer
  -> VOP current setting
  -> RF frontend
  -> cryostat
  -> detector
```

The receive chain then returns through the cryostat, RF frontend, ADC, PFB,
mixer, and accumulator. Calibration terms in `calibration.py` convert between
digital settings and physical power at several reference planes. See the
[Calibration Guide](calibration.md) for the signal-chain model and calibration
procedure.

## Digital Dynamic Range

Three transmit-side firmware controls set the digital level reaching the DAC:

- **Tone amplitude**: per-tone linear scaling from `0` to `1`. The relative
  amplitudes set the relative tone powers.
- **PSB FFT shift**: controls bit growth through the polyphase synthesis
  filterbank. Fewer shifts increase output level but can cause integer
  overflow.
- **PSB scale**: global scaling after the synthesis filterbank and before the
  DAC.

DSP overflow and DAC saturation are distinct conditions:

```python
client.check_dsp_overflow()
client.check_output_saturation()
client.check_input_saturation()
```

The power helpers account for tones that share one FFT bin. Their coherent
addition can overflow the vector accumulator (VACC), so the helpers scale the
tone amplitudes by the worst shared-bin overlap while preserving relative
powers.

For a multitone comb, phase offsets also matter. Newman phases reduce the crest
factor and allow better use of DAC range:

```python
freqs = client.get_tone_frequencies()
phases = client.generate_newman_phases(freqs)
client.set_tone_phases(phases)
```

For an offline estimate of peak-to-average power ratio (PAPR):

```python
from souk_readout_tools.firmware_lib import estimate_papr_db

papr_db = estimate_papr_db(freqs, amps, phases, sample_rate)
```

## DAC and RF Guardrails

The DAC full-scale output depends on firmware scaling, the DUC fine-mixer
scale, VOP current and inverse-sinc filter state. The recommended calibration conditions are:

- Use the `0P7` DUC fine-mixer scale rather than `1P0`.
- Use `20000 uA` VOP current for gen3 devices with `2.5 V` DAC VTT.
- Use the same DAC inverse-sinc filter state and Nyquist zone as the
  frequency-dependent DAC calibration.
- Treat DAC snapshot saturation separately from DSP overflow flags.

The config template records the hardware-specific warnings for values outside
those conditions. See [Calibration Guide - Measurement
Procedure](calibration.md#measurement-procedure) for the measured reference
conditions.

When a calibrated TX frontend model includes its input-referred 1 dB
compression point, `maximise_tx_power()` can reserve an additional margin:

```python
client.maximise_tx_power(
    reference_plane="detector",
    compression_headroom_db=10.0,
)
```

## Setting Tone Powers

### Relative Amplitudes

Use amplitudes when only relative digital levels matter:

```python
client.set_tone_amplitudes([0.5, 0.3])
```

### Calibrated Powers

Use `set_tone_powers()` for absolute powers in dBm:

```python
result = client.set_tone_powers(
    [-90, -95],
    reference_plane="detector",
)
```

The TX reference planes are:

- `'dac'`: DAC output, after VOP and before the analog frontend.
- `'rf_output'`: RF frontend output, before the cryostat.
- `'detector'`: cryogenic focal plane.

`set_tone_powers()` defaults to `optimise_dynamic_range=True`. It preserves
the requested relative tone powers while maximising useful DAC range,
selecting available TX RF controls, and reducing `psb_scale` if the analog
controls cannot absorb enough excess power. The DAC VOP setting is not changed.

Use the fast path only when the chain has already been configured and changing
amplitudes alone is intentional:

```python
result = client.set_tone_powers(
    [-90, -95],
    reference_plane="detector",
    optimise_dynamic_range=False,
)
```

The returned result includes achieved powers, power errors, warnings, and
dynamic-range diagnostics:

```python
print(result["result"]["achieved_powers_dbm"])
print(result["result"]["power_error_db"])
print(result["result"]["warnings"])
```

To inspect the calibrated model at each stage:

```python
powers, details = client.get_tone_powers(detailed_output=True)
```

`get_tone_powers()` also exposes modelled RX planes such as `'adc_input'`,
`'cryostat_output'`, and `'accumulator'`. These are forward predictions from
the configured path; they do not acquire or invert measured I/Q data. For
measured data, use `calibration.calc_adc_input_power()`.

## RX Policies

Changing TX settings can change the RX level. `set_tone_powers()` and
`maximise_tx_power()` accept an `rx_policy`:

- `'protect'`: default. Add RX attenuation or DSA only when needed to clear
  ADC saturation.
- `'compensate'`: mirror TX level changes onto the RX path to keep round-trip
  power approximately constant.
- `'maximise'`: run `maximise_rx_power()` after the TX change, optimising RX
  attenuation, ADC DSA, RX amp bypass, and PFB FFT shift.
- `'raise'`: raise an error on ADC saturation.
- `'none'`: leave the RX path untouched.

When the resonator transmission changes during a sweep, optimise with tones
parked off resonance near the highest-transmission point. Optimising RX gain
while tones sit on attenuating resonator dips can leave too little margin when
the tones move off dip, causing ADC saturation or downstream DSP overflow. A
known dip depth could instead be reserved as RX headroom, but off-resonance
parking is the simple conservative choice. The practical workflow applies this
rule consistently.

## Power Utility Summary

Use the highest-level helper that matches the operation:

| Helper | Purpose | Important defaults |
|---|---|---|
| `set_tone_powers()` | Apply calibrated per-tone target powers while preserving their relative differences | `optimise_dynamic_range=True`, `rx_policy='protect'` |
| `maximise_tx_power()` | Raise TX output until the selected limit is reached | `headroom_db=2.0`, `reference_plane='dac'`, `rx_policy='protect'` |
| `maximise_rx_power()` | Move RX level toward ADC full scale while retaining margin | `headroom_db=1.0` |
| `optimise_tx_snr()` | Improve TX SNR without intentionally changing the requested output powers | `headroom_db=2.0`, `reference_plane='detector'` |
| `optimise_rx_snr()` | Improve RX SNR using RX controls while avoiding saturation | Uses the configured RX controls |
| `fix_dac_saturation()` | Reduce `psb_scale` until DAC saturation clears | Recovery helper |
| `fix_adc_saturation()` | Adjust RX settings until ADC saturation clears | Recovery helper |

The helpers also accept advanced `force_*` controls to pin individual RF or
firmware settings during an optimisation. See the client method docstrings for
the complete list.

## Blind Tones

Blind tones are ordinary active firmware tones used for off-resonance
gain/phase monitoring. They contribute to every shared constraint:

- VACC shared-bin limits.
- PAPR and DAC saturation.
- Total TX frontend power and compression checks.
- RX level and ADC saturation.

Generate phases over the complete active tone list after adding blind tones.
If calling `set_tone_powers()` directly, pass one requested power for every
active tone or split arrays using `get_tone_metadata()` and
`get_blind_tone_indices()`.

Avoid placing blind tones on an exactly regular grid. Regular grids can make
intermodulation products add coherently. `client.suggest_blind_frequencies()`
starts from approximately even band coverage and jitters positions by default
while enforcing spacing constraints.

## Resonator Drive Power

MKID resonators are nonlinear devices. Readout drive power affects resonance
frequency, quality factor, dip depth, and ultimately detector SNR:

- Too little power gives poor readout SNR.
- Too much power distorts the resonance and can lead to bifurcation.
- A useful operating point is below the chosen nonlinearity threshold while
  retaining adequate readout SNR.

Useful signs of increasing nonlinearity include:

- Asymmetric dip shape.
- Resonance-frequency shift with power.
- Changing or saturating dip depth.
- Hysteretic jumps between upward and downward sweeps.

The operating threshold is experiment-specific. The end-to-end selection
procedure, including the `anl` fit and blackbody-load directory layout, lives
in [Resonator Drive Tuning and Noise
Measurements](resonator_noise_workflow.md).

## Power-Sweep Analysis

`souk_readout_tools.power_sweep` stores a tone-power sweep as a measurement run
with one targeted sweep artifact per power step:

```python
from souk_readout_tools import power_sweep as ps

run = ps.run_power_sweep(
    client,
    centers=freqs,
    spans=spans,
    powers_dbm=power_steps_dbm,
    output_dir="kid_power_sweep",
    follow_dips=True,
    optimise_dynamic_range=True,
    rx_policy="maximise",
)

analysis = ps.analyse_power_sweep(
    run,
    nonlinear=True,
    n_jobs=-1,
    target_anl=0.1,
)

best_arrays = ps.best_power_arrays(analysis["best_power"])
```

`run_power_sweep()` defaults to an unsaved initial center search followed by
conservative dip following between power steps. Before applying each power it
parks tones at the sweep low edge so RX optimisation is performed away from
the attenuating dips.

`analyse_power_sweep()` writes reusable outputs beneath `analysis/`, including
`fit_results.pkl`, `fit_summary.csv`, and `best_power.json`. It also writes
per-tone fit and selected-power plots beneath `plots/`.

For the full acquisition sequence and the recommended data layout, continue
with [Resonator Drive Tuning and Noise
Measurements](resonator_noise_workflow.md).

## Related

- [Resonator Drive Tuning and Noise Measurements](resonator_noise_workflow.md)
- [Calibration Guide](calibration.md)
- [Getting Started - Setting Readout Tones](getting_started.md#setting-readout-tones)
- [Getting Started - Power Calibration and Optimisation](getting_started.md#power-calibration--optimisation)
