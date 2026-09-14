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
  DAC. UFix16_8: resolution 1/256, maximum ~255.996 — verified end to end on a
  v7.11 board.

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

### Where the Digital Gain Is Taken

The three controls are not interchangeable, because they sit at different
points in the chain. Tone amplitude is the mixer/LO scale word **before** the
synthesis filterbank; `psb_scale` is **after** it. Halving the amplitude and
doubling `psb_scale` leaves the DAC level unchanged but runs the filterbank
6 dB closer to its own rounding floor, then amplifies that floor back up along
with the signal. One fewer FFT shift stage is the same 6 dB in the opposite
direction, taken inside the filterbank.

Whether that costs SNR depends on which quantisation floor dominates at the
accumulator, which is a property of the firmware build and the operating
point, not something to assume. Measure it:

```bash
python scripts/quantisation/quantisation_noise_test.py -C my_config.yaml
```

The script holds the DAC drive constant and walks both trades as a ladder
(amplitude 1, 1/2, 1/4, ... against `psb_scale` x1, x2, x4, ...; and the shift
schedule one stage hotter then progressively colder), measuring the
noise-to-carrier spectral density of a single loopback tone at the
accumulator, as the IEEE single-sideband `L(f) = S(f) / 2` in dBc/Hz -- the
same convention as `plot_timestream_psd(psd_units='dbc')`. It reports the
slope in dB of noise per dB of gain moved
downstream: `0` means the trade is free and the floor is set downstream of the
PSB, `1` means TX digital quantisation dominates and the amplitude should be
kept as high as the VACC and PAPR constraints allow. The reference point is
measured first and last so the repeatability floor is visible before any
difference is believed.

Run it in the default digital loopback first (no analog noise at all, so it is
the most sensitive test of the digital chain), then with `--loopback rf` to
see whether the effect survives the real signal path.

`scripts/quantisation/quantisation_noise_test_plot.py` plots the resulting CSV.

## DAC and RF Guardrails

The DAC full-scale output depends on firmware scaling, the DUC fine-mixer
scale, VOP current and inverse-sinc filter state. The recommended calibration conditions are:

- Use the `0P7` DUC fine-mixer scale rather than `1P0`. The DAC DUC is
  complex-to-real, where `1P0` risks overflow.
- Use `20000 uA` VOP current for gen3 devices with `2.5 V` DAC VTT.
- Use the same DAC inverse-sinc filter state and Nyquist zone as the
  frequency-dependent DAC calibration.
- Treat DAC snapshot saturation separately from DSP overflow flags.

The ADC DDC is the other way round: it is real-to-I/Q, so `1P0` is the natural
scale and is the bundled default (`adc_mixer_scale_1p0: true`). It gains 3 dB of
digital signal over `0P7` with no analog change, and it must match whatever
`adc_dbm_to_dbfs` was measured at.

The config template records the hardware-specific warnings for values outside
those conditions. See [Calibration Guide - Measurement procedure
(DAC)](calibration.md#measurement-procedure-dac) for the measured reference
conditions, and [Measurement procedure
(ADC)](calibration.md#measurement-procedure-adc) for the loopback ADC
calibration.

When a calibrated TX frontend model includes its input-referred 1 dB
compression point, `maximise_tx_power()` can reserve an additional margin:

```python
client.maximise_tx_power(
    reference_plane="detector",
    compression_headroom_db=10.0,
)
```

### Compression on the RX side

ADC levels say nothing about compression: a compressed amplifier feeding a
clean-looking -1 dBFS reads as a perfect result. Whenever the frontend model
provides an RX input P1dB, `maximise_rx_power()` therefore reports the margin to
it at each stage and refuses settings that eat into
`compression_headroom_db` (default 3 dB) — the RX amp stays bypassed, or the
attenuator stops coming down:

```text
  compression at init: RX input -40.0 dBm total, P1dB 9.9 dBm, margin 49.9 dB (target 3.0 dB)
  step 3: try enabling RX amp
    RX amp leaves only 1.9 dB to the input P1dB (target 3.0 dB) — keeping bypassed
```

This matters most for the RX amp. The module's variable attenuator sits *after*
the preamp and before the switchable amp, so enabling the amp drops the
input-referred P1dB by roughly its own gain, and every dB taken out of the
attenuator afterwards takes another dB with it — while the power arriving at
the module never changes. When the binding stage is the preamp instead, no RX
control helps and the log says so; the fix is TX power or cryostat output
attenuation.

The margin is reported as `result['rx_compression']` (and `['tx_compression']`
from `maximise_tx_power()`). Pass `compression_check=False` to optimise purely
on ADC levels. The RX input power is modelled forward from the TX settings, so
it is only as good as the configured cryostat/loopback S21.

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
| `set_tone_powers()` | Apply calibrated per-tone target powers while preserving their relative differences | `optimise_dynamic_range=True`, `rx_policy='protect'`, `headroom_db=1.0` |
| `maximise_tx_power()` | Raise TX output until the selected limit is reached | `headroom_db=1.0`, `reference_plane='detector'`, `rx_policy='protect'` |
| `maximise_rx_power()` | Move RX level toward ADC full scale while retaining margin | `headroom_db=1.0` |
| `optimise_tx_snr()` | Improve TX SNR without intentionally changing the requested output powers | `headroom_db=1.0`, `reference_plane='detector'` |
| `optimise_rx_snr()` | Improve RX SNR using RX controls while avoiding saturation | `headroom_db=1.0` |
| `fix_dac_saturation()` | Reduce `psb_scale` until DAC saturation clears | `headroom_db=1.0`, `intermittent_backoff_db=3.0`, `backoff_step_db=0.5` |
| `fix_adc_saturation()` | Adjust RX settings until ADC saturation clears | `atten_step_db=3.0`, `dsa_step_db=2.0` |

The helpers also accept advanced `force_*` controls to pin individual RF or
firmware settings during an optimisation. See the client method docstrings for
the complete list.

### The Headroom Margins

Four separate margins are settable at every call point. They guard different
limits at different points in the chain and are not interchangeable:

| Parameter | Guards | Applied to |
|---|---|---|
| `dac_headroom_db` | DAC saturation | `psb_scale` |
| `adc_headroom_db` | ADC saturation | ADC DSA and RX attenuator |
| `filterbank_headroom_db` | Modulation TX drive-restore clipping | the amplitude words |
| `compression_headroom_db` | RF frontend P1dB compression | `psb_scale` on TX; the RX amp and attenuator on RX, against the modelled frontend input power |

On the single-converter routines the margin is just `headroom_db`, since the
function name already says which converter it means — `maximise_tx_power()` and
`set_tone_powers()` take `headroom_db` for the DAC, `maximise_rx_power()` and
`optimise_rx_snr()` for the ADC. The names above are what the entry points that
touch both converters use (`wideband_sweep()`, `run_power_sweep()`), plus
`adc_headroom_db` on the TX routines, where it sets the margin the `rx_policy`
works to. The same rule gives `wideband_sweep()` its
`tx_compression_headroom_db` / `rx_compression_headroom_db` split: one sweep
drives both chains, and the two are unrelated knobs — the TX one limits output
power, the RX one only decides how much RX gain is allowed.

### Every control the sweep touches is settable on the sweep

`wideband_sweep()` makes up to five calls that carry margins —
`maximise_tx_power()` or `set_tone_powers()`, `maximise_rx_power()`,
`fix_dac_saturation()`, `fix_adc_saturation()` — and forwards to each of them:

| Sweep argument | Reaches |
|---|---|
| `dac_headroom_db` | the tone-power call, and `fix_dac_saturation()` |
| `dac_saturation_threshold` | the tone-power call |
| `adc_headroom_db` | the tone-power call's `rx_policy`, and the RX-gain step |
| `filterbank_headroom_db` | the tone-power call |
| `atten_step_db` | the tone-power call, the RX-gain step, `fix_adc_saturation()` |
| `dsa_step_db` | `fix_adc_saturation()` |
| `rts_as_saturation` | the RX-gain step, `fix_adc_saturation()` |
| `compression_check` | the tone-power call and the RX-gain step |
| `tx_compression_headroom_db` | `maximise_tx_power()` |
| `rx_compression_headroom_db` | the RX-gain step |
| `intermittent_backoff_db`, `backoff_step_db` | `fix_dac_saturation()` |

Each is `None` by default, meaning "use the server's default", so the one copy
of every default stays in `firmware_lib`. The same names are available as
`--flags` on the `wideband_sweep` command-line script.

**Every headroom is measured from true full scale.** The saturation detectors
trip at 90% of full scale, which is itself 0.92 dB of margin, so only the
excess over that is applied on top — ask for 1 dB and the DAC lands 1 dB below
full scale, not 1.9 dB. Requests inside the 0.92 dB detector floor clamp to it
with a printed note. The step logs report requested and applied dB separately:

```text
psb_scale: 3.9570 (1.00 dB headroom below full scale = 0.92 dB detector + 0.08 dB applied)
```

On the RX side the margin is measured directly from the ADC snapshot rather
than inferred from the detector, so `headroom_db` is used as-is; the detector
still acts as a backstop in the step-back loops, which floors the achievable
margin at ~0.92 dB.

**`filterbank_headroom_db` reserves amplitude-word room** so the modulation TX
drive restore (base x 1/G) has somewhere to go when a probe moves out towards
half a bin separation. `None` (the default) follows the filterbank-compensation
config; an explicit value applies whether or not compensation is enabled, which
is how to reserve modulation room without turning compensation on:

```python
client.maximise_tx_power(headroom_db=1.0, filterbank_headroom_db=1.0)
```

Both margins are forwarded by the higher-level entry points, so a sweep does
not silently fall back to the library defaults:

```python
client.wideband_sweep(dac_headroom_db=2.0, adc_headroom_db=1.0,
                      filterbank_headroom_db=1.0)

run_power_sweep(client, centers, spans, powers, out,
                dac_headroom_db=2.0, adc_headroom_db=1.0)
```

Unset parameters are not sent at all, so the single copy of each default lives
in `firmware_lib` (`DEFAULT_DAC_HEADROOM_DB`, `DEFAULT_ADC_HEADROOM_DB`) and is
never duplicated downstream.

### Why the DSA Moved

`check_input_saturation()` reports two independent trip causes, and the RX
routines log which one fired:

- `level_saturated` — the ADC snapshot exceeded the threshold.
- `rts_saturated` — an RFDC RTS over-range/over-voltage flag latched.

RTS watches the full band continuously while the snapshot is a short capture of
a decimated view, so a transient or out-of-band overrange latches RTS with
clean-looking snapshot levels — one reason the DSA can gain a dB or two when
the reported levels look far from clipping. Pass
`rts_as_saturation=False` to drive purely off the measured levels.

The reported `*_fs` levels are a true fraction of full scale for both
converters. ADC snapshots only fill 15 of their 16 bits, so before this was
normalised an ADC at 88% of full scale read `0.44` and looked like it had
plenty of margin.

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

In a later session, reload all of that without recomputing anything:

```python
analysis = ps.load_analysis("kid_power_sweep")
```

To re-choose the powers from the stored fits — say with a different
`target_anl` — without re-rendering every per-tone fit plot, disable the
stages you don't need:

```python
analysis = ps.analyse_power_sweep(
    "kid_power_sweep",
    fit=False,        # reuse analysis/fit_results.pkl
    plot_fits=False,  # skip the slow per-tone fit plots
    target_anl=0.03,  # re-pick powers and refresh the ANL plots
)
```

To balance the comb afterwards, `balance_tone_powers()` re-allocates the
per-tone powers under the bifurcation-backed caps and optional TX/RX spread
constraints, with `write_balanced_power()` / `load_balanced_power()` for
persistence and `balanced_power_arrays()` for per-tone arrays. The module
docstring (`help(ps)`) lists every piece of the pipeline in order.

## Allocating on Measured Sensitivity

`balance_tone_powers()` works in tone-power space. What actually sets a tone's
readout-limited noise is its transducer gain `|dz/dx|` — carrier amplitude,
dip depth and Duffing steepening together — and that is measured, not modelled:

```python
gain   = ps.transducer_gain(run)     # |dz/dx|, counts per unit fractional freq
levels = ps.carrier_levels(run)      # ADC level, on and off resonance
```

Tone power is a poor proxy for either. On a 379-tone array the RX accumulator
level predicts the off-resonance `dx` noise floor at r = -0.13; `|dz/dx|`
predicts it at r = -0.91.

The tones share one RX attenuator, so raising the aggregate ADC level forces
that attenuator up and costs *every* tone `epsilon` dB, where

    epsilon = -d(referred noise, dB) / d(ADC fill, dB)

`epsilon = 1` is a noise floor fixed at the ADC; `epsilon = 0` is one entering
before the attenuator, which rides down with the carrier so the ADC constraint
costs nothing but clipping. It is a property of the readout chain, not the
array, so nothing here assumes it.

It is also a *local* slope rather than a constant. Writing the noise-to-carrier
as `N_after/C + N_before/C`, only the first term moves with the carrier, so at
high fill the before-the-attenuator noise dominates and `epsilon` tends to 0,
while at low fill the ADC term takes over and it tends to 1. It depends on comb
size too. Fit it over the fill you actually run at, with the comb you actually
use, and check `epsilon_high_fill` against `epsilon_low_fill` in the result.

Measure it once per system with
`scripts/adc_headroom/01_attenuation_ladder.py`, reduce it with
`02_analyse_ladder.py`, and pass the number in:

```python
alloc = ps.optimise_tone_powers(run, epsilon=0.6, max_loss_db=6.0)
ps.plot_allocation(run, alloc["allocated_power_dbm"], metrics=alloc)
```

`optimise_tone_powers()` gives every tone the same price for ADC power and
lets each maximise its own gain against it — a water-fill generalised to
arbitrary measured curves, so it needs no assumption that gain follows power.
`max_loss_db` is the "I will sacrifice some tones, but not switch them off"
knob; bifurcation caps override it and the tones that hit that are listed in
`loss_floor_overridden`.

To score an allocation without running it, `allocation_metrics()` returns
per-tone `delta_noise_db = -delta_gain_db + epsilon * delta_aggregate_db`.
Both terms come back separately, so a different `epsilon` can be applied
afterwards without recomputing anything.

Two things worth reading off the report before changing any powers:

- `aggregate_excess_db` — how much ADC budget the loud tones take beyond a flat
  comb of the same total, i.e. the dB every ordinary tone is paying for them.
- `unused_headroom_db` — the RX optimiser normally runs with the comb parked
  off-dip, so it sets the attenuator for a level the comb never reaches on
  resonance. Measured 4.7–6.5 dB on a SOUK BB ladder, and recovering it helps
  every tone at once rather than trading between them.

Where a tone is read matters as much as how hard it is driven. At the dip
centre the Duffing gain is exactly 1 at any drive, and measured `|dz/dx|`
saturates and turns over; at the steepest-`dphi/df` point it keeps climbing,
with `d(gain)/dP` rising from 0.98 to 1.90 dB/dB approaching bifurcation. Pass
`at='max'` (default) or `at='tone'` to match how the tones are actually parked.

For the full acquisition sequence and the recommended data layout, continue
with [Resonator Drive Tuning and Noise
Measurements](resonator_noise_workflow.md).

## Related

- [Resonator Drive Tuning and Noise Measurements](resonator_noise_workflow.md)
- [Calibration Guide](calibration.md)
- [Getting Started - Setting Readout Tones](getting_started.md#setting-readout-tones)
- [Getting Started - Power Calibration and Optimisation](getting_started.md#power-calibration--optimisation)
