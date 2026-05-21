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

RX handling during TX power changes is controlled with `rx_policy`:

- `protect` keeps the ADC from saturating, adding RX attenuation or DSA only
  when needed.
- `compensate` mirrors TX power changes onto the RX path to keep ADC power
  approximately constant.
- `maximise` runs `maximise_rx_power()` after the TX change, optimising RX
  attenuation, ADC DSA, RX amp bypass state, and PFB FFT shift.
- `raise` errors on ADC saturation, and `none` leaves the RX path untouched.

The same policy string can be passed through `run_power_sweep(...,
rx_policy="maximise")`.

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
    client.perform_sweep(freqs, spans, 201, 10, wait=True)
    # ... collect data ...
```

### Fitting power sweeps

The `souk_readout_tools.power_sweep` helper keeps acquisition, fitting, and
plotting as explicit steps for this workflow:

```python
from souk_readout_tools import power_sweep as ps

run = ps.run_power_sweep(
    client,
    centers=freqs,
    spans=0.5e6,
    powers_dbm=[-95, -90, -85, -80],
    output_dir="kid_power_sweep",
    follow_dips=True,
)

data = ps.load_power_sweep("kid_power_sweep")
fits = ps.fit_power_sweep(data, nonlinear=True, n_jobs=-1)
ps.write_fit_summary(fits, "kid_power_sweep/fit_summary.csv")
ps.write_fit_results(fits, "kid_power_sweep")
ps.plot_power_sweep(data, fits, deembed=True)
```

On later notebook sessions you can skip the fit step and reload the stored
`FitResult` objects directly:

```python
data = ps.load_power_sweep("kid_power_sweep")
fits = ps.load_fit_results("kid_power_sweep", run=data)
ps.plot_power_sweep(
    data,
    fits,
    deembed=True,
    tone_indices=range(0, data["tone_count"], 10),  # quick-look subset
    save_overlay=False,  # skip the crowded combined overlay for large arrays
    dpi=90,
    fit_figsize=(7.0, 4.2),
    parameter_figsize=(5.8, 6.9),
    parameters=("fr", "empirical_dip_depth_db", "Qi", "Qc", "phi", "anl"),
)
```

Use `tone_indices` to control which per-tone PNGs are written. It accepts a
single tone index, an explicit list such as `[0, 12, 37]`, a `range(...)`, or a
NumPy index array. Leave it as `None` only when you really want diagnostics for
every resonator in the sweep. Both `plot_power_sweep()` and
`plot_best_power()` print compact progress by default while writing plots; pass
`verbose=False` to silence that output.

The parameter plots keep 1-sigma error bars by default, but use a compact
layout and one vectorised error-bar artist per parameter subplot. Reduce
`dpi`/`parameter_figsize` further for quick-look production, or set
`parameter_show_errors=False` only for very rough browse plots.

`fit_power_sweep()` has two fitting shapes. With `tone_index=None`, it calls
`batch_fit()` once per saved server sweep and returns `fits_by_power`, so the
output is organised as power step -> tone. This path preserves server tone
indices and skips blind tones by default. With `tone_index=<i>`, it extracts
that one tone from every power step and returns a single `fits` list ordered by
power. That single-tone path uses `fit_sweep_stack()` as an array adapter; rows
are currently fit independently, so the previous power's result is not used as
the next initial guess.

For quick notebook inspection, use `fit_parameter_series()` to pull one tone's
parameters onto the power axis without walking the nested fit lists:

```python
series = ps.fit_parameter_series(fits, tone_index=0)
plt.plot(series["power_dbm"], series["Qc"] / series["Qi"], marker="o")
```

The general procedure is:

1. Sweep each resonance at a range of drive powers.
2. Fit the S21 dip at each power level to extract Q_i, Q_c, and the resonance frequency.
3. Plot Q_i and resonance frequency vs drive power.
4. Identify the onset of nonlinearity as the point where Q_i or frequency begins to change rapidly with power.
5. Set the drive power just below this onset.

Fit summaries also include `empirical_*` columns (`empirical_linewidth_hz`,
`empirical_dip_depth_db`, empirical Q estimates, and `empirical_skew`). These
are measured before optimisation, so they are useful when a least-squares fit
fails or lands in an unrealistic basin. If `empirical_dip_depth_db` is below
`min_dip_depth_db`, the fitter marks that row as `success=False` and
`noise_only=True`; power selection ignores those rows by default.
Tones marked as blind monitors are saved for raw inspection, but are skipped by
the batch fitting, fit-summary, and fitted power-sweep plotting helpers.

With `follow_dips=True`, each targeted sweep recentres the next power step on
the empirical local dip near the current tone. This is useful when resonances
move with drive power. The update is conservative: blind tones are left fixed,
the search is limited to the tone's current span, candidate dips shallower than
`follow_min_depth_db` (default `0.5` dB, matching `fit_power_sweep()`'s
`min_dip_depth_db` default) keep their previous centre, and proposed moves that
would make neighbouring tone centres cross or bunch together are rejected. Set
`follow_min_depth_db=None` to accept any depth.


### Nonlinearity parameter

For nonlinear fits, the Duffing `anl` parameter can be used to automate the
readout-power choice. Work with the in-memory fit result first:

```python
fits = ps.fit_power_sweep(data, nonlinear=True, n_jobs=-1)

best = ps.find_best_power(fits, target_anl=0.01)
plots = ps.plot_best_power(fits, best, dpi=90, figsize=(6.2, 3.8))
```

`find_best_power()` fits an unweighted straight line to `log(anl)` vs tone
power for each resonator, applies one pass of MAD outlier rejection, and
returns the precise power that reaches the target ANL. Rows are excluded
before the fit when the solver reports failure, when any fitted parameter
falls at or beyond an entry in `param_valid_ranges` (defaults mirror the
fitter's physical bounds, so rows pinned at a clamp are dropped), or when
`Qi` / `Qc` / `phi` deviates from the per-tone median by more than
`param_outliers_mad_clip` MAD-equivalent sigmas (default `5.0`).
Low-clamped `anl` rows are excluded from the ANL-vs-power fit, but still
count for the measured fallback that picks the highest power with
`anl < target_anl`.

If the target lies outside the measured range, `find_best_power()` simply
returns the extrapolated value from the log-linear fit. The `range_position`
field on the pick (`within_measured_range` / `above_measured_range` /
`below_measured_range`) flags this for diagnostics. If the ANL fit is
unreliable (too few retained points, or slope below `anl_fit_min_slope`), the
selection falls back to the highest-power row whose measured `anl` is below
the target, and finally to the lowest measured power.

`plot_best_power()` writes per-tone diagnostic plots showing the measured ANL
values, rejected rows, the log-linear ANL fit, the target ANL, and the selected
power. The same helpers also accept rows from `fit_summary_rows()` or a
`fit_summary.csv` path for offline checks. For large arrays, pass
`tone_indices=...` to either plotting helper when you only need a subset of
diagnostics.

### End-to-end best-power workflow

Putting the pieces together, a typical iteration looks like this. The first
sweep brackets each tone with a wide power schedule, the chosen powers are
saved to disk, and the next sweep narrows around them:

```python
import numpy as np
from souk_readout_tools import power_sweep as ps

OUT = "kid_power_sweep_round1"
p = np.arange(-100, -70, 3)[:, np.newaxis]  # (n_steps, 1), broadcasts to tones

run = ps.run_power_sweep(
    client,
    centers=centers,
    spans=spans,
    points=301,
    samples_per_point=3,
    powers_dbm=p,
    output_dir=OUT,
    follow_dips=True,            # uses follow_min_depth_db=0.5 by default,
                                 # so shallow noise dips are not chased
    optimise_dynamic_range=True,
    rx_policy="maximise",
)

data = ps.load_power_sweep(OUT)
fits = ps.fit_power_sweep(data, nonlinear=True, n_jobs=-1,
                          try_harder=True, tol=1e-8, min_dip_depth_db=1)
ps.write_fit_results(fits, OUT)
ps.write_fit_summary(fits, f"{OUT}/fit_summary.csv")

best = ps.find_best_power(fits, target_anl=0.05)
ps.write_best_power(best, OUT)            # writes OUT/best_power.json
ps.plot_best_power(fits, best)
ps.plot_power_sweep(
    data, fits,
    parameters=("empirical_fr", "empirical_dip_depth_db", "fr",
                "Qi", "Qc", "phi", "anl"),
    show_overlay=True,
)
```

For the next iteration, reload the previous picks and centre a denser
schedule on them:

```python
arrays = ps.best_power_arrays(OUT)        # accepts a dir, file, or list
b0 = arrays["chosen_power_dbm"]
p_bif = arrays["p_bif"]                   # 1-D arrays, length == n_tones

p = b0 + np.arange(-9, 10, 3)[:, np.newaxis]
run = ps.run_power_sweep(
    client,
    centers=centers,
    spans=spans,
    powers_dbm=p,
    output_dir="kid_power_sweep_round2",
    follow_dips=True,
    optimise_dynamic_range=True,
    rx_policy="maximise",
)
```

`best_power_arrays()` returns
`{"tone_index", "chosen_power_dbm", "p_bif", "p_bif_sub_3db"}` as 1-D arrays
of length `n_tones`, with `NaN` wherever a tone was missing from
`find_best_power()`'s result. `write_best_power()` / `load_best_power()`
round-trip the full per-tone dicts (including criteria, diagnostics, and
exclusion reasons) via `OUT/best_power.json`, so any later session can
recover the chosen powers and the bifurcation estimates without rerunning
`find_best_power()`.

---

## Related

- [Calibration Guide](calibration.md) for measurement procedures and RF peripheral details
- [Getting Started - Power Calibration & Optimisation](getting_started.md#power-calibration--optimisation)
- [Getting Started - Setting Readout Tones](getting_started.md#setting-readout-tones)
- [Installation - Calibration Files](installation.md#calibration-files)
