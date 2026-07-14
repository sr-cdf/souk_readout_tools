# Resonator Drive Tuning and Noise Measurements

This guide records a practical workflow for choosing resonator drive powers and
then measuring on-resonance and off-resonance noise. It is intended for
blackbody-load campaigns where the same sequence is repeated at several load
points.

The sequence is:

1. Sweep each resonator over drive power.
2. Fit nonlinearity (`anl`) versus power and select a drive power per tone.
3. Park the tones off resonance and apply the selected powers with TX and RX
   dynamic-range optimisation enabled.
4. Retune onto resonance, saving the retune sweep as the calibration sweep.
5. Record on-resonance and off-resonance accumulated timestreams.
6. Record on-resonance and off-resonance pre-accumulator snapshots for the
   higher-frequency noise band.

The frequency sweep saved by the retune is important: it is the local
calibration used to convert I/Q noise into fractional frequency and
dissipation noise.

## Directory Layout

Keep each blackbody load point as a small container directory. Put the
tone-power sweep and the subsequent noise measurement in sibling directories:

```text
bb_campaign_2026-06-01/
  campaign.md
  loads/
    BB_050_K/
      load.json
      power_sweep/
        measurement.json
        data/
        analysis/
        plots/
        logs/
      noise/
        run_000/
          calibration_sweep.npy
          samples_on.npy
          samples_off.npy
          snapshots_on.npz
          snapshots_off.npz
```

Use the same shape for the remaining load points, for example `BB_075_K` and
`BB_100_K`.

This is preferable to using the power-sweep directory itself as the load-point
directory. `power_sweep/measurement.json` describes one specific
`tone_power_sweep` run. Keeping later data in a sibling `noise/` directory
makes that meaning clear while preserving a convenient per-load grouping for
cross-load analysis.

Record the requested and measured load values in `load.json`; the directory
name alone is not enough provenance for later analysis. A minimal file is:

```json
{
  "blackbody_setpoint_k": 50.0,
  "blackbody_temperature_k": 49.8,
  "notes": ""
}
```

Add another `noise/run_NNN/` directory when repeating a noise measurement at
the same load point. A more elaborate campaign manifest is not needed until
the acquisition is automated across load points.

## Fixed or Load-Dependent Powers

Decide whether resonator drive power is part of the controlled setup or part of
the quantity being optimised:

- For a controlled noise comparison across blackbody loads, choose powers once
  at a representative load and reuse them. Retune frequencies and save a new
  calibration sweep at every load point. This keeps drive power from becoming
  an extra changing variable.
- For a comparison of the best achievable operating point at each load, rerun
  the power sweep and select powers separately for each load. Keep the
  per-load `power_sweep/` sibling shown above so the selected powers remain
  attached to that load point.

Both are useful measurements, but they answer different questions. If powers
are fixed across the campaign, move the reference `power_sweep/` directory to a
campaign-level `tuning/` directory and omit the per-load copy.

## Imports and Paths

The snippets below are deliberately notebook-friendly but keep paths explicit:

```python
from pathlib import Path
import time

import numpy as np

from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools import power_sweep as ps
from souk_readout_tools.plotting import (
    plot_snapshots_psd,
    plot_sweep,
    plot_timestream_on_resonance,
    plot_timestream_psd,
)

client = ReadoutClient(config_file="my_config.yaml")
client.pull_config()

LOAD_DIR = Path("bb_campaign_2026-06-01") / "loads" / "BB_050_K"
POWER_SWEEP_DIR = LOAD_DIR / "power_sweep"
NOISE_DIR = LOAD_DIR / "noise" / "run_000"
NOISE_DIR.mkdir(parents=True, exist_ok=True)
```

## Tune Drive Powers

Start from a resonance file or another resonance-finding step. In this
example, each tone is swept from 15 dB below to 15 dB above an initial
`-90 dBm` estimate:

```python
resonances = client.read_resonances_file("./hp-weth-etch.resonances")
freqs = resonances["FrequencyHz"]
linewidths = resonances["LinewidthHz"]
spans = linewidths * 20

initial_powers_dbm = -90.0 * np.ones(len(freqs))
power_steps_dbm = initial_powers_dbm + np.arange(-15, 16, 3)[:, np.newaxis]

run = ps.run_power_sweep(
    client,
    centers=freqs,
    spans=spans,
    points=401,
    samples_per_point=3,
    powers_dbm=power_steps_dbm,
    output_dir=POWER_SWEEP_DIR,
    follow_dips=True,
    optimise_dynamic_range=True,
    rx_policy="maximise",
    follow_min_depth_db=0.5,
)
```

`run_power_sweep()` parks tones at the low edge of each sweep before applying
the requested power. This is deliberately conservative for the receive path.
On a resonator dip the transmitted signal is attenuated; optimising RX gain or
attenuation there can leave too little margin when the sweep moves away from
the dip. The higher off-resonance transmission can then cause ADC saturation
or downstream DSP overflow. If the dip depths are known, the optimisation
could instead reserve the corresponding receive-path headroom. For simplicity,
the workflow optimises near the highest-transmission point. TX DAC saturation
and TX DSP overflow are upstream of the resonators, so parking primarily
protects the RX-side optimisation performed by `rx_policy="maximise"`.

The sweep also pre-centres the first saved trace and follows resonator motion
between power steps by default.

Fit the saved sweeps and choose one power per resonator:

```python
analysis = ps.analyse_power_sweep(
    run,
    nonlinear=True,
    n_jobs=-1,
    target_anl=0.1,  # Experiment-specific; inspect the diagnostic plots.
    fit_kwargs={
        "try_harder": True,
        "tol": 1e-6,
        "subsample": False,
        "min_dip_depth_db": 0.5,
    },
    plot_kwargs={
        "deembed": True,
        "phase_center": True,
        "parameters": (
            "empirical_fr",
            "empirical_dip_depth_db",
            "empirical_skew",
            "fr",
            "Qi",
            "Qc",
            "phi",
            "anl",
        ),
    },
    best_power_kwargs={
        "param_uncertainty_outlier_mad_clip": 5,
        "param_outliers_mad_clip": None,
        "anl_fit_weight_by_uncertainty": False,
        "anl_fit_min_slope": 0.15,
    },
)

best = analysis["best_power"]
best_arrays = ps.best_power_arrays(best, tone_count=len(freqs))
freqs = best_arrays["fr"]
linewidths = best_arrays["empirical_linewidth_hz"]
powers_dbm = best_arrays["chosen_power_dbm"]
spans = linewidths * 20
```

`analyse_power_sweep()` writes reusable outputs beneath
`power_sweep/analysis/`, including `fit_results.pkl`, `fit_summary.csv`, and
`best_power.json`. Inspect the best-power plots and check for `NaN` values in
the arrays before programming hardware. The `target_anl` value is a detector
operating choice, not a universal constant.

`ps.load_analysis(POWER_SWEEP_DIR)` reloads all of those outputs in a later
session without recomputation. To re-run only the power selection from the
stored fits (for example after changing `target_anl` or the
`best_power_kwargs`) without re-rendering every per-tone fit plot, use
`ps.analyse_power_sweep(POWER_SWEEP_DIR, fit=False, plot_fits=False, ...)`.

## Apply Powers and Retune

Apply the selected powers while the tones are off resonance, then retune. This
keeps receive-path dynamic-range optimisation representative of the largest
transmitted signal level seen during the local sweep and the subsequent on/off
comparison, avoiding a setup that is safe only while the tones sit on the
dips:

```python
phases = client.generate_newman_phases(freqs)
parked_freqs = freqs - spans / 2

client.set_tone_frequencies(parked_freqs)
client.set_tone_phases(phases)
client.set_tone_powers(
    powers_dbm,
    reference_plane="detector",
    optimise_dynamic_range=True,
    rx_policy="maximise",
)

client.perform_retune(
    freqs,
    spans,
    points=401,
    samples_per_point=21,
    phases=phases,
    method="max_dphidf",
    wait=True,
)

calibration_sweep = client.parse_sweep_data(client.get_sweep_data())
on_resonance_freqs = client.get_tone_frequencies()
client.export_sweep(
    str(NOISE_DIR / "calibration_sweep.npy"),
    calibration_sweep,
)

plot_sweep(
    calibration_sweep,
    deembed=False,
    multi_tone="overlay",
    show_errors=True,
)
```

`perform_retune()` supports `max_gradient`, `min_mag`, and `max_dphidf`.
`max_dphidf` is useful when the desired operating point is the steepest phase
slope. The retune refreshes ADC calibration by default, freezes it before the
sweep, and leaves it frozen afterwards.

## Record On/Off Noise

Acquire an accumulated timestream and a batch of pre-accumulator snapshots in
both tone positions. Change only the tone frequencies between the on- and
off-resonance captures; do not rerun power optimisation between them.

For an upward sweep, `calibration_sweep["sweep_f"][0, :]` is the low edge of
each tone's local sweep and is a convenient off-resonance position:

```python
duration_s = 60
num_samples = int(client.get_sample_rate() * duration_s)

samples_on = client.parse_samples(client.get_samples(num_samples))
client.export_samples(str(NOISE_DIR / "samples_on.npy"), samples_on)
snapshots_on = client.batch_snapshots(
    num_snapshots=1000,
    export_file=str(NOISE_DIR / "snapshots_on.npz"),
)

off_resonance_freqs = calibration_sweep["sweep_f"][0, :]
client.set_tone_frequencies(off_resonance_freqs)
time.sleep(0.1)

try:
    samples_off = client.parse_samples(client.get_samples(num_samples))
    client.export_samples(str(NOISE_DIR / "samples_off.npy"), samples_off)
    snapshots_off = client.batch_snapshots(
        num_snapshots=1000,
        export_file=str(NOISE_DIR / "snapshots_off.npz"),
    )
finally:
    client.set_tone_frequencies(on_resonance_freqs)
```

The accumulated timestream captures all active tones simultaneously at the
normal output sample rate. `batch_snapshots()` captures pre-accumulator data
one tone after another at the higher FFT-output rate. Batch snapshots are
therefore suitable for per-tone high-frequency PSDs, but not for tone-tone
correlation analysis.

## Analyse One Tone

Reloading from disk makes the analysis independent of the acquisition
session:

```python
calibration_sweep = client.import_sweep(
    str(NOISE_DIR / "calibration_sweep.npy")
)
samples_on = client.import_samples(str(NOISE_DIR / "samples_on.npy"))
samples_off = client.import_samples(str(NOISE_DIR / "samples_off.npy"))
snapshots_on = client.import_batch_snapshots(
    str(NOISE_DIR / "snapshots_on.npz")
)
snapshots_off = client.import_batch_snapshots(
    str(NOISE_DIR / "snapshots_off.npz")
)

best_arrays = ps.best_power_arrays(POWER_SWEEP_DIR)
linewidths = best_arrays["empirical_linewidth_hz"]

tone_index = 0
reference_frequencies = np.asarray(
    samples_on["info"]["tones"]["frequencies_hz"]
)
reference_frequency = reference_frequencies[tone_index]
smooth_window_hz = linewidths[tone_index] / 3

plot_timestream_on_resonance(
    samples_on,
    calibration_sweep,
    tone_index,
    phase_center=True,
    deembed=True,
)

fig = plot_timestream_psd(
    samples_on,
    sweep_data=calibration_sweep,
    format="freq_diss",
    tones=[tone_index],
    psd_kwargs={"nperseg": samples_on["num_samples"] // 30},
    reference_tone_frequency=reference_frequency,
    label="on-res low-f",
    smooth_window_hz=smooth_window_hz,
)
plot_timestream_psd(
    samples_off,
    sweep_data=calibration_sweep,
    format="freq_diss",
    tones=[tone_index],
    psd_kwargs={"nperseg": samples_off["num_samples"] // 30},
    reference_tone_frequency=reference_frequency,
    fig=fig,
    label="off-res low-f",
    smooth_window_hz=smooth_window_hz,
)

for label, batch in (
    ("on-res high-f", snapshots_on),
    ("off-res high-f", snapshots_off),
):
    snapshot = batch["results"][tone_index]
    plot_snapshots_psd(
        snapshot,
        method="concatenated",
        psd_kwargs={
            "nperseg": snapshot["len_snapshot"] * snapshot["num_snapshots"] // 30
        },
        format="freq_diss",
        sweep_data=calibration_sweep,
        reference_tone_frequency=reference_frequency,
        fig=fig,
        label=label,
        smooth_window_hz=smooth_window_hz,
    )
```

Pass the on-resonance `reference_tone_frequency` explicitly when analysing the
off-resonance captures. Otherwise the plotting helper uses the parked tone
frequency recorded in the off-resonance timestream metadata.

`analyse_accumulator_snapshots()` remains available for inspecting legacy data
captured before the firmware snapshot fix. New captures made with the upgraded
firmware should not need that workaround.

## Compare Linearized and Möbius Conversion

The plotting examples above use the historical local tangent/normal estimate
by default. To compare that approximation against fitted-model inversion, fit
the parsed calibration sweep with the standard batch fitter and convert the
same timestream both ways:

> **Frequency-sign convention.** The low-level fitted `method="mobius"` and
> diagnostic `method="circle"` conversions return probe detuning relative to
> the fitted resonance, `f_probe - f_r`. Positive values place the probe above
> resonance. For fixed-tone detector analysis we usually want resonator
> detuning relative to the probe, `f_r - f_probe`, with the opposite sign. The
> high-level timestream helper below returns changes in that resonator-side
> convention, matching the historical linearized quadrature.

```python
import matplotlib.pyplot as plt

from souk_readout_tools import fitting
from souk_readout_tools.noise import (
    fractional_frequency_and_dissipation_timestreams,
)
from souk_readout_tools.resonator import ResonatorCalibration

fits = fitting.batch_fit(
    calibration_sweep,
    nonlinear=True,
    try_harder=True,
    n_jobs=-1,
    verbose=False,
)
calibrations = {
    fit.tone_index: ResonatorCalibration.from_fit(fit)
    for fit in fits if fit.success
}

linearized = fractional_frequency_and_dissipation_timestreams(
    samples_on,
    calibration_sweep,
    tones=[tone_index],
    reference_tone_frequency=reference_frequency,
    smooth_window_hz=smooth_window_hz,
    method="linearized",
)
mobius = fractional_frequency_and_dissipation_timestreams(
    samples_on,
    calibration_sweep,
    tones=[tone_index],
    reference_tone_frequency=reference_frequency,
    method="mobius",
    calibrations=calibrations,
)
circle = fractional_frequency_and_dissipation_timestreams(
    samples_on,
    calibration_sweep,
    tones=[tone_index],
    reference_tone_frequency=reference_frequency,
    method="circle",
    calibrations=calibrations,
)

fig, axes = plt.subplots(3, 1, sharex=True, figsize=(9, 8))
axes[0].plot(linearized["frequency"][0], label="linearized")
axes[0].plot(mobius["frequency"][0], label="Möbius")
axes[0].plot(circle["frequency"][0], label="circle coordinate")
axes[1].plot(linearized["dissipation"][0], label="linearized Delta(1 / 2Qi)")
axes[1].plot(mobius["dissipation"][0], label="Möbius Delta(1 / 2Qi)")
axes[2].plot(circle["dissipation"][0], label="circle Delta rho")
axes[0].set_ylabel("Fractional resonator-frequency motion")
axes[1].set_ylabel("Matched dissipation quadrature")
axes[2].set_ylabel("Radial proxy change")
axes[2].set_xlabel("Sample")
for axis in axes:
    axis.legend()
fig.tight_layout()
```

The high-level timestream helper reports changes relative to the sweep IQ at the
fixed probe frequency. Its frequency row is resonator motion, matching the
historical converter: fitted probe detuning is reference-subtracted and
sign-flipped. The fitted path removes the gain and cable-delay envelope at the
explicit probe frequency, phase-centres the resonance circle, applies exact
Möbius inversion for the asymmetric linear notch model, and analytically
inverts the Duffing coordinate when the fit has non-zero `anl`. To inspect the
intermediate driven-circle coordinate instead, pass `method="circle"`.

Both paths return the half-scaled `Delta(1 / (2 * Qi))` convention. This
matches the fractional-frequency quadrature and is useful for direct
frequency-versus-dissipation noise-spectrum overlays: isotropic amplifier
noise then appears at a similar level in both axes, while resonator noise is
commonly frequency dominated. Double the dissipation amplitude, or multiply
its PSD by four, for the commonly reported physical `Delta(1 / Qi)`
convention. The diagnostic `method="circle"` path instead returns the change in
signed radial proxy `Delta(abs(z_centered) / radius - 1)`. See
[`resonator_math_derivations.ipynb`](resonator_math_derivations.ipynb) for the
derivation.

The plotting helpers expose the same choice:

```python
plot_timestream_psd(
    samples_on,
    sweep_data=calibration_sweep,
    format="freq_diss",
    tones=[tone_index],
    conversion_method="mobius",
    calibrations=calibrations,
)
```

## Decorrelate Slow Noise

The slow accumulated timestream records all active tones simultaneously. That
makes it suitable for subtracting correlated readout or environmental noise
before computing PSDs. `plot_timestream_psd()` can remove the leading SVD
common modes across the calibrated fractional-frequency and dissipation
timestreams, then plot the cleaned PSD as an extra line:

```python
tone_metadata = samples_on["info"]["tones"]
regular_tones = tone_metadata.get("regular_indices")
if regular_tones is None or len(regular_tones) == 0:
    regular_tones = range(samples_on["num_tones"])

reference_frequency_by_tone = {
    tone_index: frequency
    for tone_index, frequency in enumerate(reference_frequencies)
}

plot_timestream_psd(
    samples_on,
    sweep_data=calibration_sweep,
    format="freq_diss",
    tones=[tone_index],
    psd_kwargs={"nperseg": samples_on["num_samples"] // 30},
    reference_tone_frequency=reference_frequency_by_tone,
    smooth_window_hz=smooth_window_hz,
    decorrelate_modes=1,
    decorrelate_plot="overlay",
    decorrelate_tones=regular_tones,
    label="on-res low-f",
)
```

The legend labels the cleaned line with `SVD N=1`. Use
`decorrelate_plot="replace"` to plot only the cleaned line, or increase
`decorrelate_modes` to remove more than one leading mode. Keep the mode count
small and inspect the raw overlay: an aggressive subtraction can remove real
correlated detector response. Pass regular-tone indices to
`decorrelate_tones` so blind monitors do not alter the detector decomposition.

If the capture includes blind monitors, subtract their readout-noise modes
first. The helper derives normalized amplitude and unwrapped-phase variations
from the simultaneous blind-tone timestreams, then regresses those modes out
of the regular-tone I/Q data before resonance calibration:

```python
blind_tones = tone_metadata.get("blind_indices")
if blind_tones is not None and len(blind_tones):
    plot_timestream_psd(
        samples_on,
        sweep_data=calibration_sweep,
        format="freq_diss",
        tones=[tone_index],
        psd_kwargs={"nperseg": samples_on["num_samples"] // 30},
        reference_tone_frequency=reference_frequency_by_tone,
        smooth_window_hz=smooth_window_hz,
        blind_tone_modes=1,
        blind_tones=blind_tones,
        decorrelate_modes=1,
        decorrelate_tones=regular_tones,
        label="on-res low-f",
    )
```

The cleaned line is labelled `Blind N=1, SVD N=1`. The operation order is:

1. Fit amplitude/phase modes from blind-tone timestreams.
2. Remove those modes from regular-tone I/Q timestreams.
3. Convert cleaned regular-tone I/Q to frequency/dissipation shifts using the
   regular-tone resonance sweep.
4. Remove detector-derived SVD modes from the calibrated regular-tone rows.

Blind tones are off-resonance readout monitors rather than detector
resonances, so their sweep traces are not useful for this subtraction. The
blind step is disabled by default and is simply omitted for older datasets
without blind monitors.

This operation applies only to the slow accumulated data. Pre-accumulator
snapshots are acquired one tone at a time, so they do not contain simultaneous
tone rows from which to estimate a common mode.

## Cross-Load Analysis

With the directory convention above, cross-load analysis can iterate over
`loads/BB_*/noise/run_000/` and read the load point's `load.json`. Keep the
raw capture filenames stable across loads. That makes the load point the only
outer variable and avoids special cases in later analysis notebooks.

The following notebook block loads one tone at each blackbody load, fits the
saved calibration sweep, and plots resonant frequency and `Qi` against load:

```python
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from souk_readout_tools.fitting import batch_fit

LOADS_DIR = Path("bb_campaign_2026-06-01") / "loads"
tone_index = 0
load_rows = []

for load_dir in sorted(LOADS_DIR.glob("BB_*")):
    noise_dir = load_dir / "noise" / "run_000"
    metadata = json.loads((load_dir / "load.json").read_text())
    load_k = metadata.get(
        "blackbody_temperature_k",
        metadata.get("blackbody_setpoint_k"),
    )
    if load_k is None:
        raise ValueError(f"No blackbody temperature recorded in {load_dir}")

    sweep = client.import_sweep(str(noise_dir / "calibration_sweep.npy"))
    fits = batch_fit(
        sweep,
        nonlinear=True,
        try_harder=True,
        n_jobs=-1,
        verbose=False,
    )
    fit_by_tone = {fit.tone_index: fit for fit in fits}
    load_rows.append(
        {
            "load_k": float(load_k),
            "noise_dir": noise_dir,
            "sweep": sweep,
            "fit": fit_by_tone[tone_index],
        }
    )

load_rows.sort(key=lambda row: row["load_k"])
load_k = np.asarray([row["load_k"] for row in load_rows])
fr_hz = np.asarray([row["fit"].fr for row in load_rows])
qi = np.asarray([row["fit"].Qi for row in load_rows])

fig, axes = plt.subplots(2, 1, sharex=True, figsize=(7, 6))
axes[0].plot(load_k, fr_hz / 1e9, "o-")
axes[0].set_ylabel("Resonant frequency (GHz)")
axes[1].semilogy(load_k, qi, "o-")
axes[1].set_xlabel("Blackbody load (K)")
axes[1].set_ylabel("Qi")
fig.tight_layout()
```

Inspect fit quality before interpreting these trends. If every load point was
retuned and captured after selecting its best powers, the rows compare each
load at its own optimised operating point. If powers were fixed for the
campaign, they instead compare the same drive condition across loads.

Overlay the saved resonance sweeps in fractional detuning units to make their
motion and shape changes easy to compare:

```python
colors = plt.cm.viridis(np.linspace(0.05, 0.95, len(load_rows)))
fig, ax = plt.subplots(figsize=(7, 4))

for row, color in zip(load_rows, colors):
    sweep = row["sweep"]
    fit = row["fit"]
    f = np.asarray(sweep["sweep_f"])[:, tone_index]
    z = (
        np.asarray(sweep["sweep_i"])[:, tone_index]
        + 1j * np.asarray(sweep["sweep_q"])[:, tone_index]
    )
    edge_points = np.r_[np.arange(20), np.arange(len(z) - 20, len(z))]
    off_resonance_magnitude = np.median(np.abs(z[edge_points]))
    relative_transmission_db = 20 * np.log10(
        np.abs(z) / off_resonance_magnitude
    )
    fractional_detuning = (f - fit.fr) / fit.fr
    ax.plot(
        fractional_detuning,
        relative_transmission_db,
        color=color,
        label=f'{row["load_k"]:g} K',
    )

ax.set_xlabel("Fractional detuning, (f - fr) / fr")
ax.set_ylabel("Relative transmission (dB)")
ax.legend(title="BB load")
fig.tight_layout()
```

Finally, plot the detector frequency-noise estimate at each load. This example
subtracts the off-resonance PSD from the on-resonance PSD and optionally
overlays the sequentially cleaned result. Blind-tone amplitude/phase cleaning
is applied before resonance calibration when monitors are present; regular-tone
SVD cleaning is then applied to the calibrated frequency-noise rows:

```python
from souk_readout_tools.noise import (
    fractional_frequency_and_dissipation_timestreams,
    remove_blind_tone_common_modes,
    remove_common_modes_svd,
)
from souk_readout_tools.plotting import compute_psd

SVD_MODES = 1  # Set to 0 to disable the cleaned overlay.
BLIND_TONE_MODES = 1  # Used only when blind monitors were captured.
PLOT_RAW = True


def plot_positive_psd_difference(ax, f, psd_on, psd_off, **kwargs):
    detector_psd = psd_on - psd_off
    keep = (f > 0) & (detector_psd > 0)
    ax.loglog(f[keep], detector_psd[keep], **kwargs)


fig, ax = plt.subplots(figsize=(7, 5))

for row, color in zip(load_rows, colors):
    noise_dir = row["noise_dir"]
    samples_on = client.import_samples(str(noise_dir / "samples_on.npy"))
    samples_off = client.import_samples(str(noise_dir / "samples_off.npy"))
    reference_frequencies = np.asarray(
        samples_on["info"]["tones"]["frequencies_hz"]
    )
    tone_metadata = samples_on["info"]["tones"]
    regular_tones = tone_metadata.get("regular_indices")
    if regular_tones is None or len(regular_tones) == 0:
        regular_tones = range(samples_on["num_tones"])
    blind_tones = tone_metadata.get("blind_indices")
    if blind_tones is None:
        blind_tones = []

    converted_raw_on = fractional_frequency_and_dissipation_timestreams(
        samples_on,
        row["sweep"],
        tones=regular_tones,
        reference_tone_frequency=reference_frequencies,
    )
    converted_raw_off = fractional_frequency_and_dissipation_timestreams(
        samples_off,
        row["sweep"],
        tones=regular_tones,
        reference_tone_frequency=reference_frequencies,
    )
    converted_row = {
        int(index): offset
        for offset, index in enumerate(converted_raw_on["tone_indices"])
    }[tone_index]
    nperseg = samples_on["num_samples"] // 30

    f, raw_on = compute_psd(
        converted_raw_on["frequency"][converted_row],
        samples_on["sample_rate"],
        nperseg=nperseg,
    )
    _, raw_off = compute_psd(
        converted_raw_off["frequency"][converted_row],
        samples_off["sample_rate"],
        nperseg=nperseg,
    )
    if PLOT_RAW:
        plot_positive_psd_difference(
            ax,
            f,
            raw_on,
            raw_off,
            color=color,
            linestyle=":",
            label=f'{row["load_k"]:g} K raw',
        )

    cleaned_samples_on = samples_on
    cleaned_samples_off = samples_off
    clean_steps = []
    if BLIND_TONE_MODES and len(blind_tones):
        cleaned_samples_on = remove_blind_tone_common_modes(
            samples_on,
            BLIND_TONE_MODES,
            regular_tones=regular_tones,
            blind_tones=blind_tones,
        )
        cleaned_samples_off = remove_blind_tone_common_modes(
            samples_off,
            BLIND_TONE_MODES,
            regular_tones=regular_tones,
            blind_tones=blind_tones,
        )
        clean_steps.append(f"Blind N={BLIND_TONE_MODES}")

    if SVD_MODES or clean_steps:
        converted_clean_on = fractional_frequency_and_dissipation_timestreams(
            cleaned_samples_on,
            row["sweep"],
            tones=regular_tones,
            reference_tone_frequency=reference_frequencies,
        )
        converted_clean_off = fractional_frequency_and_dissipation_timestreams(
            cleaned_samples_off,
            row["sweep"],
            tones=regular_tones,
            reference_tone_frequency=reference_frequencies,
        )
        clean_on = converted_clean_on["frequency"]
        clean_off = converted_clean_off["frequency"]
        if SVD_MODES:
            clean_on = remove_common_modes_svd(clean_on, SVD_MODES)
            clean_off = remove_common_modes_svd(clean_off, SVD_MODES)
            clean_steps.append(f"SVD N={SVD_MODES}")

        _, clean_on_psd = compute_psd(
            clean_on[converted_row],
            samples_on["sample_rate"],
            nperseg=nperseg,
        )
        _, clean_off_psd = compute_psd(
            clean_off[converted_row],
            samples_off["sample_rate"],
            nperseg=nperseg,
        )
        plot_positive_psd_difference(
            ax,
            f,
            clean_on_psd,
            clean_off_psd,
            color=color,
            label=f'{row["load_k"]:g} K; {", ".join(clean_steps)}',
        )

ax.set_xlabel("Frequency (Hz)")
ax.set_ylabel("On-res minus off-res frequency-noise PSD (1 / Hz)")
ax.legend(title=f"Tone {tone_index}")
fig.tight_layout()
```

Bins where the off-resonance PSD exceeds the on-resonance PSD are not positive
detector-noise estimates and cannot be shown on a logarithmic axis. The helper
above omits those bins rather than clipping them to an arbitrary floor.

## Related

- [Tone Power and Drive Tuning Notes](tone_power_notes.md)
- [Getting Started - Retuning](getting_started.md#retuning)
- [Getting Started - Pre-Accumulator Snapshots](getting_started.md#pre-accumulator-snapshots)
- [Calibration Guide](calibration.md)
