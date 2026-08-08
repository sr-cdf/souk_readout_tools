# Timeseries Analysis

`souk_readout_tools.timeseries` is a flat set of tools for getting numbers out
of timestreams: PSDs, band levels, `1/f` + white fits, on-minus-off detector
noise, cross-spectra, composite wideband spectra, `tau_qp`, glitch catalogues,
Allan deviations and export.

Pick the one that answers your question and call it. Every function takes plain
NumPy arrays plus a sample rate, and also accepts this package's own structures
(`parse_samples`, `parse_sweep_data`, `batch_snapshots` results). **Nothing here
requires a prior call to anything else here** — the only exception is
`stitch_psds`, which by its nature joins two spectra you already have.

```python
from souk_readout_tools import timeseries
```

## Task to function

| I want to... | Use |
|---|---|
| compute a PSD of one signal | `compute_psd` |
| compute PSDs of a whole timestream capture | `timestream_psds` |
| compute PSDs of pre-accumulation snapshots | `snapshot_psds` |
| average PSDs over repeated captures | `compute_psd_averaged` |
| treat repeats as one long record | `compute_psd_concatenated` |
| compute a PSD ignoring flagged samples | `masked_psd` |
| thin a PSD for a log-frequency axis | `log_bin_psd` |
| quote the noise level in a frequency band | `psd_band_levels` |
| separate the `1/f` and white parts | `fit_psd_model` |
| subtract an off-resonance readout floor | `subtract_psds` |
| join slow and fast PSDs into one wide trace | `stitch_psds` |
| model what the accumulator does to a spectrum | `accumulator_response` |
| fit `tau_qp` to a GR roll-off | `fit_gr_rolloff` |
| find cosmic-ray / glitch events | `find_glitches` |
| fit `tau_qp` to individual glitch pulses | `fit_glitch_pulses` |
| measure noise common to both quadratures | `cross_psd` |
| combine both quadratures into one estimate | `combine_quadratures` |
| measure or predict the responsivity ratio | `estimate_beta`, `beta_from_kernels` |
| get absolute responsivity from a load ladder | `signal_direction_from_loads` |
| see how long you can usefully integrate | `allan_deviation` |
| summarise a timestream, robustly | `timestream_stats` |
| save a result, reloadable without pickle | `export_psds`, `load_psds` |
| tabulate several results as one CSV | `export_summary` |

## The end-to-end detector-noise workflow

Take an on-resonance and an off-resonance capture at the same tone powers, plus
a calibration sweep.

```python
import numpy as np
from souk_readout_tools.timeseries import (fit_psd_model, psd_band_levels,
                                           subtract_psds, timestream_psds)

samples_on  = client.import_samples('samples_on.npy')
samples_off = client.import_samples('samples_off.npy')

# Both captures must be referred to the SAME responsivity -- see the warning
# below, this is the single easiest way to get a wrong answer here.
common = dict(
    format='freq_diss',
    tones=regular_tones,
    sweep_data=sweep,
    reference_tone_frequency=np.asarray(
        samples_on['info']['tones']['frequencies_hz']),
    psd_kwargs={'nperseg': samples_on['num_samples'] // 30},
)

on  = timestream_psds(samples_on,  **common)
off = timestream_psds(samples_off, **common)

detector = subtract_psds(on, off, clip=None)          # clip=None to quote a number
levels   = psd_band_levels(detector, bands={'science': (1.0, 10.0)})
fits     = fit_psd_model(detector, f_range=(0.1, 100))
```

`detector` has the same shape as a `timestream_psds` result, so `psd_band_levels`
and `fit_psd_model` take it directly. `positive_fraction` says whether the
subtraction resolved anything at all: near 1 the detector noise is above the
readout floor, near 0.5 the two captures agree within their scatter and the
honest statement is an upper limit.

### Four ways this goes wrong quietly

These all return believable numbers with no error, so they are worth knowing
before the first plot.

**1. Referring the two captures to different responsivities.** The off-resonance
tones are parked away from the dip, where `|dS21/df|` is small. Left to default,
each capture converts at its own recorded frequency, so the off-resonance
conversion divides by a near-zero gradient. On the August 2026 data that
inflated the off-resonance noise **1413×**. Pass the on-resonance frequencies as
`reference_tone_frequency` to *both* calls. `subtract_psds` warns if it detects
the mismatch, and `timestream_psds` records what it converted at.

**2. Averaging a clipped difference.** `subtract_psds(clip='positive')` NaNs the
non-positive bins, which is what a log axis needs — but averaging what survives
keeps only the upward fluctuations. A difference that is truly zero then comes
back at about `S_floor * sqrt(2/n)`: 6% of the readout floor over 390 Welch
averages, 24% over 22. Use `clip=None` for any number. `psd_band_levels` and
`fit_psd_model` warn when handed a clipped result.

**3. Subtracting the phase row across the dip.** Additive readout noise puts the
same power into `i`, `q` and `magnitude` whatever the carrier, so those subtract
cleanly. `phase` carries it as `S_n / A**2`, and on resonance the carrier sits in
the dip — so an on/off `magphase` difference reports most of the readout noise as
detector noise. Measured at **161×** with `positive_fraction = 1.00`, which looks
like the cleanest signal in the dataset. Use `format='freq_diss'`.

**4. Quoting `alpha` from a spectrum the model does not describe.**
`fit_psd_model` flags `high_residual` when the `1/f`-plus-white form does not fit
— on real data with vibrational lines that fires on most tones, and `alpha` is
not quotable there. It also flags `white_degenerate`: as `alpha` goes to zero the
two terms become indistinguishable and `W` alone can land an order of magnitude
below the true floor on an excellent-looking fit. Read `A + W` as the flat level
in that case.

## Estimator biases, and the `n_averages` argument

A PSD bin averaged over `n` Welch segments is `S * chi2_2n / (2n)`, which is
unbiased in the mean but not in the log or the median. Two functions take
`n_averages=` to undo that, off by default:

- `fit_psd_model` fits in log space, where `E[log10 S] = log10 S + (psi(n) - ln n)/ln 10`
  — **0.25 dex, a factor 1.8, for a single-segment periodogram**, negligible past
  ~50. `alpha` and `f_knee` are unaffected; only the levels move.
- `psd_band_levels(statistic='median')` reads 31% low in power on one segment,
  because the median of `chi2_2/2` is `ln 2`. Still the right statistic when a
  vibrational line sits in the band.

`n_averages` does **not** apply to a `subtract_psds` result: the difference of
two chi-square bins is not chi-square. Both functions warn if you try.

For coherence there is a third: `E[gamma^2] = 1/n` assumes *independent*
segments, and 50% Hann overlap gives about 5% fewer. `cross_psd` computes `n_eff`
from the window autocorrelation and reports `coherence_bias`, `coherence_95` and
`coherence_significant`. Using the raw segment count instead turned a null into
an apparent 3.8-sigma detection on real data.

## Glitches

Cosmic rays break Cooper pairs and produce a pulse decaying at `tau_qp`. This
matters beyond data quality: **by Campbell's theorem a Poisson train of such
pulses has the same Lorentzian shape and the same corner frequency as GR noise.**
Contaminating a spectrum inflates the fitted `S_0` by 1.6× while leaving `tau_qp`
untouched, with no bad-fit flag. Only the time domain separates them.

```python
from souk_readout_tools.timeseries import find_glitches, masked_psd

events = find_glitches(samples_on, threshold=5)
f, psd, *_ = masked_psd(row, sample_rate, events['any_mask'])
```

`find_glitches` high-passes with a running median before thresholding on MAD — a
plain sigma cut trips constantly on `1/f` drift. `masked_psd` drops whole Welch
segments by default, which is unbiased and assumption-free; read `lost_fraction`,
not `flagged_fraction`, as the deadtime.

Two things to set deliberately. On a stream that resolves the pulse shape, set
`min_separation`, or one impact is catalogued several times as it crosses the
threshold on the way down. And **deglitch before common-mode removal** — a single
impact hits many resonators at once, so an SVD run first absorbs the glitch into
the common mode and smears it across tones that were never hit.

Rates from the accumulated stream are lower limits: the accumulator averages the
pulse over its whole window, diluting the amplitude by roughly `tau_qp / T_acc`,
about 0.06 at `acc_len = 1000`.

## Composite wideband spectra and `tau_qp`

The accumulated stream covers `1/T` to `fs_slow/2`; the snapshots cover
`fs_fast/len_snapshot` to `fs_fast/2`. At `acc_len = 1000` those are 300 Hz and
586 Hz, so **there is a gap, not an overlap**. `stitch_psds` returns it in `gaps`
and never interpolates across it.

To close the gap and gain a real overlap for cross-validation, take the
accumulated stream at `acc_len <= 256`. That is an acquisition choice; no
analysis can repair it afterwards.

```python
from souk_readout_tools.timeseries import fit_gr_rolloff, snapshot_psds, stitch_psds

slow = timestream_psds(samples_on, **common)
fast = snapshot_psds(batch, format='freq_diss', sweep_data=sweep, tones=regular_tones)
wide = stitch_psds([slow, fast])

gr = fit_gr_rolloff(wide, fr=fit.fr, Qr=fit.Qr, f_range=(200, 3e5),
                    masked_fraction=events['flagged_fraction'].mean())
```

### Do not deconvolve the accumulator

Averaging `acc_len` samples and decimating by `acc_len` builds consecutive
outputs from *disjoint* inputs, so for white input the measured PSD is exactly
flat — verified flat to ±0.07 dB where the boxcar's own `sinc²` is −3.92 dB at
Nyquist. The sinc attenuation and the aliasing cancel. Dividing a measured trace
by that response would manufacture a spurious rise.

The cancellation is exact only for white input. Where the spectrum is already
falling there is less power in the alias zones than the boxcar removes, and a
genuine droop appears — 0.91× at the top of the slow band for a 100 µs roll-off.
`accumulator_response(f, acc_len, fs_fast, spectrum=...)` returns that droop as a
**forward** model: fit on the fast trace, push the model through the accumulator,
compare. Inverting it is ill-posed, because the fold depends on the spectrum
being solved for. `stitch_psds` flags the affected band in `alias_affected`
rather than repairing it.

### The resonator pole is not optional

The measured GR spectrum is two-pole:

```
S(f) = S_0 / [(1 + (2 pi f tau_qp)^2) (1 + (2 pi f tau_res)^2)] + white
```

with `tau_res = Q_r / (pi f_r)`. At `Q_r = 1e5` and `f_r = 1 GHz` that is ~32 µs
against a `tau_qp` of ~100 µs — the same decade. Fitting a single Lorentzian to a
true 100 µs returns **283 µs**. `tau_res` is therefore fixed from the calibration
fit, never fitted, and `fit_gr_rolloff` refuses to guess it.

`fit_glitch_pulses` gives an independent `tau_qp` from the pulse decay, with
almost disjoint systematics — this one limited by the stitch, the aliasing and
the white floor, the other by window truncation and pile-up. Agreement between
them is strong evidence; disagreement localises which model is wrong.

## Combining the two quadratures

`cross_psd` measures what the frequency and dissipation channels have in common.
Readout and TLS noise are private to each and cancel; photon and GR noise are
common and survive. That makes the coherence a responsivity-independent
measurement, and the cross-spectrum the cleanest input to `fit_gr_rolloff`.

`combine_quadratures` then combines them by generalised least squares. Whether
it is worth doing depends on the regime:

- **Readout-limited** (both channels at the same floor): combining buys
  `sqrt(1 + 1/(4 beta**2))` — 3.8% at `beta = 1.8`, 0.8% at 4, less as `beta`
  grows. `beta=None` gives frequency-only, which is a legitimate answer.
- **TLS- or vibration-limited frequency channel with a readout-limited
  dissipation channel**: the gain is real. On a synthetic tone in that regime,
  `weights='closed_form'` returned 21× against 1.03× for scalar weights.

### `beta`, and the factor of two

`beta` is the responsivity ratio. **The pipeline's dissipation row is
`Im(delta_tilde)`, already half of `delta_Qi_inv`**, so referring it to
frequency-equivalent units multiplies by `2 * beta` and the scalar weight is
`1 / (1 + 4 beta**2)`. Applying the half a second time puts every combined result
out by a factor of 2.

Three sources, in decreasing order of what they require:

1. `signal_direction_from_loads(...)` — a blackbody load ladder, the only route
   to an absolute NEP. It needs absorbed power in **watts**, which the readout
   data cannot supply: pass the optical model as a `(temperature_k, power_w)`
   curve and it interpolates at each load, or as a callable `P(T)`.
   ```python
   direction = signal_direction_from_loads(
       load_k, fr_per_load, Qi_per_load, power_curve=(T_cal, P_cal))
   ```
   Without the power scale it still gives `beta`, but then it has measured
   nothing tier 2 does not — at the cost of a whole load ladder.
2. `estimate_beta(...)` — in-band regression, timestream-only, enough for the weights.
3. `beta_from_kernels(Tc, T, fr)` — Mattis-Bardeen, no measurement at all.

Tier 3 is respectable rather than a fudge precisely because the answer is
insensitive to `beta`. But **`beta` is a strong function of readout frequency
and temperature**, so a value quoted for another instrument does not transfer.
For this instrument's Al-Ti-Al film (`Tc` ~ 1 K) the kernels give:

| `fr` | 100 mK | 150 mK | 200 mK | 300 mK |
|---|---|---|---|---|
| 0.5 GHz | 6.9 | 9.6 | 12.2 | 17.4 |
| 1.0 GHz | 4.5 | 6.1 | 7.6 | 10.7 |
| 2.0 GHz | 3.1 | 4.1 | 5.0 | 6.8 |

The assumed gap ratio barely matters — `beta` moves only 6.3 to 5.9 across
`gap_ratio` 1.4 to 2.2 — so the proximitised trilayer needs no special
treatment. `estimate_beta`'s default `beta_range` is deliberately wide enough to
span that whole table; narrow it with `beta_from_kernels` for a real check. A
measured ratio far outside it means de-embedding leakage, not physics — the same
fault the off-resonance coherence test catches independently.

## Export

Every analysis function takes `export=<path>` and returns its result unchanged,
so it drops into an existing call. `export_psds` flattens the per-panel arrays on
write, so the file reads back under numpy's default `allow_pickle=False`:

```python
levels = psd_band_levels(detector, bands=..., export=run_dir)
back   = load_psds(run_dir + '/analysis/timeseries.npz')   # no pickle needed
export_summary(run_dir, ('on', on_levels), ('off', off_levels), ('diff', levels))
```

`export_summary` writes one row per `(tone, panel, band)`, so on, off and the
difference land in one table.

## A worked session

A complete run against a real capture — a 60 s accumulated stream at 600 Hz,
100 x 1024 snapshots at 600 kHz, 379 tones, `acc_len = 1000`. A runnable
version of this is in `nep_august2026/20260708/timeseries_examples.py`.

```python
import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools.fitting import fit_sweep_stack
from souk_readout_tools.plotting import (plot_allan_deviation, plot_coherence,
                                         plot_composite_psd,
                                         plot_timestream_histogram)
from souk_readout_tools.timeseries import (allan_deviation, cross_psd,
                                           fit_gr_rolloff, psd_band_levels,
                                           snapshot_psds, stitch_psds,
                                           subtract_psds, timestream_psds,
                                           timestream_stats)

D = 'nep_august2026/20260708/BB_35.9K/'
load = lambda n: np.load(D + n, allow_pickle=True).item()
sweep    = load('calibration_sweep_a=0.1.npy')
s_on     = load('samples_on_a=0.1.npy')
s_off    = load('samples_off_a=0.1.npy')
snaps_on = ReadoutClient.import_batch_snapshots(D + 'snapshots_on.npz')

TONES = [0, 60, 120]
REF = np.asarray(s_on['info']['tones']['frequencies_hz'])   # on-resonance!
common = dict(format='freq_diss', tones=TONES, sweep_data=sweep,
              reference_tone_frequency=REF,
              psd_kwargs={'nperseg': s_on['num_samples'] // 30})
```

**1 — on minus off.** `clip=None` because we want numbers, not a log plot.

```python
on, off  = timestream_psds(s_on, **common), timestream_psds(s_off, **common)
detector = subtract_psds(on, off, clip=None)
levels   = psd_band_levels(detector, bands={'science': (1.0, 10.0)})
```

On this capture the frequency channel sits 2.4-92x above the off-resonance
floor depending on tone, with `positive_fraction` 0.98-1.00 — resolved
everywhere, so the band levels mean something.

**2 — how long can you integrate?**

```python
plot_allan_deviation(s_on, format='freq_diss', tones=TONES,
                     sweep_data=sweep, reference_tone_frequency=REF)
ad = allan_deviation(s_on, format='freq_diss', tones=TONES,
                     sweep_data=sweep, reference_tone_frequency=REF)
```

Here `tau_min` came out at 3.4 s on every tone, with log-log slopes of -0.87,
-0.34 and -0.49 — the last is textbook white, the first steeper and the second
shallower, which is the kind of spread the single-number ASD hides.

**3 — is the distribution what a PSD assumes?**

```python
plot_timestream_histogram(s_on, format='freq_diss', tones=TONES,
                          sweep_data=sweep, reference_tone_frequency=REF)
stats = timestream_stats(s_on, format='freq_diss', tones=TONES,
                         sweep_data=sweep, reference_tone_frequency=REF)
```

`std/mad_std` was 0.86-1.05 with 0-4 outliers per tone, i.e. clean. A ratio well
above 1 is the signal that a PSD has quietly averaged a glitch in.

**4 — coherence, against its own floor.**

```python
cx = cross_psd(s_on, sweep_data=sweep, tones=TONES,
               reference_tone_frequency=REF,
               psd_kwargs={'nperseg': s_on['num_samples'] // 30})
plot_coherence(cx)
```

`n_avg` 59 but `n_eff` **55.9** — the overlap correction matters — giving a bias
floor of 0.018 and a 95% line at 0.053. Mean coherence in 1-10 Hz was
0.026-0.058, i.e. straddling that line: 12-47% of bins significant. This is
exactly the regime where quoting the raw coherence would invent a detection.

**5 — the composite, and `tau_qp`.**

```python
fast = snapshot_psds(snaps_on, format='freq_diss', tones=TONES,
                     sweep_data=sweep, reference_tone_frequency=REF,
                     psd_kwargs={'nperseg': 1024})
slow = timestream_psds(s_on, **common)
wide = stitch_psds([slow, fast])
plot_composite_psd(wide, tone=0)
```

Slow band 0.5-300 Hz, fast band 586 Hz-300 kHz, so `gaps == [(300.0, 586.0)]`
and `alias_affected == (30.0, 300.0)` — both shaded on the plot. Snapshot
`f_resolution` 586 Hz, `bin_oversampling` 1, `block_drift` 0.068 against a null
of 0.031.

**`tau_res` has to come from the real fit, per tone.** On these three tones it
runs 8.1, 55.4 and 25.8 us — a single value would be wrong for two of them:

```python
f_sw = np.asarray(sweep['sweep_f'])
z_sw = np.asarray(sweep['sweep_i']) + 1j * np.asarray(sweep['sweep_q'])
fits = fit_sweep_stack(f_sw[:, TONES].T, z_sw[:, TONES].T, nonlinear=True)
fr_hz = np.array([r.fr for r in fits])
Qr    = np.array([r.Ql for r in fits])          # loaded Q

gr = fit_gr_rolloff(wide, fr=fr_hz, Qr=Qr, f_range=(1e3, 2e5))
plot_composite_psd(wide, tone=0, gr_fit=gr)
```

Read the flags before the numbers. Here tone 0 fitted cleanly at 78.7 us; tone
60 returned 75.2 us flagged `poorly_separated` (its `tau_res` is 55.4 us, within
a factor of 2, so that is a limit not a value); and tone 120 returned
`corner_above_band,tau_at_bound`, meaning the roll-off is not in the fitted
range at all. **One of three is quotable** — which is the point of the flags.

## Plotting

Each analysis function has a thin plotting consumer that takes the same
arguments, or a precomputed result via `result=`:

| Function | Plot | What the plot adds |
|---|---|---|
| `timestream_psds` | `plot_timestream_psd(..., return_data=True)` | the figure and the arrays together |
| `snapshot_psds` | `plot_snapshots_psd` | per-repeat error bars |
| `allan_deviation` | `plot_allan_deviation` | `tau_min` marked, so the useful integration time is readable |
| `timestream_stats` | `plot_timestream_histogram` | the Gaussian `mad_std` implies, with `std/mad_std` in the title |
| `cross_psd` | `plot_coherence` | the `1/n_eff` bias floor shaded and the 95% line drawn |
| `stitch_psds` | `plot_composite_psd` | the gap and alias band shaded, overlap ratio annotated, optional GR overlay |

```python
from souk_readout_tools.plotting import plot_coherence, plot_composite_psd

plot_coherence(cross_result)                       # or a raw capture + sweep_data
plot_composite_psd(wide, gr_fit=gr, tone=0)
```

The coherence floor is the reason that plot exists: magnitude-squared coherence
is biased up by `1/n_eff` even for completely independent channels, so a trace
only means something where it clears the shaded band.

## Related

- [Resonator Noise Workflow](resonator_noise_workflow.md) — acquiring the captures
- [Accumulators](accumulators.md) — what `acc_len` does, and why it sums
- [Calibration Guide](calibration.md)
