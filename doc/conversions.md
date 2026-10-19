# Conversions: from I/Q to resonator quantities

Every conversion of raw I/Q into a named physical quantity lives in
`souk_readout_tools/resonator.py`, and everything else — offline analysis
(`timeseries`, `noise`, `modulation`), the live viewer, the server's tone
tracker and stream-to-DAC — calls it. One vocabulary, one sign convention,
one set of formulas, whichever stage of the workflow you are in. This page is
the reference for all three; the module docstring has the task-to-function
table.

## Conventions

- **Resonance shift** is resonator-side: positive when the resonance moves
  up in frequency.
- **Detuning** is **resonance minus tone**: positive when the resonance sits
  above the tone, i.e. when the tone should move up. For demodulated data the
  tone is the modulation centre: `center_detuning_*`.
- **On resonance** means the phase inflection — the steepest point of the
  de-embedded response, zero phase curvature, the best signal-to-noise point
  for a frequency readout and the point tracking steers to
  (`resonator.operating_point_hz`). For a linear fit it is `fr` at any
  asymmetry; a driven (Duffing) fit displaces it, and past the bifurcation
  (`anl ≥ 4√3/9`, `resonator.BIFURCATION_ANL`) it is a cliff edge — such
  tones are flagged.
- **Matched dissipation** is `Δ(1/(2·Qi))`, positive for more loss: the
  factor 1/2 is applied, so it sits on the same scale as the fractional
  frequency shift `Δf/f` and the two can be overlaid directly.
- **Linewidth units** use the calibration's reference linewidth `fr/Ql` —
  fixed, never an instantaneous estimate, so a changing loss cannot reshape
  the signal.

## Quantities

`resonator.QUANTITIES` maps each name to its label and unit;
`resonator.canonical_quantity` resolves the older and shorter aliases
(`frequency`, `dissipation`, `mag`, `logmag`, `freq_lw`, …).

| group | quantities | needs |
|---|---|---|
| I/Q views | `i`, `q`, `amplitude`, `log_amplitude`, `phase`, `unwrapped_phase` | nothing (raw, or in a requested basis) |
| fixed-basis views | `centered_phase`, `centered_rotated_phase`, `centered_radius` | a calibration's circle |
| resonance | `resonance_shift_hz`, `fractional_resonance_shift`, `resonance_shift_linewidths`, `resonance_frequency_hz` | a sweep or a calibration (linewidths and absolute: a calibration) |
| detuning | `detuning_hz`, `fractional_detuning`, `detuning_linewidths` | a calibration |
| dissipation | `matched_dissipation_shift`, `matched_dissipation_hz`, `matched_dissipation_linewidths` | a sweep or a calibration |
| internal loss | `internal_loss` (`1/Qi`), `internal_loss_shift` (`Δ(1/Qi)`), `fractional_internal_loss_shift` (`Qi·Δ(1/Qi)`) | a calibration from a fit (for `Qi`) |
| per modulation cycle | `center_detuning_hz`, `center_detuning_linewidths`, `dphi_df`, `d2phi_df2`, `curvature_over_slope` | modulated data (detuning: a calibration) |

Every conversion produces the pair (resonance shift in Hz, matched
dissipation); `resonator.express` turns that pair into any of the resonance,
detuning, dissipation and loss quantities, so they cannot disagree.

## Bases

Raw I/Q is taken into a basis by one complex multiply-add, `A·z + B`
(`resonator.basis_coefficients`, `to_basis`), computed once per tone and
probe frequency `f`:

| basis | transform |
|---|---|
| `raw` | `z / g` |
| `deembedded` | `z_d = (z / g) · M(f)`, `M(f) = e^{2πifτ} / (a·e^{iα})`: cable delay `τ` (or a group-delay table) and gain removed at `f` |
| `centered` | `z_d − c`: about the fitted circle centre `c` |
| `centered_rotated` | `(z_d − c) · e^{iθ}`: rotated so the resonance sits at phase 0 on the positive real axis |

Two scale factors are divided out before any of it:

- **RX gain** `g`: the data's analog RX gain over the calibration sweep's,
  from the chain states both recorded (`recorded_rx_gain_ratio`; RF frontend
  total and ADC DSA; a PFB-shift mismatch is refused, not modelled).
  `match_rx_gain='auto'` (the default everywhere) applies it with a warning
  when they differ. The data is divided; the sweep is never rescaled.
- **Readout correction**: the filterbank compensation's receive half cannot
  land in hardware, so the server reports per-(point, tone) factors that
  `parse_samples` / `parse_stream` apply (`with_readout_correction` toggles
  them). A tracking commit moves the probes across the filterbank, so each
  modulation revision has its own factors; recordings carry them in their
  tone updates.

On the centred, rotated circle the linear notch is

```
z(f) = r · (1 − 2iQl·x) / (1 + 2iQl·x),   x = (f − fr) / fr
φ(f) = −2 atan(2 (f − fr) / lw),          lw = fr / Ql,   dφ/df = −4/lw on resonance
```

About the raw origin none of this holds: the phase there mixes loss into
frequency and its derivatives read the detuning 13–100× high. Anything that
needs the phase to mean something uses the centred basis.

## Fixed-tone conversions

`prepare_conversion(f_tone, method, calibration=, sweep_f=, sweep_z=)` does
everything that does not depend on the samples; `convert_iq(prepared, z)`
then returns `(resonance_shift_hz, matched_dissipation_shift)`, relative to
the sweep's I/Q at the tone (the resonator's state during the sweep) or,
without a sweep, to the fitted state. `frequency_and_dissipation` is the
one-shot form.

| method | how | needs |
|---|---|---|
| `linearized` | tangent/normal projection on the sweep's local gradient `g = dz/df`: with `u = (z − z_ref)/g`, shift `= −Re u`, `fr·Δ(1/2Qi) = −Im u` | sweep; any basis (the map is conformal) |
| `phase` | the centred phase and log-radius over the centred slope: with `w = ln(z_c / z_ref)`, shift `= −Im w / (dφ/df)`, `fr·Δ(1/2Qi) = Re w / (dφ/df)` | calibration (centring); slope and reference from the sweep, else the model (linear fits) |
| `mobius` | exact inversion of the fitted circle, `ResonatorCalibration.convert_centered_iq`; the Duffing relation is inverted analytically for driven fits | calibration |
| `lookup` | inverts the sweep's whole centred phase curve: follows large signals, NaN beyond the swept span | sweep (the calibration, if given, sets the centring) |
| `circle` | diagnostic: the circle coordinate, and the radial proxy `Δ(|z_c|/r)` in place of dissipation | calibration |

`linearized` and `phase` are the same first-order map (on the circle
`g = i·(dφ/df)·z_c`, so `u = −i·w/(dφ/df)`): both separate frequency from
dissipation exactly to first order. Raw phase over a raw slope does not —
it read 98 Hz of frequency from a 141 Hz-equivalent loss step in the tests.
`resonator.convert_samples` applies any of these to a parsed capture, by
name.

## Three-point modulation

With frequency modulation each cycle probes the resonator at a few offsets
around the modulation centre (`doc/frequency_modulation.md`). The kernel,
`resonator.phase_derivatives`, takes the per-point I/Q of a cycle (or of
blocks of cycles, averaged coherently) in the working basis:

- It works on the lowest, central and highest *offsets*, whatever the slot
  order, averaging points that share an offset coherently: `z_l`, `z_m`,
  `z_h` at `f_l < f_m < f_h`.
- Phase slope and curvature at the centre, from angles of products (nothing
  is unwrapped) — `method='fast'`, the tracker's:

  ```
  φ'  ≈ [arg(z_m z̄_l) + arg(z_h z̄_m)] / (f_h − f_l)
  φ'' ≈ 2 [arg(z_h z̄_m)/(f_h − f_m) − arg(z_m z̄_l)/(f_m − f_l)] / (f_h − f_l)
  ```

  `method='fit'` fits a quadratic to the unwrapped phase against offset
  instead, so every point counts (the same numbers for three distinct
  offsets).
- **Centre detuning**, resonance (its operating point) minus centre. Two
  estimators; the default everywhere is the first:

  - **Möbius** (`mobius_center_detuning`): each point's centred I/Q is
    inverted with the fitted model — the circle's Möbius map and, for a
    driven fit, the analytic Duffing inverse — into that probe's detuning
    from `fr`; less the point's offset, averaged over the points, plus
    `operating_point − fr`:

    ```
    d_centre = mean_p[ f_p − (f_probe,p − fr)_mobius ] + (operating_point − fr)
    ```

    Right in scale for driven resonators; works with any number of offsets,
    none included (unmodulated tracking); leans on the fitted rotation and
    nonlinearity. With a [sweep table](#the-sweep-table) (a record made with
    its sweep) each probe's detuning is read off the sweep's own frequency
    axis rather than the fitted shape.
  - **Curvature** (`detuning_from_derivatives`), in linewidths:

    ```
    d = φ'' / (2 φ' |φ'|)
    ```

    exact for the centred *linear* notch's true derivatives (invert
    `φ = −2 atan(2x)`), whatever the phase offset — a rotation error
    cancels — and without a linewidth. The finite differences read it about
    1% low at ±0.1-linewidth offsets and 5% at ±0.25. For a driven response
    only its zero, the inflection, is right: on krm4 tones driven to
    `anl ≈ 0.3` it read 2× (3–3.5× by the fit), and a gain-1 tracker then
    jumped from one side of the resonance to the other every commit.
- **Per-point gradients**, exact for a response that is a Möbius function of
  frequency — a resonance circle in any affine basis. With the difference
  quotients `D_ab = (z_b − z_a)/(f_b − f_a)`:

  ```
  dz/df = D_lm·D_mh / D_lh   at the centre (the harmonic mean of the one-sided quotients)
          D_lm·D_lh / D_mh   at the lowest offset
          D_mh·D_lh / D_lm   at the highest
  local dφ/df = Im(dz/df / z)
  ```

  A central difference would read the circle's chord for its tangent — 3.8%
  low at ±0.1-linewidth offsets on resonance, 20% at ±0.25 — and scale every
  converted shift by as much. (The `φ'` above is also low, by
  `atan(u)/u`, `u = 2·offset/linewidth`: 1.3% at ±0.1 linewidth. It feeds
  only the detuning.)

The per-cycle or per-sample conversion is then the fixed-tone one with the
live slope and gradient: `conversion='phase'` (the calibrated default, the
preferred three-point conversion: `w = ln(z_c/z_ref)` over the live local
slope — linear operations on the centred phase and log-radius), `'linear'`
(the uncalibrated default: `u = (z − z_ref)/g` with the live gradient) or
`'mobius'` (the calibration's inversion). `modulation.demodulate` gives one
value per cycle; `demodulate_timestream` converts every sample about its own
point's reference, so no integration time is lost.

### Recentres: the reconstruction identity

Tracking and recentres move the modulation centre; each move is a revision,
stamped in every frame and announced in the stream (`TONE_UPDATE`). The
resonance is always

```
f_res(t) = f_centre[rev(t)] + d_centre(t)
```

`modulation.demodulate_stream` (offline) and the live viewer demodulate each
revision with its own state and join them onto one reference, so
`resonance_shift_hz` is the resonator's own motion and never steps at a
commit: a calibrated tone through its absolute resonance, and its
`matched_dissipation_shift` through the same inversion's absolute loss, each
revision's reference inverted with the calibration (exact, free of the
finite-difference bias, as long as the calibration's frequency scale is right:
a fit off by `k` near the operating point steps by `(1 − k)` × the move, which
is what the [sweep table](#the-sweep-table) takes out); an uncalibrated tone by
carrying the reference I/Q along the measured gradient by the probe move (first
order in the accumulated move: for drifts approaching a linewidth, use a
calibration). Nothing is matched across a join, so a real change at a commit is
kept. Stream-to-DAC keeps one
reference and adds the centre steps, `(f_centre[rev] − f_centre[ref]) +
shift about the reference` — exact to first order in the centred basis,
which depends on the detuning alone.

### The tracker's estimator

`detuning_estimator='mobius'` (the default) tracks with the Möbius centre
detuning above; it needs a trustworthy calibration but no probe offsets, so
it also runs on a zero-offset modulation — tracking without modulation.
`'curvature'` needs three distinct offsets and no fitted rotation, which
makes it the choice when the rotation is in doubt (a stale calibration after
a re-arm) and the tones are close to linear. The live viewer's and the
demodulators' `center_detuning_*` use the Möbius form for calibrated tones,
so they show what the tracker acts on.

## Calibration records: how a calibration travels

A calibration is a plain, JSON-safe dict
(`schema: 'souk-resonator-calibration/1'`), made from fits or
`ResonatorCalibration` objects by `resonator.calibration_record`:
`tone_indices` and per-tone lists `fr_hz`, `Ql`, `Qi`, `tau_s`, circle
centre and radius, rotation, gain, `anl`, `operating_point_hz`,
`linewidth_hz`, `bifurcated`, plus the shared group-delay table and a
`provenance` block (creation time, software version, and the sweep's date,
RX chain gain and PFB shift for gain matching).

```python
from souk_readout_tools import fitting, resonator

sweep = client.parse_sweep_data(client.get_sweep_data())
fits = fitting.batch_fit(sweep)
client.set_resonator_calibrations(fits, sweep_data=sweep)          # to the server
resonator.save_calibrations('cal.npz', fits, sweep_data=sweep)     # and/or a file
record, sweep = resonator.load_calibration_record('cal.npz')
```

Pushed to the server, the record is served in `get_info('resonators')` and so
recorded in every stream's metadata; the tracker centres with it, and the
analysis tools, the live viewer and stream-to-DAC pick it up from any
capture by default (`resonator.calibration_from_data`). Staleness is
worked out afresh at every read, per tone: a tone no longer active, or tuned
more than five linewidths from its calibrated resonance, is dropped from the
served record and listed in `stale_tones`; the rest stays in use.

Whether a calibration still *fits* is a separate question —
`resonator.calibration_fit`, the median |z_c| / r of a tone's centred data,
is 1 while its circle still fits, and leaves 1 when the resonance changes
shape (a cooldown changes `Qi`, coupling and drive) or the levels change.
The tracker reports it per tone and stream-to-DAC for its channel.
`resonator.calibration_quality(record, sweep, capture)` checks a whole
calibration — fitted vs measured linewidth, linewidth odd for its frequency
neighbours, near bifurcation, shallow dip, the sweep table below, and this
fit on a fresh capture — and returns the `good` tones, e.g. for
`enable_tracking(enable_mask=...)`.

A saved `.npz` (no pickle) holds the record and, given `sweep_data`, the
sweep itself, so the sweep-based methods can be redone from the file alone.
Anywhere that takes `calibrations=` accepts fits, `ResonatorCalibration`
objects or a record.

### The sweep table

A fit describes its own sweep only approximately. Read back through the
fitted model, the sweep's points near the operating point come out at the
wrong frequencies by about 2% of their detuning (krm4, 2026-09-28: median
2.1%, 90th percentile 6%, worse on driven tones fitted as linear), and every
tracking commit then stepped the tracked resonance by that error times the
move. So a record made with its sweep (`sweep_data=`, as above) carries a
**sweep table** per tone (`resonator.sweep_frequency_correction`): the
swept minus the modelled frequency, tabulated against the modelled one over
the operating point ±2 linewidths, 1/20 of a linewidth between knots. The
calibration adds it to every Möbius probe detuning
(`ResonatorCalibration.correct_probe_detuning`) and to its operating point, so
frequencies come off the sweep itself: the same hold-out test read 0.19%
(90th percentile 0.6%). The tracker, the demodulators, the live viewer and
stream-to-DAC all go through that inversion and need nothing else.

The model still does the other half. A sweep shows only what a change of
frequency looks like; when loss changes too the I/Q point leaves the swept
curve, and the circle geometry is what splits the move into frequency and
loss. (Reading frequency from the angle round a fixed centre instead,
as the `'lookup'` method does, takes a loop-size change for a frequency
change almost one-for-one; during a source scan the loops changed size by
±2.4%.) The loss coordinate keeps the model's scale.

A table is cut at the first sweep point either side where the model runs
backwards (a glitch in the sweep, or the fit breaking down), past its ends
the end corrections hold, and a tone whose one-to-one stretch does not reach
±0.5 linewidths gets none: it keeps the model alone and
`calibration_quality` lists it (`no sweep table`), as it does a fit more than
20% out in frequency at its operating point, where the loss scale is suspect
too (`frequency_scale`, swept over modelled, is reported for every tone).
Records made without the sweep carry no tables and convert as before;
`calibration_record(record, sweep_data=sweep)` adds them. Per tone the record
holds `frequency_correction_hz` at knots `frequency_correction_start_hz +
k · frequency_correction_step_hz` of model probe detuning (NaN without a
table), about 0.6 MB for 380 tones.

## The workflow, stage by stage

| stage | what uses the conversions |
|---|---|
| calibration sweep and fit | `fitting.batch_fit` → `set_resonator_calibrations` / `save_calibrations` |
| modulation set-up | `modulation.params_from_sweep`: centres on `operating_point_hz`, offsets in linewidths |
| tracking (server) | the three-point kernel on the centred basis, with the pushed calibration (the server does no fitting); `detuning_estimator` |
| acquisition | `receive_stream` records the tone updates by default; `parse_stream` applies each revision's readout correction |
| offline analysis | `resonator.convert_samples`; `timeseries` / `noise` (`conversion_method`); `modulation.demodulate`, `demodulate_timestream`, `demodulate_stream` |
| live view | `souk-live-plot` panels are quantity names; `--conversion`, `--calibrations` |
| analog output | `souk-stream-to-dac`: `df_calibrated` (`output`, `method`), `ffm` (the three-point conversion, live) |

## Worked example

Plain NumPy, no hardware:

```python
import numpy as np
from souk_readout_tools import resonator

# fr = 1 GHz, Ql = 20000 (linewidth 50 kHz), no cable delay; the notch below
# (depth Ql/Qc = 0.5) is a circle of centre 0.75 and radius 0.25.
cal = resonator.ResonatorCalibration(1e9, 2e4, 0.0, 0.75 + 0j, 0.25, np.pi, Qi=4e4)
lw = cal.fr / cal.Ql

def s21(f, fr=1e9, Ql=2e4, Qc=4e4):
    return 1 - (Ql / Qc) / (1 + 2j * Ql * (f - fr) / fr)

# A fixed tone on resonance; the resonance moves up 200 Hz.
tone = 1e9
p = resonator.prepare_conversion(tone, 'mobius', calibration=cal)
shift, diss = resonator.convert_iq(p, s21(tone, fr=1e9 + 200))
float(shift), float(diss)                                    # (200.0, 0.0)
float(resonator.express('detuning_linewidths', shift, diss,
                        frequency_hz=tone, calibration=cal))  # 0.004

# Three points, the firmware default [-d, 0, +d, 0], the resonance 3 kHz
# above the modulation centre, in the centred basis:
off = 0.1 * lw * np.array([-1.0, 0.0, 1.0, 0.0])
probe = 1e9 + off
z = resonator.to_basis(s21(probe, fr=1e9 + 3000), cal, probe)[:, None]
d = resonator.phase_derivatives(z, off, per_point=True)
resonator.detuning_from_derivatives(d['dphi_df'], d['d2phi_df2']) * lw
# ~2961 Hz: resonance - centre (finite differences, 1.3% low)

# The same cycle against a reference taken with the resonance at the centre:
z_ref = resonator.to_basis(s21(probe, fr=1e9), cal, probe)[1]
resonator.phase_shift_and_dissipation(z[1, 0], z_ref, d['local_dphi_df'][1, 0])
# ~(3029 Hz, ~0): first order in the 0.06-linewidth excursion
```

## On the RFSoC ARM

Nothing here needs more than NumPy. The basis coefficients are computed once
per tone and revision, so taking a value into the centred basis is one
complex multiply-add; the three-point kernel is a few complex products and
angles per cycle, and the exact gradients three complex divisions; the
`linear` conversion needs no logarithm at all. The tracker's measured costs on
the ARM are in [tone_tracking.md](tone_tracking.md). Fitting is the only
expensive step, and it stays on the client: the server does no fitting, so
tracking and the argument-less `update_modulation()` need a pushed
calibration.
