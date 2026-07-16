# Filterbank channel-response compensation

The PSB (synthesis) and PFB (analysis) filterbanks share one prototype filter.
A tone sitting away from its filterbank bin centre is attenuated by each bank,
and near the channel edges the synthesis Nyquist image interferes with the
main tone at the accumulator. Since v1.6.1 the readout compensates this
automatically wherever mixer control words are prepared — tone setting,
sweeps, and (most importantly) frequency modulation riding the channel
overlap. The model, its hardware validation, and the correction scheme are
recorded here; the implementation lives at the top of
`souk_readout_tools/firmware_lib.py` (the `filterbank_*` functions).

## The response

Offsets below are in **bin spacings** (the `drift_bins` convention: 300 kHz on
the standard configuration; ±0.5 = the half-bin snap boundary, ±1.0 = the
channel edges, where a tone wraps).

- **Prototype (single bank)**: 8-tap sinc × DPSS (NW = 2) window on the
  mlib_devel `pfb_coeff_gen_calc.m` half-sample time axis (confirmed against
  the firmware coefficients — souk-firmware issue #117). Gain `G`: flat to
  ~0.005 dB over |δ| ≤ 0.5, −1 dB at ~0.75, −6 dB at the channel edge.
- **Combined readout**: the synthesis emits aliases at δ−2m (the channelized
  rate is two spacings); the analysis re-admits them and the RX demodulation
  folds them all onto DC, where they sum coherently:

  `H(δ) = Σₘ G(δ−2m)² · exp(4πj·m·B·τ)`

  with `B` the bin spacing and `τ` the analog loopback group delay. At τ = 0
  the sum is real: ~0.01 dB flat over the centre half, **−1 dB at 0.70 bins
  (±211 kHz, 61 kHz past the half-bin edge)**, −6 dB at the edge (the image
  lifts it from the naive −12 dB). A non-zero τ rotates the image term,
  producing the phase turn-over seen near the edges in RF loopback (the
  ordinary delay slope itself is *not* part of `H` — it is plain cable delay,
  measured and removed by the sweep calibration).

**Hardware validation (2026-07-06, single-pipeline KRM setup):** digital
loopback matched |H| to ≤ 0.04 dB across the full channel with phase flat to
0.1 mrad; RF loopback (τ = 150 ns fitted from the passband phase slope)
matched |H| and arg H to 0.05 dB / 2.7 mrad including the edge turn-over
(±0.21 rad peak at 0.78 bins). A 14-tone measurement across 0.6–2.1 GHz
matched the single-tone response to 0.009 dB — the response is
frequency-independent, so one model serves the whole band. The residual
~0.01–0.02 dB passband structure not in the model is at the coefficient
quantization level.

## The correction

Where control words are prepared, per tone (and per point, for modulation):

| part | applied | what |
|------|---------|------|
| TX scaling `base × 1/G(δ_tx)` | **hardware**, per point/slot (and at snap residuals for set-tones/sweeps when amplitudes are in hand) | restores the physical drive on the detector |
| RX magnitude `G(δ_rx)/⏐H⏐` (+ any TX clip shortfall) | **software**: the server reports per-(tone, point) `readout_correction` factors — in the modulation state (applied by `parse_samples` / `group_cycles`) and alongside sweep results (applied by `parse_sweep_data`) | flattens the readout |
| RX phase `−arg H(δ_rx)` (written as `+arg H`, the sign found to cancel on hardware) | **hardware**, per point (only non-zero when a group delay is known; sweeps resolve the same path calibration as modulation) | removes the image phase turn-over |

- The **TX factor restores the physical drive on the detector** — the main
  synthesized tone rolls off with the *single-bank* `G`, even though the
  readout only sags to `|H|` (the image recombination hides half the dB). A
  √-split of the combined response would under-correct the drive 2×.
- The **RX magnitude lives in software because the RX LO scale word is inert
  in the fabric** (found in hardware testing 2026-07-07: the compensated
  response matches a TX-only prediction to ~0.02 dB; raised in
  souk-firmware#117). RX *phase offsets* and frequency words work — only the
  scale multiplier is unconnected. If a future firmware connects it, flip
  `firmware_lib.FILTERBANK_RX_SCALE_IN_FABRIC` to `True`: the hardware RX
  scaling (on the 0.75 headroom base) switches on and the software factors
  collapse to 1, so nothing double-corrects. The software factors are applied
  by `ReadoutClient.parse_samples` (per sample, via the modulation point tag;
  the output carries `readout_correction_applied` and `group_cycles` then
  skips its own application), so plots and demod alike see a flat channel.
  Pass `parse_samples(..., apply_readout_correction=False)` for the raw
  accumulator values — those keep the residual (−0.5 dB at 0.7 bins), as do
  live-stream consumers that never call `parse_samples`/`group_cycles`.
- **Headroom**: the TX boost is +0.5 dB at the 1 dB operating point and +6 dB
  at the (unusable) channel edge. Keep base tone amplitudes ~1 dB (power)
  below full scale — including in power-optimisation results — so modulation
  can restore the drive; a clipped boost is reported in the modulation state
  warnings (client-visible), and the software readout correction absorbs the
  readout side of the shortfall.
- **Beyond ±1.0 bins** a tone has wrapped; no compensation is possible
  (factors are clipped, and arming is refused without `force` as before).

## Configuration

The **group delay for the phase term comes from the standard path-delay
calibration** — the same `rf_frontend.path_group_delay_ns` (scalar or
frequency-dependent calibration file) that resonator fitting and de-embedding
already use, evaluated per tone. The main tone and its image traverse the
full analog path, so their relative phase at recombination is set by the
local dφ/df — exactly what the calibration stores. The normal workflow
therefore needs no compensation-specific setup:

```python
client.measure_path_group_delay(save_to_config=True)   # once per setup
client.push_config()
```

Config keys (firmware defaults):

```yaml
filterbank_compensation: true            # master switch (default true)
# filterbank_group_delay_override_ns: 0  # TESTING/LOOPBACK ONLY: overrides
#                                        # the path calibration when present
```

With no calibration and no override the compensation is amplitude-only —
exact in digital loopback and correct to a few mrad below ~0.6 bins in RF.
On a real setup the path delay is hundreds of ns, putting the *uncorrected*
edge phase term at ~0.5–1 rad, so keep the calibration current. The stored
delay follows the `remove_group_delay` convention (positive τ, correction
`×e^{+2πifτ}`). If the edge phase turn-over **doubles** instead of
cancelling after calibrating, a sign in the phase chain is inverted — this
happened once (2026-07-15): empirically the correction word must carry
`+arg H`, not `−arg H`; the mechanism behind the word→readout sign is not
pinned down.

Per-request A/B switch: `enable_modulation(..., compensate_filterbank=False)`
disables the compensation for one modulation run (it sticks for subsequent
update/recenter calls until the next enable) — this is how the measurement
scripts see the raw response. The applied per-point correction is reported in
the modulation bundle (`bundle['compensation']['response_db']`).
`perform_sweep(..., compensate_filterbank=False)` /
`perform_retune(..., compensate_filterbank=False)` /
`wideband_sweep(..., compensate_filterbank=False)` are the same switch for
sweeps (None follows the config; False also stops the sweep results carrying
`readout_correction`, so `parse_sweep_data` leaves the data raw).

Already-parsed data can be viewed either way: the plotting helpers
(`plot_sweep`, `plot_timestream`, `plot_timestream_psd`,
`plot_timestream_on_resonance`) and `batch_fit` take
`apply_readout_correction` (default `True`) and toggle the software factors
on or off at plot/fit time — parsed sweeps keep the factors alongside the
data, and modulated captures re-derive them from the info snapshot's
modulation state. A `FitResult` inherits the choice made at `batch_fit` time
(the fitted frames bake it in, and deembedding absorbs smooth gain anyway),
so `plot_fits` shows whatever the fit used.

The power optimisers (`maximise_tx_power`, `set_tone_powers(...,
optimise_dynamic_range=True)`, `maximise_tx_dsp_gain`) reserve the headroom
automatically: when the compensation is enabled they cap the strongest
amplitude word `FILTERBANK_TX_HEADROOM_DB` (1 dB) below full scale — the
psb_scale ramp recovers the output level, so no power is lost. An explicit
`set_tone_powers` request that eats into the reserve is honoured but warns.

## Measuring and verifying: `scripts/filterbank/`

Standalone, self-configuring test scripts (they snapshot what they change and
state it up front; loopback and digital gains are always restored, the test
comb only with `--restore`):

- **`01_measure_channel_response.py`** — measures the raw response across a
  full channel with a single test tone, repeated at several frequencies and
  median-combined, with the model overlaid. Default mode switches digital
  loopback on itself; `--rf` expects a loopback cable and fits the path
  group delay from the passband slope (cross-check against the calibration).
- **`02_verify_compensation.py`** — the acceptance test: the same ladder
  measured with compensation off then on, asserting compensated flatness
  (and, in `--rf` mode, the phase correction). Run after firmware/package
  updates.
- **`03_check_modulation_stepping.py`** — MOVED/STATIC verdict per LO-stepping
  path (sw 1-D / sw per-tone / fw slots), for debugging before anything else.

Hard-won gotchas encoded in the scripts, worth knowing if measuring by hand:

- **Internal loopback changes the DSP input level** and can over/underflow
  the pipeline: re-optimise with `maximise_tx_power(digital_only=True,
  rx_policy='maximise')` after switching it, and restore the previous gains
  and loopback state when done.
- **Use one tone at a time.** Many simultaneous bin-centred tones generate
  intermodulation products that can alias onto the measurement, and
  bin-centred ladders lock tone pairs to exactly their armed-bin difference —
  **even** separations alias the neighbour exactly onto DC, so tones 2 bins
  apart poison each other's edge points in proportion to their power
  imbalance. (Real modulation is resonance-centred with small offsets, so
  this exact-DC lock does not occur in operation.)
- A flat-to-noise gain *and* phase "response" means the signal path bypassed
  both filterbanks (or the probes never moved) — check the loopback mode,
  then run `03_check_modulation_stepping.py`.
