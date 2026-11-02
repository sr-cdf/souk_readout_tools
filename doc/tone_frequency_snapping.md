# Tone Frequency Snapping

Every tone is placed on a whole multiple of the accumulation rate, 600 kHz /
acc_len (2500 Hz at acc_len 240), counted from the DUC. The DUC and every
filterbank bin centre already sit on that grid, so each tone's LO offset is a
whole number of accumulation rates and the comb repeats exactly once per
accumulation.

## Why

- **Intermodulation lands in the accumulator nulls.** Every product of the
  comb (DAC or amplifier intermodulation, clipping, ADC nonlinearity) is then
  also a multiple of the accumulation rate. In a tone's channel it sits a whole
  number of accumulation rates away, where the accumulation (a boxcar exactly
  one period long) has a null, so it averages to zero. Only products falling
  exactly on a tone remain, as a constant. On an unsnapped comb the same
  products are a dense forest of lines that reads as a noise floor rising with
  drive.
- **A fixed crest factor.** An unsnapped comb never repeats in practice (its
  exact period, set by the LO word, is 2 h), so its DAC peak keeps growing with
  the time watched: about 4.9 × rms in a minute and 5.3 in an hour on the 6a
  comb. A snapped comb repeats every accumulation, so its peak is a fixed
  number (3.4 × rms with Newman phases on 6a). It can be predicted from the RMS
  and lowered further by optimising the phases.

## What is snapped, and where

| Tones | Snapped in |
|---|---|
| Parked comb: `set_tone_frequencies`, the tone plan at start-up, config push, blind tones, the restore after a sweep or power sweep | `firmware_lib.prepare_tone_frequency_settings_fast` (and the katcp `prepare_tone_frequency_settings`) |
| Modulation centres and offsets, firmware-slot and software engines | the server's modulation setup (`_snap_modulation_comb`) |
| Tracking commits | the same modulation setup; the tracker moves in whole grid steps, and only once the move exceeds 0.6 of a step |
| Sweeps | not snapped: their steps are finer than the grid |

- A tone moves by at most half a grid step: 1250 Hz at 2500 Hz, 0.007
  linewidths median on 6a. Two tones landing on one grid point are moved to the
  nearest free points.
- Modulation offsets are rounded to whole grid steps. A non-zero offset is
  never rounded to zero: it gets one step, with a warning in the server log.
- `get_tone_frequencies()` and the stream info report the frequencies actually
  written. `get_info('tones')` adds `acc_grid_hz` and each tone's `off_grid_hz`.
- Changing acc_len (or the sample rate) moves the grid, and the server
  re-applies the parked comb on the new one. A running modulation keeps the old
  grid until it is armed again.
- The 32-bit LO word steps in 1.4e-4 Hz, and it is exact on a grid of 600 kHz /
  2^k, i.e. a power-of-two acc_len (256, 512, ...). At other acc_len (240) a
  tone sits up to 1e-4 Hz off the grid. That doesn't matter for
  intermodulation (the products move by under a millihertz), but an optimised
  crest factor drifts away over about 10 minutes.

## Turning it off

It is on by default. In the board config:

```yaml
firmware:
  defaults:
    snap_tones_to_acc_grid: true   # false: write tones exactly as given
```

Push the config to change it. For a single call, e.g. an unsnapped A/B
comparison, `client.set_tone_frequencies(freqs, snap=False)` (or `True`)
overrides the switch; the `firmware_lib` tone functions take the same `snap=`.
