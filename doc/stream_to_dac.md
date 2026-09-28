# Stream to DAC: live analog output of KID response

`souk-stream-to-dac` turns the readout server's continuous TCP stream into a
live analog voltage proportional to a chosen KID's response — magnitude,
phase, or any resonance quantity of the package's shared vocabulary
(resonance shift, dissipation, detuning, …; [conversions.md](conversions.md))
— on a USB DAC. The first use case is
spectral characterisation with a TOPTICA cw-THz source: the TOPTICA control
software records an analog input against its own scan-frequency axis, and we
feed it the detector response in real time.

```
RFSoC readout server ──TCP stream──▶ souk-stream-to-dac (client machine)
                                          │
                       IQ → df conversion (resonator calibration)
                                          │
                     USB DAC (LabJack) or sound card (ALSA)
                                          │
                              TOPTICA analog input
```

The RFSoC's own USB port is inaccessible, so this runs client-side. Latency
is tens of milliseconds, bounded and measurable (see
[Latency](#latency-budget-and-measurement)).

Implementation: `souk_readout_tools/stream_dac.py` (frame sources,
converters, DAC backends, the reader/output application) and the thin CLI
wrapper `client/client_scripts/stream_to_dac.py`. Config template:
`souk_readout_tools/data/config/template_stream_to_dac_config.yaml`.

## Quick start

```bash
# No hardware at all: emulated readout + dummy DAC
souk-stream-to-dac --mock --mode magnitude --rate 100

# Live server, dummy DAC (prints voltages)
souk-stream-to-dac -a 10.11.11.11 -r 10000 --config my_stream_to_dac.yaml

# Replay a previous recording through the full pipeline
souk-stream-to-dac --replay ./tmp/stream_to_dac --config my_stream_to_dac.yaml
```

Connection arguments (`-C` / `-a` / `-r`) mirror `souk-receive-stream`.
Script settings live in a separate YAML file (`--config`); the readout
config schema is untouched. Common knobs have CLI overrides: `--tone`,
`--mode`, `--rate`, `--backend`, `--quiet`.

## Architecture

Two threads, producer/consumer, matching the package's synchronous style:

- **Reader thread** — socket recv loop using the same framing as
  `ReadoutClient.receive_stream` (4-byte big-endian length prefix, payload of
  interleaved `<i4` I,Q pairs for the active tones in user order, then 10
  trailer words: flags 0–5, `tt_msb`, `tt_lsb`, `cnt`, `err`). Every raw
  frame is appended to disk in the same file + JSON-sidecar format as
  `receive_stream`, so all existing tooling can read the recording (and
  `--replay` can play it back). Parsed frames go onto a bounded drop-oldest
  queue, so a slow consumer can never build unbounded latency.
- **Output thread** — at the configured DAC update rate: drain the queue,
  apply the readout correction, boxcar-average the most recent
  `N = round(sample_rate / update_rate)` samples, convert each channel's IQ
  to a scalar, map it to volts, write the DAC. Never blocks on disk I/O.
- **Safe state** — on any exception, stream stall (watchdog), or exit, every
  output is driven to the configured `idle_voltage`, the reason is logged,
  and the process exits cleanly. SIGINT/SIGTERM are handled as in
  `receive_stream.py`.
- **Diagnostics** — dropped wire frames (`cnt` gaps), queue drops, and
  output-loop timing are counted and reported; a ~1 Hz status line shows the
  current value, volts, the channel's calibration fit (`fit=0.98`, marked
  `!` outside 0.85–1.15: time to re-sweep and push; `ffm` with a
  calibration) and drop counts (`--quiet` disables).

## Units: raw stream IQ vs calibration data

Raw stream int32 IQ is **not** in the same basis as the parsed data used to
build calibrations: `parse_samples(..., apply_readout_correction=True)`
multiplies modulated captures by the per-(point, tone) software
readout-flattening factors (the filterbank compensation's RX half, which
cannot land in hardware — see `doc/filterbank_compensation.md`). The stream
path applies exactly the same factors (`stream_dac.ReadoutCorrection`,
selected per sample via the flag5 point tag, and per revision once a
tracking commit or recentre is announced: the probes then sit at new
positions across the filterbank) before any conversion, so converted values
match the calibration's units. For unmodulated streams the
correction is the identity. `tests/test_stream_dac.py` proves byte-for-byte
agreement between the stream path and `parse_samples` on identical raw data.

## Conversion modes

| mode | output | needs |
|---|---|---|
| `magnitude` | \|IQ\| (counts) | nothing — zero-calibration first light |
| `phase` | phase re: reference IQ (rad) | reference (startup capture or config) |
| `df_modelfree` | −phase / `dphi_df`: the resonance shift (Hz) | reference + `conversion.dphi_df` (rad/Hz, the raw phase slope) |
| `df_calibrated` | `conversion.output` by `conversion.method` | a calibration: `conversion.calibration_file`, or one pushed to the server |
| `ffm` | `conversion.output`, recentre-proof, self-calibrating | frequency modulation running; a calibration optional |

`conversion.output` is a quantity of `resonator.QUANTITIES` (aliases
accepted): `resonance_shift_hz` (default; positive = resonance moved up),
`resonance_frequency_hz`, `detuning_hz` (resonance − tone),
`matched_dissipation_shift` (Δ(1/(2·Qi))), the `*_linewidths` and
fractional variants; `ffm` adds its per-cycle `center_detuning_hz`,
`center_detuning_linewidths` and `dphi_df`. Names that need a calibration
are refused at startup without one. The older `frequency` and `dissipation`
still work: they are `resonance_shift_hz` and `matched_dissipation_shift`.
**Signs changed** with the shared vocabulary: `df_calibrated`'s
`frequency` used to be the probe detuning `f_tone − fr`, which is
`−detuning_hz`, and `df_modelfree` used to output `+phase / dphi_df`.

`df_calibrated` is `stream_dac.QuantityConverter`:
`resonator.prepare_conversion` once, then `convert_iq` and `express` per
update — the functions offline analysis uses, so the DAC reads what an
analysis of the recording would. `conversion.method` is `mobius` (default:
the exact fitted inversion, one complex multiply-add plus a few flops per
update), `phase`, `linearized`, `lookup` or `circle`; the last three need the
calibration file to hold its sweep (`save_calibrations(...,
sweep_data=sweep)`). The shift is relative to the sweep's state when the
file has the sweep, else to the fitted state. The calibration comes from
`conversion.calibration_file`, else from the stream's `info['resonators']`
— the calibration pushed with `client.set_resonator_calibrations`, which
the server serves while its tones are still where it was made. An RX gain
change since its sweep is divided out automatically.

### `ffm` mode: live demodulated output

With frequency modulation running (`enable_modulation` + `enable_stream`,
either engine — fw slot tags and sw point tags ride the same flag5 field),
`FfmConverter` groups the stream into modulation cycles (settling excluded,
incomplete cycles dropped) and runs each through the kernel the offline
demodulation and the server's tracker use (`resonator.phase_derivatives`,
any slot order):

- each cycle measures a **live `dφ/df`** and the phase curvature — the
  modulation measures its own responsivity, so slow responsivity drift is
  tracked automatically;
- the centre point against a reference (its mean over the first
  `conversion.baseline_cycles`, default 20) gives the resonance shift and
  dissipation. With a calibration (the file, or the one pushed to the
  server) this happens in its circle-centred basis — the centred phase and
  log-radius over the live slope, the preferred conversion, with no
  dissipation leaking into frequency; without one, on the raw I/Q by the
  tangent/normal projection on the live gradient.

The conversion is first order in the excursion from the reference, which is
what server-side tone tracking (`enable_tracking`, `doc/tone_tracking.md`)
keeps small. Run them together.

**Recentres**: a tracking (or client) recentre moves the tone centre and
bumps the stream revision tag. The script subscribes to the typed
tone-update frames automatically in ffm mode, registers each revision's
centre, offsets and readout correction, and outputs the resonator's own
motion by the reconstruction identity:
`x = (center[rev] − center[ref]) + shift about the reference` (positive =
resonance moved up). Recentres therefore do not step the analog output;
cycles of a revision whose announcement has not yet arrived are held back
rather than read with the old state. In the calibrated basis the join is
exact to first order; on raw I/Q the cable delay adds about
`2π τ / |dφ/df|` hertz per hertz of recentre (well under 1% for typical
delays). The same updates land in the `.updates.jsonl` sidecar next to the
raw recording (with `sample_index`, as `receive_stream` writes them), so
`parse_stream` and `modulation.demodulate_stream` reconstruct the recording
offline — `tests/test_stream_dac.py` checks the DAC against that offline
analysis of its own recording. `conversion.average_cycles` boxcars the last N
per-cycle outputs before the DAC mapping; `conversion.demod_method`
(`fast`, or `accurate` for a quadratic fit) selects the estimator.

The converter interface (`stream_dac.StreamConverter`) keeps all of this
out of the I/O threads: `ffm` is just a converter with
`wants_frames = True` that consumes tagged samples instead of a boxcar
mean.

### Calibration workflow (sweep → fit → file)

```python
from souk_readout_tools import resonator

# 1. Take + parse a sweep as usual (client.perform_sweep / get_sweep_data).
sweep = client.parse_sweep_data(client.get_sweep_data())

# 2. Fit all tones and build per-tone calibrations (fitting.batch_fit
#    under the hood; kwargs are forwarded to it).
cals = resonator.calibrations_from_sweep(sweep)

# 3a. Push them to the server: every stream then carries them
#     (info['resonators']), and df_calibrated / ffm need no file ...
client.set_resonator_calibrations(cals, sweep_data=sweep)

# 3b. ... or save them (with the sweep) for conversion.calibration_file.
resonator.save_calibrations('my_array_cal.npz', cals, sweep_data=sweep)
```

The `.npz` holds the calibration record's per-tone arrays (fr, Ql, Qi,
cable delay, circle centre/radius/rotation, gain, Duffing `anl`, the
operating point, …) keyed by `tone_indices`, its provenance, and the sweep
when given — plain numpy, no pickle. `resonator.load_calibrations` restores
them (older `stream_dac.save_calibrations` files load too; the `stream_dac`
names still work).

## df → volts mapping

Per output channel:

```
volts = clip(gain * (x − x_offset) + v_offset, v_min, v_max)
```

`DAC0` plus an optional `DAC1` for a second tone; an LJTick-DAC (±10 V) is
just a channel name (`TDAC0`) with wider `v_min`/`v_max`. Match
`v_min`/`v_max` to the TOPTICA input range once known.

**Drift and clipping: `channels[].highpass_s`.** In `ffm` mode `x` is the
cumulative resonance shift (tracking recentres are added back), so slow drift
walks it into `v_min`/`v_max`. The clip is digital, before any coupling
capacitor, so AC coupling does not help: the output flat-tops, then the
plateau decays to 0 V. Setting `highpass_s` (seconds) replaces `x_offset` with
a running baseline, an exponential moving average of `x` with that time
constant, so the mapping becomes

```
volts = clip(gain * (x − baseline) + v_offset, v_min, v_max)
```

This is a single-pole high-pass with corner `1/(2π·highpass_s)`, started at
the first value. A steady drift rate still leaves a standing
`gain · rate · highpass_s`. On the `alsa` backend, choose `highpass_s` at or
below the measured coupling time constant: nothing slower survives the
capacitor anyway. On a DC-coupled DAC it is a real trade: features slower than
`highpass_s` are attenuated. The status line's `x` is always the unfiltered
value.

## DAC backends

- `labjack` — LabJack T4/T7 via `labjack-ljm` (guarded import; optional
  dependency). USB command-response writes cost ≈1 ms each, so keep
  `update_rate_hz` in the low hundreds of Hz (default 200). Channels `DAC0`/
  `DAC1` (0–5 V) or `TDAC0`.. (LJTick-DAC, ±10 V).
- `labjack_u12` — LabJack U12 via `LabJackPython` + the Exodriver
  (`liblabjackusb`; the U12 predates LJM, so it uses this separate driver
  stack — Linux/macOS). Outputs `AO0`/`AO1` (0–5 V only; `DAC0`/`DAC1`
  aliased), inputs `AI0`..`AI7` (`AIN0`.. aliased). Driver options in
  `dac.labjack` are `id` (default −1 = first device) and `serial_number`.
  Measured on hardware: each USB transaction is ≈16 ms (≈62 writes/s), and an
  AIN read costs another ≈16 ms, so a tick that also samples `ain_channels`
  is ≈32 ms (≈31/s). Set `update_rate_hz` with headroom below those ceilings —
  **≤50 Hz output-only, ≤25 Hz with AIN monitoring**. The filtered-PWM outs
  also settle slowly: expect ~0.1 V of ripple/settling lag at these rates
  (loopback error ≈0.11 V rms), so add RC filtering on the AO line if the
  TOPTICA needs a cleaner control voltage.
- `alsa` — a sound card via `pyalsaaudio`. **Linux only**: ALSA is the Linux
  sound API, so on Windows and macOS the guarded import fails and the backend
  falls back to `dummy` with a warning (the LabJack backends are the portable
  option there). Install with `sudo apt install libasound2-dev` then
  `pip install pyalsaaudio`. Cheap, fast and multichannel, and the
  fallback when no LabJack is available — but line outputs are **AC-coupled**,
  which changes what the output means. Read
  [Sound card as DAC](#sound-card-as-dac-ac-coupling) before using it.
  Options live in `dac.alsa` (`device`, `rate`, `channels`, `period_size`,
  `full_scale_volts`); channels `DAC0`/`DAC1` map to the first two audio
  channels, so a LabJack config ports over unchanged.
- `dummy` — no hardware; remembers/prints writes. Default fallback when the
  selected LabJack or ALSA driver is not installed, and what all development
  and tests use.

Optional AIN sampling (`dac.ain_channels`) reads listed analog inputs once
per output update and logs them with host time plus the `tt` of the nearest
stream frame — reserved for a future TOPTICA frequency-monitor or
step-trigger line. Off by default, and always zeros on the `alsa` backend
(there is no live capture path; the selftest opens capture separately).

## Sound card as DAC: AC coupling

A sound card is an excellent DAC in every respect but one: its line output is
capacitively coupled, so it **cannot emit a static level**. DC is blocked
outright and everything below the coupling corner `f_c` is high-passed — a
step decays back to zero with time constant `tau = 1/(2 pi f_c)`. The
stream-to-dac output is a slowly-varying, near-DC control voltage, so this is
not a detail: whether the backend is usable depends entirely on where `f_c`
sits relative to the timescale of the features in your scan.

`idle_voltage` is therefore physically 0 V on this backend whatever it is set
to (the safe-state write still happens, it just decays away), and per-channel
`v_offset` does nothing.

**Measure `f_c` before trusting a slow scan.** `souk-stream-to-dac --backend
alsa --selftest` runs three stages (about 90 s in total):

1. lists the ALSA cards and playback PCMs, so you can pick `dac.alsa.device`
   (`--list-audio-devices` alone does just this). Prefer an explicit `hw:`
   device — it bypasses PipeWire/PulseAudio volume and resampling, which would
   otherwise sit inside your voltage calibration;
2. plays a 1 kHz sine at half of digital full scale. Read it with a DMM in AC
   volts and set `dac.alsa.full_scale_volts = sqrt(2) * V_rms / 0.5`. The
   mixer gain is inside that number — set the mixer once, then leave it alone
   (and check it is not muted);
3. plays a 0.1 Hz square wave, then a sine ladder from 0.2 Hz to 100 Hz. Scope
   the output: the square wave's exponential droop gives `tau` directly, and
   the ladder gives the −3 dB point. Passing a capture device measures the
   ladder automatically, but line-**in** is AC-coupled too, so a loopback
   corner is only an upper bound on the output's own.

If the measured corner is too high for your scan, in order of preference:

- **chop the source.** If the THz source is chopped (or the TOPTICA detects at
  a modulation frequency), the information is already AC well above `f_c` and
  AC coupling costs nothing.
- **use a DC-coupled interface.** Expert Sleepers ES-8/ES-9, MOTU and similar
  interfaces have DC-coupled outputs designed for control voltages. The same
  `alsa` backend drives them with no code change — only `full_scale_volts` and
  the device name differ.
- **software pre-emphasis** (`v + (1/tau) * integral(v)`) inverts the measured
  single pole exactly. Small to add, but the integrator has finite headroom, so
  it buys a bounded scan length rather than true DC. Not implemented — measure
  first, then decide whether it is worth it.

## Latency budget and measurement

Contributions per update, all bounded:

| stage | size | knob |
|---|---|---|
| accumulator sample period | 1/sample_rate | server config |
| boxcar window | N/sample_rate = 1/update_rate | `dac.update_rate_hz` |
| queue staleness (worst case) | queue_depth/sample_rate | `stream.queue_depth` |
| USB DAC write | ≈1 ms (T-series) / ≈16 ms (U12) | backend/rate |
| ALSA output buffer | buffer_size/rate (≈21 ms at the defaults) | `dac.alsa.period_size` |

**Bench check** — `souk-stream-to-dac --selftest` (real LabJack, wire DAC0 →
AIN0): toggles the output 100 times and reports the write+read transaction
time and loopback error.

**End-to-end** — record the TOPTICA-side analog log while stepping a known
input (e.g. `update_modulation` a tone by a known offset, or chop the
source), and cross-correlate the recorded voltage against the raw recording's
`tt` timeline (the recording carries every frame's PTP timestamp; the AIN log
carries host time + nearest `tt` per update). The lag of the correlation peak
is the end-to-end latency.

## Unknown experimental parameters → config keys

Parameters not yet pinned down are config options, not assumptions:

| unknown | config key |
|---|---|
| TOPTICA analog input range | `channels[].v_min` / `v_max` (+ `gain`, `v_offset`) |
| TOPTICA logging rate | `dac.update_rate_hz` |
| chop / lock-in scheme | `conversion.mode` + `channels[].gain` (chop ref via `dac.ain_channels` later) |
| exact LabJack model | `dac.labjack.device_type` / `connection_type` / `identifier` |
| which sound card / ALSA device | `dac.alsa.device` (`--list-audio-devices`) |
| sound-card output volts at full scale | `dac.alsa.full_scale_volts` (`--selftest`) |
| sound-card coupling corner | measured with `--selftest`; see [Sound card as DAC](#sound-card-as-dac-ac-coupling) |
| ±10 V needed (LJTick-DAC) | `channels[].dac_channel` (`TDAC0`) + `v_min`/`v_max` |
| Toptica frequency-monitor / trigger line | `dac.ain_channels` (+ `recording.ain_log`) |
| which KID(s) | `channels[].tone_index` or `tone_frequency_hz` |
| acceptable stall before safe state | `stream.watchdog_s` |
| safe/idle output level | `dac.idle_voltage` |

## Recipe: analog output session, bench to beam

A full session, assuming a working readout config (`my_readout.yaml`) and a
LabJack on the client machine. Steps 1–3 are one-time preparation; 4–7 are
the observing loop. On the `alsa` backend the same recipe applies, but step 1
is the card characterisation instead of the loopback selftest — and read
[Sound card as DAC](#sound-card-as-dac-ac-coupling) first, because an
AC-coupled output changes what a slow scan looks like.

**1. Prepare the DAC + config (no readout needed).**

```bash
cp .../template_stream_to_dac_config.yaml toptica_dac.yaml
# edit: channels[0].tone_index / tone_frequency_hz, dac.backend: labjack,
#       v_min/v_max to the TOPTICA input range, dac.idle_voltage
souk-stream-to-dac --config toptica_dac.yaml --selftest   # wire DAC0->AIN0
```

The selftest reports the USB write+read transaction time (~1 ms class) and
loopback error; pick `dac.update_rate_hz` accordingly (default 200 Hz).

**2. Tune up the array as usual** (tones set, powers optimised), then take
and fit a sweep and push the calibrations (needed for `df_calibrated`;
gives `ffm` its centred basis; skip for `magnitude`/`phase`):

```python
from souk_readout_tools import resonator
sweep = client.parse_sweep_data(client.get_sweep_data())
cals = resonator.calibrations_from_sweep(sweep)
client.set_resonator_calibrations(cals, sweep_data=sweep)   # in every stream's info
resonator.save_calibrations('array_cal.npz', cals, sweep_data=sweep)  # or a file
```

**3. Zero-calibration first light.** Start streaming
(`client.enable_stream()`), then:

```bash
souk-stream-to-dac -C my_readout.yaml --config toptica_dac.yaml \
    --mode magnitude --backend dummy
```

Watch the status line respond to the source (chop by hand). Move to
`--backend labjack` and check the voltage arrives at the TOPTICA input.

**4. Choose the science mode.**

- Fixed tone: `--mode df_calibrated` (with the pushed calibration or the
  file) or `--mode phase` / `df_modelfree` for quick looks. The df→V mapping
  (`gain`, `v_offset`, clip range) sets the scale on the TOPTICA axis:
  e.g. `gain: 2e-4` puts ±10 kHz of detuning across ±2 V.
- Modulated (recommended for long scans): arm FFM + tracking on the
  server first (see the recipe in `doc/tone_tracking.md`), then
  `--mode ffm`. The output is then the resonator's own shift,
  self-calibrating and immune to recentres.

**5. Measure end-to-end latency once per setup.** Step a known input —
`client.update_modulation(center=...)` by a known offset, or chop the
source — and cross-correlate the TOPTICA-side log against the raw
recording's `tt` timeline (every frame carries its PTP timestamp; the AIN
log ties host time to the nearest `tt`). The correlation-peak lag is the
number to quote; budget contributions are in the table above.

**6. Observe.** The TOPTICA software records the analog input against its
scan axis; this side records, always: the raw stream (replayable), the
`.updates.jsonl` tone provenance (ffm mode), and the AIN log if a monitor
line is wired. On any stall/error the DAC parks at `idle_voltage`.

**7. Verify offline.** Replay the recording through the identical
pipeline (`--replay ./tmp/stream_to_dac --backend dummy`) or parse it with
the standard tooling (`parse_samples` reads the same format) and compare
against the TOPTICA-side trace.

## Testing without hardware

`tests/test_stream_dac.py` covers: the frame assembler against synthetic
byte streams (partial reads, keepalives, typed-frame top byte), trailer/flag5
decoding, the stream-path-vs-`parse_samples` units test, per-revision
readout correction, converter round trips through a `ResonatorCalibration`,
the calibrated ffm path (any slot order, loss vs frequency, recentres, held
revisions), the DAC against the offline analysis of its own recording and
the replay of its updates, calibration file round trip, a live socket test
against an in-process server, `ReadoutClient(mock=True)` frames, and a
replay → dummy-DAC end-to-end run (drop counting, safe state, re-recorded
bytes identical). Frame sources are interchangeable:
`SocketFrameSource` (live), `MockFrameSource` (emulated server),
`ReplayFrameSource` (recorded file at true rate).
