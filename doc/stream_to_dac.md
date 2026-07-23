# Stream to DAC: live analog output of KID response

`souk-stream-to-dac` turns the readout server's continuous TCP stream into a
live analog voltage proportional to a chosen KID's response — magnitude,
phase, or resonant-frequency shift — on a USB DAC. The first use case is
spectral characterisation with a TOPTICA cw-THz source: the TOPTICA control
software records an analog input against its own scan-frequency axis, and we
feed it the detector response in real time.

```
RFSoC readout server ──TCP stream──▶ souk-stream-to-dac (client machine)
                                          │
                       IQ → df conversion (resonator calibration)
                                          │
                              USB DAC (LabJack T-series)
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
  current value, volts, and drop counts (`--quiet` disables).

## Units: raw stream IQ vs calibration data

Raw stream int32 IQ is **not** in the same basis as the parsed data used to
build calibrations: `parse_samples(..., apply_readout_correction=True)`
multiplies modulated captures by the per-(point, tone) software
readout-flattening factors (the filterbank compensation's RX half, which
cannot land in hardware — see `doc/filterbank_compensation.md`). The stream
path applies exactly the same factors (`stream_dac.ReadoutCorrection`,
selected per sample via the flag5 point tag) before any conversion, so
converted values match the calibration's units. For unmodulated streams the
correction is the identity. `tests/test_stream_dac.py` proves byte-for-byte
agreement between the stream path and `parse_samples` on identical raw data.

## Conversion modes

| mode | output | needs |
|---|---|---|
| `magnitude` | \|IQ\| (counts) | nothing — zero-calibration first light |
| `phase` | phase re: reference IQ (rad) | reference (startup capture or config) |
| `df_modelfree` | phase / `dphi_df` (Hz) | reference + `conversion.dphi_df` (rad/Hz) |
| `df_calibrated` | Möbius probe detuning (Hz) or dissipation | `conversion.calibration_file` |
| `ffm` | absolute resonator shift (Hz), self-calibrating | frequency modulation running |

`df_calibrated` uses `ResonatorCalibration.tone_converter(f_tone)` — the
exact Möbius inversion, one complex multiply + add per sample — and can
output the matched dissipation quadrature instead
(`conversion.output: dissipation`).

### `ffm` mode: live demodulated output

With frequency modulation running (`enable_modulation` + `enable_stream`,
either engine — fw slot tags and sw point tags ride the same flag5 field),
`FfmConverter` groups the stream into modulation cycles (settling excluded,
incomplete cycles dropped) and per cycle:

- the phase-vs-offset slope gives a **live `dφ/df`** — the modulation
  measures its own responsivity every cycle, so no prior calibration file
  is needed and slow responsivity drift is tracked automatically;
- the centre-point phase, referenced to a startup-captured baseline
  (`conversion.baseline_cycles`, default 20 cycles) and divided by that
  slope, gives the linearised frequency shift.

The linearisation is valid while the tone sits in the linear part of the
phase-frequency curve — which is exactly what server-side tone tracking
(`enable_tracking`, `doc/tone_tracking.md`) maintains. Run them together.

**Recentering provenance**: a tracking (or client) recenter moves the tone
centre and bumps the stream revision tag. The script subscribes to the
typed tone-update frames automatically in ffm mode, keys the per-revision
centres, and outputs the **absolute** resonator shift since startup:
`x = (center[rev] − center[rev0]) − probe_side_shift` (detector-side sign:
positive = resonance moved up). Recenters therefore do not step the analog
output, and the same updates land in the `.updates.jsonl` sidecar next to
the raw recording for offline reconstruction. The startup phase baseline
absorbs the `center − fr` operating-point offset documented in
`modulation.params_from_sweep`. `conversion.average_cycles` boxcars the
last N per-cycle outputs before the DAC mapping.

The converter interface (`stream_dac.StreamConverter`) keeps all of this
out of the I/O threads: `ffm` is just a converter with
`wants_frames = True` that consumes tagged samples instead of a boxcar
mean.

### Calibration workflow (sweep → fit → file)

```python
from souk_readout_tools import stream_dac

# 1. Take + parse a sweep as usual (client.perform_sweep / get_sweep_data).
sweep = client.parse_sweep_data(client.get_sweep_data())

# 2. Fit all tones and build per-tone calibrations (fitting.batch_fit
#    under the hood; kwargs are forwarded to it).
cals = stream_dac.calibrations_from_sweep(sweep)

# 3. Save for the config's conversion.calibration_file.
stream_dac.save_calibrations('my_array_cal.npz', cals)
```

The `.npz` stores the `ResonatorCalibration` constructor arrays per tone
(fr, Ql, tau, circle centre/radius/rotation, gain, Duffing `anl`, …) keyed by
`tone_indices` — plain numpy, no pickle. `stream_dac.load_calibrations`
restores them.

## df → volts mapping

Per output channel:

```
volts = clip(gain * (x − x_offset) + v_offset, v_min, v_max)
```

`DAC0` plus an optional `DAC1` for a second tone; an LJTick-DAC (±10 V) is
just a channel name (`TDAC0`) with wider `v_min`/`v_max`. Match
`v_min`/`v_max` to the TOPTICA input range once known.

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
- `dummy` — no hardware; remembers/prints writes. Default fallback when the
  selected LabJack driver is not installed, and what all development and
  tests use.

Optional AIN sampling (`dac.ain_channels`) reads listed analog inputs once
per output update and logs them with host time plus the `tt` of the nearest
stream frame — reserved for a future TOPTICA frequency-monitor or
step-trigger line. Off by default.

## Latency budget and measurement

Contributions per update, all bounded:

| stage | size | knob |
|---|---|---|
| accumulator sample period | 1/sample_rate | server config |
| boxcar window | N/sample_rate = 1/update_rate | `dac.update_rate_hz` |
| queue staleness (worst case) | queue_depth/sample_rate | `stream.queue_depth` |
| USB DAC write | ≈1 ms (T-series) / ≈16 ms (U12) | backend/rate |

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
| ±10 V needed (LJTick-DAC) | `channels[].dac_channel` (`TDAC0`) + `v_min`/`v_max` |
| Toptica frequency-monitor / trigger line | `dac.ain_channels` (+ `recording.ain_log`) |
| which KID(s) | `channels[].tone_index` or `tone_frequency_hz` |
| acceptable stall before safe state | `stream.watchdog_s` |
| safe/idle output level | `dac.idle_voltage` |

## Recipe: analog output session, bench to beam

A full session, assuming a working readout config (`my_readout.yaml`) and a
LabJack on the client machine. Steps 1–3 are one-time preparation; 4–7 are
the observing loop.

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
and fit a sweep and build the calibration file (only needed for
`df_calibrated`; skip for `magnitude`/`phase`/`ffm`):

```python
from souk_readout_tools import stream_dac
sweep = client.parse_sweep_data(client.get_sweep_data())
cals = stream_dac.calibrations_from_sweep(sweep)
stream_dac.save_calibrations('array_cal.npz', cals)   # -> conversion.calibration_file
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

- Fixed tone: `--mode df_calibrated` (with the calibration file) or
  `--mode phase` / `df_modelfree` for quick looks. The df→V mapping
  (`gain`, `v_offset`, clip range) sets the scale on the TOPTICA axis:
  e.g. `gain: 2e-4` puts ±10 kHz of detuning across ±2 V.
- Modulated (recommended for long scans): arm FFM + tracking on the
  server first (see the recipe in `doc/tone_tracking.md`), then
  `--mode ffm`. The output is then the absolute resonator shift,
  self-calibrating and immune to recenters.

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
decoding, the stream-path-vs-`parse_samples` units test, converter round
trips through a `ResonatorCalibration`, calibration file round trip, a live
socket test against an in-process server, `ReadoutClient(mock=True)` frames,
and a replay → dummy-DAC end-to-end run (drop counting, safe state,
re-recorded bytes identical). Frame sources are interchangeable:
`SocketFrameSource` (live), `MockFrameSource` (emulated server),
`ReplayFrameSource` (recorded file at true rate).
