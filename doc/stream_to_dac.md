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
| `ffm` | — | **not implemented** (stub) |

`df_calibrated` uses `ResonatorCalibration.tone_converter(f_tone)` — the
exact Möbius inversion, one complex multiply + add per sample — and can
output the matched dissipation quadrature instead
(`conversion.output: dissipation`).

The converter sits behind a small interface (`stream_dac.StreamConverter`),
so a future FFM/`demodulate`-based mode drops in without touching the I/O
threads. Whoever implements it must subtract the documented `center − fr`
operating-point baseline (see `doc/frequency_modulation.md` and
`modulation.params_from_sweep`).

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
  `update_rate_hz` in the low hundreds of Hz (default 200).
- `dummy` — no hardware; remembers/prints writes. Default fallback when LJM
  is not installed, and what all development and tests use.

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
| USB DAC write | ≈1 ms | backend/rate |

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
