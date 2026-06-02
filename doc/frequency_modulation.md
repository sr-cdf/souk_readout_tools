# Fast Frequency Modulation

Fast frequency modulation rapidly dithers each readout tone across a small set
of probe frequencies (typically 2 or 3 points) and streams the result back as
ordinary data, tagged so you can tell which probe point every sample belongs
to. It lets you measure each resonator's local response **continuously, in real
time**, instead of relying on a single calibration sweep taken at the start of
an observation.

This document starts with a practical user guide to the main tools, then goes
into the concepts and internals further down.

---

## Contents

- [Why use it](#why-use-it)
- [Quick start](#quick-start)
- [The main tools](#the-main-tools)
  - [Set it up from a sweep](#1-set-it-up-from-a-sweep)
  - [Arm, stream, and capture](#2-arm-stream-and-capture)
  - [Demodulate](#3-demodulate)
  - [Track: update the centres live](#4-track-update-the-centres-live)
  - [Inspect the state](#5-inspect-the-state)
- [Key concepts](#key-concepts)
- [The data frame tag](#the-data-frame-tag)
- [API reference](#api-reference)
- [How it works](#how-it-works)
- [Sensitivity](#sensitivity)
- [Testing without hardware](#testing-without-hardware)
- [Status and limitations](#status-and-limitations)

---

## Why use it

A resonator's transmitted phase `φ(f)` sweeps steeply through resonance. Two
things follow from measuring that phase curve locally and continuously:

1. **Live calibration (dφ/df).** Probing each resonator at two or more nearby
   frequencies measures the local phase slope `dφ/df` *as the observation runs*.
   You then convert the streamed I/Q into a **frequency shift** (the detector
   signal) against a slope that tracks in real time — rather than a one-time
   start-of-observation sweep that goes stale as the resonators drift. With a
   resonance-circle calibration the same data also yields **dissipation**.

2. **Tracking the operating point.** With three points you can estimate the
   curvature `d²φ/df²` and find the **steepest part of the phase curve — the
   inflection point** (the most responsive operating point, near resonance
   `f₀`). A slow tracking loop can then steer each tone's centre to follow the
   resonator as it drifts, **without dropping any data**.

Because the modulation cycles far faster than the ~10 Hz science band (e.g. 3
points at ~1 kSa/s ≈ 300 cycles/s), it also behaves like a lock-in that rejects
low-frequency amplifier drift.

---

## Quick start

```python
from souk_readout_tools.client.readout_client import ReadoutClient
from souk_readout_tools import demod

client = ReadoutClient()                     # or ReadoutClient(mock=True) to try it offline

# 1. Derive modulation parameters from a sweep you already took.
sweep = client.get_sweep_data()              # {'f': (npts, ntones), 'z': (npts, ntones), ...}
cfg = demod.modulation_params_from_sweep(sweep, n_points=3, samples_per_point=4)
print(cfg['summary'])                        # review the per-tone centres and deltas

# 2. Arm modulation and start streaming/capturing.
client.enable_modulation(center=cfg['center'], offsets=cfg['offsets'],
                         mod_indices=cfg['mod_indices'],
                         samples_per_point=cfg['samples_per_point'],
                         n_settle=cfg['n_settle'])
raw = client.get_samples(3000)               # returns modulated, tagged frames
data = client.parse_samples(raw)

# 3. Demodulate to per-cycle quantities.
state = client.get_modulation_state()
grouped = demod.group_modulation_cycles(data, state)
result = demod.demodulate(grouped, linewidth_hz=cfg['linewidth_hz'])
print(result['freq_shift_hz'].shape)         # (n_cycles, n_tones)

# 4. (Optional) re-centre a tone that has drifted, with no dropped data.
client.update_modulation(center=new_centres)

# 5. Stop modulating; tones rest at their centres.
client.disable_modulation()
```

---

## The main tools

### 1. Set it up from a sweep

`demod.modulation_params_from_sweep(sweep, ...)` turns a calibration sweep into
a ready-to-use configuration. It fits each resonator for its **centre** (the
steepest/inflection point) and its **linewidth**, then builds a symmetric probe
pattern with **each tone's probe spacing scaled to its own linewidth** — narrow
resonators automatically get a smaller delta.

```python
cfg = demod.modulation_params_from_sweep(
    sweep,                 # dict with 'f' and 'z' arrays, shape (n_sweep_points, n_tones)
    n_points=3,            # 2 = slope only; 3 = slope + curvature/tracking
    samples_per_point=4,   # dwell: samples per point per cycle
    n_settle=1,            # leading samples per point flagged as 'settling'
    delta_linewidths=0.25, # probe half-span in units of each resonator's linewidth
)
# cfg = {'center', 'offsets', 'mod_indices', 'samples_per_point', 'n_settle',
#        'linewidth_hz', 'summary'}
```

The returned dict splats straight into `enable_modulation(**cfg-subset)`. The
`linewidth_hz` it returns is what the demod tool needs later to express detuning
in linewidths. You can of course build `center`/`offsets` by hand instead.

### 2. Arm, stream, and capture

Modulation is a **mode of the normal data stream**, controlled by an on/off
switch that is independent of whether the stream is running:

```python
# Arm only (does not start output):
client.enable_modulation(center=centres,        # per-tone centres (Hz), or None for the live comb
                         offsets=[-1e3, 0, 1e3], # probe offsets (Hz); (n_points,) or (n_points, n_mod)
                         mod_indices=None,        # None = all resonator tones (blind tones excluded)
                         samples_per_point=4,
                         n_settle=1)

# Then EITHER continuous streaming ...
client.enable_stream()        # tagged modulated frames flow to stream clients
# ... OR a finite capture over the request socket:
raw = client.get_samples(3000)   # returns modulated, tagged frames while armed
```

Both paths return ordinary frames; the only difference from normal data is the
per-sample modulation tag (see [the data frame tag](#the-data-frame-tag)).
`parse_samples` decodes the tag for you:

```python
data = client.parse_samples(raw)
data['modulation_point']     # 1..N (0 means modulation was off for that sample)
data['modulation_settling']  # 1 on the first n_settle samples after each switch
data['modulation_revision']  # which config revision produced the sample
```

> **Arm ≠ stream.** `enable_modulation` only loads the config. Start output
> separately with `enable_stream` (continuous) or `get_samples` (finite). A
> finite `get_samples` capture is rejected while a continuous stream is running,
> so exactly one producer drives the hardware at a time.

### 3. Demodulate

The demod tool lives in `souk_readout_tools.demod` and is a set of **pure
functions** (arrays in, arrays out) — no client or socket dependency.

```python
state = client.get_modulation_state()                 # carries the per-point offsets
grouped = demod.group_modulation_cycles(data, state)  # group by point & cycle, drop settling
result = demod.demodulate(grouped, linewidth_hz=cfg['linewidth_hz'])
```

`result` holds per-cycle, per-tone arrays of shape `(n_cycles, n_tones)`:

| key | meaning |
|-----|---------|
| `dphi_df` | local phase slope, rad/Hz |
| `d2phi_df2` | local curvature, rad/Hz² (NaN for < 3 points) |
| `freq_shift_hz` | the detector signal: centre-point phase ÷ slope |
| `detuning_linewidths` | how far the centre sits from the inflection (needs `linewidth_hz`) |
| `needs_update` | True when `|detuning_linewidths|` exceeds the threshold (default 0.1) |
| `dissipation` | loss coordinate (only with a resonance-circle calibration) |
| `z_center` | complex value at the centre probe point |

Choose the estimator with `method=`: `'fast'` (finite differences), `'accurate'`
(weighted polynomial fit — handles asymmetric offsets), or `'model'` (calibrated
non-ideal fit).

### 4. Track: update the centres live

`update_modulation` swaps in new centres and/or offsets **without dropping
data** — the change is applied at a cycle boundary and the stream never pauses:

```python
# A tracking loop computes new centres from result['needs_update'] / detuning, then:
client.update_modulation(center=new_centres)
```

Small moves ride the filterbank's natural channel overlap with no interruption.
If a move is large enough to push a tone beyond that coverage, the update is
**rejected** and you must explicitly recentre (a deliberate, brief reload of the
channel maps):

```python
ack = client.update_modulation(center=big_move)
if ack['status'] == 'error':                 # tones beyond coverage
    client.recenter_modulation()             # reload maps for the current centre
```

### 5. Inspect the state

`get_modulation_state()` returns a cheap, hardware-free snapshot you can poll
while streaming:

```python
st = client.get_modulation_state()
st['enabled'], st['num_points'], st['samples_per_point']
st['desired_revision'], st['applied_revision']    # queued vs actually-applied config
st['needs_recenter'], st['tones_beyond_coverage'] # bin-coverage health
for tone in st['tones']:
    tone['index'], tone['center_hz'], tone['offsets_hz'], tone['occupancy']
```

`occupancy` is per probe point: `'nearest'` (on the home bin), `'second'`
(riding the overlapping neighbour — fine, just flagged), or `'beyond'` (needs a
recentre).

---

## Key concepts

- **Points (N).** The number of probe frequencies per cycle. 2 gives the slope;
  3 adds curvature (and thus inflection-point tracking). N can be larger.
- **Centre + offsets.** Each tone has a per-tone **centre** (its operating
  frequency) and a small set of **probe offsets** added to it. Offsets are
  per-tone-per-point — each resonator can use a different probe spacing scaled
  to its own bandwidth.
- **Dwell (`samples_per_point`).** How many accumulations are taken at each
  point before switching. Larger dwell = higher live fraction (see
  [Sensitivity](#sensitivity)).
- **Settling (`n_settle`).** The first few samples after a switch may not be
  fully settled; they are **flagged, not dropped**, so you can discard them in
  analysis if you wish.
- **Revision.** A counter bumped on every enable/update. It is stamped into
  every frame so offline analysis knows which centre/offsets produced it.
- **Bin occupancy / overlap.** The filterbank channels overlap ~2×, so a tone
  stays well covered even as it drifts up to about a full channel away from its
  home bin. Modulation exploits this: the channel maps are held **fixed** and
  the tones ride the overlap, avoiding any map rewrite on the fast path.
- **User vs firmware indices.** Everything you pass or read back
  (`center`, `offsets`, `mod_indices`, the I/Q columns, the `tone_modulation`
  state) is in **user-facing tone order** — the same order as your tone list.
  The server maps to internal firmware/VACC indices itself.
- **Blind tones are never modulated.** They have no resonance to track; the
  default `mod_indices` excludes them and explicitly asking to modulate one is
  an error.

---

## The data frame tag

Modulation reuses the otherwise-unused sixth stream flag word (`flag5`,
`frame[-5]`) as a packed tag. **It is no longer a boolean** when modulating; the
wire format is unchanged but the meaning is not. Decode it as **unsigned**:

```
bits 0..15    active point: 0 = modulation off, 1..N = the probe point
bit  16       settling/transient marker
bits 17..31   modulation configuration revision
```

`parse_samples` does this for you (`modulation_point`, `modulation_settling`,
`modulation_revision`). Plain (non-modulated) streams leave `flag5 = 0`, so
these fields read as zeros and existing consumers are unaffected. During the
settling window the legacy `FLAG_SET_FREQS` flag is also held high so older
consumers still see "frequencies changing".

---

## API reference

### Client (`ReadoutClient`)

| method | summary |
|--------|---------|
| `enable_modulation(center=None, offsets=None, mod_indices=None, samples_per_point=1, n_settle=1)` | Arm modulation (does not start output). With no args, re-arms a previously loaded config. Returns an ack with `revision` and `needs_recenter`. |
| `update_modulation(center=None, offsets=None, on_map_change='continue')` | Live update of centres and/or offsets with no dropped data. `on_map_change='recenter'` permits a map reload if a tone leaves bin coverage; otherwise such an update is rejected. |
| `recenter_modulation()` | Reload channel maps / mixer frequencies for the current centre and recompute bin sharing (a deliberate brief break). |
| `disable_modulation()` | Pause modulation; tones rest at their centres. The config stays resident so `enable_modulation()` re-arms quickly. |
| `get_modulation_state()` | Return the `tone_modulation` state (armed flag, revisions, per-tone centres/offsets/occupancy, `needs_recenter`). Hardware-free; safe to poll. |
| `get_samples(num_samples)` | When modulation is armed, returns modulated, tagged frames; otherwise unchanged. |
| `parse_samples(raw)` | Adds `modulation_point`, `modulation_settling`, `modulation_revision` to the parsed dict. |

The same on/off applies to continuous streaming via the existing
`enable_stream()` / `disable_stream()`.

### Demod (`souk_readout_tools.demod`)

| function | summary |
|----------|---------|
| `modulation_params_from_sweep(sweep, *, n_points=3, samples_per_point=1, n_settle=1, delta_linewidths=0.25, exclude_blind=True, blind_indices=None)` | Fit a sweep → ready-to-use config (`center`, `offsets`, `mod_indices`, dwell, `linewidth_hz`, `summary`). |
| `group_modulation_cycles(data_dict, tone_modulation_state, reduce='mean')` | Group parsed samples by point and cycle (settling dropped); `reduce='mean'` averages each dwell, `reduce=None` keeps the sample axis. |
| `demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None, circle_cal=None, phase_baseline=None, threshold_linewidths=0.1)` | Per-cycle `dphi_df`, `d2phi_df2`, `freq_shift_hz`, `detuning_linewidths`, `detuning_hz`, `needs_update`, `dissipation`, `z_center`. |

### Server `get_info('tone_modulation')`

Returns the cached state (no hardware access): `enabled`, `desired_revision`,
`applied_revision`, `revision_history`, `num_points`, `samples_per_point`,
`n_settle`, `mod_indices`, `sample_rate_hz`, `cycle_rate_hz`, `needs_recenter`,
`tones_beyond_coverage`, and a per-tone `tones` list (`index`, `firmware_index`,
`center_hz`, `armed_fft_bin`, `offsets_hz`, `drift_bins`, `occupancy`).

---

## How it works

### One streamer, modulation as a mode

There is a single streaming coroutine on the server (`stream_data`). Two events
gate it: `e_stream_enabled` (is it producing frames at all?) and
`e_modulation_enabled` (modulate or emit plain frames?). Normal streaming is
simply the modulation-off case, so there is never a second task to track. The
finite `get_samples` path shares the same stepping engine; an acquisition-owner
lock ensures only one producer drives the hardware at a time.

### The double-buffer ping-pong

The firmware has two LO control buffers. To change every tone's frequency
without disturbing the point currently being accumulated, the scheduler writes
the **next** point's words into the **inactive** buffer, flips the active-buffer
index, and pulses sync; the just-vacated buffer is then free to receive the
following point during the current dwell. For **N = 2** the two points live
permanently in the two buffers, so each switch is just an index flip + sync with
no buffer write at all (the fastest possible switch). For **N ≥ 3** the
"write-inactive → swap → prep-next" pattern keeps the live buffer untouched, so
the scheme is correct for any N, including odd N across repeated cycles.

### Riding the channel overlap (armed bins)

The channel maps are computed once when you arm, from the centre comb, and then
**held fixed**. Because the filterbank channels overlap ~2×, a tone stays
covered as it dithers and drifts. Crucially, each probe point's mixer phase is
computed relative to the **armed** bin (not the tone's instantaneous nearest
bin) — otherwise the phase would jump by a whole bin the moment a tone crossed a
bin boundary. When tones eventually drift past the overlap coverage, that is
reported (`needs_recenter` / `occupancy = 'beyond'`) and a `recenter_modulation`
reloads the maps and recomputes bin sharing.

### Seamless live updates

`update_modulation` does the heavy preparation off the streaming hot path (in a
worker thread) and posts the result to a single latest-wins slot. The streamer
applies it at the next cycle boundary — it is the sole owner of all modulation
hardware writes, which avoids any race with an in-flight cycle. A same-maps
update swaps the per-point words in place (no map rewrite, no reset); the new
frequencies take effect within one cycle with no dropped frames. Every applied
update bumps the revision, and the full revision→config history is retained in
the `tone_modulation` state for offline analysis across updates.

### The demod maths

For each cycle and tone, the probe-point phases are fit against their offsets:
the first derivative gives `dphi_df`; the second difference / quadratic fit
gives `d2phi_df2` (needs ≥ 3 points, else NaN). The detector **frequency shift**
is the centre-point phase deviation divided by the local slope, relative to a
defined baseline. The **detuning** (how far the centre is from the inflection)
follows from the ratio `d²φ/df² ÷ dφ/df`, which has units of 1/Hz — so
expressing it in linewidths needs a **linewidth scale** (or a calibrated model);
without one, detuning and `needs_update` are returned as NaN/False.

> **Dissipation** is not a naive `d|S21|/df`: as a resonator detunes the
> operating point slides around the resonance circle, so a reliable dissipation
> reading needs the resonance-circle calibration (centre/radius/rotation from a
> fit), supplied via `circle_cal`. Without it, `dissipation` is NaN.

---

## Sensitivity

Time-division-multiplexing the probe across N points does **not** inherently
cost a factor of N (or √N) in sensitivity to the frequency shift, because every
probe point carries information about it. Combining points optimally, the
*variance* penalty versus parking a single tone at the steepest point is
`N·S_max² / Σᵢ Sᵢ²` (where `Sᵢ` is the local slope at point i) — and the
noise-amplitude penalty is its square root. For a symmetric two-point pattern in
the linear regime this is 1 (no penalty); for three points it is ≈ 1 when the
delta is small (points clustered near the high-slope inflection), and only grows
if the delta pushes points onto the shallow shoulders. **So keep the per-tone
delta small / near the inflection.**

The real cost is **duty cycle**: settling samples and switch time are
*potentially* unusable (the accumulator may still integrate useful signal across
part of a switch — measure the usable fraction on hardware). Keep
`samples_per_point` comfortably larger than the settling window, which is why
the switch is engineered to be fast.

---

## Testing without hardware

Everything except the firmware register writes can be exercised in mock mode.
The mock server simulates modulation with a simple resonator phase model
(inflection at `f₀`, magnitude dip), so the demod tool has ground truth, and a
mock channel grid so bin-occupancy / recentre behave realistically.

```bash
PYTHONPATH=src python src/souk_readout_tools/client/client_scripts/test_frequency_modulation.py
```

This end-to-end script checks the tag structure, gap-free counter, uint32
decode, live update + revision, occupancy/recentre, index consistency,
pause/resume, the demod tool, and `modulation_params_from_sweep`.

```python
client = ReadoutClient(mock=True)
client.enable_modulation(center=[2.0e9, 2.1e9], offsets=[-1e3, 0, 1e3],
                         samples_per_point=4, n_settle=1)
data = client.parse_samples(client.get_samples(120))
print(data['modulation_point'][:12])   # 1 1 1 1 2 2 2 2 3 3 3 3
```

---

## Status and limitations

- The data plane (tagging, decode, scheduler bookkeeping, mock, demod,
  parameter derivation) is implemented and verified in mock mode; the scheduler
  ping-pong is unit-tested for N = 2, 3, 5 with no active-buffer corruption.
- The firmware preparer and the scheduler's register writes follow the existing
  fast-path patterns but **still need on-hardware validation** (switch timing,
  odd-N correctness, recentre behaviour, and the actual sensitivity vs a
  parked-tone baseline). The half-/full-channel overlap thresholds are a
  reasonable hypothesis pending measurement.
- **Live revision persistence** into the raw-stream sidecar / G3 metadata is a
  follow-up; for now the server retains the full `revision_history` in
  `get_info('tone_modulation')`, which is sufficient to map any frame's revision
  to its config offline.
- The **automated tracking loop** (acting on `needs_update` to re-centre) is not
  yet built; the API above provides everything it needs.
```
