# Fast Frequency Modulation — Technical Reference

Fast frequency modulation steps each readout tone through a small set of probe
frequencies (N = 2 or 3, generally) on every accumulation and returns the
samples as ordinary stream frames. Each sample is tagged with the **index of the
step within the modulation cycle** (1..N; 0 when not modulating) and a
**settling flag**. It provides a real-time measurement of each resonator's local
phase response (`dφ/df`, and with three points `d²φ/df²`) for live
frequency-shift / dissipation demodulation and operating-point tracking.

- Server: `souk_readout_tools/server/readout_server.py`, `firmware_lib.py`
- Client: `souk_readout_tools/client/readout_client.py`
- Modulation toolkit (setup + demod): `souk_readout_tools/modulation.py` (pure functions)
- Mock + test: `client/mock_readout.py`, `client/client_scripts/test_frequency_modulation.py`

---

## Contents

- [Usage](#usage)
- [Data model](#data-model)
- [Frame tag (flag5)](#frame-tag-flag5)
- [Control API](#control-api)
- [Demodulation API](#demodulation-api)
- [`tone_modulation` state schema](#tone_modulation-state-schema)
- [Internals](#internals)
- [Sensitivity](#sensitivity)
- [Testing](#testing)
- [Status and limitations](#status-and-limitations)

---

## Usage

Canonical sequence (sweep → arm → acquire → demodulate):

```python
from souk_readout_tools import modulation as mod

cfg = mod.params_from_sweep(client.get_sweep_data(), n_points=3, samples_per_point=4)
client.enable_modulation(center=cfg['center'], offsets=cfg['offsets'],
                         mod_indices=cfg['mod_indices'],
                         samples_per_point=cfg['samples_per_point'],
                         n_settle=cfg['n_settle'])           # arms only
raw  = client.get_samples(3000)                              # finite, tagged capture
# or: client.enable_stream()                                # continuous, tagged stream
data = client.parse_samples(raw)
grouped = mod.group_cycles(data, client.get_modulation_state())
result  = mod.demodulate(grouped, linewidth_hz=cfg['linewidth_hz'])

# result is a dict of (n_cycles, n_tones) arrays in user tone order:
fshift  = result['freq_shift_hz']          # detector frequency-shift signal (Hz)
slope   = result['dphi_df']                # live calibration slope (rad/Hz)
detune  = result['detuning_linewidths']    # offset of centre from the inflection
retune  = result['needs_update']           # bool: True where the centre should be re-tuned
tone0_fshift = result['freq_shift_hz'][:, 0]   # time series (one value per cycle) for tone 0

client.update_modulation(center=new_centres)                # seamless live re-centre
client.disable_modulation()                                 # tones rest at centre
```

Usage notes:

- **Arm ≠ stream.** `enable_modulation` only loads the config and sets the
  modulation mode; it does **not** start output. Start output with
  `enable_stream` (continuous) or `get_samples` (finite). Both return tagged
  frames while armed.
- **Single producer.** Continuous streaming and a finite `get_samples` capture
  must not run simultaneously; a `get_samples` modulated capture is rejected
  while a continuous stream is active. `burst=True` is rejected while modulating.
- **Indices are user-facing.** `center`, `offsets`, `mod_indices`, the I/Q
  columns and the `tone_modulation` state are all in user tone order; the server
  maps to firmware/VACC indices internally.
- **`offsets` shape** is `(n_points,)` (one offset per point, broadcast across
  modulated tones) or `(n_points, len(mod_indices))` (per tone). A 1-D array is
  read as one-per-point, not one-per-tone.
- **Blind tones are never modulated.** Default `mod_indices` excludes them;
  passing a blind index is an error.
- **Decode the tag as unsigned.** `parse_samples` already does this and adds
  `modulation_point` (1..N, 0 = off), `modulation_settling`, `modulation_revision`.
- **Settling samples are flagged, not dropped.** Discard `modulation_settling==1`
  samples in analysis if required; the stream itself omits nothing intentionally.
- **Keep the probe delta small** (near the inflection) to minimise the
  sensitivity penalty — see [Sensitivity](#sensitivity).
- **Detuning needs a linewidth scale.** `detuning_linewidths` / `needs_update`
  are NaN/False unless `linewidth_hz` (or `method='model'`) is supplied.
- **Dissipation needs a circle calibration** (`circle_cal`); otherwise NaN.
- **Live updates are seamless while in coverage.** Small moves ride the
  filterbank overlap with no dropped data. A move that pushes a tone beyond
  coverage is rejected unless `on_map_change='recenter'`; use
  `recenter_modulation()` for a deliberate (brief) channel-map reload.
- **Reading `demodulate` results.** The return value is a dict of
  `(n_cycles, n_tones)` arrays in user tone order — one row per completed
  modulation cycle (cycle rate ≈ `sample_rate / (N · samples_per_point)`). Index
  `[:, tone]` for a per-cycle time series of one tone. Common keys:
  `freq_shift_hz` (detector signal), `dphi_df` / `d2phi_df2` (live calibration),
  `detuning_linewidths` + `needs_update` (tracking), `dissipation`, `z_center`,
  plus `revision` (shape `(n_cycles,)`). A NaN entry means the quantity is
  unavailable for that configuration (e.g. `d2phi_df2`/detuning with too few
  points or no `linewidth_hz`). Full list under [Demodulation API](#demodulation-api).

---

## Data model

| term | definition |
|------|------------|
| **point (N)** | One probe frequency in the cycle. N≥2 gives `dφ/df`; N≥3 adds `d²φ/df²`. |
| **center** | Per-tone operating frequency (Hz), user order, length = active tone count. |
| **offsets** | Per-point (and optionally per-tone) probe offsets (Hz) added to `center`. |
| **dwell (`samples_per_point`)** | Accumulations emitted per point per cycle. |
| **settling (`n_settle`)** | Leading samples per point flagged as transient. |
| **cycle** | One pass through all N points (`N · samples_per_point` samples). |
| **revision** | 15-bit counter, bumped per enable/update, stamped in every frame. |
| **armed bins/maps** | The FFT channel each tone is assigned to (and the VACC indices), fixed when you arm; the fast path never rewrites them, so a tone keeps using its armed channel even as it is dithered/drifts. |
| **bin occupancy** | Where each probe point lands relative to its armed channel, as a drift in channels — i.e. how well the fixed channel map still covers it. Per (tone, point): `nearest` (within ±½ channel, on its home channel), `second` (½–1 channel away — now closer to the neighbouring channel but still fully covered thanks to the ~2× filterbank oversampling; just flagged), `beyond` (>1 channel — no longer covered, so the maps must be reloaded via a recentre). Reported in `drift_bins` (signed, in channels) and `occupancy`. |

---

## Frame tag (flag5)

Modulation repurposes the sixth stream flag word `flag5` = `frame[-5]` (an
`int32`). The wire format is unchanged; the semantics are **not** boolean while
modulating. Decode as **unsigned** (the revision can set the sign bit):

```
bits  0..15   modulation step index  (0 = modulation off, 1..N = step in the cycle)
bit   16      settling/transient marker
bits 17..31   modulation configuration revision (0..0x7FFF)
```

- Non-modulated streams leave `flag5 = 0` → all derived fields read 0 (backward
  compatible).
- During the settling window the legacy `FLAG_SET_FREQS` flag is also held high
  for consumers that do not parse `flag5`.
- Server: written in `ReadoutServer.prepare_frame(..., mod_point, settling, revision)`.
- Client: `parse_samples` emits `modulation_point`, `modulation_settling`,
  `modulation_revision` (each shape `(n_samples,)`).

---

## Control API

Client methods (each maps to a server request; mock-mode supported). All return
the server ack dict (`{'status': 'success'|'error', ...}`).

### `enable_modulation(center=None, offsets=None, mod_indices=None, samples_per_point=1, n_settle=1)`
Arm modulation (does not start output).
- `center` — per-tone centre RF freqs (Hz), user order; `None` uses the current comb.
- `offsets` — probe offsets (Hz); `(n_points,)` or `(n_points, len(mod_indices))`.
- `mod_indices` — tones to modulate; `None` = all regular (resonator) tones; blind indices error.
- `samples_per_point`, `n_settle` — dwell and settling counts.
- No-args call re-arms a previously loaded config.
- Result: `{'revision', 'needs_recenter'}`. Sets the modulation-enabled event; does **not** set the stream-enabled event.

### `update_modulation(center=None, offsets=None, on_map_change='continue')`
Seamless live update of centre and/or offsets (applied at a cycle boundary, no dropped data).
- Omitted args keep their current values.
- `on_map_change='continue'` (default): rejected if any (tone, point) would leave bin coverage (returns `{'tones_beyond_coverage': [...]}`). `'recenter'`: performs the map reload instead.
- Result: `{'revision', 'op': 'update'|'recenter'}`.

### `recenter_modulation()`
Reload channel maps / mixer frequencies for the current centre and recompute VACC bin-sharing (a deliberate brief break). Result: `{'revision'}`.

### `disable_modulation()`
Clear the modulation-enabled event; rest tones at their centres. The config stays resident for a fast re-arm. (Use `disable_stream()` to stop output entirely.)

### `get_modulation_state()`
Return the `tone_modulation` section (see [schema](#tone_modulation-state-schema)). Pure server-side read (no hardware access); safe to poll while streaming.

### Acquisition (existing methods, modulation-aware)
- `enable_stream()` / `disable_stream()` — continuous output on/off.
- `get_samples(num_samples, burst=False)` — finite capture; returns tagged frames when armed (whole-cycle warm-up, aligned to point 1; `burst=True` rejected).
- `parse_samples(raw)` — adds the three `modulation_*` arrays.

Server requests: `enable_modulation`, `update_modulation`, `recenter_modulation`,
`disable_modulation`, plus `get_info('tone_modulation')`. Status dicts
(`health_check`, server info) include `modulation_streaming`.

---

## Demodulation API

`souk_readout_tools.modulation` — pure functions (arrays/dicts in, arrays out; no
client/socket dependency, relocatable server-side).

### `params_from_sweep(sweep, *, n_points=3, samples_per_point=1, n_settle=1, delta_linewidths=0.25, exclude_blind=True, blind_indices=None)`
Fit a sweep into an `enable_modulation` config.
- `sweep` — dict with `f`, `z` arrays of shape `(n_sweep_points, n_tones)` (Hz, complex S21); optional `blind_indices`.
- Per tone: centre = steepest (inflection) point; linewidth from the peak slope (`w ≈ 4/|dφ/df|_max` for an arctan phase).
- Probe pattern: symmetric `linspace(-1, 1, n_points)` scaled by `delta_linewidths · linewidth` per tone.
- Returns `{'center', 'offsets' (n_points, n_mod), 'mod_indices', 'samples_per_point', 'n_settle', 'linewidth_hz' (n_mod), 'summary'}`.

### `group_cycles(data_dict, tone_modulation_state, reduce='mean')`
Group parsed samples by point and cycle.
- Drops `settling` samples; aligns by the gap-free `packet_counter`; cycle boundary detected on point wrap.
- Per-(point, tone) offsets read from `tone_modulation_state['tones'][i]['offsets_hz']`.
- `reduce='mean'` → `z` shape `(n_cycles, N, n_tones)`; `reduce=None` → `(n_cycles, N, n_used, n_tones)`.
- Returns `{'z', 'offsets_hz' (N, n_tones), 'revision' (n_cycles,), 'point_order'}`.

### `demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None, circle_cal=None, phase_baseline=None, threshold_linewidths=0.1)`
Per-cycle, per-tone demodulation. Returns arrays of shape `(n_cycles, n_tones)`:

| key | definition |
|-----|------------|
| `dphi_df` | local phase slope (rad/Hz) |
| `d2phi_df2` | local curvature (rad/Hz²); NaN if N<3 |
| `freq_shift_hz` | `(φ_center − baseline) / dphi_df` |
| `detuning_linewidths` | `−(d²φ/df² ÷ dφ/df)·linewidth/8` (leading order); NaN without `linewidth_hz` or N<3 |
| `detuning_hz` | `detuning_linewidths · linewidth_hz` |
| `needs_update` | `|detuning_linewidths| > threshold_linewidths` |
| `dissipation` | loss coordinate from `circle_cal`; NaN otherwise |
| `z_center` | complex value at the centre probe point |

- `method`: `'fast'` (finite differences), `'accurate'` (weighted polynomial fit; handles asymmetric offsets), `'model'` (calibrated non-ideal fit; currently shares the `accurate` path).
- `phase_baseline` — per-tone reference phase for `freq_shift_hz`; `None` uses the per-tone mean centre-point phase across cycles.

---

## `tone_modulation` state schema

`get_info('tone_modulation')` / `get_modulation_state()`:

```python
{
  'enabled': bool,                      # modulation-enabled event state
  'desired_revision': int,              # last requested config revision
  'applied_revision': int,              # revision actually applied by the producer
  'revision_history': {rev: {'center', 'offsets', 'mod_indices', 'ts'}},
  'num_points': int, 'samples_per_point': int, 'n_settle': int,
  'mod_indices': [int, ...],            # user-facing
  'sample_rate_hz': float, 'cycle_rate_hz': float,
  'needs_recenter': bool,
  'tones_beyond_coverage': [int, ...],  # user-facing indices
  'tones': [
    {'index': int,                      # user-facing (matches stream I/Q columns)
     'firmware_index': int,             # internal VACC/LO index (annotation only)
     'center_hz': float,
     'armed_fft_bin': int,
     'offsets_hz': [float, ...],        # per point
     'drift_bins': [float, ...],        # per point, signed, in channels
     'occupancy': ['nearest'|'second'|'beyond', ...]},  # per point
    ...
  ],
}
```

---

## Internals

### One streamer, modulation as a mode
A single `stream_data` coroutine produces all stream frames, gated by two
events: `e_stream_enabled` (output on/off) and `e_modulation_enabled` (modulate
vs plain). Normal streaming is the modulation-off case. `get_samples` shares the
same stepping engine for finite captures; an acquisition-owner lock ensures a
single producer drives the hardware.

### Double-buffer ping-pong scheduler (`ModulationScheduler`)
The firmware has two LO control buffers. The scheduler writes the next point's
words into the **inactive** buffer, flips the active index, and pulses sync; the
vacated buffer then receives the following point during the dwell. Rules:
- Skip the write when the inactive buffer already holds the wanted point. For
  **N=2** the two points stay resident in the two buffers, so each switch is an
  index flip + sync only (no per-switch write).
- The live buffer is never written, so the scheme is correct for any N including
  odd N across repeated cycles.
- The first sample emitted is point 1 (armed live); subsequent visits emit after
  the swap, so settling marks the post-switch samples.
- Channel maps are written **once** at arm; never in the hot loop.

### Armed bins and overlap riding (`firmware_lib.prepare_modulation_settings_fast`)
Channel maps and VACC indices are computed once from the centre comb and held
fixed. Each probe point's mixer phase increment is computed relative to the
**armed** bin centre (not the tone's instantaneous nearest bin), so the phase is
continuous as a tone dithers/drifts across a bin boundary. The ~2× oversampled
filterbank keeps a tone covered to roughly a full channel of drift; occupancy
classification (`nearest`/`second`/`beyond`) reports this and triggers a
recentre when coverage is exceeded. `_rf_to_digital_baseband` performs the
RF→baseband mapping (UDC + Nyquist + DUC/DDC), shared with the fast tone setter.

### Live update mechanism
The request handler prepares the bundle off the hot path (thread executor,
including occupancy classification) and posts it to a single latest-wins slot.
The producer applies it at the next cycle boundary (`_apply_pending_modulation_command`)
and is the sole owner of modulation hardware writes — no race with an in-flight
cycle. A same-maps update swaps the per-point words in place (no map rewrite, no
reset); the new frequencies take effect within one cycle. The revision is bumped
and the full revision→config history retained for offline analysis.

### Demod definitions
For each cycle/tone, probe-point phases `φ(fᵢ)` are fit against their offsets:
`dφ/df` from a first-difference (`fast`) or degree-1 fit (`accurate`); `d²φ/df²`
from a symmetric second difference (`fast`, N=3) or degree-2 fit (`accurate`,
N≥3). `freq_shift_hz = (φ_center − baseline)/(dφ/df)`. The ratio
`d²φ/df² ÷ dφ/df` has units 1/Hz, so detuning in linewidths requires a linewidth
scale; the `fast` estimator uses the leading-order Lorentzian form
`detuning_lw ≈ −(ratio·w)/8`, and `model` would calibrate it per device.
Dissipation is not `d|S21|/df` (the operating point slides around the resonance
circle as the tone detunes); it requires the circle calibration to refer the
amplitude back to the on-resonance point.

---

## Sensitivity

TDM across N points does not inherently cost N (or √N) for the frequency-shift
signal, because every point carries information about it. The optimal-combination
*variance* penalty vs parking at the steepest point is `N·S_max² / Σᵢ Sᵢ²`
(`Sᵢ` = local slope); the noise-amplitude penalty is its square root. For a
symmetric 2-point pattern in the linear regime this is 1; for 3 points ≈1 when
the delta is small (near the high-slope inflection), growing only as the delta
reaches the shallow shoulders. The dominant real cost is **duty cycle**: settling
samples and switch time are *potentially* unusable (the accumulator may still
integrate useful signal across part of a switch — measure the usable fraction on
hardware), so keep `samples_per_point` well above the settling window. The fast
cycle rate also rejects low-frequency amplifier drift (lock-in effect).

---

## Testing

Mock mode exercises everything except the firmware register writes. The mock
simulates modulation with a resonator phase model (inflection at `f₀`, magnitude
dip) for demod ground truth, plus a mock channel grid for occupancy/recentre.

```bash
PYTHONPATH=src python src/souk_readout_tools/client/client_scripts/test_frequency_modulation.py
```

Covers: tag structure / dwell / settling, gap-free counter, unsigned decode,
live update + revision, occupancy + recentre, index consistency, pause/resume,
the demod tool, and `params_from_sweep`. The `ModulationScheduler`
ping-pong is separately unit-tested for N = 2, 3, 5 (no active-buffer
corruption; channel maps written once).

---

## Status and limitations

- Data plane (tagging, decode, scheduler bookkeeping, mock, demod, parameter
  derivation) implemented and verified in mock mode.
- Firmware preparer and the scheduler's register writes follow the existing
  fast-path patterns but require **on-hardware validation**: switch timing,
  odd-N correctness, recentre behaviour, and measured sensitivity vs a
  parked-tone baseline. The ½/1-channel overlap thresholds are provisional
  pending measurement.
- **Revision persistence** into the raw-stream sidecar / G3 metadata is a
  follow-up; `revision_history` in `tone_modulation` currently suffices to map a
  frame's revision to its config offline.
- The **automated tracking loop** (acting on `needs_update`) is not implemented;
  the API provides the required inputs and the seamless `update_modulation` path.
```
