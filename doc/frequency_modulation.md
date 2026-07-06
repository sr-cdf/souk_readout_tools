# Frequency Modulation

Resonator readout measures each detector through a probe tone parked on its
resonance: optical power shifts the resonance frequency, and the shift appears
as a phase change of the probe. Converting that phase back into a frequency
shift — the science signal — needs the local slope of phase against frequency,
`dφ/df`, and the measurement is only well behaved while the tone actually sits
at the centre of the resonance. Both are normally established with a frequency
sweep, and both drift with optical loading: change the telescope elevation and
the calibration is stale, which is why observing traditionally pauses for a
re-sweep at every turnaround.

Frequency modulation removes that dependence on repeated sweeps. Instead of
parking each tone at one frequency, the readout cycles it around a small set of
**probe points** — typically 2–4 offsets, a fraction of a linewidth apart,
dwelling a few accumulations on each — and tags every streamed sample with the
probe point it came from. Grouping the samples by point recovers each
resonator's local response *continuously*, alongside the data:

- **Two points measure the slope `dφ/df`** — a live responsivity for every
  tone, refreshed once per modulation cycle. The calibration a sweep used to
  provide now streams with the data, so recurring sweeps at each turnaround
  are no longer needed.
- **Three or more points add the curvature `d²φ/df²`.** The phase response has
  an inflection at the resonance centre — zero curvature on centre, growing as
  the tone detunes — so the curvature-to-slope ratio measures how far
  off-centre each tone sits (with a sweep calibration, the demodulator instead
  inverts the fitted model exactly). Either way you get a live
  `detuning_linewidths` and a `needs_update` flag per tone, and
  `update_modulation(center=…)` re-centres a drifting tone **seamlessly** — no
  dropped data — so tones can be kept centred on their resonances
  indefinitely.

The demodulated output is a per-cycle time series of frequency shift (and,
with a sweep-derived calibration, dissipation) for every tone. Cycles are fast
— a 4-point cycle dwelling 4 accumulations per point completes at 1/16th of
the sample rate — so tracking runs far above the drift it corrects, and the
sensitivity penalty is small (mostly duty cycle; see
[Sensitivity](#sensitivity)).

Start with the [Quick start](#quick-start) and the
[standard sweep-calibrated workflow](#standard-workflow-calibrate-from-a-sweep).
[How it works](#how-it-works) gives a one-screen picture of the machinery;
everything after it is reference material.

Where things live:

- Server: `souk_readout_tools/server/readout_server.py`, `firmware_lib.py`
- Client: `souk_readout_tools/client/readout_client.py`
- Modulation toolkit (setup + demod): `souk_readout_tools/modulation.py` (pure functions)
- Mock + test: `client/mock_readout.py`, `client/client_scripts/test_frequency_modulation.py`

---

## Contents

Getting started:

- [Quick start](#quick-start)
- [Standard workflow: calibrate from a sweep](#standard-workflow-calibrate-from-a-sweep)
- [Tracking the operating point](#tracking-the-operating-point)
- [Full-rate timestreams and noise](#full-rate-timestreams-and-noise)
- [How it works](#how-it-works)

Reference:

- [Usage notes and fine print](#usage-notes-and-fine-print)
- [Data model](#data-model)
- [Frame tag (flag5)](#frame-tag-flag5)
- [Control API](#control-api)
- [Firmware-slot modulation (v7.11)](#firmware-slot-modulation-v711)
- [Software modulation](#software-modulation)
- [Demodulation API](#demodulation-api)
- [`modulation` state schema](#modulation-state-schema)
- [Internals](#internals)
- [Sensitivity](#sensitivity)
- [Status and limitations](#status-and-limitations)

---

## Quick start

The minimal loop: enable modulation with hand-picked probe offsets, capture,
group, demodulate. No sweep or fitting required.

```python
from souk_readout_tools import modulation as mod

# Probe each tone at 4 offsets around its current frequency (Hz).
client.enable_modulation(offsets=[-2e4, 0.0, +2e4, 0.0], samples_per_point=4)

raw     = client.get_samples(4000)                   # finite, tagged capture
data    = client.parse_samples(raw)                  # adds modulation_point per sample
grouped = mod.group_cycles(data, client.get_modulation_state())
result  = mod.demodulate(grouped, linewidth_hz=1e5)  # model-free: needs a linewidth scale

fshift = result['freq_shift_hz']   # (n_cycles, n_tones) frequency shift per cycle
slope  = result['dphi_df']         # (n_cycles, n_tones) live responsivity
client.disable_modulation()
```

Three things worth knowing at this stage:

- The default engine is the v7.11 **firmware-slot** engine, which cycles
  exactly four hardware LO slots — so give it four offsets, repeating a value
  (as above) to reuse a frequency. `engine='sw'` accepts any number of points;
  everything downstream is identical.
- `offsets` is one value per probe point, applied to every modulated tone; a
  `(n_points, n_mod_tones)` array sets them per tone instead.
- Without a calibration the demodulation is **model-free**: it needs a rough
  `linewidth_hz` scale and cannot report dissipation. For exact, science-grade
  output, build the configuration from a sweep — next section.

---

## Standard workflow: calibrate from a sweep

`modulation.params_from_sweep` turns a sweep plus your own resonator fits into
a ready modulation config: per-tone operating points, probe offsets scaled by
each tone's linewidth, and a de-embedding **calibration** that makes the
demodulation exact (cable delay removed, resonance-centred phase). This is the
recommended path for science data.

```python
from souk_readout_tools import modulation as mod, fitting

# 1. Sweep and fit.
sweep = client.parse_sweep_data(client.get_sweep_data())
fits  = fitting.batch_fit(sweep, verbose=False)

# 2. Build the config. point_sequence=(1,2,3,2) gives offsets
#    [-delta, 0, +delta, 0] per tone (delta = 0.1 linewidths by default) —
#    a 4-point pattern, matching the firmware-slot engine's 4 slots.
cfg = mod.params_from_sweep(sweep, point_sequence=(1, 2, 3, 2),
                            samples_per_point=4, fits=fits)

# 3. Arm and acquire.
client.disable_stream()                     # loading rewrites channel maps
client.enable_modulation(center=cfg['center'], offsets=cfg['offsets'],
                         mod_indices=cfg['mod_indices'],
                         samples_per_point=cfg['samples_per_point'],
                         n_settle=cfg['n_settle'])
raw  = client.get_samples(3000)             # finite capture ...
# or: client.enable_stream()                # ... or continuous tagged stream

# 4. Demodulate in the calibrated (de-embedded) basis.
data    = client.parse_samples(raw)
grouped = mod.group_cycles(data, client.get_modulation_state())
result  = mod.demodulate(grouped, calibration=cfg['calibration'])

client.disable_modulation()                 # tones rest at their centres
```

`result` is a dict of `(n_cycles, n_tones)` arrays in user tone order — one
row per completed modulation cycle. The keys you will use most:

- `freq_shift_hz` — the centre probe's offset from the fitted resonance,
  `f_probe − f_r`. The probe centres are fixed, so resonance motion appears
  here with the opposite sign: for detector signal at a fixed centre, negate
  the change from a baseline (`detector_df = -(freq_shift_hz - baseline)`), or
  recover the absolute resonance directly:
  `fr_hz = cfg['center'][None, :] - result['freq_shift_hz']`.
- `dphi_df` / `d2phi_df2` — the live responsivity and curvature.
- `dissipation` — the matched-scale `Δ(1/(2·Qi))` dissipation quadrature
  (calibrated basis only).
- `detuning_linewidths` / `needs_update` — the tracking inputs; next section.

The same `cfg` drives the software engine — pass `engine='sw'` to
`enable_modulation` (any number of points works there; you are not restricted
to 4). The full key list is under [Demodulation API](#demodulation-api);
shapes, sign conventions and edge cases are in
[Usage notes and fine print](#usage-notes-and-fine-print).

---

## Tracking the operating point

This is the point of the curvature measurement: keeping tones centred without
sweeping. Each demodulated cycle reports how far each tone sits from its
resonance (`detuning_linewidths` — exact in the calibrated basis,
curvature-based model-free) and flags tones beyond a threshold with
`needs_update` (default 0.1 linewidths). Feed corrections back with:

```python
client.update_modulation(center=new_centres)    # seamless: no dropped data
```

`update_modulation` swaps the tone combs glitch-free while streaming, so small
corrections cost nothing. Two things to keep in mind:

- **The operating point is deliberately not `fr`.** `params_from_sweep` parks
  each tone at the steepest point of the de-embedded response — the best-SNR
  point for reading a frequency shift — which is displaced from the fitted
  `fr` by the resonance asymmetry. `freq_shift_hz` therefore reads a constant
  baseline (`cfg['reference_freqs']['offset_from_fr_hz']`) when everything is
  on-point, not ~0; subtract it before deciding whether to re-tune.
- **Big moves need a recentre.** The channel maps are fixed when you arm; a
  tone can drift about one filterbank channel before its map no longer covers
  it. `update_modulation` rejects moves beyond coverage; use
  `recenter_modulation()` (a deliberate, brief break) to re-arm on fresh maps.
  See [Internals](#internals).

An automated closed-loop tracker is not shipped (see
[Status and limitations](#status-and-limitations)); the API provides the
inputs (`needs_update`) and the seamless actuation (`update_modulation`).

---

## Full-rate timestreams and noise

`demodulate` gives one value per cycle (cycle rate = `sample_rate /
(n_points · samples_per_point)`). For noise measurements and PSDs you usually
want every sample instead: `demodulate_timestream` demodulates each probe
point against its cycle's local slope and reference, giving sample-aligned
`(n_samples, n_tones)` arrays at the full accumulation rate, ready for the
standard plotting helpers:

```python
grouped = mod.group_cycles(data, state, reduce=None, include_settling=True)
ts = mod.demodulate_timestream(grouped, data_dict=data)

plot_timestream(ts, format='freq_diss')
plot_timestream_psd(ts, format='freq_diss')     # no sweep needed
```

Details under [Demodulation API](#demodulation-api).

---

## How it works

Four ideas cover the machinery; everything else is detail.

**Samples are tagged, not separated.** Modulated data flows through the normal
stream as ordinary frames; the sixth flag word (`flag5`) carries which probe
point each accumulation belongs to, plus a settling marker and a config
revision. `parse_samples` decodes these into `modulation_point` (1..N, 0 = not
modulating), `modulation_settling` and `modulation_revision`, and
`group_cycles` uses them to fold the stream into a `(cycle, point)` grid. Wire
format: [Frame tag (flag5)](#frame-tag-flag5).

**Two engines produce the cycle; one API drives both.** Everything is driven
through `enable_modulation()` / `disable_modulation()` /
`get_modulation_state()`, with the `engine` argument selecting how the probe
frequencies are produced. The default **firmware-slot** engine (`engine='fw'`,
firmware v7.11) loads up to four LO combs into hardware mixer *slots* and lets
the firmware switch between them — round-robin or under manual control — with
no per-sample software writes: jitter-free, edge-aligned switching. The
original **software** engine (`engine='sw'`) steps the comb from software on
every accumulation via the two ping-pong LO buffers; it remains fully
supported and is the right choice for more than four probe points. Both emit
identical tagged data, so `parse_samples` / `group_cycles` / `demodulate` are
shared, and `get_modulation_state()` returns one unified payload with an
`engine` field (`'fw'`/`'sw'`/`None`). The engines are mutually exclusive —
they share the tag word — and run one at a time.

**Switching is clean; settling is at most one accumulation.** v7.11 aligns all
buffer/slot switches to accumulator edges, so a switch never lands
mid-accumulation. The only residual transient is the polyphase filterbank's
8-tap memory smearing the switch across ~8 *raw spectra* — under 1% of one
accumulation at a typical `acc_len` ≈ 1000, straddling only the one or two
accumulations bordering an edge — so `n_settle` (which counts whole
accumulations flagged as settling) of 0 or 1 is normally all you need.

**Channel maps are fixed at arm; probes ride the overlap.** The FFT channel
assigned to each tone (its *armed bin*) is written once when modulation is
enabled and never touched in the hot path, so the phase stays continuous as a
tone dithers or drifts. The ~2× oversampled filterbank keeps a tone covered
across roughly a full channel of drift; the state reports per-point
*occupancy* and a `needs_recenter` flag when coverage is exceeded, at which
point `recenter_modulation()` reloads the maps (a brief break). Riding past
the half-bin edge runs into the filterbank channel rolloff (−1 dB at ~0.70
bin spacings); each point's control words carry an automatic TX/RX gain (and
optionally phase) correction for it — see
[filterbank_compensation.md](filterbank_compensation.md). See
[Internals](#internals).

Everything below is reference material — dip in as needed.

---

## Usage notes and fine print

Grouped fine print that applies to both engines; engine-specific notes live in
the two engine sections.

**Arming and acquisition**

- **Arm ≠ stream.** `enable_modulation` loads the config and sets the
  modulation mode; it does **not** start data output. Start output with
  `enable_stream` (continuous) or `get_samples` (finite) — both return tagged
  frames while armed. (The firmware-slot engine does start its slot
  *switching* immediately on enable; the software engine steps only while
  producing.)
- **Disable streaming before enabling.** Loading a modulation config rewrites
  channel maps / control buffers, so continuous streaming must be off first.
- **Single producer.** Continuous streaming and a finite `get_samples` capture
  must not run simultaneously; a modulated `get_samples` is rejected while a
  continuous stream is active. `burst=True` is rejected while modulating.
- **`disable_modulation` ≠ `disable_stream`.** Disabling modulation rests the
  tones at their centres but keeps the config resident for a fast re-arm;
  `disable_stream` stops output entirely. Stopping a continuous modulated
  stream leaves modulation armed, so a following `get_samples()` is still
  modulated.
- Finite captures are aligned so sample 0 is point 1 of a cycle.

**Shapes and indexing**

- **Indices are user-facing.** `center`, `offsets`, `mod_indices`, the I/Q
  columns and the `modulation` state are all in user tone order; the server
  maps to firmware/VACC indices internally.
- **`offsets` shape** is `(n_points,)` (one offset per point, broadcast across
  modulated tones) or `(n_points, len(mod_indices))` (per tone). A 1-D array
  is read as one-per-point, never one-per-tone.
- **Blind tones are never modulated.** Default `mod_indices` excludes them;
  passing a blind index is an error.

**Tags and settling**

- **Decode the tag as unsigned.** `parse_samples` already does this and adds
  `modulation_point` (1..N, 0 = off), `modulation_settling`,
  `modulation_revision`.
- **Settling samples are flagged, not dropped.** The stream itself omits
  nothing; `group_cycles` drops flagged samples by default
  (`include_settling=True` keeps them). `n_settle` of 0 or 1 suffices — see
  [How it works](#how-it-works).

**Results**

- **Prefer the calibrated (centred) basis.** Pass
  `calibration=cfg['calibration']` (from `params_from_sweep(deembed=True)`) to
  `demodulate`. It de-embeds the cable delay and phase-centres each resonator,
  so the phase is well conditioned and the frequency shift and matched-scale
  `Δ(1/(2·Qi))` dissipation quadrature come out exactly for the linear notch
  model (Möbius inversion); the fitted Duffing model follows the same path
  with an analytic inverse. See
  [De-embedding](#de-embedding-and-the-centred-basis).
- **Model-free detuning needs a linewidth scale.** Without a `calibration`,
  `detuning_linewidths` / `needs_update` are NaN/False unless `linewidth_hz`
  is supplied; dissipation is unavailable (NaN). With a `calibration`, neither
  is needed.
- A NaN entry in a `demodulate` result means the quantity is unavailable for
  that configuration (e.g. `d2phi_df2` / detuning with too few points or no
  `linewidth_hz`). Results also carry `revision` (shape `(n_cycles,)`) mapping
  each cycle to its config revision.

**Robustness**

- **Dropped packets.** `group_cycles(..., on_missing='fill')` rebuilds the
  stream with NaN placeholders for missing accumulations (their point/settling
  inferred from the deterministic cadence) so affected cycles still appear;
  `'notify'` (default) warns and drops the incomplete cycles. A warning is
  always raised when packets are missing.
- **Live updates are seamless while in coverage.** Small moves ride the
  filterbank overlap with no dropped data. A move beyond coverage is rejected
  (fw: `force=True` or disable + enable; sw: `on_map_change='recenter'`); use
  `recenter_modulation()` for a deliberate (brief) channel-map reload.
- **Keep the probe delta small** (near the high-slope inflection) to minimise
  the sensitivity penalty — see [Sensitivity](#sensitivity).
- **Sync knobs (software engine).** `setup_sync=True` (default) pulses one
  firmware sync when modulation is armed to establish the TX/RX phase
  reference. `autosync=True` (default) additionally pulses firmware sync after
  each modulation buffer flip. Use `setup_sync=True, autosync=False` to test
  continuous per-point buffer switching after a single initial sync.

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

One engine-agnostic surface; the `engine` argument picks firmware-slot
(default) or software. Client methods map to server requests (mock-mode
supported) and return the server ack (`{'status': 'success'|'error', ...}`).

### `enable_modulation(center=None, offsets=None, mod_indices=None, samples_per_point=4, n_settle=None, engine='fw', mode='auto', compensate_rx_ticks=0, force=False, **sw_kwargs)`
Enable modulation (default `engine='fw'`).
- `offsets` — probe offsets (Hz). **fw:** per **slot** (auto mode needs exactly 4
  rows). **sw:** per **point**. Shape `(n_points,)` (broadcast across modulated
  tones) or `(n_points, len(mod_indices))` (per tone).
- `center` — per-tone centre RF freqs (Hz); `None` uses the current comb. A no-args
  call re-arms the resident config.
- `mod_indices` — tones to modulate; `None` = all regular (resonator) tones; blind
  indices error.
- `samples_per_point` — accumulations per slot/point (the fw `n_dwell`); default 4.
- `n_settle` — leading settling accumulations per slot/point (default 0 fw, 1 sw).
- `mode` (**fw**) — `'auto'` (firmware round-robin) or `'manual'` (software-selected slot).
- `compensate_rx_ticks`, `force` — RX path-delay compensation; arm despite `needs_recenter`.
- **sw-only** kwargs: `autosync`, `setup_sync`, `mrst`, `setup_mrst`, `buffer_reuse_delay_accs`.
- Result: `{'revision', ..., 'needs_recenter'}` (fw also reports `mode`/`n_slots`/`n_dwell`).

### `update_modulation(offsets=None, center=None, samples_per_point=None, n_settle=None, engine=None, mode=None, force=False, ...)`
Seamless live update (no dropped data), riding the armed bins. `engine=None`
updates whichever engine is active. **fw:** the new combs are loaded into the
inactive ping-pong buffer and flipped in one step. **sw:** the per-point words are
swapped in place. A move beyond bin coverage is rejected (fw: `force=True` or
disable+enable; sw: `on_map_change='recenter'`).

### `set_modulation_slot(slot)`  — firmware-slot, manual mode
Select the live LO slot; it goes live after the next accumulation. `slot` must be a
loaded slot; switches a running `auto` config to `manual`.

### `disable_modulation(engine=None)`
Disable modulation; tones rest at their centres. `engine=None` disables whichever is
active; the config stays resident for a fast re-arm. (Use `disable_stream()` to stop
output entirely.)

### `get_modulation_state()`
Return the unified `get_info('modulation')` section — an `engine` field
(`'fw'`/`'sw'`/`None`) plus the active engine's state (see
[schema](#modulation-state-schema)). Pure server-side read; drop-in as the
`group_cycles` state for either engine.

### `recenter_modulation(center=None, offsets=None, engine=None, ...)`
Reload the channel maps with **fresh** armed bins and re-apply (a deliberate brief
break) — the one case the seamless in-bin `update_modulation` can't cover, because
the bin routing lives outside the ping-pong buffers. `engine=None` recenters the
active engine (fw or sw). **fw:** optional `center` / `offsets` /
`samples_per_point` / `n_settle` / `mode` recenter straight onto new values an
update rejected. **sw:** recenters the current config (`autosync` / `mrst` /
`buffer_reuse_delay_accs` overrides).

### `purge_modulation_revisions()`  — software engine
Clears the software-engine revision history.

### Acquisition (existing methods, modulation-aware)
- `enable_stream()` / `disable_stream()` — continuous output on/off. Stopping a
  continuous modulated stream rests tones at their centres but leaves modulation
  armed, so a following `get_samples()` is still modulated.
- `get_samples(num_samples, burst=False)` — finite capture; returns tagged frames
  when modulating (aligned so sample 0 is point 1 of a cycle; `burst=True` rejected).
- `parse_samples(raw)` — adds `modulation_point` / `modulation_settling` /
  `modulation_revision`.

Server requests: `enable_modulation` / `enable_fw_modulation`, `update_modulation` /
`update_fw_modulation`, `recenter_modulation` / `recenter_fw_modulation`,
`set_fw_modulation_slot`, `disable_modulation` / `disable_fw_modulation`, plus
`get_info('modulation')`. Status dicts (`health_check`, server info) include
`modulation_streaming`.

---

## Firmware-slot modulation (v7.11)

Firmware v7.11 gives the mixer **four LO buffers ("slots")** per ping-pong
segment and switches between them **in hardware**, latched to accumulation
edges. Instead of the software engine's per-accumulation buffer stepping, you
load up to four full combs (centre + a per-slot offset) once and let the
firmware cycle them. The benefits:

- **No software timing jitter.** Slot switches are driven by the fabric, not by
  software writes, and land cleanly on accumulation boundaries. (v7.11 also
  aligns the ping-pong buffer switches to accumulation edges, so the software
  engine's flips are cleaner too.) The residual transient is the ~8-raw-spectra
  filterbank smear described in [How it works](#how-it-works), so
  `n_settle` ∈ {0, 1} is normally sufficient.
- **Two switching modes:**
  - **auto** — the firmware round-robins **all four slots**, advancing every
    `n_dwell` accumulations (`mixer.set_slot_auto_mode` + `set_dwell_accs`, latched
    by a sync). Firmware-deterministic timing.
  - **manual** — software selects the live slot with `set_modulation_slot(n)`;
    the chosen slot goes live **after the next complete accumulation**
    (`mixer.set_slot_manual_mode` + `set_manual_slot`). Software-timed, still
    glitch-free on the accumulation boundary.

Each accumulation is stamped by the firmware with its **live slot** (read from
the accumulator `buffer_id` register). The server writes that slot into the same
`flag5` tag as software modulation, as **`modulation_point = slot + 1`** (so slot
0 → point 1; 0 stays reserved for "no modulation"). Downstream, a firmware slot is
therefore treated exactly like a software-mod point — `parse_samples`,
`group_cycles` and `demodulate` need no special-casing (map back with
`slot = modulation_point − 1`).

Because both engines share the one tag word, firmware-slot and software modulation
are **mutually exclusive**: `enable_modulation` is rejected while software
modulation is armed, and vice-versa. Loading the slot combs rewrites channel maps
/ control buffers, so continuous streaming must be disabled first.

### Usage

```python
# --- Firmware-slot modulation: enable_modulation (engine='fw' is the default) ---
# Auto round-robin over 4 slots, dwelling 4 accumulations each, flagging the
# first accumulation after every switch as settling.
client.disable_stream()                              # loading rewrites chanmaps
client.enable_modulation(                            # engine='fw' is the default
    center=cfg['center'],                            # None uses the live comb
    offsets=[-1000.0, 0.0, +1000.0, 0.0],            # per-slot offset (Hz); 4 rows
    mod_indices=cfg['mod_indices'],
    samples_per_point=4, n_settle=1, mode='auto')    # dwell 4 accs/slot; starts immediately
raw   = client.get_samples(3000)                     # finite, slot-tagged capture
# or: client.enable_stream()                         # continuous, slot-tagged stream
data  = client.parse_samples(raw)                    # modulation_point = slot + 1
state = client.get_modulation_state()                # drop-in for group_cycles
grouped = mod.group_cycles(data, state)              # a slot groups like a point
result  = mod.demodulate(grouped, calibration=cfg['calibration'])
client.disable_modulation()                          # tones rest at the centre comb

# Manual switching: load slots, then step them under software control.
client.enable_modulation(offsets=[0.0, delta_hz], samples_per_point=4, mode='manual')
client.set_modulation_slot(1)                        # slot 1 live after next accumulation
```

Usage notes:

- **Starts immediately.** Firmware-slot `enable_modulation` loads the slots **and**
  starts the firmware switching (unlike the software engine, which only arms and
  waits for `enable_stream` / `get_samples`). Follow with `get_samples` (finite)
  or `enable_stream` (continuous).
- **Auto cycles all four slots.** The firmware has no "active slot count" — auto
  mode always round-robins all `mixer.n_slots` (4) slots, so it **requires exactly
  four offset rows** and rejects fewer (loading fewer would leave the unwritten
  slots emitting stale combs). Repeat a value to reuse a frequency, e.g.
  `[f1, f2, f3, f2]`. Use `mode='manual'` to load and step fewer slots yourself.
- **`offsets` shape** is per **slot**: `(n_slots,)` (broadcast across modulated
  tones) or `(n_slots, len(mod_indices))` (per tone). Slot `i` holds the comb at
  `center + offsets[i]`.
- **`n_settle` flags the switch transient.** The server detects each switch as a
  change in the per-accumulation slot tag and flags the leading `n_settle`
  accumulations of each dwell as `modulation_settling` (must satisfy
  `0 <= n_settle < n_dwell`). `group_cycles(..., include_settling=False)` drops
  them, exactly as for software modulation. 0 or 1 is normally enough — the
  transient is well under one accumulation (see [How it works](#how-it-works)).
- **Demod is shared.** `get_modulation_state()` carries `num_points` (=
  `n_slots`), `samples_per_point` (= `n_dwell`), `n_settle` and per-tone
  `offsets_hz` aliases, so it is a drop-in `tone_modulation_state` for
  `group_cycles`/`demodulate`. The auto round-robin's monotonic slot order
  (0→1→2→3→0) is what `group_cycles` uses to detect cycle boundaries; a static
  or arbitrarily-stepped **manual** slot is better analysed by filtering on the
  slot tag directly.
- **Seamless reconfigure.** `update_modulation(offsets=…, center=…)` swaps the
  combs glitch-free via the inactive ping-pong buffer (must stay within the armed
  bins). A move beyond coverage needs a channel-map reload — `recenter_modulation(
  offsets=…, center=…)` (a brief break). An auto↔manual mode change needs
  disable + enable (the resident config re-arms quick).

### Control API (firmware-slot)

- **`enable_modulation(center=None, offsets=None, mod_indices=None, samples_per_point=4, n_settle=0, engine='fw', mode='auto', compensate_rx_ticks=0, force=False)`**
  — with `engine='fw'` (the default): load the slot combs and start switching.
  `offsets` is per-slot (see above); `samples_per_point` is the per-slot dwell
  (`n_dwell`); `mode='auto'` needs exactly `mixer.n_slots` (4) rows, `mode='manual'`
  accepts 1..4. No-args re-arms a resident config. Rejected while software
  modulation is armed or a stream is running. Result:
  `{'revision', 'mode', 'n_slots', 'n_dwell', 'n_settle', 'needs_recenter', ...}`.
- **`set_modulation_slot(slot)`** — manual mode: select the live slot (active
  after the next accumulation). `slot` must be one of the loaded slots
  (`0..n_slots-1`); switches a running auto config to manual.
- **`disable_modulation()`** — stop switching; return the mixer to slot 0 and
  rest the tones at the centre comb. The config stays resident for a fast re-arm.
- **`get_modulation_state()`** — the `get_info('modulation')` section
  (schema below). Pure server-side read; safe to poll while streaming.

Server requests: `enable_modulation`, `set_modulation_slot`,
`disable_modulation`, plus `get_info('modulation')`. A compact hint
(`enabled`/`mode`/`n_slots`/`n_dwell`) also rides `get_info('tones')`.

### `fw_modulation` state schema

`get_info('modulation')` / `get_modulation_state()`:

```python
{
  'enabled': bool,                      # firmware-slot-enabled event state
  'mode': 'auto'|'manual',
  'applied_revision': int,
  'n_slots': int, 'n_dwell': int,
  'num_points': int,                    # == n_slots (group_cycles alias)
  'samples_per_point': int,             # == n_dwell (group_cycles alias)
  'n_settle': int,                      # leading settling accumulations per dwell
  'sample_rate_hz': float, 'cycle_rate_hz': float,   # cycle = n_slots * n_dwell
  'mod_indices': [int, ...],            # user-facing
  'needs_recenter': bool,
  'tones_beyond_coverage': [int, ...],
  'tones': [
    {'index': int, 'firmware_index': int, 'center_hz': float,
     'slot_offsets_hz': [float, ...],   # per slot
     'offsets_hz': [float, ...],        # alias of slot_offsets_hz (group_cycles)
     'occupancy': ['nearest'|'second'|'beyond', ...]},  # per slot
    ...
  ],
}
```

### Status (firmware-slot)

- Data plane (slot load, auto/manual switching, `flag5` slot tag, `n_settle`
  edge detection, seamless `update_modulation`, demod reuse) implemented and
  **hardware-validated**: the `buffer_id` slot read-back, deterministic
  edge-aligned switching (uniform dwells), and point-1 capture alignment all
  confirmed on the board.
- **Seamless update** (`update_modulation`, fw): the new combs are loaded into
  the inactive ping-pong buffer and flipped in one step — glitch-free, no sync,
  the LO phase rides. Only the mixer LO combs are ping-pong buffered, not the
  channel maps, so a move that stays in the armed bins is seamless; a move beyond
  them re-routes a tone's FFT bin and needs `recenter_modulation` (a brief break).
- `compensate_rx_ticks` mirrors the software-mod RX path-delay knob. The firmware
  PR ships `test_scripts/check_lo_switching.py` as a reference switch-timing check.

---

## Software modulation

Software modulation was the **first** frequency-modulation engine, built before
the v7.11 firmware slots existed. It is still fully supported and is the right
choice when you need **more than four probe points** (firmware-slot is capped at
`mixer.n_slots` = 4) or want the seamless per-point live update; otherwise prefer
firmware-slot. Select it with `engine='sw'`.

How it differs from firmware-slot:

- **Software-stepped, not firmware-cycled.** The server holds the two mixer
  ping-pong LO control buffers and, on the streaming hot loop, writes the next
  point's frequency words into the inactive buffer, flips the active index, and
  moves on — one accumulation at a time. There is no slot round-robin; the "point"
  is a software construct. See the
  [ping-pong scheduler](#double-buffer-ping-pong-scheduler-modulationscheduler).
- **Arm ≠ start.** `enable_modulation(engine='sw', …)` only *arms* (loads channel
  maps + control buffers); output begins with `enable_stream` (continuous) or
  `get_samples` (finite). Firmware-slot, by contrast, starts switching immediately.
- **Extra knobs:** `samples_per_point` / `n_settle` (dwell + settling), the sync
  controls `autosync` / `setup_sync` / `mrst` / `setup_mrst`, and
  `buffer_reuse_delay_accs` (a control-buffer rewrite-timing guard for N>2). It
  also supports `recenter_modulation()` and `purge_modulation_revisions()`.
- **Same downstream data.** Frames carry the same `flag5` tag (`modulation_point`
  1..N), so `parse_samples` / `group_cycles` / `demodulate` and
  `get_modulation_state()` are identical to firmware-slot.

Everything else about the software engine — armed bins / overlap riding, the
seamless live-update mechanism, and de-embedding — is in [Internals](#internals).

---

## Demodulation API

`souk_readout_tools.modulation` — pure functions (arrays/dicts in, arrays out; no
client/socket dependency, relocatable server-side). Same for both engines.

The pipeline is `parse_samples` → `group_cycles` → **one of two demodulators**:

- **`demodulate`** — one value **per modulation cycle**: `(n_cycles, n_tones)`
  arrays (`freq_shift_hz`, `dphi_df`, `dissipation`, …). Use it for
  operating-point tracking and slow readout, where one estimate per cycle
  (cycle rate ≈ `sample_rate / (N · samples_per_point)`) is what you want.
- **`demodulate_timestream`** — **sample-aligned**: `(n_samples, n_tones)` arrays
  at the full accumulation rate (every probe point demodulated with the local
  slope/reference), so you keep all the integration time. Use it for noise/PSDs
  and when each point is a useful measurement. Pair it with
  `group_cycles(..., reduce=None)`; its output plugs straight into the
  `plot_timestream*` helpers (including `plot_timestream_psd`).

### `params_from_sweep(sweep, *, n_points=None, point_sequence=None, offset_linewidths=None, samples_per_point=1, n_settle=1, delta_linewidths=0.1, exclude_blind=True, blind_indices=None, deembed=True, fits=None)`
Turn a sweep + (your own) fits into an `enable_modulation` config. **This package does not fit resonators** — fit the sweep yourself so the fitter's options stay out of the modulation API.
- `sweep` — parsed client sweep dict with `sweep_f`, `sweep_i`, `sweep_q`
  arrays of shape `(n_sweep_points, n_tones)`. Compact `f`, `z` arrays are
  also accepted. Blind-tone indices are inferred from standard metadata.
- `deembed=True` (default): build a per-tone `ResonatorCalibration` for the centred basis (see [De-embedding](#de-embedding-and-the-centred-basis)). **Requires `fits`** — raises if not supplied.
- `fits` — pre-computed per-tone fits (required when `deembed=True`). Pass
  `fitting.batch_fit(sweep)` results directly, an index-aligned sequence, or
  `{tone_index: fit}`. Each entry is a `fitting.FitResult` or a ready
  `ResonatorCalibration`; saved `FitResult.tone_index` values are honoured so
  skipped blind tones retain their indices. Missing or unsuccessful fits fall
  back to the model-free estimate.
- `deembed=False`: **model-free estimate** from the sweep (centre = measured peak `|dS21/df|`; linewidth = magnitude-dip FWHM in linear power, ≈ `fr/Ql`); no calibration. Do not pass `fits`.
- **Centre = geometric steepest point, not `fr`.** The operating point is the
  frequency of maximum frequency-shift responsivity (`argmax |dS21/df|` of the
  de-embedded response) — the best-SNR point for reading a resonator frequency
  shift. It is displaced from the fitted `fr` by the impedance-mismatch
  asymmetry (`phi`) and, for a driven nonlinear fit, by the Duffing detuning plus
  the bistable-cliff skew (cable delay is excluded — it does not move with the
  resonator). Linear: closed form `fr·(1 − Im(1/Qe)/2)`. Nonlinear: located on
  the smooth Duffing drive coordinate (the forward solver is discontinuous in
  `f`). Because the demod's `freq_shift_hz` is referenced to `fr`, it reads
  `center − fr` (see `reference_freqs['offset_from_fr_hz']`) as a constant
  baseline at the operating point, not ~0; subtract it in tracking/`needs_update`.
- Probe pattern: symmetric base `linspace(-1, 1, n_points)` scaled by
  `delta_linewidths · linewidth` per tone. Default `n_points` is 3. Pass
  `point_sequence` as 1-based base-point indices to repeat/reorder the cycle,
  e.g. `point_sequence=(1, 2, 3, 2)` gives `[-delta, 0, +delta, 0]`; when
  `n_points` is omitted it is inferred from the maximum sequence index.
- Direct pattern: pass `offset_linewidths` to specify exact offsets in linewidth
  units, bypassing `n_points`, `point_sequence`, and `delta_linewidths`; e.g.
  `offset_linewidths=(-0.25, 0, 0.25, 0)` gives
  `[-0.25·linewidth, 0, +0.25·linewidth, 0]` for every modulated tone.
- Returns `{'center', 'offsets' (n_cycle_points, n_mod), 'mod_indices',
  'point_sequence', 'offset_linewidths', 'samples_per_point', 'n_settle',
  'linewidth_hz' (n_mod), 'calibration' {tone: ResonatorCalibration},
  'reference_freqs', 'summary'}`.
- `reference_freqs` — dict of `(n_tones,)` arrays for cross-checking the operating
  point against the fit and the raw sweep: `center_hz` (chosen steepest point),
  `fitted_fr_hz` (fitted `fr`; NaN where no fit), `min_s21_hz`, `max_dz_df_hz`,
  `max_dphase_df_hz` (measured magnitude-min and peak `|dS21/df|` / phase-slope
  frequencies), and `offset_from_fr_hz` (`center − fitted_fr`). For any resonator
  with `phi ≠ 0` or `anl ≠ 0` these features sit at several distinct
  frequencies, none of them exactly `fr`.

### `group_cycles(data_dict, tone_modulation_state, reduce='mean', on_missing='notify', include_settling=False)`
Group parsed samples by point and cycle.
- Drops `settling` samples by default; aligns by `packet_counter`; cycle boundary detected on point wrap.
- Per-(point, tone) offsets and absolute frequencies read from `tone_modulation_state['tones'][i]`.
- `reduce='mean'` → `z` shape `(n_cycles, N, n_tones)`; `reduce=None` → `(n_cycles, N, n_used, n_tones)`.
- `on_missing` — gap handling on `packet_counter` (dropped accumulations). Always warns when packets are missing. `'notify'` (default): proceed (cycles straddling a gap are dropped as incomplete). `'fill'`: rebuild on a contiguous counter axis with **NaN-IQ placeholders** for the missing packets (their point/settling inferred from the deterministic cadence), so affected cycles still appear with NaN where data was lost.
- `include_settling=True` keeps samples flagged with `modulation_settling==1` in the grouped `z`/metadata arrays instead of dropping them. The flag is preserved.
- Returns `{'z', 'offsets_hz' (N, n_tones), 'freq_hz' (N, n_tones), 'revision' (n_cycles,), 'point_order'}` plus tone-role metadata (`blind_indices`, `regular_indices`, `mod_indices`, `modulated_indices`, `unmodulated_indices`, masks).

### `demodulate(grouped, offsets=None, *, method='fast', linewidth_hz=None, calibration=None, phase_baseline=None, threshold_linewidths=0.1)`
Per-cycle, per-tone demodulation in one of two bases (see [De-embedding](#de-embedding-and-the-centred-basis)). Returns arrays of shape `(n_cycles, n_tones)`:

| key | model-free (no `calibration`) | calibrated / centred (`calibration` given) |
|-----|------------|------------|
| `dphi_df` / `d2phi_df2` | slope/curvature of the **raw** I/Q phase (d2 NaN if N<3) | slope/curvature of the **centred** phase (well conditioned) |
| `freq_shift_hz` | `(φ_center − baseline) / dphi_df` | centre probe's offset from fitted resonance (Hz), exact fitted-model inversion |
| `detuning_linewidths` | `−(d²φ/df² ÷ dφ/df)·linewidth/8`; needs `linewidth_hz`, N≥3 | `freq_shift_hz / (fr/Ql)` from the calibration (no `linewidth_hz` needed) |
| `detuning_hz` | `detuning_linewidths · linewidth_hz` | centre offset from fitted resonance (Hz) |
| `needs_update` | `|detuning_linewidths| > threshold_linewidths` | same, from the calibrated detuning |
| `dissipation` | NaN | matched-scale `Delta(1 / (2 * Qi))`, per cycle |
| `z_center` | complex value at the centre probe point | (same) |

- `method`: `'fast'` (finite differences), `'accurate'` (weighted polynomial fit; handles asymmetric offsets), `'model'` (shares the `accurate` path).
- `calibration` — `{tone_index: ResonatorCalibration}` (from `params_from_sweep(..., deembed=True)['calibration']`). When present for a tone, that tone is de-embedded + phase-centred and the Möbius inversion gives probe detuning `freq_shift_hz = f_probe - f_r` plus matched-scale dissipation. The linear asymmetric notch inversion is exact. For the fitted Duffing model, an analytic inverse maps the recovered circle coordinate back to probe detuning.
- `linewidth_hz` — per-tone linewidth for the model-free detuning (ignored when a `calibration` is supplied).
- `phase_baseline` — per-tone reference phase for the model-free `freq_shift_hz`; `None` uses the per-tone mean centre-point phase.

### `demodulate_timestream(grouped, *, method='accurate', calibration=None, reference_z=None, data_dict=None, n_samples=None, fill_settling=True, use_settling_for_fit=False, threshold_linewidths=0.1)`
Sample-aligned demodulation for the case where every modulation point is a useful
resonator probe and you do not want to lose integration time. Use it with
`group_cycles(..., reduce=None)` so the individual sample indices are retained:

```python
grouped = mod.group_cycles(data, state, reduce=None, include_settling=True)
ts = mod.demodulate_timestream(grouped, data_dict=data)
```

Returns `(n_samples, n_tones)` arrays such as `freq_shift_hz`,
`resonance_shift_hz`, `z_demod`, `mag`, `phase_rad`, `phase_unwrapped_rad`,
`dphi_df`, `center_dphi_df`, `d2phi_df2`, and `dissipation`. Without a
calibration, `dissipation` uses the same local linearized tangent/normal
projection as parsed-sample `format='freq_diss'` plotting, with the modulation
points supplying the local IQ gradient. Prefer
`include_settling=True` at grouping time when the settling samples should be
part of the demodulated stream; by default they are excluded from the
per-point reference/slope fit (`use_settling_for_fit=False`) but included in
the returned sample-aligned arrays. `fill_settling=True` / `'raw'` demodulates
the settling IQ with the completed cycle's reference/slope. Use
`fill_settling='interpolate'` to replace settling sample values by linear
interpolation between non-settling demodulated samples while preserving
`modulation_settling=1`. `fill_settling=False` / `'none'` leaves dropped
settling samples as NaN. Incomplete cycles and missing packets remain NaN. The
default `method='accurate'` handles repeated centre points like
`[-1000, 0, +1000, 0]`.

The returned dict also carries parsed-timestream-compatible `i_data` / `q_data`
views of `z_demod`, plus `sample_rate`, `packet_counter`, `info`, and related
metadata when `data_dict` is supplied. It preserves parsed tone-role metadata
(`regular_indices`, `blind_indices`) in `info['tones']` / `tone_metadata` and
adds modulation-role metadata (`mod_indices`, `modulated_indices`,
`unmodulated_indices`, `tone_is_blind`, `tone_is_modulated`), so blind tones
remain available for common-mode cleaning without being treated as demodulated
resonators. That means it can be passed directly to the normal plotting helpers:

```python
plot_timestream(ts, format='magphase')
plot_timestream(ts, format='freq_diss')        # uses precomputed demod output
plot_timestream_psd(ts, format='freq_diss')    # no sweep needed
plot_timestream_on_resonance(ts, sweep, tone_index=0)
```

---

## `modulation` state schema

`get_info('modulation')` / `get_modulation_state()`:

```python
{
  'enabled': bool,                      # modulation-enabled event state
  'desired_revision': int,              # last requested config revision
  'applied_revision': int,              # revision actually applied by the producer
  'pending_op': 'enable'|'update'|'recenter',  # optional, awaiting first producer frame
  'revision_history': {rev: {'center', 'offsets', 'mod_indices', 'autosync', 'ts'}},
  'num_points': int, 'samples_per_point': int, 'n_settle': int,
  'autosync': bool,
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

The **shared** machinery applies to both engines: the single stream producer
below, armed bins / overlap riding, de-embedding, and the demod definitions.
Every frame's tag is read from the accumulator `buffer_id` register — the
firmware's ground-truth report of which ping-pong buffer (software engine) or LO
slot (firmware-slot engine) produced that accumulation — so tag and data stay
consistent across the v7.11 edge-aligned buffer switch.

The **software-engine-specific** internals are the *ping-pong scheduler* and the
*software live-update* path (both flagged below). The **firmware-slot engine**'s
internals are simpler and live in
[Firmware-slot modulation](#firmware-slot-modulation-v711): load up to four slot
combs once (sharing the same armed bins), let the firmware round-robin them, read
the live slot from `buffer_id`, and update seamlessly by loading the *inactive*
ping-pong buffer and flipping to it in one step.

### One streamer, modulation as a mode
A single `stream_data` coroutine produces all stream frames, gated by two
events: `e_stream_enabled` (output on/off) and `e_modulation_enabled` (modulate
vs plain). Normal streaming is the modulation-off case. `get_samples` shares the
same stepping engine for finite captures; an acquisition-owner lock ensures a
single producer drives the hardware.

### Double-buffer ping-pong scheduler (`ModulationScheduler`)
**(Software engine.)** The firmware has two LO control buffers. The scheduler writes the next point's
words into the **inactive** buffer, flips the active index, and optionally
pulses per-step sync; the vacated buffer then receives the following point
during the dwell. At arm, it writes phase offsets once into both buffers and can
pulse one setup sync before cycling. Rules:
- Skip the write when the inactive buffer already holds the wanted point. For
  **N=2** the two points stay resident in the two buffers, so each switch is an
  index flip + optional sync only (no per-switch write).
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
Each point's control words also carry the filterbank rolloff compensation —
TX scaling restores the drive at that point's drift, RX scaling/phase flatten
the readout ([filterbank_compensation.md](filterbank_compensation.md)); keep
base tone amplitudes ~1 dB below full scale so the TX boost has headroom.

### Live update mechanism
**(Software engine;** the firmware-slot seamless update is the inactive-buffer
flip described above.) The request handler prepares the bundle off the hot path (thread executor,
including occupancy classification) and posts it to a single latest-wins slot.
The producer applies it at the next cycle boundary (`_apply_pending_modulation_command`)
and is the sole owner of modulation hardware writes — no race with an in-flight
cycle. A same-maps update swaps the per-point words in place (no map rewrite, no
reset); the new frequencies take effect within one cycle. The revision is bumped
and the full revision→config history retained for offline analysis.

### De-embedding and the centred basis
Demodulating the **raw** I/Q phase is poorly conditioned: the measured phase
still contains the cable-delay ramp `−2π f τ` (which adds a spurious, non-science
slope across the probe offsets) and the resonance circle is offset from the
origin and rotated, so the "phase" is not referenced to resonance. The
de-embedded/phase-centred basis fixes this.

Fit each parsed sweep with `fitting.batch_fit(sweep)` and pass the results as
`params_from_sweep(..., fits=...)`; it builds a per-tone
`resonator.ResonatorCalibration` (`from_fit`) — which stores the cable delay,
gain, circle centre/radius and rotation, plus `fr`/`Ql` — without re-fitting.
When that calibration
is passed to `demodulate`, each probe point is transformed with
`ResonatorCalibration.transform_raw_iq(freq_hz, z)` (cable delay removed **at the
point's absolute frequency**, then centred + rotated). In this basis:
- the centred phase is monotonic through resonance and well conditioned, so
  `dφ/df` / `d²φ/df²` are clean;
- the exact **Möbius inversion** `convert_centered_iq` gives the frequency
  shift and matched-scale `Delta(1 / (2 * Qi))` dissipation for the linear
  notch model. Its native frequency coordinate is probe detuning
  `f_probe - f_r`; resonator detuning relative to the probe has the opposite
  sign. The inversion is
  valid for arbitrary detuning — no small-signal or linewidth assumption. For
  the fitted Duffing model it analytically maps the recovered driven-circle
  coordinate back to probe detuning.

The diagnostic `method='circle'` path exposes the raw radial loss proxy
`abs(z_centered) / radius - 1`. For the full derivation, its normalization,
and the matched-scale `Delta(1 / (2 * Qi))` convention used for
frequency-versus-dissipation noise overlays, see
[`resonator_math_derivations.ipynb`](resonator_math_derivations.ipynb).

This reuses the existing `resonator.py` / `fitting.py` machinery; the modulation
demod just supplies the per-point IQ at known absolute frequencies.

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

## Status and limitations

- Data plane (tagging, decode, scheduler bookkeeping, mock, demod, parameter
  derivation) implemented and verified in mock mode; the firmware-slot data
  plane is additionally hardware-validated (see
  [Status (firmware-slot)](#status-firmware-slot)).
- **Revision persistence** into the raw-stream sidecar / G3 metadata is a
  follow-up; `revision_history` in the `modulation` state currently suffices to map a
  frame's revision to its config offline.
- The **automated tracking loop** (acting on `needs_update`) is not implemented;
  the API provides the required inputs and the seamless `update_modulation` path.
