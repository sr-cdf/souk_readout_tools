# Tone tracking: server-side FFM auto-recentering

With fast frequency modulation (FFM) running, resonances drift with loading
and the probe tones slowly walk off their operating points. **Tracking** is
a server-side loop that watches the demodulated detuning of every modulated
tone and issues small, glitch-free centre corrections to keep each tone at
its operating point — and announces every tone change (from this loop *or*
from any client command) **in-band on the stream socket** as a typed
"tone update" frame, so clients log tone provenance as it happens.

Tracking is permanent readout infrastructure — live frequency-shift serving
and array health monitoring are other natural consumers of the same
demodulation/estimation stage. It is distinct from the sweep-based `retune`
coroutine (which measures with a sweep and re-parks tones); tracking runs
continuously on the modulated stream itself.

Everything is **off by default**: with tracking disabled the server's
behaviour is unchanged, and stream clients that do not opt in receive a
byte-identical stream.

Implementation: `souk_readout_tools/server/tracking.py` (estimator,
controller, runtime — numpy-only above the runtime, unit-testable off the
RFSoC) with minimal wiring in `readout_server.py`. Tests:
`tests/test_tracking.py`. Benchmarks: `profiling/tracking_benchmark.py`.

## Provenance and reconstruction

Post-processing reconstructs the absolute per-tone response exactly:

```
absolute_response(t) = (center[rev(t)] − center[rev0]) + freq_shift_hz(t)
```

with `freq_shift_hz` in the **detector-side** sign (resonator motion,
`= −` `demodulate`'s probe-side `freq_shift_hz`; see
`doc/frequency_modulation.md`), where `rev(t)` is the revision stamped into
each frame's flag5 tag (bits 17–31) and `center[rev]` comes from the
revision history
(`get_info('modulation')['revision_history']`) or, equivalently, from the
logged `TONE_UPDATE` frames. The tracker may follow real optically-induced
shifts — that is acceptable **only because** every correction is
revision-logged; nothing is lost, the signal just moves from
`freq_shift_hz` into centre steps and is added back in reconstruction.

**Revision wrap**: the flag5 revision field is 15 bits. Long campaigns with
periodic commits will wrap it (`(rev + 1) & 0x7FFF`), and the wrapped key
silently *replaces* the old entry in `_modulation_revision_history` — so
over one wrap period the in-memory history stays unambiguous, but entries
older than one wrap are gone. For long campaigns rely on the append-only
`TONE_UPDATE` JSONL sidecar (timestamped, never overwritten) and purge the
server-side history between campaign segments with
`purge_modulation_revisions()` (refused while modulation is enabled; see
its docstring).

## Typed stream frames (protocol)

Backward compatibility is absolute: a client that does not opt in receives
a byte-identical stream (data frames + zero-length keepalives). This is
asserted byte-for-byte in `tests/test_tracking.py`.

- **Opt-in**: after connecting to the stream port, send one JSON line:
  `{"subscribe": ["tone_updates"]}\n`. The server (which previously never
  read from stream clients) polls for this line between keepalives.
- **Frame typing**: data frames keep the existing 4-byte big-endian length
  prefix unchanged (top byte 0x00 — frames are far below 16 MB). Typed
  frames set a nonzero type in the top byte with the payload length in the
  low 24 bits. Zero-length keepalives are unchanged.
  - `0x01 TONE_UPDATE`, `0x02 SNAPSHOT` — JSON payloads.
- **`SNAPSHOT`** (sent immediately on subscribe): current applied revision,
  modulation table (mod_indices, per-tone centres, offsets, dwell), and
  tracking status — late joiners synchronise without a request-port round
  trip.
- **`TONE_UPDATE`** (sent for **every** revision change, any origin — the
  hook is in `_apply_pending_modulation_command`, the single place
  revisions land): `{ts, op (enable/update/recenter/disable/rest/
  tracking_dry_run), revision, kind ('lo' | 'bin'), source ('client' |
  'tracking'), mod_indices, centers_hz {tone: Hz, changed tones only},
  warnings, expected_transient_cycles (bin only),
  triggering_detunings_linewidths (tracking only)}`.

Client support:

- `stream_dac.SocketFrameSource(subscribe_updates=True)` subscribes, parses
  typed frames and queues them; `StreamToDac` logs them to a JSONL sidecar
  (`<recording>.updates.jsonl`) next to the raw recording.
  `receive_stream.py` is untouched and keeps working.
- `receive_stream_g3(subscribe_tone_updates=True)` emits a G3 Wiring frame
  (update JSON in `'status'`, like the start-of-stream Wiring frame) when a
  `TONE_UPDATE` arrives mid-stream.

## Data path and performance

The stream rate is limited by the `stream_data` software loop (AXI
accumulator reads dominate), so the hot path gets exactly one addition:

```
stream_data (modulated branches, fw and sw, per sample):
    payload = prepare_frame(...)          # unchanged
    write to stream clients               # unchanged
    if tracking enabled: tracking.tap(payload)   # ONE deque append, O(1)
```

`tap` appends the already-built payload bytes to a bounded drop-oldest ring
(no copy, no parsing, no per-tone work). A slow consumer can never stall
the stream — the ring just drops oldest.

The consumer task polls the ring (`poll_interval_s`, default 0.2 s), and
runs parse → cycle assembly → estimation → filtering in the thread executor
(`to_thread`; the heavy numpy ops release the GIL), **vectorised across all
tones** — no Python loops over tones anywhere in the pipeline.

**Estimator**: `estimate_detunings` is a lean whole-array implementation of
`demodulate`'s model-free `method='fast'` maths (finite-difference phase
slope + curvature; detuning from the curvature/slope ratio, which crosses
zero at resonance independent of the unknown phase offset). A unit test
asserts numerical agreement with `demodulate` to 1e-10. `modulation.py`'s
analysis API is untouched. Settling-flagged samples are excluded; only
complete cycles are used; partial cycles carry across batches.

**Benchmarks** (dev machine x86, `profiling/tracking_benchmark.py`, 2048
tones, N=3 points, 2 samples/point, 500 Hz stream — re-run on the RFSoC ARM
quad-A53 for production numbers):

| stage | cost |
|---|---|
| tap (per frame, the only hot-path work) | 0.066 µs = 0.003 % of a 2 ms frame period |
| consumer batch (parse+assemble+estimate+filter) | 107 ms per 1.2 s of stream ≈ 9 % of one core |
| estimator alone | 0.39 ms per cycle, all 2048 tones |
| `demodulate` reference on same batch | 7× slower than the lean estimator |

The tap cost is measured to be negligible, so tracking adds no measurable
stream-rate regression; the estimation runs on a different core from the
stream loop. On the A53 expect the consumer batch to scale up roughly an
order of magnitude — still comfortably real-time at 2048 tones, and
`poll_interval_s` can be raised to trade latency for duty cycle.

**`get_samples` as producer**: tracking taps **only** `stream_data`.
Finite `get_samples` captures are not tapped (they are short and usually
interactive); tracking simply sees a gap. This is deliberate and can be
revisited later.

## Estimation → decision → staged commit (state machine)

```
                    stream_data ──tap──▶ ring (drop-oldest)
                                            │  consumer task (poll)
                                            ▼
                       parse → cycles → estimate (per-cycle detunings)
                                            │
                                    averaging filter        ← get_info shows
                                            │                 filtered values
              revision changed under us? ──▶ discard batch,
              (external command = back-off)  restart confirmation
                                            │
                          deadband (threshold_linewidths, filtered)
                                            │
                        K consecutive windows over? (confirm_windows)
                                            │
                 correction = clip(gain·detuning, ±max_step·lw)
                 classify:  |Δcenter| ≤ lo headroom → 'lo' else 'bin'
                                            │
                     staged sets (per class, accumulate between commits)
                        │ lo policy               │ bin policy
                        ▼                          ▼
                 commit 'lo' (seamless)     commit 'bin' (re-arm)
                 one revision per commit ──▶ TONE_UPDATE announced
```

### Averaging filter

Per-cycle detunings feed a selectable filter; **the filter output is what
all threshold logic sees**. Both the form and the window are runtime config
because the right window is science-dependent; `get_info('tracking')`
exposes the recent filtered detunings so the window can be tuned
empirically.

- `boxcar` of W cycles: frequency response `sinc(π f W / f_cyc)` — first
  null at `f_cyc / W`; attenuates the science band above that. Default
  W=10.
- `ewma` with time constant τ: single real pole, −3 dB at `1/(2πτ)`.
  Default τ=1 s.

No decisions are taken until the filter has seen a full window (`settled`).

### The two update classes

Every correction is classified per tone:

- **`lo`** — the new centre stays within the armed-bin coverage, so only
  the tone LO/mixer words change: the seamless in-place update path (fw:
  the inactive mixer slot-buffer flip of `update_fw_modulation`; sw:
  `update_modulation`'s `install_bundle` swap). No map writes, no re-arm.
- **`bin`** — the shift needs an FFT-bin / channel-map change (the
  `needs_recenter` condition): the `recenter`-style re-arm path — slower
  and briefly disruptive.

Classification uses the reported per-(point, tone) drift and the armed TX
bin width: headroom = `(1 − max|drift|) · bin_width` (fw states report
occupancy classes instead of numeric `drift_bins`, so a conservative
class bound is used). This is a pre-check only — the commit path
re-verifies against the freshly prepared bundle's `needs_recenter`, and a
`lo` commit that turns out to need a recenter applies **nothing** and
moves those tones to the `bin` staged set.
A mixed decision batch is split: `lo` tones commit seamlessly on their
policy; `bin` tones wait for theirs (or an explicit command) and never
block the `lo` set.

### Staged apply (pre-fill, then switch)

The staged-commit design separates *staging* (the slow AXI writes /
prepare) from *committing* (a cheap switch), and both engines already
provide exactly this. **fw engine** (default): the mixer LO slot combs are
ping-pong buffered — a `lo` commit prepares the new per-slot words in the
executor, loads them into the *inactive* buffer, and flips in one step
(`_update_fw_modulation`); the running slot cycle swaps combs glitch-free.
**sw engine**: the tone-control buffers are the same ping-pong pair the
`ModulationScheduler` flips for probe-point stepping, and resolving the
design against the code, the update machinery is the staged mechanism:

1. **Staging** = `_prepare_modulation(...)` in the thread executor. It is
   pure computation (hardware *reads* only, no writes) and produces the
   complete per-point control words for the new centres.
2. **Committing** = queueing `_pending_modulation` (latest-wins). The frame
   producer applies it at a cycle boundary with `install_bundle`, which
   swaps the prepared words in and *invalidates the buffer bookkeeping* —
   whereupon the scheduler's normal rotation (`advance` writes the inactive
   buffer during each dwell, honouring `buffer_reuse_delay_accs`) weaves
   the new words into the buffers over the following cycle. No buffer is
   ever written out of band — the one about to go live is never touched.

So commit latency is decoupled from write latency by construction on both
engines: the slow prepare happens ahead of the commit moment, and the
physical buffer writes either fill the inactive slot buffer (fw) or ride
the rotation that was happening anyway (sw). Corrections add **no**
tone-control buffer writes beyond what a normal
`update_fw_modulation`/`update_modulation` performs.

### Correction transients and flagging

- **`lo` commits** are seamless on both engines (fw: inactive slot-buffer
  flip; sw: `install_bundle` — the live buffer picks up the new
  words within one cycle). The sw buffer flips are the same edge-latched flips
  modulation performs every dwell, and frames are tagged from the firmware
  `buffer_id` read-back, so the revision tag tracks the data exactly. The
  first cycle after the revision edge mixes old- and new-centre points as
  the rotation rewrites the buffers — mask **one cycle after each revision
  edge** in strict pipelines (the `TONE_UPDATE` carries the revision to cut
  on). Verify on hardware whether any residual transient is visible (see
  `buffer_reuse_delay_accs`); if so, extend the settling flag over it.
- **`bin` commits** re-arm channel maps (recenter path): expect a brief
  disturbance. The `TONE_UPDATE` carries `kind: "bin"` and
  `expected_transient_cycles: 1`; consumers should mask that window plus
  the usual settling-flagged samples. Measure both windows on hardware and
  update this section with numbers.

### Back-off and interlocks

- If the applied revision changes under a batch (an external
  `update_modulation` landed), the batch is discarded, the filter and
  confirmation restart (`backoffs` counts these).
- Tracking holds automatically while the server `sweep()`/`retune()`
  coroutines run, while streaming is disabled, and while modulation is
  disabled; buffered data is dropped during a hold so stale cycles never
  feed a decision. `hold_tracking`/`resume_tracking` give manual control —
  hold around critical dwells where even seamless steps are unwanted.
- `dry_run` (default **True** on enable) runs the full
  estimate/decide/log/announce pipeline with no hardware applies.

## Closed-loop response (what tracking does to your science data)

Tracking is a slow, deadband-gated, quantised transfer of signal from
`freq_shift_hz` into centre steps. Within the deadband nothing happens and
`freq_shift_hz` is untouched. Once the *filtered* detuning exceeds the
threshold for K consecutive windows, a step of up to
`gain · detuning` (clamped to `max_step_linewidths`) moves the centre and
`freq_shift_hz` drops by the same amount. This is nonlinear (deadband +
quantisation + confirmation delay) — but **exactly recoverable** via the
revision history (see the reconstruction formula above), so it is a
bookkeeping transformation, not a filter on the science signal.

For residual-interference analysis: the loop's effective bandwidth is
bounded above by the averaging filter (boxcar sinc / EWMA pole, above)
cascaded with the confirmation delay (K decision windows) and the per-class
minimum commit interval. Choose the window/threshold/interval so that this
cascade is far below the science band:

- signals faster than `f_cyc/W` (boxcar) or `1/(2πτ)` (EWMA) are averaged
  away and can never trigger a step;
- the deadband means drifts smaller than `threshold_linewidths` are never
  tracked at all;
- commit intervals put a hard floor on the step cadence — steps appear in
  the data as revision-tagged edges, trivially cut or corrected.

If a residual effect matters, compute it from the logged step times/sizes
(all in the JSONL / revision history) rather than modelling the loop.

## Configuration reference (`enable_tracking` / server `TRACKING_DEFAULTS`)

| key | default | meaning |
|---|---|---|
| `linewidth_hz` | (from `enable_modulation`) | per-tone linewidths; **required** here or at `enable_modulation` (`params_from_sweep` output carries it) |
| `dry_run` | `True` | decide/log/announce only, no applies |
| `threshold_linewidths` | 0.3 | deadband on the filtered detuning (wider than `demodulate`'s 0.1 by design) |
| `confirm_windows` | 3 | consecutive decision windows over threshold |
| `gain` | 1.0 | `new_center = center − gain · detuning_hz` |
| `max_step_linewidths` | 0.5 | per-decision step clamp |
| `enable_mask` | all modulated | tones allowed corrections |
| `filter` | `'boxcar'` | `'boxcar'` or `'ewma'` |
| `filter_window` | 10 | boxcar length (cycles) |
| `filter_tau_s` | 1.0 | EWMA time constant |
| `poll_interval_s` | 0.2 | consumer poll period |
| `ring_depth` | ~2 s of frames | tap ring depth |
| `lo_auto_commit` | `True` | auto-commit seamless updates |
| `lo_min_commit_interval_s` | 5 | floor between lo commits |
| `bin_auto_commit` | `False` | bin re-arms staged only, applied on demand |
| `bin_min_commit_interval_s` | 30 | floor between bin commits |
| `commit_threshold_count` | off | commit a class when ≥ M tones staged |
| `commit_interval_s` | off | commit every T s if anything staged |
| `unlock_slope_ratio` | 0.25 | `unlocked` when phase slope drops below this fraction of its settled baseline |
| `unlock_invalid_fraction` | 0.5 | `unlocked` when this fraction of recent cycles are unusable |
| `unlock_detuning_linewidths` | off | also `unlocked` when the reading exceeds this (usually left off — see below) |

Commit policies combine — first to fire wins, gated by the per-class
minimum interval; with neither optional policy set, a class commits as soon
as its minimum interval allows. Decisions accumulate into the staged sets
between commits; **one commit = one revision** covering every tone changed
in it.

## Health monitoring

Telescope-control health polling reads tracking state two ways, both
covered by the standard `get_info` / `health_check` tooling:

- **`health_check()`** carries a compact `tracking` block (the
  `summary()` below), plus `resonators_tracking` (true only when tracking
  is enabled and actually applying — not dry-run) and `max_detuning_hz`.
  This is the lean, poll-often path.
- **`get_info('tracking')`** returns `{summary, tones, controller, …}` —
  the same summary block, **plus per-tone** health, plus the full
  controller/parameter detail. `client.get_tracking_state()` fetches it.

**Summary block** (`summary`) — array-wide counts and extrema, the block a
monitor watches:

| field | meaning |
|---|---|
| `n_tracked` | tones under tracking |
| `n_locked` / `n_drifting` / `n_recenter_pending` / `n_unlocked` / `n_no_data` | tones in each lock state (below) |
| `n_over_threshold` | filtered \|detuning\| past the deadband |
| `max_abs_detuning_linewidths`, `median_abs_detuning_linewidths` | drift extrema/centre, in linewidths |
| `max_abs_detuning_hz` | worst drift in Hz |
| `staged` | `{lo, bin}` corrections awaiting commit |
| `commits`, `backoffs` | applied corrections; batches discarded on external revisions |
| `filter_settled`, `held`, `hold_reason`, `dry_run`, `last_estimate_age_s` | loop liveness (a stale `last_estimate_age_s` means the loop has stopped estimating) |

**Per-tone health** (`tones[i]`) — `state` plus
`detuning_linewidths`, `detuning_std_linewidths` (running spread — a
noisy/unstable tone shows a large std at a small mean),
`slope_ratio` (current responsivity ÷ its on-resonance baseline),
`invalid_fraction`, and `staged_center_hz` if a correction is pending.

**Lock states**: `locked` (inside deadband), `drifting` (past threshold,
correcting/suppressed), `recenter_pending` (a `bin` correction staged),
`no_data` (nothing usable yet), and **`unlocked`** — the resonance is
likely lost and the loop would be tracking noise.

`unlocked` is deliberately **not** detected from a large detuning reading:
the model-free estimate *compresses* (≈ `x/2/(1+x²)`, saturating near 0.25
linewidths), so a resonance that has run far away reads a deceptively
*small* detuning. Instead it is caught by the **phase slope collapsing**
below `unlock_slope_ratio` of the on-resonance baseline (a flat phase far
from resonance has no responsivity) and/or by the invalid-cycle fraction
exceeding `unlock_invalid_fraction`. `unlock_detuning_linewidths` can
additionally trip on the reading, but is off by default for this reason.
So the monitor's "have we lost the resonance / are we tracking noise?"
question is answered by `n_unlocked` and each tone's `slope_ratio`, not by
the detuning magnitude alone.

## Commands

Server request port (and matching `ReadoutClient` methods):

- `enable_tracking(**params)` — start/reconfigure on whichever engine is
  armed, fw (the default engine; needs `mode='auto'`) or sw; ≥ 3
  slots/points (tracking needs the curvature estimate).
- `disable_tracking()` — stop; staged corrections are dropped.
- `hold_tracking()` / `resume_tracking()` — pause without teardown.
- `commit_tracking_updates(classes=None, tones=None)` — on-demand commit of
  the staged set, optionally filtered by class and/or tones.
- `get_info('tracking')` / `client.get_tracking_state()` — the health
  `summary`, per-tone `tones` health/lock states, and the full controller
  detail (dry_run, held and why, params, filtered detunings, staged sets
  per class, applied/suppressed counters, back-offs, commit timestamps,
  decision/applied revision, ring fill). See **Health monitoring** above.

Engines: tracking runs on **both** modulation engines and picks whichever
is armed — firmware-slot (`engine='fw'`, the default engine) or software
(`engine='sw'`). Frames are tagged identically (fw slots and sw points
share the flag5 field), so the tap/estimator/controller are engine-blind;
only the commit path differs:

- **fw**: `lo` commits mirror `update_fw_modulation` — the new slot combs
  load into the inactive mixer ping-pong buffer and flip in one step
  (seamless, `_update_fw_modulation`); `bin` commits mirror
  `recenter_fw_modulation` (fresh maps, `_apply_fw_modulation`). The
  firmware owns slot switching, so applies run in the executor exactly as
  the fw client handlers do. fw tracking needs `mode='auto'` (manual slot
  selection never completes cycles). fw states report occupancy classes
  rather than numeric drifts, so the `lo`/`bin` pre-classification uses a
  conservative occupancy bound — the commit-time `needs_recenter` re-check
  is the authority either way.
- **sw**: commits ride the frame producer's `_pending_modulation`
  ownership (epoch → prepare → queue → `install_bundle` at a cycle
  boundary), as described in the staged-apply section above.

`linewidth_hz` metadata and the typed-frame announcements likewise work on
both engines (fw revisions announce from the fw handlers and from tracking
commits; the SNAPSHOT reports whichever engine is active).

## Recipe: an FFM tracking session

**1. Sweep, fit, and build the modulation config.** `params_from_sweep`
picks each tone's operating point (max-SNR steepest point) and per-tone
probe spacing, and returns everything `enable_modulation` and tracking
need — including `linewidth_hz`:

```python
sweep = client.parse_sweep_data(client.get_sweep_data())
fits = fitting.batch_fit(sweep)
cfg = modulation.params_from_sweep(sweep, fits=fits)
print(cfg['summary'])
```

**2. Arm modulation (≥ 3 points/slots — tracking needs the curvature) and
stream.** The default engine is firmware-slot (`fw`); its `auto` mode
round-robins all 4 mixer LO slots, so ask `params_from_sweep` for a
4-entry pattern (e.g. `offset_linewidths=(-0.1, 0, 0.1, 0)` in step 1).
Note `disable_stream` before the initial arm; linewidths ride along as
metadata:

```python
client.disable_stream()
client.enable_modulation(center=cfg['center'],            # engine='fw' default
                         offsets=cfg['offsets'], mod_indices=cfg['mod_indices'],
                         samples_per_point=cfg['samples_per_point'],
                         n_settle=cfg['n_settle'],
                         linewidth_hz=cfg['linewidth_hz'])
client.enable_stream()
```

(`engine='sw'` works identically here and takes any number of points ≥ 3.)

**3. Start tracking in dry-run** (the default) and watch it before letting
it touch anything:

```python
client.enable_tracking()                       # dry_run=True
state = client.get_tracking_state()
state['filtered_detuning_linewidths']          # per-tone filtered detunings
state['controller']['staged']                  # what it WOULD correct
```

Tune here: `filter_window`/`filter_tau_s` against your science band (the
filter responses are above — keep the loop bandwidth well below the
signals you care about), `threshold_linewidths` against the quiet-sky
scatter of the filtered detunings, `confirm_windows` against glitch rates.
Dry-run decisions are also announced in-band (`op: tracking_dry_run`), so
a subscribed recorder captures the would-be behaviour for offline review.

**4. Enable corrections** once the dry-run behaviour looks right:

```python
client.enable_tracking(dry_run=False)          # lo auto-commits; bin stages
```

Seamless `lo` corrections now keep every tone at its operating point,
each one revision-stamped and announced. `bin`-class shifts (a tone
walking off its armed FFT bin) accumulate in the staged set — apply them
at a convenient boundary:

```python
client.get_tracking_state()['controller']['staged_counts']
client.commit_tracking_updates(classes='bin')  # brief, flagged transient
```

**5. Record with provenance.** Any consumer that must reconstruct
absolute response subscribes to the typed frames:
`receive_stream_g3(subscribe_tone_updates=True)` (Wiring frame per
update), or `SocketFrameSource(subscribe_updates=True)` /
`souk-stream-to-dac --mode ffm` (JSONL sidecar). Legacy
`souk-receive-stream` recordings still work — the revision tag is in every
frame's flag5, and `get_info('modulation')['revision_history']` maps
revisions to centres (mind the 15-bit wrap on long runs; the JSONL/G3
records are the wrap-proof source).

**6. Around critical dwells** (calibrator crossings, where even seamless
steps are unwanted): `client.hold_tracking()` … `client.resume_tracking()`.
Sweeps and retunes hold it automatically.

**7. Reconstruct offline.** For each tone, cut the timestream at the
revision edges (mask `expected_transient_cycles` after `bin` edges and the
settling flags), demodulate as usual, and apply

```
absolute_response(t) = (center[rev(t)] − center[rev0]) + freq_shift_det(t)
```

with the detector-side sign convention noted above. The result is
identical to what an untracked (but never-drifting) tone would have
measured.
