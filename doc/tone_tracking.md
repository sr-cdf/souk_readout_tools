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

The only work tracking adds to the per-accumulation path is one coherent add
of the I/Q words into a running average, while the read buffer is still hot:

```
prepare_frame (per accumulation):
    cnt, data, err, tt = read_accumulated_data_fast(...)   # unchanged
    payload = length prefix + frame words                  # unchanged
    if tracking enabled: tracking.tap(data, point, settling, revision)
```

`tap` adds the interleaved words into a per-(block, point) running average and
does nothing else: no frames are retained, so there is no ring, no backlog and
no drop policy.

**That placement is the design.** Handing raw frames to a consumer on another
core moves megabytes per second through the 1 MB L2 the four A53s share, which
costs accumulations on *both* pipelines — including one that is not tracking,
in a separate process. At equal duty cycle, a 16 MB working set on CPU0-1 took
a pipeline from 0.114% to 22% loss; a 32 kB one cost nothing
(`doc/v712-notes.md`).

**Averaging length and shape are configurable, and single-stage.** An integer
sum *is* a boxcar, so `iq_filter='boxcar'` accumulates for `iq_average_s` and
resets. `iq_filter='ewma'` runs `S = (1-a)S + x` in the same place, with a
per-point normaliser, so its average has memory `iq_average_s` and is readable
at any time. A *sliding* boxcar is deliberately not offered: it needs every
frame of the window kept, 100 MB at 5 s and 2500 Hz. `frame_stride` averages
one accumulation in N, trading noise for stream-loop time.

The consumer wakes every `poll_interval_s` (default 5 s) and takes the
finished average — O(1), and lock-free because the stream task and the
consumer coroutine share the event loop and never interleave. Estimation,
filtering, the controller's deadband/confirmation `decide` and the health
snapshot then run in a thread on `(blocks, points, tones)` arrays of a few tens
of kB, whatever the sample rate. Only the commit returns to the event loop,
which owns the modulation writes; it is O(corrected tones), not O(all tones).

With the stream loop in its own process (`firmware.stream_process`) the tap
runs there against a shared-memory accumulator and the finished window comes
back through the same shared block; nothing else changes.

**Estimator**: `estimate_detunings` is a lean whole-array implementation of
`demodulate`'s model-free `method='fast'` maths (finite-difference phase
slope + curvature; detuning from the curvature/slope ratio, which crosses
zero at resonance independent of the unknown phase offset). Unit tests
assert numerical agreement with `demodulate` to 1e-10, on a sorted
three-point pattern and on the fw default in several slot orders.
Settling-flagged samples are excluded by the tap, so they never enter the
average.

It works on the lowest, central and highest *offsets*, not the first, middle
and last *slots*, averaging points that share an offset. The slot form — used
by this and by `demodulate(method='fast')` until 2026-09-22 — reads the fw
default `[-d, 0, +d, 0]` as ~+37 linewidths at any detuning, since its middle
slot is `+d` rather than the centre; `'accurate'` was unaffected, as it fits
phase against offset. Phase differences are angles of products of neighbouring
points, so nothing is unwrapped: two `angle` calls on `(blocks, tones)`.

Since this *is* `method='fast'`, the comparison worth running is against
`method='accurate'`, which fits a quadratic to phase against offset rather
than using the extreme points only — a real difference for a pattern like
`[0, +d, 0, -d]`, whose repeated centre point 'fast' never uses.
`estimator_compare_tones` runs both on that many tones per window and reports
`estimator_diff` in `get_info('tracking')`. Off by default: it loops over
tones in Python (1064 ms at 1000 tones, 34 ms at 32), and that GIL hold costs
accumulations. The lean estimator drives every decision regardless.

**Measured on the RFSoC ARM**, 1000 tones, 4 modulation points:

| stage | where | cost |
|---|---|---|
| `tap`, boxcar | stream task, per accumulation | **33.1 µs** (21 % of the 161 µs typical spare at 2500 Hz) |
| `tap`, boxcar, `frame_stride=2` / `4` | stream task | 17.4 / 10.3 µs |
| `tap`, ewma | stream task, per accumulation | 51.9 µs (26.8 / 14.8 strided) |
| consumer, per window | thread (off the event loop) | a few ms on ~64 kB, whatever the sample rate |
| health snapshot rebuild | thread | ~14 ms per poll (`profiling/tracking_health_benchmark.py`) |
| `status()` / `health_check()` accessor | event loop | ~0.05 µs, flat in tone count (cache read) |
| `commit` (staging + queue) | event loop | grows with corrected tones, per correction only |

Two details worth knowing:

- **The running sum is int64, not int32.** int32 would save 15.4 µs a frame,
  and at the tone powers of 2026-09-20 (peak word 902) it would have had about
  9000 times the headroom it needed. But the receive chain's filter-bank
  shifts are set to maximise signal-to-noise, so accumulator words can reach
  full int32 scale, and an int32 sum then wraps after two frames — silently,
  into plausible detunings and therefore plausible but wrong corrections. Use
  `frame_stride` to buy the time back instead. (`psb_scale` belongs to the
  transmit side and has nothing to do with this.)
- **Binding the row views once, and counting in Python integers, matters.**
  Reading the count from a numpy array and re-indexing the accumulator on
  every frame cost about 14 µs more per accumulation than the addition itself.

The only tracking work on the stream loop is the per-frame tap (negligible)
and the per-correction commit (rare, small). Everything O(all-tones) —
estimation, the deadband/confirmation decision, and the health-snapshot
rebuild — runs in the consumer's `to_thread`, on a different core from the
stream loop, so neither tracking nor a health poll adds a stream-rate
regression. On the A53 expect the off-loop consumer batch to scale up
roughly an order of magnitude — still comfortably real-time at 2048 tones,
and `poll_interval_s` can be raised to trade latency for duty cycle.

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
| `iq_filter` | `'boxcar'` | coherent I/Q average in the stream task: `'boxcar'` (16.2 µs/frame) or `'ewma'` (49.1 µs/frame) |
| `iq_average_s` | = `poll_interval_s` | averaging length: boxcar window, or EWMA time constant |
| `frame_stride` | 1 | average one accumulation in N (trades noise for stream-loop time) |
| `filter_blocks_per_batch` | 4 | interleaved averages per window; gives the scatter the health stats use |
| `estimator_compare_tones` | 0 (off) | compare this many tones against `demodulate(method='accurate')` each window, reported as `estimator_diff`; Python-looped, so it costs accumulations |
| `min_detuning_snr` | 1.0 | a reading must beat its own scatter between blocks by this factor before anything is staged |
| `snapshot_max_age_s` | 60.0 | most often the health snapshot is rebuilt (it is O(tones) Python) |
| `filter` | `'boxcar'` | post-estimator detuning filter: `'boxcar'` or `'ewma'` |
| `filter_window` | 10 | boxcar length (windows) |
| `filter_tau_s` | 1.0 | EWMA time constant |
| `poll_interval_s` | 5.0 | how often the estimator runs (and, for a boxcar, the averaging length) |
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

Both are served on the event loop, so the full payload is **cached**: the
O(number-of-tones) snapshot is rebuilt off-loop by the consumer each poll
(`poll_interval_s`, ≤ 0.2 s stale — far finer than any health cadence), and
the `get_info`/`health_check` accessors just return the cache in O(1). A
health poll therefore never runs per-tone work on the stream loop
regardless of tone count (measured ~0.05 µs and flat 256→2048 tones, vs the
~14–30 ms the on-loop build would cost; see
`profiling/tracking_health_benchmark.py`).

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

## What tracking costs on the ARM

v7.12.1, both pipelines streaming 1000 tones each, GC frozen during captures,
`timing-monitor` stopped.

**Per accumulation**, in the stream loop: the tap, 33.1 µs for a boxcar
(17.4 µs at `frame_stride=2`, 51.9 µs for an EWMA), against the loop's spare
time:

| rate | period | spare, median | spare, worst frame in a thousand |
|---|---|---|---|
| 1500 Hz | 667 µs | 425-447 µs | 309-335 µs |
| 2000 Hz | 500 µs | 263-284 µs | 162-179 µs |
| 2500 Hz | 400 µs | 162-184 µs | 62-80 µs |
| 3000 Hz | 333 µs | 93-118 µs | -9 to +9 µs, past the edge |

**Per window**, in a thread: a few ms on ~64 kB whatever the sample rate, plus
~14 ms when the health snapshot is rebuilt.

**Per correction**: never through `set_tone_frequencies` — ~30 ms on the event
loop and 25-56 lost accumulations at 1-2 kHz, since it re-derives channel
maps, mixer settings and the whole tone plan. The tracker fills the idle half
of the ping-pong buffer and flips (`profiling/tracker_apply_cost.py`, 1000
tones):

| step | cost | where it runs |
|---|---|---|
| format the words (`prepare_control_buffer_data_fast`) | 892 µs | off the event loop |
| write one slot, TX and RX | 110 µs | off the event loop |
| fill all four modulation slots | 438 µs | off the event loop |
| flip the buffer (`set_control_buffer_idx_fast`) | 60 µs | on the event loop |

An update is therefore ~1.3 ms off-loop plus a 60 µs flip: 0.013% of the
machine at a 10 s cadence, 0.13% at 1 Hz. The apply was never the expensive
part.

### Two findings that shaped this design

**The GIL, when the consumer shares the process.** The first version kept
every frame and estimated per cycle; at 1500 Hz it lost 216 705 accumulations
in ten minutes in stalls of up to 83 ms, and cutting its batch 363 → 29 ms
barely moved that. The stream loop's wait was a spin, so it held the GIL
almost continuously and the consumer got it only at
`sys.setswitchinterval` boundaries — a 40 ms batch took 350 ms, and each of
those waits cost the loop accumulations. Two obvious levers were worse:
lowering the switch interval cut the batch (350 → 65 ms) but raised losses
(9.4 → 14.4 stalls/s), and smaller consumer operations contend more often
(a tiny op in a Python loop is the worst case, 8.8 ms p99.9). The wait now
sleeps between accumulations.

**Memory traffic, not CPU share.** Even with the GIL free, a consumer that
streams megabytes through the shared L2 costs accumulations on both pipelines
— see [Data path and performance](#data-path-and-performance). Hence averaging
in the stream loop, with the consumer seeing tens of kB.

### Where to run it

1000 tones to 2500 Hz: in-process is fine with the settings above. With
`firmware.stream_process` the tap runs in the stream process and the estimator
in the server, which removes the GIL contention outright — the configuration
the 2500 Hz dual-pipeline results were taken in. At 2048 tones or 3000 Hz the
rate itself binds, not tracking.

**Still unmeasured**: the flip's effect on a *running* stream (the 60 µs above
was with the pipelines idle), and the transient when the firmware switches
buffers mid-cycle.
