# Firmware v7.12 and streaming on the KRM

v7.12.1 (souk-firmware PR #121, tag `v7.12.1` = `4fb2812`) moves the
accumulator and LO control buffers off AXI4-Lite onto 128-bit AXI4 BRAMs and
doubles the AXI control clock to 200 MHz. This document records what that
bought, what the software around it then cost, and what a board needs in order
to stream both pipelines at 2500 Hz with modulation and tone tracking without
losing accumulations.

The sections below are the conclusions; the dated runs behind them, and the
staged firmware tables, are in the appendices.

Conventions: an *accumulation* is one accumulator dump, one I/Q sample per
tone, at rate `r` and period `1/r` (400 µs at 2500 Hz). A *torn read* spans an
accumulation boundary and is flagged per frame as `packet_error`. The
criterion throughout is *zero events* — no lost and no torn accumulation over
a long capture — not a loss percentage.

## Summary

**Firmware**, software held fixed (stages S0 → S3):

| | v7.11.2 | v7.12.1 | gain |
|---|---|---|---|
| accumulator read, CPU | 79.3 ns/word, 48 MiB/s | 22.9 ns/word, 167 MiB/s | 3.5× |
| accumulator read, 2048 channels | 333 µs | 102 µs devmem, 31 µs ZDMA | 3.3×, 10.8× |
| LO control write, 1000 tones, one bulk copy | 202.5 µs | 17.6 µs | 11.5× |
| `read_accumulated_data_fast`, 1000 / 2048 tones | 226 / 531 µs | 111 / 279 µs devmem, 86 / 116 µs ZDMA | 2.0-4.6× |

~70-75% of that is the AXI4 BRAMs, ~25-30% the 200 MHz clock. CPU reads now sit
at 90% of the ARM's uncached-mapping ceiling (186 MiB/s), so only DMA goes
faster.

**One correctness bug.** Until 2026-09-16 23:10 every tone set through the
fast path on v7.12 landed in its pair-neighbour's synthesis bin: the 16-bit
PSB reorder map packs two entries per bus word and the devmem path swapped
each pair. Anything recorded through the server on v7.12 from 2026-09-14 to
that fix has misplaced tones; v7.11 data is unaffected
([details](#installing-v712)).

**Streaming**, 1000 tones per pipeline:

| | result |
|---|---|
| v7.11.2 + v1.7.10 (the starting point) | clean to ~900 Hz, `rate/2` at 1500 Hz |
| one pipeline, ZDMA | zero events to 2000 Hz; 0.004% at 2400 Hz; 0.04% at 3000 Hz |
| both pipelines, 1 h, no requests | 1500 Hz zero events; 2000 Hz 8 and 5 lost |
| both pipelines, 2500 Hz, fw modulation + dry-run tracking, 30 min | 0 and 7 lost of 4.5 M each, 0 torn |

The last row needs the stream loop in its own real-time process, which
`souk-enable-tuning` sets up ([board setup](#setting-up-a-krm-board-for-these-rates));
in-process the same load lost 893 and 1768 in 146 s.

**Open**: a sweep + fit since the upgrade, the timed-sync round trip, and a
handful of rare 2500 Hz events ([Still open](#still-open)).

## The firmware upgrade

### What v7.12 changes

PR #121 is 15 commits. `ebceb6c` carries the boot-image side of all four
changes; the fabric side is the mlib_devel `afe77f5` AXI4-BRAM yellow block and
the rebuilt `.fpg`s.

1. **AXI-lite → full AXI4** on the accumulator and LO control buffers, 128-bit
   on HPM1. CPU accesses are still single transfers; bursts are the DMA path.
2. **AXI control clock 100 → 200 MHz**, set by `psu_init` in `BOOT.BIN`; the
   design is timed for it (`make_axilite_200mhz.tcl`).
3. **UIO over `0xA000_0000`-`0xC000_0000`** (`image.ub`): CPU access without
   root. DMA still needs root.
4. **u-dma-buf landing buffers**, 8 × 2 MiB (`image.ub`), for PS-DMA ("ZDMA")
   reads.

All 224 register names are unchanged; 223 of the 224 addresses moved, and the
accumulator and LO control buffers now sit at `0xb0000000`-`0xb9020000`. The
PSB reorder map (`p*_synth_input_reorder_map*`) halved to `0x1000` bytes: same
2048 entries, 16-bit instead of 32-bit. The relocated BRAMs carry no
`xps:bram` metadata, so casperfpga falls back to a plain `Memory` — name-based
access works, but they no longer appear under `host.sbrams`.

### Installing v7.12

These land together:

- **casperfpga `a89480c`** — mmap window 32 MiB → 512 MiB, to reach the new
  addresses.
- **control_sw `7.12.1`** — version-matched to the bitstream; a mismatch fails
  to initialise.
- **souk_readout_tools ≥ 1.7.10**, ≥ 1.8.1 for the reorder-map byte order.
- **`BOOT.BIN`** (the clock) and **`boot.scr`** (byte-identical, but on
  upstream's install list).
- **`image.ub`** — UIO and the u-dma-buf nodes.
- **`top.bit` → `/lib/firmware/top.bit`** — the boot-time bitstream for the
  on-board peripherals, replaced as soon as the `.fpg` is programmed.

**The clock lives in `BOOT.BIN` alone.** `psu_init` makes two functional
changes: `PL0_REF_CTRL` divisor `0xa → 0x5`, and `FPD_SLCR.AFI_FS` setting
HPM1 to 128-bit (also its power-on value). Read throughput cannot identify
which `BOOT.BIN` is running, because at 200 MHz the CPU mapping limits reads,
not the fabric. Read `PL0_REF_CTRL` at `0xFF5E00C0` instead: divisor bits
[13:8] are `0x05` at 200 MHz, `0x0A` at 100 MHz (`profiling/board_state.py`).

**Updating the boot files.** Boot mode is `5 - SD1` (`python -m
souk_readout_tools.board_management status`), so `BOOT.BIN` comes from the SD
card's FAT32 partition and a second card is a complete rollback. Replacing the
files in place from the running system works; `BOOT.BIN` is the risky one,
since the BootROM loads it directly and a bad copy means no boot from SD —
treat that as needing someone at the board. `image.ub` is recoverable over the
console. Either way: copy to a temporary name, `sync`, drop caches, verify the
sha256 **read back from the card**, then rename over the live file.

**tcpborphserver** needs no rebuild — it sizes its window from the `.fpg` —
but `raw.c` bounds-checks katcp writes with the mapping pointer truncated to
32 bits. When the low 32 bits plus the new `0x19020000` span wrap past 4 GiB,
katcp writes to AXI-lite addresses are rejected with `register %s is outside
mapped range`: ~10% of programs, against 0.04% with the old 1.5 MiB span. It
fails loudly, and `board_state.py` checks the live mapping.

**Code changes here.** `psb_chanselect_{set,get}_channel_inmap` hardcoded a
4-byte entry size and took only the byte order from `_map_format`; both now
derive the itemsize (1.7.10). They then still viewed devmem words as `'<u2'`
while the firmware follows katcp's big-endian word order, which transposed
every pair of entries — visible on a DAC snapshot (tones loaded 1, ½, ¼, ⅛
came out ½, 1, ⅛, ¼), invisible to any round trip on one transport. The fast
path now carries the map as katcp's big-endian word stream.

**Checks after the upgrade** (2026-09-14, 2026-09-16): test suite; `.fpg`
header against the shipping build; version, `fw_supported`, `adc_clk_hz`
(`7.12.1`, 2457.6 MHz); UIO and udmabuf nodes; katcp and devmem bit-exact on
one accumulation with a real tone (4096 words, 0 differing); sparse LO writes
isolated in all eight tx/rx × buf × slot cases, with no read-modify-write
needed (the yellow block's byte enables work as claimed); katcp writes at both
ends of the map; PLL lock, RF module, LNA and sensors after the boot change;
PTP GM lock. Outstanding: the timed-sync round trip and a sweep + fit.

### Faults to report upstream

- **`acc_cnt` can be read mid-increment**, returning mixed old and new bits
  (`0x…8af` → `0x…8a0` → `0x…8b0`), ~1 in 30 000-70 000 increments at 6 kHz,
  from the v7.12 `.fpg` on — a CDC without a safe handshake. It presented as
  false 16-accumulation gaps and false torn reads. Worked around by reading
  until two reads agree (+4 µs); zero occurrences since.
- **`tt_sync_msb/lsb` reads 0 on v7.12.1**, so `timed_sync_check` reports FAIL
  for a sync that did fire.
- **`zdma_read_example.py` counts the byte-counter wrap as an error.**
  `ZDMA_CH_TOTAL_BYTE` wraps every 4 GiB and raises `BYTE_CNT_OVRFL` (ISR
  `0x008`) — a failed read every ~4.5 min at 1000 tones and 2 kHz. Exclude it
  from the error mask and clear the counter per transfer.

### What the firmware alone is worth

Stages S0 (v7.11.2) → S1 (AXI4 BRAMs) → S2 (200 MHz) → S3 (new kernel), same
software, each configuration read back from the board:

| quantity | S0 → S3 | BRAMs | clock |
|---|---|---|---|
| accumulator read, per word | 3.5× | 2.4× | 1.5× |
| LO write, 1000 tones, bulk | 11.5× | 6.2× | 1.9× |
| benchmark sweep step | 5.1× | 3.4× | 1.5× |

Bulk writes gain most because a 16-byte store is one 128-bit AXI4 write where
AXI-lite split it into four; scattered per-word stores keep one transaction
each and gain only the lighter handshake and the clock (50 → 19 ns/word
against 50 → 3.8 bulk).

The kernel adds no devmem speed, only UIO (same speed, no root) and ZDMA. ZDMA
reads cost 17 µs + 1.7 ns/word against 2 µs + 22.9, so the planner switches to
it above ~480 channels; ZDMA *writes* win only for a whole 4-slot LO register
(94 against 126 µs). In the stream, at one pipeline and 1000 tones, the
firmware alone moved the zero-event rate 800 → 1500 Hz and the < 0.1% rate
1500 → 2000 Hz (2400 Hz with ZDMA). Full tables:
[Appendix A](#appendix-a-staged-firmware-comparison-2026-09-16).

## What limits a stream

### The per-accumulation budget

Each round: wait, read, build the frame, send it, and — with tracking on — add
the I/Q words to the running average. At 1000 tones with ZDMA, against a
400 µs period at 2500 Hz:

| step | µs (p50) |
|---|---|
| read: accumulator, counter, time tag, slot tag | 67 |
| frame build, including the 33 µs tracking tap | 62 |
| send, plus an event-loop turn every few frames | 95 |
| **busy** | **227**, spare 174 |

The loop is not short of time on average. What costs accumulations is anything
that takes its core, or slows it, for hundreds of µs to milliseconds.

**A single overrun no longer costs a sample.** The accumulator holds each
accumulation until `acc_cnt` moves (`profiling/acc_hold_check.py`, verified on
the board). The wait used to read the counter and block until it *changed*, so
a round returning 10 µs late skipped an accumulation that was still sitting
there. With `catch_up` the wait takes a waiting accumulation whenever the read
can finish `ACC_CATCH_UP_GUARD_S` before the next is due (`acc_due_s`). The
deadline becomes mean busy < period, plus one period of tolerance for any
single round. Virtual clock, 1 M frames at +100 µs/frame: 10 253 lost → 0.

Late reads are on for the plain and fw-modulated stream, off for the sw
engine, sweeps and `get_samples`, which step hardware between accumulations,
and off across a retune or a new modulation revision.

**The ceiling is quantised.** Delivered rate ≤ `rate / ceil(loop / period)`,
so a higher requested rate can deliver less, and a faster read can be
invisible when both loop times fall in the same step (ZDMA's 24 µs at 1000
tones moved 6000 Hz saturation by 0.2%). Delivered rate tests the quotient;
loss % locates the knee.

### What takes the loop's time

**The server's own work.** In one process the loop shares the event loop and
the GIL with requests, `get_info`, the tracking consumer and commit staging: a
tone request is 10-23 ms, `get_info('all')` ~0.7 s, a tracking window ~15 ms.
Fixed by the [stream process](#the-stream-process).

**Cache footprint, not CPU share.** The four A53s share 1 MB of L2, so a
consumer that streams megabytes evicts what both stream loops need, across
processes. At equal duty cycle a 16 MB working set on CPU0-1 took a pipeline
from 0.114% to 22% loss while a 32 kB one cost nothing. Hence the tracking tap
in the stream loop (`doc/tone_tracking.md`) rather than frames handed to a
consumer.

**Process creation.** Each `fork`/`exec` rebuilds page tables and pollutes the
shared cache wherever it runs. The timing monitor at nine subprocesses a
second cost the loaded pipeline 2.5 accumulations a second at 2400 Hz; it now
spawns two per poll at three cadences (`doc/timing.md`). An ssh login — PAM,
logind, a new scope — cost both pipelines a frame on ~1 login in 4 during the
2026-09-22 soaks.

**The processor-to-fabric bus.** It serves one request at a time, and
tcpborphserver's driver calls hold it for milliseconds:
`rfdc.core.get_mixer_settings` 4-10 ms, `fpga.get_status` / `is_programmed`
~2.2 ms each. Both pipelines' reads stall meanwhile. FPGA status now comes
from the fast path and the kernel FPGA manager (five status calls: 311 ms and
5 stalls → 2.8 ms and none). The mixer settings still go through the driver,
so `get_info` sections `tones`, `rfdc`, `calibrations` and `health_check` cost
both streams a few accumulations. The ten registers themselves read in 0.24 ms
over devmem with no stall, so the delay is in tcpborphserver's path.

**Kernel per-CPU work.** PREEMPT_NONE, HZ 250, `CONFIG_SLAB=y`: `cache_reap`
runs on every core every 2 s for 100-300 µs and no isolation can move it — it
was the 2 s loss grid. SCHED_FIFO 50 for the stream process
(`SOUK_STREAM_RT_PRIO`) takes the core back at the reaper's next
`cond_resched()`, and is safe only because the wait sleeps rather than spins
(a spinning RT task is either throttled by `kernel.sched_rt_runtime_us`, a
50 ms stall per second, or starves its core's kernel threads). SLUB would
remove the reaper, at the cost of a kernel rebuild.

**The garbage collector.** A collection in the loop cost an accumulation every
~4 s at 2400 Hz. `_set_streaming` freezes and disables collection while a
stream runs; memory grew 2-3.6 MB per server over 4.5 h of captures.

**The send path.** `SO_SNDBUF` autotunes from 16 KiB — two frames at 1000
tones — and a producer cannot wait out the ramp, so it is pinned to 1 MiB
(`net.core.wmem_max` must allow it). Sending is ~95 µs a frame, ~32 µs of it
GSO plus the driver. end0 has two TX queues and no XPS map by default, so each
flow picks one by hash; sharing a queue makes one stream core transmit the
other's packets too (`outside` 127 / 61 µs instead of 93 / 93, fixed for the
life of the connection). The board tuning maps tx-0 → CPU2, tx-1 → CPU3.

**Timers and logging.** Every timed wake in the frame path cost an
accumulation when it landed on a busy round: the 1 s stream keepalive (72
gaps/60 s at 2 kHz), its 30 s replacement, the 0.1 s trigger poll (one per
106 ms at 2400 Hz). All now wait on events. A log line is ~0.5 ms on the ARM,
so torn reads are no longer logged — the flag rides in the frame and in
`loop_timing`.

## What each change bought

### Accumulator reads

| change | effect, 1000 tones |
|---|---|
| sparse reads (only channels holding tones) | 333 → ~168 µs on v7.11.2 |
| `acc_cnt` over devmem instead of casperfpga | 27.4 → 4.0 µs, twice per accumulation |
| counter read until two reads agree | removes the CDC glitch, +4 µs |
| ZDMA above ~480 channels | 111 → 86 µs (2048 tones 279 → 116) |
| one DMA channel + udmabuf per pipeline | dual-pipeline loss 3-30× lower |
| plan the read before the wait | first read of every capture was torn (2.95 ms) → none |
| store the new index array on a content match | a second tone set cost 35 µs/read until restart |
| contiguous comb read as a slice (`take_slice`) | ~27 µs; `data` is now a view, valid until the next read |

### Frames and sending

| change | effect |
|---|---|
| drain/yield every 8 frames / 64 KiB / 2 ms | ~83 µs a frame; usable rate ~2000 → ~3500 Hz, 3000 Hz loss 27.6% → 0.385% |
| one `b''.join` + one `struct` per frame | ~4× the cached-numpy-frame build; byte-identical |
| `SO_SNDBUF` pinned to 1 MiB | startup gaps 3 → 1 at 1 kHz |
| GC frozen while streaming | one accumulation per ~4 s at 2400 Hz |
| no timed wakes, no hot-path logging | the 1 s, 30 s and 0.1 s combs |
| drain timeout, then drop the client | two dead connections had frozen both loops until TCP gave up |

### Tone writes

LO control words go through a RAM copy of the control BRAMs: fields placed in
RAM, touched tone rows pushed as one 64-byte-aligned bulk copy, katcp gone
from amplitude and phase writes.

| request | before | after |
|---|---|---|
| `set_tone_amplitudes` | 1.26-1.34 s | 6 ms |
| `set_tone_phases` | 1.26 s | 7-9 ms |
| frequency, TX + RX, 1000 tones | 247 µs | 110 µs |
| all four fields | 354 µs | 140 µs |

The copy holds only while nothing else writes those registers.
`get_info('control_shadow')` lists words another writer changed; the tone
getters take `refresh_shadow_copy=True`. An outside katcp write is detected,
not prevented — the next fast write restores all four words of every tone row
it touches.

The first `set_tone_frequencies` after a restart took 8.66 s, 8.52 s of it
`import scipy.signal` for the filterbank prototype. The server builds the
prototype in `main()`; first call now ~30 ms.

### Tone tracking

Per-accumulation cost is one coherent add into a per-(block, point) sum:
33.1 µs at 1000 tones, 17.4 with `frame_stride=2`, 51.9 for an EWMA. The
consumer takes a finished window (~32 kB) every `poll_interval_s` and
estimates on `filter_blocks_per_batch` coherently averaged blocks, in a
thread.

The first design parsed every frame and estimated per cycle: 363 ms per 0.2 s
batch at 1000 tones / 2500 Hz and ~10 MB through the shared L2. Dry-run
tracking on one pipeline then delivered 799 of 2500 frames/s and cost the
*other* pipeline 63 023 accumulations in 10 min. Averaging first is also
slightly better at low SNR — the estimator is non-linear, and a 10-cycle
boxcar of 125 per batch discarded 92% of the data. Design and settings:
`doc/tone_tracking.md`.

### The stream process

`firmware.stream_process: true` or `SOUK_STREAM_PROCESS=1` runs wait → read →
frame → send → tap in `server/stream_process.py`, on its own core; the server
keeps requests, sweeps, `get_samples`, tracking estimation, commits and info.
The frame and client code is the server's own, so frames are byte-identical.

What crosses: a shared block in `/dev/shm/souk_stream_p<N>` (mode, the eight
stream flags, fw revision and `n_settle`, tone-set generation, the pause
handshake, the finished tracking window, the loop-timing ring and the loss
records), read every frame; and a pipe for tone indices, typed frames and the
sparse-read override, read at every yield. No call site changed —
`active_tone_indices` and `fw_modulation_state` are properties, the flags are
`_PublishedEvent`.

Before any reprogram, resource init or clock fault the server pauses the
stream process and waits for the acknowledgement (2 s, 60 s while it is still
starting); an unclocked fabric hangs the PS. It resumes when
`_create_firmware_interfaces` finds the fabric healthy. The DMA channel and
landing buffer belong to the stream process, so server-side reads use the CPU
mapping. Refused while it is on: the sw modulation engine and triggered
streams. Off by default; `souk-enable-tuning` turns it on with pinning and
SCHED_FIFO.

### The board

`souk-enable-tuning` (`data/daemon/rt_tuning.sh`): stream loops alone on CPU2
and CPU3, everything else — including each server's request side — on CPU0-1,
`wmem_max`/`rmem_max` 32 MiB, one TX queue per stream core, and
`souk-rt-kernel.service` to re-apply the kernel-side half at boot.
`--journald-volatile` keeps the journal in RAM: SD writes during a capture
fall from 31% of wall clock to 0.2%, which removed the large-gap population at
3.5-4 kHz.

### Rejected, with numbers

- **Batching frames into one write per drain.** Bench 96 → 69 µs a frame;
  hardware 3000 Hz 0.385% → 15%. One lost accumulation per flush — the
  deadline is per accumulation. Worth re-testing now that a late read absorbs
  one long round.
- **Splitting the server's threads across cores.** Worse: Python threads then
  run in parallel and contend for the GIL instead of taking turns. Superseded
  by the process split.
- **Caching hardware state** (mixer settings, tone state) to dodge the slow
  driver calls. It worked — 93 → 15 lost per ladder start — but a cache can
  report stale hardware, so it was reverted in favour of fast-path reads.
- **Lowering `sys.setswitchinterval`** (5 ms → 50 µs): shorter consumer
  batches, more loop losses (9.4 → 14.4 stalls/s).

## Where the rates stand

1000 tones per pipeline on v7.12.1 with ZDMA. Every loss below is a single
accumulation unless stated.

### One pipeline

| rate | result |
|---|---|
| 600-2000 Hz | zero events, 3 × 30 s |
| 2400 Hz | 0.000-0.006% |
| 3000 Hz | 0.04% |
| 4000 Hz | 13%, delivery at the ceiling |

2048 tones: zero events to 1500 Hz, < 0.1% to 2400 Hz.

### Both pipelines, no other load

20 min per rate, no requests during the recording (2026-09-20). "Spare" is
period − busy; the 1-in-1000 column is the worst frame in a thousand.

| rate | period | spare p50 | spare 1-in-1000 | lost in 20 min |
|---|---|---|---|---|
| 600 Hz | 1667 µs | 1398 / 1414 µs | 1239 / 1281 µs | 0 / 0 |
| 1000 Hz | 1000 µs | 747 / 767 µs | 644 / 604 µs | 0 / 0 |
| 1500 Hz | 667 µs | 425 / 447 µs | 309 / 335 µs | 0 / 0 |
| 2000 Hz | 500 µs | 263 / 284 µs | 162 / 179 µs | 49 / 3 |
| 2500 Hz | 400 µs | 162 / 184 µs | 62 / 80 µs | 446 / 190 |
| 3000 Hz | 333 µs | 93 / 118 µs | -9 / +9 µs | 7.6% / 0.7% |

Hour-long runs: 1500 Hz zero events on both (5.4 M accumulations each),
2000 Hz 62 and 18 lost, 2500 Hz 1189 and 1116, no torn read in any.

Three regimes. To 1500 Hz there is > 300 µs of spare on the worst frame in a
thousand. At 2000-2500 Hz the worst frames come within 60-180 µs of the
deadline, so any added work shows. From 3000 Hz the 1-in-1000 frame is already
over the deadline and delivery falls to the ~2800-3000 frames/s ceiling: the
rate is unachievable, not merely lossy.

### Both pipelines, modulation and tracking

2500 Hz, fw modulation on four slots at ±2 kHz (dwell 1), dry-run tracking,
1000 tones each, 10 min per run unless stated. Losses are p0 / p1.

| step | lost | torn |
|---|---|---|
| in-process (146 s, cut short by an IP drop) | 893 / 1768 | 0 / 76 |
| stream process | 54 / 54 | 0 / 73 |
| + SCHED_FIFO, catch-up fixed for late rounds | 111 / 12 | 0 / 2 |
| + tracking window swapped through the shared block | 14 / 2 | 0 / 1 |
| + ZDMA on p0, XPS per stream core | 2 / 4 | 0 / 0 |
| 30 min | 4 / 0 | 0 / 0 |
| 30 min, host watchdog timer stopped | **0 / 7** | **0 / 0** |

Pipeline 0 lost nothing in 4.5 M accumulations. Pipeline 1's seven: one at
t+1 s, four in a 5 ms burst, two at capture teardown. The burst is the only
unexplained steady-state event left, and its loss records show every stage
slowed together — read 67 → 100 µs, frame 62 → 100 µs, outside 95 → 380 µs —
i.e. cache or memory interference, not descheduling. Nothing in the journal at
that wall-clock time.

Two earlier residuals were removed along the way: the 2 s wall-clock grid
(SLAB's `cache_reap`, fixed by SCHED_FIFO) and an accumulation at most
tracking polls (the pipe hand-over, fixed by the shared-block swap). The host
watchdog's ssh login every 5.5 min accounted for two losses in the run before
the last.

## Setting up a KRM board for these rates

1. `souk-enable-tuning`, then `souk-tuning-status` to check; it restarts both
   servers. `souk-disable-tuning` reverts. KRM-specific — skip it elsewhere
   (`doc/installation.md` step 9).
2. ZDMA per pipeline in each config: `firmware.zdma: true`,
   `zdma_udmabuf: "udmabuf0"` / `"udmabuf1"`, `zdma_channel: 0` / `1`. **Quote
   the buffer name**: `udmabuf0` unquoted parses as the integer 0, the reader
   falls back to the CPU read, and the only sign is one log line and ~30 µs
   more per accumulation.
3. `souk-enable-timing` if PTP is needed.

While recording:

- **No `get_info`** on either pipeline: the mixer-settings sections stall both
  pipelines' reads. Fetch info before the stream starts
  (`receive_stream(info=...)`).
- **No logins**, and no timer that logs in.
- **Start both streams together**; starting one while the other records costs
  the other ~50-100 accumulations.
- **Stop `timing-monitor`** for the most demanding runs.
- **Set tones before streaming**: a tone request stops that pipeline's loop for
  10-23 ms.
- **Compare pipelines only at equal tone counts** — 381 tones read in 68 µs
  against 84 µs at 1000, which changes the spare and therefore the loss for
  the same disturbance. There is no per-pipeline hardware difference.

After a reboot, check the accumulation rate and ADC clock
(`profiling/clock_rate_check.py`): after one reboot both were 12/13 of nominal
with every PLL reporting lock, and a power cycle fixed it. The board also
drops its secondary IP occasionally; `ifdown`/`ifup` restores it and
renumbers the PHC, which is why chrony and the monitor use `/dev/ptp-end0`.

## How to measure

| what | tool |
|---|---|
| long dual-pipeline captures, every lost and torn accumulation | `profiling/dual_soak.py` |
| rate ladder with per-point loop timing | `profiling/rate_ladder.py --loop-timing` |
| where each accumulation's time went, and the timing around every loss | `get_info('loop_timing')` |
| does the accumulator hold a sample until `acc_cnt` moves | `profiling/acc_hold_check.py` |
| real accumulation rate and ADC clock | `profiling/clock_rate_check.py` |
| what holds a core, with kernel symbols | `profiling/perf_cpu_sampler.py` (root) |
| per-frame costs against their alternatives, RAM only | `profiling/frame_path_bench.py` |
| board configuration, read back | `profiling/board_state.py` |

`get_info('loop_timing')` gives per-accumulation wait / read / frame / outside
/ busy, missed accumulations and torn reads over the last 131072 frames (~52 s
at 2500 Hz). Because a long capture outruns that ring, every frame following a
loss or a torn read is also copied into `losses` with the board's wall-clock
time and the 15 frames before it. That is what identifies an event after the
fact: one long row means the loop was held off; every stage slower at once
means cache or memory interference.

Traps:

- **Never bench the send path over loopback** — 78 µs a frame to `127.0.0.1`
  against 96 over the NIC, and every projection from it was optimistic.
- **Total loss % includes the startup transient** and is not monotonic in loop
  time. Use steady-state gaps split by size: ≤ 3 accumulations is loop
  overrun, more is an external stall.
- **The perf sampler costs ~2 accumulations a second** at 2500 Hz, so it
  cannot run during a loss measurement.
- **Stop both servers before profiling the board**; the other one still reads
  the bus.
- **`souk_readout_tools.__version__` comes from the installed dist-info**, not
  the source: check the file (`md5sum`), not the version string.

## Still open

- **No sweep or fit since the upgrade.** Refitting a known resonator to the
  same `fr`/`Qi`/`Qc` as on v7.11, same cooldown, is the strongest remaining
  check — and the one that would have caught the reorder-map swap.
- **The timed-sync round trip** on the 200 MHz clock, which needs a check
  against the accumulations' own telescope time while `tt_sync` reads 0.
- **The mixer-settings stall** through tcpborphserver, which still costs both
  streams accumulations on any `get_info` that reaches it. A fast-path read of
  the ten registers would avoid it; whether the driver's AUTO-vs-0P7 state
  matters in practice is unsettled.
- **Retunes during a stream** cost 10-23 ms, measured before the stream
  process existed; re-measure now that the request runs elsewhere.
- **One unexplained 5 ms disturbance** per 30 min at 2500 Hz.
- **Rootless operation**: UIO reads and writes cost the same as `/dev/mem`,
  but the server still runs as root, and ZDMA and `tsu_strobe` need it.
- **Batched writes and a late-read `get_samples`**, both reopened by the
  catch-up wait.
- **A SLUB kernel**, which would remove the periodic reaper that SCHED_FIFO
  currently works around.
- **Hours at 2500 Hz with tracking**: the longest clean run is 30 min.


## Appendix A: staged firmware comparison (2026-09-16)

The board was stepped through PR #121 one part at a time with the software
held fixed, so that each column differs from the previous one by exactly one
change. **S0** is v7.11.2; **S1** adds the AXI4 BRAMs; **S2** adds the 200 MHz
clock (a new `BOOT.BIN`); **S3** adds the new kernel (`image.ub`), which brings
UIO and the DMA buffers. The software is the same throughout, and each stage's
configuration was read back from the board rather than assumed. Where a
software version is named, **SW-a** is v1.7.10, **SW-c** adds sparse reads and
the devmem counter, **SW-e** adds the stream-loop changes, **SW-f** is SW-e
with DMA reads on, and **SW-g** adds the `get_samples` client fix.

### Stages

| stage | .fpg + control_sw | BOOT.BIN | image.ub | isolates |
|---|---|---|---|---|
| **S0** | v7.11.2 | pre-v7.12 (`566b1b82`) | pre-v7.12 (`c336e0a9`) | baseline |
| **S1** | v7.12.1 | pre-v7.12 | pre-v7.12 | AXI4 BRAMs + 128-bit HPM1, clock still 100 MHz |
| **S2** | v7.12.1 | v7.12.1 (`0674ecb9`) | pre-v7.12 | the 200 MHz clock alone |
| **S3** | v7.12.1 | v7.12.1 | v7.12.1 (`ee1ef476`) | the rebuilt kernel; adds UIO and u-dma-buf |
| **S3-sw** | v7.12.1 | v7.12.1 | v7.12.1 | software steps, below |

Held fixed for S0–S3: casperfpga `a89480c` (the 512 MiB map is harmless on
v7.11), `/lib/firmware/top.bit` v7.12.1 (`c56e6b0d`; only used before the
`.fpg` loads, and never reverted), souk_readout_tools at one commit with
`firmware.zdma: false`, journald storage, `timing-monitor` stopped, both
readout servers stopped for B1–B3. S1 is a combination upstream expects to
work but has not tested: check B0 and B5 before measuring. S2 and S3 can be
merged if booting the new `BOOT.BIN` with the old `image.ub` is unwanted.

The software held fixed for S0–S3 is **SW-e**: `c111e3e` plus the uncommitted
stream-loop patches (drain/yield throttle, `prepare_frame` cache, keepalive
suppression, `SO_SNDBUF` pin), ZDMA off. So S3 *is* SW-e on v7.12.1, and the
software steps run at S3 only vary the software around it:

- **SW-a** v1.7.10 (`5d42aa5`) — old software on new hardware. With the
  2026-09-15 v1.7.10 scan (old/old), S0 (new software, old hardware) and S3
  (new/new) it completes a 2 × 2, showing whether the software and firmware
  gains add up or overlap.
- **SW-c** sparse reads + devmem `acc_cnt` (`8cb05c5`), old stream loop —
  optional; splits read-side from send-side software on the new hardware.
- **SW-f** SW-e with ZDMA on — the ZDMA test, at 100 / 1000 / 2048 tones.
- **SW-g** SW-e with the `get_samples` client fix below — host-side only.

Dropped: v1.8.0 (its only change from v1.7.10 is `timing.py`, nothing in the
read or stream path), and `c111e3e` with ZDMA on but the old stream loop
(superseded by SW-f).

**Both socket fixes are in every stage** (the deployed `readout_server.py`
matches SW-e), but `get_samples` still loses a burst at the start of every
capture, and not because of the board. The client sends its request and
*then* pre-allocates a receive buffer for 2048 tones × `num_samples` —
113 ms for 18 000 samples, 189 ms for 30 000, 436 ms for 72 000 on the host —
while the server's send buffer, which `SO_SNDBUF` can only raise to
`net.core.wmem_max` (212 992 B read back, ~53 frames at 1000 tones), fills. The
data fit: one gap at sample ~65–72, none at 600 Hz, 54–73 samples at 1000 Hz
and ~900 at 2400 Hz, identical at S0 and S1. The stream has no such
allocation and loses 0–2 samples at startup. Fix (SW-g): allocate before
sending the request. Kept out of S0–S3 so the host side stays fixed.

### B0 — configuration read back

| | S0 | S1 | S2 | S3 |
|---|---|---|---|---|
| BOOT.BIN / image.ub / top.bit (md5) | `566b1b82` / `c336e0a9` / `c56e6b0d` | same | **`0674ecb9`** / `c336e0a9` / `c56e6b0d` | **`0674ecb9`** / **`ee1ef476`** / `c56e6b0d` |
| `PL0_REF_CTRL` divisor → AXI clock | `0x01010a02`: 10 × 1 → **100 MHz** (CCF 99 999 999 Hz) | same, **100 MHz** | `0x01010502`: 5 × 1 → **200 MHz** (CCF 199 999 998 Hz) | same, **200 MHz** |
| `AFI_FS` HPM0 / HPM1 width | `0x00000800`: 32-bit / **128-bit** (HPM1 at its power-on value) | same | same (now written by `psu_init`) | same |
| control_sw / fw checkout / configured `.fpg` | 7.11.2 / v7.11.2 / `dual_…_2026-07-01_1449` (server reports fw 7.11.2.0, type 3) | 7.12.1 / v7.12.1 / `dual_…_2026-07-22_1653` | same as S1 | same as S1 |
| casperfpga `MAP_SIZE` | `0x20000000` | same | same | same |
| souk_readout_tools (md5 `readout_server.py` / `firmware_lib.py`) | `c111e3e` + uncommitted stream patches (`e893b654` / `8c3e47f7`) | same | same | same |
| UIO nodes / u-dma-buf | axi-pmon ×4, rfdc / none | same | same | **`user_io`** + axi-pmon ×4, rfdc / **udmabuf0–7**; bootargs gain `uio_pdrv_genirq.of_id=generic-uio` |
| tcpborphserver `/dev/mem` map (wraps 4 GiB?) | `0x189000` at `0xffffb8bf7000` (no) | `0x19020000` at `0xffff9afe0000` (no) | `0x19020000` at `0xffff96d20000` (no) | `0x19020000` at `0xffff6c3d0000` (no) |
| other services during B1–B3 / journald | ptp4l, tsu-strobe / default, persistent | same | same (`timing-monitor` restarted by the reboot; stopped before S2) | same (`timing-monitor` stopped again after the reboot) |
| server `firmware.zdma` (`config_6HP90-MS2-Top_p0.yaml`) | `true`, inert — no u-dma-buf | same | same | `true` → **active**; B4 run as "S3 + ZDMA", then set `false` for S3 |

S0: `board_state.py` as root, 2026-09-16 12:36, after deploying the software
held fixed for S0–S3. S1: 13:43, same software. S2: 14:48, after rebooting on the new `BOOT.BIN`. S3: 15:54, after rebooting on the new `image.ub`.

### B1 — transport cost, `t = F + n_words·W` (p50; F excludes `np.frombuffer`)

| target | quantity | S0 | S1 | S2 | S3 | S1/S0 | S2/S1 | S3/S2 |
|---|---|---|---|---|---|---|---|---|
| accumulator `p0_acc0_dout0` | F µs | 2.25 | 1.94 | 2.04 | 1.97 | 0.86 | 1.05 | 0.97 |
| | W ns/word | 79.3 | 33.4 | 22.6 | 22.9 | **0.42** | **0.68** | 1.01 |
| | W as MiB/s | 48.1 | 114.2 | 168.8 | 166.6 | | | |
| | 2048-ch read p50 / p99.9 µs | 333.4 / 356.2 | 145.1 / 165.7 | 101.1 / 144.3 | 102.2 / 122.7 | 0.44 | 0.70 | 1.01 |
| LO control `p0_mix_tx_lo0_control0` | read W ns/word (MiB/s) | 89.5 (42.6) | 40.2 (94.9) | 26.0 (146.7) | 26.2 (145.6) | 0.45 | 0.65 | 1.01 |
| | bulk write (sync) 1000 tones µs | 202.5 | 32.6 | 17.6 | 17.6 | **0.16** | **0.54** | 1.00 |
| | bulk write (sync) W as throughput | 76 MiB/s | 509 MiB/s | 0.98 GiB/s | 0.98 GiB/s | | | |
| RX channel map `p0_chan_select_map0_map1` (AXI-lite at every stage) | W ns/word (MiB/s) | 79.6 (47.9) | 64.5 (59.1) | 38.2 (99.9) | 38.2 (99.9) | 0.81 | 0.59 | 1.00 |

**S0 → S1 (the AXI4 BRAMs, clock still 100 MHz):** accumulator reads 2.4×
faster per word, LO control writes 6.2× faster. The AXI-lite channel map also
got 19% faster although neither its bus nor the clock changed — the v7.12
build also reworked AXI-lite (denser map; upstream `232b75e` "remove some dead
time in AXI transactions"). So that row isolates the clock only from S1 to S2,
not across S0 → S1.

**S1 → S2 (the 200 MHz clock alone):** accumulator reads 1.48× faster per word,
LO writes 1.85×, and the AXI-lite map 1.69×. The accumulator now reads at
22.6 ns/word, 169 MiB/s — 91% of the 186 MiB/s measured at S3 for plain DRAM
through the same kind of uncached mapping, so from here the
CPU mapping, not the fabric, limits reads. S2 matches the 2026-09-14 full
v7.12.1 profile to within 0.5%, so the new `image.ub` should not move devmem.

**How the firmware gain splits** (S0 → S2, shares in log terms):

| quantity | S0 → S2 total | AXI4 BRAMs (S0 → S1) | 200 MHz clock (S1 → S2) |
|---|---|---|---|
| accumulator read, per word | 3.5× | 2.4× (69%) | 1.5× (31%) |
| LO write, 1000 tones, bulk | 11.5× | 6.2× (75%) | 1.9× (25%) |
| benchmark step (B2), Python | 5.1× | 3.4× (75%) | 1.5× (25%) |
| benchmark step (B2), C bulk | 6.0× | 3.7× (73%) | 1.6× (27%) |

At S3 only, one row per transport (accumulator target):

| transport | F µs | W ns/word | W as throughput | 2048-ch p50 µs | 2048-ch p99.9 µs |
|---|---|---|---|---|---|
| devmem | 1.97 | 22.9 | 166.6 MiB/s | 102.2 | 122.7 |
| uio (as `casper`) | 2.09 | 22.9 | 166.6 MiB/s | 102.3 | 117.3 |
| zdma | 17.1 | 1.7 | **2.19 GiB/s** | 30.8 | 48.2 |
| DRAM, uncached mapping (CPU ceiling) | 2.00 | 20.5 | **186.1 MiB/s** | 92.1 | 106.2 |
| DRAM, cached mapping | 2.10 | 1.2 | 3.10 GiB/s | 13.5 | 23.5 |

At S3, whole-buffer writes into the LO control register
`p0_mix_tx_lo0_control0`, devmem against a PS-DMA write from a u-dma-buf
(stage + `sync_for_device` + DMA + wait for DONE; every size verified by
reading the register back over devmem), p50 / p99.9 µs:

| bytes written | devmem posted | devmem sync | ZDMA | faster |
|---|---|---|---|---|
| 16 000 (1000 tones, within one slot) | 17.9 / 128.9 | 17.7 / 27.5 | 25.5 / 35.9 | devmem, 1.4× |
| 32 768 (2048 tones, one slot) | 33.9 / 55.5 | 33.8 / 43.1 | 35.6 / 47.1 | about equal |
| 131 072 (whole register, 4 slots) | 126.2 / 144.5 | 126.0 / 140.1 | 94.2 / 116.7 | **ZDMA, 1.34×** |

Cost models: devmem sync write 2.8 µs + 3.8 ns/word (0.99 GiB/s); ZDMA write
16.0 µs + 2.4 ns/word (1.56 GiB/s). They cross at ~38 KiB, just over one full slot. Unlike reads
(22.9 → 1.7 ns/word, 167 MiB/s → 2.2 GiB/s), CPU writes into the AXI4 BRAM already stream well, so
DMA's per-word advantage on writes is only 1.6×. A DMA write pays off only when
more than one slot is written at once — loading all four firmware-modulation
slots saves ~32 µs (25%). Ordinary tone updates, one slot or less, should stay
on devmem.

### B2 — benchmark sweep step (bulk 1000-tone write + bulk 1000-channel read, 10000 steps, p50 µs)

(The MiB/s rows are payload bytes over the mean call time, as the profiler
prints them.)

| mode | op | S0 | S1 | S2 | S3 | S3 ZDMA read | S3 UIO |
|---|---|---|---|---|---|---|---|
| Python bulk | write posted / sync | 199.2 / 202.5 | 31.6 / 34.0 | 16.7 / 18.9 | 16.8 / 19.0 | 16.9 / 19.4 | 16.8 / 19.0 |
| | read | 166.7 | 75.0 | 53.4 | 54.0 | **26.7** | 53.8 |
| | step | 369.3 | 109.0 | 72.3 | 73.0 | **46.1** | 72.8 |
| C bulk | write posted / sync | 196.1 / 200.7 | 29.6 / 30.5 | 14.9 / 15.4 | 14.9 / 15.4 | — | — |
| | read | 157.9 | 66.0 | 44.5 | 45.1 | — | — |
| | step | 358.6 | 96.5 | 59.8 | 60.5 | — | — |
| C per-word | read / step | 420.1 / 620.6 | 330.6 / 451.1 | 230.1 / 290.4 | 230.1 / 290.4 | — | — |
| Python bulk | read / write sync MiB/s | 45.6 / 75.1 | 101.4 / 446.1 | 142.3 / 798.4 | 140.9 / 795.9 | **283.2** / 786.8 | 141.3 / 798.8 |
| C bulk | read / write sync MiB/s | 48.2 / 75.8 | 115.4 / 498.9 | 170.7 / 989.2 | 168.9 / 987.9 | — | — |

### B3 — read path end to end (installed software, no wait, p50 µs)

| call | S0 | S1 | S2 | S3 |
|---|---|---|---|---|
| katcp `acc._read_bram()` 2048 ch | 53 301 | 82 386 | 78 020 | 82 849 |
| LocalMem `acc._read_bram()` 2048 ch | 546 | 347 | 302 | 300 |
| katcp / LocalMem `acc.get_acc_cnt()` / devmem `read_acc_cnt_fast()` | 1008 / 27.4 / 4.0 | 1030 / 28.0 / 4.1 | 1023 / 27.8 / 3.8 | 1014 / 27.5 / 4.0 |
| `read_accumulated_data_fast` 1 / 64 / 341 tones | 37.8 / 49.6 / 104.7 | 37.8 / 44.2 / 73.3 | 36.7 / 41.6 / 65.1 | 37.0 / 42.1 / 65.6 |
| `read_accumulated_data_fast` 1000 / 2048 tones | 225.8 / 531.3 | 132.1 / 322.7 | 110.2 / 277.8 | 110.7 / 279.2 |
| same with ZDMA enabled, 1000 / 2048 tones (planner's choice recorded) | — | — | — | **86.2 / 115.8** (ZDMA chosen; devmem kept at 1–341 tones) |

### ZDMA and sparse reads — what gain to expect

ZDMA does not compete with the *full* buffer read but with the sparse one.
With `firmware.zdma: true`, `_cached_read_plan` costs the sparse devmem plan
and a single ZDMA read of the same span, and uses whichever its model says is
cheaper. Tones always occupy accumulator channels 0…M−1
(`compute_vacc_tone_indices` returns `arange(M)` unless two tones share a TX
bin), so the span, and therefore ZDMA's possible gain, depends only on tone
count. With the planner's constants (devmem 1.64 µs + 22.4 ns/word, 170 MiB/s;
ZDMA 21.94 µs + 1.4 ns/word, 2.7 GiB/s; both v7.12.1 bare transport):

| tones | devmem µs | ZDMA µs | planner uses | saving µs |
|---|---|---|---|---|
| 100 | 6.3 | 22.2 | devmem | 0 |
| 341 | 17.1 | 22.9 | devmem | 0 |
| 500 | 24.2 | 23.4 | ZDMA | 0.9 |
| 1000 | 46.4 | 24.7 | ZDMA | 21.7 |
| 2048 | 93.4 | 27.7 | ZDMA | 65.7 |

The crossover is ~480 channels by these constants (the code comment beside
them, and earlier notes, say ~335). So below ~480 tones ZDMA is never used and
can change nothing;
at 1000 tones it saves ~22 µs of a loop of a few hundred µs, which is why the
1000-tone stream showed +0.2%; at 2048 tones it saves ~66 µs on the transport
alone, and possibly more — the devmem full-buffer software path costs ~180–200 µs
above the bare read (B3: 323 µs at S1 against 145 µs), which the ZDMA path may
avoid. **2048 tones is where ZDMA can matter and where S0 was weakest.** The
plan therefore tests it at all three tone counts: 100 tones as the control
(the planner must pick devmem, no change expected), 1000 tones, and 2048 tones
as the decisive case (headline "S3 + ZDMA" column, SW-e vs SW-f).

### B4 — rate ladder (stream socket, 1000 tones, mean of 3 × 30 s)

The target is **2–4× the 600 Hz campaign rate**, so the grid is dense from
1200 to 2400 Hz. Headline, per stage — the highest grid rate such that it and
every lower rate meet the criterion. **"Zero events"** is the one that matters
for long integrations: no lost sample (startup included) and no `packet_error`
in 3 × 30 s. The loss thresholds use mean steady-state loss (index ≥ 200) and
only say whether the loop keeps up on average.

| configuration | criterion | S0 | S1 | S2 | S3 | S3 + ZDMA |
|---|---|---|---|---|---|---|
| stream, 1000 tones | **zero events** | **800 Hz (1.3×)** | **1200 Hz (2×)** | **1000 Hz (1.7×)** † | **1500 Hz (2.5×)** | **none** † (600 Hz: one 1.7 ms stall, 2 samples; 800–1200 Hz clean) |
| | < 0.1 % | 1500 Hz (2.5×) | 2000 Hz (3.3×) | 2400 Hz (4×) | 2000 Hz (3.3×) ‡ (2400 Hz: 0.48 / 0.42 / 0.007%) | 2400 Hz (4×) |
| | < 1 % | 2400 Hz (4×) | 3000 Hz (5×) | 2400 Hz (4×) ‡ | 2400 Hz (4×) | 3000 Hz (5×) ‡ (mean 0.996%) |
| `get_samples`, 1000 tones | **zero events** | **600 Hz (1×)** | **600 Hz (1×)** | **600 Hz (1×)** | **600 Hz (1×)** | **none** (600 Hz: 1 startup sample in the redone capture; see below) |
| | < 0.1 % | 2400 Hz (4×) | 600 Hz — one 340 ms stall at 800 Hz; 1000–2400 Hz ≤ 0.002 % | 3000 Hz (5×) | 3000 Hz (5×) | 3000 Hz (5×) |
| | < 1 % | 2400 Hz (4×) | 3000 Hz (5×) | 4000 Hz (6.7×) | 3000 Hz (5×) | 3000 Hz (5×) |
| stream, 100 tones | **zero events** | **1800 Hz (3×)** | **1800 Hz (3×)** | **none** † (one 27 ms stall at 600 Hz; 800–2400 Hz clean) | **600 Hz (1×)** † (800 Hz: 1 sample) | **1800 Hz (3×)** |
| | < 0.1 % | 3000 Hz (5×) | 3000 Hz (5×) | 3000 Hz (5×) | 2400 Hz (4×) | 3000 Hz (5×) |
| | < 1 % | 4000 Hz (6.7×) | 5000 Hz (8.3×) | 5000 Hz (8.3×) | 4000 Hz (6.7×) | 4000 Hz (6.7×) |
| stream, 2048 tones | **zero events** | **none** (600 Hz: 1 lost, 1 torn) | **1000 Hz (1.7×)** | **1000 Hz (1.7×)** | **1000 Hz (1.7×)** | **800 Hz (1.3×)** |
| | < 0.1 % | 1000 Hz (1.7×) | 1200 Hz (2×) | 1500 Hz (2.5×) | 1200 Hz (2×) | 1800 Hz (3×) |
| | < 1 % | 1200 Hz (2×) | 1500 Hz (2.5×) | 1800 Hz (3×) | 1800 Hz (3×) | 1800 Hz (3×) |

**What these can and cannot say.** 3 × 30 s at 1500 Hz is 135 000 samples, so
"zero events" bounds the event rate only to roughly 1 in 10⁵ — not the "odd
packet over a long integration" regime, which needs hour-long soak captures.
The ladders also run in clean conditions: `timing-monitor` stopped (its 3 s
`chronyc` comb alone cost 0.18% at 1 kHz), one pipeline streaming, no tone
tracking, a DSP-only comb, a freshly restarted server.

Stream, 1000 tones — delivered frames/s · steady-state loss % (mean of 3):

| rate Hz | S0 | S1 | S2 | S3 | S3 + ZDMA |
|---|---|---|---|---|---|
| 600 | 600.0 · 0.000 | 600.0 · 0.000 | 600.0 · 0.000 | 600.0 · 0.000 | 600.0 · 0.004 |
| 800 | 800.0 · 0.000 | 800.0 · 0.000 | 800.0 · 0.000 | 800.0 · 0.000 | 800.0 · 0.000 |
| 1000 | 1000.0 · 0.002 | 1000.0 · 0.000 | 1000.0 · 0.000 | 1000.0 · 0.000 | 1000.0 · 0.000 |
| **1200** (2×) | 1200.0 · 0.000 | 1200.0 · 0.000 | 1199.8 · 0.015 † | 1200.0 · 0.000 | 1200.0 · 0.000 |
| **1500** (2.5×) | 1500.0 · 0.002 | 1500.0 · 0.001 | 1500.0 · 0.000 | 1500.0 · 0.000 | 1500.0 · 0.002 |
| **1800** (3×) | 1799.9 · 0.106 | 1801.4 · 0.024 | 1801.4 · 0.023 | 1801.7 · 0.004 | 1800.9 · 0.035 |
| **2000** (3.3×) | 1998.5 · 0.073 | 2000.0 · 0.001 | 1999.6 · 0.020 | 1998.9 · 0.054 | 1999.7 · 0.014 |
| **2400** (4×) | 2381.5 · 0.770 | 2390.3 · 0.403 | 2398.3 · 0.071 | 2392.8 · 0.302 ‡ | 2399.0 · 0.043 |
| 3000 | 1500.2 · 49.99 | 2984.2 · 0.524 | 2791.7 · 6.94 ‡ | 2884.6 · 3.85 ‡ | 2970.1 · 0.996 |
| 4000 | 1990.3 · 50.24 | 2389.4 · 40.26 | 3522.1 · 11.95 | 2573.4 · 35.67 ‡ | 3565.3 · 10.86 |
| 5000 | 1598.6 · 68.03 | 2494.7 · 50.11 | 2491.8 · 50.17 | 2500.1 · 50.00 | 2591.1 · 48.18 |
| 6000 | 1274.4 · 78.76 | 2987.4 · 50.21 | 2899.2 · 51.68 | 2913.2 · 51.45 | 2992.8 · 50.12 |

The same layout for `packet_error`, steady small/large gaps, for
`get_samples`, and at 100 and 2048 tones; `rate_ladder.py` stores every row
(`v712_stages/S0/ladder/`, `…/S1/ladder/`, `…/S2/ladder/`).

**S0 reading.** By the strict criterion v7.11.2 is where experience says it
is: `get_samples` at 1000 tones is event-free only at 600 Hz (800 Hz already
drops 29 samples at startup), the stream only to 800 Hz, and 2048 tones not
even at 600 Hz. What the current software changed is the *average* loop: the
stream keeps up at 1000 tones to 1500 Hz with a few isolated drops (0.002%),
where v7.11 + v1.7.10 on 2026-09-15 lost 0.34% at 1200 Hz and 50% at 1500 Hz.
The accumulation rate is genuine — each capture's counter span matches its
duration (43 800 distinct frames in 29.2 s at 1500 Hz). Above 1500 Hz the margin
is thin: 1800 Hz varied 0.06–0.19% across repeats, 2400 Hz had ~17
`packet_error` frames per 30 s, and 3000 Hz falls straight to `rate/2`.
`get_samples` loses 0.2% (1000 Hz) to 1.3% (2400 Hz) of every capture in its
startup transient, which the steady-state figures exclude.

**S1 reading (AXI4 BRAMs, clock still 100 MHz).** At 1000 tones the stream's
zero-event rate rises 800 → **1200 Hz**, 1800–2000 Hz become near-clean
(0.024% / 0.001%), and 3000 Hz no longer halves (0.52%, delivering 2984/s). At
2048 tones 600–1000 Hz become event-free and the halving point moves from
1500 Hz to ~2000 Hz. 100 tones gain only at 4000 Hz and above, where the read
was the limit. `get_samples` still loses ~0.2–1% of each capture at startup, so
its zero-event rate stays at 600 Hz; one repeat at 800 Hz also hit a single
340 ms external stall (272 samples) — rate-independent, the kind of rare event
behind "the odd dropped packet", and the reason 30 s ladders can't certify a
rate.

**S2 reading (200 MHz clock).** The loop itself improved again: at 1000 tones
2400 Hz loses 0.04–0.13% per repeat (S1 0.10–0.60%), 4000 Hz delivers 3522/s
(S1 2389/s), and 2048 tones stay under 1% to 1800 Hz (S1 1500 Hz).
`get_samples` is under 0.1% to 3000 Hz. Two caveats on the headline numbers:

- † **The zero-event rates are now set by isolated gaps, not the loop.** Every
  S2 event below 3000 Hz is a single 9–27 ms gap, one per capture at most
  (1200, 1800, 2000, 2400 Hz at 1000 tones; 600 Hz at 100 tones; 1200 Hz at
  2048 tones), and at these capture lengths a matter of chance: S2's 1000 Hz
  against S1's 1200 Hz is one 13 ms gap. *Correction, after all ladders:*
  these are not time-based stalls. Five of the six are **exactly 16
  accumulations** (the sixth 20), whatever the rate — see
  [the 16-accumulation gaps](#the-16-accumulation-gaps). One
  (2400 Hz, 15:00:42) coincides with `systemd-tmpfiles-clean` (15:00:53, the
  15-min-after-boot timer; S2 started ~2 min after a reboot); the rest have no
  scheduled job. Comparing stall *rates* between stages needs long captures;
  with 90 s per point only the loop-overrun behaviour compares reliably.
- ‡ **3000 Hz is bimodal at S2:** one repeat 0.40%, two ~10%. The loop sits
  right at the 333 µs period there, so small shifts flip it; the mean is not
  a regression against S1's 0.52%.

**S3 reading (ZDMA off) and the matched ZDMA comparison.** Plain S3 differs
from S2 only by the kernel, and its loop results match S2 within repeat
scatter; the knee rates are bimodal per repeat (1000 tones: 2400 Hz 0.48 / 0.42
/ 0.007%, 4000 Hz 11.9 / 47.9 / 47.2%). Against plain S3 — same kernel, same
boot, only `firmware.zdma` changed — ZDMA:

- **2048 tones: removes the loss at 1500–1800 Hz** (0.583% → 0.005%,
  0.609% → 0.002%) and keeps 2400 Hz from halving (1200 → 2081/s); 2000 Hz is
  unchanged (~2.9%). The < 0.1% rate rises 1200 → 1800 Hz.
- **1000 tones: helps only at 2400 Hz and above** (2400 Hz 0.30% → 0.04%,
  4000 Hz 35.7% → 10.9%); 1800–2000 Hz are within scatter either way.
- **100 tones (control): unchanged**, including 5000 Hz (6.7% vs 7.1%) — so
  the 5000 Hz difference against S2 below is the kernel or noise, not ZDMA.

The first pass, against S2 before plain S3 existed, read:

- **2048 tones is where ZDMA shows, as predicted.** 1800 Hz goes 0.672% →
  0.002%, 2000 Hz 7.7% → 3.0%, and 2400 Hz stops halving (1200 → 2081/s). The
  < 0.1% rate rises 1500 → 1800 Hz (3×).
- **1000 tones: small gains.** 2400 Hz 0.071% → 0.043%, 3000 Hz 6.9% → 1.0%
  (still bimodal per repeat: 0.39 / 2.19 / 0.41%).
- **100 tones (control, planner keeps devmem): unchanged up to 3000 Hz**, as
  expected. 4000–5000 Hz came out worse (5000 Hz 0.26% → 7.1%, one repeat
  each) — either noise or the new kernel; plain S3 separates the two.
- **No sign of the feared tail regression** at the knee rates on the patched
  build.
- **One `get_samples` capture at 600 Hz was aborted by the server** after
  10 940 of 18 000 frames — `DMA error: ISR=0x008 STATUS=0x0` at 16:23:42.
  **This is a software bug in the ZDMA reader, not a hardware fault.** ISR bit 3
  is `BYTE_CNT_OVRFL` (`xzdma_hw.h`): the channel's 32-bit total-bytes counter
  (`ZDMA_CH_TOTAL_BYTE`, `0x188`) wraps every 4 GiB transferred. Xilinx's driver
  clears it (`XZDma_TotalByteClear`); `zdma.py`, vendored from upstream's
  `zdma_read_example.py`, never does, and counts the flag in `IXR_ERRORS`. When
  a poll sees the flag before DONE the read raises; when both arrive together
  they are cleared silently. The channel moved ~17 GB during the S3 profiling
  and this ladder (~4 wraps), and one was caught. At 1000 tones and 2 kHz that
  is a wrap every ~4.5 min, so with ZDMA on, captures would abort
  intermittently in normal operation. Fix (after the comparison): don't treat
  `0x008` as an error, and clear the counter when programming a transfer.
  Upstream's example has the same bug; `access_size_scaling.py`'s DMA writer
  inherits it. The aborted capture was redone at 19:14 after a server restart
  with ZDMA still on: 18 000 frames, no DMA error, one sample lost in the
  startup transient (the `get_samples` client allocation, not ZDMA).

Software × hardware, stream, 1000 tones — delivered frames/s · loss %
(zero-event rate in the last row):

| rate Hz | v1.7.10 on v7.11.2 (2026-09-15) | SW-e on v7.11.2 (S0) | SW-a on v7.12.1 | SW-c on v7.12.1 | SW-e on v7.12.1 (S3) | SW-f on v7.12.1 |
|---|---|---|---|---|---|---|
| 1200 | 1195.9 · 0.340 (60 s) | 1200.0 · 0.000 | 1199.7 · 0.024 | 1199.8 · 0.013 | 1200.0 · 0.000 | 1200.0 · 0.000 |
| 1500 | 750.0 · 50.0 (60 s) | 1500.0 · 0.002 | 1497.5 · 0.168 | 1500.0 · 0.002 | 1500.0 · 0.000 | 1500.0 · 0.002 |
| 1800 | — | 1799.9 · 0.106 | 1794.9 · 0.383 | 1801.1 · 0.042 | 1801.7 · 0.004 | 1800.9 · 0.035 |
| 2000 | 998.9 · 50.06 | 1998.5 · 0.073 | 1913.2 · 4.34 | 1996.5 · 0.172 | 1998.9 · 0.054 | 1999.7 · 0.014 |
| 2400 | — | 2381.5 · 0.770 | 1200.0 · 50.00 | 2393.9 · 0.253 | 2392.8 · 0.302 | 2399.0 · 0.043 |
| 3000 | 747.8 · 75.08 | 1500.2 · 49.99 | 1499.6 · 50.01 | 2984.0 · 0.532 | 2884.6 · 3.85 | 2970.1 · 0.996 |
| zero-event rate | ≤ 600 Hz | 800 Hz | 600 Hz | 600 Hz | 1500 Hz | none (one 1.7 ms stall at 600 Hz) |

On v7.12.1, stepping the software: the read-side changes (SW-a → SW-c:
sparse reads, devmem `acc_cnt`) remove most of the loss from 1500 to 2400 Hz
(1500 Hz 0.168% → 0.002%, 2000 Hz 4.3% → 0.17%, 2400 Hz no longer halves).
The stream-loop patches (SW-c → SW-e: drain/yield throttle, frame cache,
keepalive suppression, `SO_SNDBUF`) cut the remaining small gaps (1800 Hz
0.042% → 0.004%, 2000 Hz 0.17% → 0.05%) and lift the zero-event rate from 600
to 1500 Hz. Near the knee they do not help: 3000 Hz is 0.53% with SW-c and
bimodal with SW-e (0.40 / 0.41 / 10.7%).

`get_samples`, 1000 tones, host-side fix (SW-g) — startup samples lost per
capture · zero-event rate:

| rate Hz | SW-e on v7.12.1 (S3) | SW-g on v7.12.1 |
|---|---|---|
| 800 | 0 / 0 / 0 | 0 / 0 / 0 |
| 1000 | 70 / 51 / 51 | **0 / 0 / 0** |
| 1200 | 153 / 129 / 126 | **0 / 0 / 0** |
| 1500 | 274 / 272 / 273 | **0 / 0 / 0** |
| 1800 | 490 / 442 / 461 | **0 / 0 / 0** |
| 2000 | 609 / 569 / 570 | **0 / 0 / 0** |
| 2400 | 921 / 870 / 886 | **0 / 0 / 0** |
| 3000 | 1478 / 1406 / 1513 | **0 / 0 / 0** |
| 4000 | 2754 / 2803 / 2713 | 2 / 3 / 2 |
| zero-event rate | 600 Hz | 600 Hz |
| < 0.1% loss, whole capture | 800 Hz | **3000 Hz (5×)** |

The fix removes the startup loss outright (captured 2026-09-16 21:44, same
server and board state as S3). Its zero-event rate stays at 600 Hz only because
of three isolated 16-sample gaps, each with one torn read — at 800 Hz (20 ms),
1500 Hz (10.7 ms) and 3000 Hz (5.3 ms); the S3 run had one at 800 Hz too.
Between them 1000–2400 Hz were clean apart from one 1-sample and one 3-sample
gap.

ZDMA off vs on at the tone counts where the planner changes its choice
(stream, delivered frames/s · loss % and zero-event rate):

| rate Hz | 100 tones SW-e | 100 tones SW-f | 2048 tones SW-e | 2048 tones SW-f |
|---|---|---|---|---|
| 600 | 600.0 · 0.000 | 600.0 · 0.000 | 600.0 · 0.000 | 600.0 · 0.000 |
| 800 | 800.0 · 0.004 | 800.0 · 0.000 | 800.0 · 0.000 | 800.0 · 0.000 |
| 1000 | 1000.0 · 0.000 | 1000.0 · 0.000 | 1000.0 · 0.000 | 999.9 · 0.010 |
| 1200 | 1200.0 · 0.000 | 1200.0 · 0.000 | 1199.3 · 0.058 | 1200.0 · 0.000 |
| 1500 | 1500.0 · 0.000 | 1500.0 · 0.000 | 1491.3 · **0.583** | 1499.9 · **0.005** |
| 1800 | 1801.8 · 0.002 | 1801.8 · 0.000 | 1790.8 · **0.609** | 1801.8 · **0.002** |
| 2000 | 2000.0 · 0.000 | 1999.8 · 0.010 | 1941.3 · 2.911 | 1940.4 · 2.986 |
| 2400 | 2399.6 · 0.017 | 2399.8 · 0.007 | 1199.9 · **50.01** | 2080.8 · **13.30** |
| zero-event rate | 600 Hz | 1800 Hz | 1000 Hz | 800 Hz |

### B5–B9 — correctness and science checks

| check | S0 | S1 | S2 | S3 |
|---|---|---|---|---|
| B5 step 6: katcp and devmem agree, **with a real tone** | | | | pass — same accumulation (acc_cnt 69312485) read over both, 4096 words, 0 differing (1000 tones loaded) |
| B5 step 7: PSB inmap round trip | | | | **fail** — slow→slow exact; slow→fast and fast→slow differ: the 16-bit map is pair-transposed between katcp and devmem (see below) |
| B5 step 8: sparse LO write isolation | | | | pass — tx/rx × buf 0/1 × slot 0/1, neighbours and other LO untouched, restored |
| B5 katcp write to lowest and highest register, no `outside mapped range` | | | | pass — `p0_chan_select_map0_map1` and `p1_mix_tx_lo0_control1` |
| B6 peripherals after boot change: PLL lock, RF module, LNA | | | | pass — LMK + LMX locked (4 chips), rf_frontend / lna / sensors / fpga ready; timing reads not-ready only because `timing-monitor` is stopped |
| B7 step 9: timed sync round trip | | — | | |
| B8 step 10: sweep + fit the same resonators, same cooldown | | — | — | |
| B9 torn-read check (fw-mod slot alternation at 3 kHz) | | — | — | — |

**B5 step 7 at S3 (2026-09-16 22:20) — the PSB reorder map is pair-transposed
between katcp and devmem.** v7.12 narrows `synth_input_reorder_map{0-3}_map1`
to 16-bit entries (`VaccReorderMultiSampleIn._map_format = '>i2'`), two per
32-bit bus word. Read over both transports, every map register's devmem bytes
equal the katcp bytes with each pair of entries swapped: katcp gives words in
network order per 32-bit word (`800005a3`), devmem the same word in memory
(`a3050080`), so `'>u2'` over katcp and `'<u2'` over devmem disagree in pair
order. `psb_chanselect_{set,get}_channel_inmap` (devmem) are consistent with
each other, and upstream's katcp pair is consistent with itself, which is why
a round trip on one side passes — the 2026-09-14 "step 7 pass, no pair swap"
was wrong. The map the server loaded reads back monotonic over devmem (inputs
0, 1 → `0x1683`, `0x1688`) and pair-swapped over katcp. v7.11's map was
32-bit, so v7.11 data is unaffected.

**The firmware follows katcp, so every tone our server set on v7.12 went to
its pair-neighbour's synthesis bin** (with its own fine offset, amplitude and
phase). DAC snapshot, 4 tones at 700/1100/1500/1900 MHz loaded with amplitudes
1, ½, ¼, ⅛, peak amplitudes in frequency order:

| map written by | peaks | |
|---|---|---|
| server, old devmem code | 0.5, 1.0, 0.133, 0.228 | pairs swapped |
| same intended map over katcp | 1.0, 0.5, 0.27, 0.125 | correct |
| server again, old code | 0.5, 1.0, 0.133, 0.228 | reproduced |
| same intended map, **fixed** devmem code | 1.0, 0.5, 0.269, 0.125 | correct; katcp reads back the intended map exactly |

Fixed in `firmware_lib` (the devmem path now carries the map as katcp's
big-endian 32-bit word stream); installed on the board 2026-09-16 23:10. No streaming test could see this. A sweep would have: B8 is the check
that closes it.

B5's transports check also exposed a trap: it changed `acc_len` with
`mixer.set_acc_len` and restored it without the master reset + sync the server
does (`sync.sw_sync(mrst=True)`), which **stopped pipeline 0's accumulator**
(counter frozen at the value the check recorded). The server's next tone set
then blocked forever in control_sw's `accumulator._wait_for_acc`, which has no
timeout and runs on the event loop, so the whole server stopped answering.
A `sw_sync(mrst=True)` over katcp restarted it; `board_checks.py` now restores
the raw register and syncs.


## Appendix B: measurement log

A chronological record of the runs behind the conclusions above, kept because
several of them rule out things that would otherwise be proposed again. Where
a finding was later superseded, the entry says so.

### Earlier performance, v7.10.3 to v7.12.1, and the v7.11 rate scan (2026-06, 2026-09-15)

1000 tones / 1000 read-chans / 10000 steps, medians. Full table and analysis
in `profiling/README.md`.

**The baseline column is firmware v7.10.3, not v7.11.** It was run on
2026-06-10, before v7.11 existed (first build 2026-06-18), against offsets
`0x050000`/`0x090000` — `p0_mix_tx_lo0_control` and `p0_acc0_dout0` in the
v7.10.3 dual-pipeline map (on v7.11.2 they are at `0x080000`/`0x150000`). So
this compares v7.10.3 with v7.12.1: it includes v7.11's buffer split into
128 KiB `control0/1` and v7.11.2's LO BRAM latency increase. **No v7.11.2
profile exists**; B2 at S0 provides it.

| | v7.10.3 Python | v7.12.1 Python | v7.10.3 C-bulk | v7.12.1 C-bulk |
|---|---|---|---|---|
| write posted / sync | 207 µs / — | **16.77 / 19.08 µs** | 196 µs / — | 14.91 / 15.36 µs |
| read | 168 µs | **53.40 µs** | 163 µs | 44.49 µs |
| step | 375 µs | **72.48 µs** | 359 µs | 59.85 µs |
| read ns/word | 84 | **26.7** | 82 | 22.2 |
| read MiB/s | 45.1 | **142.4** | 46.5 | 171.0 |
| write MiB/s (posted / sync) | 73.4 / — | 952 / 800 | 77.4 / — | 1067 / 993 |

A sweep step went 375 µs → 72 µs, **5.2×** (359 → 60 µs, 6.0× in C). The plan
predicted ~4× for the *read* only; C-bulk reads came in at 3.66×, close to
that, and Python at 3.15× because its ~9 µs fixed per-call overhead is now a
large share of a 53 µs read. The **write gain of 12.3× (posted to posted) was
never predicted** and dominates the step. How much of each gain is the AXI4
path and how much the clock is not known (stages S1/S2).

C-bulk's 22.2 ns/word (171 MiB/s) is 94% of the ~190 MB/s (≈181 MiB/s) that upstream measured copying
out of *DRAM* through an uncached mapping. So the read is now limited by the
CPU's uncached mapping, not by the fabric, and only DMA into a cached buffer
gets past it.

Side effect: the Python-vs-C gap widened — reads 1.03× → 1.20×, steps
1.045× → 1.21× — not because Python got slower but because its fixed
per-call overhead no longer hides behind a slow bus. This weakens but does not
overturn the standing "don't rewrite the fast path in C": per-word C remains
far worse than bulk anything, and these profilers measure only the devmem
operations, not the Python work wrapped around them in a real sweep step.
Measure `get_sweep_data` end to end before deciding.

### The v7.11 rate scan (2026-09-15) — firmware and software both differ

Run on the same board by reverting it: the v7.11.2 dual-pipeline bitstream
(`souk_dual_pipeline_krm_2026-07-01_1449`, via the configured symlink),
control_sw `7.11.2`, `souk_readout_tools` **v1.7.10** (commit `5d42aa5`,
verified by md5 of the installed `readout_server.py`), pre-v7.12 `BOOT.BIN`
(`566b1b82`) **and** `image.ub` (`c336e0a9`). Not reverted: casperfpga
(`a89480c`) and `/lib/firmware/top.bit` (v7.12.1). 1000 tones
(`linspace(500e6, 2000e6, 1000, endpoint=False)` + `default_rng(0).uniform(±100e3)`,
Newman phases, unit amplitudes), stream socket, 30 s captures,
`timing-monitor` stopped.

**This is not a firmware-only comparison.** The v7.12 columns ran later
software on top of v1.8.0:

- **"v7.12 read-optimised"** = `c111e3e` with `firmware.zdma: true`: sparse
  reads and devmem `acc_cnt` (`0b3711b`, `8cb05c5`) plus ZDMA. `plan_accumulator_reads`
  is not in v1.8.0.
- **"v7.12 + stream patches"** = the same plus the drain/yield throttle and
  `prepare_frame` cache described below, ZDMA still on.

So v7.11 → v7.12 in these tables changes the firmware and three software
features at once. Stages S0–S3 above redo it with the software held fixed.

**v1.7.10 cannot drive v7.10** — `firmware_lib` calls `r.mixer.set_acc_len()`
with no version shim, and `acc_len` only moved into the mixer at v7.11. The
working pairings are v7.10.3 ↔ the `wip/timed-sync-v7.10` branch (setup.py
1.5.0; untagged), v7.11.2 ↔ v1.7.10, v7.12.1 ↔ v1.7.10/v1.8.0.

**Delivered frames/s** (`rate × (1 − loss)`, from `packet_counter`; no wall clock,
so immune to connect/setup time):

| rate Hz | v7.11 + v1.7.10 | v7.12 read-optimised, ZDMA on | v7.12 + stream patches, ZDMA on |
|---|---|---|---|
| 600 | 600 | 600 | 600 |
| 1000 | 999 | 1000 | 1000 |
| 2000 | **998** | 1999 | 2000 |
| 3000 | 748 | 2173 | **2988** |
| 3509 | 775 | 1755 | **3481** |
| 4000 | 766 | 2000 | **3541** |
| 6000 | 841 | 2986 | 2992 |

**The achievable rate rose: ceiling ~1000 → ~2990 → ~3540 frames/s, and the
usable rate (loss < 1%) ~1200 → 2000 → 3500 Hz** (v7.11's 1200 Hz point is
from the fine scan below, 0.34%). Roughly 3.5× end to end, for firmware and
software together.

Two caveats on the middle column. Its 3000 Hz point (27.6% loss) is a ZDMA-on
build; a devmem build with otherwise similar software lost 1.03% there, so
"usable 2000 Hz" may understate what the v7.12 firmware allows. And the
journald setting for that ladder was not recorded.

**`packet_error`:**

| rate Hz | v7.11 | v7.12 read-optimised | v7.12 + stream patches |
|---|---|---|---|
| 2000 | 28–59 | 0 | 0 |
| 3000 | **21806** | 1 | 0 |
| 4000 | **22362** | 2 | 4 |
| 6000 | **24521** | 20 | 23 |

At 3000 Hz and above on v7.11 it fires on **every frame** (21806 in 21806):
the full-buffer read is longer than the 333 µs period, so every read spans an
accumulation boundary. That the I/Q of such a frame is actually torn — part
old accumulation, part new — is inferred, not shown on data (test B9).
The flag is also not strictly like-for-like across software: v1.7.10 reads the
closing `acc_cnt` through `acc.get_acc_cnt()` (~27 µs), later versions through
devmem (3.7 µs), which widens v1.7.10's detection window slightly. If B9
confirms the tearing, **above 2 kHz on v7.11 the data you keep is corrupt, not
merely sparse** — a correctness argument for v7.12, not just a throughput one.

#### Where the v7.11 ceiling comes from — not yet measured

`profiling/access_size_scaling.py` was re-run on the reverted board with
`--read-addr 0x000d0000`, but **that offset was taken from the single-pipeline
`.fpg`**. The server loads the dual-pipeline build, where `0xa00d0000` is
`p1_mix_rx_lo0_control0`, not the accumulator (`p0_acc0_dout0` is at
`0xa0150000`). So the v7.11 numbers describe an AXI-lite LO control BRAM:

| | fixed F | per-word W | read @1000 tones |
|---|---|---|---|
| v7.11.2, `p1_mix_rx_lo0_control0` (F includes `np.frombuffer`) | 8.56 µs | 89.5 ns | 375.2 µs *modelled* for 4096 words |
| v7.12.1, `p0_acc0_dout0` (bare slice, F excludes `np.frombuffer`) | 1.64 µs | 22.4 ns | 46.4 µs *modelled*, sparse, 2000 words |

W is comparable (89.5 → 22.4 ns/word, 43 → 170 MiB/s, 4.0×) if the two AXI-lite BRAMs behave
alike. F is not: the v7.11 fit timed `np.frombuffer(mm[a:b])`, and the same
fit on v7.12 gave 8.31 µs, so most of the "8.56 → 1.64" is `np.frombuffer`
(6.9 µs on its own, measured on the board 2026-09-16), not the bus. The earlier "8.1× on the read actually
paid" and its split into ~4× transport × ~2× sparse reads rest on both
problems, and sparse reads are post-v1.8.0 software (`0b3711b`), not a v1.8.0
feature. B1 and B3 at S0 replace this.

The earlier "measured frame times of 1031 / 334 / 283 µs" were not measured:
334 and 283 µs are 1 ÷ the best delivered rate (2986 and 3541 /s), and 1031 µs
matches nothing and contradicts the fine scan below (clean at 1200 Hz means a
loop under 833 µs). Loop time needs a direct server-side measurement.

#### The ceiling is quantised, and delivered rate is non-monotonic

Delivered rate ≤ `rate / ceil(loop / period)` — a frame producer can only land
on a whole number of accumulations. This is an **upper bound, not a fit**: no
single loop time reproduces these ladders (best worst-point error 34% for
v7.11, 33% read-optimised, 13% with the stream patches). On v7.11, 1000 and
2000 Hz both deliver ~999/s (loop ≤ 1000 µs) but 3000 Hz delivers 748/s
(loop > 1000 µs), so the loop time itself grows with rate. Two consequences
still hold and will mislead anyone who does not expect them:

- **A higher requested rate can deliver less.** On v7.12 read-optimised,
  4000 Hz delivers 2000/s while 3000 Hz delivers 2173/s. Predicted, then
  confirmed.
- **A faster read can be completely invisible.** ZDMA's 24 µs saving at 1000
  tones moved saturation at 6000 Hz 2994.8 → 3000.9/s (+0.2%): any loop
  between 167 and 333 µs delivers `rate/2` there, so a 24 µs change cannot
  show. (The `ceil(275/167)` example came from a different build's series.)
  Do not conclude from such a test that the read is free.

Corollary: **loss % and delivered rate test different things.** Quantisation sets
the delivered-rate floor; jitter adds partial loss that ramps smoothly as the
period approaches the mean loop. Use delivered rate to test the quotient, loss %
to locate the knee.

#### Fine scan at the historical operating point

v7.11, 60 s captures, persistent journald — the configuration the campaign
actually ran in:

| rate Hz | frames | gaps | lost | loss % |
|---|---|---|---|---|
| 600 | 35487 | **0** | **0** | **0.000** |
| 700 | 41432 | 1 | 1 | 0.002 |
| 800 | 47320 | 3 | 7 | 0.015 |
| 900 | 53213 | 5 | 5 | 0.009 |
| 1000 | 59093 | 57 | 62 | 0.105 |
| 1200 | 70724 | 235 | 241 | 0.340 |
| 1500 | 44364 | 44363 | 44363 | **50.000** |

**v7.11 was clean to ~900 Hz and fell off a cliff between 1200 and 1500 Hz**,
bounding its loop at these rates to ~667–833 µs/frame. 600 Hz was a sound
choice: for an ~800 µs loop, a 1667 µs period gives ~108% margin, where 1000 Hz
gives only ~25%.


### What sets the v7.12 ceiling: the send path, not the read (2026-09-15/16)

Measured on the ARM at 1000 tones, 8044 B frames:

| term | µs/frame |
|---|---|
| `prepare_frame` entire body | **41.5** |
| `write()` + `drain()` + `sleep(0)` | **122.7** (135 with one competing task, 164 with three) |
| bare `await asyncio.sleep(0)` | **30** |

An earlier estimate attributed the residual mostly to frame construction. That
was an inference and it was wrong: framing is a minor term and the asyncio path
is the ceiling. Fixes landed, each measured before and after on hardware:

- **Drain/yield throttled** to whichever of 8 frames / 64 KiB / 2 ms trips first
  (`_yield_due`, `_drain_stream_clients`). Loop 188.7 → 78.4 µs on the bench;
  on hardware 3000 Hz went **27.6% → 0.385%** loss and 3509 Hz stepped
  **1754 → 3479 frames/s**.
- **`prepare_frame` caches** its array, header view and length prefix (15.3 µs,
  byte-identical output). The payload must stay a fresh `bytes` — `tracking.tap()`
  retains it uncopied, so a shared buffer would alias queued frames.
- **Stream keepalive suppressed while streaming.** `handle_stream_client` woke at
  1 Hz to send a zero-length heartbeat; each wake overran the accumulation
  deadline. At 2 kHz this was **72 gaps/60 s at a 1005 ms median spacing**, 86%
  within 5% of a whole second, against **0** on `get_samples`, which has no such
  loop. Fixed: **72 → 16 gaps, 0.071% → 0.014%**. The same signature appears on
  v7.11 + v1.7.10 (0.97 gaps/s at 1000 Hz, 999 ms median, 93% whole-second),
  and v1.7.10 has the same 1 Hz keepalive, so it is long-standing and
  firmware-independent. Something periodic remains: after the fix 60% of gaps
  still fall within 5% of a whole second, against ~10% by chance.
- **`SO_SNDBUF` pinned** (`_size_send_buffer`, 1 MiB, capped by
  `net.core.wmem_max` = 212992 here). The kernel autotunes from 16 KiB — two
  frames at 1000 tones — and a frame producer cannot wait out the ramp, so
  `drain()` blocked. This is the main cause of the "startup ramp": gaps at fixed
  sample positions (~25/43/46–82) whose size grows with rate (0 at 600 Hz, 23 at
  1 kHz, 366 at 2 kHz). Startup gaps **3 → 1**, not 0 — `wmem_max` caps the
  pinned buffer at ~26 frames. The continuous stream never showed it — its
  socket is long-lived and already grown.

#### Rejected, with numbers, so it is not re-proposed

**Batching frames into one `write()` per drain.** The bench says it should win —
96.1 → 69.4 µs/frame over the NIC — and it is protocol-transparent, since
`receive_stream` is a pure byte-stream parser. On hardware it made the rates that
matter far worse: **3000 Hz 0.385% → 15.0%**, 3509 Hz 0.796% → 13.0%. At 4000 Hz
there was one gap every ~7.5 frames against a batch of 8 (5.7 and 6.8 at 3000
and 3509 Hz), though there batching (12.7%) was no worse than the throttle alone
(11.5%). Mechanism: one lost accumulation per flush. **The deadline is per accumulation, not per batch**, so lowering the mean
does not help if one iteration in eight is long. It only helps above ~5 kHz where
nothing fit anyway. Reverted.

#### Two measurement traps

- **Never bench this over loopback.** The identical pattern measured 78.4 µs/frame
  to `127.0.0.1` and 96.1 µs over the NIC, and the loopback bench under-predicted
  the true loop by ~58–76 µs. Every projection made from it was optimistic. Use a
  remote sink.
- **Total loss % is contaminated by the startup transient** and is not monotonic
  in loop time — at 1000 Hz a run with *zero* steady-state gaps reported the
  highest total loss of three, because all its loss was in the startup cluster.
  Use steady-state gaps (index ≥ ~200) and split by size: small (≤3 samples) is
  loop overrun, large (>3) is an external stall. Different causes, different fixes.

#### journald / SD card

`Storage=volatile` cuts SD write time during a capture from **31% of wall clock
(211 writes, 9446 ms per 30 s) to 0.2% (12 writes, 49 ms)**, and removes the
large-gap population where it was compared (7 → 0 large gaps at both 3509 and
4000 Hz). **On v7.11 it makes no consistent difference** — the differences go
both ways (600 Hz 0.000% volatile vs 0.074% persistent; 5000 Hz 750 vs 831/s)
— because the loop there is too slow for SD stalls to stand out. It is only
worth having on a system that is otherwise fast enough. Note the journal then does not survive a
reboot, on a board where self-power-on needs the PC.


### What the software alone bought, on v7.11.2 hardware (2026-09-15/16)

Most of the software used for S0–S3 was written *during* the v7.12 work, to
chase costs the faster firmware exposed — but almost all of it applies to v7.11
as well. S0 therefore measures the old hardware with the new software, and the
2026-09-15 scan measured the same hardware (same dual-pipeline v7.11.2
bitstream, boot files, casperfpga and `top.bit`) with v1.7.10. Together they
isolate the software.

| change | where | per-frame effect, 1000 tones |
|---|---|---|
| **Sparse accumulator reads** — read only the channels holding tones | `0b3711b`, `8cb05c5` | read 333 → ~168 µs on v7.11.2 (S0 B1: 4096 vs 2000 words) |
| **Accumulation counter over devmem** instead of `acc.get_acc_cnt()` | `0b3711b` | 27.4 → 4.0 µs per read, read at least twice per frame (S0 B3) |
| **Drain/yield throttled** to every 8 frames / 64 KiB / 2 ms | uncommitted | ~110 µs per frame on the bench; on v7.12, 3000 Hz loss 27.6% → 0.39% |
| **`prepare_frame` buffer reuse** | uncommitted | 41.5 → 26.2 µs |
| **Stream keepalive suppressed while streaming** | uncommitted | removes a ~1 gap/s periodic loss (2 kHz: 72 → 16 gaps / 60 s) |
| **`SO_SNDBUF` pinned** | uncommitted | `get_samples` startup gaps 3 → 1 |
| **Gap warnings** in `parse_samples` / `parse_stream` | uncommitted | none — makes gapped records visible |
| `timing-monitor` stopped *(operational, not code)* | — | removes a 3.02 s `chronyc` comb, ~0.18% at 1 kHz; root cause unknown |
| journald `Storage=volatile` *(operational)* | — | no consistent effect on v7.11; removes large stalls at 3.5–4 kHz on v7.12 |
| ZDMA, UIO | `9bff6be`, `c111e3e`, `e251090`, `af11a36` | v7.12 `image.ub` only; off for S0–S3 |

Stream, 1000 tones, `timing-monitor` stopped, persistent journald — same
hardware, software changed:

| rate Hz | v1.7.10 (2026-09-15): loss % · `packet_error` | current (S0, mean of 3): loss % · `packet_error` |
|---|---|---|
| 600 | 0.074 · 1 (30 s); 0.000 (60 s) | 0.000 · 0 |
| 800 | 0.015 (60 s) | 0.000 · 0 |
| 1000 | 0.099 · 0 | 0.002 · 0.7 |
| 1200 | 0.340 (60 s) | 0.000 · 0 |
| 1500 | 50.0 (60 s) | 0.003 · 0 |
| 2000 | 50.06 · 28 | 0.073 · 0.7 |
| 3000 | 75.08 · 21 834 | 49.99 · 12.7 |
| peak delivered frames/s | ~999 | 2381 (at 2400 Hz) |

**Software alone roughly halved the per-frame loop on v7.11.2** — the rate at
which the stream falls to `rate/2` moved from ~1200–1500 Hz to ~2400–3000 Hz,
and peak throughput rose 2.4×. By the zero-event criterion the gain is much
smaller (≤600 Hz with v1.7.10, where a 30 s run already lost 13 samples, to
800 Hz), so the extra headroom is mostly *average* headroom — time left per
frame for real-time computation such as tracking — rather than a certified
higher rate. The firmware stages S1–S3 are measured on top of this software, so
their gains are in addition to it.


### Pushing the zero-drop rate with both pipelines (2026-09-16/17)

Target: both readout servers (pipelines 0 and 1) streaming, sweeping or
tracking at full speed at the same time on the 4-core A53, with no lost and no
torn accumulation.

**What still costs accumulations at S3** (one pipeline, 1000 tones, event-free
to 1500 Hz):

- *Loop overruns.* One coroutine per frame busy-waits for the accumulation,
  reads it, builds the frame and sends it; all of it must fit in one period
  (556 µs at 1800 Hz, where 6 single-sample gaps occurred in 90 s).
- *External stalls* of 2–30 ms (once 340 ms), and the torn reads that follow a
  late wake.

**Found on the board** (2026-09-16):

- `readout_server_0` ran on CPU0, which takes **every** Ethernet (`end0`,
  98 M) and SD-card (`mmc1`) interrupt, as an ordinary process (nice −20,
  `SCHED_OTHER`). `ptp4l` is the only real-time task.
- The servers' 1 MiB `SO_SNDBUF` is capped by `net.core.wmem_max` at
  212 992 B — ~26 frames, **~13 ms at 2 kHz** — so any longer host-side pause
  blocks `drain()`.
- A torn read logs from the hot path (`acc.logger.warning` in `firmware_lib`,
  `print('Packet error')` in `stream_data`), both to journald on the SD card.
- **ZDMA was not dual-pipeline safe:** every `ZdmaReader` used FPD DMA
  channel 0 (`0xFD500000`) and the template config gave both pipelines
  `udmabuf0`. The hardware and the device tree already have what a second
  pipeline needs: the v7.12.1 `image.ub` has all eight FPD DMA channels
  (`fd500000.dma`–`fd570000.dma`, 128-bit) and eight LPD ones, each bound to
  `xilinx-zynqmp-dma` with its own runtime PM (on the board, `fd500000` is
  `active`/`on` because the reader pins it, the rest `suspended`/`auto`), plus
  `udmabuf0`–`7`. `ZdmaReader` wakes and pins whichever channel it is given.
  Now configurable: `firmware.zdma_channel` (0–7) beside `zdma_udmabuf`; give
  pipeline 1 channel 1 and `udmabuf1`. Not yet run on channel 1 — validate
  with one read (a wrong guess about the clock hangs the bus).

**Tools ready:**

- `get_info('loop_timing')`: per frame, the wait, read, frame-build and
  *outside* time (sending, yielding, other event-loop work), the time from an
  accumulation to waiting for the next (`busy`), missed accumulations and torn
  reads, with the same breakdown for just the frames after a gap. Reset at
  each capture; ~1 µs per frame. `rate_ladder.py --loop-timing` stores it per
  point.
- `profiling/rt_tuning.sh apply [--journald-volatile] | revert | status`:
  housekeeping and interrupts on CPU0–1, `readout_server_0` alone on CPU2,
  `readout_server_1` alone on CPU3, `wmem_max`/`rmem_max` 32 MiB. Not
  `SCHED_FIFO`: a busy-waiting real-time task is either RT-throttled (a 50 ms
  stall every second) or starves its core's kernel threads.

**Pipeline 1 config (2026-09-16):**
`~/.souk_readout_tools/pipeline_1/config/config_6HP90-MS2-Top_p1.yaml`, now
pipeline 1's `default_config.lnk` (the old April-schema `p1_config.yaml` is
kept in `previous_default.lnk`; it had `clock_source: internal` against
pipeline 0's `external`). A copy of the pipeline 0 file with only the
pipeline-specific fields changed, as `copy_template_config` would: pipeline
1, ports 10001/20001, DAC tiles 1, ADC tile 3, `rf_channel` 1, `zdma_udmabuf`
`udmabuf1` and `zdma_channel` 1, and `rf_frontend.connected: false` (module
channel 1 is not fully populated). Cryostat, detector and LNA entries are
unchanged copies (LNA backend `fixed`, so nothing is driven). Takes effect on
the next restart of `readout_server_1`.

**Experiment order** (done 2026-09-16/17 — results in the sections below; what
is left is in the [test plan](#test-plan-of-2026-09-17-and-its-results)) (each with both pipelines streaming at once — two
`rate_ladder.py --loop-timing` runs on ports 10000 and 10001 — at 1500, 1800,
2000 and 2400 Hz, 1000 and 2048 tones, counting events over 5–10 minutes):

1. Baseline with the instrumented server, no tuning.
2. `rt_tuning.sh apply` (CPU layout + send buffer).
3. `--journald-volatile`, and rate-limit the hot-path torn-read logging.
4. ZDMA per pipeline, once channel separation is validated.
5. Throughout: log `telescope_time` and the counter around every gap, to
   settle the 16-accumulation gaps below.

**If that is not enough:** move the wait and read into a dedicated reader
process per pipeline on its own core, feeding a shared-memory ring buffer that
the server sends from. The deadline then covers only the wait and read
(~30–80 µs at 1000 tones) and network or event-loop stalls fill the ring
instead of losing data. Beyond that, buffering accumulations in the firmware
and reading them in DMA batches removes the per-accumulation software deadline
and torn reads altogether — a firmware request.

#### Dual-pipeline baseline (2026-09-16 23:12, no tuning)

Both servers streaming 1000 tones at once (two `rate_ladder.py --loop-timing`
runs, ports 10000/10001), 3 × 60 s per point, software as installed 23:10
(PSB map fix + loop timing), default CPU layout, `wmem_max` 212 992 B,
`timing-monitor` stopped. Steady-state loss % per repeat, and small / large
gaps and torn reads summed over the 3 repeats:

| rate Hz | pipeline 0 loss % | p0 small / large / torn | pipeline 1 loss % | p1 small / large / torn | single pipeline at S3 (3 × 30 s) |
|---|---|---|---|---|---|
| 1500 | 0.365 / 0.314 / 0.205 | 450 / 47 / 13 | 0.195 / 0.228 / 0.246 | 272 / 42 / 5 | 0.000, no events |
| 2000 | 0.330 / 0.285 / 0.280 | 629 / 52 / 3 | 0.396 / 0.338 / 0.279 | 689 / 51 / 5 | 0.054 |
| 2400 | **8.19 / 4.23** / 0.400 | 17 285 / 66 / 39 | 0.352 / **1.82** / 0.245 | 2 876 / 50 / 12 | 0.302 (bimodal) |

**Running both pipelines costs the zero-event rate outright** — ~1 missed
accumulation every 200–300 ms at 1500 and 2000 Hz — and the knee moves below 2400 Hz.
Where the time goes (`loop_timing`, server side):

- **The read and frame build are unchanged at the median** — read 109 µs,
  frame 35 µs, the same as one pipeline — so the fabric and the bus are not
  the problem. Their p99.9 is 140–260 µs and their maxima reach 4–6 ms:
  preemption.
- **Almost every lost accumulation is a single one**, where the frame's busy
  time (read + build + the *outside* time before the next wait) passes the
  period, usually through an *outside* spike of 400–700 µs — sending,
  yielding, or being descheduled.
- **A strict 696–697-frame period** in those spikes, on both pipelines, at
  1500 *and* 2400 Hz. *Corrected after the tuned run:* not the send path — at
  100 tones (844 B frames) the spacing is still 697 frames, at the same frame
  numbers. It was **the measurement itself**: `loop_timing` kept one new tuple
  per frame in a deque, so Python's garbage collector ran every ~700 frames,
  a 0.2–0.5 ms pause (measured on the board: every 701 iterations, 210 µs,
  up to 1.5 ms; none with the collector off). Now a preallocated ring buffer
  (no pause in 30 000 frames).
- **At 2400 Hz, pipeline 0's bad repeats run in a contended regime**: read
  140–170 µs and build 58–72 µs instead of 109/35, missing every 4th frame.
  Two busy-waiting servers and all Ethernet interrupts (CPU0) sharing four
  cores. This is what `rt_tuning.sh` addresses.
- **Each capture starts with a ~0.7 s producer stall** (outside 650–720 ms,
  ~1000–1440 accumulations) a few frames after `enable_stream`, before the
  client receives its first frame — so the client never counts it. It would
  matter if a client attached to a stream already running.
- No 16-accumulation gap was recorded, but the event list (first 32 per
  capture) filled within a second; it now keeps gaps of 4+ and torn reads
  first.

#### Dual-pipeline, tuned (2026-09-16 23:27, `rt_tuning.sh apply`)

Same run as the baseline, after `rt_tuning.sh apply`: housekeeping and both
interrupt lines on CPU0–1, `readout_server_0` alone on CPU2, `readout_server_1`
alone on CPU3, `wmem_max`/`rmem_max` 32 MiB. Loop timing still had the
garbage-collector artefact above.

| rate Hz | pipeline 0 loss % | p0 small / large / torn | pipeline 1 loss % | p1 small / large / torn | baseline p0 · p1 loss % |
|---|---|---|---|---|---|
| 1500 | 0.095 / 0.053 / 0.180 | 123 / 19 / 0 | 0.020 / 0.055 / 0.053 | 106 / 1 / 0 | 0.21–0.37 · 0.20–0.25 |
| 2000 | 0.161 / 0.240 / 0.240 | 507 / 29 / 0 | 0.145 / 0.145 / 0.148 | 500 / 3 / 1 | 0.28–0.33 · 0.28–0.40 |
| 2400 | 0.286 / 0.298 / 0.302 | 822 / 41 / 1 | 0.166 / 0.171 / 0.171 | 698 / 3 / 0 | **0.40–8.2** · 0.25–1.8 |

- **The contention is gone.** Read p99.9 130–146 µs and frame build 39–58 µs
  (baseline 137–258 / 40–102), torn reads 77 → 2, and no repeat collapses at
  2400 Hz any more. Pipeline 1's read maximum is under 0.6 ms in 8 of 9
  captures; pipeline 0 still reaches 4.3–24 ms in 6 of 9 (the early reads
  below).
- **What is left is almost all the collector artefact**: 1-sample gaps every
  697 frames (500 per 3 min at 2000 Hz = one per ~720 frames), exceeding the
  period only when the pause lands on a busy frame — hence rare at 1500 Hz and
  near-certain at 2400 Hz.
- **Pipeline 0 has extra large gaps** (19–41 vs 1–3): reads of 2.0–4.4 ms
  (one scheduler tick) in the first few hundred frames of a capture. Its RF
  frontend is the obvious difference from pipeline 1, but both servers have
  the same three threads with similar CPU time. Open.
- **The ~0.7 s producer stall at each capture start** is unchanged.

#### Dual-pipeline, tuned, collector artefact removed (2026-09-16 23:49)

Same run again after the ring-buffer fix (servers restarted 23:48, tuning
kept). Loss % per repeat and gaps / torn summed over 3 × 60 s:

| rate Hz | pipeline 0 loss % | p0 small / large / torn | pipeline 1 loss % | p1 small / large / torn |
|---|---|---|---|---|
| 1500 | 0.106 / 0.100 / 0.105 | 1 / 24 / 1 | 0.086 / 0.078 / 0.010 | 11 / 19 / 1 |
| 2000 | 0.097 / 0.100 / 0.102 | 8 / 36 / 0 | 0.048 / 0.052 / 0.007 | 4 / 18 / 0 |
| 2400 | 0.186 / 0.194 / 0.138 | 300 / 29 / 1 | 0.150 / 0.138 / 0.056 | 465 / 4 / 1 |

Small gaps at 1500–2000 Hz fell from 106–507 to 1–11 per 3 min; `outside`
p99.9 from ~450 to ~250 µs; busy p99.9 ~400 µs.

- **What is left at 1500–2000 Hz is almost all at the edges of a capture**:
  gaps of 4–13 accumulations in the first ~500 frames and the last ~0.5 s,
  with a 2.0–4.5 ms `read` or `outside` — one or half a scheduler tick. That is
  the server's own worker threads (handling `enable_stream`, `set_acc_len`,
  `disable_stream`, `get_info`) preempting the frame producer, now that each
  server has exactly one core. It will hit any request served *during* a long
  stream — monitoring `get_info`, tracking, a second client.
- **In the middle of 60 s captures**, 0–3 single-sample gaps per capture at
  2000 Hz, and the counter glitches (3 in 18 min over both pipelines), each
  followed by the ~0.5 ms warning log.
- **2400 Hz is at the two-pipeline ceiling**: busy p99.9 ~400 µs against a
  417 µs period, 62–197 single-sample gaps per minute.

#### Changes after the tuned runs (2026-09-17, installed, not yet measured)

- **Counter double read** (above).
- **No torn-read logging in the frame path** — the ~0.5 ms
  `acc.logger.warning` in `read_accumulated_data_fast` and the `Packet error`
  print in `stream_data` are gone; the flag stays in every frame and in
  `loop_timing`.
- **The event-loop thread alone on its core.** With `SOUK_STREAM_CPUS` and
  `SOUK_WORKER_CPUS` set (by `rt_tuning.sh`), the server pins its event-loop
  thread to the stream CPU and every other thread — executor workers serving
  requests, katcp I/O — to CPU0–1. Aimed at the 2–4.5 ms stalls at capture
  edges.
- **Kernel housekeeping off CPU2–3**: `rt_tuning.sh apply` now also moves the
  movable kernel threads (`watchdogd`, the Cadence hardware-watchdog pinger,
  was on CPU3), unbound workqueues and new IRQs to CPU0–1. (This kernel has no
  soft-lockup detector, so no 4 s lockup timer.)
- **`get_samples` warm-up is now a request option** (`warmup=`, default 50
  discarded accumulations; `rate_ladder.py --warmup`), to check whether it
  hides a start-up effect.

#### Dual-pipeline, fully isolated (2026-09-17 02:09) — worse

After `rt_tuning.sh apply --journald-volatile` with all the changes above
(counter double read, no torn-read logging, event-loop thread alone on its
core with the other server threads on CPU0–1, kernel threads / workqueues /
IRQs on CPU0–1, journal in RAM). Gaps / torn summed over 3 × 60 s, with the
previous run (tuned, collector artefact removed) for comparison:

| rate Hz | pipeline | loss % per repeat | small / large / torn | previous run small / large / torn |
|---|---|---|---|---|
| 1500 | 0 | 0.119 / 0.138 / 0.112 | 101 / 21 / 0 | 1 / 24 / 1 |
| 1500 | 1 | 0.087 / 0.092 / 0.103 | 97 / 11 / 0 | 11 / 19 / 1 |
| 2000 | 0 | 0.131 / 0.146 / 0.126 | 96 / 76 / 0 | 8 / 36 / 0 |
| 2000 | 1 | 0.101 / 0.100 / 0.091 | 79 / 54 / 0 | 4 / 18 / 0 |
| 2400 | 0 | 0.540 / 0.548 / 2.411 | 1873 / 78 / 0 | 300 / 29 / 1 |
| 2400 | 1 | 0.194 / 0.204 / 1.731 | 522 / 44 / 0 | 465 / 4 / 1 |

- **Torn reads went to zero** in all 18 captures (the double read removes the
  false ones), and no counter stepped backwards.
- **Gaps at the capture edges got worse**, now with 1.8–2.4 ms `read` *and*
  `outside` times, where before they were whole scheduler ticks. Likely the
  GIL: the server's other threads now run *in parallel* on CPU0–1 instead of
  taking turns with the event loop on one core, so while one runs Python code
  the event loop waits for the interpreter lock (switch interval 5 ms). Pinning
  threads apart cannot help Python threads that need the same lock.
- **At 2400 Hz, pipeline 0 gained a regular gap every 254 frames (~106 ms)**,
  cause unknown.
- **In repeat 3 at 2400 Hz, both pipelines stalled together** for 56–236 ms,
  several times within ~5 s (`outside`, i.e. sending). Both servers at once
  points at the network path or the host writing ~40 MB/s of capture files;
  not seen in earlier runs.

#### Dual-pipeline, whole servers on their cores again (2026-09-17 06:44)

Thread splitting off (`rt_tuning.sh apply --journald-volatile`: each server
process wholly on CPU2 / CPU3), everything else kept. Small / large gaps, torn
reads and total lost over 3 × 60 s:

| rate Hz | pipeline | tuned, before the changes | split threads | **whole servers** |
|---|---|---|---|---|
| 1500 | 0 | 1 / 24 / 1, lost 276 | 101 / 21 / 0, lost 330 | **0 / 27 / 0, lost 266** |
| 1500 | 1 | 11 / 19 / 1, lost 154 | 97 / 11 / 0, lost 249 | **9 / 20 / 0, lost 152** |
| 2000 | 0 | 8 / 36 / 0, lost 354 | 96 / 76 / 0, lost 477 | **22 / 22 / 0, lost 362** |
| 2000 | 1 | 4 / 18 / 0, lost 126 | 79 / 54 / 0, lost 345 | **10 / 11 / 0, lost 111** |
| 2400 | 0 | 300 / 29 / 1, lost 737 | 1873 / 78 / 0, lost 5036 | 1237 / 5 / 0, lost 1420 |
| 2400 | 1 | 465 / 4 / 1, lost 488 | 522 / 44 / 0, lost 3018 | 1004 / 1 / 0, lost 1012 |

- **Splitting the threads was the regression**; whole servers are back to the
  earlier numbers at 1500–2000 Hz, with no torn reads.
- **At 2400 Hz both pipelines lose an accumulation every 254 frames
  (106 ms).** The earlier run had the same spacing, hit less often. It is the
  `triggered_stream` task, which polled `asyncio.sleep(0.1)` on the event loop
  whenever triggered streaming was off; with the frame's busy time at
  ~400 µs of a 417 µs period, each wake costs a frame. Now it waits on its
  event instead (installed 06:54, servers restarted 06:58). `stream_data`'s own 0.1 s
  idle poll remains — it applies queued modulation disable/rest commands, so it
  was left alone; it runs only while the stream is off, i.e. during
  `get_samples`.
- **What remains at 1500–2000 Hz is at the capture edges** (request handling
  at start and end), plus a gap every 4 s on pipeline 0 at 2000 Hz
  (5429 → 13428 → 21427 frames), source still unknown.

#### Dual-pipeline, trigger poll removed (2026-09-17 06:58)

Same run, servers restarted with `triggered_stream` waiting on its event.
Total lost over 3 × 60 s, and loss % per repeat:

| rate Hz | pipeline | whole servers (06:44) | **trigger poll removed** | loss % per repeat |
|---|---|---|---|---|
| 1500 | 0 | 266 | **216** | 0.076 / 0.016 / 0.016 |
| 1500 | 1 | 152 | **106** | 0.072 / 0.044 / 0.003 |
| 2000 | 0 | 362 | **194** | 0.016 / 0.016 / 0.025 |
| 2000 | 1 | 111 | **26** | 0.008 / 0.009 / 0.004 |
| 2400 | 0 | 1420 | **187** | 0.030 / 0.031 / 0.025 |
| 2400 | 1 | 1012 | **209** | 0.057 / 0.066 / 0.024 |

**Two pipelines at 2400 Hz (4× the campaign rate) now lose 0.02–0.07%**, and
no torn reads. Most of what the client counts is at the capture edges. In the
middle of the captures, the few single-sample gaps left come on fixed periods:

- **every 30 s** (frame ~60 000 at 2000 Hz, both pipelines): the stream-client
  handler's `readline` timeout while streaming (`STREAM_KEEPALIVE_STREAMING_S`),
  ~330 µs of `outside`;
- **every 1 s** at 2400 Hz (2399-frame spacing), ~160 µs of extra `outside`;
  not the handler's `reader_eof` branch (the client never half-closes). A
  candidate is the kernel's once-a-second per-CPU `vmstat_update`
  (`vm.stat_interval`), which cannot be moved off the core;
- **every 4 s**, mostly pipeline 1: the accumulator read ~200 µs slower
  (7999 / 9599-frame spacing). Kernel work on the core, source not yet found.

Busy time is now p50 ~210–270 µs, p99.9 ~335–390 µs, against 417 µs at
2400 Hz.

#### One pipeline, current software, tuned (2026-09-17 10:11)

The TL;DR's one-pipeline ladder again, on the software and CPU layout of the
final dual run (pipeline 1's server running but idle), with ZDMA on
(10:11, read p50 86 µs) and off (10:38, 110 µs) — the only difference between
the two columns. 1000 tones, 3 × 30 s per point; delivered frames/s · loss %,
then lost samples per repeat:

| rate Hz | ZDMA off | lost | ZDMA on | lost |
|---|---|---|---|---|
| 600–1500 | clean | 0 in 15 captures | clean | 0 in 15 captures |
| 1800 | 1801.8 · 0.001 | 1 / 0 / 0 | 1801.8 · 0.000 | 0 / 0 / 0 |
| 2000 | 2000.0 · 0.000 | 0 / 0 / 0 | 2000.0 · 0.000 | 0 / 0 / 0 |
| 2400 | 2399.9 · 0.006 | 3 / 4 / 6 | 2400.0 · 0.001 | 1 / 1 / 0 |
| 3000 | 2995.4 · 0.155 | 150 / 133 / 125 | 2998.8 · 0.039 | 36 / 37 / 30 |
| 4000 | 2050.6 · 48.7 | halves | 3478.6 · 13.0 | |
| 5000, 6000 | ~rate/2 | | ~rate/2 | |

No torn reads below 5000 Hz in either. Against the same ladder on SW-e / SW-f
(2026-09-16, untuned): ZDMA off keeps the 1500 Hz zero-event rate but 2400 Hz
drops from 0.30% to 0.006%; ZDMA on goes from no zero-event rate to 2000 Hz,
and 3000 Hz from 1.0% to 0.04%.

#### The servers' read slowed by ~50 µs after a second tone set (2026-09-17, fixed)

The next run (07:16, after `sysctl vm.stat_interval=10`, servers not
restarted) came out far worse — 2400 Hz lost 2–5.6%, torn reads back — because
the server's `read` p50 had stepped from 111 to 160–165 µs on both pipelines at
every rate. It was **not** the sysctl, the board, or the other pipeline:

| check | read p50 |
|---|---|
| servers, 06:58 run (fresh) | 110–112 µs |
| servers, 07:16 run and after (not restarted) | 157–165 µs, also after reverting the sysctl |
| same `read_accumulated_data_fast` call in a separate process (CPU1, CPU2 or CPU3) | 107.6 µs |
| raw 8000 B read, right after an increment or half a period later | 48 µs either way |
| both pipelines read at once from two processes, four different phase offsets | 106–108 µs at every offset |
| server page faults while streaming | 0/s |
| servers restarted 08:53, checked at once | **111–112 µs** (2400 Hz loss 0.04–0.12%) |

So something inside the long-running server processes makes the same read
~50 µs slower, and a restart clears it.

**Found (09:21): the second tone set.** Fresh servers stayed fast after 10 min
idle (112 µs) and with `vm.stat_interval=10` (111 µs), but setting the same
1000 tones a second time made them slow at once (160–161 µs, 2400 Hz loss
1.5–2%). `_cached_read_plan` keeps the read plan with the tone-index array it
was made for and checks identity first, then contents. Each tone set gives the
server a *new* array with the same contents: the content compare matched and
returned the cached plan without storing the new array, so every later read
paid `np.array_equal` over 1000 indices — **35 µs on the A53** (identity check
1.1 µs) — until a restart. Every "fresh server" run had set tones exactly once.
Fixed: a content match now stores the new array. The `vm.stat_interval` run and
the 07:3x checks above were all on servers in this state; the sysctl had no
effect. Verified 09:40 after a restart: 111–112 µs after one tone set and still
111 µs after a second.

#### Dual-pipeline, final (2026-09-17 09:41, all fixes, tuned)

Both servers streaming 1000 tones at once, 3 × 60 s per point, every fix above
loaded, `rt_tuning.sh apply --journald-volatile` (whole servers on CPU2 / CPU3),
`vm.stat_interval` back to 1. Against the untuned baseline of the evening
before:

| rate Hz | pipeline | baseline (23:12): lost, small / large / torn | **final: lost, small / large / torn** | final loss % per repeat |
|---|---|---|---|---|
| 1500 | 0 | 783 (0.29%), 450 / 47 / 13 | **267 (0.10%), 4 / 32 / 0** | 0.067 / 0.101 / 0.099 |
| 1500 | 1 | 593 (0.22%), 272 / 42 / 5 | **228 (0.09%), 9 / 30 / 0** | 0.085 / 0.086 / 0.087 |
| 2000 | 0 | 1060 (0.30%), 629 / 52 / 3 | **374 (0.11%), 7 / 44 / 0** | 0.101 / 0.103 / 0.105 |
| 2000 | 1 | 1196 (0.34%), 689 / 51 / 5 | **272 (0.08%), 10 / 34 / 0** | 0.090 / 0.090 / 0.051 |
| 2400 | 0 | 18 238 (4.3%), 17 285 / 66 / 39 | **482 (0.11%), 37 / 41 / 0** | 0.108 / 0.108 / 0.108 |
| 2400 | 1 | 3419 (0.80%), 2876 / 50 / 12 | **283 (0.07%), 231 / 8 / 0** | 0.066 / 0.085 / 0.049 |

- **No torn reads** in 18 captures (baseline 77).
- **What the client counts is now mostly the edges of each capture** — the
  large gaps (8–15 per capture) at start and end while the test's own requests
  are served. Server loop timing over the middle of the captures:
  - pipeline 0: no lost accumulation at 1500 Hz, 0–1 per capture at 2000 Hz,
    7–9 over the last 55 s at 2400 Hz (edge included) — **~0.005% steady
    state at 4× the campaign rate, on two pipelines**;
  - pipeline 1: the same at 1500–2000 Hz, but at 2400 Hz 71–155 per 55 s,
    mostly one accumulation every 4 s (9599-frame spacing) with nothing slow in
    the frame's own timing — the 4 s kernel event on its core, still
    unidentified;
  - both: one single gap every 30 s at ≥ 2000 Hz (the stream-client
    keepalive timeout).
- The 06:58 run (before the cache fix, but on freshly set tones) measured
  similar steady-state numbers and fewer edge gaps; the edge counts vary run to
  run with request timing.

**Still open for zero events:** the 4 s kernel event on CPU3, the 30 s keepalive
wake, the capture-edge cost of serving requests during a stream, and the
~0.7 s producer stall at stream start. None has been measured over an hour.

#### Dual-pipeline with ZDMA on both (2026-09-17 11:28)

First run of ZDMA on two pipelines at once: pipeline 0 on DMA channel 0 /
`udmabuf0`, pipeline 1 on channel 1 / `udmabuf1`. Channel 1 was validated first
(`profiling/zdma_channel_check.py 0 udmabuf1 1`, as root: pipeline 0's
accumulation read through channel 1 matched devmem in all 2000 words, 17 µs per
read). Same software (installed 09:23) and tuning as the final run above; both
servers restarted 11:23 with ZDMA, read p50 86–88 µs. 3 × 60 s per point:

| rate Hz | pipeline | no ZDMA (09:41): lost, small / large / torn | **ZDMA on both: lost, small / large / torn** |
|---|---|---|---|
| 1500 | 0 | 267 (0.100%), 4 / 32 / 0 | **112 (0.042%), 1 / 13 / 14** |
| 1500 | 1 | 228 (0.086%), 9 / 30 / 0 | **0 (0.000%), 0 / 0 / 0** |
| 2000 | 0 | 374 (0.105%), 7 / 44 / 0 | **21 (0.006%), 2 / 0 / 1** |
| 2000 | 1 | 272 (0.077%), 10 / 34 / 0 | **12 (0.003%), 12 / 0 / 0** |
| 2400 | 0 | 482 (0.113%), 37 / 41 / 0 | **125 (0.029%), 37 / 6 / 5** |
| 2400 | 1 | 283 (0.066%), 231 / 8 / 0 | **34 (0.008%), 34 / 0 / 0** |
| 3000 | 0 | — | 4632 (0.87%), 4202 / 28 / 9 |
| 3000 | 1 | — | 2279 (0.43%), 2276 / 0 / 0 |

- **ZDMA cuts two-pipeline loss 3–30× at 1500–2400 Hz.** Pipeline 1 at
  1500 Hz lost nothing in 3 minutes; both pipelines at 2400 Hz (4× the campaign
  rate) stay under 0.03%.
- **Busy time p99.9 falls** from ~365–395 to ~314–353 µs; 3000 Hz (333 µs
  period) is still past the two-pipeline limit.
- **Pipeline 0's torn reads (14 at 1500 Hz) and large gaps are at capture
  starts**: a 20 s check had all 23 events in the first 0.7 s (reads stalled
  1.3–16.5 ms, i.e. scheduler ticks), none in the remaining 18.6 s. Pipeline 0
  serves more work at stream start (RF frontend). Not DMA contention.
- Pipeline 1's ADC input is open circuit, so its accumulations are often all
  zero; the timing is still representative, and transport correctness was
  checked on pipeline 0's data.

#### The 16-accumulation gaps

**Caught in loop timing (tuned run, pipeline 0, 2400 Hz): it is a bad counter
value, not lost data.**

| frame | read µs | counter step from previous frame | torn |
|---|---|---|---|
| 87172 | 108 | +1 | 0 |
| 87173 | **604** | **+17** (reads as 16 missed) | **1** |
| 87174 | 126 | **−13** (counter went backwards) | 0 |

About one period of real time passed between 87172 and 87173, yet the counter
read 17 higher, then stepped back — so the value read was wrong, and only ~2
accumulations were really missed (the slow read). Pipeline 1 at 2000 Hz shows
the same signature at +4 / −1. Both are torn frames whose read took ~600 µs
instead of ~110 — but that extra ~490 µs is the software reacting, not the bus:
the only extra work on a torn read is `acc.logger.warning('Accumulation counter
changed while reading data!')` inside `read_accumulated_data_fast` (5 of 5
torn frames show it). The counter value itself fits a read of `acc_cnt`
catching a mix of old and new bits while it increments (a clock-domain crossing without a safe handshake),
first seen with the v7.12 `.fpg` (S1). The client's counter-based loss counts
therefore overstate these events, and data from the frame itself may be torn.
**Measured directly (2026-09-17 00:00, `profiling/acc_cnt_probe.py`):** a tight
read loop over UIO, both pipelines at 6000 Hz for 10 min, servers idle:

| pipeline | counter changes | impossible jumps | steps backwards | rate |
|---|---|---|---|---|
| 0 | 3 594 661 | 63 | 48 | 1 per ~32 000 increments |
| 1 | 3 590 254 | 26 | 26 | 1 per ~69 000 increments |

The values are mixes of the old and new count, e.g. `0x…8af` → `0x…8a0` →
`0x…8b0` (low nibble already rolled over, bit 4 not yet) and `0x…dfdb` →
`0x…dfdf` → `0x…dfdc`; the bad value is never read twice. A multi-bit counter
sampled mid-transition — a clock-domain-crossing fault for upstream, not seen
at S0 (v7.11) although never probed there.

**Software fix:** `read_acc_cnt_fast` now reads until two consecutive reads
agree (one extra ~4 µs read), removing the false gaps and the false torn-read
flags. Checked with the probe's `--double-read` (02:19, same conditions):

| pipeline | counter changes | steps backwards | "jumps" |
|---|---|---|---|
| 0 | 3 594 740 | **0** (raw read: 48) | 17 |
| 1 | 3 595 603 | **0** (raw read: 26) | 6 |

The remaining "jumps" are the probe's own artefact: every one follows a
0.2–3.4 ms descheduling of the probe (it shares CPU0–1 with the interrupts and
housekeeping), the value persists afterwards, and steps are 2–20 — a delay
between taking the reading and timestamping it, not a bad value. In the
dual-pipeline run with the double read, no torn read and no backwards counter
step occurred in 18 captures.

Across every ladder (S0–S3, ZDMA, SW-a/c/g), 23 captures with ≤ 1% loss held
a single gap of more than 3 samples. **11 of those gaps are exactly 16
samples**, each with a `packet_error` (one, in 10 of the 11), at 600, 800, 1200, 1500, 1800 and 3000 Hz, on the
stream and `get_samples`, at 100, 1000 and 2048 tones, with and without ZDMA.
Two S1–S2 captures held pairs summing to 32 and 48 (largest 16 and 32), and
S1's 340 ms gap at 800 Hz was 272 = 17 × 16 samples. None of S0's large gaps
was a multiple of 16. The other single gaps are 4–7, 10 and 20 samples.

A scheduling or I/O stall loses a *time*, so its sample count would scale with
rate; these lose a fixed number of accumulations. Nothing in the server waits
on a count after a torn read (`stream_data` only prints). The simplest reading
is that the CPU's read is held off in the fabric for 16 accumulations — the
counter moves during it, hence the one torn read — but a glitch on the
accumulation counter itself would look similar in these statistics. The
events come about once per 25 min of capture, so each check needs roughly an
hour. Two ways to tell, both fitting inside the dual-pipeline runs:

- `get_info('loop_timing')['events']` keeps the timing rows of the frame
  before and the frame at each gap or torn read. A `read` of ~16 periods on
  the torn frame means the bus held the CPU off; a `wait` of ~16 periods means
  a bad counter value started the wait; a huge `missed` means the counter
  stepped backwards.
- In a kept capture, `telescope_time` (the accumulator's own time tag) across
  the gap: a real loss steps it by 17 periods along with the counter.


### LO control writes through a copy in memory (2026-09-17)

**Why.** On v7.12.1 one contiguous copy into an LO control BRAM costs 17.4 µs
for 1000 tones (4000 words), against 98.6 µs for the per-word stores the
software used; the old "staged" path lost only because it re-scattered the
values in Python first (82 µs). And amplitudes and phases went over katcp,
~1.3 s per request.

**Design** (`firmware_lib`, uncommitted, installed ~12:05):

- `get_control_shadow(r_fast)` reads every LO control register once into RAM —
  one `(n_slots, n_chans, 4)` word array per LO and ping-pong buffer — at
  server interface creation (only when the fabric is healthy: register access
  on an unclocked fabric hangs the PS).
- `prepare_control_buffer_data_fast` returns the words per field (phase step,
  RI step, phase offset, scaling) plus the tone rows;
  `write_control_buffer_data_fast` places each field with one strided copy and
  pushes the touched tone rows as one 64-byte-aligned slice (whole slot when
  the rows are not contiguous). `sparse_write` / `plan_control_buffer_write`
  are gone.
- `set_tone_amplitudes` / `set_tone_phases` take `r_fast=` and write through the
  copy wherever the server and library call them; katcp remains the fallback.
- `check_control_shadow` / `get_info('control_shadow')` compare the copy with
  the firmware and list any word another writer changed — which the next fast
  write would otherwise undo.

**Measured on the board** (`profiling/control_write_check.py`, pipeline 1,
1000 tones, TX + RX, one buffer, p50):

| write | per-word stores (before) | through the copy |
|---|---|---|
| frequency (phase step + RI step) | 247 µs | **110 µs** |
| all four fields | 354 µs | **140 µs** |
| scaling for all 2048 rows (what `set_tone_amplitudes` writes per buffer) | 251 µs | **134 µs** |
| frequency, 1000 non-contiguous tones | 248 µs | 236 µs |
| the bulk copy alone, 4000 words | — | 18.8 µs |

Per LO the rest is ~8 µs of field placement and ~13 µs of Python. Through the
live server (pipeline 1):

| request | before | after |
|---|---|---|
| `set_tone_amplitudes` | 1.26–1.34 s | **6 ms** |
| `set_tone_phases` | 1.26 s | **7–9 ms** |
| `set_tone_frequencies` | — | 30 ms (first call after a restart: 4.8 s, see below) |

**Verified:** a katcp read of all four registers equals the copy (catches a
byte-order error a devmem-only round trip would hide); `board_checks.py sparse`
(one tone rewritten, neighbours and the other LO untouched, copy == BRAM);
`get_info('control_shadow')` showed 0 stale words on both pipelines after the
restart, and still 0 after amplitude, phase and frequency requests, an 11-point
sweep, firmware modulation on and off, software modulation with a 1000-sample
capture, and a restore. A word changed from a separate process was reported
exactly (slot 3, tone 5, scaling, both values) and cleared once restored. Tone
state on pipeline 1 was restored exactly; pipeline 0 had read-only checks.

**Found along the way (not caused by the copy):**

- `perform_sweep` froze pipeline 1's ADC calibration even with
  `refresh_adc_cal=False` (unfrozen again afterwards).
- After a server restart, the first `set_tone_frequencies` without cached tone
  state rewrites the phases from the config plan (existing behaviour).
- That first `set_tone_frequencies` took **4.8 s**, later ones 30 ms; the word
  writes are microseconds, so the time is elsewhere.

**Limits:** the copy is only as good as the rule that nothing else writes
these registers. Host-side katcp or control_sw `mixer.set_*` calls are detected
(`get_info('control_shadow')`), not prevented. Code still holding an old
`r_fast` after `ensure_ready` would be a second writer (the modulation scheduler
was re-pointed). Reads still come from the firmware, not the copy. A DMA write
only pays for a whole 4-slot register push (> 38 KiB).

Rollback on the board: `~/srt_test/backup_20260917_shadow/` (the previous
`firmware_lib.py`, `readout_server.py`, `board_checks.py`).

### Test plan of 2026-09-17, and its results

The board runs the RAM-copy build (installed ~12:05) with ZDMA on both
pipelines and the `rt_tuning.sh --journald-volatile` layout. Every streaming
number in this document predates the copy, so re-check first, then use it.

| # | test | why | how | needs |
|---|---|---|---|---|
| 1 | Streaming regression on the RAM-copy build | the frame path should be untouched, but the server changed | repeat the dual ZDMA ladder (1500/2000/2400/3000 Hz, 3 × 60 s) and one single-pipeline ladder; compare with the 11:28 and 10:11 runs | nothing |
| 2 | Retunes during a stream | requests run on the event loop: a 1.3 s katcp amplitude write stalled the stream for 1.3 s; 6 ms still spans 6–15 periods | stream at 1000/2000 Hz and issue `set_tone_frequencies` / `set_tone_amplitudes` / `set_tone_phases` every few seconds; count lost accumulations per request in `loop_timing` | nothing |
| 3 | Tone update and sweep timing end to end | the per-point write fell 247 → 110 µs; how much of a sweep and a power-sweep step that is | time `perform_sweep` per point and a `set_tone_powers` step, against the pre-copy finding that ~75% of a power-sweep step was `set_tone_powers` (optimisers plus redundant phase writes over katcp) | nothing |
| 4 | First `set_tone_frequencies` after a restart: 4.8 s | first retunes after a restart matter operationally | profile that call in the server (restart, then one timed request) | restart (sudo) |
| 5 | `perform_sweep` freezes ADC calibration with `refresh_adc_cal=False` | could change RX data between sweeps | read the calibration freeze state before and after a sweep on pipeline 0 and 1; read the code path | nothing |
| 6 | B5 on the RAM-copy build | `board_checks.py` now checks writes through the copy | `sudo board_checks.py` with servers stopped | sudo, servers stopped |
| 7 | Copy staleness from outside writers | host scripts using control_sw over katcp are detected, not prevented | a katcp `mixer.set_amplitude_scale` from the host, then `get_info('control_shadow')` and one fast write; decide whether the server should re-read on detection | nothing |
| 8 | B7 timed sync round trip | never run on the 200 MHz clock | `host_checks.py sync` with `timing-monitor` running | sudo (start service) |
| 9 | B8 sweep + fit on real resonators | the check that would have caught the PSB map swap | same resonator as a v7.11 reference, same cooldown | cryostat |
| 10 | Hour-long soak, both pipelines | zero-event rates so far rest on 90–180 s per point | 1 h at 1500 and 2000 Hz with ZDMA, events counted | ~2 h of board time |
| 11 | Remaining small items | from the zero-drop work | ~~warm-up 50 vs 0~~, ~~2048-tone ladder~~, ~~stream-start stall~~ done; the 4 s event (perf while both stream at 2400 Hz) and the 30 s keepalive wake are quantified by the soak and await a fix | root for the perf run |
| 12 | `fpga` info over the fast path | `fpga.is_programmed` / `get_status` go through tcpborphserver and stall both pipelines' reads ~2.2 ms per call (five per ladder start) | read the same registers over the fast path, same return format; re-run the cross-pipeline reproduction | nothing |

Commit the uncommitted tree after tests 1–2 (the PSB map fix first).

#### Results so far (2026-09-17 12:20–13:50, RAM-copy build)

**Test 1b — one pipeline, ZDMA:** identical to the 10:11 run: no lost or torn
sample from 600 to 2000 Hz, 0.000–0.004% at 2400 Hz, 0.04% at 3000 Hz, 13% at
4000 Hz. No regression.

**Test 1a — both pipelines, ZDMA:** ~0.10% at 1500 and 2000 Hz on both
pipelines (11:28 run: 0.000–0.042%), 0.14–0.17% at 2400 Hz, 0.9–2.4% at
3000 Hz, with 12–15 large gaps and ~10 torn reads per capture. Every one of
those came in **one ~0.5 s burst per capture**, and the burst on each pipeline
fell when **the other pipeline's capture started** (the two test processes ran
~5 s apart this time, because the first `set_tone_frequencies` after the
restart took ~5 s on pipeline 0; at 11:28 they were aligned, so the same
disturbance fell at capture edges, where the client does not count it).
Reproduced directly: pipeline 0 streaming alone, a 20 s pipeline 1 capture
started mid-way → a 0.7 s burst on pipeline 0 (reads and outside time of
2.1–4.4 ms, 95 lost, 7 torn), nothing when pipeline 1's capture ended.
Individual pipeline 1 requests (`set_acc_len`, `enable_stream`,
`disable_stream`, `get_info`) did not reproduce it; the stream client
connecting and data starting to flow did. During those first ~0.6 s, CPU2
and CPU3 take bursts of function-call IPIs — kernel work on one core that
stops every core briefly. No memory compaction or page migration counted; one
`khugepaged` huge-page collapse did (THP is `madvise`, and numpy madvises
large arrays). **Cause not yet confirmed.** Operationally: starting a stream on
one pipeline costs the other pipeline ~50–100 accumulations.

Ruled out since (13:50–14:10): transparent huge pages (`never` — burst unchanged,
96 lost, 14 torn); any other thread taking CPU2 or CPU3 (nanosecond run times
from `/proc/*/task/*/schedstat`: every other thread there, kernel threads
included, ran ≤ 0.1 ms per 100 ms during the burst); memory compaction and page
migration (no counts). What remains is interrupt-level work charged to the
frame producer itself, consistent with the function-call IPIs seen at the same
moment; identifying it needs kernel tracing (ftrace or perf, root). Until then:
start streams on both pipelines together, and avoid starting or stopping a
stream on one pipeline while the other records.

**Root cause found (2026-09-17 14:00–14:40).** The kernel has no ftrace, so a
perf-event sampler (`profiling/perf_cpu_sampler.py`, root) sampled CPU2/3 at
5 kHz: server 0's thread never left its core and stayed in its own code during
the burst. A bare read loop on an idle CPU2 (`bus_stall_probe`) then showed
2–8 ms stalls on bus reads of pipeline 0's accumulator — but not on RAM copies
of the same size — only while pipeline 1's capture started, and only once its
stream client connected (not on `get_info` versions, `set_acc_len`, or
`enable_stream` alone); a plain 118 MB/s TCP send from the board caused none.
The client's `receive_stream` calls **`get_info('all')`** when it starts, to
save the full server info with the capture. Timed per section on pipeline 1,
with the read loop running:

| section | request | slow bus reads (> 1 ms) | worst |
|---|---|---|---|
| all | **697 ms** | 9 | 8.1 ms |
| tones | 216 ms | 5 | 4.3 ms |
| rfdc | 49 ms | 3 | 4.3 ms |
| calibrations | 23 ms | 2 | 4.4 ms |
| fpga | 81 ms | 2 | 2.1 ms |
| control_shadow (the RAM copy check) | 11 ms | 0 | — |
| all others | ≤ 46 ms | ≤ 1 | ≤ 2.5 ms |

So:

1. **Every stream start stalls its own frame producer ~0.7 s** — the
   `get_info('all')` runs on the server's event loop. The client is not yet
   receiving, so it never counts the loss (the "~0.7 s producer stall at stream
   start" of the dual runs).
2. **The info reads some control registers that answer slowly** (the `tones`,
   `rfdc`, `calibrations` and `fpga` sections), and the processor-to-fabric
   bus serves one request at a time, so **both pipelines' accumulator reads wait
   2–8 ms** meanwhile. That is the ~0.5–0.7 s burst on the other pipeline
   (~95 accumulations). It is not the RAM copy, DMA, huge pages, scheduling or
   the network, and it predates this build.

Any `get_info` with those sections — from any client, on either pipeline —
costs both pipelines' streams accumulations.

**Narrowed to two driver calls (14:50).** Plain register reads and writes over
katcp or devmem cause no stall (100 small reads, 20 reads of 4 KB, 20 writes:
clean). With the read loop on CPU2, pipeline 1 calling (5× each):

| call | time | slow bus reads | worst |
|---|---|---|---|
| `rfdc.core.get_mixer_settings` | 20 ms | 2 | 9.9 ms |
| `fpga.get_status()` | 280 ms (56 ms each) | 5, one per call | 2.2 ms |
| `fpga.is_programmed()` | 275 ms | 5, one per call | 2.2 ms |
| `info_rfdc` (includes the mixer settings) | 82 ms | 6 | 9.8 ms |
| `get_dsa`, `get_output_current`, `get_nyquist_zone`, `get_qmc_settings` | 7–8 ms each | 0 | — |

Both go through tcpborphserver into the converter driver and the FPGA-manager
status, which evidently hold the processor-to-fabric bus while they run. The
`fpga`, `rfdc`, `tones`, `calibrations` and `server` info sections reach them
(directly or through `get_tone_frequencies`).

**Caching tried and reverted (14:40–15:00).** Remembering the mixer settings
(cleared wherever the server sets a mixer) and answering `get_info('tones')`
from held tone state while a stream client received cut the loss. Pipeline 0
streamed 60 s at 1500 Hz without ZDMA while pipeline 1's ladder started twice:

| pipeline 0, per pipeline 1 ladder start | no cache (14:09) | cached (14:46) |
|---|---|---|
| lost accumulations | 93 | 15 |
| gaps | 5 small + 12 large | 5, of 3 each |
| longest stall | 4.7 ms | 2.2 ms |

The five left were one per FPGA-status call: `get_info(['versions', 'fpga'])`
makes two (`is_programmed`, `get_status`), `get_info('all')` three (`server`
one, `fpga` two). **Reverted:** a cache can report stale hardware state. The
direction instead is to read what the `fpga` section needs over the fast path,
in the same format ([test 12](#test-plan-of-2026-09-17-and-its-results)). The mixer
settings, the larger stall, still go through the driver.

Kept: `receive_stream(info=)`, with `rate_ladder.py` fetching the info before
`enable_stream()`. The client now connects straight away, so a ladder's
recording includes the stream's first read (2.95 ms, torn) and the 5
accumulations missed behind it. Before, the 0.7 s fetch delayed the connection
and hid them.

**Test 2 — tone requests during a stream** (pipeline 0, 1000 tones, one
request every 5 s re-sending the loaded values, 9 of each):

| request | round trip | 1000 Hz: lost per request | 2000 Hz: lost per request | longest loop stall |
|---|---|---|---|---|
| none (baseline) | — | 0 | 0 | 4.8 ms (capture start) |
| `set_tone_frequencies` | 42–46 ms | 25 | 56 | 23 ms |
| `set_tone_amplitudes` | 29–34 ms | 11 | 28 | 10 ms |
| `set_tone_phases` | 30–32 ms | 12 | 31 | 10 ms |

Tone requests run on the event loop that produces the frames, so each one
stops the stream for 10–23 ms. Before the RAM copy an amplitude or phase request
stopped it for ~1.3 s. Lossless retunes during a stream need the request work
moved off the frame producer's path.

**Test 4 — the slow first `set_tone_frequencies` after a restart (15:06).**
`profiling/first_retune_profile.py` (servers stopped, pipeline 1, CPU3) runs
the call twice in a fresh process under cProfile: **8.66 s, then 17 ms**. Of
the 8.66 s, **8.52 s is `import scipy.signal`**, done lazily by
`_filterbank_prototype` the first time filterbank compensation runs (4.9 s of
it is scipy parsing its own docstrings). The prototype needs only
`scipy.signal.windows.dpss`, but importing any submodule runs the package
`__init__`. The server now builds the prototype in `main()`, before it serves
requests. Startup is ~8.5 s longer; after the 15:10 restart the first call
took **31 ms (pipeline 0) and 30 ms (pipeline 1)**, the second 29 ms. The tones
were re-sent and restored exactly.

**Test 5 — `perform_sweep` freezing ADC calibration.** By design since
`a1dcaaf` (2026-04-21). A sweep always freezes the calibration before
sweeping and leaves it frozen. `refresh_adc_cal=True` first unfreezes it,
settles and freezes again; `False` skips that, settling first only if the
calibration was not already frozen. Not a regression; no board test needed.

**Test 6 — B5 on the RAM-copy build (15:05, servers stopped).** All pass:
transports (devmem and katcp accumulations identical, 4096 words), inmap
(all three write/read pairs, restored), sparse (8 LO/buffer/slot cases
isolated and restored, copy clean afterwards), katcp_range.

**Test 3 — sweep and power-step timing through the server (15:13, pipeline 1,
1000 tones, 1500 Hz, 100 kHz spans, 10 samples per point, settle 4).**
`profiling/sweep_step_timing.py`; state restored exactly.

| | time |
|---|---|
| sweep, 11 / 101 / 401 points | 0.98 / 2.88 / 8.78 s |
| per point | **20.0 ms**, of which 14 accumulations of data and settling are 9.3 ms |
| fixed per sweep | 0.76 s |
| `set_tone_powers`, optimisers off (`optimise_dynamic_range=False`, `rx_policy='none'`) | 1.05 s |
| `set_tone_powers`, defaults | 9.36 s |

The per-point write is ~0.1 ms of the 20 ms. The other ~10.7 ms per point has
not been broken down; `chanmap_settle_accumulations=4` alone is another 2.7 ms
whenever a channel map changes. A power-sweep step is now dominated by
`set_tone_powers`'s optimisers (9.4 s), not by tone writes. The 2026-08
estimate was ~43 s of a 57 s step outside data taking, mostly three katcp
phase writes and the optimisers. Even without optimisers `set_tone_powers`
takes a second; not profiled.

**Test 7 — a katcp write from outside the server (15:14, pipeline 1).**
`profiling/shadow_outside_writer_check.py` wrote one TX scale word (channel 5,
live buffer, slot 0) with control_sw `mixer.set_amplitude_scale`, from a
separate process:

1. The server's `get_info('control_shadow')` reported it exactly: 1 word,
   slot 0, tone 5, `scaling`, copy 4095 against firmware 2048.
2. The next `set_tone_amplitudes` (the loaded amplitudes, 16 ms) wrote the
   copy's value back: the word read 4095 again and the copy was clean. **The
   outside write was undone without a warning.**

A flush pushes whole tone rows, so any fast write that touches a tone restores
all four of that tone's words from the copy, not just the field it changes.
Outside writes are detected on request, not prevented.

**Test 11, first part (15:14–15:24, pipeline 0, ZDMA).**

*`get_samples` warm-up, 50 against 0* (1000 tones, 2000 Hz, 3 × 30 s each):

| warm-up | lost per capture | where | torn |
|---|---|---|---|
| 50 (default) | 2, 0, 0 | single gaps mid-capture | 0 |
| 0 | **10, 10, 10** | all at sample 0 | **1 each** (the first sample) |

Without the warm-up every capture starts with a torn sample and 10 missing
accumulations: the same first-read stall the stream shows after
`enable_stream` (2.95 ms, torn, 5 missed at 1500 Hz). The warm-up hides a real
start-up stall rather than masking anything else; keep 50 (25 ms at 2000 Hz).
The stall itself stays under item 11.

*The 2048-tone ladder on current software* (stream, 3 × 30 s per rate, loop
read p50 115 µs, busy p50 ~280 µs):

| rate | lost per capture | gaps |
|---|---|---|
| 1000 Hz | 0, 0, 0 | — |
| 1500 Hz | 0, 0, 0 | — |
| 2000 Hz | 1, 0, 0 | one single gap |
| 2400 Hz | 21, 30, 30 | single gaps (0.03–0.04%); 2400 Hz leaves ~135 µs of a 417 µs period |

Against SW-f's 1800 Hz (< 0.1%): < 0.1% now reaches 2400 Hz, and no lost or
torn sample to 1500 Hz.

**Test 7, follow-up: `refresh_shadow_copy` (15:50, pipeline 1).** Decision: keep
the copy; only the server sets tones. The tone getters take
`refresh_shadow_copy=False`; `True` re-reads the copy from the firmware before
the read. Same outside katcp scale write as above, then `set_tone_phases` (a
different field of the same tone rows):

| | scale word after `set_tone_phases` |
|---|---|
| no refresh | 4095: the outside write undone |
| `get_tone_amplitudes(refresh_shadow_copy=True)` first | 2048: kept |

Both runs restored the word and left the copy clean.

**Test 12 — `fpga` info over the fast path (15:38–15:50).** On the fast
transport `get_status()` reads the same registers and returns the same keys and
values. Its `programmed` is a casperfpga stub that is always True, and a
register read cannot stand in for it, because reading an unprogrammed or
unclocked fabric hangs the processor. So "programmed" comes from the kernel FPGA
manager (`/sys/class/fpga_manager/fpga0/state` = `operating`), a file read that
touches no fabric. Probe on CPU2, pipeline 1 calling, 5× each:

| call | time | bus stalls |
|---|---|---|
| sysfs FPGA-manager state | 0.9 ms | 0 |
| `r_fast.fpga.get_status()` | 2.8 ms | 0 |
| katcp `r.fpga.get_status()` | 311 ms | 5 (~2.1 ms) |

Installed as `firmware_lib.fpga_programmed` (FPGA manager on the board, katcp
elsewhere) and `info_fpga(r, r_fast)`, used by the `fpga` and `server` sections
and `health_check`. After the 15:49 restart, probe on CPU2, pipeline 1 queried:

| request | stalls before | stalls now |
|---|---|---|
| `get_info(['versions', 'fpga'])` ×3 | 6 | **0** (60 ms) |
| `get_info(['server'])` ×3 | 3 | **0** (19 ms) |
| `health_check` ×3 | — | 3 (4.3 ms each) |
| `get_info('all')` ×1 | 9 | 10 (2.4–8.9 ms, 543 ms) |

What is left has the converter mixer-settings signature (4–9 ms):
`get_tone_frequencies` (in `tones`, `calibrations` and `health_check`) and
`info_rfdc` call `rfdc.core.get_mixer_settings`. Caching was rejected. Whether a
direct register read of the mixer settings avoids the stall is not yet known.
If the slow part is the converter's register access itself, it would not.

**Test 11 — the first-read stall, found and fixed (15:44–15:55).** Every
`get_samples` without warm-up, and a stream's first frame after new tones,
began with a torn read (2.95 ms) and a gap. `read_accumulated_data_fast` waited
for the accumulation, then built its read plan (`_cached_read_plan`), then
read. A plan is rebuilt for each new tone set and for each `get_samples` (a new
params dict): 0.26 ms (devmem) or 0.51 ms (ZDMA) on the host, ~10× that on the
ARM. It now plans before waiting. Same capture as the warm-up A/B (pipeline 0,
1000 tones, 2000 Hz, warm-up 0, 3 × 30 s):

| | lost per capture | torn |
|---|---|---|
| before (15:16) | 10, 10, 10 at sample 0 | 1 each |
| after (15:53) | **0, 0, 0** | **0** |

The warm-up was hiding only this. The default stays 50; it now costs nothing
and changes nothing.

**Cross-pipeline re-run on this build (15:55, pipeline 0 streaming with ZDMA,
1000 tones, 1500 Hz, 60 s; pipeline 1's ladder started twice):** pipeline 0
lost 151 with 13 torn reads, all at the two ladder starts: **74 and 56 lost,
6 and 5 torn**, around reads stalled ~4.3 ms (the mixer-settings signature).
The FPGA-status stalls are gone; each remaining ~4.3 ms stall costs about 6
accumulations at 1500 Hz. For comparison, 14:09 (no ZDMA, FPGA-status calls
still on katcp) lost 93 per start with no torn reads; why stalls tear reads
with ZDMA and not with devmem is not established. Pipeline 1's captures were
clean.

**Where the mixer-settings stall comes from (18:15, pipeline 1, probe on
CPU2).** The Xilinx driver (`XRFdc_GetMixerSettings`, `katcp_devel/xilinx/.../rfdc/src`)
takes the sampling rate from memory and otherwise reads ten block registers
(datapath 0x034, mixer config 0x080/0x084, mode 0x088, NCO update 0x08C,
frequency word 0x094–0x09C, phase 0x0A0/0x0A4) from the block base (DAC tile
t: 0x6000 + t·0x4000; ADC: 0x16000 + t·0x4000; + block·0x400). RFDC is at
`0xa0000000` (0x40000) in the `.fpg`, offset 0 of the fast mapping. Read
directly, 5× each:

| read | time | bus stalls |
|---|---|---|
| the ten mixer registers, DAC tile 1 block 0, devmem | 0.24 ms | **0** |
| the same, ADC tile 3 block 0 | 0.25 ms | **0** |
| katcp `get_mixer_settings`, DAC | 39 ms | 4 (4.2 ms) |
| katcp `get_mixer_settings`, ADC | 39 ms | 5 (4.2 ms) |

The raw words decode to the driver's answer (NCO word ±2⁴⁶ at Fs 4915.2 MSPS =
±1228.8 MHz; ADC mode bit 4 set = scale 1P0). **The registers are not slow;
something else in tcpborphserver's path holds the bus.** tcpborphserver's
handler adds only a block-enabled check and a log line. A fast-path read of
these ten registers would give `Freq` and "scale is 1P0" live, with no stall.
The register alone cannot tell AUTO from 0P7 (the driver keeps that in its
instance state); firmware_lib only ever tests for 1P0.

**Streaming regression on this build (15:57–16:09, pipeline 0 alone, ZDMA,
1000 tones, 3 × 60 s):**

| rate | lost per capture | torn |
|---|---|---|
| 1500 Hz | 0, 0, 0 | 0 |
| 2000 Hz | 0, 1, 0 | 0 |
| 2400 Hz | 3, 14, 3 | 0 |
| 3000 Hz | 67, 61, 138 (≤ 0.08%) | 0 |

In line with the 10:11 single-pipeline ZDMA results (no lost or torn sample to
2000 Hz, < 0.1% at 3000 Hz); no torn read in any of the 12 captures. Four
captures at 2400/3000 Hz began with one lost accumulation at sample 0, none
torn.

**Test 10 — the hour-long soak (18:25 onwards).** `profiling/dual_soak.py`:
both pipelines, 1000 tones each, ZDMA, rates set and both servers queried
before either stream starts, so no request reaches either server while they
record. Frames are not stored, only each frame's counter and torn flag.

| rate | pipeline 0 | pipeline 1 |
|---|---|---|
| 1500 Hz, 1 h | **0 lost, 0 torn** (5 399 895 frames) | **0 lost, 0 torn** (5 399 948) |
| 2000 Hz, 1 h | 8 lost in 8 gaps, 0 torn (0.00011%) | 5 lost, 0 torn (0.00007%) |

**Both pipelines hold the zero-event criterion at 1500 Hz for an hour** — the
first result on this document's timescale rather than 30–60 s. Every 2000 Hz
gap is a single accumulation. Six of pipeline 0's eight and two of pipeline 1's
five sit on exact multiples of 30 s from the start of the capture, which points
at the 30 s keepalive poll (`STREAM_KEEPALIVE_STREAMING_S`, item 11); most of
the ~120 wakes per hour cost nothing. A 60 s trial at 1500 Hz beforehand was
clean on both.

Pipeline 0's `loop_timing` after the 2000 Hz hour shows one 111 ms stall and 223
missed accumulations in its last 65 s of ring buffer: that is the capture's own
end (client disconnected, `disable_stream` and the info request), after
recording stopped, not loss inside the capture.

**The 2400 Hz hour (20:25) separates the two remaining periodic costs.**

| rate | pipeline 0 | pipeline 1 |
|---|---|---|
| 2400 Hz, 1 h | 201 lost (0.0023%), 0 torn | 551 lost (0.0064%), 0 torn |

Every gap is one accumulation, and they split cleanly:

- **the 30 s keepalive wake**: 110 of pipeline 0's 201 gaps and 130 of pipeline
  1's 551 are on exact multiples of 30 s from the start of the capture, i.e.
  nearly every one of the ~120 wakes in the hour costs an accumulation at
  2400 Hz. At 2000 Hz only a handful do, and at 1500 Hz none.
- **the 4 s event**: of the rest, 273 of pipeline 1's 420 consecutive intervals
  are 4.00 s ± 50 ms, with the phase within the 4 s grid tightly clustered
  (p10–p90 0.55–0.73 s); pipeline 0 shows 36 of 90 at the same spacing but a
  different phase. The two pipelines almost never lose together (4 coincidences
  in an hour), so it is per-core or per-process work, not a board-wide event.
  It matches the earlier observation that the accumulator read runs ~200 µs
  slower every 4 s. Identifying it needs perf sampling while both pipelines
  stream at 2400 Hz (root), which has not been run.

The keepalive wake is now gone: the stream client handler waits on the client's
subscribe line and on the stream stopping, with no timer while frames flow
(installed 21:51).

**`timing-monitor` costs more than either of them (21:56, 10 min at 2400 Hz,
both pipelines).** This check ran with `timing-monitor` started for test 8, and
lost **2604 (pipeline 0) and 1248 (pipeline 1)** accumulations in ten minutes —
against 201 and 551 over a whole hour without it. The gaps come in bursts of
~10, **every 3.03 s**: the monitor runs a `chronyc` subprocess on that period,
and on this board every one of them **times out** and logs a warning, so each
cycle costs a fork/exec, a timeout wait and a journal write while the frame loop
has ~130 µs of slack at 2400 Hz. This is the "timing-monitor 3.02 s comb" seen
earlier in `get_samples`. Stop it (or slow its poll) while recording; and the
`chronyc` timeout itself looks like a misconfiguration worth fixing.

The loop's own timing is unchanged (busy p50 216 µs, p90 284 µs against a
417 µs period), so the losses are interruptions, not frame cost.

**Keepalive removal confirmed (22:11, 10 min at 2400 Hz, monitor stopped):**

| | gaps on a 30 s multiple | expected by chance | lost |
|---|---|---|---|
| 20:25, 1 h, 30 s poll | 110 (p0), 130 (p1) | ~8, ~22 | 201, 551 |
| 22:11, 10 min, no timed wake | 1 (p0), 1 (p1) | ~2, ~4 | 41, 100 |

The 30 s signature is gone. The rest is the 4 s event (21 of pipeline 1's 99
gap intervals are 4.00 s ± 50 ms). Per-CPU interrupt counters over 20 s show
nothing on a 4 s period — only the 250 Hz kernel tick and a few IPIs on each
core — so it is a timer callback or user-space work, not a device interrupt.

**The 4 s event does not preempt the frame thread (22:28).** A 60 s raw
perf-sample dump of CPU3 at 10 kHz (`perf_cpu_sampler.py --raw`, sampling every
thread on the core) while both pipelines streamed at 2400 Hz:
`readout_srv_1` holds the core for **99.9%** of the samples, and there is **no
break in its samples longer than 400 µs** in the whole minute. Nothing else
runs there apart from an idle `kpktgend_3` and an occasional kworker. No symbol
recurs on a 4 s phase. So the event is not another task taking the core, and not
an interrupt: the thread keeps running while its own work takes ~200 µs longer,
which points at the interpreter (a periodic garbage collection would look
exactly like this) or at contention for the bus or DRAM.

Two measurement notes from the same run: the sampler itself costs ~2 lost
accumulations per second at 2400 Hz (313 and 394 lost in 150 s against 41 and
100 in 600 s without it), so it cannot be run during a loss measurement; and at
2400 Hz the loop uses ~99% of the period (wait 198 µs + busy 216 µs against
417 µs), which is why any small perturbation costs an accumulation there while
1500–2000 Hz absorbs it.

**It was the garbage collector (2026-09-19 22:10).** The server now freezes the
objects alive at that moment and stops collecting while a stream runs
(`_set_streaming`), collecting again when it stops. 10 min at 2381 Hz
(`acc_len` 252 after the rounding added on 09-18), both pipelines, monitor
stopped, against the 09-17 22:11 run at 2400 Hz:

| | pipeline 0 | pipeline 1 |
|---|---|---|
| collecting (09-17) | 41 lost, 21 of 99 intervals at 4.00 s | 100 lost |
| not collecting (09-19) | **17 lost**, 0 of 16 at 4 s | **16 lost**, 0 of 15 at 4 s |

The 4 s comb is gone from both, and the steady loop is cheaper too: busy p50
216 µs -> 177 µs (pipeline 0) and 211 µs (pipeline 1). What is left sits on a
**2 s grid** (pipeline 0's gaps at even seconds, pipeline 1's at even seconds
+ 0.9 s), 12-52 s apart, and is not yet identified.

Memory while collection is off: +12 MB per server over the first ten minutes
and +0.3 MB over the next, so that was buffer growth on the first capture, not
cyclic garbage piling up.

**What a healthy `timing-monitor` costs (22:29, same 10 min at 2381 Hz).** The
monitor polls every second and each poll spawns **nine** subprocesses: four
`pmc` queries and five `chronyc` (tracking as CSV and as text, sources as CSV
and with names, and `sources -v`).

| | pipeline 0 | pipeline 1 |
|---|---|---|
| monitor stopped | 17 lost (0.028/s) | 16 lost (0.027/s) |
| monitor running | 69 lost (0.115/s) | **1496 lost (2.5/s)** |

Ten of pipeline 1's gap intervals are exactly 1.0 s, the poll period. The loop
timing barely moves (busy p99 288 -> 319 µs, read p99 100 -> 108 µs): the
monitor never takes the core -- everything of its own is pinned to CPU0-1, as
are chronyd, journald, ptp4l and tsu-strobe -- it just adds ~30 µs to the tail
through what the cores share (DRAM and the interconnect, the cluster's L2, RCU
callbacks and TLB-shootdown IPIs from nine process launches a second). Pipeline
1 sits closer to the edge (busy p50 211 µs against pipeline 0's 177 µs) and
pays 94 times more for the same nudge.

**What matters is the spawn rate, not the CPU time.** The monitor's own CPU was
5.8% of a core; the cost to the pipelines comes from each `fork`/`exec` --
page tables built and torn down, TLB-shootdown IPIs, the cluster's L2 polluted
-- which crosses cores whatever the affinity. Cutting the queries per poll:

| monitor | spawns/s | pipeline 0 lost / 10 min | pipeline 1 lost / 10 min | monitor CPU |
|---|---|---|---|---|
| stopped | 0 | 17 | 16 | — |
| as shipped | 9 | 69 | 1496 | 5.8% |
| pmc + `chronyc -c tracking` per poll | 3 | **17** | **392** | 2.1% |
| pmc every 5 s, chrony every 30 s, clock by syscall | 0.6 | **10** | **414** | 1.9% |

Going from 9 spawns/s to 3 cut the loaded pipeline's loss 3.8x, but going from
3 to 0.6 changed nothing (392 -> 414), and the residual gaps show no 1 s or 5 s
structure (1 s phase concentration 0.08 and 0.14). So **process creation
explains the first factor but not what is left**: with the monitor merely
running, the loaded pipeline loses ~0.67 accumulations a second against 0.027
with it stopped, and that part is not periodic with the poll. Either something
the monitor does off the poll cadence, or a change in board state between the
22:10 baseline and the later runs; the control (stop the monitor, re-run) has
not been done. Pipeline 0 was running 381 tones against pipeline 1's 1000 in
these runs (see below), so read the two columns as "lightly loaded" and
"loaded" rather than as a difference between the pipelines.


### What tone tracking costs, and the per-accumulation deadline (2026-09-20/21)

#### What tone tracking costs today (2026-09-20)

The existing in-process tracking runtime, exercised as it ships: firmware
modulation (four slots, +/-2 kHz), `enable_tracking(dry_run=True)` so the
estimator runs but never retunes, 1000 tones on pipeline 1, pipeline 0
streaming plain as the control, 10 min per run.

| rate | layout | p1 frames/s | p1 lost | p1 busy p99 | p0 lost (control) |
|---|---|---|---|---|---|
| 2500 Hz | no tracking | 2500 | 95 | 299 µs | 223 |
| 2500 Hz | tracking, one core | 799 | 1 020 601 | **11 209 µs** | 74 633 |
| 2500 Hz | tracking, `--split-threads` | 2129 | 222 373 | 620 µs | 152 725 |
| 1500 Hz | no tracking | 1500 | **0** | 301 µs | **0** |
| 1500 Hz | tracking, `--split-threads` | 1500 | 43 680 (5.1%) | 704 µs | 3 908 |

Three things follow.

**The GIL was the first problem.** With everything on one core the consumer
blocked the frame loop for up to 11 ms at a time and pipeline 1 delivered a
third of its frames. `SOUK_WORKER_CPUS=0,1` (via `rt_tuning.sh --split-threads`)
removed those stalls -- p99 11 ms -> 0.6 ms -- and recovered delivery to 85%.

**The estimator is the second, and it is the real one.** Even off the frame
core it cannot keep up: 15% loss at 2500 Hz and 5% at 1500 Hz, where the loop
otherwise has 425 µs of spare and loses nothing at all. It processed 122 035
cycles in 600 s (203 cycles/s) while a synthetic decimating pipeline handles
625 cycles/s for 9% of a core (`profiling/tracker_estimator_cost.py`), so the
shipped estimator costs roughly 10-30x more per cycle: it works per cycle, with
NaN-aware filtering, running stats and a controller decision, rather than one
decimated pass. **Moving it to another process would inherit that cost**; the
estimator has to come down first (decimate, fewer passes over the batch --
see `doc/tone_tracking.md`).

**It is not local to its own pipeline.** Tracking on pipeline 1 cost pipeline 0
between 3 908 and 152 725 accumulations depending on layout, and the split
layout made the neighbour worse (74 633 -> 152 725) because the workers then
compete on CPU0-1. Memory on the tracking server went 193 -> 412 MB during a
run and did not come back.

#### Making the tracking consumer affordable (2026-09-20)

Per-stage cost of the consumer on the ARM, one 0.2 s batch at 1000 tones,
2500 Hz (512 frames, 127 cycles), measured with the real functions:

| stage | before | after |
|---|---|---|
| `parse_frame_batch` | 26.8 ms | 27.5 ms |
| `assemble_cycles` | 149.7 ms | **0.7 ms** |
| `average_cycles` (new) | -- | 6.2 ms |
| detuning estimate | 180.5 ms (per cycle) | **5.5 ms** (per block) |
| `DetuningFilter.update` | ~5 ms (per-cycle loop) | 0.5 ms |
| `_update_stats` | -- | 1.4 ms |
| `controller.decide` | 14.6 ms | 14.6 ms |
| **total** | **~363 ms (182% of a core)** | **~56 ms (28%)** |

Three changes did that. The estimate now runs on `filter_blocks_per_batch`
coherently averaged blocks rather than on every cycle -- the estimator is
nonlinear (phase unwrap, then a curvature/slope ratio), so averaging I/Q first
is both cheaper and slightly better at low SNR, and it uses all the data where
a boxcar of 10 cycles against 125 per batch had been discarding 92% of it.
`assemble_cycles` replaced an `np.add.at` scatter (unbuffered, 150 ms) with
`np.add.reduceat` over the runs, which works because the (cycle, point) groups
are contiguous, and skips both the 8 MB NaN allocation when every point is
present and the reduction entirely when there is one sample per point. Batches
are capped at what the filter needs, so a backlog is shed rather than
assembled.

**What that bought, and what it did not** (10 min, both pipelines, 1000 tones,
2500 Hz, tracking dry-run on pipeline 1):

| step | p1 frames/s | p1 lost |
|---|---|---|
| no tracking | 2500 | ~95 |
| as it was | 799 | 1 020 601 |
| coherent blocks | 957 | 925 959 |
| + backlog cap | 977 | 913 757 |
| + fast assembly | 1386 | 668 212 |
| + 0.5 s poll | 1889 | 366 261 |

**The remaining cost is when the work happens, not how much there is.** The
consumer runs in the executor but shares the GIL with the frame producer, and
it holds it in lumps of 5-27 ms; at 2500 Hz a 27 ms lump alone blocks 68
accumulations. Polling four times as often made it *worse* (728 frames/s,
1 063 526 lost) because the per-batch work is mostly fixed -- `controller.decide`
costs 14.6 ms whatever the batch size -- so the fixed cost was paid 20 times a
second instead of 5. Polling at 0.5 s and shedding the surplus was the best
setting measured.

Next, in order: `controller.decide` (14.6 ms per batch, per-tone Python-level
work), and `parse_frame_batch`, which builds a complex128 array for all 512
frames (8 MB) when the grouping could be done on the raw integers and only the
averaged blocks converted. Together those would take the batch from ~56 ms to
~15 ms. If that still is not enough, the separate process on CPU0-1 with a
shared-memory ring is the remaining option -- but it was worth establishing
first that the cost is the implementation, not the placement.

#### Rate ladder and headroom, both pipelines, 1000 tones each (2026-09-20)

Overnight, `timing-monitor` stopped, GC off during captures, ZDMA on both,
`profiling/dual_soak.py` (no requests during the recording). 20 min per rate,
plus an hour each at 2500 and 2000 Hz. "Spare" is the period minus the loop's
busy time; the 1-in-1000 column is the worst frame in a thousand.

| rate | period | spare p50 | spare 1-in-1000 | lost (20 min) | torn |
|---|---|---|---|---|---|
| 600 Hz | 1667 µs | 1398 / 1414 µs | 1239 / 1281 µs | **0 / 0** | 0 |
| 1000 Hz | 1000 µs | 747 / 767 µs | 644 / 604 µs | **0 / 0** | 0 |
| 1500 Hz | 667 µs | 425 / 447 µs | 309 / 335 µs | **0 / 0** | 0 |
| 2000 Hz | 500 µs | 263 / 284 µs | 162 / 179 µs | 49 / 3 (0.002% / 0.0001%) | 0 |
| 2500 Hz | 400 µs | 162 / 184 µs | 62 / 80 µs | 446 / 190 (0.015% / 0.006%) | 0 |
| 3000 Hz | 333 µs | 93 / 118 µs | -9 / +9 µs | 274 591 / 25 750 (7.6% / 0.7%) | 0 |
| 3947 Hz | 253 µs | 14 / 38 µs | -95 / -59 µs | 50% / 17% | 0 / 4 |
| 5000 Hz | 200 µs | -38 / -15 µs | -151 / -118 µs | 50% / 50% | 110 / 0 |

Hour-long runs: **2000 Hz lost 62 and 18** (0.0009% / 0.0003%), **2500 Hz lost
1189 and 1116** (0.013% / 0.012%), no torn read in either.

Read it as three regimes. To 1500 Hz there is more than 300 µs of spare even on
the worst frame in a thousand, and nothing is lost over 20 min on either
pipeline. At 2000-2500 Hz the worst frames come within 60-180 µs of the
deadline and the loss is 1e-5 to 1.5e-4 -- usable, but this is where any extra
work (an info request, a monitor's process launch, a tracker) shows up. From
3000 Hz the 1-in-1000 frame is already over the deadline and delivery falls to
the ~2800-3000 frames/s ceiling: the loss is not a tail any more, it is the
rate being unachievable.

**Memory with collection disabled during captures** (the change that removed
the 4 s comb): over the 4.5 h of runs, pipeline 0 went 193.5 -> 195.6 MB and
pipeline 1 192.3 -> 195.9 MB, on an 8 GB board. No runaway.

**Why pipeline 1 looks fragile in these runs: it has more tones.** Pipeline 0
was carrying 381 tones and pipeline 1 1000, so pipeline 0 reads in p50 68 µs
against 84 µs and its busy p50 is 177 µs against 211 µs -- more slack per
period, hence far fewer losses for the same disturbance. With 1000 tones on
both (the 09-17 soaks) the two read identically at 84 µs. There is no
per-pipeline hardware difference here; compare pipelines only at equal tone
counts.

**Test 8 — the timed sync round trip (21:45–21:50, `timing-monitor` running).**
PTP is locked to a grandmaster and the 1 PPS strobe is healthy. Three findings:

1. **The telescope time had never been loaded** (the firmware TT read 1970 +
   board uptime), so the first `host_checks.py sync` refused: its target was in
   the past. `set_telescope_time` fixes that and is a prerequisite for B7.
2. **`set_telescope_time` reported its own success as failure.** It read the
   alignment back immediately, but `ext_sync_tt` holds the TT latched *at* the
   sync that performs the load, so it returned the pre-load counter
   (`aligned: false`, a 1970 timestamp) while the load had in fact worked: read
   directly a minute later, the PPS-latched TT was real UNIX time, **116 µs**
   off the second boundary. Fixed by waiting one more PPS.
3. **The armed sync fires, but the firmware cannot confirm where.**
   `timed_sync_time` holds the target and the countdown register reads expired,
   yet `tt_sync_msb/lsb` — "TT at the last sync", the register `timed_sync_check`
   compares against the target — stays **0** on v7.12.1. So the round trip
   cannot be proven with that register; the check reports FAIL on a sync that
   did fire. An alternative is to verify against the accumulations' own TT after
   an armed sync, which has not been written.

### What tracking actually costs the pipelines (2026-09-20)

**The binding constraint is the tracking consumer's memory footprint, not its
CPU time and not the GIL.** The four A53s share a 1 MB L2; the consumer streams
~10 MB per batch through it and evicts everything *both* stream loops need.

The evidence is that the damage crosses processes. At 2500 Hz, 1000 tones per
pipeline, with tracking running only on pipeline 1:

| | port 10000 (no tracking) | port 10001 (tracking) |
|---|---|---|
| tracking off, 5 min | 856 lost (0.114%), 0 torn | 123 (0.016%), 0 torn |
| tracking on, 10 min | **63 023 (4.2%)**, 37 torn | 320 365 (21%), 11 134 torn |

Port 10000 is a separate process that never ran a tracking thread, so no
amount of GIL or scheduling explanation covers it.

Confirmed directly by replacing tracking with a synthetic load on CPU0-1 at the
same ~13% duty cycle, varying only the working set (`profiling/gil_yield_check.py`
is the thread-level probe; the load itself is four lines of numpy):

| load on CPU0-1, 13% duty | port 10000 | port 10001 |
|---|---|---|
| idle | 856 (0.114%) | 123 (0.016%) |
| 32 KB working set | 5 313 (0.89%) | **74 (0.012%)** |
| 16 MB working set | **133 447 (22%)**, 262 torn | 36 828 (6.1%), 55 torn |

Same CPU duty in both; a 500x smaller working set is 25-500x less damaging, and
on pipeline 1 the small-footprint load is indistinguishable from idle. **Any
consumer that keeps its working set in cache is close to free; any consumer
that streams megabytes is fatal**, wherever it runs.

#### The GIL is a real but secondary cost

The in-process consumer also lost 216 705 accumulations in ten minutes at
1500 Hz in stalls of up to **83 ms**, and making it cheaper (363 ms → 29 ms per
batch) barely moved that part. That much *is* the GIL.

`firmware_lib._blocking_sleep` is a pure-Python spin, and
`_blocking_wait_for_acc_fast` sits in it for most of every frame period, so the
stream loop **holds the GIL almost continuously**. Any other thread in the
process only gets it at `sys.setswitchinterval` boundaries (5 ms by default).

Measured on the board (1000 tones, 2500 Hz, 0.5 s poll, both threads on
CPU0-1), with a partner thread doing 140 µs of GIL-held work per 400 µs frame:

| wait | consumer batch | frames lost | |
|---|---|---|---|
| spin (today) | 350 ms | 2.07/s | 0.083% |
| `time.sleep` for all but the last 120 µs | 64 ms | **0.33/s** | 0.013% |

The consumer's batch is only 40 ms of work; under the spin it takes 350 ms
because it waits ~5 ms for the GIL at every Python statement between numpy
calls.

Two things that do **not** help:

- **Lowering `sys.setswitchinterval`.** 5 ms → 50 µs cuts the consumer's batch
  (350 → 65 ms) but raises the loop's losses (9.4 → 14.4 stalls/s): more
  handoffs, same worst case.
- **Making the consumer's operations smaller.** Every large numpy call blocks
  the partner thread for 250-500 µs at p99.9 whatever it is (`astype` 256 µs,
  `add.reduceat` 248 µs, `b''.join` of the batch 518 µs, max 3.2 ms), and a
  *tiny* op in a Python loop is the worst of the lot (8.8 ms p99.9) because it
  contends for the GIL far more often.

So the fix belongs in the producer: `_blocking_wait_for_acc_fast` now sleeps
out the bulk of the wait and spins only the last `ACC_WAKE_MARGIN_S` (150 µs).
It learns the accumulation interval from its own arrival times, and forgets it
whenever more than one accumulation goes by, so a stale estimate costs one
spun wait rather than a run of late reads.

Why the numbers work on this board:

- `time.sleep` overshoots by a very tight **~60 µs** (p99.9 67 µs, max 107 µs
  over 50-500 µs sleeps), so a 150 µs margin covers it with room to spare.
- `cpuidle.off=1` is on the kernel command line, so an idle core only does WFI
  — sleeping does not drop into a deep C-state and wake up late.
- On hardware at 1500 Hz the yielding wait misses **no** accumulations and its
  worst-case interval is *tighter* than the spin's (714 µs vs 900 µs). Isolated
  at 2500 Hz (wait + a real 1000-tone read, no server), spin and sleep are
  indistinguishable: 2-3 vs 0-2 missed per 4000, p99.9 545-591 µs either way.
  A 5-minute dual soak at 2500 Hz with tracking off is also clean, so the
  yielding wait is not what costs the runs above.

#### Where that leaves tracking

Nothing on CPU0-1 may stream megabytes while the pipelines run. Two ways to
satisfy that:

1. **Accumulate in the producer.** `tap()` adds each frame into a
   `(blocks, N, 2*n_tones)` accumulator instead of keeping it, so the consumer
   reads ~32 KB rather than 10 MB. Costs one in-place int64 add of 2000 words
   per frame — **30.5 µs measured**, against a spare of p50 161 / p01 90 /
   p001 73 µs at 2500 Hz. `prepare_frame` already holds the data in a
   persistent `<i4` buffer, so no `frombuffer` is needed. int32 would be
   15.4 µs but overflows (~156 cycles per block per point at a 0.5 s poll).
   Taking one cycle in four drops it to ~7.6 µs/frame for a 2x noise penalty.
2. **Run the consumer on the host.** The stream already goes over TCP and the
   estimator is plain numpy on parsed frames, so this costs the board nothing
   and moves rather than rewrites the code. Corrections return through the
   existing apply path (1.3 ms off-loop plus a 60 µs flip, and rare).

The 32 KB control run above is the evidence that (1) would work: at the same
duty cycle it cost pipeline 1 nothing.

### The per-iteration deadline was self-imposed (2026-09-21)

`_blocking_wait_for_acc_fast` read the counter on entry and waited for it to
*change*. A loop iteration that came back even 10 µs after accumulation N+1
landed therefore waited for N+2, and N+1 -- still sitting unread in the
accumulator for most of a period -- was lost. That is the whole of two
findings above: "the deadline is per accumulation, not per batch", and the
quantised ceiling `rate / ceil(loop / period)`. It is also why every loss in
the hour-long 2500 Hz soaks was a single accumulation.

The stream loop now reads such an accumulation late (`catch_up`, plain and fw
branches only), provided the read -- its measured length plus
`ACC_CATCH_UP_GUARD_S` -- ends before the next one is due. The due time is
extrapolated from the last arrival the wait watched land. The deadline then
becomes an average (mean busy < period) plus one period of tolerance for any
single iteration. A virtual-clock model of the real wait code, with the
board's sleep overshoot and a busy time shaped like the 2500 Hz ladder, per
1 M frames:

| extra per-frame work | lost, wait for the next | lost, late read |
|---|---|---|
| none | 10 | **0** |
| +33 µs (the tracking tap) | 135 | **0** |
| +100 µs | 10 253 | **0** |
| +160 µs (mean ≈ period) | 553 518 | 3 528 (2 torn) |

With 2048 tones and no ZDMA (a 280 µs read) 2500 Hz is simply over budget;
the late read delivers 71% of frames there instead of 50%, with no torn read.

Stalls longer than about a period still lose accumulations whatever the wait
does. What is left to remove is therefore anything that holds the event loop or
the GIL for more than ~250 µs at 2500 Hz: the tracking consumer (~15 ms per
window on the ARM, in a thread), the health snapshot (~20 ms after
vectorising, was 75), a commit's staging (~1.4 ms), and any request handler
that builds or encodes a large reply.

**The premise needs one hardware check first**: that the accumulator holds an
accumulation unchanged until `acc_cnt` moves, and does not start the next dump
before it (`profiling/acc_hold_check.py`, read-only, needs a live signal). Not
yet run.

Two things this reopens, not yet re-measured: batching frames into one
`write()` per drain was rejected *only* because the flush overran one
accumulation, which a late read now absorbs; and `get_samples` could use the
same late read on its plain and fw paths.


### The stream loop in its own process (2026-09-22)

What still costs frames once late reads are in is anything that holds the event
loop or the GIL for more than ~250 µs: the tracking consumer (~15 ms per window
on the ARM), the health snapshot (~20 ms), commit staging (~1.4 ms), any request
that builds a large reply. Threads cannot fix that (see "The GIL is a real but
secondary cost"), so the split is into processes: `server/stream_process.py`.

**Turning it on.** `firmware.stream_process: true` in the config, or
`SOUK_STREAM_PROCESS=1` in the unit's environment (which wins; `=0` turns it
off). Pin with `SOUK_STREAM_CPUS` / `SOUK_WORKER_CPUS` as `rt_tuning.sh
--split-threads` already does: the stream process goes to the first, the whole
server to the second. Without them the two share a core and the kernel preempts
the frame loop whenever the server works (the server logs a warning).

**What moves.** The stream process owns the stream port and its clients and does
wait → read → frame → send → tracking tap, nothing else. Everything else stays
in the server: requests, sweeps, `get_samples`, tracking estimation and commits,
info. The frame and client code is borrowed from `ReadoutServer`, so both modes
build byte-identical frames.

**What crosses.** A shared block in `/dev/shm/souk_stream_p<N>` holds the
stream mode (idle / plain / fw), the eight flags, the fw revision and
`n_settle`, a tone-set generation, the pause handshake and the loop-timing
ring; the frame loop reads it every frame, so a flag set by a request is in the
next frame as before. A pipe carries tone indices, typed frames, tracking
window swaps and the sparse-read override, and is read at every yield. The
server's side is two properties (`active_tone_indices`, `fw_modulation_state`)
and an Event subclass for the flags and stream events, so no call site changed.
A retune is flagged `set_freqs` from the moment the server changes the tones
until the new indices have reached the frame loop.

**Tracking.** The tap (`tracking.IQAccumulator`, pulled out of
`TrackingRuntime`) runs in the stream process on a second shared file; a window
is handed over by naming which of the two boxcar buffers is finished. The
server reads and zeroes it in place, so the window never crosses the pipe.

**Safety.** Before any reprogram, resource initialisation or clock fault the
server pauses the stream process and waits for it to confirm it has stopped
reading; it refuses to program if that does not come within 2 s (60 s while the
process is still starting). The stream process re-opens its own fast interface
only when `_create_firmware_interfaces` finds the fabric healthy again. The
DMA channel and landing buffer belong to the stream process; the server reads
(sweeps, `get_samples`) through the CPU mapping.

**Not supported with it** (refused with a message): the sw modulation engine,
which steps hardware from the frame loop, and triggered streams.
`get_info('loop_timing')` reports the stream process's frames (plus a
`stream_process` status block); a `get_samples` capture's own timing is not
recorded.

**Tested off-board only.** `tests/test_stream_process.py` runs the real
process with a clock-driven fake accumulator (`stream_process._FakeAccumulator`)
and a real TCP client: frame content and order, flags within a frame, the retune
flagging order, fw tags and revision, typed frames only to subscribers,
SNAPSHOT on subscribe, a tracking window swap with the sums checked, pause /
resume, clean shutdown; plus the server hooks without a process.

**On the board (2026-09-22)**, both pipelines, 1000 tones, 2500 Hz, fw
modulation ±2 kHz (dwell 1), dry-run tracking, `profiling/dual_soak.py
--modulate 2000 --track`, files in `~/souk/v712_stages/soak/`:

| | lost | torn | where |
|---|---|---|---|
| p0 in-process (146 s, cut by an IP drop) | 893 | 0 | every 5 s tracking poll |
| p1 in-process (146 s) | 1 768 | 76 | every 5 s tracking poll |
| **p0 split, 10 min** (1.5 M frames) | **54** | **0** | about every 2nd-3rd poll |
| **p1 split, 10 min** | **54** | **73** | polls; torn on a 2.0 s grid from 438 s |

In-process, the tracking consumer holds the GIL 22-57 ms per window and every
window costs ~20 accumulations. Split, the window swap still costs the stream
process ~0.5 ms between two frames (the pipe exchange is ~120 µs of it; the rest
looks like cache pressure from the consumer, which runs on CPU0-1 and shares the
1 MB L2), so about one poll in two or three loses one accumulation. p1's torn
reads start 438 s in, sit on a 2.0 s grid and have nothing to do with tracking
-- the same unexplained 2 s grid as 2026-09-19. A 30 s split run also showed a
4.000 s comb on p0 where the stream process wakes ~300 µs late (suspect: the
softlockup watchdog's per-CPU timer; needs root to test).

**Correction, same day: most of the split residual is one external event.**
Placing every gap and torn read on the wall clock (the soak's connect time plus
frame index over rate), most sit **0.09-0.12 s after an even second** (IQR
~20 ms), for stretches of minutes, on one pipeline or the other:

| 10 min split | on the even-second grid (p0 / p1) | elsewhere (p0 / p1) |
|---|---|---|
| tracking window 4 blocks | 27 / 117 | 26 / 10 |
| `--track-param filter_blocks_per_batch=1` | 190 / 147 | 9 / 5 |

So the tracking polls cost 36 events in 10 min with the default window and 14
with one four times smaller (the consumer's L2 footprint on CPU0-1 is real), and
everything else is this event -- the unexplained 2 s grid of 2026-09-19. It is
locked to the wall clock, which points at the timing daemons or the network:
ptp4l here has `logAnnounceInterval 1` (the grandmaster announces every 2 s at a
fixed phase), and switches send spanning-tree hellos every 2 s. Not yet tested;
needs root (stop ptp4l / tsu-strobe / timing-monitor for one run each). Separately,
after the reboot the timing monitor's chronyc calls time out every 3.04 s again,
as on 2026-09-17; a restart of the monitor cleared it then.

**Found (same day, 16:03): it is the kernel's SLAB allocator.** A 60 s
`perf_cpu_sampler.py --raw` at 10 kHz during a soak shows the stream process
holding its core 99.9% of the time; the only other real work on CPU2/3 is
`kworker/N:2-mm_percpu_wq` in `cache_reap` / `drain_array` -- SLAB's per-CPU
reaper, every 2 s (`REAPTIMEOUT_AC`), 1.984/2.016 s apart, at wall phase
0.117-0.133 s on CPU2, ~100-300 µs a time. The pipeline 0 events during the
sampling sat at 0.119-0.123 s. It is per-CPU, so no isolation setting can move
it (it survived the timing fix and the re-applied `rt_tuning.sh`), and this
kernel is `PREEMPT_NONE` (`CONFIG_SLAB=y`, HZ 250), so a stream process waking
while it runs waits for it to finish. Not the cause, but also on the stream
cores: `kpktgend_2/3` (`CONFIG_NET_PKTGEN=y`, poll every 100 ms). Fixes, in
order of reach: SLUB (a kernel rebuild; no periodic reaper), or run the stream
process SCHED_FIFO (`SOUK_STREAM_RT_PRIO`), which gets the core back at the
reaper's next `cond_resched()` -- safe only with a wait that sleeps, which the
late-read wait now does (`ACC_LATE_WAKE_S`).

**With SCHED_FIFO 50 (16:40, 10 min, same load):**

| | p0 lost / torn | p1 lost / torn |
|---|---|---|
| before (isolation restored, SCHED_OTHER) | 38 / 132 | 163 / 310 |
| SCHED_FIFO, napping wait | **111 / 0** | **12 / 2** |

The 2 s grid is gone from both (3-4 events at any 2 s phase). (The first FIFO
run, 16:22, lost 5 130 / 682: the napping wait had tied catch-up to the age of
its timing anchor, so a late loop waited a period instead of catching up. Fixed,
with a regression test.) p0's remaining losses sit on the 5 s tracking poll:
with the wait now reading ~90 µs after each arrival, the ~0.5 ms the window swap
took in the stream process (a pipe request and reply) no longer fits the
catch-up margin. The swap now goes through the shared block instead
(`TRACK_REQ`/`TRACK_ACK`, the window's counts beside the control words), done
by the stream process right after a frame, with no system call.

**Then (17:00-17:33, same load, 10 min each):**

| | p0 lost / torn | p1 lost / torn | p0 / p1 spare p50 |
|---|---|---|---|
| shared-block window swap | 14 / 0 | 2 / 1 | 116 / 208 µs |
| + p0 on ZDMA, + XPS | **2 / 0** | **4 / 0** | 174 / 175 µs |

The 5 s poll is gone from the losses. What was left on p0 was headroom, from
two causes:

- **p0 had never used ZDMA.** Its board config said `zdma_udmabuf: 0` (YAML
  reads an int; p1's says `"udmabuf1"`), so it looked for
  `/sys/class/u-dma-buf/0`, fell back to the CPU read and logged it once. Read
  95 → 68 µs once set to `"udmabuf0"`.
- **Both streams can share one NIC transmit queue.** end0 has two (`mq`), but no
  XPS map was set, so each TCP connection picked one by its random flow hash.
  On a shared queue, whichever core holds the qdisc transmits everything queued,
  so one stream core can end up sending the other's packets as well (GSO
  segmentation and the driver, ~32 µs per 8 kB frame), fixed for the whole
  connection because both loops run off the same clock. Four 60 s runs:

  | TX queue bytes (q0 / q1) | `outside` p50, p0 / p1 |
  |---|---|
  | 1 / 2533 MB (shared) | 122 / 58 µs |
  | 2 / 2533 MB (shared) | 93 / 91 µs |
  | 1266 / 1268 MB | 90 / 91 µs |
  | 1266 / 1268 MB | 93 / 97 µs |

  `rt_tuning.sh kernel` now maps tx-0 to CPU2 and tx-1 to CPU3.

Of the six losses left, three (p0 17:29:58.645/.648, p1 .678) sit 12-45 ms
after an ssh login from the host: the host's `souk-readout-watchdog.timer`
(every 5.5 min: ssh, `ss`, a `python3` start and a katcp `?fpgastatus`, all
on CPU0-1). The other two (p1, 17:25:53.390/.392, 2 ms apart) have nothing in
the journal.

**30 min, same setup (17:34-18:04), 4.5 M frames a pipeline: p0 4 lost, p1 0,
no torn reads.** One of p0's (18:02:58.664) is 66 ms after a watchdog login;
the other five logins in the run cost nothing. The other three (17:48:19.970,
17:51:24.995, 17:51:55.832) have nothing in the journal and fell outside the
52 s loop-timing ring by the end of the run. Spare p50 173 / 174 µs, p0.01%
-17.5 / -5.5 µs; read 67, frame 63, outside 96 µs on both.



## Appendix C: raw ladder data (2026-09-15)

All ladders: 1000 tones, stream socket, 30 s per point, `timing-monitor` stopped,
`--skip-tones` so the comb is identical throughout. "vol" = journald
`Storage=volatile`, "persist" = default persistent. v7.11 columns ran v1.7.10.
"v7.12 read-opt" = `c111e3e` (sparse reads, devmem `acc_cnt`) with ZDMA on,
journald setting not recorded; it was labelled "unpatched" / "v1.8.0" before
2026-09-16. "v7.12 throttle" = the same plus the `_yield_due` drain/yield
throttle and `prepare_frame` cache, ZDMA on; the keepalive and `SO_SNDBUF`
fixes landed after these ladders and are measured separately below.

##### Delivered frames/s  (`rate × (1 − loss)`; derived from `packet_counter`, no wall clock)

| rate Hz | v7.11 vol | v7.11 persist | v7.12 read-opt | v7.12 throttle | v7.12 throttle vol | v7.12 +batching |
|---|---|---|---|---|---|---|
| 600 | 600 | 600 | 600 | 600 | 600 | 600 |
| 1000 | 999 | 999 | 1000 | 1000 | 997 | 1000 |
| 2000 | 998 | 999 | 1999 | 2000 | 2000 | 1998 |
| 3000 | 748 | 748 | 2173 | 2988 | 2989 | 2549 |
| 3509 | 775 | 803 | 1755 | 3481 | 3492 | 3054 |
| 4000 | 766 | 798 | 2000 | 3541 | 3633 | 3492 |
| 5000 | 750 | 831 | 2497 | 2560 | 2718 | 3769 |
| 6000 | 841 | 842 | 2986 | 2992 | 2979 | 4696 |

##### Loss %

| rate Hz | v7.11 vol | v7.11 persist | v7.12 read-opt | v7.12 throttle | v7.12 throttle vol | v7.12 +batching |
|---|---|---|---|---|---|---|
| 600 | 0.000 | 0.074 | 0.000 | 0.000 | 0.011 | 0.000 |
| 1000 | 0.089 | 0.099 | 0.048 | 0.000 | 0.268* | 0.017 |
| 2000 | 50.101 | 50.057 | 0.062 | 0.005 | 0.002 | 0.103 |
| 3000 | 75.051 | 75.075 | 27.573 | 0.385 | 0.361 | 15.043 |
| 3509 | 77.923 | 77.120 | 49.990 | 0.796 | 0.486 | 12.956 |
| 4000 | 80.843 | 80.057 | 50.008 | 11.486 | 9.172 | 12.700 |
| 5000 | 84.994 | 83.385 | 50.052 | 48.805 | 45.636 | 24.620 |
| 6000 | 85.980 | 85.959 | 50.235 | 50.131 | 50.351 | 21.735 |

\* contaminated: concurrent `ssh` to the board during that capture. The clean
60 s measurement at the same point was **0.000%, zero gaps in 59241 frames**.

##### `packet_error` count (torn reads — the frame's I/Q is internally inconsistent)

| rate Hz | v7.11 vol | v7.11 persist | v7.12 read-opt | v7.12 throttle | v7.12 throttle vol | v7.12 +batching |
|---|---|---|---|---|---|---|
| 600 | 0 | 1 | 0 | 0 | 0 | 0 |
| 1000 | 0 | 0 | 1 | 0 | 0 | 1 |
| 2000 | 59 | 28 | 0 | 0 | 0 | 0 |
| 3000 | 21806 | 21834 | 1 | 0 | 0 | 0 |
| 3509 | 22573 | 23402 | 0 | 11 | 4 | 3 |
| 4000 | 22362 | 23254 | 2 | 4 | 0 | 0 |
| 5000 | 21873 | 24217 | 20 | 10 | 1 | 3 |
| 6000 | 24521 | 24545 | 20 | 23 | 20 | 12 |

##### Steady-state gap count (index ≥ 200; excludes the startup cluster)

| rate Hz | v7.11 vol | v7.11 persist | v7.12 read-opt | v7.12 throttle | v7.12 throttle vol | v7.12 +batching |
|---|---|---|---|---|---|---|
| 600 | 0 | 5 | 0 | 0 | 1 | 0 |
| 1000 | 26 | 27 | 7 | 0 | 1 | 3 |
| 2000 | 28907 | 28930 | 36 | 3 | 1 | 60 |
| 3000 | 21605 | 21634 | 24099 | 335 | 315 | 13151 |
| 3509 | 22372 | 23201 | 51007 | 744 | 485 | 13058 |
| 4000 | 22161 | 23053 | 58163 | 13329 | 10680 | 13560 |
| 5000 | 21672 | 24016 | 72718 | 70941 | 66406 | 15195 |
| 6000 | 24320 | 24344 | 87095 | 87093 | 86761 | 18435 |

##### Samples lost

| rate Hz | v7.11 vol | v7.11 persist | v7.12 read-opt | v7.12 throttle | v7.12 throttle vol | v7.12 +batching |
|---|---|---|---|---|---|---|
| 600 | 0 | 13 | 0 | 0 | 2 | 0 |
| 1000 | 26 | 29 | 14 | 0 | 19 | 5 |
| 2000 | 29225 | 29196 | 36 | 3 | 1 | 60 |
| 3000 | 65594 | 65764 | 24177 | 337 | 316 | 13186 |
| 3509 | 79672 | 78874 | 51207 | 815 | 498 | 13277 |
| 4000 | 94362 | 93346 | 58418 | 13398 | 10705 | 14831 |
| 5000 | 123886 | 121529 | 73071 | 71227 | 66599 | 35938 |
| 6000 | 150372 | 150255 | 88119 | 87780 | 88193 | 38100 |

#### devmem access scaling, v7.11.2 — wrong target (full output)

`access_size_scaling.py --read-addr 0x000d0000 --no-write`, contiguous reads.
The offset came from the single-pipeline `.fpg`; on the dual-pipeline build the
board runs it is **`p1_mix_rx_lo0_control0`**, an AXI-lite LO control BRAM, not
the accumulator. Times include `np.frombuffer`. Kept as a record of an
AXI-lite BRAM on v7.11.2, not as an accumulator measurement:

| chans | words | bytes | median µs | ns/word |
|---|---|---|---|---|
| 1 | 2 | 8 | 8.67 | 4335.0 |
| 2 | 4 | 16 | 8.97 | 2242.5 |
| 4 | 8 | 32 | 8.88 | 1110.0 |
| 8 | 16 | 64 | 9.42 | 588.8 |
| 16 | 32 | 128 | 10.65 | 332.8 |
| 32 | 64 | 256 | 14.06 | 219.6 |
| 64 | 128 | 512 | 20.16 | 157.5 |
| 128 | 256 | 1024 | 32.28 | 126.1 |
| 256 | 512 | 2048 | 55.05 | 107.5 |
| 512 | 1024 | 4096 | 100.77 | 98.4 |
| 1024 | 2048 | 8192 | **192.12** | 93.8 |
| 2048 | 4096 | 16384 | **374.89** | 91.5 |

**fit: t[µs] = 8.56 + n_words × 0.0895**  (F = 8.56 µs, W = 89.5 ns/word = 42.6 MiB/s)

M separate 1-channel reads (scattered offsets in the same LO control BRAM), v7.11.2:

| M | median µs | µs/read | vs full buffer |
|---|---|---|---|
| 1 | 9.24 | 9.24 | faster |
| 2 | 16.62 | 8.31 | faster |
| 4 | 31.08 | 7.77 | faster |
| 8 | 59.67 | 7.46 | faster |
| 16 | 117.21 | 7.33 | faster |
| 32 | 231.62 | 7.24 | faster |
| 64 | 462.49 | 7.23 | slower |
| 128 | 923.04 | 7.21 | slower |
| 256 | 1854.56 | 7.24 | slower |

Full-region read 375.25 µs; per-tone beats span when **M < ~42.9**. For
comparison the v7.12 accumulator fit is **F = 1.64 µs, W = 22.4 ns/word (170 MiB/s)**
(2.0 µs at 64 B, 93.4 µs at 16 KiB), giving a merge threshold of ~37 channels —
but that fit is a bare slice, without the 6.9 µs `np.frombuffer` included above,
so the two F values are not comparable.

#### Per-frame software cost, on the ARM (1000 tones, 8044 B frames)

`prepare_frame` decomposition:

| part | µs |
|---|---|
| `np.zeros` alloc only | 6.52 |
| alloc + data assign | 13.18 (cumulative) |
| header pack, 10 words, incl. per-call `.view()` | 13.81 |
| `frame.tobytes()` | 2.91 |
| `struct.pack` + concat | 4.53 |
| **full body** | **39.41–41.49** |
| **with cached array/view/prefix** | **26.18** (byte-identical output) |

asyncio path, per frame:

| pattern | µs |
|---|---|
| bare `await asyncio.sleep(0)` | 30.22 |
| `write()` + `await drain()` | 54.62 |
| `write()` + `drain()` + `sleep(0)` | 122.69 |
| … with 1 competing always-ready task | 135.48 |
| … with 3 competing tasks | 164.26 |

Full loop patterns (loopback, 1 competing task):

| pattern | µs/frame |
|---|---|
| alloc-build, drain+yield every frame (original) | **188.67** |
| reuse buffer only | 160.84 |
| reuse + drain/yield every 4 | 86.28 |
| **reuse + drain/yield every 8** | **78.35** |
| reuse + drain/yield every 16 | 95.88 (worse — 16×8 KB overruns the 64 KiB high-water mark) |
| alloc-build + drain/yield every 8 | 106.98 |

Batched writes, measured over the **real NIC** (board → host sink):

| pattern | µs/frame |
|---|---|
| write() per frame, drain/yield every 8 | 96.10 |
| one write() per 8 via `b''.join` | 69.37 |
| framed into a preallocated batch buffer | 64.73 |

#### Keepalive periodicity

2 kHz, 60 s, v7.12:

| | steady gaps | gaps/s | median spacing | within 5% of a whole second |
|---|---|---|---|---|
| before | 72 | 1.22 | 1005 ms | 86% (52/60) |
| after | 16 | 0.27 | 2050 ms | 60% (9/15) |

Same signature on **v7.11 + v1.7.10**, confirming it is long-standing and
firmware-independent:

| rate | steady gaps | gaps/s | median spacing | whole-second |
|---|---|---|---|---|
| 1000 Hz | 57 | 0.97 | 999 ms | **93% (52/56)** |
| 1200 Hz | 232 | 3.93 | 132 ms | 23% (52/231) |

#### `get_samples` startup gaps (the `SO_SNDBUF` ramp)

20 s captures:

| rate | before: gaps @ positions | lost | after `SO_SNDBUF`: gaps | lost |
|---|---|---|---|---|
| 600 Hz | 0 | 0 | 0 | 0 |
| 1000 Hz | 3 @ [25, 42, 46] | 23 | **1** @ [72] | **8** |
| 2000 Hz | 3 @ [28, 44, 82] | 366 | **1** startup @ [70] | — |

#### SD-card write activity during a live capture (30 s window)

| journald | writes completed | ms writing | % of wall |
|---|---|---|---|
| persistent | 211 | 9446 | **31%** |
| volatile | 12 | 49 | **0.2%** |

#### 60 s verification, v7.12 fully patched, volatile

| rate | frames | steady gaps | lost | loss % |
|---|---|---|---|---|
| 600 | 35513 | 0 | 0 | **0.000** |
| 1000 | 59241 | 0 | 0 | **0.000** |
| 1000 (repeat) | 59173 | 1 | 19 | 0.032 |
| 2000 | 118352 | 16 | 16 | **0.014** |

#### Gap-size split, v7.12 (steady state; small ≤3 samples = loop overrun, large >3 = external stall)

| rate | read-opt small | read-opt large | throttle small | throttle large | max stall |
|---|---|---|---|---|---|
| 3000 | 24099 / 24101 | 0 / 0 | 335 / 336 | 0 / 0 | 0.7 ms |
| 3509 | 51007 / 51007 | 0 / 0 | 737 / 759 | 7 / 55 | 3.7 ms |
| 4000 | 58161 / 58196 | 2 / 22 | 13322 / 13341 | 7 / 29 | 1.2 ms |
| 5000 | 72698 / 72770 | 20 / 101 | 70931 / 70993 | 10 / 46 | 1.0 ms |
| 6000 | 87075 / 87531 | 20 / 387 | 87070 / 87449 | 23 / 131 | 45.5 ms |

(cells are `gaps / samples lost`.) With `Storage=volatile` the large-gap
population goes to **0 / 0 at both 3509 and 4000 Hz**, and max stall 3.7 → 0.9 ms.


## Appendix D: gotchas met during the upgrade


- **Do not `scp -r` this repo to the board.** `.git` is ~1.8 GB against a 4.4 MB
  `git archive` export, and it will fill the board's disk. Use
  `git archive --format=tar.gz --prefix=souk_readout_tools/ HEAD`, or
  `rsync --exclude='.git'` if you need uncommitted changes.
- **The board's souk-firmware clone is single-branch**, so a plain `git fetch`
  shows no `v7.12` however much it exists upstream. Use
  `git fetch origin tag v7.12.1`, or widen `remote.origin.fetch`.
- **`make` in `profiling/` can silently do nothing.** `devmem_profile` is
  gitignored, so an x86 binary left behind by an earlier copy will have a newer
  mtime than the archive-extracted `.c` and make will call it up to date. The
  x86 ELF then fails as `Syntax error: ")" unexpected`. `rm -f devmem_profile &&
  make`.
- **The profiler offsets change every firmware generation.** Always take them
  from `profiling/fpg_addrs.py`; do not copy them out of any README.
- **Never `rsync --delete` to the board from a fresh `git worktree`.**
  `server/souk-peripherals-control` is a git submodule, and a new worktree
  creates it *empty*. `--delete` then faithfully wipes the board's populated
  copy, and the server dies at import with
  `ModuleNotFoundError: souk_rf_mixerless_atten_amp_level`. Run
  `git submodule update --init` in the worktree first, or drop `--delete`.
  (The submodule is pinned to the same commit in v1.7.10 and v1.8, so copying it
  between them is safe.)
- **`souk_readout_tools.__version__` is read from the installed `dist-info`, not
  the source.** After overwriting `site-packages/souk_readout_tools/` with a
  different version's source, it still reports the old number. Verify the code,
  not the version string — e.g. `md5sum` the installed `readout_server.py`
  against the tree you think you deployed.
- **Reverting the firmware means reverting the control_sw too.** `souk_mkid_readout`
  must match the bitstream generation; a v7.12.1 control_sw against a v7.11.2
  `.fpg` is not a supported pairing.
- **Resolve the `.fpg` from the server's control_sw yaml, never by guessing a
  symlink.** The server loads `souk-dual-pipeline-krm.yaml` →
  `souk_dual_pipeline_krm.fpg`. Offsets once taken from the single-pipeline
  header pointed the "accumulator" measurement at `p1_mix_rx_lo0_control0`.
  `profiling/fpg_addrs.py <yaml>` and `board_state.py` do this for you.
- **Two readout servers run** (`readout_server_0` and `_1`). Stop both before
  profiling — the second one still reads the bus.
- **A revert is only as complete as what you check.** The 2026-09-15 revert
  left casperfpga at `a89480c` and `/lib/firmware/top.bit` at v7.12.1; record
  every stage with `board_state.py` rather than from memory.
