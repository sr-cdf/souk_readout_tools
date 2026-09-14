# devmem access profiling: Python vs C

Two standalone scripts to profile the `/dev/mem` (mmap) access pattern that the
fast readout path uses — a sweep step of *control-buffer writes* followed by
*accumulator reads*. One in Python (a no-dependency copy of what
`firmware_lib` actually does), one in C, with timing printouts and a per-step
text dump for inspection.

## What's being profiled

A **sweep step** =
1. **write** the tone control buffer into the mapped AXI-lite region
   (`CONTROL_N_WORDS = 4` int32 words per tone: phase_inc, ri_step, phase_off,
   scale), and
2. **read** back the accumulator. The accumulator dout is a flat array of
   interleaved complex samples, one per tone, laid out real-first as pairs of
   **int32**: `[re0, im0, re1, im1, ...]` (firmware accumulators use
   `dtype='>i4', is_complex=True`, read as `'<i4'` on the fast interface). So
   one channel = 1 complex sample = 2 × int32 = 8 bytes, and the design
   supports up to `N_TONE = 2048` complex channels.

These mirror the real code:

| profiled op | firmware_lib source |
|-------------|---------------------|
| `axil_mm[a:b] = v.tobytes()` (write) | `write_control_buffer_data_fast` (firmware_lib.py:1947) |
| `np.frombuffer(axil_mm[a:b], '<i4')` (read) | `read_accumulated_data_fast` (firmware_lib.py) |

The mmap parameters match casperfpga `LocalMemTransport`
(`transport_localmem.py:13-14`): `/dev/mem`, offset `0xA0000000`, 512 MiB,
`MAP_SHARED`. The window was 32 MiB until firmware v7.12 moved the accumulator
and LO control buffers to AXI4 BRAMs at `0xB000_0000`; casperfpga widened
`MAP_SIZE` to `0x20000000` to reach them, and these profilers follow.

## Build & run

```bash
make                 # builds ./devmem_profile from devmem_profile.c

# against an in-RAM backing file (runs anywhere, no root, no board):
./devmem_profile        --steps 10000 --tones 1000 --read-chans 1000
python3 devmem_profile.py --steps 10000 --tones 1000 --read-chans 1000
```

Each writes a text file (`devmem_profile_c.txt` / `devmem_profile_py.txt`)
with a header of summary stats followed by one row per step:
`step_index  write_posted_us  write_sync_us  read_us  step_us`.

Common flags: `--steps`, `--tones` (tones per write), `--read-chans`
(accumulator channels read back), `--warmup` (unrecorded steps), `--dev`,
`--out`, `--write-addr`, `--read-addr`, `--sync-writes/--no-sync-writes`
(C also has `--bulk`).

### Write timing: posted vs. completion-forced

MMIO writes are *posted* — the store returns once the bytes are in the CPU
store buffer / interconnect FIFO, before the AXI slave has accepted them. So
the profilers measure the write **twice**: `write(posted)` (stores handed off,
can imply rates above the physical bus limit) and `write(sync)` (a same-slave
read-back barrier forces the posted writes to drain to the endpoint first).
The step total uses `write(sync)` by default; `--no-sync-writes` switches it to
the (unphysical) posted time. Trust `write(sync)` for real bus cost.

## Running against real hardware (/dev/mem)

On `/dev/mem` the write/read offsets must point at **real firmware regions**,
or the write loop clobbers whatever registers sit at offset 0. The control
buffer is exactly what a real sweep overwrites each step, and the accumulator
`dout0` is what it reads, so aiming there reproduces a real sweep.

Get the offsets straight from the `.fpg` header (no board, no readout object
needed) — the header has `?register <name> <abs_addr> <size>` lines and the
mmap is based at `0xa0000000`, so `offset = abs_addr - 0xa0000000`. Note the
registers are no longer all in the AXI-lite region: from v7.12 the accumulator
and LO control buffers live at `0xB000_0000`+, i.e. offsets above `0x10000000`
(which is why the map must be 512 MiB):

```bash
python3 fpg_addrs.py \
    ~/souk/souk-firmware/firmware/src/souk_dual_pipeline_krm/outputs/souk_dual_pipeline_krm.fpg \
    --pipeline 0 --lo tx --acc 0
# prints e.g.:  --dev /dev/mem --write-addr 0x050000 --read-addr 0x090000
```

Then run both profilers with those offsets (root required for /dev/mem):

```bash
# offsets below are v7.12; they change EVERY firmware generation -- always
# take them from fpg_addrs.py above rather than copying these.
sudo ./devmem_profile --dev /dev/mem \
    --write-addr 0x13000000 --read-addr 0x10000000 \
    --steps 10000 --tones 1000 --read-chans 1000 --bulk \
    --out devmem_profile_c_hw_bulk.txt

sudo ~/py3.12-venv/bin/python devmem_profile.py --dev /dev/mem \
    --write-addr 0x13000000 --read-addr 0x10000000 \
    --steps 10000 --tones 1000 --read-chans 1000 \
    --out devmem_profile_py_hw.txt
```

For the dual-pipeline krm firmware, pipeline 0 is control buffer
`p0_mix_tx_lo0_control0` and accumulator `p0_acc0_dout0`. Do not hardcode their
offsets — they have moved every firmware generation (v7.11 split the ping-pong
buffers into `control0`/`control1`; v7.12 relocated both to `0xB000_0000`+), so
always take them from `fpg_addrs.py` above. Sizes are set by the design maximum
of `N_TONE = 2048` tones:

- control buffer: 2048 tones × 4 int32 words/tone × 4 B = 32 KiB per LO slot;
  the 128 KiB register backs the slots.
- accumulator: 2048 complex channels × 2 int32 (real/imag) × 4 B = 16 KiB.

Defaults of 1000 tones fit comfortably. The profilers only ever touch
`--tones` / `--read-chans` worth of each region, so the volumes they report are
`--tones × 16 B` written and `--read-chans × 8 B` read.

> Profiling the WRITE overwrites the live TX control buffer, so it changes your
> current tone settings. Re-apply your config afterwards if needed. To measure
> hardware **read** cost only, point `--write-addr` at a scratch offset.

**Alternative:** `dump_addrs.py --config my_config.yaml` pulls the same offsets
from a live readout object (uses `get_fast_read_params` /
`_get_control_buffer_addresses`). Use it if your offsets differ from the fpg
header for any reason; otherwise `fpg_addrs.py` is simpler.

## IMPORTANT: how to read the numbers

**On a backing file (RAM), Python can look *faster* than C. That is expected
and does not mean Python wins on hardware.**

- Python `mm[a:b] = bytes` is a single bulk `memcpy` over cacheable RAM.
- The C version accesses the window word-by-word through a `volatile
  uint32_t*` — no vectorization, one access per word — because that is what a
  real memory-mapped register window requires.

On real `/dev/mem` AXI-lite, **every 32-bit access is a separate
non-cacheable bus transaction**; the bulk-memcpy advantage disappears and the
word-at-a-time model is the honest one. So:

- Use the **backing-file** runs to compare *language/interpreter overhead per
  call* (Python slice + `frombuffer` + numpy vs a C loop).
- Use the **`/dev/mem`** runs to measure *actual hardware transaction cost*,
  which is what dominates a real sweep and is largely language-independent.

The C profiler has both: per-word `volatile` (default) and a `--bulk` mode that
does a single `memcpy` like Python. Running all three on hardware separates the
*access pattern* effect from the *language* effect — see Results below.

## Results (real hardware, RFSOC ARM, /dev/mem) — firmware v7.11

Pipeline 0, `--write-addr 0x050000 --read-addr 0x090000`, 1000 tones / 1000
read-chans, 10000 steps. Medians, reproducible to the microsecond across runs:

| mode | write | read | step | read ns/word |
|------|-------|------|------|--------------|
| C per-word (`volatile`) | 199 µs | 421 µs | 620 µs | 211 |
| Python bulk (`mm[a:b]`) | 207 µs | 168 µs | 375 µs | 84 |
| **C bulk (`--bulk` memcpy)** | 196 µs | **163 µs** | **359 µs** | **82** |

The `write` column here is the *posted* write (`write(posted)`); this run
predates the posted/sync split. Re-run to also get `write(sync)`, the
completion-forced time (posted + drain-to-slave via the read-back barrier),
which is the honest cost of a write reaching the AXI slave.

For reference, the in-RAM backing-file floor is ~17 µs/step — so hardware is
~20–37× slower, and that gap is **pure AXI-lite bus latency**, not CPU.

The read path dominates and is where the modes diverge. Decomposing the ratio:

| ratio (read) | value | isolates |
|--------------|-------|----------|
| C per-word ÷ C bulk | **2.58×** | **access pattern** — the whole effect |
| Python bulk ÷ C bulk | **1.03×** | **language** — essentially nil |
| C per-word ÷ Python | 2.51× | the two effects conflated (what we first saw) |

Writes tie everywhere (~50 ns/word): they are *posted* on the bus, so per-word
vs bulk doesn't matter. Reads are *blocking* round-trips — per-word `volatile`
pays the full ~211 ns latency on every word, while a bulk `memcpy` lets the bus
stream at ~82 ns/word.

Stability: C per-word has the tightest tail (0 steps > 2× median — ~2000
serialized reads resist scheduler interruption). The bulk modes are 2.5× faster
in the median but show a handful of sub-millisecond scheduler-jitter outliers
per 10000 steps. Speed wins; the determinism tradeoff is noted only.

## Results (real hardware, RFSOC ARM, /dev/mem) — firmware v7.12.1

Same rig, same invocation: pipeline 0, 1000 tones / 1000 read-chans, 10000
steps, medians. Only the addresses change —
`--write-addr 0x13000000 --read-addr 0x10000000`, from `fpg_addrs.py`.

**What is different in the hardware.** v7.12 moves the accumulator output and
the eight LO control buffers off 32-bit AXI4-Lite onto **128-bit AXI4 BRAMs at
`0xB000_0000`+**, and doubles the AXI control clock **100 → 200 MHz**
(`PL0_REF_CTRL` divisor `0xa → 0x5`, in `psu_init` inside `BOOT.BIN`). Nothing
in the profilers changed except `MAP_SIZE` (32 MiB → 512 MiB, since the new
region sits at offsets above `0x10000000`).

| mode | write(posted) | write(sync) | read | step | read ns/word | read MB/s |
|------|------|------|------|------|------|------|
| Python bulk (`mm[a:b]`) | 16.77 µs | 19.08 µs | 53.40 µs | 72.48 µs | 26.7 | 149.3 |
| **C bulk (`--bulk` memcpy)** | **14.91 µs** | **15.36 µs** | **44.49 µs** | **59.85 µs** | **22.2** | **179.3** |

Against the v7.11 rows above:

| | v7.11 | v7.12 | gain |
|---|---|---|---|
| Python read | 168 µs | 53.40 µs | **3.15×** |
| C-bulk read | 163 µs | 44.49 µs | **3.66×** |
| Python write (posted, like-for-like) | 207 µs | 16.77 µs | **12.3×** |
| C-bulk write (posted) | 196 µs | 14.91 µs | **13.1×** |
| Python step | 375 µs | 72.48 µs | **5.2×** |
| C-bulk step | 359 µs | 59.85 µs | **6.0×** |

Two caveats on that table. The v7.11 `write` column is the *posted* time (that
run predates the posted/sync split), so the write rows compare posted-to-posted;
the v7.12 `step` uses the honest `write(sync)`, which makes the step gain
slightly *understated* — posted-to-posted it is 5.34× for Python. And the
in-RAM backing-file floor is still ~17 µs/step, so hardware is now only ~3.5×
the RAM floor where it was ~21×: **the bus is no longer the overwhelming term.**

**The write gain was not predicted and dominates the step.** The plan for this
upgrade forecast a ~4× improvement on the *read* path and said little about
writes. Reads came in slightly under that (3.2–3.7×) because they are blocking
round-trips and remain latency-bound. Writes improved **12–13×** because they
are posted and stream: a 32-bit port at 100 MHz became a 128-bit port at
200 MHz, 8× the theoretical bandwidth, with better coalescing on top.

C-bulk's **22.2 ns/word / 179.3 MB/s is 94% of the ~190 MB/s uncached-Device
ceiling** quoted for this SoC, so the read path is now essentially at the limit
of what the CPU's uncached mapping can do. Getting past it needs DMA, not
firmware (see TBD below). A read near ~42 ns/word instead would indicate the
AXI clock is still at 100 MHz, i.e. a stale `BOOT.BIN`.

### The language gap reopened: 3% → 21%

| ratio | v7.11 | v7.12 |
|---|---|---|
| Python ÷ C-bulk, read | 1.03× | **1.20×** |
| Python ÷ C-bulk, step | 1.045× | **1.21×** |

Nothing about Python got slower. Its fixed per-operation overhead (~9 µs on the
read: the `bytes` allocation behind `mm[a:b]`, then `np.frombuffer`) is
unchanged — but when the bus took 163 µs that cost was invisible, and now the
bus takes 44 µs it is a fifth of the total. **The bottleneck has moved from the
bus into software.**

This weakens, but does not overturn, the v7.11 conclusion below. Still true:
per-word `volatile` C remains far worse than bulk anything, so any rewrite must
stay bulk. Against it: 12.6 µs/step is second-order next to the 303 µs/step the
upgrade already delivered, and these profilers measure *only* the devmem
operations — a real sweep step also builds frames and computes tone parameters
in Python, so the in-situ share is smaller than 21%. Measure `get_sweep_data`
end-to-end before deciding.

### Still TBD

This measures the **CPU-mapped `/dev/mem` path on v7.12.1 with the 200 MHz
clock and the AXI4 BRAMs**. Four things are not covered:

- **UIO (dropping root).** The new device tree exposes `user_io@a0000000`;
  `/dev/uio0`, `/dev/uio1` and `/dev/udmabuf0..7` are present and
  `uio_pdrv_genirq.of_id=generic-uio` survives in `/proc/cmdline`. Expected to
  be the *same speed* as `/dev/mem` — it buys permissions, not latency — but
  unmeasured here.
- **DMA-style reads (ZDMA into a cached u-dma-buf).** The only route past the
  ~190 MB/s uncached ceiling, since that ceiling is a property of the CPU
  mapping rather than the fabric. `software/rfsoc_scripts/zdma_read_example.py`
  on `v7.12` is a hardware-validated reference. Not attempted.
- **Sweeps are not broken.** These profilers write and read the *right
  addresses* but not meaningful *values* — they never exercise the tone map,
  channel map or fitter. Confirming a sweep still fits a known resonator to the
  same `fr`/`Qi`/`Qc` as v7.11 is a separate, and stronger, check.
- **Timestream *integrity* at high rate is already better; the rate *ceiling*
  is a separate question.** `packet_error` is set when the accumulation counter
  advances during the read, i.e. a straddled, possibly torn read — so it is a
  straddle probability, and the read shrinking 168 → 53 µs cuts it directly. At
  2000 Hz the accumulation window is 500 µs, so the read went from **34% of the
  window to 11%**. Post-upgrade, `packet_error` is **0 at every rate up to
  2000 Hz**, which was *not* the case on v7.11. That much is a real win and is
  attributable to this change. What is **not** established is whether the
  achievable sample *rate* rises: the residual loss at 1800–2000 Hz shows up as
  gaps in `packet_counter` with `packet_error` still 0, i.e. the loop returning
  late rather than the read overrunning — that points at the per-sample framing
  and `await writer.drain()` path in `readout_server.get_samples`. Needs a
  like-for-like rate scan against v7.11 numbers to quantify.

## Conclusions

1. **It's bus-bound, not CPU-bound.** A sweep step is ~360–620 µs on hardware
   vs ~17 µs in RAM. The AXI-lite bus, not the language, sets the floor.

2. **Access pattern is everything; language is ~3%.** The `--bulk` run proves
   the entire 2.5× read difference was per-word vs bulk access — C bulk and
   Python bulk land within 3% of each other. Both hit the ~82 ns/word bus floor.

3. **Don't rewrite the fast path in C.** It buys ~3% on reads and ~0% on writes
   — not worth a C extension. Python's existing `mm[a:b]` bulk-slice approach
   (in `firmware_lib`) already does the fast thing and is near-optimal.

4. **The only real lever on sweep speed is the read path:** read contiguously
   (already done) and read fewer channels — read cost scales linearly with
   read-chans, and at ~82 ns/word the bus is the language-independent limit. A
   burst/DMA-capable transport is the only thing that could beat the bulk read.

> **Firmware v7.12 is that transport, and it delivered.** Measured on v7.12.1:
> read 3.2× (Python) / 3.7× (C-bulk), write 12–13×, **step 5.2× / 6.0×** — see the
> v7.12.1 results section above. C-bulk reaches 22.2 ns/word, 94% of the ~190 MB/s
> uncached ceiling, so the read path is now near the limit of the CPU's uncached
> mapping. Conclusions 2 and 3 above are **v7.11-era**: at v7.12 speeds the
> Python-vs-C gap widened from 3% to 21%, because fixed interpreter overhead no
> longer hides behind a slow bus. Conclusion 4's premise also weakens — the bus is
> now ~3.5× the RAM floor rather than ~21×. Beating the remaining ceiling still
> needs ZDMA into a cached `u-dma-buf`
> (`software/rfsoc_scripts/zdma_read_example.py` in souk-firmware), which is
> untried — see **Still TBD** above.

## Files

- `devmem_profile.py` — Python profiler (mirrors `firmware_lib` bulk access)
- `devmem_profile.c` / `Makefile` — C profiler; per-word default, `--bulk` mode
- `fpg_addrs.py` — extract register offsets from a `.fpg` header (preferred)
- `dump_addrs.py` — extract offsets from a live readout object (fallback)
- `devmem_profile_{c,py}_hw.txt`, `devmem_profile_c_hw_bulk.txt` — hardware runs
- `devmem_profile_c.txt` — in-RAM backing-file floor (language-overhead only)

> The compiled `devmem_profile` binary is architecture-specific and gitignored.
> Always `make` on the board before running — an x86 binary on the ARM board
> fails with `Syntax error: "(" unexpected` (the shell tries to run the ELF as
> a script).
