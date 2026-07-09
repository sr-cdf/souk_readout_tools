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
(`transport_localmem.py:13-14`): `/dev/mem`, offset `0xA0000000`, 32 MiB,
`MAP_SHARED`.

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
needed) — the header has `?register <name> <abs_addr> <size>` lines anchored
at `0xa0000000`, so `offset = abs_addr - 0xa0000000`:

```bash
python3 fpg_addrs.py \
    ~/souk/souk-firmware/firmware/src/souk_dual_pipeline_krm/outputs/souk_dual_pipeline_krm.fpg \
    --pipeline 0 --lo tx --acc 0
# prints e.g.:  --dev /dev/mem --write-addr 0x050000 --read-addr 0x090000
```

Then run both profilers with those offsets (root required for /dev/mem):

```bash
sudo ./devmem_profile --dev /dev/mem \
    --write-addr 0x050000 --read-addr 0x090000 \
    --tones 1000 --read-chans 1000 --out devmem_profile_c_hw.txt

sudo ~/py3.12-venv/bin/python devmem_profile.py --dev /dev/mem \
    --write-addr 0x050000 --read-addr 0x090000 \
    --tones 1000 --read-chans 1000 --out devmem_profile_py_hw.txt
```

For the dual-pipeline krm firmware, pipeline 0 is control buffer
`p0_mix_tx_lo0_control` (offset `0x050000`, 64 KiB register) and accumulator
`p0_acc0_dout0` (offset `0x090000`, 16 KiB register). Both are sized for the
design maximum of `N_TONE = 2048` tones:

- control buffer: 2048 tones × 4 int32 words/tone × 4 B = 32 KiB of live
  control data; the 64 KiB register backs the two ping-pong buffers (2 × 32 KiB).
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

## Results (real hardware, RFSOC ARM, /dev/mem)

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
