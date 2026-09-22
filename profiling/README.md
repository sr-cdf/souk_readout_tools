# Readout access profiling

Tools and results for how fast the readout talks to the FPGA: control-buffer
writes, accumulator reads, and the streaming loop built on them. The document
started as one question about the transport and one about the language, and
has grown into a firmware-by-firmware comparison. Each part below keeps its
original results and compares them with what came before, so old and new
numbers sit side by side rather than replacing each other.

## TL;DR

- **Transport:** katcp costs 37–280× LocalMem (`/dev/mem`) for the same
  control_sw call (`get_acc_cnt()` 1 ms vs 27 µs; `_read_bram()` 2048
  channels 53–83 ms vs 0.3–0.55 ms), and a direct devmem counter read is 4 µs.
- **Byte order:** devmem sees the opposite byte order to katcp *per 32-bit bus
  word*. For 32-bit data that is just `'<i4'` vs `'>i4'`; for 16-bit data
  packed two to a word (v7.12's PSB reorder map, the ADC/DAC snapshots) the
  order of the two entries flips as well. Reading such data as `'<u2'`
  pair-swaps it — on v7.12 that sent every tone to its neighbour's synthesis
  bin until 2026-09-16. Fixed; see `doc/v712-notes.md`, TL;DR.
- **Counter reads can glitch on v7.12.1:** `acc_cnt` read mid-increment returns
  mixed old/new bits about 1 in 30 000–70 000 increments
  (`acc_cnt_probe.py`); read it until two reads agree.
- **Language:** bulk vs per-word matters, Python vs C barely does. Per-word C
  is 4× slower than bulk Python on v7.12.1 (sweep step 290 vs 73 µs); bulk C
  saves 17% on bulk Python (60.5 µs).
- **Firmware, v7.11.2 → v7.12.1, same software** (S0 → S3):

  | | v7.11.2 | v7.12.1 | gain |
  |---|---|---|---|
  | accumulator read | 79.3 ns/word, 48 MiB/s | 22.9 ns/word, **167 MiB/s** | 3.5× |
  | LO control write, bulk copy | 76 MiB/s (1000 tones 202.5 µs) | **0.98 GiB/s** (17.6 µs) | 11.5× |
  | LO control write, scattered per word (tone updates until 2026-09-17) | 1000 tones 217.1 µs | 77.0 µs | 2.8× |
  | benchmark step: bulk 16 KB write + bulk 8 KB raw read, Python | 369 µs | 73 µs | 5.1× |
  | `read_accumulated_data_fast`, 1000 tones (the server's per-accumulation read) | 226 µs | 111 µs | 2.0× |

  ~70–75% from the AXI4 BRAMs, ~25–30% from the 200 MHz clock; the new kernel
  changes nothing for devmem.
- **CPU reads are at the ARM's ceiling.** Plain DRAM through the same kind of
  uncached mapping gives 186 MiB/s (in line with upstream's "~190 MB/s"); the
  accumulator reaches 90% of it.
- **New transports (v7.12.1 `image.ub`):** UIO = devmem speed without root;
  ZDMA reads at 2.2 GiB/s after a 17 µs start (2048 channels 102 → 31 µs,
  through `firmware_lib` 279 → 116 µs). ZDMA writes only win for the whole
  4-slot LO register (126 → 94 µs).
- **Software write path (2026-09-17):** LO control words now go through a RAM
  copy of the control BRAMs and are pushed as one bulk copy. Frequency write to
  TX + RX at 1000 tones 247 → 110 µs; `set_tone_amplitudes` 1.3 s (katcp) →
  6 ms. See [LO control writes through a RAM copy](#lo-control-writes-through-a-ram-copy-2026-09-17).
- **What that buys in streaming** (1000 tones, loss-free rate, against the
  600 Hz campaign): ≤ 600 Hz with the old software and firmware → 1500 Hz now,
  2000 Hz with ZDMA (under 0.1% loss to 2400 / 3000 Hz).
  Two pipelines at once, tuned: 0.05–0.11% loss per minute up to 2400 Hz.
  A `get_info` on one pipeline stalls the other's reads through two slow driver
  calls (converter mixer settings, FPGA status): starting a capture on
  pipeline 1 costs pipeline 0 ~93 accumulations at 1500 Hz
  (`doc/v712-notes.md`, "Caching tried and reverted").
  Rate ladders and the software's share: `doc/v712-notes.md`, TL;DR.

## Contents

1. [How this document evolved](#how-this-document-evolved)
2. [Tools](#tools)
3. [Running a firmware stage](#running-a-firmware-stage)
4. [Part 1: katcp vs LocalMem, then Python vs C (v7.10.3)](#part-1-katcp-vs-localmem-then-python-vs-c-v7103)
5. [Part 2: v7.12.1 against Part 1](#part-2-v7121-against-part-1)
6. [Part 3: staged comparison, v7.11.2 to v7.12.1](#part-3-staged-comparison-v7112-to-v7121)
7. [Conclusions, as they stand](#conclusions-as-they-stand)
8. [Files](#files)

## How this document evolved

| when | question | method | firmware | where |
|---|---|---|---|---|
| late 2025 | katcp or LocalMem (`/dev/mem`) for accumulator reads? | `get_new_spectra()` timing, `doc/accumulators.md` | late-2025 build | Part 1 |
| 2026-06-10 | Is the fast path limited by the language or the bus? Python vs C, per-word vs bulk | `devmem_profile.py` / `.c` on `/dev/mem` | **v7.10.3** (labelled v7.11 until 2026-09-16) | Part 1 |
| 2026-09-14 | Did v7.12 deliver? | the same invocation; `access_size_scaling.py` for F and W | v7.12.1 | Part 2 |
| 2026-09-15 | Where does the v7.11 ceiling come from? | `access_size_scaling.py` on the reverted board | v7.11.2 | Part 3 — measured the wrong register |
| 2026-09-16 | Which part of the PR buys what? | stages S0–S3 with fixed software; UIO, ZDMA and DRAM-ceiling transports; rate ladder | v7.11.2 → v7.12.1 | Part 3 |
| 2026-09-17 | How hard can two pipelines stream at once, and why do counters glitch? | `rt_tuning.sh`, `rate_ladder.py --loop-timing`, `acc_cnt_probe.py`, `zdma_channel_check.py` | v7.12.1 | `doc/v712-notes.md`, "Pushing the zero-drop rate" |
| 2026-09-17 | Can tone writes use the bulk copy? | `control_write_check.py` on the RAM copy of the LO control registers | v7.12.1 | Part 3, "LO control writes through a RAM copy" |

Three corrections made on 2026-09-16 run through the rest of this file:

- **The "v7.11" baseline was v7.10.3.** The result files date from 2026-06-10,
  before v7.11 existed, and `--write-addr 0x050000 --read-addr 0x090000` are
  `p0_mix_tx_lo0_control` / `p0_acc0_dout0` in the v7.10.3 dual-pipeline map.
- **The 2026-09-15 v7.11.2 scaling run read `p1_mix_rx_lo0_control0`**, not the
  accumulator: its offset came from the single-pipeline `.fpg`, but the server
  loads the dual-pipeline one. `fpg_addrs.py` now takes the control_sw yaml.
- **F values were not comparable.** That run's F (8.56 µs) included
  `np.frombuffer` (6.9 µs on the board's CPU, measured alone 2026-09-16); the v7.12 F quoted beside it (1.64 µs)
  did not. `access_size_scaling.py` now reports both.

**Throughput units.** Rates are given as a per-word cost W (one 32-bit word) and
as throughput in binary units: MiB/s = 3814.7 / W[ns], GiB/s above 1024 MiB/s.
Throughputs from `devmem_profile` summaries are payload bytes over the *mean*
call time. Upstream's "~190 MB/s" ceiling (`zdma_read_example.py`) is decimal,
≈ 181 MiB/s; our own earlier "MB/s" figures were decimal too and are converted
here.

## Tools

| script | runs on | root? | test | measures |
|---|---|---|---|---|
| `board_state.py` | board | for the register and mapping checks | B0 | boot-file md5s, `PL0_REF_CTRL` (AXI clock) and `AFI_FS` (HPM width) read back, kernel nodes, installed software md5s, configured `.fpg`, services, tcpborphserver mapping |
| `fpg_addrs.py` | anywhere | no | — | byte offsets of the write, read and AXI-lite reference targets, from the control_sw yaml |
| `access_size_scaling.py` | board | devmem, zdma: yes; uio: no | B1 | `t = F + n_words·W` with p50/p99/p99.9/max, reads and writes, over `devmem`, `uio`, `zdma`, `dram-uncached`, `dram-cached` |
| `devmem_profile.py` | board (or a backing file anywhere) | for `/dev/mem` | B2 | a sweep step in Python: posted and sync write + read; `--dev /dev/uio0`, `--read-transport zdma` |
| `devmem_profile.c` | board | yes | B2 | the same step in C, per-word (`volatile`) or `--bulk` |
| `fast_path_timing.py` | board | yes | B3 | control_sw calls over katcp vs LocalMem; `firmware_lib.read_accumulated_data_fast` at 1–2048 tones, devmem and ZDMA |
| `run_stage.sh` | board | yes | B0–B3 | all of the above for one stage, servers stopped |
| `board_checks.py` | board | yes, servers stopped | B5 | katcp vs devmem bit-exact on one accumulation, PSB inmap round trip, sparse LO write isolation, katcp writes at both ends of the map |
| `control_write_check.py` | board | no (UIO); idle servers may stay up | — | LO control writes through the RAM copy: katcp cross-read, tone isolation, copy vs firmware; old vs new write timings on spare slot 3 (restored) |
| `host_checks.py` | host | no | B6–B8 | `peripherals` (lock/ready flags), `sync` (timed sync round trip; needs `timing-monitor`), `sweep` (fit fr/Qi/Qc, compare to a reference run) |
| `rate_ladder.py` | host | no | B4 | stream / `get_samples` ladder: delivered/s, loss, startup and steady small/large gaps, `packet_error` |
| `acc_cnt_probe.py` | board | no (UIO) | — | tight `acc_cnt` read loop: impossible jumps, backward steps, slow reads (`--double-read` tests the fix) |
| `zdma_channel_check.py` | board | yes | — | one accumulation through a given DMA channel + u-dma-buf, compared bit for bit with devmem |
| `loop_timing_summary.py` | host | no | B4 | per-capture summary of `rate_ladder.py --loop-timing` rows |
| `retune_during_stream.py` | host | no | test 2 | lost accumulations per `set_tone_frequencies` / `_amplitudes` / `_phases` request during a stream |
| `perf_cpu_sampler.py` | board | yes | — | perf-event samples of chosen CPUs (kernel symbols, thread names) per 100 ms, for a kernel without ftrace |
| `first_retune_profile.py` | board | no (UIO); servers stopped | test 4 | cProfile of a first and a second `set_tone_frequencies_fast` in a fresh process, tones re-sent unchanged |
| `sweep_step_timing.py` | host | no | test 3 | sweep time per point and fixed cost (11/101/401 points); `set_tone_powers` with and without optimisers; state restored |
| `shadow_outside_writer_check.py` | board | no; servers up | test 7 | a katcp write to one LO word: does the server's copy report it, does the next request undo it; restored |
| `dual_soak.py` | host | no | test 10 | both pipelines streaming for a long time with no requests during the recording; every lost and torn accumulation, with times. `--modulate HZ [--track]` adds fw modulation and dry-run tracking; records whether each server runs a stream process |
| `acc_hold_check.py` | board | no with `--uio`; servers must be UP (clocks) | — | whether an accumulation stays unchanged in the accumulator until `acc_cnt` moves, read at fractions of the period -- the premise of the stream loop's late read (`catch_up`); needs a live signal |
| `clock_rate_check.py` | board | no with `--uio`; servers must be UP | — | accumulation rate and ADC clock (from the TT tick rate) against what the software assumes; run after a reboot -- on 2026-09-22 both were 13/12 slow with every PLL "locked" |
| `frame_path_bench.py` | board or PC | no | — | RAM only: per-frame pieces of `prepare_frame` and `tracking.tap` against their alternatives (fancy-index vs slice gather, numpy frame vs one join, int64 tap variants) |
| `gil_yield_check.py` | board | no; servers may stay up | — | whether a second thread (the tracking consumer) can share the stream loop's process: spin vs yielding accumulation wait, consumer batch time and accumulations lost per second |
| `dump_addrs.py` | board | — | — | offsets from a live readout object (fallback to `fpg_addrs.py`) |
| `tracking_benchmark.py`, `tracking_health_benchmark.py` | host | no | — | tone tracking; unrelated to this document |

Test numbering (B0–B9) and the master result tables live in
`doc/v712-notes.md`, "Staged comparison".

## Running a firmware stage

A stage is a firmware configuration (S0 = v7.11.2 … S3 = full v7.12.1; see
`doc/v712-notes.md`). For each one:

1. **Deploy the tools** to the board — copy files, never `scp -r` the repo
   (`.git` is 1.8 GB). Then build the C profiler there:
   ```bash
   scp profiling/{*.py,*.c,Makefile,run_stage.sh} casper@10.11.11.11:~/srt_test/profiling/
   ssh casper@10.11.11.11 'cd ~/srt_test/profiling && rm -f devmem_profile && make'
   ```
   `make` can silently keep an x86 binary left by an earlier copy; always
   `rm -f devmem_profile` first.
2. **Board-side tests (B0–B3)**, about 10 minutes:
   ```bash
   cd ~/srt_test/profiling && sudo ./run_stage.sh S0
   ```
   It stops `readout_server_0`, `readout_server_1` and `timing-monitor`, and
   restarts the servers on exit. B1/B2 overwrite pipeline 0's TX LO control
   buffer, so re-set tones afterwards. UIO, ZDMA and the DRAM runs happen only
   when `user_io` and `/dev/udmabuf1` exist (S3).
3. **Copy the results back:** `scp -r casper@10.11.11.11:~/srt_test/profiling/stages/S0 <somewhere>`.
4. **Rate ladder (B4)** from the host, servers running:
   ```bash
   PYTHONPATH=src .venv/bin/python profiling/rate_ladder.py --label S0 --repeats 3 --outdir <dir>
   PYTHONPATH=src .venv/bin/python profiling/rate_ladder.py --label S0 --tones 100  --outdir <dir>
   PYTHONPATH=src .venv/bin/python profiling/rate_ladder.py --label S0 --tones 2048 --outdir <dir>
   ```
5. **Fill** the stage column in `doc/v712-notes.md` and in Part 3 below.

---

## Part 1: katcp vs LocalMem, then Python vs C (v7.10.3)

### katcp vs LocalMem (late 2025)

`doc/accumulators.md` timed the same control_sw call over both transports on
the board:

| call | katcp (tcpborphserver) | LocalMem (`/dev/mem`) |
|---|---|---|
| `acc.get_new_spectra()` | 116 ms | 18.3 ms |

That settled the transport choice, but the numbers are mostly not the
transport: `get_new_spectra()` waits for the next accumulation by polling at
0.1 s (`_wait_for_acc`), so both are dominated by the wait. Stage test B3
(`fast_path_timing.py`) re-times the read itself, `acc._read_bram()`, over both
transports.

### What's being profiled

A **sweep step** =
1. **write** the tone control buffer into the mapped window
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
| `axil_mm[a:b] = v.tobytes()` (write) | `write_control_buffer_data_fast` |
| `np.frombuffer(axil_mm[a:b], '<i4')` (read) | `read_accumulated_data_fast` |

The mmap parameters match casperfpga `LocalMemTransport`
(`transport_localmem.py`): `/dev/mem`, offset `0xA0000000`, 512 MiB,
`MAP_SHARED`. The window was 32 MiB until firmware v7.12 moved the accumulator
and LO control buffers to AXI4 BRAMs at `0xB000_0000`; casperfpga widened
`MAP_SIZE` to `0x20000000` to reach them, and these profilers follow.

### Build & run

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
(C also has `--bulk`; Python also has `--read-transport zdma`).

### Write timing: posted vs. completion-forced

MMIO writes are *posted* — the store returns once the bytes are in the CPU
store buffer / interconnect FIFO, before the AXI slave has accepted them. So
the profilers measure the write **twice**: `write(posted)` (stores handed off,
can imply rates above the physical bus limit) and `write(sync)` (a same-slave
read-back barrier forces the posted writes to drain to the endpoint first).
The step total uses `write(sync)` by default; `--no-sync-writes` switches it to
the (unphysical) posted time. Trust `write(sync)` for real bus cost.

### Running against real hardware (/dev/mem)

On `/dev/mem` the write/read offsets must point at **real firmware regions**,
or the write loop clobbers whatever registers sit at offset 0. The control
buffer is exactly what a real sweep overwrites each step, and the accumulator
`dout0` is what it reads, so aiming there reproduces a real sweep.

Take the offsets from the **control_sw yaml the server loads**, not from an
`.fpg` path you picked — the server uses the dual-pipeline build:

```bash
python3 fpg_addrs.py ~/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml
# v7.11.2: --write-addr 0x00080000 --read-addr 0x00150000
# v7.12.1: --write-addr 0x13000000 --read-addr 0x10000000
```

They move every firmware generation (v7.11 split the ping-pong buffers into
`control0`/`control1`; v7.12 relocated both to `0xB000_0000`+), so never copy
them from this file. Then run the profilers (root required for `/dev/mem`):

```bash
sudo ./devmem_profile --dev /dev/mem \
    --write-addr <WRITE> --read-addr <READ> \
    --steps 10000 --tones 1000 --read-chans 1000 --bulk \
    --out devmem_profile_c_hw_bulk.txt

sudo ~/py3.12-venv/bin/python devmem_profile.py --dev /dev/mem \
    --write-addr <WRITE> --read-addr <READ> \
    --steps 10000 --tones 1000 --read-chans 1000 \
    --out devmem_profile_py_hw.txt
```

Sizes are set by the design maximum of `N_TONE = 2048` tones:

- control buffer: 2048 tones × 4 int32 words/tone × 4 B = 32 KiB per LO slot;
  the 128 KiB register backs the slots.
- accumulator: 2048 complex channels × 2 int32 (real/imag) × 4 B = 16 KiB.

> Profiling the WRITE overwrites the live TX control buffer, so it changes your
> current tone settings. Stop both readout servers first and re-apply tones
> afterwards. To measure hardware **read** cost only, pass `--no-write` to
> `access_size_scaling.py`.

### How to read the numbers

**On a backing file (RAM), Python can look *faster* than C. That is expected
and does not mean Python wins on hardware.**

- Python `mm[a:b] = bytes` is a single bulk `memcpy` over cacheable RAM.
- The C version accesses the window word-by-word through a `volatile
  uint32_t*` — no vectorization, one access per word — because that is what a
  real memory-mapped register window requires.

On real `/dev/mem`, **every access is a separate non-cacheable bus
transaction**; the bulk-memcpy advantage over RAM disappears. So:

- Use the **backing-file** runs to compare *language/interpreter overhead per
  call* (Python slice + `frombuffer` + numpy vs a C loop).
- Use the **`/dev/mem`** runs to measure *actual hardware transaction cost*.

The C profiler has both per-word `volatile` (default) and a `--bulk` mode that
does a single `memcpy` like Python. Running all three on hardware separates the
*access pattern* effect from the *language* effect.

### Results — firmware v7.10.3, 2026-06-10

Pipeline 0, `--write-addr 0x050000 --read-addr 0x090000` (`p0_mix_tx_lo0_control`,
`p0_acc0_dout0` in the v7.10.3 map), 1000 tones / 1000 read-chans, 10000 steps.
Medians, reproducible to the microsecond across runs. **Labelled v7.11 here
until 2026-09-16; it is v7.10.3.**

| mode | write | read | step | read ns/word | read MiB/s | write MiB/s |
|------|-------|------|------|--------------|------------|-------------|
| C per-word (`volatile`) | 199 µs | 421 µs | 620 µs | 211 | 18.0 | 76.2 |
| Python bulk (`mm[a:b]`) | 207 µs | 168 µs | 375 µs | 84 | 45.1 | 73.4 |
| **C bulk (`--bulk` memcpy)** | 196 µs | **163 µs** | **359 µs** | **82** | **46.5** | 77.4 |

The `write` column is the *posted* write; this run predates the posted/sync
split.

For reference, the in-RAM backing-file floor is ~17 µs/step — so hardware was
~20–37× slower, and that gap is bus latency, not CPU.

The read path dominates and is where the modes diverge. Decomposing the ratio:

| ratio (read) | value | isolates |
|--------------|-------|----------|
| C per-word ÷ C bulk | **2.58×** | **access pattern** — the whole effect |
| Python bulk ÷ C bulk | **1.03×** | **language** — essentially nil |
| C per-word ÷ Python | 2.51× | the two effects conflated (what we first saw) |

Writes tie everywhere (~50 ns/word, ~75 MiB/s): they are *posted*, so per-word vs bulk
doesn't matter. Reads are *blocking* round-trips — per-word `volatile` pays the
full ~211 ns latency on every word (18 MiB/s), while a bulk `memcpy` lets the bus
stream at ~82 ns/word (46 MiB/s).

Stability: C per-word has the tightest tail (0 steps > 2× median — ~2000
serialized reads resist scheduler interruption). The bulk modes are 2.5× faster
in the median but show a handful of sub-millisecond scheduler-jitter outliers
per 10000 steps.

---

## Part 2: v7.12.1 against Part 1

Same rig, same invocation, 2026-09-14: pipeline 0, 1000 tones / 1000
read-chans, 10000 steps, medians. Only the addresses change —
`--write-addr 0x13000000 --read-addr 0x10000000`.

**What is different in the hardware.** v7.12 moves the accumulator output and
the eight LO control buffers off 32-bit AXI4-Lite onto **128-bit AXI4 BRAMs at
`0xB000_0000`+**, and doubles the AXI control clock **100 → 200 MHz**
(`PL0_REF_CTRL` divisor `0xa → 0x5`, in `psu_init` inside `BOOT.BIN`). Against
v7.10.3 there is also everything v7.11 changed: the LO buffer split into
`control0/1` and, in v7.11.2, a higher LO BRAM read latency.

| mode | write(posted) | write(sync) | read | step | read ns/word | read MiB/s | write (sync) MiB/s |
|------|------|------|------|------|------|------|------|
| Python bulk (`mm[a:b]`) | 16.77 µs | 19.08 µs | 53.40 µs | 72.48 µs | 26.7 | 142.4 | 799.7 |
| **C bulk (`--bulk` memcpy)** | **14.91 µs** | **15.36 µs** | **44.49 µs** | **59.85 µs** | **22.2** | **171.0** | 993.4 |

Old against new:

| | v7.10.3 (Part 1) | v7.12.1 | gain |
|---|---|---|---|
| Python read | 168 µs | 53.40 µs | **3.15×** |
| C-bulk read | 163 µs | 44.49 µs | **3.66×** |
| Python write (posted, like-for-like) | 207 µs | 16.77 µs | **12.3×** |
| C-bulk write (posted) | 196 µs | 14.91 µs | **13.1×** |
| Python step | 375 µs | 72.48 µs | **5.2×** |
| C-bulk step | 359 µs | 59.85 µs | **6.0×** |

The v7.12 `step` uses the honest `write(sync)` against Part 1's posted write,
which *understates* the step gain slightly — posted-to-posted it is 5.34× for
Python. Hardware is now only ~3.5× the in-RAM floor where it was ~21×.

**The write gain was not predicted and dominates the step.** The upgrade plan
forecast ~4× on the *read* only. C-bulk reads came in at 3.66×, close to that;
Python at 3.15×, because its fixed per-call overhead is now a large share of
the read (below). Writes improved **12–13×**. How much of each gain belongs to
the AXI4 path and how much to the clock is not known — Part 3's stages S1/S2
separate them. (An earlier version of this section explained the read shortfall
as "blocking round-trips staying latency-bound"; C-bulk's 3.66× does not
support that.)

C-bulk's **22.2 ns/word / 171 MiB/s is 94% of the ~190 MB/s (≈181 MiB/s)** that upstream
(`zdma_read_example.py`) measured copying out of **DRAM** through an uncached
mapping — no fabric involved. So the read is now limited by the CPU's uncached
mapping, not by the bus, and only DMA into a *cached* buffer gets past it.
An earlier claim here that "~42 ns/word would indicate a stale `BOOT.BIN`" was
a guess; Part 3's S1 measured the real thing, 33.4 ns/word at 100 MHz. Check
the clock by reading `PL0_REF_CTRL` (`board_state.py`), not from throughput.

### The language gap reopened

| ratio | v7.10.3 | v7.12.1 |
|---|---|---|
| Python ÷ C-bulk, read | 1.03× | **1.20×** |
| Python ÷ C-bulk, step | 1.045× | **1.21×** |

Nothing about Python got slower. Its fixed per-call overhead (~9 µs on the
read: the `bytes` allocation behind `mm[a:b]`, then `np.frombuffer`) is
unchanged — but when the bus took 163 µs that cost was invisible, and at 44 µs
it is a fifth of the total. **The bottleneck has moved from the bus into
software.**

Still true: per-word `volatile` C remains far worse than bulk anything, so any
rewrite must stay bulk. Against a rewrite: 12.6 µs/step is second-order next to
the ~300 µs/step already gained, and these profilers measure *only* the devmem
operations — a real sweep step also builds frames and computes tone parameters
in Python, so the in-situ share is smaller than 21%. Measure `get_sweep_data`
end to end before deciding.

---

## Part 3: staged comparison, v7.11.2 to v7.12.1

### Why

Part 2 compares v7.10.3 with v7.12.1, and the first attempt at a v7.11.2
baseline (2026-09-15, reverted board) measured an LO control BRAM rather than
the accumulator, with an F that included `np.frombuffer`. The rate scans of
the same days also changed software between columns. So the board is stepped
through the PR one part at a time, with software held fixed:

| stage | firmware | BOOT.BIN | image.ub | isolates |
|---|---|---|---|---|
| S0 | v7.11.2 | pre-v7.12 | pre-v7.12 | baseline |
| S1 | v7.12.1 | pre-v7.12 | pre-v7.12 | AXI4 BRAMs at 100 MHz |
| S2 | v7.12.1 | v7.12.1 | pre-v7.12 | the 200 MHz clock |
| S3 | v7.12.1 | v7.12.1 | v7.12.1 | kernel rebuild; UIO and u-dma-buf |

Full definitions, what is held fixed, and the B4–B9 tables:
`doc/v712-notes.md`, "Staged comparison". The tables below are the profiler
ones, with the earlier measurements kept alongside for comparison.

### B2 — sweep step, same invocation as Parts 1 and 2 (p50 µs)

| mode | op | v7.10.3 (Part 1) | S0 | S1 | S2 | S3 | v7.12.1 (Part 2) | S3 ZDMA read | S3 UIO |
|---|---|---|---|---|---|---|---|---|---|
| Python bulk | write posted | 207 | 199.2 | 31.6 | 16.74 | 16.80 | 16.77 | 16.89 | 16.77 |
| | write sync | — | 202.5 | 34.0 | 18.93 | 19.02 | 19.08 | 19.35 | 18.96 |
| | read | 168 | 166.7 | 75.0 | 53.37 | 53.97 | 53.40 | **26.73** | 53.82 |
| | step | 375 | 369.3 | 109.0 | 72.33 | 72.99 | 72.48 | **46.08** | 72.78 |
| C bulk | write posted / sync | 196 / — | 196.1 / 200.7 | 29.6 / 30.5 | 14.91 / 15.36 | 14.91 / 15.36 | 14.91 / 15.36 | — | — |
| | read | 163 | 157.9 | 66.0 | 44.46 | 45.09 | 44.49 | — | — |
| | step | 359 | 358.6 | 96.5 | 59.82 | 60.45 | 59.85 | — | — |
| C per-word | read / step | 421 / 620 | 420.1 / 620.6 | 330.6 / 451.1 | 230.1 / 290.4 | 230.1 / 290.4 | — | — | — |
| Python bulk | read MiB/s (8 KiB) | 45.1 | 45.6 | 101.4 | 142.3 | 140.9 | 142.4 | **283.2** | 141.3 |
| | write sync MiB/s (16 KiB) | — | 75.1 | 446.1 | 798.4 | 795.9 | 799.7 | 786.8 | 798.8 |
| C bulk | read MiB/s | 46.5 | 48.2 | 115.4 | 170.7 | 168.9 | 171.0 | — | — |
| | write sync MiB/s | — | 75.8 | 498.9 | 989.2 | 987.9 | 993.4 | — | — |
| C per-word | read MiB/s | 18.0 | 18.1 | 23.0 | 33.0 | 33.1 | — | — | — |

### B1 — transport scaling, accumulator target (p50)

| quantity | v7.11.2, wrong target, 2026-09-15 | S0 | S1 | S2 | S3 | v7.12.1, 2026-09-14 |
|---|---|---|---|---|---|---|
| F incl. `np.frombuffer` µs | 8.56 (`p1_mix_rx_lo0_control0`) | 9.00 | 8.69 | 8.79 | 8.81 | 8.31 |
| F excl. `np.frombuffer` µs | — | 2.25 | 1.94 | 2.04 | 1.97 | 1.64 |
| W ns/word | 89.5 (`p1_mix_rx_lo0_control0`) | 79.3 | 33.4 | 22.6 | 22.9 | 22.4 |
| W as MiB/s | 42.6 (same) | 48.1 | 114.2 | 168.8 | 166.6 | 170.3 |
| 2048-ch read p50 µs | 374.9 (same) | 333.4 | 145.1 | 101.1 | 102.2 | 93.4 (bare slice) |
| 2048-ch read p99.9 µs | — | 356.2 | 165.7 | 144.3 | 122.7 | — |
| merge threshold, channels | — | 57 | 130 | 195 | 192 | ~37 |

AXI-lite reference, `p0_chan_select_map0_map1` (never moves; shows the clock alone):

| quantity | S0 | S1 | S2 | S3 |
|---|---|---|---|---|
| W ns/word | 79.6 | 64.5 | 38.2 | 38.2 |
| W as MiB/s | 47.9 | 59.1 | 99.9 | 99.9 |
| 1024-ch read p50 / p99.9 µs | 171.7 / 205.5 | 140.4 / 160.8 | 86.6 / 125.4 | 86.7 / 104.5 |

LO control buffer `p0_mix_tx_lo0_control0`, bulk writes:

| quantity | S0 | S1 | S2 | S3 |
|---|---|---|---|---|
| read W ns/word (MiB/s) | 89.5 (42.6) | 40.2 (94.9) | 26.0 (146.7) | 26.2 (145.6) |
| write posted F µs / W ns/word | 0.78 / 49.9 | 2.41 / 7.6 | 2.56 / 3.8 | 2.65 / 3.8 |
| write sync F µs / W ns/word | 2.40 / 50.0 | 2.45 / 7.5 | 2.49 / 3.8 | 2.57 / 3.8 |
| write sync W as MiB/s | 76.3 | 508.6 | 1004 (0.98 GiB/s) | 1004 (0.98 GiB/s) |
| 1000-tone sync write p50 / p99.9 µs | 202.5 / 216.9 | 32.6 / 51.2 | 17.6 / 27.2 | 17.6 / 27.5 |

Transports at S3, accumulator target:

| transport | earlier probe (2026-09-14) | S3 F µs | S3 W ns/word | S3 W as throughput | S3 2048-ch p50 / p99.9 µs |
|---|---|---|---|---|---|
| devmem | F 1.64 µs, W 22.4 ns (bare slice) | 1.97 | 22.9 | 166.6 MiB/s | 102.2 / 122.7 |
| uio, as `casper` | 2.0 µs at 64 B, 93.2 µs at 16 KiB | 2.09 | 22.9 | 166.6 MiB/s | 102.3 / 117.3 |
| zdma | F 21.94 µs, W 1.4 ns | 17.1 | 1.7 | **2.19 GiB/s** | 30.8 / 48.2 |
| DRAM through an uncached mapping (CPU ceiling) | upstream: ~190 MB/s ≈ 181 MiB/s | 2.00 | 20.5 | **186.1 MiB/s** | 92.1 / 106.2 |
| DRAM through a cached mapping | — | 2.10 | 1.2 | 3.10 GiB/s | 13.5 / 23.5 |

### B3 — read path end to end (p50 µs, wait for next accumulation removed)

| call | earlier | S0 | S1 | S2 | S3 |
|---|---|---|---|---|---|
| katcp `acc._read_bram()` 2048 ch | `get_new_spectra()` 116 ms (wait-dominated) | 53 301 | 82 386 | 78 020 | 82 849 |
| LocalMem `acc._read_bram()` 2048 ch | `get_new_spectra()` 18.3 ms (wait-dominated) | 546 | 347 | 302 | 300 |
| katcp / LocalMem `acc.get_acc_cnt()` / devmem `read_acc_cnt_fast()` | — / 27.1 / 3.7 (v7.12.1) | 1008 / 27.4 / 4.0 | 1030 / 28.0 / 4.1 | 1023 / 27.8 / 3.8 | 1014 / 27.5 / 4.0 |
| `read_accumulated_data_fast`, 1 tone | 224 → 36 (v7.12.1, v1.8.0 → sparse reads) | 37.8 | 37.8 | 36.7 | 37.0 |
| … 64 / 341 tones | — → 41 / — → 65 | 49.6 / 104.7 | 44.2 / 73.3 | 41.6 / 65.1 | 42.1 / 65.6 |
| … 1000 tones | — → 109 | 225.8 | 132.1 | 110.2 | 110.7 |
| … 2048 tones | 329 → 278 | 531.3 | 322.7 | 277.8 | 279.2 |
| … 1000 / 2048 tones via ZDMA | 86 / 115 | — | — | — | 86.2 / 115.8 |

### Whole-register writes: devmem vs DMA (S3)

`access_size_scaling.py --write-bytes 0x20000 --zdma-write`: the DMA write
stages the payload in a u-dma-buf, syncs it to the device and DMAs it into the
LO register; every size is verified by reading the register back over devmem.
p50 / p99.9 µs:

| bytes | devmem posted | devmem sync | ZDMA |
|---|---|---|---|
| 16 000 (1000 tones) | 17.9 / 128.9 | 17.7 / 27.5 | 25.5 / 35.9 |
| 32 768 (one slot) | 33.9 / 55.5 | 33.8 / 43.1 | 35.6 / 47.1 |
| 131 072 (4 slots) | 126.2 / 144.5 | 126.0 / 140.1 | **94.2 / 116.7** |

Models: devmem sync 2.8 µs + 3.8 ns/word (1.0 GiB/s), ZDMA 16.0 µs + 2.4 ns/word
(1.56 GiB/s), crossing at ~38 KiB (just over one slot).

### LO control writes through a RAM copy (2026-09-17)

The bulk copy above is 17.6 µs for 1000 tones, but tone updates used per-word
stores (77–99 µs per LO), and the old "staged" option lost only because it
re-scattered values in Python (82 µs) before its bulk copy. Amplitudes and
phases went over katcp (~1.3 s per request).

`firmware_lib.get_control_shadow` now keeps a RAM copy of every LO control
register (read once when the server creates its interfaces);
`write_control_buffer_data_fast` places each field in RAM with one strided copy
and pushes the touched tone rows as one 64-byte-aligned slice.
`control_write_check.py`, pipeline 1, 1000 tones, TX + RX, one buffer, p50:

| write | per-word stores | through the copy |
|---|---|---|
| frequency (phase step + RI step) | 247 µs | **110 µs** |
| all four fields | 354 µs | **140 µs** |
| scaling, all 2048 rows | 251 µs | **134 µs** |
| frequency, 1000 non-contiguous tones | 248 µs | 236 µs |
| bulk copy alone, 4000 words | — | 18.8 µs |

Through the server: `set_tone_amplitudes` 1.26–1.34 s → 6 ms,
`set_tone_phases` → 7–9 ms, `set_tone_frequencies` 30 ms. Checks: katcp read
of all four registers equals the copy; one-tone isolation passes;
`get_info('control_shadow')` reports 0 stale words after every kind of write
and flags a word changed from another process. Design, limits and side
findings: `doc/v712-notes.md`, "LO control writes through a RAM copy".

### What S3 shows

- **The new kernel changes nothing for devmem** (accumulator 22.9 ns/word,
  167 MiB/s, vs 22.6 at S2; steps within 1%). Its value is the new transports.
- **The CPU ceiling is real:** plain DRAM through an uncached mapping reads at
  20.5 ns/word (186 MiB/s, in line with upstream's "~190 MB/s"), so the accumulator at
  167 MiB/s is at 90% of it. Through a cached mapping DRAM reads at 1.2 ns/word
  (3.1 GiB/s).
- **UIO is devmem without root:** identical cost, and rootless *writes* work.
- **ZDMA reads:** 1.7 ns/word (2.2 GiB/s) after a 17 µs fixed cost; 2048 channels in 31 µs
  against 102 µs; the sweep step drops 73 → 46 µs. Through the read planner,
  1000 tones 111 → 86 µs and 2048 tones 279 → 116 µs, the latter because ZDMA
  also avoids the slow full-buffer software path. Below ~480 tones the planner
  keeps devmem, as designed.
- **ZDMA writes** only pay above one slot: the whole 4-slot register 126 → 94 µs.

### What S1 and S2 show

- **The AXI4 BRAMs (S0 → S1, clock still 100 MHz)** make accumulator reads 2.4×
  faster per word (79.3 → 33.4 ns; 48 → 114 MiB/s) and LO writes 6.2× faster
  (76 → 509 MiB/s); the Python step
  goes 369 → 109 µs. The AXI-lite map also improves 19% (79.6 → 64.5 ns),
  because the v7.12 build reworked AXI-lite too.
- **The 200 MHz clock (S1 → S2)** adds 1.5× on reads (→ 22.6 ns/word,
  169 MiB/s, 91% of the 186 MiB/s DRAM-through-uncached-mapping ceiling
  measured at S3), 1.9× on writes (→ ~1 GiB/s),
  1.7× on AXI-lite; Python step → 72.3 µs.
- **Share of the firmware gain:** ~70–75% AXI4 BRAMs, ~25–30% clock.
- **S2 reproduces Part 2 (2026-09-14, full v7.12.1) to within 0.5%**, so the
  kernel in the new `image.ub` should not affect these devmem numbers.
- The 2048-tone full-buffer software path still costs ~177 µs above the bare
  read at S2 (278 vs 101 µs).

### What S0 (v7.11.2, 2026-09-16) already shows

- **The 2026-09-15 run did measure an LO buffer.** On v7.11.2 the accumulator
  reads at 79.3 ns/word (48 MiB/s) and the LO control buffer at 89.5 ns/word
  (43 MiB/s) — exactly the
  "v7.11 accumulator" figure. The AXI-lite channel map matches the accumulator
  (79.6). F without `np.frombuffer` is 2.25 µs, not 8.56.
- **v7.10.3 was a fair stand-in for v7.11.2.** The sweep step is 369 µs
  (Python) / 359 µs (C bulk) against 375 / 359 on v7.10.3, so Part 2's
  v7.10.3 → v7.12.1 ratios hold for v7.11.2 to within ~2%: Python step
  369 → 72 µs is 5.1×.
- **The board confirms S1 is well-posed.** With the pre-v7.12 `BOOT.BIN` the
  AXI clock reads back at 100 MHz and HPM1 is already 128-bit.
- **katcp costs ~100× LocalMem** for the same control_sw call (`_read_bram()`
  53 ms vs 546 µs; `get_acc_cnt()` 1 ms vs 27 µs).
- **The full-buffer software path costs ~200 µs above the bus read.**
  `read_accumulated_data_fast` at 2048 tones takes 531 µs against a 333 µs
  bare read, where at 1000 tones the overhead is ~60 µs. Worth a look.

---

## Conclusions, as they stand

1. **On v7.10.3 the fast path was bus-bound, not CPU-bound.** A sweep step was
   ~360–620 µs on hardware vs ~17 µs in RAM.
2. **Access pattern mattered far more than language.** On v7.10.3 C bulk and
   Python bulk were within 3%; the 2.5× read difference was per-word vs bulk.
   On v7.12.1 Python is 20% slower than C on the read, because its fixed
   per-call overhead no longer hides behind a slow bus.
3. **Don't rewrite the fast path in C — yet.** Any rewrite must stay bulk, and
   the gap is ~13 µs/step against the ~300 µs/step v7.12 already gained.
   Measure `get_sweep_data` end to end before deciding.
4. **On v7.12.1 CPU reads are at the uncached-mapping ceiling** (90% of what
   plain DRAM gives through the same kind of mapping, 167 of 186 MiB/s). Beyond
   it, only ZDMA into a cached buffer helps (2.2 GiB/s). In the stream it shows
   at 2048 tones (< 0.1% loss to 1800 Hz instead of 1200 Hz) and barely at
   1000 tones.
5. **The firmware gain splits ~70–75% AXI4 BRAMs, ~25–30% clock** (stages
   S0–S2, clock read back from `PL0_REF_CTRL`); v7.11.2 → v7.12.1 is 3.5× on
   reads, 11.5× on bulk LO writes, 5.1× on the Python benchmark step.
6. **Write the LO buffers in bulk, from a RAM copy.** Per-word stores gain only
   2.8× from v7.12; one bulk copy per LO buffer (with the fields placed in RAM
   first) takes a 1000-tone frequency write to TX + RX from 247 to 110 µs, and
   removes katcp from amplitude and phase writes.

## Files

- `board_state.py` — record the board configuration (B0)
- `fpg_addrs.py` — register offsets from the control_sw yaml or an `.fpg` header
- `access_size_scaling.py` — F/W and tails per transport and target (B1)
- `devmem_profile.py` — Python sweep-step profiler (B2)
- `devmem_profile.c` / `Makefile` — C profiler; per-word default, `--bulk` mode (B2)
- `fast_path_timing.py` — control_sw and firmware_lib read path, end to end (B3)
- `run_stage.sh` — B0–B3 for one stage, on the board
- `rate_ladder.py` — stream / `get_samples` rate ladder, from the host (B4)
- `board_checks.py` — correctness checks on the board, servers stopped (B5)
- `control_write_check.py` — LO control writes through the RAM copy: correctness + old/new timings
- `host_checks.py` — peripherals, timed sync and sweep + fit, from the host (B6–B8)
- `rt_tuning.sh` — moved to `src/souk_readout_tools/data/daemon/` and run by `souk-enable-tuning` (KRM board tuning)
- `zdma_channel_check.py` — validate a DMA channel + u-dma-buf against devmem
- `acc_cnt_probe.py` — tight `acc_cnt` read loop that records impossible jumps and backward steps
- `loop_timing_summary.py` — per-capture summary of `rate_ladder.py --loop-timing` rows
- `retune_during_stream.py` — lost accumulations per tone request during a stream (test 2)
- `perf_cpu_sampler.py` — perf-event CPU sampler with kernel symbols (no ftrace on the board)
- `first_retune_profile.py` — why the first retune after a restart is slow (test 4)
- `sweep_step_timing.py` — sweep per-point and `set_tone_powers` timing through the server (test 3)
- `shadow_outside_writer_check.py` — outside katcp writes against the server's RAM copy (test 7)
- `dual_soak.py` — long two-pipeline stream, counters and torn flags only (test 10)
- `dump_addrs.py` — offsets from a live readout object (fallback)
- `devmem_profile_{c,py}_hw.txt`, `devmem_profile_c_hw_bulk.txt` — v7.10.3
  hardware runs (Part 1); `devmem_profile_c.txt` — in-RAM backing-file floor.
  Gitignored, local only.
- `stages/` — per-stage results from `run_stage.sh` (gitignored)

> The compiled `devmem_profile` binary is architecture-specific and gitignored.
> Always `rm -f devmem_profile && make` on the board — an x86 binary on the ARM
> board fails with `Syntax error: "(" unexpected`.
