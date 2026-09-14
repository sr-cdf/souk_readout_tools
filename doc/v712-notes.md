# Notes on the firmware v7.12 upgrade

v7.12 (souk-firmware PR #121, tag `v7.12.1` = `4fb2812`) moves the
performance-critical FPGA buffers off AXI4-Lite onto 128-bit AXI4 BRAMs and
doubles the AXI control clock. It is the transport `profiling/README.md`
§Conclusions asked for: on v7.11 the sweep read path was measured as bus-bound
at ~84 ns/word, with the note that only a burst-capable transport could beat it.

Use **v7.12.1, not v7.12.0** — `4fb2812` fixes a regression in CPU access to
non-latency-1 AXI4 BRAMs, and v7.11.2 had already raised the LO control BRAM
latency to 2, which is exactly the configuration the devmem fast path uses.

Board upgraded and verified 2026-09-14. Software support shipped in **v1.7.10**.

---

## What moved

All **224 register names are byte-identical** v7.11 → v7.12 — nothing added,
renamed or removed. `get_new_spectra` keeps its 5-tuple, the accumulator dtype
is still `>i4`, `FW_TYPE_PARAMS` is untouched. The devmem fast path resolves
every address by name via `transport._get_device_address()`, and no file in
`src/` hardcodes a base address or map size, so the relocation cost **zero
code**.

| register | v7.11 | v7.12 |
|---|---|---|
| `p0_acc0_dout0` | `0xa0150000` | `0xb0000000` |
| `p1_acc0_dout0` | `0xa0154000` | `0xb5000000` |
| `p{0,1}_mix_{rx,tx}_lo0_control{0,1}` (8) | `0xa0040000`… | `0xb1000000`…`0xb9000000` |
| `p*_synth_input_reorder_map*_map1` (8) | `0x2000` bytes | `0x1000` bytes ← **16-bit map** |

The AXI-Lite map also got denser (it ends at `0xa0078420` instead of
`0xa0188420`), so *every* AXI-Lite address changed too. Total span **1.5 MiB →
400.1 MiB** (`0xa0000000`–`0xb9020000`).

The ten relocated devices carry **no `xps:bram` metadata** in the new `.fpg` —
only `?register` lines. Harmless: `casperfpga._create_memory_devices` falls
through to a plain `Memory(...)`, so name lookup and `read`/`blindwrite` keep
working; they just stop appearing in `host.sbrams`. The repo has exactly one
`.length_bytes` use (`firmware_lib.py:9966`) and it is on `acc0_snapshot_bram`,
which **keeps** its metadata, and no `.sbrams` use at all.

## Mandatory on the board

A new `.fpg` alone is **not** enough. All of these must land together:

- **casperfpga `a89480c`** — `transport_localmem.MAP_SIZE` 32 MiB → `0x20000000`
  (512 MiB). Without it the accumulator and all eight LO control buffers fall
  outside the mmap window. The span is 400.1 MiB, so 512 MiB covers it.
- **control_sw `7.12.1`** — `Fpga.check_firmware_support()` prefix-matches on
  `major.minor.rev`, so 7.11.2 ⟷ a v7.12 bitstream fails init in **either**
  direction. No mixing.
- **souk_readout_tools ≥ 1.7.10** — the 16-bit reorder-map fix below.
- **`top.bit`** — the base PL image carries the PS-PL interface configuration,
  which is where the HPM1 enable lives. It is *not* in `psu_init`.
- **`image.ub`** — new file on v7.12 (9.5 MB), carrying the kernel and device
  tree with `generic-uio` and the eight `udmabuf@N` nodes. Strictly only needed
  for the DMA/UIO work, but it was built against the same XSA as `BOOT.BIN`.

`tcpborphserver3` needs **no** rebuild. PR #121 does not touch `software/zynq/`
at all, and the server sizes its own AXI window from the loaded `.fpg`'s
register extents (`raw.c:2731-2732`), re-running it on every program. Its
startup log line `mapping area` should report **`0x19020000`** rather than the
old `~0x189000`. The `power`/`window` cap nearby is inside `#ifdef __PPC__`.

One upstream wart to know about: `raw.c:944` and `:1203` bounds-check katcp
writes by truncating a 64-bit mmap pointer to `uint32_t`. Harmless at 1.5 MiB,
reachable across the new 400 MiB span. It fails loudly with
`register %s is outside mapped range`, so it is diagnosable rather than silent.

## BOOT.BIN is a performance dependency, not a correctness gate

`psu_init` has exactly two functional deltas:

1. `PL0_REF_CTRL` divisor `0xa → 0x5` — the 100 → 200 MHz AXI control clock.
2. `FPD_SLCR.AFI_FS` mask `0x300`/value `0x0` → mask `0xF00`/value `0x800`.

The second is belt-and-braces. It sets `DW_SS1_SEL` (HPM1, where the new BRAMs
live) to `0b10` = 128-bit — **the same value the silicon already holds at
power-on**. The generated comment in `psu_init.c` reads *"00: 32-bit AXI data
width (default)"*, which looks alarming, but that is Xilinx's generic
field-encoding prose, not this register's reset value. The authoritative number
is in `psu_init.h`:

```
FPD_SLCR_AFI_FS_DW_SS1_SEL_DEFVAL   0x00000A00   SHIFT 10   MASK 0xC00
```

`0xA00` → bits [11:10] = `0b10` = 128-bit. The old `psu_init` masked only
`0x300` = SS0 [9:8] and never touched SS1. **Do not re-litigate this from the
`.c` comment.**

So a stale `BOOT.BIN` costs the 200 MHz half of the speedup, not correctness —
which makes "old BOOT.BIN + new .fpg" a legitimate bisection step. A read
measuring ~42 ns/word instead of ~22 is the signature of the clock still at
100 MHz.

## Boot source, and updating the boot files in place

The BMC boot mode on this board is **`5 - SD1`** (the SOUK standard, see
`doc/board_management.md`), so `BOOT.BIN` is read from the SD card's FAT32
`BOOT` partition — **not QSPI**. Confirm before touching anything:

```bash
python -m souk_readout_tools.board_management status     # -> "Boot from : 5 - SD1"
```

That matters because it decides the rollback. With SD1, a second SD card is a
complete rollback and the update is a file copy. Had it been QSPI, swapping
cards would not roll `psu_init` back and the old image would need reading out
first.

With no physical access to the card, the files were replaced **in place** from
the running system. `BOOT.BIN` is the one file whose corruption is not
recoverable remotely — the BootROM loads it directly, so a bad one means no
U-Boot, no serial recovery, no TFTP. `image.ub` is much safer by comparison:
U-Boot still runs, so a bad kernel is recoverable over the console. Copy to a
temporary name, `sync`, drop caches, verify the sha256 **read back from the
card**, and only then rename over the live file.

## The one code change this repo needed

`r.psb_chanselect` is `VaccReorderMultiSampleIn`, whose `_map_format` narrows
`>i4 → >i2` in v7.12 (the register size halving `0x2000 → 0x1000` for the same
2048 entries confirms 2 bytes/entry in hardware). Two byte-identical cache
blocks in `psb_chanselect_{set,get}_channel_inmap` took only the *byte order*
from `_map_format` and hardcoded the itemsize to 4. Both now derive the itemsize
too. This is version-agnostic: for `>i4` it yields depth×4, `<u4`, `>u4` —
byte-identical to the old path.

Also fixed in the same pass: a `hasattr` typo in
`prepare_control_buffer_data_fast` that rebuilt the whole-map `np.frombuffer`
view on *every* tone write (harmless at 32 MiB, 128 M elements at 512 MiB), and
the `profiling/` MAP_SIZE plus two already-broken address helpers.

## Verification results

Steps as numbered in the upgrade plan. All run on 2026-09-14.

| # | check | result |
|---|---|---|
| 1 | `pytest tests/` | pass (338) |
| 2 | `.fpg` header vs the shipping build | pass — 224 names identical, moves as tabled |
| 3 | derived map width byte-identical on `>i4` | pass |
| 4 | fw version / `fw_supported` / `fpg_file` / `adc_clk_hz` | `7.12.1`, `True`, `2026-07-22_1653`, 2457600000 |
| 5 | `/proc/cmdline`, `/dev/uio*`, `/dev/udmabuf*` | pass — uio0/1, udmabuf0..7 |
| 6 | katcp vs devmem accumulator | pass — median ratio 0.99957 |
| 7 | PSB inmap round-trip | pass — **no pair swap needed** |
| 8 | sparse control-buffer write isolation | pass — 8/8 |
| 9 | PTP GM lock | pass |
| 10 | sweep + fit vs a v7.11 resonator | **outstanding** — needs the cryostat |
| 11 | profiler re-baseline | pass — see below |

Two results are worth stating explicitly because both were flagged as genuine
unknowns going in, and both resolved without needing any code change:

**Step 7 — the 16-bit map needs no byte swap.** The concern was that two 16-bit
fabric words packing into one 32-bit little-endian devmem word would need the
same `.reshape(-1,2)[:, ::-1]` transposition the fast ADC/DAC snapshots require.
It does not: slow-write → fast-read → slow-read is exact, and so is
fast-write → slow-read. Note this exercises the AXI-**lite** path — the reorder
maps stayed at `0xa006xxxx`.

**Step 8 — sparse 32-bit writes do not clobber the 128-bit line.** One tone is
`_CONTROL_N_WORDS = 4` × 4 B = 16 B = exactly one 128-bit line, and the sparse
branch writes individual words into it. All eight combinations (tx/rx × buf 0/1
× slot 0/1) changed exactly the intended word. This confirms on hardware what
the `axi4_bram` yellow block claims in source: `Use_Byte_Write_Enable` +
`Byte_Size 8` wired to `blk_mem_gen/wea` means narrow CPU accesses arrive as
strobed lanes. **No read-modify-write is needed anywhere.**

Step 6 was run with the ADC input open, i.e. on noise. The naive "compare the
ten strongest channels" test is meaningless there — the accumulator median is
exactly zero (only mapped channels are non-zero) and the rest is noise, so
independent reads scatter. The valid form averages the per-channel noise floor
(which has real filterbank shape) and uses a **same-transport repeat as the
control**: devmem-vs-devmem 0.923, devmem-vs-katcp 0.890, median ratio 0.99957.
Modelling each estimate as true+independent noise, `corr = 1/sqrt((1+n₁)(1+n₂))`;
the control gives n = 0.0834, predicting 0.889 for the half-averaged katcp
cross-check — measured 0.890. The two transports read identical data.

## Measured performance

1000 tones / 1000 read-chans / 10000 steps, medians, same invocation as the
v7.11 baseline. Full table and analysis in `profiling/README.md`.

| | v7.11 Python | v7.12 Python | v7.12 C-bulk |
|---|---|---|---|
| write | 207 µs (posted) | **16.77 / 19.08 µs** (posted/sync) | 14.91 / 15.36 µs |
| read | 168 µs | **53.40 µs** | 44.49 µs |
| step | 375 µs | **72.48 µs** | 59.85 µs |
| read ns/word | 84 | **26.7** | 22.2 |

**A sweep step went 375 µs → 72 µs, 5.2×** (6.0× in C), past the plan's
predicted ~4×. The read gain (3.2×) came in slightly under prediction because
reads are blocking round-trips and stay latency-bound; the **write gain of
12.3× was never predicted** and dominates the step — writes are posted and
stream well, and a 32-bit port at 100 MHz became a 128-bit port at 200 MHz.
C-bulk's 22.2 ns/word is 94% of the ~190 MB/s uncached-Device ceiling, so the
read path is now close to the limit of the CPU's uncached mapping.

Side effect worth knowing: the Python-vs-C gap widened from **3% to 21%**, not
because Python got slower but because its fixed ~9 µs per-operation overhead no
longer hides behind a slow bus. The bottleneck has moved from the bus into
software. This weakens but does not overturn the standing "don't rewrite the
fast path in C" — per-word C remains far worse than bulk anything, and these
profilers measure only the devmem operations, not the Python work wrapped around
them in a real sweep step. Measure `get_sweep_data` end-to-end before deciding.

## Still outstanding

- **Sweeps are not yet confirmed unbroken** (step 10). The profilers write and
  read the right *addresses* but never meaningful *values*, and no sweep or fit
  has been run since the upgrade. Confirming a known resonator still fits to the
  same `fr`/`Qi`/`Qc` as v7.11 is the strongest remaining check.
- **Timestream integrity at high rate is improved; the rate ceiling is not yet
  characterised.** `packet_error` flags a read that straddled an accumulation
  boundary, so it scales with how much of the window the read occupies: at
  2000 Hz (500 µs window) that went from **34% on v7.11 to 11% on v7.12** as the
  read shrank 168 → 53 µs. Post-upgrade `packet_error` is 0 at every rate up to
  2000 Hz, which was not true before — a real and attributable win. Separately,
  the residual sample loss at 1800–2000 Hz appears as `packet_counter` gaps with
  `packet_error` still 0, i.e. the loop returning late rather than the read
  overrunning, which points at the per-sample framing and `await writer.drain()`
  path in `readout_server.get_samples`. A like-for-like rate scan against v7.11
  is needed to say whether the achievable rate itself has risen.
- **UIO (dropping root)** — the device tree exposes `user_io@a0000000` and the
  nodes are present, but nothing has been run against them. Expected to be the
  same speed as `/dev/mem`; it buys permissions, not latency.
- **ZDMA into a cached `u-dma-buf`** — the only route past the ~190 MB/s
  uncached ceiling, since that ceiling belongs to the CPU mapping rather than
  the fabric. `software/rfsoc_scripts/zdma_read_example.py` on `v7.12` is a
  hardware-validated reference. Not attempted.

## Gotchas met during the upgrade

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
