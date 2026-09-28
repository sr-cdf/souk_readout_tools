# Version Compatibility

Which `souk_readout_tools` releases run on which `souk-firmware` versions, what
the board's OS image, kernel and boot files had to be at each point, and which
boards each firmware generation was built for. [CHANGELOG.md](../CHANGELOG.md)
remains the record of *what changed*; this page records *what goes with what*.

Rebuilt on 2026-09-22 from the history of this repository and of
`souk-firmware` (tags, commit dates, `.fpg` outputs per design, the
`software/petalinux` tree). Entries marked *(inferred)* come from the
circumstances of a commit rather than an explicit statement. Please correct them
when you know better.

- [Software ↔ firmware ↔ board](#software--firmware--board)
- [Firmware timeline](#firmware-timeline)
- [OS image, kernel and boot files](#os-image-kernel-and-boot-files)
- [Platforms](#platforms)
- [Peripheral hardware support](#peripheral-hardware-support)
- [Keeping this page current](#keeping-this-page-current)

---

## Software ↔ firmware ↔ board

The **Firmware** column shows the firmware each release was written and tested
against. A release has no version gate of its own, so older firmware is
refused only where a register or API it needs is missing. From v1.8 onwards the
software minor version follows the firmware generation (v1.8 = v7.12); before
that, the two numbering schemes are unrelated.

| Software | Date | Firmware | Board OS / boot needed | Notes |
|---|---|---|---|---|
| *(pre-release)* | 2024-08-29 → 2024-11 | v7.3.0.1 (tag `Firmware_v7.3` in this repo, 2024-09-27) | Ubuntu 20.04 CASPER image, Python 3.8 venv | KRM single pipeline, 2.048 GHz config (`souk-single-pipeline-krm-2G.yaml`). |
| v1.0.0 → v1.0.1 | 2024-11-15 | v7.3.x | as above | `setup.py` stayed at 1.0.1 until March 2026, so "v1.0.1" covers everything in the rows below up to v1.1.0. `python_requires >= 3.8`. |
| *(1.0.1 rolling)* | 2025-03 | v7.4.x / v7.5.x | as above | Single control buffer from v7.5 (`27aedab`), Nyquist zone 2 fix (`30105dd`). The lab configs were named `config_rf_loopback_7.4.{0,1,2}` *(inferred)*. |
| *(1.0.1 rolling)* | 2025-10-16 | v7.6+ | as above | Sweep phase correction off by default, "fixed in firmware": the v7.6 PSB phase fix *(inferred)*. |
| *(1.0.1 rolling)* | 2025-11-24 → 2026-01 | v7.9.x (multitone branch) | as above | Burst mode, ADC over-voltage flags, `psb_chanselect` inmap; a pre-/post-v7.9 wrapper was added at the time. Dual pipeline from 2025-12-17, VACC multitone from 2026-01-20. |
| **v1.1.0** | 2026-03-18 | v7.9.5.x (7.9.5.4 tagged 2026-03-10) | **New image: Ubuntu 24.04, Python 3.12** (`krc4700-souk-u24`, `/home/casper/py3.12-venv`) | `python_requires >= 3.10` from 2026-03-26. Telescope-time (PTP) stamps in every frame. Test plan written for a fresh image (`doc/test_plan.md`). |
| v1.1.1 | 2026-04-20 / 04-28 | v7.9.5.4 | Ubuntu 24.04 | RF peripherals restructure, LNA bias, deprecated APIs removed. |
| v1.2.0 | 2026-05-05 | v7.9.5.4 | + `ptp4l`, chrony, `souk-timing-monitor` services (`souk-enable-timing`) | Software installs these onto the image. |
| v1.3.0 | 2026-05-08 | v7.9.5.4 | Ubuntu 24.04 | G3 streaming (so3g); optional from v1.6.x. |
| v1.4.0 | 2026-06-05 | v7.9.5.4 | Ubuntu 24.04 | Software-engine fast frequency modulation. |
| **v1.5.0** | 2026-06-26 | **v7.10** timed-sync (v7.10.2 dual on the lab board; v7.10.3 latest) | + healthy TSU PPS strobe (`souk-tsu-strobe`), PTP lock | Timed sync needs a v7.10 `souk_mkid_readout` and says so if the installed one is older. |
| **v1.6.0** | 2026-07-06 | **v7.11** (7.11.2) — **required** | Ubuntu 24.04 | Breaking: `acc_len` moves to the mixer, LO control `control{0,1}` + slot layout, `get_new_spectra` returns 5 values. v7.10 and older no longer work. Firmware-slot modulation. |
| v1.6.1 → v1.6.7 | 2026-07-15 → 07-28 | v7.11.x | Ubuntu 24.04 | RFSoC4x2 fixes in v1.6.1 (`krc-utils` checks, `needs_programming` false positive). |
| v1.7.0 → v1.7.9 | 2026-07-28 → 09-14 | v7.11.x | Ubuntu 24.04 | Software-only releases on 1.7.x (v1.8 was held for the firmware change). v1.7.0 adds LNA bias board v2; v1.7.9 adds BMC over USB. |
| **v1.7.10** | 2026-09-14 | **v7.12** code support (v7.11 still works) | For v7.12: **`BOOT.BIN`, `boot.scr`, `image.ub`, `top.bit`**, casperfpga `a89480c`, control_sw `7.12.1` | Reorder-map word width taken from the firmware. On v7.12, tones set through the fast path land in the wrong bin until v1.8.1 (see below). |
| **v1.8.0** | 2026-09-14 | **v7.12.1** validated on hardware | as v1.7.10 | First release whose minor version tracks the firmware generation. |
| **v1.8.1** | 2026-09-22 | v7.12.1 — **use this or later on v7.12** | + optional `souk-enable-tuning` (CPU isolation, `souk-rt-kernel.service`, per-core TX queues), `/dev/ptp-end0` udev name | Fixes the v7.12 PSB reorder-map pair swap: every tone set through the fast path on v7.12 before 2026-09-16 landed in its pair-neighbour's bin. v7.11 data is unaffected. |
| *Unreleased* | — | v7.12.1 | as v1.8.1 | RF mixerless module selected by `rf_frontend.mixerless_module.model`. |

**Minimum software per firmware**, read from the table:

| Firmware | Minimum software | Last software known to work |
|---|---|---|
| v7.3 – v7.5 | pre-release / 1.0.x | 1.0.1 of early 2025 *(inferred; later code assumes v7.9 inmap semantics)* |
| v7.9.x | late-2025 1.0.1 (dual pipeline: 2025-12-17) | v1.5.0 *(inferred: v1.5.0 still ran on v7.9 apart from timed sync; untested)* |
| v7.10.x | v1.5.0 | v1.5.0 |
| v7.11.x | v1.6.0 | current (1.8.x keeps the v7.11 paths; not re-validated) |
| v7.12.x | v1.7.10 (tone placement wrong on the fast path) → **v1.8.1** | current |

---

## Firmware timeline

From `souk-firmware` tags and commits. "KRM-S" and "KRM-D" are the single- and
dual-pipeline designs, both for the same KRM platform; "4x2" is the RFSoC4x2
design. Each column means a build of that design was present in
`firmware/src/*/outputs` at that point. Versions that were never
tagged are dated by their build commit.

### v3 – v6 (before this software existed)

| Version | Date | Change | Builds |
|---|---|---|---|
| v3.0 | 2023-03-14 | Zoom mode | 4x2 (and generic `souk_single_pipeline`) |
| v4.0 – v4.2 | 2023-03-28 → 05-02 | Polyphase synthesis bank; per-tone PFS scaling | 4x2 |
| v5.0 – v5.2 | 2023-05-03 → 05-06 | Pair of synthesis banks; DAC inverse-sinc on; 4x2 PPS pins | 4x2 |
| v5.4 – v5.5.5 | 2023-06-01 → 08-03 | Zoom-FFT channel select; PFB reorder parallel-first; single-pipeline RF on both DACs | 4x2 |
| v5.5 (KRM) | 2023-08-18 / 09-07 | **First KRM model and build**; also a 0.5 MHz-channel `v5.5-hires` and an RX-only KRM design | 4x2, KRM-S, KRM-RX |
| v5.6.1 | 2023-11-30 | `tcpborphserver` binaries committed | 4x2, KRM-S |
| v6.0 – v6.1 | 2023-09-23 → 10-30 | KRM single and dual models; PPS on GPIO B84.07 and LED outputs | KRM-S, 4x2 |
| v6.2 – v6.2.2 | 2024-01-18 → 02-01 | KRM clock config files (differ from 4x2) | KRM-S |
| v6.3 – v6.3.2 | 2024-02-01 → 04-11 | Single DAC by default; DAC0/DAC1 swap fix (#38); fine NCO | KRM-S |
| v6.4 | 2024-04-09 | New casperfpga status interface | KRM-S |

### v7.x

| Version | Date | Change | Builds | Software |
|---|---|---|---|---|
| v7.1 / v7.1.1 | 2024-04-11 / 04-15 | RX vs TX sync delay in firmware | KRM-S | — |
| v7.2 | 2024-05-16 | 2.048 GS/s variant (2024-05-10) | KRM-S | — |
| v7.3.0.1 | 2024-07-24 | | KRM-S | first target (Aug 2024) |
| v7.4 | 2024-09-10 → 10-08 | Phase-switch model; **first dual-pipeline KRM build** (2024-09-17) | KRM-S, KRM-D | 1.0.x |
| v7.4.1 / v7.4.2 | 2025-02-03 / 02-04 (tag 2025-04-13) | v7.4 fixes, dual pipeline | KRM-S, KRM-D | 1.0.x |
| v7.5 – v7.5.5 | 2024-10-08 → 2025-04-16 | Single LO control buffer; ±1.024 GHz LOs for 2.048 GHz BW; v7.5.5 PSB FIR/iFFT (reverted 2025-04-29) | KRM-S, KRM-D | 1.0.x (Mar 2025) |
| — | 2025-01-26 | **Non-KRM designs removed** (4x2 returns 2025-05-06) | KRM only | |
| v7.6 / v7.6.1 | 2025-06-12 | PSB phase fix; 409.6 MHz refclk | KRM-S, KRM-D, 4x2 | 1.0.x (Oct 2025) |
| v7.7 | 2025-07-08 | Timekeeping and sampling clocks may be asynchronous; PTP timestamping (merged 2025-10-23) | KRM-S, KRM-D, 4x2 | |
| v7.8 | 2025-07-10 | Compile-time ADC/DAC snapshot sizes (64 kSample on 4x2) | KRM-S, KRM-D, 4x2 | |
| v7.9 | 2025-11-11 | VACC multitone, `psb_chanselect` inmap | KRM-S, KRM-D | 1.0.x (Nov 2025) |
| v7.9.1 / 7.9.2 | 2026-01-08 | Fast dump trigger fix; PEP 440 version strings (#88) | KRM-S, KRM-D | |
| v7.9.4 / 7.9.5 | 2026-01-28 / 02-03 | Accumulator burst mode (#81, #82) | KRM-S, KRM-D | |
| v7.9.5.1 / .2 | 2026-02-08 / 02-10 | AXI read/write clash fixes | KRM-S, KRM-D | |
| **v7.9.5.4** | 2026-02-12 (tag 03-10; 4x2 04-21) | Bad phase on the first few LOs (#101) | KRM-S, KRM-D, 4x2 | v1.1.0 – v1.4.0 |
| **v7.10** – v7.10.3 | 2026-05-14 → 06-10 (branch `7.10-timed-sync`, untagged) | Timed sync; dedicated software sync register (7.10.1); dual build (7.10.2, #108); #110 (7.10.3) | KRM-S, KRM-D, 4x2 | v1.5.0 |
| **v7.11** – v7.11.2 | 2026-06-18 → 07-01 (tag `v7.11.2`) | Multiple LO slots, tone switching on accumulation edges; LO control BRAM latency 2 | KRM-S, KRM-D, 4x2 | v1.6.0 – v1.7.9 |
| **v7.12** (`v7.12.0.2`) | 2026-07-09 | AXI4 128-bit BRAMs for accumulator and LO control; 200 MHz AXI clock; **new boot files** | KRM-S, KRM-D, 4x2 | v1.7.10 |
| **v7.12.1** | 2026-07-23 (tag = `4fb2812`) | | KRM-S, KRM-D, 4x2 | v1.8.0 → |

**control_sw must match the bitstream.** `souk_mkid_readout` refuses to
initialise when its version differs from the firmware's (`__fwversion__`), so a
firmware update always means updating `souk-firmware/software/control_sw` on the
board as well.

---

## OS image, kernel and boot files

The KRM boots from SD card slot 1 (BMC `Boot from: 5 - SD1`): `BOOT.BIN`,
`boot.scr` and `image.ub` sit on the card's FAT32 partition and the Ubuntu root
filesystem on partition 2. The kernel and U-Boot come from Petalinux, which has
been **2021.2** throughout: kernel `5.10.0-xilinx-v2021.2`, U-Boot 2021.01.
Kernel changes have been configuration only (built-in drivers, device tree),
never a version change.

| Date | What changed on the board | Needed by |
|---|---|---|
| ≤ 2024 – 2026-03 | CASPER KRM image: **Ubuntu 20.04**, Python 3.8 venv (`~/py3.8venv`); `tcpborphserver3` and `casperfpga` built/installed from `souk-firmware`; `krc-utils` configures the PL clocks at boot | software up to 1.0.1 (`python_requires >= 3.8`) |
| 2024-11-19 | Firmware repo: PTP TSU output enabled on GEM2 for KRM | later PTP work |
| 2025-03-11, 07-08 | Device tree, then the full Petalinux project spec, added to `souk-firmware/software/petalinux` (earlier boot files were not versioned there) | — |
| 2025-11-21 | Netboot option added (QSPI U-Boot → TFTP `image.ub` → read-only NFS root built from Ubuntu 24.04.3 base) | not used here; the lab boards boot from SD |
| 2026-03-20 → 03-25 | **SD image rebuilt: `krc4700-souk-u24`, Ubuntu 24.04, Python 3.12** (`/home/casper/py3.12-venv`) | **software ≥ 1.1.0** (`python_requires >= 3.10`; server paths assume the 3.12 venv) |
| 2026-04-22 | Device tree states the PTP TSU clock (250 MHz) explicitly | PTP/TSU strobe work (v1.2.0, v1.5.0). *Whether the deployed `image.ub` carries it is unconfirmed.* |
| 2026-05-05 | Software installs `ptp4l.service`, `timing-monitor.service`, `ptp4l.conf` and the chrony PHC drop-in (`souk-enable-timing`) | v1.2.0 → |
| 2026-07-09 (installed 2026-09-14/15) | **v7.12 boot files.** `BOOT.BIN`: `psu_init` sets PL0 to 200 MHz (`PL0_REF_CTRL` divisor `0xa → 0x5`) and HPM1 to 128 bit. `image.ub`: UIO over `0xA000_0000–0xC000_0000`, u-dma-buf 8 × 2 MiB (built-in driver patch). `top.bit` → `/lib/firmware/top.bit`. `boot.scr` byte-identical but on the install list. casperfpga `a89480c` (mmap window 32 → 512 MiB). control_sw `7.12.1`. `tcpborphserver3` unchanged. | firmware v7.12.x; `BOOT.BIN` is for speed (the 200 MHz clock), not correctness. Backup of the previous files: `/home/sam/souk/boot_backup_v712_20260915`. |
| 2026-09-22 | Optional board tuning from software: `souk-enable-tuning` (CPU isolation, real-time stream processes, per-core transmit queues, `souk-rt-kernel.service` at boot); udev gives end0's PHC the stable name `/dev/ptp-end0` | v1.8.1 → (for 2500 Hz dual-pipeline streaming) |

To check what a board is running: `uname -a` (kernel), `cat /etc/os-release`
(Ubuntu), `get_info('versions')` (software, control_sw, firmware), and
`profiling/board_state.py`, which reads `PL0_REF_CTRL` at `0xFF5E00C0` to tell
the v7.12 `BOOT.BIN` (divisor `0x05`) from the old one (`0x0A`).

---

## Platforms

There have been only two hardware platforms, and neither has changed. The
single-pipeline and dual-pipeline firmware (one or two parallel DSP chains) are
alternative designs compiled for the same platform, not different boards.

| Platform | Firmware builds | Software support |
|---|---|---|
| **KRM** ("krm4"): a Knowledge Resources KRM-4ZU47DR RFSoC module (KR module) on a KRC-4700 carrier (KR carrier) | v5.5 (2023-09) → v7.12.1 | Primary target since the first commit (Aug 2024) |
| **RFSoC4x2** | v3 → v5.6 (original platform); dropped 2025-01-26; rebuilt from 2025-05 (v7.6), then 7.9.5.4, v7.10, v7.11.1/2, v7.12.0.2, v7.12.1 | Tolerated rather than targeted: fixes in v1.6.1 (`krc-utils` checks, `needs_programming`). `souk-enable-tuning` and board management are KRM only. |

Firmware designs per platform:

| Design | Platform | Built | Software support |
|---|---|---|---|
| `souk_single_pipeline_krm` | KRM | v5.5 (2023-09) → v7.12.1 | Since the first commit (Aug 2024) |
| `souk_dual_pipeline_krm` | KRM | first v7.4 (2024-09-17); every release since v7.9 | From 2025-12-17 (released in v1.1.0); the standard SO:UK configuration |
| `souk_single_pipeline_rx_krm` (RX only) | KRM | v5.5 (2023-09-24) only | None |
| `souk_single_pipeline_4x2` | RFSoC4x2 | as the RFSoC4x2 row above | as above |

Clients run on Linux (Ubuntu, Fedora) and Windows with Python ≥ 3.10 (≥ 3.8
before v1.1.0). SO 3G streaming is unavailable on Windows (so3g optional from
v1.6.x).

---

## Peripheral hardware support

| Hardware | Since |
|---|---|
| RF mixerless module (I2C) and RUDAT USB attenuators | v1.1.0 |
| LNA bias board v1 (I2C) | v1.1.1 |
| Mixerless-module calibration | v1.2.0 |
| External 10 MHz reference / clock-source control | v1.1.0 |
| LabJack T-series DAC output (stream-to-dac) | v1.6.5 |
| LabJack U12 backend | v1.6.7 |
| LNA bias board **v2** (per-channel output enable), centralised `souk-lna-service` | v1.7.0 |
| ALSA sound-card DAC output | v1.7.8 |
| KRM board management controller over USB serial | v1.7.9 |
| RF module models `cardiff_dual` / `manchester_single` / `rx_prototype` | Unreleased |

---

## Keeping this page current

When a release changes what the board needs, add a row to the first table (and
the OS table if the image or boot files changed) in the same commit as the
CHANGELOG entry. When `souk-firmware` tags a version, add it to the firmware
timeline along with the platforms it was built for.
