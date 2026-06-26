# Clock Source Configuration

The RFSoC programmable logic (PL) clocks are driven by a clock tree consisting of an LMK04208 clock distribution chip and three LMX2594 PLLs. The LMK can be configured to use either an on-board 12.8 MHz oscillator (**internal**) or an external 10 MHz reference signal on the clk0 input (**external**).

This is a **board-level setting** — it affects both pipelines. If running dual-pipeline servers, both pipeline configs should specify the same `clock_source` value.

> **Changing the clocks requires deprogramming the firmware first, and any live state is lost.** Applying a clock source runs `krc-utils init`, which is only safe once the PL has been reset to zero tones — doing it on a loaded pipeline can hard-lock the board ([issue #14](https://github.com/sr-cdf/souk_readout_tools/issues/14)). Because the firmware image is shared, a clock change **resets both pipelines**: all tones, and live firmware settings are dropped and the pipelines must be re-initialised.

---

## Contents

- [Overview](#overview)
- [Checking Clock Status](#checking-clock-status)
- [Setting the Clock Source](#setting-the-clock-source)
  - [Via the Client API](#via-the-client-api)
  - [Via the Config File](#via-the-config-file)
  - [Manually on the RFSoC](#manually-on-the-rfsoc)
- [How It Works](#how-it-works)
- [Troubleshooting](#troubleshooting)

---

## Overview

| Source | Reference | LMK Config File | When to Use |
|--------|-----------|-----------------|-------------|
| `internal` | On-board 12.8 MHz oscillator | `lmk04208_in_12M8_out_122M88.txt` | Default — standalone operation, bench testing |
| `external` | 10 MHz reference on clk0 input | `lmk04208_in_10M_clk0_out_122M88.txt` | Locked to a lab reference, site clock or the SOUK clock distribution board|

Both configurations produce a 122.88 MHz output from the LMK, which feeds the three LMX2594 PLLs that generate the ADC and DAC sampling clocks.

The clock source is set by pointing a symlink at one of the two LMK config files and running `krc-utils init` to apply it. **Changing the clocks requires deprogramming first, and any live state is lost.** `krc-utils init` is only ever run once the PL has been reset to its base image (zero tones) — running it while the firmware is loaded with tones can collapse the PL power rail and hard-lock the board (see [issue #14](https://github.com/sr-cdf/souk_readout_tools/issues/14) and [Troubleshooting](#troubleshooting)). The readout tools therefore apply the configured source only as part of a reset-to-base → clock-init → reprogram cycle, never against a running pipeline. Because the firmware image is shared, this **resets both pipelines** — all tones, sweeps, and streaming state are dropped and the pipelines must be re-initialised.

> **Important:** Manual changes to the clock symlinks are not authoritative — the configured `firmware.clock_source` is applied at the next (re)program. Before any firmware access the server runs a **pure, PS-side clock check** (it never runs `krc-utils init` as a gate): if a PLL is unlocked, or the live source does not match the config, it refuses to touch the firmware and reports `reset_required`. A `hard_reset` then applies the change safely (deprogram → clock-init → reprogram), which **resets both pipelines**.

---

## Checking Clock Status

### From the client

```python
# Current clock source selection ('internal' or 'external')
client.get_clock_source()
# 'internal'

# PLL lock status of all clock chips
status = client.get_clock_status()
print(status)
# {'all_locked': True, 'chips': [
#     {'name': 'lmk04208.0', 'status': 'locked'},
#     {'name': 'lmx2594.0', 'status': 'locked'},
#     {'name': 'lmx2594.1', 'status': 'locked'},
#     {'name': 'lmx2594.2', 'status': 'locked'}
# ]}
```

Clock source and lock status are also available via `get_info()`:

```python
clock = client.get_info('clock')
print(clock['source'])       # 'internal' or 'external'
print(clock['all_locked'])   # True if all PLLs are locked
print(clock['chips'])        # per-chip lock status
```

### Manually on the RFSoC

```bash
cd /home/casper
./krc-utils/krc-utils status
```

All chips should report `locked`:

```
[lmk04208.0] status: locked
[ lmx2594.0] status: locked
[ lmx2594.1] status: locked
[ lmx2594.2] status: locked
```

If any chip reports `unlocked` (or `unlocked high` or `unlocked low`), the clock tree is not stable — see [Troubleshooting](#troubleshooting).

To check which source is currently selected:

```bash
ls -l /etc/krc-utils.d/clock.d/lmk04208.txt
```

The symlink target indicates the source:
- `lmk04208_in_12M8_out_122M88.txt` → **internal** (12.8 MHz)
- `lmk04208_in_10M_clk0_out_122M88.txt` → **external** (10 MHz)

---

## Setting the Clock Source

### Via the Config File (recommended)

The config file's `firmware.clock_source` field records the desired clock source:

```yaml
firmware:
  clock_source: "external"   # or "internal"
```

```python
client.config['firmware']['clock_source'] = 'external'
client.push_config()
```

Before any firmware access the server runs a pure, PS-side clock check (it never runs `krc-utils init` as a gate). If all PLLs are locked and the live source already matches the config, it proceeds normally. If the live source **differs** from `firmware.clock_source`, the push is rejected with a message telling you to apply it with a `hard_reset` — because changing the source requires running `krc-utils init`, which is only safe on a blank PL. A `hard_reset` then applies it as part of a deprogram → clock-init → reprogram cycle, which **resets both pipelines**.

Since this is a board-level setting shared across pipelines, ensure both pipeline configs specify the same value — no coordination is performed between server instances.

> **Note:** In earlier versions, `clock_source` was under `rfsoc_host`. The server accepts both locations for backwards compatibility, but new configs should use `firmware.clock_source`.

### Programmatically (disruptive — resets both pipelines)

There is a private client method `_set_clock_source()` for forcing a source change at runtime:

```python
client._set_clock_source('external')   # or 'internal'

# Verify
print(client.get_clock_source())
print(client.get_clock_status())
```

It is deliberately private: it runs the full deprogram → clock-init → reprogram cycle on the shared dual-pipeline firmware, so **all tones are dropped and both pipelines are reset and must be re-initialised**. Prefer the config-file route above unless you specifically need to switch the reference clock on a live server.

### Manually on the RFSoC

If you need to set the clock source without the readout tools (e.g. during initial board setup or in case of failure):

> **Warning:** This manual symlink selection does not override future controls afterwards. A `hard_reset` (or `_set_clock_source()`) that applies `firmware.clock_source` will rewrite the symlink and replace any manual setting.
>
> **Never run `krc-utils init` while the firmware is loaded.** Do this only with the PL deprogrammed (e.g. `r.fpga.host.deprogram()`, or `echo tcpborphserver.bin > /sys/class/fpga_manager/fpga0/firmware`). See [Troubleshooting](#troubleshooting).

**Select external 10 MHz reference:**

```bash
cd /etc/krc-utils.d/clock.d
unlink lmk04208.txt
ln -s lmk04208_in_10M_clk0_out_122M88.txt lmk04208.txt
```

**Select internal 12.8 MHz oscillator:**

```bash
cd /etc/krc-utils.d/clock.d
unlink lmk04208.txt
ln -s lmk04208_in_12M8_out_122M88.txt lmk04208.txt
```

**Apply the settings:**

```bash
cd /home/casper
./krc-utils/krc-utils init
```

Expected output when all clocks lock successfully:

```
[lmk04208.0] Loading: /etc/krc-utils.d/clock.d/lmk04208.txt
[ lmx2594.0] Loading: /etc/krc-utils.d/clock.d/lmx2594.txt
[ lmx2594.1] Loading: /etc/krc-utils.d/clock.d/lmx2594.txt
[ lmx2594.2] Loading: /etc/krc-utils.d/clock.d/lmx2594.txt
[lmk04208.0] status: locked
[ lmx2594.0] status: locked
[ lmx2594.1] status: locked
[ lmx2594.2] status: locked
```

If running as root, ensure the symlink remains owned by the `casper` user:

```bash
chown -h casper:casper /etc/krc-utils.d/clock.d/lmk04208.txt
```

---

## How It Works

The clock configuration files live in `/etc/krc-utils.d/clock.d/`:

```
/etc/krc-utils.d/clock.d/
├── lmk04208_in_10M_clk0_out_122M88.txt     # external 10 MHz config
├── lmk04208_in_12M8_out_122M88.tcs
├── lmk04208_in_12M8_out_122M88.txt          # internal 12.8 MHz config
├── lmk04208.txt -> lmk04208_in_12M8_out_122M88.txt   # active config (symlink)
├── lmx2594_in_122M88_out_0409M6.tcs
├── lmx2594_in_122M88_out_0409M6.txt
├── lmx2594_in_122M88_out_4096M.tcs
├── lmx2594_in_122M88_out_4096M.txt
└── lmx2594.txt -> ./lmx2594_in_122M88_out_0409M6.txt  # active LMX config (symlink)
```

The `lmk04208.txt` symlink determines which reference clock the LMK uses. The `krc-utils init` command reads all `.txt` symlinks, programs the corresponding register maps into the clock chips via I2C/SPI, and reports the resulting lock status.

The `krc-utils` service also runs automatically on boot, so the clock source selection persists across power cycles (it is determined by whatever the symlink points to at boot time).

The LMX2594 config is independent of the clock source — the LMX PLLs always take 122.88 MHz from the LMK output regardless of whether that was derived from 10 MHz or 12.8 MHz.

---

## Troubleshooting

### Board hard-locks after a clock init (issue #14)

Running `krc-utils init` while the PL is programmed and loaded with many tones can collapse the PL power rail (`vccint`/`vccaux`/`vccbram` drop to 0 V): the fabric dies and the next AXI transaction hangs the PS forever, requiring a power cycle. Once collapsed there is **no software recovery** — even the FPGA-manager write fails and `krc-utils init` times out.

The readout tools prevent this by only ever running `krc-utils init` once the PL has been **reset to its base image (zero tones, minimal current)** — a PL image must stay in place for clock forwarding to work, so this is a reset-to-base, not a true blank. The recovery sequence is always **reset to base → check clocks → init clocks only if necessary → reprogram the souk design**. On detecting an unlocked PLL during operation the server drops to `server` level (no firmware interfaces) and reports `reset_required`; recover with a `hard_reset` (which resets both pipelines). If the **reference clock (LMK)** is unlocked the server additionally blocks all firmware reads/writes, since touching AXI with a dead fabric clock would hang the PS — a power cycle is usually required in that case.

If you ever need to reset the PL by hand before re-initialising the clocks:

```bash
# preferred (via casperfpga): r.fpga.host.deprogram()
# fallback (PS-side): loads the base image via the FPGA manager, avoiding the
# AXI register traffic of the casperfpga path (which hangs if the fabric is
# unresponsive). A reference clock must still be present for the image to run.
sudo sh -c "echo tcpborphserver.bin > /sys/class/fpga_manager/fpga0/firmware"
```

### Clocks report unlocked

```
[lmk04208.0] status: unlocked
[ lmx2594.0] status: unlocked high
[ lmx2594.1] status: unlocked high
[ lmx2594.2] status: unlocked high
```

**If using external clock:**
- Verify the 10 MHz reference is connected and powered
- Check signal level (the LMK clk0 input expects a standard 10 MHz reference)
- Try re-running `krc-utils init` — the PLL may need a second attempt to lock

**If using internal clock:**
- Try re-running `krc-utils init`
- If the problem persists, check the board hardware

**General:**
- Verify the symlink points to a valid config file: `ls -l /etc/krc-utils.d/clock.d/lmk04208.txt`
- Check that `krc-utils` can be found: `ls -l /home/casper/krc-utils/krc-utils`

### Clock source reads as 'unknown'

This means the `lmk04208.txt` symlink target doesn't match either of the expected filenames. Check the symlink:

```bash
ls -l /etc/krc-utils.d/clock.d/lmk04208.txt
```

Recreate it pointing to one of the two known config files.

### Clock source change fails with FileNotFoundError

The target clock config file is missing from `/etc/krc-utils.d/clock.d/`. This could mean the `krc-utils` package was not fully installed on the board. Check that both LMK config files exist:

```bash
ls /etc/krc-utils.d/clock.d/lmk04208_in_*.txt
```

### Dual-pipeline clock conflicts

Clock source is a board-level resource. If pipeline 0's config says `internal` and pipeline 1's config says `external`, the last server to apply its config wins. Always use the same `clock_source` value in both pipeline configs to avoid unpredictable behaviour.
