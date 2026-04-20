# Clock Source Configuration

The RFSoC programmable logic (PL) clocks are driven by a clock tree consisting of an LMK04208 clock distribution chip and three LMX2594 PLLs. The LMK can be configured to use either an on-board 12.8 MHz oscillator (**internal**) or an external 10 MHz reference signal on the clk0 input (**external**).

This is a **board-level setting** — it affects both pipelines. If running dual-pipeline servers, both pipeline configs should specify the same `clock_source` value.

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
| `external` | 10 MHz reference on clk0 input | `lmk04208_in_10M_clk0_out_122M88.txt` | Locked to a lab reference or site clock |

Both configurations produce a 122.88 MHz output from the LMK, which feeds the three LMX2594 PLLs that generate the ADC and DAC sampling clocks.

The clock source is set by pointing a symlink at one of the two LMK config files and running `krc-utils init` to apply it. The readout tools can do this automatically via the client API or the config file.

---

## Checking Clock Status

### From the client

```python
# Current clock source selection ('internal' or 'external')
client.get_clock_source()

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
clock = client.get_info(['clock'])['clock']
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

If any chip reports `unlocked` (or `unlocked high`), the clock tree is not stable — see [Troubleshooting](#troubleshooting).

To check which source is currently selected:

```bash
ls -l /etc/krc-utils.d/clock.d/lmk04208.txt
```

The symlink target indicates the source:
- `lmk04208_in_12M8_out_122M88.txt` → **internal** (12.8 MHz)
- `lmk04208_in_10M_clk0_out_122M88.txt` → **external** (10 MHz)

---

## Setting the Clock Source

### Via the Client API

```python
# Switch to external 10 MHz reference
result = client.set_clock_source('external')

# Switch to internal 12.8 MHz oscillator
result = client.set_clock_source('internal')

# Verify
print(client.get_clock_source())
print(client.get_clock_status())
```

`set_clock_source()` updates the symlink, runs `krc-utils init`, and returns the PLL lock status. The server handles ownership so that files remain owned by the `casper` user.

### Via the Config File

The config file's `firmware.clock_source` field is applied whenever the config is pushed to the server:

```yaml
firmware:
  clock_source: "external"   # or "internal"
```

```python
client.config['firmware']['clock_source'] = 'external'
client.push_config()
```

The clock source is applied unconditionally on every config push. Since this is a board-level setting shared across pipelines, ensure both pipeline configs specify the same value — no coordination is performed between server instances.

> **Note:** In earlier versions, `clock_source` was under `rfsoc_host`. The server accepts both locations for backwards compatibility, but new configs should use `firmware.clock_source`.

### Manually on the RFSoC

If you need to set the clock source without the readout tools (e.g. during initial board setup):

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

### set_clock_source fails with FileNotFoundError

The target clock config file is missing from `/etc/krc-utils.d/clock.d/`. This could mean the `krc-utils` package was not fully installed on the board. Check that both LMK config files exist:

```bash
ls /etc/krc-utils.d/clock.d/lmk04208_in_*.txt
```

### Dual-pipeline clock conflicts

Clock source is a board-level resource. If pipeline 0's config says `internal` and pipeline 1's config says `external`, the last server to apply its config wins. Always use the same `clock_source` value in both pipeline configs to avoid unpredictable behaviour.
