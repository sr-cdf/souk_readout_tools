# Board Management System

Each KRM-4ZU47DR RFSoC carrier board carries a **board management controller
(BMC)** owning the 12 V input, power sequencing of every rail, reset and PROG
lines, boot-mode selection, the VTT termination supply, the current monitor,
and the clock-synth configuration.

It is reached over **USB serial from the control PC**, not from the RFSoC's
own Linux, and works independently of whether Linux or the firmware is alive.

> ### ⚠️ Read this before typing anything
>
> No confirmation, no undo, and **`krm4:f` writes the current settings to
> flash permanently**.
>
> Only the queries (`2` `3` `4` `i` `s` `b` `qs` `?`) and the power commands
> (`0` `1`) are safe. Everything else — boot (`bx`), clock (`ysx` `ycx` `yl`
> `yr`), VTT (`vttbst*`), current monitor (`q*`), autostart (`a0/a1`),
> POR/PROG (`r0`-`r3`) — risks a board that will not boot, a PL with no clock
> reference, or a rail that trips under load. See
> [Command reference](#command-reference).
>
> The tooling here can issue **only** the safe subset
> ([Safety design](#safety-design)).

---

## Contents

- [Physical interface](#physical-interface)
- [Identifying boards](#identifying-boards)
- [Permissions and udev aliases](#permissions-and-udev-aliases)
  - [Windows and macOS](#windows-and-macos)
- [Connecting by hand](#connecting-by-hand)
- [Command reference](#command-reference)
- [Board status (`krm4:i`)](#board-status-krm4i)
- [Boot modes (`krm4:b`)](#boot-modes-krm4b)
- [Current monitor (`krm4:qs`)](#current-monitor-krm4qs)
- [Settings (`krm4:s`)](#settings-krm4s)
- [Board identity](#board-identity)
- [Powering the board](#powering-the-board)
- [Tooling](#tooling)
- [Troubleshooting](#troubleshooting)
- [What a PL rail collapse looks like](#what-a-pl-rail-collapse-looks-like)

---

## Physical interface

One USB cable reaches a **Silicon Labs CP2105 dual UART bridge**
(`10c4:ea70`): two UARTs on two USB interfaces sharing one serial number.

| USB interface | CP2105 port | Function |
|---------------|-------------|----------|
| `00` | Enhanced Com Port (SCI) | **ARM Linux console** — login shell, U-Boot and kernel boot messages |
| `01` | Standard Com Port (ECI) | **Board management controller** — `krm4:` commands |

**115200 baud, 8N1, no flow control**, both interfaces.

> **ttyUSB numbering moves; interface numbers do not.** On a clean machine,
> `ttyUSB0` = console and `ttyUSB1` = BMC, but another board or any other USB
> serial device shifts that. Never hard-code `ttyUSB1` — address boards by
> serial number or udev alias.
>
> Which interface is the BMC is a board-design choice (`01` on SOUK boards),
> so `souk-find-boards` confirms it by probing, falling back to this table
> only when a port cannot be probed and labelling the result `assumed`.

## Identifying boards

| | Example | Read by | Identifies |
|---|---------|---------|------------|
| **USB serial** | `01449C3F` | USB subsystem, no port opened | the CP2105 bridge |
| **Board serial** | `24NPv3ZnDm` | `krm4:3`, over the BMC link | the board itself |

udev can only match the USB serial. Both ttyUSB nodes of a board report the
same one, which is how they are grouped back into one board.

```bash
python3 -m serial.tools.list_ports -v
```

```text
/dev/ttyUSB0
    desc: CP2105 Dual USB to UART Bridge Controller - Enhanced Com Port
    hwid: USB VID:PID=10C4:EA70 SER=01449C3F LOCATION=1-4:1.0
/dev/ttyUSB1
    desc: CP2105 Dual USB to UART Bridge Controller - Standard Com Port
    hwid: USB VID:PID=10C4:EA70 SER=01449C3F LOCATION=1-4:1.1
```

The `LOCATION` suffix (`:1.0`, `:1.1`) is the USB interface number. Also via
`udevadm info -a -n /dev/ttyUSB0 | grep -m1 '{serial}'` and `udevadm info -q
property -n /dev/ttyUSB0 | grep ID_USB_INTERFACE_NUM`.

`souk-find-boards` reports both serials together:

```text
Found 1 USB serial bridge(s):

  USB serial : 01449C3F
  Board      : KRM-4ZU47DRSE1XCA-F1-SP
  Board serial: 24NPv3ZnDm
  Bridge     : CP2105 dual UART bridge (10c4:ea70)
    interface 00: /dev/ttyUSB0   console  Enhanced Com Port
    interface 01: /dev/ttyUSB1   bmc      Standard Com Port
```

`--board` then accepts any of three forms, case-insensitively; ambiguous
matches are refused:

```bash
souk-board-status --board 01449C3F      # by bridge
souk-board-status --board 24NPv3ZnDm    # by board  (needs probing)
souk-board-status --board /dev/ttyUSB1  # by device path
```

`--all-ports` lists non-SOUK bridges too. `--no-probe` never opens a port, so
it is safe while a console session is live; roles then show `(assumed)`.

> **Prefer the board serial for anything that changes board state**, and
> `souk-board-power --expect-serial`, which aborts unless the BMC reports that
> exact serial.

## Permissions and udev aliases

Add your user to `dialout` to avoid needing `sudo`:

```bash
sudo usermod -aG dialout $USER
newgrp dialout                      # or log out and back in
```

A udev rule gives every port a permanent name:

```bash
souk-find-boards --udev > 99-souk-rfsoc-serial.rules
sudo cp 99-souk-rfsoc-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload && sudo udevadm trigger
```

```
# CP2105 dual UART bridge (01449C3F)
#   board serial 24NPv3ZnDm -- KRM-4ZU47DRSE1XCA-F1-SP
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ATTRS{serial}=="01449C3F", ENV{ID_USB_INTERFACE_NUM}=="00", SYMLINK+="souk/24npv3zndm-console", GROUP="dialout", MODE="0660"
SUBSYSTEM=="tty", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea70", ATTRS{serial}=="01449C3F", ENV{ID_USB_INTERFACE_NUM}=="01", SYMLINK+="souk/24npv3zndm-bmc",     GROUP="dialout", MODE="0660"
```

giving `/dev/souk/<name>-console` and `/dev/souk/<name>-bmc`. Rules key on the
bridge serial; symlinks are named after the board serial. Rename to something
friendlier (`souk/rfsoc01-bmc`) if preferred.

> **Swapping a board while the cabling stays put leaves the rule firing under
> the wrong name.** Regenerate after any hardware change.

Commented template:
`src/souk_readout_tools/data/udev/99-souk-rfsoc-serial.rules`.

### Windows and macOS

Everything above — the `dialout` group, udev aliases, `/dev/tty*` names — is
Linux. The tooling itself is not: discovery goes through pyserial's
cross-platform `list_ports`, and the BMC protocol is identical everywhere.
What changes is only how a port is named and how a refused open is fixed.

| | Linux | Windows | macOS |
|---|---|---|---|
| port name | `/dev/ttyUSB1` | `COM7` (`\\.\COM10` and above) | `/dev/cu.usbserial-*` |
| driver | in-tree `cp210x` | Silicon Labs CP210x VCP (usually automatic on Windows 10/11) | Silicon Labs CP210x VCP |
| stable alias | udev rule (above) | none needed — select by USB serial | none needed — select by USB serial |
| access refused | join `dialout` | close the app holding the port | usually none |

**Select boards by USB serial, not by port name** — it is stable across
replugs on every platform and is the only selector that needs no aliasing:

```bash
souk-board-status --board 01449C3F     # USB serial: portable, stable
souk-board-status --board COM7         # Windows device name
```

`souk-find-boards --udev` generates Linux rules and refuses to run on Windows,
pointing at USB-serial selection instead. One further Linux-only detail: the
USB *interface* number is read from sysfs, which is what tells console from BMC
without opening anything. Elsewhere that lookup returns nothing and the role
probe (on by default) identifies the BMC port instead — so on Windows and macOS
avoid `--no-probe`, which would leave the roles resolved by plug order alone.

## Connecting by hand

```bash
picocom -b 115200 /dev/souk/rfsoc01-bmc     # or /dev/ttyUSB1
```

Exit with `Ctrl-A Ctrl-X`. Syntax is `target:command` plus a **carriage
return**, e.g. `krm4:i`.

- Lower case only.
- Responses print straight back, with no trailing prompt.
- Invalid commands give `unknown command` **or nothing at all** — silence
  means neither accepted nor rejected. Confirm with `krm4:i` or `krm4:s`.
- Echo is on by default (`e1`).

> A port takes one program at a time. pyserial `flock`s on open, so the tools
> refuse a port picocom holds, reporting *"already open"*. A `sudo` session
> holds that lock as root.

## Command reference

`krm4:?`:

```text
Guide:
Command syntax:
"target:command"
e.x."krm4:a0" to disable autostart

Power off                0
Power on                 1
Show type                2
Show serial              3
Show fw revision         4
Show board status        i
Show current settings    s
Save settings to flash   f
Show boot options        b
Set boot option          bx  (x=0..f)
Set clock option         ysx (x=0..7)
Print clock option       ypx (x=0..7)
Clear clock config       ycx (x=4..7)
Reload clock synth       yr
Upload clock config      yl
release POR              r0
assert POR               r1
release PROG             r2
assert PROG              r3

Autostart:
enable     a1
disable    a0

Echo:
enable     e1
disable    e0

Current monitor:
enable                   q1
disable                  q0
enable autostart         qa1
disable autostart        qa0
select monitored rail    qcsxy (x=1,2 y=0..7)
filter coeff short       qpax  (x=0..f
filter coeff long        qpbx  (x=0..f)
set limit value mA       qtvxxxx  (xxxx=0000..9999)
set wait time ms         qrdxxxx  (xxxx=0000..ffff)
show current settings    qs

VTT boost:
disable                  vttbst0
enable                   vttbst1

Target and commands are lower-case followed by carriage-return (\r)
```

| Risk | Commands | Consequence |
|------|----------|-------------|
| **Safe** | `2` `3` `4` `i` `s` `b` `ypx` `qs` `?` | Queries only. |
| **Operational** | `0` `1` | Power off / on; cuts rails immediately — see [Powering the board](#powering-the-board). |
| **Dangerous** | `bx` | Wrong boot source ⇒ board stops booting, recoverable only over this serial link. |
| | `ysx` `ycx` `yr` `yl` | Can leave the PL with no usable clock reference; a collapsed PL rail may not recover ([clock_source.md](clock_source.md)). |
| | `vttbst0/1` `q*` | Wrong current limit trips the board off under load; wrong VTT stresses the memory interface. |
| | `a0/a1` `r0`-`r3` | Autostart, and holding the processor or FPGA in reset. `r3` looks exactly like a dead board. |
| **Permanent** | `f` | Writes current settings to flash — everything above becomes the power-on default. |

> `krm4:b` is a query, `krm4:b0` silently rewrites the boot source. Same trap
> for `yp` vs `ys`.

## Board status (`krm4:i`)

```text
krm4:i

Status:
BMC temp:       59 C
BMC pwr:        3.2

Vin:    11.9

EXT EN:   high
EXT POK:  high
RST_OUT:  low
CFG_OK:   low

PS_RST:   released (high)
PL_DONE:  yes (high)

Start EN:    no
Run EN:      yes
systemstate: 8
powerstate:  0
P0:   0xf7
P1:   0x5d
P2:   0x3f
P3:   0x0d
```

| Field | Meaning | Normal value |
|-------|---------|--------------|
| `BMC temp` | BMC die temperature | 41 °C idle to 62 °C fully loaded, **rack open to the room**. The only field that tracks PL load. |
| `BMC pwr` | BMC's own supply rail | `3.2` in every reading, across all power states, 41-62 °C and the full load range. Read as volts; the firmware does not state units. |
| `Vin` | Board 12 V input | 11.8-12.1 V; sags only ~0.2 V from idle PL to fully loaded |
| `EXT EN` | External power enable | `high` powered, `low` off |
| `EXT POK` | External power good | Follows `EXT EN`. **`EXT EN` high with `EXT POK` low is a rail fault.** |
| `RST_OUT` | Board reset output | `low` running, `high` off |
| `CFG_OK` | Configuration OK | `high` when off and just after power-on; `low` once the base PL image loads. Not a reliable health flag alone. |
| `PS_RST` | Processor system reset | `released (high)` running, `reset (low)` off |
| `PL_DONE` | FPGA configuration done | `yes (high)` once *any* bitstream is loaded, including the base image Linux loads at boot — so this does **not** mean the readout firmware is running. |
| `Start EN` / `Run EN` | Sequencer enables | `no` / `yes` in every state observed |
| `systemstate` | Sequencer state machine | `8` running, `0` off |
| `powerstate` | Power fault state | **`0`.** Non-zero treated as a fault — an inference; see below. |
| `P0`-`P3` | Raw GPIO port dumps | Vendor-internal; see below |

### Reference values

One healthy board. Every column is a real reading.

| Field | Off | On, PL unconfigured | On, base PL image | On, dual-pipeline fw + 1000 tones/pipeline |
|-------|-----|---------------------|-------------------|--------------------------------------------|
| `EXT EN` / `EXT POK` | low / low | high / high | high / high | high / high |
| `RST_OUT` | high | low | low | low |
| `CFG_OK` | high | high | low | low |
| `PS_RST` | reset (low) | released (high) | released (high) | released (high) |
| `PL_DONE` | no (low) | no (low) | yes (high) | yes (high) |
| `systemstate` | 0 | 8 | 8 | 8 |
| `powerstate` | 0 | 0 | 0 | 0 |
| `BMC temp` | 49 °C | 48 °C | 41 °C | 48 °C programmed → 62 °C with tones |
| `Vin` | 12.1 V | 12.0 V | 12.0 V | 11.8-11.9 V |
| `P0` | `0xf3` | `0xf7` | `0xf7` | `0xf7` |
| `P1` | `0x00` | `0x5c` | `0x5d` | `0x5d` |
| `P2` | `0x53` | `0x77` | `0x3f` | `0x3f` |
| `P3` | `0x15` | `0x0e` | `0x0d` | `0x0d` or `0x0e` |

*Off* and *PL unconfigured* are single observations, the latter taken seconds
after `krm4:1`, before the bootloader started. *Base PL image* is reproduced
across sessions; *dual-pipeline* is two captures.

**Load changes temperature and nothing else.** Bare PL to fully loaded
dual-pipeline is **+21 °C** (41 → 48 → 62 °C) against 0.1-0.2 V of `Vin` sag,
barely above the one-decimal resolution. Every flag and GPIO port is identical
between the base image and full load. 62 °C is normal, not hot.

**`P0`-`P3`** bit assignments are not published. `P0`-`P2` are reproducible
and track PL *configuration* state (`0x5d` / `0x3f` configured, `0x5c` /
`0x77` not); `P1` is `0x00` powered down. `P3` is **not** reliable — both
`0x0d` and `0x0e` seen on a configured, loaded board. Corroboration only,
never a health check.

> **"PL unconfigured" means no bitstream *and* Linux not yet booted.** Those
> come apart in the failure case: a collapsed fabric has Linux **running**
> with the PL dead and reads identically bar temperature. See
> [What a PL rail collapse looks like](#what-a-pl-rail-collapse-looks-like).

**Not yet characterised:** `BMC temp` in a *closed* rack under load (all
figures are open-air; retune `TEMP_WARN_C` if an enclosure pushes a healthy
board past 75 °C), and `powerstate` other than `0` — never observed non-zero,
including under full load and through a rail collapse.

Worth capturing when firmware is first brought up:

```bash
souk-board-status --board 01449C3F --raw > bmc-before.txt
# ...program and load the firmware, then...
souk-board-status --board 01449C3F --raw > bmc-loaded.txt
diff bmc-before.txt bmc-loaded.txt
```

## Boot modes (`krm4:b`)

A bare `b` carries no selection, so the BMC errors and lists what it accepts.
**It changes nothing** — the reply opening with *invalid* is expected.

```text
krm4:b

boot mode invalid option
possible selections are:
0 - JTAG
1 - QSPI24
2 - QSPI32
5 - SD1
6 - eMMC18
7 - USB0
8 - PJTAG0
E - SD1-LS
```

`5 - SD1` is the SOUK standard. Codes are hex and only these eight are valid,
despite the setter's advertised `x=0..f`; commands are lower case, so SD1-LS
is `be`.

> **This lists the modes available, not the one in use** — that is `Boot from`
> in [`krm4:s`](#settings-krm4s). Setting one is
> [dangerous](#command-reference).

## Current monitor (`krm4:qs`)

```text
krm4:qs
current monitor settings
slave-channel 11
filter coeff short 1
filter coeff long 8
current limit 2000mA
wait time 65535ms
autostart no
```

`slave-channel 11` is the monitored rail as `qcsxy` (slave 1, channel 1). The
reply has **no colons** — the value is the last token on each line.

> **This `autostart` is not the board's `Autostart`.** `krm4:s` reports
> whether the *board* self-powers; this is whether the *current monitor*
> starts automatically.

From one board, so `souk-board-status` does not check these.

## Settings (`krm4:s`)

```text
krm4:s

Settings:
Autostart:     enabled
VTT_DAC:       2,5V
Boot from:     5 - SD1
Selected clock: 0
```

| Setting | Standard | Meaning |
|---------|----------|---------|
| `Autostart` | `enabled` | Board powers up on its own when 12 V is applied, rather than waiting for `krm4:1`. |
| `VTT_DAC` | `2,5V` | Termination supply setpoint (decimal comma). |
| `Boot from` | `5 - SD1` | Boot source — SD card slot 1. |
| `Selected clock` | `0` | Active BMC clock configuration slot. |

**These are the values a working SOUK board should report.** If any differ,
investigate before powering on rather than guessing at a fix. Live values;
they survive a power cycle only after `krm4:f`.

## Board identity

```text
krm4:2
KRM-4ZU47DRSE1XCA-F1-SP        # board type

krm4:3
24NPv3ZnDm                     # board serial (BMC-reported)

krm4:4
Mstr 200.000.000               # firmware revisions: master controller
Slv1 300.000.000               # ...and the two slave controllers
Slv2 300.000.000               #    ('slave 1'/'slave 2' in power-up)
```

The `krm4:3` serial is **not** the CP2105 USB serial — see
[Identifying boards](#identifying-boards). Record both per board.

## Powering the board

`krm4:0` cuts the rails, `krm4:1` runs the power-up sequence. Neither is
persisted.

| Step | Does | Does *not* do |
|------|------|---------------|
| `sudo poweroff` on the ARM | Stops services, syncs and unmounts the SD card, halts the CPU | **Touch board power.** Rails stay up, `krm4:i` still reports `EXT EN: high` and `systemstate: 8`. |
| `krm4:0` on the BMC | Removes board power | Give Linux any warning |

> **Halt the OS first, then cut power.** `krm4:0` gives Linux no notice and
> does not sync the filesystem. The halt leaves the rails up, so `krm4:0` is
> still needed afterwards — a halted board is not a powered-down board.
>
> Cutting power drops the firmware, both pipelines, and any live tones or
> streaming ([dual_pipeline.md](dual_pipeline.md)).

`krm4:i` and `souk-board-status` report the **rails, not the OS** — a halted
board and a running one both show `systemstate: 8`. Watch the console
interface or the network to confirm the OS has halted.

### Power off

```text
krm4:0
STOP
init slave 1...OK
not boosted
init slave 2...OK
not boosted
FEC off
INT off
PWR down
```

`PWR down` ends it. `krm4:i` should then show `EXT EN: low`, `systemstate: 0`,
`PS_RST: reset (low)`, `PL_DONE: no (low)`.

### Power on

```text
krm4:1
init slave 1...OK
not boosted
init slave 2...OK
not boosted
Start s2 c8...set...get...ign...OK
Start s0 c0...up
...
Start s1 c4...set...get...ign...OK
Clk0...entering config routine
ready state
reg map written
active state
OK
```

Each `Start sN cM` is one rail; `Clk0...` is the clock synth; the final `OK`
means it completed. **Any rail line not ending in `OK`/`up`, or a sequence
that stops early, is a hardware fault — do not retry blindly.**

`krm4:i` should then show `EXT EN: high`, `EXT POK: high`, `systemstate: 8`,
`powerstate: 0`, `PL_DONE: no`. Watch the *console* interface for U-Boot and
the Ubuntu boot messages.

### Timing

| Event | Duration |
|-------|----------|
| `krm4:1` transcript, ending `OK` | ~1 s, streaming consecutively |
| `krm4:1` → **login prompt on the console** | **~50 s** |
| `krm4:0` → `PWR down` acknowledgement | immediate |
| `krm4:0` → rails actually dropping | a few seconds — the board fan spinning down is the cue |

> **`souk-board-power on` returning means the rails are up, not the board.**
> Linux needs ~50 s more to reach a login prompt on the *console* interface,
> while the BMC reports `systemstate: 8` throughout.
>
> **Do not power straight back on after a power-off** — the acknowledgement is
> instant but the rails take seconds to discharge. Allow ten, as
> `souk-board-power cycle` does by default (`--off-delay 10`).

The tools read a power sequence until 1.5 s of silence or the closing
`OK` / `PWR down` line.

## Tooling

Three CLI commands wrapping the safe subset, installed with the client
(control-PC) install; they need `pyserial`.

| Command | Does | Changes anything? |
|---------|------|-------------------|
| `souk-find-boards` | Enumerate CP2105 bridges, identify the BMC interface, optionally emit udev rules | No |
| `souk-board-status` | Read identity, status and settings; check them and report anything unusual | No |
| `souk-board-power` | Report or change power state (`status`, `on`, `off`, `cycle`) | Power only |

`souk-board-status` exit codes: `0` all clear, `1` warnings, `2` errors or no
board.

### Running without installing

`board_management.py` imports only stdlib plus a lazy `pyserial`, so it runs
from a checkout or as a single copied file, with subcommands `find-boards`,
`status`, `power`:

```bash
bmc="python3 /path/to/souk_readout_tools/src/souk_readout_tools/board_management.py"
$bmc find-boards
$bmc status --board 01449C3F

# or, installed:  python3 -m souk_readout_tools.board_management find-boards
# or, copied:     scp .../board_management.py host: && python3 board_management.py find-boards
```

Below, `souk-find-boards` / `souk-board-status` / `souk-board-power` map to
`$bmc find-boards` / `$bmc status` / `$bmc power`.

### Safety design

These commands **cannot** do the dangerous things above, even by accident:

- `BoardManagementController._send` matches an allow-list by **exact string
  equality** — no pattern matching, no argument concatenation, so no query can
  become its setter twin. `f`, `bx`, `ysx`, `ycx`, `yl`, `yr`, `q*`,
  `vttbst*`, `a0/a1` and `r0`-`r3` raise rather than transmit.
- `power_on()` / `power_off()` are the only writers, each requiring
  `confirm=True`.
- `souk-board-power on|off|cycle` **refuses to auto-select a board** —
  `--board` is mandatory even with one attached; `--expect-serial` adds a hard
  check against the BMC-reported serial.
- Changes prompt for confirmation, and refuse outright on a non-interactive
  stdin without `--yes`.
- Ports open with DTR and RTS deasserted, and **exclusively**, so an attached
  picocom session is never fought over.
- Probing sends a bare carriage return first, and the read-only `krm4:2` only
  if the port looks nothing like a console. `--no-probe` skips it.

### Examples

```bash
souk-find-boards                                   # what is attached
souk-board-status                                  # auto-selects iff one board
souk-board-status --board 24NPv3ZnDm               # by board serial
souk-board-status --board /dev/souk/rfsoc01-bmc    # by alias
souk-board-status --all --json                     # every board, machine-readable
souk-board-status --board 01449C3F --raw           # + unparsed replies
```

```text
Board        : KRM-4ZU47DRSE1XCA-F1-SP
BMC serial   : 24NPv3ZnDm
USB serial   : 01449C3F
Device       : /dev/ttyUSB1
Power state  : ON

  BMC temp   : 41.0 C
  BMC supply : 3.2 V
  Vin        : 12.0 V
  EXT EN     : high
  EXT POK    : high
  RST_OUT    : low
  CFG_OK     : low
  PS_RST     : released
  PL_DONE    : yes
  systemstate: 8
  powerstate : 0
  GPIO ports : P0=0xf7 P1=0x5d P2=0x3f P3=0x0d

  Autostart  : enabled
  VTT_DAC    : 2,5V
  Boot from  : 5 - SD1
  Clock      : 0

  All checks OK.
```

`--raw` appends the unparsed `krm4:i`, `krm4:s`, `krm4:b` and `krm4:qs`
replies. `--json` adds the parsed boot modes and current-monitor settings.

```bash
souk-board-power status --board 01449C3F           # read-only

ssh casper@rfsoc01 sudo poweroff                   # halt the OS first
souk-board-power off --board 01449C3F              # then remove power

souk-board-power cycle --board 01449C3F --expect-serial 24NPv3ZnDm --yes
```

> `off` and `cycle` cannot shut the OS down for you. The unattended `cycle`
> form is for a board already wedged or never booted — do not wire it into
> automation that may catch a healthy running system.

### Checks applied

| Check | Level |
|-------|-------|
| `Vin` outside 10.8-13.2 V | error |
| `Vin` outside 11.4-12.6 V | warn |
| BMC supply outside 3.0-3.6 V | warn |
| BMC temperature ≥ 85 °C | error |
| BMC temperature ≥ 75 °C | warn |
| `EXT EN` high while `EXT POK` low | error |
| `powerstate` non-zero | error |
| Any status flag disagreeing with the expected on/off pattern | warn |
| Power state indeterminate (possibly mid-sequence) | warn |
| Any setting differing from the [standard values](#settings-krm4s) | warn |
| `PL_DONE` low on a powered board | info |

Thresholds are advisory, **not vendor specifications** — none is published for
this temperature sensor. The temperature pair sits above the highest observed
healthy reading (62 °C, open rack, fully loaded) with headroom for an
enclosure. Constants at the top of
`src/souk_readout_tools/board_management.py` (`VIN_WARN`, `TEMP_WARN_C`, …).

### From Python

```python
from souk_readout_tools.board_management import (
    BoardManagementController, find_boards, resolve_board, check_status)

device, usb_serial = resolve_board('24NPv3ZnDm')
with BoardManagementController(device, usb_serial) as bmc:
    print(bmc.identify())
    status = bmc.get_status()
    print(bmc.power_state(), check_status(status, bmc.get_settings()))
    # bmc.power_off(confirm=True)   # the only writer, and it must be asked
```

## Troubleshooting

| Symptom | Cause and fix |
|---------|---------------|
| `could not open /dev/ttyUSB1` | picocom/screen holds it, or you are not in `dialout` — see [Permissions](#permissions-and-udev-aliases). |
| No response to `krm4:i`, or `unknown command` on everything | Probably the ARM console, not the BMC. Try the other node or run `souk-find-boards`; a login prompt on Enter means the console. |
| Garbled output | Wrong baud rate. It is 115200. |
| Powered (`systemstate: 8`) but nothing boots | Check `Boot from` against the [standard values](#settings-krm4s), then watch the console for U-Boot. Usually a boot-source or SD card problem. |
| `EXT EN: high` but `EXT POK: low`, or `powerstate` non-zero | A rail did not come up. Power off, check the 12 V supply and `Vin`; do not repeatedly retry the power-on sequence. |
| Board powers on but the PL rail collapsed earlier | The BMC reports it as healthy — an all-clear is **not** evidence the firmware is alive. See [What a PL rail collapse looks like](#what-a-pl-rail-collapse-looks-like). |

**Roles show `(assumed)` with probing enabled.** The probe could not open or
get a reply from that port, so the role came from the interface-number table.
The reason is printed on a `!` line:

```text
    interface 01: /dev/ttyUSB1   bmc (assumed)      Standard Com Port
      ! interface 01 not probed: /dev/ttyUSB1 is already open -- close any
        picocom/screen session on it
```

| Note | Fix |
|------|-----|
| *already open* | picocom/screen holds the port. Exit with `Ctrl-A Ctrl-X`. |
| *permission denied* | Not in `dialout`; see [Permissions](#permissions-and-udev-aliases). |
| *no reply to a bare CR or to krm4:2* | Port opened cleanly but said nothing. Normal on the console side; on the BMC side check baud rate and that the board has power. |

An assumed role is safe to act on — every command path reads a parseable
`krm4:i` first, so a wrong port errors out before anything is sent. To bypass
it, name the device: `--board /dev/ttyUSB1`.

## What a PL rail collapse looks like

[clock_source.md](clock_source.md) documents the failure where `krc-utils
init` against a loaded PL collapses the fabric rail. Captured here by
reprogramming the clocks with the dual-pipeline firmware fully loaded.

**The BMC does not report the fault.**

| Field | Loaded, healthy | After the collapse |
|-------|-----------------|--------------------|
| `EXT EN` / `EXT POK` | high / high | high / high |
| `RST_OUT` | low | low |
| `PS_RST` | released (high) | released (high) |
| `systemstate` | 8 | 8 |
| `powerstate` | **0** | **0** |
| `CFG_OK` | low | high |
| `PL_DONE` | **yes (high)** | **no (low)** |
| `BMC temp` | 57 °C | 60 → 56 → 52 °C over successive reads |
| `Vin` | 11.8 V | 12.0 V |
| `P1` / `P2` | `0x5d` / `0x3f` | `0x5c` / `0x77` |

Every rail flag stays healthy, so **`souk-board-status` reports no error** —
the fabric rail is not one the BMC watches. The aftermath is visible instead:
`PL_DONE` drops `yes` → `no` (the one direct signature), the board **cools**
as the dead fabric stops dissipating, and `Vin` returns to its no-load value.

> **A single snapshot cannot distinguish a collapsed PL from a board that was
> never programmed.** Against the `krm4:i` taken seconds after a `krm4:1`, the
> collapsed readings are **identical except for temperature**: `CFG_OK` high,
> `PL_DONE` no, `P0 0xf7`, `P1 0x5c`, `P2 0x77`, `P3 0x0e`, `systemstate 8`,
> `powerstate 0`.
>
> Only history separates them: `PL_DONE` was `yes` a moment ago, temperature
> is falling rather than rising, `Vin` went up rather than down.

From the PS:

```text
Timeout: lmk04208.0 not locked
[lmk04208.0] status: unlocked
[ lmx2594.0] status: unlocked low
[ lmx2594.1] status: unlocked low
[ lmx2594.2] status: unlocked low
```

### From the PS: on-chip sensors

`client.get_info('rfsoc_sensors')` reads both SYSMON blocks over IIO (see
`info_rfsoc_sensors()` in `firmware_lib.py`). Keys carry the raw sysfs index
because `vccint`, `vccaux`, `vccbram` and `vccams` appear on **both** blocks:
`voltage0`-`voltage17` are the PS SYSMON, `voltage18` upwards the PL SYSMON.

| Sensor | Collapsed | Healthy | |
|--------|-----------|---------|---|
| `temp2_pl_temp` | **-280.2 °C** | 49.4 °C | below absolute zero — impossible |
| `voltage18_vccint` | 0.0 | 0.842 | PL SYSMON |
| `voltage19_vccaux` | 0.0 | 1.793 | PL SYSMON |
| `voltage22_vccbram` | 0.0 | 0.844 | PL SYSMON |
| `voltage20`, `23`, `24`, `25`, `26` | 0.0 | nominal | PL SYSMON |
| `voltage2_vccint` | **0.0005** | 0.843 | PS SYSMON — genuinely collapsed |
| `voltage3_vccbram` | **0.849** | 0.843 | PS SYSMON — still up |
| `voltage4_vccaux` | **1.799** | 1.792 | PS SYSMON — still up |
| every `vccps*` rail | unchanged | — | PS unaffected |

> **The PL SYSMON's zeros are not measurements** — that block is powered from
> the PL rail, so it reports zeros and a sub-absolute-zero temperature when
> the rail goes. Trust the PS SYSMON (`voltage0`-`voltage17`).
>
> Read that way, this incident was **`vccint` alone collapsing** to 0.5 mV
> while `vccbram` (0.849 V) and `vccaux` (1.799 V) stayed nominal. The
> apparent loss of all three is an artefact of reading the dead block — weigh
> that against [clock_source.md](clock_source.md), which records all three
> dropping. One incident cannot settle which.

**The clearest indicator is a PL temperature below -273.15 °C** — no real
sensor can report it, and it needs no baseline.

There is no software recovery ([clock_source.md](clock_source.md)), and with
the LMK unlocked further AXI access risks hanging the PS. **A BMC power cycle
is the recovery:**

```bash
souk-board-power cycle --board 01449C3F --expect-serial 24NPv3ZnDm
```

> **Do not trust an all-clear from the BMC as evidence the firmware is
> alive.** It tells you the board has power. Whether the PL is running is a
> question for `krc-utils status` and `PL_DONE`.

---

## See also

- [Clock Source](clock_source.md) — the LMK/LMX clock tree and `krc-utils`
- [Dual Pipeline](dual_pipeline.md) — what a power cycle costs
- [Installation Guide](installation.md) — SD card imaging and first boot
