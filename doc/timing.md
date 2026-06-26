# RFSoC Timing and Synchronisation

This note describes the timing stack used by `souk_readout_tools` from v1.2 on 
the SOUK RFSoCs.

**Contents:**

- [Scope](#scope)
- [Timing Chain](#timing-chain)
- [Installation](#installation)
- [Standalone Laptop Monitoring](#standalone-laptop-monitoring)
- [RFSoC Quick Health Check](#rfsoc-quick-health-check)
- [PTP: `ptp4l` and `pmc`](#ptp-ptp4l-and-pmc)
- [Chrony: PHC and NTP](#chrony-phc-and-ntp)
- [`phc2sys`](#phc2sys)
- [Timing Monitor](#timing-monitor)
- [Holdover Behaviour](#holdover-behaviour)
- [Firmware Timestamp Sync Procedure](#firmware-timestamp-sync-procedure)
- [Configuration Reference](#configuration-reference)
- [Site Notes](#site-notes)
- [Setting up a Local PTP Grandmaster](#setting-up-a-local-ptp-grandmaster)

## Scope

Implemented in v1.2:

- packaged `ptp4l`, chrony, and timing-monitor configuration
- `souk-enable-timing` to install config, enable services, and restart them
- `souk-timing-monitor`, a local daemon exposing timing status on a Unix socket
- `souk-test-timing-monitor`, a small lab/status client
- `client.get_info("timing")`, the public timing view for health checks
- `client.get_timing_status()`, the raw monitor/debug view

From v1.5:
- the firmware **timed sync** commands `client.timed_sync_needed/ready/arm/check`,
  which align `telescope_time` to the PPS (the load latches on a PPS edge) and arm a
  deterministic sync at a chosen second (see
  [tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md))

Not implemented yet:

- ~~a packaged multi-board coordinator. It is a short loop over the per-board
  `timed_sync_*` commands (sketch in
  [tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md)); the building
  blocks now exist.~~
- client commands for timed-sync now exist so a "multi-board coordinator" can 
  straightforwardly implement a multi-board sync, by looping over clients. Details to follow.

The timing monitor tells us whether the board is in a suitable state to perform a
firmware timestamp sync (`summary.ready_for_firmware_sync`, surfaced by
`client.timed_sync_needed()` / `timed_sync_ready()`). The `timed_sync_*` commands
then perform the sync and report its alignment and drift via
`client.timed_sync_check()`.

## Timing Chain

The RFSoC timing path is:

```text
PTP grandmaster
    |
    | PTP packets, hardware timestamped by end0
    v
ptp4l
    |
    | disciplines
    v
/dev/ptp0, the Ethernet PTP Hardware Clock (PHC)
    |
    | chrony refclock PHC0
    v
Linux CLOCK_REALTIME
    |
    | firmware sync procedure
    v
SOUK firmware telescope_time counter
```

Important distinctions:

- `ptp4l` receives PTP packets and disciplines `/dev/ptp0`.
- chrony reads `/dev/ptp0` as a local refclock named `PHC0` and disciplines
  Linux `CLOCK_REALTIME`.
- chrony can also use NTP servers as fallback for Linux time.
- the firmware `telescope_time` counter is separate. Linux time being correct
  does not automatically sync the firmware counter.
- NTP is useful fallback for Linux wall-clock time, but it is not precise
  enough for a new firmware timestamp sync.

## Installation

Install the OS packages:

```bash
sudo apt install linuxptp chrony
```

Install the packaged timing config and restart services:

```bash
souk-enable-timing
```

The installer prepares the `~/.souk_readout_tools` template tree, installs the
timing config and systemd units into `/etc`, backs up existing files before
replacing them, reloads systemd, enables the services, and restarts `ptp4l`,
chrony, and `timing-monitor`.

The packaged chrony PHC refclock uses `offset 0`, which is the expected setting
for a production grandmaster that serves UTC correctly.

The current lab hardware (PHC) grandmaster serves TAI-like PHC time and does not
assert a valid UTC offset. For that lab setup only:

```bash
souk-enable-timing --offset=-37
```

Keeping `offset -37` with a proper site grandmaster would make Linux time wrong
by 37 seconds.

To update service units while deliberately keeping the existing `/etc` timing
config files:

```bash
souk-enable-timing --preserve-config
```

## Standalone Laptop Monitoring

The timing monitor can also be used as a standalone PTP/NTP status tool on a
laptop or lab PC, without starting the SOUK readout server or installing the
full RFSoC service setup. This is useful for checking whether a machine can see
a PTP grandmaster, whether chrony is selecting PHC or NTP, and what state the
monitor would report.

Install the OS tools first:

```bash
sudo apt install linuxptp chrony ethtool netcat-openbsd
```

Check the laptop interface name and whether it has a PTP Hardware Clock. The
interface might be called `eth0`, `enp1s0`, `enx...`, or similar; on the RFSoC
timing interface it is usually `end0`.

```bash
ip link
sudo ethtool -T <iface>
```

`ethtool -T` should list hardware transmit timestamps, hardware receive
timestamps, and a hardware raw clock if the laptop is expected to follow the
same PHC timing path as the RFSoC. Many laptop Ethernet or USB Ethernet
adapters do not provide a PHC. In that case the laptop cannot track PTP time
with the RFSoC's hardware-timestamp precision, but `ptp4l` can still be useful
in software timestamp mode as a grandmaster visibility check.

Start chrony:

```bash
sudo systemctl enable --now chrony
```

Install or adapt the packaged PTP config from a source checkout:

```bash
sudo install -D -m 0644 src/souk_readout_tools/data/timing/ptp4l.conf /etc/linuxptp/ptp4l.conf
```

If the machine has a PHC and chrony should use it as `PHC0`, also install the
chrony PHC drop-in:

```bash
sudo install -D -m 0644 src/souk_readout_tools/data/timing/ptp-phc.conf /etc/chrony/conf.d/ptp-phc.conf
sudo systemctl restart chrony
```

Review `/etc/linuxptp/ptp4l.conf` for the local grandmaster profile, especially
`domainNumber`, `network_transport`, and `delay_mechanism`. Review
`/etc/chrony/conf.d/ptp-phc.conf` before enabling the PHC refclock; for a
normal UTC grandmaster keep `offset 0`, and only use `offset -37` for the
specific lab hardware GM described above. If the laptop has no PHC, leave the
PHC drop-in out and let chrony use normal NTP sources.

With a PHC-capable interface, run `ptp4l` in the foreground on the chosen
interface:

```bash
sudo ptp4l -f /etc/linuxptp/ptp4l.conf -i <iface> -s -m
```

Without a PHC, use software timestamps and keep `ptp4l` free-running so it can
listen for/select a GM without trying to steer the system clock:

```bash
sudo ptp4l -f /etc/linuxptp/ptp4l.conf -i <iface> -s -m --time_stamping software --free_running 1
```

This no-PHC mode can show that a grandmaster is present and that the PTP domain,
transport, delay mechanism, VLANs, and multicast path are plausible. It is not
equivalent to the RFSoC hardware-timestamped PHC path, and it should not be
used as evidence that firmware timestamp sync would be safe.

In another terminal, run the monitor directly from the checkout:

```bash
sudo python3 src/souk_readout_tools/server/timing_monitor.py
```

Running as root lets the monitor create `/run/timing-monitor.sock` and query
the default `ptp4l` management socket. If you have installed the server console
scripts, the equivalent command is:

```bash
sudo souk-timing-monitor
```

Query one status sample or stream updates:

```bash
python3 src/souk_readout_tools/server/server_scripts/souk_test_timing_monitor.py status
python3 src/souk_readout_tools/server/server_scripts/souk_test_timing_monitor.py stream 10
printf '{"cmd":"status"}\n' | nc -U /run/timing-monitor.sock
```

With installed console scripts, use:

```bash
souk-test-timing-monitor status
souk-test-timing-monitor stream 10
```

For NTP-only monitoring, skip the PHC chrony drop-in and `ptp4l` command. The
monitor will report states such as `ntp_synced`, `ntp_holdover`, or `free_run`
from chrony, while `ready_for_firmware_sync` remains false because no fresh PTP
lock is available.

## RFSoC Quick Health Check

Run these first when commissioning or debugging:

```bash
ip link show end0
sudo ethtool -T end0
systemctl status ptp4l chrony timing-monitor
journalctl -u ptp4l -n 80 --no-pager
sudo pmc -u -b 0 'GET PORT_DATA_SET'
sudo pmc -u -b 0 'GET TIME_STATUS_NP'
sudo pmc -u -b 0 'GET PARENT_DATA_SET'
sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
chronyc tracking
chronyc sources -v
souk-test-timing-monitor status
```

`ethtool -T end0` should report hardware transmit timestamps, hardware receive
timestamps, and a hardware raw clock. Without hardware timestamp support, PTP
can exchange packets but cannot provide the precision needed here.

Use `ptp4l` and `pmc` to check whether the RFSoC is receiving and following PTP.
`ptp4l` is the process that listens for PTP packets, selects the grandmaster,
and reports the port state. `pmc` is the command-line tool for querying the PTP management interface, which provides detailed information about the PTP state and the grandmaster.

## PTP: `ptp4l` and `pmc`

The packaged service runs:

```text
/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i end0 -s -m
```

Flags:

- `-f /etc/linuxptp/ptp4l.conf`: use the packaged/site config
- `-i end0`: use the RFSoC Ethernet interface
- `-s`: slave-only mode
- `-m`: print log messages to stdout, which systemd captures in the journal

Manual foreground test:

```bash
sudo systemctl stop ptp4l
sudo ptp4l -f /etc/linuxptp/ptp4l.conf -i end0 -s -m
sudo systemctl restart ptp4l
```

Do not leave a manual `ptp4l` running while the service is also running.

Typical port progression is:

```text
INITIALIZING -> LISTENING -> UNCALIBRATED -> SLAVE
```

If the port stays at `LISTENING`, check cabling, VLANs, multicast filtering,
PTP domain number, transport, delay mechanism, switch behaviour, and the
grandmaster.

Useful `pmc` queries:

```bash
sudo pmc -u -b 0 'GET PORT_DATA_SET'
sudo pmc -u -b 0 'GET TIME_STATUS_NP'
sudo pmc -u -b 0 'GET PARENT_DATA_SET'
sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
```

`GET PORT_DATA_SET`:

- `portState`: should settle to `SLAVE`

`GET TIME_STATUS_NP`:

- `master_offset`: PHC-to-grandmaster offset in nanoseconds
- `gmPresent`: whether `ptp4l` currently believes a GM is present
- `gmIdentity`: grandmaster identity
- `ingress_time`: timestamp of the last PTP ingress update

`ingress_time` is the important freshness signal. If it is zero, missing, or
not advancing, treat `gmPresent` and `master_offset` as stale.

`GET PARENT_DATA_SET`:

- `parentPortIdentity`: port being followed
- `gm.ClockClass`: grandmaster clock class
- `gm.ClockAccuracy`: encoded grandmaster accuracy
- `gm.OffsetScaledLogVariance`: encoded grandmaster stability/variance

`GET TIME_PROPERTIES_DATA_SET`:

- `currentUtcOffset`: current TAI-UTC offset, currently 37 seconds
- `currentUtcOffsetValid`: whether the GM says the offset is valid
- `timeTraceable`: whether time is traceable to a primary reference
- `ptpTimescale`: whether PTP timescale is in use

For a normal production GM, prefer:

```text
currentUtcOffset          37
currentUtcOffsetValid     1
timeTraceable             1
ptpTimescale              1
```

With a valid production GM, keep the chrony PHC refclock at `offset 0`.

## Chrony: PHC and NTP

Chrony disciplines Linux `CLOCK_REALTIME`. In this setup it has:

- a local PHC refclock, `PHC0`, backed by `/dev/ptp0`
- normal NTP servers as fallback for Linux time

The packaged PHC line is:

```text
refclock PHC /dev/ptp0 poll 0 dpoll -2 offset 0 prefer
```

Design choices:

- `prefer` makes PHC win when it agrees with real time.
- the PHC is not marked `trust`, because it keeps ticking when no GM is
  present. If a board starts with no GM and an incorrect PHC epoch, chrony must
  be allowed to reject PHC and select NTP.
- NTP is retained as fallback for Linux time, not as the precision source for a
  new firmware sync.

### `chronyc tracking`

```bash
chronyc tracking
```

Important fields:

- `Reference ID` / `Reference time`: source currently disciplining Linux time
- `Stratum`: distance from a reference clock
- `System time`: current correction between Linux time and chrony's best time
  estimate. This is not the Unix timestamp.
- `Last offset`: offset measured on the last update
- `RMS offset`: longer-term RMS offset; after a large time step it can remain
  large until chrony statistics are refreshed
- `Frequency`: frequency correction in ppm
- `Root delay`: delay component of the selected source error
- `Root dispersion`: accumulated uncertainty component
- `Leap status`: should normally be `Normal`

After a large clock correction, stale chrony source statistics can be reset:

```bash
sudo chronyc reset sources
sudo chronyc makestep
chronyc tracking
```

`makestep` can step Linux time. Do not run it during observations unless that
is the intended intervention.

### `chronyc sources -v`

```bash
chronyc sources -v
```

Use `chronyc sources -v` for verbose source output. Do not use
`chronyc -v sources`; `chronyc -v` prints the chronyc version and exits.

Example:

```text
MS Name/IP address         Stratum Poll Reach LastRx Last sample
===============================================================================
#* PHC0                          0   0   377     1     -1ns[   -1ns] +/-   67ns
^- prod-ntp-5.ntp4.ps5...        2  10   377   604  -5389us[-5389us] +/- 6104us
```

Mode column `M`:

- `^`: NTP server
- `=`: NTP peer
- `#`: local reference clock, such as `PHC0`

State column `S`:

- `*`: selected source
- `+`: combined with the selected source
- `-`: usable, but not combined
- `x`: falseticker
- `~`: too variable
- `?`: unreachable or unusable

The sample:

```text
-5389us[-5389us] +/- 6104us
```

means the adjusted and measured offsets are about -5.4 ms, with an estimated
error bound of about 6.1 ms.

Other useful chrony commands:

```bash
chronyc activity
chronyc sourcestats -v
chronyc waitsync 30 0.001
```

## `phc2sys`

`phc2sys` is the linuxptp tool often used to copy time between a PHC and Linux
`CLOCK_REALTIME`, for example:

```bash
sudo phc2sys -s /dev/ptp0 -c CLOCK_REALTIME -O 0
sudo phc2sys -a -r
```

The SOUK packaged setup does not use `phc2sys`. Chrony already reads
`/dev/ptp0` as `PHC0` and steers Linux time. Running `phc2sys` as well would
create a second service steering `CLOCK_REALTIME`, which would make the clock
state harder to interpret.

Current policy:

- use `ptp4l` to discipline `/dev/ptp0`
- use chrony to discipline Linux time from `PHC0` or NTP
- do not start `phc2sys` unless the timing design is deliberately changed

## Timing Monitor

The timing monitor combines PTP and chrony status into the state needed by
readout operations. It polls:

- `pmc` for PTP port state, GM data, and ingress freshness
- `chronyc` for system clock source, source health, and error estimates

It exposes a line-delimited JSON protocol on:

```text
/run/timing-monitor.sock
```

Commands:

```bash
souk-test-timing-monitor status
souk-test-timing-monitor ping
souk-test-timing-monitor stream 10
printf '{"cmd":"status"}\n' | nc -U /run/timing-monitor.sock
```

Python client:

```python
timing = client.get_info("timing")

summary = timing["summary"]
print(summary["state"], summary["active_source_type"])
print(summary["estimated_abs_error_s"], summary["ntp_best_error_s"])
print(summary["ready_for_firmware_sync"])

raw = client.get_timing_status()
print(raw["state"], raw["ready_for_firmware_sync"])
```

`get_info("timing")` is the public view. Use it for regular health logs and
operator status. `get_timing_status()` is the raw monitor/debug view; it is more
verbose and may contain parsed values that are useful only for the monitor state
machine or detailed debugging.

### Public Timing View

`client.get_info("timing")` is grouped as:

- `summary`: essential fields for health logs and dashboards
- `monitor`: daemon availability, sample time, poll count, and uptime
- `ptp`: concise PTP health fields plus raw `pmc` outputs
- `phc`: PHC selection and holdover policy fields
- `ntp`: compact NTP source/error summary
- `chrony`: concise chrony fields plus raw `chronyc tracking` and
  `chronyc sources -v` output strings

Important `summary` keys:

- `state`: monitor state, such as `locked_to_gm` or `ptp_holdover`
- `condition`: short readable state description
- `updated_unix_s`: Unix timestamp of the monitor sample
- `ready_for_firmware_sync`: true only when PTP is fresh, locked, and stable
- `absolute_time_verified`: true when absolute time is externally tied, or in
  observed PTP holdover
- `estimated_abs_error_s`: monitor estimate of absolute time error
- `active_source_type`: chrony's selected source type at the monitor sample
  (`phc`, `ntp`, `none`, or `unknown`)
- `active_source_name`: chrony's selected source name at the monitor sample
- `active_source_offset_s`: selected source offset, if available
- `active_source_error_s`: selected source error estimate, if available
- `system_synced`: true only when the system clock is actively synchronised to
  a current external source. It is false during PTP holdover.
- `system_time_offset_s`: chrony's current correction between Linux time and
  chrony's best time estimate; this is not Unix time
- `root_distance_s`: chrony's selected-source root distance
- `ptp_healthy`: fresh GM on a `SLAVE` port
- `ptp_fresh`: fresh, advancing PTP ingress data
- `ptp_stable`: several consecutive good PTP offset polls
- `ptp_gm_identity`: grandmaster identity reported by `ptp4l`, masked to an
  empty string when stale
- `ptp_last_gm_identity`: last non-stale grandmaster identity observed by the
  monitor, retained during PTP holdover
- `ptp_master_offset_ns`: PHC-to-GM offset, masked to `null` when stale
- `ptp_ingress_age_s`: age of the latest PTP ingress update
- `ptp_holdover_age_s`: seconds since the last observed good PTP lock
- `ptp_holdover_error_rate_ppm`: drift rate used for holdover error estimates
- `ptp_holdover_window_s`: nominal holdover policy window
- `ptp_holdover_expired`: whether the nominal policy window has expired
- `ntp_selected_source`: NTP source selected by chrony, if NTP is selected
- `ntp_selected_error_s`: selected NTP source error estimate
- `ntp_best_source`: lowest-error visible NTP source
- `ntp_best_error_s`: lowest visible NTP source error estimate
- `holdover_estimated_seconds_until_ntp_error`: rough estimate of when the PHC
  holdover error will reach the best visible NTP error bound

The holdover-vs-NTP estimate is calculated from
`estimated_abs_error_s`, `ntp_best_error_s`, and
`ptp_holdover_error_rate_ppm`. The same policy constants are also repeated in
the `monitor` section:

- `monitor.ptp_holdover_error_rate_ppm`
- `monitor.ptp_holdover_window_s`
- `monitor.ptp_ingress_stale_after_s`
- `monitor.ptp_lock_offset_threshold_ns`

Important detailed keys:

- `ptp.pmc.port_data_set`: raw `pmc 'GET PORT_DATA_SET'` output
- `ptp.pmc.time_status_np`: raw `pmc 'GET TIME_STATUS_NP'` output
- `ptp.pmc.parent_data_set`: raw `pmc 'GET PARENT_DATA_SET'` output
- `ptp.pmc.time_properties_data_set`: raw
  `pmc 'GET TIME_PROPERTIES_DATA_SET'` output
- `ptp.last_gm_identity`: same last non-stale GM identity, grouped with the
  PTP diagnostics
- `chrony.tracking`: raw `chronyc tracking` output
- `chrony.sources`: raw `chronyc sources -v` output
- `phc.holdover_error_rate_ppm`: drift rate used for holdover error estimates
- `phc.estimated_seconds_until_ntp_error`: same holdover-vs-NTP estimate as the
  summary, grouped with the PHC fields
- `chrony.synced`: chrony's `Leap status == Normal` view. This can remain true
  during PTP holdover because chrony is still selecting `PHC0`.

### Timing States

| State | Meaning | Operational interpretation |
| --- | --- | --- |
| `initializing` | Monitor recently started and no usable source has locked yet | Wait for PTP/PHC/NTP to settle |
| `locked_to_gm` | Fresh PTP GM on a `SLAVE` port | Firmware sync may be allowed once stable |
| `ptp_holdover` | PTP lock was observed, PTP is now lost, and PHC holdover is active | Do not perform a new firmware sync; monitor error growth |
| `phc_free_run` | Chrony is selecting `PHC0`, but this monitor has not observed PTP lock | Absolute time is unverified |
| `ntp_synced` | Chrony is selecting NTP | Linux time is synced, but firmware sync precision is NTP-class |
| `ntp_holdover` | A previous source existed but no source is currently selected | Degraded |
| `free_run` | Startup grace expired and no usable source has been seen | Do not trust absolute timestamps |

PTP data is considered stale when `TIME_STATUS_NP.ingress_time` is missing,
zero, not advancing, or the port is not `SLAVE`. In stale cases the monitor
masks stale PTP values:

```json
{
  "ptp_data_fresh": false,
  "ptp_gm_present": false,
  "ptp_master_offset_ns": null
}
```

## Holdover Behaviour

During PTP loss, chrony can continue selecting `PHC0` for as long as PHC drift
remains within NTP's claimed error bounds. This is expected behaviour, not a
bug. Chrony's selected source alone is not enough to decide whether PTP is
healthy.

Applications should use `summary.state` or raw `state`. It is normal to see:

```text
chrony:   #* PHC0
monitor:  ptp_holdover
```

The monitor reports `ptp_holdover` with a growing `estimated_abs_error_s`.
Measured GEM holdover in the lab was about 0.7 to 0.75 ppm. After roughly
7240 seconds without the PTP GM, one NTP source reported:

```text
-5389us[-5389us] +/- 6104us
```

The monitor default remains 1 ppm as a conservative estimate. Tune only with
measured board data:

```text
souk-timing-monitor --ptp-holdover-error-rate-ppm 0.7
```

To force chrony onto NTP during extended holdover:

```bash
sudo chronyc -a offline refid:PHC0
```

Reverse with:

```bash
sudo chronyc -a online refid:PHC0
```

## Firmware Timestamp Sync Procedure

The SOUK firmware `telescope_time` counter must be explicitly synced to the
timebase. This is now implemented as the `client.timed_sync_*` commands (ports of
the `scripts/timed_sync/` bring-up scripts); see
[tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md) for the command
reference and the multi-board flow. The timing monitor provides the readiness gate
those commands consume. The description below is firmware background.

### PPS Source and Firmware Reset Release

In this document, `PPS` means the one-pulse-per-second strobe generated from
the Ethernet TSU/PHC associated with `end0`; it is not a separate external PPS
input. The `end0` TSU counter is disciplined by `ptp4l` through the PHC. A TSU
compare/strobe output is programmed so that when the TSU reaches an integer
second boundary, the TSU emits a short pulse.

That pulse is wired into the FPGA firmware. The firmware samples/latches it on
the next FPGA fabric clock and stretches it internally for a short period so
the sync logic can use it reliably.

The v7.10 firmware **timed sync** reloads the internal telescope time so it latches
and aligns on a PPS edge (`r.sync.update_internal_time()`), then arms a reset+sync to
fire when the running TT reaches a chosen target second
(`r.sync.set_timed_sync(target_tt)`). These are two separate PPS edges: the TT load
aligns timekeeping on the next edge; the timed sync fires the DSP reset/sync at the
target second. `client.timed_sync_arm()` wraps this, reloading the TT value only when
needed (it drifts ~1.6 ppm but only reaches a whole second after ~7 days; the per-fire
PPS re-align handles sub-second drift). Because every board can be armed at the same
target second, this is the basis for multi-board sync.

`r.sync.sw_sync()` injects an immediate software sync pulse (no PPS alignment); it
is useful for local bring-up and non-PTP testing, but it is not a substitute for a
PTP/PPS-aligned timed sync.

The TSU/PHC PPS strobe must be running before a timed sync. It is now a packaged
service, `souk-tsu-strobe` (control: `souk-tsu-strobe-{install,start,status,...}`);
its health is surfaced in `get_info("timing")` under `strobe` and
`summary.strobe_healthy`, and `timed_sync_arm` refuses (unless `force=True`) when it
is not healthy. See [tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md).

The per-board flow (now implemented) is:

1. `client.timed_sync_ready(target_unix_s=T)` on every participant with a common
   target second `T`, far enough ahead for all control messages (plus a one-time
   ~3-4 s TT load on any board whose TT is not yet set) to complete. It checks
   `ready_for_firmware_sync`, the PPS strobe, and that `T` is in the future versus
   the server, firmware, and client clocks.
2. Proceed only if every board returns `ready == True`.
3. `client.timed_sync_arm(target_unix_s=T)` on each participant. The firmware fires
   the reset+sync autonomously when its telescope time reaches `T`.
4. `client.timed_sync_check()` on each participant to confirm alignment (boundary
   offset) and drift.

A packaged multi-board coordinator is just this loop over the per-board commands
(sketch in [tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md)). The
target second must be far enough in the future that every board has re-disciplined
its TT and armed before the edge; operating-system scheduling, Python runtime
latency, network latency, or a slow participant could otherwise leave some boards
armed for the intended edge while others miss it and fire on the following second.

## Configuration Reference

### `/etc/linuxptp/ptp4l.conf`

The packaged `ptp4l.conf` contains:

| Parameter | Meaning |
| --- | --- |
| `clientOnly 1` | Do not become grandmaster; the RFSoC is a slave/client. |
| `twoStepFlag 1` | Use two-step PTP messages, common for hardware timestamped PTP. |
| `domainNumber 0` | PTP domain. Must match the grandmaster/profile. |
| `clockClass 248` | Local clock class advertised if relevant; low quality/default. |
| `clockAccuracy 0xFE` | Local clock accuracy advertised if relevant; unknown/low quality. |
| `offsetScaledLogVariance 0xFFFF` | Local variance advertised if relevant; worst/default. |
| `dataset_comparison ieee1588` | Best-master-clock comparison algorithm. |
| `maxStepsRemoved 255` | Accept GMs up to this many PTP steps away. |
| `logAnnounceInterval 1` | Announce interval as log2 seconds; `1` means 2 s. |
| `logSyncInterval 0` | Sync interval as log2 seconds; `0` means 1 s. |
| `logMinDelayReqInterval 0` | Delay request interval as log2 seconds; `0` means 1 s. |
| `announceReceiptTimeout 3` | Number of missed announces before the GM is considered lost. |
| `clock_class_threshold 248` | Reject GMs with clock class worse than this threshold. Lab default is permissive. |
| `kernel_leap 1` | Allow kernel leap-second handling from PTP properties. |
| `first_step_threshold 0.00002` | Step threshold for first correction, in seconds. |
| `clock_servo pi` | PI servo for disciplining the PHC. |
| `uds_address /var/run/ptp4l` | Unix socket used by `pmc` for management queries. |
| `uds_file_mode 0660` | Permissions on the main management socket. |
| `uds_ro_address /var/run/ptp4lro` | Read-only management socket. |
| `uds_ro_file_mode 0666` | Permissions on the read-only socket. |
| `clock_type OC` | Ordinary clock. |
| `network_transport L2` | IEEE 802.3 layer-2 PTP transport. Match site profile. |
| `delay_mechanism E2E` | End-to-end delay measurement. Match site profile. |
| `time_stamping hardware` | Use hardware timestamping from the Ethernet MAC/PHC. |
| `tsproc_mode filter` | Timestamp processing mode. |
| `delay_filter moving_median` | Delay filter type. |
| `delay_filter_length 10` | Number of samples in the delay filter. |
| `phc_index -1` | Let linuxptp choose the PHC associated with the interface. |
| `utc_offset 37` | Commented fallback for GMs that do not announce valid UTC offset. Prefer a GM that announces this correctly. |

At site, review `domainNumber`, `network_transport`, `delay_mechanism`, and
`clock_class_threshold` against the grandmaster profile. The lab default:

```text
clock_class_threshold 248
```

is permissive. A site configuration may want a stricter value after confirming
the expected GM clock class.

### `/etc/chrony/conf.d/ptp-phc.conf`

Packaged line:

```text
refclock PHC /dev/ptp0 poll 0 dpoll -2 offset 0 prefer
```

Fields:

- `refclock PHC`: use chrony's PHC refclock driver
- `/dev/ptp0`: PHC disciplined by `ptp4l`
- `poll 0`: poll the refclock every 1 second
- `dpoll -2`: driver poll interval of 2^-2 seconds, i.e. 0.25 s
- `offset 0`: apply no PHC-to-UTC correction for a proper UTC GM
- `prefer`: prefer PHC when it agrees with other sources

The file also contains:

- `bindcmdaddress 127.0.0.1`
- `bindcmdaddress ::1`

These allow local `chronyc` queries from the monitor.

### `ptp4l.service`

Important unit settings:

- `After=network-online.target` and `Wants=network-online.target`: start after
  the network is up.
- `Type=simple`: run `ptp4l` in the foreground under systemd.
- `ExecStart=/usr/sbin/ptp4l -f /etc/linuxptp/ptp4l.conf -i end0 -s -m`: the
  full command described above.
- `Restart=on-failure` and `RestartSec=5`: restart on failure after 5 seconds.
- `CPUSchedulingPolicy=fifo` and `CPUSchedulingPriority=50`: request real-time
  scheduling for lower timing jitter.
- `WantedBy=multi-user.target`: enable for normal multi-user boot.

### `timing-monitor.service`

Important unit settings:

- `After=ptp4l.service chrony.service network-online.target` and matching
  `Wants=`: start with the timing dependencies.
- `Type=simple`: run the monitor in the foreground.
- `ExecStart=/home/casper/py3.12-venv/bin/souk-timing-monitor`: monitor command.
- `Restart=on-failure` and `RestartSec=5`: restart on failure after 5 seconds.
- `User=root`: permits access to ptp4l/chrony sockets and system timing status.
- `SyslogIdentifier=souk-timing-monitor`: journal identifier.
- `Environment=PYTHONUNBUFFERED=1`: flush log output promptly.
- `WantedBy=multi-user.target`: enable for normal multi-user boot.

### Installer Commands

`souk-enable-timing` arguments:

- `--phc-offset SECONDS`, `--offset SECONDS`: set the chrony PHC refclock offset
  while installing config. Default is `0`. Use `--offset=-37` only for the
  current lab hardware GM.
- `--preserve-config`: keep existing `/etc/linuxptp/ptp4l.conf` and
  `/etc/chrony/conf.d/ptp-phc.conf`, but update service units and restart
  services.

The underlying root script is:

```bash
sudo /home/casper/.souk_readout_tools/daemon/install_timing_services.sh
```

Script arguments are the same:

- `--phc-offset SECONDS`
- `--offset SECONDS`
- `--preserve-config`
- `--help`

The script backs up existing regular files before replacement, runs
`systemctl daemon-reload`, enables `ptp4l.service` and
`timing-monitor.service`, restarts `ptp4l.service`, `chrony.service`, and
`timing-monitor.service`, then prints service status.

### Timing Monitor Command

`souk-timing-monitor` arguments:

- `--socket-path PATH`: Unix socket to create; default `/run/timing-monitor.sock`
- `--poll-interval SECONDS`: monitor poll interval; default `1.0`
- `--ptp-holdover-window SECONDS`: nominal holdover window before
  `ptp_holdover_expired` becomes true; default `60.0`
- `--ptp-holdover-error-rate-ppm PPM`: drift rate used for
  `estimated_abs_error_s`; default `1.0`
- `--ptp-ingress-stale-after SECONDS`: age of unchanged `ingress_time` before
  PTP is stale; default `5.0`
- `--ptp-lock-offset-threshold-ns NS`: absolute `master_offset` threshold for a
  good PTP poll; default `1000`
- `--initializing-grace SECONDS`: startup grace before reporting `free_run` when
  no source has appeared; default `30.0`

`souk-test-timing-monitor` arguments:

- `status`: print one raw status object
- `ping`: check socket responsiveness
- `stream [COUNT]`: stream raw status lines; `0` streams forever
- `-n, --stream-count COUNT`: default stream count when no positional count is
  supplied
- `--socket-path PATH`: Unix socket path

## Site Notes

Before site work, record:

- GM make, model, firmware version, and reference source
- expected `grandmasterIdentity`
- expected `gm.ClockClass`
- PTP profile and domain number
- transport: L2 or UDPv4
- delay mechanism: E2E or P2P
- VLAN or multicast requirements
- switch behaviour: transparent clock, boundary clock, or ordinary switching
- site NTP fallback sources
- whether the GM asserts `currentUtcOffsetValid`, `timeTraceable`, and
  `ptpTimescale`
- desired holdover policy and drift-rate assumptions

See `doc/timing_site_checklist.md` for the commissioning checklist.

## Setting up a Local PTP Grandmaster

When there is no site or GPS grandmaster, a Linux workstation on the same network
can act as the GM for the RFSoCs — provided its NIC supports hardware PTP
timestamping (a PHC); software-timestamping mode did not work in our testing (see
below). These notes call such a workstation a
**hardware-PHC grandmaster** — an ordinary PC serving time from its NIC's PHC in
hardware-timestamping mode, as opposed to a dedicated GPS/atomic appliance. All lab
results in this note and in
[tsu_strobe_and_timed_sync.md](tsu_strobe_and_timed_sync.md) were taken against this
kind of GM.

That distinction matters here: serve from a NIC with a PTP Hardware Clock (hardware
timestamping), whose served time is stable to ~µs. Pure software timestamping (no
PHC) is OS-jitter limited (tens of µs or worse), is not traceable, and — see the lab
observation below — **did not work with the RFSoC at all in our testing**.

### Check what the NIC supports

```bash
ip link
sudo ethtool -T <iface>
```

For a hardware GM, `ethtool -T` should report:

- `PTP Hardware Clock: 0` (any index `>= 0`; `-1` / `none` means no PHC)
- `hardware-transmit` and `hardware-receive` under capabilities
- `SOF_TIMESTAMPING_TX_HARDWARE`, `..._RX_HARDWARE`, `..._RAW_HARDWARE`
- a non-empty list of Hardware Transmit/Receive Timestamp Modes

Confirm the clock device node exists:

```bash
ls -l /dev/ptp*
```

Server NICs and chips such as Intel i210/i225 expose a PHC; most consumer and USB
Ethernet adapters do not. No PHC leaves only software-timestamping mode, which did
not work with the RFSoC in our testing (see below) — so a PHC-capable interface is
effectively required.

### Install the tools

```bash
sudo apt install linuxptp chrony ethtool
```

`linuxptp` provides `ptp4l`, `phc2sys`, `phc_ctl`, and `pmc`.

### Hardware grandmaster (preferred)

Three jobs: discipline the workstation's own system clock, copy that time into the
NIC PHC, and serve the PHC out as a PTP master. The commands and outputs below are
from a working lab grandmaster (`user`/`host`/paths masked).

1. Keep the workstation's system clock sane with chrony (NTP, or a local GPS/PPS
   source if you have one):

   ```console
   user@gm:~$ sudo systemctl enable --now chrony
   user@gm:~$ chronyc tracking
   Reference ID    : B97DBE3A (prod-ntp-5.ntp4.ps5.canonical.com)
   Stratum         : 3
   System time     : 0.000012441 seconds fast of NTP time
   Last offset     : -0.000007160 seconds
   Frequency       : 52.803 ppm fast
   Leap status     : Normal
   ```

2. Copy `CLOCK_REALTIME` into the PHC with `phc2sys`. Note `-c` is the clock being
   *set* (destination) and `-s` is the *source*. This must keep running, so
   background it or give it its own terminal/service:

   ```console
   user@gm:~$ sudo phc2sys -s CLOCK_REALTIME -c /dev/ptp0 -O 0 -w -m
   phc2sys[172887.397]: /dev/ptp0 sys offset 7886173603 s0 freq      -0 delay    850
   phc2sys[172888.397]: /dev/ptp0 sys offset 7886219499 s1 freq  +45876 delay    867
   phc2sys[172889.398]: /dev/ptp0 sys offset        31 s2 freq  +45907 delay    913
   phc2sys[172890.398]: /dev/ptp0 sys offset        39 s2 freq  +45925 delay    946
   phc2sys[172899.401]: /dev/ptp0 sys offset        39 s2 freq  +45923 delay    992
   ```

   The servo state goes `s0` (unlocked) → `s1` (first step) → `s2` (locked); once in
   `s2` the offset settles to tens of nanoseconds. `-O 0` applies no offset between
   the two clocks (so the PHC carries UTC; see the UTC-offset note below) and `-w`
   waits for the source to be ready. Inspect or set the PHC by hand with
   `phc_ctl /dev/ptp0 get` / `set` if needed. This is the one place `phc2sys` *is*
   wanted — on the RFSoC slaves it is deliberately not used (see the
   [`phc2sys`](#phc2sys) section).

3. Serve PTP as master on the chosen interface. `ptp4l` becomes grandmaster via the
   BMC algorithm when it is the best clock on the segment; force it with
   `--serverOnly 1` (or `serverOnly 1` in the config). Use a sensible `clockClass`
   (`248` for a free/NTP-fed GM; a low value only if genuinely locked to a primary
   reference). This also runs continuously:

   ```console
   user@gm:~$ sudo ptp4l -i <iface> -f /etc/linuxptp/ptp4l.conf --serverOnly 1 -m
   ptp4l[172972.970]: selected /dev/ptp0 as PTP clock
   ptp4l[172972.974]: port 1 (<iface>): INITIALIZING to LISTENING on INIT_COMPLETE
   ptp4l[172979.954]: port 1 (<iface>): LISTENING to MASTER on ANNOUNCE_RECEIPT_TIMEOUT_EXPIRES
   ptp4l[172979.954]: selected local clock 1cb72c.fffe.ef2f5d as best master
   ptp4l[172979.954]: port 1 (<iface>): assuming the grand master role
   ```

   `domainNumber`, `network_transport` (L2 vs UDPv4), and `delay_mechanism`
   (E2E/P2P) must match what the RFSoCs use (see
   [Configuration Reference](#etclinuxptpptp4lconf)); a slave only follows a GM in
   its own domain and profile. On a multi-PHC machine, confirm which `/dev/ptpN`
   belongs to the serving interface (`ethtool -T <iface>` reports its `PTP Hardware
   Clock:` index) and point `phc2sys` at the same one.

Confirm the workstation is master:

```console
user@gm:~$ sudo pmc -u -b 0 'GET PORT_DATA_SET'
sending: GET PORT_DATA_SET
        1cb72c.fffe.ef2f5d-1 seq 0 RESPONSE MANAGEMENT PORT_DATA_SET
                portIdentity            1cb72c.fffe.ef2f5d-1
                portState               MASTER
                ...
```

On an RFSoC the slave port should then progress to `SLAVE` and
`souk-test-timing-monitor status` should report `locked_to_gm`.

#### Deployed as systemd services (lab example)

In the lab the two long-running commands are packaged as their own systemd units
rather than left in terminals — separate from the RFSoC-side `ptp4l.service`:

- **`ptp4l-gm.service`** ("PTP4L Grandmaster") runs a dedicated GM config:

  ```text
  /usr/local/sbin/ptp4l -f /etc/linuxptp/ptp4l-gm.conf -i <iface> -m
  ```

  The GM config mirrors the packaged `ptp4l.conf` with master-suitable values:
  `clientOnly 0`, `clockClass 248`, `domainNumber 0`, `delay_mechanism E2E`,
  `time_stamping hardware`. (`network_transport` must match the RFSoCs.)

- **`phc2sys-gm.service`** runs `phc2sys` in automatic mode, following `ptp4l` and
  keeping the PHC and system clock aligned:

  ```text
  /usr/local/sbin/phc2sys -a -r -r -m
  ```

  `-a` takes the clocks to discipline from the running `ptp4l`; `-r -r` lets the
  system (NTP-disciplined) clock act as the reference. This is the service-mode
  equivalent of the explicit `-s CLOCK_REALTIME -c /dev/ptp0` form shown above.

**UTC offset.** With `phc2sys -O 0` the PHC carries UTC while PTP normally expects a
TAI timescale, and this kind of GM does not assert a valid UTC offset. That is the
lab setup described under [Installation](#installation): the RFSoCs compensate with
`souk-enable-timing --offset=-37`. With a proper UTC grandmaster, keep `offset 0`.

### Software-timestamping mode (no PHC — did not work in lab testing)

If the NIC has no PHC, `ptp4l` can in principle serve PTP using software timestamps,
with no PHC to manage (so no `phc2sys`); `ptp4l` would serve the chrony-disciplined
system clock directly:

```bash
sudo ptp4l -i <iface> -f /etc/linuxptp/ptp4l.conf --serverOnly 1 --time_stamping software -m
```

**Lab observation (unresolved).** This did not work for us. The RFSoC never
disciplined to a software-timestamping GM; every successful result for this system
used the workstation NIC's **PHC in hardware mode**. We did not establish why
software mode failed (plausibly the OS-jitter/served-clock quality, or driver
timestamping behaviour) and did not pursue it, because the hardware PHC path worked.
Treat the command above as documented-but-unproven and use a PHC-capable interface.

To run the hardware GM persistently, wrap the chosen `ptp4l` (and `phc2sys`) command
in a systemd unit rather than leaving a foreground process running.
