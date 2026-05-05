# RFSoC Timing and Synchronisation

This note describes the timing stack used by `souk_readout_tools` v1.2 on the SOUK 
RFSoCs.

## Scope

Implemented in v1.2:

- packaged `ptp4l`, chrony, and timing-monitor configuration
- `souk-enable-timing` to install config, enable services, and restart them
- `souk-timing-monitor`, a local daemon exposing timing status on a Unix socket
- `souk-test-timing-monitor`, a small lab/status client
- `client.get_info("timing")`, the public timing view for health checks
- `client.get_timing_status()`, the raw monitor/debug view

Not implemented yet:

- the multi-board coordinator that performs a simultaneous firmware timestamp
  sync across all participating RFSoC pipelines

The timing monitor tells us whether the board is in a suitable state to perform
a firmware timestamp sync. It does not perform the firmware sync and it does not
know when the firmware counter was last synced.

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

The current lab software grandmaster serves TAI-like PHC time and does not
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
specific lab software GM described above. If the laptop has no PHC, leave the
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
timebase. The current v1.2 timing monitor provides the readiness gate; the
multi-board coordinator that carries out the sync is still to be implemented.

### PPS Source and Firmware Reset Release

In this document, `PPS` means the one-pulse-per-second strobe generated from
the Ethernet TSU/PHC associated with `end0`; it is not a separate external PPS
input. The `end0` TSU counter is disciplined by `ptp4l` through the PHC. A TSU
compare/strobe output is programmed so that when the TSU reaches an integer
second boundary, the TSU emits a short pulse.

That pulse is wired into the FPGA firmware. The firmware samples/latches it on
the next FPGA fabric clock and stretches it internally for a short period so
the sync logic can use it reliably.

When software calls `r.sync.arm_sync()`, either locally or as one step in the
future global sync coordinator, the firmware arms the sync/reset release logic.
The firmware then waits for the TSU/PHC PPS strobe before releasing the system
reset and allowing the DSP pipeline to run from the aligned edge.

`r.sync.sw_sync()` can be used to inject a pulse to mimic the PPS strobe and
release the reset when no PPS strobe is being produced. It is useful
for local bring-up and non-PTP testing, but it is not a substitute for a
PTP/PPS-aligned firmware sync.

The TSU/PHC PPS strobe must be enabled before relying on `r.sync.arm_sync()`.
At the time of writing this is done by a helper in `souk-firmware`:

```bash
sudo python3 ~/souk-firmware/software/rfsoc_scripts/ptp/run_strobe.py
```

The helper programs the GEM TSU compare registers and keeps updating the
comparison second so a pulse is emitted at each new second boundary. This
should eventually become a packaged service or be folded into the timing
setup.

The intended coordinator procedure is:

1. Poll every participating board and pipeline with `client.get_info("timing")`.
2. Require `summary.state == "locked_to_gm"`,
   `summary.ready_for_firmware_sync == True`, `summary.ptp_fresh == True`, and
   `summary.ptp_stable == True` on every participant.
3. Reject the sync if any board is in `ptp_holdover`, `phc_free_run`,
   `ntp_synced`, `ntp_holdover`, `free_run`, `initializing`, or
   `unavailable`.
4. Choose a target PTP integer second far enough in the future for all control
   messages to arrive before the edge.
5. Ensure the TSU/PHC PPS strobe from `end0` is enabled and being received by
   firmware on every participant.
6. Load the target PTP integer second into each firmware sync interface.
7. Call `r.sync.arm_sync()` on each participant so the firmware waits for the
   TSU/PHC PPS strobe before releasing reset.
8. Wait for the target integer-second PPS edge.
9. Read back firmware sync status and `telescope_time`.
10. Log the target second, readback, and `summary` block from every
    participant.

There may be practical issues fitting all coordinator control messages into a
single one-second interval. The target second must be far enough in the future
that every board has loaded the requested time and armed `r.sync.arm_sync()`
before the same PPS edge arrives. Operating system scheduling, Python runtime
latency, TCP/network latency, or a slow participant could otherwise leave some
boards armed for the intended PPS edge while others miss it and release on the
following edge. The coordinator may therefore need further development to allow
a minimum wait longer than one second, and to verify that all boards have armed
successfully before committing to a shared target edge.

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
  current lab software GM.
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
