# Timing Site Checklist

Use this checklist before and during RFSoC timing commissioning at a telescope
site.

## Before Travel

Identify the site grandmaster and network profile:

- GM make, model, firmware version, and reference source
- Expected `grandmasterIdentity`
- Expected `gm.ClockClass` during normal operation
- Desired PTP holdover policy for this hardware:
  `ptp_holdover_window_s`, `ptp_holdover_error_rate_ppm`, and what to do when
  `ptp_holdover_expired` is true but chrony is still selecting `PHC0`
- Desired freshness/quality thresholds:
  `ptp_ingress_stale_after_s` and `ptp_lock_offset_threshold_ns`
- PTP profile: default IEEE 1588, gPTP, G.8275.1, G.8275.2, or another site profile
- Domain number
- Transport: L2 or UDPv4
- Delay mechanism: E2E or P2P
- VLAN or multicast requirements
- Whether intermediate switches are transparent or boundary clocks
- Site NTP servers for fallback
- Whether the GM asserts `currentUtcOffsetValid`, `timeTraceable`, and `ptpTimescale`

If possible, test against the site GM or an identical unit before shipping.

## On Arrival

Check the link and hardware timestamp support:

```bash
ip link show end0
sudo ethtool -T end0
```

`ethtool -T` should report hardware transmit, hardware receive, and hardware raw
clock support.

Restart PTP after applying any site profile changes:

```bash
sudo systemctl restart ptp4l
sleep 30
journalctl -u ptp4l -n 50
```

Expected state progression:

```text
INITIALIZING -> LISTENING -> UNCALIBRATED -> SLAVE
```

If it stays in `LISTENING`, check cabling, VLANs, domain number, transport,
delay mechanism, multicast filtering, and IGMP.

## Verify the Grandmaster

```bash
sudo pmc -u -b 0 'GET PARENT_DATA_SET'
sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
```

For normal production operation, prefer:

```text
gm.ClockClass             6 or 7
currentUtcOffset          37
currentUtcOffsetValid     1
timeTraceable             1
ptpTimescale              1
```

If `currentUtcOffsetValid` is `1`, keep the default `offset 0`. Do not install
the lab-only `offset -37` workaround at site.

## Final End-to-End Check

```bash
sudo pmc -u -b 0 'GET PORT_DATA_SET' | grep portState
sudo pmc -u -b 0 'GET TIME_STATUS_NP'
sudo pmc -u -b 0 'GET TIME_PROPERTIES_DATA_SET'
chronyc tracking
chronyc sources -v
date -u
printf '{"cmd":"status"}\n' | nc -U /run/timing-monitor.sock
souk-test-timing-monitor stream 10
```

The monitor should settle to `locked_to_gm`, and the client should report:

```python
client.get_info("timing")["summary"]["ready_for_firmware_sync"]
```

as `True` after several stable polls.

In the timing status, `ptp_data_fresh` should be `True` and
`ptp_seconds_since_ingress_update` should stay below
`ptp_ingress_stale_after_s`. If `TIME_STATUS_NP` shows `ingress_time 0`, treat
any reported `gmPresent` or `master_offset` values as stale.
If the monitor starts with no GM but chrony is still selecting `PHC0`, expect
state `phc_free_run`, `ptp_lock_observed: false`, and
`estimated_abs_error_s: null`. That means the PHC is ticking, but absolute time
has not been verified by this monitor.

Record the typical `summary.ptp_master_offset_ns`,
`summary.estimated_abs_error_s`, `summary.ntp_best_error_s`, and
`chrony.root_distance_s` once the system has settled. Keep the raw
`ptp.pmc.*` and `chrony.sources` output with the commissioning notes if there
is any ambiguity.

## Record for Operations

Save a short site note with:

- GM identity and expected clock class
- PTP profile and domain number
- Network path and switch behaviour
- NTP fallback sources
- Typical `master_offset` range
- Timing-monitor holdover window and drift-rate settings
- Whether any UTC offset workaround is in use
- Tested PTP loss and reacquisition behaviour
