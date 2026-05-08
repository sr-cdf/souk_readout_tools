# Timing and Synchronization

Lab notes and implementation plan for timing and synchronisation of the SOUK
MKID readout.

*2026-04-23, SR*

## 1. Overview

Every accumulated data sample is tagged with a 64-bit timestamp. The tag is
the Unix time (UTC) at the moment of final accumulation, multiplied by the FPGA
fabric clock frequency (307,200,000 Hz).

Two independent clock domains are involved:

- **FPGA fabric clock** — derived from the RFSoC ADC/DAC sample clock, which
  in turn is synthesised from a 10 MHz OCXO shared between all RFSoCs on a
  given telescope.
- **PTP clock** — absolute UTC reference distributed by a networked PTP
  grandmaster, disciplined to the Ethernet PHY hardware clock on each RFSoC.

These two clocks are **not phase-locked**. The FPGA clock provides
short-term stability and a common phase reference between boards; PTP
provides the absolute UTC anchor.

## 2. Clock chain

```
        10 MHz OCXO ───┐
                       ├──> RFSoC sample clock ──> FPGA fabric clock (307.2 MHz)
       telescope ref? ─┘                                       │
                                                               v
                                                 accumulator timestamp counter
                                                               ^
   PTP grandmaster ──> ethernet PHC ──> PPS edge ──────────────┘
                       (ptp4l)          (arms / releases timestamp reset)
                           │
                           v
                     linux system clock
                       (phc2sys)
```

## 3. PTP and PPS

Each RFSoC runs `linuxptp`:

- `ptp4l -i end0 -m -s -2` aligns the on-board NIC's physical hardware
  clock (PHC) to the PTP grandmaster.
- `phc2sys -s /dev/ptp1 -m -s -2` steers the Linux system clock to the
  same PHC.

The PHC timestamp itself is **not** fed into the firmware. Instead a 1 PPS
signal is derived from the PHC and routed to the firmware as a sync
reference.

The firmware also routes the PPS signal out to a GPIO pin, so it can be
probed with an oscilloscope for debugging (e.g. to verify PPS integrity,
check cable/routing, or compare PPS edges between boards).

### Firmware sync procedure

1. Software reads the current PTP time and computes the next upcoming UTC
   integer second.
2. Software preloads the firmware `telescope_time` register with that
   value, scaled by the fabric clock frequency.
3. The firmware is held in reset until the next PPS edge arrives.
4. On the PPS edge the reset is released and the timestamp counter begins
   counting from the preloaded value, aligned to UTC seconds at the PPS
   transition.

## 4. FPGA/PTP drift

Because the FPGA clock and PTP clock are independent, their phase
relationship drifts over time. The firmware measures the offset by
counting FPGA clock cycles between consecutive PPS pulses and reporting
the deviation from the nominal count.

### Preliminary measurement

Setup: PTP sourced from a desktop PC (free-running, not GPS-disciplined),
FPGA clock from the on-board 12.8 MHz crystal (i.e. **not** the readout system's
OCXO).

- Measured offset: **59.556 μs ± 0.579 μs per second** over 60 one second intervals, FPGA clock running faster than PTP.
- Extrapolated drift: ≈ **3 ms per minute**.

This is an upper-bound scenario. The measurement must be repeated with:

1. The readout OCXO driving the FPGA clock.
2. A GPS-disciplined PTP grandmaster.

These two changes should significantly reduce drift and give a realistic
estimate for:

- Required resynchronisation cadence during normal operation.
- Holdover accuracy if the PTP grandmaster fails.

## 5. Open questions and work items

- [ ] Repeat drift measurement with readout OCXO + GPS-disciplined PTP.
- [ ] Decide a resync cadence threshold (e.g. "resync if |drift| >
      X μs/s" or "resync every N minutes").
- [ ] Decide PTP grandmaster hardware + failover strategy.
- [ ] Characterise holdover: how long can we run without PTP before
      timestamps are unusable for the science case?

### Test procedures to design

Two distinct test regimes are needed, because resync is intrusive and
long-duration drift measurements are not:

**During streaming (intrusive / short):** characterise a single resync
event while data is actively flowing to a client. Specifically:

- Wall-clock duration of a `sync.arm → PPS edge → reset release` cycle
  (worst case ≈ 1 s, since it waits for the next PPS).
- Sample-count and timestamp discontinuity across the resync event as
  observed in the live stream.
- Any dropped accumulation frames, error flags, or gaps in
  `acc_cnt`/`telescope_time` reported by the stream parser.
- Recovery time before downstream tone data is valid again.

Goal: quantify the operational cost of a resync so the resync cadence
decision above can trade drift budget against streaming interruption.

**Between observations (non-intrusive / long):** characterise drift and
holdover with no streaming load. Specifically:

- Log `r.sync.get_drift()` and `r.sync.get_tt_of_sync()` at a fixed
  cadence (e.g. 1 Hz) over hours-to-days windows.
- Repeat under varied conditions: fresh sync, post-PTP-dropout, across
  a thermal cycle, across the two different FPGA clock sources
  (crystal vs. OCXO), and with/without GPS-disciplined PTP.
- Record grandmaster offset (`pmc ... GET TIME_STATUS_NP`) and
  `phc2sys` offset in parallel so drift can be attributed to FPGA
  clock vs. PTP chain.

Goal: populate the drift/holdover numbers that §4 currently only has a
preliminary upper bound for, and establish when a sync is required
purely from telemetry (without scope inspection).

These logging hooks are already enabled by the `sync` section in
Track B of §6 — a simple polling script against
`client.get_info(sections=['sync'])` is sufficient.

---

## 6. Implementation plan (software changes)

The required software work splits into three independent tracks that can
land separately.

### Track A — PTP services as systemd units

**Goal:** `ptp4l` and `phc2sys` always run, start at boot, restart on
failure, and are controllable through the same tooling as the readout
server.

- Add two new service files to
  [src/souk_readout_tools/data/daemon/](../src/souk_readout_tools/data/daemon/):
  - `ptp4l.service` — runs `ptp4l -i end0 -m -s -2` (interface name and
    flags should be parameterised via an `EnvironmentFile`).
  - `phc2sys.service` — runs `phc2sys -s /dev/ptp1 -m -s -2`, with
    `After=ptp4l.service` and `Requires=ptp4l.service`.
- Update
  [install_systemd_service.sh](../src/souk_readout_tools/data/daemon/install_systemd_service.sh)
  and
  [remove_systemd_service.sh](../src/souk_readout_tools/data/daemon/remove_systemd_service.sh)
  to handle the two new units alongside `readout_server*.service`.
- Update
  [readout_server.service](../src/souk_readout_tools/data/daemon/readout_server.service)
  and
  [readout_server_1.service](../src/souk_readout_tools/data/daemon/readout_server_1.service)
  to add `After=phc2sys.service` so the readout server only starts once
  system time is disciplined.
- Update
  [src/souk_readout_tools/server/server_scripts/](../src/souk_readout_tools/server/server_scripts/)
  entry points (`souk-enable-daemon`, `souk-disable-daemon`,
  `souk_restart_daemon`) to list and act on the full set of units.
- Document the new units in the daemon
  [readme](../src/souk_readout_tools/data/daemon/readme).

Because these are RFSoC-only concerns, everything lives under the
existing server-side install path and is gated by
`INSTALL_SERVER=true` in [setup.py](../setup.py) — no new client
dependencies.

### Track B — Expose sync / PTP state via `get_info()`

**Goal:** a single `sync` section in the structured `get_info()` response
that surfaces both firmware-side sync state and host-side PTP state.

Changes in [readout_server.py](../src/souk_readout_tools/server/readout_server.py):

- Add `'sync'` to `DEFAULT_INFO_SECTIONS`
  ([readout_server.py:882](../src/souk_readout_tools/server/readout_server.py#L882)).
- Register a `_info_sync` dispatcher
  ([readout_server.py:901-917](../src/souk_readout_tools/server/readout_server.py#L901-L917))
  that delegates to `firmware_lib.info_sync(self.r)`.
- Include the latest sync snapshot inside `health_check()` so the
  existing health endpoint can flag a stale or drifting sync without a
  full `get_info()` call.

Changes in [firmware_lib.py](../src/souk_readout_tools/firmware_lib.py):

- New `info_sync(r)` helper, placed alongside the other `info_*`
  functions near
  [firmware_lib.py:789](../src/souk_readout_tools/firmware_lib.py#L789).
  Proposed payload:

  ```python
  {
      'ready': bool,                   # pipeline initialised and r.sync usable
      'firmware': {
          'sync_delay': int,           # r.sync.get_delay()
          'drift_fpga_clocks': int,    # r.sync.get_drift()
          'drift_seconds': float,      # drift_fpga_clocks / fpga_clk_hz
          'tt_of_last_sync': int,      # r.sync.get_tt_of_sync()
          'tt_of_last_sync_unix_s': float,
          'seconds_since_sync': float,
      },
      'ptp': {
          'ptp4l_active': bool,        # systemctl is-active ptp4l
          'phc2sys_active': bool,
          'port_state': str,           # from `pmc ... GET PORT_DATA_SET`
          'grandmaster_identity': str, # from `pmc ... GET PARENT_DATA_SET`
          'master_offset_ns': int,     # from `pmc ... GET TIME_STATUS_NP`
          'last_sync_age_s': float,
      },
  }
  ```

- The firmware sub-block reads exclusively from `r.sync` — no network
  traffic, safe to call during streaming.
- The PTP sub-block wraps `pmc` / `systemctl` calls in a helper
  (`_query_ptp_state()`) that returns `None` values on any failure so
  the section degrades gracefully on benches without PTP.
- Cache the most recent `pmc` result for ~1 s so rapid
  `health_check()` polling does not spawn excessive subprocesses.

Changes in [readout_client.py](../src/souk_readout_tools/client/readout_client.py):

- No code changes required — `get_info(sections=['sync'])` works through
  the generic dispatcher at
  [readout_client.py:542](../src/souk_readout_tools/client/readout_client.py#L542).
- Add a convenience property `client.sync_status` that returns the
  `sync` section, mirroring the pattern used for other sections if one
  exists.

### Track C — Config additions

Add a top-level `timing` block to
[template_config.yaml](../src/souk_readout_tools/data/config/template_config.yaml)
so per-deployment parameters live with the rest of the config:

```yaml
timing:
  ptp_interface: end0
  ptp_phc_device: /dev/ptp1
  ptp_grandmaster_expected: "aa:bb:cc.ff.fe.dd:ee:ff"  # optional sanity check
  resync_drift_threshold_seconds: 1.0e-3               # trigger warning above this
  resync_interval_s: null                              # null = manual only
```

Initial consumers:

- `info_sync` compares `port_state` / `grandmaster_identity` against the
  expected values and flags a mismatch.
- A later automated resync policy (not in this change set) can read
  `resync_*` fields.

### Suggested ordering

1. Track A (services) — unblocks everything downstream and is
   independent of firmware changes.
2. Track C (config schema) — tiny, lets Tracks A and B reference
   consistent field names.
3. Track B (`info_sync` + health integration) — builds on A and C.

Each track can ship as a single PR against `dev/v1.1.x`.

### Out of scope for this change set

- Automatic resync triggered by drift thresholds.
- A dedicated client-side dashboard for timing health.
- Cross-board timestamp alignment verification.
- 

These should be tracked as separate work once the monitoring surface
from Track B is in place and the drift measurements from §4 have been
repeated.