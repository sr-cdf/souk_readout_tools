#!/usr/bin/env python3
"""Measure how fast the firmware TT drifts against the PPS after a single load.

06 showed the PPS-latched TT offset from the integer second GROWS with time since
the last update_internal_time (-1 us fresh -> ~-10 us after 9 s -> ~-190 us much
later). That is the FPGA fabric clock running at a slightly different rate than the
PHC-disciplined PPS -- a free-running TT counter drifts ~1 ppm. This script
quantifies it: load the TT once, then sample the boundary offset every few seconds
and fit the drift rate (ppm).

Why it matters for the multi-board timed sync: if every board loads TT once at
startup and fires set_timed_sync minutes later, each TT has drifted independently
(~1 us/s), so they will NOT fire on the same instant. The TT must be re-disciplined
(update_internal_time) shortly before arming, OR tracked continuously.

The drift rate measured here is mostly this RFSoC's fabric oscillator vs the PHC,
so it is fairly independent of grandmaster quality. (Note: this system's GM is a
software GM on a workstation, not a hardware/GPS GM -- so the ABSOLUTE TT offset
from true UTC may be ms-level and is NOT a fault; what matters for multi-board sync
is that all boards share the same GM and that per-board drift is re-disciplined.)

Firmware registers / sync.py (block p{pid}_sync, souk_mkid_readout/blocks/sync.py):
  ext_sync_tt_msb/lsb  -- TT latched at the last PPS  (the offset we track)
  drift_msb/lsb        -- Sync.get_drift(): TT-since-last-arm vs PPS-updated TT,
                          in FPGA clocks (secondary; meaningful only after an arm)
Uses local=True (/dev/mem) -- 06 showed it is ~35x lower read latency than KATCP
and gives the same offset, so it minimises read jitter. Needs root; strobe running.
"""
import time
print("importing souk_mkid_readout (a few seconds)...")
from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
PIPELINE_ID = 0
CLK_HZ = 307_200_000
SAMPLES = 10           # number of offset samples
INTERVAL_S = 3.0       # seconds between samples

print("building the readout connection interface (local=True, /dev/mem)...")
sync = SoukMkidReadout("localhost", configfile=FW_CONFIG,
                       pipeline_id=PIPELINE_ID, local=True).sync
print(f"connected (pipeline {PIPELINE_ID}); clk_hz = {CLK_HZ}\n")

def latched_tt():
    return (sync.read_uint("ext_sync_tt_msb") << 32) + sync.read_uint("ext_sync_tt_lsb")

def boundary_off_us(tt):
    rem = (tt % CLK_HZ) / CLK_HZ
    if rem > 0.5:
        rem -= 1.0
    return rem * 1e6

print("loading TT once: Sync.update_internal_time() -> int_tt_load_*, ctrl[ext_load]...")
sync.update_internal_time(clk_hz=CLK_HZ, sync_period=CLK_HZ)
print(f"\nsampling ext_sync_tt (Sync.read_uint ext_sync_tt_msb/lsb) offset from the "
      f"integer second every {INTERVAL_S:.0f} s x{SAMPLES} "
      f"(grows if the TT is drifting). NOTE Sync.get_drift() (drift_msb/lsb) is "
      f"shown for reference but is FROZEN/meaningless here -- it measures drift vs a "
      f"counter reset from the last arm_sync(), which we do NOT call (see 08 for the "
      f"arm-based drift test):")

t0 = time.time()
ts, offs = [], []
for i in range(SAMPLES):
    tt = latched_tt()
    off = boundary_off_us(tt)
    el = time.time() - t0
    ts.append(el); offs.append(off)
    try:
        drift = sync.get_drift()
        dtxt = f"  Sync.get_drift()={drift} clks (frozen w/o arm)"
    except Exception as e:
        dtxt = f"  Sync.get_drift()=<{type(e).__name__}>"
    print(f"  t={el:5.1f}s  ext_sync_tt offset={off:+8.3f} us{dtxt}")
    time.sleep(INTERVAL_S)

# Drop the first sample from the fit: the t=0 read can catch a stale latch from
# before update_internal_time fully settled (seen as a ~100+ us outlier).
ts_fit, offs_fit = ts[1:], offs[1:]

# linear fit offset(us) vs t(s): slope = drift rate. us/s == ppm (1e-6 s per s).
n = len(ts_fit)
mt = sum(ts_fit)/n; mo = sum(offs_fit)/n
den = sum((t-mt)**2 for t in ts_fit)
slope = sum((t-mt)*(o-mo) for t, o in zip(ts_fit, offs_fit)) / den if den else float("nan")

print(f"\ndrift rate = {slope:+.3f} us/s = {slope:+.3f} ppm "
      f"(fit over samples 1..{len(ts)-1}, dropping the t=0 settling outlier; "
      f"span {offs_fit[-1]-offs_fit[0]:+.1f} us over {ts_fit[-1]-ts_fit[0]:.0f} s)")
print("=> after T seconds the TT is ~|rate|*T us off the PPS. For multi-board sync, "
      "re-run update_internal_time shortly before arming set_timed_sync.")
