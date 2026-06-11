#!/usr/bin/env python3
"""Measure TT-vs-PPS drift using the firmware's intended monitor: arm + get_drift.

This is the Sync block's documented step 4: after disciplining the TT, call
arm_sync() to set a reference (a counter reset on the arm), then periodically
read Sync.get_drift() -- the FPGA clocks between that arm-reset time and the
PPS-updated internal TT. It should grow ~linearly as the fabric clock drifts vs
the PPS, giving an independent cross-check on 07's ext_sync_tt-offset method.

(In 07, get_drift was FROZEN because no arm had set the reference. Here we arm
first, so it becomes meaningful.)

Firmware registers / sync.py (block p{pid}_sync, souk_mkid_readout/blocks/sync.py):
  Sync.update_internal_time()  -> load epoch TT (int_tt_load_*, ctrl[ext_load])
  Sync.arm_sync()              -> pulse ctrl[arm_sync_out]; sets the drift reference
  Sync.get_drift()             -> reads drift_msb/lsb = (arm-reset TT) - (PPS TT),
                                  in FPGA clocks
Uses local=True (/dev/mem). Needs root; 01_strobe must be running.
"""
import time
print("importing souk_mkid_readout (a few seconds)...")
from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
PIPELINE_ID = 0
CLK_HZ = 307_200_000
SAMPLES = 10
INTERVAL_S = 3.0

print("building the readout connection interface (local=True, /dev/mem)...")
sync = SoukMkidReadout("localhost", configfile=FW_CONFIG,
                       pipeline_id=PIPELINE_ID, local=True).sync
print(f"connected (pipeline {PIPELINE_ID}); clk_hz = {CLK_HZ}\n")

def safe_drift():
    # Sync.get_drift() re-reads if the value changes mid-read; may raise -> retry once.
    try:
        return sync.get_drift()
    except Exception:
        time.sleep(0.05)
        return sync.get_drift()

print("loading TT: Sync.update_internal_time() -> int_tt_load_*, ctrl[ext_load]...")
sync.update_internal_time(clk_hz=CLK_HZ, sync_period=CLK_HZ)

print("arming drift reference: Sync.arm_sync() -> ctrl[arm_sync_out]...")
sync.arm_sync()

d0 = safe_drift()
print(f"\nsampling Sync.get_drift() (drift_msb/lsb) every {INTERVAL_S:.0f} s x{SAMPLES} "
      f"(drift relative to the arm reset; should grow ~linearly):")

t0 = time.time()
ts, ds = [], []
for i in range(SAMPLES):
    d = safe_drift() - d0          # drift since the arm, in clocks
    el = time.time() - t0
    ts.append(el); ds.append(d)
    print(f"  t={el:5.1f}s  drift-since-arm = {d:+d} clks = {d/CLK_HZ*1e6:+.3f} us")
    time.sleep(INTERVAL_S)

# slope of drift(us) vs t(s) = drift rate; us/s == ppm. Drop the t=0 settling sample.
ts_fit, ds_us = ts[1:], [d/CLK_HZ*1e6 for d in ds[1:]]
n = len(ts_fit)
mt = sum(ts_fit)/n; md = sum(ds_us)/n
den = sum((t-mt)**2 for t in ts_fit)
slope = sum((t-mt)*(d-md) for t, d in zip(ts_fit, ds_us)) / den if den else float("nan")

print(f"\ndrift rate (arm/get_drift method) = {slope:+.3f} us/s = {slope:+.3f} ppm")
print("Compare with 07's ext_sync_tt method -- the two should agree if both are "
      "measuring the same fabric-vs-PPS drift.")
