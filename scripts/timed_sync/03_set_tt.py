#!/usr/bin/env python3
"""Load the telescope time and confirm it aligns to the PPS edge (strobe running).

Firmware registers used (block p{pid}_sync, see souk_mkid_readout/blocks/sync.py):
  int_tt_load_msb/lsb  -- value loaded into the internal TT counter on next sync
  ctrl[ext_load]       -- arm the load to latch on the next PPS
  ext_sync_tt_msb/lsb  -- internal TT latched at the last PPS (read-back)
sync.py functions:
  Sync.update_internal_time(clk_hz, sync_period)
      -> measures the sync period, computes the next-PPS epoch TT, writes
         int_tt_load_* and arms ctrl[ext_load] via Sync.load_internal_time()
  Sync.get_tt_of_ext_sync()  -> reads ext_sync_tt_* (the latched TT)

We force sync_period=CLK_HZ because the auto-detect can mis-read it; the load is
identical either way. Then we compare the latched TT to the integer-second
boundary, the system clock, and "true" time.

The PRECISE alignment metric is TT-mod-CLK_HZ vs the integer-second boundary:
both sides are firmware-internal so it resolves to ~us. A comparison against
time.time() is only good to ~ms (the latched TT is from the PPS edge but the
wall-clock read happens ms later + KATCP latency), so it serves only as a
coarse whole-second sanity check -- it does NOT reflect the real sub-us sync.
For system-clock-vs-grandmaster accuracy (sub-us when PTP-locked) see 05's
ptp_master_offset_ns. We deliberately do NOT query an NTP server (network
round-trip would add tens of ms and tell us less).
"""
import time
print("importing souk_mkid_readout (a few seconds)...")
from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
PIPELINE_ID = 0
CLK_HZ = 307_200_000

print("building the readout connection interface "
      "(KATCP + parsing .fpg + building blocks, a few seconds)...")
sync = SoukMkidReadout("localhost", configfile=FW_CONFIG, pipeline_id=PIPELINE_ID).sync
print(f"connected (pipeline {PIPELINE_ID}); clk_hz = {CLK_HZ}\n")

print("loading telescope time (update_internal_time, forcing 1 s period)...")
sync.update_internal_time(clk_hz=CLK_HZ, sync_period=CLK_HZ)

tt, _ = sync.get_tt_of_ext_sync()          # TT latched at the last real PPS
tt_s = tt / CLK_HZ

# The PRECISE alignment check: the TT latched at a PPS edge should sit on an
# integer second. Both sides are firmware-internal, so this resolves to ~us --
# it is the meaningful sub-us metric.
off_boundary = (tt % CLK_HZ) / CLK_HZ
if off_boundary > 0.5:
    off_boundary -= 1.0

# Coarse sanity: the latched TT corresponds to the right whole second (vs the
# system clock, which chrony slews to the grandmaster). NOTE this is only good to
# ~ms: tt is latched AT the PPS edge but time.time() is read now (some ms later,
# plus KATCP read latency), so the difference mostly measures "how long ago the
# last PPS was", NOT clock error. For the real sub-us number, see the boundary
# offset above; for system-vs-grandmaster accuracy, see 05 (ptp_master_offset_ns).
now_sys = time.time()

print(f"PPS-latched TT = {tt} clks = {tt_s:.6f} s ({time.ctime(tt_s)})")
print(f"  offset from integer second:  {off_boundary*1e6:+.3f} us   <- the precise check")
print(f"  whole-second matches system: {abs(tt_s - now_sys) < 1.0}  "
      f"(coarse; |dt|={abs(tt_s - now_sys):.3f}s -- limited by read timing, not clock error)")
print("PASS: TT on the second boundary and tracking the right UNIX second"
      if abs(off_boundary) < 50e-6 and abs(tt_s - now_sys) < 1.5
      else "FAIL: TT not aligned")
