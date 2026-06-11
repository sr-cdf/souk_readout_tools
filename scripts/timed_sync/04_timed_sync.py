#!/usr/bin/env python3
"""Timed sync: reset, set TT once, then fire a sync at a precise future second.

This is the multi-board model: every board runs this with the SAME TARGET_SEC, so
all fire their sync on the same telescope-time edge.

Sequence:
  1. master reset (mrst) to clear the pipeline,
  2. wait a couple of seconds,
  3. set the telescope time (disciplined to the PPS),
  4. arm a timed sync at TARGET_SEC and wait for it to fire.
Strobe (01_strobe.py) must be running.

Firmware registers used (block p{pid}_sync, see souk_mkid_readout/blocks/sync.py):
  ctrl[mrst]              -- master reset of the sync/DSP pipeline
  int_tt_load_*, ctrl[ext_load]  -- TT load (via update_internal_time)
  timed_sync_time_msb/lsb -- target TT at which the timed sync should fire
  timed_sync_ctrl[en]     -- arm the timed-sync trigger
  timed_sync_countdown    -- FPGA clocks remaining until the timed sync fires
  tt_sync_msb/lsb         -- TT at which the last system sync occurred
sync.py functions:
  Sync.assert_mrst() / Sync.deassert_mrst()  -> pulse ctrl[mrst]
  Sync.update_internal_time(...)             -> load epoch TT (see 03_set_tt.py)
  Sync.set_timed_sync(tt, wait, mrst=False)  -> write timed_sync_time_*, arm
       timed_sync_ctrl[en], poll timed_sync_countdown until it fires
  Sync.get_tt_of_sync()   -> reads tt_sync_*
  Sync.get_time_to_sync() -> reads timed_sync_countdown
"""
import time
print("importing souk_mkid_readout (a few seconds)...")
from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
PIPELINE_ID = 0
CLK_HZ = 307_200_000
LEAD_S = 5            # fire this many whole seconds in the future
# TARGET_SEC = 1781100000   # set the SAME absolute epoch second on every board for multi-board sync

print("building the readout connection interface "
      "(KATCP + parsing .fpg + building blocks, a few seconds)...")
sync = SoukMkidReadout("localhost", configfile=FW_CONFIG, pipeline_id=PIPELINE_ID).sync
print(f"connected (pipeline {PIPELINE_ID}); clk_hz = {CLK_HZ}\n")

print("[1] master reset: Sync.assert_mrst()/deassert_mrst() -> ctrl[mrst]...")
sync.assert_mrst()
sync.deassert_mrst()

print("[2] waiting 2 s for the reset to settle...")
time.sleep(2.0)

print("[3] setting telescope time: Sync.update_internal_time() "
      "-> int_tt_load_*, ctrl[ext_load]...")
sync.update_internal_time(clk_hz=CLK_HZ, sync_period=CLK_HZ)
tt, _ = sync.get_tt_of_ext_sync()          # ext_sync_tt_*
print(f"    TT now ~{tt/CLK_HZ:.3f} s ({time.ctime(tt/CLK_HZ)})")

target_sec = int(tt / CLK_HZ) + LEAD_S    # or use the fixed TARGET_SEC above
target_tt = int(round(target_sec * CLK_HZ))
print(f"[4] Sync.set_timed_sync() -> timed_sync_time_*={target_tt}, "
      f"arm timed_sync_ctrl[en]; target second {target_sec} "
      f"({time.ctime(target_sec)}), {LEAD_S}s ahead; waiting for it to fire...")
sync.set_timed_sync(target_tt, wait=True, mrst=False)

print(f"    fired. tt_sync (tt_sync_*) = {sync.get_tt_of_sync()} clks  "
      f"(timed_sync_countdown now {sync.get_time_to_sync()} clks)")
print("done.")
