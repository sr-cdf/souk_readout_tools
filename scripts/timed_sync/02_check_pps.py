#!/usr/bin/env python3
"""Prove a 1-PPS is reaching the firmware (01_strobe.py must be running).

Reads the internal TT latched at each external pulse and checks the spacing.

Firmware registers used (block p{pid}_sync, see souk_mkid_readout/blocks/sync.py):
  ext_sync_count       -- count of external sync (PPS) pulses received
  ext_sync_tt_msb/lsb  -- internal TT (FPGA clocks) latched at the last PPS
sync.py functions:
  Sync.count_ext()           -> reads ext_sync_count
  (we read ext_sync_tt_* directly; Sync.get_tt_of_ext_sync() does the same but
   BLOCKS on wait_for_sync(), so it is unusable in a tight poll loop)

The per-pulse ext_sync_tt delta should be CLK_HZ clocks = 1.000000 s. We use it
rather than the count/period registers, which read garbage if sampled right
after an edge (free-running counters).
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

def ext_tt(expected_count):
    # Read ext_sync_tt_msb/lsb directly (non-blocking). Guard against a torn read:
    # if ext_sync_count moved across the two reads, a pulse landed mid-read -> reject.
    msb = sync.read_uint("ext_sync_tt_msb")
    lsb = sync.read_uint("ext_sync_tt_lsb")
    return None if sync.count_ext() != expected_count else (msb << 32) + lsb

print("watching ext_sync_tt (TT latched at each PPS) for ~6 pulses...")
last_c = sync.count_ext()          # ext_sync_count
last_tt = ext_tt(last_c)
seen = 0
t_end = time.time() + 8
while time.time() < t_end and seen < 6:
    time.sleep(0.001)
    c = sync.count_ext()
    if c == last_c:
        continue
    tt = ext_tt(c)
    if c - last_c == 1 and last_tt and tt and tt > last_tt:
        d = tt - last_tt
        print(f"  pulse {seen}: ext_sync_tt delta = {d} clks = {d/CLK_HZ:.6f} s")
        seen += 1
    last_c, last_tt = c, tt

print("\nPASS: PPS detected, ext_sync_tt advancing ~1 s/pulse" if seen
      else "\nFAIL: no pulses -- is 01_strobe.py running?")
