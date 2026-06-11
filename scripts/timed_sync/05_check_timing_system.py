#!/usr/bin/env python3
"""Check the souk_readout_tools timing system lines up with the sync bring-up.

Queries the local timing-monitor socket (the same source the readout server uses)
and prints the fields a multi-board sync coordinator gates on: is PTP locked to a
grandmaster, is the board ready_for_firmware_sync, and how far is system time from
true. These are the preconditions for choosing a common TARGET_SEC in 04_timed_sync.

NOTE: this reads the readout-tools TIMING MONITOR (the PTP/PHC/NTP clock-discipline
side, via the /run/timing-monitor.sock unix socket and souk_readout_tools.timing) --
NOT the firmware sync block. It does NOT touch souk_mkid_readout / the FPGA. The
firmware sync state (tt_sync, timed_sync_countdown) is shown by 02-04; tying the two
together inside get_info('timing') is future work.

souk_readout_tools.timing.get_timing_summary() is exactly what the readout server's
get_info('timing') returns, so this previews the coordinator-facing timing view.
"""
print("querying timing monitor (/run/timing-monitor.sock)...")
from souk_readout_tools.timing import get_timing_summary

# get_timing_summary() reads the timing-monitor unix socket itself (it calls
# get_timing_status internally) and returns the coordinator-facing view --
# the SAME dict get_info('timing') serves.
summary = get_timing_summary()["summary"]

def show(k, unit=""):
    v = summary.get(k)
    print(f"  {k:28} {v}{unit}")

print("timing-monitor summary:")
show("state")
show("ready_for_firmware_sync")
show("absolute_time_verified")
show("ptp_fresh"); show("ptp_stable"); show("ptp_healthy")
show("active_source_type"); show("active_source_name")
show("ptp_master_offset_ns", " ns")
show("system_time_offset_s", " s")
show("estimated_abs_error_s", " s")

ready = summary.get("ready_for_firmware_sync") and summary.get("state") == "locked_to_gm"
print("\nREADY for a coordinated timed sync" if ready else
      "\nNOT ready: need state=locked_to_gm and ready_for_firmware_sync=True "
      "on every board before choosing a common TARGET_SEC.")
