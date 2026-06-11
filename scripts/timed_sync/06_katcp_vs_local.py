#!/usr/bin/env python3
"""Compare KATCP vs local (/dev/mem) transport for the sync-block reads.

These scripts run ON the RFSoC. By default souk_mkid_readout talks to the FPGA
over KATCP to localhost, which adds per-register round-trip latency through the
katcp server stack. Passing local=True swaps in casperfpga.LocalMemTransport,
which reads the registers directly via /dev/mem (root) -- much lower latency.

This measures, for BOTH transports:
  * per-register read latency (time per read_uint of ext_sync_tt_lsb)
  * the PPS-latched TT offset from the integer-second boundary
to see whether the reported offsets change much between the two. The boundary
offset is firmware-internal so it should be transport-INDEPENDENT; the read
latency is where they differ. (local=True needs root; 01_strobe must be running.)
"""
import time
print("importing souk_mkid_readout (a few seconds)...")
from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
PIPELINE_ID = 0
CLK_HZ = 307_200_000
N_READS = 200          # reads to average for the latency figure


def boundary_off_us(tt):
    rem = (tt % CLK_HZ) / CLK_HZ
    if rem > 0.5:
        rem -= 1.0
    return rem * 1e6


def bench(local):
    label = "local (/dev/mem)" if local else "KATCP (localhost)"
    print(f"\n=== {label} ===")
    print(f"  building interface (local={local})...")
    sync = SoukMkidReadout("localhost", configfile=FW_CONFIG,
                           pipeline_id=PIPELINE_ID, local=local).sync

    # per-register read latency: time Sync.read_uint('ext_sync_tt_lsb')
    t0 = time.perf_counter()
    for _ in range(N_READS):
        sync.read_uint("ext_sync_tt_lsb")
    per_read_us = (time.perf_counter() - t0) / N_READS * 1e6
    print(f"  Sync.read_uint('ext_sync_tt_lsb') latency: "
          f"{per_read_us:.1f} us/read (mean of {N_READS})")

    # PPS-latched TT from ext_sync_tt_msb/lsb, offset from the integer second
    tt = (sync.read_uint("ext_sync_tt_msb") << 32) + sync.read_uint("ext_sync_tt_lsb")
    off = boundary_off_us(tt)
    print(f"  ext_sync_tt (Sync.read_uint ext_sync_tt_msb/lsb) = {tt} clks "
          f"-> {off:+.3f} us from second boundary")
    return per_read_us, off


lat_k, off_k = bench(local=False)
lat_l, off_l = bench(local=True)

print("\n=== comparison (both reading ext_sync_tt via Sync.read_uint) ===")
print(f"  read latency:    KATCP {lat_k:.1f} us  vs  local {lat_l:.1f} us  "
      f"({lat_k/lat_l:.1f}x faster local)" if lat_l else "")
print(f"  ext_sync_tt boundary offset: KATCP {off_k:+.3f} us  vs  local {off_l:+.3f} us  "
      f"(diff {off_k - off_l:+.3f} us)")
print("  -> boundary offset should be ~transport-independent (firmware-internal); "
      "any difference is read-timing/jitter, not real misalignment.")
