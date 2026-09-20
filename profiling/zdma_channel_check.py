#!/usr/bin/env python3
"""One-shot, as root, servers may keep running (neither uses this channel yet):
read one pipeline's accumulator through a given DMA channel + u-dma-buf and
compare it bit for bit with a devmem read of the same accumulation."""
import sys, time, logging, numpy as np
logging.disable(logging.WARNING)
from souk_readout_tools import firmware_lib as fl
from souk_readout_tools.zdma import ZdmaReader, UdmabufBuffer
FW = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
pid, bufname, ch = int(sys.argv[1]), sys.argv[2], int(sys.argv[3])
rf = fl.create_fast_readout_interface(FW, pid)
p = fl.get_fast_read_params(rf)
from casperfpga import transport_localmem as _tlm
# phys_base is only filled in when ZDMA is enabled for the params; compute it the same way
mm, base, phys = rf.accumulators[0].host.transport.axil_mm, p['base_addr'], _tlm.AXIL_OFFSET + p['addrs'][0]
print(f"pipeline {pid}: DMA channel {ch} (0x{0xFD500000 + ch * 0x10000:08X}), {bufname}", flush=True)
reader = ZdmaReader(cached_buf=UdmabufBuffer(bufname), dma_base=0xFD500000 + ch * 0x10000)
print("  reader created, channel powered", flush=True)
result = None
for attempt in range(50):
    c0 = fl._blocking_wait_for_acc_fast(p)
    dev = np.frombuffer(mm[base:base + 8000], dtype='<i4').copy()
    z = np.frombuffer(reader.read_view(phys, 8000), dtype='<i4').copy()
    c1 = fl.read_acc_cnt_fast(p)
    if c0 == c1:
        result = (attempt, int(np.count_nonzero(dev)), int(np.count_nonzero(dev != z)))
        break
if result is None:
    print("  FAIL: accumulation changed during every attempt"); sys.exit(1)
print(f"  same accumulation (attempt {result[0]}): {8000 // 4} words, {result[1]} non-zero, {result[2]} differing -> {'PASS' if result[2] == 0 else 'FAIL'}")
t = np.empty(2000)
for i in range(len(t)):
    t0 = time.perf_counter(); reader.read_view(phys, 8000); t[i] = time.perf_counter() - t0
print(f"  DMA read 8000 B p50 {np.median(t)*1e6:.1f} us, p99.9 {np.percentile(t, 99.9)*1e6:.1f} us")
