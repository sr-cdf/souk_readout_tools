#!/usr/bin/env python3
"""
Minimal reproduction for realtimeradio: the accumulator's TT vs the sync block's and
the mixer's, through the documented Sync flow. Uses only souk_mkid_readout.

    python3 10_acc_tt_report.py HOST PIPELINE              # program + full sequence
    python3 10_acc_tt_report.py HOST PIPELINE --report-only  # just read the three TTs

HOST is the board running tcpborphserver (katcp, no root needed). The full sequence
REPROGRAMS the FPGA, so stop any readout server on that board first.

Sequence (sync.py's documented flow, plus acc_len so accumulations run):
  1. program(), initialize()
  2. mixer.set_acc_len(240); sync.sw_sync(mrst=True)
  3. sync.update_internal_time()                     -- TT load on the next PPS
  4. sync.set_timed_sync(next second + 5, mrst=True, wait=True)
After each step it prints every TT copy minus the host clock (PTP-disciplined):
  sync ext_sync_tt  -- TT latched at the last PPS (expect -1..0 s)
  mixer tt          -- mixer.read_tt()           (expect ~0)
  acc0 acc_tt       -- accumulators[0].read_tt() (expect within one accumulation, 0.4 ms)
"""
import sys
import time

from souk_mkid_readout import SoukMkidReadout

FW_CONFIG = '/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml'
CLK_HZ = 307_200_000      # fabric clock (ADC 2457.6 MHz / 8)
ACC_LEN = 240

host, pipeline = sys.argv[1], int(sys.argv[2])
report_only = '--report-only' in sys.argv[3:]


def report(r, label):
    now = time.time()
    ext = (r.sync.read_uint('ext_sync_tt_msb') << 32) + r.sync.read_uint('ext_sync_tt_lsb')
    mix = r.mixer.read_tt()
    acc = r.accumulators[0].read_tt()
    print(f'\n[{label}]  host clock {now:.3f}')
    for name, tt in (('sync ext_sync_tt', ext), ('mixer tt', mix), ('acc0 acc_tt', acc)):
        print(f'  {name:17s} {tt:>21d} ticks = {tt / CLK_HZ:16.3f} s   '
              f'minus host clock: {tt / CLK_HZ - now:+.6f} s')
    print(f'  acc_tt - mixer tt = {(acc - mix) / CLK_HZ:+.6f} s')


print(f'connecting to {host}, pipeline {pipeline} (katcp)...')
r = SoukMkidReadout(host, configfile=FW_CONFIG, pipeline_id=pipeline)
if report_only:
    report(r, 'current state')
    sys.exit()

print('1. program + initialize'); r.program(); r.initialize()
report(r, 'after program + initialize')

print(f'\n2. acc_len {ACC_LEN} + sw_sync(mrst=True)')
r.mixer.set_acc_len(ACC_LEN); r.sync.sw_sync(mrst=True); time.sleep(1)
report(r, 'after acc_len + sw_sync')

print('\n3. update_internal_time (TT load on the next PPS)')
r.sync.update_internal_time(clk_hz=CLK_HZ, sync_period=CLK_HZ); r.sync.wait_for_sync()
report(r, 'after TT load')

target = (int(time.time()) + 5) * CLK_HZ
print(f'\n4. set_timed_sync at TT {target} ({target / CLK_HZ:.0f}), mrst=True, wait=True')
r.sync.set_timed_sync(target, wait=True, mrst=True); time.sleep(1)
report(r, 'after timed sync')
