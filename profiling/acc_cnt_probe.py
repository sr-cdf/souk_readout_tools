#!/usr/bin/env python3
"""
Read one pipeline's accumulation counter in a tight loop and record every read
that cannot be right: a jump larger than the time since the last change allows,
a step backwards, or a read slower than --slow-us. Board-side, no root (UIO);
leave the pipeline running but not streaming.

On v7.12.1 the servers saw the counter read 17 high on a torn frame, then step
back (doc/v712-notes.md, "The 16-accumulation gaps"); this measures how often
that happens with nothing else in the loop.

Usage (one process per pipeline):
    ~/py3.12-venv/bin/python acc_cnt_probe.py --pipeline 0 --seconds 600 --out p0.json
"""
import argparse
import json
import logging
import os
import struct
import time

logging.disable(logging.WARNING)
from souk_readout_tools import firmware_lib as fl

FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pipeline', type=int, default=0)
    ap.add_argument('--seconds', type=float, default=600)
    ap.add_argument('--slow-us', type=float, default=200)
    ap.add_argument('--double-read', action='store_true',
                    help="use firmware_lib.read_acc_cnt_fast (reads until two agree) instead of one raw read")
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    rf = fl.create_fast_readout_interface(FW, args.pipeline, uio=True)
    p = fl.get_fast_read_params(rf)
    rate = float(fl.get_sample_rate(rf))
    period = 1 / rate
    mm, a = rf.accumulators[0].host.transport.axil_mm, p['acc_cnt_addr']

    def read():
        return struct.unpack('<I', mm[a:a + 4])[0]

    if args.double_read:
        read = lambda: fl.read_acc_cnt_fast(p)          # noqa: E731

    events, reads, changes = [], 0, 0
    prev, t_change = read(), time.perf_counter()
    t_end = t_change + args.seconds
    while True:
        t0 = time.perf_counter()
        c = read()
        t1 = time.perf_counter()
        reads += 1
        if t1 - t0 > args.slow_us * 1e-6:
            events.append(dict(kind='slow read', us=round((t1 - t0) * 1e6), cnt=c, prev=prev))
        if c != prev:
            step = (c - prev) & 0xFFFFFFFF
            allowed = (t1 - t_change) / period + 1.5      # accumulations that could have passed
            if step >= 1 << 31:
                events.append(dict(kind='backwards', step=step - (1 << 32), cnt=c, prev=prev,
                                   next=[read() for _ in range(3)]))
            elif step > allowed:
                events.append(dict(kind='jump', step=step, allowed=round(allowed, 1), cnt=c, prev=prev,
                                   next=[read() for _ in range(3)]))
            prev, t_change = c, t1
            changes += 1
        if t1 > t_end:
            break
    out = dict(pipeline=args.pipeline, double_read=args.double_read, rate=rate, seconds=args.seconds, reads=reads,
               changes=changes, events=events)
    with open(args.out, 'w') as f:
        json.dump(out, f, indent=1)
    kinds = {k: sum(e['kind'] == k for e in events) for k in ('jump', 'backwards', 'slow read')}
    print(f"pipeline {args.pipeline}{' (double read)' if args.double_read else ''}: {rate:.0f} Hz, {reads} reads, {changes} counter changes, {kinds}")
    for e in events[:10]:
        print('  ', e)


if __name__ == '__main__':
    main()
