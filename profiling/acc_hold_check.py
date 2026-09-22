#!/usr/bin/env python3
"""Does the accumulator hold an accumulation unchanged until acc_cnt moves?

The stream loop's late read (``fast_read_params['catch_up']``) reads an
accumulation that landed while the loop was busy, as long as the read ends
``ACC_CATCH_UP_GUARD_S`` before the next one is due. That is only right if the firmware
leaves the output BRAM alone until the next accumulation is dumped, and if
``acc_cnt`` moves no later than that dump. This checks both, read-only.

For each fraction f of the period: watch an accumulation land, read it at
once (A), wait until f of the period has passed, read it again (B), then read
the counter. With the counter unchanged, A must equal B word for word. A
mismatch at large f only means the dump starts before the counter moves, and
the margin must cover that window; a mismatch at small f would mean the BRAM
is rewritten during the accumulation, and the late read cannot be used.

It needs a live signal (pipeline 0 through the RF loopback): if successive
accumulations are identical, nothing can be detected, and it says so.

One-shot, read-only, servers must be UP (they bring up the clocks -- a read of
unclocked fabric hangs the board) with tones set; takes ~30 s. Do not run it
during a loss measurement. ``--uio`` maps the AXI window through /dev/uio0, so
no root is needed (casper is in the uio group):

    ~/py3.12-venv/bin/python acc_hold_check.py 0 --uio [--words 2000]
    sudo ~/py3.12-venv/bin/python acc_hold_check.py 0      # /dev/mem
"""
import argparse
import logging
import time

import numpy as np

logging.disable(logging.WARNING)
from souk_readout_tools import firmware_lib as fl  # noqa: E402

FW = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"
FRACTIONS = (0.05, 0.25, 0.5, 0.7, 0.8, 0.85, 0.9, 0.93, 0.96, 0.98)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pipeline', type=int)
    ap.add_argument('--words', type=int, default=2000, help='int32 words read (2 per tone)')
    ap.add_argument('--reps', type=int, default=300, help='comparisons per fraction')
    ap.add_argument('--uio', action='store_true', help='map through /dev/uio0, no root')
    args = ap.parse_args()

    rf = fl.create_fast_readout_interface(FW, args.pipeline, uio=args.uio)
    p = fl.get_fast_read_params(rf)
    mm, base = rf.accumulators[0].host.transport.axil_mm, p['base_addr']
    nbytes = 4 * args.words

    def read():
        return np.frombuffer(mm[base:base + nbytes], dtype='<i4').copy()

    def arrival():
        """Spin until the counter moves; returns (count, time it moved).

        Self-contained on purpose -- it must test the firmware, not whichever
        version of the wait is installed on the board.
        """
        c0 = fl.read_acc_cnt_fast(p)
        while True:
            c = fl.read_acc_cnt_fast(p)
            if c != c0:
                return c, time.perf_counter()

    t = [arrival()[1] for _ in range(21)]
    period = float(np.median(np.diff(t)))
    print(f"pipeline {args.pipeline}: period {period * 1e6:.1f} us, {args.words} words")

    # Is there anything to detect? Successive accumulations must differ.
    arrival(); a = read()
    arrival(); b = read()
    live = int(np.count_nonzero(a != b))
    print(f"  successive accumulations differ in {live} of {args.words} words"
          + ("" if live else "  -> NO LIVE SIGNAL, the check below cannot see anything"))

    print(f"  {'fraction':>8} {'at us':>7} {'compared':>9} {'mismatch':>9} {'cnt moved':>10}")
    for f in FRACTIONS:
        compared = mismatched = moved = 0
        for _ in range(args.reps):
            c0, t_land = arrival()
            A = read()
            target = t_land + f * period
            while time.perf_counter() < target:
                pass
            B = read()
            c1 = fl.read_acc_cnt_fast(p)
            if c1 != c0:
                moved += 1
                continue
            compared += 1
            if not np.array_equal(A, B):
                mismatched += 1
        print(f"  {f:8.2f} {f * period * 1e6:7.0f} {compared:9d} {mismatched:9d} {moved:10d}")
    print("PASS if every mismatch is 0. A mismatch only where B ends within"
          " ACC_CATCH_UP_GUARD_S of the next arrival means the guard must grow;"
          " one at small fractions means catch_up must stay off.")


if __name__ == '__main__':
    main()
