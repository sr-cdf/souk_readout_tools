#!/usr/bin/env python3
"""Is the board really running at the clock the software thinks it is?

Counts two things against the ARM's clock for a few seconds, per pipeline:

- accumulations (``acc_cnt``), against ``get_sample_rate()``;
- telescope-time ticks (the accumulator's TT, ``adc_clk / 8``), against the
  configured ADC clock.

Both ratios should be 1. On 2026-09-22, after a reboot, both were 13/12 (ADC
~2268 MHz where 2457.6 was configured, clock_source external, every PLL
"locked"): every tone was 8% low and every rate 8% slow, and nothing else said
so. Run it after a reboot or a clock_source change, before trusting any rate.

Read-only. Servers must be UP (they bring up the clocks -- a read of unclocked
fabric hangs the board). ``--uio`` needs no root:

    taskset -c 1 ~/py3.12-venv/bin/python clock_rate_check.py --uio
"""
import argparse
import logging
import time

logging.disable(logging.WARNING)
from souk_readout_tools import firmware_lib as fl  # noqa: E402

FW = "/home/casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pipelines', type=int, nargs='+', default=[0, 1])
    ap.add_argument('--seconds', type=float, default=2.0)
    ap.add_argument('--uio', action='store_true', help='map through /dev/uio0, no root')
    args = ap.parse_args()
    for pid in args.pipelines:
        rf = fl.create_fast_readout_interface(FW, pid, uio=args.uio)
        p = fl.get_fast_read_params(rf)
        c0, tt0, t0 = fl.read_acc_cnt_fast(p), fl.read_tt_fast(p), time.perf_counter()
        time.sleep(args.seconds)
        c1, tt1, t1 = fl.read_acc_cnt_fast(p), fl.read_tt_fast(p), time.perf_counter()
        dt = t1 - t0
        rate = ((c1 - c0) % 2**32) / dt
        adc = 8 * (tt1 - tt0) / dt
        nominal = fl.get_sample_rate(rf)
        ok = abs(rate / nominal - 1) < 1e-3 and abs(adc / rf.adc_clk_hz - 1) < 1e-3
        print(f"pipeline {pid}: accumulations {rate:.2f} Hz (says {nominal:.2f}, "
              f"x{nominal / rate:.5f}); ADC from TT {adc / 1e6:.2f} MHz "
              f"(configured {rf.adc_clk_hz / 1e6:.2f}) -> {'OK' if ok else 'MISMATCH'}")


if __name__ == '__main__':
    main()
