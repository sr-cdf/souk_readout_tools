#!/usr/bin/env python3
"""
Board-side timing of the real accumulator read path, end to end (test B3).

Where access_size_scaling.py times a bare transport, this times the code the
readout server actually calls, so software and firmware changes both show:

  - katcp (tcpborphserver) vs LocalMem (/dev/mem) for the same control_sw
    calls: acc._read_bram() (whole 2048-channel buffer) and acc.get_acc_cnt().
    This is the comparison doc/accumulators.md started with, minus the wait:
    its get_new_spectra() numbers (116 ms vs 18.3 ms) were dominated by
    _wait_for_acc's 0.1 s poll period, not by the transport.
  - firmware_lib.read_accumulated_data_fast at 1, 64, 341, 1000, 2048 tones,
    with the wait-for-next-accumulation removed so only the read is timed.
    Whatever the installed souk_readout_tools does by default is what gets
    timed (v1.7.10: full-buffer reads; later: sparse reads).
  - the same via ZDMA, if this souk_readout_tools and image.ub support it.

Usage (on the board, root, readout servers stopped, tcpborphserver running):
    sudo ~/py3.12-venv/bin/python fast_path_timing.py --out b3.json
"""

import argparse
import inspect
import json
import logging
import os
import time

import numpy as np
from souk_readout_tools import firmware_lib as fl

logging.disable(logging.WARNING)
FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")
TONES = (1, 64, 341, 1000, 2048)


def pct(fn, reps, warmup=10):
    for _ in range(warmup):
        fn()
    t = np.empty(reps)
    for i in range(reps):
        t0 = time.perf_counter()
        fn()
        t[i] = (time.perf_counter() - t0) * 1e6
    return dict(p50=float(np.percentile(t, 50)), p99=float(np.percentile(t, 99)),
                max=float(t.max()), reps=reps)


def show(name, s):
    print(f"  {name:44s} p50 {s['p50']:9.1f} us   p99 {s['p99']:9.1f}   max {s['max']:9.1f}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pipeline", type=int, default=0)
    ap.add_argument("--udmabuf", default="udmabuf1")
    ap.add_argument("--reps", type=int, default=1000)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    r = fl.create_standard_readout_interface(FW, args.pipeline)   # katcp
    r_fast = fl.create_fast_readout_interface(FW, args.pipeline)  # LocalMem
    acc, acc_fast = r.accumulators[0], r_fast.accumulators[0]
    res = dict(time=time.strftime("%Y-%m-%dT%H:%M:%S"), fl_file=fl.__file__)
    try:
        res["acc_len"] = int(r_fast.mixer.get_acc_len())
    except Exception:
        pass

    print("\n=== control_sw calls: katcp vs LocalMem ===")
    for name, fn, reps in (
            ("katcp    acc._read_bram() (2048 ch)", acc._read_bram, 50),
            ("localmem acc._read_bram() (2048 ch)", acc_fast._read_bram, args.reps),
            ("katcp    acc.get_acc_cnt()", acc.get_acc_cnt, 200),
            ("localmem acc.get_acc_cnt()", acc_fast.get_acc_cnt, args.reps)):
        res[name] = pct(fn, reps)
        show(name, res[name])

    # time the read only: stand in the current counter for the wait
    if hasattr(fl, "_blocking_wait_for_acc_fast"):
        fl._blocking_wait_for_acc_fast = lambda fp, *a, **k: fl.read_acc_cnt_fast(fp)
    fl._blocking_wait_for_acc = lambda a, *x, **k: a.get_acc_cnt()

    params = {"devmem": fl.get_fast_read_params(r_fast)}
    if "zdma" in inspect.signature(fl.get_fast_read_params).parameters \
            and os.path.exists(f"/dev/{args.udmabuf}"):
        p = fl.get_fast_read_params(r_fast, zdma=args.udmabuf)
        if p.get("zdma") is not None:
            params["zdma-enabled"] = p

    print("\n=== firmware_lib.read_accumulated_data_fast (no wait) ===")
    if hasattr(fl, "read_acc_cnt_fast"):
        name = "read_acc_cnt_fast()"
        res[name] = pct(lambda: fl.read_acc_cnt_fast(params["devmem"]), args.reps)
        show(name, res[name])
    for label, p in params.items():
        for M in TONES:
            ti = np.arange(M)
            # which transport and span the planner actually picks for this tone set
            plan = fl._cached_read_plan(p, ti, None) if hasattr(fl, "_cached_read_plan") else None
            used = "full buffer" if plan is None else \
                f"{plan.get('transport', 'devmem')} {sum(n for _, n in plan['ranges'])} ch"
            name = f"{label:12s} {M:5d} tones -> {used}"
            res[name] = pct(lambda: fl.read_accumulated_data_fast(p, tone_indices=ti),
                            args.reps)
            show(name, res[name])

    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=1)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
