#!/usr/bin/env python3
"""Does yielding the GIL during the accumulation wait let the tracking
consumer share the stream loop's process at 2500 Hz?

The stream loop currently waits for the next accumulation in
``firmware_lib._blocking_sleep``, a pure-Python spin, so it holds the GIL for
most of every frame period. This compares that against waiting with
``time.sleep`` for all but the last ``--spin-us``, which releases the GIL.

The producer thread does ``--work-us`` of GIL-held work per frame (the read and
frame build) and must finish within the frame period or the accumulation is
lost. The consumer thread runs the real tracking batch pipeline every ``poll``.

    taskset -c 0,1 ~/py3.12-venv/bin/python gil_yield_check.py --out y.json
"""
import argparse
import json
import sys
import threading
import time

import numpy as np

from souk_readout_tools.server import tracking as tk


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tones', type=int, default=1000)
    ap.add_argument('--rate', type=float, default=2500.0)
    ap.add_argument('--poll', type=float, default=0.5)
    ap.add_argument('--seconds', type=float, default=20.0)
    ap.add_argument('--work-us', type=float, default=140.0,
                    help='GIL-held work per frame: read + frame build')
    ap.add_argument('--spin-us', type=float, default=120.0,
                    help='busy-spin tail after the sleep (0 = spin the whole wait)')
    ap.add_argument('--mode', choices=('spin', 'sleep'), default='sleep')
    ap.add_argument('--switch-interval', type=float, default=None)
    ap.add_argument('--no-consumer', action='store_true')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    if args.switch_interval:
        sys.setswitchinterval(args.switch_interval)

    n, npts = args.tones, 4
    W = tk.STREAM_TRAILER_WORDS
    nw = 1 + 2 * n + W
    frames = int(args.rate * args.poll)
    rng = np.random.default_rng(0)
    payloads = []
    for i in range(frames):
        w = rng.integers(-2**20, 2**20, nw, dtype=np.int32)
        w[0] = 4 * (2 * n + W)
        w[-W + 5] = (i % npts) + 1
        w[-W + 8] = i
        payloads.append(w.tobytes())
    offsets = np.tile(np.array([-1e5, -3.3e4, 3.3e4, 1e5])[:, None], (1, n))
    lw = np.full(n, 5e4)

    stop = threading.Event()
    batch_ms = []

    def consumer():
        carry = None
        while not stop.is_set():
            time.sleep(args.poll)
            t0 = time.perf_counter()
            batch = tk.parse_frame_batch(payloads, n)
            cycles, carry = tk.assemble_cycles(batch, npts, carry)
            if cycles is not None:
                blocks = tk.average_cycles(cycles['z'], 4)
                est = tk.estimate_detunings(blocks, offsets, lw)
                _ = np.nanmean(est['detuning_hz'], axis=0)
            batch_ms.append(1e3 * (time.perf_counter() - t0))

    th = None
    if not args.no_consumer:
        th = threading.Thread(target=consumer, daemon=True)
        th.start()

    period = 1.0 / args.rate
    work = args.work_us * 1e-6
    spin = args.spin_us * 1e-6
    small = np.arange(64, dtype=np.int32)
    nmax = int(args.rate * args.seconds) + 1000
    busy = np.empty(nmax)            # wake -> work done, per frame
    k = 0
    t_end = time.perf_counter() + args.seconds
    next_acc = time.perf_counter() + period

    while time.perf_counter() < t_end and k < nmax:
        # wait for the "accumulation"
        if args.mode == 'sleep' and spin < period:
            d = next_acc - time.perf_counter() - spin - 60e-6   # 60 us wake latency
            if d > 0:
                time.sleep(d)
        while time.perf_counter() < next_acc:
            pass
        t_wake = time.perf_counter()
        # the GIL-held per-frame work: small numpy reads + building the payload
        end = t_wake + work
        while time.perf_counter() < end:
            s = int(small.sum())
            _ = len(small.tobytes()) + s
        busy[k] = time.perf_counter() - t_wake
        k += 1
        next_acc += period

    stop.set()
    if th:
        th.join(timeout=5)

    b = 1e6 * busy[:k]
    over = b[b > 1e6 * period]
    res = {'mode': args.mode, 'rate': args.rate, 'poll_s': args.poll,
           'work_us': args.work_us, 'spin_us': args.spin_us,
           'consumer': not args.no_consumer,
           'switch_interval_s': sys.getswitchinterval(),
           'frames': int(k), 'seconds': args.seconds,
           'batches': len(batch_ms),
           'batch_ms_p50': round(float(np.median(batch_ms)), 1) if batch_ms else None,
           'busy_us': {f'p{p}': round(float(np.percentile(b, p)), 1)
                       for p in (50, 90, 99, 99.9)},
           'busy_max_us': round(float(b.max()), 1),
           'lost': int(over.size),
           'lost_per_s': round(over.size / args.seconds, 2),
           'lost_pct': round(100.0 * over.size / max(k, 1), 4)}
    json.dump(res, open(args.out, 'w'), indent=1)
    print(json.dumps(res))


if __name__ == '__main__':
    main()
