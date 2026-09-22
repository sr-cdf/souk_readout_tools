#!/usr/bin/env python3
"""Per-frame hot-path pieces, current vs. cheaper alternatives. RAM only.

Times each piece of what ``prepare_frame`` + ``tracking.tap`` do per
accumulation, against the alternative considered for it, on synthetic data
of the real shape (1000 tones, a contiguous comb). No hardware is touched,
so it can run with the servers up -- but on the board, run it on CPU0-1 and
not during a loss measurement (see profiling/README.md).

    taskset -c 0 python frame_path_bench.py [--tones 1000] [--n 20000]

Prints p50 / p99 in microseconds. Only the ARM numbers mean anything for the
budget; on a PC read the ratios.
"""
import argparse
import struct
import time

import numpy as np


def bench(fn, n):
    t = np.empty(n)
    pc = time.perf_counter
    for i in range(n):
        t0 = pc()
        fn()
        t[i] = pc() - t0
    return 1e6 * np.percentile(t, 50), 1e6 * np.percentile(t, 99)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--tones', type=int, default=1000)
    ap.add_argument('--n', type=int, default=20000)
    args = ap.parse_args()
    m, n = args.tones, args.n
    nw = 2 * m
    rng = np.random.default_rng(0)

    # The landing buffer as ZDMA leaves it: the whole accumulator, cached.
    landing = rng.integers(-2**20, 2**20, 2 * 2048, dtype=np.int32)
    dout = np.frombuffer(landing.tobytes(), dtype='<i4')
    take = np.arange(nw)                      # plan['take'] for a contiguous comb
    sl = slice(0, nw)

    # prepare_frame's cached frame, header view and length prefix
    frame = np.zeros(nw + 10, dtype='<i4')
    hdr = frame[-10:].view('<u4')
    data_len = struct.pack('>I', (nw + 10) * 4)
    hdr_pack = struct.Struct('<10I').pack
    flags = [False] * 10

    results = []

    def row(name, fn):
        p50, p99 = bench(fn, n)
        results.append((name, p50, p99))

    # -- the read's gather --------------------------------------------------
    row('gather: dout[take] (fancy index, today)', lambda: dout[take])
    row('gather: dout[a:b] (slice view)', lambda: dout[sl])

    # -- frame build -----------------------------------------------------------
    data = dout[take]

    def build_today():
        frame[:nw] = data
        for k in range(1, 11):
            hdr[-k] = k
        return data_len + frame.tobytes()

    view = dout[sl]

    def build_join():
        h = hdr_pack(0, 1, 2, 3, 4, 5, 6, 7, 8, 9)
        return b''.join((data_len, view, h))

    row('build: copy + 10 setitems + tobytes + concat (today)', build_today)
    row('build: struct.pack header + one join from the view', build_join)
    assert len(build_today()) == len(build_join())

    # -- tracking tap ------------------------------------------------------------
    acc64 = np.zeros(nw, dtype=np.int64)
    acc32 = np.zeros(nw, dtype=np.int32)
    accf = np.zeros(nw, dtype=np.float64)
    row('tap: np.add(int64, int32, casting=unsafe) (today)',
        lambda: np.add(acc64, view, out=acc64, casting='unsafe'))
    row('tap: acc64 += view', lambda: acc64.__iadd__(view))
    row('tap: np.add(float64, int32)',
        lambda: np.add(accf, view, out=accf, casting='unsafe'))
    row('tap: np.add(int32, int32) (overflows; reference only)',
        lambda: np.add(acc32, view, out=acc32))
    buf64 = np.empty(nw, dtype=np.int64)

    def two_step():
        np.copyto(buf64, view, casting='safe')
        np.add(acc64, buf64, out=acc64)
    row('tap: copyto(int64 scratch) + same-type add', two_step)

    # -- misc per-frame Python ---------------------------------------------------
    import asyncio
    ev = asyncio.Event()
    row('6x asyncio.Event.is_set()', lambda: [ev.is_set() for _ in range(6)])
    row('time.perf_counter()', time.perf_counter)

    w = max(len(r[0]) for r in results)
    print(f'{"piece":<{w}}  {"p50 us":>8}  {"p99 us":>8}')
    for name, p50, p99 in results:
        print(f'{name:<{w}}  {p50:8.2f}  {p99:8.2f}')


if __name__ == '__main__':
    main()
