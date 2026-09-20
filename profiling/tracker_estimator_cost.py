#!/usr/bin/env python3
"""
What a real-time tone tracker's estimator costs on the ARM, per consumer batch
(doc/tone_tracking.md). Pure CPU/RAM work on synthetic frames -- no hardware,
no server, safe to run while the pipelines stream if pinned off their cores.

One batch is ``poll_interval`` seconds of frames at ``rate``, i.e.
``rate * poll / 4`` four-point modulation cycles of ``tones`` tones. Times each
stage of the pipeline a tracker needs:

  parse      np.frombuffer views of the raw frame payloads
  assemble   copy into one (cycles, 4, tones, 2) buffer
  decimate   mean over cycles -- the whole point: per-cycle work is a copy,
             the estimator then runs on four averaged points
  estimate   detuning from the centred phase, linear form
             Im(dz * conj(z0)) / |z0|^2, no arctan2
  arctan2    the same estimate the expensive way, for comparison
  filter     one exponentially-weighted update per tone

    taskset -c 1 ~/py3.12-venv/bin/python tracker_estimator_cost.py --out est.json
"""
import argparse
import json
import time

import numpy as np


def pct(ts):
    ts = 1e3 * np.asarray(ts)
    return {k: round(float(v), 3) for k, v in
            zip(('p50', 'p90', 'max'), (*np.percentile(ts, (50, 90)), ts.max()))}


def timed(fn, n):
    fn()
    ts = np.empty(n)
    for k in range(n):
        t0 = time.perf_counter()
        fn()
        ts[k] = time.perf_counter() - t0
    return pct(ts)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--tones', type=int, default=1000)
    ap.add_argument('--rate', type=float, default=2500.0)
    ap.add_argument('--poll', type=float, default=0.2, help='consumer poll interval (s)')
    ap.add_argument('--repeats', type=int, default=20)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    cycles = max(1, int(args.rate * args.poll / 4))
    frames = cycles * 4
    words = 2 * args.tones + 10
    rng = np.random.default_rng(0)
    payloads = [rng.integers(-2**20, 2**20, words, dtype=np.int32).tobytes()
                for _ in range(frames)]
    res = {'tones': args.tones, 'rate': args.rate, 'poll_s': args.poll,
           'cycles_per_batch': cycles, 'frames_per_batch': frames,
           'batch_bytes': frames * words * 4}

    buf = np.empty((cycles, 4, args.tones, 2), dtype=np.int32)
    filt = np.zeros(args.tones)

    def parse():
        return [np.frombuffer(p, dtype='<i4', count=2 * args.tones) for p in payloads]

    views = parse()

    def assemble():
        for i, v in enumerate(views):
            buf[i // 4, i % 4] = v.reshape(args.tones, 2)

    def decimate():
        return buf.mean(axis=0)

    assemble()
    points = decimate()

    def estimate(pts):
        z = pts[..., 0] + 1j * pts[..., 1]          # (4, tones)
        z0 = 0.5 * (z[0] + z[2])                    # the two centred points
        dz = z[1] - z[3]                            # +delta minus -delta
        return np.imag(dz * np.conj(z0)) / np.maximum(np.abs(z0) ** 2, 1e-12)

    def estimate_arctan2(pts):
        z = pts[..., 0] + 1j * pts[..., 1]
        return np.arctan2(np.imag(z[1] - z[3]), np.real(z[0] + z[2]))

    def filter_update(x, alpha=0.2):
        filt[:] += alpha * (x - filt)

    det = estimate(points)
    res['parse_ms'] = timed(parse, args.repeats)
    res['assemble_ms'] = timed(assemble, args.repeats)
    res['decimate_ms'] = timed(decimate, args.repeats)
    res['estimate_ms'] = timed(lambda: estimate(points), args.repeats)
    res['estimate_arctan2_ms'] = timed(lambda: estimate_arctan2(points), args.repeats)
    res['filter_ms'] = timed(lambda: filter_update(det), args.repeats)

    def whole_batch():
        vs = parse()
        for i, v in enumerate(vs):
            buf[i // 4, i % 4] = v.reshape(args.tones, 2)
        filter_update(estimate(buf.mean(axis=0)))

    # The same answer without ever holding the batch: add each frame into a
    # (4, tones, 2) accumulator as it arrives, so the data is touched once.
    acc = np.zeros((4, args.tones, 2), dtype=np.int64)

    def whole_batch_incremental():
        acc[:] = 0
        for i, p in enumerate(payloads):
            acc[i % 4] += np.frombuffer(p, dtype='<i4',
                                        count=2 * args.tones).reshape(args.tones, 2)
        filter_update(estimate(acc / cycles))

    res['whole_batch_ms'] = timed(whole_batch, args.repeats)
    res['whole_batch_incremental_ms'] = timed(whole_batch_incremental, args.repeats)
    best = min(res['whole_batch_ms']['p50'], res['whole_batch_incremental_ms']['p50'])
    res['core_fraction_pct'] = round(100 * best / (1e3 * args.poll), 2)
    json.dump(res, open(args.out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
