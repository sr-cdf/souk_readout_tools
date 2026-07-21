#!/usr/bin/env python3
"""
Tone-tracking performance benchmarks (doc/tone_tracking.md).

Measures, at 2048 tones:
1. The producer-side tap cost (the ONLY work added to the stream_data
   per-sample hot path): one deque append of the already-built payload.
2. Consumer-side batch processing: parse_frame_batch + assemble_cycles +
   estimate_detunings + filter, per cycle and per second of stream data.
3. The estimator alone (vectorised across all tones).
4. For reference, modulation.demodulate's model-free path on the same
   batch (the analysis-grade implementation the lean estimator mirrors).

Run on the RFSoC ARM (quad-A53) for the numbers that matter; dev-machine
numbers bound the algorithmic scaling.

Usage: PYTHONPATH=src python3 profiling/tracking_benchmark.py [n_tones]
"""

import sys
import time
from collections import deque

import numpy as np

from souk_readout_tools.server import tracking
from souk_readout_tools import modulation


def make_payloads(n_samples, n_tones, num_points=3, spp=2, n_settle=1):
    import struct
    payloads = []
    words = np.zeros(2 * n_tones + 10, dtype='<i4')
    rng = np.random.default_rng(0)
    for k in range(n_samples):
        point = (k // spp) % num_points + 1
        settling = (k % spp) < n_settle
        words[0:2 * n_tones] = rng.integers(-2**20, 2**20, 2 * n_tones)
        tail = np.zeros(10, dtype='<u4')
        tail[5] = np.uint32((point & 0xFFFF) | (int(settling) << 16)
                            | (1 << 17))
        tail[8] = k
        words[-10:] = tail.view('<i4')
        body = words.tobytes()
        payloads.append(struct.pack('>I', len(body)) + body)
    return payloads


def main():
    n_tones = int(sys.argv[1]) if len(sys.argv) > 1 else 2048
    num_points, spp = 3, 2
    sample_rate = 500.0     # typical stream rate at 2048 tones
    n_samples = 600         # ~1.2 s of stream, 100 cycles
    print(f'n_tones={n_tones} num_points={num_points} spp={spp} '
          f'batch={n_samples} samples ({n_samples/sample_rate:.1f} s of '
          f'stream at {sample_rate:.0f} Hz)')

    payloads = make_payloads(n_samples, n_tones, num_points, spp)

    # 1. Tap: deque append per frame (the entire hot-path addition).
    ring = deque(maxlen=2048)
    reps = 20000
    t0 = time.perf_counter()
    for i in range(reps):
        ring.append(payloads[i % n_samples])
    tap_us = 1e6 * (time.perf_counter() - t0) / reps
    print(f'1. tap (deque append) ............ {tap_us:8.3f} us/frame '
          f'({100 * tap_us * 1e-6 * sample_rate:.4f}% of a '
          f'{1e3/sample_rate:.1f} ms frame period)')

    # 2. Full consumer batch: parse + assemble + estimate + filter.
    linewidth = np.full(n_tones, 1.0e5)
    offsets = np.stack([np.full(n_tones, -1e4), np.zeros(n_tones),
                        np.full(n_tones, 1e4)])
    filt = tracking.DetuningFilter('boxcar', n_tones, window=10)
    n_rep = 5
    t0 = time.perf_counter()
    for _ in range(n_rep):
        batch = tracking.parse_frame_batch(payloads, n_tones)
        cycles, _ = tracking.assemble_cycles(batch, num_points)
        est = tracking.estimate_detunings(cycles['z'], offsets, linewidth)
        filt.update(est['detuning_linewidths'])
    per_batch_ms = 1e3 * (time.perf_counter() - t0) / n_rep
    n_cycles = cycles['z'].shape[0]
    print(f'2. consumer batch ................ {per_batch_ms:8.2f} ms/batch '
          f'({n_cycles} cycles; {per_batch_ms/(n_samples/sample_rate)*0.1:.2f}% '
          'of real time)')

    # 3. Estimator alone.
    z = cycles['z']
    t0 = time.perf_counter()
    for _ in range(20):
        tracking.estimate_detunings(z, offsets, linewidth)
    est_ms = 1e3 * (time.perf_counter() - t0) / 20
    print(f'3. estimator alone ............... {est_ms:8.2f} ms/'
          f'{n_cycles} cycles ({1e3*est_ms/n_cycles:.1f} us/cycle, all '
          f'{n_tones} tones)')

    # 4. Reference: demodulate model-free on the same cycles.
    grouped = {'z': z, 'offsets_hz': offsets}
    t0 = time.perf_counter()
    for _ in range(3):
        modulation.demodulate(grouped, method='fast', linewidth_hz=linewidth)
    demod_ms = 1e3 * (time.perf_counter() - t0) / 3
    print(f'4. demodulate (reference) ........ {demod_ms:8.2f} ms/'
          f'{n_cycles} cycles ({demod_ms/est_ms:.0f}x the lean estimator)')


if __name__ == '__main__':
    main()
