#!/usr/bin/env python3
"""
Tracking health-reporting timing: on-loop cost vs off-loop build.

get_info('tracking') and health_check() run synchronously on the server's
asyncio event loop -- the same loop as the stream producer -- so any
per-tone work they do steals time from the stream loop. This benchmark
confirms the split:

- ``status()`` (what the on-loop callers use) returns the cached snapshot
  in O(1): flat in tone count, sub-microsecond.
- ``_build_snapshot()`` (the O(n_tones) per-tone build) is what the
  consumer runs OFF the loop, once per poll, in a to_thread executor.

Run on the RFSoC ARM (quad-A53) for production numbers; dev-machine numbers
bound the scaling.

Usage: PYTHONPATH=src python3 profiling/tracking_health_benchmark.py
"""

import asyncio
import time

import numpy as np

from souk_readout_tools.server import tracking
from souk_readout_tools.server import readout_server


class _Server:
    def __init__(self, n, npts=3, lw=1.0e5):
        self.sweep_state = {'state': 'idle'}
        self.e_stream_enabled = asyncio.Event()
        self.e_modulation_enabled = asyncio.Event()
        self.e_fw_modulation_enabled = asyncio.Event()
        self.e_stream_enabled.set()
        self.e_modulation_enabled.set()
        c = 1.0e9 + np.arange(n) * 1.0e5
        off = np.stack([np.full(n, -0.1 * lw), np.zeros(n),
                        np.full(n, 0.1 * lw)])
        self.modulation_cfg = {
            'center': c, 'offsets': off, 'mod_indices': list(range(n)),
            'samples_per_point': 2, 'n_settle': 1, 'linewidth_hz': [lw] * n}
        self.modulation_state = {
            'enabled': True, 'applied_revision': 1, 'num_points': npts,
            'samples_per_point': 2, 'n_settle': 1,
            'mod_indices': list(range(n)), 'sample_rate_hz': 500.0,
            'cycle_rate_hz': 80.0,
            'tones': [{'index': i, 'drift_bins': [0.05, 0.0, 0.05]}
                      for i in range(n)]}
        self.modulation_params = {'armed': {'tx_bin_width_hz': 1.0e6}}
        self.fw_modulation_cfg = None
        self.fw_modulation_state = None
        self.fw_modulation_params = None


def _runtime(n):
    p = dict(readout_server.ReadoutServer.TRACKING_DEFAULTS)
    p['filter_window'] = 2
    rt = tracking.TrackingRuntime(_Server(n), p)
    rng = np.random.default_rng(0)
    rt.last_filtered_lw = rng.uniform(-0.5, 0.5, n)
    rt.stat_detuning_var = rng.uniform(0, 0.01, n)
    rt.stat_abs_slope = rng.uniform(1e-6, 3e-6, n)
    rt.stat_invalid_frac = rng.uniform(0, 0.1, n)
    rt.slope_baseline = rt.stat_abs_slope.copy()
    for i in range(0, n, 500):
        rt.controller.staged['bin'][i] = 1.0e9
    rt._snapshot = rt._build_snapshot()
    return rt


def _timeit(fn, reps):
    t = time.perf_counter()
    for _ in range(reps):
        fn()
    return 1e3 * (time.perf_counter() - t) / reps


def main():
    print(f'{"tones":>6}  {"status() on-loop":>18}  '
          f'{"_build_snapshot off-loop":>24}')
    for n in (256, 512, 1024, 2048):
        rt = _runtime(n)
        on_loop = _timeit(rt.status, 20000)
        off_loop = _timeit(rt._build_snapshot, 200)
        print(f'{n:6d}  {on_loop*1e3:15.3f} us  {off_loop:21.2f} ms')
    print('\nstatus() is flat (cache read); the O(n) build runs in the '
          "consumer's to_thread, off the stream loop.")


if __name__ == '__main__':
    main()
