#!/usr/bin/env python3
"""
Sweep time per point and set_tone_powers time per call, end to end through the
server (test plan item 3 in doc/v712-notes.md). Host side, no root, servers
running, nothing streaming. Pipeline 1 by default (timing does not need signal).

Sweeps: the loaded tones as centres, --span wide, at --points (a small warm-up
sweep first absorbs one-off costs). Time is measured from the request to
progress 1.0 (polled every 50 ms), so

    per point = (t[most points] - t[fewest points]) / (difference in points)
    fixed     = t[fewest points] - fewest points * per point

and per point is set against the data it collects, (samples_per_point +
settle_accumulations) / sample_rate.

set_tone_powers: the loaded powers re-sent, once with optimise_dynamic_range=False
and rx_policy='none' (the write path) and once with the defaults (optimisers
included). Amplitudes, phases, psb_scale, DSA and ADC calibration freeze are
read first and restored at the end.

    PYTHONPATH=src .venv/bin/python profiling/sweep_step_timing.py --port 10001 \\
        --out ~/souk/v712_stages/sweep_step_timing.json
"""
import argparse
import json
import time

import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient


def timed_sweep(c, centers, span, points, spp, settle):
    t0 = time.perf_counter()
    resp = c.perform_sweep(centers, np.full(len(centers), span), points, spp,
                           refresh_adc_cal=False, settle_accumulations=settle)
    if resp.get('status') != 'success':
        raise RuntimeError(f'sweep not started: {resp}')
    while True:
        p = c.get_sweep_progress()
        if isinstance(p, dict) or p >= 1.0:
            break
        time.sleep(0.05)
    return time.perf_counter() - t0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--addr', default='10.11.11.11')
    ap.add_argument('--port', type=int, default=10001)
    ap.add_argument('--span', type=float, default=100e3)
    ap.add_argument('--points', type=int, nargs='+', default=[11, 101, 401])
    ap.add_argument('--samples-per-point', type=int, default=10)
    ap.add_argument('--settle', type=int, default=4)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()

    c = ReadoutClient(address=args.addr, request_port=args.port)
    freqs = np.asarray(c.get_tone_frequencies(), dtype=float)
    amps = np.asarray(c.get_tone_amplitudes(), dtype=float)
    phases = np.asarray(c.get_tone_phases(), dtype=float)
    state = {'psb_scale': c.get_psb_scale(), 'dsa': c.get_adc_dsa()['result']['adc_dsa_db'],
             'cal_freeze': c.get_cal_freeze()}
    rate = float(c.get_sample_rate())
    res = {'time': time.strftime('%Y-%m-%dT%H:%M:%S'), 'port': args.port, 'tones': len(freqs),
           'sample_rate': rate, 'state_before': state}
    print(f'{len(freqs)} tones, {rate:.0f} Hz, state {state}', flush=True)

    try:
        timed_sweep(c, freqs, args.span, 3, args.samples_per_point, args.settle)   # warm-up
        sweeps = {}
        for n in args.points:
            sweeps[n] = timed_sweep(c, freqs, args.span, n, args.samples_per_point, args.settle)
            print(f'sweep {n} points: {sweeps[n]:.2f} s', flush=True)
        lo, hi = min(sweeps), max(sweeps)
        per_point = (sweeps[hi] - sweeps[lo]) / (hi - lo)
        data = (args.samples_per_point + args.settle) / rate
        res['sweep'] = {'seconds': sweeps, 'per_point_ms': round(1e3 * per_point, 2),
                        'fixed_s': round(sweeps[lo] - lo * per_point, 2),
                        'data_per_point_ms': round(1e3 * data, 2),
                        'overhead_per_point_ms': round(1e3 * (per_point - data), 2)}
        print(res['sweep'], flush=True)

        powers = np.asarray(c.get_tone_powers(), dtype=float)
        for label, kw in (('write_path', dict(optimise_dynamic_range=False, rx_policy='none')),
                          ('defaults', {})):
            t0 = time.perf_counter()
            try:
                c.set_tone_powers(powers, verbose=False, **kw)
                res[f'set_tone_powers_{label}_s'] = round(time.perf_counter() - t0, 3)
            except Exception as e:
                res[f'set_tone_powers_{label}_error'] = repr(e)[:200]
            print(label, res.get(f'set_tone_powers_{label}_s', res.get(f'set_tone_powers_{label}_error')),
                  flush=True)
    finally:
        c.set_psb_scale(state['psb_scale'])
        c.set_adc_dsa(state['dsa'])
        c.set_tone_frequencies(freqs)
        c.set_tone_amplitudes(amps)
        c.set_tone_phases(phases)
        c.set_cal_freeze(state['cal_freeze'])
        after = {'psb_scale': c.get_psb_scale(), 'dsa': c.get_adc_dsa()['result']['adc_dsa_db'],
                 'cal_freeze': c.get_cal_freeze()}
        res['restored'] = bool(
            after == state
            and np.allclose(np.asarray(c.get_tone_frequencies(), float), freqs)
            and np.allclose(np.asarray(c.get_tone_amplitudes(), float), amps)
            and np.allclose(np.asarray(c.get_tone_phases(), float), phases))
        res['state_after'] = after
        json.dump(res, open(args.out, 'w'), indent=1)
        print('restored:', res['restored'], after, flush=True)


if __name__ == '__main__':
    main()
