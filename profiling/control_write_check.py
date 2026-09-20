#!/usr/bin/env python3
"""
LO control-word writes through the RAM copy of the control registers
(firmware_lib.get_control_shadow): correctness and timing on the board.

Runs as casper (UIO). The readout servers may stay up if idle -- nothing here
streams -- but this process keeps its own copy, so slot 3 (the spare fw-mod slot)
carries the timing writes and is restored word for word at the end; the one
live-slot write (check_sparse, tone 5) is restored too. Refuses to run while the
mixer auto-cycles slots, when slot 3 would be live.

    taskset -c 3 python control_write_check.py --pipeline 1 --out ctl_write.json

Checks:
  katcp     slot 0 of both buffers read over katcp equals the copy (catches a
            byte-order error the devmem read/write pair would hide)
  sparse    board_checks.check_sparse: one tone's words rewritten through the copy,
            neighbours and the other LO unchanged, restored
  stale     check_control_shadow after everything: the copy equals the firmware
Timings (1000 tones, tx+rx, one ping-pong buffer, slot 3, us):
  freq      phase_steps + ri_steps (retune, sweep step, modulation point)
  all4      all four fields
  scaling   scaling for all 2048 rows (what set_tone_amplitudes writes per buffer)
  sparse    freq for 1000 non-contiguous tones (every other LO index)
  *_old     the scattered per-word store this replaced, same words
"""

import argparse
import gc
import json
import os
import sys
import time

import numpy as np
from souk_readout_tools import firmware_lib as fl

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from board_checks import check_sparse

FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")
SLOT = 3
FIELDS = ('phase_steps', 'ri_steps', 'phase_offsets', 'scaling')


def pct(ts):
    ts = 1e6 * np.asarray(ts)
    return {k: round(float(v), 2) for k, v in zip(('p50', 'p90', 'p99', 'max'),
                                                  (*np.percentile(ts, (50, 90, 99)), ts.max()))}


def timed(fn, n):
    for _ in range(50):
        fn()
    ts = np.empty(n)
    for k in range(n):
        t0 = time.perf_counter()
        fn()
        ts[k] = time.perf_counter() - t0
    return pct(ts)


def old_format(r_fast, v, tone_idx):
    """The flat '>u4' values + word indices the old prepare returned (field order kept)."""
    m = r_fast.mixer
    cols = {'phase_steps': m._PHASE_INC_WORD_OFFSET, 'ri_steps': m._RI_STEP_WORD_OFFSET,
            'phase_offsets': m._PHASE_OFFSET_WORD_OFFSET, 'scaling': m._SCALE_WORD_OFFSET}
    order = [f for f in ('phase_steps', 'phase_offsets', 'ri_steps', 'scaling') if f in v['tx']]
    vals = {lo: np.concatenate([v[lo][f] for f in order]).astype('>u4') for lo in ('tx', 'rx')}
    idx = {lo: np.concatenate([tone_idx * 4 + cols[f] for f in order]) for lo in ('tx', 'rx')}
    return vals, idx


def old_write(r_fast, buf, v, indices, view):
    """write_control_buffer_data_fast before the copy (sparse_write=None path)."""
    m = r_fast.mixer
    addrs = fl._get_control_buffer_addresses(r_fast)
    buf_size = m._n_serial_chans * m._CONTROL_N_WORDS * 4
    starts = {'tx': (addrs[('tx', buf)] + buf_size * SLOT) // 4,
              'rx': (addrs[('rx', buf)] + buf_size * SLOT) // 4}
    for lo in ('tx', 'rx'):
        idx = np.asarray(indices[lo])
        vals = v[lo].astype('<u4')
        first, last = int(idx.min()), int(idx.max())      # plan_control_buffer_write
        view[starts[lo] + idx] = vals


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pipeline", type=int, default=1)
    ap.add_argument("--n", type=int, default=2000, help="repeats per timing")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    gc.disable()

    res = dict(time=time.strftime("%Y-%m-%dT%H:%M:%S"), package=fl.__file__,
               cpus=sorted(os.sched_getaffinity(0)), pipeline=args.pipeline)
    r = fl.create_standard_readout_interface(FW, args.pipeline)
    r_fast = fl.create_fast_readout_interface(FW, args.pipeline, uio=True)
    m = r_fast.mixer
    res['slot_mode'] = m.get_slot_mode()
    if res['slot_mode'] != 'manual':
        raise SystemExit(f"mixer slot mode is {res['slot_mode']!r}: slot {SLOT} may be live")

    t0 = time.perf_counter()
    shadow = fl.get_control_shadow(r_fast, refresh=True)
    res['copy_read_ms'] = round(1e3 * (time.perf_counter() - t0), 2)
    orig = {key: words[SLOT].copy() for key, words in shadow.items()}

    # katcp cross-read of slot 0 (big-endian words over katcp, little-endian over devmem)
    kat = {'passed': True}
    for buf in (0, 1):
        k = fl.read_raw_control_buffer_data(r, buf, slot=0)
        for lo in ('tx', 'rx'):
            for f in FIELDS:
                col = getattr(m, {'phase_steps': '_PHASE_INC_WORD_OFFSET', 'ri_steps': '_RI_STEP_WORD_OFFSET',
                                  'phase_offsets': '_PHASE_OFFSET_WORD_OFFSET', 'scaling': '_SCALE_WORD_OFFSET'}[f])
                same = np.array_equal(k[lo][f'formatted_{f}'].astype(np.uint32),
                                      shadow[(lo, buf)][0, :, col])
                kat[f'{lo}{buf}_{f}'] = bool(same)
                kat['passed'] &= bool(same)
    res['katcp'] = kat

    # timings on the spare slot
    n_ch = m._n_serial_chans
    rng = np.random.default_rng(0)
    steps = rng.uniform(-np.pi, np.pi, 1000)
    lo_freq = {lo: {'phase_steps': steps, 'ri_steps': np.exp(1j * steps)} for lo in ('tx', 'rx')}
    lo_all4 = {lo: dict(lo_freq[lo], phase_offsets=rng.uniform(-np.pi, np.pi, 1000),
                        scaling=rng.uniform(0, 1, 1000)) for lo in ('tx', 'rx')}
    lo_scale = {'tx': {'scaling': np.r_[np.full(1000, 0.5), np.zeros(n_ch - 1000)]},
                'rx': {'scaling': np.r_[np.ones(1000), np.zeros(n_ch - 1000)]}}
    contiguous = np.arange(1000)
    sparse = np.arange(0, 2000, 2)
    view = np.frombuffer(memoryview(m.host.transport.axil_mm), dtype='<u4')
    timings = {}
    for name, vals, idx in (('freq', lo_freq, contiguous), ('all4', lo_all4, contiguous),
                            ('scaling', lo_scale, None), ('sparse', lo_freq, sparse)):
        timings[f'{name}_prepare'] = timed(
            lambda: fl.prepare_control_buffer_data_fast(r_fast, 0, vals, tone_indices=idx), 200)
        v, rows = fl.prepare_control_buffer_data_fast(r_fast, 0, vals, tone_indices=idx)
        timings[name] = timed(lambda: fl.write_control_buffer_data_fast(r_fast, 0, v, rows, slot=SLOT), args.n)
        v_old, i_old = old_format(r_fast, v, np.arange(n_ch) if idx is None else idx)
        timings[f'{name}_old'] = timed(lambda: old_write(r_fast, 0, v_old, i_old, view), args.n)
        # the old store bypassed the copy: put the copy's words back before comparing
        for lo in ('tx', 'rx'):
            fl._flush_control_shadow(r_fast, lo, 0, SLOT, 0, n_ch)
    words = shadow[('tx', 0)].reshape(-1)
    base = SLOT * n_ch * 4
    a = fl._get_control_buffer_addresses(r_fast)[('tx', 0)] + 4 * base
    mm = m.host.transport.axil_mm
    def flush_4000():
        mm[a:a + 16000] = words[base:base + 4000]
    timings['flush_4000_words'] = timed(flush_4000, args.n)
    res['timings_us'] = timings

    # restore slot 3 exactly as found, then the live-slot check
    for key in shadow:
        shadow[key][SLOT] = orig[key]
        fl._flush_control_shadow(r_fast, key[0], key[1], SLOT, 0, n_ch)
    res['sparse'] = check_sparse(r_fast)
    res['stale'] = fl.check_control_shadow(r_fast)
    res['slot3_restored'] = all(np.array_equal(
        fl.get_control_shadow(r_fast)[key][SLOT], orig[key]) for key in orig) and not res['stale']['stale']

    print(json.dumps({k: v for k, v in res.items() if k not in ('timings_us', 'sparse', 'stale')}, default=str))
    print('sparse', res['sparse']['passed'], 'copy_stale_words', res['sparse']['copy_stale_words'])
    print('stale', res['stale']['stale'], res['stale']['stale_words'])
    for k, t in timings.items():
        print(f'{k:20s} p50 {t["p50"]:8.2f}  p90 {t["p90"]:8.2f}  p99 {t["p99"]:8.2f}  max {t["max"]:8.2f}')
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=1, default=str)


if __name__ == "__main__":
    main()
