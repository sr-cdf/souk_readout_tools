#!/usr/bin/env python3
"""
What a tone-tracking correction would cost to apply: fill the idle half of the
LO control ping-pong with recomputed phase-accumulator words, then flip to it
(doc/tone_tracking.md, "Running the tracker on the ARM"). Board side, as casper
(UIO), servers may stay up -- nothing live is moved.

Writes only the spare fw-modulation slot (3) of the **inactive** buffer and puts
every word back at the end, so the running pipeline never sees a changed tone.
The flip is timed by writing the buffer index that is already selected, which is
the same register write without the switch.

    taskset -c 1 ~/py3.12-venv/bin/python tracker_apply_cost.py --pipeline 1 \\
        --out tracker_apply.json
"""
import argparse
import json
import logging
import os
import time

import numpy as np
from souk_readout_tools import firmware_lib as fl

FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")
SLOT = 3                      # the spare fw-modulation slot


def pct(ts):
    ts = 1e6 * np.asarray(ts)
    return {k: round(float(v), 1) for k, v in
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
    ap.add_argument('--pipeline', type=int, default=1)
    ap.add_argument('--tones', type=int, default=1000)
    ap.add_argument('--slots', type=int, default=4, help='modulation slots to fill per update')
    ap.add_argument('--repeats', type=int, default=50)
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    logging.disable(logging.WARNING)

    rf = fl.create_fast_readout_interface(FW, args.pipeline, uio=True)
    m = rf.mixer
    live_buf = m.get_current_buffer()
    idle_buf = (live_buf + 1) % 2
    res = {'time': time.strftime('%Y-%m-%dT%H:%M:%S'), 'pipeline': args.pipeline,
           'tones': args.tones, 'slots': args.slots, 'live_buffer': live_buf,
           'idle_buffer': idle_buf, 'slot_written': SLOT}

    # Keep the words we are about to overwrite, to put back at the end.
    shadow = fl.get_control_shadow(rf, refresh=True)
    original = {lo: shadow[(lo, idle_buf)][SLOT, :, :].copy() for lo in ('tx', 'rx')}

    rows = np.arange(args.tones)
    phase_steps = np.linspace(0.01, 0.3, args.tones)
    ri_steps = np.ones(args.tones)

    def prepare():
        return fl.prepare_control_buffer_data_fast(
            rf, idle_buf,
            {'tx': {'phase_steps': phase_steps, 'ri_steps': ri_steps},
             'rx': {'phase_steps': -phase_steps, 'ri_steps': ri_steps}},
            tone_indices=rows)

    v, idx = prepare()
    res['prepare_us'] = timed(lambda: prepare(), args.repeats)

    def write_one_slot():
        fl.write_control_buffer_data_fast(rf, idle_buf, v, idx, slot=SLOT)

    res['write_one_slot_us'] = timed(write_one_slot, args.repeats)

    def write_all_slots():
        for _ in range(args.slots):
            fl.write_control_buffer_data_fast(rf, idle_buf, v, idx, slot=SLOT)

    res['fill_idle_buffer_us'] = timed(write_all_slots, max(10, args.repeats // 4))
    res['flip_us'] = timed(lambda: fl.set_control_buffer_idx_fast(rf, live_buf), args.repeats)

    # Restore, then confirm the copy matches the firmware again.
    for lo in ('tx', 'rx'):
        shadow[(lo, idle_buf)][SLOT, :, :] = original[lo]
        fl._flush_control_shadow(rf, lo, idle_buf, SLOT, 0, shadow[(lo, idle_buf)].shape[1])
    res['restored_clean'] = not fl.check_control_shadow(rf)['stale']
    res['buffer_unchanged'] = (m.get_current_buffer() == live_buf)
    json.dump(res, open(args.out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
