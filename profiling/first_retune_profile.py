#!/usr/bin/env python3
"""
Why the first set_tone_frequencies after a server restart takes ~4.8 s (test plan
item 4 in doc/v712-notes.md). Board side, as casper, with the servers STOPPED
(this process writes tones, and a running server would hold a stale copy).

Builds the interfaces as the server does (standard + fast, RAM copy of the LO
control registers), reads the loaded tones, then re-sends the same frequencies,
amplitudes and phases twice through set_tone_frequencies_fast under cProfile.
A fresh process starts cold like a restarted server, so call 1 against call 2
isolates the one-off cost.

    taskset -c 3 ~/py3.12-venv/bin/python first_retune_profile.py \\
        ~/.souk_readout_tools/pipeline_1/config/config_6HP90-MS2-Top_p1.yaml \\
        --out first_retune_p1.txt
"""
import argparse
import cProfile
import io
import logging
import pstats
import time

import numpy as np
import yaml

t_import = time.perf_counter()
from souk_readout_tools import firmware_lib as fl
t_import = time.perf_counter() - t_import


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('config', help='the server config file for the pipeline')
    ap.add_argument('--out', required=True)
    ap.add_argument('--top', type=int, default=30)
    args = ap.parse_args()
    logging.disable(logging.WARNING)
    cfg = yaml.safe_load(open(args.config))
    fw_file, pipeline = cfg['firmware']['fw_config_file'], cfg['firmware']['pipeline_id']
    lines = [f'config {args.config}, pipeline {pipeline}',
             f'import firmware_lib: {t_import:.2f} s']

    t0 = time.perf_counter()
    r = fl.create_standard_readout_interface(fw_file, pipeline)
    r_fast = fl.create_fast_readout_interface(fw_file, pipeline, uio=True)   # server: /dev/mem as root
    fl.get_control_shadow(r_fast, refresh=True)
    lines.append(f'interfaces + control copy: {time.perf_counter() - t0:.2f} s')

    t0 = time.perf_counter()
    freqs = np.asarray(fl.get_tone_frequencies(r, r_fast, cfg), dtype=float)
    amps = np.asarray(fl.get_tone_amplitudes(r, r_fast, cfg), dtype=float)
    phases = np.asarray(fl.get_tone_phases(r, r_fast, cfg), dtype=float)
    lines.append(f'read loaded tones ({len(freqs)}): {time.perf_counter() - t0:.2f} s '
                 '(not part of the request)')

    for call in (1, 2):
        prof = cProfile.Profile()
        t0 = time.perf_counter()
        prof.enable()
        fl.set_tone_frequencies_fast(r, r_fast, cfg, freqs, tone_amplitudes=amps,
                                     tone_phases=phases)
        prof.disable()
        wall = time.perf_counter() - t0
        s = io.StringIO()
        pstats.Stats(prof, stream=s).sort_stats('cumulative').print_stats(args.top)
        lines += ['', f'==== call {call}: {wall:.3f} s wall ====', s.getvalue()]

    stale = fl.check_control_shadow(r_fast)
    lines.append(f'control copy after: {stale}')
    open(args.out, 'w').write('\n'.join(lines))
    print('\n'.join(l for l in lines if l.startswith(('config', 'import', 'interfaces',
                                                     'read', '====', 'control'))))


if __name__ == '__main__':
    main()
