#!/usr/bin/env python
"""
Measure the combined TX+RX filterbank channel response (standalone).

Steps a single test tone across a full channel with the software modulation
engine (fixed channel maps -- the geometry the response describes; a sweep
re-snaps every point and cannot measure this), one frequency at a time,
repeated at several frequencies and median-combined. Compensation is disabled
for the measurement, so this returns the RAW response -- compare against the
in-package model (overlaid on the plot) or feed a recalibration.

WHAT THIS SCRIPT CHANGES on the running system
  - internal (digital) loopback: switched ON for the measurement (default
    mode), with the digital gains re-optimised first
    (maximise_tx_power(digital_only=True)) to avoid DSP over/underflow.
    Loopback and gains are ALWAYS restored on exit, even on error.
  - the tone comb: replaced by the single test tone. Restored only with
    --restore; otherwise the test tone is left loaded (a notice is printed).
  - modulation: disabled on exit.

REQUIREMENTS: v7.11 firmware, server running, package installed client-side.
Digital mode needs nothing else; --rf mode expects an RF loopback cable (the
measured phase then carries the path delay: the script fits it and compares
against the config's rf_frontend.path_group_delay_ns).

Examples:
    python 01_measure_channel_response.py -C my_config.yaml
    python 01_measure_channel_response.py -C my_config.yaml --rf --restore
    python 01_measure_channel_response.py -a 10.11.11.11 --port 10000 \
        --frequencies 0.8e9 1.4e9 2.0e9
"""

import argparse
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fb_common as fb


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    fb.add_connection_args(parser)
    parser.add_argument('--rf', action='store_true',
                        help='RF loopback mode: leave internal loopback OFF '
                             '(expects a cable) and fit the path group delay')
    parser.add_argument('--frequencies', type=float, nargs='+',
                        help='test-tone frequencies in Hz (default: 3 spread '
                             "across the loaded comb's band)")
    parser.add_argument('--span-bins', type=float, default=0.98)
    parser.add_argument('--n-points', type=int, default=65)
    parser.add_argument('--n-cycles', type=int, default=8)
    parser.add_argument('--restore', action='store_true',
                        help='restore the previous tone comb on exit '
                             '(loopback/gains are always restored)')
    parser.add_argument('-f', '--filename',
                        help='output table path (default: timestamped)')
    args = parser.parse_args()

    client = fb.connect(args)
    client.disable_stream()
    snap = fb.snapshot_system(client)
    freqs = args.frequencies or fb.default_test_frequencies(snap, n=3)

    tables = []
    try:
        if not args.rf:
            fb.enter_digital_loopback(client)
        for f in freqs:
            print(f'--- test tone at {f/1e9:.6f} GHz ---')
            fb.set_test_tone(client, f)
            tables.append(fb.measure_ladder(
                client, span_bins=args.span_bins, n_points=args.n_points,
                n_cycles=args.n_cycles, compensate_filterbank=False))
    finally:
        fb.restore_system(client, snap, restore_tones=args.restore)

    table = fb.combine_tables(tables)
    tau = fb.fit_passband_delay(table)
    print(f"gain at span edges: {table['gain_db'][0]:+.3f} / "
          f"{table['gain_db'][-1]:+.3f} dB; repeat spread <= "
          f"{np.max(table['gain_db_std']):.4f} dB")
    if args.rf:
        cfg_tau = client.config.get('rf_frontend', {}).get('path_group_delay_ns')
        print(f'fitted path group delay: {tau*1e9:.1f} ns '
              f'(config path_group_delay_ns: {cfg_tau!r})')
    else:
        print(f'residual passband phase slope: {tau*1e9:+.2f} ns-equivalent '
              '(expect ~0 in digital loopback)')

    filename = args.filename or ('filterbank_response_'
                                 + time.strftime('%Y%m%d_%H%M%S') + '.txt')
    fb.save_table(filename, table, meta={
        'mode': 'rf-loopback' if args.rf else 'digital-loopback',
        'frequencies_hz': ' '.join(f'{f:.0f}' for f in freqs),
        'fitted_group_delay_ns': f'{tau*1e9:.2f}'})
    fb.plot_table(table, os.path.splitext(filename)[0] + '.png',
                  model_delay_s=(tau if args.rf else 0.0))


if __name__ == '__main__':
    main()
