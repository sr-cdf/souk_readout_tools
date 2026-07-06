#!/usr/bin/env python
"""
Verify the wired-in filterbank compensation on hardware (standalone).

Measures the same single-tone ladder twice -- compensation OFF then ON (the
per-request ``compensate_filterbank`` knob) -- and checks that the compensated
readout is flat where the raw response rolls off. This is the acceptance test
for the v1.6.1 compensation: run it in digital loopback after any firmware or
package update.

PASS criteria
  - raw run shows the expected rolloff (model agreement, sanity that the
    measurement itself works);
  - HARDWARE part: the compensated run matches raw + the TX drive boost
    (clipped at full scale) -- the RX LO scale word is inert in current
    firmware, so this is all the hardware can do;
  - SOFTWARE part: applying the server-reported per-point
    'readout_correction' factors (what modulation.group_cycles applies on
    the normal data path) flattens the readout to within tolerance out to
    --check-bins;
  - phase checked only in --rf mode with the path group delay configured
    (in digital loopback a configured RF delay would mis-correct; force
    amplitude-only there with filterbank_group_delay_override_ns: 0, or use
    the default 0-delay config).

WHAT THIS SCRIPT CHANGES: same as 01_measure_channel_response.py -- internal
loopback + digital gains (always restored; gains re-optimised with
maximise_tx_power(digital_only=True) after switching loopback), the tone comb
(single test tone; restored only with --restore), modulation state.

Examples:
    python 02_verify_compensation.py -C my_config.yaml
    python 02_verify_compensation.py -C my_config.yaml --rf --restore
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fb_common as fb
from souk_readout_tools import firmware_lib


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    fb.add_connection_args(parser)
    parser.add_argument('--rf', action='store_true',
                        help='RF loopback mode (cable): also check the phase '
                             'correction against the configured path delay')
    parser.add_argument('--frequency', type=float,
                        help='test-tone frequency in Hz (default: centre of '
                             "the loaded comb's band)")
    parser.add_argument('--span-bins', type=float, default=0.9)
    parser.add_argument('--check-bins', type=float, default=0.8,
                        help='flatness is asserted out to +/- this offset '
                             '(default 0.8; image reject, not gain, limits '
                             'operation beyond ~0.7)')
    parser.add_argument('--gain-tol-db', type=float, default=0.1)
    parser.add_argument('--phase-tol-mrad', type=float, default=30.0)
    parser.add_argument('--restore', action='store_true',
                        help='restore the previous tone comb on exit')
    args = parser.parse_args()

    client = fb.connect(args)
    client.disable_stream()
    snap = fb.snapshot_system(client)
    freq = args.frequency or fb.default_test_frequencies(snap, n=1)[0]

    try:
        if not args.rf:
            fb.enter_digital_loopback(client)
        print(f'--- test tone at {freq/1e9:.6f} GHz ---')
        fb.set_test_tone(client, freq)
        print('measuring RAW response (compensate_filterbank=False)...')
        raw = fb.measure_ladder(client, span_bins=args.span_bins,
                                compensate_filterbank=False)
        print('measuring COMPENSATED response (compensate_filterbank=True)...')
        comp = fb.measure_ladder(client, span_bins=args.span_bins,
                                 compensate_filterbank=True)
    finally:
        fb.restore_system(client, snap, restore_tones=args.restore)

    x = raw['offset_bins']
    check = np.abs(x) <= args.check_bins
    failures = []

    def report(name, cond, detail):
        print(f'  [{"PASS" if cond else "FAIL"}] {name}: {detail}')
        if not cond:
            failures.append(name)

    print(f"\n{'offset':>8} {'raw dB':>9} {'comp dB':>9} {'raw mrad':>9} {'comp mrad':>10}")
    for i in np.linspace(0, len(x) - 1, 13).astype(int):
        print(f'{x[i]:+8.3f} {raw["gain_db"][i]:+9.3f} {comp["gain_db"][i]:+9.3f} '
              f'{raw["phase_rad"][i]*1e3:+9.2f} {comp["phase_rad"][i]*1e3:+10.2f}')
    print()

    raw_edge = min(raw['gain_db'][0], raw['gain_db'][-1])
    report('raw response rolls off (measurement sane)', raw_edge < -1.0,
           f'raw gain at +/-{args.span_bins} bins = {raw_edge:+.2f} dB')

    # Hardware part: TX drive boost only (clipped at full scale); the RX LO
    # scale word is inert in current firmware.
    boost = np.minimum(1.0 / firmware_lib.filterbank_single_bank_gain(x),
                       1.0 / fb.TEST_TONE_AMPLITUDE)
    boost_db = 20 * np.log10(boost) - 20 * np.log10(np.interp(0.0, x, boost))
    hw_err = np.max(np.abs(comp['gain_db'] - (raw['gain_db'] + boost_db))[check])
    report('hardware TX correction as expected', hw_err <= 0.15,
           f'max |comp - (raw + TX boost)| = {hw_err:.3f} dB over '
           f'+/-{args.check_bins} bins (tol 0.15)')

    # Software part: the server-reported per-point readout_correction factors
    # (applied by modulation.group_cycles on the normal data path).
    if comp['readout_correction'] is None:
        report('software readout correction reported', False,
               "state carries no 'readout_correction' -- server too old or "
               'compensation off')
        corrected_span = np.inf
    else:
        corr_db = 20 * np.log10(comp['readout_correction'])
        corr_db -= np.interp(0.0, x, corr_db)
        corrected = comp['gain_db'] + corr_db
        corrected_span = np.max(np.abs(corrected[check]))
    report('software-corrected readout flat', corrected_span <= args.gain_tol_db,
           f'max |gain| {corrected_span:.3f} dB over +/-{args.check_bins} bins '
           f'(tol {args.gain_tol_db})')
    if args.rf:
        # De-slope both runs with the same fitted delay: compensation removes
        # only the image turn-over, the plain delay slope legitimately remains.
        tau = fb.fit_passband_delay(comp)
        deslope = comp['phase_rad'] - (-2 * np.pi * comp['bin_spacing_hz']
                                       * tau * x)
        phase_span = np.max(np.abs(deslope[check])) * 1e3
        report('compensated phase flat (de-sloped)',
               phase_span <= args.phase_tol_mrad,
               f'max {phase_span:.1f} mrad over +/-{args.check_bins} bins '
               f'(tol {args.phase_tol_mrad}); if roughly DOUBLE the raw edge '
               'phase, the configured path delay sign is wrong')
    else:
        print('  (phase check skipped: digital loopback; raw edge phase '
              f'{max(abs(raw["phase_rad"][0]), abs(raw["phase_rad"][-1]))*1e3:.1f} mrad)')

    print('\nRESULT:', 'PASS' if not failures else f'FAIL -> {failures}')
    sys.exit(1 if failures else 0)


if __name__ == '__main__':
    main()
