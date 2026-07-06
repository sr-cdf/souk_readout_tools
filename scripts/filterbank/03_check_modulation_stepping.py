#!/usr/bin/env python
"""
Check that modulation actually steps the LO: does the IQ change point-to-point?

Motivated by a measurement that came back flat to ~0.001 dB / 0.1 mrad across
a whole channel: the modulation *tags* cycled (scheduler ran, buffers flipped)
but the tone gave no evidence of ever moving -- which turned out to be a
signal path bypassing both filterbanks, not a modulation fault. This script
gives a definitive MOVED/STATIC verdict per stepping path before any deeper
debugging:

  A   sw engine, 1-D offsets (broadcast)       -- the long-exercised path
  A2  sw engine, 2-D per-tone offsets          -- the measurement-script path
  B   fw engine, 4 LO slots (auto round-robin) -- hardware-validated on v7.11

A 250 kHz offset (~0.83 bins) shows a few dB of rolloff and/or a clear phase
shift through any filterbank-including loopback; identical IQ at every point
means that path's frequency words are not reaching the live LO. Compensation
is disabled for the checks (it would flatten the gain signature).

WHAT THIS SCRIPT CHANGES: the tone comb (single test tone; restored only with
--restore) and the modulation state (disabled on exit). It does NOT touch
loopback or gains -- run it in whatever coherent signal path you are
debugging (internal loopback or RF cable).

Example:
    python 03_check_modulation_stepping.py -C my_config.yaml
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fb_common as fb

GAIN_THRESHOLD_DB = 0.1
PHASE_THRESHOLD_RAD = 0.05


def _per_point_iq(client, num_points, samples_per_point, n_cycles):
    num = int(n_cycles) * int(num_points) * int(samples_per_point)
    data = client.parse_samples(client.get_samples(num))
    points = np.asarray(data['modulation_point'], dtype=int)
    settling = np.asarray(data['modulation_settling'], dtype=int)
    z = (np.asarray(data['i_data']['0000'], dtype=float)
         + 1j * np.asarray(data['q_data']['0000'], dtype=float))
    out = np.full(num_points, np.nan + 0j, dtype=complex)
    for p in range(1, num_points + 1):
        keep = (points == p) & (settling == 0)
        if np.any(keep):
            out[p - 1] = z[keep].mean()
    return out


def _report(name, z_points, offsets_hz):
    rel = z_points / z_points[0]
    gain_db = 20 * np.log10(np.abs(rel))
    phase = np.angle(rel)
    moved = (np.nanmax(np.abs(gain_db)) > GAIN_THRESHOLD_DB
             or np.nanmax(np.abs(phase)) > PHASE_THRESHOLD_RAD)
    print(f'{name}:')
    for p, off in enumerate(offsets_hz):
        print(f'  point {p+1} ({off/1e3:+8.1f} kHz): {gain_db[p]:+8.3f} dB  '
              f'{phase[p]:+8.4f} rad  (rel. point 1)')
    print(f'  -> {"MOVED" if moved else "STATIC"}  '
          f'(gain span {np.nanmax(np.abs(gain_db)):.4f} dB, '
          f'phase span {np.nanmax(np.abs(phase)):.4f} rad)\n')
    return moved


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    fb.add_connection_args(parser)
    parser.add_argument('--frequency', type=float,
                        help='test-tone frequency in Hz (default: centre of '
                             "the loaded comb's band)")
    parser.add_argument('--delta-hz', type=float, default=250e3,
                        help='probe offset in Hz (default 250e3 ~ 0.83 bins)')
    parser.add_argument('--restore', action='store_true',
                        help='restore the previous tone comb on exit')
    args = parser.parse_args()

    client = fb.connect(args)
    client.disable_stream()
    snap = fb.snapshot_system(client)
    freq = args.frequency or fb.default_test_frequencies(snap, n=1)[0]
    spp, n_cycles = 8, 6
    verdicts = {}
    try:
        fb.set_test_tone(client, freq)
        print(f'--- test tone at {freq/1e9:.6f} GHz ---')

        offs = [0.0, float(args.delta_hz)]
        client.enable_modulation(offsets=offs, samples_per_point=spp,
                                 n_settle=1, engine='sw',
                                 compensate_filterbank=False)
        z = _per_point_iq(client, len(offs), spp, n_cycles)
        client.disable_modulation(engine='sw')
        verdicts['A'] = _report(
            f'A  sw engine, 1-D offsets [0, {args.delta_hz/1e3:.0f} kHz]', z, offs)

        offs2d = np.asarray(offs)[:, None]          # (n_points, 1 tone)
        client.enable_modulation(offsets=offs2d, mod_indices=[0],
                                 samples_per_point=spp, n_settle=1, engine='sw',
                                 compensate_filterbank=False)
        z = _per_point_iq(client, len(offs), spp, n_cycles)
        client.disable_modulation(engine='sw')
        verdicts['A2'] = _report('A2 sw engine, per-tone 2-D offsets', z, offs)

        offs4 = [0.0, args.delta_hz / 3.0, 2.0 * args.delta_hz / 3.0,
                 float(args.delta_hz)]
        client.enable_modulation(offsets=offs4, samples_per_point=spp,
                                 n_settle=1, engine='fw', mode='auto',
                                 compensate_filterbank=False)
        z = _per_point_iq(client, len(offs4), spp, n_cycles)
        client.disable_modulation(engine='fw')
        verdicts['B'] = _report('B  fw engine, 4 LO slots (auto)', z, offs4)
    finally:
        fb.restore_system(client, snap, restore_tones=args.restore)

    print('verdicts:', ', '.join(f'{k}={"MOVED" if v else "STATIC"}'
                                 for k, v in verdicts.items()))
    sys.exit(0 if all(verdicts.values()) else 1)


if __name__ == '__main__':
    main()
