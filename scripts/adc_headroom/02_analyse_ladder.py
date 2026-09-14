#!/usr/bin/env python
"""Fit epsilon from an attenuation ladder taken with 01_attenuation_ladder.py.

Per ladder step the off-resonance capture is the readout floor in raw I/Q
counts, and the sweep taken at the same gain gives that step's ``|dz/dx|``.
Referring one through the other,

    S_x = S_n / |dz/dx|**2        (fractional frequency, 1/Hz)

turns the floor into the units an allocation is judged in.  Regressing
``10*log10(S_x)`` against the measured ADC fill gives

    epsilon = -d(referred noise, dB) / d(fill, dB)

which is what ``allocation_metrics`` and ``balance_tone_powers`` need.  The
division by ``|dz/dx|`` is the whole point: both it and the raw noise scale
with the RX gain, so a floor fixed at the ADC only shows up once the carrier
scaling has been divided out.

Prints the per-tone distribution and writes ``epsilon.png`` plus
``epsilon.npz`` into the ladder directory.

    PYTHONPATH=src python scripts/adc_headroom/02_analyse_ladder.py <ladder_dir>
"""
import argparse
import json
from pathlib import Path

import numpy as np

from souk_readout_tools import power_sweep as ps
from souk_readout_tools import timeseries as ts

BAND_HZ = (10.0, 100.0)    # science band; above 1/f, below the anti-alias corner
NPERSEG = 1024             # 0.59 Hz bins; ~10 overlapping segments in a
                           # 10 s capture, and ~150 bins inside BAND_HZ


def _step_point(step_dir, band):
    """One ladder step -> (fill_db, referred noise dB, |dz/dx| dB) per tone."""
    sweep = np.load(step_dir / 'sweep.npy', allow_pickle=True).item()
    samples = np.load(step_dir / 'samples_off.npy', allow_pickle=True).item()

    gain = ps.transducer_gain(sweep)['gain']           # counts per unit x
    fill_db = ps.accumulator_level_db(samples)         # measured ADC fill, dB

    # timestream_psds(tones=None) means the FIRST tone only, not all of them.
    psds = ts.timestream_psds(samples, format='iq',
                              tones=list(range(fill_db.size)),
                              psd_kwargs={'nperseg': NPERSEG})
    levels = ts.psd_band_levels(psds, bands={'band': band}, statistic='median')
    # Off-resonance readout noise is isotropic in the I/Q plane (measured), so
    # either quadrature is the additive noise that a frequency shift competes
    # with; average the two to halve the estimator variance.
    s_n = 0.5 * (levels['level']['i'][:, 0] + levels['level']['q'][:, 0])

    with np.errstate(divide='ignore', invalid='ignore'):
        s_x = s_n / np.square(np.where(gain > 0, gain, np.nan))
        noise_db = 10.0 * np.log10(np.where(s_x > 0, s_x, np.nan))
        gain_db = 20.0 * np.log10(np.where(gain > 0, gain, np.nan))
    return fill_db, noise_db, gain_db


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('ladder_dir', type=Path)
    parser.add_argument('--band', type=float, nargs=2, default=BAND_HZ,
                        metavar=('LO', 'HI'))
    args = parser.parse_args()

    manifest = json.loads((args.ladder_dir / 'manifest.json').read_text())
    steps = [s for s in manifest['steps'] if not s['saturated']]
    dropped = len(manifest['steps']) - len(steps)
    print(f'{len(steps)} usable steps'
          f'{f" ({dropped} dropped as saturated)" if dropped else ""}')
    if len(steps) < 3:
        raise SystemExit('need at least 3 unsaturated steps to fit epsilon')

    fill, noise, gain = [], [], []
    for step in steps:
        f_db, n_db, g_db = _step_point(args.ladder_dir / step['dir'],
                                       tuple(args.band))
        fill.append(f_db)
        noise.append(n_db)
        gain.append(g_db)
        print(f"  {manifest['level_knob']} {step['level_db']:5.1f} dB   "
              f'median fill {np.nanmedian(f_db):7.2f} dB   '
              f'median referred noise {np.nanmedian(n_db):8.2f} dB')

    fill = np.asarray(fill)
    noise = np.asarray(noise)
    level = np.array([s['level_db'] for s in steps], dtype=float)

    # A comb that compresses at the ADC keeps a raw snapshot peak that tracks
    # the attenuator 1:1 while the per-tone accumulated level stops responding.
    # That flattens the fill axis and fits a spuriously low epsilon, so show
    # the response per step before fitting anything.
    order = np.argsort(level)
    med = np.nanmedian(fill, axis=1)[order]
    print(f"\n{'level':>7} {'fill dB':>9} {'fill response':>14}   "
          "(needs ~1.00 dB/dB; less means ADC compression)")
    for i, j in enumerate(order):
        resp = ((med[i - 1] - med[i]) / (level[order][i] - level[order][i - 1])
                if i else np.nan)
        print(f"{level[j]:7.1f} {med[i]:9.2f} {resp:14.2f}")

    try:
        result = ps.adc_fill_response(fill, noise, level_db=level)
    except ValueError as exc:
        linear = [level[order][i] for i in range(1, len(order))
                  if (med[i - 1] - med[i]) / (level[order][i]
                                              - level[order][i - 1]) >= 0.8]
        start = (min(linear) if linear else level.max()) + 2.0
        raise SystemExit(
            f'\n{exc}\n\nThe raw ADC snapshot peak cannot see this: it tracks '
            'the attenuator 1:1 while the per-tone\naccumulated level '
            'saturates, because clipping scatters power out of the tone '
            'bins.\nRe-run with the ladder starting around '
            f'{start:.0f} dB of attenuation, i.e. set\nN_HOTTER = 0 and step '
            'colder from there, so every step sits in the linear region.'
        ) from exc
    if result.get('n_compressed'):
        print(f"\ndropped {result['n_compressed']} compressed step(s); "
              f"fitting the remaining {int(result['linear_steps'].sum())}")

    eps = result['epsilon'][result['valid']]
    print(f'\nepsilon over {result["valid"].sum()} tones: '
          f'median {result["epsilon_median"]:.3f}, '
          f'pct 5/25/75/95 = {np.round(np.percentile(eps, [5, 25, 75, 95]), 3)}')
    print(f'lever arm {np.nanmedian(result["lever_arm_db"]):.1f} dB, '
          f'residual rms {np.nanmedian(result["residual_rms_db"]):.2f} dB')
    # epsilon is a local slope: it tends to 0 at high fill, where the noise
    # entering before the attenuator dominates and rides down with the carrier,
    # and to 1 at low fill where the ADC term takes over. The ladder has to stay
    # clear of clipping, so it brackets the operating point at its hot end --
    # epsilon_high_fill is the half that describes how you actually run.
    print(f'  high fill (your operating end) {result["epsilon_high_fill"]:+.3f}'
          f'   low fill {result["epsilon_low_fill"]:+.3f}')

    # With a spread of tone powers, epsilon can be split by each tone's *own*
    # level. A quiet tone sits closer to the ADC floor than a loud one, so if
    # the ADC contributes at all it shows up here first -- and the quiet tail
    # is exactly where an allocation decision would bite.
    tone_level = fill[int(np.argmax(np.nanmedian(fill, axis=1)))]   # hottest step
    ok = result['valid'] & np.isfinite(tone_level)
    if ok.sum() >= 20 and np.ptp(tone_level[ok]) > 3.0:
        edges = np.percentile(tone_level[ok], [0, 20, 40, 60, 80, 100])
        print(f'\nepsilon vs the tone\'s own level '
              f'(spread {np.ptp(tone_level[ok]):.1f} dB):')
        print(f"  {'level band (dB)':>22} {'tones':>6} {'epsilon':>9}")
        for lo, hi in zip(edges[:-1], edges[1:]):
            sel = ok & (tone_level >= lo) & (tone_level <= hi)
            if sel.sum():
                print(f'  {lo:10.1f} to {hi:8.1f} {sel.sum():6d} '
                      f'{np.median(result["epsilon"][sel]):9.3f}')
        print('  a rise toward the quiet end means the ADC is starting to '
              'contribute there')
    print('\n  epsilon ~ 1 : floor fixed at the ADC. Aggregate fill is worth '
          'having and\n                the loud tones really are taxing the '
          'quiet ones.')
    print('  epsilon ~ 0 : floor rides the RX gain (LNA-dominated). The ADC '
          'constraint\n                costs nothing but clipping; do not '
          'bother balancing.')
    print(f'\nPass this to the allocator:  '
          f'ps.allocation_metrics(run, powers, epsilon='
          f'{result["epsilon_median"]:.2f})')

    np.savez(args.ladder_dir / 'epsilon.npz',
             fill_db=fill, noise_db=noise, gain_db=np.asarray(gain),
             level_db=np.array([s['level_db'] for s in steps]),
             **{k: v for k, v in result.items() if isinstance(v, np.ndarray)},
             epsilon_median=result['epsilon_median'])

    import matplotlib.pyplot as plt
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.5, 4.2))
    n_show = min(60, fill.shape[1])
    for k in range(n_show):
        ax1.plot(fill[:, k], noise[:, k], '-', lw=0.5, color='0.7')
    ax1.plot(np.nanmedian(fill, axis=1), np.nanmedian(noise, axis=1),
             'o-', color='tab:red', lw=1.6, label='median tone')
    ax1.set_xlabel('measured ADC fill (dB)')
    ax1.set_ylabel(r'referred noise $10\log_{10}S_x$ (dB)')
    ax1.set_title(f'ladder ({args.band[0]:g}-{args.band[1]:g} Hz)')
    ax1.legend(fontsize='small')
    ax1.grid(alpha=0.3)

    ax2.hist(eps, bins=40, color='tab:blue', alpha=0.85)
    for value, colour, label in ((0.0, 'tab:grey', 'LNA-limited'),
                                 (1.0, 'tab:green', 'ADC-limited'),
                                 (result['epsilon_median'], 'tab:red',
                                  f'median {result["epsilon_median"]:.2f}')):
        ax2.axvline(value, color=colour, ls='--', lw=1.2, label=label)
    ax2.set_xlabel(r'$\epsilon$ (dB noise per dB fill)')
    ax2.set_ylabel('tones')
    ax2.set_title('per-tone epsilon')
    ax2.legend(fontsize='small')
    ax2.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(args.ladder_dir / 'epsilon.png', dpi=110, bbox_inches='tight')
    print(f'\nwrote {args.ladder_dir / "epsilon.png"} and epsilon.npz')


if __name__ == '__main__':
    main()
