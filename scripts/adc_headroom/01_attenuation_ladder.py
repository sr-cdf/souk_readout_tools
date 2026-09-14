#!/usr/bin/env python
"""How many dB of readout noise does ADC fill actually buy?  (measures epsilon)

The tones share one RX attenuator, so every allocation decision trades one
tone's ADC level against every other tone's.  The exchange rate is a single
number:

    epsilon = -d(referred noise PSD, dB) / d(ADC fill, dB)

    epsilon = 1   the floor is fixed at the ADC.  Every dB of fill is a dB of
                  noise, so aggregate ADC budget is precious and the loud tones
                  really are taxing the quiet ones.
    epsilon = 0   the floor rides the RX gain (an LNA-dominated chain).
                  Attenuation moves carrier and noise together, the ADC
                  constraint costs nothing but clipping, and there is no
                  balancing problem at all.

Anything in between is a mixture, and it is a property of *this* readout chain
and comb -- not of the detectors -- so it has to be measured per system rather
than assumed.  ``souk_readout_tools`` never defaults it silently: it is an
explicit argument to ``allocation_metrics`` and ``balance_tone_powers``.

Method.  Hold the TX drive, the tone frequencies and the comb completely fixed
so the detectors see exactly the same thing throughout, and step *only* the RX
attenuation.  At each step take

  * a calibration sweep, which gives |dz/dx| at that gain, and
  * an off-resonance timestream, which is the readout floor with no detector
    noise in it (on resonance the detector noise dilutes epsilon toward 0 and
    you measure the array, not the chain), and
  * optionally an on-resonance timestream, which shows the benefit actually
    delivered at the operating point.

Referring the off-resonance noise through that step's own |dz/dx| gives the
noise in fractional-frequency units; regressing it against the measured ADC
fill gives epsilon.  Both quantities move together by construction, which is
the point: nothing else in the chain is allowed to change.

Traps this script is built to avoid, all of which quietly destroy the
measurement:

  * Never call an optimiser inside the loop.  ``set_tone_powers`` defaults to
    ``optimise_dynamic_range=True``, and ``rx_policy='maximise'`` re-optimises
    the exact knob being stepped.  Tones are set once, before the ladder.
  * Refresh the ADC background calibration after every attenuation change and
    freeze it for the capture.  The input level has moved; a stale calibration
    puts a step into the band being measured.
  * Record the saturation state at every step -- but do not rely on it.  The
    raw ADC snapshot peak tracks the attenuator 1:1 even while the comb is
    compressing, because clipping scatters power out of the *tone bins* rather
    than out of the total.  The test that works is the per-tone accumulated
    fill: it must fall ~1 dB per dB of added attenuation.  This script prints
    that response as it goes and 02_analyse_ladder.py refuses to fit steps
    where it has collapsed.  A first run at 0-10 dB attenuation returned
    responses of -0.01/0.20/0.56/0.89 dB/dB and a confident epsilon of 0.006
    that was entirely compression.
  * The RX attenuator is 0-31.5 dB in 0.5 dB steps and the ADC DSA 0-27 dB in
    1 dB steps.  LEVEL_KNOB picks which one to walk; running both and comparing
    localises anything entering between them, in the RF frontend.

Leaves the attenuation as it was found.

    PYTHONPATH=src python scripts/adc_headroom/01_attenuation_ladder.py

Analyse with ``02_analyse_ladder.py`` in the same directory.
"""
import json
import time
from pathlib import Path

import numpy as np

from souk_readout_tools import power_sweep as ps
from souk_readout_tools.client.readout_client import ReadoutClient

ADDRESS = '10.11.11.11'
PORT = 10000

OUTDIR = time.strftime('attenuation_ladder_%Y%m%d-%H%M%S')
LEVEL_KNOB = 'atten'       # 'atten' RF frontend attenuator (0-31.5, 0.5 dB steps)
                           # 'dsa'   ADC step attenuator    (0-27,   1   dB steps)
STEP_DB = 2.0              # ladder step
N_COLDER = 8               # steps of MORE attenuation than the current setting
N_HOTTER = 0               # steps of LESS; guarded, stops early if it clips.
                           # Keep at 0 unless you already know the comb is well
                           # clear of compression at the current setting -- see
                           # the fill-response check below.
CAPTURE_ON_RESONANCE = True   # also capture at the operating point each step
POWER_SPREAD_DB = 10.0     # 0 keeps the comb as found. Non-zero applies a random
                           # spread to the tone powers before the ladder
                           # (restored afterwards), so epsilon can be split by
                           # each tone's own level -- a quiet tone sits closer to
                           # the ADC floor than a loud one and need not share its
                           # epsilon.
POWER_SPREAD_ONE_SIDED = True   # spread downward only, i.e. offsets in
                           # [-POWER_SPREAD_DB, 0].  With BEST_POWER_SWEEP set
                           # every tone starts *at* its own best power, which
                           # sits just below bifurcation, so a symmetric spread
                           # drives the upper half of the comb past it: the
                           # 2026-08-11 run at a flat -90 dBm came back with
                           # median anl 0.854 and 54% of tones bifurcated, and
                           # its |dz/dx| -- hence its epsilon -- was measured on
                           # Duffing-steepened resonators.  Downward-only keeps
                           # the level lever arm without ever exceeding the
                           # drive the science uses.
POWER_SPREAD_SEED = 0      # so a repeat run is directly comparable

# Set every tone to *find_best_power*'s own choice from a finished
# ``power_sweep_both`` directory before the ladder.  ``None`` keeps the comb as
# found.  Deliberately the raw per-tone best power and **not** what
# ``optimise_tone_powers`` or ``balance_tone_powers`` make of it: both of those
# reallocate against an epsilon, which is the quantity this measurement exists
# to provide, so feeding their output back in would measure the allocator.
# e.g. '<run_root>/20260810/BB_13.1K/power_sweep_both'
BEST_POWER_SWEEP = None
QI_MIN = 1e3               # a tone whose fitted Qi is below this gets the
                           # median power rather than its own, as in bb_ladder

SWEEP_SPAN_HZ = 200e3
SWEEP_POINTS = 401
SWEEP_SAMPLES_PER_POINT = 20
NUM_SAMPLES = 6000         # off/on timestream length; 10 s at the 600 Hz
                           # accumulation rate, which is plenty for a band
                           # average -- the captures dominate the run time
OFF_RESONANCE_HZ = 1.5e6   # how far to park tones for the readout-floor capture

ADC_CAL_SETTLE_S = 2.0
SAT_ITERATIONS = 250

ATTEN_MAX_DB = 31.5
DSA_MAX_DB = 27.0


def _reading(reply, key):
    """dB out of a client getter's reply.

    ``get_rx_attenuation`` and ``get_adc_dsa`` return the whole server
    envelope -- ``{'status': ..., 'result': {'<key>': dB}}`` -- rather than
    the value their docstrings promise, so unwrap it here.  A bare number is
    passed through, in case that is ever tidied up.
    """
    if isinstance(reply, dict):
        return float((reply.get('result') or reply)[key])
    return float(reply)


def _knob(client):
    """Getter/setter/limit for whichever RX level knob is being stepped."""
    if LEVEL_KNOB == 'atten':
        return (lambda: _reading(client.get_rx_attenuation(),
                                 'rx_attenuation_db'),
                client.set_rx_attenuation, ATTEN_MAX_DB)
    if LEVEL_KNOB == 'dsa':
        return (lambda: _reading(client.get_adc_dsa(), 'adc_dsa_db'),
                client.set_adc_dsa, DSA_MAX_DB)
    raise ValueError("LEVEL_KNOB must be 'atten' or 'dsa'")


def _refresh_adc_cal(client):
    """Let the ADC background calibration re-adapt, then freeze it for a capture."""
    client.set_cal_freeze(False)
    time.sleep(ADC_CAL_SETTLE_S)
    client.set_cal_freeze(True)


def _capture(client, num_samples):
    """One frozen-calibration timestream, parsed."""
    raw = client.get_samples(num_samples, incl_system_info=True)
    return client.parse_samples(raw)


def main():
    client = ReadoutClient(address=ADDRESS, request_port=PORT)
    get_knob, set_knob, knob_max = _knob(client)

    centers = np.asarray(client.get_tone_frequencies(), dtype=float)
    original_powers = np.asarray(client.get_tone_powers(), dtype=float)
    powers = original_powers

    if BEST_POWER_SWEEP:
        best = ps.best_power_arrays(
            str(Path(BEST_POWER_SWEEP) / 'analysis' / 'best_power.json'),
            tone_count=centers.size)
        chosen = np.asarray(best['power_dbm'], dtype=float).copy()
        qi = np.asarray(best['Qi'], dtype=float)
        bad = ~np.isfinite(chosen) | ~np.isfinite(qi) | (qi < QI_MIN)
        chosen[bad] = np.nanmedian(chosen)
        print(f'setting best powers from {BEST_POWER_SWEEP}: '
              f'median {np.nanmedian(chosen):.2f} dBm, '
              f'{np.nanmin(chosen):.1f} .. {np.nanmax(chosen):.1f}, '
              f'{int(bad.sum())} tone(s) on the median')
        client.set_tone_powers(chosen)
        powers = np.asarray(client.get_tone_powers(), dtype=float)

    if POWER_SPREAD_DB:
        # A flat comb gives one epsilon for one level.  Spreading the powers
        # lets the fit be split by each tone's *own* level, which is the thing
        # that matters: a quiet tone sits closer to the ADC floor than a loud
        # one, so epsilon need not be the same for both.  Offsets are random
        # rather than a ramp so the level is not confounded with frequency,
        # and seeded so a repeat run is comparable.
        rng = np.random.default_rng(POWER_SPREAD_SEED)
        low = -POWER_SPREAD_DB
        high = 0.0 if POWER_SPREAD_ONE_SIDED else POWER_SPREAD_DB
        offsets = rng.uniform(low, high, centers.size)
        powers = powers + offsets
        print(f'applying a {low:.0f}..{high:+.0f} dB random power spread '
              f'(seed {POWER_SPREAD_SEED}); aggregate moves '
              f'{10 * np.log10(np.mean(10 ** (offsets / 10))):+.1f} dB')
        client.set_tone_powers(powers)
        powers = np.asarray(client.get_tone_powers(), dtype=float)

    # Read the knob only after any power change, so the ladder starts from
    # whatever the power setter left the RX chain at.
    start_db = float(get_knob())
    print(f'{len(centers)} tones, {LEVEL_KNOB} starts at {start_db:.1f} dB')
    print(f'tone powers {powers.min():.1f} .. {powers.max():.1f} dBm '
          f'(spread {powers.max() - powers.min():.1f} dB, '
          'held fixed for the whole ladder)')

    # Hotter first so the guard stops before wasting time on the cold end.
    hotter = [start_db - STEP_DB * (i + 1) for i in range(N_HOTTER)]
    colder = [start_db + STEP_DB * (i + 1) for i in range(N_COLDER)]
    ladder = [v for v in sorted(hotter) if v >= 0.0] + [start_db] + \
             [v for v in colder if v <= knob_max]

    fills = []
    outdir = Path(OUTDIR)
    outdir.mkdir(parents=True, exist_ok=True)
    manifest = {
        'level_knob': LEVEL_KNOB,
        'start_db': start_db,
        'ladder_db': ladder,
        'tone_frequencies_hz': centers.tolist(),
        'tone_powers_dbm': powers.tolist(),
        'original_tone_powers_dbm': original_powers.tolist(),
        'power_spread_db': POWER_SPREAD_DB,
        'off_resonance_hz': OFF_RESONANCE_HZ,
        'num_samples': NUM_SAMPLES,
        'date': time.strftime('%Y-%m-%d %H:%M:%S'),
        'steps': [],
    }

    try:
        for index, value_db in enumerate(ladder):
            print(f'\n--- step {index}: {LEVEL_KNOB} = {value_db:.1f} dB ---')
            set_knob(value_db)
            _refresh_adc_cal(client)

            # The client returns the raw server response, not a (flag, levels)
            # tuple -- the firmware_lib function it wraps returns the tuple.
            response = client.check_input_saturation(iterations=SAT_ITERATIONS)
            saturated = bool(response.get('result'))
            levels = response.get('details') or {}
            peak_fs = max(
                abs(float(levels.get(key, 0.0)))
                for key in ('imax_fs', 'imin_fs', 'qmax_fs', 'qmin_fs')
            )
            flag = '  *** SATURATED ***' if saturated else ''
            print(f'    ADC peak {peak_fs:.3f} FS{flag}')
            if saturated and value_db < start_db:
                # Hotter than we started and clipping: everything hotter still
                # will clip too, so stop rather than record compression.
                print('    clipped: stopping the hot end of the ladder here')
                continue

            client.set_tone_frequencies(centers)
            client.perform_sweep(centers, SWEEP_SPAN_HZ, SWEEP_POINTS,
                                 SWEEP_SAMPLES_PER_POINT,
                                 refresh_adc_cal=True,
                                 adc_cal_settle_time=ADC_CAL_SETTLE_S)
            client.wait_for_sweep()
            sweep = client.parse_sweep_data(client.get_sweep_data())

            client.set_tone_frequencies(centers + OFF_RESONANCE_HZ)
            _refresh_adc_cal(client)
            samples_off = _capture(client, NUM_SAMPLES)

            fill_db = float(np.nanmedian(
                ps.accumulator_level_db(samples_off)))
            if fills:
                response = ((fills[-1][1] - fill_db)
                            / max(value_db - fills[-1][0], 1e-9))
                warn = '   <-- COMPRESSED, comb is clipping' if response < 0.8 else ''
                print(f'    fill {fill_db:.2f} dB   response '
                      f'{response:+.2f} dB/dB{warn}')
            else:
                print(f'    fill {fill_db:.2f} dB')
            fills.append((value_db, fill_db))

            samples_on = None
            if CAPTURE_ON_RESONANCE:
                client.set_tone_frequencies(centers)
                _refresh_adc_cal(client)
                samples_on = _capture(client, NUM_SAMPLES)

            step_dir = outdir / f'step_{index:02d}'
            step_dir.mkdir(exist_ok=True)
            np.save(step_dir / 'sweep.npy', sweep, allow_pickle=True)
            np.save(step_dir / 'samples_off.npy', samples_off, allow_pickle=True)
            if samples_on is not None:
                np.save(step_dir / 'samples_on.npy', samples_on, allow_pickle=True)

            manifest['steps'].append({
                'index': index,
                'level_db': float(value_db),
                'saturated': bool(saturated),
                'adc_levels': {k: float(v) for k, v in levels.items()
                               if isinstance(v, (int, float))},
                'median_fill_db': fill_db,
                'dir': step_dir.name,
            })
            with (outdir / 'manifest.json').open('w', encoding='utf-8') as handle:
                json.dump(manifest, handle, indent=2)
    finally:
        print(f'\nrestoring {LEVEL_KNOB} to {start_db:.1f} dB')
        set_knob(start_db)
        if POWER_SPREAD_DB or BEST_POWER_SWEEP:
            print('restoring the tone powers the comb was found with')
            client.set_tone_powers(original_powers)
        client.set_tone_frequencies(centers)
        _refresh_adc_cal(client)

    linear = sum(1 for i in range(1, len(fills))
                 if (fills[i - 1][1] - fills[i][1])
                 / max(fills[i][0] - fills[i - 1][0], 1e-9) >= 0.8)
    print(f'\nwrote {outdir}  ({len(manifest["steps"])} steps, '
          f'{linear} with a linear fill response)')
    if linear < 3:
        print('WARNING: fewer than 3 linear steps -- the comb is compressing '
              'at the ADC.\n         Re-run starting colder; epsilon cannot '
              'be fitted from this.')
    print('analyse with:  python scripts/adc_headroom/02_analyse_ladder.py '
          f'{outdir}')


if __name__ == '__main__':
    main()
