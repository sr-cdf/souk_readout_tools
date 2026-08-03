#!/usr/bin/env python
"""Plot the key results from a quantisation_noise_*.csv run.

Takes the newest CSV in the working directory, or a path as the first argument.
Safe to run while the test is still going: rows are flushed per step, so a
partial file just gives shorter ladders.

WHAT THE PLOTTED NUMBER IS.  Two aggregations sit between the raw PSD and every
point on these plots, and both change the answer, so both are named on the
figure and switchable here:

  PER_TONE      how one tone's in-band PSD becomes one number.  'floor' takes
                the MEDIAN across the in-band bins (discrete lines excluded),
                'rms' the MEAN (lines included).  On this hardware they
                disagree by up to 8 dB along a ladder, so the choice is not
                cosmetic -- if two ladders disagree, look at both.
  ACROSS_TONES  how the tones of a comb become one number: median or mean.  The
                shaded band shows the 16th-84th percentile across tones, so the
                spread this hides stays visible.

AND THE FLOOR IS NOT WHITE.  5-18 Hz runs 10-22 dB noisier than 68-250 Hz, and
most in-band bins are at the top end, so a single in-band number is dominated by
the quiet end of a sloped spectrum and understates what a low-frequency science
band sees.  The sub-band panel shows the slope; treat every other panel as "the
high end of the band" unless you have checked the sub-bands too.

One panel per ladder.  The x axis is the hardware setting that was stepped, in
its own units; the caption says what else moved to compensate and what was held.
The y axis is always the same measured quantity: in-band noise-to-carrier as the
IEEE single-sideband L(f) = S(f) / 2, in dBc/Hz -- NEGATIVE, and UP THE PANEL
MEANS NOISIER.  (The CSV stores it the other way up, as dB below the carrier, so
this script flips it.)  Runs recorded before the single-sideband change read
3.01 dB noisier; slopes are unaffected, so do not mix the two on one panel.

The dashed line on each curve starts at that curve's own reference rung and
shows where the noise would go if the source that ladder attacks were the only
thing setting it: 1 dB of noise per dB of gain given up.
"""
import csv
import glob
import sys
import textwrap
from collections import defaultdict

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.ticker import FuncFormatter, MaxNLocator

COMPONENT = 'complex'      # complex (mag and phase in quadrature), mag, or phase
PER_TONE = 'floor'         # 'floor' = median across in-band bins, 'rms' = mean
ACROSS_TONES = 'median'    # or 'mean'
BAND_HZ = (5.0, 250.0)     # BAND_HZ of the run; not recorded in the CSV
RAMP = ('#86b6ef', '#3987e5', '#1c5cab', '#0d366b')   # ordinal blue, validated
ACCENT = '#eb6834'
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e6e5e1'
CRITICAL = '#d03b3b'
OUTFILE = 'quantisation_noise.png'

PER_TONE_TEXT = {'floor': 'median across the in-band bins, discrete lines excluded',
                 'rms': 'mean across the in-band bins, discrete lines included'}
PER_TONE_SHORT = {'floor': 'median across in-band bins',
                  'rms': 'mean across in-band bins'}
# ladder, x column, x axis label, log2 x, what the ladder did
PANELS = (
    ('psb_scale', 'amp', 'tone amplitude word', True,
     'amplitude halved per step and psb_scale doubled to compensate, so the '
     'filterbank runs colder at an unchanged DAC drive'),
    # Same stepped variable as the panel above -- amplitude -- so the two are
    # directly comparable and the only difference is where the compensating
    # gain was taken: after the filterbank there, inside it here.
    ('fftshift', 'amp', 'tone amplitude word', True,
     'amplitude halved per step and one PSB shift stage given up to compensate, '
     'again at an unchanged DAC drive'),
    ('drive', 'psb_scale', 'psb_scale applied (after the filterbank)', True,
     'psb_scale halved per step with amplitude and shift schedule fixed, so the '
     'DAC drive itself falls'),
    ('rx_gain', 'pfb_stages', 'PFB shift stages (more = less RX gain)', False,
     'one PFB shift stage added per step with the whole TX chain untouched'),
    ('adc_level', 'rx_atten_db', 'RX attenuation (dB)', False,
     'attenuation up with the TX untouched, so the ADC input falls while the '
     'DAC output does not: only noise entering after it (RF amp, ADC, RX '
     'digital) stays put while the carrier falls'),
    ('dac_level', 'psb_scale', 'psb_scale applied (after the filterbank)', True,
     'psb_scale down and attenuation down together, holding the ADC input '
     'constant: only noise entering before it (DAC, TX chain, cable, LNA) '
     'stays put while the carrier falls'),
)
TICKS = {'amp': lambda v, _: '1' if v >= 1 else f'1/{round(1 / v)}',
         'psb_scale': lambda v, _: f'{v:.3g}'}
EDGES = np.geomspace(*BAND_HZ, 4)                   # the run's three sub-bands
SUBBANDS = tuple(zip(('lo', 'mid', 'hi'), EDGES[:-1], EDGES[1:]))

path = (sys.argv[1] if len(sys.argv) > 1
        else sorted(glob.glob('quantisation_noise_*.csv'))[-1])
rows = []
for raw in csv.DictReader(open(path)):
    row = {}
    for key, value in raw.items():
        try:
            row[key] = float(value)
        except ValueError:
            row[key] = value
    rows.append(row)

# One group per (comb, ladder, step); every group holds one row per tone.
groups = defaultdict(list)
for row in rows:
    groups[(row['num_tones'], row['test'], row['step'])].append(row)
combs = sorted({key[0] for key in groups})
colours = dict(zip(combs, RAMP))


def across(values):
    return np.median(values) if ACROSS_TONES == 'median' else np.mean(values)


def curve(comb, ladder, column):
    """Across-tone aggregate per step: (steps, value, low, high).

    low/high are the 16th and 84th percentiles across the tones of the comb --
    the spread the single plotted number hides.
    """
    keys = sorted(k for k in groups if k[0] == comb and k[1] == ladder)
    per_step = [np.array([r[column] for r in groups[k]]) for k in keys]
    return (np.array([k[2] for k in keys]),
            np.array([across(v) for v in per_step]),
            np.array([np.percentile(v, 16) for v in per_step]),
            np.array([np.percentile(v, 84) for v in per_step]))


def noise(comb, ladder, column=None):
    """Noise-to-carrier, single-sideband dBc/Hz, negative: up is noisier."""
    _, value, low, high = curve(comb, ladder, column or f'{COMPONENT}_{PER_TONE}')
    return -value, -high, -low                     # negating swaps the band ends


def knob_column(ladder, default):
    """Which attenuation the run stepped: the RF attenuator or the ADC DSA.

    LEVEL_KNOB in the test script picks one; the CSV records both, so resolve it
    from whichever actually moved rather than assuming.
    """
    if default != 'rx_atten_db':
        return default
    for candidate in ('rx_atten_db', 'adc_dsa_db'):
        if any(candidate in r for r in rows[:1]) and any(
                np.ptp(curve(c, ladder, candidate)[1]) > 0 for c in combs
                if len(curve(c, ladder, candidate)[1]) > 1):
            return candidate
    return default


def gain_given_up(comb, ladder):
    """dB of gain given up at each step.

    For most ladders that is the measured carrier drop.  Not for dac_level: it
    holds the ADC input -- and so the carrier -- constant on purpose, so the
    carrier drop is ~0 and would divide the slope by nothing.  What is given up
    there is DAC drive, which is the nominal 6.02 dB per step.
    """
    steps, signal, *_ = curve(comb, ladder, 'signal_db')
    if ladder in ('psb_scale', 'fftshift', 'dac_level'):
        return 6.0206 * (steps - steps[0])
    return signal[0] - signal          # drive, rx_gain, adc_level


def two_source_fit(x_db, noise_db):
    """noise = other + this_source * 10**(x/10), both returned as dBc/Hz.

    A big residual means the ladder is NOT a fixed term plus a term rising 1:1 --
    on this hardware the level ladders are a clean 0.5 dB/dB power law instead,
    which no two-source sum can produce.
    """
    if len(x_db) < 3 or np.ptp(x_db) < 1e-6:       # a ladder that never moved
        return np.nan, np.nan, np.nan
    design = np.vstack([np.ones_like(x_db), 10 ** (x_db / 10)]).T
    try:
        (other, this), *_ = np.linalg.lstsq(design, 10 ** (noise_db / 10), rcond=None)
    except np.linalg.LinAlgError:
        return np.nan, np.nan, np.nan
    if other <= 0 or this <= 0:                    # no two-source split to make
        return np.nan, np.nan, np.nan
    fit = 10 * np.log10(design @ [other, this])
    return (10 * np.log10(other), 10 * np.log10(this),
            np.abs(fit - noise_db).max())


def slope(x_db, noise_db):
    """dB of noise per dB of gain given up over the whole ladder; 1 = the source
    under test sets the noise on its own, 0 = it contributes nothing."""
    if len(x_db) < 2 or np.ptp(x_db) < 1e-6:
        return np.nan
    return np.polyfit(x_db, noise_db, 1)[0]


# ----------------------------------------------------------------- the numbers
print(f'{path}: {len(rows)} rows, combs {[int(c) for c in combs]}, '
      f'{COMPONENT} component, {BAND_HZ[0]:g}-{BAND_HZ[1]:g} Hz\n'
      f'per tone: {PER_TONE} ({PER_TONE_TEXT[PER_TONE]})\n'
      f'across tones: {ACROSS_TONES}\n'
      'noise is dBc/Hz in the usual sense: more negative is quieter')
flagged = sorted({(int(k[0]), k[1], int(k[2]), groups[k][0]['flags'])
                  for k in groups if groups[k][0]['flags']})
print(f'flagged steps (excluded from any conclusion): {flagged or "none"}')

for comb in combs:
    print(f'\n{int(comb):5d} tone(s)')
    reference, repeat = noise(comb, 'psb_scale')[0], noise(comb, 'reference_repeat')[0]
    if len(reference) and len(repeat):
        print(f'  reference {reference[0]:7.1f} dBc/Hz, repeated at the end '
              f'{repeat[0]:7.1f} -> repeatability {abs(repeat[0]-reference[0]):.2f} dB')
    shape = [-curve(comb, 'psb_scale', f'{COMPONENT}_{b}')[1][0] for b, *_ in SUBBANDS]
    print(f'  spectrum at that setting: {shape[0]:.1f} / {shape[1]:.1f} / '
          f'{shape[2]:.1f} dBc/Hz over {EDGES[0]:.0f}-{EDGES[1]:.0f} / '
          f'{EDGES[1]:.0f}-{EDGES[2]:.0f} / {EDGES[2]:.0f}-{EDGES[3]:.0f} Hz '
          f'-> {shape[2]-shape[0]:+.1f} dB across the band, not white')
    for ladder, *_ in PANELS:
        y = noise(comb, ladder)[0]
        if len(y) < 2:
            continue
        x = gain_given_up(comb, ladder)
        drift = np.ptp(curve(comb, ladder, 'signal_db')[1])
        other, this, resid = two_source_fit(x, y)
        alt = -curve(comb, ladder, f'{COMPONENT}_'
                     + ('rms' if PER_TONE == 'floor' else 'floor'))[1]
        print(f'  {ladder:10s} {len(y)} steps over {x[-1]:5.1f} dB of gain: '
              f'noise {y[0]:7.1f} -> {y[-1]:7.1f} dBc/Hz, slope {slope(x, y):+5.2f} '
              f'dB/dB ({slope(x, alt):+5.2f} on the other statistic), '
              f'two-source fit: other {other:7.1f}, this {this:7.1f}, '
              f'resid {resid:5.2f} dB' + ('  ** model rejected **' if resid > 3
                                          else '')
              + (f'  ** carrier drifted {drift:.2f} dB **'
                 if ladder in ('psb_scale', 'fftshift') and drift > 0.5 else ''))

# ------------------------------------------------------------------ the figure
fig, axes = plt.subplots(4, 3, figsize=(16.5, 18), constrained_layout=True)
for ax in axes.flat:
    ax.grid(True, color=GRID, lw=0.6)
    ax.set_axisbelow(True)
    ax.tick_params(colors=MUTED, labelsize=9)
    for side in ('top', 'right'):
        ax.spines[side].set_visible(False)
    for side in ('left', 'bottom'):
        ax.spines[side].set_color(GRID)


def caption(ax, text, title):
    """Title plus a wrapped caption above the axes, without them colliding."""
    lines = textwrap.fill(text, 62)
    ax.set_title(title, color=INK, fontsize=11, loc='left',
                 pad=16 + 9 * lines.count('\n'))
    ax.text(0, 1.015, lines, transform=ax.transAxes, fontsize=8, color=MUTED,
            va='bottom')


NOISE_LABEL = 'noise-to-carrier (dBc/Hz, SSB), up is noisier'
clipped = False
for ax, (ladder, xcol, xlabel, logx, what) in zip(axes.flat, PANELS):
    xcol = knob_column(ladder, xcol)
    if xcol == 'adc_dsa_db':
        xlabel = 'ADC DSA attenuation (dB)'
    drifts, drives, span, extra = [], [], [], []
    for comb in combs:
        y, low, high = noise(comb, ladder)
        if len(y) < 2:
            continue
        span += [y.min(), y.max()]
        x = curve(comb, ladder, xcol)[1]
        # The setting that compensated, now that it is off the x axis.
        if ladder in ('fftshift', 'adc_level', 'dac_level'):
            other = curve(comb, ladder, 'stages' if ladder == 'fftshift'
                          else knob_column('adc_level', 'rx_atten_db'))[1]
            extra += [other[0], other[-1]]
        ax.fill_between(x, low, high, color=colours[comb], alpha=0.18, lw=0)
        ax.plot(x, y, 'o-', ms=5, lw=1.8, color=colours[comb])
        # Anchored at this curve's own reference rung: 1 dB of noise per dB of
        # gain given up.  Defined from the gain, not from x -- two of these
        # ladders walk x downwards, and keying the slope to x flips the sign.
        ax.plot(x, y[0] + gain_given_up(comb, ladder), '--', lw=1.0,
                color=MUTED, zorder=1)
        drifts.append(np.ptp(curve(comb, ladder, 'signal_db')[1]))
        drives.append(curve(comb, ladder, 'dac_fs')[1])
        bad = [i for i, k in enumerate(sorted(k for k in groups
                                              if k[0] == comb and k[1] == ladder))
               if groups[k][0]['flags']]
        if bad:                                   # never colour alone: marker too
            clipped = True
            ax.plot(x[bad], y[bad], 'x', ms=9, mew=2, color=CRITICAL)
    if logx:
        ax.set_xscale('log', base=2)
        ax.xaxis.set_major_formatter(FuncFormatter(TICKS[xcol]))
    else:
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))
    if span:
        # The measured curves set the y scale; the dashed limits are allowed to
        # run off the panel, or a steep one squashes every real curve flat.
        margin = 0.08 * (max(span) - min(span)) + 1.0
        ax.set_ylim(min(span) - margin, max(span) + margin)
        drive = np.concatenate(drives)
        held = ladder in ('psb_scale', 'fftshift')
        what += ('.  DAC drive measured at {:.2f} FS, carrier held to {:.2f} dB'
                 if held else
                 '.  DAC drive measured from {:.2f} down to {:.2f} FS'
                 ).format(*((drive.mean(), max(drifts)) if held
                            else (drive.max(), drive.min())))
        if extra:
            what += ('.  PSB shift stages {:.0f} down to {:.0f} across it'
                     if ladder == 'fftshift' else
                     '.  attenuation {:.1f} to {:.1f} dB across it'
                     ).format(max(extra), min(extra))
    else:
        what += '.  No steps of this ladder in the file.'
    # Name the absent combs rather than letting a colour quietly vanish.
    missing = [int(c) for c in combs if len(noise(c, ladder)[0]) < 2]
    if missing and span:
        what += ('  Nothing for comb size ' + ', '.join(str(m) for m in missing)
                 + (': the RX optimiser had already chosen the coldest schedule, '
                    'so there was no stage left to add.'
                    if ladder == 'rx_gain' else '.'))
    caption(ax, what, ladder)
    ax.set_xlabel(xlabel, color=MUTED, fontsize=9)
    ax.set_ylabel(NOISE_LABEL, color=MUTED, fontsize=9)

# The single in-band number hides a strongly sloped spectrum: say so, with the
# three sub-band medians the CSV already carries.
ax = axes[2, 0]
centres = np.sqrt(EDGES[:-1] * EDGES[1:])
tilts = []
for comb in combs:
    y = [-curve(comb, 'psb_scale', f'{COMPONENT}_{b}')[1][0] for b, *_ in SUBBANDS]
    tilts.append(y[0] - y[2])
    ax.plot(centres, y, 'o-', ms=5, lw=1.8, color=colours[comb])
ax.set_xscale('log')
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
caption(ax, 'The same noise split into three sub-bands, at each comb\'s '
        f'reference setting.  It slopes by {min(tilts):.0f}-{max(tilts):.0f} dB '
        'across the band, and most in-band bins sit at the top end, so the '
        'single number on every other panel is close to the quiet end.',
        'the noise is not white')
ax.set_xlabel('sub-band centre frequency (Hz)', color=MUTED, fontsize=9)
ax.set_ylabel(NOISE_LABEL, color=MUTED, fontsize=9)

# Where the median stops describing the noise -- but see the caption: on a
# sloped spectrum the low-frequency rise itself trips this test.
ax = axes[2, 1]
for comb in combs:
    steps, spur, *_ = curve(comb, 'psb_scale', f'{COMPONENT}_spurfrac')
    if len(steps) > 1:
        ax.plot(curve(comb, 'psb_scale', 'amp')[1], spur, 'o-', ms=5, lw=1.8,
                color=colours[comb])
ax.set_xscale('log', base=2)
ax.xaxis.set_major_formatter(FuncFormatter(TICKS['amp']))
caption(ax, 'Fraction of in-band power in bins more than 6x above the median, '
        'along the psb_scale ladder.  Read with the panel to its left: a sloped '
        'spectrum puts its own low-frequency end above the median, so this '
        'counts spectral tilt as well as genuine lines.',
        'how much power sits above the median')
ax.set_xlabel('tone amplitude word', color=MUTED, fontsize=9)
ax.set_ylabel('fraction of in-band power', color=MUTED, fontsize=9)
ax.set_ylim(-0.02, 1.02)

# Per-tone spread: flat means one number describes the comb, structure means the
# noise depends on where a tone sits (sub-bin position, or products landing).
ax, comb = axes[2, 2], combs[-1]
keys = sorted(k for k in groups if k[0] == comb and k[1] == 'psb_scale')
for key, colour in ((keys[0], RAMP[1]), (keys[-1], ACCENT)):
    amp = groups[key][0]['amp']
    ax.plot([r['freq_hz'] / 1e6 for r in groups[key]],
            [-r[f'{COMPONENT}_{PER_TONE}'] for r in groups[key]], '.', ms=4,
            color=colour, alpha=0.7 if len(groups[key]) > 50 else 1.0,
            label=f'amplitude {amp:g}' if amp >= 1 else f'amplitude 1/{round(1/amp)}')
ax.legend(frameon=False, fontsize=9, labelcolor=MUTED, loc='center right')
caption(ax, 'Every tone individually, at the first and last rungs of the '
        'psb_scale ladder -- the spread the shaded bands above summarise.  '
        'Structure across the band means the noise depends on where a tone '
        'sits, not only on the gain split.',
        f'every tone of the {int(comb)}-tone comb')
ax.set_xlabel('tone frequency (MHz)', color=MUTED, fontsize=9)
ax.set_ylabel(NOISE_LABEL, color=MUTED, fontsize=9)

# Amplitude noise against phase noise: equal means the disturbance is additive
# in the IQ plane rather than a gain or phase fluctuation.
ax = axes[3, 0]
mag = np.array([-r[f'mag_{PER_TONE}'] for r in rows])
phase = np.array([-r[f'phase_{PER_TONE}'] for r in rows])
ax.plot(mag, phase, '.', ms=1.5, color=RAMP[1], alpha=0.15)
lims = [min(mag.min(), phase.min()) - 1, max(mag.max(), phase.max()) + 1]
ax.plot(lims, lims, '--', lw=1.0, color=MUTED)
ax.set_xlim(lims), ax.set_ylim(lims)
caption(ax, f'One dot per tone per step, all {len(rows)} of them, against the '
        f'dashed equality line.  Median difference '
        f'{np.median(phase - mag):+.2f} dB: the two quadratures carry the same '
        'noise, so it is additive in the IQ plane, not a gain or phase drift.',
        'amplitude noise vs phase noise')
ax.set_xlabel('magnitude noise (dBc/Hz)', color=MUTED, fontsize=9)
ax.set_ylabel('phase noise (dBc/Hz)', color=MUTED, fontsize=9)

# What a bigger comb costs per tone, at the same DAC drive.
ax = axes[3, 1]
reference = np.array([noise(c, 'psb_scale')[0][0] for c in combs])
ax.plot(combs, reference, 'o-', ms=6, lw=1.8, color=RAMP[2])
ax.plot(combs, reference[0] + 10 * np.log10(np.array(combs) / combs[0]), '--',
        lw=1.0, color=MUTED)
ax.set_xscale('log')
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
caption(ax, 'Noise per tone at each comb\'s own reference setting, against a '
        'dashed 10log10(N): what it would cost if a fixed total noise were '
        'simply shared out among more tones.',
        'what more tones cost, at the same DAC drive')
ax.set_xlabel('tones in comb', color=MUTED, fontsize=9)
ax.set_ylabel(NOISE_LABEL, color=MUTED, fontsize=9)

# The answer panel: drive moves the DAC output and the ADC input together, so
# its slope is the sum of the two ends.  adc_level holds the DAC and lowers the
# ADC; dac_level does the reverse.  Whichever is larger names the end that sets
# the floor, and adc_level + dac_level should come back to drive.
ax = axes[3, 2]
def share(comb, ladder):
    """Fraction of the noise POWER that this ladder's end contributes.

    Taken from the two-source fit rather than a slope: with 6 dB steps the first
    difference sits well up the curve and overstates the share badly (0.34 for a
    true 0.20 on simulated data).  For dac_level the fit is referred to its own
    first rung, which has the extra attenuation dialled in, so it is evaluated
    at the far end where the attenuation is back to the reference.
    """
    y = noise(comb, ladder)[0]
    x = gain_given_up(comb, ladder)
    other, this, _ = two_source_fit(x, y)
    if not np.isfinite(other) or not np.isfinite(this):
        return np.nan
    if ladder == 'dac_level':                      # refer it back to x = far end
        this += x[-1]
    return 10 ** (this / 10) / (10 ** (this / 10) + 10 ** (other / 10))


splits = (('dac_level', 'DAC end (before the attenuation)', RAMP[0], '^'),
          ('adc_level', 'ADC end (after it)', ACCENT, 's'))
found, totals = False, {}
for ladder, label, colour, marker in splits:
    xs, ys = [], []
    for comb in combs:
        if len(noise(comb, ladder)[0]) < 3:
            continue
        xs.append(comb)
        ys.append(share(comb, ladder))
        totals[comb] = totals.get(comb, 0.0) + ys[-1]
    if xs:
        found = True
        ax.plot(xs, ys, marker=marker, ls='-', ms=6, lw=1.8, color=colour,
                label=label)
if len(totals) and found:
    # The two shares must add to 1.  Where they do not, one ladder never
    # reached its own knee and its fit is the unreliable one -- check the
    # residuals in the printed summary before believing either.
    ax.plot(sorted(totals), [totals[c] for c in sorted(totals)], ls=':', lw=1.4,
            color=MUTED, marker='.', label='sum (should be 1)')
ax.axhline(0.5, ls='--', lw=1.0, color=MUTED)
ax.set_xscale('log')
ax.set_ylim(-0.03, 1.03)
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f'{v:g}'))
if found:
    ax.legend(frameon=False, fontsize=9, labelcolor=MUTED, loc='best')
caption(ax, 'Share of the noise power each end contributes, from the two-source '
        'fit to its own ladder.  The two should add to 1 (dashed: an even '
        'split).  The drive ladder cannot do this -- it moves both ends at once '
        'and reads ~1 whichever dominates.' + ('' if found else '  No attenuator '
        'ladders in this file: run with LOOPBACK = False, and once per '
        'LEVEL_KNOB to bracket the RF frontend.'),
        'is it the DAC end or the ADC end?')
ax.set_xlabel('tones in comb', color=MUTED, fontsize=9)
ax.set_ylabel('share of the noise power', color=MUTED, fontsize=9)

# One legend for every comb in the file, so the colour mapping is complete even
# where a panel has no data for one of them.
handles, labels = [], []
for comb in combs:
    reference = noise(comb, 'psb_scale')[0]
    handles.append(plt.Line2D([], [], marker='o', ms=5, lw=1.8,
                              color=colours[comb]))
    labels.append(f'{int(comb)} · {reference[0]:.0f} dBc/Hz' if len(reference)
                  else f'{int(comb)}')
handles += [plt.Line2D([], [], ls='--', lw=1.0, color=MUTED),
            plt.Rectangle((0, 0), 1, 1, color=RAMP[1], alpha=0.18)]
labels += ['limit if that source alone set the noise',
           '16-84th percentile across tones']
if clipped:
    handles.append(plt.Line2D([], [], ls='', marker='x', ms=9, mew=2,
                              color=CRITICAL))
    labels.append('clipped or overflowed')
fig.legend(handles, labels, loc='outside upper right', ncols=4, frameon=False,
           fontsize=9, labelcolor=MUTED,
           title='tones in comb · noise at its reference setting',
           title_fontsize=9)
fig.suptitle(
    f'Measured accumulator noise vs each TX digital gain setting\n'
    f'{COMPONENT}, {BAND_HZ[0]:g}-{BAND_HZ[1]:g} Hz · per tone: '
    f'{PER_TONE_SHORT[PER_TONE]} · across tones: {ACROSS_TONES}\n{path}',
    color=INK, fontsize=11, x=0.01, ha='left')
fig.savefig(OUTFILE, dpi=140, facecolor='#fcfcfb')
print(f'\nwrote {OUTFILE}')
