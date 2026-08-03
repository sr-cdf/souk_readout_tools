#!/usr/bin/env python
"""Where does the TX digital noise floor come from? (loopback, model-building)

The TX digital chain is

    tone amplitude  ->  PSB filterbank (fftshift schedule)  ->  psb_scale  ->  DAC

so the same DAC drive can be reached from many different splits of the digital
gain, and each split runs a different part of the chain closer to its own
rounding floor.  Four ladders, each moving one thing:

  1. psb_scale   amplitude down, psb_scale up: drive constant, the PSB runs
                 colder.  Noise referred to the PSB input degrades 6 dB per
                 halving; anything downstream does not move.
  2. fftshift    amplitude down, one fewer shift stage: drive constant, only the
                 stages upstream of the removed shift run colder.  Separates
                 per-stage rounding from one lumped source at the PSB input.
  3. drive       psb_scale alone, amplitude and schedule fixed, so the DAC drive
                 itself walks down.  Noise referred to DAC full scale (DAC word,
                 loopback truncation, ADC, PFB, accumulator) degrades 6 dB per
                 halving; anything proportional to the signal does not move.
                 This is what splits tests 1 and 2's amplitude-independent term.
  4. rx_gain     pfb_fftshift colder at fixed TX: isolates the accumulator- and
                 PFB-referred part of that term.  Skipped when the RX optimiser
                 already chose the coldest schedule (a single tone usually does).

Ladders 3 and 4 move the DAC output and the ADC input together, so a noise fixed
at the DAC and a noise fixed at the ADC both give 1 dB/dB and cannot be told
apart.  The RX attenuator sits between them, so with an RF cable in place
(LOOPBACK = False) two more ladders separate the two ends:

  5. adc_level   attenuation up, TX untouched: the ADC input falls while the DAC
                 output does not, so only noise entering AFTER the attenuation
                 (RF amp, ADC, RX digital) is fixed while the carrier falls.
  6. dac_level   psb_scale down and attenuation down by the same amount, holding
                 the ADC input constant, so only noise entering BEFORE it (DAC,
                 TX chain, cable, LNA) is fixed instead.

Their slopes split ladder 3's between them; whichever is the larger names the
end of the chain that sets the floor.  Both are skipped in internal loopback,
where the ADC path is a digital copy of the DAC and neither knob does anything.

LEVEL_KNOB picks which attenuation those two ladders step.  'atten' is the RF
frontend attenuator, ahead of the RF gain stages; 'dsa' is the ADC's own step
attenuator, after them and right at the converter.  Run the pair: what the two
runs agree on is the DAC and the ADC, and whatever they disagree about is
entering between the two knobs, in the RF frontend itself.

Every capture is taken with the ADC background calibration frozen, after
letting it settle unfrozen for ADC_CAL_SETTLE_S so it is current for that
configuration, and unfrozen again immediately afterwards.  A calibration that
runs during a capture steps the ADC mid-timestream and puts the step straight
into the band being measured; one left frozen for the whole run goes stale as
the configuration changes underneath it.

Ladders 1 and 2 hold the drive constant, so their signal must NOT move: that is
the check that the compensation happened.  Ladders 3-6 move it deliberately --
the level IS the axis there.  Every step also reports the measured DAC and ADC
peak levels and the PSB / psb_scale / PFB overflow counters; anything flagged
means the step clipped and is measuring saturation, not quantisation.

Reported and saved as noise-to-carrier (no calibration needed) in the IEEE
single-sideband convention L(f) = S(f) / 2, for three components -- complex
(magnitude and phase in quadrature, what sets readout SNR), magnitude alone,
phase alone:

  floor  median in-band density, dBc/Hz -- broadband, spurs excluded
  rms    mean in-band density, dBc/Hz -- the same with spurs included
  peak   worst in-band bin above the floor, dB
  band   total integrated over the band, dBc -- what lands in a science band
  saved as well: lines (in-band bins above 6x the median), spurfrac (fraction of
  band power in them), and lo/mid/hi (floor in three log-spaced sub-bands, to
  test whether the floor is white as a rounding model predicts)

Quantisation error is deterministic, so it appears as a spur forest on a
broadband floor and the spurs grow much faster than the floor -- a two-source
fit works on floor OR rms only where the spur fraction is stable along a ladder,
which is why both, and the spur split, are recorded.

ALIGN_TO_FS snaps every tone to a multiple of the accumulation rate.  Then every
intermodulation and harmonic product, at any order, sits at a multiple of the
accumulation rate too and aliases to DC rather than into the band, leaving the
broadband rounding floor on its own.  It survives only as far as the LO word
quantisation allows, so the realised residual is measured and printed; running
with it off is the way to see what the products were contributing.

With NUM_TONES > 1 each printed statistic is the median across tones, with the
best and worst tone floor alongside; the CSV keeps every tone separately.
NUM_TONES may be a sequence, e.g. (1, 10, 100): everything repeats per comb,
re-deriving that comb's reference gains.  Note that the RX optimiser picks a
different pfb_fftshift for each comb, so cross-comb comparisons mix a TX change
with an RX one -- set PFB_FFTSHIFT to pin one schedule for all of them, at the
cost of starving the dense combs.

Leaves the test tones loaded and the loopback as set; gains are restored.
"""
import csv
import time

import numpy as np
from scipy.signal import welch

from souk_readout_tools.client.readout_client import ReadoutClient

ADDRESS = '10.11.11.11'
PORT = 10000
LOOPBACK = False            # internal digital loopback
NUM_TONES = (1, 10, 100, 1000)   # a comb of N, or a sequence of comb sizes
FREQ_HZ = 1000.02e6        # single-tone frequency (already a multiple of 600 Hz)
COMB_HZ = (500e6, 2000e6)  # comb start/stop, used when NUM_TONES > 1
SCATTER_HZ = 0.25e6        # random +/- per tone; keep < half the comb spacing
SEED = 0                   # so the comb scatter repeats run to run
ALIGN_TO_FS = True         # snap tones to multiples of the accumulation rate
DURATION_S = 1.0
MAX_STEPS = 10             # cap on the length of ladders 1-3
RX_STEPS = 8               # extra pfb shift stages for ladder 4
PFB_FFTSHIFT = None        # None: optimise RX per comb; int: pin it for all
BAND_HZ = (5.0, 250.0)     # above 1/f, below Nyquist
NPERSEG = 1024
SAT_ITERATIONS = 250       # snapshots per DAC/ADC saturation check
OVERFLOW_S = 0.5           # DSP overflow watch window per step
ADC_CAL_SETTLE_S = 2.0     # let the ADC background calibration settle, unfrozen,
                           # before freezing it for a capture
LEVEL_KNOB = 'dsa'       # which RX level knob ladders 5-6 step:
                           #   'atten' RF frontend attenuator, 0-31.5 dB, before
                           #           the RF gain stages
                           #   'dsa'   ADC digital step attenuator, 0-27 dB, at
                           #           the converter, after them
                           # run both: whatever noise the pair disagree about is
                           # entering between them, in the RF frontend itself
ATTEN_STEP_DB = 6.0        # step for the two RF-only ladders
ATTEN_MAX_DB = 31.5        # programmable RX attenuator: 0-31.5 dB in 0.5 dB steps
DSA_MAX_DB = 27.0          # ADC digital step attenuator: 0-27 dB in 1 dB steps
PSB_SCALE_MAX = 255.996    # psb_scale is a UFix16_8 word: 1/256 to ~256
PSB_SCALE_MIN = 4 / 256    # keep the drive ladder a few LSBs above zero
PSB_STAGES = 12            # effective: asking for 13 (8191) gave 6 dB more gain
                           # than 12 (4095) and clipped, so bit 12 does nothing
PFB_STAGES = 11            # pfb_fftshift 2047 is the coldest schedule
COMPONENTS = ('complex', 'mag', 'phase')
STATS = ('floor', 'rms', 'peak', 'band')            # printed and saved
EXTRA = ('lines', 'spurfrac', 'lo', 'mid', 'hi')    # saved only
DAC_KEYS = ('i0max_fs', 'i0min_fs', 'q0max_fs', 'q0min_fs',
            'i1max_fs', 'i1min_fs', 'q1max_fs', 'q1min_fs')

# One CSV row per tone per step: summary statistics only, no spectra, so the
# whole run stays a small flat table.  Written and flushed step by step, so a run
# that dies half way still leaves everything it measured.
OUTFILE = time.strftime('quantisation_noise_%Y%m%d-%H%M%S.csv')
FIELDS = ('num_tones', 'test', 'step', 'amp', 'psb_scale', 'stages', 'pfb_stages',
          'rx_atten_db', 'adc_dsa_db', 'level_knob', 'dac_fs', 'adc_fs',
          'psb_ovf', 'psbscale_ovf', 'pfb_ovf', 'flags', 'tone', 'freq_hz',
          'carrier', 'signal_db'
          ) + tuple(f'{component}_{stat}' for component in COMPONENTS
                    for stat in STATS + EXTRA)


def rx_atten():
    """Current RX programmable attenuation in dB, or nan if there is no RF."""
    try:
        return float(client.get_rx_attenuation()['result']['rx_attenuation_db'])
    except (KeyError, TypeError, ValueError):
        return float('nan')


def adc_dsa():
    """Current ADC digital step attenuator setting in dB, or nan if unavailable."""
    try:
        return float(client.get_adc_dsa()['result']['adc_dsa_db'])
    except (KeyError, TypeError, ValueError):
        return float('nan')


def level_knob():
    """Read whichever RX level knob this run steps, in dB."""
    return adc_dsa() if LEVEL_KNOB == 'dsa' else rx_atten()


def set_level_knob(value_db):
    """Set that knob.  Both are recorded every step either way."""
    if LEVEL_KNOB == 'dsa':
        return client.set_adc_dsa(value_db)
    return client.set_rx_attenuation(value_db)


client = ReadoutClient(address=ADDRESS, request_port=PORT)
client.pull_config()
client.disable_modulation()                        # both would confound the PSD
client.disable_tracking()
client.set_parameter('internal_loopback', LOOPBACK)
cal_freeze_start = client.get_cal_freeze()         # restored at the end

csv_file = open(OUTFILE, 'w', newline='')
writer = csv.DictWriter(csv_file, fieldnames=FIELDS)
writer.writeheader()

fs = client.get_sample_rate()
num_samples = int(DURATION_S * fs)
n_avg = max(1, 2 * num_samples // NPERSEG - 1)      # Welch segments, 50% overlap
edges = np.geomspace(*BAND_HZ, 4)                   # three log-spaced sub-bands
print(f'{fs:.6f} Hz accumulator rate, {num_samples} samples per step, band '
      f'{BAND_HZ[0]}-{BAND_HZ[1]} Hz, {n_avg} Welch averages, loopback={LOOPBACK}\n'
      f'sub-bands {edges[0]:.1f}/{edges[1]:.1f}/{edges[2]:.1f}/{edges[3]:.1f} Hz, '
      f'tones {"aligned to" if ALIGN_TO_FS else "NOT aligned to"} the '
      f'accumulation rate\nfloor/rms in dBc/Hz and band in dBc, single '
      f'sideband (bigger is better), peak in dB\nwriting {OUTFILE}')


def analyse(z):
    """Noise-to-carrier statistics for one tone's accumulator timestream."""
    z0 = z.mean()
    a = np.abs(z) / np.abs(z0) - 1.0                 # fractional magnitude error
    phi = np.angle(z * np.conj(z0))                  # rad, small: no unwrap
    f, s_mag = welch(a, fs=fs, nperseg=NPERSEG)
    _, s_phase = welch(phi, fs=fs, nperseg=NPERSEG)
    band = (f >= BAND_HZ[0]) & (f <= BAND_HZ[1])

    # dz/z0 = a + i.phi to first order, so the complex noise-to-carrier density
    # is exactly the quadrature sum of the two, in 1/Hz either way.  Halving
    # each one-sided density gives the IEEE single-sideband L(f) = S(f) / 2,
    # the convention a synthesiser datasheet quotes and the one
    # plot_timestream_psd(psd_units='dbc') uses.
    out = {'carrier': np.abs(z0)}
    for name, s in (('complex', (s_mag + s_phase) / 2), ('mag', s_mag / 2),
                    ('phase', s_phase / 2)):
        # The median floor reads high by the chi-squared median/mean bias (1.6 dB
        # at one Welch average, ~0.2 dB at ten), so compare floors step to step
        # rather than absolutely; rms is unbiased (checked against synthetic
        # white noise of known density).
        sb = s[band]
        med = np.median(sb)
        lines = sb > 6.0 * med                       # discrete spurs
        out[name] = {
            'floor': -10 * np.log10(med),
            'rms': -10 * np.log10(sb.mean()),
            'peak': 10 * np.log10(sb.max() / med),
            'band': -10 * np.log10(np.trapezoid(sb, f[band])),
            'lines': int(lines.sum()),
            'spurfrac': float(sb[lines].sum() / sb.sum()),
            **{key: -10 * np.log10(np.median(s[(f >= lo) & (f < hi)]))
               for key, lo, hi in zip(('lo', 'mid', 'hi'), edges[:-1], edges[1:])}}
    return out


def health():
    """DAC/ADC peak levels and DSP overflow counters for the current config.

    A step with anything in ``flags`` is not comparable: the level is not where
    the gain model put it, so it is measuring clipping, not quantisation.
    """
    dac = client.check_output_saturation(iterations=SAT_ITERATIONS)
    adc = client.check_input_saturation(iterations=SAT_ITERATIONS)
    ovf = client.check_dsp_overflow(duration_s=OVERFLOW_S)['details']
    # Peak of many snapshots, so for a comb this wanders a few tenths of a dB
    # with the crest factor sampling: use it to catch clipping, and the per-tone
    # carrier (not this) as the matched-drive check.
    dac_fs = max(abs(float(dac['details'][k])) for k in DAC_KEYS)
    # In internal loopback the ADC snapshot is a digital copy of the DAC output,
    # so adc_fs is informative but adc['result'] is forced False by design --
    # the RX analog controls cannot change it, so it is never flagged.
    adc_fs = max(abs(float(adc['details'][k])) for k in
                 ('imax_fs', 'imin_fs', 'qmax_fs', 'qmin_fs'))
    bad = [name for name, flag in (('DAC_SAT', dac['result']),
                                   ('ADC_SAT', adc['result']),
                                   ('psb_ovf', ovf['psb_ovf_delta']),
                                   ('psbscale_ovf', ovf['psbscale_ovf_delta']),
                                   ('pfb_ovf', ovf['pfb_ovf_delta'])) if flag]
    return {'dac_fs': round(dac_fs, 5), 'adc_fs': round(adc_fs, 5),
            'psb_ovf': ovf['psb_ovf_delta'],
            'psbscale_ovf': ovf['psbscale_ovf_delta'],
            'pfb_ovf': ovf['pfb_ovf_delta'],
            'flags': '+'.join(bad)}                # '+': keeps it one CSV field


def measure(test, step, amp, scale, stages, freqs):
    """Apply one configuration, report its statistics, save a row per tone."""
    num_tones = len(freqs)
    client.set_tone_amplitudes(np.full(num_tones, amp))
    client.set_psb_scale(scale)
    client.set_psb_fftshift(2 ** stages - 1)
    applied = client.get_psb_scale()               # read back: UFix16_8, 1/256
    pfb_stages = bin(client.get_pfb_fftshift()).count('1')   # ladder 4 moves it
    atten, dsa = rx_atten(), adc_dsa()             # one of these is ladders 5-6's

    h = health()                                   # before the long capture
    # Freeze the ADC background calibration across the capture: while it runs it
    # steps the ADC mid-timestream, which lands in the very band being measured.
    # refresh_adc_cal unfreezes, settles for ADC_CAL_SETTLE_S so the calibration
    # is current for THIS configuration, then freezes.  Unfrozen again straight
    # after, so it tracks drift between steps rather than going stale.
    client.refresh_adc_cal(adc_cal_settle_time=ADC_CAL_SETTLE_S)
    data = client.parse_samples(client.get_samples(num_samples))
    client.set_cal_freeze(False)
    tones = [analyse(np.asarray(data['i_data'][k], dtype=float)
                     + 1j * np.asarray(data['q_data'][k], dtype=float))
             for k in sorted(data['i_data'])]
    signal = 20 * np.log10([t['carrier'] for t in tones])

    print(f'  amp {amp:7.4f}  scale {applied:7.3f}  stages {stages:2d}  '
          f'pfb {pfb_stages:2d}  atten {atten:4.1f} dsa {dsa:4.1f} dB  '
          f'signal {np.median(signal):7.2f} dB'
          + (f' (tone spread {np.ptp(signal):.2f} dB)' if num_tones > 1 else ''))
    print(f'    dac {h["dac_fs"]:.4f} FS   adc {h["adc_fs"]:.4f} FS   overflow '
          f'psb {h["psb_ovf"]} scale {h["psbscale_ovf"]} pfb {h["pfb_ovf"]}'
          + (f'   *** {h["flags"]} ***' if h['flags'] else ''))
    for name in COMPONENTS:
        s = np.array([[t[name][stat] for stat in STATS] for t in tones])
        stats = '  '.join(f'{k} {v:6.1f}' for k, v in
                          zip(STATS, np.median(s, axis=0)))
        spur = np.median([t[name]['spurfrac'] for t in tones])
        print(f'    {name:8s} {stats}  spurfrac {spur:5.2f}'
              + (f'   floor best {s[:, 0].max():6.1f} worst {s[:, 0].min():6.1f}'
                 if num_tones > 1 else ''))

    for index, (tone, freq, sig) in enumerate(zip(tones, freqs, signal)):
        row = {'num_tones': num_tones, 'test': test, 'step': step, 'amp': amp,
               'psb_scale': applied, 'stages': stages, 'pfb_stages': pfb_stages,
               'rx_atten_db': atten, 'adc_dsa_db': dsa,
               'level_knob': LEVEL_KNOB, 'tone': index, 'freq_hz': freq,
               'carrier': round(tone['carrier'], 1), 'signal_db': round(sig, 3),
               **h}
        for name in COMPONENTS:
            row.update({f'{name}_{stat}': round(tone[name][stat], 4)
                        for stat in STATS + EXTRA})
        writer.writerow(row)
    csv_file.flush()                               # survive a mid-run failure


def run(num_tones):
    """Load one comb, take the optimisers' gains as its reference, run the tests."""
    print(f'\n{"=" * 78}\n{num_tones} tone(s)\n{"=" * 78}')

    # Linear spacing plus a random scatter, so no tone sits on a PSB bin centre
    # and the tones are not commensurate with each other.  Snapping to the
    # accumulation rate then parks every intermodulation product at DC: the
    # snap is at most fs/2 = a few hundred Hz, a thousandth of a PSB bin, so it
    # does not move any tone within its channel.
    rng = np.random.default_rng(SEED)              # same comb for a given count
    freqs = np.array([FREQ_HZ]) if num_tones == 1 else (
        np.linspace(*COMB_HZ, num_tones)
        + rng.uniform(-SCATTER_HZ, SCATTER_HZ, num_tones))
    if ALIGN_TO_FS:
        freqs = np.round(freqs / fs) * fs
    client.set_tone_frequencies(freqs)
    client.set_tone_phases(client.generate_newman_phases(freqs))  # crest factor
    client.set_tone_amplitudes(np.ones(num_tones))  # the reference amplitude

    # What the hardware actually realised: the LO word is quantised, so the
    # alignment only holds to within that residual.  Two tones whose residuals
    # differ by d beat at d, so the spread is the worst product offset from DC;
    # if it reaches BAND_HZ[0] the products are back in band and the aligned
    # and unaligned runs will not differ.
    actual = np.sort(np.asarray(client.get_tone_frequencies(), dtype=float))
    resid = actual - np.sort(freqs)
    print(f'realised frequencies: max |offset from request| '
          f'{np.abs(resid).max():.3f} Hz, worst product {np.ptp(resid):.3f} Hz '
          f'from DC (band starts at {BAND_HZ[0]:g} Hz)')

    # Reference gains: whatever the house optimisers pick at amplitude 1.0,
    # digital only, with this comb loaded -- a denser comb needs a colder
    # psb_scale, so every comb gets its own reference.  The amplitude is pinned
    # so the psb_scale they find belongs to amplitude 1.0, and RX is optimised
    # once, here, then held for every ladder except 4.
    print('maximising TX then RX power...')
    client.maximise_tx_power(digital_only=True, rx_policy='none',
                             force_tone_amplitude=1.0)
    client.maximise_rx_power(digital_only=True, force_pfb_fftshift=PFB_FFTSHIFT)
    shift_stages = bin(client.get_psb_fftshift()).count('1')
    psb_scale = client.get_psb_scale()
    pfb_ref = client.get_pfb_fftshift()
    atten_start = level_knob()                     # nan without an RF frontend

    # Ladder 1 runs as far as psb_scale can double into its ceiling; ladder 3 as
    # far as it can halve towards zero.  Ladder 2 needs a shift stage to give up
    # per step and maximise_tx_power leaves none -- removing them is how it finds
    # gain -- so it starts from a COLDER schedule, with the spare psb_scale
    # doublings spent up front to hold the same DAC drive.
    steps_scale = min(MAX_STEPS, int(np.log2(PSB_SCALE_MAX / psb_scale)) + 1)
    spare = min(steps_scale - 1, PSB_STAGES - shift_stages)
    shift_start = shift_stages + spare
    scale_start = psb_scale * 2 ** spare
    steps_shift = min(MAX_STEPS, shift_start + 1, 13)  # amplitude word is 2**-12
    steps_drive = min(MAX_STEPS, int(np.log2(psb_scale / PSB_SCALE_MIN)) + 1)
    rx_stages = [bin(pfb_ref).count('1') + k for k in range(1, RX_STEPS + 1)]
    rx_stages = [n for n in rx_stages if n <= PFB_STAGES]
    n_points = steps_scale + steps_shift + steps_drive + len(rx_stages) + 1
    print(f'reference: psb_scale {psb_scale:.3f}, {shift_stages} psb shift '
          f'stages, pfb_fftshift {pfb_ref} ({bin(pfb_ref).count("1")} stages)\n'
          f'ladders: {steps_scale} psb_scale (ceiling {PSB_SCALE_MAX:g}), '
          f'{steps_shift} fftshift (cold start {shift_start} stages at psb_scale '
          f'{scale_start:.3f}), {steps_drive} drive, {len(rx_stages)} rx\n'
          f'{n_points} steps, roughly {n_points * (DURATION_S + 4) / 60:.1f} minutes')

    try:
        print('\ntest 1: amplitude down, psb_scale up (gain after the filterbank)')
        for k in range(steps_scale):
            measure('psb_scale', k, 2.0 ** -k, psb_scale * 2 ** k,
                    shift_stages, freqs)

        print('\ntest 2: amplitude down, fftshift stage removed (gain inside it)')
        for k in range(steps_shift):
            measure('fftshift', k, 2.0 ** -k, scale_start,
                    shift_start - k, freqs)

        # The last rungs land on psb_scale values the UFix16_8 word cannot hold
        # exactly, so plot the floor against the measured signal, not against the
        # nominal halvings -- both the applied scale and the signal are recorded.
        print('\ntest 3: psb_scale alone, DAC drive falls (level-referred noise)')
        for k in range(steps_drive):
            measure('drive', k, 1.0, psb_scale / 2 ** k, shift_stages, freqs)

        print('\ntest 4: pfb_fftshift colder, RX gain falls (accumulator-referred)')
        for k, n in enumerate(rx_stages, start=1):
            client.set_pfb_fftshift(2 ** n - 1)
            measure('rx_gain', k, 1.0, psb_scale, shift_stages, freqs)
        client.set_pfb_fftshift(pfb_ref)
        if not rx_stages:
            print('  skipped: the RX optimiser already chose the coldest schedule')

        # Tests 3 and 4 move the DAC output and the ADC input together, so a
        # noise fixed at the DAC and one fixed at the ADC both give 1 dB/dB and
        # cannot be told apart.  The attenuator sits between them and separates
        # the two: 5 holds the DAC and lowers the ADC, 6 lowers the DAC and
        # holds the ADC.  Their slopes should add up to test 3's.
        if LOOPBACK:
            print('\ntests 5-6 skipped: in internal loopback the ADC path is a '
                  'digital copy of the DAC, so the attenuator does nothing')
        elif not np.isfinite(atten_start):
            print('\ntests 5-6 skipped: no RF frontend attenuator to read')
        else:
            knob_max = DSA_MAX_DB if LEVEL_KNOB == 'dsa' else ATTEN_MAX_DB
            room = int((knob_max - atten_start) // ATTEN_STEP_DB)
            steps_adc = min(room, MAX_STEPS)
            steps_dac = min(room + 1, steps_drive, MAX_STEPS)
            head = ATTEN_STEP_DB * (steps_dac - 1)   # attenuation to give back

            print(f'\ntest 5: {LEVEL_KNOB} up from {atten_start:.1f} dB, TX '
                  'untouched (ADC input falls, DAC output does not)')
            # From k = 0, the reference itself: without its own first rung the
            # ladder's opening slope is measured partway up the curve.
            for k in range(steps_adc + 1):
                set_level_knob(atten_start + k * ATTEN_STEP_DB)
                measure('adc_level', k, 1.0, psb_scale, shift_stages, freqs)
            if not steps_adc:
                print(f'  skipped: only {knob_max - atten_start:.1f} dB of '
                      f'{LEVEL_KNOB} range left, less than one step')

            # A psb_scale halving is 6.0206 dB and the attenuator resolves
            # 0.5 dB, so the ADC level creeps ~0.02 dB per step -- a tenth of a
            # dB over the ladder, well under the repeatability, and the measured
            # adc_fs and carrier are recorded per step anyway.
            print(f'\ntest 6: psb_scale down and {LEVEL_KNOB} down together '
                  f'from {atten_start + head:.1f} dB, holding the ADC input '
                  'constant')
            for k in range(steps_dac):
                set_level_knob(atten_start + head - k * ATTEN_STEP_DB)
                measure('dac_level', k, 1.0, psb_scale / 2 ** k, shift_stages,
                        freqs)
            set_level_knob(atten_start)

        print('\nreference repeat (repeatability: smaller steps mean nothing)')
        measure('reference_repeat', 0, 1.0, psb_scale, shift_stages, freqs)
    finally:
        client.set_tone_amplitudes(np.ones(num_tones))
        client.set_psb_scale(psb_scale)
        client.set_psb_fftshift(2 ** shift_stages - 1)
        client.set_pfb_fftshift(pfb_ref)
        if not LOOPBACK and np.isfinite(atten_start):
            set_level_knob(atten_start)
        print("restored this comb's reference gains")


try:
    for n_tones in np.atleast_1d(NUM_TONES):       # an int or a sequence of ints
        run(int(n_tones))
finally:
    client.set_cal_freeze(cal_freeze_start)
    csv_file.close()
    print(f'\nwrote {OUTFILE}; the last comb is still loaded')
