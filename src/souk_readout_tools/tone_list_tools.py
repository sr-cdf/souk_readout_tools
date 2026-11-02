"""
Tools for tone lists: lab ToneList files, and the phases of the DAC comb.

=====================================  ========================================
task                                   function
=====================================  ========================================
read a lab ToneList file               :func:`read_automeasure_toneslist`
write a lab ToneList file              :func:`write_automeasure_toneslist`
DAC peak of a comb with given phases   :func:`comb_crest_factor`
phases that minimise the DAC peak      :func:`optimise_tone_phases`
=====================================  ========================================

On the board, ``ReadoutClient.generate_optimised_phases()`` runs
:func:`optimise_tone_phases` on the comb currently set.

A comb whose tones all sit on whole multiples of the accumulation rate (the
readout sample rate, 600 kHz / acc_len) repeats exactly once per accumulation,
so its DAC peak is one fixed number set by the tone phases. The server puts
tones on that grid by default (``doc/tone_frequency_snapping.md``). The comb
tools synthesise that one period with an FFT at the DAC's complex sample rate,
from each tone's TX digital-baseband frequency: the frequency the DAC words
carry, before the DUC. That is RF minus 1228.8 MHz in Nyquist zone 1 with no
up/down converter, and in general
``info['tones']['detailed_frequency_info']['tx']['digital_baseband_freq']``.

Two peak factors, both over rms|z| = sqrt(sum a_k^2), both at the DAC sample
instants unless oversampled:

- ``iq_factor``: max(|I|, |Q|), what clips the 16-bit DAC words and what
  ``check_output_saturation`` reads.
- ``env_factor``: max|z|, the RF envelope, what an amplifier sees.
"""
import time

import numpy as np
import scipy.fft as sfft
from scipy.optimize import minimize

DAC_SAMPLE_RATE_HZ = 2457.6e6     # complex rate of the DAC I/Q words (adc_clk_hz)


def read_automeasure_toneslist(filename,printing=True):
    """
    Read a standard tones list file from the cardiff kids lab automeasure program.

    Parameters
    ----------
    filename : str
        Path to the tab-delimited ToneList file.
    printing : bool, optional
        Print a per-tone summary as the file is read (default ``True``).

    Returns
    -------
    tuple
        ``(name, freqs, offset_att, all_flag, none_flag)`` arrays.
    """
    try:
        result =np.genfromtxt(filename,names=True,delimiter='\t',dtype=('S32','f8','f8','i4','i4'),unpack=True)
        name,freqs,offset_att,all_flag,none_flag = result
        if printing:
            print('Read %d tones from %s'%(len(freqs),filename))
            print('Name\tFreq\tOffset att\tAll\tNone')
            for k in range(len(freqs)):
                print('%s\t%f\t%.2f\t%d\t%d'%(name[k].decode(),freqs[k],offset_att[k],all_flag[k],none_flag[k]))

        return name,freqs,offset_att,all_flag,none_flag

    except FileNotFoundError:
        print(f'File not found {filename}')
    except Exception as e:
        print('Unrecognised file format, is this a standard lab ToneList file?')
        raise e



def write_automeasure_toneslist(filename,freqs,names=None,offset_att=None,all_flag=None,none_flag_flag=None,amps=None):
    """
    Write a standard tones list file for the cardiff kids lab automeasure program.

    If <amps> is given, offset_att is calculated from the amplitudes, overwriting any value given in <offset_att>.

    Parameters
    ----------
    filename : str
        Output ToneList path.
    freqs : array-like
        Tone frequencies (Hz), one per tone.
    names : list of str or None, optional
        Per-tone names; defaults to ``K000``, ``K001``, ... when ``None``.
    offset_att : array-like or None, optional
        Per-tone offset attenuation (dB); defaults to zeros (overwritten when
        ``amps`` is given).
    all_flag : array-like or None, optional
        Per-tone "All" flag; defaults to ones.
    none_flag_flag : array-like or None, optional
        Per-tone "None" flag; defaults to zeros.
    amps : array-like or None, optional
        Per-tone amplitudes; if given, ``offset_att`` is derived from them
        (relative dB, min-subtracted).
    """
    if names is None:
        names=['K%03d'%k for k in range(len(freqs))]
    if offset_att is None:
        offset_att = [0]*len(freqs)
    if all_flag is None:
        all_flag = [1]*len(freqs)
    if none_flag_flag is None:
        none_flag_flag = [0]*len(freqs)
    if amps is not None:
        att = 20*np.log10(np.abs(amps))
        att-= att.min()
        #att = 2*floor(att/2.)
        #att[att>62]=62
        offset_att=att

    file=open(filename,'w')
    file.write('Name\tFreq\tOffset att\tAll\tNone\r\n')
    for k in range(len(freqs)):
        file.write('%s\t%f\t%f\t%d\t%d\r\n'%(names[k],freqs[k],offset_att[k],all_flag[k],none_flag_flag[k]))
    file.close()
    return


# ---------------------------------------------------------------------------
# DAC comb crest factor
# ---------------------------------------------------------------------------

def _as_comb(freqs_hz, amplitudes=None, sample_rate=None):
    """Plain arrays from plain arrays or from the project's tone records.

    Accepts an ``info`` dict (stream, sweep, ``get_info``), its
    ``info['tones']``, or the ``get_tone_frequencies(detailed_output=True)``
    dict, and fills in amplitudes and the sample rate (``acc_grid_hz``) from
    the record when not given. The only place that knows those shapes.
    """
    if isinstance(freqs_hz, dict):
        rec = freqs_hz.get('info') or freqs_hz
        rec = rec.get('tones') or rec
        if amplitudes is None:
            amplitudes = rec.get('amplitudes')
        if sample_rate is None:
            sample_rate = rec.get('acc_grid_hz')
        rec = rec.get('detailed_frequency_info') or rec
        freqs_hz = rec['tx']['digital_baseband_freq']
    f = np.asarray(freqs_hz, dtype=float).ravel()
    if f.size == 0:
        raise ValueError('no tones')
    a = np.ones(f.size) if amplitudes is None else np.asarray(amplitudes, dtype=float).ravel()
    if a.size != f.size:
        raise ValueError(f'{a.size} amplitudes for {f.size} tones')
    if not sample_rate:
        raise ValueError('sample_rate (the accumulation rate in Hz) is required')
    return f, a, float(sample_rate)


def _grid(f, sample_rate, dac_sample_rate):
    """Grid index of each tone, the period in DAC samples, and the largest
    distance of a tone from its grid point (Hz)."""
    n = dac_sample_rate / sample_rate
    if abs(n - round(n)) > 1e-6:
        raise ValueError(f'sample_rate {sample_rate} Hz does not divide the DAC rate '
                         f'{dac_sample_rate} Hz, so it is not an accumulation rate')
    x = f / sample_rate
    m = np.round(x).astype(np.int64)
    return m, int(round(n)), float(np.max(np.abs(x - m)) * sample_rate)


def _synth(c, m, n, dtype=np.complex64):
    """One period of the comb: z_j = sum_k c_k exp(2 pi i m_k j / n)."""
    z = np.zeros(n, dtype=dtype)
    np.add.at(z, np.mod(m, n), c.astype(dtype))
    return sfft.ifft(z, workers=-1, norm='forward')


def _factors(z, a):
    """(iq_factor, env_factor) of one period over rms|z| = sqrt(sum a^2)."""
    rms = np.sqrt(np.sum(a**2))
    return (float(max(np.abs(z.real).max(), np.abs(z.imag).max()) / rms),
            float(np.abs(z).max() / rms))


def _ipow(x, e):
    """x**e for an integer e >= 1 by repeated squaring (much faster than ** on
    large float32 arrays)."""
    out, base = None, x
    while e:
        if e & 1:
            out = base.copy() if out is None else out * base
        e >>= 1
        if e:
            base = base * base
    return out


def _lp_cost(ph, a, m, n, p, target):
    """log of the L_p norm of the target quantity over one period, and its
    gradient over the phases. Scaled by the peak s before the power so
    float32 cannot overflow; one FFT pair per call."""
    c = a * np.exp(1j * ph)
    z = _synth(c, m, n)
    if target == 'iq':
        s = float(max(np.abs(z.real).max(), np.abs(z.imag).max()))
        x, y = z.real * np.float32(1 / s), z.imag * np.float32(1 / s)
        u, v = _ipow(x, p - 1), _ipow(y, p - 1)
        F = float(np.dot(u, x) + np.dot(v, y))
        S = sfft.fft(u + 1j * v, workers=-1)[np.mod(m, n)]
        log_s = np.log(s)
    else:
        q = z.real**2 + z.imag**2
        s = float(q.max())
        q = q * np.float32(1 / s)
        w = _ipow(q, p // 2 - 1)
        F = float(np.dot(w, q))
        S = sfft.fft(w * z, workers=-1)[np.mod(m, n)]
        log_s = 0.5 * np.log(s)
    grad = np.real(1j * c * np.conj(S)) / (s * F)
    return log_s + np.log(F) / p, grad


def comb_crest_factor(freqs_hz, amplitudes=None, phases_rad=None, sample_rate=None,
                      oversample=1, dac_sample_rate=DAC_SAMPLE_RATE_HZ):
    """
    DAC peak factors of one period of a tone comb.

    Parameters
    ----------
    freqs_hz : array_like or dict
        TX digital-baseband tone frequencies (Hz), see the module docstring;
        or an ``info`` / ``info['tones']`` / detailed-frequency dict.
    amplitudes : array_like, optional
        Relative tone amplitudes as the DAC plays them: the TX scaling words,
        ``get_tone_amplitudes()``. Default flat.
    phases_rad : array_like, optional
        Tone phases (radians). Default zeros, the worst case.
    sample_rate : float
        Accumulation rate (Hz), 600 kHz / acc_len: the grid, and one over the
        period. Taken from a record's ``acc_grid_hz`` when not given.
    oversample : int, optional
        1 (default) evaluates at the DAC sample instants; 2 or more brings
        ``env_factor`` towards the continuous envelope.
    dac_sample_rate : float, optional
        Complex sample rate of the DAC words (Hz), default 2457.6 MHz.

    Returns
    -------
    dict
        ``iq_factor``, ``env_factor`` (see the module docstring),
        ``papr_db`` = 20 log10(env_factor), and ``max_off_grid_hz``, the
        largest distance of a tone from the grid. A tone off the grid is
        evaluated at its grid point, so the factors are exact for a snapped
        comb and only a starting value for an unsnapped one, which never
        repeats and whose peak grows with the time watched.

    Examples
    --------
    >>> rng = np.random.default_rng(0)
    >>> f = 37.5e3 * rng.choice(np.arange(-30000, 30000), 64, replace=False)
    >>> newman = np.pi * np.argsort(np.argsort(f))**2 / len(f)
    >>> r = comb_crest_factor(f, phases_rad=newman, sample_rate=37.5e3)
    >>> sorted(r)
    ['env_factor', 'iq_factor', 'max_off_grid_hz', 'papr_db']
    >>> r['iq_factor'] <= r['env_factor']
    True
    """
    f, a, sample_rate = _as_comb(freqs_hz, amplitudes, sample_rate)
    ph = np.zeros(f.size) if phases_rad is None else np.asarray(phases_rad, dtype=float).ravel()
    m, n, off = _grid(f, sample_rate, dac_sample_rate)
    z = _synth(a * np.exp(1j * ph), m, n * int(oversample), np.complex128)
    iq, env = _factors(z, a)
    return {'iq_factor': iq, 'env_factor': env, 'papr_db': 20 * np.log10(env),
            'max_off_grid_hz': off}


def optimise_tone_phases(freqs_hz, amplitudes=None, sample_rate=None, initial_phases=None,
                         target='iq', max_evaluations=42, dac_sample_rate=DAC_SAMPLE_RATE_HZ):
    """
    Tone phases that minimise the DAC peak of one period of a comb.

    Minimises the L_p norm of the peak quantity over the phases (L-BFGS),
    raising p from 16 to 64 so the norm tends to the maximum. Each evaluation
    is one FFT pair over one period (983040 samples at 2500 Hz), so the cost
    hardly depends on the number of tones: the default 42 evaluations take
    about 4 s at 2500 Hz on an 8-core PC for 380 to 2000 tones, longer at
    lower sample rates, whose period is longer (about 7 s at 1562.5 Hz). On the
    6a comb this gives an iq_factor of 2.50 against 3.44 for Newman phases.

    Parameters
    ----------
    freqs_hz, amplitudes, sample_rate, dac_sample_rate
        As :func:`comb_crest_factor`.
    initial_phases : array_like, optional
        Starting phases. Default Newman phases by frequency rank. After a
        small change (a few tones moved, a re-balance) the previous phases
        with ``max_evaluations=10`` recover most of the gain in about 1.3 s.
    target : {'iq', 'env'}, optional
        'iq' (default) minimises max(|I|, |Q|), what clips the DAC words.
        'env' minimises the envelope, which is unchanged when every tone is
        shifted by the same frequency.
    max_evaluations : int, optional
        Evaluation budget. The default 42 gets within about 0.05 of a long run.

    Returns
    -------
    dict
        ``phases_rad``, one per tone in input order, in [0, 2 pi);
        ``iq_factor`` and ``env_factor`` with them, ``start_iq_factor`` and
        ``start_env_factor`` with the starting phases; ``gain_db``, the drop of
        the target factor, 20 log10(start / result); ``max_off_grid_hz`` as in
        :func:`comb_crest_factor`; ``n_evaluations``; ``elapsed_s``. If the
        optimiser ends worse than it started, the starting phases come back.

    Notes
    -----
    The phases hold only while the comb is exactly periodic: tones on the
    grid, and every tone's realised phase equal to its written one. A tone
    off the grid by e Hz drifts through every relative phase in 1/e seconds;
    the server's snapping leaves e up to ~1e-4 Hz at acc_len 240 (the LO word
    step), which loses most of the gain within half an hour, and exactly zero
    at a power-of-two acc_len. Moving tones changes the comb: a retune that
    moves 3% of them loses most of the gain until the next optimisation. A
    common power step changes nothing.

    Examples
    --------
    >>> rng = np.random.default_rng(0)
    >>> f = 37.5e3 * rng.choice(np.arange(-30000, 30000), 64, replace=False)
    >>> r = optimise_tone_phases(f, sample_rate=37.5e3, max_evaluations=20)
    >>> r['iq_factor'] < r['start_iq_factor']
    True
    >>> len(r['phases_rad'])
    64
    """
    t0 = time.perf_counter()
    if target not in ('iq', 'env'):
        raise ValueError("target must be 'iq' or 'env'")
    f, a, sample_rate = _as_comb(freqs_hz, amplitudes, sample_rate)
    m, n, off = _grid(f, sample_rate, dac_sample_rate)
    a = a / np.sqrt(np.sum(a**2))
    if initial_phases is None:
        ph0 = np.pi * np.argsort(np.argsort(f))**2 / f.size
    else:
        ph0 = np.asarray(initial_phases, dtype=float).ravel()
        if ph0.size != f.size:
            raise ValueError(f'{ph0.size} initial phases for {f.size} tones')
    start = _factors(_synth(a * np.exp(1j * ph0), m, n, np.complex128), a)
    ph, n_eval = ph0, 0
    stages = (16, 64) if max_evaluations > 15 else (64,)
    for p in stages:
        budget = max(1, max_evaluations // len(stages) - 2)
        res = minimize(_lp_cost, ph, args=(a, m, n, p, target), jac=True, method='L-BFGS-B',
                       options={'maxiter': budget, 'maxfun': budget + 2})
        ph, n_eval = res.x, n_eval + res.nfev
    ph = np.mod(ph, 2 * np.pi)
    iq, env = _factors(_synth(a * np.exp(1j * ph), m, n, np.complex128), a)
    k = 0 if target == 'iq' else 1
    if (iq, env)[k] > start[k]:
        ph, (iq, env) = np.mod(ph0, 2 * np.pi), start
    return {'phases_rad': ph, 'iq_factor': iq, 'env_factor': env,
            'start_iq_factor': start[0], 'start_env_factor': start[1],
            'gain_db': float(20 * np.log10(start[k] / (iq, env)[k])),
            'max_off_grid_hz': off, 'n_evaluations': int(n_eval),
            'elapsed_s': time.perf_counter() - t0}

