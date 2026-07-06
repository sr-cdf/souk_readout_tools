"""
Filterbank channel-response tables: measure, store, evaluate.

The synthesis (PSB) and analysis (PFB) filterbanks share the same channel
shape. A tone sitting away from its armed bin centre sees the **combined**
TX+RX response: flat (<0.01 dB) over the centre half of a channel, ~-1 dB
around 0.77 of a bin spacing from centre, ~-6 dB at the channel edge (one full
bin spacing), where it also wraps. In RF loopback the analog path adds a
group-delay phase slope across the bin, curving near the edges where aliased
images interfere.

This module handles the *response table*: a measured complex gain versus bin
offset, produced in loopback by stepping probes across a channel with the
software modulation engine (which holds the channel maps fixed -- exactly the
geometry the table describes; a sweep re-snaps every point to its nearest bin
and cannot measure this). The measurement script is
``client/client_scripts/measure_filterbank_response.py``; this module reduces
its captures and loads/evaluates the resulting tables. The compensation
consumers (modulation preparers, tone setters) evaluate tables via
:func:`evaluate_response`.

Like ``modulation.py``, everything here is pure (arrays/dicts in, arrays out;
no client or socket dependency).

Table file format: plain text, ``# key: value`` header comments, then three or
five columns::

    offset_bins   gain_db   phase_rad   [gain_db_std   phase_rad_std]

``offset_bins`` is the offset from the armed bin centre in units of the bin
*spacing* (the ``drift_bins`` convention: +/-0.5 = half-bin edges, +/-1.0 =
channel edges). ``gain_db`` / ``phase_rad`` are the combined TX+RX response
relative to the bin centre. The optional ``*_std`` columns are the
tone-to-tone spread, a measurement-quality diagnostic.
"""

import datetime
import warnings

import numpy as np


def response_from_grouped(grouped, tone_modulation_state, offset_grid=None):
    """
    Reduce a grouped modulated loopback capture to a channel-response table.

    Parameters
    ----------
    grouped : dict
        ``modulation.group_cycles(data, state)`` output with ``reduce='mean'``
        (``z`` of shape ``(n_cycles, N, n_tones)``). Cycles are averaged.
    tone_modulation_state : dict
        The ``get_modulation_state()`` payload used for the grouping. Supplies
        each modulated tone's per-point ``drift_bins`` (true offset from its
        armed bin centre, in bin spacings) -- the response x-axis.
    offset_grid : numpy.ndarray or None, optional
        Common offset grid (bin spacings) to resample the per-tone responses
        onto. ``None`` uses ``N`` points across the offset range covered by
        every tone.

    Returns
    -------
    dict
        ``offset_bins`` (grid), ``gain_db``, ``phase_rad`` (tone-median
        combined response, normalised to bin centre), ``gain_db_std`` /
        ``phase_rad_std`` (tone-to-tone spread), ``n_tones_used``,
        ``n_cycles``, plus per-tone diagnostics: ``tone_indices``,
        ``tone_offset_bins`` / ``tone_gain_db`` / ``tone_phase_rad`` (lists of
        per-tone arrays on each tone's own measured offsets).
    """
    z = np.asarray(grouped['z'])
    if z.ndim != 3:
        raise ValueError("response_from_grouped needs group_cycles(..., reduce='mean') "
                         f"(3-D z); got {z.ndim}-D")
    n_cycles, n_points, n_tones = z.shape
    if n_cycles < 1:
        raise ValueError('no complete modulation cycles in the capture')
    with warnings.catch_warnings():
        warnings.simplefilter('ignore', category=RuntimeWarning)
        z_mean = np.nanmean(z, axis=0)          # (N, n_tones) complex

    # Per-(point, tone) true offsets from the armed bin centre, from the state.
    drift = {}
    for tone in tone_modulation_state.get('tones', []):
        d = np.asarray(tone.get('drift_bins', []), dtype=float)
        if len(d) == n_points:
            drift[int(tone['index'])] = d
    if not drift:
        raise ValueError("tone_modulation_state carries no per-tone 'drift_bins'; "
                         'is software modulation armed?')

    tone_indices = []
    tone_offset_bins = []
    tone_gain_db = []
    tone_phase_rad = []
    for idx, d in sorted(drift.items()):
        if idx >= n_tones:
            continue
        zc = z_mean[:, idx]
        good = np.isfinite(zc) & (np.abs(zc) > 0)
        if np.count_nonzero(good) < 3:
            warnings.warn(f'tone {idx}: fewer than 3 finite points, skipped', stacklevel=2)
            continue
        d_g, z_g = d[good], zc[good]
        order = np.argsort(d_g)
        d_s, z_s = d_g[order], z_g[order]
        if not (d_s[0] <= 0.0 <= d_s[-1]):
            warnings.warn(f'tone {idx}: offsets do not straddle the bin centre '
                          f'({d_s[0]:.3f}..{d_s[-1]:.3f} bins), skipped', stacklevel=2)
            continue
        # Normalise to the response at the bin centre (complex-interpolated).
        ref = np.interp(0.0, d_s, z_s.real) + 1j * np.interp(0.0, d_s, z_s.imag)
        rel = z_s / ref
        phase = np.unwrap(np.angle(rel))
        phase -= np.interp(0.0, d_s, phase)     # exactly zero phase at centre
        tone_indices.append(idx)
        tone_offset_bins.append(d_s)
        tone_gain_db.append(20.0 * np.log10(np.abs(rel)))
        tone_phase_rad.append(phase)

    if not tone_indices:
        raise ValueError('no usable tones in the capture')

    # Common grid: the offset range covered by every tone.
    lo = max(d[0] for d in tone_offset_bins)
    hi = min(d[-1] for d in tone_offset_bins)
    if offset_grid is None:
        offset_grid = np.linspace(lo, hi, n_points)
    else:
        offset_grid = np.asarray(offset_grid, dtype=float)
        if offset_grid[0] < lo or offset_grid[-1] > hi:
            warnings.warn('offset_grid extends beyond the range covered by every tone; '
                          'edge values are clamped by interpolation', stacklevel=2)

    g = np.stack([np.interp(offset_grid, d, gdb)
                  for d, gdb in zip(tone_offset_bins, tone_gain_db)], axis=0)
    p = np.stack([np.interp(offset_grid, d, ph)
                  for d, ph in zip(tone_offset_bins, tone_phase_rad)], axis=0)

    return {
        'offset_bins': offset_grid,
        'gain_db': np.median(g, axis=0),
        'phase_rad': np.median(p, axis=0),
        'gain_db_std': np.std(g, axis=0),
        'phase_rad_std': np.std(p, axis=0),
        'n_tones_used': len(tone_indices),
        'n_cycles': int(n_cycles),
        'tone_indices': tone_indices,
        'tone_offset_bins': tone_offset_bins,
        'tone_gain_db': tone_gain_db,
        'tone_phase_rad': tone_phase_rad,
    }


def save_response(path, table, meta=None):
    """
    Save a response table as plain text (see the module docstring for format).

    ``meta`` is a dict of extra ``# key: value`` header entries (e.g. label,
    bin spacing in Hz, loopback type). ``date``, ``n_tones_used`` and
    ``n_cycles`` are written automatically.
    """
    header = ['souk filterbank channel response (combined TX+RX, relative to bin centre)',
              'columns: offset_bins gain_db phase_rad gain_db_std phase_rad_std',
              f'date: {datetime.datetime.now().isoformat(timespec="seconds")}']
    for key in ('n_tones_used', 'n_cycles'):
        if key in table:
            header.append(f'{key}: {table[key]}')
    for key, value in (meta or {}).items():
        header.append(f'{key}: {value}')
    data = np.column_stack([
        table['offset_bins'], table['gain_db'], table['phase_rad'],
        table.get('gain_db_std', np.zeros_like(table['gain_db'])),
        table.get('phase_rad_std', np.zeros_like(table['phase_rad'])),
    ])
    np.savetxt(path, data, fmt='%+.6e', header='\n'.join(header))


def load_response(path):
    """
    Load a response table saved by :func:`save_response`.

    Returns a dict with ``offset_bins`` / ``gain_db`` / ``phase_rad`` (and the
    ``*_std`` columns when present), plus ``meta``: the parsed ``# key: value``
    header entries (values kept as strings).
    """
    meta = {}
    with open(path) as f:
        for line in f:
            if not line.startswith('#'):
                break
            text = line.lstrip('#').strip()
            if ':' in text:
                key, _, value = text.partition(':')
                if ' ' not in key.strip():
                    meta[key.strip()] = value.strip()
    data = np.atleast_2d(np.loadtxt(path))
    if data.shape[1] < 3:
        raise ValueError(f'{path}: expected at least 3 columns '
                         '(offset_bins gain_db phase_rad)')
    table = {
        'offset_bins': data[:, 0],
        'gain_db': data[:, 1],
        'phase_rad': data[:, 2],
        'meta': meta,
    }
    if data.shape[1] >= 5:
        table['gain_db_std'] = data[:, 3]
        table['phase_rad_std'] = data[:, 4]
    return table


def analytic_gain_db(offset_bins, combined=True, ntaps=8, nfft=64, pad=32, nw=2.0):
    """
    Analytic DPSS+sinc prototype-filter gain (dB) versus bin offset.

    The souk-firmware PFB/PSB prototype is an ``ntaps``-tap sinc weighted by a
    DPSS (NW=2) window, with the half-sample coefficient convention of
    mlib_devel's ``pfb_coeff_gen_calc.m`` -- confirmed against the firmware
    coefficients in souk-firmware issue #117. The single-bank power response
    is flat to ~0.005 dB over the centre half of a channel and crosses -6 dB
    at one bin spacing (the channel edge, where adjacent every-other-bin
    responses meet).

    ``combined=True`` (default) returns the TX+RX cascade: the banks are
    identical, so the combined gain is twice the single-bank dB. This is an
    idealised model -- it has no aliased-image interference, so it diverges
    from a measured table near the channel edges; use it as a cross-check
    and fallback, prefer a measured table for compensation.

    Parameters: ``offset_bins`` in bin spacings; ``ntaps`` prototype taps;
    ``nfft`` / ``pad`` set the internal resolution (defaults resolve the
    response to ~5e-4 bins). Requires scipy.
    """
    import scipy.signal

    offset_bins = np.asarray(offset_bins, dtype=float)
    # Coefficients: sinc on the matlab pfb_coeff_gen_calc time axis (samples
    # offset by half a step -- the convention the firmware actually uses; the
    # symmetric linspace variant gives a visibly different passband).
    trange = np.arange(0.5, ntaps * nfft, 1.0) / nfft - ntaps / 2.0
    coeffs = np.sinc(trange) * scipy.signal.windows.dpss(ntaps * nfft, nw, sym=True)
    response = np.abs(np.fft.fftshift(np.fft.fft(coeffs, nfft * pad)))
    response /= np.max(response)
    # The prototype's natural frequency unit is the *channel width* (one FFT
    # bin of the critically-sampled design). The 2x-oversampled filterbank
    # spaces channels every half channel width, and offset_bins (the
    # drift_bins convention) counts those spacings -- so halve the axis.
    x = np.linspace(-nfft / 2.0, nfft / 2.0, nfft * pad, endpoint=False)
    single_db = 20.0 * np.log10(np.interp(offset_bins / 2.0, x, response))
    return 2.0 * single_db if combined else single_db


def evaluate_response(table, offset_bins):
    """
    Evaluate a response table: complex combined gain at the given bin offsets.

    Linear interpolation on ``gain_db`` and ``phase_rad``; offsets outside the
    table's range are clamped to its end values. Returns a complex array
    shaped like ``offset_bins`` (unit gain, zero phase at the bin centre by
    construction of the table).
    """
    offset_bins = np.asarray(offset_bins, dtype=float)
    x = table['offset_bins']
    gain_db = np.interp(offset_bins, x, table['gain_db'])
    phase = np.interp(offset_bins, x, table['phase_rad'])
    return 10.0 ** (gain_db / 20.0) * np.exp(1j * phase)
