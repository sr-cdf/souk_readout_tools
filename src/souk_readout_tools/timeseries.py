"""
Quantitative timestream and PSD analysis.

A flat set of tools: pick the one that answers your question and call it with
plain NumPy arrays, or hand it one of this package's own structures (a
``parse_samples`` dict, a ``parse_sweep_data`` dict) and it will resolve the
arrays itself. Nothing here requires a prior call to anything else here.

============================================  ==============================
I want to...                                  Use
============================================  ==============================
fill missing timestream samples              ``fill_gaps``
compute a PSD of one signal                   ``compute_psd``
compute PSDs of a whole timestream capture    ``timestream_psds``
compute PSDs of pre-accumulation snapshots    ``snapshot_psds``
average PSDs over repeated captures           ``compute_psd_averaged``
treat repeats as one long record              ``compute_psd_concatenated``
compute a PSD ignoring flagged samples        ``masked_psd``
thin a PSD for a log-frequency axis           ``log_bin_psd``
get the carrier power behind a dBc figure     ``carrier_power``
quote the noise level in a frequency band     ``psd_band_levels``
separate the 1/f and white parts of a PSD     ``fit_psd_model``
subtract an off-resonance readout floor       ``subtract_psds``
join slow and fast PSDs into one wide trace   ``stitch_psds``
model what the accumulator does to a spectrum ``accumulator_response``
fit tau_qp to a GR roll-off                   ``fit_gr_rolloff``
find cosmic-ray / glitch events in a stream   ``find_glitches``
fit tau_qp to individual glitch pulses        ``fit_glitch_pulses``
measure noise common to freq and dissipation  ``cross_psd``
combine both quadratures into one estimate    ``combine_quadratures``
measure the responsivity ratio beta           ``estimate_beta``
predict beta with no measurement at all       ``beta_from_kernels``
get absolute responsivity from a load ladder  ``signal_direction_from_loads``
see how long you can usefully integrate       ``allan_deviation``
summarise a timestream, robustly              ``timestream_stats``
save a result, reloadable without pickle      ``export_psds`` / ``load_psds``
tabulate several results as one flat CSV      ``export_summary``
============================================  ==============================

``timestream_psds`` is the compute half of
``plotting.plot_timestream_psd``: same arguments, same numbers, but it returns
the arrays instead of drawing them.

Dependency direction: ``plotting`` -> ``timeseries`` -> ``noise`` ->
``resonator``. Nothing in this module imports from ``plotting``.
"""

import warnings
from collections.abc import Mapping

import numpy as np
from scipy import ndimage as scipy_ndimage
from scipy import optimize as scipy_optimize
from scipy import signal as scipy_signal
from scipy import special as scipy_special
from scipy import stats as scipy_stats

from .noise import (_get_sweep_trace,
                    _with_samples_readout_correction,
                    _with_sweep_readout_correction,
                    fractional_frequency_and_dissipation_timestreams,
                    remove_blind_tone_common_modes,
                    remove_common_modes_svd,
                    remove_iq_common_modes_svd)

__all__ = [
    'fill_gaps',
    'compute_psd',
    'compute_psd_averaged',
    'compute_psd_concatenated',
    'masked_psd',
    'log_bin_psd',
    'carrier_power',
    'timestream_psds',
    'snapshot_psds',
    'psd_band_levels',
    'fit_psd_model',
    'subtract_psds',
    'accumulator_response',
    'stitch_psds',
    'fit_gr_rolloff',
    'find_glitches',
    'fit_glitch_pulses',
    'cross_psd',
    'allan_deviation',
    'timestream_stats',
    'export_psds',
    'load_psds',
    'export_summary',
    'combine_quadratures',
    'estimate_beta',
    'beta_from_kernels',
    'signal_direction_from_loads',
]

# Panel names for each timestream format, in (top, bottom) order.
PSD_PANELS = {
    'iq': ('i', 'q'),
    'magphase': ('magnitude', 'phase'),
    'freq_diss': ('frequency', 'dissipation'),
}


def fill_gaps(samples, method='nan', *, max_missing=1_000_000,
              noise_window=128, rng=None):
    """Return an independent parsed-samples copy with counter gaps inserted.

    Parameters
    ----------
    samples : dict
        Output of ``ReadoutClient.parse_samples``. I/Q arrays and all standard
        per-sample metadata must have length ``num_samples``.
    method : {'nan', 'interpolate', 'noise'}
        Fill inserted I/Q samples with NaN (default), or interpolate linearly
        between the immediately surrounding received samples. Existing NaNs
        are preserved, not repaired. ``'noise'`` draws Gaussian I/Q pairs
        with the local mean and covariance of received samples around each
        gap, independently for each tone. This matches local moments in
        expectation, not coloured spectral density or cross-tone correlation.
        Synthetic fills change noise statistics; they do not recover missing
        measurements. For modulated captures, use ``modulation.group_cycles``
        for interpolation within each point; noise filling is not supported.
    max_missing : int
        Maximum number of samples to insert, guarding against corrupt counters
        or unexpectedly large gaps. Increase explicitly for larger captures.
    noise_window : int
        For noise filling, use up to this many received samples on each side
        of each gap (default 128). At least two finite I/Q pairs are required.
    rng : int or numpy.random.Generator or None
        Seed or generator for reproducible noise filling.

    Returns
    -------
    dict
        Deep copy with expanded I/Q, counters, metadata and ``num_samples``.
        ``packet_missing`` marks inserted samples (including earlier fills).
        Missing ``packet_error`` entries are 1, telescope timestamps are 0
        (unknown), and stream flags/modulation tags are -1 (unknown). These
        metadata values are never interpolated. Other metadata is copied.
        ``sample_rate`` is unchanged. No leading/trailing losses are inferred.

        Signed or unsigned 32-bit counter wraps are supported and retained in
        the input convention. Duplicate/backward counters are rejected.

    Examples
    --------
    >>> filled = fill_gaps(samples)  # doctest: +SKIP
    >>> filled = fill_gaps(samples, method='interpolate')  # doctest: +SKIP
    >>> filled = fill_gaps(samples, method='noise', rng=42)  # doctest: +SKIP
    >>> inserted = filled['packet_missing']  # doctest: +SKIP
    """
    import copy

    if method not in ('nan', 'interpolate', 'noise'):
        raise ValueError("method must be 'nan', 'interpolate' or 'noise'")
    if method == 'noise':
        if not isinstance(noise_window, (int, np.integer)) or noise_window < 1:
            raise ValueError('noise_window must be a positive integer')
        rng = np.random.default_rng(rng)
    if not isinstance(max_missing, (int, np.integer)) or max_missing < 0:
        raise ValueError('max_missing must be a nonnegative integer')
    counter = np.asarray(samples['packet_counter'])
    if counter.ndim != 1 or (counter.size and counter.dtype.kind not in 'iu'):
        raise ValueError('packet_counter must be a 1D integer array')
    n = len(counter)
    if samples['num_samples'] != n:
        raise ValueError('num_samples does not match packet_counter')
    if n and (np.any(counter < -(2**31)) or np.any(counter > 2**32 - 1)):
        raise ValueError('packet_counter must contain 32-bit counter values')
    pc = counter.astype(np.int64)
    steps = np.diff(pc) % (2**32)
    if np.any((steps == 0) | (steps >= 2**31)):
        raise ValueError('duplicate, backward or ambiguous packet_counter jump')
    missing_count = sum(int(step) - 1 for step in steps)
    if missing_count > max_missing:
        raise ValueError(f'{missing_count} missing samples exceeds max_missing={max_missing}')
    positions = np.concatenate(([0], np.cumsum(steps))) if n else np.array([], dtype=int)
    size = n + missing_count
    inserted = np.ones(size, dtype=bool)
    inserted[positions] = False

    def expand(values, field, fill, interpolate=False):
        arr = np.asarray(values)
        if arr.ndim != 1 or len(arr) != n:
            raise ValueError(f'{field} must be a 1D array of length num_samples')
        if not missing_count:
            return arr.copy()
        dtype = np.result_type(arr.dtype, np.float64) if interpolate else arr.dtype
        if fill == -1:
            dtype = np.int64  # unknown flags must not wrap to unsigned values
        out = np.full(size, fill, dtype=dtype)
        out[positions] = arr
        if interpolate and method == 'interpolate' and n:
            targets = np.flatnonzero(inserted)
            out[targets] = np.interp(targets, positions, arr)
        return out

    if method in ('interpolate', 'noise') and missing_count:
        points = np.asarray(samples.get('modulation_point', []))
        if np.any(points != 0):
            raise ValueError('Synthetic filling of modulated samples is not supported; '
                             'use modulation.group_cycles for per-point interpolation')

    result = copy.deepcopy(samples)
    for field in ('i_data', 'q_data'):
        result[field] = {
            tone: expand(values, f'{field}[{tone}]', np.nan, interpolate=True)
            for tone, values in samples[field].items()
        }
    if method == 'noise' and missing_count:
        if samples['i_data'].keys() != samples['q_data'].keys():
            raise ValueError('i_data and q_data must contain the same tones')
        prior = np.asarray(samples.get('packet_missing', np.zeros(n, dtype=bool)))
        if prior.shape != (n,):
            raise ValueError('packet_missing must have length num_samples')
        for tone in samples['i_data']:
            iq = np.column_stack((samples['i_data'][tone], samples['q_data'][tone]))
            for left in np.flatnonzero(steps > 1):
                lo = max(0, left + 1 - noise_window)
                hi = min(n, left + 1 + noise_window)
                neighbours = iq[lo:hi]
                good = np.all(np.isfinite(neighbours), axis=1) & ~prior[lo:hi].astype(bool)
                neighbours = neighbours[good]
                if len(neighbours) < 2:
                    raise ValueError(f'Not enough finite received I/Q pairs near gap for tone {tone}')
                start, stop = int(positions[left]) + 1, int(positions[left + 1])
                draws = rng.multivariate_normal(
                    neighbours.mean(axis=0), np.cov(neighbours, rowvar=False),
                    size=stop - start)
                result['i_data'][tone][start:stop] = draws[:, 0]
                result['q_data'][tone][start:stop] = draws[:, 1]
    for field, fill in (('packet_error', 1), ('telescope_time', 0),
                        ('modulation_point', -1), ('modulation_settling', -1),
                        ('modulation_revision', -1)):
        if field in samples:
            result[field] = expand(samples[field], field, fill)
    if 'stream_flags' in samples:
        result['stream_flags'] = {
            flag: expand(values, f'stream_flags[{flag}]', -1)
            for flag, values in samples['stream_flags'].items()
        }
    if missing_count:
        counters = (int(pc[0]) + np.arange(size, dtype=np.int64)) % (2**32)
        if np.any(pc < 0) or counter.dtype == np.dtype('int32'):
            counters = (counters + 2**31) % (2**32) - 2**31
        result['packet_counter'] = counters
    else:
        result['packet_counter'] = counter.copy()
    prior_missing = np.asarray(samples.get('packet_missing', np.zeros(n, dtype=bool)))
    result['packet_missing'] = expand(prior_missing, 'packet_missing', True).astype(bool)
    result['num_samples'] = size
    return result


# ---------------------------------------------------------------------------
# PSD primitives (moved verbatim from plotting/_psd.py)
# ---------------------------------------------------------------------------

def compute_psd(data, sample_rate, method='welch', nperseg=None,
                window='hann', noverlap=None, scaling='density'):
    """
    Compute power spectral density of a 1D signal.

    Args:
        data: 1D array (real or complex).
        sample_rate: Sample rate in Hz.
        method: 'welch' for Welch's method, 'periodogram' for single FFT.
        nperseg: Segment length for Welch. Default: min(len(data), 256).
        window: Window function name (e.g. 'hann', 'blackman').
        noverlap: Overlap samples. Default: nperseg // 2.
        scaling: 'density' (V^2/Hz) or 'spectrum' (V^2).

    Returns:
        f: 1D frequency array (Hz).
        psd: 1D power spectral density array.
    """
    data = np.asarray(data)
    n = len(data)
    is_complex = np.iscomplexobj(data)

    if nperseg is None:
        nperseg = min(n, 256)
    if noverlap is None:
        noverlap = nperseg // 2

    if method == 'welch':
        if is_complex:
            # scipy.signal.welch doesn't handle complex directly;
            # compute two-sided PSD via FFT segments manually
            f, psd_i = scipy_signal.welch(
                data.real, fs=sample_rate, nperseg=nperseg,
                window=window, noverlap=noverlap, scaling=scaling,
                return_onesided=False)
            _, psd_q = scipy_signal.welch(
                data.imag, fs=sample_rate, nperseg=nperseg,
                window=window, noverlap=noverlap, scaling=scaling,
                return_onesided=False)
            psd = psd_i + psd_q
            # Sort by frequency for display
            order = np.argsort(f)
            f = f[order]
            psd = psd[order]
        else:
            f, psd = scipy_signal.welch(
                data, fs=sample_rate, nperseg=nperseg,
                window=window, noverlap=noverlap, scaling=scaling)
    elif method == 'periodogram':
        if is_complex:
            f, psd_i = scipy_signal.periodogram(
                data.real, fs=sample_rate, window=window,
                scaling=scaling, return_onesided=False)
            _, psd_q = scipy_signal.periodogram(
                data.imag, fs=sample_rate, window=window,
                scaling=scaling, return_onesided=False)
            psd = psd_i + psd_q
            order = np.argsort(f)
            f = f[order]
            psd = psd[order]
        else:
            f, psd = scipy_signal.periodogram(
                data, fs=sample_rate, window=window, scaling=scaling)
    else:
        raise ValueError(f"Unknown method '{method}'. Use 'welch' or 'periodogram'.")

    return f, psd


def carrier_power(z):
    """Carrier power ``|mean(z)|**2`` of a complex timestream."""
    power = float(np.abs(np.mean(np.asarray(z, dtype=complex))) ** 2)
    if not power > 0:
        raise ValueError(
            "psd_units='dbc' needs a carrier, but the mean I/Q is zero")
    return power


def log_bin_psd(f, psd, bins_per_decade=10):
    """
    Average PSD samples in logarithmically spaced positive-frequency bins.

    Frequencies in each occupied bin are represented by their geometric mean;
    PSD values are represented by their arithmetic mean. Non-positive
    frequencies are dropped because they cannot be displayed on a log x-axis.

    Args:
        f: 1D frequency array (Hz).
        psd: 1D PSD array aligned with ``f``.
        bins_per_decade: Number of logarithmic bins per frequency decade.

    Returns:
        f_binned: 1D geometric-mean bin frequencies.
        psd_binned: 1D mean PSD values.
    """
    try:
        bins_per_decade = float(bins_per_decade)
    except (TypeError, ValueError) as exc:
        raise ValueError("bins_per_decade must be a positive number") from exc
    if not np.isfinite(bins_per_decade) or bins_per_decade <= 0:
        raise ValueError("bins_per_decade must be a positive number")

    f = np.asarray(f, dtype=float)
    psd = np.asarray(psd, dtype=float)
    if f.shape != psd.shape:
        raise ValueError(f"f and psd must have the same shape, got {f.shape} and {psd.shape}")

    valid = np.isfinite(f) & np.isfinite(psd) & (f > 0)
    if not np.any(valid):
        return np.empty(0, dtype=float), np.empty(0, dtype=float)

    f = f[valid]
    psd = psd[valid]
    order = np.argsort(f)
    f = f[order]
    psd = psd[order]
    if f[0] == f[-1]:
        return f.copy(), psd.copy()

    n_bins = max(1, int(np.ceil(np.log10(f[-1] / f[0]) * bins_per_decade)))
    edges = np.logspace(np.log10(f[0]), np.log10(f[-1]), n_bins + 1)
    bin_index = np.searchsorted(edges, f, side='right') - 1
    bin_index = np.clip(bin_index, 0, n_bins - 1)

    f_binned = []
    psd_binned = []
    for b in range(n_bins):
        in_bin = bin_index == b
        if not np.any(in_bin):
            continue
        f_binned.append(np.exp(np.mean(np.log(f[in_bin]))))
        psd_binned.append(np.mean(psd[in_bin]))

    return np.asarray(f_binned), np.asarray(psd_binned)


def compute_psd_averaged(data_2d, sample_rate, **kwargs):
    """
    Compute PSD averaged over multiple repetitions.

    Computes PSD for each row of data_2d, then returns the mean and
    standard deviation across repetitions.

    Args:
        data_2d: 2D array, shape (N_reps, N_samples).
        sample_rate: Sample rate in Hz.
        **kwargs: Passed to compute_psd().

    Returns:
        f: 1D frequency array (Hz).
        psd_mean: Mean PSD across repetitions.
        psd_std: Standard deviation of PSD across repetitions.
    """
    data_2d = np.atleast_2d(data_2d)
    n_reps = data_2d.shape[0]

    psds = []
    f = None
    for i in range(n_reps):
        fi, psd_i = compute_psd(data_2d[i], sample_rate, **kwargs)
        psds.append(psd_i)
        if f is None:
            f = fi

    psds = np.array(psds)
    return f, np.mean(psds, axis=0), np.std(psds, axis=0)


def compute_psd_concatenated(data_2d, sample_rate, detrend_blocks='constant',
                             **kwargs):
    """
    Concatenate repetitions then compute a single PSD.

    Args:
        data_2d: 2D array, shape (N_reps, N_samples).
        sample_rate: Sample rate in Hz.
        detrend_blocks: Remove a per-block baseline before concatenating.
            ``'constant'`` (the default, also spelled ``True``) subtracts each
            block's own mean; ``'linear'`` removes a per-block straight-line
            fit; ``False`` concatenates the blocks unchanged. The default is
            on because block-to-block drift is the one real hazard of
            concatenation -- see Notes.
        **kwargs: Passed to compute_psd().

    Returns:
        f: 1D frequency array (Hz).
        psd: PSD of the concatenated signal.

    Notes:
        Blocks are captured at different times, so gain or phase drift
        between them makes the concatenated record a step train rather than
        one continuous stream. Welch detrends each *segment*, not each block,
        so the steps survive whenever ``nperseg`` spans a block boundary.
        For a block-to-block offset ``m`` and block length ``L`` they add a
        flat one-sided level ``2 m**2 L / fs`` below ``fs / L``, with an
        ``f**-2`` skirt above it. Against a white floor ``2 sigma**2 / fs``
        that is a ratio of ``L (m / sigma)**2``, so an offset of a few
        percent of the sample rms can dominate the answer. Detrending
        removes it, at the cost of also suppressing genuine power on
        timescales longer than one block (below ``fs / L``).
    """
    data_2d = np.atleast_2d(data_2d)
    if detrend_blocks:
        kind = 'constant' if detrend_blocks is True else detrend_blocks
        if kind not in ('constant', 'linear'):
            raise ValueError(
                "detrend_blocks must be False, True, 'constant' or 'linear', "
                f"got {detrend_blocks!r}")
        data_2d = scipy_signal.detrend(data_2d, axis=-1, type=kind)
    concatenated = data_2d.ravel()
    return compute_psd(concatenated, sample_rate, **kwargs)


# ---------------------------------------------------------------------------
# Timestream -> quantity helpers (moved from plotting/timestream.py)
# ---------------------------------------------------------------------------

def _get_tone_data(ts_data, tones=None):
    """
    Extract I/Q arrays for selected tones from parsed timestream data.

    Args:
        ts_data: dict from parse_samples() with 'i_data', 'q_data'.
        tones: List of tone indices (int) or None for first tone only.

    Returns:
        List of (tone_key, i_array, q_array) tuples.
    """
    i_data = ts_data['i_data']
    q_data = ts_data['q_data']
    tone_keys = sorted(i_data.keys())

    if tones is None:
        tones = [0]
    selected = []
    for t in tones:
        key = tone_keys[t]
        selected.append((key, np.array(i_data[key], dtype=float),
                         np.array(q_data[key], dtype=float)))
    return selected


def _resolve_reference_tone_frequency(ts_data, tone_idx, sweep_f,
                                      reference_tone_frequency):
    """Resolve the timestream tone frequency for freq/diss computation.

    reference_tone_frequency:
        None: look up ``ts_data['info']['tones']['frequencies_hz'][tone_idx]``.
            Falls back to ``np.mean(sweep_f)`` with a warning if unavailable.
        scalar: used directly for every tone.
        mapping: ``{tone_idx: frequency_hz}`` or ``{'0000': frequency_hz}``.
        array-like: indexed by absolute tone index.
    """
    if reference_tone_frequency is None:
        info = ts_data.get('info')
        if isinstance(info, dict):
            tf = info.get('tones', {}).get('frequencies_hz')
            if tf is not None:
                try:
                    return float(np.asarray(tf)[tone_idx])
                except (IndexError, ValueError, TypeError):
                    pass
        warnings.warn(
            f"Tone {tone_idx} has no frequency in ts_data['info']; "
            "falling back to the midpoint of the sweep range. Pass "
            "reference_tone_frequency to set this explicitly.",
            stacklevel=3)
        return float(np.mean(sweep_f))
    if isinstance(reference_tone_frequency, Mapping):
        for key in (tone_idx, f"{tone_idx:04d}"):
            if key in reference_tone_frequency:
                return float(reference_tone_frequency[key])
        raise ValueError(
            f"reference_tone_frequency mapping missing tone {tone_idx}")

    if np.ndim(reference_tone_frequency):
        try:
            return float(np.asarray(reference_tone_frequency, dtype=float)[tone_idx])
        except IndexError as exc:
            raise ValueError(
                "array-like reference_tone_frequency is indexed by absolute "
                f"tone index; tone {tone_idx} is missing. Use a mapping for "
                "non-contiguous selected tones."
            ) from exc

    return float(reference_tone_frequency)


def _compute_freq_diss(ts_data, tone_key, i_arr, q_arr, sweep_data,
                       reference_tone_frequency=None, smooth_window_hz=1000,
                       conversion_method='linearized', calibrations=None):
    """
    Compute fractional frequency and dissipation from timestream + sweep.

    Uses the canonical resonator conversion helpers. ``conversion_method``
    selects the historical local linear estimate or fitted-model inversion.
    """
    from .resonator import (
        calibration_for_tone,
        interpolate_complex_trace,
        linearized_frequency_and_dissipation,
    )

    z_ts = i_arr + 1j * q_arr
    tone_idx = int(tone_key)

    sweep_f, sweep_i, sweep_q = _get_sweep_trace(sweep_data, tone_idx)
    sweep_z = sweep_i + 1j * sweep_q

    tone_freq = _resolve_reference_tone_frequency(
        ts_data, tone_idx, sweep_f, reference_tone_frequency)

    if conversion_method == 'linearized':
        frac_f, frac_d = linearized_frequency_and_dissipation(
            sweep_f, sweep_z, tone_freq, z_ts,
            smooth_window_hz=smooth_window_hz)
    elif conversion_method in ('mobius', 'circle'):
        calibration = calibration_for_tone(calibrations, tone_idx)
        if calibration is None:
            raise ValueError(
                f"conversion_method={conversion_method!r} requires a "
                f"calibration for tone {tone_idx}")
        reference_s21 = interpolate_complex_trace(sweep_f, sweep_z, tone_freq)
        df_hz, frac_d = calibration.convert_referenced_raw_iq(
            tone_freq, z_ts, reference_s21, method=conversion_method)
        frac_f = df_hz / tone_freq
    else:
        raise ValueError(
            "conversion_method must be 'linearized', 'mobius', or 'circle'")

    return frac_f, frac_d


def _has_precomputed_freq_diss(ts_data):
    """Return True when a timestream dict already carries demodulated freq/loss."""
    return any(key in ts_data for key in (
        'resonance_shift_hz',
        'frequency_shift_hz',
        'frequency_hz',
    ))


def _warn_ignored_conversion_args(conversion_method, calibrations):
    """
    Warn when conversion knobs cannot take effect because the timestream
    already carries demodulated frequency/dissipation.
    """
    ignored = []
    if conversion_method is not None:
        ignored.append(f'conversion_method={conversion_method!r}')
    if calibrations is not None:
        ignored.append('calibrations')
    if not ignored:
        return
    warnings.warn(
        'ts_data already carries demodulated frequency/dissipation, so '
        + ' and '.join(ignored) + ' cannot affect this result. The '
        'conversion was chosen when the timestream was demodulated: pass '
        'calibration= to demodulate_timestream for the calibrated Mobius '
        'inversion, or omit it for the linearized estimate.',
        stacklevel=3)


def _sample_tone_column(ts_data, key, tone_idx):
    """Extract a tone column from sample-major or tone-major arrays."""
    arr = np.asarray(ts_data[key])
    if arr.ndim == 1:
        if tone_idx != 0:
            raise ValueError(f"{key!r} is 1D and contains only tone 0")
        return arr
    if arr.ndim != 2:
        raise ValueError(f"{key!r} must be 1D or 2D, got {arr.ndim}D")

    n_samples = ts_data.get('num_samples')
    if n_samples is not None and arr.shape[0] == int(n_samples):
        return arr[:, tone_idx]
    if tone_idx < arr.shape[0]:
        return arr[tone_idx]
    if tone_idx < arr.shape[1]:
        return arr[:, tone_idx]
    raise ValueError(f"Tone {tone_idx} not present in {key!r} shape {arr.shape}")


def _precomputed_freq_diss(ts_data, tone_key, reference_tone_frequency=None):
    """Read demodulated fractional frequency and dissipation from ``ts_data``."""
    tone_idx = int(tone_key)
    freq_key = next(
        key for key in ('resonance_shift_hz', 'frequency_shift_hz', 'frequency_hz')
        if key in ts_data)
    freq_hz = _sample_tone_column(ts_data, freq_key, tone_idx)

    info = ts_data.get('info') if isinstance(ts_data, dict) else None
    tone_freq = None
    if reference_tone_frequency is None and isinstance(info, dict):
        freqs = info.get('tones', {}).get('frequencies_hz')
        if freqs is not None:
            try:
                tone_freq = float(np.asarray(freqs, dtype=float)[tone_idx])
            except (IndexError, TypeError, ValueError):
                tone_freq = None
    elif reference_tone_frequency is not None:
        if isinstance(reference_tone_frequency, Mapping):
            for key in (tone_idx, tone_key):
                if key in reference_tone_frequency:
                    tone_freq = float(reference_tone_frequency[key])
                    break
        elif np.ndim(reference_tone_frequency):
            try:
                tone_freq = float(np.asarray(reference_tone_frequency, dtype=float)[tone_idx])
            except IndexError as exc:
                raise ValueError(
                    "array-like reference_tone_frequency is indexed by absolute "
                    f"tone index; tone {tone_idx} is missing. Use a mapping for "
                    "non-contiguous selected tones."
                ) from exc
        else:
            tone_freq = float(reference_tone_frequency)

    if tone_freq is None or tone_freq == 0.0:
        raise ValueError(
            "Precomputed frequency timestreams need tone frequencies in "
            "ts_data['info']['tones']['frequencies_hz'] or an explicit "
            "reference_tone_frequency to plot fractional frequency.")

    frac_f = freq_hz / tone_freq
    if 'dissipation' in ts_data:
        frac_d = _sample_tone_column(ts_data, 'dissipation', tone_idx)
    else:
        frac_d = np.full_like(frac_f, np.nan, dtype=float)
    return frac_f, frac_d


def _filter_psd_dc_point(f, p, include_dc_point):
    """Drop the zero-frequency PSD point for log-x plotting by default."""
    if include_dc_point:
        return f, p

    f = np.asarray(f)
    p = np.asarray(p)
    non_dc = f != 0
    return f[non_dc], p[non_dc]


def _prepare_psd_for_log_plot(f, p, include_dc_point, log_bin, bins_per_decade):
    """Apply PSD display filters shared by all PSD panels."""
    f, p = _filter_psd_dc_point(f, p, include_dc_point)
    if log_bin:
        f, p = log_bin_psd(f, p, bins_per_decade=bins_per_decade)
    return f, p


def _compute_cleaned_iq(ts_data, decorrelate_modes, decorrelate_tones,
                        blind_tone_modes, blind_tones):
    """Return cleaned (I, Q) rows keyed by tone index, without a sweep.

    Blind-tone subtraction runs first, then SVD common-mode removal, matching
    the order used for freq/diss. Both steps work on the raw I/Q, so this path
    needs no resonance sweep -- it is the one to use for loopback data.
    """
    analysis_tones = decorrelate_tones
    cleaned_ts_data = ts_data
    if blind_tone_modes:
        cleaned_ts_data, blind_info = remove_blind_tone_common_modes(
            ts_data,
            n_modes=blind_tone_modes,
            regular_tones=analysis_tones,
            blind_tones=blind_tones,
            return_info=True,
        )
        analysis_tones = blind_info['regular_tones']
    if decorrelate_modes:
        cleaned_ts_data, svd_info = remove_iq_common_modes_svd(
            cleaned_ts_data,
            n_modes=decorrelate_modes,
            tones=analysis_tones,
            return_info=True,
        )
        analysis_tones = svd_info['tones']

    tone_keys = sorted(cleaned_ts_data['i_data'])
    cleaned = {}
    for tone_index in analysis_tones:
        tone_key = tone_keys[int(tone_index)]
        cleaned[int(tone_key)] = (
            np.asarray(cleaned_ts_data['i_data'][tone_key], dtype=float),
            np.asarray(cleaned_ts_data['q_data'][tone_key], dtype=float),
        )
    return cleaned


def _compute_cleaned_freq_diss(
        ts_data, sweep_data, plotted_tones, decorrelate_modes,
        decorrelate_tones, blind_tone_modes, blind_tones,
        reference_tone_frequency, smooth_window_hz, conversion_method,
        calibrations):
    """Return reusable raw rows and sequentially cleaned freq/diss rows."""
    analysis_tones = plotted_tones
    if decorrelate_modes:
        info = ts_data.get('info')
        metadata = info.get('tones', {}) if isinstance(info, dict) else {}
        regular_tones = metadata.get('regular_indices')
        analysis_tones = decorrelate_tones
        if analysis_tones is None:
            analysis_tones = regular_tones if (
                regular_tones is not None and len(regular_tones)
            ) else None

    cleaned_ts_data = ts_data
    if blind_tone_modes:
        cleaned_ts_data, blind_info = remove_blind_tone_common_modes(
            ts_data,
            n_modes=blind_tone_modes,
            regular_tones=analysis_tones,
            blind_tones=blind_tones,
            return_info=True,
        )
        if analysis_tones is None:
            analysis_tones = blind_info['regular_tones']

    converted = fractional_frequency_and_dissipation_timestreams(
        cleaned_ts_data,
        sweep_data,
        tones=analysis_tones,
        reference_tone_frequency=reference_tone_frequency,
        smooth_window_hz=smooth_window_hz,
        method=conversion_method,
        calibrations=calibrations,
    )
    if decorrelate_modes:
        frequency = remove_common_modes_svd(
            converted['frequency'], decorrelate_modes)
        dissipation = remove_common_modes_svd(
            converted['dissipation'], decorrelate_modes)
    else:
        frequency = converted['frequency']
        dissipation = converted['dissipation']

    raw = {}
    if not blind_tone_modes:
        raw = {
            int(tone_index): (converted['frequency'][row],
                              converted['dissipation'][row])
            for row, tone_index in enumerate(converted['tone_indices'])
        }
    cleaned = {
        int(tone_index): (frequency[row], dissipation[row])
        for row, tone_index in enumerate(converted['tone_indices'])
    }
    return raw, cleaned


def _nonsettling_decimation(ts_data, sample_rate):
    """Non-settling sample indices + the decimated sample rate for a PSD.

    ``demodulate_timestream`` leaves ``modulation_settling`` samples in place (as
    NaN when ``fill_settling='none'``), so a PSD of that stream is all-NaN.
    Dropping the flagged samples only yields a well-defined spectrum if the
    survivors are uniformly spaced -- true when each slot dwell keeps a single
    run of good samples (e.g. ``dwell - n_settle == 1``). Returns
    ``(keep_idx, decimated_rate)``; raises if the flag is missing or the
    survivors are not uniformly spaced.
    """
    settling = ts_data.get('modulation_settling')
    if settling is None:
        raise ValueError(
            "exclude_settling=True but ts_data has no 'modulation_settling' flag; "
            "pass a demodulated/parsed modulation timestream, or exclude_settling=False.")
    settling = np.asarray(settling, dtype=int)
    keep_idx = np.flatnonzero(settling == 0)
    if keep_idx.size < 2:
        raise ValueError(
            f"exclude_settling kept {keep_idx.size} non-settling samples; need >= 2.")
    strides = np.unique(np.diff(keep_idx))
    if strides.size != 1:
        raise ValueError(
            "non-settling samples are not uniformly spaced "
            f"(sample strides {strides.tolist()}), so their PSD is not well "
            "defined. Use a dwell / n_settle split that keeps one run of good "
            "samples per slot (e.g. dwell - n_settle == 1), or fill the settling "
            "samples (fill_settling='interpolate') instead of dropping them.")
    return keep_idx, sample_rate / float(strides[0])


def _warn_if_psd_nan(arr, tone_key, quantity):
    """Warn (instead of silently returning nothing) when a PSD input is non-finite."""
    finite = np.isfinite(np.asarray(arr))
    if finite.size == 0 or not np.any(finite):
        warnings.warn(
            f"tone {tone_key}: {quantity} timestream is entirely non-finite; "
            "its PSD will be empty. For a demodulated modulation timestream the "
            "settling samples are NaN -- pass exclude_settling=True, or demodulate "
            "with fill_settling='interpolate'.", stacklevel=3)
    elif not np.all(finite):
        n_bad = int(np.sum(~finite))
        warnings.warn(
            f"tone {tone_key}: {quantity} timestream has {n_bad} non-finite "
            "sample(s); the Welch PSD will be NaN. Drop or fill them "
            "(exclude_settling=True, or fill_settling='interpolate').",
            stacklevel=3)


# ---------------------------------------------------------------------------
# Public compute entry point
# ---------------------------------------------------------------------------

def _as_ts_data(data, sample_rate=None):
    """Normalise ``data`` into a ``parse_samples``-shaped dict.

    Accepts a ``parse_samples`` dict (returned unchanged), a complex array of
    shape ``(n_tones, n_samples)`` or ``(n_samples,)``, or an ``(i, q)`` pair of
    real arrays with the same shapes. This is the only place in the module that
    inspects the input's type; everything below it works on arrays.
    """
    if isinstance(data, Mapping):
        if sample_rate is not None and 'sample_rate' not in data:
            data = dict(data, sample_rate=float(sample_rate))
        return data

    if isinstance(data, tuple) and len(data) == 2:
        i_rows = np.atleast_2d(np.asarray(data[0], dtype=float))
        q_rows = np.atleast_2d(np.asarray(data[1], dtype=float))
        if i_rows.shape != q_rows.shape:
            raise ValueError(
                f"i and q must have the same shape, got {i_rows.shape} and "
                f"{q_rows.shape}")
    else:
        z = np.atleast_2d(np.asarray(data))
        i_rows = z.real.astype(float)
        q_rows = z.imag.astype(float)

    if sample_rate is None:
        raise ValueError(
            "sample_rate is required when passing plain arrays; pass a "
            "parse_samples dict to take it from the capture metadata.")

    keys = [f'{n:04d}' for n in range(i_rows.shape[0])]
    return {
        'i_data': {k: i_rows[n] for n, k in enumerate(keys)},
        'q_data': {k: q_rows[n] for n, k in enumerate(keys)},
        'sample_rate': float(sample_rate),
        'num_samples': int(i_rows.shape[1]),
    }


def timestream_psds(ts_data, format='iq', tones=None, sweep_data=None,
                    reference_tone_frequency=None, psd_kwargs=None,
                    smooth_window_hz=1000, *,
                    sample_rate=None,
                    conversion_method=None,
                    calibrations=None,
                    include_dc_point=False,
                    log_bin=False,
                    bins_per_decade=20,
                    decorrelate_modes=0,
                    decorrelate_tones=None,
                    blind_tone_modes=0,
                    blind_tones=None,
                    exclude_settling=False,
                    apply_readout_correction=True, export=None):
    """
    Power spectral densities of every selected tone in a timestream capture.

    This is the compute half of ``plotting.plot_timestream_psd``: identical
    arguments and identical numbers, returned as arrays rather than drawn.

    Parameters
    ----------
    ts_data : dict or ndarray or tuple of ndarray
        A ``parse_samples()`` dict, a complex array of shape
        ``(n_tones, n_samples)`` or ``(n_samples,)``, or an ``(i, q)`` pair of
        real arrays. Plain arrays require ``sample_rate`` and support
        ``format='iq'`` and ``'magphase'`` only, since the other options read
        capture metadata.
    format : {'iq', 'magphase', 'freq_diss'}, optional
        Quantity whose PSD is computed. Default ``'iq'``.
    tones : sequence of int, optional
        Tone indices to include. ``None`` (default) selects tone 0 only.
    sweep_data : dict, optional
        ``parse_sweep_data()`` dict. Required for ``format='freq_diss'`` unless
        ``ts_data`` already carries demodulated frequency/dissipation.
    reference_tone_frequency : float or Mapping or array-like, optional
        Tone frequency used by the freq/diss conversion. ``None`` (default)
        reads ``ts_data['info']['tones']['frequencies_hz']``. A scalar applies
        to every tone; a ``{tone_idx: hz}`` mapping or an array indexed by
        absolute tone index sets them individually.
    psd_kwargs : dict, optional
        Passed through to `compute_psd` (``method``, ``nperseg``, ``window``,
        ``noverlap``, ``scaling``).
    smooth_window_hz : float, optional
        Sweep smoothing for the linearized freq/diss conversion, in Hz.
        ``None`` or ``0`` disables it. Default 1000.
    sample_rate : float, optional
        Sample rate in Hz. Required when ``ts_data`` is a plain array; ignored
        when the dict already carries one.
    conversion_method : {'linearized', 'mobius', 'circle'}, optional
        Freq/diss conversion. ``None`` (default) means ``'linearized'``.
        Ignored, with a warning, when ``ts_data`` is already demodulated and no
        cleaning modes are active. Note that only ``'linearized'`` and
        ``'mobius'`` return a true frequency/dissipation pair; ``'circle'``
        returns the radial proxy in the dissipation row.
    calibrations : optional
        Per-tone resonator calibrations or fit results, required by
        ``conversion_method='mobius'`` and ``'circle'``.
    include_dc_point : bool, optional
        Keep the zero-frequency PSD point. Default ``False``.
    log_bin : bool, optional
        Average the PSD into logarithmically spaced positive-frequency bins.
        Default ``False``.
    bins_per_decade : float, optional
        Log bins per decade when ``log_bin=True``. Default 20.
    decorrelate_modes : int, optional
        Remove this many leading SVD common modes across the tone rows. ``0``
        (default) disables it.
    decorrelate_tones : sequence of int, optional
        Tones used to estimate the SVD modes. ``None`` (default) uses the
        regular tones from the saved metadata when available.
    blind_tone_modes : int, optional
        Fit this many temporal modes from blind-tone variations and regress
        them out of the regular-tone I/Q. ``0`` (default) disables it.
    blind_tones : sequence of int, optional
        Explicit blind-tone indices. ``None`` (default) infers them from
        metadata.
    exclude_settling : bool, optional
        Compute from only the samples not flagged ``modulation_settling``, at
        the decimated rate. Incompatible with the cleaning options. Default
        ``False``.
    apply_readout_correction : bool, optional
        Apply the stored readout correction to samples and sweep. Default
        ``True``.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'format'`` : str
            The ``format`` argument, echoed.
        ``'panels'`` : tuple of str
            Panel names, ``('i', 'q')`` / ``('magnitude', 'phase')`` /
            ``('frequency', 'dissipation')``.
        ``'f'`` : ndarray, shape (n_freq,)
            Frequency axis in Hz, shared by every tone and panel.
        ``'sample_rate'`` : float
            Rate the PSDs were computed at -- the decimated rate when
            ``exclude_settling=True``.
        ``'tones'`` : ndarray, shape (n_tones,)
            Absolute tone indices, in the order of the PSD rows.
        ``'psd'`` : dict of ndarray
            One ``(n_tones, n_freq)`` array per panel name. Units are the
            square of the panel quantity per Hz: raw I/Q counts for ``'iq'``,
            counts and rad for ``'magphase'``, fractional for ``'freq_diss'``.
        ``'psd_cleaned'`` : dict of ndarray or None
            Same shape, after blind-tone and/or SVD cleaning. ``None`` when no
            cleaning option is active.
        ``'carrier'`` : ndarray, shape (n_tones,)
            ``|mean(z)|**2`` of each tone's raw I/Q, so a dBc figure can be
            reproduced later. ``0.0`` for a tone whose mean I/Q vanishes.
        ``'conversion_method'`` : str or None
            The freq/diss conversion actually used, or ``None`` for ``'iq'``
            and ``'magphase'``. ``'circle'`` marks a dissipation row that holds
            the radial proxy ``Delta(rho)`` rather than ``Im(delta_tilde)``.
        ``'reference_tone_frequency'`` : ndarray or None
            Per tone, the frequency the conversion was referred to, in Hz
            (``NaN`` where it could not be determined). ``None`` for ``'iq'``
            and ``'magphase'``. `subtract_psds` compares this between two
            captures to catch an on/off pair referred to different
            responsivities.
        ``'cleaning'`` : dict
            The cleaning options that were applied.
        ``'psd_kwargs'`` : dict
            The ``compute_psd`` options that were applied.
        ``'info'`` : dict or None
            ``ts_data['info']`` verbatim -- tone frequencies, powers, config.

    Raises
    ------
    ValueError
        If ``format`` is unknown, if ``format='freq_diss'`` is requested
        without ``sweep_data`` on a non-demodulated timestream, if
        ``exclude_settling`` is combined with a cleaning option or the kept
        samples are not uniformly spaced, or if ``log_bin`` produces
        different-length spectra for different tones.
    TypeError
        If ``decorrelate_modes`` or ``blind_tone_modes`` is not an integer.

    Notes
    -----
    The dissipation row follows the package convention: for
    ``conversion_method='linearized'`` and ``'mobius'`` it is
    ``Im(delta_tilde) = delta_Qi_inv / 2``, with the factor of 1/2 already
    applied.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> i = rng.normal(size=(2, 4096)) + 100.0
    >>> q = rng.normal(size=(2, 4096))
    >>> out = timestream_psds((i, q), sample_rate=488.28, tones=[0, 1])
    >>> out['panels']
    ('i', 'q')
    >>> out['psd']['i'].shape[0]
    2
    >>> bool(np.all(out['f'] > 0))
    True
    """
    ts_data = _as_ts_data(ts_data, sample_rate)
    ts_data = _with_samples_readout_correction(ts_data, apply_readout_correction)
    sweep_data = _with_sweep_readout_correction(sweep_data, apply_readout_correction)
    selected = _get_tone_data(ts_data, tones)
    capture_rate = ts_data['sample_rate']
    psd_kw = dict(psd_kwargs or {})

    keep_idx = None
    psd_sample_rate = capture_rate
    if exclude_settling:
        if decorrelate_modes or blind_tone_modes:
            raise ValueError(
                "exclude_settling is not supported together with SVD / blind-tone "
                "cleaning (those operate on the full-rate stream).")
        keep_idx, psd_sample_rate = _nonsettling_decimation(ts_data, capture_rate)

    precomputed_freq_diss = _has_precomputed_freq_diss(ts_data)
    if format == 'freq_diss' and sweep_data is None and not precomputed_freq_diss:
        raise ValueError("sweep_data is required for format='freq_diss'")
    if not isinstance(decorrelate_modes, (int, np.integer)):
        raise TypeError("decorrelate_modes must be an integer")
    if decorrelate_modes < 0:
        raise ValueError("decorrelate_modes must be non-negative")
    if not isinstance(blind_tone_modes, (int, np.integer)):
        raise TypeError("blind_tone_modes must be an integer")
    if blind_tone_modes < 0:
        raise ValueError("blind_tone_modes must be non-negative")
    if format not in PSD_PANELS:
        raise ValueError(
            f"Unknown format '{format}'. Use 'iq', 'magphase', or 'freq_diss'.")
    if ((decorrelate_modes or blind_tone_modes)
            and format == 'freq_diss' and sweep_data is None):
        raise ValueError(
            "SVD and blind-tone cleaning require sweep_data; precomputed "
            "modulation demodulation can be used only without these "
            "cleaning modes.")
    # With cleaning modes active the freq/diss is recomputed from raw I/Q, so
    # the conversion knobs do apply; otherwise a precomputed (demodulated)
    # timestream is used as-is and they cannot.
    if (format == 'freq_diss' and precomputed_freq_diss
            and not (decorrelate_modes or blind_tone_modes)):
        _warn_ignored_conversion_args(conversion_method, calibrations)
    if conversion_method is None:
        conversion_method = 'linearized'

    cleaning_on = bool(decorrelate_modes or blind_tone_modes)

    def _psd(arr, tone_key=None, quantity=None):
        """compute_psd, dropping settling samples first when exclude_settling."""
        a = np.asarray(arr)
        if keep_idx is not None:
            a = a[keep_idx]
        if quantity is not None:
            _warn_if_psd_nan(a, tone_key, quantity)
        f, p = compute_psd(a, psd_sample_rate, **psd_kw)
        return _prepare_psd_for_log_plot(
            f, p, include_dc_point, log_bin, bins_per_decade)

    panels = PSD_PANELS[format]

    def _panel_pair(key, i_arr, q_arr):
        """The two panels' (f, psd) pairs for one tone's I/Q rows."""
        if format == 'iq':
            return _psd(i_arr, key, 'I'), _psd(q_arr, key, 'Q')
        z = np.asarray(i_arr) + 1j * np.asarray(q_arr)
        zc = z[keep_idx] if keep_idx is not None else z
        _warn_if_psd_nan(zc, key, 'magnitude/phase')
        mag_f, mag_p = compute_psd(np.abs(zc), psd_sample_rate, **psd_kw)
        ph_f, ph_p = compute_psd(np.unwrap(np.angle(zc)), psd_sample_rate, **psd_kw)
        return (_prepare_psd_for_log_plot(mag_f, mag_p, include_dc_point,
                                          log_bin, bins_per_decade),
                _prepare_psd_for_log_plot(ph_f, ph_p, include_dc_point,
                                          log_bin, bins_per_decade))

    tone_indices = []
    carriers = []
    raw_rows = ([], [])
    cleaned_rows = ([], [])

    if format in ('iq', 'magphase'):
        cleaned_iq = {}
        if cleaning_on:
            cleaned_iq = _compute_cleaned_iq(
                ts_data, decorrelate_modes, decorrelate_tones,
                blind_tone_modes, blind_tones)

        for key, i_arr, q_arr in selected:
            tone_index = int(key)
            tone_indices.append(tone_index)
            carriers.append(
                float(np.abs(np.mean(i_arr + 1j * q_arr)) ** 2))

            (f1, p1), (f2, p2) = _panel_pair(key, i_arr, q_arr)
            raw_rows[0].append((f1, p1))
            raw_rows[1].append((f2, p2))

            if cleaning_on:
                if tone_index not in cleaned_iq:
                    raise ValueError(
                        f"Tone {tone_index} selected for analysis is not in "
                        "the cleaned tone set")
                (c1, q1), (c2, q2) = _panel_pair(key, *cleaned_iq[tone_index])
                cleaned_rows[0].append((c1, q1))
                cleaned_rows[1].append((c2, q2))
    else:
        raw_converted = {}
        cleaned = {}
        if cleaning_on:
            raw_converted, cleaned = _compute_cleaned_freq_diss(
                ts_data,
                sweep_data,
                [int(key) for key, _, _ in selected],
                decorrelate_modes,
                decorrelate_tones,
                blind_tone_modes,
                blind_tones,
                reference_tone_frequency,
                smooth_window_hz,
                conversion_method,
                calibrations,
            )

        for key, i_arr, q_arr in selected:
            tone_index = int(key)
            tone_indices.append(tone_index)
            carriers.append(
                float(np.abs(np.mean(i_arr + 1j * q_arr)) ** 2))

            if precomputed_freq_diss and not cleaning_on:
                frac_f, frac_d = _precomputed_freq_diss(
                    ts_data, key,
                    reference_tone_frequency=reference_tone_frequency)
            elif cleaning_on and tone_index in raw_converted:
                frac_f, frac_d = raw_converted[tone_index]
            else:
                frac_f, frac_d = _compute_freq_diss(
                    ts_data, key, i_arr, q_arr, sweep_data,
                    reference_tone_frequency=reference_tone_frequency,
                    smooth_window_hz=smooth_window_hz,
                    conversion_method=conversion_method,
                    calibrations=calibrations)
            raw_rows[0].append(_psd(frac_f, key, 'frequency'))
            raw_rows[1].append(_psd(frac_d, key, 'dissipation'))

            if cleaning_on:
                if tone_index not in cleaned:
                    raise ValueError(
                        f"Tone {tone_index} selected for analysis is not in "
                        "the cleaned tone set")
                clean_f, clean_d = cleaned[tone_index]
                cf, cp = compute_psd(clean_f, psd_sample_rate, **psd_kw)
                df, dp = compute_psd(clean_d, psd_sample_rate, **psd_kw)
                cleaned_rows[0].append(_prepare_psd_for_log_plot(
                    cf, cp, include_dc_point, log_bin, bins_per_decade))
                cleaned_rows[1].append(_prepare_psd_for_log_plot(
                    df, dp, include_dc_point, log_bin, bins_per_decade))

    f_axis = _common_frequency_axis(raw_rows + cleaned_rows)

    result = {
        'format': format,
        'panels': panels,
        'f': f_axis,
        'sample_rate': float(psd_sample_rate),
        'tones': np.asarray(tone_indices, dtype=int),
        'psd': {name: _stack(rows) for name, rows in zip(panels, raw_rows)},
        'psd_cleaned': (
            {name: _stack(rows) for name, rows in zip(panels, cleaned_rows)}
            if cleaning_on else None),
        'carrier': np.asarray(carriers, dtype=float),
        # Tagged so a later consumer can tell a real dissipation row from the
        # radial proxy Delta(rho) that conversion_method='circle' returns.
        'conversion_method': conversion_method if format == 'freq_diss' else None,
        # The responsivity each row was referred to, recorded per tone so that
        # a later on/off subtraction can check both used the same one.
        'reference_tone_frequency': (
            _referred_frequencies(ts_data, tone_indices,
                                  reference_tone_frequency)
            if format == 'freq_diss' else None),
        'cleaning': {
            'decorrelate_modes': int(decorrelate_modes),
            'decorrelate_tones': decorrelate_tones,
            'blind_tone_modes': int(blind_tone_modes),
            'blind_tones': blind_tones,
        },
        'psd_kwargs': psd_kw,
        'info': ts_data.get('info'),
    }
    return _maybe_export(result, export)


def _referred_frequencies(ts_data, tone_indices, reference_tone_frequency):
    """Per-tone frequency the freq/diss conversion was referred to.

    ``NaN`` where it cannot be determined. Recording this is what lets
    `subtract_psds` catch an on/off pair whose two captures were referred to
    different responsivities, which inflates the off-resonance rows by orders
    of magnitude.
    """
    refs = []
    for idx in tone_indices:
        if reference_tone_frequency is None:
            # Defaulted: each capture converts at its own parked frequency.
            tf = None
            info = ts_data.get('info') if isinstance(ts_data, Mapping) else None
            if isinstance(info, Mapping):
                tf = (info.get('tones') or {}).get('frequencies_hz')
            try:
                refs.append(float(np.asarray(tf)[idx]))
            except (IndexError, ValueError, TypeError):
                refs.append(np.nan)
        else:
            # Explicit: resolves without touching ts_data or the sweep.
            try:
                refs.append(_resolve_reference_tone_frequency(
                    {}, idx, None, reference_tone_frequency))
            except (ValueError, TypeError):
                refs.append(np.nan)
    return np.asarray(refs, dtype=float)


def _common_frequency_axis(row_groups):
    """The frequency axis shared by every computed spectrum.

    ``log_bin`` drops empty bins, so a tone whose PSD is partly non-finite can
    come back on a shorter axis than its neighbours. Rows keyed by panel only
    make sense on one axis, so that case is an error rather than a silent
    ragged array.
    """
    axes = [f for rows in row_groups for f, _ in rows]
    if not axes:
        return np.empty(0, dtype=float)
    f_axis = np.asarray(axes[0], dtype=float)
    for other in axes[1:]:
        other = np.asarray(other, dtype=float)
        if other.shape != f_axis.shape or not np.allclose(other, f_axis):
            raise ValueError(
                "tones produced different PSD frequency axes "
                f"({f_axis.size} vs {other.size} points). This happens when "
                "log_bin=True and some tone's spectrum is partly non-finite; "
                "compute those tones separately, or drop the bad samples "
                "(exclude_settling=True, or fill_settling='interpolate').")
    return f_axis


def _stack(rows):
    """Stack per-tone (f, psd) pairs into one (n_tones, n_freq) array."""
    if not rows:
        return np.empty((0, 0), dtype=float)
    return np.vstack([np.asarray(p, dtype=float) for _, p in rows])


# ---------------------------------------------------------------------------
# PSDs with samples excluded
# ---------------------------------------------------------------------------

def _welch_segments(x, sample_rate, nperseg, noverlap, window, scaling):
    """Per-segment PSDs whose mean is exactly ``compute_psd``'s Welch result.

    Returns ``(f, S, starts)`` with ``S`` of shape ``(n_freq, n_segments)`` and
    ``starts`` the first sample index of each segment.
    """
    # detrend='constant' and the complex branch below must stay in step with
    # compute_psd: the equality "mean over these segments == compute_psd" is
    # what makes segment rejection an unbiased Welch estimate, and a change to
    # either function alone breaks it silently.
    kw = dict(fs=sample_rate, nperseg=nperseg, noverlap=noverlap,
              window=window, scaling=scaling, mode='psd', detrend='constant')
    if np.iscomplexobj(x):
        # Mirror compute_psd's complex branch: two-sided I plus two-sided Q.
        f, _, s_i = scipy_signal.spectrogram(x.real, return_onesided=False, **kw)
        _, _, s_q = scipy_signal.spectrogram(x.imag, return_onesided=False, **kw)
        s = s_i + s_q
        order = np.argsort(f)
        f, s = f[order], s[order]
    else:
        f, _, s = scipy_signal.spectrogram(x, **kw)

    step = nperseg - noverlap
    starts = np.arange(s.shape[1]) * step
    return f, s, starts


def masked_psd(data, sample_rate, flagged, method='segment', nperseg=None,
               noverlap=None, window='hann', scaling='density', export=None):
    """
    PSD of a 1-D signal with flagged samples excluded.

    Parameters
    ----------
    data : ndarray
        1-D signal, real or complex.
    sample_rate : float
        Sample rate in Hz.
    flagged : ndarray of bool
        Same length as ``data``. ``True`` marks a sample to exclude -- the
        ``'mask'`` rows from `find_glitches` have this sense.
    method : {'segment', 'interpolate'}, optional
        ``'segment'`` (default) drops whole Welch segments containing a
        flagged sample. Assumption-free, and the segment structure Welch
        already has makes it free; it costs frequency resolution only when
        the event rate is high. ``'interpolate'`` linearly fills the flagged
        samples instead, which keeps every segment but biases the
        low-frequency end.
    nperseg, noverlap, window, scaling
        As for `compute_psd`. ``nperseg`` defaults to ``min(len(data), 256)``
        and ``noverlap`` to ``nperseg // 2``.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'f'`` : ndarray, frequency axis in Hz.
        ``'psd'`` : ndarray, the PSD on that axis.
        ``'method'`` : str, the method used.
        ``'flagged_fraction'`` : float, fraction of input samples flagged.
        ``'n_segments'`` : int, segments Welch would have used.
        ``'n_segments_used'`` : int, segments surviving rejection. Equal to
        ``n_segments`` for ``method='interpolate'``.
        ``'lost_fraction'`` : float, ``1 - n_segments_used / n_segments``.
        This is the deadtime that matters, and it is larger than
        ``flagged_fraction`` whenever events are spread out.

    Raises
    ------
    ValueError
        If ``flagged`` does not match ``data`` in length, if ``method`` is
        unknown, or if every segment contains a flagged sample.

    Notes
    -----
    With nothing flagged, ``method='segment'`` reproduces
    ``compute_psd(data, sample_rate)`` to floating-point rounding.

    A sparse set of events can reject a large fraction of segments: one
    isolated sample flagged inside every ``nperseg``-long stretch rejects
    everything while flagging almost nothing. Check ``lost_fraction``, not
    ``flagged_fraction``, before trusting the result.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> x = rng.normal(size=8192)
    >>> bad = np.zeros(8192, dtype=bool)
    >>> out = masked_psd(x, 488.28, bad)
    >>> f, p = compute_psd(x, 488.28)
    >>> bool(np.allclose(out['psd'], p))
    True
    >>> x[4000] = 500.0                      # a glitch
    >>> bad[4000] = True
    >>> out = masked_psd(x, 488.28, bad)
    >>> out['n_segments_used'] < out['n_segments']
    True
    """
    x = np.asarray(data)
    flagged = np.asarray(flagged, dtype=bool)
    if x.ndim != 1:
        raise ValueError(f"data must be 1-D, got {x.ndim}-D")
    if flagged.shape != x.shape:
        raise ValueError(
            f"flagged must match data in shape, got {flagged.shape} and {x.shape}")
    if method not in ('segment', 'interpolate'):
        raise ValueError(
            f"method must be 'segment' or 'interpolate', got {method!r}")

    n = x.size
    if nperseg is None:
        nperseg = min(n, 256)
    if noverlap is None:
        noverlap = nperseg // 2
    flagged_fraction = float(np.mean(flagged)) if n else 0.0

    if method == 'interpolate':
        filled = x.astype(complex if np.iscomplexobj(x) else float, copy=True)
        good = ~flagged
        if not np.any(good):
            raise ValueError("every sample is flagged; nothing to interpolate from")
        idx = np.arange(n)
        if np.any(flagged):
            filled[flagged] = np.interp(idx[flagged], idx[good], filled.real[good])
            if np.iscomplexobj(x):
                filled[flagged] += 1j * np.interp(
                    idx[flagged], idx[good], np.asarray(x).imag[good])
        f, p = compute_psd(filled, sample_rate, nperseg=nperseg,
                           noverlap=noverlap, window=window, scaling=scaling)
        n_seg = max(1, (n - noverlap) // max(1, nperseg - noverlap))
        return {'f': f, 'psd': p, 'method': method,
                'flagged_fraction': flagged_fraction,
                'n_segments': int(n_seg), 'n_segments_used': int(n_seg),
                'lost_fraction': 0.0}

    f, s, starts = _welch_segments(x, sample_rate, nperseg, noverlap,
                                   window, scaling)
    bad_seg = np.array([bool(np.any(flagged[st:st + nperseg])) for st in starts])
    if bad_seg.all():
        raise ValueError(
            f"all {bad_seg.size} Welch segments contain a flagged sample "
            f"(nperseg={nperseg}); shorten nperseg or use method='interpolate'")
    keep = ~bad_seg
    lost = float(1.0 - keep.sum() / bad_seg.size)
    if lost > 0.05:
        warnings.warn(
            f"segment rejection discarded {lost:.1%} of Welch segments "
            f"({bad_seg.size - keep.sum()} of {bad_seg.size}) from only "
            f"{flagged_fraction:.2%} of samples. Deadtime this large biases "
            "the estimate toward quiet stretches; shorten nperseg so fewer "
            "segments straddle an event, or use method='interpolate'.",
            stacklevel=2)
    return _maybe_export({
        'f': f,
        'psd': s[:, keep].mean(axis=1),
        'method': method,
        'flagged_fraction': flagged_fraction,
        'n_segments': int(bad_seg.size),
        'n_segments_used': int(keep.sum()),
        'lost_fraction': lost,
    }, export)


# ---------------------------------------------------------------------------
# Band levels and the 1/f + white fit
# ---------------------------------------------------------------------------

def _as_psd_rows(data, psd=None, cleaned=False):
    """Normalise a PSD input into ``(f, {panel: rows}, panels, tones)``.

    Accepts a `timestream_psds` result or a bare ``(f, psd)`` pair, where
    ``psd`` is 1-D for one spectrum or ``(n_tones, n_freq)`` for several. This
    is the only place these functions inspect the input's type.
    """
    if isinstance(data, Mapping):
        if 'f' not in data or 'psd' not in data:
            raise ValueError(
                "dict input must be a timestream_psds() result with 'f' and "
                f"'psd' keys; got keys {sorted(data)[:8]}")
        key = 'psd_cleaned' if cleaned else 'psd'
        rows = data.get(key)
        if rows is None:
            raise ValueError(
                "cleaned=True but this result has no 'psd_cleaned'; it was "
                "computed without blind-tone or SVD cleaning")
        panels = tuple(data.get('panels', tuple(rows)))
        tones = np.asarray(data.get('tones', np.arange(
            np.atleast_2d(rows[panels[0]]).shape[0])), dtype=int)
        return (np.asarray(data['f'], dtype=float),
                {p: np.atleast_2d(np.asarray(rows[p], dtype=float)) for p in panels},
                panels, tones)

    if psd is None:
        raise ValueError(
            "pass either a timestream_psds() result, or a frequency array and "
            "a psd array")
    f = np.asarray(data, dtype=float)
    rows = np.atleast_2d(np.asarray(psd, dtype=float))
    if rows.shape[1] != f.size:
        raise ValueError(
            f"psd last axis ({rows.shape[1]}) must match f ({f.size})")
    return f, {'psd': rows}, ('psd',), np.arange(rows.shape[0], dtype=int)


def _warn_if_difference(data, n_averages=None):
    """Warn about the two ways a `subtract_psds` result breaks these estimators.

    Clipping: ``clip='positive'`` sets the non-positive bins to NaN, so
    averaging what survives keeps only the upward fluctuations. Where the
    difference is genuinely near zero this biases the level up to about
    ``sqrt(2/pi) * S_floor * sqrt(2/n)`` -- 6% of the readout floor at n=390,
    24% at n=22. A difference resolved well above the floor is unaffected,
    since nothing gets clipped.

    Distribution: both ``n_averages`` corrections here assume a bin is
    ``S * chi2_2n / 2n``. The difference of two such bins is not, so neither
    the median nor the log-space correction is the right size for it.
    """
    if not isinstance(data, Mapping) or 'positive_fraction' not in data:
        return
    if data.get('clip') == 'positive':
        warnings.warn(
            "this is a subtract_psds() result with clip='positive', so its "
            "non-positive bins are NaN and any level taken from it is biased "
            "upward wherever the difference is near zero. Recompute the "
            "difference with clip=None to quote a number, and read "
            "'positive_fraction' to see whether it is resolved at all.",
            stacklevel=3)
    if n_averages is not None:
        warnings.warn(
            "n_averages corrects for chi2_2n bin statistics, which a "
            "difference of two PSDs does not follow; the correction is only "
            "approximate here, and closest where the difference is well above "
            "the subtracted floor. Omit it to leave the level uncorrected.",
            stacklevel=3)


def _resolve_bands(bands, f):
    """Return ``(edges, names)`` for a list of pairs or a {name: pair} dict."""
    if bands is None:
        raise ValueError("bands is required, e.g. bands=[(1, 10), (10, 100)]")
    if isinstance(bands, Mapping):
        names = [str(k) for k in bands]
        pairs = [bands[k] for k in bands]
    else:
        pairs = list(bands)
        if pairs and np.isscalar(pairs[0]):
            pairs = [pairs]
        names = [f'{lo:g}-{hi:g} Hz' for lo, hi in pairs]

    edges = np.asarray(pairs, dtype=float)
    if edges.ndim != 2 or edges.shape[1] != 2:
        raise ValueError(
            f"bands must be (lo, hi) pairs, got shape {edges.shape}")
    if np.any(edges[:, 0] >= edges[:, 1]):
        bad = [n for n, (lo, hi) in zip(names, edges) if lo >= hi]
        raise ValueError(f"band lo must be below hi; offending bands: {bad}")

    finite = f[np.isfinite(f)]
    if finite.size:
        f_lo, f_hi = float(np.min(finite)), float(np.max(finite))
        outside = [n for n, (lo, hi) in zip(names, edges) if hi < f_lo or lo > f_hi]
        if outside:
            raise ValueError(
                f"bands {outside} lie entirely outside the available "
                f"frequency range {f_lo:g}-{f_hi:g} Hz")
    return edges, names


def psd_band_levels(data, psd=None, bands=None, statistic='mean',
                    cleaned=False, n_averages=None, export=None):
    """
    Average PSD level, and the ASD, in one or more frequency bands.

    Parameters
    ----------
    data : dict or ndarray
        A `timestream_psds` result, or the frequency array of an ``(f, psd)``
        pair.
    psd : ndarray, optional
        PSD values when ``data`` is a frequency array: 1-D for a single
        spectrum, or ``(n_tones, n_freq)``.
    bands : sequence of (lo, hi), or dict of {name: (lo, hi)}
        Band edges in Hz, inclusive of ``lo`` and exclusive of ``hi``.
        Required.
    statistic : {'mean', 'median'}, optional
        How bins inside a band are combined. ``'median'`` is the one to reach
        for when a band contains a spectral line. Default ``'mean'``.
    cleaned : bool, optional
        Use the ``'psd_cleaned'`` rows of a `timestream_psds` result instead
        of the raw ones. Default ``False``.
    n_averages : int, optional
        Number of Welch segments averaged into each PSD bin. Only used by
        ``statistic='median'``, where it removes that estimator's downward
        bias (see Notes). ``None`` (default) applies no correction.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'bands'`` : ndarray, shape (n_bands, 2), the edges used.
        ``'names'`` : ndarray of str, one label per band.
        ``'tones'`` : ndarray, shape (n_tones,).
        ``'panels'`` : tuple of str.
        ``'statistic'`` : str.
        ``'level'`` : dict of ndarray, ``(n_tones, n_bands)`` per panel, in
        the PSD's own units (per Hz).
        ``'asd'`` : dict of ndarray, ``sqrt(level)`` -- the amplitude spectral
        density, which is what normally gets quoted.
        ``'n_points'`` : dict of ndarray, finite bins averaged per band. A
        band resting on one or two bins is not a measurement; this is how you
        see that.

    Raises
    ------
    ValueError
        If ``bands`` is missing or malformed, if a band lies entirely outside
        the frequency range, or if ``statistic`` is unknown.

    Warns
    -----
    UserWarning
        When a band contains no finite bins. That band's level is NaN.

    Notes
    -----
    ``statistic='mean'`` is unbiased: a PSD bin averaged over ``n`` segments is
    ``S * chi2_{2n} / 2n``, whose mean is ``S``.

    ``'median'`` is **not**. The median of ``chi2_2 / 2`` is ``ln 2``, so on a
    single-segment PSD it reads 31% low in power and 17% low in amplitude; the
    bias falls to ~1% by 60 segments. Pass ``n_averages`` to divide it out.
    That is the price of its robustness against a spectral line sitting in the
    band, so prefer ``'median'`` when a line is present and correct for it,
    rather than avoiding it.

    Examples
    --------
    >>> import numpy as np
    >>> f = np.linspace(0.0, 100.0, 1001)[1:]
    >>> psd = 1e-18 / f + 1e-20                  # 1/f plus a white floor
    >>> out = psd_band_levels(f, psd, bands=[(1, 10), (10, 100)])
    >>> out['level']['psd'].shape
    (1, 2)
    >>> bool(out['level']['psd'][0, 0] > out['level']['psd'][0, 1])
    True
    >>> bool(np.allclose(out['asd']['psd'], np.sqrt(out['level']['psd'])))
    True
    """
    if statistic not in ('mean', 'median'):
        raise ValueError(
            f"statistic must be 'mean' or 'median', got {statistic!r}")
    f, rows, panels, tones = _as_psd_rows(data, psd, cleaned)
    _warn_if_difference(data, n_averages)
    edges, names = _resolve_bands(bands, f)
    combine = np.mean if statistic == 'mean' else np.median

    # The median of a chi2_{2n}/2n bin sits below its mean; divide that out so
    # 'median' and 'mean' quote the same level on line-free data.
    median_bias = 1.0
    if statistic == 'median' and n_averages is not None:
        dof = 2 * int(n_averages)
        median_bias = float(scipy_stats.chi2.median(dof) / dof)

    level, asd, n_points = {}, {}, {}
    empty = []
    for panel in panels:
        arr = rows[panel]
        out = np.full((arr.shape[0], edges.shape[0]), np.nan)
        counts = np.zeros((arr.shape[0], edges.shape[0]), dtype=int)
        for b, (lo, hi) in enumerate(edges):
            in_band = (f >= lo) & (f < hi)
            for t in range(arr.shape[0]):
                sel = in_band & np.isfinite(arr[t])
                counts[t, b] = int(sel.sum())
                if counts[t, b]:
                    out[t, b] = combine(arr[t][sel]) / median_bias
                else:
                    empty.append((panel, names[b]))
        level[panel] = out
        with np.errstate(invalid='ignore'):
            asd[panel] = np.sqrt(out)
        n_points[panel] = counts

    if empty:
        shown = sorted({f'{p}:{n}' for p, n in empty})[:6]
        warnings.warn(
            f"no finite PSD bins in {len(empty)} band/tone combination(s) "
            f"({', '.join(shown)}); their level is NaN. The available "
            f"frequency range is {np.nanmin(f):g}-{np.nanmax(f):g} Hz.",
            stacklevel=2)

    return _maybe_export({'bands': edges, 'names': np.asarray(names), 'tones': tones,
            'panels': panels, 'statistic': statistic, 'level': level,
            'asd': asd, 'n_points': n_points,
            'median_bias_correction': median_bias}, export)


def _seed_psd_model(f, p):
    """Seed ``(log10 A, alpha, log10 W)`` for the 1/f + white fit."""
    log_f = np.log10(f)
    top = log_f >= (log_f.max() - 1.0)          # top decade -> the white floor
    w = float(np.median(p[top])) if np.any(top) else float(np.median(p))
    w = max(w, np.finfo(float).tiny)

    bottom = log_f <= (log_f.min() + 1.0)       # bottom decade -> the 1/f part
    excess = p[bottom] - w
    ok = excess > 0
    if np.count_nonzero(ok) >= 2:
        slope, intercept = np.polyfit(log_f[bottom][ok], np.log10(excess[ok]), 1)
        alpha = float(np.clip(-slope, 0.0, 6.0))
        log10_a = float(intercept)
    else:
        alpha, log10_a = 1.0, float(np.log10(max(p.max(), w)))
    return [log10_a, alpha, float(np.log10(w))]


def _psd_model_log10(f, log10_a, alpha, log10_w):
    """log10 of ``A f**-alpha + W``, the space the fit is done in."""
    return np.log10(10.0 ** log10_a * f ** (-alpha) + 10.0 ** log10_w)


def fit_psd_model(data, psd=None, model='oneoverf_plus_white', f_range=None,
                  cleaned=False, rms_flag_dex=0.15, n_averages=None, export=None):
    """
    Fit ``S(f) = A f**-alpha + W`` to each tone and panel of a PSD.

    The fit is a least-squares minimisation of the residual in ``log10 S``,
    which gives every bin equal weight across many orders of magnitude -- the
    right choice for a spectrum plotted on log-log axes. Note that on a linear
    Welch frequency axis most bins sit in the top decade, so this weights
    *bins* equally, not decades; pass log-binned input (`log_bin_psd`) if you
    want the decades themselves weighted equally.

    Parameters
    ----------
    data : dict or ndarray
        A `timestream_psds` result, or the frequency array of an ``(f, psd)``
        pair.
    psd : ndarray, optional
        PSD values when ``data`` is a frequency array.
    model : {'oneoverf_plus_white'}, optional
        The only model for now; named so later ones can be added without
        changing the signature.
    f_range : (lo, hi), optional
        Restrict the fit to this frequency range in Hz. ``None`` (default)
        uses every positive, finite bin.
    cleaned : bool, optional
        Fit the ``'psd_cleaned'`` rows of a `timestream_psds` result.
    rms_flag_dex : float, optional
        Residual rms, in decades, above which a fit is flagged
        ``'high_residual'``. Default 0.15.
    n_averages : int, optional
        Number of Welch segments averaged into each PSD bin. Supplying it
        removes the downward bias that log-space fitting carries (see Notes),
        which is severe on few-segment spectra: 0.25 dex, a factor 1.8, for a
        single-segment periodogram. ``None`` (default) applies no correction
        and leaves ``A`` and ``W`` biased low.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'model'``, ``'panels'``, ``'tones'``, ``'f_range'`` : provenance.
        ``'A'``, ``'alpha'``, ``'W'`` : dict of ndarray, ``(n_tones,)`` per
        panel. ``A`` is in PSD units at 1 Hz, ``W`` in PSD units.
        ``'f_knee'`` : dict of ndarray, ``(A / W) ** (1 / alpha)`` in Hz --
        where the 1/f part crosses the white floor.
        ``'A_err'``, ``'alpha_err'``, ``'W_err'`` : dict of ndarray, one
        standard error each. ``A_err`` and ``W_err`` are propagated from the
        fitted log10 parameters.
        ``'covariance'`` : dict of ndarray, ``(n_tones, 3, 3)`` in the fitted
        basis ``(log10 A, alpha, log10 W)`` -- not ``(A, alpha, W)``. This is
        the basis a later weighted fit should consume.
        ``'rms_residual_dex'`` : dict of ndarray, rms of ``log10`` residuals.
        ``'n_points'`` : dict of ndarray, bins used per fit.
        ``'flags'`` : dict of ndarray of str, comma-separated per tone and
        empty when the fit is clean. Possible values: ``'no_data'``,
        ``'fit_failed'``, ``'high_residual'``, ``'alpha_at_bound'``,
        ``'white_degenerate'``, ``'no_1overf_in_range'``,
        ``'no_white_floor_in_range'``.

    Raises
    ------
    ValueError
        If ``model`` is unknown or ``f_range`` excludes every bin.

    Notes
    -----
    No reduced chi-square is reported, because the degrees of freedom are not
    knowable from ``(f, psd)`` alone: it depends on how many Welch segments
    were averaged, which `compute_psd` does not return. ``rms_residual_dex``
    is the honest quality metric here -- for a Welch PSD averaged over
    ``n`` segments the expected value is roughly ``0.434 / sqrt(n)``, so
    compare against that rather than against zero.

    **Levels come out low unless ``n_averages`` is given.** A PSD bin averaged
    over ``n`` segments is distributed as ``S * chi2_{2n} / 2n``, and taking
    its logarithm is not the logarithm of its mean:
    ``E[log10 S_hat] = log10 S + (psi(n) - ln n) / ln 10``. That offset is
    -0.2507 dex at ``n = 1`` (so ``A`` and ``W`` both read 44% low), -0.027
    dex at ``n = 8``, and under 0.01 dex beyond ``n ~ 50``. ``alpha`` and
    ``f_knee`` are unaffected, because the bias is the same at every
    frequency. Supplying ``n_averages`` adds the offset back before fitting.

    Bins from `compute_psd_concatenated` are **oversampled and correlated**:
    concatenating blocks of length ``L`` smooths the spectrum with a kernel of
    width ``fs / L``, so bins spaced more finely than that are not
    independent. Treating them as independent understates ``alpha_err`` by
    roughly ``sqrt(nperseg / L)``. The covariance below carries no knowledge
    of this.

    ``covariance`` comes from the log-space objective and is scaled by the
    residual variance, so it describes the fit's own consistency, not
    calibrated measurement errors.

    **Check the flags before quoting ``W``.** The model is over-parameterised
    on a spectrum with no visible 1/f knee: as ``alpha`` approaches zero
    ``A f**-alpha`` becomes a constant, so only the sum ``A + W`` is
    determined and the split between them is arbitrary. On white input this
    can put ``W`` an order of magnitude below the true floor while the fit
    still looks excellent. ``'white_degenerate'`` marks exactly that case --
    read ``A + W`` as the flat level there. Likewise
    ``'no_white_floor_in_range'`` means the spectrum had not levelled off by
    the top of the fitted band, so ``W`` is an extrapolation rather than a
    measurement.

    Examples
    --------
    >>> import numpy as np
    >>> f = np.logspace(-1, 2, 400)
    >>> psd = 3e-18 * f ** -1.5 + 2e-20
    >>> out = fit_psd_model(f, psd)
    >>> bool(abs(out['alpha']['psd'][0] - 1.5) < 0.01)
    True
    >>> bool(abs(np.log10(out['W']['psd'][0] / 2e-20)) < 0.01)
    True
    >>> str(out['flags']['psd'][0])
    ''
    """
    if model != 'oneoverf_plus_white':
        raise ValueError(
            f"unknown model {model!r}; only 'oneoverf_plus_white' is available")
    f, rows, panels, tones = _as_psd_rows(data, psd, cleaned)
    _warn_if_difference(data, n_averages)

    in_range = np.isfinite(f) & (f > 0)
    if f_range is not None:
        lo, hi = float(f_range[0]), float(f_range[1])
        in_range &= (f >= lo) & (f <= hi)
        if not np.any(in_range):
            raise ValueError(
                f"f_range {f_range} leaves no positive finite bins; the PSD "
                f"covers {np.nanmin(f[f > 0]):g}-{np.nanmax(f):g} Hz")

    # log10 of a chi2_{2n}/2n bin sits below log10 of its mean; add it back so
    # the fitted A and W are levels rather than log-means.
    log_bias = 0.0
    if n_averages is not None:
        n_avg = int(n_averages)
        if n_avg < 1:
            raise ValueError(f"n_averages must be >= 1, got {n_averages}")
        log_bias = float(
            (np.log(n_avg) - scipy_special.digamma(n_avg)) / np.log(10.0))

    out = {name: {} for name in
           ('A', 'alpha', 'W', 'f_knee', 'A_err', 'alpha_err', 'W_err',
            'covariance', 'rms_residual_dex', 'n_points', 'flags')}

    for panel in panels:
        arr = rows[panel]
        n_tones = arr.shape[0]
        vals = {k: np.full(n_tones, np.nan) for k in
                ('A', 'alpha', 'W', 'f_knee', 'A_err', 'alpha_err', 'W_err',
                 'rms_residual_dex')}
        cov = np.full((n_tones, 3, 3), np.nan)
        n_pts = np.zeros(n_tones, dtype=int)
        flags = []

        for t in range(n_tones):
            sel = in_range & np.isfinite(arr[t]) & (arr[t] > 0)
            n_pts[t] = int(sel.sum())
            if n_pts[t] < 4:
                flags.append('no_data')
                continue
            ff, pp = f[sel], arr[t][sel]
            log_pp = np.log10(pp) + log_bias
            try:
                popt, pcov = scipy_optimize.curve_fit(
                    _psd_model_log10, ff, log_pp,
                    p0=_seed_psd_model(ff, pp * 10.0 ** log_bias),
                    bounds=([-np.inf, 0.0, -np.inf], [np.inf, 6.0, np.inf]),
                    maxfev=20000)
            except (RuntimeError, ValueError):
                flags.append('fit_failed')
                continue

            log10_a, alpha, log10_w = popt
            resid = log_pp - _psd_model_log10(ff, *popt)
            rms = float(np.sqrt(np.mean(resid ** 2)))
            err = np.sqrt(np.diag(pcov))
            ln10 = np.log(10.0)

            vals['A'][t] = 10.0 ** log10_a
            vals['alpha'][t] = alpha
            vals['W'][t] = 10.0 ** log10_w
            vals['A_err'][t] = vals['A'][t] * ln10 * err[0]
            vals['alpha_err'][t] = err[1]
            vals['W_err'][t] = vals['W'][t] * ln10 * err[2]
            vals['rms_residual_dex'][t] = rms
            cov[t] = pcov

            # Below this the knee is meaningless (A and W are degenerate) and
            # the exponent overflows; leave it NaN rather than report inf.
            knee = np.nan
            if alpha >= 0.25:
                knee = float(10.0 ** ((log10_a - log10_w) / alpha))
            vals['f_knee'][t] = knee

            tags = []
            if rms > rms_flag_dex:
                tags.append('high_residual')
            if alpha >= 6.0 - 1e-6:
                tags.append('alpha_at_bound')
            # A f**-alpha goes flat as alpha -> 0, at which point it is
            # indistinguishable from W: the fit splits one constant between
            # them arbitrarily and W on its own is not the white floor.
            if alpha < 0.25:
                tags.append('white_degenerate')
            # Where the 1/f part sits relative to the fitted band. Below 1 at
            # the low edge means no 1/f excess was seen at all; above 1 at the
            # high edge means the spectrum never reached its floor.
            ratio_lo = 10.0 ** (log10_a - alpha * np.log10(ff[0]) - log10_w)
            ratio_hi = 10.0 ** (log10_a - alpha * np.log10(ff[-1]) - log10_w)
            if ratio_lo < 1.0:
                tags.append('no_1overf_in_range')
            if ratio_hi > 1.0:
                tags.append('no_white_floor_in_range')
            flags.append(','.join(tags))

        for k, v in vals.items():
            out[k][panel] = v
        out['covariance'][panel] = cov
        out['n_points'][panel] = n_pts
        out['flags'][panel] = np.asarray(flags, dtype=object).astype(str)

    out['model'] = model
    out['panels'] = panels
    out['tones'] = tones
    out['f_range'] = (np.nan, np.nan) if f_range is None else (
        float(f_range[0]), float(f_range[1]))
    return _maybe_export(out, export)


# ---------------------------------------------------------------------------
# Snapshot (pre-accumulation) PSDs
# ---------------------------------------------------------------------------

def _snapshot_acc_len(snapshot_data, sweep_data):
    """Resolve acc_len for matching pre-accumulation snapshots to sweep I/Q."""
    info = (sweep_data.get('info') if isinstance(sweep_data, Mapping) else None) \
           or (snapshot_data.get('info') if isinstance(snapshot_data, Mapping)
               else None)
    if isinstance(info, Mapping):
        acc_len = (info.get('pipeline') or {}).get('acc_len')
        if acc_len is not None:
            return acc_len
    raise ValueError(
        "Cannot determine acc_len for freq_diss scaling. "
        "Provide sweep_data with 'info.pipeline.acc_len' set.")


def _single_tone_snapshot(snapshot_data, func_name):
    """
    Return the single-tone snapshot dict these functions expect.

    ``batch_snapshots()`` returns one snapshot dict per tone under
    ``'results'``, so passing the batch dict (or its ``'results'`` mapping)
    here is an easy slip.  Unwrap it when it holds a single tone, and
    otherwise say which tone to pick.
    """
    if 'snapshots' in snapshot_data:
        return snapshot_data

    results = snapshot_data.get('results', snapshot_data)
    tones = sorted(tidx for tidx, snap in results.items()
                   if isinstance(snap, Mapping) and 'snapshots' in snap) \
        if isinstance(results, Mapping) else []
    if len(tones) == 1:
        return results[tones[0]]
    if tones:
        raise ValueError(
            f"{func_name}() plots one tone, but was given batch snapshot data "
            f"for tones {tones}. Pass a single tone, e.g. "
            f"batch_data['results'][{tones[0]}], or use "
            "plot_batch_snapshots() for all of them.")
    raise KeyError(
        "'snapshots' — expected snapshot data from "
        "get_accumulator_snapshots() or batch_snapshots().")


def _batch_info(batch, results, chosen):
    """An ``info``-shaped dict for a ``batch_snapshots()`` result.

    ``batch_snapshots`` now records ``'info'`` at capture time, so this is
    normally a pass-through. Captures saved before that carry the tone
    frequencies only in ``'tone_frequencies'``, indexed alongside
    ``'tone_indices'``; rebuild an ``info`` from those, because otherwise the
    freq/diss conversion finds no tone frequency and silently falls back to the
    midpoint of the sweep range, which is not where the tone was parked and
    puts the referred noise orders of magnitude out.
    """
    info = batch.get('info')
    if isinstance(info, Mapping) and (info.get('tones') or {}).get(
            'frequencies_hz') is not None:
        return info

    freqs = batch.get('tone_frequencies')
    idx = batch.get('tone_indices')
    if freqs is None:
        # Per-tone dicts may still carry it.
        freqs, idx = [], []
        for t in chosen:
            entry = results.get(t) or {}
            hz = entry.get('tone_frequency_hz', entry.get('frequency_hz'))
            if hz is None:
                return info
            freqs.append(float(hz))
            idx.append(int(t))
    freqs = np.asarray(freqs, dtype=float)
    idx = (np.arange(freqs.size) if idx is None
           else np.asarray(idx, dtype=int))
    # Index by absolute tone index, as the conversion expects.
    table = np.full(int(idx.max()) + 1 if idx.size else 0, np.nan)
    table[idx] = freqs
    merged = dict(info) if isinstance(info, Mapping) else {}
    tones_meta = dict(merged.get('tones') or {})
    tones_meta['frequencies_hz'] = table
    merged['tones'] = tones_meta
    return merged


def _as_snapshot_blocks(data, sample_rate=None, tones=None):
    """Normalise a snapshot input into ``(blocks, sample_rate, info)``.

    ``blocks`` is a list of ``(tone_index, complex 2-D array)`` pairs, one per
    selected tone, each ``(n_blocks, len_snapshot)``. Accepts a
    ``batch_snapshots`` dict, a single ``get_accumulator_snapshots`` dict, or a
    plain complex array. This is the only place these functions inspect the
    input's type.
    """
    if isinstance(data, Mapping):
        results = data.get('results')
        if isinstance(results, Mapping) and 'snapshots' not in data:
            available = sorted(t for t, s in results.items()
                               if isinstance(s, Mapping) and 'snapshots' in s)
            chosen = available if tones is None else [int(t) for t in tones]
            missing = [t for t in chosen if t not in results]
            if missing:
                raise ValueError(
                    f"tones {missing} are not in this batch; it holds "
                    f"{available}")
            per_tone = [(int(t), np.asarray(results[t]['snapshots'],
                                            dtype=complex)) for t in chosen]
            rate = data.get('sample_rate')
            if rate is None and per_tone:
                rate = results[chosen[0]].get('sample_rate')
            return per_tone, float(rate), _batch_info(data, results, chosen)

        single = _single_tone_snapshot(data, 'snapshot_psds')
        idx = int(single.get('tone_index', 0))
        if tones is not None and idx not in [int(t) for t in tones]:
            raise ValueError(
                f"this capture holds tone {idx}, but tones={list(tones)} was "
                "requested")
        return ([(idx, np.asarray(single['snapshots'], dtype=complex))],
                float(single['sample_rate']), single.get('info'))

    arr = np.asarray(data, dtype=complex)
    if arr.ndim == 1:
        arr = arr[None, :]
    if arr.ndim != 2:
        raise ValueError(
            "array input must be (n_blocks, len_snapshot) complex, got shape "
            f"{arr.shape}")
    if sample_rate is None:
        raise ValueError("array input requires sample_rate=")
    return [(0, arr)], float(sample_rate), None


def snapshot_psds(snapshot_data, format='iq', tones=None, sweep_data=None,
                  reference_tone_frequency=None, psd_kwargs=None,
                  smooth_window_hz=1000, *,
                  sample_rate=None,
                  method='concatenated',
                  detrend_blocks='constant',
                  conversion_method=None,
                  calibrations=None,
                  include_dc_point=False,
                  log_bin=False,
                  bins_per_decade=20, export=None):
    """
    Power spectral densities of pre-accumulation snapshot blocks.

    The fast counterpart of `timestream_psds`, returning the same shape so the
    two can be handed to `stitch_psds` together. Snapshots are taken one tone
    at a time in separate acquisitions, so the blocks are *not* contiguous in
    time and cannot be cleaned across tones.

    Parameters
    ----------
    snapshot_data : dict or ndarray
        A ``batch_snapshots()`` dict (several tones), a single
        ``get_accumulator_snapshots()`` dict, or a complex array of shape
        ``(n_blocks, len_snapshot)``. Arrays require ``sample_rate``.
    format : {'iq', 'magphase', 'freq_diss'}, optional
        Quantity whose PSD is computed. Default ``'iq'``.
    tones : sequence of int, optional
        Tone indices to include. ``None`` (default) takes every tone present.
    sweep_data : dict, optional
        ``parse_sweep_data()`` dict, required for ``format='freq_diss'``.
    reference_tone_frequency : float or Mapping or array-like, optional
        As in `timestream_psds`.
    psd_kwargs : dict, optional
        Passed to `compute_psd` (``nperseg``, ``window``, ``noverlap``, ...).
    smooth_window_hz : float, optional
        Sweep smoothing for the linearized conversion, in Hz. Default 1000.
    sample_rate : float, optional
        Sample rate in Hz. Required for array input.
    method : {'concatenated', 'averaged'}, optional
        ``'concatenated'`` (default) joins the blocks into one record and takes
        a single Welch PSD; ``'averaged'`` takes one periodogram per block and
        averages them, which also gives ``'psd_std'``.
    detrend_blocks : {'constant', 'linear', False}, optional
        Per-block detrend applied before concatenation, as in
        `compute_psd_concatenated`. Default ``'constant'``. Gain or phase drift
        between acquisitions turns each block boundary into a real step, and a
        random-step sequence has an ``f**-2`` tail that mimics ``1/f``.
        Ignored by ``method='averaged'``, which never joins blocks.
    conversion_method : {'linearized', 'mobius', 'circle'}, optional
        Freq/diss conversion. ``None`` (default) means ``'linearized'``.
    calibrations : optional
        Per-tone calibrations, required by ``'mobius'`` and ``'circle'``.
    include_dc_point : bool, optional
        Keep the zero-frequency point. Default ``False``.
    log_bin : bool, optional
        Average into logarithmically spaced bins. Default ``False``.
    bins_per_decade : float, optional
        Log bins per decade when ``log_bin=True``. Default 20.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        Everything `timestream_psds` returns (``'format'``, ``'panels'``,
        ``'f'``, ``'sample_rate'``, ``'tones'``, ``'psd'``, ``'carrier'``,
        ``'conversion_method'``, ``'reference_tone_frequency'``,
        ``'psd_kwargs'``, ``'info'``), with ``'psd_cleaned'`` always ``None``,
        plus:

        ``'source'`` : str
            ``'snapshots'``, so `stitch_psds` can tell the traces apart.
        ``'method'`` : str
            The ``method`` argument, echoed.
        ``'psd_std'`` : dict of ndarray or None
            Per-panel standard deviation across blocks, for
            ``method='averaged'`` only.
        ``'f_resolution'`` : float
            True spectral resolution, ``sample_rate / len_snapshot`` in Hz.
            Concatenation convolves the spectrum with a Fejer kernel of this
            width, so structure narrower than it is not resolved however fine
            the bin spacing looks.
        ``'bin_oversampling'`` : float
            Bins per resolution element, ``max(1, nperseg / len_snapshot)``.
            Above 1 the bins are correlated and a fit that treats them as
            independent understates its errors by ``sqrt`` of this.
        ``'n_avg'`` : int
            Welch segments (``'concatenated'``) or blocks (``'averaged'``).
        ``'effective_dof'`` : float
            ``2 * n_avg / bin_oversampling``, the chi-square degrees of freedom
            per resolution element. `fit_gr_rolloff` uses this rather than the
            bin count.
        ``'n_blocks'``, ``'len_snapshot'`` : int
            Block count and block length.
        ``'block_drift'`` : dict of ndarray
            Per panel, the standard deviation across blocks of each block's
            removed mean, divided by the within-block standard deviation.

            **Read it against** ``1/sqrt(len_snapshot)``, not against 1. The
            step train contributes about ``len_snapshot * block_drift**2``
            times the white floor, so at ``len_snapshot=1024`` the null value
            is 0.031, and that is also where the step floor *equals* the white
            floor. Measured with ``detrend_blocks=False``: 0.030 gives +0.7 dB,
            0.091 gives +7.4 dB, 0.82 gives +27 dB. The default per-block
            detrend removes it; this number says how much there was.

    Raises
    ------
    ValueError
        For an unknown ``format`` or ``method``, for array input without
        ``sample_rate``, for ``format='freq_diss'`` without ``sweep_data``, or
        when ``acc_len`` cannot be resolved for the freq/diss scaling.

    See Also
    --------
    timestream_psds : the accumulated-stream counterpart.
    stitch_psds : join this to an accumulated trace.
    compute_psd_concatenated : the underlying block-joining estimator.

    Notes
    -----
    Snapshots are pre-accumulation while the sweep I/Q are sums over
    ``acc_len`` samples, so ``format='freq_diss'`` scales the snapshots up by
    ``acc_len`` before converting. That factor is read from
    ``info['pipeline']['acc_len']``.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> blocks = (1.0 + 0.01 * rng.standard_normal((16, 1024))
    ...           + 0.01j * rng.standard_normal((16, 1024)))
    >>> out = snapshot_psds(blocks, sample_rate=600e3, psd_kwargs={'nperseg': 1024})
    >>> out['f_resolution'], out['bin_oversampling'], out['n_blocks']
    (585.9375, 1.0, 16)
    >>> out['psd']['i'].shape == (1, out['f'].size)
    True
    """
    if format not in PSD_PANELS:
        raise ValueError(
            f"unknown format {format!r}; use one of {tuple(PSD_PANELS)}")
    if method not in ('concatenated', 'averaged'):
        raise ValueError(
            f"unknown method {method!r}; use 'concatenated' or 'averaged'")
    if format == 'freq_diss' and sweep_data is None:
        raise ValueError("sweep_data is required for format='freq_diss'")

    blocks, rate, info = _as_snapshot_blocks(snapshot_data, sample_rate, tones)
    if not blocks:
        raise ValueError("no snapshot tones selected")
    panels = PSD_PANELS[format]
    psd_kw = dict(psd_kwargs or {})
    conversion = conversion_method or 'linearized'

    len_snapshot = blocks[0][1].shape[1]
    n_blocks = blocks[0][1].shape[0]
    nperseg = psd_kw.get('nperseg')
    if method == 'averaged':
        oversampling = 1.0
        n_avg = n_blocks
    else:
        total = n_blocks * len_snapshot
        # scipy silently clamps nperseg to the record length, so use the value
        # actually applied -- otherwise the reported oversampling, and the
        # error inflation fit_gr_rolloff derives from it, are overstated.
        seg = min(int(nperseg) if nperseg else min(total, 256), total)
        noverlap = psd_kw.get('noverlap')
        step = seg - (int(noverlap) if noverlap is not None else seg // 2)
        oversampling = max(1.0, seg / len_snapshot)
        n_avg = max(1, 1 + (total - seg) // max(step, 1))

    tone_indices, carriers = [], []
    rows = [[] for _ in panels]
    stds = [[] for _ in panels]
    drift = [[] for _ in panels]

    for tone_index, snaps in blocks:
        if snaps.shape[1] != len_snapshot or snaps.shape[0] != n_blocks:
            raise ValueError(
                f"tone {tone_index} has {snaps.shape} snapshots but tone "
                f"{tone_indices[0] if tone_indices else '?'} has "
                f"({n_blocks}, {len_snapshot}); PSD rows keyed by panel need "
                "one shape")
        tone_indices.append(tone_index)
        carriers.append(float(np.abs(np.mean(snaps)) ** 2))
        top, bot = _snapshot_panel_data(
            snaps, format, tone_index, info, sweep_data,
            reference_tone_frequency, smooth_window_hz, conversion,
            calibrations, snapshot_data)

        for slot, data2d in enumerate((top, bot)):
            # Per-block mean scatter against within-block scatter: how much of
            # the low-frequency end is block-to-block drift rather than noise.
            means = np.mean(data2d, axis=1)
            within = float(np.mean(np.std(data2d, axis=1)))
            drift[slot].append(
                float(np.std(means) / within) if within > 0 else np.nan)

            if method == 'averaged':
                fa, p_mean, p_std = compute_psd_averaged(
                    data2d, rate, **psd_kw)
                rows[slot].append(_prepare_psd_for_log_plot(
                    fa, p_mean, include_dc_point, log_bin, bins_per_decade))
                stds[slot].append(_prepare_psd_for_log_plot(
                    fa, p_std, include_dc_point, log_bin, bins_per_decade)
                    if p_std is not None else (fa, np.zeros_like(p_mean)))
            else:
                fc, p = compute_psd_concatenated(
                    data2d, rate, detrend_blocks=detrend_blocks, **psd_kw)
                rows[slot].append(_prepare_psd_for_log_plot(
                    fc, p, include_dc_point, log_bin, bins_per_decade))

    f_axis = _common_frequency_axis(rows + (stds if method == 'averaged' else []))
    return _maybe_export({
        'format': format,
        'source': 'snapshots',
        'method': method,
        'panels': panels,
        'f': f_axis,
        'sample_rate': float(rate),
        'tones': np.asarray(tone_indices, dtype=int),
        'psd': {name: _stack(r) for name, r in zip(panels, rows)},
        'psd_cleaned': None,
        'psd_std': ({name: _stack(s) for name, s in zip(panels, stds)}
                    if method == 'averaged' else None),
        'carrier': np.asarray(carriers, dtype=float),
        'conversion_method': conversion if format == 'freq_diss' else None,
        'reference_tone_frequency': (
            _referred_frequencies({'info': info}, tone_indices,
                                  reference_tone_frequency)
            if format == 'freq_diss' else None),
        'f_resolution': float(rate) / len_snapshot,
        'bin_oversampling': float(oversampling),
        'n_avg': int(n_avg),
        'effective_dof': 2.0 * n_avg / oversampling,
        'n_blocks': int(n_blocks),
        'len_snapshot': int(len_snapshot),
        'block_drift': {name: np.asarray(d, dtype=float)
                        for name, d in zip(panels, drift)},
        'cleaning': None,
        'psd_kwargs': psd_kw,
        'info': info,
    }, export)


def _snapshot_panel_data(snaps, format, tone_index, info, sweep_data,
                         reference_tone_frequency, smooth_window_hz,
                         conversion_method, calibrations, snapshot_data):
    """The two (n_blocks, len_snapshot) panel arrays for one tone."""
    if format == 'iq':
        return snaps.real, snaps.imag
    if format == 'magphase':
        return np.abs(snaps), np.unwrap(np.angle(snaps), axis=-1)

    # Snapshots are pre-accumulation; sweep I/Q are sums over acc_len samples,
    # so scale up to put both in the same units before converting.
    acc_len = _snapshot_acc_len(snapshot_data, sweep_data)
    scaled = snaps * acc_len
    ts_like = {'info': info}
    top, bot = [], []
    for block in range(scaled.shape[0]):
        ff, fd = _compute_freq_diss(
            ts_like, tone_index, scaled[block].real, scaled[block].imag,
            sweep_data, reference_tone_frequency=reference_tone_frequency,
            smooth_window_hz=smooth_window_hz,
            conversion_method=conversion_method, calibrations=calibrations)
        top.append(ff)
        bot.append(fd)
    return np.asarray(top), np.asarray(bot)


# ---------------------------------------------------------------------------
# On-minus-off detector noise
# ---------------------------------------------------------------------------

# Panels already divided by their own carrier, so that subtracting two
# captures with different carrier powers leaves a readout-noise residual.
# Off-resonance noise is additive readout noise -- amplifier or ADC, both
# additive in the digitised I/Q: it puts the same power into
# 'i', 'q' and 'magnitude' whatever the carrier, so those subtract cleanly,
# while 'phase' carries it as S_n / A**2 and does not.
_CARRIER_SCALED_PANELS = ('phase',)

# Carrier ratio above which the phase-PSD mismatch is worth flagging. A 5%
# mismatch leaves 5% of the readout floor behind, below the sqrt(2/n) per-bin
# scatter of a typical Welch estimate but not below a band average of it.
_CARRIER_RATIO_TOLERANCE = 1.05


def _subtract_operand(data, f, cleaned, name):
    """One side of `subtract_psds`: a result dict, or PSD rows plus ``f``.

    Returns ``(f, rows, panels, tones, meta)``. ``meta`` carries the
    `timestream_psds` metadata the difference has to echo, filled with
    ``None`` for plain-array input, so that nothing after this call has to
    know whether the operand was a dict.
    """
    keys = ('format', 'sample_rate', 'carrier', 'conversion_method',
            'cleaning', 'psd_kwargs', 'info', 'reference_tone_frequency')
    if isinstance(data, Mapping):
        meta = {k: data.get(k) for k in keys}
        meta['has_cleaned'] = data.get('psd_cleaned') is not None
        return _as_psd_rows(data, cleaned=cleaned) + (meta,)
    if f is None:
        raise ValueError(
            f"{name} is an array, so the shared frequency axis must be given "
            "as f=")
    meta = dict.fromkeys(keys)
    meta['has_cleaned'] = False
    return _as_psd_rows(f, data) + (meta,)


def _same_option(a, b):
    """Equality for a psd_kwargs / cleaning value, which may be an array."""
    if a is None or b is None:
        return a is b
    try:
        return bool(np.array_equal(a, b))
    except (TypeError, ValueError):
        return a is b


def _check_carrier_scaling(meta_on, meta_off, panels):
    """Warn when carrier-relative rows are subtracted across unequal carriers."""
    scaled = [p for p in panels if p in _CARRIER_SCALED_PANELS]
    if not scaled:
        return
    c_on, c_off = meta_on['carrier'], meta_off['carrier']
    if c_on is None or c_off is None:
        return
    c_on = np.asarray(c_on, dtype=float)
    c_off = np.asarray(c_off, dtype=float)
    if c_on.shape != c_off.shape:
        return
    with np.errstate(divide='ignore', invalid='ignore'):
        ratio = c_on / c_off
    ratio = ratio[np.isfinite(ratio) & (ratio > 0)]
    if ratio.size == 0:
        return
    worst = float(max(np.max(ratio), 1.0 / np.min(ratio)))
    if worst > _CARRIER_RATIO_TOLERANCE:
        warnings.warn(
            f"panels {scaled} are already divided by their own carrier, and "
            f"the two captures' carrier powers differ by up to {worst:.3g}x, "
            "so the same readout noise lands at different levels in the two "
            "and does not subtract out. On resonance the carrier sits in the "
            "dip, so this is expected for any real on/off pair. Use "
            "format='freq_diss', which refers both to one responsivity.",
            stacklevel=3)


def _check_referred_frequencies(meta_on, meta_off):
    """Warn when the two captures were referred to different responsivities."""
    r_on = meta_on['reference_tone_frequency']
    r_off = meta_off['reference_tone_frequency']
    if r_on is None or r_off is None:
        return
    r_on = np.asarray(r_on, dtype=float)
    r_off = np.asarray(r_off, dtype=float)
    if r_on.shape != r_off.shape:
        return
    both = np.isfinite(r_on) & np.isfinite(r_off)
    if not np.any(both) or np.allclose(r_on[both], r_off[both]):
        return
    worst = float(np.max(np.abs(r_on - r_off)[both]))
    warnings.warn(
        "the two captures' freq/diss rows were referred to different tone "
        f"frequencies (by up to {worst:.0f} Hz), so they do not share a "
        "responsivity and their difference is not a detector-noise estimate. "
        "Pass the on-resonance frequencies as reference_tone_frequency to "
        "both timestream_psds calls; left to default, the off capture "
        "converts at its parked position where |dS21/df| is small and its "
        "referred noise is inflated by orders of magnitude.",
        stacklevel=3)


def subtract_psds(on, off, f=None, *, clip='positive', export=None):
    """
    Difference of two PSDs, e.g. on-resonance minus an off-resonance floor.

    The off-resonance capture measures the readout noise alone (amplifier, ADC
    or both),
    so subtracting it from the on-resonance PSD leaves the detector's own
    contribution. The result has the same shape as a `timestream_psds` result,
    so `psd_band_levels` and `fit_psd_model` apply to it directly.

    Parameters
    ----------
    on, off : dict or array_like
        Two `timestream_psds` results, or two PSD arrays of shape
        ``(n_tones, n_freq)`` or ``(n_freq,)`` sharing the axis ``f``.
    f : array_like, optional
        Frequency axis in Hz. Required when ``on`` and ``off`` are arrays,
        ignored when they are result dicts.
    clip : {'positive', None}, optional
        ``'positive'`` (default) replaces non-positive bins with ``NaN``, which
        is what a logarithmic axis needs. ``None`` keeps them: a band that
        comes out negative is the honest statement that the detector noise
        there is below the readout floor, not that it is zero. Either way
        ``'positive_fraction'`` records how much of each row survived.

        **Use ``clip=None`` to quote a number.** Averaging a clipped difference
        keeps only its upward fluctuations, so a difference that is really zero
        comes back at about ``0.8 * S_floor * sqrt(2 / n_averages)`` -- 24% of
        the readout floor over 22 Welch averages. `psd_band_levels` and
        `fit_psd_model` warn if they are handed a clipped result.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'f'`` : ndarray, shape (n_freq,)
            Frequency axis in Hz.
        ``'psd'`` : dict of ndarray
            One ``(n_tones, n_freq)`` array of ``on - off`` per panel name, in
            the units of the inputs: fractional squared per Hz for
            ``format='freq_diss'``, raw counts squared per Hz for ``'iq'``.
        ``'psd_cleaned'`` : dict of ndarray or None
            The same difference of the cleaned rows, when *both* inputs carry
            them; ``None`` otherwise.
        ``'positive_fraction'`` : dict of ndarray, shape (n_tones,) per panel
            The fraction of each tone's finite bins above zero before clipping,
            over ``f > 0``. Near 1 means the detector noise is resolved above
            the floor; near 0.5 means the two captures agree to within their
            scatter and only a limit can be quoted. See Notes -- on a linear
            frequency axis this is weighted towards the top decade.
        ``'n_points'`` : dict of ndarray, shape (n_tones,) per panel
            The number of finite bins behind each fraction, so a row with three
            usable bins is not read like one with five hundred.
        ``'clip'`` : str or None
            The ``clip`` argument, echoed. `psd_band_levels` and
            `fit_psd_model` read this to warn about the clipping bias.
        ``'reference_tone_frequency'`` : ndarray or None
            The per-tone frequency both captures' freq/diss rows were referred
            to, carried from ``on``.
        ``'panels'``, ``'tones'``, ``'format'``, ``'sample_rate'``,
        ``'carrier'``, ``'conversion_method'``, ``'cleaning'``,
        ``'psd_kwargs'``, ``'info'``
            Carried through from ``on`` so the result reads like a
            `timestream_psds` result. ``'carrier'`` is the on-resonance one.
            ``None`` (and ``NaN`` for ``'sample_rate'``) for array input.

    Raises
    ------
    ValueError
        If the two inputs disagree on their frequency axis, panels, tones or
        freq/diss conversion method; if ``clip`` is not ``'positive'`` or
        ``None``; if either input is an array and ``f`` was not given; or if a
        dict input is not a `timestream_psds` result.

    Warns
    -----
    UserWarning
        If the two inputs were computed with different ``psd_kwargs`` or
        different cleaning; if only one of them carries cleaned rows; if their
        freq/diss rows were referred to different tone frequencies; or if a
        ``'phase'`` row is subtracted across captures whose carrier powers
        differ by more than 5%.

    See Also
    --------
    timestream_psds : produces the two inputs.
    psd_band_levels : quote the difference as a number in a band.

    Notes
    -----
    Both captures must be referred to the *same* responsivity: pass the
    on-resonance tone frequencies as ``reference_tone_frequency`` to both
    `timestream_psds` calls. Left to default, the off capture converts at its
    own parked frequency, where ``|dS21/df|`` is small, so the conversion
    divides by a near-zero gradient and the off-resonance noise comes out
    orders of magnitude too high. An on/off ratio far below 1 means this was
    missed, not that the detector is quiet.

    The off-resonance capture is dominated by *additive* readout noise -- whether
    it originates in the amplifier chain or at the ADC, it is additive in the
    digitised I/Q -- which
    puts the same power into ``'i'``, ``'q'`` and ``'magnitude'`` whatever the
    carrier amplitude, so those rows subtract cleanly across the dip. ``'phase'``
    is the exception: it carries that noise as ``S_n / A**2``, so on resonance,
    where the carrier sits in the dip, the same readout noise appears larger
    and does not cancel. Subtracting ``'phase'`` rows across an on/off pair
    therefore reports most of the readout noise as detector noise, and warns.
    ``format='freq_diss'`` is the safe choice: both rows are referred to one
    responsivity.

    ``'positive_fraction'`` is counted over every bin of a linear Welch axis,
    where most bins lie in the top decade. A tone fully resolved below 10 Hz
    but at the floor above it reports about 0.65, not 1 -- read it as a
    statement about the whole axis, and use `psd_band_levels` with ``clip=None``
    for a per-band answer, where a negative level (and hence a ``NaN`` ASD) is
    the honest "below the readout floor" result.

    ``conversion_method='circle'`` is passed through rather than rejected: a
    difference of two ``Delta(rho)`` proxies is self-consistent. It is still not
    ``Im(delta_tilde)``, and the returned dict is otherwise indistinguishable
    from a real dissipation difference, so check ``'conversion_method'`` before
    quoting one as the other.

    Examples
    --------
    >>> import numpy as np
    >>> f = np.linspace(0, 500, 501)
    >>> off = np.full((1, f.size), 1e-18)          # readout floor
    >>> on = off + 4e-18 / np.maximum(f, 1.0)      # plus detector 1/f
    >>> d = subtract_psds(on, off, f=f)
    >>> bool(np.isclose(d['psd']['psd'][0, 10], 4e-19))
    True
    >>> float(d['positive_fraction']['psd'][0])    # resolved at every bin
    1.0

    A tone sitting at the readout floor comes back roughly half positive,
    which is the answer -- its detector noise is below the floor:

    >>> at_floor = off * (1 + 0.02 * np.cos(f))
    >>> lim = subtract_psds(at_floor, off, f=f)
    >>> 0.4 < float(lim['positive_fraction']['psd'][0]) < 0.6
    True
    """
    if clip not in ('positive', None):
        raise ValueError(f"clip must be 'positive' or None, got {clip!r}")

    f_on, on_rows, panels, tones, meta_on = _subtract_operand(
        on, f, False, 'on')
    f_off, off_rows, off_panels, off_tones, meta_off = _subtract_operand(
        off, f, False, 'off')

    if panels != off_panels:
        raise ValueError(
            f"on and off hold different panels ({panels} vs {off_panels}); "
            "compute both with the same format=")
    if f_on.shape != f_off.shape or not np.allclose(
            f_on, f_off, equal_nan=True):
        raise ValueError(
            f"on and off are on different frequency axes ({f_on.size} vs "
            f"{f_off.size} points); compute both with the same psd_kwargs and "
            "the same record length")
    if not np.array_equal(tones, off_tones):
        raise ValueError(
            f"on and off cover different tones ({tones} vs {off_tones}); "
            "select the same tones= in both calls")

    m_on, m_off = meta_on['conversion_method'], meta_off['conversion_method']
    if m_on != m_off:
        raise ValueError(
            f"on and off used different freq/diss conversions ({m_on!r} vs "
            f"{m_off!r}); their dissipation rows are not the same quantity")
    for label, key in (('psd_kwargs', 'psd_kwargs'), ('cleaning', 'cleaning')):
        a = meta_on[key] or {}
        b = meta_off[key] or {}
        # Values can be arrays (an explicit window, a tone list), so compare
        # key by key under np.array_equal rather than with dict ==.
        differing = sorted(k for k in set(a) | set(b)
                           if not _same_option(a.get(k), b.get(k)))
        if differing:
            warnings.warn(
                f"on and off were computed with different {label} "
                f"({differing}); the difference mixes two estimators and the "
                "result reports on's",
                stacklevel=2)
    _check_carrier_scaling(meta_on, meta_off, panels)
    _check_referred_frequencies(meta_on, meta_off)

    diff = {p: on_rows[p] - off_rows[p] for p in panels}

    positive = f_on > 0
    fraction, n_points = {}, {}
    for p in panels:
        valid = np.isfinite(diff[p]) & positive
        n_valid = valid.sum(axis=1)
        n_pos = (valid & (diff[p] > 0)).sum(axis=1)
        n_points[p] = n_valid
        with np.errstate(invalid='ignore'):
            fraction[p] = np.where(n_valid > 0, n_pos / n_valid, np.nan)

    if clip == 'positive':
        for p in panels:
            diff[p][~(diff[p] > 0)] = np.nan

    cleaned_diff = None
    if meta_on['has_cleaned'] and meta_off['has_cleaned']:
        _, on_clean, _, _, _ = _subtract_operand(on, f, True, 'on')
        _, off_clean, _, _, _ = _subtract_operand(off, f, True, 'off')
        cleaned_diff = {p: on_clean[p] - off_clean[p] for p in panels}
        if clip == 'positive':
            for p in panels:
                cleaned_diff[p][~(cleaned_diff[p] > 0)] = np.nan
    elif meta_on['has_cleaned'] != meta_off['has_cleaned']:
        warnings.warn(
            "only one of on/off was cleaned, so no cleaned difference is "
            "returned; clean both or neither",
            stacklevel=2)

    rate = meta_on['sample_rate']
    return _maybe_export({
        'format': meta_on['format'],
        'panels': panels,
        'f': f_on,
        'sample_rate': np.nan if rate is None else float(rate),
        'tones': tones,
        'psd': diff,
        'psd_cleaned': cleaned_diff,
        'positive_fraction': fraction,
        'n_points': n_points,
        'clip': clip,
        'reference_tone_frequency': meta_on['reference_tone_frequency'],
        'carrier': meta_on['carrier'],
        'conversion_method': meta_on['conversion_method'],
        'cleaning': meta_on['cleaning'],
        'psd_kwargs': meta_on['psd_kwargs'],
        'info': meta_on['info'],
    }, export)


# ---------------------------------------------------------------------------
# Composite wideband PSD
# ---------------------------------------------------------------------------

def _boxcar_power_response(f, acc_len, fs_fast):
    """``|H(f)|**2`` of an ``acc_len``-sample boxcar average, peak-normalised."""
    x = np.pi * np.asarray(f, dtype=float) / fs_fast
    den = np.sin(x)
    tiny = np.abs(den) < 1e-12
    safe = np.where(tiny, 1.0, den)
    out = np.where(tiny, float(acc_len) ** 2, (np.sin(acc_len * x) / safe) ** 2)
    return out / float(acc_len) ** 2


def accumulator_response(f, acc_len, fs_fast, spectrum=None, n_alias=None):
    """
    Forward model of what the accumulator does to a spectrum. Never invert it.

    Averaging ``acc_len`` samples and decimating by exactly ``acc_len`` builds
    consecutive outputs from *disjoint* inputs, so for white input the outputs
    are uncorrelated and the measured PSD is exactly flat: the boxcar's
    ``sinc**2`` attenuation and the aliasing from the decimation cancel. The
    familiar -3.9 dB at Nyquist is the *pre-decimation* filter response, which
    is not what anyone measures. Dividing a measured slow trace by it would
    manufacture a spurious rise towards Nyquist.

    The cancellation is exact only for white input. Where the true spectrum
    falls with frequency there is less power in the alias zones than the boxcar
    removes in band, so the compensation under-fills and a real droop appears --
    exactly the case near a GR roll-off. This function returns that droop, so a
    model fitted on the fast trace can be pushed *through* the accumulator and
    compared with the slow trace.

    Parameters
    ----------
    f : array_like
        Frequencies in Hz, within the decimated band ``|f| <= fs_fast/(2*acc_len)``.
    acc_len : int
        Accumulation length, the boxcar width and the decimation factor.
    fs_fast : float
        Pre-accumulation sample rate in Hz.
    spectrum : callable, optional
        The assumed true spectrum ``S(f)``, called with frequencies in Hz up to
        ``fs_fast/2`` -- it must therefore be a function, not an array over the
        slow band, since the aliased power comes from outside that band.
        ``None`` (default) means white, for which the response is 1 everywhere.
    n_alias : int, optional
        Number of alias zones summed on each side. ``None`` (default) covers
        the full band to ``fs_fast/2``.

    Returns
    -------
    ndarray
        The factor by which the measured slow PSD sits below the true spectrum
        at each ``f``. ``1.0`` means unaffected; ``0.5`` means the measurement
        reads half the truth there.

    Raises
    ------
    ValueError
        If any ``f`` lies outside the decimated band
        ``|f| <= fs_fast / (2 * acc_len)``.
    TypeError
        If ``spectrum`` is given and is not callable.

    See Also
    --------
    stitch_psds : flags the top of the slow band rather than correcting it.

    Notes
    -----
    **Forward-model, never invert.** Inverting an aliased response is ill-posed,
    because the fold depends on the very spectrum being solved for. Fit the
    model on the fast trace, push it through this function, and compare with the
    measured slow trace; do not divide the slow trace by the result.

    The measured droop is bounded by how fast the true spectrum falls. Verified
    against direct simulation: for a 100 us GR roll-off at ``acc_len=1000`` and
    ``fs_fast=600 kHz`` the top of the slow band reads 0.91x, and the model
    matches the simulation to a median ratio of 1.000.

    Examples
    --------
    White input is unaffected, right up to Nyquist:

    >>> import numpy as np
    >>> f = np.array([1.0, 100.0, 290.0])
    >>> np.round(accumulator_response(f, 1000, 600e3), 6)
    array([1., 1., 1.])

    A spectrum already falling as ``f**-2`` is genuinely suppressed near the
    top of the band, because there is less power to fold back:

    >>> falling = lambda x: 1.0 / (1.0 + (x / 50.0) ** 2)
    >>> r = accumulator_response(f, 1000, 600e3, spectrum=falling)
    >>> bool(r[0] > 0.99), bool(r[-1] < 0.9)
    (True, True)
    """
    f = np.asarray(f, dtype=float)
    acc_len = int(acc_len)
    fs_slow = fs_fast / acc_len
    if np.any(np.abs(f) > fs_slow / 2 * (1 + 1e-9)):
        raise ValueError(
            f"f must lie in the decimated band |f| <= {fs_slow / 2:g} Hz; got "
            f"up to {np.max(np.abs(f)):g} Hz")
    if n_alias is None:
        n_alias = acc_len // 2
    a = np.arange(-n_alias, n_alias + 1)[:, None]
    f_alias = f[None, :] + a * fs_slow
    inside = np.abs(f_alias) <= fs_fast / 2

    weight = _boxcar_power_response(f_alias, acc_len, fs_fast) * inside
    if spectrum is None:
        # Flat input: the fold exactly refills what the boxcar took out.
        return np.ones_like(f)
    if not callable(spectrum):
        raise TypeError(
            "spectrum must be a callable S(f); the aliased power comes from "
            "outside the decimated band, so an array over that band cannot "
            "supply it")
    s_alias = np.asarray(spectrum(np.abs(f_alias)), dtype=float)
    with np.errstate(divide='ignore', invalid='ignore'):
        folded = np.sum(weight * s_alias, axis=0) / np.sum(weight, axis=0)
        return folded / np.asarray(spectrum(np.abs(f)), dtype=float)


def _trace_band(f):
    """Positive-frequency span actually covered by a trace."""
    pos = np.asarray(f, dtype=float)
    pos = pos[np.isfinite(pos) & (pos > 0)]
    if pos.size == 0:
        return (np.nan, np.nan)
    return (float(pos.min()), float(pos.max()))


def stitch_psds(traces, overlap='check', allow_cleaning_mismatch=False,
                panels=None, export=None):
    """
    Join accumulated and snapshot PSDs into one wideband trace per tone.

    The accumulated stream covers ``1/T`` up to ``fs_slow/2``; the snapshots
    cover ``fs_fast/len_snapshot`` up to ``fs_fast/2``. At the documented lab
    settings those are 300 Hz and 586 Hz, so there is a **gap**, not an
    overlap. This never interpolates across it -- the gap is returned so a plot
    can shade it.

    To close the gap and gain a real overlap for cross-validation, take the
    accumulated stream at ``acc_len < 512`` (``fs_fast/(2*acc_len) >
    fs_fast/1024``). That is an acquisition choice, not something analysis can
    repair.

    Parameters
    ----------
    traces : sequence of dict
        Two or more `timestream_psds` / `snapshot_psds` results for the same
        tones and the same quantity. Order does not matter; they are sorted by
        their frequency coverage.
    overlap : {'check', 'scale', 'none'}, optional
        What to do where two traces overlap. ``'check'`` (default) reports the
        ratio and changes nothing -- the discrepancy is the diagnostic, and a
        scale factor would hide it. ``'scale'`` multiplies each later trace
        onto the first and records the factor, warning past 1 dB. ``'none'``
        skips the comparison.
    allow_cleaning_mismatch : bool, optional
        Permit stitching a cleaned trace to an uncleaned one. Default
        ``False``. The accumulated stream is simultaneous across tones and so
        can be SVD- or blind-cleaned; snapshots are taken one tone at a time
        and cannot be. Cleaning one side only puts an artificial step at the
        join, in the same band as the alias-affected slow bins.
    panels : sequence of str, optional
        Panels to stitch. ``None`` (default) uses those common to every trace.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'f'`` : ndarray
            Ascending frequency axis of the concatenated traces.
        ``'psd'`` : dict of ndarray
            ``(n_tones, n_freq)`` per panel, the traces laid end to end. Bins
            in a gap are absent, not interpolated.
        ``'source'`` : ndarray of int
            Index into ``traces`` for each bin of ``'f'``, so any part of the
            composite can be traced to where it came from.
        ``'gaps'`` : list of tuple
            ``(lo_hz, hi_hz)`` bands no trace covers.
        ``'overlaps'`` : list of dict
            Per overlapping pair: ``'traces'``, ``'band'``, ``'ratio'`` per
            panel (median over the shared bins, later over earlier), and
            ``'scale'`` if one was applied.
        ``'alias_affected'`` : tuple or None
            The top decade of the accumulated band, where a falling spectrum
            reads low (see `accumulator_response`). ``None`` if no accumulated
            trace was given.
        ``'bands'`` : list of tuple
            Each trace's covered span, in the sorted order used.
        ``'n_avg'``, ``'effective_dof'`` : ndarray
            Per bin, carried from the contributing trace, since Welch segments
            and snapshot blocks give different scatter.
        ``'tones'``, ``'panels'``, ``'format'``, ``'conversion_method'``
            Carried through after the compatibility checks.

    Raises
    ------
    ValueError
        If fewer than two traces are given, or they disagree on tones, format,
        panels, freq/diss conversion, referred frequency, or (unless
        ``allow_cleaning_mismatch``) cleaning provenance.

    Warns
    -----
    UserWarning
        If ``overlap='scale'`` applies a factor larger than 1 dB. A disagreement
        that big is usually a real provenance difference, not a calibration
        offset, and scaling hides it.

    See Also
    --------
    accumulator_response : why the top of the slow band is flagged, not fixed.
    fit_gr_rolloff : fit ``tau_qp`` to the result.

    Notes
    -----
    **Take the accumulated stream at** ``acc_len <= 256`` **if the composite is
    to be validated rather than merely joined.** The traces overlap only when
    ``fs_fast / (2 * acc_len) > fs_fast / len_snapshot``, i.e. ``acc_len < 512``.
    At the documented ``acc_len = 1000`` there is a 300-586 Hz gap; at
    ``acc_len = 32768`` it runs 9.2 Hz to 586 Hz, six octaves.

    The board carries a second accumulator that could in principle run at a
    different ``acc_len`` and give a third trace, but reading it is too
    expensive to be worth it in practice and it is not used. It would not have
    helped anyway: two accumulated traces overlap each *other* at the low end,
    not the snapshots, so the 300-586 Hz gap is untouched either way. Lowering
    ``acc_len`` is the only thing that closes it.

    ``'alias_affected'`` marks the top decade of the accumulated band. The
    accumulator does not droop for white input, but where the true spectrum is
    already falling it reads low there -- see `accumulator_response`. The band
    is flagged rather than corrected, since inverting the fold is ill-posed.

    Traces are laid end to end, each contributing only the bins above where the
    previous one stopped, so an overlap is never double-counted. ``'n_avg'`` and
    ``'effective_dof'`` are carried per bin because Welch segments and snapshot
    blocks give different scatter, and error bars must not imply otherwise.

    Examples
    --------
    >>> import numpy as np
    >>> slow = {'f': np.array([1.0, 10.0, 100.0]), 'panels': ('psd',),
    ...         'psd': {'psd': np.array([[3.0, 2.0, 1.0]])},
    ...         'tones': np.array([0]), 'format': None}
    >>> fast = {'f': np.array([600.0, 6000.0]), 'panels': ('psd',),
    ...         'psd': {'psd': np.array([[1.0, 1.0]])},
    ...         'tones': np.array([0]), 'format': None}
    >>> out = stitch_psds([slow, fast])
    >>> [float(x) for x in out['f']]
    [1.0, 10.0, 100.0, 600.0, 6000.0]
    >>> out['gaps']
    [(100.0, 600.0)]
    >>> [int(x) for x in out['source']]      # which trace each bin came from
    [0, 0, 0, 1, 1]
    """
    traces = list(traces)
    if len(traces) < 2:
        raise ValueError(
            f"stitch_psds needs at least two traces, got {len(traces)}")
    if overlap not in ('check', 'scale', 'none'):
        raise ValueError(
            f"overlap must be 'check', 'scale' or 'none', got {overlap!r}")

    order = sorted(range(len(traces)),
                   key=lambda i: _trace_band(traces[i]['f'])[0])
    traces = [traces[i] for i in order]

    ref = traces[0]
    common = None
    for t in traces:
        p = tuple(t.get('panels') or tuple(t['psd']))
        common = p if common is None else tuple(x for x in common if x in p)
    use = tuple(panels) if panels is not None else common
    if not use:
        raise ValueError(
            "the traces share no panel; stitch traces computed with the same "
            "format=")

    for t in traces[1:]:
        _check_stitch_compatible(ref, t, allow_cleaning_mismatch)

    bands = [_trace_band(t['f']) for t in traces]
    overlaps = []
    scales = [1.0] * len(traces)
    for i in range(len(traces) - 1):
        lo, hi = bands[i + 1][0], bands[i][1]
        if not (np.isfinite(lo) and np.isfinite(hi)) or lo > hi:
            continue
        record = {'traces': (order[i], order[i + 1]), 'band': (lo, hi),
                  'ratio': {}}
        if overlap != 'none':
            for panel in use:
                record['ratio'][panel] = _overlap_ratio(
                    traces[i], traces[i + 1], panel, lo, hi)
            if overlap == 'scale':
                finite = [r for r in record['ratio'].values() if np.isfinite(r)]
                factor = float(np.median(finite)) if finite else 1.0
                scales[i + 1] = scales[i] / factor if factor > 0 else scales[i]
                record['scale'] = scales[i + 1]
                if factor > 0 and abs(10 * np.log10(factor)) > 1.0:
                    warnings.warn(
                        f"traces {record['traces']} disagree by "
                        f"{10 * np.log10(factor):+.2f} dB over "
                        f"{lo:g}-{hi:g} Hz; overlap='scale' has hidden that "
                        "behind a scale factor. The discrepancy is usually a "
                        "real provenance difference, not a calibration offset",
                        stacklevel=2)
        overlaps.append(record)

    gaps = []
    for i in range(len(traces) - 1):
        if np.isfinite(bands[i][1]) and np.isfinite(bands[i + 1][0]) \
                and bands[i + 1][0] > bands[i][1]:
            gaps.append((bands[i][1], bands[i + 1][0]))

    # Lay the traces end to end. In an overlap the *higher* trace wins: the
    # top of an accumulated band is where a falling spectrum reads low to the
    # accumulator's alias fold, while the snapshot bins covering the same
    # frequencies are unaffected. Preferring the lower trace would keep the
    # biased bins and throw away the clean ones.
    f_parts, src_parts, avg_parts, dof_parts = [], [], [], []
    row_parts = {p: [] for p in use}
    for i, t in enumerate(traces):
        f = np.asarray(t['f'], dtype=float)
        keep = np.isfinite(f) & (f > 0)
        nxt = next((bands[j][0] for j in range(i + 1, len(traces))
                    if np.isfinite(bands[j][0])), None)
        if nxt is not None:
            keep &= f < nxt
        if not np.any(keep):
            continue
        n_keep = int(keep.sum())
        f_parts.append(f[keep])
        src_parts.append(np.full(n_keep, order[i], dtype=int))
        avg_parts.append(np.full(n_keep, float(t.get('n_avg', np.nan))))
        dof_parts.append(np.full(n_keep, float(t.get('effective_dof', np.nan))))
        for p in use:
            row_parts[p].append(
                np.atleast_2d(np.asarray(t['psd'][p], dtype=float))[:, keep]
                * scales[i])

    # Flag the top decade of the *highest* accumulated trace: that is the band
    # adjacent to the join, and the one a GR corner is most likely to sit in.
    alias = None
    for t in traces:
        if t.get('source') != 'snapshots':
            hi = _trace_band(t['f'])[1]
            if np.isfinite(hi):
                alias = (hi / 10.0, hi)

    return _maybe_export({
        'f': np.concatenate(f_parts) if f_parts else np.empty(0),
        'psd': {p: (np.concatenate(row_parts[p], axis=1) if row_parts[p]
                    else np.empty((0, 0))) for p in use},
        'source': (np.concatenate(src_parts) if src_parts
                   else np.empty(0, dtype=int)),
        'gaps': gaps,
        'overlaps': overlaps,
        'alias_affected': alias,
        'bands': bands,
        'n_avg': np.concatenate(avg_parts) if avg_parts else np.empty(0),
        'effective_dof': (np.concatenate(dof_parts) if dof_parts
                          else np.empty(0)),
        'tones': np.asarray(ref.get('tones', []), dtype=int),
        'panels': use,
        'format': ref.get('format'),
        'conversion_method': ref.get('conversion_method'),
    }, export)


def _check_stitch_compatible(a, b, allow_cleaning_mismatch):
    """Everything that would put a step at the join if it differed."""
    if not np.array_equal(np.asarray(a.get('tones', []), dtype=int),
                          np.asarray(b.get('tones', []), dtype=int)):
        raise ValueError(
            f"traces cover different tones ({a.get('tones')} vs "
            f"{b.get('tones')}); stitch one tone set at a time")
    for key, label in (('format', 'format'),
                       ('conversion_method', 'freq/diss conversion')):
        if a.get(key) != b.get(key):
            raise ValueError(
                f"traces used a different {label} ({a.get(key)!r} vs "
                f"{b.get(key)!r}); their rows are not the same quantity")
    # Snapshots are pre-accumulation and the accumulator sums, so raw-unit rows
    # sit acc_len**2 apart -- 60 dB at acc_len=1000, 90 dB at 32768. Only the
    # freq/diss conversion (which scales by acc_len) puts the two on one scale.
    if a.get('format') in ('iq', 'magphase') \
            and (a.get('source') == 'snapshots') != (b.get('source') == 'snapshots'):
        raise ValueError(
            f"cannot stitch format={a.get('format')!r} across the accumulator: "
            "snapshot rows are pre-accumulation and accumulated rows are sums "
            "over acc_len samples, so their raw units differ by acc_len**2 "
            "(60 dB at acc_len=1000) and the join is a step. Use "
            "format='freq_diss', which scales the snapshots by acc_len, or "
            "compare the two in dBc")
    r_a, r_b = a.get('reference_tone_frequency'), b.get('reference_tone_frequency')
    if r_a is not None and r_b is not None:
        r_a, r_b = np.asarray(r_a, dtype=float), np.asarray(r_b, dtype=float)
        both = (r_a.shape == r_b.shape) and np.isfinite(r_a) & np.isfinite(r_b)
        if np.any(both) and not np.allclose(r_a[both], r_b[both]):
            raise ValueError(
                "traces were referred to different tone frequencies, so their "
                "responsivities differ and the join is a step; pass the same "
                "reference_tone_frequency to both")
    if not allow_cleaning_mismatch:
        clean_a = _cleaning_active(a)
        clean_b = _cleaning_active(b)
        if clean_a != clean_b:
            raise ValueError(
                "one trace is cleaned and the other is not "
                f"({clean_a} vs {clean_b}), which puts an artificial step at "
                "the join. Snapshots are taken one tone at a time and cannot "
                "be cleaned across tones, so the composite default is "
                "uncleaned throughout. Pass allow_cleaning_mismatch=True to "
                "override")


def _cleaning_active(trace):
    """Whether any common-mode cleaning was applied to a trace."""
    cleaning = trace.get('cleaning') or {}
    return bool(cleaning.get('decorrelate_modes')
                or cleaning.get('blind_tone_modes'))


def _overlap_ratio(earlier, later, panel, lo, hi):
    """Median ratio later/earlier over the band both traces cover."""
    out = []
    for trace in (earlier, later):
        f = np.asarray(trace['f'], dtype=float)
        sel = (f >= lo) & (f <= hi)
        if not np.any(sel):
            return np.nan
        out.append((f[sel],
                    np.atleast_2d(np.asarray(trace['psd'][panel],
                                             dtype=float))[:, sel]))
    (f_e, p_e), (f_l, p_l) = out
    # Different bin spacings either side of the join, so put the later trace
    # onto the earlier one's grid before comparing.
    #
    # Compare the *means* over the band, not the median of the per-bin ratios.
    # A chi2_d bin's median sits below its mean by a factor that depends on d,
    # and the two traces have different n_avg by construction, so a median of
    # ratios reports a level difference that is not there -- -1.0 dB for a
    # single-segment trace against a well-averaged one, enough to trip the
    # 1 dB warning on traces that agree exactly.
    ratios = []
    for row in range(min(p_e.shape[0], p_l.shape[0])):
        interp = np.interp(f_e, f_l, p_l[row])
        ok = np.isfinite(interp) & np.isfinite(p_e[row]) & (p_e[row] > 0)
        if np.any(ok) and np.mean(p_e[row][ok]) > 0:
            ratios.append(np.mean(interp[ok]) / np.mean(p_e[row][ok]))
    return float(np.median(ratios)) if ratios else np.nan


def _log_chi2_bias(n_averages, trace_n_avg, n_freq):
    """Per-bin ``E[log10 S_hat] - log10 S`` offset, or 0.0 where unknown.

    Accepts a scalar (one Welch configuration) or a per-bin array, which is
    what `stitch_psds` carries -- the bands either side of a join have
    different ``n_avg``, so one scalar cannot describe both.
    """
    n = n_averages if n_averages is not None else trace_n_avg
    if n is None:
        return 0.0
    n = np.asarray(n, dtype=float)
    if n.ndim and n.size not in (1, n_freq):
        return 0.0
    with np.errstate(invalid='ignore', divide='ignore'):
        bias = (np.log(n) - scipy_special.digamma(n)) / np.log(10.0)
    return np.where(np.isfinite(bias) & (n >= 1), bias, 0.0)


def _gr_operand(data, psd):
    """`fit_gr_rolloff`'s input: rows plus the provenance the fit needs.

    Returns ``(f, rows, panels, tones, meta)``, where ``meta`` carries
    ``'effective_dof'`` and ``'gaps'`` from a `stitch_psds` or `snapshot_psds`
    result and neutral defaults for plain arrays, so nothing after this call
    has to know whether the input was a dict.
    """
    meta = {'oversampling': 1.0, 'gaps': [], 'n_avg': None,
            'effective_dof': None, 'source': None}
    if isinstance(data, Mapping):
        over = data.get('bin_oversampling')
        if over is not None:
            meta['oversampling'] = max(1.0, float(over))
        meta['gaps'] = list(data.get('gaps') or [])
        meta['n_avg'] = data.get('n_avg')
        dof = data.get('effective_dof')
        if isinstance(dof, np.ndarray):
            finite = dof[np.isfinite(dof)]
            dof = float(np.min(finite)) if finite.size else None
        meta['effective_dof'] = None if dof is None else float(dof)

        # A cross_psd result puts the *auto*-spectra under 'psd'; the
        # cross-spectrum, which is the whole reason to use one here, lives
        # under 'cross'. Reading 'psd' would quietly fit the readout floor
        # that the cross-spectrum exists to remove.
        if 'cross' in data and 'coherence' in data:
            panels = tuple(data.get('panels') or ())
            rows = np.atleast_2d(np.real(np.asarray(data['cross'])))
            meta['source'] = 'cross'
            f = np.asarray(data['f'], dtype=float)
            tones = np.asarray(data.get('tones', np.arange(rows.shape[0])),
                               dtype=int)
            name = 'x'.join(panels) if panels else 'cross'
            return f, {name: rows}, (name,), tones, meta
    return _as_psd_rows(data, psd) + (meta,)


def _resolve_tau_res(tau_res, fr, Qr, calibrations=None):
    """Ring-down time ``Q_r / (pi f_r)``, and where it came from.

    Scalars stay scalars; array-like ``tau_res``, or array-like ``fr``/``Qr``,
    give one value per tone. Real resonators differ enough that this matters:
    across three tones of one capture ``tau_res`` ran 7.9 to 44.7 us.
    """
    if tau_res is not None:
        arr = np.asarray(tau_res, dtype=float)
        return (float(arr) if arr.ndim == 0 else arr), 'given'
    if fr is not None and Qr is not None:
        fr_a = np.asarray(fr, dtype=float)
        qr_a = np.asarray(Qr, dtype=float)
        out = qr_a / (np.pi * fr_a)
        return (float(out) if out.ndim == 0 else out,
                'derived from fr and Qr')
    if calibrations is not None:
        cal = calibrations
        fr_c = getattr(cal, 'fr', None)
        qr_c = getattr(cal, 'Qr', None)
        if fr_c is not None and qr_c is not None:
            return float(qr_c) / (np.pi * float(fr_c)), 'derived from the fit'
    raise ValueError(
        "tau_res is required: pass it directly, or pass fr= and Qr= from the "
        "calibration fit. The resonator ring-down and the quasiparticle "
        "lifetime sit in the same decade, so fitting a single Lorentzian "
        "returns a blend of the two and biases tau_qp")


def _gr_model(f, s0, tau_qp, white, tau_res):
    """GR Lorentzian through the resonator pole, plus a white floor."""
    wq = (2.0 * np.pi * f * tau_qp) ** 2
    wr = (2.0 * np.pi * f * tau_res) ** 2
    return s0 / ((1.0 + wq) * (1.0 + wr)) + white


def fit_gr_rolloff(data, psd=None, tau_res=None, fr=None, Qr=None,
                   f_range=None, panels=None, calibrations=None,
                   effective_dof=None, n_averages=None,
                   masked_fraction=None, event_rate=None, export=None):
    """
    Fit the quasiparticle lifetime to a generation-recombination roll-off.

    The GR Lorentzian is ``S_0 / (1 + (2 pi f tau_qp)**2)``, but the resonator
    itself low-passes with its ring-down ``tau_res = Q_r / (pi f_r)``, so what
    is measured is the two-pole shape::

        S(f) = S_0 / [(1 + (2 pi f tau_qp)^2) (1 + (2 pi f tau_res)^2)] + white

    At ``Q_r = 1e5`` and ``f_r = 1 GHz``, ``tau_res`` is ~32 us (corner ~5 kHz)
    while a ``tau_qp`` of ~100 us corners at ~1.6 kHz -- the same decade. The
    two are not separable from one trace, so ``tau_res`` is **fixed**, never
    fitted.

    Parameters
    ----------
    data : dict or array_like
        A `stitch_psds`, `snapshot_psds`, `timestream_psds` or `cross_psd`
        result, or a frequency array with ``psd``. A `cross_psd` result is the
        best input: readout and TLS noise are private to the frequency and
        dissipation quadratures and cancel in the cross-spectrum, leaving the
        common quasiparticle term on a floor those two have already left.
    psd : array_like, optional
        PSD rows, when ``data`` is a frequency array.
    tau_res : float or array_like, optional
        Ring-down time in seconds. ``None`` derives it from ``fr`` and ``Qr``.
        An array is taken per tone -- **use one**: real resonators differ enough
        that a single value is wrong for most of them. Across three tones of one
        capture ``tau_res`` ran 7.9 to 44.7 us.
    fr, Qr : float or array_like, optional
        Resonance frequency in Hz and loaded Q from the calibration fit, scalar
        or one per tone. ``Qr`` is the loaded Q, ``1/Qr = 1/Qi + 1/|Qc|``, which
        `fitting.fit_sweep_stack` returns as ``Ql``.
    f_range : tuple of float, optional
        ``(lo, hi)`` in Hz to fit over. ``None`` (default) uses every positive
        finite bin.
    panels : sequence of str, optional
        Panels to fit. ``None`` (default) fits all of them.
    calibrations : optional
        A calibration or fit result carrying ``fr`` and ``Qr``.
    effective_dof : float, optional
        Independent degrees of freedom behind the PSD, echoed into the result
        as provenance. The error inflation itself comes from
        ``bin_oversampling`` on a `snapshot_psds` result: concatenated snapshot
        bins are oversampled and correlated, so treating them as independent
        makes the quoted ``tau_qp`` error optimistic by ``sqrt`` of the
        oversampling -- around 5x at the usual settings.
    n_averages : int or array_like, optional
        Welch averages behind each bin, used to undo the log-space
        chi-square bias. ``None`` (default) takes the per-bin ``'n_avg'`` a
        `stitch_psds` result carries. This matters most on a composite: the
        bands either side of a join have different ``n_avg``, so an
        uncorrected fit reads the ~0.7 dB step between their biases as
        spectral structure.
    masked_fraction, event_rate : float, optional
        Deglitching provenance, echoed into the result. Glitch trains and GR
        noise are spectrally identical (Campbell's theorem gives a pulse train
        of decay ``tau_qp`` the same Lorentzian with the same corner), so an
        unmasked cosmic-ray population inflates ``S_0`` while leaving the corner
        roughly right, with no bad-fit flag. Only the time domain separates
        them -- see `find_glitches`.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'tau_qp'``, ``'S_0'``, ``'white'`` : dict of ndarray
            Per panel, one value per tone. ``tau_qp`` is in seconds.
        ``'f_corner'`` : dict of ndarray
            ``1 / (2 pi tau_qp)`` in Hz.
        ``'tau_qp_err'``, ``'S_0_err'``, ``'white_err'`` : dict of ndarray
            Standard errors, scaled up for ``effective_dof`` where it is known.
        ``'covariance'`` : dict of ndarray
            ``(n_tones, 3, 3)`` in the fitted ``(log10 S_0, log10 tau_qp,
            log10 white)`` basis.
        ``'tau_res'`` : float or ndarray
            The fixed ring-down used, per tone if one was given per tone.
        ``'tau_res_source'`` : str
            Where it came from.
        ``'flags'`` : dict of ndarray of str
            Comma-separated per tone. ``'poorly_separated'`` when ``tau_qp`` is
            within a factor of 2 of ``tau_res`` -- report a limit, not a value;
            ``'corner_in_gap'`` when the corner falls in a `stitch_psds` gap,
            so nothing constrains it; ``'corner_above_band'`` when it sits
            above the highest fitted bin, where the PFB channel response also
            shapes the spectrum; plus ``'no_data'``, ``'fit_failed'``,
            ``'corner_below_band'``.
        ``'n_points'``, ``'effective_dof'``, ``'masked_fraction'``,
        ``'event_rate'``, ``'f_range'``, ``'panels'``, ``'tones'``
            Provenance.

    Raises
    ------
    ValueError
        If ``tau_res`` can be neither given nor derived, or ``f_range`` leaves
        no bins.

    Warns
    -----
    UserWarning
        If ``masked_fraction`` is not given, so the result does not record
        whether the input was deglitched. A glitch train has the same
        Lorentzian and the same corner as GR noise, so an unmasked population
        inflates ``S_0`` with no bad-fit flag -- measured at 1.6x with an empty
        flags string.

    See Also
    --------
    fit_glitch_pulses : the independent time-domain estimator of ``tau_qp``.
    stitch_psds : builds the wideband input.
    cross_psd : the preferred input, with readout and TLS noise removed.

    Notes
    -----
    ``tau_qp`` from this fit and ``tau_qp`` from `fit_glitch_pulses` have almost
    disjoint systematics -- this one is limited by the stitch, the aliasing and
    the white floor, the other by window truncation, threshold selection and
    pile-up. Agreement is strong evidence; disagreement localises which model is
    wrong.

    Examples
    --------
    >>> import numpy as np
    >>> tau_qp, tau_res = 100e-6, 32e-6
    >>> f = np.logspace(1, 5, 400)
    >>> s = 1e-16 / ((1 + (2*np.pi*f*tau_qp)**2) * (1 + (2*np.pi*f*tau_res)**2))
    >>> out = fit_gr_rolloff(f, s[None] + 1e-21, tau_res=tau_res)
    >>> bool(abs(out['tau_qp']['psd'][0] / tau_qp - 1) < 0.05)
    True
    >>> float(np.round(out['f_corner']['psd'][0], -1))
    1590.0
    """
    tau_res, source = _resolve_tau_res(tau_res, fr, Qr, calibrations)
    f, rows, all_panels, tones, meta = _gr_operand(data, psd)
    tau_res_arr = np.atleast_1d(np.asarray(tau_res, dtype=float))
    use = tuple(panels) if panels is not None else all_panels
    if effective_dof is None:
        effective_dof = meta['effective_dof']
    gaps = meta['gaps']
    # Correlated bins inflate the variance by the oversampling factor, and by
    # nothing else: effective_dof is a per-bin dof, not a bin count, so
    # dividing a bin count by it would be wrong by orders of magnitude.
    variance_inflation = meta['oversampling']

    # log10 of a chi2_{2n}/2n bin sits below log10 of its mean. On a stitched
    # composite n differs between bands, so an uncorrected fit sees the step
    # between them as spectral structure.
    log_bias = _log_chi2_bias(n_averages, meta['n_avg'], f.size)

    in_range = np.isfinite(f) & (f > 0)
    if f_range is not None:
        in_range &= (f >= float(f_range[0])) & (f <= float(f_range[1]))
    if not np.any(in_range):
        raise ValueError(
            f"f_range {f_range} leaves no positive finite bins; the PSD covers "
            f"{np.nanmin(f[f > 0]):g}-{np.nanmax(f):g} Hz")

    out = {k: {} for k in ('tau_qp', 'S_0', 'white', 'f_corner', 'tau_qp_err',
                           'S_0_err', 'white_err', 'covariance', 'n_points',
                           'flags')}
    for panel in use:
        arr = rows[panel]
        n_tones = arr.shape[0]
        vals = {k: np.full(n_tones, np.nan) for k in
                ('tau_qp', 'S_0', 'white', 'f_corner', 'tau_qp_err',
                 'S_0_err', 'white_err')}
        cov = np.full((n_tones, 3, 3), np.nan)
        n_pts = np.zeros(n_tones, dtype=int)
        flags = []

        for t in range(n_tones):
            sel = in_range & np.isfinite(arr[t]) & (arr[t] > 0)
            n_pts[t] = int(sel.sum())
            if n_pts[t] < 4:
                flags.append('no_data')
                continue
            tau_r = float(tau_res_arr[t if tau_res_arr.size > 1 else 0])
            ff, pp = f[sel], arr[t][sel]
            bias = log_bias if np.isscalar(log_bias) or np.ndim(log_bias) == 0 \
                else np.asarray(log_bias)[sel]
            y = np.log10(pp) + bias
            seed = _seed_gr(ff, 10.0 ** y, tau_r)
            try:
                popt, pcov = scipy_optimize.curve_fit(
                    lambda x, ls, lt, lw: np.log10(
                        _gr_model(x, 10.0 ** ls, 10.0 ** lt, 10.0 ** lw,
                                  tau_r)),
                    ff, y, p0=seed, maxfev=20000,
                    bounds=([-np.inf, np.log10(tau_r) - 4, -np.inf],
                            [np.inf, np.log10(tau_r) + 4, np.inf]))
            except (RuntimeError, ValueError):
                flags.append('fit_failed')
                continue

            # Bins are correlated when oversampled, so the curve_fit covariance
            # (which assumes independence) understates the errors.
            pcov = pcov * variance_inflation
            s0, tq, wh = 10.0 ** popt
            vals['S_0'][t], vals['tau_qp'][t], vals['white'][t] = s0, tq, wh
            vals['f_corner'][t] = 1.0 / (2.0 * np.pi * tq)
            ln10 = np.log(10.0)
            for name, value, idx in (('S_0_err', s0, 0), ('tau_qp_err', tq, 1),
                                     ('white_err', wh, 2)):
                vals[name][t] = abs(value) * ln10 * np.sqrt(abs(pcov[idx, idx]))
            cov[t] = pcov

            tags = []
            ratio = tq / tau_r
            if 0.5 <= ratio <= 2.0:
                tags.append('poorly_separated')
            corner = vals['f_corner'][t]
            if corner > ff[-1]:
                tags.append('corner_above_band')
            if corner < ff[0]:
                tags.append('corner_below_band')
            if any(lo < corner < hi for lo, hi in gaps):
                tags.append('corner_in_gap')
            # This model has no 1/f term. A composite that reaches down into
            # the TLS decade will drag S_0 up and tau_qp long with no bad-fit
            # signature, so say when the low end is still rising.
            edge = max(4, len(ff) // 20)
            if np.median(pp[:edge]) > 3.0 * _gr_model(
                    ff[0], 10.0 ** popt[0], tq, wh, tau_r):
                tags.append('low_end_rising')
            if abs(np.log10(tq / tau_r)) > 3.9:
                tags.append('tau_at_bound')
            flags.append(','.join(tags))

        for k, v in vals.items():
            out[k][panel] = v
        out['covariance'][panel] = cov
        out['n_points'][panel] = n_pts
        out['flags'][panel] = np.asarray(flags, dtype=object).astype(str)

    out['tau_res'] = (float(tau_res) if np.ndim(tau_res) == 0
                      else np.asarray(tau_res, dtype=float))
    out['tau_res_source'] = source
    out['effective_dof'] = effective_dof
    out['masked_fraction'] = masked_fraction
    out['event_rate'] = event_rate
    out['panels'] = use
    out['tones'] = tones
    out['f_range'] = (np.nan, np.nan) if f_range is None else (
        float(f_range[0]), float(f_range[1]))
    if masked_fraction is None:
        warnings.warn(
            "no masked_fraction was given, so this fit does not record whether "
            "the input was deglitched. A cosmic-ray train has the same "
            "Lorentzian shape and corner as GR noise, so an unmasked "
            "population inflates S_0 with no bad-fit flag; run find_glitches "
            "and masked_psd first",
            stacklevel=2)
    return _maybe_export(out, export)


def _seed_gr(f, p, tau_res):
    """Seed ``(log10 S_0, log10 tau_qp, log10 white)`` from the trace itself.

    The plateau is taken from the flattest stretch rather than from the lowest
    bins: on a composite that reaches into the TLS decade the lowest bins are
    the ``1/f`` level, not ``S_0``, and seeding from them puts ``tau_qp``
    several decades out.
    """
    p = np.asarray(p, dtype=float)
    hi = float(np.median(p[-max(1, len(p) // 10):]))
    white = max(hi, float(np.max(p)) * 1e-12)

    # Walk down from the top: the plateau is the lowest frequency at which the
    # trace has stopped rising as fast as 1/f would.
    logf, logp = np.log10(f), np.log10(np.maximum(p, np.finfo(float).tiny))
    slope = np.gradient(logp, logf)
    flat = np.where(slope > -0.5)[0]
    start = int(flat[0]) if flat.size else 0
    lo = float(np.median(p[start:start + max(1, len(p) // 20)]))
    s0 = max(lo - white, lo * 1e-3)

    half = s0 / 2.0 + white
    below = np.where((p <= half) & (np.arange(p.size) >= start))[0]
    corner = float(f[below[0]]) if below.size else float(f[len(f) // 2])
    tau = 1.0 / (2.0 * np.pi * max(corner, f[0]))
    tau = min(max(tau, tau_res / 1e3), tau_res * 1e3)
    return [np.log10(s0), np.log10(tau), np.log10(white)]


# ---------------------------------------------------------------------------
# Glitch detection
# ---------------------------------------------------------------------------

def _as_time_rows(data, sample_rate=None, tones=None, quantity='magnitude'):
    """Normalise a timestream input into ``(rows, sample_rate, tone_indices)``.

    Accepts a ``parse_samples`` dict, a real or complex array of shape
    ``(n_tones, n_samples)`` or ``(n_samples,)``, or an ``(i, q)`` pair. This
    is the only place `find_glitches` inspects the input's type.
    """
    if isinstance(data, Mapping):
        if 'i_data' not in data:
            raise ValueError(
                "dict input must be a parse_samples() result with 'i_data' "
                f"and 'q_data'; got keys {sorted(data)[:8]}")
        selected = _get_tone_data(data, tones if tones is not None
                                  else range(len(data['i_data'])))
        z = np.vstack([i + 1j * q for _, i, q in selected])
        idx = np.asarray([int(k) for k, _, _ in selected], dtype=int)
        rate = sample_rate if sample_rate is not None else data['sample_rate']
        return _project(z, quantity), float(rate), idx

    if isinstance(data, tuple) and len(data) == 2:
        i_rows = np.atleast_2d(np.asarray(data[0], dtype=float))
        q_rows = np.atleast_2d(np.asarray(data[1], dtype=float))
        z = i_rows + 1j * q_rows
    else:
        z = np.atleast_2d(np.asarray(data))

    if sample_rate is None:
        raise ValueError(
            "sample_rate is required when passing plain arrays; pass a "
            "parse_samples dict to take it from the capture metadata.")
    rows = _project(z, quantity)
    idx = np.arange(rows.shape[0], dtype=int)
    if tones is not None:
        pick = np.asarray(list(tones), dtype=int)
        rows, idx = rows[pick], idx[pick]
    return rows, float(sample_rate), idx


def _project(z, quantity):
    """Reduce complex rows to the real quantity glitches are searched in."""
    if not np.iscomplexobj(z):
        return np.asarray(z, dtype=float)
    if quantity in ('magnitude', 'auto'):
        return np.abs(z)
    if quantity == 'phase':
        return np.unwrap(np.angle(z), axis=-1)
    if quantity == 'i':
        return z.real.astype(float)
    if quantity == 'q':
        return z.imag.astype(float)
    raise ValueError(
        f"quantity must be 'magnitude', 'phase', 'i' or 'q', got {quantity!r}")


def find_glitches(data, sample_rate=None, tones=None, threshold=5.0,
                  method='mad', template=None, min_separation=None,
                  window=None, quantity='magnitude', export=None):
    """
    Locate glitch events (cosmic-ray hits and similar impulses) in a stream.

    A running median is subtracted first, then the residual is thresholded at
    a multiple of its median absolute deviation. Both halves matter: a running
    *mean* would be dragged by the very events being looked for, and a plain
    standard-deviation threshold trips continuously on ``1/f`` drift.

    Parameters
    ----------
    data : dict or ndarray or tuple of ndarray
        A ``parse_samples()`` dict, a real or complex array of shape
        ``(n_tones, n_samples)`` or ``(n_samples,)``, or an ``(i, q)`` pair.
    sample_rate : float, optional
        Sample rate in Hz. Required for plain arrays.
    tones : sequence of int, optional
        Tone indices to search. ``None`` (default) searches every tone.
    threshold : float, optional
        Detection threshold in units of the residual MAD. Default 5.
    method : {'mad', 'matched'}, optional
        ``'mad'`` (default) thresholds the high-passed residual directly and
        needs no prior knowledge. ``'matched'`` correlates with ``template``
        first, which is worth it on a stream that resolves the pulse shape.
    template : ndarray or float, optional
        Matched-filter template, or a decay time in seconds from which a
        one-sided exponential is built. Required for ``method='matched'``.
    min_separation : float, optional
        Merge detections closer together than this many seconds into one
        event. ``None`` (default) merges only adjacent samples for
        ``method='mad'``, and detections within one template length for
        ``method='matched'``. **Set this on a stream that resolves the pulse
        shape**: a decaying pulse crosses the threshold several times on the
        way down, and each crossing would otherwise be catalogued as its own
        event, inflating the rate several-fold.
    window : int, optional
        Running-median length in samples. ``None`` (default) uses one second
        of data, capped at an eighth of the record so that edge padding
        cannot dominate, then clipped to an odd value in ``[5, n_samples]``.
    quantity : {'magnitude', 'phase', 'i', 'q'}, optional
        Which real quantity to search when the input is complex or a
        ``parse_samples`` dict. Default ``'magnitude'``.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'mask'`` : ndarray of bool, ``(n_tones, n_samples)``, ``True`` on
        flagged samples. This is what `masked_psd` consumes.
        ``'any_mask'`` : ndarray of bool, ``(n_samples,)``, flagged in any tone.
        ``'tones'`` : ndarray, ``(n_tones,)`` absolute tone indices.
        ``'sample_rate'`` : float.
        ``'events'`` : dict of ndarray, one entry per detection --
        ``'tone'`` (absolute index), ``'sample'``, ``'time_s'``,
        ``'amplitude'`` (peak of the high-passed excursion, in the searched
        quantity's own units, so it means the same for both methods),
        ``'snr'`` (peak detection statistic / its MAD), ``'width'`` (flagged
        samples), ``'n_coincident'`` (how many *tones* are flagged within one
        sample of it -- 1 means this tone alone).
        ``'n_events'`` : ndarray, ``(n_tones,)``.
        ``'flagged_fraction'`` : ndarray, ``(n_tones,)``.
        ``'mad'`` : ndarray, ``(n_tones,)``, residual MAD scaled to a Gaussian
        sigma.
        ``'threshold'``, ``'method'``, ``'window'``, ``'quantity'`` :
        provenance.

    Raises
    ------
    ValueError
        If ``method='matched'`` without a ``template``, or ``method`` is
        unknown.

    Notes
    -----
    **Rates from an accumulated stream are lower limits.** The accumulator
    averages a pulse over its whole window, diluting the amplitude by roughly
    ``tau_qp / T_acc`` -- about 0.06 for a 100 us lifetime at ``acc_len=1000``.
    Small hits therefore fall under any threshold, and amplitudes from the
    accumulated and snapshot streams are not on the same scale.

    **Deglitch before common-mode removal.** One impact deposits phonons
    across the substrate and hits many resonators at once, so glitches are
    partly common across tones. Running an SVD first absorbs them into the
    common mode and smears them over every tone, including ones never hit.
    This function reads raw rows for that reason, and ``'n_coincident'`` is
    the cleanest discriminator available: real impacts are coincident across
    tones, per-detector artefacts are not.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> rows = rng.normal(size=(2, 4000))
    >>> rows[0, 1500] += 40.0                    # one clear event
    >>> out = find_glitches(rows, sample_rate=488.28)
    >>> int(out['n_events'][0]), int(out['n_events'][1])
    (1, 0)
    >>> int(out['events']['sample'][0])
    1500
    >>> bool(out['mask'][0, 1500])
    True
    """
    if method not in ('mad', 'matched'):
        raise ValueError(f"method must be 'mad' or 'matched', got {method!r}")
    rows, rate, idx = _as_time_rows(data, sample_rate, tones, quantity)
    n_tones, n_samples = rows.shape

    kernel = None
    if method == 'matched':
        if template is None:
            raise ValueError("method='matched' requires template=")
        if np.isscalar(template):
            n_t = max(2, int(round(5.0 * float(template) * rate)))
            kernel = np.exp(-np.arange(n_t) / (float(template) * rate))
        else:
            kernel = np.asarray(template, dtype=float)
        kernel = kernel / np.sqrt(np.sum(kernel ** 2))

    if window is None:
        # One second of data, but never a large fraction of the record: with
        # mode='nearest' a window comparable to the array is dominated by edge
        # padding, which turns the residual into a ramp and manufactures
        # detections. An eighth of the record is still a long high-pass.
        window = int(min(max(round(rate), 5), max(5, n_samples // 8)))
    window = int(window)
    if window % 2 == 0:
        window -= 1
    window = max(3, min(window, n_samples if n_samples % 2 else n_samples - 1))

    mask = np.zeros((n_tones, n_samples), dtype=bool)
    mads = np.zeros(n_tones)
    ev = {k: [] for k in ('tone', 'sample', 'time_s', 'amplitude', 'snr', 'width')}
    if min_separation is not None:
        min_gap = max(1, int(round(float(min_separation) * rate)))
    elif kernel is not None:
        # A resolved pulse crosses the threshold repeatedly as it decays; the
        # template length is the natural scale over which that is one event.
        min_gap = len(kernel)
    else:
        min_gap = 1

    for t in range(n_tones):
        # 'reflect' rather than 'nearest': repeating a single edge sample
        # across half a window leaves a baseline step there, and the matched
        # filter integrates that step into a spurious high-SNR detection.
        baseline = scipy_ndimage.median_filter(rows[t], size=window,
                                               mode='reflect')
        excursion = rows[t] - baseline
        if kernel is not None:
            # Correlate, keeping lag 0 aligned with the sample the pulse
            # starts on -- 'same' convolution would report it (len-1)//2 late.
            detect = np.correlate(excursion, kernel, mode='full')
            detect = detect[len(kernel) - 1:len(kernel) - 1 + n_samples]
        else:
            detect = excursion
        mad = 1.4826 * float(np.median(np.abs(detect - np.median(detect))))
        mads[t] = mad
        if not mad > 0:
            continue
        hit = np.abs(detect) > threshold * mad
        mask[t] = hit
        if not hit.any():
            continue

        # Group runs of flagged samples, merging any closer than min_gap.
        edges = np.flatnonzero(np.diff(np.concatenate(([0], hit.view(np.int8), [0]))))
        starts, stops = edges[::2], edges[1::2]
        keep_s, keep_e = [starts[0]], [stops[0]]
        for s, e in zip(starts[1:], stops[1:]):
            if s - keep_e[-1] <= min_gap:
                keep_e[-1] = e
            else:
                keep_s.append(s)
                keep_e.append(e)
        for s, e in zip(keep_s, keep_e):
            peak = s + int(np.argmax(np.abs(detect[s:e])))
            ev['tone'].append(int(idx[t]))
            ev['sample'].append(peak)
            ev['time_s'].append(peak / rate)
            # Amplitude always comes from the high-passed excursion, so it
            # means the same thing for both methods; the matched filter's own
            # output is scaled by the template norm and is only a statistic.
            ev['amplitude'].append(float(excursion[peak]))
            ev['snr'].append(float(abs(detect[peak]) / mad))
            ev['width'].append(int(e - s))

    events = {k: np.asarray(v, dtype=float if k in ('time_s', 'amplitude', 'snr')
                            else int) for k, v in ev.items()}
    any_mask = mask.any(axis=0)
    # Coincidence across tones: dilate each tone's mask by +-1 sample first,
    # then count how many TONES are flagged there. Summing the tone axis
    # before the +-1 window instead would let one wide single-tone event
    # count as three, which is exactly the case this has to distinguish.
    padded = np.zeros((n_tones, n_samples + 2), dtype=bool)
    padded[:, 1:-1] = mask
    dilated = padded[:, :-2] | padded[:, 1:-1] | padded[:, 2:]
    tones_near = dilated.sum(axis=0)
    events['n_coincident'] = (tones_near[events['sample']].astype(int)
                              if events['sample'].size
                              else np.zeros(0, dtype=int))

    n_events = np.asarray([int(np.sum(events['tone'] == i)) for i in idx],
                          dtype=int)
    return _maybe_export({
        'mask': mask, 'any_mask': any_mask, 'tones': idx,
        'sample_rate': float(rate), 'events': events, 'n_events': n_events,
        'flagged_fraction': mask.mean(axis=1),
        'mad': mads, 'threshold': float(threshold), 'method': method,
        'window': int(window), 'quantity': quantity,
    }, export)


# ---------------------------------------------------------------------------
# Pulse-decay lifetime
# ---------------------------------------------------------------------------

def _two_pole_shape(dt, tau_qp, tau_res):
    """Unit-integral two-pole impulse response, zero before ``dt = 0``."""
    out = np.zeros_like(dt)
    live = dt >= 0
    if abs(tau_qp - tau_res) < 1e-12 * max(tau_qp, tau_res):
        out[live] = (dt[live] / tau_qp) * np.exp(-dt[live] / tau_qp) / tau_qp
    else:
        out[live] = (np.exp(-dt[live] / tau_qp)
                     - np.exp(-dt[live] / tau_res)) / (tau_qp - tau_res)
    return out


def _two_pole_pulse(t, energy, t0, tau_qp, tau_res, baseline):
    """Impulse through a quasiparticle decay and the resonator ring-down.

    Parameterised by ``energy``, the *integral* of the excursion. The
    unnormalised two-pole shape has unit integral, so its prefactor is the
    deposited energy directly. Peak-normalising instead would make the fitted
    parameter a peak height, which at fixed energy scales as ``1/tau_qp`` and
    builds a spurious anticorrelation into ``tau_vs_amplitude``.
    """
    dt = np.asarray(t, dtype=float) - t0
    return baseline + energy * _two_pole_shape(dt, tau_qp, tau_res)


def _two_pole_peak(tau_qp, tau_res):
    """Peak value of the unit-integral shape, for converting energy to height."""
    if abs(tau_qp - tau_res) < 1e-12 * max(tau_qp, tau_res):
        return float(np.exp(-1.0) / tau_qp)
    t_peak = (np.log(tau_qp / tau_res) / (1.0 / tau_res - 1.0 / tau_qp))
    return float(_two_pole_shape(np.array([t_peak]), tau_qp, tau_res)[0])


def _pulse_blocks(data, sample_rate, tones, quantity):
    """``([(tone, rows)], sample_rate)`` for the pulse fitter.

    Takes snapshot-shaped input where it can, and falls back to the timestream
    normaliser so a `parse_samples` capture works too. Rows come back already
    projected onto ``quantity``.
    """
    try:
        blocks, rate, _ = _as_snapshot_blocks(data, sample_rate, tones)
        return [(t, _project(r, quantity)) for t, r in blocks], rate
    except (ValueError, KeyError, TypeError):
        rows, rate, tone_idx = _as_time_rows(data, sample_rate, tones,
                                             quantity=quantity)
        return [(int(t), r) for t, r in zip(tone_idx, rows)], rate


def _pulse_arrivals(events, blocks, rate, threshold, detect_window,
                    min_separation, quantity):
    """``(tone, block, sample)`` triples to fit."""
    if events is None:
        out = []
        for tone_index, rows in blocks:
            # Search each block separately. The blocks are separate
            # acquisitions, so joining them end to end puts a step at every
            # boundary -- a pulse still decaying at the end of one block meets
            # the fresh baseline of the next -- and each of those is detected
            # as a spurious event.
            rows_2d = np.atleast_2d(rows)
            found = find_glitches(rows_2d, sample_rate=rate,
                                  threshold=threshold, window=detect_window,
                                  min_separation=min_separation,
                                  quantity=quantity)
            for block_idx, sample in zip(found['events']['tone'],
                                         found['events']['sample']):
                out.append((tone_index, int(block_idx), int(sample)))
        return out

    ev = events.get('events', events) if isinstance(events, Mapping) else {}
    if 'sample' not in ev:
        raise ValueError(
            "events must be a find_glitches() result or its 'events' "
            f"sub-dict; got keys {sorted(ev)[:8]}")
    # A find_glitches run over an (n_blocks, len_snapshot) array treats each
    # block as a row, so its 'tone' column is a block index and 'sample' is
    # already within-block. A run over a continuous capture indexes one long
    # record per tone, which has to be folded back into blocks.
    widths = {t: (r.shape[-1] if r.ndim == 2 else r.size) for t, r in blocks}
    blocked = {t: (r.ndim == 2 and r.shape[0] > 1) for t, r in blocks}
    single_tone = len(blocks) == 1
    tone_col = ev.get('tone', np.zeros(len(ev['sample']), dtype=int))
    out = []
    for row, sample in zip(tone_col, ev['sample']):
        row, sample = int(row), int(sample)
        if single_tone:
            tone_index = blocks[0][0]
            if blocked[tone_index]:
                out.append((tone_index, row, sample))
                continue
        else:
            tone_index = row
        width = widths.get(tone_index)
        if width is not None:
            out.append((tone_index, sample // width, sample % width))
    return out


def fit_glitch_pulses(data, events=None, sample_rate=None, tau_res=None,
                      fr=None, Qr=None, calibrations=None, tones=None,
                      quantity='magnitude', window=None, threshold=5.0,
                      truncation_tail=5.0, detect_window=None,
                      min_separation=None, export=None):
    """
    Fit ``tau_qp`` to individual glitch pulses, resolved in a fast stream.

    A phonon impact breaks Cooper pairs and the excess decays at ``tau_qp``.
    The resonator cannot respond faster than its ring-down, so the observed
    pulse is the two-pole shape ``(exp(-t/tau_qp) - exp(-t/tau_res)) /
    (tau_qp - tau_res)`` -- rise set by ``tau_res``, decay by ``tau_qp``. As in
    `fit_gr_rolloff`, ``tau_res`` is **fixed** from the calibration fit and
    degeneracy is flagged when the two land within a factor of 2.

    Parameters
    ----------
    data : dict or array_like
        A ``batch_snapshots()`` dict, a single snapshot dict, a complex
        ``(n_blocks, len_snapshot)`` array, or a `parse_samples` dict or array.
        Arrays require ``sample_rate``.
    events : dict, optional
        A `find_glitches` result, or its ``'events'`` sub-dict, giving the
        arrivals to fit. ``None`` (default) runs `find_glitches` here.
    sample_rate : float, optional
        Sample rate in Hz. Required for array input.
    tau_res : float, optional
        Ring-down in seconds. ``None`` derives it from ``fr`` and ``Qr``.
    fr, Qr : float, optional
        Resonance frequency in Hz and loaded Q from the calibration fit.
    calibrations : optional
        A calibration or fit result carrying ``fr`` and ``Qr``.
    tones : sequence of int, optional
        Tone indices to search. ``None`` (default) takes all of them.
    quantity : {'magnitude', 'phase', 'i', 'q'}, optional
        Projection searched and fitted. Default ``'magnitude'``.
    window : int, optional
        Samples after each arrival to **fit**. ``None`` (default) uses the rest
        of the containing block. Keep it a few times ``tau_qp``: the detector
        high-passes but the fit does not, so hundreds of samples of trailing
        baseline let a slow drift pull the decay -- an 8% in-block drift biases
        ``tau_qp`` about 15% low.
    threshold : float, optional
        Detection threshold in MAD, when ``events`` is not given. Default 5.
    truncation_tail : float, optional
        Multiples of the fitted ``tau_qp`` that must fit inside the block for
        an event to count as complete. Default 5.
    detect_window : int, optional
        Running-median length for **detection**, when ``events`` is not given.
        Separate from ``window``: a long fit window must not disable the
        detector's high-pass. ``None`` (default) leaves `find_glitches` to pick
        it, including its cap at an eighth of the record.
    min_separation : int, optional
        Samples within which detections merge into one event, when ``events``
        is not given. ``None`` (default) uses ``10 * tau_res`` in samples, on
        the grounds that ``tau_qp`` is typically a few times ``tau_res``.
        **A resolved pulse crosses the threshold several times on the way
        down**, so without this one impact is catalogued two or three times and
        the event rate -- which is what separates a Campbell glitch train from
        real GR noise -- is inflated. Raise it if ``tau_qp >> tau_res``.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'events'`` : dict of ndarray
            Per event: ``'tone'``, ``'block'``, ``'sample'``, ``'time_s'``,
            ``'energy'`` (the fitted integral of the excursion, i.e. the
            deposited energy), ``'amplitude'`` (the peak height that energy
            produces, which is what a threshold sees), ``'tau_qp'``,
            ``'tau_qp_err'``, ``'baseline'``, ``'rms_residual'``,
            ``'n_points'``, ``'truncated'``, ``'flags'``.

            ``'sample'`` and ``'time_s'`` are the *detection* position, which
            for the default ``method='mad'`` is the pulse maximum rather than
            its start -- tens of microseconds late at typical lifetimes. The
            fitted arrival is what the fit's own ``t0`` absorbs.
        ``'tau_qp'`` : float
            Median fitted lifetime over the accepted (untruncated) events.
        ``'tau_qp_mad'`` : float
            Their MAD scaled to a Gaussian sigma -- the population spread, not
            the error on the median.
        ``'n_events'``, ``'n_accepted'`` : int
        ``'acceptance'`` : float
            Fraction of fitted events that were not truncated.
        ``'arrival_histogram'`` : tuple of ndarray
            ``(counts, edges)`` of arrival sample within the block. A real
            population is uniform; a deficit late in the block is the
            truncation bias showing itself.
        ``'amplitude_spectrum'`` : tuple of ndarray
            ``(counts, edges)`` over accepted ``'energy'`` values -- the
            deposited-energy distribution.
        ``'tau_vs_amplitude'`` : dict
            ``'r'`` (Spearman) and ``'p'`` between ``'energy'`` and ``tau_qp``
            over accepted events. These should be independent; a correlation
            indicates saturation or resonator nonlinearity, and marks where the
            pulse fit stops being trustworthy.
        ``'tau_res'``, ``'tau_res_source'``, ``'window'``, ``'quantity'``,
        ``'sample_rate'``
            Provenance.

    Raises
    ------
    ValueError
        If ``tau_res`` can be neither given nor derived, or array input arrives
        without ``sample_rate``.

    Warns
    -----
    UserWarning
        When more than a third of events are truncated, since the surviving
        population is then biased towards short lifetimes.

    See Also
    --------
    find_glitches : produces ``events``.
    fit_gr_rolloff : the independent frequency-domain estimator of ``tau_qp``.

    Notes
    -----
    **Window truncation is the main selection bias.** A 1024-sample block is
    1.707 ms at 600 kHz. A ``tau_qp`` of 100 us decays inside ~5 tau = 500 us
    and fits comfortably; 300 us needs 1.5 ms and is marginal; longer, or
    arriving near the end of the block, is truncated. Rejecting truncated
    pulses biases the survivors towards **short** lifetimes, so the acceptance
    fraction and the arrival distribution are reported rather than silently
    corrected.

    Examples
    --------
    >>> import numpy as np
    >>> fs, tau_qp, tau_res = 600e3, 100e-6, 32e-6
    >>> t = np.arange(1024) / fs
    >>> rng = np.random.default_rng(0)
    >>> dt = t - t[100]
    >>> shape = np.where(dt >= 0, np.exp(-dt/tau_qp) - np.exp(-dt/tau_res), 0)
    >>> block = (1.0 + 0.001 * rng.standard_normal((4, 1024))
    ...          + 0.05 * shape / shape.max())
    >>> out = fit_glitch_pulses(block + 0j, sample_rate=fs, tau_res=tau_res)
    >>> bool(abs(out['tau_qp'] / tau_qp - 1) < 0.15)
    True
    >>> out['n_accepted'] == out['n_events'] > 0    # none truncated
    True
    """
    tau_res, source = _resolve_tau_res(tau_res, fr, Qr, calibrations)
    blocks, rate = _pulse_blocks(data, sample_rate, tones, quantity)
    if not blocks:
        raise ValueError("no tones selected")
    if min_separation is None:
        min_separation = max(1, int(round(10.0 * tau_res * rate)))
    arrivals = _pulse_arrivals(events, blocks, rate, threshold, detect_window,
                               min_separation, quantity)
    by_tone = dict(blocks)
    first = blocks[0][1]
    block_len = first.shape[-1] if first.ndim == 2 else first.size

    keys = ('tone', 'block', 'sample', 'time_s', 'amplitude', 'energy',
            'tau_qp', 'tau_qp_err', 'baseline', 'rms_residual', 'n_points')
    cat = {k: [] for k in keys}
    truncated, flags = [], []

    for tone_index, block_idx, start in arrivals:
        rows = by_tone[tone_index]
        series = rows[block_idx] if rows.ndim == 2 else rows
        stop = len(series) if window is None else min(len(series),
                                                      start + int(window))
        lead = 2                      # a couple of pre-arrival points anchor
        seg = np.asarray(series[max(start - lead, 0):stop], dtype=float)
        if seg.size < 8:
            continue
        t_seg = np.arange(seg.size) / rate

        baseline0 = float(np.median(seg[-max(3, seg.size // 10):]))
        peak0 = float(np.max(seg) - baseline0)
        tau0 = max(tau_res * 3, 1e-9)
        energy0 = peak0 / max(_two_pole_peak(tau0, tau_res), 1e-30)
        try:
            popt, pcov = scipy_optimize.curve_fit(
                lambda x, e, t0, lt, b: _two_pole_pulse(
                    x, e, t0, 10.0 ** lt, tau_res, b),
                t_seg, seg,
                p0=[energy0, lead / rate, np.log10(tau0), baseline0],
                maxfev=20000)
        except (RuntimeError, ValueError):
            continue

        energy, t0, log_tau, base = popt
        tau = float(10.0 ** log_tau)
        model = _two_pole_pulse(t_seg, energy, t0, tau, tau_res, base)
        rms = float(np.sqrt(np.mean((seg - model) ** 2)))
        err = (float(tau * np.log(10.0) * np.sqrt(abs(pcov[2, 2])))
               if np.all(np.isfinite(pcov)) else np.nan)
        # 'energy' is the integral (the deposited energy); 'amplitude' is the
        # peak height it produces, which is what a threshold actually sees.
        amp = float(energy) * _two_pole_peak(tau, tau_res)

        is_truncated = bool((start + truncation_tail * tau * rate) > block_len)
        tags = []
        if is_truncated:
            tags.append('truncated')
        if 0.5 <= tau / tau_res <= 2.0:
            tags.append('poorly_separated')
        if not np.isfinite(err):
            tags.append('no_error')

        for key, value in zip(keys, (tone_index, block_idx, start,
                                     start / rate, amp, float(energy), tau,
                                     err, float(base), rms, seg.size)):
            cat[key].append(value)
        truncated.append(is_truncated)
        flags.append(','.join(tags))

    cat = {k: np.asarray(v, dtype=float if k not in ('tone', 'block', 'sample',
                                                     'n_points') else int)
           for k, v in cat.items()}
    cat['truncated'] = np.asarray(truncated, dtype=bool)
    cat['flags'] = np.asarray(flags, dtype=object).astype(str)

    n_events = int(cat['tau_qp'].size)
    good = ((~cat['truncated']) & np.isfinite(cat['tau_qp'])) if n_events \
        else np.zeros(0, dtype=bool)
    accepted = cat['tau_qp'][good]
    acceptance = float(good.mean()) if n_events else np.nan
    if n_events and acceptance < 2.0 / 3.0:
        warnings.warn(
            f"{100 * (1 - acceptance):.0f}% of pulses are truncated by the "
            "block length, so the surviving population is biased towards "
            "short lifetimes; read 'acceptance' and 'arrival_histogram' "
            "before quoting tau_qp",
            stacklevel=2)

    # Correlate against energy, not peak height. At fixed deposited energy the
    # peak scales as 1/tau_qp, so using 'amplitude' here would report a strong
    # anticorrelation for a perfectly monoenergetic, perfectly linear detector.
    corr = {'r': np.nan, 'p': np.nan}
    if accepted.size >= 4:
        energies = cat['energy'][good]
        if np.ptp(energies) > 0 and np.ptp(accepted) > 0:
            r, p = scipy_stats.spearmanr(energies, accepted)
            corr = {'r': float(r), 'p': float(p)}

    empty_hist = (np.zeros(10, dtype=int), np.zeros(11))
    return _maybe_export({
        'events': cat,
        'tau_qp': float(np.median(accepted)) if accepted.size else np.nan,
        'tau_qp_mad': (float(1.4826 * np.median(
            np.abs(accepted - np.median(accepted))))
            if accepted.size else np.nan),
        'n_events': n_events,
        'n_accepted': int(accepted.size),
        'acceptance': acceptance,
        'arrival_histogram': (np.histogram(cat['sample'], bins=10,
                                           range=(0, max(block_len, 1)))
                              if n_events else empty_hist),
        'amplitude_spectrum': (np.histogram(cat['energy'][good], bins=10)
                               if accepted.size else empty_hist),
        'tau_vs_amplitude': corr,
        'tau_res': float(tau_res),
        'tau_res_source': source,
        'window': window,
        'quantity': quantity,
        'sample_rate': float(rate),
    }, export)


# ---------------------------------------------------------------------------
# Export and reload
# ---------------------------------------------------------------------------

# Separator between a key and its panel name in a flattened .npz. Chosen so it
# cannot collide with a panel name ('i', 'q', 'magnitude', 'phase',
# 'frequency', 'dissipation', 'psd').
_PANEL_SEP = '__'


def _flatten_result(result):
    """Split a result dict into flat arrays plus JSON-able metadata.

    The results here nest arrays one level deep under a panel name, which
    ``np.savez`` can only store by pickling into a 0-d object array -- and
    ``np.load`` then refuses to read under its default ``allow_pickle=False``.
    Flattening on write keeps the in-memory shape convenient and the file
    readable without pickle.
    """
    arrays, meta = {}, {}
    for key, value in result.items():
        if isinstance(value, Mapping):
            for panel, sub in value.items():
                arr = np.asarray(sub)
                if arr.dtype.kind in 'fiub':
                    arrays[f'{key}{_PANEL_SEP}{panel}'] = arr
                elif arr.dtype.kind in 'SU':
                    arrays[f'{key}{_PANEL_SEP}{panel}'] = arr.astype('U')
                else:
                    meta.setdefault(key, {})[str(panel)] = _jsonable(sub)
            meta.setdefault('_panel_keys', []).append(key)
        elif isinstance(value, np.ndarray):
            arrays[key] = (value.astype('U') if value.dtype.kind == 'O'
                           else value)
        elif isinstance(value, (int, float, str, bool, type(None))):
            meta[key] = value
        else:
            meta[key] = _jsonable(value)
    return arrays, meta


def _jsonable(value):
    """Best-effort conversion of provenance into something JSON can hold."""
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, Mapping):
        return {str(k): _jsonable(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_jsonable(v) for v in value]
    if isinstance(value, (np.integer, np.floating, np.bool_)):
        return value.item()
    if isinstance(value, (int, float, str, bool)) or value is None:
        return value
    return str(value)


def _maybe_export(result, export):
    """Write ``result`` if ``export`` is a path, and return it either way.

    This is what makes ``export=`` droppable into an existing call: the return
    value is unchanged, so adding it never restructures anything.
    """
    if export is not None:
        export_psds(result, export)
    return result


def export_psds(result, path):
    """
    Write an analysis result to a ``.npz`` that loads without pickle.

    Any result dict from this module is accepted. Arrays nested one level under
    a panel name are flattened to ``key__panel`` on write, so the file can be
    read back with ``np.load(path)`` at its default ``allow_pickle=False``;
    `load_psds` restores the nested shape.

    Parameters
    ----------
    result : dict
        A result from `timestream_psds`, `snapshot_psds`, `subtract_psds`,
        `psd_band_levels`, `fit_psd_model`, `cross_psd`, `stitch_psds`,
        `allan_deviation`, `timestream_stats` or the others here.
    path : str or pathlib.Path
        Destination. ``.npz`` is appended if absent. A directory is treated as
        a run directory and the file goes in ``analysis/`` inside it, matching
        the existing ``power_sweep`` layout.

    Returns
    -------
    str
        The path actually written.

    See Also
    --------
    load_psds : read it back.
    export_summary : a flat CSV for several results at once.

    Examples
    --------
    >>> import numpy as np, tempfile, os
    >>> f = np.linspace(0, 100, 51)
    >>> res = {'f': f, 'panels': ('frequency', 'dissipation'),
    ...        'psd': {'frequency': np.ones((2, 51)),
    ...                'dissipation': np.full((2, 51), 2.0)},
    ...        'tones': np.array([0, 3]), 'format': 'freq_diss'}
    >>> path = export_psds(res, os.path.join(tempfile.mkdtemp(), 'noise'))
    >>> back = load_psds(path)
    >>> bool(np.allclose(back['psd']['dissipation'], 2.0)), back['format']
    (True, 'freq_diss')
    """
    import json
    import os

    path = str(path)
    if os.path.isdir(path):
        path = os.path.join(path, 'analysis', 'timeseries')
    if not path.endswith('.npz'):
        path += '.npz'
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    arrays, meta = _flatten_result(result)
    arrays['_metadata_json'] = np.asarray(json.dumps(meta, default=str))
    np.savez_compressed(path, **arrays)
    return path


def load_psds(path):
    """
    Read a result written by `export_psds` back into its nested shape.

    Loads with ``allow_pickle=False``, so the file cannot execute anything on
    open.

    Parameters
    ----------
    path : str or pathlib.Path
        The ``.npz`` written by `export_psds`.

    Returns
    -------
    dict
        The result dict, with ``key__panel`` entries restored to
        ``{key: {panel: array}}``. Downstream functions such as
        `psd_band_levels` and `fit_psd_model` accept it directly.

    See Also
    --------
    export_psds : write it.

    Examples
    --------
    See `export_psds`.
    """
    import json

    with np.load(str(path), allow_pickle=False) as handle:
        raw = {k: handle[k] for k in handle.files}
    meta = json.loads(str(raw.pop('_metadata_json'))) if '_metadata_json' \
        in raw else {}
    panel_keys = set(meta.pop('_panel_keys', []))

    out = {}
    for key, value in raw.items():
        if _PANEL_SEP in key:
            head, panel = key.split(_PANEL_SEP, 1)
            out.setdefault(head, {})[panel] = value
        else:
            out[key] = value
    for key, value in meta.items():
        if key in panel_keys and isinstance(value, Mapping):
            out.setdefault(key, {}).update(value)
        else:
            out.setdefault(key, value)
    # panels round-trips through JSON as a list; keep it a tuple as produced.
    if isinstance(out.get('panels'), list):
        out['panels'] = tuple(out['panels'])
    return out


def export_summary(path, *results, tones=None):
    """
    Write one flat CSV row per ``(tone, panel, band)`` across several results.

    Takes any number of `psd_band_levels`, `fit_psd_model` or
    `timestream_stats` results, so an on-resonance, off-resonance and
    difference set land in one table ready for a spreadsheet.

    Parameters
    ----------
    path : str or pathlib.Path
        Destination. ``.csv`` is appended if absent; a directory becomes
        ``analysis/`` inside it.
    *results : dict
        Results to tabulate. Pass ``(label, result)`` pairs to name the rows,
        or bare results to have them numbered.
    tones : sequence of int, optional
        Restrict to these tone indices.

    Returns
    -------
    str
        The path actually written.

    See Also
    --------
    export_psds : the full arrays, rather than a summary.

    Examples
    --------
    >>> import numpy as np, tempfile, os
    >>> f = np.linspace(1, 100, 100)
    >>> psd = np.full((2, 100), 1e-18)
    >>> lv = psd_band_levels(f, psd, bands={'science': (1.0, 10.0)})
    >>> p = export_summary(os.path.join(tempfile.mkdtemp(), 's'), ('on', lv))
    >>> open(p).readline().strip().split(',')[:4]
    ['label', 'tone', 'panel', 'band']
    """
    import csv
    import os

    path = str(path)
    if os.path.isdir(path):
        path = os.path.join(path, 'analysis', 'summary')
    if not path.endswith('.csv'):
        path += '.csv'
    parent = os.path.dirname(path)
    if parent:
        os.makedirs(parent, exist_ok=True)

    rows, fields = [], ['label', 'tone', 'panel', 'band']
    for n, entry in enumerate(results):
        label, result = entry if isinstance(entry, tuple) else (str(n), entry)
        rows.extend(_summary_rows(str(label), result, tones, fields))

    with open(path, 'w', newline='') as handle:
        writer = csv.DictWriter(handle, fieldnames=fields,
                                extrasaction='ignore')
        writer.writeheader()
        writer.writerows(rows)
    return path


def _summary_rows(label, result, tones, fields):
    """Flatten one result into ``(tone, panel, band)`` rows."""
    panels = tuple(result.get('panels') or ())
    tone_list = np.asarray(result.get('tones', [])).tolist()
    names = list(result.get('names') or [])
    rows = []
    # Every per-panel entry whose array is indexed by tone (and maybe band).
    per_panel = {k: v for k, v in result.items()
                 if isinstance(v, Mapping) and set(v).issuperset(set(panels))
                 and panels}
    for t, tone in enumerate(tone_list):
        if tones is not None and tone not in list(tones):
            continue
        for panel in panels:
            n_bands = 1
            for value in per_panel.values():
                arr = np.asarray(value[panel])
                if arr.ndim == 2 and arr.shape[0] > t:
                    n_bands = max(n_bands, arr.shape[1])
            for b in range(n_bands):
                row = {'label': label, 'tone': tone, 'panel': panel,
                       'band': names[b] if b < len(names) else
                       ('' if n_bands == 1 else b)}
                for key, value in per_panel.items():
                    arr = np.asarray(value[panel])
                    try:
                        row[key] = (arr[t, b] if arr.ndim == 2 else arr[t])
                    except (IndexError, ValueError):
                        continue
                    if key not in fields:
                        fields.append(key)
                rows.append(row)
    return rows


# ---------------------------------------------------------------------------
# Time-domain views
# ---------------------------------------------------------------------------

def _panel_time_rows(ts_data, format, tones, sweep_data, sample_rate,
                     reference_tone_frequency, smooth_window_hz,
                     conversion_method, calibrations, decorrelate_modes,
                     decorrelate_tones, blind_tone_modes, blind_tones):
    """The same rows `timestream_psds` would spectrum, before any PSD.

    Shares the conversion and cleaning path so a time-domain view and a PSD of
    the same capture are guaranteed to be of the same numbers.
    """
    ts_data = _as_ts_data(ts_data, sample_rate)
    rate = ts_data.get('sample_rate', sample_rate)
    if rate is None:
        raise ValueError("sample_rate is required when passing plain arrays")
    if format not in PSD_PANELS:
        raise ValueError(
            f"unknown format {format!r}; use one of {tuple(PSD_PANELS)}")
    panels = PSD_PANELS[format]
    selected = _get_tone_data(ts_data, tones if tones is not None else [0])
    conversion = conversion_method or 'linearized'
    cleaning_on = bool(decorrelate_modes or blind_tone_modes)

    tone_indices = [int(k) for k, _, _ in selected]
    rows = [[], []]

    if format in ('iq', 'magphase'):
        cleaned = {}
        if cleaning_on:
            cleaned = _compute_cleaned_iq(
                ts_data, decorrelate_modes, decorrelate_tones,
                blind_tone_modes, blind_tones)
        for key, i_arr, q_arr in selected:
            idx = int(key)
            if cleaning_on and idx in cleaned:
                i_arr, q_arr = cleaned[idx]
            if format == 'iq':
                rows[0].append(np.asarray(i_arr, dtype=float))
                rows[1].append(np.asarray(q_arr, dtype=float))
            else:
                z = np.asarray(i_arr) + 1j * np.asarray(q_arr)
                rows[0].append(np.abs(z))
                rows[1].append(np.unwrap(np.angle(z)))
    else:
        raw, cleaned = {}, {}
        if cleaning_on:
            raw, cleaned = _compute_cleaned_freq_diss(
                ts_data, sweep_data, tone_indices, decorrelate_modes,
                decorrelate_tones, blind_tone_modes, blind_tones,
                reference_tone_frequency, smooth_window_hz, conversion,
                calibrations)
        for key, i_arr, q_arr in selected:
            idx = int(key)
            if cleaning_on and idx in cleaned:
                frac_f, frac_d = cleaned[idx]
            elif _has_precomputed_freq_diss(ts_data):
                frac_f, frac_d = _precomputed_freq_diss(
                    ts_data, key,
                    reference_tone_frequency=reference_tone_frequency)
            else:
                frac_f, frac_d = _compute_freq_diss(
                    ts_data, key, i_arr, q_arr, sweep_data,
                    reference_tone_frequency=reference_tone_frequency,
                    smooth_window_hz=smooth_window_hz,
                    conversion_method=conversion, calibrations=calibrations)
            rows[0].append(np.asarray(frac_f, dtype=float))
            rows[1].append(np.asarray(frac_d, dtype=float))

    stacked = {name: np.vstack(r) for name, r in zip(panels, rows)}
    return stacked, panels, np.asarray(tone_indices, dtype=int), float(rate)


def _octave_taus(n_samples, sample_rate):
    """Octave-spaced averaging times from one sample to a tenth of the record."""
    max_m = max(1, n_samples // 10)
    m = []
    step = 1
    while step <= max_m:
        m.append(step)
        step *= 2
    return np.asarray(m, dtype=int) / float(sample_rate)


def allan_deviation(data, format='freq_diss', tones=None, sweep_data=None,
                    taus=None, kind='overlapping', *, sample_rate=None,
                    reference_tone_frequency=None, smooth_window_hz=1000,
                    conversion_method=None, calibrations=None,
                    decorrelate_modes=0, decorrelate_tones=None,
                    blind_tone_modes=0, blind_tones=None, export=None):
    """
    Allan deviation of the rows `timestream_psds` would take a spectrum of.

    The time-domain counterpart of a PSD: where a spectrum shows *where* the
    noise sits in frequency, the Allan deviation shows how long you can
    integrate before drift stops the averaging down. White noise falls as
    ``tau**-1/2``; a minimum marks the integration time beyond which drift
    dominates.

    Takes the same conversion and cleaning arguments as `timestream_psds`, and
    shares its row preparation, so the two views are guaranteed to be of the
    same numbers.

    Parameters
    ----------
    data : dict or ndarray or tuple of ndarray
        A `parse_samples` dict, a complex array of shape ``(n_tones,
        n_samples)`` or ``(n_samples,)``, or an ``(i, q)`` pair. Plain arrays
        require ``sample_rate`` and support ``format='iq'`` and ``'magphase'``.
    format : {'iq', 'magphase', 'freq_diss'}, optional
        Quantity to analyse. Default ``'freq_diss'``.
    tones : sequence of int, optional
        Tone indices. ``None`` (default) selects tone 0.
    sweep_data : dict, optional
        ``parse_sweep_data()`` dict, required for ``format='freq_diss'``.
    taus : array_like, optional
        Averaging times in seconds. ``None`` (default) gives octave-spaced
        values from ``1/sample_rate`` to a tenth of the record.
    kind : {'overlapping', 'standard'}, optional
        ``'overlapping'`` (default) reuses every sample offset and so has far
        lower scatter at long tau. ``'standard'`` uses disjoint bins.
    sample_rate : float, optional
        Sample rate in Hz. Required for plain arrays.
    reference_tone_frequency, smooth_window_hz, conversion_method, calibrations
        Freq/diss conversion, as in `timestream_psds`.
    decorrelate_modes, decorrelate_tones, blind_tone_modes, blind_tones
        Cleaning, as in `timestream_psds`.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'tau'`` : ndarray, shape (n_tau,)
            Averaging times actually used, in seconds.
        ``'adev'`` : dict of ndarray
            ``(n_tones, n_tau)`` Allan deviation per panel, in the panel's own
            units.
        ``'adev_err'`` : dict of ndarray
            ``adev / sqrt(n_pairs)``, the standard scatter estimate.
        ``'n_pairs'`` : ndarray, shape (n_tau,)
            Differences averaged at each tau; long tau rests on very few.
        ``'tau_min'`` : dict of ndarray
            Per tone, the tau of the minimum -- the useful integration time.
        ``'slope'`` : dict of ndarray
            Fitted log-log slope over the first decade. ``-0.5`` is white,
            ``0`` flicker, ``+0.5`` a random walk.
        ``'panels'``, ``'tones'``, ``'sample_rate'``, ``'kind'``, ``'format'``
            Provenance.

    Raises
    ------
    ValueError
        For an unknown ``format`` or ``kind``, for plain arrays without
        ``sample_rate``, or if no ``tau`` is short enough for the record.

    See Also
    --------
    timestream_psds : the frequency-domain view of the same rows.
    timestream_stats : cheap per-tone summary statistics.

    Notes
    -----
    Allan deviation and the PSD carry the same information for a stationary
    process, so this adds nothing a spectrum cannot show. It is easier to read
    for the question "how long can I integrate", and it does not need the
    record to be stationary in the way a Welch estimate implicitly assumes.

    ``'n_pairs'`` matters: at tau near a tenth of the record only a handful of
    differences enter, so the last few points scatter hard and a minimum there
    may not be real. The default ``taus`` stop at that tenth for this reason.

    Examples
    --------
    White noise falls as ``tau**-1/2``:

    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> x = rng.standard_normal(20000)
    >>> out = allan_deviation(x + 0j, format='iq', sample_rate=100.0)
    >>> bool(abs(out['slope']['i'][0] + 0.5) < 0.05)
    True

    A random walk rises instead:

    >>> out = allan_deviation(np.cumsum(rng.standard_normal(20000)) + 0j,
    ...                       format='iq', sample_rate=100.0)
    >>> bool(out['slope']['i'][0] > 0.3)
    True
    """
    if kind not in ('overlapping', 'standard'):
        raise ValueError(
            f"kind must be 'overlapping' or 'standard', got {kind!r}")
    rows, panels, tone_idx, rate = _panel_time_rows(
        data, format, tones, sweep_data, sample_rate,
        reference_tone_frequency, smooth_window_hz, conversion_method,
        calibrations, decorrelate_modes, decorrelate_tones, blind_tone_modes,
        blind_tones)

    n_samples = rows[panels[0]].shape[1]
    if taus is None:
        taus = _octave_taus(n_samples, rate)
    taus = np.atleast_1d(np.asarray(taus, dtype=float))
    m = np.unique(np.maximum(1, np.round(taus * rate).astype(int)))
    m = m[m <= max(1, n_samples // 2)]
    if m.size == 0:
        raise ValueError(
            f"no tau is short enough for a {n_samples}-sample record at "
            f"{rate:g} Hz; the longest usable is {n_samples / (2 * rate):g} s")
    tau_used = m / rate

    adev = {p: np.full((len(tone_idx), m.size), np.nan) for p in panels}
    err = {p: np.full((len(tone_idx), m.size), np.nan) for p in panels}
    n_pairs = np.zeros(m.size, dtype=int)

    for p in panels:
        arr = rows[p]
        for j, mm in enumerate(m):
            for t in range(arr.shape[0]):
                sigma, pairs = _allan_point(arr[t], int(mm), kind)
                adev[p][t, j] = sigma
                if p == panels[0] and t == 0:
                    n_pairs[j] = pairs
                if pairs > 0:
                    err[p][t, j] = sigma / np.sqrt(pairs)

    tau_min, slope = {}, {}
    for p in panels:
        best = np.full(len(tone_idx), np.nan)
        sl = np.full(len(tone_idx), np.nan)
        for t in range(len(tone_idx)):
            y = adev[p][t]
            good = np.isfinite(y) & (y > 0)
            if np.any(good):
                best[t] = tau_used[good][np.argmin(y[good])]
                # Slope over the first decade, where the statistics are good.
                decade = good & (tau_used <= tau_used[good][0] * 10)
                if decade.sum() >= 3:
                    sl[t] = np.polyfit(np.log10(tau_used[decade]),
                                       np.log10(y[decade]), 1)[0]
        tau_min[p], slope[p] = best, sl

    return _maybe_export({
        'tau': tau_used,
        'adev': adev,
        'adev_err': err,
        'n_pairs': n_pairs,
        'tau_min': tau_min,
        'slope': slope,
        'panels': panels,
        'tones': tone_idx,
        'sample_rate': rate,
        'kind': kind,
        'format': format,
    }, export)


def _allan_point(x, m, kind):
    """``(sigma_y, n_pairs)`` at averaging length ``m`` samples."""
    n = x.size
    if n < 2 * m + 1:
        return np.nan, 0
    if kind == 'overlapping':
        # Cumulative sums give every overlapping m-sample average at once.
        c = np.concatenate(([0.0], np.cumsum(x, dtype=float)))
        means = (c[m:] - c[:-m]) / m
        d = means[m:] - means[:-m]
    else:
        n_bins = n // m
        means = x[:n_bins * m].reshape(n_bins, m).mean(axis=1)
        d = np.diff(means)
    if d.size == 0:
        return np.nan, 0
    return float(np.sqrt(0.5 * np.mean(d ** 2))), int(d.size)


def timestream_stats(data, format='freq_diss', tones=None, sweep_data=None,
                     outlier_sigma=5.0, *, sample_rate=None,
                     reference_tone_frequency=None, smooth_window_hz=1000,
                     conversion_method=None, calibrations=None,
                     decorrelate_modes=0, decorrelate_tones=None,
                     blind_tone_modes=0, blind_tones=None, export=None):
    """
    Per-tone summary statistics of a timestream, including robust ones.

    Cheap, and it catches the "this one tone glitched" case that a PSD averages
    away. Takes the same conversion and cleaning arguments as
    `timestream_psds`, over the same rows.

    Parameters
    ----------
    data : dict or ndarray or tuple of ndarray
        As in `allan_deviation`.
    format : {'iq', 'magphase', 'freq_diss'}, optional
        Quantity to summarise. Default ``'freq_diss'``.
    tones : sequence of int, optional
        Tone indices. ``None`` (default) selects tone 0.
    sweep_data : dict, optional
        Required for ``format='freq_diss'``.
    outlier_sigma : float, optional
        Threshold in robust sigma for the outlier count. Default 5.
    sample_rate : float, optional
        Required for plain arrays.
    reference_tone_frequency, smooth_window_hz, conversion_method, calibrations
        Freq/diss conversion, as in `timestream_psds`.
    decorrelate_modes, decorrelate_tones, blind_tone_modes, blind_tones
        Cleaning, as in `timestream_psds`.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        One ``(n_tones,)`` array per panel under each of: ``'mean'``,
        ``'std'``, ``'ptp'``, ``'median'``, ``'mad_std'`` (MAD scaled to a
        Gaussian sigma), ``'skew'``, ``'kurtosis'`` (excess), ``'n_outliers'``
        (samples beyond ``outlier_sigma * mad_std`` of the median),
        ``'outlier_fraction'``, ``'drift'`` (least-squares slope per second)
        and ``'drift_fraction'`` (drift over the record, in units of
        ``mad_std``). Plus ``'panels'``, ``'tones'``, ``'sample_rate'``,
        ``'n_samples'``, ``'outlier_sigma'``, ``'format'``.

    See Also
    --------
    allan_deviation : how the noise averages down.
    find_glitches : locate the outliers this only counts.

    Notes
    -----
    ``std`` and ``mad_std`` differing by much is the useful signal: they agree
    for Gaussian noise, and ``std`` runs well above ``mad_std`` when a few
    glitches or a drift dominate. Compare them before trusting any number that
    assumes Gaussianity.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> x = rng.standard_normal(10000)
    >>> x[500] = 40.0                       # one big glitch
    >>> out = timestream_stats(x + 0j, format='iq', sample_rate=100.0)
    >>> bool(out['std']['i'][0] > 1.05 * out['mad_std']['i'][0])
    True
    >>> int(out['n_outliers']['i'][0]) >= 1
    True
    """
    rows, panels, tone_idx, rate = _panel_time_rows(
        data, format, tones, sweep_data, sample_rate,
        reference_tone_frequency, smooth_window_hz, conversion_method,
        calibrations, decorrelate_modes, decorrelate_tones, blind_tone_modes,
        blind_tones)

    keys = ('mean', 'std', 'ptp', 'median', 'mad_std', 'skew', 'kurtosis',
            'n_outliers', 'outlier_fraction', 'drift', 'drift_fraction')
    out = {k: {} for k in keys}
    n_samples = rows[panels[0]].shape[1]
    t = np.arange(n_samples) / rate

    for p in panels:
        arr = rows[p]
        vals = {k: np.full(arr.shape[0], np.nan) for k in keys}
        for i in range(arr.shape[0]):
            x = arr[i]
            finite = x[np.isfinite(x)]
            if finite.size < 2:
                continue
            med = float(np.median(finite))
            mad = float(1.4826 * np.median(np.abs(finite - med)))
            vals['mean'][i] = float(np.mean(finite))
            vals['std'][i] = float(np.std(finite, ddof=1))
            vals['ptp'][i] = float(np.ptp(finite))
            vals['median'][i] = med
            vals['mad_std'][i] = mad
            vals['skew'][i] = float(scipy_stats.skew(finite))
            vals['kurtosis'][i] = float(scipy_stats.kurtosis(finite))
            if mad > 0:
                n_out = int(np.sum(np.abs(finite - med)
                                   > outlier_sigma * mad))
                vals['n_outliers'][i] = n_out
                vals['outlier_fraction'][i] = n_out / finite.size
            ok = np.isfinite(x)
            if ok.sum() >= 2:
                slope = float(np.polyfit(t[ok], x[ok], 1)[0])
                vals['drift'][i] = slope
                if mad > 0:
                    vals['drift_fraction'][i] = slope * t[-1] / mad
        for k in keys:
            out[k][p] = vals[k]

    out['panels'] = panels
    out['tones'] = tone_idx
    out['sample_rate'] = rate
    out['n_samples'] = int(n_samples)
    out['outlier_sigma'] = float(outlier_sigma)
    out['format'] = format
    return _maybe_export(out, export)


# ---------------------------------------------------------------------------
# Combining the two quadratures
# ---------------------------------------------------------------------------

# The pipeline's dissipation row is Im(delta_tilde) = delta_Qi_inv / 2, i.e.
# the factor 1/2 is ALREADY applied (see doc/ and resonator.mobius_coordinate).
# For the same absorbed power the frequency row reads delta_x and the
# dissipation row reads delta_x / (2 beta), so referring dissipation to
# frequency-equivalent units multiplies by 2*beta -- not beta. Getting this
# wrong scales every combined result by 2.
def _dissipation_to_frequency_units(beta):
    """Factor putting an ``Im(delta_tilde)`` row into ``delta_x``-equivalent units."""
    return 2.0 * float(beta)


def beta_from_kernels(Tc, T, fr, gap_ratio=1.764):
    """
    Predict the responsivity ratio ``beta`` from Mattis-Bardeen kernels.

    ``beta = (1/2) |d sigma_2 / d n_qp| / (d sigma_1 / d n_qp)`` is a property
    of the film and the readout frequency alone, so this needs no measurement
    at all -- it is tier 3 of `combine_quadratures`'s fallback ladder, and the
    reason a missing optical loading sweep is not a blocker.

    Parameters
    ----------
    Tc : float
        Superconducting transition temperature in K.
    T : float
        Bath (or effective quasiparticle) temperature in K.
    fr : float
        Readout frequency in Hz.
    gap_ratio : float, optional
        ``Delta(0) / (k_B Tc)``. Default 1.764, the BCS weak-coupling value.

    Returns
    -------
    float
        The predicted ``beta``, dimensionless and positive.

    See Also
    --------
    estimate_beta : measure it instead, from the timestreams.
    combine_quadratures : what consumes it.

    Notes
    -----
    Uses the low-temperature limits

    ``d sigma_1 / d n_qp  ~  sqrt(2 Delta / (pi k T)) sinh(xi) K_0(xi)``,
    ``d sigma_2 / d n_qp  ~ -pi [1 + sqrt(2 Delta / (pi k T)) e^-xi I_0(xi)]``,

    with ``xi = h fr / (2 k T)``, both in units of ``1 / (N_0 h fr)``, which
    cancels in the ratio. Valid for ``T << Tc`` and ``h fr << Delta``; outside
    that the prediction should not be leaned on.

    Examples
    --------
    >>> b = beta_from_kernels(Tc=1.2, T=0.1, fr=1e9)
    >>> bool(1.0 < b < 6.0)
    True
    >>> bool(beta_from_kernels(1.2, 0.3, 1e9) > beta_from_kernels(1.2, 0.1, 1e9))
    True
    """
    k_b = 1.380649e-23
    h = 6.62607015e-34
    delta = gap_ratio * k_b * float(Tc)
    xi = h * float(fr) / (2.0 * k_b * float(T))
    root = np.sqrt(2.0 * delta / (np.pi * k_b * float(T)))

    d_sigma1 = root * np.sinh(xi) * scipy_special.k0(xi)
    d_sigma2 = np.pi * (1.0 + root * np.exp(-xi) * scipy_special.i0(xi))
    if not d_sigma1 > 0:
        raise ValueError(
            f"Mattis-Bardeen kernel is degenerate at Tc={Tc}, T={T}, fr={fr}")
    return float(0.5 * d_sigma2 / d_sigma1)


def _resolve_load_power(load_k, power_w, power_curve):
    """Absorbed power at each load, in watts, or ``None`` if unavailable."""
    if power_w is not None:
        power = np.atleast_1d(np.asarray(power_w, dtype=float))
        if power.size != load_k.size:
            raise ValueError(
                f"power_w has {power.size} entries but there are "
                f"{load_k.size} loads")
        return power
    if power_curve is None:
        return None
    if callable(power_curve):
        return np.atleast_1d(np.asarray(power_curve(load_k), dtype=float))

    t_cal, p_cal = power_curve
    t_cal = np.asarray(t_cal, dtype=float)
    p_cal = np.asarray(p_cal, dtype=float)
    if t_cal.shape != p_cal.shape:
        raise ValueError(
            f"power_curve arrays differ in shape ({t_cal.shape} vs "
            f"{p_cal.shape}); pass (temperature_k, power_w)")
    order = np.argsort(t_cal)
    t_cal, p_cal = t_cal[order], p_cal[order]
    outside = (load_k < t_cal[0]) | (load_k > t_cal[-1])
    if np.any(outside):
        warnings.warn(
            f"loads {np.round(load_k[outside], 2).tolist()} K lie outside the "
            f"power curve's {t_cal[0]:g}-{t_cal[-1]:g} K range, so their "
            "absorbed power is extrapolated flat by np.interp. Extend the "
            "curve or drop those loads",
            stacklevel=3)
    return np.interp(load_k, t_cal, p_cal)


def signal_direction_from_loads(load_k, fr, Qi, power_w=None,
                                power_curve=None, tones=None, export=None):
    """
    Measure the responsivity direction from an optical loading ladder.

    Tier 1 of `combine_quadratures`'s ladder, and the only one that fixes the
    **absolute** scale: regressing the fitted resonance frequency and internal
    loss against absorbed power gives ``R_x`` and ``R_d`` separately, not just
    their ratio.

    Parameters
    ----------
    load_k : array_like, shape (n_loads,)
        Blackbody temperature at each load, in K.
    fr : array_like, shape (n_loads,) or (n_loads, n_tones)
        Fitted resonance frequency at each load, in Hz.
    Qi : array_like, same shape as ``fr``
        Fitted internal quality factor at each load.
    power_w : array_like, optional
        Absorbed power at each load, in W. Given, the responsivities come back
        per watt and the result is an absolute calibration.
    power_curve : tuple of array_like, or callable, optional
        A ``(temperature_k, power_w)`` pair to interpolate at ``load_k``, or a
        callable ``P(T)`` returning watts. This is the usual route: the optical
        model is measured or computed once and reused across campaigns.
    tones : sequence of int, optional
        Tone indices, for labelling the output.
    export : str or pathlib.Path, optional
        Write the result with `export_psds` and return it unchanged.

    Returns
    -------
    dict
        ``'beta'`` : ndarray
            ``R_x / R_d``, per tone -- the ratio `combine_quadratures` and
            `estimate_beta` use, on the ``delta_Qi_inv`` convention. Available
            whether or not the power scale is known, since it is a ratio.
        ``'R_x'``, ``'R_d'`` : ndarray or None
            ``d(delta_x)/dP`` and ``d(delta_Qi_inv)/dP`` per watt, or ``None``
            when no power was supplied.
        ``'R_x_per_k'``, ``'R_d_per_k'`` : ndarray
            The same slopes against blackbody temperature, always available.
        ``'r2_x'``, ``'r2_d'`` : ndarray
            Coefficient of determination of each straight-line fit. Well below
            1 means the response curves over the load range and a single
            responsivity does not describe it.
        ``'power_w'`` : ndarray or None
            The absorbed power actually used.
        ``'n_loads'``, ``'load_k'``, ``'tones'``, ``'has_absolute_scale'``
            Provenance.

    Raises
    ------
    ValueError
        If fewer than three loads are given, or the shapes disagree.

    Warns
    -----
    UserWarning
        If a load lies outside the ``power_curve``'s range, or if a tone's
        response is poorly described by a straight line.

    See Also
    --------
    estimate_beta : the same ratio from timestreams alone, no load ladder.
    beta_from_kernels : predict it with no measurement at all.
    combine_quadratures : what consumes ``beta`` and ``R_x``.

    Notes
    -----
    ``beta`` here is ``R_x / R_d`` with ``R_d`` in ``delta_Qi_inv`` units, which
    is what `combine_quadratures` expects -- it applies the factor of 2 for the
    pipeline's ``Im(delta_tilde)`` row internally. Do not pre-halve it.

    **Without a power scale this measures nothing that `estimate_beta` does
    not**, and costs a whole load ladder to do it. Its unique output is ``R_x``
    in W, which needs the optical efficiency, throughput and band -- pass them
    in as ``power_curve``; they are not derivable from the readout data.

    Examples
    --------
    >>> import numpy as np
    >>> loads = np.array([5.0, 20.0, 50.0, 80.0])
    >>> power = np.array([1e-13, 4e-13, 1e-12, 1.6e-12])   # from the optics
    >>> fr = 1e9 * (1 - 3e-6 * power / 1e-12)              # x falls with load
    >>> Qi = 1.0 / (1e-6 + 1e-6 * power / 1e-12)           # loss rises
    >>> out = signal_direction_from_loads(loads, fr, Qi, power_w=power)
    >>> bool(abs(out['beta'][0] - 3.0) < 0.05)             # R_x / R_d = 3
    True
    >>> out['has_absolute_scale']
    True
    """
    load_k = np.atleast_1d(np.asarray(load_k, dtype=float))
    fr = np.asarray(fr, dtype=float)
    Qi = np.asarray(Qi, dtype=float)
    if fr.ndim == 1:
        fr, Qi = fr[:, None], Qi[:, None]
    if fr.shape != Qi.shape or fr.shape[0] != load_k.size:
        raise ValueError(
            f"fr {fr.shape} and Qi {Qi.shape} must both be "
            f"(n_loads={load_k.size}, n_tones)")
    if load_k.size < 3:
        raise ValueError(
            f"need at least three loads to fit a slope, got {load_k.size}")

    power = _resolve_load_power(load_k, power_w, power_curve)
    n_tones = fr.shape[1]
    tone_idx = (np.arange(n_tones) if tones is None
                else np.asarray(list(tones), dtype=int))

    # Fractional frequency and internal loss, referenced to the coldest load.
    coldest = int(np.argmin(load_k))
    delta_x = (fr - fr[coldest]) / fr[coldest]
    delta_qi_inv = 1.0 / Qi - 1.0 / Qi[coldest]

    def _fit(x, y):
        slope = np.full(n_tones, np.nan)
        r2 = np.full(n_tones, np.nan)
        for t in range(n_tones):
            ok = np.isfinite(x) & np.isfinite(y[:, t])
            if ok.sum() < 3:
                continue
            m, c = np.polyfit(x[ok], y[ok, t], 1)
            slope[t] = m
            resid = y[ok, t] - (m * x[ok] + c)
            var = np.var(y[ok, t])
            r2[t] = 1.0 - np.var(resid) / var if var > 0 else np.nan
        return slope, r2

    rx_k, r2_x = _fit(load_k, delta_x)
    rd_k, r2_d = _fit(load_k, delta_qi_inv)
    rx_w = rd_w = None
    if power is not None:
        rx_w, r2_x = _fit(power, delta_x)
        rd_w, r2_d = _fit(power, delta_qi_inv)

    with np.errstate(divide='ignore', invalid='ignore'):
        beta = (rx_w / rd_w) if power is not None else (rx_k / rd_k)
    beta = np.abs(beta)

    poor = np.isfinite(r2_x) & np.isfinite(r2_d) & ((r2_x < 0.9) | (r2_d < 0.9))
    if np.any(poor):
        warnings.warn(
            f"tones {tone_idx[poor].tolist()[:8]} are poorly described by a "
            f"straight line (r2 down to {np.nanmin([r2_x[poor].min(), r2_d[poor].min()]):.2f}); "
            "the response curves over this load range, so a single "
            "responsivity does not describe it. Narrow the load range or fit "
            "the curvature",
            stacklevel=2)

    return _maybe_export({
        'beta': beta,
        'R_x': rx_w,
        'R_d': rd_w,
        'R_x_per_k': rx_k,
        'R_d_per_k': rd_k,
        'r2_x': r2_x,
        'r2_d': r2_d,
        'power_w': power,
        'n_loads': int(load_k.size),
        'load_k': load_k,
        'tones': tone_idx,
        'has_absolute_scale': power is not None,
    }, export)


def estimate_beta(data, other=None, sample_rate=None, tones=None,
                  f_range=None, sweep_data=None, psd_kwargs=None,
                  reference_tone_frequency=None, conversion_method=None,
                  calibrations=None, beta_range=(1.5, 20.0), export=None):
    """
    Measure the responsivity ratio ``beta`` by in-band regression.

    Regresses the dissipation row on the frequency row over a band where
    genuine sky signal dominates. Tier 2 of `combine_quadratures`'s ladder:
    timestream-only, no load step needed, and sufficient for the weights, since
    only the ratio enters those.

    Parameters
    ----------
    data : dict or array_like
        A `cross_psd` result, a `timestream_psds` result with
        ``format='freq_diss'``, a `parse_samples` dict plus ``sweep_data``, or
        the frequency row as an array with the dissipation row in ``other``.
    other : array_like, optional
        Dissipation row, when ``data`` is the frequency row.
    sample_rate : float, optional
        Sample rate in Hz. Required for plain arrays.
    tones : sequence of int, optional
        Tone indices to use.
    f_range : tuple of float, optional
        ``(lo, hi)`` band to regress over. Choose one where a real signal
        dominates both channels; in a band of pure uncorrelated noise the
        regression measures nothing and the answer is meaningless.
    sweep_data, reference_tone_frequency, conversion_method, calibrations, psd_kwargs
        As in `cross_psd`, when a raw capture is given.
    beta_range : tuple of float, optional
        Plausible range for the sanity check. Default ``(1.5, 20.0)``, wide
        enough to cover this instrument's whole operating space (see Notes) and
        so loose enough only to catch values that are physically absurd.
        **For a real check, narrow it with** `beta_from_kernels` **for the film,
        temperature and resonance frequency in hand** -- a band that fits one
        instrument will cry wolf on another.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'beta'`` : ndarray, one per tone.
        ``'slope'`` : ndarray
            The fitted ``d(dissipation) / d(frequency)``; ``beta`` is
            ``1 / (2 * slope)``, the 2 because the dissipation row is
            ``Im(delta_tilde)``, already half of ``delta_Qi_inv``.
        ``'coherence'`` : ndarray
            Mean magnitude-squared coherence in the band. A low value means
            there is no common signal to regress and ``beta`` is not measured.
        ``'in_kernel_range'`` : ndarray of bool
            Whether each tone's ``beta`` falls inside ``beta_range``.
        ``'beta_range'``, ``'f_range'``, ``'tones'``, ``'n_avg'`` : provenance.

    Warns
    -----
    UserWarning
        When a tone's ``beta`` falls outside ``beta_range``. This is a
        safeguard, not a formality: a ratio well outside the kernel range means
        de-embedding leakage, not physics, and is the same fault the
        off-resonance coherence test in `cross_psd` catches independently.

    Notes
    -----
    ``beta`` is ``1 / (2 * slope)``, not ``1 / slope``: the pipeline's
    dissipation row is ``Im(delta_tilde)``, already half of ``delta_Qi_inv``.

    Read the returned ``'coherence'`` before the ``'beta'``. The regression only
    means something in a band where a real common signal dominates both
    channels; run it over pure uncorrelated noise and the slope is a ratio of
    two independent noise levels, which is a number but not a responsivity
    ratio.

    ``beta`` is a strong function of readout frequency and temperature, which is
    why the default range is wide. For this instrument's Al-Ti-Al film
    (``Tc`` ~ 1 K) `beta_from_kernels` gives:

    ========  ======  ======  ======  ======
    ``fr``    100 mK  150 mK  200 mK  300 mK
    ========  ======  ======  ======  ======
    0.5 GHz     6.9     9.6    12.2    17.4
    1.0 GHz     4.5     6.1     7.6    10.7
    2.0 GHz     3.1     4.1     5.0     6.8
    ========  ======  ======  ======  ======

    The assumed gap ratio barely matters -- at 150 mK and 1 GHz, ``beta`` moves
    only 6.3 to 5.9 across ``gap_ratio`` 1.4 to 2.2 -- so a proximitised
    trilayer needs no special treatment here.

    See Also
    --------
    beta_from_kernels : predict it instead, with no measurement.
    combine_quadratures : what consumes it.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> n = 40000
    >>> signal = np.cumsum(rng.standard_normal(n))      # common sky signal
    >>> x = signal + 0.05 * rng.standard_normal(n)
    >>> d = signal / (2 * 3.0) + 0.01 * rng.standard_normal(n)   # beta = 3
    >>> out = estimate_beta(x, d, sample_rate=1000.0, f_range=(1, 50))
    >>> bool(abs(out['beta'][0] / 3.0 - 1) < 0.1)
    True
    """
    if isinstance(data, Mapping) and 'cross' in data and 'coherence' in data:
        cx = data
    else:
        cx = cross_psd(data, other, sample_rate=sample_rate, tones=tones,
                       sweep_data=sweep_data, psd_kwargs=psd_kwargs,
                       reference_tone_frequency=reference_tone_frequency,
                       conversion_method=conversion_method,
                       calibrations=calibrations)

    f = np.asarray(cx['f'], dtype=float)
    sel = np.isfinite(f) & (f > 0)
    if f_range is not None:
        sel &= (f >= float(f_range[0])) & (f <= float(f_range[1]))
    if not np.any(sel):
        raise ValueError(
            f"f_range {f_range} leaves no bins; the spectrum covers "
            f"{np.nanmin(f[f > 0]):g}-{np.nanmax(f):g} Hz")

    panels = tuple(cx['panels'])
    s_xx = np.atleast_2d(cx['psd'][panels[0]])[:, sel]
    cross = np.atleast_2d(np.real(cx['cross']))[:, sel]
    coh = np.atleast_2d(cx['coherence'])[:, sel]

    # Regression slope of dissipation on frequency is Re(S_xd) / S_xx.
    with np.errstate(divide='ignore', invalid='ignore'):
        slope = np.sum(cross, axis=1) / np.sum(s_xx, axis=1)
        beta = 1.0 / (2.0 * slope)

    lo, hi = float(beta_range[0]), float(beta_range[1])
    inside = np.isfinite(beta) & (beta >= lo) & (beta <= hi)
    if not np.all(inside):
        bad = np.asarray(cx['tones'])[~inside]
        warnings.warn(
            f"beta for tones {bad.tolist()[:8]} falls outside the "
            f"{lo}-{hi} Mattis-Bardeen range "
            f"(measured {np.round(beta[~inside], 2).tolist()[:8]}). A ratio "
            "this far out is de-embedding leakage rather than physics -- "
            "check the freq/diss conversion before combining with it",
            stacklevel=2)

    return _maybe_export({
        'beta': beta,
        'slope': slope,
        'coherence': np.mean(coh, axis=1),
        'in_kernel_range': inside,
        'beta_range': (lo, hi),
        'f_range': (np.nan, np.nan) if f_range is None else (
            float(f_range[0]), float(f_range[1])),
        'tones': np.asarray(cx['tones'], dtype=int),
        'n_avg': cx.get('n_avg'),
    }, export)


def combine_quadratures(data, other=None, *, beta=None, weights='scalar',
                        R_x=None, f_range=None, beta_range=(1.8, 4.0), export=None,
                        **cross_kwargs):
    """
    Combine the frequency and dissipation quadratures into one noise estimate.

    Refers both channels to a common scale so the response vector becomes
    ``(1, 1)`` -- two noisy measurements of one quantity -- and combines them by
    generalised least squares. With ``a`` private to frequency, ``b`` private to
    dissipation and ``c`` common to both, the closed form is::

        S_comb = c**2 + (1/a**2 + 1/b**2)**-1
        (w_x, w_d) = (b**2, a**2) / (a**2 + b**2)

    Common photon and GR noise passes through untouched, private noise adds in
    inverse quadrature, and ``c`` drops out of the weights entirely, so the
    combination is never worse than the better single channel.

    Parameters
    ----------
    data : dict or array_like
        A `cross_psd` result (what this is built for), a `parse_samples` dict
        plus ``sweep_data=``, or the frequency row as an array.
    other : array_like, optional
        Dissipation row, when ``data`` is the frequency row. Both then need
        ``sample_rate=``.
    beta : float or None, optional
        Responsivity ratio. ``None`` (default) is **tier 4**: frequency-only,
        ``w_d = 0``. That is a legitimate outcome, not a failure -- in the
        readout-limited regime combining buys about 4%.
    weights : {'scalar', 'closed_form'}, optional
        ``'scalar'`` (default) uses the constant ``w_d = 1 / (1 + 4 beta**2)``,
        which recovers the whole available gain where in-band noise is white.
        ``'closed_form'`` fits ``a``, ``b``, ``c`` per bin from the
        cross-spectrum and weights per bin.
    R_x : float, optional
        Frequency responsivity in ``delta_x`` per watt. Given, the output is an
        absolute NEP in W/sqrt(Hz); omitted, it is in ``delta_x``-equivalent
        units, which is all a noise comparison needs.
    f_range : tuple of float, optional
        Band to report summary numbers over. ``None`` uses every positive bin.
    beta_range : tuple of float, optional
        Range over which to report the sensitivity to ``beta``. Default
        ``(1.8, 4.0)``. Pass the range `beta_from_kernels` gives for the
        actual film and readout frequency if the point estimate is uncertain --
        for an Al-Ti-Al film at ``Tc`` ~ 1 K that is nearer 4-8 at 1 GHz, and
        7-17 at 0.5 GHz.
    **cross_kwargs
        Passed to `cross_psd` when ``data`` is not already one of its results.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'f'`` : ndarray
        ``'psd'`` : ndarray, ``(n_tones, n_freq)``
            The combined PSD, in ``delta_x``-equivalent units squared per Hz,
            or W**2/Hz when ``R_x`` is given.
        ``'nep'`` : ndarray or None
            ``sqrt(psd) / R_x`` when ``R_x`` is given, else ``None``.
        ``'w_x'``, ``'w_d'`` : ndarray
            Weights, summing to 1. Scalar per tone for ``weights='scalar'``,
            per bin for ``'closed_form'``.
        ``'a2'``, ``'b2'``, ``'c2'`` : ndarray or None
            The private/private/common decomposition, for ``'closed_form'``.
        ``'gain'`` : ndarray
            Improvement over the frequency channel alone, as an amplitude
            ratio, over ``f_range``. 1.0 means the combination bought nothing.
        ``'gain_over_beta_range'`` : dict
            ``'beta'``, ``'gain'`` across ``beta_range``. If the span is narrow
            the measurement of ``beta`` was never needed, and that is visible
            rather than assumed.
        ``'tier'`` : int
            1 with ``R_x``, 2 or 3 with ``beta``, 4 without.
        ``'beta'``, ``'weights'``, ``'f_range'``, ``'tones'`` : provenance.

    See Also
    --------
    beta_from_kernels, estimate_beta : the two ways to supply ``beta``.
    cross_psd : produces the input.

    Notes
    -----
    **The dissipation row is** ``Im(delta_tilde)``, already half of
    ``delta_Qi_inv``, so referring it to frequency-equivalent units multiplies
    by ``2 * beta`` and the scalar weight is ``1 / (1 + 4 beta**2)``. Working
    from a note written in ``delta_Qi_inv`` and applying the half again gives
    every combined result out by a factor of 2.

    **The answer is insensitive to** ``beta``, which is what makes the
    kernel-prediction tier respectable rather than a fudge: across the whole
    1.8-4 range the readout-limited gain ``sqrt(1 + 1/(4 beta**2))`` moves
    only from 3.8% to 0.8% (both verified against simulation), and it only
    shrinks further at larger ``beta``. In the TLS-limited regime the weights
    are dominated by the measured ``a`` and ``b`` instead, and the gain is
    real: on a synthetic tone whose frequency channel is TLS-limited and whose
    dissipation channel is readout-limited, ``'closed_form'`` returned 21x
    against 1.03x for the scalar weights over 0.1-5 Hz.

    ``beta`` is a strong function of readout frequency and temperature -- for
    an Al-Ti-Al film at ``Tc`` ~ 1 K it runs 4.5 at 1 GHz and 100 mK to 17 at
    0.5 GHz and 300 mK -- so a value quoted for one instrument, or one bath
    temperature, does not transfer. Use `beta_from_kernels` with the actual
    resonance frequency; see `estimate_beta`'s Notes for a table.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> n = 60000
    >>> x = rng.standard_normal(n)
    >>> d = rng.standard_normal(n)              # equal, private, uncorrelated
    >>> out = combine_quadratures(x, d, beta=0.5, sample_rate=1000.0,
    ...                           psd_kwargs={'nperseg': 1024})
    >>> float(np.round(out['w_d'][0], 3))       # 1 / (1 + 4 * 0.25)
    0.5
    >>> out['tier']
    3

    Without ``beta`` it falls back to frequency-only, which is a legitimate
    answer rather than an error:

    >>> plain = combine_quadratures(x, d, sample_rate=1000.0)
    >>> plain['tier'], float(plain['gain'][0]), float(plain['w_d'][0])
    (4, 1.0, 0.0)
    """
    if weights not in ('scalar', 'closed_form'):
        raise ValueError(
            f"weights must be 'scalar' or 'closed_form', got {weights!r}")
    if isinstance(data, Mapping) and 'cross' in data and 'coherence' in data:
        cx = data
    else:
        cx = cross_psd(data, other, **cross_kwargs)

    f = np.asarray(cx['f'], dtype=float)
    panels = tuple(cx['panels'])
    s_xx = np.atleast_2d(np.asarray(cx['psd'][panels[0]], dtype=float))
    s_dd = np.atleast_2d(np.asarray(cx['psd'][panels[1]], dtype=float))
    s_xd = np.atleast_2d(np.real(np.asarray(cx['cross'])))

    band = np.isfinite(f) & (f > 0)
    if f_range is not None:
        band &= (f >= float(f_range[0])) & (f <= float(f_range[1]))

    combined, w_x, w_d, a2, b2, c2 = _combine_rows(
        s_xx, s_dd, s_xd, beta, weights, band)

    with np.errstate(divide='ignore', invalid='ignore'):
        gain = np.sqrt(np.nanmean(s_xx[:, band], axis=1)
                       / np.nanmean(combined[:, band], axis=1))

    betas = np.linspace(float(beta_range[0]), float(beta_range[1]), 9)
    sweep = []
    for b in betas:
        c_b, _, _, _, _, _ = _combine_rows(s_xx, s_dd, s_xd, b, weights, band)
        with np.errstate(divide='ignore', invalid='ignore'):
            sweep.append(np.sqrt(np.nanmean(s_xx[:, band], axis=1)
                                 / np.nanmean(c_b[:, band], axis=1)))

    tier = 4 if beta is None else (1 if R_x is not None else 3)
    nep = None
    if R_x is not None:
        with np.errstate(divide='ignore', invalid='ignore'):
            nep = np.sqrt(combined) / float(R_x)

    return _maybe_export({
        'f': f,
        'psd': combined,
        'nep': nep,
        'w_x': w_x,
        'w_d': w_d,
        'a2': a2,
        'b2': b2,
        'c2': c2,
        'gain': gain,
        'gain_over_beta_range': {'beta': betas,
                                 'gain': np.asarray(sweep).T},
        'tier': tier,
        'beta': beta,
        'weights': weights,
        'R_x': R_x,
        'f_range': (np.nan, np.nan) if f_range is None else (
            float(f_range[0]), float(f_range[1])),
        'tones': np.asarray(cx['tones'], dtype=int),
    }, export)


def _combine_rows(s_xx, s_dd, s_xd, beta, weights, band):
    """Combined PSD and weights, in delta_x-equivalent units."""
    n_tones = s_xx.shape[0]
    if beta is None:                      # tier 4: frequency only
        ones = np.ones(n_tones)
        return s_xx.copy(), ones, np.zeros(n_tones), None, None, None

    # Refer the dissipation row to delta_x-equivalent units. The 2 is because
    # the row is Im(delta_tilde), not delta_Qi_inv.
    scale = _dissipation_to_frequency_units(beta)
    s_dd_ref = s_dd * scale ** 2
    s_xd_ref = s_xd * scale

    if weights == 'scalar':
        w_d_val = 1.0 / (1.0 + 4.0 * float(beta) ** 2)
        w_x = np.full(n_tones, 1.0 - w_d_val)
        w_d = np.full(n_tones, w_d_val)
        wx, wd = w_x[:, None], w_d[:, None]
        combined = (wx ** 2 * s_xx + wd ** 2 * s_dd_ref
                    + 2.0 * wx * wd * s_xd_ref)
        return combined, w_x, w_d, None, None, None

    # Private / common decomposition, per bin. c is the part common to both.
    c2 = np.clip(s_xd_ref, 0.0, None)
    a2 = np.clip(s_xx - c2, np.finfo(float).tiny, None)
    b2 = np.clip(s_dd_ref - c2, np.finfo(float).tiny, None)
    total = a2 + b2
    w_x = b2 / total
    w_d = a2 / total
    combined = c2 + 1.0 / (1.0 / a2 + 1.0 / b2)
    return combined, w_x, w_d, a2, b2, c2


# ---------------------------------------------------------------------------
# Cross-spectrum and coherence
# ---------------------------------------------------------------------------

def _freq_diss_rows(ts_data, sweep_data, tones, reference_tone_frequency,
                    smooth_window_hz, conversion_method, calibrations,
                    apply_readout_correction):
    """Frequency and dissipation rows for a capture, as ``(n_tones, n_samples)``.

    Shares every conversion helper with `timestream_psds`, so the rows here are
    the same numbers whose PSDs that function returns.
    """
    ts_data = _with_samples_readout_correction(ts_data, apply_readout_correction)
    sweep_data = _with_sweep_readout_correction(sweep_data, apply_readout_correction)
    selected = _get_tone_data(ts_data, tones)
    precomputed = _has_precomputed_freq_diss(ts_data)
    if not precomputed and sweep_data is None:
        raise ValueError(
            "sweep_data is required to convert a raw I/Q capture into "
            "frequency and dissipation")
    if precomputed:
        _warn_ignored_conversion_args(conversion_method, calibrations)
    method = conversion_method or 'linearized'
    # Only 'linearized' and 'mobius' return Im(delta_tilde). 'circle' returns
    # the radial proxy Delta(rho), which is a different quantity: its cross
    # spectrum against the frequency row has no common-quasiparticle reading.
    if method == 'circle':
        raise ValueError(
            "conversion_method='circle' returns the radial proxy Delta(rho), "
            "not the dissipation quadrature Im(delta_tilde); a cross spectrum "
            "against it is not interpretable. Use 'linearized' or 'mobius'.")

    freq, diss, idx = [], [], []
    for key, i_arr, q_arr in selected:
        if precomputed:
            ff, dd = _precomputed_freq_diss(
                ts_data, key, reference_tone_frequency=reference_tone_frequency)
        else:
            ff, dd = _compute_freq_diss(
                ts_data, key, i_arr, q_arr, sweep_data,
                reference_tone_frequency=reference_tone_frequency,
                smooth_window_hz=smooth_window_hz,
                conversion_method=method, calibrations=calibrations)
        freq.append(np.asarray(ff, dtype=float))
        diss.append(np.asarray(dd, dtype=float))
        idx.append(int(key))
    return (np.vstack(freq), np.vstack(diss), np.asarray(idx, dtype=int),
            float(ts_data['sample_rate']), method, ts_data.get('info'))


def _effective_segments(window, nperseg, noverlap, n_seg):
    """Independent-segment count behind ``n_seg`` overlapping Welch segments.

    Overlapping segments share samples, so they are correlated and average
    down more slowly than their count suggests. For a window ``w`` and step
    ``d`` the correlation between segments ``k`` apart is
    ``rho_k = (sum w[n] w[n + k d])**2 / (sum w**2)**2``, and the variance
    behaves as if there were ``n_seg / (1 + 2 sum_k (1 - k/n_seg) rho_k)``
    independent ones. For the default Hann window at 50% overlap this is
    about 0.95 ``n_seg`` -- small, but it is the whole difference between a
    coherence floor that is right and one that is optimistic.
    """
    step = nperseg - noverlap
    if step <= 0 or n_seg <= 1:
        return float(max(n_seg, 1))
    w = np.asarray(scipy_signal.get_window(window, nperseg), dtype=float)
    denom = float(np.sum(w ** 2)) ** 2
    if denom <= 0:
        return float(n_seg)
    total = 0.0
    for k in range(1, min(n_seg, int(np.ceil(nperseg / step)))):
        shift = k * step
        if shift >= nperseg:
            break
        rho = float(np.sum(w[:nperseg - shift] * w[shift:])) ** 2 / denom
        total += (1.0 - k / n_seg) * rho
    return float(n_seg / (1.0 + 2.0 * total))


def cross_psd(data, other=None, sample_rate=None, tones=None, sweep_data=None,
              psd_kwargs=None, panels=('frequency', 'dissipation'),
              reference_tone_frequency=None, smooth_window_hz=1000,
              conversion_method=None, calibrations=None,
              apply_readout_correction=True, coherence_warn_segments=30, export=None):
    """
    Cross spectrum and coherence between two timestream quantities.

    The cross spectrum rejects everything uncorrelated between the two
    channels. Amplifier noise is isotropic in ``S_21`` and therefore
    uncorrelated between the frequency and dissipation quadratures, and TLS
    noise is private to the frequency channel, so both drop out -- leaving the
    quasiparticle term that is common to both. That makes this a
    **responsivity-independent measurement of photon plus generation-
    recombination noise**, and a sharper test of photon-limited operation than
    fitting either channel's own PSD.

    Parameters
    ----------
    data : dict or ndarray
        A ``parse_samples()`` dict (converted to frequency/dissipation using
        ``sweep_data``, or read directly if already demodulated), or the first
        of two plain arrays, shaped ``(n_tones, n_samples)`` or
        ``(n_samples,)``.
    other : ndarray, optional
        The second array, when ``data`` is an array. Same shape as ``data``.
    sample_rate : float, optional
        Sample rate in Hz. Required for plain arrays.
    tones : sequence of int, optional
        Tone indices. ``None`` (default) uses tone 0 for a dict input.
    sweep_data : dict, optional
        ``parse_sweep_data()`` dict, required to convert a raw I/Q capture.
    psd_kwargs : dict, optional
        ``nperseg``, ``noverlap``, ``window``, ``scaling`` -- applied
        identically to the auto- and cross-spectra, which is what makes the
        coherence meaningful.
    panels : tuple of str, optional
        Names for the two channels in the output. Default
        ``('frequency', 'dissipation')``.
    reference_tone_frequency, smooth_window_hz, conversion_method, calibrations
        As for `timestream_psds`. ``conversion_method='circle'`` is rejected.
    apply_readout_correction : bool, optional
        Default ``True``.
    coherence_warn_segments : int, optional
        Warn when fewer than this many Welch segments were averaged. Default
        30.

    export : str or pathlib.Path, optional
        Write the result to this path with `export_psds` and return it
        unchanged, so this drops into an existing call without
        restructuring anything. ``None`` (default) writes nothing.

    Returns
    -------
    dict
        ``'f'`` : ndarray, ``(n_freq,)`` Hz.
        ``'sample_rate'``, ``'tones'``, ``'panels'`` : provenance.
        ``'psd'`` : dict of ndarray, the two auto-spectra, ``(n_tones, n_freq)``.
        ``'cross'`` : ndarray, complex ``(n_tones, n_freq)``, ``S_xy``.
        ``'coherence'`` : ndarray, magnitude-squared coherence
        ``|S_xy|**2 / (S_xx S_yy)``, in ``[0, 1]``.
        ``'cross_phase'`` : ndarray, ``angle(S_xy)`` in radians.
        ``'matrix'`` : ndarray, complex ``(n_tones, n_freq, 2, 2)``, the
        Hermitian spectral matrix per bin.
        ``'n_avg'`` : int, Welch segments averaged.
        ``'n_eff'`` : float, the *independent*-segment equivalent after
        accounting for the correlation between overlapping segments. This is
        what the two levels below are computed from, and it is what a fit
        should use as its degrees of freedom.
        ``'coherence_bias'`` : float, ``1 / n_eff`` -- the level an
        *uncorrelated* pair reads at. Subtract or compare against it; do not
        read coherence below it as signal.
        ``'coherence_95'`` : float, the 95% significance level under the null
        of zero coherence, ``1 - 0.05 ** (1 / (n_eff - 1))``.
        ``'coherence_significant'`` : ndarray of bool, ``coherence >
        coherence_95``.
        ``'conversion_method'``, ``'psd_kwargs'``, ``'info'`` : provenance.

    Raises
    ------
    ValueError
        If ``other`` is missing for array input, if the two arrays differ in
        shape, if ``sweep_data`` is needed and absent, or if
        ``conversion_method='circle'``.

    Warns
    -----
    UserWarning
        When fewer than ``coherence_warn_segments`` segments were averaged.

    Notes
    -----
    **Coherence is biased upward.** With ``n`` segments an entirely
    uncorrelated pair still reads ``E[gamma2] ~ 1 / n``: 0.5 at 2 segments,
    0.1 at 10, 0.02 at 50. Reading that as photon noise would find a signal
    that is pure estimator bias, which is why ``'coherence_bias'`` and
    ``'coherence_95'`` are returned rather than left to the caller. They use
    ``n_eff`` rather than the raw segment count, because overlapping segments
    are correlated and average down more slowly -- about 5% fewer effective
    segments at the default 50% Hann overlap, which is enough to turn a null
    result into an apparent detection.

    **Non-zero coherence in the readout-dominated band is a fault, not a
    result.** Amplifier noise is isotropic in ``S_21``, hence isotropic in
    ``delta_tilde``, hence uncorrelated between the channels. Coherence there
    means the large ``delta_x`` is leaking into dissipation through a
    de-embedding error -- residual cable delay, rotation error, or a misplaced
    circle centre. Run this on an off-resonance capture as an acceptance test
    before trusting any combination of the two channels.

    **That test needs a long capture to mean anything.** A leak of amplitude
    fraction ``L`` produces only ``gamma2 = L**2 / (1 + L**2)``, so it has to
    clear a significance level of roughly ``3 / n_avg``: detecting it needs
    about ``n_avg > 3 / L**2`` segments. That is ~33 segments for a 30% leak
    but ~300 for 10% and ~1200 for 5%. A short capture returning low coherence
    is therefore not evidence of clean de-embedding -- check ``n_avg`` against
    the leak you care about excluding before reading the test as a pass.

    The dissipation row is ``Im(delta_tilde) = Delta(1 / 2 Q_i)``, with the
    factor of 1/2 already applied by the conversion.

    Examples
    --------
    >>> import numpy as np
    >>> rng = np.random.default_rng(0)
    >>> n = 16384
    >>> common = rng.normal(size=n)            # shared, e.g. quasiparticles
    >>> x = common + rng.normal(size=n)        # plus private noise
    >>> y = common + rng.normal(size=n)
    >>> out = cross_psd(x, y, sample_rate=488.28, psd_kwargs={'nperseg': 1024})
    >>> out['n_avg'] > 30
    True
    >>> bool(0.2 < np.median(out['coherence'][0]) < 0.3)   # analytic 0.25
    True
    >>> bool(abs(np.median(out['cross_phase'][0])) < 0.2)  # in phase
    True

    An uncorrelated pair reads the bias floor, not zero:

    >>> null = cross_psd(rng.normal(size=n), rng.normal(size=n),
    ...                  sample_rate=488.28, psd_kwargs={'nperseg': 1024})
    >>> bool(abs(np.mean(null['coherence'][0]) - null['coherence_bias']) < 0.01)
    True
    """
    psd_kw = dict(psd_kwargs or {})
    info = None
    method = None

    if isinstance(data, Mapping):
        if other is not None:
            raise ValueError(
                "pass either a capture dict or two arrays, not both")
        rows_x, rows_y, idx, rate, method, info = _freq_diss_rows(
            data, sweep_data, tones, reference_tone_frequency,
            smooth_window_hz, conversion_method, calibrations,
            apply_readout_correction)
    else:
        if other is None:
            raise ValueError(
                "cross_psd needs two signals; pass the second as `other`")
        rows_x = np.atleast_2d(np.asarray(data, dtype=float))
        rows_y = np.atleast_2d(np.asarray(other, dtype=float))
        if rows_x.shape != rows_y.shape:
            raise ValueError(
                f"the two inputs must have the same shape, got "
                f"{rows_x.shape} and {rows_y.shape}")
        if sample_rate is None:
            raise ValueError(
                "sample_rate is required when passing plain arrays")
        rate = float(sample_rate)
        idx = np.arange(rows_x.shape[0], dtype=int)
        if tones is not None:
            pick = np.asarray(list(tones), dtype=int)
            rows_x, rows_y, idx = rows_x[pick], rows_y[pick], idx[pick]

    n_samples = rows_x.shape[1]
    nperseg = psd_kw.pop('nperseg', None) or min(n_samples, 256)
    noverlap = psd_kw.pop('noverlap', None)
    if noverlap is None:
        noverlap = nperseg // 2
    kw = dict(fs=rate, nperseg=nperseg, noverlap=noverlap,
              window=psd_kw.pop('window', 'hann'),
              scaling=psd_kw.pop('scaling', 'density'))

    sxx, syy, sxy = [], [], []
    for row_x, row_y in zip(rows_x, rows_y):
        f, pxx = scipy_signal.welch(row_x, **kw)
        _, pyy = scipy_signal.welch(row_y, **kw)
        _, pxy = scipy_signal.csd(row_x, row_y, **kw)
        sxx.append(pxx)
        syy.append(pyy)
        sxy.append(pxy)
    sxx = np.vstack(sxx)
    syy = np.vstack(syy)
    sxy = np.vstack(sxy)

    with np.errstate(divide='ignore', invalid='ignore'):
        coherence = np.abs(sxy) ** 2 / (sxx * syy)
    coherence = np.clip(np.nan_to_num(coherence, nan=0.0), 0.0, 1.0)

    n_avg = int(max(1, (n_samples - noverlap) // max(1, nperseg - noverlap)))
    n_eff = _effective_segments(kw['window'], nperseg, noverlap, n_avg)
    bias = 1.0 / n_eff
    sig95 = 1.0 - 0.05 ** (1.0 / (n_eff - 1)) if n_eff > 1 else 1.0
    if n_avg < coherence_warn_segments:
        warnings.warn(
            f"coherence averaged over only {n_avg} Welch segments "
            f"({n_eff:.1f} independent), so an "
            f"uncorrelated pair already reads {bias:.2f} and anything below "
            f"{sig95:.2f} is not significant. Lengthen the capture or shorten "
            "nperseg before reading a common-noise term off this.",
            stacklevel=2)

    n_tones, n_freq = sxx.shape
    matrix = np.empty((n_tones, n_freq, 2, 2), dtype=complex)
    matrix[..., 0, 0] = sxx
    matrix[..., 0, 1] = sxy
    matrix[..., 1, 0] = np.conj(sxy)
    matrix[..., 1, 1] = syy

    return _maybe_export({
        'f': f,
        'sample_rate': rate,
        'tones': idx,
        'panels': tuple(panels),
        'psd': {panels[0]: sxx, panels[1]: syy},
        'cross': sxy,
        'coherence': coherence,
        'cross_phase': np.angle(sxy),
        'matrix': matrix,
        'n_avg': n_avg,
        'n_eff': n_eff,
        'coherence_bias': float(bias),
        'coherence_95': float(sig95),
        'coherence_significant': coherence > sig95,
        'conversion_method': method,
        'psd_kwargs': {k: v for k, v in kw.items() if k != 'fs'},
        'info': info,
    }, export)
