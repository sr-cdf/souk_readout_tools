"""
Power spectral density computation utilities.
"""

import numpy as np
from scipy import signal as scipy_signal


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


def compute_psd_concatenated(data_2d, sample_rate, **kwargs):
    """
    Concatenate repetitions then compute a single PSD.

    Args:
        data_2d: 2D array, shape (N_reps, N_samples).
        sample_rate: Sample rate in Hz.
        **kwargs: Passed to compute_psd().

    Returns:
        f: 1D frequency array (Hz).
        psd: PSD of the concatenated signal.
    """
    data_2d = np.atleast_2d(data_2d)
    concatenated = data_2d.ravel()
    return compute_psd(concatenated, sample_rate, **kwargs)
