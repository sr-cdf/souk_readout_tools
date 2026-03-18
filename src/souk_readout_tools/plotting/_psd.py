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
