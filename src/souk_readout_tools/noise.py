"""
Noise-analysis utilities for calibrated multi-tone timestreams.

Function map
------------
Public API:
- ``remove_common_modes_svd(data, ...)``: Called by notebooks and timestream
  PSD plotting; takes real ``(channels, samples)`` rows and returns SVD-cleaned
  rows, optionally with diagnostics.
- ``remove_blind_tone_common_modes(ts_data, ...)``: Called by notebooks and
  timestream PSD plotting; takes parsed simultaneous I/Q samples and returns a
  shallow copy with selected regular-tone I/Q rows cleaned using blind
  amplitude/phase modes, optionally with diagnostics.
- ``remove_iq_common_modes_svd(ts_data, ...)``: Called by notebooks and
  timestream PSD plotting; takes parsed simultaneous I/Q samples and returns a
  shallow copy with the selected tone rows cleaned of leading SVD common modes
  fitted from the tones' own amplitude/phase variations, optionally with
  diagnostics. Needs no resonance sweep, so it also works on loopback data.
- ``fractional_frequency_and_dissipation_timestreams(ts_data, sweep_data,
  ...)``: Called by notebooks and timestream PSD plotting; takes parsed I/Q
  samples plus regular-tone calibration sweeps and returns selected tone
  indices with calibrated fractional-frequency and dissipation rows. A
  capture-level wrapper round the conversions in ``resonator``.
- ``rx_gain_ratio(sweep_data, ts_data, ...)``: re-exported from
  ``resonator``; the per-tone amplitude ratio between a timestream and a sweep
  taken at different ADC drive. Feeds
  ``fractional_frequency_and_dissipation_timestreams(match_rx_gain=...)``.

Private helpers:
- ``_remove_temporal_modes(data, temporal_modes)``: Called by blind-tone
  cleaning; takes real rows and orthonormal temporal modes and returns cleaned
  rows plus fitted coefficients.
- ``_tone_metadata(ts_data)``: Called by blind-tone cleaning; takes parsed
  samples and returns saved regular/blind role metadata when available.
- ``_resolve_tone_indices(ts_data, tones, label)``: Called by complex-row
  extraction; takes parsed samples and requested tone indices and returns
  validated ``(index, key)`` pairs.
- ``_complex_tone_rows(ts_data, tones, label)``: Called by blind-reference
  fitting and regular-tone cleaning; takes parsed I/Q samples and returns a
  complex row matrix, numeric indices, and validated key pairs.
- ``_amplitude_phase_variations(z)``: Called by blind-reference fitting and
  regular-tone cleaning; takes complex rows and returns amplitude/phase
  baselines and normalized variations.
- ``_blind_tone_temporal_modes(ts_data, ...)``: Called by blind-tone cleaning;
  takes parsed samples and selected blind monitors and returns fitted
  amplitude/phase temporal-mode diagnostics.

Sweep traces, tone frequencies, RX gain matching and the filterbank readout
correction are in ``resonator`` (``sweep_trace``, ``resolve_tone_frequency``,
``rx_gain_ratio``, ``with_readout_correction``).
"""

import operator

import numpy as np

# Sweep traces, tone frequencies, RX gain matching and the readout correction
# live with the rest of the conversion contract in resonator.py.
from .resonator import rx_gain_ratio


def remove_common_modes_svd(data, n_modes=1, *, standardise=True,
                            return_info=False):
    """
    Remove leading common modes from channel timestreams using an SVD.

    Parameters
    ----------
    data : array-like
        Finite 2-D array shaped ``(n_channels, n_samples)``.
    n_modes : int, optional
        Number of leading SVD modes to subtract. The default is ``1``.
        Pass ``0`` to return an unchanged copy.
    standardise : bool, optional
        Divide each channel by its standard deviation before the SVD, then
        restore the original scale afterwards. This prevents the noisiest
        channel from dominating the decomposition. The default is ``True``.
    return_info : bool, optional
        Also return decomposition diagnostics. The default is ``False``.

    Returns
    -------
    cleaned : numpy.ndarray
        Cleaned timestreams with the same shape as ``data``.
    info : dict, optional
        Returned only when ``return_info=True``. Contains ``n_modes``,
        ``scales``, ``singular_values``, ``explained_variance_ratio``,
        ``channel_modes``, and ``temporal_modes``.

    Notes
    -----
    Means are preserved: only time-varying components are included in the
    decomposition. Clean or interpolate missing values before calling this
    function; NaN and infinite values are rejected explicitly.
    """
    values = np.asarray(data, dtype=float)
    if values.ndim != 2:
        raise ValueError(
            "data must be a 2-D array shaped (n_channels, n_samples)")
    if 0 in values.shape:
        raise ValueError("data must contain at least one channel and sample")
    if not np.all(np.isfinite(values)):
        raise ValueError("data must contain only finite values")

    try:
        n_modes = operator.index(n_modes)
    except TypeError as exc:
        raise TypeError("n_modes must be an integer") from exc

    max_modes = min(values.shape)
    if not 0 <= n_modes <= max_modes:
        raise ValueError(
            f"n_modes must be between 0 and {max_modes}, got {n_modes}")

    scales = np.ones(values.shape[0], dtype=float)
    if standardise:
        scales = np.std(values, axis=1)
        scales[scales == 0] = 1.0

    normalised = values / scales[:, np.newaxis]
    offsets = np.mean(normalised, axis=1, keepdims=True)
    centered = normalised - offsets
    # Decomposing the tall orientation is substantially faster with LAPACK.
    if centered.shape[0] <= centered.shape[1]:
        temporal_modes_t, singular_values, channel_modes_t = np.linalg.svd(
            centered.T, full_matrices=False)
        channel_modes = channel_modes_t.T
        temporal_modes = temporal_modes_t.T
    else:
        channel_modes, singular_values, temporal_modes = np.linalg.svd(
            centered, full_matrices=False)

    if n_modes:
        reconstructed = (
            channel_modes[:, :n_modes] * singular_values[:n_modes]
        ) @ temporal_modes[:n_modes]
        cleaned = (normalised - reconstructed) * scales[:, np.newaxis]
    else:
        cleaned = values.copy()

    if not return_info:
        return cleaned

    variance = singular_values ** 2
    variance_sum = np.sum(variance)
    explained_variance_ratio = (
        variance / variance_sum if variance_sum else np.zeros_like(variance)
    )
    info = {
        "n_modes": n_modes,
        "scales": scales,
        "singular_values": singular_values,
        "explained_variance_ratio": explained_variance_ratio,
        "channel_modes": channel_modes,
        "temporal_modes": temporal_modes,
    }
    return cleaned, info


def _remove_temporal_modes(data, temporal_modes):
    """Subtract orthonormal temporal modes from rows while preserving means."""
    values = np.asarray(data, dtype=float)
    if values.ndim != 2:
        raise ValueError(
            "data must be a 2-D array shaped (n_channels, n_samples)")
    if 0 in values.shape:
        raise ValueError("data must contain at least one channel and sample")
    if not np.all(np.isfinite(values)):
        raise ValueError("data must contain only finite values")

    temporal_modes = np.asarray(temporal_modes, dtype=float)
    if temporal_modes.ndim != 2:
        raise ValueError(
            "temporal_modes must be a 2-D array shaped (n_modes, n_samples)")
    if temporal_modes.shape[1] != values.shape[1]:
        raise ValueError(
            "data and temporal_modes must contain the same number of samples")

    centered = values - np.mean(values, axis=1, keepdims=True)
    coefficients = centered @ temporal_modes.T
    return values - coefficients @ temporal_modes, coefficients


def _tone_metadata(ts_data):
    """Return saved role metadata for parsed timestream data."""
    info = ts_data.get("info")
    tones = info.get("tones", {}) if isinstance(info, dict) else {}
    if tones:
        return tones
    return ts_data.get("tone_metadata", {})


def _resolve_tone_indices(ts_data, tones, label):
    """Validate tone indices and return matching dictionary keys."""
    i_data = ts_data["i_data"]
    tone_keys = sorted(i_data)

    selected = []
    for tone_index in tones:
        try:
            tone_index = operator.index(tone_index)
            tone_key = tone_keys[tone_index]
        except (IndexError, TypeError) as exc:
            raise ValueError(
                f"{label} tone {tone_index} not in timestream data "
                f"({len(tone_keys)} tones)") from exc
        if tone_index < 0:
            raise ValueError(
                f"{label} tone {tone_index} not in timestream data "
                f"({len(tone_keys)} tones)")
        selected.append((tone_index, tone_key))
    return selected


def _complex_tone_rows(ts_data, tones, label):
    """Return selected complex timestream rows and their tone indices."""
    i_data = ts_data["i_data"]
    q_data = ts_data["q_data"]
    selected = _resolve_tone_indices(ts_data, tones, label)
    if not selected:
        raise ValueError(f"No {label.lower()} tones selected")

    n_samples = len(i_data[selected[0][1]])
    rows = np.empty((len(selected), n_samples), dtype=complex)
    for row, (tone_index, tone_key) in enumerate(selected):
        try:
            rows[row] = (
                np.asarray(i_data[tone_key], dtype=float)
                + 1j * np.asarray(q_data[tone_key], dtype=float)
            )
        except ValueError as exc:
            raise ValueError(
                f"{label} tone {tone_index} has an inconsistent sample count"
            ) from exc
    if not np.all(np.isfinite(rows)):
        raise ValueError(f"{label}-tone I/Q data must contain only finite values")

    return rows, np.asarray([tone_index for tone_index, _ in selected]), selected


def _amplitude_phase_variations(z):
    """Return baselines and variations for complex timestream rows."""
    amplitude = np.abs(z)
    amplitude_baseline = np.mean(amplitude, axis=1, keepdims=True)
    if np.any(amplitude_baseline == 0):
        raise ValueError("Tone timestreams must have non-zero mean amplitude")
    amplitude_variation = amplitude / amplitude_baseline - 1.0

    phase = np.unwrap(np.angle(z), axis=1)
    phase_baseline = np.mean(phase, axis=1, keepdims=True)
    phase_variation = phase - phase_baseline
    return (
        amplitude_baseline,
        phase_baseline,
        amplitude_variation,
        phase_variation,
    )


def _blind_tone_temporal_modes(
        ts_data, n_modes, blind_tones, standardise):
    """Fit temporal modes from normalized blind amplitude/phase variations."""
    if blind_tones is None:
        blind_tones = _tone_metadata(ts_data).get("blind_indices")
    if blind_tones is None or len(blind_tones) == 0:
        raise ValueError(
            "No blind tones found. Pass blind_tones explicitly or capture "
            "timestream metadata containing info['tones']['blind_indices'].")

    blind_z, blind_tones, _ = _complex_tone_rows(
        ts_data, blind_tones, "Blind")
    _, _, amplitude_variation, phase_variation = (
        _amplitude_phase_variations(blind_z)
    )
    references = np.concatenate([amplitude_variation, phase_variation])
    scales = np.ones(references.shape[0], dtype=float)
    if standardise:
        scales = np.std(references, axis=1)
        scales[scales == 0] = 1.0
    references = references / scales[:, np.newaxis]

    if references.shape[0] <= references.shape[1]:
        _, singular_values, temporal_modes = np.linalg.svd(
            references, full_matrices=False)
    else:
        temporal_modes_t, singular_values, _ = np.linalg.svd(
            references.T, full_matrices=False)
        temporal_modes = temporal_modes_t.T

    tolerance = (
        max(references.shape) * np.finfo(float).eps * singular_values[0]
        if len(singular_values) else 0.0
    )
    rank = int(np.sum(singular_values > tolerance))
    if n_modes > rank:
        raise ValueError(
            f"Requested {n_modes} blind-tone modes, but the selected blind "
            f"tones contain only {rank} non-zero amplitude/phase modes")

    variance = singular_values ** 2
    variance_sum = np.sum(variance)
    explained_variance_ratio = (
        variance / variance_sum if variance_sum else np.zeros_like(variance)
    )
    return {
        "n_modes": n_modes,
        "blind_tones": blind_tones,
        "reference_scales": scales,
        "singular_values": singular_values,
        "explained_variance_ratio": explained_variance_ratio,
        "temporal_modes": temporal_modes,
    }


def remove_blind_tone_common_modes(
        ts_data, n_modes=1, *, regular_tones=None, blind_tones=None,
        standardise=True,
        return_info=False):
    """
    Remove blind-monitor amplitude/phase modes from regular-tone I/Q data.

    Temporal reference modes are fitted only from normalized amplitude and
    unwrapped-phase variations in the selected blind monitors. Their
    amplitudes are fitted and subtracted independently from the corresponding
    variations in each selected regular tone. The cleaned regular-tone I/Q
    timestreams can then be converted to frequency/dissipation shifts using a
    detector resonance sweep.

    Parameters
    ----------
    ts_data : dict
        Parsed accumulated samples containing simultaneous blind-tone I/Q
        rows and, normally, ``info['tones']['blind_indices']`` metadata.
    n_modes : int, optional
        Number of leading blind-reference modes to subtract. The default is
        ``1``. Pass ``0`` to return an unchanged copy.
    regular_tones : iterable of int or None, optional
        Regular-tone indices to clean. ``None`` infers them from ``ts_data``
        metadata, falling back to all non-blind tones.
    blind_tones : iterable of int or None, optional
        Explicit blind-tone indices. ``None`` infers them from ``ts_data``.
    standardise : bool, optional
        Scale each blind amplitude/phase reference row by its standard
        deviation before fitting the reference modes. The default is ``True``.
    return_info : bool, optional
        Also return the fitted mode diagnostics. The default is ``False``.

    Returns
    -------
    cleaned : dict
        Shallow copy of ``ts_data`` with cleaned I/Q arrays for the selected
        regular tones. Blind-monitor rows and other metadata are preserved.
    info : dict, optional
        Returned only when ``return_info=True``. Contains ``n_modes``,
        ``blind_tones``, ``reference_scales``, ``singular_values``,
        ``explained_variance_ratio``, ``temporal_modes``, and
        amplitude/phase ``coefficients``.

    Notes
    -----
    The blind tones do not require resonance sweeps or frequency/dissipation
    calibration. Their sweep traces are not used.
    """
    try:
        n_modes = operator.index(n_modes)
    except TypeError as exc:
        raise TypeError("n_modes must be an integer") from exc
    if n_modes < 0:
        raise ValueError("n_modes must be non-negative")
    if n_modes == 0:
        cleaned = dict(ts_data)
        cleaned["i_data"] = dict(ts_data["i_data"])
        cleaned["q_data"] = dict(ts_data["q_data"])
        if return_info:
            return cleaned, {"n_modes": 0}
        return cleaned

    info = _blind_tone_temporal_modes(
        ts_data, n_modes, blind_tones, standardise)
    blind_tones = info["blind_tones"]
    if regular_tones is None:
        regular_tones = _tone_metadata(ts_data).get("regular_indices")
        if regular_tones is None or len(regular_tones) == 0:
            blind_set = set(blind_tones)
            regular_tones = [
                tone_index for tone_index in range(len(ts_data["i_data"]))
                if tone_index not in blind_set
            ]

    regular_z, regular_tones, selected = _complex_tone_rows(
        ts_data, regular_tones, "Regular")
    if set(regular_tones) & set(blind_tones):
        raise ValueError("regular_tones and blind_tones must not overlap")

    amplitude_baseline, phase_baseline, amplitude_variation, phase_variation = (
        _amplitude_phase_variations(regular_z)
    )
    temporal_modes = info["temporal_modes"][:n_modes]
    cleaned_amplitude_variation, amplitude_coefficients = (
        _remove_temporal_modes(amplitude_variation, temporal_modes)
    )
    cleaned_phase_variation, phase_coefficients = _remove_temporal_modes(
        phase_variation, temporal_modes)

    cleaned_amplitude = amplitude_baseline * (
        1.0 + cleaned_amplitude_variation)
    if np.any(cleaned_amplitude <= 0):
        raise ValueError(
            "Blind-tone cleaning produced a non-positive regular-tone "
            "amplitude; reduce n_modes or inspect the selected monitors")
    cleaned_phase = phase_baseline + cleaned_phase_variation
    cleaned_z = cleaned_amplitude * np.exp(1j * cleaned_phase)

    cleaned = dict(ts_data)
    cleaned["i_data"] = dict(ts_data["i_data"])
    cleaned["q_data"] = dict(ts_data["q_data"])
    for row, (_, tone_key) in enumerate(selected):
        cleaned["i_data"][tone_key] = cleaned_z[row].real
        cleaned["q_data"][tone_key] = cleaned_z[row].imag

    if not return_info:
        return cleaned

    info["regular_tones"] = regular_tones
    info["amplitude_coefficients"] = amplitude_coefficients
    info["phase_coefficients"] = phase_coefficients
    return cleaned, info


def _default_regular_tones(ts_data):
    """Return saved regular-tone indices, else every non-blind tone."""
    metadata = _tone_metadata(ts_data)
    regular_tones = metadata.get("regular_indices")
    if regular_tones is not None and len(regular_tones):
        return regular_tones
    blind_tones = metadata.get("blind_indices") or []
    blind_set = {operator.index(tone_index) for tone_index in blind_tones}
    return [tone_index for tone_index in range(len(ts_data["i_data"]))
            if tone_index not in blind_set]


def remove_iq_common_modes_svd(ts_data, n_modes=1, *, tones=None,
                               standardise=True, return_info=False):
    """
    Remove leading common modes from tone I/Q data using an SVD.

    Modes are fitted from the tones' own normalized amplitude and unwrapped-
    phase variations -- the coordinates a common gain drift or LO phase drift
    moves -- and are removed from the amplitude and phase rows separately, in
    the same way ``remove_common_modes_svd`` treats calibrated
    frequency/dissipation rows. The cleaned rows are recombined into I/Q.

    Unlike the frequency/dissipation cleaning, this needs no resonance sweep,
    so it applies to loopback captures and other data without resonators.

    Parameters
    ----------
    ts_data : dict
        Parsed accumulated samples with simultaneous tone I/Q rows.
    n_modes : int, optional
        Number of leading SVD modes to subtract. The default is ``1``.
        Pass ``0`` to return an unchanged copy.
    tones : iterable of int or None, optional
        Tone indices used to estimate the modes and cleaned. ``None``
        (default) uses regular tones from the saved metadata when available,
        otherwise every non-blind tone.
    standardise : bool, optional
        Divide each amplitude/phase row by its standard deviation before the
        SVD, then restore the original scale. The default is ``True``.
    return_info : bool, optional
        Also return the decomposition diagnostics. The default is ``False``.

    Returns
    -------
    cleaned : dict
        Shallow copy of ``ts_data`` with cleaned I/Q arrays for the selected
        tones. Unselected rows and other metadata are preserved.
    info : dict, optional
        Returned only when ``return_info=True``. Contains ``n_modes``,
        ``tones``, and the ``amplitude`` and ``phase`` diagnostics returned by
        ``remove_common_modes_svd``.

    Notes
    -----
    A common mode fitted from the tones themselves also contains any signal
    that is common to them, so keep ``n_modes`` small and compare against the
    uncleaned spectrum.
    """
    try:
        n_modes = operator.index(n_modes)
    except TypeError as exc:
        raise TypeError("n_modes must be an integer") from exc
    if n_modes < 0:
        raise ValueError("n_modes must be non-negative")

    if tones is None:
        tones = _default_regular_tones(ts_data)
    z, tones, selected = _complex_tone_rows(ts_data, tones, "Regular")

    cleaned = dict(ts_data)
    cleaned["i_data"] = dict(ts_data["i_data"])
    cleaned["q_data"] = dict(ts_data["q_data"])
    if n_modes == 0:
        if return_info:
            return cleaned, {"n_modes": 0, "tones": tones}
        return cleaned

    (amplitude_baseline, phase_baseline, amplitude_variation,
     phase_variation) = _amplitude_phase_variations(z)
    cleaned_amplitude_variation, amplitude_info = remove_common_modes_svd(
        amplitude_variation, n_modes, standardise=standardise,
        return_info=True)
    cleaned_phase_variation, phase_info = remove_common_modes_svd(
        phase_variation, n_modes, standardise=standardise, return_info=True)

    cleaned_amplitude = amplitude_baseline * (
        1.0 + cleaned_amplitude_variation)
    if np.any(cleaned_amplitude <= 0):
        raise ValueError(
            "SVD cleaning produced a non-positive tone amplitude; reduce "
            "n_modes or inspect the selected tones")
    cleaned_z = cleaned_amplitude * np.exp(
        1j * (phase_baseline + cleaned_phase_variation))

    for row, (_, tone_key) in enumerate(selected):
        cleaned["i_data"][tone_key] = cleaned_z[row].real
        cleaned["q_data"][tone_key] = cleaned_z[row].imag

    if not return_info:
        return cleaned
    info = {
        "n_modes": n_modes,
        "tones": tones,
        "amplitude": amplitude_info,
        "phase": phase_info,
    }
    return cleaned, info


def fractional_frequency_and_dissipation_timestreams(
        ts_data, sweep_data, tones=None, reference_tone_frequency=None,
        smooth_window_hz=1000, method="linearized", calibrations=None,
        match_rx_gain="auto"):
    """
    Convert parsed I/Q timestreams into calibrated fractional shifts.

    A capture-level wrapper round ``resonator.prepare_conversion`` /
    ``resonator.convert_iq``: every method returns resonator motion (positive
    when the resonance moves up) and the matched dissipation
    ``Delta(1 / (2 * Qi))``, both relative to the sweep's I/Q at the tone.

    Parameters
    ----------
    ts_data : dict
        Parsed accumulated samples from ``ReadoutClient.parse_samples()``.
    sweep_data : dict
        Matching calibration sweep.
    tones : iterable of int or None, optional
        Tone indices to convert. ``None`` converts every active tone.
    reference_tone_frequency : scalar, array-like, mapping, or None, optional
        Tone frequency or frequencies (``resonator.resolve_tone_frequency``).
        ``None`` uses ``ts_data['info']['tones']['frequencies_hz']``. For an
        off-resonance capture, pass the on-resonance frequencies explicitly.
    smooth_window_hz : float or None, optional
        Sweep smoothing for the sweep-based methods.
    method : {"linearized", "phase", "mobius", "lookup", "circle"}, optional
        See ``resonator.prepare_conversion``. ``"linearized"`` (default) needs
        only the sweep; ``"phase"``, ``"mobius"`` and ``"circle"`` need
        ``calibrations``; ``"lookup"`` uses them for its centring if given.
        ``"circle"`` is diagnostic: its dissipation row is the radial proxy
        ``Delta(abs(z_centered) / radius)``.
    calibrations : record, mapping or sequence, optional
        Per-tone ``resonator.ResonatorCalibration`` objects, fit results or a
        calibration record.
    match_rx_gain : {"auto", True, False}, float or array-like, optional
        Bring the timestream onto the sweep's RX scale, for when the two were
        taken at different ADC drive. ``"auto"`` (default) uses the recorded
        RX chain states when both are known and differ (with a warning);
        ``True`` measures the carrier ratio with ``rx_gain_ratio`` (assumes the
        resonance did not move); ``False`` does nothing; a scalar or per-tone
        array is the timestream/sweep amplitude ratio. The timestream is
        divided by the ratio, so a fitted calibration stays valid. This
        corrects RX gain only -- a sweep at a different *tone power* is a
        different resonance and cannot be rescaled.

    Returns
    -------
    dict
        ``tone_indices``, ``frequency`` (fractional resonance shift) and
        ``dissipation``, the two arrays shaped ``(n_tones, n_samples)``.
    """
    from souk_readout_tools.resonator import (
        CONVERSION_METHODS,
        ResonatorCalibration,
        calibration_for_tone,
        convert_iq,
        is_calibration_record,
        prepare_conversion,
        resolve_rx_gain,
        resolve_tone_frequency,
        sweep_trace,
        warn_adc_overvoltage,
    )

    if method not in CONVERSION_METHODS:
        raise ValueError(f"method must be one of {CONVERSION_METHODS}")
    warn_adc_overvoltage(samples=ts_data)

    tone_keys = sorted(ts_data["i_data"])
    if tones is None:
        tones = range(len(tone_keys))

    selected = []
    for selection_index in tones:
        try:
            selection_index = operator.index(selection_index)
            tone_key = tone_keys[selection_index]
        except (IndexError, TypeError) as exc:
            raise ValueError(
                f"Tone {selection_index} not in timestream data "
                f"({len(tone_keys)} tones)") from exc
        if selection_index < 0:
            raise ValueError(
                f"Tone {selection_index} not in timestream data "
                f"({len(tone_keys)} tones)")
        selected.append((int(tone_key), tone_key))
    if not selected:
        raise ValueError("tones must select at least one timestream tone")

    if match_rx_gain is True:
        gain = rx_gain_ratio(
            sweep_data, ts_data, tones=[index for index, _ in selected])["ratio"]
    elif match_rx_gain in (False, None, "auto"):
        source = calibrations if is_calibration_record(calibrations) else sweep_data
        gain = np.full(len(selected), resolve_rx_gain(
            match_rx_gain, source, ts_data.get("info")))
    else:
        gain = np.asarray(match_rx_gain, dtype=float).ravel()
        if gain.size == 1:
            gain = np.full(len(selected), float(gain[0]))
        if gain.size != len(selected):
            raise ValueError(
                "match_rx_gain must be 'auto', True, False, a scalar, or one "
                "ratio per selected tone.")
    gain = np.where(np.isfinite(gain) & (gain > 0), gain, 1.0)

    first_tone_key = selected[0][1]
    n_samples = len(ts_data["i_data"][first_tone_key])
    frequency = np.empty((len(selected), n_samples), dtype=float)
    dissipation = np.empty_like(frequency)
    for row, (tone_index, tone_key) in enumerate(selected):
        sweep_f, sweep_i, sweep_q = sweep_trace(sweep_data, tone_index)
        tone_frequency = resolve_tone_frequency(
            ts_data, tone_index, reference_tone_frequency, sweep_f)
        calibration = calibration_for_tone(calibrations, tone_index)
        if not isinstance(calibration, ResonatorCalibration):
            calibration = None
        if calibration is None and method in ("phase", "mobius", "circle"):
            raise ValueError(
                f"method={method!r} requires a calibration for tone {tone_index}")
        prepared = prepare_conversion(
            tone_frequency, method, calibration=calibration, sweep_f=sweep_f,
            sweep_z=np.asarray(sweep_i) + 1j * np.asarray(sweep_q),
            smooth_window_hz=smooth_window_hz)
        timestream_z = (
            np.asarray(ts_data["i_data"][tone_key], dtype=float)
            + 1j * np.asarray(ts_data["q_data"][tone_key], dtype=float))
        shift_hz, frac_d = convert_iq(prepared, timestream_z, gain[row])
        try:
            frequency[row] = shift_hz / tone_frequency
            dissipation[row] = frac_d
        except ValueError as exc:
            raise ValueError(
                f"Tone {tone_index} has a different sample count from "
                f"tone {int(first_tone_key)}") from exc

    return {
        "tone_indices": np.asarray([tone_index for tone_index, _ in selected]),
        "frequency": frequency,
        "dissipation": dissipation,
    }


__all__ = [
    "fractional_frequency_and_dissipation_timestreams",
    "rx_gain_ratio",
    "remove_blind_tone_common_modes",
    "remove_common_modes_svd",
    "remove_iq_common_modes_svd",
]
