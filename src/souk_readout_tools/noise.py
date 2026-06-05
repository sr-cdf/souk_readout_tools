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
- ``fractional_frequency_and_dissipation_timestreams(ts_data, sweep_data,
  ...)``: Called by notebooks and timestream PSD plotting; takes parsed I/Q
  samples plus regular-tone calibration sweeps and returns selected tone
  indices with calibrated fractional-frequency and dissipation rows.

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
- ``_get_sweep_trace(sweep_data, tone_index)``: Called by calibrated
  conversion; takes a sweep dictionary and tone index and returns one tone's
  frequency, I, and Q sweep arrays.
- ``_resolve_reference_frequency(ts_data, tone_index, sweep_f, override)``:
  Called by calibrated conversion; takes sample metadata and an optional
  override and returns the calibration frequency for one tone.
"""

from collections.abc import Mapping
import operator

import numpy as np


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


def _get_sweep_trace(sweep_data, tone_index):
    """Extract a tone's sweep trace from targeted or wideband sweep data."""
    sweep_f = np.atleast_2d(sweep_data["sweep_f"])
    sweep_i = np.atleast_2d(sweep_data["sweep_i"])
    sweep_q = np.atleast_2d(sweep_data["sweep_q"])

    if sweep_data.get("wideband_sweep", False) or sweep_f.shape[0] == 1:
        return sweep_f[0], sweep_i[0], sweep_q[0]

    n_tones = sweep_f.shape[1]
    if not 0 <= tone_index < n_tones:
        raise ValueError(
            f"Tone {tone_index} not in sweep data ({n_tones} tones)")
    return (
        sweep_f[:, tone_index],
        sweep_i[:, tone_index],
        sweep_q[:, tone_index],
    )


def _resolve_reference_frequency(ts_data, tone_index, sweep_f,
                                 reference_tone_frequency):
    """Resolve the operating frequency used for one tone's calibration."""
    if reference_tone_frequency is None:
        info = ts_data.get("info")
        tones = info.get("tones", {}) if isinstance(info, dict) else {}
        frequencies = tones.get("frequencies_hz")
        if frequencies is not None:
            try:
                return float(np.asarray(frequencies)[tone_index])
            except (IndexError, TypeError, ValueError):
                pass
        import warnings
        warnings.warn(
            f"Tone {tone_index} has no frequency in ts_data['info']; "
            "falling back to the midpoint of the sweep range. Pass "
            "reference_tone_frequency to set this explicitly.",
            stacklevel=3,
        )
        return float(np.mean(sweep_f))

    if isinstance(reference_tone_frequency, Mapping):
        for key in (tone_index, f"{tone_index:04d}"):
            if key in reference_tone_frequency:
                return float(reference_tone_frequency[key])
        raise ValueError(
            f"reference_tone_frequency mapping missing tone {tone_index}")

    if np.ndim(reference_tone_frequency):
        return float(np.asarray(reference_tone_frequency)[tone_index])

    return float(reference_tone_frequency)


def fractional_frequency_and_dissipation_timestreams(
        ts_data, sweep_data, tones=None, reference_tone_frequency=None,
        smooth_window_hz=1000, method="linearized", calibrations=None):
    """
    Convert parsed I/Q timestreams into calibrated fractional shifts.

    Parameters
    ----------
    ts_data : dict
        Parsed accumulated samples from ``ReadoutClient.parse_samples()``.
    sweep_data : dict
        Matching calibration sweep.
    tones : iterable of int or None, optional
        Tone indices to convert. ``None`` converts every active tone.
    reference_tone_frequency : scalar, array-like, mapping, or None, optional
        Calibration frequency or frequencies. ``None`` uses frequencies from
        ``ts_data['info']['tones']['frequencies_hz']``. For an off-resonance
        capture, pass the corresponding on-resonance frequencies explicitly.
    smooth_window_hz : float or None, optional
        Sweep smoothing window used by ``method="linearized"``.
    method : {"linearized", "mobius", "circle"}, optional
        IQ conversion method. ``"linearized"`` preserves the historical local
        tangent/normal estimate: its loss row is the matched-scale quadrature
        ``Delta(1 / (2 * Qi))`` in the symmetric small-signal limit.
        ``"mobius"`` uses fitted-model frequency inversion and returns the
        matched-scale ``Delta(1 / (2 * Qi))`` loss coordinate.
        ``"circle"`` exposes the effective frequency-circle coordinate for
        diagnostics; its loss row is the change in signed radial proxy
        ``Delta(abs(z_centered) / radius - 1)``.
    calibrations : mapping or sequence, optional
        Per-tone ``resonator.ResonatorCalibration`` objects or fit results.
        Required by ``method="mobius"`` and ``method="circle"``.

    Returns
    -------
    dict
        ``tone_indices``, ``frequency``, and ``dissipation``. The two
        timestream arrays are shaped ``(n_tones, n_samples)``. The
        ``dissipation`` key is retained for compatibility; its coordinate
        convention depends on ``method`` as described above. All rows are
        changes relative to the sweep IQ at the fixed probe frequency.
        Unlike the low-level fitted converters, which return probe detuning
        ``f_probe - f_r``, this helper returns resonator motion
        ``Delta(f_r - f_probe)`` by reference-subtracting and sign-flipping the
        fitted frequency coordinate.
    """
    from souk_readout_tools.resonator import (
        calibration_for_tone,
        interpolate_complex_trace,
        linearized_frequency_and_dissipation,
    )

    if method not in ("linearized", "mobius", "circle"):
        raise ValueError("method must be 'linearized', 'mobius', or 'circle'")

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

    first_tone_key = selected[0][1]
    n_samples = len(ts_data["i_data"][first_tone_key])
    frequency = np.empty((len(selected), n_samples), dtype=float)
    dissipation = np.empty_like(frequency)
    for row, (tone_index, tone_key) in enumerate(selected):
        sweep_f, sweep_i, sweep_q = _get_sweep_trace(sweep_data, tone_index)
        tone_frequency = _resolve_reference_frequency(
            ts_data, tone_index, sweep_f, reference_tone_frequency)
        timestream_z = (
            np.asarray(ts_data["i_data"][tone_key], dtype=float)
            + 1j * np.asarray(ts_data["q_data"][tone_key], dtype=float)
        )
        if method == "linearized":
            frac_f, frac_d = linearized_frequency_and_dissipation(
                sweep_f,
                sweep_i + 1j * sweep_q,
                tone_frequency,
                timestream_z,
                smooth_window_hz=smooth_window_hz,
            )
        else:
            calibration = calibration_for_tone(calibrations, tone_index)
            if calibration is None:
                raise ValueError(
                    f"method={method!r} requires a calibration for "
                    f"tone {tone_index}")
            sweep_z = sweep_i + 1j * sweep_q
            reference_s21 = interpolate_complex_trace(
                sweep_f, sweep_z, tone_frequency)
            df_hz, frac_d = calibration.convert_referenced_raw_iq(
                tone_frequency, timestream_z, reference_s21, method=method)
            frac_f = df_hz / tone_frequency
        try:
            frequency[row] = frac_f
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
    "remove_blind_tone_common_modes",
    "remove_common_modes_svd",
]
