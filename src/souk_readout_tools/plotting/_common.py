"""
Shared utilities for the plotting subpackage.
"""

import os
import warnings

import numpy as np


# Supported units for normalisation
UNITS = ('raw', 'peak', 'adc_fs', 'dbfs', 'dbm')

# Reference planes the plot-time calibration can report at when units='dbm'.
#   'adc_input'       — power at the ADC input (requires firmware.adc_dbm_to_dbfs)
#   'cryostat_output' — power at the cryostat output, i.e. at the RX port of
#                       the cryostat, deembedding the RX analog chain
#                       (requires rf_frontend + cryostat calibration entries)
VALID_REFERENCE_PLANES = ('adc_input', 'cryostat_output', 'detector')
_REFERENCE_PLANE_LABELS = {
    'adc_input': 'ADC input',
    'cryostat_output': 'cryostat output',
    'detector': 'detector',
}

# Directory that frequency-dependent cal files (referenced by filename in the
# config) live in.  Must match firmware_lib.USER_DIR.
_CAL_USER_DIR = os.path.expanduser('~/.souk_readout_tools/')


def _resolve_cal_value(value, frequencies=None):
    """Resolve a calibration entry to a scalar or per-frequency array.

    Mirrors firmware_lib._resolve_cal_value but lives here so the plotting
    subpackage doesn't have to import firmware_lib (which pulls in the
    souk_mkid_readout runtime dependency).

    Supported forms:
      - None          → None
      - scalar        → float (no frequencies needed)
      - [[f, dB], …]  → per-sample nearest-neighbour, shape matches frequencies
      - str           → CSV filename under ~/.souk_readout_tools/, same
                        nearest-neighbour interpolation

    Returns None when the value is array-like / file-backed but no
    frequency axis has been provided — callers should treat that as
    "calibration unavailable at plot time".
    """
    if value is None:
        return None
    if isinstance(value, (int, float, np.integer, np.floating)):
        return float(value)
    if frequencies is None:
        return None
    frequencies = np.asarray(frequencies, dtype=float)
    if isinstance(value, str):
        cal_path = os.path.join(_CAL_USER_DIR, value)
        if not os.path.isfile(cal_path):
            warnings.warn(
                f"Calibration file '{cal_path}' not found — "
                "calibration unavailable at plot time.",
                RuntimeWarning, stacklevel=3)
            return None
        cal_f, cal_db = np.loadtxt(cal_path, ndmin=2).T
    else:
        cal_f, cal_db = np.array(value, ndmin=2).T
    flat = frequencies.ravel()
    resolved = np.array([cal_db[np.argmin(np.abs(cal_f - f))] for f in flat])
    return resolved.reshape(frequencies.shape)


def _cal_or_zero(value, frequencies=None):
    """_resolve_cal_value but maps None → 0.0 (scalar)."""
    resolved = _resolve_cal_value(value, frequencies)
    return 0.0 if resolved is None else resolved


def _resolve_adc_dbm_to_dbfs(config, frequencies=None):
    """Return the ADC dBFS→dBm offset, or None if unavailable.

    Reads ``config['firmware']['adc_dbm_to_dbfs']``.  When the entry is a
    scalar the result is a single float; when it's frequency-dependent
    (array / CSV filename) and ``frequencies`` is provided, the result is
    a per-sample array.  Returns None when the entry is missing, None, or
    frequency-dependent without a frequency axis to interpolate onto.
    """
    if config is None:
        return None
    val = config.get('firmware', {}).get('adc_dbm_to_dbfs')
    return _resolve_cal_value(val, frequencies)


def _rx_chain_gain_db(frequencies, info, config):
    """Per-sample RX analog chain gain from cryostat output to ADC input (dB).

    Subtracting this from a power at the ADC input gives the equivalent
    power at the cryostat output.  Sign conventions match
    ``calibration.calc_adc_input_power``: S21 values add to the forward
    gain, explicit loss/attenuator values subtract via their absolute
    magnitude.  The ADC DSA value is read from
    ``info['dsa']`` (captured at sweep/timestream time).

    Frequency-dependent cal entries (CSV filename / [[f, dB], …]) are
    resolved at each entry of ``frequencies``; scalar entries broadcast.

    Returns 0.0 when neither rf_frontend nor cryostat is marked
    ``connected: true``; returns None when the config is missing.
    """
    if config is None:
        return None
    rf = config.get('rf_frontend', {}) or {}
    cryo = config.get('cryostat', {}) or {}
    rf_connected = bool(rf.get('connected', False))
    cryo_connected = bool(cryo.get('connected', False))
    if not (rf_connected or cryo_connected):
        return 0.0

    rx_rf_s21 = 0.0
    rx_if_s21 = 0.0
    rx_bypass_amp_s21 = 0.0
    rx_mixer_conv = 0.0
    rx_combiner = 0.0
    rx_atten = 0.0
    cryo_output_s21 = 0.0

    if rf_connected:
        rx_rf_s21 = _cal_or_zero(rf.get('rx_rf_s21_db'), frequencies)
        rx_if_s21 = _cal_or_zero(rf.get('rx_if_s21_db'), frequencies)
        rx_bypass_amp_s21 = _cal_or_zero(rf.get('rx_bypass_amp_s21_db'), frequencies)
        rx_mixer_conv = _cal_or_zero(rf.get('rx_mixer_conversion_loss_db'), frequencies)
        rx_combiner = _cal_or_zero(rf.get('rx_combiner_loss_db'), frequencies)
        rx_atten_raw = rf.get('rx_attenuator_value_db')
        rx_atten = 0.0 if rx_atten_raw is None else float(rx_atten_raw)
    if cryo_connected:
        cryo_output_s21 = _cal_or_zero(cryo.get('output_s21_db'), frequencies)

    adc_dsa_raw = (info or {}).get('dsa', 0)
    adc_dsa_db = 0.0 if adc_dsa_raw is None else float(adc_dsa_raw)

    return (
        cryo_output_s21
        + rx_bypass_amp_s21
        + rx_rf_s21
        - np.abs(rx_mixer_conv)
        + rx_if_s21
        - np.abs(rx_atten)
        - np.abs(rx_combiner)
        - np.abs(adc_dsa_db)
    )


def _reference_plane_offset_db(reference_plane, frequencies, info, config):
    """dB offset to *add* to an ADC-input power to get power at the plane.

    Returns None when the cal is unavailable (caller should warn and fall
    back).  Returns 0.0 for ``'adc_input'`` (the identity).
    """
    if reference_plane == 'adc_input':
        return 0.0
    if reference_plane in ('cryostat_output', 'detector'):
        gain = _rx_chain_gain_db(frequencies, info, config)
        if gain is None:
            return None
        return -gain
    raise ValueError(
        f"reference_plane must be one of {VALID_REFERENCE_PLANES}, "
        f"got {reference_plane!r}")


def _get_pyplot():
    """Lazy import of matplotlib.pyplot."""
    import matplotlib.pyplot as plt
    return plt


def _compute_mag_phase(z,unwrap=True):
    """
    Compute log magnitude (dB) and unwrapped phase from complex S21.

    Args:
        z: Complex array.
        unwrap: Whether to unwrap the phase.

    Returns:
        log_mag: 20*log10(|z|) in dB.
        phase: Unwrapped phase in radians.
    """
    log_mag = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z)) if unwrap else np.angle(z)
    return log_mag, phase


def _propagate_errors_mag(si, sq, ei, eq):
    """
    Propagate I/Q errors to log-magnitude and phase errors.

    Matches the convention used in wideband_sweep.py.

    Args:
        si, sq: I and Q signal arrays.
        ei, eq: I and Q standard error arrays.

    Returns:
        e_logmag: Error on 20*log10(|S21|) in dB.
        e_phase: Error on phase in radians.
    """
    mag_sq = si ** 2 + sq ** 2
    mag = np.sqrt(mag_sq)
    # Error on linear magnitude
    e_mag = np.sqrt((si * ei) ** 2 + (sq * eq) ** 2) / mag
    # Error on log magnitude
    e_logmag = 20 / (mag * np.log(10)) * e_mag
    # Error on phase
    e_phase = np.sqrt((sq * ei) ** 2 + (si * eq) ** 2) / mag_sq
    return e_logmag, e_phase


def _apply_deembed(frequencies, z, deembed, frequency=None, group_delay_cal=None):
    """
    Apply true RF deembedding (cable delay removal + baseline normalisation).

    For sweep data (frequencies provided, ``deembed is True``), computes
    deembedding from scratch.  For timestream data, pass pre-computed
    params as ``deembed`` (a dict from ``resonator.deembed()``).

    Args:
        frequencies: 1D frequency array (Hz), or None for timestream.
        z: 1D complex S21 array.
        deembed: bool or dict. If True, compute from scratch (requires
            frequencies). If dict, use as pre-computed params.
        frequency: Tone frequency (Hz) for cable delay removal when
            applying pre-computed params to timestream data.
        group_delay_cal: Frequency-dependent group delay calibration
            (from ``measure_path_group_delay()``).  Forwarded to
            ``resonator.deembed()`` when computing from scratch.

    Returns:
        z_out: Deembedded complex array (off-resonance at (1, 0)).
        params: dict of deembed parameters, or None.
    """
    if deembed is True:
        from .. import resonator
        return resonator.deembed(frequencies, z, group_delay_cal=group_delay_cal)
    elif isinstance(deembed, dict):
        from .. import resonator
        return resonator.apply_deembed_params(z, deembed, frequency=frequency), deembed

    return z, None


def _apply_phase_center(z, phase_center):
    """
    Apply phase centering (circle centering + rotation).

    For sweep or timestream data with ``phase_center is True``, computes
    centering from scratch.  Pass pre-computed params (a dict from
    ``resonator.phase_center()``) to apply to new data.

    Args:
        z: 1D complex S21 array.
        phase_center: bool or dict. If True, compute from scratch. If
            dict, use as pre-computed params.

    Returns:
        z_out: Phase-centered complex array.
        params: dict of phase-centering parameters, or None.
    """
    if phase_center is True:
        from .. import resonator
        return resonator.phase_center(z)
    elif isinstance(phase_center, dict):
        from .. import resonator
        return resonator.apply_phase_center_params(z, phase_center), phase_center

    return z, None


def _digital_gain(info, config=None, pre_accumulation=False):
    """Compute the total digital gain from system_information and config.

    Returns the linear scale factor that was applied to ADC codes to produce
    the raw I/Q values.  Dividing raw values by this factor recovers ADC
    amplitude in codes.

    Args:
        info: system_information dict (from sweep_data or ts_data).
        config: Optional config dict.  Only needed when rx_mix_scale != 1.
        pre_accumulation: True for snapshot data (no acc_len contribution).
    """
    pfb_fftshift = info.get('pfb_fftshift', 0)
    pfb_gain = 2 ** (13 - bin(pfb_fftshift).count('1'))

    rx_mix_scale = 0.7
    if config is not None:
        rx_mix_scale = config.get('firmware', {}).get('rx_mix_scale', 1.0)
    if rx_mix_scale is None:
        rx_mix_scale = 0.7

    if pre_accumulation:
        acc_gain = 1.0
    else:
        acc_gain = float(info.get('acc_len', 1))

    return pfb_gain * rx_mix_scale * acc_gain


def _normalise_iq(si, sq, units, info=None, config=None,
                  pre_accumulation=False, ei=None, eq=None,
                  reference_plane='adc_input', frequencies=None):
    """Normalise I/Q (and optionally error) arrays to the requested units.

    Args:
        si, sq: I and Q signal arrays.
        units: One of UNITS:
            'raw'    - no normalisation (accumulator codes).
            'peak'   - normalise to the peak magnitude of the data.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs'   - dB relative to ADC full-scale.
            'dbm'    - estimated power in dBm at ``reference_plane``.
        info: system_information dict.  Required for 'adc_fs', 'dbfs',
            'dbm'; ignored for 'raw' and 'peak'.
        config: Config dict (needed for 'dbm' and non-default rx_mix_scale).
        pre_accumulation: True for snapshot data.
        ei, eq: Optional error arrays (scaled identically for linear units).
        reference_plane: Only meaningful when ``units='dbm'``.  One of
            VALID_REFERENCE_PLANES.  ``'adc_input'`` (default) reports
            power at the ADC input; ``'cryostat_output'`` additionally
            deembeds the RX analog chain using the rf_frontend / cryostat
            entries in ``config``.  Labels are updated to reflect the
            chosen plane; the linear I/Q values themselves always remain
            in the ADC-input FS scale so I-vs-Q plots keep their native
            geometry — the plane-dependent offset is applied at the
            log-magnitude step in ``_compute_mag_phase_units``.
        frequencies: 1D array of per-sample frequencies (Hz), required
            only for ``reference_plane='cryostat_output'`` when the RX
            cal entries are frequency-dependent.

    Returns:
        (si, sq, ei, eq, iq_label, mag_label)
        where iq_label is the Y-axis label for I/Q plots and mag_label is
        the Y-axis label for magnitude plots.  ei, eq are None if not supplied.
    """
    if reference_plane not in VALID_REFERENCE_PLANES:
        raise ValueError(
            f"reference_plane must be one of {VALID_REFERENCE_PLANES}, "
            f"got {reference_plane!r}")
    if units == 'raw':
        return si, sq, ei, eq, '', '|S21| (dB)'

    if units == 'peak':
        peak = np.max(np.sqrt(si ** 2 + sq ** 2))
        if peak == 0:
            peak = 1.0
        p_si = si / peak
        p_sq = sq / peak
        p_ei = ei / peak if ei is not None else None
        p_eq = eq / peak if eq is not None else None
        return (p_si, p_sq, p_ei, p_eq,
                '(peak-normalised)', '|S21| (dB, peak-normalised)')

    gain = _digital_gain(info, config, pre_accumulation)

    # ADC codes (undo digital gain)
    adc_i = si / gain
    adc_q = sq / gain
    adc_ei = ei / gain if ei is not None else None
    adc_eq = eq / gain if eq is not None else None

    adc_bits = 16  # register width used by firmware
    half_scale = 2 ** (adc_bits - 1)

    if units == 'adc_fs':
        fs_i = adc_i / half_scale
        fs_q = adc_q / half_scale
        fs_ei = adc_ei / half_scale if adc_ei is not None else None
        fs_eq = adc_eq / half_scale if adc_eq is not None else None
        return fs_i, fs_q, fs_ei, fs_eq, '(ADC FS)', '|S21| (ADC FS)'

    if units == 'dbm':
        # Check ADC cal availability; without it we can't produce dBm at any
        # reference plane.
        if _resolve_adc_dbm_to_dbfs(config, frequencies) is None:
            warnings.warn(
                "units='dbm' requested but config['firmware']['adc_dbm_to_dbfs'] "
                "is not available — cannot compute dBm at the ADC input. "
                "Falling back to 'dbfs'.",
                RuntimeWarning,
                stacklevel=3,
            )
            units = 'dbfs'
        elif reference_plane in ('cryostat_output', 'detector'):
            # Need the RX chain cal to deembed; warn and drop back to
            # adc_input if it's not available.
            if _reference_plane_offset_db(
                    reference_plane, frequencies, info, config) is None:
                warnings.warn(
                    f"reference_plane='{reference_plane}' requested but the "
                    "RX analog chain calibration is not available in the "
                    "config — falling back to reference_plane='adc_input'.",
                    RuntimeWarning,
                    stacklevel=3,
                )
                reference_plane = 'adc_input'

    if units == 'dbfs':
        # Return linear FS values; magnitude will be computed as dBFS
        fs_i = adc_i / half_scale
        fs_q = adc_q / half_scale
        fs_ei = adc_ei / half_scale if adc_ei is not None else None
        fs_eq = adc_eq / half_scale if adc_eq is not None else None
        return fs_i, fs_q, fs_ei, fs_eq, '(FS)', '|S21| (dBFS)'

    if units == 'dbm':
        # Same linear scaling as dbfs; magnitude label changes
        mixer_qmc_gain = 1.0
        mixer_scale_is_1p0 = False
        if info.get('mixer_qmc_settings_adc') is not None:
            qmc = info['mixer_qmc_settings_adc']
            # GainCorrectionFactor is only applied when EnableGain is set;
            # when disabled the firmware reports a factor of 0.0, which must
            # be treated as unity (no correction).
            if qmc.get('EnableGain', 0):
                mixer_qmc_gain = qmc.get('GainCorrectionFactor', 1.0)
        if info.get('mixer_scale_1p0_adc') is not None:
            mixer_scale_is_1p0 = info['mixer_scale_1p0_adc']
        # Undo mixer effects that were applied before the PFB
        ddc_i = adc_i.copy()
        ddc_q = adc_q.copy()
        if not mixer_scale_is_1p0:
            ddc_i = ddc_i * np.sqrt(2)
            ddc_q = ddc_q * np.sqrt(2)
        ddc_i = ddc_i / mixer_qmc_gain
        ddc_q = ddc_q / mixer_qmc_gain
        fs_i = ddc_i / half_scale
        fs_q = ddc_q / half_scale

        fs_ei = None
        fs_eq = None
        if adc_ei is not None:
            ddc_ei = adc_ei.copy()
            ddc_eq = adc_eq.copy()
            if not mixer_scale_is_1p0:
                ddc_ei = ddc_ei * np.sqrt(2)
                ddc_eq = ddc_eq * np.sqrt(2)
            ddc_ei = ddc_ei / mixer_qmc_gain
            ddc_eq = ddc_eq / mixer_qmc_gain
            fs_ei = ddc_ei / half_scale
            fs_eq = ddc_eq / half_scale

        # Linear I/Q are in ADC-input FS scale; the dBFS→dBm offset and
        # any reference-plane shift are applied at the log-magnitude step
        # in _compute_mag_phase_units so the I-vs-Q geometry is preserved.
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        iq_label = '(ADC input FS)'
        mag_label = f'power @ {plane_label} (dBm)'
        return fs_i, fs_q, fs_ei, fs_eq, iq_label, mag_label

    raise ValueError(f"Unknown units '{units}'. Use one of {UNITS}.")


def _compute_mag_phase_units(z, units, info=None, config=None,
                              reference_plane='adc_input', frequencies=None, unwrap=True):
    """Compute magnitude and phase with units-aware magnitude labels.

    For 'raw' and 'adc_fs', magnitude is 20*log10(|z|).
    For 'dbfs', magnitude is 20*log10(|z|) (z is already in FS units).
    For 'dbm', magnitude is 20*log10(|z|) + adc_dbm_to_dbfs + plane_offset,
    where plane_offset shifts from the ADC input to ``reference_plane``
    (0 dB for ``'adc_input'``; -rx_chain_gain for ``'cryostat_output'``).

    Must be called with the same (reference_plane, frequencies) that were
    passed to ``_normalise_iq`` — the offset is applied here, not in the
    linear normalisation.  If either the ADC cal or (for non-adc_input
    planes) the RX chain cal is unavailable, the corresponding offset is
    silently omitted so the result degrades gracefully to dBFS; the
    warning has already been emitted by ``_normalise_iq``.
    """
    mag_db = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z)) if unwrap else np.angle(z)

    if units == 'dbm':
        cal_db = _resolve_adc_dbm_to_dbfs(config, frequencies)
        if cal_db is not None:
            mag_db = mag_db + cal_db
            plane_offset = _reference_plane_offset_db(
                reference_plane, frequencies, info, config)
            if plane_offset is not None:
                mag_db = mag_db + plane_offset

    return mag_db, phase


# Standard error bar style (legacy, kept for non-sweep plots)
ERRORBAR_STYLE = dict(fmt='.', capsize=0, ecolor='red', markersize=2)

# Fill style for fast error-band rendering on large traces
ERROR_FILL_STYLE = dict(alpha=0.25)


def _resolve_label(ax, label=None, suffix=None):
    """Resolve the label for a plot line.

    If label and suffix both given, returns '{label} {suffix}'.
    If only label, returns label.
    If only suffix, returns str(suffix).
    If neither, returns the next integer index based on existing lines on ax.
    """
    if label is not None:
        return f'{label} {suffix}' if suffix else label
    if suffix is not None:
        return str(suffix)
    return str(len(ax.get_lines()))
