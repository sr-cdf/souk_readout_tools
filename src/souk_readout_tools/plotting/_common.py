"""
Shared utilities for the plotting subpackage.
"""

import os
import warnings

import numpy as np

from souk_readout_tools import config_utils


# Supported units for normalisation. ``s21`` is kept as a backwards-compatible
# alias for ``s21_log``.
UNITS = (
    'raw', 'peak', 'adc_units', 'adc_fs', 'dbfs', 'dbm',
    'volts', 'v', 'watts', 'w',
    's21', 's21_log', 's21_linear',
)
_S21_UNITS = ('s21_log', 's21_linear')
_VOLTAGE_UNITS = ('volts',)
_WATT_UNITS = ('watts',)
_PHYSICAL_LINEAR_UNITS = _VOLTAGE_UNITS + _WATT_UNITS
_CALIBRATED_MAGNITUDE_UNITS = (
    'dbfs', 'dbm', 'volts', 'watts', 's21_log', 's21_linear',
)
_DBM_MAGNITUDE_UNITS = ('dbm', 'volts', 'watts', 's21_log', 's21_linear')
_DEFAULT_RF_IMPEDANCE_OHM = 50.0

# Reference planes the plot-time calibration can report at for calibrated
# magnitude units ('dbfs', 'dbm', 'volts', 'watts', and 's21').
#   'adc_input'       — power at the ADC input
#   'cryostat_output' — power at the cryostat output, i.e. at the RX port of
#                       the cryostat, deembedding the RX analog chain
#                       (requires rf_frontend + cryostat calibration entries)
VALID_REFERENCE_PLANES = ('adc_input', 'cryostat_output', 'detector')
_TX_EQUIVALENT_RX_PLANES = ('cryostat_output', 'detector')
_REFERENCE_PLANE_LABELS = {
    'adc_input': 'ADC input',
    'cryostat_output': 'cryostat output',
    'detector': 'detector',
}

# Directory that frequency-dependent cal files (referenced by filename in the
# config) live in. Shares config_utils.get_user_dir() with firmware_lib.USER_DIR
# so the two stay in lockstep (and both resolve the right home under sudo).
_CAL_USER_DIR = config_utils.get_user_dir()
_CAL_FILE_CACHE = {}
_CONFIG_TEXT_CACHE = {}


def _canonical_units(units):
    """Return the internal spelling for a public plotting unit string."""
    aliases = {
        's21': 's21_log',
        'adc': 'adc_units',
        'adc_unit': 'adc_units',
        'adc_code': 'adc_units',
        'adc_codes': 'adc_units',
        'v': 'volts',
        'volt': 'volts',
        'voltage': 'volts',
        'w': 'watts',
        'watt': 'watts',
    }
    units = str(units).lower()
    return aliases.get(units, units)


def _is_s21_unit(units):
    """Return True for linear or log S21 units."""
    return _canonical_units(units) in _S21_UNITS


def _is_s21_linear_unit(units):
    """Return True when magnitudes should be shown as linear |S21|."""
    return _canonical_units(units) == 's21_linear'


def _is_voltage_unit(units):
    """Return True when magnitudes should be shown as RMS volts."""
    return _canonical_units(units) in _VOLTAGE_UNITS


def _is_watt_unit(units):
    """Return True when magnitudes should be shown as watts."""
    return _canonical_units(units) in _WATT_UNITS


def _is_linear_physical_unit(units):
    """Return True for calibrated linear physical magnitude units."""
    return _canonical_units(units) in _PHYSICAL_LINEAR_UNITS


def _is_calibrated_magnitude_unit(units):
    """Return True when magnitude plotting needs calibration/reference-plane work."""
    return _canonical_units(units) in _CALIBRATED_MAGNITUDE_UNITS


def _validate_reference_plane(reference_plane):
    """Validate a plotting reference plane."""
    if reference_plane == 'accumulator':
        raise ValueError(
            "reference_plane='accumulator' is not a plotting reference "
            "plane. Use units='raw' for raw accumulator-unit plots, or use "
            f"one of {VALID_REFERENCE_PLANES} for calibrated plots.")
    if reference_plane not in VALID_REFERENCE_PLANES:
        raise ValueError(
            f"reference_plane must be one of {VALID_REFERENCE_PLANES}, "
            f"got {reference_plane!r}")


def _frequency_cache_key(frequencies):
    """Small cache key for repeated calibration at one frequency array."""
    if frequencies is None:
        return None
    arr = np.asarray(frequencies)
    if arr.size == 0:
        return ('empty', arr.shape)
    flat = arr.ravel()
    return (
        id(arr),
        arr.shape,
        str(arr.dtype),
        int(arr.size),
        float(flat[0]) if np.isfinite(flat[0]) else np.nan,
        float(flat[-1]) if np.isfinite(flat[-1]) else np.nan,
        float(np.nanmean(flat)) if np.any(np.isfinite(flat)) else np.nan,
    )


def _config_from_info(info):
    """Recover the config dict embedded in a structured info blob, if present."""
    if not isinstance(info, dict):
        return None
    config_info = info.get('config')
    if not isinstance(config_info, dict):
        return None
    if isinstance(config_info.get('config_dict'), dict):
        return config_info['config_dict']
    config_text = config_info.get('config_text')
    if not config_text:
        return None
    if config_text in _CONFIG_TEXT_CACHE:
        return _CONFIG_TEXT_CACHE[config_text]
    try:
        import yaml
        parsed = yaml.safe_load(config_text)
    except Exception:
        return None
    parsed = parsed if isinstance(parsed, dict) else None
    _CONFIG_TEXT_CACHE[config_text] = parsed
    return parsed


def _effective_config(config=None, info=None, calibration_cache=None):
    """Prefer an explicit config, falling back to one stored in ``info``."""
    if config is not None:
        return config
    if calibration_cache is not None and 'effective_config' in calibration_cache:
        return calibration_cache['effective_config']
    config = _config_from_info(info)
    if calibration_cache is not None:
        calibration_cache['effective_config'] = config
    return config


def _info_section(info, name):
    """Return a named structured-info section as a dict, or an empty dict."""
    if not isinstance(info, dict):
        return {}
    section = info.get(name, {})
    return section if isinstance(section, dict) else {}


def _first_present(*values):
    """Return the first non-None value from ``values``."""
    for value in values:
        if value is not None:
            return value
    return None


def _rf_impedance_ohm(info=None, config=None, calibration_cache=None):
    """Return the RF impedance used for dBm <-> volts conversion."""
    cache_key = 'rf_impedance_ohm'
    if calibration_cache is not None and cache_key in calibration_cache:
        return calibration_cache[cache_key]

    config = _effective_config(config, info, calibration_cache)
    firmware = config.get('firmware', {}) if config is not None else {}
    rf = config.get('rf_frontend', {}) if config is not None else {}
    cal_info = _info_section(info, 'calibrations')
    rf_info = _info_section(info, 'rf_frontend')
    value = _first_present(
        firmware.get('rf_impedance_ohm'),
        firmware.get('adc_input_impedance_ohm'),
        rf.get('impedance_ohm'),
        cal_info.get('rf_impedance_ohm'),
        cal_info.get('adc_input_impedance_ohm'),
        rf_info.get('impedance_ohm'),
    )
    try:
        impedance = float(value)
    except (TypeError, ValueError):
        impedance = _DEFAULT_RF_IMPEDANCE_OHM
    if not np.isfinite(impedance) or impedance <= 0.0:
        impedance = _DEFAULT_RF_IMPEDANCE_OHM

    if calibration_cache is not None:
        calibration_cache[cache_key] = impedance
    return impedance


def _watts_from_dbm(power_dbm):
    """Convert dBm to watts."""
    return 1e-3 * (10.0 ** (np.asarray(power_dbm, dtype=float) / 10.0))


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
    arr = np.asarray(value) if not isinstance(value, str) else None
    if arr is not None and arr.ndim == 1:
        if arr.size == 1:
            return float(arr[0])
        if frequencies is not None:
            frequencies = np.asarray(frequencies, dtype=float)
            if arr.shape == frequencies.shape:
                return arr.astype(float)
        return None
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
        if cal_path in _CAL_FILE_CACHE:
            cal_f, cal_db = _CAL_FILE_CACHE[cal_path]
        else:
            cal_f, cal_db = np.loadtxt(cal_path, ndmin=2).T
            _CAL_FILE_CACHE[cal_path] = (cal_f, cal_db)
    else:
        cal_f, cal_db = np.array(value, ndmin=2).T
    cal_f = np.asarray(cal_f, dtype=float).ravel()
    cal_db = np.asarray(cal_db, dtype=float).ravel()
    good_cal = np.isfinite(cal_f) & np.isfinite(cal_db)
    if not np.any(good_cal):
        return None
    cal_f = cal_f[good_cal]
    cal_db = cal_db[good_cal]
    order = np.argsort(cal_f)
    cal_f = cal_f[order]
    cal_db = cal_db[order]

    flat = frequencies.ravel()
    resolved = np.full(flat.shape, np.nan, dtype=float)
    good = np.isfinite(flat)
    if np.any(good):
        idx = np.searchsorted(cal_f, flat[good], side='left')
        right = np.clip(idx, 0, cal_f.size - 1)
        left = np.clip(idx - 1, 0, cal_f.size - 1)
        choose_right = (
            np.abs(cal_f[right] - flat[good])
            < np.abs(flat[good] - cal_f[left])
        )
        nearest = np.where(choose_right, right, left)
        resolved[good] = cal_db[nearest]
    return resolved.reshape(frequencies.shape)


def _cal_or_zero(value, frequencies=None):
    """_resolve_cal_value but maps None → 0.0 (scalar)."""
    resolved = _resolve_cal_value(value, frequencies)
    return 0.0 if resolved is None else resolved


def _resolve_adc_dbm_to_dbfs(config, frequencies=None, info=None,
                             calibration_cache=None):
    """Return the ADC dBFS→dBm offset, or None if unavailable.

    Reads ``config['firmware']['adc_dbm_to_dbfs']``.  When the entry is a
    scalar the result is a single float; when it's frequency-dependent
    (array / CSV filename) and ``frequencies`` is provided, the result is
    a per-sample array.  Returns None when the entry is missing, None, or
    frequency-dependent without a frequency axis to interpolate onto.
    """
    cache_key = ('adc_dbm_to_dbfs', _frequency_cache_key(frequencies))
    if calibration_cache is not None and cache_key in calibration_cache:
        return calibration_cache[cache_key]

    config = _effective_config(config, info, calibration_cache)
    if config is None:
        cal_info = _info_section(info, 'calibrations')
        val = cal_info.get('adc_dbm_to_dbfs')
        result = _resolve_cal_value(val, frequencies)
        if calibration_cache is not None:
            calibration_cache[cache_key] = result
        return result
    val = config.get('firmware', {}).get('adc_dbm_to_dbfs')
    result = _resolve_cal_value(val, frequencies)
    if calibration_cache is not None:
        calibration_cache[cache_key] = result
    return result


def _rx_chain_gain_db(frequencies, info, config, calibration_cache=None):
    """Per-sample RX analog chain gain from cryostat output to ADC input (dB).

    Subtracting this from a power at the ADC input gives the equivalent
    power at the cryostat output.  Sign conventions match
    ``calibration.calc_adc_input_power``: S21 values add to the forward
    gain, explicit loss/attenuator values subtract via their absolute
    magnitude.  Live per-sweep state in ``info`` takes precedence over static
    config for dynamic controls such as RF RX attenuation, RX amp bypass, and
    ADC DSA.

    Frequency-dependent cal entries (CSV filename / [[f, dB], …]) are
    resolved at each entry of ``frequencies``; scalar entries broadcast.

    Returns 0.0 when neither rf_frontend nor cryostat is marked
    ``connected: true``; returns None when no config/metadata is available.
    """
    cache_key = ('rx_chain_gain_db', _frequency_cache_key(frequencies))
    if calibration_cache is not None and cache_key in calibration_cache:
        return calibration_cache[cache_key]

    config = _effective_config(config, info, calibration_cache)
    if config is None and not isinstance(info, dict):
        return None
    rf = config.get('rf_frontend', {}) if config is not None else {}
    cryo = config.get('cryostat', {}) if config is not None else {}
    rf_info = _info_section(info, 'rf_frontend')
    lna_info = _info_section(info, 'lna')
    cal_info = _info_section(info, 'calibrations')
    if config is None and not (rf_info or lna_info or cal_info):
        return None

    rf_connected = bool(_first_present(
        rf_info.get('connected'), rf.get('connected'), False))
    cryo_connected = bool(_first_present(
        lna_info.get('cryostat_connected'), cryo.get('connected'), False))
    if not (rf_connected or cryo_connected):
        if calibration_cache is not None:
            calibration_cache[cache_key] = 0.0
        return 0.0

    rx_rf_s21 = 0.0
    rx_if_s21 = 0.0
    rx_bypass_amp_s21 = 0.0
    rx_mixer_conv = 0.0
    rx_combiner = 0.0
    rx_atten = 0.0
    rx_dynamic_gain = None
    cryo_output_s21 = 0.0

    if rf_connected:
        rx_rf_s21 = _cal_or_zero(_first_present(
            rf.get('rx_rf_s21_db'), cal_info.get('rx_rf_s21_db'),
            rf_info.get('rx_rf_s21_db')), frequencies)
        rx_if_s21 = _cal_or_zero(_first_present(
            rf.get('rx_if_s21_db'), cal_info.get('rx_if_s21_db'),
            rf_info.get('rx_if_s21_db')), frequencies)
        rx_dynamic_gain_raw = rf_info.get('rx_total_gain_db')
        if rx_dynamic_gain_raw is not None:
            try:
                rx_dynamic_gain = float(rx_dynamic_gain_raw)
            except (TypeError, ValueError):
                rx_dynamic_gain = None
            if rx_dynamic_gain is not None and not np.isfinite(rx_dynamic_gain):
                rx_dynamic_gain = None

        rx_amp_bypass = _first_present(
            rf_info.get('rx_amp_bypass'),
            (rf.get('bypass_amps', {}) or {}).get('rx_amp_bypass'))
        if rx_dynamic_gain is None and rx_amp_bypass is not None:
            rx_amp_key = (
                'rx_amp_bypassed_s21_db'
                if rx_amp_bypass else 'rx_amp_enabled_s21_db'
            )
            rx_bypass_amp_s21 = _cal_or_zero(_first_present(
                rf_info.get('rx_bypass_amp_s21_db'),
                rf_info.get(rx_amp_key),
                (rf.get('mixerless_module', {}) or {}).get(rx_amp_key),
                cal_info.get(f'mixerless_module.{rx_amp_key}')), frequencies)
        rx_mixer_conv = _cal_or_zero(_first_present(
            rf.get('rx_mixer_conversion_loss_db'),
            cal_info.get('rx_mixer_conversion_loss_db'),
            rf_info.get('rx_mixer_conversion_loss_db')), frequencies)
        rx_combiner = _cal_or_zero(_first_present(
            rf.get('rx_combiner_loss_db'), cal_info.get('rx_combiner_loss_db'),
            rf_info.get('rx_combiner_loss_db')), frequencies)
        rx_atten_raw = _first_present(
            rf_info.get('rx_attenuation_db'),
            (rf.get('attenuator', {}) or {}).get('rx_value_db'))
        rx_atten = 0.0 if rx_atten_raw is None else float(rx_atten_raw)
    if cryo_connected:
        cryo_output_s21 = _cal_or_zero(_first_present(
            cryo.get('output_s21_db'),
            cal_info.get('cryostat_output_s21_db')), frequencies)

    rfdc_info = _info_section(info, 'rfdc')
    adc_dsa_raw = _first_present((info or {}).get('dsa'), rfdc_info.get('dsa'), 0)
    adc_dsa_db = 0.0 if adc_dsa_raw is None else float(adc_dsa_raw)

    result = (
        cryo_output_s21
        + (
            rx_dynamic_gain
            if rx_dynamic_gain is not None
            else rx_bypass_amp_s21 - np.abs(rx_atten)
        )
        + rx_rf_s21
        - np.abs(rx_mixer_conv)
        + rx_if_s21
        - np.abs(rx_combiner)
        - np.abs(adc_dsa_db)
    )
    if calibration_cache is not None:
        calibration_cache[cache_key] = result
    return result


def _reference_plane_offset_db(reference_plane, frequencies, info, config,
                               calibration_cache=None):
    """dB offset to *add* to an ADC-input power to get power at the plane.

    Returns None when the cal is unavailable (caller should warn and fall
    back).  Returns 0.0 for ``'adc_input'`` (the identity).
    """
    _validate_reference_plane(reference_plane)
    if reference_plane == 'adc_input':
        return 0.0
    if reference_plane in ('cryostat_output', 'detector'):
        cache_key = (
            'reference_plane_offset_db',
            reference_plane,
            _frequency_cache_key(frequencies),
        )
        if calibration_cache is not None and cache_key in calibration_cache:
            return calibration_cache[cache_key]
        gain = _rx_chain_gain_db(
            frequencies, info, config, calibration_cache=calibration_cache)
        if gain is None:
            return None
        result = -gain
        if calibration_cache is not None:
            calibration_cache[cache_key] = result
        return result
    raise ValueError(f"Unsupported reference_plane={reference_plane!r}")


def _get_pyplot():
    """Lazy import of matplotlib.pyplot."""
    import matplotlib.pyplot as plt
    return plt


def _compute_mag_phase(z,unwrap=True):
    """
    Compute log magnitude (dB) and phase from a complex response.

    Args:
        z: Complex array.
        unwrap: Whether to unwrap the phase.

    Returns:
        log_mag: 20*log10(|z|) in dB.
        phase: Phase in radians, unwrapped when requested.
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
        e_logmag: Error on 20*log10(|z|) in dB.
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


def _digital_gain(info, config=None, pre_accumulation=False,
                  calibration_cache=None):
    """Compute the total digital gain from the structured info dict and config.

    Returns the linear scale factor that was applied to ADC codes to produce
    the raw I/Q values.  Dividing raw values by this factor recovers ADC
    amplitude in codes.

    Args:
        info: structured info dict (from sweep_data or ts_data) with at
            least a 'pipeline' section.
        config: Optional config dict.  Only needed when rx_mix_scale != 1.
        pre_accumulation: True for snapshot data (no acc_len contribution).
    """
    cache_key = ('digital_gain', bool(pre_accumulation))
    if calibration_cache is not None and cache_key in calibration_cache:
        return calibration_cache[cache_key]

    config = _effective_config(config, info, calibration_cache)
    pipeline = info.get('pipeline', {}) if info else {}
    pfb_fftshift = pipeline.get('pfb_fftshift', 0) or 0
    pfb_gain = 2 ** (13 - bin(pfb_fftshift).count('1'))

    rx_mix_scale = 0.7
    if config is not None:
        rx_mix_scale = config.get('firmware', {}).get('rx_mix_scale', 1.0)
    if rx_mix_scale is None:
        rx_mix_scale = 0.7

    if pre_accumulation:
        acc_gain = 1.0
    else:
        acc_gain = float(pipeline.get('acc_len', 1) or 1)

    gain = pfb_gain * rx_mix_scale * acc_gain
    if calibration_cache is not None:
        calibration_cache[cache_key] = gain
    return gain


def _adc_fullscale_bits(info=None, config=None, calibration_cache=None):
    """Return the configured ADC full-scale register width."""
    if calibration_cache is not None and 'adc_fullscale_bits' in calibration_cache:
        return calibration_cache['adc_fullscale_bits']
    config = _effective_config(config, info, calibration_cache)
    if config is None:
        return 16
    bits = int(config.get('firmware', {}).get('adc_fullscale_bits', 16) or 16)
    if calibration_cache is not None:
        calibration_cache['adc_fullscale_bits'] = bits
    return bits


def _warn_reference_plane_fallback(reference_plane):
    warnings.warn(
        f"reference_plane='{reference_plane}' requested but the RX analog "
        "chain calibration is not available — falling back to "
        "reference_plane='adc_input'.",
        RuntimeWarning,
        stacklevel=3,
    )


def _validated_reference_plane(reference_plane, frequencies, info, config,
                               *, strict=False, calibration_cache=None):
    """Return a usable reference plane for calibrated magnitude units."""
    if reference_plane == 'adc_input':
        return reference_plane
    if _reference_plane_offset_db(
            reference_plane, frequencies, info, config,
            calibration_cache=calibration_cache) is None:
        if strict:
            raise ValueError(
                f"reference_plane='{reference_plane}' requested but the RX "
                "analog chain calibration is not available.")
        _warn_reference_plane_fallback(reference_plane)
        return 'adc_input'
    return reference_plane


def _with_sweep_readout_correction(sweep_data, apply_readout_correction):
    """
    Return sweep data with the software readout correction (the filterbank
    compensation's RX half) applied or removed to match the request.

    Parsed sweep data carries the per-(point, tone) factors
    (``readout_correction``) and whether they are already applied
    (``readout_correction_applied``), so the correction can be toggled either
    way at plot time. No-op (returns the input) when the data is already in
    the requested state, or when the factors are unavailable or shape-
    mismatched (older captures, concatenated wideband data).
    """
    if not isinstance(sweep_data, dict):
        return sweep_data
    want = bool(apply_readout_correction)
    applied = bool(sweep_data.get('readout_correction_applied', False))
    if want == applied:
        return sweep_data
    rc = sweep_data.get('readout_correction')
    if rc is None or 'sweep_i' not in sweep_data:
        return sweep_data
    rc = np.asarray(rc, dtype=float)
    if rc.shape != np.shape(sweep_data['sweep_i']):
        return sweep_data
    gain = rc if want else 1.0 / rc
    out = dict(sweep_data)
    for key in ('sweep_i', 'sweep_q', 'sweep_ei', 'sweep_eq'):
        if key in out and out[key] is not None:
            out[key] = np.asarray(out[key], dtype=float) * gain
    out['readout_correction_applied'] = want
    return out


def _with_samples_readout_correction(ts_data, apply_readout_correction):
    """
    Return parsed timestream data with the software readout correction (the
    filterbank compensation's RX half) applied or removed to match the
    request, per sample via the modulation point tag.

    No-op (returns the input) when the data is already in the requested state,
    when the capture carries no modulation tags, or when the info snapshot
    reports no correction factors (compensation off, older captures).
    """
    if not isinstance(ts_data, dict):
        return ts_data
    want = bool(apply_readout_correction)
    applied = bool(ts_data.get('readout_correction_applied', False))
    if want == applied:
        return ts_data
    points = ts_data.get('modulation_point')
    if points is None:
        return ts_data
    points = np.asarray(points, dtype=int)
    if not np.any(points > 0):
        return ts_data
    from souk_readout_tools.modulation import readout_correction_factors
    num_tones = int(ts_data['num_tones'])
    factors = readout_correction_factors(ts_data.get('info'), num_tones)
    if factors is None:
        return ts_data
    n_points = factors.shape[0]
    valid = (points >= 1) & (points <= n_points)
    gain = np.ones((len(points), num_tones), dtype=float)
    gain[valid] = factors[points[valid] - 1]
    if not want:
        gain = 1.0 / gain

    def _scaled(per_tone_data):
        scaled = {}
        for key, values in per_tone_data.items():
            try:
                col = int(key)
            except (TypeError, ValueError):
                scaled[key] = values
                continue
            values = np.asarray(values)
            if col < num_tones and len(values) == len(points):
                scaled[key] = values.astype(float) * gain[:, col]
            else:
                scaled[key] = values
        return scaled

    out = dict(ts_data)
    out['i_data'] = _scaled(ts_data['i_data'])
    out['q_data'] = _scaled(ts_data['q_data'])
    out['readout_correction_applied'] = want
    return out


def _scale_iq_components(si, sq, ei, eq, scale):
    """Apply one linear scale to I/Q data and optional I/Q uncertainties."""
    si = np.asarray(si) * scale
    sq = np.asarray(sq) * scale
    if ei is not None:
        ei = np.asarray(ei) * scale
    if eq is not None:
        eq = np.asarray(eq) * scale
    return si, sq, ei, eq


def _tx_reference_plane_offset_db(source_plane, target_plane, frequencies,
                                  info, config, calibration_cache=None):
    """dB offset to add to a TX power to express it at another RX plane.

    The live ``info['tones']['powers_dbm']`` snapshot is detector-plane by
    convention.  For S21 plots we sometimes need the corresponding modelled
    TX/RX-chain power at the ADC input so the denominator is at the same plane
    as the received-power numerator.
    """
    source_plane = str(source_plane)
    target_plane = str(target_plane)
    if source_plane == target_plane:
        return 0.0
    if (source_plane in _TX_EQUIVALENT_RX_PLANES
            and target_plane in _TX_EQUIVALENT_RX_PLANES):
        return 0.0
    if source_plane in _TX_EQUIVALENT_RX_PLANES and target_plane == 'adc_input':
        return _rx_chain_gain_db(
            frequencies, info, config, calibration_cache=calibration_cache)
    if source_plane == 'adc_input' and target_plane in _TX_EQUIVALENT_RX_PLANES:
        gain = _rx_chain_gain_db(
            frequencies, info, config, calibration_cache=calibration_cache)
        return None if gain is None else -gain
    return None


def _convert_tx_power_reference_plane(tx_power_dbm, source_plane, target_plane,
                                      info=None, config=None,
                                      frequencies=None, *, strict=False,
                                      calibration_cache=None):
    """Convert a TX-power vector between supported plot reference planes."""
    if tx_power_dbm is None:
        return None
    if source_plane is None:
        source_plane = target_plane
    offset_db = _tx_reference_plane_offset_db(
        source_plane, target_plane, frequencies, info, config,
        calibration_cache=calibration_cache)
    if offset_db is None:
        if strict:
            raise ValueError(
                "units='s21' could not convert tx_power_dbm from "
                f"{source_plane!r} to {target_plane!r}; calibration metadata "
                "for the intervening chain is unavailable.")
        return None
    return np.asarray(tx_power_dbm, dtype=float) + offset_db


def _tx_tone_powers_from_info(info, reference_plane='detector', config=None,
                              frequencies=None, calibration_cache=None):
    """Resolve the live tone powers from structured ``info`` at a plot plane."""
    tones = _info_section(info, 'tones')
    powers = tones.get('powers_dbm')
    if powers is None:
        return None
    powers = np.asarray(powers, dtype=float).ravel()
    if powers.size == 0:
        return None
    source_plane = tones.get('powers_reference_plane', 'detector')
    if frequencies is None:
        frequencies = tones.get('frequencies_hz')
    return _convert_tx_power_reference_plane(
        powers, source_plane, reference_plane, info=info, config=config,
        frequencies=frequencies, calibration_cache=calibration_cache)


def _normalise_iq(si, sq, units, info=None, config=None,
                  pre_accumulation=False, ei=None, eq=None,
                  reference_plane='adc_input', frequencies=None,
                  calibration_cache=None):
    """Normalise I/Q (and optionally error) arrays to the requested units.

    Args:
        si, sq: I and Q signal arrays.
        units: One of UNITS:
            'raw'    - no normalisation (accumulator codes).
            'peak'   - normalise to the peak magnitude of the data.
            'adc_units' - linear ADC units, optionally referred to
                ``reference_plane``.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs'   - dB relative to ADC full-scale.
            'dbm'    - estimated power in dBm at ``reference_plane``.
            'volts'/'v' - estimated RMS voltage at ``reference_plane``.
            'watts'/'w' - estimated power in watts at ``reference_plane``.
            's21'/'s21_log' - received power minus known transmitted power
                in dB.
            's21_linear' - linear |S21| from the same calibrated power ratio.
        info: structured info dict.  Required for 'adc_units', 'adc_fs',
            'dbfs', 'dbm', 'volts', and 'watts'; ignored for 'raw' and
            'peak'.
        config: Config dict (needed when the sweep ``info`` does not contain
            the run config, or for non-default rx_mix_scale in older data).
        pre_accumulation: True for snapshot data.
        ei, eq: Optional error arrays (scaled identically for linear units).
        reference_plane: One of VALID_REFERENCE_PLANES. ``'adc_input'``
            (default) reports power at the ADC input;
            ``'cryostat_output'``/``'detector'`` remove the RX analog-chain
            gain using config or structured-info calibration entries.
            ``units='raw'`` always remains accumulator units and does not use
            ``reference_plane``.  Use ``units='adc_units'`` for linear ADC
            coordinates referred to ``reference_plane``.  For calibrated
            log-magnitude units
            (``'dbfs'``, ``'dbm'``, ``'volts'``, ``'watts'``, and ``'s21'``),
            the linear I/Q values themselves remain in ADC-input full-scale
            units so I-vs-Q plots keep their native geometry; the plane-
            dependent offset is applied at the log-magnitude step in
            ``_compute_mag_phase_units``.
        frequencies: 1D array of per-sample frequencies (Hz), required
            only for ``reference_plane='cryostat_output'`` when the RX
            cal entries are frequency-dependent.

    Returns:
        (si, sq, ei, eq, iq_label, mag_label)
        where iq_label is the Y-axis label for I/Q plots and mag_label is
        the Y-axis label for magnitude plots.  ei, eq are None if not supplied.
    """
    units = _canonical_units(units)
    if units == 'raw':
        _validate_reference_plane(reference_plane)
        if reference_plane != 'adc_input':
            raise ValueError(
                "units='raw' always means accumulator units and does not have "
                "a detector/cryostat reference plane. Use units='adc_units' "
                "to undo the firmware accumulator path and refer linear ADC "
                f"units to reference_plane={reference_plane!r}.")
        return si, sq, ei, eq, '', '|RX| (dB)'

    _validate_reference_plane(reference_plane)
    if units == 'peak':
        peak = np.max(np.sqrt(si ** 2 + sq ** 2))
        if peak == 0:
            peak = 1.0
        p_si = si / peak
        p_sq = sq / peak
        p_ei = ei / peak if ei is not None else None
        p_eq = eq / peak if eq is not None else None
        return (p_si, p_sq, p_ei, p_eq,
                '(peak norm)', '|RX| (dB, peak-normalised)')

    gain = _digital_gain(
        info, config, pre_accumulation,
        calibration_cache=calibration_cache)

    # ADC codes (undo digital gain)
    adc_i = si / gain
    adc_q = sq / gain
    adc_ei = ei / gain if ei is not None else None
    adc_eq = eq / gain if eq is not None else None

    adc_bits = _adc_fullscale_bits(
        info, config, calibration_cache=calibration_cache)
    half_scale = 2 ** (adc_bits - 1)

    mixer_qmc_gain = 1.0
    mixer_scale_is_1p0 = False
    rfdc = info.get('rfdc', {}) if info else {}
    if rfdc.get('qmc_settings_adc') is not None:
        qmc = rfdc['qmc_settings_adc']
        # GainCorrectionFactor is only applied when EnableGain is set; when
        # disabled the firmware reports a factor of 0.0, which must be treated
        # as unity (no correction).
        if qmc.get('EnableGain', 0):
            mixer_qmc_gain = qmc.get('GainCorrectionFactor', 1.0)
    if rfdc.get('mixer_scale_1p0_adc') is not None:
        mixer_scale_is_1p0 = rfdc['mixer_scale_1p0_adc']

    # Undo mixer effects that were applied before the PFB.  These corrected
    # values are ADC-input full-scale complex amplitudes.
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

    if units in _DBM_MAGNITUDE_UNITS:
        # Check ADC cal availability; without it we can't produce dBm at any
        # reference plane.
        if _resolve_adc_dbm_to_dbfs(
                config, frequencies, info=info,
                calibration_cache=calibration_cache) is None:
            if units in _S21_UNITS or units in _PHYSICAL_LINEAR_UNITS:
                raise ValueError(
                    f"units='{units}' requires "
                    "config['firmware']['adc_dbm_to_dbfs'] or equivalent "
                    "calibration metadata.")
            warnings.warn(
                f"units='{units}' requested but "
                "config['firmware']['adc_dbm_to_dbfs'] is not available — "
                "cannot compute calibrated ADC-input power. Falling back "
                "to 'dbfs'.",
                RuntimeWarning,
                stacklevel=3,
            )
            units = 'dbfs'

    if units in _CALIBRATED_MAGNITUDE_UNITS:
        reference_plane = _validated_reference_plane(
            reference_plane, frequencies, info, config,
            strict=(units in _S21_UNITS),
            calibration_cache=calibration_cache)

    if units == 'adc_units':
        reference_plane = _validated_reference_plane(
            reference_plane, frequencies, info, config,
            calibration_cache=calibration_cache)
        scale = half_scale
        if reference_plane != 'adc_input':
            plane_offset = _reference_plane_offset_db(
                reference_plane, frequencies, info, config,
                calibration_cache=calibration_cache)
            scale = scale * (10.0 ** (plane_offset / 20.0))
        ai, aq, ae_i, ae_q = _scale_iq_components(
            fs_i, fs_q, fs_ei, fs_eq, scale)
        if reference_plane == 'adc_input':
            return (
                ai, aq, ae_i, ae_q,
                '(ADC units)',
                '|RX @ ADC input| (dB ADC units)',
            )
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        return (
            ai, aq, ae_i, ae_q,
            f'(ADC units @ {plane_label})',
            f'|RX| @ {plane_label} (dB ADC units)',
        )

    if units == 'adc_fs':
        return fs_i, fs_q, fs_ei, fs_eq, '(ADC FS)', '|RX @ ADC input| (dBFS)'

    if units == 'dbfs':
        # Return linear FS values; magnitude will be computed as dBFS.
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        mag_label = f'power @ {plane_label} (dBFS)'
        return fs_i, fs_q, fs_ei, fs_eq, '(ADC FS)', mag_label

    if units == 'dbm':
        # Linear I/Q are in ADC-input FS scale; the dBFS→dBm offset and
        # any reference-plane shift are applied at the log-magnitude step
        # in _compute_mag_phase_units so the I-vs-Q geometry is preserved.
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        iq_label = '(ADC FS)'
        mag_label = f'power @ {plane_label} (dBm)'
        return fs_i, fs_q, fs_ei, fs_eq, iq_label, mag_label

    if units == 'volts':
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        iq_label = '(ADC FS)'
        mag_label = f'voltage @ {plane_label} (V RMS)'
        return fs_i, fs_q, fs_ei, fs_eq, iq_label, mag_label

    if units == 'watts':
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        iq_label = '(ADC FS)'
        mag_label = f'power @ {plane_label} (W)'
        return fs_i, fs_q, fs_ei, fs_eq, iq_label, mag_label

    if units in _S21_UNITS:
        plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
        iq_label = '(ADC FS)'
        unit_label = 'dB' if units == 's21_log' else 'linear'
        mag_label = f'|S21| @ {plane_label} ({unit_label})'
        return fs_i, fs_q, fs_ei, fs_eq, iq_label, mag_label

    raise ValueError(f"Unknown units '{units}'. Use one of {UNITS}.")


def _compute_mag_phase_units(z, units, info=None, config=None,
                              reference_plane='adc_input', frequencies=None,
                              unwrap=True, tx_power_dbm=None,
                              calibration_cache=None):
    """Compute magnitude and phase with units-aware magnitude labels.

    For 'raw', 'adc_units', and 'adc_fs', magnitude is 20*log10(|z|).
    ``adc_units`` traces may already have an ADC-unit reference-plane scale
    applied by ``_normalise_iq``.
    For 'dbfs', magnitude is 20*log10(|z|) + plane_offset.
    For 'dbm', magnitude is
    20*log10(|z|) + adc_dbm_to_dbfs + plane_offset, where plane_offset
    shifts from the ADC input to ``reference_plane`` (0 dB for
    ``'adc_input'``; -rx_chain_gain for ``'cryostat_output'``/``'detector'``).
    For 'volts' and 'watts', the calibrated received power is converted to
    linear RMS voltage or watts.  For 's21'/'s21_log', the same received power
    is computed and then ``tx_power_dbm`` at the same reference plane is
    subtracted. For 's21_linear', that log ratio is converted to linear |S21|.

    Must be called with the same (reference_plane, frequencies) that were
    passed to ``_normalise_iq`` — the offset is applied here, not in the
    linear normalisation.  If either the ADC cal or (for non-adc_input
    planes) the RX chain cal is unavailable, the corresponding offset is
    silently omitted so the result degrades gracefully to dBFS; the
    warning has already been emitted by ``_normalise_iq``.
    """
    units = _canonical_units(units)
    mag_db = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z)) if unwrap else np.angle(z)

    if units in _CALIBRATED_MAGNITUDE_UNITS:
        adc_cal_available = True
        if units in _DBM_MAGNITUDE_UNITS:
            cal_db = _resolve_adc_dbm_to_dbfs(
                config, frequencies, info=info,
                calibration_cache=calibration_cache)
            if cal_db is not None:
                mag_db = mag_db + cal_db
            elif units in _S21_UNITS or units in _PHYSICAL_LINEAR_UNITS:
                raise ValueError(
                    f"units='{units}' requires "
                    "config['firmware']['adc_dbm_to_dbfs'] or equivalent "
                    "calibration metadata.")
            else:
                adc_cal_available = False
        plane_offset = _reference_plane_offset_db(
            reference_plane, frequencies, info, config,
            calibration_cache=calibration_cache)
        if plane_offset is not None:
            mag_db = mag_db + plane_offset
        elif units in _S21_UNITS:
            raise ValueError(
                f"reference_plane='{reference_plane}' requested but the RX "
                "analog chain calibration is not available.")
        if units in _S21_UNITS:
            if tx_power_dbm is None:
                raise ValueError(
                    f"units='{units}' requires tx_power_dbm at the requested "
                    "reference plane.")
            tx_power_dbm = np.asarray(tx_power_dbm, dtype=float)
            if tx_power_dbm.size == 1:
                tx_power_dbm = float(tx_power_dbm.ravel()[0])
            else:
                tx_power_dbm = np.broadcast_to(tx_power_dbm, mag_db.shape)
            mag_db = mag_db - tx_power_dbm
            if units == 's21_linear':
                return 10.0 ** (mag_db / 20.0), phase
        elif units == 'watts' and adc_cal_available:
            return _watts_from_dbm(mag_db), phase
        elif units == 'volts' and adc_cal_available:
            impedance = _rf_impedance_ohm(
                info=info, config=config, calibration_cache=calibration_cache)
            return np.sqrt(_watts_from_dbm(mag_db) * impedance), phase

    return mag_db, phase


def _scale_iq_to_magnitude_units(z, units, info=None, config=None,
                                 reference_plane='adc_input',
                                 frequencies=None, tx_power_dbm=None,
                                 calibration_cache=None):
    """Scale linear I/Q values into the coordinates behind mag-unit plots.

    ``_normalise_iq`` intentionally leaves calibrated I/Q traces in ADC-input
    full-scale units because ordinary IQ plots are usually about geometry.
    Combined magnitude/phase/IQ plots are different: the IQ panel should use a
    physical linear coordinate where one exists.  This helper takes the
    already-normalised complex values from ``_normalise_iq`` and applies the
    same reference-plane / transmitted-power offsets used by
    ``_compute_mag_phase_units`` as a linear complex scale.  For
    ``units='watts'``, the magnitude axis is power, but the IQ plane remains a
    linear RMS-voltage phasor.
    """
    units = _canonical_units(units)
    z = np.asarray(z, dtype=complex)
    _validate_reference_plane(reference_plane)
    if units == 'raw':
        return z, ''
    if units == 'peak':
        return z, '(peak norm)'
    if units == 'adc_units':
        return z, '(ADC units)'
    if units == 'adc_fs':
        return z, '(ADC FS)'
    if units not in _CALIBRATED_MAGNITUDE_UNITS:
        raise ValueError(f"Unknown units '{units}'. Use one of {UNITS}.")

    reference_plane = _validated_reference_plane(
        reference_plane, frequencies, info, config,
        strict=(units in _S21_UNITS),
        calibration_cache=calibration_cache)
    plane_offset = _reference_plane_offset_db(
        reference_plane, frequencies, info, config,
        calibration_cache=calibration_cache)
    scale_db = 0.0 if plane_offset is None else plane_offset

    adc_cal = None
    adc_cal_available = True
    if units in _DBM_MAGNITUDE_UNITS:
        adc_cal = _resolve_adc_dbm_to_dbfs(
            config, frequencies, info=info,
            calibration_cache=calibration_cache)
        if adc_cal is None:
            if units in _S21_UNITS or units in _PHYSICAL_LINEAR_UNITS:
                raise ValueError(
                    f"units='{units}' requires "
                    "config['firmware']['adc_dbm_to_dbfs'] or equivalent "
                    "calibration metadata.")
            adc_cal_available = False
        else:
            scale_db = scale_db + adc_cal

    plane_label = _REFERENCE_PLANE_LABELS[reference_plane]
    if units in _S21_UNITS:
        if tx_power_dbm is None:
            raise ValueError(
                f"units='{units}' requires tx_power_dbm at the requested "
                "reference plane.")
        tx_power_dbm = np.asarray(tx_power_dbm, dtype=float)
        if tx_power_dbm.size == 1:
            tx_power_dbm = float(tx_power_dbm.ravel()[0])
        else:
            tx_power_dbm = np.broadcast_to(tx_power_dbm, z.shape)
        scale_db = scale_db - tx_power_dbm
        label = '(S21)'
    elif units == 'dbm' and adc_cal_available:
        impedance = _rf_impedance_ohm(
            info=info, config=config, calibration_cache=calibration_cache)
        scale = np.sqrt(1e-3 * impedance) * (10.0 ** (scale_db / 20.0))
        return z * scale, '(V RMS)'
    elif units == 'volts' and adc_cal_available:
        impedance = _rf_impedance_ohm(
            info=info, config=config, calibration_cache=calibration_cache)
        scale = np.sqrt(1e-3 * impedance) * (10.0 ** (scale_db / 20.0))
        return z * scale, '(V RMS)'
    elif units == 'watts' and adc_cal_available:
        impedance = _rf_impedance_ohm(
            info=info, config=config, calibration_cache=calibration_cache)
        scale = np.sqrt(1e-3 * impedance) * (10.0 ** (scale_db / 20.0))
        return z * scale, '(V RMS)'
    else:
        label = '(ADC FS)' if reference_plane == 'adc_input' else '(FS equiv.)'

    return z * (10.0 ** (scale_db / 20.0)), label


def _magnitude_error_units(magnitude, e_logmag, units):
    """Convert a log-magnitude error to the plotted magnitude units."""
    if _is_s21_linear_unit(units) or _is_voltage_unit(units):
        return np.abs(magnitude) * np.log(10.0) / 20.0 * e_logmag
    if _is_watt_unit(units):
        return np.abs(magnitude) * np.log(10.0) / 10.0 * e_logmag
    return e_logmag


def _apply_compact_scientific_ticks(ax, axis='y', values=None,
                                    low=1e-3, high=1e4, precision=3):
    """Format ticks directly, using scientific notation when useful.

    Matplotlib's default scalar formatter often puts a shared offset/scale
    string beside the axis (for example ``+1e9`` or ``1e6``).  For compact
    diagnostic plots that can be easy to miss, so this helper disables that
    offset text and writes either plain compact values or per-tick scientific
    notation.
    """
    from matplotlib.ticker import FuncFormatter

    axis_obj = ax.yaxis if axis == 'y' else ax.xaxis
    tick_values = np.asarray(axis_obj.get_ticklocs(), dtype=float)
    if values is None:
        check_values = tick_values
    else:
        check_values = np.asarray(values, dtype=float)
    decision_values = tick_values[np.isfinite(tick_values)]
    if decision_values.size == 0:
        decision_values = check_values[np.isfinite(check_values)]
    finite = np.abs(decision_values)
    finite = finite[finite > 0.0]
    use_scientific = bool(
        finite.size
        and (np.nanmin(finite) < float(low) or np.nanmax(finite) >= float(high))
    )
    sci_precision = int(precision)
    if use_scientific:
        finite_ticks = tick_values[np.isfinite(tick_values)]
        finite_ticks = np.unique(finite_ticks)
        finite_ticks = finite_ticks[np.abs(finite_ticks) > 0.0]
        if finite_ticks.size >= 2:
            diffs = np.diff(np.sort(finite_ticks))
            diffs = np.abs(diffs[diffs != 0.0])
            if diffs.size:
                ref = float(np.nanmax(np.abs(finite_ticks)))
                step = float(np.nanmin(diffs))
                if ref > 0.0 and step > 0.0:
                    exponent = int(np.floor(np.log10(ref)))
                    mantissa_step = step / (10.0 ** exponent)
                    if mantissa_step > 0.0:
                        needed = int(np.ceil(-np.log10(mantissa_step))) + 1
                        sci_precision = min(max(sci_precision, needed), 10)

    def _format(value, _pos):
        if not np.isfinite(value):
            return ''
        if abs(value) < np.finfo(float).tiny:
            return '0'
        if use_scientific:
            mantissa, exponent = f'{value:.{sci_precision}e}'.split('e')
            mantissa = mantissa.rstrip('0').rstrip('.')
            return f'{mantissa}e{int(exponent)}'
        text = f'{value:.6g}'
        return '0' if text == '-0' else text

    axis_obj.set_major_formatter(FuncFormatter(_format))
    axis_obj.get_offset_text().set_visible(False)


def _disable_axis_offsets(ax):
    """Disable matplotlib's shared-offset annotation on x/y tick labels.

    Resonator sweeps often span a small interval around a large carrier
    frequency, which makes ``ScalarFormatter`` collapse the labels into a
    short suffix plus an ``+1.5e9``-style offset.  Simply turning the
    offset off leaves the tick labels as long literal values (for example
    ``1500000010.0``); this helper also routes through
    ``_apply_compact_scientific_ticks`` so per-tick scientific notation
    kicks in when the magnitudes warrant it, matching the parameter-plot
    formatting.
    """
    _apply_compact_scientific_ticks(ax, axis='x')
    _apply_compact_scientific_ticks(ax, axis='y')


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
