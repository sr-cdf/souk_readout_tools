"""
Shared utilities for the plotting subpackage.
"""

import numpy as np


# Supported units for normalisation
UNITS = ('raw', 'peak', 'adc_fs', 'dbfs', 'dbm')


def _get_pyplot():
    """Lazy import of matplotlib.pyplot."""
    import matplotlib.pyplot as plt
    return plt


def _compute_mag_phase(z):
    """
    Compute log magnitude (dB) and unwrapped phase from complex S21.

    Args:
        z: Complex array.

    Returns:
        log_mag: 20*log10(|z|) in dB.
        phase: Unwrapped phase in radians.
    """
    log_mag = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z))
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


def _apply_deembedding(frequencies, z, deembed, params=None):
    """
    Apply deembedding if requested.

    Args:
        frequencies: 1D frequency array (Hz).
        z: 1D complex S21 array.
        deembed: bool or dict. If True, compute deembedding. If dict,
                 use as pre-computed params.
        params: Pre-computed deembed params (alternative to passing as
                deembed argument). If provided, applies these params.

    Returns:
        z_out: Processed complex array.
        deembed_params: dict of deembedding parameters, or None.
    """
    if params is not None:
        from .. import resonator
        return resonator.apply_deembed_params(z, params), params

    if deembed is True:
        from .. import resonator
        return resonator.deembed(frequencies, z)
    elif isinstance(deembed, dict):
        from .. import resonator
        return resonator.apply_deembed_params(z, deembed), deembed

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

    rx_mix_scale = 1.0
    if config is not None:
        rx_mix_scale = config.get('firmware', {}).get('rx_mix_scale', 1.0)

    if pre_accumulation:
        acc_gain = 1.0
    else:
        acc_gain = float(info.get('acc_len', 1))

    return pfb_gain * rx_mix_scale * acc_gain


def _normalise_iq(si, sq, units, info=None, config=None,
                  pre_accumulation=False, ei=None, eq=None):
    """Normalise I/Q (and optionally error) arrays to the requested units.

    Args:
        si, sq: I and Q signal arrays.
        units: One of UNITS:
            'raw'    – no normalisation (accumulator codes).
            'peak'   – normalise to the peak magnitude of the data.
            'adc_fs' – fraction of ADC full-scale.
            'dbfs'   – dB relative to ADC full-scale.
            'dbm'    – estimated ADC input power in dBm.
        info: system_information dict.  Required for 'adc_fs', 'dbfs',
            'dbm'; ignored for 'raw' and 'peak'.
        config: Config dict (needed for 'dbm' and non-default rx_mix_scale).
        pre_accumulation: True for snapshot data.
        ei, eq: Optional error arrays (scaled identically for linear units).

    Returns:
        (si, sq, ei, eq, iq_label, mag_label)
        where iq_label is the Y-axis label for I/Q plots and mag_label is
        the Y-axis label for magnitude plots.  ei, eq are None if not supplied.
    """
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

    if units == 'dbfs':
        # Return linear FS values; magnitude will be computed as dBFS
        fs_i = adc_i / half_scale
        fs_q = adc_q / half_scale
        fs_ei = adc_ei / half_scale if adc_ei is not None else None
        fs_eq = adc_eq / half_scale if adc_eq is not None else None
        return fs_i, fs_q, fs_ei, fs_eq, '(FS)', '|S21| (dBFS)'

    if units == 'dbm':
        # Same linear scaling as dbfs; magnitude label changes
        adc_dbm_to_dbfs = 12.0  # default
        if config is not None:
            adc_dbm_to_dbfs = config.get('firmware', {}).get('adc_dbm_to_dbfs', 12.0)
        mixer_qmc_gain = 1.0
        mixer_scale_is_1p0 = False
        if info.get('mixer_qmc_settings_adc') is not None:
            mixer_qmc_gain = info['mixer_qmc_settings_adc'].get('GainCorrectionFactor', 1.0)
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

        # Magnitude in dBFS, then shift to dBm
        # The actual dBFS->dBm conversion is applied when computing magnitude,
        # so we pass the linear FS values and adjust the label.
        return fs_i, fs_q, fs_ei, fs_eq, '(dBm equiv.)', f'(dBm, offset={adc_dbm_to_dbfs:+g})'

    raise ValueError(f"Unknown units '{units}'. Use one of {UNITS}.")


def _compute_mag_phase_units(z, units, info=None, config=None):
    """Compute magnitude and phase with units-aware magnitude labels.

    For 'raw' and 'adc_fs', magnitude is 20*log10(|z|).
    For 'dbfs', magnitude is 20*log10(|z|) (z is already in FS units).
    For 'dbm', magnitude is 20*log10(|z|) + adc_dbm_to_dbfs.
    """
    mag_db = 20 * np.log10(np.abs(z))
    phase = np.unwrap(np.angle(z))

    if units == 'dbm' and info is not None:
        adc_dbm_to_dbfs = 12.0
        if config is not None:
            adc_dbm_to_dbfs = config.get('firmware', {}).get('adc_dbm_to_dbfs', 12.0)
        mag_db = mag_db + adc_dbm_to_dbfs

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
