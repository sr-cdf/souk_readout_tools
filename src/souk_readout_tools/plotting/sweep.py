"""
Sweep data plotting functions.

Supports wideband sweeps (single concatenated trace) and per-tone sweeps
(multiple tones). Formats: I vs Q, I/Q vs frequency, magnitude/phase vs
frequency, all with optional deembedding or phase centering and error bars.
"""

import numpy as np
from ._common import (_get_pyplot, _propagate_errors_mag,
                       _apply_deembed, ERROR_FILL_STYLE, _resolve_label,
                       _normalise_iq, _compute_mag_phase_units, UNITS,
                       _convert_tx_power_reference_plane,
                       _tx_tone_powers_from_info, _canonical_units,
                       _is_s21_unit, _is_calibrated_magnitude_unit,
                       _magnitude_error_units,
                       _scale_iq_to_magnitude_units,
                       _validate_reference_plane,
                       _disable_axis_offsets)


def _extract_traces(sweep_data, tones=None):
    """
    Extract per-trace arrays from a sweep data dict.

    Returns list of (f, si, sq, ei, eq, tone_label) tuples.
    """
    sf = np.atleast_2d(sweep_data['sweep_f'])
    si = np.atleast_2d(sweep_data['sweep_i'])
    sq = np.atleast_2d(sweep_data['sweep_q'])
    ei = np.atleast_2d(sweep_data.get('sweep_ei', np.zeros_like(si)))
    eq = np.atleast_2d(sweep_data.get('sweep_eq', np.zeros_like(sq)))

    is_wideband = sweep_data.get('wideband_sweep', False)

    if is_wideband or sf.shape[0] == 1:
        # Single concatenated trace: shape (1, N)
        return [(sf[0], si[0], sq[0], ei[0], eq[0], None)]

    # Per-tone: shape (N_points, N_tones)
    n_points, n_tones = sf.shape
    if tones is None:
        tones = list(range(n_tones))

    traces = []
    for t in tones:
        traces.append((sf[:, t], si[:, t], sq[:, t],
                        ei[:, t], eq[:, t], t))
    return traces


def _resolve_phase_specs(phase_center=False, phase_rotate=False):
    """Return resolved phase-center and phase-rotate requests."""
    if phase_center is None:
        phase_center = False
    if phase_rotate is None:
        phase_rotate = False
    return phase_center, phase_rotate


def _phase_param_source(phase_center, phase_rotate):
    """Return one shared phase-transform params dict, if provided."""
    sources = [
        spec for spec in (phase_center, phase_rotate)
        if isinstance(spec, dict)
    ]
    if len(sources) > 1 and sources[0] is not sources[1]:
        raise ValueError(
            'phase_center and phase_rotate cannot use different params dicts.'
        )
    return sources[0] if sources else None


def _apply_phase_ops(z, phase_center=False, phase_rotate=False,
                     ei=None, eq=None):
    """Apply optional circle centering and/or rotation to a trace."""
    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    if not phase_center and not phase_rotate:
        return z, ei, eq, None

    from .. import resonator

    params = _phase_param_source(phase_center, phase_rotate)
    if params is None:
        _, params = resonator.phase_center(z)

    z = np.asarray(z, dtype=complex)
    if phase_center:
        z = z - params['center']
    if phase_rotate:
        multiplier = np.exp(1j * params['rotation_angle'])
        z = z * multiplier
        if ei is not None and eq is not None:
            z_err = resonator.transform_s21_error(ei + 1j * eq, multiplier)
            ei = z_err.real
            eq = z_err.imag
    return z, ei, eq, params


def _transform_state_parts(deembed, phase_center=False, phase_rotate=False,
                           *, short=False):
    """Return ordered transform labels for titles or axis annotations."""
    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    parts = []
    if deembed:
        parts.append('demb' if short else 'deembedded')
    if phase_center and phase_rotate:
        parts.append('center+rot' if short else 'centered + rotated')
    elif phase_center:
        parts.append('center' if short else 'centered')
    elif phase_rotate:
        parts.append('rot' if short else 'rotated')
    return parts


def _resolve_axis_phase_specs(default_center, default_rotate,
                              axis_center, axis_rotate):
    """Resolve one mag/phase axis's center/rotate requests."""
    center = default_center if axis_center is None else axis_center
    rotate = default_rotate if axis_rotate is None else axis_rotate
    return _resolve_phase_specs(center, rotate)


def _phase_axis_unwrap(phase_center=False, phase_rotate=False):
    """Return whether the plotted phase axis should be unwrapped."""
    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    # Centering alone moves the circle to the origin but leaves the angular
    # branch cut arbitrary, so unwrap it to show the winding.  Once the trace
    # is explicitly rotated, keep it wrapped so the familiar +/-pi crossing is
    # visible.
    return not bool(phase_rotate)


def _resolve_unwrap_phase(unwrap_phase=None, phase_center=False,
                          phase_rotate=False):
    """Resolve explicit phase wrapping override against the legacy default."""
    if unwrap_phase is None:
        return _phase_axis_unwrap(phase_center, phase_rotate)
    return bool(unwrap_phase)


_MAGPHASE_IQ_FORMAT = 'mag+phase+iq'
_PHYSICAL_IQ_UNITS = ('dbm', 'volts', 'watts')
_SI_PREFIXES = {
    -12: 'p',
    -9: 'n',
    -6: 'u',
    -3: 'm',
    0: '',
    3: 'k',
    6: 'M',
    9: 'G',
}


def _is_magphase_format(format):
    """Return True for formats with magnitude/phase axes."""
    return format in ('magphase', _MAGPHASE_IQ_FORMAT)


def _format_choices():
    return "'iq', 'iq_vs_f', 'magphase', or 'mag+phase+iq'"


def _phase_values(z, unwrap=True):
    """Return phase without running magnitude calibration."""
    phase = np.angle(z)
    return np.unwrap(phase) if unwrap else phase


def _is_physical_iq_unit(units):
    """Return True when IQ axes should be converted to physical SI units."""
    return _canonical_units(units) in _PHYSICAL_IQ_UNITS


def _axis_coordinate_values(*values):
    """Flatten real axis coordinates from real or complex arrays."""
    pieces = []
    for value in values:
        if value is None:
            continue
        arr = np.asarray(value)
        if np.iscomplexobj(arr):
            pieces.append(arr.real.ravel())
            pieces.append(arr.imag.ravel())
        else:
            pieces.append(arr.ravel())
    if not pieces:
        return np.array([], dtype=float)
    return np.concatenate([np.asarray(piece, dtype=float) for piece in pieces])


def _si_prefix_for_values(*values):
    """Return ``(scale, prefix)`` for compact SI axis labels."""
    coords = _axis_coordinate_values(*values)
    finite = np.abs(coords[np.isfinite(coords)])
    finite = finite[finite > 0.0]
    if finite.size == 0:
        return 1.0, ''
    exponent = int(np.floor(np.log10(np.max(finite)) / 3.0) * 3)
    exponent = min(max(exponent, min(_SI_PREFIXES)), max(_SI_PREFIXES))
    return 10.0 ** exponent, _SI_PREFIXES[exponent]


def _physical_base_unit(units, label=None):
    """Return the base unit text for physical linear plotting units."""
    if label is not None:
        if 'V RMS' in label:
            return 'V RMS'
        if 'W' in label:
            return 'W'
    units = _canonical_units(units)
    if units in ('dbm', 'volts'):
        return 'V RMS'
    if units == 'watts':
        return 'W'
    return None


def _prefix_physical_label(label, units, prefix, base_unit=None):
    """Apply an SI prefix to a physical axis label."""
    base_unit = base_unit or _physical_base_unit(units, label=label)
    if base_unit is None:
        return label
    prefixed = f'{prefix}{base_unit}' if prefix else base_unit
    return label.replace(base_unit, prefixed, 1)


def _physical_si_state(ax, units, label, *scale_values):
    """Return/create one stable SI prefix state for a physical axis block."""
    units = _canonical_units(units)
    if units not in _PHYSICAL_IQ_UNITS:
        return None, label
    base_unit = _physical_base_unit(units, label=label)
    if base_unit not in label:
        return None, label

    states = getattr(ax, '_souk_physical_si_prefixes', None)
    if states is None:
        states = {}
        setattr(ax, '_souk_physical_si_prefixes', states)
    state = states.get(base_unit)
    if state is None:
        scale, prefix = _si_prefix_for_values(*scale_values)
        state = {'base_unit': base_unit, 'scale': scale, 'prefix': prefix}
        states[base_unit] = state
    return state, _prefix_physical_label(
        label, units, state['prefix'], base_unit=base_unit)


def _apply_physical_si_prefix(ax, values, units, label, *scale_values):
    """Scale physical plot values to the shared SI prefix for this axis block."""
    state, label = _physical_si_state(ax, units, label, values, *scale_values)
    if state is None:
        return values, label
    return values / state['scale'], label


def _apply_physical_si_prefix_with_scale(
        ax, values, units, label, *scale_values):
    """Scale physical values and return the scale that was used."""
    state, label = _physical_si_state(ax, units, label, values, *scale_values)
    if state is None:
        return values, label, None
    return values / state['scale'], label, state['scale']


def _apply_iq_si_prefix(ax, z, units, label, *scale_values):
    """Scale physical IQ coordinates to one stable SI prefix per axis."""
    return _apply_physical_si_prefix(ax, z, units, label, *scale_values)


def _apply_iq_si_prefix_with_scale(ax, z, units, label, *scale_values):
    """Scale physical IQ coordinates and return the scale that was used."""
    return _apply_physical_si_prefix_with_scale(
        ax, z, units, label, *scale_values)


def _validate_magnitude_unit_transforms(format, units, deembed=False,
                                        phase_center=False,
                                        mag_centered=None,
                                        reference_plane='adc_input'):
    """Reject transform/unit combinations that would relabel non-physical data."""
    units = _canonical_units(units)
    _validate_reference_plane(reference_plane)
    if units == 'raw' and reference_plane != 'adc_input':
        raise ValueError(
            "units='raw' always means accumulator units and does not have a "
            "detector/cryostat reference plane. Use units='adc_units' to "
            "undo the firmware accumulator path and refer linear ADC units "
            f"to reference_plane={reference_plane!r}.")
    if _is_s21_unit(units) and not _is_magphase_format(format):
        raise ValueError(
            f"units='{units}' is only supported for magnitude/phase formats, "
            "where it can be computed as received power minus transmitted "
            "power.")

    if not _is_calibrated_magnitude_unit(units):
        return

    if deembed and not _is_magphase_format(format):
        raise ValueError(
            f"units='{units}' reports calibrated received magnitude and "
            "cannot be combined with deembed=True for this format. Use "
            "format='magphase' to keep the magnitude physical while applying "
            "deembedding to the phase axis.")

    if _is_magphase_format(format):
        mag_phase_center, _ = _resolve_axis_phase_specs(
            phase_center, False, mag_centered, None)
        if mag_phase_center:
            raise ValueError(
                f"units='{units}' reports calibrated received magnitude and "
                "cannot use magnitude centering because centering changes "
                "the plotted magnitude. Set mag_centered=False to keep the "
                "magnitude physical while centering the phase axis.")


def _magphase_axis_deembed(units, deembed):
    """Return separate deembed settings for mag/phase axes."""
    if _is_calibrated_magnitude_unit(units):
        return False, deembed
    return deembed, deembed


def _format_axis_deembed(format, units, deembed):
    """Return mag/phase deembed settings for the requested format."""
    if format == _MAGPHASE_IQ_FORMAT:
        return deembed, deembed
    return _magphase_axis_deembed(units, deembed)


def _check_tx_power_reference(reference_plane, tx_power_reference_plane):
    """Validate that TX and RX powers are being compared at the same plane."""
    if tx_power_reference_plane is None:
        return
    if str(tx_power_reference_plane) != str(reference_plane):
        raise ValueError(
            "units='s21' could not resolve tx_power_dbm at the same "
            "reference_plane as the received-power plot. Got "
            f"tx_power_reference_plane={tx_power_reference_plane!r} and "
            f"reference_plane={reference_plane!r}.")


def _resolve_tx_power_dbm(tx_power_dbm, tx_power_reference_plane,
                          reference_plane, info=None, config=None,
                          frequencies=None, calibration_cache=None):
    """Resolve TX power at the same reference plane as the plotted RX power."""
    if tx_power_dbm is not None:
        tx_arr = np.asarray(tx_power_dbm, dtype=float)
        if tx_arr.size and np.any(np.isfinite(tx_arr)):
            converted = _convert_tx_power_reference_plane(
                tx_power_dbm, tx_power_reference_plane, reference_plane,
                info=info, config=config, frequencies=frequencies,
                calibration_cache=calibration_cache)
            if converted is not None:
                return converted, reference_plane
    info_power = _tx_tone_powers_from_info(
        info, reference_plane=reference_plane, config=config,
        frequencies=frequencies, calibration_cache=calibration_cache)
    if info_power is not None:
        return info_power, reference_plane
    if tx_power_dbm is None:
        return None, tx_power_reference_plane
    return tx_power_dbm, tx_power_reference_plane


def _select_tx_power_dbm(tx_power_dbm, *, trace_position=None,
                         tone_index=None, total=None):
    """Select a scalar TX power for one plotted trace."""
    if tx_power_dbm is None:
        return None
    arr = np.asarray(tx_power_dbm, dtype=float).ravel()
    if arr.size == 0:
        return None
    if arr.size == 1:
        return float(arr[0])
    if total is not None and arr.size == int(total) and trace_position is not None:
        return float(arr[int(trace_position)])
    if tone_index is not None:
        try:
            tone_index = int(tone_index)
        except (TypeError, ValueError):
            tone_index = None
        if tone_index is not None and 0 <= tone_index < arr.size:
            return float(arr[tone_index])
    raise ValueError(
        "tx_power_dbm must be a scalar, one value per plotted trace, or one "
        "value per tone when units='s21'.")


def _sweep_tx_power_dbm(sweep_data, reference_plane='detector',
                        config=None, use_readback=True,
                        calibration_cache=None):
    """Return per-tone TX powers for a sweep at the requested plot plane."""
    info = sweep_data.get('info')
    powers, plane = _resolve_tx_power_dbm(
        None, None, reference_plane, info=info, config=config,
        calibration_cache=calibration_cache)
    if powers is not None:
        return powers, plane
    keys = (
        ('readback_tone_powers_dbm', 'requested_tone_powers_dbm')
        if use_readback else
        ('requested_tone_powers_dbm', 'readback_tone_powers_dbm')
    )
    for key in keys:
        powers = sweep_data.get(key)
        if powers is not None:
            powers = np.asarray(powers, dtype=float).ravel()
            if powers.size:
                source_plane = sweep_data.get('tone_powers_reference_plane')
                powers, plane = _resolve_tx_power_dbm(
                    powers, source_plane, reference_plane, info=info,
                    config=config, calibration_cache=calibration_cache)
                return powers, plane
    return None, None


def _apply_transforms(f, z, deembed, phase_center, phase_rotate=False,
                      ei=None, eq=None, group_delay_cal=None):
    """Apply deembed and optional phase operations to a trace and errors.

    Returns (z_out, ei_out, eq_out, deembed_params, phase_params).
    ei_out/eq_out are None when the input errors are None.
    """
    d_params = None
    pc_params = None
    if deembed:
        if ei is None or eq is None:
            z, d_params = _apply_deembed(f, z, deembed,
                                         group_delay_cal=group_delay_cal)
        else:
            from .. import resonator
            z_err = ei + 1j * eq
            if deembed is True:
                z, z_err, d_params = resonator.deembed(
                    f, z, group_delay_cal=group_delay_cal, s21_err=z_err)
            elif isinstance(deembed, dict):
                z, z_err = resonator.apply_deembed_params(
                    z, deembed, s21_err=z_err)
                d_params = deembed
            else:
                z, d_params = _apply_deembed(f, z, deembed,
                                             group_delay_cal=group_delay_cal)
            if d_params is not None:
                ei = z_err.real
                eq = z_err.imag
    elif group_delay_cal is not None:
        # Remove group delay without full deembedding (no baseline
        # normalisation).  This corrects the linear phase slope so that
        # phase-vs-frequency plots show the resonator response only.
        from ..resonator import remove_group_delay
        if ei is None or eq is None:
            z, _ = remove_group_delay(f, z, group_delay_cal)
        else:
            z, z_err, _ = remove_group_delay(
                f, z, group_delay_cal, s21_err=ei + 1j * eq)
            ei = z_err.real
            eq = z_err.imag
    z, ei, eq, pc_params = _apply_phase_ops(
        z, phase_center=phase_center, phase_rotate=phase_rotate,
        ei=ei, eq=eq)
    return z, ei, eq, d_params, pc_params


def _transform_title_suffix(deembed, phase_center, phase_rotate=False):
    """Return a parenthesised title suffix describing active transforms."""
    parts = _transform_state_parts(
        deembed, phase_center, phase_rotate, short=False)
    return f' ({", ".join(parts)})' if parts else ''


def _axis_label_suffix(deembed, phase_center, phase_rotate=False):
    """Return a short second-line suffix for axis-frame transforms."""
    parts = _transform_state_parts(
        False, phase_center, phase_rotate, short=True)
    return f'\n({", ".join(parts)})' if parts else ''


def _has_offset_coordinate_transform(phase_center=False):
    """Return True when the plotted coordinates have an explicit offset."""
    phase_center, _ = _resolve_phase_specs(phase_center, False)
    return bool(phase_center)


def _iq_axis_name(component, phase_center=False, phase_rotate=False):
    """Return an I/Q coordinate name with relative coordinates made explicit."""
    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    if phase_center:
        return f'Delta {component}'
    if phase_rotate:
        return f'Rotated {component}'
    return component


def _iq_axis_text(component, iq_axis_label, phase_center=False,
                  phase_rotate=False, iq_suffix=''):
    """Return a full I/Q axis label."""
    label = f'{_iq_axis_name(component, phase_center, phase_rotate)} {iq_axis_label}'
    return label.strip() + iq_suffix


def _offset_magnitude_label(label, phase_center=False, phase_rotate=False):
    """Return a magnitude label that makes transformed coordinates explicit."""
    if not _has_offset_coordinate_transform(phase_center):
        return label
    replacements = (
        ('|S21|', '|Delta S21|'),
        ('|RX|', '|Delta RX|'),
        ('voltage ', 'voltage offset '),
        ('power ', 'power offset '),
    )
    for old, new in replacements:
        if label.startswith(old):
            return label.replace(old, new, 1)
    return f'offset {label}'


def _phase_axis_label(phase_center=False, phase_rotate=False):
    """Return a phase axis label with relative phase made explicit."""
    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    if phase_center:
        return 'Offset phase (rad)'
    if phase_rotate:
        return 'Rotated phase (rad)'
    return 'Phase (rad)'


def _deembedded_iq_label(deembed):
    """Return an IQ label for traces already normalised by deembedding."""
    return '(S21)' if deembed else None


def _deembedded_trace_labels():
    """Return labels for traces already normalised by deembedding."""
    return '(S21)', '|S21| (dB)'


def _deembedded_magnitude_values(z, units):
    """Return deembedded magnitude values and label for a complex trace."""
    if _deembedded_linear_magnitude_unit(units):
        return np.abs(z), '|S21| (linear, deembedded)'
    return 20.0 * np.log10(np.abs(z)), '|S21| (dB, deembedded)'


def _deembedded_linear_magnitude_unit(units):
    """Return True when deembedded magnitude should stay on a linear axis."""
    return _canonical_units(units) in ('s21_linear', 'volts')


def _magnitude_error_unit_key(format, units, mag_deembed):
    """Return the unit key matching the already-computed magnitude values."""
    if format == _MAGPHASE_IQ_FORMAT and mag_deembed:
        return 's21_linear' if _deembedded_linear_magnitude_unit(units) else 's21'
    return units


def _configure_magphase_iq_axis(ax):
    """Keep combined-format IQ y-axis labels away from right-side colorbars."""
    ax.yaxis.set_label_position('left')
    ax.yaxis.tick_left()
    ax.yaxis.labelpad = 6
    ax.tick_params(
        axis='y', left=True, right=False, labelleft=True, labelright=False)


def _iq_ylabel_text(iq_axis_label, iq_suffix='', phase_center=False,
                    phase_rotate=False):
    """Return a compact, wrapped Q-axis label for the combined IQ panel."""
    label = _iq_axis_text(
        'Q', iq_axis_label, phase_center=phase_center,
        phase_rotate=phase_rotate)
    if len(label) > 28:
        label = label.replace(' @ ', '\n@ ', 1)
    if len(label) > 38:
        label = label.replace(', ', ',\n', 1)
    return label + iq_suffix


def _magphase_iq_axes(plt, fig=None, figsize=None):
    """Return magnitude, phase, and full-height IQ axes."""
    if fig is not None:
        if len(fig.axes) < 3:
            raise ValueError(
                "format='mag+phase+iq' requires a figure with at least "
                "three axes: magnitude, phase, and IQ."
            )
        _configure_magphase_iq_axis(fig.axes[2])
        return fig, fig.axes[0], fig.axes[1], fig.axes[2]
    fig = plt.figure(figsize=figsize or (12, 6))
    gs = fig.add_gridspec(
        2, 2,
        width_ratios=(1.45, 1.0),
        height_ratios=(1.0, 1.0),
        hspace=0.08,
        wspace=0.42,
    )
    ax_mag = fig.add_subplot(gs[0, 0])
    ax_phase = fig.add_subplot(gs[1, 0], sharex=ax_mag)
    ax_iq = fig.add_subplot(gs[:, 1])
    ax_mag.tick_params(labelbottom=False)
    _configure_magphase_iq_axis(ax_iq)
    return fig, ax_mag, ax_phase, ax_iq


def _magphase_iq_grid_axes(plt, n_traces, fig=None, figsize=None):
    """Return per-trace magnitude/phase/IQ axes for grid plotting."""
    if fig is not None:
        needed = 3 * int(n_traces)
        if len(fig.axes) < needed:
            raise ValueError(
                "format='mag+phase+iq' grid plots require three axes per "
                f"trace; got {len(fig.axes)} axes for {n_traces} traces."
            )
        for i in range(int(n_traces)):
            _configure_magphase_iq_axis(fig.axes[3 * i + 2])
        return fig, [
            tuple(fig.axes[3 * i:3 * i + 3])
            for i in range(int(n_traces))
        ]

    fig = plt.figure(figsize=figsize or (12, 3.4 * int(n_traces)))
    gs = fig.add_gridspec(
        max(2 * int(n_traces), 1), 2,
        width_ratios=(1.45, 1.0),
        hspace=0.18,
        wspace=0.42,
    )
    axes = []
    for i in range(int(n_traces)):
        ax_mag = fig.add_subplot(gs[2 * i, 0])
        ax_phase = fig.add_subplot(gs[2 * i + 1, 0], sharex=ax_mag)
        ax_iq = fig.add_subplot(gs[2 * i:2 * i + 2, 1])
        ax_mag.tick_params(labelbottom=False)
        _configure_magphase_iq_axis(ax_iq)
        axes.append((ax_mag, ax_phase, ax_iq))
    return fig, axes


def plot_sweep(sweep_data, format='magphase', tones=None, deembed=False,
               phase_center=False, phase_rotate=False,
               show_errors=None, multi_tone='overlay',
               fig=None, label=None, units='raw', config=None,
               reference_plane='adc_input', group_delay_cal=None,
               mag_centered=None, phase_centered=None,
               mag_rotated=None, phase_rotated=None,
               unwrap_phase=None, tx_power_dbm=None,
               tx_power_reference_plane=None, **kwargs):
    """
    General-purpose sweep plot.

    Args:
        sweep_data: dict with keys 'sweep_f', 'sweep_i', 'sweep_q',
            optionally 'sweep_ei', 'sweep_eq'.
        format: 'iq' | 'iq_vs_f' | 'magphase' | 'mag+phase+iq'
        tones: List of tone indices to plot, or None for all.
        deembed: bool, apply true RF deembedding (cable delay removal +
            baseline normalisation).  Off-resonance → (1, 0).  For
            ``format='magphase'`` with calibrated magnitude units
            (``'dbfs'``, ``'dbm'``, ``'volts'``, ``'watts'``, ``'s21'``),
            deembedding is applied to the phase axis only so the magnitude
            remains received power.  For ``format='mag+phase+iq'``,
            deembedding is applied to magnitude, phase, and IQ together so all
            panels share one complex frame.
        phase_center: bool or dict, apply the circle-centering step after
            deembedding.
        phase_rotate: bool or dict, apply the post-centering rotation step.
        show_errors: True/False/None.  None (default) draws error fills for
            per-tone sweeps but skips them for wideband sweeps, where the
            concatenated trace can be hundreds of thousands of points and
            ``fill_between`` becomes the dominant render cost.  Pass ``True``
            to force errors on (slow for large wideband sweeps) or ``False``
            to always skip.
        multi_tone: 'overlay' (shared axes) or 'grid' (one subplot per tone).
        fig: Existing figure. If None, create new.
        label: Legend label. If None, uses an auto-incrementing index.
        units: Unit for I/Q normalisation.  One of:
            'raw' (default) - accumulator units. ``reference_plane`` must be
                ``'adc_input'`` because raw values are not calibrated to a
                detector/cryostat plane.
            'peak' - normalise to the peak magnitude of the data.
            'adc_units' / 'adc' - linear ADC units.  The firmware
                accumulator path is undone and, for non-ADC reference planes,
                the RX-chain calibration is removed.
            'adc_fs' - fraction of ADC full-scale.
            'dbfs' - dB relative to ADC full-scale, optionally referred to
                ``reference_plane`` for magnitude plots.
            'dbm' - estimated power in dBm at ``reference_plane``;
                IQ axes use an SI-prefixed RMS voltage scale.
            'volts' / 'v' - estimated RMS voltage at ``reference_plane``;
                IQ axes use an SI-prefixed voltage scale.
            'watts' / 'w' - estimated power at ``reference_plane``;
                IQ axes still use an SI-prefixed RMS voltage scale.
            's21_log' / 's21' - received power at ``reference_plane`` minus
                known transmitted power at the same plane, in dB.  Requires
                ``tx_power_dbm`` or sweep tone-power metadata.
            's21_linear' - the same calibrated |S21| ratio in linear units.
            All options except 'raw' and 'peak' require 'info' in sweep_data;
            non-ADC reference planes use 'info' or ``config`` when calibration
            is required.
        config: Config dict (needed when the sweep metadata does not include
            the run config, or for non-default rx_mix_scale in older data).
        reference_plane: Reference plane for calibrated magnitude plotting.
            Use 'adc_input' (default), 'cryostat_output', or 'detector'.  The
            latter two remove the RX analog-chain gain using frequency-
            dependent calibration entries in ``config`` or
            ``sweep_data['info']``; ``'s21'`` raises if that calibration is
            not available.  ``units='raw'`` does not use this argument beyond
            validation; use ``units='adc_units'`` for a linear ADC-unit view
            referred to ``reference_plane``.
        group_delay_cal: Frequency-dependent group delay calibration from
            ``measure_path_group_delay()``.  Used when ``deembed=True`` to
            remove the measured path group delay instead of auto-estimating
            a scalar cable delay.
        unwrap_phase: bool or None, optional.  ``None`` preserves the
            automatic behaviour: unwrap unrotated phase traces, including
            center-only traces, but leave explicitly rotated phase wrapped.
            Pass ``True`` or ``False`` to force the phase subplot behaviour.
        tx_power_dbm: Scalar or per-tone transmitted power in dBm, used only
            with ``units='s21'``.  If omitted, ``sweep_data['info']`` tone
            powers are resolved at ``reference_plane`` when possible, then
            stored ``readback_tone_powers_dbm``/``requested_tone_powers_dbm``
            are used as a fallback.
        tx_power_reference_plane: Reference plane for explicit
            ``tx_power_dbm``.  If it differs from ``reference_plane``, the
            structured info/config calibration is used to convert it when
            possible.
        **kwargs: Passed to matplotlib plot/errorbar calls.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    custom_title = kwargs.pop('title', None)
    units = _canonical_units(units)
    calibration_cache = {}
    traces = _extract_traces(sweep_data, tones)
    info = sweep_data.get('info')
    _validate_magnitude_unit_transforms(
        format, units, deembed=deembed, phase_center=phase_center,
        mag_centered=mag_centered, reference_plane=reference_plane)

    if _is_s21_unit(units):
        if tx_power_dbm is None:
            tx_power_dbm, tx_power_reference_plane = _sweep_tx_power_dbm(
                sweep_data, reference_plane=reference_plane, config=config,
                calibration_cache=calibration_cache)
        else:
            tx_power_dbm, tx_power_reference_plane = _resolve_tx_power_dbm(
                tx_power_dbm, tx_power_reference_plane, reference_plane,
                info=info, config=config,
                calibration_cache=calibration_cache)
        _check_tx_power_reference(reference_plane, tx_power_reference_plane)

    # Normalise traces
    if units == 'raw':
        normalised = []
        for f, si, sq, ei, eq, tidx in traces:
            si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
                si, sq, units, info, config=config, ei=ei, eq=eq,
                reference_plane=reference_plane, frequencies=f,
                calibration_cache=calibration_cache)
            normalised.append((f, si, sq, ei, eq, tidx))
        traces = normalised
    else:
        if units != 'peak' and info is None:
            raise ValueError("sweep_data must contain 'info' for non-raw units.")
        normalised = []
        for f, si, sq, ei, eq, tidx in traces:
            si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
                si, sq, units, info, config=config, ei=ei, eq=eq,
                reference_plane=reference_plane, frequencies=f,
                calibration_cache=calibration_cache)
            normalised.append((f, si, sq, ei, eq, tidx))
        traces = normalised

    if show_errors is None:
        # Auto: skip error fills on wideband sweeps where fill_between is slow.
        show_errors = not sweep_data.get('wideband_sweep', False)
    has_errors = show_errors and any(np.any(ei != 0) for _, _, _, ei, _, _ in traces)
    n_traces = len(traces)
    is_multi = n_traces > 1 and traces[0][5] is not None
    suffix = _transform_title_suffix(deembed, phase_center, phase_rotate)

    # Determine subplot layout
    if is_multi and multi_tone == 'grid':
        n_cols = min(n_traces, 4)
        n_rows = int(np.ceil(n_traces / n_cols))
        if format == 'iq':
            if fig is None:
                fig, axes = plt.subplots(n_rows, n_cols, figsize=(4 * n_cols, 4 * n_rows))
            else:
                axes = np.array(fig.axes).reshape(n_rows, n_cols)
            axes_flat = np.atleast_1d(axes).ravel()
            for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
                ax = axes_flat[i]
                z = si + 1j * sq
                z, _, _, _, _ = _apply_transforms(
                    f, z, deembed, phase_center, phase_rotate,
                    group_delay_cal=group_delay_cal)
                axis_iq_label = iq_label
                if _is_physical_iq_unit(units):
                    z, axis_iq_label = _scale_iq_to_magnitude_units(
                        z, units, info=info, config=config,
                        reference_plane=reference_plane, frequencies=f,
                        calibration_cache=calibration_cache)
                    z, axis_iq_label = _apply_iq_si_prefix(
                        ax, z, units, axis_iq_label)
                trace_label = _resolve_label(ax, label, suffix=f'Tone {tidx}' if tidx is not None else None)
                ax.plot(z.real, z.imag, linewidth=0.8, label=trace_label, **kwargs)
                ax.set_aspect('equal', adjustable='datalim')
                ax.set_title(f'Tone {tidx}')
                iq_suffix = _axis_label_suffix(
                    deembed, phase_center, phase_rotate)
                ax.set_xlabel(_iq_axis_text(
                    'I', axis_iq_label, phase_center=phase_center,
                    phase_rotate=phase_rotate, iq_suffix=iq_suffix))
                ax.set_ylabel(_iq_axis_text(
                    'Q', axis_iq_label, phase_center=phase_center,
                    phase_rotate=phase_rotate, iq_suffix=iq_suffix))
                ax.legend(fontsize='small')
            # Hide unused axes
            for i in range(n_traces, len(axes_flat)):
                axes_flat[i].set_visible(False)
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig
        if format == _MAGPHASE_IQ_FORMAT:
            fig, axis_blocks = _magphase_iq_grid_axes(plt, n_traces, fig=fig)
            for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
                ax_mag, ax_phase, ax_iq = axis_blocks[i]
                trace_label = _resolve_label(
                    ax_mag, label,
                    suffix=f'Tone {tidx}' if tidx is not None else None)
                trace_tx_power = (
                    _select_tx_power_dbm(
                        tx_power_dbm, trace_position=i, tone_index=tidx,
                        total=n_traces)
                    if _is_s21_unit(units) else None
                )
                _plot_single_trace(
                    ax_mag, ax_phase, f, si, sq, ei, eq,
                    format, deembed, phase_center, phase_rotate,
                    has_errors, trace_label,
                    ax3=ax_iq,
                    units=units, info=info, config=config,
                    reference_plane=reference_plane,
                    iq_label=iq_label, mag_label=mag_label,
                    group_delay_cal=group_delay_cal,
                    mag_centered=mag_centered,
                    phase_centered=phase_centered,
                    mag_rotated=mag_rotated,
                    phase_rotated=phase_rotated,
                    unwrap_phase=unwrap_phase,
                    tx_power_dbm=trace_tx_power,
                    calibration_cache=calibration_cache,
                    **kwargs)
                ax_mag.set_title(f'Tone {tidx}')
                ax_mag.legend(fontsize='small')
                ax_phase.legend(fontsize='small')
                ax_iq.legend(fontsize='small')
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig

        else:
            # Two rows per tone for dual-axis formats
            if fig is None:
                fig, axes = plt.subplots(n_traces, 2, figsize=(12, 3 * n_traces),
                                         squeeze=False)
            else:
                axes = np.array(fig.axes).reshape(n_traces, 2)
            for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
                trace_label = _resolve_label(axes[i, 0], label,
                                             suffix=f'Tone {tidx}' if tidx is not None else None)
                trace_tx_power = (
                    _select_tx_power_dbm(
                        tx_power_dbm, trace_position=i, tone_index=tidx,
                        total=n_traces)
                    if _is_s21_unit(units) else None
                )
                _plot_single_trace(axes[i, 0], axes[i, 1], f, si, sq, ei, eq,
                                   format, deembed, phase_center,
                                   phase_rotate,
                                   has_errors, trace_label,
                                   units=units, info=info, config=config,
                                   reference_plane=reference_plane,
                                   iq_label=iq_label, mag_label=mag_label,
                                   group_delay_cal=group_delay_cal,
                                   mag_centered=mag_centered,
                                   phase_centered=phase_centered,
                                   mag_rotated=mag_rotated,
                                   phase_rotated=phase_rotated,
                                   unwrap_phase=unwrap_phase,
                                   tx_power_dbm=trace_tx_power,
                                   calibration_cache=calibration_cache,
                                   **kwargs)
                axes[i, 0].legend(fontsize='small')
                axes[i, 1].legend(fontsize='small')
            if custom_title is not None:
                fig.suptitle(custom_title)
            plt.tight_layout()
            return fig

    # Overlay or single trace
    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=(8, 8))
        else:
            ax = fig.gca()
        axis_iq_label = iq_label
        for f, si, sq, ei, eq, tidx in traces:
            z = si + 1j * sq
            z, _, _, _, _ = _apply_transforms(
                f, z, deembed, phase_center, phase_rotate,
                group_delay_cal=group_delay_cal)
            if _is_physical_iq_unit(units):
                z, axis_iq_label = _scale_iq_to_magnitude_units(
                    z, units, info=info, config=config,
                    reference_plane=reference_plane, frequencies=f,
                    calibration_cache=calibration_cache)
                z, axis_iq_label = _apply_iq_si_prefix(
                    ax, z, units, axis_iq_label)
            trace_label = _resolve_label(ax, label,
                                         suffix=f'Tone {tidx}' if tidx is not None else None)
            ax.plot(z.real, z.imag, linewidth=0.8, label=trace_label, **kwargs)
        iq_suffix = _axis_label_suffix(deembed, phase_center, phase_rotate)
        ax.set_xlabel(_iq_axis_text(
            'I', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax.set_ylabel(_iq_axis_text(
            'Q', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax.set_aspect('equal', adjustable='datalim')
        ax.legend(fontsize='small')
        ax.set_title(custom_title if custom_title is not None
                     else 'Complex Plane' + suffix)
        plt.tight_layout()
        return fig

    # Combined magnitude/phase plus IQ layout
    if format == _MAGPHASE_IQ_FORMAT:
        fig, ax1, ax2, ax3 = _magphase_iq_axes(plt, fig=fig)
        for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
            trace_label = _resolve_label(
                ax1, label,
                suffix=f'Tone {tidx}' if tidx is not None else None)
            trace_tx_power = (
                _select_tx_power_dbm(
                    tx_power_dbm, trace_position=i, tone_index=tidx,
                    total=n_traces)
                if _is_s21_unit(units) else None
            )
            _plot_single_trace(
                ax1, ax2, f, si, sq, ei, eq,
                format, deembed, phase_center, phase_rotate,
                has_errors, trace_label,
                ax3=ax3,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                iq_label=iq_label, mag_label=mag_label,
                group_delay_cal=group_delay_cal,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                unwrap_phase=unwrap_phase,
                tx_power_dbm=trace_tx_power,
                calibration_cache=calibration_cache,
                **kwargs)

        ax1.legend(fontsize='small')
        ax2.legend(fontsize='small')
        ax3.legend(fontsize='small')
        if custom_title is not None:
            title = custom_title
        else:
            bw = sweep_data.get('bandwidth_hz')
            title = f'Sweep ({bw/1e6:.1f} MHz)' if bw else 'Sweep'
            title += suffix
        fig.suptitle(title)
        plt.tight_layout()
        return fig

    # Dual-axis formats: magphase or iq_vs_f
    if fig is None:
        fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True, figsize=(10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for i, (f, si, sq, ei, eq, tidx) in enumerate(traces):
        trace_label = _resolve_label(ax1, label,
                                     suffix=f'Tone {tidx}' if tidx is not None else None)
        trace_tx_power = (
            _select_tx_power_dbm(
                tx_power_dbm, trace_position=i, tone_index=tidx,
                total=n_traces)
            if _is_s21_unit(units) else None
        )
        _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                           format, deembed, phase_center,
                           phase_rotate,
                           has_errors, trace_label,
                           units=units, info=info, config=config,
                           reference_plane=reference_plane,
                           iq_label=iq_label, mag_label=mag_label,
                           group_delay_cal=group_delay_cal,
                           mag_centered=mag_centered,
                           phase_centered=phase_centered,
                           mag_rotated=mag_rotated,
                           phase_rotated=phase_rotated,
                           unwrap_phase=unwrap_phase,
                           tx_power_dbm=trace_tx_power,
                           calibration_cache=calibration_cache,
                           **kwargs)

    ax1.legend(fontsize='small')
    ax2.legend(fontsize='small')

    if custom_title is not None:
        title = custom_title
    else:
        bw = sweep_data.get('bandwidth_hz')
        title = f'Sweep ({bw/1e6:.1f} MHz)' if bw else 'Sweep'
        title += suffix
    fig.suptitle(title)
    plt.tight_layout()
    return fig


def _plot_single_trace(ax1, ax2, f, si, sq, ei, eq,
                        format, deembed, phase_center, phase_rotate,
                        has_errors, label,
                        ax3=None,
                        units='raw', info=None, config=None,
                        reference_plane='adc_input',
                        iq_label='', mag_label='|RX| (dB)',
                        group_delay_cal=None,
                        mag_centered=None, phase_centered=None,
                        mag_rotated=None, phase_rotated=None,
                        unwrap_phase=None, tx_power_dbm=None,
                        calibration_cache=None,
                        **kwargs):
    """Plot a single trace on a pair of axes."""
    z_in = si + 1j * sq

    if _is_magphase_format(format):
        mag_deembed, phase_deembed = _format_axis_deembed(
            format, units, deembed)
        mc, mrot = _resolve_axis_phase_specs(
            phase_center, phase_rotate, mag_centered, mag_rotated)
        pc, prot = _resolve_axis_phase_specs(
            phase_center, phase_rotate, phase_centered, phase_rotated)
        phase_unwrap = _resolve_unwrap_phase(unwrap_phase, pc, prot)
        z_mag, ei_mag, eq_mag, _, _ = _apply_transforms(
            f, z_in, mag_deembed, mc, mrot, ei, eq,
            group_delay_cal=group_delay_cal)
        if mag_deembed == phase_deembed and mc == pc and mrot == prot:
            z_phase, ei_phase, eq_phase = z_mag, ei_mag, eq_mag
        else:
            z_phase, ei_phase, eq_phase, _, _ = _apply_transforms(
                f, z_in, phase_deembed, pc, prot, ei, eq,
                group_delay_cal=group_delay_cal)
        f_mhz = f / 1e6
        if format == _MAGPHASE_IQ_FORMAT and mag_deembed:
            mag_db, mag_axis_label = _deembedded_magnitude_values(
                z_mag, units)
        else:
            mag_db, _ = _compute_mag_phase_units(
                z_mag, units, info=info, config=config,
                reference_plane=reference_plane, frequencies=f,
                tx_power_dbm=tx_power_dbm,
                calibration_cache=calibration_cache)
            mag_axis_label = mag_label
        if format == _MAGPHASE_IQ_FORMAT:
            if ax3 is None:
                raise ValueError(
                    "format='mag+phase+iq' requires an IQ axis."
                )
            mag_db, mag_axis_label = _apply_physical_si_prefix(
                ax3, mag_db, units, mag_axis_label)
        phase = _phase_values(z_phase, unwrap=phase_unwrap)
        if has_errors and ei_mag is not None and np.any(ei_mag != 0):
            e_mag, _ = _propagate_errors_mag(
                z_mag.real, z_mag.imag, ei_mag, eq_mag)
            e_mag = _magnitude_error_units(
                mag_db, e_mag,
                _magnitude_error_unit_key(format, units, mag_deembed))
            _, e_phase = _propagate_errors_mag(
                z_phase.real, z_phase.imag, ei_phase, eq_phase)
            line, = ax1.plot(f_mhz, mag_db, linewidth=0.8, label=label, **kwargs)
            ax1.fill_between(f_mhz, mag_db - e_mag, mag_db + e_mag,
                             color=line.get_color(), **ERROR_FILL_STYLE)
            line, = ax2.plot(f_mhz, phase, linewidth=0.8, label=label, **kwargs)
            ax2.fill_between(f_mhz, phase - e_phase, phase + e_phase,
                             color=line.get_color(), **ERROR_FILL_STYLE)
        else:
            ax1.plot(f_mhz, mag_db, linewidth=0.8, label=label, **kwargs)
            ax2.plot(f_mhz, phase, linewidth=0.8, label=label, **kwargs)
        ax1.set_ylabel(
            _offset_magnitude_label(mag_axis_label, mc, mrot)
            + _axis_label_suffix(mag_deembed, mc, mrot))
        ax2.set_ylabel(
            _phase_axis_label(pc, prot)
            + _axis_label_suffix(phase_deembed, pc, prot))
        ax2.set_xlabel('Frequency (MHz)')
        if format == _MAGPHASE_IQ_FORMAT:
            z_iq, _, _, _, _ = _apply_transforms(
                f, z_in, deembed, phase_center, phase_rotate, ei, eq,
                group_delay_cal=group_delay_cal)
            iq_axis_label = _deembedded_iq_label(deembed)
            if iq_axis_label is None:
                iq_axis_label = iq_label
                if units not in ('raw', 'adc_units'):
                    z_iq, iq_axis_label = _scale_iq_to_magnitude_units(
                        z_iq, units, info=info, config=config,
                        reference_plane=reference_plane, frequencies=f,
                        tx_power_dbm=tx_power_dbm,
                        calibration_cache=calibration_cache)
                    z_iq, iq_axis_label = _apply_iq_si_prefix(
                        ax3, z_iq, units, iq_axis_label)
            iq_suffix = _axis_label_suffix(
                deembed, phase_center, phase_rotate)
            ax3.plot(
                z_iq.real, z_iq.imag, linewidth=0.8,
                label=label, **kwargs)
            ax3.set_xlabel(_iq_axis_text(
                'I', iq_axis_label, phase_center=phase_center,
                phase_rotate=phase_rotate, iq_suffix=iq_suffix))
            ax3.set_ylabel(_iq_ylabel_text(
                iq_axis_label, iq_suffix, phase_center=phase_center,
                phase_rotate=phase_rotate))
            ax3.set_aspect('equal', adjustable='datalim')
        return

    z, ei, eq, _, _ = _apply_transforms(
        f, z_in, deembed, phase_center, phase_rotate, ei, eq,
        group_delay_cal=group_delay_cal)
    axis_iq_label = iq_label
    if _is_physical_iq_unit(units):
        z, axis_iq_label = _scale_iq_to_magnitude_units(
            z, units, info=info, config=config,
            reference_plane=reference_plane, frequencies=f,
            tx_power_dbm=tx_power_dbm,
            calibration_cache=calibration_cache)
        z, axis_iq_label = _apply_iq_si_prefix(
            ax1, z, units, axis_iq_label)
    si, sq = z.real, z.imag

    f_mhz = f / 1e6
    iq_suffix = _axis_label_suffix(deembed, phase_center, phase_rotate)

    if format == 'iq_vs_f':
        if has_errors and np.any(ei != 0):
            line, = ax1.plot(f_mhz, si, linewidth=0.8, label=label, **kwargs)
            ax1.fill_between(f_mhz, si - ei, si + ei,
                             color=line.get_color(), **ERROR_FILL_STYLE)
            line, = ax2.plot(f_mhz, sq, linewidth=0.8, label=label, **kwargs)
            ax2.fill_between(f_mhz, sq - eq, sq + eq,
                             color=line.get_color(), **ERROR_FILL_STYLE)
        else:
            ax1.plot(f_mhz, si, linewidth=0.8, label=label, **kwargs)
            ax2.plot(f_mhz, sq, linewidth=0.8, label=label, **kwargs)
        ax1.set_ylabel(_iq_axis_text(
            'I', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax2.set_ylabel(_iq_axis_text(
            'Q', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax2.set_xlabel('Frequency (MHz)')

    else:
        raise ValueError(f"Unknown format '{format}'. Use {_format_choices()}.")


def _coerce_fit_results(fit_results):
    """Return fit results as a non-empty list."""
    if fit_results is None:
        raise ValueError('fit_results must contain at least one fit result.')
    if hasattr(fit_results, 'z_data') and hasattr(fit_results, 'z_fit'):
        fit_results = [fit_results]
    else:
        fit_results = list(fit_results)
    if not fit_results:
        raise ValueError('fit_results must contain at least one fit result.')
    return fit_results


def _fit_parameter_values(fit_results, name):
    """Return one real-valued fit parameter array."""
    if not all(hasattr(fit, name) for fit in fit_results):
        raise ValueError(f"plot_fit_params() unknown fit parameter: {name!r}")
    values = np.asarray([getattr(fit, name) for fit in fit_results])
    if np.iscomplexobj(values):
        raise ValueError(
            f"plot_fit_params() only supports real-valued fit parameters; {name!r} is complex."
        )
    return values


def _fit_param_color_values(fit_results, color):
    """Resolve color as either a literal Matplotlib color or fit parameter."""
    if color is None:
        return None, None
    if isinstance(color, str) and all(hasattr(fit, color) for fit in fit_results):
        return _fit_parameter_values(fit_results, color), color

    color_values = np.asarray(color)
    if color_values.ndim == 0:
        return None, None
    if color_values.shape != (len(fit_results),):
        return None, None
    if np.iscomplexobj(color_values):
        raise ValueError('plot_fit_params() color values must be real-valued.')
    if np.issubdtype(color_values.dtype, np.number) or color_values.dtype == bool:
        return color_values, None
    return None, None


def _fit_trace_title(fit, index):
    """Return a compact per-fit title/label suffix."""
    fr = float(getattr(fit, 'fr', np.nan))
    if np.isfinite(fr):
        return f'{fr / 1e6:.3f} MHz'
    return f'Fit {index + 1}'


def _fit_trace_label(label, fit, index, total, label_suffix=True):
    """Resolve the legend label for one fit trace."""
    suffix = _fit_trace_title(fit, index) if label_suffix and total > 1 else None
    if label is None:
        return suffix or 'Fit'
    return f'{label} {suffix}' if suffix else label


def _fit_has_errors(fit):
    """Return True when a fit result carries non-zero I/Q uncertainties."""
    z_err = getattr(fit, 'z_err_data', None)
    if z_err is None:
        return False
    z_err = np.asarray(z_err)
    if np.iscomplexobj(z_err):
        return np.any(z_err.real != 0) or np.any(z_err.imag != 0)
    return np.any(z_err != 0)


def _fit_error_arrays(fit, size):
    """Return sigma_I and sigma_Q arrays from a fit result."""
    z_err = getattr(fit, 'z_err_data', None)
    if z_err is None:
        return None, None
    z_err = np.asarray(z_err)
    if z_err.size != size:
        z_err = z_err.reshape(size)
    if np.iscomplexobj(z_err):
        return np.abs(z_err.real).ravel(), np.abs(z_err.imag).ravel()
    err = np.abs(z_err.astype(float, copy=False)).ravel()
    return err, err


def _normalise_fit_trace(f, z_data, z_fit, ei, eq, units='raw', info=None,
                         config=None, reference_plane='adc_input',
                         calibration_cache=None):
    """Normalise one data/model trace pair using plot_sweep conventions."""
    units = _canonical_units(units)
    if units == 'raw':
        si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
            z_data.real, z_data.imag, units, info, config=config,
            ei=ei, eq=eq, reference_plane=reference_plane, frequencies=f,
            calibration_cache=calibration_cache)
        if z_fit is None:
            return si + 1j * sq, None, ei, eq, iq_label, mag_label
        fit_si, fit_sq, _, _, _, _ = _normalise_iq(
            z_fit.real, z_fit.imag, units, info, config=config,
            reference_plane=reference_plane, frequencies=f,
            calibration_cache=calibration_cache)
        return si + 1j * sq, fit_si + 1j * fit_sq, ei, eq, iq_label, mag_label

    if units == 'peak':
        peak = np.max(np.abs(z_data))
        if peak == 0:
            peak = 1.0
        z_data = z_data / peak
        if z_fit is not None:
            z_fit = z_fit / peak
        if ei is not None:
            ei = ei / peak
            eq = eq / peak
        return (z_data, z_fit, ei, eq,
                '(peak norm)', '|RX| (dB, peak-normalised)')

    if info is None:
        raise ValueError(
            "plot_fits() requires info=... for non-raw units because "
            'FitResult does not store sweep metadata.'
        )

    si, sq, ei, eq, iq_label, mag_label = _normalise_iq(
        z_data.real, z_data.imag, units, info, config=config,
        ei=ei, eq=eq, reference_plane=reference_plane, frequencies=f,
        calibration_cache=calibration_cache)
    if z_fit is None:
        return si + 1j * sq, None, ei, eq, iq_label, mag_label
    fit_si, fit_sq, _, _, _, _ = _normalise_iq(
        z_fit.real, z_fit.imag, units, info, config=config,
        reference_plane=reference_plane, frequencies=f,
        calibration_cache=calibration_cache)
    return si + 1j * sq, fit_si + 1j * fit_sq, ei, eq, iq_label, mag_label


def _apply_fit_raw_deembed_transforms(f, fit, z_data, z_fit, ei, eq,
                                      phase_center=False,
                                      phase_rotate=False):
    """Apply raw-unit fit-derived deembed/rotation transforms.

    This keeps plot_fits() consistent with the fitted gain/delay/rotation
    parameters used to build the FitResult, instead of re-estimating them from
    the plotted trace.
    """
    if isinstance(phase_center, dict) or isinstance(phase_rotate, dict):
        return None

    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    gain_amplitude = float(getattr(fit, 'a', np.nan))
    gain_phase = float(getattr(fit, 'alpha', np.nan))
    tau = float(getattr(fit, 'tau', np.nan))
    if (not np.isfinite(gain_amplitude) or gain_amplitude == 0.0
            or not np.isfinite(gain_phase) or not np.isfinite(tau)):
        return None

    deembed_scale = (
        np.exp(1j * 2.0 * np.pi * np.asarray(f, dtype=float) * tau)
        * np.exp(-1j * gain_phase) / gain_amplitude
    )
    z_data = np.asarray(z_data, dtype=complex) * deembed_scale
    if z_fit is not None:
        z_fit = np.asarray(z_fit, dtype=complex) * deembed_scale

    z_err = None
    if ei is not None and eq is not None:
        from .. import resonator
        z_err = resonator.transform_s21_error(
            np.asarray(ei, dtype=float) + 1j * np.asarray(eq, dtype=float),
            deembed_scale)

    if phase_center:
        center = complex(getattr(fit, 'iq_center_deembed', complex(np.nan, np.nan)))
        if not (np.isfinite(center.real) and np.isfinite(center.imag)):
            return None
        z_data = z_data - center
        if z_fit is not None:
            z_fit = z_fit - center
        if phase_rotate:
            rotation_angle = float(getattr(
                fit, 'phase_center_rotation_angle', np.nan))
            if not np.isfinite(rotation_angle):
                return None
            rotation = np.exp(1j * rotation_angle)
            z_data = z_data * rotation
            if z_fit is not None:
                z_fit = z_fit * rotation
            if z_err is not None:
                z_err = resonator.transform_s21_error(z_err, rotation)
    elif phase_rotate:
        rotation_angle = float(getattr(fit, 'deembed_rotation_angle', np.nan))
        if not np.isfinite(rotation_angle):
            return None
        rotation = np.exp(1j * rotation_angle)
        z_data = 1.0 + (z_data - 1.0) * rotation
        if z_fit is not None:
            z_fit = 1.0 + (z_fit - 1.0) * rotation
        if z_err is not None:
            from .. import resonator
            z_err = resonator.transform_s21_error(z_err, rotation)

    if z_err is None:
        ei_out = None
        eq_out = None
    else:
        ei_out = z_err.real
        eq_out = z_err.imag
    return z_data, z_fit, ei_out, eq_out


def _transform_fit_model(f, z_fit, deembed_params=None,
                         phase_center=False, phase_rotate=False,
                         phase_params=None, group_delay_cal=None):
    """Apply a data-derived transform to the fitted model trace."""
    if z_fit is None:
        return None
    if deembed_params is not None:
        from .. import resonator
        z_fit = resonator.apply_deembed_params(z_fit, deembed_params,
                                               frequency=f)
    elif group_delay_cal is not None:
        z_fit, _, _, _, _ = _apply_transforms(
            f, z_fit, False, False, False,
            group_delay_cal=group_delay_cal)

    phase_center, phase_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)
    if phase_center or phase_rotate:
        center_spec = phase_params if phase_center and phase_params is not None else phase_center
        rotate_spec = phase_params if phase_rotate and phase_params is not None else phase_rotate
        z_fit, _, _, _ = _apply_phase_ops(
            z_fit, phase_center=center_spec, phase_rotate=rotate_spec)
    return z_fit


def _prepare_fit_trace(fit, deembed=False, phase_center=False,
                       phase_rotate=False, units='raw', info=None, config=None,
                       reference_plane='adc_input', group_delay_cal=None,
                       calibration_cache=None):
    """Return one fit trace pair in the requested plotting coordinates."""
    f = getattr(fit, 'f_data', None)
    if f is None:
        raise ValueError('Each fit result must include f_data and z_data arrays.')
    f = np.asarray(f, float).ravel()

    resolved_center, resolved_rotate = _resolve_phase_specs(
        phase_center, phase_rotate)

    if deembed and resolved_center is True and resolved_rotate is True:
        z_pc = getattr(fit, 'z_data_deembed_phase_centered', None)
        if z_pc is not None:
            z_data = np.asarray(z_pc, complex).ravel()
            z_fit_attr = getattr(fit, 'z_fit_deembed_phase_centered', None)
            if z_fit_attr is not None:
                z_fit = np.asarray(z_fit_attr, complex).ravel()
                finite_fit = np.isfinite(z_fit.real) & np.isfinite(z_fit.imag)
                if f.size != z_fit.size or not np.any(finite_fit):
                    z_fit = None
            else:
                z_fit = None
            # The deembed+rotate transform is fit-derived, not data-derived,
            # so propagating the original raw I/Q uncertainties would mislabel
            # them; drop them on this path.
            iq_label, mag_label = _deembedded_trace_labels()
            return f, z_data, z_fit, None, None, iq_label, mag_label

    z_data_attr = getattr(fit, 'z_data', None)
    if z_data_attr is None:
        raise ValueError('Each fit result must include f_data and z_data arrays.')
    z_data = np.asarray(z_data_attr, complex).ravel()
    if f.size != z_data.size:
        raise ValueError('FitResult f_data and z_data must have matching lengths.')
    z_fit_attr = getattr(fit, 'z_fit', None)
    if z_fit_attr is not None:
        z_fit = np.asarray(z_fit_attr, complex).ravel()
        finite_fit = np.isfinite(z_fit.real) & np.isfinite(z_fit.imag)
        if f.size != z_fit.size or not np.any(finite_fit):
            z_fit = None
    else:
        z_fit = None

    ei, eq = _fit_error_arrays(fit, f.size)
    if deembed and group_delay_cal is None:
        transformed = _apply_fit_raw_deembed_transforms(
            f, fit, z_data, z_fit, ei, eq,
            phase_center=phase_center, phase_rotate=phase_rotate)
        if transformed is not None:
            z_data, z_fit, ei, eq = transformed
            iq_label, mag_label = _deembedded_trace_labels()
            return f, z_data, z_fit, ei, eq, iq_label, mag_label

    z_data, z_fit, ei, eq, iq_label, mag_label = _normalise_fit_trace(
        f, z_data, z_fit, ei, eq, units=units, info=info, config=config,
        reference_plane=reference_plane, calibration_cache=calibration_cache)

    z_data, ei, eq, deembed_params, phase_params = _apply_transforms(
        f, z_data, deembed, phase_center, phase_rotate, ei, eq,
        group_delay_cal=group_delay_cal)
    z_fit = _transform_fit_model(
        f, z_fit, deembed_params=deembed_params,
        phase_center=phase_center, phase_rotate=phase_rotate,
        phase_params=phase_params,
        group_delay_cal=group_delay_cal)
    if deembed:
        iq_label, mag_label = _deembedded_trace_labels()
    return f, z_data, z_fit, ei, eq, iq_label, mag_label


def _base_fit_styles(kwargs, data_kwargs=None, fit_kwargs=None):
    """Build default marker/line styles for plot_fits()."""
    data_specific = {} if data_kwargs is None else dict(data_kwargs)
    fit_specific = {} if fit_kwargs is None else dict(fit_kwargs)

    data_style = dict(kwargs)
    fit_style = dict(kwargs)
    data_style.update(data_specific)
    fit_style.update(fit_specific)

    if 'linestyle' not in data_specific:
        data_style['linestyle'] = 'None'
    if 'marker' not in data_specific:
        data_style['marker'] = 'o'
    data_style.setdefault('markersize', 3)
    data_style.setdefault('alpha', 0.4)
    data_style.setdefault('markeredgewidth', 0)

    if 'marker' not in fit_specific:
        fit_style['marker'] = None
    fit_style.setdefault('linestyle', '--')
    fit_style.setdefault('linewidth', 1.2)
    return data_style, fit_style


def _pair_trace_styles(ax, data_style, fit_style):
    """Assign a shared color to the data markers and fit line."""
    data_style = dict(data_style)
    fit_style = dict(fit_style)
    color = fit_style.get('color', data_style.get('color'))
    if color is None:
        color = ax._get_lines.get_next_color()
    data_style.setdefault('color', color)
    fit_style.setdefault('color', color)
    return data_style, fit_style


def _plot_single_fit_trace(ax1, ax2, fit, format,
                           deembed=False, phase_center=False,
                           phase_rotate=False,
                           has_errors=False, data_label=None,
                           fit_label='_nolegend_', ax3=None, units='raw',
                           info=None, config=None,
                           reference_plane='adc_input',
                           group_delay_cal=None,
                           data_style=None, fit_style=None,
                           mag_centered=None, phase_centered=None,
                           mag_rotated=None, phase_rotated=None,
                           unwrap_phase=None, tx_power_dbm=None,
                           calibration_cache=None):
    """Plot one fit result as markers for data and a line for the model."""
    if _is_magphase_format(format):
        mag_deembed, phase_deembed = _format_axis_deembed(
            format, units, deembed)
        mc, mrot = _resolve_axis_phase_specs(
            phase_center, phase_rotate, mag_centered, mag_rotated)
        pc, prot = _resolve_axis_phase_specs(
            phase_center, phase_rotate, phase_centered, phase_rotated)
        phase_unwrap = _resolve_unwrap_phase(unwrap_phase, pc, prot)
        (f, z_data_mag, z_fit_mag, ei_mag, eq_mag, iq_label,
         mag_label) = _prepare_fit_trace(
            fit, deembed=mag_deembed, phase_center=mc, phase_rotate=mrot,
            units=units,
            info=info, config=config, reference_plane=reference_plane,
            group_delay_cal=group_delay_cal,
            calibration_cache=calibration_cache)
        if mag_deembed == phase_deembed and mc == pc and mrot == prot:
            z_data_phase = z_data_mag
            z_fit_phase = z_fit_mag
            ei_phase, eq_phase = ei_mag, eq_mag
        else:
            (_, z_data_phase, z_fit_phase, ei_phase, eq_phase, _,
             _) = _prepare_fit_trace(
                fit, deembed=phase_deembed, phase_center=pc,
                phase_rotate=prot,
                units=units,
                info=info, config=config, reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                calibration_cache=calibration_cache)
        data_style, fit_style = _pair_trace_styles(ax1, data_style, fit_style)
        f_mhz = f / 1e6
        if format == _MAGPHASE_IQ_FORMAT and mag_deembed:
            data_mag, mag_axis_label = _deembedded_magnitude_values(
                z_data_mag, units)
        else:
            data_mag, _ = _compute_mag_phase_units(
                z_data_mag, units, info=info, config=config,
                reference_plane=reference_plane, frequencies=f,
                tx_power_dbm=tx_power_dbm,
                calibration_cache=calibration_cache)
            mag_axis_label = mag_label
        fit_mag = None
        if z_fit_mag is not None:
            if format == _MAGPHASE_IQ_FORMAT and mag_deembed:
                fit_mag, _ = _deembedded_magnitude_values(z_fit_mag, units)
            else:
                fit_mag, _ = _compute_mag_phase_units(
                    z_fit_mag, units, info=info, config=config,
                    reference_plane=reference_plane, frequencies=f,
                    tx_power_dbm=tx_power_dbm,
                    calibration_cache=calibration_cache)
        if format == _MAGPHASE_IQ_FORMAT:
            if ax3 is None:
                raise ValueError(
                    "format='mag+phase+iq' requires an IQ axis."
                )
            data_mag, mag_axis_label, mag_scale = (
                _apply_physical_si_prefix_with_scale(
                    ax3, data_mag, units, mag_axis_label, fit_mag)
            )
            if mag_scale is not None and fit_mag is not None:
                fit_mag = fit_mag / mag_scale
        data_phase = _phase_values(z_data_phase, unwrap=phase_unwrap)
        if fit_mag is not None:
            ax1.plot(f_mhz, fit_mag, label=fit_label, **fit_style)
        if z_fit_phase is not None:
            fit_phase = _phase_values(z_fit_phase, unwrap=phase_unwrap)
            ax2.plot(f_mhz, fit_phase, label=fit_label, **fit_style)
        if has_errors and ei_mag is not None and np.any(ei_mag != 0):
            e_mag, _ = _propagate_errors_mag(
                z_data_mag.real, z_data_mag.imag, ei_mag, eq_mag)
            e_mag = _magnitude_error_units(
                data_mag, e_mag,
                _magnitude_error_unit_key(format, units, mag_deembed))
            _, e_phase = _propagate_errors_mag(
                z_data_phase.real, z_data_phase.imag, ei_phase, eq_phase)
            ax1.fill_between(
                f_mhz, data_mag - e_mag, data_mag + e_mag,
                color=data_style['color'], **ERROR_FILL_STYLE)
            ax2.fill_between(
                f_mhz, data_phase - e_phase, data_phase + e_phase,
                color=data_style['color'], **ERROR_FILL_STYLE)
        ax1.plot(f_mhz, data_mag, label=data_label, **data_style)
        ax2.plot(f_mhz, data_phase, label=data_label, **data_style)
        ax1.set_ylabel(
            _offset_magnitude_label(mag_axis_label, mc, mrot)
            + _axis_label_suffix(mag_deembed, mc, mrot))
        ax2.set_ylabel(
            _phase_axis_label(pc, prot)
            + _axis_label_suffix(phase_deembed, pc, prot))
        ax2.set_xlabel('Frequency (MHz)')
        if format == _MAGPHASE_IQ_FORMAT:
            (_, z_data_iq, z_fit_iq, _, _, iq_label_iq,
             _) = _prepare_fit_trace(
                fit, deembed=deembed, phase_center=phase_center,
                phase_rotate=phase_rotate, units=units,
                info=info, config=config, reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                calibration_cache=calibration_cache)
            iq_axis_label = _deembedded_iq_label(deembed)
            if iq_axis_label is None:
                iq_axis_label = iq_label_iq
                if units not in ('raw', 'adc_units'):
                    z_data_iq, iq_axis_label = _scale_iq_to_magnitude_units(
                        z_data_iq, units, info=info, config=config,
                        reference_plane=reference_plane, frequencies=f,
                        tx_power_dbm=tx_power_dbm,
                        calibration_cache=calibration_cache)
                    if z_fit_iq is not None:
                        z_fit_iq, _ = _scale_iq_to_magnitude_units(
                            z_fit_iq, units, info=info, config=config,
                            reference_plane=reference_plane, frequencies=f,
                            tx_power_dbm=tx_power_dbm,
                            calibration_cache=calibration_cache)
                    z_data_iq, iq_axis_label, iq_scale = (
                        _apply_iq_si_prefix_with_scale(
                            ax3, z_data_iq, units, iq_axis_label, z_fit_iq)
                    )
                    if iq_scale is not None and z_fit_iq is not None:
                        z_fit_iq = z_fit_iq / iq_scale
            iq_suffix = _axis_label_suffix(
                deembed, phase_center, phase_rotate)
            if z_fit_iq is not None:
                ax3.plot(
                    z_fit_iq.real, z_fit_iq.imag,
                    label=fit_label, **fit_style)
            ax3.plot(
                z_data_iq.real, z_data_iq.imag,
                label=data_label, **data_style)
            ax3.set_xlabel(_iq_axis_text(
                'I', iq_axis_label, phase_center=phase_center,
                phase_rotate=phase_rotate, iq_suffix=iq_suffix))
            ax3.set_ylabel(_iq_ylabel_text(
                iq_axis_label, iq_suffix, phase_center=phase_center,
                phase_rotate=phase_rotate))
            ax3.set_aspect('equal', adjustable='datalim')
        return

    f, z_data, z_fit, ei, eq, iq_label, mag_label = _prepare_fit_trace(
        fit, deembed=deembed, phase_center=phase_center,
        phase_rotate=phase_rotate, units=units,
        info=info, config=config, reference_plane=reference_plane,
        group_delay_cal=group_delay_cal,
        calibration_cache=calibration_cache)
    data_style, fit_style = _pair_trace_styles(ax1, data_style, fit_style)
    iq_suffix = _axis_label_suffix(deembed, phase_center, phase_rotate)
    axis_iq_label = iq_label
    if format in ('iq', 'iq_vs_f') and _is_physical_iq_unit(units):
        z_data, axis_iq_label = _scale_iq_to_magnitude_units(
            z_data, units, info=info, config=config,
            reference_plane=reference_plane, frequencies=f,
            calibration_cache=calibration_cache)
        if z_fit is not None:
            z_fit, _ = _scale_iq_to_magnitude_units(
                z_fit, units, info=info, config=config,
                reference_plane=reference_plane, frequencies=f,
                calibration_cache=calibration_cache)
        z_data, axis_iq_label, iq_scale = _apply_iq_si_prefix_with_scale(
            ax1, z_data, units, axis_iq_label, z_fit)
        if iq_scale is not None and z_fit is not None:
            z_fit = z_fit / iq_scale

    if format == 'iq':
        if z_fit is not None:
            ax1.plot(z_fit.real, z_fit.imag, label=fit_label, **fit_style)
        ax1.plot(z_data.real, z_data.imag, label=data_label, **data_style)
        ax1.set_xlabel(_iq_axis_text(
            'I', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax1.set_ylabel(_iq_axis_text(
            'Q', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax1.set_aspect('equal', adjustable='datalim')
        return

    f_mhz = f / 1e6

    if format == 'iq_vs_f':
        if z_fit is not None:
            ax1.plot(f_mhz, z_fit.real, label=fit_label, **fit_style)
            ax2.plot(f_mhz, z_fit.imag, label=fit_label, **fit_style)
        if has_errors and ei is not None and np.any(ei != 0):
            ax1.fill_between(
                f_mhz, z_data.real - ei, z_data.real + ei,
                color=data_style['color'], **ERROR_FILL_STYLE)
            ax2.fill_between(
                f_mhz, z_data.imag - eq, z_data.imag + eq,
                color=data_style['color'], **ERROR_FILL_STYLE)
        ax1.plot(f_mhz, z_data.real, label=data_label, **data_style)
        ax2.plot(f_mhz, z_data.imag, label=data_label, **data_style)
        ax1.set_ylabel(_iq_axis_text(
            'I', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax2.set_ylabel(_iq_axis_text(
            'Q', axis_iq_label, phase_center=phase_center,
            phase_rotate=phase_rotate, iq_suffix=iq_suffix))
        ax2.set_xlabel('Frequency (MHz)')
        return

    raise ValueError(f"Unknown format '{format}'. Use {_format_choices()}.")


def plot_fits(fit_results, format='magphase', deembed=False,
              phase_center=False, phase_rotate=False, show_errors=None,
              multi_tone='overlay', fig=None, label=None,
              units='raw', info=None, config=None,
              reference_plane='adc_input', group_delay_cal=None,
              show_fit_legend=False, data_kwargs=None, fit_kwargs=None,
              figsize=None, label_suffix=True, finalize=True,
              mag_centered=None, phase_centered=None,
              mag_rotated=None, phase_rotated=None,
              unwrap_phase=None, tx_power_dbm=None,
              tx_power_reference_plane=None,
              **kwargs):
    """Plot one or more FitResult traces using the sweep plotting styles.

    Args:
        fit_results: One FitResult or an iterable of FitResult objects.
        format: 'iq' | 'iq_vs_f' | 'magphase' | 'mag+phase+iq'
        deembed: Apply RF deembedding before plotting. For
            ``format='magphase'`` with calibrated magnitude units
            (``'dbfs'``, ``'dbm'``, ``'volts'``, ``'watts'``, ``'s21'``),
            deembedding is applied to the phase axis only so the magnitude
            remains received power. For ``format='mag+phase+iq'``, deembedding
            is applied to magnitude, phase, and IQ together so all panels share
            one complex frame.
        phase_center: Apply the circle-centering step before plotting.
        phase_rotate: Apply the rotation step after centering.
        show_errors: True/False/None. ``None`` draws error bands when any fit
            result carries ``z_err_data``.
        multi_tone: 'overlay' (shared axes) or 'grid' (one subplot per fit).
        fig: Existing figure. If None, create a new one.
        label: Optional legend label prefix.
        units: Same units as plot_sweep(). For non-raw units, pass ``info``.
            ``'adc_units'`` / ``'adc'`` uses linear ADC units and can be
            referred to a detector/cryostat plane.
            ``'dbm'`` uses dBm for magnitude and RMS volts for IQ.
            ``'volts'``/``'v'`` use SI-prefixed voltage IQ axes;
            ``'watts'``/``'w'`` uses watts for magnitude and RMS volts for IQ.
            ``units='s21'``, ``'s21_log'``, and ``'s21_linear'`` also require
            ``tx_power_dbm`` or tone-power metadata in ``info``.
        info: Sweep metadata needed for non-raw units. FitResult does not
            retain the original sweep_data dict.
        config: Plotting config used for calibrated units.
        reference_plane: Reference plane used for calibrated magnitude
            plotting. Use ``'adc_input'``, ``'cryostat_output'``, or
            ``'detector'``. ``units='raw'`` is accumulator units only; use
            ``units='adc_units'`` for a linear ADC-unit view referred to a
            detector/cryostat plane.
        group_delay_cal: Group delay calibration forwarded to sweep transforms.
        show_fit_legend: If True, add separate legend entries for fit lines.
        data_kwargs: Matplotlib keyword overrides for the data markers only.
        fit_kwargs: Matplotlib keyword overrides for the fit lines only.
        figsize: Optional figure size in inches when ``fig`` is not supplied.
        label_suffix: If True, append each fit's resonance frequency to labels
            when multiple fits are plotted together. Set False for grouped
            overlays where all traces in one call share the same legend label.
        finalize: If True, draw legends, titles, and tight layout before
            returning. Set False when repeatedly adding traces to the same
            figure and finalize once after the loop.
        unwrap_phase: bool or None, optional. ``None`` preserves the
            automatic behaviour: unwrap unrotated phase traces, including
            center-only traces, but leave explicitly rotated phase wrapped.
            Pass ``True`` or ``False`` to force phase unwrapping/wrapping in
            ``format='magphase'``.
        tx_power_dbm: Scalar, per-fit, or per-tone transmitted power in dBm,
            used only with ``units='s21'``. If omitted and ``info`` contains
            tone powers, they are resolved at ``reference_plane``.
        tx_power_reference_plane: Reference plane for explicit
            ``tx_power_dbm``. If it differs from ``reference_plane``, the
            structured info/config calibration is used to convert it when
            possible.
        **kwargs: Base matplotlib keyword arguments applied to both data and
            fit traces before the marker/line defaults are enforced.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    custom_title = kwargs.pop('title', None)
    units = _canonical_units(units)
    calibration_cache = {}
    fit_results = _coerce_fit_results(fit_results)
    _validate_magnitude_unit_transforms(
        format, units, deembed=deembed, phase_center=phase_center,
        mag_centered=mag_centered, reference_plane=reference_plane)
    if _is_s21_unit(units):
        tx_power_dbm, tx_power_reference_plane = _resolve_tx_power_dbm(
            tx_power_dbm, tx_power_reference_plane, reference_plane,
            info=info, config=config,
            calibration_cache=calibration_cache)
        _check_tx_power_reference(reference_plane, tx_power_reference_plane)
    if units != 'raw' and units != 'peak' and info is None:
        raise ValueError(
            "plot_fits() requires info=... for non-raw units because "
            'FitResult does not store sweep metadata.'
        )

    if show_errors is None:
        show_errors = any(_fit_has_errors(fit) for fit in fit_results)

    n_traces = len(fit_results)
    is_multi = n_traces > 1
    suffix = _transform_title_suffix(deembed, phase_center, phase_rotate)
    data_style_base, fit_style_base = _base_fit_styles(
        kwargs, data_kwargs=data_kwargs, fit_kwargs=fit_kwargs)

    if is_multi and multi_tone == 'grid':
        if format == 'iq':
            n_cols = min(n_traces, 4)
            n_rows = int(np.ceil(n_traces / n_cols))
            if fig is None:
                grid_figsize = figsize or (4 * n_cols, 4 * n_rows)
                fig, axes = plt.subplots(
                    n_rows, n_cols, figsize=grid_figsize)
            else:
                axes = np.array(fig.axes).reshape(n_rows, n_cols)
            axes_flat = np.atleast_1d(axes).ravel()
            for i, fit in enumerate(fit_results):
                ax = axes_flat[i]
                trace_label = _fit_trace_label(
                    label, fit, i, n_traces, label_suffix=label_suffix)
                fit_label = (
                    f'{trace_label} fit' if show_fit_legend else '_nolegend_'
                )
                trace_tx_power = (
                    _select_tx_power_dbm(
                        tx_power_dbm, trace_position=i,
                        tone_index=getattr(fit, 'tone_index', i),
                        total=n_traces)
                    if _is_s21_unit(units) else None
                )
                _plot_single_fit_trace(
                    ax, None, fit, format, deembed=deembed,
                    phase_center=phase_center, phase_rotate=phase_rotate,
                    has_errors=show_errors,
                    data_label=trace_label, fit_label=fit_label,
                    units=units, info=info, config=config,
                    reference_plane=reference_plane,
                    group_delay_cal=group_delay_cal,
                    data_style=data_style_base, fit_style=fit_style_base,
                    mag_centered=mag_centered,
                    phase_centered=phase_centered,
                    mag_rotated=mag_rotated,
                    phase_rotated=phase_rotated,
                    unwrap_phase=unwrap_phase,
                    tx_power_dbm=trace_tx_power,
                    calibration_cache=calibration_cache)
                ax.set_title(_fit_trace_title(fit, i))
                if finalize:
                    ax.legend(fontsize='small')
            for i in range(n_traces, len(axes_flat)):
                axes_flat[i].set_visible(False)
            if finalize and custom_title is not None:
                fig.suptitle(custom_title)
            if finalize:
                plt.tight_layout()
            for ax in fig.axes:
                _disable_axis_offsets(ax)
            return fig

        if format == _MAGPHASE_IQ_FORMAT:
            grid_figsize = figsize or (12, 3.4 * n_traces)
            fig, axis_blocks = _magphase_iq_grid_axes(
                plt, n_traces, fig=fig, figsize=grid_figsize)
            for i, fit in enumerate(fit_results):
                ax_mag, ax_phase, ax_iq = axis_blocks[i]
                trace_label = _fit_trace_label(
                    label, fit, i, n_traces, label_suffix=label_suffix)
                fit_label = (
                    f'{trace_label} fit'
                    if show_fit_legend else '_nolegend_'
                )
                trace_tx_power = (
                    _select_tx_power_dbm(
                        tx_power_dbm, trace_position=i,
                        tone_index=getattr(fit, 'tone_index', i),
                        total=n_traces)
                    if _is_s21_unit(units) else None
                )
                _plot_single_fit_trace(
                    ax_mag, ax_phase, fit, format, deembed=deembed,
                    phase_center=phase_center, phase_rotate=phase_rotate,
                    has_errors=show_errors,
                    data_label=trace_label, fit_label=fit_label,
                    ax3=ax_iq,
                    units=units, info=info, config=config,
                    reference_plane=reference_plane,
                    group_delay_cal=group_delay_cal,
                    data_style=data_style_base, fit_style=fit_style_base,
                    mag_centered=mag_centered,
                    phase_centered=phase_centered,
                    mag_rotated=mag_rotated,
                    phase_rotated=phase_rotated,
                    unwrap_phase=unwrap_phase,
                    tx_power_dbm=trace_tx_power,
                    calibration_cache=calibration_cache)
                ax_mag.set_title(_fit_trace_title(fit, i))
                if finalize:
                    ax_mag.legend(fontsize='small')
                    ax_phase.legend(fontsize='small')
                    ax_iq.legend(fontsize='small')
            if finalize and custom_title is not None:
                fig.suptitle(custom_title)
            if finalize:
                plt.tight_layout()
            for ax in fig.axes:
                _disable_axis_offsets(ax)
            return fig

        if fig is None:
            grid_figsize = figsize or (12, 3 * n_traces)
            fig, axes = plt.subplots(n_traces, 2, figsize=grid_figsize,
                                     squeeze=False)
        else:
            axes = np.array(fig.axes).reshape(n_traces, 2)
        for i, fit in enumerate(fit_results):
            trace_label = _fit_trace_label(
                label, fit, i, n_traces, label_suffix=label_suffix)
            fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
            trace_tx_power = (
                _select_tx_power_dbm(
                    tx_power_dbm, trace_position=i,
                    tone_index=getattr(fit, 'tone_index', i),
                    total=n_traces)
                if _is_s21_unit(units) else None
            )
            _plot_single_fit_trace(
                axes[i, 0], axes[i, 1], fit, format, deembed=deembed,
                phase_center=phase_center, phase_rotate=phase_rotate,
                has_errors=show_errors,
                data_label=trace_label, fit_label=fit_label,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                data_style=data_style_base, fit_style=fit_style_base,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                unwrap_phase=unwrap_phase,
                tx_power_dbm=trace_tx_power,
                calibration_cache=calibration_cache)
            if finalize:
                axes[i, 0].legend(fontsize='small')
                axes[i, 1].legend(fontsize='small')
            axes[i, 0].set_title(_fit_trace_title(fit, i))
        if finalize and custom_title is not None:
            fig.suptitle(custom_title)
        if finalize:
            plt.tight_layout()
        for ax in fig.axes:
            _disable_axis_offsets(ax)
        return fig

    if format == 'iq':
        if fig is None:
            fig, ax = plt.subplots(1, 1, figsize=figsize or (8, 8))
        else:
            ax = fig.gca()
        for i, fit in enumerate(fit_results):
            trace_label = _fit_trace_label(
                label, fit, i, n_traces, label_suffix=label_suffix)
            fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
            trace_tx_power = (
                _select_tx_power_dbm(
                    tx_power_dbm, trace_position=i,
                    tone_index=getattr(fit, 'tone_index', i),
                    total=n_traces)
                if _is_s21_unit(units) else None
            )
            _plot_single_fit_trace(
                ax, None, fit, format, deembed=deembed,
                phase_center=phase_center, phase_rotate=phase_rotate,
                has_errors=show_errors,
                data_label=trace_label, fit_label=fit_label,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                data_style=data_style_base, fit_style=fit_style_base,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                unwrap_phase=unwrap_phase,
                tx_power_dbm=trace_tx_power,
                calibration_cache=calibration_cache)
        if finalize:
            ax.legend(fontsize='small')
            ax.set_title(custom_title if custom_title is not None
                         else 'Fitted Sweeps' + suffix)
            plt.tight_layout()
        for ax in fig.axes:
            _disable_axis_offsets(ax)
        return fig

    if format == _MAGPHASE_IQ_FORMAT:
        fig, ax1, ax2, ax3 = _magphase_iq_axes(
            plt, fig=fig, figsize=figsize)
        for i, fit in enumerate(fit_results):
            trace_label = _fit_trace_label(
                label, fit, i, n_traces, label_suffix=label_suffix)
            fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
            trace_tx_power = (
                _select_tx_power_dbm(
                    tx_power_dbm, trace_position=i,
                    tone_index=getattr(fit, 'tone_index', i),
                    total=n_traces)
                if _is_s21_unit(units) else None
            )
            _plot_single_fit_trace(
                ax1, ax2, fit, format, deembed=deembed,
                phase_center=phase_center, phase_rotate=phase_rotate,
                has_errors=show_errors,
                data_label=trace_label, fit_label=fit_label,
                ax3=ax3,
                units=units, info=info, config=config,
                reference_plane=reference_plane,
                group_delay_cal=group_delay_cal,
                data_style=data_style_base, fit_style=fit_style_base,
                mag_centered=mag_centered,
                phase_centered=phase_centered,
                mag_rotated=mag_rotated,
                phase_rotated=phase_rotated,
                unwrap_phase=unwrap_phase,
                tx_power_dbm=trace_tx_power,
                calibration_cache=calibration_cache)

        if finalize:
            ax1.legend(fontsize='small')
            ax2.legend(fontsize='small')
            ax3.legend(fontsize='small')
            fig.suptitle(custom_title if custom_title is not None
                         else 'Fitted Sweeps' + suffix)
            plt.tight_layout()
        for ax in fig.axes:
            _disable_axis_offsets(ax)
        return fig

    if fig is None:
        fig, (ax1, ax2) = plt.subplots(
            2, 1, sharex=True, figsize=figsize or (10, 6))
    else:
        ax1, ax2 = fig.axes[:2]

    for i, fit in enumerate(fit_results):
        trace_label = _fit_trace_label(
            label, fit, i, n_traces, label_suffix=label_suffix)
        fit_label = f'{trace_label} fit' if show_fit_legend else '_nolegend_'
        trace_tx_power = (
            _select_tx_power_dbm(
                tx_power_dbm, trace_position=i,
                tone_index=getattr(fit, 'tone_index', i),
                total=n_traces)
            if _is_s21_unit(units) else None
        )
        _plot_single_fit_trace(
            ax1, ax2, fit, format, deembed=deembed,
            phase_center=phase_center, phase_rotate=phase_rotate,
            has_errors=show_errors,
            data_label=trace_label, fit_label=fit_label,
            units=units, info=info, config=config,
            reference_plane=reference_plane,
            group_delay_cal=group_delay_cal,
            data_style=data_style_base, fit_style=fit_style_base,
            mag_centered=mag_centered,
            phase_centered=phase_centered,
            mag_rotated=mag_rotated,
            phase_rotated=phase_rotated,
            unwrap_phase=unwrap_phase,
            tx_power_dbm=trace_tx_power,
            calibration_cache=calibration_cache)

    if finalize:
        ax1.legend(fontsize='small')
        ax2.legend(fontsize='small')
        fig.suptitle(custom_title if custom_title is not None
                     else 'Fitted Sweeps' + suffix)
        plt.tight_layout()
    for ax in fig.axes:
        _disable_axis_offsets(ax)
    return fig


def plot_fit_params(fit_results, x='fr', y='Ql', color=None, fig=None,
                    label=None, colorbar=True, colorbar_label=None,
                    **kwargs):
    """Plot one fitted parameter against another.

    Args:
        fit_results: One FitResult or an iterable of FitResult objects.
        x: Attribute name for the x-axis.
        y: Attribute name for the y-axis.
        color: Either a literal Matplotlib color, a fit-parameter name to map
            to point color with a colorbar, or a numeric array with one entry
            per fit result.
        fig: Existing figure. If None, create a new one.
        label: Optional legend label.
        colorbar: When True, draw a colorbar for parameter/numeric coloring.
        colorbar_label: Optional colorbar label. Defaults to the parameter name
            when ``color`` is a fit parameter.
        **kwargs: Passed to matplotlib plot() for literal colors and to
            matplotlib scatter() for parameter-based coloring. Defaults to
            marker-only styling.

    Returns:
        matplotlib.figure.Figure
    """
    plt = _get_pyplot()
    fit_results = _coerce_fit_results(fit_results)

    x_values = _fit_parameter_values(fit_results, x)
    y_values = _fit_parameter_values(fit_results, y)
    color_values, color_name = _fit_param_color_values(fit_results, color)

    if fig is None:
        fig, ax = plt.subplots(1, 1, figsize=(8, 5))
    else:
        ax = fig.gca()

    if color_values is None:
        kwargs.setdefault('linestyle', 'None')
        kwargs.setdefault('marker', 'o')
        if color is not None:
            kwargs.setdefault('color', color)
        ax.plot(x_values, y_values, label=label, **kwargs)
    else:
        scatter_kwargs = dict(kwargs)
        scatter_kwargs.pop('linestyle', None)
        scatter_kwargs.setdefault('marker', 'o')
        artist = ax.scatter(x_values, y_values, c=color_values,
                            label=label, **scatter_kwargs)
        if colorbar:
            fig.colorbar(
                artist, ax=ax,
                label=colorbar_label or color_name,
            )
    ax.set_xlabel(x)
    ax.set_ylabel(y)
    if label is not None:
        ax.legend(fontsize='small')
    plt.tight_layout()
    return fig


# Convenience wrappers

def plot_sweep_iq(sweep_data, **kwargs):
    """Plot sweep as I vs Q in the complex plane. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='iq', **kwargs)


def plot_sweep_magphase(sweep_data, **kwargs):
    """Plot sweep as magnitude/phase vs frequency. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='magphase', **kwargs)


def plot_sweep_magphase_iq(sweep_data, **kwargs):
    """Plot sweep as magnitude/phase plus IQ. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='mag+phase+iq', **kwargs)


def plot_sweep_iq_vs_f(sweep_data, **kwargs):
    """Plot sweep as I and Q vs frequency. See plot_sweep() for args."""
    return plot_sweep(sweep_data, format='iq_vs_f', **kwargs)
