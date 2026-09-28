"""Signal extraction shared by the live viewer and possible GUI front ends.

Tone selections refer to absolute tone IDs, including in subset captures.
Centering and frequency calibration are derived from the supplied sweep once,
never from the (usually very short) arc covered by a live timestream.
"""

from collections.abc import Mapping

import numpy as np

from .. import resonator, timeseries
from ..noise import _get_sweep_trace


SIGNAL_LABELS = {
    'i': ('I', 'counts'),
    'q': ('Q', 'counts'),
    'amp': ('Amplitude', 'counts'),
    'phase': ('Phase', 'rad'),
    'logmag': ('Log magnitude', 'dB'),
    'centered_phase': ('Centered phase', 'rad'),
    'centered_rotated_phase': ('Centered, rotated phase', 'rad'),
    'frequency_shift': ('Frequency shift', 'Hz'),
    'fractional_frequency': ('Fractional frequency shift', '1'),
    'dissipation': ('Dissipation, Δ(1 / 2Qi)', '1'),
    'frequency_linewidths': ('Resonance-frequency shift', 'linewidths'),
    'dissipation_linewidths': ('Dissipation, Qr·Δ(1 / 2Qi)', '1'),
    'dphi_df': ('Phase slope (responsivity)', 'rad/Hz'),
    'd2phi_df2': ('Phase curvature', 'rad/Hz²'),
    'curvature_over_slope': ('Phase curvature / slope', '1/Hz'),
    'detuning': ('Probe detuning', 'Hz'),
    'detuning_linewidths': ('Probe detuning', 'linewidths'),
}

_ALIASES = {
    'amplitude': 'amp', 'magnitude': 'amp', 'mag': 'amp',
    'log_magnitude': 'logmag', 'log_mag': 'logmag',
    'centred_phase': 'centered_phase',
    'centred_rotated_phase': 'centered_rotated_phase',
    'frequency': 'frequency_shift', 'frequency_shift_hz': 'frequency_shift',
    'fractional': 'fractional_frequency', 'frac_frequency': 'fractional_frequency',
    'freq': 'frequency_shift', 'diss': 'dissipation',
    'freq_lw': 'frequency_linewidths', 'diss_lw': 'dissipation_linewidths',
    'detuning_hz': 'detuning',
    'responsivity': 'dphi_df', 'phase_slope': 'dphi_df',
    'curvature': 'd2phi_df2', 'phase_curvature': 'd2phi_df2',
}
_FREQUENCY_KEYS = ('resonance_shift_hz', 'frequency_shift_hz', 'frequency_hz')
_CONVERTED_SIGNALS = {'frequency_shift', 'fractional_frequency', 'dissipation'}
# The linewidth signals are the fractional pair times each tone's loaded Q.
_LINEWIDTH_SIGNALS = {'frequency_linewidths': 'fractional_frequency',
                      'dissipation_linewidths': 'dissipation'}


def canonical_signal(name):
    """Validate a signal name and resolve its common aliases."""
    if not isinstance(name, str):
        raise ValueError('signal names must be strings')
    name = name.strip().lower().replace(' ', '_').replace('-', '_')
    name = _ALIASES.get(name, name)
    if name not in SIGNAL_LABELS:
        raise ValueError(f'Unknown signal {name!r}; choose from '
                         + ', '.join(SIGNAL_LABELS))
    return name


def _tone_keys(samples, tones):
    available = samples.get('i_data', {})
    keys_by_id = {int(key): key for key in available}
    if len(keys_by_id) != len(available):
        raise ValueError('i_data contains duplicate tone IDs')
    if tones is None or (isinstance(tones, str) and tones == 'all'):
        return [keys_by_id[tone] for tone in sorted(keys_by_id)]
    if isinstance(tones, (int, np.integer, str)):
        tones = [tones]
    result = []
    for tone in tones:
        if not isinstance(tone, (int, np.integer, str)):
            raise ValueError('tones must contain integer tone IDs')
        try:
            tone_id = int(tone)
            key = keys_by_id[tone_id]
        except (ValueError, KeyError) as exc:
            raise ValueError(f'Tone {tone!r} is not present in the samples') from exc
        if key not in result:
            result.append(key)
    return result


def _unwrap_finite(z):
    """Unwrap each finite run independently so a gap cannot poison its tail."""
    z = np.asarray(z, dtype=complex)
    valid = np.isfinite(z.real) & np.isfinite(z.imag)
    result = np.full(z.shape, np.nan)
    edges = np.flatnonzero(np.diff(np.r_[False, valid, False]))
    for start, stop in zip(edges[::2], edges[1::2]):
        result[start:stop] = np.unwrap(np.angle(z[start:stop]))
    return result


def _precomputed_column(samples, field, key):
    values = samples[field]
    if isinstance(values, Mapping):
        for candidate in (key, int(key), f'{int(key):04d}'):
            if candidate in values:
                return np.asarray(values[candidate], dtype=float)
        raise ValueError(f'{field} is missing tone {key}')
    return np.asarray(timeseries._sample_tone_column(samples, field, int(key)),
                      dtype=float)


class SignalExtractor:
    """Extract real 1D signal arrays from ``parse_samples`` dictionaries.

    ``extract`` returns ``{canonical_signal: {original_tone_key: values}}``.
    Phase is unwrapped independently across finite runs. Log magnitude is
    ``20 log10(abs(I + 1j Q))`` relative to one raw count.

    Centered phases require ``sweep_data`` or ``phase_center_params``: a
    per-tone mapping of dictionaries produced by ``resonator.phase_center``
    (a single parameter dictionary may also be shared by all tones). These
    parameters must describe the same IQ coordinate system as the samples.
    The centered phase subtracts the circle center; the rotated variant also
    puts the sweep's off-resonance reference on the negative real axis.

    Frequency shift and fractional frequency follow the resonance-shift sign
    convention of ``timeseries._compute_freq_diss``. Both ``linearized`` and
    ``mobius`` return dissipation in ``Delta(1 / (2 Qi))`` units. The exact
    method needs per-tone resonator calibrations or fits plus a reference
    sweep. Precomputed demodulated fields take precedence over conversion.
    ``frequency_linewidths`` and ``dissipation_linewidths`` are the fractional
    pair multiplied by each tone's loaded Q from ``calibrations``, as for
    ``format='linewidths'`` in :mod:`souk_readout_tools.timeseries`.
    Create a new extractor if replacing the sweep or its calibrations.
    """

    def __init__(self, *, sweep_data=None, calibrations=None,
                 conversion_method='linearized', reference_tone_frequency=None,
                 smooth_window_hz=1000, phase_center_params=None):
        if conversion_method not in ('linearized', 'mobius'):
            raise ValueError("conversion_method must be 'linearized' or 'mobius'")
        self.sweep_data = sweep_data
        self.calibrations = calibrations
        self.conversion_method = conversion_method
        self.reference_tone_frequency = reference_tone_frequency
        self.smooth_window_hz = smooth_window_hz
        self.phase_center_params = phase_center_params
        self._sweeps = {}
        self._centers = {}
        self._conversions = {}

    def _sweep(self, key):
        if key not in self._sweeps:
            if self.sweep_data is None:
                raise ValueError('sweep_data is required for this signal')
            f, i, q = _get_sweep_trace(self.sweep_data, int(key))
            self._sweeps[key] = (np.asarray(f, dtype=float),
                                 np.asarray(i) + 1j * np.asarray(q))
        return self._sweeps[key]

    def _center(self, key):
        if key not in self._centers:
            params = self.phase_center_params
            if params is not None:
                if 'center' not in params:
                    params = next((params[candidate] for candidate in
                                   (key, int(key), f'{int(key):04d}')
                                   if candidate in params), None)
                if params is None:
                    raise ValueError(f'phase_center_params is missing tone {key}')
                params = dict(params)
                params.setdefault('rotation_angle', 0.0)
                if 'center' not in params or not np.isfinite(params['center']):
                    raise ValueError('phase_center_params needs a finite center')
                if not np.isfinite(params['rotation_angle']):
                    raise ValueError('phase_center_params needs a finite rotation_angle')
            else:
                if self.sweep_data is None:
                    raise ValueError('Centered phase requires sweep_data or '
                                     'phase_center_params')
                _, z = self._sweep(key)
                z = z[np.isfinite(z)]
                if len(z) < 3:
                    raise ValueError('Phase centering requires three finite sweep points')
                _, params = resonator.phase_center(z)
                if not np.isfinite(params['radius']) or params['radius'] <= 0:
                    raise ValueError(f'Cannot phase-center the sweep for tone {key}')
            self._centers[key] = params
        return self._centers[key]

    def _reference_frequency(self, samples, key, sweep_f=None):
        if sweep_f is None and self.reference_tone_frequency is None:
            frequencies = (samples.get('info') or {}).get('tones', {}).get('frequencies_hz')
            try:
                frequency = float(np.asarray(frequencies)[int(key)])
            except (IndexError, TypeError, ValueError) as exc:
                raise ValueError('Fractional frequency requires tone frequencies in '
                                 "samples['info'] or reference_tone_frequency") from exc
        else:
            frequency = timeseries._resolve_reference_tone_frequency(
                samples, int(key), sweep_f, self.reference_tone_frequency)
        if not np.isfinite(frequency) or frequency <= 0:
            raise ValueError(f'Tone {key} needs a finite positive reference frequency')
        return frequency

    def _convert(self, samples, key, z):
        f, sweep_z = self._sweep(key)
        frequency = self._reference_frequency(samples, key, f)
        cache_key = (key, frequency)
        if cache_key not in self._conversions:
            if self.conversion_method == 'linearized':
                calibration = resonator.LinearizedResonatorCalibration.from_sweep(
                    f, sweep_z, frequency, smooth_window_hz=self.smooth_window_hz)
                self._conversions[cache_key] = (calibration, None)
            else:
                calibration = resonator.calibration_for_tone(self.calibrations, int(key))
                if calibration is None:
                    raise ValueError(f"conversion_method='mobius' requires a calibration "
                                     f'for tone {key}')
                reference = resonator.interpolate_complex_trace(f, sweep_z, frequency)
                self._conversions[cache_key] = (calibration, reference)
        calibration, reference = self._conversions[cache_key]
        if self.conversion_method == 'linearized':
            frac_f, diss = calibration.convert_raw_iq(z)
            shift = frac_f * frequency
        else:
            shift, diss = calibration.convert_referenced_raw_iq(
                frequency, z, reference, method='mobius')
            frac_f = shift / frequency
        return {'frequency_shift': shift, 'fractional_frequency': frac_f,
                'dissipation': diss}

    def extract(self, samples, signals, tones=None):
        """Extract selected signals and absolute tone IDs (``None`` means all)."""
        if isinstance(signals, str):
            signals = [signals]
        names = list(dict.fromkeys(canonical_signal(name) for name in signals))
        result = {name: {} for name in names}
        for key in _tone_keys(samples, tones):
            i = np.asarray(samples['i_data'][key], dtype=float)
            q = np.asarray(samples['q_data'][key], dtype=float)
            if i.ndim != 1 or q.shape != i.shape:
                raise ValueError(f'I/Q for tone {key} must be matching 1D arrays')
            if 'num_samples' in samples and len(i) != samples['num_samples']:
                raise ValueError(f'I/Q length for tone {key} does not match num_samples')
            z = i + 1j * q
            values = {'i': i, 'q': q}
            if 'amp' in names or 'logmag' in names:
                values['amp'] = np.abs(z)
            if 'logmag' in names:
                with np.errstate(divide='ignore', invalid='ignore'):
                    values['logmag'] = 20 * np.log10(values['amp'])
            if 'phase' in names:
                values['phase'] = _unwrap_finite(z)
            for name in ('centered_phase', 'centered_rotated_phase'):
                if name in names:
                    if samples.get('modulation_demodulated'):
                        raise ValueError('Use phase for demodulated data; centered phase requires raw IQ')
                    centered = resonator.apply_phase_center_stages(
                        z, self._center(key), rotate=name == 'centered_rotated_phase')
                    values[name] = _unwrap_finite(centered)
            for name in ('detuning', 'detuning_linewidths', 'dphi_df',
                         'd2phi_df2', 'curvature_over_slope'):
                if name in names:
                    field = 'detuning_hz' if name == 'detuning' else name
                    if field not in samples:
                        raise ValueError(f'{name} requires demodulated data with modulation diagnostics')
                    values[name] = _precomputed_column(samples, field, key)
            converted = set(names) & _CONVERTED_SIGNALS
            converted |= {_LINEWIDTH_SIGNALS[name] for name in names
                          if name in _LINEWIDTH_SIGNALS}
            if converted:
                frequency_field = next((field for field in _FREQUENCY_KEYS
                                        if field in samples), None)
                if frequency_field is not None:
                    shift = _precomputed_column(samples, frequency_field, key)
                    values['frequency_shift'] = shift
                    if 'fractional_frequency' in converted:
                        values['fractional_frequency'] = shift / self._reference_frequency(samples, key)
                    values['dissipation'] = (_precomputed_column(samples, 'dissipation', key)
                                             if 'dissipation' in samples else np.full_like(shift, np.nan))
                elif converted == {'dissipation'} and 'dissipation' in samples:
                    values['dissipation'] = _precomputed_column(samples, 'dissipation', key)
                else:
                    if np.any(np.asarray(samples.get('modulation_point', [])) > 0):
                        raise ValueError('Frequency/dissipation from modulated IQ requires '
                                         'data_mode=demodulated')
                    values.update(self._convert(samples, key, z))
            for name, fractional in _LINEWIDTH_SIGNALS.items():
                if name in names:
                    values[name] = (timeseries._linewidth_scale(self.calibrations, key)
                                    * np.asarray(values[fractional], dtype=float))
            for name in names:
                value = np.asarray(values[name], dtype=float)
                if value.shape != i.shape:
                    raise ValueError(f'{name} for tone {key} must match the I/Q sample count')
                result[name][key] = value
        return result


def extract_signals(samples, signals, tones=None, *, sweep_data=None,
                    calibrations=None, conversion_method='linearized',
                    reference_tone_frequency=None, smooth_window_hz=1000,
                    phase_center_params=None):
    """One-shot ``SignalExtractor.extract``; reuse an extractor for live refreshes."""
    return SignalExtractor(
        sweep_data=sweep_data, calibrations=calibrations,
        conversion_method=conversion_method,
        reference_tone_frequency=reference_tone_frequency,
        smooth_window_hz=smooth_window_hz,
        phase_center_params=phase_center_params,
    ).extract(samples, signals, tones)
