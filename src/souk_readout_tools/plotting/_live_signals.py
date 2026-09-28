"""Signal extraction for the live viewer and possible GUI front ends.

A thin adapter over :func:`souk_readout_tools.resonator.convert_samples`,
which holds every definition: this only keeps prepared conversions between
refreshes, and -- given a sweep but no calibrations -- builds the model-free
sweep calibration (``ResonatorCalibration.from_sweep``) once per tone so the
centred panels still have a frame. Tone selections refer to absolute tone
IDs, including in subset captures.
"""

from .. import resonator


SIGNAL_LABELS = resonator.QUANTITIES
canonical_signal = resonator.canonical_quantity

# Quantities that need a circle-centring frame from some calibration.
_NEEDS_FRAME = ('centered_phase', 'centered_rotated_phase', 'centered_radius')


class SignalExtractor:
    """Extract named quantities from ``parse_samples``-shaped dictionaries.

    ``extract`` returns ``{canonical_name: {original_tone_key: values}}``; the
    names, signs and units are ``resonator.QUANTITIES``. Conversions use
    ``calibrations`` (a record, fits or ``ResonatorCalibration`` objects),
    else the calibration the capture carries, else -- for the centred phases
    and radius only -- a model-free calibration of ``sweep_data`` (Kasa circle
    after an estimated cable delay; ``note`` says so), which is close to but
    not the fitted frame the tracker uses. Create a new extractor when the
    sweep or calibrations change.
    """

    def __init__(self, *, sweep_data=None, calibrations=None,
                 conversion_method='linearized', reference_tone_frequency=None,
                 smooth_window_hz=1000):
        if conversion_method not in resonator.CONVERSION_METHODS:
            raise ValueError(
                f'conversion_method must be one of {resonator.CONVERSION_METHODS}')
        self.sweep_data = sweep_data
        self.calibrations = calibrations
        self.conversion_method = conversion_method
        self.reference_tone_frequency = reference_tone_frequency
        self.smooth_window_hz = smooth_window_hz
        self.note = ''
        self._cache = {}
        self._sweep_calibrations = {}

    def _sweep_frame(self, samples, tones):
        """Model-free calibrations of the sweep, one per tone, built once."""
        for key in resonator.capture_tone_keys(samples, tones):
            tone = int(key)
            if tone not in self._sweep_calibrations:
                f, i, q = resonator.sweep_trace(self.sweep_data, tone)
                self._sweep_calibrations[tone] = resonator.ResonatorCalibration.from_sweep(
                    f, i + 1j * q)
        self.note = ('centred with a model-free sweep calibration; pass '
                     'calibrations for the fitted frame the tracker uses')
        return self._sweep_calibrations

    def extract(self, samples, signals, tones=None):
        """Extract selected quantities for absolute tone IDs (``None`` means all)."""
        if isinstance(signals, str):
            signals = [signals]
        names = list(dict.fromkeys(canonical_signal(name) for name in signals))
        calibrations = self.calibrations
        if calibrations is None:
            calibrations = resonator.calibration_from_data(samples)
        if (calibrations is None and self.sweep_data is not None
                and any(name in _NEEDS_FRAME for name in names)):
            calibrations = self._sweep_frame(samples, tones)
        if calibrations is None and any(name in _NEEDS_FRAME for name in names):
            raise ValueError('Centred quantities need calibrations or sweep_data')
        return resonator.convert_samples(
            samples, names, tones, method=self.conversion_method,
            calibrations=calibrations, sweep_data=self.sweep_data,
            reference_tone_frequency=self.reference_tone_frequency,
            smooth_window_hz=self.smooth_window_hz, cache=self._cache)


def extract_signals(samples, signals, tones=None, *, sweep_data=None,
                    calibrations=None, conversion_method='linearized',
                    reference_tone_frequency=None, smooth_window_hz=1000):
    """One-shot ``SignalExtractor.extract``; reuse an extractor for live refreshes."""
    return SignalExtractor(
        sweep_data=sweep_data, calibrations=calibrations,
        conversion_method=conversion_method,
        reference_tone_frequency=reference_tone_frequency,
        smooth_window_hz=smooth_window_hz,
    ).extract(samples, signals, tones)
