"""
Standalone MKID peak/resonance finding module.

This module provides the core logic for finding MKID resonances in sweep data:
  1. Choose data format (magnitude, phase, group delay, etc.)
  2. Apply filtering (lowpass, highpass, median)
  3. Run scipy find_peaks
  4. Return list of resonance candidates

Can be used standalone or integrated with the GUI app or readout_client.

Author: Sam Rowe
Date: January 2026
"""

import numpy as np
from scipy.signal import butter, filtfilt, find_peaks
from scipy.ndimage import median_filter
from dataclasses import dataclass, field
from typing import Optional, Tuple, List, Dict, Any, Union


@dataclass
class FilterParams:
    """Parameters for data filtering before peak detection."""
    highpass_edge: float = 0.0  # 0-1 normalized frequency (0 = disabled)
    lowpass_edge: float = 1.0   # 0-1 normalized frequency (1 = disabled)
    median_kernel_size: int = 1  # 1 = disabled, must be odd

    def __post_init__(self):
        self.highpass_edge = max(0.0, min(1.0, self.highpass_edge))
        self.lowpass_edge = max(0.0, min(1.0, self.lowpass_edge))
        if self.lowpass_edge < self.highpass_edge:
            self.lowpass_edge = self.highpass_edge
        self.median_kernel_size = max(1, int(self.median_kernel_size))
        if self.median_kernel_size % 2 == 0:
            self.median_kernel_size -= 1


@dataclass
class PeakFinderParams:
    """Parameters for scipy.signal.find_peaks."""
    prominence_enabled: bool = True
    prominence_min: float = 1.0
    prominence_max: float = 100.0
    
    width_enabled: bool = True
    width_min: float = 100.0  # Hz
    width_max: float = 10_000_000.0  # Hz
    
    distance_enabled: bool = True
    distance_value: float = 1000.0  # Hz
    
    height_enabled: bool = False
    height_min: float = 0.0
    height_max: float = 1.0
    
    threshold_enabled: bool = False
    threshold_min: float = 0.0
    threshold_max: float = 1.0
    
    peak_direction: int = -1  # -1 for dips (resonances), +1 for peaks
    max_num_peaks: int = 10000

    # Frequency-range trim — exclude band edges where filtering artefacts
    # often produce spurious peaks.  None disables the trim on that side.
    f_low: Optional[float] = None   # Hz; drop peaks below this frequency
    f_high: Optional[float] = None  # Hz; drop peaks above this frequency


def _inner_frequency_trim(frequencies, low_fraction=0.10, high_fraction=0.90):
    """Return inner-band frequency bounds for wideband auto-detection."""
    f = np.asarray(frequencies, dtype=float).ravel()
    good = np.isfinite(f)
    if np.count_nonzero(good) < 2:
        return None, None
    f_min = float(np.nanmin(f[good]))
    f_max = float(np.nanmax(f[good]))
    bandwidth = f_max - f_min
    if not np.isfinite(bandwidth) or bandwidth <= 0.0:
        return None, None
    return (
        f_min + low_fraction * bandwidth,
        f_min + high_fraction * bandwidth,
    )


def _with_default_param_overrides(defaults, overrides, cls):
    """Build params from defaults plus caller-supplied dict overrides."""
    if overrides is None:
        return defaults
    if isinstance(overrides, dict):
        values = dict(vars(defaults))
        values.update(overrides)
        return cls(**values)
    return overrides


def _frequency_step_hz(frequencies) -> float:
    """Return a positive representative sweep step in Hz."""
    f = np.asarray(frequencies, dtype=float).ravel()
    good = np.isfinite(f)
    if np.count_nonzero(good) < 2:
        return 1.0
    diffs = np.abs(np.diff(f[good]))
    diffs = diffs[np.isfinite(diffs) & (diffs > 0.0)]
    if diffs.size == 0:
        return 1.0
    step = float(np.nanmedian(diffs))
    return step if np.isfinite(step) and step > 0.0 else 1.0


def wideband_resonance_search_params(frequencies, filter_params=None,
                                     finder_params=None):
    """Return conservative wideband resonance-search parameters.

    These defaults are intended for automatic wideband MKID resonance searches:
    ignore the noisy filter edges, look for dips, and require enough
    prominence/width/spacing to avoid fitting every small noise fluctuation.
    Caller-supplied dicts override individual defaults; fully constructed
    parameter objects are used unchanged.

    Parameters
    ----------
    frequencies : array-like
        Sweep frequency axis (Hz), used to set the inner frequency trim.
    filter_params : dict or FilterParams or None, optional
        Overrides for the pre-filter defaults (a dict updates individual
        fields; a :class:`FilterParams` is used as-is).
    finder_params : dict or PeakFinderParams or None, optional
        Overrides for the peak-finder defaults (prominence/width/spacing/dip
        settings), same dict-vs-object rule as ``filter_params``.
    """
    f_low, f_high = _inner_frequency_trim(frequencies)
    default_filter = FilterParams(
        highpass_edge=0.0,
        lowpass_edge=0.5,
        median_kernel_size=1,
    )
    default_finder = PeakFinderParams(
        prominence_enabled=True,
        prominence_min=1.0,
        prominence_max=100.0,
        width_enabled=True,
        width_min=1_000.0,
        width_max=10_000_000.0,
        distance_enabled=True,
        distance_value=100_000.0,
        peak_direction=-1,
        f_low=f_low,
        f_high=f_high,
    )
    return (
        _with_default_param_overrides(default_filter, filter_params, FilterParams),
        _with_default_param_overrides(default_finder, finder_params, PeakFinderParams),
    )


@dataclass
class ResonanceResult:
    """Result for a single detected resonance."""
    peak_idx: int
    frequency: float
    fwhm: Optional[float] = None
    q_factor: Optional[float] = None
    qc: Optional[float] = None
    qi: Optional[float] = None
    dip_depth: Optional[float] = None
    skew: Optional[float] = None
    marker_mag: Optional[float] = None
    marker_filt: Optional[float] = None


@dataclass
class ResonanceSearchResult:
    """Container returned by client-level resonance searches.

    It is list-like over ``all_resonances`` for compatibility with the old
    wideband return value, and mapping-like for metadata shared by wideband and
    targeted searches.
    """
    mode: str
    all_resonances: List[ResonanceResult] = field(default_factory=list)
    per_tone: List[List[ResonanceResult]] = field(default_factory=list)
    flagged_tones: List[int] = field(default_factory=list)
    num_tones: int = 0

    _mapping_keys = (
        'mode',
        'all_resonances',
        'per_tone',
        'flagged_tones',
        'num_tones',
    )

    def __iter__(self):
        return iter(self.all_resonances)

    def __len__(self):
        return len(self.all_resonances)

    def __bool__(self):
        return bool(self.all_resonances)

    def __getitem__(self, key):
        if isinstance(key, str):
            if key not in self._mapping_keys:
                raise KeyError(key)
            return getattr(self, key)
        return self.all_resonances[key]

    def __contains__(self, item):
        if isinstance(item, str) and item in self._mapping_keys:
            return True
        return item in self.all_resonances

    def keys(self):
        return self._mapping_keys

    def values(self):
        return tuple(getattr(self, key) for key in self._mapping_keys)

    def items(self):
        return tuple((key, getattr(self, key)) for key in self._mapping_keys)

    def get(self, key, default=None):
        """Dict-style accessor: return field ``key`` if present, else ``default``."""
        if key in self._mapping_keys:
            return getattr(self, key)
        return default

    def as_dict(self):
        return dict(self.items())


class DataProcessor:
    """
    Converts raw S21 complex data into various analysis formats.
    """
    
    FORMATS = [
        'lin_magnitude',
        'log_magnitude',
        'phase',
        'unwrapped_phase',
        'group_delay',
        'complex_gradient',
    ]
    
    def __init__(self, frequencies: np.ndarray, s21_complex: np.ndarray):
        """
        Initialize with sweep data.
        
        Args:
            frequencies: Array of frequencies in Hz
            s21_complex: Complex S21 data
        """
        self.frequencies = np.asarray(frequencies)
        self.s21_complex = np.asarray(s21_complex)
        self._cache: Dict[str, np.ndarray] = {}
        
    @property
    def frequency_step(self) -> float:
        """Average frequency step size."""
        return _frequency_step_hz(self.frequencies)
    
    def get_data(self, format_name: str) -> np.ndarray:
        """
        Get data in the specified format.
        
        Args:
            format_name: One of FORMATS
            
        Returns:
            Processed data array
        """
        if format_name in self._cache:
            return self._cache[format_name]
        
        if format_name == 'lin_magnitude':
            data = np.abs(self.s21_complex)
        elif format_name == 'log_magnitude':
            data = 20.0 * np.log10(np.abs(self.s21_complex) + 1e-30)
        elif format_name == 'phase':
            data = np.angle(self.s21_complex)
        elif format_name == 'unwrapped_phase':
            data = np.unwrap(np.angle(self.s21_complex))
        elif format_name == 'group_delay':
            phase = np.unwrap(np.angle(self.s21_complex))
            data = -np.gradient(phase, self.frequencies) * 1e6  # microseconds
        elif format_name == 'complex_gradient':
            data = np.abs(np.gradient(self.s21_complex, self.frequencies))
        else:
            raise ValueError(f"Unknown format: {format_name}")
        
        self._cache[format_name] = data
        return data


def apply_filter(data: np.ndarray, params: FilterParams) -> np.ndarray:
    """
    Apply lowpass, highpass, and median filters to data.
    
    Args:
        data: Input data array
        params: FilterParams instance
        
    Returns:
        Filtered data array
    """
    result = data.copy()
    n = len(result)
    
    # Adjust filter edges based on data length
    hp_edge = params.highpass_edge
    lp_edge = params.lowpass_edge
    
    if hp_edge > 0 and hp_edge <= 1.0 / n:
        hp_edge = 1.0 / n
    if lp_edge < 1.0:
        lp_edge = max(lp_edge, 1.0 / n)
        lp_edge = min(lp_edge, 1.0 - 1.0 / n)
    
    # Highpass filter
    if hp_edge > 0 and hp_edge < 1.0:
        b, a = butter(2, hp_edge, btype='highpass', fs=2.0)
        result = filtfilt(b, a, result)
    
    # Lowpass filter
    if lp_edge < 1.0 and lp_edge > 0:
        b, a = butter(2, lp_edge, btype='lowpass', fs=2.0)
        result = filtfilt(b, a, result)
    
    # Median filter
    kernel = min(params.median_kernel_size, n)
    if kernel > 1:
        if kernel % 2 == 0:
            kernel -= 1
        if kernel >= 1:
            result = median_filter(result, kernel)
    
    return result


def find_resonances(
    data: np.ndarray,
    frequencies: np.ndarray,
    params: PeakFinderParams,
) -> Tuple[np.ndarray, Dict[str, np.ndarray]]:
    """
    Find peaks/dips in filtered data using scipy.signal.find_peaks.
    
    Args:
        data: Filtered data array
        frequencies: Frequency array in Hz
        params: PeakFinderParams instance
        
    Returns:
        Tuple of (peak_indices, properties_dict)
    """
    freq_step = _frequency_step_hz(frequencies)
    
    # Build find_peaks kwargs
    kwargs: Dict[str, Any] = {}
    
    if params.prominence_enabled:
        kwargs['prominence'] = (params.prominence_min, params.prominence_max)
    
    if params.width_enabled:
        w_min = max(1, params.width_min / freq_step)
        w_max = max(1, params.width_max / freq_step)
        kwargs['width'] = (w_min, w_max)
    
    if params.distance_enabled and params.distance_value > 0:
        kwargs['distance'] = max(1, params.distance_value / freq_step)
    
    if params.height_enabled:
        kwargs['height'] = (params.height_min, params.height_max)
    
    if params.threshold_enabled:
        kwargs['threshold'] = (params.threshold_min, params.threshold_max)
    
    # Apply peak direction (multiply by -1 for dips)
    search_data = data * params.peak_direction
    
    peaks, properties = find_peaks(search_data, **kwargs)

    # Drop peaks outside [f_low, f_high].  Done AFTER find_peaks so the
    # filter / prominence / width logic still sees full-band context — only
    # the returned peak list is trimmed.
    if params.f_low is not None or params.f_high is not None:
        peak_freqs = frequencies[peaks]
        mask = np.ones(len(peaks), dtype=bool)
        if params.f_low is not None:
            mask &= peak_freqs >= params.f_low
        if params.f_high is not None:
            mask &= peak_freqs <= params.f_high
        peaks = peaks[mask]
        properties = {k: v[mask] for k, v in properties.items()}

    # Limit number of peaks
    if len(peaks) > params.max_num_peaks:
        peaks = peaks[:params.max_num_peaks]
        properties = {k: v[:params.max_num_peaks] for k, v in properties.items()}

    return peaks, properties


def analyze_resonance(
    peak_idx: int,
    frequencies: np.ndarray,
    s21_complex: np.ndarray,
    log_magnitude: np.ndarray,
    filtered_data: np.ndarray,
) -> ResonanceResult:
    """
    Analyze a single resonance to extract Q-factor, FWHM, etc.
    
    Args:
        peak_idx: Index of the peak in the data arrays
        frequencies: Frequency array in Hz
        s21_complex: Complex S21 data
        log_magnitude: Log magnitude data in dB
        filtered_data: Filtered data used for peak finding
        
    Returns:
        ResonanceResult with analysis data
    """
    from .resonator import estimate_resonance_empirical

    estimate = estimate_resonance_empirical(
        frequencies,
        s21_complex,
        peak_index=peak_idx,
    )
    
    return ResonanceResult(
        peak_idx=peak_idx,
        frequency=estimate.fr,
        fwhm=estimate.linewidth_hz,
        q_factor=estimate.Ql,
        qc=estimate.Qc,
        qi=estimate.Qi,
        dip_depth=estimate.dip_depth_db,
        skew=estimate.skew,
        marker_mag=float(log_magnitude[peak_idx]),
        marker_filt=float(filtered_data[peak_idx]),
    )


def _verbose_level(verbose):
    if isinstance(verbose, bool):
        return 1 if verbose else 0
    try:
        return int(verbose)
    except (TypeError, ValueError):
        return 0


def _candidate_analysis_slice(peak_idx, candidate_index, properties, n_points,
                              pad_widths=10.0, min_points=101):
    """Return a local data window wide enough to estimate one candidate."""
    start = int(peak_idx)
    stop = int(peak_idx) + 1

    left_ips = properties.get('left_ips')
    right_ips = properties.get('right_ips')
    if left_ips is not None and right_ips is not None:
        left = float(left_ips[candidate_index])
        right = float(right_ips[candidate_index])
        if np.isfinite(left) and np.isfinite(right) and right > left:
            width = right - left
            margin = pad_widths * width
            start = int(np.floor(left - margin))
            stop = int(np.ceil(right + margin)) + 1

    if stop - start < min_points:
        extra = int(np.ceil((min_points - (stop - start)) / 2.0))
        start -= extra
        stop += extra

    start = max(0, start)
    stop = min(n_points, stop)
    if stop <= start:
        stop = min(n_points, start + 1)
    return slice(start, stop)


def find_mkid_resonances(
    frequencies: np.ndarray,
    s21_complex: np.ndarray,
    data_format: str = 'log_magnitude',
    filter_params: Optional[FilterParams] = None,
    finder_params: Optional[PeakFinderParams] = None,
    verbose: Union[bool, int] = False,
) -> List[ResonanceResult]:
    """
    Main entry point: find MKID resonances in sweep data.
    
    This is the complete pipeline:
      1. Convert S21 to chosen data format
      2. Apply filtering
      3. Find peaks
      4. Analyze each resonance
    
    Args:
        frequencies: Frequency array in Hz
        s21_complex: Complex S21 data
        data_format: One of DataProcessor.FORMATS
        filter_params: Optional FilterParams (defaults used if None)
        finder_params: Optional PeakFinderParams (defaults used if None)
        verbose: Print resonance-search progress. ``True`` prints stage
            summaries; ``2`` also prints each analyzed candidate.
        
    Returns:
        List of ResonanceResult objects sorted by frequency
        
    Example:
        >>> import numpy as np
        >>> from souk_readout_tools.peak_finder import find_mkid_resonances, FilterParams, PeakFinderParams
        >>> 
        >>> # Load your sweep data
        >>> data = np.load('sweep.npy', allow_pickle=True).item()
        >>> f = data['sweep_f']
        >>> z = data['sweep_i'] + 1j * data['sweep_q']
        >>> 
        >>> # Find resonances with default parameters
        >>> resonances = find_mkid_resonances(f, z)
        >>> 
        >>> # Or customize parameters
        >>> filter_params = FilterParams(highpass_edge=0.001, lowpass_edge=0.5)
        >>> finder_params = PeakFinderParams(prominence_min=2.0, distance_value=50000)
        >>> resonances = find_mkid_resonances(f, z, filter_params=filter_params, finder_params=finder_params)
        >>> 
        >>> # Get frequencies of found resonances
        >>> freqs_mhz = [r.frequency / 1e6 for r in resonances]
    """
    if filter_params is None:
        filter_params = FilterParams()
    if finder_params is None:
        finder_params = PeakFinderParams()
    verbose = _verbose_level(verbose)
    frequencies = np.asarray(frequencies, dtype=float).ravel()
    s21_complex = np.asarray(s21_complex, dtype=complex).ravel()
    if frequencies.shape != s21_complex.shape:
        raise ValueError("frequencies and s21_complex must have the same shape.")
    
    # Process data
    if verbose:
        print(
            f"Finding resonances in {frequencies.size} sweep points "
            f"using {data_format}...",
            flush=True,
        )
    processor = DataProcessor(frequencies, s21_complex)
    raw_data = processor.get_data(data_format)
    log_mag = processor.get_data('log_magnitude')
    
    # Apply filtering
    if verbose:
        print("  Filtering resonance-search data...", flush=True)
    filtered_data = apply_filter(raw_data, filter_params)
    
    # Find peaks
    if verbose:
        print("  Running peak finder...", flush=True)
    peak_indices, properties = find_resonances(
        filtered_data, frequencies, finder_params)
    total = len(peak_indices)
    if verbose:
        print(f"  Found {total} candidate resonances.", flush=True)
        if total >= finder_params.max_num_peaks:
            print(
                f"  Candidate list reached max_num_peaks="
                f"{finder_params.max_num_peaks}; peak finding may be noise-limited.",
                flush=True,
            )
    
    # Analyze each resonance
    results = []
    report_every = max(1, int(np.ceil(total / 10.0))) if total else 1
    for candidate_index, idx in enumerate(peak_indices):
        window = _candidate_analysis_slice(
            idx, candidate_index, properties, frequencies.size)
        local_idx = int(idx - window.start)
        result = analyze_resonance(
            peak_idx=local_idx,
            frequencies=frequencies[window],
            s21_complex=s21_complex[window],
            log_magnitude=log_mag[window],
            filtered_data=filtered_data[window],
        )
        result.peak_idx = int(idx)
        result.marker_mag = float(log_mag[idx])
        result.marker_filt = float(filtered_data[idx])
        results.append(result)
        completed = candidate_index + 1
        if verbose >= 2 or (verbose == 1 and (
            completed == 1 or completed == total or completed % report_every == 0
        )):
            print(
                f"  Analyzed {completed}/{total} candidates "
                f"({100.0 * completed / max(total, 1):.0f}%).",
                flush=True,
            )
    
    # Sort by frequency
    results.sort(key=lambda r: r.frequency)
    if verbose:
        print(f"  Resonance analysis complete: {len(results)} candidates.", flush=True)
    
    return results


# Convenience function for quick use
def find_resonance_frequencies(
    frequencies: np.ndarray,
    s21_complex: np.ndarray,
    **kwargs
) -> np.ndarray:
    """
    Quick helper to get just the resonance frequencies.
    
    Args:
        frequencies: Frequency array in Hz
        s21_complex: Complex S21 data
        **kwargs: Passed to find_mkid_resonances
        
    Returns:
        Array of resonance frequencies in Hz
    """
    results = find_mkid_resonances(frequencies, s21_complex, **kwargs)
    return np.array([r.frequency for r in results])
