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
from scipy.signal import butter, filtfilt, find_peaks, peak_widths
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
    marker_mag: Optional[float] = None
    marker_filt: Optional[float] = None


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
        if len(self.frequencies) < 2:
            return 1.0
        diffs = np.diff(self.frequencies)
        return float(np.median(diffs))
    
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
    freq_step = np.median(np.diff(frequencies)) if len(frequencies) > 1 else 1.0
    
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
    log_magnitude: np.ndarray,
    filtered_data: np.ndarray,
    peak_direction: int = -1,
) -> ResonanceResult:
    """
    Analyze a single resonance to extract Q-factor, FWHM, etc.
    
    Args:
        peak_idx: Index of the peak in the data arrays
        frequencies: Frequency array in Hz
        log_magnitude: Log magnitude data in dB
        filtered_data: Filtered data used for peak finding
        peak_direction: -1 for dips, +1 for peaks
        
    Returns:
        ResonanceResult with analysis data
    """
    n = len(frequencies)
    i_p1 = min(n - 1, peak_idx + 1)
    i_n1 = max(0, peak_idx - 1)
    
    frequency = frequencies[peak_idx]
    frequency_step = (frequencies[i_p1] - frequencies[i_n1]) / max(1, i_p1 - i_n1)
    
    # Adjust data for peak direction
    adjusted_data = peak_direction * filtered_data
    
    try:
        results_half = peak_widths(adjusted_data, [peak_idx], rel_height=0.5)
        width_samples = results_half[0][0]
        width_hz = width_samples * frequency_step
        fwhm = width_hz if width_hz > 0 else frequency_step
        q_factor = frequency / fwhm
        
        # Calculate dip depth in region around resonance
        mask = (frequencies < frequency + 5 * fwhm) & (frequencies > frequency - 5 * fwhm)
        if np.any(mask):
            dip_depth = float(np.max(log_magnitude[mask]) - np.min(log_magnitude[mask]))
        else:
            dip_depth = 0.0
            
    except Exception:
        fwhm = frequency_step
        q_factor = frequency / fwhm
        dip_depth = 0.0
    
    # Calculate Qc and Qi
    if dip_depth > 0:
        qc = q_factor / (1 - 10 ** (-dip_depth / 20))
    else:
        qc = float('inf')
    
    if q_factor != 0 and qc != 0:
        qi = 1.0 / (1.0 / q_factor - 1.0 / qc) if (1.0 / q_factor - 1.0 / qc) != 0 else float('inf')
    else:
        qi = float('inf')
    
    return ResonanceResult(
        peak_idx=peak_idx,
        frequency=frequency,
        fwhm=fwhm,
        q_factor=q_factor,
        qc=qc,
        qi=qi,
        dip_depth=dip_depth,
        marker_mag=float(log_magnitude[peak_idx]),
        marker_filt=float(filtered_data[peak_idx]),
    )


def find_mkid_resonances(
    frequencies: np.ndarray,
    s21_complex: np.ndarray,
    data_format: str = 'log_magnitude',
    filter_params: Optional[FilterParams] = None,
    finder_params: Optional[PeakFinderParams] = None,
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
    
    # Process data
    processor = DataProcessor(frequencies, s21_complex)
    raw_data = processor.get_data(data_format)
    log_mag = processor.get_data('log_magnitude')
    
    # Apply filtering
    filtered_data = apply_filter(raw_data, filter_params)
    
    # Find peaks
    peak_indices, _ = find_resonances(filtered_data, frequencies, finder_params)
    
    # Analyze each resonance
    results = []
    for idx in peak_indices:
        result = analyze_resonance(
            peak_idx=idx,
            frequencies=frequencies,
            log_magnitude=log_mag,
            filtered_data=filtered_data,
            peak_direction=finder_params.peak_direction,
        )
        results.append(result)
    
    # Sort by frequency
    results.sort(key=lambda r: r.frequency)
    
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
