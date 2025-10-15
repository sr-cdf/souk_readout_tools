import sys
import os
import traceback

try:
    from importlib.resources import  files  # Python 3.9+
except ImportError:
    from importlib_resources import  files  # Python < 3.9

import numpy as np

from scipy.signal import butter, filtfilt, find_peaks, peak_widths
from scipy.ndimage import median_filter

import matplotlib
import matplotlib.transforms
import matplotlib.pyplot as plt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.widgets import RectangleSelector
from matplotlib.path import Path
from matplotlib.textpath import TextPath
from matplotlib.transforms import Affine2D




from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout,
    QFileDialog, QSplitter, QGroupBox, QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget,
    QTableWidgetItem, QCheckBox, QHeaderView, QMessageBox, QInputDialog, QMenu, QAction,
    QGridLayout, QSplashScreen, QProgressBar, QPlainTextEdit
)

# Use adaptive step if available
from PyQt5.QtWidgets import QAbstractSpinBox
try:
    StepType = QAbstractSpinBox.AdaptiveDecimalStepType
except AttributeError:
    StepType = QAbstractSpinBox.DefaultStepType

from PyQt5.QtCore import Qt, QSettings, QByteArray, QPoint, QTimer, QEvent, QItemSelection, QItemSelectionModel,QRegExp,QPropertyAnimation
from PyQt5.QtGui import QIcon,QRegExpValidator,QPixmap,QPainter
from PyQt5 import QtGui



class ScientificSpinBox(QDoubleSpinBox):
    def __init__(self, parent=None):
        super().__init__(parent)
        # Customize the validator to allow scientific notation:
        # This regex allows optional sign, digits, optional decimal point,
        # and an optional exponent part with 'e' or 'E'.
        regex = QRegExp(r"^[+-]?(\d+(\.\d*)?|\.\d+)([eE][+-]?\d+)?$")
        validator = QRegExpValidator(regex)
        self.lineEdit().setValidator(validator)

    def valueFromText(self, text):
        try:
            text=text.replace(',','')
            return float(text)
        except ValueError:
            return np.nan
    

class Resonance:
    def __init__(self,id=0):
        self.peak_idx = None  # Index in the sweep frequency array
        self.frequency = None  # Resonance frequency
        self.fwhm = None
        self.q_factor = None
        self.dip_depth=None
        self.marker_freq=None
        self.marker_mag = None
        self.marker_filt = None
        # self.marker_color = 'r'
        # self.marker_shape = 'o'
        # self.marker_text = None
        self.save = True  # Whether to save/export this resonance
        self.is_active = False
        self.is_selected = False
        self.id = id
        self.name = '%04d'%id

    def analyse(self, frequencies, logmag_data, filtered_data, peak_direction, peak_idx):
        """Perform analysis and store results."""
        print(f'Resonance, analyse, {frequencies[peak_idx]/1e6}')
        self.peak_idx = peak_idx
        i_p1 = min(len(frequencies)-1, peak_idx + 1)
        i_n1 = min(0, peak_idx - 1)

        self.frequency = frequencies[peak_idx]
        frequency_step = (frequencies[i_p1] - frequencies[i_n1]) / (i_p1 - i_n1)

        # Adjust data for peak direction
        adjusted_data = peak_direction * filtered_data
        try:
            # Use scipy peak_widths to calculate width at half maximum
            results_half = peak_widths(adjusted_data, [peak_idx], rel_height=0.5)
            width_samples = results_half[0][0]  # Width in number of samples
            width_hz = width_samples * frequency_step
            fwhm = width_hz if width_hz else frequency_step
            q_factor = self.frequency / fwhm
            mask = (frequencies < self.frequency + 10*fwhm/2) & (frequencies > self.frequency - 10*fwhm/2) 
            dip_depth = max(logmag_data[mask]) - min(logmag_data[mask])
            
        except Exception as e:
            print(f"Error in analyse method: {e}")
            width_hz = frequency_step
            fwhm = width_hz
            q_factor = self.frequency / fwhm
            dip_depth = 0
        
        qc = q_factor/(1-10**(-dip_depth/20)) if dip_depth > 0 else np.inf
        qi = 1/(1/q_factor - 1/qc) if (1/q_factor - 1/qc) != 0 else np.inf

        #coupling constant:
        

        # Store results
        self.fwhm = fwhm
        self.q_factor = q_factor
        self.qc = qc
        self.qi = qi
        self.dip_depth = dip_depth

        # Store marker values
        self.marker_freq = self.frequency
        self.marker_mag = logmag_data[peak_idx]
        self.marker_filt = filtered_data[peak_idx]

    def set_id(self,new_id):
        self.id = new_id
    
    def set_save_state(self,state):
        self.save = state
        # self.update_marker_style()
    
    def set_active_state(self,state):
        self.is_active = state
        # self.update_marker_style()
    
    def set_selected_state(self,state):
        self.is_selected = state
        # self.update_marker_style()
    
    def set_name(self,new_name):
        self.name = new_name
        # self.update_marker_style()

    def toggle_save_state(self):
        self.save = not self.save
        # self.update_marker_style()



class DataLoader():
    def __init__(self):
        pass

    def load_file(self,filename):
        if not filename:
            raise ValueError('No filename provided')
        ext = os.path.splitext(filename)[-1]
        if ext == '.fits':
            f,z=self.load_from_fits(filename)
        elif ext == '.npy':
            f,z=self.load_from_npy(filename)
        elif ext == '.txt':
            f,z=self.load_from_txt(filename)
        else:
            QMessageBox.warning(None, "Error", f"File type not supported: {ext}")
            return None,None
        print(f'loaded {len(f)} points, shape={f.shape}')

        return f,z

    def load_from_fits(self,filename):
        try:
            from astropy.io import fits
        except ImportError as e:
            QMessageBox.warning(None, "Error", "Cannot open fits file, please try:\npip install astropy")
            raise(e)
        try:
            hdu,data = fits.open(filename)
            f = data.data['Freq']
            z = data.data['ReS21'] + 1j*data.data['ImS21']
        except Exception as e:
            QMessageBox.warning(None, "Error", "Failed to interpret file as a sweep: "+filename)
            raise(e)
        ss=np.argsort(f.ravel())
        frequencies = f.ravel()[ss]
        s21_complex = z.ravel()[ss]
        return frequencies, s21_complex

    def load_from_npy(self, filename):
        print(filename)
        data = np.load(filename,allow_pickle=True)
        if data.dtype == np.object_:
            #try load readout_client sweep
            data = data.item()
            try:
                print(data.keys())
                f = data['sweep_f']
                z = data['sweep_i']+1j*data['sweep_q']    
            except Exception as e:
                    QMessageBox.warning(None, "Error", "Failed to interpret file as a sweep: "+filename)
                    raise(e)
            ss=np.argsort(f.ravel())
            frequencies=f.ravel()[ss]
            s21_complex=z.ravel()[ss]
            return frequencies, s21_complex
        elif data.ndim == 2 and data.shape[0] == 2:
            f = data[0].real
            z = data[1]
            ss=np.argsort(f.ravel())
            frequencies=f.ravel()[ss]
            s21_complex=z.ravel()[ss]
            return frequencies, s21_complex
        elif data.ndim == 2 and data.shape[0] == 3:
            f = data[0]
            z = data[1]+1j*data[2]
            ss=np.argsort(f.ravel())
            frequencies=f.ravel()[ss]
            s21_complex=z.ravel()[ss]
            return frequencies, s21_complex
        else:
            QMessageBox.warning(None, "Error", "Failed to interpret file as a sweep: "+filename)
            raise ValueError('format of data in npy file not understood')

    def load_from_txt(self, filename):
        try:
            data = np.loadtxt(filename, delimiter=',')
            frequencies = data[:, 0]
            s21_complex = data[:, 1] + 1j * data[:, 2]
        except Exception as e:
            print(f'not a txt file with columns = f,i,q: {e}')  
            QMessageBox.warning(None, "Error", f'not a txt file with columns = f,i,q: {e}')
            raise(e)
        return frequencies, s21_complex
    



class FilterManager():
    def __init__(self):
        self.highpass_edge = 0.0
        self.lowpass_edge = 1.0
        self.median_kernel_size = 1

    def get_filter_params(self):
        params = {'highpass_edge':self.highpass_edge,
                  'lowpass_edge':self.lowpass_edge,
                  'median_kernel_size':self.median_kernel_size}
        return params

    def set_filter_params(self,filter_params):
        highpass_edge = filter_params.get('highpass_edge', self.highpass_edge)
        lowpass_edge = filter_params.get('lowpass_edge', self.lowpass_edge)
        median_kernel_size = filter_params.get('median_kernel_size', self.median_kernel_size)
        self._set_highpass_edge(highpass_edge)
        self._set_lowpass_edge(lowpass_edge)
        self._set_median_kernel_size(median_kernel_size)
        
    def _set_highpass_edge(self, value):
        if value < 0:
            value = 0.0
        if value > 1:
            value = 1.0
        if value > self.lowpass_edge:
            return
        self.highpass_edge = value
    
    def _set_lowpass_edge(self, value):
        if value <0:
            value = 0.0
        if value > 1:
            value = 1.0
        if value < self.highpass_edge:
            value = self.highpass_edge
        self.lowpass_edge = value

    def _set_median_kernel_size(self,value):
        value = round(value)
        if value % 2 == 0:
            value -= 1
        if value < 1:
            value = 1
        self.median_kernel_size = int(value)

    def filter_data(self, data):
        print('filter_data')
        self.align_with_sample_interval(len(data))
        data = self.perform_highpass(data, self.highpass_edge)
        data = self.perform_lowpass(data, self.lowpass_edge)
        data = self.perform_median_filter(data, self.median_kernel_size)
        return data

    def align_with_sample_interval(self, num_samples):
        print('align_with_sample_interval')
        # self.highpass_edge = self.highpass_edge %  (1./num_samples)
        # self.lowpass_edge = self.lowpass_edge % (1./num_samples)

        if self.highpass_edge == 0:
            pass
        elif self.highpass_edge <= 1./num_samples:
            self.highpass_edge = 1./num_samples
        else:
            self.highpass_edge = max(self.highpass_edge, 1./num_samples)
            self.highpass_edge =  min(self.highpass_edge, 1 - 1./num_samples)
        if self.lowpass_edge != 1:
            self.lowpass_edge = max(self.lowpass_edge, 1./num_samples)
            self.lowpass_edge =  min(self.lowpass_edge, 1 - 1./num_samples)
        if self.median_kernel_size > num_samples:
            self._set_median_kernel_size(num_samples)
            
    def perform_highpass(self, data, highpass_edge):
        if self.highpass_edge == 0:
            return data
        else:
            b, a = butter(2, highpass_edge, btype='highpass',fs=2.0)
            return filtfilt(b, a, data)
    
    def perform_lowpass(self, data,lowpass_edge):
        if lowpass_edge == 1:
            return data
        else:
            b, a = butter(2, lowpass_edge, btype='lowpass', fs=2.0)
            return filtfilt(b, a, data)
    
    def perform_median_filter(self, data, kernel_size):
        if kernel_size == 1:
            return data
        else:
            return median_filter(data, kernel_size)            



class PeakFinderManager():
    def __init__(self,max_num_peaks=10000):
        self.max_num_peaks = max_num_peaks
        self.prominence_enabled = False
        self.prominence_min = None
        self.prominence_max = None
        self.width_enabled = False
        self.width_min = None
        self.width_max = None
        self.threshold_enabled = False
        self.threshold_min = None
        self.threshold_max = None
        self.height_enabled = False
        self.height_min = None
        self.height_max = None
        self.distance_enabled = False
        self.distance_value = None
        self.peak_direction = None
        self.frequency_stepsize = 1

    def get_finder_params(self):
        params = {'prominence_enabled':self.prominence_enabled,
                  'prominence_min':self.prominence_min,
                  'prominence_max':self.prominence_max,
                  'width_enabled':self.width_enabled,
                  'width_min':self.width_min,
                  'width_max':self.width_max,
                  'threshold_enabled':self.threshold_enabled,
                  'threshold_min':self.threshold_min,
                  'threshold_max':self.threshold_max,
                  'height_enabled':self.height_enabled,
                  'height_min':self.height_min,
                  'height_max':self.height_max,
                  'distance_enabled':self.distance_enabled,
                  'distance_value':self.distance_value,
                  'peak_direction':self.peak_direction,
                  'frequency_stepsize':float(self.frequency_stepsize)}
        return params

    def set_finder_parameters(self,params):
        self.prominence_enabled = params.get('prominence_enabled', self.prominence_enabled)
        self.prominence_min = params.get('prominence_min', self.prominence_min)
        self.prominence_max = params.get('prominence_max', self.prominence_max)
        self.width_enabled = params.get('width_enabled', self.width_enabled)
        self.width_min = params.get('width_min', self.width_min)
        self.width_max = params.get('width_max', self.width_max)
        self.threshold_enabled = params.get('threshold_enabled', self.threshold_enabled)
        self.threshold_min = params.get('threshold_min', self.threshold_min)
        self.threshold_max = params.get('threshold_max', self.threshold_max)
        self.height_enabled = params.get('height_enabled', self.height_enabled)
        self.height_min = params.get('height_min', self.height_min)
        self.height_max = params.get('height_max', self.height_max)
        self.distance_enabled = params.get('distance_enabled', self.distance_enabled)
        self.distance_value = params.get('distance_value', self.distance_value)
        self.peak_direction = params.get('peak_direction', self.peak_direction)
        self.frequency_stepsize = params.get('frequency_stepsize', self.frequency_stepsize)


    def perform_find_peaks(self, data):
        print('perform_find_peaks',data)
        params = self.get_finder_params()
        prominence = (params['prominence_min'], params['prominence_max']) if params['prominence_enabled'] else None
        width = (params['width_min'], params['width_max']) if params['width_enabled'] else None
        threshold = (params['threshold_min'], params['threshold_max']) if params['threshold_enabled'] else None
        height = (params['height_min'], params['height_max']) if params['height_enabled'] else None
        distance = (params['distance_value']) if params['distance_enabled'] else None
        peak_direction = params['peak_direction']
        frequency_stepsize = params['frequency_stepsize']

        width = (max(1,width[0]/frequency_stepsize),max(1,width[1]/frequency_stepsize)) if width is not None else None
        distance = max(1,distance/frequency_stepsize) if distance is not None else None

        try:
            print('find_peaks')
            peaks, properties = find_peaks(
                data * peak_direction,
                prominence=prominence,
                width=width,
                distance=distance,
                threshold=threshold,
                height=height
            )
            peaks = peaks[:self.max_num_peaks]
            properties = {key: value[:self.max_num_peaks] for key, value in properties.items()}
            return peaks, properties

        except Exception as e:
            tb=traceback.format_exc()
            print(f"Error in find_peaks: {e}\n{tb}")
            raise(e)

    

class ResonanceFinderApp(QMainWindow):
    """
    An interactive MKID resonance finder application built with PyQt5.
    """
    def __init__(self,splash_screen):
        super().__init__()
        splash_progress = 0
        splash_steps = 100

        # Managers
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing data loader...")
        splash_progress += 1
        self.dataLoader = DataLoader()


        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing filter manager...")
        splash_progress += 1
        self.filterManager = FilterManager()
        
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing peak finder manager...")
        splash_progress += 1
        self.peakFinderManager = PeakFinderManager()
        
        # Variables
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing variables...")
        splash_progress += 1

        # Raw data
        self.filename = None
        self.frequencies = np.zeros(0)
        self.s21_complex = np.zeros(0,dtype='complex')

        # Processing
        self.frequency_stepsize = None
        self.magnitude = np.zeros(0)
        self.log_magnitude = np.zeros(0)
        self.phase = np.zeros(0)
        self.unwrap_phase = np.zeros(0)
        self.group_delay_us = np.zeros(0)
        self.complex_gradient = np.zeros(0)
        self.sin_iq_didq = np.zeros(0)

        # Filtering
        self.analysis_formats = ['Lin Magnitude V','Log Magnitude dB', 'Phase rad', 'Unwrapped Phase rad',
            'Group Delay us (-dphi/df)', 'Complex Gradient V/Hz (speed)','Sin(IQ,dIdQ) (?)']
        self.active_format = ''
        self.filtered_data = np.zeros(0)

        # Peak finding
        self.peak_finder_labels = {
            'Lin Magnitude V':                 {'direction': 'Direction (peak or dip)','prominence': 'Prominence (dip depth) [V]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [V]', 'threshold': 'Threshold (?) [V]'},
            'Log Magnitude dB':                {'direction': 'Direction (peak or dip)','prominence': 'Prominence (dip depth) [dB]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [dB]', 'threshold': 'Threshold (?) [dB]'},
            'Phase rad':                       {'direction': 'Direction (peak or dip)','prominence': 'Prominence (peak depth) [rad]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [rad]', 'threshold': 'Threshold (?) [rad]'},
            'Unwrapped Phase rad':             {'direction': 'Direction (peak or dip)','prominence': 'Prominence (peak depth) [rad]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [rad]', 'threshold': 'Threshold (?) [rad]'},
            'Group Delay us (-dphi/df)':       {'direction': 'Direction (peak or dip)','prominence': 'Prominence (peak height) [us]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [us]', 'threshold': 'Threshold (?) [us]'},
            'Complex Gradient V/Hz (speed)':   {'direction': 'Direction (peak or dip)','prominence': 'Prominence (peak height) [V/Hz]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [V/Hz]', 'threshold': 'Threshold (?) [V/Hz]'},
            'Sin(IQ,dIdQ) (?)':                {'direction': 'Direction (peak or dip)','prominence': 'Prominence (peak height) [?]', 'width': 'Width (linewidth) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [?]', 'threshold': 'Threshold (?) [?]'}
        }
        self.peak_finder_params = {}

        # Resonances
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing resonances...")
        splash_progress += 1
        self.resonances = []
        self.active_resonance_index = None
        self.selected_resonance_indexes = []
        self.rectangle_selectors = {}

        # Markers
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Initializing markers...")
        splash_progress += 1
        self.marker_text_dict = {}
        self.max_num_text_labels = 10000
        self.initMarkerTexts()
        self.markers_raw = None
        self.markers_filtered = None
        self.markers_active_raw = None
        self.markers_active_filtered = None
        self.marker_texts_raw = None
        self.marker_texts_filtered = None
        self.marker_texts_active_raw = None
        self.marker_texts_active_filtered = None

        # Build the UI
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Building UI...")
        splash_progress += 1
        self.initUI()

        # Load settings
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress+1}/{splash_steps}: Loading settings...")
        splash_progress += 1
        self.settings = QSettings("mkid_resonance_finder", "ResonanceFinder2")
        self.loadSettings()

        # Get started
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress}/{splash_steps}: Loading data...")
        splash_progress +=1
        
        self.loadFile()

        splash_progress = splash_steps
        splash_screen.show_progress(splash_progress,splash_steps)
        splash_screen.add_log_line(f"{splash_progress}/{splash_steps}: Done.")

        


    def loadSettings(self):
        print('loadSettings')
        self.is_loading_settings = True
        try:
            self.loadSettingsWindowGeometry()
            self.loadSettingsSplitterSizes()
            self.loadSettingsFilename()
            self.loadSettingsActiveFormat()
            self.loadSettingsFilterParameters()
            self.loadSettingsFinderParameters()
            self.refreshUI()
        except Exception as e:
            tb=traceback.format_exc()
            print('Error loading settings:',e)
            print(tb)
        self.is_loading_settings = False

    def saveSettings(self):
        print('saveSettings')
        self.saveSettingsWindowGeometry()
        self.saveSettingsSplitterSizes()
        self.saveSettingsFilename()
        self.saveSettingsActiveFormat()
        self.saveSettingsFilterParameters()
        self.saveSettingsFinderParameters()
        

    def loadSettingsWindowGeometry(self):
        print('loadSettingsWindowGeometry')
        self.settings.beginGroup("MainWindow")
        self.restoreGeometry(self.settings.value("geometry")) if self.settings.value("geometry") is not None else None
        self.restoreState(self.settings.value("windowState")) if self.settings.value("windowState") is not None else None
        self.settings.endGroup()

    def saveSettingsWindowGeometry(self):
        print('saveSettingsWindowGeometry')
        self.settings.beginGroup("MainWindow")
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        self.settings.endGroup()

    def loadSettingsSplitterSizes(self):
        print('loadSettingsSplitterSizes')
        self.settings.beginGroup("MainWindow")
        splitter_sizes = self.settings.value("splitterSizes", defaultValue=[200, 200],type=int)
        self.hsplitter.setSizes(splitter_sizes)
        self.settings.endGroup()

    def saveSettingsSplitterSizes(self):
        print('saveSettingsSplitterSizes')
        self.settings.beginGroup("MainWindow")
        self.settings.setValue("splitterSizes", self.hsplitter.sizes())
        self.settings.endGroup()

    def loadSettingsFilename(self):
        print('loadSettingsFilename')
        self.settings.beginGroup("Analysis")
        filename = self.settings.value("filename", defaultValue=None)
        self.settings.endGroup()
        self.filename = filename
        
    def saveSettingsFilename(self):
        print('saveSettingsFilename')
        self.settings.beginGroup("Analysis")
        self.settings.setValue("filename", self.filename if self.filename is not None else '')
        self.settings.endGroup()

    def loadSettingsActiveFormat(self):
        print('loadSettingsActiveFormat')
        self.settings.beginGroup('Analysis')
        active_format = self.settings.value('active_format', 'Log Magnitude dB')
        self.settings.endGroup()
        self.active_format = active_format

    def saveSettingsActiveFormat(self):
        print('saveSettingsActiveFormat')
        self.settings.beginGroup('Analysis')
        self.settings.setValue('active_format', self.active_format)
        self.settings.endGroup()

    def loadSettingsFilterParameters(self):
        print('loadSettingsFilterParameters')
        self.settings.beginGroup('Filter')
        highpass_edge = self.settings.value('highpass_edge', 0.0, type=float)
        lowpass_edge = self.settings.value('lowpass_edge', 1.0, type=float)
        median_kernel_size = self.settings.value('median_kernel_size', 1, type=int)
        self.settings.endGroup()
        params = {'highpass_edge': highpass_edge, 'lowpass_edge': lowpass_edge, 'median_kernel_size': median_kernel_size}
        self.filterManager.set_filter_params(params)
        
    def saveSettingsFilterParameters(self):
        print('saveSettingsFilterParameters')
        params = self.filterManager.get_filter_params()
        self.settings.beginGroup('Filter')
        self.settings.setValue('highpass_edge', params['highpass_edge'])
        self.settings.setValue('lowpass_edge', params['lowpass_edge'])
        self.settings.setValue('median_kernel_size', params['median_kernel_size'])
        self.settings.endGroup()
    
    def loadSettingsFinderParameters(self):
        print('loadSettingsFinderParameters')
        for format in self.analysis_formats:
            self.settings.beginGroup(f'Finder/{format}')
            params = {
                'prominence_enabled': self.settings.value('prominence_enabled', True, type=bool),
                'prominence_min': self.settings.value('prominence_min', 1.0, type=float),
                'prominence_max': self.settings.value('prominence_max', 100.0, type=float),
                'width_enabled': self.settings.value('width_enabled', True, type=bool),
                'width_min': self.settings.value('width_min', 100.0, type=float),
                'width_max': self.settings.value('width_max', 10000000.0, type=float),
                'threshold_enabled': self.settings.value('threshold_enabled', False, type=bool),
                'threshold_min': self.settings.value('threshold_min', 0.0, type=float),
                'threshold_max': self.settings.value('threshold_max', 1.0, type=float),
                'height_enabled': self.settings.value('height_enabled', False, type=bool),
                'height_min': self.settings.value('height_min', 0.0, type=float),
                'height_max': self.settings.value('height_max', 1.0, type=float),
                'distance_enabled': self.settings.value('distance_enabled', True, type=bool),
                'distance_value': self.settings.value('distance_value', 1000.0, type=float),
                'peak_direction': self.settings.value('peak_direction', 1, type=int),
                'frequency_stepsize': self.settings.value('frequency_stepsize', 1.0, type=float)
            }
            self.settings.endGroup()
            self.peak_finder_params[format] = params
        active_params = self.peak_finder_params[self.active_format]
        self.peakFinderManager.set_finder_parameters(active_params)

    def saveSettingsFinderParameters(self):
        print('saveSettingsFinderParameters')
        for format, params in self.peak_finder_params.items():
            self.settings.beginGroup(f'Finder/{format}')
            for key, value in params.items():
                if key == 'frequency_stepsize':
                    value = float(value)
                self.settings.setValue(key, value)
            self.settings.endGroup()
    

    def loadFile(self,filename=None):
        print('loadFile')
        if not filename:
            filename = self.filename
        if not filename:
            print('No filename provided')
            return
        try:
            print('loading',filename)
            self.frequencies, self.s21_complex = self.dataLoader.load_file(filename)
            self.filename = filename
            self.saveSettingsFilename()

            self.updateDataArrays()
            self.applyFiltering()
            self.updateResonances()
            self.refreshUI()
            self.label_filename.setText(filename)

        except Exception as e:
            self.handleLoadError(e)
            self.filename=None

            

    def postLoadDataSetup(self):
        print('postLoadDataSetup')
    
    def handleLoadError(self, e):
        tb=traceback.format_exc()
        print(f"Error loading file: {e}\n{tb}")
        QMessageBox.warning(None, "Error", f"Error loading file: {e}\n{tb}")

    def updateDataArrays(self):
        print('updateDataArrays')
        self.frequency_stepsize = np.mean(median_filter(np.diff(self.frequencies),3))
        self.magnitude = np.abs(self.s21_complex)
        self.log_magnitude = 20*np.log10(self.magnitude)
        self.phase = np.angle(self.s21_complex)
        self.unwrap_phase = np.unwrap(self.phase)
        self.group_delay_us = -np.gradient(self.unwrap_phase, self.frequencies)*1e6
        self.complex_gradient = np.abs(np.gradient(self.s21_complex, self.frequencies))
        self.sin_di_dq = np.sin(self.phase)*np.gradient(self.magnitude, self.frequencies)


    def applyFiltering(self):
        print('applyFiltering')
        raw_data = self.getDataToFilter() 
        filter_params = self.getFilterParameters()
        self.filterManager.set_filter_params(filter_params)
        self.filtered_data = self.filterManager.filter_data(raw_data)
    
    def getDataToFilter(self):
        print('getDataToFilter')
        analysis_format = self.active_format

        if analysis_format == "Lin Magnitude V":
            data = self.magnitude.copy()
        elif analysis_format == "Log Magnitude dB":
            data = self.log_magnitude.copy()
        elif analysis_format == "Phase rad":
            data = self.phase.copy()
        elif analysis_format == "Unwrapped Phase rad":
            data = self.unwrap_phase.copy()
        elif analysis_format == "Group Delay us (-dphi/df)":
            data = self.group_delay_us.copy()
        elif analysis_format == "Complex Gradient V/Hz (speed)":
            data = self.complex_gradient.copy()
        elif analysis_format == "sin(IQ,didq)":
            data = self.sin_di_dq.copy()
        else:
            print(analysis_format,'?')
            data = self.log_magnitude.copy()
        return data
        
    def getFilterParameters(self):
        print('getFilterParameters')
        return self.filterManager.get_filter_params()
    
    def setFilterParams(self,params):
        print('setFilterParams')
        self.filterManager.set_filter_params(params)
    

    def updateResonances(self):
        print('updateResonances')
        self.performPeakFinding()
        self.updateResonancesList()
        self.updateResonanceNames()


    def performPeakFinding(self):
        print('performPeakFinding')
        # self.getPeakFinderParameters()
        params = self.peak_finder_params[self.active_format]
        print(self.frequency_stepsize,type(self.frequency_stepsize))
        params['frequency_stepsize'] = self.frequency_stepsize
        self.peakFinderManager.set_finder_parameters(params)
        result = self.peakFinderManager.perform_find_peaks(self.filtered_data)
        print(result)
        if type(result) is Exception:
            raise result
        else:
            peaks, peak_properties = result
            self.peaks = peaks
            self.peak_properties = peak_properties

    def updateResonancesList(self):
        print('updateResonancesList')
        # Save old resonances to match with new peaks
        old_resonances = self.resonances.copy()
        self.resonances.clear()
        frequency_tolerance = self.frequency_stepsize
        peak_direction = self.peak_finder_params[self.active_format]['peak_direction']
        
        # Iterate over detected peaks
        for i, peak_idx in enumerate(self.peaks):
            peak_freq = self.frequencies[peak_idx]
            matched_resonance = None

            # Match with existing resonances
            for resonance in old_resonances:
                if abs(resonance.frequency - peak_freq) <= frequency_tolerance:
                    matched_resonance = resonance
                    old_resonances.remove(resonance)
                    break

            if matched_resonance:
                # # Update existing resonance
                matched_resonance.analyse(self.frequencies,self.log_magnitude,self.filtered_data,peak_direction,peak_idx)
                self.resonances.append(matched_resonance)
            else:
                # Create new resonance
                new_resonance = Resonance(id=len(self.resonances))
                new_resonance.analyse(self.frequencies,self.log_magnitude,self.filtered_data,peak_direction,peak_idx)
                self.resonances.append(new_resonance)
        self.updateResonanceIndexes()
        self.updateResonanceNames()
        self.updateActiveResonanceIndex()
        return
    
    # def getResonanceIndex(self,resonance_id):
    #     print('getResonanceIndex')
    #     for i in range(len(self.resonances)):
    #         if self.resonances[i].id == resonance_id:
    #             return i
    #     return None
    
    # def getResonanceID(self,resonance_index):
    #     print('getResonanceID')
    #     return self.resonances[resonance_index].id

    def updateResonanceIndexes(self):
        print('updateResonanceIndexes')
        for i in range(len(self.resonances)):
            self.resonances[i].set_id(i)


    def updateResonanceNames(self):
        n=0
        print('updateResonanceNames')
        for i in range(len(self.resonances)):
            if self.resonances[i].save:
                self.resonances[i].set_name('%04d'%n)
                n+=1
            else:
                self.resonances[i].set_name('not_saved')
        return
            
    def updateActiveResonanceIndex(self):
        print('updateActiveResonanceIndex')
        for i in range(len(self.resonances)):
            if self.resonances[i].is_active:
                self.active_resonance_index = i
                return
        self.active_resonance_index = None

    def setActiveResonanceIndex(self,resonance_index):
        print('setActiveResonanceIndex')
        for i in range(len(self.resonances)):
            self.resonances[i].set_active_state(i == resonance_index)
        self.active_resonance_index = resonance_index

    def updateResonanceSelection(self):
        print('updateResonanceSelection')
        self.selected_resonance_indexes = []
        for i in range(len(self.resonances)):
            if self.resonances[i].is_selected:
                self.selected_resonance_indexes.append(i)
        

    def addResonance(self, frequency_mhz=None, refreshUI=True):
        print('addResonance',frequency_mhz)
        if frequency_mhz is None:
            if self.active_resonance_index is not None:
                default_value = self.resonances[self.active_resonance_index].frequency / 1e6
            else:
                value = 0
            frequency_mhz, ok = QInputDialog.getDouble(self, "Add Resonance","Enter new frequency (MHz):",
                                                        value=default_value, decimals=6)
            if not ok:
                print('No frequency entered')
                return
        frequency = frequency_mhz * 1e6
        # Find the closest peak index
        peak_idx = np.abs(self.frequencies - frequency).argmin()
        # Create and analyze the new resonance
        new_resonance = Resonance()
        new_resonance.analyse(self.frequencies,
                                self.log_magnitude,
                                self.filtered_data,
                                self.peak_finder_params[self.active_format]['peak_direction'],
                                peak_idx)
        self.resonances.append(new_resonance)
        self.resonances.sort(key=lambda r: r.frequency)
        self.updateResonanceIndexes()
        self.updateResonanceNames()
        self.updateActiveResonanceIndex()
        self.updateResonanceSelection()
        if refreshUI:
            self.refreshUI()      


    def editResonance(self, resonance_index, new_frequency_mhz=None,refreshUI=True):
        print('editResonance')
        if (resonance_index <0) or (resonance_index>=len(self.resonances)):
            print(f'invalid index,{resonance_index}/{len(self.resonances)}')
            return
        resonance = self.resonances[resonance_index]

        new_frequency_mhz, ok = QInputDialog.getDouble(
            self, "Edit Resonance", "Enter new frequency (MHz):",
            value=resonance.frequency / 1e6, decimals=6
        )
        if ok:
            resonance.frequency = new_frequency_mhz * 1e6
            peak_idx = np.abs(self.frequencies - resonance.frequency).argmin()
            # Re-analyse with the new frequency and peak_idx
            resonance.analyse(self.frequencies,
                              self.log_magnitude,
                              self.filtered_data,
                              self.peak_finder_params[self.active_format]['peak_direction'],
                              peak_idx)
            # Save a reference to the edited resonance
            edited_resonance = resonance
            # Sort the resonance list
            self.resonances.sort(key=lambda r: r.frequency)
            # Find the new index of the edited resonance
            self.updateResonanceIndexes()
            self.updateResonanceNames()
            self.updateActiveResonanceIndex()
            self.updateResonanceSelection()
            if refreshUI:
                self.refreshUI()       


    def deleteResonance(self,resonance_index,refreshUI=True):
        print('deleteResonance')
        for i in range(len(self.resonances)):
            if self.resonances[i].id == resonance_index:
                self.resonances.pop(i)
                break
        self.updateResonanceIndexes()
        self.updateResonanceNames()
        self.updateActiveResonanceIndex()
        self.updateResonanceSelection()

        self.resonances_table.clearSelection()
        self.resonances_table.setCurrentCell(-1, -1)


        if refreshUI:
            self.refreshUI()

    def deleteResonances(self,resonance_indexes,refreshUI=True):
        print('deleteResonances')
        for i in range(len(resonance_indexes)):
            for j in range(len(self.resonances)):
                if self.resonances[j].id == resonance_indexes[i]:
                    self.resonances.pop(j)
                    break
        self.updateResonanceIndexes()
        self.updateResonanceNames()
        self.updateActiveResonanceIndex()
        self.updateResonanceSelection()
        self.resonances_table.clearSelection()
        self.resonances_table.setCurrentCell(-1, -1)

        if refreshUI:
            self.refreshUI()


    def deleteSelectedResonances(self,refreshUI=True):
        print('deleteSelectedResonances')
        # dialog to warn about deletion...
        if not self.selected_resonance_indexes:
            if self.active_resonance_index is None:
                QMessageBox.warning(self, "Delete Resonances", "No resonances selected for deletion.")
                return
            answer = QMessageBox.question(self, "Delete Resonance", "Are you sure you want to delete the current resonance?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer == QMessageBox.Yes:
                self.deleteResonance(self.active_resonance_index,refreshUI=False)
        else:        
            answer = QMessageBox.question(self, "Delete Resonances", "Are you sure you want to delete the selected resonances?", QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if answer == QMessageBox.Yes:
                self.deleteResonances(self.selected_resonance_indexes,refreshUI=False)
        
        if refreshUI:
            self.refreshUI()



    def refreshUI(self):
        print('refreshUI')
        self.refreshControls()
        self.refreshPlots()
        self.refreshResonancesTable()
        self.refreshMarkers()
        self.refreshFigures()
        self.refreshActiveResonance()
        self.updateNavigationButtons()
        self.refreshSelectedResonances()
        self.refreshPeakFinderLabels()
        self.refreshActiveResonanceLabel()
        self.refreshResonancesLabel()


    def refreshControls(self):
        print('refreshControls')
        self.setUIAnalysisFormat()
        # self.setUIFilterParameters() # called by setUIAnalysisFormat
        # self.setUIPeakFinderParameters() # called by setUIAnalysisFormat

    def refreshPlots(self):
        print('updatePlots')
        self.plotRaw()
        self.plotFiltered()
        self.plotActiveRaw()
        self.plotActiveFiltered()

    def plotRaw(self):
        print('plotRaw')
        if self.fig_raw is None:
            self.ax_raw.set_xlabel("Frequency (MHz)")
            self.ax_raw.set_ylabel("Magnitude (dB)")
        else:
            self.line_raw.set_data(self.frequencies/1e6, self.log_magnitude)
        self.ax_raw.relim()
        self.ax_raw.autoscale_view()
        self.ax_raw.margins(0.02,0.2)
        self.canvas_raw.draw_idle()

    def plotFiltered(self):
        print('plotFiltered')
        if self.fig_filtered is None:
            self.ax_filtered.set_xlabel("Frequency (MHz)")
            self.ax_filtered.set_ylabel(self.active_format)
        else:
            self.line_filtered.set_data(self.frequencies/1e6, self.filtered_data)
            self.ax_filtered.set_ylabel(self.active_format)
        self.ax_filtered.relim()
        self.ax_filtered.autoscale_view()
        self.ax_filtered.margins(0.02,0.2)
        self.canvas_filtered.draw_idle()

    def plotActiveRaw(self):
        print('plotActiveRaw')
        if self.fig_active_raw is None:
            self.ax_active_raw.set_xlabel("Frequency (MHz)")
            self.ax_active_raw.set_ylabel("Magnitude (dB)")
        else:
            self.line_active_raw.set_data(self.frequencies/1e6, self.log_magnitude)
        self.ax_active_raw.relim()
        self.ax_active_raw.autoscale_view()
        self.ax_active_raw.margins(0.02,0.2)
        self.canvas_active_raw.draw_idle()
    
    def plotActiveFiltered(self):
        print('plotActiveFiltered')
        if self.fig_active_filtered is None:
            self.ax_active_filtered.set_xlabel("Frequency (MHz)")
            self.ax_active_filtered.set_ylabel(self.active_format)
        else:
            self.line_active_filtered.set_data(self.frequencies/1e6, self.filtered_data)
            self.ax_active_filtered.set_ylabel(self.active_format)
        self.ax_active_filtered.relim()
        self.ax_active_filtered.autoscale_view()
        self.ax_active_filtered.margins(0.02,0.2)
        self.canvas_active_filtered.draw_idle()

    def refreshFigures(self):
        print('refreshFigures')
        for ax in [self.ax_raw, self.ax_filtered, self.ax_active_raw, self.ax_active_filtered]:
            if ax is not None:
                ax.relim()
                ax.autoscale_view()
                ax.margins(0.02,0.2)
        for figure in [self.fig_raw, self.fig_filtered, self.fig_active_raw, self.fig_active_filtered]:
            if figure is not None:
                figure.tight_layout()
                figure.canvas.draw_idle()



    def refreshResonancesTable(self):
        print('updateResonancesTable')
        self.resonances_table.setRowCount(len(self.resonances))
        for i, resonance in enumerate(self.resonances):
            self.updateResonancesTableRow(i, resonance)
        #scroll to active resonance
        if self.active_resonance_index is not None:
            self.resonances_table.scrollToItem(self.resonances_table.item(self.active_resonance_index, 1))
        if (self.active_resonance_index is not None and 0 <= self.active_resonance_index < len(self.resonances)):
            self.resonances_table.scrollToItem(self.resonances_table.item(self.active_resonance_index, 1))

    def updateResonancesTableRow(self, row, resonance):
        print('updateResonancesTableRow',row)
        
        # Save checkbox
        save_checkbox = self.resonances_table.cellWidget(row, 0)
        if save_checkbox is None:
            save_checkbox = QCheckBox()
            save_checkbox.stateChanged.connect(self.onSaveCheckboxChanged)
            save_checkbox.setProperty('resonance_id', resonance.id)
            self.resonances_table.setCellWidget(row, 0, save_checkbox)
        else:
            if save_checkbox.property('resonance_id') != resonance.id:
                save_checkbox.setProperty('resonance_id', resonance.id)
        save_checkbox.blockSignals(True)
        save_checkbox.setChecked(resonance.save)
        save_checkbox.blockSignals(False)

        # name
        name_item = self.resonances_table.item(row, 1)
        if name_item is None:
            name_item = QTableWidgetItem()
            name_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 1, name_item)
        name_item.setText(resonance.name)
        
        # Frequency
        freq_item = self.resonances_table.item(row, 2)
        if freq_item is None:
            freq_item = QTableWidgetItem()
            freq_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 2, freq_item)
        freq_item.setText(f"{resonance.frequency / 1e6:.6f}")

        # FWHM
        fwhm_item = self.resonances_table.item(row, 3)
        if fwhm_item is None:
            fwhm_item = QTableWidgetItem()
            fwhm_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 3, fwhm_item)
        fwhm_value = resonance.fwhm
        fwhm_item.setText(f"{fwhm_value / 1e3:.3f}" if fwhm_value is not None else "N/A")

        # Depth
        depth_item = self.resonances_table.item(row, 4)
        if depth_item is None:
            depth_item = QTableWidgetItem()
            depth_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 4, depth_item)
        depth_value = resonance.dip_depth
        depth_item.setText(f"{depth_value:.3f}" if depth_value is not None else "N/A")

        # Q-factor
        q_item = self.resonances_table.item(row, 5)
        if q_item is None:
            q_item = QTableWidgetItem()
            q_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 5, q_item)
        q_value = resonance.q_factor
        q_item.setText(f"{q_value:.1f}" if q_value is not None else "N/A")


        # Qc
        qc_item = self.resonances_table.item(row, 6)
        if qc_item is None:
            qc_item = QTableWidgetItem()
            qc_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 6, qc_item)
        qc_value = resonance.qc
        qc_item.setText(f"{qc_value:.1f}" if qc_value is not None else "N/A")

        # Qi
        qi_item = self.resonances_table.item(row, 7)
        if qi_item is None:
            qi_item = QTableWidgetItem()
            qi_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
            self.resonances_table.setItem(row, 7, qi_item)
        qi_value = resonance.qi
        qi_item.setText(f"{qi_value:.1f}" if qi_value is not None else "N/A")
        

    def onSaveCheckboxChanged(self,state):
        print('onSaveCheckboxChanged')
        checkbox = self.sender()
        id = checkbox.property('resonance_id')
        for resonance in self.resonances:
            if resonance.id == id:
                resonance.set_save_state(state==Qt.Checked)
                self.refreshResonancesTable()       # Because the table changed
                self.refreshMarkers()               # Because marker color may have changed
                self.refreshResonancesLabel()       # Because # saved might have changed
                self.refreshActiveResonanceLabel()  # If the user changed the “active” one
                break


    def initMarkerTexts0(self):
        # slow as hell for 10000 markers
        print('initMarkerTexts')
        print('initMarkerTexts: mtexts')
        mtexts = ["ignored"] + ["%04d"%i for i in range(self.max_num_text_labels)] 
        print('initMarkerTexts: mobjs')
        mobjs = [matplotlib.markers.MarkerStyle("$"+t+"$") for t in mtexts]
        print('initMarkerTexts: mpaths')
        mpaths = [mobj.get_path() for mobj in mobjs]
        rotation = matplotlib.transforms.Affine2D().rotate_deg(90)
        translation = matplotlib.transforms.Affine2D().translate(0, -1.0)
        print('initMarkerTexts: transforms')
        transforms = [mobj.get_transform() + rotation + translation for mobj in mobjs]
        print('initMarkerTexts: transformedmpaths')
        transformedmpaths = [mpath.transformed(transform) for mpath, transform in zip(mpaths, transforms)]
        print('initMarkerTexts: marker_text_dict')
        self.marker_text_dict = dict(zip(mtexts,transformedmpaths))


    def initMarkerTexts(self):
        print('initMarkerTexts')

        # 1) Build the list of marker labels:
        print('initMarkerTexts: mtexts')
        mtexts = ["ignored"] + [f"{i:04d}" for i in range(self.max_num_text_labels)]

        # 2) Create/cache digit paths & widths once (0–9):
        print('initMarkerTexts: digit_paths')
        FONT_PROP = matplotlib.font_manager.FontProperties(size=10,weight='ultralight')
        digit_verts = {}
        digit_codes = {}
        digit_widths = {}

        # for d in "0123456789":
        for d in [chr(i) for i in range(128)]:
            # Create a path for this single character (NOT LaTeX).
            dp = TextPath((0, 0), d, prop=FONT_PROP)
            # Cache its vertices, codes, and bounding-box width
            digit_verts[d] = dp.vertices  # shape (N,2)
            digit_codes[d] = dp.codes    # shape (N,)
            digit_widths[d] = dp.get_extents().width

        def _make_path_from_digits(label):
            """
            Build a Path by manually concatenating digit_verts for each digit in `label`.
            If `label` is 'ignored', return a simple empty path.
            """
            # if label == "ignored":
            #     # Return a simple empty path (or something minimal).
            #     return Path(np.array([[0, 0]]), [Path.MOVETO])

            # Each label is like "0000", "0001", etc.
            x_offset = 0.0
            verts_list = []
            codes_list = []

            for char in label:
                # Copy the base vertices so we don't mutate the cached version
                v = digit_verts[char].copy()
                # Shift x-coordinates by x_offset
                v[:, 0] += x_offset

                # Append them to our path build
                verts_list.append(v)
                codes_list.append(digit_codes[char])

                # Advance offset by the width of this digit
                x_offset += digit_widths[char]*1.2

            # Combine into one path
            all_verts = np.concatenate(verts_list)
            all_codes = np.concatenate(codes_list)
            all_verts[:,0] = all_verts[:,0] - np.mean(all_verts[:,0])
            all_verts[:,1] = all_verts[:,1] - np.mean(all_verts[:,1])

            return Path(all_verts, all_codes)
        # 3) Build the final transform (rotation + translation):
        #    same as in your original code.
        print('initMarkerTexts: transforms')
        rotation = Affine2D().rotate_deg(90)
        translation = Affine2D().translate(0,-20)
        final_transform = rotation + translation

        # 4) Create each label path, then apply the final transform
        print('initMarkerTexts: transformedmpaths')
        marker_text_dict = {}
        for t in mtexts:
            base_path = _make_path_from_digits(t)
            # Apply your original final transform
            marker_text_dict[t] = base_path.transformed(final_transform)

        # 5) Store in self
        print('initMarkerTexts: marker_text_dict')
        self.marker_text_dict = marker_text_dict

    def addMarkerText(self,mtext):
        print('addMarkerText')
        if mtext not in self.marker_text_dict:
            FONT_PROP = matplotlib.font_manager.FontProperties(size=10,weight='ultralight')
            digit_verts = {}
            digit_codes = {}
            digit_widths = {}

            # for d in "0123456789":
            for d in mtext:
                # Create a path for this single character (NOT LaTeX).
                dp = TextPath((0, 0), d, prop=FONT_PROP)
                # Cache its vertices, codes, and bounding-box width
                digit_verts[d] = dp.vertices  # shape (N,2)
                digit_codes[d] = dp.codes    # shape (N,)
                digit_widths[d] = dp.get_extents().width

            def _make_path_from_digits(label):
                """
                Build a Path by manually concatenating digit_verts for each digit in `label`.
                If `label` is 'ignored', return a simple empty path.
                """
                # Each label is like "0000", "0001", etc.
                x_offset = 0.0
                verts_list = []
                codes_list = []

                for char in label:
                    # Copy the base vertices so we don't mutate the cached version
                    v = digit_verts[char].copy()
                    # Shift x-coordinates by x_offset
                    v[:, 0] += x_offset

                    # Append them to our path build
                    verts_list.append(v)
                    codes_list.append(digit_codes[char])

                    # Advance offset by the width of this digit
                    x_offset += digit_widths[char]*1.2

                # Combine into one path
                all_verts = np.concatenate(verts_list)
                all_codes = np.concatenate(codes_list)
                all_verts[:,0] = all_verts[:,0] - np.mean(all_verts[:,0])
                all_verts[:,1] = all_verts[:,1] - np.mean(all_verts[:,1])

                return Path(all_verts, all_codes)
            
            print('initMarkerTexts: transforms')
            rotation = Affine2D().rotate_deg(90)
            translation = Affine2D().translate(0,-20)
            final_transform = rotation + translation

            print('initMarkerTexts: marker_text_dict')
            base_path = _make_path_from_digits(mtext)
            self.marker_text_dict[mtext] = base_path.transformed(final_transform)



    def refreshMarkers(self):
        print('refreshMarkers')
        marker_freqs = np.array([resonance.frequency for resonance in self.resonances],dtype=float)
        marker_mags = np.array([resonance.marker_mag for resonance in self.resonances],dtype=float)
        marker_filts = np.array([resonance.marker_filt for resonance in self.resonances],dtype=float)
        marker_active = np.array([resonance.is_active for resonance in self.resonances],dtype=bool)
        marker_select = np.array([resonance.is_selected for resonance in self.resonances],dtype=bool)
        marker_save = np.array([resonance.save for resonance in self.resonances],dtype=bool)

        freq_active = marker_freqs[marker_active]
        mag_active = marker_mags[marker_active]
        filt_active = marker_filts[marker_active]
        freq_selected = marker_freqs[marker_select]
        mag_selected = marker_mags[marker_select]
        filt_selected = marker_filts[marker_select]
        freq_saved = marker_freqs[marker_save]
        mag_saved = marker_mags[marker_save]
        filt_saved = marker_filts[marker_save]
        
        pickradius = 5

        if not hasattr(self,'marker_line_raw'):

            #selected marker
            self.marker_line_raw_selected, = self.ax_raw.plot(freq_selected/1e6, mag_selected, 'o', ms=12, color='cyan')
            self.marker_line_filtered_selected, = self.ax_filtered.plot(freq_selected/1e6, filt_selected, 'o', ms=12, color='cyan')
            self.marker_line_active_raw_selected, = self.ax_active_raw.plot(freq_selected/1e6, mag_selected, 'o', ms=12, color='cyan')
            self.marker_line_active_filtered_selected, = self.ax_active_filtered.plot(freq_selected/1e6, filt_selected, 'o', ms=12, color='cyan')

            #all markers
            self.marker_line_raw, = self.ax_raw.plot(marker_freqs/1e6, marker_mags, '.', ms=12, color='k', picker=True, pickradius=pickradius)
            self.marker_line_filtered, = self.ax_filtered.plot(marker_freqs/1e6, marker_filts, '.', ms=12, color='k', picker=True, pickradius=pickradius)
            self.marker_line_active_raw, = self.ax_active_raw.plot(marker_freqs/1e6, marker_mags, '.', ms=12, color='k', picker=True, pickradius=pickradius)
            self.marker_line_active_filtered, = self.ax_active_filtered.plot(marker_freqs/1e6, marker_filts, '.', ms=12, color='k', picker=True, pickradius=pickradius)

            #active marker
            self.marker_line_raw_active, = self.ax_raw.plot(freq_active/1e6, mag_active, 'o', ms=9, color='red')
            self.marker_line_filtered_active, = self.ax_filtered.plot(freq_active/1e6, filt_active, 'o', ms=9, color='red')
            self.marker_line_active_raw_active, = self.ax_active_raw.plot(freq_active/1e6, mag_active, 'o', ms=9, color='red')
            self.marker_line_active_filtered_active, = self.ax_active_filtered.plot(freq_active/1e6, filt_active, 'o', ms=9, color='red')

            #saved marker
            self.marker_line_raw_saved, = self.ax_raw.plot(freq_saved/1e6, mag_saved, 'o', ms=6, color='g')
            self.marker_line_filtered_saved, = self.ax_filtered.plot(freq_saved/1e6, filt_saved, 'o', ms=6, color='g')
            self.marker_line_active_raw_saved, = self.ax_active_raw.plot(freq_saved/1e6, mag_saved, 'o', ms=6, color='g')
            self.marker_line_active_filtered_saved, = self.ax_active_filtered.plot(freq_saved/1e6, filt_saved, 'o', ms=6, color='g')


        else:
            #all markers
            self.marker_line_raw.set_data(marker_freqs/1e6, marker_mags)
            self.marker_line_filtered.set_data(marker_freqs/1e6, marker_filts)
            self.marker_line_active_raw.set_data(marker_freqs/1e6, marker_mags)
            self.marker_line_active_filtered.set_data(marker_freqs/1e6, marker_filts)

            #active marker
            self.marker_line_raw_active.set_data(freq_active/1e6, mag_active)
            self.marker_line_filtered_active.set_data(freq_active/1e6, filt_active)
            self.marker_line_active_raw_active.set_data(freq_active/1e6, mag_active)
            self.marker_line_active_filtered_active.set_data(freq_active/1e6, filt_active)

            #saved marker
            self.marker_line_raw_saved.set_data(freq_saved/1e6, mag_saved)
            self.marker_line_filtered_saved.set_data(freq_saved/1e6, filt_saved)
            self.marker_line_active_raw_saved.set_data(freq_saved/1e6, mag_saved)
            self.marker_line_active_filtered_saved.set_data(freq_saved/1e6, filt_saved)

            #selected marker
            self.marker_line_raw_selected.set_data(freq_selected/1e6, mag_selected)
            self.marker_line_filtered_selected.set_data(freq_selected/1e6, filt_selected)
            self.marker_line_active_raw_selected.set_data(freq_selected/1e6, mag_selected)
            self.marker_line_active_filtered_selected.set_data(freq_selected/1e6, filt_selected)

        marker_text = [resonance.name for resonance in self.resonances]
        for i in range(len(marker_text)):
            if marker_text[i] not in self.marker_text_dict:
                self.addMarkerText(marker_text[i])

        if hasattr(self,'marker_texts_raw'):
            if self.marker_texts_raw is not None:
                print(self.marker_texts_raw)
                self.marker_texts_raw.remove()
                self.marker_texts_raw=None
                self.marker_texts_filtered.remove()
                self.marker_texts_filtered=None
                self.marker_texts_active_raw.remove()
                self.marker_texts_active_raw=None
                self.marker_texts_active_filtered.remove()
                self.marker_texts_active_filtered=None


        self.marker_texts_raw = self.ax_raw.scatter(marker_freqs/1e6, marker_mags, c='k',s=1)
        self.marker_texts_raw.set_paths([self.marker_text_dict[t] for t in marker_text])

        self.marker_texts_filtered = self.ax_filtered.scatter(marker_freqs/1e6, marker_filts, c='k',s=1)
        self.marker_texts_filtered.set_paths([self.marker_text_dict[t] for t in marker_text])

        self.marker_texts_active_raw = self.ax_active_raw.scatter(marker_freqs/1e6, marker_mags, c='k',s=1)
        self.marker_texts_active_raw.set_paths([self.marker_text_dict[t] for t in marker_text])

        self.marker_texts_active_filtered = self.ax_active_filtered.scatter(marker_freqs/1e6, marker_filts, c='k',s=1)
        self.marker_texts_active_filtered.set_paths([self.marker_text_dict[t] for t in marker_text])

        self.canvas_raw.draw_idle()
        self.canvas_filtered.draw_idle()
        self.canvas_active_raw.draw_idle()
        self.canvas_active_filtered.draw_idle()


    def refreshActiveResonance(self):
        print('refreshActiveResonance')
        self.refreshActiveResonanceIndex()
        self.refreshActiveResonancePlots()
        self.refreshActiveResonanceTable()
        self.refreshActiveResonanceMarkers()
        self.updateNavigationButtons()
    
    def refreshActiveResonanceIndex(self):
        print('refreshActiveResonanceIndex')
        for i in range(len(self.resonances)):
            if self.resonances[i].is_active:
                self.active_resonance_index = i
                return
        self.active_resonance_index = None
    
    def setActiveResonanceIndex(self,index):
        print('setActiveResonanceIndex',index)
        available_ids = [resonance.id for resonance in self.resonances]
        if index not in available_ids:
            index = None
        for i in range(len(self.resonances)):
            resonance = self.resonances[i]
            resonance.set_active_state(resonance.id == index)
        self.active_resonance_index = index
        print('active resonance index:',self.active_resonance_index)
        self.refreshActiveResonance()
    

    def nextActiveResonance(self):
        print('nextActiveResonance')
        if not self.resonances:
            return
        num_resonances = len(self.resonances)
        if self.active_resonance_index is None:
            self.setActiveResonanceIndex(0)
            return
        next_idx =self.active_resonance_index + 1
        if next_idx == num_resonances:
            next_idx = 0

        self.setActiveResonanceIndex(next_idx)
    
    def prevActiveResonance(self):
        print('prevActiveResonance')
        if not self.resonances:
            return
        num_resonances = len(self.resonances)
        if self.active_resonance_index is None:
            self.setActiveResonanceIndex(num_resonances-1)
            return
        prev_idx = self.active_resonance_index - 1
        if prev_idx < 0:
            prev_idx = num_resonances - 1

        self.setActiveResonanceIndex(prev_idx)

    def refreshActiveResonancePlots(self):
        print('refreshActiveResonancePlots')
        if self.active_resonance_index is not None:
            resonance_index = self.active_resonance_index
            try:
                resonance = self.resonances[resonance_index]

            except:
                print('resonance not found')
                return

            f = resonance.frequency
            fwhm = resonance.fwhm
            if fwhm is None or fwhm <= 0:
                fwhm = self.frequencies[1]-self.frequencies[0]  # Use default value to prevent zero span
            freq_span = 10 * fwhm
            margin = [0.1, 0.1, 0.1, 0.3] # L,R,T,B
            freq_min = f - freq_span / 2
            freq_max = f + freq_span / 2
            freq_view_min = freq_min - margin[0] * (freq_max - freq_min)
            freq_view_max = freq_max + margin[1] * (freq_max - freq_min)

            mask = (self.frequencies >= freq_view_min) & (self.frequencies <= freq_view_max)
            # Handle empty mask
            if not np.any(mask):
                mask = slice(None)

            raw_min = np.min(self.log_magnitude[mask])
            raw_max = np.max(self.log_magnitude[mask])
            raw_view_min = raw_min - margin[3] * (raw_max - raw_min)
            raw_view_max = raw_max + margin[2] * (raw_max - raw_min)

            filt_min = np.min(self.filtered_data[mask])
            filt_max = np.max(self.filtered_data[mask])
            filt_view_min = filt_min - margin[3] * (filt_max - filt_min)
            filt_view_max = filt_max + margin[2] * (filt_max - filt_min)

            self.ax_active_raw.set_xlim(freq_view_min/1e6, freq_view_max/1e6)
            self.ax_active_raw.set_ylim(raw_view_min, raw_view_max)
            self.ax_active_filtered.set_xlim(freq_view_min/1e6, freq_view_max/1e6)
            self.ax_active_filtered.set_ylim(filt_view_min, filt_view_max)

            self.canvas_active_raw.draw_idle()
            self.canvas_active_filtered.draw_idle()
        else:
            self.ax_active_raw.relim()
            self.ax_active_raw.autoscale_view()
            self.ax_active_raw.margins(0.02,0.2)
            self.ax_active_filtered.relim()
            self.ax_active_filtered.autoscale_view()
            self.ax_active_filtered.margins(0.02,0.2)
        
            self.canvas_active_raw.draw_idle()
            self.canvas_active_filtered.draw_idle()

    
    def refreshActiveResonanceTable(self):
        print('refreshActiveResonanceTable')

        #make the active resonance row bold, scroll to row, and if not multiple selection, select the row
        active_row = self.active_resonance_index
        
        if active_row is not None:
            if len(self.selected_resonance_indexes)==0:
                # self.resonances_table.selectRow(active_row)
                self.resonances_table.scrollToItem(self.resonances_table.item(active_row,0))

        for row in range(self.resonances_table.rowCount()):
            for col in range(self.resonances_table.columnCount()):
                item = self.resonances_table.item(row, col)
                if item is not None:
                    if hasattr(item, 'font'):    
                        font = item.font()
                        font.setBold(row == active_row)
                        item.setFont(font)



    def refreshActiveResonanceMarkers(self):
        print('refreshActiveResonanceMarkers')
        self.refreshMarkers()
    

    def refreshSelectedResonances(self):
        print('refreshSelectedResonances')
        self.getSelectedResonances()
        self.refreshSelectedResonancesTable()
        self.refreshSelectedResonancesMarkers()
        self.updateNavigationButtons()
        self.refreshResonancesLabel()


    def getSelectedResonances(self):
        print('getSelectedResonances')
        selected_resonance_indexes = []
        for i in range(len(self.resonances)):
            resonance = self.resonances[i]
            if resonance.is_selected:
                selected_resonance_indexes.append(i)
        self.selected_resonance_indexes = selected_resonance_indexes

    def setSelectedResonances(self,resonance_indexes):
        print('setSelectedResonances')
        for i in range(len(self.resonances)):
            resonance = self.resonances[i]
            resonance.set_selected_state(i in resonance_indexes)
        self.selected_resonance_indexes = resonance_indexes
        self.refreshSelectedResonances()

    
    def refreshSelectedResonancesTable(self):
        print('refreshSelectedResonancesTable')
        selected_rows = self.selected_resonance_indexes
        selection_model = self.resonances_table.selectionModel()
        selection = QItemSelection()
        for row in selected_rows:
            index = self.resonances_table.model().index(row, 0)
            selection.select(index, index)
        # Apply the new selection
        selection_model.select(selection, QItemSelectionModel.ClearAndSelect | QItemSelectionModel.Rows)

    def refreshSelectedResonancesMarkers(self):
        print('refreshSelectedResonancesMarkers')
        self.refreshMarkers()
    
    def deselectAllResonances(self):
        print('deselectAllResonances')
        for i in range(len(self.resonances)):
            resonance = self.resonances[i]
            resonance.set_selected_state(False)
        self.refreshSelectedResonances()

    
    def toggleSaveSelectedResonances(self):
        print('toggleSaveSelectedResonances')
        selected = [r.is_selected for r in self.resonances]
        if not(any(selected)):
            [resonance.toggle_save_state() for resonance in self.resonances if resonance.is_active]
        else:
            [resonance.toggle_save_state() for resonance in self.resonances if resonance.is_selected] 
        self.refreshResonancesTable()
        self.refreshMarkers()
        self.refreshResonancesLabel()
        self.refreshActiveResonanceLabel()
        self.updateNavigationButtons()

    def toggleSaveAllResonances(self):
        print('toggleSaveAll')
        save = [resonance.save for resonance in self.resonances]
        if not(any(save)):
            [resonance.set_save_state(True) for resonance in self.resonances]
            self.refreshResonancesTable()
            self.refreshMarkers()
            self.refreshResonancesLabel()
            self.refreshActiveResonanceLabel()
            self.updateNavigationButtons()
            return
        if all(save):
            [resonance.set_save_state(False) for resonance in self.resonances]
            self.refreshResonancesTable()
            self.refreshMarkers()
            self.refreshResonancesLabel()
            self.refreshActiveResonanceLabel()
            self.updateNavigationButtons()
            return
        elif any(save):
            [resonance.set_save_state(True) for resonance in self.resonances]
            self.refreshResonancesTable()
            self.refreshMarkers()
            self.refreshResonancesLabel()
            self.refreshActiveResonanceLabel()
            self.updateNavigationButtons()
            return


    def updateNavigationButtons(self):
        print('updateNavigationButtons')
        has_resonances = len(self.resonances)!=0
        has_active = self.active_resonance_index is not None
        has_selection = bool(self.selected_resonance_indexes)
        self.button_previous_resonance.setEnabled(has_resonances)
        self.button_next_resonance.setEnabled(has_resonances)
        self.button_edit_resonance.setEnabled(has_active)
        self.button_remove_resonance.setEnabled(has_active or has_selection)
        self.button_toggle_save.setEnabled(has_active or has_selection)
        self.button_toggle_save_all.setEnabled(has_resonances)

    def refreshPeakFinderLabels(self):
        print('updatePeakFinderLabels')
        labels = self.peak_finder_labels.get(self.active_format, None)
        if labels:
            self.label_prominence.setText(labels.get('prominence', 'Prominence'))
            self.label_width.setText(labels.get('width', 'Width'))
            self.label_distance.setText(labels.get('distance', 'Distance'))
            self.label_height.setText(labels.get('height', 'Height'))
            self.label_threshold.setText(labels.get('threshold', 'Threshold'))
            self.label_peak_direction.setText(labels.get('peak_direction', 'Peak Direction'))
        else:
            self.label_prominence.setText("Prominence")
            self.label_width.setText("Width")
            self.label_distance.setText("Distance")
            self.label_height.setText("Height")
            self.label_threshold.setText("Threshold")
            self.label_peak_direction.setText("Peak Direction")


    def refreshResonancesLabel(self):
        print('refreshResonancesLabel')
        num_resonances = len(self.resonances)
        num_saved_resonances = sum([1 for resonance in self.resonances if resonance.save])
        self.label_num_resonances.setText(
            f"Resonances found = {num_resonances}. Resonances to save = {num_saved_resonances}"
    )

    def refreshActiveResonanceLabel(self):
        print('refreshActiveResonanceLabel')
        if self.active_resonance_index is not None:
            resonance = self.resonances[self.active_resonance_index]
            self.label_analysis.setText(f'Active Resonance: {resonance.name}\nFrequency: {resonance.frequency/1e6:.6f} MHz\nDip depth: {resonance.dip_depth:.3f} dB\nFWHM: {resonance.fwhm:.3f} Hz\nQr: {resonance.q_factor:.3f}\nQc: {resonance.qc:.3f}\nQi: {resonance.qi:.3f}\nSave: {resonance.save}')

        else:
            if len(self.resonances) > 0:
                self.label_analysis.setText("No active resonance\n\nPick a resonance to view")
            else:
                self.label_analysis.setText("No resonances found\n\nTry adjusting parameters")

    def closeEvent(self, event):
        print('closeEvent')
        self.saveSettings()
        super().closeEvent(event)  # Ensure the base class method is called

    def initUI(self):
        print('initUI')
        self.initUILayout()
        self.initUIActions()
        self.initUIControls()
        self.initUIConnections()
        

    def initUIActions(self):
        print('initUIActions')
        self.toggle_save_action = QAction("Toggle Save", self)
        self.toggle_save_action.setShortcut(Qt.Key_Space)
        self.toggle_save_action.triggered.connect(self.toggleSaveSelectedResonances)

        self.toggle_save_all_action = QAction("Toggle Save All", self)
        self.toggle_save_all_action.setShortcut(Qt.CTRL + Qt.Key_Space)
        self.toggle_save_all_action.triggered.connect(self.toggleSaveAllResonances)


        self.deselect_action = QAction("Deselect All", self)
        self.deselect_action.setShortcut(Qt.Key_Escape)
        self.deselect_action.triggered.connect(self.deselectAllResonances)

        self.next_active_resonance_action = QAction("Next Active Resonance", self)
        self.next_active_resonance_action.setShortcut(Qt.Key_Right)
        self.next_active_resonance_action.triggered.connect(self.onNextResonance)

        self.prev_active_resonance_action = QAction("Previous Active Resonance", self)
        self.prev_active_resonance_action.setShortcut(Qt.Key_Left)
        self.prev_active_resonance_action.triggered.connect(self.onPreviousResonance)

        self.delete_action = QAction("Delete Resonance(s)", self)
        self.delete_action.setShortcut(Qt.Key_Delete)
        self.delete_action.triggered.connect(self.onDeleteResonance)

        self.add_action = QAction("Add Resonance", self)
        self.add_action.triggered.connect(self.onAddResonance)

        self.edit_action = QAction("Edit Resonance", self)
        self.edit_action.triggered.connect(self.onEditResonance)



        self.addAction(self.toggle_save_action)
        self.addAction(self.toggle_save_all_action)
        self.addAction(self.deselect_action)
        self.addAction(self.next_active_resonance_action)
        self.addAction(self.prev_active_resonance_action)
        self.addAction(self.delete_action)
        self.addAction(self.add_action)
        self.addAction(self.edit_action)


    def initUIControls(self):
        print('initUIControls')
        self.is_loading_settings = True
        self.setUIAnalysisFormat()
        # self.setUIFilterParameters() # called by setUIAnalysisFormat
        # self.setUIPeakFinderParameters() # called by setUIAnalysisFormat
        self.is_loading_settings = False

    def setUIAnalysisFormat(self):
        print('setUIAnalysisFormat')
        format = self.active_format
        self.combo_analysis_format.setCurrentText(format)
        self.setUIFilterParameters()
        self.setUIPeakFinderParameters()

    def getUIAnalysisFormat(self):
        print('getUIAnalysisFormat')
        return self.combo_analysis_format.currentText()
    
    def onAnalysisFormatChanged(self):
        print('onAnalysisFormatChanged')
        format = self.getUIAnalysisFormat()
        if not self.is_loading_settings:
            self.active_format = format
            self.setUIFilterParameters()
            self.setUIPeakFinderParameters()
            self.saveSettingsActiveFormat()
            self.applyFiltering()
            self.updateResonances()
            self.refreshUI()
        else:
            # print('Ignoring format change during settings load')
            pass


    def setUIFilterParameters(self):
        print('setUIFilterParameters')
        params = self.filterManager.get_filter_params()
        was_loading_settings = self.is_loading_settings
        self.is_loading_settings = True
        self.spin_lowpass.setValue(params.get('lowpass_edge', 0.75))
        self.spin_highpass.setValue(params.get('highpass_edge', 0.001))
        self.spin_median_kernel.setValue(params.get('median_kernel_size', 1))
        self.is_loading_settings = was_loading_settings

    def getUIFilterParameters(self):
        print('getUIFilterParameters')
        params = {
            'lowpass_edge': self.spin_lowpass.value(),
            'highpass_edge': self.spin_highpass.value(),
            'median_kernel_size': self.spin_median_kernel.value()
        }
        return params

    def onFilterParameterChanged(self):
        print('onFilterParameterChanged')
        if not self.is_loading_settings:
            params = self.getUIFilterParameters()
            print(params)
            self.filterManager.set_filter_params(params)
            self.applyFiltering()
            self.saveSettingsFilterParameters()
            self.updateResonances()
            self.refreshUI()
        else:
            # print('Ignoring parameter change during settings load')
            pass

    def setUIPeakFinderParameters(self):
        print('setUIPeakFinderParameters')
        format = self.active_format
        params = self.peak_finder_params.get(format, {})

        was_loading_settings = self.is_loading_settings
        self.is_loading_settings = True
        
        # Set parameters into UI controls
        if format=="Log Magnitude dB":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 100.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 100.0))
            self.spin_width_max.setValue(params.get('width_max', 1000000.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', False))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=="Lin Magnitude V":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=="Phase rad":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=="Unwrapped Phase rad":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=="Group Delay us (-dphi/df)":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=="Complex Gradient V/Hz (speed)":
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        elif format=='Sin(IQ,didq)':
            self.check_prominence.setChecked(params.get('prominence_enabled', True))
            self.spin_prominence_min.setValue(params.get('prominence_min', 0.1))
            self.spin_prominence_max.setValue(params.get('prominence_max', 10.0))
            self.check_width.setChecked(params.get('width_enabled', True))
            self.spin_width_min.setValue(params.get('width_min', 0.1))
            self.spin_width_max.setValue(params.get('width_max', 100.0))
            self.check_threshold.setChecked(params.get('threshold_enabled', True))
            self.spin_threshold_min.setValue(params.get('threshold_min', 0.0))
            self.spin_threshold_max.setValue(params.get('threshold_max', 1.0))
            self.check_height.setChecked(params.get('height_enabled', False))
            self.spin_height_min.setValue(params.get('height_min', 0.0))
            self.spin_height_max.setValue(params.get('height_max', 1.0))
            self.check_distance.setChecked(params.get('distance_enabled', True))
            self.spin_distance_min.setValue(params.get('distance_value', 1000))
            self.combo_peak_direction.setCurrentIndex([1,-1].index(params.get('peak_direction', 1)))
        self.is_loading_settings = was_loading_settings
    
    def getUIPeakFinderParameters(self):
        print('getUIPeakFinderParameters')
        params = {
            'prominence_enabled': self.check_prominence.isChecked(),
            'prominence_min': self.spin_prominence_min.value(),
            'prominence_max': self.spin_prominence_max.value(),
            'width_enabled': self.check_width.isChecked(),
            'width_min': self.spin_width_min.value(),
            'width_max': self.spin_width_max.value(),
            'threshold_enabled': self.check_threshold.isChecked(),
            'threshold_min': self.spin_threshold_min.value(),
            'threshold_max': self.spin_threshold_max.value(),
            'height_enabled': self.check_height.isChecked(),
            'height_min': self.spin_height_min.value(),
            'height_max': self.spin_height_max.value(),
            'distance_enabled': self.check_distance.isChecked(),
            'distance_value': self.spin_distance_min.value(),
            'peak_direction': [1,-1][self.combo_peak_direction.currentIndex()]
        }
        self.peak_finder_params[self.active_format] = params
        return params

    def onPeakFinderParameterChanged(self):
        print('onPeakFinderParameterChanged')
        # Save current parameters into the dictionary, manager and settings
        if not self.is_loading_settings:
            params = self.getUIPeakFinderParameters()
            params['frequency_stepsize'] = self.frequency_stepsize
            self.peak_finder_params[self.active_format] = params
            self.peakFinderManager.set_finder_parameters(params)
            self.saveSettingsFinderParameters()
            self.updateResonances()
            self.refreshUI()
        else:
            # print('Ignoring parameter change during settings load')
            pass
        

    def initUILayout(self):
        self.setWindowTitle("MKID Resonance Finder")  
        # self.setWindowIcon(QIcon(":/icons/icon.png"))
        self.setGeometry(100, 100, 1200, 800)

        # Main central widget
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)

        # Main vertical layout for the entire window
        self.layout_main = QVBoxLayout(self.central_widget)

        # -- Top area: Horizontal layout with Load button + filename label --
        self.layout_open_hbox = QHBoxLayout()
        self.button_open = QPushButton("Open")
        self.label_filename = QLabel("No file loaded")
        self.label_filename.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.layout_open_hbox.addWidget(self.button_open)
        self.layout_open_hbox.addWidget(self.label_filename)
        self.layout_open_hbox.addStretch()

        # Add the open HBox to the main layout
        self.layout_main.addLayout(self.layout_open_hbox)

        # -- Splitter: Left + Right sections --
        self.hsplitter = QSplitter(Qt.Horizontal)
        self.layout_main.addWidget(self.hsplitter)

        # Left pane
        self.left_pane = QWidget()
        self.left_layout = QVBoxLayout(self.left_pane)
        self.hsplitter.addWidget(self.left_pane)

        # Right pane
        self.right_pane = QWidget()
        self.right_layout = QVBoxLayout(self.right_pane)
        self.hsplitter.addWidget(self.right_pane)

        # Adjust initial splitter sizes and collapsibility (as in first code)
        self.hsplitter.setSizes([(self.width() * 2) // 3, (self.width() * 1) // 3])
        self.hsplitter.setCollapsible(0, False)
        self.hsplitter.setCollapsible(1, False)

        # Set the timers for resize/splitter move debouncing
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)
        self.resize_timer.timeout.connect(self.onResizeFinished)

        self.hsplitter_timer = QTimer(self)
        self.hsplitter_timer.setSingleShot(True)
        self.hsplitter_timer.timeout.connect(self.onSplitterMoveFinished)


        # ------------------- LEFT PANE ------------------- #

        # Selection mode checkbox
        self.check_selection_mode = QCheckBox(
            "Enable click and drag in figure for multiple selection "
            "(pan and zoom buttons must be disabled)"
        )
        self.left_layout.addWidget(self.check_selection_mode)

        # Raw sweep magnitude groupbox
        self.group_raw_sweep = QGroupBox("Raw sweep magnitude")
        raw_sweep_layout = QVBoxLayout()
        self.fig_raw, self.ax_raw = plt.subplots()
        self.line_raw, = self.ax_raw.plot([],[],picker=True, pickradius=5)
        self.ax_raw.set_xlabel("Frequency (MHz)")
        self.ax_raw.set_ylabel("Magnitude (dB)")
        self.canvas_raw = FigureCanvas(self.fig_raw)
        self.toolbar_raw = NavigationToolbar(self.canvas_raw, self)
        raw_sweep_layout.addWidget(self.toolbar_raw)
        raw_sweep_layout.addWidget(self.canvas_raw)
        self.group_raw_sweep.setLayout(raw_sweep_layout)

        # Filtered sweep groupbox
        self.group_filtered_sweep = QGroupBox("Filtered sweep for peak finding")
        filtered_sweep_layout = QVBoxLayout()
        self.fig_filtered, self.ax_filtered = plt.subplots()
        self.ax_filtered.sharex(self.ax_raw)
        self.line_filtered, = self.ax_filtered.plot([],[],picker=True, pickradius=5)
        self.ax_filtered.set_xlabel("Frequency (MHz)")
        self.ax_filtered.set_ylabel("Magnitude (dB)")
        self.canvas_filtered = FigureCanvas(self.fig_filtered)
        self.toolbar_filtered = NavigationToolbar(self.canvas_filtered, self)
        filtered_sweep_layout.addWidget(self.toolbar_filtered)
        filtered_sweep_layout.addWidget(self.canvas_filtered)
        self.group_filtered_sweep.setLayout(filtered_sweep_layout)

        # Resonance viewer groupbox (contains two “zoom” subplots + analysis)
        self.group_resonance_viewer = QGroupBox("Resonance viewer")
        resonance_viewer_layout = QHBoxLayout(self.group_resonance_viewer)

        # Left zoom (active raw)
        self.widget_zoom_raw = QWidget()
        zoom_raw_layout = QVBoxLayout(self.widget_zoom_raw)
        self.fig_active_raw, self.ax_active_raw = plt.subplots()
        self.line_active_raw, = self.ax_active_raw.plot([],[],picker=True, pickradius=5)
        self.ax_active_raw.set_xlabel("Frequency (MHz)")
        self.ax_active_raw.set_ylabel("Magnitude (dB)")
        self.canvas_active_raw = FigureCanvas(self.fig_active_raw)
        zoom_raw_layout.addWidget(self.canvas_active_raw)

        # Right zoom (active filtered)
        self.widget_zoom_filtered = QWidget()
        zoom_filtered_layout = QVBoxLayout(self.widget_zoom_filtered)
        self.fig_active_filtered, self.ax_active_filtered = plt.subplots()
        self.ax_active_filtered.sharex(self.ax_active_raw)

        self.line_active_filtered, = self.ax_active_filtered.plot([],[],picker=True, pickradius=5)
        self.ax_active_filtered.set_xlabel("Frequency (MHz)")
        self.ax_active_filtered.set_ylabel("Magnitude (dB)")
        self.canvas_active_filtered = FigureCanvas(self.fig_active_filtered)
        zoom_filtered_layout.addWidget(self.canvas_active_filtered)

        # Analysis layout (Prev/Next, Add, Delete, toggle save, etc.)
        self.widget_analysis = QWidget()
        analysis_layout = QVBoxLayout(self.widget_analysis)

        self.label_analysis = QLabel("Analysis Results")
        self.label_analysis.setTextInteractionFlags(Qt.TextSelectableByMouse)

        analysis_layout.addWidget(self.label_analysis)

        # Navigation buttons (Prev/Next)
        nav_layout = QHBoxLayout()
        self.button_previous_resonance = QPushButton("Prev")
        self.button_next_resonance = QPushButton("Next")
        nav_layout.addWidget(self.button_previous_resonance)
        nav_layout.addWidget(self.button_next_resonance)
        analysis_layout.addLayout(nav_layout)

        toggle_layout = QHBoxLayout()
        self.button_toggle_save = QPushButton("Toggle Save Selected")
        self.button_toggle_save_all = QPushButton("Toggle Save All")
        toggle_layout.addWidget(self.button_toggle_save)
        toggle_layout.addWidget(self.button_toggle_save_all)
        analysis_layout.addLayout(toggle_layout)

        # Add/Edit/Delete
        btn_layout = QHBoxLayout()
        self.button_add_resonance = QPushButton("Add")
        self.button_edit_resonance = QPushButton("Edit")
        self.button_remove_resonance = QPushButton("Delete")
        btn_layout.addWidget(self.button_add_resonance)
        btn_layout.addWidget(self.button_edit_resonance)
        btn_layout.addWidget(self.button_remove_resonance)
        analysis_layout.addLayout(btn_layout)

        # Place the three sub-widgets in the resonance viewer layout
        resonance_viewer_layout.addWidget(self.widget_zoom_raw)
        resonance_viewer_layout.addWidget(self.widget_zoom_filtered)
        resonance_viewer_layout.addWidget(self.widget_analysis)

        # Now add all groupboxes to the left vertical layout
        self.left_layout.addWidget(self.group_raw_sweep)
        self.left_layout.addWidget(self.group_filtered_sweep)
        self.left_layout.addWidget(self.group_resonance_viewer)


        # ------------------- RIGHT PANE ------------------- #

        # Filter Parameters groupbox
        self.group_filter_params = QGroupBox("Smoothing Filter Parameters")
        filter_params_layout = QGridLayout()

        self.label_analysis_format = QLabel("Analysis Format:")
        self.combo_analysis_format = QComboBox()
        self.combo_analysis_format.addItems(self.analysis_formats)
        self.combo_analysis_format.setCurrentIndex(1)
        filter_params_layout.addWidget(self.label_analysis_format, 0, 0)
        filter_params_layout.addWidget(self.combo_analysis_format, 0, 1)

        self.label_highpass = QLabel("Highpass Edge (Nyq=1):")
        self.spin_highpass = ScientificSpinBox()
        self.spin_highpass.setKeyboardTracking(False)
        self.spin_highpass.setStepType(StepType)
        self.spin_highpass.setDecimals(6)
        self.spin_highpass.setRange(0, 1)
        filter_params_layout.addWidget(self.label_highpass, 1, 0)
        filter_params_layout.addWidget(self.spin_highpass, 1, 1)

        self.label_lowpass = QLabel("Lowpass Edge (Nyq=1):")
        self.spin_lowpass = ScientificSpinBox()
        self.spin_lowpass.setKeyboardTracking(False)
        self.spin_lowpass.setStepType(StepType)
        self.spin_lowpass.setDecimals(6)
        self.spin_lowpass.setRange(0,1)
        filter_params_layout.addWidget(self.label_lowpass, 2, 0)
        filter_params_layout.addWidget(self.spin_lowpass, 2, 1)

        self.label_median_kernel = QLabel("Median Kernel Size:")
        self.spin_median_kernel = QSpinBox()
        self.spin_median_kernel.setKeyboardTracking(False)
        self.spin_median_kernel.setMinimum(1)
        self.spin_median_kernel.setValue(1)
        self.spin_median_kernel.setSingleStep(2)
        filter_params_layout.addWidget(self.label_median_kernel, 3, 0)
        filter_params_layout.addWidget(self.spin_median_kernel, 3, 1)

        self.group_filter_params.setLayout(filter_params_layout)

        # Peak Finder Parameters groupbox
        self.group_peak_finder_params = QGroupBox("Peak Finder Parameters")
        peak_finder_layout = QGridLayout()
        
        row = 0
        self.label_prominence = QLabel("Prominence")
        self.check_prominence = QCheckBox()
        self.check_prominence.setChecked(True) 
        self.spin_prominence_min = ScientificSpinBox()
        self.spin_prominence_min.setKeyboardTracking(False)
        self.spin_prominence_min.setGroupSeparatorShown(True)
        self.spin_prominence_min.setStepType(StepType)
        self.spin_prominence_min.setDecimals(6)
        self.spin_prominence_min.setValue(1.0)
        self.spin_prominence_min.setRange(0, 1000000)
        self.spin_prominence_max = ScientificSpinBox()
        self.spin_prominence_max.setKeyboardTracking(False)
        self.spin_prominence_max.setGroupSeparatorShown(True)
        self.spin_prominence_max.setStepType(StepType)
        self.spin_prominence_max.setRange(0, 1000000)
        self.spin_prominence_max.setDecimals(6)
        self.spin_prominence_max.setValue(100.0)
        peak_finder_layout.addWidget(self.label_prominence,row,0)
        peak_finder_layout.addWidget(self.check_prominence,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.spin_prominence_min,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.spin_prominence_max,row,5)

        row += 1
        self.label_width = QLabel("Width")
        self.check_width = QCheckBox()
        self.check_width.setChecked(True)
        self.spin_width_min = ScientificSpinBox()
        self.spin_width_min.setKeyboardTracking(False)
        self.spin_width_min.setGroupSeparatorShown(True)
        self.spin_width_min.setStepType(StepType)
        self.spin_width_min.setRange(0, np.inf)
        self.spin_width_min.setDecimals(3)
        self.spin_width_min.setMinimum(1)
        self.spin_width_min.setValue(100.0)
        self.spin_width_max = ScientificSpinBox()
        self.spin_width_max.setKeyboardTracking(False)
        self.spin_width_max.setGroupSeparatorShown(True)
        self.spin_width_max.setStepType(StepType)
        self.spin_width_max.setRange(0, np.inf)
        self.spin_width_max.setDecimals(3)
        self.spin_width_max.setMinimum(1)
        self.spin_width_max.setValue(10000000.0)
        peak_finder_layout.addWidget(self.label_width,row,0)
        peak_finder_layout.addWidget(self.check_width,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.spin_width_min,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.spin_width_max,row,5)

        row += 1
        self.label_distance = QLabel("Distance")
        self.check_distance = QCheckBox()
        self.check_distance.setChecked(True)
        self.spin_distance_min = ScientificSpinBox()
        self.spin_distance_min.setKeyboardTracking(False)
        self.spin_distance_min.setGroupSeparatorShown(True)
        self.spin_distance_min.setStepType(StepType)
        self.spin_distance_min.setDecimals(3)
        self.spin_distance_min.setRange(0, np.inf)
        self.spin_distance_min.setMinimum(0)
        self.spin_distance_min.setValue(1000)
        peak_finder_layout.addWidget(self.label_distance,row,0)
        peak_finder_layout.addWidget(self.check_distance,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.spin_distance_min,row,3)

        row += 1
        self.label_height = QLabel("Height")
        self.check_height = QCheckBox()
        self.check_height.setChecked(False)
        self.spin_height_min = ScientificSpinBox()
        self.spin_height_min.setKeyboardTracking(False)
        self.spin_height_min.setGroupSeparatorShown(True)
        self.spin_height_min.setStepType(StepType)
        self.spin_height_min.setDecimals(6)
        self.spin_height_min.setRange(-np.inf, np.inf)
        self.spin_height_min.setMinimum(0)
        self.spin_height_min.setValue(0.0)
        self.spin_height_max = ScientificSpinBox()
        self.spin_height_max.setKeyboardTracking(False)
        self.spin_height_max.setGroupSeparatorShown(True)
        self.spin_height_max.setStepType(StepType)
        self.spin_height_max.setRange(-np.inf, np.inf)
        self.spin_height_max.setDecimals(6)
        self.spin_height_max.setMinimum(0)
        self.spin_height_max.setValue(100.0)
        peak_finder_layout.addWidget(self.label_height,row,0)
        peak_finder_layout.addWidget(self.check_height,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.spin_height_min,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.spin_height_max,row,5)

        row += 1
        self.label_threshold = QLabel("Threshold")
        self.check_threshold = QCheckBox()
        self.check_threshold.setChecked(False)
        self.spin_threshold_min = ScientificSpinBox()
        self.spin_threshold_min.setKeyboardTracking(False)
        self.spin_threshold_min.setGroupSeparatorShown(True)
        self.spin_threshold_min.setStepType(StepType)
        self.spin_threshold_min.setDecimals(6)
        self.spin_threshold_min.setRange(-np.inf,np.inf)
        self.spin_threshold_min.setMinimum(0)
        self.spin_threshold_min.setValue(0)
        self.spin_threshold_max = ScientificSpinBox()
        self.spin_threshold_max.setKeyboardTracking(False)
        self.spin_threshold_max.setGroupSeparatorShown(True)
        self.spin_threshold_max.setStepType(StepType)
        self.spin_threshold_max.setRange(-np.inf, np.inf)
        self.spin_threshold_max.setDecimals(6)
        self.spin_threshold_max.setMinimum(0)
        self.spin_threshold_max.setValue(100.0)
        peak_finder_layout.addWidget(self.label_threshold,row,0)
        peak_finder_layout.addWidget(self.check_threshold,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.spin_threshold_min,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.spin_threshold_max,row,5)

        row += 1
        self.label_peak_direction = QLabel("Peak Direction")
        self.combo_peak_direction = QComboBox()
        self.combo_peak_direction.addItems(["Peaks","Dips"])
        self.combo_peak_direction.setCurrentIndex(1)
        peak_finder_layout.addWidget(self.label_peak_direction,row,0)
        peak_finder_layout.addWidget(self.combo_peak_direction,row,3)

        self.group_peak_finder_params.setLayout(peak_finder_layout)

        # Resonances List groupbox (table)
        self.group_resonances = QGroupBox("Resonances List")
        resonances_layout = QVBoxLayout()

        self.resonances_table = QTableWidget()
        self.resonances_table.setColumnCount(8)
        self.resonances_table.setHorizontalHeaderLabels([
            "Save", "ID", "Frequency (MHz)",  "FWHM (kHz)","Depth (dB)", "Q-factor","Q-coupling","Q-internal"])
        self.resonances_table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.resonances_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.resonances_table.verticalHeader().setVisible(False)
        self.resonances_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.resonances_table.setSelectionMode(QTableWidget.ExtendedSelection)
        self.resonances_table.resizeColumnsToContents()


        self.label_num_resonances = QLabel("Resonances found: n/a")
        self.button_fixids = QPushButton("Fix IDs")
        self.button_save = QPushButton("Save/Export")

        resonances_layout.addWidget(self.resonances_table)
        resonances_layout.addWidget(self.label_num_resonances)
        # resonances_layout.addWidget(self.button_fixids) #need to implement correct naming of unsaved resonances
        resonances_layout.addWidget(self.button_save)
        self.group_resonances.setLayout(resonances_layout)

        # Add everything to the right layout
        self.right_layout.addWidget(self.group_filter_params)
        self.right_layout.addWidget(self.group_peak_finder_params)
        self.right_layout.addWidget(self.group_resonances)


        # Window is ready
        self.show()


    def initUIConnections(self):
        print('initUIConnections')

        # Splitter events for debouncing
        self.hsplitter.splitterMoved.connect(self.onSplitterMoved)

        # Buttons
        self.button_open.clicked.connect(self.onOpen)
        self.button_save.clicked.connect(self.onSave)
        self.button_fixids.clicked.connect(self.onFixIDs)
        self.button_add_resonance.clicked.connect(self.onAddResonance)
        self.button_edit_resonance.clicked.connect(self.onEditResonance)
        self.button_remove_resonance.clicked.connect(self.onDeleteResonance)
        self.button_next_resonance.clicked.connect(self.onNextResonance)
        self.button_previous_resonance.clicked.connect(self.onPreviousResonance)
        self.button_toggle_save.clicked.connect(self.onToggleSave)
        self.button_toggle_save_all.clicked.connect(self.onToggleSaveAll)

        # Filter params
        self.combo_analysis_format.currentTextChanged.connect(self.onAnalysisFormatChanged)

        for spinbox in [self.spin_highpass, self.spin_lowpass, self.spin_median_kernel]:
            spinbox.valueChanged.connect(self.onFilterParameterChanged)

        # Finder params
        for checkbox in [self.check_prominence, self.check_width, self.check_threshold, self.check_height, self.check_distance]:
            checkbox.stateChanged.connect(self.onPeakFinderParameterChanged)

        for spinbox in [self.spin_prominence_min, self.spin_prominence_max, 
                        self.spin_width_min, self.spin_width_max, 
                        self.spin_threshold_min, self.spin_threshold_max, 
                        self.spin_height_min, self.spin_height_max, 
                        self.spin_distance_min]:
            spinbox.valueChanged.connect(self.onPeakFinderParameterChanged)
        self.combo_peak_direction.currentIndexChanged.connect(self.onPeakFinderParameterChanged)

        # Table
        self.resonances_table.cellClicked.connect(self.onResonanceTableCellClicked)
        self.resonances_table.itemSelectionChanged.connect(self.onResonanceTableSelectionChanged)
        self.resonances_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.resonances_table.customContextMenuRequested.connect(self.showResonancesTableContextMenu)

        # Plots
        self.check_selection_mode.stateChanged.connect(self.toggleSelectionMode)
        
        self.canvas_raw.mpl_connect("button_press_event", self.onAxesButtonPress)
        self.canvas_filtered.mpl_connect("button_press_event", self.onAxesButtonPress)
        self.canvas_active_raw.mpl_connect("button_press_event", self.onAxesButtonPress)
        self.canvas_active_filtered.mpl_connect("button_press_event", self.onAxesButtonPress)




    def resizeEvent(self, event):
        super().resizeEvent(event)
        if self.resize_timer:
            self.resize_timer.start(200)

    def onResizeFinished(self):
        """
        Called once the window resizing has finished.
        Perform any layout updates or redraws here.
        """
        print('onResizeFinished')
        self.refreshFigures()

    def onSplitterMoved(self, pos, index):
        print('onSplitterMoved')
        if self.hsplitter_timer:
            self.hsplitter_timer.start(200)


    def onSplitterMoveFinished(self):
        """
        Called once the splitter has finished moving.
        Perform any layout updates or redraws here.
        """
        print('onSplitterMoveFinished')
        self.refreshFigures()        

    def onOpen(self):
        print('onOpen')
        self.openFile()

    def onSave(self):
        print('onSave')
        self.saveResonances()

    def onFixIDs(self):
        print('onFixIDs')
        self.fixResonanceIDs()
        self.refreshUI()

    def onAddResonance(self):
        print('onAddResonance')
        self.addResonance()
        self.refreshUI()

    def onEditResonance(self):
        print('onEditResonance')
        self.editResonance(self.active_resonance_index)
        self.refreshUI()
        
    def onDeleteResonance(self):
        print('onDeleteResonance')
        self.deleteSelectedResonances()
        self.refreshUI()

    def onNextResonance(self):
        print('onNextResonance')
        self.nextActiveResonance()
        self.refreshUI()

    def onPreviousResonance(self):
        print('onPreviousResonance')
        self.prevActiveResonance()
        self.refreshUI()

    def onToggleSave(self):
        print('onToggleSave')
        self.toggleSaveSelectedResonances()
        self.refreshUI()

    def onToggleSaveAll(self):
        print('onToggleSaveAll')
        self.toggleSaveAllResonances()
        self.refreshUI()

    def onResonanceTableCellClicked(self, row, column):
        print('onResonanceTableCellClicked')
        self.setActiveResonanceIndex(row)
        self.refreshUI()


    def onResonanceTableSelectionChanged(self):
        print('onResonanceTableSelectionChanged')
        selected_resonances = self.getSelectedResonances()
        selected_indexes = [i.row() for i in self.resonances_table.selectionModel().selectedRows()]

        if selected_resonances is not None:
            if len(selected_resonances) == 0:
                if len(selected_indexes) == 0 :
                    pass
                elif len(selected_indexes) == 1:
                    self.setActiveResonanceIndex(selected_indexes[0])
                    self.setSelectedResonances(selected_indexes)
                elif len(selected_indexes) > 1:
                    self.setSelectedResonances(selected_indexes)
            else:
                if  len(selected_indexes) > 0 :
                    self.setSelectedResonances(selected_indexes)
                    self.setActiveResonanceIndex(selected_indexes[-1])
                elif len(selected_indexes) > 1:
                    self.setSelectedResonances(selected_indexes)
        else:
            if len(selected_indexes) == 0:
                self.setSelectedResonances(selected_indexes)
            elif  len(selected_indexes) > 0 :
                self.setActiveResonanceIndex(selected_indexes[-1])
                self.setSelectedResonances(selected_indexes)

            elif len(selected_indexes) > 1:
                self.setSelectedResonances(selected_indexes)


        self.refreshUI()



    def toggleSelectionMode(self, state):
        print('toggleSelectionMode')
        if state == Qt.Checked:
            self.activateRectangleSelectors()
        else:
            self.deactivateRectangleSelectors()

    def activateRectangleSelectors(self):
        print('activateRectangleSelectors')
        axes_list = [self.ax_raw, self.ax_filtered, self.ax_active_raw, self.ax_active_filtered]  # List of axes to enable selection on
        for ax in axes_list:
            ax.set_autoscale_on(False)

            if ax not in self.rectangle_selectors:
                selector = RectangleSelector(
                    ax,
                    onselect=self.onSelectRectangle,
                    useblit=True,
                    button=[1],  # Left mouse button
                    minspanx=5,
                    minspany=5,
                    spancoords='pixels',
                    interactive=True,
                    drag_from_anywhere=False,
                    props=dict(facecolor=None, edgecolor='black', alpha=0.5, fill=False)
                )
                self.rectangle_selectors[ax] = selector
            else:
                self.rectangle_selectors[ax].set_active(True)
        
        # Change cursor to crosshair for all canvases
        self.canvas_raw.setCursor(Qt.CrossCursor)
        self.canvas_filtered.setCursor(Qt.CrossCursor)
        self.canvas_active_raw.setCursor(Qt.CrossCursor)
        self.canvas_active_filtered.setCursor(Qt.CrossCursor)
        
    def deactivateRectangleSelectors(self):
        print('deactivateRectangleSelectors')
        for selector in self.rectangle_selectors.values():
            selector.set_active(False)
        axes_list = [self.ax_raw, self.ax_filtered, self.ax_active_raw, self.ax_active_filtered]  # List of axes to enable selection on
        for ax in axes_list:
            ax.set_autoscale_on(True)
        # Reset cursors
        self.canvas_raw.setCursor(Qt.ArrowCursor)
        self.canvas_filtered.setCursor(Qt.ArrowCursor)
        self.canvas_active_raw.setCursor(Qt.ArrowCursor)
        self.canvas_active_filtered.setCursor(Qt.ArrowCursor)

    def onSelectRectangle(self, eclick, erelease):
        print('onSelectRectangle')
        ax = eclick.inaxes
        if ax is None:
            return
        self.handleRectangleSelection(ax, eclick, erelease)


    def handleRectangleSelection(self, ax, eclick, erelease):
        print('handleRectangleSelection')
        # Get rectangle coordinates in data units
        x_min, x_max = sorted([eclick.xdata, erelease.xdata])
        y_min, y_max = sorted([eclick.ydata, erelease.ydata])

        # Determine which data to use based on the axes
        if ax == self.ax_raw or ax == self.ax_active_raw:
            x_data = self.frequencies / 1e6  # Convert to MHz
            y_data = self.log_magnitude
        elif ax == self.ax_filtered or ax == self.ax_active_filtered:
            x_data = self.frequencies / 1e6  # Convert to MHz
            y_data = self.filtered_data
        else:
            return  # Axes not recognized

        # Find resonances within the rectangle
        selected_indices = []
        for idx, resonance in enumerate(self.resonances):
            freq_mhz = resonance.frequency / 1e6
            if resonance.peak_idx is not None:
                y_value = y_data[resonance.peak_idx]
                if x_min <= freq_mhz <= x_max and y_min <= y_value <= y_max:
                    selected_indices.append(idx)
        print(selected_indices)
        if not selected_indices:
            # QMessageBox.information(self, "Selection", "No resonances found in the selected area.")
            pass
        selector = self.rectangle_selectors[ax]  # or wherever you store the selector
        selector.set_visible(False)
        
        self.setSelectedResonances(selected_indices)
        self.refreshUI()


    def showResonanceContextMenu(self, resonance_index, mouse_event):
        """
        Show a right-click menu for the given resonance index.
        """
        print('showResonanceContextMenu')
        menu = QMenu()

        action_add = QAction("Add Resonance", self)
        action_edit = QAction("Edit Resonance", self)
        action_delete = QAction("Delete Resonance", self)

        # Connect them to lambdas or methods
        action_add.triggered.connect(lambda: self.addResonance(frequency_mhz=self.resonances[resonance_index].frequency/1e6))
        action_edit.triggered.connect(lambda: self.editResonance(resonance_index))
        action_delete.triggered.connect(lambda: self.deleteResonance(resonance_index))

        menu.addAction(action_add)
        menu.addAction(action_edit)
        menu.addAction(action_delete)

        # We need global screen coords to pop up a QMenu
        global_pos = QtGui.QCursor.pos()  
        # or mouse_event.globalPos() in some PyQt versions

        menu.exec_(global_pos)
   

    def showEmptySpaceContextMenu(self, event):
        print('showEmptySpaceContextMenu')
        menu = QMenu()
        action_add_new = QAction("Add Resonance Here", self)
        action_add_new.triggered.connect(lambda: self.addResonance(frequency_mhz=event.xdata))
        menu.addAction(action_add_new)
        menu.exec_(QtGui.QCursor.pos())

    def getClickedResonanceIndex(self, event, pick_tolerance=10):
        """
        Return the resonance index if the click is within pick_tolerance (pixels) of a marker,
        or None if not near any marker.
        """
        print('getClickedResonanceIndex')
        if event.inaxes == self.ax_raw:
            xdata, ydata = self.marker_line_raw.get_data()
        elif event.inaxes == self.ax_filtered:
            xdata, ydata = self.marker_line_filtered.get_data()
        elif event.inaxes == self.ax_active_raw:
            xdata, ydata = self.marker_line_active_raw.get_data()
        elif event.inaxes == self.ax_active_filtered:
            xdata, ydata = self.marker_line_active_filtered.get_data()

        # Transform from data coords to pixel coords
        trans = event.inaxes.transData
        click_x, click_y = event.x, event.y  # already in pixels
        
        min_dist = float('inf')
        best_index = None
        
        for i in range(len(xdata)):
            # data->pixel
            point_px, point_py = trans.transform((xdata[i], ydata[i]))
            dist = np.hypot(click_x - point_px, click_y - point_py)
            if dist < pick_tolerance and dist < min_dist:
                min_dist = dist
                best_index = i

        # best_index is the index in the xdata,ydata arrays
        # this corresponds to self.resonances[best_index], if you stored them in order
        return best_index

    def onAxesButtonPress(self, event):
        """
        This method is called whenever a mouse button is pressed on one of the Matplotlib canvases.
        We’ll route the event to onAxesLeftClick or onAxesRightClick as needed.
        """
        print('onAxesButtonPress')
        # If user is in Pan/Zoom mode, ignore
        if event.inaxes == self.ax_raw:
            if self.toolbar_raw.mode != '':
                return
        elif event.inaxes == self.ax_filtered:
            if self.toolbar_filtered.mode != '':
                return
        elif event.inaxes == self.ax_active_raw:
            if self.toolbar_active_raw.mode != '':
                return
        elif event.inaxes == self.ax_active_filtered:
            if self.toolbar_active_filtered.mode != '':
                return
        else:
            return
        
        if event.button == 1:   # Left-click
            self.onAxesLeftClick(event)
        elif event.button == 3: # Right-click
            self.onAxesRightClick(event)


    def onAxesRightClick(self, event):
        """
        Right-click logic. Show a context menu depending on whether we clicked near a marker.
        """
        print('onAxesRightClick')
        pick_tolerance = 10
        clicked_index = self.getClickedResonanceIndex(event, pick_tolerance)
        
        if clicked_index is not None:
            # Right-click near a resonance marker -> show resonance context menu
            print(f"Right-click near resonance index={clicked_index}")
            self.setActiveResonanceIndex(clicked_index)
            self.refreshUI()
            self.showResonanceContextMenu(clicked_index, event)
        else:
            # Right-click in empty space -> show empty context menu (or do nothing)
            print("Right-click in empty space.")
            self.showEmptySpaceContextMenu(event)


    def onAxesLeftClick(self, event):
        """
        Left-click logic. If near a marker, select that resonance.
        Otherwise, you might do something like adding a new resonance or ignoring.
        """
        print('onAxesLeftClick')
        # Tolerance in pixels
        pick_tolerance = 10  
        clicked_index = self.getClickedResonanceIndex(event, pick_tolerance)
        
        if clicked_index is not None:
            # Found a resonance near the click
            print(f"Left-click near resonance index={clicked_index}")
            self.setActiveResonanceIndex(clicked_index)
            self.refreshUI()
        else:
            # Clicked empty space (not near a marker)
            print("Left-click in empty space.")
            pass

    def addResonanceAtPlotPosition(self, event):
        print('addResonanceAtPlotPosition',event)
        ax = event.inaxes
        if ax is None or event.xdata is None:
            return
        freq_mhz = event.xdata
        frequency = freq_mhz * 1e6
        self.addResonance(initial_freq_mhz=freq_mhz)

    def editResonanceAtPlotPosition(self, resonance_idx):
        print('editResonanceAtPlotPosition',resonance_idx)
        if resonance_idx is None:
            QMessageBox.information(self, "Edit Resonance", "No resonance selected.")
            return
        self.setCurrentResonanceIndex(resonance_idx)
        self.editResonance()


    def deleteResonanceAtPosition(self, resonance_idx):
        print('deleteResonanceAtPosition',resonance_idx)
        if resonance_idx is None:
            QMessageBox.information(self, "Delete Resonance", "No resonance selected.")
            return
        reply = QMessageBox.question(
            self, 'Delete Resonance',
            "Are you sure you want to delete the selected resonance?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            resonance = self.resonances.pop(resonance_idx)
            resonance.remove_markers()
            self.setCurrentResonanceIndex(None)
            self.updateResonanceIDs()
            self.updateResonancesTable()
            self.updateMarkers()
            self.updateResonancesLabel()
            self.updateNavigationButtons()
            # Redraw canvases
            self.raw_canvas.draw_idle()
            self.filtered_canvas.draw_idle()
            self.zoom_raw_canvas.draw_idle()
            self.zoom_filt_canvas.draw_idle()

    def toggleSaveResonanceAtPosition(self, resonance_idx):
        print('toggleSaveResonanceAtPosition',resonance_idx)
        if resonance_idx is None:
            QMessageBox.information(self, "Toggle Save Resonance", "No resonance selected.")
            return
        resonance = self.resonances[resonance_idx]
        resonance.save = not resonance.save
        self.updateResonanceIDs()
        self.updateResonancesTable()
        self.updateMarkers()
        self.updateResonancesLabel()
        self.updateNavigationButtons()


    def openFile(self):
        print('openFile')
        options = QFileDialog.Options()
        lastfile = self.settings.value("lastFile", "")
        last_dir = os.path.dirname(lastfile) if lastfile else ""
        filename, _ = QFileDialog.getOpenFileName(self, "Open Sweep File", last_dir, 
                                                  "All Files (*)", options=options)
        if filename:
            self.loadFile(filename)
        else:
            print('No file selected.')

    def saveResonances(self):
        print('saveResonances')
        options = QFileDialog.Options()
        if os.path.splitext(self.label_filename.text())[-1] == '.fits':
            default_ext = '.txt'
            default_filename = os.path.splitext(self.label_filename.text())[0] + default_ext
            filename, _ = QFileDialog.getSaveFileName(self, "Save Resonances", default_filename,
                                                    "KIDLAB Toneslist Files (*.txt);;Resonance Files (*.resonances);;All Files (*)", options=options)
        else:
            default_ext = ".resonances"
            default_filename = os.path.splitext(self.label_filename.text())[0] + default_ext
            filename, _ = QFileDialog.getSaveFileName(self, "Save Resonances", default_filename,
                                                    "Resonance Files (*.resonances);;KIDLAB Toneslist Files (*.txt);;All Files (*)", options=options)
        if filename:
            try:
                if filename.endswith('.resonances'):
                    with open(filename, 'w') as f:
                        # f.write("ID,Frequency(Hz),Q-factor,FWHM(Hz)\n")
                        f.write("#ID\tFrequency(Hz)\tLinewidth(Hz)\tQfactor(Qr)\tQcoupling\tQinternal\tDipDepth(dB)\n")
                        for resonance in self.resonances:
                            if resonance.save and resonance.id is not None:
                                f.write('%04d\t%16.6f\t%16.6f\t%16.6f\t%16.6f\t%16.6f\t%16.6f\n'%(resonance.id,resonance.frequency,resonance.fwhm,resonance.q_factor,resonance.qc,resonance.qi,resonance.dip_depth))

                elif filename.endswith('.txt'):
                    with open(filename, 'w') as f:
                        f.write("Name\tFreq\tOffset att\tAll\tNone\n")
                        for resonance in self.resonances:
                            if resonance.save and resonance.id is not None:
                                f.write('K%03d\t%f\t%f\t%d\t%d\n'%(resonance.id,resonance.frequency,0,1,0))

                else:
                    with open(filename, 'w') as f:
                        # f.write("ID,Frequency(Hz),Q-factor,FWHM(Hz)\n")
                        f.write("#ID,Frequency(Hz)\n")
                        for resonance in self.resonances:
                            if resonance.save and resonance.id is not None:
                                f.write(f"{resonance.id},{resonance.frequency}\n")
                                        # f"{resonance.analysis.get('q', 0)},{resonance.analysis.get('fwhm', 0)}\n")
                QMessageBox.information(self, "Save Resonances", f"Resonances saved to {filename}")
            except Exception as e:
                QMessageBox.critical(self, "Error", f"Failed to save resonances: {e}")
        else:
            QMessageBox.information(self, "Save Resonances", "No file selected.")


    def showResonancesTableContextMenu(self, position):
        """
        Show a right-click context menu for the resonances table.
        """
        menu = QMenu(self)

        # Figure out which row was clicked:
        index = self.resonances_table.indexAt(position)
        row_clicked = index.row()

        # Create some actions:
        action_add = QAction("Add Resonance", self)
        action_edit = QAction("Edit Resonance", self)
        action_delete = QAction("Delete Resonance", self)

        # Connect them:
        # If you want "Add" to not rely on the clicked row, you can pass no arguments
        # or pass a frequency. E.g. self.addResonance(frequency_mhz=some_freq)
        action_add.triggered.connect(lambda: self.addResonance())

        # If the row is valid, connect to edit/delete that row
        if row_clicked >= 0:
            action_edit.triggered.connect(lambda: self.editResonance(row_clicked))
            action_delete.triggered.connect(lambda: self.deleteResonance(row_clicked))
        else:
            # If the user right-clicked outside a row (invalid row), 
            # we can disable edit/delete or remove them from the menu:
            action_edit.setEnabled(False)
            action_delete.setEnabled(False)

        # Add to the menu
        menu.addAction(action_add)
        menu.addAction(action_edit)
        menu.addAction(action_delete)

        # Show the menu at the cursor position in global coords
        menu.exec_(self.resonances_table.viewport().mapToGlobal(position))




class SplashScreen(QSplashScreen):
    def __init__(self, iconpixmap=None):
        """
        A splash screen with:
          - A progress bar
          - A multi-line text box for logging (scrolls automatically)
        """
        # Create a default pixmap if not provided
        if iconpixmap is None:
            iconpixmap = QPixmap(400, 600)
            iconpixmap.fill(Qt.lightGray)

        # Create the gray base pixmap
        base_pixmap = QPixmap(400, 600)
        base_pixmap.fill(Qt.lightGray)

        # Paint the icon on top of the gray pixmap
        painter = QPainter(base_pixmap)
        iconpixmap = iconpixmap.scaled(128, 128, Qt.KeepAspectRatio, Qt.SmoothTransformation)
        x = (base_pixmap.width() - iconpixmap.width()) // 2
        y = 20  # a bit from the top edge
        painter.drawPixmap(x, y, iconpixmap)
        painter.end()

        super().__init__(base_pixmap)

        # Keep splash on top
        self.setWindowFlag(Qt.WindowStaysOnTopHint)

        # A progress bar at the bottom
        self.progress_bar = QProgressBar(self)
        self.progress_bar.setRange(0, 100)
        self.progress_bar.setValue(0)
        # Position the progress bar manually (just above the bottom)
        bar_width = base_pixmap.width() - 100
        self.progress_bar.setGeometry(50, base_pixmap.height() - 50, bar_width, 20)

        # A multi-line text box above the progress bar
        self.log_box = QPlainTextEdit(self)
        self.log_box.setReadOnly(True)
        # Position it above the progress bar; adjust sizes to your preference
        log_height = 300
        self.log_box.setGeometry(
            50, 
            base_pixmap.height() - 50 - log_height - 10,  # 10px gap above the progress bar
            bar_width, 
            log_height)

        # # Show the splash screen immediately
        # self.show()
        # QApplication.processEvents()  # Ensure the splash screen is displayed

    def fadeOut(self,duration=1000):
        """
        Animate the windowOpacity from 1.0 to 0.0 over 'duration' milliseconds.
        Then close the splash screen.
        """
        self.anim = QPropertyAnimation(self, b"windowOpacity", self)
        self.anim.setDuration(duration)
        self.anim.setStartValue(1.0)
        self.anim.setEndValue(0.0)
        self.anim.finished.connect(self.close)
        self.anim.start()

            

    def show_progress(self, current_step, total_steps):
        """Update the splash screen's progress bar."""
        percent = int(current_step / total_steps * 100)
        self.progress_bar.setValue(percent)
        QApplication.processEvents()

    def add_log_line(self, message):
        """Append a line to the log box and scroll to the latest line."""
        self.log_box.appendPlainText(message)
        # Scroll to bottom
        vertical_scroll = self.log_box.verticalScrollBar()
        vertical_scroll.setValue(vertical_scroll.maximum())
        QApplication.processEvents()


def main():
    app = QApplication(sys.argv)

    iconpng = str(files("souk_readout_tools").joinpath("mkid_finder_app.png"))
    iconico = str(files("souk_readout_tools").joinpath("mkid_finder_app.ico"))
    if sys.platform in ['nt','win32']:
        app.setWindowIcon(QIcon(iconico))
    else:
        app.setWindowIcon(QIcon(iconpng))

    # Pass the combined pixmap to your splash screen
    splash = SplashScreen(QPixmap(iconpng))
    splash.show()

    QApplication.processEvents()

    window = ResonanceFinderApp(splash)
    window.show()

    splash.fadeOut(duration=3000)

    app.exec_()

if __name__ == "__main__":
    sys.exit(main())



# do a butterbowrth lowpass filterdef butter_lowpass(cutoff, fs, order=5):
def lowpass(data, cutoff, fs=1, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    y = filtfilt(b, a, data)
    return y

def highpass(data, cutoff, fs=1, order=5):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='high', analog=False)
    y = filtfilt(b, a, data)
    return y