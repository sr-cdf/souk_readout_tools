import sys
import os
import traceback

try:
    from importlib.resources import  files  # Python 3.9+
except ImportError:
    from importlib_resources import  files  # Python < 3.9


import numpy as np
import matplotlib.pyplot as plt
import matplotlib
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.widgets import RectangleSelector

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QHBoxLayout, QVBoxLayout,
    QFileDialog, QSplitter, QGroupBox, QComboBox, QSpinBox, QDoubleSpinBox, QTableWidget,
    QTableWidgetItem, QCheckBox, QHeaderView, QMessageBox, QInputDialog, QMenu, QAction,
    QFormLayout,QGridLayout # Import QFormLayout
)

# Use adaptive step if available
from PyQt5.QtWidgets import QAbstractSpinBox
try:
    StepType = QAbstractSpinBox.AdaptiveDecimalStepType
except AttributeError:
    StepType = QAbstractSpinBox.DefaultStepType


from PyQt5.QtCore import Qt, QSettings, QByteArray, QPoint, QTimer, QEvent, QItemSelection, QItemSelectionModel
from PyQt5.QtGui import QIcon
from PyQt5 import QtGui

from scipy.signal import butter, filtfilt, find_peaks, medfilt, peak_widths


class Resonance:
    def __init__(self, frequency, peak_idx=None, save=True):
        self.frequency = frequency  # Resonance frequency
        self.peak_idx = peak_idx  # Index in the frequency array
        self.save = save  # Whether to save/export this resonance
        self.analysis = {}  # Dictionary to store analysis results (e.g., 'q', 'fwhm')
        self.id = None  # ID assigned only if `save` is True
        self.markers = {}  # Dictionary to store marker handles


    def analyse(self, frequencies, logmag_data, filtered_data, peak_direction):
        """Perform analysis and store results in self.analysis."""
        if self.peak_idx is not None:
            peak_idx = self.peak_idx
        else:
            peak_idx = np.abs(self.frequency - frequencies).argmin()
        
        try:
            # Adjust data for peak direction
            adjusted_data = peak_direction * filtered_data
            
            # Use peak_widths to calculate width at half maximum
            results_half = peak_widths(adjusted_data, [peak_idx], rel_height=0.5)
            width_samples = results_half[0][0]  # Width in number of samples

            # Convert width to Hz using frequency step size
            frequency_step = np.mean(np.diff(frequencies))
            width_hz = width_samples * frequency_step

            # Calculate frequency, FWHM, and Q-factor
            f = frequencies[peak_idx]
            fwhm = width_hz if width_hz else frequency_step
            q_factor = f / fwhm
            mask = (frequencies < f + 10*fwhm/2) & (frequencies > f - 10*fwhm/2) 
            dip_depth = max(logmag_data[mask]) - min(logmag_data[mask])

            # Store results in self.analysis
            self.analysis['f'] = f
            self.analysis['fwhm'] = fwhm
            self.analysis['q'] = q_factor
            self.analysis['dip_depth'] = dip_depth

        except Exception as e:
            print(f"Error in analyse method: {e}")
            self.analysis['f'] = frequencies[peak_idx]
            self.analysis['fwhm'] = frequency_step
            self.analysis['q'] = self.analysis['f']/self.analysis['fwhm']
            self.analysis['dip_depth'] = 0


    def update_markers(self, frequencies, log_magnitude, filtered_data, is_selected, axes_dict):
        # Determine marker color and style based on save status
        if self.save:
            color = 'green' if is_selected else 'red'
            marker_style = '.'
            id_text = str(self.id).zfill(4) if self.id is not None else ""
        else:
            color = 'black'
            marker_style = '.'
            id_text = ''  # Indicate not saved

        # Calculate a local peak index if self.peak_idx is None
        if self.peak_idx is not None:
            peak_idx = self.peak_idx
        else:
            peak_idx = np.abs(self.frequency - frequencies).argmin()

        # Extract data points
        freq_mhz = frequencies[peak_idx] / 1e6
        mag = log_magnitude[peak_idx]
        filt = filtered_data[peak_idx]

        # Modify marker style based on selection
        if is_selected:
            marker_style = 'D'  # Diamond for selected
            # color = 'red'
        else:
            marker_style = '.'
            # color = 'blue'

        # Update or create markers for each plot
        self._update_marker('raw', freq_mhz, mag, color, marker_style, id_text, axes_dict['raw_ax'])
        self._update_marker('filt', freq_mhz, filt, color, marker_style, id_text, axes_dict['filt_ax'])
        self._update_marker('zoom_raw', freq_mhz, mag, color, marker_style, id_text, axes_dict['zoom_raw_ax'])
        self._update_marker('zoom_filt', freq_mhz, filt, color, marker_style, id_text, axes_dict['zoom_filt_ax'])

    def _update_marker(self, key, x, y, color, marker_style, text, ax):
        text=text+' '
        if y is None:
            return  # Cannot plot without y data
        
        if key not in self.markers:
            # Create new marker and text
            marker_line, = ax.plot([x], [y], marker=marker_style, color=color, linestyle='None',picker=10)
            marker_text = ax.text(x, y, text, color=color, rotation=90, ha='center', va='top',picker=10)
            self.markers[key] = {'line': marker_line, 'text': marker_text}
        else:
            # Update existing marker and text
            self.markers[key]['line'].set_data([x],[y])
            self.markers[key]['line'].set_color(color)
            self.markers[key]['line'].set_marker(marker_style)
            self.markers[key]['line'].set_picker(10)
            
            self.markers[key]['text'].set_position((x, y))
            self.markers[key]['text'].set_color(color)
            self.markers[key]['text'].set_text(text)
            self.markers[key]['text'].set_picker(10)
            

    def remove_markers(self):
        for marker_dict in self.markers.values():
            line = marker_dict.get('line')
            text = marker_dict.get('text')
            try:
                if line:
                    line.remove()
                if text:
                    text.remove()
            except ValueError as e:
                print(f"Marker already removed: {e}")
        self.markers.clear()



class ResonanceFinder(QMainWindow):
    """
    An interactive MKID resonance finder application built with PyQt5.
    """
    def __init__(self):
        super().__init__()

        self.frequencies = np.zeros(16)
        self.frequency_stepsize = 1
        self.s21_complex = np.zeros(16,dtype='complex')
        self.magnitude = np.zeros(16)
        self.log_magnitude = np.zeros(16)
        self.phase = np.zeros(16)
        self.unwrap_phase = np.zeros(16)
        self.group_delay_us = np.zeros(16)
        self.complex_gradient = np.zeros(16)
        self.filtered_data = np.zeros(16)
        self.resonances = []
        self.current_resonance_index = None
        self.rectangle_selectors = {}  # Dictionary to store selectors per axes
        self.analysis_quantities = ['Lin Magnitude V','Log Magnitude dB', 'Phase rad', 'Unwrapped Phase rad',
            'Group Delay us (-dphi/df)', 'Complex Gradient V/Hz (speed)']
        self.current_quantity = 'Log Magnitude dB'
        
        self.peak_finder_labels = {
            'Lin Magnitude V':                 {'prominence': 'Prominence (dip depth) [V]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [V]', 'threshold': 'Threshold (?) [V]'},
            'Log Magnitude dB':                {'prominence': 'Prominence (dip depth) [dB]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [dB]', 'threshold': 'Threshold (?) [dB]'},
            'Phase rad':                       {'prominence': 'Prominence (peak depth) [rad]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [rad]', 'threshold': 'Threshold (?) [rad]'},
            'Unwrapped Phase rad':             {'prominence': 'Prominence (peak depth) [rad]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [rad]', 'threshold': 'Threshold (?) [rad]'},
            'Group Delay us (-dphi/df)':       {'prominence': 'Prominence (peak height) [us]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [us]', 'threshold': 'Threshold (?) [us]'},
            'Complex Gradient V/Hz (speed)':   {'prominence': 'Prominence (peak height) [V/Hz]', 'width': 'Width (FWHM) [Hz]','distance': 'Distance (spacing) [Hz]', 'height': 'Height (?) [V/Hz]', 'threshold': 'Threshold (?) [V/Hz]'}
        }
        self.peak_finder_params = {}
        
        #prevent recursion errors
        self.is_updating_save_state = False
        self.is_loading_settings = False

        self.settings = QSettings("souk", "ResonanceFinder")

        self.initUI()
        self.initPlots()
        self.loadSettings()
        self.updatePeaks()

        self.toggle_save_action = QAction("Toggle Save", self)
        self.toggle_save_action.setShortcut(Qt.Key_Space)
        self.toggle_save_action.triggered.connect(self.toggleSaveSelectedResonances)
        self.addAction(self.toggle_save_action)
        
        



    def closeEvent(self, event):
        # Save window geometry and state
        self.settings.setValue("geometry", self.saveGeometry())
        self.settings.setValue("windowState", self.saveState())
        
        # Save other widget states here
        self.saveWidgetStates()

        # Save current peak finderparameters
        self.savePeakFinderParameters(self.current_quantity)
        # Save all parameters to QSettings
        self.saveAllPeakFinderParameters()
        
        self.saveFilterParameters()

        super().closeEvent(event)  # Ensure the base class method is called

    def saveWidgetStates(self):
        # Save splitter position
        self.settings.setValue("splitterSizes", self.hsplitter.sizes())

    def loadAllPeakFinderParameters(self):
        for quantity in self.analysis_quantities:
            self.settings.beginGroup(f'PeakFinder/{quantity}')
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
            }
            self.settings.endGroup()
            self.peak_finder_params[quantity] = params

    def saveAllPeakFinderParameters(self):
        for quantity, params in self.peak_finder_params.items():
            self.settings.beginGroup(f'PeakFinder/{quantity}')
            for key, value in params.items():
                self.settings.setValue(key, value)
            self.settings.endGroup()

    def loadPeakFinderParameters(self, quantity):
        params = self.peak_finder_params.get(quantity, {})

        was_loading_settings = self.is_loading_settings
        self.is_loading_settings = True
        
        # Load parameters into UI controls
        if quantity=="Log Magnitude dB":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 100.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 100.0))
            self.width_max_spin.setValue(params.get('width_max', 1000000.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', False))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        elif quantity=="Lin Magnitude V":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 0.1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 10.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 0.1))
            self.width_max_spin.setValue(params.get('width_max', 100.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', True))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        elif quantity=="Phase rad":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 0.1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 10.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 0.1))
            self.width_max_spin.setValue(params.get('width_max', 100.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', True))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        elif quantity=="Unwrapped Phase rad":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 0.1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 10.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 0.1))
            self.width_max_spin.setValue(params.get('width_max', 100.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', True))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        elif quantity=="Group Delay us (-dphi/df)":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 0.1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 10.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 0.1))
            self.width_max_spin.setValue(params.get('width_max', 100.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', True))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        elif quantity=="Complex Gradient V/Hz (speed)":
            self.prominence_checkbox.setChecked(params.get('prominence_enabled', True))
            self.prominence_min_spin.setValue(params.get('prominence_min', 0.1))
            self.prominence_max_spin.setValue(params.get('prominence_max', 10.0))
            self.width_checkbox.setChecked(params.get('width_enabled', True))
            self.width_min_spin.setValue(params.get('width_min', 0.1))
            self.width_max_spin.setValue(params.get('width_max', 100.0))
            self.threshold_checkbox.setChecked(params.get('threshold_enabled', True))
            self.threshold_min_spin.setValue(params.get('threshold_min', 0.0))
            self.threshold_max_spin.setValue(params.get('threshold_max', 1.0))
            self.height_checkbox.setChecked(params.get('height_enabled', False))
            self.height_min_spin.setValue(params.get('height_min', 0.0))
            self.height_max_spin.setValue(params.get('height_max', 1.0))
            self.distance_checkbox.setChecked(params.get('distance_enabled', True))
            self.distance_min_spin.setValue(params.get('distance_value', 1000))
        self.is_loading_settings = was_loading_settings
        
    def savePeakFinderParameters(self, quantity):
        params = {
            'prominence_enabled': self.prominence_checkbox.isChecked(),
            'prominence_min': self.prominence_min_spin.value(),
            'prominence_max': self.prominence_max_spin.value(),
            'width_enabled': self.width_checkbox.isChecked(),
            'width_min': self.width_min_spin.value(),
            'width_max': self.width_max_spin.value(),
            'threshold_enabled': self.threshold_checkbox.isChecked(),
            'threshold_min': self.threshold_min_spin.value(),
            'threshold_max': self.threshold_max_spin.value(),
            'height_enabled': self.height_checkbox.isChecked(),
            'height_min': self.height_min_spin.value(),
            'height_max': self.height_max_spin.value(),
            'distance_enabled': self.distance_checkbox.isChecked(),
            'distance_value': self.distance_min_spin.value(),
        }
        # Update the dictionary
        self.peak_finder_params[quantity] = params

    def onPeakFinderParameterChanged(self):
        # Save current parameters into the dictionary
        self.savePeakFinderParameters(self.current_quantity)
        # Optionally, update peaks immediately
        if not self.is_loading_settings:
            self.updatePeaks()
        

    def saveFilterParameters(self):
        self.settings.beginGroup('FilterParameters')
        self.settings.setValue('quantity', self.analysis_quantity_combo.currentText())
        self.settings.setValue('highpass_cutoff', self.highpass_spin.value())
        self.settings.setValue('lowpass_cutoff', self.lowpass_spin.value())
        self.settings.setValue('median_kernel', self.median_kernel_spin.value())
        # Save other filter parameters as needed
        self.settings.endGroup()

    def loadFilterParameters(self):
        self.settings.beginGroup('FilterParameters')
        quantity = self.settings.value('quantity', 'Log Magnitude dB', type=str)
        self.analysis_quantity_combo.setCurrentText(quantity)
        highpass_cutoff = self.settings.value('highpass_cutoff', 0.001, type=float)
        self.highpass_spin.setValue(highpass_cutoff)
        lowpass_cutoff = self.settings.value('lowpass_cutoff', 0.75, type=float)
        self.lowpass_spin.setValue(lowpass_cutoff)
        median_kernel = self.settings.value('median_kernel', 1, type=int)
        self.median_kernel_spin.setValue(median_kernel)
        # Load other filter parameters as needed
        self.settings.endGroup()

    def onFilterParameterChanged(self):
        self.saveFilterParameters()
        self.loadPeakFinderParameters(self.current_quantity)
        self.updateFilter()


    def initUI(self):
        self.setWindowTitle("MKID Resonance Finder")

        # Main widget
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)

        # Open button and label
        self.open_button = QPushButton("Open")
        self.open_button.clicked.connect(self.openFile)
        self.file_label = QLabel("No file loaded")
        self.file_label.setTextInteractionFlags(Qt.TextSelectableByMouse)

        open_hbox = QHBoxLayout()
        open_hbox.addWidget(self.open_button)
        open_hbox.addWidget(self.file_label)
        open_hbox.addStretch()

        # Splitter between left and right vboxes
        self.hsplitter = QSplitter(Qt.Horizontal)

        # Left VBox
        left_vbox = QVBoxLayout()
        left_widget = QWidget()
        left_widget.setLayout(left_vbox)

        # Right VBox
        right_vbox = QVBoxLayout()
        right_widget = QWidget()
        right_widget.setLayout(right_vbox)

        # Add widgets to splitter
        self.hsplitter.addWidget(left_widget)
        self.hsplitter.addWidget(right_widget)
        self.hsplitter.setSizes([(self.width() *2)//3, (self.width() *1)//3 ])
        self.hsplitter.setCollapsible(0, False)
        self.hsplitter.setCollapsible(1, False)

        # Left VBox contents
        self.initLeftVBox(left_vbox)

        # Right VBox contents
        self.initRightVBox(right_vbox)

        # Main layout
        main_vbox = QVBoxLayout()
        main_vbox.addLayout(open_hbox)
        main_vbox.addWidget(self.hsplitter)
        self.main_widget.setLayout(main_vbox)

        # Initialize timers for debounce a resize
        self.resize_timer = QTimer(self)
        self.resize_timer.setSingleShot(True)
        self.resize_timer.timeout.connect(self.onResizeFinished)

        self.hsplitter_timer = QTimer(self)
        self.hsplitter_timer.setSingleShot(True)
        self.hsplitter_timer.timeout.connect(self.onSplitterMoveFinished)

        self.resize(1200, 800)

        self.updateNavigationButtons()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        # Restart the timer every time a resize event occurs
        self.resize_timer.start(200)  # 200 milliseconds debounce interval

    def onSplitterMoved(self, pos, index):
        # Restart the timer every time the splitter is moved
        self.hsplitter_timer.start(200)  # 200 milliseconds debounce interval

    def onResizeFinished(self):
        """
        Called once the window resizing has finished.
        Perform any layout updates or redraws here.
        """
        self.refreshFigures()

    def onSplitterMoveFinished(self):
        """
        Called once the splitter has finished moving.
        Perform any layout updates or redraws here.
        """
        self.refreshFigures()


    def initLeftVBox(self, vbox):
        # Raw magnitude sweep groupbox
        raw_groupbox = QGroupBox("Raw magnitude sweep")
        raw_layout = QVBoxLayout()
        self.raw_figure = plt.figure()
        self.raw_canvas = FigureCanvas(self.raw_figure)
        raw_layout.addWidget(self.raw_canvas)
        self.raw_toolbar = NavigationToolbar(self.raw_canvas, self)
        raw_layout.addWidget(self.raw_toolbar)
        raw_groupbox.setLayout(raw_layout)
        
        # Filtered sweep groupbox
        filtered_groupbox = QGroupBox("Filtered sweep")
        filtered_layout = QVBoxLayout()
        self.filtered_figure = plt.figure()
        self.filtered_canvas = FigureCanvas(self.filtered_figure)
        filtered_layout.addWidget(self.filtered_canvas)
        self.filtered_toolbar = NavigationToolbar(self.filtered_canvas, self)
        filtered_layout.addWidget(self.filtered_toolbar)
        filtered_groupbox.setLayout(filtered_layout)

        # Resonance viewer groupbox
        resonance_groupbox = QGroupBox("Resonance viewer")
        resonance_layout = QHBoxLayout()
        resonance_groupbox.setLayout(resonance_layout)

        # Zoom plots and analysis results
        self.initResonanceViewer(resonance_layout)


        # Selection Mode Checkbox
        self.selection_mode_checkbox = QCheckBox("Enable click and drag in figure for multiple selection (pan and zoom buttons must be disabled)")
        self.selection_mode_checkbox.stateChanged.connect(self.toggleSelectionMode)

        # Add groupboxes to vbox
        vbox.addWidget(self.selection_mode_checkbox)
        vbox.addWidget(raw_groupbox)
        vbox.addWidget(filtered_groupbox)
        vbox.addWidget(resonance_groupbox)

    def initResonanceViewer(self, layout):
        # Left zoom plot (magnitude)
        zoom_raw_layout = QVBoxLayout()
        self.zoom_raw_figure = plt.figure()
        self.zoom_raw_canvas = FigureCanvas(self.zoom_raw_figure)
        zoom_raw_layout.addWidget(self.zoom_raw_canvas)
        zoom_raw_widget = QWidget()
        self.zoom_raw_toolbar = NavigationToolbar(self.zoom_raw_canvas, self)
        zoom_raw_layout.addWidget(self.zoom_raw_toolbar)
        zoom_raw_widget.setLayout(zoom_raw_layout)

        # Right zoom plot (filtered)
        zoom_filt_layout = QVBoxLayout()
        self.zoom_filt_figure = plt.figure()
        self.zoom_filt_canvas = FigureCanvas(self.zoom_filt_figure)
        zoom_filt_layout.addWidget(self.zoom_filt_canvas)
        zoom_filt_widget = QWidget()
        self.zoom_filt_toolbar = NavigationToolbar(self.zoom_filt_canvas, self)
        zoom_filt_layout.addWidget(self.zoom_filt_toolbar)
        zoom_filt_widget.setLayout(zoom_filt_layout)

        # Analysis results and navigation buttons
        analysis_layout = QVBoxLayout()
        self.analysis_text = QLabel("Analysis Results")
        analysis_layout.addWidget(self.analysis_text)

        # Navigation buttons
        nav_layout = QHBoxLayout()
        self.prev_button = QPushButton("Prev")
        self.next_button = QPushButton("Next")
        self.prev_button.clicked.connect(self.prevResonance)
        self.next_button.clicked.connect(self.nextResonance)
        nav_layout.addWidget(self.prev_button)
        nav_layout.addWidget(self.next_button)
        analysis_layout.addLayout(nav_layout)

        self.toggle_save_button = QPushButton("Toggle Save Selected")
        self.toggle_save_button.clicked.connect(self.toggleSaveSelectedResonances)
        analysis_layout.addWidget(self.toggle_save_button)

        btn_layout = QHBoxLayout()
        self.add_button = QPushButton("Add")
        self.edit_button = QPushButton("Edit")
        self.delete_button = QPushButton("Delete Selected")
        self.add_button.clicked.connect(self.addResonance)
        self.edit_button.clicked.connect(self.editResonance)
        self.delete_button.clicked.connect(self.deleteResonance)
        btn_layout.addWidget(self.add_button)
        btn_layout.addWidget(self.edit_button)
        btn_layout.addWidget(self.delete_button)
        analysis_layout.addLayout(btn_layout)

        analysis_widget = QWidget()
        analysis_widget.setLayout(analysis_layout)

        # Add to main resonance viewer layout
        layout.addWidget(zoom_raw_widget)
        layout.addWidget(zoom_filt_widget)
        layout.addWidget(analysis_widget)

    def initRightVBox(self, vbox):
        # Filter parameters groupbox with labels to the left
        filter_groupbox = QGroupBox("Filter Parameters")
        filter_layout = QFormLayout()  # Use QFormLayout instead of QVBoxLayout

        # Analysis quantity combobox
        self.analysis_quantity_combo = QComboBox()
        self.analysis_quantity_combo.addItems([
            "Lin Magnitude V","Log Magnitude dB", "Phase rad", "Unwrapped Phase rad",
            "Group Delay us (-dphi/df)", "Complex Gradient V/Hz (speed)"
        ])
        self.analysis_quantity_combo.setCurrentIndex(1)
        self.analysis_quantity_combo.currentIndexChanged.connect(self.updateQuantity)
        filter_layout.addRow("Analysis Quantity:", self.analysis_quantity_combo)

        # Highpass filter cutoff
        self.highpass_spin = QDoubleSpinBox()
        self.highpass_spin.setStepType(StepType)
        self.highpass_spin.setDecimals(6)
        self.highpass_spin.setRange(0.0, 1.0)
        self.highpass_spin.setValue(0.001)  
        self.highpass_spin.valueChanged.connect(self.filterChanged)
        self.highpass_spin.editingFinished.connect(self.filterEdited)

        filter_layout.addRow("Highpass Cutoff (Nyq=1):", self.highpass_spin)

        # Lowpass filter cutoff
        self.lowpass_spin = QDoubleSpinBox()
        self.lowpass_spin.setStepType(StepType)
        self.lowpass_spin.setDecimals(6)
        self.lowpass_spin.setRange(0.0, 1.0)
        self.lowpass_spin.setValue(0.75)
        self.lowpass_spin.valueChanged.connect(self.filterChanged)
        self.lowpass_spin.editingFinished.connect(self.filterEdited)
        filter_layout.addRow("Lowpass Cutoff (Nyq=1):", self.lowpass_spin)

        # Median filter kernel size
        self.median_kernel_spin = QSpinBox()
        self.median_kernel_spin.setMinimum(1)
        self.median_kernel_spin.setSingleStep(2)
        self.median_kernel_spin.setValue(1)  # Default odd value
        self.median_kernel_spin.valueChanged.connect(self.ensureOddMedianKernel)
        self.median_kernel_spin.valueChanged.connect(self.filterChanged)
        self.median_kernel_spin.editingFinished.connect(self.filterEdited)
        
        filter_layout.addRow("Median Kernel Size:", self.median_kernel_spin)

        filter_groupbox.setLayout(filter_layout)

        # Peak Finder Parameters groupbox with labels to the left
        
        peak_finder_group = QGroupBox("Peak Finder Parameters")
        peak_finder_layout = QGridLayout()
        peak_finder_group.setLayout(peak_finder_layout)


        # Prominence
        row = 0
        self.prominence_checkbox = QCheckBox()
        self.prominence_checkbox.setChecked(True) 
        self.prominence_checkbox.stateChanged.connect(self.finderEdited) 

        self.prominence_min_spin = QDoubleSpinBox()
        self.prominence_min_spin.setStepType(StepType)
        self.prominence_min_spin.setDecimals(6)
        self.prominence_min_spin.setRange(-1000, 1000.0)  
        self.prominence_min_spin.setValue(1.0)  
        self.prominence_min_spin.valueChanged.connect(self.finderChanged)

        self.prominence_min_spin.editingFinished.connect(self.finderEdited)
        self.prominence_max_spin = QDoubleSpinBox()
        self.prominence_max_spin.setStepType(StepType)
        self.prominence_max_spin.setDecimals(6)
        self.prominence_max_spin.setRange(-1000, 1000.0)  
        self.prominence_max_spin.setValue(100)  
        self.prominence_max_spin.valueChanged.connect(self.finderChanged)
        self.prominence_max_spin.editingFinished.connect(self.finderEdited)

        self.peak_finder_prominence_label = QLabel("Prominence")
        peak_finder_layout.addWidget(self.peak_finder_prominence_label,row,0)
        peak_finder_layout.addWidget(self.prominence_checkbox,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.prominence_min_spin,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.prominence_max_spin,row,5)
        # peak_finder_layout.addWidget(QLabel('(Dip depth)'),row,6)
        
        # Width
        row += 1
        self.width_checkbox = QCheckBox()
        self.width_checkbox.setChecked(True) 
        self.width_checkbox.stateChanged.connect(self.finderEdited) 

        self.width_min_spin = QDoubleSpinBox()
        self.width_min_spin.setStepType(StepType)
        self.width_min_spin.setDecimals(1)
        self.width_min_spin.setRange(1.0, 1e9)  
        self.width_min_spin.setValue(100)  
        self.width_min_spin.valueChanged.connect(self.finderChanged)
        self.width_min_spin.editingFinished.connect(self.finderEdited)

        self.width_max_spin = QDoubleSpinBox()
        self.width_max_spin.setStepType(StepType)
        self.width_max_spin.setDecimals(1)
        self.width_max_spin.setRange(1.0, 1e9)  
        self.width_max_spin.setValue(10e6)  
        self.width_max_spin.valueChanged.connect(self.finderChanged)
        self.width_max_spin.editingFinished.connect(self.finderEdited)

        self.peak_finder_width_label = QLabel("Width")
        peak_finder_layout.addWidget(self.peak_finder_width_label,row,0)
        peak_finder_layout.addWidget(self.width_checkbox,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.width_min_spin,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.width_max_spin,row,5)
        # peak_finder_layout.addWidget(QLabel("(FWHM)"),row,6)

        # Distance
        row += 1
        self.distance_checkbox = QCheckBox()
        self.distance_checkbox.setChecked(True)  
        self.distance_checkbox.stateChanged.connect(self.finderEdited)  

        self.distance_min_spin = QDoubleSpinBox()
        self.distance_min_spin.setStepType(StepType)
        self.distance_min_spin.setDecimals(1)
        self.distance_min_spin.setRange(0.0, 1e9)  
        self.distance_min_spin.setValue(1e3)  
        self.distance_min_spin.valueChanged.connect(self.finderChanged)
        self.distance_min_spin.editingFinished.connect(self.finderEdited)

        self.peak_finder_distance_label = QLabel("Distance")
        peak_finder_layout.addWidget(self.peak_finder_distance_label,row,0)      
        peak_finder_layout.addWidget(self.distance_checkbox,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.distance_min_spin,row,3)
        # peak_finder_layout.addWidget(QLabel("(Spacing)"),row,4,1,3)
        
        # Height
        row += 1
        self.height_checkbox = QCheckBox()
        self.height_checkbox.setChecked(False)
        self.height_checkbox.stateChanged.connect(self.finderEdited) 

        self.height_min_spin = QDoubleSpinBox()
        self.height_min_spin.setStepType(StepType)
        self.height_min_spin.setDecimals(6)
        self.height_min_spin.setRange(0.0, 1000.0)
        self.height_min_spin.valueChanged.connect(self.finderChanged)
        self.height_min_spin.editingFinished.connect(self.finderEdited)

        self.height_max_spin = QDoubleSpinBox()
        self.height_max_spin.setStepType(StepType)
        self.height_max_spin.setDecimals(6)
        self.height_max_spin.setRange(0.0, 1000.0)
        self.height_max_spin.valueChanged.connect(self.finderChanged)
        self.height_max_spin.editingFinished.connect(self.finderEdited)

        self.peak_finder_height_label = QLabel("Height")
        peak_finder_layout.addWidget(self.peak_finder_height_label,row,0)
        peak_finder_layout.addWidget(self.height_checkbox,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.height_min_spin,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.height_max_spin,row,5)
        # peak_finder_layout.addWidget(QLabel("(Untested)"),row,6)

        # Threshold
        row += 1
        self.threshold_checkbox = QCheckBox()
        self.threshold_checkbox.setChecked(False)
        self.threshold_checkbox.stateChanged.connect(self.finderEdited)

        self.threshold_min_spin = QDoubleSpinBox()
        self.threshold_min_spin.setStepType(StepType)
        self.threshold_min_spin.setDecimals(3)
        self.threshold_min_spin.setRange(0.0, 1000.0)  
        self.threshold_min_spin.valueChanged.connect(self.finderChanged)
        self.threshold_min_spin.editingFinished.connect(self.finderEdited)
        
        self.threshold_max_spin = QDoubleSpinBox()
        self.threshold_max_spin.setStepType(StepType)
        self.threshold_max_spin.setDecimals(3)
        self.threshold_max_spin.setRange(0.0, 1000.0)  
        self.threshold_max_spin.valueChanged.connect(self.finderChanged)
        self.threshold_max_spin.editingFinished.connect(self.finderEdited)
        
        self.peak_finder_threshold_label = QLabel("Threshold")
        peak_finder_layout.addWidget(self.peak_finder_threshold_label,row,0)
        peak_finder_layout.addWidget(self.threshold_checkbox,row,1)
        peak_finder_layout.addWidget(QLabel("Min"),row,2)
        peak_finder_layout.addWidget(self.threshold_min_spin,row,3)
        peak_finder_layout.addWidget(QLabel("Max"),row,4)
        peak_finder_layout.addWidget(self.threshold_max_spin)
        # peak_finder_layout.addWidget(QLabel("(Untested)"),row,6)
        
        # Resonances List groupbox
        resonances_groupbox = QGroupBox("Resonances List")
        resonances_layout = QVBoxLayout()
        self.resonances_table = QTableWidget()
        self.resonances_table.setColumnCount(6)
        self.resonances_table.setHorizontalHeaderLabels([
            "Save", "ID", "Frequency (MHz)", "Q-factor", "FWHM (kHz)", "Depth (dB)"])
        self.resonances_table.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.resonances_table.horizontalHeader().setSectionResizeMode(0, QHeaderView.ResizeToContents)
        self.resonances_table.verticalHeader().setVisible(False)
        self.resonances_table.setSelectionBehavior(QTableWidget.SelectRows)
        self.resonances_table.setSelectionMode(QTableWidget.ExtendedSelection)
        self.resonances_table.resizeColumnsToContents()

        self.resonances_table.itemSelectionChanged.connect(self.resonanceSelectionChanged)
        self.resonances_table.setContextMenuPolicy(Qt.CustomContextMenu)
        self.resonances_table.customContextMenuRequested.connect(self.showResonancesTableContextMenu)
        
        # Install event filter to capture double-click events
        # self.resonances_table.viewport().installEventFilter(self)

        # Number of resonances label
        self.num_resonances_label = QLabel("Resonances found: 0")
        # Save/Export button
        self.save_button = QPushButton("Save/Export")
        self.save_button.clicked.connect(self.saveResonances)

        resonances_layout.addWidget(self.resonances_table)
        resonances_layout.addWidget(self.num_resonances_label)
        resonances_layout.addWidget(self.save_button)
        resonances_groupbox.setLayout(resonances_layout)


        # Save/Export button
        self.save_button = QPushButton("Save/Export")
        self.save_button.clicked.connect(self.saveResonances)

        # Add widgets to right vbox
        vbox.addWidget(filter_groupbox)
        vbox.addWidget(peak_finder_group)
        vbox.addWidget(resonances_groupbox)


    def toggleSelectionMode(self, state):
        if state == Qt.Checked:
            self.activateRectangleSelectors()
        else:
            self.deactivateRectangleSelectors()

    def activateRectangleSelectors(self):
        axes_list = [self.raw_ax, self.filt_ax, self.zoom_raw_ax, self.zoom_filt_ax]  # List of axes to enable selection on
        for ax in axes_list:
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
        self.raw_canvas.setCursor(Qt.CrossCursor)
        self.filtered_canvas.setCursor(Qt.CrossCursor)
        self.zoom_raw_canvas.setCursor(Qt.CrossCursor)
        self.zoom_filt_canvas.setCursor(Qt.CrossCursor)
        
    def deactivateRectangleSelectors(self):
        for selector in self.rectangle_selectors.values():
            selector.set_active(False)
        # Reset cursors
        self.raw_canvas.setCursor(Qt.ArrowCursor)
        self.filtered_canvas.setCursor(Qt.ArrowCursor)
        self.zoom_raw_canvas.setCursor(Qt.ArrowCursor)
        self.zoom_filt_canvas.setCursor(Qt.ArrowCursor)

        # # Clear selection in the table
        # self.resonances_table.clearSelection()
        # # Set current resonance index to None
        # self.setCurrentResonanceIndex(None)
        # # Update markers to reflect the cleared selection
        # self.updateMarkers()

    def onSelectRectangle(self, eclick, erelease):
        ax = eclick.inaxes
        if ax is None:
            return
        self.handleRectangleSelection(ax, eclick, erelease)


    def handleRectangleSelection(self, ax, eclick, erelease):
        # Get rectangle coordinates in data units
        x_min, x_max = sorted([eclick.xdata, erelease.xdata])
        y_min, y_max = sorted([eclick.ydata, erelease.ydata])

        # Determine which data to use based on the axes
        if ax == self.raw_ax or ax == self.zoom_raw_ax:
            x_data = self.frequencies / 1e6  # Convert to MHz
            y_data = self.log_magnitude
        elif ax == self.filt_ax or ax == self.zoom_filt_ax:
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
        
        # Update the selection in the table using QItemSelection
        selection_model = self.resonances_table.selectionModel()
        selection = QItemSelection()
        for idx in selected_indices:
            index = self.resonances_table.model().index(idx, 0)
            selection.select(index, index)
        # Apply the new selection
        selection_model.select(selection, QItemSelectionModel.ClearAndSelect | QItemSelectionModel.Rows)


    # def eventFilter(self, obj, event):
    #     if obj == self.resonances_table.viewport():
    #         if event.type() == QEvent.MouseButtonDblClick:
    #             if event.button() == Qt.LeftButton:
    #                 # Get the position of the mouse event
    #                 pos = event.pos()
    #                 index = self.resonances_table.indexAt(pos)
    #                 if index.isValid():
    #                     # Valid row, perform edit action
    #                     row = index.row()
    #                     self.setCurrentResonanceIndex(row)
    #                     self.editResonance()
    #                 else:
    #                     # No valid row, perform add action with default frequency
    #                     self.addResonance()
    #                 return True  # Event handled
    #         if event.type() == QEvent.KeyPress:
    #             if event.key() == Qt.Key_Delete:
    #                 self.deleteResonance()
    #                 return True
    #             if event.key() == Qt.Key_Left:
    #                 self.prevResonance()
    #                 return True
    #             if event.key() == Qt.Key_Right:
    #                 self.nextResonance()
    #                 return True
    #     return QMainWindow.eventFilter(self, obj, event)


    def ensureOddMedianKernel(self):
        value = self.median_kernel_spin.value()
        if value % 2 == 0:
            # Temporarily block signals to prevent recursive calls
            self.median_kernel_spin.blockSignals(True)
            self.median_kernel_spin.setValue(value + 1)
            self.median_kernel_spin.blockSignals(False)
            # Optionally, inform the user
            QMessageBox.information(
                self, "Odd Kernel Size Required",
                f"Median Kernel Size was even and has been adjusted to {value + 1}."
            )

    def filterChanged(self):
        self.onFilterParameterChanged()
        self.updateFilter()
        #self.updatePeaks()
        #self.updateResonancesTable()
        #self.updateMarkers()
    
    def filterEdited(self):
        self.updateFilter()
        self.updatePeaks()
        self.updateResonancesTable()
        self.updateMarkers()
        self.updateNavigationButtons()
    
    def finderChanged(self):
        self.onPeakFinderParameterChanged()
        # self.updatePeaks()
        # self.updateResonancesTable()
        # self.updateMarkers()
        pass
   
    def finderEdited(self):
        self.updatePeaks()
        self.updateResonancesTable()
        self.updateMarkers()
        self.updateNavigationButtons()
        self.savePeakFinderParameters(self.current_quantity)
        self.saveAllPeakFinderParameters()
        
    def initPlots(self):
        self.raw_ax = self.raw_figure.add_subplot(111)
        self.raw_line, = self.raw_ax.plot([],[])
        self.raw_ax.set_xlabel("Frequency (MHz)")
        self.raw_ax.set_ylabel("Magnitude (dB)")
        self.raw_figure.tight_layout()
        self.raw_canvas.draw_idle()

        self.filt_ax = self.filtered_figure.add_subplot(111,sharex=self.raw_ax)
        self.filt_line, = self.filt_ax.plot([],[])
        self.filt_ax.set_xlabel("Frequency (MHz)")
        self.filt_ax.set_ylabel(self.analysis_quantity_combo.currentText())
        self.filtered_figure.tight_layout()
        self.filtered_canvas.draw_idle()

        self.zoom_raw_ax = self.zoom_raw_figure.add_subplot(111)
        self.zoom_raw_line, = self.zoom_raw_ax.plot([],[])
        self.zoom_raw_ax.set_xlabel("Frequency (MHz)")
        self.zoom_raw_ax.set_ylabel("Magnitude (dB)")
        self.zoom_raw_figure.tight_layout()
        self.zoom_raw_canvas.draw_idle()

        self.zoom_filt_ax = self.zoom_filt_figure.add_subplot(111)
        self.zoom_filt_line, = self.zoom_filt_ax.plot([],[])
        self.zoom_filt_ax.set_xlabel("Frequency (MHz)")
        self.zoom_filt_ax.set_ylabel(self.analysis_quantity_combo.currentText())
        self.zoom_filt_figure.tight_layout()
        self.zoom_filt_canvas.draw_idle()

        self.raw_canvas.mpl_connect('pick_event', self.onPick)
        self.filtered_canvas.mpl_connect('pick_event', self.onPick)
        self.zoom_raw_canvas.mpl_connect('pick_event', self.onPick)
        self.zoom_filt_canvas.mpl_connect('pick_event', self.onPick)

        self.raw_canvas.mpl_connect('button_press_event', self.onAxesRightClick)
        self.filtered_canvas.mpl_connect('button_press_event', self.onAxesRightClick)
        self.zoom_raw_canvas.mpl_connect('button_press_event', self.onAxesRightClick)
        self.zoom_filt_canvas.mpl_connect('button_press_event', self.onAxesRightClick)


        self.axes_dict = {
            'raw_ax': self.raw_ax,
            'filt_ax': self.filt_ax,
            'zoom_raw_ax': self.zoom_raw_ax,
            'zoom_filt_ax': self.zoom_filt_ax
        }

    def onPick(self, event):
        artist = event.artist
        for idx, resonance in enumerate(self.resonances):
            for marker_dict in resonance.markers.values():
                line = marker_dict.get('line')
                text = marker_dict.get('text')
                if artist == line or artist == text:
                    # Found the resonance
                    # Select this resonance
                    self.resonances_table.blockSignals(True)
                    self.resonances_table.clearSelection()
                    self.resonances_table.selectRow(idx)
                    self.resonances_table.blockSignals(False)
                    # Scroll the table to make the row visible
                    self.resonances_table.scrollToItem(self.resonances_table.item(idx, 0))
                    # Update the current resonance index
                    self.setCurrentResonanceIndex(idx)
                    return

    def onAxesRightClick(self, event):
        if event.button != 3:  # Right mouse button
            return
        if event.inaxes is None:
            return

        # Determine if click is near a marker or text
        picked_resonance_idx = None
        min_distance = float('inf')
        pick_tolerance = 30  # pixels

        for idx, resonance in enumerate(self.resonances):
            for marker_dict in resonance.markers.values():
                line = marker_dict.get('line')
                text = marker_dict.get('text')
                for artist in [line, text]:
                    if artist is not None and artist.axes == event.inaxes:
                        if isinstance(artist, matplotlib.lines.Line2D):
                            xdata, ydata = artist.get_data()
                            xdisp, ydisp = artist.axes.transData.transform((xdata[0], ydata[0]))
                        elif isinstance(artist, matplotlib.text.Text):
                            xdata, ydata = artist.get_position()
                            xdisp, ydisp = artist.axes.transData.transform((xdata, ydata))
                        else:
                            continue
                        distance = np.hypot(event.x - xdisp, event.y - ydisp)
                        if distance < pick_tolerance and distance < min_distance:
                            min_distance = distance
                            picked_resonance_idx = idx

        # Create context menu
        menu = QMenu()
        add_action = QAction("Add Resonance", self)
        edit_action = QAction("Edit Resonance", self)
        delete_action = QAction("Delete Resonance", self)
        toggle_save_action = QAction("Toggle Save Resonance", self)

        # Connect actions to the appropriate methods
        add_action.triggered.connect(lambda: self.addResonanceAtPlotPosition(event))
        edit_action.triggered.connect(lambda: self.editResonanceAtPlotPosition(picked_resonance_idx))
        delete_action.triggered.connect(lambda: self.deleteResonanceAtPosition(picked_resonance_idx))
        toggle_save_action.triggered.connect(lambda: self.toggleSaveResonanceAtPosition(picked_resonance_idx))

        menu.addAction(add_action)
        if picked_resonance_idx is not None:
            menu.addAction(edit_action)
            menu.addAction(delete_action)
            menu.addAction(toggle_save_action)
        else:
            edit_action.setEnabled(False)
            delete_action.setEnabled(False)
            toggle_save_action.setEnabled(False)

        # Show the context menu at the mouse position
        menu.exec_(QtGui.QCursor.pos())


    def addResonanceAtPlotPosition(self, event):
        ax = event.inaxes
        if ax is None or event.xdata is None:
            return
        freq_mhz = event.xdata
        frequency = freq_mhz * 1e6
        self.addResonance(initial_freq_mhz=freq_mhz)

    def editResonanceAtPlotPosition(self, resonance_idx):
        if resonance_idx is None:
            QMessageBox.information(self, "Edit Resonance", "No resonance selected.")
            return
        self.setCurrentResonanceIndex(resonance_idx)
        self.editResonance()


    def deleteResonanceAtPosition(self, resonance_idx):
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


    def refreshFigures(self):
        self.raw_ax.set_ylabel('Magnitude (dB)')
        self.raw_ax.relim()
        self.raw_ax.autoscale()
        self.raw_figure.tight_layout()
        self.raw_canvas.draw_idle()
        self.filt_ax.set_ylabel(self.analysis_quantity_combo.currentText())
        self.filt_ax.relim()
        self.filt_ax.autoscale()
        self.filtered_figure.tight_layout()
        self.filtered_canvas.draw_idle()
        self.zoom_raw_ax.set_ylabel('Magnitude (dB)')
        self.zoom_raw_ax.relim()
        self.zoom_raw_ax.autoscale()
        self.zoom_raw_figure.tight_layout()
        self.zoom_raw_canvas.draw_idle()
        self.zoom_filt_ax.set_ylabel(self.analysis_quantity_combo.currentText())
        self.zoom_filt_ax.relim()
        self.zoom_filt_ax.autoscale()
        self.zoom_filt_figure.tight_layout()
        self.zoom_filt_canvas.draw_idle()
        self.updateZoomPlots()
        
    def openFile(self):
        options = QFileDialog.Options()
        lastfile = self.settings.value("lastFile", "")
        last_dir = os.path.dirname(lastfile) if lastfile else ""
        filename, _ = QFileDialog.getOpenFileName(
            self, "Open Sweep File", last_dir, "All Files (*)", options=options)

        if filename:
            try:
                self.loadRawData(filename)
            except Exception as e:

                tb=traceback.format_exc()

                QMessageBox.warning(self, "Error", "Failed to load file: " + filename + f'\n\n{e}\n\n{tb}')
                self.file_label.setText("Problem loading file")    
                return

        else:
            self.file_label.setText("No file loaded")

    def loadRawData(self, filename):
        # Load data from file
        if os.path.splitext(filename)[-1] == '.fits':
            try:
                from astropy.io import fits
            except ImportError as e:
                QMessageBox.warning(self, "Error", "Cannot open fits file, please try:\npip install astropy")
                raise(e)

            try:
                hdu,data = fits.open(filename)
                f = data.data['Freq']
                z = data.data['ReS21'] + 1j*data.data['ImS21']
            except Exception as e:
                QMessageBox.warning(self, "Error", "Failed to interpret file as a sweep: "+filename)
                raise(e)
            ss=np.argsort(f.ravel())
            self.frequencies = f.ravel()[ss]
            self.s21_complex = z.ravel()[ss]
            print(f'loaded {len(self.frequencies)} points, shape={f.shape}')
            
        else:
            try:
                # try load numpy
                data = np.load(filename,allow_pickle=True).item()
                print(filename)
                try:
                    #try load readout_client sweep
                    print(data.keys())
                    f = data['sweep_f']
                    z = data['sweep_i']+1j*data['sweep_q']
                    if f.ndim>1:
                        ss=np.argsort(f.ravel())
                        self.frequencies=f.ravel()[ss]
                        self.s21_complex=z.ravel()[ss]
                        
                except Exception as e:
                    print(f'not a readout_client sweep file: {e}')
            except Exception as e:
                print(f'not a npy file {e}')
                try:
                    data = np.loadtxt(filename, delimiter=',')
                    self.frequencies = data[:, 0]
                    self.s21_complex = data[:, 1] + 1j * data[:, 2]
                except Exception as e:
                    print(f'not a txt file with columns = f,i,q: {e}')                    
                    raise(e)
                

                
        
        self.frequency_stepsize = np.mean(np.diff(self.frequencies))
        self.magnitude = np.abs(self.s21_complex)
        self.log_magnitude = 20*np.log10(self.magnitude)
        self.phase = np.angle(self.s21_complex)
        self.unwrap_phase = np.unwrap(self.phase)
        self.group_delay_us = -np.gradient(self.unwrap_phase, self.frequencies)*1e6
        self.complex_gradient = np.abs(np.gradient(self.s21_complex, self.frequencies))

        self.plotRawData()
        self.updateFilter()
        self.updatePeaks()
        self.updateResonancesTable()
        self.updateZoomPlots()
        self.updateMarkers()
        self.updateNavigationButtons()
        self.settings.setValue("lastFile", os.path.abspath(filename))
        self.file_label.setText(os.path.abspath(filename))

                                   

    def plotRawData(self):
        self.raw_line.set_xdata(self.frequencies/1e6)
        self.raw_line.set_ydata(self.log_magnitude)
        self.raw_ax.relim()
        self.raw_ax.autoscale()
        self.raw_canvas.draw_idle()

        self.zoom_raw_line.set_xdata(self.frequencies/1e6)
        self.zoom_raw_line.set_ydata(self.log_magnitude)
        self.zoom_raw_ax.relim()
        self.zoom_raw_ax.autoscale()
        self.zoom_raw_canvas.draw_idle()

    def updateQuantity(self):
        current_quantity = self.analysis_quantity_combo.currentText()
        self.current_quantity = current_quantity
        self.loadPeakFinderParameters(current_quantity)
        self.updatePeakFinderLabels()
        self.updateFilter()
        self.updatePeaks()
        self.updateResonancesTable()
        self.updateZoomPlots()
        self.updateMarkers()
        self.refreshFigures()

    def updatePeakFinderLabels(self):
        labels = self.peak_finder_labels.get(self.current_quantity, None)
        if labels:
            self.peak_finder_prominence_label.setText(labels.get('prominence', 'Prominence'))
            self.peak_finder_width_label.setText(labels.get('width', 'Width'))
            self.peak_finder_distance_label.setText(labels.get('distance', 'Distance'))
            self.peak_finder_height_label.setText(labels.get('height', 'Height'))
            self.peak_finder_threshold_label.setText(labels.get('threshold', 'Threshold'))
        else:
            self.peak_finder_prominence_label.setText("Prominence")
            self.peak_finder_width_label.setText("Width")
            self.peak_finder_distance_label.setText("Distance")
            self.peak_finder_height_label.setText("Height")
            self.peak_finder_threshold_label.setText("Threshold")

    def updateFilter(self):
        
        # Get filter parameters
        analysis_quantity = self.analysis_quantity_combo.currentText()
        highpass_cutoff = self.highpass_spin.value()
        lowpass_cutoff = self.lowpass_spin.value()
        median_kernel_size = self.median_kernel_spin.value()
        

        # if highpass_cutoff >= lowpass_cutoff:
        #     QMessageBox.warning(self, "Warning", "Highpass cutoff should be less than lowpass cutoff.")
        #     return
                

        # Apply filters based on analysis quantity
        if analysis_quantity == "Lin Magnitude V":
            data = self.magnitude.copy()
        elif analysis_quantity == "Log Magnitude dB":
            data = self.log_magnitude.copy()
        elif analysis_quantity == "Phase rad":
            data = self.phase.copy()
        elif analysis_quantity == "Unwrapped Phase rad":
            data = self.unwrap_phase.copy()
        elif analysis_quantity == "Group Delay us (-dphi/df)":
            data = self.group_delay_us.copy()
        elif analysis_quantity == "Complex Gradient V/Hz (speed)":
            data = self.complex_gradient.copy()
        else:
            print(analysis_quantity,'?')
            data = self.log_magnitude.copy()
        # Apply highpass and lowpass filters
        if highpass_cutoff > 0:
            b, a = butter(2, highpass_cutoff, btype='highpass',fs=2.0)
            data = filtfilt(b, a, data)
        if lowpass_cutoff > 0:
            b, a = butter(2, lowpass_cutoff, btype='lowpass',fs=2.0)
            data = filtfilt(b, a, data)

        # Apply median filter
        if median_kernel_size % 2 == 0:
            median_kernel_size += 1  # Ensure odd integer
        if median_kernel_size > 1:
            data = medfilt(data, kernel_size=int(median_kernel_size))

        self.filtered_data = data
        self.plotFilteredData()

    def plotFilteredData(self):
        self.filt_line.set_xdata(self.frequencies/1e6)
        self.filt_line.set_ydata(self.filtered_data)
        self.filt_ax.relim()
        self.filt_ax.autoscale(axis='y')
        self.filtered_canvas.draw_idle()

        self.zoom_filt_line.set_xdata(self.frequencies/1e6)
        self.zoom_filt_line.set_ydata(self.filtered_data)
        self.zoom_filt_ax.relim()
        self.zoom_filt_ax.autoscale(axis='y')
        self.zoom_filt_canvas.draw_idle()
        
    def get_peak_direction(self):
        quantity = self.analysis_quantity_combo.currentText()
        if quantity in ["Lin Magnitude V", "Log Magnitude dB", "Phase rad", "Unwrapped Phase rad", "Group Delay us (-dphi/df)"]:
            return -1  # Looking for minima (negative peaks)
        elif quantity in ["Complex Gradient V/Hz (speed)"]:
            return 1  # Looking for maxima
        else:
            return 1  # Default to maxima



    def updatePeaks(self):
        if self.is_loading_settings:
            return
        # Get parameters from the dictionary
        params = self.peak_finder_params.get(self.current_quantity, {})
        
        # Prepare peak finder arguments
        prominence = (
            params['prominence_min'], params['prominence_max']
        ) if params.get('prominence_enabled', False) else None
        width = (
            max(1,params['width_min']/self.frequency_stepsize), max(1,params['width_max']/self.frequency_stepsize)
        ) if params.get('width_enabled', False) else None
        threshold = (
            params['threshold_min'], params['threshold_max']
        ) if params.get('threshold_enabled', False) else None
        height = (
            params['height_min'], params['height_max']
        ) if params.get('height_enabled', False) else None
        distance = max(1,params['distance_value']/self.frequency_stepsize) if params.get('distance_enabled', False) else None

        peak_direction = self.get_peak_direction()

        # Find peaks
        try:
            peaks, properties = find_peaks(
                peak_direction * self.filtered_data,
                prominence=prominence,
                width=width,
                distance=distance,
                threshold=threshold,
                height=height
            )
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Error in peak finding: {e}")
            return
        
        if len(peaks) > 10000:
            QMessageBox.warning(self, "Too Many Peaks", f"Too many peaks found ({len(peaks)}). Please adjust parameters.")
            return
            
        # Prepare for matching with existing resonances
        old_resonances = self.resonances.copy()
        self.resonances.clear()
        frequency_tolerance = self.frequency_stepsize

        # List to keep track of matched old resonances
        matched_old_resonances = []

        # Iterate over detected peaks
        for i, peak_idx in enumerate(peaks):
            peak_freq = self.frequencies[peak_idx]
            matched_resonance = None

            # Match with existing resonances
            for resonance in old_resonances:
                if abs(resonance.frequency - peak_freq) <= frequency_tolerance:
                    matched_resonance = resonance
                    old_resonances.remove(resonance)
                    break

            if matched_resonance:
                # Update existing resonance
                matched_resonance.peak_idx = peak_idx
                matched_resonance.frequency = peak_freq
                matched_resonance.analyse(self.frequencies,self.log_magnitude,self.filtered_data,peak_direction)
                self.resonances.append(matched_resonance)
            else:
                # Create new resonance
                new_resonance = Resonance(
                    frequency=peak_freq,
                    peak_idx=peak_idx
                )
                new_resonance.analyse(self.frequencies,self.log_magnitude,self.filtered_data,peak_direction)
                self.resonances.append(new_resonance)
        
        # Remove markers of unmatched old resonances
        unmatched_old_resonances = [res for res in old_resonances if res not in matched_old_resonances]
        for resonance in unmatched_old_resonances:
            resonance.remove_markers()


        # Update IDs and UI
        self.updateResonanceIDs()
        self.updateResonancesTable()
        self.updateMarkers()
        self.updateResonancesLabel()
        self.updateNavigationButtons()



    def updateResonanceIDs(self):
        current_id = 0
        for resonance in self.resonances:
            if resonance.save:
                resonance.id = current_id
                current_id += 1
            else:
                resonance.id = None  # Undefined ID for unsaved resonances
    

    def updateResonancesTable(self):
        self.resonances_table.blockSignals(True)
        row_count = len(self.resonances)
        self.resonances_table.setRowCount(row_count)

        for row, resonance in enumerate(self.resonances):
            # Save checkbox
            save_checkbox = self.resonances_table.cellWidget(row, 0)
            if save_checkbox is None:
                save_checkbox = QCheckBox()
                save_checkbox.stateChanged.connect(lambda state, r=resonance: self.onResonanceSaveStateChanged(r, state))
                self.resonances_table.setCellWidget(row, 0, save_checkbox)
            save_checkbox.setChecked(resonance.save)

            # ID
            id_item = self.resonances_table.item(row, 1)
            if id_item is None:
                id_item = QTableWidgetItem()
                id_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.resonances_table.setItem(row, 1, id_item)
            id_text = str(resonance.id).zfill(4) if resonance.id is not None else ""
            id_item.setText(id_text)

            # Frequency
            freq_item = self.resonances_table.item(row, 2)
            if freq_item is None:
                freq_item = QTableWidgetItem()
                freq_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.resonances_table.setItem(row, 2, freq_item)
            freq_item.setText(f"{resonance.frequency / 1e6:.6f}")

            # Q-factor
            q_item = self.resonances_table.item(row, 3)
            if q_item is None:
                q_item = QTableWidgetItem()
                q_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.resonances_table.setItem(row, 3, q_item)
            q_value = resonance.analysis.get('q', None)
            q_item.setText(f"{q_value:.1f}" if q_value is not None else "N/A")

            # FWHM
            fwhm_item = self.resonances_table.item(row, 4)
            if fwhm_item is None:
                fwhm_item = QTableWidgetItem()
                fwhm_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.resonances_table.setItem(row, 4, fwhm_item)
            fwhm_value = resonance.analysis.get('fwhm', None)
            fwhm_item.setText(f"{fwhm_value / 1e3:.3f}" if fwhm_value is not None else "N/A")

            # Depth
            depth_item = self.resonances_table.item(row, 5)
            if depth_item is None:
                depth_item = QTableWidgetItem()
                depth_item.setFlags(Qt.ItemIsSelectable | Qt.ItemIsEnabled)
                self.resonances_table.setItem(row, 5, depth_item)
            depth_value = resonance.analysis.get('dip_depth', None)
            depth_item.setText(f"{depth_value:.3f}" if depth_value is not None else "N/A")
        
        self.resonances_table.blockSignals(False)
        self.updateResonancesLabel()

    def updateResonancesLabel(self):
        total_resonances = len(self.resonances)
        num_saved_resonances = sum(1 for resonance in self.resonances if resonance.save)
        self.num_resonances_label.setText(
            f"Resonances found = {total_resonances}. Resonances to save = {num_saved_resonances}"
    )


    def onResonanceSaveStateChanged(self, resonance, state):
        if self.is_updating_save_state:
            return  # Prevent recursion

        self.is_updating_save_state = True

        resonance.save = (state == Qt.Checked)
        self.updateResonanceIDs()

        self.resonances_table.blockSignals(True)
        self.updateResonancesTable()
        self.resonances_table.blockSignals(True)
    
        self.updateMarkers()
        self.updateResonancesLabel()
        self.updateNavigationButtons()
        self.is_updating_save_state = False


    def setCurrentResonanceIndex(self, index):
        if not self.resonances:
            self.current_resonance_index = None
            self.analysis_text.setText("")
            self.updateNavigationButtons()
            return

        if self.current_resonance_index == index:
            return  # No change

        # Deselect previous resonance
        if self.current_resonance_index is not None and self.current_resonance_index < len(self.resonances):
            prev_resonance = self.resonances[self.current_resonance_index]
            prev_resonance.update_markers(self.frequencies, self.log_magnitude, self.filtered_data,
                                        is_selected=False, axes_dict=self.axes_dict)

        if index is not None and 0 <= index < len(self.resonances):
            self.current_resonance_index = index
            current_resonance = self.resonances[self.current_resonance_index]
            current_resonance.update_markers(self.frequencies, self.log_magnitude, self.filtered_data,
                                            is_selected=True, axes_dict=self.axes_dict)
            self.updateZoomPlots()
        else:
            self.current_resonance_index = None
            self.analysis_text.setText("")
            # Since no single resonance is selected, we might want to update the zoom plots accordingly
            # For now, we can leave it as is or implement custom behavior if needed
        
        if index is not None:
            self.resonances_table.blockSignals(True)
            self.resonances_table.clearSelection()
            self.resonances_table.selectRow(index)
            self.resonances_table.blockSignals(False)
            self.resonances_table.scrollToItem(self.resonances_table.item(index, 0))
            
        
        # Update plots
        self.raw_canvas.draw_idle()
        self.filtered_canvas.draw_idle()
        self.zoom_raw_canvas.draw_idle()
        self.zoom_filt_canvas.draw_idle()

        # Update table selection
        # No need to update table selection here since it's handled elsewhere

        self.updateNavigationButtons()

    def updateNavigationButtons(self):
        has_resonances = bool(self.resonances)
        self.prev_button.setEnabled(has_resonances)
        self.next_button.setEnabled(has_resonances)
        selected_rows = self.resonances_table.selectionModel().selectedRows()
        has_selection = bool(selected_rows)
        self.delete_button.setEnabled(has_resonances and has_selection)
        self.toggle_save_button.setEnabled(has_resonances and has_selection)

    def resonanceSelectionChanged(self):
        selected_rows = self.resonances_table.selectionModel().selectedRows()
        selected_indices = [index.row() for index in selected_rows]

        if len(selected_indices) == 1:
            index = selected_indices[0]
            self.setCurrentResonanceIndex(index)
        else:
            self.setCurrentResonanceIndex(None)

        self.updateNavigationButtons()
        self.updateMarkers()

        
    def updateZoomPlots(self):
        if self.current_resonance_index is None:
            return

        resonance = self.resonances[self.current_resonance_index]
        f = resonance.frequency
        fwhm = resonance.analysis.get('fwhm', None) 
        if fwhm is None or fwhm <= 0:
            fwhm = self.frequencies[1]-self.frequencies[0]  # Use default value to prevent zero span

        freq_span = 10 * fwhm

        print(resonance.id, f, fwhm)

        margin = 0.1
        freq_min = f - freq_span / 2
        freq_max = f + freq_span / 2
        freq_view_min = freq_min - margin * (freq_max - freq_min)
        freq_view_max = freq_max + margin * (freq_max - freq_min)

        mask = (self.frequencies >= freq_view_min) & (self.frequencies <= freq_view_max)
        # Handle empty mask
        if not np.any(mask):
            mask = slice(None)

        raw_min = np.min(self.log_magnitude[mask])
        raw_max = np.max(self.log_magnitude[mask])
        raw_view_min = raw_min - margin * (raw_max - raw_min)
        raw_view_max = raw_max + margin * (raw_max - raw_min)

        filt_min = np.min(self.filtered_data[mask])
        filt_max = np.max(self.filtered_data[mask])
        filt_view_min = filt_min - margin * (filt_max - filt_min)
        filt_view_max = filt_max + margin * (filt_max - filt_min)

        self.zoom_raw_ax.set_xlim(freq_view_min/1e6, freq_view_max/1e6)
        self.zoom_raw_ax.set_ylim(raw_view_min, raw_view_max)
        self.zoom_filt_ax.set_xlim(freq_view_min/1e6, freq_view_max/1e6)
        self.zoom_filt_ax.set_ylim(filt_view_min, filt_view_max)

        self.zoom_raw_canvas.draw_idle()
        self.zoom_filt_canvas.draw_idle()

        # Update analysis text
        if resonance.id is not None:
            id_text = f"{resonance.id}"
        else:
            id_text = "Do Not Save"
        analysis_text = f"Resonance ID: {id_text}\n"
        analysis_text += f"Frequency [MHz]: {f / 1e6:.6f} MHz\n"
        q_value = resonance.analysis.get('q', None)
        if q_value is not None:
            analysis_text += f"Q-factor: {q_value:.1f}\n"
        else:
            analysis_text += "Q-factor: N/A\n"

        fwhm_value = resonance.analysis.get('fwhm', None)
        if fwhm_value is not None:
            analysis_text += f"FWHM [kHz]: {fwhm_value / 1e3:.3f} kHz\n"
        else:
            analysis_text += "FWHM [kHz]: N/A\n"

        depth_value = resonance.analysis.get('dip_depth', None)
        if depth_value is not None:
            analysis_text += f"Depth [dB]: {depth_value:.3f}\n"
        else:
            analysis_text += "Depth [dB]: N/A\n"

        self.analysis_text.setText(analysis_text)

    def prevResonance(self):
        if not self.resonances:
            QMessageBox.information(self, "No Resonances", "There are no resonances to select.")
            return

        if self.current_resonance_index is None:
            tmpidx = len(self.resonances) - 1
        else:
            tmpidx = self.current_resonance_index - 1

        if tmpidx < 0:
            tmpidx = len(self.resonances) - 1  # Wrap around to the last resonance

        self.setCurrentResonanceIndex(tmpidx)

    def nextResonance(self):
        if not self.resonances:
            QMessageBox.information(self, "No Resonances", "There are no resonances to select.")
            return

        if self.current_resonance_index is None:
            tmpidx = 0
        else:
            tmpidx = self.current_resonance_index + 1

        if tmpidx >= len(self.resonances):
            tmpidx = 0  # Wrap around to the first resonance

        self.setCurrentResonanceIndex(tmpidx)

    def toggleSaveSelectedResonances(self):
        selected_rows = self.resonances_table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.information(self, "No Resonance Selected", "Please select one or more resonances to toggle save state.")
            return

        for index in selected_rows:
            idx = index.row()
            resonance = self.resonances[idx]
            resonance.save = not resonance.save
        self.updateResonanceIDs()
        self.updateResonancesTable()
        self.updateMarkers()
        self.updateResonancesLabel()
        self.updateNavigationButtons()

    def addResonance(self, initial_freq_mhz=None):
        if initial_freq_mhz is None:
            # Use mid-frequency as default
            initial_freq_mhz = self.frequencies[len(self.frequencies) // 2] / 1e6
        freq_mhz, ok = QInputDialog.getDouble(
            self, "Add Resonance", "Enter new frequency (MHz):",
            value=initial_freq_mhz, decimals=6
        )
        if ok:
            frequency = freq_mhz * 1e6
            # Find the closest peak index
            peak_idx = np.abs(self.frequencies - frequency).argmin()
            # Create and analyze the new resonance
            new_resonance = Resonance(frequency=frequency, peak_idx=peak_idx)
            new_resonance.analyse(self.frequencies, self.log_magnitude,self.filtered_data, self.get_peak_direction())
            self.resonances.append(new_resonance)
            self.resonances.sort(key=lambda r: r.frequency)
            new_index = self.resonances.index(new_resonance)
            self.setCurrentResonanceIndex(new_index)
            # Update UI and markers
            self.updateResonanceIDs()
            self.updateResonancesTable()
            self.updateMarkers()
            self.updateResonancesLabel()
            self.updateNavigationButtons()


    def editResonance(self):
        if self.current_resonance_index is None:
            QMessageBox.information(self, "Edit Resonance", "No resonance selected.")
            return
        resonance = self.resonances[self.current_resonance_index]
        freq_mhz, ok = QInputDialog.getDouble(
            self, "Edit Resonance", "Enter new frequency (MHz):",
            value=resonance.frequency / 1e6, decimals=6
        )
        if ok:
            resonance.frequency = freq_mhz * 1e6
            # Update peak_idx to the closest index
            resonance.peak_idx = np.abs(self.frequencies - resonance.frequency).argmin()
            # Re-analyse with the new frequency and peak_idx
            resonance.analyse(self.frequencies, self.log_magnitude,self.filtered_data, self.get_peak_direction())
            # Save a reference to the edited resonance
            edited_resonance = resonance
            # Sort the resonance list
            self.resonances.sort(key=lambda r: r.frequency)
            # Find the new index of the edited resonance
            new_index = self.resonances.index(edited_resonance)
            # Update current resonance index
            self.setCurrentResonanceIndex(new_index)
            # Update IDs and UI
            self.updateResonanceIDs()
            self.updateResonancesTable()
            self.updateMarkers()
            self.updateResonancesLabel()
            self.updateNavigationButtons()


    def deleteResonance(self):
        selected_rows = self.resonances_table.selectionModel().selectedRows()
        if not selected_rows:
            QMessageBox.information(self, "No Resonance Selected", "Please select one or more resonances to delete.")
            return

        reply = QMessageBox.question(
            self, 'Delete Resonances',
            f"Are you sure you want to delete the selected {len(selected_rows)} resonance(s)?",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply == QMessageBox.Yes:
            # Collect indices in reverse order to avoid index shifting issues
            indices = sorted([row.row() for row in selected_rows], reverse=True)
            for idx in indices:
                resonance = self.resonances.pop(idx)
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
                


    def updateMarkers(self):
        selected_rows = [index.row() for index in self.resonances_table.selectionModel().selectedRows()]
        for i, resonance in enumerate(self.resonances):
            is_selected = i in selected_rows
            resonance.update_markers(
                self.frequencies, self.log_magnitude, self.filtered_data,
                is_selected, self.axes_dict
            )
        # Redraw plots
        self.raw_canvas.draw_idle()
        self.filtered_canvas.draw_idle()
        self.zoom_raw_canvas.draw_idle()
        self.zoom_filt_canvas.draw_idle()

    def saveResonances(self):
        
        options = QFileDialog.Options()
        if os.path.splitext(self.file_label.text())[-1] == '.fits':
            default_ext = '.txt'
            default_filename = os.path.splitext(self.file_label.text())[0] + default_ext
            filename, _ = QFileDialog.getSaveFileName(self, "Save Resonances", default_filename,
                                                    "KIDLAB Toneslist Files (*.txt);;Resonance Files (*.resonances);;All Files (*)", options=options)
        else:
            default_ext = ".resonances"
            default_filename = os.path.splitext(self.file_label.text())[0] + default_ext
            filename, _ = QFileDialog.getSaveFileName(self, "Save Resonances", default_filename,
                                                    "Resonance Files (*.resonances);;KIDLAB Toneslist Files (*.txt);;All Files (*)", options=options)
        if filename:
            try:
                if filename.endswith('.resonances'):

                    with open(filename, 'w') as f:
                        # f.write("ID,Frequency(Hz),Q-factor,FWHM(Hz)\n")
                        f.write("#ID,Frequency(Hz)\n")
                        for resonance in self.resonances:
                            if resonance.save and resonance.id is not None:
                                f.write(f"{resonance.id},{resonance.frequency}\n")
                                        # f"{resonance.analysis.get('q', 0)},{resonance.analysis.get('fwhm', 0)}\n")

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

    def showResonancesTableContextMenu(self, position):
        menu = QMenu()

        # Actions
        add_action = QAction("Add Resonance", self)
        edit_action = QAction("Edit Resonance", self)
        delete_action = QAction("Delete Selected Resonance(s)", self)
        toggle_save_action = QAction("Toggle Save Selected Resonance(s)", self)

        # Connect actions to methods
        add_action.triggered.connect(lambda: self.addResonanceAtTablePosition(position))
        edit_action.triggered.connect(lambda: self.editResonanceAtTablePosition(position))
        delete_action.triggered.connect(self.deleteResonance)
        toggle_save_action.triggered.connect(self.toggleSaveSelectedResonances)

        # Add actions to menu
        menu.addAction(add_action)
        menu.addAction(edit_action)
        menu.addAction(delete_action)
        menu.addAction(toggle_save_action)

        # Get selected items to determine if actions should be enabled
        selected_rows = self.resonances_table.selectionModel().selectedRows()
        has_selection = len(selected_rows) > 0

        # Enable or disable actions based on selection
        edit_action.setEnabled(has_selection and len(selected_rows) == 1)
        delete_action.setEnabled(has_selection)
        toggle_save_action.setEnabled(has_selection)

        # Show the context menu at the cursor position
        menu.exec_(self.resonances_table.viewport().mapToGlobal(position))

    def addResonanceAtTablePosition(self, position):
        index = self.resonances_table.indexAt(position)
        if index.isValid():
            row_clicked = index.row()
            resonance = self.resonances[row_clicked]
            initial_freq_mhz = resonance.frequency / 1e6
        else:
            initial_freq_mhz = self.frequencies[len(self.frequencies) // 2] / 1e6  # Mid frequency

        freq_mhz, ok = QInputDialog.getDouble(
            self, "Add Resonance", "Enter new frequency (MHz):",
            value=initial_freq_mhz, decimals=6
        )
        if ok:
            self.addResonance(initial_freq_mhz=freq_mhz)

    def editResonanceAtTablePosition(self, position):
        index = self.resonances_table.indexAt(position)
        if index.isValid():
            row_clicked = index.row()
            self.setCurrentResonanceIndex(row_clicked)
            self.editResonance()
        else:
            QMessageBox.information(self, "Edit Resonance", "No resonance selected.")


    def loadSettings(self):
        self.is_loading_settings = True
        self.restoreGeometry(self.settings.value("geometry", type=QByteArray))
        self.restoreState(self.settings.value("windowState", type=QByteArray))
        try:
            self.loadRawData(self.settings.value("lastFile", ""))
        except:
            pass
        # Restore other widget states here
        splitter_sizes = self.settings.value("splitterSizes")
        if splitter_sizes:
            self.hsplitter.setSizes([int(i) for i in list(splitter_sizes)])
        # Restore other parameters if needed
                # Load all parameters from QSettings

        self.loadFilterParameters()
        
        self.loadAllPeakFinderParameters()
        self.current_quantity = self.analysis_quantity_combo.currentText()
        self.loadPeakFinderParameters(self.current_quantity)
        self.updatePeakFinderLabels()
        self.is_loading_settings = False
        


    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Left:
            self.prevResonance()
        elif event.key() == Qt.Key_Right:
            self.nextResonance()
        elif event.key() == Qt.Key_Delete:
            self.deleteResonance()
        elif event.key() == Qt.Key_Space:
            self.toggleSaveSelectedResonances()
        else:
            super().keyPressEvent(event)

    # def onRawPlotClick(self, event):
    #     if event.button == 1:  # Left-click
    #         frequency = event.xdata
    #         if frequency is None:
    #             return
    #         # Check if clicked near a resonance marker
    #         for idx, resonance in enumerate(self.resonances):
    #             if abs(resonance['frequency'] - frequency) < 0.01:
    #                 self.current_resonance_index = idx
    #                 self.resonances_table.selectRow(idx)
    #                 return
    #     elif event.button == 3:  # Right-click
    #         menu = QMenu()
    #         add_action = QAction('Add Resonance', self)
    #         add_action.triggered.connect(lambda: self.addResonanceAt(event.xdata))
    #         menu.addAction(add_action)
    #         menu.exec_(self.raw_canvas.mapToGlobal(QPoint(event.x, event.y)))

    # def addResonanceAt(self, frequency):
    #     resonance = {
    #         'save': True,
    #         'id': len(self.resonances),
    #         'frequency': frequency,
    #         'q_factor': 0,  # Placeholder
    #         'fwhm': 0  # Placeholder
    #     }
    #     self.resonances.append(resonance)
    #     self.resonances.sort(key=lambda r: r['frequency'])
    #     self.updateResonancesTable()
    #     self.updateMarkers()

def main():
    app = QApplication(sys.argv)

    if sys.platform in ['nt','win32']:
        app.setWindowIcon(QIcon(str(files("souk_readout_tools").joinpath("mkid_finder_app.ico"))))
    else:
        app.setWindowIcon(QIcon(str(files("souk_readout_tools").joinpath("mkid_finder_app.png"))))
    
    window = ResonanceFinder()
    window.show()
    app.exec_()

if __name__ == "__main__":
    sys.exit(main())
