"""Mock readout-server state for ``ReadoutClient(..., mock=True)``.

This module models the server side rather than providing a
separate mock client class.  The public API remains ``ReadoutClient``; mock
mode swaps socket traffic for this in-process state object.
"""

import base64
import copy
import datetime
import importlib.metadata
import json
import logging
import os
import sys
import time
import traceback

import numpy as np
import yaml

# so3g / spt3g are optional; only the receive_stream_g3() path needs them.
# They have no PyPI wheels and are hard to build on Windows, so defer failure.
try:
    import so3g
    import spt3g.core
    _G3_IMPORT_ERROR = None
except ImportError as _e:
    so3g = None
    spt3g = None
    _G3_IMPORT_ERROR = _e


def _require_g3():
    '''Raise a helpful error if the optional so3g/spt3g stack is unavailable.'''
    if _G3_IMPORT_ERROR is not None:
        raise ImportError(
            "The G3 stream format requires the 'so3g' / 'spt3g' packages, which "
            "are not installed. Install with `pip install souk_readout_tools[g3]` "
            "(unavailable on Windows). All other functionality works without them."
        ) from _G3_IMPORT_ERROR

from souk_readout_tools.config_utils import get_template_config_path
from souk_readout_tools.timing import unix_to_iso, format_duration_s


class MockReadoutServer:
    """In-process stand-in for the RFSoC readout server."""

    DEFAULT_INFO_SECTIONS = [
        'server', 'versions', 'clock', 'timing', 'sync', 'fpga', 'rfdc',
        'pipeline', 'tones', 'rf_frontend', 'lna', 'rfsoc_sensors',
    ]
    ALL_INFO_SECTIONS = DEFAULT_INFO_SECTIONS + [
        'diagnostics', 'config', 'calibrations', 'resonators', 'registers',
        'modulation', 'tracking',
    ]

    @staticmethod
    def default_config(address='127.0.0.1', request_port=10000,
                       stream_port=20000):
        """Load the bundled template config and patch mock connection fields.

        ``address``, ``request_port`` and ``stream_port`` are written into the
        returned config as the mock server's connection details.
        """
        with open(get_template_config_path(), 'r') as file:
            config = yaml.safe_load(file)

        config['rfsoc_host']['address'] = address
        config['rfsoc_host']['request_port'] = request_port
        config['rfsoc_host']['stream_port'] = stream_port
        return config

    def __init__(self, client):
        self.client = client
        self.server_start_unix_s = time.time()
        self.config_on_rfsoc = (
            yaml.dump(client.config, sort_keys=False)
            if client.config is not None else ''
        )
        self.is_streaming = False
        self.triggered_stream_enabled = False
        self.packet_counter = 0
        self._last_timed_sync = None    # result of the last mock timed_sync_arm
        self._last_tt_load_unix_s = None  # wall-clock of the last mock TT load (drift ref)
        # fast frequency modulation (mock): _mod is the armed config dict (or None);
        # _mod_enabled is the on/off switch. A simple resonator phase model gives the
        # demod tool ground truth, and a mock channel grid lets us exercise the
        # bin-occupancy / recenter logic without real firmware.
        self._mod = None
        self._mod_enabled = False
        self._mod_revision = 0
        self._mod_revision_history = {}
        self._mod_f0 = None             # per-tone "true" resonance frequencies (Hz)
        self._mod_linewidth = 1.0e5     # mock resonator FWHM (Hz)
        self._mod_bin_hz = 1.0e6        # mock filterbank channel spacing (Hz)
        self._mod_armed_bin_center = None   # per-tone armed bin centre (Hz), fixed until recenter
        self._tracking = None           # status-level mock of the tracking loop
        # firmware-slot modulation (mock): _fw_mod is the loaded config (or None),
        # _fw_mod_enabled the on/off switch, _fw_mod_slot the live slot in manual
        # mode. Frames are tagged with slot+1 in the same flag5 field as software
        # modulation (the two schemes are mutually exclusive).
        self._fw_mod = None
        self._fw_mod_enabled = False
        self._fw_mod_slot = 0
        self._fw_mod_revision = 0
        self._fw_mod_f0 = None          # per-tone "true" resonance frequencies (Hz)
        self._fw_n_slots = 4            # mock mixer.n_slots (auto cycles all of them)
        self._fw_dwell_slot = -1        # last slot emitted (settling-edge tracking)
        self._fw_dwell_pos = 0          # accumulations into the current dwell
        self.cal_freeze = True
        self.clock_source = (
            client.config.get('firmware', {}).get('clock_source', 'internal')
            if client.config is not None else 'internal'
        )
        self.latest_sweep_data = None
        self.sweep_progress = 1.0
        self.calibration_files = {}
        self.rng = np.random.default_rng(12345)
        self._resonator_catalog = self._generate_resonator_catalog()
        self._init_tone_state()

    @property
    def config(self):
        return self.client.config

    @property
    def sample_rate(self):
        """Streaming/accumulator sample rate (Hz), derived from ``acc_len``.

        Mirrors firmware_lib.get_sample_rate so that setting either ``acc_len``
        or ``sample_rate_hz`` keeps the two consistent, just like the real
        server.
        """
        return self._fft_bw_hz / self.acc_len

    def _acc_len_for_rate(self, sample_rate_hz):
        """Nearest valid ``acc_len`` for a requested sample rate.

        Matches firmware_lib.set_sample_rate: round to an integer, force a
        multiple of 4, and clamp to the 1..2**16-1 register range.
        """
        acc_len = int(round(self._fft_bw_hz / sample_rate_hz))
        if acc_len % 4 != 0:
            acc_len = 4 * int(np.ceil(acc_len / 4))
        return int(min(max(acc_len, 1), 2**16 - 1))

    def _init_tone_state(self):
        defaults = (self.config or {}).get('firmware', {}).get('defaults', {})
        freqs = (
            defaults.get('tone_frequencies')
            or defaults.get('frequencies')
            or defaults.get('blind_frequencies')
            or [2.1e9]
        )
        freqs = np.atleast_1d(freqs).astype(float).tolist()
        n_tones = len(freqs)

        # acc_len is the source of truth, exactly as on the real server: the
        # streaming/accumulator sample rate is derived from it (see the
        # ``sample_rate`` property and firmware_lib.get_sample_rate). The mock
        # FFT bandwidth mirrors firmware_lib: adc_clk_hz / (N_RX_FFT /
        # N_RX_OVERSAMPLE), with N_RX_FFT = 8192 and N_RX_OVERSAMPLE = 2.
        self.adc_clk_hz = 2457600000.0
        self._fft_bw_hz = self.adc_clk_hz / (8192 / 2)
        acc_len = defaults.get('acc_len')
        if acc_len is None:
            requested_rate = defaults.get(
                'sample_rate_hz',
                defaults.get('acc_freq', defaults.get('sample_rate')))
            acc_len = (self._acc_len_for_rate(float(requested_rate))
                       if requested_rate is not None else 1000)
        self.acc_len = int(acc_len)
        self.tone_frequencies = freqs
        self.tone_amplitudes = self._per_tone_list(
            defaults.get('tone_amplitudes', defaults.get('amplitudes', 1.0)),
            n_tones, 1.0)
        self.tone_phases = self._per_tone_list(
            defaults.get('tone_phases', defaults.get('phases', 0.0)),
            n_tones, 0.0)
        self.tone_powers_dbm = self._per_tone_list(
            defaults.get('tone_powers_dbm', defaults.get('powers_dbm', -60.0)),
            n_tones, -60.0)

    @staticmethod
    def _per_tone_list(values, n_tones, default):
        """Return a list of length ``n_tones`` from scalar or per-tone values."""
        values = np.atleast_1d(default if values is None else values).astype(float)
        if len(values) == n_tones:
            return values.tolist()
        if len(values) == 1:
            return np.full(n_tones, values[0], dtype=float).tolist()
        result = np.full(n_tones, default, dtype=float)
        result[:min(n_tones, len(values))] = values[:min(n_tones, len(values))]
        return result.tolist()

    def _resize_tone_state(self, n_tones):
        """Keep mock per-tone arrays aligned with the active tone count."""
        self.tone_amplitudes = self._per_tone_list(
            self.tone_amplitudes, n_tones, 1.0)
        self.tone_phases = self._per_tone_list(
            self.tone_phases, n_tones, 0.0)
        self.tone_powers_dbm = self._per_tone_list(
            self.tone_powers_dbm, n_tones, -60.0)

    def _generate_resonator_catalog(self, count=500, f_min_hz=500.0e6,
                                    f_max_hz=2000.0e6):
        """Create a stable mock resonator catalog for wideband sweeps."""
        count = int(count)
        freqs = np.linspace(f_min_hz, f_max_hz, count)
        spacing = (f_max_hz - f_min_hz) / max(count - 1, 1)
        scatter = self.rng.uniform(-0.25, 0.25, count) * spacing
        freqs = np.clip(freqs + scatter, f_min_hz, f_max_hz)
        freqs.sort()

        qc = self.rng.uniform(40.0e3, 60.0e3, count)
        qi = self.rng.uniform(400.0e3, 600.0e3, count)
        qr = 1.0 / (1.0 / qc + 1.0 / qi)
        phi = self.rng.normal(0.0, 0.08, count)
        return {
            'frequency_hz': freqs,
            'qc': qc,
            'qi': qi,
            'qr': qr,
            'phi_rad': phi,
        }

    @staticmethod
    def _combined_q(qc, qi):
        return 1.0 / (1.0 / qc + 1.0 / qi)

    @staticmethod
    def _baseline_response(frequencies_hz):
        """Return a gently sloped complex baseline for mock sweeps."""
        frequencies_hz = np.asarray(frequencies_hz, dtype=float)
        x = (frequencies_hz - 1250.0e6) / 1500.0e6
        amplitude = (
            1.0
            + 0.035 * x
            + 0.012 * np.sin(2.0 * np.pi * (2.3 * x + 0.1))
        )
        phase = 0.18 * x + 0.035 * np.sin(2.0 * np.pi * (1.7 * x))
        return amplitude * np.exp(1j * phase)

    @staticmethod
    def _apply_resonator_response(frequencies_hz, response, fr, qc, qi,
                                  phi_rad=0.0):
        """Apply a single notch-resonator response in place."""
        qr = MockReadoutServer._combined_q(qc, qi)
        coupling = (qr / qc) * np.exp(1j * phi_rad)
        x = (frequencies_hz - fr) / fr
        response *= 1.0 - coupling / (1.0 + 2.0j * qr * x)

    def _apply_catalog_resonators(self, frequencies_hz, response):
        """Apply catalog resonators that overlap the requested frequencies."""
        flat_f = np.ravel(frequencies_hz)
        flat_z = np.ravel(response)
        if flat_f.size == 0:
            return

        f_min = float(np.nanmin(flat_f))
        f_max = float(np.nanmax(flat_f))
        catalog_f = self._resonator_catalog['frequency_hz']
        relevant = np.where(
            (catalog_f >= f_min - 5.0e6) & (catalog_f <= f_max + 5.0e6)
        )[0]

        for index in relevant:
            fr = self._resonator_catalog['frequency_hz'][index]
            qc = self._resonator_catalog['qc'][index]
            qi = self._resonator_catalog['qi'][index]
            qr = self._resonator_catalog['qr'][index]
            phi = self._resonator_catalog['phi_rad'][index]
            window_hz = max(2.0e6, 30.0 * fr / qr)
            mask = np.abs(flat_f - fr) <= window_hz
            if np.any(mask):
                masked_response = flat_z[mask]
                self._apply_resonator_response(
                    flat_f[mask], masked_response, fr, qc, qi, phi)
                flat_z[mask] = masked_response

    def _apply_local_sweep_resonators(self, sweep_f, sweep_z, centers, spans):
        """Ensure narrow per-tone sweeps still contain plausible resonators."""
        catalog_f = self._resonator_catalog['frequency_hz']
        for index, (center, span) in enumerate(zip(centers, spans)):
            half_span = max(float(span) / 2.0, 0.0)
            has_catalog_resonator = np.any(
                (catalog_f >= center - half_span)
                & (catalog_f <= center + half_span)
            )
            if has_catalog_resonator:
                continue

            local_span = max(float(span), 100.0e3)
            fr = center + self.rng.uniform(-0.2, 0.2) * local_span
            qc = self.rng.uniform(40.0e3, 60.0e3)
            qi = self.rng.uniform(400.0e3, 600.0e3)
            phi = self.rng.normal(0.0, 0.08)
            self._apply_resonator_response(
                sweep_f[:, index], sweep_z[:, index], fr, qc, qi, phi)

    def _tone_metadata(self):
        """Return user-facing tone metadata matching firmware_lib's shape."""
        n_tones = len(self.tone_frequencies)
        defaults = (self.config or {}).get('firmware', {}).get('defaults', {})
        n_regular = len(np.atleast_1d(defaults.get('frequencies', [])))
        n_blind = len(np.atleast_1d(defaults.get('blind_frequencies', [])))
        configured_count = n_regular + n_blind
        matches_config = configured_count == n_tones and configured_count > 0

        if matches_config:
            regular_indices = list(range(n_regular))
            blind_indices = list(range(n_regular, n_regular + n_blind))
            tone_types = ['regular'] * n_regular + ['blind'] * n_blind
            is_blind = [False] * n_regular + [True] * n_blind
        else:
            regular_indices = list(range(n_tones))
            blind_indices = []
            tone_types = ['regular'] * n_tones
            is_blind = [False] * n_tones

        return {
            'tone_types': tone_types,
            'is_blind': is_blind,
            'regular_indices': regular_indices,
            'blind_indices': blind_indices,
            'num_regular_tones': len(regular_indices),
            'num_blind_tones': len(blind_indices),
            'metadata_matches_config': matches_config,
            'configured_num_tones': configured_count,
        }

    def _tone_frequency_details(self):
        """Return detailed frequency metadata matching the real server shape."""
        n_tones = len(self.tone_frequencies)
        indices = list(range(n_tones))
        freqs = list(self.tone_frequencies)
        details = {
            'tx': {
                'tone_indices': indices,
                'filterbank_bins': indices,
                'mixer_lo_phase_increment': [0.0] * n_tones,
                'mixer_lo_ri_step': [(1.0, 0.0)] * n_tones,
                'mixer_lo_phase_step': [0.0] * n_tones,
                'mixer_lo_offset_freq': [0.0] * n_tones,
                'filterbank_center_freq': freqs,
                'digital_baseband_freq': freqs,
                'analog_output_freq': freqs,
                'rf_output_freq': freqs,
            },
            'rx': {
                'tone_indices': indices,
                'filterbank_bins': indices,
                'mixer_lo_phase_increment': [0.0] * n_tones,
                'mixer_lo_ri_step': [(1.0, 0.0)] * n_tones,
                'mixer_lo_phase_step': [0.0] * n_tones,
                'mixer_lo_offset_freq': [0.0] * n_tones,
                'filterbank_center_freq': freqs,
                'digital_baseband_freq': freqs,
                'analog_input_freq': freqs,
                'rf_input_freq': freqs,
            },
        }
        details.update(self._tone_metadata())
        return details

    def info(self, sections=None):
        """Return structured system information by section.

        ``sections`` selects which section(s) to build: ``None``/``'all'`` for
        everything, or a section name (or list of names) such as ``'tones'``.
        """
        dispatchers = {
            'server': self._info_server,
            'versions': self._info_versions,
            'clock': self._info_clock,
            'timing': self._info_timing,
            'sync': self._info_sync,
            'fpga': self._info_fpga,
            'rfdc': self._info_rfdc,
            'pipeline': self._info_pipeline,
            'tones': self._info_tones,
            'rf_frontend': self._info_rf_frontend,
            'lna': self._info_lna,
            'rfsoc_sensors': self._info_rfsoc_sensors,
            'diagnostics': self._info_diagnostics,
            'config': self._info_config,
            'calibrations': self._info_calibrations,
            'resonators': self._info_resonators,
            'registers': self._info_registers,
            'modulation': self._info_modulation,
            'tracking': self._info_tracking,
        }
        if sections == 'list':
            return {
                'all': [s for s in self.ALL_INFO_SECTIONS if s in dispatchers],
                'default': [s for s in self.DEFAULT_INFO_SECTIONS if s in dispatchers],
            }
        if sections is None:
            return {s: dispatchers[s]() for s in self.DEFAULT_INFO_SECTIONS}
        if sections == 'all':
            return {s: dispatchers[s]() for s in self.ALL_INFO_SECTIONS}
        if isinstance(sections, str):
            return dispatchers.get(sections, lambda: {})()
        return [dispatchers[s]() for s in sections if s in dispatchers]

    def _info_server(self):
        now_unix_s = time.time()
        try:
            uname = os.uname()
            uname_text = (
                f'{uname.nodename} {uname.sysname} {uname.release} '
                f'{uname.version} {uname.machine}'
            )
        except Exception:
            uname_text = ''
        return {
            'ready': True,
            'process_name': 'mock_readout_server',
            'ip_addresses': self.client.request_server_address,
            'pipeline_id': self.client.pipeline_id,
            'pipeline_dirs': {},
            'pwd': os.getcwd(),
            'sys_executable': sys.executable,
            'sys_argv': sys.argv,
            'uname': uname_text,
            'python_version': sys.version,
            'server_version': self._package_version(),
            'config_file': self.client.config_file,
            'initialisation_level': 'pipeline',
            'current_time_unix_s': now_unix_s,
            'current_time_iso': datetime.datetime.fromtimestamp(
                now_unix_s, datetime.timezone.utc).isoformat(),
            'server_start_unix_s': self.server_start_unix_s,
            'server_start_iso': datetime.datetime.fromtimestamp(
                self.server_start_unix_s, datetime.timezone.utc).isoformat(),
            'server_uptime_s': now_unix_s - self.server_start_unix_s,
            'board_uptime_s': None,
            'request_clients': 1,
            'request_client_addrs': [],
            'stream_clients': 0,
            'stream_client_addrs': [],
            'streaming': self.is_streaming,
            'triggered_streaming': self.triggered_stream_enabled,
            'sweeping': False,
            'task_count': 0,
            'latest_sweep_data_valid': self.latest_sweep_data is not None,
            'firmware_interface_exists': True,
            'firmware_interface_ready': True,
            'firmware_fast_interface_exists': True,
            'firmware_fast_interface_ready': True,
            'mock': True,
        }

    def _info_versions(self):
        return {
            'ready': True,
            'souk_readout_tools_version': self._package_version(),
            'souk_mkid_readout_sw_version': 'mock',
            'souk_mkid_readout_fw_version': 'mock',
            'souk_readout_tools_commit': None,
            'souk_firmware_commit': None,
            'souk_peripherals_commit': None,
            'mock': True,
        }

    @staticmethod
    def _package_version():
        try:
            return importlib.metadata.version('souk_readout_tools')
        except Exception:
            return None

    def _info_clock(self):
        return {
            'ready': True,
            'source': self.clock_source,
            'all_locked': True,
            'chips': [],
        }

    def _info_timing(self):
        now = time.time()
        return {
            'summary': {
                'state': 'mock',
                'condition': 'Mock timing source.',
                'updated_unix_s': now,
                'ready_for_firmware_sync': True,
                'absolute_time_verified': True,
                'estimated_abs_error_s': 0.0,
                'active_source_type': 'mock',
                'active_source_name': 'mock',
                'active_source_state': 'locked',
                'active_source_offset_s': 0.0,
                'active_source_error_s': 0.0,
                'ntp_selected_source': 'mock',
                'ntp_selected_error_s': 0.0,
                'ntp_best_source': 'mock',
                'ntp_best_error_s': 0.0,
                'system_synced': True,
                'system_time_offset_s': 0.0,
                'last_offset_s': 0.0,
                'rms_offset_s': 0.0,
                'root_distance_s': 0.0,
                'ptp_healthy': True,
                'ptp_fresh': True,
                'ptp_stable': True,
                'ptp_gm_identity': 'mock',
                'ptp_last_gm_identity': 'mock',
                'ptp_master_offset_ns': 0,
                'ptp_ingress_age_s': 0.0,
                'ptp_holdover_age_s': None,
                'ptp_holdover_error_rate_ppm': None,
                'ptp_holdover_window_s': None,
                'ptp_holdover_expired': False,
                'holdover_estimated_seconds_until_ntp_error': None,
            },
            'monitor': {
                'available': True,
                'updated_unix_s': now,
                'error': None,
                'ptp_holdover_error_rate_ppm': None,
                'ptp_holdover_window_s': None,
                'ptp_ingress_stale_after_s': None,
                'ptp_lock_offset_threshold_ns': None,
            },
            'ptp': {},
            'phc': {},
            'ntp': {},
            'chrony': {},
            'sync_readiness': {'ptp_ready': True, 'pps_active': True,
                               'can_sync': True, 'reasons': []},
        }

    def _info_sync(self):
        """Mock firmware timed-sync status for ``get_info('sync')`` (mirrors the server)."""
        now = time.time()
        last_pps_unix_s = round(now)
        ever_synced = self._last_timed_sync is not None
        last_armed = self._last_timed_sync
        if last_armed is not None:
            last_sync_unix_s = float(last_armed.get('target_tt_unix_s', last_pps_unix_s))
            time_since_last_sync = max(0.0, now - last_sync_unix_s)
        else:
            last_sync_unix_s = None
            time_since_last_sync = None
        loaded = self._last_tt_load_unix_s
        time_since_tt_load = (now - loaded) if loaded is not None else None
        pps_boundary_offset_s = 1e-6
        drift_ppm = (pps_boundary_offset_s / time_since_tt_load * 1e6
                     if time_since_tt_load is not None and time_since_tt_load >= 2.0 else None)
        return {
            'available': True,
            'state': 'aligned' if (ever_synced or last_sync_unix_s is not None) else 'never_synced',
            'ever_synced': ever_synced,
            'readiness': {'ptp_ready': True, 'pps_active': True, 'can_sync': True},
            'aligned': True,
            'align_tol_s': 1e-4,
            'pps_boundary_offset_s': pps_boundary_offset_s,
            'system_offset_s': last_pps_unix_s - now,
            'last_pps_unix_s': float(last_pps_unix_s),
            'last_pps_utc': unix_to_iso(last_pps_unix_s),
            'last_sync_tt': (int(last_sync_unix_s * 307_200_000) if last_sync_unix_s else None),
            'last_sync_unix_s': last_sync_unix_s,
            'last_sync_utc': unix_to_iso(last_sync_unix_s),
            'time_since_last_sync_s': time_since_last_sync,
            'time_since_last_sync': format_duration_s(time_since_last_sync),
            'tt_loaded_unix_s': loaded,
            'tt_loaded_utc': unix_to_iso(loaded),
            'time_since_tt_load_s': time_since_tt_load,
            'drift_ppm': drift_ppm,
            'drift_offset_s': pps_boundary_offset_s,
            'last_armed': last_armed,
        }

    def _info_fpga(self):
        return {
            'ready': True,
            'fpga_status': {
                'programmed': True,
                'timestamp': datetime.datetime.now(datetime.timezone.utc).isoformat(),
                'host': 'mock',
                'sw_version': 'mock',
                'fw_version': 'mock',
                'fw_supported': True,
            },
            'fpg_file': self.config.get('firmware', {}).get('fw_config_file'),
            'pipeline_id': self.client.pipeline_id,
            'adc_clk_hz': int(self.adc_clk_hz),
        }

    def _info_rfdc(self):
        defaults = self.config.get('firmware', {}).get('defaults', {})
        nyquist_zone = defaults.get('nyquist_zone', 1)
        inverse_sinc_enabled = defaults.get(
            'dac_inverse_sinc_filter_enabled', True)
        inverse_sinc_mode = nyquist_zone if inverse_sinc_enabled else 0
        return {
            'ready': True,
            'dsa': defaults.get('dsa', 0),
            'vop_dac0': defaults.get('vop', 20000),
            'vop_dac1': defaults.get('vop', 20000),
            'dac_duc_mixer_frequency_hz': defaults.get(
                'dac_duc_mixer_frequency_hz', 0.0),
            'adc_ddc_mixer_frequency_hz': defaults.get(
                'adc_ddc_mixer_frequency_hz', 0.0),
            'nyquist_zone_adc': nyquist_zone,
            'nyquist_zone_dac0': nyquist_zone,
            'nyquist_zone_dac1': nyquist_zone,
            'inverse_sinc_fir_mode_dac0': inverse_sinc_mode,
            'inverse_sinc_fir_mode_dac1': inverse_sinc_mode,
            'inverse_sinc_filter_enabled_dac0': bool(inverse_sinc_enabled),
            'inverse_sinc_filter_enabled_dac1': bool(inverse_sinc_enabled),
            'mixer_scale_1p0_dac0': defaults.get('dac_mixer_scale_1p0', False),
            'mixer_scale_1p0_dac1': defaults.get('dac_mixer_scale_1p0', False),
            'mixer_scale_1p0_adc': defaults.get('adc_mixer_scale_1p0', False),
            'qmc_settings_dac0': None,
            'qmc_settings_dac1': None,
            'qmc_settings_adc': None,
            'adc_cal_frozen': self.cal_freeze,
            'rts_events': {'rts_available': False},
        }

    def _info_pipeline(self):
        defaults = self.config.get('firmware', {}).get('defaults', {})
        return {
            'ready': True,
            'output_mode': 'PSB',
            'sync_delay': defaults.get('sync_delay', 0),
            'tx_rx_skew': defaults.get('tx_rx_skew', 0),
            'buffer_switch_skew': defaults.get('buffer_switch_skew', 0),
            'internal_loopback': defaults.get('internal_loopback', False),
            'psb_scale': defaults.get('psb_scale', 1),
            'psb_fftshift': defaults.get('psb_fftshift', 0),
            'pfb_fftshift': defaults.get('pfb_fftshift', 0),
            'acc_len': self.acc_len,
            'acc_freq_hz': self.sample_rate,
        }

    def _info_tones(self):
        details = self._tone_frequency_details()
        metadata = self._tone_metadata()
        defaults = self.config.get('firmware', {}).get('defaults', {})
        blind_spans = (
            np.atleast_1d(defaults.get('blind_spans', [])).astype(float).tolist()
            if metadata['metadata_matches_config'] else []
        )
        return {
            'ready': True,
            'count': len(self.tone_frequencies),
            'frequencies_hz': list(self.tone_frequencies),
            'amplitudes': list(self.tone_amplitudes),
            'phases_rad': list(self.tone_phases),
            'powers_dbm': list(self.tone_powers_dbm),
            'powers_reference_plane': 'detector',
            'firmware_indices': details['rx']['tone_indices'],
            'blind_spans': blind_spans,
            **metadata,
            'detailed_frequency_info': details,
        }

    def _info_rf_frontend(self):
        rf_cfg = self.config.get('rf_frontend', {}) or {}
        mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}
        bypass_cfg = rf_cfg.get('bypass_amps', {}) or {}

        def cal_mean(value):
            """Mean S21 (dB) from a calibration entry, taking column 1 of an
            (freq, value) table; ``None`` for missing/empty/non-numeric data."""
            if value is None:
                return None
            if isinstance(value, str):
                try:
                    return float(value)
                except ValueError:
                    return None
            try:
                arr = np.asarray(value, dtype=float)
            except (TypeError, ValueError):
                return None
            if arr.size == 0:
                return None
            if arr.ndim == 0:
                values = np.asarray([float(arr)])
            elif arr.ndim >= 2 and arr.shape[-1] >= 2:
                values = arr[..., 1]
            else:
                values = arr
            finite = values[np.isfinite(values)]
            return None if finite.size == 0 else float(np.mean(finite))

        def amp_cal(path, bypassed):
            """Modelled amp S21 (dB) for a TX/RX path in the given bypass state,
            using the direct cal if present or the enabled/bypass delta otherwise."""
            direct_key = (
                f'{path}_amp_bypassed_s21_db'
                if bypassed else f'{path}_amp_enabled_s21_db'
            )
            direct = cal_mean(mixerless_cfg.get(direct_key))
            if direct is not None:
                return direct
            delta = cal_mean(mixerless_cfg.get(f'{path}_amp_bypass_delta_s21_db'))
            if delta is None:
                return None
            enabled = cal_mean(mixerless_cfg.get(f'{path}_amp_enabled_s21_db'))
            bypass_state = cal_mean(
                mixerless_cfg.get(f'{path}_amp_bypassed_s21_db'))
            if bypassed and enabled is not None:
                return enabled + delta
            if not bypassed and bypass_state is not None:
                return bypass_state - delta
            return None

        tx_amp_bypass = bool(bypass_cfg.get('tx_amp_bypass', False))
        rx_amp_bypass = bool(bypass_cfg.get('rx_amp_bypass', False))
        tx_amp_cal = amp_cal('tx', tx_amp_bypass)
        rx_amp_cal = amp_cal('rx', rx_amp_bypass)
        tx_total_model = 0.0
        rx_total_model = 0.0
        tx_total_cal = tx_amp_cal if tx_amp_cal is not None else None
        rx_total_cal = rx_amp_cal if rx_amp_cal is not None else None
        info = {
            'ready': True,
            'connected': rf_cfg.get('connected', False),
            'hardware_id': rf_cfg.get('hardware_id'),
            'hardware_available': False,
            'controllable': True,
            'supports_bypass_amps': bool(mixerless_cfg.get('connected', False)),
            'attenuator_backend': 'mock',
            'rf_channel': mixerless_cfg.get('rf_channel'),
            'tx_attenuation_db': 0.0,
            'rx_attenuation_db': 0.0,
            'tx_total_gain_db': (
                tx_total_cal if tx_total_cal is not None else tx_total_model
            ),
            'rx_total_gain_db': (
                rx_total_cal if rx_total_cal is not None else rx_total_model
            ),
            'tx_total_gain_source': (
                'calibrated_config'
                if tx_total_cal is not None else 'peripheral_model'
            ),
            'rx_total_gain_source': (
                'calibrated_config'
                if rx_total_cal is not None else 'peripheral_model'
            ),
            'tx_total_gain_model_db': tx_total_model,
            'rx_total_gain_model_db': rx_total_model,
            'tx_total_gain_calibrated_estimate_db': tx_total_cal,
            'rx_total_gain_calibrated_estimate_db': rx_total_cal,
            'tx_input_1db_comp_dbm': mixerless_cfg.get('tx_input_1db_comp_dbm'),
            'rx_input_1db_comp_dbm': mixerless_cfg.get('rx_input_1db_comp_dbm'),
        }
        if info['supports_bypass_amps']:
            tx_amp_effective = tx_amp_cal if tx_amp_cal is not None else 0.0
            rx_amp_effective = rx_amp_cal if rx_amp_cal is not None else 0.0
            info.update({
                'tx_amp_bypass': tx_amp_bypass,
                'rx_amp_bypass': rx_amp_bypass,
                'tx_bypass_amp_s21_db': tx_amp_effective,
                'rx_bypass_amp_s21_db': rx_amp_effective,
                'tx_bypass_amp_s21_source': (
                    'calibrated_config'
                    if tx_amp_cal is not None else 'peripheral_model'
                ),
                'rx_bypass_amp_s21_source': (
                    'calibrated_config'
                    if rx_amp_cal is not None else 'peripheral_model'
                ),
                'tx_bypass_amp_s21_model_db': 0.0,
                'rx_bypass_amp_s21_model_db': 0.0,
                'tx_bypass_amp_s21_calibrated_estimate_db': tx_amp_cal,
                'rx_bypass_amp_s21_calibrated_estimate_db': rx_amp_cal,
            })
        for key in ('tx_mixer_lo_frequency_hz', 'rx_mixer_lo_frequency_hz',
                    'tx_mixer_sideband', 'rx_mixer_sideband',
                    'tx_mixer_conversion_loss_db', 'rx_mixer_conversion_loss_db',
                    'tx_combiner_loss_db', 'rx_combiner_loss_db',
                    'tx_if_s21_db', 'rx_if_s21_db',
                    'tx_rf_s21_db', 'rx_rf_s21_db', 'loopback'):
            info[key] = rf_cfg.get(key)
        for key in ('tx_amp_enabled_s21_db', 'tx_amp_bypassed_s21_db',
                    'tx_amp_bypass_delta_s21_db',
                    'rx_amp_enabled_s21_db', 'rx_amp_bypassed_s21_db',
                    'rx_amp_bypass_delta_s21_db',
                    'tx_input_1db_comp_dbm', 'rx_input_1db_comp_dbm',
                    'tx_group_delay_ns', 'rx_group_delay_ns'):
            info[key] = mixerless_cfg.get(key)
        return info

    def _info_lna(self):
        cryo_cfg = self.config.get('cryostat', {}) or {}
        lna_cfg = cryo_cfg.get('lna_bias', {}) or {}
        return {
            'ready': True,
            'enabled': lna_cfg.get('enabled', False),
            'cryostat_connected': cryo_cfg.get('connected', False),
            'hardware_available': False,
            'controllable': True,
            'backend': 'mock',
            'lna_channel': lna_cfg.get('channel'),
            'bias_voltage_v': lna_cfg.get('voltage_v'),
            'soft_off': False,
            'method': lna_cfg.get('method', 'target_current'),
            'blind': lna_cfg.get('blind', False),
            'lna_model': cryo_cfg.get('lna_model'),
            'bias_readings': None,
        }

    def _info_rfsoc_sensors(self):
        return {'ready': True, 'available': False, 'sensor_path': None}

    def _info_diagnostics(self):
        return {
            'ready': True,
            'adc_saturation': {'saturated': False, 'mock': True},
            'dac_saturation': {'saturated': False, 'mock': True},
            'dsp_overflow': {'overflow': False, 'mock': True},
        }

    def _info_config(self):
        config_text = yaml.dump(self.config, sort_keys=False)
        config_id = None
        try:
            config_id = self.config.get('config', {}).get('config_id')
        except Exception:
            pass
        return {
            'ready': True,
            'config_file': self.client.config_file,
            'config_id': config_id,
            'config_text': config_text,
            'config_matches_applied': True,
        }

    def _info_calibrations(self):
        return {'ready': True, 'calibration_files': list(self.calibration_files)}

    def _info_resonators(self):
        driven = np.asarray(self.tone_frequencies, dtype=float)
        if driven.size:
            catalog_f = self._resonator_catalog['frequency_hz']
            nearest = np.array([
                catalog_f[np.argmin(np.abs(catalog_f - frequency))]
                for frequency in driven
            ])
            detuning = driven - nearest
            fractional_detuning = detuning / nearest
            accumulated_phase = np.zeros_like(driven)
            tracking_timestamp = time.time()
        else:
            nearest = None
            detuning = None
            fractional_detuning = None
            accumulated_phase = None
            tracking_timestamp = None

        return {
            'ready': True,
            'tracking_enabled': False,
            'tone_count': int(driven.size),
            'driven_frequencies_hz': driven.tolist(),
            'estimated_resonant_frequencies_hz': (
                nearest.tolist() if nearest is not None else None),
            'detuning_hz': detuning.tolist() if detuning is not None else None,
            'fractional_detuning': (
                fractional_detuning.tolist()
                if fractional_detuning is not None else None),
            'accumulated_phase_rad': (
                accumulated_phase.tolist()
                if accumulated_phase is not None else None),
            'tracking_timestamp': tracking_timestamp,
            'tracking_interval_s': None,
        }

    def _info_registers(self):
        return {'ready': False, 'available': False, 'dump': None}

    def health_check(self):
        return {
            'initialisation_level': 'pipeline',
            'clock_locked': True,
            'timing_ready': True,
            'timing_state': 'mock',
            'streaming': self.is_streaming,
            'triggered_streaming': self.triggered_stream_enabled,
            'sweeping': False,
            'rts_events': False,
            'adc_saturated': False,
            'dac_saturated': False,
            'dsp_overflow': False,
            'rf_frontend_available': False,
            'lna_available': False,
            'tone_count': len(self.tone_frequencies),
            'client_count': 1,
            'resonators_tracking': (self._tracking is not None
                                    and not self._tracking.get('dry_run',
                                                               True)),
            'max_detuning_hz': None,
            'tracking': (None if self._tracking is None
                         else dict(self._tracking.get('summary') or {})),
        }

    def get_parameter(self, param_name, message=None):
        """Return a mock value for a parameter-server style ``get`` request.

        ``param_name`` is one of the server-registered names handled below
        (the same names as :py:meth:`ReadoutClient.get_parameter`);
        ``message`` is the optional full request dict (request metadata such
        as ``reference_plane`` is read from it where relevant).
        """
        if param_name == 'sample_rate_hz':
            return self.sample_rate
        if param_name == 'tone_frequencies':
            return list(self.tone_frequencies)
        if param_name == 'tone_frequencies_detailed':
            return self._tone_frequency_details()
        if param_name == 'tone_metadata':
            return self._tone_metadata()
        if param_name == 'tone_amplitudes':
            return list(self.tone_amplitudes)
        if param_name == 'tone_phases':
            return list(self.tone_phases)
        if param_name == 'tone_powers':
            return list(self.tone_powers_dbm)
        if param_name == 'tone_powers_detailed':
            return {
                'powers_dbm': list(self.tone_powers_dbm),
                'reference_plane': (message or {}).get('reference_plane', 'detector'),
                'mock': True,
            }
        if param_name == 'telescope_time':
            return int(time.time_ns())
        if param_name == 'cal_freeze':
            return self.cal_freeze
        if param_name == 'clock_source':
            return self.clock_source
        if param_name == 'clock_status':
            return {'all_locked': True, 'chips': []}
        pipeline = self._info_pipeline()
        if param_name in pipeline:
            return pipeline[param_name]
        return None

    def set_parameter(self, param_name, param_value):
        """Update mock state for a parameter-server style ``set`` request.

        ``param_name`` / ``param_value`` follow
        :py:meth:`ReadoutClient.set_parameter` (the names handled below).
        """
        if param_name == 'sample_rate_hz':
            # Mirror the real server: store the derived acc_len, not the rate.
            self.acc_len = self._acc_len_for_rate(float(param_value))
        elif param_name == 'acc_len':
            self.acc_len = int(param_value)
        elif param_name == 'tone_frequencies':
            self.tone_frequencies = np.atleast_1d(param_value).astype(float).tolist()
            self._resize_tone_state(len(self.tone_frequencies))
        elif param_name == 'tone_amplitudes':
            self.tone_amplitudes = self._per_tone_list(
                param_value, len(self.tone_frequencies), 1.0)
        elif param_name == 'tone_phases':
            self.tone_phases = self._per_tone_list(
                param_value, len(self.tone_frequencies), 0.0)
        elif param_name == 'tone_powers':
            self.tone_powers_dbm = self._per_tone_list(
                param_value, len(self.tone_frequencies), -60.0)
        elif param_name == 'cal_freeze':
            self.cal_freeze = bool(param_value)
        elif param_name == 'clock_source':
            self.clock_source = str(param_value)
        elif param_name == 'internal_loopback':
            defaults = self.config.setdefault('firmware', {}).setdefault('defaults', {})
            defaults['internal_loopback'] = bool(param_value)
        return {'status': 'success'}

    def _resonator_z(self, f, f0, w, scale=1000.0):
        """
        Simple resonator S21 model used as demod ground truth.

        Phase ``-2*arctan(2*(f - f0)/w)`` has its inflection (steepest slope,
        d2phi/df2 = 0) at the resonance ``f0``; the magnitude dips at ``f0``.
        The magnitude is the notch-power profile ``sqrt((b + x**2)/(1 + x**2))``,
        whose half-power FWHM is ``w`` -- so the magnitude linewidth matches the
        phase-slope linewidth ``4/w``, as for a real resonator.

        Parameters
        ----------
        f : numpy.ndarray
            Probe frequencies (Hz).
        f0 : numpy.ndarray
            Per-tone resonance frequencies (Hz).
        w : float
            Resonator FWHM linewidth (Hz).
        scale : float, optional
            Amplitude scale (~int counts). Default 1000.
        """
        x = 2.0 * (np.asarray(f, dtype=float) - np.asarray(f0, dtype=float)) / w
        phi = -2.0 * np.arctan(x)
        mag = np.sqrt((0.01 + x ** 2) / (1.0 + x ** 2))   # dip to 0.1 at resonance
        return scale * mag * np.exp(1j * phi)

    def _mock_expand_offsets(self, offsets, mod_indices, n_tones):
        """Expand user offsets to a full ``(n_points, n_tones)`` matrix (0 for
        non-modulated tones).

        ``offsets`` shape ``(n_points,)`` (broadcast) or ``(n_points,
        len(mod_indices))``; ``mod_indices`` the modulated tone indices;
        ``n_tones`` the active tone count.
        """
        # 1D of length n_points => one offset per point (broadcast across tones):
        # reshape to (n_points, 1), NOT (1, n_points).
        offsets = np.asarray(offsets, dtype=float)
        if offsets.ndim == 1:
            offsets = offsets[:, None]
        n_points = offsets.shape[0]
        full = np.zeros((n_points, n_tones), dtype=float)
        if offsets.shape[1] in (1, len(mod_indices)):
            full[:, mod_indices] = offsets
        else:
            raise ValueError(f'offsets has {offsets.shape[1]} columns; '
                             f'expected 1 or len(mod_indices)={len(mod_indices)}')
        return full, n_points

    def _mock_modulation_state(self, center, full_offsets, mod_indices, spp, n_settle,
                               armed_bin_center, autosync=False, mrst=False):
        """Assemble a ``tone_modulation`` state dict mirroring the server's shape,
        with per-(tone,point) bin occupancy relative to the fixed mock armed bins.

        Parameters
        ----------
        center : numpy.ndarray
            Per-tone centre frequencies (Hz).
        full_offsets : numpy.ndarray
            ``(n_points, n_tones)`` probe offsets (Hz).
        mod_indices : list
            Modulated tone indices (user order).
        spp, n_settle : int
            Dwell and settling-sample counts.
        armed_bin_center : numpy.ndarray
            Per-tone armed bin-centre frequencies (Hz), held fixed across updates.
        autosync : bool
            Mocked modulation sync mode, mirrored in the state payload.
        mrst : bool
            Mocked per-step master-reset mode, mirrored in the state payload.
        """
        n_tones = len(center)
        n_points = full_offsets.shape[0]
        probe = center[None, :] + full_offsets
        drift = (probe - armed_bin_center[None, :]) / self._mod_bin_hz   # channels
        ad = np.abs(drift)
        occ = np.where(ad <= 0.5, 'nearest', np.where(ad <= 1.0, 'second', 'beyond'))
        beyond = sorted({i for i in range(n_tones) if 'beyond' in set(occ[:, i])})
        beyond_half = sorted({
            i for i in range(n_tones)
            if {'second', 'beyond'} & set(occ[:, i])
        })
        warnings_list = []
        if beyond_half:
            msg = (
                'modulation point(s) for tone(s) '
                f'{beyond_half} are more than half an FFT bin from their armed '
                'bin; channel maps remain fixed, so those points ride the '
                'overlapping channel response (occupancy "second").'
            )
            if beyond:
                msg += (
                    ' Tone(s) '
                    f'{beyond} are more than one FFT bin away and need '
                    'recenter_modulation() or smaller offsets.'
                )
            warnings_list.append(msg)
        tones = []
        for i in range(n_tones):
            tones.append({
                'index': i, 'firmware_index': i,
                'center_hz': float(center[i]),
                'armed_fft_bin': int(round(armed_bin_center[i] / self._mod_bin_hz)),
                'offsets_hz': full_offsets[:, i].tolist(),
                'drift_bins': drift[:, i].tolist(),
                'occupancy': [str(o) for o in occ[:, i]],
            })
        return {
            'enabled': bool(self._mod_enabled),
            'desired_revision': int(self._mod_revision),
            'applied_revision': int(self._mod_revision),
            'revision_history': dict(self._mod_revision_history),
            'num_points': int(n_points), 'samples_per_point': int(spp), 'n_settle': int(n_settle),
            'autosync': bool(autosync),
            'mrst': bool(mrst),
            'mod_indices': list(mod_indices),
            'sample_rate_hz': float(self.sample_rate),
            'cycle_rate_hz': float(self.sample_rate) / (n_points * spp) if n_points else float('nan'),
            'needs_recenter': bool(len(beyond) > 0),
            'any_beyond_half_bin': bool(beyond_half),
            'tones_beyond_half_bin': beyond_half,
            'tones_beyond_coverage': beyond,
            'warnings': warnings_list,
            'tones': tones,
        }

    def _mock_arm(self, center, offsets, mod_indices, spp, n_settle, reload_bins, set_f0,
                  autosync=None, mrst=None):
        """Resolve and **commit** an armed modulation config (bumps the revision).

        Parameters
        ----------
        center, offsets, mod_indices : array-like or None
            New config; ``None`` reuses the resident value where sensible.
        spp, n_settle : int
            Dwell and settling-sample counts.
        reload_bins : bool
            Recompute the armed bin centres from ``center`` (enable/recenter) vs
            reuse the existing ones (live update riding the overlap).
        set_f0 : bool
            Capture ``center`` as the "true" resonance frequencies (enable only),
            so later centre updates detune relative to them.
        autosync : bool or None
            Sync mode to store. ``None`` preserves the resident mode, defaulting
            to ``True`` for a fresh arm.
        mrst : bool or None
            Per-step master-reset mode to store. ``None`` preserves the resident
            mode, defaulting to ``False`` for a fresh arm.
        """
        freqs = np.asarray(self.tone_frequencies, dtype=float)
        n_tones = len(freqs)
        if center is None:
            center = self._mod['center'].copy() if self._mod is not None else freqs.copy()
        else:
            center = np.asarray(center, dtype=float)
        if mod_indices is None:
            mod_indices = list(self._mod['mod_indices']) if self._mod is not None else list(range(n_tones))
        else:
            mod_indices = [int(i) for i in np.atleast_1d(mod_indices)]
        if offsets is None and self._mod is not None:
            full = self._mod['offsets'].copy()
            n_points = full.shape[0]
            raw_off = full
        else:
            full, n_points = self._mock_expand_offsets(offsets, mod_indices, n_tones)
            raw_off = np.atleast_2d(np.asarray(offsets, dtype=float))
        if set_f0 or self._mod_f0 is None:
            self._mod_f0 = center.copy()
        if reload_bins or self._mod_armed_bin_center is None:
            self._mod_armed_bin_center = np.round(center / self._mod_bin_hz) * self._mod_bin_hz
        if autosync is None:
            autosync = self._mod.get('autosync', False) if self._mod is not None else False
        autosync = bool(autosync)
        if mrst is None:
            mrst = self._mod.get('mrst', False) if self._mod is not None else False
        mrst = bool(mrst)
        self._mod_revision = (self._mod_revision + 1) & 0x7FFF
        self._mod_revision_history[self._mod_revision] = {
            'center': center.tolist(), 'offsets': raw_off.tolist(),
            'mod_indices': list(mod_indices), 'autosync': autosync, 'mrst': mrst, 'ts': time.time()}
        state = self._mock_modulation_state(center, full, mod_indices, spp, n_settle,
                                            self._mod_armed_bin_center, autosync=autosync, mrst=mrst)
        self._mod = {'center': center, 'offsets': full, 'mod_indices': mod_indices,
                     'n_points': n_points, 'samples_per_point': int(spp),
                     'n_settle': int(n_settle), 'autosync': autosync, 'mrst': mrst,
                     'revision': self._mod_revision, 'state': state}
        return state

    def _info_tone_modulation(self):
        """Return the cached mock software-modulation state (``get_info('modulation')``, sw engine)."""
        if self._mod is None:
            return {'enabled': False, 'num_points': 0, 'tones': []}
        state = dict(self._mod['state'])
        state['enabled'] = bool(self._mod_enabled)
        return state

    def _mock_fw_state(self, center, full_offsets, mod_indices, n_dwell, n_settle, mode):
        """Assemble a ``fw_modulation`` state dict mirroring the server's shape.

        Each row of ``full_offsets`` is one LO slot; occupancy is computed against
        the mock fixed channel grid just like the software-mod state. Includes the
        ``num_points`` / ``samples_per_point`` / ``n_settle`` / per-tone
        ``offsets_hz`` aliases so the state is drop-in usable with
        ``modulation.group_cycles``.
        """
        center = np.asarray(center, dtype=float)
        n_tones = len(center)
        n_slots = full_offsets.shape[0]
        armed = np.round(center / self._mod_bin_hz) * self._mod_bin_hz
        drift = (center[None, :] + full_offsets - armed[None, :]) / self._mod_bin_hz
        ad = np.abs(drift)
        occ = np.where(ad <= 0.5, 'nearest', np.where(ad <= 1.0, 'second', 'beyond'))
        beyond = sorted({i for i in range(n_tones) if 'beyond' in set(occ[:, i])})
        beyond_half = sorted({
            i for i in range(n_tones) if {'second', 'beyond'} & set(occ[:, i])})
        tones = []
        for i in range(n_tones):
            tones.append({
                'index': i, 'firmware_index': i, 'center_hz': float(center[i]),
                'slot_offsets_hz': full_offsets[:, i].tolist(),
                'offsets_hz': full_offsets[:, i].tolist(),
                'occupancy': [str(o) for o in occ[:, i]],
            })
        return {
            'enabled': bool(self._fw_mod_enabled), 'mode': str(mode),
            'slot': 0 if mode == 'manual' else None,
            'applied_revision': int(self._fw_mod_revision),
            'n_slots': int(n_slots), 'n_dwell': int(n_dwell),
            'num_points': int(n_slots), 'samples_per_point': int(n_dwell),
            'n_settle': int(n_settle),
            'sample_rate_hz': float(self.sample_rate),
            'cycle_rate_hz': (float(self.sample_rate) / (n_slots * n_dwell)
                              if (n_slots and n_dwell) else float('nan')),
            'mod_indices': list(mod_indices),
            'needs_recenter': bool(len(beyond) > 0),
            'any_beyond_half_bin': bool(beyond_half),
            'tones_beyond_half_bin': beyond_half, 'tones_beyond_coverage': beyond,
            'warnings': [], 'tones': tones,
        }

    def _info_fw_modulation(self):
        """Return the cached mock firmware-slot state (``get_info('modulation')``, fw engine)."""
        if self._fw_mod is None:
            return {'enabled': False, 'n_slots': 0, 'tones': []}
        state = dict(self._fw_mod['state'])
        state['enabled'] = bool(self._fw_mod_enabled)
        return state

    def _info_modulation(self):
        """Unified mock modulation state ('engine' field), mirroring the server."""
        if self._fw_mod_enabled or (self._fw_mod is not None and not self._mod_enabled):
            engine, state = 'fw', self._info_fw_modulation()
        elif self._mod_enabled or self._mod is not None:
            engine, state = 'sw', self._info_tone_modulation()
        else:
            return {'engine': None, 'enabled': False, 'num_points': 0, 'tones': []}
        state = dict(state)
        state['engine'] = engine
        return state

    def _info_tracking(self):
        """Mock ``get_info('tracking')`` payload (status-level)."""
        if self._tracking is None:
            return {'enabled': False}
        return dict(self._tracking)

    def stream_frame(self, num_tones=None):
        """Build one modern stream packet: IQ words plus six flags, TT, cnt, err.

        When modulation is armed and enabled, this cycles through the N probe
        points, evaluates the resonator model at each tone's probe frequency
        (centre + offset), and stamps the flag5 modulation tag (point/settling/
        revision). Otherwise it emits the legacy synthetic frame with flag5 = 0.

        ``num_tones`` sets how many tone IQ words to include; ``None`` uses the
        current active tone count.
        """
        if num_tones is None:
            num_tones = len(self.tone_frequencies)

        if self._mod is not None and self._mod_enabled:
            mod = self._mod
            N = mod['n_points']
            spp = max(1, int(mod['samples_per_point']))
            point = (self.packet_counter // spp) % N
            settling = (self.packet_counter % spp) < int(mod['n_settle'])
            center = np.asarray(mod['center'], dtype=float)[:num_tones]
            offs = np.asarray(mod['offsets'], dtype=float)[point][:num_tones]
            f0 = np.asarray(self._mod_f0, dtype=float)[:num_tones]
            z = self._resonator_z(center + offs, f0, self._mod_linewidth)
            i_words = np.round(z.real).astype('<i4')
            q_words = np.round(z.imag).astype('<i4')
            flag5 = (((int(point) + 1) & 0xFFFF)
                     | (int(bool(settling)) << 16)
                     | ((int(mod['revision']) & 0x7FFF) << 17))
        elif self._fw_mod is not None and self._fw_mod_enabled:
            # Firmware-slot modulation: the (mock) firmware picks the live slot
            # -- auto round-robin every n_dwell accumulations, or the manually
            # selected slot. Evaluate the resonator model at that slot's probe
            # and tag the frame with slot+1 in the same flag5 field.
            fw = self._fw_mod
            n_slots = fw['n_slots']
            n_dwell = max(1, int(fw['n_dwell']))
            if fw['mode'] == 'manual':
                slot = int(self._fw_mod_slot) % n_slots
            else:
                slot = (self.packet_counter // n_dwell) % n_slots
            # Flag the leading n_settle accumulations of each dwell as settling,
            # detected by the slot tag changing (mirrors the server).
            if slot != self._fw_dwell_slot:
                self._fw_dwell_slot = slot
                self._fw_dwell_pos = 0
            else:
                self._fw_dwell_pos += 1
            settling = self._fw_dwell_pos < int(fw.get('n_settle', 0))
            center = np.asarray(fw['center'], dtype=float)[:num_tones]
            offs = np.asarray(fw['offsets'], dtype=float)[slot][:num_tones]
            f0 = np.asarray(self._fw_mod_f0, dtype=float)[:num_tones]
            z = self._resonator_z(center + offs, f0, self._mod_linewidth)
            i_words = np.round(z.real).astype('<i4')
            q_words = np.round(z.imag).astype('<i4')
            flag5 = (((int(slot) + 1) & 0xFFFF)
                     | (int(bool(settling)) << 16)
                     | ((int(fw['revision']) & 0x7FFF) << 17))
        else:
            tone_idx = np.arange(num_tones, dtype=float)
            phase = 0.07 * self.packet_counter + tone_idx
            i_words = np.round(1000.0 * np.sin(phase)).astype('<i4')
            q_words = np.round(1000.0 * np.cos(phase)).astype('<i4')
            flag5 = 0

        iq_words = np.empty(2*num_tones, dtype='<i4')
        iq_words[0::2] = i_words
        iq_words[1::2] = q_words

        tt = int(time.time_ns())
        tail_u = np.zeros(10, dtype='<u4')
        tail_u[5] = np.uint32(flag5)   # flag5 (frame[-5]) carries the modulation tag
        tail_u[6] = (tt >> 32) & 0xFFFFFFFF
        tail_u[7] = tt & 0xFFFFFFFF
        tail_u[8] = self.packet_counter & 0xFFFFFFFF
        tail_u[9] = 0
        self.packet_counter += 1
        return np.concatenate((iq_words, tail_u.view('<i4'))).tobytes()

    def get_samples(self, num_samples, incl_system_info=True, burst=False):
        """Return mock raw samples in the same shape as ReadoutClient.get_samples.

        ``num_samples``, ``incl_system_info`` and ``burst`` mean the same as in
        :py:meth:`ReadoutClient.get_samples`.
        """
        self.client._warn_zero_phases()
        num_tones = len(self.tone_frequencies)
        data_raw = bytearray()
        t0 = time.time()
        frame_bytes = 0
        for _ in range(num_samples):
            frame = self.stream_frame(num_tones)
            frame_bytes = len(frame)
            data_raw.extend(frame)
        # Throttle to the real acquisition time (num_samples / sample_rate) so
        # the mock doesn't return far faster than hardware. burst is a single
        # fast transfer on real hardware, so it is left unthrottled.
        if not burst and self.sample_rate > 0:
            desired_time = num_samples / self.sample_rate
            elapsed = time.time() - t0
            if elapsed < desired_time:
                time.sleep(desired_time - elapsed)
        t1 = time.time()
        print(f"MOCK: Received {num_samples} samples in ~{t1-t0} seconds "
              f"(~{num_samples/(t1-t0)} samples per second)")
        info = self.info('all') if incl_system_info else {}
        return {
            'data_raw': data_raw,
            'sample_rate': self.sample_rate,
            'info': info,
            'frame_bytes': frame_bytes,
        }

    def stream_metadata(self, num_tones, info):
        """Metadata sidecar shared by mock binary and G3 stream receivers.

        ``num_tones`` is the active tone count recorded in the metadata, and
        ``info`` is the system-info dict to embed.
        """
        return {
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
            'num_tones': num_tones,
            'sample_rate': self.sample_rate,
            'format': '<i4',
            'index_err': 2*num_tones-1+10,
            'index_cnt': 2*num_tones-1+9,
            'index_tt_lsb': 2*num_tones-1+8,
            'index_tt_msb': 2*num_tones-1+7,
            'index_flag_5': 2*num_tones-1+6,
            'index_flag_4': 2*num_tones-1+5,
            'index_flag_3': 2*num_tones-1+4,
            'index_flag_2': 2*num_tones-1+3,
            'index_flag_1': 2*num_tones-1+2,
            'index_flag_0': 2*num_tones-1+1,
            'ordering': (
                'I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, '
                'Q_tone1_sample0,..flags, tt_msb, tt_lsb, cnt, err .'
            ),
            'info': info,
        }

    def receive_stream(self, num_tones=None, filename=None, print_data=False):
        """Mock implementation of continuous binary stream capture.

        ``num_tones``, ``filename`` and ``print_data`` mean the same as in
        :py:meth:`ReadoutClient.receive_stream`.
        """
        iq_data = None
        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_stream')
            print(f"No filename specified, writing to {filename}")
        if not os.path.exists(os.path.dirname(os.path.abspath(filename))):
            os.makedirs(os.path.dirname(os.path.abspath(filename)))

        if num_tones is None:
            num_tones = len(self.tone_frequencies)
        info = self.info('all')
        metadata = self.stream_metadata(num_tones, info)

        with open(filename+'.json', 'w') as file:
            json.dump(metadata, file, indent=4)

        with open(filename, 'wb') as file:
            print(f"MOCK: Writing data to {filename}")
            t0 = time.time()
            count = 0
            # Throttle each frame to the accumulator period so the mock writes
            # at roughly the hardware rate rather than as fast as possible.
            frame_period = 1.0 / self.sample_rate if self.sample_rate > 0 else 0.0
            while True:
                try:
                    t0_frame = time.time()
                    frame = self.stream_frame(num_tones)
                    file.write(frame)
                    count += 1

                    if print_data:
                        all_data = np.frombuffer(frame, dtype='<i4')
                        i = all_data[:2*num_tones:2]
                        q = all_data[1:2*num_tones:2]
                        tail_u = all_data[-10:].view('<u4')
                        tt = (int(tail_u[6]) << 32) + int(tail_u[7])
                        cnt = int(tail_u[8])
                        err = int(tail_u[9])
                        iq_data = i+1j*q
                        print(f"MOCK: Received IQ data: err={err} cnt={cnt} "
                              f"tt={tt} {iq_data.tolist()}\r", end='', flush=True)

                    this_frame_time = time.time() - t0_frame
                    if this_frame_time < frame_period:
                        time.sleep(frame_period - this_frame_time)
                except KeyboardInterrupt:
                    break
                except Exception as e:
                    print(f"MOCK: Error receiving stream data: {e}")
                    print(traceback.format_exc())
                    break

        t1 = time.time()
        print()
        print(f"MOCK: Received {count} samples in ~{t1-t0} seconds "
              f"(~{count/(t1-t0)} samples per second)")
        return iq_data

    def receive_stream_g3(self, num_tones=None, filename=None, print_data=False,
                          kid_stream_id='UNSET', duration=30):
        """Mock implementation of ``receive_stream_g3`` using generated packets.

        ``num_tones``, ``filename``, ``print_data``, ``kid_stream_id`` and
        ``duration`` mean the same as in
        :py:meth:`ReadoutClient.receive_stream_g3`.
        """
        _require_g3()
        SOSTREAM_VERSION = 1
        num_sample_rows_per_frame = 400
        iq_data = None

        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_stream.g3')
            print(f"No filename specified, writing to {filename}")
        if not filename.endswith('.g3'):
            filename += '.g3'
        if not os.path.exists(os.path.dirname(os.path.abspath(filename))):
            os.makedirs(os.path.dirname(os.path.abspath(filename)))

        if num_tones is None:
            num_tones = len(self.tone_frequencies)
        info = self.info('all')
        metadata = self.stream_metadata(num_tones, info)

        pid = os.getpid()
        log_filename = filename[:-3] + '_log_' + str(pid) + '.log'
        logging.basicConfig(filename=log_filename, level=logging.INFO,
                            format='%(asctime)s : %(levelname)s : %(message)s')
        logger = logging.getLogger(__name__)

        # JL setting similar to Smurf at the the moment - some of our primary names won't exist.
        primary_names = [
            'UnixTime', 'FluxRampIncrement', 'FluxRampOffset', 'Counter0',
            'Counter1', 'Counter2', 'AveragingResetBits', 'FrameCounter',
            'TESRelaySetting']
        primary_idxs = {name: idx for idx, name in enumerate(primary_names)}

        with open(filename[:-3]+'.json', 'w') as file:
            json.dump(metadata, file, indent=4)

        logger.info('MOCK: Wrote JSON header to '+filename[:-3]+'.json')
        logger.info(f'MOCK: Will stream for duration {duration}')

        with spt3g.core.G3Writer(filename=filename) as writer:
            logger.info(f"MOCK: Writing data to {filename}")
            print(f"MOCK: Writing data to {filename}")
            t0 = time.time()
            session_id = int(t0)
            frame_count = 0
            count = 0

            fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Observation)
            fr['frame_num'] = frame_count
            fr['session_id'] = session_id
            fr['sostream_id'] = kid_stream_id
            fr['sostream_version'] = SOSTREAM_VERSION
            fr['stream_placement'] = 'start'
            fr['time'] = spt3g.core.G3Time(t0 * spt3g.core.G3Units.s)
            writer(fr)
            frame_count += 1

            fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Wiring)
            fr['frame_num'] = frame_count
            fr['session_id'] = session_id
            fr['sostream_id'] = kid_stream_id
            fr['sostream_version'] = SOSTREAM_VERSION
            fr['time'] = spt3g.core.G3Time(time.time() * spt3g.core.G3Units.s)
            fr['dump'] = True
            fr['status'] = json.dumps(metadata).encode()
            writer(fr)
            frame_count += 1

            sample_rate = metadata['sample_rate']
            total_rows = max(1, int(float(duration) * sample_rate))
            rows_remaining = total_rows
            # Throttle the mock writer so frames are emitted at roughly the
            # real hardware rate. ``sample_rate`` is the live accumulator
            # rate (set by acc_len on hardware); without this the mock writes
            # the whole file far faster than expected and the OCS agent warns
            # that streaming finished early.
            desired_frame_write_time = (
                num_sample_rows_per_frame / sample_rate if sample_rate > 0 else 0.0)
            chans = np.arange(num_tones)
            names = ['_']*(2*num_tones+6+1+1+1)
            names[0:2*len(chans):2] = [f'i{ch:0>4}' for ch in chans]
            names[1:2*len(chans):2] = [f'q{ch:0>4}' for ch in chans]
            names[num_tones*2:num_tones*2+6] = [
                f'flag{flag}' for flag in list(range(6))]
            names[num_tones*2+6] = 'telescope_time'
            names[num_tones*2+7] = 'cnt'
            names[num_tones*2+8] = 'err'

            while rows_remaining > 0:
                t0_frame = time.time()
                size_of_this_frame = min(num_sample_rows_per_frame, rows_remaining)
                rows = []
                for _ in range(size_of_this_frame):
                    frame = self.stream_frame(num_tones)
                    all_data = np.frombuffer(frame, dtype='<i4')
                    tail = all_data[-10:].astype(np.int64)
                    tail_u = all_data[-10:].view('<u4')
                    tt = (int(tail_u[6]) << 32) + int(tail_u[7])
                    cnt = int(tail_u[8])
                    err = int(tail_u[9])
                    full_row = np.concatenate((
                        all_data[:2*num_tones].astype(np.int64),
                        tail[:6],
                        np.array([tt, cnt, err], dtype=np.int64),
                    ))
                    rows.append(full_row)
                    count += 1

                    if print_data:
                        i = all_data[:2*num_tones:2]
                        q = all_data[1:2*num_tones:2]
                        iq_data = i+1j*q
                        print(f"MOCK: datalen {len(frame)} row frame count {len(rows)} "
                              f"Received IQ data: err={err} cnt={cnt} tt={tt} "
                              f"{iq_data.tolist()} \r", end='', flush=True)

                data_frame_buffer = np.column_stack(rows)
                start = t0 + (count - size_of_this_frame) / sample_rate
                times = start + np.arange(size_of_this_frame) / sample_rate
                g3times = spt3g.core.G3VectorTime(times * spt3g.core.G3Units.s)

                fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Scan)
                fr['data'] = so3g.G3SuperTimestream(names, g3times, data_frame_buffer)
                frame_counter = np.arange(0, size_of_this_frame, dtype=int)
                primary_data = np.zeros((len(primary_names), size_of_this_frame),
                                        dtype=np.int64)
                primary_data[primary_idxs['UnixTime'], :] = (times * 1e9).astype(int)
                primary_data[primary_idxs['FrameCounter'], :] = frame_counter
                fr['primary'] = so3g.G3SuperTimestream(primary_names, g3times,
                                                       primary_data)
                fr['timing_paradigm'] = 'High Precision'
                fr['num_samples'] = size_of_this_frame
                fr['frame_num'] = frame_count
                fr['session_id'] = session_id
                fr['sostream_id'] = kid_stream_id
                fr['sostream_version'] = SOSTREAM_VERSION
                fr['time'] = spt3g.core.G3Time(time.time() * spt3g.core.G3Units.s)
                writer(fr)

                this_frame_time = time.time() - t0_frame
                if this_frame_time < desired_frame_write_time:
                    time.sleep(desired_frame_write_time - this_frame_time)

                frame_count += 1
                rows_remaining -= size_of_this_frame

            logger.info(f'MOCK Streaming duration {duration} expired.')
            logger.info("MOCK: Writing final observation frame..")
            t1 = time.time()
            fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Observation)
            fr['frame_num'] = frame_count
            fr['session_id'] = session_id
            fr['sostream_id'] = kid_stream_id
            fr['sostream_version'] = SOSTREAM_VERSION
            fr['stream_placement'] = 'end'
            fr['time'] = spt3g.core.G3Time(t1 * spt3g.core.G3Units.s)
            writer(fr)

        print()
        print(f"MOCK: Received {count} samples in ~{t1-t0} seconds "
              f"(~{count/(t1-t0)} samples per second)")
        logger.info(f"MOCK: Received {count} samples in ~{t1-t0} seconds "
                    f"(~{count/(t1-t0)} samples per second)")
        return iq_data

    def sweep_data(self, centers=None, spans=None, points=11, samples_per_point=10):
        """Create base64-encoded sweep data compatible with parse_sweep_data.

        Parameters
        ----------
        centers : array-like or None, optional
            Per-tone sweep centre frequencies (Hz); defaults to the current
            tone frequencies.
        spans : array-like or None, optional
            Per-tone sweep spans (Hz); a default span is used when ``None``.
        points : int, optional
            Frequency points per tone (default 11).
        samples_per_point : int, optional
            Averaged samples per frequency point (default 10).
        """
        if centers is None:
            centers = self.tone_frequencies
        centers = np.asarray(centers, dtype=float).reshape(-1)
        spans = np.broadcast_to(np.atleast_1d(1.0e6 if spans is None else spans),
                                centers.shape).astype(float)
        spans = np.abs(spans)
        points = max(1, int(points))
        samples_per_point = max(1, int(samples_per_point))
        point_offsets = (
            np.array([0.0])
            if points == 1 else np.linspace(-0.5, 0.5, points)
        )
        offsets = point_offsets[:, None] * spans[None, :]
        sweep_f = centers[None, :] + offsets
        sweep_z = self._baseline_response(sweep_f).astype(np.complex128)
        self._apply_catalog_resonators(sweep_f, sweep_z)
        self._apply_local_sweep_resonators(sweep_f, sweep_z, centers, spans)

        noise_sigma = 0.006 / np.sqrt(samples_per_point)
        noise = (
            self.rng.normal(0.0, noise_sigma, sweep_z.shape)
            + 1j * self.rng.normal(0.0, noise_sigma, sweep_z.shape)
        )
        sweep_z += noise
        sweep_e = np.full_like(
            sweep_z, noise_sigma + 1j * noise_sigma, dtype=np.complex128)
        sweep_tt = (
            np.arange(points, dtype=np.uint64) + np.uint64(time.time_ns())
        )
        return {
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
            'num_tones': int(len(centers)),
            'num_points': points,
            'centers': centers.tolist(),
            'spans': spans.tolist(),
            'samples_per_point': samples_per_point,
            'info': self.info('all'),
            'sweep': {
                'f': base64.b64encode(sweep_f.astype('f8').tobytes()).decode(),
                'z': base64.b64encode(sweep_z.astype('complex128').tobytes()).decode(),
                'e': base64.b64encode(sweep_e.astype('complex128').tobytes()).decode(),
                'tt': base64.b64encode(sweep_tt.tobytes()).decode(),
            },
        }

    def send_request(self, message):
        """Emulate common readout-server requests for ``mock=True`` clients.

        ``message`` is the request dict the client would otherwise send over
        the socket (must contain a ``'request'`` key).
        """
        request = message.get('request')

        if request == 'get':
            value = self.get_parameter(message.get('param'), message)
            if value is None:
                return {'status': 'error',
                        'message': f"Invalid mock parameter {message.get('param')}"}
            return {'status': 'success', 'value': value}

        if request == 'set':
            return self.set_parameter(message.get('param'), message.get('value'))

        if request == 'get_info':
            return {'status': 'success', 'data': self.info(message.get('sections'))}

        if request == 'health_check':
            return {'status': 'success', 'data': self.health_check()}

        if request == 'get_timing_status':
            return {'status': 'success', 'data': self._info_timing()}

        # v7.10 firmware timed sync (mock): always "ready/aligned", small drift, and a
        # tracked last-armed target so timed_sync_check is coherent after timed_sync_arm.
        if request == 'timed_sync_needed':
            now = time.time()
            last_pps_unix_s = round(now)
            align_tol_s = float(message.get('align_tol_s', 1e-4))
            ever = self._last_timed_sync is not None
            return {'status': 'success', 'result': {
                'needs_sync': not ever,
                'reason': ('no timed sync has been armed since the server started' if not ever
                           else 'telescope time is aligned on the second boundary'),
                'ever_synced': ever,
                'can_sync': True, 'ptp_ready': True, 'pps_active': True,
                'readiness_reasons': [],
                'align_tol_s': align_tol_s,
                'pps_boundary_offset_s': 1e-6,
                'system_offset_s': last_pps_unix_s - now,
                'last_pps_tt': int(last_pps_unix_s * 307_200_000),
                'last_pps_unix_s': float(last_pps_unix_s),
                'last_pps_utc': unix_to_iso(last_pps_unix_s),
            }}

        if request == 'timed_sync_ready':
            target_unix_s = message.get('target_unix_s')
            seconds_from_now = message.get('seconds_from_now')
            if target_unix_s is not None and seconds_from_now is not None:
                return {'status': 'error',
                        'message': 'give target_unix_s OR seconds_from_now, not both'}
            server_now = time.time()
            if target_unix_s is not None:
                target_sec = float(int(round(target_unix_s)))
            else:
                lead = 5 if seconds_from_now is None else int(seconds_from_now)
                target_sec = float(int(server_now) + lead)
            target_future = target_sec > server_now + 1
            return {'status': 'success', 'result': {
                'ready': bool(target_future),
                'checks': {'ptp_ready': True, 'pps_active': True,
                           'target_future_server': target_future,
                           'target_future_firmware': target_future},
                'reasons': ([] if target_future else
                            [f"target {time.ctime(target_sec)} is not >1 s ahead of server time"]),
                'target_tt_unix_s': target_sec,
                'target_tt_utc': unix_to_iso(target_sec),
                'server_now_unix_s': server_now,
                'server_now_utc': unix_to_iso(server_now),
                'firmware_last_pps_unix_s': float(int(server_now)),
                'firmware_last_pps_utc': unix_to_iso(int(server_now)),
                'pps_boundary_offset_s': 1e-6,
            }}

        if request == 'timed_sync_arm':
            target_unix_s = message.get('target_unix_s')
            seconds_from_now = message.get('seconds_from_now')
            if target_unix_s is not None and seconds_from_now is not None:
                return {'status': 'error',
                        'message': 'give target_unix_s OR seconds_from_now, not both'}
            now = time.time()
            if target_unix_s is not None:
                target_sec = int(round(target_unix_s))
            else:
                lead = 5 if seconds_from_now is None else int(seconds_from_now)
                target_sec = int(now) + lead
            # 'auto' reloads only on the first arm (mock proxy for "TT not yet loaded").
            reload_tt = message.get('reload_tt', 'auto')
            do_reload = (self._last_timed_sync is None) if reload_tt == 'auto' else bool(reload_tt)
            result = {
                'target_tt_unix_s': float(target_sec),
                'target_tt_utc': unix_to_iso(target_sec),
                'target_tt_value': int(target_sec * 307_200_000),
                'clk_hz': 307_200_000,
                'countdown_remaining_s': float(target_sec - now),
                'armed_at_unix_s': now,
                'armed_at_utc': unix_to_iso(now),
                'reloaded_tt': do_reload,
                'tt_loaded_unix_s': (float(int(now)) if do_reload else None),
                'fired': bool(message.get('wait', False)),
            }
            self._last_timed_sync = result
            if do_reload:  # a reload re-zeroes the boundary offset -> new drift reference
                self._last_tt_load_unix_s = result['tt_loaded_unix_s']
            return {'status': 'success', 'result': result}

        if request == 'timed_sync_check':
            now = time.time()
            last_pps_unix_s = round(now)
            align_tol_s = float(message.get('align_tol_s', 1e-4))
            resample_drift = bool(message.get('resample_drift', False))
            pps_boundary_offset_s = 1e-6
            last_armed = self._last_timed_sync
            if last_armed is not None:
                last_sync_unix_s = float(last_armed.get('target_tt_unix_s', round(now)))
                time_since_last_sync = max(0.0, now - last_sync_unix_s)
            else:
                last_sync_unix_s = None
                time_since_last_sync = None
            # Drift references the last TT load, not the last sync.
            loaded = self._last_tt_load_unix_s
            time_since_tt_load = (now - loaded) if loaded is not None else None
            drift_ppm = (pps_boundary_offset_s / time_since_tt_load * 1e6
                         if time_since_tt_load is not None and time_since_tt_load >= 2.0 else None)
            # Fall back to the (mock) sampler when the instant drift is unavailable.
            drift_resampled_ppm = 1.6 if (resample_drift or drift_ppm is None) else None
            return {'status': 'success', 'result': {
                'aligned': bool(abs(pps_boundary_offset_s) < align_tol_s),
                'align_tol_s': align_tol_s,
                'pps_boundary_offset_s': pps_boundary_offset_s,
                'system_offset_s': last_pps_unix_s - now,
                'last_pps_tt': int(last_pps_unix_s * 307_200_000),
                'last_pps_unix_s': float(last_pps_unix_s),
                'last_pps_utc': unix_to_iso(last_pps_unix_s),
                'last_sync_tt': (int(last_sync_unix_s * 307_200_000) if last_sync_unix_s else None),
                'last_sync_unix_s': last_sync_unix_s,
                'last_sync_utc': unix_to_iso(last_sync_unix_s),
                'time_since_last_sync_s': time_since_last_sync,
                'tt_loaded_unix_s': loaded,
                'tt_loaded_utc': unix_to_iso(loaded),
                'time_since_tt_load_s': time_since_tt_load,
                'drift_offset_s': pps_boundary_offset_s,
                'drift_ppm': drift_ppm,
                'drift_resampled_ppm': drift_resampled_ppm,
                'strobe_healthy': True,
                'last_armed': last_armed,
            }}

        if request == 'set_telescope_time':
            now = time.time()
            last_pps_unix_s = round(now)
            # TT (re)loaded -> reset the drift reference to the PPS-aligned load second.
            self._last_tt_load_unix_s = float(last_pps_unix_s)
            return {'status': 'success', 'result': {
                'loaded': True,
                'aligned': True,
                'pps_boundary_offset_s': 1e-6,
                'system_offset_s': last_pps_unix_s - now,
                'last_pps_tt': int(last_pps_unix_s * 307_200_000),
                'last_pps_unix_s': float(last_pps_unix_s),
                'last_pps_utc': unix_to_iso(last_pps_unix_s),
                'clk_hz': 307_200_000,
            }}

        if request == 'pull_config':
            return {'status': 'success',
                    'config_filename': self.client.config_file or 'mock_config.yaml',
                    'config_contents': yaml.dump(self.config, sort_keys=False)}

        if request == 'push_config':
            self.config_on_rfsoc = message.get('config_contents', '')
            if self.config_on_rfsoc:
                self.client.config = yaml.safe_load(self.config_on_rfsoc)
                self.client.pipeline_id = self.client.config.get(
                    'firmware', {}).get('pipeline_id', 0)
            return {'status': 'success'}

        if request == 'push_calibration':
            self.calibration_files[message.get('cal_filename')] = (
                message.get('cal_contents', ''))
            return {'status': 'success'}

        if request == 'pull_calibration':
            filename = message.get('cal_filename')
            return {'status': 'success',
                    'cal_contents': self.calibration_files.get(filename, '')}

        if request in ('initialise_server', 'initialise_firmware',
                       'initialise_pipeline', 'ensure_ready', 'hard_reset',
                       'cancel', 'refresh_adc_cal', 'send_fake_trigger'):
            return {'status': 'success'}

        if request == 'enable_stream':
            self.is_streaming = True
            return {'status': 'success'}

        if request == 'disable_stream':
            self.is_streaming = False
            return {'status': 'success'}

        if request == 'enable_modulation':
            try:
                if (message.get('offsets') is None and message.get('center') is None
                        and self._mod is not None):
                    # Resume a resident config with no args.
                    if 'autosync' in message:
                        self._mod['autosync'] = bool(message['autosync'])
                        self._mod['state']['autosync'] = bool(message['autosync'])
                    if 'mrst' in message:
                        self._mod['mrst'] = bool(message['mrst'])
                        self._mod['state']['mrst'] = bool(message['mrst'])
                    self._mod_enabled = True
                    self._mod['state']['enabled'] = True
                    return {'status': 'success',
                            'warnings': list(self._mod['state'].get('warnings', [])),
                            'result': {
                                'revision': self._mod['revision'],
                                'needs_recenter': self._mod['state']['needs_recenter'],
                                'any_beyond_half_bin': self._mod['state']['any_beyond_half_bin'],
                                'tones_beyond_half_bin': self._mod['state']['tones_beyond_half_bin']}}
                spp = int(message.get('samples_per_point', 4))
                n_settle = int(message.get('n_settle', 1))
                autosync = bool(message.get('autosync', False))
                mrst = bool(message.get('mrst', False))
                force = bool(message.get('force', False))
                linewidth_hz = message.get('linewidth_hz')
                if message.get('offsets') is None:
                    raise ValueError('offsets required to arm modulation')
                old_mod = copy.deepcopy(self._mod)
                old_mod_enabled = self._mod_enabled
                old_revision = self._mod_revision
                old_revision_history = copy.deepcopy(self._mod_revision_history)
                old_mod_f0 = None if self._mod_f0 is None else self._mod_f0.copy()
                old_armed_bin_center = None if self._mod_armed_bin_center is None else self._mod_armed_bin_center.copy()
                state = self._mock_arm(message.get('center'), message.get('offsets'),
                                       message.get('mod_indices'), spp, n_settle,
                                       reload_bins=True, set_f0=True, autosync=autosync, mrst=mrst)
                if state['needs_recenter'] and not force:
                    self._mod = old_mod
                    self._mod_enabled = old_mod_enabled
                    self._mod_revision = old_revision
                    self._mod_revision_history = old_revision_history
                    self._mod_f0 = old_mod_f0
                    self._mod_armed_bin_center = old_armed_bin_center
                    raise ValueError(
                        'modulation offsets push at least one tone beyond fixed-bin coverage; '
                        'reduce offsets, use a sweep/cross-bin method, or pass force=True to '
                        'arm anyway (those tones will wrap to the other end of the bin)')
                self._mod_enabled = True
                state['enabled'] = True
                if linewidth_hz is not None:
                    # Pure metadata for the tracking loop (params_from_sweep's
                    # per-modulated-tone linewidths), as on the real server.
                    self._mod['linewidth_hz'] = list(
                        np.atleast_1d(np.asarray(linewidth_hz, dtype=float)))
                return {'status': 'success',
                        'warnings': list(state.get('warnings', [])),
                        'result': {
                            'revision': self._mod_revision,
                            'needs_recenter': state['needs_recenter'],
                            'forced': bool(state['needs_recenter']),
                            'any_beyond_half_bin': state['any_beyond_half_bin'],
                            'tones_beyond_half_bin': state['tones_beyond_half_bin'],
                            'tones_beyond_coverage': state['tones_beyond_coverage']}}
            except Exception as e:
                return {'status': 'error', 'message': str(e)}

        if request == 'update_modulation':
            if self._mod is None:
                return {'status': 'error', 'message': 'modulation not armed'}
            on_map_change = message.get('on_map_change', 'continue')
            try:
                # Tentatively evaluate occupancy on the *existing* armed bins
                # (ride the overlap) without committing, so a rejection has no
                # side effect.
                center = (self._mod['center'].copy() if message.get('center') is None
                          else np.asarray(message.get('center'), dtype=float))
                mod_indices = self._mod['mod_indices']
                if message.get('offsets') is None:
                    full = self._mod['offsets'].copy()
                else:
                    full, _ = self._mock_expand_offsets(message.get('offsets'), mod_indices, len(center))
                autosync = (
                    self._mod.get('autosync', False) if message.get('autosync') is None
                    else bool(message.get('autosync')))
                mrst = (
                    self._mod.get('mrst', False) if message.get('mrst') is None
                    else bool(message.get('mrst')))
                tentative = self._mock_modulation_state(
                    center, full, mod_indices, self._mod['samples_per_point'],
                    self._mod['n_settle'], self._mod_armed_bin_center,
                    autosync=autosync, mrst=mrst)
                if tentative['needs_recenter'] and on_map_change != 'recenter':
                    return {'status': 'error',
                            'message': 'update would push tones beyond bin coverage; '
                                       'recenter required',
                            'warnings': list(tentative.get('warnings', [])),
                            'result': {
                                'tones_beyond_half_bin': tentative['tones_beyond_half_bin'],
                                'tones_beyond_coverage': tentative['tones_beyond_coverage']}}
                reload_bins = bool(tentative['needs_recenter'])  # recenter path reloads bins
                state = self._mock_arm(message.get('center'), message.get('offsets'), None,
                                       self._mod['samples_per_point'], self._mod['n_settle'],
                                       reload_bins=reload_bins, set_f0=False, autosync=autosync, mrst=mrst)
                self._mod_enabled = True
                return {'status': 'success',
                        'warnings': list(state.get('warnings', [])),
                        'result': {
                            'revision': self._mod_revision,
                            'op': 'recenter' if reload_bins else 'update',
                            'any_beyond_half_bin': state['any_beyond_half_bin'],
                            'tones_beyond_half_bin': state['tones_beyond_half_bin']}}
            except Exception as e:
                return {'status': 'error', 'message': str(e)}

        if request == 'recenter_modulation':
            if self._mod is None:
                return {'status': 'error', 'message': 'modulation not armed'}
            autosync = (
                self._mod.get('autosync', False) if message.get('autosync') is None
                else bool(message.get('autosync')))
            mrst = (
                self._mod.get('mrst', False) if message.get('mrst') is None
                else bool(message.get('mrst')))
            state = self._mock_arm(self._mod['center'], None, None, self._mod['samples_per_point'],
                                   self._mod['n_settle'], reload_bins=True, set_f0=False,
                                   autosync=autosync, mrst=mrst)
            self._mod_enabled = True
            return {'status': 'success',
                    'warnings': list(state.get('warnings', [])),
                    'result': {
                        'revision': self._mod_revision,
                        'any_beyond_half_bin': state['any_beyond_half_bin'],
                        'tones_beyond_half_bin': state['tones_beyond_half_bin']}}

        if request == 'disable_modulation':
            self._mod_enabled = False
            if self._mod is not None:
                self._mod['state']['enabled'] = False
            return {'status': 'success'}

        # ----- FFM tone tracking (status-level mock) ----------------------
        # The real tracking loop lives in the server; the mock keeps enough
        # state (params, held/dry_run flags, empty staged sets) for client
        # code paths and get_info('tracking') to be exercised end-to-end.
        if request == 'enable_tracking':
            # Whichever engine is armed (fw first -- it is the default).
            if self._fw_mod is not None and self._fw_mod_enabled:
                engine, mod = 'fw', self._fw_mod
                n_points, revision = mod['n_slots'], mod['revision']
            elif self._mod is not None:
                engine, mod = 'sw', self._mod
                n_points, revision = mod['n_points'], mod['revision']
            else:
                return {'status': 'error',
                        'message': 'modulation must be armed (enable_modulation, '
                                   'either engine) before enable_tracking'}
            if int(n_points) < 3:
                return {'status': 'error',
                        'message': 'tracking needs >= 3 modulation points'}
            linewidth = message.get('linewidth_hz', mod.get('linewidth_hz'))
            if linewidth is None:
                return {'status': 'error',
                        'message': 'tracking requires per-tone linewidth_hz'}
            params = {k: v for k, v in message.items() if k != 'request'}
            n_tracked = len(mod['mod_indices'])
            self._tracking = {
                'enabled': True,
                'engine': engine,
                'dry_run': bool(message.get('dry_run', True)),
                'held': False,
                'hold_reason': None,
                'params': params,
                'cycles_seen': 0,
                'filtered_detuning_linewidths': {},
                # Health blocks mirroring the real get_info('tracking'):
                # the mock has no estimation loop, so tones report no_data.
                'tones': {int(i): {'state': 'no_data',
                                   'detuning_linewidths': None,
                                   'detuning_std_linewidths': None,
                                   'slope_ratio': None,
                                   'invalid_fraction': None,
                                   'staged_center_hz': None}
                          for i in mod['mod_indices']},
                'summary': {
                    'enabled': True, 'engine': engine,
                    'dry_run': bool(message.get('dry_run', True)),
                    'held': False, 'hold_reason': None,
                    'filter_settled': False,
                    'n_tracked': n_tracked, 'n_locked': 0, 'n_drifting': 0,
                    'n_recenter_pending': 0, 'n_unlocked': 0,
                    'n_no_data': n_tracked, 'n_over_threshold': 0,
                    'max_abs_detuning_linewidths': None,
                    'median_abs_detuning_linewidths': None,
                    'max_abs_detuning_hz': None,
                    'staged': {'lo': 0, 'bin': 0},
                    'commits': {'lo': 0, 'bin': 0}, 'backoffs': 0,
                    'last_estimate_age_s': None,
                    'applied_revision': revision,
                },
                'controller': {'staged': {'lo': {}, 'bin': {}},
                               'staged_counts': {'lo': 0, 'bin': 0}},
                'commits': {'lo': 0, 'bin': 0},
                'backoffs': 0,
                'applied_revision': revision,
            }
            return {'status': 'success', 'result': dict(self._tracking)}

        if request == 'disable_tracking':
            self._tracking = None
            return {'status': 'success'}

        if request == 'hold_tracking':
            if self._tracking is None:
                return {'status': 'error', 'message': 'tracking is not enabled'}
            self._tracking['held'] = True
            return {'status': 'success'}

        if request == 'resume_tracking':
            if self._tracking is None:
                return {'status': 'error', 'message': 'tracking is not enabled'}
            self._tracking['held'] = False
            return {'status': 'success'}

        if request == 'commit_tracking_updates':
            if self._tracking is None:
                return {'status': 'error', 'message': 'tracking is not enabled'}
            return {'status': 'success', 'result': {'commits': []}}

        if request == 'enable_fw_modulation':
            try:
                freqs = np.asarray(self.tone_frequencies, dtype=float)
                n_tones = len(freqs)
                if (message.get('offsets') is None and message.get('center') is None
                        and self._fw_mod is not None):
                    # Resume a resident config with no args.
                    fw = self._fw_mod
                    center = np.asarray(fw['center'], dtype=float)
                    full = np.asarray(fw['offsets'], dtype=float)
                    mod_indices = list(fw['mod_indices'])
                    n_dwell = int(message.get('n_dwell', fw['n_dwell']))
                    n_settle = int(message.get('n_settle', fw.get('n_settle', 0)))
                    mode = message.get('mode', fw['mode'])
                else:
                    if message.get('offsets') is None:
                        raise ValueError('offsets required to arm firmware-slot modulation')
                    center = (freqs.copy() if message.get('center') is None
                              else np.asarray(message['center'], dtype=float))
                    mod_indices = (list(range(n_tones)) if message.get('mod_indices') is None
                                   else [int(i) for i in np.atleast_1d(message['mod_indices'])])
                    full, _n = self._mock_expand_offsets(message['offsets'], mod_indices, n_tones)
                    n_dwell = int(message.get('n_dwell', 4))
                    n_settle = int(message.get('n_settle', 0))
                    mode = message.get('mode', 'auto')
                if not 0 <= n_settle < n_dwell:
                    raise ValueError(
                        f'n_settle must satisfy 0 <= n_settle < n_dwell ({n_dwell}), got {n_settle}')
                if not 1 <= full.shape[0] <= self._fw_n_slots:
                    raise ValueError(f'need 1..{self._fw_n_slots} slot-offset rows, got {full.shape[0]}')
                if mode == 'auto' and full.shape[0] != self._fw_n_slots:
                    raise ValueError(
                        f'auto mode round-robins all {self._fw_n_slots} LO slots, so it needs '
                        f'exactly {self._fw_n_slots} offset rows (repeat a value to reuse a '
                        f'frequency); got {full.shape[0]}. Use mode="manual" to load fewer slots.')
                force = bool(message.get('force', False))
                self._fw_mod_revision = (self._fw_mod_revision + 1) & 0x7FFF
                state = self._mock_fw_state(center, full, mod_indices, n_dwell, n_settle, mode)
                if state['needs_recenter'] and not force:
                    self._fw_mod_revision = (self._fw_mod_revision - 1) & 0x7FFF
                    raise ValueError(
                        'slot offsets push at least one tone beyond fixed-bin coverage; '
                        'reduce the offsets or pass force=True to arm anyway (those tones '
                        'will wrap to the other end of the bin)')
                self._fw_mod_f0 = center.copy()
                self._fw_mod = {'center': center, 'offsets': full, 'mod_indices': mod_indices,
                                'n_dwell': int(n_dwell), 'n_settle': int(n_settle),
                                'mode': str(mode), 'n_slots': int(full.shape[0]),
                                'revision': int(self._fw_mod_revision), 'state': state}
                if message.get('linewidth_hz') is not None:
                    # Pure metadata (params_from_sweep per-tone linewidths).
                    self._fw_mod['linewidth_hz'] = list(np.atleast_1d(
                        np.asarray(message['linewidth_hz'], dtype=float)))
                self._fw_mod_enabled = True
                self._fw_mod_slot = 0
                self._fw_dwell_slot = -1
                self._fw_dwell_pos = 0
                state['enabled'] = True
                return {'status': 'success', 'warnings': list(state.get('warnings', [])),
                        'result': {'revision': self._fw_mod_revision, 'mode': str(mode),
                                   'n_slots': int(full.shape[0]), 'n_dwell': int(n_dwell),
                                   'n_settle': int(n_settle),
                                   'needs_recenter': state['needs_recenter'],
                                   'any_beyond_half_bin': state['any_beyond_half_bin'],
                                   'tones_beyond_half_bin': state['tones_beyond_half_bin'],
                                   'tones_beyond_coverage': state['tones_beyond_coverage']}}
            except Exception as e:
                return {'status': 'error', 'message': str(e)}

        if request == 'set_fw_modulation_slot':
            if not self._fw_mod_enabled:
                return {'status': 'error',
                        'message': 'firmware-slot modulation not enabled; call '
                                   "enable_modulation(engine='fw', mode='manual') first"}
            slot = int(message['slot'])
            n_loaded = int(self._fw_mod['n_slots']) if self._fw_mod is not None else 0
            if not 0 <= slot < n_loaded:
                return {'status': 'error',
                        'message': f'slot must be in 0..{n_loaded - 1} (loaded slots), got {slot}'}
            self._fw_mod_slot = slot
            if self._fw_mod is not None:
                self._fw_mod['mode'] = 'manual'
                self._fw_mod['state']['mode'] = 'manual'
                self._fw_mod['state']['slot'] = slot
            return {'status': 'success', 'result': {'slot': slot}}

        if request == 'get_fw_modulation_slot':
            if not self._fw_mod_enabled:
                return {'status': 'error',
                        'message': 'firmware-slot modulation not enabled; call '
                                   "enable_modulation(engine='fw', mode='manual') first"}
            mode = self._fw_mod['mode'] if self._fw_mod is not None else 'auto'
            slot = self._fw_mod_slot if mode == 'manual' else None
            return {'status': 'success', 'result': {'slot': slot, 'mode': mode}}

        if request == 'update_fw_modulation':
            if self._fw_mod is None:
                return {'status': 'error', 'message': 'firmware-slot modulation not armed'}
            try:
                fw = self._fw_mod
                n_tones = len(np.asarray(self.tone_frequencies))
                center = (np.asarray(fw['center'], dtype=float) if message.get('center') is None
                          else np.asarray(message['center'], dtype=float))
                if message.get('offsets') is None:
                    full = np.asarray(fw['offsets'], dtype=float)
                else:
                    full, _n = self._mock_expand_offsets(message['offsets'], fw['mod_indices'], n_tones)
                n_dwell = int(message.get('n_dwell', fw['n_dwell']))
                n_settle = int(message.get('n_settle', fw.get('n_settle', 0)))
                mode = message.get('mode', fw['mode'])
                if not 0 <= n_settle < n_dwell:
                    raise ValueError(
                        f'n_settle must satisfy 0 <= n_settle < n_dwell ({n_dwell}), got {n_settle}')
                self._fw_mod_revision = (self._fw_mod_revision + 1) & 0x7FFF
                state = self._mock_fw_state(center, full, fw['mod_indices'], n_dwell, n_settle, mode)
                if state['needs_recenter'] and not bool(message.get('force', False)):
                    self._fw_mod_revision = (self._fw_mod_revision - 1) & 0x7FFF
                    raise ValueError('update pushes a tone beyond fixed-bin coverage; force=True to override')
                self._fw_mod = {'center': center, 'offsets': full, 'mod_indices': fw['mod_indices'],
                                'n_dwell': int(n_dwell), 'n_settle': int(n_settle),
                                'mode': str(mode), 'n_slots': int(full.shape[0]),
                                'revision': int(self._fw_mod_revision), 'state': state}
                state['enabled'] = bool(self._fw_mod_enabled)
                return {'status': 'success', 'warnings': list(state.get('warnings', [])),
                        'result': {'revision': self._fw_mod_revision, 'mode': str(mode),
                                   'n_slots': int(full.shape[0]), 'n_dwell': int(n_dwell),
                                   'n_settle': int(n_settle),
                                   'needs_recenter': state['needs_recenter'],
                                   'any_beyond_half_bin': state['any_beyond_half_bin'],
                                   'tones_beyond_half_bin': state['tones_beyond_half_bin']}}
            except Exception as e:
                return {'status': 'error', 'message': str(e)}

        if request == 'recenter_fw_modulation':
            if self._fw_mod is None:
                return {'status': 'error', 'message': 'firmware-slot modulation not armed'}
            try:
                fw = self._fw_mod
                n_tones = len(np.asarray(self.tone_frequencies))
                center = (np.asarray(fw['center'], dtype=float) if message.get('center') is None
                          else np.asarray(message['center'], dtype=float))
                if message.get('offsets') is None:
                    full = np.asarray(fw['offsets'], dtype=float)
                else:
                    full, _n = self._mock_expand_offsets(message['offsets'], fw['mod_indices'], n_tones)
                n_dwell = int(message.get('n_dwell', fw['n_dwell']))
                n_settle = int(message.get('n_settle', fw.get('n_settle', 0)))
                mode = message.get('mode', fw['mode'])
                if not 0 <= n_settle < n_dwell:
                    raise ValueError(
                        f'n_settle must satisfy 0 <= n_settle < n_dwell ({n_dwell}), got {n_settle}')
                self._fw_mod_revision = (self._fw_mod_revision + 1) & 0x7FFF
                # Recenter snaps fresh bins (the mock recomputes armed bins from the
                # centre), so it accepts moves the seamless update would reject.
                state = self._mock_fw_state(center, full, fw['mod_indices'], n_dwell, n_settle, mode)
                self._fw_mod = {'center': center, 'offsets': full, 'mod_indices': fw['mod_indices'],
                                'n_dwell': int(n_dwell), 'n_settle': int(n_settle),
                                'mode': str(mode), 'n_slots': int(full.shape[0]),
                                'revision': int(self._fw_mod_revision), 'state': state}
                state['enabled'] = bool(self._fw_mod_enabled)
                return {'status': 'success', 'warnings': list(state.get('warnings', [])),
                        'result': {'revision': self._fw_mod_revision, 'mode': str(mode),
                                   'n_slots': int(full.shape[0]), 'n_dwell': int(n_dwell),
                                   'n_settle': int(n_settle),
                                   'needs_recenter': state['needs_recenter'],
                                   'any_beyond_half_bin': state['any_beyond_half_bin'],
                                   'tones_beyond_half_bin': state['tones_beyond_half_bin']}}
            except Exception as e:
                return {'status': 'error', 'message': str(e)}

        if request == 'disable_fw_modulation':
            self._fw_mod_enabled = False
            if self._fw_mod is not None:
                self._fw_mod['state']['enabled'] = False
            return {'status': 'success'}

        if request == 'purge_modulation_revisions':
            if self._mod_enabled:
                return {'status': 'error',
                        'message': 'modulation is enabled; call disable_modulation() '
                                   'before purging revisions (an armed config can carry '
                                   'a pending snapshot that would resurrect the history)'}
            purged = len(self._mod_revision_history)
            self._mod_revision = 0
            self._mod_revision_history = {}
            if self._mod is not None:
                self._mod['state']['revision_history'] = {}
            return {'status': 'success', 'result': {'purged': purged, 'revision': 0}}

        if request == 'enable_triggered_stream':
            self.triggered_stream_enabled = True
            return {'status': 'success'}

        if request == 'disable_triggered_stream':
            self.triggered_stream_enabled = False
            return {'status': 'success'}

        if request == 'check_input_saturation':
            return {'status': 'success', 'result': False,
                    'details': {'threshold': 0.95, 'mock': True}}

        if request == 'check_output_saturation':
            return {'status': 'success', 'result': False,
                    'details': {'threshold': 0.95, 'mock': True}}

        if request == 'check_dsp_overflow':
            return {'status': 'success', 'result': False,
                    'details': {'psb_ovf_delta': 0, 'pfb_ovf_delta': 0,
                                'mock': True}}

        if request in ('maximise_tx_power', 'maximise_rx_power',
                       'maximise_rx_dsp_gain',
                       'optimise_tx_snr', 'optimise_rx_snr',
                       'fix_dac_saturation', 'fix_adc_saturation',
                       'fix_dsp_overflow'):
            return {'status': 'success', 'result': {'mock': True, 'warnings': []}}

        if request == 'sweep':
            self.latest_sweep_data = self.sweep_data(
                message.get('centers'), message.get('spans'),
                message.get('points', 11), message.get('samples_per_point', 10))
            self.sweep_progress = 1.0
            return {'status': 'success'}

        if request == 'retune':
            centers = np.atleast_1d(message.get('centers', self.tone_frequencies))
            self.tone_frequencies = centers.astype(float).tolist()
            self._resize_tone_state(len(self.tone_frequencies))
            self.latest_sweep_data = self.sweep_data(
                centers, message.get('spans'), message.get('points', 11),
                message.get('samples_per_point', 10))
            self.sweep_progress = 1.0
            return {'status': 'success'}

        if request == 'get_sweep_progress':
            return {'status': 'success', 'progress': self.sweep_progress}

        if request == 'get_sweep_data':
            if self.latest_sweep_data is None:
                self.latest_sweep_data = self.sweep_data()
            return {'status': 'success', 'data': self.latest_sweep_data}

        if request == 'get_sweep_raw_samples':
            samples = 1
            points = 1
            tones = len(self.tone_frequencies)
            data = np.zeros((samples, points, tones), dtype='float64')
            return {'status': 'success', 'data': {
                'samples': samples,
                'points': points,
                'tones': tones,
                'data_i': base64.b64encode(data.tobytes()).decode(),
                'data_q': base64.b64encode(data.tobytes()).decode(),
            }}

        if request == 'get_sweep_txt':
            return {'status': 'success', 'data': 'MOCK sweep data'}

        if request == 'get_blind_tones':
            return {'status': 'success', 'result': {
                'frequencies': list(self.tone_frequencies),
                'regular_indices': list(range(len(self.tone_frequencies))),
                'blind_indices': [],
            }}

        if request in ('set_blind_tones', 'remove_blind_tones'):
            return {'status': 'success', 'result': {
                'config_defaults': {},
                'mock': True,
            }}

        if request and (request.startswith('get_') or request.startswith('set_')
                        or request.startswith('soft_')):
            return {'status': 'success', 'result': {'mock': True}}

        return {'status': 'success', 'result': {'mock': True, 'request': request}}
