"""Mock readout-server state for ``ReadoutClient(..., mock=True)``.

This module models the server side rather than providing a
separate mock client class.  The public API remains ``ReadoutClient``; mock
mode swaps socket traffic for this in-process state object.
"""

import base64
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
import so3g
import spt3g.core

from souk_readout_tools.config_utils import get_template_config_path


class MockReadoutServer:
    """In-process stand-in for the RFSoC readout server."""

    DEFAULT_INFO_SECTIONS = [
        'server', 'versions', 'clock', 'timing', 'fpga', 'rfdc',
        'pipeline', 'tones', 'rf_frontend', 'lna', 'rfsoc_sensors',
    ]
    ALL_INFO_SECTIONS = DEFAULT_INFO_SECTIONS + [
        'diagnostics', 'config', 'calibrations', 'resonators', 'registers',
    ]

    @staticmethod
    def default_config(address='127.0.0.1', request_port=10000,
                       stream_port=20000):
        """Load the bundled template config and patch mock connection fields."""
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

        self.sample_rate = float(
            defaults.get('sample_rate_hz',
                         defaults.get('acc_freq',
                                      defaults.get('sample_rate', 500.0)))
        )
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
        """Return structured system information by section."""
        dispatchers = {
            'server': self._info_server,
            'versions': self._info_versions,
            'clock': self._info_clock,
            'timing': self._info_timing,
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
            'adc_clk_hz': 2457600000,
        }

    def _info_rfdc(self):
        defaults = self.config.get('firmware', {}).get('defaults', {})
        return {
            'ready': True,
            'dsa': defaults.get('dsa', 0),
            'vop_dac0': defaults.get('vop', 20000),
            'vop_dac1': defaults.get('vop', 20000),
            'dac_duc_mixer_frequency_hz': defaults.get(
                'dac_duc_mixer_frequency_hz', 0.0),
            'adc_ddc_mixer_frequency_hz': defaults.get(
                'adc_ddc_mixer_frequency_hz', 0.0),
            'nyquist_zone_adc': defaults.get('nyquist_zone', 1),
            'nyquist_zone_dac0': defaults.get('nyquist_zone', 1),
            'nyquist_zone_dac1': defaults.get('nyquist_zone', 1),
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
            'internal_loopback': defaults.get('internal_loopback', False),
            'psb_scale': defaults.get('psb_scale', 1),
            'psb_fftshift': defaults.get('psb_fftshift', 0),
            'pfb_fftshift': defaults.get('pfb_fftshift', 0),
            'acc_len': defaults.get('acc_len', 1000),
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
            'firmware_indices': details['rx']['tone_indices'],
            'blind_spans': blind_spans,
            **metadata,
            'detailed_frequency_info': details,
        }

    def _info_rf_frontend(self):
        rf_cfg = self.config.get('rf_frontend', {}) or {}
        mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}
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
            'tx_total_gain_db': 0.0,
            'rx_total_gain_db': 0.0,
            'tx_input_1db_comp_dbm': mixerless_cfg.get('tx_input_1db_comp_dbm'),
            'rx_input_1db_comp_dbm': mixerless_cfg.get('rx_input_1db_comp_dbm'),
        }
        if info['supports_bypass_amps']:
            info.update({
                'tx_amp_bypass': False,
                'rx_amp_bypass': False,
                'tx_bypass_amp_s21_db': mixerless_cfg.get('tx_amp_bypassed_s21_db'),
                'rx_bypass_amp_s21_db': mixerless_cfg.get('rx_amp_bypassed_s21_db'),
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
            'resonators_tracking': False,
            'max_detuning_hz': None,
        }

    def get_parameter(self, param_name, message=None):
        """Return a mock value for a parameter-server style ``get`` request."""
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
        """Update mock state for a parameter-server style ``set`` request."""
        if param_name == 'sample_rate_hz':
            self.sample_rate = float(param_value)
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

    def stream_frame(self, num_tones=None):
        """Build one modern stream packet: IQ words plus six flags, TT, cnt, err."""
        if num_tones is None:
            num_tones = len(self.tone_frequencies)
        tone_idx = np.arange(num_tones, dtype=float)
        phase = 0.07 * self.packet_counter + tone_idx
        i_words = np.round(1000.0 * np.sin(phase)).astype('<i4')
        q_words = np.round(1000.0 * np.cos(phase)).astype('<i4')
        iq_words = np.empty(2*num_tones, dtype='<i4')
        iq_words[0::2] = i_words
        iq_words[1::2] = q_words

        tt = int(time.time_ns())
        tail_u = np.zeros(10, dtype='<u4')
        tail_u[6] = (tt >> 32) & 0xFFFFFFFF
        tail_u[7] = tt & 0xFFFFFFFF
        tail_u[8] = self.packet_counter & 0xFFFFFFFF
        tail_u[9] = 0
        self.packet_counter += 1
        return np.concatenate((iq_words, tail_u.view('<i4'))).tobytes()

    def get_samples(self, num_samples, incl_system_info=True, burst=False):
        """Return mock raw samples in the same shape as ReadoutClient.get_samples."""
        self.client._warn_zero_phases()
        num_tones = len(self.tone_frequencies)
        data_raw = bytearray()
        t0 = time.time()
        frame_bytes = 0
        for _ in range(num_samples):
            frame = self.stream_frame(num_tones)
            frame_bytes = len(frame)
            data_raw.extend(frame)
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
        """Metadata sidecar shared by mock binary and G3 stream receivers."""
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
        """Mock implementation of continuous binary stream capture."""
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
            while True:
                try:
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
        """Mock implementation of ``receive_stream_g3`` using generated packets."""
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
        """Create base64-encoded sweep data compatible with parse_sweep_data."""
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
        """Emulate common readout-server requests for ``mock=True`` clients."""
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
