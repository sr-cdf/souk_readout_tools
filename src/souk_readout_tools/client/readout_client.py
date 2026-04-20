#!/usr/bin/env python3

"""
Client for the SOUK MKID readout server

This client can be used to interact with the SOUK MKID readout server on
the RFSoC ARM to initialize the firmware, get and set parameters,
enable and disable streaming, perform retuning, and receive streamed
samples.

The client can be used as a standalone script or imported as a module.

Example usage to begin triggered streaming:
    python readout_client.py config/config.yaml

Example general usage:
    $ ipython

    In [1]: import readout_client, numpy as np, matplotlib.pyplot as plt

    In [2]: client = readout_client.ReadoutClient()

    In [3]: client.get_server_status()
    Out[3]:
    {'process_name': 'readout_daemon',
     'ip_addresses': '10.11.11.11 192.168.2.224',
     'pwd': '/home/casper/readout_server',
    ....}

    In [4]: client.get_sample_rate()
    Out[4]: 500.0

    In [5]: num_tones = len(client.get_tone_frequencies())

    In [6]: raw_samples = client.get_samples(500)
    Received 500 samples in ~0.9946386814117432 seconds (~502.6951086301245 samples per second)

    In [7]: data = readout_client.ReadoutClient.parse_samples(raw_samples,num_tones)

    In [8]: print( np.all(np.diff(data['packet_counter']) == 1) )
    True

    In [9]: t = np.arange(len(data['packet_counter'])) / client.get_sample_rate()

    In [10]: z0 = data['i_data']['0000'] + 1j*data['q_data']['0000']

    In [11]: plt.plot(t, np.abs(z0))
    Out[11]: [<matplotlib.lines.Line2D at 0x7f5fda517580>]

    In [12]: plt.show()


Author: Sam Rowe
Date: July 2024
Version: 1.1.0

"""

import socket
import json
import struct
from urllib import response
import numpy as np
import yaml
import sys
import time
import os
import traceback
import csv
import base64
from scipy import signal

from souk_readout_tools.config_utils import get_template_config_path, copy_template_config


# Config keys whose string values are calibration file paths.
# Each entry is (section, key).  These fields can also hold scalars,
# inline [freq, dB] arrays, or None — only strings trigger file handling.
CAL_FILE_KEYS = [
    ('firmware', 'dac0_dbfs_to_dbm'),
    ('firmware', 'dac1_dbfs_to_dbm'),
    ('firmware', 'adc_dbm_to_dbfs'),
    ('rf_frontend', 'tx_combiner_loss_db'),
    ('rf_frontend', 'rx_combiner_loss_db'),
    ('rf_frontend', 'tx_if_s21_db'),
    ('rf_frontend', 'rx_if_s21_db'),
    ('rf_frontend', 'tx_rf_s21_db'),
    ('rf_frontend', 'rx_rf_s21_db'),
    ('rf_frontend', 'tx_mixer_conversion_loss_db'),
    ('rf_frontend', 'rx_mixer_conversion_loss_db'),
    ('rf_frontend', 'tx_bypass_amp_s21_db'),
    ('rf_frontend', 'rx_bypass_amp_s21_db'),
    ('rf_frontend', 'path_group_delay_ns'),
    ('cryostat', 'input_s21_db'),
    ('cryostat', 'output_s21_db'),
]


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKCYAN = '\033[96m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class ReadoutClient:
    def __init__(self, config_file=None, address=None, request_port=None, stream_port=None):
        """
        Initialize the ReadoutClient.

        There are two ways to create a client:

        1. With a config file (normal usage):
            client = ReadoutClient(config_file='my_config.yaml')

        2. With connection details (for pulling a config from a running server):
            client = ReadoutClient(address='10.11.11.11', request_port=10000)
            client.pull_config(save_as='my_config.yaml')

        Args:
            config_file: Path to a config YAML file.
            address: RFSoC IP address (used when no config_file is provided).
            request_port: TCP request port (required with address).
                          Pipeline 0 uses 10000, pipeline 1 uses 10001.
            stream_port: TCP stream port. If None, derived as request_port + 10000.
                          Pipeline 0 uses 20000, pipeline 1 uses 20001.
        """
        if config_file is not None:
            # Load config from file
            if not os.path.exists(config_file):
                raise FileNotFoundError(f"Config file not found: {config_file}")

            with open(config_file, 'r') as f:
                self.config = yaml.safe_load(f)

            self.config_file = os.path.abspath(config_file)
            self.config_dir = os.path.dirname(self.config_file)
            self.pipeline_id = self.config.get('firmware', {}).get('pipeline_id', 0)

            self.request_server_address = self.config['rfsoc_host']['address']
            self.request_server_port = self.config['rfsoc_host']['request_port']
            self.stream_server_address = self.config['rfsoc_host']['address']
            self.stream_server_port = self.config['rfsoc_host']['stream_port']

            print(f'Config loaded: {self.config_file} (pipeline {self.pipeline_id})')

        elif address is not None:
            if request_port is None:
                raise ValueError(
                    'request_port is required when connecting by address.\n'
                    'Pipeline 0 uses 10000, pipeline 1 uses 10001.\n'
                    'Example: ReadoutClient(address="10.11.11.11", request_port=10000)'
                )
            if stream_port is None:
                stream_port = request_port + 10000

            # Connect without a config file - user will pull_config from the server
            self.config = None
            self.config_file = None
            self.config_dir = os.getcwd()
            self.pipeline_id = None

            self.request_server_address = address
            self.request_server_port = request_port
            self.stream_server_address = address
            self.stream_server_port = stream_port

            print(f'Connecting to {address}:{request_port} (stream port {stream_port}, no local config)')
            print(f'Use client.pull_config(save_as="my_config.yaml") to fetch and save the running config.')

        else:
            # No config file, no address - help the user get started
            print(f'{bcolors.FAIL}No config file or server address provided.{bcolors.ENDC}')
            print()
            print('To connect to a running server and pull its config:')
            print(f'  client = ReadoutClient(address="10.11.11.11", request_port=10000)')
            print(f'  client.pull_config(save_as="my_config.yaml")')
            print()
            print('To create a config file from the template:')
            print(f'  from souk_readout_tools.config_utils import copy_template_config')
            print(f'  copy_template_config("my_config.yaml", pipeline_id=0)')
            print()
            raise ValueError('ReadoutClient requires either config_file or address.')

        self.system_information = None
        self.parameters = {}
        self.calibration_files = {}  # basename -> contents, populated by pull_config

    @property
    def cal_dir(self):
        """Directory for calibration files (``{config_dir}/calibrations/``)."""
        return os.path.join(self.config_dir, 'calibrations')

    @staticmethod
    def _resolve_export_path(filename, file_format, supported, default=None):
        """Resolve output filepath and format for export functions.

        Args:
            filename: User-provided filename.
            file_format: Explicit format string, or None to infer.
            supported: Tuple of recognised format strings (e.g. ('npy', 'json', 'csv')).
            default: Default format when neither file_format nor a recognised
                     extension is present.  None means raise an error instead.

        Returns:
            (filepath, file_format) tuple.
        """
        import warnings

        if file_format is not None:
            # Check for mismatch with filename extension
            parts = filename.rsplit('.', 1)
            if len(parts) == 2 and parts[1] in supported and parts[1] != file_format:
                warnings.warn(
                    f"Filename has extension '.{parts[1]}' but file_format='{file_format}' "
                    f"was specified. Writing to '{filename}.{file_format}'."
                )
            filepath = filename + '.' + file_format
        else:
            parts = filename.rsplit('.', 1)
            if len(parts) == 2 and parts[1] in supported:
                file_format = parts[1]
                filepath = filename
            elif default is not None:
                file_format = default
                filepath = filename + '.' + default
            else:
                raise ValueError(
                    f"No file_format provided and filename has no recognised "
                    f"extension ({', '.join('.' + s for s in supported)})."
                )
        return filepath, file_format

    def send_request(self, message):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.request_server_address, self.request_server_port))
            except socket.gaierror as e:
                print(f"Error connecting to request server {(self.request_server_address,self.request_server_port)}: {e}")
                return {'status': 'error', 'message': f"Error connecting to request server {(self.request_server_address,self.request_server_port)}: {e}"}
            except ConnectionRefusedError as e:
                print(f"Connection refused, is the server running? {(self.request_server_address,self.request_server_port)}: {e}")
                return {'status': 'error', 'message': f"Connection refused connecting to request server {(self.request_server_address,self.request_server_port)}: {e}"}
            except:
                print(f"Unhandled exception connecting to request server {(self.request_server_address,self.request_server_port)}: {sys.exc_info()[0]}")
                return {'status': 'error', 'message': f"Error connecting to request server {(self.request_server_address,self.request_server_port)}: {sys.exc_info()[0]}"}
            
            # Send message length + data
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)
            # print(f'sent: {message_data}')

            # Receive message length
            raw_msglen = s.recv(4)
            if not raw_msglen:
                return None
            datalen = struct.unpack('>I', raw_msglen)[0]

            # Pre-allocate bytearray to expected data length and receive the data
            response_data = bytearray(datalen)
            view = memoryview(response_data)
            received_len = 0
            while received_len < datalen:
                packet_len = s.recv_into(view[received_len:], datalen - received_len)
                if packet_len == 0:
                    break
                received_len += packet_len
            if received_len < datalen:
                print(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                return None

            # print(f'received: {response_data}')
            return json.loads(response_data.decode())

    def _initialise_server(self,config_file=None):
        message = {'request': 'initialise_server','config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()

        return response

    def _initialise_firmware(self,config_file=None):
        print(bcolors.WARNING + 'Warning: initialize_firmware will reset all pipelines, other clients will be affected.' + bcolors.ENDC)
        message = {'request': 'initialise_firmware', 'config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response
    
    def _initialise_pipeline(self,config_file=None):
        message = {'request': 'initialise_pipeline', 'config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response

    def ensure_ready(self, config_file=None, level="pipeline"):
        message = {'request': 'ensure_ready', 'config_filename': config_file, 'level': level}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response

    def pull_config(self, save_as=None, pull_calibration_files=True):
        """
        Pull the running config from the server and load it into this client.

        Calibration file paths in the config are detected and the file contents
        are fetched into memory (``self.calibration_files``).  Nothing is written
        to disk until ``save_config()`` is called (or ``save_as`` is provided
        as a convenience shortcut).

        Args:
            save_as: Path to save the config and calibration files to disk.
                     If None, everything stays in memory only.
            pull_calibration_files: If True (default), fetch referenced
                     calibration files from the server into memory.
        """
        message = {'request': 'pull_config'}
        response = self.send_request(message)
        if response['status'] == 'success':
            config_contents = response['config_contents']
            self.config_raw_text = config_contents
            self.config = yaml.safe_load(config_contents)
            self.pipeline_id = self.config.get('firmware', {}).get('pipeline_id', 0)
            print(f'Config pulled from server (pipeline {self.pipeline_id})')

            if pull_calibration_files:
                self._pull_config_cal_files()

            if save_as is not None:
                self.save_config(save_as)
        else:
            return response

    def _pull_config_cal_files(self):
        """
        For each calibration key in the config that holds a string (file path),
        pull the file contents from the server into self.calibration_files.

        Does not write to disk or modify config paths — that happens in
        save_config().
        """
        for section, key in CAL_FILE_KEYS:
            value = self.config.get(section, {}).get(key)
            if not isinstance(value, str):
                continue
            basename = os.path.basename(value)
            if basename in self.calibration_files:
                continue  # already fetched (e.g. shared between keys)
            msg = {'request': 'pull_calibration', 'cal_filename': basename}
            response = self.send_request(msg)
            if response.get('status') == 'success':
                self.calibration_files[basename] = response['cal_contents']
                print(f'  Pulled calibration file: {basename}')
            else:
                print(f'  WARNING: could not pull calibration file {basename} '
                      f'for {section}.{key}')

    def save_config(self, filename=None):
        """
        Save the in-memory config dict to a local YAML file.

        If calibration file contents are held in memory (from pull_config),
        they are written to a ``calibrations/`` directory next to the config
        file, and the config paths are rewritten to local relative paths.

        Args:
            filename: Path to write the config file. Defaults to self.config_file.
        """
        if self.config is None:
            raise RuntimeError('No config loaded. Use pull_config() first.')
        if filename is None:
            filename = self.config_file
        if filename is None:
            raise ValueError('No filename specified and no config_file set. Pass a filename.')
        filename = os.path.abspath(filename)
        self.config_file = filename
        self.config_dir = os.path.dirname(filename)

        # Write any in-memory calibration files and rewrite config paths
        config_text = getattr(self, 'config_raw_text', None)
        if self.calibration_files:
            os.makedirs(self.cal_dir, exist_ok=True)
            for basename, contents in self.calibration_files.items():
                dest = os.path.join(self.cal_dir, basename)
                with open(dest, 'w') as f:
                    f.write(contents)
            # Rewrite config paths to local relative form
            for section, key in CAL_FILE_KEYS:
                value = self.config.get(section, {}).get(key)
                if not isinstance(value, str):
                    continue
                basename = os.path.basename(value)
                if basename in self.calibration_files:
                    new_path = os.path.join('calibrations', basename)
                    if config_text is not None:
                        config_text = config_text.replace(value, new_path)
                    self.config[section][key] = new_path

        with open(filename, 'w') as f:
            if config_text is not None:
                f.write(config_text)
            else:
                yaml.dump(self.config, f, default_flow_style=False, sort_keys=False)
        print(f'Config saved to {filename}')

    def push_config(self, push_calibration_files=True):
        """
        Push the current config to the server.

        If push_calibration_files is True (default), any calibration parameters
        in the config that point to local files are automatically pushed to the
        server's calibrations directory and the paths are rewritten to the
        server-relative form ``pipeline_N/calibrations/filename``.

        The local config is not modified — only the copy sent to the server has
        rewritten paths.
        """
        name = os.path.basename(self.config_file) if self.config_file else 'config.yaml'

        # Deep-copy config so local version is not modified
        import copy
        push_config = copy.deepcopy(self.config)
        pipeline_id = push_config.get('firmware', {}).get('pipeline_id', 0)

        if push_calibration_files:
            for section, key in CAL_FILE_KEYS:
                value = push_config.get(section, {}).get(key)
                if not isinstance(value, str):
                    continue
                # Resolve the local file path
                local_path = self._resolve_local_cal_path(value)
                if local_path is None:
                    print(f'  WARNING: calibration file not found for {section}.{key}: {value}')
                    continue
                # Push the file to the server
                self.push_calibration(local_path)
                # Rewrite the config path to server-relative form
                basename = os.path.basename(local_path)
                server_path = f'pipeline_{pipeline_id}/calibrations/{basename}'
                push_config[section][key] = server_path
                print(f'  {section}.{key}: pushed {basename}, path -> {server_path}')

        config_contents = yaml.dump(push_config, sort_keys=False)
        message = {'request': 'push_config', 'config_filename': name, 'config_contents': config_contents}
        response = self.send_request(message)
        if response['status'] == 'success':
            source = self.config_file or 'memory'
            print(f'Config pushed from {source} to RFSoC')
            return
        else:
            return response

    def _resolve_local_cal_path(self, path_str):
        """
        Resolve a calibration file path string from the config to a local file.

        Tries in order:
          1. As-is (absolute path or cwd-relative)
          2. Relative to the config file directory
          3. Basename only, in a 'calibrations/' subdir next to the config

        Returns the resolved absolute path, or None if not found.
        """
        # 1. As-is
        if os.path.isfile(path_str):
            return os.path.abspath(path_str)
        # 2. Relative to config dir
        rel_to_config = os.path.join(self.config_dir, path_str)
        if os.path.isfile(rel_to_config):
            return os.path.abspath(rel_to_config)
        # 3. Basename in cal_dir
        cal_path = os.path.join(self.cal_dir, os.path.basename(path_str))
        if os.path.isfile(cal_path):
            return os.path.abspath(cal_path)
        return None


    def push_calibration(self, calibration_file):
        with open(calibration_file,'r') as file:
            cal = file.read()
        cal_filename=os.path.basename(calibration_file)
        message = {'request':'push_calibration',
                   'cal_filename':cal_filename,
                   'cal_contents':cal}
        response = self.send_request(message)
        if response['status'] == 'success':
            print(f'Pushed {cal_filename} to RFSoC')
        else:
            return response

    def pull_calibration(self, remote_file, destination_file=None):
        if destination_file is None:
            destination_file = os.path.join(self.config_dir, os.path.basename(remote_file))
        message = {'request':'pull_calibration',
                   'cal_filename':remote_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            cal = response['cal_contents']
            with open(destination_file,'w') as file:
                file.write(cal)
            print(f'Written calibration {os.path.basename(remote_file)} from RFSoC to {destination_file}')
        else:
            return response

    def hard_reset(self):
        print(bcolors.WARNING + 'Warning: hard_reset will reset all pipelines, other clients will be affected.' + bcolors.ENDC)
        message = {'request':'hard_reset'}
        return self.send_request(message)

    def cancel_all_tasks(self):
        message = {'request': 'cancel'}
        return self.send_request(message)

    def get_server_status(self):
        message = {'request': 'server_status'}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response
        else:
            print(f"Error getting server status: {response['message']}")
            return response

    def get_system_information(self):
        """Legacy flat-dict system information. Prefer get_info() instead."""
        message = {'request': 'get_system_information'}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.system_information = response['data']
            return response['data']
        else:
            print(f"Error getting system information: {response['message']}")
            return response

    def get_info(self, sections=None):
        """Get structured system information by section.

        Parameters
        ----------
        sections : list of str or ``'all'``, optional
            Which sections to include.  ``None`` returns the default set
            (server, versions, clock, fpga, rfdc, pipeline, tones,
            rf_frontend, lna).  ``'all'`` includes diagnostics, config,
            calibrations, resonators, and registers as well.

        Returns
        -------
        dict
            ``{section_name: section_dict, ...}`` where each section dict
            contains a ``'ready'`` bool and section-specific keys.
        """
        message = {'request': 'get_info'}
        if sections is not None:
            message['sections'] = sections
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['data']
        else:
            print(f"Error getting info: {response.get('message', 'unknown error')}")
            return response

    def health_check(self):
        """Quick system health summary for intermittent polling.

        Returns
        -------
        dict
            Compact health indicators including initialisation_level,
            clock_locked, streaming/sweeping state, saturation/overflow
            bools, tone_count, client_count, and resonator tracking status.
        """
        message = {'request': 'health_check'}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['data']
        else:
            print(f"Error getting health check: {response.get('message', 'unknown error')}")
            return response

    def sync_config_from_system(self):
        """
        Update the in-memory config with live hardware state from the server.

        Fetches system information and writes the current firmware settings
        back into config['firmware']['defaults'], and current tone
        frequencies/amplitudes/phases into the defaults section.  This
        captures the running state so that save_config() or push_config()
        will persist it.

        Does not write to disk — call save_config() afterwards to save.

        Returns the system_information dict.
        """
        if self.config is None:
            raise RuntimeError('No config loaded. Use pull_config() or load a config file first.')

        info = self.get_system_information()
        if not isinstance(info, dict) or 'pipeline_id' not in info:
            raise RuntimeError(f'Failed to get system information: {info}')

        defaults = self.config.setdefault('firmware', {}).setdefault('defaults', {})

        # Direct mappings: system_info key -> defaults key
        DIRECT_MAPS = {
            'sync_delay':                'sync_delay',
            'acc_len':                   'acc_len',
            'internal_loopback':         'internal_loopback',
            'psb_scale':                 'psb_scale',
            'psb_fftshift':              'psb_fftshift',
            'pfb_fftshift':              'pfb_fftshift',
            'dsa':                       'dsa',
            'dac_duc_mixer_frequency_hz': 'dac_duc_mixer_frequency_hz',
            'adc_ddc_mix_frequency_hz':  'adc_ddc_mixer_frequency_hz',
        }
        for info_key, defaults_key in DIRECT_MAPS.items():
            if info_key in info and info[info_key] is not None:
                defaults[defaults_key] = info[info_key]

        # VOP — use dac0 value
        if info.get('vop_dac0') is not None and info['vop_dac0'] != 0:
            defaults['vop'] = int(info['vop_dac0'])

        # Mixer scale modes
        if info.get('mixer_scale_1p0_dac0') is not None:
            defaults['dac_mixer_scale_1p0'] = bool(info['mixer_scale_1p0_dac0'])
        if info.get('mixer_scale_1p0_adc') is not None:
            defaults['adc_mixer_scale_1p0'] = bool(info['mixer_scale_1p0_adc'])

        # Nyquist zone — use DAC0 value
        if info.get('nyquist_zone_dac0') is not None and info['nyquist_zone_dac0'] != 1:
            defaults['nyquist_zone'] = int(info['nyquist_zone_dac0'])

        # QMC settings from DAC0
        dac_qmc = info.get('mixer_qmc_settings_dac0')
        if dac_qmc is not None:
            if 'GainCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_gain'] = dac_qmc['GainCorrectionFactor']
            if 'OffsetCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_offset'] = dac_qmc['OffsetCorrectionFactor']
            if 'PhaseCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_phase'] = dac_qmc['PhaseCorrectionFactor']

        # QMC settings from ADC
        adc_qmc = info.get('mixer_qmc_settings_adc')
        if adc_qmc is not None:
            if 'GainCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_gain'] = adc_qmc['GainCorrectionFactor']
            if 'OffsetCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_offset'] = adc_qmc['OffsetCorrectionFactor']
            if 'PhaseCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_phase'] = adc_qmc['PhaseCorrectionFactor']

        # Tone state
        if 'tone_frequencies' in info:
            defaults['frequencies'] = info['tone_frequencies']
        if 'tone_amplitudes' in info:
            defaults['amplitudes'] = info['tone_amplitudes']
        if 'tone_phases' in info:
            defaults['phases'] = info['tone_phases']

        # RF frontend peripheral state (attenuator, amp bypass)
        rf_response = self.get_rf_peripheral_status()
        rf_status = rf_response.get('result', {}) if isinstance(rf_response, dict) else {}
        rf = self.config.setdefault('rf_frontend', {})
        bypass = rf.setdefault('bypass_amps', {})
        if isinstance(rf_status, dict) and rf_status.get('enabled'):
            rf['tx_attenuator_value_db'] = rf_status['tx_attenuation_db']
            rf['rx_attenuator_value_db'] = rf_status['rx_attenuation_db']
            bypass['tx_amp_bypass'] = rf_status['tx_amp_bypass']
            bypass['rx_amp_bypass'] = rf_status['rx_amp_bypass']
        else:
            rf['tx_attenuator_value_db'] = None
            rf['rx_attenuator_value_db'] = None
            bypass['tx_amp_bypass'] = None
            bypass['rx_amp_bypass'] = None

        n_changed = sum(1 for k in DIRECT_MAPS.values() if k in defaults)
        print(f'Config defaults updated from live system state '
              f'({n_changed} parameters, {len(defaults.get("frequencies", []))} tones)')
        print('Use save_config() to write to disk, or push_config() to persist on the server.')

        return info

    def set_parameter(self, param_name, param_value):
        message = {'request': 'set', 'param': param_name, 'value': param_value}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response
        else:
            print(f"Error setting parameter {param_name}: {response['message']}")
            return response

    def get_parameter(self, param_name, **kwargs):
        message = {'request': 'get', 'param': param_name}
        message.update(kwargs)
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['value']
        else:
            print(f"Error getting parameter {param_name}: {response['message']}")
            return response

    def set_sample_rate(self, sample_rate_hz):
        return self.set_parameter('sample_rate_hz',sample_rate_hz)

    def get_sample_rate(self):
        return self.get_parameter('sample_rate_hz')

    def get_telescope_time(self):
        return self.get_parameter('telescope_time')

    def set_tone_frequencies(self, tone_frequencies):
        tone_frequencies = np.atleast_1d(tone_frequencies).tolist()
        return self.set_parameter('tone_frequencies',tone_frequencies)

    def get_tone_frequencies(self,detailed_output=False):
        if detailed_output:
            return self.get_parameter('tone_frequencies_detailed')
        else:
            return np.atleast_1d(self.get_parameter('tone_frequencies'))

    def set_tone_amplitudes(self, tone_amplitudes):
        tone_amplitudes = np.atleast_1d(tone_amplitudes).tolist()
        return self.set_parameter('tone_amplitudes',tone_amplitudes)

    def get_tone_amplitudes(self):
        return np.atleast_1d(self.get_parameter('tone_amplitudes'))

    def set_tone_phases(self, tone_phases):
        tone_phases = np.atleast_1d(tone_phases).tolist()
        return self.set_parameter('tone_phases',tone_phases)

    def get_tone_phases(self):
        return np.atleast_1d(self.get_parameter('tone_phases'))

    def _warn_zero_phases(self):
        """Check if all tone phases are zero and warn about crest factor.

        When multiple tones all have phase = 0, the first sample of every
        cosine waveform lines up perfectly, giving worst-case crest factor
        and risking DAC saturation / clipping.
        """
        import warnings
        try:
            phases = self.get_tone_phases()
        except Exception:
            return  # can't check — don't block the operation
        if len(phases) > 1 and np.all(phases == 0):
            warnings.warn(
                "\n*** ALL TONE PHASES ARE ZERO ***\n"
                "This produces the worst-case crest factor because every tone's "
                "cosine waveform peaks at the same instant, causing maximum coherent "
                "addition and risking DAC saturation / clipping.\n"
                "Consider calling set_tone_phases() with generate_newman_phases(freqs) "
                "or generate_random_phases(freqs) before proceeding.",
                stacklevel=3
            )

    # -- Pipeline DSP parameters --

    def set_sync_delay(self, value):
        """Set the sync delay (integer)."""
        return self.set_parameter('sync_delay', int(value))

    def get_sync_delay(self):
        """Get the current sync delay."""
        return self.get_parameter('sync_delay')

    def set_acc_len(self, value):
        """Set the accumulation length (integer)."""
        return self.set_parameter('acc_len', int(value))

    def get_acc_len(self):
        """Get the current accumulation length."""
        return self.get_parameter('acc_len')

    def set_internal_loopback(self, enabled):
        """Enable or disable the internal loopback."""
        return self.set_parameter('internal_loopback', bool(enabled))

    def get_internal_loopback(self):
        """Get the internal loopback state."""
        return self.get_parameter('internal_loopback')

    def set_psb_scale(self, value):
        """Set the PSB scale factor (integer)."""
        return self.set_parameter('psb_scale', int(value))

    def get_psb_scale(self):
        """Get the current PSB scale factor."""
        return self.get_parameter('psb_scale')

    def set_psb_fftshift(self, value):
        """Set the PSB FFT shift pattern (integer bitmask)."""
        return self.set_parameter('psb_fftshift', int(value))

    def get_psb_fftshift(self):
        """Get the current PSB FFT shift pattern."""
        return self.get_parameter('psb_fftshift')

    def set_pfb_fftshift(self, value):
        """Set the PFB FFT shift pattern (integer bitmask)."""
        return self.set_parameter('pfb_fftshift', int(value))

    def get_pfb_fftshift(self):
        """Get the current PFB FFT shift pattern."""
        return self.get_parameter('pfb_fftshift')

    def set_tone_powers(self, tone_powers_dbm, reference_plane='detector',
                        optimise_dynamic_range=True, rx_policy='protect'):
        """Set tone powers to specified levels in dBm.

        Parameters
        ----------
        tone_powers_dbm : array-like
            Target power per tone in dBm.
        reference_plane : str
            Where the target power is specified: 'dac', 'rf_output', or
            'detector' (default).
        optimise_dynamic_range : bool
            If True (default), maximise DAC bit utilisation and adjust
            the analog chain (attenuator, amp bypass, DSA) to hit the
            target power.  Set to False to skip optimisation for speed.
        rx_policy : str
            How to manage the RX path when the TX power change risks
            saturating the ADC.  One of:

            - ``'protect'`` (default) — if ADC saturates, increase RX
              attenuation or DSA to clear it and warn.
            - ``'compensate'`` — mirror TX power changes onto the RX
              path to keep round-trip power constant.
            - ``'raise'`` — raise error if ADC saturates.
            - ``'none'`` — don't touch the RX path.
        """
        tone_powers_dbm = np.atleast_1d(tone_powers_dbm).tolist()
        message = {'request': 'set', 'param': 'tone_powers',
                   'value': tone_powers_dbm,
                   'reference_plane': reference_plane,
                   'optimise_dynamic_range': optimise_dynamic_range,
                   'rx_policy': rx_policy}
        response = self.send_request(message)
        if response.get('status') != 'success':
            print(f"Error setting tone powers: {response.get('message')}")
            return response
        if response.get('result'):
            r = response['result']
            for w in r.get('warnings', []):
                print(f'  WARNING: {w}')
            if r.get('effective_bits_per_tone'):
                bits = np.array(r['effective_bits_per_tone'])
                valid = bits[np.isfinite(bits)]
                if len(valid) > 0:
                    if np.max(valid) > 16:
                        print(f'  WARNING: DAC overdriven ({np.max(valid):.1f} effective bits, '
                              f'max is 16)')
                    else:
                        print(f'  Effective DAC bits per tone: '
                              f'{np.min(valid):.1f} — {np.max(valid):.1f} (of 16)')
        return response

    def get_tone_powers(self,detailed_output=False,reference_plane='detector'):
        """Get current tone powers at the specified reference plane.

        Parameters
        ----------
        detailed_output : bool
            If True, return a dict with power at every stage in the signal
            chain (both TX and RX).
        reference_plane : str
            TX chain (DAC -> detector):
                'dac'              - DAC output (after VOP, before analog frontend)
                'rf_output'        - RF frontend output (after amp, before cryostat)
                'detector'         - cryogenic focal plane (default)
            RX chain (detector -> accumulator):
                'cryostat_output'  - cryostat output (before RX frontend)
                'adc_input'        - ADC input (after RX frontend)
                'accumulator'      - raw accumulated IQ magnitude in dB

        Returns
        -------
        numpy.ndarray or tuple
            Tone powers in dBm (or dB for 'accumulator'). If detailed_output
            is True, returns (powers, details) where details is a dict of
            per-stage values across the full TX and RX chain.
        """
        if detailed_output:
            return self.get_parameter('tone_powers_detailed', reference_plane=reference_plane)
        else:
            return np.atleast_1d(self.get_parameter('tone_powers', reference_plane=reference_plane))

    def check_input_saturation(self,iterations=250):
        message = {'request': 'check_input_saturation','iterations':iterations}
        return self.send_request(message)

    def check_output_saturation(self,iterations=250):
        message = {'request': 'check_output_saturation','iterations':iterations}
        return self.send_request(message)

    def check_dsp_overflow(self,duration_s=0.2):
        message = {'request': 'check_dsp_overflow','duration_s':duration_s}
        return self.send_request(message)
    
    # TODO: Add option to save the resulting parameters to the config file after
    #       maximise/optimise/fix operations (requires save_config, see push_config TODO).
    def maximise_tx_power(self, headroom_db=2.0, reference_plane='dac',
                          power_limit_dbm=None, rx_policy='protect'):
        """Maximise TX output power at the chosen reference plane.

        Parameters
        ----------
        headroom_db : float
            Safety margin below DAC saturation (default 2.0 dB).
        reference_plane : str
            'dac' (default), 'rf_output', or 'detector'.
        power_limit_dbm : float or None
            Maximum allowed tone power in dBm at the reference plane.
        rx_policy : str
            How to manage the RX path when TX power increases risk
            saturating the ADC.  One of:

            - ``'protect'`` (default) — if ADC saturates, increase RX
              attenuation or DSA to clear it and warn.
            - ``'compensate'`` — mirror TX power changes onto the RX
              path to keep round-trip power constant.
            - ``'raise'`` — raise error if ADC saturates.
            - ``'none'`` — don't touch the RX path.
        """
        msg = {'request': 'maximise_tx_power', 'headroom_db': headroom_db,
               'reference_plane': reference_plane, 'rx_policy': rx_policy}
        if power_limit_dbm is not None:
            msg['power_limit_dbm'] = power_limit_dbm
        return self.send_request(msg)

    def maximise_rx_power(self, headroom_db=1.0):
        return self.send_request({'request': 'maximise_rx_power', 'headroom_db': headroom_db})

    def optimise_tx_snr(self, reference_plane='detector', headroom_db=2.0):
        return self.send_request({'request': 'optimise_tx_snr',
                                  'reference_plane': reference_plane,
                                  'headroom_db': headroom_db})

    def optimise_rx_snr(self):
        return self.send_request({'request': 'optimise_rx_snr'})

    def fix_dac_saturation(self):
        return self.send_request({'request': 'fix_dac_saturation'})
    
    def fix_adc_saturation(self):
        return self.send_request({'request': 'fix_adc_saturation'})

    def fix_dsp_overflow(self, duration_s=0.5, max_iterations=10):
        return self.send_request({'request': 'fix_dsp_overflow',
                                  'duration_s': duration_s,
                                  'max_iterations': max_iterations})

    # -- RF peripheral (attenuator / amp bypass) control --

    def get_rf_peripheral_status(self):
        return self.send_request({'request': 'get_rf_peripheral_status'})

    def set_tx_attenuation(self, value_db):
        return self.send_request({'request': 'set_tx_attenuation', 'value': float(value_db)})

    def get_tx_attenuation(self):
        return self.send_request({'request': 'get_tx_attenuation'})

    def set_rx_attenuation(self, value_db):
        return self.send_request({'request': 'set_rx_attenuation', 'value': float(value_db)})

    def get_rx_attenuation(self):
        return self.send_request({'request': 'get_rx_attenuation'})

    def set_tx_amp_bypass(self, bypass=True):
        return self.send_request({'request': 'set_tx_amp_bypass', 'bypass': bool(bypass)})

    def get_tx_amp_bypass(self):
        return self.send_request({'request': 'get_tx_amp_bypass'})

    def set_rx_amp_bypass(self, bypass=True):
        return self.send_request({'request': 'set_rx_amp_bypass', 'bypass': bool(bypass)})

    def get_rx_amp_bypass(self):
        return self.send_request({'request': 'get_rx_amp_bypass'})

    # --- LNA bias control ---

    def get_lna_controller_status(self):
        """Get LNA bias controller status (enabled, hardware, lna_channel)."""
        return self.send_request({'request': 'get_lna_controller_status'})

    def get_lna_bias_status(self, channel=None):
        """Read LNA bias voltage and current for a single channel.

        Args:
            channel: LNA channel index (1-14). Defaults to this pipeline's configured channel.

        Returns:
            dict with remote_voltage_v, local_voltage_v, bias_current_a.
        """
        msg = {'request': 'get_lna_bias_status'}
        if channel is not None:
            msg['channel'] = int(channel)
        return self.send_request(msg)

    def get_lna_bias_status_all(self):
        """Read LNA bias voltage and current for all 14 channels."""
        return self.send_request({'request': 'get_lna_bias_status_all'})

    def set_lna_bias_voltage(self, voltage_v, channel=None,
                             method='remote', blind=False):
        """Set LNA bias voltage.

        Args:
            voltage_v: Target voltage in volts.
            channel: LNA channel index (1-14). Defaults to this pipeline's configured channel.
            method: 'remote' (iterative feedback, default) or 'local' (direct DAC).
            blind: If True and method='remote', skip LNA voltage validation.

        Returns:
            dict with achieved voltage and any error message.
        """
        msg = {
            'request': 'set_lna_bias_voltage',
            'voltage_v': float(voltage_v),
            'method': method,
            'blind': blind,
        }
        if channel is not None:
            msg['channel'] = int(channel)
        return self.send_request(msg)

    def set_lna_bias_voltage_all(self, voltage_v, method='remote', blind=False):
        """Set LNA bias voltage for all 14 channels.

        Args:
            voltage_v: Target voltage in volts.
            method: 'remote' (default) or 'local'.
            blind: If True and method='remote', skip LNA voltage validation.

        Returns:
            dict of per-channel results keyed by channel index.
        """
        return self.send_request({
            'request': 'set_lna_bias_voltage_all',
            'voltage_v': float(voltage_v),
            'method': method,
            'blind': blind,
        })

    def enable_stream(self):
        self._warn_zero_phases()
        message = {'request': 'enable_stream'}
        return self.send_request(message)

    def disable_stream(self):
        message = {'request': 'disable_stream'}
        return self.send_request(message)

    def enable_triggered_stream(self):
        self._warn_zero_phases()
        message = {'request': 'enable_triggered_stream'}
        return self.send_request(message)

    def disable_triggered_stream(self):
        message = {'request': 'disable_triggered_stream'}
        return self.send_request(message)

    def send_fake_trigger(self):
        message = {'request': 'send_fake_trigger'}
        return self.send_request(message)

    def get_cal_freeze(self):
        return self.get_parameter('cal_freeze')

    def set_cal_freeze(self,freeze):
        return self.set_parameter('cal_freeze',freeze)

    def get_clock_source(self):
        """Get the current reference clock source ('internal' or 'external')."""
        return self.get_parameter('clock_source')

    def set_clock_source(self, source):
        """Set the reference clock source.

        Parameters
        ----------
        source : str
            'internal' for the on-board 12.8 MHz oscillator, or
            'external' for a 10 MHz reference on clk0.

        Returns
        -------
        dict
            Server response including clock lock status after applying.
        """
        return self.set_parameter('clock_source', source)

    def get_clock_status(self):
        """Get PLL lock status of all clock chips.

        Returns
        -------
        dict
            Keys: 'all_locked' (bool), 'chips' (list of dicts with
            'name' and 'status' per chip).
        """
        return self.get_parameter('clock_status')

    def get_samples(self, num_samples,incl_system_info=True,burst=False):
        """
        Acquire num_samples samples from the readout server and return concatenated raw data.

        The server sends only active tones in user order, so the frame
        size depends on the number of active tones. The per-frame byte
        count is stored in sample_data['frame_bytes'] for parse_samples.
        """
        self._warn_zero_phases()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.request_server_address, self.request_server_port))
            message = {'request': 'get_samples', 'num_samples': num_samples, 'burst': burst}
            # Send message length
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)
            # Pre-allocate bytearray (conservative upper bound)
            max_frame_bytes = 2048*2*4 + 10*4
            data_raw = bytearray(max_frame_bytes*num_samples)
            view = memoryview(data_raw)

            t0=time.time()
            frame_bytes = 0
            write_offset = 0
            for j in range(num_samples):
                # Read data length
                raw_datalen = s.recv(4)
                if not raw_datalen:
                    break
                frame_bytes = struct.unpack('>I', raw_datalen)[0]
                received_len = 0
                while received_len < frame_bytes:
                    packet_len = s.recv_into(view[write_offset+received_len:], frame_bytes - received_len)
                    if packet_len == 0:
                        break
                    received_len += packet_len
                if received_len < frame_bytes:
                    print(f"Expected {frame_bytes} bytes, but only received {received_len} bytes.")
                    break
                write_offset += frame_bytes
            t1=time.time()
            print(f"Received {num_samples} samples in ~{t1-t0} seconds (~{num_samples/(t1-t0)} samples per second)")
            if incl_system_info:
                info = self.get_system_information()
            else:
                info = {'system_information':'No system information requested'}
            sample_rate = self.get_sample_rate()
            # Trim buffer to actual data received
            data_raw = data_raw[:write_offset]
            sample_data = {'data_raw':data_raw,'sample_rate':sample_rate,
                           'system_information':info,'frame_bytes':frame_bytes}
            return sample_data


    @staticmethod
    def parse_samples(sample_data, num_tones=None):
        """
        Parse raw sample data into per-tone I/Q arrays.

        The server sends only active tones in user order, so the data
        is already correctly ordered. Frame size is determined from
        sample_data['frame_bytes'] (set by get_samples) or derived
        from tone_indices in system_information.

        Falls back to 2048 channels for data from older servers that
        send all channels.
        """
        data_raw = sample_data['data_raw']
        sample_rate = sample_data['sample_rate']
        info = sample_data['system_information']
        num_headers = 10

        # Determine frame size and num_tones
        if 'frame_bytes' in sample_data:
            frame_bytes = sample_data['frame_bytes']
            num_tones = (frame_bytes // 4 - num_headers) // 2
        elif num_tones is None:
            tone_indices = info.get('tone_indices')
            if tone_indices is not None:
                num_tones = len(tone_indices)
            else:
                num_tones = 2048

        datalen = num_tones * 2 * 4 + num_headers * 4
        num_samples = len(data_raw) // datalen
        i_data = np.zeros((num_samples, num_tones), dtype='<i4')
        q_data = np.zeros((num_samples, num_tones), dtype='<i4')
        cnt = np.zeros(num_samples, dtype=int)
        err = np.zeros(num_samples, dtype=int)
        tt = np.zeros(num_samples, dtype=np.uint64)
        flags = np.zeros((num_samples, 6), dtype=int)
        for j in range(num_samples):
            packet_offset = j * datalen
            all_data = np.frombuffer(data_raw[packet_offset:packet_offset+datalen], dtype='<i4')
            i_data[j] = all_data[::2][:num_tones]
            q_data[j] = all_data[1::2][:num_tones]
            err[j] = all_data[-1]
            cnt[j] = all_data[-2]
            tt_lsb = int(all_data[-3]) & 0xFFFFFFFF
            tt_msb = int(all_data[-4]) & 0xFFFFFFFF
            tt[j] = (tt_msb << 32) + tt_lsb
            flags[j] = all_data[-10:-4]
        data_dict = {'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
                    'num_tones':num_tones,
                    'num_samples':num_samples,
                    'sample_rate':sample_rate,
                    'system_information':info,
                    'i_data':{f'{i:04d}':i_data[:,i] for i in range(num_tones)},
                    'q_data':{f'{i:04d}':q_data[:,i] for i in range(num_tones)},
                    'packet_counter':cnt,
                    'packet_error':err,
                    'telescope_time':tt,
                    'stream_flags':{f'flag{i}':flags[:,i] for i in range(6)}
                    }

        return data_dict

    @staticmethod
    def export_samples(filename, sample_data, num_tones_to_save=None,file_format=None):
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json', 'csv'))

        if 'data_raw' in sample_data.keys():
            data_dict = ReadoutClient.parse_samples(sample_data,num_tones=num_tones_to_save)
        else:
            data_dict = sample_data

        if file_format == 'npy':
            np.save(filepath, data_dict)

        elif file_format == 'json':
            # Convert numpy arrays to lists for JSON serialization, including nested arrays
            json_data_dict = {}
            for key, value in data_dict.items():
                if isinstance(value, dict):
                    json_data_dict[key] = {
                        sub_key: sub_value.tolist() if isinstance(sub_value, np.ndarray) else sub_value
                        for sub_key, sub_value in value.items()
                    }
                elif isinstance(value, np.ndarray):
                    json_data_dict[key] = value.tolist()
                else:
                    json_data_dict[key] = value

            with open(filepath, 'w') as file:
                json.dump(json_data_dict, file, indent=4)

        elif file_format == 'csv':
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                # Write the metadata
                writer.writerow(['# date', data_dict['date']])
                writer.writerow(['# num_tones', data_dict['num_tones']])
                writer.writerow(['# num_samples', data_dict['num_samples']])
                writer.writerow(['# sample_rate', data_dict['sample_rate']])
                for key,value in data_dict['system_information'].items():
                    writer.writerow([f'# {key}', value])
                # Write the header for i_data, q_data, packet_counter, packet_error, and stream_flags
                header = []
                for i in range(data_dict['num_tones']):
                    header.extend([f'i_data_{i:04d}', f'q_data_{i:04d}'])
                header.extend(['packet_counter', 'packet_error', 'telescope_time'])
                num_flags = len(data_dict['stream_flags'])
                header.extend([f'flag{i}' for i in range(num_flags)])
                writer.writerow(header)
                # Write the data rows
                for j in range(data_dict['num_samples']):
                    row = []
                    for i in range(data_dict['num_tones']):
                        row.extend([data_dict['i_data'][f'{i:04d}'][j], data_dict['q_data'][f'{i:04d}'][j]])
                    row.append(data_dict['packet_counter'][j])
                    row.append(data_dict['packet_error'][j])
                    row.append(int(data_dict['telescope_time'][j]))
                    row.extend([data_dict['stream_flags'][f'flag{k}'][j] for k in range(num_flags)])
                    writer.writerow(row)

        elif file_format == 'dirfile':
            raise NotImplementedError("dirfile format not yet implemented.")

        elif file_format == 'hdf5':
            raise NotImplementedError("hdf5 format not yet implemented.")

        else:
            raise ValueError(f"Invalid file_format {file_format}. Must be one of 'npy', 'json', 'csv', 'dirfile', or 'hdf5'.")

    @staticmethod
    def import_samples(filename):
        data_dict={}
        if filename.endswith('.npy'):
            data_dict = np.load(filename,allow_pickle=True).item()

        elif filename.endswith('.json'):
            with open(filename,'r') as file:
                data_dict = json.load(file)
                for item in data_dict:
                    if isinstance(data_dict[item],list):
                        data_dict[item] = np.array(data_dict[item])
                    if isinstance(data_dict[item],dict):
                        for sub_item in data_dict[item]:
                            if isinstance(data_dict[item][sub_item],list):
                                data_dict[item][sub_item] = np.array(data_dict[item][sub_item])
                            if isinstance(data_dict[item][sub_item],dict):
                                for sub_sub_item in data_dict[item][sub_item]:
                                    if isinstance(data_dict[item][sub_item][sub_sub_item],list):
                                        data_dict[item][sub_item][sub_sub_item] = np.array(data_dict[item][sub_item][sub_sub_item])

        elif filename.endswith('.csv'):
            with open(filename, mode='r') as file:
                lines=file.readlines()
                header_lines=0
                for line in lines:
                    if line.startswith('#'):
                        header_lines+=1
                        key = line.split(',')[0].lstrip('# ')
                        value = line[line.find(',')+1:].strip()
                        if key=='date':
                            value = value
                        elif value.startswith('"') and value.endswith('"'):
                            value = eval(value[1:-1])
                        else:
                            try:
                                value = eval(value)
                            except NameError:
                                value = value
                        data_dict[key] = value

            data = np.genfromtxt(filename, delimiter=',',names=True,skip_header=header_lines)
            data_dict['i_data'] = {f'{i:04d}':data[f'i_data_{i:04d}'].astype(int) for i in range(data_dict['num_tones'])}
            data_dict['q_data'] = {f'{i:04d}':data[f'q_data_{i:04d}'].astype(int) for i in range(data_dict['num_tones'])}
            data_dict['packet_counter'] = data['packet_counter'].astype(int)
            data_dict['packet_error'] = data['packet_error'].astype(int)
            if 'telescope_time' in data.dtype.names:
                data_dict['telescope_time'] = data['telescope_time'].astype(np.uint64)
            else:
                data_dict['telescope_time'] = np.zeros(len(data['packet_counter']), dtype=np.uint64)
            num_flags = sum(1 for name in data.dtype.names if name.startswith('flag'))
            data_dict['stream_flags'] = {f'flag{i}':data[f'flag{i}'].astype(int) for i in range(num_flags)}

        elif filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        elif os.path.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        else:
            raise ValueError(f"Invalid file format {filename.split('.')[-1]}")
        return data_dict

    def get_accumulator_snapshots(self, tone_index, num_snapshots):
        """
        Acquire num_snapshots pre-accumulation snapshots for a single tone.

        Each snapshot contains 1024 complex samples at the FFT output rate
        (before accumulation).

        Args:
            tone_index (int): Tone index to snapshot.
            num_snapshots (int): Number of snapshots to acquire.

        Returns:
            dict with keys:
                'snapshots': Complex array of shape (num_snapshots, 1024).
                'tone_index': The tone index that was snapshotted.
                'sample_rate': The pre-accumulation sample rate in Hz
                               (FFT output rate = accumulated rate * acc_len).
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.request_server_address, self.request_server_port))
            message = {'request': 'get_accumulator_snapshots',
                       'tone_index': tone_index,
                       'num_snapshots': num_snapshots}
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)

            result = None
            t0 = time.time()
            for j in range(num_snapshots):
                # Read data length
                raw_datalen = s.recv(4)
                if not raw_datalen:
                    break
                datalen = struct.unpack('>I', raw_datalen)[0]

                # Receive the snapshot data
                data_buf = bytearray(datalen)
                view = memoryview(data_buf)
                received_len = 0
                while received_len < datalen:
                    packet_len = s.recv_into(view[received_len:], datalen - received_len)
                    if packet_len == 0:
                        break
                    received_len += packet_len
                if received_len < datalen:
                    print(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                    break

                snapshot = np.frombuffer(data_buf, dtype=np.complex128)
                if result is None:
                    result = np.zeros((num_snapshots, len(snapshot)), dtype=np.complex128)
                result[j] = snapshot

            t1 = time.time()
            print(f"Received {num_snapshots} snapshots in {t1-t0:.3f}s "
                  f"({num_snapshots/(t1-t0):.1f} snapshots/s)")

        info = self.get_system_information()
        acc_len = info['acc_len']
        accumulated_rate = self.get_sample_rate()
        snapshot_rate = accumulated_rate * acc_len

        return {'snapshots': result,
                'tone_index': tone_index,
                'sample_rate': snapshot_rate,
                'num_snapshots': num_snapshots,
                'len_snapshot': result.shape[1] if result is not None else 0}

    def batch_snapshots(self, tone_indices=None, num_snapshots=10,
                        export_file=None, plot=False, verbose=True):
        """
        Acquire pre-accumulator snapshots for multiple tones in a single
        server request.

        Uses batch_accumulator_snapshots on the server so the
        tone-to-firmware-channel lookup happens once, and all data streams
        over a single TCP connection.

        Note on indexing: tone_indices are user-facing ordinal indices
        (0, 1, 2, ...) corresponding to the order tones were set, not
        firmware LO channel indices (which may be non-contiguous due to
        VACC constraints). The translation to firmware channels happens
        server-side.

        Args:
            tone_indices: List of user-facing tone indices to snapshot,
                          or None for all active tones.
            num_snapshots (int): Number of 1024-sample snapshots per tone.
            export_file (str): Path to save results as .npz. None to skip.
            plot (bool): If True, plot time-domain and power spectrum for
                         each tone.
            verbose (bool): Print progress.

        Returns:
            dict with keys:
                'results': dict mapping tone_index -> snapshot dict (as
                           returned by get_accumulator_snapshots).
                'sample_rate': Pre-accumulator sample rate in Hz.
                'num_snapshots': Snapshots per tone.
                'tone_frequencies': Array of tone frequencies in Hz.
                'firmware_indices': Firmware LO channel indices for each
                                    tone (from detailed tone frequency query).
        """
        freqs = self.get_tone_frequencies()
        n_tones = len(freqs)

        # Get firmware-level tone index mapping for reference
        freq_details = self.get_tone_frequencies(detailed_output=True)
        fw_indices = freq_details.get('rx', {}).get('tone_indices', list(range(n_tones)))

        if tone_indices is None:
            tone_indices = list(range(n_tones))
        else:
            tone_indices = list(tone_indices)
            for idx in tone_indices:
                if idx < 0 or idx >= n_tones:
                    raise ValueError(
                        f"Tone index {idx} out of range (0 to {n_tones - 1})")

        info = self.get_system_information()
        acc_len = info['acc_len']
        accumulated_rate = self.get_sample_rate()
        snapshot_rate = accumulated_rate * acc_len

        # Single server request for all tones
        results = {}
        total_frames = len(tone_indices) * num_snapshots
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.request_server_address, self.request_server_port))
            message = {'request': 'batch_accumulator_snapshots',
                       'tone_indices': tone_indices,
                       'num_snapshots': num_snapshots}
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)

            t0 = time.time()
            for i, tidx in enumerate(tone_indices):
                if verbose:
                    fw_idx = fw_indices[tidx] if tidx < len(fw_indices) else '?'
                    print(f"Snapshotting tone {tidx} (fw chan {fw_idx}, "
                          f"{i+1}/{len(tone_indices)}, "
                          f"{freqs[tidx]/1e6:.3f} MHz)...")
                tone_data = None
                for j in range(num_snapshots):
                    raw_datalen = s.recv(4)
                    if not raw_datalen:
                        raise RuntimeError("Server closed connection unexpectedly")
                    datalen = struct.unpack('>I', raw_datalen)[0]
                    data_buf = bytearray(datalen)
                    view = memoryview(data_buf)
                    received_len = 0
                    while received_len < datalen:
                        packet_len = s.recv_into(view[received_len:], datalen - received_len)
                        if packet_len == 0:
                            break
                        received_len += packet_len
                    if received_len < datalen:
                        raise RuntimeError(f"Expected {datalen} bytes, got {received_len}")
                    snapshot = np.frombuffer(data_buf, dtype=np.complex128)
                    if tone_data is None:
                        tone_data = np.zeros((num_snapshots, len(snapshot)), dtype=np.complex128)
                    tone_data[j] = snapshot
                results[tidx] = {
                    'snapshots': tone_data,
                    'tone_index': tidx,
                    'sample_rate': snapshot_rate,
                }
            t1 = time.time()
            if verbose:
                print(f"Received {total_frames} snapshots for {len(tone_indices)} tones "
                      f"in {t1-t0:.3f}s ({total_frames/(t1-t0):.1f} snapshots/s)")

        output = {
            'results': results,
            'sample_rate': snapshot_rate,
            'num_snapshots': num_snapshots,
            'tone_frequencies': freqs,
            'firmware_indices': fw_indices,
        }

        if export_file is not None:
            self._export_batch_snapshots(output, export_file, verbose)

        if plot:
            self._plot_batch_snapshots(output)

        return output

    @staticmethod
    def _export_batch_snapshots(batch_data, filepath, verbose=True):
        """Save batch snapshot data to a .npz file."""
        if not filepath.endswith('.npz'):
            filepath += '.npz'

        save_dict = {
            'sample_rate': batch_data['sample_rate'],
            'num_snapshots': batch_data['num_snapshots'],
            'tone_frequencies': batch_data['tone_frequencies'],
            'tone_indices': np.array(list(batch_data['results'].keys())),
            'firmware_indices': np.array(batch_data['firmware_indices']),
        }
        for tidx, snap in batch_data['results'].items():
            save_dict[f'snapshots_tone_{tidx}'] = snap['snapshots']

        np.savez(filepath, **save_dict)
        if verbose:
            print(f"Batch snapshots saved to {filepath}")

    @staticmethod
    def _plot_batch_snapshots(batch_data):
        """Plot time-domain and power spectrum for each tone in a batch."""
        from souk_readout_tools.plotting import plot_batch_snapshots
        import matplotlib.pyplot as plt
        plot_batch_snapshots(batch_data, format='iq_vs_t',
                             repetitions='mean', psd=True)
        plt.show()

    # -- ADC / DAC snapshots --

    def get_adc_snapshot(self):
        """
        Capture a single ADC snapshot (4096 complex128 samples).

        Returns:
            dict with keys:
                'snapshot': complex128 array of shape (4096,).
                'system_information': System info at time of capture.
        """
        response = self.send_request({'request': 'get_adc_snapshot'})
        if response['status'] != 'success':
            raise RuntimeError(f"ADC snapshot failed: {response.get('message')}")
        result = response['result']
        snapshot = np.frombuffer(
            base64.b64decode(result['snapshot']), dtype=np.complex128,
        ).copy()
        info = self.get_system_information()
        return {
            'snapshot': snapshot,
            'system_information': info,
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
        }

    def get_dac_snapshot(self):
        """
        Capture a single DAC snapshot (4096 complex128 samples per DAC).

        Returns:
            dict with keys:
                'dac0': complex128 array of shape (4096,).
                'dac1': complex128 array of shape (4096,).
                'system_information': System info at time of capture.
        """
        response = self.send_request({'request': 'get_dac_snapshot'})
        if response['status'] != 'success':
            raise RuntimeError(f"DAC snapshot failed: {response.get('message')}")
        result = response['result']
        dac0 = np.frombuffer(
            base64.b64decode(result['dac0']), dtype=np.complex128,
        ).copy()
        dac1 = np.frombuffer(
            base64.b64decode(result['dac1']), dtype=np.complex128,
        ).copy()
        info = self.get_system_information()
        return {
            'dac0': dac0,
            'dac1': dac1,
            'system_information': info,
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
        }

    @staticmethod
    def parse_adc_snapshot(snapshot_data):
        """
        Parse raw ADC snapshot data into a results dictionary.

        Args:
            snapshot_data: dict from get_adc_snapshot().

        Returns:
            dict with 'adc_i', 'adc_q', 'adc_amplitude', 'adc_phase',
                 'length', and metadata.
        """
        snapshot = snapshot_data['snapshot']
        return {
            'date': snapshot_data.get('date', ''),
            'system_information': snapshot_data.get('system_information', {}),
            'length': len(snapshot),
            'adc_i': snapshot.real,
            'adc_q': snapshot.imag,
            'adc_amplitude': np.abs(snapshot),
            'adc_phase': np.angle(snapshot),
        }

    @staticmethod
    def parse_dac_snapshot(snapshot_data):
        """
        Parse raw DAC snapshot data into a results dictionary.

        Args:
            snapshot_data: dict from get_dac_snapshot().

        Returns:
            dict with 'dac0_i', 'dac0_q', 'dac1_i', 'dac1_q', amplitudes,
                 phases, 'length', and metadata.
        """
        dac0 = snapshot_data['dac0']
        dac1 = snapshot_data['dac1']
        return {
            'date': snapshot_data.get('date', ''),
            'system_information': snapshot_data.get('system_information', {}),
            'length': len(dac0),
            'dac0_i': dac0.real,
            'dac0_q': dac0.imag,
            'dac0_amplitude': np.abs(dac0),
            'dac0_phase': np.angle(dac0),
            'dac1_i': dac1.real,
            'dac1_q': dac1.imag,
            'dac1_amplitude': np.abs(dac1),
            'dac1_phase': np.angle(dac1),
        }

    @staticmethod
    def export_adc_snapshot(filename, snapshot_data, file_format=None):
        """
        Export ADC snapshot data to file.

        Args:
            filename: Output file path. If it has a recognised extension
                      (.npy, .json), the format is inferred from it.
            snapshot_data: dict from get_adc_snapshot() or parse_adc_snapshot().
            file_format: If given, appended as extension to filename.
                         Otherwise inferred from filename, defaulting to 'npy'.
        """
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json'), default='npy')

        if 'adc_i' not in snapshot_data:
            data_dict = ReadoutClient.parse_adc_snapshot(snapshot_data)
        else:
            data_dict = snapshot_data

        dirpath = os.path.dirname(filepath)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath)

        if file_format == 'npy':
            np.save(filepath, data_dict)
        elif file_format == 'json':
            json_dict = {}
            for key, value in data_dict.items():
                if isinstance(value, np.ndarray):
                    json_dict[key] = value.tolist()
                elif isinstance(value, dict):
                    json_dict[key] = {
                        k: v.tolist() if isinstance(v, np.ndarray) else v
                        for k, v in value.items()
                    }
                else:
                    json_dict[key] = value
            with open(filepath, 'w') as f:
                json.dump(json_dict, f, indent=4)
        else:
            raise ValueError(f"Unsupported file_format '{file_format}'. Use 'npy' or 'json'.")

    @staticmethod
    def export_dac_snapshot(filename, snapshot_data, file_format=None):
        """
        Export DAC snapshot data to file.

        Args:
            filename: Output file path. If it has a recognised extension
                      (.npy, .json), the format is inferred from it.
            snapshot_data: dict from get_dac_snapshot() or parse_dac_snapshot().
            file_format: If given, appended as extension to filename.
                         Otherwise inferred from filename, defaulting to 'npy'.
        """
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json'), default='npy')

        if 'dac0_i' not in snapshot_data:
            data_dict = ReadoutClient.parse_dac_snapshot(snapshot_data)
        else:
            data_dict = snapshot_data

        dirpath = os.path.dirname(filepath)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath)

        if file_format == 'npy':
            np.save(filepath, data_dict)
        elif file_format == 'json':
            json_dict = {}
            for key, value in data_dict.items():
                if isinstance(value, np.ndarray):
                    json_dict[key] = value.tolist()
                elif isinstance(value, dict):
                    json_dict[key] = {
                        k: v.tolist() if isinstance(v, np.ndarray) else v
                        for k, v in value.items()
                    }
                else:
                    json_dict[key] = value
            with open(filepath, 'w') as f:
                json.dump(json_dict, f, indent=4)
        else:
            raise ValueError(f"Unsupported file_format '{file_format}'. Use 'npy' or 'json'.")

    def perform_sweep(self, centers, spans, points, samples_per_point,direction='up', phases=None):
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        if phases is not None:
            self.set_tone_phases(np.atleast_1d(phases))
        else:
            self._warn_zero_phases()

        message = {
            'request': 'sweep',
            'centers': centers.tolist(),
            'spans': spans.tolist(),
            'points': points,
            'samples_per_point': samples_per_point,
            'direction': direction
        }
        return self.send_request(message)

    def perform_retune(self, centers, spans, points, samples_per_point, direction='up', method='max_gradient', freq_offsets=None, phases=None):
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        if phases is not None:
            self.set_tone_phases(np.atleast_1d(phases))
        else:
            self._warn_zero_phases()

        #handle freq_offsets, if None, all zeros, if scalar, make array of that value, if array, ensure correct length
        if freq_offsets is None:
            freq_offsets = np.zeros_like(centers)
        elif np.isscalar(freq_offsets):
            freq_offsets = np.full_like(centers, freq_offsets)
        else:
            freq_offsets = np.atleast_1d(freq_offsets)
        if freq_offsets.shape != centers.shape:
            raise ValueError("freq_offsets must be None, a scalar, or have the same shape as centers")

        assert method in ['max_gradient','min_mag'], "method must be 'max_gradient' or 'min_mag'"

        #warn if freq_offsets are much larger than half of the spans
        if np.any(np.abs(freq_offsets) > spans / 2):
            print("Info: some freq_offsets are larger than half of the spans."
                  "Noise calculations may end up missing a corresponding sweep point.")

        message = {
            'request': 'retune',
            'centers': centers.tolist(),
            'spans': spans.tolist(),
            'points': points,
            'samples_per_point': samples_per_point,
            'direction': direction,
            'method': method,
            'freq_offsets': freq_offsets.tolist()
        }
        return self.send_request(message)

    def get_sweep_progress(self):
        message = {'request': 'get_sweep_progress'}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['progress']
        else:
            print(f"Error getting sweep progress: {response['message']}")
            return response

    def wait_for_sweep(self, poll_interval=1.0, progress_bar=True):
        """
        Block until the current sweep completes, optionally displaying progress.

        Args:
            poll_interval (float): Seconds between progress polls. Default 1.0.
            progress_bar (bool): If True, display an ASCII progress bar. If False,
                                 print a plain numeric progress line. Default True.
        """
        import sys
        bar_width = 40
        while True:
            progress = self.get_sweep_progress()
            if isinstance(progress, dict):
                # Error response from get_sweep_progress
                break
            pct = float(progress)
            if progress_bar:
                filled = int(bar_width * pct)
                bar = '#' * filled + '-' * (bar_width - filled)
                sys.stdout.write(f'\rSweep progress: [{bar}] {pct*100:5.1f}%')
                sys.stdout.flush()
            else:
                sys.stdout.write(f'\rSweep progress: {pct*100:5.1f}%')
                sys.stdout.flush()
            if pct >= 1.0:
                sys.stdout.write('\n')
                sys.stdout.flush()
                break
            time.sleep(poll_interval)

    def get_sweep_data(self):
        message = {'request': 'get_sweep_data'}
        response = self.send_request(message)
        if response['status'] == 'success':
            sweep_data = response['data']
            return sweep_data
        else:
            print(f"Error getting sweep_data: {response['message']}")
            return response

    def get_sweep_raw_samples(self):
        message = {'request': 'get_sweep_raw_samples'}
        response = self.send_request(message)
        if response['status'] == 'success':
            sweep_data = response['data']
            # return np.array(sweep_data['data_i'])+1j*np.array(sweep_data['data_q'])
            samples,points,tones = sweep_data['samples'],sweep_data['points'],sweep_data['tones']
            data_i_bytes = base64.b64decode(sweep_data['data_i'])
            data_q_bytes = base64.b64decode(sweep_data['data_q'])
            data_i = np.frombuffer(data_i_bytes, dtype='float64').reshape((samples,points,tones)).copy()
            data_q = np.frombuffer(data_q_bytes, dtype='float64').reshape((samples,points,tones)).copy()
            return data_i+1j*data_q
        else:
            print(f"Error getting sweep_data: {response['message']}")
            return response

    def get_sweep_txt(self):
        message = {'request': 'get_sweep_txt'}
        response = self.send_request(message)
        if response['status'] == 'success':
            sweep_txt = response['data']
            return sweep_txt
        else:
            print(f"Error getting sweep_data: {response['message']}")
            return response

    def parse_sweep_data(self, sweep_data, apply_phase_correction=False):
        """
        Parse raw sweep data from the server into numpy arrays.
        
        Args:
            sweep_data: Raw sweep data dictionary from get_sweep_data()
            apply_phase_correction (bool): Correct for phase jumps at filterbank channel
                                           edges. Default is False. DEPRECATED: This 
                                           correction is no longer needed following 
                                           firmware fixes and will be removed in a 
                                           future version.
        
        Returns:
            dict: Parsed sweep data with 'sweep_f', 'sweep_i', 'sweep_q', 'sweep_ei', 
                  'sweep_eq' arrays and metadata.
        """
        info = sweep_data['system_information']
        date = sweep_data['date']
        num_tones = int(sweep_data['num_tones'])
        num_points = int(sweep_data['num_points'])
        samples_per_point = int(sweep_data['samples_per_point'])
        
        sweep_f_bytes = base64.b64decode(sweep_data['sweep']['f'])
        sweep_z_bytes = base64.b64decode(sweep_data['sweep']['z'])
        sweep_e_bytes = base64.b64decode(sweep_data['sweep']['e'])
        sweep_f = np.frombuffer(sweep_f_bytes, dtype='f8').reshape((num_points, num_tones)).copy()
        sweep_z = np.frombuffer(sweep_z_bytes, dtype='complex128').reshape((num_points, num_tones)).copy()
        sweep_e = np.frombuffer(sweep_e_bytes, dtype='complex128').reshape((num_points, num_tones)).copy()
        if 'tt' in sweep_data['sweep']:
            sweep_tt_bytes = base64.b64decode(sweep_data['sweep']['tt'])
            sweep_tt = np.frombuffer(sweep_tt_bytes, dtype='u8').copy()
        else:
            sweep_tt = np.zeros(num_points, dtype=np.uint64)

        if apply_phase_correction:
                
                #get bin indexes
                udc = self.config['rf_frontend']['connected']
                lo = self.config['rf_frontend'].get('tx_mixer_lo_frequency_hz', 0.0) if udc else 0.0
                sb = self.config['rf_frontend'].get('tx_mixer_sideband', 1) if udc else 1
                if lo is None:
                    lo = 0.0
                if sb is None:
                    sb = 1

                adcclk = info['adc_clk_hz']
                dacclk = adcclk
                dacduc = info['dac_duc_mixer_frequency_hz']
                dacnyq = info['nyquist_zone_dac0']
                txnfft = 8192
                rxnfft = 8192
                bin_freqs = np.fft.fftfreq(txnfft, 1.0/(dacclk))
                sorted_indices = np.argsort(bin_freqs)
                bin_freqs_sorted = bin_freqs[sorted_indices]

                rffreqs = sweep_f
                iffreqs = sb*(rffreqs-lo)
                bbfreqs = iffreqs - dacduc
                bbflat = np.ravel(bbfreqs.swapaxes(0,1))
                idx = np.searchsorted(bin_freqs_sorted, bbflat)
                idx = np.clip(idx, 1, 8192 - 1)
                left = bin_freqs_sorted[idx - 1]
                right = bin_freqs_sorted[idx]
                closer_on_right = (bbflat - left) > (right - bbflat)
                final_indices = sorted_indices[idx - 1 + closer_on_right.astype(int)]
                bbbins = final_indices.reshape(bbfreqs.swapaxes(0,1).shape).swapaxes(0,1)
    
                filterbank_bin_numbers = np.around(bbbins)
                sweep_z[filterbank_bin_numbers%2==1]*=np.exp(1j*np.pi)

        sweep_i = sweep_z.real
        sweep_q = sweep_z.imag
        err_i = sweep_e.real
        err_q = sweep_e.imag

        # sweep_f = np.array([sweep_data['sweep'][f'{i:04d}']['f'] for i in range(num_tones)])
        # sweep_i = np.array([sweep_data['sweep'][f'{i:04d}']['i'] for i in range(num_tones)])
        # sweep_q = np.array([sweep_data['sweep'][f'{i:04d}']['q'] for i in range(num_tones)])
        # err_i = np.array([sweep_data['sweep'][f'{i:04d}']['ei'] for i in range(num_tones)])
        # err_q = np.array([sweep_data['sweep'][f'{i:04d}']['eq'] for i in range(num_tones)])
        # sweep_z = sweep_i + 1j*sweep_q
        # sweep_e = err_i + 1j*err_q
        data_dict = {'date': date,
                        'num_tones': num_tones,
                        'num_points': num_points,
                        'samples_per_point': samples_per_point,
                        'system_information': info,
                        'sweep_f': sweep_f,
                        'sweep_i': sweep_i,
                        'sweep_q': sweep_q,
                        'sweep_ei': err_i,
                        'sweep_eq': err_q,
                        'telescope_time': sweep_tt
                        }
        return data_dict

    @staticmethod
    def export_sweep(filename, sweep_data, file_format=None):
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json', 'csv', 'dirfile', 'hdf5'), default='npy')

        dirpath = os.path.dirname(filepath)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath)

        if 'sweep_eq' not in sweep_data.keys():
            sweep_dict = ReadoutClient.parse_sweep_data(sweep_data)
        else:
            sweep_dict = sweep_data

        if file_format == 'npy':
            np.save(filepath, sweep_dict)

        elif file_format == 'json':
            # Convert numpy arrays to lists for JSON serialization, including nested arrays
            json_data_dict = {}
            for key, value in sweep_dict.items():
                if isinstance(value, dict):
                    json_data_dict[key] = {
                        sub_key: sub_value.tolist() if isinstance(sub_value, np.ndarray) else sub_value
                        for sub_key, sub_value in value.items()
                    }
                elif isinstance(value, np.ndarray):
                    json_data_dict[key] = value.tolist()
                else:
                    json_data_dict[key] = value

            with open(filepath, 'w') as file:
                json.dump(json_data_dict, file, indent=4)

        elif file_format == 'csv':
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['# date', sweep_dict['date']])
                writer.writerow(['# num_tones', sweep_dict['num_tones']])
                writer.writerow(['# num_points', sweep_dict['num_points']])
                writer.writerow(['# samples_per_point', sweep_dict['samples_per_point']])
                for key,value in sweep_dict['system_information'].items():
                    writer.writerow([f'# {key}', value])
                if 'telescope_time' in sweep_dict:
                    writer.writerow(['# telescope_time_per_point'] + [int(t) for t in sweep_dict['telescope_time']])
                header = []
                for k in range(len(sweep_dict['sweep_f'])):
                    header.extend([f'sweep_f_{k:04d}', f'sweep_i_{k:04d}', f'sweep_q_{k:04d}', f'err_i_{k:04d}', f'err_q_{k:04d}'])
                writer.writerow(header)
                for j in range(len(sweep_dict['sweep_f'][0])):
                    row = []
                    for i in range(len(sweep_dict['sweep_f'])):
                        row.extend([
                            f'{sweep_dict["sweep_f"][i][j]}',
                            f'{sweep_dict["sweep_i"][i][j]}',
                            f'{sweep_dict["sweep_q"][i][j]}',
                            f'{sweep_dict["sweep_ei"][i][j]}',
                            f'{sweep_dict["sweep_eq"][i][j]}'
                        ])
                    writer.writerow(row)

        elif file_format == 'dirfile':
            raise NotImplementedError("dirfile format not yet implemented.")
        elif file_format == 'hdf5':
            raise NotImplementedError("hdf5 format not yet implemented.")
        else:
            raise ValueError(f"Invalid file_format {file_format}. Must be one of 'json', 'npy', 'csv', 'dirfile' or 'hdf5'.")

    @staticmethod
    def import_sweep(filename):
        sweep_dict={}
        if filename.endswith('.npy'):
            sweep_dict = np.load(filename,allow_pickle=True).item()

        elif filename.endswith('.json'):
            with open(filename,'r') as file:
                sweep_dict = json.load(file)
                for item in sweep_dict:
                    if isinstance(sweep_dict[item],list):
                        sweep_dict[item] = np.array(sweep_dict[item])
                    if isinstance(sweep_dict[item],dict):
                        for sub_item in sweep_dict[item]:
                            if isinstance(sweep_dict[item][sub_item],list):
                                sweep_dict[item][sub_item] = np.array(sweep_dict[item][sub_item])
                            if isinstance(sweep_dict[item][sub_item],dict):
                                for sub_sub_item in sweep_dict[item][sub_item]:
                                    if isinstance(sweep_dict[item][sub_item][sub_sub_item],list):
                                        sweep_dict[item][sub_item][sub_sub_item] = np.array(sweep_dict[item][sub_item][sub_sub_item])

        elif filename.endswith('.csv'):
            with open(filename, mode='r') as file:
                lines=file.readlines()
                header_lines=0
                for line in lines:
                    if line.startswith('#'):
                        header_lines+=1
                        key = line.split(',')[0].lstrip('# ')
                        value = line[line.find(',')+1:].strip()
                        if value.startswith('"') and value.endswith('"'):
                            try:
                                value = eval(value[1:-1])
                            except:
                                value=value
                        else:
                            try:
                                value = eval(value)
                            except:
                                value = value
                        sweep_dict[key] = value

            data = np.genfromtxt(filename, delimiter=',',names=True,skip_header=header_lines)
            num_tones = sweep_dict['num_tones']
            num_points = sweep_dict['num_points']
            #samples_per_point = sweep_dict['samples_per_point']
            
            sweep_f = np.array([data[f'sweep_f_{i:04d}'] for i in range(num_tones)])
            sweep_i = np.array([data[f'sweep_i_{i:04d}'] for i in range(num_tones)])
            sweep_q = np.array([data[f'sweep_q_{i:04d}'] for i in range(num_tones)])
            err_i = np.array([data[f'err_i_{i:04d}'] for i in range(num_tones)])
            err_q = np.array([data[f'err_q_{i:04d}'] for i in range(num_tones)])
            sweep_dict['sweep_f'] = sweep_f
            sweep_dict['sweep_i'] = sweep_i
            sweep_dict['sweep_q'] = sweep_q
            sweep_dict['sweep_ei'] = err_i
            sweep_dict['sweep_eq'] = err_q

        elif filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        elif os.path.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        else:
            raise ValueError(f"Invalid file format {filename.split('.')[-1]}")

        return sweep_dict


    def receive_stream(self, num_tones=None, filename=None, print_data=False):
        iq_data=None
        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_stream')
            print(f"No filename specified, writing to {filename}")
        info = self.get_system_information()

        # Determine num_tones from system info (server sends only active tones)
        tone_indices = info.get('tone_indices')
        if num_tones is None:
            num_tones = len(tone_indices) if tone_indices is not None else 2048

        max_frame_bytes = 2048*2*4 + 10*4
        data = bytearray(max_frame_bytes)
        view = memoryview(data)

        metadata = {}
        metadata['date'] = time.strftime('%Y-%m-%d %H:%M:%S UTC%z')
        metadata['num_tones'] = num_tones
        metadata['sample_rate'] = self.get_sample_rate()
        metadata['format'] = '<i4'
        metadata['index_err'] = 2*num_tones-1+10
        metadata['index_cnt'] = 2*num_tones-1+9
        metadata['index_tt_lsb'] = 2*num_tones-1+8
        metadata['index_tt_msb'] = 2*num_tones-1+7
        metadata['index_flag_5'] = 2*num_tones-1+6
        metadata['index_flag_4'] = 2*num_tones-1+5
        metadata['index_flag_3'] = 2*num_tones-1+4
        metadata['index_flag_2'] = 2*num_tones-1+3
        metadata['index_flag_1'] = 2*num_tones-1+2
        metadata['index_flag_0'] = 2*num_tones-1+1
        metadata['ordering'] = 'I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, Q_tone1_sample0,..flags, tt_msb, tt_lsb, cnt, err .'
        metadata['system_information'] = info

        with open(filename+'.json','w') as file:
            json.dump(metadata,file,indent=4)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.stream_server_address, self.stream_server_port))
            with open(filename, 'wb') as file:
                print(f"Writing data to {filename}")
                t0=time.time()
                count=0
                ppid = os.getppid()
                while True:
                    try:
                        #quit if parent has changed, prevents zombie processes
                        if os.getppid() != ppid:
                            break

                        # Read data length
                        raw_datalen = s.recv(4)
                        if not raw_datalen:
                            continue
                        datalen = struct.unpack('>I', raw_datalen)[0]
                        if datalen==0:
                            continue
                        received_len = 0
                        while received_len < datalen:
                            packet_len = s.recv_into(view[received_len:], datalen - received_len)
                            if packet_len == 0:
                                break
                            received_len += packet_len
                        if received_len < datalen:
                            print(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                            break

                        # Write entire frame (already only active tones in user order)
                        file.write(data[:datalen])
                        count+=1

                        if print_data:
                            tone_data_bytes = datalen - 10*4
                            i = np.frombuffer(data[:tone_data_bytes:], dtype='<i4')[::2]
                            q = np.frombuffer(data[:tone_data_bytes:], dtype='<i4')[1::2]
                            err = np.frombuffer(data[datalen-4:datalen], dtype='<i4')
                            cnt = np.frombuffer(data[datalen-8:datalen-4], dtype='<i4')
                            tt_lsb = int(np.frombuffer(data[datalen-12:datalen-8], dtype='<i4')[0]) & 0xFFFFFFFF
                            tt_msb = int(np.frombuffer(data[datalen-16:datalen-12], dtype='<i4')[0]) & 0xFFFFFFFF
                            tt = (tt_msb << 32) + tt_lsb
                            iq_data=i+1j*q
                            print(f"Received IQ data: err={err} cnt={cnt} tt={tt} {iq_data.tolist()}\r",end='',flush=True)
                    except KeyboardInterrupt:
                        break

                    except Exception as e:
                        print(f"Error receiving stream data: {e}")
                        print(traceback.format_exc())
                        break

                t1=time.time()
                print()
                print(f"Received {count} samples in ~{t1-t0} seconds (~{count/(t1-t0)} samples per second)")
        return iq_data


    def receive_triggered_stream(self, num_tones=None, filename=None, print_data=False):
        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_triggered_stream')
            print(f"No filename specified, writing to {filename}")

        info = self.get_system_information()

        # Determine num_tones from system info (server sends only active tones)
        tone_indices = info.get('tone_indices')
        if num_tones is None:
            num_tones = len(tone_indices) if tone_indices is not None else 2048

        max_frame_bytes = 2048*2*4 + 10*4
        data = bytearray(max_frame_bytes)
        view = memoryview(data)

        metadata = {}
        metadata['date'] = time.strftime('%Y-%m-%d %H:%M:%S UTC%z')
        metadata['num_tones'] = num_tones
        metadata['sample_rate'] = self.get_sample_rate()
        metadata['format'] = '<i4'
        metadata['index_err'] = 2*num_tones-1+10
        metadata['index_cnt'] = 2*num_tones-1+9
        metadata['index_tt_lsb'] = 2*num_tones-1+8
        metadata['index_tt_msb'] = 2*num_tones-1+7
        metadata['index_flag_5'] = 2*num_tones-1+6
        metadata['index_flag_4'] = 2*num_tones-1+5
        metadata['index_flag_3'] = 2*num_tones-1+4
        metadata['index_flag_2'] = 2*num_tones-1+3
        metadata['index_flag_1'] = 2*num_tones-1+2
        metadata['index_flag_0'] = 2*num_tones-1+1
        metadata['ordering'] = 'I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, Q_tone1_sample0,..flags, tt_msb, tt_lsb, cnt, err .'
        metadata['system_information'] = info

        with open(filename+'.json','w') as file:
            json.dump(metadata,file,indent=4)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.stream_server_address, self.stream_server_port))
            with open(filename, 'wb') as file:
                print(f"Writing data to {filename}")
                t0=time.time()
                count=0
                ppid = os.getppid()
                while True:
                    try:
                        #quit if parent no longer exists
                        if os.getppid() != ppid:
                            break

                        # Read data length
                        raw_datalen = s.recv(4)
                        if not raw_datalen:
                            break
                        datalen = struct.unpack('>I', raw_datalen)[0]
                        received_len = 0
                        while received_len < datalen:
                            packet_len = s.recv_into(view[received_len:], datalen - received_len)
                            if packet_len == 0:
                                break
                            received_len += packet_len
                        if received_len < datalen:
                            print(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                            break

                        # Write entire frame (already only active tones in user order)
                        file.write(data[:datalen])
                        count+=1

                        if print_data:
                            tone_data_bytes = datalen - 10*4
                            i = np.frombuffer(data[:tone_data_bytes:], dtype='<i4')[::2]
                            q = np.frombuffer(data[:tone_data_bytes:], dtype='<i4')[1::2]
                            err = np.frombuffer(data[datalen-4:datalen], dtype='<i4')
                            cnt = np.frombuffer(data[datalen-8:datalen-4], dtype='<i4')
                            tt_lsb = int(np.frombuffer(data[datalen-12:datalen-8], dtype='<i4')[0]) & 0xFFFFFFFF
                            tt_msb = int(np.frombuffer(data[datalen-16:datalen-12], dtype='<i4')[0]) & 0xFFFFFFFF
                            tt = (tt_msb << 32) + tt_lsb
                            iq_data=i+1j*q
                            print(f"Received IQ data: err={err} cnt={cnt} tt={tt} {iq_data.tolist()}\r",end='',flush=True)
                    except KeyboardInterrupt:
                        break
                    except Exception as e:
                        print(f"Error receiving triggered stream data: {e}")
                        print(traceback.format_exc())
                        break

                t1=time.time()
                print()
                print(f"Received {count} samples in ~{t1-t0} seconds (~{count/(t1-t0)} samples per second)")
                s.close()

    @staticmethod
    def parse_stream(filename):
        """
        Parse a saved stream file into per-tone I/Q arrays.

        Data is already in user-tone order (the server sends only active
        tones in user order). num_tones in the metadata reflects the
        actual number of active tones.
        """
        with open(filename+'.json','r') as file:
            metadata = json.load(file)
        date = metadata['date']
        num_tones = metadata['num_tones']
        sample_rate = metadata['sample_rate']
        format = metadata['format']
        index_err = metadata['index_err']
        index_cnt = metadata['index_cnt']
        index_tt_lsb = metadata.get('index_tt_lsb')
        index_tt_msb = metadata.get('index_tt_msb')
        index_flag_0 = metadata['index_flag_0']
        index_flag_5 = metadata.get('index_flag_5', metadata.get('index_flag_7', index_flag_0 + 5))
        info = metadata['system_information']

        data = np.fromfile(filename,dtype=format)
        data = data.reshape(-1,2*num_tones+10).swapaxes(0,1)
        num_samples = data.shape[1]

        err = data[index_err]
        cnt = data[index_cnt]
        if index_tt_msb is not None and index_tt_lsb is not None:
            tt_msb = data[index_tt_msb].astype(np.int64) & 0xFFFFFFFF
            tt_lsb = data[index_tt_lsb].astype(np.int64) & 0xFFFFFFFF
            tt = (tt_msb << 32) + tt_lsb
        else:
            tt = np.zeros(num_samples, dtype=np.uint64)
        flags = data[index_flag_0:index_flag_5+1]

        i_data = data[:2*num_tones:2]
        q_data = data[1:2*num_tones:2]

        data_dict = {'date':date,
                     'num_tones':num_tones,
                     'num_samples':num_samples,
                     'sample_rate':sample_rate,
                     'system_information':info,
                     'i_data':{f'{i:04d}':i_data[i] for i in range(num_tones)},
                     'q_data':{f'{i:04d}':q_data[i] for i in range(num_tones)},
                     'packet_counter':cnt,
                     'packet_error':err,
                     'telescope_time':tt,
                     'stream_flags':{f'flag{i}':flags[i] for i in range(flags.shape[0])}
                     }
        return data_dict

    @staticmethod
    def export_stream_data(filename,data_dict,file_format=None):
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json', 'csv'), default='npy')

        date = data_dict['date']
        num_tones = data_dict['num_tones']
        num_samples = data_dict['num_samples']
        sample_rate = data_dict['sample_rate']
        info = data_dict['system_information']
        i_data = data_dict['i_data']
        q_data = data_dict['q_data']
        cnt = data_dict['packet_counter']
        err = data_dict['packet_error']
        flags = data_dict['stream_flags']

        if file_format == 'npy':
            np.save(filepath, data_dict)

        elif file_format == 'json':
            # Convert numpy arrays to lists for JSON serialization, including nested arrays
            json_data_dict = {}
            for key, value in data_dict.items():
                if isinstance(value, dict):
                    json_data_dict[key] = {
                        sub_key: sub_value.tolist() if isinstance(sub_value, np.ndarray) else sub_value
                        for sub_key, sub_value in value.items()
                    }
                elif isinstance(value, np.ndarray):
                    json_data_dict[key] = value.tolist()
                else:
                    json_data_dict[key] = value

            with open(filepath, 'w') as file:
                json.dump(json_data_dict, file)

        elif file_format == 'csv':
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['# date', date])
                writer.writerow(['# num_tones', num_tones])
                writer.writerow(['# num_samples', num_samples])
                writer.writerow(['# sample_rate',sample_rate])
                for key,value in info.items():
                    writer.writerow([f'# {key}', value])

                num_flags = len(data_dict['stream_flags'])
                header = []
                header.extend(['packet_counter', 'packet_error', 'telescope_time'])
                header.extend([f'flag{i}' for i in range(num_flags)])
                for i in range(data_dict['num_tones']):
                    header.extend([f'i_{i:04d}'])
                    header.extend([f'q_{i:04d}'])

                writer.writerow(header)
                # Write the data rows
                for j in range(data_dict['num_samples']):
                    row = []
                    row.append(data_dict['packet_counter'][j])
                    row.append(data_dict['packet_error'][j])
                    row.append(int(data_dict['telescope_time'][j]))
                    row.extend([data_dict['stream_flags'][f'flag{k}'][j] for k in range(num_flags)])
                    for i in range(data_dict['num_tones']):
                        row.extend([data_dict['i_data'][f'{i:04d}'][j]])
                        row.extend([data_dict['q_data'][f'{i:04d}'][j]])
                    writer.writerow(row)

        elif file_format == 'dirfile':
            raise NotImplementedError("dirfile format not yet implemented.")
        elif file_format == 'hdf5':
            raise NotImplementedError("hdf5 format not yet implemented.")
        else:
            raise ValueError(f"Invalid file_format {file_format}. Must be one of 'npy', 'json', 'csv', 'dirfile', or 'hdf5'.")

    @staticmethod
    def import_stream_data(filename):
        data_dict={}
        if filename.endswith('.npy'):
            data_dict = np.load(filename,allow_pickle=True).item()
        elif filename.endswith('.json'):
            with open(filename,'r') as file:
                data_dict = json.load(file)
                for item in data_dict:
                    if isinstance(data_dict[item],list):
                        data_dict[item] = np.array(data_dict[item])
                    if isinstance(data_dict[item],dict):
                        for sub_item in data_dict[item]:
                            if isinstance(data_dict[item][sub_item],list):
                                data_dict[item][sub_item] = np.array(data_dict[item][sub_item])
                            if isinstance(data_dict[item][sub_item],dict):
                                for sub_sub_item in data_dict[item][sub_item]:
                                    if isinstance(data_dict[item][sub_item][sub_sub_item],list):
                                        data_dict[item][sub_item][sub_sub_item] = np.array(data_dict[item][sub_item][sub_sub_item])
        elif filename.endswith('.csv'):
            with open(filename, mode='r') as file:
                lines=file.readlines()
                header_lines=0
                for line in lines:
                    if line.startswith('#'):
                        header_lines+=1
                        key = line.split(',')[0].lstrip('# ')
                        value = line[line.find(',')+1:].strip()
                        if key=='date':
                            value = value
                        elif value.startswith('"') and value.endswith('"'):
                            value = eval(value[1:-1])
                        else:
                            try:
                                value = eval(value)
                            except NameError:
                                value = value
                        data_dict[key] = value

            data = np.genfromtxt(filename, delimiter=',',names=True,skip_header=header_lines)
            i_data = {f'{i:04d}':data[f'i_{i:04d}'] for i in range(data_dict['num_tones'])}
            q_data = {f'{i:04d}':data[f'q_{i:04d}'] for i in range(data_dict['num_tones'])}
            data_dict['i_data'] = i_data
            data_dict['q_data'] = q_data
            data_dict['packet_counter'] = data['packet_counter']
            data_dict['packet_error'] = data['packet_error']
            if 'telescope_time' in data.dtype.names:
                data_dict['telescope_time'] = data['telescope_time'].astype(np.uint64)
            else:
                data_dict['telescope_time'] = np.zeros(len(data['packet_counter']), dtype=np.uint64)
            num_flags = sum(1 for name in data.dtype.names if name.startswith('flag'))
            data_dict['stream_flags'] = {f'flag{i}':data[f'flag{i}'] for i in range(num_flags)}

        elif filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        elif os.path.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        else:
            raise ValueError(f"Invalid file format {filename.split('.')[-1]}")
        return data_dict

    @staticmethod
    def generate_random_phases(freqs):
        """
        Generate random phases for a set of frequencies.
        """
        return np.random.uniform(0,2*np.pi,len(freqs))
    
    @staticmethod
    def generate_newman_phases(freqs):
        """
        Generate the Newman phases for a set of frequencies.

        If frequencies are exactly evenly spaced, the crest factor is minimised.

        If the frequency spacing is not exactly equal, the phases are offset to account for the spacing. For largely varying spacings, this method is pretty much the same as picking random frequencies.

        """
        freqs=np.atleast_1d(freqs)
        n=len(freqs)
        if n == 1:
            return np.zeros(1)
        freqssorted = np.sort(freqs)
        k = np.arange(len(freqs))
        # k = (freqs-freqssorted[0]) / (freqssorted[-1] - freqssorted[0])*(n-1)
        #k should range from 0 to n-1, and elements are proportional to the frequencies
        return np.pi*k**2/n

    @staticmethod
    def calculate_frequency_and_dissipation_noise(sweep_frequencies,sweep_complex_data,timestream_tone_frequency,timestream_complex_data,smooth_window_hz=1000):
        """
        Calculate the fractional frequency and dissipation noise timestreams from a sweep and complex timestream data.
        Valid only for small frequency and dissipation shifts close to the tone frequency.

        Parameters
        ----------
        sweep_frequencies : array
            The frequencies of the tone in the sweep.
        sweep_complex_data : array
            The complex data of the tone in the sweep.
        timestream_tone_frequency : float
            The frequency of the tone in the timestream
        timestream_complex_data : array
            The complex timestream data.
        smooth_window_hz : float, optional
            The window size for a Savitzky-Golay filter with poly-order=1. The default is 1000 Hz.
            The filter is applied to the sweep data to improve the estimate of the gradient.
            Timestram data is not smoothed.        

        Returns
        -------
        fractional_frequency_noise : array
            The fractional frequency noise timestream.
        fractional_dissipation_noise : array
            The fractional dissipation noise timestream.
        si0 : float
            The in-phase component of the smoothed sweep at the tone frequency.
        sq0 : float
            The quadrature component of the smoothed sweep at the tone frequency.
        didf : float
            The gradient of the in-phase component of the smoothed sweep at the tone frequency.
        dqdf : float
            The gradient of the quadrature component of the smoothed sweep at the tone frequency.
        """
        
        # # Find the index of the tone frequency in the sweep frequencies
        # tone_index = np.argmin(np.abs(sweep_frequencies-tone_frequency))
        # Note: now using interpolation instead of finding the closest frequency

        # Shorthands for the real and imaginary parts of the sweep and timestream data
        si = sweep_complex_data.real
        sq = sweep_complex_data.imag
        ti = timestream_complex_data.real
        tq = timestream_complex_data.imag

        #smooth the sweep data
        if smooth_window_hz:
            window_samples = np.max([3,int(smooth_window_hz/(sweep_frequencies[1]-sweep_frequencies[0]))])
            si = signal.savgol_filter(si, window_samples,1)
            sq = signal.savgol_filter(sq, window_samples,1)
            sz = si+1j*sq
        else:
            sz = sweep_complex_data
        
        # Calculate the gradient of the complex sweep data wrt the sweep frequencies 
        grad = np.gradient(sz,sweep_frequencies)

        # Calculate values at the tone frequency (with interpolation)
        # i0 = si[tone_index]
        # q0 = sq[tone_index]
        # didf = grad[tone_index].real
        # dqdf = grad[tone_index].imag
        si0 = np.interp(timestream_tone_frequency,sweep_frequencies,sz.real)
        sq0 = np.interp(timestream_tone_frequency,sweep_frequencies,sz.imag)
        didf = np.interp(timestream_tone_frequency,sweep_frequencies,grad.real)
        dqdf = np.interp(timestream_tone_frequency,sweep_frequencies,grad.imag)


        #Compute the frequency and dissipation timestreams
        divisor = didf**2 + dqdf**2
        frequency_noise = ((si0 - ti) * didf + (sq0 - tq) * dqdf) / divisor
        dissipation_noise = ((sq0 - tq) * didf - (si0 - ti) * dqdf) / divisor

        #Scale by the tone frequency to get fractional frequency and fractional dissipation
        fractional_frequency_noise = frequency_noise/timestream_tone_frequency
        fractional_dissipation_noise = dissipation_noise/timestream_tone_frequency

        return fractional_frequency_noise, fractional_dissipation_noise, si0, sq0, didf, dqdf
    
    @staticmethod
    def read_resonances_file(filename):
        """
        Read a resonances file and return columns keyed by the names that
        numpy.genfromtxt assigns (sanitized from the file header).
        """
        # Let genfromtxt parse the header itself
        data = np.genfromtxt(
            filename,
            delimiter='\t',
            names=True,       # use header row for field names
            autostrip=True,
            dtype=None,       # let numpy auto-detect dtypes
            encoding=None
        )

        # Build dict: use the field names that genfromtxt generated
        resonances = {name: data[name] for name in data.dtype.names}
        return resonances


    # TODO: Add a tone_powers parameter to wideband_sweep to allow specifying
    #       power levels across the band (e.g. per-tone or per-band).
    def parse_wideband_sweep(self, sweep_data, remove_phase_slope=True,
                             apply_phase_correction=False):
        """
        Parse raw sweep data from the server as a wideband sweep.

        Takes the raw sweep data dictionary (from get_sweep_data()) and produces
        a single concatenated trace across all tones, as if it were a single-tone
        sweep covering the full bandwidth.

        This is the same parsing that wideband_sweep() applies internally. Use it
        when you have already collected sweep data with wideband-style tone
        configuration and want to post-process it without re-running the sweep.

        Args:
            sweep_data: Raw sweep data dictionary from get_sweep_data().
            remove_phase_slope (bool): Remove linear phase slope from the
                concatenated sweep data. Default is True.
            apply_phase_correction (bool): Correct for phase jumps at filterbank
                channel edges. Default is False. DEPRECATED.

        Returns:
            dict: Parsed wideband sweep data with keys:
                - 'sweep_f': Array of frequencies [1, N_total_points]
                - 'sweep_i': Array of I values [1, N_total_points]
                - 'sweep_q': Array of Q values [1, N_total_points]
                - 'sweep_ei': Array of I errors [1, N_total_points]
                - 'sweep_eq': Array of Q errors [1, N_total_points]
                - 'wideband_sweep': True
                - Plus other metadata from parse_sweep_data
        """
        s = self.parse_sweep_data(sweep_data, apply_phase_correction=apply_phase_correction)
        f = s['sweep_f']
        z = s['sweep_i'] + 1j * s['sweep_q']

        # Concatenate all tones
        fcat = np.ravel(f.T)
        zcat = np.ravel(z.T)

        # Optionally remove linear phase slope
        if remove_phase_slope:
            phicat = np.unwrap(np.angle(zcat))
            slope = np.nanmedian(np.gradient(phicat, fcat))
            zcat *= np.exp(-1j * (slope * fcat))

        # Reformat as single-row arrays (like a single-tone sweep covering all frequencies)
        s['sweep_f'] = np.array([fcat])
        s['sweep_i'] = np.array([np.real(zcat)])
        s['sweep_q'] = np.array([np.imag(zcat)])
        s['sweep_ei'] = np.array([np.ravel(s['sweep_ei'].T)])
        s['sweep_eq'] = np.array([np.ravel(s['sweep_eq'].T)])

        s['wideband_sweep'] = True
        return s

    def wideband_sweep(self, bandwidth_hz=None, center_freq_hz=None, step_size_hz=10000,
                       num_tones=1024, samples_per_point=10, tone_powers_dbm='auto',
                       reference_plane='detector',
                       apply_phase_correction=False,
                       remove_phase_slope=True, 
                       optimise_tx_dynamic_range=True,
                       optimise_rx_gain=True,
                       cal_freeze=True,
                       verbose=True):
        """
        Perform a wideband sweep of the system using multiple tones.

        This method configures tones across the bandwidth, performs a sweep, and returns
        the concatenated sweep data covering the full requested bandwidth.

        Args:
            bandwidth_hz (float): Total bandwidth to measure. Default is full available bandwidth.
            center_freq_hz (float): Center frequency of the sweep. Default is band center.
            step_size_hz (float): Step size of the sweep in Hz. Number of sweep steps =
                                  bandwidth / step_size / num_tones. Default is 10000.
            num_tones (int): Number of tones to use in the sweep. More tones = fewer sweep
                             steps but wider spacing. Default is 1024.
            samples_per_point (int): Number of samples to integrate per sweep point. Default is 10.
            tone_powers_dbm (float or array-like or 'auto'): Tone power in dBm.
                If 'auto', calls maximise_tx_power() to optimise the dynamic range.
                If a scalar, all tones are set to uniform power at this level (the
                strongest tone achieves the specified power at the reference plane).
                If an array (must match num_tones), per-tone powers are applied.
                Default is 'auto'.
            reference_plane (str): Where tone_powers_dbm is specified: 'dac',
                'rf_output', or 'detector' (default).
            apply_phase_correction (bool): DEPRECATED. Correct for phase jumps at filterbank
                                           channel edges. Default is False. This correction is
                                           no longer needed following firmware fixes.
            remove_phase_slope (bool): Remove linear phase slope from the sweep data.
                                       Default is True.
            optimise_tx_dynamic_range (bool): If True, maximise DAC bit utilisation and
                adjust the TX analog chain when setting tone powers. Default is True.
            optimise_rx_gain (bool): If True, maximise ADC power utilisation and
                optimise the PFB FFT shift for best RX dynamic range after tones
                are configured. Calls maximise_rx_power(). Default is True.
            cal_freeze (bool): If True, freeze the ADC calibration after tones
                are configured and before sweeping, then unfreeze after the sweep
                completes. Prevents ADC drift during the sweep. Default is True.
            verbose (bool): Print progress information. Default is True.

        Returns:
            dict: Sweep data dictionary with keys:
                - 'sweep_f': Array of frequencies [1, N_total_points]
                - 'sweep_i': Array of I values [1, N_total_points]
                - 'sweep_q': Array of Q values [1, N_total_points]
                - 'sweep_ei': Array of I errors [1, N_total_points]
                - 'sweep_eq': Array of Q errors [1, N_total_points]
                - 'num_tones': Number of tones used
                - 'samples_per_point': Samples per point
                - 'system_information': System info at time of sweep
                - Plus other metadata from parse_sweep_data

        Raises:
            RuntimeError: If a sweep is already in progress, saturation detected, or sweep fails.
            ValueError: If requested bandwidth is out of range or tone spacing is too small.
        """
        # Check if a sweep is already running
        p = self.get_sweep_progress()
        if isinstance(p, dict):
            # Request failed (e.g. connection error) — no sweep running
            p = 0.0
        if p != 0.0 and p != 1.0:
            raise RuntimeError(f'Sweep already in progress ({p*100:.3f}%), wait for it to finish.')
        
        info = self.get_system_information()
                
        # Get RF frontend mixer configuration
        # TODO: if we have a frontend connected it might not have a mixer - needs updating.
        udc = self.config['rf_frontend']['connected']
        lo = self.config['rf_frontend'].get('tx_mixer_lo_frequency_hz', 0.0) if udc else 0.0
        sb = self.config['rf_frontend'].get('tx_mixer_sideband', +1) if udc else 1
        if lo is None:
            lo = 0.0
        if sb is None:
            sb = 1
        
        adcclk = info['adc_clk_hz']
        dacclk = adcclk
        dacduc = info['dac_duc_mixer_frequency_hz']
        txnfft = 8192
        
        # Calculate baseband and RF frequency limits
        dbbmin = -dacclk / 2
        dbbmax = +dacclk / 2
        dacmin = min([abs(dbbmin + dacduc), abs(dbbmax + dacduc)])
        dacmax = max([abs(dbbmin + dacduc), abs(dbbmax + dacduc)])
        
        rfmin = dacmin
        rfmax = dacmax

        
        if udc:
            if sb == 1:
                rfmin = lo + dacmin
                rfmax = lo + dacmax
            elif sb == -1:
                rfmin = lo - dacmax
                rfmax = lo - dacmin
            else:
                raise ValueError(f"Invalid sideband value {sb}, should be +1 for USB or -1 for LSB")

        # Set defaults for bandwidth and center frequency
        if bandwidth_hz is None:
            bandwidth_hz = (rfmax - rfmin)*0.90 # the last few hz of bandwidth tend to screw up the tones somehow
        if center_freq_hz is None:
            center_freq_hz = (rfmax + rfmin) / 2

        fmin = center_freq_hz - bandwidth_hz / 2
        fmax = center_freq_hz + bandwidth_hz / 2

        if (fmin < rfmin) or (fmax > rfmax):
            raise ValueError(f'Requested sweep out of band (band = {rfmin/1e6:.1f} - {rfmax/1e6:.1f} MHz, '
                           f'requested {fmin/1e6:.1f} - {fmax/1e6:.1f} MHz)')

        # Calculate tone start frequencies
        freqs, spacings = np.linspace(fmin, fmax, num_tones, endpoint=False, retstep=True)
        
        #DEPRECATED since v7.9-multitone
        if False:
            if spacings <= dacclk / txnfft:
                raise ValueError(f'Tone spacing must be greater than {dacclk/txnfft:.0f} Hz but is {spacings:.0f} Hz. '
                               f'Try fewer tones or wider bandwidth.')

        # Calculate the center frequencies
        sweep_points = int(bandwidth_hz / step_size_hz / num_tones)
        sweep_span = spacings * (sweep_points - 1) / sweep_points
        center_freqs = freqs + np.floor(sweep_points / 2) * spacings / sweep_points # converts start freqs to center freqs
        
        # Add small random offsets to avoid intermodulation distortion effects
        small_offset_scale = 0.9 # if larger than one then segments will overlap, if zero there will be worst possible IMD
        small_offsets = np.random.uniform(-sweep_span / sweep_points / 2 * small_offset_scale, 
                                          +sweep_span / sweep_points / 2 * small_offset_scale,
                                          num_tones)
        # Dont't add the offset to the endpoints to avoid going out of band
        small_offsets[0] = 0.0
        small_offsets[-1] = 0.0

        center_freqs += small_offsets

        # Compute phases to minimise crest factor (Newman phases)
        tone_phases = self.generate_newman_phases(center_freqs)

        if verbose:
            print(f'Wideband sweep configuration:')
            print(f'  RF band: {rfmin/1e6:.1f} - {rfmax/1e6:.1f} MHz')
            print(f'  Sweep range: {fmin/1e6:.1f} - {fmax/1e6:.1f} MHz ({bandwidth_hz/1e6:.1f} MHz)')
            print(f'  Num tones: {num_tones}, sweep points: {sweep_points}')
            print(f'  Total points: {num_tones * sweep_points}')

        # Configure tones
        self.set_tone_frequencies(center_freqs)
        self.set_tone_phases(tone_phases)

        if isinstance(tone_powers_dbm, str) and tone_powers_dbm == 'auto':
            # Maximum power/dynamic-range: set unit amplitudes first, then maximise
            self.set_tone_amplitudes(np.ones(num_tones))
            result = self.maximise_tx_power()
            if verbose:
                print(f'  Auto TX power: maximise_tx_power() -> {result}')
        elif tone_powers_dbm is not None:
            powers = np.broadcast_to(np.atleast_1d(tone_powers_dbm), num_tones).copy()
            if verbose:
                print(f'  Setting tone powers: {tone_powers_dbm} dBm at {reference_plane} '
                      f'(optimise_dynamic_range={optimise_tx_dynamic_range})')
            stp_response = self.set_tone_powers(powers,
                                reference_plane=reference_plane,
                                optimise_dynamic_range=optimise_tx_dynamic_range)
            if stp_response.get('status') != 'success':
                raise RuntimeError(
                    f"Failed to set tone powers: {stp_response.get('message', 'unknown error')}")
            if verbose:
                print(f'  set_tone_powers(): {stp_response.get("status")}')
        else:
            #should this be a no-op?
            self.set_tone_amplitudes(np.ones(num_tones))

        # # Check for saturation/overflow before sweeping and attempt to fix
        # outps = self.check_output_saturation()
        # inps = self.check_input_saturation()
        # dspof = self.check_dsp_overflow()

        # if outps['result']:
        #     if verbose:
        #         print(f'  DAC saturation detected — attempting fix...')
        #     self.fix_dac_saturation()
        #     outps = self.check_output_saturation()
        #     if outps['result']:
        #         raise RuntimeError(
        #             f"DAC saturation persists. Reduce tone_powers_dbm or num_tones.")
        # if inps['result']:
        #     if verbose:
        #         print(f'  ADC saturation detected — attempting fix...')
        #     self.fix_adc_saturation()
        #     inps = self.check_input_saturation()
        #     if inps['result']:
        #         raise RuntimeError(
        #             f"ADC saturation persists. Reduce tone_powers_dbm or num_tones.")
        # if dspof['result']:
        #     if verbose:
        #         print(f'  DSP overflow detected — attempting fix...')
        #     self.fix_dsp_overflow()
        #     dspof = self.check_dsp_overflow()
        #     if dspof['result']:
        #         raise RuntimeError(
        #             f"DSP overflow persists. Reduce tone_powers_dbm or num_tones.")

        # Optimise RX gain: maximise ADC power and PFB FFT shift
        if optimise_rx_gain:
            if verbose:
                print(f'  Optimising RX gain...')
            rx_result = self.maximise_rx_power()
            if verbose:
                print(f'  maximise_rx_power() -> {rx_result["status"]}')

        # Freeze calibration before sweep
        if cal_freeze:
            if verbose:
                print(f'  Freezing calibration...')
            self.set_cal_freeze(True)


        # Double check for saturation/overflow before sweeping and attempt to fix
        outps = self.check_output_saturation()
        inps = self.check_input_saturation()
        dspof = self.check_dsp_overflow()

        if outps['result']:
            if verbose:
                print(f'  DAC saturation detected — attempting fix...')
            self.fix_dac_saturation()
            outps = self.check_output_saturation()
            if outps['result']:
                raise RuntimeError(
                    f"DAC saturation persists. Reduce tone_powers_dbm or num_tones.")
        if inps['result']:
            if verbose:
                print(f'  ADC saturation detected — attempting fix...')
            self.fix_adc_saturation()
            inps = self.check_input_saturation()
            if inps['result']:
                raise RuntimeError(
                    f"ADC saturation persists. Reduce tone_powers_dbm or num_tones.")
        if dspof['result']:
            if verbose:
                print(f'  DSP overflow detected — attempting fix...')
            self.fix_dsp_overflow()
            dspof = self.check_dsp_overflow()
            if dspof['result']:
                raise RuntimeError(
                    f"DSP overflow persists. Reduce tone_powers_dbm or num_tones.")


        # Perform the sweep
        response = self.perform_sweep(center_freqs, sweep_span,
                                      points=sweep_points,
                                      samples_per_point=samples_per_point,
                                      direction='up')
        
        if response['status'] != 'success':
            raise RuntimeError(f"Sweep failed: {response['message']}")

        # Wait for sweep to complete
        import time
        while True:
            p = self.get_sweep_progress()
            if verbose:
                print(f'Sweep progress: {100*p:.1f}%', end='\r', flush=True)
            if p == 1.0:
                break
            time.sleep(1.0)
        if verbose:
            print()  # Newline after progress

        # Unfreeze calibration after sweep
        if cal_freeze:
            if verbose:
                print(f'  Unfreezing calibration...')
            self.set_cal_freeze(False)

        # Get and parse the sweep data as a wideband sweep
        s = self.parse_wideband_sweep(self.get_sweep_data(),
                                      remove_phase_slope=remove_phase_slope,
                                      apply_phase_correction=apply_phase_correction)

        # Add wideband_sweep-specific metadata
        s['bandwidth_hz'] = bandwidth_hz
        s['center_freq_hz'] = center_freq_hz
        s['step_size_hz'] = step_size_hz
        s['num_tones_used'] = num_tones
        s['sweep_points_per_tone'] = sweep_points
        s['tone_powers_dbm'] = self.get_tone_powers(reference_plane=reference_plane) if tone_powers_dbm is not None else None
        s['tone_powers_reference_plane'] = reference_plane

        return s


    def set_tones_helper(self, freqs, amps=None, phases=None, powers_dbm=None):
        """
        Convenience method: set frequencies, powers/amplitudes, and phases in one call.

        Checks for multiple tones mapping to the same FFT bin and warns the user.
        When using amplitudes (not powers_dbm), automatically scales per-bin
        amplitudes so that the sum per bin does not exceed 1.0.

        Args:
            freqs: Tone frequencies in Hz.
            amps: LO amplitude scales (0 to 1.0). Ignored if powers_dbm is provided.
            phases: Phase offsets in radians. Defaults to Newman phases if None.
            powers_dbm: Per-tone output power in dBm. If provided, overrides amps and
                        uses set_tone_powers() to apply calibrated power levels.
        """
        import warnings

        if freqs is None or len(freqs) == 0:
            raise ValueError("Frequencies must be provided and cannot be empty.")
        freqs = np.atleast_1d(freqs)
        if phases is None:
            phases = self.generate_newman_phases(freqs)

        self.set_tone_frequencies(freqs)

        # Check for multiple tones per FFT bin
        detailed = self.get_tone_frequencies(detailed_output=True)
        tx_bins = np.array(detailed['tx']['filterbank_bins'])
        unique_bins, counts = np.unique(tx_bins, return_counts=True)
        shared_mask = counts > 1

        if np.any(shared_mask):
            # Build a descriptive warning
            lines = []
            for fft_bin, count in zip(unique_bins[shared_mask], counts[shared_mask]):
                idxs = np.nonzero(tx_bins == fft_bin)[0]
                tone_freqs = freqs[idxs]
                freq_strs = ', '.join(f'{f/1e6:.4f} MHz' for f in tone_freqs)
                lines.append(f'  FFT bin {fft_bin}: {count} tones (tones {idxs.tolist()}, freqs [{freq_strs}])')
            detail_str = '\n'.join(lines)
            warnings.warn(
                f'Multiple tones map to the same FFT bin. '
                f'Their amplitudes will add coherently in the PSB output.\n{detail_str}'
            )

        if powers_dbm is not None:
            if np.any(shared_mask):
                warnings.warn(
                    'Tones sharing FFT bins detected with powers_dbm mode. '
                    'Cannot automatically scale powers — the actual output '
                    'power per bin will be higher than requested due to '
                    'coherent addition. Consider separating these tones.'
                )
            self.set_tone_powers(powers_dbm)
        else:
            if amps is None:
                amps = np.ones_like(freqs)
            amps = np.atleast_1d(amps).copy()
            if np.any(shared_mask):
                # Find the worst-case bin (highest amplitude sum) and scale
                # ALL tones uniformly so that bin stays <= 1.0.  This keeps
                # relative power constant across all tones.
                worst_scale = 1.0
                worst_bin = None
                for fft_bin, count in zip(unique_bins[shared_mask], counts[shared_mask]):
                    idxs = np.nonzero(tx_bins == fft_bin)[0]
                    bin_amp_sum = np.sum(amps[idxs])
                    if bin_amp_sum > 1.0:
                        scale_factor = 1.0 / bin_amp_sum
                        if scale_factor < worst_scale:
                            worst_scale = scale_factor
                            worst_bin = fft_bin
                if worst_scale < 1.0:
                    amps *= worst_scale
                    warnings.warn(
                        f'Scaled all tone amplitudes by {worst_scale:.4f} to '
                        f'keep per-bin sum <= 1.0 (worst-case bin {worst_bin})'
                    )
            self.set_tone_amplitudes(amps)
        self.set_tone_phases(phases)
        return

    def measure_path_group_delay(self,
                                 sweep_data=None,
                                 kid_frequencies=None,
                                 kid_q_factors=None,
                                 mask_hwhm_factor=10.0,
                                 median_filter_mhz=10.0,
                                 savgol_mhz=10.0,
                                 savgol_poly_order=3,
                                 save_to_config=False,
                                 save_to_csv=None,
                                 verbose=True,
                                 **sweep_kwargs):
        """
        Measure the full TX+RX path group delay across the band.

        Performs (or re-uses) a wideband sweep, computes the phase gradient
        (group delay) across the band, suppresses MKID resonance contributions,
        and returns a smooth group-delay vs frequency calibration.

        Filtering strategy
        ------------------
        1. **Phase derivative** — group delay is computed from
           ``np.gradient(unwrap(phase), freqs)``.

        2. **Resonance masking** — if ``kid_frequencies`` and ``kid_q_factors``
           are supplied, frequency channels within ``mask_hwhm_factor`` × HWHM
           of each resonance are excluded from subsequent filtering.

        3. **Median filter** — a sliding median of width ``median_filter_mhz``
           is applied to the unmasked group-delay points.  This removes
           outliers and resonance-tail contributions.

        4. **Savitzky-Golay filter** — a Savitzky-Golay filter with window
           ``savgol_mhz`` and order ``savgol_poly_order`` produces the final
           smooth calibration.  Masked regions are filled by interpolation
           before filtering.

        Args:
            sweep_data (dict, optional): Existing wideband sweep dict (from
                ``wideband_sweep()``).  If None a new sweep is performed using
                ``**sweep_kwargs``.
            kid_frequencies (array-like, optional): Known MKID resonance
                frequencies in Hz.  Used with ``kid_q_factors`` to build
                frequency masks that exclude resonance regions.
            kid_q_factors (array-like, optional): Loaded Q factors (Ql) for
                each resonance in ``kid_frequencies``.  If scalar, the same
                value is used for all resonances.  Required if
                ``kid_frequencies`` is provided; defaults to 10 000 if omitted.
            mask_hwhm_factor (float): Half-width of the exclusion zone around
                each resonance, expressed as a multiple of the HWHM
                (= fr / (2 * Ql)).  Default 10 — catches the main Lorentzian
                body and near tails.
            median_filter_mhz (float): Width of the sliding median filter
                (MHz).  Default 20 MHz.
            savgol_mhz (float): Window width of the Savitzky-Golay filter
                (MHz).  Default 20 MHz.
            savgol_poly_order (int): Polynomial order of the Savitzky-Golay
                filter.  Default 3.
            save_to_config (bool): If True, write the calibration to a CSV
                file and set ``rf_frontend.path_group_delay_ns`` in the
                config to the CSV basename so that ``push_config()`` will
                transfer it.  Uses ``save_to_csv`` as the filename, or
                ``path_group_delay.csv`` by default.
            save_to_csv (str, optional): If a file path is given, write a
                two-column CSV (``freq_hz,tau_ns``) to that path.  When
                combined with ``save_to_config``, also updates the config.
            verbose (bool): Print progress and summary.  Default True.
            **sweep_kwargs: Passed to ``wideband_sweep()`` when ``sweep_data``
                is None.

        Returns:
            dict with keys:
                ``'frequencies'``: 1-D array of frequencies (Hz).
                ``'tau_ns'``: 1-D array of smooth group delay values (ns).
                ``'tau_ns_raw'``: 1-D array of raw (unfiltered) group delay (ns).
                ``'mask'``: boolean array, True where data was included.
                ``'sweep_data'``: the sweep dict used (new or supplied).
        """
        import numpy as np
        try:
            from scipy.signal import medfilt, savgol_filter
        except ImportError:
            medfilt = None
            savgol_filter = None

        # --- acquire sweep data -------------------------------------------------
        if sweep_data is None:
            if verbose:
                print('measure_path_group_delay: performing wideband sweep...')
            sweep_kwargs.setdefault('remove_phase_slope', False)
            sweep_data = self.wideband_sweep(verbose=verbose, **sweep_kwargs)

        freqs = np.asarray(sweep_data['sweep_f'], dtype=float).ravel()
        s21 = (np.asarray(sweep_data['sweep_i'], dtype=float).ravel() +
               1j * np.asarray(sweep_data['sweep_q'], dtype=float).ravel())

        # Sort by frequency (wideband sweep may have tiled segments)
        order = np.argsort(freqs)
        freqs = freqs[order]
        s21 = s21[order]

        n_pts = len(freqs)

        # --- raw group delay from phase gradient --------------------------------
        phase = np.unwrap(np.angle(s21))
        dphase_df = np.gradient(phase, freqs)
        tau_ns_raw = -dphase_df / (2.0 * np.pi) * 1e9

        # --- resonance mask -----------------------------------------------------
        mask = np.ones(n_pts, dtype=bool)  # True = include

        if kid_frequencies is not None:
            kid_frequencies = np.atleast_1d(np.asarray(kid_frequencies, dtype=float))
            if kid_q_factors is None:
                q_arr = np.full(len(kid_frequencies), 1e4)
            else:
                q_arr = np.broadcast_to(
                    np.atleast_1d(np.asarray(kid_q_factors, dtype=float)),
                    kid_frequencies.shape).copy()

            for fr, Ql in zip(kid_frequencies, q_arr):
                hwhm = fr / (2.0 * max(Ql, 1.0))
                half_width = mask_hwhm_factor * hwhm
                mask &= ~((freqs >= fr - half_width) & (freqs <= fr + half_width))

            n_masked = np.sum(~mask)
            if verbose:
                print(f'  Resonance masking: excluded {n_masked} of {n_pts} points '
                      f'({100*n_masked/n_pts:.1f}%) around {len(kid_frequencies)} resonances')

        # --- median filter on included data ------------------------------------
        tau_ns_work = tau_ns_raw.copy()
        included_idx = np.where(mask)[0]

        if len(included_idx) < 3:
            raise RuntimeError(
                f'Too few unmasked points ({len(included_idx)}) for smoothing. '
                f'Reduce mask_hwhm_factor.')

        if n_pts > 1 and median_filter_mhz > 0:
            freq_span_mhz = (freqs[-1] - freqs[0]) / 1e6
            if freq_span_mhz > 0:
                pts_per_mhz = n_pts / freq_span_mhz
                kernel = max(3, int(round(pts_per_mhz * median_filter_mhz)))
                if kernel % 2 == 0:
                    kernel += 1
                if len(included_idx) > kernel:
                    if medfilt is not None:
                        tau_ns_work[included_idx] = medfilt(
                            tau_ns_work[included_idx],
                            kernel_size=min(kernel, len(included_idx) | 1))
                    else:
                        half = kernel // 2
                        filtered = tau_ns_work[included_idx].copy()
                        for i in range(len(included_idx)):
                            lo = max(0, i - half)
                            hi = min(len(included_idx), i + half + 1)
                            filtered[i] = np.median(tau_ns_work[included_idx[lo:hi]])
                        tau_ns_work[included_idx] = filtered
                    if verbose:
                        print(f'  Median filter: kernel {kernel} points '
                              f'({kernel/pts_per_mhz:.1f} MHz)')

        # --- interpolate across masked regions before savgol --------------------
        tau_for_savgol = tau_ns_work.copy()
        excluded_idx = np.where(~mask)[0]
        if len(excluded_idx) > 0:
            tau_for_savgol[excluded_idx] = np.interp(
                freqs[excluded_idx],
                freqs[included_idx],
                tau_ns_work[included_idx])

        # --- Savitzky-Golay filter ---------------------------------------------
        freq_span_mhz = (freqs[-1] - freqs[0]) / 1e6
        if savgol_filter is not None and freq_span_mhz > 0 and n_pts > savgol_poly_order + 2:
            pts_per_mhz = n_pts / freq_span_mhz
            savgol_window = max(savgol_poly_order + 2, int(round(pts_per_mhz * savgol_mhz)))
            if savgol_window % 2 == 0:
                savgol_window += 1
            savgol_window = min(savgol_window, n_pts if n_pts % 2 == 1 else n_pts - 1)

            tau_ns_smooth = savgol_filter(tau_for_savgol, savgol_window, savgol_poly_order)

            if verbose:
                print(f'  Savitzky-Golay filter: window {savgol_window} points '
                      f'({savgol_window/pts_per_mhz:.1f} MHz), order {savgol_poly_order}')
        else:
            tau_ns_smooth = tau_for_savgol
            if verbose:
                print('  Savitzky-Golay filter: skipped (scipy not available or too few points)')

        if verbose:
            tau_mean = float(np.mean(tau_ns_smooth))
            tau_min = float(np.min(tau_ns_smooth))
            tau_max = float(np.max(tau_ns_smooth))
            print(f'  Smoothed group delay: '
                  f'τ = {tau_mean:.2f} ns (range {tau_min:.2f}–{tau_max:.2f} ns)')

        result = {
            'frequencies': freqs,
            'tau_ns': tau_ns_smooth,
            'tau_ns_raw': tau_ns_raw,
            'mask': mask,
            'sweep_data': sweep_data,
        }

        # --- save to CSV --------------------------------------------------------
        # When save_to_config is True but no CSV path given, generate one
        # automatically so we don't bloat the config with huge inline arrays.
        if save_to_csv is None and save_to_config:
            import os
            os.makedirs(self.cal_dir, exist_ok=True)
            save_to_csv = os.path.join(self.cal_dir, 'path_group_delay.csv')

        if save_to_csv is not None:
            import csv, os
            save_to_csv = os.path.abspath(save_to_csv)
            with open(save_to_csv, 'w', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow(['freq_hz', 'tau_ns'])
                for f, tau in zip(freqs, tau_ns_smooth):
                    writer.writerow([f'{f:.3f}', f'{tau:.6f}'])
            if verbose:
                print(f'  Saved group delay calibration to {save_to_csv}')
            if save_to_config and self.config is not None:
                # Store the local path so _resolve_local_cal_path can find it;
                # push_config rewrites it to the server-relative form.
                self.config.setdefault('rf_frontend', {})['path_group_delay_ns'] = save_to_csv
                basename = os.path.basename(save_to_csv)
                with open(save_to_csv) as f:
                    self.calibration_files[basename] = f.read()
                if verbose:
                    print(f'  Config rf_frontend.path_group_delay_ns set to "{save_to_csv}"')

        return result

    def find_resonances(self, sweep_data=None, mode='wideband',
                        data_format='log_magnitude',
                        filter_params=None, finder_params=None, **kwargs):
        """
        Find MKID resonances in sweep data using the peak_finder module.

        Supports two modes:
        - 'wideband': searches the full concatenated sweep trace for all
          resonances (default, for wideband_sweep data).
        - 'targeted': searches within each tone's individual sweep for
          resonances. Returns per-tone results and flags tones with
          multiple resonances (doubles/triples).

        Args:
            sweep_data (dict, optional): Sweep data dictionary with keys
                'sweep_f', 'sweep_i', 'sweep_q'. If None, performs a new
                wideband_sweep (for mode='wideband') or raises an error
                (for mode='targeted').
            mode (str): 'wideband' or 'targeted'.
            data_format (str): Analysis format for peak finding. One of:
                'lin_magnitude', 'log_magnitude', 'phase', 'unwrapped_phase',
                'group_delay', 'complex_gradient'. Default is 'log_magnitude'.
            filter_params: FilterParams instance or dict.
            finder_params: PeakFinderParams instance or dict.
            **kwargs: Passed to wideband_sweep if sweep_data is None.

        Returns:
            For mode='wideband':
                list of ResonanceResult objects sorted by frequency.

            For mode='targeted':
                dict with keys:
                    'per_tone': list of lists, per_tone[i] is the list of
                        ResonanceResult objects found in tone i's sweep.
                    'all_resonances': flat list of all ResonanceResult objects.
                    'flagged_tones': list of tone indices with >1 resonance
                        (doubles, triples, etc.).
                    'num_tones': total number of tones.
        """
        from ..peak_finder import (
            find_mkid_resonances, FilterParams, PeakFinderParams
        )

        if isinstance(filter_params, dict):
            filter_params = FilterParams(**filter_params)
        if isinstance(finder_params, dict):
            finder_params = PeakFinderParams(**finder_params)

        if mode == 'wideband':
            if sweep_data is None:
                sweep_data = self.wideband_sweep(**kwargs)

            frequencies = np.ravel(sweep_data['sweep_f'])
            s21_complex = (np.ravel(sweep_data['sweep_i'])
                           + 1j * np.ravel(sweep_data['sweep_q']))

            return find_mkid_resonances(
                frequencies=frequencies,
                s21_complex=s21_complex,
                data_format=data_format,
                filter_params=filter_params,
                finder_params=finder_params,
            )

        elif mode == 'targeted':
            if sweep_data is None:
                raise ValueError(
                    "sweep_data must be provided for mode='targeted'. "
                    "Use parse_sweep_data() output (shape: N_points x N_tones).")

            sf = np.atleast_2d(sweep_data['sweep_f'])
            si = np.atleast_2d(sweep_data['sweep_i'])
            sq = np.atleast_2d(sweep_data['sweep_q'])

            if sf.shape[0] == 1:
                raise ValueError(
                    "Targeted mode requires per-tone sweep data "
                    "(shape N_points x N_tones), not wideband.")

            n_points, n_tones = sf.shape
            per_tone = []
            all_resonances = []
            flagged_tones = []

            for t in range(n_tones):
                f_tone = sf[:, t]
                z_tone = si[:, t] + 1j * sq[:, t]

                results = find_mkid_resonances(
                    frequencies=f_tone,
                    s21_complex=z_tone,
                    data_format=data_format,
                    filter_params=filter_params,
                    finder_params=finder_params,
                )
                per_tone.append(results)
                all_resonances.extend(results)

                if len(results) > 1:
                    flagged_tones.append(t)

            all_resonances.sort(key=lambda r: r.frequency)

            return {
                'per_tone': per_tone,
                'all_resonances': all_resonances,
                'flagged_tones': flagged_tones,
                'num_tones': n_tones,
            }

        else:
            raise ValueError(f"Unknown mode '{mode}'. Use 'wideband' or 'targeted'.")

    def find_resonance_frequencies(self, sweep_data=None, **kwargs):
        """
        Convenience method to get just the resonance frequencies.
        
        Args:
            sweep_data: Optional sweep data dict. If None, performs a sweep.
            **kwargs: Passed to find_resonances.
        
        Returns:
            np.ndarray: Array of resonance frequencies in Hz
        """
        resonances = self.find_resonances(sweep_data, **kwargs)
        return np.array([r.frequency for r in resonances])


if __name__=='__main__':
    import argparse
    parser = argparse.ArgumentParser(description="SOUK MKID readout client")
    parser.add_argument(
        "config",
        help="Path to config YAML file.",
    )
    args = parser.parse_args()

    client = ReadoutClient(config_file=args.config)
    print('Starting triggered stream...')
    client.enable_triggered_stream()
    try:
        client.receive_triggered_stream()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error receiving triggered stream data: {e}")
    finally:
        pass
    sys.exit(0)

"""
# Example usage
client = ReadoutClient(config_file='config/config.yaml')

# Get a parameter
sample_rate_hz = client.get_parameter('sample_rate_hz')
print(f"Sample rate, Hz: {sample_rate_hz}")

# Set a parameter
set_result = client.set_parameter('nyquist_zone', 2)
print(f"Set result: {set_result}")

# Initialize the firmware
initialize_result = client.initialize_firmware()
print(f"Initialize firmware result: {initialize_result}")

# Enable continuous streaming
enable_stream_result = client.enable_stream()
print(f"Enable stream result: {enable_stream_result}")

# Disable continuous streaming
disable_stream_result = client.disable_stream()
print(f"Disable stream result: {disable_stream_result}")

# Enable triggered streaming
enable_triggered_stream_result = client.enable_triggered_stream()
print(f"Enable triggered stream result: {enable_triggered_stream_result}")

# Disable triggered streaming
disable_triggered_stream_result = client.disable_triggered_stream()
print(f"Disable triggered stream result: {disable_triggered_stream_result}")

# Perform a retune
retune_result = client.perform_retune(1.5e9, 1e6, 101, 10)
print(f"Retune result: {retune_result}")

# Stream a finite number of samples
client.get_samples(10)

# Cancel tasks
cancel_result = client.cancel_tasks()
print(f"Cancel result: {cancel_result}")

"""
