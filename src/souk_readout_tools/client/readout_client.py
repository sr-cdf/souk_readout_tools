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

    In [3]: client.get_info('server')
    Out[3]:
    {'process_name': 'readout_srv_0',
     'ip_addresses': '10.11.11.11 192.168.2.224',
     'pwd': '/home/casper/readout_server',
     ....}
    }

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
Version: 1.2.0

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
import ast
import base64
import io
import logging
import so3g
import spt3g.core

from souk_readout_tools.config_utils import copy_template_config


# Config keys whose string values are calibration file paths.
# Each entry is a path tuple of arbitrary depth into the config dict.
# These fields can also hold scalars, inline [freq, dB] arrays, or None —
# only strings trigger file handling.
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
    ('rf_frontend', 'mixerless_module', 'tx_amp_enabled_s21_db'),
    ('rf_frontend', 'mixerless_module', 'tx_amp_bypassed_s21_db'),
    ('rf_frontend', 'mixerless_module', 'tx_amp_bypass_delta_s21_db'),
    ('rf_frontend', 'mixerless_module', 'rx_amp_enabled_s21_db'),
    ('rf_frontend', 'mixerless_module', 'rx_amp_bypassed_s21_db'),
    ('rf_frontend', 'mixerless_module', 'rx_amp_bypass_delta_s21_db'),
    ('rf_frontend', 'mixerless_module', 'tx_group_delay_ns'),
    ('rf_frontend', 'mixerless_module', 'rx_group_delay_ns'),
    ('rf_frontend', 'path_group_delay_ns'),
    ('cryostat', 'input_s21_db'),
    ('cryostat', 'output_s21_db'),
]


def _cal_path_get(cfg, path):
    """Walk a path tuple into a nested dict; return None if any step is missing."""
    cur = cfg
    for k in path:
        if not isinstance(cur, dict):
            return None
        cur = cur.get(k)
        if cur is None:
            return None
    return cur


def _cal_path_set(cfg, path, value):
    """Walk a path tuple into a nested dict, creating intermediate dicts as needed."""
    cur = cfg
    for k in path[:-1]:
        cur = cur.setdefault(k, {})
    cur[path[-1]] = value


def _cal_path_str(path):
    """Render a path tuple as 'a.b.c' for log/error messages."""
    return '.'.join(path)


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

    def __init__(self, config_file=None, address=None, request_port=None,
                 stream_port=None, mock=False):
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
            mock: If True, emulate the readout server locally instead of opening
                  request/stream sockets.  This is intended for OCS/controller
                  testing without RFSoC hardware attached.
        """
        self.mock = bool(mock)
        connect_message = None
        connect_hint = None
        if self.mock and config_file is None and address is None:
            address = '127.0.0.1'
            if request_port is None:
                request_port = 10000

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
            self.pipeline_id = None
            self.config_file = None
            self.config_dir = os.getcwd()

            self.request_server_address = address
            self.request_server_port = request_port
            self.stream_server_address = address
            self.stream_server_port = stream_port

            connect_message = f'Connecting to {address}:{request_port} (stream port {stream_port}, no local config)'
            connect_hint = 'Use client.pull_config(save_as="my_config.yaml") to fetch and save the running config.'

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

        self.parameters = {}
        self.calibration_files = {}  # basename -> contents, populated by pull_config
        self._mock_server = None
        if self.mock:
            from souk_readout_tools.client.mock_readout import MockReadoutServer
            if self.config is None:
                self.config = MockReadoutServer.default_config(
                    self.request_server_address,
                    self.request_server_port,
                    self.stream_server_port)
                self.pipeline_id = self.config.get('firmware', {}).get('pipeline_id', 0)
            self._mock_server = MockReadoutServer(self)
            print(f'Using mock readout client (pipeline {self.pipeline_id})')
        elif connect_message is not None:
            print(connect_message)
            print(connect_hint)

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

    @staticmethod
    def _csv_metadata_value(value):
        """Escape embedded newlines so each metadata item stays on one CSV row."""
        if isinstance(value, str):
            return value.replace('\r\n', '\n').replace('\r', '\n').replace('\n', r'\n')
        return value

    @staticmethod
    def _write_csv_info_metadata(writer, info):
        """Write nested info metadata as comment rows in exported CSV files."""
        for section, section_data in info.items():
            if isinstance(section_data, dict):
                for key, value in section_data.items():
                    writer.writerow([f'# {section}.{key}', ReadoutClient._csv_metadata_value(value)])
            else:
                writer.writerow([f'# {section}', ReadoutClient._csv_metadata_value(section_data)])

    @staticmethod
    def _read_csv_comment_metadata(filename):
        """Read leading ``#`` metadata rows from a CSV export."""
        metadata = {}
        header_lines = 0
        with open(filename, mode='r') as file:
            for line in file:
                if not line.startswith('#'):
                    break
                header_lines += 1
                key = line.split(',')[0].lstrip('# ')
                value = line[line.find(',')+1:].strip()
                if value.startswith('"') and value.endswith('"'):
                    value = value[1:-1]
                try:
                    value = ast.literal_eval(value)
                except (ValueError, SyntaxError):
                    pass
                metadata[key] = value
        return metadata, header_lines

    @staticmethod
    def _json_ready(value):
        """Convert nested numpy/complex values into JSON-serialisable data."""
        if isinstance(value, np.ndarray):
            if np.iscomplexobj(value):
                return {
                    '__complex_ndarray__': True,
                    'real': value.real.tolist(),
                    'imag': value.imag.tolist(),
                }
            return value.tolist()
        if isinstance(value, np.generic):
            return value.item()
        if isinstance(value, complex):
            return {
                '__complex__': True,
                'real': float(value.real),
                'imag': float(value.imag),
            }
        if isinstance(value, dict):
            return {
                str(key): ReadoutClient._json_ready(sub_value)
                for key, sub_value in value.items()
            }
        if isinstance(value, (list, tuple)):
            return [ReadoutClient._json_ready(item) for item in value]
        return value

    @staticmethod
    def _restore_json_value(value):
        """Restore values written by ``_json_ready``."""
        if isinstance(value, dict):
            if value.get('__complex_ndarray__'):
                return (np.asarray(value['real'], dtype=float)
                        + 1j*np.asarray(value['imag'], dtype=float))
            if value.get('__complex__'):
                return complex(value['real'], value['imag'])
            return {
                key: ReadoutClient._restore_json_value(sub_value)
                for key, sub_value in value.items()
            }
        if isinstance(value, list):
            restored = [ReadoutClient._restore_json_value(item) for item in value]
            try:
                array = np.asarray(restored)
                if array.dtype != object:
                    return array
            except (TypeError, ValueError):
                pass
            return restored
        return value

    @staticmethod
    def _snapshot_csv_indices(field_names):
        """Return contiguous snapshot indices from snapshot_NNNN_i/q columns."""
        indices = []
        for name in field_names:
            if name.startswith('snapshot_') and name.endswith('_i'):
                suffix = name[len('snapshot_'):-len('_i')]
                if suffix.isdigit():
                    indices.append(int(suffix))

        indices = sorted(indices)
        if not indices:
            raise ValueError("No snapshot_NNNN_i columns found in snapshot CSV.")
        if indices != list(range(len(indices))):
            raise ValueError(
                "Snapshot CSV columns must be contiguous from snapshot_0000.")
        for index in indices:
            if f'snapshot_{index:04d}_q' not in field_names:
                raise ValueError(
                    f"Snapshot CSV missing snapshot_{index:04d}_q column.")
        return indices

    @staticmethod
    def _sweep_csv_indices(field_names, prefix):
        """Return and validate the numeric suffixes for one sweep CSV column family."""
        column_prefix = f'{prefix}_'
        indices = []
        for name in field_names:
            if name.startswith(column_prefix):
                suffix = name[len(column_prefix):]
                if suffix.isdigit():
                    indices.append(int(suffix))

        indices = sorted(indices)
        if not indices:
            raise ValueError(f"No {prefix}_NNNN columns found in sweep CSV.")
        if indices != list(range(len(indices))):
            raise ValueError(
                f"Sweep CSV {prefix}_NNNN columns must be contiguous from 0000.")
        return indices

    @staticmethod
    def _infer_sweep_csv_layout(sweep_dict, field_count, row_count):
        """Infer whether a sweep CSV uses corrected tone columns or legacy point columns."""
        marked_layout = sweep_dict.get('csv_layout')
        if marked_layout is not None:
            marked_layout = str(marked_layout).lower()
            if marked_layout in ('tone_columns', 'point_columns'):
                return marked_layout
            raise ValueError(f"Invalid sweep CSV layout marker {marked_layout!r}.")

        num_tones = int(sweep_dict['num_tones'])
        num_points = int(sweep_dict['num_points'])
        tone_columns_match = (field_count == num_tones and row_count == num_points)
        point_columns_match = (field_count == num_points and row_count == num_tones)

        if tone_columns_match and not point_columns_match:
            return 'tone_columns'
        if point_columns_match and not tone_columns_match:
            return 'point_columns'
        if tone_columns_match and point_columns_match:
            # Unmarked square CSVs are ambiguous. Older exports had no marker,
            # so preserve backward compatibility by treating them as legacy.
            return 'point_columns'

        # Old wideband CSVs were single-trace exports whose metadata still
        # recorded the original multi-tone sweep dimensions.
        if field_count == 1:
            return 'point_columns'

        raise ValueError(
            "Sweep CSV dimensions do not match num_tones/num_points metadata.")

    @staticmethod
    def _read_sweep_csv_columns(data, sweep_indices, prefix, csv_layout,
                                wideband_sweep=False):
        """Load one sweep CSV column family into the internal sweep array shape."""
        columns = np.array([
            np.atleast_1d(data[f'{prefix}_{i:04d}'])
            for i in sweep_indices
        ])
        if csv_layout == 'tone_columns':
            columns = columns.T
            if wideband_sweep and columns.shape[1] == 1:
                columns = columns.T
        return columns

    def send_request(self, message):
        """Send a length-prefixed JSON request to the server and return its response.

        ``message`` is the request ``dict`` (must contain a ``'request'`` key);
        it is JSON-encoded and sent to the server (or the mock).
        """
        if self.mock:
            return self._mock_server.send_request(message)

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.connect((self.request_server_address, self.request_server_port))
            except socket.gaierror as e:
                raise ConnectionError(
                    f"Error connecting to request server "
                    f"{(self.request_server_address, self.request_server_port)}: {e}"
                ) from e
            except ConnectionRefusedError as e:
                raise ConnectionError(
                    f"Connection refused connecting to request server "
                    f"{(self.request_server_address, self.request_server_port)}: {e}"
                ) from e
            except OSError as e:
                raise ConnectionError(
                    f"Error connecting to request server "
                    f"{(self.request_server_address, self.request_server_port)}: {e}"
                ) from e
            
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
        """Ask the server process to initialise itself from a config file."""
        message = {'request': 'initialise_server','config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()

        return response

    def _initialise_firmware(self,config_file=None):
        """Program and initialise firmware resources for all pipelines."""
        print(bcolors.WARNING + 'Warning: initialize_firmware will reset all pipelines, other clients will be affected.' + bcolors.ENDC)
        message = {'request': 'initialise_firmware', 'config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response
    
    def _initialise_pipeline(self,config_file=None):
        """Initialise only this client's configured readout pipeline."""
        message = {'request': 'initialise_pipeline', 'config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response

    def ensure_ready(self, config_file=None, level="pipeline"):
        """Ensure the server, firmware, or pipeline is ready before use.

        ``config_file`` is an optional config the server should (re)load first;
        ``level`` selects how much to bring up, one of ``'server'``,
        ``'firmware'``, or ``'pipeline'`` (default).
        """
        message = {'request': 'ensure_ready', 'config_filename': config_file, 'level': level}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response

    def pull_config(self, save_as=None, pull_calibration_files=True):
        """
        Pull the active config file from the server and load it into this client.

        This fetches the desired/applied config file only. Live hardware state
        is not patched into the config; use ``sync_config_from_system()`` when
        you explicitly want to capture current runtime settings into a new
        local config.

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
        for path in CAL_FILE_KEYS:
            value = _cal_path_get(self.config, path)
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
                      f'for {_cal_path_str(path)}')

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
            for path in CAL_FILE_KEYS:
                value = _cal_path_get(self.config, path)
                if not isinstance(value, str):
                    continue
                basename = os.path.basename(value)
                if basename in self.calibration_files:
                    new_path = os.path.join('calibrations', basename)
                    if config_text is not None:
                        config_text = config_text.replace(value, new_path)
                    _cal_path_set(self.config, path, new_path)

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
            for path in CAL_FILE_KEYS:
                value = _cal_path_get(push_config, path)
                if not isinstance(value, str):
                    continue
                # Resolve the local file path
                local_path = self._resolve_local_cal_path(value)
                if local_path is None:
                    print(f'  WARNING: calibration file not found for {_cal_path_str(path)}: {value}')
                    continue
                # Push the file to the server
                self.push_calibration(local_path)
                # Rewrite the config path to server-relative form
                basename = os.path.basename(local_path)
                server_path = f'pipeline_{pipeline_id}/calibrations/{basename}'
                _cal_path_set(push_config, path, server_path)
                print(f'  {_cal_path_str(path)}: pushed {basename}, path -> {server_path}')

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

    @staticmethod
    def _parse_group_delay_calibration_text(cal_text, source='<memory>'):
        """Parse a path-group-delay CSV into the resonator helper dict format."""
        data = np.genfromtxt(io.StringIO(cal_text), delimiter=',', names=True,
                             autostrip=True)

        if getattr(data, 'dtype', None) is not None and data.dtype.names:
            column_map = {name.lower(): name for name in data.dtype.names}
            freq_key = column_map.get('freq_hz') or column_map.get('frequency_hz')
            tau_key = column_map.get('tau_ns') or column_map.get('group_delay_ns')
            if freq_key is not None and tau_key is not None:
                return {
                    'frequencies': np.asarray(data[freq_key], dtype=float).ravel(),
                    'tau_ns': np.asarray(data[tau_key], dtype=float).ravel(),
                }

        raw = np.loadtxt(io.StringIO(cal_text), delimiter=',', ndmin=2)
        if raw.shape[1] < 2:
            raise ValueError(
                f"Group delay calibration '{source}' must have at least two columns.")
        return {
            'frequencies': np.asarray(raw[:, 0], dtype=float).ravel(),
            'tau_ns': np.asarray(raw[:, 1], dtype=float).ravel(),
        }

    def load_path_group_delay_calibration(self, value=None, pull_if_missing=True):
        """Load ``rf_frontend.path_group_delay_ns`` into an in-memory calibration dict.

        ``value`` is an explicit calibration (dict, array, or file path); when
        ``None`` it is read from the loaded config.  ``pull_if_missing`` pulls
        the config from the server first if none is loaded locally.
        """
        if value is None:
            if self.config is None:
                raise RuntimeError(
                    'No config loaded; pass a calibration entry or load a config first.')
            value = _cal_path_get(self.config, ('rf_frontend', 'path_group_delay_ns'))

        if value is None:
            return None
        if isinstance(value, dict):
            return value
        if not isinstance(value, str):
            arr = np.asarray(value, dtype=float)
            if arr.ndim == 0:
                return float(arr)
            return {
                'frequencies': arr[:, 0].ravel(),
                'tau_ns': arr[:, 1].ravel(),
            }

        basename = os.path.basename(value)
        if basename in self.calibration_files:
            return self._parse_group_delay_calibration_text(
                self.calibration_files[basename], source=basename)

        local_path = self._resolve_local_cal_path(value)
        if local_path is not None:
            with open(local_path, 'r') as file:
                return self._parse_group_delay_calibration_text(
                    file.read(), source=local_path)

        if pull_if_missing:
            response = self.send_request({
                'request': 'pull_calibration',
                'cal_filename': basename,
            })
            if response.get('status') == 'success':
                cal_text = response['cal_contents']
                self.calibration_files[basename] = cal_text
                return self._parse_group_delay_calibration_text(
                    cal_text, source=basename)

        raise FileNotFoundError(
            'Could not resolve path-group-delay calibration entry '
            f"'{value}'. Save the config locally or pull calibration files first.")


    def push_calibration(self, calibration_file):
        """Upload one local calibration file (``calibration_file`` path) to the
        RFSoC server, keyed by its basename."""
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
        """Download one calibration file from the RFSoC server.

        ``remote_file`` is the server-side filename; ``destination_file`` is
        the local path to write (defaults to the config dir + basename).
        """
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
        """Request a full firmware reset that affects all pipelines."""
        print(bcolors.WARNING + 'Warning: hard_reset will reset all pipelines, other clients will be affected.' + bcolors.ENDC)
        message = {'request':'hard_reset'}
        return self.send_request(message)

    def cancel_all_tasks(self):
        """Cancel all currently running asynchronous server tasks."""
        message = {'request': 'cancel'}
        return self.send_request(message)

    def get_info(self, sections=None):
        """Get structured system information by section.

        Parameters
        ----------
        sections : str, list of str, or ``'all'``, optional
            Which sections to include.  ``None`` returns the default set
            (server, versions, clock, timing, fpga, rfdc, pipeline, tones,
            rf_frontend, lna, rfsoc_sensors).  ``'all'`` includes
            diagnostics, config, calibrations, resonators, and registers
            as well. A string returns that section directly. A list returns
            a list of section dictionaries in the same order.

        Returns
        -------
        dict or list
            With ``None`` or ``'all'``, returns ``{section_name: section_dict}``.
            With a single section string, returns that section dict. With a
            list, returns a list of section dicts in the requested order.
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

    def get_timing_status(self):
        """Return PTP/chrony timing status reported by the RFSoC monitor."""
        response = self.send_request({'request': 'get_timing_status'})
        if response['status'] == 'success':
            return response['data']
        else:
            print(f"Error getting timing status: {response.get('message', 'unknown error')}")
            return response

    def health_check(self):
        """Quick system health summary for intermittent polling.

        Returns
        -------
        dict
            Compact health indicators including initialisation_level,
            clock_locked, timing_state, timing_ready, streaming/sweeping
            state, saturation/overflow bools, tone_count, client_count,
            and resonator tracking status.
        """
        message = {'request': 'health_check'}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['data']
        else:
            print(f"Error getting health check: {response.get('message', 'unknown error')}")
            return response

    def sync_config_from_system(self, save_as=None):
        """
        Update the in-memory config with live hardware state from the server.

        Fetches system information and writes the current firmware settings
        back into config['firmware']['defaults'], current regular/blind tone
        frequencies/amplitudes/phases into the defaults section, and the
        current RF peripheral state (attenuator dB values, amp bypass
        state) into config['rf_frontend'].  This captures the running
        state so that save_config() or push_config() will persist it.

        Does not write to disk unless ``save_as`` is provided — call
        save_config() afterwards to save.
        Returns the updated config dict; inspect ``self.config`` directly to
        see what was synced.
        """
        if self.config is None:
            raise RuntimeError('No config loaded. Use pull_config() or load a config file first.')

        sections = ['server', 'pipeline', 'rfdc', 'tones']
        info_list = self.get_info(sections)
        if not isinstance(info_list, list) or len(info_list) != len(sections):
            raise RuntimeError(f'Failed to get info: {info_list}')
        info = dict(zip(sections, info_list))
        if 'server' not in info:
            raise RuntimeError(f'Failed to get info: {info}')

        pipeline = info.get('pipeline', {})
        rfdc = info.get('rfdc', {})
        tones = info.get('tones', {})

        defaults = self.config.setdefault('firmware', {}).setdefault('defaults', {})

        # Direct mappings: (source_section, source_key) -> defaults key
        DIRECT_MAPS = [
            (pipeline, 'sync_delay',                'sync_delay'),
            (pipeline, 'acc_len',                   'acc_len'),
            (pipeline, 'internal_loopback',         'internal_loopback'),
            (pipeline, 'psb_scale',                 'psb_scale'),
            (pipeline, 'psb_fftshift',              'psb_fftshift'),
            (pipeline, 'pfb_fftshift',              'pfb_fftshift'),
            (rfdc,     'dsa',                       'dsa'),
            (rfdc,     'dac_duc_mixer_frequency_hz', 'dac_duc_mixer_frequency_hz'),
            (rfdc,     'adc_ddc_mixer_frequency_hz', 'adc_ddc_mixer_frequency_hz'),
        ]
        for src, info_key, defaults_key in DIRECT_MAPS:
            if info_key in src and src[info_key] is not None:
                defaults[defaults_key] = src[info_key]

        # VOP — use dac0 value
        if rfdc.get('vop_dac0') is not None and rfdc['vop_dac0'] != 0:
            defaults['vop'] = int(rfdc['vop_dac0'])

        # Mixer scale modes
        if rfdc.get('mixer_scale_1p0_dac0') is not None:
            defaults['dac_mixer_scale_1p0'] = bool(rfdc['mixer_scale_1p0_dac0'])
        if rfdc.get('mixer_scale_1p0_adc') is not None:
            defaults['adc_mixer_scale_1p0'] = bool(rfdc['mixer_scale_1p0_adc'])

        # Nyquist zone — use DAC0 value
        if rfdc.get('nyquist_zone_dac0') is not None:
            defaults['nyquist_zone'] = int(rfdc['nyquist_zone_dac0'])

        # QMC settings from DAC0
        dac_qmc = rfdc.get('qmc_settings_dac0')
        if dac_qmc is not None:
            if 'GainCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_gain'] = dac_qmc['GainCorrectionFactor']
            if 'OffsetCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_offset'] = dac_qmc['OffsetCorrectionFactor']
            if 'PhaseCorrectionFactor' in dac_qmc:
                defaults['dac_qmc_phase'] = dac_qmc['PhaseCorrectionFactor']

        # QMC settings from ADC
        adc_qmc = rfdc.get('qmc_settings_adc')
        if adc_qmc is not None:
            if 'GainCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_gain'] = adc_qmc['GainCorrectionFactor']
            if 'OffsetCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_offset'] = adc_qmc['OffsetCorrectionFactor']
            if 'PhaseCorrectionFactor' in adc_qmc:
                defaults['adc_qmc_phase'] = adc_qmc['PhaseCorrectionFactor']

        # Tone state.  Preserve the configured regular/blind split when the
        # live tone count still matches the config metadata.
        blind_indices = tones.get('blind_indices') or []
        regular_indices = tones.get('regular_indices') or []
        split_blind = bool(blind_indices) and tones.get('metadata_matches_config', False)

        def _take(values, indices):
            """Select indexed values while preserving None for unavailable state."""
            if values is None:
                return None
            return [values[i] for i in indices]

        if split_blind:
            if tones.get('frequencies_hz') is not None:
                defaults['frequencies'] = _take(tones['frequencies_hz'], regular_indices)
                defaults['blind_frequencies'] = _take(tones['frequencies_hz'], blind_indices)
            if tones.get('amplitudes') is not None:
                defaults['amplitudes'] = _take(tones['amplitudes'], regular_indices)
                defaults['blind_amplitudes'] = _take(tones['amplitudes'], blind_indices)
            if tones.get('phases_rad') is not None:
                defaults['phases'] = _take(tones['phases_rad'], regular_indices)
                defaults['blind_phases'] = _take(tones['phases_rad'], blind_indices)
            if tones.get('blind_spans') is not None:
                defaults['blind_spans'] = tones['blind_spans']
        else:
            if tones.get('frequencies_hz') is not None:
                defaults['frequencies'] = tones['frequencies_hz']
            if tones.get('amplitudes') is not None:
                defaults['amplitudes'] = tones['amplitudes']
            if tones.get('phases_rad') is not None:
                defaults['phases'] = tones['phases_rad']
            defaults['blind_frequencies'] = []
            defaults['blind_amplitudes'] = []
            defaults['blind_phases'] = []
            defaults['blind_spans'] = []

        # RF frontend peripheral state (attenuator, amp bypass)
        rf_response = self.get_rf_peripheral_status()
        rf_status = rf_response.get('result', {}) if isinstance(rf_response, dict) else {}
        rf = self.config.setdefault('rf_frontend', {})
        atten = rf.setdefault('attenuator', {})
        bypass = rf.setdefault('bypass_amps', {})
        rf_enabled = isinstance(rf_status, dict) and rf_status.get('enabled')
        readable_attenuation = rf_enabled and (
            rf_status.get('hardware')
            or rf_status.get('attenuator_backend') == 'fixed'
        )
        if readable_attenuation:
            atten['tx_value_db'] = rf_status.get('tx_attenuation_db')
            atten['rx_value_db'] = rf_status.get('rx_attenuation_db')
            if 'tx_amp_bypass' in rf_status:
                bypass['tx_amp_bypass'] = rf_status['tx_amp_bypass']
            if 'rx_amp_bypass' in rf_status:
                bypass['rx_amp_bypass'] = rf_status['rx_amp_bypass']
            if not rf_status.get('supports_bypass_amps', False):
                bypass['tx_amp_bypass'] = None
                bypass['rx_amp_bypass'] = None
        else:
            atten['tx_value_db'] = None
            atten['rx_value_db'] = None
            bypass['tx_amp_bypass'] = None
            bypass['rx_amp_bypass'] = None

        lna_response = self.get_lna_controller_status()
        lna_status = lna_response.get('result', {}) if isinstance(lna_response, dict) else {}
        if isinstance(lna_status, dict) and lna_status.get('enabled'):
            lna_cfg = self.config.setdefault('cryostat', {}).setdefault('lna_bias', {})
            for src, dst in (
                ('backend', 'backend'),
                ('lna_channel', 'lna_channel'),
                ('bias_voltage_v', 'bias_voltage_v'),
                ('soft_off', 'soft_off'),
                ('method', 'method'),
                ('blind', 'blind'),
            ):
                if src in lna_status and lna_status[src] is not None:
                    lna_cfg[dst] = lna_status[src]

        n_changed = sum(1 for _, _, k in DIRECT_MAPS if k in defaults)
        n_regular_tones = len(defaults.get("frequencies", []))
        n_blind_tones = len(defaults.get("blind_frequencies", []))
        n_tones = n_regular_tones + n_blind_tones
        # The raw text from pull_config() is now stale because this method
        # intentionally creates a new captured config.
        self.config_raw_text = None
        print(f'Config captured from live system state '
              f'({n_changed} parameters, {n_tones} tones, '
              f'{n_blind_tones} blind)')
        if save_as is not None:
            self.save_config(save_as)
        print('Use save_config() to write to disk, or push_config() to persist on the server.')
        return self.config

    def sync_config_to_local(self, save_as=None):
        """Capture current runtime settings into this client's local config.

        ``save_as`` optionally writes the synced config to that path (otherwise
        it is only updated in memory).
        """
        return self.sync_config_from_system(save_as=save_as)

    def set_parameter(self, param_name, param_value, **kwargs):
        """Set a named server/firmware parameter.

        Parameters
        ----------
        param_name : str
            Name of a server-registered parameter, e.g. ``'sample_rate_hz'``,
            ``'tone_frequencies'``, ``'tone_powers'``, ``'tone_amplitudes'``,
            ``'tone_phases'``, ``'fft_shift'``, ``'tx_attenuation_db'``,
            ``'rx_attenuation_db'``.  Most named setters on this client
            (``set_sample_rate`` etc.) are thin wrappers over this.
        param_value
            New value, in the type/shape the server expects for that
            parameter (scalar, array-like, or dict as appropriate).
        **kwargs
            Extra fields merged into the request and interpreted by the
            server's handler for ``param_name``. Tone-setting handlers accept
            ``autosync``.
        """
        message = {'request': 'set', 'param': param_name, 'value': param_value}
        message.update(kwargs)
        response = self.send_request(message)
        if response['status'] == 'success':
            return response
        else:
            print(f"Error setting parameter {param_name}: {response['message']}")
            return response

    def get_parameter(self, param_name, **kwargs):
        """Get a named server/firmware parameter, with optional request metadata.

        Parameters
        ----------
        param_name : str
            Name of a server-registered parameter (see
            :py:meth:`set_parameter` for examples).
        **kwargs
            Extra fields merged into the request and interpreted by the
            server's handler for ``param_name``.  For power-related
            parameters this includes ``reference_plane`` (one of ``'dac'``,
            ``'rf_output'``, ``'adc_input'``, ``'detector'``); other handlers
            accept their own qualifiers.
        """
        message = {'request': 'get', 'param': param_name}
        message.update(kwargs)
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['value']
        else:
            print(f"Error getting parameter {param_name}: {response['message']}")
            return response

    def set_sample_rate(self, sample_rate_hz):
        """Set the accumulator sample rate to ``sample_rate_hz`` (Hz)."""
        return self.set_parameter('sample_rate_hz',sample_rate_hz)

    def get_sample_rate(self):
        """Return the accumulator sample rate in Hz."""
        return self.get_parameter('sample_rate_hz')

    def get_telescope_time(self):
        """Return the latest telescope timestamp reported by the server."""
        return self.get_parameter('telescope_time')

    def set_tone_frequencies(self, tone_frequencies, autosync=True):
        """Set the active ``tone_frequencies`` (array-like, Hz)."""
        tone_frequencies = np.atleast_1d(tone_frequencies).tolist()
        return self.set_parameter(
            'tone_frequencies', tone_frequencies, autosync=bool(autosync))

    def get_tone_frequencies(self,detailed_output=False):
        """Return active tone frequencies; with ``detailed_output=True`` return
        the per-tone metadata dict instead of the bare frequency array."""
        if detailed_output:
            return self.get_parameter('tone_frequencies_detailed')
        else:
            return np.atleast_1d(self.get_parameter('tone_frequencies'))

    def get_tone_metadata(self):
        """Return tone role metadata, including regular/blind indices."""
        return self.get_parameter('tone_metadata')

    def get_blind_tone_indices(self):
        """Return integer indices of tones marked as blind monitors."""
        metadata = self.get_tone_metadata()
        return np.asarray(metadata.get('blind_indices', []), dtype=int)

    def get_regular_tone_indices(self):
        """Return integer indices of tones marked as regular readout tones."""
        metadata = self.get_tone_metadata()
        return np.asarray(metadata.get('regular_indices', []), dtype=int)

    def _update_tone_defaults_from_blind_result(self, result):
        """Patch local config defaults from a blind-tone server operation result."""
        defaults_update = result.get('config_defaults') if isinstance(result, dict) else None
        if defaults_update is None or self.config is None:
            return
        defaults = self.config.setdefault('firmware', {}).setdefault('defaults', {})
        for key, value in defaults_update.items():
            defaults[key] = value
        self.config_raw_text = None

    def get_blind_tones(self, reference_plane='detector'):
        """Return current blind-tone state and user-facing indices.

        ``reference_plane`` ('dac', 'rf_output', 'adc_input', or 'detector')
        sets the plane any reported powers are referred to.
        """
        response = self.send_request({
            'request': 'get_blind_tones',
            'reference_plane': reference_plane,
        })
        if response.get('status') != 'success':
            print(f"Error getting blind tones: {response.get('message')}")
            return response
        return response['result']

    def set_blind_tones(self, frequencies, amplitudes=None, phases=None,
                        spans=None, powers_dbm=None,
                        reference_plane='detector',
                        optimise_dynamic_range=False,
                        rx_policy='protect',
                        autosync=True):
        """Create or replace blind tones interactively.

        The server snapshots the currently active regular tones, appends the
        supplied blind tones, updates its in-memory tone
        metadata, and immediately applies to firmware.

        Parameters
        ----------
        frequencies : array-like
            Blind-tone frequencies (Hz).
        amplitudes : array-like or None, optional
            Per-tone amplitudes; server default used when ``None``.
        phases : array-like or None, optional
            Per-tone phases (radians); server default used when ``None``.
        spans : array-like or None, optional
            Per-tone sweep spans (Hz) recorded with the blind tones.
        powers_dbm : array-like or None, optional
            Per-tone powers (dBm at ``reference_plane``); used instead of
            ``amplitudes`` when given.
        reference_plane : str, optional
            Plane ``powers_dbm`` is specified at (default ``'detector'``).
        optimise_dynamic_range : bool, optional
            Re-optimise DAC bit utilisation when applying (default ``False``).
        rx_policy : str, optional
            RX-path policy when applying, as in :py:meth:`set_tone_powers`
            (default ``'protect'``).
        autosync : bool, optional
            If True (default), trigger firmware sync after applying tone
            frequency/amplitude changes.
        """
        message = {
            'request': 'set_blind_tones',
            'frequencies': np.atleast_1d(frequencies).tolist(),
            'reference_plane': reference_plane,
            'optimise_dynamic_range': optimise_dynamic_range,
            'rx_policy': rx_policy,
            'autosync': bool(autosync),
        }
        if amplitudes is not None:
            message['amplitudes'] = np.atleast_1d(amplitudes).tolist()
        if phases is not None:
            message['phases'] = np.atleast_1d(phases).tolist()
        if spans is not None:
            message['spans'] = np.atleast_1d(spans).tolist()
        if powers_dbm is not None:
            message['powers_dbm'] = np.atleast_1d(powers_dbm).tolist()
        response = self.send_request(message)
        if response.get('status') != 'success':
            print(f"Error setting blind tones: {response.get('message')}")
            return response
        self._update_tone_defaults_from_blind_result(response['result'])
        return response

    def remove_blind_tones(self, autosync=True):
        """Remove blind tones and leave the current regular tones active."""
        response = self.send_request({
            'request': 'remove_blind_tones',
            'autosync': bool(autosync),
        })
        if response.get('status') != 'success':
            print(f"Error removing blind tones: {response.get('message')}")
            return response
        self._update_tone_defaults_from_blind_result(response['result'])
        return response

    def set_tone_amplitudes(self, tone_amplitudes, autosync=True):
        """Set the per-tone amplitude scale factors from ``tone_amplitudes``."""
        tone_amplitudes = np.atleast_1d(tone_amplitudes).tolist()
        return self.set_parameter(
            'tone_amplitudes', tone_amplitudes, autosync=bool(autosync))

    def get_tone_amplitudes(self):
        """Return per-tone amplitude scale factors."""
        return np.atleast_1d(self.get_parameter('tone_amplitudes'))

    def set_tone_phases(self, tone_phases, autosync=True):
        """Set the per-tone phase offsets (radians) from ``tone_phases``."""
        tone_phases = np.atleast_1d(tone_phases).tolist()
        return self.set_parameter(
            'tone_phases', tone_phases, autosync=bool(autosync))

    def get_tone_phases(self):
        """Return per-tone phase offsets in radians."""
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
        """Set the sync delay to integer ``value``."""
        return self.set_parameter('sync_delay', int(value))

    def get_sync_delay(self):
        """Get the current sync delay."""
        return self.get_parameter('sync_delay')

    def set_acc_len(self, value):
        """Set the accumulation length to integer ``value``."""
        return self.set_parameter('acc_len', int(value))

    def get_acc_len(self):
        """Get the current accumulation length."""
        return self.get_parameter('acc_len')

    def set_internal_loopback(self, enabled):
        """Enable (``enabled=True``) or disable (``False``) the internal loopback."""
        return self.set_parameter('internal_loopback', bool(enabled))

    def get_internal_loopback(self):
        """Get the internal loopback state."""
        return self.get_parameter('internal_loopback')

    def set_psb_scale(self, value):
        """Set the PSB scale factor to integer ``value``."""
        return self.set_parameter('psb_scale', int(value))

    def get_psb_scale(self):
        """Get the current PSB scale factor."""
        return self.get_parameter('psb_scale')

    def set_psb_fftshift(self, value):
        """Set the PSB FFT shift pattern to integer bitmask ``value``."""
        return self.set_parameter('psb_fftshift', int(value))

    def get_psb_fftshift(self):
        """Get the current PSB FFT shift pattern."""
        return self.get_parameter('psb_fftshift')

    def set_pfb_fftshift(self, value):
        """Set the PFB FFT shift pattern to integer bitmask ``value``."""
        return self.set_parameter('pfb_fftshift', int(value))

    def get_pfb_fftshift(self):
        """Get the current PFB FFT shift pattern."""
        return self.get_parameter('pfb_fftshift')

    @staticmethod
    def _add_power_force_controls(
            message, force_tx_amp_bypass=None, force_rx_amp_bypass=None,
            force_tx_attenuation_db=None, force_rx_attenuation_db=None,
            force_adc_dsa_db=None, force_tone_amplitudes=None,
            force_tone_amplitude=None, force_psb_fftshift=None,
            force_psb_shift=None, force_psb_scale=None,
            force_pfb_fftshift=None, force_pfb_shift=None):
        """Add fixed-value power optimisation controls to a request message."""
        if force_tone_amplitudes is not None and force_tone_amplitude is not None:
            raise ValueError(
                'Specify only one of force_tone_amplitudes or force_tone_amplitude')
        if force_psb_fftshift is not None and force_psb_shift is not None:
            raise ValueError(
                'Specify only one of force_psb_fftshift or force_psb_shift')
        if force_pfb_fftshift is not None and force_pfb_shift is not None:
            raise ValueError(
                'Specify only one of force_pfb_fftshift or force_pfb_shift')

        if force_tone_amplitudes is None:
            force_tone_amplitudes = force_tone_amplitude
        if force_psb_fftshift is None:
            force_psb_fftshift = force_psb_shift
        if force_pfb_fftshift is None:
            force_pfb_fftshift = force_pfb_shift

        controls = {
            'force_tx_amp_bypass': force_tx_amp_bypass,
            'force_rx_amp_bypass': force_rx_amp_bypass,
            'force_tx_attenuation_db': force_tx_attenuation_db,
            'force_rx_attenuation_db': force_rx_attenuation_db,
            'force_adc_dsa_db': force_adc_dsa_db,
            'force_tone_amplitudes': force_tone_amplitudes,
            'force_psb_fftshift': force_psb_fftshift,
            'force_psb_scale': force_psb_scale,
            'force_pfb_fftshift': force_pfb_fftshift,
        }
        for key, value in controls.items():
            if value is None:
                continue
            if key in ('force_tx_amp_bypass', 'force_rx_amp_bypass'):
                message[key] = bool(value)
            elif key == 'force_tone_amplitudes':
                message[key] = np.atleast_1d(value).astype(float).tolist()
            elif key in ('force_psb_fftshift', 'force_pfb_fftshift'):
                message[key] = int(value)
            else:
                message[key] = float(value)
        return message

    def set_tone_powers(self, tone_powers_dbm, reference_plane='detector',
                        optimise_dynamic_range=True, rx_policy='protect',
                        verbose=True, autosync=True, *,
                        force_tx_amp_bypass=None,
                        force_rx_amp_bypass=None,
                        force_tx_attenuation_db=None,
                        force_rx_attenuation_db=None,
                        force_adc_dsa_db=None, force_tone_amplitudes=None,
                        force_tone_amplitude=None, force_psb_fftshift=None,
                        force_psb_shift=None, force_psb_scale=None,
                        force_pfb_fftshift=None, force_pfb_shift=None):
        """Set tone powers to specified levels in dBm.

        Parameters
        ----------
        tone_powers_dbm : array-like
            Target power per tone in dBm.
        reference_plane : str
            Where the target power is specified: 'dac', 'rf_output', or
            'detector' (default).
        optimise_dynamic_range : bool
            If True (default), maximise DAC bit utilisation, adjust available
            TX RF controls (programmable attenuator and amp bypass), and
            reduce PSB scale if those controls cannot absorb enough excess
            power.  Set to False to skip optimisation for speed.
        rx_policy : str
            How to manage the RX path when the TX power change risks
            saturating the ADC.  One of:

            - ``'protect'`` (default) — if ADC saturates, increase RX
              attenuation or DSA to clear it and warn.
            - ``'compensate'`` — mirror TX power changes onto the RX
              path to keep round-trip power constant.
            - ``'maximise'`` — run ``maximise_rx_power()`` after the TX
              change to optimise RX attenuation, DSA, RX amp, and PFB
              FFT shift.
            - ``'raise'`` — raise error if ADC saturates.
            - ``'none'`` — don't touch the RX path.
        verbose : bool
            If False, suppress client-side informational summaries.  Warnings
            and errors are still printed.
        autosync : bool
            If True (default), trigger firmware sync after tone-amplitude writes.
        force_tx_amp_bypass, force_rx_amp_bypass : bool or None, optional
            Pin the TX/RX amplifier bypass state instead of letting the
            optimiser choose it.
        force_tx_attenuation_db, force_rx_attenuation_db : float or None, optional
            Pin the TX/RX programmable-attenuator values (dB).
        force_adc_dsa_db : float or None, optional
            Pin the ADC digital step-attenuator value (dB).
        force_tone_amplitudes : array-like or None, optional
            Pin the per-tone digital amplitudes.
        force_tone_amplitude : float or None, optional
            Pin a single amplitude for all tones (mutually exclusive with
            ``force_tone_amplitudes``).
        force_psb_fftshift : int or None, optional
            Pin the PSB FFT-shift schedule (``force_psb_shift`` is an accepted
            alias).
        force_psb_shift : int or None, optional
            Alias for ``force_psb_fftshift``.
        force_psb_scale : float or None, optional
            Pin the PSB output scale.
        force_pfb_fftshift : int or None, optional
            Pin the PFB FFT-shift schedule (``force_pfb_shift`` is an accepted
            alias).
        force_pfb_shift : int or None, optional
            Alias for ``force_pfb_fftshift``.

        Notes
        -----
        The ``force_*`` overrides pin individual power-chain controls instead
        of letting ``optimise_dynamic_range`` choose them; any left ``None``
        are optimised as usual.  The same set is accepted by
        :py:meth:`maximise_tx_power`, :py:meth:`maximise_rx_power`,
        :py:meth:`optimise_tx_snr`, and :py:meth:`optimise_rx_snr`.
        """
        tone_powers_dbm = np.atleast_1d(tone_powers_dbm).tolist()
        message = {'request': 'set', 'param': 'tone_powers',
                   'value': tone_powers_dbm,
                   'reference_plane': reference_plane,
                   'optimise_dynamic_range': optimise_dynamic_range,
                   'rx_policy': rx_policy,
                   'autosync': bool(autosync)}
        self._add_power_force_controls(
            message,
            force_tx_amp_bypass=force_tx_amp_bypass,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_tx_attenuation_db=force_tx_attenuation_db,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_tone_amplitudes=force_tone_amplitudes,
            force_tone_amplitude=force_tone_amplitude,
            force_psb_fftshift=force_psb_fftshift,
            force_psb_shift=force_psb_shift,
            force_psb_scale=force_psb_scale,
            force_pfb_fftshift=force_pfb_fftshift,
            force_pfb_shift=force_pfb_shift)
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
                    elif verbose:
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
            RX chain, modelled forward from the configured TX endpoint:
                'cryostat_output'  - cryostat output (before RX frontend)
                'adc_input'        - ADC input (after RX frontend)
                'accumulator'      - modelled accumulated IQ magnitude in dB

            RX reference planes are predictions from current TX settings and
            calibration, useful for configured loopback or known-through paths.
            If a detector/resonator is present, include its per-tone S21 in the
            configured model or apply it separately. This method does not
            acquire accumulator samples.

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
        """Check whether the ADC/input path is saturating.

        ``iterations`` is the number of samples checked (default 250).
        """
        message = {'request': 'check_input_saturation','iterations':iterations}
        return self.send_request(message)

    def check_output_saturation(self,iterations=250):
        """Check whether the DAC/output path is saturating.

        ``iterations`` is the number of samples checked (default 250).
        """
        message = {'request': 'check_output_saturation','iterations':iterations}
        return self.send_request(message)

    def check_dsp_overflow(self,duration_s=0.2):
        """Check whether DSP overflow flags occur during a short interval.

        ``duration_s`` is how long to watch the overflow flags (seconds).
        """
        message = {'request': 'check_dsp_overflow','duration_s':duration_s}
        return self.send_request(message)
    
    # TODO: Add option to save the resulting parameters to the config file after
    #       maximise/optimise/fix operations (requires save_config, see push_config TODO).
    def maximise_tx_power(self, headroom_db=2.0, reference_plane='dac',
                          power_limit_dbm=None, compression_headroom_db=None,
                          rx_policy='protect', digital_only=False, rf_only=False,
                          *, force_tx_amp_bypass=None,
                          force_rx_amp_bypass=None,
                          force_tx_attenuation_db=None,
                          force_rx_attenuation_db=None,
                          force_adc_dsa_db=None, force_tone_amplitudes=None,
                          force_tone_amplitude=None, force_psb_fftshift=None,
                          force_psb_shift=None, force_psb_scale=None,
                          force_pfb_fftshift=None, force_pfb_shift=None):
        """Maximise TX output power at the chosen reference plane.

        Parameters
        ----------
        headroom_db : float
            Safety margin below DAC saturation (default 2.0 dB).
        reference_plane : str
            'dac' (default), 'rf_output', or 'detector'.
        power_limit_dbm : float or None
            Maximum allowed tone power in dBm at the reference plane.
        compression_headroom_db : float or None
            Optional margin below the RF frontend TX input 1 dB compression
            point.  For example, pass 10.0 to keep total TX input power
            at least 10 dB below the modelled P1dB.  Disabled by default.
        rx_policy : str
            How to manage the RX path when TX power increases risk
            saturating the ADC.  One of:

            - ``'protect'`` (default) — if ADC saturates, increase RX
              attenuation or DSA to clear it and warn.
            - ``'compensate'`` — mirror TX power changes onto the RX
              path to keep round-trip power constant.
            - ``'maximise'`` — run ``maximise_rx_power()`` after each TX
              power change to optimise RX attenuation, DSA, RX amp, and
              PFB FFT shift.
            - ``'raise'`` — raise error if ADC saturates.
            - ``'none'`` — don't touch the RX path.
        digital_only : bool
            If True, only firmware/RFDC parameters are adjusted.
        rf_only : bool
            If True, only RF frontend attenuators and bypass amps are adjusted.
        force_tx_amp_bypass, force_rx_amp_bypass, force_tx_attenuation_db, force_rx_attenuation_db, force_adc_dsa_db, force_tone_amplitudes, force_tone_amplitude, force_psb_fftshift, force_psb_shift, force_psb_scale, force_pfb_fftshift, force_pfb_shift : optional
            Pin individual power-chain controls instead of optimising them; see
            :py:meth:`set_tone_powers` for the full description of each.
        """
        msg = {'request': 'maximise_tx_power', 'headroom_db': headroom_db,
               'reference_plane': reference_plane, 'rx_policy': rx_policy,
               'digital_only': digital_only, 'rf_only': rf_only}
        self._add_power_force_controls(
            msg,
            force_tx_amp_bypass=force_tx_amp_bypass,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_tx_attenuation_db=force_tx_attenuation_db,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_tone_amplitudes=force_tone_amplitudes,
            force_tone_amplitude=force_tone_amplitude,
            force_psb_fftshift=force_psb_fftshift,
            force_psb_shift=force_psb_shift,
            force_psb_scale=force_psb_scale,
            force_pfb_fftshift=force_pfb_fftshift,
            force_pfb_shift=force_pfb_shift)
        if power_limit_dbm is not None:
            msg['power_limit_dbm'] = power_limit_dbm
        if compression_headroom_db is not None:
            msg['compression_headroom_db'] = compression_headroom_db
        return self.send_request(msg)

    def maximise_rx_power(self, headroom_db=1.0, digital_only=False, rf_only=False,
                          *, force_tx_amp_bypass=None,
                          force_rx_amp_bypass=None,
                          force_tx_attenuation_db=None,
                          force_rx_attenuation_db=None,
                          force_adc_dsa_db=None, force_tone_amplitudes=None,
                          force_tone_amplitude=None, force_psb_fftshift=None,
                          force_psb_shift=None, force_psb_scale=None,
                          force_pfb_fftshift=None, force_pfb_shift=None):
        """Maximise RX chain power while retaining the requested headroom.

        Parameters
        ----------
        headroom_db : float, optional
            Safety margin below ADC saturation to retain (default 1.0 dB).
        digital_only : bool, optional
            If True, only firmware/RFDC parameters are adjusted.
        rf_only : bool, optional
            If True, only RF frontend attenuators and bypass amps are adjusted.
        force_tx_amp_bypass, force_rx_amp_bypass, force_tx_attenuation_db, force_rx_attenuation_db, force_adc_dsa_db, force_tone_amplitudes, force_tone_amplitude, force_psb_fftshift, force_psb_shift, force_psb_scale, force_pfb_fftshift, force_pfb_shift : optional
            Pin individual power-chain controls instead of optimising them; see
            :py:meth:`set_tone_powers` for the full description of each.
        """
        msg = {'request': 'maximise_rx_power',
               'headroom_db': headroom_db,
               'digital_only': digital_only,
               'rf_only': rf_only}
        self._add_power_force_controls(
            msg,
            force_tx_amp_bypass=force_tx_amp_bypass,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_tx_attenuation_db=force_tx_attenuation_db,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_tone_amplitudes=force_tone_amplitudes,
            force_tone_amplitude=force_tone_amplitude,
            force_psb_fftshift=force_psb_fftshift,
            force_psb_shift=force_psb_shift,
            force_psb_scale=force_psb_scale,
            force_pfb_fftshift=force_pfb_fftshift,
            force_pfb_shift=force_pfb_shift)
        return self.send_request(msg)

    def optimise_tx_snr(self, reference_plane='detector', headroom_db=2.0,
                        digital_only=False, rf_only=False, *,
                        force_tx_amp_bypass=None,
                        force_rx_amp_bypass=None,
                        force_tx_attenuation_db=None,
                        force_rx_attenuation_db=None,
                        force_adc_dsa_db=None, force_tone_amplitudes=None,
                        force_tone_amplitude=None, force_psb_fftshift=None,
                        force_psb_shift=None, force_psb_scale=None,
                        force_pfb_fftshift=None, force_pfb_shift=None):
        """Optimise TX settings for SNR at the requested reference plane.

        Parameters
        ----------
        reference_plane : str, optional
            Plane the optimisation targets: 'dac', 'rf_output', or 'detector'
            (default).
        headroom_db : float, optional
            Safety margin below DAC saturation to retain (default 2.0 dB).
        digital_only : bool, optional
            If True, only firmware/RFDC parameters are adjusted.
        rf_only : bool, optional
            If True, only RF frontend attenuators and bypass amps are adjusted.
        force_tx_amp_bypass, force_rx_amp_bypass, force_tx_attenuation_db, force_rx_attenuation_db, force_adc_dsa_db, force_tone_amplitudes, force_tone_amplitude, force_psb_fftshift, force_psb_shift, force_psb_scale, force_pfb_fftshift, force_pfb_shift : optional
            Pin individual power-chain controls instead of optimising them; see
            :py:meth:`set_tone_powers` for the full description of each.
        """
        msg = {'request': 'optimise_tx_snr',
               'reference_plane': reference_plane,
               'headroom_db': headroom_db,
               'digital_only': digital_only,
               'rf_only': rf_only}
        self._add_power_force_controls(
            msg,
            force_tx_amp_bypass=force_tx_amp_bypass,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_tx_attenuation_db=force_tx_attenuation_db,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_tone_amplitudes=force_tone_amplitudes,
            force_tone_amplitude=force_tone_amplitude,
            force_psb_fftshift=force_psb_fftshift,
            force_psb_shift=force_psb_shift,
            force_psb_scale=force_psb_scale,
            force_pfb_fftshift=force_pfb_fftshift,
            force_pfb_shift=force_pfb_shift)
        return self.send_request(msg)

    def optimise_rx_snr(self, digital_only=False, rf_only=False, *,
                        force_tx_amp_bypass=None,
                        force_rx_amp_bypass=None,
                        force_tx_attenuation_db=None,
                        force_rx_attenuation_db=None,
                        force_adc_dsa_db=None, force_tone_amplitudes=None,
                        force_tone_amplitude=None, force_psb_fftshift=None,
                        force_psb_shift=None, force_psb_scale=None,
                        force_pfb_fftshift=None, force_pfb_shift=None):
        """Optimise RX settings for SNR.

        Parameters
        ----------
        digital_only : bool, optional
            If True, only firmware/RFDC parameters are adjusted.
        rf_only : bool, optional
            If True, only RF frontend attenuators and bypass amps are adjusted.
        force_tx_amp_bypass, force_rx_amp_bypass, force_tx_attenuation_db, force_rx_attenuation_db, force_adc_dsa_db, force_tone_amplitudes, force_tone_amplitude, force_psb_fftshift, force_psb_shift, force_psb_scale, force_pfb_fftshift, force_pfb_shift : optional
            Pin individual power-chain controls instead of optimising them; see
            :py:meth:`set_tone_powers` for the full description of each.
        """
        msg = {'request': 'optimise_rx_snr',
               'digital_only': digital_only,
               'rf_only': rf_only}
        self._add_power_force_controls(
            msg,
            force_tx_amp_bypass=force_tx_amp_bypass,
            force_rx_amp_bypass=force_rx_amp_bypass,
            force_tx_attenuation_db=force_tx_attenuation_db,
            force_rx_attenuation_db=force_rx_attenuation_db,
            force_adc_dsa_db=force_adc_dsa_db,
            force_tone_amplitudes=force_tone_amplitudes,
            force_tone_amplitude=force_tone_amplitude,
            force_psb_fftshift=force_psb_fftshift,
            force_psb_shift=force_psb_shift,
            force_psb_scale=force_psb_scale,
            force_pfb_fftshift=force_pfb_fftshift,
            force_pfb_shift=force_pfb_shift)
        return self.send_request(msg)

    def fix_dac_saturation(self):
        """Ask the server to reduce or reconfigure output drive to clear DAC saturation."""
        return self.send_request({'request': 'fix_dac_saturation'})
    
    def fix_adc_saturation(self):
        """Ask the server to reduce or reconfigure input gain to clear ADC saturation."""
        return self.send_request({'request': 'fix_adc_saturation'})

    def fix_dsp_overflow(self, duration_s=0.5, max_iterations=10):
        """Ask the server to adjust DSP settings until overflow clears.

        ``duration_s`` is the overflow-watch interval per attempt (seconds);
        ``max_iterations`` caps the number of adjustment attempts.
        """
        return self.send_request({'request': 'fix_dsp_overflow',
                                  'duration_s': duration_s,
                                  'max_iterations': max_iterations})

    # -- RF peripheral (attenuator / amp bypass) control --

    def get_rf_peripheral_status(self):
        """Return status for configured RF attenuator and amplifier-bypass hardware."""
        return self.send_request({'request': 'get_rf_peripheral_status'})

    def set_tx_attenuation(self, value_db):
        """Set the TX programmable attenuation to ``value_db`` (dB)."""
        return self.send_request({'request': 'set_tx_attenuation', 'value': float(value_db)})

    def get_tx_attenuation(self):
        """Return current TX attenuation in dB."""
        return self.send_request({'request': 'get_tx_attenuation'})

    def set_rx_attenuation(self, value_db):
        """Set the RX programmable attenuation to ``value_db`` (dB)."""
        return self.send_request({'request': 'set_rx_attenuation', 'value': float(value_db)})

    def get_rx_attenuation(self):
        """Return current RX attenuation in dB."""
        return self.send_request({'request': 'get_rx_attenuation'})

    def set_tx_amp_bypass(self, bypass=True):
        """Enable or disable the TX amplifier bypass path."""
        return self.send_request({'request': 'set_tx_amp_bypass', 'bypass': bool(bypass)})

    def get_tx_amp_bypass(self):
        """Return whether the TX amplifier bypass path is enabled."""
        return self.send_request({'request': 'get_tx_amp_bypass'})

    def set_rx_amp_bypass(self, bypass=True):
        """Enable or disable the RX amplifier bypass path."""
        return self.send_request({'request': 'set_rx_amp_bypass', 'bypass': bool(bypass)})

    def get_rx_amp_bypass(self):
        """Return whether the RX amplifier bypass path is enabled."""
        return self.send_request({'request': 'get_rx_amp_bypass'})

    # --- LNA bias control ---

    def get_lna_controller_status(self):
        """Get LNA bias controller status and configured soft-off state."""
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
            blind: If True and method='remote', skip the LNA voltage sanity
                check (``v_remote > v_lna > 0``).  Use this if the iterative
                feedback algorithm rejects valid setpoints because the
                downstream LNA is not yet powered or drawing current.

        Returns:
            On success: ``{'status': 'success', 'result': {...}}`` where
            ``result`` includes ``channel``, ``voltage_v`` (achieved), ``method``,
            ``message``, and ``success: True``.

            On failure (remote-feedback rejection): ``{'status': 'error',
            'message': ..., 'result': {..., 'success': False}}``.
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

    def soft_off_lna_bias(self, channel=None):
        """Drive one LNA bias channel to its minimum local voltage.

        This is a soft off only: the LNA rail is not fully disabled because
        the bias board does not expose a software shutdown pin.

        Args:
            channel: LNA channel index (1-14). Defaults to this pipeline's
                configured channel.

        Returns:
            ``{'status': 'success', 'result': {...}}`` on success. The result
            includes ``soft_off: True`` and a message noting that this is not a
            hard power cut.
        """
        msg = {'request': 'soft_off_lna_bias'}
        if channel is not None:
            msg['channel'] = int(channel)
        return self.send_request(msg)

    def set_lna_bias_voltage_all(self, voltage_v, method='remote', blind=False):
        """Set LNA bias voltage for all 14 channels.

        Args:
            voltage_v: Target voltage in volts.
            method: 'remote' (default) or 'local'.
            blind: If True and method='remote', skip the LNA voltage sanity
                check (``v_remote > v_lna > 0``).

        Returns:
            On full success: ``{'status': 'success', 'result': {chn: {...}, ...}}``.

            If any channel fails: ``{'status': 'error', 'message': ...,
            'result': {chn: {..., 'success': bool}, ...}}``.  Channels that
            did succeed still have their per-channel results populated.
        """
        return self.send_request({
            'request': 'set_lna_bias_voltage_all',
            'voltage_v': float(voltage_v),
            'method': method,
            'blind': blind,
        })

    def soft_off_lna_bias_all(self):
        """Drive all LNA bias channels to their minimum local voltage.

        Returns:
            ``{'status': 'success', 'result': {chn: {...}, ...}}`` if every
            channel succeeds. If unconfigured channels are present the response
            is ``status: error`` with per-channel results.
        """
        return self.send_request({'request': 'soft_off_lna_bias_all'})

    def enable_stream(self):
        """Enable continuous sample streaming on the server."""
        self._warn_zero_phases()
        message = {'request': 'enable_stream'}
        return self.send_request(message)

    def disable_stream(self):
        """Disable continuous sample streaming on the server."""
        message = {'request': 'disable_stream'}
        return self.send_request(message)

    def enable_modulation(self, center=None, offsets=None, mod_indices=None,
                          samples_per_point=1, n_settle=1, autosync=True,
                          setup_sync=True):
        """
        Arm fast tone-frequency modulation. This only **arms** (loads the config
        on the server); it does not start output. Call :meth:`enable_stream` for
        continuous modulated streaming, or :meth:`get_samples` for a finite
        modulated capture — both return frames tagged with the active point.

        Parameters
        ----------
        center : array-like or None
            Per-tone centre RF frequencies in Hz (user-facing tone order).
            ``None`` uses the current live comb. With no args at all (and a
            previously-armed config) this re-arms that config.
        offsets : array-like or None
            Probe offsets in Hz: shape ``(n_points,)`` (broadcast across the
            modulated tones) or ``(n_points, len(mod_indices))`` (per tone).
        mod_indices : array-like or None
            User-facing indices of tones to modulate. ``None`` = all regular
            (resonator) tones. Blind tones are rejected by the server.
        samples_per_point : int, optional
            Dwell: accumulations per point per cycle (default 1).
        n_settle : int, optional
            Leading samples per point flagged as settling (default 1).
        autosync : bool, optional
            Whether each modulation buffer flip should pulse firmware sync
            (default ``True``). Set ``False`` to test unsynced buffer flips.
        setup_sync : bool, optional
            Whether to pulse one firmware sync at arm time to establish the
            TX/RX phase reference before any buffer flips (default ``True``).
            Independent of ``autosync``: with ``setup_sync=True, autosync=False``
            the LO is aligned once and then rides continuous accumulation across
            buffer flips with no per-step sync.

        Returns
        -------
        dict
            Server ack, including ``result.revision`` and
            ``result.needs_recenter``.
        """
        self._warn_zero_phases()
        message = {'request': 'enable_modulation',
                   'samples_per_point': int(samples_per_point),
                   'n_settle': int(n_settle),
                   'autosync': bool(autosync),
                   'setup_sync': bool(setup_sync)}
        if center is not None:
            message['center'] = np.asarray(center, dtype=float).tolist()
        if offsets is not None:
            message['offsets'] = np.asarray(offsets, dtype=float).tolist()
        if mod_indices is not None:
            message['mod_indices'] = [int(i) for i in np.atleast_1d(mod_indices)]
        return self.send_request(message)

    def update_modulation(self, center=None, offsets=None, on_map_change='continue',
                          autosync=None):
        """
        Seamlessly update the modulation centre and/or offsets while armed, with
        no dropped frames. Rides the existing armed channel maps.

        Parameters
        ----------
        center : array-like or None
            New per-tone centre RF frequencies in Hz (user order). ``None`` keeps
            the current centres.
        offsets : array-like or None
            New probe offsets in Hz (same shapes as :meth:`enable_modulation`).
            ``None`` keeps the current offsets.
        on_map_change : {'continue', 'recenter'}, optional
            What to do if the update would push a tone beyond the filterbank
            overlap coverage: ``'continue'`` (default) rejects with diagnostics
            and asks you to recenter; ``'recenter'`` performs the brief map reload.
        autosync : bool or None, optional
            Override the existing modulation sync mode. ``None`` preserves it.

        Returns
        -------
        dict
            Server ack (``result.revision`` and ``result.op``), or an error with
            ``result.tones_beyond_coverage`` if a recenter is required.
        """
        message = {'request': 'update_modulation', 'on_map_change': on_map_change}
        if center is not None:
            message['center'] = np.asarray(center, dtype=float).tolist()
        if offsets is not None:
            message['offsets'] = np.asarray(offsets, dtype=float).tolist()
        if autosync is not None:
            message['autosync'] = bool(autosync)
        return self.send_request(message)

    def recenter_modulation(self, autosync=None):
        """
        Recenter modulation: reload the channel maps / mixer frequencies for the
        current centre and recompute VACC bin-sharing (a deliberate brief break).
        Use when :meth:`update_modulation` reports tones beyond bin coverage.
        ``autosync=None`` preserves the existing sync mode; pass a bool to change it.
        """
        message = {'request': 'recenter_modulation'}
        if autosync is not None:
            message['autosync'] = bool(autosync)
        return self.send_request(message)

    def disable_modulation(self):
        """
        Pause modulation; tones rest at their centre frequencies. The armed
        config stays resident so :meth:`enable_modulation` with no args re-arms
        it quickly. Use :meth:`disable_stream` to stop output entirely.
        """
        return self.send_request({'request': 'disable_modulation'})

    def get_modulation_state(self):
        """
        Return the per-tone modulation state (the ``get_info('tone_modulation')``
        section): armed flag, desired/applied revision, per-tone centres, offsets,
        bin occupancy, and ``needs_recenter`` / ``tones_beyond_coverage``. A pure
        server-side read (no hardware access), safe to poll while streaming.
        """
        return self.get_info('tone_modulation')

    def enable_triggered_stream(self):
        """Enable triggered sample streaming on the server."""
        self._warn_zero_phases()
        message = {'request': 'enable_triggered_stream'}
        return self.send_request(message)

    def disable_triggered_stream(self):
        """Disable triggered sample streaming on the server."""
        message = {'request': 'disable_triggered_stream'}
        return self.send_request(message)

    def send_fake_trigger(self):
        """Send a software trigger for testing triggered streaming."""
        message = {'request': 'send_fake_trigger'}
        return self.send_request(message)

    def get_cal_freeze(self):
        """Return whether ADC calibration is currently frozen."""
        return self.get_parameter('cal_freeze')

    def set_cal_freeze(self,freeze):
        """Freeze (``freeze=True``) or unfreeze (``False``) the ADC background
        calibration."""
        return self.set_parameter('cal_freeze',freeze)

    def refresh_adc_cal(self, adc_cal_settle_time=2.0):
        """
        Refresh the ADC calibration by unfreezing, waiting for it to settle,
        then freezing again.

        Args:
            adc_cal_settle_time (float): Seconds to wait for calibration to
                settle after unfreezing. Default 2.0.
        """
        message = {'request': 'refresh_adc_cal', 'adc_cal_settle_time': adc_cal_settle_time}
        return self.send_request(message)

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

        Parameters
        ----------
        num_samples : int
            Number of accumulator samples to acquire.
        incl_system_info : bool, optional
            Include a system-info block in the returned data (default ``True``).
        burst : bool, optional
            Use a single burst transfer instead of streaming frames (default
            ``False``).
        """
        if self.mock:
            return self._mock_server.get_samples(num_samples, incl_system_info, burst)

        self._warn_zero_phases()
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.request_server_address, self.request_server_port))
            message = {'request': 'get_samples', 'num_samples': num_samples, 'burst': burst}
            # Send message length
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)

            def recv_exact(nbytes):
                buf = bytearray(nbytes)
                view = memoryview(buf)
                received = 0
                while received < nbytes:
                    n = s.recv_into(view[received:], nbytes - received)
                    if n == 0:
                        if received == 0:
                            return None
                        raise RuntimeError(
                            f'get_samples socket closed mid-frame '
                            f'({received}/{nbytes} bytes received)')
                    received += n
                return buf

            # Pre-allocate bytearray (conservative upper bound)
            max_frame_bytes = 2048*2*4 + 10*4
            data_raw = bytearray(max_frame_bytes*num_samples)
            view = memoryview(data_raw)

            t0=time.time()
            frame_bytes = 0
            write_offset = 0
            frames_received = 0
            for j in range(num_samples):
                # Read data length
                raw_datalen = recv_exact(4)
                if raw_datalen is None:
                    break
                frame_bytes = struct.unpack('>I', raw_datalen)[0]
                frame = recv_exact(frame_bytes)
                if frame is None:
                    break
                if write_offset + frame_bytes > len(data_raw):
                    raise RuntimeError(
                        f'get_samples frame buffer too small for frame of '
                        f'{frame_bytes} bytes')
                view[write_offset:write_offset + frame_bytes] = frame
                write_offset += frame_bytes
                frames_received += 1
            t1=time.time()
            elapsed = max(t1 - t0, 1e-12)
            print(f"Received {frames_received}/{num_samples} samples in ~{elapsed} seconds "
                  f"(~{frames_received/elapsed} samples per second)")
            if frames_received != num_samples:
                raise RuntimeError(
                    f'get_samples received {frames_received}/{num_samples} frames. '
                    'The server likely rejected or closed the capture; check the server log.')
            if incl_system_info:
                info = self.get_info('all')
            else:
                info = {}
            sample_rate = self.get_sample_rate()
            # Trim buffer to actual data received
            data_raw = data_raw[:write_offset]
            sample_data = {'data_raw':data_raw,'sample_rate':sample_rate,
                           'info':info,'frame_bytes':frame_bytes}
            return sample_data


    @staticmethod
    def parse_samples(sample_data, num_tones=None):
        """
        Parse raw sample data into per-tone I/Q arrays.

        The server sends only active tones in user order, so the data
        is already correctly ordered. Frame size is determined from
        sample_data['frame_bytes'] (set by get_samples) or derived
        from firmware_indices in the info['tones'] section.

        Falls back to 2048 channels for data from older servers that
        send all channels.

        Parameters
        ----------
        sample_data : dict
            Raw sample data returned by :py:meth:`get_samples`.
        num_tones : int or None, optional
            Override the tone count; ``None`` (default) infers it from the
            frame size / tone metadata.
        """
        data_raw = sample_data['data_raw']
        sample_rate = sample_data['sample_rate']
        info = sample_data['info']
        num_headers = 10

        # Determine frame size and num_tones
        if 'frame_bytes' in sample_data:
            frame_bytes = sample_data['frame_bytes']
            num_tones = (frame_bytes // 4 - num_headers) // 2
        elif num_tones is None:
            tone_indices = info.get('tones', {}).get('firmware_indices')
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
        # Fast-modulation tag lives in flag5 (frame[-5]); decode as UNSIGNED
        # because the revision (bits 17..31) can set the int32 sign bit:
        #   bits 0..15  = active point (0 = modulation off, 1..N = the point)
        #   bit 16      = settling/transient marker
        #   bits 17..31 = config revision
        # Plain (non-modulated) streams leave flag5 = 0, so these decode to zeros
        # and remain fully backward compatible.
        f5 = flags[:, 5].astype(np.uint32)
        data_dict = {'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
                    'num_tones':num_tones,
                    'num_samples':num_samples,
                    'sample_rate':sample_rate,
                    'info':info,
                    'i_data':{f'{i:04d}':i_data[:,i] for i in range(num_tones)},
                    'q_data':{f'{i:04d}':q_data[:,i] for i in range(num_tones)},
                    'packet_counter':cnt,
                    'packet_error':err,
                    'telescope_time':tt,
                    'stream_flags':{f'flag{i}':flags[:,i] for i in range(6)},
                    'modulation_point': (f5 & 0xFFFF).astype(int),
                    'modulation_settling': ((f5 >> 16) & 0x1).astype(int),
                    'modulation_revision': ((f5 >> 17) & 0x7FFF).astype(int),
                    }

        return data_dict

    @staticmethod
    def export_samples(filename, sample_data, num_tones_to_save=None,file_format=None):
        """Export raw or parsed sample captures to npy, json, or CSV.

        Parameters
        ----------
        filename : str
            Output path; its extension selects the format if ``file_format``
            is not given.
        sample_data : dict
            Raw (from :py:meth:`get_samples`) or already-parsed sample data.
        num_tones_to_save : int or None, optional
            Limit the number of tones written when parsing raw data; ``None``
            saves all.
        file_format : {'npy', 'json', 'csv'} or None, optional
            Output format; ``None`` (default) infers it from ``filename``.
        """
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
                ReadoutClient._write_csv_info_metadata(writer, data_dict['info'])
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
        """Import sample captures from ``filename`` (written by
        ``export_samples``); the format is inferred from its extension."""
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
                        if value.startswith('"') and value.endswith('"'):
                            value = value[1:-1]
                        try:
                            value = ast.literal_eval(value)
                        except (ValueError, SyntaxError):
                            pass
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

    def get_accumulator_snapshots(self, tone_index, num_snapshots, fast=False):
        """
        Acquire num_snapshots pre-accumulation snapshots for a single tone.

        Each snapshot contains 1024 complex samples at the FFT output rate
        (before accumulation).

        Args:
            tone_index (int): Tone index to snapshot.
            num_snapshots (int): Number of snapshots to acquire.
            fast (bool): If True, use the server's manual devmem path.
                         Default False uses the standard CASPER snapshot API.

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
                       'num_snapshots': num_snapshots,
                       'fast': bool(fast)}
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

        acc_len = self.get_info('pipeline')['acc_len']
        accumulated_rate = self.get_sample_rate()
        snapshot_rate = accumulated_rate * acc_len

        return {'snapshots': result,
                'tone_index': tone_index,
                'sample_rate': snapshot_rate,
                'num_snapshots': num_snapshots,
                'len_snapshot': result.shape[1] if result is not None else 0,
                'fast': bool(fast)}

    @staticmethod
    def _normalise_snapshot_data(snapshot_data):
        """Return accumulator snapshot data with consistent array metadata."""
        if 'snapshots' not in snapshot_data:
            raise ValueError(
                "Accumulator snapshot data must contain a 'snapshots' array.")

        data_dict = dict(snapshot_data)
        snapshots = np.asarray(data_dict['snapshots'], dtype=np.complex128)
        if snapshots.ndim == 1:
            snapshots = snapshots[np.newaxis, :]
        if snapshots.ndim != 2:
            raise ValueError(
                "snapshots must be a 2D array of shape "
                "(num_snapshots, len_snapshot).")

        data_dict['snapshots'] = snapshots
        data_dict.setdefault('num_snapshots', snapshots.shape[0])
        data_dict.setdefault('len_snapshot', snapshots.shape[1])
        return data_dict

    @staticmethod
    def export_snapshot(filename, snapshot_data, file_format=None):
        """
        Export accumulator snapshot data to npy, json, or CSV.

        This is for pre-accumulation snapshots returned by
        ``get_accumulator_snapshots()``, not ADC/DAC snapshots.

        Parameters
        ----------
        filename : str
            Output path; its extension selects the format when ``file_format``
            is not given.
        snapshot_data : dict
            Snapshot data to export.
        file_format : {'npy', 'json', 'csv'} or None, optional
            Output format; ``None`` (default) infers it from ``filename``
            (falling back to ``'npy'``).
        """
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json', 'csv'), default='npy')
        data_dict = ReadoutClient._normalise_snapshot_data(snapshot_data)

        dirpath = os.path.dirname(filepath)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath)

        if file_format == 'npy':
            np.save(filepath, data_dict)

        elif file_format == 'json':
            with open(filepath, 'w') as file:
                json.dump(ReadoutClient._json_ready(data_dict), file, indent=4)

        elif file_format == 'csv':
            snapshots = data_dict['snapshots']
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                if 'date' in data_dict:
                    writer.writerow(['# date', data_dict['date']])
                if 'tone_index' in data_dict:
                    writer.writerow(['# tone_index', data_dict['tone_index']])
                writer.writerow(['# sample_rate', data_dict.get('sample_rate', '')])
                writer.writerow(['# num_snapshots', snapshots.shape[0]])
                writer.writerow(['# len_snapshot', snapshots.shape[1]])
                if 'fast' in data_dict:
                    writer.writerow(['# fast', data_dict['fast']])
                writer.writerow(['# csv_layout', 'snapshot_columns'])
                if isinstance(data_dict.get('info'), dict):
                    ReadoutClient._write_csv_info_metadata(
                        writer, data_dict['info'])

                header = ['sample_index']
                for snap_index in range(snapshots.shape[0]):
                    header.extend([
                        f'snapshot_{snap_index:04d}_i',
                        f'snapshot_{snap_index:04d}_q',
                    ])
                writer.writerow(header)
                for sample_index in range(snapshots.shape[1]):
                    row = [sample_index]
                    for snap_index in range(snapshots.shape[0]):
                        value = snapshots[snap_index, sample_index]
                        row.extend([value.real, value.imag])
                    writer.writerow(row)

        else:
            raise ValueError(
                f"Invalid file_format {file_format}. Must be one of "
                "'npy', 'json', or 'csv'.")

    @staticmethod
    def import_snapshot(filename):
        """Import accumulator snapshot data from ``filename`` (written by
        ``export_snapshot``)."""
        if filename.endswith('.npy'):
            data_dict = np.load(filename, allow_pickle=True).item()
            return ReadoutClient._normalise_snapshot_data(data_dict)

        if filename.endswith('.json'):
            with open(filename, 'r') as file:
                data_dict = ReadoutClient._restore_json_value(json.load(file))
            return ReadoutClient._normalise_snapshot_data(data_dict)

        if filename.endswith('.csv'):
            data_dict, header_lines = ReadoutClient._read_csv_comment_metadata(
                filename)
            data = np.genfromtxt(
                filename, delimiter=',', names=True, skip_header=header_lines)
            data = np.atleast_1d(data)
            snapshot_indices = ReadoutClient._snapshot_csv_indices(
                data.dtype.names)
            snapshots = np.vstack([
                np.atleast_1d(data[f'snapshot_{index:04d}_i'])
                + 1j*np.atleast_1d(data[f'snapshot_{index:04d}_q'])
                for index in snapshot_indices
            ])
            data_dict['snapshots'] = snapshots
            data_dict.setdefault('num_snapshots', snapshots.shape[0])
            data_dict.setdefault('len_snapshot', snapshots.shape[1])
            return ReadoutClient._normalise_snapshot_data(data_dict)

        if filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        if filename.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        raise ValueError(f"Invalid file format {filename.split('.')[-1]}")

    @staticmethod
    def analyse_accumulator_snapshots(snapshot_data, threshold=6.0):
        """
        Summarise per-frame changes in pre-accumulator snapshots.

        Frames are flagged using a robust (MAD-based) z-score on each
        per-frame metric: |mean|, real/imag mean, noise RMS, I std, Q std.
        A frame is suspect if any metric's |z| exceeds ``threshold``. The
        MAD is scaled by 1.4826 so ``threshold`` is in units of equivalent
        Gaussian sigma.

        Args:
            snapshot_data: Dict returned by get_accumulator_snapshots(), or a
                           2D complex array with shape (frames, samples).
            threshold: MAD z-score above which frames are flagged. Typical
                       values are 5-6. Lower flags more frames; higher
                       flags fewer.

        Returns:
            Dict of per-frame arrays, per-metric z-scores (``*_z``), and
            ``suspect_indices``.
        """
        if isinstance(snapshot_data, dict):
            snapshots = snapshot_data.get('snapshots')
        else:
            snapshots = snapshot_data

        snapshots = np.asarray(snapshots)
        if snapshots.ndim != 2:
            raise ValueError('snapshots must be a 2D array of shape '
                             '(num_snapshots, len_snapshot)')
        if snapshots.shape[0] == 0:
            raise ValueError('snapshots array is empty')

        eps = np.finfo(float).tiny
        threshold = float(threshold)
        if threshold <= 0.0:
            raise ValueError('threshold must be positive')

        frame_mean = np.mean(snapshots, axis=1)
        centered = snapshots - frame_mean[:, None]
        noise_rms = np.sqrt(np.mean(np.abs(centered) ** 2, axis=1))
        i_std = np.std(snapshots.real, axis=1)
        q_std = np.std(snapshots.imag, axis=1)
        abs_mean = np.abs(frame_mean)

        reference_mean = (np.median(frame_mean.real)
                          + 1j * np.median(frame_mean.imag))
        reference_abs_mean = max(float(np.median(abs_mean)), eps)
        reference_noise_rms = max(float(np.median(noise_rms)), eps)
        reference_i_std = max(float(np.median(i_std)), eps)
        reference_q_std = max(float(np.median(q_std)), eps)
        reference_i_mean = float(np.median(frame_mean.real))
        reference_q_mean = float(np.median(frame_mean.imag))

        if abs(reference_mean) > eps:
            complex_mean_ratio = frame_mean / reference_mean
            phase_offset_rad = np.angle(complex_mean_ratio)
        else:
            complex_mean_ratio = np.full(frame_mean.shape, np.nan + 1j*np.nan)
            phase_offset_rad = np.full(frame_mean.shape, np.nan)

        abs_mean_ratio = abs_mean / reference_abs_mean
        noise_rms_ratio = noise_rms / reference_noise_rms
        i_std_ratio = i_std / reference_i_std
        q_std_ratio = q_std / reference_q_std
        if abs(reference_i_mean) > eps:
            i_mean_ratio = frame_mean.real / reference_i_mean
        else:
            i_mean_ratio = np.full(frame_mean.shape, np.nan)
        if abs(reference_q_mean) > eps:
            q_mean_ratio = frame_mean.imag / reference_q_mean
        else:
            q_mean_ratio = np.full(frame_mean.shape, np.nan)

        def _mad_z(values):
            """Robust z-scores via the median absolute deviation (0 if no spread)."""
            v = np.asarray(values, dtype=float)
            med = np.median(v)
            scale = 1.4826 * np.median(np.abs(v - med))
            if scale <= eps:
                return np.zeros_like(v)
            return (v - med) / scale

        abs_mean_z = _mad_z(abs_mean)
        noise_rms_z = _mad_z(noise_rms)
        i_std_z = _mad_z(i_std)
        q_std_z = _mad_z(q_std)
        i_mean_z = _mad_z(frame_mean.real)
        q_mean_z = _mad_z(frame_mean.imag)

        suspect_mask = (
            (np.abs(abs_mean_z) > threshold) |
            (np.abs(noise_rms_z) > threshold) |
            (np.abs(i_std_z) > threshold) |
            (np.abs(q_std_z) > threshold) |
            (np.abs(i_mean_z) > threshold) |
            (np.abs(q_mean_z) > threshold)
        )

        return {
            'frame_mean': frame_mean,
            'complex_mean_ratio': complex_mean_ratio,
            'abs_mean_ratio': abs_mean_ratio,
            'i_mean_ratio': i_mean_ratio,
            'q_mean_ratio': q_mean_ratio,
            'phase_offset_rad': phase_offset_rad,
            'noise_rms': noise_rms,
            'noise_rms_ratio': noise_rms_ratio,
            'i_std': i_std,
            'q_std': q_std,
            'i_std_ratio': i_std_ratio,
            'q_std_ratio': q_std_ratio,
            'abs_mean_z': abs_mean_z,
            'noise_rms_z': noise_rms_z,
            'i_std_z': i_std_z,
            'q_std_z': q_std_z,
            'i_mean_z': i_mean_z,
            'q_mean_z': q_mean_z,
            'reference_mean': reference_mean,
            'reference_abs_mean': reference_abs_mean,
            'reference_i_mean': reference_i_mean,
            'reference_q_mean': reference_q_mean,
            'reference_noise_rms': reference_noise_rms,
            'suspect_indices': np.flatnonzero(suspect_mask),
        }

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
            export_file (str): Path to save results. Defaults to .npz when
                               no recognised extension is supplied. None to skip.
            plot (bool): If True, plot time-domain and power spectrum for
                         each tone.
            verbose (bool): Print progress.

        Returns:
            dict with keys:
                'results': dict mapping tone_index -> snapshot dict (as
                           returned by get_accumulator_snapshots).
                'sample_rate': Pre-accumulator sample rate in Hz.
                'num_snapshots': Snapshots per tone.
                'len_snapshot': Samples per snapshot.
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

        acc_len = self.get_info('pipeline')['acc_len']
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
                    'num_snapshots': num_snapshots,
                    'len_snapshot': (
                        tone_data.shape[1] if tone_data is not None else 0),
                }
            t1 = time.time()
            if verbose:
                print(f"Received {total_frames} snapshots for {len(tone_indices)} tones "
                      f"in {t1-t0:.3f}s ({total_frames/(t1-t0):.1f} snapshots/s)")

        output = {
            'results': results,
            'sample_rate': snapshot_rate,
            'num_snapshots': num_snapshots,
            'tone_indices': np.asarray(tone_indices, dtype=int),
            'tone_frequencies': freqs,
            'firmware_indices': fw_indices,
        }
        output = self._normalise_batch_snapshot_data(output)

        if export_file is not None:
            self._export_batch_snapshots(output, export_file, verbose)

        if plot:
            self._plot_batch_snapshots(output)

        return output

    def batch_snapshot(self, *args, **kwargs):
        """Alias for :py:meth:`batch_snapshots` using singular naming.

        ``*args`` and ``**kwargs`` are forwarded unchanged; see
        :py:meth:`batch_snapshots` for the accepted parameters.
        """
        return self.batch_snapshots(*args, **kwargs)

    @staticmethod
    def _normalise_batch_snapshot_data(batch_data):
        """Return batch accumulator snapshot data with consistent metadata."""
        if 'results' not in batch_data:
            raise ValueError("Batch snapshot data must contain 'results'.")

        data_dict = dict(batch_data)
        results = {}
        for tone_index, snap_data in data_dict['results'].items():
            tone_index = int(tone_index)
            snap_dict = dict(snap_data)
            snap_dict.setdefault('tone_index', tone_index)
            if 'sample_rate' not in snap_dict and 'sample_rate' in data_dict:
                snap_dict['sample_rate'] = data_dict['sample_rate']
            results[tone_index] = ReadoutClient._normalise_snapshot_data(
                snap_dict)

        tone_indices = np.asarray(list(results.keys()), dtype=int)
        data_dict['results'] = results
        data_dict['tone_indices'] = np.asarray(
            data_dict.get('tone_indices', tone_indices), dtype=int)
        data_dict['num_snapshots'] = int(data_dict.get(
            'num_snapshots',
            next(iter(results.values()))['num_snapshots']
            if results else 0))
        if 'len_snapshot' not in data_dict:
            if results:
                len_snapshots = np.asarray(
                    [snap['len_snapshot'] for snap in results.values()],
                    dtype=int)
                if np.all(len_snapshots == len_snapshots[0]):
                    data_dict['len_snapshot'] = int(len_snapshots[0])
                else:
                    data_dict['len_snapshot'] = len_snapshots
            else:
                data_dict['len_snapshot'] = 0
        if 'sample_rate' not in data_dict and results:
            data_dict['sample_rate'] = next(iter(results.values())).get(
                'sample_rate', 0.0)
        data_dict['tone_frequencies'] = np.asarray(
            data_dict.get('tone_frequencies', []))
        data_dict['firmware_indices'] = np.asarray(
            data_dict.get('firmware_indices', []))
        return data_dict

    @staticmethod
    def export_batch_snapshots(filename, batch_data, file_format=None):
        """
        Export batch accumulator snapshot data to npz, npy, json, or CSV.

        ``npz`` is the default and matches the original batch snapshot save
        layout used by ``batch_snapshots(export_file=...)``.

        Parameters
        ----------
        filename : str
            Output path; its extension selects the format when ``file_format``
            is not given.
        batch_data : dict
            Batch snapshot data from :py:meth:`batch_snapshots`.
        file_format : {'npz', 'npy', 'json', 'csv'} or None, optional
            Output format; ``None`` (default) infers it from ``filename``
            (falling back to ``'npz'``).
        """
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npz', 'npy', 'json', 'csv'), default='npz')
        data_dict = ReadoutClient._normalise_batch_snapshot_data(batch_data)

        dirpath = os.path.dirname(filepath)
        if dirpath and not os.path.exists(dirpath):
            os.makedirs(dirpath)

        if file_format == 'npz':
            save_dict = {
                'sample_rate': data_dict['sample_rate'],
                'num_snapshots': data_dict['num_snapshots'],
                'len_snapshot': data_dict['len_snapshot'],
                'tone_frequencies': data_dict['tone_frequencies'],
                'tone_indices': data_dict['tone_indices'],
                'firmware_indices': data_dict['firmware_indices'],
            }
            for tone_index, snap in data_dict['results'].items():
                save_dict[f'snapshots_tone_{tone_index}'] = snap['snapshots']
            np.savez(filepath, **save_dict)

        elif file_format == 'npy':
            np.save(filepath, data_dict)

        elif file_format == 'json':
            with open(filepath, 'w') as file:
                json.dump(ReadoutClient._json_ready(data_dict), file, indent=4)

        elif file_format == 'csv':
            tone_indices = np.asarray(data_dict['tone_indices'], dtype=int)
            num_snapshots = int(data_dict['num_snapshots'])
            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                if 'date' in data_dict:
                    writer.writerow(['# date', data_dict['date']])
                writer.writerow(['# sample_rate', data_dict.get('sample_rate', '')])
                writer.writerow(['# num_snapshots', num_snapshots])
                len_snapshot = data_dict.get('len_snapshot', '')
                if isinstance(len_snapshot, np.ndarray):
                    len_snapshot = len_snapshot.tolist()
                writer.writerow(['# len_snapshot', len_snapshot])
                writer.writerow(['# tone_indices', tone_indices.tolist()])
                writer.writerow([
                    '# tone_frequencies',
                    np.asarray(data_dict['tone_frequencies']).tolist(),
                ])
                writer.writerow([
                    '# firmware_indices',
                    np.asarray(data_dict['firmware_indices']).tolist(),
                ])
                writer.writerow(['# csv_layout', 'batch_snapshot_columns'])

                header = ['tone_index', 'sample_index']
                for snap_index in range(num_snapshots):
                    header.extend([
                        f'snapshot_{snap_index:04d}_i',
                        f'snapshot_{snap_index:04d}_q',
                    ])
                writer.writerow(header)

                for tone_index in tone_indices:
                    snapshots = data_dict['results'][int(tone_index)]['snapshots']
                    if snapshots.shape[0] != num_snapshots:
                        raise ValueError(
                            "All batch snapshot entries must have "
                            "num_snapshots rows for CSV export.")
                    for sample_index in range(snapshots.shape[1]):
                        row = [int(tone_index), sample_index]
                        for snap_index in range(num_snapshots):
                            value = snapshots[snap_index, sample_index]
                            row.extend([value.real, value.imag])
                        writer.writerow(row)

        else:
            raise ValueError(
                f"Invalid file_format {file_format}. Must be one of "
                "'npz', 'npy', 'json', or 'csv'.")

    @staticmethod
    def import_batch_snapshots(filename):
        """Import batch accumulator snapshots from ``filename`` (written by
        ``export_batch_snapshots``)."""
        if filename.endswith('.npz'):
            with np.load(filename, allow_pickle=True) as npz_data:
                sample_rate = np.asarray(npz_data['sample_rate']).item()
                num_snapshots = int(np.asarray(
                    npz_data['num_snapshots']).item())
                len_snapshot = (
                    np.asarray(npz_data['len_snapshot'])
                    if 'len_snapshot' in npz_data.files else None)
                tone_indices = np.asarray(npz_data['tone_indices'], dtype=int)
                tone_frequencies = np.asarray(
                    npz_data['tone_frequencies']
                    if 'tone_frequencies' in npz_data.files else [])
                firmware_indices = np.asarray(
                    npz_data['firmware_indices']
                    if 'firmware_indices' in npz_data.files else [])
                results = {}
                for tone_index in tone_indices:
                    tone_index = int(tone_index)
                    snapshots = np.asarray(
                        npz_data[f'snapshots_tone_{tone_index}'],
                        dtype=np.complex128)
                    results[tone_index] = {
                        'snapshots': snapshots,
                        'tone_index': tone_index,
                        'sample_rate': sample_rate,
                        'num_snapshots': num_snapshots,
                        'len_snapshot': snapshots.shape[1],
                    }
            data_dict = {
                'results': results,
                'sample_rate': sample_rate,
                'num_snapshots': num_snapshots,
                'tone_indices': tone_indices,
                'tone_frequencies': tone_frequencies,
                'firmware_indices': firmware_indices,
            }
            if len_snapshot is not None:
                data_dict['len_snapshot'] = (
                    int(len_snapshot.item())
                    if len_snapshot.size == 1 else len_snapshot.astype(int))
            return ReadoutClient._normalise_batch_snapshot_data(data_dict)

        if filename.endswith('.npy'):
            data_dict = np.load(filename, allow_pickle=True).item()
            return ReadoutClient._normalise_batch_snapshot_data(data_dict)

        if filename.endswith('.json'):
            with open(filename, 'r') as file:
                data_dict = ReadoutClient._restore_json_value(json.load(file))
            return ReadoutClient._normalise_batch_snapshot_data(data_dict)

        if filename.endswith('.csv'):
            data_dict, header_lines = ReadoutClient._read_csv_comment_metadata(
                filename)
            data = np.genfromtxt(
                filename, delimiter=',', names=True, skip_header=header_lines)
            data = np.atleast_1d(data)
            snapshot_indices = ReadoutClient._snapshot_csv_indices(
                data.dtype.names)
            if 'tone_indices' in data_dict:
                tone_indices = np.asarray(data_dict['tone_indices'], dtype=int)
            else:
                tone_indices = np.unique(data['tone_index'].astype(int))

            results = {}
            for tone_index in tone_indices:
                mask = data['tone_index'].astype(int) == int(tone_index)
                tone_rows = np.atleast_1d(data[mask])
                order = np.argsort(tone_rows['sample_index'])
                tone_rows = tone_rows[order]
                snapshots = np.vstack([
                    np.atleast_1d(tone_rows[f'snapshot_{index:04d}_i'])
                    + 1j*np.atleast_1d(
                        tone_rows[f'snapshot_{index:04d}_q'])
                    for index in snapshot_indices
                ])
                results[int(tone_index)] = {
                    'snapshots': snapshots,
                    'tone_index': int(tone_index),
                    'sample_rate': data_dict.get('sample_rate', 0.0),
                    'num_snapshots': snapshots.shape[0],
                    'len_snapshot': snapshots.shape[1],
                }

            data_dict['results'] = results
            data_dict['tone_indices'] = tone_indices
            data_dict.setdefault('num_snapshots', len(snapshot_indices))
            data_dict.setdefault('tone_frequencies', [])
            data_dict.setdefault('firmware_indices', [])
            return ReadoutClient._normalise_batch_snapshot_data(data_dict)

        if filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        if filename.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        raise ValueError(f"Invalid file format {filename.split('.')[-1]}")

    import_batch_snapshot = import_batch_snapshots
    export_batch_snapshot = export_batch_snapshots

    @staticmethod
    def _export_batch_snapshots(batch_data, filepath, verbose=True):
        """Compatibility wrapper for the original batch snapshot saver."""
        resolved_path, _ = ReadoutClient._resolve_export_path(
            filepath, None, ('npz', 'npy', 'json', 'csv'), default='npz')
        ReadoutClient.export_batch_snapshots(filepath, batch_data)
        if verbose:
            print(f"Batch snapshots saved to {resolved_path}")

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
                'info': Structured info dict at time of capture.
        """
        response = self.send_request({'request': 'get_adc_snapshot'})
        if response['status'] != 'success':
            raise RuntimeError(f"ADC snapshot failed: {response.get('message')}")
        result = response['result']
        snapshot = np.frombuffer(
            base64.b64decode(result['snapshot']), dtype=np.complex128,
        ).copy()
        info = self.get_info('all')
        return {
            'snapshot': snapshot,
            'info': info,
            'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
        }

    def get_dac_snapshot(self):
        """
        Capture a single DAC snapshot (4096 complex128 samples per DAC).

        Returns:
            dict with keys:
                'dac0': complex128 array of shape (4096,).
                'dac1': complex128 array of shape (4096,).
                'info': Structured info dict at time of capture.
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
        info = self.get_info('all')
        return {
            'dac0': dac0,
            'dac1': dac1,
            'info': info,
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
            'info': snapshot_data.get('info', {}),
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
            'info': snapshot_data.get('info', {}),
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

    def perform_sweep(self, centers, spans, points, samples_per_point,
                      direction='up', phases=None, refresh_adc_cal=True,
                      adc_cal_settle_time=2.0, wait=False, autosync=True,
                      setup_sync=True):
        """
        Perform a frequency sweep.

        ADC calibration is always frozen before sweeping and left frozen
        afterwards. By default the calibration is refreshed first (unfreeze,
        settle, freeze) so it adapts to the current tone configuration.

        Args:
            centers: Center frequencies for sweep tones.
            spans: Sweep span(s).
            points: Number of sweep points.
            samples_per_point: Number of samples per sweep point.
            direction: Sweep direction, 'up' or 'down'. Default 'up'.
            phases: Tone phases. If None, warns about zero phases.
            refresh_adc_cal (bool): If True (default), refresh ADC calibration
                before sweeping (unfreeze, settle, freeze). If False, skip
                the refresh but still ensure the calibration is frozen.
            adc_cal_settle_time (float): Seconds to wait for ADC calibration
                to settle. Default 2.0.
            wait (bool): If True, print the dispatch response, then block until
                the sweep completes and return the final completion response.
                Default False.
            autosync (bool): If True (default), trigger firmware sync when
                tone frequencies are applied before/during/after the sweep
                (the per-step sync after each buffer flip).
            setup_sync (bool): If True (default), pulse one firmware sync at the
                start of the sweep to establish the TX/RX phase reference before
                stepping. Independent of ``autosync``: with
                ``setup_sync=True, autosync=False`` the LO is aligned once and
                rides continuous accumulation across the per-step buffer flips.
        """
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers, autosync=autosync)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        if phases is not None:
            self.set_tone_phases(np.atleast_1d(phases), autosync=autosync)
        else:
            self._warn_zero_phases()

        message = {
            'request': 'sweep',
            'centers': centers.tolist(),
            'spans': spans.tolist(),
            'points': points,
            'samples_per_point': samples_per_point,
            'direction': direction,
            'refresh_adc_cal': refresh_adc_cal,
            'adc_cal_settle_time': adc_cal_settle_time,
            'autosync': bool(autosync),
            'setup_sync': bool(setup_sync),
        }
        response = self.send_request(message)
        if wait:
            print(response)
            if response.get('status') == 'success':
                return self.wait_for_sweep(completion_message='Sweep complete')
        return response

    def perform_retune(self, centers, spans, points, samples_per_point,
                       direction='up', method='max_gradient',
                       freq_offsets=None, phases=None, refresh_adc_cal=True,
                       adc_cal_settle_time=2.0, wait=False, autosync=True,
                       setup_sync=True):
        """
        Perform a retune sweep to find optimal tone frequencies.

        ADC calibration is always frozen before sweeping and left frozen
        afterwards. By default the calibration is refreshed first.

        Args:
            centers: Center frequencies for sweep tones.
            spans: Sweep span(s).
            points: Number of sweep points.
            samples_per_point: Number of samples per sweep point.
            direction: Sweep direction, 'up' or 'down'. Default 'up'.
            method: Retune method, 'max_gradient', 'min_mag', or 'max_dphidf'. Default 'max_gradient'.
            freq_offsets: Frequency offsets for noise estimation. Default None (zeros).
            phases: Tone phases. If None, warns about zero phases.
            refresh_adc_cal (bool): If True (default), refresh ADC calibration
                before sweeping. If False, skip refresh but still ensure frozen.
            adc_cal_settle_time (float): Seconds to wait for ADC calibration
                to settle. Default 2.0.
            wait (bool): If True, print the dispatch response, then block until
                the retune completes and return the final completion response.
                Default False.
            autosync (bool): If True (default), trigger firmware sync when
                tone frequencies are applied before/during/after retune
                (the per-step sync after each buffer flip).
            setup_sync (bool): If True (default), pulse one firmware sync at the
                start of the underlying sweep to establish the TX/RX phase
                reference. Independent of ``autosync``.
        """
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers, autosync=autosync)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        if phases is not None:
            self.set_tone_phases(np.atleast_1d(phases), autosync=autosync)
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

        assert method in ['max_gradient','min_mag','max_dphidf'], "method must be 'max_gradient', 'min_mag', or 'max_dphidf'"

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
            'freq_offsets': freq_offsets.tolist(),
            'refresh_adc_cal': refresh_adc_cal,
            'adc_cal_settle_time': adc_cal_settle_time,
            'autosync': bool(autosync),
            'setup_sync': bool(setup_sync),
        }
        response = self.send_request(message)
        if wait:
            print(response)
            if response.get('status') == 'success':
                return self.wait_for_sweep(completion_message='Retune complete')
        return response

    def get_sweep_progress(self):
        """Return current sweep progress as a float from 0.0 to 1.0."""
        message = {'request': 'get_sweep_progress'}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response['progress']
        else:
            print(f"Error getting sweep progress: {response['message']}")
            return response

    def wait_for_sweep(self, poll_interval=1.0, progress_bar=True, completion_message='Sweep complete'):
        """
        Block until the current sweep completes, optionally displaying progress.

        Args:
            poll_interval (float): Seconds between progress polls. Default 1.0.
            progress_bar (bool): If True, display an ASCII progress bar. If False,
                                 print a plain numeric progress line. Default True.
            completion_message (str): Message returned in the final success
                response once progress reaches 100%. Default 'Sweep complete'.
        """
        import sys
        bar_width = 40
        printed_progress = False
        while True:
            response = self.send_request({'request': 'get_sweep_progress'})
            if not isinstance(response, dict):
                response = {'status': 'error', 'message': 'No response getting sweep progress'}
            if response.get('status') != 'success':
                print(f"Error getting sweep progress: {response.get('message', 'unknown error')}")
                if printed_progress:
                    sys.stdout.write('\n')
                    sys.stdout.flush()
                return response
            progress = response['progress']
            pct = float(progress)
            if progress_bar:
                filled = int(bar_width * pct)
                bar = '#' * filled + '-' * (bar_width - filled)
                sys.stdout.write(f'\rSweep progress: [{bar}] {pct*100:5.1f}%')
                sys.stdout.flush()
            else:
                sys.stdout.write(f'\rSweep progress: {pct*100:5.1f}%')
                sys.stdout.flush()
            printed_progress = True
            final_state = response.get('state')
            if pct >= 1.0 or final_state in ('idle', 'cancelled', 'error'):
                sys.stdout.write('\n')
                sys.stdout.flush()
                final_status = 'error' if final_state in ('cancelled', 'error') else 'success'
                final_message = response.get('message') or completion_message
                return {'status': final_status, 'message': final_message}
            time.sleep(poll_interval)

    def get_sweep_data(self):
        """Fetch the latest averaged sweep data from the server."""
        message = {'request': 'get_sweep_data'}
        response = self.send_request(message)
        if not isinstance(response, dict):
            raise RuntimeError("Error getting sweep_data: no response from server")
        if response.get('status') == 'success':
            sweep_data = response['data']
            return sweep_data
        raise RuntimeError(
            f"Error getting sweep_data: {response.get('message', 'unknown error')}")

    def get_sweep_raw_samples(self):
        """Fetch unaveraged per-sample sweep data as a complex array."""
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
        """Fetch the server's plain-text representation of the latest sweep."""
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
        if not isinstance(sweep_data, dict):
            raise TypeError("sweep_data must be a dictionary returned by get_sweep_data().")

        if sweep_data.get('status') == 'error':
            raise RuntimeError(
                f"Error getting sweep_data: {sweep_data.get('message', 'unknown error')}")
        if sweep_data.get('status') == 'success' and 'data' in sweep_data:
            sweep_data = sweep_data['data']

        required_keys = (
            'info', 'date', 'num_tones', 'num_points', 'samples_per_point',
            'sweep')
        missing_keys = [key for key in required_keys if key not in sweep_data]
        if missing_keys:
            raise ValueError(
                "Invalid sweep data: missing "
                + ", ".join(repr(key) for key in missing_keys))

        info = sweep_data['info']
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

                adcclk = info['fpga']['adc_clk_hz']
                dacclk = adcclk
                dacduc = info['rfdc']['dac_duc_mixer_frequency_hz']
                dacnyq = info['rfdc']['nyquist_zone_dac0']
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
                        'info': info,
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
        """Export parsed or raw sweep data to npy, json, or CSV.

        CSV exports use tone-numbered column groups: ``sweep_f_0000`` is
        tone/trace zero, and each data row is one sweep point.  Wideband sweeps
        are stored as a single trace so importing them preserves the
        ``(1, N_total_points)`` shape used by plotting and analysis helpers.

        Parameters
        ----------
        filename : str
            Output path; its extension selects the format when ``file_format``
            is not given.
        sweep_data : dict
            Parsed or raw sweep-data dict to export.
        file_format : {'npy', 'json', 'csv', 'dirfile', 'hdf5'} or None, optional
            Output format; ``None`` (default) infers it from ``filename``
            (falling back to ``'npy'``).
        """
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
            sweep_f = np.atleast_2d(np.asarray(sweep_dict['sweep_f']))
            sweep_i = np.atleast_2d(np.asarray(sweep_dict['sweep_i']))
            sweep_q = np.atleast_2d(np.asarray(sweep_dict['sweep_q']))
            sweep_ei = np.atleast_2d(np.asarray(sweep_dict['sweep_ei']))
            sweep_eq = np.atleast_2d(np.asarray(sweep_dict['sweep_eq']))
            if not (sweep_f.shape == sweep_i.shape == sweep_q.shape
                    == sweep_ei.shape == sweep_eq.shape):
                raise ValueError("Sweep arrays must all have the same shape.")

            with open(filepath, mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['# date', sweep_dict['date']])
                writer.writerow(['# num_tones', sweep_dict['num_tones']])
                writer.writerow(['# num_points', sweep_dict['num_points']])
                writer.writerow(['# samples_per_point', sweep_dict['samples_per_point']])
                writer.writerow(['# csv_layout', 'tone_columns'])
                if 'wideband_sweep' in sweep_dict:
                    writer.writerow(['# wideband_sweep', sweep_dict['wideband_sweep']])
                ReadoutClient._write_csv_info_metadata(writer, sweep_dict['info'])
                if 'telescope_time' in sweep_dict:
                    writer.writerow(['# telescope_time_per_point'] + [int(t) for t in sweep_dict['telescope_time']])

                if sweep_dict.get('wideband_sweep', False) and sweep_f.shape[0] == 1:
                    sweep_f = sweep_f.T
                    sweep_i = sweep_i.T
                    sweep_q = sweep_q.T
                    sweep_ei = sweep_ei.T
                    sweep_eq = sweep_eq.T

                num_csv_points, num_csv_tones = sweep_f.shape

                header = []
                for tone in range(num_csv_tones):
                    header.extend([f'sweep_f_{tone:04d}', f'sweep_i_{tone:04d}',
                                   f'sweep_q_{tone:04d}', f'err_i_{tone:04d}',
                                   f'err_q_{tone:04d}'])
                writer.writerow(header)
                for point in range(num_csv_points):
                    row = []
                    for tone in range(num_csv_tones):
                        row.extend([
                            f'{sweep_f[point][tone]}',
                            f'{sweep_i[point][tone]}',
                            f'{sweep_q[point][tone]}',
                            f'{sweep_ei[point][tone]}',
                            f'{sweep_eq[point][tone]}'
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
        """Import sweep data from ``filename`` (npy, json, or CSV; format from
        the extension).

        CSV imports accept both the corrected tone-column layout and the legacy
        point-column layout.  New CSVs include a ``csv_layout`` marker; older
        files are inferred from the number of data rows and numbered column
        groups in the header.
        """
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
                            value = value[1:-1]
                        try:
                            value = ast.literal_eval(value)
                        except (ValueError, SyntaxError):
                            pass
                        sweep_dict[key] = value

            data = np.genfromtxt(filename, delimiter=',',names=True,skip_header=header_lines)
            field_names = data.dtype.names
            sweep_indices = ReadoutClient._sweep_csv_indices(field_names, 'sweep_f')
            for prefix in ('sweep_i', 'sweep_q', 'err_i', 'err_q'):
                if ReadoutClient._sweep_csv_indices(field_names, prefix) != sweep_indices:
                    raise ValueError(
                        f"Sweep CSV {prefix}_NNNN columns do not match sweep_f_NNNN.")

            first_column = np.atleast_1d(data[f'sweep_f_{sweep_indices[0]:04d}'])
            csv_layout = ReadoutClient._infer_sweep_csv_layout(
                sweep_dict, len(sweep_indices), len(first_column))
            #samples_per_point = sweep_dict['samples_per_point']

            wideband_sweep = sweep_dict.get('wideband_sweep', False)
            sweep_f = ReadoutClient._read_sweep_csv_columns(
                data, sweep_indices, 'sweep_f', csv_layout, wideband_sweep)
            sweep_i = ReadoutClient._read_sweep_csv_columns(
                data, sweep_indices, 'sweep_i', csv_layout, wideband_sweep)
            sweep_q = ReadoutClient._read_sweep_csv_columns(
                data, sweep_indices, 'sweep_q', csv_layout, wideband_sweep)
            err_i = ReadoutClient._read_sweep_csv_columns(
                data, sweep_indices, 'err_i', csv_layout, wideband_sweep)
            err_q = ReadoutClient._read_sweep_csv_columns(
                data, sweep_indices, 'err_q', csv_layout, wideband_sweep)
            sweep_dict['sweep_f'] = sweep_f
            sweep_dict['sweep_i'] = sweep_i
            sweep_dict['sweep_q'] = sweep_q
            sweep_dict['sweep_ei'] = err_i
            sweep_dict['sweep_eq'] = err_q
            sweep_dict['csv_layout'] = csv_layout

        elif filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        elif filename.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        else:
            raise ValueError(f"Invalid file format {filename.split('.')[-1]}")

        return sweep_dict


    def receive_stream(self, num_tones=None, filename=None, print_data=False):
        """Receive continuous stream frames from the stream socket and write them to disk.

        Parameters
        ----------
        num_tones : int or None, optional
            Number of active tones to expect per frame; ``None`` infers it from
            the server info.
        filename : str or None, optional
            Output basename; defaults to ``tmp_stream`` in the cwd.
        print_data : bool, optional
            Print frames as they arrive (default ``False``).
        """
        if self.mock:
            return self._mock_server.receive_stream(num_tones, filename, print_data)

        iq_data=None
        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_stream')
            print(f"No filename specified, writing to {filename}")
        info = self.get_info('all')

        # Determine num_tones from info (server sends only active tones)
        tone_indices = info.get('tones', {}).get('firmware_indices')
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
        metadata['info'] = info

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


    def receive_stream_g3(self, num_tones=None, filename=None, print_data=False,
                          kid_stream_id='UNSET', duration=30):
        '''Receive a data stream and write it to a G3 (spt3g) file.

        Parameters
        ----------
        num_tones : int or None, optional
            Number of active tones to expect; ``None`` infers it from server info.
        filename : str or None, optional
            Output basename; a default is chosen when ``None``.
        print_data : bool, optional
            Print frames as they arrive (default ``False``).
        kid_stream_id : str, optional
            Stream identifier written into the G3 metadata (default ``'UNSET'``).
        duration : float, optional
            Capture duration in seconds (default ``30``).
        '''
        if self.mock:
            return self._mock_server.receive_stream_g3(
                num_tones=num_tones, filename=filename, print_data=print_data,
                kid_stream_id=kid_stream_id, duration=duration)

        # JL: Presumably this gets updated if something radically changes in this code
        SOSTREAM_VERSION = 1
        # JL: Level 1 data shows this is typically around 400
        num_sample_rows_per_frame = 400
        num_headers = 10

        data = bytearray(2048*2*4 + num_headers*4)
        view = memoryview(data)
        iq_data = None
        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_stream.g3')
            print(f"No filename specified, writing to {filename}")
        if not filename.endswith('.g3'):
            filename += '.g3'
        if not os.path.exists(os.path.dirname(os.path.abspath(filename))):
            os.makedirs(os.path.dirname(os.path.abspath(filename)))

        # Set up logging
        pid = os.getpid()
        # Assumes input filename ends in '.g3'
        log_filename = filename[:-3] + '_log_' + str(pid) + '.log'
        logging.basicConfig(filename=log_filename, level=logging.INFO,
                            format='%(asctime)s : %(levelname)s : %(message)s')
        logger = logging.getLogger(__name__)

        info = self.get_info('all')

        tone_indices = info.get('tones', {}).get('firmware_indices')
        if num_tones is None:
            num_tones = len(tone_indices) if tone_indices is not None else 2048

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
        metadata['info'] = info

        # JL setting similar to Smurf at the the moment - some of our primary names won't exist.
        primary_names = [
            'UnixTime', 'FluxRampIncrement', 'FluxRampOffset', 'Counter0',
            'Counter1', 'Counter2', 'AveragingResetBits', 'FrameCounter',
            'TESRelaySetting']
        primary_idxs = {name: idx for idx, name in enumerate(primary_names)}

        # JL Indexing below strips the assumed .g3 extension from the supplied filename and replaces it with .json
        # pdb.set_trace()
        with open(filename[:-3]+'.json', 'w') as file:
            json.dump(metadata, file, indent=4)

        logger.info('Wrote JSON header to '+filename[:-3]+'.json')
        logger.info('Preparing to receive TCP/IP data from : '+ self.stream_server_address +':'+ str(self.stream_server_port))
        logger.info(f'Will stream for duration {duration}')

        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.stream_server_address, self.stream_server_port))

            with spt3g.core.G3Writer(filename=filename) as writer:
                logger.info(f"Writing data to {filename}")
                print(f"Writing data to {filename}")
                t0 = time.time()
                frame_count = 0 # Counter for total number of frames (each of  length num_sample_rows_per_frame) written/
                count = 0    # counter for total number of packets (data rows) received.
                row_frame_count = 0 # Counter for number of packets received within the frame so far.
                ppid = os.getppid()

                key_interupt = False
                general_exception = False

                # First write out an observation frame
                # JL: TO DO
                # increment frame count
                fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Observation)
                t0 = time.time()
                session_id = int(t0) # Unix start time in whole seconds
                fr['frame_num'] = frame_count
                fr['session_id'] = session_id
                fr['sostream_id'] = kid_stream_id
                fr['sostream_version'] = SOSTREAM_VERSION
                fr['stream_placement'] = 'start'
                fr['time'] = spt3g.core.G3Time(t0 * spt3g.core.G3Units.s)
                writer(fr)
                frame_count += 1
                #################################################
                # Then write out a  write out a Wiring Frame
                fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Wiring)
                t0 = time.time()
                fr['frame_num'] = frame_count
                fr['session_id'] = session_id
                fr['sostream_id'] = kid_stream_id
                fr['sostream_version'] = SOSTREAM_VERSION
                fr['time'] = spt3g.core.G3Time(t0 * spt3g.core.G3Units.s)
                fr['dump'] = True
                # Persistent bug here with the yaml string being truncated.
                '''
                yaml_dump_string =  yaml.dump(metadata)
                logger.info('###################################')
                logger.info(str(type(metadata)))
                logger.info('###################################')
                logger.info(yaml_dump_string)
                logger.info('###################################')
                logger.info(str(metadata))
                logger.info('###################################')
                logger.info(json.dumps(metadata).encode())
                logger.info('###################################')
                logger.info('###################################')
                logger.info(len(yaml_dump_string))
                logger.info('###################################')
                logger.info(len(str(metadata)))
                logger.info('###################################')
                logger.info(len(json.dumps(metadata).encode()))
                logger.info('###################################')
                logger.info('###################################')
                logger.info(yaml_dump_string[-50:])
                logger.info('###################################')
                logger.info(str(metadata)[-50:])
                logger.info('###################################')
                logger.info(json.dumps(metadata).encode()[-50:])
                fr['status'] = yaml_dump_string
                '''
                # Thus write the json string instead, which does not have this problem.
                fr['status'] = json.dumps(metadata).encode()
                writer(fr)
                frame_count += 1
                #################################################
                start = time.time() # JL will be ultimately derived from the PTP data in the packets
                time_now = time.time()

                while (time_now-t0) < duration:
                    while True:
                        try:
                            #quit if parent has changed, prevents zombie processes
                            if os.getppid() != ppid:
                                size_of_this_frame = row_frame_count
                                break

                            # Read data length
                            raw_datalen = s.recv(4)
                            if not raw_datalen:
                                continue
                            datalen = struct.unpack('>I', raw_datalen)[0]
                            if datalen == 0:
                                continue
                            if datalen > len(data):
                                data = bytearray(datalen)
                                view = memoryview(data)
                            received_len = 0
                            while received_len < datalen:
                                packet_len = s.recv_into(view[received_len:], datalen - received_len)
                                if packet_len == 0:
                                    break
                                received_len += packet_len
                            if received_len < datalen:
                                logger.error(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                                print(f"Expected {datalen} bytes, but only received {received_len} bytes.")
                                size_of_this_frame = row_frame_count
                                break

                            #############################################
                            # JL Pickout out data from the bufferm contruct a 1-D "row" transfomar into a column vector
                            # then hstack to build up the 2-D data_frame_buffer
                            # Example row structure in the original PR:
                            # i_data_0000,q_data_0000,i_data_0001,q_data_0001,i_data_0002,q_data_0002,i_data_0003,q_data_0003,i_data_0004,q_data_0004,i_data_0005,q_data_0005,i_data_0006
                            # ,q_data_0006,packet_counter,packet_error,flag0,flag1,flag2,flag3,flag4,flag5,flag6,flag7
                            # Number of columns in a row =
                            # iq_data = num_tones * 2
                            # cnt = 1
                            # err = 1
                            # flags = 8
                            # = num_tones*2 + 10
                            #############################################
                            # SR: Modern stream frames are active tones in user order, then:
                            # flag0..flag5, tt_msb, tt_lsb, cnt, err.
                            frame_words = datalen // 4
                            frame_tones = (frame_words - num_headers) // 2
                            if datalen % 4 != 0 or frame_tones < num_tones:
                                raise ValueError(
                                    f"Stream frame length {datalen} bytes cannot contain "
                                    f"{num_tones} tones plus {num_headers} header words.")

                            all_data = np.frombuffer(data[:datalen], dtype='<i4')
                            iq_words = all_data[:2*num_tones].astype(np.int64)
                            tail = all_data[-num_headers:].astype(np.int64)
                            flags = tail[:6]
                            tt_msb = int(tail[6]) & 0xFFFFFFFF
                            tt_lsb = int(tail[7]) & 0xFFFFFFFF
                            tt = (tt_msb << 32) + tt_lsb
                            cnt = int(tail[8])
                            err = int(tail[9])
                            full_row = np.concatenate(
                                (iq_words, flags, np.array([tt, cnt, err], dtype=np.int64)),
                                axis=0)
                            full_row = full_row.reshape(-1,1) # make into column vector
                            if row_frame_count == 0:
                                data_frame_buffer = full_row
                            else:
                                data_frame_buffer = np.hstack((data_frame_buffer, full_row))
                            #print('##############################')
                            #print(f"Frame is {data_frame_buffer}")
                            #print("Shape is ", np.shape(data_frame_buffer))
                            #print(f"Frame is {data_frame_buffer}\r", end='', flush=True)

                            count += 1
                            row_frame_count += 1

                            if (row_frame_count == num_sample_rows_per_frame):
                                size_of_this_frame = row_frame_count
                                # Frame is full, reset counter and exit the loop
                                row_frame_count = 0
                                break

                            if print_data:
                                i = iq_words[::2]
                                q = iq_words[1::2]
                                iq_data = i+1j*q
                                print(f"datalen {datalen} row frame count {row_frame_count} Received IQ data: err={err} cnt={cnt} tt={tt} {iq_data.tolist()} \r",end='',flush=True)

                        except KeyboardInterrupt:
                            size_of_this_frame = row_frame_count
                            key_interupt = True
                            break

                        except Exception as e:
                            size_of_this_frame = row_frame_count
                            general_exception = True
                            logger.error(f"Error receiving stream data: {e}")
                            logger.error(traceback.format_exc())
                            print(f"Error receiving stream data: {e}")
                            print(traceback.format_exc())
                            break

                    # End "while true" data frame buffer contruction loop
                    # We arrive here if a frame has become completely filled or a keyboard interrupt or exception has happened.
                    # in either case size_of_this_frame contains the numer of rows in the frame
                    logger.info(f'About to write frame {frame_count}')
                    # If we arrive here after an exception leave the loop entirely.
                    if key_interupt or general_exception:
                        logger.info(f"Key Interrupt: {key_interupt} General exception: {general_exception} ")
                        break
                    if size_of_this_frame == 0:
                        break

                    #### Write the scan frame out
                    fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Scan)
                    sample_rate = metadata['sample_rate']
                    #Setup the 1-D time array for the data part of the frame
                    times = np.linspace(start,start+(size_of_this_frame)/sample_rate, size_of_this_frame) # JL Utimately will be from PTP within packets

                    #SR: once telescope time from ptp is confirmed working, this should be replaced with the following to get the correct timestamp for each sample:
                    if False:
                        times = tt

                    g3times = spt3g.core.G3VectorTime(times * spt3g.core.G3Units.s)

                    chans = np.arange(num_tones)
                    # Set up the row descriptive names for the data part of the frame
                    # v1.2.0 uses 6 flags and combines tt_msb/tt_lsb into telescope_time.
                    names = ['_']*(2*num_tones+6+1+1+1) # 6 flags, 1 telescope time, 1 cnt, 1 err
                    names[0:2*len(chans):2] = [f'i{ch:0>4}' for ch in chans] # i followed by zero padded 4 digit channel (tone) number
                    names[1:2*len(chans):2] = [f'q{ch:0>4}' for ch in chans] # q followed by zero padded 4 digit channel (tone) number
                    names[num_tones*2:num_tones*2+6] = [f'flag{flag}' for flag in list(range(6))]
                    names[num_tones*2+6] = 'telescope_time'
                    names[num_tones*2+7] = 'cnt'
                    names[num_tones*2+8] = 'err'

                    # Write the data frame - row names (len = 2*num_tones + 9), times (len = size_of_this_frame), 2-D data_frame_buffer  = len(row_names) * len(times).
                    fr['data'] = so3g.G3SuperTimestream(names, g3times, data_frame_buffer)
                    #pdb.set_trace()
                    # This is purely a counter of how much data is in the frame - look at cnt to see if packets have been dropped.
                    frame_counter = np.arange(0,size_of_this_frame, dtype=int)
                    primary_data = np.zeros((len(primary_names), size_of_this_frame), dtype=np.int64)
                    primary_data[primary_idxs['UnixTime'], :] = (times * 1e9).astype(int)
                    primary_data[primary_idxs['FrameCounter'], :] = frame_counter
                    fr['primary'] = so3g.G3SuperTimestream(primary_names, g3times, primary_data)

                    fr['timing_paradigm'] = 'High Precision'
                    #SR: client.get_info('timing') tells us the current status, eg ptp_locked, or ptp_holdover, or ntp or free-running.
                    # we should match up our available status values to the available timing paradigms in the G3 frame metadata

                    fr['num_samples'] = size_of_this_frame # per frame
                    fr['frame_num'] = frame_count # JL Numbering from 0
                    fr['session_id'] = session_id
                    fr['sostream_id'] = kid_stream_id
                    fr['sostream_version'] = SOSTREAM_VERSION
                    fr['time'] = spt3g.core.G3Time(time.time() * spt3g.core.G3Units.s) # JL Presumably meant to be the time when frame is written out, not the timestamp of the first element of the frame??
                    writer(fr)
                    frame_count += 1
                    time_now = time.time()
                #end while (time_now-t0) < duration:

                logger.info(f'Streaming duration {duration} expired.')
                logger.info(f"Writing final observation frame..")
                t1 = time.time()

                # At the end of the observation write out an observation frame.
                fr = spt3g.core.G3Frame(spt3g.core.G3FrameType.Observation)
                fr['frame_num'] = frame_count
                fr['session_id'] = session_id
                fr['sostream_id'] = kid_stream_id
                fr['sostream_version'] = SOSTREAM_VERSION
                fr['stream_placement'] = 'end'
                fr['time'] = spt3g.core.G3Time(t1 * spt3g.core.G3Units.s)
                writer(fr)
            # End of "with core..."
        # end of "with socket..."
        print()
        print(f"Received {count} samples in ~{t1-t0} seconds (~{count/(t1-t0)} samples per second)")
        logger.info(f"Received {count} samples in ~{t1-t0} seconds (~{count/(t1-t0)} samples per second)")
        return iq_data


    def receive_triggered_stream(self, num_tones=None, filename=None, print_data=False):
        """Receive triggered stream frames from the stream socket and write them to disk.

        Parameters
        ----------
        num_tones : int or None, optional
            Number of active tones to expect per frame; ``None`` infers it from
            the server info.
        filename : str or None, optional
            Output basename; defaults to ``tmp_triggered_stream`` in the cwd.
        print_data : bool, optional
            Print frames as they arrive (default ``False``).
        """
        if self.mock:
            return self._mock_server.receive_stream(num_tones, filename, print_data)

        if filename is None:
            filename = os.path.join(os.getcwd(), 'tmp_triggered_stream')
            print(f"No filename specified, writing to {filename}")

        info = self.get_info('all')

        # Determine num_tones from info (server sends only active tones)
        tone_indices = info.get('tones', {}).get('firmware_indices')
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
        metadata['info'] = info

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

        ``filename`` is the stream-capture basename (without extension); the
        matching ``.json`` metadata and binary data files are read from it.
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
        info = metadata['info']

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
                     'info':info,
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
        """Export parsed stream data to npy, json, or CSV.

        Parameters
        ----------
        filename : str
            Output path; its extension selects the format when ``file_format``
            is not given.
        data_dict : dict
            Parsed stream data (e.g. from :py:meth:`parse_stream`).
        file_format : {'npy', 'json', 'csv'} or None, optional
            Output format; ``None`` (default) infers it from ``filename``
            (falling back to ``'npy'``).
        """
        filepath, file_format = ReadoutClient._resolve_export_path(
            filename, file_format, ('npy', 'json', 'csv'), default='npy')

        date = data_dict['date']
        num_tones = data_dict['num_tones']
        num_samples = data_dict['num_samples']
        sample_rate = data_dict['sample_rate']
        info = data_dict['info']
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
                ReadoutClient._write_csv_info_metadata(writer, info)

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
        """Import stream data from ``filename`` (written by
        ``export_stream_data``)."""
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
                        if value.startswith('"') and value.endswith('"'):
                            value = value[1:-1]
                        try:
                            value = ast.literal_eval(value)
                        except (ValueError, SyntaxError):
                            pass
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
        Generate uniform-random phases (radians), one per frequency in ``freqs``.
        """
        return np.random.uniform(0,2*np.pi,len(freqs))
    
    @staticmethod
    def generate_newman_phases(freqs):
        """
        Generate the Newman phases (radians), one per frequency in ``freqs``.

        If frequencies are exactly evenly spaced, the crest factor is minimised.

        If the frequency spacing is not exactly equal, the phases are offset to account for the spacing. For largely varying spacings, this method is pretty much the same as picking random frequencies.

        """
        freqs = np.atleast_1d(freqs)
        n = len(freqs)
        if n == 1:
            return np.zeros(1)
        # Sort frequencies and get sort order
        sort_idx = np.argsort(freqs)
        inv_sort_idx = np.argsort(sort_idx)
        k = np.arange(n)
        # Compute phases for sorted frequencies
        phases_sorted = np.pi * k**2 / n
        # Return phases in the original order
        return phases_sorted[inv_sort_idx]

    @staticmethod
    def suggest_blind_frequencies(resonance_frequencies, count, band_hz,
                                  min_distance_hz, edge_margin_hz=0.0,
                                  candidate_spacing_hz=None,
                                  random_offset_fraction=0.35,
                                  rng=None):
        """Suggest blind-tone centers away from known resonances.

        Parameters
        ----------
        resonance_frequencies : array-like
            Frequencies to avoid, usually the current resonator centers.
        count : int
            Number of blind tones to suggest.
        band_hz : tuple
            ``(f_min, f_max)`` search band in Hz.
        min_distance_hz : float
            Minimum allowed distance from any resonance and from other blind
            tones.
        edge_margin_hz : float
            Margin excluded at each band edge.
        candidate_spacing_hz : float, optional
            Grid spacing for candidate centers. Defaults to
            the larger of ``min_distance_hz / 5`` and one part in 5000 of
            the usable band.
        random_offset_fraction : float
            Fraction of the nominal blind-tone spacing used to randomly jitter
            the ideal target positions. This avoids placing blind tones on a
            perfectly regular comb, which can align intermodulation products.
            Set to 0 for deterministic evenly-spaced targets.
        rng : numpy random generator, optional
            Random generator used for jitter. Defaults to ``np.random``.
        """
        if resonance_frequencies is None:
            avoid = np.array([], dtype=float)
        else:
            avoid = np.atleast_1d(resonance_frequencies).astype(float)
        count = int(count)
        if count < 0:
            raise ValueError('count must be non-negative')
        if count == 0:
            return np.array([], dtype=float)
        f_min, f_max = map(float, band_hz)
        edge_margin_hz = float(edge_margin_hz)
        min_distance_hz = float(min_distance_hz)
        lo = f_min + edge_margin_hz
        hi = f_max - edge_margin_hz
        if hi <= lo:
            raise ValueError('band_hz is empty after applying edge_margin_hz')
        if candidate_spacing_hz is None:
            candidate_spacing_hz = max(min_distance_hz / 5.0, (hi - lo) / 5000.0)
        candidate_spacing_hz = float(candidate_spacing_hz)
        if candidate_spacing_hz <= 0:
            raise ValueError('candidate_spacing_hz must be positive')
        random_offset_fraction = float(random_offset_fraction)
        if random_offset_fraction < 0:
            raise ValueError('random_offset_fraction must be non-negative')

        # Build a fine grid of possible parking spaces.  This grid is only used
        # for the search; final choices are picked from it after applying
        # resonance-avoidance and blind-to-blind spacing constraints.
        candidates = np.arange(lo, hi + candidate_spacing_hz / 2, candidate_spacing_hz)
        if candidates.size == 0:
            raise ValueError('No blind-tone candidates in requested band')

        def min_distance_to(values, points):
            """Return each candidate's distance to the nearest excluded point."""
            if len(points) == 0:
                return np.full(len(values), np.inf)
            return np.min(np.abs(values[:, None] - points[None, :]), axis=1)

        resonance_distance = min_distance_to(candidates, avoid)
        allowed = resonance_distance >= min_distance_hz
        if not np.any(allowed):
            raise ValueError('No blind-tone candidates satisfy min_distance_hz')

        selected = []
        # Start from evenly-spaced ideal target locations so the blind tones
        # monitor the whole band rather than clustering in one clean gap.
        targets = np.linspace(lo, hi, count + 2)[1:-1]
        # Regularly-spaced tones can produce aligned intermodulation products.
        # Jitter the targets by less than half their nominal spacing, then use
        # the nearest safe candidate to each jittered target.
        if random_offset_fraction > 0 and count > 1:
            if rng is None:
                rng = np.random
            nominal_spacing = (hi - lo) / (count + 1)
            max_offset = min(
                random_offset_fraction * nominal_spacing,
                0.45 * nominal_spacing)
            offsets = rng.uniform(-max_offset, max_offset, count)
            targets = np.clip(targets + offsets, lo, hi)

        for target in targets:
            selected_arr = np.asarray(selected, dtype=float)
            selected_distance = min_distance_to(candidates, selected_arr)
            available = allowed & (selected_distance >= min_distance_hz)
            if not np.any(available):
                raise ValueError(
                    f'Could only place {len(selected)} blind tones with '
                    f'min_distance_hz={min_distance_hz}')
            available_idx = np.nonzero(available)[0]
            nearest_idx = available_idx[np.argmin(np.abs(candidates[available_idx] - target))]
            selected.append(candidates[nearest_idx])

        return np.asarray(selected, dtype=float)

    @staticmethod
    def calculate_frequency_and_dissipation_noise(sweep_frequencies,sweep_complex_data,timestream_tone_frequency,timestream_complex_data,smooth_window_hz=1000):
        """
        Calculate the fractional frequency and dissipation noise timestreams from a sweep and complex timestream data.
        Valid only for small frequency and dissipation shifts close to the tone frequency.

        This compatibility wrapper delegates to the canonical linearized
        calibration in ``souk_readout_tools.resonator``.

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
            Timestream data is not smoothed.

        Returns
        -------
        fractional_frequency_noise : array
            The fractional frequency noise timestream.
        fractional_dissipation_noise : array
            The matched-scale dissipation quadrature. For the symmetric
            small-signal model this is ``Delta(1 / (2 * Qi))``.
        si0 : float
            The in-phase component of the smoothed sweep at the tone frequency.
        sq0 : float
            The quadrature component of the smoothed sweep at the tone frequency.
        didf : float
            The gradient of the in-phase component of the smoothed sweep at the tone frequency.
        dqdf : float
            The gradient of the quadrature component of the smoothed sweep at the tone frequency.
        """
        from souk_readout_tools.resonator import (
            linearized_frequency_and_dissipation)

        frequency, dissipation, calibration = (
            linearized_frequency_and_dissipation(
                sweep_frequencies,
                sweep_complex_data,
                timestream_tone_frequency,
                timestream_complex_data,
                smooth_window_hz=smooth_window_hz,
                return_calibration=True,
            )
        )
        return (
            frequency,
            dissipation,
            calibration.reference_iq.real,
            calibration.reference_iq.imag,
            calibration.gradient.real,
            calibration.gradient.imag,
        )
    
    @staticmethod
    def read_resonances_file(filename):
        """
        Read a resonances file ``filename`` and return columns keyed by the
        names that numpy.genfromtxt assigns (sanitized from the file header).
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
    def parse_wideband_sweep(self, sweep_data,
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
                       optimise_tx_dynamic_range=True,
                       optimise_rx_gain=True,
                       refresh_adc_cal=True,
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
            tone_powers_dbm (float or array-like or 'auto' or None): Tone power in dBm.
                If 'auto', calls maximise_tx_power() to optimise the dynamic range.
                If a scalar, all tones are set to uniform power at this level (the
                strongest tone achieves the specified power at the reference plane).
                If an array (must match num_tones), per-tone powers are applied.
                If None, tone amplitudes are left untouched — useful when repeating
                a sweep after only an external change (e.g. TX attenuator) with the
                same num_tones. Typically pair with optimise_rx_gain=False so the
                RX chain is not re-adjusted either.
                Default is 'auto'.
            reference_plane (str): Where tone_powers_dbm is specified: 'dac',
                'rf_output', or 'detector' (default).
            apply_phase_correction (bool): DEPRECATED. Correct for phase jumps at filterbank
                                           channel edges. Default is False. This correction is
                                           no longer needed following firmware fixes.
            optimise_tx_dynamic_range (bool): If True, maximise DAC bit utilisation and
                adjust the TX analog chain when setting tone powers. Default is True.
            optimise_rx_gain (bool): If True, maximise ADC power utilisation and
                optimise the PFB FFT shift for best RX dynamic range after tones
                are configured. Calls maximise_rx_power(). Default is True.
            refresh_adc_cal (bool): If True (default), refresh ADC calibration
                before sweeping (unfreeze, settle, freeze). If False, skip
                the refresh but still ensure the calibration is frozen.
                Calibration is always left frozen after the sweep.
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
                - 'info': Structured info dict at time of sweep
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
        
        sections = ['fpga', 'rfdc']
        info_list = self.get_info(sections)
        if not isinstance(info_list, list) or len(info_list) != len(sections):
            raise RuntimeError(f'Failed to get info: {info_list}')
        info = dict(zip(sections, info_list))

        # Get RF frontend mixer configuration
        # TODO: if we have a frontend connected it might not have a mixer - needs updating.
        udc = self.config['rf_frontend']['connected']
        lo = self.config['rf_frontend'].get('tx_mixer_lo_frequency_hz', 0.0) if udc else 0.0
        sb = self.config['rf_frontend'].get('tx_mixer_sideband', +1) if udc else 1
        if lo is None:
            lo = 0.0
        if sb is None:
            sb = 1

        adcclk = info['fpga']['adc_clk_hz']
        dacclk = adcclk
        dacduc = info['rfdc']['dac_duc_mixer_frequency_hz']
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
                                optimise_dynamic_range=optimise_tx_dynamic_range,
                                verbose=verbose)
            if stp_response.get('status') != 'success':
                raise RuntimeError(
                    f"Failed to set tone powers: {stp_response.get('message', 'unknown error')}")
            if verbose:
                print(f'  set_tone_powers(): {stp_response.get("status")}')
        else:
            # tone_powers_dbm is None: leave tone amplitudes untouched
            if verbose:
                print(f'  tone_powers_dbm=None: leaving existing tone amplitudes unchanged')

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

        # ADC calibration management before sweep
        # ADC calibration: always frozen before sweep, left frozen after
        if refresh_adc_cal:
            # Full refresh: unfreeze, settle, freeze
            if verbose:
                print(f'  Refreshing ADC calibration...')
            self.set_cal_freeze(False)
            if verbose:
                print(f'  Waiting for ADC calibration to settle...')
            time.sleep(2.0)
            if verbose:
                print(f'  Freezing calibration...')
            self.set_cal_freeze(True)
        else:
            # Ensure frozen, settling first if needed
            if not self.get_cal_freeze():
                if verbose:
                    print(f'  Waiting for ADC calibration to settle...')
                time.sleep(2.0)
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
        while True:
            p = self.get_sweep_progress()
            if verbose:
                print(f'Sweep progress: {100*p:.1f}%', end='\r', flush=True)
            if p == 1.0:
                break
            time.sleep(1.0)
        if verbose:
            print()  # Newline after progress

        # Get and parse the sweep data as a wideband sweep
        s = self.parse_wideband_sweep(self.get_sweep_data(),
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


    def set_tones_helper(self, freqs, amps=None, phases=None, powers_dbm=None,
                         autosync=True):
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
            autosync: If True (default), trigger firmware sync after tone
                      frequency/amplitude/phase writes.
        """
        import warnings

        if freqs is None or len(freqs) == 0:
            raise ValueError("Frequencies must be provided and cannot be empty.")
        freqs = np.atleast_1d(freqs)

        self.set_tone_frequencies(freqs, autosync=autosync)
        active_freqs = self.get_tone_frequencies()
        if phases is None:
            phases = self.generate_newman_phases(active_freqs)

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
                tone_freqs = active_freqs[idxs]
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
            self.set_tone_powers(powers_dbm, autosync=autosync)
        else:
            if amps is None:
                amps = np.ones_like(active_freqs)
            amps = np.atleast_1d(amps).copy()
            if len(amps) == len(freqs) and len(active_freqs) > len(freqs):
                try:
                    current_amps = self.get_tone_amplitudes()
                    amps = np.concatenate([amps, current_amps[len(freqs):]])
                except Exception:
                    amps = np.concatenate([
                        amps,
                        np.ones(len(active_freqs) - len(freqs), dtype=float)])
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
            self.set_tone_amplitudes(amps, autosync=autosync)
        self.set_tone_phases(phases, autosync=autosync)
        return

    def measure_path_group_delay(self,
                                 sweep_data=None,
                                 kid_frequencies=None,
                                 kid_q_factors=None,
                                 auto_mask_resonances=True,
                                 mask_hwhm_factor=50.0,
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
              are supplied, or if ``auto_mask_resonances`` is True and no
              explicit resonance list is supplied, frequency channels within
              ``mask_hwhm_factor`` × HWHM of each resonance are excluded from
              subsequent filtering.

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
            auto_mask_resonances (bool): If True (default), automatically find
                MKID resonances in the supplied wideband sweep and mask them
                when ``kid_frequencies`` is not provided.
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
        import warnings
        try:
            from scipy.ndimage import median_filter as nd_median_filter
            from scipy.signal import savgol_filter
        except ImportError:
            nd_median_filter = None
            savgol_filter = None

        # --- acquire sweep data -------------------------------------------------
        if sweep_data is None:
            if verbose:
                print('measure_path_group_delay: performing wideband sweep...')
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

        if kid_frequencies is None and auto_mask_resonances:
            try:
                from ..peak_finder import find_mkid_resonances

                auto_resonances = find_mkid_resonances(freqs, s21)
                if auto_resonances:
                    kid_frequencies = np.array(
                        [res.frequency for res in auto_resonances], dtype=float)
                    kid_q_factors = np.array([
                        1e4 if res.q_factor is None or not np.isfinite(res.q_factor)
                        else max(float(res.q_factor), 1.0)
                        for res in auto_resonances
                    ], dtype=float)
                    if verbose:
                        print(f'  Auto resonance masking: found {len(kid_frequencies)} resonances')
                elif verbose:
                    print('  Auto resonance masking: found no resonances; leaving sweep unmasked')
            except Exception as exc:
                warnings.warn(
                    f'Automatic resonance masking failed; proceeding without masking: {exc}')

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

        # Compute filter kernels in points. Boundary handling below avoids
        # flat endpoint padding, which biases the filtered delay at both ends.
        freq_span_mhz = (freqs[-1] - freqs[0]) / 1e6 if n_pts > 1 else 0.0
        pts_per_mhz = n_pts / freq_span_mhz if freq_span_mhz > 0 else 0.0
        med_kernel = 0
        savgol_window = 0

        if pts_per_mhz > 0 and median_filter_mhz > 0:
            med_kernel = max(3, int(round(pts_per_mhz * median_filter_mhz)))
            if med_kernel % 2 == 0:
                med_kernel += 1

        if (savgol_filter is not None and pts_per_mhz > 0
                and n_pts > savgol_poly_order + 2):
            savgol_window = max(savgol_poly_order + 2,
                                int(round(pts_per_mhz * savgol_mhz)))
            if savgol_window % 2 == 0:
                savgol_window += 1
            savgol_window = min(savgol_window,
                                n_pts if n_pts % 2 == 1 else n_pts - 1)

        if n_pts > 1 and med_kernel >= 3 and len(included_idx) > med_kernel:
            vals = tau_ns_work[included_idx]
            if nd_median_filter is not None:
                filtered_vals = nd_median_filter(
                    vals, size=med_kernel, mode='reflect')
            else:
                half = med_kernel // 2
                padded = np.pad(vals, (half, half), mode='reflect')
                half = med_kernel // 2
                filtered_vals = np.empty_like(vals)
                for i in range(len(vals)):
                    window = padded[i:i + 2 * half + 1]
                    filtered_vals[i] = np.median(window)
            tau_ns_work[included_idx] = filtered_vals
            if verbose:
                print(f'  Median filter: kernel {med_kernel} points '
                      f'({med_kernel/pts_per_mhz:.1f} MHz)')

        # --- interpolate across masked regions before savgol --------------------
        tau_for_savgol = tau_ns_work.copy()
        excluded_idx = np.where(~mask)[0]
        if len(excluded_idx) > 0:
            tau_for_savgol[excluded_idx] = np.interp(
                freqs[excluded_idx],
                freqs[included_idx],
                tau_ns_work[included_idx])

        # --- Savitzky-Golay filter ---------------------------------------------
        if savgol_window >= savgol_poly_order + 2:
            tau_ns_smooth = savgol_filter(
                tau_for_savgol,
                savgol_window,
                savgol_poly_order,
                mode='interp')

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
                self.config_raw_text = None
                basename = os.path.basename(save_to_csv)
                with open(save_to_csv) as f:
                    self.calibration_files[basename] = f.read()
                if verbose:
                    print(f'  Config rf_frontend.path_group_delay_ns set to "{save_to_csv}"')

        return result

    def save_path_group_delay_calibration(self, group_delay, frequencies=None,
                                          filename='path_group_delay.csv',
                                          update_config=True, verbose=True):
        """Write a path-group-delay calibration CSV into ``self.cal_dir``.

        Args:
            group_delay: Either a result dict from ``measure_path_group_delay()``
                or a 1-D array of ``tau_ns`` values.
            frequencies: 1-D array of frequencies in Hz when ``group_delay`` is
                not a result dict.
            filename: Output CSV filename or path. Relative paths are written
                inside ``self.cal_dir``.
            update_config: If True (default), update
                ``rf_frontend.path_group_delay_ns`` in the in-memory config and
                cache the file contents for ``save_config()`` / ``push_config()``.
            verbose: Print status messages.

        Returns:
            Absolute path to the written calibration CSV.
        """
        import csv
        import os

        if isinstance(group_delay, dict):
            if frequencies is None:
                frequencies = group_delay.get('frequencies')
            tau_ns = group_delay.get('tau_ns')
        else:
            tau_ns = group_delay

        if frequencies is None:
            raise ValueError('frequencies must be provided when group_delay is not a result dict')

        frequencies = np.asarray(frequencies, dtype=float).ravel()
        tau_ns = np.asarray(tau_ns, dtype=float).ravel()
        if frequencies.shape != tau_ns.shape:
            raise ValueError('frequencies and group delay arrays must have the same shape')

        if os.path.isabs(filename):
            save_to_csv = filename
        else:
            os.makedirs(self.cal_dir, exist_ok=True)
            save_to_csv = os.path.join(self.cal_dir, filename)
        save_to_csv = os.path.abspath(save_to_csv)
        os.makedirs(os.path.dirname(save_to_csv), exist_ok=True)

        with open(save_to_csv, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['freq_hz', 'tau_ns'])
            for freq_hz, tau_val in zip(frequencies, tau_ns):
                writer.writerow([f'{freq_hz:.3f}', f'{tau_val:.6f}'])

        basename = os.path.basename(save_to_csv)
        with open(save_to_csv) as f:
            self.calibration_files[basename] = f.read()

        if update_config:
            if self.config is None:
                raise RuntimeError('No config loaded; cannot update rf_frontend.path_group_delay_ns')
            self.config.setdefault('rf_frontend', {})['path_group_delay_ns'] = save_to_csv
            self.config_raw_text = None
            if verbose:
                print(f'Config rf_frontend.path_group_delay_ns set to "{save_to_csv}"')

        if verbose:
            print(f'Saved path group delay calibration to {save_to_csv}')
            print('Use push_config() to persist this calibration on the RFSoC.')

        return save_to_csv


    @staticmethod
    def _overlapping_tone_groups(sweep_f):
        """Return connected groups of tone columns whose sweep ranges overlap."""
        sf = np.atleast_2d(np.asarray(sweep_f, dtype=float))
        intervals = []
        for tone in range(sf.shape[1]):
            finite = sf[:, tone][np.isfinite(sf[:, tone])]
            if finite.size == 0:
                continue
            lo = float(np.min(finite))
            hi = float(np.max(finite))
            intervals.append((lo, hi, tone))

        if len(intervals) < 2:
            return []

        intervals.sort(key=lambda item: (item[0], item[1], item[2]))
        groups = []
        group = [intervals[0][2]]
        group_hi = intervals[0][1]

        for lo, hi, tone in intervals[1:]:
            if lo <= group_hi:
                group.append(tone)
                group_hi = max(group_hi, hi)
            else:
                if len(group) > 1:
                    groups.append(group)
                group = [tone]
                group_hi = hi

        if len(group) > 1:
            groups.append(group)
        return groups

    @staticmethod
    def _select_overlapping_targeted_resonances(per_tone, sweep_f):
        """Assign ordered candidates across overlapping targeted sweep columns.

        This only resolves the simple common case: an overlap group of N tones
        where every tone found exactly N ordered resonances.  Then the
        lower-frequency tone center gets the lower-frequency candidate, etc.
        More ambiguous doubles/triples are left in place and flagged normally.
        """
        sf = np.atleast_2d(np.asarray(sweep_f, dtype=float))
        selected = [list(results) for results in per_tone]
        resolved_tones = set()

        centers = []
        for tone in range(sf.shape[1]):
            finite = sf[:, tone][np.isfinite(sf[:, tone])]
            center = float(np.median(finite)) if finite.size else float(tone)
            centers.append(center)

        for group in ReadoutClient._overlapping_tone_groups(sf):
            n_group = len(group)
            if not all(len(per_tone[tone]) == n_group for tone in group):
                continue
            ordered_tones = sorted(group, key=lambda tone: (centers[tone], tone))
            for rank, tone in enumerate(ordered_tones):
                selected[tone] = [per_tone[tone][rank]]
                resolved_tones.add(tone)

        return selected, sorted(resolved_tones)


    def find_resonances(self, sweep_data=None, mode='auto',
                        data_format='log_magnitude',
                        filter_params=None, finder_params=None, **kwargs):
        """
        Find MKID resonances in sweep data using the peak_finder module.

                Supports three modes:
                        - 'auto': infers the sweep type from parsed sweep metadata/shape.
                        - 'wideband': searches the full concatenated sweep trace for all
                            resonances (default, for wideband_sweep data).
                        - 'targeted': searches within each tone's individual sweep for
                            resonances. Returns per-tone results and flags tones with
                            multiple resonances (doubles/triples).

        Args:
            sweep_data (dict, optional): Sweep data dictionary with keys
                'sweep_f', 'sweep_i', 'sweep_q'. If None, performs a new
                wideband_sweep (for mode='wideband' or mode='auto') or raises
                an error (for mode='targeted').
            mode (str): 'auto', 'wideband', or 'targeted'.
            data_format (str): Analysis format for peak finding. One of:
                'lin_magnitude', 'log_magnitude', 'phase', 'unwrapped_phase',
                'group_delay', 'complex_gradient'. Default is 'log_magnitude'.
            filter_params: FilterParams instance or dict using exact keys:
                ``highpass_edge``, ``lowpass_edge``, ``median_kernel_size``.
            finder_params: PeakFinderParams instance or dict using exact keys:
                ``prominence_enabled``, ``prominence_min``,
                ``prominence_max``, ``width_enabled``, ``width_min``,
                ``width_max``, ``distance_enabled``, ``distance_value``,
                ``height_enabled``, ``height_min``, ``height_max``,
                ``threshold_enabled``, ``threshold_min``, ``threshold_max``,
                ``peak_direction``, ``max_num_peaks``, ``f_low``,
                ``f_high``. For example:
                ``finder_params={'prominence_min': 0.5}``.
                In wideband mode, omitted values use conservative MKID defaults:
                inner 10-90% of the frequency span, dip finding, 1-100 dB
                prominence, 1 kHz-10 MHz width, 100 kHz minimum spacing,
                lowpass 0.5 and highpass 0. Dicts override individual defaults.
                In targeted mode, omitted values use ``PeakFinderParams()``
                defaults. Dict aliases such as ``min_height`` are not accepted;
                use the exact field name such as ``height_min`` and set the
                matching ``*_enabled`` field when needed.
            **kwargs: Passed to wideband_sweep if sweep_data is None.

        Returns:
            ResonanceSearchResult for all modes. It is list-like over all
            ResonanceResult objects, so existing wideband-style usage such as
            ``len(result)``, ``for r in result``, and ``result[:5]`` still
            works. It also supports dict-style access to:
                'mode': resolved search mode, 'wideband' or 'targeted'.
                'per_tone': list of lists. In wideband mode this has one
                    entry containing the single concatenated-trace result.
                'all_resonances': flat list of all ResonanceResult objects.
                'flagged_tones': tone indices with >1 resonance after simple
                    overlap resolution.
                'num_tones': total number of tones in the sweep metadata.
        """
        from ..peak_finder import (
            find_mkid_resonances, FilterParams, PeakFinderParams,
            ResonanceSearchResult, wideband_resonance_search_params,
        )

        if mode == 'auto':
            if sweep_data is None:
                mode = 'wideband'
            elif sweep_data.get('wideband_sweep', False):
                mode = 'wideband'
            else:
                sweep_f = np.atleast_2d(sweep_data['sweep_f'])
                mode = 'wideband' if sweep_f.shape[0] == 1 else 'targeted'

        if mode == 'wideband':
            if sweep_data is None:
                sweep_data = self.wideband_sweep(**kwargs)

            frequencies = np.ravel(sweep_data['sweep_f'])
            s21_complex = (np.ravel(sweep_data['sweep_i'])
                           + 1j * np.ravel(sweep_data['sweep_q']))
            filter_params, finder_params = wideband_resonance_search_params(
                frequencies, filter_params=filter_params,
                finder_params=finder_params)

            results = find_mkid_resonances(
                frequencies=frequencies,
                s21_complex=s21_complex,
                data_format=data_format,
                filter_params=filter_params,
                finder_params=finder_params,
            )
            num_tones = int(sweep_data.get(
                'num_tones_used', sweep_data.get('num_tones', 1)))

            return ResonanceSearchResult(
                mode='wideband',
                all_resonances=results,
                per_tone=[results],
                flagged_tones=[],
                num_tones=num_tones,
            )

        elif mode == 'targeted':
            if isinstance(filter_params, dict):
                filter_params = FilterParams(**filter_params)
            if isinstance(finder_params, dict):
                finder_params = PeakFinderParams(**finder_params)

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

            _, n_tones = sf.shape
            per_tone_raw = []

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
                per_tone_raw.append(results)

            per_tone, _ = self._select_overlapping_targeted_resonances(
                per_tone_raw, sf)

            all_resonances = []
            flagged_tones = []
            for t, results in enumerate(per_tone):
                all_resonances.extend(results)
                if len(results) == 0:
                    print(f'Warning: no resonances found in tone {t} sweep.')

                if len(results) > 1:
                    flagged_tones.append(t)

            all_resonances.sort(key=lambda r: r.frequency)

            return ResonanceSearchResult(
                mode='targeted',
                all_resonances=all_resonances,
                per_tone=per_tone,
                flagged_tones=flagged_tones,
                num_tones=n_tones,
            )

        else:
            raise ValueError(
                f"Unknown mode '{mode}'. Use 'auto', 'wideband', or 'targeted'.")

    def find_resonance_frequencies(self, sweep_data, only_first=True,**kwargs):
        """
        Convenience method to get just the resonance frequencies.
        
        Args:
            sweep_data: Optional sweep data dict. If None, performs a sweep.
            only_first: If True (default), return only the first found resonance per tone but 
                        if no resonance found for a tone, return the frequency from the sweep info dict.
                        If False, return all found resonances in a flat list.
            **kwargs: Passed to find_resonances.
        
        Returns:
            np.ndarray: Array of resonance frequencies in Hz.
                For targeted sweeps this is the flattened
                ``all_resonances`` frequency list.
        """
        resonances = self.find_resonances(sweep_data, **kwargs)

        if only_first:
            if hasattr(resonances, 'per_tone'):
                resonances = resonances.per_tone
            elif isinstance(resonances, dict):
                resonances = resonances['per_tone']

            freqs= []

            for i,p in enumerate(resonances):
                if len(p) == 0:
                    freqs.append(sweep_data['info']['tones']['frequencies_hz'][i])
                else:
                    freqs.append(p[0].frequency)
            return np.array(freqs)
        
        else:
            if hasattr(resonances, 'all_resonances'):
                resonances = resonances.all_resonances
            elif isinstance(resonances, dict):
                resonances = resonances['all_resonances']

            return np.array([r.frequency for r in resonances])


    def open_kid_finder_app(self, sweep_data=None, sweep_file=None, prompt_save=False):
        """
        Launch the KID Finder App GUI with the given sweep data.
        Args:
            sweep_data: Sweep data dictionary (as returned by wideband_sweep or parse_sweep_data).
            sweep_file: Path to a sweep .npy file. If not provided and sweep_data is given, a temp file is used.
            prompt_save: If True and sweep_data is given, prompt the user to save the file instead of using a temp file.
        """
        import subprocess, sys, tempfile, os
        import numpy as np
        from pathlib import Path

        if sweep_file is None and sweep_data is not None:
            if prompt_save:
                # Prompt user for save location (simple CLI prompt)
                out_path = input("Enter filename to save sweep data (or leave blank for temp file): ").strip()
                if out_path:
                    np.save(out_path, sweep_data)
                    sweep_file = out_path
                else:
                    tmp = tempfile.NamedTemporaryFile(suffix='.npy', delete=False)
                    np.save(tmp.name, sweep_data)
                    sweep_file = tmp.name
            else:
                tmp = tempfile.NamedTemporaryFile(suffix='.npy', delete=False)
                np.save(tmp.name, sweep_data)
                sweep_file = tmp.name
        elif sweep_file is None:
            raise ValueError("Must provide either sweep_data or sweep_file.")

        # Find the mkid_finder_app.py script location
        app_path = Path(__file__).parent.parent / "mkid_finder_app.py"
        if not app_path.exists():
            raise FileNotFoundError(f"Could not find mkid_finder_app.py at {app_path}")

        # Launch the app with the sweep file as argument
        cmd = [sys.executable, str(app_path), str(sweep_file)]
        try:
            subprocess.Popen(cmd)
        except Exception as e:
            print(f"Failed to launch KID Finder App: {e}")
            raise




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
