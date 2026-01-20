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
     'pwd': '/home/casper/src/readout_server',
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
Version: 0.1

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

try:
    from importlib.resources import files as importlib_files
except ImportError:
    from importlib_resources import files as importlib_files


def get_pipeline_dirs(pipeline_id):
    """
    Get pipeline-specific directory paths.
    
    For dual-pipeline support, each pipeline uses its own subdirectory:
      ~/.souk_readout_tools/pipeline_<id>/config/
      ~/.souk_readout_tools/pipeline_<id>/calibrations/
      ~/.souk_readout_tools/pipeline_<id>/tmp/
    
    Returns a dict with keys: 'config', 'calibrations', 'tmp', 'default_config'
    """
    home = os.path.expanduser('~')
    base_dir = os.path.join(home, '.souk_readout_tools', f'pipeline_{pipeline_id}')
    dirs = {
        'base': base_dir,
        'config': os.path.join(base_dir, 'config'),
        'calibrations': os.path.join(base_dir, 'calibrations'),
        'tmp': os.path.join(base_dir, 'tmp'),
        'default_config': os.path.join(base_dir, 'config', 'default_config.lnk')
    }
    return dirs


def ensure_pipeline_dirs(pipeline_id):
    """
    Ensure pipeline-specific directories exist.
    If the default config doesn't exist, copy template files from package data.
    """
    dirs = get_pipeline_dirs(pipeline_id)
    for key in ('config', 'calibrations', 'tmp'):
        os.makedirs(dirs[key], exist_ok=True)
    
    # Copy template config files if default config doesn't exist
    if not os.path.exists(dirs['default_config']):
        _copy_template_configs(dirs, pipeline_id)
    
    return dirs


def _copy_template_configs(dirs, pipeline_id):
    """
    Copy template configuration files from package data to the user's pipeline config directory.
    Updates pipeline_id in template_config.yaml to match the target pipeline.
    """
    print(f"First run for pipeline {pipeline_id}: copying template config files to {dirs['config']}")
    
    try:
        # Access package data directory
        pkg_config_dir = importlib_files('souk_readout_tools').joinpath('data', 'config')
        
        # Copy template_config.yaml and update pipeline_id
        template_src = pkg_config_dir.joinpath('template_config.yaml')
        template_dst = os.path.join(dirs['config'], 'template_config.yaml')
        
        with open(str(template_src), 'r') as f:
            template_content = yaml.safe_load(f)
        
        # Update pipeline_id in the template to match target pipeline
        if 'firmware' in template_content:
            template_content['firmware']['pipeline_id'] = pipeline_id
        
        with open(template_dst, 'w') as f:
            yaml.dump(template_content, f, default_flow_style=False, sort_keys=False)
        
        os.chmod(template_dst, 0o664)
        
        # Create default_config.lnk pointing to template_config.yaml
        default_lnk_dst = dirs['default_config']
        with open(default_lnk_dst, 'w') as f:
            f.write(template_dst)
        
        os.chmod(default_lnk_dst, 0o664)
        
        print(f"  Created {template_dst}")
        print(f"  Created {default_lnk_dst} -> {template_dst}")
        print(f"\033[93mNote: Please edit {template_dst} with your system-specific settings.\033[0m")
        
    except Exception as e:
        print(f"\033[91mWarning: Could not copy template config files: {e}\033[0m")
        print(f"\033[93mYou may need to manually create a config file in {dirs['config']}\033[0m")


def extract_pipeline_id_from_config(config_file):
    """
    Extract pipeline_id from a config file without fully loading it.
    Returns the pipeline_id (int) or 0 if not found.
    """
    if config_file is None:
        return 0
    try:
        # Handle .lnk files that contain a path to the real config
        with open(config_file, 'r') as f:
            content = yaml.safe_load(f)
        if isinstance(content, str):
            # It's a link file, follow it
            linked_file = content
            if not os.path.exists(linked_file):
                # Try searching in standard locations
                for pid in (0, 1):
                    test_path = os.path.join(get_pipeline_dirs(pid)['config'], linked_file)
                    if os.path.exists(test_path):
                        linked_file = test_path
                        break
            with open(linked_file, 'r') as f:
                content = yaml.safe_load(f)
        if isinstance(content, dict) and 'firmware' in content:
            return content['firmware'].get('pipeline_id', 0)
    except Exception as e:
        print(f"Warning: Could not extract pipeline_id from {config_file}: {e}")
    return 0


# Legacy module-level variables for backward compatibility
# These will be overwritten per-instance in ReadoutClient
USER_CALIBRATIONS_DIR = os.path.expanduser('~/.souk_readout_tools/pipeline_0/calibrations')
USER_CONFIG_DIR = os.path.expanduser('~/.souk_readout_tools/pipeline_0/config')
USER_TMP_DIR = os.path.expanduser('~/.souk_readout_tools/pipeline_0/tmp')
DEFAULT_CONFIG = os.path.join(USER_CONFIG_DIR, 'default_config.lnk')

# Ensure default pipeline_0 dirs exist for backward compatibility
ensure_pipeline_dirs(0)

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
    def __init__(self, config_file=None, pipeline_id=None):
        """
        Initialize the ReadoutClient.
        
        For dual-pipeline support, each pipeline uses separate directories:
          ~/.souk_readout_tools/pipeline_0/  (for pipeline_id=0)
          ~/.souk_readout_tools/pipeline_1/  (for pipeline_id=1)
        
        The pipeline_id is determined from the config file's firmware.pipeline_id value,
        which is authoritative since the server uses it to create firmware interfaces.
        
        The explicit pipeline_id parameter is only used as a hint for locating the 
        default config when no config_file is specified.
        
        Args:
            config_file: Path to config YAML. If None, uses default_config.lnk from
                         the pipeline directory specified by pipeline_id.
            pipeline_id: Only used when config_file is None, to select which pipeline's
                         default config to load. Ignored if config_file is provided
                         (config file's pipeline_id takes precedence).
        """
        # Step 1: If no config file specified, use pipeline_id hint to find default config
        initial_pipeline_id = pipeline_id if pipeline_id is not None else 0
        
        if config_file is None:
            # Use the hint pipeline_id to find the default config
            hint_dirs = ensure_pipeline_dirs(initial_pipeline_id)
            config_file = hint_dirs['default_config']
            print(f'Loading default config file from {config_file}')
        
        # Step 2: Load the config file (following links if necessary)
        if not os.path.exists(config_file):
            # Try searching in the hint pipeline's config dir
            hint_dirs = get_pipeline_dirs(initial_pipeline_id)
            config_file = os.path.join(hint_dirs['config'], config_file)
        
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        
        if type(config) is str:
            # File contains a link to another config file
            config_file = config
            if not os.path.exists(config_file):
                # Try searching in standard locations
                for pid in (0, 1):
                    test_path = os.path.join(get_pipeline_dirs(pid)['config'], config_file)
                    if os.path.exists(test_path):
                        config_file = test_path
                        break
            with open(config_file, 'r') as file:
                config = yaml.safe_load(file)
        
        print(f'Config file loaded: {config_file}')
        
        self.config = config
        self.config_file = config_file
        
        # Step 3: Extract pipeline_id FROM THE CONFIG (this is authoritative)
        self.pipeline_id = self.config.get('firmware', {}).get('pipeline_id', 0)
        
        # Warn if explicit pipeline_id was provided and differs from config
        if pipeline_id is not None and pipeline_id != self.pipeline_id:
            print(f'{bcolors.FAIL}ERROR: Explicit pipeline_id ({pipeline_id}) differs from '
                  f'config file pipeline_id ({self.pipeline_id}).{bcolors.ENDC}')
            print(f'{bcolors.WARNING}Using config file pipeline_id={self.pipeline_id} '
                  f'(this is what the firmware will use).{bcolors.ENDC}')
        
        # Step 4: Now set up directories based on CONFIG's pipeline_id
        self.pipeline_dirs = ensure_pipeline_dirs(self.pipeline_id)
        self.user_config_dir = self.pipeline_dirs['config']
        self.user_calibrations_dir = self.pipeline_dirs['calibrations']
        self.user_tmp_dir = self.pipeline_dirs['tmp']
        self.default_config = self.pipeline_dirs['default_config']
        
        print(f'Using pipeline {self.pipeline_id} directories:')
        print(f'  config: {self.user_config_dir}')
        print(f'  calibrations: {self.user_calibrations_dir}')
        print(f'  tmp: {self.user_tmp_dir}')
        
        # Update THIS pipeline's default config link
        with open(self.default_config, 'w') as file:
            file.write(os.path.abspath(config_file))
        
        self.request_server_address = self.config['rfsoc_host']['address']
        self.request_server_port = self.config['rfsoc_host']['request_port']
        self.stream_server_address = self.config['rfsoc_host']['address']
        self.stream_server_port = self.config['rfsoc_host']['stream_port']
        self.system_information = None
        self.parameters={}

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

    def pull_config(self, destination_dir=None):
        if destination_dir is None:
            destination_dir = self.user_config_dir
        message = {'request': 'pull_config'}
        response = self.send_request(message)
        if response['status'] == 'success':
            config_filename = response['config_filename']
            config_contents = response['config_contents']
            destination_file = os.path.join(destination_dir, os.path.basename(config_filename))
            with open(destination_file, 'w') as file:
                file.write(config_contents)
            print(f'Config file pulled from RFSoC into {destination_file}')
            self.config_file = destination_file
            self.config = yaml.safe_load(config_contents)
            print(f'Config loaded {self.config_file}')
            with open(self.default_config, 'w') as file:
                file.write(os.path.abspath(self.config_file))

            
        else:
            return response

    def push_config(self):
        name = os.path.basename(self.config_file)
        config = yaml.dump(self.config,sort_keys=False)
        message = {'request': 'push_config', 'config_filename': name, 'config_contents': config}
        response = self.send_request(message)
        if response['status'] == 'success':
            print(f'Config file pushed from {self.config_file} to RFSoC')
            return 
        else:
            return response
       # if response['status'] == 'success':
       #     response=self.initialise_firmware(config_file=name)
       # else:
       #     print(f"Error saving config: {response['message']}")
       #     return response
        #if default:
        #    self.set_config_default(name)


    #def set_config_default(self,config_filename):
    #    message = {'request': 'set_default_config', 'config_filename': config_filename}
    #    response = self.send_request(message)
    #    return response


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
            destination_file = os.path.join(self.user_calibrations_dir, os.path.basename(remote_file))
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
        message = {'request': 'get_system_information'}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.system_information = response['data']
            return response['data']
        else:
            print(f"Error getting system information: {response['message']}")
            return response

    def set_parameter(self, param_name, param_value):
        message = {'request': 'set', 'param': param_name, 'value': param_value}
        response = self.send_request(message)
        if response['status'] == 'success':
            return response
        else:
            print(f"Error setting parameter {param_name}: {response['message']}")
            return response

    def get_parameter(self, param_name):
        message = {'request': 'get', 'param': param_name}
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

    def set_tone_powers(self, tone_powers_dbm):
        tone_powers_dbm = np.atleast_1d(tone_powers_dbm).tolist()
        return self.set_parameter('tone_powers',tone_powers_dbm)

    def get_tone_powers(self,detailed_output=False):
        if detailed_output:
            return self.get_parameter('tone_powers_detailed')
        else:
            return np.atleast_1d(self.get_parameter('tone_powers'))

    def check_input_saturation(self,iterations=10):
        message = {'request': 'check_input_saturation','iterations':iterations}
        return self.send_request(message)

    def check_output_saturation(self,iterations=10):
        message = {'request': 'check_output_saturation','iterations':iterations}
        return self.send_request(message)

    def check_dsp_overflow(self,duration_s=0.2):
        message = {'request': 'check_dsp_overflow','duration_s':duration_s}
        return self.send_request(message)
    
    def maximise_tx_power(self):
        return self.send_request({'request': 'maximise_tx_power'})
    
    def maximise_rx_power(self):
        return self.send_request({'request': 'maximise_rx_power'})
    
    def optimise_tx_snr(self):
        return self.send_request({'request': 'optimise_tx_snr'})
    
    def optimise_rx_snr(self):
        return self.send_request({'request': 'optimise_rx_snr'})

    def fix_dac_saturation(self):
        return self.send_request({'request': 'fix_dac_saturation'})
    
    def fix_adc_saturation(self):
        return self.send_request({'request': 'fix_adc_saturation'})

    def enable_stream(self):
        message = {'request': 'enable_stream'}
        return self.send_request(message)

    def disable_stream(self):
        message = {'request': 'disable_stream'}
        return self.send_request(message)

    def enable_triggered_stream(self):
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

    def get_samples(self, num_samples,incl_system_info=True):
        """
        Acquire num_samples samples from the readout server and return concatenated raw data.
        """
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((self.request_server_address, self.request_server_port))
            message = {'request': 'get_samples', 'num_samples': num_samples}
            # Send message length
            message_data = json.dumps(message).encode()
            message_len = struct.pack('>I', len(message_data))
            s.sendall(message_len + message_data)
            # # Pre-allocate bytearray to expected data length
            alldatalen = 2048*2*4 + 10*4
            data_raw = bytearray(alldatalen*num_samples)
            view = memoryview(data_raw)

            # i_data = np.zeros((num_samples,num_tones),dtype=int)
            # q_data = np.zeros((num_samples,num_tones),dtype=int)
            # cnt = np.zeros(num_samples,dtype=int)
            # err = np.zeros(num_samples,dtype=int)
            # flags = np.zeros((num_samples,8),dtype=int)
            t0=time.time()
            next_datalen=0
            for j in range(num_samples):
                packet_offset = j*next_datalen

                # Read data length
                raw_datalen = s.recv(4)
                if not raw_datalen:
                    break
                next_datalen = struct.unpack('>I', raw_datalen)[0]
                received_len = 0
                while received_len < next_datalen:
                    packet_len = s.recv_into(view[packet_offset+received_len:], next_datalen - received_len)
                    if packet_len == 0:
                        break
                    received_len += packet_len
                if received_len < next_datalen:
                    print(f"Expected {next_datalen} bytes, but only received {received_len} bytes.")
                    break
            t1=time.time()
            print(f"Received {num_samples} samples in ~{t1-t0} seconds (~{num_samples/(t1-t0)} samples per second)")
            if incl_system_info:
                info = self.get_system_information()
            else:
                info = {'system_information':'No system information requested'}
            sample_rate = self.get_sample_rate()
            sample_data = {'data_raw':data_raw,'sample_rate':sample_rate,'system_information':info}
            return sample_data


    @staticmethod
    def parse_samples(sample_data,num_tones=2048):
        data_raw = sample_data['data_raw']
        sample_rate = sample_data['sample_rate']
        info = sample_data['system_information']
        datalen = 2048*2*4 + 10*4
        num_samples = len(data_raw)//datalen
        i_data = np.zeros((num_samples,num_tones),dtype='<i4')
        q_data = np.zeros((num_samples,num_tones),dtype='<i4')
        cnt = np.zeros(num_samples,dtype=int)
        err = np.zeros(num_samples,dtype=int)
        flags = np.zeros((num_samples,8),dtype=int)
        for j in range(num_samples):
            packet_offset = j*datalen
            all_data = np.frombuffer(data_raw[packet_offset:packet_offset+datalen],dtype='<i4')
            i_data[j] = all_data[::2][:num_tones]
            q_data[j] = all_data[1::2][:num_tones]
            err[j] = all_data[-1]
            cnt[j] = all_data[-2]
            flags[j] = all_data[-10:-2]
        data_dict = {'date': time.strftime('%Y-%m-%d %H:%M:%S UTC%z'),
                    'num_tones':num_tones,
                    'num_samples':num_samples,
                    'sample_rate':sample_rate,
                    'system_information':info,
                    'i_data':{f'{i:04d}':i_data[:,i] for i in range(num_tones)},
                    'q_data':{f'{i:04d}':q_data[:,i] for i in range(num_tones)},
                    'packet_counter':cnt,
                    'packet_error':err,
                    'stream_flags':{f'flag{i}':flags[:,i] for i in range(8)}
                    }

        return data_dict

    @staticmethod
    def export_samples(filename, sample_data, num_tones_to_save=None,file_format=None):
        if num_tones_to_save is None:
            num_tones_to_save = 2048

        if file_format is None:
            try:
                file_format = filename.split('.')[-1]
            except IndexError:
                raise ValueError("No file format provided and unable to determine from filename.")

        if 'data_raw' in sample_data.keys():
            data_dict = ReadoutClient.parse_samples(sample_data,num_tones=num_tones_to_save)
        else:
            data_dict = sample_data

        if file_format == 'npy':
            np.save(filename.rstrip('npy') + 'npy', data_dict)

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

            with open(filename.rstrip('json') + 'json', 'w') as file:
                json.dump(json_data_dict, file, indent=4)

        elif file_format == 'csv':
            with open(filename.rstrip('csv') + 'csv', mode='w', newline='') as file:
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
                header.extend(['packet_counter', 'packet_error'])
                header.extend([f'flag{i}' for i in range(8)])
                writer.writerow(header)
                # Write the data rows
                for j in range(data_dict['num_samples']):
                    row = []
                    for i in range(data_dict['num_tones']):
                        row.extend([data_dict['i_data'][f'{i:04d}'][j], data_dict['q_data'][f'{i:04d}'][j]])
                    row.append(data_dict['packet_counter'][j])
                    row.append(data_dict['packet_error'][j])
                    row.extend([data_dict['stream_flags'][f'flag{k}'][j] for k in range(8)])
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
            data_dict['stream_flags'] = {f'flag{i}':data[f'flag{i}'].astype(int) for i in range(8)}

        elif filename.endswith('.hdf5'):
            raise NotImplementedError("hdf5 format not yet implemented.")
        elif os.path.endswith('.dirfile'):
            raise NotImplementedError("dirfile format not yet implemented.")
        else:
            raise ValueError(f"Invalid file format {filename.split('.')[-1]}")
        return data_dict


    def perform_sweep(self, centers, spans, points, samples_per_point,direction='up'):
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        message = {
            'request': 'sweep',
            'centers': centers.tolist(),
            'spans': spans.tolist(),
            'points': points,
            'samples_per_point': samples_per_point,
            'direction': direction
        }
        return self.send_request(message)

    def perform_retune(self, centers, spans, points, samples_per_point, direction='up', method='max_gradient', freq_offsets=None):
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

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

        if apply_phase_correction:
                
                #get bin indexes
                udc = self.config['rf_frontend']['connected']
                lo = self.config['rf_frontend']['tx_mixer_lo_frequency_hz'] if udc else 0.0
                sb = self.config['rf_frontend']['tx_mixer_sideband'] if udc else 1
                
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
                        'sweep_eq': err_q
                        }
        return data_dict

    @staticmethod
    def export_sweep(filename, sweep_data, file_format='npy'):
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))

        if 'sweep_eq' not in sweep_data.keys():
            sweep_dict = ReadoutClient.parse_sweep_data(sweep_data)
        else:
            sweep_dict = sweep_data

        if file_format == 'npy':
            np.save(filename.replace('.npy', '')+'.npy', sweep_dict)

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

            with open(filename.replace('.json', '')+'.json', 'w') as file:
                json.dump(json_data_dict, file, indent=4)


        elif file_format == 'csv':
            with open(filename.replace('.csv', '') + '.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['# date', sweep_dict['date']])
                writer.writerow(['# num_tones', sweep_dict['num_tones']])
                writer.writerow(['# num_points', sweep_dict['num_points']])
                writer.writerow(['# samples_per_point', sweep_dict['samples_per_point']])
                for key,value in sweep_dict['system_information'].items():
                    writer.writerow([f'# {key}', value])
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


    def receive_stream(self, num_tones=2048, filename=None, print_data=False):
        data = bytearray(2048*2*4 + 10*4)
        view = memoryview(data)
        iq_data=None
        if filename is None:
            filename = os.path.join(self.user_tmp_dir, 'tmp_stream')
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))
        info = self.get_system_information()

        metadata = {}
        metadata['date'] = time.strftime('%Y-%m-%d %H:%M:%S UTC%z')
        metadata['num_tones'] = num_tones
        metadata['sample_rate'] = self.get_sample_rate()
        metadata['format'] = '<i4'
        metadata['index_err'] = 2*num_tones-1+10
        metadata['index_cnt'] = 2*num_tones-1+9
        metadata['index_flag_7'] = 2*num_tones-1+8
        metadata['index_flag_6'] = 2*num_tones-1+7
        metadata['index_flag_5'] = 2*num_tones-1+6
        metadata['index_flag_4'] = 2*num_tones-1+5
        metadata['index_flag_3'] = 2*num_tones-1+4
        metadata['index_flag_2'] = 2*num_tones-1+3
        metadata['index_flag_1'] = 2*num_tones-1+2
        metadata['index_flag_0'] = 2*num_tones-1+1
        metadata['ordering'] = 'I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, Q_tone1_sample0,..flags, cnt, err .'
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
                        # file.write(data) # whole frame
                        # file.write(data[-1]) #err
                        # file.write(data[-2]) #cnt
                        # file.write(data[:num_tones*2*4]) #data

                        file.write(data[:num_tones*2*4]) # data
                        file.write(data[-40:]) # extras
                        count+=1

                        if print_data:
                            i = np.frombuffer(data[:datalen][::2], dtype='<i4')[:num_tones]
                            q = np.frombuffer(data[:datalen][1::2], dtype='<i4')[:num_tones]
                            err = np.frombuffer(data[-4:], dtype='<i4')
                            cnt = np.frombuffer(data[-8:-4], dtype='<i4')
                            flags = np.frombuffer(data[-40:-8], dtype='<i4')
                            iq_data=i+1j*q
                            print(f"Received IQ data: {err} {cnt} {iq_data.tolist()}\r",end='',flush=True)
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


    def receive_triggered_stream(self, num_tones=2048, filename=None, print_data=False):
        data = bytearray(4096*4 + 10*4)
        view = memoryview(data)
        if filename is None:
            filename = os.path.join(self.user_tmp_dir, 'tmp_triggered_stream')
        if not os.path.exists(os.path.dirname(filename)):
            os.makedirs(os.path.dirname(filename))

        info = self.get_system_information()

        metadata = {}
        metadata['date'] = time.strftime('%Y-%m-%d %H:%M:%S UTC%z')
        metadata['num_tones'] = num_tones
        metadata['sample_rate'] = self.get_sample_rate()
        metadata['format'] = '<i4'
        metadata['index_err'] = 2*num_tones-1+10
        metadata['index_cnt'] = 2*num_tones-1+9
        metadata['index_flag_7'] = 2*num_tones-1+8
        metadata['index_flag_6'] = 2*num_tones-1+7
        metadata['index_flag_5'] = 2*num_tones-1+6
        metadata['index_flag_4'] = 2*num_tones-1+5
        metadata['index_flag_3'] = 2*num_tones-1+4
        metadata['index_flag_2'] = 2*num_tones-1+3
        metadata['index_flag_1'] = 2*num_tones-1+2
        metadata['index_flag_0'] = 2*num_tones-1+1
        metadata['ordering'] = 'I_tone0_sample_0, Q_tone0_sample0, I_tone1_sample0, Q_tone1_sample0,..flags, cnt, err .'
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
                        # file.write(data) # whole frame
                        # file.write(data[-1]) #err
                        # file.write(data[-2]) #cnt
                        # file.write(data[:num_tones*2*4]) #data

                        file.write(data[:num_tones*2*4]) # data
                        file.write(data[-40:]) # extras
                        count+=1

                        if print_data:
                            i = np.frombuffer(data[:datalen][::2], dtype='<i4')[:num_tones]
                            q = np.frombuffer(data[:datalen][1::2], dtype='<i4')[:num_tones]
                            err = np.frombuffer(data[-4:], dtype='<i4')
                            cnt = np.frombuffer(data[-8:-4], dtype='<i4')
                            flags = np.frombuffer(data[-40:-8], dtype='<i4')
                            iq_data=i+1j*q
                            print(f"Received IQ data: {err} {cnt} {iq_data.tolist()}\r",end='',flush=True)
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
        with open(filename+'.json','r') as file:
            metadata = json.load(file)
        date = metadata['date']
        num_tones = metadata['num_tones']
        sample_rate = metadata['sample_rate']
        format = metadata['format']
        index_err = metadata['index_err']
        index_cnt = metadata['index_cnt']
        index_flag_0 = metadata['index_flag_0']
        index_flag_7 = metadata['index_flag_7']
        info = metadata['system_information']

        data = np.fromfile(filename,dtype=format)
        data = data.reshape(-1,2*num_tones+10).swapaxes(0,1)
        num_samples = data.shape[1]

        err = data[index_err]
        cnt = data[index_cnt]
        flags = data[index_flag_0:index_flag_7+1]

        i_data = data[:2*num_tones:2]
        q_data = data[1:2*num_tones:2]
        # iq_data = iq_data[::2]+1j*iq_data[1::2]
        data_dict = {'date':date,
                     'num_tones':num_tones,
                     'num_samples':num_samples,
                     'sample_rate':sample_rate,
                     'system_information':info,
                     'i_data':{f'{i:04d}':i_data[i] for i in range(num_tones)},
                     'q_data':{f'{i:04d}':q_data[i] for i in range(num_tones)},
                     'packet_counter':cnt,
                     'packet_error':err,
                     'stream_flags':{f'flag{i}':flags[i] for i in range(8)}
                     }
        return data_dict

    @staticmethod
    def export_stream_data(filename,data_dict,file_format='npy'):

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

        # metadata = data_dict['metadata']

        if file_format == 'npy':
            np.save(filename.rstrip('.npy') + '.npy', data_dict)

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

            with open(filename.rstrip('.json') + '.json', 'w') as file:
                json.dump(json_data_dict, file)

        elif file_format == 'csv':
            with open(filename.rstrip('.csv') + '.csv', mode='w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['# date', date])
                writer.writerow(['# num_tones', num_tones])
                writer.writerow(['# num_samples', num_samples])
                writer.writerow(['# sample_rate',sample_rate])
                for key,value in info.items():
                    writer.writerow([f'# {key}', value])

                header = []
                header.extend(['packet_counter', 'packet_error'])
                header.extend([f'flag{i}' for i in range(8)])
                for i in range(data_dict['num_tones']):
                    header.extend([f'i_{i:04d}'])
                    header.extend([f'q_{i:04d}'])

                writer.writerow(header)
                # Write the data rows
                for j in range(data_dict['num_samples']):
                    row = []
                    row.append(data_dict['packet_counter'][j])
                    row.append(data_dict['packet_error'][j])
                    row.extend([data_dict['stream_flags'][f'flag{k}'][j] for k in range(8)])
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
            data_dict['stream_flags'] = {f'flag{i}':data[f'flag{i}'] for i in range(8)}

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
        n=len(freqs)
        freqs=np.atleast_1d(freqs)
        freqssorted = np.sort(freqs)
        k = (freqs-freqssorted[0]) / (freqssorted[-1] - freqssorted[0])*(len(freqs)-1)
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


    def wideband_sweep(self, bandwidth_hz=None, center_freq_hz=None, step_size_hz=10000, 
                       num_tones=1024, samples_per_point=10, apply_phase_correction=False,
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
            apply_phase_correction (bool): DEPRECATED. Correct for phase jumps at filterbank 
                                           channel edges. Default is False. This correction is
                                           no longer needed following firmware fixes.
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
        if p != 0.0 and p != 1.0:
            raise RuntimeError(f'Sweep already in progress ({p*100:.3f}%), wait for it to finish.')
        
        info = self.get_system_information()
        
        # Get RF frontend configuration
        udc = self.config['rf_frontend']['connected']
        lo = self.config['rf_frontend']['tx_mixer_lo_frequency_hz']
        sb = self.config['rf_frontend']['tx_mixer_sideband']
        
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
            bandwidth_hz = rfmax - rfmin
        if center_freq_hz is None:
            center_freq_hz = (rfmax + rfmin) / 2

        fmin = center_freq_hz - bandwidth_hz / 2
        fmax = center_freq_hz + bandwidth_hz / 2

        if (fmin < rfmin) or (fmax > rfmax):
            raise ValueError(f'Requested sweep out of band (band = {rfmin/1e6:.1f} - {rfmax/1e6:.1f} MHz, '
                           f'requested {fmin/1e6:.1f} - {fmax/1e6:.1f} MHz)')

        # Calculate tone frequencies
        freqs, spacings = np.linspace(fmin, fmax, num_tones, endpoint=False, retstep=True)
        
        if spacings <= dacclk / txnfft:
            raise ValueError(f'Tone spacing must be greater than {dacclk/txnfft:.0f} Hz but is {spacings:.0f} Hz. '
                           f'Try fewer tones or wider bandwidth.')

        sweep_points = int(bandwidth_hz / step_size_hz / num_tones)
        sweep_span = spacings * (sweep_points - 1) / sweep_points

        # Add small random offsets to avoid systematic effects
        small_offsets = np.random.uniform(-sweep_span / sweep_points / 2, 
                                          +sweep_span / sweep_points / 2, num_tones)
        freqs += small_offsets
        center_freqs = freqs + np.floor(sweep_points / 2) * spacings / sweep_points
        
        tone_amplitudes = np.ones(num_tones)
        tone_phases = self.generate_newman_phases(center_freqs)

        if verbose:
            print(f'Wideband sweep configuration:')
            print(f'  RF band: {rfmin/1e6:.1f} - {rfmax/1e6:.1f} MHz')
            print(f'  Sweep range: {fmin/1e6:.1f} - {fmax/1e6:.1f} MHz ({bandwidth_hz/1e6:.1f} MHz)')
            print(f'  Num tones: {num_tones}, sweep points: {sweep_points}')
            print(f'  Total points: {num_tones * sweep_points}')

        # Configure tones
        self.set_tone_frequencies(center_freqs)
        self.set_tone_amplitudes(tone_amplitudes)
        self.set_tone_phases(tone_phases)

        # Check for saturation/overflow before sweeping
        outps = self.check_output_saturation()
        inps = self.check_input_saturation()
        dspof = self.check_dsp_overflow()
        
        if outps['result']:
            raise RuntimeError(f"Output saturation detected: {outps['details']}")
        if inps['result']:
            raise RuntimeError(f"Input saturation detected: {inps['details']}")
        if dspof['result']:
            raise RuntimeError(f"DSP overflow detected: {dspof['details']}")

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

        # Get and parse the sweep data
        s = self.parse_sweep_data(self.get_sweep_data(), apply_phase_correction=apply_phase_correction)
        f = s['sweep_f']
        z = s['sweep_i'] + 1j * s['sweep_q']

        # Remove slope from phase (concatenate all tones)
        fcat = np.ravel(f.T)
        zcat = np.ravel(z.T)
        phicat = np.angle(zcat)
        slope = np.nanmedian(np.gradient(phicat, fcat))
        zcat *= np.exp(-1j * (slope * fcat))

        # Reformat as single-row arrays (like a single-tone sweep covering all frequencies)
        s['sweep_f'] = np.array([fcat])
        s['sweep_i'] = np.array([np.real(zcat)])
        s['sweep_q'] = np.array([np.imag(zcat)])
        s['sweep_ei'] = np.array([np.ravel(s['sweep_ei'].T)])
        s['sweep_eq'] = np.array([np.ravel(s['sweep_eq'].T)])
        
        # Add metadata
        s['wideband_sweep'] = True
        s['bandwidth_hz'] = bandwidth_hz
        s['center_freq_hz'] = center_freq_hz
        s['step_size_hz'] = step_size_hz
        s['num_tones_used'] = num_tones
        s['sweep_points_per_tone'] = sweep_points

        return s


    def set_tones_helper(self, freqs, amps=None, phases=None):
        if freqs is None or len(freqs) == 0:
            raise ValueError("Frequencies must be provided and cannot be empty.")
        if amps is None:
            amps = np.ones_like(freqs)
        if phases is None:
            phases = self.generate_newman_phases(freqs)

        self.set_tone_frequencies(freqs)
        self.set_tone_amplitudes(amps)
        self.set_tone_phases(phases)
        return

    def find_resonances(self, sweep_data=None, data_format='log_magnitude', 
                        filter_params=None, finder_params=None, **kwargs):
        """
        Find MKID resonances in sweep data using the peak_finder module.
        
        This is a convenience wrapper around the standalone peak_finder module
        that can work with sweep data directly from wideband_sweep() or from file.
        
        Args:
            sweep_data (dict, optional): Sweep data dictionary with keys 'sweep_f', 
                'sweep_i', 'sweep_q'. If None, performs a new wideband_sweep.
            data_format (str): Analysis format for peak finding. One of:
                'lin_magnitude', 'log_magnitude', 'phase', 'unwrapped_phase',
                'group_delay', 'complex_gradient'. Default is 'log_magnitude'.
            filter_params: FilterParams instance or dict with keys:
                - highpass_edge (float): 0-1 normalized (0 = disabled)
                - lowpass_edge (float): 0-1 normalized (1 = disabled)
                - median_kernel_size (int): 1 = disabled
            finder_params: PeakFinderParams instance or dict with keys:
                - prominence_enabled (bool), prominence_min/max (float)
                - width_enabled (bool), width_min/max (float) in Hz
                - distance_enabled (bool), distance_value (float) in Hz
                - peak_direction (int): -1 for dips, +1 for peaks
            **kwargs: Passed to wideband_sweep if sweep_data is None.
        
        Returns:
            list: List of ResonanceResult objects with attributes:
                - frequency (float): Resonance frequency in Hz
                - fwhm (float): Full width at half maximum in Hz
                - q_factor (float): Quality factor
                - qc (float): Coupling Q
                - qi (float): Internal Q
                - dip_depth (float): Depth of resonance dip in dB
        
        Example:
            >>> client = ReadoutClient()
            >>> # Find resonances from a new sweep
            >>> resonances = client.find_resonances()
            >>> print([r.frequency / 1e6 for r in resonances])  # MHz
            
            >>> # Find resonances from existing sweep data  
            >>> sweep = client.wideband_sweep()
            >>> resonances = client.find_resonances(sweep)
            
            >>> # Customize parameters
            >>> from souk_readout_tools.peak_finder import FilterParams, PeakFinderParams
            >>> fp = FilterParams(highpass_edge=0.001, lowpass_edge=0.5)
            >>> pp = PeakFinderParams(prominence_min=2.0, distance_value=50000)
            >>> resonances = client.find_resonances(filter_params=fp, finder_params=pp)
        """
        from ..peak_finder import (
            find_mkid_resonances, FilterParams, PeakFinderParams
        )
        
        # Perform sweep if no data provided
        if sweep_data is None:
            sweep_data = self.wideband_sweep(**kwargs)
        
        # Extract arrays from sweep data
        frequencies = np.ravel(sweep_data['sweep_f'])
        i_data = np.ravel(sweep_data['sweep_i'])
        q_data = np.ravel(sweep_data['sweep_q'])
        s21_complex = i_data + 1j * q_data
        
        # Convert dicts to dataclass instances if needed
        if isinstance(filter_params, dict):
            filter_params = FilterParams(**filter_params)
        if isinstance(finder_params, dict):
            finder_params = PeakFinderParams(**finder_params)
        
        # Find resonances
        resonances = find_mkid_resonances(
            frequencies=frequencies,
            s21_complex=s21_complex,
            data_format=data_format,
            filter_params=filter_params,
            finder_params=finder_params,
        )
        
        return resonances

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
        nargs="?",
        default=None,
        help="Path to config YAML (or .lnk). If omitted, uses default_config.lnk for the specified pipeline.",
    )
    parser.add_argument(
        "-p", "--pipeline",
        type=int,
        default=None,
        choices=[0, 1],
        help="Pipeline ID (0 or 1). If omitted, extracted from config file or defaults to 0.",
    )
    args = parser.parse_args()
    
    client = ReadoutClient(config_file=args.config, pipeline_id=args.pipeline)
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
