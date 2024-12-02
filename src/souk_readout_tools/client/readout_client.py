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
import numpy as np
import yaml
import sys
import time
import os
import traceback
import csv
import base64
from scipy import signal

USER_CALIBRATIONS_DIR = os.path.expanduser('~/.souk_readout_tools/calibrations')
USER_CONFIG_DIR = os.path.expanduser('~/.souk_readout_tools/config')
USER_TMP_DIR = os.path.expanduser('~/.souk_readout_tools/tmp')

DEFAULT_CONFIG = os.path.join(USER_CONFIG_DIR,'default_config.lnk')

class ReadoutClient:
    def __init__(self,config_file=None):
        if config_file is None:
            config_file = DEFAULT_CONFIG
            print(f'Loading default config file from {DEFAULT_CONFIG}')
        if not os.path.exists(config_file):
            #print(f'Config file not found {config_file}, searching in {USER_CONFIG_DIR}')
            config_file = os.path.join(USER_CONFIG_DIR,config_file)
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        if type(config) is str:
            config_file = config
            if not os.path.exists(config_file):
                #print(f'Linked config file not found {config_file}, searching in {USER_CONFIG_DIR}')
                config_file = os.path.join(USER_CONFIG_DIR,config_file)
            #file is a link, try again using the link contents as the config file path
            with open(config_file,'r') as file:
                config = yaml.safe_load(file)
        print(f'Config file loaded: {config_file}')
        with open(DEFAULT_CONFIG,'w') as file:
            file.write(os.path.abspath(config_file))
        self.config = config
        self.config_file = config_file
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

    def initialise_server(self,config_file=None):
        message = {'request': 'initialise_server','config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()

        return response

    def initialise_firmware(self,config_file=None):
        message = {'request': 'initialise_firmware', 'config_filename': config_file}
        response = self.send_request(message)
        if response['status'] == 'success':
            self.pull_config()
        return response

    def pull_config(self,destination_dir=None):
        if destination_dir is None:
            destination_dir = USER_CONFIG_DIR
        message = {'request': 'pull_config'}
        response = self.send_request(message)
        if response['status'] == 'success':
            config_filename = response['config_filename']
            config_contents = response['config_contents']
            destination_file = os.path.join(destination_dir,os.path.basename(config_filename))
            with open(destination_file,'w') as file:
                file.write(config_contents)
            print(f'Config file pulled from RFSoC into {destination_file}')
            self.config_file = destination_file
            self.config = yaml.safe_load(config_contents)
            print(f'Config loaded {self.config_file}')
            with open(DEFAULT_CONFIG,'w') as file:
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

    def pull_calibration(self,remote_file,destination_file=None):
        if destination_file is None: 
            destination_file = os.path.join(USER_CALIBRATIONS_DIR,os.path.basename(remote_file))
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
        return self.initialise_firmware()

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

    def perform_retune(self, centers, spans, points, samples_per_point, direction='up',method='max_gradient'):
        #need to check the tones can be set otherwise the sweep task in the server will fail silently
        response = self.set_tone_frequencies(centers)
        if response['status'] != 'success':
            print(f"Error setting tone frequencies: {response['message']}")
            return response
        centers=np.atleast_1d(centers)
        spans=np.atleast_1d(spans)

        message = {
            'request': 'retune',
            'centers': centers,
            'spans': spans,
            'points': points,
            'samples_per_point': samples_per_point,
            'direction': direction,
            'method': method
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

    def parse_sweep_data(self,sweep_data, apply_phase_correction=True):
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
                
                udc = self.config['rf_frontend']['connected']
                lo = self.config['rf_frontend']['tx_mixer_lo_frequency_hz'] if udc else 0.0
                sb = self.config['rf_frontend']['tx_mixer_sideband'] if udc else 1
                
                adcclk = info['adc_clk_hz']
                dacclk = adcclk
                dacduc = info['dac_duc_mixer_frequency_hz']
                dacnyq = info['nyquist_zone_dac0']
                txnfft = 8192
                rxnfft = 8192

                rffreqs = sweep_f
                iffreqs = sb*(rffreqs-lo)
                bbfreqs = iffreqs - dacduc
                bbbins  = bbfreqs/(dacclk)*txnfft
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
                        if key=='date':
                            value = value
                        elif value.startswith('"') and value.endswith('"'):
                            value = eval(value[1:-1])
                        else:
                            try:
                                value = eval(value)
                            except NameError:
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


    def receive_stream(self, num_tones=2048, filename=None,print_data=False):
        data = bytearray(2048*2*4 + 10*4)
        view = memoryview(data)
        iq_data=None
        if filename is None:
            filename =os.path.join(USER_TMP_DIR,'tmp_stream')
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


    def receive_triggered_stream(self, num_tones=2048, filename=None,print_data=False):
        data = bytearray(4096*4 + 10*4)
        view = memoryview(data)
        if filename is None:
            filename =os.path.join(USER_TMP_DIR,'tmp_triggered_stream')
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
            window_samples = max(3,int(smooth_window_hz/(sweep_frequencies[1]-sweep_frequencies[0])))
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
    


if __name__=='__main__':
    config_file = sys.argv[1]
    if not config_file:
        print(f'No config file specified. Using default from {DEFAULT_CONFIG}.')
        config_file = DEFAULT_CONFIG
    client = ReadoutClient(config_file=config_file)
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
