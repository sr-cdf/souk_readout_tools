#!/usr/bin/env python3

"""
Readout server for MKID firmware

This server runs on the RFSoC ARM processor and provides an interface for external clients to interact with the firmware.

The server can be used to get and set parameters, initialize the firmware, enable and disable streaming, and perform sweeps and retuning.

The server should be run as a standalone script on the RFSoC ARM processor. It can be daemonized using the systemd service file provided.

Requires python3.8

Example usage:
    ssh rfsoc_host
    cd src/readout_server
    sudo python readout_server.py config/config.yaml
    
Author: Sam Rowe
Date: July 2024 
Version: 0.1

"""

def check_if_running_on_rfsoc_arm():
    """
    Check if the script is running on the RFSoC ARM processor.
    Looks for "xilinx" in output of uname.
    """
    import os
    if 'xilinx' not in os.uname().release:
        print('This script should only be run on the RFSoC ARM processor')
        raise RuntimeError('This script should only be run on the RFSoC ARM processor')
    
def set_process_name():
    """
    Set the process name for the current process.
    Useful for identifying the process in the output of 'top' or 'ps caux'.
    """
    import os
    import ctypes
    process_name = os.getenv('READOUT_SERVER_NAME', 'readout_server')
    libc = ctypes.CDLL('libc.so.6')
    libc.prctl(15, ctypes.c_char_p(process_name.encode('utf-8')), 0, 0, 0)
    return process_name

def get_host_ips():
    """
    Get the IP addresses of the host machine.
    """
    from subprocess import check_output
    host_ips = check_output(['hostname', '--all-ip-addresses']).strip().decode()
    return host_ips



import asyncio, contextvars, functools
import json
import struct
import yaml
import socket
import os
import traceback
import sys
print(sys.version)
import numpy as np

import firmware_lib

import time

#STREAM FLAGS 
FLAG_SERVER_REQUEST = 0
FLAG_SET_FREQS = 1
FLAG_SET_AMPS = 2
FLAG_SET_PHASES = 3
FLAG_CAL_FREEZE = 4
FLAG_5 = 5
FLAG_6 = 6
FLAG_7 = 7


class ReadoutServer:
    """
    This server provides an interface for external clients to interact with the RFSoC firmware.
    
    Connected clients can get and set parameters, enable and disable streaming, and perform sweeps and retuning.

    There is a request server that responds to client requests and a stream server that streams data to clients.
    
    Tasks for continuous data streaming and triggered streaming are started automatically and enabled/disabled by request.
    """

    def __init__(self, config_file):
        """
        Initialize the readout server.
        The server will load the specified configuration file and create the firmware interface.
        The server will then start the request server and stream server, and create tasks for streaming and triggered streaming.
        """
        check_if_running_on_rfsoc_arm()

        #server attributes
        self.config = None
        self.request_clients = []
        self.stream_clients = []
        self.sweep_task = None
        self.stream_task = None
        self.triggered_stream_task = None
        self.stream_enabled = asyncio.Event()
        self.triggered_stream_enabled = False
        self.tasks = []
        self.fake_trigger_event = asyncio.Event()
        self.stream_flags = [asyncio.Event() for _ in range(8)]

        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.latest_sweep_data = {'f':None,'z':None,'e':None}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0

        #initialize server
        self.config_file = config_file
        self.init_server(self.config_file)
        

    def load_config(self, config_file):
        """
        Load a new configuration file.
        Does not reload the firmware.
        """
        if not config_file:
            print('No config file specified, reverting to default: config/config.yaml')
            config_file = 'config/config.yaml'
            
        # if not os.path.exists(config_file):
        #     print(f'Config file "{config_file}" does not exist, reverting to default: config/config.yaml')
        #     config_file = 'config/config.yaml'
            
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)

        print(yaml.dump(config,sort_keys=False))
        self.config_file = config_file
        self.config = config
        return config
    
    def save_config(self, config_filename, config_contents):
        """
        Save a configuration file.
        """
        with open(config_filename, 'w') as file:
            file.write(config_contents)
        os.chmod(config_filename, 0o666)
        return
    
    def set_default_config(self,config_filename):
        """
        Set the default config file by linking the given filename to config/config.yaml
        """
        os.unlink('config/config.yaml')
        os.symlink(config_filename.split('/')[-1], 'config/config.yaml')       
        
    def init_server(self, config_file):
        """
        Initialize the server with the specified configuration file.
        This function is called when the server is started or reset, and when the firmware is reprogrammed.
        """

        #server attributes
        self.config = None
        self.request_clients = []
        self.stream_clients = []
        self.sweep_task = None
        self.stream_task = None
        self.triggered_stream_task = None
        self.stream_enabled = asyncio.Event()
        self.triggered_stream_enabled = False
        self.tasks = []
        self.stream_flags = [asyncio.Event() for _ in range(8)]


        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.latest_sweep_data = {'f':None,'z':None,'e':None}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0
        
        #load config
        self.config_file = config_file
        self.load_config(config_file)
        self.server_address = '0.0.0.0'
        self.request_server_port = self.config['rfsoc_host']['request_port']
        self.stream_server_port = self.config['rfsoc_host']['stream_port']
        self.trigger_source_pin = self.config['rfsoc_host']['trigger_source_pin']
        
        #interface with firmware
        fw_config_file = self.config['firmware']['fw_config_file']
        self.r = firmware_lib.create_standard_readout_interface(fw_config_file)
        self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file)
        
        #program firmware if necessary
        if firmware_lib.needs_programming(self.r):
            self.init_firmware()
        
        system_information = self.get_system_information()
        print(system_information)
        return
   
             
    def init_firmware(self,config_file=None):
        """
        Restart the firmware.
        This function is called when the firmware needs to be reprogrammed or reinitialised.
        """
        if config_file is None:
            #simple reload of the same firmware, eg: hard reset
            self.r, self.r_fast = firmware_lib.reload_firmware(self.config)
            return
        else:
            #reload with new firmware, reinitialise server, eg: firmware update
            self.load_config(config_file)
            r, r_fast = firmware_lib.reload_firmware(self.config)
            self.init_server(self.config_file)
            return

    def get_system_information(self):
        """
        Get the system information from the firmware.
        This function is called when a client sends a 'get_system_information' request.
        Not properly implemented yet.
        """

        system_information = firmware_lib.get_system_information(self.r,self.config)
        return system_information
    
    def get_server_status(self):
        """
        Get the status of the server.
        This function is called when a client sends a 'server_status' request.
        """

        status = {
            'process_name': process_name,
            'ip_addresses': host_ips,
            'pwd': os.getcwd(),
            'sys.executable': sys.executable,
            'sys.argv': sys.argv,
            'uname': os.uname().nodename+' '+os.uname().sysname+' '+os.uname().release + ' ' + os.uname().version + ' ' + os.uname().machine,
            'python_version': sys.version,
            'config_file': self.config_file,
            'request_clients': len(self.request_clients),
            'request_client_addrs': [client.get_extra_info('peername') for client in self.request_clients],
            'stream_clients': len(self.stream_clients),
            'stream_client_addrs': [client.get_extra_info('peername') for client in self.stream_clients],
            'stream_task_started': self.stream_task is not None and not self.stream_task.done(),
            'stream_enabled': self.stream_enabled.is_set(),
            'triggered_stream_task_started': self.triggered_stream_task is not None and not self.triggered_stream_task.done(),
            'triggered_stream_enabled': self.triggered_stream_enabled,
            'sweep_task_running': self.sweep_task is not None and not self.sweep_task.done(),
            'tasks': len(self.tasks),
            'firmware_interface_alive': bool(self.r),
            'firmware_fast_interface_alive': bool(self.r_fast),
            'system_information': self.get_system_information(),
            'latest_sweep_data_valid': self.latest_sweep_data_valid
        }
        return status
    
    async def handle_request_client(self, reader, writer):
        """
        Handle a request client connection.
        This function is called when a new client connects to the request server.
        The client can send requests to the server to get and set parameters, initialize the firmware, enable and disable streaming, and perform sweeps and retuning.
        """
        addr = writer.get_extra_info('peername')
        print(f"request client connected: {addr}")
        self.request_clients.append(writer)

        try:
            while True:
                raw_msglen = await reader.readexactly(4)
                if not raw_msglen:
                    break
                self.stream_flags[FLAG_SERVER_REQUEST].set()
                await asyncio.sleep(0)
                
                msglen = struct.unpack('>I', raw_msglen)[0]

                data = await reader.readexactly(msglen)
                print(data)
                message = json.loads(data.decode())
                request = message.get('request')

                if request == 'server_status':
                    status = self.get_server_status()
                    await self.send_response(writer, {'status': 'success', 'message': status})

                elif request == 'initialise_server':
                    config_file = message.get('config_filename')
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_server(config_file)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'initialise_firmware':
                    config_file = message.get('config_filename')
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        await self.send_response(writer, {'status': 'success'})
                        self.init_firmware(config_file) #implicitly calls init server
                
                elif request == 'save_config':
                    config_filename = message.get('config_filename')
                    config_contents = message.get('config_contents')
                    self.save_config(config_filename, config_contents)
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'get_config':
                    await self.send_response(writer, {'status': 'success', 'config_filename': self.config_file, 'config_contents': yaml.dump(self.config,sort_keys=False)})

                elif request == 'set_default_config':
                    config_filename = message.get('config_filename')
                    self.set_default_config(config_filename)
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'get_system_information':
                    info = self.get_system_information()
                    await self.send_response(writer, {'status': 'success', 'data': info})

                elif request == 'get':
                    param_name = message.get('param')
                    response = {'status': 'error', 'message': f'Invalid parameter name {param_name}'}
                    if param_name == 'sample_rate_hz':
                        value = firmware_lib.get_sample_rate(self.r)
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'tone_frequencies':
                        value = firmware_lib.get_tone_frequencies(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_frequencies_detailed':
                        value = firmware_lib.get_tone_frequencies(self.r,self.config,detailed_output=True)[1]
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'tone_amplitudes':
                        value = firmware_lib.get_tone_amplitudes(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_phases':
                        value = firmware_lib.get_tone_phases(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_powers':
                        value = firmware_lib.get_tone_powers(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_powers_detailed':
                        value = firmware_lib.get_tone_powers(self.r,self.config,detailed_output=True)[1]
                        response = {'status': 'success', 'value': value}    
                    elif param_name == 'cal_freeze':
                        value = firmware_lib.get_cal_freeze(self.r,self.config)
                        if value:
                            self.stream_flags[FLAG_CAL_FREEZE].set()
                        else:
                            self.stream_flags[FLAG_CAL_FREEZE].clear()
                        response = {'status': 'success', 'value': value}
                    
                    await self.send_response(writer, response)

                elif request == 'set':
                    param_name = message.get('param')
                    param_value = message.get('value')
                    response = {'status': 'error', 'message': f'Invalid parameter name {param_name}'}
                    if param_name == 'sample_rate_hz':
                        firmware_lib.set_sample_rate(self.r, param_value)
                        response = {'status': 'success'}
                    
                    elif param_name == 'tone_frequencies':
                        self.stream_flags[FLAG_SET_FREQS].set()
                        await asyncio.sleep(0)
                        firmware_lib.set_tone_frequencies(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_FREQS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}
                    
                    elif param_name == 'tone_amplitudes':
                        self.stream_flags[FLAG_SET_AMPS].set()
                        await asyncio.sleep(0)
                        firmware_lib.set_tone_amplitudes(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_AMPS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}
                    
                    elif param_name == 'tone_phases':
                        self.stream_flags[FLAG_SET_PHASES].set()
                        await asyncio.sleep(0)
                        firmware_lib.set_tone_phases(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_PHASES].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}

                    elif param_name == 'tone_powers':
                        self.stream_flags[FLAG_SET_AMPS].set()
                        await asyncio.sleep(0)
                        firmware_lib.set_tone_powers(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_AMPS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}


                    elif param_name == 'cal_freeze':
                        if param_value:
                            self.stream_flags[FLAG_CAL_FREEZE].set()
                        else:
                            self.stream_flags[FLAG_CAL_FREEZE].clear()
                        await asyncio.sleep(0)
                        firmware_lib.set_cal_freeze(self.r, self.config, param_value)
                        response = {'status': 'success'}
                    
                    await self.send_response(writer, response)


                elif request == 'enable_stream':
                    self.triggered_stream_enabled = False
                    self.stream_enabled.set()
                    print('Stream enabled, clients:', self.stream_clients)
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'disable_stream':
                    self.stream_enabled.clear()
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'enable_triggered_stream':
                    self.stream_enabled = False
                    self.triggered_stream_enabled = True
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'disable_triggered_stream':
                    self.triggered_stream_enabled = False
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'send_fake_trigger':
                    self.fake_trigger_event.set()
                    await self.send_response(writer, {'status': 'error', 'message': 'Not implemented'})

                elif request == 'check_input_saturation':
                    iterations = message.get('iterations')
                    result,details = firmware_lib.check_input_saturation(self.r,iterations=iterations)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})
                
                elif request == 'check_output_saturation':
                    iterations = message.get('iterations')
                    result,details = firmware_lib.check_output_saturation(self.r,iterations=iterations)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})

                elif request == 'check_dsp_overflow':
                    duration_s = message.get('duration_s')
                    result,details = firmware_lib.check_dsp_overflow(self.r,duration_s=duration_s)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})

                elif request == 'get_samples':
                    num_samples = message.get('num_samples')
                    task = asyncio.create_task(self.get_samples(writer, num_samples))
                    self.tasks.append(task)
                
                elif request == 'sweep':
                    if self.sweep_task is None or self.sweep_task.done():
                        centers = message.get('centers')
                        spans = message.get('spans')
                        points = message.get('points')
                        samples_per_point = message.get('samples_per_point')
                        direction = message.get('direction')
                        #print('asyncio create task, sweep task')
                        self.sweep_task = asyncio.create_task(
                            self.sweep(centers, spans, points, samples_per_point, direction)
                        )
                        
                        #print('await send response')
                        await self.send_response(writer, {'status': 'success', 'message': 'Sweep in progress'})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'Sweep already in progress'})
                
                elif request == 'get_sweep_progress':
                    await self.send_response(writer, {'status': 'success', 'progress': self.sweep_progress})

                elif request == 'get_sweep_data':
                    if self.latest_sweep_data_valid:
                        sweep_f = self.latest_sweep_data['f']
                        sweep_z = self.latest_sweep_data['z']
                        sweep_e = self.latest_sweep_data['e']
                        # data = '#Sweep file\n'
                        # data = f'# date: {time.strftime("%Y-%m-%d %H:%M:%S %Z")}\n'
                        # data += f'# num_tones: {len(sweep_f)}\n'
                        # data += f'# num_points: {len(sweep_f[0])}\n'
                        # data += '# system_info: "+str(self.get_system_information())'
                        # data += '#' + ' '.join([f'sweep_f_{k:04d} sweep_i_{k:04d} sweep_q_{k:04d} err_i_{k:04d} err_q_{k:04d}' for k in range(len(sweep_f))]) + '\n'

                        # for j in range(len(sweep_f[0])):
                        #     for i in range(len(sweep_f)):
                        #         data += f'{sweep_f[i][j]:.6f} {sweep_z[i][j].real:.6f} {sweep_z[i][j].imag:.6f} {sweep_e[i][j].real:.6f} {sweep_e[i][j].imag:.6f} '
                        #     data += '\n'
                        
                        sweep = {}
                        for i in range(len(sweep_f)):
                            tone={}
                            tone['f'] = sweep_f[i].tolist()
                            tone['i'] = sweep_z[i].real.tolist()
                            tone['q'] = sweep_z[i].imag.tolist()
                            tone['ei'] = sweep_e[i].real.tolist()
                            tone['eq'] = sweep_e[i].imag.tolist()
                            sweep[f'{i:04d}'] = tone

                        data={'date': time.strftime("%Y-%m-%d %H:%M:%S UTC%z"),
                              'num_tones': len(sweep_f),
                              'num_points': len(sweep_f[0]),
                              'sweep': sweep,
                              'system_information': self.get_system_information()}
                        

                        await self.send_response(writer, {'status': 'success', 'data': data})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'No valid sweep data available yet, please perform a sweep first'})
                
                elif request == 'get_sweep_txt':
                    if self.latest_sweep_data_valid:
                        sweep_f = self.latest_sweep_data['f']
                        sweep_z = self.latest_sweep_data['z']
                        sweep_e = self.latest_sweep_data['e']
                        data = '# Sweep file\n'
                        data += f'# date: {time.strftime("%Y-%m-%d %H:%M:%S %Z")}\n'
                        data += f'# num_tones: {len(sweep_f)}\n'
                        data += f'# num_points: {len(sweep_f[0])}\n'
                        data += '# system_info: '+str(self.get_system_information()) +'\n'
                        data += '#' + ' '.join([f'sweep_f_{k:04d} sweep_i_{k:04d} sweep_q_{k:04d} err_i_{k:04d} err_q_{k:04d}' for k in range(len(sweep_f))]) + '\n'

                        for j in range(len(sweep_f[0])):
                            for i in range(len(sweep_f)):
                                data += f'{sweep_f[i][j]:.6f} {sweep_z[i][j].real:.6f} {sweep_z[i][j].imag:.6f} {sweep_e[i][j].real:.6f} {sweep_e[i][j].imag:.6f} '
                            data += '\n'
                        

                        await self.send_response(writer, {'status': 'success', 'data': data})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'No valid sweep data available yet, please perform a sweep first'})

                elif request == 'retune':
                    if self.sweep_task is None or self.sweep_task.done():
                        centers = message.get('centers')
                        spans = message.get('spans')
                        points = message.get('points')
                        samples_per_point = message.get('samples_per_point')
                        direction = message.get('direction')
                        method = message.get('method')
                        self.sweep_task = asyncio.create_task(
                            self.retune(centers, spans, points, samples_per_point, direction, method)
                        )
                        await self.send_response(writer, {'status': 'success', 'message': 'Retune in progress'})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'Sweep or retune already in progress'})

                elif request == 'cancel':
                    if self.sweep_task:
                        self.sweep_task.cancel()
                        self.sweep_task = None
                    for task in self.tasks:
                        task.cancel()
                    self.tasks = []
                    await self.send_response(writer, {'status': 'success'})

                else:
                    await self.send_response(writer, {'status': 'error', 'message': f'Invalid request {request}'})
                
                self.stream_flags[FLAG_SERVER_REQUEST].clear()
                await asyncio.sleep(0)
                

        except asyncio.IncompleteReadError:
            print(f"request client disconnected: {addr} (IncompleteReadError)")
        except Exception as e:
            print(f"Error handling request client {addr}: {e}")
            print(traceback.format_exc())
            await self.send_response(writer, {'status': 'error', 'message': f'Error handling request: {e}'})
        finally:
            if writer in self.request_clients:
                self.request_clients.remove(writer)
            writer.close()
            await writer.wait_closed()

    def stream_keepalive(self, sock, after_idle_sec=1, interval_sec=3, max_fails=5):
        """
        Enable TCP keepalive on an open socket.
        If the connection is idle for after_idle_sec seconds, start sending keepalive packets every interval_sec seconds.
        If max_fails keepalive packets are sent with no response, the connection is considered dead.
        If the connection is dead, the next operation on the socket will raise an exception, the socket will be closed, and the client will be disconnected (removed from the list of clients).
        """
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPIDLE, after_idle_sec)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPINTVL, interval_sec)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_KEEPCNT, max_fails)


    async def handle_stream_client(self, reader, writer):
        """
        Handle a new stream client connection.
        Data streams will be sent to all connected stream clients.
        Simply keep the stream alive by sending zero length frames every second.
        Clients should receive that the length is zero and not try to unpack the frame.
        """
        addr = writer.get_extra_info('peername')
        print(f"Stream client connected: {addr}")
        sock = writer.get_extra_info('socket')
        self.stream_keepalive(sock) # Enable TCP keepalive so we can detect when the client disconnects
        writer.addr = addr
        self.stream_clients.append(writer)

        while writer in self.stream_clients:
            try:
                writer.write(b'\x00\x00\x00\x00')
                await writer.drain()
                await asyncio.sleep(1)
            except ConnectionResetError:
                print(f"Stream client disconnected: {addr} (ConnectionResetError)")
                self.stream_clients.remove(writer)
                break


    async def send_response(self, writer, response):
        """
        Send a response to a client in JSON format.
        """
        try:
            response_data = json.dumps(response).encode()
            response_len = struct.pack('>I', len(response_data))
            writer.write(response_len + response_data)
            await writer.drain()
        except Exception as e:
            print(f"Error sending response: {response} \n {e}")
            print(traceback.format_exc())
    
    def prepare_frame(self,fast_read_params):
        """
        Prepare a frame for sending to a client.
        """
        num_headers = 10
        # # cnt = await firmware_lib._wait_for_acc(fast_read_params['acc'],0.0001)
        # cnt = firmware_lib._wait_for_acc(fast_read_params['acc'],0.0001)

        cnt,data,err = firmware_lib.read_accumulated_data_fast(fast_read_params)
       
        # frame=data
        frame = np.zeros(len(data)+num_headers,dtype='<i4')
        frame[:len(data)] = data
        frame[-1] = err
        frame[-2] = cnt
        frame[-3] = int(self.stream_flags[7].is_set())
        frame[-4] = int(self.stream_flags[6].is_set())
        frame[-5] = int(self.stream_flags[5].is_set())
        frame[-6] = int(self.stream_flags[FLAG_CAL_FREEZE].is_set())
        frame[-7] = int(self.stream_flags[FLAG_SET_PHASES].is_set())
        frame[-8] = int(self.stream_flags[FLAG_SET_AMPS].is_set())
        frame[-9] = int(self.stream_flags[FLAG_SET_FREQS].is_set())
        frame[-10] = int(self.stream_flags[FLAG_SERVER_REQUEST].is_set())
        
        data_bytes = frame.tobytes()

        data_len = struct.pack('>I', len(data_bytes))

        payload = data_len+data_bytes

        # return payload
        return payload,cnt,err
    

    async def get_samples(self, writer, num_samples):
        
        """
        A coroutine that gets a fixed number of samples from the firmware and sends them to a client through the request channel.
        """
        
        try:
            fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
            err_count=0
            prev_cnt=0
            for _ in range(num_samples):
                #cnt,data,err = firmware_lib.read_accumulated_data_fast(self.r_fast,fast_read_params)
                # # data_bytes = data.tobytes()
                payload, cnt, err =  self.prepare_frame(fast_read_params)
                writer.write(payload)
                await writer.drain()
                # await asyncio.sleep(0.0001)  
            print('total packet counter errors:', err_count)
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error getting samples: {e}")
            print(traceback.format_exc())
        finally:
            self.tasks.remove(asyncio.current_task())
            writer.close()
            await writer.wait_closed()
        



    async def stream_data(self):
        """
        This coroutine runs as an asynchronous task and will stream data to all connected clients through the stream channel.
        Streaming is enabled by setting self.stream_enabled to True.
        """
        err_count=0
        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
        prev_cnt=0
        while True:
            try:
                if self.stream_enabled.is_set():
                    payload, cnt, err =  self.prepare_frame(fast_read_params)
                    if err:
                        err_count+=1
                        print('Packet error:',cnt, err_count)
                    for client in self.stream_clients:
                        try:
                            client.write(payload)
                            await client.drain()
                        except ConnectionResetError:
                            print(f"Stream client disconnected {client.addr} (ConnectionResetError)")
                            self.stream_clients.remove(client)
                        except Exception as e:
                            print(f"Error streaming data to client {client.addr}: {e}")
                            print(traceback.format_exc())
                            self.stream_clients.remove(client)
                    await asyncio.sleep(0) # release control to other tasks
                else:
                    await asyncio.sleep(0.1) # do nothing
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error streaming data: {e}")
                print(traceback.format_exc())


    async def triggered_stream(self):
        """
        This coroutine runs as an asynchronous task and will stream data to all connected clients through the stream channel when a trigger is received on GPIO.
        Triggered treaming is enabled by setting self.triggered_stream_enabled to True.
        The GPIO port is specified in the configuration file.
        """
        # err_count=0
        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
        while True:
            try:
                if self.triggered_stream_enabled:
                    triggered = await self.to_thread(firmware_lib.wait_for_gpio_pulse,self.r, self.trigger_source_pin,self.fake_trigger_event)
                    
                    if triggered:

                        # cnt,data,err = firmware_lib.read_accumulated_data_fast(self.r_fast,fast_read_params)
                        payload, cnt, err =  self.prepare_frame(fast_read_params)

                        # #data_bytes = data.tobytes()
                        # if err:
                        #     err_count+=1
                        #     print('Packet count error:',cnt)

                        for client in self.stream_clients:
                            try:
                                client.write(payload)
                                await client.drain()
                            except ConnectionResetError:
                                print(f"Stream client disconnected {client.addr} (ConnectionResetError)")
                                self.stream_clients.remove(client)
                            except Exception as e:
                                print(f"Error sending triggered data to client: {e}")
                                print(traceback.format_exc())
                                self.stream_clients.remove(client)
                    
                    await asyncio.sleep(0)  
                else:
                    await asyncio.sleep(0.1)
            except asyncio.CancelledError:
                break
            except Exception as e:
                print(f"Error waiting for trigger: {e}")
                print(traceback.format_exc())
                await asyncio.sleep(0.1)

    async def sweep(self, centers, spans, points, samples_per_point, direction):
        """
        A coroutine that performs a frequency sweep and stores the results in self.latest_sweep_data.
        """
        
        try:
            
            self.latest_sweep_data_valid = False
            
            centers = np.atleast_1d(centers)
            spans = np.atleast_1d(spans)
            if len(spans)==1:
                spans = np.full(len(centers),spans[0])
            assert len(centers) == len(spans)
            num_points=int(points)
            samples_per_point=int(samples_per_point)
            assert direction in ('up','down')

            num_tones = len(centers) 
            channels = np.arange(num_tones,dtype=int)
            sweepfreqs = np.zeros((num_tones,num_points),dtype=float)
            for t in range(num_tones):
                cf=centers[t]
                sp=spans[t]
                sweepfreqs[t] = np.linspace(cf-sp/2.,cf+sp/2.,num_points)
                if direction=='down':
                    sweepfreqs[t] = sweepfreqs[t][::-1]

            acc_counts = np.zeros((num_points,samples_per_point),dtype=int)
            sweep_data = np.zeros((num_tones,num_points,samples_per_point),dtype=complex)
            acc_errs = np.zeros((num_points,samples_per_point),dtype=bool)

            # after the sweep, the tones should be set back to their original frequencies.
            # unless the number of tones has changed, or the tones were not within the span of the sweep.
            # in which case they should be set to the center frequencies.
            initial_freqs = firmware_lib.get_tone_frequencies(self.r, self.config)
            print('initial freqs:',initial_freqs)
            if len(initial_freqs)!=len(centers):
                initial_freqs = centers
                print('initial freqs updated:',initial_freqs)
            for i in range(len(centers)):
                if initial_freqs[i]<centers[i]-spans[i]/2. or initial_freqs[i]>centers[i]+spans[i]/2.:
                    initial_freqs[i] = centers[i]
                    print('initial freqs updated2:',initial_freqs)
            firmware_lib.set_tone_frequencies(self.r,self.config,centers,autosync=True)
            print('initial_freqs:',initial_freqs)

            fast_read_params = firmware_lib.get_fast_read_params(self.r_fast) 
            for p in range(num_points):
                print('sweeping: setting tone frequencies',sweepfreqs[:,p])
                firmware_lib.set_tone_frequencies(self.r,
                                    self.config,
                                    sweepfreqs[:,p],
                                    autosync=True)
                print('sweeping: getting_samples')
                for s in range(samples_per_point):
                    cnt,data,err = firmware_lib.read_accumulated_data_fast(
                                                            fast_read_params,
                                                            num_tones=num_tones)
                    acc_counts[p,s] = cnt
                    sweep_data[:,p,s] = data[::2]+1j*data[1::2]
                    acc_errs[p,s] = err
                self.sweep_progress = float(p/(num_points-1))
                await asyncio.sleep(0.001)

            print('reset initial freqs:',initial_freqs)
            firmware_lib.set_tone_frequencies(self.r,self.config,initial_freqs,autosync=True)

            sweep_responses = np.mean(sweep_data.real,axis=2) + 1j*np.mean(sweep_data.imag,axis=2)
            sweep_stds = np.std(sweep_data.real,axis=2) + 1j*np.std(sweep_data.imag,axis=2)

            results = {
                'sweep_frequencies': sweepfreqs,
                'sweep_responses': sweep_responses,
                'sweep_stds': sweep_stds,
                'samples_per_point': samples_per_point,
                'samples_per_second': firmware_lib.get_sample_rate(self.r_fast),
                'accumulation_counts': acc_counts,
                'accumulation_errors': acc_errs
                }
            # results = firmware_lib.perform_sweep(self.r,self.r_fast, self.config, center, span, points, samples_per_point, direction)
            self.latest_sweep_data['f'] = results['sweep_frequencies']
            self.latest_sweep_data['z'] = results['sweep_responses']
            self.latest_sweep_data['e'] = results['sweep_stds']
            self.latest_sweep_data_valid = True

        except asyncio.CancelledError:
            print('ayncio cancelled')
            firmware_lib.set_tone_frequencies(self.r,self.config,initial_freqs,autosync=True)

            pass
        except Exception as e:
            print(f"Error performing sweep: {e}")
            print(traceback.format_exc())
        
    async def retune(self, center, span, points, samples_per_point, direction, method):
        """
        A coroutine that performs a frequency sweep, finds the resonance peaks using the specified method, and sets the tones to the peak frequencies.
        """
        try:
            if method not in ('max_gradient','min_mag'):
                raise ValueError(f'Invalid retune method "{method}", must be "max_gradient" or "min_mag"')
            
            await self.sweep(center, span, points, samples_per_point, direction)

            sweep_f = self.latest_sweep_data['f']
            sweep_z = self.latest_sweep_data['z']
            sweep_e = self.latest_sweep_data['e']

            retune_freqs = np.zeros_like(sweep_f[:,0])

            if method == 'max_gradient':
                for t in range(len(center)):
                    freqs = sweep_f[t]
                    grads = np.abs(np.gradient(sweep_z[t]))
                    max_grad = np.argmax(grads)
                    retune_freqs[t] = freqs[max_grad]
            elif method == 'min_mag':
                for t in range(len(center)):
                    freqs = sweep_f[t]
                    mags = np.abs(sweep_z[t])
                    min_mag = np.argmin(mags)
                    retune_freqs[t] = freqs[min_mag]

            print('Retune freqs:',retune_freqs)

            firmware_lib.set_tone_frequencies(self.r,self.config,retune_freqs)
            print('New frequencies:',firmware_lib.get_tone_frequencies(self.r,self.config))

            # results = firmware_lib.perform_retune(self.r,self.r_fast, self.config, center, span, points, samples_per_point, direction, method)
            # self.latest_sweep_data['f'] = results['sweep_frequencies']
            # self.latest_sweep_data['z'] = results['sweep_responses']
            # self.latest_sweep_data['e'] = results['sweep_stds']
            # self.latest_sweep_data_valid = True

        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error performing retune: {e}")
            print(traceback.format_exc())

    async def to_thread(self, func, /, *args, **kwargs):
        """
        Run a synchronous function in a separate thread.
        In later python versions, this can be replaced with asyncio.to_thread.
        """
        loop = asyncio.get_running_loop()
        ctx = contextvars.copy_context()
        func_call = functools.partial(ctx.run, func, *args, **kwargs)
        return await loop.run_in_executor(None, func_call)

    async def main(self):
        """
        Main function that starts the request server, stream server, and tasks for streaming and triggered streaming.
        """
        request_server = await asyncio.start_server(self.handle_request_client, self.server_address, self.request_server_port)
        stream_server = await asyncio.start_server(self.handle_stream_client, self.server_address, self.stream_server_port)
        
        print('Request server serving on', request_server.sockets[0].getsockname())
        print('Stream server serving on', stream_server.sockets[0].getsockname())
        
        self.stream_task = asyncio.create_task(self.stream_data())
        self.triggered_stream_task = asyncio.create_task(self.triggered_stream())
        
        async with request_server, stream_server:
            await asyncio.gather(
                request_server.serve_forever(),
                stream_server.serve_forever()
            )

if __name__ == '__main__':

    check_if_running_on_rfsoc_arm()
    process_name = set_process_name()
    host_ips = get_host_ips()

    readout_server = ReadoutServer('config/config.yaml')
    asyncio.run(readout_server.main())
