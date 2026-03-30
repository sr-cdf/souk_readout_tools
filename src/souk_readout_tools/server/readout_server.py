#!/usr/bin/env python3

"""
Readout server for MKID firmware

This server runs on the RFSoC ARM processor and provides an interface for external clients to interact with the firmware.

The server can be used to get and set parameters, initialize the firmware, enable and disable streaming, and perform sweeps and retuning.

The server should be run as a standalone script on the RFSoC ARM processor. It can be daemonized using the systemd service file provided.

Requires python3.8

Example usage:
    ssh rfsoc_host
    cd working_directory 
    sudo ~/py3.12-venv/bin/souk-readout-server
    
Author: Sam Rowe
Date: July 2024
Version: 1.1.0

"""



import asyncio, contextvars, functools
import importlib.metadata
import json
import struct
import yaml
import socket
import os
import pwd
import shutil
import traceback
import sys
import numpy as np
import base64

from importlib.resources import files as importlib_files
from souk_readout_tools.config_utils import copy_template_config

import argparse

import time


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


#STREAM FLAGS 
FLAG_SERVER_REQUEST = 0
FLAG_SET_FREQS = 1
FLAG_SET_AMPS = 2
FLAG_SET_PHASES = 3
FLAG_CAL_FREEZE = 4
FLAG_5 = 5
FLAG_6 = 6
FLAG_7 = 7


#Because we need sudo to access /dev/mem (setcap not effective)

SUDO = (os.geteuid() == 0)

# If root, choose the real target user and home explicitly
TARGET_USER = 'casper'
if SUDO:
    # hardcode paths or derive from passwd
    try:
        pw = pwd.getpwnam(TARGET_USER)
        HOME = pw.pw_dir
        TARGET_UID, TARGET_GID = pw.pw_uid, pw.pw_gid
    except KeyError:
        # fallback to /home/casper if the user isn't in /etc/passwd for some reason
        HOME = '/home/casper'
        TARGET_UID = int(os.getenv('SUDO_UID') or 0)
        TARGET_GID = int(os.getenv('SUDO_GID') or 0)
else:
    HOME = os.path.expanduser('~')
    pw = pwd.getpwnam(os.getenv('USER'))
    TARGET_UID, TARGET_GID = pw.pw_uid, pw.pw_gid


def get_pipeline_dirs(pipeline_id):
    """
    Get pipeline-specific directory paths.

    For dual-pipeline support, each pipeline uses its own subdirectory:
      ~/.souk_readout_tools/pipeline_<id>/config/
      ~/.souk_readout_tools/pipeline_<id>/calibrations/

    Returns a dict with keys: 'config', 'calibrations', 'default_config'
    """
    base_dir = os.path.join(HOME, '.souk_readout_tools', f'pipeline_{pipeline_id}')
    dirs = {
        'base': base_dir,
        'config': os.path.join(base_dir, 'config'),
        'calibrations': os.path.join(base_dir, 'calibrations'),
        'default_config': os.path.join(base_dir, 'config', 'default_config.lnk')
    }
    return dirs


def ensure_pipeline_dirs(pipeline_id):
    """
    Ensure pipeline-specific directories exist with correct ownership.
    If the default config doesn't exist, copy template files from package data.
    """
    dirs = get_pipeline_dirs(pipeline_id)
    for key in ('config', 'calibrations'):
        d = dirs[key]
        os.makedirs(d, exist_ok=True)
        if SUDO:
            # Also ensure parent directories have correct ownership
            parent = os.path.dirname(d)
            while parent and parent != HOME:
                if os.path.exists(parent):
                    os.chown(parent, TARGET_UID, TARGET_GID)
                parent = os.path.dirname(parent)
            os.chown(d, TARGET_UID, TARGET_GID)
    
    # Copy template config and calibration files if default config doesn't exist
    if not os.path.exists(dirs['default_config']):
        _copy_template_configs(dirs, pipeline_id)
        _copy_calibration_files(dirs)

    # Copy daemon files (service file into pipeline dir, control scripts to top-level daemon/)
    _ensure_daemon_files(dirs, pipeline_id)

    return dirs


def _copy_template_configs(dirs, pipeline_id):
    """
    Copy template configuration files from package data to the user's pipeline config directory.
    Updates pipeline_id in template_config.yaml to match the target pipeline.
    Copies as raw text to preserve comments.
    """
    print(f"First run for pipeline {pipeline_id}: copying template config files to {dirs['config']}")

    try:
        template_dst = os.path.join(dirs['config'], 'template_config.yaml')

        copy_template_config(template_dst, pipeline_id=pipeline_id)

        # Create default_config.lnk pointing to template_config.yaml
        default_lnk_dst = dirs['default_config']
        with open(default_lnk_dst, 'w') as f:
            f.write(template_dst)

        os.chmod(default_lnk_dst, 0o664)
        if SUDO:
            os.chown(default_lnk_dst, TARGET_UID, TARGET_GID)

        print(f"  Created {template_dst}")
        print(f"  Created {default_lnk_dst} -> {template_dst}")
        print(f"{bcolors.WARNING}Note: Please edit {template_dst} with your system-specific settings.{bcolors.ENDC}")

    except Exception as e:
        print(f"{bcolors.FAIL}Warning: Could not copy template config files: {e}{bcolors.ENDC}")
        print(f"{bcolors.WARNING}You may need to manually create a config file in {dirs['config']}{bcolors.ENDC}")


def _copy_calibration_files(dirs):
    """
    Copy example calibration files from package data to the user's pipeline calibrations directory.
    Skips files that already exist.
    """
    try:
        pkg_cal_dir = importlib_files('souk_readout_tools').joinpath('data', 'calibrations')
        for item in pkg_cal_dir.iterdir():
            dst = os.path.join(dirs['calibrations'], item.name)
            if not os.path.exists(dst):
                shutil.copy2(str(item), dst)
                if SUDO:
                    os.chown(dst, TARGET_UID, TARGET_GID)
    except Exception as e:
        print(f"{bcolors.WARNING}Warning: Could not copy calibration files: {e}{bcolors.ENDC}")


def _ensure_daemon_files(dirs, pipeline_id):
    """
    Copy daemon files from package data into the user directory structure.

    - The pipeline-specific service file goes into the pipeline directory.
    - The control scripts (install/remove) go into a top-level daemon/ directory.
    """
    import shutil

    try:
        pkg_daemon_dir = importlib_files('souk_readout_tools').joinpath('data', 'daemon')

        # Copy the service file into the pipeline directory
        # Both pipelines use the same base filename within their own directory
        if pipeline_id == 0:
            src_service = pkg_daemon_dir.joinpath('readout_server.service')
        else:
            src_service = pkg_daemon_dir.joinpath('readout_server_1.service')

        dst_service = os.path.join(dirs['base'], 'readout_server.service')
        if not os.path.exists(dst_service):
            shutil.copy2(str(src_service), dst_service)
            os.chmod(dst_service, 0o664)
            if SUDO:
                os.chown(dst_service, TARGET_UID, TARGET_GID)

        # Copy control scripts to top-level daemon/ directory
        daemon_dir = os.path.join(HOME, '.souk_readout_tools', 'daemon')
        os.makedirs(daemon_dir, exist_ok=True)
        if SUDO and os.path.exists(daemon_dir):
            os.chown(daemon_dir, TARGET_UID, TARGET_GID)

        for script in ('install_systemd_service.sh', 'remove_systemd_service.sh'):
            dst_script = os.path.join(daemon_dir, script)
            if not os.path.exists(dst_script):
                shutil.copy2(str(pkg_daemon_dir.joinpath(script)), dst_script)
                os.chmod(dst_script, 0o775)
                if SUDO:
                    os.chown(dst_script, TARGET_UID, TARGET_GID)

    except Exception as e:
        print(f"{bcolors.WARNING}Warning: Could not copy daemon files: {e}{bcolors.ENDC}")


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
# These will be overwritten per-instance in ReadoutServer
USER_CONFIG_DIR = os.path.join(HOME, '.souk_readout_tools', 'pipeline_0', 'config')
USER_CALIBRATIONS_DIR = os.path.join(HOME, '.souk_readout_tools', 'pipeline_0', 'calibrations')
DEFAULT_CONFIG = os.path.join(USER_CONFIG_DIR, 'default_config.lnk')

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


class ReadoutServer:
    """
    This server provides an interface for external clients to interact with the RFSoC firmware.
    
    Connected clients can get and set parameters, enable and disable streaming, and perform sweeps and retuning.

    There is a request server that responds to client requests and a stream server that streams data to clients.
    
    Tasks for continuous data streaming and triggered streaming are started automatically and enabled/disabled by request.
    """

    def __init__(self, config_file=None, pipeline_id=None):
        """
        Initializes the readout server.
        The server will load the specified configuration file and create the firmware interface.
        
        For dual-pipeline support, each pipeline uses separate directories:
          ~/.souk_readout_tools/pipeline_0/  (for pipeline_id=0)
          ~/.souk_readout_tools/pipeline_1/  (for pipeline_id=1)
        
        The pipeline_id is determined from the config file's firmware.pipeline_id value,
        which is authoritative since it's used to create firmware interfaces.
        
        The explicit pipeline_id parameter is only used as a hint for locating the 
        default config when no config_file is specified.
        
        Args:
            config_file: Path to config YAML. If None, uses default_config.lnk from
                         the pipeline directory specified by pipeline_id.
            pipeline_id: Only used when config_file is None, to select which pipeline's
                         default config to load. Ignored if config_file is provided
                         (config file's pipeline_id takes precedence).
        """
        
        print('************************************************')
        print('__init__')
        print('config_file:',config_file)
        print('pipeline_id (hint):',pipeline_id)
        print('************************************************')
        check_if_running_on_rfsoc_arm()
        self.process_name = set_process_name()
        self.ip_addresses = get_host_ips()
        
        # Step 1: Use pipeline_id hint to find default config if none specified
        initial_pipeline_id = pipeline_id if pipeline_id is not None else 0
        
        if config_file is None:
            # Use the hint pipeline_id to find the default config
            hint_dirs = ensure_pipeline_dirs(initial_pipeline_id)
            config_file = hint_dirs['default_config']
            print(f'Loading default config file from {config_file}')
        
        # Step 2: Load and parse config to get the authoritative pipeline_id
        # (We need to do this before setting up directories)
        resolved_config_file = config_file
        if not os.path.exists(resolved_config_file):
            hint_dirs = get_pipeline_dirs(initial_pipeline_id)
            resolved_config_file = os.path.join(hint_dirs['config'], config_file)
        
        with open(resolved_config_file, 'r') as file:
            config = yaml.safe_load(file)
        
        if type(config) is str:
            # File contains a link to another config file
            resolved_config_file = config
            if not os.path.exists(resolved_config_file):
                for pid in (0, 1):
                    test_path = os.path.join(get_pipeline_dirs(pid)['config'], resolved_config_file)
                    if os.path.exists(test_path):
                        resolved_config_file = test_path
                        break
            with open(resolved_config_file, 'r') as file:
                config = yaml.safe_load(file)
        
        # Step 3: Extract pipeline_id FROM THE CONFIG (this is authoritative)
        self.pipeline_id = config.get('firmware', {}).get('pipeline_id', 0)
        
        # Warn if explicit pipeline_id was provided and differs from config
        if pipeline_id is not None and pipeline_id != self.pipeline_id:
            print(f'{bcolors.FAIL}ERROR: Explicit pipeline_id ({pipeline_id}) differs from '
                  f'config file pipeline_id ({self.pipeline_id}).{bcolors.ENDC}')
            print(f'{bcolors.WARNING}Using config file pipeline_id={self.pipeline_id} '
                  f'(this is what the firmware interfaces will use).{bcolors.ENDC}')
        
        print(f'Config file loaded: {resolved_config_file}')
        print(f'Pipeline ID from config: {self.pipeline_id}')
        
        # Step 4: Now set up directories based on CONFIG's pipeline_id
        self.pipeline_dirs = ensure_pipeline_dirs(self.pipeline_id)
        self.user_config_dir = self.pipeline_dirs['config']
        self.user_calibrations_dir = self.pipeline_dirs['calibrations']
        self.default_config = self.pipeline_dirs['default_config']

        print(f'Using pipeline {self.pipeline_id} directories:')
        print(f'  config: {self.user_config_dir}')
        print(f'  calibrations: {self.user_calibrations_dir}')
        
        #server attributes
        self.config = None
        self.config_file = None
        self.request_clients = []
        self.stream_clients = []
        self.sweep_task = None
        self.stream_task = None
        self.triggered_stream_task = None
        self.e_stream_enabled = asyncio.Event()
        self.e_triggered_stream_enabled = asyncio.Event()
        self.tasks = []
        self.fake_trigger_event = asyncio.Event()
        self.stream_flags = [asyncio.Event() for _ in range(8)]

        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.latest_sweep_results = {}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0.0

        #rf peripheral controller
        self.rf_peripherals = None

        #initialize server (this will load config again, but that's fine)
        self.init_server(resolved_config_file, ensure_ready=False, force_ready=False)
    
    
    def ensure_ready(self, config_file=None, level="pipeline"):
        """
        Ensure the firmware is ready up to the requested init level.

        Does NOT force reprogramming or reinitialisation if not needed.

        Does not notice if config parameters have changed - but it definitely should!
        
        level:
          - "server": no firmware operations
          - "firmware": (re)program if needed, then initialise shared resources
          - "pipeline": firmware level + initialise pipeline resources
        """
        if level not in ("server", "firmware", "pipeline"):
            raise ValueError(f"Invalid ready level: {level}")

        self.load_config(config_file)

        #re-establish firmware interfaces in case they were initially created before programming
        fw_config_file = self.config['firmware']['fw_config_file']
        pipeline_id = self.config['firmware']['pipeline_id']
        self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
        self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)



        if level == "server":
            return

        if level == "firmware":
            # 1) Program firmware if needed
            if firmware_lib.needs_programming(self.r, self.config):
                self.r, self.r_fast = firmware_lib.reload_firmware(self.config)
            # 2) Shared resources init if needed
            if firmware_lib.needs_shared_resource_initialising(self.r, self.config):
                firmware_lib.initialise_shared_resources(self.r, self.config)
            
            #re-establish firmware interfaces in case they were initially created before programming
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)


            return

        if level == "pipeline":
            # 1) Program firmware if needed
            if firmware_lib.needs_programming(self.r, self.config):
                self.r, self.r_fast = firmware_lib.reload_firmware(self.config)
            # 2) Shared resources init if needed
            if firmware_lib.needs_shared_resource_initialising(self.r, self.config):
                firmware_lib.initialise_shared_resources(self.r, self.config)
            # 3) Pipeline resources init if needed
            if firmware_lib.needs_pipeline_initialising(self.r, self.config):
                firmware_lib.initialise_pipeline_resources(self.r, self.config)

            #re-establish firmware interfaces in case they were initially created before programming
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)
                
            return


    def force_ready(self, level='pipeline'):
        """
        Ensure the firmware is ready up to the requested init level by applying init.

        Forces reprogramming and reinitialisation up to the requested init level.

        level:
          - "server": no firmware operations
          - "firmware": (re)program, then initialise shared resources
          - "pipeline": firmware level + initialise pipeline resources
        """
        if level not in ("server", "firmware", "pipeline"):
            raise ValueError(f"Invalid ready level: {level}")

        #re-establish firmware interfaces in case they were initially created before programming
        fw_config_file = self.config['firmware']['fw_config_file']
        pipeline_id = self.config['firmware']['pipeline_id']
        self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
        self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)




        if level == "server":
            self.init_server(self.config_file,ensure_ready=False, force_ready=False)
            return
        
        if level == "firmware":
            # 1) Program firmware 
            self.r, self.r_fast = firmware_lib.reload_firmware(self.config)
            # 2) Shared resources
            firmware_lib.initialise_shared_resources(self.r, self.config)

            #re-establish firmware interfaces in case they were initially created before programming
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)

            return

        if level == "pipeline":
            # 1) Program firmware 
            self.r, self.r_fast = firmware_lib.reload_firmware(self.config)
            # 2) Shared resources
            firmware_lib.initialise_shared_resources(self.r, self.config)
            # 3) Pipeline resources
            firmware_lib.initialise_pipeline_resources(self.r, self.config)

            #re-establish firmware interfaces in case they were initially created before programming
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)

            return
        return
  

    def init_server(self, config_file,ensure_ready=False, force_ready=False):
        """
        Initialize server runtime + load config + create firmware interfaces.
        
        If ensure_ready is True, bring system to pipeline-ready state

        If force_ready is True, reprogram and bring system to pipeline-ready state

        """

        print('************************************************')
        print('init_server')
        print('config_file:',config_file)
        print('ensure_ready:',ensure_ready)
        print('force_ready:',force_ready)
        print('************************************************')

        #server attributes
        self.config = None
        self.config_file = None
        self.request_clients = []
        self.stream_clients = []
        self.sweep_task = None
        self.stream_task = None
        self.triggered_stream_task = None
        self.e_stream_enabled = asyncio.Event()
        self.e_triggered_stream_enabled = asyncio.Event()
        self.tasks = []
        self.stream_flags = [asyncio.Event() for _ in range(8)]

        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.latest_sweep_results = {}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0.0
        
        #load config
        self.load_config(config_file)
        self.server_address = '0.0.0.0'
        self.request_server_port = self.config['rfsoc_host']['request_port']
        self.stream_server_port = self.config['rfsoc_host']['stream_port']
        self.trigger_source_pin = self.config['rfsoc_host']['trigger_source_pin']
        
        #interface with firmware
        fw_config_file = self.config['firmware']['fw_config_file']
        pipeline_id = self.config['firmware']['pipeline_id']

        self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
        self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)
        

        if ensure_ready and force_ready:
            ensure_ready=False

        if ensure_ready:
            self.ensure_ready(level="pipeline")

        if force_ready:
            self.force_ready(level='pipeline')


        #see if we can succesfully load system information
        try:
            system_information = self.get_system_information()
        except Exception as e:
            print(bcolors.WARNING+'Warning: could not get system information from firmware:',e,bcolors.ENDC)
            print('Try hard reset')

        if firmware_lib.needs_programming(self.r,self.config):
            print(bcolors.WARNING+'Warning: firmware needs programming'+bcolors.ENDC)
        if firmware_lib.needs_shared_resource_initialising(self.r,self.config):
            print(bcolors.WARNING+'Warning: shared resources need initialising'+bcolors.ENDC)
        if firmware_lib.needs_pipeline_initialising(self.r,self.config):
            print(bcolors.WARNING+'Warning: pipeline resources need initialising'+bcolors.ENDC)

        #initialize rf peripheral controller (attenuators, amp bypass)
        try:
            self.rf_peripherals = RFPeripheralController(self.config, self.pipeline_id)
            if self.rf_peripherals.enabled:
                print(f'{bcolors.OKGREEN}RF frontend initialised ({self.rf_peripherals.attenuator_backend}){bcolors.ENDC}')
                status = self.rf_peripherals.get_status()
                print(f'  TX atten: {status["tx_attenuation_db"]:.1f} dB, '
                      f'amp bypass: {status["tx_amp_bypass"]}, '
                      f'total gain: {status["tx_total_gain_db"]:.1f} dB')
                print(f'  RX atten: {status["rx_attenuation_db"]:.1f} dB, '
                      f'amp bypass: {status["rx_amp_bypass"]}, '
                      f'total gain: {status["rx_total_gain_db"]:.1f} dB')
        except Exception as e:
            print(bcolors.WARNING+f'Warning: RF peripheral init failed: {e}'+bcolors.ENDC)
            self.rf_peripherals = None

        # Initialize LNA bias controller
        try:
            self.lna_controller = LNABiasController(self.config, self.pipeline_id)
            if self.lna_controller.enabled:
                print(f'{bcolors.OKGREEN}LNA bias controller initialised '
                      f'(channel {self.lna_controller.lna_channel}){bcolors.ENDC}')
        except Exception as e:
            print(bcolors.WARNING+f'Warning: LNA bias init failed: {e}'+bcolors.ENDC)
            self.lna_controller = None

        return
   
             
    def init_firmware(self,config_file=None):
        """
        Reprogram firmware (optionally using config_file), then init shared fw resources.
        Does NOT implicitly also init pipeline unless you request ensure_ready("pipeline").
        """
        print('************************************************')
        print('init_firmware')
        print('config_file:',config_file)
        print('************************************************')
        
        if config_file is not None:
            self.load_config(config_file)
       
        self.force_ready(level="firmware")

        return


    def init_pipeline(self, config_file=None):
        """
        Ensure shared resources and pipeline resources are initialised.
        Does not force a reprogram unless needs_programming() says so.
        Does not force shared resource initialisation unless needs_shared_resource_initialising() says so.
        """
        print('************************************************')
        print('init_pipeline')
        print('config_file:',config_file)
        print('************************************************')

        if config_file is not None:
            self.load_config(config_file)
            # rebuild interfaces in case pipeline_id / fw_config_file changed
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file, pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file, pipeline_id)

        
        #ensure pipeline is ready
        self.ensure_ready(level="pipeline")
    
        #force pipeline init
        firmware_lib.initialise_pipeline_resources(self.r, self.config)        

        return



    def load_config(self, config_file):
        """
        Load a new configuration file.
        Does not reload or initialise the firmware.
        Uses pipeline-specific directories.
        """
        print('************************************************')
        print('load_config')
        print('config_file:',config_file)
        print('************************************************')
        if config_file is None:
            config_file = self.default_config
        if not os.path.exists(config_file):
            #print(f'Config file not found {config_file}, searching in {self.user_config_dir}')
            config_file = os.path.join(self.user_config_dir, config_file)
        with open(config_file, 'r') as file:
            config = yaml.safe_load(file)
        if type(config) is str:
            #file contains a link to a config file
            config_file = config
            if not os.path.exists(config_file):
                #print(f'Linked config file not found {config_file}, searching in {self.user_config_dir}')
                config_file = os.path.join(self.user_config_dir, config_file)
            #file is a link, try again using the link contents as the config file path
            with open(config_file,'r') as file:
                config = yaml.safe_load(file)
        print(f'Found config file: {config_file}')
        self.config = config
        self.config_file = config_file

        return config
    
    def set_config(self, config_filename, config_contents, default=True):
        """
        Apply a new configuration and re-initialise the firmware
        If default is true, overwrites the default_config.lnk so that this config is persistent
        Uses pipeline-specific directories.

        Now also applies any modified config parameters in hardware.
        """
        print('************************************************')
        print('set_config')
        print('config_filename:',config_filename)
        print('************************************************')

        filename = os.path.join(self.user_config_dir, os.path.basename(config_filename))
        with open( filename, 'w') as file:
            file.write(yaml.dump(config_contents,sort_keys=False))
        os.chmod(filename,0o664)

        if SUDO:
            os.chown(filename,int(TARGET_UID),int(TARGET_GID))

        print(f'Saved config to {filename}')
        
        print('************************************************')
        print('set_config')
        print('filename:',filename)
        print('************************************************')


        firmware_lib.apply_config(config_contents, self.r, self.config)

        # Apply RF peripheral hardware settings (attenuators, amp bypass) from new config
        if self.rf_peripherals is not None and self.rf_peripherals.enabled:
            self.rf_peripherals.apply_config(config_contents)

        self.ensure_ready(config_file=filename, level="pipeline")

        if default:
            defaultname = os.path.join(self.user_config_dir, 'default_config.lnk')
            prevname = os.path.join(self.user_config_dir, 'previous_default.lnk')
        
            #make a note of the previous default config
            if os.path.exists(defaultname):
                shutil.copy(defaultname, prevname) 
                if SUDO:
                    os.chown(prevname,int(TARGET_UID),int(TARGET_GID))
            
            #link the new default
            with open(defaultname,'w') as file:
                file.write(filename)
            os.chmod(defaultname, 0o664)
            if SUDO:
                os.chown(defaultname,int(TARGET_UID),int(TARGET_GID))



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
            'process_name': self.process_name,
            'ip_addresses': self.ip_addresses,
            'pipeline_id': self.pipeline_id,
            'pipeline_dirs': self.pipeline_dirs,
            'pwd': os.getcwd(),
            'sys.executable': sys.executable,
            'sys.argv': sys.argv,
            'uname': os.uname().nodename+' '+os.uname().sysname+' '+os.uname().release + ' ' + os.uname().version + ' ' + os.uname().machine,
            'python_version': sys.version,
            'server_version': importlib.metadata.version('souk_readout_tools'),
            'config_file': self.config_file,
            'request_clients': len(self.request_clients),
            'request_client_addrs': [client.get_extra_info('peername') for client in self.request_clients],
            'stream_clients': len(self.stream_clients),
            'stream_client_addrs': [client.get_extra_info('peername') for client in self.stream_clients],
            'stream_task_started': self.stream_task is not None and not self.stream_task.done(),
            'stream_enabled': self.e_stream_enabled.is_set(),
            'triggered_stream_task_started': self.triggered_stream_task is not None and not self.triggered_stream_task.done(),
            'triggered_stream_enabled': self.e_triggered_stream_enabled.is_set(),
            'sweep_task_running': self.sweep_task is not None and not self.sweep_task.done(),
            'tasks': len(self.tasks),
            'firmware_interface_exists': bool(self.r),
            'firmware_fast_interface_exists': bool(self.r_fast),
            'firmware_programmed': not firmware_lib.needs_programming(self.r,self.config),
            'firmware_shared_resources_ready': not firmware_lib.needs_shared_resource_initialising(self.r,self.config),
            'firmware_pipeline_resources_ready': not firmware_lib.needs_pipeline_initialising(self.r,self.config),
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
                raw_msglen = await reader.read(4)
                # raw_msglen = await reader.readexactly(4)
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

                elif request == 'ensure_ready':
                    level = message.get('level', 'pipeline')
                    config_file = message.get('config_filename', None)
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.ensure_ready(config_file=config_file, level=level)
                        await self.send_response(writer, {'status': 'success', 'server_status': self.get_server_status()})

                elif request == 'hard_reset':
                    config_file = message.get('config_filename', None)
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_server(config_file,ensure_ready=False, force_ready=True)
                        await self.send_response(writer, {'status': 'success', 'server_status': self.get_server_status()})

                elif request == 'initialise_server':
                    config_file = message.get('config_filename')
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_server(config_file,ensure_ready=False, force_ready=False)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'initialise_firmware':
                    config_file = message.get('config_filename')
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        await self.send_response(writer, {'status': 'success'})
                        self.init_firmware(config_file) 
                
                elif request == 'initialise_pipeline':
                    config_file = message.get('config_filename')
                    if config_file is None:
                        config_file = self.config_file
                    if not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_pipeline(config_file)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'push_config':
                    config_filename = message.get('config_filename')
                    config_contents = message.get('config_contents')
                    self.set_config(config_filename, yaml.safe_load(config_contents), default=True)
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'pull_config':
                    await self.send_response(writer, {'status': 'success', 'config_filename': self.config_file, 'config_contents': yaml.dump(self.config,sort_keys=False)})

                elif request == 'set_default_config':
                    config_filename = message.get('config_filename')
                    #self.set_default_config(config_filename)
                    await self.send_response(writer, {'status': 'deprecated',
                        'message':'use "set_config" to push the current config and make it persistent'})

                elif request == 'push_calibration':
                    cal_filename = message.get('cal_filename')
                    cal_contents = message.get('cal_contents')
                    destination_filename = os.path.join(self.user_calibrations_dir, os.path.basename(cal_filename))
                    with open(destination_filename,'w') as file:
                        file.write(cal_contents)
                    if SUDO:
                        os.chown(destination_filename,int(TARGET_UID),int(TARGET_GID))
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'pull_calibration':
                    cal_filename = message.get('cal_filename')
                    with open(os.path.join(self.user_calibrations_dir,os.path.basename(cal_filename))) as file:
                        cal_contents = file.read()
                    await self.send_response(writer, {'status': 'success', 'cal_contents': cal_contents})

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
                        ref_plane = message.get('reference_plane', 'detector')
                        value = firmware_lib.get_tone_powers(self.r,self.config,reference_plane=ref_plane)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_powers_detailed':
                        ref_plane = message.get('reference_plane', 'detector')
                        value = firmware_lib.get_tone_powers(self.r,self.config,detailed_output=True,reference_plane=ref_plane)[1]
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
                        result = firmware_lib.set_tone_powers(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_AMPS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success', 'result': result}


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
                    self.e_triggered_stream_enabled.clear()
                    self.e_stream_enabled.set()
                    print('Stream enabled, clients:', self.stream_clients)
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'disable_stream':
                    self.e_stream_enabled.clear()
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'enable_triggered_stream':
                    self.e_stream_enabled.clear()
                    self.e_triggered_stream_enabled.set()
                    await self.send_response(writer, {'status': 'success'})

                elif request == 'disable_triggered_stream':
                    self.e_triggered_stream_enabled.clear()
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

                elif request == 'get_rx_tone_powers':
                    ref_plane = message.get('reference_plane', 'adc_input')
                    powers = firmware_lib.get_rx_tone_powers(self.r, self.config, reference_plane=ref_plane)
                    await self.send_response(writer, {'status': 'success', 'powers': powers.tolist()})

                elif request == 'maximise_tx_power':
                    headroom_db = message.get('headroom_db', 2.0)
                    amps,psb_fft_shift,psb_scale,dsp,dac = firmware_lib.maximise_tx_power(self.r,self.config, headroom_db=headroom_db)
                    result = {'amps': amps.tolist(), 'psb_fft_shift': psb_fft_shift, 'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'maximise_rx_power':
                    headroom_db = message.get('headroom_db', 2.0)
                    dsa,pfb_fft_shift,dsp,adc,rx_atten = firmware_lib.maximise_rx_power(self.r,self.config, headroom_db=headroom_db, rf_peripherals=self.rf_peripherals)
                    result = {'dsa': dsa, 'pfb_fft_shift': pfb_fft_shift, 'dsp_ovf': dsp, 'adc_levels': adc, 'rx_attenuation_db': rx_atten}
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'optimise_tx_snr':
                    amps,psb_fft_shift,psb_scale,dsp,dac = firmware_lib.optimise_tx_snr(self.r,self.config)
                    result = {'amps': amps.tolist(), 'psb_fft_shift': psb_fft_shift, 'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'optimise_rx_snr':
                    pfb_fft_shift,dsp,adc = firmware_lib.optimise_rx_snr(self.r,self.config)
                    result = {'pfb_fft_shift': pfb_fft_shift, 'dsp_ovf': dsp, 'adc_levels': adc}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'fix_dac_saturation':
                    psb_scale, dsp, dac = firmware_lib.fix_dac_saturation(self.r,self.config)
                    result = {'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'fix_adc_saturation':
                    dsa,fftshift, dsp_ovf, levels, rx_atten = firmware_lib.fix_adc_saturation(self.r,self.config, rf_peripherals=self.rf_peripherals)
                    result = {'dsa': dsa, 'fftshift': fftshift, 'dsp_ovf': dsp_ovf, 'adc_levels': levels, 'rx_attenuation_db': rx_atten}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                # -- RF peripheral (attenuator / amp bypass) commands --

                elif request == 'get_rf_peripheral_status':
                    if self.rf_peripherals is not None and self.rf_peripherals.enabled:
                        result = self.rf_peripherals.get_status()
                    else:
                        result = {'enabled': False}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_tx_attenuation':
                    value = float(message.get('value'))
                    self.rf_peripherals.set_tx_attenuation(value)
                    result = {'tx_attenuation_db': self.rf_peripherals.get_tx_attenuation()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_tx_attenuation':
                    result = {'tx_attenuation_db': self.rf_peripherals.get_tx_attenuation()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_rx_attenuation':
                    value = float(message.get('value'))
                    self.rf_peripherals.set_rx_attenuation(value)
                    result = {'rx_attenuation_db': self.rf_peripherals.get_rx_attenuation()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_rx_attenuation':
                    result = {'rx_attenuation_db': self.rf_peripherals.get_rx_attenuation()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_tx_amp_bypass':
                    bypass = bool(message.get('bypass'))
                    self.rf_peripherals.set_tx_amp_bypass(bypass)
                    result = {'tx_amp_bypass': self.rf_peripherals.get_tx_amp_bypass()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_tx_amp_bypass':
                    result = {'tx_amp_bypass': self.rf_peripherals.get_tx_amp_bypass()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_rx_amp_bypass':
                    bypass = bool(message.get('bypass'))
                    self.rf_peripherals.set_rx_amp_bypass(bypass)
                    result = {'rx_amp_bypass': self.rf_peripherals.get_rx_amp_bypass()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_rx_amp_bypass':
                    result = {'rx_amp_bypass': self.rf_peripherals.get_rx_amp_bypass()}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                # --- LNA bias control ---

                elif request == 'get_lna_controller_status':
                    if self.lna_controller is not None:
                        result = self.lna_controller.get_status()
                    else:
                        result = {'enabled': False}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_lna_bias_status':
                    channel = message.get('channel')
                    if channel is not None:
                        channel = int(channel)
                    result = self.lna_controller.get_lna_bias_status(channel)
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_lna_bias_status_all':
                    result = self.lna_controller.get_lna_bias_status_all()
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_lna_bias_voltage':
                    voltage_v = float(message.get('voltage_v'))
                    channel = message.get('channel')
                    if channel is not None:
                        channel = int(channel)
                    method = message.get('method', 'remote')
                    blind = message.get('blind', False)
                    result = self.lna_controller.set_lna_bias_voltage(
                        voltage_v, channel, method, blind,
                    )
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_lna_bias_voltage_all':
                    voltage_v = float(message.get('voltage_v'))
                    method = message.get('method', 'remote')
                    blind = message.get('blind', False)
                    result = self.lna_controller.set_lna_bias_voltage_all(
                        voltage_v, method, blind,
                    )
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_samples':
                    num_samples = message.get('num_samples')
                    task = asyncio.create_task(self.get_samples(writer, num_samples))
                    self.tasks.append(task)

                elif request == 'get_accumulator_snapshots':
                    tone_index = message.get('tone_index')
                    num_snapshots = message.get('num_snapshots')
                    task = asyncio.create_task(self.get_accumulator_snapshots(writer, tone_index, num_snapshots))
                    self.tasks.append(task)

                elif request == 'get_adc_snapshot':
                    try:
                        snapshot = firmware_lib.get_adc_snapshot(self.r)
                        data = base64.b64encode(snapshot.tobytes()).decode()
                        result = {'snapshot': data, 'length': len(snapshot)}
                        await self.send_response(writer, {'status': 'success', 'result': result})
                    except Exception as e:
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'get_dac_snapshot':
                    try:
                        dac0, dac1 = firmware_lib.get_dac_snapshot(self.r)
                        data0 = base64.b64encode(dac0.tobytes()).decode()
                        data1 = base64.b64encode(dac1.tobytes()).decode()
                        result = {'dac0': data0, 'dac1': data1, 'length': len(dac0)}
                        await self.send_response(writer, {'status': 'success', 'result': result})
                    except Exception as e:
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

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
                        sweep_f = self.latest_sweep_results['sweep_frequencies'].astype('f8')
                        sweep_z = self.latest_sweep_results['sweep_responses'].astype('complex128')
                        sweep_e = self.latest_sweep_results['sweep_sems'].astype('complex128')
                        sweep = {}
                        sweep['f'] = base64.b64encode(sweep_f.tobytes()).decode()
                        sweep['z'] = base64.b64encode(sweep_z.tobytes()).decode()
                        sweep['e'] = base64.b64encode(sweep_e.tobytes()).decode()

                        # for i in range(len(sweep_f[0])):
                        #     tone={}
                        #     tone['f'] = sweep_f[:,i].tolist()
                        #     tone['i'] = sweep_z[:,i].real.tolist()
                        #     tone['q'] = sweep_z[:,i].imag.tolist()
                        #     tone['ei'] = sweep_e[:,i].real.tolist()
                        #     tone['eq'] = sweep_e[:,i].imag.tolist()
                        #     sweep[f'{i:04d}'] = tone

                        data={'date': time.strftime("%Y-%m-%d %H:%M:%S UTC%z"),
                              'num_tones': len(sweep_f[0]),
                              'num_points': len(sweep_f),
                              'samples_per_point': self.latest_sweep_results['samples_per_point'],
                              'sweep': sweep,
                              'system_information': self.get_system_information()}
                        

                        await self.send_response(writer, {'status': 'success', 'data': data})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'No valid sweep data available yet, please perform a sweep first'})
                
                elif request == 'get_sweep_raw_samples':
                    if self.latest_sweep_data_valid:
                        sweep_z = self.latest_sweep_data['sweep_data']
                        samples,points,tones = sweep_z.shape

                        # data_i = sweep_z.real.tolist()
                        # data_q = sweep_z.imag.tolist()
                        data_i = base64.b64encode(sweep_z.real.astype('float64').tobytes()).decode()
                        data_q = base64.b64encode(sweep_z.imag.astype('float64').tobytes()).decode()
                        data = {'samples':samples,
                                'points':points,
                                'tones':tones,
                                'data_i': data_i,
                                'data_q': data_q}
                        
                        await self.send_response(writer, {'status': 'success', 'data': data})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'No valid sweep data available yet, please perform a sweep first'})
                



                elif request == 'get_sweep_txt':
                    if self.latest_sweep_data_valid:
                        sweep_f = self.latest_sweep_results['sweep_frequencies']
                        sweep_z = self.latest_sweep_results['sweep_responses']
                        sweep_e = self.latest_sweep_results['sweep_sems']
                        data = '# Sweep file\n'
                        data += f'# date: {time.strftime("%Y-%m-%d %H:%M:%S %Z")}\n'
                        data += f'# num_tones: {len(sweep_f[0])}\n'
                        data += f'# num_points: {len(sweep_f)}\n'
                        data += f'# samples_per_point: {self.latest_sweep_results["samples_per_point"]}\n'
                        
                        data += '# system_info: '+str(self.get_system_information()) +'\n'
                        data += '#' + ' '.join([f'sweep_f_{k:04d} sweep_i_{k:04d} sweep_q_{k:04d} err_i_{k:04d} err_q_{k:04d}' for k in range(len(sweep_f))]) + '\n'

                        for j in range(len(sweep_f)):
                            for i in range(len(sweep_f[0])):
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
                        freq_offsets = message.get('freq_offsets', None)
                        self.sweep_task = asyncio.create_task(
                            self.retune(centers, spans, points, samples_per_point, direction, method, freq_offsets)
                        )
                        await self.send_response(writer, {'status': 'success', 'message': 'Retune in progress'})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'Sweep or retune already in progress'})

                elif request == 'cancel':
                    if self.sweep_task:
                        self.sweep_task.cancel()
                        self.sweep_task = None
                        self.sweep_progress=0.0
                    for task in self.tasks:
                        task.cancel()
                    self.tasks = []
                    await self.send_response(writer, {'status': 'success'})

                else:
                    await self.send_response(writer, {'status': 'error', 'message': f'Invalid request {request}'})
                
                self.stream_flags[FLAG_SERVER_REQUEST].clear()
                await asyncio.sleep(0)
                

        except asyncio.IncompleteReadError:
            print(f"request client disconnected unexpectedly: {addr} (IncompleteReadError)")
        except Exception as e:
            print(f"Error handling request client {addr}: {e}")
            print(traceback.format_exc())
            await self.send_response(writer, {'status': 'error', 'message': f'Error handling request: {e}'})
        finally:
            print(f"request client disconnected: {addr}")
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
            
            # warm up a bit to avoid initial delays
            rate = firmware_lib.get_sample_rate(self.r_fast)
            if rate>100:
                prev_cnt=0
                iter=0
                for j in range(50):
                    payload, cnt, err =  self.prepare_frame(fast_read_params)
                    continue
            
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

    async def get_accumulator_snapshots(self, writer, tone_index, num_snapshots):
        """
        Acquire num_snapshots pre-accumulation snapshots for a single tone
        and send to client.

        Each snapshot is 1024 complex samples at full rate (before accumulation).
        """
        try:
            # Resolve tone index to firmware channel once
            details = firmware_lib.get_tone_frequencies(self.r, self.config, detailed_output=True)[1]
            firmware_indices = details['rx']['tone_indices']
            if tone_index >= len(firmware_indices):
                raise ValueError(f'Tone index {tone_index} out of range '
                                 f'(only {len(firmware_indices)} tones active)')
            fw_chan = firmware_indices[tone_index]
            acc = self.r.accumulators[0]
            acc.set_snapshot_chan(fw_chan)

            for _ in range(num_snapshots):
                data = np.asarray(acc.get_new_snapshot(), dtype=np.complex128)
                data_bytes = data.tobytes()
                data_len = struct.pack('>I', len(data_bytes))
                writer.write(data_len + data_bytes)
                await writer.drain()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error getting accumulator snapshots: {e}")
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
                if self.e_stream_enabled.is_set():
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
                if self.e_triggered_stream_enabled.is_set():
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
            self.sweep_progress = 0.0
            
            centers = np.atleast_1d(centers)
            spans = np.atleast_1d(spans)
            if len(spans)==1:
                spans = np.full(len(centers),spans[0])
            assert len(centers) == len(spans)
            num_points=int(points)
            samples_per_point=int(samples_per_point)
            assert direction in ('up','down')

            # after the sweep, the tones should be set back to their original frequencies.
            # unless the number of tones has changed, or the tones were not within the span of the sweep.
            # in which case they should be set to the center frequencies.
            initial_freqs = firmware_lib.get_tone_frequencies(self.r, self.config)
            if len(initial_freqs)!=len(centers):
                initial_freqs = centers
            for i in range(len(centers)):
                if initial_freqs[i]<centers[i]-spans[i]/2. or initial_freqs[i]>centers[i]+spans[i]/2.:
                    initial_freqs[i] = centers[i]

            print('Setting initial tone frequencies')
            firmware_lib.set_tone_frequencies(self.r,self.config,centers,autosync=True)

            print('Preparing sweep')
            num_tones = len(centers) 
            channels = np.arange(num_tones,dtype=int)
            sweepfreqs = np.zeros((num_points,num_tones),dtype=float)
            for t in range(num_tones):
                cf=centers[t]
                sp=spans[t]
                if direction == 'up':
                    sweepfreqs[:,t] = np.linspace(cf-sp/2.,cf+sp/2.,num_points)
                elif direction=='down':
                    sweepfreqs[:,t] = np.linspace(cf-sp/2.,cf+sp/2.,num_points)[::-1]
            
            print('Preparing faster sweep settings')
            fast_read_params = firmware_lib.get_fast_read_params(self.r_fast) 
            # fast_write_params = []
            # for p in range(num_points):
            #     fast_write_params.append(firmware_lib.prepare_tone_frequency_settings_fast(self.r, self.config, sweepfreqs[p]))
            fast_sweep_params = firmware_lib.prepare_sweep_settings_fast(self.r_fast,self.config,sweepfreqs)
            
            # Get tone_indices array - shape (num_points, num_tones)
            # These may change at each sweep point as tones cross FFT bin boundaries
            tone_indices_arr = fast_sweep_params.get('tone_indices')
            
            print('Starting sweep')
            acc_counts = np.zeros((samples_per_point,num_points),dtype=int)
            sweep_data = np.zeros((samples_per_point,num_points,num_tones),dtype=complex)
            acc_errs = np.zeros((samples_per_point,num_points),dtype=bool)

            for p in range(num_points):
                # print('sweeping: setting tone frequencies',sweepfreqs[p])
                
                # #slowest method
                # firmware_lib.set_tone_frequencies(self.r,
                #                     self.config,
                #                     sweepfreqs[p],
                #                     autosync=True)
                
                # #fast method (fast_write_mixer)
                # firmware_lib.set_tone_frequencies_fast(self.r,
                #                     self.r_fast,
                #                     self.config,
                #                     sweepfreqs[p],
                #                     autosync=True)
                
                # #faster method (fast_write_mixer and skip unnecessary chanmap updates)
                # firmware_lib.apply_tone_frequency_settings_fast(self.r,
                #                                                 self.r_fast,
                #                                                 fast_write_params[p],
                #                                                 autosync=True)


                # even faster method (fast_write_mixer, skip unnecessary chanmap updates and vectorised preparation of tone frequency settings)      
                firmware_lib.apply_sweep_step_fast(self.r,
                                                   self.r_fast,
                                                   fast_sweep_params,
                                                   p,
                                                   autosync=True)
                
                # must wait for everything to settle.
                # two acc is enough at 500 samps/sec
                for _ in range(2):
                    firmware_lib._wait_for_acc(self.r_fast,0,0.0001)

                
                print('sweeping: getting_samples')
                # Get tone_indices for this sweep point - may change as tones cross FFT bins
                tone_indices_p = tone_indices_arr[p] if tone_indices_arr is not None else np.arange(num_tones)
                for s in range(samples_per_point):
                    cnt,data,err = firmware_lib.read_accumulated_data_fast(
                                                            fast_read_params,
                                                            tone_indices=tone_indices_p)
                    acc_counts[s,p] = cnt
                    sweep_data[s,p] = data[::2]+1j*data[1::2]
                    acc_errs[s,p] = err
                # time.sleep(0.001)

                self.sweep_progress = float(p/(num_points-1))
                await asyncio.sleep(0.0001)

            # print('reset initial freqs:',initial_freqs)
            firmware_lib.set_tone_frequencies(self.r,self.config,initial_freqs,autosync=True)

            sweep_responses = np.mean(sweep_data.real,axis=0) + 1j*np.mean(sweep_data.imag,axis=0)
            sweep_stds = np.std(sweep_data.real,axis=0) + 1j*np.std(sweep_data.imag,axis=0)
            sweep_sems = np.std(sweep_data.real,axis=0)/np.sqrt(samples_per_point) + 1j*np.std(sweep_data.imag,axis=0)/np.sqrt(samples_per_point)

            self.latest_sweep_results = {
                'sweep_frequencies': sweepfreqs,
                'sweep_responses': sweep_responses,
                'sweep_stds': sweep_stds,
                'sweep_sems': sweep_sems,
                'samples_per_point': samples_per_point,
                'samples_per_second': firmware_lib.get_sample_rate(self.r_fast),
                'accumulation_counts': acc_counts,
                'accumulation_errors': acc_errs
                }
            self.latest_sweep_data = {
                'sweep_data': sweep_data
                }
            # results = firmware_lib.perform_sweep(self.r,self.r_fast, self.config, center, span, points, samples_per_point, direction)
            #self.latest_sweep_data['f'] = results['sweep_frequencies']
            #self.latest_sweep_data['z'] = results['sweep_responses']
            #self.latest_sweep_data['e'] = results['sweep_stds']
            self.latest_sweep_data_valid = True

        except asyncio.CancelledError:
            print('ayncio sweep cancelled')
            firmware_lib.set_tone_frequencies(self.r,self.config,initial_freqs,autosync=True)
            self.latest_sweep_results = {
                'sweep_frequencies': sweepfreqs,
                'sweep_responses': sweep_responses,
                'sweep_stds': sweep_stds,
                'sweep_sems': sweep_sems,
                'samples_per_point': samples_per_point,
                'samples_per_second': firmware_lib.get_sample_rate(self.r_fast),
                'accumulation_counts': acc_counts,
                'accumulation_errors': acc_errs
                }

            pass
        except Exception as e:
            self.sweep_progress = float(1.0)
            print(f"Error performing sweep: {e}")
            print(traceback.format_exc())
        
    async def retune(self, center, span, points, samples_per_point, direction, method,freq_offsets=None):
        """
        A coroutine that performs a frequency sweep, finds the resonance peaks using the specified method, and sets the tones to the peak frequencies.
        """
       
        center = np.atleast_1d(center)
        span = np.atleast_1d(span)
        if len(span)==1:
            span = np.full(len(center),span[0])
        assert len(center) == len(span)
        num_points=int(points)
        samples_per_point=int(samples_per_point)
        assert direction in ('up','down')

       
        #handle freq_offsets, if None, all zeros, if scalar, make array of that value, if array, ensure correct length
        if freq_offsets is None:
            freq_offsets = np.zeros_like(center)
        elif np.isscalar(freq_offsets):
            freq_offsets = np.full_like(center, freq_offsets)
        else:
            freq_offsets = np.atleast_1d(freq_offsets)
        if freq_offsets.shape != center.shape:
            raise ValueError("freq_offsets must be None, a scalar, or have the same shape as centers")
        if np.any(np.abs(freq_offsets) > span/2):
            print("Warning: some freq_offsets are larger than half the span, which may cause tones to be set outside the sweep range")
        
        try:
            if method not in ('max_gradient','min_mag'):
                raise ValueError(f'Invalid retune method "{method}", must be "max_gradient" or "min_mag"')
            
            await self.sweep(center, span, points, samples_per_point, direction)

            # sweep_f = self.latest_sweep_data['f']
            # sweep_z = self.latest_sweep_data['z']
            # sweep_e = self.latest_sweep_data['e']

            sweep_f = self.latest_sweep_results['sweep_frequencies']
            sweep_z = self.latest_sweep_results['sweep_responses']
            sweep_e = self.latest_sweep_results['sweep_sems']

            retune_freqs = np.zeros_like(sweep_f[0])

            if method == 'max_gradient':
                for t in range(len(center)):
                    freqs = sweep_f[:,t]
                    grads = np.abs(np.gradient(sweep_z[:,t]))
                    max_grad = np.argmax(grads)
                    retune_freqs[t] = freqs[max_grad] + freq_offsets[t]
            elif method == 'min_mag':
                for t in range(len(center)):
                    freqs = sweep_f[:,t]
                    mags = np.abs(sweep_z[:,t])
                    min_mag = np.argmin(mags)
                    retune_freqs[t] = freqs[min_mag] + freq_offsets[t]

            print('Retune freqs = found freqs + freq offsets = ',retune_freqs)

            firmware_lib.set_tone_frequencies(self.r,self.config,retune_freqs)
            # print('New frequencies:',firmware_lib.get_tone_frequencies(self.r,self.config))

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

    async def async_main(self):
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

import time
import argparse
import ctypes


def main():

    parser = argparse.ArgumentParser(description="SOUK MKID readout server (RFSoC)")
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

    # Defer heavy imports until after argument parsing for fast --help.
    global calibration, firmware_lib, RFPeripheralController, LNABiasController
    from souk_readout_tools import calibration
    from souk_readout_tools import firmware_lib
    from souk_readout_tools.server.rf_peripherals import RFPeripheralController
    from souk_readout_tools.server.lna_controller import LNABiasController

    readout_server = ReadoutServer(config_file=args.config, pipeline_id=args.pipeline)
    asyncio.run(readout_server.async_main())

if __name__=="__main__":
    main()
