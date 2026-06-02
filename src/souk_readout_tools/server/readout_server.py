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
Version: 1.2.0

"""



import asyncio, contextvars, functools
import copy
import datetime
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
from souk_readout_tools.timing import get_timing_summary, get_timing_status
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


def _format_log_value(value):
    if value is None:
        return 'none'
    if isinstance(value, bool):
        return 'yes' if value else 'no'
    return str(value)


def _format_request_scalar(value, max_chars=48):
    """Format a single scalar for a request log line (short, one line)."""
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float):
        return f'{value:.6g}'
    # Truncate long reprs (with an ellipsis) so request logs stay one line.
    text = _format_log_value(value)
    if len(text) > max_chars:
        text = text[:max_chars - 3] + '...'
    return text


def _format_request_value(value, max_items=4, max_chars=96):
    """Format any request value for logging, previewing arrays/lists/dicts
    compactly (shape, first few items, lengths) rather than dumping them."""
    if isinstance(value, np.ndarray):
        values = _format_request_value(
            value.ravel().tolist(), max_items=max_items, max_chars=max_chars)
        return f'array(shape={value.shape}, {values})'

    if isinstance(value, (list, tuple)):
        if not value:
            return '[]'
        preview = ', '.join(
            _format_request_scalar(item, max_chars=24)
            for item in value[:max_items]
        )
        if len(value) > max_items:
            preview += ', ...'
        return f'[{preview}] (len={len(value)})'

    if isinstance(value, dict):
        if not value:
            return '{}'
        keys = list(value.keys())
        preview = ', '.join(str(key) for key in keys[:max_items])
        if len(keys) > max_items:
            preview += ', ...'
        return f'{{{preview}}} (keys={len(keys)})'

    return _format_request_scalar(value, max_chars=max_chars)


def _format_request_log(message):
    """Build a one-line summary of an incoming request for the server log,
    showing only the fields that matter for the request type."""
    request = message.get('request')
    if request in ('get', 'set'):
        fields = [('param', message.get('param'))]
        if request == 'set' and 'value' in message:
            fields.append(('value', message.get('value')))
        for key in (
            'reference_plane',
            'optimise_dynamic_range',
            'rx_policy',
        ):
            if key in message:
                fields.append((key, message[key]))
        for key in sorted(k for k in message if k.startswith('force_')):
            fields.append((key, message[key]))
        details = ' '.join(
            f'{key}={_format_request_value(value)}'
            for key, value in fields
        )
        return f'{request} {details}'

    request_log_fields = {
        'get_info': ('sections',),
        'get_samples': ('num_samples', 'burst'),
        'get_accumulator_snapshots': ('tone_index', 'num_snapshots', 'fast'),
        'batch_accumulator_snapshots': ('tone_indices', 'num_snapshots'),
        'sweep': ('centers', 'spans', 'points', 'samples_per_point', 'direction'),
        'retune': (
            'centers', 'spans', 'points', 'samples_per_point',
            'direction', 'method'),
        'refresh_adc_cal': ('adc_cal_settle_time',),
        'enable_modulation': ('mod_indices', 'samples_per_point', 'n_settle'),
        'update_modulation': ('on_map_change',),
    }
    fields = [
        (key, message[key])
        for key in request_log_fields.get(request, ())
        if key in message
    ]
    if not fields:
        return _format_request_value(request)
    details = ' '.join(
        f'{key}={_format_request_value(value)}'
        for key, value in fields
    )
    return f'{request} {details}'


def _server_log(message, source='general'):
    print(f'server:{source}: {message}', flush=True)


def _server_log_fields(title, fields, source='general'):
    _server_log(title, source=source)
    for label, value in fields:
        _server_log(f'  {label}: {_format_log_value(value)}', source=source)


def _format_socket_name(sockname):
    if isinstance(sockname, tuple) and len(sockname) >= 2:
        return f'{sockname[0]}:{sockname[1]}'
    return str(sockname)


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

    ``pipeline_id`` selects the pipeline subdirectory.
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

    ``pipeline_id`` selects which pipeline's directories to create/populate.
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
    Updates pipeline-specific defaults in template_config.yaml to match the target pipeline.
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
        print(f"Note: Use copy_template_config() to create a named config from this template, then edit it with your system-specific settings.")

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
    - The control scripts and board-level service files go into a top-level
      daemon/ directory.
    - Timing config templates go into a top-level timing/ directory.
    """
    import shutil

    try:
        pkg_daemon_dir = importlib_files('souk_readout_tools').joinpath('data', 'daemon')

        def copy_package_file(src, dst, mode):
            src = str(src)
            needs_copy = True
            if os.path.exists(dst):
                try:
                    with open(src, 'rb') as src_fh, open(dst, 'rb') as dst_fh:
                        needs_copy = src_fh.read() != dst_fh.read()
                except OSError:
                    needs_copy = True
            if needs_copy:
                shutil.copy2(src, dst)
            os.chmod(dst, mode)
            if SUDO:
                os.chown(dst, TARGET_UID, TARGET_GID)

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

        for script in ('install_systemd_service.sh', 'remove_systemd_service.sh',
                       'restart_systemd_service.sh', 'install_timing_services.sh'):
            dst_script = os.path.join(daemon_dir, script)
            copy_package_file(pkg_daemon_dir.joinpath(script), dst_script, 0o775)

        for service in ('ptp4l.service', 'timing-monitor.service'):
            dst_service = os.path.join(daemon_dir, service)
            copy_package_file(pkg_daemon_dir.joinpath(service), dst_service, 0o664)

        pkg_timing_dir = importlib_files('souk_readout_tools').joinpath('data', 'timing')
        timing_dir = os.path.join(HOME, '.souk_readout_tools', 'timing')
        os.makedirs(timing_dir, exist_ok=True)
        if SUDO and os.path.exists(timing_dir):
            os.chown(timing_dir, TARGET_UID, TARGET_GID)

        for config_file in ('ptp4l.conf', 'ptp-phc.conf'):
            dst_config = os.path.join(timing_dir, config_file)
            copy_package_file(pkg_timing_dir.joinpath(config_file), dst_config, 0o664)

    except Exception as e:
        print(f"{bcolors.WARNING}Warning: Could not copy daemon files: {e}{bcolors.ENDC}")


def extract_pipeline_id_from_config(config_file):
    """
    Extract pipeline_id from the config at ``config_file`` without fully
    loading it.  Returns the pipeline_id (int) or 0 if not found.
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
PROCESS_NAME_COMM_LIMIT = 15

def check_if_running_on_rfsoc_arm():
    """
    Check if the script is running on the RFSoC ARM processor.
    Looks for "xilinx" in output of uname.
    """
    import os
    if 'xilinx' not in os.uname().release:
        print('This script should only be run on the RFSoC ARM processor')
        raise RuntimeError('This script should only be run on the RFSoC ARM processor')
    
def set_process_name(pipeline_id=None):
    """
    Set the process name for the current process.
    Useful for identifying the process in the output of 'top' or 'ps caux'.

    Linux task names shown by top/ps comm are limited to 15 visible bytes, so
    keep the default short enough that the pipeline suffix is not truncated.

    ``pipeline_id`` is appended to the process name when given so the two
    pipelines are distinguishable in ``top``/``ps``.
    """
    import os
    import ctypes
    default_process_name = (
        f'readout_srv_{pipeline_id}' if pipeline_id is not None else 'readout_server'
    )
    requested_process_name = os.getenv('READOUT_SERVER_NAME', default_process_name)
    legacy_process_names = set()
    if pipeline_id is not None:
        legacy_process_names.update({
            f'readout_daemon_{pipeline_id}',
            f'readout_server_{pipeline_id}',
        })
    if requested_process_name in legacy_process_names:
        requested_process_name = default_process_name
    process_name = requested_process_name
    if len(process_name.encode('utf-8')) > PROCESS_NAME_COMM_LIMIT:
        process_name = (
            process_name.encode('utf-8')[:PROCESS_NAME_COMM_LIMIT]
            .decode('utf-8', errors='ignore')
        )
        print(
            f"Warning: READOUT_SERVER_NAME '{requested_process_name}' exceeds "
            f"{PROCESS_NAME_COMM_LIMIT} bytes; using '{process_name}' for top/ps."
        )
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


class ModulationScheduler:
    """
    Drive fast tone-frequency modulation across N points using the two firmware
    control buffers, with correct double-buffer ping-pong for any N.

    The firmware has exactly two LO control buffers. To change every tone's
    frequency without disturbing the point currently being accumulated, we
    write the *next* point's words into the **inactive** buffer, then flip the
    active-buffer index and pulse sync. The just-vacated buffer is then free to
    receive the following point during the current point's dwell, so for N > 2
    the per-visit buffer write is hidden under the accumulation window.

    Why not reuse ``apply_sweep_step_fast``: its buffer assignment is by point
    parity (0,1,0,1,...), which is only safe for a single forward traversal (a
    sweep). When *cycling* (…, N-1, 0, 1, …) that scheme can write the next
    point into the buffer that is still active — corrupting live data for odd N.
    This scheduler instead tracks which point lives in each buffer and only ever
    writes the inactive one.

    N=2 fast path: points 1 and 2 live permanently in buffers 0 and 1 and never
    collide, so after arming each visit is just a buffer-index flip + sync with
    **no** per-visit buffer write. This falls out naturally from the
    "skip the write if the inactive buffer already holds the wanted point" rule.

    Channel maps are applied **once** at arm and never touched afterwards — the
    armed maps are held fixed and tones ride the ~2x filterbank overlap (see
    :func:`firmware_lib.prepare_modulation_settings_fast`).

    Parameters
    ----------
    r_fast : object
        Fast firmware interface used for all control-buffer / sync writes.
    bundle : dict
        Output of :func:`firmware_lib.prepare_modulation_settings_fast`
        (per-point control words + armed channel maps + tone indices).
    samples_per_point : int
        Number of accumulations emitted per point per cycle (the dwell).
    n_settle : int
        Number of leading samples per point flagged as settling/transient.
    revision : int
        Configuration revision stamped into each emitted frame's tag.
    """

    def __init__(self, r_fast, bundle, samples_per_point, n_settle, revision):
        self.r_fast = r_fast
        self.bundle = bundle
        self.N = int(bundle['num_points'])
        self.samples_per_point = int(samples_per_point)
        self.n_settle = int(n_settle)
        self.revision = int(revision)
        # Bookkeeping: which modulation point currently lives in each buffer, and
        # which buffer/point is live. ``None`` = unknown/empty.
        self._buf_holds = {0: None, 1: None}
        self._active_buf = 0
        self._active_point = 0

    def _write_point(self, buf, point):
        """Write point ``point``'s control words into control buffer ``buf``.

        ``buf`` is the target buffer index (0 or 1); ``point`` is the modulation
        point index (0-based). Records the buffer's contents so future visits can
        skip redundant writes (this is what gives the N=2 fast path).
        """
        firmware_lib.write_control_buffer_data_fast(
            self.r_fast, buf,
            self.bundle['control_values'][point],
            self.bundle['control_indices'][point])
        self._buf_holds[buf] = point

    def arm(self):
        """
        Load the armed channel maps and prime both buffers so the first emitted
        sample is **point 1** (index 0), already live.

        Writes the chanmaps once, loads point 0 into buffer 0 and makes it
        active (index flip + sync), then pre-loads point 1 into buffer 1 ready
        for the first swap. Safe to call again to re-arm after a pause/update.
        """
        # Channel maps: written once here, never in the hot loop.
        firmware_lib.psb_chanselect_set_channel_inmap(self.r_fast, self.bundle['chanmap_psb_inmap'])
        firmware_lib.chanselect_set_channel_outmap(self.r_fast, self.bundle['chanmap_pfb'])
        # Point 0 -> buffer 0, make it the live buffer.
        self._write_point(0, 0)
        firmware_lib.set_control_buffer_idx_fast(self.r_fast, 0)
        firmware_lib.force_sync_fast(self.r_fast)
        self._active_buf = 0
        self._active_point = 0
        # Pre-load the next point into the inactive buffer (nothing to do for N=1).
        if self.N > 1:
            self._write_point(1, 1 % self.N)

    def install_bundle(self, bundle, revision, samples_per_point, n_settle):
        """
        Swap in a new per-point control bundle **without re-arming** — used for a
        seamless live update whose channel maps are unchanged.

        Parameters
        ----------
        bundle : dict
            New per-point control words (same armed maps as the running config).
        revision : int
            New configuration revision to stamp into subsequent frames.
        samples_per_point : int
            Updated dwell (samples per point per cycle).
        n_settle : int
            Updated settling-sample count per point.

        Buffer bookkeeping is invalidated so each subsequent :meth:`advance`
        re-writes the inactive buffer with the new words; the live buffer picks up
        the new frequencies within one cycle (no chanmap change, no reset).
        """
        self.bundle = bundle
        self.N = int(bundle['num_points'])
        self.revision = int(revision)
        self.samples_per_point = int(samples_per_point)
        self.n_settle = int(n_settle)
        self._buf_holds = {0: None, 1: None}
        if self._active_point >= self.N:
            self._active_point = 0

    def current_point(self):
        """Return the 0-based modulation point index that is currently live."""
        return self._active_point

    def advance(self):
        """
        Step to the next modulation point: flip to the buffer already holding it
        (index flip + sync), then pre-load the *following* point into the now
        inactive buffer during this point's dwell.

        For N <= 2 the pre-load is skipped whenever the inactive buffer already
        holds the wanted point, which is always the case once armed — that is the
        zero-extra-write fast path. No-op for N == 1.
        """
        if self.N <= 1:
            return
        nxt = (self._active_point + 1) % self.N
        inactive = 1 - self._active_buf
        # Normally the inactive buffer was pre-loaded with ``nxt`` last visit; only
        # write if not (e.g. immediately after arming, or after a re-arm).
        if self._buf_holds[inactive] != nxt:
            self._write_point(inactive, nxt)
        # Flip live buffer to the one holding ``nxt`` and latch with a sync.
        firmware_lib.set_control_buffer_idx_fast(self.r_fast, inactive)
        firmware_lib.force_sync_fast(self.r_fast)
        self._active_buf = inactive
        self._active_point = nxt
        # Pre-load the following point into the freshly-vacated buffer (hidden
        # under this point's dwell). Skipped when it already holds it (N<=2).
        following = (nxt + 1) % self.N
        new_inactive = 1 - self._active_buf
        if self._buf_holds[new_inactive] != following:
            self._write_point(new_inactive, following)


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
        _server_log_fields('starting readout server', [
            ('config file', config_file or 'default'),
            ('pipeline hint', pipeline_id if pipeline_id is not None else 'default'),
        ], source='init')
        check_if_running_on_rfsoc_arm()
        self.ip_addresses = get_host_ips()
        self.server_start_unix_s = time.time()
        
        # Step 1: Use pipeline_id hint to find default config if none specified
        initial_pipeline_id = pipeline_id if pipeline_id is not None else 0
        
        if config_file is None:
            # Use the hint pipeline_id to find the default config
            hint_dirs = ensure_pipeline_dirs(initial_pipeline_id)
            config_file = hint_dirs['default_config']
            _server_log(f'default config link: {config_file}', source='init')
        
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
        self.process_name = set_process_name(self.pipeline_id)
        
        # Warn if explicit pipeline_id was provided and differs from config
        if pipeline_id is not None and pipeline_id != self.pipeline_id:
            _server_log(
                f'ERROR: explicit pipeline_id {pipeline_id} differs from '
                f'config pipeline_id {self.pipeline_id}',
                source='init',
            )
            _server_log(
                f'using config pipeline_id={self.pipeline_id}; this is what '
                f'the firmware interfaces use',
                source='init',
            )
        
        _server_log_fields('config selected', [
            ('config file', resolved_config_file),
            ('pipeline', self.pipeline_id),
        ], source='init')
        
        # Step 4: Now set up directories based on CONFIG's pipeline_id
        self.pipeline_dirs = ensure_pipeline_dirs(self.pipeline_id)
        self.user_config_dir = self.pipeline_dirs['config']
        self.user_calibrations_dir = self.pipeline_dirs['calibrations']
        self.default_config = self.pipeline_dirs['default_config']

        _server_log_fields(f'pipeline {self.pipeline_id} directories', [
            ('config', self.user_config_dir),
            ('calibrations', self.user_calibrations_dir),
        ], source='init')

        #server attributes
        self.config = None
        self.config_file = None
        self.applied_config = None
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

        #fast frequency modulation state
        self.e_modulation_enabled = asyncio.Event()   # modulation armed (alongside e_stream_enabled)
        self.modulation_params = None                 # prepared bundle (per-point freq words + shared amp/phase + armed maps)
        self.modulation_cfg = None                    # center, offsets, mod_indices, samples_per_point, n_settle
        self.modulation_state = None                  # per-tone observability + revision (served by get_info)
        self.modulation_sched = None                  # ModulationScheduler owning the two control buffers
        self._pending_modulation = None               # latest-wins command applied by the frame producer at a cycle boundary
        self._modulation_owner = None                 # acquisition-owner lock: 'stream' | 'samples' | None
        self._modulation_revision = 0
        self._modulation_revision_history = {}        # revision -> {center, offsets, mod_indices, ts}

        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.active_tone_indices = None
        self.latest_sweep_results = {}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0.0
        self.sweep_state = {'state': 'idle', 'message': 'No sweep in progress'}

        #rf peripheral controller
        self.rf_peripherals = None

        #initialize server (this will load config again, but that's fine)
        self.init_server(resolved_config_file, ensure_ready=True, force_ready=False)
    
    
    def ensure_ready(self, config_file=None, level="pipeline", log_source='ready'):
        """
        Ensure the firmware is ready up to the requested init level.

        Does NOT force reprogramming or reinitialisation if not needed.

        Does not notice if config parameters have changed - but it definitely should!
        
        config_file:
          optional config to (re)load first; None keeps the current config.
        level:
          - "server": no firmware operations
          - "firmware": (re)program if needed, then initialise shared resources
          - "pipeline": firmware level + initialise pipeline resources
        log_source:
          label used to tag this operation's server-log lines.
        """
        if level not in ("server", "firmware", "pipeline"):
            raise ValueError(f"Invalid ready level: {level}")

        self.load_config(config_file, log_source=log_source)

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
                firmware_lib.initialise_pipeline_resources(self.r, self.r_fast, self.config)
                self.applied_config = copy.deepcopy(self.config)

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
            firmware_lib.initialise_pipeline_resources(self.r, self.r_fast, self.config)
            self.applied_config = copy.deepcopy(self.config)

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

        ``config_file`` is the configuration to load on startup.

        If ensure_ready is True, bring system to pipeline-ready state

        If force_ready is True, reprogram and bring system to pipeline-ready state

        """

        _server_log_fields('initialising runtime', [
            ('config file', config_file),
            ('ensure ready', ensure_ready),
            ('force ready', force_ready),
        ], source='init')

        #server attributes
        self.config = None
        self.config_file = None
        self.applied_config = None
        self.request_clients = []
        self.stream_clients = []
        self.sweep_task = None
        self.stream_task = None
        self.triggered_stream_task = None
        self.e_stream_enabled = asyncio.Event()
        self.e_triggered_stream_enabled = asyncio.Event()
        self.tasks = []
        self.stream_flags = [asyncio.Event() for _ in range(8)]

        #fast frequency modulation state
        self.e_modulation_enabled = asyncio.Event()   # modulation armed (alongside e_stream_enabled)
        self.modulation_params = None                 # prepared bundle (per-point freq words + shared amp/phase + armed maps)
        self.modulation_cfg = None                    # center, offsets, mod_indices, samples_per_point, n_settle
        self.modulation_state = None                  # per-tone observability + revision (served by get_info)
        self.modulation_sched = None                  # ModulationScheduler owning the two control buffers
        self._pending_modulation = None               # latest-wins command applied by the frame producer at a cycle boundary
        self._modulation_owner = None                 # acquisition-owner lock: 'stream' | 'samples' | None
        self._modulation_revision = 0
        self._modulation_revision_history = {}        # revision -> {center, offsets, mod_indices, ts}

        #firmware interface attributes
        self.r = None
        self.r_fast = None
        self.active_tone_indices = None
        self.latest_sweep_results = {}
        self.latest_sweep_data_valid = False
        self.sweep_progress = 0.0
        self.sweep_state = {'state': 'idle', 'message': 'No sweep in progress'}

        #load config
        self.load_config(config_file, log_source='init')
        self.server_address = '0.0.0.0'
        self.request_server_port = self.config['rfsoc_host']['request_port']
        self.stream_server_port = self.config['rfsoc_host']['stream_port']
        # trigger_source_pin: prefer firmware section, fall back to rfsoc_host for older configs
        self.trigger_source_pin = self.config.get('firmware', {}).get(
            'trigger_source_pin',
            self.config.get('rfsoc_host', {}).get('trigger_source_pin', 0)
        )
        
        #interface with firmware
        fw_config_file = self.config['firmware']['fw_config_file']
        pipeline_id = self.config['firmware']['pipeline_id']

        self.r = firmware_lib.create_standard_readout_interface(fw_config_file,pipeline_id)
        self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file,pipeline_id)
        

        if ensure_ready and force_ready:
            ensure_ready=False

        if ensure_ready:
            try:
                self.ensure_ready(level="pipeline", log_source='init')
            except Exception as e:
                _server_log(f'ensure_ready failed: {e}', source='init')
                _server_log(
                    'server remains at server init level; use a client to diagnose '
                    'and retry (ensure_ready / hard_reset)',
                    source='init',
                )

        if force_ready:
            self.force_ready(level='pipeline')


        #see if we can succesfully load system information
        try:
            self.get_info()
            self.update_active_tone_indices()
        except Exception as e:
            _server_log(f'warning: could not get system information from firmware: {e}', source='init')
            _server_log('try hard reset', source='init')

        firmware_needs_programming = firmware_lib.needs_programming(
            self.r, self.config, verbose=True
        )
        shared_needs_initialising = firmware_lib.needs_shared_resource_initialising(
            self.r, self.config, verbose=True
        )
        pipeline_needs_initialising = firmware_lib.needs_pipeline_initialising(
            self.r, self.config, verbose=True
        )

        if firmware_needs_programming:
            _server_log('warning: firmware needs programming', source='init')
        if shared_needs_initialising:
            _server_log('warning: shared resources need initialising', source='init')
        if pipeline_needs_initialising:
            _server_log('warning: pipeline resources need initialising', source='init')

        # initialise rf peripheral controller (attenuators, amp bypass)
        try:
            self.rf_peripherals = RFPeripheralController(self.config, self.pipeline_id)
        except Exception as e:
            _server_log(f'warning: init failed: {e}', source='rf')
            self.rf_peripherals = None

        if self.rf_peripherals is not None and self.rf_peripherals.enabled:
            _server_log(
                f'frontend initialised: {self.rf_peripherals.attenuator_backend}, '
                f'hardware={_format_log_value(self.rf_peripherals.is_hardware)}',
                source='rf',
            )
            if (self.rf_peripherals.is_hardware
                    or self.rf_peripherals.attenuator_backend == 'fixed'):
                status = self.rf_peripherals.get_status()
                tx_bypass = _format_log_value(status.get('tx_amp_bypass', 'unknown'))
                rx_bypass = _format_log_value(status.get('rx_amp_bypass', 'unknown'))
                _server_log(
                    f'  TX: atten={status["tx_attenuation_db"]:.1f} dB, '
                    f'amp_bypass={tx_bypass}, '
                    f'total_gain_model={status["tx_total_gain_db"]:.1f} dB',
                    source='rf',
                )
                _server_log(
                    f'  RX: atten={status["rx_attenuation_db"]:.1f} dB, '
                    f'amp_bypass={rx_bypass}, '
                    f'total_gain_model={status["rx_total_gain_db"]:.1f} dB',
                    source='rf',
                )

        # initialise LNA bias controller
        try:
            self.lna_controller = LNABiasController(self.config, self.pipeline_id)
        except Exception as e:
            _server_log(f'warning: bias init failed: {e}', source='lna')
            self.lna_controller = None

        if self.lna_controller is not None and self.lna_controller.enabled:
            _server_log(
                f'bias controller initialised: {self.lna_controller.backend}, '
                f'channel={self.lna_controller.lna_channel}, '
                f'hardware={_format_log_value(self.lna_controller.is_hardware)}',
                source='lna',
            )

        return
   
             
    def init_firmware(self,config_file=None):
        """
        Reprogram firmware (optionally using config_file), then init shared fw resources.
        Does NOT implicitly also init pipeline unless you request ensure_ready("pipeline").
        """
        _server_log_fields('initialising firmware', [
            ('config file', config_file),
        ], source='firmware')
        
        if config_file is not None:
            self.load_config(config_file, log_source='firmware')
       
        self.force_ready(level="firmware")

        return


    def init_pipeline(self, config_file=None):
        """
        Ensure shared resources and pipeline resources are initialised.
        Does not force a reprogram unless needs_programming() says so.
        Does not force shared resource initialisation unless needs_shared_resource_initialising() says so.

        ``config_file`` optionally (re)loads a config first; ``None`` keeps the
        current one.
        """
        _server_log_fields('initialising pipeline', [
            ('config file', config_file),
        ], source='pipeline')

        if config_file is not None:
            self.load_config(config_file, log_source='pipeline')
            # rebuild interfaces in case pipeline_id / fw_config_file changed
            fw_config_file = self.config['firmware']['fw_config_file']
            pipeline_id = self.config['firmware']['pipeline_id']
            self.r = firmware_lib.create_standard_readout_interface(fw_config_file, pipeline_id)
            self.r_fast = firmware_lib.create_fast_readout_interface(fw_config_file, pipeline_id)

        
        #ensure pipeline is ready
        self.ensure_ready(level="pipeline", log_source='pipeline')
    
        #force pipeline init
        firmware_lib.initialise_pipeline_resources(self.r, self.r_fast, self.config)
        self.applied_config = copy.deepcopy(self.config)

        return



    def _resolve_request_config_file(self, config_file):
        """Resolve the config file for an init/reset request, with fallback.

        If the caller supplied an explicit path, it is returned (or rejected
        upstream if missing). If the caller supplied None, fall back to
        ``self.config_file`` if it still exists on disk, otherwise to
        ``default_config.lnk``. This handles the case where the active config
        file was deleted out from under the server.
        """
        if config_file is not None:
            return config_file
        if self.config_file and os.path.exists(self.config_file):
            return self.config_file
        if self.default_config and os.path.exists(self.default_config):
            if self.config_file:
                _server_log(
                    f'active config {self.config_file} no longer exists; '
                    f'falling back to {self.default_config}',
                    source='config',
                )
            return self.default_config
        return self.config_file

    def load_config(self, config_file, log_source='config'):
        """
        Load a new configuration file.
        Does not reload or initialise the firmware.
        Uses pipeline-specific directories.

        ``config_file`` is the config path to load (``None`` falls back to the
        pipeline default); ``log_source`` tags this operation's server-log lines.
        """
        requested_config_file = config_file
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
        if requested_config_file != config_file:
            _server_log(
                f'config loaded: {config_file} (requested {requested_config_file or "default"})',
                source=log_source,
            )
        else:
            _server_log(f'config loaded: {config_file}', source=log_source)
        self.config = config
        self.config_file = config_file
        with open(config_file, 'r') as file:
            self.config_raw_text = file.read()

        return config

    def _render_config_text(self):
        """YAML text for the active config file.

        ``pull_config()`` deliberately returns the desired/applied config file,
        not a live-state patched view. Runtime hardware state is available via
        status/info calls and can be explicitly captured by the client.
        """
        if self.config_file and os.path.exists(self.config_file):
            with open(self.config_file, 'r') as file:
                return file.read()
        raw = getattr(self, 'config_raw_text', None)
        if raw:
            return raw
        return yaml.dump(self.config, sort_keys=False)

    def set_config(self, config_filename, config_contents, default=True):
        """
        Apply a new configuration and re-initialise the firmware
        If default is true, overwrites the default_config.lnk so that this config is persistent
        Uses pipeline-specific directories.

        Now also applies any modified config parameters in hardware.

        ``config_filename`` is the name to save the config under and
        ``config_contents`` is its text (e.g. uploaded by a client).
        """
        _server_log_fields('applying config', [
            ('config filename', config_filename),
        ], source='config')

        filename = os.path.join(self.user_config_dir, os.path.basename(config_filename))
        with open( filename, 'w') as file:
            file.write(yaml.dump(config_contents,sort_keys=False))
        os.chmod(filename,0o664)

        if SUDO:
            os.chown(filename,int(TARGET_UID),int(TARGET_GID))

        _server_log(f'saved config: {filename}', source='config')


        firmware_lib.apply_config(config_contents, self.r, self.r_fast, self.applied_config)
        self.applied_config = copy.deepcopy(config_contents)
        self.update_active_tone_indices()

        self.ensure_ready(config_file=filename, level="pipeline", log_source='config')

        # Rebind/rebuild RF peripheral control after ensure_ready() reloads
        # self.config, then apply the requested RF settings to that live dict.
        try:
            self.rf_peripherals = RFPeripheralController(self.config, self.pipeline_id)
            if self.rf_peripherals.enabled:
                self.rf_peripherals.apply_config(config_contents)
        except Exception as e:
            _server_log(f'warning: re-init failed: {e}', source='rf')
            self.rf_peripherals = None

        try:
            self.lna_controller = LNABiasController(self.config, self.pipeline_id)
            if self.lna_controller.enabled:
                self.lna_controller.apply_config(config_contents)
        except Exception as e:
            _server_log(f'warning: bias re-init failed: {e}', source='lna')
            self.lna_controller = None

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
        

    # ------------------------------------------------------------------
    # Structured info system
    # ------------------------------------------------------------------

    DEFAULT_INFO_SECTIONS = [
        'server', 'versions', 'clock', 'timing', 'fpga', 'rfdc',
        'pipeline', 'tones', 'rf_frontend', 'lna', 'rfsoc_sensors',
    ]
    ALL_INFO_SECTIONS = DEFAULT_INFO_SECTIONS + [
        'diagnostics', 'config', 'calibrations', 'resonators', 'registers',
        'tone_modulation',
    ]

    def get_info(self, sections=None):
        """Return system information organised by named sections.

        Parameters
        ----------
        sections : str, list of str, or ``'all'``, optional
            Which sections to include.  ``None`` returns
            ``DEFAULT_INFO_SECTIONS`` (fast path — excludes diagnostics,
            config, calibrations, resonators, and registers).
            ``'all'`` returns every section including expensive ones.
            A single section name returns that section dictionary directly.
            A list returns a list of section dictionaries in the same order.
        """
        dispatchers = {
            'server':       self._info_server,
            'versions':     self._info_versions,
            'clock':        self._info_clock,
            'timing':       self._info_timing,
            'fpga':         self._info_fpga,
            'rfdc':         self._info_rfdc,
            'pipeline':     self._info_pipeline,
            'tones':        self._info_tones,
            'rf_frontend':  self._info_rf_frontend,
            'lna':          self._info_lna,
            'rfsoc_sensors': self._info_rfsoc_sensors,
            'diagnostics':  self._info_diagnostics,
            'config':       self._info_config,
            'calibrations': self._info_calibrations,
            'resonators':   self._info_resonators,
            'registers':    self._info_registers,
            'tone_modulation': self._info_tone_modulation,
        }
        if sections is None:
            return {
                s: dispatchers[s]()
                for s in self.DEFAULT_INFO_SECTIONS
                if s in dispatchers
            }
        if sections == 'all':
            return {
                s: dispatchers[s]()
                for s in self.ALL_INFO_SECTIONS
                if s in dispatchers
            }
        if isinstance(sections, str):
            if sections not in dispatchers:
                return {}
            return dispatchers[sections]()
        return [dispatchers[s]() for s in sections if s in dispatchers]

    def health_check(self):
        """Compact health summary for intermittent polling."""
        # Initialisation level
        programmed = self.r is not None and self.r.fpga.is_programmed()
        shared_ready = programmed and hasattr(self.r, 'autocorr')
        pipeline_ready = (shared_ready and hasattr(self.r, 'accumulators')
                          and len(self.r.accumulators) > 0
                          and self.r.accumulators[0].get_acc_len() > 0)
        if pipeline_ready:
            init_level = 'pipeline'
        elif shared_ready:
            init_level = 'shared'
        elif programmed:
            init_level = 'programmed'
        else:
            init_level = 'not_programmed'

        clock = firmware_lib.get_clock_status()
        timing = get_timing_status(timeout_s=0.2)

        # Diagnostics — only run if pipeline is ready
        adc_sat = dac_sat = dsp_ovf = False
        if pipeline_ready:
            try:
                adc_sat, _ = firmware_lib.check_input_saturation(
                    self.r, self.r_fast, iterations=10, verbose=False)
            except Exception:
                pass
            try:
                dac_sat, _ = firmware_lib.check_output_saturation(
                    self.r_fast, iterations=10, verbose=False)
            except Exception:
                pass
            try:
                dsp_ovf, _ = firmware_lib.check_dsp_overflow(
                    self.r, duration_s=0.05, verbose=False)
            except Exception:
                pass

        # RTS events
        rts_any = False
        try:
            rts_any, _ = firmware_lib.check_rfdc_rts_events(self.r, clear=False)
        except Exception:
            pass

        # Tone count
        tone_count = 0
        if pipeline_ready:
            try:
                tone_count = len(firmware_lib.get_tone_frequencies(self.r, self.config))
            except Exception:
                pass

        return {
            'initialisation_level': init_level,
            'clock_locked': clock.get('all_locked', False),
            'timing_ready': timing.get('ready_for_firmware_sync', False),
            'timing_state': timing.get('state'),
            'streaming': self.e_stream_enabled.is_set() and self.stream_task is not None and not self.stream_task.done(),
            'triggered_streaming': self.e_triggered_stream_enabled.is_set() and self.triggered_stream_task is not None and not self.triggered_stream_task.done(),
            'modulation_streaming': self.e_modulation_enabled.is_set(),
            'sweeping': self.sweep_task is not None and not self.sweep_task.done(),
            'rts_events': rts_any,
            'adc_saturated': adc_sat,
            'dac_saturated': dac_sat,
            'dsp_overflow': dsp_ovf,
            'rf_frontend_available': (getattr(self, 'rf_peripherals', None) is not None
                                      and self.rf_peripherals.enabled
                                      and self.rf_peripherals.is_hardware),
            'lna_available': (getattr(self, 'lna_controller', None) is not None
                              and self.lna_controller.enabled
                              and self.lna_controller.is_hardware),
            'tone_count': tone_count,
            'client_count': len(self.request_clients),
            'resonators_tracking': False,  # placeholder until tracking module
            'max_detuning_hz': None,       # placeholder
        }

    # -- Section helpers for get_info --

    def _info_server(self):
        programmed = self.r is not None and self.r.fpga.is_programmed()
        shared_ready = programmed and hasattr(self.r, 'autocorr')
        pipeline_ready = (shared_ready and hasattr(self.r, 'accumulators')
                          and len(self.r.accumulators) > 0
                          and self.r.accumulators[0].get_acc_len() > 0)
        if pipeline_ready:
            init_level = 'pipeline'
        elif shared_ready:
            init_level = 'shared'
        elif programmed:
            init_level = 'programmed'
        else:
            init_level = 'not_programmed'

        now_unix_s = time.time()
        try:
            with open('/proc/uptime', 'r') as fh:
                board_uptime_s = float(fh.read().split()[0])
        except Exception:
            board_uptime_s = None

        return {
            'ready': True,
            'process_name': self.process_name,
            'ip_addresses': self.ip_addresses,
            'pipeline_id': self.pipeline_id,
            'pipeline_dirs': self.pipeline_dirs,
            'pwd': os.getcwd(),
            'sys_executable': sys.executable,
            'sys_argv': sys.argv,
            'uname': (os.uname().nodename + ' ' + os.uname().sysname + ' '
                      + os.uname().release + ' ' + os.uname().version
                      + ' ' + os.uname().machine),
            'python_version': sys.version,
            'server_version': importlib.metadata.version('souk_readout_tools'),
            'config_file': self.config_file,
            'initialisation_level': init_level,
            'current_time_unix_s': now_unix_s,
            'current_time_iso': datetime.datetime.fromtimestamp(now_unix_s, datetime.timezone.utc).isoformat(),
            'server_start_unix_s': self.server_start_unix_s,
            'server_start_iso': datetime.datetime.fromtimestamp(self.server_start_unix_s, datetime.timezone.utc).isoformat(),
            'server_uptime_s': now_unix_s - self.server_start_unix_s,
            'board_uptime_s': board_uptime_s,
            'request_clients': len(self.request_clients),
            'request_client_addrs': [c.get_extra_info('peername') for c in self.request_clients],
            'stream_clients': len(self.stream_clients),
            'stream_client_addrs': [c.get_extra_info('peername') for c in self.stream_clients],
            'streaming': self.e_stream_enabled.is_set() and self.stream_task is not None and not self.stream_task.done(),
            'triggered_streaming': (self.e_triggered_stream_enabled.is_set()
                                    and self.triggered_stream_task is not None
                                    and not self.triggered_stream_task.done()),
            'modulation_streaming': self.e_modulation_enabled.is_set(),
            'sweeping': self.sweep_task is not None and not self.sweep_task.done(),
            'task_count': len(self.tasks),
            'latest_sweep_data_valid': self.latest_sweep_data_valid,
            'firmware_interface_exists': bool(self.r),
            'firmware_interface_ready': bool(self.r) and hasattr(self.r, 'accumulators'),
            'firmware_fast_interface_exists': bool(self.r_fast),
            'firmware_fast_interface_ready': bool(self.r_fast) and hasattr(self.r_fast, 'adc_clk_hz'),
        }

    def _info_versions(self):
        return firmware_lib.info_versions()

    def _info_clock(self):
        return firmware_lib.info_clock()

    def _info_timing(self):
        return get_timing_summary(timeout_s=0.2)

    def _info_fpga(self):
        return firmware_lib.info_fpga(self.r)

    def _info_rfdc(self):
        return firmware_lib.info_rfdc(self.r, self.config)

    def _info_pipeline(self):
        return firmware_lib.info_pipeline(self.r)

    def _info_tones(self):
        info = firmware_lib.info_tones(
            self.r, self.config, rf_peripherals=self.rf_peripherals)
        # Compact modulation hint so a get_info('tones') caller sees the state
        # without the full per-tone detail (which lives in 'tone_modulation').
        state = self.modulation_state
        try:
            info['modulation'] = {
                'enabled': bool(self.e_modulation_enabled.is_set()),
                'any_at_limit': bool(state.get('needs_recenter')) if state else False,
            }
        except Exception:
            pass
        return info

    def _info_tone_modulation(self):
        """
        Return the cached fast-frequency-modulation state for
        ``get_info('tone_modulation')``.

        This is a pure read of ``self.modulation_state`` (assembled by
        :meth:`_build_modulation_state` when a config was prepared) — it performs
        **no hardware access**, so a client can poll it without perturbing the
        streaming hot loop. ``enabled`` is refreshed from the live event so it
        always reflects the current armed/paused state. Returns a minimal
        ``{'enabled': False, ...}`` stub when modulation has never been armed.
        """
        state = self.modulation_state
        if state is None:
            return {'enabled': False, 'num_points': 0, 'tones': []}
        # Reflect the live armed flag (state snapshot may predate a pause/resume).
        state['enabled'] = bool(self.e_modulation_enabled.is_set())
        return state

    def _info_rf_frontend(self):
        rf = getattr(self, 'rf_peripherals', None)
        rf_cfg = self.config.get('rf_frontend', {})
        attn_cfg   = rf_cfg.get('attenuator', {}) or {}
        mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}

        if rf is None or not rf.enabled:
            return {'ready': False, 'connected': rf_cfg.get('connected', False)}

        status = rf.get_status()
        info = {
            'ready': True,
            'connected': rf_cfg.get('connected', False),
            'hardware_id': rf_cfg.get('hardware_id'),
            'hardware_available': rf.is_hardware,
            'controllable': rf.is_controllable,
            'supports_bypass_amps': rf.supports_bypass_amps,
            'attenuator_backend': rf.attenuator_backend,
            'rf_channel': status.get('rf_channel', mixerless_cfg.get('rf_channel')),
        }

        # Backend-specific identity
        if rf.attenuator_backend == 'rudat':
            info['rudat_tx_serial'] = attn_cfg.get('rudat_tx_serial')
            info['rudat_rx_serial'] = attn_cfg.get('rudat_rx_serial')

        # Live state from hardware, or explicit fixed values from config.
        if rf.is_hardware or rf.attenuator_backend == 'fixed':
            tx_total_gain_model = status.get('tx_total_gain_db')
            rx_total_gain_model = status.get('rx_total_gain_db')
            tx_total_gain_cal = firmware_lib.estimate_rf_total_gain_from_config(
                self.config, status, 'tx')
            rx_total_gain_cal = firmware_lib.estimate_rf_total_gain_from_config(
                self.config, status, 'rx')
            info['tx_attenuation_db'] = status.get('tx_attenuation_db')
            info['rx_attenuation_db'] = status.get('rx_attenuation_db')
            info['tx_total_gain_db'] = (
                tx_total_gain_cal
                if tx_total_gain_cal is not None else tx_total_gain_model
            )
            info['rx_total_gain_db'] = (
                rx_total_gain_cal
                if rx_total_gain_cal is not None else rx_total_gain_model
            )
            info['tx_total_gain_source'] = (
                'calibrated_config'
                if tx_total_gain_cal is not None else 'peripheral_model'
            )
            info['rx_total_gain_source'] = (
                'calibrated_config'
                if rx_total_gain_cal is not None else 'peripheral_model'
            )
            info['tx_total_gain_model_db'] = tx_total_gain_model
            info['rx_total_gain_model_db'] = rx_total_gain_model
            info['tx_total_gain_calibrated_estimate_db'] = tx_total_gain_cal
            info['rx_total_gain_calibrated_estimate_db'] = rx_total_gain_cal
            info['tx_input_1db_comp_dbm'] = status.get('tx_input_1db_comp_dbm')
            info['rx_input_1db_comp_dbm'] = status.get('rx_input_1db_comp_dbm')
        else:
            info['tx_attenuation_db'] = None
            info['rx_attenuation_db'] = None
            info['tx_total_gain_db'] = None
            info['rx_total_gain_db'] = None
            info['tx_total_gain_source'] = None
            info['rx_total_gain_source'] = None
            info['tx_total_gain_model_db'] = None
            info['rx_total_gain_model_db'] = None
            info['tx_total_gain_calibrated_estimate_db'] = None
            info['rx_total_gain_calibrated_estimate_db'] = None
            info['tx_input_1db_comp_dbm'] = None
            info['rx_input_1db_comp_dbm'] = None

        # Bypass-amp state (only when the mixerless module is the active frontend)
        if rf.supports_bypass_amps:
            tx_amp_bypass = status.get('tx_amp_bypass')
            rx_amp_bypass = status.get('rx_amp_bypass')
            tx_amp_model = status.get('tx_bypass_amp_s21_db')
            rx_amp_model = status.get('rx_bypass_amp_s21_db')
            tx_amp_cal = (
                firmware_lib.estimate_rf_bypass_amp_s21_from_config(
                    self.config, 'tx', bool(tx_amp_bypass))
                if tx_amp_bypass is not None else None
            )
            rx_amp_cal = (
                firmware_lib.estimate_rf_bypass_amp_s21_from_config(
                    self.config, 'rx', bool(rx_amp_bypass))
                if rx_amp_bypass is not None else None
            )
            info['tx_amp_bypass'] = status.get('tx_amp_bypass')
            info['rx_amp_bypass'] = status.get('rx_amp_bypass')
            info['tx_bypass_amp_s21_db'] = (
                tx_amp_cal if tx_amp_cal is not None else tx_amp_model
            )
            info['rx_bypass_amp_s21_db'] = (
                rx_amp_cal if rx_amp_cal is not None else rx_amp_model
            )
            info['tx_bypass_amp_s21_source'] = (
                'calibrated_config'
                if tx_amp_cal is not None else 'peripheral_model'
            )
            info['rx_bypass_amp_s21_source'] = (
                'calibrated_config'
                if rx_amp_cal is not None else 'peripheral_model'
            )
            info['tx_bypass_amp_s21_model_db'] = tx_amp_model
            info['rx_bypass_amp_s21_model_db'] = rx_amp_model
            info['tx_bypass_amp_s21_calibrated_estimate_db'] = tx_amp_cal
            info['rx_bypass_amp_s21_calibrated_estimate_db'] = rx_amp_cal

        # Updownconverter characterisation (from config, flat keys)
        for key in ('tx_mixer_lo_frequency_hz', 'rx_mixer_lo_frequency_hz',
                     'tx_mixer_sideband', 'rx_mixer_sideband',
                     'tx_mixer_conversion_loss_db', 'rx_mixer_conversion_loss_db',
                     'tx_combiner_loss_db', 'rx_combiner_loss_db',
                     'tx_if_s21_db', 'rx_if_s21_db',
                     'tx_rf_s21_db', 'rx_rf_s21_db',
                     'loopback'):
            info[key] = rf_cfg.get(key)

        # Mixerless-module measured calibration overrides.
        for key in ('tx_amp_enabled_s21_db', 'tx_amp_bypassed_s21_db',
                     'tx_amp_bypass_delta_s21_db',
                     'rx_amp_enabled_s21_db', 'rx_amp_bypassed_s21_db',
                     'rx_amp_bypass_delta_s21_db',
                     'tx_input_1db_comp_dbm', 'rx_input_1db_comp_dbm',
                     'tx_group_delay_ns', 'rx_group_delay_ns'):
            info[key] = mixerless_cfg.get(key)

        return info

    def _info_lna(self):
        lna = getattr(self, 'lna_controller', None)
        cryo_cfg = self.config.get('cryostat', {})
        lna_cfg = cryo_cfg.get('lna_bias', {})

        if lna is None or not lna.enabled:
            return {
                'ready': False,
                'enabled': lna_cfg.get('enabled', False),
                'cryostat_connected': cryo_cfg.get('connected', False),
            }

        lna_status = lna.get_status()
        info = {
            'ready': True,
            'enabled': True,
            'cryostat_connected': cryo_cfg.get('connected', False),
            'hardware_available': lna.is_hardware,
            'controllable': lna.is_controllable,
            'backend': lna.backend,
            'lna_channel': lna.lna_channel,
            'bias_voltage_v': lna_status.get('bias_voltage_v'),
            'soft_off': lna_status.get('soft_off', False),
            'method': lna_status.get('method', lna.DEFAULT_METHOD),
            'blind': lna_status.get('blind', lna.DEFAULT_BLIND),
            'lna_model': cryo_cfg.get('lna_model'),
        }

        # Bias readings
        if lna.is_hardware:
            try:
                info['bias_readings'] = lna.get_lna_bias_status_all()
            except Exception:
                info['bias_readings'] = None
        elif lna.backend == 'fixed':
            info['bias_readings'] = {
                lna.lna_channel: lna.get_lna_bias_status()
            }
        else:
            info['bias_readings'] = None

        return info

    def _info_rfsoc_sensors(self):
        return firmware_lib.info_rfsoc_sensors()

    def _info_diagnostics(self):
        return firmware_lib.info_diagnostics(self.r, self.r_fast, self.config)

    def _info_config(self):
        config_text = self._render_config_text()
        config_id = None
        try:
            config_id = self.config.get('config', {}).get('config_id')
        except Exception:
            pass

        matches_applied = False
        if self.applied_config is not None and self.config is not None:
            try:
                matches_applied = self.config == self.applied_config
            except Exception:
                pass

        return {
            'ready': True,
            'config_file': self.config_file,
            'config_id': config_id,
            'config_text': config_text,
            'config_matches_applied': matches_applied,
        }

    def _info_calibrations(self):
        return firmware_lib.info_calibrations(self.r, self.config)

    def _info_resonators(self):
        """Resonator detuning tracking — placeholder until tracking module."""
        return {
            'ready': False,
            'tracking_enabled': False,
            'tone_count': 0,
            'driven_frequencies_hz': None,
            'estimated_resonant_frequencies_hz': None,
            'detuning_hz': None,
            'fractional_detuning': None,
            'accumulated_phase_rad': None,
            'tracking_timestamp': None,
            'tracking_interval_s': None,
        }

    def _info_registers(self):
        """Full firmware register dump — placeholder."""
        return {'ready': False, 'available': False, 'dump': None}

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
                message = json.loads(data.decode())
                request = message.get('request')
                print(f"request: {_format_request_log(message)}")

                if request == 'ensure_ready':
                    level = message.get('level', 'pipeline')
                    config_file = self._resolve_request_config_file(message.get('config_filename', None))
                    if not config_file or not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.ensure_ready(config_file=config_file, level=level)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'hard_reset':
                    config_file = self._resolve_request_config_file(message.get('config_filename', None))
                    if not config_file or not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_server(config_file,ensure_ready=False, force_ready=True)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'initialise_server':
                    config_file = self._resolve_request_config_file(message.get('config_filename'))
                    if not config_file or not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        self.init_server(config_file,ensure_ready=False, force_ready=False)
                        await self.send_response(writer, {'status': 'success'})

                elif request == 'initialise_firmware':
                    config_file = self._resolve_request_config_file(message.get('config_filename'))
                    if not config_file or not os.path.exists(config_file):
                        await self.send_response(writer, {'status': 'error', 'message': f'Config file {config_file} does not exist on RFSoC'})
                    else:
                        await self.send_response(writer, {'status': 'success'})
                        self.init_firmware(config_file)

                elif request == 'initialise_pipeline':
                    config_file = self._resolve_request_config_file(message.get('config_filename'))
                    if not config_file or not os.path.exists(config_file):
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
                    config_text = self._render_config_text()
                    await self.send_response(writer, {'status': 'success', 'config_filename': self.config_file, 'config_contents': config_text})

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

                elif request == 'get_info':
                    sections = message.get('sections', None)
                    info = self.get_info(sections)
                    await self.send_response(writer, {'status': 'success', 'data': info})

                elif request == 'health_check':
                    result = self.health_check()
                    await self.send_response(writer, {'status': 'success', 'data': result})

                elif request == 'get_timing_status':
                    result = get_timing_status()
                    await self.send_response(writer, {'status': 'success', 'data': result})

                elif request == 'get_blind_tones':
                    ref_plane = message.get('reference_plane', 'detector')
                    result = self.get_blind_tones(reference_plane=ref_plane)
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_blind_tones':
                    self.stream_flags[FLAG_SET_FREQS].set()
                    self.stream_flags[FLAG_SET_AMPS].set()
                    self.stream_flags[FLAG_SET_PHASES].set()
                    await asyncio.sleep(0)
                    try:
                        result = self.set_blind_tones(
                            message.get('frequencies', []),
                            amplitudes=message.get('amplitudes', None),
                            phases=message.get('phases', None),
                            spans=message.get('spans', None),
                            powers_dbm=message.get('powers_dbm', None),
                            reference_plane=message.get('reference_plane', 'detector'),
                            optimise_dynamic_range=message.get(
                                'optimise_dynamic_range', False),
                            rx_policy=message.get('rx_policy', 'protect'))
                    finally:
                        self.stream_flags[FLAG_SET_PHASES].clear()
                        self.stream_flags[FLAG_SET_AMPS].clear()
                        self.stream_flags[FLAG_SET_FREQS].clear()
                    await asyncio.sleep(0)
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'remove_blind_tones':
                    self.stream_flags[FLAG_SET_FREQS].set()
                    await asyncio.sleep(0)
                    try:
                        result = self.remove_blind_tones()
                    finally:
                        self.stream_flags[FLAG_SET_FREQS].clear()
                    await asyncio.sleep(0)
                    await self.send_response(writer, {'status': 'success', 'result': result})

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
                    elif param_name == 'tone_metadata':
                        active_count = len(firmware_lib.get_tone_frequencies(self.r, self.config))
                        value = firmware_lib.get_configured_tone_metadata(
                            self.config, active_count=active_count)
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'tone_amplitudes':
                        value = firmware_lib.get_tone_amplitudes(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_phases':
                        value = firmware_lib.get_tone_phases(self.r,self.config)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_powers':
                        ref_plane = message.get('reference_plane', 'detector')
                        value = firmware_lib.get_tone_powers(self.r,self.config,reference_plane=ref_plane,rf_peripherals=self.rf_peripherals)
                        response = {'status': 'success', 'value': value.tolist()}
                    elif param_name == 'tone_powers_detailed':
                        ref_plane = message.get('reference_plane', 'detector')
                        value = firmware_lib.get_tone_powers(self.r,self.config,detailed_output=True,reference_plane=ref_plane,rf_peripherals=self.rf_peripherals)[1]
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'telescope_time':
                        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
                        value = firmware_lib.read_tt_fast(fast_read_params)
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'cal_freeze':
                        value = firmware_lib.get_cal_freeze(self.r,self.config)
                        if value:
                            self.stream_flags[FLAG_CAL_FREEZE].set()
                        else:
                            self.stream_flags[FLAG_CAL_FREEZE].clear()
                        response = {'status': 'success', 'value': value}

                    elif param_name == 'clock_source':
                        value = firmware_lib.get_clock_source()
                        response = {'status': 'success', 'value': value}

                    elif param_name == 'clock_status':
                        value = firmware_lib.get_clock_status()
                        response = {'status': 'success', 'value': value}

                    elif param_name == 'sync_delay':
                        value = self.r.sync.get_delay()
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'acc_len':
                        value = self.r.accumulators[0].get_acc_len()
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'internal_loopback':
                        value = self.r.input.loopback_enabled()
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'psb_scale':
                        value = self.r.psbscale.get_scale()
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'psb_fftshift':
                        value = self.r.psb.get_fftshift()
                        response = {'status': 'success', 'value': value}
                    elif param_name == 'pfb_fftshift':
                        value = self.r.pfb.get_fftshift()
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
                        param_value = self._expand_frequencies_for_blind(param_value)
                        firmware_lib.set_tone_frequencies_fast(self.r, self.r_fast, self.config, param_value)
                        self.update_active_tone_indices()
                        self.stream_flags[FLAG_SET_FREQS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}
                    
                    elif param_name == 'tone_amplitudes':
                        self.stream_flags[FLAG_SET_AMPS].set()
                        await asyncio.sleep(0)
                        param_value = self._expand_values_for_blind(
                            param_value, firmware_lib.get_tone_amplitudes,
                            plan_values_key='amplitudes', default_value=1.0)
                        firmware_lib.set_tone_amplitudes(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_AMPS].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}
                    
                    elif param_name == 'tone_phases':
                        self.stream_flags[FLAG_SET_PHASES].set()
                        await asyncio.sleep(0)
                        param_value = self._expand_values_for_blind(
                            param_value, firmware_lib.get_tone_phases,
                            plan_values_key='phases', default_value=0.0)
                        firmware_lib.set_tone_phases(self.r, self.config, param_value)
                        self.stream_flags[FLAG_SET_PHASES].clear()
                        await asyncio.sleep(0)
                        response = {'status': 'success'}

                    elif param_name == 'tone_powers':
                        self.stream_flags[FLAG_SET_AMPS].set()
                        await asyncio.sleep(0)
                        ref_plane = message.get('reference_plane', 'detector')
                        opt_dr = message.get('optimise_dynamic_range', False)
                        rx_pol = message.get('rx_policy', 'protect')
                        param_value = self._expand_values_for_blind(
                            param_value, firmware_lib.get_tone_powers,
                            getter_kwargs={'reference_plane': ref_plane,
                                           'rf_peripherals': self.rf_peripherals},
                            plan_values_key=None, default_value=None)
                        result = firmware_lib.set_tone_powers(
                            self.r, self.r_fast, self.config, param_value,
                            reference_plane=ref_plane,
                            optimise_dynamic_range=opt_dr,
                            rf_peripherals=self.rf_peripherals,
                            rx_policy=rx_pol,
                            **{k: message[k] for k in firmware_lib.POWER_FORCE_CONTROL_KEYS
                               if k in message})
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
                    
                    elif param_name == 'burst_mode':
                        firmware_lib.set_burst_mode(self.r, param_value)
                        response = {'status': 'success'}

                    elif param_name == 'clock_source':
                        try:
                            status = firmware_lib.set_clock_source(param_value)
                            response = {'status': 'success', 'clock_status': status}
                        except (ValueError, FileNotFoundError) as exc:
                            response = {'status': 'error', 'message': str(exc)}

                    elif param_name == 'sync_delay':
                        self.r.sync.set_delay(int(param_value))
                        response = {'status': 'success'}
                    elif param_name == 'acc_len':
                        self.r.accumulators[0].set_acc_len(int(param_value))
                        response = {'status': 'success'}
                    elif param_name == 'internal_loopback':
                        if param_value:
                            self.r.input.enable_loopback()
                        else:
                            self.r.input.disable_loopback()
                        response = {'status': 'success'}
                    elif param_name == 'psb_scale':
                        self.r.psbscale.set_scale(int(param_value))
                        response = {'status': 'success'}
                    elif param_name == 'psb_fftshift':
                        self.r.psb.set_fftshift(int(param_value))
                        response = {'status': 'success'}
                    elif param_name == 'pfb_fftshift':
                        self.r.pfb.set_fftshift(int(param_value))
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

                elif request == 'enable_modulation':
                    # Arm fast frequency modulation. Arms only — does NOT start
                    # continuous output (call enable_stream) and does NOT itself
                    # capture (call get_samples). Heavy prep runs off the hot path.
                    try:
                        if (message.get('offsets') is None and message.get('center') is None
                                and self.modulation_cfg is not None):
                            c = self.modulation_cfg          # resume a resident config
                            center, offsets = c['center'], c['offsets']
                            mod_indices = c['mod_indices']
                            spp, n_settle = c['samples_per_point'], c['n_settle']
                        else:
                            center = message.get('center')
                            offsets = message.get('offsets')
                            mod_indices = message.get('mod_indices')
                            spp = int(message.get('samples_per_point', 1))
                            n_settle = int(message.get('n_settle', 1))
                            if offsets is None:
                                raise ValueError('offsets required to arm modulation')
                        bundle, state, cfg = await self.to_thread(
                            self._prepare_modulation, center, offsets, mod_indices, spp, n_settle)
                        rev = self._next_modulation_revision(cfg)
                        state['enabled'] = True
                        state['desired_revision'] = rev
                        state['revision_history'] = dict(self._modulation_revision_history)
                        self._pending_modulation = {
                            'op': 'enable', 'bundle': bundle, 'state': state, 'cfg': cfg, 'revision': rev}
                        self.e_triggered_stream_enabled.clear()
                        self.e_modulation_enabled.set()
                        await self.send_response(writer, {'status': 'success', 'result': {
                            'revision': rev, 'needs_recenter': bundle['needs_recenter']}})
                    except Exception as e:
                        print(traceback.format_exc())
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'update_modulation':
                    # Seamless live update of centre and/or offsets. Rides the
                    # existing armed maps; if any (tone,point) would drift beyond
                    # bin coverage it is rejected unless on_map_change='recenter'.
                    try:
                        if self.modulation_cfg is None or self.modulation_params is None:
                            raise ValueError('modulation not armed; call enable_modulation first')
                        c = self.modulation_cfg
                        center = message.get('center', c['center'])
                        offsets = message.get('offsets', c['offsets'])
                        on_map_change = message.get('on_map_change', 'continue')
                        armed = self.modulation_params['armed']
                        bundle, state, cfg = await self.to_thread(
                            self._prepare_modulation, center, offsets, c['mod_indices'],
                            c['samples_per_point'], c['n_settle'], armed)
                        if bundle['needs_recenter'] and on_map_change != 'recenter':
                            await self.send_response(writer, {'status': 'error',
                                'message': 'update would push tones beyond bin coverage; '
                                           'call recenter_modulation() or pass on_map_change="recenter"',
                                'result': {'tones_beyond_coverage': state['tones_beyond_coverage']}})
                        else:
                            if bundle['needs_recenter']:
                                # recenter: re-arm with fresh bins/maps for the new centre
                                bundle, state, cfg = await self.to_thread(
                                    self._prepare_modulation, center, offsets, c['mod_indices'],
                                    c['samples_per_point'], c['n_settle'])
                                op = 'recenter'
                            else:
                                op = 'update'
                            rev = self._next_modulation_revision(cfg)
                            state['enabled'] = True
                            state['desired_revision'] = rev
                            state['revision_history'] = dict(self._modulation_revision_history)
                            self._pending_modulation = {
                                'op': op, 'bundle': bundle, 'state': state, 'cfg': cfg, 'revision': rev}
                            await self.send_response(writer, {'status': 'success',
                                'result': {'revision': rev, 'op': op}})
                    except Exception as e:
                        print(traceback.format_exc())
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'recenter_modulation':
                    # Deliberate brief break: reload chanmaps/mixer for the current
                    # centre and recompute VACC bin-sharing, then re-arm.
                    try:
                        if self.modulation_cfg is None:
                            raise ValueError('modulation not armed; call enable_modulation first')
                        c = self.modulation_cfg
                        bundle, state, cfg = await self.to_thread(
                            self._prepare_modulation, c['center'], c['offsets'],
                            c['mod_indices'], c['samples_per_point'], c['n_settle'])
                        rev = self._next_modulation_revision(cfg)
                        state['enabled'] = True
                        state['desired_revision'] = rev
                        state['revision_history'] = dict(self._modulation_revision_history)
                        self._pending_modulation = {
                            'op': 'recenter', 'bundle': bundle, 'state': state, 'cfg': cfg, 'revision': rev}
                        await self.send_response(writer, {'status': 'success', 'result': {'revision': rev}})
                    except Exception as e:
                        print(traceback.format_exc())
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'disable_modulation':
                    # Pause modulation; tones rest at their centres. Bundle stays
                    # resident so enable_modulation() with no args re-arms quickly.
                    self.e_modulation_enabled.clear()
                    self._pending_modulation = {'op': 'disable'}
                    if not self.e_stream_enabled.is_set():
                        # No frame producer running: apply the rest-at-centre now.
                        self._apply_pending_modulation_command()
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
                    result,details = firmware_lib.check_input_saturation(self.r,self.r_fast,iterations=iterations)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})

                elif request == 'check_output_saturation':
                    iterations = message.get('iterations')
                    result,details = firmware_lib.check_output_saturation(self.r_fast,iterations=iterations)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})

                elif request == 'check_dsp_overflow':
                    duration_s = message.get('duration_s')
                    result,details = firmware_lib.check_dsp_overflow(self.r,duration_s=duration_s)
                    await self.send_response(writer, {'status': 'success', 'result': result, 'details': details})

                elif request == 'maximise_tx_power':
                    headroom_db = message.get('headroom_db', 2.0)
                    reference_plane = message.get('reference_plane', 'dac')
                    power_limit_dbm = message.get('power_limit_dbm', None)
                    compression_headroom_db = message.get('compression_headroom_db', None)
                    rx_policy = message.get('rx_policy', 'protect')
                    digital_only = message.get('digital_only', False)
                    rf_only = message.get('rf_only', False)
                    forced_controls = {
                        k: message[k] for k in firmware_lib.POWER_FORCE_CONTROL_KEYS
                        if k in message
                    }
                    amps,psb_fft_shift,psb_scale,dsp,dac = firmware_lib.maximise_tx_power(
                        self.r, self.r_fast, self.config, headroom_db=headroom_db,
                        reference_plane=reference_plane, rf_peripherals=self.rf_peripherals,
                        power_limit_dbm=power_limit_dbm,
                        compression_headroom_db=compression_headroom_db,
                        rx_policy=rx_policy, digital_only=digital_only,
                        rf_only=rf_only, **forced_controls)
                    result = {'amps': amps.tolist(), 'psb_fft_shift': psb_fft_shift, 'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    if isinstance(dac, dict) and 'tx_compression' in dac:
                        result['tx_compression'] = dac['tx_compression']
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'maximise_rx_power':
                    kwargs = {'rf_peripherals': self.rf_peripherals}
                    if 'headroom_db' in message:
                        kwargs['headroom_db'] = message['headroom_db']
                    if 'digital_only' in message:
                        kwargs['digital_only'] = message['digital_only']
                    if 'rf_only' in message:
                        kwargs['rf_only'] = message['rf_only']
                    for key in firmware_lib.POWER_FORCE_CONTROL_KEYS:
                        if key in message:
                            kwargs[key] = message[key]
                    dsa, pfb_fft_shift, dsp, adc, rx_atten = firmware_lib.maximise_rx_power(self.r, self.r_fast, self.config, **kwargs)
                    # Get RX amp bypass state if available
                    has_bypass_amps = hasattr(self.rf_peripherals, 'get_rx_amp_bypass')
                    rx_amp_bypass = self.rf_peripherals.get_rx_amp_bypass() if has_bypass_amps else None
                    result = {
                        'dsa': dsa,
                        'pfb_fft_shift': pfb_fft_shift,
                        'dsp_ovf': dsp,
                        'adc_levels': adc,
                        'rx_attenuation_db': rx_atten,
                        'rx_amp_bypass': rx_amp_bypass
                    }
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'optimise_tx_snr':
                    kwargs = {'rf_peripherals': self.rf_peripherals}
                    if 'reference_plane' in message:
                        kwargs['reference_plane'] = message['reference_plane']
                    if 'headroom_db' in message:
                        kwargs['headroom_db'] = message['headroom_db']
                    if 'digital_only' in message:
                        kwargs['digital_only'] = message['digital_only']
                    if 'rf_only' in message:
                        kwargs['rf_only'] = message['rf_only']
                    for key in firmware_lib.POWER_FORCE_CONTROL_KEYS:
                        if key in message:
                            kwargs[key] = message[key]
                    amps,psb_fft_shift,psb_scale,dsp,dac = firmware_lib.optimise_tx_snr(self.r,self.r_fast,self.config, **kwargs)
                    result = {'amps': amps.tolist(), 'psb_fft_shift': psb_fft_shift, 'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'optimise_rx_snr':
                    kwargs = {'rf_peripherals': self.rf_peripherals}
                    if 'headroom_db' in message:
                        kwargs['headroom_db'] = message['headroom_db']
                    if 'digital_only' in message:
                        kwargs['digital_only'] = message['digital_only']
                    if 'rf_only' in message:
                        kwargs['rf_only'] = message['rf_only']
                    for key in firmware_lib.POWER_FORCE_CONTROL_KEYS:
                        if key in message:
                            kwargs[key] = message[key]
                    pfb_fft_shift,dsp,adc = firmware_lib.optimise_rx_snr(self.r,self.r_fast,self.config, **kwargs)
                    result = {'pfb_fft_shift': pfb_fft_shift, 'dsp_ovf': dsp, 'adc_levels': adc}
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'fix_dac_saturation':
                    psb_scale, dsp, dac = firmware_lib.fix_dac_saturation(self.r,self.r_fast,self.config)
                    result = {'psbscale': psb_scale, 'dsp_ovf': dsp, 'dac_levels': dac}
                    await self.send_response(writer, {'status': 'success', 'result': result})
                
                elif request == 'fix_adc_saturation':
                    result = firmware_lib.fix_adc_saturation(self.r,self.r_fast,self.config, rf_peripherals=self.rf_peripherals)
                    await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'fix_dsp_overflow':
                    duration_s = message.get('duration_s', 0.5)
                    max_iterations = message.get('max_iterations', 10)
                    changed, details = firmware_lib.fix_dsp_overflow(self.r, duration_s=duration_s, max_iterations=max_iterations)
                    result = {'changed': changed, 'details': details}
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
                    if not result.get('success', True):
                        await self.send_response(writer, {
                            'status': 'error',
                            'message': result.get('message', 'LNA bias set failed'),
                            'result': result,
                        })
                    else:
                        await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'soft_off_lna_bias':
                    channel = message.get('channel')
                    if channel is not None:
                        channel = int(channel)
                    result = self.lna_controller.soft_off_lna_bias(channel)
                    if not result.get('success', True):
                        await self.send_response(writer, {
                            'status': 'error',
                            'message': result.get('message', 'LNA soft off failed'),
                            'result': result,
                        })
                    else:
                        await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'set_lna_bias_voltage_all':
                    voltage_v = float(message.get('voltage_v'))
                    method = message.get('method', 'remote')
                    blind = message.get('blind', False)
                    result = self.lna_controller.set_lna_bias_voltage_all(
                        voltage_v, method, blind,
                    )
                    failed = [r for r in result.values() if not r.get('success', True)]
                    if failed:
                        msgs = '; '.join(
                            f"chn {r['channel']}: {r.get('message', 'failed')}"
                            for r in failed
                        )
                        await self.send_response(writer, {
                            'status': 'error',
                            'message': f'{len(failed)}/{len(result)} channels failed: {msgs}',
                            'result': result,
                        })
                    else:
                        await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'soft_off_lna_bias_all':
                    result = self.lna_controller.soft_off_lna_bias_all()
                    failed = [r for r in result.values() if not r.get('success', True)]
                    if failed:
                        msgs = '; '.join(
                            f"chn {r['channel']}: {r.get('message', 'failed')}"
                            for r in failed
                        )
                        await self.send_response(writer, {
                            'status': 'error',
                            'message': f'{len(failed)}/{len(result)} channels failed: {msgs}',
                            'result': result,
                        })
                    else:
                        await self.send_response(writer, {'status': 'success', 'result': result})

                elif request == 'get_samples':
                    num_samples = message.get('num_samples')
                    burst = message.get('burst', False)
                    task = asyncio.create_task(self.get_samples(writer, num_samples, burst=burst))
                    self.tasks.append(task)

                elif request == 'get_accumulator_snapshots':
                    tone_index = message.get('tone_index')
                    num_snapshots = message.get('num_snapshots')
                    if message.get('fast', False):
                        task = asyncio.create_task(self.batch_accumulator_snapshots(writer, [tone_index], num_snapshots))
                    else:
                        task = asyncio.create_task(self.get_accumulator_snapshots(writer, tone_index, num_snapshots))
                    self.tasks.append(task)

                elif request == 'batch_accumulator_snapshots':
                    tone_indices = message.get('tone_indices')
                    num_snapshots = message.get('num_snapshots')
                    task = asyncio.create_task(self.batch_accumulator_snapshots(writer, tone_indices, num_snapshots))
                    self.tasks.append(task)

                elif request == 'get_adc_snapshot':
                    try:
                        snapshot = firmware_lib.get_adc_snapshot_fast(self.r_fast)
                        data = base64.b64encode(snapshot.tobytes()).decode()
                        result = {'snapshot': data, 'length': len(snapshot)}
                        await self.send_response(writer, {'status': 'success', 'result': result})
                    except Exception as e:
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'get_dac_snapshot':
                    try:
                        dac0, dac1 = firmware_lib.get_dac_snapshot_fast(self.r_fast)
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
                        refresh_adc_cal = message.get('refresh_adc_cal', True)
                        adc_cal_settle_time = message.get('adc_cal_settle_time', 2.0)
                        self.latest_sweep_data_valid = False
                        self.sweep_progress = 0.0
                        self.sweep_state = {'state': 'running', 'message': 'Sweep in progress'}
                        #print('asyncio create task, sweep task')
                        self.sweep_task = asyncio.create_task(
                            self.sweep(centers, spans, points, samples_per_point, direction, refresh_adc_cal=refresh_adc_cal, adc_cal_settle_time=adc_cal_settle_time)
                        )

                        #print('await send response')
                        await self.send_response(writer, {'status': 'success', 'message': 'Sweep in progress'})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'Sweep already in progress'})
                
                elif request == 'get_sweep_progress':
                    progress = self.sweep_progress
                    if self.sweep_task is not None and not self.sweep_task.done():
                        progress = min(progress, 0.999999)
                    response = {'status': 'success', 'progress': progress}
                    response.update(self.sweep_state)
                    await self.send_response(writer, response)
                    
                elif request == 'get_sweep_data':
                    if self.latest_sweep_data_valid:
                        sweep_f = self.latest_sweep_results['sweep_frequencies'].astype('f8')
                        sweep_z = self.latest_sweep_results['sweep_responses'].astype('complex128')
                        sweep_e = self.latest_sweep_results['sweep_sems'].astype('complex128')
                        sweep_tt = self.latest_sweep_results['telescope_time'].astype('u8')
                        sweep = {}
                        sweep['f'] = base64.b64encode(sweep_f.tobytes()).decode()
                        sweep['z'] = base64.b64encode(sweep_z.tobytes()).decode()
                        sweep['e'] = base64.b64encode(sweep_e.tobytes()).decode()
                        sweep['tt'] = base64.b64encode(sweep_tt.tobytes()).decode()

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
                              'info': self.get_info('all')}
                        

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
                        
                        data += '# info: '+str(self.get_info('all')) +'\n'
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
                        refresh_adc_cal = message.get('refresh_adc_cal', True)
                        adc_cal_settle_time = message.get('adc_cal_settle_time', 2.0)
                        self.latest_sweep_data_valid = False
                        self.sweep_progress = 0.0
                        self.sweep_state = {'state': 'running', 'message': 'Retune in progress'}
                        self.sweep_task = asyncio.create_task(
                            self.retune(centers, spans, points, samples_per_point, direction, method, freq_offsets, refresh_adc_cal=refresh_adc_cal, adc_cal_settle_time=adc_cal_settle_time)
                        )
                        await self.send_response(writer, {'status': 'success', 'message': 'Retune in progress'})
                    else:
                        await self.send_response(writer, {'status': 'error', 'message': 'Sweep or retune already in progress'})

                elif request == 'refresh_adc_cal':
                    # Inline rather than firmware_lib.refresh_adc_cal() to use
                    # async sleep and keep the stream flag in sync.
                    try:
                        adc_cal_settle_time = message.get('adc_cal_settle_time', 2.0)
                        firmware_lib.set_cal_freeze(self.r, self.config, False)
                        self.stream_flags[FLAG_CAL_FREEZE].clear()
                        await asyncio.sleep(adc_cal_settle_time)
                        firmware_lib.set_cal_freeze(self.r, self.config, True)
                        self.stream_flags[FLAG_CAL_FREEZE].set()
                        await self.send_response(writer, {'status': 'success'})
                    except Exception as e:
                        await self.send_response(writer, {'status': 'error', 'message': str(e)})

                elif request == 'cancel':
                    if self.sweep_task:
                        self.sweep_task.cancel()
                        self.sweep_task = None
                        self.sweep_progress=0.0
                        self.sweep_state = {'state': 'cancelled', 'message': 'Sweep cancelled'}
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
        Enable TCP keepalive on the open socket ``sock``.
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
    
    def _tone_plan(self):
        return firmware_lib.get_configured_tone_plan(self.config)

    def _tone_defaults(self):
        return self.config.setdefault('firmware', {}).setdefault('defaults', {})

    @staticmethod
    def _normalise_interactive_values(values, n_values, name,
                                      default_values=None,
                                      default_value=None):
        if n_values == 0:
            return np.array([], dtype=float)
        if values is None:
            if default_values is not None and len(default_values) == n_values:
                return np.asarray(default_values, dtype=float)
            if default_value is None:
                return None
            return np.full(n_values, default_value, dtype=float)
        values = np.atleast_1d(values).astype(float)
        if len(values) == n_values:
            return values
        if len(values) == 1 and n_values > 1:
            return np.full(n_values, float(values[0]), dtype=float)
        raise ValueError(
            f'Number of {name} values ({len(values)}) must match number of '
            f'blind tones ({n_values})')

    def _live_tone_state(self):
        freqs = np.asarray(
            firmware_lib.get_tone_frequencies(self.r, self.config),
            dtype=float)
        amps = np.asarray(
            firmware_lib.get_tone_amplitudes(self.r, self.config),
            dtype=float)
        phases = np.asarray(
            firmware_lib.get_tone_phases(self.r, self.config),
            dtype=float)
        metadata = firmware_lib.get_configured_tone_metadata(
            self.config, active_count=len(freqs))
        return freqs, amps, phases, metadata

    def _tone_config_snapshot(self):
        defaults = self._tone_defaults()
        keys = (
            'frequencies', 'amplitudes', 'phases',
            'blind_frequencies', 'blind_amplitudes', 'blind_phases',
            'blind_spans',
        )
        return {key: copy.deepcopy(defaults.get(key, [])) for key in keys}

    def _set_tone_defaults_from_split(self, regular_freqs, regular_amps,
                                      regular_phases, blind_freqs,
                                      blind_amps=None, blind_phases=None,
                                      blind_spans=None):
        defaults = self._tone_defaults()
        defaults['frequencies'] = np.asarray(regular_freqs, dtype=float).tolist()
        defaults['amplitudes'] = np.asarray(regular_amps, dtype=float).tolist()
        defaults['phases'] = np.asarray(regular_phases, dtype=float).tolist()
        defaults['blind_frequencies'] = np.asarray(blind_freqs, dtype=float).tolist()
        defaults['blind_amplitudes'] = (
            [] if blind_amps is None else np.asarray(blind_amps, dtype=float).tolist()
        )
        defaults['blind_phases'] = (
            [] if blind_phases is None else np.asarray(blind_phases, dtype=float).tolist()
        )
        defaults['blind_spans'] = (
            [] if blind_spans is None else np.asarray(blind_spans, dtype=float).tolist()
        )

    def get_blind_tones(self, reference_plane='detector'):
        """Return the current blind-tone state and indices.

        ``reference_plane`` ('dac', 'rf_output', 'adc_input', or 'detector')
        sets the plane any reported blind-tone powers are referred to.
        """
        freqs, amps, phases, metadata = self._live_tone_state()
        blind_indices = metadata['blind_indices']
        powers = None
        if blind_indices:
            try:
                powers_all = firmware_lib.get_tone_powers(
                    self.r, self.config, reference_plane=reference_plane,
                    rf_peripherals=self.rf_peripherals)
                powers = np.asarray(powers_all, dtype=float)[blind_indices].tolist()
            except Exception as e:
                print(f'Warning: could not read blind tone powers: {e}')

        plan = self._tone_plan()
        spans = plan['blind_spans']
        if len(spans) != len(blind_indices):
            spans = np.array([], dtype=float)
        blind_freqs = freqs[blind_indices].tolist() if blind_indices else []
        blind_phases = phases[blind_indices].tolist() if blind_indices else []

        return {
            'indices': blind_indices,
            'frequencies': blind_freqs,
            'frequencies_hz': blind_freqs,
            'amplitudes': amps[blind_indices].tolist() if blind_indices else [],
            'phases': blind_phases,
            'phases_rad': blind_phases,
            'spans': spans.tolist(),
            'powers_dbm': powers,
            'powers_reference_plane': reference_plane,
            'regular_indices': metadata['regular_indices'],
            'tone_types': metadata['tone_types'],
            'all_frequencies': freqs.tolist(),
            'metadata': metadata,
            'config_defaults': self._tone_config_snapshot(),
        }

    def set_blind_tones(self, frequencies, amplitudes=None, phases=None,
                        spans=None, powers_dbm=None,
                        reference_plane='detector',
                        optimise_dynamic_range=False,
                        rx_policy='protect'):
        """Append blind tones to the active comb and apply to firmware.

        ``frequencies`` are the blind-tone frequencies (Hz); ``amplitudes``,
        ``phases``, ``spans`` and ``powers_dbm`` are optional per-tone arrays
        (server defaults used when omitted).  ``reference_plane`` is the plane
        ``powers_dbm`` is given at, ``optimise_dynamic_range`` re-optimises DAC
        utilisation on apply, and ``rx_policy`` is the RX-path policy (as in the
        client's ``set_tone_powers``).  Empty ``frequencies`` removes blind tones.
        """
        blind_freqs = np.atleast_1d(frequencies).astype(float)
        if len(blind_freqs) == 0:
            return self.remove_blind_tones()

        freqs, amps, current_phases, metadata = self._live_tone_state()
        regular_indices = metadata['regular_indices']
        regular_freqs = freqs[regular_indices]
        regular_amps = amps[regular_indices]
        regular_phases = current_phases[regular_indices]

        old_plan = self._tone_plan()
        old_blind_amps = old_plan['amplitudes']
        old_blind_phases = old_plan['phases']
        if (old_blind_amps is not None
                and len(old_blind_amps) == old_plan['num_tones']
                and old_plan['num_blind_tones'] == len(blind_freqs)):
            old_blind_amps = np.asarray(old_blind_amps)[old_plan['blind_indices']]
        else:
            old_blind_amps = None
        if (old_blind_phases is not None
                and len(old_blind_phases) == old_plan['num_tones']
                and old_plan['num_blind_tones'] == len(blind_freqs)):
            old_blind_phases = np.asarray(old_blind_phases)[old_plan['blind_indices']]
        else:
            old_blind_phases = None

        blind_amps = self._normalise_interactive_values(
            amplitudes, len(blind_freqs), 'amplitudes',
            default_values=old_blind_amps, default_value=1.0)
        blind_phases = self._normalise_interactive_values(
            phases, len(blind_freqs), 'phases',
            default_values=old_blind_phases, default_value=0.0)
        blind_spans = self._normalise_interactive_values(
            spans, len(blind_freqs), 'spans', default_value=None)
        if blind_spans is None:
            blind_spans = np.array([], dtype=float)

        current_regular_powers = None
        if powers_dbm is not None:
            current_powers = firmware_lib.get_tone_powers(
                self.r, self.config, reference_plane=reference_plane,
                rf_peripherals=self.rf_peripherals)
            current_powers = np.asarray(current_powers, dtype=float)
            current_regular_powers = current_powers[regular_indices]

        self._set_tone_defaults_from_split(
            regular_freqs, regular_amps, regular_phases,
            blind_freqs, blind_amps=blind_amps, blind_phases=blind_phases,
            blind_spans=blind_spans)

        combined_freqs = np.concatenate([regular_freqs, blind_freqs])
        combined_amps = np.concatenate([regular_amps, blind_amps])
        combined_phases = np.concatenate([regular_phases, blind_phases])
        firmware_lib.set_tone_frequencies_fast(
            self.r, self.r_fast, self.config, combined_freqs,
            tone_amplitudes=combined_amps, tone_phases=combined_phases)
        self.update_active_tone_indices()

        if powers_dbm is not None:
            blind_powers = self._normalise_interactive_values(
                powers_dbm, len(blind_freqs), 'powers_dbm')
            target_powers = np.concatenate([
                current_regular_powers, blind_powers])
            firmware_lib.set_tone_powers(
                self.r, self.r_fast, self.config, target_powers,
                reference_plane=reference_plane,
                optimise_dynamic_range=optimise_dynamic_range,
                rf_peripherals=self.rf_peripherals,
                rx_policy=rx_policy)
            # Capture the amplitude result after calibrated power setting.
            final_amps = firmware_lib.get_tone_amplitudes(self.r, self.config)
            defaults = self._tone_defaults()
            defaults['amplitudes'] = np.asarray(
                final_amps[:len(regular_freqs)], dtype=float).tolist()
            defaults['blind_amplitudes'] = np.asarray(
                final_amps[len(regular_freqs):], dtype=float).tolist()

        return self.get_blind_tones(reference_plane=reference_plane)

    def remove_blind_tones(self):
        freqs, amps, phases, metadata = self._live_tone_state()
        regular_indices = metadata['regular_indices']
        regular_freqs = freqs[regular_indices]
        regular_amps = amps[regular_indices]
        regular_phases = phases[regular_indices]
        self._set_tone_defaults_from_split(
            regular_freqs, regular_amps, regular_phases,
            [], blind_amps=[], blind_phases=[], blind_spans=[])
        if len(regular_freqs) > 0:
            firmware_lib.set_tone_frequencies_fast(
                self.r, self.r_fast, self.config, regular_freqs,
                tone_amplitudes=regular_amps,
                tone_phases=regular_phases)
        else:
            firmware_lib.set_tone_frequencies_fast(
                self.r, self.r_fast, self.config, [])
        self.update_active_tone_indices()
        return self.get_blind_tones()

    def _expand_frequencies_for_blind(self, frequencies):
        """Append configured blind centers when a user supplies regular tones."""
        freqs = np.atleast_1d(frequencies).astype(float)
        plan = self._tone_plan()
        n_regular = plan['num_regular_tones']
        n_blind = plan['num_blind_tones']
        if n_blind == 0:
            return freqs.tolist()
        if len(freqs) == n_regular:
            return np.concatenate([freqs, plan['blind_frequencies']]).tolist()
        if len(freqs) == n_regular + n_blind:
            freqs = freqs.copy()
            freqs[plan['blind_indices']] = plan['blind_frequencies']
            return freqs.tolist()
        return freqs.tolist()

    def _expand_values_for_blind(self, values, getter, getter_kwargs=None,
                                 plan_values_key=None, default_value=None):
        """Append blind per-tone values when only regular values are supplied."""
        vals = np.atleast_1d(values).astype(float)
        plan = self._tone_plan()
        n_regular = plan['num_regular_tones']
        n_blind = plan['num_blind_tones']
        total = n_regular + n_blind
        if n_blind == 0 or len(vals) != n_regular or len(vals) == total:
            return vals.tolist()

        getter_kwargs = getter_kwargs or {}
        blind_values = None
        try:
            current = np.atleast_1d(getter(self.r, self.config, **getter_kwargs))
            if len(current) == total:
                blind_values = current[plan['blind_indices']]
        except Exception as e:
            print(f'Warning: could not read current blind tone values: {e}')

        if blind_values is None and plan_values_key is not None:
            plan_values = plan.get(plan_values_key)
            if plan_values is not None and len(plan_values) == total:
                blind_values = np.asarray(plan_values)[plan['blind_indices']]

        if blind_values is None:
            if default_value is None:
                return vals.tolist()
            blind_values = np.full(n_blind, default_value, dtype=float)

        return np.concatenate([vals, blind_values]).tolist()

    def _expand_sweep_request_for_blind(self, centers, spans):
        """Append configured blind sweep centers/spans to regular tone sweeps."""
        centers = np.atleast_1d(centers).astype(float)
        spans = np.atleast_1d(spans).astype(float)
        plan = self._tone_plan()
        n_regular = plan['num_regular_tones']
        n_blind = plan['num_blind_tones']
        total = n_regular + n_blind

        if n_blind == 0:
            if len(spans) == 1:
                spans = np.full(len(centers), spans[0])
            return centers, spans

        if len(centers) not in (n_regular, total):
            if len(spans) == 1:
                spans = np.full(len(centers), spans[0])
            return centers, spans

        if len(spans) == 1:
            spans = np.full(len(centers), spans[0])

        if len(centers) == n_regular:
            blind_spans = plan['blind_spans']
            if len(blind_spans) != n_blind:
                fallback_span = float(np.median(spans)) if len(spans) else 0.0
                blind_spans = np.full(n_blind, fallback_span, dtype=float)
            centers = np.concatenate([centers, plan['blind_frequencies']])
            spans = np.concatenate([spans, blind_spans])
        else:
            centers = centers.copy()
            centers[plan['blind_indices']] = plan['blind_frequencies']
            if len(spans) != total:
                spans = np.full(total, float(np.median(spans)), dtype=float)

        return centers, spans

    def update_active_tone_indices(self):
        """
        Refresh active_tone_indices from the firmware.

        Called after tone frequencies are set so that prepare_frame sends
        only active tones in user order.
        """
        try:
            details = firmware_lib.get_tone_frequencies(
                self.r, self.config, detailed_output=True)[1]
            self.active_tone_indices = np.asarray(
                details['rx']['tone_indices'])
        except Exception as e:
            print(f"Warning: could not update active tone indices: {e}")
            self.active_tone_indices = None

    def prepare_frame(self, fast_read_params, mod_point=0, settling=False, revision=0):
        """
        Prepare a frame for sending to a client.

        Reads accumulated data at the active tone indices (user order)
        so that clients receive only active tones without needing to
        reindex.

        ``fast_read_params`` is the precomputed fast-read parameter bundle
        (firmware addresses/sizes) used to read the accumulator efficiently.

        Fast frequency modulation packs the otherwise-unused ``flag5`` word
        (``frame[-5]``) with the per-sample modulation tag when ``mod_point > 0``:
        bits 0..15 = active modulation point (1..N; 0 = modulation off),
        bit 16 = settling/transient marker, bits 17..31 = config revision.
        Decode it as **unsigned** on the client. When not modulating the word
        keeps its legacy ``stream_flags[5]`` boolean meaning.
        """
        num_headers = 10

        cnt,data,err,tt = firmware_lib.read_accumulated_data_fast(
            fast_read_params, tone_indices=self.active_tone_indices)

        tt_msb = np.uint32((tt >> 32) & 0xFFFFFFFF).view(np.int32)
        tt_lsb = np.uint32(tt & 0xFFFFFFFF).view(np.int32)

        frame = np.zeros(len(data)+num_headers,dtype='<i4')
        frame[:len(data)] = data
        frame[-1] = err
        frame[-2] = cnt
        frame[-3] = tt_lsb
        frame[-4] = tt_msb
        if mod_point > 0:
            tag = (int(mod_point) & 0xFFFF) | (int(bool(settling)) << 16) | ((int(revision) & 0x7FFF) << 17)
            frame[-5] = np.uint32(tag).view(np.int32)
            # keep FLAG_SET_FREQS high during the transient window for older consumers
            set_freqs = int(self.stream_flags[FLAG_SET_FREQS].is_set() or settling)
        else:
            frame[-5] = int(self.stream_flags[5].is_set())
            set_freqs = int(self.stream_flags[FLAG_SET_FREQS].is_set())
        frame[-6] = int(self.stream_flags[FLAG_CAL_FREEZE].is_set())
        frame[-7] = int(self.stream_flags[FLAG_SET_PHASES].is_set())
        frame[-8] = int(self.stream_flags[FLAG_SET_AMPS].is_set())
        frame[-9] = set_freqs
        frame[-10] = int(self.stream_flags[FLAG_SERVER_REQUEST].is_set())

        data_bytes = frame.tobytes()

        data_len = struct.pack('>I', len(data_bytes))

        payload = data_len+data_bytes

        return payload,cnt,err
    

    async def get_samples(self, writer, num_samples, burst=False):
        """
        Get a fixed number of samples over the request channel.

        Plain mode (modulation disarmed) is unchanged: a short warm-up to avoid
        initial latency, then ``num_samples`` untagged frames. Modulated mode
        (``e_modulation_enabled`` set) drives the **same shared stepping engine**
        as :meth:`stream_data`, so the returned frames carry the per-sample
        modulation tag exactly as a continuous stream would.

        Parameters
        ----------
        writer : asyncio.StreamWriter
            Request-channel writer the framed samples are sent to.
        num_samples : int
            Number of accumulator samples to return.
        burst : bool, optional
            Single-burst transfer. Incompatible with modulation (rejected).
        """
        modulating = self.e_modulation_enabled.is_set() and self.modulation_params is not None
        try:
            if modulating:
                # Single hardware-write owner: refuse to step while the continuous
                # streamer is running, and refuse burst (it cannot interleave steps).
                if burst:
                    raise ValueError('burst=True is not supported while modulation is armed')
                if self.e_stream_enabled.is_set():
                    raise ValueError('continuous streaming is active; disable_stream before a '
                                     'modulated get_samples capture')
                self._modulation_owner = 'samples'

            fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
            rate = firmware_lib.get_sample_rate(self.r_fast)

            if modulating:
                # This coroutine owns the hardware now: apply any queued command,
                # then (re-)arm so buffers/maps are in a known state, poised at point 1.
                self._apply_pending_modulation_command()
                sched = self.modulation_sched
                if sched is None:
                    raise ValueError('modulation armed but no scheduler available')
                sched.arm()

                # Warm-up: prime the comb/pipeline by stepping whole cycles and
                # discarding their frames, so the first *returned* frame is settled
                # and aligned to point 1 (index 0). Rounded up to whole cycles.
                if rate > 100:
                    cycle = max(1, sched.N * sched.samples_per_point)
                    warm_cycles = max(1, (50 + cycle - 1) // cycle)
                    for _ in range(warm_cycles):
                        for _v in range(sched.N):
                            for _s in range(sched.samples_per_point):
                                self.prepare_frame(fast_read_params, mod_point=sched.current_point() + 1)
                            sched.advance()
                    # whole cycles -> live point back at index 0 (point 1)

                # Capture: step through points, tagging every returned frame.
                emitted = 0
                while emitted < num_samples:
                    p = sched.current_point()
                    for s in range(sched.samples_per_point):
                        if emitted >= num_samples:
                            break
                        settling = s < sched.n_settle
                        payload, cnt, err = self.prepare_frame(
                            fast_read_params, mod_point=p + 1, settling=settling,
                            revision=sched.revision)
                        writer.write(payload)
                        emitted += 1
                    sched.advance()
                    await asyncio.sleep(0)
                await writer.drain()
            else:
                # Plain path (unchanged): warm up a bit to avoid initial delays.
                if (rate > 100) and not burst:
                    for j in range(50):
                        payload, cnt, err = self.prepare_frame(fast_read_params)
                        continue
                for _ in range(num_samples):
                    payload, cnt, err = self.prepare_frame(fast_read_params)
                    writer.write(payload)
                    await writer.drain()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error getting samples: {e}")
            print(traceback.format_exc())
        finally:
            if self._modulation_owner == 'samples':
                self._modulation_owner = None
            self.tasks.remove(asyncio.current_task())
            writer.close()
            await writer.wait_closed()

    async def get_accumulator_snapshots(self, writer, tone_index, num_snapshots):
        """
        Acquire num_snapshots pre-accumulation snapshots for a single tone
        using the standard CASPER snapshot API and stream them to the client.

        This is slower than the manual devmem path but keeps the snapshot
        control/read sequence inside the firmware library implementation.
        """
        try:
            details = firmware_lib.get_tone_frequencies(
                self.r, self.config, detailed_output=True)[1]
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

    async def batch_accumulator_snapshots(self, writer, tone_indices, num_snapshots):
        """
        Acquire num_snapshots pre-accumulation snapshots for multiple tones
        and stream to client.

        Resolves tone-to-firmware-channel mapping once, then iterates over
        tones and snapshots using the fast devmem path.

        Wire format: for each tone, num_snapshots frames of (4-byte big-endian
        length prefix + raw complex128 data).
        """
        try:
            details = firmware_lib.get_tone_frequencies(self.r, self.config, detailed_output=True)[1]
            firmware_indices = details['rx']['tone_indices']

            for tone_index in tone_indices:
                if tone_index >= len(firmware_indices):
                    raise ValueError(f'Tone index {tone_index} out of range '
                                     f'(only {len(firmware_indices)} tones active)')
                fw_chan = firmware_indices[tone_index]
                for _ in range(num_snapshots):
                    data = np.asarray(firmware_lib._read_accumulator_snapshot_fast(self.r_fast, fw_chan), dtype=np.complex128)
                    data_bytes = data.tobytes()
                    data_len = struct.pack('>I', len(data_bytes))
                    writer.write(data_len + data_bytes)
                    await writer.drain()
        except asyncio.CancelledError:
            pass
        except Exception as e:
            print(f"Error getting batch accumulator snapshots: {e}")
            print(traceback.format_exc())
        finally:
            self.tasks.remove(asyncio.current_task())
            writer.close()
            await writer.wait_closed()

    def _prepare_modulation(self, center, offsets, mod_indices,
                            samples_per_point, n_settle, armed=None):
        """
        Build a modulation bundle + observability state from a centre comb and
        per-point probe offsets. Pure computation (hardware *reads* only, no
        writes) so it is safe to run in a thread executor off the streaming hot
        path.

        Parameters
        ----------
        center : array-like or None
            Per-tone centre RF frequencies (Hz), user-facing order. ``None``
            uses the current live comb.
        offsets : array-like
            Probe offsets (Hz): shape ``(n_points,)`` (broadcast across the
            modulated tones) or ``(n_points, len(mod_indices))`` (per tone).
        mod_indices : array-like or None
            User-facing indices of tones to modulate. ``None`` = all regular
            (resonator) tones. Including a blind tone raises ``ValueError``.
        samples_per_point : int
            Dwell: accumulations emitted per point per cycle.
        n_settle : int
            Number of leading samples per point flagged as settling.
        armed : dict or None
            Existing armed bins/maps to reuse for a live update (ride the same
            fixed maps). ``None`` arms fresh bins from ``center``.

        Returns
        -------
        (bundle, state, cfg) : tuple of dict
            ``bundle`` from :func:`firmware_lib.prepare_modulation_settings_fast`;
            ``state`` the ``get_info('tone_modulation')`` payload (static parts);
            ``cfg`` the resolved configuration (center/offsets/mod_indices/dwell).
        """
        freqs, amps, phases, metadata = self._live_tone_state()
        n_tones = len(freqs)
        regular = list(metadata.get('regular_indices', list(range(n_tones))))
        blind = set(int(b) for b in metadata.get('blind_indices', []))

        center = np.asarray(freqs if center is None else center, dtype=float)
        if len(center) != n_tones:
            raise ValueError(f'center length ({len(center)}) must match active tone count ({n_tones})')

        # mod_indices default to the resonator tones; blind tones must never modulate.
        if mod_indices is None:
            mod_indices = [int(i) for i in regular]
        else:
            mod_indices = [int(i) for i in np.atleast_1d(mod_indices)]
            bad = sorted(i for i in mod_indices if i in blind)
            if bad:
                raise ValueError(
                    f'cannot modulate blind tones (indices {bad}); blind tones must stay fixed')

        # Expand offsets to full per-tone columns (0 for tones we do not modulate).
        offsets = np.atleast_2d(np.asarray(offsets, dtype=float))
        num_points = offsets.shape[0]
        point_offsets = np.zeros((num_points, n_tones), dtype=float)
        if offsets.shape[1] == 1:
            point_offsets[:, mod_indices] = offsets            # one offset per point, all modulated tones
        elif offsets.shape[1] == len(mod_indices):
            point_offsets[:, mod_indices] = offsets            # per-tone offsets
        else:
            raise ValueError(
                f'offsets has {offsets.shape[1]} columns; expected 1 or '
                f'len(mod_indices)={len(mod_indices)}')

        bundle = firmware_lib.prepare_modulation_settings_fast(
            self.r, self.config, center, point_offsets,
            tone_amplitudes=amps, tone_phases=phases, armed=armed)

        cfg = {'center': center, 'offsets': offsets, 'mod_indices': mod_indices,
               'samples_per_point': int(samples_per_point), 'n_settle': int(n_settle)}
        state = self._build_modulation_state(bundle, cfg)
        return bundle, state, cfg

    def _build_modulation_state(self, bundle, cfg):
        """
        Assemble the ``get_info('tone_modulation')`` payload from a prepared
        ``bundle`` and resolved ``cfg``. Per-tone entries are in user-facing
        order (matching the stream's I/Q columns); ``firmware_index`` is an
        annotation only. ``enabled`` / revision fields are filled by the frame
        producer when the command is actually applied.

        Parameters
        ----------
        bundle : dict
            Output of :func:`firmware_lib.prepare_modulation_settings_fast`.
        cfg : dict
            Resolved config (center/offsets/mod_indices/samples_per_point/n_settle).
        """
        mod_indices = cfg['mod_indices']
        occupancy = bundle['occupancy']
        drift = bundle['drift_bins']
        center = np.asarray(cfg['center'], dtype=float)
        n_tones = int(bundle['num_tones'])
        num_points = int(bundle['num_points'])

        # Annotate the internal firmware/VACC index for reference (never used as API).
        try:
            firmware_idx = firmware_lib.get_tone_frequencies(
                self.r, self.config, detailed_output=True)[1]['rx']['tone_indices']
        except Exception:
            firmware_idx = bundle['tone_indices']

        # Full per-tone offset matrix for reporting.
        off = np.atleast_2d(np.asarray(cfg['offsets'], dtype=float))
        point_offsets = np.zeros((num_points, n_tones), dtype=float)
        point_offsets[:, mod_indices] = off if off.shape[1] != 1 else off

        tones = []
        for i in range(n_tones):
            tones.append({
                'index': i,
                'firmware_index': int(firmware_idx[i]) if i < len(firmware_idx) else None,
                'center_hz': float(center[i]),
                'armed_fft_bin': int(bundle['armed_tx_bins'][i]),
                'offsets_hz': point_offsets[:, i].tolist(),
                'drift_bins': drift[:, i].tolist(),
                'occupancy': [str(o) for o in occupancy[:, i]],
            })
        beyond = sorted({i for i in range(n_tones) if 'beyond' in set(occupancy[:, i])})

        try:
            sample_rate = float(firmware_lib.get_sample_rate(self.r_fast))
        except Exception:
            sample_rate = float('nan')
        cycle_rate = sample_rate / (num_points * cfg['samples_per_point']) if num_points else float('nan')

        return {
            'enabled': False,
            'desired_revision': 0,
            'applied_revision': 0,
            'revision_history': {},
            'num_points': num_points,
            'samples_per_point': int(cfg['samples_per_point']),
            'n_settle': int(cfg['n_settle']),
            'mod_indices': list(mod_indices),
            'sample_rate_hz': sample_rate,
            'cycle_rate_hz': cycle_rate,
            'needs_recenter': bool(bundle['needs_recenter']),
            'tones_beyond_coverage': beyond,
            'tones': tones,
        }

    def _next_modulation_revision(self, cfg):
        """
        Bump and return the modulation configuration revision, recording the
        config in the revision history (used to reconstruct which centre/offsets
        applied to frames offline).

        ``cfg`` is the resolved configuration dict being applied.
        """
        self._modulation_revision = (self._modulation_revision + 1) & 0x7FFF
        rev = self._modulation_revision
        self._modulation_revision_history[rev] = {
            'center': np.asarray(cfg['center'], dtype=float).tolist(),
            'offsets': np.asarray(cfg['offsets'], dtype=float).tolist(),
            'mod_indices': list(cfg['mod_indices']),
            'ts': time.time(),
        }
        return rev

    def _invalidate_modulation(self, reason=''):
        """
        Drop any armed/resident modulation bundle and disarm. Called when an
        ordinary tone / amplitude / phase / blind-tone change makes a staged
        bundle stale (its centre/offsets/maps no longer match the comb), so the
        user must reconfigure via ``enable_modulation``.

        ``reason`` is a short human-readable note for the server log.
        """
        if (self.modulation_params is not None or self.e_modulation_enabled.is_set()
                or self._pending_modulation is not None):
            print(f'Modulation invalidated{": " + reason if reason else ""}; re-arm with enable_modulation.')
        self.e_modulation_enabled.clear()
        self.modulation_params = None
        self.modulation_cfg = None
        self.modulation_state = None
        self.modulation_sched = None
        self._pending_modulation = None

    def _write_to_stream_clients(self, payload):
        """
        Write one framed ``payload`` to every connected stream client, dropping
        any client whose socket has failed.

        ``payload`` is the length-prefixed frame from :meth:`prepare_frame`.
        Shared by the plain and modulated branches of :meth:`stream_data` so both
        behave identically toward clients. (Drain is deferred to the caller.)
        """
        for client in list(self.stream_clients):
            try:
                client.write(payload)
            except ConnectionResetError:
                print(f"Stream client disconnected {getattr(client, 'addr', '?')} (ConnectionResetError)")
                self.stream_clients.remove(client)
            except Exception as e:
                print(f"Error streaming data to client: {e}")
                print(traceback.format_exc())
                self.stream_clients.remove(client)

    def _apply_pending_modulation_command(self):
        """
        Apply a queued modulation command at a cycle boundary. The frame producer
        (``stream_data`` or ``get_samples``) is the **sole owner** of modulation
        hardware writes, so all arming / buffer / chanmap / sync writes happen
        here rather than in the request handler — avoiding races with an in-flight
        cycle. Latest-wins: only the most recent pending command is applied.

        Recognised ``op`` values on ``self._pending_modulation``:
        ``'enable'`` / ``'recenter'`` — (re)arm with a fresh scheduler (applies
        the channel maps); ``'update'`` — swap new per-point words into the
        running scheduler in place (maps unchanged, seamless); ``'disable'`` —
        rest the tones at their centres and clear the armed flag.
        """
        cmd = self._pending_modulation
        if cmd is None:
            return
        self._pending_modulation = None
        op = cmd['op']
        try:
            if op == 'disable':
                self.e_modulation_enabled.clear()
                # Rest the tones at their centre frequencies (re-snaps to nearest bins).
                if self.modulation_cfg is not None:
                    firmware_lib.set_tone_frequencies_fast(
                        self.r, self.r_fast, self.config,
                        np.asarray(self.modulation_cfg['center'], dtype=float),
                        autosync=True)
                    self.update_active_tone_indices()
                if self.modulation_state is not None:
                    self.modulation_state['enabled'] = False
                return

            self.modulation_params = cmd['bundle']
            self.modulation_cfg = cmd['cfg']
            self.modulation_state = cmd['state']
            spp = int(cmd['cfg']['samples_per_point'])
            n_settle = int(cmd['cfg']['n_settle'])
            if op == 'update' and self.modulation_sched is not None:
                # Same channel maps: swap words in place, no re-arm, no map write.
                self.modulation_sched.install_bundle(cmd['bundle'], cmd['revision'], spp, n_settle)
            else:
                # enable / recenter: build a fresh scheduler and arm (applies maps).
                self.modulation_sched = ModulationScheduler(
                    self.r_fast, cmd['bundle'], spp, n_settle, cmd['revision'])
                self.modulation_sched.arm()
            # Record which revision actually landed (vs the desired one in state).
            self.modulation_state['applied_revision'] = cmd['revision']
        except Exception as e:
            print(f"Error applying modulation command ({op}): {e}")
            print(traceback.format_exc())

    async def stream_data(self):
        """
        Continuous streamer coroutine: while ``e_stream_enabled`` is set it sends
        accumulated frames to all connected stream clients. When
        ``e_modulation_enabled`` is also set it steps the modulation points and
        tags each frame; otherwise it sends plain (point-0, untagged) frames.
        This single coroutine is "one streamer to rule them all" — normal
        streaming is just the modulation-off case.
        """
        err_count=0
        fast_read_params = firmware_lib.get_fast_read_params(self.r_fast)
        while True:
            try:
                if not self.e_stream_enabled.is_set():
                    await asyncio.sleep(0.1)  # idle
                    continue

                # Apply any queued enable/update/disable/recenter at the boundary.
                self._apply_pending_modulation_command()

                if self.e_modulation_enabled.is_set() and self.modulation_sched is not None:
                    self._modulation_owner = 'stream'
                    sched = self.modulation_sched
                    # One full cycle through the N points; point 1 (already live
                    # from arm/last advance) is emitted before the first swap.
                    for _ in range(sched.N):
                        p = sched.current_point()
                        for s in range(sched.samples_per_point):
                            settling = s < sched.n_settle
                            payload, cnt, err = self.prepare_frame(
                                fast_read_params, mod_point=p + 1,
                                settling=settling, revision=sched.revision)
                            self._write_to_stream_clients(payload)
                        sched.advance()
                        await asyncio.sleep(0)  # stay responsive to new commands
                    # drain once per cycle to bound buffering
                    for client in list(self.stream_clients):
                        try:
                            await client.drain()
                        except Exception:
                            pass
                else:
                    self._modulation_owner = None
                    payload, cnt, err = self.prepare_frame(fast_read_params)
                    if err:
                        err_count += 1
                        print('Packet error:', cnt, err_count)
                    self._write_to_stream_clients(payload)
                    for client in list(self.stream_clients):
                        try:
                            await client.drain()
                        except ConnectionResetError:
                            print(f"Stream client disconnected {getattr(client, 'addr', '?')} (ConnectionResetError)")
                            if client in self.stream_clients:
                                self.stream_clients.remove(client)
                    await asyncio.sleep(0)  # release control to other tasks
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

    async def sweep(self, centers, spans, points, samples_per_point, direction, refresh_adc_cal=True, adc_cal_settle_time=2.0):
        """
        A coroutine that performs a frequency sweep and stores the results in self.latest_sweep_data.

        ADC calibration is always frozen before sweeping and left frozen
        afterwards. By default the calibration is refreshed first (unfreeze,
        settle, freeze) so it adapts to the current tone configuration. Set
        refresh_adc_cal=False to skip the refresh and sweep immediately with the
        existing frozen calibration.

        Args:
            refresh_adc_cal (bool): If True (default), refresh ADC calibration
                before sweeping (unfreeze, settle, freeze). If False, ensure
                the calibration is frozen (settling first if not already
                frozen) but skip the full refresh.
            adc_cal_settle_time (float): Seconds to wait for ADC calibration to settle.
                Default 2.0.
        """

        sweep_tone_amplitudes = None
        sweep_tone_phases = None
        init_psb_scale = None

        try:

            self.latest_sweep_data_valid = False
            self.sweep_progress = 0.0

            centers, spans = self._expand_sweep_request_for_blind(centers, spans)
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
            metadata = firmware_lib.get_configured_tone_metadata(
                self.config, active_count=len(centers))
            if metadata['blind_indices']:
                plan = self._tone_plan()
                initial_freqs[metadata['blind_indices']] = plan['blind_frequencies']

            print('Setting initial tone frequencies')
            firmware_lib.set_tone_frequencies_fast(self.r,self.r_fast,self.config,centers,autosync=True)
            try:
                sweep_tone_amplitudes = firmware_lib.get_tone_amplitudes(
                    self.r, self.config)
                sweep_tone_phases = firmware_lib.get_tone_phases(
                    self.r, self.config)
            except Exception as e:
                print(f'Warning: could not preserve sweep tone amplitudes/phases: {e}')
                sweep_tone_amplitudes = None
                sweep_tone_phases = None
            if sweep_tone_amplitudes is not None and len(sweep_tone_amplitudes) != len(centers):
                sweep_tone_amplitudes = None
            if sweep_tone_phases is not None and len(sweep_tone_phases) != len(centers):
                sweep_tone_phases = None

            # ADC calibration: always frozen before sweep, left frozen after
            # Note: uses inline set_cal_freeze rather than firmware_lib.refresh_adc_cal()
            # because we need async sleep to avoid blocking the event loop.
            if refresh_adc_cal:
                # Full refresh: unfreeze, settle, freeze
                print('Refreshing ADC calibration...')
                firmware_lib.set_cal_freeze(self.r, self.config, False)
                self.stream_flags[FLAG_CAL_FREEZE].clear()
                print(f'Waiting {adc_cal_settle_time}s for ADC calibration to settle...')
                await asyncio.sleep(adc_cal_settle_time)
                print('Freezing ADC calibration')
                firmware_lib.set_cal_freeze(self.r, self.config, True)
                self.stream_flags[FLAG_CAL_FREEZE].set()
            else:
                # Ensure frozen, settling first if needed
                if not firmware_lib.get_cal_freeze(self.r, self.config):
                    print(f'Waiting {adc_cal_settle_time}s for ADC calibration to settle...')
                    await asyncio.sleep(adc_cal_settle_time)
                    print('Freezing ADC calibration')
                    firmware_lib.set_cal_freeze(self.r, self.config, True)
                    self.stream_flags[FLAG_CAL_FREEZE].set()

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
            fast_sweep_params = firmware_lib.prepare_sweep_settings_fast(
                self.r_fast, self.config, sweepfreqs,
                tone_amplitudes=sweep_tone_amplitudes,
                tone_phases=sweep_tone_phases)

            # If tone amplitudes were globally scaled down to protect the VACC,
            # raise psb_scale by the inverse to keep absolute output power.
            amplitude_scale_factor = fast_sweep_params.get('amplitude_scale_factor', 1.0)
            if amplitude_scale_factor and amplitude_scale_factor < 1.0:
                psb_scale_min = 1.0 / 256
                psb_scale_max = 255.0
                init_psb_scale = self.r.psbscale.get_scale()
                desired_scale = init_psb_scale / amplitude_scale_factor
                new_scale = float(np.clip(desired_scale, psb_scale_min, psb_scale_max))
                self.r.psbscale.set_scale(new_scale)
                if new_scale < desired_scale:
                    shortfall_db = 20 * np.log10(new_scale / desired_scale)
                    print(f'Warning: psb_scale clipped to {new_scale:.4f} '
                          f'(wanted {desired_scale:.4f} to fully compensate '
                          f'amplitude scaling of {amplitude_scale_factor:.4f}); '
                          f'sweep absolute power reduced by {-shortfall_db:.2f} dB')
                else:
                    print(f'psb_scale raised {init_psb_scale:.4f} -> {new_scale:.4f} '
                          f'to compensate for amplitude scaling of {amplitude_scale_factor:.4f}')

            # Get tone_indices array - shape (num_points, num_tones)
            # These may change at each sweep point as tones cross FFT bin boundaries
            tone_indices_arr = fast_sweep_params.get('tone_indices')
            
            print('Starting sweep')
            acc_counts = np.zeros((samples_per_point,num_points),dtype=int)
            sweep_data = np.zeros((samples_per_point,num_points,num_tones),dtype=complex)
            acc_errs = np.zeros((samples_per_point,num_points),dtype=bool)
            sweep_tt = np.zeros(num_points,dtype=np.uint64)

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
                    cnt,data,err,tt = firmware_lib.read_accumulated_data_fast(
                                                            fast_read_params,
                                                            tone_indices=tone_indices_p)
                    acc_counts[s,p] = cnt
                    sweep_data[s,p] = data[::2]+1j*data[1::2]
                    acc_errs[s,p] = err
                    if s == 0:
                        sweep_tt[p] = tt
                # time.sleep(0.001)

                self.sweep_progress = float(p/(num_points-1))
                await asyncio.sleep(0.0001)

            # print('reset initial freqs:',initial_freqs)
            firmware_lib.set_tone_frequencies_fast(
                self.r, self.r_fast, self.config, initial_freqs,
                autosync=True,
                tone_amplitudes=sweep_tone_amplitudes,
                tone_phases=sweep_tone_phases)
            self.update_active_tone_indices()

            if init_psb_scale is not None:
                self.r.psbscale.set_scale(init_psb_scale)
                init_psb_scale = None

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
                'accumulation_errors': acc_errs,
                'telescope_time': sweep_tt
                }
            self.latest_sweep_data = {
                'sweep_data': sweep_data
                }
            # results = firmware_lib.perform_sweep(self.r,self.r_fast, self.config, center, span, points, samples_per_point, direction)
            #self.latest_sweep_data['f'] = results['sweep_frequencies']
            #self.latest_sweep_data['z'] = results['sweep_responses']
            #self.latest_sweep_data['e'] = results['sweep_stds']
            self.latest_sweep_data_valid = True
            self.sweep_state = {'state': 'success', 'message': 'Sweep complete'}
            return True

        except asyncio.CancelledError:
            print('ayncio sweep cancelled')
            firmware_lib.set_tone_frequencies_fast(
                self.r, self.r_fast, self.config, initial_freqs,
                autosync=True,
                tone_amplitudes=sweep_tone_amplitudes,
                tone_phases=sweep_tone_phases)
            self.update_active_tone_indices()
            if init_psb_scale is not None:
                self.r.psbscale.set_scale(init_psb_scale)
                init_psb_scale = None
            self.latest_sweep_results = {
                'sweep_frequencies': sweepfreqs,
                'sweep_responses': sweep_responses,
                'sweep_stds': sweep_stds,
                'sweep_sems': sweep_sems,
                'samples_per_point': samples_per_point,
                'samples_per_second': firmware_lib.get_sample_rate(self.r_fast),
                'accumulation_counts': acc_counts,
                'accumulation_errors': acc_errs,
                'telescope_time': sweep_tt
                }

            self.sweep_state = {'state': 'cancelled', 'message': 'Sweep cancelled'}
            return False
        except Exception as e:
            self.sweep_progress = float(1.0)
            self.sweep_state = {'state': 'error', 'message': f'Error performing sweep: {e}'}
            print(f"Error performing sweep: {e}")
            print(traceback.format_exc())
            return False
        finally:
            if init_psb_scale is not None:
                try:
                    self.r.psbscale.set_scale(init_psb_scale)
                except Exception as e:
                    print(f'Warning: could not restore psb_scale={init_psb_scale}: {e}')

    async def retune(self, center, span, points, samples_per_point, direction, method,freq_offsets=None, refresh_adc_cal=True, adc_cal_settle_time=2.0):
        """
        A coroutine that performs a frequency sweep, finds the resonance peaks using the specified method, and sets the tones to the peak frequencies.

        Args:
            refresh_adc_cal (bool): If True (default), refresh ADC calibration
                before sweeping. If False, skip refresh but still ensure frozen.
            adc_cal_settle_time (float): Seconds to wait for ADC calibration to settle.
                Default 2.0.
        """

        center, span = self._expand_sweep_request_for_blind(center, span)
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
            freq_offsets = np.atleast_1d(freq_offsets).astype(float)
            plan = self._tone_plan()
            if (plan['num_blind_tones'] > 0
                    and len(freq_offsets) == plan['num_regular_tones']
                    and len(center) == plan['num_tones']):
                freq_offsets = np.concatenate([
                    freq_offsets,
                    np.zeros(plan['num_blind_tones'], dtype=float)])
        if freq_offsets.shape != center.shape:
            raise ValueError("freq_offsets must be None, a scalar, or have the same shape as centers")
        if np.any(np.abs(freq_offsets) > span/2):
            print("Warning: some freq_offsets are larger than half the span, which may cause tones to be set outside the sweep range")

        try:
            if method not in ('max_gradient','min_mag','max_dphidf'):
                raise ValueError(f'Invalid retune method "{method}", must be "max_gradient", "min_mag", or "max_dphidf"')

            sweep_ok = await self.sweep(center, span, points, samples_per_point, direction, refresh_adc_cal=refresh_adc_cal, adc_cal_settle_time=adc_cal_settle_time)
            if not sweep_ok:
                if self.sweep_state.get('state') not in ('cancelled', 'error'):
                    self.sweep_progress = float(1.0)
                    self.sweep_state = {'state': 'error', 'message': 'Error performing retune: sweep failed'}
                return False

            # sweep_f = self.latest_sweep_data['f']
            # sweep_z = self.latest_sweep_data['z']
            # sweep_e = self.latest_sweep_data['e']

            sweep_f = self.latest_sweep_results['sweep_frequencies']
            sweep_z = self.latest_sweep_results['sweep_responses']
            sweep_e = self.latest_sweep_results['sweep_sems']

            metadata = firmware_lib.get_configured_tone_metadata(
                self.config, active_count=len(center))
            regular_indices = metadata['regular_indices']
            blind_indices = metadata['blind_indices']
            plan = self._tone_plan()
            retune_freqs = np.array(center, dtype=float)

            if method == 'max_gradient':
                for t in regular_indices:
                    freqs = sweep_f[:,t]
                    grads = np.abs(np.gradient(sweep_z[:,t]))
                    max_grad = np.argmax(grads)
                    retune_freqs[t] = freqs[max_grad] + freq_offsets[t]
            elif method == 'min_mag':
                for t in regular_indices:
                    freqs = sweep_f[:,t]
                    mags = np.abs(sweep_z[:,t])
                    min_mag = np.argmin(mags)
                    retune_freqs[t] = freqs[min_mag] + freq_offsets[t]
            elif method == 'max_dphidf':
                for t in regular_indices:
                    freqs = sweep_f[:,t]
                    phase = np.unwrap(np.angle(sweep_z[:,t]))
                    dphidf = np.abs(np.gradient(phase, freqs))
                    max_slope = np.argmax(dphidf)
                    retune_freqs[t] = freqs[max_slope] + freq_offsets[t]

            if blind_indices:
                retune_freqs[blind_indices] = plan['blind_frequencies']

            print('Retune freqs = found freqs + freq offsets = ',retune_freqs)

            firmware_lib.set_tone_frequencies_fast(self.r,self.r_fast,self.config,retune_freqs)
            self.update_active_tone_indices()
            self.sweep_progress = 1.0
            self.sweep_state = {'state': 'success', 'message': 'Retune complete'}
            # print('New frequencies:',firmware_lib.get_tone_frequencies(self.r,self.config))

            # results = firmware_lib.perform_retune(self.r,self.r_fast, self.config, center, span, points, samples_per_point, direction, method)
            # self.latest_sweep_data['f'] = results['sweep_frequencies']
            # self.latest_sweep_data['z'] = results['sweep_responses']
            # self.latest_sweep_data['e'] = results['sweep_stds']
            # self.latest_sweep_data_valid = True
            return True

        except asyncio.CancelledError:
            self.sweep_state = {'state': 'cancelled', 'message': 'Retune cancelled'}
            return False
        except Exception as e:
            self.sweep_progress = float(1.0)
            self.sweep_state = {'state': 'error', 'message': f'Error performing retune: {e}'}
            print(f"Error performing retune: {e}")
            print(traceback.format_exc())
            return False

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
        
        _server_log(
            f'request server listening on '
            f'{_format_socket_name(request_server.sockets[0].getsockname())}',
            source='network',
        )
        _server_log(
            f'stream server listening on '
            f'{_format_socket_name(stream_server.sockets[0].getsockname())}',
            source='network',
        )
        
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
