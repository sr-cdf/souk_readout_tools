"""
Shared configuration utilities for souk_readout_tools.

Functions for locating and copying template config files, used by both
the client and server packages.
"""

import os
import re
from datetime import date
from importlib.resources import files as importlib_files

try:
    import pwd  # POSIX-only; not available on Windows
except ImportError:
    pwd = None

# The user that owns the persistent souk_readout_tools data on the board. The
# server runs under sudo (needed for /dev/mem), so this is resolved explicitly
# rather than from $HOME, which would point at /root.
TARGET_USER = 'casper'


def _get_target_ownership():
    """
    If running as root (e.g. via sudo), return (uid, gid) of the target user
    so that created files are owned by them rather than root.
    Returns None if not running as root, or on platforms without POSIX
    user/permission semantics (e.g. Windows).
    """
    if pwd is None or not hasattr(os, 'geteuid'):
        return None
    if os.geteuid() != 0:
        return None
    try:
        pw = pwd.getpwnam(TARGET_USER)
        return pw.pw_uid, pw.pw_gid
    except KeyError:
        uid = int(os.getenv('SUDO_UID') or 0)
        gid = int(os.getenv('SUDO_GID') or 0)
        return uid, gid


def get_user_dir():
    """
    Return the ``~/.souk_readout_tools`` data directory for the target user.

    Persistent data - configs, calibration files - lives in the user's home.
    The server runs under sudo, so ``expanduser('~')`` would resolve to /root;
    when running as root, resolve the target user's home explicitly instead
    (mirroring the server's HOME resolution).
    """
    home = None
    if pwd is not None and hasattr(os, 'geteuid') and os.geteuid() == 0:
        try:
            home = pwd.getpwnam(TARGET_USER).pw_dir
        except KeyError:
            home = os.path.join('/home', TARGET_USER)
    if home is None:
        home = os.path.expanduser('~')
    return os.path.join(home, '.souk_readout_tools')


def get_template_config_path():
    """Return the path to the bundled template config file in the package data."""
    return str(importlib_files('souk_readout_tools').joinpath('data', 'config', 'template_config.yaml'))


# ---------------------------------------------------------------------
# Site file
# ---------------------------------------------------------------------
# Board-wide settings that are the same for every pipeline on a machine, and
# that describe where shared telescope hardware lives. Kept out of the
# per-pipeline configs so that moving a piece of shared hardware means editing
# one file per machine rather than every config.

SITE_CONFIG_FILENAME = 'site.yaml'

DEFAULT_LNA_SERVICE_PORT = 10500
DEFAULT_LNA_SERVICE_TIMEOUT_S = 15.0


def get_site_config_path():
    """Return the path to this machine's ``site.yaml``.

    The file is optional; ``load_site_config`` returns an empty dict when it
    does not exist.
    """
    return os.path.join(get_user_dir(), SITE_CONFIG_FILENAME)


def load_site_config():
    """Load this machine's ``site.yaml``, or return ``{}`` if absent.

    A malformed site file raises, rather than being silently ignored — a
    typo there would otherwise send LNA traffic nowhere with no explanation.
    """
    import yaml

    path = get_site_config_path()
    if not os.path.exists(path):
        return {}
    with open(path) as f:
        content = yaml.safe_load(f)
    if content is None:
        return {}
    if not isinstance(content, dict):
        raise ValueError(
            f'Site config {path} must contain a YAML mapping, '
            f'got {type(content).__name__}'
        )
    return content


def resolve_lna_service_endpoint(lna_cfg=None, site_cfg=None):
    """Resolve where the LNA bias service lives, as ``(host, port, timeout_s)``.

    Only one RFSoC per telescope is wired to the LNA bias board, so every
    other readout server reaches it over the network. The address is looked up
    in this order:

    1. ``host``/``port`` in the pipeline config's ``cryostat.lna_bias``
       section (per-pipeline override, useful on the bench),
    2. ``lna_service.host``/``.port`` in this machine's ``site.yaml``.

    Raises ValueError naming the site file if neither supplies a host, since
    that is the actionable fix.
    """
    lna_cfg = lna_cfg or {}
    if site_cfg is None:
        site_cfg = load_site_config()
    service_cfg = site_cfg.get('lna_service') or {}

    host = lna_cfg.get('host') or service_cfg.get('host')
    if not host:
        raise ValueError(
            "LNA bias backend 'remote' needs the address of the RFSoC "
            'running the LNA bias service. Set lna_service.host in '
            f'{get_site_config_path()}, or cryostat.lna_bias.host in the '
            'pipeline config.'
        )
    port = (lna_cfg.get('request_port') or service_cfg.get('port')
            or DEFAULT_LNA_SERVICE_PORT)
    timeout_s = (lna_cfg.get('timeout_s') or service_cfg.get('timeout_s')
                 or DEFAULT_LNA_SERVICE_TIMEOUT_S)
    return str(host), int(port), float(timeout_s)


def copy_template_config(destination, pipeline_id=0, nyquist_zone=1,
                         config_id=None, created_by=None, comments=None):
    """
    Copy the template config to a destination file, updating pipeline-specific fields.

    Copies the template as raw text so that comments are preserved.

    Args:
        destination: Path to write the config file.
        pipeline_id: Pipeline ID (0 or 1) to set in the template.
            This also sets the default SOUK mixerless-module
            ``rf_frontend.mixerless_module.rf_channel`` to the same value.
            It does not change ``cryostat.lna_bias.i2c.bus``; the LNA bias
            board is always on SMBus(0).
        nyquist_zone: Nyquist zone (1 or 2). Sets the ``nyquist_zone`` key
            in the config, which controls the DAC/ADC mix-mode setting and 
            mixer frequencies: zone 1 uses fs/4 (~1228.8 MHz), zone 2 uses 
            3*fs/4 (~3686.4 MHz). Default is 1.
        config_id: Optional config identifier string.
        created_by: Optional author/creator string.
        comments: Optional comments string.
    """
    template_src = get_template_config_path()

    with open(template_src, 'r') as f:
        text = f.read()

    # Update creation date to today
    text = re.sub(
        r'(creation_date:\s*)"[^"]*"',
        rf'\1"{date.today().isoformat()}"',
        text,
    )

    # Optional metadata overrides
    if config_id is not None:
        text = re.sub(
            r'(config_id:\s*)"[^"]*"',
            rf'\1"{config_id}"',
            text,
        )
    if created_by is not None:
        text = re.sub(
            r'(created_by:\s*)"[^"]*"',
            rf'\1"{created_by}"',
            text,
        )
    if comments is not None:
        text = re.sub(
            r'(  comments:\s*)"[^"]*"',
            rf'\1"{comments}"',
            text,
            count=1,
        )

    # Nyquist zone
    if nyquist_zone not in (1, 2):
        raise ValueError(f'nyquist_zone must be 1 or 2, got {nyquist_zone}')
    if nyquist_zone != 1:
        text = re.sub(
            r'(nyquist_zone:\s*)1',
            rf'\g<1>{nyquist_zone}',
            text,
        )

    # Pipeline-specific fields
    text = re.sub(
        r'(pipeline_id:\s*)0',
        rf'\g<1>{pipeline_id}',
        text,
    )
    text = re.sub(
        r'(request_port:\s*)10000',
        rf'\g<1>{10000 + pipeline_id}',
        text,
    )
    text = re.sub(
        r'(stream_port:\s*)20000',
        rf'\g<1>{20000 + pipeline_id}',
        text,
    )
    text = re.sub(
        r'(rf_channel:\s*)\d+',
        rf'\g<1>{pipeline_id}',
        text,
    )

    # RFDC tile/block mapping for pipeline 1
    if pipeline_id == 1:
        text = re.sub(r'(dac0_tile:\s*)0', r'\g<1>1', text)
        text = re.sub(r'(dac1_tile:\s*)0', r'\g<1>1', text)
        text = re.sub(r'(adc_tile:\s*)2', r'\g<1>3', text)

    with open(destination, 'w') as f:
        f.write(text)

    os.chmod(destination, 0o664)
    ownership = _get_target_ownership()
    if ownership is not None:
        os.chown(destination, *ownership)

    print(f'Template config written to {destination} '
          f'(pipeline {pipeline_id}, Nyquist zone {nyquist_zone})')
    print(f'Edit this file with your system-specific settings before use.')
