"""
LNA bias control and monitoring for the SOUK cryostat.

Wraps the souk-peripherals-control ``SOUKLNABiasControlMonitor`` to provide
a simplified interface for setting and reading LNA bias voltages and
currents.  Supports all 14 LNA channels in the cryostat.

Each readout pipeline maps to one LNA channel via ``cryostat.lna_bias.lna_channel``
in the config.  Any LNA channel (1-14) can also be addressed directly.

Hardware control requires ``smbus2`` and the ``souk-peripherals-control``
submodule on the server.
"""

import logging
import os
import sys

logger = logging.getLogger(__name__)

# Add the submodule to sys.path so its internal imports resolve.
_SUBMODULE_DIR = os.path.join(os.path.dirname(__file__), 'souk-peripherals-control')
if _SUBMODULE_DIR not in sys.path:
    sys.path.insert(0, _SUBMODULE_DIR)

# Hardware imports -- may fail on client machines.
try:
    from smbus2 import SMBus
    from souk_lna_bias_control_monitor import (
        SOUKLNABiasControlMonitor,
        SOUKLNABiasControlMonitorHWConfig,
    )
    from lna_monitor import LNAMonitorHWConfig
    from ad511_0_2_4bcpz_5_10_80 import AD511_0_2_4BCPZ_5_10_80HWConfig
    from ltc2481cdd import LTC2481CDDHWConfig
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False


NUM_LNA_CHANNELS = 14


def _default_lna_hw_config():
    """Return the default hardware config for the SOUK LNA bias board (rev 1).

    All 14 channels are configured with identical per-channel parameters
    matching the production board.  Resistor selections match the default
    solder-jumper positions.
    """
    if not _HW_AVAILABLE:
        return None

    per_channel = LNAMonitorHWConfig(
        r_dac_hw_config=AD511_0_2_4BCPZ_5_10_80HWConfig(
            DEV_ADDR=0x2C, RESOLUTION=128, R_FULL_SCALE_KOHM=10.0,
        ),
        remote_adc_hw_config=LTC2481CDDHWConfig(CA0="low", CA1="float"),
        imonitor_adc_hw_config=LTC2481CDDHWConfig(CA0="float", CA1="float"),
        switch_status=True,
        r_LDO_set_kOhm=150.0,
        r_RTop1_kOhm=9.88,
        r_RBot1_kOhm=12.4,
        r_RAdj1_kOhm=1e7,
        r_RSENSE_OHMS=10.0,
    )

    # M17-M30 map to LNA channels 1-14
    refdes = [f"M{i}" for i in range(17, 31)]
    lna_configs = {ref: per_channel for ref in refdes}

    return SOUKLNABiasControlMonitorHWConfig(
        lna_monitor_hw_configs=lna_configs,
        r9_r12="R12",
        r8_r10="R10",
        r7_r5="R7",
        r11_r13="R13",
        r14_r15="R14",
        r6_r4="R6",
    )


class LNABiasController:
    """
    Controller for the SOUK cryostat LNA bias system.

    Provides voltage and current control/monitoring for up to 14 LNA
    channels via I2C.  Each readout pipeline maps to one LNA channel
    specified by ``cryostat.lna_bias.lna_channel`` in the config.

    Parameters
    ----------
    config_dict : dict
        The live server config (modified in place).
    pipeline_id : int
        Pipeline index.
    """

    def __init__(self, config_dict, pipeline_id=0):
        self.config = config_dict
        self.pipeline_id = pipeline_id
        self._monitor = None

        cryo_cfg = config_dict.get('cryostat', {})
        lna_cfg = cryo_cfg.get('lna_bias', {})

        self.enabled = lna_cfg.get('enabled', False)
        if not self.enabled:
            logger.info('LNA bias control disabled in config')
            return

        self._lna_channel = lna_cfg.get('lna_channel', 1)
        i2c_bus_num = lna_cfg.get('i2c_bus', 0)

        if not _HW_AVAILABLE:
            logger.warning(
                'LNA bias hardware libraries not available '
                '(smbus2 / souk-peripherals-control)'
            )
            return

        try:
            bus = SMBus(i2c_bus_num)
            hw_config = _default_lna_hw_config()
            self._monitor = SOUKLNABiasControlMonitor(bus, hw_config)
            logger.info(
                'LNA bias controller initialised on I2C bus %d, '
                'pipeline LNA channel %d',
                i2c_bus_num, self._lna_channel,
            )
        except Exception:
            logger.exception('Failed to initialise LNA bias hardware')
            self._monitor = None

    # -- properties --

    @property
    def is_hardware(self):
        """True if controlling real hardware."""
        return self._monitor is not None

    @property
    def lna_channel(self):
        """The default LNA channel index (1-14) for this pipeline."""
        return self._lna_channel

    # -- set voltage --

    def set_lna_bias_voltage(self, voltage_v, channel=None,
                             method='remote', blind=False):
        """
        Set LNA bias voltage.

        Parameters
        ----------
        voltage_v : float
            Target voltage in volts.
        channel : int, optional
            LNA channel index (1-14).  Defaults to this pipeline's lna_channel.
        method : str
            ``'remote'`` (default, iterative feedback) or ``'local'`` (direct
            DAC setting).
        blind : bool
            If True and method='remote', skip LNA voltage validation.

        Returns
        -------
        dict
            Result with achieved voltage and any error message.
        """
        self._require_hardware()
        chn = channel if channel is not None else self._lna_channel
        self._validate_channel(chn)

        if method == 'local':
            result = self._monitor.set_lna_bias_local(chn=[chn], v_local=voltage_v)
            return {
                'channel': chn,
                'voltage_v': result[chn],
                'method': 'local',
            }
        else:
            result = self._monitor.set_lna_bias_remote(
                chn=[chn], v_local=voltage_v, blind=blind,
            )
            return {
                'channel': chn,
                'voltage_v': result[chn][0],
                'method': 'remote',
                'message': result[chn][1],
            }

    def set_lna_bias_voltage_all(self, voltage_v, method='remote', blind=False):
        """
        Set bias voltage for all 14 LNA channels.

        Parameters
        ----------
        voltage_v : float
            Target voltage in volts.
        method : str
            ``'remote'`` (default) or ``'local'``.
        blind : bool
            If True and method='remote', skip LNA voltage validation.

        Returns
        -------
        dict
            Per-channel results keyed by channel index (1-14).
        """
        self._require_hardware()
        channels = list(range(1, NUM_LNA_CHANNELS + 1))

        if method == 'local':
            result = self._monitor.set_lna_bias_local(
                chn=channels, v_local=voltage_v,
            )
            return {
                chn: {'channel': chn, 'voltage_v': result[chn], 'method': 'local'}
                for chn in channels
            }
        else:
            result = self._monitor.set_lna_bias_remote(
                chn=channels, v_local=voltage_v, blind=blind,
            )
            return {
                chn: {
                    'channel': chn,
                    'voltage_v': result[chn][0],
                    'method': 'remote',
                    'message': result[chn][1],
                }
                for chn in channels
            }

    # -- read status --

    def get_lna_bias_status(self, channel=None):
        """
        Read LNA bias voltage and current.

        Parameters
        ----------
        channel : int, optional
            LNA channel index (1-14).  Defaults to this pipeline's lna_channel.

        Returns
        -------
        dict
            With keys: channel, remote_voltage_v, local_voltage_v, bias_current_a.
        """
        self._require_hardware()
        chn = channel if channel is not None else self._lna_channel
        self._validate_channel(chn)

        status = self._monitor.read_lna_status(chn=[chn])
        s = status[chn]
        return {
            'channel': chn,
            'remote_voltage_v': s['remote voltage'],
            'local_voltage_v': s['local voltage'],
            'bias_current_a': s['bias current'],
        }

    def get_lna_bias_status_all(self):
        """
        Read bias voltage and current for all 14 LNA channels.

        Returns
        -------
        dict
            Per-channel status keyed by channel index (1-14).
        """
        self._require_hardware()
        channels = list(range(1, NUM_LNA_CHANNELS + 1))
        status = self._monitor.read_lna_status(chn=channels)
        return {
            chn: {
                'channel': chn,
                'remote_voltage_v': status[chn]['remote voltage'],
                'local_voltage_v': status[chn]['local voltage'],
                'bias_current_a': status[chn]['bias current'],
            }
            for chn in channels
        }

    # -- status summary --

    def get_status(self):
        """Return a summary dict of the controller state."""
        if not self.enabled:
            return {'enabled': False}
        return {
            'enabled': True,
            'hardware': self.is_hardware,
            'lna_channel': self._lna_channel,
        }

    # -- internal --

    def _validate_channel(self, channel):
        if not (1 <= channel <= NUM_LNA_CHANNELS):
            raise ValueError(
                f'LNA channel must be 1-{NUM_LNA_CHANNELS}, got {channel}'
            )

    def _require_hardware(self):
        if self._monitor is None:
            raise RuntimeError('LNA bias hardware not initialised')
