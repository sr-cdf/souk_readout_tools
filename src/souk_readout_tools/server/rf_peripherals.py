"""
Abstraction layer for RF peripheral hardware (attenuators, amplifier bypass).

Supports multiple backends:
- **souk-peripherals-control** (default): I2C-controlled SOUK RF Mixerless
  Module with variable attenuator and bypassable amplifier per path.
- **rudat**: Mini-Circuits RUDAT USB attenuators (attenuator-only, no amp
  bypass). Useful for bench testing without the mixerless module.

Hardware control requires smbus2 (mixerless) or pyusb (RUDAT) on the server.
When neither is available, a software-only mimic is used for offline
transfer-function analysis.
"""

import logging
import os
import sys

logger = logging.getLogger(__name__)

# Add the submodule to sys.path so its internal imports resolve.
_SUBMODULE_DIR = os.path.join(os.path.dirname(__file__), 'souk-peripherals-control')
if _SUBMODULE_DIR not in sys.path:
    sys.path.insert(0, _SUBMODULE_DIR)

# Import the software-only model classes (no smbus2 required).
from souk_rf_mixerless_atten_amp_level import (
    SOUKRFMixerlessTransmitAttenAmpLevel,
    SOUKRFMixerlessRecvAttenAmpLevel,
    SOUKRFMixerlessAttenAmpTransfer,
)

# Hardware imports – may fail on client machines.
try:
    from smbus2 import SMBus
    from souk_rf_mixerless_module import (
        SOUKRFMixerlessModule,
        SOUKRFMixerlessModuleChnHWConfig,
    )
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False

# Software mimic – always available.
from souk_rf_mixerless_atten_amp_level import (
    mimicSOUKRFMixerlessModule,
)

# RUDAT USB attenuator support – optional.
# rudat.py must be on sys.path (e.g. pip install, or add its directory to
# PYTHONPATH / sys.path before starting the server).
try:
    from rudat import Attenuator as RudatAttenuator, find_rudats
    _RUDAT_AVAILABLE = True
except ImportError:
    _RUDAT_AVAILABLE = False


# ---------------------------------------------------------------------
# RUDAT adapter — presents two USB attenuators with the same interface
# that RFPeripheralController expects from _module().
# ---------------------------------------------------------------------

class _RudatStubAmpLevel:
    """Stub for the amp portion — no amp present, 0 dB contribution."""
    total_gain_il = 0.0

class _RudatStubAttenAmpLevel:
    """Stub atten_amp_level for RUDAT (attenuator-only, no amp)."""
    def __init__(self):
        self.amp = _RudatStubAmpLevel()
        self.total_gain_il = 0.0
        self.input_1dB_comp = 999.0  # effectively unlimited

class _RudatStubAttenAmp:
    """Stub returned by _get_atten_amp for RUDAT."""
    def __init__(self):
        self.atten_amp_level = _RudatStubAttenAmpLevel()


class RudatAdapter:
    """
    Adapts two RUDAT USB attenuators to the interface expected by
    RFPeripheralController._module().

    No bypass amplifier — amp methods are no-ops returning fixed values.

    Parameters
    ----------
    tx_rudat : rudat.Attenuator
        RUDAT controlling the TX path attenuation.
    rx_rudat : rudat.Attenuator
        RUDAT controlling the RX path attenuation.
    """

    def __init__(self, tx_rudat, rx_rudat):
        self._attens = {
            'transmit_atten': tx_rudat,
            'recv_atten': rx_rudat,
        }
        self._bypass_state = {
            'transmit_atten': True,   # no amp — always "bypassed"
            'recv_atten': True,
        }

    def set_attenuation(self, channel, dev_name, attenuation_db):
        self._attens[dev_name].att = float(attenuation_db)

    def get_attenuation_value(self, channel, dev_name):
        return float(self._attens[dev_name].att)

    def set_amp_bypass_state(self, channel, dev_name, bypass):
        # No amp to bypass — accept the call but do nothing.
        pass

    def get_amp_bypass_state(self, channel, dev_name):
        return True  # always bypassed (no amp)

    def _get_atten_amp(self, channel, dev_name):
        stub = _RudatStubAttenAmp()
        atten = self.get_attenuation_value(channel, dev_name)
        stub.atten_amp_level.total_gain_il = -abs(atten)
        return stub

    def get_transfer(self, channel):
        return None  # not applicable for standalone attenuators


class RFPeripheralController:
    """
    Unified interface for the SOUK RF Mixerless Module.

    Provides attenuator control (0–31.5 dB in 0.5 dB steps) and amplifier
    bypass for both TX and RX paths.  When hardware is available, commands
    are forwarded to the I2C-controlled module.  Otherwise a software mimic
    is used for offline modelling.

    After every state change the controller updates the supplied config dict
    in memory so that the calibration chain sees the correct values.

    Parameters
    ----------
    config_dict : dict
        The live server config (modified in place).
    pipeline_id : int
        Pipeline index, used to select the hardware channel.
    """

    # -- attenuation limits (hardware) --
    ATTEN_MIN = 0.0
    ATTEN_MAX = 31.5
    ATTEN_STEP = 0.5

    def __init__(self, config_dict, pipeline_id=0):
        self.config = config_dict
        self.pipeline_id = pipeline_id
        self._hw_module = None
        self._mimic_module = None

        rf_cfg = config_dict.get('rf_frontend', {})
        mod_cfg = rf_cfg.get('mixerless_module', {})

        connected = rf_cfg.get('connected', False)
        attenuator_backend = rf_cfg.get('attenuator_backend') or None
        self.enabled = connected and attenuator_backend is not None
        if not connected:
            logger.info('RF frontend not connected in config')
            return
        if attenuator_backend is None:
            logger.info('No programmable attenuator backend configured')
            return
        if attenuator_backend not in ('i2c', 'rudat'):
            raise ValueError(
                f"Unknown attenuator_backend '{attenuator_backend}'. "
                f"Supported: 'i2c', 'rudat'"
            )

        self._i2c_bus_num = mod_cfg.get('i2c_bus', 0)
        self._channel = mod_cfg.get('channel', pipeline_id)

        # -- RUDAT attenuator backend --
        if attenuator_backend == 'rudat':
            if not _RUDAT_AVAILABLE:
                raise ImportError(
                    'RUDAT backend selected but rudat module not found. '
                    'Install pyusb and ensure rudat.py is on sys.path.'
                )
            tx_serial = rf_cfg.get('rudat_tx_serial')
            rx_serial = rf_cfg.get('rudat_rx_serial')
            if tx_serial is None or rx_serial is None:
                raise ValueError(
                    'RUDAT backend requires rudat_tx_serial and '
                    'rudat_rx_serial in rf_frontend config'
                )
            rudats = find_rudats()
            tx_key = int(tx_serial) if str(tx_serial).isdigit() else tx_serial
            rx_key = int(rx_serial) if str(rx_serial).isdigit() else rx_serial
            if tx_key not in rudats:
                raise ValueError(
                    f'TX RUDAT serial {tx_serial} not found. '
                    f'Available: {list(rudats.keys())}'
                )
            if rx_key not in rudats:
                raise ValueError(
                    f'RX RUDAT serial {rx_serial} not found. '
                    f'Available: {list(rudats.keys())}'
                )
            tx_att = RudatAttenuator(rudats[tx_key]['bus'], rudats[tx_key]['address'])
            rx_att = RudatAttenuator(rudats[rx_key]['bus'], rudats[rx_key]['address'])
            self._hw_module = RudatAdapter(tx_att, rx_att)
            self.ATTEN_MIN = max(tx_att.att_min, rx_att.att_min)
            self.ATTEN_MAX = min(tx_att.att_max, rx_att.att_max)
            self.ATTEN_STEP = max(tx_att.resolution, rx_att.resolution)
            logger.info(
                'RUDAT attenuators initialised: TX serial %s, RX serial %s',
                tx_serial, rx_serial,
            )
            self._sync_config_from_hardware()
            return

        # -- I2C attenuator backend (default) --
        hw_configs = mod_cfg.get('hw_config', None)

        if _HW_AVAILABLE:
            try:
                bus = SMBus(self._i2c_bus_num)
                if hw_configs is not None:
                    cfg_list = [
                        SOUKRFMixerlessModuleChnHWConfig(**c)
                        for c in hw_configs
                    ]
                else:
                    # Use defaults – two channels, both MAX7329
                    cfg_list = [
                        SOUKRFMixerlessModuleChnHWConfig.default_config(),
                        SOUKRFMixerlessModuleChnHWConfig.default_config(),
                    ]
                self._hw_module = SOUKRFMixerlessModule(bus, cfg_list)
                logger.info(
                    'RF mixerless module initialised on I2C bus %d, channel %d',
                    self._i2c_bus_num, self._channel,
                )
                # Sync initial hardware state into config.
                self._sync_config_from_hardware()
            except Exception:
                logger.exception(
                    'Failed to initialise RF mixerless hardware – '
                    'falling back to software mimic'
                )
                self._hw_module = None

        if self._hw_module is None:
            self._mimic_module = mimicSOUKRFMixerlessModule()
            logger.info('Using software mimic for RF mixerless module')

    # -- public properties --

    @property
    def is_hardware(self):
        """True if controlling real hardware."""
        return self._hw_module is not None

    # -- TX attenuation --

    def set_tx_attenuation(self, attenuation_db):
        """Set TX variable attenuator (0–31.5 dB)."""
        self._set_attenuation('transmit_atten', attenuation_db)

    def get_tx_attenuation(self):
        """Get current TX variable attenuator setting in dB."""
        return self._get_attenuation('transmit_atten')

    # -- RX attenuation --

    def set_rx_attenuation(self, attenuation_db):
        """Set RX variable attenuator (0–31.5 dB)."""
        self._set_attenuation('recv_atten', attenuation_db)

    def get_rx_attenuation(self):
        """Get current RX variable attenuator setting in dB."""
        return self._get_attenuation('recv_atten')

    # -- TX amplifier bypass --

    def set_tx_amp_bypass(self, bypass):
        """Set TX amplifier bypass state (True = bypassed)."""
        self._set_amp_bypass('transmit_atten', bypass)

    def get_tx_amp_bypass(self):
        """Get TX amplifier bypass state."""
        return self._get_amp_bypass('transmit_atten')

    # -- RX amplifier bypass --

    def set_rx_amp_bypass(self, bypass):
        """Set RX amplifier bypass state (True = bypassed)."""
        self._set_amp_bypass('recv_atten', bypass)

    def get_rx_amp_bypass(self):
        """Get RX amplifier bypass state."""
        return self._get_amp_bypass('recv_atten')

    # -- transfer functions --

    def get_transfer(self):
        """Return (tx_transfer, rx_transfer) dataclasses for the current state."""
        mod = self._hw_module or self._mimic_module
        return mod.get_transfer(self._channel)

    def get_tx_input_1db_comp(self):
        """Return the TX path input 1 dB compression point (dBm) at the current settings."""
        mod = self._hw_module or self._mimic_module
        atten_amp = mod._get_atten_amp(self._channel, 'transmit_atten')
        return atten_amp.atten_amp_level.input_1dB_comp

    def get_rx_input_1db_comp(self):
        """Return the RX path input 1 dB compression point (dBm) at the current settings."""
        mod = self._hw_module or self._mimic_module
        atten_amp = mod._get_atten_amp(self._channel, 'recv_atten')
        return atten_amp.atten_amp_level.input_1dB_comp

    def get_tx_total_gain(self):
        """Return the TX path total gain/insertion-loss (dB) at the current settings."""
        mod = self._hw_module or self._mimic_module
        atten_amp = mod._get_atten_amp(self._channel, 'transmit_atten')
        return atten_amp.atten_amp_level.total_gain_il

    def get_rx_total_gain(self):
        """Return the RX path total gain/insertion-loss (dB) at the current settings."""
        mod = self._hw_module or self._mimic_module
        atten_amp = mod._get_atten_amp(self._channel, 'recv_atten')
        return atten_amp.atten_amp_level.total_gain_il

    # -- status --

    @property
    def attenuator_backend(self):
        """Name of the active attenuator backend."""
        if isinstance(self._hw_module, RudatAdapter):
            return 'rudat'
        elif self._hw_module is not None:
            return 'i2c'
        elif self._mimic_module is not None:
            return 'mimic'
        return 'none'

    def get_status(self):
        """Return a dict summarising the current peripheral state."""
        if not self.enabled:
            return {'enabled': False}
        return {
            'enabled': True,
            'hardware': self.is_hardware,
            'attenuator_backend': self.attenuator_backend,
            'channel': self._channel,
            'tx_attenuation_db': self.get_tx_attenuation(),
            'rx_attenuation_db': self.get_rx_attenuation(),
            'tx_amp_bypass': self.get_tx_amp_bypass(),
            'rx_amp_bypass': self.get_rx_amp_bypass(),
            'tx_total_gain_db': self.get_tx_total_gain(),
            'rx_total_gain_db': self.get_rx_total_gain(),
            'tx_input_1db_comp_dbm': self.get_tx_input_1db_comp(),
            'rx_input_1db_comp_dbm': self.get_rx_input_1db_comp(),
        }

    # -- config application --

    def apply_config(self, config_dict=None):
        """Apply peripheral-relevant config values to hardware.

        Counterpart to ``firmware_lib.apply_config`` — that function handles
        FPGA/firmware parameters, this one handles RF peripheral hardware
        (variable attenuators and amplifier bypass via I2C).

        Reads ``tx_attenuator_value_db``, ``rx_attenuator_value_db`` from
        rf_frontend and ``bypass_amps.tx_amp_bypass`` / ``rx_amp_bypass``
        and programs the hardware to match.
        Derived config values (``tx_bypass_amp_s21_db`` etc.) are updated after
        each set operation via the normal _sync_config path.
        """
        if not self.enabled:
            return

        cfg = config_dict if config_dict is not None else self.config
        rf_cfg = cfg.get('rf_frontend', {})
        bypass_cfg = rf_cfg.get('bypass_amps', {})

        # Set hardware attenuators to match config
        tx_atten = rf_cfg.get('tx_attenuator_value_db')
        if tx_atten is not None:
            try:
                self.set_tx_attenuation(float(tx_atten))
            except (ValueError, RuntimeError) as e:
                logger.warning('Could not apply TX attenuation from config: %s', e)

        rx_atten = rf_cfg.get('rx_attenuator_value_db')
        if rx_atten is not None:
            try:
                self.set_rx_attenuation(float(rx_atten))
            except (ValueError, RuntimeError) as e:
                logger.warning('Could not apply RX attenuation from config: %s', e)

        # Set amplifier bypass state to match config (if bypass_amps enabled)
        if bypass_cfg.get('enabled', False):
            tx_bypass = bypass_cfg.get('tx_amp_bypass')
            if tx_bypass is not None:
                try:
                    self.set_tx_amp_bypass(bool(tx_bypass))
                except (ValueError, RuntimeError) as e:
                    logger.warning('Could not apply TX amp bypass from config: %s', e)

            rx_bypass = bypass_cfg.get('rx_amp_bypass')
            if rx_bypass is not None:
                try:
                    self.set_rx_amp_bypass(bool(rx_bypass))
                except (ValueError, RuntimeError) as e:
                    logger.warning('Could not apply RX amp bypass from config: %s', e)

    # -- internal helpers --

    def _module(self):
        mod = self._hw_module or self._mimic_module
        if mod is None:
            raise RuntimeError('RF mixerless module not initialised')
        return mod

    def _set_attenuation(self, dev_name, attenuation_db):
        self._module().set_attenuation(self._channel, dev_name, attenuation_db)
        self._sync_config(dev_name)

    def _get_attenuation(self, dev_name):
        return self._module().get_attenuation_value(self._channel, dev_name)

    def _set_amp_bypass(self, dev_name, bypass):
        self._module().set_amp_bypass_state(self._channel, dev_name, bool(bypass))
        self._sync_config(dev_name)

    def _get_amp_bypass(self, dev_name):
        return self._module().get_amp_bypass_state(self._channel, dev_name)

    def _sync_config(self, dev_name):
        """Update the in-memory config to reflect the current peripheral state."""
        rf_cfg = self.config.setdefault('rf_frontend', {})
        bypass_cfg = rf_cfg.setdefault('bypass_amps', {})

        if dev_name == 'transmit_atten':
            rf_cfg['tx_attenuator_value_db'] = self.get_tx_attenuation()
            rf_cfg['tx_bypass_amp_s21_db'] = self._get_amp_s21('transmit_atten')
            bypass_cfg['tx_amp_bypass'] = self.get_tx_amp_bypass()
        elif dev_name == 'recv_atten':
            rf_cfg['rx_attenuator_value_db'] = self.get_rx_attenuation()
            rf_cfg['rx_bypass_amp_s21_db'] = self._get_amp_s21('recv_atten')
            bypass_cfg['rx_amp_bypass'] = self.get_rx_amp_bypass()

    def _sync_config_from_hardware(self):
        """Read all hardware state and update config on first init."""
        self._sync_config('transmit_atten')
        self._sync_config('recv_atten')

    def _get_amp_s21(self, dev_name):
        """Return the current amplifier S21 contribution (gain or bypass IL)."""
        mod = self._module()
        atten_amp = mod._get_atten_amp(self._channel, dev_name)
        return atten_amp.atten_amp_level.amp.total_gain_il
