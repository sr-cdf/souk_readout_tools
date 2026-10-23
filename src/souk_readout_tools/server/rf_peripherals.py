"""
Abstraction layer for RF peripheral hardware (attenuators, amplifier bypass).

The active RF frontend is identified by ``rf_frontend.hardware_id``. When the
SOUK mixerless module is the active frontend (``mixerless_module.connected:
true``), the controller drives it via i2c (smbus2) — that hardware provides
both i2c attenuators and bypassable amplifiers on the same MAX7329 chips.

Independent dimensions:

- ``rf_frontend.mixerless_module.connected`` — whether the SOUK mixerless
  module is the active RF frontend. Drives bypass-amp support and (when
  attenuator backend is ``i2c``) the attenuator role.
- ``rf_frontend.mixerless_module.model`` — which physical module is fitted
  (a key of ``MIXERLESS_MODULE_MODELS``): its i2c addresses and which paths
  have a bypass amp.
- ``rf_frontend.attenuator.backend`` — how programmable attenuators are
  controlled: ``i2c`` (uses the mixerless module's on-board attenuators),
  ``rudat`` (Mini-Circuits USB), or ``fixed`` (explicit non-controllable
  values from config).

Hardware init is soft-fail: if the underlying device doesn't respond, the
relevant slot stays ``None`` and a loud error is logged. Subsequent set/get
calls then raise ``RuntimeError`` so clients see a real error instead of a
silent fake-success.
"""

import logging
import os
import sys
from contextlib import contextmanager

from souk_readout_tools.server.i2c_lock import i2c_bus_lock

logger = logging.getLogger(__name__)

# Add the submodule to sys.path so its internal imports resolve.
_SUBMODULE_DIR = os.path.join(os.path.dirname(__file__), 'souk-peripherals-control')
if _SUBMODULE_DIR not in sys.path:
    sys.path.insert(0, _SUBMODULE_DIR)

# Software-only model classes (no smbus2 required).
from souk_rf_mixerless_atten_amp_level import (
    SOUKRFMixerlessTransmitAttenAmpLevel,
    SOUKRFMixerlessRecvAttenAmpLevel,
    SOUKRFMixerlessAttenAmpTransfer,
)

# Hardware imports — may fail on client machines.
try:
    from smbus2 import SMBus
    from souk_rf_mixerless_module import (
        SOUKRFMixerlessModule,
        SOUKRFMixerlessModuleChnHWConfig,
        GPIO_ADDR_RESISTOR_MAP,
    )
    from max732_8_9 import DEV_ADDRS_MAP
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False

# RUDAT USB attenuator support — optional.
# rudat.py must be on sys.path (e.g. pip install, or add its directory to
# PYTHONPATH / sys.path before starting the server).
try:
    from souk_readout_tools.server.rudat import Attenuator as RudatAttenuator, find_rudats
    _RUDAT_AVAILABLE = True
except ImportError:
    _RUDAT_AVAILABLE = False


# The SOUK mixerless modules we have, selected by
# rf_frontend.mixerless_module.model. Each channel, in rf_channel order, gives
# the i2c addresses of its receive and transmit GPIO expanders (what
# `i2cdetect -y -r 0 0x38 0x3f` shows) and the paths whose amplifier can be
# bypassed from software.
#
# A path with no bypass amp is not the circuit the submodule's component model
# describes: its amp is reported as never bypassed, the model's gain and P1dB
# are not used, and its S21 must be measured into
# rf_frontend.mixerless_module.{tx,rx}_amp_enabled_s21_db.
MIXERLESS_MODULE_MODELS = {
    'cardiff_dual': [
        {'recv': 0x3B, 'transmit': 0x3A, 'bypass_amps': ('tx', 'rx')},
        {'recv': 0x39, 'transmit': 0x38, 'bypass_amps': ('tx', 'rx')},
    ],
    'manchester_single': [
        {'recv': 0x3B, 'transmit': 0x3A, 'bypass_amps': ('tx', 'rx')},
    ],
    # New RX circuit, no RX bypass amp. One RF channel: its RX attenuator is
    # on 0x3D and its TX attenuator and TX bypass on 0x38, channel 1's
    # transmit expander (measured in loopback on krm4, 2026-09-23). 0x39 and
    # 0x3C also answer but are not in the RF path, and 0x3C must never be
    # driven: its "TX bypass" bit cuts the whole RX chain by ~47 dB.
    'rx_prototype': [
        {'recv': 0x3D, 'transmit': 0x38, 'bypass_amps': ('tx',)},
    ],
}
DEFAULT_MIXERLESS_MODULE_MODEL = 'cardiff_dual'


def _hw_config(channel):
    """The submodule's strap-resistor config for a channel at these addresses.

    The reverse of the lookup ``MAX732_8_9.__init__`` does: the address fixes
    the part and its ad0/ad1/ad2 levels in ``DEV_ADDRS_MAP``, and each level
    fixes which resistor of the pair is fitted in ``GPIO_ADDR_RESISTOR_MAP``.
    """
    straps = {}
    for role, names, part_field in (
        ('recv', ('r8_r13', 'r9_r14', 'r12_r17'), 'u4_type'),
        ('transmit', ('r18_r21', 'r19_r22', 'r20_r23'), 'u8_type'),
    ):
        (part, levels), = DEV_ADDRS_MAP[channel[role]].items()
        straps[part_field] = part
        for pin, name in zip(('ad0', 'ad1', 'ad2'), names):
            resistors = GPIO_ADDR_RESISTOR_MAP[role][pin]
            straps[name] = next(r for r, level in resistors.items()
                                if level == levels[pin])
    return SOUKRFMixerlessModuleChnHWConfig(**straps)


def _answers(bus, addr):
    try:
        bus.read_byte(addr)
        return True
    except OSError:
        return False


def _models_on_bus(bus):
    """The models whose expanders all answer, keeping only the fullest.

    A module can answer at addresses its model does not use (the prototype's
    0x39/0x3C), so this is not an exact match; and a dual module also answers
    at a single one's addresses, so of the matches only those with the most
    expanders are kept.
    """
    def addresses(model):
        return {ch[role] for ch in MIXERLESS_MODULE_MODELS[model]
                for role in ('recv', 'transmit')}
    known = set().union(*(addresses(m) for m in MIXERLESS_MODULE_MODELS))
    answering = {addr for addr in known if _answers(bus, addr)}
    matches = [m for m in MIXERLESS_MODULE_MODELS if addresses(m) <= answering]
    most = max((len(addresses(m)) for m in matches), default=0)
    return [m for m in matches if len(addresses(m)) == most]


def _answering_channels(bus, model):
    """The leading channels of ``model`` whose expanders answer on the bus.

    ``SOUKRFMixerlessModule.__init__`` raises on the first channel that does
    not answer, which would take the working channels down with it, so
    channels are probed first. The list stops at the first missing channel
    rather than skipping it: the submodule numbers channels by list position,
    so skipping one would move the next into its rf_channel and drive another
    pipeline's RF chain.
    """
    present = []
    for idx, channel in enumerate(MIXERLESS_MODULE_MODELS[model]):
        missing = [f'{role}@0x{channel[role]:02x}'
                   for role in ('recv', 'transmit')
                   if not _answers(bus, channel[role])]
        if missing:
            logger.warning(
                'SOUK mixerless module %r channel %d not answering (%s) -- '
                'using channels before it only. A missing channel on a '
                'module that has it is a hardware fault.',
                model, idx, ', '.join(missing),
            )
            break
        present.append(channel)
    if not present:
        matches = _models_on_bus(bus)
        hint = (f'the bus matches {matches}: set '
                f'rf_frontend.mixerless_module.model' if matches else
                'card absent, unpowered or on another bus')
        raise RuntimeError(
            f'no channel of SOUK mixerless module {model!r} answers on i2c '
            f'bus 0 -- {hint}')
    return present


# ---------------------------------------------------------------------
# RUDAT adapter — pure attenuator (no bypass amp).
# ---------------------------------------------------------------------

class RudatAdapter:
    """
    Adapts two RUDAT USB attenuators to the attenuator interface used by
    RFPeripheralController.

    RUDAT is a pure attenuator. Bypass amps are a property of the RF
    frontend (currently only the SOUK mixerless module has them) and are
    handled separately by the controller — not stubbed here.

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

    def set_attenuation(self, channel, dev_name, attenuation_db):
        """Set device ``dev_name`` on ``channel`` to ``attenuation_db`` (dB)."""
        self._attens[dev_name].att = float(attenuation_db)

    def get_attenuation_value(self, channel, dev_name):
        """Return the current attenuation (dB) of ``dev_name`` on ``channel``."""
        return float(self._attens[dev_name].att)

    def get_transfer(self, channel):
        """Return the transfer calibration for ``channel`` (``None`` for
        standalone attenuators)."""
        return None  # not applicable for standalone attenuators


class RFPeripheralController:
    """
    Unified interface for RF peripheral hardware.

    Owns two independent components:

    - ``_attenuator``: the active programmable-attenuator driver (the
      SOUK mixerless module when ``attenuator.backend == 'i2c'``, a
      ``RudatAdapter`` when ``backend == 'rudat'``, or ``None`` for fixed
      values / hardware init failures).
    - ``_mixerless_module``: the ``SOUKRFMixerlessModule`` instance, present
      iff ``mixerless_module.connected: true``. Drives bypass amps and
      (when the attenuator backend is i2c) the attenuator role.

    Both slots are soft-fail: if hardware init raises, the slot stays
    ``None``, the controller still reports ``enabled=True``, and subsequent
    set/get calls raise ``RuntimeError``.

    Parameters
    ----------
    config_dict : dict
        The desired server config. Runtime hardware state is kept separately
        in ``runtime_state`` and does not modify this dict.
    pipeline_id : int
        Pipeline index, used as the default rf_channel.
    """

    # -- attenuation limits (mixerless module hardware) --
    ATTEN_MIN = 0.0
    ATTEN_MAX = 31.5
    ATTEN_STEP = 0.5

    def __init__(self, config_dict, pipeline_id=0):
        self.config = config_dict
        self.pipeline_id = pipeline_id
        self._attenuator = None
        self._mixerless_module = None
        self._bypass_paths = ()
        self.runtime_state = {
            'attenuator': {},
            'bypass_amps': {},
        }

        rf_cfg        = config_dict.get('rf_frontend', {})
        mixerless_cfg = rf_cfg.get('mixerless_module', {}) or {}
        attn_cfg      = rf_cfg.get('attenuator', {}) or {}

        connected = rf_cfg.get('connected', False)
        self.hardware_id = rf_cfg.get('hardware_id')
        self.mixerless_model = (mixerless_cfg.get('model')
                                or DEFAULT_MIXERLESS_MODULE_MODEL)
        self._attenuator_backend = attn_cfg.get('backend') or None
        self.enabled = connected and self._attenuator_backend is not None
        self._channel = mixerless_cfg.get('rf_channel', pipeline_id)

        if not connected:
            logger.info('RF frontend not connected in config')
            return
        if self._attenuator_backend is None:
            logger.info('No programmable attenuator backend configured')
            return
        if self._attenuator_backend not in ('i2c', 'rudat', 'fixed'):
            raise ValueError(
                f"Unknown attenuator backend '{self._attenuator_backend}'. "
                f"Supported: 'i2c', 'rudat', 'fixed'"
            )

        mixerless_connected = bool(mixerless_cfg.get('connected', False))

        # 1. If the mixerless module is connected, bring it up. That single
        #    instance fills whichever roles it's wired to: attenuator (when
        #    attenuator.backend == 'i2c') and/or bypass amps.
        if mixerless_connected:
            channels = MIXERLESS_MODULE_MODELS.get(self.mixerless_model)
            if channels is None:
                raise ValueError(
                    f"Unknown mixerless_module.model '{self.mixerless_model}'. "
                    f"Known: {', '.join(MIXERLESS_MODULE_MODELS)}"
                )
            if not 0 <= self._channel < len(channels):
                raise ValueError(
                    f"rf_channel {self._channel} does not exist on mixerless "
                    f"module '{self.mixerless_model}' "
                    f"({len(channels)} channel(s))"
                )
            self._bypass_paths = channels[self._channel]['bypass_amps']
            for path in ('tx', 'rx'):
                if (path not in self._bypass_paths
                        and mixerless_cfg.get(f'{path}_amp_enabled_s21_db') is None):
                    logger.warning(
                        "Mixerless module %r has no %s bypass amp on rf_channel "
                        "%d, so its %s S21 is not modelled: set "
                        "rf_frontend.mixerless_module.%s_amp_enabled_s21_db "
                        "(and %s_input_1db_comp_dbm). Power calibration "
                        "assumes 0 dB until then.",
                        self.mixerless_model, path.upper(), self._channel,
                        path.upper(), path, path,
                    )

            if not _HW_AVAILABLE:
                logger.error(
                    "mixerless_module.connected: true but smbus2 / "
                    "souk-peripherals-control are not installed — "
                    "subsequent attenuator/bypass calls will fail."
                )
            else:
                try:
                    with i2c_bus_lock(purpose='RF module init'):
                        bus = SMBus(0)
                        present = _answering_channels(bus, self.mixerless_model)
                        if self._channel >= len(present):
                            raise RuntimeError(
                                f'rf_channel {self._channel} is not answering')
                        self._mixerless_module = SOUKRFMixerlessModule(
                            bus, [_hw_config(ch) for ch in present])
                    logger.info(
                        'SOUK mixerless module %r (%s) initialised, rf_channel %d',
                        self.hardware_id, self.mixerless_model, self._channel,
                    )
                except Exception:
                    logger.exception(
                        "Failed to initialise SOUK mixerless module %r — "
                        "subsequent attenuator/bypass calls will fail.",
                        self.hardware_id,
                    )

        # 2. Wire up the attenuator according to backend.
        if self._attenuator_backend == 'fixed':
            logger.info('Using fixed attenuator values from config')

        elif self._attenuator_backend == 'i2c':
            if not mixerless_connected:
                logger.warning(
                    "attenuator.backend == 'i2c' requires "
                    "mixerless_module.connected: true; ignoring."
                )
            self._attenuator = self._mixerless_module  # may be None on init failure

        elif self._attenuator_backend == 'rudat':
            if not _RUDAT_AVAILABLE:
                raise ImportError(
                    'RUDAT backend selected but rudat module not found. '
                    'Install pyusb and ensure rudat.py is on sys.path.'
                )
            tx_serial = attn_cfg.get('rudat_tx_serial')
            rx_serial = attn_cfg.get('rudat_rx_serial')
            if tx_serial is None or rx_serial is None:
                raise ValueError(
                    'RUDAT backend requires attenuator.rudat_tx_serial and '
                    'attenuator.rudat_rx_serial in rf_frontend config'
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
            self._attenuator = RudatAdapter(tx_att, rx_att)
            self.ATTEN_MIN = max(tx_att.att_min, rx_att.att_min)
            self.ATTEN_MAX = min(tx_att.att_max, rx_att.att_max)
            self.ATTEN_STEP = max(tx_att.resolution, rx_att.resolution)
            logger.info(
                'RUDAT attenuators initialised: TX serial %s, RX serial %s',
                tx_serial, rx_serial,
            )

        # Sync initial state from hardware into runtime_state (only updates
        # fields for components that actually came up).
        self._sync_runtime_state_from_hardware()

    # -- public properties --

    @property
    def is_hardware(self):
        """True when the configured attenuator backend is responding."""
        return self._attenuator is not None

    @property
    def is_controllable(self):
        """True when attenuation can be changed by software."""
        return self._attenuator is not None

    @property
    def supports_bypass_amps(self):
        """True iff the SOUK mixerless module is connected and responding and
        this channel has a bypass amp on at least one path."""
        return self._mixerless_module is not None and bool(self._bypass_paths)

    def has_bypass_amp(self, path):
        """True iff the ``path`` ('tx' or 'rx') amp bypass on this channel is
        software-controllable."""
        return self._mixerless_module is not None and path in self._bypass_paths

    @property
    def attenuator_backend(self):
        """Name of the active attenuator backend."""
        return self._attenuator_backend or 'none'

    # -- TX/RX attenuation --

    def set_tx_attenuation(self, attenuation_db):
        """Set the TX variable attenuator to ``attenuation_db`` (0-31.5 dB)."""
        self._set_attenuation('transmit_atten', attenuation_db)

    def get_tx_attenuation(self):
        """Get current TX variable attenuator setting in dB."""
        return self._get_attenuation('transmit_atten')

    def set_rx_attenuation(self, attenuation_db):
        """Set the RX variable attenuator to ``attenuation_db`` (0-31.5 dB)."""
        self._set_attenuation('recv_atten', attenuation_db)

    def get_rx_attenuation(self):
        """Get current RX variable attenuator setting in dB."""
        return self._get_attenuation('recv_atten')

    # -- TX/RX amplifier bypass --

    @contextmanager
    def _i2c_lock(self):
        """Hold the shared I2C bus lock while touching the mixerless module.

        The LNA bias board sits on the same bus on the RFSoC that is wired to
        it, so this keeps RF traffic from interleaving with the LNA service's
        multi-step mux operations. A no-op for the RUDAT and fixed backends,
        which never touch I2C.
        """
        if self._mixerless_module is None:
            yield
            return
        with i2c_bus_lock(purpose='RF peripherals'):
            yield

    def set_tx_amp_bypass(self, bypass):
        """Set TX amplifier bypass state (True = bypassed)."""
        self._require_bypass_amp('tx')
        with self._i2c_lock():
            self._mixerless_module.set_amp_bypass_state(
                self._channel, 'transmit_atten', bool(bypass),
            )
        self._sync_runtime_state('transmit_atten')

    def get_tx_amp_bypass(self):
        """Get TX amplifier bypass state (False on a path with no bypass amp)."""
        if self._mixerless_module is not None and not self.has_bypass_amp('tx'):
            return False
        self._require_bypass_amp('tx')
        with self._i2c_lock():
            return self._mixerless_module.get_amp_bypass_state(
                self._channel, 'transmit_atten',
            )

    def set_rx_amp_bypass(self, bypass):
        """Set RX amplifier bypass state (True = bypassed)."""
        self._require_bypass_amp('rx')
        with self._i2c_lock():
            self._mixerless_module.set_amp_bypass_state(
                self._channel, 'recv_atten', bool(bypass),
            )
        self._sync_runtime_state('recv_atten')

    def get_rx_amp_bypass(self):
        """Get RX amplifier bypass state (False on a path with no bypass amp)."""
        if self._mixerless_module is not None and not self.has_bypass_amp('rx'):
            return False
        self._require_bypass_amp('rx')
        with self._i2c_lock():
            return self._mixerless_module.get_amp_bypass_state(
                self._channel, 'recv_atten',
            )

    # -- transfer functions / gain / 1 dB compression --

    def get_transfer(self):
        """Return (tx_transfer, rx_transfer) for the current state.

        ``None`` for backends that don't model a full transfer function (RUDAT).
        """
        if self._mixerless_module is not None:
            return self._mixerless_module.get_transfer(self._channel)
        return None

    def get_tx_total_gain(self):
        """TX path total gain/insertion-loss (dB) at the current settings."""
        if self._attenuator_backend == 'fixed':
            gain = -abs(self.get_tx_attenuation())
            if self.has_bypass_amp('tx'):
                gain += self._get_amp_s21('transmit_atten')
            return gain
        if self._mixerless_module is not None:
            if not self.has_bypass_amp('tx'):
                return None  # not the circuit the submodule models
            atten_amp = self._mixerless_module._get_atten_amp(
                self._channel, 'transmit_atten',
            )
            return atten_amp.atten_amp_level.total_gain_il
        return -abs(self.get_tx_attenuation())

    def get_rx_total_gain(self):
        """RX path total gain/insertion-loss (dB) at the current settings."""
        if self._attenuator_backend == 'fixed':
            gain = -abs(self.get_rx_attenuation())
            if self.has_bypass_amp('rx'):
                gain += self._get_amp_s21('recv_atten')
            return gain
        if self._mixerless_module is not None:
            if not self.has_bypass_amp('rx'):
                return None  # not the circuit the submodule models
            atten_amp = self._mixerless_module._get_atten_amp(
                self._channel, 'recv_atten',
            )
            return atten_amp.atten_amp_level.total_gain_il
        return -abs(self.get_rx_attenuation())

    def get_tx_input_1db_comp(self):
        """TX path input 1 dB compression point (dBm) at the current settings."""
        override = self._mixerless_module_cal_value('tx_input_1db_comp_dbm')
        if override is not None:
            return float(override)
        if self.has_bypass_amp('tx'):
            atten_amp = self._mixerless_module._get_atten_amp(
                self._channel, 'transmit_atten',
            )
            return self._corrected_tx_input_1db_comp(
                atten_amp.atten_amp_level)
        return None

    def get_rx_input_1db_comp(self):
        """RX path input 1 dB compression point (dBm) at the current settings."""
        override = self._mixerless_module_cal_value('rx_input_1db_comp_dbm')
        if override is not None and self.mixerless_model == 'rx_prototype':
            # The prototype's override is measured at 0 dB RX attenuation, and
            # its attenuator sits ahead of the stage that compresses -- -38 dBm
            # total in at 31.5 dB stayed linear to 0.01 dB (krm4, 2026-09-23) --
            # so the input-referred P1dB rises with the attenuation.
            return float(override) + float(self.get_rx_attenuation())
        if override is not None:
            return float(override)
        if self.has_bypass_amp('rx'):
            atten_amp = self._mixerless_module._get_atten_amp(
                self._channel, 'recv_atten',
            )
            return self._corrected_rx_input_1db_comp(
                atten_amp.atten_amp_level)
        return None

    # -- status --

    def get_status(self):
        """Return a dict summarising the current peripheral state."""
        if not self.enabled:
            return {'enabled': False}
        has_readable_attenuation = (
            self.is_hardware or self._attenuator_backend == 'fixed'
        )
        status = {
            'enabled': True,
            'hardware': self.is_hardware,
            'controllable': self.is_controllable,
            'attenuator_backend': self.attenuator_backend,
            'supports_bypass_amps': self.supports_bypass_amps,
            'rf_channel': self._channel,
            'tx_attenuation_db': (
                self.get_tx_attenuation() if has_readable_attenuation else None
            ),
            'rx_attenuation_db': (
                self.get_rx_attenuation() if has_readable_attenuation else None
            ),
            'tx_total_gain_db': (
                self.get_tx_total_gain() if has_readable_attenuation else None
            ),
            'rx_total_gain_db': (
                self.get_rx_total_gain() if has_readable_attenuation else None
            ),
            'tx_input_1db_comp_dbm': self.get_tx_input_1db_comp() if self.is_hardware else None,
            'rx_input_1db_comp_dbm': self.get_rx_input_1db_comp() if self.is_hardware else None,
        }
        if self._mixerless_module is not None:
            status['tx_amp_bypass'] = self.get_tx_amp_bypass()
            status['rx_amp_bypass'] = self.get_rx_amp_bypass()
            status['tx_bypass_amp_s21_db'] = self._get_amp_s21('transmit_atten')
            status['rx_bypass_amp_s21_db'] = self._get_amp_s21('recv_atten')
        if self._attenuator_backend == 'rudat':
            attn_cfg = self.config.get('rf_frontend', {}).get('attenuator', {})
            status['rudat_tx_serial'] = attn_cfg.get('rudat_tx_serial')
            status['rudat_rx_serial'] = attn_cfg.get('rudat_rx_serial')
        return status

    # -- config application --

    def apply_config(self, config_dict=None):
        """Apply peripheral-relevant config values to hardware.

        Counterpart to ``firmware_lib.apply_config`` — that function handles
        FPGA/firmware parameters, this one handles RF peripheral hardware
        (variable attenuators and amplifier bypass).

        Reads ``attenuator.tx_value_db`` / ``attenuator.rx_value_db`` and
        ``bypass_amps.tx_amp_bypass`` / ``bypass_amps.rx_amp_bypass`` and
        programs the hardware to match.

        ``config_dict`` is the config to apply; ``None`` (default) uses this
        controller's stored ``self.config``.
        """
        if not self.enabled:
            return

        cfg = config_dict if config_dict is not None else self.config
        rf_cfg     = cfg.get('rf_frontend', {})
        attn_cfg   = rf_cfg.get('attenuator', {}) or {}
        bypass_cfg = rf_cfg.get('bypass_amps', {}) or {}

        if self._attenuator_backend == 'fixed':
            for key in ('tx_value_db', 'rx_value_db'):
                if key in attn_cfg:
                    self.runtime_state['attenuator'][key] = attn_cfg[key]
        else:
            tx_atten = attn_cfg.get('tx_value_db')
            if tx_atten is not None:
                try:
                    self.set_tx_attenuation(float(tx_atten))
                except (ValueError, RuntimeError) as e:
                    logger.warning('Could not apply TX attenuation from config: %s', e)

            rx_atten = attn_cfg.get('rx_value_db')
            if rx_atten is not None:
                try:
                    self.set_rx_attenuation(float(rx_atten))
                except (ValueError, RuntimeError) as e:
                    logger.warning('Could not apply RX attenuation from config: %s', e)

        if bypass_cfg.get('enabled', False):
            if not self.supports_bypass_amps:
                logger.warning(
                    'bypass_amps.enabled: true but no SOUK mixerless module '
                    'is connected — ignoring bypass-amp settings.'
                )
            else:
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

    def _require_attenuator(self):
        if self._attenuator is None:
            if self._attenuator_backend == 'fixed':
                raise RuntimeError(
                    'Fixed attenuator backend is not controllable.'
                )
            if self._attenuator_backend is None:
                raise RuntimeError(
                    'No attenuator backend configured: set '
                    'rf_frontend.attenuator.backend (and, for i2c, '
                    'rf_frontend.mixerless_module.connected: true).'
                )
            if (self._attenuator_backend == 'i2c'
                    and not (self.config.get('rf_frontend', {})
                             .get('mixerless_module') or {}).get('connected')):
                raise RuntimeError(
                    "attenuator.backend is 'i2c' but "
                    'rf_frontend.mixerless_module.connected is not true.'
                )
            raise RuntimeError(
                'Attenuator not initialised — check server logs for hardware errors.'
            )
        return self._attenuator

    def _require_bypass_amp(self, path):
        if self._mixerless_module is None:
            raise RuntimeError(
                'Bypass amps not available: the SOUK mixerless module is not '
                'connected or did not initialise.'
            )
        if path not in self._bypass_paths:
            raise RuntimeError(
                f'Mixerless module {self.mixerless_model!r} has no '
                f'{path.upper()} bypass amp on rf_channel {self._channel}.'
            )

    def _set_attenuation(self, dev_name, attenuation_db):
        attenuator = self._require_attenuator()
        with self._i2c_lock():
            attenuator.set_attenuation(self._channel, dev_name, attenuation_db)
        self._sync_runtime_state(dev_name)

    def _get_attenuation(self, dev_name):
        if self._attenuator_backend == 'fixed':
            key = 'tx_value_db' if dev_name == 'transmit_atten' else 'rx_value_db'
            value = self.runtime_state.get('attenuator', {}).get(key)
            if value is None:
                value = (
                    self.config.get('rf_frontend', {})
                    .get('attenuator', {})
                    .get(key)
                )
            return 0.0 if value is None else float(value)
        attenuator = self._require_attenuator()
        with self._i2c_lock():
            return attenuator.get_attenuation_value(self._channel, dev_name)

    def _sync_runtime_state(self, dev_name):
        """Update runtime_state to reflect the current peripheral state."""
        attn_state = self.runtime_state.setdefault('attenuator', {})
        bypass_state = self.runtime_state.setdefault('bypass_amps', {})

        if self._attenuator_backend == 'fixed':
            attn_cfg = self.config.get('rf_frontend', {}).get('attenuator', {})
            key = 'tx_value_db' if dev_name == 'transmit_atten' else 'rx_value_db'
            if key in attn_cfg:
                attn_state[key] = attn_cfg[key]

        if dev_name == 'transmit_atten':
            if self.is_hardware:
                attn_state['tx_value_db'] = self.get_tx_attenuation()
            if self._mixerless_module is not None:
                bypass_state['tx_amp_bypass'] = self.get_tx_amp_bypass()
        elif dev_name == 'recv_atten':
            if self.is_hardware:
                attn_state['rx_value_db'] = self.get_rx_attenuation()
            if self._mixerless_module is not None:
                bypass_state['rx_amp_bypass'] = self.get_rx_amp_bypass()

    def _sync_runtime_state_from_hardware(self):
        """Read all hardware state and update runtime_state on first init."""
        self._sync_runtime_state('transmit_atten')
        self._sync_runtime_state('recv_atten')

    def _get_amp_s21(self, dev_name):
        """Return the module S21 in its current bypass state, excluding the
        variable attenuator.

        This is the plane the calibration chain expects: ``calc_tone_powers``
        applies ``tx_attenuator_value_db`` as a separate term, so the value
        here must cover everything else in the module (amp/preamp, filter,
        equaliser, fixed attenuator) but not the variable attenuator.

        The amplifier's own ``total_gain_il`` is *not* the right quantity --
        using it drops the module's fixed loss and makes reported tone powers
        ~16 dB optimistic on TX (and ~5 dB pessimistic on RX).  The config
        overrides ``{tx,rx}_amp_{enabled,bypassed}_s21_db`` are measured at
        this same plane and take priority over this model.

        ``None`` on a path with no bypass amp, which is not the circuit the
        model describes.
        """
        if not self.has_bypass_amp('tx' if dev_name == 'transmit_atten' else 'rx'):
            return None
        level = self._mixerless_module._get_atten_amp(
            self._channel, dev_name).atten_amp_level
        return level.total_gain_il - level.variable_atten.total_gain_il

    def _mixerless_module_cal_value(self, key):
        """Return a scalar mixerless-module calibration override if set."""
        value = (
            self.config.get('rf_frontend', {})
            .get('mixerless_module', {})
            .get(key)
        )
        return None if value is None else value

    @staticmethod
    def _amp_input_1db_comp(amp):
        """Input P1dB for the amplifier without relying on submodule math."""
        if amp.bypass_state:
            return amp._bypass_1dB_comp
        return amp._gain_1dB_comp

    @classmethod
    def _corrected_tx_input_1db_comp(cls, level):
        """Input-referred TX P1dB using wrapper-side component arithmetic."""
        amp_comp = cls._amp_input_1db_comp(level.amp)
        return min(
            level.variable_atten.input_1dB_comp,
            level.filter.input_1dB_comp
            - level.variable_atten.total_gain_il,
            level.equalizer.input_1dB_comp
            - level.variable_atten.total_gain_il
            - level.filter.total_gain_il,
            amp_comp
            - level.equalizer.total_gain_il
            - level.filter.total_gain_il
            - level.variable_atten.total_gain_il,
            level.fixed_atten.input_1dB_comp
            - level.amp.total_gain_il
            - level.equalizer.total_gain_il
            - level.filter.total_gain_il
            - level.variable_atten.total_gain_il,
        )

    @classmethod
    def _corrected_rx_input_1db_comp(cls, level):
        """Input-referred RX P1dB using wrapper-side component arithmetic."""
        amp_comp = cls._amp_input_1db_comp(level.amp)
        preamp_comp = cls._amp_input_1db_comp(level.preamp)
        return min(
            amp_comp
            - level.fixed_atten.total_gain_il
            - level.variable_atten.total_gain_il
            - level.preamp.total_gain_il
            - level.filter.total_gain_il
            - level.equalizer.total_gain_il,
            preamp_comp
            - level.equalizer.total_gain_il
            - level.filter.total_gain_il,
        )

    def get_runtime_state(self):
        """Return a copy of mutable RF frontend state."""
        self._sync_runtime_state_from_hardware()
        return {
            'attenuator': dict(self.runtime_state.get('attenuator', {})),
            'bypass_amps': dict(self.runtime_state.get('bypass_amps', {})),
        }


# ---------------------------------------------------------------------
# Generic attenuator discovery
# ---------------------------------------------------------------------

def _detected_module(bus):
    """``(model, channels, SOUKRFMixerlessModule)`` for the module on ``bus``.

    Discovery has no config to name the model, so it is read off the bus;
    ``(None, [], None)`` when no single model matches what answers.
    """
    matches = _models_on_bus(bus)
    if len(matches) != 1:
        print(f'No single SOUK mixerless module model matches i2c bus 0 '
              f'(matches: {matches or "none"}).')
        return None, [], None
    channels = MIXERLESS_MODULE_MODELS[matches[0]]
    return (matches[0], channels,
            SOUKRFMixerlessModule(bus, [_hw_config(ch) for ch in channels]))


def find_attenuators(include_state=False):
    """Discover all connected programmable attenuators.

    Searches for:
    - Mini-Circuits RUDAT USB attenuators (via pyusb)
    - I2C attenuators on the SOUK mixerless module (via smbus2) — both
      TX and RX paths on each of the module's channels.

    If ``include_state=True``, each entry also reports the current
    attenuation level in dB (key: ``attenuation_db``).

    Returns a list of dicts. Common keys: backend, model, bus, address.
    Mixerless-module entries also include channel and path ('TX'/'RX').
    """
    results = []

    # -- RUDAT USB attenuators --
    if _RUDAT_AVAILABLE:
        try:
            rudats = find_rudats()
            for serial, info in rudats.items():
                from souk_readout_tools.server.rudat import Attenuator as _Att
                att = _Att(info['bus'], info['address'])
                entry = {
                    'backend': 'rudat',
                    'serial': serial,
                    'bus': info['bus'],
                    'address': info['address'],
                    'model': att.get_model(),
                }
                if include_state:
                    try:
                        entry['attenuation_db'] = float(att.att)
                    except Exception:
                        entry['attenuation_db'] = None
                results.append(entry)
        except Exception as e:
            print(f'RUDAT discovery error: {e}')
    else:
        print('RUDAT support not available (pyusb not installed)')

    # -- I2C attenuators (SOUK mixerless module on SMBus(0)) --
    if _HW_AVAILABLE:
        try:
            with i2c_bus_lock(purpose='attenuator discovery'):
                bus = SMBus(0)
                model, channels, mod = _detected_module(bus)
                # Each channel has two independent MAX7329-driven attenuators —
                # one on the TX path and one on the RX path. Probe both.
                for ch in range(len(channels)):
                    for path, label in (('transmit_atten', 'TX'),
                                        ('recv_atten', 'RX')):
                        try:
                            atten_amp = mod._get_atten_amp(ch, path)
                            entry = {
                                'backend': 'i2c',
                                'serial': None,
                                'bus': 0,
                                'address': hex(atten_amp.atten_amp.addr),
                                'channel': ch,
                                'path': label,
                                'model': f'SOUK RF Mixerless Module ({model}) ch{ch} {label}',
                            }
                            if include_state:
                                try:
                                    entry['attenuation_db'] = mod.get_attenuation_value(ch, path)
                                except Exception:
                                    entry['attenuation_db'] = None
                            results.append(entry)
                        except Exception:
                            pass
                bus.close()
        except Exception as e:
            print(f'I2C mixerless-module discovery error: {e}')
    else:
        print('I2C support not available (smbus2 not installed)')

    return results


def find_bypass_amps(include_state=False):
    """Discover bypassable amplifiers.

    Currently only the SOUK mixerless module has bypass amps — up to one per
    TX/RX path per channel (see ``MIXERLESS_MODULE_MODELS``), sharing the
    MAX7329 GPIO with the attenuator.

    If ``include_state=True``, each entry reports the current bypass
    state (key: ``bypassed`` — True means the amp is bypassed).

    Returns a list of dicts. Keys: backend, model, bus, address, channel, path.
    """
    results = []

    if _HW_AVAILABLE:
        try:
            with i2c_bus_lock(purpose='bypass-amp discovery'):
                bus = SMBus(0)
                model, channels, mod = _detected_module(bus)
                for ch, channel in enumerate(channels):
                    for path, label in (('transmit_atten', 'TX'),
                                        ('recv_atten', 'RX')):
                        if label.lower() not in channel['bypass_amps']:
                            continue
                        try:
                            atten_amp = mod._get_atten_amp(ch, path)
                            entry = {
                                'backend': 'i2c',
                                'bus': 0,
                                'address': hex(atten_amp.atten_amp.addr),
                                'channel': ch,
                                'path': label,
                                'model': f'SOUK RF Mixerless Module ({model}) ch{ch} {label} bypass amp',
                            }
                            if include_state:
                                try:
                                    entry['bypassed'] = mod.get_amp_bypass_state(ch, path)
                                except Exception:
                                    entry['bypassed'] = None
                            results.append(entry)
                        except Exception:
                            pass
                bus.close()
        except Exception as e:
            print(f'Bypass-amp discovery error: {e}')
    else:
        print('I2C support not available (smbus2 not installed)')

    return results


def _format_value(val):
    if val is None:
        return 'unknown'
    if isinstance(val, bool):
        return str(val)
    if isinstance(val, float):
        return f'{val:.3f}'
    return str(val)


def _print_results(results, kind):
    """Pretty-print a list of discovery dicts."""
    if not results:
        print(f'No {kind}s found.')
        return
    print(f'Found {len(results)} {kind}(s):\n')
    state_keys = (
        'attenuation_db', 'bypassed',
        'remote_voltage_v', 'local_voltage_v', 'bias_current_a',
    )
    for r in results:
        print(f'  Backend: {r["backend"]}')
        if r.get('serial'):
            print(f'  Serial:  {r["serial"]}')
        if r.get('model'):
            print(f'  Model:   {r["model"]}')
        if r.get('refdes'):
            print(f'  Refdes:  {r["refdes"]}')
        if r.get('channel') is not None:
            extra = f' ({r["path"]})' if r.get('path') else ''
            print(f'  Channel: {r["channel"]}{extra}')
        bus = r.get('bus')
        addr = r.get('address')
        if bus is not None:
            line = f'  Bus:     {bus}'
            if addr is not None:
                line += f', Address: {addr}'
            print(line)
        for key in state_keys:
            if key in r:
                print(f'  {key}: {_format_value(r[key])}')
        print()


def _cli_main():
    """CLI entry point for souk-find-attenuators."""
    import argparse
    parser = argparse.ArgumentParser(description='Discover programmable attenuators.')
    parser.add_argument('--status', action='store_true',
                        help='Also report the current attenuation level on each.')
    args = parser.parse_args()

    print('\nSearching for programmable attenuators...\n')
    _print_results(find_attenuators(include_state=args.status), 'attenuator')


def _cli_main_bypass_amps():
    """CLI entry point for souk-find-bypass-amps."""
    import argparse
    parser = argparse.ArgumentParser(description='Discover bypass amplifiers.')
    parser.add_argument('--status', action='store_true',
                        help='Also report the current bypass state on each.')
    args = parser.parse_args()

    print('\nSearching for bypass amplifiers...\n')
    _print_results(find_bypass_amps(include_state=args.status), 'bypass amp')


if __name__ == '__main__':
    _cli_main()
