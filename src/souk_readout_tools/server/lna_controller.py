"""
LNA bias control and monitoring for the SOUK cryostat.

Wraps the souk-peripherals-control ``SOUKLNABiasControlMonitor`` to provide
a simplified interface for setting and reading LNA bias voltages and
currents.  Supports all 14 LNA channels in the cryostat.

Each readout pipeline maps to one LNA channel via ``cryostat.lna_bias.lna_channel``
in the config.  Any LNA channel (1-14) can also be addressed directly.

The ``i2c`` backend controls hardware via ``smbus2`` and the
``souk-peripherals-control`` submodule on the server.  The ``fixed`` backend
records an explicit, non-controllable bias value in config.
"""

import logging
import math
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
        REFDES_LNA_MONITOR_CHN_MAP,
    )
    from lna_monitor import LNAMonitorHWConfig
    from ad511_0_2_4bcpz_5_10_80 import AD511_0_2_4BCPZ_5_10_80HWConfig
    from ltc2481cdd import LTC2481CDDHWConfig
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False
    REFDES_LNA_MONITOR_CHN_MAP = {}


class _I2CRetryWarningFilter(logging.Filter):
    """Drop the ``... trying again after delay ...`` retry-warnings emitted
    by souk-peripherals-control's i2c_devices module when the underlying
    OSError is ``No such device or address`` — i.e. an unpopulated LNA
    slot. Real I2C transients with other root causes still surface."""

    _SUPPRESS_PATTERNS = ('trying again after delay', 'No such device or address')

    def filter(self, record):
        msg = record.getMessage()
        return not all(p in msg for p in self._SUPPRESS_PATTERNS)


if _HW_AVAILABLE:
    _root_logger = logging.getLogger()
    if not any(isinstance(f, _I2CRetryWarningFilter) for f in _root_logger.filters):
        _root_logger.addFilter(_I2CRetryWarningFilter())


NUM_LNA_CHANNELS = 14


# Substrings in the upstream message that mean "the requested voltage was
# NOT achieved".  An empty message, or a message about the lowest local
# voltage already exceeding the target, is treated as a successful set.
_LNA_FAILURE_HINTS = (
    'Cannot set remote voltage',
    'Reached maximum local voltage',
    'LNA monitor not configured',
)


def _lna_set_succeeded(message):
    """Interpret the upstream set-voltage message string.

    The submodule returns an empty string on full success and a sentence
    describing the failure mode otherwise.  We treat anything matching a
    known failure hint as a failure, and the empty / "lowest local voltage
    already exceeds" cases as success.
    """
    if not message:
        return True
    return not any(hint in message for hint in _LNA_FAILURE_HINTS)


def _finite_voltage(value):
    """True if a hardware result is a real finite voltage."""
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def _local_lna_result(chn, voltage):
    success = _finite_voltage(voltage)
    return {
        'channel': chn,
        'voltage_v': voltage,
        'method': 'local',
        'success': success,
        'message': '' if success else 'LNA monitor not configured.',
    }


def _out_of_local_range_result(chn, voltage_v, method, vmin, vmax):
    """Result dict for a v_local request outside the LDO's achievable range."""
    return {
        'channel': chn,
        'voltage_v': float('nan'),
        'method': method,
        'message': (
            f'Requested local voltage {voltage_v:.3f} V is outside the '
            f'achievable range for channel {chn} '
            f'({vmin:.3f}-{vmax:.3f} V).'
        ),
        'success': False,
    }


def _soft_off_unavailable_result(chn):
    return {
        'channel': chn,
        'voltage_v': float('nan'),
        'method': 'local',
        'soft_off': False,
        'message': (
            f'Cannot soft-off LNA channel {chn}: local voltage range is '
            'unavailable because the LNA monitor is not configured.'
        ),
        'success': False,
    }


def _soft_off_message(chn, voltage_v):
    return (
        f'LNA channel {chn} driven to minimum local voltage '
        f'({voltage_v:.3f} V). This is a soft off only; the LNA rail is '
        'not fully disabled.'
    )


# A populated, biased LNA draws several mA. A reading of <0.5 mA together
# with a remote voltage near mid-rail (~Vref/2) is the signature of an
# unconnected LNA on the channel.
_OPEN_CIRCUIT_BIAS_CURRENT_A = 0.5e-3
_OPEN_CIRCUIT_V_REMOTE_V = 2.0


def _open_circuit_message(chn, current_a, remote_v):
    """Open-circuit hint string when readings match the unconnected-LNA
    signature; empty string otherwise."""
    try:
        i = float(current_a)
        v = float(remote_v)
    except (TypeError, ValueError):
        return ''
    if not (math.isfinite(i) and math.isfinite(v)):
        return ''
    if i < _OPEN_CIRCUIT_BIAS_CURRENT_A and v > _OPEN_CIRCUIT_V_REMOTE_V:
        return (
            f'LNA on channel {chn} appears to be disconnected '
            f'(bias current {i * 1e3:.3f} mA, '
            f'remote voltage {v:.3f} V — open-circuit signature).'
        )
    return ''


_LNA_REFDES = [f"M{i}" for i in range(17, 31)]
_LNA_BOARD_RESISTORS = dict(
    r9_r12="R12", r8_r10="R10", r7_r5="R7",
    r11_r13="R13", r14_r15="R14", r6_r4="R6",
)


def _default_lna_monitor_hw_config():
    """Per-channel LNA monitor hw_config used by every populated LNA slot.

    Identical for every channel — matches the production board.
    """
    return LNAMonitorHWConfig(
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


def _detect_lna_channels(bus, per_channel_cfg):
    """Probe each LNA refdes (M17-M30) and return the list that responds.

    For each candidate channel, build a single-channel
    SOUKLNABiasControlMonitorHWConfig and try to instantiate it.  If
    construction raises (typically ConnectionError because no LNAMonitor
    hardware is wired to that mux path), the channel is treated as
    unpopulated and skipped.
    """
    detected = []
    for ref in _LNA_REFDES:
        single = {r: None for r in _LNA_REFDES}
        single[ref] = per_channel_cfg
        probe_cfg = SOUKLNABiasControlMonitorHWConfig(
            lna_monitor_hw_configs=single,
            **_LNA_BOARD_RESISTORS,
        )
        try:
            SOUKLNABiasControlMonitor(bus, probe_cfg)
            detected.append(ref)
        except ConnectionError:
            pass
    return detected


def _build_lna_hw_config(detected_refdes, per_channel_cfg):
    """Build a SOUKLNABiasControlMonitorHWConfig populated with only the
    given refdes (others left None)."""
    lna_configs = {ref: None for ref in _LNA_REFDES}
    for ref in detected_refdes:
        lna_configs[ref] = per_channel_cfg
    return SOUKLNABiasControlMonitorHWConfig(
        lna_monitor_hw_configs=lna_configs,
        **_LNA_BOARD_RESISTORS,
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
        The desired server config. Runtime LNA state is kept separately
        in ``runtime_state`` and does not modify this dict.
    pipeline_id : int
        Pipeline index.
    """

    SUPPORTED_BACKENDS = ('i2c', 'fixed')
    CONTROLLABLE_BACKENDS = ('i2c',)
    DEFAULT_BIAS_VOLTAGE_V = 1.5
    DEFAULT_METHOD = 'local'
    DEFAULT_BLIND = False
    DEFAULT_SOFT_OFF = False

    def __init__(self, config_dict, pipeline_id=0):
        self.config = config_dict
        self.pipeline_id = pipeline_id
        self._monitor = None
        self.runtime_state = {'lna_bias': {}}
        # Per-channel last-observed health state for transition-only logging.
        self._last_lna_state = {}

        cryo_cfg = config_dict.get('cryostat', {})
        lna_cfg = cryo_cfg.get('lna_bias', {})

        # cryostat.connected gates RF calibration only. LNA bias has its own
        # enable flag so the bias board can be used during bench/loopback
        # setups where the cryostat RF path is intentionally disconnected.
        self.cryostat_connected = bool(cryo_cfg.get('connected', False))
        config_enabled = lna_cfg.get('enabled', False)
        self._backend = lna_cfg.get('backend', 'i2c') or None
        self.enabled = bool(config_enabled) and self._backend is not None
        if not self.enabled:
            if config_enabled:
                logger.info('No LNA bias backend configured')
            else:
                logger.info('LNA bias control disabled in config')
            return
        if self._backend not in self.SUPPORTED_BACKENDS:
            raise ValueError(
                f"Unknown LNA bias backend '{self._backend}'. "
                f"Supported: {', '.join(self.SUPPORTED_BACKENDS)}"
            )

        self._lna_channel = lna_cfg.get('lna_channel', 1)
        self._i2c_bus_num = (lna_cfg.get('i2c', {}) or {}).get('bus', 0)

        if self._backend == 'fixed':
            if lna_cfg.get('soft_off', self.DEFAULT_SOFT_OFF):
                logger.warning(
                    'LNA soft_off requested but fixed backend is not controllable'
                )
            self._sync_fixed_state(
                self._configured_bias_voltage(lna_cfg),
                method='fixed',
                blind=False,
                soft_off=False,
            )
            logger.info('Using fixed LNA bias value from config')
            return

        if not _HW_AVAILABLE:
            logger.error(
                'LNA bias backend i2c enabled in config but smbus2 / '
                'souk-peripherals-control are not installed — '
                'subsequent LNA calls will fail.'
            )
            return

        try:
            bus = SMBus(self._i2c_bus_num)
            per_channel_cfg = _default_lna_monitor_hw_config()
            detected = _detect_lna_channels(bus, per_channel_cfg)
            if not detected:
                logger.error(
                    'No LNA channels responded on i2c bus %d — '
                    'subsequent LNA calls will fail.',
                    self._i2c_bus_num,
                )
                return
            hw_config = _build_lna_hw_config(detected, per_channel_cfg)
            self._monitor = SOUKLNABiasControlMonitor(bus, hw_config)
            self._detected_refdes = detected
            logger.info(
                'LNA bias controller initialised on backend %s, bus %d: '
                '%d/%d channels detected (%s); '
                'pipeline LNA channel %d',
                self._backend, self._i2c_bus_num,
                len(detected), len(_LNA_REFDES), ', '.join(detected),
                self._lna_channel,
            )
        except Exception:
            logger.exception(
                'Failed to initialise LNA bias hardware — '
                'subsequent LNA calls will fail.'
            )

    # -- properties --

    @property
    def is_hardware(self):
        """True if controlling real hardware."""
        return self._monitor is not None

    @property
    def is_controllable(self):
        """True when LNA bias can be changed by software."""
        return self._backend in self.CONTROLLABLE_BACKENDS and self._monitor is not None

    @property
    def backend(self):
        """Name of the active LNA bias backend."""
        return self._backend or 'none'

    @property
    def lna_channel(self):
        """The default LNA channel index (1-14) for this pipeline."""
        return self._lna_channel

    # -- config application --

    def apply_config(self, config_dict=None):
        """Apply LNA-bias settings from config to hardware.

        If ``cryostat.lna_bias.soft_off`` is true, drives this pipeline's
        configured LNA channel to the minimum achievable local voltage.
        Otherwise reads ``bias_voltage_v`` and sets the channel to that
        voltage. Missing/None voltage defaults to 1.5 V.
        """
        if not self.enabled:
            return None

        cfg = config_dict if config_dict is not None else self.config
        lna_cfg = cfg.get('cryostat', {}).get('lna_bias', {}) or {}

        voltage_v = lna_cfg.get('bias_voltage_v', self.DEFAULT_BIAS_VOLTAGE_V)
        if voltage_v is None:
            voltage_v = self.DEFAULT_BIAS_VOLTAGE_V
        method = lna_cfg.get('method', self.DEFAULT_METHOD) or self.DEFAULT_METHOD
        blind = bool(lna_cfg.get('blind', self.DEFAULT_BLIND))
        soft_off = bool(lna_cfg.get('soft_off', self.DEFAULT_SOFT_OFF))
        channel = int(lna_cfg.get('lna_channel', self._lna_channel))
        self._validate_channel(channel)
        self._lna_channel = channel

        if method not in ('remote', 'local'):
            method = self.DEFAULT_METHOD

        if self._backend == 'fixed':
            self._sync_fixed_state(
                float(voltage_v), method='fixed', blind=False, soft_off=False,
            )
            if soft_off:
                message = (
                    'Fixed LNA bias backend is not controllable; soft_off '
                    'requires backend: i2c.'
                )
                logger.warning('Could not apply LNA soft_off from config: %s', message)
                return {
                    'channel': int(channel),
                    'voltage_v': float(voltage_v),
                    'method': 'fixed',
                    'soft_off': False,
                    'message': message,
                    'success': False,
                }
            return {
                'channel': int(channel),
                'voltage_v': float(voltage_v),
                'method': 'fixed',
                'soft_off': False,
                'message': 'Fixed LNA bias backend is not controllable.',
                'success': True,
            }

        try:
            if soft_off:
                result = self.soft_off_lna_bias(channel=int(channel))
            else:
                result = self.set_lna_bias_voltage(
                    float(voltage_v), channel=int(channel),
                    method=method, blind=blind,
                )
        except (ValueError, RuntimeError) as e:
            logger.warning('Could not apply LNA bias from config: %s', e)
            return None

        if not result.get('success', True):
            logger.warning(
                'Could not apply LNA bias from config: %s',
                result.get('message', 'LNA bias set failed'),
            )
        return result

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
        if method not in ('remote', 'local'):
            raise ValueError("method must be 'remote' or 'local'")

        if method == 'local':
            vmin, vmax = self._monitor.lna_local_voltage_ranges.get(
                chn, (float('nan'), float('nan'))
            )
            if (math.isfinite(vmin) and math.isfinite(vmax)
                    and not (vmin - 1e-6 <= voltage_v <= vmax + 1e-6)):
                return _out_of_local_range_result(chn, voltage_v, 'local', vmin, vmax)
            result = self._monitor.set_lna_bias_local(chn=[chn], v_local=voltage_v)
            out = _local_lna_result(chn, result[chn])
        else:
            result = self._monitor.set_lna_bias_remote(
                chn=[chn], v_local=voltage_v, blind=blind,
            )
            message = result[chn][1]
            success = _lna_set_succeeded(message)
            if not success and 'Cannot set remote voltage' in message:
                message = self._augment_with_open_circuit_hint(chn, message)
            out = {
                'channel': chn,
                'voltage_v': result[chn][0],
                'method': 'remote',
                'message': message,
                'success': success,
            }
        if out.get('success', True):
            self._sync_runtime_state_from_result(out, blind=blind)
        return out

    def soft_off_lna_bias(self, channel=None):
        """Drive one LNA channel to its minimum achievable local voltage.

        This is a soft off only. The LNA bias board does not expose a hard
        shutdown/enable pin, so this leaves the rail at the LDO's minimum
        local-voltage setting rather than fully removing power.
        """
        self._require_hardware()
        chn = channel if channel is not None else self._lna_channel
        self._validate_channel(chn)

        vmin = self._minimum_local_voltage(chn)
        if not _finite_voltage(vmin):
            return _soft_off_unavailable_result(chn)

        result = self._monitor.set_lna_bias_local(chn=[chn], v_local=vmin)
        out = _local_lna_result(chn, result[chn])
        out['soft_off'] = out.get('success', True)
        if out.get('success', True):
            out['message'] = _soft_off_message(chn, out['voltage_v'])
            self._sync_runtime_state_from_result(out, blind=False, soft_off=True)
        return out

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
        if method not in ('remote', 'local'):
            raise ValueError("method must be 'remote' or 'local'")
        channels = list(range(1, NUM_LNA_CHANNELS + 1))

        if method == 'local':
            ranges = self._monitor.lna_local_voltage_ranges
            out = {}
            in_range = []
            for chn in channels:
                vmin, vmax = ranges.get(chn, (float('nan'), float('nan')))
                if (math.isfinite(vmin) and math.isfinite(vmax)
                        and not (vmin - 1e-6 <= voltage_v <= vmax + 1e-6)):
                    out[chn] = _out_of_local_range_result(
                        chn, voltage_v, 'local', vmin, vmax,
                    )
                else:
                    in_range.append(chn)
            if in_range:
                result = self._monitor.set_lna_bias_local(
                    chn=in_range, v_local=voltage_v,
                )
                for chn in in_range:
                    out[chn] = _local_lna_result(chn, result[chn])
            ordered = {chn: out[chn] for chn in channels}
            default_result = ordered.get(self._lna_channel)
            if default_result and default_result.get('success', True):
                self._sync_runtime_state_from_result(
                    default_result, blind=blind, soft_off=False,
                )
            return ordered
        else:
            result = self._monitor.set_lna_bias_remote(
                chn=channels, v_local=voltage_v, blind=blind,
            )
            out = {}
            for chn in channels:
                message = result[chn][1]
                success = _lna_set_succeeded(message)
                if not success and 'Cannot set remote voltage' in message:
                    message = self._augment_with_open_circuit_hint(chn, message)
                out[chn] = {
                    'channel': chn,
                    'voltage_v': result[chn][0],
                    'method': 'remote',
                    'message': message,
                    'success': success,
                }
            default_result = out.get(self._lna_channel)
            if default_result and default_result.get('success', True):
                self._sync_runtime_state_from_result(
                    default_result, blind=blind, soft_off=False,
                )
            return out

    def soft_off_lna_bias_all(self):
        """Drive all LNA channels to their minimum local voltage.

        Returns per-channel results keyed by channel index. Unconfigured
        channels are reported as failures in the same shape as set-all calls.
        """
        self._require_hardware()
        return {
            chn: self.soft_off_lna_bias(channel=chn)
            for chn in range(1, NUM_LNA_CHANNELS + 1)
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
        chn = channel if channel is not None else self._lna_channel
        self._validate_channel(chn)

        if self._backend == 'fixed':
            return self._fixed_lna_status(chn)

        self._require_hardware()

        status = self._monitor.read_lna_status(chn=[chn])
        s = status[chn]
        message = self._check_status_health(
            chn, s['remote voltage'], s['bias current'],
        )
        return {
            'channel': chn,
            'remote_voltage_v': s['remote voltage'],
            'local_voltage_v': s['local voltage'],
            'bias_current_a': s['bias current'],
            'message': message,
        }

    def get_lna_bias_status_all(self):
        """
        Read bias voltage and current for all 14 LNA channels.

        Returns
        -------
        dict
            Per-channel status keyed by channel index (1-14).
        """
        channels = list(range(1, NUM_LNA_CHANNELS + 1))

        if self._backend == 'fixed':
            return {chn: self._fixed_lna_status(chn) for chn in channels}

        self._require_hardware()
        status = self._monitor.read_lna_status(chn=channels)
        return {
            chn: {
                'channel': chn,
                'remote_voltage_v': status[chn]['remote voltage'],
                'local_voltage_v': status[chn]['local voltage'],
                'bias_current_a': status[chn]['bias current'],
                'message': self._check_status_health(
                    chn,
                    status[chn]['remote voltage'],
                    status[chn]['bias current'],
                ),
            }
            for chn in channels
        }

    # -- status summary --

    def get_status(self):
        """Return a summary dict of the controller state."""
        if not self.enabled:
            return {
                'enabled': False,
                'cryostat_connected': self.cryostat_connected,
            }
        return {
            'enabled': True,
            'cryostat_connected': self.cryostat_connected,
            'hardware': self.is_hardware,
            'controllable': self.is_controllable,
            'backend': self.backend,
            'lna_channel': self._lna_channel,
            'bias_voltage_v': self._runtime_bias_voltage(),
            'soft_off': self._runtime_soft_off(),
            'method': self.runtime_state.get('lna_bias', {}).get(
                'method',
                self.config.get('cryostat', {}).get('lna_bias', {}).get(
                    'method', self.DEFAULT_METHOD,
                ),
            ),
            'blind': self.runtime_state.get('lna_bias', {}).get(
                'blind',
                self.config.get('cryostat', {}).get('lna_bias', {}).get(
                    'blind', self.DEFAULT_BLIND,
                ),
            ),
        }

    # -- internal --

    def _configured_bias_voltage(self, lna_cfg=None):
        if lna_cfg is None:
            lna_cfg = self.config.get('cryostat', {}).get('lna_bias', {}) or {}
        value = lna_cfg.get('bias_voltage_v', self.DEFAULT_BIAS_VOLTAGE_V)
        return self.DEFAULT_BIAS_VOLTAGE_V if value is None else float(value)

    def _configured_soft_off(self, lna_cfg=None):
        if lna_cfg is None:
            lna_cfg = self.config.get('cryostat', {}).get('lna_bias', {}) or {}
        return bool(lna_cfg.get('soft_off', self.DEFAULT_SOFT_OFF))

    def _runtime_bias_voltage(self):
        value = self.runtime_state.get('lna_bias', {}).get('bias_voltage_v')
        return self._configured_bias_voltage() if value is None else float(value)

    def _runtime_soft_off(self):
        value = self.runtime_state.get('lna_bias', {}).get('soft_off')
        return self._configured_soft_off() if value is None else bool(value)

    def _sync_fixed_state(self, voltage_v, method='fixed', blind=False,
                          soft_off=False):
        lna_state = self.runtime_state.setdefault('lna_bias', {})
        lna_state['backend'] = self.backend
        lna_state['lna_channel'] = self._lna_channel
        lna_state['bias_voltage_v'] = voltage_v
        lna_state['method'] = method
        lna_state['blind'] = bool(blind)
        lna_state['soft_off'] = bool(soft_off)

    def _fixed_lna_status(self, channel):
        return {
            'channel': channel,
            'remote_voltage_v': (
                self._runtime_bias_voltage()
                if channel == self._lna_channel else None
            ),
            'local_voltage_v': None,
            'bias_current_a': None,
            'message': '',
        }

    def _sync_runtime_state_from_result(self, result, blind=False, soft_off=False):
        lna_state = self.runtime_state.setdefault('lna_bias', {})
        lna_state['backend'] = self.backend
        if result['channel'] != self._lna_channel:
            return
        lna_state['lna_channel'] = self._lna_channel
        lna_state['bias_voltage_v'] = result['voltage_v']
        lna_state['method'] = result.get('method', self.DEFAULT_METHOD)
        lna_state['soft_off'] = bool(soft_off)
        if result.get('method') == 'remote':
            lna_state['blind'] = bool(blind)

    def get_runtime_state(self):
        """Return a copy of mutable LNA state."""
        return {'lna_bias': dict(self.runtime_state.get('lna_bias', {}))}

    def _minimum_local_voltage(self, chn):
        ranges = self._monitor.lna_local_voltage_ranges
        vmin, _ = ranges.get(chn, (float('nan'), float('nan')))
        return vmin

    def _validate_channel(self, channel):
        if not (1 <= channel <= NUM_LNA_CHANNELS):
            raise ValueError(
                f'LNA channel must be 1-{NUM_LNA_CHANNELS}, got {channel}'
            )

    def _require_hardware(self):
        if self._monitor is None:
            if self._backend == 'fixed':
                raise RuntimeError('Fixed LNA bias backend is not controllable')
            raise RuntimeError('LNA bias hardware not initialised')

    def _augment_with_open_circuit_hint(self, chn, message):
        """Append an open-circuit hint when bias current is near zero and
        v_remote is sitting at the unconnected mid-rail voltage."""
        try:
            s = self._monitor.read_lna_status(chn=[chn])[chn]
        except Exception:
            return message
        hint = _open_circuit_message(chn, s['bias current'], s['remote voltage'])
        return f'{message} {hint}' if hint else message

    def _check_status_health(self, chn, remote_v, current_a):
        """Classify a channel's readings and log only on state transitions.

        Returns the message string to surface in the status response. State
        is one of 'ok', 'open', 'unconfigured'."""
        try:
            i = float(current_a) if current_a is not None else float('nan')
            v = float(remote_v) if remote_v is not None else float('nan')
        except (TypeError, ValueError):
            i, v = float('nan'), float('nan')

        if not (math.isfinite(i) and math.isfinite(v)):
            state = 'unconfigured'
            message = f'LNA monitor for channel {chn} is not configured.'
        elif i < _OPEN_CIRCUIT_BIAS_CURRENT_A and v > _OPEN_CIRCUIT_V_REMOTE_V:
            state = 'open'
            message = _open_circuit_message(chn, i, v)
        else:
            state = 'ok'
            message = ''

        prev = self._last_lna_state.get(chn)
        if state != prev:
            self._last_lna_state[chn] = state
            if state == 'open':
                logger.warning(message)
            elif state == 'unconfigured' and prev is not None:
                # Don't bark on the first read of an unpopulated channel —
                # only when a previously-healthy channel goes silent.
                logger.warning(message)
            elif state == 'ok' and prev in ('open', 'unconfigured'):
                logger.info(
                    'LNA channel %d now reads healthy '
                    '(bias current %.3f mA, remote voltage %.3f V).',
                    chn, i * 1e3, v,
                )
        return message


# ---------------------------------------------------------------------
# LNA discovery
# ---------------------------------------------------------------------

def _refdes_to_channel(refdes):
    """Map an LNA refdes ('M17'..'M30') to its LNA channel number (1..14)."""
    entry = REFDES_LNA_MONITOR_CHN_MAP.get(refdes, {})
    if not entry:
        return None
    return next(iter(entry.keys()))


def find_lnas(include_state=False):
    """Discover populated LNA bias monitors on the SOUK LNA bias board.

    Probes each candidate refdes (M17-M30) on SMBus(0); only refdes whose
    hardware responds are returned. If ``include_state=True``, each entry
    also reports remote/local voltage and bias current.

    Returns a list of dicts. Keys: backend, model, bus, refdes, channel.
    """
    if not _HW_AVAILABLE:
        print('I2C support not available (smbus2 not installed)')
        return []

    try:
        bus = SMBus(0)
    except Exception as e:
        print(f'LNA discovery error opening SMBus(0): {e}')
        return []

    per_channel_cfg = _default_lna_monitor_hw_config()
    detected = _detect_lna_channels(bus, per_channel_cfg)
    if not detected:
        return []

    monitor = None
    if include_state:
        try:
            hw_config = _build_lna_hw_config(detected, per_channel_cfg)
            monitor = SOUKLNABiasControlMonitor(bus, hw_config)
        except Exception as e:
            print(f'LNA status read setup failed: {e}')
            monitor = None

    results = []
    for refdes in detected:
        chn = _refdes_to_channel(refdes)
        entry = {
            'backend': 'i2c',
            'bus': 0,
            'refdes': refdes,
            'channel': chn,
            'model': f'SOUK LNA bias monitor {refdes}',
        }
        if include_state and monitor is not None and chn is not None:
            try:
                status = monitor.read_lna_status(chn=[chn])[chn]
                entry['remote_voltage_v'] = float(status['remote voltage'])
                entry['local_voltage_v'] = float(status['local voltage'])
                entry['bias_current_a'] = float(status['bias current'])
            except Exception:
                entry['remote_voltage_v'] = None
                entry['local_voltage_v'] = None
                entry['bias_current_a'] = None
        results.append(entry)
    return results


def _cli_main():
    """CLI entry point for souk-find-lnas."""
    import argparse
    # Import the shared printer from rf_peripherals to keep output consistent.
    from souk_readout_tools.server.rf_peripherals import _print_results

    parser = argparse.ArgumentParser(description='Discover LNA bias monitors.')
    parser.add_argument('--status', action='store_true',
                        help='Also report remote/local voltage and bias '
                             'current on each detected channel.')
    args = parser.parse_args()

    print('\nSearching for LNA bias monitors...\n')
    _print_results(find_lnas(include_state=args.status), 'LNA')


if __name__ == '__main__':
    _cli_main()
