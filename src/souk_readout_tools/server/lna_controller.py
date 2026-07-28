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

import json
import logging
import math
import os
import socket
import struct
import sys
import time

from souk_readout_tools.config_utils import (
    get_site_config_path,
    resolve_lna_service_endpoint,
)
from souk_readout_tools.server.i2c_lock import i2c_bus_lock as _i2c_bus_lock

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
        REFDES_OE_CHN_MAP,
        OE_ADDR_RESISTOR_MAP,
        SWITCH_ADDR_RESISTOR_MAP,
        OE_PIN_I2C_SWITCH_RESET,
        ROOT_LEAF_CONN,
        AWAIT_TURN_ON_DELAY,
    )
    from lna_monitor import LNAMonitor, LNAMonitorHWConfig
    from ad511_0_2_4bcpz_5_10_80 import AD511_0_2_4BCPZ_5_10_80HWConfig
    from ltc2481cdd import LTC2481CDDHWConfig
    from max732_8_9 import MAX732_8_9
    from tca9548 import TCA9548
    _HW_AVAILABLE = True
except ImportError:
    _HW_AVAILABLE = False
    REFDES_LNA_MONITOR_CHN_MAP = {}
    REFDES_OE_CHN_MAP = {}


class _I2CRetryWarningFilter(logging.Filter):
    """Drop the retry/failure log noise emitted by souk-peripherals-control's
    i2c_devices module when the underlying OSError is ``No such device or
    address`` — i.e. an unpopulated LNA slot. Real I2C transients with other
    root causes still surface.

    Matches both the per-attempt ``... retrying after delay ...`` warnings and
    the final ``Failed ... after N attempts`` error.
    """

    _ABSENT_DEVICE = 'No such device or address'
    _RETRY_MARKERS = ('retrying after delay', 'attempts:')

    def filter(self, record):
        """Logging filter: drop ``record`` if it matches the suppressed
        retry/noise patterns, else keep it."""
        msg = record.getMessage()
        if self._ABSENT_DEVICE not in msg:
            return True
        return not any(marker in msg for marker in self._RETRY_MARKERS)


if _HW_AVAILABLE:
    _root_logger = logging.getLogger()
    if not any(isinstance(f, _I2CRetryWarningFilter) for f in _root_logger.filters):
        _root_logger.addFilter(_I2CRetryWarningFilter())


NUM_LNA_CHANNELS = 14
LNA_I2C_BUS_NUM = 0


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


def _output_enabled_value(raw):
    """Normalise the driver's ``output enable`` reading to True/False/None.

    v1 boards have no output-enable switch, and the driver reports NaN for
    them; None says "not applicable" rather than implying "off".
    """
    if isinstance(raw, bool):
        return raw
    if raw is None:
        return None
    try:
        if math.isnan(float(raw)):
            return None
    except (TypeError, ValueError):
        return None
    return bool(raw)


def _output_enable_unsupported_result(chn, hw_version):
    return {
        'channel': chn,
        'output_enabled': None,
        'message': (
            f'LNA bias board hw v{hw_version} has no per-channel output '
            'enable; use soft_off to drive the channel to its minimum '
            'local voltage instead.'
        ),
        'success': False,
    }


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

SUPPORTED_HW_VERSIONS = (1, 2)

# I2C address-select resistor fits for the mux tree, per board revision.
# v2 moved two of the address straps (r7_r5 and r11_r13).
_LNA_BOARD_RESISTORS = {
    1: dict(r9_r12="R12", r8_r10="R10", r7_r5="R7",
            r11_r13="R13", r14_r15="R14", r6_r4="R6"),
    2: dict(r9_r12="R12", r8_r10="R10", r7_r5="R5",
            r11_r13="R11", r14_r15="R14", r6_r4="R6"),
}

# Output-enable expander (U6/U7) address straps. These parts only exist on v2
# boards, but the upstream hw_config dataclass requires the fields on both
# revisions, so v1 carries them unused.
_LNA_OE_RESISTORS = dict(
    r28_r30="R28", r29_r31="R29", r32_r33="R32",
    r34_r36="R36", r35_r37="R35", r38_r39="R38",
    u6_dev_type="MAX7329", u7_dev_type="MAX7329",
)


def _default_lna_monitor_hw_config(hw_version=1):
    """Per-channel LNA monitor hw_config used by every populated LNA slot.

    Identical for every channel on a given board revision. The v1 values are
    those of our production board and deliberately differ from the submodule's
    own ``LNAMonitorHWConfig.default_config('v1')``, which describes a
    different build.
    """
    if int(hw_version) == 2:
        # v2 drops the switched divider leg entirely (switch_status=False) and
        # swaps the remote ADC address straps.
        return LNAMonitorHWConfig(
            r_dac_hw_config=AD511_0_2_4BCPZ_5_10_80HWConfig(
                DEV_ADDR=0x2C, RESOLUTION=128, R_FULL_SCALE_KOHM=10.0,
            ),
            remote_adc_hw_config=LTC2481CDDHWConfig(CA0="float", CA1="low"),
            imonitor_adc_hw_config=LTC2481CDDHWConfig(CA0="float", CA1="float"),
            switch_status=False,
            r_LDO_set_kOhm=150.0,
            r_RTop1_kOhm=4.7,
            r_RBot1_kOhm=8.2,
            r_RAdj1_kOhm=200.0,
            r_RSENSE_OHMS=10.0,
        )
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


def _build_lna_hw_config(hw_version=1, refdes=None):
    """Build a SOUKLNABiasControlMonitorHWConfig for ``hw_version``.

    ``refdes`` restricts the populated slots to that iterable; the default
    (None) populates all of M17-M30 and lets the driver discover which ones
    actually answer.
    """
    per_channel_cfg = _default_lna_monitor_hw_config(hw_version)
    wanted = _LNA_REFDES if refdes is None else list(refdes)
    lna_configs = {
        ref: (per_channel_cfg if ref in wanted else None)
        for ref in _LNA_REFDES
    }
    return SOUKLNABiasControlMonitorHWConfig(
        lna_monitor_hw_configs=lna_configs,
        **_LNA_BOARD_RESISTORS[int(hw_version)],
        **_LNA_OE_RESISTORS,
    )


def _detect_hw_version(bus):
    """Return the LNA bias board revision present on ``bus`` (1 or 2).

    The v2 board carries two MAX7329 output-enable expanders (U6/U7) that the
    v1 board does not, so a successful probe of U6 identifies v2. On a v1
    board the probe costs one full I2C retry cycle (a couple of seconds); its
    log noise is suppressed by _I2CRetryWarningFilter.
    """
    try:
        MAX732_8_9(
            dev_name="oe_probe",
            i2c_bus=bus,
            ad2=OE_ADDR_RESISTOR_MAP["U6"]["ad2"][_LNA_OE_RESISTORS['r32_r33']],
            ad1=OE_ADDR_RESISTOR_MAP["U6"]["ad1"][_LNA_OE_RESISTORS['r29_r31']],
            ad0=OE_ADDR_RESISTOR_MAP["U6"]["ad0"][_LNA_OE_RESISTORS['r28_r30']],
            dev_type=_LNA_OE_RESISTORS['u6_dev_type'],
        )
    except OSError:
        return 1
    return 2


class _BiasMonitor(SOUKLNABiasControlMonitor):
    """LNA bias board driver covering both v1 and v2 hardware.

    Mirrors the upstream constructor with two deliberate differences:

    * The U6/U7 output-enable expanders only exist on v2, so they are built
      only there and every output-enable operation is a no-op on v1.
    * Neither revision powers the LNAs off on construction. Upstream disables
      all outputs in ``__init__``; here that would cut bias to every LNA in
      the cryostat every time the service restarts.

    Multi-channel output-enable operations are also batched into one
    read-modify-write per expander instead of one per channel.
    """

    def __init__(self, i2c_bus, hw_config, hw_version=1):
        self.hw_version = int(hw_version)
        if self.hw_version == 2:
            self._oe_u6 = MAX732_8_9(
                dev_name="oe_u6",
                i2c_bus=i2c_bus,
                ad2=OE_ADDR_RESISTOR_MAP["U6"]["ad2"][hw_config.r32_r33],
                ad1=OE_ADDR_RESISTOR_MAP["U6"]["ad1"][hw_config.r29_r31],
                ad0=OE_ADDR_RESISTOR_MAP["U6"]["ad0"][hw_config.r28_r30],
                dev_type=hw_config.u6_dev_type,
            )
            self._oe_u7 = MAX732_8_9(
                dev_name="oe_u7",
                i2c_bus=i2c_bus,
                ad2=OE_ADDR_RESISTOR_MAP["U7"]["ad2"][hw_config.r38_r39],
                ad1=OE_ADDR_RESISTOR_MAP["U7"]["ad1"][hw_config.r35_r37],
                ad0=OE_ADDR_RESISTOR_MAP["U7"]["ad0"][hw_config.r34_r36],
                dev_type=hw_config.u7_dev_type,
            )
            # U6 also drives the mux reset line; pulse it so a wedged switch
            # tree recovers on startup.
            self._reset_channel_switch()
        else:
            self._oe_u6 = None
            self._oe_u7 = None

        self._root_switch = TCA9548(
            dev_name="root_switch",
            i2c_bus=i2c_bus,
            a0=SWITCH_ADDR_RESISTOR_MAP["root"]["A0"][hw_config.r8_r10],
            a1=SWITCH_ADDR_RESISTOR_MAP["root"]["A1"][hw_config.r7_r5],
            a2=SWITCH_ADDR_RESISTOR_MAP["root"]["A2"][hw_config.r6_r4],
        )
        self._root_switch.turn_off_channel()
        self._root_switch.turn_on_channel(ROOT_LEAF_CONN)
        self._leaf_switch = TCA9548(
            dev_name="leaf_switch",
            i2c_bus=i2c_bus,
            a0=SWITCH_ADDR_RESISTOR_MAP["leaf"]["A0"][hw_config.r14_r15],
            a1=SWITCH_ADDR_RESISTOR_MAP["leaf"]["A1"][hw_config.r11_r13],
            a2=SWITCH_ADDR_RESISTOR_MAP["leaf"]["A2"][hw_config.r9_r12],
        )
        self._leaf_switch.turn_off_channel()

        self._lna_monitors = {}
        for refdes, lna_hw_config in hw_config.lna_monitor_hw_configs.items():
            if lna_hw_config is None:
                self._lna_monitors[refdes] = None
                continue
            chn = list(REFDES_LNA_MONITOR_CHN_MAP[refdes].keys())[0]
            self._turn_on_channel(chn)
            try:
                self._lna_monitors[refdes] = LNAMonitor(
                    i2c_bus=i2c_bus, hw_config=lna_hw_config,
                )
            except OSError:
                # Unpopulated slot, or a monitor that has stopped answering.
                self._lna_monitors[refdes] = None
            finally:
                self._turn_off_all_channels()
        self._hw_config = hw_config

    @property
    def supports_output_enable(self):
        """True when the board has per-channel hard output-enable switches."""
        return self.hw_version == 2

    @property
    def detected_refdes(self):
        """Refdes (M17-M30) whose monitors answered during construction."""
        return [ref for ref, mon in self._lna_monitors.items() if mon is not None]

    @property
    def bias_oe_status(self):
        """Output-enable state as ``{channel: bool}``; empty dict on v1.

        Reads each expander once rather than once per channel.
        """
        if not self.supports_output_enable:
            return {}
        bytes_by_dev = {
            'U6': self._oe_u6.read_gpio(),
            'U7': self._oe_u7.read_gpio(),
        }
        return {
            chn: bool(bytes_by_dev[dev_name] & (1 << bit))
            for chn, (dev_name, bit) in REFDES_OE_CHN_MAP.items()
        }

    def _reset_channel_switch(self):
        """Pulse the mux reset line (v2 only)."""
        if self.supports_output_enable:
            self._oe_u6.pulse_gpio_bit(OE_PIN_I2C_SWITCH_RESET, polarity=False)

    def set_lna_bias_output(self, chn, enabled):
        """Set the hard output-enable state for one or more channels.

        No-op on v1 hardware, which has no output-enable switches. Waits for
        the LNA rail to settle only when a channel actually turns on.
        """
        if not self.supports_output_enable:
            return
        channels = [chn] if isinstance(chn, int) else list(chn)
        enabled = bool(enabled)
        turning_on = enabled and any(
            not state
            for c, state in self.bias_oe_status.items() if c in channels
        )
        grouped = {}
        for c in channels:
            dev_name, bit = REFDES_OE_CHN_MAP[c]
            grouped.setdefault(dev_name, []).append(bit)
        for dev_name, bits in grouped.items():
            dev = self._oe_u6 if dev_name == 'U6' else self._oe_u7
            dev.set_gpio_bit(bits, [enabled] * len(bits))
        if turning_on:
            time.sleep(AWAIT_TURN_ON_DELAY)

    def enable_lna_bias_output(self, chn):
        """Enable the bias output for one or more channels (v2 only)."""
        self.set_lna_bias_output(chn, True)

    def disable_lna_bias_output(self, chn):
        """Disable the bias output for one or more channels (v2 only)."""
        self.set_lna_bias_output(chn, False)

    def enable_all_lna_bias_outputs(self):
        """Enable the bias output for every channel (v2 only)."""
        self.set_lna_bias_output(list(REFDES_OE_CHN_MAP), True)

    def disable_all_lna_bias_outputs(self):
        """Disable the bias output for every channel (v2 only)."""
        self.set_lna_bias_output(list(REFDES_OE_CHN_MAP), False)


def open_bias_monitor(bus, hw_version='auto', refdes=None):
    """Open the LNA bias board on ``bus`` and return ``(monitor, hw_version)``.

    ``hw_version`` may be 1, 2, or ``'auto'`` (probe for the v2 output-enable
    expanders). ``refdes`` optionally restricts which slots are probed; the
    default probes all of M17-M30, and slots that do not answer are marked
    unpopulated by the driver.

    Probing an absent slot costs a full I2C retry cycle (a couple of seconds
    each), so on a sparsely populated board this can take tens of seconds.
    That cost is why the bias board is opened once by a long-lived service
    rather than on every readout-server start.
    """
    if hw_version in (None, 'auto'):
        hw_version = _detect_hw_version(bus)
    hw_version = int(hw_version)
    if hw_version not in SUPPORTED_HW_VERSIONS:
        raise ValueError(
            f'Unsupported LNA bias board hw_version {hw_version}; '
            f'supported: {SUPPORTED_HW_VERSIONS}'
        )
    hw_config = _build_lna_hw_config(hw_version, refdes=refdes)
    return _BiasMonitor(bus, hw_config, hw_version), hw_version


def _recv_exactly(sock, count):
    """Read exactly ``count`` bytes from ``sock`` or raise ConnectionError."""
    buf = bytearray(count)
    view = memoryview(buf)
    got = 0
    while got < count:
        n = sock.recv_into(view[got:], count - got)
        if n == 0:
            raise ConnectionError(
                f'LNA service closed the connection after {got}/{count} bytes'
            )
        got += n
    return bytes(buf)


class _LNAServiceClient:
    """Request client for the LNA bias service.

    Speaks the same 4-byte-length-prefixed JSON framing as the readout
    server, and opens a fresh connection per request so a restarted service
    never leaves a stale socket behind.
    """

    def __init__(self, host, port, timeout_s):
        self.host = host
        self.port = int(port)
        self.timeout_s = float(timeout_s)

    @property
    def endpoint(self):
        """``host:port`` of the service, for logging and status."""
        return f'{self.host}:{self.port}'

    def request(self, message):
        """Send ``message`` and return the service's ``result`` payload.

        Raises RuntimeError if the service reports an error, or an OSError
        subclass if it cannot be reached.
        """
        payload = json.dumps(message).encode()
        with socket.create_connection(
            (self.host, self.port), timeout=self.timeout_s,
        ) as sock:
            sock.settimeout(self.timeout_s)
            sock.sendall(struct.pack('>I', len(payload)) + payload)
            (length,) = struct.unpack('>I', _recv_exactly(sock, 4))
            body = _recv_exactly(sock, length)
        response = json.loads(body.decode())
        if response.get('status') != 'success':
            raise RuntimeError(
                response.get('message') or 'LNA bias service reported an error'
            )
        return response.get('result')


class LNABiasController:
    """
    Controller for the SOUK cryostat LNA bias system.

    Provides voltage and current control/monitoring for up to 14 LNA
    channels.  Each readout pipeline maps to one LNA channel specified by
    ``cryostat.lna_bias.lna_channel`` in the config.

    Backends
    --------
    ``remote``
        Talk to the LNA bias service on the one RFSoC that is wired to the
        bias board. This is the normal telescope deployment: every readout
        server, including those on the wired RFSoC itself, goes through the
        service so there is a single owner of the board.
    ``i2c``
        Drive the bias board directly over the local I2C bus. Intended for
        bench setups where a board is attached to the machine under test.
    ``fixed``
        Record an explicit, non-controllable bias value from config.

    Parameters
    ----------
    config_dict : dict
        The desired server config. Runtime LNA state is kept separately
        in ``runtime_state`` and does not modify this dict.
    pipeline_id : int
        Pipeline index.
    """

    SUPPORTED_BACKENDS = ('remote', 'i2c', 'fixed')
    CONTROLLABLE_BACKENDS = ('remote', 'i2c')
    DEFAULT_BIAS_VOLTAGE_V = 1.5
    DEFAULT_METHOD = 'local'
    DEFAULT_BLIND = False
    DEFAULT_SOFT_OFF = False

    def __init__(self, config_dict, pipeline_id=0):
        self.config = config_dict
        self.pipeline_id = pipeline_id
        self._monitor = None
        self._service_client = None
        self._hw_version = None
        self._detected_refdes = []
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
        configured_bus = (lna_cfg.get('i2c', {}) or {}).get(
            'bus', LNA_I2C_BUS_NUM,
        )
        try:
            configured_bus = int(configured_bus)
        except (TypeError, ValueError):
            logger.warning(
                'Invalid cryostat.lna_bias.i2c.bus value %r; using SMBus(%d).',
                configured_bus, LNA_I2C_BUS_NUM,
            )
            configured_bus = LNA_I2C_BUS_NUM
        if configured_bus != LNA_I2C_BUS_NUM:
            logger.warning(
                'Ignoring cryostat.lna_bias.i2c.bus=%d for pipeline %d; '
                'the LNA bias board is always on SMBus(%d).',
                configured_bus, self.pipeline_id, LNA_I2C_BUS_NUM,
            )
        self._i2c_bus_num = LNA_I2C_BUS_NUM

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

        if self._backend == 'remote':
            try:
                host, port, timeout_s = resolve_lna_service_endpoint(lna_cfg)
            except ValueError as e:
                logger.error(
                    'LNA bias backend remote enabled but unusable: %s', e,
                )
                return
            self._service_client = _LNAServiceClient(host, port, timeout_s)
            logger.info(
                'LNA bias controller using remote service at %s; '
                'pipeline LNA channel %d',
                self._service_client.endpoint, self._lna_channel,
            )
            return

        if not _HW_AVAILABLE:
            logger.error(
                'LNA bias backend i2c enabled in config but smbus2 / '
                'souk-peripherals-control are not installed — '
                'subsequent LNA calls will fail.'
            )
            return

        try:
            with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias init'):
                bus = SMBus(self._i2c_bus_num)
                monitor, hw_version = open_bias_monitor(
                    bus, lna_cfg.get('hw_version', 'auto'),
                )
            detected = monitor.detected_refdes
            if not detected:
                logger.error(
                    'No LNA channels responded on i2c bus %d — '
                    'subsequent LNA calls will fail.',
                    self._i2c_bus_num,
                )
                return
            self._monitor = monitor
            self._hw_version = hw_version
            self._detected_refdes = detected
            logger.info(
                'LNA bias controller initialised on backend %s, bus %d, '
                'board hw v%d: %d/%d channels detected (%s); '
                'pipeline LNA channel %d',
                self._backend, self._i2c_bus_num, hw_version,
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
        """True if this controller is backed by real hardware.

        True for the remote backend too: the bias board is real, it just
        lives on another RFSoC.
        """
        return self._monitor is not None or self._service_client is not None

    @property
    def is_controllable(self):
        """True when LNA bias can be changed by software."""
        return self._backend in self.CONTROLLABLE_BACKENDS and self.is_hardware

    @property
    def is_remote(self):
        """True when this controller reaches the board via the LNA service."""
        return self._service_client is not None

    @property
    def backend(self):
        """Name of the active LNA bias backend."""
        return self._backend or 'none'

    @property
    def hw_version(self):
        """LNA bias board revision (1 or 2), or None if not yet known.

        For the remote backend this is filled in from the first successful
        service response.
        """
        return self._hw_version

    @property
    def supports_output_enable(self):
        """True when the board has per-channel hard output-enable switches."""
        if self._monitor is not None:
            return self._monitor.supports_output_enable
        return self._hw_version == 2

    @property
    def detected_refdes(self):
        """Refdes (M17-M30) whose monitors answered when the board was opened.

        Empty for the remote and fixed backends, which do not open a board.
        """
        return list(self._detected_refdes)

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

        ``config_dict`` is the config to apply; ``None`` (default) uses this
        controller's stored ``self.config``.
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

        if self.is_remote:
            out = self._remote_call(
                'set_lna_bias_voltage', channel=chn,
                voltage_v=float(voltage_v), method=method, blind=bool(blind),
                fallback={'channel': chn, 'voltage_v': float('nan'),
                          'method': method},
            )
            if out.get('success', True):
                self._sync_runtime_state_from_result(out, blind=blind)
            return out

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias set'):
            if method == 'local':
                vmin, vmax = self._monitor.lna_local_voltage_ranges.get(
                    chn, (float('nan'), float('nan'))
                )
                if (math.isfinite(vmin) and math.isfinite(vmax)
                        and not (vmin - 1e-6 <= voltage_v <= vmax + 1e-6)):
                    return _out_of_local_range_result(
                        chn, voltage_v, 'local', vmin, vmax,
                    )
                result = self._monitor.set_lna_bias_local(
                    chn=[chn], v_local=voltage_v,
                )
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

        if self.is_remote:
            out = self._remote_call(
                'soft_off_lna_bias', channel=chn,
                fallback={'channel': chn, 'voltage_v': float('nan'),
                          'method': 'local', 'soft_off': False},
            )
            if out.get('success', True):
                self._sync_runtime_state_from_result(
                    out, blind=False, soft_off=True,
                )
            return out

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias soft-off'):
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

        if self.is_remote:
            out = self._remote_call(
                'set_lna_bias_voltage_all', voltage_v=float(voltage_v),
                method=method, blind=bool(blind),
            )
            out = self._keyed_by_channel(out, channels, method=method)
            default_result = out.get(self._lna_channel)
            if default_result and default_result.get('success', True):
                self._sync_runtime_state_from_result(
                    default_result, blind=blind, soft_off=False,
                )
            return out

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias set-all'):
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
        channels = list(range(1, NUM_LNA_CHANNELS + 1))
        if self.is_remote:
            out = self._remote_call('soft_off_lna_bias_all')
            return self._keyed_by_channel(out, channels, method='local')
        return {chn: self.soft_off_lna_bias(channel=chn) for chn in channels}

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

        if self.is_remote:
            return self._remote_call(
                'get_lna_bias_status', channel=chn,
                fallback={'channel': chn, 'remote_voltage_v': None,
                          'local_voltage_v': None, 'bias_current_a': None,
                          'output_enabled': None},
            )

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias status'):
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
            'output_enabled': _output_enabled_value(s.get('output enable')),
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

        if self.is_remote:
            out = self._remote_call('get_lna_bias_status_all')
            return self._keyed_by_channel(out, channels)

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA bias status-all'):
            status = self._monitor.read_lna_status(chn=channels)
        return {
            chn: {
                'channel': chn,
                'remote_voltage_v': status[chn]['remote voltage'],
                'local_voltage_v': status[chn]['local voltage'],
                'bias_current_a': status[chn]['bias current'],
                'output_enabled': _output_enabled_value(
                    status[chn].get('output enable')),
                'message': self._check_status_health(
                    chn,
                    status[chn]['remote voltage'],
                    status[chn]['bias current'],
                ),
            }
            for chn in channels
        }

    # -- output enable (v2 boards only) --

    def set_lna_output_enabled(self, enabled, channel=None):
        """Hard-enable or disable the bias output for one LNA channel.

        v2 bias boards have a per-channel output switch, so this genuinely
        removes power from the LNA rather than driving it to its minimum
        voltage. On v1 boards there is no such switch and this returns a
        failure result pointing at ``soft_off_lna_bias`` instead.
        """
        self._require_hardware()
        chn = channel if channel is not None else self._lna_channel
        self._validate_channel(chn)

        if self.is_remote:
            return self._remote_call(
                'set_lna_output_enabled', channel=chn, enabled=bool(enabled),
                fallback={'channel': chn, 'output_enabled': None},
            )

        if not self.supports_output_enable:
            return _output_enable_unsupported_result(chn, self._hw_version)

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA output enable'):
            self._monitor.set_lna_bias_output(chn, bool(enabled))
            state = self._monitor.bias_oe_status.get(chn)
        return {
            'channel': chn,
            'output_enabled': _output_enabled_value(state),
            'message': '',
            'success': bool(state) == bool(enabled),
        }

    def set_lna_output_enabled_all(self, enabled):
        """Hard-enable or disable the bias output for all 14 LNA channels.

        Channels are switched together so the rail settling delay is paid
        once rather than once per channel.
        """
        self._require_hardware()
        channels = list(range(1, NUM_LNA_CHANNELS + 1))

        if self.is_remote:
            out = self._remote_call(
                'set_lna_output_enabled_all', enabled=bool(enabled),
            )
            return self._keyed_by_channel(out, channels)

        if not self.supports_output_enable:
            return {
                chn: _output_enable_unsupported_result(chn, self._hw_version)
                for chn in channels
            }

        with _i2c_bus_lock(self._i2c_bus_num, 'LNA output enable-all'):
            self._monitor.set_lna_bias_output(channels, bool(enabled))
            states = self._monitor.bias_oe_status
        return {
            chn: {
                'channel': chn,
                'output_enabled': _output_enabled_value(states.get(chn)),
                'message': '',
                'success': bool(states.get(chn)) == bool(enabled),
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
            'hw_version': self._hw_version,
            'supports_output_enable': self.supports_output_enable,
            'service_endpoint': (
                self._service_client.endpoint if self.is_remote else None
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
        if not self.is_hardware:
            if self._backend == 'fixed':
                raise RuntimeError('Fixed LNA bias backend is not controllable')
            if self._backend == 'remote':
                raise RuntimeError(
                    'LNA bias service endpoint not configured; see '
                    f'{get_site_config_path()}'
                )
            raise RuntimeError('LNA bias hardware not initialised')

    # -- remote backend --

    def _remote_call(self, request, fallback=None, **kwargs):
        """Forward one request to the LNA bias service and return its result.

        Transport and service-side failures become the same failure-shaped
        result dict the local backends return, so an unreachable service
        degrades this server's LNA control rather than breaking the server.
        """
        message = {'request': request}
        message.update(kwargs)
        try:
            result = self._service_client.request(message)
        except Exception as e:
            logger.warning(
                'LNA bias service request %r to %s failed: %s',
                request, self._service_client.endpoint, e,
            )
            out = dict(fallback or {})
            out['success'] = False
            out['message'] = (
                f'LNA bias service at {self._service_client.endpoint} '
                f'is unreachable: {e}'
            )
            return out
        if isinstance(result, dict) and result.get('hw_version'):
            self._hw_version = int(result['hw_version'])
        return result

    def _keyed_by_channel(self, result, channels, method=None):
        """Re-key a per-channel service response by int channel index.

        JSON object keys are strings, so an all-channel response comes back
        keyed '1'..'14'. A failure-shaped dict (from an unreachable service)
        is expanded to one entry per channel so callers always see the same
        shape.
        """
        if not isinstance(result, dict):
            result = {}
        if 'success' in result and result.get('success') is False:
            per_channel = {}
            for chn in channels:
                entry = dict(result)
                entry['channel'] = chn
                if method is not None:
                    entry['method'] = method
                per_channel[chn] = entry
            return per_channel
        out = {}
        for chn in channels:
            entry = result.get(str(chn), result.get(chn))
            if entry is None:
                entry = {
                    'channel': chn,
                    'message': 'No result returned by the LNA bias service.',
                    'success': False,
                }
            out[chn] = entry
        return out

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

    with _i2c_bus_lock(LNA_I2C_BUS_NUM, 'LNA discovery'):
        try:
            bus = SMBus(LNA_I2C_BUS_NUM)
        except Exception as e:
            print(f'LNA discovery error opening SMBus({LNA_I2C_BUS_NUM}): {e}')
            return []

        try:
            monitor, hw_version = open_bias_monitor(bus)
        except Exception as e:
            print(f'LNA discovery failed: {e}')
            return []

        detected = monitor.detected_refdes
        oe_status = monitor.bias_oe_status if include_state else {}

        results = []
        for refdes in detected:
            chn = _refdes_to_channel(refdes)
            entry = {
                'backend': 'i2c',
                'bus': LNA_I2C_BUS_NUM,
                'refdes': refdes,
                'channel': chn,
                'model': f'SOUK LNA bias monitor {refdes} (board hw v{hw_version})',
            }
            if include_state and chn is not None:
                entry['output_enabled'] = oe_status.get(chn)
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
