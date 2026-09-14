"""RFSoC board management controller (BMC) over USB serial.

The KRM-4ZU47DR carrier boards carry a small board management controller
that owns the power sequencing, rail monitoring, boot-mode selection and
clock-synth configuration for the whole board.  It is reached over a USB
serial link from the *control PC* (not from the RFSoC's own ARM cores):
a single Silicon Labs CP2105 dual UART bridge presents two
``/dev/ttyUSB*`` nodes,
one wired to the ARM Linux console and one to the BMC command interface.

The BMC command syntax is ``target:command`` terminated by a carriage
return, e.g. ``krm4:i`` for a board status dump.  See
``doc/board_management.md`` for the full command list, the standard
settings values, and the safety rules.

This module is deliberately **read-only by construction** apart from the
two power commands:

- ``_send()`` accepts only exact command strings from :data:`READ_COMMANDS`.
- ``power_on()`` / ``power_off()`` are the only writers, they are the only
  callers allowed to pass :data:`POWER_COMMANDS`, and they each require an
  explicit ``confirm=True``.
- Nothing in this module can emit a settings command (``bx``, ``ysx``,
  ``a0``/``a1``, ``q*``, ``vttbst*``, ``r*``) or the flash-save command
  (``f``).  Those are typed by hand, deliberately, in picocom.

Board selection is equally deliberate: every port is identified by the
bridge's unique USB serial number, and the power commands refuse to
run against an auto-selected board.

CLI entry points: ``souk-find-boards``, ``souk-board-status``,
``souk-board-power``.
"""

import argparse
import json
import os
import re
import sys
import time

# ---------------------------------------------------------------------
# Link and device constants
# ---------------------------------------------------------------------
BAUDRATE = 115200          # as used by `picocom -b 115200 /dev/ttyUSB1`

# Port naming and the fix for a refused open are the only OS-dependent parts
# of this module: discovery itself goes through pyserial's cross-platform
# list_ports, and the BMC protocol is the same everywhere.
_WINDOWS = os.name == 'nt'
_EXAMPLE_DEVICE = 'COM7' if _WINDOWS else '/dev/ttyUSB1'
_PERMISSION_HINT = (
    'close any terminal session holding the port (PuTTY, Tera Term), or '
    'check the port is not claimed by another application'
    if _WINDOWS else
    "add yourself to the 'dialout' group (sudo usermod -aG dialout $USER, "
    'then log out and back in), or re-run with sudo')

# USB-serial bridges known to appear on SOUK RFSoC carriers, keyed by
# (vid, pid). The KRM-4ZU47DR uses a Silicon Labs CP2105, whose two UARTs
# enumerate as separate USB interfaces sharing one serial number:
#   interface 00 -- "Enhanced Com Port" (SCI)
#   interface 01 -- "Standard Com Port" (ECI)
# Which of those is wired to the ARM console and which to the BMC is a
# board-design choice, so the roles are still probed rather than assumed.
KNOWN_BRIDGES = {
    (0x10C4, 0xEA70): 'CP2105 dual UART bridge',
}
CP2105_VID = 0x10C4
CP2105_PID = 0xEA70

# Observed CP2105 wiring on SOUK KRM-4ZU47DR boards:
#   interface 00 (Enhanced / SCI) -> ARM Linux console
#   interface 01 (Standard  / ECI) -> board management controller
# The ttyUSB *numbers* move with plug order; the interface numbers do not.
# Probing still wins where it gives an answer -- this mapping is the
# fallback for unprobed or inconclusive ports, and any port resolved this
# way is labelled 'assumed'. Acting on a wrongly assumed port is caught
# before anything is sent, because every command path reads a parseable
# board status first.
EXPECTED_INTERFACE_ROLES = {0: 'console', 1: 'bmc'}

TARGET = 'krm4'            # BMC command target prefix
EOL = '\r'                 # commands are terminated with a carriage return

# Response-collection timing. The BMC streams a multi-line reply with no
# trailing prompt, so reads finish on an idle gap rather than a marker.
READ_IDLE = 0.30           # s of silence that ends a normal reply
READ_TIMEOUT = 3.0         # s hard cap on a normal reply
# The BMC power sequence is a rail-by-rail transcript ending in 'OK'
# (power on) or 'PWR down' (power off). Measured: the lines stream out
# consecutively and the whole sequence is complete in about a second, so
# the 1.5 s idle gap below cannot truncate it mid-sequence. The ~50 s from
# krm4:1 to a login prompt is almost all Linux booting afterwards, and that
# happens on the *console* interface, not this one -- the timeout here
# covers only the BMC transcript.
POWER_IDLE = 1.50          # s of silence that ends a power sequence
POWER_TIMEOUT = 60.0       # s hard cap on a power sequence

# ---------------------------------------------------------------------
# Command allow-lists
#
# _send() compares against these by exact string equality -- there is no
# pattern matching, so no argument can ever be appended to turn a query
# into a setter.
# ---------------------------------------------------------------------
READ_COMMANDS = {
    'type':            '2',    # show board type
    'serial':          '3',    # show board serial
    'fw_revision':     '4',    # show BMC firmware revision
    'status':          'i',    # show board status
    'settings':        's',    # show current settings
    'boot_options':    'b',    # list boot modes (bare 'b' sets nothing; it
                               #   replies 'boot mode invalid option' and
                               #   then lists the valid selections)
    'current_monitor': 'qs',   # show current-monitor settings
    'help':            '?',    # show the command guide
}

POWER_COMMANDS = {
    'off': '0',
    'on':  '1',
}

_READ_SET = frozenset(READ_COMMANDS.values())
_POWER_SET = frozenset(POWER_COMMANDS.values())

# ---------------------------------------------------------------------
# Expected / standard values
#
# Advisory limits used to flag "unusual" readings. They are not vendor
# specifications -- see doc/board_management.md.
# ---------------------------------------------------------------------
VIN_NOMINAL = 12.0
VIN_WARN = (11.4, 12.6)      # +/- 5 %
VIN_ERROR = (10.8, 13.2)     # +/- 10 %
BMC_PWR_NOMINAL = 3.3
BMC_PWR_WARN = (3.0, 3.6)
# Measured on an open rack: 41 C with only the base PL image, 48 C with the
# dual-pipeline design programmed, 62 C with 1000 tones on each pipeline.
# A fully loaded board in an enclosed rack will sit higher still, so these
# are set above the highest observed working value rather than near it --
# a threshold that fires in normal service is worse than none. They remain
# advisory: no vendor maximum is published for this sensor.
TEMP_WARN_C = 75.0
TEMP_ERROR_C = 85.0

# Flag states observed on a healthy board.
STATE_ON = {
    'ext_en': 'high', 'ext_pok': 'high', 'rst_out': 'low',
    'ps_rst': 'released', 'systemstate': 8, 'powerstate': 0,
}
STATE_OFF = {
    'ext_en': 'low', 'ext_pok': 'low', 'rst_out': 'high',
    'ps_rst': 'reset', 'systemstate': 0, 'powerstate': 0,
}

# Standard settings, as reported by `krm4:s`.
STANDARD_SETTINGS = {
    'autostart': 'enabled',
    'vtt_dac': '2,5V',
    'boot_from': '5 - SD1',
    'selected_clock': 0,
}

# Console-side patterns used to recognise the ARM Linux tty so the probe
# never mistakes it for the BMC.
_CONSOLE_PATTERNS = (
    'login:', 'password:', 'ubuntu', 'linux', 'u-boot', 'zynqmp',
    'command not found', 'incorrect', '$ ', '# ',
)


class BoardManagementError(RuntimeError):
    """Raised for refused commands, bad replies, or port problems."""


# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------
def _import_serial():
    """Import pyserial lazily so the rest of the package stays importable."""
    try:
        import serial                      # noqa: F401
        import serial.tools.list_ports      # noqa: F401
    except ImportError as exc:
        raise BoardManagementError(
            'pyserial is required for board management access '
            '(pip install pyserial)') from exc
    return serial


def _interface_number(device):
    """USB interface number (0 or 1) behind a /dev/ttyUSB* node, or None.

    The bridge always presents the console on one USB interface and the
    BMC on the other; only the ttyUSB numbering moves around. Reading the
    interface number from sysfs is therefore the stable identifier.
    """
    name = os.path.basename(os.path.realpath(device))
    path = f'/sys/class/tty/{name}/device/../bInterfaceNumber'
    try:
        with open(path) as f:
            return int(f.read().strip(), 16)
    except (OSError, ValueError):
        pass
    return None


def _slug(text):
    """Lower-case, filesystem-safe token for udev symlink names."""
    return re.sub(r'[^a-z0-9_-]+', '-', str(text).lower()).strip('-')


def _looks_like_device(name):
    """True if `name` is a serial device rather than a serial *number*.

    POSIX device nodes and udev aliases start with ``/dev/``; Windows names
    its ports ``COM1``.. (``\\\\.\\COM10`` and above), and those exist as
    neither a path nor a file, so ``os.path.exists`` cannot be the only test.
    """
    if not name:
        return False
    text = str(name)
    if text.startswith('/dev/') or os.path.exists(text):
        return True
    return bool(re.fullmatch(r'(?:\\\\\.\\)?COM\d+', text, re.IGNORECASE))


def _open_port(device, timeout=0.1):
    """Open a serial port without disturbing the board.

    DTR and RTS are deasserted *before* the port is opened so nothing is
    pulsed on connect, and the port is opened exclusively so an attached
    picocom session is never fought over.
    """
    serial = _import_serial()
    ser = serial.Serial()
    ser.port = device
    ser.baudrate = BAUDRATE
    ser.timeout = timeout
    ser.write_timeout = 2.0
    ser.dtr = False
    ser.rts = False
    ser.exclusive = True
    try:
        ser.open()
    except Exception as exc:
        raise BoardManagementError(
            f'could not open {device}: {exc}\n'
            f'  (is a terminal session attached? {_PERMISSION_HINT})') from exc
    return ser


def _read_until_idle(ser, idle=READ_IDLE, timeout=READ_TIMEOUT, stop=()):
    """Collect a reply, ending on `idle` seconds of silence.

    `stop` is an optional tuple of strings; the read returns early once
    one of them appears on a line of its own.
    """
    deadline = time.time() + timeout
    last = time.time()
    buf = ''
    while time.time() < deadline:
        chunk = ser.read(4096)
        if chunk:
            buf += chunk.decode('utf-8', errors='replace')
            last = time.time()
            if stop and any(line.strip() in stop for line in buf.splitlines()):
                time.sleep(idle)
                buf += ser.read(4096).decode('utf-8', errors='replace')
                break
        elif buf and (time.time() - last) >= idle:
            break
        elif not buf and (time.time() - last) >= idle:
            time.sleep(0.02)
    return buf


def _strip_echo(reply, sent):
    """Drop the echoed command line (the BMC echoes when `e1` is set)."""
    lines = reply.splitlines()
    while lines and (not lines[0].strip() or lines[0].strip() == sent.strip()):
        lines.pop(0)
    return '\n'.join(lines).strip()


def _parse_fields(text):
    """Parse `key: value` lines into a dict with normalised keys."""
    out = {}
    for line in text.splitlines():
        if ':' not in line:
            continue
        key, _, value = line.partition(':')
        key = re.sub(r'[^a-z0-9]+', '_', key.strip().lower()).strip('_')
        value = value.strip()
        if not key or not value:
            continue
        out[key] = _coerce(key, value)
    return out


def _parse_boot_options(text):
    """Parse the boot-mode list from ``krm4:b`` into {code: name}.

    The reply opens with ``boot mode invalid option`` -- bare ``b`` carries
    no selection, so the BMC lists what it would accept. It reports the
    *available* modes, not the active one; the active one is ``Boot from``
    in ``krm4:s``.
    """
    out = {}
    for line in text.splitlines():
        m = re.match(r'\s*([0-9a-fA-F])\s*-\s*(\S.*?)\s*$', line)
        if m:
            out[m.group(1).lower()] = m.group(2)
    return out


# Header lines that carry no key/value pair.
_MONITOR_HEADERS = ('current monitor settings',)


def _parse_current_monitor(text):
    """Parse ``krm4:qs`` into a dict.

    Unlike ``krm4:i`` and ``krm4:s``, these lines have no colon: the value
    is the final whitespace-separated token and the key is everything
    before it (``filter coeff short 1`` -> ``filter_coeff_short: '1'``).
    """
    out = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or line.lower() in _MONITOR_HEADERS:
            continue
        words = line.split()
        if len(words) < 2:
            continue
        key = re.sub(r'[^a-z0-9]+', '_', ' '.join(words[:-1]).lower()).strip('_')
        if key:
            out[key] = words[-1]
    return out


def _coerce(key, value):
    """Convert known numeric fields; leave everything else as text."""
    if key.startswith('p') and re.fullmatch(r'0x[0-9a-fA-F]+', value):
        return int(value, 16)
    if key in ('bmc_temp',):
        m = re.match(r'(-?\d+(?:\.\d+)?)', value)
        return float(m.group(1)) if m else value
    if key in ('bmc_pwr', 'vin'):
        try:
            return float(value)
        except ValueError:
            return value
    if key in ('systemstate', 'powerstate', 'selected_clock'):
        try:
            return int(value)
        except ValueError:
            return value
    if key in ('ext_en', 'ext_pok', 'rst_out', 'cfg_ok'):
        return value.lower()
    if key in ('ps_rst', 'pl_done'):
        # 'released (high)' / 'yes (high)' -> 'released' / 'yes'
        return value.split()[0].lower()
    if key in ('start_en', 'run_en'):
        return value.lower()
    return value


# ---------------------------------------------------------------------
# Controller
# ---------------------------------------------------------------------
class BoardManagementController:
    """Read-only BMC client, with power on/off behind an explicit confirm.

    Parameters
    ----------
    device : str
        Serial device for the BMC interface, e.g. ``/dev/ttyUSB1`` or a
        udev alias such as ``/dev/souk/rfsoc01-bmc``.
    usb_serial : str, optional
        USB serial number this device was resolved from; carried through
        to reports so the board is always named in output.
    """

    def __init__(self, device, usb_serial=None):
        self.device = device
        self.usb_serial = usb_serial
        self._ser = None

    # -- lifecycle ----------------------------------------------------
    def open(self):
        if self._ser is None:
            self._ser = _open_port(self.device)
        return self

    def close(self):
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def __enter__(self):
        return self.open()

    def __exit__(self, *exc):
        self.close()
        return False

    # -- transport ----------------------------------------------------
    def _send(self, command, _power=False, idle=READ_IDLE,
              timeout=READ_TIMEOUT, stop=()):
        """Send one allow-listed command and return its reply text.

        Only exact members of :data:`READ_COMMANDS` are accepted. Power
        commands additionally require ``_power=True``, which only
        :meth:`power_on` and :meth:`power_off` pass.
        """
        if command in _READ_SET:
            pass
        elif command in _POWER_SET and _power:
            pass
        else:
            raise BoardManagementError(
                f'refusing to send {TARGET}:{command!r} -- not an allowed '
                'command. This tool only issues queries and, on explicit '
                'request, power on/off. Settings, boot, clock, POR/PROG '
                'and flash-save commands must be typed by hand.')

        if self._ser is None:
            self.open()
        line = f'{TARGET}:{command}{EOL}'
        self._ser.reset_input_buffer()
        self._ser.write(line.encode('ascii'))
        self._ser.flush()
        reply = _read_until_idle(self._ser, idle=idle, timeout=timeout,
                                 stop=stop)
        return _strip_echo(reply, f'{TARGET}:{command}')

    # -- queries ------------------------------------------------------
    def get_type(self):
        """Board type string, e.g. ``KRM-4ZU47DRSE1XCA-F1-SP``."""
        return self._send(READ_COMMANDS['type']).strip()

    def get_serial(self):
        """Board serial as held by the BMC (not the bridge USB serial)."""
        return self._send(READ_COMMANDS['serial']).strip()

    def get_fw_revision(self):
        """BMC firmware revision block.

        Reports one line each for the master and the two slave
        controllers (the same ``slave 1``/``slave 2`` devices the
        power-up sequence initialises).
        """
        return self._send(READ_COMMANDS['fw_revision']).strip()

    def get_help(self):
        """The BMC's own command guide."""
        return self._send(READ_COMMANDS['help'])

    def get_status(self, raw=False):
        """Parsed ``krm4:i`` board status."""
        text = self._send(READ_COMMANDS['status'])
        fields = _parse_fields(text)
        if not fields:
            raise BoardManagementError(
                f'no board status returned from {self.device} -- is this the '
                'BMC interface rather than the ARM console?')
        return (fields, text) if raw else fields

    def get_boot_options(self, raw=False):
        """Boot modes the BMC will accept, as ``{code: name}``.

        This lists what is *available*, not what is selected -- the active
        selection is ``boot_from`` in :meth:`get_settings`.
        """
        text = self._send(READ_COMMANDS['boot_options'])
        options = _parse_boot_options(text)
        return (options, text) if raw else options

    def get_current_monitor(self, raw=False):
        """Parsed ``krm4:qs`` current-monitor settings.

        Note its ``autostart`` is the *monitor's* autostart, unrelated to
        the board ``Autostart`` in :meth:`get_settings`.
        """
        text = self._send(READ_COMMANDS['current_monitor'])
        fields = _parse_current_monitor(text)
        return (fields, text) if raw else fields

    def get_settings(self, raw=False):
        """Parsed ``krm4:s`` settings."""
        text = self._send(READ_COMMANDS['settings'])
        fields = _parse_fields(text)
        return (fields, text) if raw else fields

    def identify(self):
        """Type + serial + firmware revision, for confirming the target."""
        return {
            'device': self.device,
            'usb_serial': self.usb_serial,
            'type': self.get_type(),
            'serial': self.get_serial(),
            'fw_revision': self.get_fw_revision(),
        }

    def power_state(self):
        """``'on'``, ``'off'`` or ``'unknown'`` from the status flags."""
        return power_state(self.get_status())

    # -- the only writers ---------------------------------------------
    def power_on(self, confirm=False):
        """Power the board up (``krm4:1``). Requires ``confirm=True``.

        Returns the raw power-sequencer transcript.
        """
        if confirm is not True:
            raise BoardManagementError(
                'power_on() requires confirm=True')
        return self._send(POWER_COMMANDS['on'], _power=True,
                          idle=POWER_IDLE, timeout=POWER_TIMEOUT,
                          stop=('OK',))

    def power_off(self, confirm=False):
        """Power the board down (``krm4:0``). Requires ``confirm=True``.

        This removes board power. It is not an OS shutdown and gives the
        ARM Linux system no warning, so shut the OS down cleanly first
        (``sudo poweroff``) where the board is reachable. Note that the OS
        shutdown does *not* touch board power -- this call is still what
        actually removes it.
        """
        if confirm is not True:
            raise BoardManagementError(
                'power_off() requires confirm=True')
        return self._send(POWER_COMMANDS['off'], _power=True,
                          idle=POWER_IDLE, timeout=POWER_TIMEOUT,
                          stop=('PWR down',))


# ---------------------------------------------------------------------
# Status interpretation
# ---------------------------------------------------------------------
def power_state(status):
    """Infer the board power state from a parsed status dict."""
    ext_en = status.get('ext_en')
    system = status.get('systemstate')
    if ext_en == 'high' and system == 8:
        return 'on'
    if ext_en == 'low' and system == 0:
        return 'off'
    return 'unknown'


def check_status(status, settings=None):
    """Flag anything unusual in a parsed status (and optional settings).

    Returns a list of ``(level, message)`` with level in
    ``'error'``, ``'warn'``, ``'info'``.
    """
    issues = []
    state = power_state(status)

    vin = status.get('vin')
    if isinstance(vin, float):
        if not (VIN_ERROR[0] <= vin <= VIN_ERROR[1]):
            issues.append(('error', f'Vin {vin:.1f} V is far from the '
                                    f'{VIN_NOMINAL:.0f} V nominal supply'))
        elif not (VIN_WARN[0] <= vin <= VIN_WARN[1]):
            issues.append(('warn', f'Vin {vin:.1f} V is outside '
                                   f'{VIN_WARN[0]}-{VIN_WARN[1]} V'))

    pwr = status.get('bmc_pwr')
    if isinstance(pwr, float) and not (BMC_PWR_WARN[0] <= pwr <= BMC_PWR_WARN[1]):
        issues.append(('warn', f'BMC supply {pwr:.1f} V is outside '
                               f'{BMC_PWR_WARN[0]}-{BMC_PWR_WARN[1]} V'))

    temp = status.get('bmc_temp')
    if isinstance(temp, float):
        if temp >= TEMP_ERROR_C:
            issues.append(('error', f'BMC temperature {temp:.0f} C is at or '
                                    f'above {TEMP_ERROR_C:.0f} C -- check '
                                    'airflow'))
        elif temp >= TEMP_WARN_C:
            issues.append(('warn', f'BMC temperature {temp:.0f} C is at or '
                                   f'above {TEMP_WARN_C:.0f} C'))

    if status.get('ext_en') == 'high' and status.get('ext_pok') == 'low':
        issues.append(('error', 'EXT EN is high but EXT POK is low -- a '
                                'board power rail is not good'))

    powerstate = status.get('powerstate')
    if isinstance(powerstate, int) and powerstate != 0:
        issues.append(('error', f'powerstate {powerstate} is non-zero -- the '
                                'power sequencer reports a fault'))

    if state == 'unknown':
        issues.append(('warn', 'power state is indeterminate '
                               f"(EXT EN={status.get('ext_en')}, "
                               f"systemstate={status.get('systemstate')}) -- "
                               'the board may be mid-sequence'))
    else:
        expected = STATE_ON if state == 'on' else STATE_OFF
        for key, want in expected.items():
            got = status.get(key)
            if got is not None and got != want:
                issues.append(('warn', f'{key} is {got!r}, expected {want!r} '
                                       f'for a board that is {state}'))

    if state == 'on' and status.get('pl_done') == 'no':
        issues.append(('info', 'PL_DONE is low -- no bitstream is loaded. '
                               'Normal between power-on and Linux loading '
                               'the base PL image; but a PL rail collapse '
                               'leaves exactly this reading too, with every '
                               'other field still healthy. If the board was '
                               'running firmware, check the clocks from the '
                               'PS (krc-utils status).'))

    if settings:
        for key, want in STANDARD_SETTINGS.items():
            got = settings.get(key)
            if got is not None and str(got) != str(want):
                issues.append(('warn', f'setting {key} is {got!r}, standard '
                                       f'value is {want!r}'))

    return issues


# ---------------------------------------------------------------------
# Discovery
# ---------------------------------------------------------------------
def _probe_role(device):
    """Classify a port, returning ``(role, note, identity)``.

    `role` is ``'bmc'``, ``'console'`` or ``'unknown'``; `note` explains
    an inconclusive probe (no permission, port busy, no reply) so the
    caller can say *why* it fell back to the interface table rather than
    silently guessing. `identity` is ``{'type', 'serial'}`` when the port
    turned out to be a BMC -- the probe already has the port open and has
    asked for the board type, so reading the board serial costs one more
    query and binds the board to the bridge it is wired to.

    The probe is harmless on both interfaces: it sends a bare carriage
    return first, which draws a login/shell prompt from the ARM console
    and nothing meaningful from the BMC. Only if that looks nothing like
    a console does it try the read-only ``krm4:2`` query.
    """
    def _query(ser, command):
        ser.reset_input_buffer()
        ser.write(f'{TARGET}:{command}{EOL}'.encode('ascii'))
        ser.flush()
        return _strip_echo(_read_until_idle(ser, idle=0.4, timeout=2.0),
                           f'{TARGET}:{command}')

    try:
        ser = _open_port(device)
    except BoardManagementError as exc:
        return 'unknown', _probe_open_note(device, exc), None
    try:
        ser.reset_input_buffer()
        ser.write(EOL.encode('ascii'))
        ser.flush()
        reply = _read_until_idle(ser, idle=0.4, timeout=1.5)
        if any(p in reply.lower() for p in _CONSOLE_PATTERNS):
            return 'console', None, None

        reply = _query(ser, READ_COMMANDS['type'])
        low = reply.lower()
        if reply and 'unknown command' not in low:
            if any(p in low for p in _CONSOLE_PATTERNS):
                return 'console', None, None
            serial = _query(ser, READ_COMMANDS['serial']).strip() or None
            return 'bmc', None, {'type': reply.strip(), 'serial': serial}
        if 'unknown command' in low:
            return 'bmc', None, None
        return 'unknown', ('no reply to a bare CR or to krm4:2 '
                           '(port opened cleanly)'), None
    except Exception as exc:
        return 'unknown', f'{type(exc).__name__}: {exc}', None
    finally:
        ser.close()


def _probe_open_note(device, exc):
    """Explain an open failure, with the likely fix."""
    text = str(exc)
    low = text.lower()
    if 'permission denied' in low or 'access is denied' in low:
        return f'permission denied opening {device} -- {_PERMISSION_HINT}'
    # pyserial takes flock(LOCK_EX|LOCK_NB); a held port fails as
    # "Could not exclusively lock port ...: Resource temporarily
    # unavailable" rather than as EBUSY.
    if ('device or resource busy' in low or 'exclusive' in low
            or 'resource temporarily unavailable' in low):
        return (f'{device} is already open -- close any picocom/screen '
                'session on it (note: a session started with sudo holds '
                'the lock as root)')
    return text.replace('\n', ' ')


def find_boards(probe=True, vid=None, pid=None, all_ports=False):
    """Discover board USB-serial bridges and group their ports by serial.

    A control PC can easily have dozens of ``/dev/ttyUSB*`` nodes, so by
    default only bridges in :data:`KNOWN_BRIDGES` are considered. Pass
    explicit `vid`/`pid`, or ``all_ports=True``, to widen the search.

    Returns a list of dicts, one per USB device, sorted by serial::

        {'usb_serial': '01449C3F', 'bridge': 'CP2105 dual UART bridge',
         'board_serial': '24NPv3ZnDm', 'board_type': 'KRM-4ZU47DR...',
         'vid': 0x10c4, 'pid': 0xea70, 'product': ..., 'manufacturer': ...,
         'ports': {0: '/dev/ttyUSB0', 1: '/dev/ttyUSB1'},
         'descriptions': {0: 'CP2105 ... Enhanced Com Port', 1: '...'},
         'roles': {0: 'console', 1: 'bmc'},
         'probe_notes': {},   # why any port came back 'unknown'
         'bmc_port': '/dev/ttyUSB1', 'console_port': '/dev/ttyUSB0'}

    With ``probe=False`` the roles are left unresolved -- useful when a
    console session is live and should not be touched at all.
    """
    _import_serial()
    from serial.tools import list_ports

    boards = {}
    for port in list_ports.comports():
        if not all_ports:
            if vid is not None or pid is not None:
                if vid is not None and port.vid != vid:
                    continue
                if pid is not None and port.pid != pid:
                    continue
            elif (port.vid, port.pid) not in KNOWN_BRIDGES:
                continue
        key = port.serial_number or f'unknown@{port.location}'
        entry = boards.setdefault(key, {
            'usb_serial': port.serial_number,
            'bridge': KNOWN_BRIDGES.get((port.vid, port.pid), 'unknown'),
            'vid': port.vid,
            'pid': port.pid,
            'product': port.product,
            'manufacturer': port.manufacturer,
            'ports': {},
            'descriptions': {},
            'roles': {},
            'probe_notes': {},
            'board_type': None,     # from krm4:2, only when probed
            'board_serial': None,   # from krm4:3, only when probed
            'bmc_port': None,
            'console_port': None,
        })
        ifnum = _interface_number(port.device)
        if ifnum is None and port.location and '.' in port.location:
            try:
                ifnum = int(port.location.rsplit('.', 1)[1])
            except ValueError:
                ifnum = None
        if ifnum is None:
            ifnum = len(entry['ports'])
        entry['ports'][ifnum] = port.device
        entry['descriptions'][ifnum] = port.description

    for entry in boards.values():
        for ifnum, device in sorted(entry['ports'].items()):
            if probe:
                role, note, ident = _probe_role(device)
                if note:
                    entry['probe_notes'][ifnum] = note
                if ident:
                    entry['board_type'] = ident.get('type')
                    entry['board_serial'] = ident.get('serial')
            else:
                role = 'unprobed'
            entry['roles'][ifnum] = role
            if role == 'bmc' and entry['bmc_port'] is None:
                entry['bmc_port'] = device
            elif role == 'console' and entry['console_port'] is None:
                entry['console_port'] = device

        # Fall back to the known interface wiring for anything the probe
        # did not settle.
        for ifnum, expected in EXPECTED_INTERFACE_ROLES.items():
            device = entry['ports'].get(ifnum)
            key = f'{expected}_port'
            if device is None or entry[key] is not None:
                continue
            if entry['roles'].get(ifnum) in (None, 'unprobed', 'unknown'):
                entry[key] = device
                entry['roles'][ifnum] = f'{expected} (assumed)'

    return [boards[k] for k in sorted(boards, key=str)]


def resolve_board(board=None, probe=True):
    """Resolve a board selector to ``(device, usb_serial)``.

    `board` may be:

    - a device path (``/dev/ttyUSB1``, or a udev alias such as
      ``/dev/souk/rfsoc01-bmc``);
    - the bridge's **USB serial number** (``01449C3F``), which needs no
      port to be opened; or
    - the **board serial** the BMC itself reports (``24NPv3ZnDm``), which
      names the board rather than the USB bridge wired to it. Matching on
      this requires probing, so it does not work with ``probe=False``.

    If `board` is None, a board is auto-selected only when exactly one is
    present -- and the power CLI never allows that.
    """
    if _looks_like_device(board):
        return board, None

    found = find_boards(probe=probe)
    if board:
        matches = [b for b in found if b['usb_serial'] == board]
        if not matches:
            # Fall back to the board serial the BMC reports, then to a
            # case-insensitive comparison of either.
            low = board.lower()
            matches = [b for b in found if (b.get('board_serial') or '') == board]
            if not matches:
                matches = [b for b in found
                           if low in {str(b['usb_serial']).lower(),
                                      str(b.get('board_serial') or '').lower()}]
        if not matches:
            known = ', '.join(
                f"{b['usb_serial']}"
                + (f" (board {b['board_serial']})" if b.get('board_serial') else '')
                for b in found) or 'none'
            raise BoardManagementError(
                f'no board matching {board!r} by USB serial or board serial '
                f'(found: {known})')
        if len(matches) > 1:
            raise BoardManagementError(
                f'{board!r} matches more than one board -- use the USB '
                'serial or the device path')
        entry = matches[0]
    else:
        if not found:
            raise BoardManagementError(
                'no known USB serial bridges found')
        if len(found) > 1:
            known = ', '.join(str(b['usb_serial']) for b in found)
            raise BoardManagementError(
                f'{len(found)} boards attached ({known}) -- name one with '
                '--board')
        entry = found[0]

    if entry['bmc_port'] is None:
        raise BoardManagementError(
            f"could not identify the BMC interface on board "
            f"{entry['usb_serial']} (ports: "
            f"{sorted(entry['ports'].values())}); name the device directly "
            f"with --board {_EXAMPLE_DEVICE}")
    return entry['bmc_port'], entry['usb_serial']


def udev_rules(boards=None):
    """Generate udev rules giving each board's two ports stable aliases."""
    if boards is None:
        boards = find_boards()
    lines = [
        '# SOUK RFSoC USB serial aliases',
        '# Generated by `souk-find-boards --udev`. Rename the symlinks to',
        '# something meaningful (e.g. souk/rfsoc01-bmc), then install with:',
        '#   sudo cp 99-souk-rfsoc-serial.rules /etc/udev/rules.d/',
        '#   sudo udevadm control --reload && sudo udevadm trigger',
        '',
    ]
    for entry in boards:
        usb_serial = entry['usb_serial']
        if not usb_serial:
            continue
        # udev can only match on USB attributes, so the rule is always
        # keyed by the bridge serial -- but the symlink is named after the
        # board itself when the probe has told us what it is.
        name = _slug(entry.get('board_serial') or usb_serial)
        vid = entry.get('vid') or CP2105_VID
        pid = entry.get('pid') or CP2105_PID
        lines.append(f'# {entry.get("bridge") or "USB serial bridge"} '
                     f'({usb_serial})')
        if entry.get('board_serial'):
            lines.append(f'#   board serial {entry["board_serial"]}'
                         + (f' -- {entry["board_type"]}'
                            if entry.get('board_type') else ''))
        for ifnum in sorted(entry['ports']):
            role = entry['roles'].get(ifnum)
            suffix = role if role in ('bmc', 'console') else f'if{ifnum}'
            desc = entry.get('descriptions', {}).get(ifnum)
            if desc:
                lines.append(f'#   interface {ifnum:02d}: {desc}')
            lines.append(
                f'SUBSYSTEM=="tty", ATTRS{{idVendor}}=="{vid:04x}", '
                f'ATTRS{{idProduct}}=="{pid:04x}", '
                f'ATTRS{{serial}}=="{usb_serial}", '
                f'ENV{{ID_USB_INTERFACE_NUM}}=="{ifnum:02d}", '
                f'SYMLINK+="souk/{name}-{suffix}", '
                f'GROUP="dialout", MODE="0660"')
        lines.append('')
    return '\n'.join(lines)


# ---------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------
_LEVEL_TAG = {'error': 'ERROR', 'warn': 'WARN ', 'info': 'info '}


def _short_description(entry, ifnum):
    """The distinguishing tail of a port description.

    CP2105 descriptions repeat the controller name on both ports and only
    differ after the final ' - ' ("Enhanced Com Port" / "Standard Com
    Port"), so only that part is worth showing per line.
    """
    desc = (entry.get('descriptions') or {}).get(ifnum) or ''
    return desc.rsplit(' - ', 1)[-1].strip()


def _print_status(ident, status, settings, issues):
    print(f"\nBoard        : {ident.get('type') or '<unknown>'}")
    print(f"BMC serial   : {ident.get('serial') or '<unknown>'}")
    if ident.get('usb_serial'):
        print(f"USB serial   : {ident['usb_serial']}")
    print(f"Device       : {ident['device']}")
    print(f"Power state  : {power_state(status).upper()}")
    print()
    print(f"  BMC temp   : {status.get('bmc_temp')} C")
    print(f"  BMC supply : {status.get('bmc_pwr')} V")
    print(f"  Vin        : {status.get('vin')} V")
    print(f"  EXT EN     : {status.get('ext_en')}")
    print(f"  EXT POK    : {status.get('ext_pok')}")
    print(f"  RST_OUT    : {status.get('rst_out')}")
    print(f"  CFG_OK     : {status.get('cfg_ok')}")
    print(f"  PS_RST     : {status.get('ps_rst')}")
    print(f"  PL_DONE    : {status.get('pl_done')}")
    print(f"  systemstate: {status.get('systemstate')}")
    print(f"  powerstate : {status.get('powerstate')}")
    ports = ' '.join(f'{k.upper()}=0x{status[k]:02x}'
                     for k in ('p0', 'p1', 'p2', 'p3') if k in status)
    if ports:
        print(f"  GPIO ports : {ports}")
    if settings:
        print()
        print(f"  Autostart  : {settings.get('autostart')}")
        print(f"  VTT_DAC    : {settings.get('vtt_dac')}")
        print(f"  Boot from  : {settings.get('boot_from')}")
        print(f"  Clock      : {settings.get('selected_clock')}")

    print()
    if not issues:
        print('  All checks OK.')
    else:
        for level, message in issues:
            print(f'  [{_LEVEL_TAG[level]}] {message}')
    print()


def _exit_code(issues):
    if any(level == 'error' for level, _ in issues):
        return 2
    if any(level == 'warn' for level, _ in issues):
        return 1
    return 0


# ---------------------------------------------------------------------
# CLI: souk-find-boards
# ---------------------------------------------------------------------
def _cli_find_boards():
    parser = argparse.ArgumentParser(
        description='List attached RFSoC USB serial bridges and identify '
                    'which port is the board management interface. '
                    'Read-only.')
    parser.add_argument('--no-probe', action='store_true',
                        help='do not open the ports to classify them')
    parser.add_argument('--all-ports', action='store_true',
                        help='list every USB serial port, not just the '
                             'bridges known to appear on SOUK boards')
    parser.add_argument('--json', action='store_true',
                        help='emit machine-readable JSON')
    parser.add_argument('--udev', action='store_true',
                        help='emit a udev rules file giving each port a '
                             'stable /dev/souk/... alias (Linux only; on '
                             'Windows select boards by USB serial instead)')
    args = parser.parse_args()

    try:
        boards = find_boards(probe=not args.no_probe,
                             all_ports=args.all_ports)
    except BoardManagementError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2

    if args.udev:
        if _WINDOWS:
            print('--udev generates Linux udev rules and does nothing on '
                  'Windows; select a board by its USB serial number '
                  '(-b/--board) instead, which is stable across replugs.',
                  file=sys.stderr)
            return 1
        print(udev_rules(boards))
        return 0
    if args.json:
        print(json.dumps(boards, indent=2, default=str))
        return 0

    if not boards:
        print('No known USB serial bridges found '
              '(try --all-ports to list every serial device).')
        return 1
    print(f'\nFound {len(boards)} USB serial bridge(s):\n')
    for entry in boards:
        print(f"  USB serial : {entry['usb_serial']}")
        if entry.get('board_serial'):
            print(f"  Board      : {entry.get('board_type') or '<unknown>'}")
            print(f"  Board serial: {entry['board_serial']}")
        print(f"  Bridge     : {entry.get('bridge')} "
              f"({entry.get('vid') or 0:04x}:{entry.get('pid') or 0:04x})")
        roles = {i: entry['roles'].get(i, 'unprobed')
                 for i in entry['ports']}
        width = max((len(r) for r in roles.values()), default=0)
        for ifnum in sorted(entry['ports']):
            print(f"    interface {ifnum:02d}: "
                  f"{entry['ports'][ifnum]:<14} "
                  f"{roles[ifnum]:<{width}}  "
                  f"{_short_description(entry, ifnum)}".rstrip())
        for ifnum in sorted(entry.get('probe_notes') or {}):
            print(f"      ! interface {ifnum:02d} not probed: "
                  f"{entry['probe_notes'][ifnum]}")
        print()
    return 0


# ---------------------------------------------------------------------
# CLI: souk-board-status
# ---------------------------------------------------------------------
def _cli_status():
    parser = argparse.ArgumentParser(
        description='Read and check the board management controller state. '
                    'Read-only: this command cannot change any setting.')
    parser.add_argument('-b', '--board', default=None,
                        help='USB serial number, or device name such as '
                             f'{_EXAMPLE_DEVICE} '
                             '(optional when exactly one board is attached)')
    parser.add_argument('-a', '--all', action='store_true',
                        help='report every attached board')
    parser.add_argument('--raw', action='store_true',
                        help='print the raw BMC reply text as well')
    parser.add_argument('--json', action='store_true',
                        help='emit machine-readable JSON')
    args = parser.parse_args()

    try:
        if args.all:
            targets = [(b['bmc_port'], b['usb_serial'])
                       for b in find_boards() if b['bmc_port']]
            if not targets:
                raise BoardManagementError('no BMC interfaces found')
        else:
            targets = [resolve_board(args.board)]
    except BoardManagementError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2

    reports, worst = [], 0
    for device, usb_serial in targets:
        try:
            with BoardManagementController(device, usb_serial) as bmc:
                ident = bmc.identify()
                status, status_raw = bmc.get_status(raw=True)
                settings, settings_raw = bmc.get_settings(raw=True)
                # Extra read-only queries, only for a full capture.
                extra = {}
                if args.raw or args.json:
                    boot, boot_raw = bmc.get_boot_options(raw=True)
                    monitor, mon_raw = bmc.get_current_monitor(raw=True)
                    extra = {'boot_options': boot, 'current_monitor': monitor,
                             'raw': [('krm4:b', boot_raw),
                                     ('krm4:qs', mon_raw)]}
        except BoardManagementError as exc:
            print(f'ERROR: {device}: {exc}', file=sys.stderr)
            worst = 2
            continue

        issues = check_status(status, settings)
        worst = max(worst, _exit_code(issues))
        if args.json:
            reports.append({'identity': ident, 'status': status,
                            'settings': settings,
                            'boot_options': extra.get('boot_options'),
                            'current_monitor': extra.get('current_monitor'),
                            'power_state': power_state(status),
                            'issues': [{'level': lvl, 'message': msg}
                                       for lvl, msg in issues]})
        else:
            _print_status(ident, status, settings, issues)
            if args.raw:
                print('--- krm4:i ---')
                print(status_raw)
                print('--- krm4:s ---')
                print(settings_raw)
                for label, text in extra.get('raw', ()):
                    print(f'--- {label} ---')
                    print(text)
                print()

    if args.json:
        print(json.dumps(reports if args.all else (reports[0] if reports else {}),
                         indent=2, default=str))
    return worst


# ---------------------------------------------------------------------
# CLI: souk-board-power
# ---------------------------------------------------------------------
def _confirm(prompt, assume_yes):
    if assume_yes:
        return True
    if not sys.stdin.isatty():
        print('ERROR: refusing to change board power without --yes on a '
              'non-interactive stdin', file=sys.stderr)
        return False
    try:
        return input(f'{prompt} [y/N] ').strip().lower() in ('y', 'yes')
    except (EOFError, KeyboardInterrupt):
        print()
        return False


def _cli_power():
    parser = argparse.ArgumentParser(
        description='Report or change the RFSoC board power state via the '
                    'board management controller. Power changes require an '
                    'explicitly named board and a confirmation.')
    parser.add_argument('action', choices=['status', 'on', 'off', 'cycle'],
                        help="'status' is read-only; 'on'/'off'/'cycle' "
                             'switch the board rails')
    parser.add_argument('-b', '--board', default=None,
                        help='USB serial number, or device name such as '
                             f'{_EXAMPLE_DEVICE}. '
                             'Required for on/off/cycle.')
    parser.add_argument('--expect-serial', default=None,
                        help='abort unless the BMC reports this board serial '
                             '(krm4:3) -- the safest way to pin the target')
    parser.add_argument('--yes', action='store_true',
                        help='skip the interactive confirmation')
    parser.add_argument('--off-delay', type=float, default=10.0,
                        help='seconds to wait between off and on for '
                             "'cycle' (default: 10). The BMC acknowledges "
                             'a power-off immediately, but the rails take '
                             'a few seconds to actually drop -- the board '
                             'fan is the visible cue -- so do not shorten '
                             'this without reason')
    args = parser.parse_args()

    if args.action != 'status' and not args.board:
        parser.error(f"'{args.action}' requires --board; this tool will not "
                     'auto-select a board for a power change')

    try:
        device, usb_serial = resolve_board(args.board)
    except BoardManagementError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2

    try:
        with BoardManagementController(device, usb_serial) as bmc:
            ident = bmc.identify()
            state = bmc.power_state()
            label = (f"{ident['type']} serial {ident['serial']} "
                     f"on {device}")
            print(f'\nTarget : {label}')
            print(f'State  : {state.upper()}')

            if args.expect_serial and ident['serial'] != args.expect_serial:
                print(f"\nERROR: board serial {ident['serial']!r} does not "
                      f'match --expect-serial {args.expect_serial!r}; '
                      'aborting.', file=sys.stderr)
                return 2

            if args.action == 'status':
                print()
                return 0

            if args.action == 'on' and state == 'on':
                print('\nAlready powered on; nothing to do.')
                return 0
            if args.action == 'off' and state == 'off':
                print('\nAlready powered off; nothing to do.')
                return 0

            verb = {'on': 'Power ON', 'off': 'Power OFF',
                    'cycle': 'Power CYCLE'}[args.action]
            if args.action in ('off', 'cycle'):
                print('\nNote: this removes board power immediately, with no '
                      'warning to the ARM Linux system.')
                print('      Shut the OS down cleanly first '
                      '(`sudo poweroff`) and wait for it to halt; an OS '
                      'shutdown')
                print('      does not touch board power, so this step is '
                      'still needed to remove it.')
            if not _confirm(f'\n{verb} {label}?', args.yes):
                print('Aborted.')
                return 1

            if args.action in ('off', 'cycle'):
                print('\n' + bmc.power_off(confirm=True))
            if args.action == 'cycle':
                print(f'\nWaiting {args.off_delay:.0f} s...')
                time.sleep(args.off_delay)
            if args.action in ('on', 'cycle'):
                print('\n' + bmc.power_on(confirm=True))

            status = bmc.get_status()
            issues = check_status(status)
            print(f'\nState  : {power_state(status).upper()}')
            if args.action in ('on', 'cycle'):
                print('         (rails only -- Linux needs roughly 50 s '
                      'more to reach a login prompt;')
                print('          watch the console interface, not this '
                      'one)')
            for level, message in issues:
                print(f'  [{_LEVEL_TAG[level]}] {message}')
            print()
            return _exit_code(issues)
    except BoardManagementError as exc:
        print(f'ERROR: {exc}', file=sys.stderr)
        return 2


# ---------------------------------------------------------------------
# Standalone dispatch
#
# This module imports nothing from the rest of the package (stdlib plus a
# lazy pyserial), so it can be run straight from a checkout with no
# install at all:
#
#     python3 src/souk_readout_tools/board_management.py find-boards
#     python3 -m souk_readout_tools.board_management status --board 01449C3F
# ---------------------------------------------------------------------
_COMMANDS = {
    'find-boards': _cli_find_boards,
    'status': _cli_status,
    'power': _cli_power,
}

_USAGE = """usage: board_management.py <command> [options]

commands:
  find-boards   list attached board USB serial bridges (read-only)
  status        read and check the board management controller (read-only)
  power         report or change the board power state

Add -h after a command for its options, e.g.
  python3 board_management.py find-boards -h
"""


def _main(argv=None):
    argv = sys.argv if argv is None else argv
    if len(argv) > 1 and argv[1] in _COMMANDS:
        command = argv.pop(1)
        argv[0] = f'{os.path.basename(argv[0])} {command}'
        return _COMMANDS[command]()
    print(_USAGE, file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(_main())
