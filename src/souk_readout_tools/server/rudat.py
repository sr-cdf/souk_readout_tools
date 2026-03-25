#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
rudat_auto.py — Stateless Mini-Circuits RUDAT-6000-30 USB driver (pyusb)

This version is **stateless**: no persistent USB handle is kept.
Every operation opens the device, performs the command, and immediately
releases it. This enables safe multi-user / multi-process use.

Public API is kept compatible with the original:
    - class Attenuator(bus, address)
    - .att property (get/set)
    - .get_* helpers, find_rudats()

Optional per-device advisory lock (fcntl.flock) prevents interleaved commands
from multiple writers targeting the same physical unit. Disable by setting
USE_ADVISORY_LOCK = False.

Requires:
    pip install pyusb
udev rule example (so no sudo required):
    SUBSYSTEMS=="usb", ATTRS{idVendor}=="20ce", ATTRS{idProduct}=="0023", MODE="0666", GROUP="plugdev"
"""

import os
import time
import tempfile
import usb.core
import usb.util
from contextlib import contextmanager

try:
    import fcntl  # Linux/Unix advisory file locking
    HAS_FCNTL = True
except Exception:
    HAS_FCNTL = False

# ---------------------------------------------------------------------
# USB constants
# ---------------------------------------------------------------------
VEND_ID     = 0x20CE  # Mini-Circuits
PROD_ID     = 0x0023  # RUDAT programmable attenuator

GET_DEVICE_MODEL_NAME       = 40
GET_DEVICE_SERIAL_NUMBER    = 41
SET_ATTENUATION             = 19
READ_ATTENUATION            = 18
GET_FIRMWARE                = 99

NULL         = 0
TX_BUF_SIZE  = 64
RX_BUF_SIZE  = 64
READ_TIMEOUT = 1000  # ms

ATT_MAX     = 30.0
ATT_MIN     = 0.0
RESOLUTION  = 0.25

# Model defaults (extend as needed)
MODEL_DEFAULTS = {
    "RUDAT-6000-30": {"att_min": 0.0, "att_max": 30.0, "resolution": 0.25},
}



# Behaviour knobs
RETRY_COUNT         = 2       # extra tries on transient USB errors
RETRY_SLEEP_SEC     = 0.05
USE_ADVISORY_LOCK   = True    # set False to disable /tmp file lock

# ---------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------


def _quantize_att(v, res, vmin, vmax):
    """Round attenuation to nearest resolution and clamp to range."""
    k = round(float(v) / res)
    q = k * res
    q = round(q, 6)  # avoid float artifacts
    # clamp
    clamped = min(max(q, vmin), vmax)
    return clamped


def _tx_packet(cmd, data=None):
    data = list(data or [])
    buf = [cmd] + data
    if len(buf) > TX_BUF_SIZE:
        raise ValueError("Payload too long for HID report")
    buf.extend([NULL] * (TX_BUF_SIZE - len(buf)))
    return bytes(buf)

@contextmanager
def _device_open(bus, address, detach_kernel_if_needed=True):
    """
    Context manager that opens the device by bus/address, sets configuration,
    yields (dev, intf, ep_out, ep_in), then cleans up and re-attaches kernel drivers.
    """
    dev = usb.core.find(idVendor=VEND_ID, idProduct=PROD_ID, bus=bus, address=address)
    if dev is None:
        raise ValueError(f'RUDAT not found at bus {bus} address {address}')

    detached = []  # track interfaces we detached
    try:
        # Setting configuration (ignore EBUSY already set)
        try:
            dev.set_configuration()
        except usb.core.USBError as e:
            if getattr(e, "errno", None) != 16:  # Resource busy
                raise

        cfg = dev.get_active_configuration()
        intf = cfg[(0, 0)]

        # Detach kernel driver if active (Linux)
        if detach_kernel_if_needed and hasattr(dev, "is_kernel_driver_active"):
            for i in range(intf.bInterfaceNumber, intf.bInterfaceNumber + 1):
                try:
                    if dev.is_kernel_driver_active(i):
                        dev.detach_kernel_driver(i)
                        detached.append(i)
                except usb.core.USBError:
                    # continue; some platforms raise here
                    pass

        ep_out = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        ep_in = usb.util.find_descriptor(
            intf, custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        if ep_out is None or ep_in is None:
            raise RuntimeError("Could not locate IN/OUT endpoints")

        yield dev, intf, ep_out, ep_in

    finally:
        # Release interface & resources
        try:
            usb.util.dispose_resources(dev)
        except Exception:
            pass
        # Re-attach kernel drivers we detached
        for i in detached:
            try:
                dev.attach_kernel_driver(i)
            except Exception:
                pass

@contextmanager
def _device_lock(serial_for_lock: str | None):
    """
    Advisory per-device lock to serialize commands across processes.
    Lock file path: /tmp/rudat.lock.<serial> (or 'bus_addr' fallback).
    """
    if not (USE_ADVISORY_LOCK and HAS_FCNTL and serial_for_lock):
        yield
        return
    path = os.path.join(tempfile.gettempdir(), f"rudat.lock.{serial_for_lock}")
    # Ensure file exists
    fd = os.open(path, os.O_CREAT | os.O_RDWR, 0o666)
    try:
        with os.fdopen(fd, "r+") as f:
            fcntl.flock(f, fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(f, fcntl.LOCK_UN)
    except Exception:
        # If locking fails for any reason, just proceed without it
        yield

def _hid_transaction(bus, address, tx_bytes, expect_read=True, timeout_ms=READ_TIMEOUT):
    """
    Perform one HID transaction: open → write → (optional) read → close.
    Retries on transient USB errors.
    """
    last_err = None
    for attempt in range(RETRY_COUNT + 1):
        try:
            with _device_open(bus, address) as (_, _, ep_out, ep_in):
                wrote = ep_out.write(tx_bytes)
                if wrote != len(tx_bytes):
                    raise RuntimeError(f"Short USB write: {wrote}/{len(tx_bytes)}")
                if not expect_read:
                    return None
                rx = ep_in.read(RX_BUF_SIZE, timeout=timeout_ms)
                return bytes(rx)
        except (usb.core.USBError, TimeoutError, RuntimeError) as e:
            last_err = e
            time.sleep(RETRY_SLEEP_SEC)
    # Exhausted retries
    raise last_err

def _read_string_field(bus, address, command_code, serial_for_lock=None):
    with _device_lock(serial_for_lock):
        rx = _hid_transaction(bus, address, _tx_packet(command_code), expect_read=True)
    start = 3 if command_code == GET_FIRMWARE else 1
    rx = rx or b""
    if len(rx) <= start:
        raise ValueError("Incomplete string response")
    # trim at first NULL
    try:
        end = rx.index(0, start)
        data = rx[start:end]
    except ValueError:
        data = rx[start:]
    try:
        return data.decode("ascii").strip()
    except UnicodeDecodeError:
        return repr(bytes(data))

def _read_atten_raw(bus, address, serial_for_lock=None):
    with _device_lock(serial_for_lock):
        rx = _hid_transaction(bus, address, _tx_packet(READ_ATTENUATION), expect_read=True)
    if rx is None or len(rx) < 3:
        raise ValueError("Incomplete attenuation response")
    integer = float(rx[1])
    frac_quarters = float(rx[2]) / 4.0
    return integer + frac_quarters

def _write_atten_raw(bus, address, att_value, serial_for_lock=None):
    # Device expects integer part and 0.25 dB step count
    integer_part = int(att_value)
    step_count = int(round((att_value - integer_part) / RESOLUTION))
    payload = [integer_part, step_count]
    with _device_lock(serial_for_lock):
        _ = _hid_transaction(bus, address, _tx_packet(SET_ATTENUATION, payload), expect_read=True)




# ---------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------
class Attenuator:
    """
    Stateless controller identified by USB bus & address.
    Stores per-device parameters (min/max/resolution) on the Python object.
    """

    def __init__(self, usb_bus, usb_address, att_min=None, att_max=None, resolution=None):
        self._bus  = int(usb_bus)
        self._addr = int(usb_address)

        # IDs (helpful to keep around)
        self.vid = VEND_ID
        self.pid = PROD_ID

        # Lazy-read strings
        self._model = None
        self._serial = None
        self._firmware = None

        # Try to cache serial early (useful for locks/labels); ignore failure
        try:
            self._serial = _read_string_field(self._bus, self._addr, GET_DEVICE_SERIAL_NUMBER)
        except Exception:
            pass

        # Try to get model now (so we can seed defaults); ignore failure
        try:
            self._model = _read_string_field(self._bus, self._addr, GET_DEVICE_MODEL_NAME, self._serial or None)
        except Exception:
            pass

        # Seed device parameter defaults by model, else fall back to globals
        d = MODEL_DEFAULTS.get(self._model or "", {})
        self.att_min   = att_min   if att_min   is not None else d.get("att_min", ATT_MIN)
        self.att_max   = att_max   if att_max   is not None else d.get("att_max", ATT_MAX)
        self.resolution= resolution if resolution is not None else d.get("resolution", RESOLUTION)

    # ---- info getters unchanged (use cache=True to avoid extra traffic) ----
    def get_model(self, cache=True):
        if cache and self._model is not None:
            return self._model
        s = _read_string_field(self._bus, self._addr, GET_DEVICE_MODEL_NAME, self._serial or None)
        if cache: self._model = s
        return s

    def get_serial(self, cache=True):
        if cache and self._serial is not None:
            return self._serial
        s = _read_string_field(self._bus, self._addr, GET_DEVICE_SERIAL_NUMBER, self._serial or None)
        if cache: self._serial = s
        return s

    def get_firmware_version(self, cache=True):
        if cache and self._firmware is not None:
            return self._firmware
        s = _read_string_field(self._bus, self._addr, GET_FIRMWARE, self._serial or None)
        if cache: self._firmware = s
        return s

    def get_status(self):
        try:
            _ = self.get_model(cache=False)
            return "Connected."
        except Exception as e:
            sn = self._serial or f"bus {self._bus}, addr {self._addr}"
            print(f"Device status check failed for {sn}: {e}")
            return "Not Connected / Error."

    # --- Attenuation (stateless read/write) ---
    @property
    def att(self):
        try:
            return _read_atten_raw(self._bus, self._addr, self._serial or None)
        except Exception as e:
            sn = self._serial or f"bus {self._bus}, addr {self._addr}"
            print(f"Error reading attenuation for {sn}: {e}")
            return None

    @att.setter
    def att(self, value):
        try:
            v_in = float(value)
        except Exception:
            raise ValueError(f"Invalid attenuation value: {value!r}")

        v_q = _quantize_att(v_in, self.resolution, self.att_min, self.att_max)

        # Build warning if adjusted
        reasons = []
        if abs(v_q - v_in) > 1e-9:
            # Distinguish rounding vs clamping for clarity
            # If v_in is out of range, both can happen; list both.
            if (self.att_min <= v_in <= self.att_max) and (abs((v_in/self.resolution) - round(v_in/self.resolution)) > 1e-9):
                reasons.append("rounded to nearest %.2f dB" % self.resolution)
            if v_in < self.att_min or v_in > self.att_max:
                reasons.append("clamped to [%.2f, %.2f] dB" % (self.att_min, self.att_max))

        if reasons:
            model = self._model or "RUDAT"
            serial = self._serial or "?"
            ident = f"{model} SN {serial} (bus {self._bus}, addr {self._addr})"
            print(f"Warning: {ident}: requested {v_in:.3f} dB, set to {v_q:.2f} dB ({', '.join(reasons)}).")

        try:
            _write_atten_raw(self._bus, self._addr, v_q, self._serial or None)
        except Exception as e:
            sn = self._serial or f"bus{self._bus}_addr{self._addr}"
            raise RuntimeError(f"Failed to set attenuation on {sn}: {e}")


    def set_params(self, att_min=None, att_max=None, resolution=None):
        """Update stored device parameters on this object."""
        if att_min is not None:   self.att_min = float(att_min)
        if att_max is not None:   self.att_max = float(att_max)
        if resolution is not None:self.resolution = float(resolution)

    def params(self):
        """Return a dict of stored parameters and identity."""
        return {
            "model": self._model or None,
            "serial": self._serial or None,
            "firmware": self._firmware or None,
            "bus": self._bus,
            "address": self._addr,
            "vid": self.vid,
            "pid": self.pid,
            "att_min": self.att_min,
            "att_max": self.att_max,
            "resolution": self.resolution,
        }

    def describe(self):
        """Pretty print a one-line descriptor."""
        model = self._model or "RUDAT"
        serial = self._serial or "?"
        return (f"{model} SN {serial} (bus {self._bus}, addr {self._addr}); "
                f"limits [{self.att_min:.2f}, {self.att_max:.2f}] dB; "
                f"step {self.resolution:.2f} dB")



    # --- Convenience aliases ---
    def get_atten(self):  # alias
        return self.att

    def set_atten(self, atten):  # alias
        self.att = atten


def find_rudats():
    """
    Discover connected RUDAT devices and return:
        { serial(int or str): {'bus': int, 'address': int} }

    This briefly opens each device to read its serial, then releases it.
    """
    devices = usb.core.find(idVendor=VEND_ID, idProduct=PROD_ID, find_all=True)
    dev_list = list(devices) if devices else []
    if not dev_list:
        print('No RUDAT devices found matching VID/PID on USB buses.')
        return {}

    print(f'Found {len(dev_list)} potential RUDAT device(s).')
    out = {}
    temp_dir = tempfile.gettempdir()

    for dev in dev_list:
        bus, address = dev.bus, dev.address
        # Stateless: open just long enough to read serial
        try:
            with _device_open(bus, address) as _ctx:
                pass  # ensure we can talk, but we read serial via helper below
            serial = _read_string_field(bus, address, GET_DEVICE_SERIAL_NUMBER)
            try:
                key = int(serial)
            except ValueError:
                key = serial  # fallback to string if serial not purely numeric

            out[key] = {'bus': bus, 'address': address}
            # Save a small helper file so other tools can resolve the device
            p = os.path.join(temp_dir, f"rudat_id_{key}")
            try:
                with open(p, "w") as f:
                    f.write(f"{bus},{address}")
                print(f"  Bus {bus:03d} Addr {address:03d} Serial {serial}  →  {p}")
            except Exception as e:
                print(f"  (Warn) Could not write temp id file for serial {serial}: {e}")

        except Exception as e:
            print(f"  * FAILED init at Bus {bus:03d} Address {address:03d}: {e}")

    if not out:
        print("No controllable RUDAT devices were initialized.")
    else:
        print(f"Successfully initialized {len(out)} RUDAT(s).")

    # sort by key for stable order
    return {k: out[k] for k in sorted(out, key=lambda x: str(x))}


# ---------------------------------------------------------------------
# Example CLI probe (read-only)
# ---------------------------------------------------------------------
if __name__ == "__main__":
    print("\nSearching for Mini-Circuits RUDAT Attenuators...")
    rudats = find_rudats()
    if not rudats:
        print("\nNo controllable RUDAT devices were found.")
    else:
        print(f"\n--- Querying {len(rudats)} device(s) ---")
        for serial, info in rudats.items():
            print(f"\nSerial: {serial} (Bus {info['bus']}, Addr {info['address']})")
            r = Attenuator(info['bus'], info['address'])
            try:
                print("  Model:    ", r.get_model())
                print("  Firmware: ", r.get_firmware_version())
                print("  Status:   ", r.get_status())
                a = r.att
                print(f"  Attenuation: {a:.2f} dB" if a is not None else "  Attenuation: <read error>")
            except Exception as e:
                print("  ERROR:", e)
        print("\n--- Done ---")


