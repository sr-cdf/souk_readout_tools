#!/usr/bin/env python3
"""
Timing monitor: polls ptp4l (via pmc) and chrony (via chronyc),
maintains timing state, exposes JSON status and commands on a Unix socket.

State machine:
  - locked_to_gm:  fresh PTP data from a grandmaster
  - ptp_holdover:  chrony is following PHC after this monitor saw PTP lock
  - phc_free_run:  chrony is following PHC, but no PTP lock has been observed
  - ntp_synced:    chrony is following an NTP source
  - ntp_holdover:  no source is currently selected, but one was seen before
  - free_run:      no usable source has been seen after startup grace expired
  - initializing:  startup grace period, no usable source yet

The authoritative "PTP healthy" signal is pmc port state plus fresh ingress
activity, not chrony's source selection. Chrony can keep trusting a PHC after
PTP loss because PHC drift is slow relative to NTP error bounds.

Protocol on the Unix socket (one JSON object per line, both directions):

  Client -> Server requests:
    {"cmd": "status"}                -> returns one status object
    {"cmd": "subscribe"}             -> server streams status every poll_interval
    {"cmd": "ping"}                  -> returns {"pong": <unix_time>}

  Server -> Client responses:
    {"ok": true,  "data": {...status...}}
    {"ok": false, "error": "..."}

Status object fields documented in the Status dataclass below.
"""
from __future__ import annotations

import argparse
import asyncio
import ctypes
import json
import logging
import os
import re
import time
from dataclasses import dataclass, field, asdict
from enum import Enum
from pathlib import Path
from typing import Optional


def parse_bool(value: Optional[str]) -> Optional[bool]:
    """Parse common pmc/chrony boolean spellings in ``value`` (None if unknown)."""
    if value is None:
        return None
    value_l = value.strip().lower()
    if value_l in ("1", "true", "yes", "y", "on"):
        return True
    if value_l in ("0", "false", "no", "n", "off"):
        return False
    return None


def parse_seconds(value: Optional[str]) -> Optional[float]:
    """Parse a chrony time string ``value`` into seconds (None if not parseable)."""
    if value is None:
        return None
    value = value.strip()
    if not value or value == "-":
        return None
    try:
        return float(value)
    except ValueError:
        pass

    m = re.match(r"^([+-]?\d+(?:\.\d+)?)(ns|us|ms|s|m|h|d|y)$", value)
    if not m:
        return None
    number = float(m.group(1))
    scale = {
        "ns": 1e-9,
        "us": 1e-6,
        "ms": 1e-3,
        "s": 1.0,
        "m": 60.0,
        "h": 3600.0,
        "d": 86400.0,
        "y": 31557600.0,
    }[m.group(2)]
    return number * scale


class _Timex(ctypes.Structure):
    """struct timex, for ntp_adjtime(2)."""
    _fields_ = [("modes", ctypes.c_uint), ("offset", ctypes.c_long), ("freq", ctypes.c_long),
                ("maxerror", ctypes.c_long), ("esterror", ctypes.c_long), ("status", ctypes.c_int),
                ("constant", ctypes.c_long), ("precision", ctypes.c_long),
                ("tolerance", ctypes.c_long), ("time_sec", ctypes.c_long),
                ("time_usec", ctypes.c_long), ("tick", ctypes.c_long), ("ppsfreq", ctypes.c_long),
                ("jitter", ctypes.c_long), ("shift", ctypes.c_int), ("stabil", ctypes.c_long),
                ("jitcnt", ctypes.c_long), ("calcnt", ctypes.c_long), ("errcnt", ctypes.c_long),
                ("stbcnt", ctypes.c_long), ("tai", ctypes.c_int), ("_pad", ctypes.c_int * 11)]


class _PtpClockTime(ctypes.Structure):
    _fields_ = [("sec", ctypes.c_int64), ("nsec", ctypes.c_uint32), ("reserved", ctypes.c_uint32)]


class _PtpSysOffsetPrecise(ctypes.Structure):
    _fields_ = [("device", _PtpClockTime), ("sys_realtime", _PtpClockTime),
                ("sys_monoraw", _PtpClockTime), ("_rsv", ctypes.c_uint32 * 4)]


STA_UNSYNC = 0x0040
PTP_SYS_OFFSET_PRECISE = 0xC0403D08   # _IOWR('=', 8, struct ptp_sys_offset_precise)
# end0's PHC under a name that follows it (data/timing/90-ptp-end0.rules): the
# kernel renumbers /dev/ptpN every time the interface goes down and up.
PHC_DEVICE = "/dev/ptp-end0"


def kernel_clock_status() -> dict:
    """
    The kernel's own view of the system clock, from ntp_adjtime(2): ~7 us and no
    subprocess, against ~4 ms to spawn `chronyc tracking` for the same facts.
    Returns synced (STA_UNSYNC clear), freq_ppm, esterror_s and maxerror_s, or
    an empty dict if the call fails.
    """
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        t = _Timex()
        rc = libc.ntp_adjtime(ctypes.byref(t))
        if rc < 0:
            return {}
        return {"synced": rc != 5 and not (t.status & STA_UNSYNC),   # 5 = TIME_ERROR
                "freq_ppm": t.freq / 65536.0,
                "esterror_s": t.esterror / 1e6,
                "maxerror_s": t.maxerror / 1e6}
    except Exception:
        return {}


class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_int64), ("tv_nsec", ctypes.c_int64)]


CLOCK_REALTIME = 0


def _fd_to_clockid(fd: int) -> int:
    """The dynamic POSIX clock id of an open /dev/ptpN (FD_TO_CLOCKID)."""
    return (~fd << 3) | 3


def phc_sys_offset_s(fd: int) -> Optional[float]:
    """
    System clock minus PHC, measuring the two clocks instead of reporting what
    chrony believes about them. Needs an open PHC_DEVICE (root); None if neither
    method works.

    Uses PTP_SYS_OFFSET_PRECISE where the driver cross-timestamps both clocks in
    hardware (~15 us), else reads the PHC between two system-clock reads and
    takes the midpoint -- three clock_gettime calls, a few microseconds, with an
    error bounded by the time the sandwich took.
    """
    if fd is None or fd < 0:
        # FD_TO_CLOCKID(-1) is CLOCK_THREAD_CPUTIME_ID, which would succeed and
        # return a plausible-looking number, so refuse a bad fd outright.
        return None
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        o = _PtpSysOffsetPrecise()
        if libc.ioctl(fd, ctypes.c_ulong(PTP_SYS_OFFSET_PRECISE), ctypes.byref(o)) == 0:
            phc_ns = o.device.sec * 10**9 + o.device.nsec
            sys_ns = o.sys_realtime.sec * 10**9 + o.sys_realtime.nsec
            return (sys_ns - phc_ns) / 1e9

        before, phc, after = _Timespec(), _Timespec(), _Timespec()
        clockid = _fd_to_clockid(fd)
        if (libc.clock_gettime(CLOCK_REALTIME, ctypes.byref(before)) != 0
                or libc.clock_gettime(clockid, ctypes.byref(phc)) != 0
                or libc.clock_gettime(CLOCK_REALTIME, ctypes.byref(after)) != 0):
            return None
        b = before.tv_sec * 10**9 + before.tv_nsec
        a = after.tv_sec * 10**9 + after.tv_nsec
        p = phc.tv_sec * 10**9 + phc.tv_nsec
        return ((b + a) / 2 - p) / 1e9
    except Exception:
        return None


def set_proc_comm(name: str) -> None:
    """Set kernel comm name (max 15 chars). Affects /proc/PID/comm."""
    try:
        libc = ctypes.CDLL("libc.so.6", use_errno=True)
        PR_SET_NAME = 15
        truncated = name.encode("utf-8")[:15]
        libc.prctl(PR_SET_NAME, ctypes.c_char_p(truncated), 0, 0, 0)
    except Exception:
        pass


def set_process_title(name: str) -> None:
    """Set the user-space and kernel process names to ``name`` when optional
    support exists."""
    try:
        import setproctitle
        setproctitle.setproctitle(name)
    except ImportError:
        pass
    set_proc_comm(name)

# ----------------------------------------------------------------------------
# Configuration
# ----------------------------------------------------------------------------

SOCKET_PATH = "/run/timing-monitor.sock"
POLL_INTERVAL_S = 1.0

# TSU strobe daemon status file (written by tsu_strobe_service.py each second).
STROBE_STATUS_PATH = "/run/tsu-strobe.status"
# Status older than this is considered stale -> strobe not healthy.
STROBE_STALE_AFTER_S = 3.0

# Threshold for considering PTP "well locked" (nanoseconds)
PTP_LOCK_OFFSET_THRESHOLD_NS = 1_000  # 1 us
LOCK_STABLE_POLLS = 5   # consecutive good polls required for "stable"
PTP_INGRESS_STALE_AFTER_S = 5.0
INITIALIZING_GRACE_S = 30.0

# Nominal holdover policy. PHC-selected holdover can continue after the window,
# but the status will flag it as expired.
PTP_HOLDOVER_WINDOW_S = 60.0
PTP_HOLDOVER_ERROR_RATE_PPM = 1.0

# Subprocess timeouts
PMC_TIMEOUT_S = 2.0
CHRONYC_TIMEOUT_S = 2.0
# How often the slow-changing detail is refreshed. Every poll runs two
# subprocesses -- pmc PORT_DATA_SET and TIME_STATUS_NP, the PTP link and
# grandmaster offset -- plus two syscalls for the system clock itself
# (kernel_clock_status, phc_sys_offset_s), which is what the lock and readiness
# logic needs live. Everything chrony reports is identity or history that only
# changes when the source does (ref name/id, leap status, the source listings),
# so it moves here, as do the grandmaster parent data and UTC properties.
# Nine spawns a second cost a streaming pipeline 2.5 accumulations a second at
# 2400 Hz (doc/v712-notes.md); a spawn is ~4 ms against ~7 us for the syscalls.
# Status carries `detail_age_s` so the slow fields are never read as live.
DETAIL_INTERVAL_S = 30.0
# How often the two pmc queries run. They are the only per-poll subprocesses
# left, and what costs a streaming pipeline is the spawn itself -- page tables
# built and torn down, TLB-shootdown IPIs to other cores, the cluster's L2
# polluted -- not the CPU time. Only pmc can say whether ptp4l still has the
# grandmaster (if it loses it, the PHC free-runs and the system clock follows it
# happily, so no syscall notices), so this is the detection latency for that.
PTP_INTERVAL_S = 5.0

log = logging.getLogger("timing-monitor")


# ----------------------------------------------------------------------------
# State definitions
# ----------------------------------------------------------------------------

class TimingState(str, Enum):
    INITIALIZING = "initializing"
    LOCKED_TO_GM = "locked_to_gm"
    PTP_HOLDOVER = "ptp_holdover"
    PHC_FREE_RUN = "phc_free_run"
    NTP_SYNCED   = "ntp_synced"
    NTP_HOLDOVER = "ntp_holdover"
    FREE_RUN     = "free_run"


@dataclass
class Status:
    # Meta
    state: str = TimingState.INITIALIZING.value
    timestamp: float = 0.0
    poll_count: int = 0
    monitor_uptime_s: float = 0.0
    initializing_grace_s: float = INITIALIZING_GRACE_S

    # Two separate notions of "lock"
    ptp_link_up: bool = False              # port SLAVE, fresh GM present
    ptp_quality_good: bool = False         # offset within threshold this poll
    ptp_quality_stable: bool = False       # offset within threshold for N consecutive polls
    consecutive_good_polls: int = 0        # how many polls in a row with good quality

    # PTP fields (from pmc)
    ptp_port_state: str = "UNKNOWN"
    ptp_data_fresh: bool = False
    ptp_ingress_time_ns: Optional[int] = None
    ptp_seconds_since_ingress: Optional[float] = None
    ptp_seconds_since_ingress_update: Optional[float] = None
    ptp_ingress_stale_after_s: float = PTP_INGRESS_STALE_AFTER_S
    ptp_master_offset_ns: Optional[float] = None
    ptp_gm_present: bool = False
    ptp_gm_identity: str = ""
    ptp_last_gm_identity: str = ""
    ptp_gm_clock_class: Optional[int] = None
    ptp_gm_clock_accuracy: str = ""
    ptp_gm_offset_scaled_log_variance: Optional[int] = None
    ptp_utc_offset: Optional[int] = None
    ptp_utc_offset_valid: Optional[bool] = None
    ptp_port_data_set_text: str = ""
    ptp_time_status_np_text: str = ""
    ptp_parent_data_set_text: str = ""
    ptp_time_properties_data_set_text: str = ""

    # Chrony fields (from chronyc -c)
    chrony_ref_id: str = ""
    chrony_ref_name: str = ""
    chrony_stratum: Optional[int] = None
    chrony_system_time_s: Optional[float] = None
    chrony_system_time_offset_s: Optional[float] = None
    chrony_last_offset_s: Optional[float] = None
    chrony_rms_offset_s: Optional[float] = None
    chrony_root_delay_s: Optional[float] = None
    chrony_root_dispersion_s: Optional[float] = None
    chrony_root_distance_s: Optional[float] = None
    chrony_leap_status: str = ""
    chrony_selected_source: str = ""
    chrony_source_type: str = "unknown"
    chrony_sources: list[dict] = field(default_factory=list)
    chrony_freq_ppm: Optional[float] = None
    chrony_tracking_text: str = ""
    chrony_sources_text: str = ""
    detail_age_s: Optional[float] = None   # age of the slow-cadence fields above
    ptp_age_s: Optional[float] = None      # age of the pmc fields (PTP_INTERVAL_S)

    # Measured every poll by syscall, no subprocess (see kernel_clock_status,
    # phc_sys_offset_s). system_clock_synced is the kernel's own sync flag;
    # phc_sys_offset_s is the system clock against the PHC, latched together.
    system_clock_synced: Optional[bool] = None
    system_clock_freq_ppm: Optional[float] = None
    phc_sys_offset_s: Optional[float] = None

    # Derived / tracked
    seconds_since_ptp_lock: Optional[float] = None
    seconds_since_any_sync: Optional[float] = None
    seconds_since_timing_source_sync: Optional[float] = None
    seconds_since_selected_source_seen: Optional[float] = None
    estimated_abs_error_s: Optional[float] = None
    ptp_lock_observed: bool = False
    ptp_holdover_window_s: float = PTP_HOLDOVER_WINDOW_S
    ptp_holdover_error_rate_ppm: float = PTP_HOLDOVER_ERROR_RATE_PPM
    ptp_lock_offset_threshold_ns: int = PTP_LOCK_OFFSET_THRESHOLD_NS
    ptp_holdover_within_window: bool = False
    ptp_holdover_expired: bool = False
    ptp_holdover_age_known: bool = False

    # For firmware timestamp sync decisions
    ready_for_firmware_sync: bool = False

    # TSU strobe (1-PPS into firmware) -- from the strobe daemon's status file
    # /run/tsu-strobe.status. strobe_present=False means the file is missing
    # (daemon never ran); strobe_running reflects the daemon's own flag; the
    # status is "healthy" only if present, running, and fresh.
    strobe_present: bool = False
    strobe_running: bool = False
    strobe_healthy: bool = False
    strobe_pid: Optional[int] = None
    strobe_rearms: Optional[int] = None
    strobe_tsu_sec: Optional[int] = None
    strobe_status_age_s: Optional[float] = None


# ----------------------------------------------------------------------------
# Subprocess helpers
# ----------------------------------------------------------------------------

async def _run_subprocess(*args: str, timeout: float) -> str:
    """Run a subprocess, return stdout decoded, empty string on failure."""
    try:
        proc = await asyncio.create_subprocess_exec(
            *args,
            stdout=asyncio.subprocess.PIPE,
            stderr=asyncio.subprocess.PIPE,
        )
        try:
            stdout, _ = await asyncio.wait_for(
                proc.communicate(), timeout=timeout
            )
            return stdout.decode("utf-8", errors="replace")
        except asyncio.TimeoutError:
            proc.kill()
            await proc.wait()
            log.warning("subprocess timeout: %s", args[0])
            return ""
    except FileNotFoundError:
        log.error("command not found: %s", args[0])
        return ""
    except Exception:
        log.exception("subprocess error: %s", args[0])
        return ""


# ----------------------------------------------------------------------------
# pmc query helpers
# ----------------------------------------------------------------------------

class PMCClient:
    """Async wrapper around `pmc` for ptp4l management queries."""

    @staticmethod
    async def _get(message: str) -> str:
        # `sudo` may be needed depending on uds_address permissions.
        # Default ptp4l UDS path is /var/run/ptp4l with mode 0660,
        # so reading it usually requires root or membership of the right group.
        # Easiest: run timing-monitor as root via systemd.
        return await _run_subprocess(
            "pmc", "-u", "-b", "0", message, timeout=PMC_TIMEOUT_S
        )

    @staticmethod
    def _parse_field(text: str, key: str) -> Optional[str]:
        # Match "key value" with the key as a whole word
        m = re.search(rf"^\s*{re.escape(key)}\s+(\S+)", text, re.MULTILINE)
        return m.group(1) if m else None

    @classmethod
    async def get_port_data_set(cls) -> dict:
        text = await cls._get("GET PORT_DATA_SET")
        return {
            "port_state": cls._parse_field(text, "portState") or "UNKNOWN",
            "_text": text,
        }

    @classmethod
    async def get_time_status(cls) -> dict:
        text = await cls._get("GET TIME_STATUS_NP")
        return {
            "master_offset": cls._parse_field(text, "master_offset"),
            "ingress_time":  cls._parse_field(text, "ingress_time"),
            "gm_present":    cls._parse_field(text, "gmPresent"),
            "gm_identity":   cls._parse_field(text, "gmIdentity"),
            "_text": text,
        }

    @classmethod
    async def get_parent(cls) -> dict:
        text = await cls._get("GET PARENT_DATA_SET")
        return {
            "gm_clock_class": (
                cls._parse_field(text, "gm.ClockClass")
                or cls._parse_field(text, "gm.clockClass")
                or cls._parse_field(text, "clockClass")
            ),
            "gm_clock_accuracy": (
                cls._parse_field(text, "gm.ClockAccuracy")
                or cls._parse_field(text, "gm.clockAccuracy")
                or cls._parse_field(text, "clockAccuracy")
            ),
            "gm_offset_scaled_log_variance": (
                cls._parse_field(text, "gm.OffsetScaledLogVariance")
                or cls._parse_field(text, "gm.offsetScaledLogVariance")
                or cls._parse_field(text, "offsetScaledLogVariance")
            ),
            "_text": text,
        }

    @classmethod
    async def get_time_properties(cls) -> dict:
        text = await cls._get("GET TIME_PROPERTIES_DATA_SET")
        return {
            "current_utc_offset":       cls._parse_field(text, "currentUtcOffset"),
            "current_utc_offset_valid": cls._parse_field(text, "currentUtcOffsetValid"),
            "_text": text,
        }


# ----------------------------------------------------------------------------
# chronyc query helpers
# ----------------------------------------------------------------------------

class ChronycClient:
    """Async wrapper around `chronyc -c` (CSV output)."""

    @staticmethod
    async def _get(*args: str) -> str:
        return await _run_subprocess(
            "chronyc", "-c", *args, timeout=CHRONYC_TIMEOUT_S
        )

    @staticmethod
    async def _get_plain(*args: str) -> str:
        # -n: numeric addresses. Without it the human-readable `sources` hangs
        # on name resolution on the board (>10 s, measured 2026-09-22, while
        # `getent hosts` answers in 6 ms), so every call hit the timeout and
        # logged a warning; the CSV forms were never affected.
        return await _run_subprocess(
            "chronyc", "-n", *args, timeout=CHRONYC_TIMEOUT_S
        )

    @staticmethod
    async def _get_source_names(*args: str) -> str:
        return await _run_subprocess(
            "chronyc", "-N", "-c", *args, timeout=CHRONYC_TIMEOUT_S
        )

    @classmethod
    async def tracking(cls) -> dict:
        # CSV: ref_id, ref_name, stratum, ref_time, sys_time, last_offset,
        #      rms_offset, freq, resid_freq, skew, root_delay, root_disp,
        #      update_int, leap_status
        text = await cls._get("tracking")
        parts = text.strip().split(",")
        if len(parts) < 14:
            return {}
        try:
            return {
                "ref_id":            parts[0],
                "ref_name":          parts[1],
                "stratum":           int(parts[2]),
                "system_time_s":     float(parts[4]),
                "last_offset_s":     float(parts[5]),
                "rms_offset_s":      float(parts[6]),
                "freq_ppm":          float(parts[7]),
                "root_delay_s":      float(parts[10]),
                "root_dispersion_s": float(parts[11]),
                "root_distance_s":   float(parts[10]) / 2 + float(parts[11]),
                "leap_status":       parts[13],
            }
        except (ValueError, IndexError):
            return {}

    @classmethod
    async def tracking_text(cls) -> str:
        """The human-readable `chronyc tracking`, for the detail cadence."""
        return await cls._get_plain("tracking")

    @staticmethod
    def _parse_sources_csv(text: str) -> list[dict]:
        # CSV columns are usually M,S,name,stratum,poll,reach,last_rx,
        # adjusted_offset,measured_offset,estimated_error. Some chrony versions
        # combine M/S as ^* or #*.
        mode_names = {"^": "ntp", "=": "peer", "#": "refclock"}
        state_names = {
            "*": "selected",
            "+": "combined",
            "-": "not_combined",
            "x": "falseticker",
            "~": "too_variable",
            "?": "unusable",
        }
        sources = []
        for line in text.strip().split("\n"):
            if not line.strip():
                continue
            parts = line.split(",")
            if len(parts) >= 8 and parts[0] in mode_names and parts[1] in state_names:
                mode_symbol = parts[0]
                state_symbol = parts[1]
                name_index = 2
            elif len(parts) >= 7 and len(parts[0]) >= 2:
                mode_symbol = parts[0][0]
                state_symbol = parts[0][1]
                name_index = 1
            else:
                continue

            if mode_symbol not in mode_names or state_symbol not in state_names:
                continue

            def parse_int(index):
                try:
                    return int(parts[index])
                except (ValueError, IndexError):
                    return None

            last_rx = (
                parse_seconds(parts[name_index + 4])
                if len(parts) > name_index + 4 else None
            )
            adjusted_offset = (
                parse_seconds(parts[name_index + 5])
                if len(parts) > name_index + 5 else None
            )
            measured_offset = (
                parse_seconds(parts[name_index + 6])
                if len(parts) > name_index + 6 else None
            )
            error = (
                parse_seconds(parts[name_index + 7])
                if len(parts) > name_index + 7 else None
            )
            sources.append({
                "name": parts[name_index],
                "mode": mode_names[mode_symbol],
                "mode_symbol": mode_symbol,
                "state": state_names[state_symbol],
                "state_symbol": state_symbol,
                "selected": state_symbol == "*",
                "usable": state_symbol in ("*", "+", "-"),
                "stratum": parse_int(name_index + 1),
                "poll": parse_int(name_index + 2),
                "reach": parse_int(name_index + 3),
                "last_rx_s": last_rx,
                "offset_s": adjusted_offset,
                "adjusted_offset_s": adjusted_offset,
                "measured_offset_s": measured_offset,
                "estimated_error_s": error,
            })
        return sources

    @classmethod
    async def sources(cls) -> list[dict]:
        sources = cls._parse_sources_csv(await cls._get("sources"))
        name_sources = cls._parse_sources_csv(
            await cls._get_source_names("sources")
        )

        for index, source in enumerate(sources):
            original_name = None
            if index < len(name_sources):
                candidate = name_sources[index]
                if (
                    candidate.get("mode_symbol") == source.get("mode_symbol")
                    and candidate.get("state_symbol") == source.get("state_symbol")
                ):
                    original_name = candidate.get("name")

            label = source.get("name", "")
            if source.get("mode") in ("ntp", "peer"):
                source["address"] = label
                source["configured_name"] = original_name or label
                source["name"] = source["configured_name"]
            else:
                source["address"] = None
                source["configured_name"] = original_name or label

        return sources

    @classmethod
    async def sources_text(cls) -> str:
        return await cls._get_plain("sources", "-v")

    @classmethod
    async def selected_source(cls) -> str:
        for source in await cls.sources():
            if source.get("selected"):
                return source.get("name", "")
        return ""


# ----------------------------------------------------------------------------
# State machine
# ----------------------------------------------------------------------------

class TimingMonitor:
    def __init__(
        self,
        ptp_holdover_window_s: float = PTP_HOLDOVER_WINDOW_S,
        ptp_holdover_error_rate_ppm: float = PTP_HOLDOVER_ERROR_RATE_PPM,
        ptp_ingress_stale_after_s: float = PTP_INGRESS_STALE_AFTER_S,
        ptp_lock_offset_threshold_ns: int = PTP_LOCK_OFFSET_THRESHOLD_NS,
        initializing_grace_s: float = INITIALIZING_GRACE_S,
        detail_interval_s: float = DETAIL_INTERVAL_S,
        ptp_interval_s: float = PTP_INTERVAL_S,
        time_func=time.time,
        monotonic_func=time.monotonic,
    ) -> None:
        self.status = Status()
        self._time_func = time_func
        self._monotonic_func = monotonic_func
        self._start_monotonic = self._monotonic_func()
        self.ptp_holdover_window_s = float(ptp_holdover_window_s)
        self.ptp_holdover_error_rate_ppm = float(ptp_holdover_error_rate_ppm)
        self.ptp_ingress_stale_after_s = float(ptp_ingress_stale_after_s)
        self.ptp_lock_offset_threshold_ns = int(ptp_lock_offset_threshold_ns)
        self.initializing_grace_s = float(initializing_grace_s)
        self.status.ptp_holdover_window_s = self.ptp_holdover_window_s
        self.status.ptp_holdover_error_rate_ppm = self.ptp_holdover_error_rate_ppm
        self.status.ptp_ingress_stale_after_s = self.ptp_ingress_stale_after_s
        self.status.ptp_lock_offset_threshold_ns = self.ptp_lock_offset_threshold_ns
        self.status.initializing_grace_s = self.initializing_grace_s
        self._last_ptp_lock_time: Optional[float] = None
        self._last_any_sync_time: Optional[float] = None
        self._last_ptp_abs_error_s: Optional[float] = None
        self._last_ingress_time_ns: Optional[int] = None
        self._last_ingress_change_monotonic: Optional[float] = None
        self._lock = asyncio.Lock()
        self.detail_interval_s = float(detail_interval_s)
        self.ptp_interval_s = float(ptp_interval_s)
        self._ptp: Optional[tuple] = None             # last pmc results
        self._ptp_monotonic: Optional[float] = None
        self._phc_fd: Optional[int] = self._open_phc()
        self._detail: Optional[tuple] = None          # last slow-query results
        self._detail_monotonic: Optional[float] = None

    @staticmethod
    def _open_phc() -> Optional[int]:
        """An fd on PHC_DEVICE, or None (not root, or no such device)."""
        try:
            return os.open(PHC_DEVICE, os.O_RDONLY)
        except OSError:
            return None

    async def poll_once(self) -> None:
        now = self._time_func()
        monotonic_now = self._monotonic_func()
        monitor_uptime_s = max(0.0, monotonic_now - self._start_monotonic)

        # pmc is the only spawning query left on the regular path.
        if (self._ptp is None or self._ptp_monotonic is None
                or monotonic_now - self._ptp_monotonic >= self.ptp_interval_s):
            self._ptp = await asyncio.gather(
                PMCClient.get_port_data_set(),
                PMCClient.get_time_status(),
                return_exceptions=False,
            )
            self._ptp_monotonic = monotonic_now
        port_data, time_status = self._ptp
        ptp_age_s = monotonic_now - self._ptp_monotonic
        # The detail refreshes on its own, slower, cadence (DETAIL_INTERVAL_S).
        detail_due = (self._detail is None or self._detail_monotonic is None
                      or monotonic_now - self._detail_monotonic >= self.detail_interval_s)
        if detail_due:
            self._detail = await asyncio.gather(
                PMCClient.get_parent(),
                PMCClient.get_time_properties(),
                ChronycClient.tracking(),
                ChronycClient.sources(),
                ChronycClient.sources_text(),
                ChronycClient.tracking_text(),
                return_exceptions=False,
            )
            self._detail_monotonic = monotonic_now
        (parent, time_props, tracking, chrony_sources, chrony_sources_text,
         tracking_text) = self._detail
        detail_age_s = monotonic_now - self._detail_monotonic
        clock = kernel_clock_status()
        phc_offset = phc_sys_offset_s(self._phc_fd) if self._phc_fd is not None else None
        if phc_offset is None:
            # An ifdown/ifup replaces the PHC, and the old fd reads nothing
            # from then on: open it again by name and retry once.
            if self._phc_fd is not None:
                os.close(self._phc_fd)
            self._phc_fd = self._open_phc()
            phc_offset = phc_sys_offset_s(self._phc_fd) if self._phc_fd is not None else None
        port_state = (port_data.get("port_state") or "UNKNOWN").upper()
        selected = next(
            (source.get("name", "") for source in chrony_sources
             if source.get("selected")),
            "",
        )

        async with self._lock:
            s = self.status
            s.timestamp = now
            s.poll_count += 1
            s.monitor_uptime_s = monitor_uptime_s
            s.initializing_grace_s = self.initializing_grace_s
            s.ptp_holdover_window_s = self.ptp_holdover_window_s
            s.ptp_holdover_error_rate_ppm = self.ptp_holdover_error_rate_ppm
            s.ptp_ingress_stale_after_s = self.ptp_ingress_stale_after_s
            s.ptp_lock_offset_threshold_ns = self.ptp_lock_offset_threshold_ns

            # ---- PTP freshness ----
            try:
                ing = time_status.get("ingress_time")
                ingress_time_ns = int(ing) if ing and ing != "0" else None
            except ValueError:
                ingress_time_ns = None

            raw_gm_present = parse_bool(time_status.get("gm_present")) is True
            if ingress_time_ns is not None:
                if ingress_time_ns != self._last_ingress_time_ns:
                    self._last_ingress_time_ns = ingress_time_ns
                    self._last_ingress_change_monotonic = monotonic_now
                ingress_change_age_s = (
                    monotonic_now - self._last_ingress_change_monotonic
                    if self._last_ingress_change_monotonic is not None else None
                )
            else:
                ingress_change_age_s = None
                self._last_ingress_time_ns = None
                self._last_ingress_change_monotonic = None

            ingress_age_s = (
                now - ingress_time_ns * 1e-9
                if ingress_time_ns is not None else None
            )
            ingress_change_age_ok = (
                ingress_change_age_s is not None
                and ingress_change_age_s <= self.ptp_ingress_stale_after_s
            )
            ptp_data_fresh = (
                ingress_time_ns is not None
                and ingress_time_ns > 0
                and port_state == "SLAVE"
                and raw_gm_present
                and ingress_change_age_ok
            )

            # ---- PTP fields ----
            s.ptp_port_state = port_state
            s.ptp_data_fresh = ptp_data_fresh
            s.ptp_ingress_time_ns = ingress_time_ns
            s.ptp_seconds_since_ingress = (
                max(0.0, ingress_age_s)
                if ingress_time_ns is not None else None
            )
            s.ptp_seconds_since_ingress_update = ingress_change_age_s

            fresh_gm_identity = (
                time_status.get("gm_identity") if ptp_data_fresh else ""
            ) or ""
            s.ptp_gm_present = raw_gm_present if ptp_data_fresh else False
            s.ptp_gm_identity = fresh_gm_identity
            if fresh_gm_identity:
                s.ptp_last_gm_identity = fresh_gm_identity
            s.ptp_port_data_set_text = port_data.get("_text", "")
            s.ptp_time_status_np_text = time_status.get("_text", "")
            s.ptp_parent_data_set_text = parent.get("_text", "")
            s.ptp_time_properties_data_set_text = time_props.get("_text", "")
            try:
                ofs = time_status.get("master_offset")
                s.ptp_master_offset_ns = (
                    float(ofs)
                    if ptp_data_fresh and ofs is not None else None
                )
            except ValueError:
                s.ptp_master_offset_ns = None
            try:
                cc = parent.get("gm_clock_class")
                s.ptp_gm_clock_class = (
                    int(cc)
                    if ptp_data_fresh and cc is not None else None
                )
            except ValueError:
                s.ptp_gm_clock_class = None
            s.ptp_gm_clock_accuracy = (
                parent.get("gm_clock_accuracy") if ptp_data_fresh else ""
            ) or ""
            try:
                var = parent.get("gm_offset_scaled_log_variance")
                s.ptp_gm_offset_scaled_log_variance = (
                    int(var, 0)
                    if ptp_data_fresh and var is not None else None
                )
            except ValueError:
                s.ptp_gm_offset_scaled_log_variance = None
            try:
                u = time_props.get("current_utc_offset")
                s.ptp_utc_offset = (
                    int(u)
                    if ptp_data_fresh and u is not None else None
                )
            except ValueError:
                s.ptp_utc_offset = None
            uv = time_props.get("current_utc_offset_valid")
            s.ptp_utc_offset_valid = (
                parse_bool(uv) if ptp_data_fresh and uv is not None else None
            )

            # ---- Chrony fields ----
            s.chrony_ref_id            = tracking.get("ref_id", "")
            s.chrony_ref_name          = tracking.get("ref_name", "")
            s.chrony_stratum           = tracking.get("stratum")
            system_time_s              = tracking.get("system_time_s")
            s.chrony_system_time_s     = system_time_s
            s.chrony_system_time_offset_s = system_time_s
            s.chrony_last_offset_s     = tracking.get("last_offset_s")
            s.chrony_rms_offset_s      = tracking.get("rms_offset_s")
            s.chrony_root_delay_s      = tracking.get("root_delay_s")
            s.chrony_root_dispersion_s = tracking.get("root_dispersion_s")
            s.chrony_root_distance_s   = tracking.get("root_distance_s")
            s.chrony_leap_status       = tracking.get("leap_status", "")
            s.chrony_freq_ppm          = tracking.get("freq_ppm")
            s.chrony_selected_source   = selected
            s.chrony_sources           = chrony_sources
            s.chrony_tracking_text     = tracking_text
            s.chrony_sources_text      = chrony_sources_text
            s.detail_age_s             = detail_age_s
            s.ptp_age_s                = ptp_age_s
            s.system_clock_synced      = clock.get("synced")
            s.system_clock_freq_ppm    = clock.get("freq_ppm")
            s.phc_sys_offset_s         = phc_offset

            # ---- Lock quality tracking ----
            ptp_link_up = (port_state == "SLAVE" and s.ptp_gm_present)
            ptp_quality_good = (
                ptp_link_up
                and s.ptp_master_offset_ns is not None
                and abs(s.ptp_master_offset_ns) < self.ptp_lock_offset_threshold_ns
            )

            s.ptp_link_up = ptp_link_up
            s.ptp_quality_good = ptp_quality_good

            if ptp_quality_good:
                s.consecutive_good_polls += 1
            else:
                s.consecutive_good_polls = 0

            s.ptp_quality_stable = s.consecutive_good_polls >= LOCK_STABLE_POLLS

            # ---- State machine ----
            selected_u = (selected or "").upper()
            ref_name_u = (s.chrony_ref_name or "").upper()
            ref_id_u = (s.chrony_ref_id or "").upper()
            chrony_ref_id_has_source = ref_id_u not in ("", "0", "00000000", "-")
            chrony_has_selected_source = bool(selected)
            chrony_tracking_has_source = (
                bool(s.chrony_ref_name) or chrony_ref_id_has_source
            )
            chrony_leap_normal = s.chrony_leap_status == "Normal"
            chrony_phc_selected = (
                selected_u.startswith("PHC")
                or (
                    chrony_leap_normal
                    and (
                        ref_name_u.startswith("PHC")
                        or ref_id_u in ("PHC0", "50484330")
                    )
                )
            )
            chrony_has_source = (
                chrony_has_selected_source or chrony_tracking_has_source
            )
            chrony_ntp_selected = (
                not chrony_phc_selected
                and chrony_leap_normal
                and chrony_has_source
            )
            if chrony_phc_selected:
                s.chrony_source_type = "phc"
            elif chrony_ntp_selected:
                s.chrony_source_type = "ntp"
            elif chrony_has_source:
                s.chrony_source_type = "unknown"
            else:
                s.chrony_source_type = "none"

            ptp_lock_observed = self._last_ptp_lock_time is not None
            ptp_holdover_s = (
                now - self._last_ptp_lock_time
                if ptp_lock_observed else None
            )
            ptp_holdover_within_window = (
                ptp_holdover_s is not None
                and ptp_holdover_s < self.ptp_holdover_window_s
            )
            ptp_holdover_active = chrony_phc_selected and ptp_lock_observed
            s.ptp_holdover_within_window = False
            s.ptp_holdover_expired = False
            s.ptp_holdover_age_known = False

            if ptp_link_up:
                # Link up - either fully locked or briefly degraded
                s.state = TimingState.LOCKED_TO_GM.value
                self._last_any_sync_time = now
                if ptp_quality_good:
                    self._last_ptp_lock_time = now
                    self._last_ptp_abs_error_s = (
                        abs(s.ptp_master_offset_ns or 0.0) * 1e-9
                    )
                s.estimated_abs_error_s = (
                    abs(s.ptp_master_offset_ns or 1e-6) * 1e-9
                )
            elif ptp_holdover_active:
                s.state = TimingState.PTP_HOLDOVER.value
                if chrony_phc_selected:
                    self._last_any_sync_time = now
                s.ptp_holdover_age_known = True
                s.ptp_holdover_within_window = ptp_holdover_within_window
                s.ptp_holdover_expired = not ptp_holdover_within_window
                drift_s = (
                    ptp_holdover_s * self.ptp_holdover_error_rate_ppm * 1e-6
                )
                s.estimated_abs_error_s = (
                    (self._last_ptp_abs_error_s or 0.0) + drift_s
                )
            elif chrony_phc_selected:
                s.state = TimingState.PHC_FREE_RUN.value
                s.ptp_holdover_expired = True
                s.estimated_abs_error_s = None
            elif chrony_ntp_selected:
                s.state = TimingState.NTP_SYNCED.value
                self._last_any_sync_time = now
                s.estimated_abs_error_s = (
                    s.chrony_root_distance_s or s.chrony_root_dispersion_s
                )
            elif self._last_any_sync_time is not None:
                s.state = TimingState.NTP_HOLDOVER.value
                holdover_s = now - self._last_any_sync_time
                s.estimated_abs_error_s = holdover_s * 1e-6
            elif monitor_uptime_s < self.initializing_grace_s:
                s.state = TimingState.INITIALIZING.value
                s.estimated_abs_error_s = None
            else:
                s.state = TimingState.FREE_RUN.value
                s.estimated_abs_error_s = None

            s.seconds_since_ptp_lock = (
                now - self._last_ptp_lock_time
                if self._last_ptp_lock_time is not None else None
            )
            s.seconds_since_any_sync = (
                now - self._last_any_sync_time
                if self._last_any_sync_time is not None else None
            )
            s.seconds_since_timing_source_sync = s.seconds_since_any_sync
            s.seconds_since_selected_source_seen = s.seconds_since_any_sync
            s.ptp_lock_observed = self._last_ptp_lock_time is not None

            # Firmware timestamp sync requires sustained good PTP lock.
            s.ready_for_firmware_sync = s.ptp_quality_stable

            # TSU strobe status, read from the strobe daemon's status file.
            self._read_strobe_status(s, now)

    def _read_strobe_status(self, s: "Status", now: float) -> None:
        """Populate s.strobe_* from /run/tsu-strobe.status (best-effort).

        Missing file -> strobe_present=False (daemon never ran). Stale or
        running=False -> strobe_healthy=False.
        """
        try:
            with open(STROBE_STATUS_PATH) as f:
                st = json.load(f)
        except FileNotFoundError:
            s.strobe_present = False
            s.strobe_running = False
            s.strobe_healthy = False
            s.strobe_pid = s.strobe_rearms = s.strobe_tsu_sec = None
            s.strobe_status_age_s = None
            return
        except (OSError, ValueError):
            # Unreadable / mid-write garbage: treat as present-but-unhealthy.
            s.strobe_present = True
            s.strobe_running = False
            s.strobe_healthy = False
            return

        s.strobe_present = True
        s.strobe_running = bool(st.get("running"))
        s.strobe_pid = st.get("pid")
        s.strobe_rearms = st.get("rearms")
        s.strobe_tsu_sec = st.get("tsu_sec")
        updated = st.get("updated_unix_s")
        s.strobe_status_age_s = (now - updated) if updated is not None else None
        s.strobe_healthy = bool(
            s.strobe_running
            and s.strobe_status_age_s is not None
            and s.strobe_status_age_s < STROBE_STALE_AFTER_S
        )

    async def get_status_dict(self) -> dict:
        async with self._lock:
            return asdict(self.status)


# ----------------------------------------------------------------------------
# Server (Unix socket, line-delimited JSON)
# ----------------------------------------------------------------------------

async def handle_client(
    monitor: TimingMonitor,
    reader: asyncio.StreamReader,
    writer: asyncio.StreamWriter,
    poll_interval_s: float = POLL_INTERVAL_S,
) -> None:
    peer = writer.get_extra_info("peername") or "?"
    log.info("client connected: %s", peer)

    streaming = False
    stream_task: Optional[asyncio.Task] = None

    async def stream_loop():
        try:
            while True:
                d = await monitor.get_status_dict()
                writer.write((json.dumps({"ok": True, "data": d}) + "\n").encode())
                await writer.drain()
                await asyncio.sleep(poll_interval_s)
        except (ConnectionResetError, BrokenPipeError):
            pass
        except Exception:
            log.exception("stream loop error")

    try:
        while True:
            line = await reader.readline()
            if not line:
                break
            try:
                req = json.loads(line.decode("utf-8", errors="replace"))
            except json.JSONDecodeError as e:
                resp = {"ok": False, "error": f"json decode: {e}"}
                writer.write((json.dumps(resp) + "\n").encode())
                await writer.drain()
                continue

            cmd = req.get("cmd", "")

            if cmd == "status":
                d = await monitor.get_status_dict()
                resp = {"ok": True, "data": d}

            elif cmd == "ping":
                resp = {"ok": True, "data": {"pong": time.time()}}

            elif cmd == "subscribe":
                if not streaming:
                    streaming = True
                    stream_task = asyncio.create_task(stream_loop())
                    resp = {"ok": True, "data": {"subscribed": True}}
                else:
                    resp = {"ok": True, "data": {"already_subscribed": True}}

            else:
                resp = {"ok": False, "error": f"unknown cmd: {cmd!r}"}

            writer.write((json.dumps(resp) + "\n").encode())
            await writer.drain()

    except (ConnectionResetError, BrokenPipeError):
        pass
    except Exception:
        log.exception("client handler error")
    finally:
        if stream_task is not None:
            stream_task.cancel()
            try:
                await stream_task
            except asyncio.CancelledError:
                pass
        writer.close()
        try:
            await writer.wait_closed()
        except Exception:
            pass
        log.info("client disconnected: %s", peer)


# ----------------------------------------------------------------------------
# Main
# ----------------------------------------------------------------------------

async def async_main(
    socket_path: str = SOCKET_PATH,
    poll_interval_s: float = POLL_INTERVAL_S,
    detail_interval_s: float = DETAIL_INTERVAL_S,
    ptp_interval_s: float = PTP_INTERVAL_S,
    ptp_holdover_window_s: float = PTP_HOLDOVER_WINDOW_S,
    ptp_holdover_error_rate_ppm: float = PTP_HOLDOVER_ERROR_RATE_PPM,
    ptp_ingress_stale_after_s: float = PTP_INGRESS_STALE_AFTER_S,
    ptp_lock_offset_threshold_ns: int = PTP_LOCK_OFFSET_THRESHOLD_NS,
    initializing_grace_s: float = INITIALIZING_GRACE_S,
) -> None:
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s: %(message)s",
    )
    set_process_title("timing-monitor")

    monitor = TimingMonitor(
        ptp_holdover_window_s=ptp_holdover_window_s,
        ptp_holdover_error_rate_ppm=ptp_holdover_error_rate_ppm,
        ptp_ingress_stale_after_s=ptp_ingress_stale_after_s,
        ptp_lock_offset_threshold_ns=ptp_lock_offset_threshold_ns,
        initializing_grace_s=initializing_grace_s,
        detail_interval_s=detail_interval_s,
        ptp_interval_s=ptp_interval_s,
    )

    # Bind the Unix socket
    sock_path = Path(socket_path)
    if sock_path.exists():
        try:
            sock_path.unlink()
        except OSError:
            pass

    server = await asyncio.start_unix_server(
        lambda r, w: handle_client(monitor, r, w, poll_interval_s=poll_interval_s),
        path=str(sock_path),
    )
    # Permissive perms during testing; tighten later (group-only access etc.).
    sock_path.chmod(0o666)

    async def poll_loop():
        while True:
            try:
                await monitor.poll_once()
            except Exception:
                log.exception("poll error")
            await asyncio.sleep(poll_interval_s)

    log.info("timing-monitor listening on %s", socket_path)
    await asyncio.gather(
        server.serve_forever(),
        poll_loop(),
    )


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Monitor RFSoC PTP/chrony timing state and expose JSON status."
    )
    parser.add_argument(
        "--socket-path",
        default=SOCKET_PATH,
        help=f"Unix socket path to create (default: {SOCKET_PATH})",
    )
    parser.add_argument(
        "--poll-interval",
        type=float,
        default=POLL_INTERVAL_S,
        help=f"Status poll interval in seconds (default: {POLL_INTERVAL_S})",
    )
    parser.add_argument(
        "--detail-interval",
        type=float,
        default=DETAIL_INTERVAL_S,
        help=(
            "How often to refresh the slow-changing detail (grandmaster parent "
            "data, UTC properties, source listings, tracking text) in seconds. "
            "Every poll always refreshes what the lock and readiness logic "
            f"needs (default: {DETAIL_INTERVAL_S})"
        ),
    )
    parser.add_argument(
        "--ptp-interval",
        type=float,
        default=PTP_INTERVAL_S,
        help=(
            "How often to run the two pmc queries, in seconds -- the detection "
            "latency for losing the grandmaster, and the only subprocesses on "
            f"the regular path (default: {PTP_INTERVAL_S})"
        ),
    )
    parser.add_argument(
        "--ptp-holdover-window",
        type=float,
        default=PTP_HOLDOVER_WINDOW_S,
        help=(
            "Nominal seconds after the last good PTP lock before "
            "ptp_holdover_expired becomes true. If this monitor observed the "
            "lock, PHC-selected chrony holdover can remain in ptp_holdover "
            f"after this (default: {PTP_HOLDOVER_WINDOW_S})"
        ),
    )
    parser.add_argument(
        "--ptp-holdover-error-rate-ppm",
        type=float,
        default=PTP_HOLDOVER_ERROR_RATE_PPM,
        help=(
            "Assumed local timestamp drift during PTP holdover, in ppm, for "
            f"estimated_abs_error_s (default: {PTP_HOLDOVER_ERROR_RATE_PPM})"
        ),
    )
    parser.add_argument(
        "--ptp-ingress-stale-after",
        type=float,
        default=PTP_INGRESS_STALE_AFTER_S,
        help=(
            "Seconds after the latest pmc ingress_time update before PTP data "
            f"is considered stale (default: {PTP_INGRESS_STALE_AFTER_S})"
        ),
    )
    parser.add_argument(
        "--ptp-lock-offset-threshold-ns",
        type=int,
        default=PTP_LOCK_OFFSET_THRESHOLD_NS,
        help=(
            "Absolute master_offset threshold in ns for a good PTP lock "
            f"(default: {PTP_LOCK_OFFSET_THRESHOLD_NS})"
        ),
    )
    parser.add_argument(
        "--initializing-grace",
        type=float,
        default=INITIALIZING_GRACE_S,
        help=(
            "Seconds after timing-monitor startup to report initializing, "
            "rather than free_run, when no usable timing source has been seen "
            f"(default: {INITIALIZING_GRACE_S})"
        ),
    )
    args = parser.parse_args()

    asyncio.run(
        async_main(
            socket_path=args.socket_path,
            poll_interval_s=args.poll_interval,
            detail_interval_s=args.detail_interval,
            ptp_interval_s=args.ptp_interval,
            ptp_holdover_window_s=args.ptp_holdover_window,
            ptp_holdover_error_rate_ppm=args.ptp_holdover_error_rate_ppm,
            ptp_ingress_stale_after_s=args.ptp_ingress_stale_after,
            ptp_lock_offset_threshold_ns=args.ptp_lock_offset_threshold_ns,
            initializing_grace_s=args.initializing_grace,
        )
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
