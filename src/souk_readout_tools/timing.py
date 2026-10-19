"""Helpers for querying the local SOUK timing monitor."""
from __future__ import annotations

import json
import socket
import time
import re
from datetime import datetime, timezone
from typing import Any


DEFAULT_TIMING_SOCKET = "/run/timing-monitor.sock"

# Default tolerance (seconds) for the firmware-TT "aligned on the second boundary"
# verdict: |pps_boundary_offset_s| under this counts as aligned. 0.1 ms; a fresh TT
# load lands ~1 us off. One source of truth for timed_sync_check / timed_sync_needed /
# set_telescope_time so the threshold stays consistent across them.
DEFAULT_ALIGN_TOL_S = 1e-4


def unix_to_iso(unix_s: float | None) -> str | None:
    """Format a UNIX timestamp (seconds since the epoch) as an ISO-8601 UTC string.

    e.g. ``1782206913.0 -> '2026-06-23T09:28:33.000Z'``. UTC is used because these
    timestamps are absolute epochs meant to line up across boards. Returns ``None``
    for a ``None`` input so it can wrap optional fields directly.
    """
    if unix_s is None:
        return None
    return datetime.fromtimestamp(unix_s, tz=timezone.utc).strftime(
        "%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def format_duration_s(seconds: float | None) -> str | None:
    """Human-readable duration for a span in seconds, e.g. '42 s', '5.3 min', '2.1 h',
    '3.4 d'. Returns None for None (so it can wrap optional fields directly)."""
    if seconds is None:
        return None
    s = abs(seconds)
    if s < 90:
        text = f"{s:.0f} s"
    elif s < 5400:
        text = f"{s / 60:.1f} min"
    elif s < 172800:
        text = f"{s / 3600:.1f} h"
    else:
        text = f"{s / 86400:.1f} d"
    return ("-" + text) if seconds < 0 else text


class TimingStatusError(RuntimeError):
    """Raised when the local timing monitor cannot return a valid status."""


def unavailable_timing_status(error: str) -> dict[str, Any]:
    """Return a structured timing status when the monitor is unavailable.

    ``error`` is the failure message embedded in the returned status dict.
    """
    return {
        "ready": False,
        "available": False,
        "state": "unavailable",
        "timestamp": time.time(),
        "error": error,
        "ready_for_firmware_sync": False,
    }


def get_timing_status(
    socket_path: str = DEFAULT_TIMING_SOCKET,
    timeout_s: float = 0.5,
    raise_on_error: bool = False,
) -> dict[str, Any]:
    """Query the timing-monitor Unix socket and return its status object.

    When ``raise_on_error`` is false, connection/protocol failures are returned
    as a normal structured status so callers can include timing in broad status
    reports without making the readout server depend on the timing service.

    Parameters
    ----------
    socket_path : str, optional
        Path to the timing-monitor Unix socket (default
        :data:`DEFAULT_TIMING_SOCKET`).
    timeout_s : float, optional
        Socket connect/recv timeout in seconds (default ``0.5``).
    raise_on_error : bool, optional
        Raise :class:`TimingStatusError` on failure instead of returning an
        ``unavailable`` status (default ``False``).
    """
    try:
        with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout_s)
            sock.connect(socket_path)
            sock.sendall(b'{"cmd":"status"}\n')
            chunks = []
            while True:
                chunk = sock.recv(4096)
                if not chunk:
                    break
                chunks.append(chunk)
                if b"\n" in chunk:
                    break
    except OSError as exc:
        if raise_on_error:
            raise TimingStatusError(str(exc)) from exc
        return unavailable_timing_status(str(exc))

    raw = b"".join(chunks).split(b"\n", 1)[0]
    try:
        response = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        if raise_on_error:
            raise TimingStatusError(f"invalid monitor response: {exc}") from exc
        return unavailable_timing_status(f"invalid monitor response: {exc}")

    if not response.get("ok"):
        error = response.get("error", "timing monitor returned an error")
        if raise_on_error:
            raise TimingStatusError(error)
        return unavailable_timing_status(error)

    data = response.get("data")
    if not isinstance(data, dict):
        error = "timing monitor response did not contain a status object"
        if raise_on_error:
            raise TimingStatusError(error)
        return unavailable_timing_status(error)

    data.setdefault("ready", True)
    data.setdefault("available", True)
    data.setdefault("ready_for_firmware_sync", False)
    return data


def _state_summary(state: str) -> str:
    summaries = {
        "locked_to_gm": "PTP is locked to a fresh grandmaster.",
        "ptp_holdover": "PTP was locked; local PHC holdover is active.",
        "phc_free_run": (
            "The PHC is selected, but no PTP lock has been observed by this "
            "monitor."
        ),
        "ntp_synced": "The system clock is synchronised from NTP.",
        "ntp_holdover": "No source is currently selected; previous sync is aging.",
        "free_run": "No usable timing source has been seen.",
        "initializing": "Timing monitor is still initializing.",
        "unavailable": "Timing monitor is unavailable.",
    }
    return summaries.get(state, "Timing state is unknown.")


def _source_label(source: dict[str, Any]) -> str:
    name = source.get("name") or source.get("configured_name") or ""
    address = source.get("address")
    if address and address != name:
        return f"{name} ({address})"
    return name


def _float_or_none(value: Any) -> float | None:
    try:
        if value is None:
            return None
        return float(value)
    except (TypeError, ValueError):
        return None


def _abs_float_or_none(value: Any) -> float | None:
    parsed = _float_or_none(value)
    return abs(parsed) if parsed is not None else None


def _best_source_by_error(sources: list[dict[str, Any]]) -> dict[str, Any] | None:
    candidates = [
        source for source in sources
        if source.get("usable")
        and _float_or_none(source.get("estimated_error_s")) is not None
    ]
    if not candidates:
        return None
    return min(
        candidates,
        key=lambda source: abs(_float_or_none(source.get("estimated_error_s")) or 0.0),
    )


def timing_public_view(status: dict[str, Any]) -> dict[str, Any]:
    """Return the user-facing timing view for ``get_info('timing')``.

    ``status`` is a raw status dict from :func:`get_timing_status`.
    """
    state = status.get("state", "unknown")
    source_type = status.get("chrony_source_type", "unknown")
    chrony_sources = status.get("chrony_sources") or []
    selected_source = next(
        (source for source in chrony_sources if source.get("selected")),
        None,
    )
    ntp_sources = [source for source in chrony_sources if source.get("mode") == "ntp"]
    ntp_selected = next(
        (source for source in ntp_sources if source.get("selected")),
        None,
    )
    ntp_best = _best_source_by_error(ntp_sources)

    phc_source = next(
        (source for source in chrony_sources
         if source.get("mode") == "refclock"
         and str(source.get("name", "")).upper().startswith("PHC")),
        None,
    )

    absolute_time_verified = state in ("locked_to_gm", "ptp_holdover", "ntp_synced")
    chrony_synced = status.get("chrony_leap_status") == "Normal"
    system_synced = chrony_synced and state in ("locked_to_gm", "ntp_synced")
    ready_for_firmware_sync = status.get("ready_for_firmware_sync", False)
    estimated_abs_error_s = status.get("estimated_abs_error_s")
    holdover_error_rate_ppm = status.get("ptp_holdover_error_rate_ppm")
    system_time_offset_s = status.get(
        "chrony_system_time_offset_s",
        status.get("chrony_system_time_s"),
    )
    selected_source_error_s = (
        _abs_float_or_none(selected_source.get("estimated_error_s"))
        if selected_source else None
    )
    selected_source_name = (
        _source_label(selected_source) if selected_source else
        status.get("chrony_selected_source", "") or status.get("chrony_ref_name", "")
    )
    ntp_best_error_s = (
        _abs_float_or_none(ntp_best.get("estimated_error_s"))
        if ntp_best else None
    )
    ntp_selected_error_s = (
        _abs_float_or_none(ntp_selected.get("estimated_error_s"))
        if ntp_selected else None
    )
    holdover_time_to_ntp_error_s = None
    if (
        state == "ptp_holdover"
        and ntp_best_error_s is not None
        and _float_or_none(estimated_abs_error_s) is not None
        and _float_or_none(holdover_error_rate_ppm) is not None
        and float(holdover_error_rate_ppm) > 0.0
    ):
        remaining_error_s = ntp_best_error_s - float(estimated_abs_error_s)
        holdover_time_to_ntp_error_s = max(
            0.0,
            remaining_error_s / (float(holdover_error_rate_ppm) * 1e-6),
        )

    return {
        "summary": {
            "state": state,
            "condition": _state_summary(state),
            "updated_unix_s": status.get("timestamp"),
            "ready_for_firmware_sync": ready_for_firmware_sync,
            "strobe_healthy": status.get("strobe_healthy", False),
            "absolute_time_verified": absolute_time_verified,
            "estimated_abs_error_s": estimated_abs_error_s,
            "active_source_type": source_type,
            "active_source_name": selected_source_name,
            "active_source_state": (
                selected_source.get("state") if selected_source else None
            ),
            "active_source_offset_s": (
                selected_source.get("offset_s") if selected_source else None
            ),
            "active_source_error_s": selected_source_error_s,
            "ntp_selected_source": (
                _source_label(ntp_selected) if ntp_selected else ""
            ),
            "ntp_selected_error_s": ntp_selected_error_s,
            "ntp_best_source": _source_label(ntp_best) if ntp_best else "",
            "ntp_best_error_s": ntp_best_error_s,
            "system_synced": system_synced,
            "system_time_offset_s": system_time_offset_s,
            "last_offset_s": status.get("chrony_last_offset_s"),
            "rms_offset_s": status.get("chrony_rms_offset_s"),
            "root_distance_s": status.get("chrony_root_distance_s"),
            "ptp_healthy": status.get("ptp_link_up", False),
            "ptp_fresh": status.get("ptp_data_fresh", False),
            "ptp_stable": status.get("ptp_quality_stable", False),
            "ptp_gm_identity": status.get("ptp_gm_identity", ""),
            "ptp_last_gm_identity": status.get("ptp_last_gm_identity", ""),
            "ptp_master_offset_ns": status.get("ptp_master_offset_ns"),
            "ptp_ingress_age_s": status.get("ptp_seconds_since_ingress_update"),
            "ptp_holdover_age_s": status.get("seconds_since_ptp_lock"),
            "ptp_holdover_error_rate_ppm": holdover_error_rate_ppm,
            "ptp_holdover_window_s": status.get("ptp_holdover_window_s"),
            "ptp_holdover_expired": status.get("ptp_holdover_expired", False),
            "holdover_estimated_seconds_until_ntp_error": (
                holdover_time_to_ntp_error_s
            ),
        },
        "monitor": {
            "available": status.get("available", True),
            "updated_unix_s": status.get("timestamp"),
            "poll_count": status.get("poll_count"),
            "uptime_s": status.get("monitor_uptime_s"),
            "initializing_grace_s": status.get("initializing_grace_s"),
            "ptp_holdover_error_rate_ppm": holdover_error_rate_ppm,
            "ptp_holdover_window_s": status.get("ptp_holdover_window_s"),
            "ptp_ingress_stale_after_s": status.get("ptp_ingress_stale_after_s"),
            "ptp_lock_offset_threshold_ns": status.get(
                "ptp_lock_offset_threshold_ns"
            ),
        },
        "ptp": {
            "healthy": status.get("ptp_link_up", False),
            "fresh": status.get("ptp_data_fresh", False),
            "stable": status.get("ptp_quality_stable", False),
            "port_state": status.get("ptp_port_state"),
            "gm_present": status.get("ptp_gm_present", False),
            "gm_identity": status.get("ptp_gm_identity", ""),
            "last_gm_identity": status.get("ptp_last_gm_identity", ""),
            "master_offset_ns": status.get("ptp_master_offset_ns"),
            "ingress_age_s": status.get("ptp_seconds_since_ingress_update"),
            "ingress_stale_after_s": status.get("ptp_ingress_stale_after_s"),
            "seconds_since_lock": status.get("seconds_since_ptp_lock"),
            "pmc": {
                "port_data_set": status.get("ptp_port_data_set_text", ""),
                "time_status_np": status.get("ptp_time_status_np_text", ""),
                "parent_data_set": status.get("ptp_parent_data_set_text", ""),
                "time_properties_data_set": status.get(
                    "ptp_time_properties_data_set_text",
                    "",
                ),
            },
        },
        "phc": {
            "selected": source_type == "phc",
            "source_state": phc_source.get("state") if phc_source else None,
            "source_offset_s": phc_source.get("offset_s") if phc_source else None,
            "source_error_s": (
                phc_source.get("estimated_error_s") if phc_source else None
            ),
            "ptp_lock_observed": status.get("ptp_lock_observed", False),
            "holdover_expired": status.get("ptp_holdover_expired", False),
            "holdover_window_s": status.get("ptp_holdover_window_s"),
            "holdover_error_rate_ppm": holdover_error_rate_ppm,
            "estimated_seconds_until_ntp_error": holdover_time_to_ntp_error_s,
        },
        "ntp": {
            "selected": source_type == "ntp",
            "selected_source": _source_label(ntp_selected) if ntp_selected else "",
            "selected_address": (
                ntp_selected.get("address") if ntp_selected else None
            ),
            "selected_source_error_s": ntp_selected_error_s,
            "best_source": _source_label(ntp_best) if ntp_best else "",
            "best_source_error_s": ntp_best_error_s,
            "best_source_offset_s": (
                ntp_best.get("offset_s") if ntp_best else None
            ),
            "root_distance_s": (
                status.get("chrony_root_distance_s")
                if source_type == "ntp" else None
            ),
            "source_count": len(ntp_sources),
            "usable_source_count": sum(
                1 for source in ntp_sources if source.get("usable")
            ),
        },
        "chrony": {
            "synced": chrony_synced,
            "leap_status": status.get("chrony_leap_status", ""),
            "stratum": status.get("chrony_stratum"),
            "source_type": source_type,
            "selected_source": status.get("chrony_selected_source", ""),
            "system_time_offset_s": system_time_offset_s,
            "last_offset_s": status.get("chrony_last_offset_s"),
            "rms_offset_s": status.get("chrony_rms_offset_s"),
            "root_distance_s": status.get("chrony_root_distance_s"),
            "frequency_ppm": status.get("chrony_freq_ppm"),
            "seconds_since_selected_source_seen": status.get(
                "seconds_since_selected_source_seen",
                status.get(
                    "seconds_since_timing_source_sync",
                    status.get("seconds_since_any_sync"),
                ),
            ),
            "tracking": status.get("chrony_tracking_text", ""),
            "sources": status.get("chrony_sources_text", ""),
        },
        "strobe": {
            # TSU 1-PPS strobe daemon status (from /run/tsu-strobe.status, read by
            # the timing monitor). present=False -> daemon never ran.
            "present": status.get("strobe_present", False),
            "running": status.get("strobe_running", False),
            "healthy": status.get("strobe_healthy", False),
            "pid": status.get("strobe_pid"),
            "rearms": status.get("strobe_rearms"),
            "tsu_sec": status.get("strobe_tsu_sec"),
            "status_age_s": status.get("strobe_status_age_s"),
        },
    }


def get_timing_summary(
    socket_path: str = DEFAULT_TIMING_SOCKET,
    timeout_s: float = 0.5,
) -> dict[str, Any]:
    """Return the public timing view used by ``get_info('timing')``.

    ``socket_path`` and ``timeout_s`` are forwarded to
    :func:`get_timing_status`.
    """
    status = get_timing_status(socket_path=socket_path, timeout_s=timeout_s)
    if not status.get("available", True):
        return {
            "summary": {
                "state": status.get("state", "unavailable"),
                "condition": status.get("error", "Timing monitor unavailable."),
                "updated_unix_s": status.get("timestamp"),
                "ready_for_firmware_sync": False,
                "strobe_healthy": False,
                "absolute_time_verified": False,
                "estimated_abs_error_s": None,
                "active_source_type": "none",
                "active_source_name": "",
                "active_source_state": None,
                "active_source_offset_s": None,
                "active_source_error_s": None,
                "ntp_selected_source": "",
                "ntp_selected_error_s": None,
                "ntp_best_source": "",
                "ntp_best_error_s": None,
                "system_synced": False,
                "system_time_offset_s": None,
                "last_offset_s": None,
                "rms_offset_s": None,
                "root_distance_s": None,
                "ptp_healthy": False,
                "ptp_fresh": False,
                "ptp_stable": False,
                "ptp_gm_identity": "",
                "ptp_last_gm_identity": "",
                "ptp_master_offset_ns": None,
                "ptp_ingress_age_s": None,
                "ptp_holdover_age_s": None,
                "ptp_holdover_error_rate_ppm": None,
                "ptp_holdover_window_s": None,
                "ptp_holdover_expired": None,
                "holdover_estimated_seconds_until_ntp_error": None,
            },
            "monitor": {
                "available": False,
                "updated_unix_s": status.get("timestamp"),
                "error": status.get("error"),
                "ptp_holdover_error_rate_ppm": None,
                "ptp_holdover_window_s": None,
                "ptp_ingress_stale_after_s": None,
                "ptp_lock_offset_threshold_ns": None,
            },
            "ptp": {},
            "phc": {},
            "ntp": {},
            "chrony": {},
            "strobe": {"present": False, "running": False, "healthy": False},
        }
    return timing_public_view(status)


def report_sync_readiness(summary: dict[str, Any]) -> dict[str, Any]:
    """Decide whether the system is ready for a firmware *timed* sync.

    Pure function over the ``summary`` sub-dict from :func:`get_timing_summary`
    (i.e. ``get_timing_summary()['summary']``). It folds the timing-monitor view
    into the two preconditions a timed sync needs, plus an overall verdict and
    human-readable reasons for anything not ready. This is the readout-tools port of
    ``scripts/timed_sync/05_check_timing_system.py``'s gate, reused by the
    ``timed_sync_*`` server handlers so the readiness logic lives in one place.

    Preconditions:
      ptp_ready  -- PTP is locked to a grandmaster (``state == 'locked_to_gm'``) and
                    the monitor reports ``ready_for_firmware_sync`` (stable lock, and
                    chrony disciplining Linux time from the PHC). Without it, the
                    telescope time the firmware disciplines to may be wrong.
      pps_active -- the TSU 1-PPS strobe is healthy. Without a PPS the firmware has
                    nothing to align its telescope time to (the load latches on a PPS
                    edge), and ``update_internal_time`` would block/fail.

    :param summary: the ``summary`` dict from :func:`get_timing_summary`.
    :return: dict with ``ptp_ready``, ``pps_active``, ``can_sync`` (both true), and
        ``reasons`` (human strings; empty when ``can_sync`` is True).
    """
    state = summary.get("state", "unknown")
    ptp_ready = bool(summary.get("ready_for_firmware_sync")) and state == "locked_to_gm"
    pps_active = bool(summary.get("strobe_healthy"))

    reasons = []
    if not ptp_ready and state == "locked_to_gm" and summary.get("active_source_type") != "phc":
        reasons.append(
            "PTP is locked but chrony is not using the PHC (Linux time is on "
            f"{summary.get('active_source_type')} {summary.get('active_source_name')!r}); "
            "on the board run `sudo chronyc reset sources` and wait for '#* PHC0' "
            "in `chronyc sources`."
        )
    elif not ptp_ready:
        reasons.append(
            f"PTP not ready for a firmware sync (state={state}, "
            f"ready_for_firmware_sync={summary.get('ready_for_firmware_sync')}); "
            "the telescope time may be wrong."
        )
    if not pps_active:
        reasons.append(
            "TSU 1-PPS strobe is not healthy (systemd: souk-tsu-strobe); the firmware "
            "has no PPS to align telescope time to."
        )

    return {
        "ptp_ready": ptp_ready,
        "pps_active": pps_active,
        "can_sync": ptp_ready and pps_active,
        "reasons": reasons,
    }


def format_timing_status(status: dict[str, Any]) -> str:
    """Render ``get_info('timing')`` as a short human-readable report.

    The raw dict is ~60 keys across six sub-dicts, which buries the handful of
    facts a user actually needs: is it locked, is absolute time trustworthy, and
    what timescale are the frame timestamps on.

    The TAI line is the one that surprises people. PTP runs on TAI
    (``ptpTimescale 1``), so the PHC -- and the TSU strobe's ``tsu_sec`` that
    follows it -- sit ``currentUtcOffset`` seconds ahead of UTC (37 s at
    present). chrony's PHC0 refclock applies ``offset -37`` so that the system
    clock is UTC; the PHC and ``tsu_sec`` stay on TAI. A refclock without that
    offset sits 37 s from the UTC-based NTP servers, chrony marks PHC0 a
    falseticker, and a board genuinely locked to a grandmaster reports ``ntp``
    as the active source.

    ``telescope_time`` in the frame headers is a **different clock** and is not
    covered by that offset: it is the firmware TT counter, in fabric-clock ticks
    at ``adc_clk_hz / 8`` (307.2 MHz), free-running from whenever it was last
    loaded. It only represents wall-clock time once ``set_telescope_time`` has
    latched the system clock into it on a PPS edge -- and the system clock is
    UTC, not TAI.

    Parameters
    ----------
    status : dict
        The dict returned by ``client.get_info('timing')``.

    Returns
    -------
    str
        Multi-line report. ``print()`` it.

    Examples
    --------
    >>> print(format_timing_status(client.get_info('timing')))   # doctest: +SKIP
    """
    s = status.get("summary", {}) or {}
    ptp = status.get("ptp", {}) or {}
    phc = status.get("phc", {}) or {}
    strobe = status.get("strobe", {}) or {}
    ready = status.get("sync_readiness", {}) or {}

    def _ns(v):
        return "-" if v is None else f"{v * 1e9:,.0f} ns"

    lines = [
        f"state       : {s.get('state')}  -- {s.get('condition')}",
        f"can sync    : {ready.get('can_sync')}   ptp_ready={ready.get('ptp_ready')}"
        f"  pps_active={ready.get('pps_active')}"
        + (f"  reasons={ready['reasons']}" if ready.get("reasons") else ""),
        "",
        f"PTP         : {ptp.get('port_state')}  gm={ptp.get('gm_identity')}"
        f"  offset={ptp.get('master_offset_ns')} ns"
        f"  healthy={ptp.get('healthy')} fresh={ptp.get('fresh')} stable={ptp.get('stable')}",
        f"absolute    : verified={s.get('absolute_time_verified')}"
        f"  err={_ns(s.get('estimated_abs_error_s'))}"
        f"  via {s.get('active_source_type')} ({s.get('active_source_name')})",
        f"strobe      : running={strobe.get('running')} healthy={strobe.get('healthy')}"
        f"  rearms={strobe.get('rearms')}  tsu_sec={strobe.get('tsu_sec')}",
    ]

    # The TAI/UTC discrepancy, stated rather than left to be rediscovered.
    #
    # Key this off the TSU, NOT off chrony's PHC offset. The `offset -37` on
    # chrony's PHC0 refclock fixes the *system* clock but leaves the PHC -- and
    # so the TSU, and so tsu_sec -- on TAI. Keying off the chrony offset would
    # make this warning vanish exactly when it is still needed.
    utc_offset, ptp_timescale = _ptp_utc_offset(ptp)
    tsu, now = strobe.get("tsu_sec"), s.get("updated_unix_s")
    skew = (tsu - now) if (tsu is not None and now is not None) else None
    if skew is not None and abs(skew) > 1.0:
        n = utc_offset if utc_offset is not None else round(skew)
        lines += [
            "",
            f"timescale   : the PHC/TSU runs on TAI, ~{n} s ahead of UTC"
            + ("  (ptpTimescale=1)" if ptp_timescale else "")
            + f"  [measured skew {skew:+.0f} s]",
            f"              Subtract {n} from strobe tsu_sec for UTC. NOTE"
            " telescope_time is a different clock: firmware TT in fabric ticks,",
            "              free-running until set_telescope_time loads the"
            " (UTC) system clock into it.",
        ]
    return "\n".join(lines)


def _ptp_utc_offset(ptp: dict[str, Any]) -> tuple[int | None, bool]:
    """Pull ``currentUtcOffset`` and ``ptpTimescale`` out of the pmc text blob.

    ``timing_public_view`` passes the raw ``pmc`` output through as strings, so
    these two authoritative numbers are only available by parsing them back out.
    Returns ``(None, False)`` if the blob is missing or unparsable.
    """
    text = (ptp.get("pmc") or {}).get("time_properties_data_set") or ""
    m = re.search(r"currentUtcOffset\s+(-?\d+)", text)
    offset = int(m.group(1)) if m else None
    scale = bool(re.search(r"ptpTimescale\s+1", text))
    return offset, scale
