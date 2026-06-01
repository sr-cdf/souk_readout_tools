"""Helpers for querying the local SOUK timing monitor."""
from __future__ import annotations

import json
import socket
import time
from typing import Any


DEFAULT_TIMING_SOCKET = "/run/timing-monitor.sock"


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
        }
    return timing_public_view(status)
