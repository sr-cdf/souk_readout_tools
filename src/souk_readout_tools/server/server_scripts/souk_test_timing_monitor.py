#!/usr/bin/env python3
"""Small lab client for checking the live timing-monitor socket."""
from __future__ import annotations

import argparse
import asyncio
import json
from typing import Any


DEFAULT_SOCKET_PATH = "/run/timing-monitor.sock"


async def request(socket_path: str, cmd: str) -> dict[str, Any]:
    reader, writer = await asyncio.open_unix_connection(socket_path)
    writer.write((json.dumps({"cmd": cmd}) + "\n").encode())
    await writer.drain()
    line = await reader.readline()
    writer.close()
    await writer.wait_closed()
    if not line:
        raise RuntimeError("timing-monitor closed the connection without a reply")
    return json.loads(line)


def _fmt(value: Any, width: int = 8) -> str:
    text = "None" if value is None else str(value)
    return f"{text:>{width}}"


def _format_stream_line(status: dict[str, Any]) -> str:
    state = str(status.get("state") or "?")
    chrony_type = str(status.get("chrony_source_type") or "?")
    chrony_selected = str(status.get("chrony_selected_source") or "")
    ready = status.get("ready_for_firmware_sync")
    return (
        f"[{state:14}] "
        f"fresh={str(status.get('ptp_data_fresh')):5} "
        f"link={str(status.get('ptp_link_up')):5} "
        f"qual={str(status.get('ptp_quality_good')):5} "
        f"stable={str(status.get('ptp_quality_stable')):5} "
        f"good={_fmt(status.get('consecutive_good_polls'), 3)} "
        f"offset_ns={_fmt(status.get('ptp_master_offset_ns'), 9)} "
        f"ingress_age={_fmt(status.get('ptp_seconds_since_ingress_update'), 6)} "
        f"chrony={chrony_type:>7}:{chrony_selected:>8} "
        f"last_s={_fmt(status.get('chrony_last_offset_s'), 9)} "
        f"err_s={status.get('estimated_abs_error_s')} "
        f"ready={ready}"
    )


async def stream(socket_path: str, count: int) -> None:
    reader, writer = await asyncio.open_unix_connection(socket_path)
    writer.write((json.dumps({"cmd": "subscribe"}) + "\n").encode())
    await writer.drain()

    ack = await reader.readline()
    if not ack:
        raise RuntimeError("timing-monitor closed the connection before subscribe ack")
    ack_obj = json.loads(ack)
    if not ack_obj.get("ok"):
        raise RuntimeError(ack_obj.get("error", "subscribe failed"))

    try:
        remaining = count
        while count == 0 or remaining > 0:
            line = await reader.readline()
            if not line:
                break
            response = json.loads(line)
            if response.get("ok"):
                print(_format_stream_line(response.get("data", {})), flush=True)
            else:
                print(json.dumps(response), flush=True)
            if count > 0:
                remaining -= 1
    finally:
        writer.close()
        await writer.wait_closed()


async def async_main(args: argparse.Namespace) -> None:
    if args.cmd == "stream":
        count = args.count if args.count is not None else args.stream_count
        await stream(args.socket_path, count)
        return

    response = await request(args.socket_path, args.cmd)
    print(json.dumps(response, indent=2, sort_keys=True))


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Smoke-test the live SOUK timing-monitor Unix socket."
    )
    parser.add_argument(
        "cmd",
        nargs="?",
        default="status",
        choices=("status", "ping", "stream"),
        help="Monitor command to send (default: status)",
    )
    parser.add_argument(
        "count",
        nargs="?",
        type=int,
        help="Number of streamed status lines; stream forever with 0",
    )
    parser.add_argument(
        "-n",
        "--stream-count",
        type=int,
        default=10,
        help="Default number of status lines for stream mode (default: 10)",
    )
    parser.add_argument(
        "--socket-path",
        default=DEFAULT_SOCKET_PATH,
        help=f"Unix socket path (default: {DEFAULT_SOCKET_PATH})",
    )
    args = parser.parse_args()
    if args.count is not None and args.count < 0:
        parser.error("count must be >= 0")
    if args.stream_count < 0:
        parser.error("--stream-count must be >= 0")

    asyncio.run(async_main(args))


if __name__ == "__main__":
    main()
