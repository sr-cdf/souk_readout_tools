"""Read-only following of binary receive_stream files, without hardware."""

import json
from contextlib import nullcontext
import math
import os
import stat
import time


def _positive(value, name):
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f'{name} must be positive and finite') from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f'{name} must be positive and finite')
    return value


def _read_metadata(path, stop_event):
    # receive_stream rewrites the sidecar after inspecting its first frame.
    # A concurrent reader can briefly see empty or incomplete JSON.
    deadline = time.monotonic() + 1.0
    while not stop_event.is_set():
        try:
            with open(path, encoding='utf-8') as file:
                metadata = json.load(file)
            break
        except json.JSONDecodeError as exc:
            if time.monotonic() >= deadline:
                raise ValueError(f'Invalid stream metadata in {path}: {exc}') from exc
            stop_event.wait(0.05)
    else:
        return None
    if not isinstance(metadata, dict):
        raise ValueError('Stream metadata must be a JSON object')
    tones = metadata.get('num_tones')
    if type(tones) is not int or not 0 <= tones <= 2048:
        raise ValueError('Stream metadata num_tones must be an integer from 0 to 2048')
    rate = _positive(metadata.get('sample_rate'), 'metadata sample_rate')
    if metadata.get('format') != '<i4':
        raise ValueError("Stream metadata format must be '<i4' (little-endian int32)")
    frame_bytes = 8 * tones + 40
    if metadata.get('frame_bytes', frame_bytes) != frame_bytes:
        raise ValueError('Stream metadata frame_bytes does not match num_tones')
    metadata['sample_rate'] = rate
    metadata['frame_bytes'] = frame_bytes
    return metadata


def _check_file(file, path, identity, previous_size):
    current = os.fstat(file.fileno())
    try:
        named = os.stat(path)
    except FileNotFoundError as exc:
        raise RuntimeError('Stream file was removed; open the new capture in a new viewer') from exc
    if (named.st_dev, named.st_ino) != identity:
        raise RuntimeError('Stream file was replaced; open the new capture in a new viewer')
    if current.st_size < previous_size or current.st_size < file.tell():
        raise RuntimeError('Stream file was truncated; open the new capture in a new viewer')
    return current.st_size


def follow_stream_file(filename, frame_callback, stop_event, *,
                       update_callback=None, history_seconds=60,
                       poll_interval=0.1):
    """Read the latest history and follow newly appended complete IQ frames.

    The file is the binary capture basename, accompanied by ``filename +
    '.json'``. It contains fixed-size payloads, without TCP length prefixes.
    ``frame_callback(payload_bytes, metadata)`` receives only complete records.
    Metadata includes ``sample_index_offset``, the number of saved records
    skipped at startup. With ``update_callback``, subscribed typed records
    from ``filename + '.updates.jsonl'`` are delivered before their IQ frames.
    Polling and draining stop when ``stop_event`` is set.

    EOF waits for more data; partial trailing records wait for completion.
    Truncation or replacement raises rather than combining two captures.
    All files are opened read-only. No client object or socket is needed.
    """
    if not callable(frame_callback):
        raise TypeError('frame_callback must be callable')
    if update_callback is not None and not callable(update_callback):
        raise TypeError('update_callback must be callable or None')
    history_seconds = _positive(history_seconds, 'history_seconds')
    poll_interval = _positive(poll_interval, 'poll_interval')
    if stop_event.is_set():
        return
    path = os.fspath(filename)
    metadata_path = path + '.json'
    metadata = _read_metadata(metadata_path, stop_event)
    if metadata is None:
        return
    with open(path, 'rb', buffering=0) as file:
        initial = os.fstat(file.fileno())
        if not stat.S_ISREG(initial.st_mode):
            raise ValueError('Stream file must be a regular binary capture file')
        identity = (initial.st_dev, initial.st_ino)
        size = _check_file(file, path, identity, initial.st_size)
        # An observer may attach before the writer's first frame. Reload the
        # sidecar once bytes arrive, after receive_stream has corrected its
        # initially inferred tone count from the real frame shape.
        if size == 0:
            while size == 0 and not stop_event.wait(poll_interval):
                size = _check_file(file, path, identity, size)
            if stop_event.is_set():
                return
            metadata = _read_metadata(metadata_path, stop_event)
            if metadata is None:
                return
        slots = history_seconds * metadata['sample_rate']
        if not math.isfinite(slots):
            raise ValueError('history_seconds * sample_rate is too large')
        frame_bytes = metadata['frame_bytes']
        available = size // frame_bytes
        skip = max(0, available - max(1, math.ceil(slots)))
        metadata['sample_index_offset'] = skip
        file.seek(skip * frame_bytes)
        pending = bytearray()
        chunk_bytes = max(1, (1024 * 1024) // frame_bytes) * frame_bytes
        updates_path = path + '.updates.jsonl'
        updates_reader = (open(updates_path, encoding='utf-8')
                          if metadata.get('tone_updates_subscribed') and update_callback
                          else nullcontext(None))
        with updates_reader as updates:
            next_update = None
            record_index = skip

            def forward_updates(before_sample):
                nonlocal next_update
                if updates is None:
                    return
                while True:
                    if next_update is None:
                        position = updates.tell()
                        line = updates.readline()
                        if not line:
                            return
                        if not line.endswith('\n'):
                            updates.seek(position)
                            return
                        next_update = json.loads(line)
                    if int(next_update['sample_index']) > before_sample:
                        return
                    record = dict(next_update)
                    record['sample_index'] = int(record['sample_index']) - skip
                    update_callback(record)
                    next_update = None

            forward_updates(skip)
            while not stop_event.is_set():
                size = _check_file(file, path, identity, size)
                chunk = file.read(chunk_bytes)
                if not chunk:
                    forward_updates(record_index)
                    stop_event.wait(poll_interval)
                    continue
                pending.extend(chunk)
                end = len(pending) // frame_bytes * frame_bytes
                for offset in range(0, end, frame_bytes):
                    if stop_event.is_set():
                        return
                    forward_updates(record_index)
                    frame_callback(bytes(pending[offset:offset + frame_bytes]), metadata)
                    record_index += 1
                del pending[:end]
