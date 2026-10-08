"""
Read large stream captures in pieces.

``ReadoutClient.parse_stream`` loads a whole capture into memory. A 12-hour
capture of 400 tones at 2500 samples/s is ~350 GB, so instead:

- :func:`stream_file_info` -- how many samples, tones, seconds and bytes a
  capture holds, without reading its data;
- :func:`read_stream_chunk` -- a sample range, with only the tones and header
  fields asked for, in the same parsed format as ``parse_stream``;
- :func:`concatenate_stream_chunks` -- join chunks back into one;
- :func:`stream_time_model` -- the real time of every sample, and whether the
  recorded telescope time can be trusted (it often cannot);
- :func:`find_sample` -- the sample at a given time, in seconds on any size;
- :func:`sample_times` -- the real time of each sample of a chunk.

For example, look at the first ten minutes of tone 1, then add two more
pieces::

    from souk_readout_tools import stream_file
    layout = stream_file.stream_file_info('run1')
    rate = layout['sample_rate']
    first = stream_file.read_stream_chunk('run1', 0, int(600 * rate), tones=1)
    more = [stream_file.read_stream_chunk('run1', int(a * rate), int(b * rate), tones=1)
            for a, b in [(3600, 4200), (7200, 7800)]]
    joined = stream_file.concatenate_stream_chunks([first] + more)

or read ten minutes from 02:00 UTC, with real times::

    model = stream_file.stream_time_model('run1')       # warns if the TT is wrong
    start = stream_file.find_sample('run1', '2026-09-30T02:00:00Z', model)
    stop = stream_file.find_sample('run1', '2026-09-30T02:10:00Z', model)
    chunk = stream_file.read_stream_chunk('run1', start, stop, tones=1)
    t = stream_file.sample_times(chunk, model)

The file is read front to back in blocks, which is what a spinning disk is
fast at, and a progress line shows the rate and time left. Ctrl+C stops it
between blocks. A frame is ~3 kB and a disk page 4 kB, so the disk has to
read every frame in the range even for one tone: the time depends on the
length of the range, not on the number of tones.

Every chunk carries ``sample_index`` (the absolute sample number in the
capture) and ``time_s`` (``sample_index / sample_rate``), and the tone-update
records keep absolute ``sample_index`` values too, so chunks can be joined,
and ``modulation.demodulate_stream`` can follow the tracking revisions
through a chunk.
"""

import copy
import json
import os
import time
import warnings

import numpy as np

# Header fields a chunk can carry, as named in parse_stream's output.
HEADER_FIELDS = ('packet_counter', 'packet_error', 'telescope_time', 'stream_flags',
                 'modulation_point', 'modulation_settling', 'modulation_revision')

# The per-sample entries of a parsed chunk: arrays, and dicts of arrays.
_PER_SAMPLE_ARRAYS = (HEADER_FIELDS[:3] + HEADER_FIELDS[4:]
                      + ('sample_index', 'time_s', 'adc_overvoltage'))
_PER_SAMPLE_DICTS = ('i_data', 'q_data', 'stream_flags')

# Refuse a read that would need more than this fraction of the free memory.
MEMORY_FRACTION = 0.8


def stream_file_info(filename):
    """Size and layout of a stream capture, without reading its data.

    Parameters
    ----------
    filename : str or os.PathLike
        The capture basename, as returned by ``receive_stream`` (the binary
        file; its ``.json`` metadata sits beside it).

    Returns
    -------
    dict
        ``num_samples`` (complete frames in the file), ``num_tones``,
        ``sample_rate``, ``duration_s``, ``frame_bytes``, ``file_bytes``,
        ``trailing_bytes`` (a partial last frame, left by a killed capture),
        ``date``, ``tone_updates_subscribed`` and ``metadata`` (the whole
        ``.json``, including ``info``).
    """
    filename = os.fspath(filename)
    with open(filename + '.json') as file:
        metadata = json.load(file)
    if metadata['format'] != '<i4':
        raise ValueError(f"unsupported stream format {metadata['format']!r}")
    num_tones = int(metadata['num_tones'])
    frame_bytes = int(metadata.get('frame_bytes', 4 * (2 * num_tones + 10)))
    if frame_bytes != 4 * (2 * num_tones + 10):
        raise ValueError(f'frame_bytes {frame_bytes} does not match {num_tones} tones')
    file_bytes = os.path.getsize(filename)
    num_samples = file_bytes // frame_bytes
    sample_rate = float(metadata['sample_rate'])
    return {
        'num_samples': num_samples,
        'num_tones': num_tones,
        'sample_rate': sample_rate,
        'duration_s': num_samples / sample_rate,
        'frame_bytes': frame_bytes,
        'file_bytes': file_bytes,
        'trailing_bytes': file_bytes - num_samples * frame_bytes,
        'date': metadata['date'],
        'tone_updates_subscribed': bool(metadata.get('tone_updates_subscribed', False)),
        'metadata': metadata,
    }


def read_stream_chunk(filename, start=0, stop=None, tones=None, fields=None, *,
                      apply_readout_correction=True, progress=True, block_mb=64,
                      check_memory=True):
    """Read samples ``start`` to ``stop`` of a stream capture, in parsed form.

    Parameters
    ----------
    filename : str or os.PathLike
        The capture basename (see :func:`stream_file_info`).
    start, stop : int, optional
        Sample range, ``start <= sample < stop``, counted from the start of
        the capture. ``stop=None`` (default) reads to the end.
    tones : int or sequence of int or None, optional
        Tone indices to read. ``None`` (default) reads every tone; ``[]``
        reads none (header fields only).
    fields : str or sequence of str or None, optional
        Header fields to include, from :data:`HEADER_FIELDS`. ``None``
        (default) includes them all; ``[]`` none.
    apply_readout_correction : bool, optional
        As for ``parse_stream`` (default ``True``). The corrected I/Q are
        float64, three times the memory of the raw int32.
    progress : bool, optional
        Print a progress line with the read rate and time left (default).
    block_mb : float, optional
        Size of each disk read (default 64 MB).
    check_memory : bool, optional
        Raise ``MemoryError`` before reading when the result would need more
        than :data:`MEMORY_FRACTION` of the free memory (default ``True``).

    Returns
    -------
    dict
        The ``parse_stream`` format, restricted to the chosen tones (keyed by
        their capture index, e.g. ``'0007'``) and fields, plus
        ``sample_index`` and ``time_s``, and ``stream_file`` (the absolute
        path). ``num_tones`` is the number of tones read. For a capture with
        tone updates, ``tone_updates`` holds the records inside the range and,
        from before it, the latest one of each revision the chunk's samples
        are tagged with; the per-revision tables cover the capture up to
        ``stop``.
    """
    from .client.readout_client import ReadoutClient
    from . import resonator

    filename = os.path.abspath(os.fspath(filename))
    layout = stream_file_info(filename)
    metadata = layout['metadata']
    total_tones = layout['num_tones']
    words_per_frame = layout['frame_bytes'] // 4

    stop = layout['num_samples'] if stop is None else min(int(stop), layout['num_samples'])
    start = int(start)
    if not 0 <= start < stop:
        raise ValueError(f'need 0 <= start < stop; got start={start}, stop={stop} '
                         f"(the capture has {layout['num_samples']} samples)")
    n = stop - start

    if tones is None:
        tones = list(range(total_tones))
    else:
        tones = sorted(set(int(t) for t in np.atleast_1d(tones)))
    if tones and not 0 <= tones[0] <= tones[-1] < total_tones:
        raise ValueError(f'tones must lie in 0..{total_tones - 1}')
    if fields is None:
        fields = list(HEADER_FIELDS)
    else:
        fields = [fields] if isinstance(fields, str) else list(fields)
    unknown = set(fields) - set(HEADER_FIELDS)
    if unknown:
        raise ValueError(f'unknown fields {sorted(unknown)}; choose from {HEADER_FIELDS}')

    # Which words of each frame to keep: I and Q of each tone, then the header
    # words the fields need. flag5 always comes along -- it is 4 bytes a
    # sample, and the readout correction and the tone updates need it.
    # The six flags are contiguous from index_flag_0, as parse_stream reads them.
    flag_index = [metadata['index_flag_0'] + k for k in range(6)]
    header_words = {'flag5': flag_index[5]}
    if 'packet_counter' in fields:
        header_words['cnt'] = metadata['index_cnt']
    if 'packet_error' in fields:
        header_words['err'] = metadata['index_err']
    has_tt = metadata.get('index_tt_msb') is not None and metadata.get('index_tt_lsb') is not None
    if 'telescope_time' in fields and has_tt:
        header_words['tt_msb'] = metadata['index_tt_msb']
        header_words['tt_lsb'] = metadata['index_tt_lsb']
    if 'stream_flags' in fields:
        header_words.update({f'flag{k}': flag_index[k] for k in range(6)})
    columns = [word for t in tones for word in (2 * t, 2 * t + 1)] + list(header_words.values())
    column_of = {name: 2 * len(tones) + k for k, name in enumerate(header_words)}

    if check_memory:
        per_sample = 4 * len(columns) + 16            # the words kept + sample_index, time_s
        if tones and apply_readout_correction:
            per_sample += 16 * len(tones)             # float64 corrected I and Q
        per_sample += 8 * ('packet_counter' in fields) + 16 * ('telescope_time' in fields)
        per_sample += 24                              # modulation point/settling/revision
        _check_memory(n * per_sample, f'{n} samples of {len(tones)} tone(s)')

    picked = np.empty((n, len(columns)), dtype='<i4')
    rows_per_block = max(1, int(block_mb * 2**20) // layout['frame_bytes'])
    block = np.empty(rows_per_block * words_per_frame, dtype='<i4')
    total_bytes = n * layout['frame_bytes']
    if progress:
        print(f'Reading {n} samples ({n / layout["sample_rate"]:.1f} s, '
              f'{_size(total_bytes)}) of {os.path.basename(filename)}, '
              f'{len(tones)} tone(s)')
    t0 = last_print = time.monotonic()
    with open(filename, 'rb', buffering=0) as file:
        if hasattr(os, 'posix_fadvise'):
            # Ask the kernel for a large read-ahead: the read is sequential.
            os.posix_fadvise(file.fileno(), 0, 0, os.POSIX_FADV_SEQUENTIAL)
        file.seek(start * layout['frame_bytes'])
        done = 0
        while done < n:
            rows = min(rows_per_block, n - done)
            frames = block[:rows * words_per_frame]
            _read_exact(file, frames)
            picked[done:done + rows] = frames.reshape(rows, words_per_frame)[:, columns]
            done += rows
            now = time.monotonic()
            if progress and (now - last_print > 1.0 or done == n):
                _print_progress(done * layout['frame_bytes'], total_bytes, now - t0)
                last_print = now
    if progress:
        print()
    del block

    # Assemble the parse_stream format from the kept words.
    unsigned = picked.view('<u4')
    point, settling, revision = ReadoutClient._decode_modulation_flag(
        unsigned[:, column_of['flag5']])
    info = ReadoutClient._normalise_modulated_sample_info(
        copy.deepcopy(metadata['info']), point, total_tones)
    sample_index = np.arange(start, stop, dtype=np.int64)
    result = {
        'date': metadata['date'],
        'num_tones': len(tones),
        'num_samples': n,
        'sample_rate': layout['sample_rate'],
        'info': info,
        'i_data': {f'{t:04d}': picked[:, 2 * k] for k, t in enumerate(tones)},
        'q_data': {f'{t:04d}': picked[:, 2 * k + 1] for k, t in enumerate(tones)},
    }
    if 'packet_counter' in fields:
        result['packet_counter'] = unsigned[:, column_of['cnt']].astype(np.int64)
        ReadoutClient._warn_on_dropped_accumulations(result['packet_counter'],
                                                     'read_stream_chunk')
        result['adc_overvoltage'], result['adc_overvoltage_events'] = \
            capture_adc_overvoltage(metadata, result['packet_counter'])
    if 'packet_error' in fields:
        result['packet_error'] = unsigned[:, column_of['err']]
    if 'telescope_time' in fields:
        if has_tt:
            result['telescope_time'] = ((unsigned[:, column_of['tt_msb']].astype(np.uint64) << 32)
                                        + unsigned[:, column_of['tt_lsb']].astype(np.uint64))
        else:
            result['telescope_time'] = np.zeros(n, dtype=np.uint64)
    if 'stream_flags' in fields:
        result['stream_flags'] = {f'flag{k}': unsigned[:, column_of[f'flag{k}']]
                                  for k in range(6)}
    # The readout correction reads the point and revision tags, so they go in
    # now and come out again below if they were not asked for.
    result['modulation_point'] = point
    result['modulation_settling'] = settling
    result['modulation_revision'] = revision
    result['readout_correction_applied'] = False
    result['sample_index'] = sample_index
    result['time_s'] = sample_index / layout['sample_rate']
    result['stream_file'] = filename

    if layout['tone_updates_subscribed']:
        records, by_revision, factors_by_revision = _read_tone_updates(
            filename, start, stop, set(np.unique(revision).tolist()), total_tones)
        result['tone_updates'] = records
        result['tone_centers_by_revision'] = by_revision
        result['readout_correction_by_revision'] = factors_by_revision
    if tones and apply_readout_correction:
        result = resonator.with_readout_correction(result, True)
    for field in ('modulation_point', 'modulation_settling', 'modulation_revision'):
        if field not in fields:
            del result[field]
    return result


def concatenate_stream_chunks(chunks, check_memory=True):
    """Join chunks from :func:`read_stream_chunk` into one, in sample order.

    The chunks must come from the same capture with the same tones, fields
    and readout correction. They may be passed in any order. Where chunks
    overlap each sample is kept once. Gaps between chunks are allowed but
    warned about: the joined record is then not evenly sampled, and its
    ``sample_index`` / ``time_s`` say where each sample belongs.

    Parameters
    ----------
    chunks : sequence of dict
        Chunks from :func:`read_stream_chunk`.
    check_memory : bool, optional
        Raise ``MemoryError`` first when the joined copy would need more than
        :data:`MEMORY_FRACTION` of the free memory (default ``True``).

    Returns
    -------
    dict
        One chunk in the same format covering all the samples.
    """
    chunks = [chunk for chunk in chunks if chunk['num_samples']]
    if not chunks:
        raise ValueError('no chunks with samples to join')
    chunks.sort(key=lambda chunk: int(chunk['sample_index'][0]))
    first = chunks[0]
    for chunk in chunks[1:]:
        for key in ('stream_file', 'readout_correction_applied'):
            if chunk.get(key) != first.get(key):
                raise ValueError(f'chunks differ in {key}: {first.get(key)!r} vs {chunk.get(key)!r}')
        for key in ('i_data', 'q_data'):
            if chunk[key].keys() != first[key].keys():
                raise ValueError('chunks hold different tones; read them with the same tones')
        if ({key for key in chunk if key in HEADER_FIELDS}
                != {key for key in first if key in HEADER_FIELDS}):
            raise ValueError('chunks hold different header fields; read them with the same fields')

    if check_memory:
        need = sum(_nbytes(chunk) for chunk in chunks)
        _check_memory(need, f'joining {len(chunks)} chunks')

    # Where chunks overlap, np.unique keeps the first copy of each sample.
    index = np.concatenate([chunk['sample_index'] for chunk in chunks])
    keep = None
    if np.any(np.diff(index) <= 0):
        _, keep = np.unique(index, return_index=True)

    def join(arrays):
        joined = np.concatenate(arrays)
        return joined if keep is None else joined[keep]

    result = dict(first)
    for key in _PER_SAMPLE_ARRAYS:
        if key in first:
            result[key] = join([chunk[key] for chunk in chunks])
    for key in _PER_SAMPLE_DICTS:
        if key in first:
            result[key] = {name: join([chunk[key][name] for chunk in chunks])
                           for name in first[key]}
    result['num_samples'] = len(result['sample_index'])

    if 'tone_updates' in first:
        seen = {}
        for chunk in chunks:
            for record in chunk['tone_updates']:
                seen[(record['sample_index'], record['frame_type'],
                      record['host_time_unix'])] = record
        result['tone_updates'] = sorted(seen.values(), key=lambda r: r['sample_index'])
        # Each chunk's tables cover the capture up to its end, so the one
        # reaching furthest holds them all.
        last = max(chunks, key=lambda chunk: int(chunk['sample_index'][-1]))
        result['tone_centers_by_revision'] = last['tone_centers_by_revision']
        result['readout_correction_by_revision'] = last['readout_correction_by_revision']

    gaps = np.flatnonzero(np.diff(result['sample_index']) > 1)
    if gaps.size:
        warnings.warn(
            f'the joined chunks have {gaps.size} gap(s), the first after sample '
            f"{int(result['sample_index'][gaps[0]])}: the record is not evenly "
            'sampled; use sample_index or time_s for the time of each sample.',
            stacklevel=2)
    return result


# ---------------------------------------------------------------------------
# The real time of each sample
# ---------------------------------------------------------------------------

# How well each kind of record pins a sample's time (s).
ANCHOR_SIGMA_S = {
    'time_anchor': 0.002,       # server clock read with the accumulation count
    'update_ts': 0.05,          # server clock when a typed frame was queued
    'first_frame': 0.05,        # client clock when a frame arrived
    'last_frame': 0.05,
    'update_host': 0.1,         # client clock when a typed frame arrived
    'info_server_time': 2.0,    # server clock at get_info, just before the capture
    'file_mtime': 2.0,          # last write to the file (client clock; lost by a copy)
}
# The recorded TT is right if it is within this of the anchors (or of 3 sigma).
TT_TOLERANCE_S = 1.0
# A TT before this (2001-09-09) was never loaded: it counts from firmware start.
TT_LOADED_AFTER_UNIX = 1e9
# Warn when the counter's rate or the TT's drift is further off than these.
RATE_TOLERANCE_PPM = 50.0
TT_DRIFT_TOLERANCE_PPM = 5.0


def adc_overvoltage_events_in_capture(events, first_frame, last_frame):
    """The ADC over-voltage events (``firmware_lib.adc_overvoltage_events``) in a capture.

    Matched by accumulation count against the capture's ``first_frame`` /
    ``last_frame`` anchors (its ``.json``); an event without a count is
    matched by time instead.
    """
    if not first_frame or not last_frame:
        return []
    c0 = int(first_frame['packet_counter'])
    span = (int(last_frame['packet_counter']) - c0) % 2**32
    kept = []
    for event in events:
        if 'acc_cnt' in event:
            if (int(event['acc_cnt']) - c0) % 2**32 <= span:
                kept.append(event)
        elif first_frame['host_time_unix'] <= event['server_time_unix'] <= last_frame['host_time_unix']:
            kept.append(event)
    return kept


def adc_overvoltage_mask(packet_counter, events, latched_at_start=False, counter0=None):
    """Per sample: whether the ADC's over-voltage protection was engaged.

    While engaged the ADC's DSA is held at 15 dB (the RX scale differs from the
    rest of the capture). A ``'trip'`` event starts an engaged interval; a
    ``'release'`` (that did not trip again) or ``'cleared'`` ends it.
    ``latched_at_start`` is the state when the capture began (its
    ``info['rfdc']['adc_overvoltage_latched']``). The monitor polls once a
    second, so an interval starts up to a second after the trip itself.
    ``counter0`` is the capture's first packet counter (default: this
    array's first), so a chunk from mid-capture gets the right state.
    """
    counter = np.asarray(packet_counter, dtype=np.int64)
    mask = np.full(len(counter), bool(latched_at_start))
    if not len(counter):
        return mask
    c0 = int(counter[0] if counter0 is None else counter0)
    relative = (counter - c0) % 2**32
    timed = [e for e in events if 'acc_cnt' in e]
    for event in sorted(timed, key=lambda e: (int(e['acc_cnt']) - c0) % 2**32):
        start = int(np.searchsorted(relative, (int(event['acc_cnt']) - c0) % 2**32))
        if event['kind'] == 'trip':
            mask[start:] = True
        elif event['kind'] in ('release', 'cleared') and not event.get('still_over_voltage'):
            mask[start:] = False
    return mask


def capture_adc_overvoltage(metadata, packet_counter):
    """``(mask, events)`` for samples of a capture, from its ``.json`` metadata."""
    events = list(metadata.get('adc_overvoltage_events') or [])
    rfdc = (metadata.get('info') or {}).get('rfdc') or {}
    first = metadata.get('first_frame') or {}
    mask = adc_overvoltage_mask(packet_counter, events,
                                latched_at_start=bool(rfdc.get('adc_overvoltage_latched')),
                                counter0=first.get('packet_counter'))
    return mask, events


def parse_overvoltage_journal(lines):
    """Events from the server's ``ADC_OVERVOLTAGE {json}`` journal lines.

    For a capture whose receiver could not fetch the server's log at the end,
    e.g. ``journalctl -u readout_server_0 | grep ADC_OVERVOLTAGE``.
    """
    if isinstance(lines, str):
        lines = lines.splitlines()
    events = []
    for line in lines:
        if 'ADC_OVERVOLTAGE ' in line:
            events.append(json.loads(line.split('ADC_OVERVOLTAGE ', 1)[1]))
    return events


def add_adc_overvoltage_events(filename, events):
    """Attach over-voltage events to a capture's ``.json``; returns those that fit.

    Events outside the capture are ignored, and ones already there are not
    duplicated.
    """
    with open(f'{filename}.json') as handle:
        metadata = json.load(handle)
    inside = adc_overvoltage_events_in_capture(
        events, metadata.get('first_frame'), metadata.get('last_frame'))
    existing = list(metadata.get('adc_overvoltage_events') or [])
    seen = {(e.get('kind'), e.get('server_time_unix')) for e in existing}
    added = [e for e in inside if (e.get('kind'), e.get('server_time_unix')) not in seen]
    metadata['adc_overvoltage_events'] = sorted(
        existing + added, key=lambda e: e.get('server_time_unix', 0.0))
    with open(f'{filename}.json', 'w') as handle:
        json.dump(metadata, handle, indent=4)
    return added


def stream_time_model(filename, progress=True):
    """Work out the real (UNIX) time of every sample of a capture, and check its TT.

    The telescope time in the data cannot be taken on trust: it may never have
    been loaded (it then counts from firmware start), or be loaded but wrong (on
    2026-09-29 the accumulator's copy was 7.25 days behind), or jump if it was
    reloaded mid-capture. So the time is also worked out independently, from the
    packet counter (one count per accumulation, gaps included) and every clock
    reading the capture holds, fitted as ``unix = t0 + count / rate``:

    - ``time_anchor`` in the tone-update records: the server clock read together
      with the accumulation count (servers from 2026-09-30);
    - ``ts`` / ``host_time_unix`` of every tone-update record, against the frame
      it arrived after;
    - ``first_frame`` / ``last_frame`` in the ``.json`` (clients from 2026-09-30);
    - the server time in the capture's ``info``, taken just before it began, and
      the file's modification time, both only to a couple of seconds.

    Then the TT is compared with that: ``tt_status`` is ``'ok'``,
    ``'not_loaded'``, ``'offset'`` (loaded but wrong) or ``'unknown'`` (nothing
    to compare with). A TT jump inside the capture and a drift against the
    anchors are checked, and so is the counter's rate against the nominal
    sample rate (an ADC clock off frequency shows up there). Each problem is
    warned about and listed in ``warnings``.

    Parameters
    ----------
    filename : str or os.PathLike
        The capture basename.
    progress : bool, optional
        Print progress while the tone-update log is read (default ``True``).

    Returns
    -------
    dict
        ``time_source`` (``'telescope_time'`` when the TT is right, else
        ``'packet_counter'``), ``first_sample_unix`` / ``last_sample_unix``,
        ``rate_hz`` (fitted accumulation rate) and ``rate_ppm`` (against the
        nominal ``sample_rate``), ``precision_s``, ``tt_status``,
        ``tt_offset_s`` (TT minus real time), ``tt_drift_ppm``, ``tt_jump_s``,
        ``anchors`` (``counter``, ``unix``, ``tt_unix``, ``sigma_s``,
        ``source``), ``warnings``, and what :func:`sample_times` and
        :func:`find_sample` need (``counter0``, ``t0_unix``, ``clk_hz``).
    """
    filename = os.path.abspath(os.fspath(filename))
    layout = stream_file_info(filename)
    metadata, n = layout['metadata'], layout['num_samples']
    info = metadata.get('info') or {}
    if n < 2:
        raise ValueError(f'{filename} has {n} complete samples; need at least 2')
    nominal_rate = layout['sample_rate']
    # The first two frames give the TT ticks per accumulation, which with the
    # nominal sample rate is the fabric clock the server converts TT with.
    cnt, tt = _read_frame_headers(filename, layout, [0, 1, n - 1])
    counter0 = int(cnt[0])

    def counter(c):
        return _signed_counts(c, counter0)

    ticks_per_count = (tt[1] - tt[0]) / max(1, counter(cnt[1]))
    clk_hz = ticks_per_count * nominal_rate

    # --- Every clock reading the capture holds, as (counter, unix, tt_unix, source).
    anchors = []
    for key in ('first_frame', 'last_frame'):
        record = metadata.get(key)
        if record:
            anchors.append((counter(record['packet_counter']), record['host_time_unix'],
                            record['telescope_time'] / clk_hz, key))
    server_time = (info.get('server') or {}).get('current_time_unix_s')
    if server_time is not None:
        anchors.append((0, float(server_time), tt[0] / clk_hz, 'info_server_time'))
    mtime = os.path.getmtime(filename)
    if server_time is None or mtime > server_time:
        anchors.append((counter(cnt[2]), mtime, tt[2] / clk_hz, 'file_mtime'))
    if layout['tone_updates_subscribed'] and os.path.exists(filename + '.updates.jsonl'):
        anchors += _update_anchors(filename, layout, counter, clk_hz, progress)

    model = {'stream_file': filename, 'num_samples': n}
    return _time_model(model, anchors, cnt, tt, nominal_rate)


def _time_model(model, anchors, cnt, tt, nominal_rate):
    """Fit real time to the counter and check the TT, for both model builders.

    ``anchors`` are ``(counter, unix, tt_unix, source)`` clock readings;
    ``cnt`` / ``tt`` are the packet counter and raw TT of the first, second
    and last samples. Fills in and returns ``model`` (see
    :func:`stream_time_model`).
    """
    problems = []

    def problem(message):
        problems.append(message)
        warnings.warn(message, stacklevel=4)

    counter0 = int(cnt[0])

    def counter(c):
        return _signed_counts(c, counter0)

    ticks_per_count = (tt[1] - tt[0]) / max(1, counter(cnt[1]))
    clk_hz = ticks_per_count * nominal_rate
    model.update({'nominal_sample_rate': nominal_rate, 'counter0': counter0,
                  'clk_hz': clk_hz, 'ticks_per_count': ticks_per_count,
                  'warnings': problems})
    if not anchors:
        model.update(time_source='sample_index', tt_status='unknown', t0_unix=np.nan,
                     rate_hz=nominal_rate, rate_ppm=np.nan, precision_s=np.nan)
        problem('no clock reading in the capture: real time cannot be worked out')
        return model

    source = np.array([a[3] for a in anchors])
    counts = np.array([a[0] for a in anchors], dtype=float)
    unix = np.array([a[1] for a in anchors], dtype=float)
    tt_unix = np.array([a[2] for a in anchors], dtype=float)
    sigma = np.array([ANCHOR_SIGMA_S[s] for s in source])

    # --- Fit unix = t0 + counter / rate, dropping readings far off the rest.
    reference = unix.min()
    keep = np.ones(len(unix), dtype=bool)
    for _ in range(2):
        t0, rate = _fit_clock(counts[keep], unix[keep] - reference, sigma[keep], nominal_rate)
        residual = unix - reference - (t0 + counts / rate)
        outliers = keep & (np.abs(residual) > 5 * sigma + 0.01)
        if not outliers.any():
            break
        keep &= ~outliers
    # Update records often go out a little after their ts (queued behind frames),
    # so dropping some of those is normal; anything else is worth a warning.
    dropped = ~keep & ~np.isin(source, ('update_ts', 'update_host'))
    if dropped.any():
        problem(f'{int(dropped.sum())} clock reading(s) disagree with the rest by up to '
                f'{np.abs(residual[dropped]).max():.2f} s and were left out '
                f'({sorted(set(source[dropped].tolist()))})')
    t0_unix = reference + t0
    rate_ppm = (rate / nominal_rate - 1.0) * 1e6
    precision = float(np.sqrt(np.average(residual[keep] ** 2, weights=1 / sigma[keep] ** 2))
                      ) if keep.sum() > 2 else float(sigma[keep].min())
    # How well the rate is known: two of the best readings at the span's ends.
    span_s = np.ptp(counts[keep]) / nominal_rate
    rate_known_ppm = 3e6 * float(sigma[keep].min()) / span_s if span_s > 600 else np.inf
    if abs(rate_ppm) > max(RATE_TOLERANCE_PPM, rate_known_ppm):
        problem(f'accumulations arrive at {rate:.4f} Hz, {rate_ppm:+.0f} ppm from the nominal '
                f'{nominal_rate} Hz: the sample clock is off frequency (check the ADC '
                'reference), so sample_index / sample_rate is not real time')

    # --- Check the TT against the fitted clock.
    good = keep & (sigma <= min(0.1, sigma[keep].min() * 3))
    if not good.any():
        good = keep
    tolerance = max(TT_TOLERANCE_S, 3 * float(sigma[good].max()))
    offsets = tt_unix - (t0_unix + counts / rate)
    tt_offset = float(np.median(offsets[good]))
    tt_jump = ((tt[2] - tt[0]) - ticks_per_count * counter(cnt[2])) / clk_hz
    tt_drift = np.nan
    if good.sum() >= 3 and np.ptp(unix[good]) > 600:
        tt_drift = float(np.polyfit(unix[good] - reference, offsets[good], 1)[0] * 1e6)

    if tt[0] / clk_hz < TT_LOADED_AFTER_UNIX:
        tt_status = 'not_loaded'
        problem(f'the telescope time was never loaded: it reads {tt[0] / clk_hz / 86400:.2f} '
                'days since the epoch (clock ticks since the firmware started); times come '
                'from the packet counter and the clock readings instead')
    elif abs(tt_offset) > tolerance:
        tt_status = 'offset'
        problem(f'the telescope time is {tt_offset:+.3f} s ({tt_offset / 86400:+.3f} days) '
                'off real time: it was loaded but is wrong (the data carries the '
                "accumulator's TT; see doc/tsu_strobe_and_timed_sync.md); times come from "
                'the packet counter and the clock readings instead')
    else:
        tt_status = 'ok'
    if abs(tt_jump) > 1.0 / nominal_rate:
        problem(f'the telescope time jumps by {tt_jump:+.3f} s somewhere in the capture '
                '(reloaded while recording?): it is not continuous with the packet counter')
        if tt_status == 'ok':
            tt_status = 'offset'
    # (A jump also reads as a drift, so only a continuous TT is checked for it.)
    if (np.isfinite(tt_drift) and abs(tt_drift) > TT_DRIFT_TOLERANCE_PPM
            and abs(tt_jump) <= 1.0 / nominal_rate):
        problem(f'the telescope time drifts {tt_drift:+.1f} ppm against the clock readings')

    model.update(
        time_source='telescope_time' if tt_status == 'ok' else 'packet_counter',
        t0_unix=t0_unix, rate_hz=rate, rate_ppm=rate_ppm, precision_s=precision,
        first_sample_unix=t0_unix,
        last_sample_unix=t0_unix + counter(cnt[2]) / rate,
        tt_status=tt_status, tt_offset_s=tt_offset, tt_drift_ppm=tt_drift, tt_jump_s=tt_jump,
        anchors={'counter': counts, 'unix': unix, 'tt_unix': tt_unix, 'sigma_s': sigma,
                 'source': source, 'used': keep})
    return model


def capture_time_model(samples):
    """:func:`stream_time_model` for a capture already in memory.

    ``samples`` is a parsed capture: ``parse_samples`` of a ``get_samples``
    capture (which brackets it with server time anchors, ``time_anchors``), or
    a chunk from :func:`read_stream_chunk` (its tone-update records' anchors).
    It needs ``packet_counter`` and ``telescope_time``. The model is relative to
    this capture's first sample; use it with :func:`sample_times`.
    """
    counts = np.asarray(samples['packet_counter'], dtype=np.int64)
    ticks = np.asarray(samples['telescope_time'], dtype=np.uint64)
    n = len(counts)
    if n < 2:
        raise ValueError('need at least 2 samples')
    cnt = counts[[0, 1, n - 1]]
    tt = [int(ticks[0]), int(ticks[1]), int(ticks[n - 1])]
    readings = list(samples.get('time_anchors') or [])
    readings += [record['payload']['time_anchor'] for record in samples.get('tone_updates', ())
                 if (record.get('payload') or {}).get('time_anchor')]
    clk_hz = (tt[1] - tt[0]) / max(1, int(_signed_counts(cnt[1], cnt[0]))) \
        * float(samples['sample_rate'])
    anchors = [(_signed_counts(r['acc_cnt'], cnt[0]), r['server_time_unix'],
                r['telescope_time'] / clk_hz, 'time_anchor') for r in readings]
    return _time_model({'num_samples': n}, anchors, cnt, tt, float(samples['sample_rate']))


def sample_times(chunk, model):
    """UNIX time of each sample of a chunk, from :func:`stream_time_model`.

    The chunk's own TT when the model found it right (``time_source`` is
    ``'telescope_time'``), else its packet counter through the fitted clock --
    so read the chunk with ``'telescope_time'`` or ``'packet_counter'`` in its
    fields (the default reads both).
    """
    if model['time_source'] == 'telescope_time' and 'telescope_time' in chunk:
        return np.asarray(chunk['telescope_time'], dtype=float) / model['clk_hz']
    if 'packet_counter' not in chunk:
        raise ValueError("read the chunk with 'packet_counter' (or a TT the model "
                         'found right) to place it in time')
    counts = _signed_counts(chunk['packet_counter'], model['counter0'])
    return model['t0_unix'] + counts / model['rate_hz']


def find_sample(filename, when, model=None):
    """The first sample at or after ``when``: a start or stop for :func:`read_stream_chunk`.

    Bisects the file on the packet counter (or on the TT, when the model found
    it right), reading a few dozen frames, so it takes seconds on any size.

    Parameters
    ----------
    filename : str or os.PathLike
        The capture basename.
    when : float, datetime.datetime or str
        UNIX seconds, a datetime, or an ISO-8601 string. Datetimes and strings
        without a time zone are taken as UTC.
    model : dict or None, optional
        From :func:`stream_time_model`; pass it when looking up several times,
        since building it reads the tone-update log.

    Returns
    -------
    int
        Sample index (``num_samples`` when ``when`` is after the capture).
    """
    filename = os.path.abspath(os.fspath(filename))
    if model is None:
        model = stream_time_model(filename)
    layout = stream_file_info(filename)
    target_unix = _unix_seconds(when)
    if not np.isfinite(model.get('t0_unix', np.nan)):
        raise ValueError('the capture cannot be placed in time (no clock readings)')
    first, last = model['first_sample_unix'], model['last_sample_unix']
    if not first <= target_unix <= last:
        warnings.warn(f'{when} is outside the capture ({_iso(first)} to {_iso(last)})',
                      stacklevel=2)
    use_tt = model['time_source'] == 'telescope_time'
    if use_tt:
        target = target_unix * model['clk_hz']
    else:
        target = (target_unix - model['t0_unix']) * model['rate_hz']

    lo, hi = 0, layout['num_samples']
    while lo < hi:
        mid = (lo + hi) // 2
        cnt, tt = _read_frame_headers(filename, layout, [mid])
        value = tt[0] if use_tt else _signed_counts(cnt, model['counter0'])[0]
        if value < target:
            lo = mid + 1
        else:
            hi = mid
    return lo


def _update_anchors(filename, layout, counter, clk_hz, progress):
    """Clock readings from the tone-update log, as (counter, unix, tt_unix, source)."""
    n = layout['num_samples']
    records = []
    size = os.path.getsize(filename + '.updates.jsonl')
    last_print, done = time.monotonic(), 0
    with open(filename + '.updates.jsonl') as file:
        for line in file:
            done += len(line)
            if not line.strip():
                continue
            record = json.loads(line)
            payload = record.get('payload') or {}
            anchor = payload.get('time_anchor')
            if anchor:
                records.append((None, anchor))
            elif payload.get('ts') is not None or record.get('host_time_unix') is not None:
                # It arrived after sample_index frames: pin it to the last of them.
                frame = min(max(int(record['sample_index']) - 1, 0), n - 1)
                records.append((frame, (payload.get('ts'), record.get('host_time_unix'))))
            if progress and time.monotonic() - last_print > 1.0:
                print(f'\r  reading the tone-update log: {100 * done / size:.0f}%   ',
                      end='', flush=True)
                last_print = time.monotonic()
    if progress and records:
        print(f'\r  reading the tone-update log: {len(records)} clock readings   ')
    frames = sorted({frame for frame, _ in records if frame is not None})
    cnt, tt = _read_frame_headers(filename, layout, frames)
    header = dict(zip(frames, zip(cnt, tt)))
    anchors = []
    for frame, reading in records:
        if frame is None:
            anchors.append((counter(reading['acc_cnt']), reading['server_time_unix'],
                            reading['telescope_time'] / clk_hz, 'time_anchor'))
            continue
        frame_cnt, frame_tt = header[frame]
        server_ts, host_time = reading
        if server_ts is not None:
            anchors.append((counter(frame_cnt), server_ts, frame_tt / clk_hz, 'update_ts'))
        else:
            anchors.append((counter(frame_cnt), host_time, frame_tt / clk_hz, 'update_host'))
    return anchors


def _fit_clock(counts, unix, sigma, nominal_rate):
    """``(t0, rate)`` of ``unix = t0 + counts / rate``, weighted by ``1/sigma``.

    The rate is fitted when the readings span over ten minutes; otherwise it
    is the nominal rate and only ``t0`` is fitted.
    """
    weights = 1.0 / sigma ** 2
    span_s = np.ptp(counts) / nominal_rate
    if len(counts) >= 2 and span_s > 600:
        slope, t0 = np.polyfit(counts, unix, 1, w=1.0 / sigma)
        return float(t0), float(1.0 / slope)
    t0 = np.average(unix - counts / nominal_rate, weights=weights)
    return float(t0), float(nominal_rate)


def _read_frame_headers(filename, layout, indices):
    """``(packet_counter, telescope_time)`` of the frames at ``indices``.

    One seek per frame, in file order; the TT comes back as Python ints (exact
    beyond 2**53).
    """
    metadata = layout['metadata']
    frame_bytes = layout['frame_bytes']
    cnt_index = metadata['index_cnt']
    msb_index, lsb_index = metadata.get('index_tt_msb'), metadata.get('index_tt_lsb')
    counts, tts = [], []
    with open(filename, 'rb') as file:
        for index in indices:
            file.seek(int(index) * frame_bytes)
            words = np.frombuffer(file.read(frame_bytes), dtype='<u4')
            counts.append(int(words[cnt_index]))
            tts.append((int(words[msb_index]) << 32) + int(words[lsb_index])
                       if msb_index is not None and lsb_index is not None else 0)
    return np.asarray(counts, dtype=np.int64), tts


def _signed_counts(counts, counter0):
    """Accumulations since ``counter0``, across the 32-bit wrap, negative before it."""
    counts = (np.asarray(counts, dtype=np.int64) - int(counter0)) % 2**32
    return np.where(counts >= 2**31, counts - 2**32, counts)


def _unix_seconds(when):
    import datetime
    if isinstance(when, str):
        when = datetime.datetime.fromisoformat(when)
    if isinstance(when, datetime.datetime):
        if when.tzinfo is None:
            when = when.replace(tzinfo=datetime.timezone.utc)
        return when.timestamp()
    return float(when)


def _iso(unix_s):
    import datetime
    return datetime.datetime.fromtimestamp(unix_s, datetime.timezone.utc).isoformat()


def _read_tone_updates(filename, start, stop, revisions, num_tones):
    """The tone-update records a chunk keeps, and the per-revision tables.

    Records from ``start`` to ``stop`` are all kept. From before ``start``
    only the latest record of each revision in ``revisions`` is kept: frames
    tagged with an earlier revision can still arrive after its successor was
    announced, and ``demodulate_stream`` needs that revision's state. The
    tables are built from every record up to ``stop``, since the centre map
    is cumulative; records are read one line at a time, so a long capture's
    update log is never held whole.
    """
    from .client.readout_client import ReadoutClient

    kept_before = {}
    kept = []

    def records():
        with open(filename + '.updates.jsonl') as file:
            for line in file:
                if not line.strip():
                    continue
                record = json.loads(line)
                if record['sample_index'] >= stop:
                    return
                if record['sample_index'] >= start:
                    kept.append(record)
                else:
                    revision = record['payload'].get('revision')
                    if revision is not None and int(revision) & 0x7FFF in revisions:
                        kept_before[int(revision) & 0x7FFF] = record
                yield record

    by_revision, factors_by_revision = ReadoutClient._tone_update_tables(records(), num_tones)
    before = sorted(kept_before.values(), key=lambda r: r['sample_index'])
    return before + kept, by_revision, factors_by_revision


def _read_exact(file, array):
    """Fill ``array`` from ``file``; an unbuffered read may return less."""
    view = memoryview(array).cast('B')
    got = 0
    while got < len(view):
        count = file.readinto(view[got:])
        if not count:
            raise EOFError(f'{file.name} ended {len(view) - got} bytes early')
        got += count


def _available_memory():
    """Bytes of memory free for use (Linux MemAvailable), or None if unknown."""
    try:
        with open('/proc/meminfo') as file:
            for line in file:
                if line.startswith('MemAvailable:'):
                    return int(line.split()[1]) * 1024
    except OSError:
        pass
    try:
        import psutil
        return psutil.virtual_memory().available
    except ImportError:
        return None


def _check_memory(need, what):
    available = _available_memory()
    if available is None:
        warnings.warn(f'cannot tell how much memory is free; {what} needs ~{_size(need)}',
                      stacklevel=3)
    elif need > MEMORY_FRACTION * available:
        raise MemoryError(
            f'{what} needs ~{_size(need)} but only {_size(available)} is free: '
            'read a shorter range or fewer tones (or pass check_memory=False)')


def _nbytes(chunk):
    """Bytes held by a chunk's per-sample arrays."""
    total = sum(chunk[key].nbytes for key in _PER_SAMPLE_ARRAYS if key in chunk)
    for key in _PER_SAMPLE_DICTS:
        total += sum(values.nbytes for values in chunk.get(key, {}).values())
    return total


def _size(nbytes):
    for unit in ('B', 'kB', 'MB', 'GB', 'TB'):
        if nbytes < 1000 or unit == 'TB':
            return f'{nbytes:.1f} {unit}' if unit != 'B' else f'{nbytes:.0f} B'
        nbytes /= 1000


def _print_progress(done_bytes, total_bytes, elapsed):
    rate = done_bytes / elapsed if elapsed > 0 else 0.0
    left = (total_bytes - done_bytes) / rate if rate > 0 else float('nan')
    print(f'\r  {_size(done_bytes)} / {_size(total_bytes)} '
          f'({100 * done_bytes / total_bytes:.0f}%), {_size(rate)}/s, '
          f'{left:.0f} s left   ', end='', flush=True)
