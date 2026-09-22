#!/usr/bin/env python3
"""
Both pipelines streaming at once for a long time, counting every lost or torn
accumulation (test plan item 10 in doc/v712-notes.md). Host side, no root.

Both rates are set and both servers queried before either stream is enabled,
so no request reaches either server while they record: an info
request on one pipeline stalls the other's reads (doc/v712-notes.md, "Where the
mixer-settings stall comes from"). Frames are not kept -- an hour at 1500 Hz is
~43 GB per pipeline -- only each frame's accumulation counter and torn-read flag.

Output (JSON): per pipeline, frames, lost, gaps (position, size and time of
each), torn reads (position and time), and the loop timing read back afterwards.

    PYTHONPATH=src .venv/bin/python profiling/dual_soak.py --acc-len 400 --seconds 3600 \\
        --out ~/souk/v712_stages/soak/soak_1500.json

With ``--modulate HZ`` both pipelines run fw modulation ([-HZ, 0, +HZ, 0], the
fw default engine) while they record, and ``--track`` adds dry-run tone tracking
on top -- the production load for the zero-drop target. Whether each server
runs its stream loop in a separate process (server/stream_process.py) is read
back and recorded, so in-process and split runs can be compared from the files.
"""
import argparse
import json
import socket
import struct
import threading
import time

import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient


MAX_LISTED = 5000          # per-event detail kept per pipeline
STREAM_SILENCE_S = 5.0     # no bytes for this long ends the capture


def recv_exact(sock, view, n):
    got = 0
    while got < n:
        k = sock.recv_into(view[got:n], n - got)
        if k == 0:
            raise ConnectionError('stream closed')
        got += k


def receive(client, seconds, result, rate):
    """Keep (counter, torn flag) of every frame for ``seconds``.

    Preallocated: an hour at 5 kHz is 18 M frames per pipeline, and Python lists
    of that would cost the host hundreds of MB.
    """
    room = int(rate * seconds * 1.05) + 1000
    cnt = np.zeros(room, dtype=np.int64)
    err = np.zeros(room, dtype=np.int64)
    i = 0
    buf = bytearray(2048 * 8 + 64)
    view = memoryview(buf)
    with socket.create_connection((client.stream_server_address, client.stream_server_port)) as s:
        # A stream that goes silent ends the capture, keeping what came in: on
        # 2026-09-22 the board dropped its IP 589 s into a 600 s run and a blocking
        # recv held the whole result hostage.
        s.settimeout(STREAM_SILENCE_S)
        result['t_connect'] = time.time()
        end = time.monotonic() + seconds
        try:
            while time.monotonic() < end and i < room:
                recv_exact(s, view, 4)
                n = struct.unpack('>I', buf[:4])[0]
                if n == 0:
                    continue                             # idle heartbeat
                recv_exact(s, view, n)
                cnt[i], err[i] = struct.unpack_from('<Ii', buf, n - 8)
                i += 1
        except (socket.timeout, ConnectionError, OSError) as e:
            result['ended_early'] = {'reason': f'{type(e).__name__}: {e}',
                                     'at': time.strftime('%Y-%m-%dT%H:%M:%S'),
                                     'frames': i}
    result['cnt'] = cnt[:i]
    result['err'] = err[:i]


def summarise(result, rate):
    cnt, err = result['cnt'], result['err']
    d = np.diff(cnt)
    d[d < 0] += 1 << 32                                  # 32-bit counter wrap
    gaps = np.flatnonzero(d > 1)
    torn = np.flatnonzero(err != 0)
    return {
        'frames': int(len(cnt)), 'seconds': round(len(cnt) / rate, 1),
        'lost': int((d[gaps] - 1).sum()), 'gaps': int(len(gaps)), 'torn': int(len(torn)),
        # Counts are complete; the per-event lists are capped so a bad run at
        # 5 kHz does not write a 10 MB file.
        'gap_list': [{'frame': int(g), 'lost': int(d[g] - 1), 't_s': round(float(g) / rate, 3)}
                     for g in gaps[:MAX_LISTED]],
        'torn_list': [{'frame': int(t), 't_s': round(float(t) / rate, 3)} for t in torn[:MAX_LISTED]],
        'lists_truncated': bool(len(gaps) > MAX_LISTED or len(torn) > MAX_LISTED),
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--addr', default='10.11.11.11')
    ap.add_argument('--ports', type=int, nargs='+', default=[10000, 10001])
    ap.add_argument('--acc-len', type=int, required=True)
    ap.add_argument('--seconds', type=float, required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--modulate', type=float, default=None, metavar='HZ',
                    help='fw modulation with offsets [-HZ, 0, +HZ, 0] while recording')
    ap.add_argument('--dwell', type=int, default=1, help='accumulations per slot')
    ap.add_argument('--linewidth', type=float, default=5e4, help='per-tone linewidth, Hz')
    ap.add_argument('--track', action='store_true', help='dry-run tone tracking too')
    ap.add_argument('--track-poll', type=float, default=5.0, help='tracking window, s')
    ap.add_argument('--track-param', action='append', default=[], metavar='KEY=VALUE',
                    help='extra enable_tracking parameter (repeatable), e.g. '
                         'filter_blocks_per_batch=1; VALUE is parsed as JSON')
    args = ap.parse_args()
    if args.track and args.modulate is None:
        ap.error('--track needs --modulate')

    clients = [ReadoutClient(address=args.addr, request_port=p) for p in args.ports]
    run = {'acc_len': args.acc_len, 'seconds': args.seconds, 'pipelines': {}}
    run.update({'modulate_hz': args.modulate, 'dwell': args.dwell, 'track': args.track,
                'track_params': args.track_param})
    for p, c in zip(args.ports, clients):
        c.set_acc_len(args.acc_len)
        run['pipelines'][p] = {'rate': float(c.get_sample_rate()),
                               'tones': c.get_info('all').get('tones', {}).get('count')}
        if args.modulate is not None:
            d = float(args.modulate)
            c.enable_modulation(offsets=[-d, 0.0, d, 0.0], samples_per_point=args.dwell,
                                linewidth_hz=args.linewidth)
        if args.track:
            # Directly, so the newer tracking parameters need no client change.
            request = {'request': 'enable_tracking', 'dry_run': True,
                       'poll_interval_s': args.track_poll}
            for kv in args.track_param:
                key, value = kv.split('=', 1)
                request[key] = json.loads(value)
            reply = c.send_request(request)
            run['pipelines'][p]['tracking_enable'] = reply.get('status')
    results = {p: {} for p in args.ports}
    threads = [threading.Thread(target=receive,
                                args=(c, args.seconds, results[p], run['pipelines'][p]['rate']))
               for p, c in zip(args.ports, clients)]
    run['start'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    for c in clients:
        c.enable_stream()
    for t in threads:
        t.start()
    print(f"{run['start']} streaming {args.seconds:.0f} s on {args.ports}", flush=True)
    for t in threads:
        t.join()
    run['end'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    early = {p: results[p]['ended_early'] for p in args.ports if 'ended_early' in results[p]}
    if early:
        # Save what arrived before asking a server anything: it may be gone.
        run['ended_early'] = early
        for p in args.ports:
            run['pipelines'][p].update(summarise(results[p], run['pipelines'][p]['rate']))
        with open(args.out, 'w') as f:
            json.dump(run, f, indent=1, default=str)
        print(f"streams ended early: {early}; partial results in {args.out}", flush=True)
    for c in clients:
        c.disable_stream()
    for p, c in zip(args.ports, clients):
        if args.track:
            t = c.get_info(sections='tracking')
            summary = t.get('summary', {}) if isinstance(t, dict) else {}
            run['pipelines'][p]['tracking'] = {
                k: t.get(k) for k in ('frames_tapped', 'frames_skipped', 'cycles_seen',
                                      'consume_ms', 'consume_ms_max', 'stage_ms')
                if isinstance(t, dict)}
            run['pipelines'][p]['tracking']['summary'] = summary
            c.disable_tracking()
        if args.modulate is not None:
            c.disable_modulation()
    for p, c in zip(args.ports, clients):
        info = run['pipelines'][p]
        info.update(summarise(results[p], info['rate']))
        info['t_connect'] = results[p].get('t_connect')
        lt = c.get_info(sections='loop_timing')
        info['loop_timing'] = lt
        info['stream_process'] = lt.get('stream_process') if isinstance(lt, dict) else None
        if isinstance(lt, dict) and isinstance(lt.get('busy'), dict) and lt.get('period_us'):
            # Headroom per accumulation: what is left of the period once the
            # frame is read, built and sent.
            # Spare = period - busy, so a high busy percentile is a LOW spare
            # one: spare_p01 is the 1-in-100 worst frame, spare_min the worst seen.
            names = {'p50': 'spare_p50', 'p90': 'spare_p10', 'p99': 'spare_p01',
                     'p999': 'spare_p001', 'max': 'spare_min'}
            info['spare_us'] = {name: round(lt['period_us'] - lt['busy'][q], 1)
                                for q, name in names.items() if q in lt['busy']}
            info['period_us'] = round(lt['period_us'], 1)
        mode = 'split' if info['stream_process'] else 'in-process'
        print(f"port {p} ({mode}): {info['frames']} frames at {info['rate']:.0f} Hz, lost {info['lost']} "
              f"in {info['gaps']} gaps, torn {info['torn']}, spare us {info.get('spare_us')}", flush=True)
    with open(args.out, 'w') as f:
        json.dump(run, f, indent=1, default=str)


if __name__ == '__main__':
    main()
