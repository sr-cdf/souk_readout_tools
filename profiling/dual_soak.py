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
        result['t_connect'] = time.time()
        end = time.monotonic() + seconds
        while time.monotonic() < end and i < room:
            recv_exact(s, view, 4)
            n = struct.unpack('>I', buf[:4])[0]
            if n == 0:
                continue                                 # idle heartbeat
            recv_exact(s, view, n)
            cnt[i], err[i] = struct.unpack_from('<Ii', buf, n - 8)
            i += 1
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
    args = ap.parse_args()

    clients = [ReadoutClient(address=args.addr, request_port=p) for p in args.ports]
    run = {'acc_len': args.acc_len, 'seconds': args.seconds, 'pipelines': {}}
    for p, c in zip(args.ports, clients):
        c.set_acc_len(args.acc_len)
        run['pipelines'][p] = {'rate': float(c.get_sample_rate()),
                               'tones': c.get_info('all').get('tones', {}).get('count')}
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
    for c in clients:
        c.disable_stream()
    run['end'] = time.strftime('%Y-%m-%dT%H:%M:%S')
    for p, c in zip(args.ports, clients):
        info = run['pipelines'][p]
        info.update(summarise(results[p], info['rate']))
        info['t_connect'] = results[p].get('t_connect')
        lt = c.get_info(sections='loop_timing')
        info['loop_timing'] = lt
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
        print(f"port {p}: {info['frames']} frames at {info['rate']:.0f} Hz, lost {info['lost']} "
              f"in {info['gaps']} gaps, torn {info['torn']}, spare us {info.get('spare_us')}", flush=True)
    with open(args.out, 'w') as f:
        json.dump(run, f, indent=1, default=str)


if __name__ == '__main__':
    main()
