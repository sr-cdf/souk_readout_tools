#!/usr/bin/env python3
"""
Lost accumulations per tone request while a stream runs (test plan item 2 in
doc/v712-notes.md). Host side, no root.

Tone requests are served on the server's event loop, the same loop that
produces the frames, so a request that takes longer than an accumulation period
costs accumulations. For each rate this captures one stream with no requests
(the baseline: capture edges and background) and one per request kind, each
re-sending the tone values already loaded every --every seconds, so nothing
changes on the board.

    lost per request = (lost in the capture - lost in the baseline) / requests

Captures run through rate_ladder.py (--loop-timing), so each row also keeps the
server's timing around every gap.

Usage:
    PYTHONPATH=src .venv/bin/python profiling/retune_during_stream.py \\
        --port 10000 --rates 1000 2000 --outdir ~/souk/v712_stages/retune
"""
import argparse
import glob
import json
import os
import subprocess
import sys
import time

import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient

HERE = os.path.dirname(os.path.abspath(__file__))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--addr', default='10.11.11.11')
    ap.add_argument('--port', type=int, default=10000)
    ap.add_argument('--rates', type=int, nargs='+', default=[1000, 2000])
    ap.add_argument('--kinds', nargs='+', default=['none', 'frequencies', 'amplitudes', 'phases'])
    ap.add_argument('--seconds', type=float, default=60)
    ap.add_argument('--every', type=float, default=5)
    ap.add_argument('--outdir', required=True)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)

    c = ReadoutClient(address=args.addr, request_port=args.port)
    freqs = np.asarray(c.get_tone_frequencies(), dtype=float)
    amps = np.asarray(c.get_tone_amplitudes(), dtype=float)
    phases = np.asarray(c.get_tone_phases(), dtype=float)
    send = {'frequencies': lambda: c.set_tone_frequencies(freqs),
            'amplitudes': lambda: c.set_tone_amplitudes(amps),
            'phases': lambda: c.set_tone_phases(phases)}
    print(f"{len(freqs)} tones loaded; re-sending the same values", flush=True)

    summary = []
    for rate in args.rates:
        acc_len = int(round(600000 / rate))
        base_lost = None
        for kind in args.kinds:
            label = f"retune-{kind}-{rate}"
            sub = os.path.join(args.outdir, label)
            ladder = subprocess.Popen(
                [sys.executable, '-u', os.path.join(HERE, 'rate_ladder.py'), '--addr', args.addr,
                 '--port', str(args.port), '--label', label, '--transport', 'stream',
                 '--tones', str(len(freqs)), '--seconds', str(args.seconds), '--repeats', '1',
                 '--acc-lens', str(acc_len), '--skip-tones', '--loop-timing', '--outdir', sub],
                stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            durations = []
            if kind != 'none':
                time.sleep(10)                                   # past the stream start
                t_stop = time.time() + args.seconds - 15         # clear of the capture end
                while time.time() < t_stop and ladder.poll() is None:
                    t0 = time.perf_counter()
                    send[kind]()
                    durations.append(time.perf_counter() - t0)
                    time.sleep(max(0.0, args.every - durations[-1]))
            ladder.wait()
            row = json.loads(open(glob.glob(os.path.join(sub, '*.jsonl'))[0]).readline())
            if 'error' in row:
                print(f"{rate} Hz {kind}: capture failed: {row['error']}", flush=True)
                continue
            lt = row['loop_timing']
            n = len(durations)
            if kind == 'none':
                base_lost = row['lost']
            per_req = ((row['lost'] - base_lost) / n) if (n and base_lost is not None) else None
            out = dict(rate=rate, kind=kind, requests=n, lost=row['lost'], baseline_lost=base_lost,
                       lost_per_request=per_req, torn=row['packet_error'],
                       request_ms_p50=round(1e3 * float(np.median(durations)), 1) if n else None,
                       request_ms_max=round(1e3 * float(np.max(durations)), 1) if n else None,
                       outside_max_ms=round(lt['outside']['max'] / 1e3, 2),
                       period_ms=round(lt['period_us'] / 1e3, 3))
            summary.append(out)
            print(json.dumps(out), flush=True)
    with open(os.path.join(args.outdir, 'retune_summary.json'), 'w') as f:
        json.dump(summary, f, indent=1)


if __name__ == '__main__':
    main()
