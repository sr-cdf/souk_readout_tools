#!/usr/bin/env python3
"""
Sample-rate ladder over the stream socket or get_samples (test B4). Runs on the
host against a live readout server; no root.

For each acc_len it captures N seconds and reports, from packet_counter:

  delivered/s   rate * (1 - loss); no wall clock, so connect/setup time is
                irrelevant
  loss %        samples missing / samples spanned
  startup       gaps below index 200 (the TCP send-buffer ramp) -- excluded
                from everything below, it is not monotonic in loop time
  small / large steady-state gaps of <=3 / >3 samples: loop overrun vs an
                external stall (e.g. SD-card writes). Different causes.
  max stall ms  largest steady gap in time
  packet_error  reads during which the accumulation counter moved

Tones: 1000 by default, linspace(500, 2000 MHz) + default_rng(0) +/-100 kHz,
Newman phases, unit amplitudes -- the recipe of every ladder in
doc/v712-notes.md. Record the server-side state separately with
board_state.py; this script stores get_info() versions/fpga alongside.

Usage:
    PYTHONPATH=src .venv/bin/python profiling/rate_ladder.py --label S0 \\
        --transport stream samples --repeats 3 --outdir ~/souk/v712_stages/S0
"""

import argparse
import gc
import json
import os
import signal
import tempfile
import time

import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient

STARTUP = 200
# rate = 600000 / acc_len. Dense between 1200 and 2400 Hz -- 2-4x the 600 Hz
# campaign rate, the range that matters -- then sparse up to 6000 Hz.
ACC_LENS = [1000, 750, 600, 500, 400, 333, 300, 250, 200, 150, 120, 100]
#           600   800  1000  1200 1500 1802 2000 2400 3000 4000 5000 6000 Hz


class _Timeout(KeyboardInterrupt):
    """receive_stream catches KeyboardInterrupt, so inherit from it."""


def _alarm(sig, frame):
    raise _Timeout()


def analyse(data, rate):
    pc = np.asarray(data['packet_counter']).astype(np.int64)
    d = np.diff(pc)
    idx = np.flatnonzero(d > 1)
    sizes = d[idx] - 1
    steady = idx >= STARTUP
    s = sizes[steady]
    lost = int(sizes.sum())
    loss = lost / (len(d) + lost) if len(d) else float('nan')
    n_steady = max(len(d) - STARTUP, 0)
    steady_loss = s.sum() / (n_steady + s.sum()) if n_steady else float('nan')
    return dict(
        frames=int(len(pc)), lost=lost, loss_pct=100 * loss,
        steady_loss_pct=100 * float(steady_loss),
        delivered_per_s=rate * (1 - loss),
        startup_gaps=int((~steady).sum()), startup_lost=int(sizes[~steady].sum()),
        small_gaps=int((s <= 3).sum()), small_lost=int(s[s <= 3].sum()),
        large_gaps=int((s > 3).sum()), large_lost=int(s[s > 3].sum()),
        max_stall_ms=float(s.max() / rate * 1e3) if s.size else 0.0,
        packet_error=int(np.asarray(data['packet_error']).astype(bool).sum()),
        first_gaps=idx[:12].tolist())


def capture(client, transport, tones, seconds, rate, tmpdir, warmup=None):
    if transport == 'samples':
        return client.parse_samples(client.get_samples(int(rate * seconds), warmup=warmup))
    fn = os.path.join(tmpdir, 'stream.bin')
    info = client.get_info('all')          # before the stream starts: see receive_stream(info=)
    client.enable_stream()
    signal.setitimer(signal.ITIMER_REAL, seconds)
    try:
        client.receive_stream(num_tones=tones, filename=fn, info=info)
    except KeyboardInterrupt:
        pass
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
        client.disable_stream()
    try:
        return ReadoutClient.parse_stream(fn)
    finally:
        os.remove(fn)                     # ~0.7 GB per point at 3 kHz


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--addr', default='10.11.11.11')
    ap.add_argument('--port', type=int, default=10000)
    ap.add_argument('--label', required=True, help='stage label, e.g. S0 or S3-zdma')
    ap.add_argument('--transport', nargs='+', choices=['stream', 'samples'],
                    default=['stream', 'samples'])
    ap.add_argument('--tones', type=int, default=1000)
    ap.add_argument('--seconds', type=float, default=30.0)
    ap.add_argument('--acc-lens', type=int, nargs='+', default=ACC_LENS)
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--skip-tones', action='store_true',
                    help='reuse the tones already loaded')
    ap.add_argument('--loop-timing', action='store_true',
                    help="store the server's per-capture get_info('loop_timing') in each row "
                         "(servers with the loop-timing instrumentation only)")
    ap.add_argument('--warmup', type=int, default=None,
                    help="get_samples: accumulations the server discards first (server default 50)")
    ap.add_argument('--outdir', required=True)
    args = ap.parse_args()
    os.makedirs(args.outdir, exist_ok=True)
    out = os.path.join(args.outdir, f'ladder_{args.label}_{args.tones}tones.jsonl')

    client = ReadoutClient(address=args.addr, request_port=args.port)
    info = client.get_info(sections=['versions', 'fpga'])
    with open(os.path.join(args.outdir, f'ladder_{args.label}_info.json'), 'w') as f:
        json.dump(info, f, indent=1, default=str)
    if not args.skip_tones:
        rng = np.random.default_rng(0)
        freqs = np.linspace(500e6, 2000e6, args.tones, endpoint=False)
        freqs = freqs + rng.uniform(-100e3, 100e3, args.tones)
        client.set_tone_frequencies(freqs)
        client.set_tone_amplitudes(np.ones(args.tones))
        client.set_tone_phases(np.asarray(client.generate_newman_phases(freqs), float))
    signal.signal(signal.SIGALRM, _alarm)

    print(f"\n{'transport':>9} {'rate':>6} {'rep':>3} {'deliv/s':>8} {'loss %':>7} "
          f"{'steady%':>7} {'startup':>7} {'small':>6} {'large':>5} {'stall ms':>8} {'pkt_err':>7}")
    with tempfile.TemporaryDirectory(dir=args.outdir) as tmpdir:   # not tmpfs: ~1.5 GB/point
        for transport in args.transport:
            for acc in args.acc_lens:
                client.set_acc_len(acc)
                rate = float(client.get_sample_rate())
                for rep in range(args.repeats):
                    row = dict(label=args.label, transport=transport, tones=args.tones,
                               acc_len=acc, rate=rate, repeat=rep, seconds=args.seconds,
                               time=time.strftime('%Y-%m-%dT%H:%M:%S'))
                    try:
                        row.update(analyse(capture(client, transport, args.tones,
                                                   args.seconds, rate, tmpdir, args.warmup), rate))
                        row['warmup'] = args.warmup
                        if args.loop_timing:
                            row['loop_timing'] = client.get_info(sections='loop_timing')
                        print(f"{transport:>9} {rate:>6.0f} {rep:>3} "
                              f"{row['delivered_per_s']:>8.1f} {row['loss_pct']:>7.3f} "
                              f"{row['steady_loss_pct']:>7.3f} {row['startup_gaps']:>7} {row['small_gaps']:>6} "
                              f"{row['large_gaps']:>5} {row['max_stall_ms']:>8.1f} "
                              f"{row['packet_error']:>7}")
                    except Exception as exc:
                        row['error'] = repr(exc)
                        print(f"{transport:>9} {rate:>6.0f} {rep:>3}  FAILED: {exc!r}")
                    with open(out, 'a') as f:
                        f.write(json.dumps(row) + '\n')
                    gc.collect()
    print(f"\nrows appended to {out}")


if __name__ == '__main__':
    main()
