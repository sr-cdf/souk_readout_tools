#!/usr/bin/env python3
"""
Summarise rate_ladder.py --loop-timing rows: per capture, where each frame's
time went (server side) against the accumulation period, and the timing rows
around every gap or torn read.

Usage:
    python profiling/loop_timing_summary.py ~/souk/v712_stages/dual/baseline/p0/*.jsonl [...]
"""
import json
import sys


def main(paths):
    for path in paths:
        print(f"\n== {path}")
        print(f"{'rate':>5} {'rep':>3} {'lost':>5} {'torn':>4} {'period':>7} | p50 / p99.9 / max us:"
              f" {'wait':>17} {'read':>17} {'frame':>17} {'outside':>19} {'busy':>19}")
        for line in open(path):
            row = json.loads(line)
            lt = row.get('loop_timing')
            if 'error' in row or not isinstance(lt, dict) or 'busy' not in lt:
                print(f"{row['rate']:>5.0f} {row['repeat']:>3}  no loop timing ({row.get('error', '')})")
                continue
            cols = ' '.join(f"{lt[k]['p50']:5.0f}/{lt[k]['p999']:5.0f}/{lt[k]['max']:5.0f}"
                            for k in ('wait', 'read', 'frame', 'outside', 'busy'))
            print(f"{row['rate']:>5.0f} {row['repeat']:>3} {row['lost']:>5} {row['packet_error']:>4} "
                  f"{lt['period_us']:7.0f} | {cols}")
            for ev in lt.get('events', [])[:6]:
                if len(ev['rows']) < 2:               # the capture's first frame
                    continue
                (pw, pr, pf, po, pm, pt), (w, r, f, o, m, t) = ev['rows']
                print(f"      frame {ev['frame']:>7}: before wait {pw:.0f} read {pr:.0f} frame {pf:.0f} "
                      f"outside {po:.0f} torn {pt} | this wait {w:.0f} read {r:.0f} frame {f:.0f} "
                      f"outside {o:.0f} missed {m} torn {t}")


if __name__ == '__main__':
    main(sys.argv[1:])
