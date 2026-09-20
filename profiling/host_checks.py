#!/usr/bin/env python3
"""
Host-side correctness checks for a firmware stage (tests B6-B8 in
doc/v712-notes.md). Run against a live readout server; no root.

    PYTHONPATH=src .venv/bin/python profiling/host_checks.py peripherals --out b6.json
    PYTHONPATH=src .venv/bin/python profiling/host_checks.py sync --out b7.json
    PYTHONPATH=src .venv/bin/python profiling/host_checks.py sweep \\
        --centers res_hz.npy --span 400e3 --out b8.json [--reference b8_S0.json]

peripherals  B6: after a boot-file change, dump the clock, timing, RF front-end,
             LNA, sensor and FPGA sections of get_info() and list every flag
             that looks like a lock / ready / error / status field.
sync         B7: a timed sync round trip on the current clock: check readiness,
             arm 5 s ahead, wait for it to fire, then confirm the firmware's
             last-sync time equals the armed target and the TT is aligned.
             A sync restarts accumulations: don't run it during a capture.
             Needs timing-monitor running: readiness (ptp_ready) is read from it.
sweep        B8: sweep the given resonator centres, fit each with
             fitting.batch_fit, save fr / Qi / Qc per tone, and with
             --reference compare against an earlier run of the same resonators
             (fr in ppm, Qi and Qc as ratios). Needs the cryostat cold, and a
             reference from the SAME cooldown to be meaningful.
"""

import argparse
import json
import time

import numpy as np
from souk_readout_tools.client.readout_client import ReadoutClient

FLAG_WORDS = ('lock', 'ready', 'error', 'status', 'ok', 'supported', 'fault', 'alarm')


def flags(d, prefix=''):
    """Flatten a nested dict to the entries whose key looks like a health flag."""
    out = {}
    if isinstance(d, dict):
        for k, v in d.items():
            name = f'{prefix}.{k}' if prefix else str(k)
            if isinstance(v, (dict, list)):
                out.update(flags(v, name))
            elif any(w in str(k).lower() for w in FLAG_WORDS):
                out[name] = v
    elif isinstance(d, list):
        for i, v in enumerate(d):
            out.update(flags(v, f'{prefix}[{i}]'))
    return out


def peripherals(client, args):
    sections = ['clock', 'timing', 'rf_frontend', 'lna', 'rfsoc_sensors', 'fpga']
    info = client.get_info(sections=sections)
    if isinstance(info, list):                   # one entry per requested section
        info = dict(zip(sections, info))
    f = flags(info)
    for k, v in f.items():
        print(f'  {k:70s} {v}')
    return dict(flags=f, info=info)


def sync(client, args):
    res = dict(needed=client.timed_sync_needed())
    ready = client.timed_sync_ready(seconds_from_now=args.lead)
    res['ready'] = ready
    if not ready.get('ready'):
        print('not ready:', ready.get('reasons'))
        res['passed'] = False
        return res
    arm = client.timed_sync_arm(seconds_from_now=args.lead, wait=True)
    res['arm'] = arm
    time.sleep(1.0)
    chk = client.timed_sync_check()
    res['check'] = chk
    fired_on_target = (chk.get('last_sync_unix_s') is not None and
                       abs(chk['last_sync_unix_s'] - arm['target_tt_unix_s']) < 1e-3)
    res['passed'] = bool(fired_on_target and chk.get('aligned'))
    print(f"target {arm.get('target_tt_utc')}  last sync {chk.get('last_sync_utc')}  "
          f"aligned {chk.get('aligned')}  pps offset {chk.get('pps_boundary_offset_s')}  "
          f"-> {'PASS' if res['passed'] else 'FAIL'}")
    return res


def sweep(client, args):
    from souk_readout_tools import fitting
    centers = np.load(args.centers)
    phases = np.asarray(client.generate_newman_phases(centers), float)
    client.perform_sweep(centers, args.span, args.points, args.samples_per_point,
                         phases=phases, wait=True)
    fits = fitting.batch_fit(client.get_sweep_data(), verbose=False)
    rows = []
    for fit in fits:
        row = fitting.fit_result_summary_row(fit, include_message=False)
        row['tone_index'] = int(getattr(fit, 'tone_index', len(rows)))
        rows.append(row)
    res = dict(centers=centers.tolist(), fits=rows)
    if args.reference:
        ref = {r['tone_index']: r for r in json.load(open(args.reference))['fits']}
        comp = []
        for r in rows:
            o = ref.get(r['tone_index'])
            if o is None:
                continue
            comp.append(dict(tone_index=r['tone_index'],
                             fr_ppm=1e6 * (r['fr'] - o['fr']) / o['fr'],
                             Qi_ratio=r['Qi'] / o['Qi'], Qc_ratio=r['Qc'] / o['Qc']))
        res['comparison'] = comp
        if comp:
            a = {k: np.array([c[k] for c in comp]) for k in ('fr_ppm', 'Qi_ratio', 'Qc_ratio')}
            print(f"{len(comp)} tones vs reference: fr median {np.median(a['fr_ppm']):+.2f} ppm, "
                  f"Qi ratio median {np.median(a['Qi_ratio']):.3f}, "
                  f"Qc ratio median {np.median(a['Qc_ratio']):.3f}")
    for r in rows[:10]:
        print(f"  tone {r['tone_index']:4d}  fr {r['fr']/1e6:11.6f} MHz  Qi {r['Qi']:9.0f}  Qc {r['Qc']:9.0f}")
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('check', choices=['peripherals', 'sync', 'sweep'])
    ap.add_argument('--addr', default='10.11.11.11')
    ap.add_argument('--port', type=int, default=10000)
    ap.add_argument('--lead', type=float, default=5.0, help='sync: seconds ahead to arm')
    ap.add_argument('--centers', help='sweep: .npy of resonator centre frequencies (Hz)')
    ap.add_argument('--span', type=float, default=400e3)
    ap.add_argument('--points', type=int, default=401)
    ap.add_argument('--samples-per-point', type=int, default=10)
    ap.add_argument('--reference', help='sweep: JSON from an earlier sweep run')
    ap.add_argument('--out', default=None)
    args = ap.parse_args()

    client = ReadoutClient(address=args.addr, request_port=args.port)
    res = dict(check=args.check, time=time.strftime('%Y-%m-%dT%H:%M:%S'))
    res.update({'peripherals': peripherals, 'sync': sync, 'sweep': sweep}[args.check](client, args))
    if args.out:
        with open(args.out, 'w') as f:
            json.dump(res, f, indent=1, default=str)
        print(f'wrote {args.out}')


if __name__ == '__main__':
    main()
