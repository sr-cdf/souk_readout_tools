#!/usr/bin/env python3
"""
Board-side correctness checks for a firmware stage (test B5 in doc/v712-notes.md).

Each check reads back what it wrote and restores what it changed. Run as root
with both readout servers stopped (they share the buffers and the bus):

    sudo systemctl stop readout_server_0 readout_server_1
    sudo ~/py3.12-venv/bin/python board_checks.py --out stages/S3/b5_checks.json
    sudo systemctl start readout_server_0 readout_server_1

Checks:
  transports   katcp and devmem read the SAME accumulation bit for bit. The
               accumulation is slowed to 2 Hz for the check (restored after),
               so a katcp read (~80 ms) fits inside one accumulation and
               acc_cnt proves both reads saw the same one. Stronger than
               comparing noise statistics, and needs no tone.
  inmap        PSB (TX) reorder map: slow write -> fast and slow reads agree;
               fast write -> slow read agrees. Original map restored.
  sparse       LO control-word writes (through the RAM copy of the registers)
               change exactly the intended tone's words: tx/rx x ping-pong
               buf 0/1 x slot 0/1, and the copy matches the firmware after.
               Restored.
  katcp_range  katcp writes accepted at a low AXI-lite offset (the RX channel
               map) and at the highest register (p1 TX LO control1): each
               writes back the value already there, so nothing changes, but
               tcpborphserver's bounds check (raw.c:944/1203) is exercised.
"""

import argparse
import json
import logging
import os
import time

import numpy as np
from souk_readout_tools import firmware_lib as fl

logging.disable(logging.WARNING)
FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")


def wait_new_acc(p, timeout_s=5.0):
    """Return the accumulation counter just after it next changes."""
    c0 = fl.read_acc_cnt_fast(p)
    t0 = time.time()
    while fl.read_acc_cnt_fast(p) == c0:
        if time.time() - t0 > timeout_s:
            raise RuntimeError("accumulation counter did not advance")
        time.sleep(0.001)
    return fl.read_acc_cnt_fast(p)


def check_transports(r, r_fast, tries=5):
    acc = r.accumulators[0]
    p = fl.get_fast_read_params(r_fast)
    mm = r_fast.accumulators[0].host.transport.axil_mm
    base, nbytes = p['base_addr'], p['nbytes']
    # Restore the raw register: control_sw's get_acc_len() is 4x what
    # set_acc_len() takes. Every acc_len change needs a master reset + sync, as
    # the server does -- without it the accumulator stops (2026-09-16: froze
    # pipeline 0 until a sw_sync, and hung the server's next tone set).
    old = r_fast.mixer.read_uint('acc_len')
    r_fast.mixer.set_acc_len(76800)                  # ~0.5 s per accumulation
    r.sync.sw_sync(mrst=True)
    try:
        for attempt in range(tries):
            c0 = wait_new_acc(p)
            dev = np.frombuffer(mm[base:base + nbytes], dtype='<i4').copy()
            kat = np.frombuffer(acc.read('dout0', nbytes), dtype='>i4')
            c1 = fl.read_acc_cnt_fast(p)
            if c0 == c1:
                same = bool(np.array_equal(dev, kat))
                return dict(passed=same, acc_cnt=int(c0), attempt=attempt,
                            words=int(dev.size), nonzero_words=int(np.count_nonzero(dev)),
                            differing_words=int(np.count_nonzero(dev != kat)))
        return dict(passed=False, reason=f"accumulation changed during all {tries} tries")
    finally:
        r_fast.mixer.write_int('acc_len', old)
        r.sync.sw_sync(mrst=True)


def check_inmap(r, r_fast):
    orig = np.asarray(fl.psb_chanselect_get_channel_inmap_slow(r)).copy()
    rng = np.random.default_rng(1)
    out = {}
    try:
        a = rng.permutation(orig)                    # same values, new order: a valid map
        fl.psb_chanselect_set_channel_inmap_slow(r, a)
        fast_read = np.asarray(fl.psb_chanselect_get_channel_inmap(r_fast))
        slow_read = np.asarray(fl.psb_chanselect_get_channel_inmap_slow(r))
        out['slow_write_fast_read'] = bool(np.array_equal(fast_read, a))
        out['slow_write_slow_read'] = bool(np.array_equal(slow_read, a))
        b = rng.permutation(orig)
        fl.psb_chanselect_set_channel_inmap(r_fast, b)
        out['fast_write_slow_read'] = bool(np.array_equal(
            np.asarray(fl.psb_chanselect_get_channel_inmap_slow(r)), b))
    finally:
        fl.psb_chanselect_set_channel_inmap_slow(r, orig)
        out['restored'] = bool(np.array_equal(
            np.asarray(fl.psb_chanselect_get_channel_inmap_slow(r)), orig))
    out['passed'] = all(out.values())
    return out


def check_sparse(r_fast, tone=5):
    """One tone's 4 words rewritten through the RAM copy of the control registers
    (which pushes whole 64-byte chunks, so its neighbours are rewritten too, with
    their current values); neighbours and the other LO must not move, and the
    copy must match the firmware afterwards."""
    m = r_fast.mixer
    nw = m._CONTROL_N_WORDS
    addrs = fl._get_control_buffer_addresses(r_fast)
    buf_size = m._n_serial_chans * nw * 4
    mm = m.host.transport.axil_mm
    fields = {'phase_steps': m._PHASE_INC_WORD_OFFSET, 'ri_steps': m._RI_STEP_WORD_OFFSET,
              'phase_offsets': m._PHASE_OFFSET_WORD_OFFSET, 'scaling': m._SCALE_WORD_OFFSET}
    fl.get_control_shadow(r_fast, refresh=True)
    rows, ok = [], True
    for lo in ('tx', 'rx'):
        other_lo = 'rx' if lo == 'tx' else 'tx'
        for buf in (0, 1):
            for slot in (0, 1):
                def words(l, first, n):
                    a = addrs[(l, buf)] + buf_size * slot + first * 4
                    return np.frombuffer(mm[a:a + n * 4], dtype='<u4').copy()
                def put(w):
                    v = {lo: {f: w[k:k + 1] for f, k in fields.items()}, other_lo: {}}
                    fl.write_control_buffer_data_fast(r_fast, buf, v, slice(tone, tone + 1), slot=slot)
                around = words(lo, (tone - 1) * nw, 3 * nw)      # tones n-1, n, n+1
                other = words(other_lo, (tone - 1) * nw, 3 * nw)
                new = (around[nw:2 * nw] ^ 0x5A5A5A5A).astype('<u4')
                try:
                    put(new)
                    after = words(lo, (tone - 1) * nw, 3 * nw)
                    expect = around.copy(); expect[nw:2 * nw] = new
                    good = bool(np.array_equal(after, expect)) and bool(np.array_equal(
                        words(other_lo, (tone - 1) * nw, 3 * nw), other))
                finally:
                    put(around[nw:2 * nw])
                    restored = bool(np.array_equal(words(lo, (tone - 1) * nw, 3 * nw), around))
                rows.append(dict(lo=lo, buf=buf, slot=slot, isolated=good, restored=restored))
                ok &= good and restored
    stale = fl.check_control_shadow(r_fast)['stale_words']
    return dict(passed=bool(ok and stale == 0), copy_stale_words=stale, cases=rows)


def check_katcp_range(r):
    out, ok = {}, True
    for name in ('p0_chan_select_map0_map1', 'p1_mix_tx_lo0_control1'):
        try:
            val = r._cfpga.read(name, 4)
            r._cfpga.blindwrite(name, val)            # same value back: no state change
            back = r._cfpga.read(name, 4)
            out[name] = dict(write_accepted=True, unchanged=(back == val))
            ok &= back == val
        except Exception as exc:                      # e.g. "outside mapped range"
            out[name] = dict(write_accepted=False, error=repr(exc))
            ok = False
    out['passed'] = bool(ok)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pipeline", type=int, default=0)
    ap.add_argument("--only", nargs="+", choices=["transports", "inmap", "sparse", "katcp_range"])
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    r = fl.create_standard_readout_interface(FW, args.pipeline)    # katcp
    r_fast = fl.create_fast_readout_interface(FW, args.pipeline)   # devmem
    checks = {"transports": lambda: check_transports(r, r_fast),
              "inmap": lambda: check_inmap(r, r_fast),
              "sparse": lambda: check_sparse(r_fast),
              "katcp_range": lambda: check_katcp_range(r)}
    res = dict(time=time.strftime("%Y-%m-%dT%H:%M:%S"))
    for name in (args.only or checks):
        try:
            res[name] = checks[name]()
        except Exception as exc:
            res[name] = dict(passed=False, error=repr(exc))
        print(f"{name:12s} {'PASS' if res[name].get('passed') else 'FAIL'}  "
              f"{json.dumps({k: v for k, v in res[name].items() if k not in ('passed', 'cases')})}")
    if args.out:
        with open(args.out, "w") as f:
            json.dump(res, f, indent=1, default=str)


if __name__ == "__main__":
    main()
