#!/usr/bin/env python3
"""
What happens to an LO control word written over katcp by something other than
the server, now that the server writes through a RAM copy of those registers
(test plan item 7 in doc/v712-notes.md). Board side, as casper, servers running,
nothing streaming. Pipeline 1 by default.

1. get_info('control_shadow') on the server: expected clean.
2. katcp, from this process: control_sw ``mixer.set_amplitude_scale`` on one LO
   channel, TX only, live buffer, slot 0 -- the kind of write a host script makes.
3. get_info('control_shadow'): does the server report the difference?
4. With --refresh: get_tone_amplitudes(refresh_shadow_copy=True), so the copy
   adopts the katcp word.
5. set_tone_phases with the phases already loaded: a normal request that
   rewrites the same tone rows but a different field.
6. katcp read-back of the scale word: kept, or put back to the copy's value?
7. Restores the word over katcp, then refreshes the copy again if it was used.

    ~/py3.12-venv/bin/python shadow_outside_writer_check.py --out shadow_outside_writer.json
    ~/py3.12-venv/bin/python shadow_outside_writer_check.py --refresh --out shadow_outside_writer_refresh.json
"""
import argparse
import json
import logging
import os
import time

import numpy as np
from souk_readout_tools import firmware_lib as fl
from souk_readout_tools.client.readout_client import ReadoutClient

FW = os.path.expanduser(
    "~casper/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--pipeline', type=int, default=1)
    ap.add_argument('--chan', type=int, default=5, help='LO channel (row) to write')
    ap.add_argument('--scale', type=float, default=0.5)
    ap.add_argument('--refresh', action='store_true',
                    help='refresh the server copy (refresh_shadow_copy=True) before the request')
    ap.add_argument('--out', required=True)
    args = ap.parse_args()
    logging.disable(logging.WARNING)
    res = {'time': time.strftime('%Y-%m-%dT%H:%M:%S'), 'pipeline': args.pipeline,
           'chan': args.chan, 'refresh': args.refresh}

    c = ReadoutClient(address='10.11.11.11', request_port=10000 + args.pipeline)
    r = fl.create_standard_readout_interface(FW, args.pipeline)
    m = r.mixer
    buf, slot = m.get_current_buffer(), 0
    reg = f'tx_lo0_control{buf}'
    word_offset = (slot * m._n_serial_chans + args.chan) * m._CONTROL_N_WORDS + m._SCALE_WORD_OFFSET

    def word():
        return m.read_uint(reg, word_offset=word_offset)

    def shadow():
        info = c.get_info(sections='control_shadow')
        return {k: info[k] for k in ('stale', 'stale_words')} | {
            'tx': info.get(f'tx{buf}', {}).get('first')}

    phases = np.asarray(c.get_tone_phases(), dtype=float)
    original = word()
    res['buffer'] = buf
    res['1_before'] = {'word': original, 'shadow': shadow()}

    m.set_amplitude_scale(args.chan, args.scale, los=['tx'], next_buf=buf, slot=slot)
    res['2_katcp_write'] = {'word': word()}
    res['3_after_katcp'] = {'shadow': shadow()}

    if args.refresh:
        c.get_tone_amplitudes(refresh_shadow_copy=True)
        res['4_after_refresh'] = {'shadow': shadow()}

    t0 = time.perf_counter()
    c.set_tone_phases(phases)
    res['5_set_tone_phases_s'] = round(time.perf_counter() - t0, 3)
    after = word()
    res['6_after_request'] = {'word': after, 'shadow': shadow(),
                              'katcp_write_kept': after == res['2_katcp_write']['word']}

    if after != original:
        m.write_int(reg, original, word_offset=word_offset)
    if args.refresh:
        c.get_tone_amplitudes(refresh_shadow_copy=True)
    res['7_restored'] = {'word': word(), 'shadow': shadow()}
    json.dump(res, open(args.out, 'w'), indent=1)
    print(json.dumps(res, indent=1))


if __name__ == '__main__':
    main()
