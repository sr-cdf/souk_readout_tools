#!/usr/bin/env python3
"""
Dump the real AXI-lite byte offsets for the control buffer (write target) and
the accumulator dout (read target), so the profilers can aim at live firmware
regions instead of offset 0.

These offsets are firmware/config dependent, so we read them from a live
readout object rather than hardcoding. Run this on the board, then paste the
printed --write-addr / --read-addr / --tones / --read-chans into the
profilers, e.g.:

    sudo ~/py3.12-venv/bin/python dump_addrs.py --config my_config.yaml
    sudo ./devmem_profile --dev /dev/mem --write-addr 0x.. --read-addr 0x.. \
         --tones N --read-chans N --out devmem_profile_c_hw.txt

What it prints:
  - write-addr : base of the TX control buffer (buffer 0). A control-buffer
    write in the real code targets this region
    (firmware_lib.write_control_buffer_data_fast / _get_control_buffer_addresses).
  - read-addr  : base_addr of accumulator dout0
    (firmware_lib.get_fast_read_params -> 'base_addr').
  - tones      : mixer.n_chans (words written per buffer = tones*CONTROL_N_WORDS).
  - read-chans : acc.n_chans (read-back is read-chans*2 int32 words).

NOTE: pointing the profiler's WRITE at the control buffer is safe in the sense
that the control buffer is exactly what a real sweep overwrites every step; it
will, however, change your current tone settings. Re-apply your config after
profiling if needed.
"""

import argparse


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", required=True,
                   help="firmware config yaml (same one you pass to the readout)")
    p.add_argument("--pipeline-id", type=int, default=0)
    p.add_argument("--lo", choices=["tx", "rx"], default="tx",
                   help="which control buffer to use as the write target")
    args = p.parse_args()

    from souk_readout_tools import firmware_lib

    r_fast = firmware_lib.create_fast_readout_interface(
        args.config, pipeline_id=args.pipeline_id)

    # write target: control buffer base (buffer 0)
    addrs = firmware_lib._get_control_buffer_addresses(r_fast)
    write_addr = int(addrs[args.lo])
    n_tones = int(r_fast.mixer.n_chans)
    control_n_words = int(r_fast.mixer._CONTROL_N_WORDS)
    n_serial = int(r_fast.mixer._n_serial_chans)
    buf_size = n_serial * control_n_words * 4

    # read target: accumulator dout0 base
    frp = firmware_lib.get_fast_read_params(r_fast)
    read_addr = int(frp["base_addr"])
    nbranch = int(frp["nbranch"])
    acc = frp["acc"]
    read_chans = int(acc.n_chans)

    print("\n=== live firmware AXI-lite offsets ===")
    print(f"control buffer ({args.lo}) base : 0x{write_addr:08x}  ({write_addr})")
    print(f"  CONTROL_N_WORDS={control_n_words}  n_serial_chans={n_serial}  "
          f"buf_size={buf_size} bytes (one buffer)")
    print(f"accumulator dout0 base      : 0x{read_addr:08x}  ({read_addr})")
    print(f"  nbranch={nbranch}  (profiler models the nbranch==1 contiguous read)")
    print(f"mixer.n_chans (tones)       : {n_tones}")
    print(f"acc.n_chans  (read chans)   : {read_chans}")

    if nbranch != 1:
        print("\nWARNING: nbranch != 1 on this firmware. The real read is split "
              "across {0} interleaved branches; the profiler models a single "
              "contiguous read of read-chans*2 words from read-addr, which is a "
              "close but not exact match.".format(nbranch))

    print("\n--- ready-to-paste profiler args ---")
    print(f"--dev /dev/mem --write-addr 0x{write_addr:08x} "
          f"--read-addr 0x{read_addr:08x} --tones {n_tones} "
          f"--read-chans {read_chans}\n")


if __name__ == "__main__":
    main()
