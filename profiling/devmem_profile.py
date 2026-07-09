#!/usr/bin/env python3
"""
Standalone Python profiler for the devmem (mmap) access pattern used by the
fast readout path.

This reproduces, with no souk_readout_tools dependency, exactly what
firmware_lib does on the fast interface:

  - control-buffer WRITE: slice-assign bytes into an mmap of /dev/mem, i.e.
        axil_mm[start:start+length] = v.astype('<u4').tobytes()
    (see write_control_buffer_data_fast, firmware_lib.py:1947)

  - accumulator READ: slice-read bytes out of the mmap, then np.frombuffer:
        raw  = axil_mm[base_addr:base_addr+nbytes]
        dout = np.frombuffer(raw, dtype='<i4')
    (see read_accumulated_data_fast, firmware_lib.py)

A "sweep step" here is: write the tone control buffer, then read back the
accumulator. We time many steps and report per-op statistics. This is the
apples-to-apples partner of devmem_profile.c.

By default it runs against an ordinary backing file so you can profile the
pure Python/mmap mechanics on any machine. Pass --dev /dev/mem (needs root and
the real board) to measure against actual hardware AXI-lite.

Underlying mmap parameters match casperfpga LocalMemTransport:
    AXIL_OFFSET = 0xA0000000 ; MAP_SIZE = 32 MiB
(transport_localmem.py:13-14). On a backing file we map from offset 0.
"""

import argparse
import mmap
import os
import statistics
import sys
import time

import numpy as np

# --- mmap parameters, mirroring casperfpga LocalMemTransport ----------------
AXIL_OFFSET = 0xA0000000        # offset into /dev/mem for the AXI-lite region
MAP_SIZE = 32 * 1024 * 1024     # 32 MiB

# --- firmware control-buffer geometry (realistic defaults) ------------------
# These mirror the per-tone layout the mixer uses. One tone occupies
# CONTROL_N_WORDS 32-bit words; a buffer holds N_SERIAL_CHANS tones.
CONTROL_N_WORDS = 4             # words per tone (phase_inc, ri_step, phase_off, scale)
DEFAULT_N_TONES = 1000          # tones written per control-buffer write

# --- accumulator read-back geometry -----------------------------------------
# The accumulator dout is a flat array of interleaved complex samples, one per
# tone, laid out real-first:  [re0, im0, re1, im1, ...]. Each of the real and
# imaginary parts is a 32-bit signed int (firmware accumulators are built with
# dtype='>i4', is_complex=True; firmware_lib reads them as '<i4' on the fast
# mmap interface). So one tone == one complex sample == 2 int32 == 8 bytes.
# The design generates N_TONE tones, so the accumulator holds up to MAX_TONES
# complex channels (N_TONE * 8 bytes = 16 KiB for MAX_TONES=2048).
READ_DTYPE = '<i4'              # interleaved int32 real/imag pairs
READ_BYTES_PER_COMPONENT = 4    # int32 per real / per imag
MAX_TONES = 2048               # N_TONE: max complex channels the design supports


def make_mmap(dev_path):
    """Open and mmap the device/file the same way LocalMemTransport does.

    Returns (mm, fd, base_offset). For /dev/mem we map at AXIL_OFFSET; for a
    regular backing file we map at 0 (and create/size it if needed).
    """
    if dev_path == "/dev/mem":
        fd = os.open(dev_path, os.O_RDWR | os.O_SYNC)
        mm = mmap.mmap(fd, MAP_SIZE, offset=AXIL_OFFSET,
                       flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE)
        return mm, fd, 0
    # regular backing file: create and size to MAP_SIZE
    fd = os.open(dev_path, os.O_RDWR | os.O_CREAT, 0o644)
    if os.fstat(fd).st_size < MAP_SIZE:
        os.ftruncate(fd, MAP_SIZE)
    mm = mmap.mmap(fd, MAP_SIZE, flags=mmap.MAP_SHARED,
                   prot=mmap.PROT_READ | mmap.PROT_WRITE)
    return mm, fd, 0


def run(dev_path, n_steps, n_tones, n_read_chans, warmup, out_path,
        write_addr, read_addr, sync_writes=True):
    if n_read_chans > MAX_TONES:
        print(f"warning: --read-chans {n_read_chans} exceeds the design maximum "
              f"of {MAX_TONES} complex channels (N_TONE)", file=sys.stderr)
    write_words = n_tones * CONTROL_N_WORDS
    write_bytes = write_words * 4
    # each read channel is one complex sample: int32 real + int32 imag,
    # interleaved as [re0, im0, re1, im1, ...]
    read_bytes = n_read_chans * 2 * READ_BYTES_PER_COMPONENT

    mm, fd, base = make_mmap(dev_path)

    # Pre-build the payload once (we are profiling the devmem access, not the
    # numpy formatting). firmware writes v.astype('<u4').tobytes().
    payload = np.arange(write_words, dtype='<u4')
    payload_bytes = payload.tobytes()

    # Byte regions for the write (control) and read (accumulator) buffers,
    # inside the mapped window. On a backing file these default to scratch
    # offsets; for /dev/mem pass the real firmware offsets (see dump_addrs.py
    # or fpg_addrs.py) so we exercise the actual control buffer / accumulator.
    write_addr = base + write_addr
    read_addr = base + read_addr

    write_post_times = []   # seconds per write, posting only (stores handed off)
    write_sync_times = []   # seconds per write, incl. completion (drain to slave)
    read_times = []         # seconds per accumulator read
    step_times = []         # seconds per full step (write + read)

    def one_step():
        t0 = time.perf_counter()
        mm[write_addr:write_addr + write_bytes] = payload_bytes
        # posting done: the slice-assignment returns once the bytes are in the
        # CPU store buffer / interconnect posted-write FIFO — the AXI slave has
        # NOT necessarily accepted them yet, so this time can imply rates above
        # the physical bus limit.
        tw_post = time.perf_counter()
        # force completion: read back the last word we wrote, from the SAME
        # slave. A load cannot return until the posted writes ahead of it have
        # drained to the endpoint, so tw_sync includes real write completion.
        # (Python has no DSB; the same-slave read-back is the mechanism.)
        wb = mm[write_addr + write_bytes - 4:write_addr + write_bytes]
        tw_sync = time.perf_counter()
        # accumulator read: pull the interleaved int32 real/imag pairs back out
        raw = mm[read_addr:read_addr + read_bytes]   # read always after completion
        dout = np.frombuffer(raw, dtype=READ_DTYPE)
        t2 = time.perf_counter()
        # touch results so neither access is optimised away
        if (dout.size and dout[0] == 0xDEADBEEF) or wb == b'\xde\xad\xbe\xef':
            print("unreachable", file=sys.stderr)
        w_post = tw_post - t0
        w_sync = tw_sync - t0
        r = t2 - tw_sync
        # step total uses whichever write the flag selects (default: sync)
        s = (w_sync if sync_writes else w_post) + r
        return w_post, w_sync, r, s

    # warmup (page faults, cache) — not recorded
    for _ in range(warmup):
        one_step()

    t_start = time.perf_counter()
    for _ in range(n_steps):
        wp, ws, r, s = one_step()
        write_post_times.append(wp)
        write_sync_times.append(ws)
        read_times.append(r)
        step_times.append(s)
    t_total = time.perf_counter() - t_start

    # flush() is meaningless (and errors with EINVAL) on a /dev/mem mapping —
    # there is no backing file to write back. Only flush a real backing file.
    if dev_path != "/dev/mem":
        mm.flush()
    mm.close()
    os.close(fd)

    def us(x):
        return x * 1e6

    def summarize(name, times, payload_bytes):
        arr = sorted(times)
        n = len(arr)
        mean = statistics.fmean(arr)
        median = arr[n // 2]
        p99 = arr[min(n - 1, int(0.99 * n))]
        mn, mx = arr[0], arr[-1]
        thru = payload_bytes / mean / 1e6 if mean > 0 else 0.0  # MB/s
        print(f"  {name:14s} mean={us(mean):8.2f}us  median={us(median):8.2f}us  "
              f"min={us(mn):7.2f}us  p99={us(p99):8.2f}us  max={us(mx):8.2f}us  "
              f"({payload_bytes}B, {thru:7.1f} MB/s)")
        return dict(name=name, n=n, payload_bytes=payload_bytes,
                    mean_us=us(mean), median_us=us(median), min_us=us(mn),
                    p99_us=us(p99), max_us=us(mx), mb_per_s=thru)

    print(f"\n=== Python devmem profile ===")
    print(f"device           : {dev_path}")
    print(f"step total uses  : "
          f"{'completion-forced (sync)' if sync_writes else 'posted'} write")
    print(f"steps            : {n_steps} (warmup {warmup})")
    print(f"tones/write      : {n_tones}  -> {write_bytes} bytes")
    print(f"read chans       : {n_read_chans} -> {read_bytes} bytes "
          f"(int32 re/im pairs)")
    print(f"total wall time  : {t_total*1e3:.2f} ms  "
          f"({us(t_total/n_steps):.2f} us/step)")
    print(f"per-op timings:")
    s_write_post = summarize("write(posted)", write_post_times, write_bytes)
    s_write_sync = summarize("write(sync)", write_sync_times, write_bytes)
    s_read = summarize("read", read_times, read_bytes)
    s_step = summarize("step(w+r)", step_times, write_bytes + read_bytes)
    print("  (write(posted) = stores handed off; write(sync) = stores completed "
          "on the bus.")
    print("   posted can exceed the physical bus rate — trust write(sync).)")

    # --- dump raw per-step samples for inspection ---------------------------
    with open(out_path, "w") as f:
        f.write("# Python devmem profile\n")
        f.write(f"# device={dev_path} steps={n_steps} warmup={warmup} "
                f"step_write={'sync' if sync_writes else 'posted'}\n")
        f.write(f"# tones_per_write={n_tones} write_bytes={write_bytes} "
                f"read_chans={n_read_chans} read_bytes={read_bytes} "
                f"read_dtype={READ_DTYPE}\n")
        f.write(f"# total_wall_ms={t_total*1e3:.4f}\n")
        for s in (s_write_post, s_write_sync, s_read, s_step):
            f.write(f"# summary {s['name']}: mean_us={s['mean_us']:.3f} "
                    f"median_us={s['median_us']:.3f} min_us={s['min_us']:.3f} "
                    f"p99_us={s['p99_us']:.3f} max_us={s['max_us']:.3f} "
                    f"MB_s={s['mb_per_s']:.2f}\n")
        f.write("# columns: step_index  write_posted_us  write_sync_us  "
                "read_us  step_us\n")
        for i, (wp, ws, r, s) in enumerate(
                zip(write_post_times, write_sync_times, read_times, step_times)):
            f.write(f"{i}\t{us(wp):.4f}\t{us(ws):.4f}\t{us(r):.4f}\t{us(s):.4f}\n")
    print(f"\nwrote per-step samples -> {out_path}\n")


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--dev", default="./devmem_backing.bin",
                   help="device or backing file to mmap "
                        "(use /dev/mem for real hardware; default: backing file)")
    p.add_argument("--steps", type=int, default=10000,
                   help="number of sweep steps to time (default 10000)")
    p.add_argument("--tones", type=int, default=DEFAULT_N_TONES,
                   help=f"tones per control-buffer write (default {DEFAULT_N_TONES})")
    p.add_argument("--read-chans", type=int, default=DEFAULT_N_TONES,
                   help="accumulator channels (complex tones) read back per step "
                        f"(default {DEFAULT_N_TONES}, design max {MAX_TONES})")
    p.add_argument("--warmup", type=int, default=1000,
                   help="warmup steps, not recorded (default 1000)")
    p.add_argument("--out", default="devmem_profile_py.txt",
                   help="output text file for per-step samples")
    p.add_argument("--write-addr", type=lambda x: int(x, 0), default=0x000000,
                   help="byte offset within the mapped window for the control "
                        "buffer write (default 0x0; pass the real offset for "
                        "/dev/mem, see dump_addrs.py / fpg_addrs.py)")
    p.add_argument("--read-addr", type=lambda x: int(x, 0), default=0x100000,
                   help="byte offset within the mapped window for the "
                        "accumulator read (default 0x100000)")
    p.add_argument("--sync-writes", action=argparse.BooleanOptionalAction,
                   default=True,
                   help="force write completion (barrier read-back) before "
                        "stopping the write timer, and use that for the step "
                        "total (default: on). Both posted and completion-forced "
                        "write times are always measured and reported; this only "
                        "selects which feeds the step total. Use --no-sync-writes "
                        "to make the step total use the (unphysical) posted time.")
    args = p.parse_args()
    if args.write_addr == args.read_addr:
        p.error("--write-addr and --read-addr must differ (the write would "
                "clobber the region being read)")
    run(args.dev, args.steps, args.tones, args.read_chans, args.warmup,
        args.out, args.write_addr, args.read_addr, args.sync_writes)


if __name__ == "__main__":
    main()
