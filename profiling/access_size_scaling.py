#!/usr/bin/env python3
"""
Measure the fixed and per-word cost of a devmem access, to decide whether
sparse (partial) accumulator reads and sparse control-buffer writes are
actually faster than transferring the whole buffer.

Every access costs roughly  t = F + n_words * W  --  a fixed per-transaction
cost F plus a per-word streaming cost W. Which access strategy wins is then
arithmetic:

    full      : F + 4096*W                (2048 complex chans, re+im int32)
    span      : F + span_words*W          (one read of min(idx)..max(idx))
    per-tone  : M * (F + 2*W)             (M separate 1-channel reads)

so `span` beats `full` whenever the tones are clustered, and `per-tone` only
beats `span` when M*F is smaller than the span it would have to cover. This
script measures F and W and prints the crossovers, rather than guessing them.

Writes matter too: on v7.12 a bulk write streams at ~4.8 ns/word, faster than
any single posted store, so scattering M tones may well be SLOWER than simply
rewriting the whole buffer. Same fit, same question.

Usage (on the board, as root):
    sudo ~/py3.12-venv/bin/python access_size_scaling.py \
        --write-addr 0x13000000 --read-addr 0x10000000

Offsets come from fpg_addrs.py -- they move every firmware generation.
"""

import argparse
import mmap
import os
import statistics
import sys
import time

import numpy as np

AXIL_OFFSET = 0xA0000000
MAP_SIZE = 0x20000000          # 512 MiB -- must cover the v7.12 AXI4 BRAMs
MAX_CHANS = 2048               # N_TONE
CONTROL_N_WORDS = 4            # u32 per tone in the LO control buffer
REPS = 400                     # timed repeats per point (median is reported)


def make_mmap(dev_path):
    """Map the device the same way LocalMemTransport does."""
    if dev_path == "/dev/mem":
        fd = os.open(dev_path, os.O_RDWR | os.O_SYNC)
        mm = mmap.mmap(fd, MAP_SIZE, offset=AXIL_OFFSET, flags=mmap.MAP_SHARED,
                       prot=mmap.PROT_READ | mmap.PROT_WRITE)
        return mm, fd
    fd = os.open(dev_path, os.O_RDWR | os.O_CREAT, 0o644)
    if os.fstat(fd).st_size < MAP_SIZE:
        os.ftruncate(fd, MAP_SIZE)
    mm = mmap.mmap(fd, MAP_SIZE, flags=mmap.MAP_SHARED,
                   prot=mmap.PROT_READ | mmap.PROT_WRITE)
    return mm, fd


def median_us(fn, reps=REPS, warmup=20):
    for _ in range(warmup):
        fn()
    samples = []
    for _ in range(reps):
        t0 = time.perf_counter()
        fn()
        samples.append((time.perf_counter() - t0) * 1e6)
    return statistics.median(samples)


def fit_F_W(sizes_words, times_us):
    """Least-squares fit of t = F + n*W. Returns (F_us, W_ns_per_word)."""
    n = np.asarray(sizes_words, dtype=float)
    t = np.asarray(times_us, dtype=float)
    W, F = np.polyfit(n, t, 1)
    return F, W * 1000.0


def read_scaling(mm, read_addr):
    """Time one contiguous read of n channels, for n over a wide range."""
    print("\n=== contiguous READ vs size ===")
    print(f"{'chans':>6} {'words':>6} {'bytes':>7} {'median us':>10} {'ns/word':>9}")
    sizes, times = [], []
    n = 1
    while n <= MAX_CHANS:
        nwords = n * 2                       # re + im, int32 each
        nbytes = nwords * 4
        fn = (lambda a=read_addr, b=nbytes:
              np.frombuffer(mm[a:a + b], dtype='<i4'))
        us = median_us(fn)
        sizes.append(nwords); times.append(us)
        print(f"{n:>6} {nwords:>6} {nbytes:>7} {us:>10.2f} {us*1000/nwords:>9.1f}")
        n *= 2
    F, W = fit_F_W(sizes, times)
    print(f"\nfit: t[us] = {F:.2f} + n_words * {W/1000:.4f}   "
          f"(F = {F:.2f} us fixed, W = {W:.1f} ns/word)")
    return F, W


def multi_read_scaling(mm, read_addr, F, W):
    """M separate 1-channel reads -- the fully-scattered case.

    Offsets stay INSIDE the accumulator buffer (2048 chans x 8 B = 16 KiB).
    Reading past it would hit unrelated firmware registers, some of which are
    clear-on-read.
    """
    print("\n=== M separate 1-channel READS (scattered tones) ===")
    print(f"{'M':>6} {'median us':>10} {'us/read':>9} {'vs full-buffer':>15}")
    full_us = F + 4096 * W / 1000.0
    for M in (1, 2, 4, 8, 16, 32, 64, 128, 256):
        stride_chans = max(1, MAX_CHANS // M)              # spread evenly, stay in bounds
        offs = [read_addr + (i * stride_chans) * 8 for i in range(M)]
        assert offs[-1] + 8 <= read_addr + MAX_CHANS * 8, "scatter walked out of the buffer"
        def fn(o=offs):
            for a in o:
                np.frombuffer(mm[a:a + 8], dtype='<i4')
        us = median_us(fn, reps=max(40, REPS // max(1, M)))
        print(f"{M:>6} {us:>10.2f} {us/M:>9.2f} "
              f"{'FASTER' if us < full_us else 'slower':>15}")
    print(f"(full-buffer read for comparison: {full_us:.2f} us)")


def write_scaling(mm, write_addr):
    """Bulk contiguous write vs scattered per-tone writes, both for M tones.

    Mirrors the real sparse path: each selected tone contributes
    CONTROL_N_WORDS *contiguous* words, and the selected tones are spread
    evenly across the buffer. Stays inside one LO slot
    (MAX_CHANS * CONTROL_N_WORDS words).

    NOTE: this overwrites the LO control buffer. Stop the readout server and
    re-set your tones afterwards, or pass --no-write.
    """
    print("\n=== WRITE: bulk vs scattered ===")
    print(f"{'M tones':>8} {'bulk us':>9} {'scatter us':>11} {'winner':>9}")
    flat = np.frombuffer(memoryview(mm), dtype='<u4')        # the sparse path's own view
    base_word = write_addr // 4
    slot_words = MAX_CHANS * CONTROL_N_WORDS
    for M in (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1000, 2048):
        payload = np.arange(M * CONTROL_N_WORDS, dtype='<u4')
        nbytes = M * CONTROL_N_WORDS * 4
        bulk = median_us(lambda a=write_addr, b=nbytes, p=payload:
                         mm.__setitem__(slice(a, a + b), p.tobytes()))
        # M tones spread evenly over the slot, 4 contiguous words each
        stride = max(1, MAX_CHANS // M) * CONTROL_N_WORDS
        starts = base_word + np.arange(M) * stride
        idx = starts.repeat(CONTROL_N_WORDS) + np.tile(np.arange(CONTROL_N_WORDS), M)
        assert idx.max() < base_word + slot_words, "scatter walked out of the slot"
        scat = median_us(lambda i=idx, p=payload: flat.__setitem__(i, p))
        print(f"{M:>8} {bulk:>9.2f} {scat:>11.2f} "
              f"{'bulk' if bulk < scat else 'scatter':>9}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dev", default="/dev/mem")
    ap.add_argument("--read-addr", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--write-addr", type=lambda s: int(s, 0), required=True)
    ap.add_argument("--no-write", action="store_true",
                    help="skip the write tests (they clobber the control buffer)")
    args = ap.parse_args()

    mm, fd = make_mmap(args.dev)
    try:
        F, W = read_scaling(mm, args.read_addr)
        multi_read_scaling(mm, args.read_addr, F, W)
        if not args.no_write:
            write_scaling(mm, args.write_addr)
        print("\n--- crossovers ---")
        full_us = F + 4096 * W / 1000.0
        span_break = (full_us - F) / (W / 1000.0)
        print(f"full-buffer read            : {full_us:.2f} us")
        print(f"span read beats full below  : {span_break:.0f} words "
              f"({span_break/2:.0f} channels of span)")
        print(f"per-tone read beats span when M*(F + 2W) < F + span*W, "
              f"i.e. M < ~{(full_us / (F + 2*W/1000.0)):.1f} for a full-width span")
    finally:
        mm.close(); os.close(fd)


if __name__ == "__main__":
    sys.exit(main())
