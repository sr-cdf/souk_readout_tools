#!/usr/bin/env python3
"""
Measure the fixed and per-word cost of one memory access, t = F + n_words * W,
for a chosen transport and target, with the latency tail, not just the median.

Why F and W: which access strategy wins is then arithmetic,

    full      : F + 4096*W                (2048 complex chans, re+im int32)
    span      : F + span_words*W          (one read of min(idx)..max(idx))
    per-tone  : M * (F + 2*W)             (M separate 1-channel reads)

and comparing F/W across firmware stages and transports says *where* a gain
came from. Every point reports p50 / p99 / p99.9 / max; the fit uses p50.

Transports (--transport):
    devmem         mmap of /dev/mem at 0xA0000000 (root) -- what LocalMemTransport does
    uio            mmap of /dev/uio0 map0, same window, no root (v7.12 image.ub only)
    zdma           PS DMA into a cached u-dma-buf (root, v7.12 image.ub). READS ONLY.
    dram-uncached  /dev/udmabufN opened O_SYNC: ordinary DRAM through an uncached
                   mapping. No fabric at all -- this is the CPU-mapping ceiling
                   that upstream quotes as ~190 MB/s.
    dram-cached    /dev/udmabufN without O_SYNC: the cached-DRAM floor.
    A backing file (--dev path/to/file, transport devmem) runs anywhere.

Targets come from fpg_addrs.py (pass it the control_sw yaml). Addresses are
byte offsets into the 0xA0000000 window; for the dram transports they are
offsets into the udmabuf and default to 0.

Usage (on the board, readout servers stopped):
    sudo ~/py3.12-venv/bin/python access_size_scaling.py --transport devmem \\
        --read-addr 0x10000000 --write-addr 0x13000000 --out acc.json
    sudo ~/py3.12-venv/bin/python access_size_scaling.py --transport devmem \\
        --read-addr 0x00058000 --max-bytes 8192 --no-write --out ref.json
The write tests overwrite the LO control buffer: re-set tones afterwards.
"""

import argparse
import importlib.util
import json
import mmap
import os
import sys
import time

import numpy as np

AXIL_OFFSET = 0xA0000000
MAP_SIZE = 0x20000000          # 512 MiB -- must cover the v7.12 AXI4 BRAMs
MAX_CHANS = 2048               # N_TONE
CONTROL_N_WORDS = 4            # u32 per tone in the LO control buffer
TRANSPORTS = ("devmem", "uio", "zdma", "dram-uncached", "dram-cached")
ZDMA_REF = os.path.expanduser(
    "~/souk-firmware/software/rfsoc_scripts/zdma_read_example.py")


def make_mmap(transport, dev, udmabuf):
    """Return (mm, fd) for the CPU-mapped transports, or (None, None) for zdma."""
    if transport == "zdma":
        return None, None
    if transport == "uio":
        fd = os.open(dev or "/dev/uio0", os.O_RDWR)
        return mmap.mmap(fd, MAP_SIZE, mmap.MAP_SHARED, offset=0), fd  # map0
    if transport.startswith("dram"):
        flags = os.O_RDWR | (os.O_SYNC if transport == "dram-uncached" else 0)
        fd = os.open(f"/dev/{udmabuf}", flags)
        with open(f"/sys/class/u-dma-buf/{udmabuf}/size") as f:
            size = int(f.read())
        return mmap.mmap(fd, size, mmap.MAP_SHARED), fd
    if dev == "/dev/mem":
        fd = os.open(dev, os.O_RDWR | os.O_SYNC)
        return mmap.mmap(fd, MAP_SIZE, offset=AXIL_OFFSET, flags=mmap.MAP_SHARED,
                         prot=mmap.PROT_READ | mmap.PROT_WRITE), fd
    fd = os.open(dev, os.O_RDWR | os.O_CREAT, 0o644)     # backing file
    if os.fstat(fd).st_size < MAP_SIZE:
        os.ftruncate(fd, MAP_SIZE)
    return mmap.mmap(fd, MAP_SIZE, flags=mmap.MAP_SHARED,
                     prot=mmap.PROT_READ | mmap.PROT_WRITE), fd


def zdma_module():
    try:
        from souk_readout_tools import zdma
        return zdma
    except ImportError:                                  # older installed package
        spec = importlib.util.spec_from_file_location("zdma_ref", ZDMA_REF)
        ref = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(ref)
        return ref


def make_reader(transport, mm, udmabuf):
    """Return read(addr, nbytes) -> int32 array, the way firmware_lib consumes it."""
    if transport != "zdma":
        return lambda a, n: np.frombuffer(mm[a:a + n], dtype='<i4')
    zd = zdma_module()
    rdr = zd.ZdmaReader(cached_buf=zd.UdmabufBuffer(udmabuf))
    return lambda a, n: np.frombuffer(rdr.read_view(AXIL_OFFSET + a, n), dtype='<i4')


def make_zdma_writer(udmabuf, dst_abs):
    """Return write(payload): stage into a u-dma-buf, then PS DMA it to dst_abs.

    Upstream's reader only copies *into* its landing buffer. A write is the same
    in-register descriptor with source and destination swapped, after a
    sync_for_device so the DMA sees what the CPU staged. It waits for DONE, so
    it is the completion-forced cost, comparable to devmem "sync".
    """
    zd = zdma_module()
    buf = zd.UdmabufBuffer(udmabuf)
    rdr = zd.ZdmaReader(cached_buf=buf)        # pins the channel on, simple mode
    sysfs = f"/sys/class/u-dma-buf/{udmabuf}"
    with open(sysfs + "/sync_direction", "w") as fh:
        fh.write("1")                          # DMA_TO_DEVICE
    size_fd = os.open(sysfs + "/sync_size", os.O_WRONLY)
    dev_fd = os.open(sysfs + "/sync_for_device", os.O_WRONLY)
    programmed = [None]

    def write(payload):
        n = len(payload)
        buf._mm[:n] = payload
        os.pwrite(size_fd, str(n).encode(), 0)
        os.pwrite(dev_fd, b"1", 0)
        if programmed[0] != n:
            for off, val in ((zd.ZDMA_CH_SRC_WORD0, buf.phys & 0xFFFFFFFF),
                             (zd.ZDMA_CH_SRC_WORD1, buf.phys >> 32),
                             (zd.ZDMA_CH_SRC_WORD2, n), (zd.ZDMA_CH_SRC_WORD3, 0),
                             (zd.ZDMA_CH_DST_WORD0, dst_abs & 0xFFFFFFFF),
                             (zd.ZDMA_CH_DST_WORD1, dst_abs >> 32),
                             (zd.ZDMA_CH_DST_WORD2, n), (zd.ZDMA_CH_DST_WORD3, 0)):
                rdr._wr(off, val)
            programmed[0] = n
        rdr._wr(zd.ZDMA_CH_CTRL2, 0x1)         # go
        t0 = time.perf_counter()
        while True:
            isr = rdr._rd(zd.ZDMA_CH_ISR)
            if isr & zd.IXR_DMA_DONE:
                break
            if isr & zd.IXR_ERRORS or time.perf_counter() - t0 > 1.0:
                rdr._wr(zd.ZDMA_CH_ISR, zd.IXR_ALL)
                programmed[0] = None
                raise RuntimeError(f"DMA write failed: ISR=0x{isr:03x} "
                                   f"STATUS=0x{rdr._rd(zd.ZDMA_CH_STATUS):x}")
        rdr._wr(zd.ZDMA_CH_ISR, zd.IXR_ALL)

    assert buf.size >= 0x20000, "u-dma-buf smaller than an LO control register"
    return write


def time_us(fn, reps, warmup=20):
    """Per-call times in microseconds."""
    for _ in range(warmup):
        fn()
    t = np.empty(reps)
    for i in range(reps):
        t0 = time.perf_counter()
        fn()
        t[i] = (time.perf_counter() - t0) * 1e6
    return t


def pct(t):
    return dict(p50=float(np.percentile(t, 50)), p99=float(np.percentile(t, 99)),
                p999=float(np.percentile(t, 99.9)), max=float(t.max()))


def fit_F_W(sizes_words, times_us):
    """Least-squares fit of t = F + n*W. Returns (F_us, W_ns_per_word)."""
    W, F = np.polyfit(np.asarray(sizes_words, float), np.asarray(times_us, float), 1)
    return float(F), float(W * 1000.0)


def read_scaling(read, read_addr, max_chans, reps):
    """Time one contiguous read of n channels, for n over a wide range."""
    print("\n=== contiguous READ vs size ===")
    print(f"{'chans':>6} {'bytes':>7} {'p50 us':>9} {'p99 us':>9} {'p99.9 us':>9} "
          f"{'max us':>9} {'ns/word':>8}")
    rows = []
    n = 1
    while n <= max_chans:
        nwords = n * 2                       # re + im, int32 each
        s = pct(time_us(lambda b=nwords * 4: read(read_addr, b), reps))
        rows.append(dict(chans=n, words=nwords, **s))
        print(f"{n:>6} {nwords*4:>7} {s['p50']:>9.2f} {s['p99']:>9.2f} {s['p999']:>9.2f} "
              f"{s['max']:>9.2f} {s['p50']*1000/nwords:>8.1f}")
        n *= 2
    F, W = fit_F_W([r['words'] for r in rows], [r['p50'] for r in rows])
    # np.frombuffer is O(1) but not free on the ARM (6.9 us); it lands entirely
    # in F. Report F with and without it: the 2026-09 v7.11 fit (8.56 us)
    # included it, the v7.12 one quoted as 1.64 us did not.
    fb = pct(time_us(lambda b=bytes(8): np.frombuffer(b, dtype='<i4'), reps))['p50']
    print(f"\nfit (p50): t[us] = {F:.2f} + n_words * {W/1000:.4f}   "
          f"(F = {F:.2f} us fixed, W = {W:.1f} ns/word, {4e3/W:.0f} MB/s)")
    print(f"np.frombuffer alone {fb:.2f} us  ->  F excluding it = {F - fb:.2f} us")
    return rows, F, W, fb


def multi_read_scaling(read, read_addr, F, W, max_chans, reps):
    """M separate 1-channel reads -- the fully-scattered case.

    Offsets stay INSIDE the target region: reading past it would hit unrelated
    firmware registers, some of which are clear-on-read.
    """
    print("\n=== M separate 1-channel READS (scattered tones) ===")
    print(f"{'M':>6} {'p50 us':>9} {'us/read':>9} {'vs full region':>15}")
    full_us = F + 2 * max_chans * W / 1000.0
    rows = []
    for M in (1, 2, 4, 8, 16, 32, 64, 128, 256):
        if M > max_chans:
            break
        offs = [read_addr + i * (max_chans // M) * 8 for i in range(M)]
        def fn(o=offs):
            for a in o:
                read(a, 8)
        us = pct(time_us(fn, max(40, reps // M)))['p50']
        rows.append(dict(M=M, p50=us))
        print(f"{M:>6} {us:>9.2f} {us/M:>9.2f} {'FASTER' if us < full_us else 'slower':>15}")
    print(f"(full-region read for comparison: {full_us:.2f} us)")
    return rows


def write_scaling(mm, write_addr, reps):
    """Bulk contiguous write (posted and completion-forced) vs scattered tones.

    posted: the slice-assign returns once the stores are handed off.
    sync  : plus a read-back of the last word written, from the same slave --
            a load cannot complete until the posted writes ahead of it have.
    scatter mirrors the sparse path: CONTROL_N_WORDS contiguous words per tone,
    tones spread evenly across one LO slot.

    NOTE: this overwrites the LO control buffer. Stop the readout servers and
    re-set your tones afterwards, or pass --no-write.
    """
    print("\n=== WRITE: bulk posted / bulk sync / scattered ===")
    print(f"{'M tones':>8} {'posted p50':>11} {'sync p50':>9} {'sync p99.9':>11} "
          f"{'scatter p50':>12} {'winner':>8}")
    flat = np.frombuffer(memoryview(mm), dtype='<u4')        # the sparse path's own view
    base_word = write_addr // 4
    rows = []
    for M in (1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1000, 2048):
        payload = np.arange(M * CONTROL_N_WORDS, dtype='<u4')
        pb = payload.tobytes()
        a, b = write_addr, write_addr + len(pb)
        posted = pct(time_us(lambda: mm.__setitem__(slice(a, b), pb), reps))
        def sync():
            mm[a:b] = pb
            mm[b - 4:b]
        synced = pct(time_us(sync, reps))
        stride = max(1, MAX_CHANS // M) * CONTROL_N_WORDS
        idx = (base_word + np.arange(M) * stride).repeat(CONTROL_N_WORDS) \
            + np.tile(np.arange(CONTROL_N_WORDS), M)
        assert idx.max() < base_word + MAX_CHANS * CONTROL_N_WORDS, "scatter left the slot"
        scat = pct(time_us(lambda i=idx, p=payload: flat.__setitem__(i, p), reps))
        rows.append(dict(tones=M, words=M * CONTROL_N_WORDS, posted=posted,
                         sync=synced, scatter=scat))
        print(f"{M:>8} {posted['p50']:>11.2f} {synced['p50']:>9.2f} {synced['p999']:>11.2f} "
              f"{scat['p50']:>12.2f} {'bulk' if synced['p50'] < scat['p50'] else 'scatter':>8}")
    for kind in ("posted", "sync"):
        F, W = fit_F_W([r['words'] for r in rows], [r[kind]['p50'] for r in rows])
        print(f"fit bulk {kind:6s}: F = {F:.2f} us, W = {W:.1f} ns/word")
    return rows


def full_write_compare(mm, write_addr, write_bytes, reps, zdma_write):
    """Whole-buffer LO writes: devmem posted / sync, and PS DMA if given.

    Sizes: 1000 tones (16 KiB), one slot of 2048 tones (32 KiB), and the whole
    register (write_bytes, 128 KiB = 4 slots on v7.12, what loading every
    firmware-modulation slot writes). Before timing the DMA, the region is
    cleared, written once by DMA, and read back over devmem to prove it landed.
    """
    print("\n=== FULL LO BUFFER WRITE: devmem vs ZDMA (p50 / p99.9 us) ===")
    print(f"{'bytes':>7} {'devmem posted':>14} {'devmem sync':>14} {'zdma':>14} {'verified':>9}")
    rng = np.random.default_rng(0)
    rows = []
    for n in sorted({1000 * CONTROL_N_WORDS * 4, MAX_CHANS * CONTROL_N_WORDS * 4, write_bytes}):
        payload = rng.integers(0, 2**32, n // 4, dtype=np.uint32).astype('<u4').tobytes()
        a, b = write_addr, write_addr + n
        posted = pct(time_us(lambda: mm.__setitem__(slice(a, b), payload), reps))
        def sync():
            mm[a:b] = payload
            mm[b - 4:b]
        synced = pct(time_us(sync, reps))
        row = dict(bytes=n, words=n // 4, devmem_posted=posted, devmem_sync=synced)
        z = "—"
        if zdma_write is not None:
            mm[a:b] = bytes(n)
            zdma_write(payload)
            row["zdma_verified"] = verified = (mm[a:b] == payload)
            if verified:
                row["zdma"] = pct(time_us(lambda: zdma_write(payload), reps))
                z = f"{row['zdma']['p50']:.1f} / {row['zdma']['p999']:.1f}"
        print(f"{n:>7} {posted['p50']:>6.1f} / {posted['p999']:<6.1f} "
              f"{synced['p50']:>6.1f} / {synced['p999']:<6.1f} {z:>14} "
              f"{str(row.get('zdma_verified', '—')):>9}")
        rows.append(row)
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--transport", choices=TRANSPORTS, default="devmem")
    ap.add_argument("--dev", default="/dev/mem",
                    help="devmem: /dev/mem or a backing file; uio: /dev/uioN")
    ap.add_argument("--udmabuf", default="udmabuf1",
                    help="u-dma-buf for zdma and dram-* (default udmabuf1, leaving "
                         "udmabuf0 to the readout server)")
    ap.add_argument("--read-addr", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--write-addr", type=lambda s: int(s, 0), default=None)
    ap.add_argument("--max-bytes", type=lambda s: int(s, 0), default=MAX_CHANS * 8,
                    help="size of the read target region (default 16 KiB, the "
                         "accumulator; 8192 for the RX channel map)")
    ap.add_argument("--reps", type=int, default=2000,
                    help="timed calls per point (default 2000, enough for p99.9)")
    ap.add_argument("--no-write", action="store_true",
                    help="skip the write tests (they clobber the control buffer)")
    ap.add_argument("--write-bytes", type=lambda s: int(s, 0), default=None,
                    help="size of the LO control register (0x20000 on v7.12); enables "
                         "the whole-buffer write comparison")
    ap.add_argument("--zdma-write", action="store_true",
                    help="add PS-DMA writes to the whole-buffer comparison (root, "
                         "v7.12 image.ub, devmem transport)")
    ap.add_argument("--label", default="", help="free text stored in --out")
    ap.add_argument("--out", default=None, help="write all numbers to this JSON file")
    args = ap.parse_args()

    dram = args.transport.startswith("dram")
    if args.read_addr is None:
        if not dram:
            ap.error("--read-addr is required (take it from fpg_addrs.py)")
        args.read_addr = 0
    if args.write_addr is None:
        args.write_addr = 0x10000 if dram else None
    no_write = args.no_write or args.transport == "zdma" or args.write_addr is None
    max_chans = min(MAX_CHANS, args.max_bytes // 8)

    print(f"transport={args.transport} dev={args.dev} euid={os.geteuid()} "
          f"read_addr=0x{args.read_addr:08x} max_bytes={args.max_bytes} reps={args.reps}")
    mm, fd = make_mmap(args.transport, args.dev, args.udmabuf)
    read = make_reader(args.transport, mm, args.udmabuf)
    result = dict(vars(args), euid=os.geteuid(), time=time.strftime("%Y-%m-%dT%H:%M:%S"))
    try:
        rows, F, W, fb = read_scaling(read, args.read_addr, max_chans, args.reps)
        result.update(read=rows, read_F_us=F, read_W_ns=W, frombuffer_us=fb)
        result["multi_read"] = multi_read_scaling(read, args.read_addr, F, W,
                                                  max_chans, args.reps)
        if not no_write:
            result["write"] = write_scaling(mm, args.write_addr, args.reps // 4)
            if args.write_bytes:
                zw = (make_zdma_writer(args.udmabuf, AXIL_OFFSET + args.write_addr)
                      if args.zdma_write else None)
                result["full_write"] = full_write_compare(mm, args.write_addr,
                                                          args.write_bytes,
                                                          args.reps // 4, zw)
        print("\n--- crossovers ---")
        full_us = F + 4096 * W / 1000.0
        print(f"full 2048-chan read (model) : {full_us:.2f} us")
        print(f"merge runs closer than      : {F / (2 * W / 1000.0):.0f} channels")
        print(f"per-tone read beats a full-width span when M < "
              f"~{full_us / (F + 2 * W / 1000.0):.1f}")
    finally:
        if mm is not None:
            mm.close()
            os.close(fd)
    if args.out:
        with open(args.out, "w") as f:
            json.dump(result, f, indent=1)
        print(f"\nwrote {args.out}")


if __name__ == "__main__":
    sys.exit(main())
