#!/usr/bin/env python3
"""
Sample what runs on given CPUs (kernel symbols included) with perf_event_open,
for kernels without ftrace. Root. Waits for a trigger file, samples, and writes
per-100 ms counts of (cpu, thread, kernel symbol or 'user') to a JSON file.

    sudo ~/py3.12-venv/bin/python perf_cpu_sampler.py --cpus 2 3 --seconds 14
    (as casper, when ready)  touch /tmp/perf_go

Uses the software cpu-clock event (no PMU needed) at --freq Hz per CPU.
"""
import argparse, bisect, collections, ctypes, json, mmap, os, struct, sys, time

SYS_perf_event_open = 241                 # arm64
PERF_TYPE_SOFTWARE, PERF_COUNT_SW_CPU_CLOCK = 1, 0
SAMPLE_IP, SAMPLE_TID, SAMPLE_TIME, SAMPLE_CPU = 1, 2, 4, 128
PERF_EVENT_IOC_ENABLE, PERF_EVENT_IOC_DISABLE = 0x2400, 0x2401
PERF_RECORD_SAMPLE, PERF_RECORD_LOST = 9, 2
READY, GO, OUT = '/tmp/perf_ready', '/tmp/perf_go', '/tmp/perf_cpu23.json'
RAW = '/tmp/perf_cpu23_raw.json'


class Attr(ctypes.Structure):             # struct perf_event_attr, PERF_ATTR_SIZE_VER5 (112)
    _fields_ = [('type', ctypes.c_uint32), ('size', ctypes.c_uint32), ('config', ctypes.c_uint64),
                ('sample_period', ctypes.c_uint64), ('sample_type', ctypes.c_uint64),
                ('read_format', ctypes.c_uint64), ('flags', ctypes.c_uint64),
                ('wakeup_events', ctypes.c_uint32), ('bp_type', ctypes.c_uint32),
                ('config1', ctypes.c_uint64), ('config2', ctypes.c_uint64),
                ('branch_sample_type', ctypes.c_uint64), ('sample_regs_user', ctypes.c_uint64),
                ('sample_stack_user', ctypes.c_uint32), ('clockid', ctypes.c_int32),
                ('sample_regs_intr', ctypes.c_uint64), ('aux_watermark', ctypes.c_uint32),
                ('sample_max_stack', ctypes.c_uint16), ('reserved', ctypes.c_uint16)]


def open_cpu(libc, cpu, period_ns, pages):
    a = Attr()
    a.type, a.size, a.config = PERF_TYPE_SOFTWARE, ctypes.sizeof(Attr), PERF_COUNT_SW_CPU_CLOCK
    a.sample_period = period_ns
    a.sample_type = SAMPLE_IP | SAMPLE_TID | SAMPLE_TIME | SAMPLE_CPU
    a.flags = 1                           # disabled; include kernel, user and idle
    fd = libc.syscall(SYS_perf_event_open, ctypes.byref(a), -1, cpu, -1, 8)
    if fd < 0:
        raise OSError(ctypes.get_errno(), f'perf_event_open on CPU{cpu} failed')
    page = mmap.PAGESIZE
    mm = mmap.mmap(fd, (1 + pages) * page, mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE)
    return fd, mm


def drain(mm, out, lost):
    page = mmap.PAGESIZE
    head, tail = struct.unpack_from('<QQ', mm, 1024)
    size = len(mm) - page
    while tail < head:
        off = page + (tail % size)
        hdr = bytes(mm[off:off + 8]) if off + 8 <= len(mm) else bytes(mm[off:]) + bytes(mm[page:page + 8 - (len(mm) - off)])
        rtype, _misc, rsize = struct.unpack('<IHH', hdr)
        rec = bytearray()
        for k in range(rsize):                         # records may wrap the ring
            rec.append(mm[page + ((tail + k) % size)])
        if rtype == PERF_RECORD_SAMPLE:
            ip, pid, tid, t, cpu, _r = struct.unpack_from('<QIIQII', rec, 8)
            out.append((t, cpu, tid, ip))
        elif rtype == PERF_RECORD_LOST:
            lost[0] += struct.unpack_from('<QQ', rec, 8)[1]
        tail += rsize
    struct.pack_into('<Q', mm, 1032, tail)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--cpus', type=int, nargs='+', default=[2, 3])
    ap.add_argument('--seconds', type=float, default=14)
    ap.add_argument('--freq', type=int, default=5000)
    ap.add_argument('--raw', action='store_true',
                    help="also write every sample's time, CPU, thread and symbol to "
                         "/tmp/perf_cpu23_raw.json, for correlating with an event at a "
                         "known instant (the 100 ms windows are too coarse for that)")
    args = ap.parse_args()
    if os.geteuid() != 0:
        sys.exit('run as root (sudo)')
    libc = ctypes.CDLL(None, use_errno=True)
    libc.syscall.restype = ctypes.c_long
    for f in (READY, GO, OUT, RAW):
        if os.path.exists(f): os.remove(f)
    syms = []
    for line in open('/proc/kallsyms'):
        addr, kind, name = line.split()[:3]
        if kind.lower() in ('t', 'w'):
            syms.append((int(addr, 16), name))
    syms.sort()
    addrs = [s[0] for s in syms]
    evs = [open_cpu(libc, c, int(1e9 / args.freq), 256) for c in args.cpus]
    open(READY, 'w').close(); os.chmod(READY, 0o644)
    print(f'READY: {len(syms)} kernel symbols; waiting for {GO} (up to 20 min)', flush=True)
    for _ in range(12000):
        if os.path.exists(GO): break
        time.sleep(0.1)
    else:
        sys.exit('no trigger')
    samples, lost = [], [0]
    t_wall0, t_mono0 = time.time(), time.monotonic()
    for fd, _mm in evs: libc.ioctl(fd, PERF_EVENT_IOC_ENABLE, 0)
    print('SAMPLING', flush=True)
    end = time.monotonic() + args.seconds
    while time.monotonic() < end:
        for _fd, mm in evs: drain(mm, samples, lost)
        time.sleep(0.01)
    for fd, mm in evs:
        libc.ioctl(fd, PERF_EVENT_IOC_DISABLE, 0); drain(mm, samples, lost)
    comms = {}
    windows = collections.defaultdict(collections.Counter)
    t0 = min(s[0] for s in samples) if samples else 0
    for t, cpu, tid, ip in samples:
        if tid not in comms:
            try: comms[tid] = open(f'/proc/{tid}/comm').read().strip()
            except OSError: comms[tid] = 'swapper' if tid == 0 else f'tid{tid}'
        if ip >= 0xffff000000000000:
            i = bisect.bisect_right(addrs, ip) - 1
            where = syms[i][1] if i >= 0 else hex(ip)
        else:
            where = 'user'
        windows[int((t - t0) // 100_000_000)][f'CPU{cpu} {comms[tid]}:{tid} {where}'] += 1
    json.dump({'wall_at_enable': t_wall0, 'mono_at_enable': t_mono0, 'freq': args.freq, 'lost': lost[0],
               'samples': len(samples), 'windows': {str(k): dict(v) for k, v in sorted(windows.items())}},
              open(OUT, 'w'))
    os.chmod(OUT, 0o644)
    if args.raw:
        where = []
        for _t, _cpu, _tid, ip in samples:
            if ip >= 0xffff000000000000:
                i = bisect.bisect_right(addrs, ip) - 1
                where.append(syms[i][1] if i >= 0 else hex(ip))
            else:
                where.append('user')
        json.dump({'t0_mono_ns': t0, 'wall_at_enable': t_wall0, 'mono_at_enable': t_mono0,
                   't_ns': [s[0] - t0 for s in samples], 'cpu': [s[1] for s in samples],
                   'comm': [comms[s[2]] for s in samples], 'sym': where},
                  open(RAW, 'w'))
        os.chmod(RAW, 0o644)
    os.remove(READY)
    print(f'DONE: {len(samples)} samples, {lost[0]} lost, {OUT}', flush=True)


if __name__ == '__main__':
    main()
