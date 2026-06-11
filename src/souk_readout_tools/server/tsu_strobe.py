#!/usr/bin/env python3
"""TSU strobe daemon -- emit a 1-PPS into the RFSoC firmware off the PHC.

Run by tsu-strobe.service (ExecStart=souk-tsu-strobe). The standalone test
equivalent is scripts/timed_sync/01_strobe.py.

(TSU = the GEM Timestamp Unit, the Zynq Ethernet MAC timestamp counter whose
compare output produces the strobe.)

Each time the GEM TSU seconds counter ticks, arm the comparison register so a
strobe fires on the next integer-second boundary (ns==0). The GEM compare is
one-shot, so it must be re-armed every second -- this loop must keep running.
Single-instance flock; needs root for /dev/mem.

Publishes status (for the timing monitor / readout-tools) as JSON to
STATUS_PATH every second: running, pid, rearms, tsu_sec, updated_unix_s.
"""
import fcntl
import json
import logging
import os
import struct
import sys
import time
from mmap import mmap, PROT_READ, PROT_WRITE, MAP_SHARED

HEARTBEAT_S = 60.0                       # how often to log a "still running" line
STATUS_PATH = "/run/tsu-strobe.status"   # JSON status, refreshed each second
LOCK_PATH = "/run/tsu-strobe.lock"

# Zynq UltraScale+ GEM2 TSU registers (AMD UG1087, GEM module).
GEM2 = 0xFF0D0000
TSU_TIMER_MSB_SEC, TSU_TIMER_SEC = 0x1C0, 0x1D0
TSU_CMP_MSB_SEC, TSU_CMP_SEC, TSU_CMP_NSEC = 0x0E4, 0x0E0, 0x0DC

log = logging.getLogger("tsu-strobe")


def _write_status(path, **fields):
    # Atomic-ish write so a reader never sees a half-written file.
    fields["updated_unix_s"] = time.time()
    tmp = path + ".tmp"
    try:
        with open(tmp, "w") as f:
            json.dump(fields, f)
        os.replace(tmp, path)
    except OSError as exc:
        log.warning("could not write status file %s: %s", path, exc)


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s",
                        stream=sys.stderr)

    # Single-instance lock (the GEM has ONE compare register; two strobes race).
    lock = open(LOCK_PATH, "r+" if os.path.exists(LOCK_PATH) else "w+")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except OSError:
        held = lock.read().strip() or "?"
        log.error("another strobe instance is already running (pid %s)", held)
        sys.exit(1)
    lock.seek(0); lock.truncate(); lock.write(f"{os.getpid()}\n"); lock.flush()

    try:
        fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
    except PermissionError:
        log.error("cannot open /dev/mem -- the strobe must run as root")
        sys.exit(1)
    mm = mmap(fd, 0x200, offset=GEM2, flags=MAP_SHARED, prot=PROT_READ | PROT_WRITE)
    rd = lambda a: struct.unpack("<I", mm[a:a + 4])[0]
    wr = lambda a, v: mm.__setitem__(slice(a, a + 4), struct.pack("<I", v & 0xFFFFFFFF))

    wr(TSU_CMP_NSEC, 0)                      # fire at ns==0 (second boundary)
    sec_msb = rd(TSU_TIMER_MSB_SEC)
    wr(TSU_CMP_MSB_SEC, sec_msb)
    t_start = rd(TSU_TIMER_SEC)
    log.info("strobe started; TSU sec = %d, MSB = %d (pid %d)", t_start, sec_msb, os.getpid())
    if t_start == 0:
        log.warning("TSU seconds = 0: PHC not disciplined yet -- re-arming anyway, "
                    "but the PPS will not be time-aligned until ptp4l/chrony lock.")

    t_old = t_start
    rearms = 0
    last_beat = time.monotonic()
    _write_status(STATUS_PATH, running=True, pid=os.getpid(), rearms=0, tsu_sec=t_start)
    try:
        while True:
            t = rd(TSU_TIMER_SEC)
            if t != t_old:
                if t == 0xFFFFFFFF:          # 32-bit seconds rollover
                    sec_msb += 1; wr(TSU_CMP_MSB_SEC, sec_msb)
                    log.info("TSU seconds MSB rolled over to %d", sec_msb)
                wr(TSU_CMP_SEC, t + 1)       # arm next-second strobe
                t_old = t
                rearms += 1
                _write_status(STATUS_PATH, running=True, pid=os.getpid(),
                              rearms=rearms, tsu_sec=t)
                now = time.monotonic()
                if now - last_beat >= HEARTBEAT_S:
                    last_beat = now
                    log.info("alive: TSU sec %d, %d re-arms so far", t, rearms)
            time.sleep(0.001)
    finally:
        # Mark not-running on exit so readers don't see a stale "running" forever.
        _write_status(STATUS_PATH, running=False, pid=os.getpid(),
                      rearms=rearms, tsu_sec=t_old)


if __name__ == "__main__":
    main()
