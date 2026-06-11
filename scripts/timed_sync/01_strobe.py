#!/usr/bin/env python3
"""Enable the TSU strobe: emit a 1-PPS into the firmware off the PTP-disciplined PHC.

(TSU = the GEM Timestamp Unit -- the Zynq Ethernet MAC's timestamp counter, whose
compare output produces the strobe.)

Each time the GEM TSU seconds counter ticks, arm the comparison register so a
strobe fires on the next integer-second boundary (ns==0). Must keep running --
the GEM compare is one-shot, so it is re-armed every second. Run as root.
"""
import fcntl, os, struct, sys, time
from mmap import mmap, PROT_READ, PROT_WRITE, MAP_SHARED

# Zynq UltraScale+ GEM2 TSU registers (AMD UG1087, GEM module).
GEM2 = 0xFF0D0000
TSU_TIMER_MSB_SEC, TSU_TIMER_SEC = 0x1C0, 0x1D0
TSU_CMP_MSB_SEC, TSU_CMP_SEC, TSU_CMP_NSEC = 0x0E4, 0x0E0, 0x0DC

# Single-instance lock: the GEM TSU has ONE compare register, so two strobes
# would race it. Hold an exclusive lock (released automatically on exit).
_lock = open("/run/tsu-strobe.lock", "r+" if os.path.exists("/run/tsu-strobe.lock") else "w+")
try:
    fcntl.flock(_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
except OSError:
    held_pid = _lock.read().strip() or "?"
    sys.exit(f"another strobe instance is already running (pid {held_pid}); "
             f"kill it with: sudo kill {held_pid}")
_lock.seek(0); _lock.truncate(); _lock.write(f"{os.getpid()}\n"); _lock.flush()

fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
mm = mmap(fd, 0x200, offset=GEM2, flags=MAP_SHARED, prot=PROT_READ | PROT_WRITE)
rd = lambda a: struct.unpack("<I", mm[a:a+4])[0]
wr = lambda a, v: mm.__setitem__(slice(a, a+4), struct.pack("<I", v & 0xFFFFFFFF))

wr(TSU_CMP_NSEC, 0)                       # fire at ns==0 (second boundary)
sec_msb = rd(TSU_TIMER_MSB_SEC)
wr(TSU_CMP_MSB_SEC, sec_msb)
print(f"strobe running; TSU sec = {rd(TSU_TIMER_SEC)} (Ctrl-C to stop)")
print("first 3 re-arms (should be ~1/s, TSU sec +1 each):")

t_old = rd(TSU_TIMER_SEC)
shown = 0
while True:
    t = rd(TSU_TIMER_SEC)
    if t != t_old:
        if t == 0xFFFFFFFF:              # 32-bit seconds rollover
            sec_msb += 1; wr(TSU_CMP_MSB_SEC, sec_msb)
        wr(TSU_CMP_SEC, t + 1)           # arm next-second strobe
        if shown < 3:                    # show only the first 3 seconds as proof of life
            sys_off_ms = (time.time() % 1.0) * 1e3   # this tick vs system-clock second
            print(f"  armed strobe for TSU sec {t + 1}  (sys-clock {sys_off_ms:6.1f} ms into its second)")
            shown += 1
            if shown == 3:
                print("  ...running (re-arming silently every second; Ctrl-C to stop)")
        t_old = t
    time.sleep(0.001)
