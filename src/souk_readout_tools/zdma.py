"""ZDMA (PS DMA) reader for the accumulator, board-side only.

Vendored, near-verbatim, from ``software/rfsoc_scripts/zdma_read_example.py`` at
souk-firmware ``v7.12.1`` -- upstream's own hardware-validated reference. Kept
here rather than imported from the firmware checkout so a pip install of this
package is self-contained; re-sync it if upstream changes.

Why this exists. A CPU read of the accumulator goes through an uncached Device
mapping whose ceiling is a property of that mapping, not of the fabric: measured
1.64 us fixed + 22.4 ns/word, i.e. ~100 us for the full 16 KiB buffer, and C
reaches 94% of that ceiling so there is nothing left to win on the CPU side.
ZDMA lands the data in a *cached* u-dma-buf instead and inverts the cost model
entirely -- 21.94 us fixed + 1.4 ns/word, measured on this board 2026-09-14.

The two transports are therefore complementary rather than competing:

    devmem  cheap to start, expensive per word -> wins below ~335 channels
    ZDMA    expensive to start, cheap per word -> wins above, 3.6x at 2048

so the read planner picks between them by size. See
``firmware_lib.plan_accumulator_reads`` and
[[devmem-access-cost-model]] in the project notes.

Requires root (``/dev/mem``), the ``u-dma-buf`` module loaded with at least one
buffer, and a free FPD GDMA channel. Each process must claim its own
``/dev/udmabufN``: the landing buffer is overwritten by every transfer.
"""

import mmap
import os
import sys
import time

class UdmabufBuffer(object):
    """
    Landing buffer backed by the u-dma-buf kernel module
    (github.com/ikwzm/udmabuf): the kernel allocates a physically
    contiguous DMA buffer, publishes its physical address in sysfs, mmaps
    it CACHED to userspace (open without O_SYNC), and performs proper
    kernel-side cache maintenance (dma_sync_single_for_cpu) when the
    sync_for_cpu attribute is written.

    Load-time setup (once, e.g. via modprobe.d or a boot script):
        insmod u-dma-buf.ko udmabuf0=0x200000 ...
    """

    def __init__(self, name="udmabuf0"):
        sysfs = "/sys/class/u-dma-buf/%s" % name
        with open(sysfs + "/phys_addr") as fh:
            self.phys = int(fh.read(), 16)
        with open(sysfs + "/size") as fh:
            self.size = int(fh.read())
        fd = os.open("/dev/%s" % name, os.O_RDWR)   # no O_SYNC -> cached
        self._mm = mmap.mmap(fd, self.size, mmap.MAP_SHARED)
        os.close(fd)
        # keep the sysfs sync attributes open; per-read sync is then one
        # (or two, if the size changed) small pwrite syscalls
        with open(sysfs + "/sync_direction", "w") as fh:
            fh.write("2")                            # DMA_FROM_DEVICE
        with open(sysfs + "/sync_offset", "w") as fh:
            fh.write("0")
        self._size_fd = os.open(sysfs + "/sync_size", os.O_WRONLY)
        self._sync_fd = os.open(sysfs + "/sync_for_cpu", os.O_WRONLY)
        self._sync_nbytes = None
        self.invalidate(self.size)                   # purge any stale lines

    def invalidate(self, nbytes):
        """Kernel-side cache invalidate of the first nbytes."""
        if nbytes != self._sync_nbytes:
            os.pwrite(self._size_fd, str(nbytes).encode(), 0)
            self._sync_nbytes = nbytes
        os.pwrite(self._sync_fd, b"1", 0)

    def view(self, nbytes):
        """Zero-copy view of the buffer (call invalidate() first)."""
        return memoryview(self._mm)[:nbytes]


# ZDMA per-channel register offsets. [UG1087] module ZDMA;
# [embeddedsw] xzdma_hw.h XZDMA_CH_*_OFFSET.
ZDMA_CH_ISR       = 0x100  # interrupt status, write-1-to-clear
ZDMA_CH_IDS       = 0x10C  # interrupt disable
ZDMA_CH_CTRL0     = 0x110  # [7] OVR_FETCH, [6] POINT_TYPE (0=simple/in-register,
                           # 1=scatter-gather), [5:4] MODE (00=normal R/W)
ZDMA_CH_STATUS    = 0x11C  # channel state (useful when debugging errors)
ZDMA_CH_SRC_WORD0 = 0x128  # in-register descriptor: src address [31:0]
ZDMA_CH_SRC_WORD1 = 0x12C  #   src address [48:32]
ZDMA_CH_SRC_WORD2 = 0x130  #   size in bytes [29:0]
ZDMA_CH_SRC_WORD3 = 0x134  #   [0] COHRNT: route via CCI for cache coherency
ZDMA_CH_DST_WORD0 = 0x138  # same four words for the destination
ZDMA_CH_DST_WORD1 = 0x13C
ZDMA_CH_DST_WORD2 = 0x140
ZDMA_CH_DST_WORD3 = 0x144
ZDMA_CH_CTRL2     = 0x200  # [0] EN: start the programmed transfer

# ZDMA_CH_ISR bits. [UG1087] ZDMA_CH_ISR; [embeddedsw] XZDMA_IXR_*_MASK.
IXR_DMA_DONE = 0x400
IXR_ALL      = 0xFFF
IXR_ERRORS   = IXR_ALL & ~(0x400 | 0x800)  # everything but DONE and PAUSE


class ZdmaReader(object):
    """
    Drive one PS-DMA channel in simple (in-register descriptor) mode to copy
    physical memory (e.g. a PL BRAM) into a caller-supplied DRAM buffer.

    dma_base: channel register block. FPD-DMA ("GDMA") channels 0-7 live at
        0xFD500000 + N*0x10000 (full-power domain, 128-bit bus); LPD-DMA
        ("ADMA") channels at 0xFFA80000 + N*0x10000 (64-bit bus).
        [UG1085] ch. "DMA Controller"; [UG1087] ZDMA.
    dst_phys/dst_size: the reserved-DRAM landing buffer (or pass cached_buf).
    """

    def __init__(self, dst_phys=None, dst_size=0x100000, dma_base=0xFD500000,
                 cached_buf=None):
        # must happen before the first register access: a runtime-suspended
        # channel is clock-gated and register access hangs the bus
        self._ensure_channel_powered(dma_base)
        self._fd = os.open("/dev/mem", os.O_RDWR | os.O_SYNC)
        self._regs = self._map(dma_base, 0x1000)
        self._cached = cached_buf
        if cached_buf is not None:
            self._dst_mm = cached_buf._mm
            self.dst_phys = cached_buf.phys
            self.dst_size = cached_buf.size
        else:
            if dst_phys is None:
                raise ValueError("need dst_phys (reserved DRAM) or cached_buf")
            self._dst_mm = mmap.mmap(self._fd, dst_size, mmap.MAP_SHARED,
                                     offset=dst_phys)
            self.dst_phys = dst_phys
            self.dst_size = dst_size
        self._programmed = None  # (src, nbytes) currently in the descriptor regs

        # one-time channel setup
        self._wr(ZDMA_CH_IDS, IXR_ALL)   # no interrupts; we poll ISR (raw status)
        self._wr(ZDMA_CH_ISR, IXR_ALL)   # clear stale status
        # simple mode, normal R/W, over-fetch OFF so the DMA never reads past
        # the end of the BRAM's assigned address segment ([UG1085] OVR_FETCH)
        self._wr(ZDMA_CH_CTRL0, 0x0)

    @staticmethod
    def _ensure_channel_powered(dma_base):
        """
        The kernel's zynqmp_dma driver runtime-suspends idle channels,
        gating their clocks -- register access then hangs the bus. If the
        channel is suspended, log it and pin it on (the sysfs equivalent of
        `echo on > .../power/control`, which resumes the device and
        prevents future autosuspend). This resets on reboot, hence checking
        on every startup.
        """
        dev = "/sys/bus/platform/devices/%x.dma" % dma_base
        try:
            with open(dev + "/power/runtime_status") as fh:
                status = fh.read().strip()
        except FileNotFoundError:
            # no driver bound to this channel: nothing is managing (or
            # gating) its clocks, so there is nothing to wake
            return
        if status == "active":
            return
        print("zdma: %s is runtime-%s; switching power/control to 'on'"
              % (dev, status), file=sys.stderr)
        try:
            with open(dev + "/power/control", "w") as fh:
                fh.write("on")
        except PermissionError:
            raise RuntimeError(
                "%s is runtime-suspended (clock-gated) and this process "
                "lacks permission to wake it; run as root or "
                "`echo on | sudo tee %s/power/control` first" % (dev, dev))
        with open(dev + "/power/runtime_status") as fh:
            status = fh.read().strip()
        if status != "active":
            raise RuntimeError("failed to wake %s (runtime_status=%r)"
                               % (dev, status))

    def _map(self, phys, length):
        """
        mmap physical address range phys..phys+length as a 32-bit-word view.

        The .cast('I') item access gives the aligned 32-bit reads/writes
        that device registers require (plain mmap slicing does byte
        accesses, which are not safe on register space).
        """
        m = mmap.mmap(self._fd, length, mmap.MAP_SHARED, offset=phys)
        return memoryview(m).cast("I")

    def _wr(self, off, val):
        """32-bit write to the channel register at byte offset off."""
        self._regs[off >> 2] = val

    def _rd(self, off):
        """32-bit read of the channel register at byte offset off."""
        return self._regs[off >> 2]

    def _program(self, src, nbytes):
        """Load the in-register SRC/DST descriptors for src -> dst_phys."""
        self._wr(ZDMA_CH_SRC_WORD0, src & 0xFFFFFFFF)
        self._wr(ZDMA_CH_SRC_WORD1, (src >> 32) & 0x1FFFF)
        self._wr(ZDMA_CH_SRC_WORD2, nbytes)
        self._wr(ZDMA_CH_SRC_WORD3, 0)   # non-coherent (the CCI does not snoop
                                         # for this master under stock boot fw;
                                         # cached mode invalidates instead)
        self._wr(ZDMA_CH_DST_WORD0, self.dst_phys & 0xFFFFFFFF)
        self._wr(ZDMA_CH_DST_WORD1, (self.dst_phys >> 32) & 0x1FFFF)
        self._wr(ZDMA_CH_DST_WORD2, nbytes)
        self._wr(ZDMA_CH_DST_WORD3, 0)
        self._programmed = (src, nbytes)

    def _xfer(self, src, nbytes, timeout_s):
        """
        Program (only if src/nbytes changed), start, and wait for one
        transfer, then invalidate the landing-buffer cache lines in cached
        mode. On return the buffer holds fresh, CPU-visible data.
        """
        if nbytes > self.dst_size:
            raise ValueError("nbytes %d exceeds landing buffer %d"
                             % (nbytes, self.dst_size))
        if self._programmed != (src, nbytes):
            self._program(src, nbytes)
        self._wr(ZDMA_CH_CTRL2, 0x1)     # go
        t0 = time.perf_counter()
        while True:
            isr = self._rd(ZDMA_CH_ISR)
            if isr & IXR_DMA_DONE:
                break
            if isr & IXR_ERRORS:
                self._wr(ZDMA_CH_ISR, IXR_ALL)
                self._programmed = None
                raise RuntimeError("DMA error: ISR=0x%03x STATUS=0x%x"
                                   % (isr, self._rd(ZDMA_CH_STATUS)))
            if time.perf_counter() - t0 > timeout_s:
                self._programmed = None
                raise RuntimeError(
                    "DMA timeout: ISR=0x%03x STATUS=0x%x (wrong channel? "
                    "clock gated? bad address?)"
                    % (isr, self._rd(ZDMA_CH_STATUS)))
        self._wr(ZDMA_CH_ISR, IXR_ALL)   # clear DONE for the next transfer
        if self._cached is not None:
            # drop stale/speculatively-fetched (clean) lines so CPU reads
            # see the DMA'd data
            self._cached.invalidate(nbytes)

    def read(self, src, nbytes, timeout_s=1.0):
        """
        DMA nbytes from physical address src into the landing buffer and
        return them as bytes. Safe to call repeatedly; the descriptor
        registers are only rewritten when src/nbytes change.
        """
        self._xfer(src, nbytes, timeout_s)
        # copy out of the landing buffer into ordinary memory
        # (np.frombuffer(data, '<i4') etc. from here)
        return self._dst_mm[:nbytes]

    def read_view(self, src, nbytes, timeout_s=1.0):
        """
        Like read(), but returns a zero-copy memoryview of the landing
        buffer (cached mode only) -- wrap it with numpy and process in
        place. The view's contents are only valid until the next read.
        """
        if self._cached is None:
            raise RuntimeError("read_view() requires cached_buf mode")
        self._xfer(src, nbytes, timeout_s)
        return self._cached.view(nbytes)

