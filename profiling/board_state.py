#!/usr/bin/env python3
"""
Record what the board is actually running, before any measurement (test B0).

Every number in the staged comparison (doc/v712-notes.md, profiling/README.md)
must be traceable to a configuration read back from the board, not assumed:
twice now a label has been wrong because a symlink or a version string was
trusted instead. This prints and saves:

  - boot files: md5 of BOOT.BIN / image.ub / boot.scr / top.bit, named if known
  - PS registers (root): PL0_REF_CTRL (the AXI clock divisor, i.e. 100 vs 200 MHz)
    and FPD_SLCR.AFI_FS (HPM0/HPM1 data width) -- the direct test of BOOT.BIN
  - kernel: cmdline, uio/udmabuf nodes, cpufreq, net send-buffer limits
  - software: control_sw fw version, casperfpga MAP_SIZE, souk_readout_tools
    version plus md5 of the installed server/firmware_lib (the dist-info
    version string can lie), souk-firmware checkout HEAD
  - the .fpg the server will load, resolved from the control_sw yaml
  - services: readout servers, timing-monitor, journald storage
  - tcpborphserver's /dev/mem mapping (root): does low32(addr) + size wrap
    past 4 GiB? If so its katcp write bounds check (raw.c:944/1203) rejects
    writes to low-offset registers.

Usage (on the board; root gives the register and mapping checks):
    sudo ~/py3.12-venv/bin/python board_state.py --out state.json
"""

import argparse
import glob
import hashlib
import json
import mmap
import os
import re
import socket
import struct
import subprocess
import time

FW_REPO = os.path.expanduser("~casper/souk-firmware")
FW_CONFIG = FW_REPO + "/software/control_sw/config/souk-dual-pipeline-krm.yaml"

# md5 prefixes of the files seen on this board / in souk-firmware
KNOWN = {
    "566b1b826615": "pre-v7.12 BOOT.BIN (original SD image)",
    "0674ecb9cbb4": "v7.12.1 BOOT.BIN",
    "c336e0a98bae": "pre-v7.12 image.ub (original SD image)",
    "ee1ef4768a99": "v7.12.1 image.ub",
    "6139b67a2116": "boot.scr (identical v7.11.2 / v7.12.1)",
    "0d596da763f4": "v7.11.2 top.bit",
    "c56e6b0d17ca": "v7.12.1 top.bit",
}

PL0_REF_CTRL = 0xFF5E00C0    # CRL_APB: [24] CLKACT, [21:16] DIV1, [13:8] DIV0, [2:0] SRCSEL
AFI_FS = 0xFD615000          # FPD_SLCR: [9:8] HPM0 width, [11:10] HPM1 width
WIDTH = {0: "32-bit", 1: "64-bit", 2: "128-bit", 3: "reserved"}


def sh(cmd):
    try:
        return subprocess.run(cmd, shell=True, capture_output=True, text=True,
                              timeout=10).stdout.strip()
    except Exception as exc:
        return f"error: {exc}"


def md5(path):
    try:
        with open(path, "rb") as f:
            d = hashlib.md5(f.read()).hexdigest()[:12]
        return f"{d} {KNOWN.get(d, '(unknown)')}"
    except OSError as exc:
        return f"missing ({exc.strerror})"


def read_reg(phys):
    page = phys & ~0xFFF
    fd = os.open("/dev/mem", os.O_RDONLY | os.O_SYNC)
    try:
        mm = mmap.mmap(fd, 0x1000, mmap.MAP_SHARED, mmap.PROT_READ, offset=page)
        val = struct.unpack("<I", mm[phys - page:phys - page + 4])[0]
        mm.close()
        return val
    finally:
        os.close(fd)


def ps_registers():
    if os.geteuid() != 0:
        return "needs root"
    pl0, afi = read_reg(PL0_REF_CTRL), read_reg(AFI_FS)
    div0, div1 = (pl0 >> 8) & 0x3F, (pl0 >> 16) & 0x3F
    out = {
        "PL0_REF_CTRL": f"0x{pl0:08x}", "pl0_div0": div0, "pl0_div1": div1,
        "pl0_srcsel": pl0 & 0x7, "pl0_clkact": (pl0 >> 24) & 1,
        "pl0_expected_MHz": {5: 200, 10: 100}.get(div0 * div1, "?"),
        "AFI_FS": f"0x{afi:08x}",
        "hpm0_width": WIDTH[(afi >> 8) & 3], "hpm1_width": WIDTH[(afi >> 10) & 3],
    }
    rate = sh("cat /sys/kernel/debug/clk/pl0_ref/clk_rate 2>/dev/null")
    if rate:
        out["pl0_ref_clk_rate_Hz"] = int(rate)
    return out


def software():
    py = os.path.expanduser("~casper/py3.12-venv/bin/python")
    probe = (
        "import json, hashlib, os\n"
        "out = {}\n"
        "try:\n"
        "    import souk_mkid_readout as m; out['control_sw_fwversion'] = m.__fwversion__\n"
        "except Exception as e: out['control_sw_fwversion'] = repr(e)\n"
        "try:\n"
        "    from casperfpga import transport_localmem as t; out['casperfpga_MAP_SIZE'] = hex(t.MAP_SIZE)\n"
        "except Exception as e: out['casperfpga_MAP_SIZE'] = repr(e)\n"
        "try:\n"
        "    import souk_readout_tools as s\n"
        "    d = os.path.dirname(s.__file__); out['srt_version_string'] = s.__version__\n"
        "    for f in ('server/readout_server.py', 'firmware_lib.py'):\n"
        "        out['md5 ' + f] = hashlib.md5(open(os.path.join(d, f), 'rb').read()).hexdigest()[:12]\n"
        "    out['srt_has_zdma'] = os.path.exists(os.path.join(d, 'zdma.py'))\n"
        "    out['srt_path'] = d\n"
        "except Exception as e: out['souk_readout_tools'] = repr(e)\n"
        "print(json.dumps(out))\n")
    try:
        run = subprocess.run([py, "-c", probe], capture_output=True, text=True, timeout=60)
        return json.loads(run.stdout)
    except (ValueError, OSError, subprocess.TimeoutExpired) as exc:
        return f"probe failed: {exc!r}"


def configured_fpg():
    try:
        with open(FW_CONFIG) as f:
            fpg = re.search(r"^fpgfile:\s*(\S+)", f.read(), re.M).group(1).strip("'\"")
        return os.path.realpath(os.path.join(os.path.dirname(FW_CONFIG), fpg))
    except Exception as exc:
        return f"error: {exc}"


def tcpborph_mapping():
    """Find tcpborphserver's /dev/mem mapping and test the uint32 wrap."""
    pid = sh("pgrep -f tcpborphserver | head -1")
    if not pid:
        return "not running"
    if os.geteuid() != 0:
        return f"pid {pid}; needs root to read maps"
    maps = []
    for line in open(f"/proc/{pid}/maps"):
        if "/dev/mem" in line:
            lo, hi = (int(x, 16) for x in line.split()[0].split("-"))
            size = hi - lo
            maps.append(dict(start=f"0x{lo:x}", size=f"0x{size:x}",
                             low32_plus_size_wraps=(lo & 0xFFFFFFFF) + size > 0xFFFFFFFF))
    return maps or "no /dev/mem mapping"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--label", default="", help="stage label, e.g. S0")
    ap.add_argument("--out", default=None)
    args = ap.parse_args()

    state = {
        "label": args.label, "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "host": socket.gethostname(), "euid": os.geteuid(),
        "uptime": sh("uptime -p"), "kernel": sh("uname -rv"),
        "cmdline": open("/proc/cmdline").read().strip(),
        "boot_files": {f: md5(f) for f in ("/boot/BOOT.BIN", "/boot/image.ub",
                                           "/boot/boot.scr", "/lib/firmware/top.bit")},
        "ps_registers": ps_registers(),
        "uio": {os.path.basename(d): open(d + "/name").read().strip()
                for d in sorted(glob.glob("/sys/class/uio/uio*"))},
        "udmabuf": sorted(os.path.basename(p) for p in glob.glob("/dev/udmabuf[0-9]*")),
        "zdma_power_control": sh("cat /sys/bus/platform/devices/fd500000.dma/power/control"),
        "software": software(),
        "fw_repo_head": sh(f"git -C {FW_REPO} describe --tags --always 2>/dev/null"),
        "configured_fpg": configured_fpg(),
        "services": {s: sh(f"systemctl is-active {s}") for s in
                     ("readout_server_0", "readout_server_1", "timing-monitor",
                      "tsu-strobe", "ptp4l", "lna-service")},
        "journald_storage": sh("grep -hs '^Storage' /etc/systemd/journald.conf "
                               "/etc/systemd/journald.conf.d/*.conf") or "default (auto)",
        "var_log_journal_exists": os.path.isdir("/var/log/journal"),
        "cpufreq": sh("cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor "
                      "/sys/devices/system/cpu/cpu0/cpufreq/scaling_cur_freq | tr '\\n' ' '"),
        "net_core_wmem_max": sh("cat /proc/sys/net/core/wmem_max"),
        "tcp_wmem": sh("cat /proc/sys/net/ipv4/tcp_wmem"),
        "tcpborphserver_devmem_map": tcpborph_mapping(),
    }
    print(json.dumps(state, indent=1))
    if args.out:
        with open(args.out, "w") as f:
            json.dump(state, f, indent=1)


if __name__ == "__main__":
    main()
