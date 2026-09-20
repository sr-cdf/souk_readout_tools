#!/usr/bin/env python3
"""
Extract the real byte offsets of the profiling targets straight from the .fpg
header — no live readout object, no hardware needed.

The .fpg header contains lines of the form:
    ?register <name> <abs_addr_hex> <size_hex>
The byte offset into the mmap window used by the profilers is
abs_addr - AXIL_OFFSET (matching transport_localmem._get_device_address).

Pass the **control_sw config yaml the server uses** rather than an .fpg path.
The yaml's `fpgfile` is what actually gets loaded; guessing the .fpg by hand
is how a single-pipeline header once got used against a dual-pipeline board
(the v7.11 "accumulator" measurement that really read p1_mix_rx_lo0_control0).

Usage:
    python3 fpg_addrs.py ~/souk-firmware/software/control_sw/config/souk-dual-pipeline-krm.yaml
    python3 fpg_addrs.py path/to/design.fpg [--pipeline 0] [--lo tx] [--acc 0]

Targets printed:
    write  p{n}_mix_{lo}_lo0_control0  LO control buffer (per-step sweep write)
    read   p{n}_acc{a}_dout0           accumulator (per-step sweep read)
    ref    p{n}_chan_select_map0_map1  RX channel map: AXI-lite in EVERY
                                       firmware generation, so it isolates the
                                       AXI clock from the AXI4-BRAM change.
                                       Read-only -- writing it breaks the RX map.
"""

import argparse
import os
import re
import sys

AXIL_OFFSET = 0xA0000000

# ?register <name> <addr> <size>, addr/size may be 0x-prefixed hex
_LINE = re.compile(rb"^\?register\s+(\S+)\s+(0x[0-9a-fA-F]+|\d+)\s+(0x[0-9a-fA-F]+|\d+)")


def resolve_fpg(path):
    """Return the real .fpg path, following a control_sw config yaml if given."""
    if path.endswith((".yaml", ".yml")):
        with open(path) as f:
            m = re.search(r"^fpgfile:\s*(\S+)", f.read(), re.M)
        if not m:
            sys.exit(f"ERROR: no fpgfile in {path}")
        fpg = m.group(1).strip("'\"")
        if not os.path.isabs(fpg):
            # control_sw resolves it relative to the config file's directory
            fpg = os.path.join(os.path.dirname(os.path.abspath(path)), fpg)
        path = fpg
    return os.path.realpath(path)


def parse_registers(fpg_path):
    """Return {name: (abs_addr, size)} parsed from the fpg text header."""
    regs = {}
    with open(fpg_path, "rb") as f:
        for raw in f:
            if raw.startswith(b"?quit"):
                break
            m = _LINE.match(raw)
            if m:
                regs[m.group(1).decode()] = (int(m.group(2), 0), int(m.group(3), 0))
    return regs


def find_one(regs, candidates, what):
    for name in candidates:
        if name in regs:
            return name, regs[name]
    print(f"ERROR: could not find {what}. Tried: {candidates}", file=sys.stderr)
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("fpg", help="control_sw config yaml (preferred) or .fpg path")
    p.add_argument("--pipeline", type=int, default=0, help="pipeline id (default 0)")
    p.add_argument("--lo", choices=["tx", "rx"], default="tx",
                   help="control buffer LO to use as write target (default tx)")
    p.add_argument("--acc", type=int, default=0, help="accumulator index (default 0)")
    p.add_argument("--shell", action="store_true",
                   help="print only VAR=value lines, for `eval` in run_stage.sh")
    args = p.parse_args()

    fpg = resolve_fpg(args.fpg)
    regs = parse_registers(fpg)
    if not regs:
        sys.exit("ERROR: no ?register lines parsed — is this an fpg file?")

    pp = f"p{args.pipeline}"
    targets = {
        "WRITE": find_one(regs, [f"{pp}_mix_{args.lo}_lo0_control0",  # v7.11+
                                 f"{pp}_mix_{args.lo}_lo0_control"],  # pre-v7.11
                          "LO control buffer"),
        "READ": find_one(regs, [f"{pp}_acc{args.acc}_dout0"], "accumulator dout0"),
        "REF": find_one(regs, [f"{pp}_chan_select_map0_map1"], "RX channel map"),
    }

    if args.shell:
        print(f"FPG={fpg}")
        for key, (name, (addr, size)) in targets.items():
            print(f"{key}_NAME={name} {key}_ADDR=0x{addr - AXIL_OFFSET:08x} "
                  f"{key}_ABS=0x{addr:08x} {key}_SIZE={size}")
        return

    print(f"\n=== fpg: {fpg} ===")
    print(f"registers parsed : {len(regs)}")
    for key, (name, (addr, size)) in targets.items():
        print(f"{key.lower():5s} target : {name}")
        print(f"  abs=0x{addr:08x}  offset=0x{addr - AXIL_OFFSET:08x}  size=0x{size:x}")
    w, r = targets["WRITE"][1][0], targets["READ"][1][0]
    print("\n--- ready-to-paste profiler args ---")
    print(f"--dev /dev/mem --write-addr 0x{w - AXIL_OFFSET:08x} "
          f"--read-addr 0x{r - AXIL_OFFSET:08x}\n")


if __name__ == "__main__":
    main()
