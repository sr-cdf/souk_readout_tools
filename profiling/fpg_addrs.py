#!/usr/bin/env python3
"""
Extract the real AXI-lite byte offsets for the control buffer (write target)
and accumulator dout (read target) straight from the .fpg header — no live
readout object, no hardware needed.

The .fpg header contains lines of the form:
    ?register <name> <abs_addr_hex> <size_hex>
where abs_addr is anchored at AXIL_OFFSET (0xa0000000). The byte offset into
the mmap window used by the profilers is simply abs_addr - AXIL_OFFSET
(matching transport_localmem._get_device_address, which subtracts AXIL_OFFSET).

Usage:
    python3 fpg_addrs.py path/to/design.fpg [--pipeline 0] [--lo tx] [--acc 0]

It prints ready-to-paste --write-addr / --read-addr for the profilers. The
control buffer is the per-step write target of a real sweep; the accumulator
dout0 is the per-step read target.
"""

import argparse
import re
import sys

AXIL_OFFSET = 0xA0000000

# ?register <name> <addr> <size>, addr/size may be 0x-prefixed hex
_LINE = re.compile(rb"^\?register\s+(\S+)\s+(0x[0-9a-fA-F]+|\d+)\s+(0x[0-9a-fA-F]+|\d+)")


def parse_registers(fpg_path):
    """Return {name: (abs_addr, size)} parsed from the fpg text header."""
    regs = {}
    with open(fpg_path, "rb") as f:
        for raw in f:
            if not raw.startswith(b"?"):
                # header ends before the binary blob; once we hit a non-? line
                # at the start that isn't a known directive we can stop.
                if raw.startswith(b"?quit") or raw.startswith(b"?uploadbin"):
                    continue
                # keep scanning until clearly into binary (lots of files have a
                # ?quit marker; bail on first non-? after we've seen registers)
                if regs and not raw.startswith(b"?"):
                    break
                continue
            m = _LINE.match(raw)
            if m:
                name = m.group(1).decode()
                addr = int(m.group(2), 0)
                size = int(m.group(3), 0)
                regs[name] = (addr, size)
    return regs


def find_one(regs, candidates, what):
    for name in candidates:
        if name in regs:
            return name, regs[name]
    print(f"ERROR: could not find {what}. Tried: {candidates}", file=sys.stderr)
    print("Available matching registers:", file=sys.stderr)
    for n in sorted(regs):
        if any(tok in n for tok in ("control", "dout0")):
            a, s = regs[n]
            print(f"  {n}  0x{a:08x}  size=0x{s:x}", file=sys.stderr)
    sys.exit(1)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("fpg", help="path to .fpg file")
    p.add_argument("--pipeline", type=int, default=0, help="pipeline id (default 0)")
    p.add_argument("--lo", choices=["tx", "rx"], default="tx",
                   help="control buffer LO to use as write target (default tx)")
    p.add_argument("--acc", type=int, default=0, help="accumulator index (default 0)")
    args = p.parse_args()

    regs = parse_registers(args.fpg)
    if not regs:
        print("ERROR: no ?register lines parsed — is this an fpg file?", file=sys.stderr)
        sys.exit(1)

    pp = f"p{args.pipeline}"
    write_name, (write_abs, write_size) = find_one(
        regs,
        [f"{pp}_mix_{args.lo}_lo0_control"],
        f"{args.lo} control buffer for pipeline {args.pipeline}")
    read_name, (read_abs, read_size) = find_one(
        regs,
        [f"{pp}_acc{args.acc}_dout0"],
        f"accumulator {args.acc} dout0 for pipeline {args.pipeline}")

    write_off = write_abs - AXIL_OFFSET
    read_off = read_abs - AXIL_OFFSET

    print(f"\n=== fpg: {args.fpg} ===")
    print(f"registers parsed : {len(regs)}")
    print(f"write target     : {write_name}")
    print(f"  abs=0x{write_abs:08x}  offset=0x{write_off:06x}  size=0x{write_size:x}")
    print(f"read target      : {read_name}")
    print(f"  abs=0x{read_abs:08x}  offset=0x{read_off:06x}  size=0x{read_size:x}")
    print("\n--- ready-to-paste profiler args ---")
    print(f"--dev /dev/mem --write-addr 0x{write_off:06x} --read-addr 0x{read_off:06x}\n")


if __name__ == "__main__":
    main()
