#!/usr/bin/env python3
"""Fail if the firmware flash/RAM footprint regresses beyond a threshold.

Reconstructs the linker's flash/RAM region usage from the ELF section table
(flash = sections whose LMA lies in flash; RAM = sections whose VMA lies in the
main SRAM window) and compares against a committed baseline. --update rewrites
the baseline instead of checking.
"""

import argparse
import json
import subprocess
import sys

FLASH_BASE = 0x10000000
RAM_BASE = 0x20000000
RAM_END = 0x20080000  # scratch_x/scratch_y above this are separate regions


def measure(objdump: str, elf: str) -> dict[str, int]:
    out = subprocess.run(
        [objdump, "--section-headers", elf], capture_output=True, text=True, check=True
    ).stdout
    flash = 0
    ram = 0
    for line in out.splitlines():
        parts = line.split()
        # "Idx Name Size VMA LMA Type"; skip headers and the null section.
        if len(parts) < 5 or not parts[0].isdigit():
            continue
        try:
            size = int(parts[2], 16)
            vma = int(parts[3], 16)
            lma = int(parts[4], 16)
        except ValueError:
            continue
        if size == 0:
            continue
        section_type = parts[5] if len(parts) > 5 else ""
        # Flash counts only content sections; a NOLOAD .bss can carry a flash LMA
        # but occupies no flash.
        if FLASH_BASE <= lma < RAM_BASE and section_type in ("TEXT", "DATA"):
            flash += size
        if RAM_BASE <= vma < RAM_END:
            ram += size
    return {"flash": flash, "ram": ram}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--objdump", required=True)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--baseline", required=True)
    parser.add_argument("--max-increase-pct", type=float, default=5.0)
    parser.add_argument("--update", action="store_true")
    parser.add_argument("--stamp")
    args = parser.parse_args()

    current = measure(args.objdump, args.elf)

    if args.update:
        with open(args.baseline, "w") as f:
            json.dump(current, f, indent=2)
            f.write("\n")
        print(f"updated baseline {args.baseline}: {current}")
        return 0

    with open(args.baseline) as f:
        baseline = json.load(f)

    failed = False
    for axis in ("flash", "ram"):
        base = baseline[axis]
        cur = current[axis]
        limit = base * (1.0 + args.max_increase_pct / 100.0)
        delta_pct = (cur - base) / base * 100.0 if base else 0.0
        status = "OK"
        if cur > limit:
            status = "FAIL"
            failed = True
        print(f"{axis:5}: {cur:>7} bytes (baseline {base}, {delta_pct:+.2f}%) [{status}]")

    if failed:
        print(
            f"FAIL: footprint exceeds baseline by more than {args.max_increase_pct}%. "
            "If intended, run `uv run tools/update_binary_size_gate.py` to reset the baseline."
        )
        return 1

    if args.stamp:
        with open(args.stamp, "w") as f:
            f.write("binary-size: ok\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
