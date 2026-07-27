#!/usr/bin/env python3
"""Fail if any first-party object references a dynamic-memory allocator.

The coding guidelines forbid dynamic allocation. This turns that review rule into
a hard gate scoped to our own archives (the SDK and newlib legitimately allocate
-- e.g. the alarm pool -- so a whole-image check is not meaningful). It runs `nm`
over each first-party archive and fails if any allocator symbol (malloc family,
sbrk, or C++ operator new/delete) is defined in or referenced from our code.
"""

import argparse
import subprocess
import sys

# Mangled C++ operator new/delete (Itanium ABI) plus the sized/aligned variants,
# alongside the C allocator families. Matched against exact symbol names.
FORBIDDEN = {
    "malloc",
    "calloc",
    "realloc",
    "free",
    "reallocarray",
    "_malloc_r",
    "_calloc_r",
    "_realloc_r",
    "_free_r",
    "sbrk",
    "_sbrk",
    "_sbrk_r",
    "brk",
    "_Znwm",  # operator new(size_t)
    "_Znam",  # operator new[](size_t)
    "_ZdlPv",  # operator delete(void*)
    "_ZdaPv",  # operator delete[](void*)
    "_ZdlPvm",  # operator delete(void*, size_t)
    "_ZdaPvm",  # operator delete[](void*, size_t)
    "_ZnwmSt11align_val_t",
    "_ZnamSt11align_val_t",
    "_ZdlPvSt11align_val_t",
    "_ZdaPvSt11align_val_t",
}


def check_archive(nm: str, archive: str) -> dict[str, str]:
    result = subprocess.run([nm, archive], capture_output=True, text=True, check=True)
    found = {}
    for line in result.stdout.splitlines():
        parts = line.split()
        if not parts:
            continue
        # `nm` lines are "<addr> <type> <name>" or "<type> <name>" (undefined).
        name = parts[-1]
        sym_type = parts[-2] if len(parts) >= 2 else "?"
        if name in FORBIDDEN:
            found.setdefault(name, sym_type)
    return found


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--nm", required=True, help="path to the nm binary")
    parser.add_argument("--archives", nargs="+", required=True)
    parser.add_argument("--stamp", help="file to touch on success (Bazel output)")
    args = parser.parse_args()

    failed = False
    for archive in args.archives:
        if not archive.endswith(".a"):
            continue
        found = check_archive(args.nm, archive)
        if found:
            failed = True
            print(f"FAIL: {archive} references forbidden allocator symbols:")
            for name, sym_type in sorted(found.items()):
                kind = "undefined ref" if sym_type.upper() == "U" else f"defined ({sym_type})"
                print(f"  {name}: {kind}")

    if failed:
        return 1

    if args.stamp:
        with open(args.stamp, "w") as f:
            f.write("no-alloc: ok\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())
