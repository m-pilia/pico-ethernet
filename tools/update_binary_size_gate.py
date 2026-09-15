#!/usr/bin/env -S uv run --script
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martino Pilia

# /// script
# requires-python = ">=3.9"
# dependencies = []
# ///

"""Rebuild the RP2350 firmware and rewrite the binary-size gate baseline.

Run after an intentional flash/RAM footprint change makes
//tools/checks:binary_size_gate fail, then commit the updated baseline.
"""

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
BASELINE = REPO_ROOT / "tools" / "checks" / "firmware_size_baseline.json"
CHECKER = REPO_ROOT / "tools" / "checks" / "binary_size_check.py"

CONFIG = "--config=rp2350"
FIRMWARE = "//src:pico_ethernet_firmware"
OBJDUMP = "@llvm_toolchain//:bin/llvm-objdump"


def bazelisk_output(*args: str) -> str:
    return subprocess.run(
        ["bazelisk", *args], cwd=REPO_ROOT, check=True, stdout=subprocess.PIPE, text=True
    ).stdout.strip()


def output_file(target: str) -> Path:
    # Resolved through cquery rather than the bazel-bin symlink, which follows
    # whichever configuration was built last.
    # `info` cannot resolve the platform label --config=rp2350 sets, and the
    # execution root does not depend on the configuration anyway.
    execution_root = Path(bazelisk_output("info", "execution_root"))
    return execution_root / bazelisk_output("cquery", CONFIG, "--output=files", target)


def main() -> int:
    subprocess.run(["bazelisk", "build", CONFIG, FIRMWARE, OBJDUMP], cwd=REPO_ROOT, check=True)
    subprocess.run(
        [
            sys.executable,
            str(CHECKER),
            "--objdump",
            str(output_file(OBJDUMP)),
            "--elf",
            str(output_file(FIRMWARE)),
            "--baseline",
            str(BASELINE),
            "--update",
        ],
        check=True,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
