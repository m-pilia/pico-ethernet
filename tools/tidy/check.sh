#!/usr/bin/env bash
# Runs the clang-tidy gate over every first-party non-test translation unit.
# Two passes are needed because the aspect sees one toolchain per invocation:
# the host (clang) pass covers the host-compilable sources, and the rp2350
# (arm-none-eabi gcc) pass covers the firmware and its hardware-glue sources.
set -euo pipefail

cd "$(dirname "$0")/../.."

bazelisk build //src/... --config=tidy "$@"
bazelisk build //src:pico_ethernet_firmware --config=tidy --config=rp2350 "$@"
