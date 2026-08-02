#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR/.."

bazelisk test //...
bazelisk test --config=asan //...
bazelisk test --config=ubsan //...
bazelisk test --config=msan //...
bazelisk build --config=rp2350 //src:pico_ethernet_firmware
bazelisk build --config=rp2350 //tools/checks:binary_size_gate
bazelisk build --config=rp2350 //tools/checks:no_alloc_gate
bazelisk run //tools/format:check
./tools/tidy/check.sh
