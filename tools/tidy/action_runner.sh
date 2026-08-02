#!/usr/bin/env bash
# Runs clang-tidy and, on success, writes the stamp that marks the action done.
# Args: <stamp> <clang-tidy invocation...>
set -euo pipefail

stamp="$1"
shift

"$@"
: > "${stamp}"
