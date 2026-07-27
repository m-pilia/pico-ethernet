#!/usr/bin/env bash
# Must be invoked with `bazel run`.
set -euo pipefail

# --- begin runfiles.bash initialization ---
if [[ ! -d "${RUNFILES_DIR:-/dev/null}" && ! -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
    if [[ -f "$0.runfiles_manifest" ]]; then
        export RUNFILES_MANIFEST_FILE="$0.runfiles_manifest"
    elif [[ -f "$0.runfiles/MANIFEST" ]]; then
        export RUNFILES_MANIFEST_FILE="$0.runfiles/MANIFEST"
    elif [[ -d "$0.runfiles" ]]; then
        export RUNFILES_DIR="$0.runfiles"
    fi
fi
if [[ -f "${RUNFILES_DIR:-/dev/null}/bazel_tools/tools/bash/runfiles/runfiles.bash" ]]; then
    source "${RUNFILES_DIR}/bazel_tools/tools/bash/runfiles/runfiles.bash"
elif [[ -f "${RUNFILES_MANIFEST_FILE:-/dev/null}" ]]; then
    source "$(grep -m1 '^bazel_tools/tools/bash/runfiles/runfiles.bash ' \
        "${RUNFILES_MANIFEST_FILE}" | cut -d ' ' -f 2-)"
else
    echo "runfiles.bash not found" >&2
    exit 1
fi
# --- end runfiles.bash initialization ---

clang_format="$(rlocation "$1")"
mode="$2"

cd "${BUILD_WORKSPACE_DIRECTORY:?must be run with 'bazel run'}"

mapfile -t files < <(git ls-files '*.c' '*.cc' '*.cpp' '*.h' '*.hpp')

case "${mode}" in
    check) exec "${clang_format}" --dry-run --Werror "${files[@]}" ;;
    fix) exec "${clang_format}" -i "${files[@]}" ;;
    *)
        echo "unknown mode: ${mode}" >&2
        exit 2
        ;;
esac
