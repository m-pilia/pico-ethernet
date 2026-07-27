#!/usr/bin/env bash
# Builds a MemorySanitizer-instrumented libc++/libc++abi/libunwind against the
# hermetic LLVM toolchain and packages it in the layout toolchains_llvm expects
# for its `libcxx_url` attribute (extracted into `libcxx-msan/`). MSan needs an
# instrumented C++ standard library or it reports false positives for any data
# flowing through libc++; the upstream LLVM release tarballs do not ship one, so
# we build it here. The resulting tarball + its sha256 are wired into
# MODULE.bazel (llvm.toolchain(libcxx_url=..., libcxx_sha256=...)).
#
# Output: .cache/msan_libcxx_build/libcxx-msan-<llvm>-<triple>.tar.xz (+ .sha256)
set -euo pipefail

LLVM_VERSION="${LLVM_VERSION:-20.1.3}"
TARGET_TRIPLE="${TARGET_TRIPLE:-x86_64-unknown-linux-gnu}"
SRC_SHA256="b6183c41281ee3f23da7fda790c6d4f5877aed103d1e759763b1008bdd0e2c50"

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
work="${repo_root}/.cache/msan_libcxx_build"
mkdir -p "${work}"
cd "${work}"

# Resolve the hermetic clang so the instrumented runtimes match the toolchain
# that will link against them.
ext="$(bazelisk info output_base)/external"
clang_bin="$(find "${ext}" -path '*llvm_toolchain_llvm/bin/clang' 2>/dev/null | head -1)"
if [[ -z "${clang_bin}" ]]; then
    echo "hermetic clang not found; run a host build first to fetch the toolchain" >&2
    exit 1
fi
clang_dir="$(dirname "${clang_bin}")"

tarball="llvm-project-${LLVM_VERSION}.src.tar.xz"
if [[ ! -f "${tarball}" ]]; then
    curl -fsSL -o "${tarball}" \
        "https://github.com/llvm/llvm-project/releases/download/llvmorg-${LLVM_VERSION}/${tarball}"
fi
echo "${SRC_SHA256}  ${tarball}" | sha256sum -c -

src="llvm-project-${LLVM_VERSION}.src"
[[ -d "${src}" ]] || tar xf "${tarball}"

build="build-${LLVM_VERSION}"
stage="stage-${LLVM_VERSION}"
rm -rf "${build}" "${stage}"

cmake -S "${src}/runtimes" -B "${build}" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_C_COMPILER="${clang_dir}/clang" \
    -DCMAKE_CXX_COMPILER="${clang_dir}/clang++" \
    -DCMAKE_C_COMPILER_TARGET="${TARGET_TRIPLE}" \
    -DCMAKE_CXX_COMPILER_TARGET="${TARGET_TRIPLE}" \
    -DCMAKE_INSTALL_PREFIX="${stage}" \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DLLVM_ENABLE_RUNTIMES="libcxx;libcxxabi;libunwind" \
    -DLLVM_ENABLE_PER_TARGET_RUNTIME_DIR=ON \
    -DLLVM_USE_SANITIZER=MemoryWithOrigins \
    -DLIBCXX_ENABLE_SHARED=OFF \
    -DLIBCXX_ENABLE_STATIC=ON \
    -DLIBCXXABI_ENABLE_SHARED=OFF \
    -DLIBCXXABI_ENABLE_STATIC=ON \
    -DLIBCXXABI_USE_LLVM_UNWINDER=ON \
    -DLIBUNWIND_ENABLE_SHARED=OFF \
    -DLIBUNWIND_ENABLE_STATIC=ON \
    -DLIBCXX_CXX_ABI=libcxxabi \
    -DLIBCXX_INCLUDE_BENCHMARKS=OFF \
    -DLIBCXX_INCLUDE_TESTS=OFF

cmake --build "${build}" --target install-cxx install-cxxabi install-unwind -j"$(nproc)"

# toolchains_llvm expects, under libcxx-msan/: include/c++/v1,
# include/<triple>/c++/v1 (holds __config_site), and the static archives under
# lib and lib/<triple>. The per-target runtime dir install already yields this.
out="libcxx-msan-${LLVM_VERSION}-${TARGET_TRIPLE}"
rm -rf "${out}"
mkdir -p "${out}"
cp -a "${stage}/include" "${out}/include"
cp -a "${stage}/lib" "${out}/lib"

archive="${out}.tar.xz"
tar -C "${out}" -cJf "${archive}" include lib
sha256sum "${archive}" | tee "${archive}.sha256"

echo "built ${work}/${archive}"
