"""A minimal clang-tidy aspect.

Reconstructs each first-party translation unit's real compile command line from
the cc toolchain and runs clang-tidy over it, emitting one stamp per source.
Works for both the host (clang) and the rp2350 (arm-none-eabi gcc) toolchains:
for the gcc target the command line is normalized so clang-tidy's clang can
parse it. gcc-only flags are dropped and clang is pointed at the arm gcc install
(`--target`/`--sysroot`/`--gcc-toolchain`, plus `-stdlib=libstdc++`) so it
rediscovers newlib and libstdc++ the way gcc does implicitly; the one multilib
header directory clang's discovery misses is injected as `-isystem`.
"""

load("@bazel_tools//tools/cpp:toolchain_utils.bzl", "find_cpp_toolchain", "use_cpp_toolchain")
load("@rules_cc//cc:action_names.bzl", "ACTION_NAMES")

_SOURCE_EXTENSIONS = ["cpp", "cc", "cxx", "c"]

# gcc-only flags that clang-tidy's embedded clang rejects. Exact matches.
_GCC_ONLY_FLAGS = [
    "-no-canonical-prefixes",
    "-fno-canonical-system-headers",
    "-mcmse",
]

# gcc-only flags matched by prefix (they carry a value).
_GCC_ONLY_PREFIXES = [
    "-frandom-seed=",
]

def _is_first_party_non_test(label):
    if label.workspace_name:
        return False
    pkg = label.package
    if pkg != "src" and not pkg.startswith("src/"):
        return False
    return "test" not in pkg.split("/")

def _sources(ctx):
    srcs = []
    for src in getattr(ctx.rule.attr, "srcs", []):
        for f in src.files.to_list():
            if f.is_source and f.extension in _SOURCE_EXTENSIONS and f.path.startswith("src/"):
                srcs.append(f)
    return srcs

def _strip_diagnostic_flags(flags):
    # The compiler's warning flags (-Wall/-Wextra/-pedantic) fire on third-party
    # headers through clang's stricter parser and would drown the bug-focused
    # .clang-tidy check set. -Qunused-arguments silences the driver's
    # unused-argument warning for link-only flags left on the compile line.
    kept = ["-Qunused-arguments"]
    for flag in flags:
        if flag.startswith("-W") or flag == "-pedantic" or flag == "--pedantic":
            continue
        kept.append(flag)
    return kept

# The rp2350 toolchain does not expose its target triple or builtin include
# directories through the provider (both come back empty), so the arm gcc
# install is located among the toolchain input files and handed to clang, which
# then rediscovers newlib and the correct libstdc++ multilib the way gcc does
# implicitly.
def _arm_gcc_root(cc_toolchain):
    for f in cc_toolchain.all_files.to_list():
        idx = f.path.find("arm_gcc_linux")
        if idx == -1:
            continue
        end = f.path.find("/", idx)
        return f.path[:end] if end != -1 else f.path
    return None

# libstdc++ ships bits/c++config.h only under the multilib subdirectory, which
# clang's --gcc-toolchain search does not add. The rp2350 always builds for the
# cortex-m33 softfp multilib below; its c++config.h is located among the
# toolchain inputs so the directory can be injected without hard-coding the gcc
# version.
_RP2350_MULTILIB_CONFIG = "/thumb/v8-m.main+fp/softfp/bits/c++config.h"

def _arm_multilib_include(cc_toolchain):
    suffix = "/bits/c++config.h"
    for f in cc_toolchain.all_files.to_list():
        if f.path.endswith(_RP2350_MULTILIB_CONFIG):
            return f.path[:-len(suffix)]
    return None

def _normalize_for_target(flags, arm_root, multilib_include):
    kept = []
    for flag in flags:
        if flag in _GCC_ONLY_FLAGS:
            continue
        if [p for p in _GCC_ONLY_PREFIXES if flag.startswith(p)]:
            continue
        kept.append(flag)

    prefix = [
        "--target=arm-none-eabi",
        "--sysroot=" + arm_root + "/arm-none-eabi",
        "--gcc-toolchain=" + arm_root,
        # Without this clang defaults to its own bundled libc++ instead of the
        # gcc install's libstdc++, and fails looking for __config_site.
        "-stdlib=libstdc++",
    ]
    if multilib_include:
        prefix += ["-isystem", multilib_include]
    return prefix + kept

_TidyStampsInfo = provider(doc = "Transitive clang-tidy stamps.", fields = ["stamps"])

def _transitive_stamps(ctx):
    return [
        dep[_TidyStampsInfo].stamps
        for dep in getattr(ctx.rule.attr, "deps", [])
        if _TidyStampsInfo in dep
    ]

def _clang_tidy_aspect_impl(target, ctx):
    if CcInfo not in target or not _is_first_party_non_test(ctx.label):
        return []

    dep_stamps = _transitive_stamps(ctx)
    srcs = _sources(ctx)
    if not srcs:
        stamps = depset(transitive = dep_stamps)
        return [_TidyStampsInfo(stamps = stamps), OutputGroupInfo(report = stamps)]

    cc_toolchain = find_cpp_toolchain(ctx)
    feature_config = cc_common.configure_features(
        ctx = ctx,
        cc_toolchain = cc_toolchain,
        requested_features = ctx.features,
        unsupported_features = ctx.disabled_features,
    )

    comp_ctx = target[CcInfo].compilation_context
    cpp = ctx.fragments.cpp
    user_flags = cpp.copts + cpp.cxxopts + getattr(ctx.rule.attr, "copts", [])
    arm_root = _arm_gcc_root(cc_toolchain)
    arm_multilib_include = _arm_multilib_include(cc_toolchain) if arm_root else None

    stamps = []
    for src in srcs:
        compile_vars = cc_common.create_compile_variables(
            feature_configuration = feature_config,
            cc_toolchain = cc_toolchain,
            user_compile_flags = user_flags,
            source_file = src.path,
            include_directories = comp_ctx.includes,
            quote_include_directories = comp_ctx.quote_includes,
            system_include_directories = comp_ctx.system_includes,
            framework_include_directories = comp_ctx.framework_includes,
            preprocessor_defines = depset(transitive = [comp_ctx.defines, comp_ctx.local_defines]),
        )
        flags = cc_common.get_memory_inefficient_command_line(
            feature_configuration = feature_config,
            action_name = ACTION_NAMES.cpp_compile,
            variables = compile_vars,
        )
        flags = _strip_diagnostic_flags(flags)
        if arm_root:
            flags = _normalize_for_target(flags, arm_root, arm_multilib_include)

        stamp = ctx.actions.declare_file("%s.%s.tidy" % (ctx.label.name, src.basename))
        args = ctx.actions.args()
        args.add(stamp)
        args.add(ctx.file._clang_tidy)
        args.add("--quiet")
        args.add("--config-file", ctx.file._config)
        args.add(src)
        args.add("--")
        args.add_all(flags)

        ctx.actions.run(
            executable = ctx.executable._runner,
            arguments = [args],
            inputs = depset(
                direct = [src, ctx.file._clang_tidy, ctx.file._config],
                transitive = [comp_ctx.headers, cc_toolchain.all_files],
            ),
            outputs = [stamp],
            mnemonic = "ClangTidy",
            progress_message = "clang-tidy %s" % src.short_path,
        )
        stamps.append(stamp)

    all_stamps = depset(direct = stamps, transitive = dep_stamps)
    return [_TidyStampsInfo(stamps = all_stamps), OutputGroupInfo(report = all_stamps)]

clang_tidy_aspect = aspect(
    implementation = _clang_tidy_aspect_impl,
    attr_aspects = ["deps"],
    fragments = ["cpp"],
    toolchains = use_cpp_toolchain(),
    required_providers = [CcInfo],
    attrs = {
        "_clang_tidy": attr.label(
            default = "@llvm_toolchain//:clang-tidy",
            allow_single_file = True,
            cfg = "exec",
        ),
        "_config": attr.label(
            default = "//:.clang-tidy",
            allow_single_file = True,
        ),
        "_runner": attr.label(
            default = "//tools/tidy:action_runner",
            executable = True,
            cfg = "exec",
        ),
    },
)
