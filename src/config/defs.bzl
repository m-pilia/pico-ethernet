# Core-0 stack the linker reserves (crt0.S) via PICO_STACK_SIZE, overriding the
# SDK's 0x800 default through the generated PICO_CONFIG_EXTRA_HEADER; see the
# pico_config_extra target in this package's BUILD.
PICO_STACK_SIZE_BYTES = 2048

# Per-function stack-frame ceiling for firmware translation units. -Wstack-usage is
# GCC-only, so it is scoped to the arm-none-eabi (rp2350) build; the host clang
# build has no equivalent flag.
STACK_USAGE_COPTS = select({
    "@pico-sdk//bazel/constraint:rp2350": [
        "-Werror=stack-usage={}".format(PICO_STACK_SIZE_BYTES),
    ],
    "//conditions:default": [],
})
