# Core-0 stack the linker reserves (crt0.S) via PICO_STACK_SIZE, overriding the
# SDK's 0x800 default through the generated PICO_CONFIG_EXTRA_HEADER; see the
# pico_config_extra target in this package's BUILD. The matching per-function
# stack-frame ceiling (-Werror=stack-usage) is applied to all first-party rp2350
# sources via a per_file_copt in .bazelrc; keep that budget in sync with this value.
PICO_STACK_SIZE_BYTES = 2048
