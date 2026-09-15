# Pico 2 Ethernet NIC Emulation

Technical demo of a 10BASE-T Ethernet NIC built on top of a Raspberry Pi Pico 2
(RP2350) microcontroller, presented to a Linux host via USB as a [CDC-NCM
device](https://en.wikipedia.org/wiki/Ethernet_over_USB).

# Goals

* Compliant 10BASE-T Ethernet NIC, highest possible 10BASE-T speed over a USB
  1.1 link.
* Own design and implementation of the PHY, no external hardware or software
  solution.
* Efficient implementation with minimal resources. Core 1 is completely unused,
  and so are most of the memory and PIO SMs. There is room to comfortably run a
  non-trivial second application in parallel on the Pico.
* Production-quality firmware implementation following industrial best practices.

# Building

```bash
# Build the firmware for RP2350
bazelisk build --config=rp2350 //src:pico_ethernet_firmware

# Run host-based unit tests
bazelisk test //src/...
```

The build produces a UF2 image at `bazel-bin/src/pico_ethernet_firmware.uf2`.

# Checks

A pre-push script runs the checks locally:
```bash
./tools/pre_push.sh
```

The host unit tests can be run under sanitizers:

```bash
bazelisk test --config=asan  //src/...
bazelisk test --config=ubsan //src/...
bazelisk test --config=msan  //src/...
```

Additional checks:

```bash
bazelisk run   //tools/format:check
bazelisk run   //tools/format:fix
bazelisk build --config=rp2350 //tools/checks:no_alloc_gate
bazelisk build --config=rp2350 //tools/checks:binary_size_gate
```

## On-target TX PIO self-test

Two binaries to test TX and RX PIO SM logic on target are provided. To execute
them, build and flash, then check the results over a UART console (e.g. via
`minicom -D /dev/ttyACM0 -b 115200`):

```bash
bazelisk build --config=rp2350 //src/phy/test:rx_pio_selftest
bazelisk build --config=rp2350 //src/phy/test:tx_pio_selftest
```

Since the tests use a slower clock, it is recommended to disconnect power to
the TX circuit (e.g. by unplugging the VBUS pin connection) to avoid holding
the transformer under prolonged differential drive.

## Reading device statistics

The firmware exposes TX and RX statistics over the standard
`GET_ETHERNET_STATISTIC` control request (advertised via `bmEthernetStatistics`
in the Ethernet Networking Functional Descriptor).

Since no mainline Linux tool issues this request, a script to read the
statistics is provided (add a udev rule for VID:PID `1209:0001` to avoid the
need for root):

```bash
uv run tools/read_ethernet_stats.py
```

## Credits

Most of the source code is LLM generated with a mix of models (GLM 5.3 and 5.2,
GPT-5.6-Sol and GPT-6-astra, Claude Opus 4.8).

## License

This project is made available under a [MIT LICENSE](LICENSE).

## Disclaimer

This is a technical demo, and while it is developed following best engineering
practices and with significant effort on testing and validation, it remains
primarily a project for demonstration purpose. Do not consider it
production-ready, and use it solely at your own risk.
