# Pico 2 Ethernet NIC Emulation

Technical demo of a 10BASE-T Ethernet NIC built on top of a Raspberry Pi Pico 2
(RP2350) microcontroller, presented to a Linux host via USB as a [CDC-NCM
device](https://en.wikipedia.org/wiki/Ethernet_over_USB).

# Goals

* Compliant 10BASE-T Ethernet NIC, highest possible 10BASE-T speed over a USB
  1.1 link. Support for full-duplex and half-duplex with CSMA/CD.
* Own design and implementation of the PHY, no external hardware or software
  solution.
* Efficient implementation with minimal resources. Core 1 is completely unused,
  and so are PIO 1 and 2, 14 out of the 16 DMA channels, 3 out of 4 DMA
  interrupt lines, and most of the memory. There is room to comfortably run a
  non-trivial second application in parallel on the Pico.
* Quality firmware implementation following best practices.

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

## On-target tests

Binaries to test logic that is runnable on target platform only, e.g. TX and RX
PIO SMs, are provided. To execute them, build and flash, then check the results
over a UART console (e.g. via `minicom -D /dev/ttyACM0 -b 115200`):

```bash
bazelisk build --config=rp2350 //src/phy/test:rx_pio_selftest
bazelisk build --config=rp2350 //src/phy/test:tx_pio_selftest
bazelisk build --config=rp2350 //src/phy/test:csma_cd_selftest
bazelisk build --config=rp2350 //src/phy/test:autoneg_selftest
```

## Peer testing

Two tools are provided to test against a peer interface (on the same
development host).

`peer_flood_test.py` is the simpler one. It allows to produce a flood of frames
(using [Mausezahn](https://en.wikipedia.org/wiki/Mausezahn), which needs to be
installed and available in the `$PATH`).

```bash
uv run tools/peer_flood_test.py --help
```

`wire_test.py` allows to send traffic between peers, and verifies that frames
are correctly received.

```bash
uv run tools/wire_test.ph --help
```

Both tools allow to optionally show counters on host or device.

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

## Debugging

To build with debug symbols:
```bash
bazelisk build --config=rp2350 --copt=-g --strip=never //src:pico_ethernet_firmware
```

To attach a debugger on a running application (or potentially a crashed one),
e.g. using [Raspberry Pi pico's openocd
fork](https://github.com/raspberrypi/openocd):
```bash
openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg -c "adapter speed 5000"
```

Then connect a debugger (e.g. `gdb`) as usual, for example:
```
gdb bazel-bin/src/pico_ethernet_firmware \
  -ex "target extended-remote :3333" \
  -ex "bt" -ex "frame 2" -ex "info args" -ex "info locals" \
  -ex "p *ep" -ex "p/x *buf_reg" -ex "x/1xw 0x50110058"
```

## Credits

Most of the source code is LLM generated with a mix of models (GLM 5.3 and 5.2,
GPT-5.6-Sol and GPT-6-astra, Claude Opus 4.8).

## Disclaimer

This is a technical demo, and while it is developed following best engineering
practices and with significant effort on testing and validation, it remains
primarily a project for demonstration purpose. Do not consider it
production-ready, and use it solely at your own risk.

## License

This project is made available under a [MIT LICENSE](LICENSE).
