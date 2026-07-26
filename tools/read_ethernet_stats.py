#!/usr/bin/env -S uv run --script
# SPDX-License-Identifier: MIT
# Copyright (c) 2026 Martino Pilia

# /// script
# requires-python = ">=3.9"
# dependencies = ["pyusb>=1.2"]
# ///

"""Read the Pico 2 Ethernet NIC's device-internal counters over USB.

USB access needs root: add a udev rule with uaccess tag:
   SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", TAG+="uaccess"
or with GROUP+MODE (works over ssh/headless, but requires adding the user to the
`plugdev` group):
   SUBSYSTEM=="usb", ATTR{idVendor}=="1209", ATTR{idProduct}=="0001", MODE="0660", GROUP="plugdev"

GET_ETHERNET_STATISTIC is directed at the CDC communications (control) interface
(recipient = interface). Linux refuses such control transfers while cdc_ether owns
that interface (EBUSY / "Resource busy"), so we detach the kernel driver for the
read and reattach it afterwards. Detaching briefly tears down the CDC-ECM netdev;
since the counters are cumulative, read them as a snapshot after a traffic run.
"""

import struct
import sys

import usb.core
import usb.util

VID = 0x1209
PID = 0x0001

GET_ETHERNET_STATISTIC = 0x44

BM_REQUEST_TYPE = usb.util.build_request_type(
    usb.util.CTRL_IN, usb.util.CTRL_TYPE_CLASS, usb.util.CTRL_RECIPIENT_INTERFACE
)

SELECTORS = {
    0x01: "xmit_ok",
    0x02: "rcv_ok",
    0x03: "xmit_error",
    0x04: "rcv_error",
    0x12: "rcv_crc_error",
    # Private diagnostic selectors: the internal RX sub-counters that make up
    # rcv_error (see RxDiagnostic in src/phy/phy_stats.h).
    0xF0: "  bad_preamble",
    0xF1: "  runt",
    0xF2: "  giant",
    0xF3: "  bad_fcs",
    0xF4: "  carrier_glitch",
    0xF5: "  decode_error",
    0xF6: "  pool_overflow",
    0xF7: "  host_backpressure",
    # TEMPORARY: MILESTONE 1.5 Step 1 instrumentation (see src/util/instrumentation.h).
    # Removed together with the on-device module at the end of the milestone.
    0xE0: "uptime_ms",
    0xE1: "main_loop_laps",
    0xE2: "can_xmit_true",
    0xE3: "tud_task_count",
    0xE4: "tud_task_avg_ns",
    0xE5: "tud_task_worst_ns",
    0xE6: "recover_frame_count",
    0xE7: "recover_frame_avg_ns",
    0xE8: "recover_frame_worst_ns",
    0xE9: "rx_irq_count",
    0xEA: "rx_irq_avg_ns",
    0xEB: "rx_irq_worst_ns",
}

CDC_COMMUNICATIONS_CLASS = 0x02


def control_interface_number(dev):
    cfg = dev.get_active_configuration()
    for intf in cfg:
        if intf.bInterfaceClass == CDC_COMMUNICATIONS_CLASS:
            return intf.bInterfaceNumber
    raise SystemExit("no CDC communications (control) interface found")


def main():
    dev = usb.core.find(idVendor=VID, idProduct=PID)
    if dev is None:
        raise SystemExit(f"device {VID:04x}:{PID:04x} not found")

    itf = control_interface_number(dev)

    reattach = dev.is_kernel_driver_active(itf)
    if reattach:
        dev.detach_kernel_driver(itf)
    usb.util.claim_interface(dev, itf)
    values = {}
    try:
        for selector, label in SELECTORS.items():
            try:
                data = dev.ctrl_transfer(
                    BM_REQUEST_TYPE, GET_ETHERNET_STATISTIC, selector, itf, 4
                )
            except usb.core.USBError as exc:
                print(f"{label:<22} <unavailable> ({exc.strerror})", file=sys.stderr)
                continue
            (value,) = struct.unpack("<I", bytes(data))
            values[selector] = value
            print(f"{label:<22} {value}")
    finally:
        usb.util.release_interface(dev, itf)
        if reattach:
            try:
                dev.attach_kernel_driver(itf)
            except usb.core.USBError:
                pass

    # TEMPORARY (MILESTONE 1.5 Step 1): derived per-second rates from the snapshot.
    uptime_ms = values.get(0xE0, 0)
    if uptime_ms:
        secs = uptime_ms / 1000.0
        print("\nderived (per second, cumulative since boot):")
        for selector, label in (
            (0xE1, "main_loop_laps/s"),
            (0xE2, "can_xmit_true/s"),
            (0x02, "rcv_ok/s"),
            (0xE9, "rx_irq/s"),
            (0xE6, "recover_frame/s"),
        ):
            if selector in values:
                print(f"  {label:<20} {values[selector] / secs:,.0f}")


if __name__ == "__main__":
    main()
