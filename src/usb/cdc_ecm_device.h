// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// Pico 2 Ethernet NIC - CDC-ECM Device
// Phase 1: USB CDC-ECM device presenting the "cable unplugged" (link down) state.

#ifndef USB_CDC_ECM_DEVICE_H
#define USB_CDC_ECM_DEVICE_H

#include "src/mac/mac_address.h"

namespace pico_ethernet {

// Thin application-layer wrapper over TinyUSB's CDC-ECM net class driver.
//
// Phase 1 holds the link down because the PHY does not exist yet; later phases
// will drive the link state from real hardware. The error-prone USB plumbing
// (descriptors, endpoints, notifications) lives in TinyUSB and usb_descriptors;
// this class owns only the device lifecycle and the link-state policy.
class CdcEcmDevice {
public:
    explicit CdcEcmDevice(const MacAddress& mac_address);

    // Publishes the MAC to TinyUSB, starts the USB device stack, and asserts the
    // initial link state.
    void initialize();

    // Services USB events; call repeatedly from the main loop.
    void task();

    void set_link_up(bool up);
    bool link_up() const { return link_up_; }
    const MacAddress& mac_address() const { return mac_address_; }

private:
    MacAddress mac_address_;
    bool link_up_{false};  // Phase 1: always down
};

} // namespace pico_ethernet

#endif // USB_CDC_ECM_DEVICE_H
