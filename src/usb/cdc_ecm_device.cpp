// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// Pico 2 Ethernet NIC - CDC-ECM Device
// Phase 1: USB CDC-ECM device presenting the "cable unplugged" (link down) state.

#include "src/usb/cdc_ecm_device.h"

#include <algorithm>
#include <cstdint>

#include "tusb.h"

namespace pico_ethernet {

namespace {
constexpr std::uint8_t USB_RHPORT{0};
}

CdcEcmDevice::CdcEcmDevice(const MacAddress& mac_address) : mac_address_{mac_address} {}

void CdcEcmDevice::initialize() {
    // TinyUSB reads tud_network_mac_address while building descriptors, so it
    // must be set before the stack starts.
    std::ranges::copy(mac_address_.bytes(), tud_network_mac_address);

    tusb_init();

    // Phase 1: report the cable as unplugged.
    set_link_up(false);
}

void CdcEcmDevice::task() {
    tud_task();
}

void CdcEcmDevice::set_link_up(bool up) {
    link_up_ = up;
    tud_network_link_state(USB_RHPORT, up);
}

} // namespace pico_ethernet

// TinyUSB network class callbacks (C linkage).
extern "C" {

// Defined by the application; TinyUSB reads it for the iMACAddress descriptor.
std::uint8_t tud_network_mac_address[6] = {0};

bool tud_network_recv_cb(const std::uint8_t* src, std::uint16_t size) {
    (void)src;
    (void)size;
    // Phase 1 (cable unplugged): silently discard. Returning false tells the
    // driver to re-arm the OUT endpoint on our behalf.
    return false;
}

std::uint16_t tud_network_xmit_cb(std::uint8_t* dst, void* ref, std::uint16_t arg) {
    (void)dst;
    (void)ref;
    (void)arg;
    // Phase 1: never transmit.
    return 0;
}

void tud_network_init_cb(void) {
    // No per-interface state to reset in Phase 1.
}

// The combined ECM/RNDIS driver references this RNDIS handler, but we never
// advertise RNDIS (ECM-only descriptors), so it is never called. Stubbing it
// avoids compiling TinyUSB's rndis_reports.c, which depends on lwIP.
void rndis_class_set_handler(std::uint8_t* data, int size) {
    (void)data;
    (void)size;
}

} // extern "C"
