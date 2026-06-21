// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// Pico 2 Ethernet NIC - Main Entry Point
// Phase 1: present the Pico 2 to a Linux host as a CDC-ECM Ethernet NIC whose
// link is down ("cable unplugged"), since the PHY layer is not implemented yet.

#include <cstdint>

#include "src/mac/mac_address.h"
#include "src/usb/cdc_ecm_device.h"
#include "src/util/led_heartbeat.h"

int main() {
    constexpr pico_ethernet::MacAddress DEVICE_MAC{pico_ethernet::MacAddress::generate_default()};
    constexpr std::uint32_t LED_PIN{25}; // Pico 2 onboard LED

    pico_ethernet::CdcEcmDevice device{DEVICE_MAC};
    device.initialize();

    pico_ethernet::LedHeartbeat led{LED_PIN};
    led.bad();  // Phase 1: the link is always down ("cable unplugged").

    while (true) {
        device.task();
    }

    return 0;
}
