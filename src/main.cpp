// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <cstdint>

#include "src/mac/mac_address.h"
#include "src/phy/phy.h"
#include "src/usb/cdc_ecm_device.h"
#include "src/util/led_heartbeat.h"

int main() {
    constexpr pico_ethernet::MacAddress DEVICE_MAC{pico_ethernet::MacAddress::generate_default()};
    constexpr std::uint32_t LED_PIN{25}; // Pico 2 onboard LED

    // Bring up the PHY first: it sets the 120 MHz system clock the PIO timing
    // depends on, before the USB stack starts.
    pico_ethernet::Phy phy{};
    const bool phy_ok{phy.initialize()};

    pico_ethernet::CdcEcmDevice device{DEVICE_MAC, phy};
    device.initialize();

    pico_ethernet::LedHeartbeat led{LED_PIN};
    if (phy_ok) {
        led.good();
    } else {
        led.bad();
    }

    while (true) {
        device.task();
    }

    return 0;
}
