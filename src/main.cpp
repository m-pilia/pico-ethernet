// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <cstddef>
#include <cstdint>

#include "src/mac/mac_address.h"
#include "src/phy/phy.h"
#include "src/usb/usb_net_device.h"
#include "src/util/led_heartbeat.h"

// Bounds the combined footprint of the two large .bss objects so buffer growth
// trips a compile error instead of a silent RAM overrun.
static constexpr std::size_t NIC_STATIC_RAM_BUDGET{std::size_t{40} * 1024};
static_assert(sizeof(pico_ethernet::Phy) + sizeof(pico_ethernet::UsbNetDevice) <= NIC_STATIC_RAM_BUDGET);

int main() {
    constexpr pico_ethernet::MacAddress DEVICE_MAC{pico_ethernet::MacAddress::generate_default()};
    constexpr std::uint32_t LED_PIN{25}; // Pico 2 onboard LED

    // Bring up the PHY first: it sets the 120 MHz system clock the PIO timing
    // depends on, before the USB stack starts.
    static pico_ethernet::Phy phy{};
    const bool phy_ok{phy.initialize()};

    static pico_ethernet::UsbNetDevice device{DEVICE_MAC, phy};
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
