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

// Folds the station address into the seed for the collision backoff draw, which
// must not be shared with the peer on the other end of the segment.
constexpr std::uint32_t backoff_seed(const pico_ethernet::MacAddress& mac) {
    std::uint32_t seed{0};
    for (const std::uint8_t octet : mac.bytes()) {
        seed = seed * 31u + octet;
    }
    return seed;
}

int main() {
    constexpr pico_ethernet::MacAddress DEVICE_MAC{pico_ethernet::MacAddress::generate_default()};
    constexpr std::uint32_t LED_PIN{25}; // Pico 2 onboard LED

    // Bring up the PHY first: it sets the 120 MHz system clock the PIO timing
    // depends on, before the USB stack starts.
    static pico_ethernet::Phy phy{backoff_seed(DEVICE_MAC)};
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
