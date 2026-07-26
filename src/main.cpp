// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <cstdint>

#include "src/mac/mac_address.h"
#include "src/phy/phy.h"
#include "src/usb/cdc_ecm_device.h"
#include "src/util/instrumentation.h" // TEMPORARY: MILESTONE 1.5 Step 1 diagnostics
#include "src/util/led_heartbeat.h"

int main() {
    constexpr pico_ethernet::MacAddress DEVICE_MAC{pico_ethernet::MacAddress::generate_default()};
    constexpr std::uint32_t LED_PIN{25}; // Pico 2 onboard LED

    // Bring up the PHY first: it sets the 120 MHz system clock the PIO timing
    // depends on, before the USB stack starts.
    static pico_ethernet::Phy phy{};
    const bool phy_ok{phy.initialize()};

    static pico_ethernet::CdcEcmDevice device{DEVICE_MAC, phy};
    device.initialize();

    // TEMPORARY: MILESTONE 1.5 Step 1 diagnostics. The DWT cycle counter must be
    // armed after the final system clock is set (done in phy.initialize()).
    pico_ethernet::instrument_init();

    pico_ethernet::LedHeartbeat led{LED_PIN};
    if (phy_ok) {
        led.good();
    } else {
        led.bad();
    }

    while (true) {
        ++pico_ethernet::g_instrument.main_loop_laps; // TEMPORARY: M1.5 Step 1
        device.task();
    }

    return 0;
}
