// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef UTIL_NIC_LEDS_H
#define UTIL_NIC_LEDS_H

#include <cstdint>
#include <initializer_list>

#include "pico/stdlib.h" // IWYU pragma: keep

#include "src/util/activity_blink.h"

namespace pico_ethernet {

// The jack's link and activity LEDs, both GPIO-sourced and active high. Stepped
// from the main loop rather than a timer, so it adds no interrupt latency; one lap
// is a small fraction of a blink phase.
class NicLeds {
  public:
    struct Pins {
        std::uint32_t link;
        std::uint32_t activity;
    };

    explicit NicLeds(const Pins& pins)
        : pins_{pins} {
        for (const std::uint32_t pin : {pins_.link, pins_.activity}) {
            gpio_init(pin);
            gpio_set_dir(pin, GPIO_OUT);
            gpio_put(pin, false);
        }
    }

    // `activity` is a monotonic count of wire events; only its changes matter.
    void update(bool link_up, std::uint32_t activity) {
        gpio_put(pins_.link, link_up);
        gpio_put(pins_.activity, activity_blink_.step(activity, time_us_32()));
    }

  private:
    Pins pins_;
    ActivityBlink activity_blink_{};
};

} // namespace pico_ethernet

#endif // UTIL_NIC_LEDS_H
