// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// Blinks a status LED to signal device health using a hardware repeating timer,
// so it runs autonomously without any polling from the main loop. The caller
// selects a rate with good()/bad() and can stop the blink with stop().

#ifndef UTIL_LED_HEARTBEAT_H
#define UTIL_LED_HEARTBEAT_H

#include <cstdint>

#include "pico/stdlib.h" // IWYU pragma: keep

namespace pico_ethernet {

// Drives a GPIO-connected LED as a square-wave heartbeat off a repeating timer.
// Two rates encode a coarse health signal: a slow 1 Hz blink for "good", a fast
// 10 Hz blink for "bad". good()/bad() start the timer (or re-arm it at the new
// rate if already running); stop() cancels it and turns the LED off. The toggle
// happens in the timer's IRQ callback, so no main-loop polling is needed.
//
// Registered with the timer by address, so it is neither copyable nor movable.
class LedHeartbeat {
  public:
    explicit LedHeartbeat(std::uint32_t pin)
        : pin_{pin} {
        gpio_init(pin_);
        gpio_set_dir(pin_, GPIO_OUT);
        gpio_put(pin_, on_);
    }

    ~LedHeartbeat() { stop(); }

    LedHeartbeat(const LedHeartbeat&) = delete;
    LedHeartbeat& operator=(const LedHeartbeat&) = delete;

    void good() { set_rate(GOOD_INTERVAL_MS); }
    void bad() { set_rate(BAD_INTERVAL_MS); }

    void stop() {
        cancel();
        on_ = false;
        gpio_put(pin_, on_);
    }

  private:
    // Time between toggles (half the blink period).
    static constexpr std::int32_t GOOD_INTERVAL_MS{500}; // 1 Hz blink
    static constexpr std::int32_t BAD_INTERVAL_MS{50};   // 10 Hz blink

    void set_rate(std::int32_t interval_ms) {
        if (running_ && interval_ms == interval_ms_) {
            return;
        }
        interval_ms_ = interval_ms;
        cancel();
        add_repeating_timer_ms(
            interval_ms_, &LedHeartbeat::on_timer, this, &timer_);
        running_ = true;
    }

    void cancel() {
        if (running_) {
            cancel_repeating_timer(&timer_);
            running_ = false;
        }
    }

    static bool on_timer(repeating_timer_t* timer) {
        auto& self = *static_cast<LedHeartbeat*>(timer->user_data);
        self.on_ = !self.on_;
        gpio_put(self.pin_, self.on_);
        return true; // keep repeating
    }

    std::uint32_t pin_;
    repeating_timer_t timer_{};
    std::int32_t interval_ms_{GOOD_INTERVAL_MS};
    bool running_{false};
    bool on_{false};
};

} // namespace pico_ethernet

#endif // UTIL_LED_HEARTBEAT_H
