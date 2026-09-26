// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef UTIL_ACTIVITY_BLINK_H
#define UTIL_ACTIVITY_BLINK_H

#include <cstdint>

namespace pico_ethernet {

// Blinks at a fixed cadence while a monotonic activity count keeps changing, so a
// single event gives one flash and continuous activity a steady flicker rather
// than a solid light.
//
// Times are compared as unsigned differences and the count only for inequality, so
// wraps of either are transparent. Whether a blink is running is kept explicitly
// rather than derived from the elapsed time, so an idle period longer than the
// clock wrap cannot fake a flash.
class ActivityBlink {
  public:
    static constexpr std::uint32_t ON_US{50'000};
    static constexpr std::uint32_t PERIOD_US{2 * ON_US};

    // Returns whether the indicator is lit.
    [[nodiscard]] constexpr bool step(std::uint32_t activity, std::uint32_t now_us) {
        if (blinking_ && now_us - start_us_ >= PERIOD_US) {
            blinking_ = false;
        }
        if (!blinking_ && activity != seen_) {
            seen_ = activity;
            start_us_ = now_us;
            blinking_ = true;
        }
        return blinking_ && now_us - start_us_ < ON_US;
    }

  private:
    std::uint32_t seen_{0};
    std::uint32_t start_us_{0};
    bool blinking_{false};
};

} // namespace pico_ethernet

#endif // UTIL_ACTIVITY_BLINK_H
