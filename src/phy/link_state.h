// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_LINK_STATE_H
#define PHY_LINK_STATE_H

#include <cstdint>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// 10BASE-T link integrity test (IEEE 802.3 Clause 14.2.1.7): the link passes after
// LINK_UP_EVENTS consecutive valid link pulses spaced no further apart than the
// link-loss window, or at once on a received frame, and fails again once the window
// passes with neither.
//
// Pure: pulses and frames are reported by the caller as they are observed, and the
// loss timer is driven by advance(). Timestamps are microseconds from a free-running
// 32-bit counter, compared as unsigned differences so a counter wrap is transparent.
class LinkState {
  public:
    constexpr void on_pulse(std::uint32_t now_us) {
        if (silent_for(now_us)) {
            events_ = 0;
        }
        last_activity_us_ = now_us;
        if (events_ < LINK_UP_EVENTS) {
            ++events_;
        }
        up_ = events_ == LINK_UP_EVENTS;
    }

    constexpr void on_frame(std::uint32_t now_us) {
        last_activity_us_ = now_us;
        events_ = LINK_UP_EVENTS;
        up_ = true;
    }

    constexpr void advance(std::uint32_t now_us) {
        if (events_ > 0 && silent_for(now_us)) {
            events_ = 0;
            up_ = false;
        }
    }

    [[nodiscard]] constexpr bool up() const { return up_; }

  private:
    [[nodiscard]] constexpr bool silent_for(std::uint32_t now_us) const {
        return events_ > 0 && now_us - last_activity_us_ > LINK_LOSS_US;
    }

    std::uint32_t last_activity_us_{0};
    std::uint32_t events_{0};
    bool up_{false};
};

} // namespace pico_ethernet

#endif // PHY_LINK_STATE_H
