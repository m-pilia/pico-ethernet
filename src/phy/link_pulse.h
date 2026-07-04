// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_LINK_PULSE_H
#define PHY_LINK_PULSE_H

#include <cstdint>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// Normal Link Pulse cadence. A 10BASE-T PHY emits one ~100 ns positive pulse
// every 16 ms +/- 8 ms while the line is otherwise idle; the jitter keeps the
// pulse train from beating against the peer's pulses or frame traffic. Only the
// interval is pure and host-testable here -- emitting the pulse itself is the
// LEVEL state machine's job (a short symbol stream kicked by a timer). FLP burst
// encode/decode for autonegotiation is added later.
class NlpScheduler {
  public:
    explicit constexpr NlpScheduler(std::uint32_t seed = 0x1234'5678u)
        : rng_{seed != 0 ? seed : 1u} {}

    // Next inter-pulse interval, uniform in
    // [NLP_PERIOD_MS - NLP_JITTER_MS, NLP_PERIOD_MS + NLP_JITTER_MS].
    [[nodiscard]] constexpr std::uint32_t next_interval_ms() {
        rng_ = rng_ * 1664525u + 1013904223u; // Numerical Recipes LCG
        constexpr std::uint32_t span{2 * NLP_JITTER_MS + 1};
        return NLP_PERIOD_MS - NLP_JITTER_MS + (rng_ >> 16) % span;
    }

  private:
    std::uint32_t rng_;
};

} // namespace pico_ethernet

#endif // PHY_LINK_PULSE_H
