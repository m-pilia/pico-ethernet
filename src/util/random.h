// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef UTIL_RANDOM_H
#define UTIL_RANDOM_H

#include <cassert>
#include <cstdint>

namespace pico_ethernet {

// Linear congruential generator (Numerical Recipes constants). Deterministic for a
// given seed, which is what the link-pulse jitter and the backoff draw both need:
// enough spread to avoid beating against a peer, and reproducible under test.
class Lcg {
  public:
    explicit constexpr Lcg(std::uint32_t seed)
        : state_{seed != 0 ? seed : 1u} {}

    // The low bits of an LCG have short periods, so the draw comes from the high
    // half of the state.
    constexpr std::uint32_t below(std::uint32_t bound) {
        assert(bound > 0);
        return (next() >> 16) % bound;
    }

  private:
    constexpr std::uint32_t next() {
        state_ = state_ * 1664525u + 1013904223u;
        return state_;
    }

    std::uint32_t state_;
};

} // namespace pico_ethernet

#endif // UTIL_RANDOM_H
