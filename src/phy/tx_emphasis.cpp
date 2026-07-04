// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/tx_emphasis.h"

#include <cassert>

namespace pico_ethernet {

namespace {
// Bit i of the frame in transmission order: octets are sent LSB-first.
constexpr bool bit_at(std::span<const std::uint8_t> frame, std::size_t i) {
    return ((frame[i >> 3] >> (i & 7)) & 1u) != 0;
}

// The leading half of a data bit is full-amplitude iff the bit boundary carries a
// fresh transition, which happens exactly when the previous and current data bits
// are equal.
constexpr bool leading_half_full(bool prev_bit, bool cur_bit) { return prev_bit == cur_bit; }
} // namespace

std::size_t compute_emphasis_bits(std::span<const std::uint8_t> frame, std::span<std::uint8_t> out) {
    const std::size_t nbits{frame.size() * 8};
    const std::size_t nbytes{(nbits + 7) / 8};
    assert(out.size() >= nbytes);

    for (std::size_t i{0}; i < nbytes; ++i) {
        out[i] = 0;
    }
    if (nbits == 0) {
        return 0;
    }

    // The leading half of bit 0 transitions out of the idle line, so it is full.
    out[0] |= static_cast<std::uint8_t>(1u);
    bool prev{bit_at(frame, 0)};
    for (std::size_t i{1}; i < nbits; ++i) {
        const bool cur{bit_at(frame, i)};
        if (leading_half_full(prev, cur)) {
            out[i >> 3] |= static_cast<std::uint8_t>(1u << (i & 7));
        }
        prev = cur;
    }
    return nbits;
}

} // namespace pico_ethernet
