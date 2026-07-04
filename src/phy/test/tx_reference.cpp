// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/test/tx_reference.h"

#include <cassert>

#include "src/phy/tx_emphasis.h"

namespace pico_ethernet {

namespace {
constexpr bool bit_at(std::span<const std::uint8_t> frame, std::size_t i) {
    return ((frame[i >> 3] >> (i & 7)) & 1u) != 0;
}

// Leading-half emphasis: full iff the bit boundary carries a fresh transition,
// i.e. the previous and current data bits are equal.
constexpr bool leading_half_full(bool prev_bit, bool cur_bit) { return prev_bit == cur_bit; }

// Manchester encoding: 1 = low->high, 0 = high->low.
void emit(std::span<HalfBit> out, std::size_t i, bool cur, bool full) {
    out[2 * i] = HalfBit{cur ? LEVEL_NEG : LEVEL_POS, full};
    out[2 * i + 1] = HalfBit{cur ? LEVEL_POS : LEVEL_NEG, true};
}
} // namespace

std::size_t encode_reference(std::span<const std::uint8_t> frame, std::span<HalfBit> out) {
    const std::size_t nbits{frame.size() * 8};
    assert(out.size() >= nbits * 2);
    if (nbits == 0) {
        return 0;
    }

    // The leading half of bit 0 transitions out of the idle line, so it is full.
    bool prev{bit_at(frame, 0)};
    emit(out, 0, prev, true);
    for (std::size_t i{1}; i < nbits; ++i) {
        const bool cur{bit_at(frame, i)};
        emit(out, i, cur, leading_half_full(prev, cur));
        prev = cur;
    }
    return nbits * 2;
}

} // namespace pico_ethernet
