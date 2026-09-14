// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/test/tx_reference.h"

#include <cassert>

#include "src/phy/tx_level.h"

namespace pico_ethernet {

namespace {
// Bit i of the frame in transmission order: octets are sent LSB-first.
constexpr bool bit_at(std::span<const std::uint8_t> frame, std::size_t i) {
    return ((frame[i >> 3] >> (i & 7)) & 1u) != 0;
}
} // namespace

std::size_t encode_reference(std::span<const std::uint8_t> frame, std::span<HalfBit> out) {
    const std::size_t nbits{frame.size() * 8};
    assert(out.size() >= nbits * 2);

    // Manchester encoding: 1 = low->high, 0 = high->low.
    for (std::size_t i{0}; i < nbits; ++i) {
        const bool one{bit_at(frame, i)};
        out[2 * i] = HalfBit{one ? LEVEL_NEG : LEVEL_POS};
        out[2 * i + 1] = HalfBit{one ? LEVEL_POS : LEVEL_NEG};
    }
    return nbits * 2;
}

} // namespace pico_ethernet
