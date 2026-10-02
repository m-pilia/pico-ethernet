// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_TX_REFERENCE_H
#define PHY_TEST_TX_REFERENCE_H

#include <cstddef>
#include <cstdint>
#include <span>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// Reference (oracle) Manchester encoder for the PHY transmit path.

// One transmitted half-bit as its {TXP,TXN} level code.
struct HalfBit {
    std::uint8_t level; // LEVEL_POS / LEVEL_NEG / LEVEL_IDLE
};

inline constexpr std::size_t TP_IDL_HALF_BITS{TP_IDL_NS / HALF_BIT_NS};
static_assert(TP_IDL_NS % HALF_BIT_NS == 0);

// Encodes a frame as two half-bits per data bit, in transmission order. Returns
// the number of half-bits written (2 * data bits); `out` must hold that many.
std::size_t encode_reference(std::span<const std::uint8_t> frame, std::span<HalfBit> out);

// Encodes a whole transmission: the frame, then the positive start of idle, after
// which the line is idle. Returns the number of half-bits written; `out` must hold
// that many.
std::size_t encode_transmission(std::span<const std::uint8_t> frame, std::span<HalfBit> out);

} // namespace pico_ethernet

#endif // PHY_TEST_TX_REFERENCE_H
