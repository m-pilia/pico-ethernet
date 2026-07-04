// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_TX_REFERENCE_H
#define PHY_TEST_TX_REFERENCE_H

#include <cstddef>
#include <cstdint>
#include <span>

namespace pico_ethernet {

// Reference (oracle) Manchester + pre-emphasis encoder for the PHY transmit path.

// One transmitted half-bit: the 2-bit {TXP,TXN} level code plus the TXE emphasis
// select (true = full amplitude, false = reduced tail).
struct HalfBit {
    std::uint8_t level; // LEVEL_POS / LEVEL_NEG / LEVEL_IDLE
    bool full;
};

// Encodes a frame as two half-bits per data bit, in transmission order. Returns
// the number of half-bits written (2 * data bits); `out` must hold that many.
std::size_t encode_reference(std::span<const std::uint8_t> frame, std::span<HalfBit> out);

} // namespace pico_ethernet

#endif // PHY_TEST_TX_REFERENCE_H
