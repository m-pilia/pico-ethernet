// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TX_EMPHASIS_H
#define PHY_TX_EMPHASIS_H

#include <cstddef>
#include <cstdint>
#include <span>

namespace pico_ethernet {

// {TXP,TXN} SET-group codes (TXP is the low bit). The differential pair is driven
// opposite for +/-Vdiff and equal (low) for the 0 V line idle.
inline constexpr std::uint8_t LEVEL_IDLE{0b00};
inline constexpr std::uint8_t LEVEL_POS{0b01}; // TXP=1,TXN=0 -> +Vdiff (Manchester H)
inline constexpr std::uint8_t LEVEL_NEG{0b10}; // TXP=0,TXN=1 -> -Vdiff (Manchester L)

// Emphasis annotation stream consumed by the EMPHASIS state machine: one bit per
// data bit (the leading-half emphasis; the SM hard-codes the trailing half to
// full). The leading half of a data bit is driven full only when the bit boundary
// carries a fresh line transition, i.e. when the previous and current data bits
// are equal; otherwise it continues the previous level (the pulse tail) and is
// reduced. Bits are packed LSB-first to match the LEVEL SM's LSB-first octet shift
// (IEEE 802.3 transmits each octet least-significant-bit first). The leading half
// of the very first bit transitions out of the idle line, so it is full. Returns
// the number of data bits written; `out` must hold ceil(bits / 8) bytes.
std::size_t compute_emphasis_bits(std::span<const std::uint8_t> frame, std::span<std::uint8_t> out);

} // namespace pico_ethernet

#endif // PHY_TX_EMPHASIS_H
