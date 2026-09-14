// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TX_LEVEL_H
#define PHY_TX_LEVEL_H

#include <cstdint>

namespace pico_ethernet {

// {TXP,TXN} SET-group codes (TXP is the low bit). The differential pair is driven
// opposite for +/-Vdiff and equal (low) for the 0 V line idle.
inline constexpr std::uint8_t LEVEL_IDLE{0b00};
inline constexpr std::uint8_t LEVEL_POS{0b01}; // TXP=1,TXN=0 -> +Vdiff (Manchester H)
inline constexpr std::uint8_t LEVEL_NEG{0b10}; // TXP=0,TXN=1 -> -Vdiff (Manchester L)

} // namespace pico_ethernet

#endif // PHY_TX_LEVEL_H
