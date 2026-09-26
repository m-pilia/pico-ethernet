// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_PEER_CODE_WORDS_H
#define PHY_TEST_PEER_CODE_WORDS_H

#include <cstdint>

#include "src/phy/autoneg.h"

namespace pico_ethernet {

inline constexpr std::uint16_t PEER_HALF_AND_FULL{LCW_SELECTOR_IEEE_802_3 | LCW_10BASE_T | LCW_10BASE_T_FULL_DUPLEX};
inline constexpr std::uint16_t PEER_HALF_ONLY{LCW_SELECTOR_IEEE_802_3 | LCW_10BASE_T};

} // namespace pico_ethernet

#endif // PHY_TEST_PEER_CODE_WORDS_H
