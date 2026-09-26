// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_DUPLEX_H
#define PHY_DUPLEX_H

#include <cstdint>

namespace pico_ethernet {

enum class Duplex : std::uint8_t { Half, Full };

} // namespace pico_ethernet

#endif // PHY_DUPLEX_H
