// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>

#include "src/mac/crc32.h"

namespace pico_ethernet {

// "123456789" is the canonical CRC-32/ISO-HDLC check vector; its FCS is a
// well-known constant, which pins both the polynomial and the reflect/invert
// convention.
constexpr std::array<std::uint8_t, 9> CHECK_INPUT{'1', '2', '3', '4',
                                                  '5', '6', '7', '8', '9'};

static_assert(crc32(CHECK_INPUT) == 0xCBF43926u);
static_assert(crc32(std::span<const std::uint8_t>{}) == 0u);

TEST(Crc32Test, CanonicalCheckVector) {
    EXPECT_EQ(crc32(CHECK_INPUT), 0xCBF43926u);
}

TEST(Crc32Test, EmptyInputIsZero) {
    EXPECT_EQ(crc32(std::span<const std::uint8_t>{}), 0u);
}

// Appending a frame's own (little-endian) FCS and re-running the CRC yields the
// fixed residual 0x2144DF1C — the property the receiver uses to validate a frame
// in one pass over destination..FCS.
TEST(Crc32Test, FullFrameResidualIsConstant) {
    std::array<std::uint8_t, 13> framed{};
    std::ranges::copy(CHECK_INPUT, framed.begin());
    const std::uint32_t fcs{crc32(CHECK_INPUT)};
    framed[9] = static_cast<std::uint8_t>(fcs);
    framed[10] = static_cast<std::uint8_t>(fcs >> 8);
    framed[11] = static_cast<std::uint8_t>(fcs >> 16);
    framed[12] = static_cast<std::uint8_t>(fcs >> 24);

    EXPECT_EQ(crc32(framed), 0x2144DF1Cu);
}

TEST(Crc32Test, SingleBitChangeAltersCrc) {
    std::array<std::uint8_t, 9> mutated{CHECK_INPUT};
    mutated[0] ^= 0x01;
    EXPECT_NE(crc32(mutated), crc32(CHECK_INPUT));
}

} // namespace pico_ethernet
