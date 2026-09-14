// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/test/tx_reference.h"

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>

#include "src/phy/tx_level.h"

namespace pico_ethernet {
namespace {

TEST(ReferenceEncoder, OneIsLowThenHighAndZeroIsHighThenLow) {
    // 0x01 = bit0=1 then seven 0s (LSB-first).
    const std::array<std::uint8_t, 1> frame{0x01};
    std::array<HalfBit, 16> out{};
    ASSERT_EQ(encode_reference(frame, out), 16u);

    EXPECT_EQ(out[0].level, LEVEL_NEG);
    EXPECT_EQ(out[1].level, LEVEL_POS);
    for (std::size_t i{2}; i < out.size(); i += 2) {
        EXPECT_EQ(out[i].level, LEVEL_POS) << "half-bit " << i;
        EXPECT_EQ(out[i + 1].level, LEVEL_NEG) << "half-bit " << i + 1;
    }
}

TEST(ReferenceEncoder, OctetsAreSentLsbFirst) {
    // Only the MSB of 0x80 is set, so it is the last bit of the octet on the wire.
    const std::array<std::uint8_t, 1> frame{0x80};
    std::array<HalfBit, 16> out{};
    ASSERT_EQ(encode_reference(frame, out), 16u);

    for (std::size_t i{0}; i < 14; i += 2) {
        EXPECT_EQ(out[i].level, LEVEL_POS) << "half-bit " << i;
        EXPECT_EQ(out[i + 1].level, LEVEL_NEG) << "half-bit " << i + 1;
    }
    EXPECT_EQ(out[14].level, LEVEL_NEG);
    EXPECT_EQ(out[15].level, LEVEL_POS);
}

TEST(ReferenceEncoder, ByteBoundaryContinuesWithTheNextOctetsFirstBit) {
    // The last bit of 0x80 (1) is followed directly by the first bit of 0x01 (1),
    // then by the 0 bits of the second octet.
    const std::array<std::uint8_t, 2> frame{0x80, 0x01};
    std::array<HalfBit, 32> out{};
    ASSERT_EQ(encode_reference(frame, out), 32u);

    EXPECT_EQ(out[14].level, LEVEL_NEG);
    EXPECT_EQ(out[15].level, LEVEL_POS);
    EXPECT_EQ(out[16].level, LEVEL_NEG);
    EXPECT_EQ(out[17].level, LEVEL_POS);
    EXPECT_EQ(out[18].level, LEVEL_POS);
    EXPECT_EQ(out[19].level, LEVEL_NEG);
}

TEST(ReferenceEncoder, WritesTwoHalfBitsPerDataBitAndNoMore) {
    const std::array<std::uint8_t, 3> frame{0x55, 0xD5, 0xFF};
    std::array<HalfBit, 50> out{};
    ASSERT_EQ(encode_reference(frame, out), 48u);

    for (std::size_t i{0}; i < 48; ++i) {
        EXPECT_NE(out[i].level, LEVEL_IDLE) << "half-bit " << i;
    }
    EXPECT_EQ(out[48].level, LEVEL_IDLE);
    EXPECT_EQ(out[49].level, LEVEL_IDLE);
}

TEST(ReferenceEncoder, EmptyFrameWritesNothing) {
    std::array<HalfBit, 2> out{};
    EXPECT_EQ(encode_reference({}, out), 0u);
    EXPECT_EQ(out[0].level, LEVEL_IDLE);
    EXPECT_EQ(out[1].level, LEVEL_IDLE);
}

} // namespace
} // namespace pico_ethernet
