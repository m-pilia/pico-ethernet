// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/tx_emphasis.h"

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

#include "src/phy/test/tx_reference.h"

namespace pico_ethernet {
namespace {

// Extract emphasis bit i (LSB-first packing) from a compute_emphasis_bits result.
bool emphasis_bit(std::span<const std::uint8_t> packed, std::size_t i) {
    return ((packed[i >> 3] >> (i & 7)) & 1u) != 0;
}

TEST(ComputeEmphasisBits, AlternatingByteIsAllReducedExceptFirst) {
    // 0x55 = 0101_0101; LSB-first the bits alternate 1,0,1,0,... so every bit
    // boundary is a transition -> reduced, except the leading half of bit 0 which
    // transitions out of idle -> full.
    const std::array<std::uint8_t, 1> frame{0x55};
    std::array<std::uint8_t, 1> out{};
    const std::size_t nbits{compute_emphasis_bits(frame, out)};
    ASSERT_EQ(nbits, 8u);

    EXPECT_TRUE(emphasis_bit(out, 0));
    for (std::size_t i{1}; i < 8; ++i) {
        EXPECT_FALSE(emphasis_bit(out, i)) << "bit " << i;
    }
}

TEST(ComputeEmphasisBits, RunOfEqualBitsIsAllFull) {
    // 0x00 = all zeros: no bit changes, so every leading half is full.
    const std::array<std::uint8_t, 1> frame{0x00};
    std::array<std::uint8_t, 1> out{};
    ASSERT_EQ(compute_emphasis_bits(frame, out), 8u);
    for (std::size_t i{0}; i < 8; ++i) {
        EXPECT_TRUE(emphasis_bit(out, i)) << "bit " << i;
    }
}

TEST(ComputeEmphasisBits, BoundaryAcrossOctetsUsesPreviousBit) {
    // 0xFF then 0xFF: the last bit of octet 0 (1) meets the first bit of octet 1
    // (1) -> equal -> full at the octet boundary.
    const std::array<std::uint8_t, 2> frame{0xFF, 0xFF};
    std::array<std::uint8_t, 2> out{};
    ASSERT_EQ(compute_emphasis_bits(frame, out), 16u);
    EXPECT_TRUE(emphasis_bit(out, 8)); // first bit of second octet
}

TEST(ReferenceEncoder, ManchesterPolarityAndEmphasisForOneByte) {
    // 0x01 = bit0=1 then seven 0s (LSB-first): 1,0,0,0,0,0,0,0.
    const std::array<std::uint8_t, 1> frame{0x01};
    std::array<HalfBit, 16> out{};
    const std::size_t n{encode_reference(frame, out)};
    ASSERT_EQ(n, 16u);

    // bit 0 = 1 -> L then H, leading half full (out of idle).
    EXPECT_EQ(out[0].level, LEVEL_NEG);
    EXPECT_TRUE(out[0].full);
    EXPECT_EQ(out[1].level, LEVEL_POS);
    EXPECT_TRUE(out[1].full);

    // bit 1 = 0 -> H then L; boundary 1->0 differs -> leading half reduced.
    EXPECT_EQ(out[2].level, LEVEL_POS);
    EXPECT_FALSE(out[2].full);
    EXPECT_EQ(out[3].level, LEVEL_NEG);
    EXPECT_TRUE(out[3].full);

    // bit 2 = 0 -> H then L; boundary 0->0 equal -> leading half full.
    EXPECT_EQ(out[4].level, LEVEL_POS);
    EXPECT_TRUE(out[4].full);

    // The trailing half of every bit is always full.
    for (std::size_t i{1}; i < n; i += 2) {
        EXPECT_TRUE(out[i].full) << "trailing half " << i;
    }
}

TEST(ReferenceEncoder, EmphasisStreamMatchesComputeEmphasisBits) {
    // The reference encoder's leading-half emphasis must equal the packed
    // annotation the EMPHASIS SM consumes, for the same input.
    const std::array<std::uint8_t, 4> frame{0x55, 0xD5, 0x00, 0xFF};
    std::array<HalfBit, 64> ref{};
    std::array<std::uint8_t, 4> packed{};
    const std::size_t nhalf{encode_reference(frame, ref)};
    const std::size_t nbits{compute_emphasis_bits(frame, packed)};
    ASSERT_EQ(nhalf, nbits * 2);

    for (std::size_t i{0}; i < nbits; ++i) {
        EXPECT_EQ(ref[2 * i].full, emphasis_bit(packed, i)) << "bit " << i;
    }
}

} // namespace
} // namespace pico_ethernet
