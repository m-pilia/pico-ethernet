// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"
#include "src/mac/test/frame_builder_by_value.h"

namespace pico_ethernet {
namespace {

// A host frame of `size` bytes (>= 14 so the header is well-formed) with a
// recognizable, position-dependent payload.
std::vector<std::uint8_t> make_host_frame(std::size_t size) {
    std::vector<std::uint8_t> frame(size);
    for (std::size_t i{0}; i < size; ++i) {
        frame[i] = static_cast<std::uint8_t>(i & 0xFF);
    }
    return frame;
}

// The MAC frame (destination..payload..FCS), i.e. the wire frame without the
// preamble and SFD.
std::span<const std::uint8_t> mac_frame_with_fcs(const WireFrame& frame) {
    return frame.view().subspan(PREAMBLE_SFD_LEN);
}

} // namespace

TEST(FrameBuilderTest, PrependsPreambleAndSfd) {
    const auto host{make_host_frame(100)};
    const auto built{build_frame(host)};
    ASSERT_TRUE(built.has_value());

    for (std::size_t i{0}; i < PREAMBLE_LEN; ++i) {
        EXPECT_EQ(built->bytes[i], PREAMBLE_BYTE) << "preamble byte " << i;
    }
    EXPECT_EQ(built->bytes[PREAMBLE_LEN], SFD_BYTE);
}

TEST(FrameBuilderTest, CopiesFrameAndAppendsFcsForFullSizeFrame) {
    const auto host{make_host_frame(100)};
    const auto built{build_frame(host)};
    ASSERT_TRUE(built.has_value());

    EXPECT_EQ(built->length, PREAMBLE_SFD_LEN + 100 + FCS_LEN);

    const auto mac{mac_frame_with_fcs(*built)};
    EXPECT_TRUE(std::ranges::equal(mac.first(host.size()), host));
    // A correct frame + its appended FCS yields the fixed CRC residual.
    EXPECT_EQ(crc32(mac), 0x2144DF1Cu);
}

TEST(FrameBuilderTest, PadsShortFrameToMinimumWithZeros) {
    const auto host{make_host_frame(20)};
    const auto built{build_frame(host)};
    ASSERT_TRUE(built.has_value());

    EXPECT_EQ(built->length, PREAMBLE_SFD_LEN + MIN_FRAME_NO_FCS + FCS_LEN);

    const auto mac{mac_frame_with_fcs(*built)};
    for (std::size_t i{host.size()}; i < MIN_FRAME_NO_FCS; ++i) {
        EXPECT_EQ(mac[i], 0) << "pad byte " << i;
    }
    EXPECT_EQ(crc32(mac), 0x2144DF1Cu);
}

TEST(FrameBuilderTest, AcceptsMaximumSizeFrame) {
    const auto host{make_host_frame(MAX_FRAME_NO_FCS)};
    const auto built{build_frame(host)};
    ASSERT_TRUE(built.has_value());
    EXPECT_EQ(built->length, WIRE_CAPACITY);
}

TEST(FrameBuilderTest, RejectsOversizeFrame) {
    const auto host{make_host_frame(MAX_FRAME_NO_FCS + 1)};
    const auto built{build_frame(host)};
    ASSERT_FALSE(built.has_value());
    EXPECT_EQ(built.error(), FrameError::TooLong);
}

} // namespace pico_ethernet
