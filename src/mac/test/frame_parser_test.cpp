// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"
#include "src/mac/frame_filter.h"
#include "src/mac/frame_parser.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {
namespace {

constexpr MacAddress make_mac(MacAddress::Bytes bytes) {
    return MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(bytes));
}

constexpr MacAddress OUR_MAC{make_mac({0x02, 0, 0, 0, 0, 0x01})};
constexpr MacAddress OTHER_MAC{make_mac({0x06, 0, 0, 0, 0, 0x09})};

// A host frame addressed to `dest` with a position-dependent payload.
std::vector<std::uint8_t> make_host_frame(const MacAddress& dest, std::size_t size) {
    std::vector<std::uint8_t> frame(size);
    for (std::size_t i{0}; i < size; ++i) {
        frame[i] = static_cast<std::uint8_t>(i & 0xFF);
    }
    std::ranges::copy(dest.bytes(), frame.begin());
    return frame;
}

// Wrap a raw MAC frame (destination..payload..FCS) in a preamble + SFD without
// otherwise validating it, for crafting malformed inputs.
std::vector<std::uint8_t> wrap_with_preamble(std::span<const std::uint8_t> mac_frame) {
    std::vector<std::uint8_t> wire(PREAMBLE_LEN, PREAMBLE_BYTE);
    wire.push_back(SFD_BYTE);
    wire.insert(wire.end(), mac_frame.begin(), mac_frame.end());
    return wire;
}

FrameFilter promiscuous() {
    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(FrameFilter::PROMISCUOUS);
    return filter;
}

} // namespace

TEST(FrameParserTest, RoundTripRecoversHostFrame) {
    const auto host = make_host_frame(OUR_MAC, 100);
    const auto built = build_frame(host);
    ASSERT_TRUE(built.has_value());

    const auto parsed = parse_frame(built->view(), promiscuous());
    ASSERT_TRUE(parsed.has_value());
    EXPECT_TRUE(std::ranges::equal(*parsed, host));
}

TEST(FrameParserTest, RoundTripRecoversPaddedShortFrame) {
    const auto host = make_host_frame(OUR_MAC, 20);
    const auto built = build_frame(host);
    ASSERT_TRUE(built.has_value());

    const auto parsed = parse_frame(built->view(), promiscuous());
    ASSERT_TRUE(parsed.has_value());
    // The recovered frame is the padded 60-byte frame, not the original 20.
    EXPECT_EQ(parsed->size(), MIN_FRAME_NO_FCS);
    EXPECT_TRUE(std::ranges::equal(parsed->first(host.size()), host));
}

TEST(FrameParserTest, RejectsMissingSfd) {
    const std::vector<std::uint8_t> all_preamble(PREAMBLE_LEN, PREAMBLE_BYTE);
    const auto parsed = parse_frame(all_preamble, promiscuous());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::BadPreamble);
}

TEST(FrameParserTest, RejectsGarbagePrefix) {
    const std::array<std::uint8_t, 4> garbage{0x00, 0x11, 0x22, 0x33};
    const auto parsed = parse_frame(garbage, promiscuous());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::BadPreamble);
}

TEST(FrameParserTest, RejectsRunt) {
    const std::vector<std::uint8_t> short_mac(MIN_FRAME_WITH_FCS - 1, 0xAB);
    const auto wire = wrap_with_preamble(short_mac);
    const auto parsed = parse_frame(wire, promiscuous());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::Runt);
}

TEST(FrameParserTest, RejectsGiant) {
    const std::vector<std::uint8_t> long_mac(MAX_FRAME_WITH_FCS + 1, 0xAB);
    const auto wire = wrap_with_preamble(long_mac);
    const auto parsed = parse_frame(wire, promiscuous());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::Giant);
}

TEST(FrameParserTest, RejectsCorruptedFcs) {
    const auto host = make_host_frame(OUR_MAC, 100);
    auto built = build_frame(host);
    ASSERT_TRUE(built.has_value());
    // Flip a payload bit inside the frame (after preamble/SFD), keeping length valid.
    built->bytes[PREAMBLE_SFD_LEN + MAC_HEADER_LEN] ^= 0x01;

    const auto parsed = parse_frame(built->view(), promiscuous());
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::BadFcs);
}

TEST(FrameParserTest, DirectedFilterAcceptsOurFrame) {
    const auto host = make_host_frame(OUR_MAC, 100);
    const auto built = build_frame(host);
    ASSERT_TRUE(built.has_value());

    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(FrameFilter::DIRECTED);
    const auto parsed = parse_frame(built->view(), filter);
    ASSERT_TRUE(parsed.has_value());
    EXPECT_TRUE(std::ranges::equal(*parsed, host));
}

TEST(FrameParserTest, DirectedFilterRejectsForeignFrame) {
    const auto host = make_host_frame(OTHER_MAC, 100);
    const auto built = build_frame(host);
    ASSERT_TRUE(built.has_value());

    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(FrameFilter::DIRECTED);
    const auto parsed = parse_frame(built->view(), filter);
    ASSERT_FALSE(parsed.has_value());
    EXPECT_EQ(parsed.error(), FrameError::Filtered);
}

} // namespace pico_ethernet
