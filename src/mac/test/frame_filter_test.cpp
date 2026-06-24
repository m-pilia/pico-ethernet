// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

#include "src/mac/frame_filter.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {
namespace {

constexpr MacAddress make_mac(MacAddress::Bytes bytes) {
    return MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(bytes));
}

constexpr MacAddress OUR_MAC{make_mac({0x02, 0, 0, 0, 0, 0x01})};
constexpr MacAddress OTHER_UNICAST{make_mac({0x06, 0, 0, 0, 0, 0x09})};
constexpr MacAddress BROADCAST{make_mac({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF})};
constexpr MacAddress MCAST_A{make_mac({0x01, 0x00, 0x5E, 0x00, 0x00, 0x01})};
constexpr MacAddress MCAST_B{make_mac({0x01, 0x00, 0x5E, 0x00, 0x00, 0x02})};

constexpr FrameFilter with_filter(std::uint16_t bitmap) {
    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(bitmap);
    return filter;
}

} // namespace

// A freshly constructed filter (no host configuration) accepts nothing.
static_assert(!FrameFilter{OUR_MAC}.accept(OUR_MAC));
static_assert(!FrameFilter{OUR_MAC}.accept(BROADCAST));

TEST(FrameFilterTest, DefaultAcceptsNothing) {
    const FrameFilter filter{OUR_MAC};
    EXPECT_FALSE(filter.accept(OUR_MAC));
    EXPECT_FALSE(filter.accept(BROADCAST));
    EXPECT_FALSE(filter.accept(MCAST_A));
    EXPECT_FALSE(filter.accept(OTHER_UNICAST));
}

TEST(FrameFilterTest, PromiscuousAcceptsEverything) {
    const auto filter = with_filter(FrameFilter::PROMISCUOUS);
    EXPECT_TRUE(filter.accept(OUR_MAC));
    EXPECT_TRUE(filter.accept(OTHER_UNICAST));
    EXPECT_TRUE(filter.accept(BROADCAST));
    EXPECT_TRUE(filter.accept(MCAST_A));
}

TEST(FrameFilterTest, DirectedAcceptsOnlyOurUnicast) {
    const auto filter = with_filter(FrameFilter::DIRECTED);
    EXPECT_TRUE(filter.accept(OUR_MAC));
    EXPECT_FALSE(filter.accept(OTHER_UNICAST));
    EXPECT_FALSE(filter.accept(BROADCAST));
    EXPECT_FALSE(filter.accept(MCAST_A));
}

TEST(FrameFilterTest, BroadcastBitGatesBroadcastOnly) {
    const auto filter = with_filter(FrameFilter::BROADCAST);
    EXPECT_TRUE(filter.accept(BROADCAST));
    EXPECT_FALSE(filter.accept(OUR_MAC));
    EXPECT_FALSE(filter.accept(MCAST_A));
}

TEST(FrameFilterTest, AllMulticastAcceptsAnyGroupButNotUnicast) {
    const auto filter = with_filter(FrameFilter::ALL_MULTICAST);
    EXPECT_TRUE(filter.accept(MCAST_A));
    EXPECT_TRUE(filter.accept(MCAST_B));
    EXPECT_FALSE(filter.accept(OUR_MAC));
    EXPECT_FALSE(filter.accept(OTHER_UNICAST));
    // Broadcast is gated by its own bit, not ALL_MULTICAST.
    EXPECT_FALSE(filter.accept(BROADCAST));
}

TEST(FrameFilterTest, MulticastListAcceptsOnlySubscribedGroups) {
    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(FrameFilter::MULTICAST);
    const std::array<MacAddress, 1> list{MCAST_A};
    filter.set_multicast_list(list);

    EXPECT_TRUE(filter.accept(MCAST_A));
    EXPECT_FALSE(filter.accept(MCAST_B));
}

TEST(FrameFilterTest, MulticastBitWithoutListAcceptsNoGroup) {
    const auto filter = with_filter(FrameFilter::MULTICAST);
    EXPECT_FALSE(filter.accept(MCAST_A));
}

TEST(FrameFilterTest, CombinedBitsAccumulate) {
    const auto filter = with_filter(FrameFilter::DIRECTED | FrameFilter::BROADCAST);
    EXPECT_TRUE(filter.accept(OUR_MAC));
    EXPECT_TRUE(filter.accept(BROADCAST));
    EXPECT_FALSE(filter.accept(OTHER_UNICAST));
    EXPECT_FALSE(filter.accept(MCAST_A));
}

TEST(FrameFilterTest, MulticastListTruncatesAtCapacity) {
    FrameFilter filter{OUR_MAC};
    filter.set_packet_filter(FrameFilter::MULTICAST);

    std::array<MacAddress, FrameFilter::MAX_MULTICAST + 1> list{};
    for (std::size_t i{0}; i < list.size(); ++i) {
        list[i] = make_mac({0x01, 0x00, 0x5E, 0x00,
                            static_cast<std::uint8_t>(i >> 8),
                            static_cast<std::uint8_t>(i)});
    }
    filter.set_multicast_list(list);

    // The first MAX_MULTICAST entries are kept; the overflow entry is dropped.
    EXPECT_TRUE(filter.accept(list[0]));
    EXPECT_TRUE(filter.accept(list[FrameFilter::MAX_MULTICAST - 1]));
    EXPECT_FALSE(filter.accept(list[FrameFilter::MAX_MULTICAST]));
}

} // namespace pico_ethernet
