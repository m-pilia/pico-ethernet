// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {

constexpr std::array<std::uint8_t, MAC_HEADER_LEN> HEADER{
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01,  // destination
    0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,  // source
    0x08, 0x00,                          // EtherType (IPv4)
};

constexpr MacAddress make_mac(MacAddress::Bytes bytes) {
    return MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(bytes));
}

static_assert(destination_mac(HEADER) == make_mac({0x02, 0, 0, 0, 0, 1}));
static_assert(source_mac(HEADER) == make_mac({0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}));
static_assert(ethertype(HEADER) == 0x0800);

TEST(EthernetFrameTest, ExtractsAddressesAndType) {
    EXPECT_EQ(destination_mac(HEADER), make_mac({0x02, 0, 0, 0, 0, 1}));
    EXPECT_EQ(source_mac(HEADER), make_mac({0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF}));
    EXPECT_EQ(ethertype(HEADER), 0x0800);
}

TEST(EthernetFrameTest, WireFrameViewReflectsLength) {
    WireFrame frame{};
    frame.bytes[0] = 0x42;
    frame.length = 1;
    EXPECT_EQ(frame.view().size(), 1u);
    EXPECT_EQ(frame.view()[0], 0x42);
}

} // namespace pico_ethernet
