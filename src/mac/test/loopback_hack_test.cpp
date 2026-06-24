// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// TEMPORARY loopback hack unit tests (remove in Phase 3).

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/loopback_hack.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {

TEST(LoopbackHackTest, SwapsDestinationAndSource) {
    std::array<std::uint8_t, MAC_HEADER_LEN> frame{
        0x02, 0x00, 0x00, 0x00, 0x00, 0x01,  // destination
        0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF,  // source
        0x08, 0x00,                          // EtherType
    };

    loopback_swap_addresses(frame);

    EXPECT_EQ(destination_mac(frame),
              MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(
                  MacAddress::Bytes{0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF})));
    EXPECT_EQ(source_mac(frame),
              MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(
                  MacAddress::Bytes{0x02, 0x00, 0x00, 0x00, 0x00, 0x01})));
    // EtherType is untouched.
    EXPECT_EQ(ethertype(frame), 0x0800);
}

} // namespace pico_ethernet
