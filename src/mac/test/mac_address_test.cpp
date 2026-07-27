// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

#include "src/mac/mac_address.h"

namespace pico_ethernet {

// Build a MacAddress from six octets. The public API takes a span (addresses are
// extracted from frame buffers); this keeps the test literals readable.
constexpr MacAddress make_mac(MacAddress::Bytes bytes) {
    return MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(bytes));
}

// The whole type is usable in constant expressions; verify the key operations
// actually evaluate at compile time.
static_assert(MacAddress::generate_default() == make_mac({0x02, 0, 0, 0, 0, 1}));
static_assert(MacAddress::parse("02:00:00:00:00:01") == MacAddress::generate_default());
static_assert(MacAddress::parse("020000000001") == MacAddress::generate_default());
static_assert(!MacAddress::parse("nope").has_value());
static_assert(make_mac({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}).is_multicast());
static_assert(make_mac({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}).is_broadcast());
static_assert(!make_mac({0x01, 0x00, 0x5E, 0x00, 0x00, 0x01}).is_broadcast());
static_assert(!MacAddress::generate_default().is_multicast());
static_assert(!MacAddress::generate_default().is_broadcast());
static_assert(MacAddress::generate_default().to_imac_string()[0] == '0');
static_assert(MacAddress::generate_default().to_string()[2] == ':');

class MacAddressTest : public ::testing::Test {
  protected:
    void SetUp() override {}
    void TearDown() override {}
};

TEST_F(MacAddressTest, default_generation) {
    const auto mac = MacAddress::generate_default();

    EXPECT_EQ(mac.bytes(), MacAddress::DEFAULT_PHASE1);
}

TEST_F(MacAddressTest, default_constructed_is_all_zero) {
    const MacAddress mac{};

    EXPECT_EQ(mac.bytes(), MacAddress::Bytes{}); // 00:00:00:00:00:00
    EXPECT_NE(mac, MacAddress::generate_default());
    EXPECT_FALSE(mac.is_multicast());
}

TEST_F(MacAddressTest, default_properties) {
    const auto mac = MacAddress::generate_default();

    // 02:... is a unicast (individual) address: the group bit is clear.
    EXPECT_FALSE(mac.is_multicast());
}

TEST_F(MacAddressTest, byte_access) {
    const auto mac = MacAddress::generate_default();

    EXPECT_EQ(mac.byte(0), 0x02);
    EXPECT_EQ(mac.byte(1), 0x00);
    EXPECT_EQ(mac.byte(2), 0x00);
    EXPECT_EQ(mac.byte(3), 0x00);
    EXPECT_EQ(mac.byte(4), 0x00);
    EXPECT_EQ(mac.byte(5), 0x01);
}

#ifndef NDEBUG
TEST_F(MacAddressTest, byte_out_of_range_asserts) {
    const auto mac = MacAddress::generate_default();

    EXPECT_DEATH((void)mac.byte(MacAddress::LENGTH), "");
}
#endif

TEST_F(MacAddressTest, construction_from_span) {
    // Extract a MAC from the first six octets of a larger buffer.
    const std::array<std::uint8_t, 8> frame = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01, 0xAA, 0xBB};
    const auto mac = MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(frame.data(), MacAddress::LENGTH));

    EXPECT_EQ(mac, MacAddress::generate_default());
}

TEST_F(MacAddressTest, to_string) {
    const auto mac = MacAddress::generate_default();

    EXPECT_EQ(std::string_view(mac.to_string().data()), "02:00:00:00:00:01");
    EXPECT_EQ(std::string_view(mac.to_string('-').data()), "02-00-00-00-00-01");
    EXPECT_EQ(std::string_view(mac.to_string('.').data()), "02.00.00.00.00.01");
}

TEST_F(MacAddressTest, to_string_uppercase_hex) {
    // Exercises the A-F output path (every nibble printed in uppercase).
    const auto mac = make_mac({0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45});

    EXPECT_EQ(std::string_view(mac.to_string().data()), "AB:CD:EF:01:23:45");
}

TEST_F(MacAddressTest, to_imac_string) {
    const auto mac = MacAddress::generate_default();
    const auto imac = mac.to_imac_string();

    EXPECT_EQ(std::string(imac.begin(), imac.end()), "020000000001");
}

TEST_F(MacAddressTest, to_imac_string_uppercase_hex) {
    const auto mac = make_mac({0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45});
    const auto imac = mac.to_imac_string();

    // 12 chars, uppercase, no separators.
    EXPECT_EQ(std::string(imac.begin(), imac.end()), "ABCDEF012345");
}

TEST_F(MacAddressTest, parse_string) {
    auto mac = MacAddress::parse("02:00:00:00:00:01");
    ASSERT_TRUE(mac.has_value());
    EXPECT_EQ(*mac, MacAddress::generate_default());

    mac = MacAddress::parse("02-00-00-00-00-01");
    ASSERT_TRUE(mac.has_value());
    EXPECT_EQ(*mac, MacAddress::generate_default());

    mac = MacAddress::parse("02.00.00.00.00.01");
    ASSERT_TRUE(mac.has_value());
    EXPECT_EQ(*mac, MacAddress::generate_default());

    mac = MacAddress::parse("020000000001");
    ASSERT_TRUE(mac.has_value());
    EXPECT_EQ(*mac, MacAddress::generate_default());
}

TEST_F(MacAddressTest, parse_case_insensitive) {
    // Uses a-f / A-F digits so case actually matters. Lower, upper, and mixed
    // case (separated and not) must all yield the same address.
    const auto expected = make_mac({0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45});

    for (const std::string_view str :
         {"ab:cd:ef:01:23:45", "AB:CD:EF:01:23:45", "aB:Cd:eF:01:23:45", "abcdef012345", "ABCDEF012345"}) {
        const auto mac = MacAddress::parse(str);
        ASSERT_TRUE(mac.has_value()) << str;
        EXPECT_EQ(*mac, expected) << str;
    }
}

TEST_F(MacAddressTest, parse_invalid) {
    EXPECT_FALSE(MacAddress::parse("").has_value());                     // empty
    EXPECT_FALSE(MacAddress::parse("02:00:00").has_value());             // too short
    EXPECT_FALSE(MacAddress::parse("02:00:00:00:00:01:02").has_value()); // too long
    EXPECT_FALSE(MacAddress::parse("02:0:00:00:00:01").has_value());     // ragged group
    EXPECT_FALSE(MacAddress::parse("GG:00:00:00:00:01").has_value());    // bad hex (sep form)
    EXPECT_FALSE(MacAddress::parse("0200000000GG").has_value());         // bad hex (12-char form)
}

TEST_F(MacAddressTest, parse_rejects_inconsistent_separator) {
    // Right length, but the separator is not used uniformly.
    EXPECT_FALSE(MacAddress::parse("02:00:00-00:00:01").has_value());
}

TEST_F(MacAddressTest, parse_rejects_unknown_separator) {
    // Right length, but '/' is not an accepted separator.
    EXPECT_FALSE(MacAddress::parse("02/00/00/00/00/01").has_value());
}

TEST_F(MacAddressTest, equality_operators) {
    const auto mac1 = make_mac({0x02, 0x00, 0x00, 0x00, 0x00, 0x01});
    const auto mac2 = make_mac({0x02, 0x00, 0x00, 0x00, 0x00, 0x02});
    const auto mac3 = make_mac({0x02, 0x00, 0x00, 0x00, 0x00, 0x01});

    EXPECT_EQ(mac1, mac1);
    EXPECT_EQ(mac1, mac3);
    EXPECT_NE(mac1, mac2);
}

TEST_F(MacAddressTest, broadcast_is_group_address) {
    const auto broadcast = make_mac({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF});

    EXPECT_TRUE(broadcast.is_multicast());
    EXPECT_TRUE(broadcast.is_broadcast());
}

TEST_F(MacAddressTest, multicast_address) {
    const auto multicast = make_mac({0x01, 0x00, 0x5E, 0x00, 0x00, 0x01});

    EXPECT_TRUE(multicast.is_multicast());
    EXPECT_FALSE(multicast.is_broadcast());
}

TEST_F(MacAddressTest, unicast_is_neither_group_nor_broadcast) {
    const auto unicast = MacAddress::generate_default();

    EXPECT_FALSE(unicast.is_multicast());
    EXPECT_FALSE(unicast.is_broadcast());
}

TEST_F(MacAddressTest, almost_broadcast_is_not_broadcast) {
    // A single non-0xFF octet must disqualify the broadcast match.
    EXPECT_FALSE(make_mac({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE}).is_broadcast());
    EXPECT_FALSE(make_mac({0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}).is_broadcast());
}

TEST_F(MacAddressTest, iteration) {
    const auto mac = MacAddress::generate_default();

    EXPECT_EQ(mac.size(), 6);

    const std::uint8_t* begin{mac.begin()};
    const std::uint8_t* end{mac.end()};

    EXPECT_EQ(begin[0], 0x02);
    EXPECT_EQ(begin[1], 0x00);
    EXPECT_EQ(begin[2], 0x00);
    EXPECT_EQ(begin[3], 0x00);
    EXPECT_EQ(begin[4], 0x00);
    EXPECT_EQ(begin[5], 0x01);

    EXPECT_EQ(end - begin, 6);
}

} // namespace pico_ethernet
