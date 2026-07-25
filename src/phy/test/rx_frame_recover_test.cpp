// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/rx_frame_recover.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

#include <gtest/gtest.h>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"

namespace pico_ethernet {
namespace {

constexpr std::array<std::uint8_t, MAC_HEADER_LEN + 8> HOST_FRAME{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // destination (broadcast)
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01, // source
    0x08, 0x00,                         // EtherType (IPv4)
    0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67};

// Packs a chronological (LSB-first) bit list into the octet stream the RX decoder
// lands in memory: bit k lands in byte k/8 at position k%8.
std::vector<std::uint8_t> pack_bits(const std::vector<std::uint8_t>& bits) {
    std::vector<std::uint8_t> out((bits.size() + 7) / 8, 0);
    for (std::size_t k{0}; k < bits.size(); ++k) {
        out[k / 8] |= static_cast<std::uint8_t>(bits[k] << (k % 8));
    }
    return out;
}

void push_byte_lsb_first(std::vector<std::uint8_t>& bits, std::uint8_t byte) {
    for (std::size_t p{0}; p < 8; ++p) {
        bits.push_back(static_cast<std::uint8_t>((byte >> p) & 1u));
    }
}

// Builds the decoder's bit stream for a given carrier-gate bit offset: `offset`
// leading preamble bits (alternating, ending in 0 so they flow into the frame's
// first preamble bit without forging a false SFD), the whole wire frame, then a
// trailing octet so the final data octet is complete (as it is on the wire, where
// the line keeps toggling past the FCS before carrier drops).
std::vector<std::uint8_t> stream_at_offset(std::span<const std::uint8_t> wire, std::size_t offset) {
    std::vector<std::uint8_t> bits;
    for (std::size_t b{0}; b < offset; ++b) {
        bits.push_back(static_cast<std::uint8_t>(((offset - 1 - b) % 2) == 1 ? 1 : 0));
    }
    for (const std::uint8_t byte : wire) {
        push_byte_lsb_first(bits, byte);
    }
    push_byte_lsb_first(bits, PREAMBLE_BYTE);
    return bits;
}

TEST(RxFrameRecover, RecoversFrameAtEveryBitOffset) {
    const auto built = build_frame(HOST_FRAME);
    ASSERT_TRUE(built.has_value());
    const std::span<const std::uint8_t> wire{built->view()};
    const std::span<const std::uint8_t> body{wire.subspan(PREAMBLE_SFD_LEN)}; // destination..FCS

    for (std::size_t offset{0}; offset < 8; ++offset) {
        const std::vector<std::uint8_t> raw{pack_bits(stream_at_offset(wire, offset))};
        std::array<std::uint8_t, WIRE_CAPACITY> out{};
        const auto len{recover_frame(raw, out)};

        ASSERT_TRUE(len.has_value()) << "offset " << offset;
        EXPECT_EQ(*len, body.size()) << "offset " << offset;
        for (std::size_t i{0}; i < body.size(); ++i) {
            EXPECT_EQ(out[i], body[i]) << "offset " << offset << " byte " << i;
        }
    }
}

// The single pass must stop at the FCS residual and ignore whatever the
// carrier-gated capture ran on into after the frame.
TEST(RxFrameRecover, DelimitsFrameIgnoringTrailingJunk) {
    const auto built = build_frame(HOST_FRAME);
    ASSERT_TRUE(built.has_value());
    const std::span<const std::uint8_t> wire{built->view()};
    const std::span<const std::uint8_t> body{wire.subspan(PREAMBLE_SFD_LEN)};

    std::vector<std::uint8_t> bits;
    for (const std::uint8_t byte : wire) {
        push_byte_lsb_first(bits, byte);
    }
    for (const std::uint8_t junk : {0x37, 0x9C, 0xE1, 0x4B, 0xA5}) {
        push_byte_lsb_first(bits, junk);
    }
    const std::vector<std::uint8_t> raw{pack_bits(bits)};

    std::array<std::uint8_t, WIRE_CAPACITY> out{};
    const auto len{recover_frame(raw, out)};
    ASSERT_TRUE(len.has_value());
    EXPECT_EQ(*len, body.size());
    for (std::size_t i{0}; i < body.size(); ++i) {
        EXPECT_EQ(out[i], body[i]) << "byte " << i;
    }
}

TEST(RxFrameRecover, CorruptedFrameFailsFcs) {
    const auto built = build_frame(HOST_FRAME);
    ASSERT_TRUE(built.has_value());
    const std::span<const std::uint8_t> wire{built->view()};

    std::array<std::uint8_t, WIRE_CAPACITY> wire_bytes{};
    std::copy(wire.begin(), wire.end(), wire_bytes.begin());
    wire_bytes[PREAMBLE_SFD_LEN + 20] ^= 0xFF; // flip a payload byte

    std::vector<std::uint8_t> bits;
    for (std::size_t i{0}; i < wire.size(); ++i) {
        push_byte_lsb_first(bits, wire_bytes[i]);
    }
    const std::vector<std::uint8_t> raw{pack_bits(bits)};

    std::array<std::uint8_t, WIRE_CAPACITY> out{};
    const auto len{recover_frame(raw, out)};
    ASSERT_FALSE(len.has_value());
    EXPECT_EQ(len.error(), FrameError::BadFcs);
}

TEST(RxFrameRecover, TooFewOctetsAfterSfdIsRunt) {
    std::vector<std::uint8_t> bits;
    push_byte_lsb_first(bits, PREAMBLE_BYTE);
    push_byte_lsb_first(bits, SFD_BYTE);
    for (std::size_t i{0}; i < MIN_FRAME_WITH_FCS - 1; ++i) {
        push_byte_lsb_first(bits, static_cast<std::uint8_t>(i));
    }
    const std::vector<std::uint8_t> raw{pack_bits(bits)};

    std::array<std::uint8_t, WIRE_CAPACITY> out{};
    const auto len{recover_frame(raw, out)};
    ASSERT_FALSE(len.has_value());
    EXPECT_EQ(len.error(), FrameError::Runt);
}

TEST(RxFrameRecover, NoSfdIsBadPreamble) {
    std::array<std::uint8_t, 64> raw{};
    raw.fill(PREAMBLE_BYTE); // pure preamble, never contains the SFD delimiter
    std::array<std::uint8_t, WIRE_CAPACITY> out{};
    const auto len{recover_frame(raw, out)};
    ASSERT_FALSE(len.has_value());
    EXPECT_EQ(len.error(), FrameError::BadPreamble);
}

TEST(RxFrameRecover, EmptyInputIsBadPreamble) {
    std::array<std::uint8_t, WIRE_CAPACITY> out{};
    const auto len{recover_frame(std::span<const std::uint8_t>{}, out)};
    ASSERT_FALSE(len.has_value());
    EXPECT_EQ(len.error(), FrameError::BadPreamble);
}

} // namespace
} // namespace pico_ethernet
