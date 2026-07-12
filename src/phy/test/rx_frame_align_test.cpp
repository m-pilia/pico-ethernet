// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/rx_frame_align.h"

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
// would autopush: bit k lands in byte k/8 at position k%8.
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

TEST(RxFrameAlign, RecoversFrameBytesAtEveryBitOffset) {
    const auto built = build_frame(HOST_FRAME);
    ASSERT_TRUE(built.has_value());
    const std::span<const std::uint8_t> wire{built->view()};
    const std::span<const std::uint8_t> body{wire.subspan(PREAMBLE_SFD_LEN)}; // destination..FCS

    for (std::size_t offset{0}; offset < 8; ++offset) {
        const std::vector<std::uint8_t> raw{pack_bits(stream_at_offset(wire, offset))};
        std::array<std::uint8_t, WIRE_CAPACITY> out{};
        const auto len{align_to_sfd(raw, out)};

        ASSERT_TRUE(len.has_value()) << "offset " << offset;
        ASSERT_GE(*len, body.size() + 1) << "offset " << offset;
        EXPECT_EQ(out[0], SFD_BYTE) << "offset " << offset;
        for (std::size_t i{0}; i < body.size(); ++i) {
            EXPECT_EQ(out[i + 1], body[i]) << "offset " << offset << " byte " << i;
        }
    }
}

TEST(RxFrameAlign, NoSfdReturnsNullopt) {
    std::array<std::uint8_t, 16> raw{};
    raw.fill(PREAMBLE_BYTE); // pure preamble, never contains the SFD delimiter
    std::array<std::uint8_t, 32> out{};
    EXPECT_FALSE(align_to_sfd(raw, out).has_value());
}

TEST(RxFrameAlign, EmptyInputReturnsNullopt) {
    std::array<std::uint8_t, 8> out{};
    EXPECT_FALSE(align_to_sfd(std::span<const std::uint8_t>{}, out).has_value());
}

} // namespace
} // namespace pico_ethernet
