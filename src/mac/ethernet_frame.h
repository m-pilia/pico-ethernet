// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_ETHERNET_FRAME_H
#define MAC_ETHERNET_FRAME_H

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/mac_address.h"

namespace pico_ethernet {

inline constexpr std::uint8_t PREAMBLE_BYTE{0x55};
inline constexpr std::uint8_t SFD_BYTE{0xD5};
inline constexpr std::size_t PREAMBLE_LEN{7};
inline constexpr std::size_t SFD_LEN{1};
inline constexpr std::size_t PREAMBLE_SFD_LEN{PREAMBLE_LEN + SFD_LEN}; // 8

// Sizes exclude preamble/SFD, which are framing bytes owned by the MAC and not
// counted as part of the frame. "Frame" here is destination..payload[..FCS].
inline constexpr std::size_t MAC_HEADER_LEN{14}; // DA(6) + SA(6) + EtherType(2)
inline constexpr std::size_t FCS_LEN{4};
inline constexpr std::size_t MIN_PAYLOAD{46};
inline constexpr std::size_t MAX_PAYLOAD{1500};
inline constexpr std::size_t MIN_FRAME_NO_FCS{MAC_HEADER_LEN + MIN_PAYLOAD};
inline constexpr std::size_t MAX_FRAME_NO_FCS{MAC_HEADER_LEN + MAX_PAYLOAD};
inline constexpr std::size_t MIN_FRAME_WITH_FCS{MIN_FRAME_NO_FCS + FCS_LEN};
inline constexpr std::size_t MAX_FRAME_WITH_FCS{MAX_FRAME_NO_FCS + FCS_LEN};
inline constexpr std::size_t WIRE_CAPACITY{PREAMBLE_SFD_LEN + MAX_FRAME_WITH_FCS};

enum class FrameError : std::uint8_t {
    TooLong,     // TX: host frame exceeds MAX_FRAME_NO_FCS
    BadPreamble, // RX: no valid preamble/SFD prefix
    Runt,        // RX: shorter than MIN_FRAME_WITH_FCS
    Giant,       // RX: longer than MAX_FRAME_WITH_FCS
    BadFcs,      // RX: FCS does not match the frame contents
    Filtered,    // RX: destination address rejected by the filter
};

// The canonical in-memory representation: a full wire frame
// (preamble + SFD + destination..payload + FCS, up to WIRE_CAPACITY bytes).
// The host-facing frame is the destination..payload sub-span (no preamble/FCS),
// so a TX build and the subsequent RX parse round-trip the exact same bytes.
struct WireFrame {
    std::array<std::uint8_t, WIRE_CAPACITY> bytes{};
    std::size_t length{0};

    constexpr std::span<const std::uint8_t> view() const { return std::span<const std::uint8_t>(bytes.data(), length); }
};

// The accessors below take a frame starting at the destination address (no
// preamble), i.e. destination..payload[..FCS]. The caller must have length-
// validated the frame to at least MAC_HEADER_LEN bytes; that is a parser
// invariant, not external input, so the precondition is an assert.

constexpr MacAddress destination_mac(std::span<const std::uint8_t> frame) {
    assert(frame.size() >= MAC_HEADER_LEN);
    return MacAddress(frame.subspan<0, MacAddress::LENGTH>());
}

constexpr MacAddress source_mac(std::span<const std::uint8_t> frame) {
    assert(frame.size() >= MAC_HEADER_LEN);
    return MacAddress(frame.subspan<MacAddress::LENGTH, MacAddress::LENGTH>());
}

constexpr std::uint16_t ethertype(std::span<const std::uint8_t> frame) {
    assert(frame.size() >= MAC_HEADER_LEN);
    return static_cast<std::uint16_t>((frame[12] << 8) | frame[13]);
}

} // namespace pico_ethernet

#endif // MAC_ETHERNET_FRAME_H
