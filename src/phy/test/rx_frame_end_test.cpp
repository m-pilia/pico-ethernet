// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/rx_frame_end.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {
namespace {

// A capture of a `Length`-octet frame (destination..FCS) with a valid FCS, followed
// by `Pad` zero octets, as the RX SM lands it.
template <std::size_t Length, std::size_t Pad>
constexpr std::array<std::uint8_t, Length + Pad> padded_capture() {
    std::array<std::uint8_t, Length + Pad> capture{};
    for (std::size_t i{0}; i < Length - FCS_LEN; ++i) {
        capture[i] = static_cast<std::uint8_t>(i * 7 + 1);
    }
    const std::uint32_t fcs{crc32(std::span{capture}.first(Length - FCS_LEN))};
    for (std::size_t i{0}; i < FCS_LEN; ++i) {
        capture[Length - FCS_LEN + i] = static_cast<std::uint8_t>(fcs >> (8 * i));
    }
    return capture;
}

template <std::size_t Size>
constexpr std::array<std::uint8_t, Size> junk() {
    std::array<std::uint8_t, Size> capture{};
    for (std::size_t i{0}; i < Size; ++i) {
        capture[i] = static_cast<std::uint8_t>(i * 13 + 5);
    }
    return capture;
}

template <std::size_t Size>
constexpr std::array<std::uint8_t, Size>
flip(std::array<std::uint8_t, Size> capture, std::size_t index, std::uint8_t mask) {
    capture[index] ^= mask;
    return capture;
}

// The end check on `capture`, given the running CRC the sniffer reads after the
// capture DMA has landed it.
template <std::size_t Size>
constexpr std::expected<std::size_t, FrameError> frame_end(const std::array<std::uint8_t, Size>& capture) {
    return find_frame_end(capture.size(), std::ranges::fold_left(capture, CRC32_INIT, crc32_update));
}

constexpr std::size_t FRAME_LEN{100};

// Every pad the RX SM can add, and the longest deliverable frame.
static_assert(frame_end(padded_capture<MIN_FRAME_WITH_FCS, 4>()) == MIN_FRAME_WITH_FCS);
static_assert(frame_end(padded_capture<MIN_FRAME_WITH_FCS + 1, 3>()) == MIN_FRAME_WITH_FCS + 1);
static_assert(frame_end(padded_capture<MIN_FRAME_WITH_FCS + 2, 2>()) == MIN_FRAME_WITH_FCS + 2);
static_assert(frame_end(padded_capture<MIN_FRAME_WITH_FCS + 3, 1>()) == MIN_FRAME_WITH_FCS + 3);
static_assert(frame_end(padded_capture<MAX_FRAME_WITH_FCS, 2>()) == MAX_FRAME_WITH_FCS);

// A corrupted payload, and a corrupted FCS.
static_assert(
    frame_end(flip(padded_capture<FRAME_LEN, 4>(), MAC_HEADER_LEN, 0x01)) == std::unexpected(FrameError::BadFcs));
static_assert(
    frame_end(flip(padded_capture<FRAME_LEN, 4>(), FRAME_LEN - 1, 0x80)) == std::unexpected(FrameError::BadFcs));

// Frames with a valid FCS, but outside the frame length limits.
static_assert(frame_end(padded_capture<MIN_FRAME_WITH_FCS - 4, 4>()) == std::unexpected(FrameError::Runt));
static_assert(frame_end(padded_capture<MAX_FRAME_WITH_FCS + 1, 1>()) == std::unexpected(FrameError::Giant));

// Captures that match no pad.
static_assert(frame_end(junk<MAX_FRAME_WITH_FCS + 2>()) == std::unexpected(FrameError::Giant));
static_assert(frame_end(junk<MIN_FRAME_WITH_FCS - 4>()) == std::unexpected(FrameError::Runt));

} // namespace
} // namespace pico_ethernet
