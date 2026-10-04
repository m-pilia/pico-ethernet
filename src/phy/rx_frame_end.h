// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_FRAME_END_H
#define PHY_RX_FRAME_END_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <expected>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// The RX SM lands a capture as whole words of four octets, from the first
// destination octet onwards. A frame that ends on TP_IDL is followed by 1 to
// RX_OCTETS_PER_WORD zero octets: the SM pads its last word, and pushes a whole
// zero word when the frame already ended on a word boundary.
inline constexpr std::size_t RX_OCTETS_PER_WORD{4};

namespace detail {

// The running CRC after a frame, its valid FCS and then k zero pad octets, at index
// k - 1.
inline constexpr std::array<std::uint32_t, RX_OCTETS_PER_WORD> PADDED_RESIDUALS{[] {
    std::array<std::uint32_t, RX_OCTETS_PER_WORD> residuals{};
    std::uint32_t crc{CRC32_RESIDUAL_RAW};
    for (std::uint32_t& residual : residuals) {
        crc = crc32_update(crc, 0);
        residual = crc;
    }
    return residuals;
}()};

// Distinct, so the running CRC of a capture matches at most one pad.
static_assert([] {
    std::array<std::uint32_t, RX_OCTETS_PER_WORD> sorted{PADDED_RESIDUALS};
    std::ranges::sort(sorted);
    return std::ranges::adjacent_find(sorted) == sorted.end();
}());

} // namespace detail

// Locates the end of the frame in a capture of `capture_octets` octets, given the
// running CRC over all of them, and returns the frame's length (destination..FCS).
// A capture that matches no pad is a Giant once it is longer than any frame (only a
// capture that filled its buffer is), a Runt when shorter than any.
[[nodiscard]] constexpr std::expected<std::size_t, FrameError>
find_frame_end(std::size_t capture_octets, std::uint32_t crc) {
    assert(capture_octets > 0 && capture_octets % RX_OCTETS_PER_WORD == 0);

    for (std::size_t pad{1}; pad <= RX_OCTETS_PER_WORD; ++pad) {
        if (crc != detail::PADDED_RESIDUALS[pad - 1]) {
            continue;
        }
        const std::size_t length{capture_octets - pad};
        if (length < MIN_FRAME_WITH_FCS) {
            return std::unexpected(FrameError::Runt);
        }
        if (length > MAX_FRAME_WITH_FCS) {
            return std::unexpected(FrameError::Giant);
        }
        return length;
    }
    if (capture_octets > MAX_FRAME_WITH_FCS) {
        return std::unexpected(FrameError::Giant);
    }
    if (capture_octets < MIN_FRAME_WITH_FCS) {
        return std::unexpected(FrameError::Runt);
    }
    return std::unexpected(FrameError::BadFcs);
}

} // namespace pico_ethernet

#endif // PHY_RX_FRAME_END_H
