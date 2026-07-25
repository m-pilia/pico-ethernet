// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_TEST_FRAME_PARSER_H
#define MAC_TEST_FRAME_PARSER_H

#include <cstddef>
#include <cstdint>
#include <expected>
#include <optional>
#include <span>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_filter.h"

namespace pico_ethernet {

// Length of the shortest prefix of `data` (at least `min_len` bytes) whose last
// four bytes are a valid FCS for the bytes before them, or nullopt if none. The
// carrier-gated receiver captures past the FCS into trailing line noise, so this
// recovers the true frame boundary in one O(n) pass instead of assuming the whole
// capture is the frame.
[[nodiscard]] constexpr std::optional<std::size_t>
fcs_frame_length(std::span<const std::uint8_t> data, std::size_t min_len) {
    std::uint32_t crc{0xFFFFFFFFu};
    for (std::size_t i{0}; i < data.size(); ++i) {
        crc = detail::CRC32_TABLE[(crc ^ data[i]) & 0xFFu] ^ (crc >> 8);
        if (i + 1 >= min_len && crc == CRC32_RESIDUAL_RAW) {
            return i + 1;
        }
    }
    return std::nullopt;
}

// Parse a received wire frame (preamble + SFD + destination..payload + FCS) into
// the host-facing frame (destination..payload, no FCS) to deliver over CDC-ECM.
// The returned span aliases `wire_frame`, which must outlive its use. All
// failures are data errors (malformed/unwanted frames), reported via FrameError.
[[nodiscard]] constexpr std::expected<std::span<const std::uint8_t>, FrameError>
parse_frame(std::span<const std::uint8_t> wire_frame, const FrameFilter& filter) {
    std::size_t pos{0};
    while (pos < wire_frame.size() && wire_frame[pos] == PREAMBLE_BYTE)
        ++pos;
    if (pos == wire_frame.size() || wire_frame[pos] != SFD_BYTE) {
        return std::unexpected(FrameError::BadPreamble);
    }
    const std::span<const std::uint8_t> tail{wire_frame.subspan(pos + 1)};
    if (tail.size() < MIN_FRAME_WITH_FCS)
        return std::unexpected(FrameError::Runt);

    // The decoder is carrier-gated and keeps running past the FCS into trailing
    // line noise until it stalls, so `tail` is the frame plus trailing junk. Find
    // the true FCS boundary rather than assuming the whole capture is the frame.
    const std::size_t scan_len{tail.size() < MAX_FRAME_WITH_FCS ? tail.size() : MAX_FRAME_WITH_FCS};
    const std::optional<std::size_t> frame_len{fcs_frame_length(tail.first(scan_len), MIN_FRAME_WITH_FCS)};
    if (!frame_len) {
        return std::unexpected(tail.size() > MAX_FRAME_WITH_FCS ? FrameError::Giant : FrameError::BadFcs);
    }
    const std::span<const std::uint8_t> frame{tail.first(*frame_len)};

    if (!filter.accept(destination_mac(frame))) {
        return std::unexpected(FrameError::Filtered);
    }

    return frame.first(*frame_len - FCS_LEN);
}

} // namespace pico_ethernet

#endif // MAC_TEST_FRAME_PARSER_H
