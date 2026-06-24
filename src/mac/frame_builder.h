// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_FRAME_BUILDER_H
#define MAC_FRAME_BUILDER_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// Build a wire frame from the host's Ethernet frame (destination..payload, no
// FCS, as delivered over CDC-ECM): zero-pad short frames to the 60-byte
// minimum, append the CRC-32 FCS, and prepend the preamble and SFD.
[[nodiscard]] constexpr std::expected<WireFrame, FrameError> build_frame(std::span<const std::uint8_t> host_frame) {
    if (host_frame.size() > MAX_FRAME_NO_FCS) {
        return std::unexpected(FrameError::TooLong);
    }

    const std::size_t frame_len{std::max(host_frame.size(), MIN_FRAME_NO_FCS)};

    WireFrame out{};
    std::size_t pos{0};

    for (std::size_t i{0}; i < PREAMBLE_LEN; ++i) {
        out.bytes[pos] = PREAMBLE_BYTE;
        ++pos;
    }
    out.bytes[pos] = SFD_BYTE;
    ++pos;

    const std::size_t frame_start{pos};
    std::ranges::copy(host_frame, out.bytes.begin() + frame_start);
    // Bytes from host_frame.size() to frame_len stay zero (padding).
    pos = frame_start + frame_len;

    const std::uint32_t fcs{crc32(std::span<const std::uint8_t>(out.bytes).subspan(frame_start, frame_len))};
    out.bytes[pos] = static_cast<std::uint8_t>(fcs);
    out.bytes[pos + 1] = static_cast<std::uint8_t>(fcs >> 8);
    out.bytes[pos + 2] = static_cast<std::uint8_t>(fcs >> 16);
    out.bytes[pos + 3] = static_cast<std::uint8_t>(fcs >> 24);
    pos += 4;

    out.length = pos;
    return out;
}

} // namespace pico_ethernet

#endif // MAC_FRAME_BUILDER_H
