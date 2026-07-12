// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_FRAME_ALIGN_H
#define PHY_RX_FRAME_ALIGN_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// Recover byte alignment from the raw octet stream the RX decoder produces.
// Because the decoder is gated on carrier detect, it starts shifting bits at an
// arbitrary point in the preamble, so its autopush octet boundaries are offset
// from the frame's true byte boundaries by 0..7 bits and a byte-level SFD search
// fails on 7 of 8 frames.
//
// The received bit stream is LSB-first, so the preamble (0x55) is the alternating
// pattern 1,0,1,0,... and the SFD (0xD5) is 1,0,1,0,1,0,1,1 -- the only 8-bit
// window that closes with two 1s, and thus the unambiguous byte-alignment marker.
// This finds it at the bit level and rewrites `out` as a byte-aligned frame: a
// synthetic leading SFD byte followed by the destination..FCS octets reassembled
// at the recovered bit offset, ready for the byte-oriented MAC parser. Returns the
// aligned length, or std::nullopt if no SFD is present (idle noise / partial
// capture) or `out` is too small.
[[nodiscard]] constexpr std::optional<std::size_t>
align_to_sfd(std::span<const std::uint8_t> raw, std::span<std::uint8_t> out) {
    const std::size_t total_bits{raw.size() * 8};
    if (total_bits < 8 || out.empty()) {
        return std::nullopt;
    }

    const auto bit_at = [raw](std::size_t g) -> std::uint8_t {
        return static_cast<std::uint8_t>((raw[g / 8] >> (g % 8)) & 1u);
    };

    constexpr std::array<std::uint8_t, 8> SFD_BITS{1, 0, 1, 0, 1, 0, 1, 1};
    std::size_t sfd_start{total_bits}; // sentinel: not found
    for (std::size_t g{0}; g + 8 <= total_bits; ++g) {
        bool match{true};
        for (std::size_t b{0}; b < 8; ++b) {
            if (bit_at(g + b) != SFD_BITS[b]) {
                match = false;
                break;
            }
        }
        if (match) {
            sfd_start = g;
            break;
        }
    }
    if (sfd_start == total_bits) {
        return std::nullopt;
    }

    const std::size_t data_start{sfd_start + 8};
    const std::size_t data_bytes{(total_bits - data_start) / 8};
    if (out.size() < data_bytes + 1) {
        return std::nullopt;
    }

    out[0] = SFD_BYTE;
    for (std::size_t j{0}; j < data_bytes; ++j) {
        std::uint8_t byte{0};
        for (std::size_t b{0}; b < 8; ++b) {
            byte |= static_cast<std::uint8_t>(bit_at(data_start + j * 8 + b) << b);
        }
        out[j + 1] = byte;
    }
    return data_bytes + 1;
}

} // namespace pico_ethernet

#endif // PHY_RX_FRAME_ALIGN_H
