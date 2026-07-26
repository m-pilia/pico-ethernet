// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_FRAME_RECOVER_H
#define PHY_RX_FRAME_RECOVER_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <expected>
#include <span>

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// Recover a byte-aligned, FCS-delimited frame from the raw octet stream the RX
// decoder autopushes, in a single pass over the captured bits.
//
// Two artefacts of the carrier-gated decoder have to be undone. First, it starts
// shifting bits at an arbitrary point in the preamble, so its octet boundaries are
// offset from the frame's true byte boundaries by 0..7 bits (a byte-level SFD
// search fails on 7 of 8 frames). Second, it keeps running past the FCS into
// trailing line noise until the carrier drops, so the capture is the frame plus
// junk and the true end must be found rather than assumed.
//
// The received bit stream is LSB-first, so the preamble (0x55) is the alternating
// pattern 1,0,1,0,... and the SFD (0xD5) is 1,0,1,0,1,0,1,1 -- the only 8-bit
// window closing with two 1s, and thus the unambiguous byte-alignment marker. This
// finds it at the bit level, then reassembles the following octets at the recovered
// bit offset while feeding each finished octet into the running FCS and writing it
// straight into `out`, stopping the instant the running CRC hits the end-of-frame
// residual -- alignment, frame-end delimiting and FCS validation in one walk.
//
// `out` receives the destination..FCS frame (no preamble/SFD); the returned length
// spans it including the FCS. Filtering and stripping the FCS are the caller's.
// Errors mirror the MAC parser: BadPreamble (no SFD -- idle noise or a partial
// capture), Runt (< MIN_FRAME_WITH_FCS octets after the SFD), Giant
// (MAX_FRAME_WITH_FCS octets reached with no valid FCS), BadFcs (capture exhausted
// with no valid FCS).
[[nodiscard]] constexpr std::expected<std::size_t, FrameError>
recover_frame(std::span<const std::uint8_t> raw, std::span<std::uint8_t> out) {
    assert(out.size() >= MAX_FRAME_WITH_FCS);

    const std::size_t total_bits{raw.size() * 8};
    const auto bit_at = [raw](std::size_t g) -> std::uint8_t {
        return static_cast<std::uint8_t>((raw[g / 8] >> (g % 8)) & 1u);
    };

    // SFD delimiter (LSB-first): 1,0,1,0,1,0,1,1.
    std::size_t data_start{total_bits}; // sentinel: not found
    for (std::size_t g{0}; g + 8 <= total_bits; ++g) {
        if (bit_at(g) == 1 && bit_at(g + 1) == 0 && bit_at(g + 2) == 1 && bit_at(g + 3) == 0 && bit_at(g + 4) == 1 &&
            bit_at(g + 5) == 0 && bit_at(g + 6) == 1 && bit_at(g + 7) == 1) {
            data_start = g + 8;
            break;
        }
    }
    if (data_start == total_bits) {
        return std::unexpected(FrameError::BadPreamble);
    }

    // The SFD gives a single bit offset for the whole frame, so realignment is a
    // fixed shift rather than a per-bit reassembly: each output byte is the low
    // (8 - bit_off) bits of one raw byte ORed with the high bit_off bits of the next
    // -- one shift-combine per byte instead of eight per-bit reads. Shift, feed the
    // running CRC, and write to `out` in the same pass, stopping at the end-of-frame
    // residual.
    const std::size_t byte_base{data_start / 8};
    const std::size_t bit_off{data_start % 8};
    // With a nonzero offset each output byte straddles two raw bytes, so the last one
    // needs a raw byte beyond it; with a zero offset it does not.
    const std::size_t last{bit_off == 0 ? raw.size() : raw.size() - 1};

    std::uint32_t crc{0xFFFFFFFFu};
    std::size_t len{0};
    for (std::size_t i{byte_base}; i < last; ++i) {
        const unsigned high{bit_off == 0 ? 0u : static_cast<unsigned>(raw[i + 1]) << (8 - bit_off)};
        const std::uint8_t byte{static_cast<std::uint8_t>((raw[i] >> bit_off) | high)};
        out[len] = byte;
        crc = detail::CRC32_TABLE[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
        ++len;
        if (len >= MIN_FRAME_WITH_FCS && crc == CRC32_RESIDUAL_RAW) {
            return len;
        }
        if (len >= MAX_FRAME_WITH_FCS) {
            return std::unexpected(FrameError::Giant);
        }
    }
    return std::unexpected(len < MIN_FRAME_WITH_FCS ? FrameError::Runt : FrameError::BadFcs);
}

} // namespace pico_ethernet

#endif // PHY_RX_FRAME_RECOVER_H
