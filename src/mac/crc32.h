// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_CRC32_H
#define MAC_CRC32_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

namespace pico_ethernet {

// IEEE 802.3 FCS: reflected CRC-32 (polynomial 0xEDB88320), initial value all
// ones, final inversion. The 256-entry lookup table is built at compile time.
//
// A target-only optimization could replace this with the RP2350 DMA CRC sniffer
// (DMA_SNIFF_CRC32 with output bit-reverse + invert), which matches this
// reflected/inverted convention and computes the FCS for free alongside a DMA
// copy of the frame. The software table is kept for host tests and portability.
namespace detail {

constexpr std::array<std::uint32_t, 256> make_crc32_table() {
    constexpr std::uint32_t POLYNOMIAL{0xEDB88320u};
    std::array<std::uint32_t, 256> table{};
    for (std::size_t i{0}; i < table.size(); ++i) {
        std::uint32_t crc{static_cast<std::uint32_t>(i)};
        for (std::size_t bit{0}; bit < 8; ++bit) {
            crc = (crc & 1u) ? (POLYNOMIAL ^ (crc >> 1)) : (crc >> 1);
        }
        table[i] = crc;
    }
    return table;
}

inline constexpr std::array<std::uint32_t, 256> CRC32_TABLE{make_crc32_table()};

} // namespace detail

// Running crc32() over a frame followed by its own little-endian FCS yields this
// fixed residual, letting a receiver validate destination..FCS in a single pass.
inline constexpr std::uint32_t CRC32_RESIDUAL{0x2144DF1Cu};

[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> data) {
    std::uint32_t crc{0xFFFFFFFFu};
    for (const std::uint8_t byte : data) {
        crc = detail::CRC32_TABLE[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
    }
    return crc ^ 0xFFFFFFFFu;
}

} // namespace pico_ethernet

#endif // MAC_CRC32_H
