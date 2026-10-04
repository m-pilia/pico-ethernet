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

inline constexpr std::uint32_t CRC32_INIT{0xFFFFFFFFu};

// One octet step of the running (pre-final-inversion) CRC.
[[nodiscard]] constexpr std::uint32_t crc32_update(std::uint32_t crc, std::uint8_t byte) {
    return detail::CRC32_TABLE[(crc ^ byte) & 0xFFu] ^ (crc >> 8);
}

[[nodiscard]] constexpr std::uint32_t crc32(std::span<const std::uint8_t> data) {
    std::uint32_t crc{CRC32_INIT};
    for (const std::uint8_t byte : data) {
        crc = crc32_update(crc, byte);
    }
    return crc ^ 0xFFFFFFFFu;
}

// The running (pre-final-inversion) CRC reaches this exactly when the bytes
// consumed so far form a frame followed by its valid FCS -- i.e. it is the
// CRC32_RESIDUAL check moved before the final inversion, so it can be applied to a
// streaming CRC without recomputing per length.
inline constexpr std::uint32_t CRC32_RESIDUAL_RAW{CRC32_RESIDUAL ^ 0xFFFFFFFFu};

} // namespace pico_ethernet

#endif // MAC_CRC32_H
