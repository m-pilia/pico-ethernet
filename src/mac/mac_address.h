// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_MAC_ADDRESS_H
#define MAC_MAC_ADDRESS_H

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <string_view>

namespace pico_ethernet {

// An IEEE 802 MAC address (six octets).
//
// The default address 02:00:00:00:00:01 is locally administered (bit 1 of the
// first octet set) and unicast (bit 0 clear).
class MacAddress {
  public:
    static constexpr std::size_t LENGTH{6};
    using Bytes = std::array<std::uint8_t, LENGTH>;

    static constexpr Bytes DEFAULT_BYTES{0x02, 0x00, 0x00, 0x00, 0x00, 0x01};

    constexpr MacAddress() = default;

    // Construct from the first six octets of a buffer (e.g. an Ethernet frame
    // header, or a byte array). The fixed-extent span makes the length a
    // precondition, so the caller must have validated it (frames are
    // length-checked before parsing).
    explicit constexpr MacAddress(std::span<const std::uint8_t, LENGTH> bytes) {
        for (std::size_t i{0}; i < LENGTH; ++i) {
            address_[i] = bytes[i];
        }
    }

    static constexpr MacAddress generate_default() {
        return MacAddress(std::span<const std::uint8_t, LENGTH>(DEFAULT_BYTES));
    }

    // Parse "AABBCCDDEEFF" or "AA:BB:CC:DD:EE:FF" (also '-' or '.' separators).
    // Hex is case-insensitive; the separator must be consistent. Returns nullopt
    // on any malformed input.
    [[nodiscard]] static constexpr std::optional<MacAddress> parse(std::string_view str) {
        Bytes out{};

        if (str.size() == LENGTH * 2) { // no separators: 12 hex digits
            for (std::size_t i{0}; i < LENGTH; ++i) {
                if (!parse_octet(str.substr(i * 2, 2), out[i]))
                    return std::nullopt;
            }
            return MacAddress(out);
        }

        if (str.size() == LENGTH * 3 - 1) { // six octets joined by five separators
            const char sep{str[2]};
            if (sep != ':' && sep != '-' && sep != '.')
                return std::nullopt;
            for (std::size_t i{0}; i < LENGTH; ++i) {
                const std::size_t pos{i * 3};
                if (i + 1 < LENGTH && str[pos + 2] != sep)
                    return std::nullopt;
                if (!parse_octet(str.substr(pos, 2), out[i]))
                    return std::nullopt;
            }
            return MacAddress(out);
        }

        return std::nullopt;
    }

    constexpr const Bytes& bytes() const { return address_; }
    constexpr std::uint8_t byte(std::size_t index) const {
        assert(index < LENGTH);
        return address_[index];
    }

    // Human-readable form for debug/logging, e.g. "02:00:00:00:00:01".
    // Null-terminated and returned by value (no heap), so it is safe on target.
    constexpr std::array<char, 18> to_string(char separator = ':') const {
        std::array<char, 18> out{};
        for (std::size_t i{0}; i < LENGTH; ++i) {
            out[i * 3] = hex_char(address_[i] >> 4);
            out[i * 3 + 1] = hex_char(address_[i]);
            out[i * 3 + 2] = (i + 1 < LENGTH) ? separator : '\0';
        }
        return out;
    }

    // CDC iMACAddress descriptor string: 12 uppercase hex digits, no separators.
    constexpr std::array<char, LENGTH * 2> to_imac_string() const {
        std::array<char, LENGTH * 2> out{};
        for (std::size_t i{0}; i < LENGTH; ++i) {
            out[i * 2] = hex_char(address_[i] >> 4);
            out[i * 2 + 1] = hex_char(address_[i]);
        }
        return out;
    }

    // A group address (multicast or broadcast): bit 0 of the first octet set.
    // Used by frame filtering (accept our address or any group address).
    constexpr bool is_multicast() const { return (address_[0] & 0x01) != 0; }

    // The broadcast address (all ones), a special case of a group address.
    constexpr bool is_broadcast() const {
        for (const std::uint8_t octet : address_) {
            if (octet != 0xFF)
                return false;
        }
        return true;
    }

    constexpr bool operator==(const MacAddress&) const = default;

    constexpr Bytes::const_iterator begin() const { return address_.begin(); }
    constexpr Bytes::const_iterator end() const { return address_.end(); }
    constexpr std::size_t size() const { return address_.size(); }

  private:
    // Low nibble of `value` as an uppercase hex digit.
    static constexpr char hex_char(std::uint8_t value) { return "0123456789ABCDEF"[value & 0x0F]; }

    // Parse exactly two hex digits into `out`; false if not two hex digits.
    static constexpr bool parse_octet(std::string_view pair, std::uint8_t& out) {
        assert(pair.size() == 2);
        const std::optional<std::uint8_t> hi{hex_digit(pair[0])};
        const std::optional<std::uint8_t> lo{hex_digit(pair[1])};
        if (!hi || !lo)
            return false;
        out = static_cast<std::uint8_t>((*hi << 4) | *lo);
        return true;
    }

    static constexpr std::optional<std::uint8_t> hex_digit(char c) {
        if (c >= '0' && c <= '9')
            return static_cast<std::uint8_t>(c - '0');
        if (c >= 'a' && c <= 'f')
            return static_cast<std::uint8_t>(c - 'a' + 10);
        if (c >= 'A' && c <= 'F')
            return static_cast<std::uint8_t>(c - 'A' + 10);
        return std::nullopt;
    }

    Bytes address_{};
};

} // namespace pico_ethernet

#endif // MAC_MAC_ADDRESS_H
