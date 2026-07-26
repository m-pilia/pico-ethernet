// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_FRAME_FILTER_H
#define MAC_FRAME_FILTER_H

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/mac_address.h"

namespace pico_ethernet {

// Decides whether a received frame's destination address is accepted, mirroring
// a real NIC's behavior driven by the host. The packet-filter bitmap comes from
// the host's SetEthernetPacketFilter request; the multicast list from
// SetEthernetMulticastFilters. Until the host configures it, nothing is
// accepted.
class FrameFilter {
public:
    static constexpr std::size_t MAX_MULTICAST{16};

    // Ethernet Packet Filter Bitmap (CDC ECM 1.2, Table 8; reused by NCM).
    static constexpr std::uint16_t PROMISCUOUS{0x0001};
    static constexpr std::uint16_t ALL_MULTICAST{0x0002};
    static constexpr std::uint16_t DIRECTED{0x0004};
    static constexpr std::uint16_t BROADCAST{0x0008};
    static constexpr std::uint16_t MULTICAST{0x0010};

    explicit constexpr FrameFilter(const MacAddress& our_address)
        : our_address_{our_address} {}

    constexpr void set_packet_filter(std::uint16_t bitmap) { packet_filter_ = bitmap; }
    constexpr std::uint16_t packet_filter() const { return packet_filter_; }

    // Replace the subscribed multicast list. Addresses beyond MAX_MULTICAST are
    // dropped; the host learns our capacity from the Ethernet Networking functional descriptor's
    // wNumberMCFilters and is expected to fall back to ALL_MULTICAST rather than
    // overflow, so truncation should not occur in practice.
    constexpr void set_multicast_list(std::span<const MacAddress> addresses) {
        multicast_count_ = std::min(addresses.size(), MAX_MULTICAST);
        for (std::size_t i{0}; i < multicast_count_; ++i) {
            multicast_[i] = addresses[i];
        }
    }

    [[nodiscard]] constexpr bool accept(const MacAddress& dest) const {
        if ((packet_filter_ & PROMISCUOUS) != 0) return true;
        if (dest.is_broadcast()) return (packet_filter_ & BROADCAST) != 0;
        if (dest.is_multicast()) {
            if ((packet_filter_ & ALL_MULTICAST) != 0) return true;
            return (packet_filter_ & MULTICAST) != 0 && is_subscribed(dest);
        }
        return (packet_filter_ & DIRECTED) != 0 && dest == our_address_;
    }

private:
    constexpr bool is_subscribed(const MacAddress& dest) const {
        for (std::size_t i{0}; i < multicast_count_; ++i) {
            if (multicast_[i] == dest) return true;
        }
        return false;
    }

    MacAddress our_address_;
    std::uint16_t packet_filter_{0};
    std::array<MacAddress, MAX_MULTICAST> multicast_{};
    std::size_t multicast_count_{0};
};

} // namespace pico_ethernet

#endif // MAC_FRAME_FILTER_H
