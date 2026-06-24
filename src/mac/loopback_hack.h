// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// !!! TEMPORARY - PHASE 2 ONLY. REMOVE IN PHASE 3. !!!
//
// With no PHY yet, the loopback demo feeds the TX pipeline output straight back
// into the RX pipeline. Real outbound frames are addressed to a peer, not to the
// host, so to make the echo visible to the host stack we forge a reply by
// swapping the source and destination MAC addresses. This is a wire-level lie
// with no place in a real NIC; delete it once the PHY (Phase 3) provides a
// genuine receive path.

#ifndef MAC_LOOPBACK_HACK_H
#define MAC_LOOPBACK_HACK_H

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>
#include <utility>

#include "src/mac/ethernet_frame.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {

// Swap destination and source addresses in place so a parsed frame appears
// addressed back to the host. Requires a length-validated frame (>= header).
inline void loopback_swap_addresses(std::span<std::uint8_t> frame) {
    assert(frame.size() >= MAC_HEADER_LEN);
    for (std::size_t i{0}; i < MacAddress::LENGTH; ++i) {
        std::swap(frame[i], frame[i + MacAddress::LENGTH]);
    }
}

} // namespace pico_ethernet

#endif // MAC_LOOPBACK_HACK_H
