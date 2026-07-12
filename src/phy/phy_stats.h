// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_STATS_H
#define PHY_PHY_STATS_H

#include <array>
#include <cstdint>
#include <optional>

#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// Transmit-side outcome counters.
struct TxStats {
    std::uint32_t accepted{0};     // frames handed to the PHY
    std::uint32_t build_failed{0}; // host frame too long to frame
    std::uint32_t dropped_busy{0}; // PHY still transmitting a prior frame
};

// Receive-side counters. The per-FrameError fields come out of the MAC parser
// unchanged; decode_error and carrier_glitch are PHY-level events with no MAC
// equivalent (a Manchester code violation, and carrier that never yielded a
// deliverable frame).
struct RxStats {
    std::uint32_t delivered{0};
    std::uint32_t bad_preamble{0};
    std::uint32_t runt{0};
    std::uint32_t giant{0};
    std::uint32_t bad_fcs{0};
    std::uint32_t filtered{0};
    std::uint32_t decode_error{0};
    std::uint32_t carrier_glitch{0};

    constexpr void record_error(FrameError error) {
        switch (error) {
            case FrameError::BadPreamble:
                ++bad_preamble;
                break;
            case FrameError::Runt:
                ++runt;
                break;
            case FrameError::Giant:
                ++giant;
                break;
            case FrameError::BadFcs:
                ++bad_fcs;
                break;
            case FrameError::Filtered:
                ++filtered;
                break;
            case FrameError::TooLong:
                break; // TX-only; never produced by the RX parse
        }
    }

    // Frames dropped because of a corruption/decode fault. Filtered frames are an
    // intentional drop (not addressed to us), not an error, so they are excluded.
    [[nodiscard]] constexpr std::uint32_t error_total() const {
        return bad_preamble + runt + giant + bad_fcs + decode_error + carrier_glitch;
    }
};

// CDC ECM feature selectors for GetEthernetStatistic; one 32-bit counter is
// returned per selector. Only the subset the device actually maintains is listed.
enum class EthernetStatistic : std::uint16_t {
    XmitOk = 0x01,
    RcvOk = 0x02,
    XmitError = 0x03,
    RcvError = 0x04,
    RcvCrcError = 0x12,
};

inline constexpr std::array<EthernetStatistic, 5> SUPPORTED_STATISTICS{
    EthernetStatistic::XmitOk,
    EthernetStatistic::RcvOk,
    EthernetStatistic::XmitError,
    EthernetStatistic::RcvError,
    EthernetStatistic::RcvCrcError,
};

// bmEthernetStatistics bitmap advertised in the Ethernet Networking Functional
// Descriptor: bit (selector - 1) is set for each supported selector.
inline constexpr std::uint32_t ETHERNET_STATISTICS_BITMAP{[] {
    std::uint32_t bitmap{0};
    for (const EthernetStatistic selector : SUPPORTED_STATISTICS) {
        bitmap |= 1u << (static_cast<std::uint32_t>(selector) - 1);
    }
    return bitmap;
}()};

// Locks the exact bitmap that goes on the wire in the functional descriptor:
// selectors 0x01..0x04 (bits 0..3) plus RCV_CRC_ERROR 0x12 (bit 17).
static_assert(ETHERNET_STATISTICS_BITMAP == 0x0002'000Fu);

[[nodiscard]] constexpr std::optional<std::uint32_t>
ethernet_statistic(EthernetStatistic selector, const TxStats& tx, const RxStats& rx) {
    switch (selector) {
        case EthernetStatistic::XmitOk:
            return tx.accepted;
        case EthernetStatistic::RcvOk:
            return rx.delivered;
        case EthernetStatistic::XmitError:
            return tx.build_failed + tx.dropped_busy;
        case EthernetStatistic::RcvError:
            return rx.error_total();
        case EthernetStatistic::RcvCrcError:
            return rx.bad_fcs;
    }
    return std::nullopt;
}

[[nodiscard]] constexpr std::optional<std::uint32_t>
ethernet_statistic(std::uint16_t selector, const TxStats& tx, const RxStats& rx) {
    return ethernet_statistic(static_cast<EthernetStatistic>(selector), tx, rx);
}

} // namespace pico_ethernet

#endif // PHY_PHY_STATS_H
