// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_STATS_H
#define PHY_PHY_STATS_H

#include <array>
#include <cstdint>
#include <optional>

#include "src/mac/ethernet_frame.h"

namespace pico_ethernet {

// Transmit-side outcome counters. Each field has a single writer so the reader in
// the USB ISR (GET_ETHERNET_STATISTIC) needs no locking: build_failed and the
// sent/underrun mirrors of the PHY's counters are written only on the main loop,
// usb_tx_overflow/oversize_dropped only in the USB ISR (see UsbNetDevice). A
// naturally-aligned 32-bit counter with one writer is read as a single coherent
// value on the Cortex-M33.
struct TxStats {
    std::uint32_t build_failed{0};     // host frame could not be framed (main loop)
    std::uint32_t usb_tx_overflow{0};  // host frame dropped: USB->PHY queue full under load (ISR)
    std::uint32_t oversize_dropped{0}; // host datagram larger than a wire frame; dropped in the ISR
    std::uint32_t sent{0};             // frames fully clocked onto the wire without underrun
    std::uint32_t underrun{0};         // frames corrupted by a mid-frame TX FIFO underrun
};

// Half-duplex transmit outcomes, tallied per frame once its fate is decided, which
// is what the CDC collision selectors mean: a frame is counted at most once, by how
// many collisions it took rather than by how many collisions occurred. Written only
// on the main loop and read by the USB ISR (GET_ETHERNET_STATISTIC); single-writer
// aligned 32-bit access needs no lock (see TxStats).
struct CsmaStats {
    std::uint32_t deferred{0};             // first attempt had to wait for a busy medium
    std::uint32_t single_collision{0};     // sent after exactly one collision
    std::uint32_t multiple_collisions{0};  // sent after two or more
    std::uint32_t excessive_collisions{0}; // abandoned after MAX_TX_ATTEMPTS attempts
    std::uint32_t late_collisions{0};      // collision past the slot time; dropped, never retried
    std::uint32_t link_down_dropped{0};    // queued frame discarded because the link went down
    std::uint32_t link_transitions{0};     // link up/down changes reported to the host
};

// Receive-side counters. The per-FrameError fields come out of the MAC parser
// unchanged; decode_error and carrier_glitch are PHY-level events with no MAC
// equivalent (a Manchester code violation, and carrier that never yielded a
// deliverable frame). All fields are written only on the main loop and read by the
// USB ISR (GET_ETHERNET_STATISTIC); single-writer aligned 32-bit access needs no
// lock (see TxStats).
struct RxStats {
    std::uint32_t delivered{0};
    std::uint32_t bad_preamble{0};
    std::uint32_t runt{0};
    std::uint32_t giant{0};
    std::uint32_t bad_fcs{0};
    std::uint32_t filtered{0};
    std::uint32_t decode_error{0};
    std::uint32_t carrier_glitch{0};
    std::uint32_t pool_overflow{0};     // buffer pool exhausted; frame dropped under load
    std::uint32_t host_backpressure{0}; // USB TX busy (frame did not fit the current NCM NTB)

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
        return bad_preamble + runt + giant + bad_fcs + decode_error + carrier_glitch + pool_overflow +
               host_backpressure;
    }
};

// CDC feature selectors for GetEthernetStatistic; one 32-bit counter is
// returned per selector. Only the subset the device actually maintains is listed.
enum class EthernetStatistic : std::uint16_t {
    XmitOk = 0x01,
    RcvOk = 0x02,
    XmitError = 0x03,
    RcvError = 0x04,
    RcvCrcError = 0x12,
    XmitOneCollision = 0x15,
    XmitMoreCollisions = 0x16,
    XmitDeferred = 0x17,
    XmitMaxCollisions = 0x18,
    XmitUnderrun = 0x1A,
    XmitLateCollisions = 0x1D,
};

inline constexpr std::array<EthernetStatistic, 11> SUPPORTED_STATISTICS{
    EthernetStatistic::XmitOk,
    EthernetStatistic::RcvOk,
    EthernetStatistic::XmitError,
    EthernetStatistic::RcvError,
    EthernetStatistic::RcvCrcError,
    EthernetStatistic::XmitOneCollision,
    EthernetStatistic::XmitMoreCollisions,
    EthernetStatistic::XmitDeferred,
    EthernetStatistic::XmitMaxCollisions,
    EthernetStatistic::XmitUnderrun,
    EthernetStatistic::XmitLateCollisions,
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
// selectors 0x01..0x04 (bits 0..3), RCV_CRC_ERROR 0x12 (bit 17), the collision and
// deferral selectors 0x15..0x18 (bits 20..23), XMIT_UNDERRUN 0x1A (bit 25) and
// XMIT_LATE_COLLISIONS 0x1D (bit 28).
static_assert(ETHERNET_STATISTICS_BITMAP == 0x12F2'000Fu);

[[nodiscard]] constexpr std::optional<std::uint32_t>
ethernet_statistic(EthernetStatistic selector, const TxStats& tx, const RxStats& rx, const CsmaStats& csma) {
    switch (selector) {
        case EthernetStatistic::XmitOk:
            return tx.sent;
        case EthernetStatistic::RcvOk:
            return rx.delivered;
        case EthernetStatistic::XmitError:
            return tx.build_failed + tx.usb_tx_overflow + tx.oversize_dropped + tx.underrun +
                   csma.excessive_collisions + csma.late_collisions + csma.link_down_dropped;
        case EthernetStatistic::RcvError:
            return rx.error_total();
        case EthernetStatistic::RcvCrcError:
            return rx.bad_fcs;
        case EthernetStatistic::XmitOneCollision:
            return csma.single_collision;
        case EthernetStatistic::XmitMoreCollisions:
            return csma.multiple_collisions;
        case EthernetStatistic::XmitDeferred:
            return csma.deferred;
        case EthernetStatistic::XmitMaxCollisions:
            return csma.excessive_collisions;
        case EthernetStatistic::XmitUnderrun:
            return tx.underrun;
        case EthernetStatistic::XmitLateCollisions:
            return csma.late_collisions;
    }
    return std::nullopt;
}

// Selectors for counters CDC has no code for: the RX sub-counters behind the
// aggregate rcv_error, and the link events. Read through the same
// GET_ETHERNET_STATISTIC request so no debug UART is needed to diagnose losses.
//
// These codes lie in the range USB-IF reserves for future CDC assignment, which is
// a deliberate deviation: the bmEthernetStatistics bitmap is 32 bits wide and can
// only ever advertise selectors 0x01..0x20, so a private counter cannot be
// advertised at all. No conformant host issues an unadvertised selector, and every
// unknown one is still stalled.
enum class Diagnostic : std::uint16_t {
    BadPreamble = 0xF0,
    Runt = 0xF1,
    Giant = 0xF2,
    BadFcs = 0xF3,
    CarrierGlitch = 0xF4,
    DecodeError = 0xF5,
    PoolOverflow = 0xF6,
    HostBackpressure = 0xF7,
    LinkDownDropped = 0xF8,
    LinkTransitions = 0xF9,
};

[[nodiscard]] constexpr std::optional<std::uint32_t>
diagnostic(std::uint16_t selector, const RxStats& rx, const CsmaStats& csma) {
    switch (static_cast<Diagnostic>(selector)) {
        case Diagnostic::BadPreamble:
            return rx.bad_preamble;
        case Diagnostic::Runt:
            return rx.runt;
        case Diagnostic::Giant:
            return rx.giant;
        case Diagnostic::BadFcs:
            return rx.bad_fcs;
        case Diagnostic::CarrierGlitch:
            return rx.carrier_glitch;
        case Diagnostic::DecodeError:
            return rx.decode_error;
        case Diagnostic::PoolOverflow:
            return rx.pool_overflow;
        case Diagnostic::HostBackpressure:
            return rx.host_backpressure;
        case Diagnostic::LinkDownDropped:
            return csma.link_down_dropped;
        case Diagnostic::LinkTransitions:
            return csma.link_transitions;
    }
    return std::nullopt;
}

[[nodiscard]] constexpr std::optional<std::uint32_t>
ethernet_statistic(std::uint16_t selector, const TxStats& tx, const RxStats& rx, const CsmaStats& csma) {
    if (const std::optional<std::uint32_t> standard{
            ethernet_statistic(static_cast<EthernetStatistic>(selector), tx, rx, csma)}) {
        return standard;
    }
    return diagnostic(selector, rx, csma);
}

} // namespace pico_ethernet

#endif // PHY_PHY_STATS_H
