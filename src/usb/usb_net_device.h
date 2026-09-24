// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef USB_USB_NET_DEVICE_H
#define USB_USB_NET_DEVICE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_filter.h"
#include "src/mac/mac_address.h"
#include "src/phy/phy.h"
#include "src/phy/phy_stats.h"

namespace pico_ethernet {

// Application-layer wrapper over TinyUSB's CDC-NCM net class driver. Owns the USB
// device lifecycle and the RX packet filter, and bridges the host to the software
// PHY: frames the host sends over USB are built into wire frames and handed to
// the PHY transmit path, and frames the PHY receives are filtered and passed back
// to the host. The error-prone USB plumbing (descriptors, endpoints, notifications)
// lives in TinyUSB.
//
// USB is serviced from the USB controller interrupt: a shared USBCTRL_IRQ handler
// runs tud_task_ext() so host events are handled the instant they occur, rather
// than once per lap of the main loop. The tud_network_recv_cb callback therefore
// runs in ISR context and does minimal work (copies the host frame into a queue);
// the heavy framing (build_frame + CRC) and the PHY transmit happen in the
// main-loop dispatcher. USBCTRL_IRQ is given a less-urgent NVIC priority than the
// PHY's real-time IRQs so a long USB service never delays frame capture. Any
// main-loop access to the TinyUSB net-driver API or to state shared with the ISR
// callbacks is guarded by a brief USB-interrupt disable (UsbInterruptLock).
class UsbNetDevice {
  public:
    UsbNetDevice(const MacAddress& mac_address, Phy& phy);

    // Publishes the MAC to TinyUSB, starts the USB device stack, registers the
    // shared USB interrupt handler that drives tud_task_ext(), and asserts the
    // initial link state.
    void initialize();

    // Main-loop dispatcher: drains host frames the ISR queued into the PHY transmit
    // path and feeds recovered RX frames back to the host. USB itself is serviced
    // from the interrupt, not here.
    void task();

    const MacAddress& mac_address() const { return mac_address_; }
    const TxStats& tx_stats() const { return stats_; }
    const RxStats& rx_stats() const { return rx_stats_; }

    // Handlers invoked by the extern "C" TinyUSB network callbacks. recv runs in
    // ISR context (USB is interrupt-driven); the others fire from tud_task_ext()
    // in the same USB ISR.
    bool on_frame_received(std::span<const std::uint8_t> host_frame);
    std::uint16_t on_frame_transmit(std::span<std::uint8_t> dst);
    void on_packet_filter(std::uint16_t bitmap) { filter_.set_packet_filter(bitmap); }
    void on_multicast_filter(std::span<const std::uint8_t> addresses, std::uint16_t count);
    void on_network_init();

    // Answers a GetEthernetStatistic request: resolves the feature
    // selector against the TX and PHY RX counters. False for unsupported
    // selectors so the control transfer is stalled.
    [[nodiscard]] bool on_get_statistic(std::uint16_t selector, std::uint32_t& value) const;

  private:
    // Notifies the host when the wire-driven link state changes. The first call
    // always notifies, so the host learns the initial state.
    void publish_link_state(bool up);

    // Applies the destination filter to a byte-aligned, FCS-delimited frame from
    // the PHY and, on acceptance, strips the FCS and hands the host-facing frame to
    // the NCM transmit path (appended as a datagram to the current NTB).
    void deliver_to_host(std::span<const std::uint8_t> frame);

    // Moves host frames the ISR queued (USB->PHY direction) into the PHY transmit
    // engine, building wire frames and applying PHY-TX backpressure.
    void drain_usb_tx();

    MacAddress mac_address_;
    Phy& phy_;
    bool link_up_{false};
    bool link_state_published_{false};
    FrameFilter filter_;
    TxStats stats_{};
    RxStats rx_stats_{};
    std::span<const std::uint8_t> pending_host_frame_{};

    // SPSC queue of raw host frames bridging the USB ISR (producer) to the main
    // loop (consumer). Single-producer/single-consumer: the ISR owns usb_tx_head_,
    // the main loop owns usb_tx_tail_, so the indices need no lock. Sized to absorb
    // a full OUT NTB burst (NCM packs up to CFG_TUD_NCM_OUT_MAX_DATAGRAMS_PER_NTB = 6
    // datagrams per transfer) so a queued NTB rarely overflows before it drains.
    struct UsbTxSlot {
        std::array<std::uint8_t, MAX_FRAME_NO_FCS> data{};
        std::uint16_t len{0};
    };
    static constexpr std::size_t USB_TX_QUEUE_SIZE{8};
    static constexpr std::size_t usb_tx_advance(std::size_t i) { return (i + 1 == USB_TX_QUEUE_SIZE) ? 0 : i + 1; }
    std::array<UsbTxSlot, USB_TX_QUEUE_SIZE> usb_tx_slots_{};
    std::atomic<std::size_t> usb_tx_head_{0};
    std::atomic<std::size_t> usb_tx_tail_{0};
};

} // namespace pico_ethernet

#endif // USB_USB_NET_DEVICE_H
