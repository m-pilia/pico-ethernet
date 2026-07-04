// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef USB_CDC_ECM_DEVICE_H
#define USB_CDC_ECM_DEVICE_H

#include <cstdint>
#include <span>

#include "src/mac/frame_filter.h"
#include "src/mac/mac_address.h"
#include "src/phy/phy.h"

namespace pico_ethernet {

// Application-layer wrapper over TinyUSB's CDC-ECM net class driver. Owns the USB
// device lifecycle and the RX packet filter, and bridges the host to the software
// PHY: frames the host sends over USB are built into wire frames and handed to
// the PHY transmit path. The wire-to-host receive path is added with the PHY RX
// bring-up; until then the device-to-host direction is empty. The error-prone USB
// plumbing (descriptors, endpoints, notifications) lives in TinyUSB.
class CdcEcmDevice {
  public:
    // TX outcome counters for on-device verification.
    struct TxStats {
        std::uint32_t accepted{0};     // frames handed to the PHY
        std::uint32_t build_failed{0}; // host frame too long to frame
        std::uint32_t dropped_busy{0}; // PHY still transmitting a prior frame
    };

    CdcEcmDevice(const MacAddress& mac_address, Phy& phy);

    // Publishes the MAC to TinyUSB, starts the USB device stack, registers this
    // instance for the C callbacks, and asserts the initial link state.
    void initialize();

    // Services USB events and the PHY transmit engine; call from the loop.
    void task();

    void set_link_up(bool up);
    bool link_up() const { return link_up_; }
    const MacAddress& mac_address() const { return mac_address_; }
    const TxStats& tx_stats() const { return stats_; }

    // Handlers invoked by the extern "C" TinyUSB network callbacks.
    bool on_frame_received(std::span<const std::uint8_t> host_frame);
    std::uint16_t on_frame_transmit(std::span<std::uint8_t> dst);
    void on_packet_filter(std::uint16_t bitmap) { filter_.set_packet_filter(bitmap); }
    void on_multicast_filter(std::span<const std::uint8_t> addresses, std::uint16_t count);
    void on_network_init();

  private:
    MacAddress mac_address_;
    Phy& phy_;
    bool link_up_{false};
    FrameFilter filter_;
    TxStats stats_{};
};

} // namespace pico_ethernet

#endif // USB_CDC_ECM_DEVICE_H
