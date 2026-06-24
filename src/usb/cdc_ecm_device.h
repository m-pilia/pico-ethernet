// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef USB_CDC_ECM_DEVICE_H
#define USB_CDC_ECM_DEVICE_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_filter.h"
#include "src/mac/mac_address.h"

namespace pico_ethernet {

// Application-layer wrapper over TinyUSB's CDC-ECM net class driver. It brings
// the link up and, lacking a PHY, loops host frames through the MAC TX and RX
// pipelines back to the host (see loopback_hack.h). The error-prone USB
// plumbing (descriptors, endpoints, notifications) lives in TinyUSB; this class
// owns the device lifecycle, the RX filter, and the loopback staging.
class CdcEcmDevice {
public:
    // RX outcome counters for on-device verification.
    struct RxStats {
        std::uint32_t accepted{0};
        std::uint32_t tx_build_failed{0};  // host frame too long to frame
        std::uint32_t bad_preamble{0};
        std::uint32_t runt{0};
        std::uint32_t giant{0};
        std::uint32_t bad_fcs{0};
        std::uint32_t filtered{0};
        std::uint32_t dropped_busy{0};     // a prior frame was still pending loopback
    };

    explicit CdcEcmDevice(const MacAddress& mac_address);

    // Publishes the MAC to TinyUSB, starts the USB device stack, registers this
    // instance for the C callbacks, and asserts the initial link state.
    void initialize();

    // Services USB events and drains a staged loopback frame; call from the loop.
    void task();

    void set_link_up(bool up);
    bool link_up() const { return link_up_; }
    const MacAddress& mac_address() const { return mac_address_; }
    const RxStats& rx_stats() const { return stats_; }

    // Handlers invoked by the extern "C" TinyUSB network callbacks.
    bool on_frame_received(std::span<const std::uint8_t> host_frame);
    std::uint16_t on_frame_transmit(std::span<std::uint8_t> dst);
    void on_packet_filter(std::uint16_t bitmap) { filter_.set_packet_filter(bitmap); }
    void on_multicast_filter(std::span<const std::uint8_t> addresses, std::uint16_t count);
    void on_network_init();

private:
    void service_loopback();
    void record_drop(FrameError error);

    MacAddress mac_address_;
    bool link_up_{false};
    FrameFilter filter_;
    RxStats stats_{};

    // Loopback staging: one host frame in flight at a time.
    std::array<std::uint8_t, MAX_FRAME_NO_FCS> loopback_buf_{};
    std::size_t loopback_len_{0};
    bool loopback_pending_{false};
};

} // namespace pico_ethernet

#endif // USB_CDC_ECM_DEVICE_H
