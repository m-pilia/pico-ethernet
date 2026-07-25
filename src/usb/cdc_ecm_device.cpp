// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/usb/cdc_ecm_device.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"

#include "tusb.h"

namespace pico_ethernet {

namespace {
constexpr std::uint8_t USB_RHPORT{0};

// The TinyUSB network callbacks have C linkage and no user-data argument, so the
// active device is published here for them to forward to.
CdcEcmDevice* g_instance{nullptr};
} // namespace

CdcEcmDevice::CdcEcmDevice(const MacAddress& mac_address, Phy& phy)
    : mac_address_{mac_address},
      phy_{phy},
      filter_{mac_address} {}

void CdcEcmDevice::initialize() {
    g_instance = this;

    // TinyUSB reads tud_network_mac_address while building descriptors, so it
    // must be set before the stack starts.
    std::ranges::copy(mac_address_.bytes(), tud_network_mac_address);

    tusb_init();

    // Present the cable as connected so the host activates the data interface and
    // sends frames. The wire-driven link state replaces this forced-up value with
    // the link FSM bring-up.
    set_link_up(true);
}

void CdcEcmDevice::task() {
    tud_task();
    phy_.service();

    // Mirror the interrupt-maintained pool-overflow counter into the RX stats.
    rx_stats_.pool_overflow = phy_.rx_pool_overflow();

    // recover_frame (bit realignment + CRC scan) is the costly part of the receive
    // path, so spend it only on a frame we can hand to USB right now. CDC-ECM keeps
    // one frame in flight, so at most one is deliverable per lap anyway; recovering
    // more just to drop them slows the loop and lets more captures overflow. Frames
    // left in the pool are dropped cheaply in the capture interrupt (pool_overflow).
    if (!tud_network_can_xmit(MAX_FRAME_NO_FCS)) {
        return;
    }
    const Phy::RxFrame received{phy_.poll_rx()};
    switch (received.kind) {
        case Phy::RxFrame::Kind::Frame:
            deliver_to_host(received.frame);
            break;
        case Phy::RxFrame::Kind::Glitch:
            ++rx_stats_.carrier_glitch;
            break;
        case Phy::RxFrame::Kind::Error:
            rx_stats_.record_error(received.error);
            break;
        case Phy::RxFrame::Kind::None:
            break;
    }
}

void CdcEcmDevice::deliver_to_host(std::span<const std::uint8_t> frame) {
    // The PHY already byte-aligned and FCS-delimited the frame; only the
    // destination filter and FCS strip remain before handing it to the host.
    if (!filter_.accept(destination_mac(frame))) {
        rx_stats_.record_error(FrameError::Filtered);
        return;
    }
    const std::span<const std::uint8_t> host_frame{frame.first(frame.size() - FCS_LEN)};

    // CDC-ECM carries one frame in flight over Full Speed; if the previous transfer
    // has not completed, this frame is a counted drop rather than a silent loss.
    if (!tud_network_can_xmit(static_cast<std::uint16_t>(host_frame.size()))) {
        ++rx_stats_.host_backpressure;
        return;
    }

    // tud_network_xmit invokes on_frame_transmit synchronously, so aliasing the
    // PHY's receive buffer through pending_host_frame_ is safe for this call.
    pending_host_frame_ = host_frame;
    tud_network_xmit(this, static_cast<std::uint16_t>(host_frame.size()));
    pending_host_frame_ = {};
    ++rx_stats_.delivered;
}

void CdcEcmDevice::set_link_up(bool up) {
    link_up_ = up;
    tud_network_link_state(USB_RHPORT, up);
}

bool CdcEcmDevice::on_frame_received(std::span<const std::uint8_t> host_frame) {
    // Build the wire frame (pad, FCS, preamble/SFD) and hand it to the PHY. We
    // copy synchronously here, so returning false lets TinyUSB re-arm reception.
    const auto built = build_frame(host_frame);
    if (!built) {
        ++stats_.build_failed;
        return false;
    }

    if (!phy_.transmit(built->view())) {
        ++stats_.dropped_busy;
        return false;
    }

    ++stats_.accepted;
    return false;
}

std::uint16_t CdcEcmDevice::on_frame_transmit(std::span<std::uint8_t> dst) {
    if (pending_host_frame_.empty() || pending_host_frame_.size() > dst.size()) {
        return 0;
    }
    std::ranges::copy(pending_host_frame_, dst.begin());
    return static_cast<std::uint16_t>(pending_host_frame_.size());
}

void CdcEcmDevice::on_multicast_filter(std::span<const std::uint8_t> addresses, std::uint16_t count) {
    const std::size_t n{std::min<std::size_t>(count, FrameFilter::MAX_MULTICAST)};
    std::array<MacAddress, FrameFilter::MAX_MULTICAST> list{};
    for (std::size_t i{0}; i < n; ++i) {
        list[i] = MacAddress(addresses.subspan(i * MacAddress::LENGTH).first<MacAddress::LENGTH>());
    }
    filter_.set_multicast_list(std::span(list).first(n));
}

void CdcEcmDevice::on_network_init() {
    // The host reprograms the filter on bring-up; start from accept-nothing.
    filter_.set_packet_filter(0);
}

bool CdcEcmDevice::on_get_statistic(std::uint16_t selector, std::uint32_t& value) const {
    const auto result = ethernet_statistic(selector, stats_, rx_stats_);
    if (!result) {
        return false;
    }
    value = *result;
    return true;
}

} // namespace pico_ethernet

// TinyUSB network class callbacks (C linkage), forwarding to the active device.
extern "C" {

// Defined by the application; TinyUSB reads it for the iMACAddress descriptor.
std::uint8_t tud_network_mac_address[6] = {0};

bool tud_network_recv_cb(const std::uint8_t* src, std::uint16_t size) {
    if (pico_ethernet::g_instance == nullptr)
        return false;
    return pico_ethernet::g_instance->on_frame_received(std::span(src, size));
}

std::uint16_t tud_network_xmit_cb(std::uint8_t* dst, void* ref, std::uint16_t arg) {
    (void)ref;
    (void)arg;
    if (pico_ethernet::g_instance == nullptr)
        return 0;
    return pico_ethernet::g_instance->on_frame_transmit(std::span(dst, pico_ethernet::MAX_FRAME_NO_FCS));
}

void tud_network_init_cb(void) {
    if (pico_ethernet::g_instance != nullptr) {
        pico_ethernet::g_instance->on_network_init();
    }
}

void tud_network_set_packet_filter_cb(std::uint16_t packet_filter) {
    if (pico_ethernet::g_instance != nullptr) {
        pico_ethernet::g_instance->on_packet_filter(packet_filter);
    }
}

void tud_network_set_multicast_filter_cb(const std::uint8_t* addresses, std::uint16_t count) {
    if (pico_ethernet::g_instance != nullptr) {
        pico_ethernet::g_instance->on_multicast_filter(
            std::span(addresses, static_cast<std::size_t>(count) * pico_ethernet::MacAddress::LENGTH), count);
    }
}

bool tud_network_get_statistic_cb(std::uint16_t feature_selector, std::uint32_t* value) {
    if (pico_ethernet::g_instance == nullptr) {
        return false;
    }
    return pico_ethernet::g_instance->on_get_statistic(feature_selector, *value);
}

// The combined ECM/RNDIS driver references this RNDIS handler, but we never
// advertise RNDIS (ECM-only descriptors), so it is never called. Stubbing it
// avoids compiling TinyUSB's rndis_reports.c, which depends on lwIP.
void rndis_class_set_handler(std::uint8_t* data, int size) {
    (void)data;
    (void)size;
}

} // extern "C"
