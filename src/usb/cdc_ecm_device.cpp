// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/usb/cdc_ecm_device.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

#include "src/mac/frame_builder.h"
#include "src/mac/frame_parser.h"
#include "src/mac/loopback_hack.h"  // TEMPORARY: remove in Phase 3

#include "tusb.h"

namespace pico_ethernet {

namespace {
constexpr std::uint8_t USB_RHPORT{0};

// The TinyUSB network callbacks have C linkage and no user-data argument, so the
// active device is published here for them to forward to.
CdcEcmDevice* g_instance{nullptr};
}

CdcEcmDevice::CdcEcmDevice(const MacAddress& mac_address)
    : mac_address_{mac_address}, filter_{mac_address} {}

void CdcEcmDevice::initialize() {
    g_instance = this;

    // TinyUSB reads tud_network_mac_address while building descriptors, so it
    // must be set before the stack starts.
    std::ranges::copy(mac_address_.bytes(), tud_network_mac_address);

    tusb_init();

    // Present the cable as connected so the host activates the data
    // interface and exchanges frames for the loopback demo.
    set_link_up(true);
}

void CdcEcmDevice::task() {
    tud_task();
    service_loopback();
}

void CdcEcmDevice::set_link_up(bool up) {
    link_up_ = up;
    tud_network_link_state(USB_RHPORT, up);
}

void CdcEcmDevice::record_drop(FrameError error) {
    switch (error) {
        case FrameError::TooLong:     ++stats_.tx_build_failed; break;
        case FrameError::BadPreamble: ++stats_.bad_preamble; break;
        case FrameError::Runt:        ++stats_.runt; break;
        case FrameError::Giant:       ++stats_.giant; break;
        case FrameError::BadFcs:      ++stats_.bad_fcs; break;
        case FrameError::Filtered:    ++stats_.filtered; break;
    }
}

bool CdcEcmDevice::on_frame_received(std::span<const std::uint8_t> host_frame) {
    // A frame is still staged for transmit; drop this one (we copy out of the
    // driver's buffer here, so returning false lets it re-arm the OUT endpoint).
    if (loopback_pending_) {
        ++stats_.dropped_busy;
        return false;
    }

    const auto built = build_frame(host_frame);
    if (!built) {
        ++stats_.tx_build_failed;
        return false;
    }

    const auto parsed = parse_frame(built->view(), filter_);
    if (!parsed) {
        record_drop(parsed.error());
        return false;
    }

    std::ranges::copy(*parsed, loopback_buf_.begin());
    loopback_len_ = parsed->size();
    // TEMPORARY: forge a reply addressed back to the host so the echo
    // is visible. Remove together with loopback_hack.h in Phase 3.
    loopback_swap_addresses(std::span(loopback_buf_).first(loopback_len_));
    loopback_pending_ = true;
    ++stats_.accepted;
    return false;
}

std::uint16_t CdcEcmDevice::on_frame_transmit(std::span<std::uint8_t> dst) {
    if (!loopback_pending_) return 0;

    const std::span<const std::uint8_t> frame{std::span(loopback_buf_).first(loopback_len_)};
    std::ranges::copy(frame, dst.begin());
    loopback_pending_ = false;
    return static_cast<std::uint16_t>(frame.size());
}

void CdcEcmDevice::on_multicast_filter(std::span<const std::uint8_t> addresses,
                                       std::uint16_t count) {
    const std::size_t n{std::min<std::size_t>(count, FrameFilter::MAX_MULTICAST)};
    std::array<MacAddress, FrameFilter::MAX_MULTICAST> list{};
    for (std::size_t i{0}; i < n; ++i) {
        list[i] = MacAddress(
            addresses.subspan(i * MacAddress::LENGTH).first<MacAddress::LENGTH>());
    }
    filter_.set_multicast_list(std::span(list).first(n));
}

void CdcEcmDevice::on_network_init() {
    loopback_pending_ = false;
    loopback_len_ = 0;
    // The host reprograms the filter on bring-up; start from accept-nothing.
    filter_.set_packet_filter(0);
}

void CdcEcmDevice::service_loopback() {
    if (loopback_pending_ && tud_network_can_xmit(static_cast<std::uint16_t>(loopback_len_))) {
        tud_network_xmit(nullptr, 0);
    }
}

} // namespace pico_ethernet

// TinyUSB network class callbacks (C linkage), forwarding to the active device.
extern "C" {

// Defined by the application; TinyUSB reads it for the iMACAddress descriptor.
std::uint8_t tud_network_mac_address[6] = {0};

bool tud_network_recv_cb(const std::uint8_t* src, std::uint16_t size) {
    if (pico_ethernet::g_instance == nullptr) return false;
    return pico_ethernet::g_instance->on_frame_received(std::span(src, size));
}

std::uint16_t tud_network_xmit_cb(std::uint8_t* dst, void* ref, std::uint16_t arg) {
    (void)ref;
    (void)arg;
    if (pico_ethernet::g_instance == nullptr) return 0;
    return pico_ethernet::g_instance->on_frame_transmit(
        std::span(dst, pico_ethernet::MAX_FRAME_NO_FCS));
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
            std::span(addresses, static_cast<std::size_t>(count) * pico_ethernet::MacAddress::LENGTH),
            count);
    }
}

// The combined ECM/RNDIS driver references this RNDIS handler, but we never
// advertise RNDIS (ECM-only descriptors), so it is never called. Stubbing it
// avoids compiling TinyUSB's rndis_reports.c, which depends on lwIP.
void rndis_class_set_handler(std::uint8_t* data, int size) {
    (void)data;
    (void)size;
}

} // extern "C"
