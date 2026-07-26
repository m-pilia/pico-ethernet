// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/usb/usb_net_device.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "hardware/irq.h"

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"
#include "src/util/instrumentation.h" // TEMPORARY: MILESTONE 1.5 Step 1 diagnostics

#include "tusb.h"

// Shared USB interrupt handler driving tud_task_ext(); defined below.
extern "C" void pico_ethernet_usb_irq_handler(void);

namespace pico_ethernet {

namespace {
constexpr std::uint8_t USB_RHPORT{0};

// NVIC priority for USBCTRL_IRQ. Higher numeric value = less urgent on Cortex-M33;
// this is below the PHY's IRQs (RXC edge on IO_IRQ_BANK0 and TX-DMA on DMA_IRQ_0,
// both at the pico-sdk default 0x80), so the real-time capture path preempts the
// USB service (which runs the whole device task, including a full-MTU frame copy).
// This is the NVIC hardware priority, distinct from the shared-handler *order*
// priority that sequences our handler after TinyUSB's on the same line.
constexpr std::uint8_t USB_IRQ_PRIORITY{0xC0};

// The TinyUSB network callbacks have C linkage and no user-data argument, so the
// active device is published here for them to forward to.
UsbNetDevice* g_instance{nullptr};

// Disables the USB controller interrupt for its scope. tud_task_ext() runs in that
// interrupt, so any main-loop call into the TinyUSB net-driver API
// (tud_network_can_xmit/xmit, tud_network_link_state) and any read of state the ISR
// callbacks rewrite (filter_) must be guarded by this to avoid racing the ISR. It
// masks only USBCTRL_IRQ, so the PHY's real-time IRQs stay live while it is held.
class UsbInterruptLock {
  public:
    UsbInterruptLock() : was_enabled_{irq_is_enabled(USBCTRL_IRQ)} { irq_set_enabled(USBCTRL_IRQ, false); }
    ~UsbInterruptLock() { irq_set_enabled(USBCTRL_IRQ, was_enabled_); }
    UsbInterruptLock(const UsbInterruptLock&) = delete;
    UsbInterruptLock& operator=(const UsbInterruptLock&) = delete;

  private:
    bool was_enabled_;
};
} // namespace

UsbNetDevice::UsbNetDevice(const MacAddress& mac_address, Phy& phy)
    : mac_address_{mac_address},
      phy_{phy},
      filter_{mac_address} {}

void UsbNetDevice::initialize() {
    g_instance = this;

    // TinyUSB reads tud_network_mac_address while building descriptors, so it
    // must be set before the stack starts.
    std::ranges::copy(mac_address_.bytes(), tud_network_mac_address);

    tusb_init();

    // Service the device stack from the USB interrupt. TinyUSB's DCD registered its
    // handler at the highest order priority during tusb_init(); ours runs after it
    // (lowest order priority) and drains the queued events, so the net callbacks fire
    // in ISR context instead of from the main loop.
    irq_add_shared_handler(USBCTRL_IRQ, &pico_ethernet_usb_irq_handler,
                           PICO_SHARED_IRQ_HANDLER_LOWEST_ORDER_PRIORITY);
    irq_set_priority(USBCTRL_IRQ, USB_IRQ_PRIORITY);

    // Present the cable as connected so the host activates the data interface and
    // sends frames. The wire-driven link state replaces this forced-up value with
    // the link FSM bring-up.
    set_link_up(true);
}

void UsbNetDevice::task() {
    phy_.service();

    // Move host frames the USB ISR queued into the PHY transmit path.
    drain_usb_tx();

    // Mirror the interrupt-maintained pool-overflow counter into the RX stats.
    rx_stats_.pool_overflow = phy_.rx_pool_overflow();

    // Drain recovered frames to the host, letting the NCM driver aggregate several
    // datagrams into one NTB per USB transfer. recover_frame (bit realignment + CRC
    // scan) is the costly part, so gate each recovery on being able to hand the frame
    // to USB right now; stop as soon as the NTB path is full or the pool is empty.
    // Frames left in the pool are dropped cheaply in the capture interrupt
    // (pool_overflow).
    for (;;) {
        {
            UsbInterruptLock lock;
            if (!tud_network_can_xmit(MAX_FRAME_NO_FCS)) {
                break;
            }
        }
        ++g_instrument.can_xmit_true; // TEMPORARY: M1.5 Step 1
        const Phy::RxFrame received{phy_.poll_rx()};
        if (received.kind == Phy::RxFrame::Kind::None) {
            break;
        }
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
                break; // handled above
        }
    }
}

void UsbNetDevice::drain_usb_tx() {
    // Retry a frame held over from a full PHY TX queue.
    if (tx_backpressured_) {
        if (phy_.transmit(pending_tx_.view())) {
            tx_backpressured_ = false;
            ++stats_.accepted;
        } else {
            return; // PHY TX still congested; dequeueing more would just fail too
        }
    }

    // Frame and transmit what the ISR queued. The slot is released (tail advanced)
    // before the PHY transmit, since the wire frame has already been copied out into
    // build_frame's result, so the ISR is free to reuse the slot immediately.
    for (;;) {
        const std::size_t tail{usb_tx_tail_.load(std::memory_order_relaxed)};
        if (tail == usb_tx_head_.load(std::memory_order_acquire)) {
            return; // queue empty
        }
        const std::span<const std::uint8_t> raw{usb_tx_slots_[tail].data.data(), usb_tx_slots_[tail].len};
        const auto built{build_frame(raw)};
        usb_tx_tail_.store(usb_tx_advance(tail), std::memory_order_release);
        if (!built) {
            ++stats_.build_failed;
            continue;
        }
        if (!phy_.transmit(built->view())) {
            pending_tx_ = *built;
            tx_backpressured_ = true;
            return; // PHY TX full; hold this frame, stop draining
        }
        ++stats_.accepted;
    }
}

void UsbNetDevice::deliver_to_host(std::span<const std::uint8_t> frame) {
    // The PHY already byte-aligned and FCS-delimited the frame; only the
    // destination filter and FCS strip remain before handing it to the host.
    const std::span<const std::uint8_t> host_frame{frame.first(frame.size() - FCS_LEN)};

    // filter_ is rewritten by the ISR's packet-filter/multicast callbacks and the
    // xmit path races tud_task_ext() in the ISR, so the filter check and the whole
    // TinyUSB transfer run under the USB interrupt lock.
    UsbInterruptLock lock;
    if (!filter_.accept(destination_mac(frame))) {
        rx_stats_.record_error(FrameError::Filtered);
        return;
    }

    // NCM appends this frame as a datagram to the current NTB; tud_network_can_xmit
    // reports whether it still fits the current (or a fresh) NTB. If not, it is a
    // counted drop rather than a silent loss, and this lap's drain stops.
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

void UsbNetDevice::set_link_up(bool up) {
    link_up_ = up;
    UsbInterruptLock lock;
    tud_network_link_state(USB_RHPORT, up);
}

bool UsbNetDevice::on_frame_received(std::span<const std::uint8_t> host_frame) {
    // ISR context, NCM per-datagram callback. Copy the raw host frame into the SPSC
    // queue for the main loop to build into a wire frame and transmit, then pump the
    // next datagram of the NTB. Returning true tells the NCM driver the datagram was
    // consumed (it advances); tud_network_recv_renew() drives the driver's own
    // re-entrancy-safe loop through the rest of the NTB's datagrams.
    //
    // Both failure paths still count as consumed so the NTB keeps draining rather than
    // stalling on a datagram we will never accept: a frame too large to ever be framed
    // (larger than a wire frame can hold), and a full queue under load -- both are
    // visible counted drops, not silent losses. These counters are ISR-owned
    // (oversize_dropped, usb_tx_overflow), distinct from the main loop's build_failed,
    // so every counter has a single writer (see TxStats).
    if (host_frame.size() > MAX_FRAME_NO_FCS) {
        ++stats_.oversize_dropped;
    } else {
        const std::size_t head{usb_tx_head_.load(std::memory_order_relaxed)};
        const std::size_t next{usb_tx_advance(head)};
        if (next == usb_tx_tail_.load(std::memory_order_acquire)) {
            ++stats_.usb_tx_overflow;
        } else {
            UsbTxSlot& slot{usb_tx_slots_[head]};
            std::ranges::copy(host_frame, slot.data.begin());
            slot.len = static_cast<std::uint16_t>(host_frame.size());
            usb_tx_head_.store(next, std::memory_order_release);
        }
    }
    tud_network_recv_renew();
    return true;
}

std::uint16_t UsbNetDevice::on_frame_transmit(std::span<std::uint8_t> dst) {
    if (pending_host_frame_.empty() || pending_host_frame_.size() > dst.size()) {
        return 0;
    }
    std::ranges::copy(pending_host_frame_, dst.begin());
    return static_cast<std::uint16_t>(pending_host_frame_.size());
}

void UsbNetDevice::on_multicast_filter(std::span<const std::uint8_t> addresses, std::uint16_t count) {
    const std::size_t n{std::min<std::size_t>(count, FrameFilter::MAX_MULTICAST)};
    std::array<MacAddress, FrameFilter::MAX_MULTICAST> list{};
    for (std::size_t i{0}; i < n; ++i) {
        list[i] = MacAddress(addresses.subspan(i * MacAddress::LENGTH).first<MacAddress::LENGTH>());
    }
    filter_.set_multicast_list(std::span(list).first(n));
}

void UsbNetDevice::on_network_init() {
    // The host reprograms the filter on bring-up; start from accept-nothing.
    filter_.set_packet_filter(0);
}

bool UsbNetDevice::on_get_statistic(std::uint16_t selector, std::uint32_t& value) const {
    // Runs in the USB ISR. It reads stats_/rx_stats_ without a lock: every counter
    // has a single writer (see TxStats), so each aligned 32-bit read is coherent.
    //
    // TEMPORARY: M1.5 Step 1 diagnostics live in a private selector range that does
    // not overlap the standard/diagnostic selectors resolved below.
    if (const auto instrumented = instrument_statistic(selector)) {
        value = *instrumented;
        return true;
    }
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

// Shared USB controller interrupt handler: runs after TinyUSB's DCD handler
// (highest order priority) has enqueued device events, and drains them through
// tud_task_ext() so the device stack is serviced from the interrupt rather than the
// main loop. Registered in UsbNetDevice::initialize().
void pico_ethernet_usb_irq_handler(void) {
    const pico_ethernet::InstrumentScope timer{pico_ethernet::g_instrument.tud_task}; // TEMPORARY: M1.5 Step 1
    tud_task_ext(0, true);
}

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

} // extern "C"
