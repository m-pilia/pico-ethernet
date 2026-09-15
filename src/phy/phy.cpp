// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/phy.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <expected>
#include <optional>
#include <span>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/sync.h"

#include "src/phy/phy_timing.h"
#include "src/phy/rx_frame_recover.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/tx_level.h"

#include "rx.pio.h"
#include "tx_level.pio.h"

namespace pico_ethernet {

namespace {
// The GPIO IRQ callback carries no user data, so the active PHY is published here
// for the RXC end-of-frame trampoline to forward to.
Phy* s_rx_irq_phy{nullptr};
} // namespace

Phy::Phy(const Pins& pins)
    : pins_{pins} {
    // TXP/TXN must be adjacent so the TX SM can drive them as one 2-pin SET
    // group; RXD/RXC must be adjacent so the RX SM can gate on RXC via IN base + 1.
    assert(pins_.txn == pins_.txp + 1);
    assert(pins_.rxc == pins_.rxd + 1);
}

bool Phy::initialize() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);

    if (!pio_can_add_program(pio_, &tx_level_program) || !pio_can_add_program(pio_, &rx_manchester_program)) {
        return false;
    }
    offset_tx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_level_program));
    offset_rx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &rx_manchester_program));

    const int smt{pio_claim_unused_sm(pio_, false)};
    const int smr{pio_claim_unused_sm(pio_, false)};
    if (smt < 0 || smr < 0) {
        return false;
    }
    sm_tx_ = static_cast<std::uint32_t>(smt);
    sm_rx_ = static_cast<std::uint32_t>(smr);

    dma_tx_ = dma_claim_unused_channel(false);
    dma_rx_ = dma_claim_unused_channel(false);
    if (dma_tx_ < 0 || dma_rx_ < 0) {
        return false;
    }

    configure_tx();
    configure_rx();
    force_idle();
    rearm_capture(rx_ring_.capture_slot());

    // End of frame is the carrier (RXC) falling edge; finalize each frame there
    // instead of waiting for the main loop to come back around.
    s_rx_irq_phy = this;
    gpio_set_irq_enabled_with_callback(pins_.rxc, GPIO_IRQ_EDGE_FALL, true, &Phy::rx_irq_handler);

    // Finalize each transmit (idle drive + receiver re-arm) in the TX-DMA completion
    // interrupt so the receiver comes back the instant our frame drains, rather than
    // waiting for the main loop.
    dma_channel_set_irq0_enabled(dma_tx_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, &Phy::tx_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    nlp_alarm_ = add_alarm_in_ms(nlp_.next_interval_ms(), &Phy::nlp_alarm_cb, this, true);
    return true;
}

void Phy::configure_tx() {
    pio_gpio_init(pio_, pins_.txp);
    pio_gpio_init(pio_, pins_.txn);

    // The default 4mA pad drive is too low to reach target differential amplitude.
    for (const std::uint32_t pin : {pins_.txp, pins_.txn}) {
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, pins_.txp, 2, true);

    // SET group {TXP,TXN}; autopull the raw frame LSB-first (802.3), one octet per
    // FIFO entry (8-bit DMA -> one byte per pull). /1 clock = 12 cycles/bit at
    // 120 MHz.
    tx_cfg_ = tx_level_program_get_default_config(offset_tx_);
    sm_config_set_set_pins(&tx_cfg_, pins_.txp, 2);
    sm_config_set_out_shift(&tx_cfg_, true, true, 8);
    sm_config_set_clkdiv(&tx_cfg_, 1.0f);
    pio_sm_init(pio_, sm_tx_, offset_tx_, &tx_cfg_);
}

void Phy::configure_rx() {
    // The comparators drive RXD/RXC externally. gpio_init de-isolates the pads
    // (RP2350 resets pads isolated, which gpio_set_input_enabled does not clear)
    // and enables their inputs without driving them; PIO reads a pad's input
    // independently of its function select, and the RXC end-of-frame IRQ and its
    // gpio_get() spurious-edge guard read the same input via SIO.
    gpio_init(pins_.rxd);
    gpio_init(pins_.rxc);

    rx_cfg_ = rx_manchester_program_get_default_config(offset_rx_);
    configure_rx_shift(rx_cfg_, pins_.rxd);
    pio_sm_init(pio_, sm_rx_, offset_rx_, &rx_cfg_);
}

void Phy::rearm_capture(std::size_t slot) {
    pio_sm_set_enabled(pio_, sm_rx_, false);
    pio_sm_clear_fifos(pio_, sm_rx_);
    // Full reset: PC back to the carrier-gate at the program start and the ISR
    // shift counter cleared, so the next frame's octets are byte-aligned.
    pio_sm_init(pio_, sm_rx_, offset_rx_, &rx_cfg_);

    dma_channel_config c{dma_channel_get_default_config(dma_rx_)};
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio_, sm_rx_, false));
    dma_channel_configure(dma_rx_, &c, rx_pool_[slot].data(), &pio_->rxf[sm_rx_], RX_WORD_CAPACITY, true);

    pio_sm_set_enabled(pio_, sm_rx_, true);
}

void Phy::rx_irq_handler(uint gpio, std::uint32_t events) {
    (void)gpio;
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u && s_rx_irq_phy != nullptr) {
        s_rx_irq_phy->on_rx_eof();
    }
}

void Phy::on_rx_eof() {
    if (gpio_get(pins_.rxc)) {
        return; // spurious edge (carrier envelope ripple); the frame is still live
    }

    // The RX SM has stalled on the missing edges and DMA has drained every complete
    // word, so the remaining count is stable and readable without stopping the DMA.
    const std::uint32_t remaining_before{dma_channel_hw_addr(dma_rx_)->transfer_count};
    const std::size_t words_before{RX_WORD_CAPACITY - remaining_before};

    // Half-duplex self-reception: while we transmit, our own signal is at our own
    // receiver. Don't enqueue it -- discard and keep capturing into the same slot.
    // A carrier blip that decoded nothing is likewise discarded.
    if (active_ || words_before == 0) {
        dma_channel_abort(dma_rx_);
        rearm_capture(rx_ring_.capture_slot());
        return;
    }

    // The frame's final FCS octet(s) may sit in the RX ISR below the 32-bit autopush
    // threshold (the line goes idle after the FCS, so no trailing edges flush the
    // word). Push the residual through to the DMA before stopping it, otherwise every
    // frame loses its FCS tail and fails validation.
    //
    // Only when the DMA still has a landing slot. If the capture ran the buffer full
    // (remaining_before == 0, i.e. words_before == capacity), the DMA is finished, the
    // SM has filled the RX FIFO with no drain, and forcing another autopush would stall
    // the SM on a full FIFO forever -- flush_rx_isr blocks on that exec, hanging the
    // end-of-frame interrupt. An over-length capture is a discarded giant anyway, so
    // skip the flush and let it be rejected.
    if (words_before < RX_WORD_CAPACITY) {
        flush_rx_isr(pio_, sm_rx_);
        while (!pio_sm_is_rx_fifo_empty(pio_, sm_rx_) && dma_channel_hw_addr(dma_rx_)->transfer_count != 0) {
            tight_loop_contents(); // let the DMA carry the flushed word into the buffer
        }
    }

    dma_channel_abort(dma_rx_);
    const std::uint32_t remaining{dma_channel_hw_addr(dma_rx_)->transfer_count};
    const std::size_t words{RX_WORD_CAPACITY - remaining};

    // Hand the DMA a free buffer for the next frame and queue this one for the
    // main-loop drain. No parsing or copying here -- that is the drain's job.
    rearm_capture(rx_ring_.publish(words));
}

Phy::RxFrame Phy::poll_rx() {
    const std::optional<RxSlotRing<RX_POOL_SIZE>::Completed> done{rx_ring_.peek()};
    if (!done) {
        return {}; // completed-frame queue empty
    }

    // The words pack four recovered octets each, LSB-first, so on this little-endian
    // core the capture buffer reads directly as the packed octet stream (octet j =
    // byte j). recover_frame() re-aligns and FCS-delimits it in one pass -- the
    // carrier gate started the decoder mid-preamble, so its octet boundaries are
    // bit-offset from the frame's.
    const std::span<const std::uint8_t> raw{
        reinterpret_cast<const std::uint8_t*>(rx_pool_[done->slot].data()), done->word_count * RX_OCTETS_PER_WORD};
    const std::expected<std::size_t, FrameError> recovered{recover_frame(raw, rx_frame_)};
    rx_ring_.release();

    if (!recovered) {
        // No SFD means the carrier carried no decodable frame (idle noise or a
        // partial capture); the remaining errors are captured-but-rejected frames.
        if (recovered.error() == FrameError::BadPreamble) {
            return {.kind = RxFrame::Kind::Glitch};
        }
        return {.kind = RxFrame::Kind::Error, .error = recovered.error()};
    }
    return {.kind = RxFrame::Kind::Frame, .frame = std::span<const std::uint8_t>(rx_frame_.data(), *recovered)};
}

bool Phy::transmit(std::span<const std::uint8_t> wire_frame) {
    if (wire_frame.empty() || wire_frame.size() > WIRE_CAPACITY) {
        return false;
    }
    const std::size_t head{tx_head_.load(std::memory_order_relaxed)};
    const std::size_t next{tx_advance(head)};
    if (next == tx_tail_.load(std::memory_order_acquire)) {
        return false; // full: the caller applies backpressure
    }

    // tx_head_ is producer-owned and the slot it points at is free, so fill it before
    // publishing it to the consumer with the release store.
    WireFrame& slot{tx_queue_[head]};
    std::ranges::copy(wire_frame, slot.bytes.begin());
    slot.length = wire_frame.size();
    tx_head_.store(next, std::memory_order_release);
    return true;
}

void Phy::start_tx(const WireFrame& frame) {
    // Reset the SM to the program start with a cleared FIFO and empty OSR so the
    // frame's first bit is shifted from the start of its first octet.
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_clear_fifos(pio_, sm_tx_);
    pio_sm_init(pio_, sm_tx_, offset_tx_, &tx_cfg_);

    dma_channel_config c{dma_channel_get_default_config(dma_tx_)};
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio_, sm_tx_, true));
    dma_channel_configure(dma_tx_, &c, &pio_->txf[sm_tx_], frame.bytes.data(), frame.length, false);

    active_ = true;
    // The end-of-frame IRQ stays enabled through the transmit: our own carrier holds
    // RXC high (no falling edge) until on_tx_complete() drives idle, and any capture
    // is dropped by the active_ guard in on_rx_eof().
    //
    // The SM stalls on autopull until the first octet lands, which latches TXSTALL.
    // Clear it once that octet is delivered so a TXSTALL seen in on_tx_complete()
    // means a mid-frame FIFO underrun. Interrupts are masked so a long ISR cannot
    // delay the clear past the end of the frame.
    const std::uint32_t irq_state{save_and_disable_interrupts()};
    pio_sm_set_enabled(pio_, sm_tx_, true);
    dma_channel_start(dma_tx_);
    while (dma_channel_hw_addr(dma_tx_)->transfer_count == frame.length) {
        tight_loop_contents();
    }
    pio_->fdebug = tx_stall_mask();
    restore_interrupts(irq_state);
}

std::uint32_t Phy::tx_stall_mask() const { return 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_tx_); }

bool Phy::transmitting() const {
    if (dma_tx_ < 0) {
        return false;
    }
    return dma_channel_is_busy(dma_tx_) || !pio_sm_is_tx_fifo_empty(pio_, sm_tx_);
}

void Phy::service() {
    // Start the next queued frame once the transmitter is idle and the interframe
    // gap has elapsed since the previous frame drained.
    if (active_ || transmitting()) {
        return;
    }
    const std::size_t tail{tx_tail_.load(std::memory_order_relaxed)};
    if (tail == tx_head_.load(std::memory_order_acquire)) {
        return; // queue empty
    }
    if (time_us_32() - tx_last_end_us_.load(std::memory_order_relaxed) < (IFG_NS + 999) / 1000) {
        return;
    }
    start_tx(tx_queue_[tail]);
}

void Phy::tx_dma_irq_handler() {
    if (s_rx_irq_phy != nullptr) {
        s_rx_irq_phy->on_tx_complete();
    }
}

void Phy::on_tx_complete() {
    dma_channel_acknowledge_irq0(dma_tx_);

    // The DMA has just queued the final octets, so while the FIFO still holds data the
    // SM cannot have stalled at the end of the frame: a TXSTALL then is an underrun.
    // An empty FIFO here (IRQ latency beyond the queued tail) is not counted, so the
    // count can miss underruns but never reports false ones.
    const bool underrun{(pio_->fdebug & tx_stall_mask()) != 0u && !pio_sm_is_tx_fifo_empty(pio_, sm_tx_)};
    std::atomic<std::uint32_t>& outcome{underrun ? tx_underrun_ : tx_sent_};
    outcome.store(outcome.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);

    // The DMA has handed off the last byte, but the PIO FIFO and shifter still hold
    // a few bytes; wait so force_idle() does not clip the tail of the frame. The FIFO
    // draining leaves up to one octet still shifting out of the OSR after the FIFO
    // reads empty, so add a two-octet margin before driving idle.
    while (transmitting()) {
        tight_loop_contents();
    }
    busy_wait_at_least_cycles(2 * 8 * PIO_CYCLES_PER_BIT);
    force_idle();

    // The RX SM captured our own transmission; drop it into a clean buffer so the
    // receiver is immediately live again for the next incoming frame.
    rearm_capture(rx_ring_.capture_slot());

    tx_last_end_us_.store(time_us_32(), std::memory_order_relaxed);
    // Release the just-sent slot back to the producer, then clear active_ last so a
    // service() that observes it false also sees the advanced tail.
    tx_tail_.store(tx_advance(tx_tail_.load(std::memory_order_relaxed)), std::memory_order_release);
    active_ = false;
}

void Phy::force_idle() {
    // Drive the differential pair to 0 V (TP_IDL then line idle). Injected while
    // the SM is stalled on an empty FIFO, so the level holds until the next frame.
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

void Phy::emit_nlp() {
    // A single ~100 ns positive excursion. Skip while a frame is in flight so the
    // pulse never corrupts data.
    if (active_ || transmitting()) {
        return;
    }
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_POS));
    busy_wait_at_least_cycles(PIO_CYCLES_PER_BIT);
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

std::int64_t Phy::nlp_alarm_cb(alarm_id_t /*id*/, void* user) {
    auto* self{static_cast<Phy*>(user)};
    self->emit_nlp();
    // Reschedule with fresh jitter; positive return = us from this scheduled fire.
    return static_cast<std::int64_t>(self->nlp_.next_interval_ms()) * 1000;
}

} // namespace pico_ethernet
