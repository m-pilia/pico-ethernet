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

#include "src/phy/phy_timing.h"
#include "src/phy/rx_frame_recover.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/tx_emphasis.h"
#include "src/util/instrumentation.h" // TEMPORARY: MILESTONE 1.5 Step 1 diagnostics

#include "rx.pio.h"
#include "tx_emphasis.pio.h"
#include "tx_level.pio.h"

namespace pico_ethernet {

namespace {
// The GPIO IRQ callback carries no user data, so the active PHY is published here
// for the RXC end-of-frame trampoline to forward to.
Phy* s_rx_irq_phy{nullptr};
} // namespace

Phy::Phy(const Pins& pins)
    : pins_{pins} {
    // TXP/TXN must be adjacent so the LEVEL SM can drive them as one 2-pin SET
    // group; RXD/RXC must be adjacent so the RX SM can gate on RXC via IN base + 1.
    assert(pins_.txn == pins_.txp + 1);
    assert(pins_.rxc == pins_.rxd + 1);
}

bool Phy::initialize() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);

    if (!pio_can_add_program(pio_, &tx_level_program) || !pio_can_add_program(pio_, &tx_emphasis_program) ||
        !pio_can_add_program(pio_, &rx_manchester_program)) {
        return false;
    }
    offset_level_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_level_program));
    offset_emphasis_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_emphasis_program));
    offset_rx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &rx_manchester_program));

    const int sml{pio_claim_unused_sm(pio_, false)};
    const int sme{pio_claim_unused_sm(pio_, false)};
    const int smr{pio_claim_unused_sm(pio_, false)};
    if (sml < 0 || sme < 0 || smr < 0) {
        return false;
    }
    sm_level_ = static_cast<std::uint32_t>(sml);
    sm_emphasis_ = static_cast<std::uint32_t>(sme);
    sm_rx_ = static_cast<std::uint32_t>(smr);

    dma_level_ = dma_claim_unused_channel(false);
    dma_emphasis_ = dma_claim_unused_channel(false);
    dma_rx_ = dma_claim_unused_channel(false);
    if (dma_level_ < 0 || dma_emphasis_ < 0 || dma_rx_ < 0) {
        return false;
    }

    configure_state_machines();
    configure_rx();
    force_idle();
    rearm_capture(rx_ring_.capture_slot());

    // End of frame is the carrier (RXC) falling edge; finalize each frame there
    // instead of waiting for the main loop to come back around.
    s_rx_irq_phy = this;
    gpio_set_irq_enabled_with_callback(pins_.rxc, GPIO_IRQ_EDGE_FALL, true, &Phy::rx_irq_handler);

    nlp_alarm_ = add_alarm_in_ms(nlp_.next_interval_ms(), &Phy::nlp_alarm_cb, this, true);
    return true;
}

void Phy::configure_state_machines() {
    pio_gpio_init(pio_, pins_.txp);
    pio_gpio_init(pio_, pins_.txn);
    pio_gpio_init(pio_, pins_.txe);

    // The default 4mA pad drive is too low to reach target differential amplitude.
    for (const std::uint32_t pin : {pins_.txp, pins_.txn, pins_.txe}) {
        gpio_set_drive_strength(pin, GPIO_DRIVE_STRENGTH_12MA);
        gpio_set_slew_rate(pin, GPIO_SLEW_RATE_FAST);
    }

    pio_sm_set_consecutive_pindirs(pio_, sm_level_, pins_.txp, 2, true);
    pio_sm_set_consecutive_pindirs(pio_, sm_emphasis_, pins_.txe, 1, true);

    // LEVEL SM: SET group {TXP,TXN}; autopull the raw frame LSB-first (802.3),
    // one octet per FIFO entry (8-bit DMA -> one byte per pull). /1 clock =
    // 12 cycles/bit at 120 MHz.
    level_cfg_ = tx_level_program_get_default_config(offset_level_);
    sm_config_set_set_pins(&level_cfg_, pins_.txp, 2);
    sm_config_set_out_shift(&level_cfg_, true, true, 8);
    sm_config_set_clkdiv(&level_cfg_, 1.0f);
    pio_sm_init(pio_, sm_level_, offset_level_, &level_cfg_);

    // EMPHASIS SM: SET and OUT pin = TXE; autopull the packed emphasis bits
    // LSB-first, one octet per FIFO entry.
    emphasis_cfg_ = tx_emphasis_program_get_default_config(offset_emphasis_);
    sm_config_set_set_pins(&emphasis_cfg_, pins_.txe, 1);
    sm_config_set_out_pins(&emphasis_cfg_, pins_.txe, 1);
    sm_config_set_out_shift(&emphasis_cfg_, true, true, 8);
    sm_config_set_clkdiv(&emphasis_cfg_, 1.0f);
    pio_sm_init(pio_, sm_emphasis_, offset_emphasis_, &emphasis_cfg_);
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
    (void)events;
    if (s_rx_irq_phy != nullptr) {
        s_rx_irq_phy->on_rx_eof();
    }
}

void Phy::on_rx_eof() {
    // TEMPORARY: M1.5 Step 1. Times every IRQ fire, including the spurious-edge and
    // self-transmit early returns, so the count reflects the true IRQ load.
    const InstrumentScope timer{g_instrument.rx_irq};
    if (gpio_get(pins_.rxc)) {
        return; // spurious edge (carrier envelope ripple); the frame is still live
    }

    // The RX SM has stalled on the missing edges and DMA has drained every complete
    // word, so the remaining count is stable.
    dma_channel_abort(dma_rx_);
    const std::uint32_t remaining{dma_channel_hw_addr(dma_rx_)->transfer_count};
    const std::size_t words{RX_WORD_CAPACITY - remaining};

    // Half-duplex self-reception: while we transmit, our own signal is at our own
    // receiver. Don't enqueue it -- discard and keep capturing into the same slot.
    if (active_ || words == 0) {
        rearm_capture(rx_ring_.capture_slot());
        return;
    }

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
    const std::expected<std::size_t, FrameError> recovered{[&] {
        const InstrumentScope timer{g_instrument.recover_frame}; // TEMPORARY: M1.5 Step 1
        return recover_frame(raw, rx_frame_);
    }()};
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
    if (active_ || transmitting()) {
        return false;
    }
    if (wire_frame.empty() || wire_frame.size() > WIRE_CAPACITY) {
        return false;
    }

    tx_len_ = wire_frame.size();
    std::ranges::copy(wire_frame, tx_frame_.begin());
    // One emphasis bit per data bit, packed one octet per octet of frame, so both
    // DMA channels move exactly tx_len_ bytes.
    compute_emphasis_bits(std::span(tx_frame_).first(tx_len_), std::span(tx_emphasis_).first(tx_len_));

    // Reset both SMs to the program start with cleared FIFOs so the two-SM phase
    // is deterministic from the first bit.
    pio_sm_set_enabled(pio_, sm_level_, false);
    pio_sm_set_enabled(pio_, sm_emphasis_, false);
    pio_sm_clear_fifos(pio_, sm_level_);
    pio_sm_clear_fifos(pio_, sm_emphasis_);
    pio_sm_init(pio_, sm_level_, offset_level_, &level_cfg_);
    pio_sm_init(pio_, sm_emphasis_, offset_emphasis_, &emphasis_cfg_);

    dma_channel_config cl{dma_channel_get_default_config(dma_level_)};
    channel_config_set_transfer_data_size(&cl, DMA_SIZE_8);
    channel_config_set_read_increment(&cl, true);
    channel_config_set_write_increment(&cl, false);
    channel_config_set_dreq(&cl, pio_get_dreq(pio_, sm_level_, true));
    dma_channel_configure(dma_level_, &cl, &pio_->txf[sm_level_], tx_frame_.data(), tx_len_, false);

    dma_channel_config ce{dma_channel_get_default_config(dma_emphasis_)};
    channel_config_set_transfer_data_size(&ce, DMA_SIZE_8);
    channel_config_set_read_increment(&ce, true);
    channel_config_set_write_increment(&ce, false);
    channel_config_set_dreq(&ce, pio_get_dreq(pio_, sm_emphasis_, true));
    dma_channel_configure(dma_emphasis_, &ce, &pio_->txf[sm_emphasis_], tx_emphasis_.data(), tx_len_, false);

    active_ = true;
    // Half-duplex: our own carrier will assert RXC and its trailing fall would fire
    // the end-of-frame IRQ. Suppress it for the whole transmit; service() discards
    // the self-received capture and re-enables once the line is idle again.
    gpio_set_irq_enabled(pins_.rxc, GPIO_IRQ_EDGE_FALL, false);
    // Enable both SMs on the same cycle (phase-locked dividers), then kick both
    // DMA channels together. TXE-vs-polarity phase alignment is confirmed and
    // trimmed on-target by the DBG_PADOUT self-test.
    pio_enable_sm_mask_in_sync(pio_, (1u << sm_level_) | (1u << sm_emphasis_));
    dma_start_channel_mask(
        (1u << static_cast<std::uint32_t>(dma_level_)) | (1u << static_cast<std::uint32_t>(dma_emphasis_)));
    return true;
}

bool Phy::transmitting() const {
    if (dma_level_ < 0) {
        return false;
    }
    return dma_channel_is_busy(dma_level_) || dma_channel_is_busy(dma_emphasis_) ||
           !pio_sm_is_tx_fifo_empty(pio_, sm_level_) || !pio_sm_is_tx_fifo_empty(pio_, sm_emphasis_);
}

void Phy::service() {
    if (active_ && !transmitting()) {
        force_idle();
        active_ = false;
        // The RX SM captured our own transmission; drop it into a clean buffer, then
        // clear the latched TX-end edge and re-enable end-of-frame reception. The IRQ
        // is disabled here, so this cannot race the handler.
        rearm_capture(rx_ring_.capture_slot());
        gpio_acknowledge_irq(pins_.rxc, GPIO_IRQ_EDGE_FALL);
        gpio_set_irq_enabled(pins_.rxc, GPIO_IRQ_EDGE_FALL, true);
    }
}

void Phy::force_idle() {
    // Drive the differential pair to 0 V (TP_IDL then line idle). Injected while
    // the SM is stalled on an empty FIFO, so the level holds until the next frame.
    pio_sm_exec(pio_, sm_level_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

void Phy::emit_nlp() {
    // A single ~100 ns positive excursion. Skip while a frame is in flight so the
    // pulse never corrupts data.
    if (active_ || transmitting()) {
        return;
    }
    pio_sm_exec(pio_, sm_level_, pio_encode_set(pio_pins, LEVEL_POS));
    busy_wait_at_least_cycles(PIO_CYCLES_PER_BIT);
    pio_sm_exec(pio_, sm_level_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

std::int64_t Phy::nlp_alarm_cb(alarm_id_t /*id*/, void* user) {
    auto* self{static_cast<Phy*>(user)};
    self->emit_nlp();
    // Reschedule with fresh jitter; positive return = us from this scheduled fire.
    return static_cast<std::int64_t>(self->nlp_.next_interval_ms()) * 1000;
}

} // namespace pico_ethernet
