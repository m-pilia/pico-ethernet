// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/phy.h"

#include <algorithm>
#include <cassert>

#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "src/phy/phy_timing.h"
#include "src/phy/tx_emphasis.h"

#include "tx_emphasis.pio.h"
#include "tx_level.pio.h"

namespace pico_ethernet {

Phy::Phy(const Pins& pins)
    : pins_{pins} {
    // TXP/TXN must be adjacent so the LEVEL SM can drive them as one 2-pin SET
    // group.
    assert(pins_.txn == pins_.txp + 1);
}

bool Phy::initialize() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);

    if (!pio_can_add_program(pio_, &tx_level_program) || !pio_can_add_program(pio_, &tx_emphasis_program)) {
        return false;
    }
    offset_level_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_level_program));
    offset_emphasis_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_emphasis_program));

    const int sml{pio_claim_unused_sm(pio_, false)};
    const int sme{pio_claim_unused_sm(pio_, false)};
    if (sml < 0 || sme < 0) {
        return false;
    }
    sm_level_ = static_cast<std::uint32_t>(sml);
    sm_emphasis_ = static_cast<std::uint32_t>(sme);

    dma_level_ = dma_claim_unused_channel(false);
    dma_emphasis_ = dma_claim_unused_channel(false);
    if (dma_level_ < 0 || dma_emphasis_ < 0) {
        return false;
    }

    configure_state_machines();
    force_idle();

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
    }
}

void Phy::force_idle() {
    // Drive the differential pair to 0 V (TP_IDL then line idle). Injected while
    // the SM is stalled on an empty FIFO, so the level holds until the next frame.
    pio_sm_exec(pio_, sm_level_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

void Phy::emit_nlp() {
    // A single ~100 ns positive excursion. Skip while a frame is in flight so the
    // pulse never corrupts data. The exact width is trimmed on-target against the
    // scope.
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
