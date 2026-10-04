// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/phy.h"

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
#include "src/phy/rx_frame_end.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/tx_level.h"
#include "src/util/single_writer.h"

#include "carrier_qualify.pio.h"
#include "rx.pio.h"
#include "tx_level.pio.h"

namespace pico_ethernet {

// Everything the interrupt handlers and the masked transmit step run is placed in
// RAM (__not_in_flash_func). From flash, cache misses stretch them by microseconds,
// long enough to hold the carrier interrupt past the gap between two frames.

namespace {
// The GPIO IRQ callback carries no user data, so the active PHY is published here
// for the carrier and TX-done trampolines to forward to.
Phy* s_irq_phy{nullptr};

// The qualification loop in carrier_qualify.pio samples RXC once per iteration and
// runs its loaded count plus one times.
constexpr std::uint32_t QUALIFY_CYCLES_PER_ITERATION{2};
constexpr std::uint32_t QUALIFY_LOOP_COUNT{CARRIER_QUALIFY_US * SYS_CYCLES_PER_US / QUALIFY_CYCLES_PER_ITERATION - 1};

// A colliding station always completes its preamble and SFD before it jams.
static_assert(CARRIER_QUALIFY_US * 1000 < (PREAMBLE_SFD_LEN * 8 + JAM_BITS) * BIT_TIME_NS);

static_assert(tx_level_TP_IDL_CYCLES == PIO_CYCLES_PER_TP_IDL);

// The qualifier and the transmitter each raise the interrupt flag of their own
// state machine's index; the qualification goes to the PIO's interrupt line 0, the
// end of a transmission to line 1.
constexpr std::uint32_t QUALIFY_IRQ_INDEX{0};
constexpr std::uint32_t TX_DONE_IRQ_INDEX{1};

pio_interrupt_source_t __not_in_flash_func(sm_interrupt_source)(std::uint32_t sm) {
    return static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm);
}

// The latest received code word shares one atomic word with a flag for whether it
// was consecutive and a sequence number, which tells the loop a new code word from
// the last one it saw, and a skipped one -- overwritten before the loop got to it --
// from the next.
constexpr std::uint32_t CODE_WORD_CONSECUTIVE{1u << 16};
constexpr std::uint32_t CODE_WORD_SEQUENCE_SHIFT{17};
constexpr std::uint32_t CODE_WORD_SEQUENCE_MASK{(1u << (32 - CODE_WORD_SEQUENCE_SHIFT)) - 1};

// RP2350-E5: a channel aborted while still enabled can re-trigger, so it is disabled
// first. Whatever starts the channel again has to enable it again.
void __not_in_flash_func(abort_dma_channel)(std::uint32_t channel) {
    hw_clear_bits(&dma_channel_hw_addr(channel)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(channel);
}
} // namespace

Phy::Phy(std::uint32_t backoff_seed, const Pins& pins)
    : pins_{pins},
      csma_{backoff_seed},
      autoneg_{time_us_32()} {
    // TXP/TXN must be adjacent so the TX SM can drive them as one 2-pin SET
    // group; RXD/RXC must be adjacent so the RX SM can gate on RXC via IN base + 1.
    assert(pins_.txn == pins_.txp + 1);
    assert(pins_.rxc == pins_.rxd + 1);

    jam_.fill(JAM_OCTET);
}

bool Phy::initialize() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);

    if (!pio_can_add_program(pio_, &tx_level_program) || !pio_can_add_program(pio_, &carrier_qualify_program) ||
        !pio_can_add_program(rx_pio_, &rx_manchester_program) ||
        !pio_can_add_program(rx_pio_, &tp_idl_watchdog_program)) {
        return false;
    }
    offset_tx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_level_program));
    offset_qualify_ = static_cast<std::uint32_t>(pio_add_program(pio_, &carrier_qualify_program));
    offset_rx_ = static_cast<std::uint32_t>(pio_add_program(rx_pio_, &rx_manchester_program));
    offset_watchdog_ = static_cast<std::uint32_t>(pio_add_program(rx_pio_, &tp_idl_watchdog_program));

    const std::int32_t smt{pio_claim_unused_sm(pio_, false)};
    const std::int32_t smq{pio_claim_unused_sm(pio_, false)};
    const std::int32_t smr{pio_claim_unused_sm(rx_pio_, false)};
    const std::int32_t smw{pio_claim_unused_sm(rx_pio_, false)};
    if (smt < 0 || smq < 0 || smr < 0 || smw < 0) {
        return false;
    }
    sm_tx_ = static_cast<std::uint32_t>(smt);
    sm_qualify_ = static_cast<std::uint32_t>(smq);
    sm_rx_ = static_cast<std::uint32_t>(smr);
    sm_watchdog_ = static_cast<std::uint32_t>(smw);

    dma_tx_ = dma_claim_unused_channel(false);
    dma_rx_ = dma_claim_unused_channel(false);
    if (dma_tx_ < 0 || dma_rx_ < 0) {
        return false;
    }

    configure_tx();
    configure_rx();
    configure_carrier_qualify();
    rearm_capture(rx_ring_.capture_slot());

    // Both carrier edges matter: the rising edge times a link pulse; the falling edge
    // is end of frame, and starts the interframe gap.
    s_irq_phy = this;
    gpio_set_irq_enabled_with_callback(
        pins_.rxc, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &Phy::carrier_irq_handler);

    // Finalize each transmit (outcome, interframe gap) as soon as the TX state
    // machine has released the line, rather than waiting for the main loop. Its flag
    // reaches the CPU only while a transmission is under way: start_tx() enables the
    // source and on_tx_complete() disables it.
    const std::uint32_t tx_done_irq{static_cast<std::uint32_t>(pio_get_irq_num(pio_, TX_DONE_IRQ_INDEX))};
    irq_set_exclusive_handler(tx_done_irq, &Phy::tx_done_irq_handler);
    irq_set_enabled(tx_done_irq, true);

    // The qualifier runs all the time, but its flag only reaches the CPU while we
    // transmit in half duplex: start_tx() enables the source and on_tx_complete()
    // disables it.
    const std::uint32_t qualify_irq{static_cast<std::uint32_t>(pio_get_irq_num(pio_, QUALIFY_IRQ_INDEX))};
    irq_set_exclusive_handler(qualify_irq, &Phy::carrier_qualified_irq_handler);
    irq_set_enabled(qualify_irq, true);

    link_pulse_alarm_ = add_alarm_in_ms(NLP_RETRY_MS, &Phy::link_pulse_alarm_cb, this, true);
    return true;
}

void Phy::configure_tx() {
    pio_gpio_init(pio_, pins_.txp);
    pio_gpio_init(pio_, pins_.txn);

    pio_sm_set_consecutive_pindirs(pio_, sm_tx_, pins_.txp, 2, true);

    // SET group {TXP,TXN}; autopull the raw frame LSB-first (802.3), one octet per
    // FIFO entry (8-bit DMA -> one byte per pull). /1 clock = 12 cycles/bit at
    // 120 MHz.
    tx_cfg_ = tx_level_program_get_default_config(offset_tx_);
    sm_config_set_set_pins(&tx_cfg_, pins_.txp, 2);
    sm_config_set_out_shift(&tx_cfg_, true, true, 8);
    sm_config_set_clkdiv(&tx_cfg_, 1.0f);
    pio_sm_init(pio_, sm_tx_, offset_tx_, &tx_cfg_);
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

void Phy::configure_rx() {
    // The comparators drive RXD/RXC externally. gpio_init de-isolates the pads
    // (RP2350 resets pads isolated, which gpio_set_input_enabled does not clear)
    // and enables their inputs without driving them; PIO reads a pad's input
    // independently of its function select, and the carrier interrupt, its
    // spurious-edge guard and the carrier-sense check all read the same input via
    // SIO.
    gpio_init(pins_.rxd);
    gpio_init(pins_.rxc);

    rx_cfg_ = rx_manchester_config(offset_rx_, pins_.rxd);
    start_tp_idl_watchdog(rx_pio_, sm_watchdog_, offset_watchdog_, pins_.rxd);
    enable_rx_sniffer(dma_rx_);
}

void Phy::configure_carrier_qualify() {
    pio_sm_config cfg{carrier_qualify_program_get_default_config(offset_qualify_)};
    sm_config_set_in_pins(&cfg, pins_.rxc);
    sm_config_set_jmp_pin(&cfg, pins_.rxc);
    sm_config_set_clkdiv(&cfg, 1.0f);
    pio_sm_init(pio_, sm_qualify_, offset_qualify_, &cfg);

    // The window stays in the OSR for the program to reload X from on every assert.
    pio_sm_put(pio_, sm_qualify_, QUALIFY_LOOP_COUNT);
    pio_sm_exec(pio_, sm_qualify_, pio_encode_pull(false, false));
    pio_sm_set_enabled(pio_, sm_qualify_, true);
}

void __not_in_flash_func(Phy::rearm_capture)(std::size_t slot) {
    restart_rx(rx_pio_, sm_rx_, offset_rx_, rx_cfg_);
    start_rx_capture(dma_rx_, rx_pio_, sm_rx_, rx_pool_[slot]);
    pio_sm_set_enabled(rx_pio_, sm_rx_, true);
}

void __not_in_flash_func(Phy::carrier_irq_handler)(uint gpio, std::uint32_t events) {
    (void)gpio;
    if (s_irq_phy == nullptr) {
        return;
    }
    // A pulse short enough to latch both edges before the handler runs is reported
    // as both; take them in the order they physically occurred.
    if ((events & GPIO_IRQ_EDGE_RISE) != 0u) {
        s_irq_phy->on_carrier_assert();
    }
    if ((events & GPIO_IRQ_EDGE_FALL) != 0u) {
        s_irq_phy->on_rx_eof();
    }
}

void __not_in_flash_func(Phy::on_carrier_assert)() { carrier_asserted_us_ = time_us_32(); }

void __not_in_flash_func(Phy::carrier_qualified_irq_handler)() {
    if (s_irq_phy != nullptr) {
        s_irq_phy->on_carrier_qualified();
    }
}

void __not_in_flash_func(Phy::on_carrier_qualified)() {
    pio_interrupt_clear(pio_, sm_qualify_);
    carrier_qualified_.store(true, std::memory_order_relaxed);

    // A peer transmitting while we are is a collision -- and only the first one of
    // an attempt is. The flag can still arrive after on_tx_complete() has disabled
    // its source, when it was raised just before.
    if (active_.load(std::memory_order_relaxed) && !collided_.load(std::memory_order_relaxed)) {
        on_collision();
    }
}

void __not_in_flash_func(Phy::on_collision)() {
    collision_elapsed_us_.store(
        collision_offset_us(tx_start_us_.load(std::memory_order_relaxed), time_us_32()), std::memory_order_relaxed);
    collided_.store(true, std::memory_order_relaxed);

    abort_dma_channel(dma_tx_);
    hw_set_bits(&dma_channel_hw_addr(dma_tx_)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);

    // The FIFO is deliberately left alone. The DMA runs octets ahead of the wire, so
    // what it holds is the part of the frame not yet transmitted -- letting it drain
    // ahead of the jam keeps the jam from ever preceding the preamble onto the line,
    // and splices at an octet boundary instead of mid-symbol. The jam reaches the FIFO
    // long before it drains, so the transmitter never finds its data ended in between.
    dma_channel_set_read_addr(dma_tx_, jam_.data(), false);
    dma_channel_set_trans_count(dma_tx_, jam_.size(), true);
}

void __not_in_flash_func(Phy::on_rx_eof)() {
    // RXC is high again: a dip in the carrier envelope, or the next frame's carrier
    // already rising by the time this handler runs. The two look the same here, so
    // the capture goes on. A frame that already ended on TP_IDL waits, finished, for
    // the next carrier's end, and the frame in that carrier is lost.
    if (gpio_get(pins_.rxc)) {
        return;
    }
    note_medium_free();

    const bool frame_ended{pio_interrupt_get(rx_pio_, RX_DONE_FLAG)};
    const bool qualified{
        pio_interrupt_get(pio_, sm_qualify_) || carrier_qualified_.exchange(false, std::memory_order_relaxed)};
    pio_interrupt_clear(pio_, sm_qualify_);

    // The RX SM has stalled and DMA has drained every complete word, so the
    // remaining count is stable and readable without stopping the DMA.
    const std::size_t landed{RX_WORD_CAPACITY - dma_channel_hw_addr(dma_rx_)->transfer_count};

    // No SFD: the SM pushes nothing before it, and a frame that ends on TP_IDL
    // pushes at least its padded last word. A carrier blip is a link pulse, or line
    // noise the pulse decoder rejects; a carrier that outlasted one is a glitch.
    if (!frame_ended && landed == 0) {
        if (qualified) {
            increment_single_writer(rx_glitches_);
            increment_single_writer(rx_captures_);
        } else {
            on_link_pulse();
        }
        discard_capture();
        return;
    }

    // A frame received while the link is down counts, and can restore the link, but
    // is not delivered: it arrived under the link state the loop last published.
    increment_single_writer(rx_captures_);
    if (!link_up_mirror_.load(std::memory_order_relaxed)) {
        discard_capture();
        return;
    }

    // A capture that ran the buffer full is a giant whether or not it reached
    // TP_IDL; the drain tells it by its length. Any other one that did not reach
    // TP_IDL lost its carrier mid-frame.
    if (!frame_ended && landed < RX_WORD_CAPACITY) {
        increment_single_writer(rx_truncated_);
        discard_capture();
        return;
    }

    while (!pio_sm_is_rx_fifo_empty(rx_pio_, sm_rx_) && dma_channel_hw_addr(dma_rx_)->transfer_count != 0) {
        tight_loop_contents(); // let the DMA carry the padded last word into the buffer
    }

    // Read while the channel is still live: an aborted channel reports no transfers
    // remaining, whatever it had left to do.
    const std::size_t words{RX_WORD_CAPACITY - dma_channel_hw_addr(dma_rx_)->transfer_count};
    const std::uint32_t crc{dma_sniffer_get_data_accumulator()};
    abort_dma_channel(dma_rx_);

    // Hand the DMA a free buffer for the next frame and queue this one for the
    // main-loop drain.
    rearm_capture(rx_ring_.publish(words, crc));
}

void __not_in_flash_func(Phy::discard_capture)() {
    abort_dma_channel(dma_rx_);
    rearm_capture(rx_ring_.capture_slot());
}

void __not_in_flash_func(Phy::on_link_pulse)() {
    const LinkPulseEvent event{pulse_decoder_.on_pulse(carrier_asserted_us_)};
    switch (event.kind) {
        case LinkPulseEvent::Kind::None:
            return;
        case LinkPulseEvent::Kind::ValidNlp:
            increment_single_writer(valid_nlps_);
            return;
        case LinkPulseEvent::Kind::CodeWord: {
            const std::uint32_t sequence{
                (code_word_slot_.load(std::memory_order_relaxed) >> CODE_WORD_SEQUENCE_SHIFT) + 1};
            code_word_slot_.store(
                (sequence << CODE_WORD_SEQUENCE_SHIFT) | (event.consecutive ? CODE_WORD_CONSECUTIVE : 0u) |
                    event.code_word,
                std::memory_order_relaxed);
            return;
        }
    }
}

Phy::RxFrame Phy::poll_rx() {
    const std::optional<RxSlotRing<RX_POOL_SIZE>::Completed> done{rx_ring_.peek()};
    if (!done) {
        return {};
    }

    // The DMA'd words pack four frame-aligned octets each, LSB-first, so on this
    // little-endian core the capture buffer holds the frame in order.
    const std::expected<std::size_t, FrameError> length{
        find_frame_end(done->word_count * RX_OCTETS_PER_WORD, done->crc)};
    if (!length) {
        return {.kind = RxFrame::Kind::Error, .error = length.error()};
    }
    return {.kind = RxFrame::Kind::Frame, .frame = std::span<const std::uint8_t>{rx_pool_[done->slot]}.first(*length)};
}

WireFrame* Phy::tx_slot() {
    if (tx_advance(tx_head_) == tx_tail_) {
        return nullptr;
    }
    return &tx_queue_[tx_head_];
}

void Phy::commit_tx() {
    assert(tx_advance(tx_head_) != tx_tail_); // a slot was claimed
    assert(tx_queue_[tx_head_].length > 0 && tx_queue_[tx_head_].length <= WIRE_CAPACITY);
    tx_head_ = tx_advance(tx_head_);
}

void __not_in_flash_func(Phy::start_tx)(const WireFrame& frame) {
    // Reset the SM to the program start with a cleared FIFO and empty OSR so the
    // frame's first bit is shifted from the start of its first octet. The previous
    // transmission left the machine raising its end flag, which is cleared before its
    // source is enabled.
    pio_sm_set_enabled(pio_, sm_tx_, false);
    pio_sm_clear_fifos(pio_, sm_tx_);
    pio_sm_init(pio_, sm_tx_, offset_tx_, &tx_cfg_);
    pio_interrupt_clear(pio_, sm_tx_);

    dma_channel_config c{dma_channel_get_default_config(dma_tx_)};
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, true);
    channel_config_set_write_increment(&c, false);
    channel_config_set_dreq(&c, pio_get_dreq(pio_, sm_tx_, true));
    dma_channel_configure(dma_tx_, &c, &pio_->txf[sm_tx_], frame.bytes.data(), frame.length, false);

    // In half duplex a qualified carrier while we transmit is a peer talking over
    // us, which is exactly the collision we must catch. Its interrupt takes active_
    // as its cue to abort and jam, so the flag is raised behind the caller's mask,
    // together with the machine and the transfer it would be aborting -- a collision
    // seen between them would re-point the transfer at the jam while the machine is
    // still stopped, leaving octets in a FIFO that nothing will drain. The caller has
    // just seen the carrier low, so a raised qualification flag belongs to a carrier
    // that has already ended and is cleared; one qualifying under the mask is
    // delivered when it lifts.
    //
    // In full duplex the peer's carrier is on the other pair and never a collision.
    // The qualifier keeps running regardless; its interrupt is simply never enabled.
    ++tx_attempts_;
    tx_start_us_.store(time_us_32(), std::memory_order_relaxed);
    active_.store(true, std::memory_order_relaxed);
    if (duplex_.load(std::memory_order_relaxed) == Duplex::Half) {
        pio_interrupt_clear(pio_, sm_qualify_);
        pio_set_irqn_source_enabled(pio_, QUALIFY_IRQ_INDEX, sm_interrupt_source(sm_qualify_), true);
    }
    pio_set_irqn_source_enabled(pio_, TX_DONE_IRQ_INDEX, sm_interrupt_source(sm_tx_), true);
    pio_sm_set_enabled(pio_, sm_tx_, true);
    dma_channel_start(dma_tx_);
}

bool __not_in_flash_func(Phy::transmitting)() const { return active_.load(std::memory_order_relaxed); }

void Phy::service() {
    advance_link(time_us_32());
    if (attempt_in_flight_ && active_.load(std::memory_order_acquire)) {
        return; // still on the wire
    }

    // Sensing the medium and starting on what was sensed are one step: an interrupt
    // between them would let the frame start on a carrier reading and an interframe
    // gap that no longer hold -- onto a peer's frame already under way, whose carrier
    // then raises no edge and no fresh qualification to flag the collision.
    const std::uint32_t irq_state{save_and_disable_interrupts()};
    advance_tx();
    restore_interrupts(irq_state);
}

void __not_in_flash_func(Phy::advance_tx)() {
    // The carrier flag the interrupts maintain is the timestamp of the last
    // deassert; the pin itself is the authority on the present state. Reading it
    // here means a lost edge cannot leave the transmitter deferring forever.
    csma_.observe_medium(gpio_get(pins_.rxc), medium_free_us_.load(std::memory_order_relaxed));

    if (attempt_in_flight_) {
        attempt_in_flight_ = false;
        resolve_attempt();
    }

    if (tx_tail_ == tx_head_) {
        return;
    }

    // A link that is down has no medium to contend for, so queued frames go nowhere:
    // discard them one per lap, the same place a frame would otherwise be started.
    if (!link_up()) {
        ++csma_stats_.link_down_dropped;
        release_frame();
        return;
    }

    const std::uint32_t now_us{time_us_32()};
    if (!csma_.may_transmit(now_us)) {
        csma_.on_held_back(now_us);
        return;
    }
    start_tx(tx_queue_[tx_tail_]);
    attempt_in_flight_ = true;
}

void Phy::advance_link(std::uint32_t now_us) {
    const bool was_up{link_up()};

    // More pulses than it takes to pass the link integrity test tell it nothing new,
    // so a backlog collapses into that many and the loop stays bounded.
    const std::uint32_t nlps{valid_nlps_.load(std::memory_order_relaxed)};
    const std::uint32_t pending_nlps{nlps - valid_nlps_seen_};
    valid_nlps_seen_ = nlps;
    for (std::uint32_t i{0}; i < pending_nlps && i < LINK_UP_EVENTS; ++i) {
        link_.on_pulse(now_us);
    }
    const std::uint32_t captures{rx_captures_.load(std::memory_order_relaxed)};
    if (captures != rx_captures_seen_) {
        rx_captures_seen_ = captures;
        link_.on_frame(now_us);
    }
    link_.advance(now_us);

    const std::uint32_t slot{code_word_slot_.load(std::memory_order_relaxed)};
    const std::uint32_t sequence{slot >> CODE_WORD_SEQUENCE_SHIFT};
    if (sequence != code_word_sequence_seen_) {
        const bool next{sequence == ((code_word_sequence_seen_ + 1) & CODE_WORD_SEQUENCE_MASK)};
        code_word_sequence_seen_ = sequence;
        autoneg_.on_code_word(static_cast<std::uint16_t>(slot), next && (slot & CODE_WORD_CONSECUTIVE) != 0, now_us);
    }
    const std::uint32_t bursts{flp_bursts_sent_.load(std::memory_order_relaxed)};
    for (; flp_bursts_seen_ != bursts; ++flp_bursts_seen_) {
        autoneg_.on_burst_sent(now_us);
    }
    autoneg_.advance(now_us, link_.up());

    link_pulses_.store(autoneg_.link_pulses(), std::memory_order_relaxed);
    code_word_.store(autoneg_.code_word(), std::memory_order_relaxed);

    const std::optional<Duplex> link{autoneg_.link()};
    if (link.has_value() != was_up) {
        ++csma_stats_.link_transitions;
        if (link) {
            // The duplex only changes as the link comes up, when no attempt can be
            // under way under the other duplex's rules.
            assert(!attempt_in_flight_);
            csma_.set_duplex(*link);
            duplex_.store(*link, std::memory_order_relaxed);
        }
    }
    link_up_mirror_.store(link.has_value(), std::memory_order_relaxed);
}

void __not_in_flash_func(Phy::resolve_attempt)() {
    if (!collided_.exchange(false, std::memory_order_relaxed)) {
        if (csma_.collisions() == 1) {
            ++csma_stats_.single_collision;
        } else if (csma_.collisions() > 1) {
            ++csma_stats_.multiple_collisions;
        }
        release_frame();
        return;
    }

    switch (csma_.on_collision(collision_elapsed_us_.load(std::memory_order_relaxed))) {
        case CsmaCd::Collision::Backoff:
            return; // the frame keeps its place in the queue for the retry
        case CsmaCd::Collision::Excessive:
            ++csma_stats_.excessive_collisions;
            break;
        case CsmaCd::Collision::Late:
            ++csma_stats_.late_collisions;
            break;
    }
    release_frame();
}

void __not_in_flash_func(Phy::release_frame)() {
    if (csma_.deferred()) {
        ++csma_stats_.deferred;
    }
    csma_.on_frame_done();
    tx_tail_ = tx_advance(tx_tail_);
}

void __not_in_flash_func(Phy::note_medium_free)() { medium_free_us_.store(time_us_32(), std::memory_order_relaxed); }

void __not_in_flash_func(Phy::tx_done_irq_handler)() {
    if (s_irq_phy != nullptr) {
        s_irq_phy->on_tx_complete();
    }
}

void __not_in_flash_func(Phy::on_tx_complete)() {
    // The machine keeps raising its flag until it is restarted.
    pio_set_irqn_source_enabled(pio_, TX_DONE_IRQ_INDEX, sm_interrupt_source(sm_tx_), false);

    // Data still queued once the machine has ended the transmission came too late
    // for it: the FIFO ran dry mid-frame, and the frame ended on the wire there.
    const bool underrun{dma_channel_is_busy(dma_tx_) || !pio_sm_is_tx_fifo_empty(pio_, sm_tx_)};
    if (underrun) {
        abort_dma_channel(dma_tx_);
    }

    // A collided attempt ends in a jam, not a frame: it is neither sent nor underrun.
    if (!collided_.load(std::memory_order_relaxed)) {
        increment_single_writer(underrun ? tx_underrun_ : tx_sent_);
    }

    pio_set_irqn_source_enabled(pio_, QUALIFY_IRQ_INDEX, sm_interrupt_source(sm_qualify_), false);
    note_medium_free();
    active_.store(false, std::memory_order_release);
}

void __not_in_flash_func(Phy::emit_pulse)() {
    // A single ~100 ns positive excursion.
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_POS));
    busy_wait_at_least_cycles(PIO_CYCLES_PER_BIT);
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

bool __not_in_flash_func(Phy::emit_nlp)() {
    // Only onto a medium that is free: a pulse laid over a frame corrupts it. That is
    // our own frame, and in half duplex also the peer's. In full duplex the peer's
    // frames are on the other pair, and holding back for them would starve our pulses
    // under a one-way flood; its end of frame still restarts the gap, which delays a
    // pulse by no more than the gap.
    const bool peer_carrier{duplex_.load(std::memory_order_relaxed) == Duplex::Half && gpio_get(pins_.rxc)};
    if (transmitting() || peer_carrier) {
        return false;
    }
    if (time_us_32() - medium_free_us_.load(std::memory_order_relaxed) < IFG_US) {
        return false;
    }
    emit_pulse();
    note_medium_free();
    return true;
}

std::uint32_t __not_in_flash_func(Phy::emit_link_pulses)() {
    // A burst under way runs to its end whatever the arbitration has moved on to, so
    // the peer never sees a truncated one.
    if (!flp_burst_) {
        switch (link_pulses_.load(std::memory_order_relaxed)) {
            case Autoneg::LinkPulses::None:
                return NLP_RETRY_MS * 1000;
            case Autoneg::LinkPulses::Nlp:
                // A pulse held back for a busy medium is retried far sooner than the
                // cadence, so traffic barely shifts the pulse train the peer's link
                // timer is watching.
                return (emit_nlp() ? nlp_.next_interval_ms() : NLP_RETRY_MS) * 1000;
            case Autoneg::LinkPulses::Flp:
                break;
        }
    }
    FlpBurst& burst{flp_burst_ ? *flp_burst_ : flp_burst_.emplace(code_word_.load(std::memory_order_relaxed))};

    // FLP pulses are never held back: they only go out while the link is down, when
    // nothing is transmitted, and the peer's pulses are on the other pair.
    emit_pulse();
    if (const std::optional<std::uint32_t> next_us{burst.next_pulse_us()}) {
        return *next_us;
    }
    flp_burst_.reset();
    increment_single_writer(flp_bursts_sent_);
    return FLP_BURST_PERIOD_US - FLP_BURST_US;
}

std::int64_t __not_in_flash_func(Phy::link_pulse_alarm_cb)(alarm_id_t /*id*/, void* user) {
    // Positive return = us from this scheduled fire, so pulse spacing does not drift
    // with the callback's own latency.
    return static_cast<Phy*>(user)->emit_link_pulses();
}

} // namespace pico_ethernet
