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
#include "src/phy/rx_frame_recover.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/tx_level.h"

#include "carrier_qualify.pio.h"
#include "rx.pio.h"
#include "tx_level.pio.h"

namespace pico_ethernet {

// Everything the interrupt handlers and the masked transmit step run is placed in
// RAM (__not_in_flash_func). From flash, cache misses stretch them by microseconds,
// long enough to hold the carrier interrupt past the gap between two frames.

namespace {
// The GPIO IRQ callback carries no user data, so the active PHY is published here
// for the carrier and TX-DMA trampolines to forward to.
Phy* s_irq_phy{nullptr};

// The qualification loop in carrier_qualify.pio samples RXC once per iteration and
// runs its loaded count plus one times.
constexpr std::uint32_t QUALIFY_CYCLES_PER_ITERATION{2};
constexpr std::uint32_t QUALIFY_LOOP_COUNT{
    CARRIER_QUALIFY_US * (SYS_CLOCK_HZ / 1'000'000) / QUALIFY_CYCLES_PER_ITERATION - 1};

// A colliding station always completes its preamble and SFD before it jams.
static_assert(CARRIER_QUALIFY_US * 1000 < (PREAMBLE_SFD_LEN * 8 + JAM_BITS) * BIT_TIME_NS);

// RP2350-E5: a channel aborted while still enabled can re-trigger, so it is disabled
// first. Whatever starts the channel again has to enable it again.
void __not_in_flash_func(abort_dma_channel)(uint channel) {
    hw_clear_bits(&dma_channel_hw_addr(channel)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);
    dma_channel_abort(channel);
}
} // namespace

Phy::Phy(std::uint32_t backoff_seed, const Pins& pins)
    : pins_{pins},
      csma_{backoff_seed} {
    // TXP/TXN must be adjacent so the TX SM can drive them as one 2-pin SET
    // group; RXD/RXC must be adjacent so the RX SM can gate on RXC via IN base + 1.
    assert(pins_.txn == pins_.txp + 1);
    assert(pins_.rxc == pins_.rxd + 1);

    jam_.fill(JAM_OCTET);
}

bool Phy::initialize() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);

    if (!pio_can_add_program(pio_, &tx_level_program) || !pio_can_add_program(pio_, &rx_manchester_program) ||
        !pio_can_add_program(pio_, &carrier_qualify_program)) {
        return false;
    }
    offset_tx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &tx_level_program));
    offset_rx_ = static_cast<std::uint32_t>(pio_add_program(pio_, &rx_manchester_program));
    offset_qualify_ = static_cast<std::uint32_t>(pio_add_program(pio_, &carrier_qualify_program));

    const int smt{pio_claim_unused_sm(pio_, false)};
    const int smr{pio_claim_unused_sm(pio_, false)};
    const int smq{pio_claim_unused_sm(pio_, false)};
    if (smt < 0 || smr < 0 || smq < 0) {
        return false;
    }
    sm_tx_ = static_cast<std::uint32_t>(smt);
    sm_rx_ = static_cast<std::uint32_t>(smr);
    sm_qualify_ = static_cast<std::uint32_t>(smq);

    dma_tx_ = dma_claim_unused_channel(false);
    dma_rx_ = dma_claim_unused_channel(false);
    if (dma_tx_ < 0 || dma_rx_ < 0) {
        return false;
    }

    configure_tx();
    configure_rx();
    configure_carrier_qualify();
    force_idle();
    rearm_capture(rx_ring_.capture_slot());

    // Both carrier edges matter: the rising edge is link activity; the falling edge
    // is end of frame, and starts the interframe gap.
    s_irq_phy = this;
    gpio_set_irq_enabled_with_callback(
        pins_.rxc, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &Phy::carrier_irq_handler);

    // Finalize each transmit (idle drive, interframe gap) in the TX-DMA completion
    // interrupt so the line is released the instant our frame drains, rather than
    // waiting for the main loop.
    dma_channel_set_irq0_enabled(dma_tx_, true);
    irq_set_exclusive_handler(DMA_IRQ_0, &Phy::tx_dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // The qualifier runs all the time, but its flag only reaches the CPU while we
    // transmit: start_tx() enables the source and on_tx_complete() disables it.
    const auto qualify_irq{static_cast<uint>(pio_get_irq_num(pio_, 0))};
    irq_set_exclusive_handler(qualify_irq, &Phy::carrier_qualified_irq_handler);
    irq_set_enabled(qualify_irq, true);

    nlp_alarm_ = add_alarm_in_ms(nlp_.next_interval_ms(), &Phy::nlp_alarm_cb, this, true);
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

    rx_cfg_ = rx_manchester_program_get_default_config(offset_rx_);
    configure_rx_shift(rx_cfg_, pins_.rxd);
    pio_sm_init(pio_, sm_rx_, offset_rx_, &rx_cfg_);
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

pio_interrupt_source_t __not_in_flash_func(Phy::qualify_interrupt_source)() const {
    // The program raises its flag relative to its own state machine.
    return static_cast<pio_interrupt_source_t>(pis_interrupt0 + sm_qualify_);
}

void __not_in_flash_func(Phy::rearm_capture)(std::size_t slot) {
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

void __not_in_flash_func(Phy::on_carrier_assert)() {
    // Every carrier assert is evidence the peer is alive, whether it goes on to
    // carry a frame or is just a link pulse.
    carrier_events_.fetch_add(1, std::memory_order_relaxed);
}

void __not_in_flash_func(Phy::carrier_qualified_irq_handler)() {
    if (s_irq_phy != nullptr) {
        s_irq_phy->on_carrier_qualified();
    }
}

void __not_in_flash_func(Phy::on_carrier_qualified)() {
    pio_interrupt_clear(pio_, sm_qualify_);

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

    // Aborting a channel mid-transfer can leave a completion flagged; masking the
    // channel's interrupt across the abort and clearing it afterwards keeps the
    // aborted frame from being mistaken for a finished one.
    dma_channel_set_irq0_enabled(dma_tx_, false);
    abort_dma_channel(dma_tx_);
    dma_channel_acknowledge_irq0(dma_tx_);
    dma_channel_set_irq0_enabled(dma_tx_, true);
    hw_set_bits(&dma_channel_hw_addr(dma_tx_)->al1_ctrl, DMA_CH0_CTRL_TRIG_EN_BITS);

    // The FIFO is deliberately left alone. The DMA runs octets ahead of the wire, so
    // what it holds is the part of the frame not yet transmitted -- letting it drain
    // ahead of the jam keeps the jam from ever preceding the preamble onto the line,
    // and splices at an octet boundary instead of mid-symbol.
    dma_channel_set_read_addr(dma_tx_, jam_.data(), false);
    dma_channel_set_trans_count(dma_tx_, jam_.size(), true);
}

void __not_in_flash_func(Phy::on_rx_eof)() {
    // RXC is high again: a dip in the carrier envelope, or the next frame's carrier
    // already rising by the time this handler runs. The two look the same here, so
    // the capture goes on; in the second case the next frame is merged into it and
    // lost.
    if (gpio_get(pins_.rxc)) {
        return;
    }
    note_medium_free();

    // The RX SM has stalled on the missing edges and DMA has drained every complete
    // word, so the remaining count is stable and readable without stopping the DMA.
    const std::uint32_t remaining_before{dma_channel_hw_addr(dma_rx_)->transfer_count};
    const std::size_t words_before{RX_WORD_CAPACITY - remaining_before};

    // A carrier blip that decoded nothing -- a link pulse, or line noise -- is
    // discarded; the capture keeps the same slot.
    if (words_before == 0) {
        abort_dma_channel(dma_rx_);
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

    // Read while the channel is still live, which the flush has left stable: an
    // aborted channel reports no transfers remaining, whatever it had left to do.
    const std::uint32_t remaining{dma_channel_hw_addr(dma_rx_)->transfer_count};
    abort_dma_channel(dma_rx_);
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

    // A qualified carrier while we transmit is a peer talking over us, which is
    // exactly the collision we must catch. Its interrupt takes active_ as its cue to
    // abort and jam, so the flag is raised behind the caller's mask, together with
    // the machine and the transfer it would be aborting -- a collision seen between
    // them would re-point the transfer at the jam while the machine is still stopped,
    // leaving octets in a FIFO that nothing will drain. The caller has just seen the
    // carrier low, so a raised qualification flag belongs to a carrier that has
    // already ended and is cleared; one qualifying under the mask is delivered when
    // it lifts.
    //
    // The SM stalls on autopull until the first octet lands, which latches TXSTALL.
    // Clear it once that octet is delivered so a TXSTALL seen in on_tx_complete()
    // means a mid-frame FIFO underrun. The mask also keeps a long ISR from delaying
    // that clear past the end of the frame.
    tx_start_us_.store(time_us_32(), std::memory_order_relaxed);
    active_.store(true, std::memory_order_relaxed);
    pio_interrupt_clear(pio_, sm_qualify_);
    pio_set_irq0_source_enabled(pio_, qualify_interrupt_source(), true);
    pio_sm_set_enabled(pio_, sm_tx_, true);
    dma_channel_start(dma_tx_);
    while (dma_channel_hw_addr(dma_tx_)->transfer_count == frame.length) {
        tight_loop_contents();
    }
    pio_->fdebug = tx_stall_mask();
}

std::uint32_t __not_in_flash_func(Phy::tx_stall_mask)() const { return 1u << (PIO_FDEBUG_TXSTALL_LSB + sm_tx_); }

bool __not_in_flash_func(Phy::transmitting)() const {
    if (dma_tx_ < 0) {
        return false;
    }
    return dma_channel_is_busy(dma_tx_) || !pio_sm_is_tx_fifo_empty(pio_, sm_tx_);
}

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
        return; // queue empty
    }

    // A link that is down has no medium to contend for, so queued frames go nowhere:
    // discard them one per lap, the same place a frame would otherwise be started.
    if (!link_.up()) {
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
    const bool was_up{link_.up()};

    // More activity than it takes to declare the link up tells the FSM nothing new,
    // so a burst collapses into that many events and the loop stays bounded.
    const std::uint32_t events{carrier_events_.load(std::memory_order_relaxed)};
    const std::uint32_t pending{events - carrier_events_seen_};
    carrier_events_seen_ = events;
    for (std::uint32_t i{0}; i < pending && i < LINK_UP_EVENTS; ++i) {
        link_.on_activity(now_us);
    }

    link_.advance(now_us);
    if (link_.up() != was_up) {
        ++csma_stats_.link_transitions;
    }
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

void __not_in_flash_func(Phy::tx_dma_irq_handler)() {
    if (s_irq_phy != nullptr) {
        s_irq_phy->on_tx_complete();
    }
}

void __not_in_flash_func(Phy::on_tx_complete)() {
    if (!dma_channel_get_irq0_status(dma_tx_)) {
        return; // a collision aborted the transfer this interrupt was raised for
    }
    dma_channel_acknowledge_irq0(dma_tx_);

    // A collided attempt ends in a jam, not a frame: it is neither sent nor
    // underrun, and TXSTALL means nothing across the abort.
    if (!collided_.load(std::memory_order_relaxed)) {
        // The DMA has just queued the final octets, so while the FIFO still holds data the
        // SM cannot have stalled at the end of the frame: a TXSTALL then is an underrun.
        // An empty FIFO here (IRQ latency beyond the queued tail) is not counted, so the
        // count can miss underruns but never reports false ones.
        const bool underrun{(pio_->fdebug & tx_stall_mask()) != 0u && !pio_sm_is_tx_fifo_empty(pio_, sm_tx_)};
        std::atomic<std::uint32_t>& outcome{underrun ? tx_underrun_ : tx_sent_};
        outcome.store(outcome.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
    }

    // The DMA has handed off the last byte, but the PIO FIFO and shifter still hold
    // a few bytes; wait so force_idle() does not clip the tail. The FIFO draining
    // leaves up to one octet still shifting out of the OSR after the FIFO reads
    // empty, so add a two-octet margin before driving idle.
    while (transmitting()) {
        tight_loop_contents();
    }
    busy_wait_at_least_cycles(2 * 8 * PIO_CYCLES_PER_BIT);
    force_idle();

    pio_set_irq0_source_enabled(pio_, qualify_interrupt_source(), false);
    note_medium_free();
    active_.store(false, std::memory_order_release);
}

void __not_in_flash_func(Phy::force_idle)() {
    // Drive the differential pair to 0 V (TP_IDL then line idle). Injected while
    // the SM is stalled on an empty FIFO, so the level holds until the next frame.
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
}

bool Phy::emit_nlp() {
    // A single ~100 ns positive excursion, and only onto a medium that is free: a
    // pulse laid over a frame -- ours or the peer's -- corrupts it.
    if (active_.load(std::memory_order_relaxed) || transmitting() || gpio_get(pins_.rxc)) {
        return false;
    }
    if (time_us_32() - medium_free_us_.load(std::memory_order_relaxed) < IFG_US) {
        return false;
    }

    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_POS));
    busy_wait_at_least_cycles(PIO_CYCLES_PER_BIT);
    pio_sm_exec(pio_, sm_tx_, pio_encode_set(pio_pins, LEVEL_IDLE));
    note_medium_free();
    return true;
}

std::int64_t Phy::nlp_alarm_cb(alarm_id_t /*id*/, void* user) {
    auto* self{static_cast<Phy*>(user)};
    // A pulse held back for a busy medium is retried far sooner than the cadence, so
    // traffic barely shifts the pulse train the peer's link timer is watching.
    const std::uint32_t delay_ms{self->emit_nlp() ? self->nlp_.next_interval_ms() : NLP_RETRY_MS};
    // Positive return = us from this scheduled fire.
    return static_cast<std::int64_t>(delay_ms) * 1000;
}

} // namespace pico_ethernet
