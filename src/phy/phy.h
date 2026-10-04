// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_H
#define PHY_PHY_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h" // IWYU pragma: keep  (alarm_id_t, repeating timers)

#include "src/mac/ethernet_frame.h"
#include "src/phy/autoneg.h"
#include "src/phy/csma_cd.h"
#include "src/phy/duplex.h"
#include "src/phy/link_pulse.h"
#include "src/phy/link_state.h"
#include "src/phy/phy_stats.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/rx_slot_ring.h"

namespace pico_ethernet {

// Software 10BASE-T PHY. Transmit: serializes complete MAC wire frames as
// Manchester symbols on the differential pair {TXP, TXN} using one PIO state
// machine fed by DMA, and emits link pulses when the line is idle. Receive: on a
// PIO block of its own, an SM slices the RXD/RXC comparator outputs back into the
// frame's octets, from its SFD to its TP_IDL, which a second SM detects. DMA lands
// them in a pool of capture buffers, and the DMA CRC sniffer computes their FCS on
// the way. The RXC falling edge (end of carrier) is finalized in an interrupt that
// hands the DMA the next free buffer, and poll_rx() delivers completed frames from
// the main loop. The system clock is run at 120 MHz so a 50 ns half-bit is an exact
// 6 PIO cycles.
//
// The link comes up by autonegotiation, or by parallel detection for a peer that
// only sends NLPs. Link pulses -- NLPs and FLP bursts -- go out from a timer alarm
// and are decoded in the carrier interrupt; Autoneg arbitrates in the main loop, and
// once the link is up LinkState's link integrity verdict keeps it up. The host-facing
// link notification follows the result.
//
// In half duplex RXC is carrier sense as well as end-of-frame, so transmission
// defers to an occupied medium, and a carrier that appears mid-transmit and stays
// asserted for CARRIER_QUALIFY_US is a collision. Both are routed through CsmaCd,
// which decides deferral, backoff and abandonment; this class performs the hardware
// actions those decisions call for. In full duplex the pairs are independent, and
// only the interframe gap still holds a frame back.
class Phy {
  public:
    struct Pins {
        std::uint32_t txp; // TX SM SET-group bit 0
        std::uint32_t txn; // TX SM SET-group bit 1 (must be txp + 1)
        std::uint32_t rxd; // RX SM IN base + JMP pin: data slicer
        std::uint32_t rxc; // RX SM IN base + 1: carrier detect (must be rxd + 1)
    };
    static constexpr Pins DEFAULT_PINS{.txp = 2, .txn = 3, .rxd = 8, .rxc = 9};

    // Outcome of a receive poll. `frame` aliases its capture buffer, which stays
    // valid until release_rx().
    struct RxFrame {
        enum class Kind : std::uint8_t {
            None,  // no completed reception this poll
            Frame, // a byte-aligned destination..FCS frame is ready in `frame`
            Error, // a frame was captured but rejected by the MAC checks (`error`)
        };
        Kind kind{Kind::None};
        std::span<const std::uint8_t> frame{}; // destination..FCS when Kind::Frame
        FrameError error{};                    // valid when Kind::Error
    };

    // `backoff_seed` seeds the collision backoff draw. Two stations sharing a seed
    // would redraw the same backoff after every collision and keep colliding, so it
    // must differ per device; it is a parameter so a test can pin the series.
    explicit Phy(std::uint32_t backoff_seed, const Pins& pins = DEFAULT_PINS);

    Phy(const Phy&) = delete;
    Phy& operator=(const Phy&) = delete;

    // Sets the 120 MHz system clock, loads the PIO programs, configures the state
    // machines, DMA channels and GPIOs, arms the receiver, and starts the link pulse
    // alarm. Call once before the first transmit. Returns false if PIO/DMA resources
    // could not be claimed.
    [[nodiscard]] bool initialize();

    // The queue slot to build the next wire frame (preamble..FCS) into, or nullptr
    // when the queue is full and the caller must apply backpressure. Non-owning, and
    // valid until commit_tx() enqueues it. The slot still holds the bytes of an
    // earlier frame, so the builder must write every byte it then counts in `length`.
    [[nodiscard]] WireFrame* tx_slot();

    // Enqueues the slot tx_slot() returned, once it holds a complete wire frame.
    // Queued frames are clocked out one at a time by service(), subject to carrier
    // sense and the interframe gap.
    void commit_tx();

    // True while a transmission is on the wire, its start of idle included.
    [[nodiscard]] bool transmitting() const;

    // Drives the link and the transmit side: advances autonegotiation and the link
    // integrity test, resolves the attempt that just ended, starts the next queued
    // frame once the transmit policy allows it, and discards queued frames while the
    // link is down. Collision response (abort, jam) happens in the
    // carrier-qualification interrupt, and end-of-transmit finalization in the
    // interrupt the TX state machine raises once it has released the line, not here.
    // Call from the loop.
    void service();

    // Checks the oldest frame the end-of-frame interrupt captured and returns it;
    // None when the completed-frame queue is empty. Any other result stays queued
    // until release_rx(), which must come before the next poll_rx(). Call from the
    // main loop.
    [[nodiscard]] RxFrame poll_rx();

    // Dequeues the frame the last poll_rx() returned, handing its buffer back to the
    // capture.
    void release_rx() { rx_ring_.release(); }

    // True when a completed frame is waiting in the pool. Lets the drain test for
    // work before touching the USB path.
    [[nodiscard]] bool rx_pending() const { return rx_ring_.peek().has_value(); }

    // Whether autonegotiation or parallel detection has brought the link up, and the
    // link integrity test has kept it up since. Follows the wire, so it is false until
    // a peer is seen. Main-loop state, updated by service().
    [[nodiscard]] bool link_up() const { return autoneg_.link().has_value(); }

    // Running count of frames dropped because the receive buffer pool was exhausted
    // (the drain fell behind line rate). A visible, counted drop, not a silent loss.
    [[nodiscard]] std::uint32_t rx_pool_overflow() const { return rx_ring_.overflow(); }

    // Running counts of captures dropped at the end of their carrier: carriers that
    // outlasted a link pulse but never showed an SFD, and frames whose carrier
    // dropped without TP_IDL.
    [[nodiscard]] std::uint32_t rx_glitches() const { return rx_glitches_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint32_t rx_truncated() const { return rx_truncated_.load(std::memory_order_relaxed); }

    // Running counts of transmitted frames: clean ones, and ones the TX FIFO ran dry
    // in, which end on the wire where their data ran out.
    [[nodiscard]] std::uint32_t tx_sent() const { return tx_sent_.load(std::memory_order_relaxed); }
    [[nodiscard]] std::uint32_t tx_underrun() const { return tx_underrun_.load(std::memory_order_relaxed); }

    [[nodiscard]] const CsmaStats& csma_stats() const { return csma_stats_; }

    // Running count of wire activity: every transmit attempt, collided ones
    // included, and every capture that decoded data, whether it was queued or
    // dropped. Link pulses either way are not activity. Wraps.
    [[nodiscard]] std::uint32_t wire_activity() const {
        return tx_attempts_ + rx_captures_.load(std::memory_order_relaxed);
    }

  private:
    void configure_tx();
    void configure_rx();
    void configure_carrier_qualify();
    void rearm_capture(std::size_t slot);
    void on_rx_eof();
    void discard_capture();
    void on_link_pulse();
    void on_carrier_assert();
    void on_carrier_qualified();
    static void carrier_qualified_irq_handler();
    void on_collision();
    static void carrier_irq_handler(uint gpio, std::uint32_t events);
    // Both run with interrupts masked, by service().
    void advance_tx();
    void start_tx(const WireFrame& frame);
    void on_tx_complete();
    static void tx_done_irq_handler();
    void emit_pulse();
    [[nodiscard]] bool emit_nlp();
    [[nodiscard]] std::uint32_t emit_link_pulses();
    static std::int64_t link_pulse_alarm_cb(alarm_id_t id, void* user);
    void advance_link(std::uint32_t now_us);
    void resolve_attempt();
    void release_frame();
    void note_medium_free();

    Pins pins_;
    PIO pio_{pio0};
    PIO rx_pio_{pio1};
    std::uint32_t sm_tx_{0};
    std::uint32_t sm_rx_{0};
    std::uint32_t sm_qualify_{0};
    std::uint32_t sm_watchdog_{0};
    std::uint32_t offset_tx_{0};
    std::uint32_t offset_rx_{0};
    std::uint32_t offset_qualify_{0};
    std::uint32_t offset_watchdog_{0};
    pio_sm_config tx_cfg_{};
    pio_sm_config rx_cfg_{};
    std::int32_t dma_tx_{-1};
    std::int32_t dma_rx_{-1};

    // The link pulse alarm's own state.
    NlpScheduler nlp_{};
    std::optional<FlpBurst> flp_burst_{};
    alarm_id_t link_pulse_alarm_{-1};

    // What the alarm sends, mirrored from autoneg_ by the loop, and how many FLP
    // bursts it has completed, counted in the alarm and consumed by the loop.
    std::atomic<Autoneg::LinkPulses> link_pulses_{Autoneg::LinkPulses::None};
    std::atomic<std::uint16_t> code_word_{ADVERTISED_LCW};
    std::atomic<std::uint32_t> flp_bursts_sent_{0};
    std::uint32_t flp_bursts_seen_{0};

    // The resolved duplex, written by the loop only as the link comes up.
    std::atomic<Duplex> duplex_{Duplex::Half};

    // Whether received frames are delivered, mirrored from link_up() by the loop for
    // the end-of-frame interrupt.
    std::atomic<bool> link_up_mirror_{false};

    // TX queue: host frames are built in place here and clocked out one at a time.
    // Both indices belong to the main loop -- commit_tx() fills at tx_head_ and
    // service() consumes at tx_tail_ -- because a frame is not finished with when its
    // transmission ends: a collision leaves it queued for a retry, and only the loop
    // knows whether it is retried, abandoned or discarded. The transmit interrupts
    // therefore never move the queue on; they publish the attempt's outcome and the
    // loop acts on it. One slot stays free to tell full from empty, so N slots queue
    // N-1 frames.
    static constexpr std::size_t TX_QUEUE_SIZE{5};
    static constexpr std::size_t tx_advance(std::size_t i) { return (i + 1 == TX_QUEUE_SIZE) ? 0 : i + 1; }
    std::array<WireFrame, TX_QUEUE_SIZE> tx_queue_{};
    std::size_t tx_head_{0};
    std::size_t tx_tail_{0};
    bool attempt_in_flight_{false};
    std::uint32_t tx_attempts_{0};

    std::atomic<bool> active_{false}; // set in start_tx (thread), cleared in on_tx_complete (IRQ)

    // Written only in on_tx_complete() (TX-done interrupt).
    std::atomic<std::uint32_t> tx_sent_{0};
    std::atomic<std::uint32_t> tx_underrun_{0};

    // When the medium last fell idle -- a carrier deasserting, or our own
    // transmission (frame or jam) draining -- which is where the interframe gap and
    // the backoff grid are measured from. Written in the carrier and TX-done
    // interrupts, read by the loop.
    std::atomic<std::uint32_t> medium_free_us_{0};

    // A carrier qualified while we transmit. The qualification interrupt aborts the
    // frame, starts the jam and publishes how far into the frame the carrier
    // appeared; the loop decides what becomes of the frame. Written in the
    // qualification interrupt.
    std::atomic<bool> collided_{false};
    std::atomic<std::uint32_t> collision_elapsed_us_{0};
    std::atomic<std::uint32_t> tx_start_us_{0};

    // The qualification interrupt clears the qualifier's flag, so it records here
    // that the carrier qualified, for the end-of-frame interrupt to tell a carrier
    // from a link pulse.
    std::atomic<bool> carrier_qualified_{false};

    // Carrier blips that decoded no data are link pulses, timed from their rising
    // edge and decoded in the carrier interrupt.
    std::uint32_t carrier_asserted_us_{0};
    LinkPulseDecoder pulse_decoder_{};

    // Decoded link pulses, handed from the carrier interrupt to the loop: a count of
    // valid NLPs, and the latest code word packed with its sequence number.
    std::atomic<std::uint32_t> valid_nlps_{0};
    std::uint32_t valid_nlps_seen_{0};
    std::atomic<std::uint32_t> code_word_slot_{0};
    std::uint32_t code_word_sequence_seen_{0};

    CsmaCd csma_;
    LinkState link_{};
    Autoneg autoneg_;
    CsmaStats csma_stats_{};

    // The 32-bit jam that tells the peer the frame it is hearing is a collision. It
    // is DMA'd like a frame, so it lives in RAM rather than in flash.
    static constexpr std::size_t JAM_OCTETS{JAM_BITS / 8};
    static constexpr std::uint8_t JAM_OCTET{0xAA};
    std::array<std::uint8_t, JAM_OCTETS> jam_{};

    // The RX SM autopushes four recovered octets per 32-bit FIFO word; DMA lands
    // them densely into the current pool buffer. A maximum-size frame and the one
    // pad octet it needs at least is the longest capture that ends on TP_IDL; any
    // longer one fills the buffer, and is told apart as a giant by its length alone.
    static constexpr std::size_t RX_WORD_CAPACITY{
        (MAX_FRAME_WITH_FCS + 1 + RX_OCTETS_PER_WORD - 1) / RX_OCTETS_PER_WORD};
    static constexpr std::size_t RX_OCTET_CAPACITY{RX_WORD_CAPACITY * RX_OCTETS_PER_WORD};
    static_assert(
        RX_OCTET_CAPACITY > MAX_FRAME_WITH_FCS && RX_OCTET_CAPACITY - RX_OCTETS_PER_WORD <= MAX_FRAME_WITH_FCS);

    // A pool of capture buffers so the DMA can start the next frame the instant one
    // ends (the EOF interrupt hands it a free buffer) while the main loop is still
    // draining an earlier one -- capture and processing overlap instead of taking
    // turns. rx_ring_ tracks which buffer is capturing and which are completed.
    // Held as octets but word-aligned, since the DMA writes whole words into them.
    static constexpr std::size_t RX_POOL_SIZE{6};
    alignas(std::uint32_t) std::array<std::array<std::uint8_t, RX_OCTET_CAPACITY>, RX_POOL_SIZE> rx_pool_{};
    RxSlotRing<RX_POOL_SIZE> rx_ring_{};

    // Written only in on_rx_eof() (end-of-frame interrupt). Captures are consumed by
    // the loop as received frames for the link integrity test.
    std::atomic<std::uint32_t> rx_captures_{0};
    std::uint32_t rx_captures_seen_{0};
    std::atomic<std::uint32_t> rx_glitches_{0};
    std::atomic<std::uint32_t> rx_truncated_{0};
};

} // namespace pico_ethernet

#endif // PHY_PHY_H
