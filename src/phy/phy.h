// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_H
#define PHY_PHY_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <span>

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h" // IWYU pragma: keep  (alarm_id_t, repeating timers)

#include "src/mac/ethernet_frame.h"
#include "src/phy/link_pulse.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/rx_slot_ring.h"

namespace pico_ethernet {

// Software 10BASE-T PHY. Transmit: serializes complete MAC wire frames as
// Manchester symbols on the differential pair {TXP, TXN} using one PIO state
// machine fed by DMA, and emits Normal Link Pulses when the line is idle. Receive:
// a second PIO SM slices the RXD/RXC comparator outputs back into octets via DMA
// into a pool of capture buffers; the
// RXC falling edge (end of frame) is finalized in an interrupt that hands the DMA
// the next free buffer, and poll_rx() drains and recovers completed frames from the
// main loop. The system clock is run at 120 MHz so a 50 ns half-bit is an exact 6
// PIO cycles.
class Phy {
  public:
    struct Pins {
        std::uint32_t txp; // TX SM SET-group bit 0
        std::uint32_t txn; // TX SM SET-group bit 1 (must be txp + 1)
        std::uint32_t rxd; // RX SM IN base + JMP pin: data slicer
        std::uint32_t rxc; // RX SM IN base + 1: carrier detect (must be rxd + 1)
    };
    static constexpr Pins DEFAULT_PINS{.txp = 2, .txn = 3, .rxd = 8, .rxc = 9};

    // Outcome of a receive poll. `frame` aliases an internal buffer valid until the
    // next poll_rx() call.
    struct RxFrame {
        enum class Kind : std::uint8_t {
            None,   // no completed reception this poll
            Frame,  // a byte-aligned destination..FCS frame is ready in `frame`
            Glitch, // carrier came and went without a decodable frame (no SFD)
            Error,  // a frame was captured but rejected by the MAC checks (`error`)
        };
        Kind kind{Kind::None};
        std::span<const std::uint8_t> frame{}; // destination..FCS when Kind::Frame
        FrameError error{};                    // valid when Kind::Error
    };

    explicit Phy(const Pins& pins = DEFAULT_PINS);

    Phy(const Phy&) = delete;
    Phy& operator=(const Phy&) = delete;

    // Sets the 120 MHz system clock, loads the PIO programs, configures the state
    // machines, DMA channels and GPIOs, arms the receiver, and starts the NLP
    // cadence. Call once before the first transmit. Returns false if PIO/DMA
    // resources could not be claimed.
    [[nodiscard]] bool initialize();

    // Enqueues a complete wire frame (preamble..FCS) for transmission. The bytes
    // are copied into the TX queue, so the span need not outlive the call. Returns
    // false if the queue is full (the caller should apply backpressure) or the frame
    // is empty or larger than a wire frame. Queued frames are clocked out one at a
    // time by service(), spaced by the interframe gap.
    [[nodiscard]] bool transmit(std::span<const std::uint8_t> wire_frame);

    // True while a frame is being clocked out.
    [[nodiscard]] bool transmitting() const;

    // Starts the next queued frame when the transmitter is idle and the interframe
    // gap has elapsed. End-of-transmit finalization (idle drive, receiver re-arm)
    // is handled in the TX-DMA completion interrupt, not here. Call from the loop.
    void service();

    // Dequeues one frame the end-of-frame interrupt captured, recovers it in a
    // single pass, and returns it; None when the completed-frame queue is empty.
    // Call from the main loop.
    [[nodiscard]] RxFrame poll_rx();

    // True when a completed frame is waiting in the pool, without the cost of
    // recovering it. Lets the drain test for work before touching the USB path.
    [[nodiscard]] bool rx_pending() const { return rx_ring_.peek().has_value(); }

    // Running count of frames dropped because the receive buffer pool was exhausted
    // (the drain fell behind line rate). A visible, counted drop, not a silent loss.
    [[nodiscard]] std::uint32_t rx_pool_overflow() const { return rx_ring_.overflow(); }

  private:
    void configure_tx();
    void configure_rx();
    void rearm_capture(std::size_t slot);
    void on_rx_eof();
    static void rx_irq_handler(uint gpio, std::uint32_t events);
    void start_tx(const WireFrame& frame);
    void on_tx_complete();
    static void tx_dma_irq_handler();
    void force_idle();
    void emit_nlp();
    static std::int64_t nlp_alarm_cb(alarm_id_t id, void* user);

    Pins pins_;
    PIO pio_{pio0};
    std::uint32_t sm_tx_{0};
    std::uint32_t sm_rx_{0};
    std::uint32_t offset_tx_{0};
    std::uint32_t offset_rx_{0};
    pio_sm_config tx_cfg_{};
    pio_sm_config rx_cfg_{};
    int dma_tx_{-1};
    int dma_rx_{-1};

    NlpScheduler nlp_{};
    alarm_id_t nlp_alarm_{-1};

    // TX queue: host frames are enqueued (copied) here and clocked out one at a
    // time. Single-producer/single-consumer, like RxSlotRing: transmit() (main-loop
    // thread) is the producer and owns tx_head_; on_tx_complete() (TX-DMA interrupt)
    // is the consumer and owns tx_tail_. One slot stays free to tell full from empty,
    // so N slots queue N-1 frames.
    static constexpr std::size_t TX_QUEUE_SIZE{5};
    static constexpr std::size_t tx_advance(std::size_t i) { return (i + 1 == TX_QUEUE_SIZE) ? 0 : i + 1; }
    std::array<WireFrame, TX_QUEUE_SIZE> tx_queue_{};
    std::atomic<std::size_t> tx_head_{0};
    std::atomic<std::size_t> tx_tail_{0};
    std::atomic<std::uint32_t> tx_last_end_us_{0}; // for interframe-gap spacing

    std::atomic<bool> active_{false}; // set in start_tx (thread), cleared in on_tx_complete (IRQ)

    // The RX SM autopushes four recovered octets per 32-bit FIFO word; DMA lands
    // them densely into the current pool buffer. A worst case capture is a
    // full-preamble max wire frame plus the trailing carrier octets needed to flush
    // the word that holds the final FCS octet.
    static constexpr std::size_t RX_TRAILING_OCTETS{RX_OCTETS_PER_WORD - 1};
    static constexpr std::size_t RX_WORD_CAPACITY{
        (WIRE_CAPACITY + RX_TRAILING_OCTETS + RX_OCTETS_PER_WORD - 1) / RX_OCTETS_PER_WORD};

    // A pool of capture buffers so the DMA can start the next frame the instant one
    // ends (the EOF interrupt hands it a free buffer) while the main loop is still
    // draining an earlier one -- capture and processing overlap instead of taking
    // turns. rx_ring_ tracks which buffer is capturing and which are completed.
    static constexpr std::size_t RX_POOL_SIZE{6};
    std::array<std::array<std::uint32_t, RX_WORD_CAPACITY>, RX_POOL_SIZE> rx_pool_{};
    RxSlotRing<RX_POOL_SIZE> rx_ring_{};

    // The drain unpacks the packed capture words straight into rx_frame_ as the
    // byte-aligned destination..FCS frame in a single pass. Main-loop-owned.
    std::array<std::uint8_t, WIRE_CAPACITY> rx_frame_{};
};

} // namespace pico_ethernet

#endif // PHY_PHY_H
