// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_H
#define PHY_PHY_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>

#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h" // IWYU pragma: keep  (alarm_id_t, repeating timers)

#include "src/mac/ethernet_frame.h"
#include "src/phy/link_pulse.h"
#include "src/phy/rx_pio_config.h"

namespace pico_ethernet {

// Software 10BASE-T PHY. Transmit: serializes complete MAC wire frames as
// Manchester symbols with pre-emphasis using two synchronized PIO state machines
// -- LEVEL drives the differential pair {TXP, TXN}, EMPHASIS drives the
// amplitude-select pin TXE -- each fed by its own DMA channel, and emits Normal
// Link Pulses when the line is idle. Receive: a third PIO SM slices the RXD/RXC
// comparator outputs back into wire-frame octets via DMA, recovered by poll_rx().
// The system clock is run at 120 MHz so a 50 ns half-bit is an exact 6 PIO
// cycles. The link/CSMA state machine is added later.
class Phy {
  public:
    struct Pins {
        std::uint32_t txp; // LEVEL SM SET-group bit 0
        std::uint32_t txn; // LEVEL SM SET-group bit 1 (must be txp + 1)
        std::uint32_t txe; // EMPHASIS SM SET/OUT pin
        std::uint32_t rxd; // RX SM IN base + JMP pin: data slicer
        std::uint32_t rxc; // RX SM IN base + 1: carrier detect (must be rxd + 1)
    };
    static constexpr Pins DEFAULT_PINS{.txp = 2, .txn = 3, .txe = 4, .rxd = 8, .rxc = 9};

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

    // Hands a complete wire frame (preamble..FCS) to the transmit engine. The
    // bytes are copied into an internal DMA buffer, so the span need not outlive
    // the call. Returns false if a previous frame is still transmitting, or the
    // frame is empty or larger than a wire frame.
    [[nodiscard]] bool transmit(std::span<const std::uint8_t> wire_frame);

    // True while a frame is being clocked out.
    [[nodiscard]] bool transmitting() const;

    // Completes end-of-frame idle (TP_IDL) once a transmit drains. Call from the
    // main loop so idle handling stays out of IRQ context.
    void service();

    // Drains the receiver: detects carrier via RXC, and on carrier drop (EOF)
    // returns the octets the RX SM recovered, then re-arms for the next frame.
    // Call from the main loop.
    [[nodiscard]] RxFrame poll_rx();

  private:
    void configure_state_machines();
    void configure_rx();
    void arm_rx();
    void force_idle();
    void emit_nlp();
    static std::int64_t nlp_alarm_cb(alarm_id_t id, void* user);

    Pins pins_;
    PIO pio_{pio0};
    std::uint32_t sm_level_{0};
    std::uint32_t sm_emphasis_{0};
    std::uint32_t sm_rx_{0};
    std::uint32_t offset_level_{0};
    std::uint32_t offset_emphasis_{0};
    std::uint32_t offset_rx_{0};
    pio_sm_config level_cfg_{};
    pio_sm_config emphasis_cfg_{};
    pio_sm_config rx_cfg_{};
    int dma_level_{-1};
    int dma_emphasis_{-1};
    int dma_rx_{-1};

    NlpScheduler nlp_{};
    alarm_id_t nlp_alarm_{-1};

    std::array<std::uint8_t, WIRE_CAPACITY> tx_frame_{};
    std::array<std::uint8_t, WIRE_CAPACITY> tx_emphasis_{};
    std::size_t tx_len_{0};
    bool active_{false};

    // The RX SM autopushes four recovered octets per 32-bit FIFO word; DMA lands
    // them densely here. poll_rx() reads the packed words directly (no intermediate
    // byte copy) and recover_frame() writes the byte-aligned destination..FCS frame
    // into rx_frame_ in a single pass. A worst case capture is a full-preamble max
    // wire frame plus the trailing carrier octets needed to flush the word that
    // holds the final FCS octet.
    static constexpr std::size_t RX_TRAILING_OCTETS{RX_OCTETS_PER_WORD - 1};
    static constexpr std::size_t RX_WORD_CAPACITY{
        (WIRE_CAPACITY + RX_TRAILING_OCTETS + RX_OCTETS_PER_WORD - 1) / RX_OCTETS_PER_WORD};
    std::array<std::uint32_t, RX_WORD_CAPACITY> rx_words_{};
    std::array<std::uint8_t, WIRE_CAPACITY> rx_frame_{};
    bool receiving_{false};
};

} // namespace pico_ethernet

#endif // PHY_PHY_H
