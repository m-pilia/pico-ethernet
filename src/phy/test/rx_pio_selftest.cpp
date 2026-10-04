// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the receive PIO programs. A stimulus SM on
// another PIO block replays precomputed RXD waveforms (derived from the reference
// Manchester encoder) into the RX decoder and the TP_IDL watchdog, configured as
// the PHY configures them -- no wire, no scope -- while a GPIO holds RXC. The
// decoder's words are collected by DMA through the CRC sniffer and checked against
// the source frames: SFD alignment after garbage, the end on TP_IDL with its zero
// padding, the frame end found from the sniffer state, and the watchdog threshold.
// Results are reported over the UART (debugprobe console).

#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <span>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"
#include "src/phy/phy_timing.h"
#include "src/phy/rx_frame_end.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/test/rx_stimulus.h"
#include "src/phy/test/tx_reference.h"
#include "src/phy/tx_level.h"
#include "src/util/led_heartbeat.h"

#include "rx.pio.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t RXD_PIN{8}; // stimulus output, RX SM IN base
constexpr std::uint32_t RXC_PIN{9}; // carrier, driven high via SIO (RX IN base + 1)
constexpr std::uint32_t LED_PIN{25};

constexpr std::size_t CAPTURE_WORDS{32};

// Frame lengths (destination..FCS) for each pad the RX SM can add, 4 down to 1.
constexpr std::array<std::size_t, RX_OCTETS_PER_WORD> FRAME_LENGTHS{
    MIN_FRAME_WITH_FCS,
    MIN_FRAME_WITH_FCS + 1,
    MIN_FRAME_WITH_FCS + 2,
    MIN_FRAME_WITH_FCS + 3};

// High runs on either side of TP_IDL_DETECT_NS, in whole half-bits.
constexpr std::size_t SHORT_HIGH_HALF_BITS{3};
constexpr std::size_t LONG_HIGH_HALF_BITS{4};
static_assert(SHORT_HIGH_HALF_BITS * HALF_BIT_NS < TP_IDL_DETECT_NS);
static_assert(LONG_HIGH_HALF_BITS * HALF_BIT_NS > TP_IDL_DETECT_NS);

struct Capture {
    bool ended;         // RX_DONE_FLAG raised: the frame ended on TP_IDL
    std::size_t octets; // landed by the DMA
    std::uint32_t crc;  // sniffer state over those octets
};

class RxSelfTest {
  public:
    [[nodiscard]] bool configure() {
        // RXC carrier: SIO output held high for every injection.
        gpio_init(RXC_PIN);
        gpio_set_dir(RXC_PIN, GPIO_OUT);
        gpio_put(RXC_PIN, true);

        if (!stimulus_.configure(pio0, RXD_PIN) || !pio_can_add_program(rx_pio_, &rx_manchester_program) ||
            !pio_can_add_program(rx_pio_, &tp_idl_watchdog_program)) {
            return false;
        }
        offset_rx_ = static_cast<std::uint32_t>(pio_add_program(rx_pio_, &rx_manchester_program));
        const std::uint32_t offset_watchdog{
            static_cast<std::uint32_t>(pio_add_program(rx_pio_, &tp_idl_watchdog_program))};
        const std::int32_t sm_rx{pio_claim_unused_sm(rx_pio_, false)};
        const std::int32_t sm_watchdog{pio_claim_unused_sm(rx_pio_, false)};
        dma_rx_ = dma_claim_unused_channel(false);
        if (sm_rx < 0 || sm_watchdog < 0 || dma_rx_ < 0) {
            return false;
        }
        sm_rx_ = static_cast<std::uint32_t>(sm_rx);

        rx_cfg_ = rx_manchester_config(offset_rx_, RXD_PIN);
        start_tp_idl_watchdog(rx_pio_, static_cast<std::uint32_t>(sm_watchdog), offset_watchdog, RXD_PIN);
        enable_rx_sniffer(dma_rx_);
        return true;
    }

    Capture run(const RxWaveform& waveform) {
        // Nonzero, so the zero padding can only come from the RX SM.
        octets_.fill(0xFF);
        restart_rx(rx_pio_, sm_rx_, offset_rx_, rx_cfg_);
        start_rx_capture(dma_rx_, rx_pio_, sm_rx_, octets_);
        pio_sm_set_enabled(rx_pio_, sm_rx_, true);

        stimulus_.play(waveform);
        busy_wait_us(RX_SETTLE_US);

        const bool ended{pio_interrupt_get(rx_pio_, RX_DONE_FLAG)};
        pio_sm_set_enabled(rx_pio_, sm_rx_, false);
        const std::size_t words{CAPTURE_WORDS - dma_channel_hw_addr(dma_rx_)->transfer_count};
        const std::uint32_t crc{dma_sniffer_get_data_accumulator()};
        dma_channel_abort(dma_rx_);
        return {.ended = ended, .octets = words * RX_OCTETS_PER_WORD, .crc = crc};
    }

    [[nodiscard]] bool tp_idl_seen() const { return pio_interrupt_get(rx_pio_, TP_IDL_FLAG); }

    [[nodiscard]] std::span<const std::uint8_t> octets(const Capture& capture) const {
        return std::span{octets_}.first(capture.octets);
    }

  private:
    RxStimulus stimulus_{};
    PIO rx_pio_{pio1};
    std::uint32_t sm_rx_{0};
    std::uint32_t offset_rx_{0};
    std::int32_t dma_rx_{-1};
    pio_sm_config rx_cfg_{};
    alignas(std::uint32_t) std::array<std::uint8_t, CAPTURE_WORDS * RX_OCTETS_PER_WORD> octets_{};
};

std::span<const std::uint8_t> body(const WireFrame& wire) { return wire.view().subspan(PREAMBLE_SFD_LEN); }

// Garbage ahead of the preamble that restarts the SFD hunt: an alternating run cut
// short by a 1,1, then a 0,0. `offset` more alternating bits, ending in 0 so they
// flow into the preamble's first 1, shift the frame against the decoder's start.
RxWaveform& lead(RxWaveform& waveform, std::size_t offset) {
    for (const bool one : {true, false, true, false, true, true, false, false}) {
        waveform.bit(one);
    }
    for (std::size_t b{0}; b < offset; ++b) {
        waveform.bit(((offset - 1 - b) % 2) == 1);
    }
    return waveform;
}

// The frame must come out byte-aligned from its first destination octet, followed
// by the zero padding of its last word, with the sniffer agreeing with the software
// CRC and the end check finding its length.
bool check_frame_capture(RxSelfTest& test, const Capture& capture, std::span<const std::uint8_t> frame) {
    const std::size_t pad{RX_OCTETS_PER_WORD - frame.size() % RX_OCTETS_PER_WORD};
    const std::span<const std::uint8_t> octets{test.octets(capture)};
    bool ok{true};
    if (!capture.ended) {
        printf("  no end on TP_IDL\n");
        ok = false;
    }
    if (octets.size() != frame.size() + pad) {
        printf(
            "  captured %" PRIu32 " octets, expected %" PRIu32 "\n",
            static_cast<std::uint32_t>(octets.size()),
            static_cast<std::uint32_t>(frame.size() + pad));
        return false;
    }
    if (!std::ranges::equal(octets.first(frame.size()), frame)) {
        printf("  frame octets mismatch\n");
        ok = false;
    }
    if (!std::ranges::all_of(octets.subspan(frame.size()), [](std::uint8_t octet) { return octet == 0; })) {
        printf("  padding is not zero\n");
        ok = false;
    }
    const std::uint32_t software_crc{std::ranges::fold_left(octets, CRC32_INIT, crc32_update)};
    if (capture.crc != software_crc) {
        printf("  sniffer 0x%08" PRIX32 ", software CRC 0x%08" PRIX32 "\n", capture.crc, software_crc);
        ok = false;
    }
    const std::expected<std::size_t, FrameError> length{find_frame_end(octets.size(), capture.crc)};
    if (!length || *length != frame.size()) {
        printf("  end check did not find the %" PRIu32 "-octet frame\n", static_cast<std::uint32_t>(frame.size()));
        ok = false;
    }
    return ok;
}

bool check_tp_idl_end_matrix(RxSelfTest& test) {
    static RxWaveform waveform{};
    bool ok{true};
    for (const std::size_t length : FRAME_LENGTHS) {
        for (const bool last_bit : {false, true}) {
            const WireFrame& wire{test_frame(length, last_bit)};
            for (std::size_t offset{0}; offset < 8; ++offset) {
                lead(waveform.clear(), offset).transmission(wire.view());
                if (!check_frame_capture(test, test.run(waveform), body(wire))) {
                    printf(
                        "FAIL: %" PRIu32 "-octet frame, last bit %d, offset %" PRIu32 "\n",
                        static_cast<std::uint32_t>(length),
                        last_bit ? 1 : 0,
                        static_cast<std::uint32_t>(offset));
                    ok = false;
                }
            }
        }
    }
    return ok;
}

bool check_carrier_drop_without_tp_idl(RxSelfTest& test) {
    static RxWaveform waveform{};
    lead(waveform.clear(), 0).octets(test_frame(MIN_FRAME_WITH_FCS, true).view());
    const Capture capture{test.run(waveform)};
    if (capture.ended || capture.octets == 0) {
        printf(
            "FAIL: frame without TP_IDL: ended %d, %" PRIu32 " octets\n",
            capture.ended ? 1 : 0,
            static_cast<std::uint32_t>(capture.octets));
        return false;
    }
    return true;
}

bool check_high_before_sfd(RxSelfTest& test) {
    static RxWaveform waveform{};
    const WireFrame& wire{test_frame(MIN_FRAME_WITH_FCS, false)};
    lead(waveform.clear(), 0).level(LEVEL_POS, TP_IDL_HALF_BITS).level(LEVEL_NEG, 1).transmission(wire.view());
    if (!check_frame_capture(test, test.run(waveform), body(wire))) {
        printf("FAIL: a TP_IDL-long high before the SFD disturbed the frame\n");
        return false;
    }
    return true;
}

// A high run of `half_bits` after the SFD, from a low line, must raise the watchdog
// flag exactly when it outlasts TP_IDL_DETECT_NS.
bool check_watchdog_threshold(RxSelfTest& test, std::size_t half_bits, bool expected) {
    static RxWaveform waveform{};
    const WireFrame& wire{test_frame(MIN_FRAME_WITH_FCS, false)};
    waveform.clear().octets(wire.view().first(PREAMBLE_SFD_LEN + MAC_HEADER_LEN));
    waveform.level(LEVEL_NEG, 1).level(LEVEL_POS, half_bits);
    (void)test.run(waveform);
    if (test.tp_idl_seen() != expected) {
        printf(
            "FAIL: a %" PRIu32 " ns high %s the watchdog\n",
            static_cast<std::uint32_t>(half_bits * HALF_BIT_NS),
            expected ? "did not trigger" : "triggered");
        return false;
    }
    return true;
}

} // namespace

bool run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();
    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\nrx_pio_selftest: start\n");

    static RxSelfTest test{};
    bool ok{test.configure()};
    if (!ok) {
        printf("FAIL: could not claim the PIO and DMA resources\n");
    }
    if (ok) {
        ok = check_tp_idl_end_matrix(test) && ok;
        ok = check_carrier_drop_without_tp_idl(test) && ok;
        ok = check_high_before_sfd(test) && ok;
        ok = check_watchdog_threshold(test, SHORT_HIGH_HALF_BITS, false) && ok;
        ok = check_watchdog_threshold(test, LONG_HIGH_HALF_BITS, true) && ok;
    }

    printf("rx_pio_selftest: %s\n", ok ? "PASS" : "FAIL");

    LedHeartbeat led{LED_PIN};
    if (ok) {
        led.good();
    } else {
        led.bad();
    }
    while (true) {
        sleep_ms(10'000);
    }
    return ok;
}

} // namespace pico_ethernet

int main() { return pico_ethernet::run_selftest() ? 0 : 1; }
