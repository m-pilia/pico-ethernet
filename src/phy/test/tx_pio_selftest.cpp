// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the transmit PIO program. The TX state machine
// runs at its real /1 clock while a capture state machine, started in lockstep with
// it, records the driven {TXP,TXN} levels one sample per PIO cycle. The recovered
// sequence is asserted against the reference encoder -- Manchester polarity,
// LSB-first bit order, the start of idle after frames ending in either bit, idle
// before and after the transmission -- and its run lengths against the nominal
// half-bit, so the 6/6 half-bit split, the 12-cycle branch balance and the start of
// idle's length are checked as real time rather than as relative cycle counts.
// PASS/FAIL is reported over the UART (debugprobe console).

#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "src/phy/phy_timing.h"
#include "src/phy/test/pin_capture.h"
#include "src/phy/test/symbol_trace.h"
#include "src/phy/test/tx_reference.h"
#include "src/phy/tx_level.h"
#include "src/util/led_heartbeat.h"

#include "tx_level.pio.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t TXP_PIN{2};
constexpr std::uint32_t TXN_PIN{3};
constexpr std::uint32_t LED_PIN{25};

// Test frames exercising both branch arms and both polarities: 0x55 alternating bits
// (merged bit-boundary runs), 0xFF/0x00 runs (single-half runs), 0xD5 mixed. The
// last bit on the wire is the MSB of the last octet: 1 in the first frame, 0 in the
// second.
constexpr std::size_t FRAME_OCTETS{4};
using TestFrame = std::array<std::uint8_t, FRAME_OCTETS>;
constexpr TestFrame ENDS_IN_ONE{0x55, 0xFF, 0x00, 0xD5};
constexpr TestFrame ENDS_IN_ZERO{0xD5, 0x00, 0xFF, 0x55};
constexpr std::size_t TRANSMISSION_HALFBITS{FRAME_OCTETS * 8 * 2 + TP_IDL_HALF_BITS};
constexpr std::size_t TRANSMISSION_SAMPLES{TRANSMISSION_HALFBITS * HALF_BIT_SAMPLES};

// Room for the idle on either side of the transmission.
constexpr std::size_t CAPTURE_WORDS{64};
using Capture = PinCapture<CAPTURE_WORDS>;
static_assert(Capture::SAMPLE_COUNT > 2 * TRANSMISSION_SAMPLES);

// The {TXP,TXN} SET-group code (LEVEL_POS/NEG/IDLE) in a DBG_PADOUT word.
std::uint8_t padout_level(std::uint32_t padout) {
    return static_cast<std::uint8_t>(((padout >> TXP_PIN) & 1u) | (((padout >> TXN_PIN) & 1u) << 1));
}

struct SelfTest {
    PIO pio{pio0};
    std::uint32_t sm_tx{0};
    std::uint32_t offset{0};
    pio_sm_config cfg{};
    Capture capture{};
    std::array<std::uint8_t, Capture::SAMPLE_COUNT> samples{};
    std::array<HalfBit, TRANSMISSION_HALFBITS> expected{};

    [[nodiscard]] bool configure() {
        offset = static_cast<std::uint32_t>(pio_add_program(pio, &tx_level_program));
        sm_tx = static_cast<std::uint32_t>(pio_claim_unused_sm(pio, true));

        pio_gpio_init(pio, TXP_PIN);
        pio_gpio_init(pio, TXN_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm_tx, TXP_PIN, 2, true);

        cfg = tx_level_program_get_default_config(offset);
        sm_config_set_set_pins(&cfg, TXP_PIN, 2);
        sm_config_set_out_shift(&cfg, true, true, 8);
        sm_config_set_clkdiv(&cfg, 1.0f);
        return capture.configure(pio, TXP_PIN);
    }

    void restart() {
        pio_sm_set_enabled(pio, sm_tx, false);
        pio_sm_clear_fifos(pio, sm_tx);
        pio_sm_init(pio, sm_tx, offset, &cfg);
        pio_sm_exec(pio, sm_tx, pio_encode_set(pio_pins, LEVEL_IDLE));
        pio_interrupt_clear(pio, sm_tx);
    }

    // Preloads the FIFO (the 4-byte frame fits the 4-deep FIFO) so the frame starts
    // the cycle the machine is enabled, then runs transmitter and capture together.
    void run_capture(const TestFrame& frame) {
        encode_transmission(frame, expected);

        for (const std::uint8_t octet : frame) {
            pio_sm_put_blocking(pio, sm_tx, octet);
        }

        capture.arm();
        pio_enable_sm_mask_in_sync(pio, (1u << sm_tx) | capture.sm_mask());
        capture.wait();

        for (std::size_t i{0}; i < samples.size(); ++i) {
            samples[i] = capture.level(i);
        }
    }

    [[nodiscard]] bool line_is_idle() const { return padout_level(pio->dbg_padout) == LEVEL_IDLE; }

    [[nodiscard]] bool end_flag_raised() const { return pio_interrupt_get(pio, sm_tx); }
};

// Reconstructs the half-bit sequence from the captured levels and compares it
// against the reference transmission, then checks that the line is released after
// it and the transmission's total duration. Prints the first mismatch on failure.
bool analyze(const SelfTest& t) {
    static std::array<LevelRun, 2 * TRANSMISSION_HALFBITS> runs_buf{};
    const std::size_t run_count{level_runs(t.samples, runs_buf)};
    const std::span<const LevelRun> runs{runs_buf.data(), run_count};

    std::size_t index{0};
    while (index < runs.size() && runs[index].level == LEVEL_IDLE) {
        ++index;
    }
    if (index == runs.size()) {
        printf("FAIL: the line was never driven\n");
        return false;
    }
    const std::size_t start{runs[index].start};

    std::size_t half_bits{0};
    for (; index < runs.size() && half_bits < TRANSMISSION_HALFBITS; ++index) {
        const LevelRun& run{runs[index]};
        const std::size_t count{half_bits_in(run)};
        if (count == 0 || half_bits + count > TRANSMISSION_HALFBITS) {
            printf(
                "FAIL: run at %" PRIu32 " len %" PRIu32 " (%s) does not fit the half-bits left\n",
                static_cast<std::uint32_t>(run.start),
                static_cast<std::uint32_t>(run.len),
                level_name(run.level));
            return false;
        }
        for (std::size_t i{0}; i < count; ++i) {
            if (run.level != t.expected[half_bits].level) {
                printf(
                    "FAIL: half-bit %" PRIu32 " is %s, expected %s\n",
                    static_cast<std::uint32_t>(half_bits),
                    level_name(run.level),
                    level_name(t.expected[half_bits].level));
                return false;
            }
            ++half_bits;
        }
    }

    if (half_bits != TRANSMISSION_HALFBITS) {
        printf(
            "FAIL: recovered %" PRIu32 " half-bits, expected %" PRIu32 "\n",
            static_cast<std::uint32_t>(half_bits),
            static_cast<std::uint32_t>(TRANSMISSION_HALFBITS));
        return false;
    }
    if (index + 1 != runs.size() || runs[index].level != LEVEL_IDLE) {
        printf("FAIL: the line is not released to idle for good after the start of idle\n");
        return false;
    }

    // Every bit took both its arms of the branch, and the start of idle its length,
    // in real time.
    const std::size_t span{runs[index].start - start};
    printf(
        "  transmission span = %" PRIu32 " cycles (expected %" PRIu32 ")\n",
        static_cast<std::uint32_t>(span),
        static_cast<std::uint32_t>(TRANSMISSION_SAMPLES));
    if (span + RUN_TOLERANCE_SAMPLES < TRANSMISSION_SAMPLES || span > TRANSMISSION_SAMPLES + RUN_TOLERANCE_SAMPLES) {
        printf("FAIL: transmission duration off the nominal bit time\n");
        return false;
    }
    return true;
}

bool check_transmission(SelfTest& t, const TestFrame& frame) {
    t.restart();
    bool ok{true};
    if (!t.line_is_idle() || t.end_flag_raised()) {
        printf("FAIL: line not idle, or end flag already raised, before enable\n");
        ok = false;
    }

    t.run_capture(frame);
    ok = analyze(t) && ok;

    if (!t.line_is_idle() || !t.end_flag_raised()) {
        printf("FAIL: line not idle, or end flag not raised, after the transmission\n");
        ok = false;
    }
    return ok;
}

} // namespace

bool run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();

    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\ntx_pio_selftest: start\n");

    static SelfTest test{};
    bool ok{test.configure()};
    if (!ok) {
        printf("FAIL: could not claim the capture state machine or DMA channel\n");
    }

    if (ok) {
        printf("frame ending in 1:\n");
        ok = check_transmission(test, ENDS_IN_ONE) && ok;
        printf("frame ending in 0:\n");
        ok = check_transmission(test, ENDS_IN_ZERO) && ok;
    }

    printf("tx_pio_selftest: %s\n", ok ? "PASS" : "FAIL");

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
