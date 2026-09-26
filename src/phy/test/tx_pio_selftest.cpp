// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the transmit PIO program. The TX state machine
// runs at its real /1 clock while a capture state machine, started in lockstep with
// it, records the driven {TXP,TXN} levels one sample per PIO cycle. The recovered
// sequence is asserted against the reference encoder -- Manchester polarity,
// LSB-first bit order, idle before and after the frame -- and its run lengths
// against the nominal half-bit, so the 6/6 half-bit split and the 12-cycle branch
// balance are checked as real time rather than as relative cycle counts. PASS/FAIL
// is reported over the UART (debugprobe console).

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

// Test frame exercising both branch arms and both polarities: 0x55 alternating bits
// (merged bit-boundary runs), 0xFF/0x00 runs (single-half runs), 0xD5 mixed.
constexpr std::array<std::uint8_t, 4> TEST_FRAME{0x55, 0xFF, 0x00, 0xD5};
constexpr std::size_t FRAME_BITS{TEST_FRAME.size() * 8};
constexpr std::size_t FRAME_HALFBITS{FRAME_BITS * 2};
constexpr std::size_t FRAME_SAMPLES{FRAME_HALFBITS * HALF_BIT_SAMPLES};

// Room for the leading idle and the terminal stall on either side of the frame.
constexpr std::size_t CAPTURE_WORDS{64};
using Capture = PinCapture<CAPTURE_WORDS>;
static_assert(Capture::SAMPLE_COUNT > 2 * FRAME_SAMPLES);

// The {TXP,TXN} SET-group code (LEVEL_POS/NEG/IDLE) in a DBG_PADOUT word.
std::uint8_t padout_level(std::uint32_t padout) {
    return static_cast<std::uint8_t>(((padout >> TXP_PIN) & 1u) | (((padout >> TXN_PIN) & 1u) << 1));
}

struct SelfTest {
    PIO pio{pio0};
    std::uint32_t sm_tx{0};
    Capture capture{};
    std::array<std::uint8_t, Capture::SAMPLE_COUNT> samples{};
    std::array<HalfBit, FRAME_HALFBITS> expected{};

    [[nodiscard]] bool configure() {
        const std::uint32_t offset{static_cast<std::uint32_t>(pio_add_program(pio, &tx_level_program))};
        sm_tx = static_cast<std::uint32_t>(pio_claim_unused_sm(pio, true));

        pio_gpio_init(pio, TXP_PIN);
        pio_gpio_init(pio, TXN_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm_tx, TXP_PIN, 2, true);

        pio_sm_config cfg{tx_level_program_get_default_config(offset)};
        sm_config_set_set_pins(&cfg, TXP_PIN, 2);
        sm_config_set_out_shift(&cfg, true, true, 8);
        sm_config_set_clkdiv(&cfg, 1.0f);
        pio_sm_init(pio, sm_tx, offset, &cfg);

        inject_idle();
        return capture.configure(pio, TXP_PIN);
    }

    // Preloads the FIFO (the 4-byte frame fits the 4-deep FIFO) so the frame starts
    // the cycle the machine is enabled, then runs transmitter and capture together.
    void run_capture() {
        encode_reference(TEST_FRAME, expected);

        for (const std::uint8_t octet : TEST_FRAME) {
            pio_sm_put_blocking(pio, sm_tx, octet);
        }

        capture.arm();
        pio_enable_sm_mask_in_sync(pio, (1u << sm_tx) | capture.sm_mask());
        capture.wait();
        pio_sm_set_enabled(pio, sm_tx, false);

        for (std::size_t i{0}; i < samples.size(); ++i) {
            samples[i] = capture.level(i);
        }
    }

    [[nodiscard]] bool line_is_idle() const { return padout_level(pio->dbg_padout) == LEVEL_IDLE; }

    void inject_idle() { pio_sm_exec(pio, sm_tx, pio_encode_set(pio_pins, LEVEL_IDLE)); }
};

// Reconstructs the half-bit sequence from the captured levels and compares it
// against the reference encoder, then checks the frame's total duration. Prints the
// first mismatch on failure.
bool analyze(const SelfTest& t) {
    static std::array<LevelRun, 2 * FRAME_HALFBITS> runs_buf{};
    const std::size_t run_count{level_runs(t.samples, runs_buf)};
    const std::span<const LevelRun> runs{runs_buf.data(), run_count};

    std::size_t half_bits{0};
    std::size_t frame_start{0};
    std::size_t frame_end{0};
    for (const LevelRun& run : runs) {
        if (run.level == LEVEL_IDLE) {
            continue; // leading idle before the first symbol
        }
        if (half_bits >= FRAME_HALFBITS) {
            break;
        }
        if (half_bits == 0) {
            frame_start = run.start;
        }

        // The last half-bit merges into the post-frame stall, where the machine
        // holds the level it drove last, so its own duration cannot be measured.
        const std::size_t count{is_stall(run) ? 1 : half_bits_in(run)};
        if (count == 0) {
            printf(
                "FAIL: run at %" PRIu32 " len %" PRIu32 " is not one or two half-bits (%s)\n",
                static_cast<std::uint32_t>(run.start),
                static_cast<std::uint32_t>(run.len),
                level_name(run.level));
            return false;
        }
        frame_end = run.start + count * HALF_BIT_SAMPLES;

        for (std::size_t i{0}; i < count && half_bits < FRAME_HALFBITS; ++i) {
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

    if (half_bits != FRAME_HALFBITS) {
        printf(
            "FAIL: recovered %" PRIu32 " half-bits, expected %" PRIu32 "\n",
            static_cast<std::uint32_t>(half_bits),
            static_cast<std::uint32_t>(FRAME_HALFBITS));
        return false;
    }

    // Every bit took both its arms of the branch, in real time.
    const std::size_t span{frame_end - frame_start};
    printf(
        "  frame span = %" PRIu32 " cycles (expected %" PRIu32 ")\n",
        static_cast<std::uint32_t>(span),
        static_cast<std::uint32_t>(FRAME_SAMPLES));
    if (span + RUN_TOLERANCE_SAMPLES < FRAME_SAMPLES || span > FRAME_SAMPLES + RUN_TOLERANCE_SAMPLES) {
        printf("FAIL: frame duration off the nominal bit time\n");
        return false;
    }
    return true;
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
        if (!test.line_is_idle()) {
            printf("FAIL: line not idle before enable\n");
            ok = false;
        }

        test.run_capture();
        ok = analyze(test) && ok;

        test.inject_idle();
        if (!test.line_is_idle()) {
            printf("FAIL: line not idle after TP_IDL inject\n");
            ok = false;
        }
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
