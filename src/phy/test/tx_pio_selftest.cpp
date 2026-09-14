// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the transmit PIO program. It runs the TX state
// machine at a large clock divider so each 50 ns symbol is stretched to a value the
// CPU can sample deterministically, captures the driven pin levels from
// PIO_DBG_PADOUT, and asserts the recovered {TXP,TXN} sequence against the
// reference encoder -- Manchester polarity, LSB-first bit order, the 6/6 half-bit
// balance, and idle before and after the frame -- reporting PASS/FAIL over the UART
// (debugprobe console).
//
// The stretched half-bits are near-DC: disconnect the transmit driver inputs from
// TXP/TXN before running it so the transformer is not held under differential drive.

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "hardware/clocks.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "src/phy/phy_timing.h"
#include "src/phy/test/tx_reference.h"
#include "src/phy/tx_level.h"
#include "src/util/led_heartbeat.h"

#include "tx_level.pio.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t TXP_PIN{2};
constexpr std::uint32_t TXN_PIN{3};
constexpr std::uint32_t LED_PIN{25};

// A large clock divider stretches each 50 ns half-bit (6 PIO cycles) to 600 us,
// so a fixed 10 us CPU sample cadence yields ~60 samples per half-bit.
constexpr float CLKDIV{12000.0f};
constexpr std::uint32_t SAMPLE_PERIOD_US{10};
constexpr std::size_t SAMPLE_COUNT{6144}; // 61 ms >> the ~38 ms test frame

// Test frame exercising both branch arms and both polarities: 0x55 alternating bits
// (merged bit-boundary runs), 0xFF/0x00 runs (single-half runs), 0xD5 mixed.
constexpr std::array<std::uint8_t, 4> TEST_FRAME{0x55, 0xFF, 0x00, 0xD5};
constexpr std::size_t FRAME_BITS{TEST_FRAME.size() * 8};
constexpr std::size_t FRAME_HALFBITS{FRAME_BITS * 2};

// The {TXP,TXN} SET-group code (LEVEL_POS/NEG/IDLE) in a DBG_PADOUT word.
std::uint8_t sample_level(std::uint32_t padout) {
    return static_cast<std::uint8_t>(((padout >> TXP_PIN) & 1u) | (((padout >> TXN_PIN) & 1u) << 1));
}

struct SelfTest {
    PIO pio{pio0};
    std::uint32_t sm_tx{0};
    std::array<std::uint8_t, SAMPLE_COUNT> samples{};
    std::array<HalfBit, FRAME_HALFBITS> expected{};

    void configure() {
        const std::uint32_t offset{static_cast<std::uint32_t>(pio_add_program(pio, &tx_level_program))};
        sm_tx = static_cast<std::uint32_t>(pio_claim_unused_sm(pio, true));

        pio_gpio_init(pio, TXP_PIN);
        pio_gpio_init(pio, TXN_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm_tx, TXP_PIN, 2, true);

        pio_sm_config cfg{tx_level_program_get_default_config(offset)};
        sm_config_set_set_pins(&cfg, TXP_PIN, 2);
        sm_config_set_out_shift(&cfg, true, true, 8);
        sm_config_set_clkdiv(&cfg, CLKDIV);
        pio_sm_init(pio, sm_tx, offset, &cfg);

        // Idle the differential pair before the run.
        inject_idle();
    }

    // Preload the FIFO (the 4-byte frame fits the 4-deep FIFO), enable the SM, and
    // capture the driven pin levels at a fixed cadence.
    void run_capture() {
        encode_reference(TEST_FRAME, expected);

        for (const std::uint8_t b : TEST_FRAME) {
            pio_sm_put_blocking(pio, sm_tx, b);
        }

        pio_sm_set_enabled(pio, sm_tx, true);
        for (std::size_t i{0}; i < SAMPLE_COUNT; ++i) {
            samples[i] = sample_level(pio->dbg_padout);
            busy_wait_us(SAMPLE_PERIOD_US);
        }
    }

    [[nodiscard]] bool line_is_idle() const { return sample_level(pio->dbg_padout) == LEVEL_IDLE; }

    void inject_idle() { pio_sm_exec(pio, sm_tx, pio_encode_set(pio_pins, LEVEL_IDLE)); }
};

// A maximal run of equal polarity level, with the sample index where it starts.
struct Run {
    std::uint8_t level;
    std::size_t start;
    std::size_t len;
};

// Run-length-encode the polarity (TXP/TXN) stream. Returns the number of runs.
std::size_t polarity_runs(std::span<const std::uint8_t> samples, std::span<Run> out) {
    std::size_t n{0};
    std::uint8_t cur{samples[0]};
    std::size_t start{0};
    for (std::size_t i{1}; i < samples.size(); ++i) {
        const std::uint8_t level{samples[i]};
        if (level != cur) {
            if (n < out.size()) {
                out[n] = Run{cur, start, i - start};
                ++n;
            }
            cur = level;
            start = i;
        }
    }
    if (n < out.size()) {
        out[n] = Run{cur, start, samples.size() - start};
        ++n;
    }
    return n;
}

// Shortest non-idle run = one half-bit (6 PIO cycles) in samples.
std::size_t half_bit_unit(std::span<const Run> runs) {
    std::size_t unit{SAMPLE_COUNT};
    for (const Run& r : runs) {
        if (r.level == LEVEL_IDLE)
            continue;
        if (r.len < unit)
            unit = r.len;
    }
    return unit;
}

const char* level_name(std::uint8_t level) {
    switch (level) {
        case LEVEL_IDLE:
            return "idle";
        case LEVEL_POS:
            return "+V";
        case LEVEL_NEG:
            return "-V";
        default:
            return "??";
    }
}

// Reconstruct the half-bit level sequence from the captured samples and compare
// against the reference encoder. Prints the first mismatch on failure.
bool analyze(const SelfTest& t) {
    static std::array<Run, 128> runs_buf{};
    const std::size_t nruns{polarity_runs(t.samples, runs_buf)};
    const std::span<const Run> runs{runs_buf.data(), nruns};

    const std::size_t unit{half_bit_unit(runs)};
    printf("  half-bit unit = %u samples (expected ~60)\n", static_cast<unsigned>(unit));
    if (unit < 30 || unit > 120) {
        printf("FAIL: half-bit unit out of plausible range\n");
        return false;
    }

    // Balance tolerance: each run must be within 35% of a 1x or 2x unit.
    const std::size_t tol{(35 * unit) / 100};
    // A run far longer than two half-bits is the terminal stall (the SM holds the
    // last level after the frame drains); it caps the sequence.
    const std::size_t stall_threshold{3 * unit};

    bool ok{true};
    std::size_t nhb{0};
    for (const Run& r : runs) {
        if (r.level == LEVEL_IDLE)
            continue; // leading idle / gaps
        if (nhb >= FRAME_HALFBITS)
            break;

        std::size_t count{0};
        if (r.len > stall_threshold) {
            // Terminal trailing half-bit followed by the post-frame stall.
            count = 1;
        } else {
            count = (r.len + unit / 2) / unit; // 1 or 2 half-bits
            const std::size_t nominal{count * unit};
            const std::size_t diff{r.len > nominal ? r.len - nominal : nominal - r.len};
            if (count < 1 || count > 2 || diff > tol) {
                printf(
                    "FAIL: run at %u len %u not ~1x/2x unit (%s)\n",
                    static_cast<unsigned>(r.start),
                    static_cast<unsigned>(r.len),
                    level_name(r.level));
                ok = false;
                break;
            }
        }

        for (std::size_t k{0}; k < count && nhb < FRAME_HALFBITS; ++k) {
            const HalfBit& exp{t.expected[nhb]};
            if (r.level != exp.level) {
                printf(
                    "FAIL: half-bit %u level %s, expected %s\n",
                    static_cast<unsigned>(nhb),
                    level_name(r.level),
                    level_name(exp.level));
                ok = false;
                break;
            }
            ++nhb;
        }
        if (!ok)
            break;
    }

    if (ok && nhb != FRAME_HALFBITS) {
        printf(
            "FAIL: recovered %u half-bits, expected %u\n",
            static_cast<unsigned>(nhb),
            static_cast<unsigned>(FRAME_HALFBITS));
        ok = false;
    }
    return ok;
}

} // namespace

int run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();

    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\ntx_pio_selftest: start\n");

    static SelfTest test{};
    test.configure();

    bool ok{true};
    if (!test.line_is_idle()) {
        printf("FAIL: line not idle before enable\n");
        ok = false;
    }

    test.run_capture();
    ok = analyze(test) && ok;

    test.inject_idle();
    busy_wait_us(SAMPLE_PERIOD_US * 4);
    if (!test.line_is_idle()) {
        printf("FAIL: line not idle after TP_IDL inject\n");
        ok = false;
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

    return ok ? 0 : 1;
}

} // namespace pico_ethernet

int main() { return pico_ethernet::run_selftest(); }
