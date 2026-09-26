// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for autonegotiation and the full-duplex path: the
// timing and content of the FLP bursts the PHY sends, negotiation against a full
// duplex peer, a half-duplex-only peer and an NLP-only peer, full-duplex transmission
// over the peer's carrier, and renegotiation after link loss. It drives the
// production PHY with its receive inputs moved to a free pin pair, where the test
// plays the peer by CPU-timed pulses; transmission stays on the real pads, where the
// test observes it. Results are reported over the UART (debugprobe console).

#include <array>
#include <cinttypes>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>
#include <span>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "src/mac/ethernet_frame.h"
#include "src/phy/autoneg.h"
#include "src/phy/phy.h"
#include "src/phy/phy_stats.h"
#include "src/phy/phy_timing.h"
#include "src/phy/test/peer_code_words.h"
#include "src/phy/test/phy_harness.h"
#include "src/phy/test/pin_capture.h"
#include "src/phy/tx_level.h"
#include "src/util/led_heartbeat.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t LED_PIN{25};

// Transmit on the real pads; receive from a free pin pair the test drives itself.
constexpr Phy::Pins TEST_PINS{.txp = 2, .txn = 3, .rxd = 20, .rxc = 21};

constexpr std::uint32_t BACKOFF_SEED{0x5EEDu};

// The whole burst at the full PIO clock, from a little before its first pulse to a
// little after its last.
constexpr std::uint32_t CAPTURE_LEAD_US{100};
constexpr std::size_t CAPTURE_WORDS{
    (2 * CAPTURE_LEAD_US + FLP_BURST_US) * SYS_CYCLES_PER_US / PinCapture<1>::SAMPLES_PER_WORD + 1};
using Capture = PinCapture<CAPTURE_WORDS>;

// Longer than any spacing within a burst, so the pulse after a silence this long
// opens a burst.
constexpr std::uint32_t BURST_QUIET_US{1'000};
static_assert(BURST_QUIET_US > FLP_TEST_MAX_US && BURST_QUIET_US < FLP_BURST_PERIOD_US - FLP_BURST_US);

// A code word is complete only once the next burst begins, so a match on three
// takes a fourth burst to be seen, and one more for margin.
constexpr std::uint32_t MATCH_BURSTS{5};

// The link loss window runs from the last NLP that counted, and a pulse only counts
// once the next one arrives.
constexpr std::uint32_t LINK_LOSS_TIMEOUT_US{LINK_LOSS_US + NLP_PERIOD_MS * 1000 + TIMING_SLACK_US};

// A maximum-length frame, long enough to hold the injected carrier well inside it.
constexpr std::size_t TEST_FRAME_LEN{PREAMBLE_SFD_LEN + MAX_FRAME_WITH_FCS};
constexpr std::uint32_t TEST_FRAME_US{TEST_FRAME_LEN * 8 * BIT_TIME_NS / 1000};

// How long the frame is allowed to take to start once it is queued and nothing holds
// it back: a handful of loop laps, plus the state machine and DMA setup.
constexpr std::uint32_t FRAME_START_US{40};

// Far past the qualification window, so in half duplex it would be a collision.
constexpr std::uint32_t CARRIER_HOLD_US{20};
static_assert(CARRIER_HOLD_US > 4 * CARRIER_QUALIFY_US && FRAME_START_US + CARRIER_HOLD_US < TEST_FRAME_US);

// Our link pulses are ~100 ns, far too brief to poll for, so their rising edges are
// read from the pad's raw edge status, latched whether or not the interrupt is
// enabled.
void clear_tx_edge() { gpio_acknowledge_irq(TEST_PINS.txp, GPIO_IRQ_EDGE_RISE); }

bool tx_edge() {
    return ((io_bank0_hw->intr[TEST_PINS.txp / 8] >> (4 * (TEST_PINS.txp % 8))) & GPIO_IRQ_EDGE_RISE) != 0;
}

// Services the PHY until a pulse opens a burst, and returns when it did.
std::optional<std::uint32_t> wait_for_burst_start(Phy& phy, std::uint32_t timeout_us) {
    const std::uint32_t start_us{time_us_32()};
    std::uint32_t quiet_since_us{start_us};
    clear_tx_edge();
    while (time_us_32() - start_us <= timeout_us) {
        phy.service();
        if (!tx_edge()) {
            continue;
        }
        const std::uint32_t now_us{time_us_32()};
        clear_tx_edge();
        if (now_us - quiet_since_us >= BURST_QUIET_US) {
            return now_us;
        }
        quiet_since_us = now_us;
    }
    return std::nullopt;
}

// Whether the pulse that just opened a burst is followed by another within the
// burst, as an FLP burst's first clock pulse is and a lone NLP is not.
bool followed_within_burst() {
    busy_wait_us(FLP_TEST_MAX_US);
    return tx_edge();
}

std::uint32_t samples_to_ns(std::uint32_t samples) { return samples * 1000 / SYS_CYCLES_PER_US; }

bool in_interval(std::uint32_t spacing_ns) {
    return spacing_ns >= FLP_INTERVAL_MIN_NS && spacing_ns <= FLP_INTERVAL_MAX_NS;
}

// Reads the captured burst back into its code word, checking every pulse spacing
// against the interval timer on the way.
bool decode_burst(std::span<const std::uint32_t> starts, std::uint16_t& code_word) {
    code_word = 0;
    std::uint32_t clocks{1};
    std::size_t i{1};
    while (i < starts.size()) {
        const std::uint32_t spacing_ns{samples_to_ns(starts[i] - starts[i - 1])};
        if (in_interval(spacing_ns)) {
            if (i + 1 == starts.size() || !in_interval(samples_to_ns(starts[i + 1] - starts[i]))) {
                printf(
                    "FAIL: data pulse %u is not followed by a clock pulse one interval later\n",
                    static_cast<unsigned>(clocks));
                return false;
            }
            code_word = static_cast<std::uint16_t>(code_word | (1u << (clocks - 1)));
            i += 2;
        } else if (spacing_ns >= 2 * FLP_INTERVAL_MIN_NS && spacing_ns <= 2 * FLP_INTERVAL_MAX_NS) {
            // A 0 bit: two intervals from clock to clock.
            ++i;
        } else {
            printf(
                "FAIL: pulse %u follows the previous one after %u ns\n",
                static_cast<unsigned>(i),
                static_cast<unsigned>(spacing_ns));
            return false;
        }
        ++clocks;
    }
    if (clocks != FLP_CLOCK_PULSES) {
        printf(
            "FAIL: burst has %u clock pulses, expected %u\n",
            static_cast<unsigned>(clocks),
            static_cast<unsigned>(FLP_CLOCK_PULSES));
        return false;
    }
    return true;
}

bool check_flp_bursts(Phy& phy, Capture& capture) {
    const std::optional<std::uint32_t> first_us{wait_for_burst_start(phy, BREAK_LINK_US + TIMING_SLACK_US)};
    const std::optional<std::uint32_t> second_us{wait_for_burst_start(phy, FLP_BURST_PERIOD_US + TIMING_SLACK_US)};
    if (!first_us || !second_us) {
        printf("FAIL: no FLP bursts after break_link\n");
        return false;
    }
    const std::uint32_t period_us{*second_us - *first_us};
    printf("  burst period %u us\n", static_cast<unsigned>(period_us));
    if (period_us < FLP_BURST_PERIOD_MIN_US || period_us > FLP_BURST_PERIOD_MAX_US) {
        printf(
            "FAIL: burst period outside %" PRIu32 "-%" PRIu32 " us\n",
            FLP_BURST_PERIOD_MIN_US,
            FLP_BURST_PERIOD_MAX_US);
        return false;
    }

    // The bursts keep their period, so the next one is captured from just before it
    // starts. Arming clears the whole buffer, which takes a sizeable part of a burst,
    // so it is done ahead of the wait.
    capture.arm();
    const std::uint32_t capture_after_us{FLP_BURST_PERIOD_US - CAPTURE_LEAD_US};
    const std::uint32_t since_second_us{time_us_32() - *second_us};
    if (since_second_us >= capture_after_us) {
        printf("FAIL: arming the capture overran the next burst\n");
        return false;
    }
    busy_wait_us(capture_after_us - since_second_us);
    capture.start();
    capture.wait();

    std::array<std::uint32_t, 2 * FLP_CLOCK_PULSES> starts{};
    std::size_t count{0};
    for (std::size_t sample{1}; sample < Capture::SAMPLE_COUNT; ++sample) {
        if (capture.level(sample) != LEVEL_POS || capture.level(sample - 1) == LEVEL_POS) {
            continue;
        }
        if (count == starts.size()) {
            printf("FAIL: more pulses captured than a burst holds\n");
            return false;
        }
        starts[count] = static_cast<std::uint32_t>(sample);
        ++count;
    }

    if (count == 0) {
        printf("FAIL: no pulse captured\n");
        return false;
    }
    printf(
        "  %u pulses captured, the first %u us in (%u us lead intended)\n",
        static_cast<unsigned>(count),
        static_cast<unsigned>(starts[0] / SYS_CYCLES_PER_US),
        static_cast<unsigned>(CAPTURE_LEAD_US));

    std::uint16_t code_word{0};
    if (!decode_burst(std::span{starts}.first(count), code_word)) {
        return false;
    }
    printf("  burst carries 0x%04x\n", code_word);
    if (code_word != ADVERTISED_LCW) {
        printf("FAIL: code word 0x%04x, expected 0x%04x\n", code_word, ADVERTISED_LCW);
        return false;
    }
    return true;
}

// Plays an autonegotiating peer advertising `code_word` to a PHY already sending FLP
// bursts: the code word until the PHY must have matched it, then acknowledged until
// it must have matched that too, then NLPs until the link is up.
bool negotiate(Phy& phy, PeerLink& peer, std::uint16_t code_word) {
    peer.send_flp_bursts(phy, code_word, MATCH_BURSTS);
    peer.send_flp_bursts(phy, static_cast<std::uint16_t>(code_word | LCW_ACKNOWLEDGE), MATCH_BURSTS);
    const std::uint32_t timeout_us{COMPLETE_ACK_BURSTS * FLP_BURST_PERIOD_US + LINK_FAIL_INHIBIT_US};
    if (!peer.send_nlps_until(phy, [&phy] { return phy.link_up(); }, timeout_us)) {
        printf("FAIL: no link after negotiating with a peer advertising 0x%04x\n", code_word);
        return false;
    }
    return true;
}

bool keep_link_up(Phy& phy, PeerLink& peer) {
    peer.refresh_link(phy);
    if (!phy.link_up()) {
        printf("FAIL: link went down\n");
        return false;
    }
    return true;
}

// In full duplex the peer's carrier is on the other pair: a frame starts over it
// without deferring, and goes out whole without a collision.
bool check_full_duplex_transmit(Phy& phy, PeerLink& peer) {
    if (!keep_link_up(phy, peer)) {
        return false;
    }
    const std::uint32_t sent_before{phy.tx_sent()};
    const CsmaStats before{phy.csma_stats()};

    peer.set_carrier(true);
    if (!enqueue_zero_frame(phy, TEST_FRAME_LEN)) {
        peer.set_carrier(false);
        return false;
    }
    service_until(phy, [&phy] { return phy.transmitting(); }, FRAME_START_US);
    const bool started{phy.transmitting()};
    busy_wait_us(CARRIER_HOLD_US);
    peer.set_carrier(false);
    service_until(phy, [&phy] { return !phy.transmitting(); }, 2 * TEST_FRAME_US);
    phy.service();

    const CsmaStats& after{phy.csma_stats()};
    const std::uint32_t collisions{
        (after.single_collision - before.single_collision) + (after.multiple_collisions - before.multiple_collisions) +
        (after.excessive_collisions - before.excessive_collisions) + (after.late_collisions - before.late_collisions)};
    const std::uint32_t sent{phy.tx_sent() - sent_before};
    const std::uint32_t deferred{after.deferred - before.deferred};
    if (!started || sent != 1 || collisions != 0 || deferred != 0) {
        printf(
            "FAIL: full duplex over the peer's carrier: started %u, sent %u, collisions %u, deferred %u\n",
            static_cast<unsigned>(started),
            static_cast<unsigned>(sent),
            static_cast<unsigned>(collisions),
            static_cast<unsigned>(deferred));
        return false;
    }
    return true;
}

// In half duplex a frame waits for the peer's carrier to clear.
bool check_half_duplex_defers(Phy& phy, PeerLink& peer) {
    if (!keep_link_up(phy, peer)) {
        return false;
    }
    peer.set_carrier(true);
    if (!enqueue_zero_frame(phy, TEST_FRAME_LEN)) {
        peer.set_carrier(false);
        return false;
    }
    service_for(phy, FRAME_START_US + CARRIER_HOLD_US);
    const bool started{phy.transmitting()};
    peer.set_carrier(false);
    service_for(phy, 2 * TEST_FRAME_US);
    if (started) {
        printf("FAIL: transmitted over the peer's carrier, so the link is not half duplex\n");
        return false;
    }
    return true;
}

bool wait_for_link_loss(Phy& phy) {
    service_until(phy, [&phy] { return !phy.link_up(); }, LINK_LOSS_TIMEOUT_US);
    if (phy.link_up()) {
        printf("FAIL: link still up after the peer fell silent\n");
        return false;
    }
    return true;
}

bool check_renegotiates_after_link_loss(Phy& phy) {
    if (!wait_for_link_loss(phy)) {
        return false;
    }

    clear_tx_edge();
    service_for(phy, BREAK_LINK_US - TIMING_SLACK_US);
    if (tx_edge()) {
        printf("FAIL: link pulses during break_link\n");
        return false;
    }

    if (!wait_for_burst_start(phy, 2 * TIMING_SLACK_US) || !followed_within_burst()) {
        printf("FAIL: FLP bursts did not resume after break_link\n");
        return false;
    }
    return true;
}

bool check_parallel_detection(Phy& phy, PeerLink& peer) {
    if (!wait_for_link_loss(phy)) {
        return false;
    }
    const std::uint32_t down_us{time_us_32()};

    const std::uint32_t earliest_us{BREAK_LINK_US + AUTONEG_WAIT_US};
    if (!peer.send_nlps_until(phy, [&phy] { return phy.link_up(); }, earliest_us + TIMING_SLACK_US)) {
        printf("FAIL: no link by parallel detection\n");
        return false;
    }
    const std::uint32_t took_us{time_us_32() - down_us};
    printf("  parallel detection took %u us from link loss\n", static_cast<unsigned>(took_us));
    if (took_us < earliest_us) {
        printf(
            "FAIL: link up before break_link and autoneg_wait ran out (%u us)\n", static_cast<unsigned>(earliest_us));
        return false;
    }
    return true;
}

} // namespace

[[noreturn]] void run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();
    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\nautoneg_selftest: start\n");

    static Phy phy{BACKOFF_SEED, TEST_PINS};
    static Capture capture{};
    bool ok{phy.initialize()};
    if (!ok) {
        printf("FAIL: the PHY could not claim its PIO and DMA resources\n");
    }
    ok = ok && capture.configure(pio0, TEST_PINS.txp);
    if (!ok) {
        printf("FAIL: could not claim the capture state machine or DMA channel\n");
    }

    if (ok) {
        PeerLink peer{TEST_PINS};

        ok = check_flp_bursts(phy, capture) && ok;

        ok = negotiate(phy, peer, PEER_HALF_AND_FULL) && ok;
        ok = check_full_duplex_transmit(phy, peer) && ok;
        ok = check_renegotiates_after_link_loss(phy) && ok;

        ok = negotiate(phy, peer, PEER_HALF_ONLY) && ok;
        ok = check_half_duplex_defers(phy, peer) && ok;

        ok = check_parallel_detection(phy, peer) && ok;
        ok = check_half_duplex_defers(phy, peer) && ok;
    }

    printf("autoneg_selftest: %s\n", ok ? "PASS" : "FAIL");

    LedHeartbeat led{LED_PIN};
    if (ok) {
        led.good();
    } else {
        led.bad();
    }

    while (true) {
        sleep_ms(10'000);
    }
}

} // namespace pico_ethernet

int main() { pico_ethernet::run_selftest(); }
