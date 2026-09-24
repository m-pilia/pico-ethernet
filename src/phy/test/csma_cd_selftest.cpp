// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the half-duplex link: carrier-sense deferral,
// collision abort and jam, backoff, the late-collision drop, carrier qualification,
// and the link integrity FSM. It drives the production PHY with its receive inputs moved to a free pin
// pair, so a carrier can be injected from the CPU at a chosen instant without
// fighting the comparators; transmission stays on the real pads, observed at full
// speed by a capture state machine. Results are reported over the UART (debugprobe
// console).

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <span>

#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "src/mac/ethernet_frame.h"
#include "src/phy/csma_cd.h"
#include "src/phy/phy.h"
#include "src/phy/phy_stats.h"
#include "src/phy/phy_timing.h"
#include "src/phy/test/pin_capture.h"
#include "src/phy/test/symbol_trace.h"
#include "src/phy/tx_level.h"
#include "src/util/led_heartbeat.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t LED_PIN{25};

// Transmit on the real pads; receive from a free pin pair the test drives itself.
constexpr Phy::Pins TEST_PINS{.txp = 2, .txn = 3, .rxd = 20, .rxc = 21};

// Pinned so the backoff the PHY draws can be predicted from an identical policy.
constexpr std::uint32_t BACKOFF_SEED{0x5EEDu};

constexpr std::uint32_t CYCLES_PER_US{SYS_CLOCK_HZ / 1'000'000};

// How far into the frame the injected carrier lands: comfortably past the preamble
// so the aborted octets are frame data, and far short of the slot time so the
// collision counts as ordinary contention.
constexpr std::uint32_t COLLISION_AT_US{10};
static_assert(COLLISION_AT_US * 1000 > PREAMBLE_SFD_LEN * 8 * BIT_TIME_NS);
static_assert(COLLISION_AT_US < SLOT_TIME_US);

// What the abort cannot cut short: the octets queued in the 4-entry TX FIFO and the
// one shifting out of the OSR drain ahead of the jam, and then the jam goes out.
constexpr std::uint32_t QUEUED_OCTETS{4 + 1};
constexpr std::uint32_t DRAIN_AND_JAM_US{((QUEUED_OCTETS * 8 + JAM_BITS) * BIT_TIME_NS + 999) / 1000};

// Headroom for the qualification interrupt's latency, and for this test's own delay
// in noticing that the frame has started, which the measured response includes.
constexpr std::uint32_t RESPONSE_SLACK_US{12};

// The longest the line may stay driven after the carrier appears.
constexpr std::uint32_t MAX_COLLISION_RESPONSE_US{CARRIER_QUALIFY_US + DRAIN_AND_JAM_US + RESPONSE_SLACK_US};

// How long the injected carrier is then held: past the whole collision response, so
// the medium falls free when the test says it does rather than when the jam ends.
constexpr std::uint32_t CARRIER_HOLD_US{MAX_COLLISION_RESPONSE_US + IFG_US};

// A maximum-length wire frame whose data octets are all zero. Zero octets are
// Manchester single half-bit runs while the 0xAA jam is paired runs, so the jam
// stands out in the capture from the frame data that drained ahead of it. The
// length matters too: a collision has to be injectable beyond the slot time while
// the frame is still going out.
constexpr std::size_t TEST_FRAME_LEN{PREAMBLE_SFD_LEN + MAX_FRAME_WITH_FCS};
constexpr std::uint32_t TEST_FRAME_US{TEST_FRAME_LEN * 8 * BIT_TIME_NS / 1000};
constexpr std::uint32_t LATE_COLLISION_AT_US{SLOT_TIME_US + IFG_US};
static_assert(TEST_FRAME_US > LATE_COLLISION_AT_US + CARRIER_HOLD_US);

// A carrier that drops inside the qualification window, as a link pulse and the RC
// tail of the carrier detect do.
constexpr std::uint32_t SHORT_CARRIER_US{CARRIER_QUALIFY_US / 2};

// How long the frame is allowed to take to start once it is queued and the medium
// is free: a handful of loop laps, plus the state machine and DMA setup.
constexpr std::uint32_t FRAME_START_US{40};

// Everything the capture has to span: the wait for the frame to start, the frame up
// to the collision, and the response to it.
constexpr std::size_t CAPTURE_WORDS{768};
using Capture = PinCapture<CAPTURE_WORDS>;
static_assert(Capture::SAMPLE_COUNT > (FRAME_START_US + COLLISION_AT_US + MAX_COLLISION_RESPONSE_US) * CYCLES_PER_US);

// The whole capture has to be run-length encodable: all-zero frame data is one run
// per half-bit, and a buffer that ran out would truncate the trace before the jam.
constexpr std::size_t RUN_CAPACITY{Capture::SAMPLE_COUNT / HALF_BIT_SAMPLES + 16};

// The jam is 32 bits of 0xAA: alternating data bits, so every half-bit pair but the
// first and the last merges into a two-half-bit run.
constexpr std::size_t JAM_PAIRED_RUNS{JAM_BITS - 1};

std::array<std::uint8_t, Capture::SAMPLE_COUNT> g_samples{};
std::array<LevelRun, RUN_CAPACITY> g_runs{};

// Builds the test frame straight into the PHY's next queue slot and enqueues it.
bool enqueue_test_frame(Phy& phy) {
    WireFrame* const slot{phy.tx_slot()};
    if (slot == nullptr) {
        printf("FAIL: could not queue the test frame\n");
        return false;
    }
    std::fill_n(slot->bytes.begin(), TEST_FRAME_LEN, 0);
    std::fill_n(slot->bytes.begin(), PREAMBLE_LEN, PREAMBLE_BYTE);
    slot->bytes[PREAMBLE_LEN] = SFD_BYTE;
    slot->length = TEST_FRAME_LEN;
    phy.commit_tx();
    return true;
}

void set_carrier(bool asserted) { gpio_put(TEST_PINS.rxc, asserted); }

// Masked, so the carrier interrupt the rising edge raises on this same core cannot
// run inside the pulse and stretch it; it is delivered once the pulse has ended.
void pulse_carrier(std::uint32_t duration_us) {
    const std::uint32_t irq_state{save_and_disable_interrupts()};
    set_carrier(true);
    busy_wait_at_least_cycles(duration_us * CYCLES_PER_US);
    set_carrier(false);
    restore_interrupts(irq_state);
}

// Spins the PHY loop until `ready` holds or `timeout_us` has passed.
template <typename Ready>
void service_until(Phy& phy, Ready ready, std::uint32_t timeout_us) {
    const std::uint32_t start{time_us_32()};
    while (!ready() && time_us_32() - start <= timeout_us) {
        phy.service();
    }
}

void service_for(Phy& phy, std::uint32_t duration_us) {
    const std::uint32_t start{time_us_32()};
    while (time_us_32() - start < duration_us) {
        phy.service();
    }
}

// Runs the loop long enough for anything a check left queued -- a frame waiting out
// a backoff, a retry still going out -- to finish, so a check that fails partway
// cannot leave a frame behind for the next one to trip over.
void drain(Phy& phy) { service_for(phy, 2 * TEST_FRAME_US + 4 * SLOT_TIME_US); }

// The delay the transmit policy will impose on the frame that collides after
// `collided_frames_before` earlier frames have each drawn a backoff, from a policy
// seeded identically to the one under test.
std::uint32_t predicted_backoff_us(std::uint32_t collided_frames_before) {
    CsmaCd oracle{BACKOFF_SEED};
    for (std::uint32_t i{0}; i <= collided_frames_before; ++i) {
        oracle.observe_medium(false, 0);
        (void)oracle.on_collision(0);
        if (i < collided_frames_before) {
            oracle.on_frame_done();
        }
    }
    for (std::uint32_t delay{0}; delay < (1u << BACKOFF_TRUNCATION) * SLOT_TIME_US; ++delay) {
        if (oracle.may_transmit(delay)) {
            return delay;
        }
    }
    return 0;
}

// A peer's link pulse: a carrier too brief to qualify, carrying no decodable data.
void inject_link_pulse(Phy& phy) {
    pulse_carrier(SHORT_CARRIER_US);
    phy.service();
}

bool check_link_comes_up(Phy& phy) {
    for (std::uint32_t i{0}; i < LINK_UP_EVENTS - 1; ++i) {
        inject_link_pulse(phy);
        if (phy.link_up()) {
            printf("FAIL: link came up after %u pulses, expected %u\n", i + 1, LINK_UP_EVENTS);
            return false;
        }
    }
    inject_link_pulse(phy);
    if (!phy.link_up()) {
        printf("FAIL: link still down after %u pulses\n", LINK_UP_EVENTS);
        return false;
    }
    return true;
}

bool check_link_drops_when_silent(Phy& phy) {
    service_for(phy, LINK_LOSS_US / 2);
    if (!phy.link_up()) {
        printf("FAIL: link dropped inside the loss window\n");
        return false;
    }
    service_for(phy, LINK_LOSS_US);
    if (phy.link_up()) {
        printf("FAIL: link still up after the loss window\n");
        return false;
    }
    return true;
}

bool check_defers_to_carrier(Phy& phy) {
    set_carrier(true);
    if (!enqueue_test_frame(phy)) {
        return false;
    }
    service_for(phy, 4 * IFG_US);
    if (phy.transmitting()) {
        printf("FAIL: transmitted while the carrier was asserted\n");
        set_carrier(false);
        return false;
    }

    const std::uint32_t released_us{time_us_32()};
    set_carrier(false);
    service_until(phy, [&phy] { return phy.transmitting(); }, 20 * IFG_US);
    const std::uint32_t waited{time_us_32() - released_us};
    if (!phy.transmitting()) {
        printf("FAIL: never transmitted after the carrier cleared\n");
        return false;
    }
    if (waited < IFG_US) {
        printf("FAIL: transmitted %u us after the carrier cleared, before the %u us gap\n", waited, IFG_US);
        return false;
    }
    // The excess over the gap is the end-of-frame interrupt still re-arming the
    // receiver when the gap expires, plus the transmit setup. Reported rather than
    // asserted on: how much of that interrupt overlaps the wait depends on how long
    // the wait is, so it is not a constant of the transmit path.
    printf(
        "  deferred, then started %u us after the carrier cleared (%u us beyond the gap)\n", waited, waited - IFG_US);
    return true;
}

// Reads the capture back as runs and locates the jam: the trailing group of paired
// half-bit runs, which the all-zero frame data cannot produce. `driven_samples` is
// how long the line was driven in total.
bool check_jam_shape(std::size_t& driven_samples) {
    const std::size_t run_count{level_runs(g_samples, g_runs)};
    if (run_count < JAM_PAIRED_RUNS + 2) {
        printf("FAIL: only %u level runs captured\n", static_cast<unsigned>(run_count));
        return false;
    }
    if (run_count == g_runs.size()) {
        printf("FAIL: the run buffer filled, so the trace stops short of the jam\n");
        return false;
    }

    // The transmission ends where the machine stalls holding its last symbol, which
    // the post-frame idle drive then ends.
    std::size_t last{run_count};
    while (last > 0 && (g_runs[last - 1].level == LEVEL_IDLE || !is_stall(g_runs[last - 1]))) {
        --last;
    }
    if (last == 0) {
        printf("FAIL: no end of transmission in the capture\n");
        return false;
    }
    const LevelRun& tail{g_runs[last - 1]};

    std::size_t first_driven{0};
    while (first_driven < run_count && g_runs[first_driven].level == LEVEL_IDLE) {
        ++first_driven;
    }
    driven_samples = tail.start + HALF_BIT_SAMPLES - g_runs[first_driven].start;

    // Walking back from the stall, the jam is its opening half-bit run, the pairs,
    // and the final half-bit the stall itself absorbed.
    std::size_t paired{0};
    std::size_t index{last - 1};
    while (index > 0 && half_bits_in(g_runs[index - 1]) == 2) {
        ++paired;
        --index;
    }
    if (paired != JAM_PAIRED_RUNS) {
        printf(
            "FAIL: jam is %u paired runs, expected %u (%u bits of jam)\n",
            static_cast<unsigned>(paired),
            static_cast<unsigned>(JAM_PAIRED_RUNS),
            static_cast<unsigned>(paired + 1));
        return false;
    }
    if (index == 0 || half_bits_in(g_runs[index - 1]) != 1) {
        printf("FAIL: jam does not open on a single half-bit\n");
        return false;
    }
    return true;
}

// Idles the medium so the queued frame will start on the next lap rather than
// after an interframe gap of unknown remainder, and queues it.
bool queue_frame(Phy& phy) {
    service_for(phy, 2 * IFG_US);
    return enqueue_test_frame(phy);
}

// Starts the queued frame and lets a carrier appear partway through it, as a peer
// talking over us would. Leaves the transmitter just past the end of the jam, and
// reports the instant the medium was freed -- the same edge the PHY measures its
// interframe gap and its backoff from.
bool start_and_collide(Phy& phy, std::uint32_t& medium_free_us) {
    service_until(phy, [&phy] { return phy.transmitting(); }, FRAME_START_US);
    if (!phy.transmitting()) {
        printf("FAIL: the frame never started\n");
        return false;
    }

    busy_wait_us(COLLISION_AT_US);
    set_carrier(true);
    busy_wait_us(CARRIER_HOLD_US);
    medium_free_us = time_us_32();
    set_carrier(false);
    return true;
}

bool check_collision_aborts_and_jams(Phy& phy, Capture& capture) {
    if (!queue_frame(phy)) {
        return false;
    }
    capture.arm();
    capture.start();
    std::uint32_t medium_free_us{0};
    if (!start_and_collide(phy, medium_free_us)) {
        return false;
    }
    capture.wait();
    for (std::size_t i{0}; i < g_samples.size(); ++i) {
        g_samples[i] = capture.level(i);
    }

    if (phy.transmitting()) {
        printf("FAIL: still transmitting after the jam should have drained\n");
        return false;
    }

    std::size_t driven_samples{0};
    if (!check_jam_shape(driven_samples)) {
        return false;
    }

    // What went onto the wire after the carrier appeared: the qualification window,
    // the interrupt's own latency, the octets already queued ahead of the abort, and
    // the jam.
    const std::size_t before_collision{COLLISION_AT_US * CYCLES_PER_US};
    if (driven_samples <= before_collision) {
        printf("FAIL: the line stopped before the collision was injected\n");
        return false;
    }
    const std::size_t response_samples{driven_samples - before_collision};
    printf(
        "  aborted after %u cycles of frame; collision response %u cycles (%u ns), jam %u bits\n",
        static_cast<unsigned>(before_collision),
        static_cast<unsigned>(response_samples),
        static_cast<unsigned>(response_samples * 1000 / CYCLES_PER_US),
        static_cast<unsigned>(JAM_BITS));

    if (response_samples > MAX_COLLISION_RESPONSE_US * CYCLES_PER_US) {
        printf("FAIL: kept transmitting for more than %u us after the collision\n", MAX_COLLISION_RESPONSE_US);
        return false;
    }
    return true;
}

// Measured without the capture: waiting for a capture buffer to fill would stop the
// loop that starts the retry, and the delay being measured is the point.
bool check_backoff_before_retry(Phy& phy) {
    const std::uint32_t expected{predicted_backoff_us(0)};
    std::uint32_t medium_free_us{0};
    if (!queue_frame(phy) || !start_and_collide(phy, medium_free_us)) {
        return false;
    }

    service_until(phy, [&phy] { return phy.transmitting(); }, 4 * SLOT_TIME_US);
    const std::uint32_t waited{time_us_32() - medium_free_us};
    if (!phy.transmitting()) {
        printf("FAIL: the collided frame was never retried\n");
        return false;
    }

    // The retry cannot come early: the policy holds the frame for the whole backoff,
    // and this reference is the same edge it measures from. It can be late, by
    // however long the loop takes to get the frame out -- the allowance a frame
    // start gets anywhere else in this test. The window that leaves is narrower than
    // a slot time, so the draws the policy could have made stay distinguishable.
    const std::uint32_t lower{expected > IFG_US ? expected - IFG_US : 0};
    const std::uint32_t upper{expected + FRAME_START_US};
    printf("  retried %u us after the medium fell free, expected %u us (%u..%u)\n", waited, expected, lower, upper);
    if (waited < lower || waited > upper) {
        printf("FAIL: retry delay does not match the seeded backoff\n");
        return false;
    }
    return true;
}

bool check_late_collision_is_dropped(Phy& phy) {
    const std::uint32_t late_before{phy.csma_stats().late_collisions};
    if (!queue_frame(phy)) {
        return false;
    }
    service_until(phy, [&phy] { return phy.transmitting(); }, FRAME_START_US);
    if (!phy.transmitting()) {
        printf("FAIL: the frame never started\n");
        return false;
    }

    busy_wait_us(LATE_COLLISION_AT_US);
    set_carrier(true);
    busy_wait_us(CARRIER_HOLD_US);
    set_carrier(false);

    service_until(phy, [&phy] { return !phy.transmitting(); }, 4 * TEST_FRAME_US);
    phy.service();

    if (phy.csma_stats().late_collisions != late_before + 1) {
        printf("FAIL: the collision past the slot time was not counted as late\n");
        return false;
    }

    // A late collision is a broken collision domain, not contention: the frame is
    // dropped rather than retried, so the transmitter must stay quiet.
    service_for(phy, 4 * SLOT_TIME_US);
    if (phy.transmitting()) {
        printf("FAIL: a late collision was retried\n");
        return false;
    }
    return true;
}

// `at_us` into the frame, a short carrier must leave it untouched: sent whole, and
// neither jammed nor dropped.
bool check_short_carrier_is_not_a_collision(Phy& phy, std::uint32_t at_us) {
    const std::uint32_t sent_before{phy.tx_sent()};
    const std::uint32_t underrun_before{phy.tx_underrun()};
    const std::uint32_t late_before{phy.csma_stats().late_collisions};
    if (!queue_frame(phy)) {
        return false;
    }
    service_until(phy, [&phy] { return phy.transmitting(); }, FRAME_START_US);
    if (!phy.transmitting()) {
        printf("FAIL: the frame never started\n");
        return false;
    }

    busy_wait_us(at_us);
    pulse_carrier(SHORT_CARRIER_US);

    service_until(phy, [&phy] { return !phy.transmitting(); }, 4 * TEST_FRAME_US);
    phy.service();

    // Only an attempt that went out whole counts as sent: one that collided early
    // is left queued for its retry, and one that collided late is dropped.
    const std::uint32_t sent{phy.tx_sent() - sent_before};
    const std::uint32_t underrun{phy.tx_underrun() - underrun_before};
    const std::uint32_t late{phy.csma_stats().late_collisions - late_before};
    if (sent != 1 || underrun != 0 || late != 0) {
        printf(
            "FAIL: a %u us carrier %u us into the frame disturbed it: sent %u, underrun %u, late collisions %u\n",
            static_cast<unsigned>(SHORT_CARRIER_US),
            static_cast<unsigned>(at_us),
            static_cast<unsigned>(sent),
            static_cast<unsigned>(underrun),
            static_cast<unsigned>(late));
        return false;
    }
    return true;
}

} // namespace

int run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();
    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\ncsma_cd_selftest: start\n");

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
        // The PHY configured both receive pins as inputs; the test drives them, RXD
        // held low so an injected carrier gates the decoder on but yields no symbols.
        gpio_set_dir(TEST_PINS.rxd, GPIO_OUT);
        gpio_set_dir(TEST_PINS.rxc, GPIO_OUT);
        gpio_put(TEST_PINS.rxd, false);
        set_carrier(false);

        ok = check_link_comes_up(phy) && ok;
        ok = check_link_drops_when_silent(phy) && ok;
        ok = check_link_comes_up(phy) && ok;

        // The backoff check predicts the draw the policy will make, so it has to run
        // before any other check has made one.
        ok = check_defers_to_carrier(phy) && ok;
        drain(phy);
        ok = check_backoff_before_retry(phy) && ok;
        drain(phy);
        ok = check_collision_aborts_and_jams(phy, capture) && ok;
        drain(phy);
        ok = check_late_collision_is_dropped(phy) && ok;
        // Inside the slot time and past it, where a false collision would cost a
        // retry and the frame respectively.
        drain(phy);
        ok = check_short_carrier_is_not_a_collision(phy, COLLISION_AT_US) && ok;
        drain(phy);
        ok = check_short_carrier_is_not_a_collision(phy, LATE_COLLISION_AT_US) && ok;
    }

    printf("csma_cd_selftest: %s\n", ok ? "PASS" : "FAIL");

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
