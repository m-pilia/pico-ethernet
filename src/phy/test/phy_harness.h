// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_PHY_HARNESS_H
#define PHY_TEST_PHY_HARNESS_H

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <optional>

#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "pico/stdlib.h"

#include "src/mac/ethernet_frame.h"
#include "src/phy/link_pulse.h"
#include "src/phy/phy.h"
#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// Slack for the loop and for the alarm driving the link pulses, on timings measured
// in milliseconds.
inline constexpr std::uint32_t TIMING_SLACK_US{20'000};

// Spins the PHY loop until `ready` holds or `timeout_us` has passed.
template <typename Ready>
void service_until(Phy& phy, Ready ready, std::uint32_t timeout_us) {
    const std::uint32_t start{time_us_32()};
    while (!ready() && time_us_32() - start <= timeout_us) {
        phy.service();
    }
}

inline void service_for(Phy& phy, std::uint32_t duration_us) {
    const std::uint32_t start{time_us_32()};
    while (time_us_32() - start < duration_us) {
        phy.service();
    }
}

// Builds a wire frame of `length` octets -- preamble, SFD, then all zeros -- straight
// into the PHY's next queue slot and enqueues it.
inline bool enqueue_zero_frame(Phy& phy, std::size_t length) {
    WireFrame* const slot{phy.tx_slot()};
    if (slot == nullptr) {
        printf("FAIL: could not queue the test frame\n");
        return false;
    }
    std::fill_n(slot->bytes.begin(), length, 0);
    std::fill_n(slot->bytes.begin(), PREAMBLE_LEN, PREAMBLE_BYTE);
    slot->bytes[PREAMBLE_LEN] = SFD_BYTE;
    slot->length = length;
    phy.commit_tx();
    return true;
}

// A link partner simulated on the PHY's receive inputs, driven by the CPU in place
// of the comparators: RXD is held low, so a carrier gates the decoder on but yields
// no symbols, and a link pulse is a carrier blip. Takes the pins over from the PHY,
// so it is constructed after Phy::initialize().
class PeerLink {
  public:
    explicit PeerLink(const Phy::Pins& pins)
        : rxc_{pins.rxc} {
        gpio_set_dir(pins.rxd, GPIO_OUT);
        gpio_set_dir(pins.rxc, GPIO_OUT);
        gpio_put(pins.rxd, false);
        set_carrier(false);
    }

    void set_carrier(bool asserted) { gpio_put(rxc_, asserted); }

    // Masked, so the carrier interrupt the rising edge raises on this same core
    // cannot run inside the pulse and stretch it; it is delivered once the pulse has
    // ended.
    void pulse_carrier(std::uint32_t duration_us) {
        const std::uint32_t irq_state{save_and_disable_interrupts()};
        set_carrier(true);
        busy_wait_at_least_cycles(duration_us * SYS_CYCLES_PER_US);
        set_carrier(false);
        restore_interrupts(irq_state);
    }

    void send_nlps(Phy& phy, std::uint32_t count) {
        for (std::uint32_t i{0}; i < count; ++i) {
            pulse_carrier(LINK_PULSE_US);
            service_for(phy, NLP_PERIOD_US);
        }
    }

    // Keeps the link up across a check that sends no pulses of its own. It takes
    // REFRESH_NLPS: a pulse only counts once the next one arrives, and the first may
    // come too soon after a check's own carrier did.
    void refresh_link(Phy& phy) { send_nlps(phy, REFRESH_NLPS); }

    // Sends NLPs until `done` holds or `timeout_us` has passed, and returns `done`.
    template <typename Done>
    [[nodiscard]] bool send_nlps_until(Phy& phy, Done done, std::uint32_t timeout_us) {
        const std::uint32_t start{time_us_32()};
        while (!done() && time_us_32() - start <= timeout_us) {
            pulse_carrier(LINK_PULSE_US);
            service_until(phy, done, NLP_PERIOD_US);
        }
        return done();
    }

    // The loop is only serviced between bursts: a burst is timed pulse by pulse.
    void send_flp_bursts(Phy& phy, std::uint16_t code_word, std::uint32_t count) {
        for (std::uint32_t i{0}; i < count; ++i) {
            const std::uint64_t start_us{time_us_64()};
            FlpBurst burst{code_word};
            std::uint64_t at_us{start_us};
            pulse_carrier(LINK_PULSE_US);
            for (std::optional<std::uint32_t> next_us{burst.next_pulse_us()}; next_us;
                 next_us = burst.next_pulse_us()) {
                at_us += *next_us;
                busy_wait_until(from_us_since_boot(at_us));
                pulse_carrier(LINK_PULSE_US);
            }
            const std::uint64_t next_burst_us{start_us + FLP_BURST_PERIOD_US};
            service_until(phy, [next_burst_us] { return time_us_64() >= next_burst_us; }, FLP_BURST_PERIOD_US);
        }
    }

  private:
    // Brief enough not to qualify as a carrier, like the stretched blip the carrier
    // detect makes of a real link pulse.
    static constexpr std::uint32_t LINK_PULSE_US{CARRIER_QUALIFY_US / 2};
    static constexpr std::uint32_t NLP_PERIOD_US{NLP_PERIOD_MS * 1000};
    static constexpr std::uint32_t REFRESH_NLPS{3};

    std::uint32_t rxc_;
};

} // namespace pico_ethernet

#endif // PHY_TEST_PHY_HARNESS_H
