// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_LINK_PULSE_H
#define PHY_LINK_PULSE_H

#include <cstdint>
#include <optional>

#include "src/phy/phy_timing.h"
#include "src/util/random.h"

namespace pico_ethernet {

// Normal Link Pulse cadence. A 10BASE-T PHY emits one ~100 ns positive pulse
// every 16 ms +/- 8 ms while the line is otherwise idle; the jitter keeps the
// pulse train from beating against the peer's pulses or frame traffic. Only the
// interval is pure and host-testable here -- emitting the pulse itself is the
// LEVEL state machine's job (a short symbol stream kicked by a timer).
class NlpScheduler {
  public:
    explicit constexpr NlpScheduler(std::uint32_t seed = 0x1234'5678u)
        : rng_{seed} {}

    // Next inter-pulse interval, uniform in
    // [NLP_PERIOD_MS - NLP_JITTER_MS, NLP_PERIOD_MS + NLP_JITTER_MS].
    [[nodiscard]] constexpr std::uint32_t next_interval_ms() {
        return NLP_PERIOD_MS - NLP_JITTER_MS + rng_.below(2 * NLP_JITTER_MS + 1);
    }

  private:
    Lcg rng_;
};

// The pulse train of one FLP burst carrying a 16-bit link code word, as the spacing
// from each pulse to the next. The burst opens on its first clock pulse.
class FlpBurst {
  public:
    explicit constexpr FlpBurst(std::uint16_t code_word)
        : code_word_{code_word} {}

    // Spacing from the pulse just emitted to the next one, or nullopt once the
    // final clock pulse is out.
    [[nodiscard]] constexpr std::optional<std::uint32_t> next_pulse_us() {
        if (after_data_pulse_) {
            after_data_pulse_ = false;
            ++bit_;
            return FLP_DATA_TO_CLOCK_US;
        }
        if (bit_ == FLP_CLOCK_PULSES - 1) {
            return std::nullopt;
        }
        if (((code_word_ >> bit_) & 1u) != 0) {
            after_data_pulse_ = true;
            return FLP_CLOCK_TO_DATA_US;
        }
        ++bit_;
        return FLP_CLOCK_PERIOD_US;
    }

  private:
    std::uint16_t code_word_;
    // The code word bit whose clock pulse opens the current clock period.
    std::uint32_t bit_{0};
    bool after_data_pulse_{false};
};

// What a received link pulse completed: nothing, a valid Normal Link Pulse, or the
// link code word of a well-formed FLP burst.
struct LinkPulseEvent {
    enum class Kind : std::uint8_t { None, ValidNlp, CodeWord };
    Kind kind{Kind::None};
    std::uint16_t code_word{0};
    // The burst followed the previous code word's burst within the NLP test window,
    // with nothing else in between.
    bool consecutive{false};
};

// Classifies received link pulses into FLP bursts and lone NLPs (IEEE 802.3
// Clause 28 receive windows, Clause 14 link_test_min).
//
// Pure and O(1) per pulse, so it can run in the carrier interrupt. A burst ends only
// where the next one begins, so each is reported one burst late: milliseconds,
// against arbitration and link timers counted in tens of milliseconds to seconds.
// Timestamps are microseconds from a free-running 32-bit counter, compared as
// unsigned differences so a counter wrap is transparent.
class LinkPulseDecoder {
  public:
    [[nodiscard]] constexpr LinkPulseEvent on_pulse(std::uint32_t now_us) {
        if (clocks_ > 0) {
            const std::uint32_t since_pulse_us{now_us - last_pulse_us_};
            if (since_pulse_us < FLP_TEST_MIN_US) {
                return {};
            }
            if (since_pulse_us <= FLP_TEST_MAX_US) {
                last_pulse_us_ = now_us;
                on_burst_pulse(now_us);
                return {};
            }
        }
        const LinkPulseEvent closed{close_burst(now_us)};
        start_burst(now_us);
        return closed;
    }

  private:
    constexpr void on_burst_pulse(std::uint32_t now_us) {
        if (clocks_ == FLP_CLOCK_PULSES) {
            malformed_ = true;
            return;
        }

        const std::uint32_t since_clock_us{now_us - last_clock_us_};
        if (!data_pulse_ && since_clock_us >= DATA_DETECT_MIN_US && since_clock_us <= DATA_DETECT_MAX_US) {
            data_pulse_ = true;
            return;
        }
        if (since_clock_us <= DATA_DETECT_MAX_US) {
            malformed_ = true;
            return;
        }

        // A clock pulse, closing the bit the previous one opened.
        code_word_ |= static_cast<std::uint16_t>(static_cast<std::uint16_t>(data_pulse_) << (clocks_ - 1));
        data_pulse_ = false;
        last_clock_us_ = now_us;
        ++clocks_;
    }

    [[nodiscard]] constexpr LinkPulseEvent close_burst(std::uint32_t now_us) {
        if (clocks_ == FLP_CLOCK_PULSES && !malformed_) {
            const std::uint32_t since_previous_us{burst_start_us_ - chain_start_us_};
            const bool consecutive{
                chain_ && since_previous_us >= NLP_TEST_MIN_US && since_previous_us <= NLP_TEST_MAX_US};
            chain_ = true;
            chain_start_us_ = burst_start_us_;
            return {.kind = LinkPulseEvent::Kind::CodeWord, .code_word = code_word_, .consecutive = consecutive};
        }

        chain_ = false;
        const bool lone_pulse{clocks_ == 1 && !data_pulse_ && !malformed_};
        if (!lone_pulse) {
            return {};
        }
        // Reported only once the next pulse has arrived, an NLP older than the
        // link-loss window would have expired already.
        const bool fresh{now_us - burst_start_us_ <= LINK_LOSS_US};
        const bool spaced{!nlp_seen_ || burst_start_us_ - last_nlp_us_ >= LINK_TEST_MIN_US};
        if (!fresh || !spaced) {
            return {};
        }
        nlp_seen_ = true;
        last_nlp_us_ = burst_start_us_;
        return {.kind = LinkPulseEvent::Kind::ValidNlp};
    }

    constexpr void start_burst(std::uint32_t now_us) {
        burst_start_us_ = now_us;
        last_pulse_us_ = now_us;
        last_clock_us_ = now_us;
        clocks_ = 1;
        code_word_ = 0;
        data_pulse_ = false;
        malformed_ = false;
    }

    // The burst being received.
    std::uint32_t burst_start_us_{0};
    std::uint32_t last_pulse_us_{0};
    std::uint32_t last_clock_us_{0};
    std::uint32_t clocks_{0};
    std::uint16_t code_word_{0};
    bool data_pulse_{false};
    bool malformed_{false};

    // The run of code words the next one may be consecutive to.
    std::uint32_t chain_start_us_{0};
    bool chain_{false};

    std::uint32_t last_nlp_us_{0};
    bool nlp_seen_{false};
};

} // namespace pico_ethernet

#endif // PHY_LINK_PULSE_H
