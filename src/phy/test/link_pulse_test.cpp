// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/link_pulse.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {
namespace {

TEST(NlpScheduler, IntervalsStayWithinTolerance) {
    NlpScheduler nlp{};
    for (std::size_t i{0}; i < 10000; ++i) {
        const std::uint32_t ms{nlp.next_interval_ms()};
        EXPECT_GE(ms, NLP_PERIOD_MS - NLP_JITTER_MS);
        EXPECT_LE(ms, NLP_PERIOD_MS + NLP_JITTER_MS);
    }
}

TEST(NlpScheduler, IntervalsAreJittered) {
    NlpScheduler nlp{};
    const std::uint32_t first{nlp.next_interval_ms()};
    bool varied{false};
    for (std::size_t i{0}; i < 100; ++i) {
        if (nlp.next_interval_ms() != first) {
            varied = true;
            break;
        }
    }
    EXPECT_TRUE(varied);
}

TEST(NlpScheduler, DeterministicForAGivenSeed) {
    NlpScheduler a{42u};
    NlpScheduler b{42u};
    for (std::size_t i{0}; i < 100; ++i) {
        EXPECT_EQ(a.next_interval_ms(), b.next_interval_ms());
    }
}

constexpr std::uint16_t CODE_WORDS[]{0x0000, 0xFFFF, 0x0001, 0x8000, 0x4061, 0xA5C3};

constexpr std::uint32_t START_US{1'000'000};

// Pulse times of a burst relative to its first clock pulse, as FlpBurst schedules it.
std::vector<std::uint32_t> scheduled_burst(std::uint16_t code_word) {
    FlpBurst flp{code_word};
    std::vector<std::uint32_t> at_us{0};
    for (std::optional<std::uint32_t> next{flp.next_pulse_us()}; next; next = flp.next_pulse_us()) {
        at_us.push_back(at_us.back() + *next);
    }
    return at_us;
}

// Pulse times of a burst relative to its first clock pulse: a clock pulse every
// FLP_CLOCK_PERIOD_US, and one `data_at_us` after the clock for each 1 bit, D0 first.
std::vector<std::uint32_t> burst(std::uint16_t code_word, std::uint32_t data_at_us = FLP_CLOCK_TO_DATA_US) {
    std::vector<std::uint32_t> at_us{};
    for (std::uint32_t bit{0}; bit < FLP_CLOCK_PULSES - 1; ++bit) {
        at_us.push_back(bit * FLP_CLOCK_PERIOD_US);
        if (((code_word >> bit) & 1u) != 0) {
            at_us.push_back(bit * FLP_CLOCK_PERIOD_US + data_at_us);
        }
    }
    at_us.push_back(FLP_BURST_US);
    return at_us;
}

// Feeds a burst starting at `start_us` and returns what its pulses completed.
std::vector<LinkPulseEvent>
receive(LinkPulseDecoder& decoder, std::uint32_t start_us, const std::vector<std::uint32_t>& at_us) {
    std::vector<LinkPulseEvent> events{};
    for (const std::uint32_t offset_us : at_us) {
        const LinkPulseEvent event{decoder.on_pulse(start_us + offset_us)};
        if (event.kind != LinkPulseEvent::Kind::None) {
            events.push_back(event);
        }
    }
    return events;
}

// A burst is only complete once the next one begins: receives `at_us` at START_US,
// then a pulse one burst period later, and returns what that pulse completed.
LinkPulseEvent receive_one(const std::vector<std::uint32_t>& at_us) {
    LinkPulseDecoder decoder{};
    EXPECT_TRUE(receive(decoder, START_US, at_us).empty());
    return decoder.on_pulse(START_US + FLP_BURST_PERIOD_US);
}

bool decodes_to(const LinkPulseEvent& event, std::uint16_t code_word) {
    return event.kind == LinkPulseEvent::Kind::CodeWord && event.code_word == code_word;
}

TEST(FlpBurst, SendsSeventeenClockPulsesWithADataPulsePerOneBit) {
    for (const std::uint16_t code_word : CODE_WORDS) {
        EXPECT_EQ(scheduled_burst(code_word), burst(code_word)) << std::hex << code_word;
    }
}

TEST(FlpBurst, StaysEndedAfterTheFinalClockPulse) {
    FlpBurst flp{0x0000};
    for (std::uint32_t i{0}; i < FLP_CLOCK_PULSES - 1; ++i) {
        ASSERT_TRUE(flp.next_pulse_us().has_value());
    }
    EXPECT_FALSE(flp.next_pulse_us().has_value());
    EXPECT_FALSE(flp.next_pulse_us().has_value());
}

TEST(LinkPulseDecoder, DecodesWhatFlpBurstSends) {
    for (const std::uint16_t code_word : CODE_WORDS) {
        EXPECT_TRUE(decodes_to(receive_one(scheduled_burst(code_word)), code_word)) << std::hex << code_word;
    }
}

TEST(LinkPulseDecoder, AcceptsDataPulsesAcrossTheDataDetectWindow) {
    EXPECT_TRUE(decodes_to(receive_one(burst(0xFFFF, DATA_DETECT_MIN_US)), 0xFFFF));
    EXPECT_TRUE(decodes_to(receive_one(burst(0xFFFF, DATA_DETECT_MAX_US)), 0xFFFF));
}

TEST(LinkPulseDecoder, DiscardsBurstsWithDataPulsesOutsideTheDataDetectWindow) {
    EXPECT_EQ(receive_one(burst(0xFFFF, DATA_DETECT_MIN_US - 1)).kind, LinkPulseEvent::Kind::None);
    EXPECT_EQ(receive_one(burst(0xFFFF, DATA_DETECT_MAX_US + 1)).kind, LinkPulseEvent::Kind::None);
}

TEST(LinkPulseDecoder, IgnoresPulsesCloserThanFlpTestMin) {
    std::vector<std::uint32_t> at_us{burst(0x4061)};
    at_us.insert(at_us.begin() + 1, FLP_TEST_MIN_US - 1);
    EXPECT_TRUE(decodes_to(receive_one(at_us), 0x4061));
}

TEST(LinkPulseDecoder, DiscardsBurstsWithAnExtraPulse) {
    std::vector<std::uint32_t> early{burst(0x4061)};
    early.insert(early.begin() + 1, FLP_TEST_MIN_US);
    EXPECT_EQ(receive_one(early).kind, LinkPulseEvent::Kind::None);

    std::vector<std::uint32_t> second_data{burst(0x0001)};
    second_data.insert(second_data.begin() + 2, FLP_CLOCK_TO_DATA_US + DATA_DETECT_MIN_US);
    EXPECT_EQ(receive_one(second_data).kind, LinkPulseEvent::Kind::None);

    std::vector<std::uint32_t> extra_clock{burst(0x4061)};
    extra_clock.push_back(FLP_BURST_US + FLP_CLOCK_PERIOD_US);
    EXPECT_EQ(receive_one(extra_clock).kind, LinkPulseEvent::Kind::None);
}

TEST(LinkPulseDecoder, DiscardsBurstsMissingAClockPulse) {
    // With every bit set, clock pulse k is pulse 2k of the burst.
    for (const std::uint32_t missing : {std::uint32_t{1}, FLP_CLOCK_PULSES / 2, FLP_CLOCK_PULSES - 1}) {
        std::vector<std::uint32_t> at_us{burst(0xFFFF)};
        at_us.erase(at_us.begin() + 2 * missing);
        EXPECT_EQ(receive_one(at_us).kind, LinkPulseEvent::Kind::None) << "clock " << missing;
    }
}

TEST(LinkPulseDecoder, PulsesUpToFlpTestMaxApartShareABurst) {
    LinkPulseDecoder decoder{};
    EXPECT_TRUE(receive(decoder, START_US, {0, FLP_TEST_MAX_US}).empty());
    EXPECT_EQ(decoder.on_pulse(START_US + FLP_BURST_PERIOD_US).kind, LinkPulseEvent::Kind::None);
}

TEST(LinkPulseDecoder, APulseBeyondFlpTestMaxStartsANewBurst) {
    LinkPulseDecoder decoder{};
    const std::vector<LinkPulseEvent> events{receive(decoder, START_US, {0, FLP_TEST_MAX_US + 1})};
    ASSERT_EQ(events.size(), 1u);
    EXPECT_EQ(events[0].kind, LinkPulseEvent::Kind::ValidNlp);
}

TEST(LinkPulseDecoder, LonePulsesAtLeastLinkTestMinApartAreValidNlps) {
    LinkPulseDecoder decoder{};
    const std::vector<LinkPulseEvent> events{
        receive(decoder, START_US, {0, LINK_TEST_MIN_US, 2 * LINK_TEST_MIN_US, 3 * LINK_TEST_MIN_US})};
    ASSERT_EQ(events.size(), 3u);
    for (const LinkPulseEvent& event : events) {
        EXPECT_EQ(event.kind, LinkPulseEvent::Kind::ValidNlp);
    }
}

TEST(LinkPulseDecoder, ALonePulseTooSoonAfterAValidNlpIsNotOne) {
    LinkPulseDecoder decoder{};
    const std::vector<LinkPulseEvent> events{
        receive(decoder, START_US, {0, LINK_TEST_MIN_US - 1, 2 * LINK_TEST_MIN_US, 3 * LINK_TEST_MIN_US})};
    ASSERT_EQ(events.size(), 2u);
    for (const LinkPulseEvent& event : events) {
        EXPECT_EQ(event.kind, LinkPulseEvent::Kind::ValidNlp);
    }
}

TEST(LinkPulseDecoder, ALonePulseReportedPastTheLinkLossWindowIsStale) {
    LinkPulseDecoder fresh{};
    EXPECT_EQ(receive(fresh, START_US, {0, LINK_LOSS_US}).size(), 1u);

    LinkPulseDecoder stale{};
    EXPECT_TRUE(receive(stale, START_US, {0, LINK_LOSS_US + 1}).empty());
}

// Receives two bursts `spacing_us` apart and reports whether the second is
// consecutive to the first.
bool consecutive_at(std::uint32_t spacing_us) {
    LinkPulseDecoder decoder{};
    EXPECT_TRUE(receive(decoder, START_US, burst(0x4061)).empty());
    const std::vector<LinkPulseEvent> first{receive(decoder, START_US + spacing_us, burst(0x4061))};
    const LinkPulseEvent second{decoder.on_pulse(START_US + 2 * spacing_us)};
    EXPECT_EQ(first.size(), 1u);
    EXPECT_TRUE(!first.empty() && decodes_to(first[0], 0x4061) && !first[0].consecutive);
    EXPECT_TRUE(decodes_to(second, 0x4061));
    return second.consecutive;
}

TEST(LinkPulseDecoder, CodeWordsWithinTheNlpTestWindowAreConsecutive) {
    EXPECT_TRUE(consecutive_at(NLP_TEST_MIN_US));
    EXPECT_TRUE(consecutive_at(FLP_BURST_PERIOD_US));
    EXPECT_TRUE(consecutive_at(NLP_TEST_MAX_US));
}

TEST(LinkPulseDecoder, CodeWordsOutsideTheNlpTestWindowAreNotConsecutive) {
    EXPECT_FALSE(consecutive_at(NLP_TEST_MIN_US - 1));
    EXPECT_FALSE(consecutive_at(NLP_TEST_MAX_US + 1));
}

TEST(LinkPulseDecoder, AnythingButACodeWordInBetweenBreaksTheRun) {
    const std::vector<std::uint32_t> lone_pulse{0};
    std::vector<std::uint32_t> malformed{burst(0x4061)};
    malformed.pop_back();

    for (const std::vector<std::uint32_t>& interloper : {lone_pulse, malformed}) {
        LinkPulseDecoder decoder{};
        EXPECT_TRUE(receive(decoder, START_US, burst(0x4061)).empty());
        EXPECT_EQ(receive(decoder, START_US + FLP_BURST_PERIOD_US, interloper).size(), 1u);
        receive(decoder, START_US + 2 * FLP_BURST_PERIOD_US, burst(0x4061));
        const LinkPulseEvent after{decoder.on_pulse(START_US + 3 * FLP_BURST_PERIOD_US)};
        EXPECT_TRUE(decodes_to(after, 0x4061));
        EXPECT_FALSE(after.consecutive);
    }
}

TEST(LinkPulseDecoder, SurvivesATimestampWrap) {
    constexpr std::uint32_t BEFORE_WRAP{0xFFFF'FFFFu - FLP_BURST_US / 2};
    LinkPulseDecoder decoder{};
    EXPECT_TRUE(receive(decoder, BEFORE_WRAP, burst(0xA5C3)).empty());
    const std::vector<LinkPulseEvent> first{receive(decoder, BEFORE_WRAP + FLP_BURST_PERIOD_US, burst(0xA5C3))};
    ASSERT_EQ(first.size(), 1u);
    EXPECT_TRUE(decodes_to(first[0], 0xA5C3));

    const LinkPulseEvent second{decoder.on_pulse(BEFORE_WRAP + 2 * FLP_BURST_PERIOD_US)};
    EXPECT_TRUE(decodes_to(second, 0xA5C3));
    EXPECT_TRUE(second.consecutive);
}

} // namespace
} // namespace pico_ethernet
