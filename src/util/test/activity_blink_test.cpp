// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/util/activity_blink.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace pico_ethernet {
namespace {

constexpr std::uint32_t START_US{1'000'000};
constexpr std::uint32_t ON_US{ActivityBlink::ON_US};
constexpr std::uint32_t PERIOD_US{ActivityBlink::PERIOD_US};

void expect_one_blink(ActivityBlink& blink, std::uint32_t activity, std::uint32_t start_us) {
    EXPECT_TRUE(blink.step(activity, start_us));
    EXPECT_TRUE(blink.step(activity, start_us + ON_US - 1));
    EXPECT_FALSE(blink.step(activity, start_us + ON_US));
    EXPECT_FALSE(blink.step(activity, start_us + PERIOD_US - 1));
}

TEST(ActivityBlink, StaysDarkWithoutActivity) {
    ActivityBlink blink{};
    for (std::uint32_t t{0}; t <= 3 * PERIOD_US; t += ON_US / 2) {
        EXPECT_FALSE(blink.step(0, START_US + t));
    }
}

TEST(ActivityBlink, OneEventGivesOneBlinkThenIdle) {
    ActivityBlink blink{};
    expect_one_blink(blink, 1, START_US);
    EXPECT_FALSE(blink.step(1, START_US + PERIOD_US));
    EXPECT_FALSE(blink.step(1, START_US + 3 * PERIOD_US));
}

TEST(ActivityBlink, AnEventAfterIdleStartsABlinkImmediately) {
    ActivityBlink blink{};
    expect_one_blink(blink, 1, START_US);
    EXPECT_FALSE(blink.step(1, START_US + 2 * PERIOD_US));

    const std::uint32_t later_us{START_US + 2 * PERIOD_US + 123};
    expect_one_blink(blink, 2, later_us);
}

TEST(ActivityBlink, AnEventDuringTheOnPhaseStartsAnotherBlinkAfterTheOffPhase) {
    ActivityBlink blink{};
    EXPECT_TRUE(blink.step(1, START_US));
    EXPECT_TRUE(blink.step(2, START_US + ON_US / 2));
    EXPECT_FALSE(blink.step(2, START_US + ON_US));
    expect_one_blink(blink, 2, START_US + PERIOD_US);
    EXPECT_FALSE(blink.step(2, START_US + 2 * PERIOD_US));
}

TEST(ActivityBlink, AnEventDuringTheOffPhaseStartsAnotherBlinkAfterIt) {
    ActivityBlink blink{};
    EXPECT_TRUE(blink.step(1, START_US));
    EXPECT_FALSE(blink.step(2, START_US + ON_US));
    expect_one_blink(blink, 2, START_US + PERIOD_US);
    EXPECT_FALSE(blink.step(2, START_US + 2 * PERIOD_US));
}

TEST(ActivityBlink, ContinuousActivityGivesASteadyCadence) {
    ActivityBlink blink{};
    std::uint32_t activity{0};
    for (std::uint32_t period{0}; period < 5; ++period) {
        const std::uint32_t period_start_us{START_US + period * PERIOD_US};
        for (std::uint32_t t{0}; t < PERIOD_US; t += ON_US / 5) {
            ++activity;
            EXPECT_EQ(blink.step(activity, period_start_us + t), t < ON_US) << "period " << period << " t " << t;
        }
    }
}

TEST(ActivityBlink, ClockWrapMidBlinkKeepsTheCadence) {
    ActivityBlink blink{};
    const std::uint32_t start_us{std::numeric_limits<std::uint32_t>::max() - ON_US / 2};
    expect_one_blink(blink, 1, start_us);
    EXPECT_FALSE(blink.step(1, start_us + PERIOD_US));
}

TEST(ActivityBlink, IdleLongerThanTheClockWrapDoesNotFakeAFlash) {
    ActivityBlink blink{};
    expect_one_blink(blink, 1, START_US);
    EXPECT_FALSE(blink.step(1, START_US + PERIOD_US));

    // 2^32 us after the blink, the clock reads inside its on phase again.
    EXPECT_FALSE(blink.step(1, START_US + ON_US / 2));
}

} // namespace
} // namespace pico_ethernet
