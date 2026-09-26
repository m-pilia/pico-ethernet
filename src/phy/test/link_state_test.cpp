// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/link_state.h"

#include <gtest/gtest.h>

#include <cstdint>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t START_US{1'000'000};

// A peer link pulse arrives every 16 +/- 8 ms, comfortably inside the window.
constexpr std::uint32_t PULSE_SPACING_US{16'000};

// Feeds `count` valid link pulses spaced a pulse apart and returns the timestamp of
// the last one.
std::uint32_t pulse_train(LinkState& link, std::uint32_t from_us, std::uint32_t count) {
    std::uint32_t now_us{from_us};
    for (std::uint32_t i{0}; i < count; ++i) {
        link.on_pulse(now_us);
        now_us += PULSE_SPACING_US;
    }
    return now_us - PULSE_SPACING_US;
}

TEST(LinkState, StartsDown) {
    LinkState link{};
    EXPECT_FALSE(link.up());
}

TEST(LinkState, StaysDownBelowTheActivityThreshold) {
    LinkState link{};
    pulse_train(link, START_US, LINK_UP_EVENTS - 1);
    EXPECT_FALSE(link.up());
}

TEST(LinkState, ComesUpOnceEnoughConsecutiveEventsArrive) {
    LinkState link{};
    pulse_train(link, START_US, LINK_UP_EVENTS);
    EXPECT_TRUE(link.up());
}

TEST(LinkState, AGapBeforeTheThresholdRestartsTheCount) {
    LinkState link{};
    const std::uint32_t last_us{pulse_train(link, START_US, LINK_UP_EVENTS - 1)};

    const std::uint32_t after_gap_us{last_us + LINK_LOSS_US + 1};
    link.on_pulse(after_gap_us);
    EXPECT_FALSE(link.up());

    pulse_train(link, after_gap_us + PULSE_SPACING_US, LINK_UP_EVENTS - 1);
    EXPECT_TRUE(link.up());
}

TEST(LinkState, StaysUpWhileActivityContinues) {
    LinkState link{};
    std::uint32_t last_us{pulse_train(link, START_US, LINK_UP_EVENTS)};
    ASSERT_TRUE(link.up());

    for (std::uint32_t i{0}; i < 100; ++i) {
        link.advance(last_us + PULSE_SPACING_US);
        last_us = pulse_train(link, last_us + PULSE_SPACING_US, 1);
        ASSERT_TRUE(link.up()) << "pulse " << i;
    }
}

TEST(LinkState, DropsOnceTheLossWindowPassesWithoutActivity) {
    LinkState link{};
    const std::uint32_t last_us{pulse_train(link, START_US, LINK_UP_EVENTS)};
    ASSERT_TRUE(link.up());

    link.advance(last_us + LINK_LOSS_US);
    EXPECT_TRUE(link.up());

    link.advance(last_us + LINK_LOSS_US + 1);
    EXPECT_FALSE(link.up());
}

TEST(LinkState, ComesBackUpAfterADrop) {
    LinkState link{};
    const std::uint32_t last_us{pulse_train(link, START_US, LINK_UP_EVENTS)};
    link.advance(last_us + LINK_LOSS_US + 1);
    ASSERT_FALSE(link.up());

    pulse_train(link, last_us + 2 * LINK_LOSS_US, LINK_UP_EVENTS);
    EXPECT_TRUE(link.up());
}

TEST(LinkState, AFrameRestoresAFailedLinkAtOnce) {
    LinkState link{};
    link.on_frame(START_US);
    EXPECT_TRUE(link.up());
}

TEST(LinkState, FramesKeepAnUpLinkAliveWithoutPulses) {
    LinkState link{};
    std::uint32_t last_us{pulse_train(link, START_US, LINK_UP_EVENTS)};
    ASSERT_TRUE(link.up());

    for (std::uint32_t i{0}; i < 10; ++i) {
        last_us += LINK_LOSS_US;
        link.advance(last_us);
        link.on_frame(last_us);
        ASSERT_TRUE(link.up()) << "frame " << i;
    }
}

TEST(LinkState, DropsOnceTheLossWindowPassesAfterTheLastFrame) {
    LinkState link{};
    link.on_frame(START_US);
    link.advance(START_US + LINK_LOSS_US);
    EXPECT_TRUE(link.up());

    link.advance(START_US + LINK_LOSS_US + 1);
    EXPECT_FALSE(link.up());
}

TEST(LinkState, PulsesAfterAFrameDroppedStillNeedTheFullCount) {
    LinkState link{};
    link.on_frame(START_US);
    link.advance(START_US + LINK_LOSS_US + 1);
    ASSERT_FALSE(link.up());

    const std::uint32_t last_us{pulse_train(link, START_US + 2 * LINK_LOSS_US, LINK_UP_EVENTS - 1)};
    EXPECT_FALSE(link.up());
    pulse_train(link, last_us + PULSE_SPACING_US, 1);
    EXPECT_TRUE(link.up());
}

TEST(LinkState, SurvivesATimestampWrap) {
    constexpr std::uint32_t BEFORE_WRAP{0xFFFF'FF00u};
    LinkState link{};
    const std::uint32_t last_us{pulse_train(link, BEFORE_WRAP, LINK_UP_EVENTS)};
    EXPECT_TRUE(link.up());

    link.advance(last_us + LINK_LOSS_US);
    EXPECT_TRUE(link.up());

    link.advance(last_us + LINK_LOSS_US + 1);
    EXPECT_FALSE(link.up());
}

} // namespace
} // namespace pico_ethernet
