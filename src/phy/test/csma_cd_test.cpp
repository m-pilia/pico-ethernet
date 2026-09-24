// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/csma_cd.h"

#include <gtest/gtest.h>

#include <cstdint>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t SEED{0xC0FFEEu};
constexpr std::uint32_t MEDIUM_FREE_US{1'000'000};

// Well inside the slot time, so a collision at this point is ordinary contention.
constexpr std::uint32_t EARLY_COLLISION_US{SLOT_TIME_US / 4};

// Longer than any backoff can last: the truncated window is 2^10 slot times.
constexpr std::uint32_t BACKOFF_LIMIT_US{(1u << BACKOFF_TRUNCATION) * SLOT_TIME_US};

// How long after the medium frees the transmitter is held back. Readiness only
// ever turns on as time passes, so it bisects.
std::uint32_t wait_after_free(const CsmaCd& csma, std::uint32_t free_us) {
    std::uint32_t lo{0};
    std::uint32_t hi{BACKOFF_LIMIT_US};
    while (lo < hi) {
        const std::uint32_t mid{lo + (hi - lo) / 2};
        if (csma.may_transmit(free_us + mid)) {
            hi = mid;
        } else {
            lo = mid + 1;
        }
    }
    return lo;
}

// Collides the pending frame `n` times in a row and returns the resulting backoff.
// Each retry starts from a fresh medium-free instant, as it does on the wire.
std::uint32_t backoff_after_collisions(CsmaCd& csma, std::uint32_t n) {
    std::uint32_t free_us{MEDIUM_FREE_US};
    for (std::uint32_t i{1}; i < n; ++i) {
        csma.observe_medium(false, free_us);
        EXPECT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
        free_us += BACKOFF_LIMIT_US;
    }
    csma.observe_medium(false, free_us);
    EXPECT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
    return wait_after_free(csma, free_us);
}

TEST(CsmaCd, TransmitsOnceTheMediumHasBeenFreeForTheInterframeGap) {
    CsmaCd csma{SEED};
    csma.observe_medium(false, MEDIUM_FREE_US);
    EXPECT_TRUE(csma.may_transmit(MEDIUM_FREE_US + IFG_US));
}

TEST(CsmaCd, DefersWhileCarrierIsAsserted) {
    CsmaCd csma{SEED};
    csma.observe_medium(true, MEDIUM_FREE_US);
    EXPECT_FALSE(csma.may_transmit(MEDIUM_FREE_US + 10 * IFG_US));
}

TEST(CsmaCd, DefersUntilTheInterframeGapHasElapsed) {
    CsmaCd csma{SEED};
    csma.observe_medium(false, MEDIUM_FREE_US);
    EXPECT_FALSE(csma.may_transmit(MEDIUM_FREE_US + IFG_US - 1));
    EXPECT_TRUE(csma.may_transmit(MEDIUM_FREE_US + IFG_US));
}

TEST(CsmaCd, RecordsADeferralOnlyWhenTheMediumHeldTheFrameBack) {
    CsmaCd free_medium{SEED};
    free_medium.observe_medium(false, MEDIUM_FREE_US);
    EXPECT_TRUE(free_medium.may_transmit(MEDIUM_FREE_US + IFG_US));
    EXPECT_FALSE(free_medium.deferred());

    CsmaCd busy_medium{SEED};
    busy_medium.observe_medium(true, MEDIUM_FREE_US);
    EXPECT_FALSE(busy_medium.may_transmit(MEDIUM_FREE_US + IFG_US));
    busy_medium.on_held_back(MEDIUM_FREE_US + IFG_US);
    EXPECT_TRUE(busy_medium.deferred());
}

TEST(CsmaCd, WaitingOutABackoffIsNotADeferral) {
    CsmaCd csma{SEED};
    csma.observe_medium(false, MEDIUM_FREE_US);
    ASSERT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
    EXPECT_FALSE(csma.may_transmit(MEDIUM_FREE_US));
    csma.on_held_back(MEDIUM_FREE_US);
    EXPECT_FALSE(csma.deferred());
}

TEST(CsmaCd, BackoffStaysWithinTheWindowForTheAttempt) {
    for (std::uint32_t n{1}; n <= BACKOFF_TRUNCATION + 2; ++n) {
        CsmaCd csma{SEED + n};
        const std::uint32_t wait{backoff_after_collisions(csma, n)};
        const std::uint32_t exponent{n < BACKOFF_TRUNCATION ? n : BACKOFF_TRUNCATION};
        const std::uint32_t window{((1u << exponent) - 1) * SLOT_TIME_US};
        EXPECT_GE(wait, IFG_US) << "collision " << n;
        EXPECT_LE(wait, window > IFG_US ? window : IFG_US) << "collision " << n;
        EXPECT_EQ(csma.collisions(), n) << "collision " << n;
    }
}

TEST(CsmaCd, FirstBackoffDrawsBothSlotsOfItsWindow) {
    // k is uniform in [0, 1] after one collision: k = 0 leaves only the interframe
    // gap to wait out, k = 1 one slot time.
    bool drew_zero{false};
    bool drew_one{false};
    for (std::uint32_t trial{0}; trial < 100; ++trial) {
        CsmaCd csma{SEED + trial};
        const std::uint32_t wait{backoff_after_collisions(csma, 1)};
        drew_zero |= wait == IFG_US;
        drew_one |= wait == SLOT_TIME_US;
        ASSERT_TRUE(wait == IFG_US || wait == SLOT_TIME_US) << "wait " << wait;
    }
    EXPECT_TRUE(drew_zero);
    EXPECT_TRUE(drew_one);
}

TEST(CsmaCd, BackoffSeriesIsDeterministicForASeed) {
    CsmaCd a{SEED};
    CsmaCd b{SEED};
    std::uint32_t free_us{MEDIUM_FREE_US};
    for (std::uint32_t n{1}; n < MAX_TX_ATTEMPTS; ++n) {
        a.observe_medium(false, free_us);
        b.observe_medium(false, free_us);
        ASSERT_EQ(a.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
        ASSERT_EQ(b.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
        EXPECT_EQ(wait_after_free(a, free_us), wait_after_free(b, free_us)) << "collision " << n;
        free_us += BACKOFF_LIMIT_US;
    }
}

TEST(CsmaCd, BackoffSeriesDiffersBetweenSeeds) {
    CsmaCd a{SEED};
    CsmaCd b{SEED + 1};
    std::uint32_t free_us{MEDIUM_FREE_US};
    bool differed{false};
    for (std::uint32_t n{1}; n < MAX_TX_ATTEMPTS && !differed; ++n) {
        a.observe_medium(false, free_us);
        b.observe_medium(false, free_us);
        ASSERT_EQ(a.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
        ASSERT_EQ(b.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
        differed = wait_after_free(a, free_us) != wait_after_free(b, free_us);
        free_us += BACKOFF_LIMIT_US;
    }
    EXPECT_TRUE(differed);
}

TEST(CsmaCd, AbandonsTheFrameAfterSixteenCollisions) {
    CsmaCd csma{SEED};
    csma.observe_medium(false, MEDIUM_FREE_US);
    for (std::uint32_t n{1}; n < MAX_TX_ATTEMPTS; ++n) {
        EXPECT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff) << "collision " << n;
    }
    EXPECT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Excessive);
    EXPECT_EQ(csma.collisions(), MAX_TX_ATTEMPTS);
}

TEST(CsmaCd, CollisionPastTheSlotTimeIsNotRetried) {
    CsmaCd csma{SEED};
    csma.observe_medium(false, MEDIUM_FREE_US);
    EXPECT_EQ(csma.on_collision(SLOT_TIME_US), CsmaCd::Collision::Late);
}

TEST(CsmaCd, FrameCompletionClearsTheAttemptState) {
    CsmaCd csma{SEED};
    csma.observe_medium(true, MEDIUM_FREE_US);
    csma.on_held_back(MEDIUM_FREE_US);
    ASSERT_TRUE(csma.deferred());
    csma.observe_medium(false, MEDIUM_FREE_US);
    ASSERT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);

    csma.on_frame_done();
    EXPECT_EQ(csma.collisions(), 0u);
    EXPECT_FALSE(csma.deferred());
    EXPECT_TRUE(csma.may_transmit(MEDIUM_FREE_US + IFG_US));
}

TEST(CsmaCd, DeferralAndBackoffSurviveATimestampWrap) {
    constexpr std::uint32_t BEFORE_WRAP{0xFFFF'FF00u};
    CsmaCd csma{SEED};
    csma.observe_medium(false, BEFORE_WRAP);
    EXPECT_FALSE(csma.may_transmit(BEFORE_WRAP + IFG_US - 1));
    EXPECT_TRUE(csma.may_transmit(BEFORE_WRAP + IFG_US));

    ASSERT_EQ(csma.on_collision(EARLY_COLLISION_US), CsmaCd::Collision::Backoff);
    EXPECT_LE(wait_after_free(csma, BEFORE_WRAP), SLOT_TIME_US);
}

TEST(CollisionOffset, IsMeasuredFromTheAssertNotFromTheQualification) {
    constexpr std::uint32_t TX_START_US{MEDIUM_FREE_US};
    EXPECT_EQ(
        collision_offset_us(TX_START_US, TX_START_US + EARLY_COLLISION_US + CARRIER_QUALIFY_US), EARLY_COLLISION_US);
}

TEST(CollisionOffset, CarrierAssertedBeforeTheFrameCollidesWithItsFirstBit) {
    constexpr std::uint32_t TX_START_US{MEDIUM_FREE_US};
    EXPECT_EQ(collision_offset_us(TX_START_US, TX_START_US), 0u);
    EXPECT_EQ(collision_offset_us(TX_START_US, TX_START_US + CARRIER_QUALIFY_US), 0u);
}

TEST(CollisionOffset, SurvivesATimestampWrap) {
    constexpr std::uint32_t TX_START_US{0xFFFF'FFF0u};
    EXPECT_EQ(collision_offset_us(TX_START_US, TX_START_US + SLOT_TIME_US + CARRIER_QUALIFY_US), SLOT_TIME_US);
}

} // namespace
} // namespace pico_ethernet
