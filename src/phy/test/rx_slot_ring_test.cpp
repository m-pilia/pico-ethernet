// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/rx_slot_ring.h"

#include <gtest/gtest.h>

namespace pico_ethernet {
namespace {

TEST(RxSlotRing, EmptyRingHasNothingToDrain) {
    RxSlotRing<4> ring;
    EXPECT_EQ(ring.capture_slot(), 0u);
    EXPECT_FALSE(ring.peek().has_value());
    EXPECT_EQ(ring.overflow(), 0u);
}

TEST(RxSlotRing, PublishAdvancesCaptureAndCarriesLength) {
    RxSlotRing<4> ring;
    EXPECT_EQ(ring.publish(100), 1u);
    EXPECT_EQ(ring.capture_slot(), 1u);

    const auto done{ring.peek()};
    ASSERT_TRUE(done.has_value());
    EXPECT_EQ(done->slot, 0u);
    EXPECT_EQ(done->word_count, 100u);

    ring.release();
    EXPECT_FALSE(ring.peek().has_value());
}

// A ring of N slots keeps one for the live capture, so it queues at most N-1
// completed frames; the next publish is a counted overflow, not a silent loss.
TEST(RxSlotRing, FullRingDropsNewestAndCountsOverflow) {
    RxSlotRing<4> ring;
    ASSERT_EQ(ring.publish(1), 1u);
    ASSERT_EQ(ring.publish(2), 2u);
    ASSERT_EQ(ring.publish(3), 3u);

    EXPECT_EQ(ring.publish(4), 3u); // full: capture slot unchanged, frame dropped
    EXPECT_EQ(ring.overflow(), 1u);

    for (const std::size_t expected : {1u, 2u, 3u}) {
        const auto done{ring.peek()};
        ASSERT_TRUE(done.has_value());
        EXPECT_EQ(done->word_count, expected);
        ring.release();
    }
    EXPECT_FALSE(ring.peek().has_value());
    EXPECT_EQ(ring.overflow(), 1u);
}

TEST(RxSlotRing, SlotsAndCounterWrapAround) {
    RxSlotRing<4> ring;
    ASSERT_EQ(ring.publish(1), 1u);
    ASSERT_EQ(ring.publish(2), 2u);
    ring.release(); // drop slot 0; read advances to 1

    EXPECT_EQ(ring.publish(3), 3u);
    EXPECT_EQ(ring.publish(4), 0u); // capture wraps back to slot 0
    EXPECT_EQ(ring.publish(5), 0u); // ring full again -> overflow, capture stays
    EXPECT_EQ(ring.overflow(), 1u);

    for (const std::size_t expected : {2u, 3u, 4u}) {
        const auto done{ring.peek()};
        ASSERT_TRUE(done.has_value());
        EXPECT_EQ(done->word_count, expected);
        ring.release();
    }
    EXPECT_FALSE(ring.peek().has_value());
}

} // namespace
} // namespace pico_ethernet
