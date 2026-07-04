// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/link_pulse.h"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

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

} // namespace
} // namespace pico_ethernet
