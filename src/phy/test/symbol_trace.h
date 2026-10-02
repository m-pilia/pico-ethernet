// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_SYMBOL_TRACE_H
#define PHY_TEST_SYMBOL_TRACE_H

#include <cstddef>
#include <cstdint>
#include <span>

#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// Reading a full-speed PinCapture back as 10BASE-T symbols. One sample is one PIO
// cycle, so a half-bit is PIO_CYCLES_PER_HALF_BIT samples and durations are checked
// against that rather than against a unit inferred from the capture itself.

inline constexpr std::size_t HALF_BIT_SAMPLES{PIO_CYCLES_PER_HALF_BIT};

// A maximal run of one driven {TXP,TXN} level, measured in samples.
struct LevelRun {
    std::uint8_t level;
    std::size_t start;
    std::size_t len;
};

// The margin allowed on a run length. A transition seen by the capture is delayed
// by the input synchronizer, equally at both ends of a run, so only the sample the
// transition itself lands on is in question.
inline constexpr std::size_t RUN_TOLERANCE_SAMPLES{1};

// Run-length encodes captured levels. Returns the number of runs written, which
// stops at the capacity of `out`.
std::size_t level_runs(std::span<const std::uint8_t> samples, std::span<LevelRun> out);

// How many half-bits a run spans: the nearest whole number of nominal half-bits
// when its length matches that many, 0 when it matches none.
std::size_t half_bits_in(const LevelRun& run);

const char* level_name(std::uint8_t level);

} // namespace pico_ethernet

#endif // PHY_TEST_SYMBOL_TRACE_H
