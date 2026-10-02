// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/test/symbol_trace.h"

#include <cstddef>
#include <cstdint>
#include <span>

#include "src/phy/tx_level.h"

namespace pico_ethernet {

std::size_t level_runs(std::span<const std::uint8_t> samples, std::span<LevelRun> out) {
    if (samples.empty() || out.empty()) {
        return 0;
    }

    std::size_t count{0};
    std::uint8_t level{samples[0]};
    std::size_t start{0};
    for (std::size_t i{1}; i < samples.size() && count < out.size(); ++i) {
        if (samples[i] == level) {
            continue;
        }
        out[count] = LevelRun{.level = level, .start = start, .len = i - start};
        ++count;
        level = samples[i];
        start = i;
    }
    if (count < out.size()) {
        out[count] = LevelRun{.level = level, .start = start, .len = samples.size() - start};
        ++count;
    }
    return count;
}

std::size_t half_bits_in(const LevelRun& run) {
    const std::size_t count{(run.len + HALF_BIT_SAMPLES / 2) / HALF_BIT_SAMPLES};
    const std::size_t nominal{count * HALF_BIT_SAMPLES};
    const std::size_t error{run.len > nominal ? run.len - nominal : nominal - run.len};
    return error <= RUN_TOLERANCE_SAMPLES ? count : 0;
}

const char* level_name(std::uint8_t level) {
    switch (level) {
        case LEVEL_IDLE:
            return "idle";
        case LEVEL_POS:
            return "+V";
        case LEVEL_NEG:
            return "-V";
        default:
            return "??";
    }
}

} // namespace pico_ethernet
