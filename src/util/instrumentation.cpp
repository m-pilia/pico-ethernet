// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// TEMPORARY diagnostics for MILESTONE 1.5, Step 1. See instrumentation.h.

#include "src/util/instrumentation.h"

#include "hardware/sync.h" // save_and_disable_interrupts
#include "pico/time.h"     // time_us_64

#include "src/phy/phy_timing.h" // SYS_CLOCK_HZ

namespace pico_ethernet {

InstrumentMetrics g_instrument{};

namespace {
constexpr std::uintptr_t DEMCR_ADDR{0xE000EDFC};
constexpr std::uintptr_t DWT_CTRL_ADDR{0xE0001000};
constexpr std::uintptr_t DWT_CYCCNT_ADDR{0xE0001004};
constexpr std::uint32_t DEMCR_TRCENA{1u << 24};
constexpr std::uint32_t DWT_CTRL_CYCCNTENA{1u << 0};

std::uint32_t cycles_to_ns(std::uint32_t cycles) {
    return static_cast<std::uint32_t>(static_cast<std::uint64_t>(cycles) * 1'000'000'000ull / SYS_CLOCK_HZ);
}

std::uint32_t avg_ns(const InstrumentStat& stat) {
    if (stat.count == 0) {
        return 0;
    }
    return cycles_to_ns(static_cast<std::uint32_t>(stat.total_cycles / stat.count));
}
} // namespace

void instrument_init() {
    *reinterpret_cast<volatile std::uint32_t*>(DEMCR_ADDR) |= DEMCR_TRCENA;
    *reinterpret_cast<volatile std::uint32_t*>(DWT_CYCCNT_ADDR) = 0;
    *reinterpret_cast<volatile std::uint32_t*>(DWT_CTRL_ADDR) |= DWT_CTRL_CYCCNTENA;
}

std::optional<std::uint32_t> instrument_statistic(std::uint16_t selector) {
    // rx_irq is updated in interrupt context; snapshot the whole struct under an
    // interrupt lock so the 64-bit accumulators cannot be read mid-update.
    InstrumentMetrics m;
    const std::uint32_t save{save_and_disable_interrupts()};
    m = g_instrument;
    restore_interrupts(save);

    switch (static_cast<InstrumentSelector>(selector)) {
        case InstrumentSelector::UptimeMs:
            return static_cast<std::uint32_t>(time_us_64() / 1000);
        case InstrumentSelector::MainLoopLaps:
            return m.main_loop_laps;
        case InstrumentSelector::CanXmitTrue:
            return m.can_xmit_true;
        case InstrumentSelector::TudTaskCount:
            return m.tud_task.count;
        case InstrumentSelector::TudTaskAvgNs:
            return avg_ns(m.tud_task);
        case InstrumentSelector::TudTaskWorstNs:
            return cycles_to_ns(m.tud_task.worst_cycles);
        case InstrumentSelector::RecoverFrameCount:
            return m.recover_frame.count;
        case InstrumentSelector::RecoverFrameAvgNs:
            return avg_ns(m.recover_frame);
        case InstrumentSelector::RecoverFrameWorstNs:
            return cycles_to_ns(m.recover_frame.worst_cycles);
        case InstrumentSelector::RxIrqCount:
            return m.rx_irq.count;
        case InstrumentSelector::RxIrqAvgNs:
            return avg_ns(m.rx_irq);
        case InstrumentSelector::RxIrqWorstNs:
            return cycles_to_ns(m.rx_irq.worst_cycles);
    }
    return std::nullopt;
}

} // namespace pico_ethernet
