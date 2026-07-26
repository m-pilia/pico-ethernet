// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// TEMPORARY diagnostics for MILESTONE 1.5, Step 1 (localize the RX delivery
// limiter). Remove this module and its call sites at the end of the milestone.
//
// Lightweight on-device timers/counters read out through the existing
// GET_ETHERNET_STATISTIC path (private selector range 0xE0..0xEF), so no debug
// UART is needed. Durations are measured with the Cortex-M33 DWT cycle counter
// for sub-microsecond resolution and converted to nanoseconds on readout.

#ifndef UTIL_INSTRUMENTATION_H
#define UTIL_INSTRUMENTATION_H

#include <cstdint>
#include <optional>

namespace pico_ethernet {

// A timed section: call count, summed duration, and worst case, all in raw CPU
// cycles (converted to nanoseconds only on readout).
struct InstrumentStat {
    std::uint64_t total_cycles{0};
    std::uint32_t count{0};
    std::uint32_t worst_cycles{0};

    void record(std::uint32_t cycles) {
        total_cycles += cycles;
        ++count;
        if (cycles > worst_cycles) {
            worst_cycles = cycles;
        }
    }
};

struct InstrumentMetrics {
    InstrumentStat tud_task{};      // cost of servicing USB per lap
    InstrumentStat recover_frame{}; // cost of the bit-realign + CRC-scan pass
    InstrumentStat rx_irq{};        // cost of the end-of-frame IRQ (all fires)
    std::uint32_t main_loop_laps{0};
    std::uint32_t can_xmit_true{0}; // laps that observed the USB TX path open
};

// Written from both thread and IRQ context; read as a consistent snapshot under
// an interrupt lock in instrument_statistic().
extern InstrumentMetrics g_instrument;

// Enables the DWT cycle counter. Call once after the final system clock is set.
void instrument_init();

inline std::uint32_t instrument_now_cycles() {
    return *reinterpret_cast<volatile std::uint32_t*>(0xE0001004); // DWT->CYCCNT
}

// Times its scope in CPU cycles and folds the result into a stat on destruction.
class InstrumentScope {
  public:
    explicit InstrumentScope(InstrumentStat& stat)
        : stat_{stat},
          start_{instrument_now_cycles()} {}
    ~InstrumentScope() { stat_.record(instrument_now_cycles() - start_); }
    InstrumentScope(const InstrumentScope&) = delete;
    InstrumentScope& operator=(const InstrumentScope&) = delete;

  private:
    InstrumentStat& stat_;
    std::uint32_t start_;
};

// Private GET_ETHERNET_STATISTIC selectors carrying the diagnostics.
enum class InstrumentSelector : std::uint16_t {
    UptimeMs = 0xE0,
    MainLoopLaps = 0xE1,
    CanXmitTrue = 0xE2,
    TudTaskCount = 0xE3,
    TudTaskAvgNs = 0xE4,
    TudTaskWorstNs = 0xE5,
    RecoverFrameCount = 0xE6,
    RecoverFrameAvgNs = 0xE7,
    RecoverFrameWorstNs = 0xE8,
    RxIrqCount = 0xE9,
    RxIrqAvgNs = 0xEA,
    RxIrqWorstNs = 0xEB,
};

[[nodiscard]] std::optional<std::uint32_t> instrument_statistic(std::uint16_t selector);

} // namespace pico_ethernet

#endif // UTIL_INSTRUMENTATION_H
