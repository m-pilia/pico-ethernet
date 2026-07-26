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

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>

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

// Length buckets (bytes) for published captures, to see whether the wire delivers
// frame-sized captures or full-buffer continuous-carrier merges. Thresholds:
// [0,64) [64,256) [256,1024) [1024,1500) [1500,+): the top bucket is a capture
// that (nearly) filled the DMA buffer, i.e. carrier never dropped.
inline constexpr std::size_t CAPTURE_HIST_BUCKETS{5};

struct InstrumentMetrics {
    InstrumentStat tud_task{};      // cost of servicing USB per lap
    InstrumentStat recover_frame{}; // cost of the bit-realign + CRC-scan pass
    InstrumentStat rx_irq{};        // cost of the end-of-frame IRQ (all fires)
    std::uint32_t main_loop_laps{0};
    std::uint32_t can_xmit_true{0}; // laps that observed the USB TX path open

    // Giant diagnosis (Step 1).
    std::array<std::uint32_t, CAPTURE_HIST_BUCKETS> capture_len_hist{};
    std::uint32_t giant_one_sfd{0};   // giant capture with <=1 SFD pattern (noise/single)
    std::uint32_t giant_multi_sfd{0}; // giant capture with >=2 SFD patterns (merge)
    std::uint32_t eof_active_discard{0}; // EOF IRQ dropped: transmitting (self-reception)
    std::uint32_t eof_empty_discard{0};  // EOF IRQ dropped: empty capture

    // RXC edge-servicing diagnosis (Step 1 follow-up). rxc_rise_count is the raw
    // per-frame edge rate at the pin (rising-edge IRQ, never disabled), to compare
    // against the falling-edge EOF servicing (rx_irq) the TX guard disables.
    std::uint32_t rxc_rise_count{0};
    std::uint64_t rxc_disabled_total_cycles{0}; // time the EOF (fall) IRQ is off
    std::uint32_t rxc_disable_windows{0};
    std::uint32_t rxc_disabled_worst_cycles{0};
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
    // Giant diagnosis (Step 1).
    CaptureHist0 = 0xC0, // < 64 B
    CaptureHist1 = 0xC1, // 64..255 B
    CaptureHist2 = 0xC2, // 256..1023 B
    CaptureHist3 = 0xC3, // 1024..1499 B
    CaptureHist4 = 0xC4, // >= 1500 B (buffer near-full: continuous carrier)
    GiantOneSfd = 0xC5,
    GiantMultiSfd = 0xC6,
    EofActiveDiscard = 0xC7,
    EofEmptyDiscard = 0xC8,
    // RXC edge-servicing diagnosis (Step 1 follow-up).
    RxcRiseCount = 0xD0,
    RxcDisabledTotalMs = 0xD1,
    RxcDisableWindows = 0xD2,
    RxcDisabledWorstNs = 0xD3,
};

// Records a published capture's length into the histogram (IRQ context).
void instrument_capture(std::size_t bytes);

// Classifies a giant capture by how many SFD patterns it contains (thread
// context): >=2 indicates back-to-back frames merged into one capture.
void instrument_giant(std::span<const std::uint8_t> raw);

void instrument_eof_active_discard();
void instrument_eof_empty_discard();

// Raw RXC rising-edge count (IRQ context).
void instrument_rxc_rise();

// Bracket the window in which the RXC falling-edge (EOF) IRQ is disabled by the TX
// self-reception guard: begin() at disable, end() at re-enable (both thread context).
void instrument_rxc_disabled_begin();
void instrument_rxc_disabled_end();

[[nodiscard]] std::optional<std::uint32_t> instrument_statistic(std::uint16_t selector);

} // namespace pico_ethernet

#endif // UTIL_INSTRUMENTATION_H
