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

std::uint32_t cycles_to_ms(std::uint64_t cycles) { return static_cast<std::uint32_t>(cycles / (SYS_CLOCK_HZ / 1000)); }

std::uint32_t s_rxc_disabled_since{0};

std::uint32_t avg_ns(const InstrumentStat& stat) {
    if (stat.count == 0) {
        return 0;
    }
    return cycles_to_ns(static_cast<std::uint32_t>(stat.total_cycles / stat.count));
}

std::uint8_t bit_at(std::span<const std::uint8_t> raw, std::size_t g) {
    return static_cast<std::uint8_t>((raw[g / 8] >> (g % 8)) & 1u);
}
} // namespace

void instrument_capture(std::size_t bytes) {
    std::size_t bucket{0};
    if (bytes >= 1500) {
        bucket = 4;
    } else if (bytes >= 1024) {
        bucket = 3;
    } else if (bytes >= 256) {
        bucket = 2;
    } else if (bytes >= 64) {
        bucket = 1;
    }
    ++g_instrument.capture_len_hist[bucket];
}

void instrument_giant(std::span<const std::uint8_t> raw) {
    // SFD (LSB-first) is 1,0,1,0,1,0,1,1; the preamble 0x55 never matches (it ends
    // 1,0), so a clean single frame yields exactly one match. Data/noise can hold
    // false matches, so read >=2 as "likely a merge", not a proof.
    const std::size_t total_bits{raw.size() * 8};
    std::uint32_t sfd_count{0};
    for (std::size_t g{0}; g + 8 <= total_bits; ++g) {
        if (bit_at(raw, g) == 1 && bit_at(raw, g + 1) == 0 && bit_at(raw, g + 2) == 1 && bit_at(raw, g + 3) == 0 &&
            bit_at(raw, g + 4) == 1 && bit_at(raw, g + 5) == 0 && bit_at(raw, g + 6) == 1 && bit_at(raw, g + 7) == 1) {
            ++sfd_count;
        }
    }
    if (sfd_count >= 2) {
        ++g_instrument.giant_multi_sfd;
    } else {
        ++g_instrument.giant_one_sfd;
    }
}

void instrument_eof_active_discard() { ++g_instrument.eof_active_discard; }

void instrument_eof_empty_discard() { ++g_instrument.eof_empty_discard; }

void instrument_rxc_rise() { ++g_instrument.rxc_rise_count; }

void instrument_rxc_disabled_begin() { s_rxc_disabled_since = instrument_now_cycles(); }

void instrument_rxc_disabled_end() {
    const std::uint32_t elapsed{instrument_now_cycles() - s_rxc_disabled_since};
    g_instrument.rxc_disabled_total_cycles += elapsed;
    ++g_instrument.rxc_disable_windows;
    if (elapsed > g_instrument.rxc_disabled_worst_cycles) {
        g_instrument.rxc_disabled_worst_cycles = elapsed;
    }
}

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
        case InstrumentSelector::CaptureHist0:
            return m.capture_len_hist[0];
        case InstrumentSelector::CaptureHist1:
            return m.capture_len_hist[1];
        case InstrumentSelector::CaptureHist2:
            return m.capture_len_hist[2];
        case InstrumentSelector::CaptureHist3:
            return m.capture_len_hist[3];
        case InstrumentSelector::CaptureHist4:
            return m.capture_len_hist[4];
        case InstrumentSelector::GiantOneSfd:
            return m.giant_one_sfd;
        case InstrumentSelector::GiantMultiSfd:
            return m.giant_multi_sfd;
        case InstrumentSelector::EofActiveDiscard:
            return m.eof_active_discard;
        case InstrumentSelector::EofEmptyDiscard:
            return m.eof_empty_discard;
        case InstrumentSelector::RxcRiseCount:
            return m.rxc_rise_count;
        case InstrumentSelector::RxcDisabledTotalMs:
            return cycles_to_ms(m.rxc_disabled_total_cycles);
        case InstrumentSelector::RxcDisableWindows:
            return m.rxc_disable_windows;
        case InstrumentSelector::RxcDisabledWorstNs:
            return cycles_to_ns(m.rxc_disabled_worst_cycles);
    }
    return std::nullopt;
}

} // namespace pico_ethernet
