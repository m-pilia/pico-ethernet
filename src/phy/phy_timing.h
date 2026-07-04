// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_PHY_TIMING_H
#define PHY_PHY_TIMING_H

#include <cstdint>

namespace pico_ethernet {

// 10BASE-T line timing (IEEE 802.3 Clause 14). One bit = 100 ns, split into two
// 50 ns Manchester half-bits.
inline constexpr std::uint32_t BIT_RATE_BPS{10'000'000};
inline constexpr std::uint32_t BIT_TIME_NS{1'000'000'000u / BIT_RATE_BPS};
inline constexpr std::uint32_t HALF_BIT_NS{BIT_TIME_NS / 2};

// Interframe gap (96 bit-times) and half-duplex CSMA/CD parameters, kept here so
// all line-timing constants live in one place.
inline constexpr std::uint32_t IFG_BITS{96};
inline constexpr std::uint32_t IFG_NS{IFG_BITS * BIT_TIME_NS};
inline constexpr std::uint32_t SLOT_TIME_BITS{512};
inline constexpr std::uint32_t SLOT_TIME_NS{SLOT_TIME_BITS * BIT_TIME_NS};
inline constexpr std::uint32_t JAM_BITS{32};
inline constexpr std::uint32_t MAX_TX_ATTEMPTS{16};

// Normal Link Pulse: one ~100 ns positive pulse every 16 ms +/- 8 ms when idle.
inline constexpr std::uint32_t NLP_PERIOD_MS{16};
inline constexpr std::uint32_t NLP_JITTER_MS{NLP_PERIOD_MS / 2};
inline constexpr std::uint32_t NLP_PULSE_NS{BIT_TIME_NS};

// Link-loss timer window: the link is dropped after this long with no pulses or
// data.
inline constexpr std::uint32_t LINK_LOSS_MIN_MS{50};
inline constexpr std::uint32_t LINK_LOSS_MAX_MS{150};

// The system PLL is run at 120 MHz (an underclock, not an overclock) so that a
// 50 ns half-bit is an integer number of PIO cycles: 120 MHz * 50 ns = 6 cycles.
// A /1 PIO clock then places every symbol edge on an exact cycle boundary with no
// fractional-divider jitter. The USB controller runs off the independent 48 MHz
// USB PLL, so this does not disturb CDC-ECM.
inline constexpr std::uint32_t SYS_CLOCK_HZ{120'000'000};
inline constexpr std::uint32_t PIO_CYCLES_PER_HALF_BIT{
    static_cast<std::uint32_t>(static_cast<std::uint64_t>(SYS_CLOCK_HZ) * HALF_BIT_NS / 1'000'000'000)};
inline constexpr std::uint32_t PIO_CYCLES_PER_BIT{2 * PIO_CYCLES_PER_HALF_BIT};

static_assert(BIT_TIME_NS == 100 && HALF_BIT_NS == 50);
// The 50 ns half-bit must be an exact integer of PIO cycles at SYS_CLOCK_HZ.
static_assert(
    static_cast<std::uint64_t>(SYS_CLOCK_HZ) * HALF_BIT_NS % 1'000'000'000 == 0,
    "half-bit is not an integer number of PIO cycles at SYS_CLOCK_HZ");
static_assert(PIO_CYCLES_PER_HALF_BIT == 6 && PIO_CYCLES_PER_BIT == 12);

} // namespace pico_ethernet

#endif // PHY_PHY_TIMING_H
