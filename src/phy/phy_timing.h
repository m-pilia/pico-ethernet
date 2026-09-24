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

// The truncation point of the binary exponential backoff: after the n-th collision
// the draw spans [0, 2^min(n, BACKOFF_TRUNCATION) - 1] slot times.
inline constexpr std::uint32_t BACKOFF_TRUNCATION{10};

// Neither gap is a whole number of microseconds (9.6 and 51.2), and the deferral
// timers run on the 1 us system timer, so both round up: waiting marginally longer
// than the standard requires is always legal, waiting less is not.
inline constexpr std::uint32_t IFG_US{(IFG_NS + 999) / 1000};
inline constexpr std::uint32_t SLOT_TIME_US{(SLOT_TIME_NS + 999) / 1000};

// How long a carrier seen while we transmit must stay asserted before it counts as a
// collision. It has to outlast a link pulse together with the tail the RC filter on
// RXC adds to it, and stay well short of the shortest carrier a colliding station
// produces (its preamble and SFD, then the jam).
inline constexpr std::uint32_t CARRIER_QUALIFY_US{4};

// Normal Link Pulse: one ~100 ns positive pulse every 16 ms +/- 8 ms when idle.
inline constexpr std::uint32_t NLP_PERIOD_MS{16};
inline constexpr std::uint32_t NLP_JITTER_MS{NLP_PERIOD_MS / 2};
inline constexpr std::uint32_t NLP_PULSE_NS{BIT_TIME_NS};

// Link-loss timer window: the link is dropped after this long with no pulses or
// data.
inline constexpr std::uint32_t LINK_LOSS_MIN_MS{50};
inline constexpr std::uint32_t LINK_LOSS_MAX_MS{150};
inline constexpr std::uint32_t LINK_LOSS_MS{100};
inline constexpr std::uint32_t LINK_LOSS_US{LINK_LOSS_MS * 1000};
static_assert(LINK_LOSS_MS >= LINK_LOSS_MIN_MS && LINK_LOSS_MS <= LINK_LOSS_MAX_MS);

// Consecutive link-activity events needed to declare the link up. Clause 14 allows
// 2 to 10; three tolerates a lost pulse without letting a lone noise blip flap the
// host's link state.
inline constexpr std::uint32_t LINK_UP_EVENTS{3};

// A link pulse due while the medium is busy is held back rather than corrupting the
// frame in flight, and retried this soon -- an order of magnitude below the cadence
// itself, so a busy line barely shifts the pulse train.
inline constexpr std::uint32_t NLP_RETRY_MS{1};

// The system PLL is run at 120 MHz (an underclock, not an overclock) so that a
// 50 ns half-bit is an integer number of PIO cycles: 120 MHz * 50 ns = 6 cycles.
// A /1 PIO clock then places every symbol edge on an exact cycle boundary with no
// fractional-divider jitter. The USB controller runs off the independent 48 MHz
// USB PLL, so this does not disturb CDC-NCM.
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
