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

// Start of idle (TP_IDL): every transmission, a jam included, ends with the line
// held positive from the end of its last bit cell, longer than any Manchester
// symbol, before it is released.
inline constexpr std::uint32_t TP_IDL_MIN_NS{250};
inline constexpr std::uint32_t TP_IDL_NS{300};
static_assert(TP_IDL_NS >= TP_IDL_MIN_NS);

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

// Shortest spacing from the previous valid NLP at which a lone link pulse counts as
// one (link_test_min, 2-7 ms): the receiver takes the tolerant end of the range.
inline constexpr std::uint32_t LINK_TEST_MIN_US{2'000};

// Fast Link Pulse burst (IEEE 802.3 Clause 28): 17 clock pulses, each followed by a
// data pulse for a 1 bit of the 16-bit link code word, D0 first. Both halves of a
// clock period sit in the interval-timer window, splitting its 125 us midpoint
// period.
inline constexpr std::uint32_t FLP_INTERVAL_MIN_NS{55'500};
inline constexpr std::uint32_t FLP_INTERVAL_MAX_NS{69'500};
inline constexpr std::uint32_t FLP_CLOCK_TO_DATA_US{62};
inline constexpr std::uint32_t FLP_DATA_TO_CLOCK_US{63};
inline constexpr std::uint32_t FLP_CLOCK_PERIOD_US{FLP_CLOCK_TO_DATA_US + FLP_DATA_TO_CLOCK_US};
inline constexpr std::uint32_t FLP_CLOCK_PULSES{17};
inline constexpr std::uint32_t FLP_BURST_US{(FLP_CLOCK_PULSES - 1) * FLP_CLOCK_PERIOD_US};
static_assert(FLP_CLOCK_TO_DATA_US * 1000 >= FLP_INTERVAL_MIN_NS && FLP_CLOCK_TO_DATA_US * 1000 <= FLP_INTERVAL_MAX_NS);
static_assert(FLP_DATA_TO_CLOCK_US * 1000 >= FLP_INTERVAL_MIN_NS && FLP_DATA_TO_CLOCK_US * 1000 <= FLP_INTERVAL_MAX_NS);
static_assert(FLP_CLOCK_PERIOD_US == 125);

// First pulse to first pulse of consecutive bursts, fixed inside the burst period
// range; the silence between bursts then sits inside transmit_link_burst_timer
// (5.7-22.3 ms).
inline constexpr std::uint32_t FLP_BURST_PERIOD_MIN_US{8'000};
inline constexpr std::uint32_t FLP_BURST_PERIOD_MAX_US{16'000};
inline constexpr std::uint32_t FLP_BURST_PERIOD_US{12'000};
static_assert(FLP_BURST_PERIOD_US >= FLP_BURST_PERIOD_MIN_US && FLP_BURST_PERIOD_US <= FLP_BURST_PERIOD_MAX_US);
static_assert(FLP_BURST_PERIOD_US - FLP_BURST_US >= 5'700 && FLP_BURST_PERIOD_US - FLP_BURST_US <= 22'300);

// Arbitration timers, each at the midpoint of its standard range.
inline constexpr std::uint32_t BREAK_LINK_US{(1'200'000 + 1'500'000) / 2};
inline constexpr std::uint32_t LINK_FAIL_INHIBIT_US{(750'000 + 1'000'000) / 2};
inline constexpr std::uint32_t AUTONEG_WAIT_US{(500'000 + 1'000'000) / 2};
inline constexpr std::uint32_t COMPLETE_ACK_BURSTS{(6 + 8) / 2};

// Link-pulse receive windows, each at the tolerant end of its standard range. A
// pulse closer than FLP_TEST_MIN_US to the previous one is noise, and one further
// than FLP_TEST_MAX_US starts a new burst. Within a burst, a pulse DATA_DETECT_MIN_US
// to DATA_DETECT_MAX_US after a clock pulse is a data pulse. Code words from bursts
// NLP_TEST_MIN_US to NLP_TEST_MAX_US apart are consecutive.
inline constexpr std::uint32_t FLP_TEST_MIN_US{5};
inline constexpr std::uint32_t FLP_TEST_MAX_US{185};
inline constexpr std::uint32_t DATA_DETECT_MIN_US{15};
inline constexpr std::uint32_t DATA_DETECT_MAX_US{100};
inline constexpr std::uint32_t NLP_TEST_MIN_US{5'000};
inline constexpr std::uint32_t NLP_TEST_MAX_US{150'000};
static_assert(DATA_DETECT_MIN_US < FLP_CLOCK_TO_DATA_US && FLP_CLOCK_TO_DATA_US < DATA_DETECT_MAX_US);
static_assert(DATA_DETECT_MAX_US < FLP_CLOCK_PERIOD_US && FLP_CLOCK_PERIOD_US < FLP_TEST_MAX_US);

// The system PLL is run at 120 MHz (an underclock, not an overclock) so that a
// 50 ns half-bit is an integer number of PIO cycles: 120 MHz * 50 ns = 6 cycles.
// A /1 PIO clock then places every symbol edge on an exact cycle boundary with no
// fractional-divider jitter. The USB controller runs off the independent 48 MHz
// USB PLL, so this does not disturb CDC-NCM.
inline constexpr std::uint32_t SYS_CLOCK_HZ{120'000'000};
inline constexpr std::uint32_t SYS_CYCLES_PER_US{SYS_CLOCK_HZ / 1'000'000};
inline constexpr std::uint32_t PIO_CYCLES_PER_HALF_BIT{
    static_cast<std::uint32_t>(static_cast<std::uint64_t>(SYS_CLOCK_HZ) * HALF_BIT_NS / 1'000'000'000)};
inline constexpr std::uint32_t PIO_CYCLES_PER_BIT{2 * PIO_CYCLES_PER_HALF_BIT};

static_assert(BIT_TIME_NS == 100 && HALF_BIT_NS == 50);
// The 50 ns half-bit must be an exact integer of PIO cycles at SYS_CLOCK_HZ.
static_assert(
    static_cast<std::uint64_t>(SYS_CLOCK_HZ) * HALF_BIT_NS % 1'000'000'000 == 0,
    "half-bit is not an integer number of PIO cycles at SYS_CLOCK_HZ");
static_assert(PIO_CYCLES_PER_HALF_BIT == 6 && PIO_CYCLES_PER_BIT == 12);

inline constexpr std::uint32_t PIO_CYCLES_PER_TP_IDL{TP_IDL_NS * SYS_CYCLES_PER_US / 1000};
static_assert(TP_IDL_NS * SYS_CYCLES_PER_US % 1000 == 0, "TP_IDL is not an integer number of PIO cycles");

} // namespace pico_ethernet

#endif // PHY_PHY_TIMING_H
