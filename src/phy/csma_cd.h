// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_CSMA_CD_H
#define PHY_CSMA_CD_H

#include <algorithm>
#include <cassert>
#include <cstdint>

#include "src/phy/duplex.h"
#include "src/phy/phy_timing.h"
#include "src/util/random.h"

namespace pico_ethernet {

// How far into the frame started at `tx_start_us` the carrier that qualified at
// `qualified_us` actually appeared: qualification lags the assert by the whole
// qualification window, and early and late collisions are told apart by the assert.
// A carrier that appeared before the frame started collides with its first bit.
[[nodiscard]] constexpr std::uint32_t collision_offset_us(std::uint32_t tx_start_us, std::uint32_t qualified_us) {
    const std::uint32_t since_start_us{qualified_us - tx_start_us};
    return since_start_us > CARRIER_QUALIFY_US ? since_start_us - CARRIER_QUALIFY_US : 0;
}

// Transmit policy (IEEE 802.3 Clause 4): when a queued frame may go onto the medium,
// and what a collision does to it. Half duplex is CSMA/CD; full duplex has no shared
// medium to sense or collide on, so only the interframe gap is left to keep.
//
// Pure and clock-free -- every timestamp arrives as an argument, so the hardware
// timer, the carrier-detect pin and the transmitter all stay with the caller and
// the whole policy is host-testable. Timestamps are microseconds from a free-running
// 32-bit counter, compared as unsigned differences so a counter wrap is transparent.
class CsmaCd {
  public:
    // What a collision leaves the frame in: retransmit after the backoff, or drop
    // it. A collision detected after the slot time has elapsed means the collision
    // domain's timing assumptions are broken (a duplex mismatch or an over-long
    // segment), not contention -- the peer may not have detected it at all, so
    // backoff would no longer converge and the frame is not retried.
    enum class Collision : std::uint8_t { Backoff, Excessive, Late };

    explicit constexpr CsmaCd(std::uint32_t seed)
        : rng_{seed} {}

    constexpr void set_duplex(Duplex duplex) { duplex_ = duplex; }

    // `busy` is the carrier-detect level now; `free_since_us` is when the medium
    // last became free, which is where the interframe gap is measured from.
    constexpr void observe_medium(bool busy, std::uint32_t free_since_us) {
        busy_ = busy;
        free_since_us_ = free_since_us;
    }

    // Whether the pending frame may start now: the medium has been free for the
    // interframe gap, and any backoff has run out. In full duplex the gap runs from
    // the peer's frames too, which can only hold a frame back for longer than needed.
    [[nodiscard]] constexpr bool may_transmit(std::uint32_t now_us) const {
        return medium_idle(now_us) && now_us - backoff_from_us_ >= backoff_us_;
    }

    // Records that the pending frame was held back at `now_us`. Only waiting on the
    // medium before the first attempt is a deferral; waiting out a backoff is not.
    constexpr void on_held_back(std::uint32_t now_us) {
        deferred_ |= duplex_ == Duplex::Half && collisions_ == 0 && !medium_idle(now_us);
    }

    // `elapsed_us` is how long the aborted frame had been transmitting, which places
    // the collision inside or beyond the slot time.
    [[nodiscard]] constexpr Collision on_collision(std::uint32_t elapsed_us) {
        assert(duplex_ == Duplex::Half);
        ++collisions_;
        if (elapsed_us >= SLOT_TIME_US) {
            return Collision::Late;
        }
        if (collisions_ >= MAX_TX_ATTEMPTS) {
            return Collision::Excessive;
        }

        // Truncated binary exponential backoff: k slot times, k uniform in
        // [0, 2^min(n, 10) - 1] after the n-th collision. Measured from the end of
        // the jam rather than from now, so a late lap of the caller's loop does not
        // stretch the slot grid.
        const std::uint32_t exponent{std::min(collisions_, BACKOFF_TRUNCATION)};
        backoff_us_ = rng_.below(1u << exponent) * SLOT_TIME_US;
        backoff_from_us_ = free_since_us_;
        return Collision::Backoff;
    }

    constexpr void on_frame_done() {
        collisions_ = 0;
        deferred_ = false;
        backoff_us_ = 0;
    }

    [[nodiscard]] constexpr std::uint32_t collisions() const { return collisions_; }
    [[nodiscard]] constexpr bool deferred() const { return deferred_; }

  private:
    [[nodiscard]] constexpr bool medium_idle(std::uint32_t now_us) const {
        return (duplex_ == Duplex::Full || !busy_) && now_us - free_since_us_ >= IFG_US;
    }

    Lcg rng_;
    Duplex duplex_{Duplex::Half};
    std::uint32_t free_since_us_{0};
    std::uint32_t backoff_from_us_{0};
    std::uint32_t backoff_us_{0};
    std::uint32_t collisions_{0};
    bool busy_{false};
    bool deferred_{false};
};

} // namespace pico_ethernet

#endif // PHY_CSMA_CD_H
