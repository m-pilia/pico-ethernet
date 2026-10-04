// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_SLOT_RING_H
#define PHY_RX_SLOT_RING_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>

#include "src/util/single_writer.h"

namespace pico_ethernet {

// Single-producer/single-consumer ring that hands ownership of fixed receive
// buffers from the end-of-frame interrupt (producer) to the main-loop drain
// (consumer). It stores only slot bookkeeping -- which buffer the capture DMA is
// filling, which buffers are completed, and each completed buffer's length and
// CRC -- while the buffers themselves live beside it. The producer never writes
// into a completed or the consumer's slot, and never advances into the consumer's
// read position, so no locking is needed between the interrupt and the loop.
//
// One slot is always reserved for the in-flight capture, so a ring of N slots
// queues at most N-1 completed frames.
template <std::size_t N>
class RxSlotRing {
    static_assert(N >= 2);

  public:
    struct Completed {
        std::size_t slot;
        std::size_t word_count;
        std::uint32_t crc;
    };

    // Producer: the slot the capture DMA is currently filling.
    [[nodiscard]] std::size_t capture_slot() const { return write_.load(std::memory_order_relaxed); }

    // Producer: publish the current capture slot as a completed frame of
    // `word_count` words whose running CRC is `crc`, and move the capture on to the
    // next free slot. If the ring is full the frame is dropped (overflow counter
    // bumped) and the capture slot is left unchanged. Returns the slot to capture
    // into next.
    [[nodiscard]] std::size_t publish(std::size_t word_count, std::uint32_t crc) {
        const std::size_t write{write_.load(std::memory_order_relaxed)};
        const std::size_t next{advance(write)};
        if (next == read_.load(std::memory_order_acquire)) {
            increment_single_writer(overflow_);
            return write;
        }
        word_counts_[write] = word_count;
        crcs_[write] = crc;
        write_.store(next, std::memory_order_release);
        return next;
    }

    // Consumer: the oldest completed frame, or nullopt if none are ready.
    [[nodiscard]] std::optional<Completed> peek() const {
        const std::size_t read{read_.load(std::memory_order_relaxed)};
        if (read == write_.load(std::memory_order_acquire)) {
            return std::nullopt;
        }
        return Completed{.slot = read, .word_count = word_counts_[read], .crc = crcs_[read]};
    }

    // Consumer: release the oldest completed frame's buffer back to the producer.
    void release() {
        const std::size_t read{read_.load(std::memory_order_relaxed)};
        read_.store(advance(read), std::memory_order_release);
    }

    [[nodiscard]] std::uint32_t overflow() const { return overflow_.load(std::memory_order_relaxed); }

  private:
    static constexpr std::size_t advance(std::size_t i) { return (i + 1 == N) ? 0 : i + 1; }

    std::array<std::size_t, N> word_counts_{};
    std::array<std::uint32_t, N> crcs_{};
    std::atomic<std::size_t> write_{0};
    std::atomic<std::size_t> read_{0};
    std::atomic<std::uint32_t> overflow_{0};
};

} // namespace pico_ethernet

#endif // PHY_RX_SLOT_RING_H
