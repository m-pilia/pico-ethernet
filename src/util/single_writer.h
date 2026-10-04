// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef UTIL_SINGLE_WRITER_H
#define UTIL_SINGLE_WRITER_H

#include <atomic>
#include <cstdint>

namespace pico_ethernet {

// Increments a counter that only one context ever writes. With a single writer a
// relaxed load and store cannot lose an update, so no read-modify-write is needed;
// readers in other contexts see either the old or the new count.
inline void increment_single_writer(std::atomic<std::uint32_t>& counter) {
    counter.store(counter.load(std::memory_order_relaxed) + 1, std::memory_order_relaxed);
}

} // namespace pico_ethernet

#endif // UTIL_SINGLE_WRITER_H
