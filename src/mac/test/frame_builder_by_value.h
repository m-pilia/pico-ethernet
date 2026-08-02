// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_TEST_FRAME_BUILDER_BY_VALUE_H
#define MAC_TEST_FRAME_BUILDER_BY_VALUE_H

#include <cstdint>
#include <expected>
#include <span>

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"

namespace pico_ethernet {

// Returns the wire frame by value. Production TX builds into a caller-provided
// WireFrame to keep the ~1.5 KB buffer off the stack; tests are not
// stack-constrained and read more clearly with a returned value.
[[nodiscard]] constexpr std::expected<WireFrame, FrameError> build_frame(std::span<const std::uint8_t> host_frame) {
    WireFrame out{};
    if (const auto result{build_frame(host_frame, out)}; !result) {
        return std::unexpected(result.error());
    }
    return out;
}

} // namespace pico_ethernet

#endif // MAC_TEST_FRAME_BUILDER_BY_VALUE_H
