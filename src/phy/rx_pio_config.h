// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_PIO_CONFIG_H
#define PHY_RX_PIO_CONFIG_H

#include <cstddef>
#include <cstdint>

#include "hardware/pio.h"

namespace pico_ethernet {

// The RX Manchester SM autopushes a full 32-bit word per FIFO entry, so DMA lands
// four recovered octets per word -- a quarter of the traffic of a byte-per-word
// push. Both the octet unpacking and the landing-buffer sizing derive from these.
inline constexpr std::uint32_t RX_FIFO_WORD_BITS{32};
inline constexpr std::size_t RX_OCTETS_PER_WORD{RX_FIFO_WORD_BITS / 8};

// `cfg` must already be seeded from rx_manchester_program_get_default_config().
// RXD is the IN base and the JMP pin; RXC (carrier gate) is IN base + 1.
inline void configure_rx_shift(pio_sm_config& cfg, std::uint32_t rxd_pin) {
    sm_config_set_in_pins(&cfg, rxd_pin);
    sm_config_set_jmp_pin(&cfg, rxd_pin);
    sm_config_set_in_shift(&cfg, true, true, RX_FIFO_WORD_BITS); // shift right, autopush a full word
    sm_config_set_clkdiv(&cfg, 1.0f);                            // 12 cycles/bit at 120 MHz
}

// Flush any sub-word residual left in the RX ISR into the FIFO. The SM shifts
// right and autopushes a full word, so a frame that ends mid-word (the line goes
// idle after the FCS with no further transitions) leaves its final octet(s) in the
// ISR below the threshold, which a plain SM reset would discard. Clocking zero bits
// in until the word autopushes shifts the residual down to the low bits, so it
// lands byte-aligned and contiguous with the frame in the capture buffer; the added
// high zero bits fall past the FCS and are ignored by the recovery. Exactly one
// autopush occurs: the residual is fewer than RX_FIFO_WORD_BITS bits, so the clocks
// after it cannot reach the threshold again (and an already word-aligned capture
// autopushes one all-zero word, likewise ignored).
inline void flush_rx_isr(PIO pio, std::uint32_t sm) {
    for (std::uint32_t i{0}; i < RX_FIFO_WORD_BITS; ++i) {
        pio_sm_exec_wait_blocking(pio, sm, pio_encode_in(pio_null, 1));
    }
}

} // namespace pico_ethernet

#endif // PHY_RX_PIO_CONFIG_H
