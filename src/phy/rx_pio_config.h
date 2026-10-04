// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_RX_PIO_CONFIG_H
#define PHY_RX_PIO_CONFIG_H

#include <cassert>
#include <cstdint>
#include <span>

#include "hardware/dma.h"
#include "hardware/pio.h"

#include "src/mac/crc32.h"
#include "src/mac/ethernet_frame.h"
#include "src/phy/phy_timing.h"
#include "src/phy/rx_frame_end.h"

#include "rx.pio.h"

namespace pico_ethernet {

// RXD is the IN base and the JMP pin; RXC (carrier gate) is IN base + 1.
[[nodiscard]] inline pio_sm_config rx_manchester_config(std::uint32_t offset, std::uint32_t rxd_pin) {
    pio_sm_config cfg{rx_manchester_program_get_default_config(offset)};
    sm_config_set_in_pins(&cfg, rxd_pin);
    sm_config_set_jmp_pin(&cfg, rxd_pin);
    sm_config_set_in_shift(&cfg, true, true, RX_OCTETS_PER_WORD * 8); // shift right, autopush a full word
    sm_config_set_mov_status(&cfg, STATUS_IRQ_SET, TP_IDL_FLAG);
    sm_config_set_clkdiv(&cfg, 1.0f); // 12 cycles/bit at 120 MHz
    return cfg;
}

// Restarts rx_manchester at its carrier gate with an empty ISR, ready to hunt the
// next frame's SFD.
inline void restart_rx(PIO pio, std::uint32_t sm, std::uint32_t offset, const pio_sm_config& cfg) {
    // The SFD hunt compares its samples of the bits' first halves, which are the
    // complements of the bits, with Y.
    constexpr std::uint32_t SFD_SAMPLES{static_cast<std::uint8_t>(~SFD_BYTE)};

    pio_sm_set_enabled(pio, sm, false);
    pio_interrupt_clear(pio, RX_DONE_FLAG);
    pio_sm_init(pio, sm, offset, &cfg);
    pio_sm_put(pio, sm, SFD_SAMPLES);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_exec(pio, sm, pio_encode_mov(pio_y, pio_osr));
}

inline void start_tp_idl_watchdog(PIO pio, std::uint32_t sm, std::uint32_t offset, std::uint32_t rxd_pin) {
    // The watchdog's loop in rx.pio samples RXD once per iteration and runs its
    // loaded count plus one times.
    constexpr std::uint32_t CYCLES_PER_ITERATION{2};
    constexpr std::uint32_t LOOP_COUNT{TP_IDL_DETECT_NS * SYS_CYCLES_PER_US / 1000 / CYCLES_PER_ITERATION - 1};

    pio_sm_config cfg{tp_idl_watchdog_program_get_default_config(offset)};
    sm_config_set_in_pins(&cfg, rxd_pin);
    sm_config_set_jmp_pin(&cfg, rxd_pin);
    sm_config_set_clkdiv(&cfg, 1.0f);
    pio_sm_init(pio, sm, offset + tp_idl_watchdog_offset_start, &cfg);

    // The window stays in the OSR for the program to reload X from on every high run.
    pio_sm_put(pio, sm, LOOP_COUNT);
    pio_sm_exec(pio, sm, pio_encode_pull(false, false));
    pio_sm_set_enabled(pio, sm, true);
}

// Assigns the chip's single DMA CRC sniffer to `channel`. CRC32R with a bit-reversed
// result reads back as the reflected running CRC of crc32_update(), not inverted.
inline void enable_rx_sniffer(std::uint32_t channel) {
    dma_sniffer_enable(channel, DMA_SNIFF_CTRL_CALC_VALUE_CRC32R, false);
    dma_sniffer_set_output_reverse_enabled(true);
}

// Starts `channel` landing the RX SM's words into `buffer`, with the sniffer seeded
// to compute their CRC.
inline void start_rx_capture(std::uint32_t channel, PIO pio, std::uint32_t sm, std::span<std::uint8_t> buffer) {
    assert(buffer.size() % RX_OCTETS_PER_WORD == 0);

    dma_channel_config c{dma_channel_get_default_config(channel)};
    channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, pio_get_dreq(pio, sm, false));
    channel_config_set_sniff_enable(&c, true);
    dma_sniffer_set_data_accumulator(CRC32_INIT);
    dma_channel_configure(channel, &c, buffer.data(), &pio->rxf[sm], buffer.size() / RX_OCTETS_PER_WORD, true);
}

} // namespace pico_ethernet

#endif // PHY_RX_PIO_CONFIG_H
