// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_RX_STIMULUS_H
#define PHY_TEST_RX_STIMULUS_H

#include <algorithm>
#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <span>

#include "hardware/dma.h"
#include "hardware/pio.h"

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"
#include "src/phy/test/tx_reference.h"
#include "src/phy/tx_level.h"

#include "rx_stimulus.pio.h"

namespace pico_ethernet {

// Ample for the RX SM to pad and push its last word once a waveform has ended.
inline constexpr std::uint32_t RX_SETTLE_US{5};

// Builds a broadcast frame of `length` octets (destination..FCS) whose last bit on
// the wire -- the top bit of its last FCS octet -- is `last_bit`.
inline const WireFrame& test_frame(std::size_t length, bool last_bit) {
    assert(length >= MIN_FRAME_WITH_FCS && length <= MAX_FRAME_WITH_FCS);

    static std::array<std::uint8_t, MAX_FRAME_NO_FCS> host{};
    static WireFrame wire{};
    std::fill_n(host.begin(), MacAddress::LENGTH, 0xFF);
    for (std::uint32_t tweak{0};; ++tweak) {
        host[MAC_HEADER_LEN] = static_cast<std::uint8_t>(tweak);
        (void)build_frame(std::span{host}.first(length - FCS_LEN), wire);
        if (((wire.bytes[wire.length - 1] >> 7) != 0) == last_bit) {
            return wire;
        }
    }
}

// A single-ended RXD waveform, one level per 50 ns half-bit, built front to back.
class RxWaveform {
  public:
    static constexpr std::size_t MAX_HALF_BITS{2048};

    // One data bit as two Manchester half-bits.
    RxWaveform& bit(bool one) { return level(one ? LEVEL_NEG : LEVEL_POS, 1).level(one ? LEVEL_POS : LEVEL_NEG, 1); }

    // Wire octets, LSB-first, from the reference encoder.
    RxWaveform& octets(std::span<const std::uint8_t> wire) {
        size_ += encode_reference(wire, std::span{half_bits_}.subspan(size_));
        return *this;
    }

    // Wire octets, then TP_IDL, from the reference encoder.
    RxWaveform& transmission(std::span<const std::uint8_t> wire) {
        size_ += encode_transmission(wire, std::span{half_bits_}.subspan(size_));
        return *this;
    }

    RxWaveform& level(std::uint8_t level, std::size_t half_bits) {
        assert(size_ + half_bits <= MAX_HALF_BITS);
        std::fill_n(half_bits_.begin() + static_cast<std::ptrdiff_t>(size_), half_bits, HalfBit{level});
        size_ += half_bits;
        return *this;
    }

    RxWaveform& clear() {
        size_ = 0;
        return *this;
    }

    [[nodiscard]] std::span<const HalfBit> half_bits() const { return std::span{half_bits_}.first(size_); }

  private:
    std::array<HalfBit, MAX_HALF_BITS> half_bits_{};
    std::size_t size_{0};
};

// Replays an RxWaveform onto an RXD pin from a PIO state machine, in place of the
// data slicer: high for a +V half-bit, low otherwise.
class RxStimulus {
  public:
    [[nodiscard]] bool configure(PIO pio, std::uint32_t rxd_pin) {
        pio_ = pio;
        if (!pio_can_add_program(pio_, &rx_stimulus_program)) {
            return false;
        }
        const std::uint32_t offset{static_cast<std::uint32_t>(pio_add_program(pio_, &rx_stimulus_program))};
        const std::int32_t sm{pio_claim_unused_sm(pio_, false)};
        dma_ = dma_claim_unused_channel(false);
        if (sm < 0 || dma_ < 0) {
            return false;
        }
        sm_ = static_cast<std::uint32_t>(sm);

        pio_gpio_init(pio_, rxd_pin);
        pio_sm_set_consecutive_pindirs(pio_, sm_, rxd_pin, 1, true);
        pio_sm_config cfg{rx_stimulus_program_get_default_config(offset)};
        sm_config_set_out_pins(&cfg, rxd_pin, 1);
        sm_config_set_set_pins(&cfg, rxd_pin, 1);
        sm_config_set_out_shift(&cfg, true, true, 32);
        sm_config_set_clkdiv(&cfg, 1.0f);
        pio_sm_init(pio_, sm_, offset, &cfg);
        pio_sm_exec(pio_, sm_, pio_encode_set(pio_pins, 0));
        pio_sm_set_enabled(pio_, sm_, true);
        return true;
    }

    // Returns once the whole waveform has been driven. RXD is left low: the state
    // machine stalls on a trailing all-low word.
    void play(const RxWaveform& waveform) {
        const std::span<const HalfBit> half_bits{waveform.half_bits()};
        const std::size_t word_count{half_bits.size() / 32 + 1};
        words_.fill(0);
        for (std::size_t i{0}; i < half_bits.size(); ++i) {
            if (half_bits[i].level == LEVEL_POS) {
                words_[i / 32] |= 1u << (i % 32);
            }
        }

        dma_channel_config c{dma_channel_get_default_config(dma_)};
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, true);
        channel_config_set_write_increment(&c, false);
        channel_config_set_dreq(&c, pio_get_dreq(pio_, sm_, true));
        dma_channel_configure(dma_, &c, &pio_->txf[sm_], words_.data(), word_count, true);
        dma_channel_wait_for_finish_blocking(dma_);
        // The trailing word is pulled only once every earlier one has been shifted out.
        while (!pio_sm_is_tx_fifo_empty(pio_, sm_)) {
            tight_loop_contents();
        }
    }

  private:
    PIO pio_{};
    std::uint32_t sm_{0};
    std::int32_t dma_{-1};
    std::array<std::uint32_t, RxWaveform::MAX_HALF_BITS / 32 + 1> words_{};
};

} // namespace pico_ethernet

#endif // PHY_TEST_RX_STIMULUS_H
