// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_TEST_PIN_CAPTURE_H
#define PHY_TEST_PIN_CAPTURE_H

#include <array>
#include <cstddef>
#include <cstdint>

#include "hardware/dma.h"
#include "hardware/pio.h"

#include "pin_capture.pio.h"

namespace pico_ethernet {

// Records what a pin pair is driven to, at full PIO clock rate, into a DMA buffer.
//
// It replaces CPU polling of PIO_DBG_PADOUT, which can only follow a state machine
// slowed by a large clock divider. Here the observed machine runs at its real /1
// clock, so the capture measures true symbol timing rather than relative cycle
// structure, and the transmit driver can stay connected because no symbol is
// stretched towards DC.
template <std::size_t WORDS>
class PinCapture {
  public:
    static constexpr std::size_t SAMPLES_PER_WORD{16};
    static constexpr std::size_t SAMPLE_COUNT{WORDS * SAMPLES_PER_WORD};

    [[nodiscard]] bool configure(PIO pio, std::uint32_t base_pin) {
        pio_ = pio;
        if (!pio_can_add_program(pio_, &pin_capture_program)) {
            return false;
        }
        const std::uint32_t offset{static_cast<std::uint32_t>(pio_add_program(pio_, &pin_capture_program))};
        const int sm{pio_claim_unused_sm(pio_, false)};
        dma_ = dma_claim_unused_channel(false);
        if (sm < 0 || dma_ < 0) {
            return false;
        }
        sm_ = static_cast<std::uint32_t>(sm);

        // The pins are driven by the machine under observation; only their inputs
        // are read here, so no pin direction or function select is claimed.
        pio_sm_config cfg{pin_capture_program_get_default_config(offset)};
        sm_config_set_in_pins(&cfg, base_pin);
        sm_config_set_in_shift(&cfg, true, true, 32);
        sm_config_set_clkdiv(&cfg, 1.0f);
        pio_sm_init(pio_, sm_, offset, &cfg);
        return true;
    }

    // Readies the capture. The state machine is left disabled so the caller can
    // start it in lockstep with the one it is observing.
    void arm() {
        pio_sm_set_enabled(pio_, sm_, false);
        pio_sm_clear_fifos(pio_, sm_);
        words_.fill(0);

        dma_channel_config c{dma_channel_get_default_config(dma_)};
        channel_config_set_transfer_data_size(&c, DMA_SIZE_32);
        channel_config_set_read_increment(&c, false);
        channel_config_set_write_increment(&c, true);
        channel_config_set_dreq(&c, pio_get_dreq(pio_, sm_, false));
        dma_channel_configure(dma_, &c, words_.data(), &pio_->rxf[sm_], WORDS, true);
    }

    // Starts sampling on its own, for an observed machine whose start the test does
    // not control. Run lengths are unaffected; only the phase reference is lost.
    void start() { pio_sm_set_enabled(pio_, sm_, true); }

    void wait() {
        dma_channel_wait_for_finish_blocking(dma_);
        pio_sm_set_enabled(pio_, sm_, false);
    }

    // Shift-right autopush packs the samples least-significant first, so sample i
    // occupies bits [2i, 2i+1] of its word.
    [[nodiscard]] std::uint8_t level(std::size_t sample) const {
        return static_cast<std::uint8_t>(words_[sample / SAMPLES_PER_WORD] >> (2 * (sample % SAMPLES_PER_WORD))) & 0b11;
    }

    [[nodiscard]] std::uint32_t sm_mask() const { return 1u << sm_; }

  private:
    PIO pio_{};
    std::uint32_t sm_{0};
    int dma_{-1};
    std::array<std::uint32_t, WORDS> words_{};
};

} // namespace pico_ethernet

#endif // PHY_TEST_PIN_CAPTURE_H
