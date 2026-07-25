// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// On-target automated self-test for the receive PIO decoder. A stimulus SM
// replays a precomputed single-ended RXD waveform (derived from the reference
// Manchester encoder) into the RX decoder on-chip -- no wire, no scope -- while a
// GPIO holds RXC. The decoder's autopushed octets are collected by DMA and
// compared against the source wire frame; the recovered frame is then run through
// the MAC parser to confirm preamble lock, octet alignment and FCS. A corrupted
// run confirms a damaged frame is rejected by the FCS rather than silently
// accepted. Results are reported over the UART (debugprobe console).

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <expected>
#include <span>

#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"

#include "src/mac/ethernet_frame.h"
#include "src/mac/frame_builder.h"
#include "src/mac/frame_filter.h"
#include "src/mac/mac_address.h"
#include "src/mac/test/frame_parser.h"
#include "src/phy/phy_timing.h"
#include "src/phy/rx_frame_recover.h"
#include "src/phy/rx_pio_config.h"
#include "src/phy/test/tx_reference.h"
#include "src/phy/tx_emphasis.h"
#include "src/util/led_heartbeat.h"

#include "rx.pio.h"
#include "rx_stimulus.pio.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t RXD_PIN{8}; // stimulus output and RX SM IN base
constexpr std::uint32_t RXC_PIN{9}; // carrier, driven high via SIO (RX IN base + 1)
constexpr std::uint32_t LED_PIN{25};

constexpr std::size_t MAX_WIRE{128};
constexpr std::size_t MAX_HALFBITS{MAX_WIRE * 8 * 2};
constexpr std::size_t MAX_WORDS{(MAX_HALFBITS + 31) / 32};

std::uint8_t recovered_octet(std::span<const std::uint32_t> words, std::size_t octet) {
    // Shift-right autopush packs RX_OCTETS_PER_WORD octets into each word LSB-first,
    // so octet j is byte j%RX_OCTETS_PER_WORD of the little-endian word.
    return static_cast<std::uint8_t>(words[octet / RX_OCTETS_PER_WORD] >> (8 * (octet % RX_OCTETS_PER_WORD)));
}

// The corrupted run flattens one bit's mid-bit transition to forge a decode
// fault; the mismatch it produces must land at or after the octet that carries
// that bit -- an earlier mismatch would be a decode or startup glitch, not the
// injected fault.
constexpr std::size_t CORRUPT_HALFBIT{200};                   // leading half of the flattened bit
constexpr std::size_t CORRUPT_OCTET{CORRUPT_HALFBIT / 2 / 8}; // first octet the fault can reach

// Writes one data bit as two Manchester half-bits (bit 1 = -V then +V; bit 0 =
// +V then -V, matching the reference encoder) and returns the next half-bit index.
std::size_t put_bit(std::span<HalfBit> halfbits, std::size_t h, bool one) {
    halfbits[h] = {one ? LEVEL_NEG : LEVEL_POS, true};
    halfbits[h + 1] = {one ? LEVEL_POS : LEVEL_NEG, true};
    return h + 2;
}

// Packs a source wire frame into the stimulus half-bit stream (RXD level per
// half-bit, LSB-first, 32 half-bits per 32-bit word). `lead_bits` prepends that
// many preamble bits so the frame's octet boundaries shift, emulating a carrier
// gate that starts the decoder mid-preamble; `trail_bits` appends preamble bits so
// the final data octet completes (on the wire the line keeps toggling past the FCS
// until carrier drops). Optionally drops one mid-bit transition to forge a decode
// fault. Returns the number of stimulus words.
std::size_t pack_stimulus(
    std::span<const std::uint8_t> wire,
    std::span<std::uint32_t> words,
    bool corrupt,
    std::size_t lead_bits = 0,
    std::size_t trail_bits = 0) {
    std::array<HalfBit, MAX_HALFBITS> halfbits{};

    std::size_t h{0};
    // Alternating and ending in 0, so the lead flows into the frame's first
    // preamble bit (1) without forging a false SFD "1,1".
    for (std::size_t b{0}; b < lead_bits; ++b) {
        h = put_bit(halfbits, h, ((lead_bits - 1 - b) % 2) == 1);
    }

    const std::size_t frame_start{h};
    h += encode_reference(wire, std::span(halfbits).subspan(h));

    if (corrupt && h > frame_start + CORRUPT_HALFBIT + 1) {
        // Force a half-bit to equal its predecessor so the guaranteed Manchester
        // mid-bit edge is missing there.
        halfbits[frame_start + CORRUPT_HALFBIT + 1].level = halfbits[frame_start + CORRUPT_HALFBIT].level;
    }

    for (std::size_t b{0}; b < trail_bits; ++b) {
        h = put_bit(halfbits, h, true);
    }

    const std::size_t nhalf{h};
    for (std::uint32_t& w : words) {
        w = 0;
    }
    for (std::size_t k{0}; k < nhalf; ++k) {
        if (halfbits[k].level == LEVEL_POS) {
            words[k / 32] |= 1u << (k % 32);
        }
    }
    return (nhalf + 31) / 32;
}

struct RxSelfTest {
    PIO pio{pio0};
    std::uint32_t sm_stim{0};
    std::uint32_t sm_rx{0};
    std::uint32_t off_stim{0};
    std::uint32_t off_rx{0};
    int dma_stim{-1};
    int dma_rx{-1};
    std::array<std::uint32_t, MAX_WORDS> stim_words{};
    std::array<std::uint32_t, MAX_WORDS + 16> rx_words{};
    pio_sm_config stim_cfg{};
    pio_sm_config rx_cfg{};

    void configure() {
        off_stim = static_cast<std::uint32_t>(pio_add_program(pio, &rx_stimulus_program));
        off_rx = static_cast<std::uint32_t>(pio_add_program(pio, &rx_manchester_program));
        sm_stim = static_cast<std::uint32_t>(pio_claim_unused_sm(pio, true));
        sm_rx = static_cast<std::uint32_t>(pio_claim_unused_sm(pio, true));
        dma_stim = dma_claim_unused_channel(true);
        dma_rx = dma_claim_unused_channel(true);

        // RXC carrier: SIO output held high for the whole injection.
        gpio_init(RXC_PIN);
        gpio_set_dir(RXC_PIN, GPIO_OUT);
        gpio_put(RXC_PIN, true);
        gpio_set_input_enabled(RXC_PIN, true);

        // Stimulus drives RXD; the RX SM reads the same pad, so only the stimulus
        // sets the pin direction (output).
        pio_gpio_init(pio, RXD_PIN);
        pio_sm_set_consecutive_pindirs(pio, sm_stim, RXD_PIN, 1, true);

        stim_cfg = rx_stimulus_program_get_default_config(off_stim);
        sm_config_set_out_pins(&stim_cfg, RXD_PIN, 1);
        sm_config_set_set_pins(&stim_cfg, RXD_PIN, 1); // to force an idle-low pad between runs
        sm_config_set_out_shift(&stim_cfg, true, true, 32);
        sm_config_set_clkdiv(&stim_cfg, 1.0f);
        pio_sm_init(pio, sm_stim, off_stim, &stim_cfg);

        rx_cfg = rx_manchester_program_get_default_config(off_rx);
        configure_rx_shift(rx_cfg, RXD_PIN);
        pio_sm_init(pio, sm_rx, off_rx, &rx_cfg);
    }

    // Injects the packed stimulus, collects decoded octets, and returns the count.
    std::size_t run(std::size_t nwords) {
        pio_sm_set_enabled(pio, sm_stim, false);
        pio_sm_set_enabled(pio, sm_rx, false);
        pio_sm_clear_fifos(pio, sm_stim);
        pio_sm_clear_fifos(pio, sm_rx);
        pio_sm_init(pio, sm_stim, off_stim, &stim_cfg);
        pio_sm_init(pio, sm_rx, off_rx, &rx_cfg);

        // Force the pad idle-low before injection so the first sampled bit does not
        // depend on the level a previous run left driven.
        pio_sm_exec(pio, sm_stim, pio_encode_set(pio_pins, 0));

        for (std::uint32_t& w : rx_words) {
            w = 0;
        }

        dma_channel_config cr{dma_channel_get_default_config(dma_rx)};
        channel_config_set_transfer_data_size(&cr, DMA_SIZE_32);
        channel_config_set_read_increment(&cr, false);
        channel_config_set_write_increment(&cr, true);
        channel_config_set_dreq(&cr, pio_get_dreq(pio, sm_rx, false));
        dma_channel_configure(dma_rx, &cr, rx_words.data(), &pio->rxf[sm_rx], rx_words.size(), true);

        dma_channel_config ct{dma_channel_get_default_config(dma_stim)};
        channel_config_set_transfer_data_size(&ct, DMA_SIZE_32);
        channel_config_set_read_increment(&ct, true);
        channel_config_set_write_increment(&ct, false);
        channel_config_set_dreq(&ct, pio_get_dreq(pio, sm_stim, true));
        dma_channel_configure(dma_stim, &ct, &pio->txf[sm_stim], stim_words.data(), nwords, true);

        // FIFOs are primed by the running DMAs; enable both SMs on the same cycle
        // so the decoder samples the first half-bit in phase.
        pio_enable_sm_mask_in_sync(pio, (1u << sm_stim) | (1u << sm_rx));

        dma_channel_wait_for_finish_blocking(dma_stim);
        busy_wait_us(50); // let the last FIFO-buffered half-bits clock through

        dma_channel_abort(dma_rx);
        const std::uint32_t remaining{dma_channel_hw_addr(dma_rx)->transfer_count};
        return rx_words.size() - remaining;
    }
};

// A minimal broadcast frame padded to the 60-byte minimum by the builder.
constexpr std::array<std::uint8_t, MAC_HEADER_LEN + 8> HOST_FRAME{
    0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, // destination (broadcast)
    0x02, 0x00, 0x00, 0x00, 0x00, 0x01, // source
    0x08, 0x00,                         // EtherType (IPv4)
    0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x23, 0x45, 0x67};

FrameFilter promiscuous_filter() {
    constexpr std::array<std::uint8_t, MacAddress::LENGTH> ANY{0x02, 0, 0, 0, 0, 0x01};
    FrameFilter filter{MacAddress(std::span<const std::uint8_t, MacAddress::LENGTH>(ANY))};
    filter.set_packet_filter(FrameFilter::PROMISCUOUS);
    return filter;
}

// Injects the stimulus and returns the index of the first decoded octet that
// differs from the source wire frame, or wire.size() if every octet matches. The
// first discrepancy is always printed so it is available for diagnosis in both the
// clean (unexpected) and corrupted (expected) runs.
std::size_t first_mismatch(RxSelfTest& test, std::span<const std::uint8_t> wire, bool corrupt) {
    const std::size_t nwords{pack_stimulus(wire, test.stim_words, corrupt)};
    // No trailing bits: the source wire frame here is a whole number of words (its
    // length is a multiple of RX_OCTETS_PER_WORD), so the final word autopushes and
    // every octet is recovered.
    const std::size_t got{test.run(nwords) * RX_OCTETS_PER_WORD};

    const std::size_t comparable{got < wire.size() ? got : wire.size()};
    for (std::size_t i{0}; i < comparable; ++i) {
        const std::uint8_t b{recovered_octet(test.rx_words, i)};
        if (b != wire[i]) {
            printf("  octet %u decoded 0x%02X, expected 0x%02X\n", static_cast<unsigned>(i), b, wire[i]);
            return i;
        }
    }
    if (got < wire.size()) {
        printf("  decoded %u octets, expected %u\n", static_cast<unsigned>(got), static_cast<unsigned>(wire.size()));
        return got;
    }
    return wire.size();
}

} // namespace

int run_selftest() {
    set_sys_clock_khz(SYS_CLOCK_HZ / 1000, true);
    stdio_init_all();
    sleep_ms(2500); // let the debugprobe UART console attach
    printf("\nrx_pio_selftest: start\n");

    const auto built = build_frame(HOST_FRAME);
    bool ok{built.has_value()};
    if (!ok) {
        printf("FAIL: could not build the source wire frame\n");
    }

    RxSelfTest test{};
    test.configure();

    if (ok) {
        const std::span<const std::uint8_t> wire{built->view()};

        // Clean decode: the recovered octets must equal the source wire frame.
        if (first_mismatch(test, wire, false) != wire.size()) {
            printf("FAIL: clean decode mismatch\n");
            ok = false;
        }

        // The recovered frame must pass the MAC parser (preamble lock, alignment, FCS).
        std::array<std::uint8_t, WIRE_CAPACITY> recovered{};
        for (std::size_t i{0}; i < wire.size(); ++i) {
            recovered[i] = recovered_octet(test.rx_words, i);
        }
        const FrameFilter filter{promiscuous_filter()};
        const auto parsed = parse_frame(std::span(recovered).first(wire.size()), filter);
        if (!parsed) {
            printf("FAIL: parser rejected the cleanly decoded frame\n");
            ok = false;
        } else if (parsed->size() != MIN_FRAME_NO_FCS) {
            printf("FAIL: parsed host frame length %u unexpected\n", static_cast<unsigned>(parsed->size()));
            ok = false;
        }

        // Corrupted decode: a dropped mid-bit edge must surface as a mismatch at or
        // after the injected octet (the FCS then rejects it). A mismatch before that
        // point would signal a decode or startup glitch rather than the injected fault.
        const std::size_t corrupt_at{first_mismatch(test, wire, true)};
        if (corrupt_at == wire.size()) {
            printf("FAIL: corrupted frame decoded identical to the source\n");
            ok = false;
        } else if (corrupt_at < CORRUPT_OCTET) {
            printf(
                "FAIL: mismatch at octet %u precedes the injected fault (octet %u)\n",
                static_cast<unsigned>(corrupt_at),
                static_cast<unsigned>(CORRUPT_OCTET));
            ok = false;
        }

        // Octet-alignment and FCS delimiting across all 8 carrier-gate bit offsets.
        // On the wire the decoder starts mid-preamble, so its octet boundaries are
        // bit-offset from the frame; recover_frame must reassemble the exact
        // destination..FCS bytes at every offset. The 40 trailing bits guarantee the
        // word carrying the final FCS octet autopushes regardless of offset.
        std::array<std::uint8_t, WIRE_CAPACITY> aligned{};
        const std::span<const std::uint8_t> body{wire.subspan(PREAMBLE_SFD_LEN)};
        for (std::size_t offset{0}; offset < 8; ++offset) {
            const std::size_t nwords{pack_stimulus(wire, test.stim_words, false, offset, 40)};
            const std::size_t gotwords{test.run(nwords)};
            const std::span<const std::uint8_t> raw{
                reinterpret_cast<const std::uint8_t*>(test.rx_words.data()), gotwords * RX_OCTETS_PER_WORD};
            const std::expected<std::size_t, FrameError> len{recover_frame(raw, aligned)};
            if (!len || *len != body.size()) {
                printf("FAIL: offset %u: recover_frame did not recover the frame\n", static_cast<unsigned>(offset));
                ok = false;
                continue;
            }
            bool match{true};
            for (std::size_t i{0}; i < body.size(); ++i) {
                if (aligned[i] != body[i]) {
                    match = false;
                    break;
                }
            }
            if (!match) {
                printf("FAIL: offset %u: recovered frame bytes mismatch\n", static_cast<unsigned>(offset));
                ok = false;
            }
        }
    }

    printf("rx_pio_selftest: %s\n", ok ? "PASS" : "FAIL");

    LedHeartbeat led{LED_PIN};
    if (ok) {
        led.good();
    } else {
        led.bad();
    }
    while (true) {
        sleep_ms(10'000);
    }
    return ok ? 0 : 1;
}

} // namespace pico_ethernet

int main() { return pico_ethernet::run_selftest(); }
