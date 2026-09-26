// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef PHY_AUTONEG_H
#define PHY_AUTONEG_H

#include <cstdint>
#include <optional>
#include <utility>

#include "src/phy/duplex.h"
#include "src/phy/phy_timing.h"

namespace pico_ethernet {

// Base-page link code word fields (IEEE 802.3 Clause 28.2.1.2), D0 in bit 0.
inline constexpr std::uint16_t LCW_SELECTOR_MASK{0x001F};
inline constexpr std::uint16_t LCW_SELECTOR_IEEE_802_3{0x0001};
inline constexpr std::uint16_t LCW_10BASE_T{1u << 5};
inline constexpr std::uint16_t LCW_10BASE_T_FULL_DUPLEX{1u << 6};
inline constexpr std::uint16_t LCW_ACKNOWLEDGE{1u << 14};

// No remote fault and no next page are ever advertised.
inline constexpr std::uint16_t ADVERTISED_LCW{LCW_SELECTOR_IEEE_802_3 | LCW_10BASE_T | LCW_10BASE_T_FULL_DUPLEX};

// Base-page autonegotiation arbitration (IEEE 802.3 Clause 28, without next page):
// exchanges link code words over FLP bursts, resolves the highest common ability,
// and falls back to half duplex by parallel detection when the peer only sends
// NLPs. The 10BASE-T link integrity verdict is an input, and once the link is
// good it alone keeps it up; its failure restarts negotiation through break_link.
//
// Pure: received code words, sent bursts and the passage of time are reported by the
// caller. Timestamps are microseconds from a free-running 32-bit counter, compared as
// unsigned differences so a counter wrap is transparent.
class Autoneg {
  public:
    enum class LinkPulses : std::uint8_t { None, Flp, Nlp };

    // Starts in break_link, so a peer still linked to an earlier session sees the
    // link fail before negotiation begins.
    explicit constexpr Autoneg(std::uint32_t now_us)
        : entered_us_{now_us} {}

    constexpr void on_code_word(std::uint16_t code_word, bool consecutive, std::uint32_t now_us) {
        last_code_word_us_ = now_us;
        switch (state_) {
            case State::AbilityDetect:
                // The peer may already be acknowledging us, which says nothing about
                // its abilities.
                if (count_match(without_acknowledge(code_word), consecutive) == MATCH_COUNT) {
                    partner_ = without_acknowledge(code_word);
                    enter(State::AcknowledgeDetect, now_us);
                }
                return;
            case State::AcknowledgeDetect:
                if ((code_word & LCW_ACKNOWLEDGE) == 0) {
                    match_count_ = 0;
                    return;
                }
                if (count_match(code_word, consecutive) == MATCH_COUNT) {
                    const bool consistent{without_acknowledge(code_word) == partner_};
                    enter(consistent ? State::CompleteAcknowledge : State::TransmitDisable, now_us);
                }
                return;
            case State::LinkStatusCheck:
                // The peer negotiates after all, so the parallel detection is void.
                enter(State::TransmitDisable, now_us);
                return;
            case State::TransmitDisable:
            case State::CompleteAcknowledge:
            case State::FlpLinkGoodCheck:
            case State::LinkGood:
                return;
        }
    }

    constexpr void on_burst_sent(std::uint32_t now_us) {
        if (state_ != State::CompleteAcknowledge) {
            return;
        }
        ++bursts_;
        if (bursts_ == COMPLETE_ACK_BURSTS) {
            resolved_ = resolve(partner_);
            enter(State::FlpLinkGoodCheck, now_us);
        }
    }

    // `link_integrity` is the 10BASE-T link integrity verdict: enough valid NLPs or
    // received frames, recently enough.
    constexpr void advance(std::uint32_t now_us, bool link_integrity) {
        const std::uint32_t elapsed_us{now_us - entered_us_};
        switch (state_) {
            case State::TransmitDisable:
                if (elapsed_us >= BREAK_LINK_US) {
                    enter(State::AbilityDetect, now_us);
                }
                return;
            case State::AbilityDetect:
                if (link_integrity) {
                    enter(State::LinkStatusCheck, now_us);
                }
                return;
            case State::FlpLinkGoodCheck:
                if (link_integrity && resolved_) {
                    enter(State::LinkGood, now_us);
                } else if (elapsed_us >= LINK_FAIL_INHIBIT_US) {
                    enter(State::TransmitDisable, now_us);
                }
                return;
            case State::LinkStatusCheck:
                if (!link_integrity) {
                    enter(State::TransmitDisable, now_us);
                } else if (elapsed_us >= AUTONEG_WAIT_US) {
                    resolved_ = Duplex::Half;
                    enter(State::LinkGood, now_us);
                }
                return;
            case State::LinkGood:
                if (!link_integrity) {
                    enter(State::TransmitDisable, now_us);
                }
                return;
            case State::AcknowledgeDetect:
                // flp_receive_idle: the peer stopped sending FLP bursts.
                if (now_us - last_code_word_us_ >= NLP_TEST_MAX_US) {
                    enter(State::TransmitDisable, now_us);
                }
                return;
            case State::CompleteAcknowledge:
                // Ends on our own bursts, whether or not the peer still sends any.
                return;
        }
    }

    // The resolved duplex while the link is up, nullopt while it is down.
    [[nodiscard]] constexpr std::optional<Duplex> link() const {
        return state_ == State::LinkGood ? resolved_ : std::nullopt;
    }

    [[nodiscard]] constexpr LinkPulses link_pulses() const {
        switch (state_) {
            case State::TransmitDisable:
                return LinkPulses::None;
            case State::AbilityDetect:
            case State::AcknowledgeDetect:
            case State::CompleteAcknowledge:
                return LinkPulses::Flp;
            case State::FlpLinkGoodCheck:
            case State::LinkStatusCheck:
            case State::LinkGood:
                return LinkPulses::Nlp;
        }
        std::unreachable();
    }

    // The code word the FLP bursts carry.
    [[nodiscard]] constexpr std::uint16_t code_word() const {
        const bool acknowledging{state_ == State::AcknowledgeDetect || state_ == State::CompleteAcknowledge};
        return acknowledging ? static_cast<std::uint16_t>(ADVERTISED_LCW | LCW_ACKNOWLEDGE) : ADVERTISED_LCW;
    }

  private:
    enum class State : std::uint8_t {
        TransmitDisable,
        AbilityDetect,
        AcknowledgeDetect,
        CompleteAcknowledge,
        FlpLinkGoodCheck,
        LinkStatusCheck,
        LinkGood,
    };

    // Identical consecutive code words that make an ability or acknowledge match.
    static constexpr std::uint32_t MATCH_COUNT{3};

    [[nodiscard]] static constexpr std::uint16_t without_acknowledge(std::uint16_t code_word) {
        return static_cast<std::uint16_t>(code_word & ~LCW_ACKNOWLEDGE);
    }

    // Priority resolution: 10BASE-T full duplex over half duplex. The ability bits
    // only mean the same to both ends under the same selector.
    [[nodiscard]] static constexpr std::optional<Duplex> resolve(std::uint16_t partner) {
        if ((partner & LCW_SELECTOR_MASK) != (ADVERTISED_LCW & LCW_SELECTOR_MASK)) {
            return std::nullopt;
        }
        const std::uint16_t common{static_cast<std::uint16_t>(partner & ADVERTISED_LCW)};
        if ((common & LCW_10BASE_T_FULL_DUPLEX) != 0) {
            return Duplex::Full;
        }
        if ((common & LCW_10BASE_T) != 0) {
            return Duplex::Half;
        }
        return std::nullopt;
    }

    // Length of the run of identical consecutive code words `code_word` extends.
    constexpr std::uint32_t count_match(std::uint16_t code_word, bool consecutive) {
        match_count_ = consecutive && match_count_ > 0 && code_word == match_word_ ? match_count_ + 1 : 1;
        match_word_ = code_word;
        return match_count_;
    }

    constexpr void enter(State state, std::uint32_t now_us) {
        state_ = state;
        entered_us_ = now_us;
        match_count_ = 0;
        bursts_ = 0;
    }

    State state_{State::TransmitDisable};
    std::uint32_t entered_us_;
    std::uint32_t last_code_word_us_{0};
    std::uint16_t match_word_{0};
    std::uint32_t match_count_{0};
    std::uint16_t partner_{0};
    std::uint32_t bursts_{0};
    std::optional<Duplex> resolved_{};
};

} // namespace pico_ethernet

#endif // PHY_AUTONEG_H
