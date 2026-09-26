// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include "src/phy/autoneg.h"

#include <gtest/gtest.h>

#include <cstdint>
#include <optional>

#include "src/phy/duplex.h"
#include "src/phy/phy_timing.h"
#include "src/phy/test/peer_code_words.h"

namespace pico_ethernet {
namespace {

constexpr std::uint32_t START_US{1'000'000};

constexpr std::uint16_t LCW_100BASE_TX{1u << 7};
constexpr std::uint16_t LCW_REMOTE_FAULT{1u << 13};
constexpr std::uint16_t LCW_NEXT_PAGE{1u << 15};

constexpr std::uint16_t PEER_100BASE_TX_ONLY{LCW_SELECTOR_IEEE_802_3 | LCW_100BASE_TX};
constexpr std::uint16_t PEER_WITH_NEXT_PAGE_AND_REMOTE_FAULT{PEER_HALF_AND_FULL | LCW_NEXT_PAGE | LCW_REMOTE_FAULT};

constexpr std::uint16_t acknowledged(std::uint16_t code_word) {
    return static_cast<std::uint16_t>(code_word | LCW_ACKNOWLEDGE);
}

// Drives an Autoneg through time, with code words arriving a burst period apart.
class Negotiation {
  public:
    void wait(std::uint32_t duration_us, bool link_integrity = false) {
        now_us_ += duration_us;
        autoneg_.advance(now_us_, link_integrity);
    }

    void receive(std::uint16_t code_word, bool consecutive = true) {
        now_us_ += FLP_BURST_PERIOD_US;
        autoneg_.on_code_word(code_word, consecutive, now_us_);
        autoneg_.advance(now_us_, false);
    }

    void receive_times(std::uint16_t code_word, std::uint32_t count) {
        for (std::uint32_t i{0}; i < count; ++i) {
            receive(code_word);
        }
    }

    void send_bursts(std::uint32_t count) {
        for (std::uint32_t i{0}; i < count; ++i) {
            now_us_ += FLP_BURST_PERIOD_US;
            autoneg_.on_burst_sent(now_us_);
            autoneg_.advance(now_us_, false);
        }
    }

    void finish_break_link() { wait(BREAK_LINK_US); }

    // From break_link through ability and acknowledge detection and the complete
    // acknowledge bursts, against a peer advertising `peer`.
    void exchange(std::uint16_t peer) {
        finish_break_link();
        receive_times(peer, 3);
        receive_times(acknowledged(peer), 3);
        send_bursts(COMPLETE_ACK_BURSTS);
    }

    [[nodiscard]] bool acknowledging() const {
        return autoneg_.link_pulses() == Autoneg::LinkPulses::Flp &&
               autoneg_.code_word() == acknowledged(ADVERTISED_LCW);
    }

    [[nodiscard]] const Autoneg& autoneg() const { return autoneg_; }

  private:
    std::uint32_t now_us_{START_US};
    Autoneg autoneg_{START_US};
};

TEST(Autoneg, StaysSilentForBreakLinkThenSendsFlpBursts) {
    Negotiation n{};
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
    n.wait(BREAK_LINK_US - 1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);

    n.wait(1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Flp);
    EXPECT_EQ(n.autoneg().code_word(), ADVERTISED_LCW);
    EXPECT_FALSE(n.autoneg().link().has_value());
}

TEST(Autoneg, FullDuplexPeerResolvesToFullDuplex) {
    Negotiation n{};
    n.exchange(PEER_HALF_AND_FULL);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Nlp);
    EXPECT_FALSE(n.autoneg().link().has_value());

    n.wait(0, true);
    EXPECT_EQ(n.autoneg().link(), std::optional{Duplex::Full});
}

TEST(Autoneg, HalfDuplexOnlyPeerResolvesToHalfDuplex) {
    Negotiation n{};
    n.exchange(PEER_HALF_ONLY);
    n.wait(0, true);
    EXPECT_EQ(n.autoneg().link(), std::optional{Duplex::Half});
}

TEST(Autoneg, NoCommonAbilityBringsNoLinkAndRenegotiates) {
    Negotiation n{};
    n.exchange(PEER_100BASE_TX_ONLY);
    n.wait(0, true);
    EXPECT_FALSE(n.autoneg().link().has_value());

    n.wait(LINK_FAIL_INHIBIT_US, true);
    EXPECT_FALSE(n.autoneg().link().has_value());
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, AbilityMatchNeedsThreeConsecutiveIdenticalCodeWords) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 2);
    EXPECT_FALSE(n.acknowledging());

    n.receive(PEER_HALF_AND_FULL, false);
    n.receive(PEER_HALF_AND_FULL);
    EXPECT_FALSE(n.acknowledging());
    n.receive(PEER_HALF_ONLY);
    n.receive_times(PEER_HALF_AND_FULL, 2);
    EXPECT_FALSE(n.acknowledging());

    n.receive(PEER_HALF_AND_FULL);
    EXPECT_TRUE(n.acknowledging());
}

TEST(Autoneg, AbilityMatchIgnoresTheAcknowledgeBit) {
    Negotiation n{};
    n.finish_break_link();
    n.receive(PEER_HALF_AND_FULL);
    n.receive(acknowledged(PEER_HALF_AND_FULL));
    n.receive(PEER_HALF_AND_FULL);
    EXPECT_TRUE(n.acknowledging());
}

TEST(Autoneg, AcknowledgeMatchNeedsThreeConsecutiveAcknowledgedCodeWords) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 3);
    n.receive_times(acknowledged(PEER_HALF_AND_FULL), 2);
    n.receive(PEER_HALF_AND_FULL);
    n.receive_times(acknowledged(PEER_HALF_AND_FULL), 2);
    n.receive(acknowledged(PEER_HALF_AND_FULL), false);
    n.receive(acknowledged(PEER_HALF_AND_FULL));
    n.send_bursts(COMPLETE_ACK_BURSTS);
    EXPECT_TRUE(n.acknowledging());

    n.receive(acknowledged(PEER_HALF_AND_FULL));
    n.send_bursts(COMPLETE_ACK_BURSTS);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Nlp);
}

TEST(Autoneg, AcknowledgeDetectWaitsForAPeerThatKeepsSendingBursts) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 3 + NLP_TEST_MAX_US / FLP_BURST_PERIOD_US);
    EXPECT_TRUE(n.acknowledging());
}

TEST(Autoneg, PeerBurstsStoppingDuringAcknowledgeDetectRestarts) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 3);
    n.wait(NLP_TEST_MAX_US - 1);
    EXPECT_TRUE(n.acknowledging());

    n.wait(1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, CompleteAcknowledgeSendsSevenMoreBursts) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 3);
    n.receive_times(acknowledged(PEER_HALF_AND_FULL), 3);
    n.send_bursts(COMPLETE_ACK_BURSTS - 1);
    EXPECT_TRUE(n.acknowledging());

    n.send_bursts(1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Nlp);
}

TEST(Autoneg, AnAcknowledgedCodeWordInconsistentWithTheAbilitiesRestarts) {
    Negotiation n{};
    n.finish_break_link();
    n.receive_times(PEER_HALF_AND_FULL, 3);
    n.receive_times(acknowledged(PEER_HALF_ONLY), 3);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, PeerNextPageAndRemoteFaultDoNotAlterTheOutcome) {
    Negotiation n{};
    n.exchange(PEER_WITH_NEXT_PAGE_AND_REMOTE_FAULT);
    n.wait(0, true);
    EXPECT_EQ(n.autoneg().link(), std::optional{Duplex::Full});
}

TEST(Autoneg, NlpOnlyPeerFallsBackToHalfDuplexAfterAutonegWait) {
    Negotiation n{};
    n.finish_break_link();
    n.wait(0, true);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Nlp);
    EXPECT_FALSE(n.autoneg().link().has_value());

    n.wait(AUTONEG_WAIT_US - 1, true);
    EXPECT_FALSE(n.autoneg().link().has_value());
    n.wait(1, true);
    EXPECT_EQ(n.autoneg().link(), std::optional{Duplex::Half});
}

TEST(Autoneg, ACodeWordDuringParallelDetectionAbortsIt) {
    Negotiation n{};
    n.finish_break_link();
    n.wait(0, true);
    n.receive(PEER_HALF_AND_FULL);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, LinkIntegrityLostDuringParallelDetectionRestarts) {
    Negotiation n{};
    n.finish_break_link();
    n.wait(0, true);
    n.wait(AUTONEG_WAIT_US / 2, false);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, LinkFailInhibitExpiryWithoutLinkIntegrityRestarts) {
    Negotiation n{};
    n.exchange(PEER_HALF_AND_FULL);
    n.wait(LINK_FAIL_INHIBIT_US - 1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Nlp);

    n.wait(1);
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);
}

TEST(Autoneg, LinkFailureRestartsNegotiationThroughBreakLink) {
    Negotiation n{};
    n.exchange(PEER_HALF_AND_FULL);
    n.wait(0, true);
    ASSERT_TRUE(n.autoneg().link().has_value());

    n.wait(0, false);
    EXPECT_FALSE(n.autoneg().link().has_value());
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::None);

    n.finish_break_link();
    EXPECT_EQ(n.autoneg().link_pulses(), Autoneg::LinkPulses::Flp);
    EXPECT_EQ(n.autoneg().code_word(), ADVERTISED_LCW);
}

} // namespace
} // namespace pico_ethernet
