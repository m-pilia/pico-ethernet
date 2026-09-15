// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#include <gtest/gtest.h>

#include <cstdint>

#include "src/mac/ethernet_frame.h"
#include "src/phy/phy_stats.h"

namespace pico_ethernet {
namespace {

TEST(RxStatsTest, RecordErrorIncrementsMatchingField) {
    RxStats stats{};
    stats.record_error(FrameError::BadPreamble);
    stats.record_error(FrameError::Runt);
    stats.record_error(FrameError::Giant);
    stats.record_error(FrameError::BadFcs);
    stats.record_error(FrameError::BadFcs);
    stats.record_error(FrameError::Filtered);

    EXPECT_EQ(stats.bad_preamble, 1u);
    EXPECT_EQ(stats.runt, 1u);
    EXPECT_EQ(stats.giant, 1u);
    EXPECT_EQ(stats.bad_fcs, 2u);
    EXPECT_EQ(stats.filtered, 1u);
}

TEST(RxStatsTest, TooLongIsNotAnRxError) {
    RxStats stats{};
    stats.record_error(FrameError::TooLong);
    EXPECT_EQ(stats.error_total(), 0u);
}

TEST(RxStatsTest, ErrorTotalExcludesFilteredAndDelivered) {
    RxStats stats{};
    stats.delivered = 10;
    stats.filtered = 3;
    stats.bad_preamble = 1;
    stats.runt = 2;
    stats.giant = 1;
    stats.bad_fcs = 4;
    stats.decode_error = 5;
    stats.carrier_glitch = 6;

    EXPECT_EQ(stats.error_total(), 1u + 2u + 1u + 4u + 5u + 6u);
}

TEST(EthernetStatisticTest, SelectorsMapToTheRightCounters) {
    const TxStats tx{.build_failed = 2, .usb_tx_overflow = 3, .sent = 7, .underrun = 1};
    RxStats rx{};
    rx.delivered = 11;
    rx.bad_fcs = 4;
    rx.runt = 1;
    rx.decode_error = 2;

    EXPECT_EQ(ethernet_statistic(EthernetStatistic::XmitOk, tx, rx), 7u);
    EXPECT_EQ(ethernet_statistic(EthernetStatistic::RcvOk, tx, rx), 11u);
    EXPECT_EQ(ethernet_statistic(EthernetStatistic::XmitError, tx, rx), 6u);
    EXPECT_EQ(ethernet_statistic(EthernetStatistic::RcvError, tx, rx), rx.error_total());
    EXPECT_EQ(ethernet_statistic(EthernetStatistic::RcvCrcError, tx, rx), 4u);
    EXPECT_EQ(ethernet_statistic(EthernetStatistic::XmitUnderrun, tx, rx), 1u);
}

TEST(EthernetStatisticTest, UnknownSelectorHasNoValue) {
    const TxStats tx{};
    const RxStats rx{};
    EXPECT_FALSE(ethernet_statistic(static_cast<std::uint16_t>(0x99), tx, rx).has_value());
}

// The advertised bitmap must agree with what the handler actually answers: a bit
// is set exactly for the selectors ethernet_statistic() resolves to a value.
TEST(EthernetStatisticTest, AdvertisedBitmapMatchesHandledSelectors) {
    const TxStats tx{};
    const RxStats rx{};
    for (std::uint32_t selector{0}; selector <= 32; ++selector) {
        const std::uint32_t bit{selector == 0 ? 0u : 1u << (selector - 1)};
        const bool advertised{(ETHERNET_STATISTICS_BITMAP & bit) != 0};
        const bool handled{ethernet_statistic(static_cast<std::uint16_t>(selector), tx, rx).has_value()};
        EXPECT_EQ(advertised, handled) << "selector 0x" << std::hex << selector;
    }
}

} // namespace
} // namespace pico_ethernet
