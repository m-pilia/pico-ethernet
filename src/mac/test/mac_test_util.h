// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia

#ifndef MAC_TEST_MAC_TEST_UTIL_H
#define MAC_TEST_MAC_TEST_UTIL_H

#include "src/mac/mac_address.h"

namespace pico_ethernet {

constexpr MacAddress test_mac_address() {
    constexpr MacAddress::Bytes BYTES{0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
    return MacAddress{BYTES};
}

} // namespace pico_ethernet

#endif // MAC_TEST_MAC_TEST_UTIL_H
