// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Martino Pilia
//
// Pico 2 Ethernet NIC - TinyUSB configuration
// Phase 1: CDC-ECM device, full speed, single configuration.
//
// CFG_TUSB_MCU / CFG_TUSB_OS / CFG_TUSB_DEBUG are supplied by the Pico SDK
// tinyusb_port target, so they are only defaulted defensively here.

#ifndef PICO_ETHERNET_TUSB_CONFIG_H
#define PICO_ETHERNET_TUSB_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef CFG_TUSB_MCU
#error "CFG_TUSB_MCU must be defined (provided by the Pico SDK tinyusb port)"
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS OPT_OS_NONE
#endif

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))
#endif

// Enable the device stack at full speed (the RP2350 USB controller is USB 1.1).
#define CFG_TUD_ENABLED 1
#define CFG_TUSB_RHPORT0_MODE (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)

#define CFG_TUD_ENDPOINT0_SIZE 64

// Network class only. The net class has two drivers, ECM/RNDIS and NCM; we use
// ECM (Linux binds cdc_ether natively). Exactly one must be enabled.
#define CFG_TUD_ECM_RNDIS 1
#define CFG_TUD_NCM 0

// Standard Ethernet MTU (frame payload up to 1514 bytes).
#define CFG_TUD_NET_MTU 1514

#ifdef __cplusplus
}
#endif

#endif // PICO_ETHERNET_TUSB_CONFIG_H
