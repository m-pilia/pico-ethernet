# Build file for our patched TinyUSB (see MODULE.bazel). Based on the Pico SDK's
# tinyusb.BUILD, with lib/networking added to the include path: the net class
# driver compiles ECM and RNDIS from a single file guarded by CFG_TUD_ECM_RNDIS,
# so enabling CDC-ECM needs the RNDIS protocol headers (rndis_protocol.h,
# ndis.h). We deliberately do NOT compile rndis_reports.c because it pulls in
# lwIP; RNDIS is never advertised at runtime (ECM-only descriptors), and the one
# symbol it would provide (rndis_class_set_handler) is stubbed in our firmware.

package(default_visibility = ["//visibility:public"])

exports_files(
    glob(["**/*"]),
    visibility = ["//visibility:public"],
)

cc_library(
    name = "tinyusb",
    srcs = [
        "src/class/audio/audio_device.c",
        "src/class/cdc/cdc_device.c",
        "src/class/dfu/dfu_device.c",
        "src/class/dfu/dfu_rt_device.c",
        "src/class/hid/hid_device.c",
        "src/class/midi/midi_device.c",
        "src/class/msc/msc_device.c",
        "src/class/net/ecm_rndis_device.c",
        "src/class/net/ncm_device.c",
        "src/class/usbtmc/usbtmc_device.c",
        "src/class/vendor/vendor_device.c",
        "src/class/video/video_device.c",
        "src/common/tusb_fifo.c",
        "src/device/usbd.c",
        "src/portable/raspberrypi/rp2040/dcd_rp2040.c",
        "src/portable/raspberrypi/rp2040/rp2040_usb.c",
        "src/tusb.c",
    ],
    hdrs = glob([
        "src/**/*.h",
        "hw/bsp/*.h",
        "hw/bsp/rp2040/**/*.h",
        "lib/networking/*.h",
    ]),
    includes = [
        "hw",
        "hw/bsp",
        "lib/networking",
        "src",
    ],
    deps = ["@pico-sdk//src/rp2_common/tinyusb:tinyusb_port"],
)
