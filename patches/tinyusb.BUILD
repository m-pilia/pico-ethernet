# Build file for our patched TinyUSB (see MODULE.bazel). Based on the Pico SDK's
# tinyusb.BUILD. We compile the CDC-NCM net class driver (ncm_device.c); the
# combined ECM/RNDIS driver is omitted since we advertise NCM-only descriptors.

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
    ]),
    includes = [
        "hw",
        "hw/bsp",
        "src",
    ],
    deps = ["@pico-sdk//src/rp2_common/tinyusb:tinyusb_port"],
)
