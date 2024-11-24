# PicoUSB

A smaller than tiny USB Host library for the Raspberry Pi Pico/W (rp2040/rp2350).

## Summary

The rp2040/rp2350 chips in the Raspberry Pi Pico/W boards support USB Host and
USB Device modes. Most examples and implementations use the excellent
[TinyUSB](https://github.com/hathach/tinyusb) library to provide this support.
However, TinyUSB has some limitations. These include a somewhat hard coded
initial configuration that requires the code to be recompiled when changes are
made.

## Goals

The goal of this repo is to offer a simple USB Host library, using only the code
from https://github.com/raspberrypi/pico-sdk. Ideally, this could help form the
basis, or at least inspire, other efforts. Since the initial target is for the
Pico/W boards and since the goal is to be tiny, even "tinier" than TinyUSB, it
seems the name PicoUSB is fitting.

## Helpful USB Information

* [USB Overview](https://github.com/shreeve/picousb/blob/main/docs/usb-overview.md)

## Host Example

The current status, as of the end of November 2024, showing the output from
[enumeration of an FTDI board](https://github.com/shreeve/picousb/blob/main/docs/enumeration.md).

## Compiling

Since we are taking over the physical USB connector on the board, the
easiest way to program the board and test things out is by using a
pico debug unit. These can be purchased from numerous sources. The .vscode
directory contains some configuration for this setup.

## Disable TinyUSB

Since this repo doesn't use TinyUSB at all, we need to disable it from the
pico-sdk code. The only way I found to do this is by commenting out the
following lines in `${PICO_SDK_PATH}/src/rp2_common/CMakeLists.txt`.

```
# pico_add_subdirectory(tinyusb)
# pico_add_subdirectory(pico_stdio_usb)
```

## RP2040 Notes

```
NOTE: A hardware polled endpoint needs FOUR things to fire:

1) usb_hw->int_ep_ctrl |= (1 << epn)                          ; // Activate the endpoint
2) *ep->ecr            |= EP_CTRL_ENABLE_BITS                 ; // Enable EP in ecr
3) *bcr                 = hold | USB_BUF_CTRL_AVAIL           ; // Buffer is available
4) usb_hw->sie_ctrl     = sie  | USB_SIE_CTRL_START_TRANS_BITS; // Start transfer
```

## License

BSD-3-Clause license, the same as code in [pico-examples](https://github.com/raspberrypi/pico-examples/tree/master/usb/device/dev_lowlevel).
