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

Since this repo doesn't use TinyUSB at all, it's been disabled through the CMake configs

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

## Using Wireshark on macOS

```
# Reboot your mac in Recovery Mode (instructions online) and launch a
# terminal. Run the following command to disable SPI. Afterwards, reboot.
$ csrutil disable

# Reboot as normal and find your USB controller name (mine is "XHC2")
$ tcpdump -D

1.ap1 [Up, Running, Wireless, Association status unknown]
2.en1 [Up, Running, Wireless, Associated]
3.awdl0 [Up, Running, Wireless, Associated]
4.llw0 [Up, Running, Wireless, Not associated]
5.utun0 [Up, Running]
6.utun1 [Up, Running]
7.utun2 [Up, Running]
8.utun3 [Up, Running]
9.lo0 [Up, Running, Loopback]
10.anpi0 [Up, Running, Disconnected]
11.anpi1 [Up, Running, Disconnected]
12.en0 [Up, Running, Disconnected]
13.en4 [Up, Running, Disconnected]
14.en5 [Up, Running, Disconnected]
15.en2 [Up, Running, Disconnected]
16.en3 [Up, Running, Disconnected]
17.bridge0 [Up, Running, Disconnected]
18.gif0 [none]
19.stf0 [none]
20.XHC0 [none]
21.XHC1 [none]
22.XHC2 [none]

# Enable capturing USB on the XHC2 interface
sudo ifconfig XHC2 up

# Launch Wireshark and inspect USB traffic on the XHC2 interface
```

### Other links

* https://qsantos.fr/2025/11/21/fixing-the-rp2350-usb-a-not-working-as-usb-host/