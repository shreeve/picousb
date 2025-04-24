#ifndef PICO_SDK_CONFIG_H
#define PICO_SDK_CONFIG_H

// This file contains overrides for Pico SDK configuration

// Explicitly disable TinyUSB
#define PICO_ENABLE_USB 0
#define PICO_USB_DEVICE_ENABLE 0
#define PICO_USB_HOST_ENABLE 0
#define PICO_STDIO_USB_ENABLE 0
#define PICO_STDIO_USB_DEFAULT_CRLF 0

// Disable TinyUSB device stack
#define CFG_TUD_ENABLED 0

// Disable TinyUSB host stack
#define CFG_TUH_ENABLED 0

// Ensure stdio USB is not enabled
#define PICO_STDIO_USB 0

#endif // PICO_SDK_CONFIG_H