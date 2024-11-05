#ifndef _USB_COMMON_H
#define _USB_COMMON_H

#include "pico/types.h"
#include "hardware/structs/usb.h"

#define USB_DIR_OUT                      0x00
#define USB_DIR_IN                       0x80

#define USB_REQ_TYPE_STANDARD            0x00
#define USB_REQ_TYPE_TYPE_CLASS          0x20
#define USB_REQ_TYPE_TYPE_VENDOR         0x40
#define USB_REQ_TYPE_TYPE_MASK           0x60

#define USB_REQ_TYPE_RECIPIENT_DEVICE    0x00
#define USB_REQ_TYPE_RECIPIENT_INTERFACE 0x01
#define USB_REQ_TYPE_RECIPIENT_ENDPOINT  0x02
#define USB_REQ_TYPE_RECIPIENT_MASK      0x1f

#define USB_TRANSFER_TYPE_CONTROL        0x00
#define USB_TRANSFER_TYPE_ISOCHRONOUS    0x01
#define USB_TRANSFER_TYPE_BULK           0x02
#define USB_TRANSFER_TYPE_INTERRUPT      0x03
#define USB_TRANSFER_TYPE_BITS           0x03

#define USB_DT_DEVICE                    0x01
#define USB_DT_CONFIG                    0x02
#define USB_DT_STRING                    0x03
#define USB_DT_INTERFACE                 0x04
#define USB_DT_ENDPOINT                  0x05
#define USB_DT_DEVICE_QUALIFIER          0x06
#define USB_DT_OTHER_SPEED_CONFIG        0x07
#define USB_DT_INTERFACE_POWER           0x08
#define USB_DT_OTG                       0x09
#define USB_DT_DEBUG                     0x0a
#define USB_DT_INTERFACE_ASSOCIATION     0x0b

#define USB_DT_CS_DEVICE                 0x21
#define USB_DT_CS_CONFIGURATION          0x22
#define USB_DT_CS_STRING                 0x23
#define USB_DT_CS_INTERFACE              0x24
#define USB_DT_CS_ENDPOINT               0x25

#define USB_REQUEST_GET_STATUS           0x00
#define USB_REQUEST_CLEAR_FEATURE        0x01
#define USB_REQUEST_SET_FEATURE          0x03
#define USB_REQUEST_SET_ADDRESS          0x05
#define USB_REQUEST_GET_DESCRIPTOR       0x06
#define USB_REQUEST_SET_DESCRIPTOR       0x07
#define USB_REQUEST_GET_CONFIGURATION    0x08
#define USB_REQUEST_SET_CONFIGURATION    0x09
#define USB_REQUEST_GET_INTERFACE        0x0a
#define USB_REQUEST_SET_INTERFACE        0x0b
#define USB_REQUEST_SYNC_FRAME           0x0c

#define USB_REQUEST_MSC_GET_MAX_LUN      0xfe
#define USB_REQUEST_MSC_RESET            0xff

#define USB_FEAT_ENDPOINT_HALT           0x00
#define USB_FEAT_DEVICE_REMOTE_WAKEUP    0x01
#define USB_FEAT_TEST_MODE               0x02

struct usb_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
} __packed;

struct usb_device_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __packed;

struct usb_configuration_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __packed;

struct usb_interface_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bInterfaceNumber;
    uint8_t  bAlternateSetting;
    uint8_t  bNumEndpoints;
    uint8_t  bInterfaceClass;
    uint8_t  bInterfaceSubClass;
    uint8_t  bInterfaceProtocol;
    uint8_t  iInterface;
} __packed;

struct usb_string_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bUnicode;
} __packed;

struct usb_endpoint_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __packed;

struct usb_endpoint_descriptor_long {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
    uint8_t  bRefresh;
    uint8_t  bSyncAddr;
} __packed;

struct usb_setup_packet {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __packed;

struct usb_interface_assoc_descriptor {
	uint8_t  bLength;
	uint8_t  bDescriptorType;
	uint8_t  bFirstInterface;
	uint8_t  bInterfaceCount;
	uint8_t  bFunctionClass;
	uint8_t  bFunctionSubClass;
	uint8_t  bFunctionProtocol;
	uint8_t  iFunction;
} __attribute__ ((packed));

typedef struct usb_descriptor                 usb_descriptor_t;
typedef struct usb_device_descriptor          usb_device_descriptor_t;
typedef struct usb_configuration_descriptor   usb_configuration_descriptor_t;
typedef struct usb_interface_descriptor       usb_interface_descriptor_t;
typedef struct usb_string_descriptor          usb_string_descriptor_t;
typedef struct usb_endpoint_descriptor        usb_endpoint_descriptor_t;
typedef struct usb_endpoint_descriptor_long   usb_endpoint_descriptor_long_t;
typedef struct usb_setup_packet               usb_setup_packet_t;
typedef struct usb_interface_assoc_descriptor usb_interface_assoc_descriptor_t;

typedef enum {
    USB_CLASS_UNSPECIFIED          = 0x00,
    USB_CLASS_AUDIO                = 0x01,
    USB_CLASS_CDC                  = 0x02,
    USB_CLASS_HID                  = 0x03,
    USB_CLASS_RESERVED_4           = 0x04,
    USB_CLASS_PHYSICAL             = 0x05,
    USB_CLASS_IMAGE                = 0x06,
    USB_CLASS_PRINTER              = 0x07,
    USB_CLASS_MSC                  = 0x08,
    USB_CLASS_HUB                  = 0x09,
    USB_CLASS_CDC_DATA             = 0x0a,
    USB_CLASS_SMART_CARD           = 0x0b,
    USB_CLASS_RESERVED_12          = 0x0c,
    USB_CLASS_CONTENT_SECURITY     = 0x0d,
    USB_CLASS_VIDEO                = 0x0e,
    USB_CLASS_PERSONAL_HEALTHCARE  = 0x0f,
    USB_CLASS_AUDIO_VIDEO          = 0x10,
    USB_CLASS_BILLBOARD            = 0x11,
    USB_CLASS_DIAGNOSTIC           = 0xdc,
    USB_CLASS_WIRELESS_CONTROLLER  = 0xe0,
    USB_CLASS_MISC                 = 0xef,
    USB_CLASS_APPLICATION_SPECIFIC = 0xfe,
    USB_CLASS_VENDOR_SPECIFIC      = 0xff,
} usb_class_code_t; // USB Device Class Codes

// ==[ Minimal Audio Class support ]============================================

// A.3 - Audio Function Protocol Codes
typedef enum {
    USB_AUDIO_FUNCTION_PROTOCOL_CODE_UNDEF = 0x00, // Undefined
    USB_AUDIO_FUNCTION_PROTOCOL_CODE_V2    = 0x20, // Version 2.0
} usb_audio_function_protocol_code_t;

// A.5 - Audio Interface Subclass Codes
typedef enum {
    USB_SUBCLASS_AUDIO_UNDEFINED      = 0x00, // Undefined
    USB_SUBCLASS_AUDIO_CONTROL        = 0x01, // Audio Control
    USB_SUBCLASS_AUDIO_STREAMING      = 0x02, // Audio Streaming
    USB_SUBCLASS_AUDIO_MIDI_STREAMING = 0x03, // MIDI Streaming
} usb_audio_subclass_t;

// ==[ Minimal CDC Class support ]==============================================

typedef enum {
    USB_SUBCLASS_CDC_DIRECT_LINE_CONTROL_MODEL      = 0x01, // Direct Line Control Model
    USB_SUBCLASS_CDC_ABSTRACT_CONTROL_MODEL         = 0x02, // Abstract Control Model
    USB_SUBCLASS_CDC_TELEPHONE_CONTROL_MODEL        = 0x03, // Telephone Control Model
    USB_SUBCLASS_CDC_MULTICHANNEL_CONTROL_MODEL     = 0x04, // Multi-Channel Control Model
    USB_SUBCLASS_CDC_CAPI_CONTROL_MODEL             = 0x05, // CAPI Control Model
    USB_SUBCLASS_CDC_ETHERNET_CONTROL_MODEL         = 0x06, // Ethernet Networking Control Model
    USB_SUBCLASS_CDC_ATM_NETWORKING_CONTROL_MODEL   = 0x07, // ATM Networking Control Model
    USB_SUBCLASS_CDC_WIRELESS_HANDSET_CONTROL_MODEL = 0x08, // Wireless Handset Control Model
    USB_SUBCLASS_CDC_DEVICE_MANAGEMENT              = 0x09, // Device Management
    USB_SUBCLASS_CDC_MOBILE_DIRECT_LINE_MODEL       = 0x0a, // Mobile Direct Line Model
    USB_SUBCLASS_CDC_OBEX                           = 0x0b, // OBEX
    USB_SUBCLASS_CDC_ETHERNET_EMULATION_MODEL       = 0x0c, // Ethernet Emulation Model
    USB_SUBCLASS_CDC_NETWORK_CONTROL_MODEL          = 0x0d, // Network Control Model
} usb_cdc_subclass_t;

#endif
