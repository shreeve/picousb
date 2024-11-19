#ifndef _USB_COMMON_H
#define _USB_COMMON_H

#include "pico/types.h"
#include "hardware/structs/usb.h"

#define USB_DIR_OUT                      0x00
#define USB_DIR_IN                       0x80

#define USB_REQ_TYPE_STANDARD            0x00
#define USB_REQ_TYPE_CLASS               0x20
#define USB_REQ_TYPE_VENDOR              0x40
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

#define USB_ENDPOINT_TYPE_MASK           0x03
#define USB_ENDPOINT_TYPE_CONTROL        0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS    0x01
#define USB_ENDPOINT_TYPE_BULK           0x02
#define USB_ENDPOINT_TYPE_INTERRUPT      0x03
#define USB_ENDPOINT_DIR_MASK            0x80

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

#define USB_DESCRIPTOR_TYPE_DEVICE       0x01
#define USB_DESCRIPTOR_TYPE_CONFIG       0x02
#define USB_DESCRIPTOR_TYPE_STRING       0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE    0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT     0x05

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_INLINE         __attribute__ ((always_inline)) static inline
#define SDK_NOINLINE       __attribute__ ((noinline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

#define memclr(ptr, len) memset((ptr), 0, (len)) 
#define nop() __asm volatile("nop" ::: "memory")

#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))
#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))

typedef enum {
  transfer_RESULT_SUCCESS = 0,
  transfer_RESULT_FAILED,
  transfer_RESULT_STALLED,
  transfer_RESULT_TIMEOUT,
  transfer_RESULT_INVALID
} transfer_result_t;

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

struct usb_string_descriptor {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bString[];
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

struct usb_urb_control_packet {
    uint8_t  bmRequestType;
    uint8_t  bRequest;
    uint16_t wValue;
    uint16_t wIndex;
    uint16_t wLength;
} __packed;

struct usb_cdc_line_coding {
    uint32_t dwDTERate;   // Data terminal rate in bits per second
    uint8_t  bCharFormat; // Number of stop bits
    uint8_t  bParityType; // Parity bit type, 0=None, 1=Odd, 2=Even, 3=Mark, 4=Space
    uint8_t  bDataBits;   // Number of data bits (5, 6, 7, 8, or 16)
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
typedef struct usb_urb_control_packet         usb_urb_control_packet_t;
typedef struct usb_cdc_line_coding            usb_cdc_line_coding_t;
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

typedef enum {
  CDC_REQUEST_SEND_ENCAPSULATED_COMMAND                    = 0x00, ///< is used to issue a command in the format of the supported control protocol of the Communications Class interface
  CDC_REQUEST_GET_ENCAPSULATED_RESPONSE                    = 0x01, ///< is used to request a response in the format of the supported control protocol of the Communications Class interface.
  CDC_REQUEST_SET_COMM_FEATURE                             = 0x02,
  CDC_REQUEST_GET_COMM_FEATURE                             = 0x03,
  CDC_REQUEST_CLEAR_COMM_FEATURE                           = 0x04,

  CDC_REQUEST_SET_AUX_LINE_STATE                           = 0x10,
  CDC_REQUEST_SET_HOOK_STATE                               = 0x11,
  CDC_REQUEST_PULSE_SETUP                                  = 0x12,
  CDC_REQUEST_SEND_PULSE                                   = 0x13,
  CDC_REQUEST_SET_PULSE_TIME                               = 0x14,
  CDC_REQUEST_RING_AUX_JACK                                = 0x15,

  CDC_REQUEST_SET_LINE_CODING                              = 0x20,
  CDC_REQUEST_GET_LINE_CODING                              = 0x21,
  CDC_REQUEST_SET_CONTROL_LINE_STATE                       = 0x22,
  CDC_REQUEST_SEND_BREAK                                   = 0x23,

  CDC_REQUEST_SET_RINGER_PARMS                             = 0x30,
  CDC_REQUEST_GET_RINGER_PARMS                             = 0x31,
  CDC_REQUEST_SET_OPERATION_PARMS                          = 0x32,
  CDC_REQUEST_GET_OPERATION_PARMS                          = 0x33,
  CDC_REQUEST_SET_LINE_PARMS                               = 0x34,
  CDC_REQUEST_GET_LINE_PARMS                               = 0x35,
  CDC_REQUEST_DIAL_DIGITS                                  = 0x36,
  CDC_REQUEST_SET_UNIT_PARAMETER                           = 0x37,
  CDC_REQUEST_GET_UNIT_PARAMETER                           = 0x38,
  CDC_REQUEST_CLEAR_UNIT_PARAMETER                         = 0x39,
  CDC_REQUEST_GET_PROFILE                                  = 0x3A,

  CDC_REQUEST_SET_ETHERNET_MULTICAST_FILTERS               = 0x40,
  CDC_REQUEST_SET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x41,
  CDC_REQUEST_GET_ETHERNET_POWER_MANAGEMENT_PATTERN_FILTER = 0x42,
  CDC_REQUEST_SET_ETHERNET_PACKET_FILTER                   = 0x43,
  CDC_REQUEST_GET_ETHERNET_STATISTIC                       = 0x44,

  CDC_REQUEST_SET_ATM_DATA_FORMAT                          = 0x50,
  CDC_REQUEST_GET_ATM_DEVICE_STATISTICS                    = 0x51,
  CDC_REQUEST_SET_ATM_DEFAULT_VC                           = 0x52,
  CDC_REQUEST_GET_ATM_VC_STATISTICS                        = 0x53,

  CDC_REQUEST_MDLM_SEMANTIC_MODEL                          = 0x60,
} cdc_management_request_t;

typedef enum {
  CDC_CONTROL_LINE_STATE_DTR  = 0x01,
  CDC_CONTROL_LINE_STATE_RTS  = 0x02,
} usb_cdc_control_line_state_t;

typedef enum {
  CDC_LINE_CODING_STOP_BITS_1   = 0, // 1   bit
  CDC_LINE_CODING_STOP_BITS_1_5 = 1, // 1.5 bits
  CDC_LINE_CODING_STOP_BITS_2   = 2, // 2   bits
} usb_cdc_line_coding_stopbits_t;

typedef enum {
  CDC_LINE_CODING_PARITY_NONE  = 0,
  CDC_LINE_CODING_PARITY_ODD   = 1,
  CDC_LINE_CODING_PARITY_EVEN  = 2,
  CDC_LINE_CODING_PARITY_MARK  = 3,
  CDC_LINE_CODING_PARITY_SPACE = 4,
} usb_cdc_line_coding_parity_t;

typedef struct {
  uint8_t* buffer          ; // buffer pointer
  uint16_t depth           ; // max items

  struct SDR_PACKED {
    uint16_t item_size : 15; // size of each item
    bool overwritable  : 1 ; // ovwerwritable when full
  }

  volatile uint16_t wr_idx ; // write index
  volatile uint16_t rd_idx ; // read index
} tu_fifo_t;

typedef struct {
  bool is_host; // host or device most
  union {
      uint8_t daddr;
      uint8_t rhport;
      uint8_t hwid;
  };
  uint8_t ep_addr;
  uint8_t ep_speed;

  uint16_t ep_packetsize;
  uint16_t ep_bufsize;

  // TODO xfer_fifo can skip this buffer
  uint8_t* ep_buf;

  tu_fifo_t ff;

  // mutex: read if ep rx, write if e tx
  uint8_t mutex;

}tu_edpt_stream_t;

#define MAX_CDC_IN_BUFFER_SIZE  64
#define MAX_CDC_OUT_BUFFER_SIZE 64
#define MAX_PACKET_SIZE 64

typedef struct {
  uint8_t daddr;
  uint8_t bInterfaceNumber;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;

  uint8_t ep_notif;
  uint8_t serial_drid; // Serial Driver ID
  bool mounted;        // Enumeration is complete

  SDK_ALIGNED(4) usb_cdc_line_coding_t line_coding; // Baudrate, stop bits, parity, data width
  uint8_t line_state;                               // DTR (bit0), RTS (bit1)

  usb_cdc_line_coding_t requested_line_coding;

  usb_transfer_cb_t user_control_cb;

  struct {
    tu_edpt_stream_t tx;
    tu_edpt_stream_t rx;

    uint8_t tx_ff_buf[MAX_CDC_OUT_BUFFER_SIZE];
    SDK_ALIGNED(4) uint8_t tx_ep_buf[MAX_PACKET_SIZE];

    uint8_t rx_ff_buf[MAX_CDC_IN_BUFFER_SIZE];
    SDK_ALIGNED(4) uint8_t rx_ep_buf[MAX_PACKET_SIZE];
  } stream;
} cdch_interface_t;


// forward declaration
struct usb_transfer_s;
typedef struct usb_transfer_s usb_transfer_t;

// void my_transfer_callback(usb_transfer_t* transfer) { }

typedef void (*usb_transfer_cb_t)(usb_transfer_t* transfer);

struct usb_transfer_s {
  uint8_t daddr;
  uint8_t ep_addr;
  uint8_t TU_RESERVED;      // reserved
  transfer_result_t result;

  uint32_t actual_len;      // excluding setup packet

  union {
    usb_urb_control_packet_t const* setup; // setup packet pointer if control transfer
    uint32_t buflen;                     // expected length if not control transfer (not available in callback)
  };

  uint8_t* buffer;           // not available in callback if not control transfer
  usb_transfer_cb_t complete_cb;
  uintptr_t user_data;

  // uint32_t timeout_ms;    // place holder, not supported yet
};

typedef struct {
  uint8_t daddr;
  usb_interface_descriptor_t desc;
} tuh_itf_info_t;


typedef enum {
  XFER_RESULT_SUCCESS = 0,
  XFER_RESULT_FAILED,
  XFER_RESULT_STALLED,
  XFER_RESULT_TIMEOUT,
  XFER_RESULT_INVALID
} usb_transfer_result_t;

#define BYTES_TO_U16(_high_byte, _low_byte)   ((uint16_t) (((_high_byte) << 8) | (_low_byte)))
#define HIGH_BYTE_FROM_U16(_u16)     ((uint8_t) (((_u16) >> 8) & 0x00ff))
#define LOW_BYTE_FROM_U16(_u16)      ((uint8_t) ((_u16)       & 0x00ff))
#define U16_TO_BYTES_BE(_u16)   EXTRACT_HIGH_BYTE_FROM_U16(_u16), EXTRACT_LOW_BYTE_FROM_U16(_u16)
#define U16_TO_BYTES_LE(_u16)   EXTRACT_LOW_BYTE_FROM_U16(_u16), EXTRACT_HIGH_BYTE_FROM_U16(_u16)

#define TUSB_INDEX_INVALID_8
#define TU_ARRAY_SIZE(_arr)   ( sizeof(_arr) / sizeof(_arr[0]) )

#endif /* _USB_COMMON_H */