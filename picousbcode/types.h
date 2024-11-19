
#include "pico/stdlib.h"
# include "usb_common.h"
// ==[ Endpoints ]==============================================================

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct {
    uint8_t    dev_addr  ; // Device address // HOST ONLY
    uint8_t    ep_addr   ; // Endpoint address
    uint8_t    if_id     ; // Interface ID
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   maxsize   ; // Maximum packet size
    uint16_t   interval  ; // Polling interval in ms
    uint8_t    data_pid  ; // Toggle between DATA0/DATA1 packets
    bool       configured; // Endpoint is configured
    bool       active    ; // Transfer is active
    bool       setup     ; // Setup packet flag

    // Hardware registers and data buffer
    io_rw_32  *ecr       ; // Endpoint control register
    io_rw_32  *bcr       ; // Buffer control register
    volatile               // Data buffer is volative
    uint8_t   *buf       ; // Data buffer in DPSRAM

    // Shared with application code
    uint8_t   *user_buf  ; // User buffer in DPSRAM, RAM, or flash
    uint16_t   bytes_left; // Bytes left to transfer
    uint16_t   bytes_done; // Bytes done transferring
    endpoint_c cb        ; // Callback function
} endpoint_t;

// ==[ Device ]========================================================

enum {
    DISCONNECTED,
    LOW_SPEED,
    FULL_SPEED,
};

enum {
    DEVICE_DISCONNECTED,
    DEVICE_ALLOCATED,
    DEVICE_ENUMERATING,
    DEVICE_ADDRESSED,
    DEVICE_ACTIVE,
    DEVICE_SUSPENDED,
};

typedef struct {
    uint8_t  state       ; // Current device state
    uint8_t  speed       ; // Device speed (0:disconnected, 1:full, 2:high)
    uint8_t  class       ; // Device class
    uint8_t  subclass    ; // Device subclass
    uint8_t  protocol    ; // Device protocol
    uint16_t vid         ; // Vendor Id  (0x0403: FTDI)
    uint16_t pid         ; // Product Id (0xcd18: Abaxis Piccolo Xpress)
    uint16_t version     ; // Version number
    uint8_t  manufacturer; // String index of manufacturer
    uint8_t  product     ; // String index of product
    uint8_t  serial      ; // String index of serial number
} device_t;

// ==[ Interface ]=============================================================

// struct usb_interface_descriptor {
//     uint8_t  bLength;
//     uint8_t  bDescriptorType;
//     uint8_t  bInterfaceNumber;
//     uint8_t  bAlternateSetting;
//     uint8_t  bNumEndpoints;
//     uint8_t  bInterfaceClass;
//     uint8_t  bInterfaceSubClass;
//     uint8_t  bInterfaceProtocol;
//     uint8_t  iInterface;
// } __packed;

typedef struct {
    uint8_t  type;
    uint8_t  class;
    uint8_t  subclass;
    uint8_t  protocol;
    uint8_t  num_ep;
    uint8_t  dev_id;
} interface_t;

// ==[ Transfer ]==============================================================

enum {
    TRANSFER_SUCCESS, // not used yet
    TRANSFER_FAILED,  // not used yet
    TRANSFER_STALLED, // not used yet
    TRANSFER_TIMEOUT, // not used yet
    TRANSFER_INVALID, // not used yet
};

enum {
    USB_SIE_CTRL_BASE = USB_SIE_CTRL_PULLDOWN_EN_BITS   // Ready for devices
                      | USB_SIE_CTRL_VBUS_EN_BITS       // Supply VBUS
                      | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Enable low speed
                      | USB_SIE_CTRL_SOF_EN_BITS        // Enable full speed
};

// ==[ Enumeration ]===========================================================

enum {
    ENUMERATION_START,
    ENUMERATION_GET_MAXSIZE,
    ENUMERATION_SET_ADDRESS,
    ENUMERATION_GET_DEVICE,
    ENUMERATION_GET_CONFIG_SHORT,
    ENUMERATION_GET_CONFIG_FULL,
    ENUMERATION_SET_CONFIG,
    FTDI_RESET_REQUEST,
    FTDI_SET_BAUDRATE_REQUEST,
};

// ==[ Task ]==================================================================

enum {
    TASK_CALLBACK,
    TASK_CONNECT,
    TASK_TRANSFER,
};

typedef struct {
    uint8_t type;
    uint32_t guid;

    union {
        struct {
            void (*fn) (void *);
            void *arg;
        } callback;

        struct {
            uint8_t speed;
        } connect;

        struct {
            endpoint_t *ep;     // TODO: Risky to just sent this pointer?
            uint16_t    len;    // TODO: Should we point to a safe buffer of this length?
            uint8_t     status; // TODO: Are we even using this?
        } transfer;
    };
} task_t;

// ==[ Association ] ===========================================================


// ==[ Driver ]================================================================

typedef struct {
  const char *name;
  void (* const init  )(void);
//   bool (* const open  )(uint8_t dev_addr, const usb_interface_descriptor_t *ifd,
//                         uint16_t len);
//   bool (* const config)(uint8_t dev_addr, uint8_t itf_num);
//   bool (* const cb    )(uint8_t dev_addr, uint8_t ep_addr, // Ugh... xfer_result_t result,
//                         uint32_t xferred_bytes);
//   void (* const close )(uint8_t dev_addr);
} driver_t;

#define _DRIVER_COUNT(arr) (sizeof(arr) / sizeof(driver_t))