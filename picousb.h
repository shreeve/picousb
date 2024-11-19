#ifndef PICOUSB_H
#define PICOUSB_H

// =============================================================================
// PicoUSB - A smaller than tiny USB Host library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: November 18, 2024
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach for TinyUSB and https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for his https://github.com/Panda381/PicoLibSDK
// =============================================================================

#include <stdio.h>                // For printf
#include <string.h>               // For memcpy

#include "pico/stdlib.h"          // Pico stdlib
#include "pico/util/queue.h"      // Multicore and IRQ safe queue
#include "hardware/regs/usb.h"    // USB hardware registers from pico-sdk
#include "hardware/structs/usb.h" // USB hardware structs from pico-sdk
#include "hardware/irq.h"         // Interrupts and definitions
#include "hardware/resets.h"      // Resetting the native USB controller

#include "usb_common.h"           // USB 2.0 definitions
#include "helpers.h"              // Helper functions

// ==[ Constants ]==============================================================

#define MAX_HUBS        1 // root +  0
#define MAX_DEVICES     2 // dev0 +  1
#define MAX_ENDPOINTS  16 // epx  + 15
#define MAX_CTRL_BUF  320 // Large enough to handle a full config descriptor

#define MAKE_U16(x, y) (((x) << 8) | ((y)     ))
#define SWAP_U16(x)    (((x) >> 8) | ((x) << 8))

#define SDK_ALIGNED(bytes) __attribute__ ((aligned(bytes)))
#define SDK_FAST           __attribute__ ((noinline, section(".time_critical")))
#define SDK_INJECT         __attribute__ ((always_inline)) static inline
#define SDK_INLINE         __attribute__ ((always_inline))        inline
#define SDK_NOINLINE       __attribute__ ((noinline))
#define SDK_PACKED         __attribute__ ((packed))
#define SDK_WEAK           __attribute__ ((weak))

#define memclr(ptr, len) memset((ptr), 0, (len))
#define nop() __asm volatile("nop" ::: "memory")

#define usb_hw_clear ((usb_hw_t *) hw_clear_alias_untyped(usb_hw))
#define usb_hw_set   ((usb_hw_t *) hw_set_alias_untyped  (usb_hw))

// ==[ Debug ]==================================================================

#define DEBUG_ROW \
    "•───────•──────•─────────────────────────────────────•────────────•\n"

extern uint8_t usb_debug_level; // Dynamic debug level

void usb_debug(uint8_t level);

// ==[ Hubs ]===================================================================

typedef struct {
    // Nothing yet
} hub_t;

static hub_t hubs[MAX_HUBS], *root = hubs;

// ==[ Devices ]================================================================

typedef enum {
    DISCONNECTED,
    LOW_SPEED,
    FULL_SPEED,
} connection_t;

typedef enum {
    DEVICE_DISCONNECTED,
    DEVICE_ALLOCATED,
    DEVICE_ENUMERATING,
    DEVICE_ADDRESSED,
    DEVICE_ACTIVE,
    DEVICE_READY,
    DEVICE_SUSPENDED,
} device_status_t;

typedef struct {
    uint8_t  dev_addr    ; // Device address
    uint8_t  state       ; // Current device state
    uint8_t  speed       ; // Device speed (0:disconnected, 1:full, 2:high)
    uint16_t maxsize0    ; // Maximum packet size for EP0

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

extern device_t devices[MAX_DEVICES], *dev0;

// ==[ Endpoints ]==============================================================

typedef enum {
    SINGLE_BUFFER = EP_CTRL_INTERRUPT_PER_BUFFER,
    DOUBLE_BUFFER = EP_CTRL_DOUBLE_BUFFERED_BITS
                  | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER,
} buffering_t;

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct {

    // USB config
    uint8_t    dev_addr  ; // Device address
    uint8_t    ep_addr   ; // Endpoint address (0x81 is EP1/IN)
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   interval  ; // Polling interval in ms
    uint16_t   maxsize   ; // Maximum packet size

    // Memory buffers
    volatile               // Data buffer is volative
    uint8_t   *buf       ; // Data buffer in DPSRAM
    uint8_t   *user_buf  ; // User buffer in DPSRAM, RAM, or flash

    // Hardware registers
    io_rw_32  *ecr       ; // Endpoint control register
    io_rw_32  *bcr       ; // Buffer control register

    // Setup status
    bool       configured; // Endpoint is configured

    // Transfer details
    bool       active    ; // Transfer is active
    bool       setup     ; // Setup packet flag
    uint8_t    data_pid  ; // Toggle between DATA0/DATA1 packets
    uint16_t   bytes_left; // Bytes left to transfer
    uint16_t   bytes_done; // Bytes done transferring
    endpoint_c cb        ; // Callback function
} endpoint_t;

extern endpoint_t eps[MAX_ENDPOINTS], *epx;

// ==[ Buffers ]================================================================

// ==[ Transactions ]===========================================================

// ==[ Transfers ]==============================================================

typedef enum {
    TRANSFER_SUCCESS,
    TRANSFER_FAILED,  // not used yet
    TRANSFER_STALLED, // not used yet
    TRANSFER_TIMEOUT, // not used yet
    TRANSFER_INVALID, // not used yet
} transfer_status_t;

void transfer_zlp(void *arg);
void control_transfer(device_t *dev, usb_setup_packet_t *setup);
void command(device_t *dev, uint8_t bmRequestType, uint8_t bRequest,
             uint16_t wValue, uint16_t wIndex, uint16_t wLength);
void bulk_transfer(endpoint_t *ep, uint8_t *ptr, uint16_t len);
void reset_ftdi(device_t *dev);

// ==[ Descriptors ]============================================================

// ==[ Classes ]================================================================

// ==[ Drivers ]================================================================

// ==[ Enumeration ]============================================================

// ==[ Callbacks ]==============================================================

void on_device_active();

// ==[ Tasks ]==================================================================

enum {
    TASK_CONNECT,
    TASK_TRANSFER,
    TASK_CALLBACK,
};

typedef struct {
    uint8_t type;
    uint32_t guid;

    union {

        // Device connect or disconnect
        struct {
            uint8_t speed; // LS is 1.5 Mbps, FS is 12Mbps
        } connect;

        // Transfer has completed
        struct {
            uint8_t    status    ; // Transfer status
            uint8_t    dev_addr  ; // Device address
            uint8_t    ep_num    ; // Endpoint number (direction not included)
            uint8_t   *user_buf  ; // User buffer in DPSRAM, RAM, or flash
            uint16_t   len       ; // Bytes transferred
            endpoint_c cb        ; // Callback function
        } transfer;

        // Generic function callback
        struct {
            void (*fn) (void *); // Function pointer
            void *arg          ; // One argument
        } callback;
    };
} task_t;

void queue_callback(void (*fn)(void *), void *arg);
void usb_task();

// ==[ Interrupts ]=============================================================

// ==[ Setup USB Host ]=========================================================

void usb_init();

// ==[ End ]====================================================================

#endif // PICOUSB_H
