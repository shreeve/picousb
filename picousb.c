// =============================================================================
// PicoUSB - A USB host and device library for the rp2040 (Raspberry Pi Pico/W)
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: November 5, 2024
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach for TinyUSB and https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for his https://github.com/Panda381/PicoLibSDK
//
// NOTE: For now, this only supports control and bulk transfers on EPX
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

int debug_mode = 1;               // Dynamically enables or disables printf()

// ==[ PicoUSB ]================================================================

#define USER_HUBS      0 // No hub support yet
#define USER_DEVICES   4 // Not including dev0

enum {
    MAX_DEVICES   =   1 + USER_DEVICES,
    MAX_ENDPOINTS =  16, // 1 EPX + 15 polled hardware endpoints
    MAX_TEMP      = 255, // Must be 255 or less
};

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

static uint8_t ctrl_buf[MAX_TEMP]; // Shared buffer for control transfers
static uint8_t REMOVE_THIS[2048]; // FIXME: Remove this!

void usb_task(); // Forward declaration

// ==[ Devices ]================================================================

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
    DEVICE_READY,
    DEVICE_SUSPENDED,
};

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

static device_t devices[MAX_DEVICES], *dev0 = devices;

device_t *get_device(uint8_t dev_addr) {
    if (dev_addr < MAX_DEVICES) return &devices[dev_addr];
    panic("Device %u does not exist", dev_addr);
    return NULL;
}

device_t *next_device() {
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        if (devices[i].state == DEVICE_DISCONNECTED) {
            devices[i].state =  DEVICE_ALLOCATED;
            devices[i].dev_addr = i;
            return &devices[i];
        }
    }
    panic("No free devices remaining");
    return NULL;
}

void reset_device(uint8_t dev_addr) {
    device_t *dev = get_device(dev_addr);
    memclr(dev, sizeof(device_t));
}

void reset_devices() {
    memclr(devices, sizeof(devices));
}

// ==[ Endpoints ]==============================================================

enum {
    SINGLE_BUFFER = EP_CTRL_INTERRUPT_PER_BUFFER,
    DOUBLE_BUFFER = EP_CTRL_DOUBLE_BUFFERED_BITS
                  | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER,
};

typedef void (*endpoint_c)(uint8_t *buf, uint16_t len);

typedef struct {
    uint8_t    dev_addr  ; // Device address
    uint8_t    ep_addr   ; // Endpoint address
    uint8_t    type      ; // Transfer type: control/bulk/interrupt/isochronous
    uint16_t   interval  ; // Polling interval in ms
    uint16_t   maxsize   ; // Maximum packet size
    bool       configured; // Endpoint is configured

    io_rw_32  *ecr       ; // Endpoint control register
    io_rw_32  *bcr       ; // Buffer control register
    volatile               // Data buffer is volative
    uint8_t   *buf       ; // Data buffer in DPSRAM

    bool       setup     ; // Setup packet flag
    uint8_t    data_pid  ; // Toggle between DATA0/DATA1 packets
    bool       active    ; // Transfer is active
    uint8_t   *user_buf  ; // User buffer in DPSRAM, RAM, or flash
    uint16_t   bytes_left; // Bytes left to transfer
    uint16_t   bytes_done; // Bytes done transferring
} endpoint_t;

static endpoint_t eps[MAX_ENDPOINTS], *epx = eps;

SDK_INLINE const char *transfer_type(uint8_t bits) {
    switch (bits & USB_TRANSFER_TYPE_BITS) {
        case USB_TRANSFER_TYPE_CONTROL:     return "Control"    ; break;
        case USB_TRANSFER_TYPE_ISOCHRONOUS: return "Isochronous"; break;
        case USB_TRANSFER_TYPE_BULK:        return "Bulk"       ; break;
        case USB_TRANSFER_TYPE_INTERRUPT:   return "Interrupt"  ; break;
        default:                            return "Unknown"    ; break;
    }
}

SDK_INLINE const char *ep_dir(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN ? "IN" : "OUT";
}

SDK_INLINE bool ep_in(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN;
}

SDK_INLINE uint8_t ep_num(endpoint_t *ep) {
    return ep->ep_addr & 0xf;
}

SDK_INLINE void show_endpoint(endpoint_t *ep) {
    printf(" │ %-3uEP%-2d%3s │\n", ep->dev_addr, ep_num(ep), ep_dir(ep));
}

void setup_endpoint(endpoint_t *ep, usb_endpoint_descriptor_t *usb, uint8_t *user_buf) {
    if (!user_buf) panic("Endpoints require a valid buffer");

    // Populate the endpoint (clears all fields not present)
    *ep = (endpoint_t) {
        .dev_addr = ep->dev_addr,
        .ep_addr  = usb->bEndpointAddress,
        .type     = usb->bmAttributes,
        .interval = usb->bInterval,
        .maxsize  = usb->wMaxPacketSize,
        .user_buf = user_buf,
    };

    // USB 2.0 max packet size is 64 bytes, but isochronous can be up to 1,023!
    if (ep->maxsize > 64) panic("Packet size is currently limited to 64 bytes");

    if (!ep_num(ep)) { // Use the shared EPX
        if (ep->type) panic("EP0 must use a control transfer type");

        // Setup shared EPX endpoint and enable double buffering
        ep->ecr = &usbh_dpram->epx_ctrl;
        ep->bcr = &usbh_dpram->epx_buf_ctrl;
        ep->buf = &usbh_dpram->epx_data[0];
       *ep->ecr = EP_CTRL_ENABLE_BITS         // Enable endpoint
                | DOUBLE_BUFFER               // Interrupt per double buffer
                | ep->type << 26              // Set transfer type
                | (uint32_t) ep->buf & 0xfff; // Offset from DSPRAM

    } else { // Use a polled hardware endpoint
        if (!ep->type) panic("Control endpoints cannot be polled");

        // Locate an unused polled hardware endpoint
        uint8_t hwep;
        for (hwep = 1; hwep < MAX_ENDPOINTS; hwep++) {
            if (!usbh_dpram->int_ep_ctrl[hwep - 1].ctrl) break;
        }
        if (hwep == MAX_ENDPOINTS) panic("No free polled endpoints remaining");

        // Setup polled hardware endpoint (these only support single buffering)
        ep->ecr = &usbh_dpram->int_ep_ctrl       [hwep - 1].ctrl;
        ep->bcr = &usbh_dpram->int_ep_buffer_ctrl[hwep - 1].ctrl;
        ep->buf = &usbh_dpram->epx_data         [(hwep + 1) * 64];

        bool     ls  = false;              // TODO: Set to true if low-speed
        bool     in  = ep_in(ep);          // IN direction?
        uint32_t dar = (!ls ? 0 : 1 << 26) // Preamble: LS on a FS hub
                     | ( in ? 0 : 1 << 25) // Direction: In=0, Out=1
                     | (ep_num(ep)  << 16) // Endpoint address (4 bits)
                     |  ep->dev_addr;      // Device address (7 bits)

        // Set the device and endpoint address and enable polling
        usb_hw    ->int_ep_addr_ctrl[hwep - 1] = dar;
       *ep->ecr = EP_CTRL_ENABLE_BITS             // Enable endpoint
                | SINGLE_BUFFER                   // Interrupt per single buffer
                | ep->type << 26                  // Set transfer type
                | ((ep->interval || 1) - 1) << 16 // Polling interval - 1 ms
                | (uint32_t) ep->buf & 0xfff;     // Offset from DSPRAM
       *ep->bcr = 0;
        usb_hw_set->int_ep_ctrl = 1 << hwep;
    }

    // Set as configured
    ep->configured = true;
}

endpoint_t *get_endpoint(uint8_t dev_addr, uint8_t ep_addr) {
    if (!(ep_addr & 0xf)) return epx; // All EP0s use one shared EPX
    for (uint8_t hwep = 1; hwep < MAX_ENDPOINTS; hwep++) {
        endpoint_t *ep = &eps[hwep];
        if (ep->configured) {
            if (ep->dev_addr == dev_addr && ep_num(ep) == ep_addr)
                return ep;
        }
    }
    panic("Endpoint 0x%02x for device %u does not exist", ep_addr, dev_addr);
    return NULL;
}

endpoint_t *next_endpoint(uint8_t dev_addr, usb_endpoint_descriptor_t *usb,
                          uint8_t *user_buf) {
    if (!(usb->bEndpointAddress & 0xf)) panic("EP0 cannot be requested");
    for (uint8_t hwep = 1; hwep < MAX_ENDPOINTS; hwep++) {
        endpoint_t *ep = &eps[hwep];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, usb, user_buf);
            return ep;
        }
    }
    panic("No free endpoints remaining");
    return NULL;
}

SDK_INLINE void clear_endpoint(endpoint_t *ep) {
    ep->setup      = false;
    ep->data_pid   = 0;
    ep->active     = false;
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

// TODO: Do we need a reset_endpoint(endpoint_t *ep)?

void reset_epx() {
    setup_endpoint(epx, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 0, // TODO: Is this ok?
        .bInterval        = 0,
    }), ctrl_buf);
}

// Clear all endpoints
void reset_endpoints() {
    memclr(eps, sizeof(eps));
    reset_epx();
}

// ==[ Buffers ]================================================================

enum { // Used to mask availability in the BCR
    UNAVAILABLE = ~(USB_BUF_CTRL_AVAIL << 16 | USB_BUF_CTRL_AVAIL)
};

// Read an inbound data buffer from an endpoint and return its length
uint16_t read_buffer(endpoint_t *ep, uint8_t buf_id, uint32_t bcr) {
    bool     in   = ep_in(ep);                   // Buffer is inbound
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Buffer is full (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // IN: Copy inbound data from the endpoint to the user buffer
    if (in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        uint8_t *src = (uint8_t *) (ep->buf + buf_id * 64);
        memcpy(ptr, src, len);
        hexdump(buf_id ? "│IN/2" : "│IN/1", ptr, len, 1); // ~7.5 ms
        ep->bytes_done += len;
    }

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize)
        ep->bytes_left = 0;

    return len;
}

// Prepare an outbound data buffer for an endpoint and return its half of BCR
uint16_t prep_buffer(endpoint_t *ep, uint8_t buf_id) {
    bool     in  = ep_in(ep);                         // Buffer is inbound
    bool     mas = ep->bytes_left > ep->maxsize;      // Any more packets?

    // FIXME: This is goofy! Pretends like there is more data to prevent LAST
    if (in && ep_num(ep) && !ep->bytes_left && !ep->bytes_done) {
        mas = 1;
    }

    // Calculate BCR
    uint8_t  pid = ep->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(ep->bytes_left, ep->maxsize);  // Buffer length
    uint16_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (mas ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 |            USB_BUF_CTRL_AVAIL      // Buffer available now
                 | len;                               // Length of next buffer

    // Toggle DATA0/DATA1 pid
    ep->data_pid = pid ^ 1u;

    // OUT: Copy outbound data from the user buffer to the endpoint
    if (!in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        uint8_t *dst = (uint8_t *) (ep->buf + buf_id * 64);
        memcpy(dst, ptr, len);
        hexdump(buf_id ? "│OUT/2" : "│OUT/1", ptr, len, 1);
        ep->bytes_done += len;
    }

    // Update byte counts
    ep->bytes_left -= len;

    return bcr;
}

// Transfer one or two buffers (SIE_CTRL_START_TRANS needed for new transfers)
void transfer(endpoint_t *ep) {
    uint32_t ecr = *ep->ecr;
    uint32_t bcr = prep_buffer(ep, 0);

    // Handle double buffering for EPX
    if (!ep_num(ep)) {
        if (bcr & USB_BUF_CTRL_LAST) {
            ecr &= ~DOUBLE_BUFFER; // Disable double-buffering
            ecr |=  SINGLE_BUFFER; // Enable  single-buffering
        } else {
            ecr &= ~SINGLE_BUFFER; // Disable single-buffering
            ecr |=  DOUBLE_BUFFER; // Enable  double-buffering
            bcr |= prep_buffer(ep, 1) << 16;
        }
    }

    // Update ECR and BCR (set BCR first so controller has time to settle)
    *ep->bcr = bcr & UNAVAILABLE;
    *ep->ecr = ecr;
    nop();
    nop();
    *ep->bcr = bcr;
    *ep->bcr = bcr; // Set this twice?
}

// Processes buffers in ISR context
void handle_buffers(endpoint_t *ep) {
    if (!ep->active) show_endpoint(ep), panic("Illegal buffer completion");

    // Read current buffer(s)
    uint32_t ecr = *ep->ecr;                          // ECR is single or double
    uint32_t bcr = *ep->bcr;                          // Buffer control register
    if (ecr & EP_CTRL_DOUBLE_BUFFERED_BITS) {         // When double buffered...
        if (read_buffer(ep, 0, bcr) == ep->maxsize)   // If first buffer is full
            read_buffer(ep, 1, bcr >> 16);            // Then, read second also
    } else {                                          // When single buffered...
        uint32_t bch = usb_hw->buf_cpu_should_handle; // Check CPU handling bits
        if (bch & 1u) bcr >>= 16;                     // Do RP2040-E4 workaround
        read_buffer(ep, 0, bcr);                      // And read the one buffer
    }

    // Continue the transfer
    if (ep->bytes_left) transfer(ep);
}

// ==[ Transfers ]==============================================================

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

// TODO: Clear a stall and toggle data PID back to DATA0
// TODO: Abort a transfer if not yet started and return true on success

void start_transfer(endpoint_t *ep) {
    if (ep->active) panic("Transfer already active on endpoint");
    ep->active = true;

    // Transfer polled hardware endpoints
    if (ep_num(ep)) {
        uint8_t  was = ep->bytes_left;
        uint32_t dar = ep->dev_addr | ep_num(ep) << 16;
        uint32_t ecr = *ep->ecr;
        uint32_t bcr = prep_buffer(ep, 0);
        uint8_t  now = ep->bytes_left;

        // Update ECR and BCR (set BCR first so controller has time to settle)
        *ep->bcr = bcr & UNAVAILABLE;
        *ep->ecr = ecr;
        nop(); // NOTE: Not needed if we have debugging delays anyway...
        nop(); // NOTE: Not needed if we have debugging delays anyway...

        // Debug output
        printf("\n");
        printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
        printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transfer started");
        show_endpoint(ep);
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump("│DAR", dar);
        bindump("│ECR", ecr);
        bindump("│BCR", bcr);
        if (!ep_in(ep) && was > now) {
            uint8_t len = was - now;
            printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            hexdump("│SENT", &ep->user_buf[ep->bytes_done - len], len, 1);
            printf( "└───────┴──────┴──────────────────────────────────────────────────┘\n");
        } else {
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│%s\t│ %-4s │ Device %-28u │            │\n", ep_in(ep) ? "POLL" : "ZLP", ep_dir(ep), ep->dev_addr);
            printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
        }

        // Trigger the actual transfer
        *ep->bcr = bcr;

        return;
    }

    // Calculate flags
    bool ls  = false;
    bool in = ep_in(ep);
    bool ss = ep->setup && !ep->bytes_done; // Start of a SETUP packet

    // Calculate registers
    uint32_t dar = ep->dev_addr | ep_num(ep) << 16;  // Set dev_addr and ep_num
    uint32_t scr = USB_SIE_CTRL_BASE                 // SIE_CTRL defaults
      | (!ls ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS)    // Preamble: LS on a FS hub
      | (!ss ? 0 : USB_SIE_CTRL_SEND_SETUP_BITS)     // Toggle SETUP packet
      | ( in ?     USB_SIE_CTRL_RECEIVE_DATA_BITS    // Receive bit means IN
                 : USB_SIE_CTRL_SEND_DATA_BITS)      // Send bit means OUT
      |            USB_SIE_CTRL_START_TRANS_BITS;    // Start the transfer now

    // Set hardware registers and fill buffers
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    transfer(ep); // ~20 μs

    // Debug output
    printf("\n");
    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transfer started");
    show_endpoint(ep);
    printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    bindump("│SSR", usb_hw->sie_status);
    bindump("│SCR", usb_hw->sie_ctrl);
    printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    bindump("│DAR", usb_hw->dev_addr_ctrl);
    bindump("│ECR", *ep->ecr);
    bindump("│BCR", *ep->bcr);
    if (ep->setup) {
        uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
        printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
        hexdump("│SETUP", packet, sizeof(usb_setup_packet_t), 1);
        printf( "└───────┴──────┴──────────────────────────────────────────────────┘\n");
    } else if (!ep->bytes_left) {
        char *str = in ? "IN" : "OUT";
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        printf( "│ZLP\t│ %-4s │ Device %-28u │            │\n", str, ep->dev_addr);
        printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
    } else {
        printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
    }

    // Trigger the actual transfer
    usb_hw->sie_ctrl = scr;
}

void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Force direction to OUT
    ep->ep_addr &= ~USB_DIR_IN;
    ep->bytes_left = 0;

    // Send the ZLP transfer
    ep->data_pid = 1;
    start_transfer(ep);
}

void control_transfer(device_t *dev, usb_setup_packet_t *setup) {
    if (!epx->configured) panic("Endpoint not configured");
    if ( epx->type)       panic("Not a control endpoint");

    // Copy the setup packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_setup_packet_t));

    epx->dev_addr   = dev->dev_addr;
    epx->ep_addr    = setup->bmRequestType & USB_DIR_IN;
    epx->maxsize    = dev->maxsize0;
    epx->setup      = true;
    epx->data_pid   = 1; // NOTE: rp2040 does SETUP+DATA0+ACK, so DATA1 is next
    epx->user_buf   = ctrl_buf;
    epx->bytes_left = setup->wLength;
    epx->bytes_done = 0;

    // Flip the endpoint direction if there is no data phase
    if (!epx->bytes_left) epx->ep_addr ^= USB_DIR_IN;

    start_transfer(epx);
}

void command(device_t *dev, uint8_t bmRequestType, uint8_t bRequest,
             uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = bmRequestType,
        .bRequest      = bRequest,
        .wValue        = wValue,
        .wIndex        = wIndex,
        .wLength       = wLength,
    }));
}

void bulk_transfer(endpoint_t *ep, uint8_t *ptr, uint16_t len) {
    if (!ep->configured)                     panic("Endpoint not configured");
    if ( ep->type != USB_TRANSFER_TYPE_BULK) panic("Not a bulk endpoint");

    ep->data_pid   = 1;
    ep->user_buf   = ptr;
    ep->bytes_left = len;
    ep->bytes_done = 0;

    start_transfer(ep);
}

void reset_piccolo(device_t *dev) {
    static uint8_t (states[USER_DEVICES]) = { 0 };
    uint8_t state = ++states[dev->dev_addr];

    printf("Piccolo reset step %u\n", state);

    switch (state) {
        case  1: command(dev, 0x40,  0,  0    , 1, 0); break; // reset both
        case  2: command(dev, 0x40,  0,  2    , 1, 0); break; // reset TX
        case  3: command(dev, 0x40,  0,  1    , 1, 0); break; // reset RX
        case  4: command(dev, 0x40,  2,  0    , 1, 0); break; // set flow control (none)
        case  5: command(dev, 0xc0, 10,  0    , 1, 1); break; // get latency
        case  6: command(dev, 0x40,  9, 16    , 1, 0); break; // set latency to 16ms
        case  7: command(dev, 0xc0,  5,  0    , 1, 2); break; // get modem status
        case  8: command(dev, 0x40,  3, 0x4138, 1, 0); break; // 9600 baud (3MHz/312.5)
        case  9: command(dev, 0x40,  1, 0x0303, 1, 0); break; // enable DTR/RTS
        case 10: command(dev, 0x40,  2, 0x1311, 1, 0); break; // set flow control (XON=0x11, XOFF=0x13)
        default:
            states[dev->dev_addr] = 0;
            dev->state = DEVICE_READY;
            printf("Piccolo reset complete\n");
            break;
    }
}

// ==[ Descriptors ]============================================================

SDK_INLINE void get_descriptor(device_t *dev, uint8_t type, uint8_t len) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(type, 0),
        .wIndex        = 0,
        .wLength       = len,
    }));
}

void get_string_descriptor_blocking(device_t *dev, uint8_t index) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_STRING, index),
        .wIndex        = 0,
        .wLength       = MAX_TEMP,
    }));

    do { usb_task(); } while (epx->active); // This transfer...
    do { usb_task(); } while (epx->active); // The ZLP...
}

void load_device_descriptor(void *ptr, device_t *dev) {
    usb_device_descriptor_t *d = (usb_device_descriptor_t *) ptr;

    dev->class        = d->bDeviceClass;
    dev->subclass     = d->bDeviceSubClass;
    dev->protocol     = d->bDeviceProtocol;

    dev->vid          = d->idVendor;
    dev->pid          = d->idProduct;
    dev->version      = d->bcdDevice;
    dev->manufacturer = d->iManufacturer;
    dev->product      = d->iProduct;
    dev->serial       = d->iSerialNumber;
}

void show_device_descriptor(void *ptr) {
    usb_device_descriptor_t *d = (usb_device_descriptor_t *) ptr;

    printf("Connected Device:\n");
    printf("  Total Length: %u\n"    , d->bLength);
    printb("  USB Version:  "        , d->bcdUSB);
    printf("  Device Class: %u\n"    , d->bDeviceClass);
    printf("    Subclass:   %u\n"    , d->bDeviceSubClass);
    printf("    Protocol:   %u\n"    , d->bDeviceProtocol);
    printf("  Packet Size:  %u\n"    , d->bMaxPacketSize0);
    printf("  Vendor Id:    0x%04x\n", d->idVendor);
    printf("  Product Id:   0x%04x\n", d->idProduct);
    printb("  Version:      "        , d->bcdDevice);
    printf("  Manufacturer: [#%u]\n" , d->iManufacturer);
    printf("  Product:      [#%u]\n" , d->iProduct);
    printf("  Serial:       [#%u]\n" , d->iSerialNumber);
    printf("\n");
}

void show_configuration_descriptor(void *ptr) {
    usb_configuration_descriptor_t *d = (usb_configuration_descriptor_t *) ptr;

    printf("Configuration Descriptor:\n");
    printf("  Total Length: %u\n"   , d->wTotalLength);
    printf("  Interfaces:   %u\n"   , d->bNumInterfaces);
    printf("  Config Value: %u\n"   , d->bConfigurationValue);
    printf("  Config Name:  [#%u]\n", d->iConfiguration);
    printf("  Attributes:   ");
    {
        char *sp = d->bmAttributes & 0x40 ? "Self Powered"  : NULL;
        char *rw = d->bmAttributes & 0x20 ? "Remote Wakeup" : NULL;

        if (sp && rw) printf("%s, %s\n", sp, rw);
        else if  (sp) printf("%s\n", sp);
        else if  (rw) printf("%s\n", rw);
        else          printf("None\n");
    }
    printf("  Max Power:    %u mA\n", d->bMaxPower * 2);
    printf("\n");
}

void show_endpoint_descriptor(void *ptr) {
    usb_endpoint_descriptor_t *d = (usb_endpoint_descriptor_t *) ptr;
    uint8_t ep_addr = d->bEndpointAddress;

    printf("Endpoint Descriptor:\n");
    printf("  Length:           %u\n"   , d->bLength);
    printf("  Endpoint Address: 0x%02x" , d->bEndpointAddress);
    printf(" (EP%u/%s)\n", ep_addr & 0xf, ep_addr & USB_DIR_IN ? "IN" : "OUT");
    printf("  Attributes:       0x%02x" , d->bmAttributes);
    printf(" (%s Transfer Type)\n"      , transfer_type(d->bmAttributes));
    printf("  Max Packet Size:  %u\n"   , d->wMaxPacketSize);
    printf("  Interval:         %u\n"   , d->bInterval);
    printf("\n");
}

void show_string_blocking(device_t *dev, uint8_t index) {

    // Request a string and wait for it
    get_string_descriptor_blocking(dev, index);
    uint8_t *ptr = ctrl_buf;

    // Prepare to parse Unicode string
    uint8_t   len =              *ptr / 2 - 1;
    uint16_t *uni = (uint16_t *) (ptr + 2);

    // Convert string from Unicode to UTF-8
    char *utf = (char[MAX_TEMP]) { 0 };
    char *cur = utf;
    while (len--) {
        uint16_t u = *uni++;
        if (u < 0x80) {
            *cur++ = (char)          u;
        } else if (u < 0x800) {
            *cur++ = (char) (0xc0 | (u >>  6));
            *cur++ = (char) (0x80 | (u        & 0x3f));
        } else {
            *cur++ = (char) (0xe0 | (u >> 12));
            *cur++ = (char) (0x80 |((u >>  6) & 0x3f));
            *cur++ = (char) (0x80 | (u        & 0x3f));
        }
    }
    *cur++ = 0;

    printf("[String #%u]: \"%s\"\n", index, utf);
}

// ==[ Classes ]================================================================

void cdch_init() {
    printf("CDC Host Driver Initialized\n");
}

bool cdch_open(uint8_t dev_addr, const usb_interface_descriptor_t *ifd,
               uint16_t len) {
    printf("CDC Host Driver Opened\n");
    return true;
}

bool cdch_config(uint8_t dev_addr, uint8_t itf_num) {
    printf("CDC Host Driver Configured\n");
    return true;
}

bool cdch_cb(uint8_t dev_addr, uint8_t ep_addr, // Ugh... xfer_result_t result,
             uint32_t xferred_bytes) {
    printf("CDC Host Driver Callback\n");
    return true;
}

void cdch_close(uint8_t dev_addr) {
    printf("CDC Host Driver Closed\n");
}

// ==[ Drivers ]================================================================

typedef struct {
  const char *name;
  void (* const init  )(void);
  bool (* const open  )(uint8_t dev_addr, const usb_interface_descriptor_t *ifd,
                        uint16_t len);
  bool (* const config)(uint8_t dev_addr, uint8_t itf_num);
  bool (* const cb    )(uint8_t dev_addr, uint8_t ep_addr, // Ugh... xfer_result_t result,
                        uint32_t xferred_bytes);
  void (* const close )(uint8_t dev_addr);
} driver_t;

static const driver_t drivers[] = {
    {
        .name   = "CDC",
        .init   = cdch_init,
        .open   = cdch_open,
        .config = cdch_config,
        .cb     = cdch_cb,
        .close  = cdch_close,
    }
};

enum {
    DRIVER_COUNT = sizeof(drivers) / sizeof(driver_t)
};

// Determine the length of an interface descriptor by adding up all its elements
static uint16_t interface_len(usb_interface_descriptor_t *ifd,
                              uint8_t ias, uint16_t max) {
    uint8_t  *cur = (uint8_t *) ifd;
    uint16_t  len = 0;

    while (ias--) {
        len += *cur;
        cur += *cur;
        while (len < max) {
            if  (cur[1] == USB_DT_INTERFACE_ASSOCIATION) return len;
            if ((cur[1] == USB_DT_INTERFACE) &&
               ((usb_interface_descriptor_t *) cur)->bAlternateSetting == 0) {
                break;
            }
            len += *cur;
            cur += *cur;
        }
    }

    return len;
}

// Setup drivers for a device from its configuration descriptor
bool setup_drivers(void *ptr, device_t *dev) {
    usb_configuration_descriptor_t   *cfd; // Configuration descriptor
    usb_interface_assoc_descriptor_t *iad; // Interface association descriptor
    usb_interface_descriptor_t       *ifd; // Interface descriptor
    usb_endpoint_descriptor_t        *epd; // Endpoint descriptor

    uint8_t *cur = ptr; // Points to a list of other descriptors
    uint8_t *end = cur + ((usb_configuration_descriptor_t *) cur)->wTotalLength;
    uint8_t  ias = 1; // Number of interface associations

    // Iterate through each descriptor
    for ( ; cur < end ; cur += *cur ) {
        uint8_t type = cur[1];

        // Debug output
        printf("\n");
        hexdump("Detail", cur, *cur, 1);
        printf("\n");

        // Handle each descriptor
        switch (type) {

            // Configuration descriptor
            case USB_DT_CONFIG:
                cfd = (usb_configuration_descriptor_t *) cur;
                show_configuration_descriptor(cfd);
                break;

            // Interface Assocation Descriptor (IAD)
            case USB_DT_INTERFACE_ASSOCIATION:
                iad = (usb_interface_assoc_descriptor_t *) cur;
                ias = iad->bInterfaceCount;
                break;

            // Interface Descriptor
            case USB_DT_INTERFACE:
                ifd = (usb_interface_descriptor_t *) cur;

                // Special case CDC needs two interfaces: CDC Control + CDC Data
                if (ias                     == 1             &&
                    ifd->bInterfaceClass    == USB_CLASS_CDC &&
                    ifd->bInterfaceSubClass == USB_SUBCLASS_CDC_ABSTRACT_CONTROL_MODEL)
                    ias = 2;

                // Ensure there is enough data for an interface descriptor
                if (interface_len(ifd, ias, (uint16_t) (end - cur)) <
                    sizeof(usb_interface_descriptor_t))
                    panic("Interface descriptor is not big enough");

                // Try to find a driver for this interface
                for (uint8_t i = 0; i < DRIVER_COUNT; i++) {
                    // const driver_t *driver = &drivers[i];
                    //
                    // if (driver->open(dev_addr, cur, len)) {
                    //     printf("  %s driver opened\n", driver->name);
                    //
                    //     // Bind each interface association to the driver
                    //     for (uint8_t j = 0; j < ias; j++) {
                    //         uint8_t k = cur->bInterfaceNumber + j;
                    //         dev->itf2drv[k] = i; // TODO: This needs to start with an invalid value
                    //     }
                    //
                    //     // Bind all endpoints to the driver
                    //     endpoint_bind_driver(dev->ep2drv, cur, len, i);
                    //
                    //     break;
                    // }

                    // Complain if we didn't find a matching driver
                    if (i == DRIVER_COUNT - 1) {
                        printf("Interface %u skipped: Class=%u, Subclass=%u, Protocol=%u\n",
                            ifd->bInterfaceNumber,
                            ifd->bInterfaceClass,
                            ifd->bInterfaceSubClass,
                            ifd->bInterfaceProtocol
                        );
                    }
                }
                break;

            // Endpoint descriptor
            case USB_DT_ENDPOINT:
                epd = (usb_endpoint_descriptor_t *) cur;
                show_endpoint_descriptor(epd);
                next_endpoint(dev->dev_addr, epd, REMOVE_THIS);
                break;

            // Unknown descriptor
            default:
                // FIXME: Panic for now, but handle more gracefully later
                panic("Unknown descriptor type: 0x%02x\n", type);
                break;
        }
    }
}

// ==[ Enumeration ]============================================================

enum {
    ENUMERATION_START,
    ENUMERATION_GET_MAXSIZE,
    ENUMERATION_SET_ADDRESS,
    ENUMERATION_GET_DEVICE,
    ENUMERATION_GET_CONFIG_SHORT,
    ENUMERATION_GET_CONFIG_FULL,
    ENUMERATION_SET_CONFIG,
    ENUMERATION_END,
};

void get_device_descriptor(device_t *dev) {
    printf("Get device descriptor\n");

    uint8_t len = sizeof(usb_device_descriptor_t);

    if (!dev->maxsize0)
        len = dev->maxsize0 = 8; // Default per USB 2.0 spec

    get_descriptor(dev, USB_DT_DEVICE, len);
}

void set_device_address(device_t *dev) {
    printf("Set device address to %u\n", dev->dev_addr);

    control_transfer(dev0, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = dev->dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void get_configuration_descriptor(device_t *dev, uint8_t len) {
    printf("Get configuration descriptor\n");

    get_descriptor(dev, USB_DT_CONFIG, len);
}

void set_configuration(device_t *dev, uint16_t cfg) {
    printf("Set configuration to %u\n", cfg);

    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_CONFIGURATION,
        .wValue        = cfg,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void enumerate(void *arg) {
    device_t *dev = (device_t *) arg;
    static uint8_t step, new_addr;

    if (!dev->maxsize0)
        step = ENUMERATION_START;

    switch (step++) {

        case ENUMERATION_START:
            printf("Enumeration started\n");

            printf("Starting GET_MAXSIZE\n");
            get_device_descriptor(dev);
            break;

        case ENUMERATION_GET_MAXSIZE: {
            device_t *dev  = next_device();
            dev->state     = DEVICE_ENUMERATING;
            dev->speed     = dev0->speed;
            dev->maxsize0  = ((usb_device_descriptor_t *) ctrl_buf)
                                 ->bMaxPacketSize0;

            printf("Starting SET_ADDRESS\n");
            new_addr = dev->dev_addr;
            set_device_address(dev);
        }   break;

        case ENUMERATION_SET_ADDRESS: {
            device_t *dev = get_device(new_addr);
            dev->state    = DEVICE_ADDRESSED;
            reset_device(0);

            printf("Starting GET_DEVICE\n");
            get_device_descriptor(dev);
        }   break;

        case ENUMERATION_GET_DEVICE: {
            load_device_descriptor(ctrl_buf, dev);
            show_device_descriptor(ctrl_buf);

            uint8_t len = sizeof(usb_configuration_descriptor_t);
            printf("Starting GET_CONFIG_SHORT (%u bytes)\n", len);
            get_configuration_descriptor(dev, len);
        }   break;

        case ENUMERATION_GET_CONFIG_SHORT: {
            uint16_t size =
                ((usb_configuration_descriptor_t *) ctrl_buf)->wTotalLength;
            if (size > MAX_TEMP) {
                show_configuration_descriptor(ctrl_buf);
                panic("Configuration descriptor too large (%u bytes)", size);
            }

            uint8_t len = (uint8_t) size;
            printf("Starting GET_CONFIG_FULL (%u bytes)\n", len);
            get_configuration_descriptor(dev, len);
        }   break;

        case ENUMERATION_GET_CONFIG_FULL: {
            setup_drivers(ctrl_buf, dev);

            printf("Starting SET_CONFIG\n");
            set_configuration(dev, 1);
        }   break;

        case ENUMERATION_SET_CONFIG:
            show_string_blocking(dev, dev->manufacturer);
            show_string_blocking(dev, dev->product     );
            show_string_blocking(dev, dev->serial      );

            dev->state = DEVICE_ACTIVE;
            printf("Enumeration completed\n");

            // FIXME: Where and how should this be called? A task? A callback?
            printf("\nCalling reset_piccolo\n");
            reset_piccolo(dev);

            break;
    }
}

// ==[ Callbacks ]==============================================================

void poll_ep1_in(void *arg) {
    bulk_transfer((endpoint_t *) &eps[1], (uint8_t *) REMOVE_THIS, (uint16_t) 0);
}

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
        struct {
            uint8_t speed;
        } connect;

        struct {
            endpoint_t *ep;     // TODO: Risky to just sent this pointer?
            uint16_t    len;    // TODO: Should we point to a safe buffer of this length?
            uint8_t     status; // TODO: Are we even using this?
        } transfer;

        struct {
            void (*fn) (void *);
            void *arg;
        } callback;
    };
} task_t;

static uint32_t guid = 1;

static queue_t *queue = &((queue_t) { 0 });

SDK_INLINE const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        case TASK_CALLBACK: return "TASK_CALLBACK";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

SDK_INLINE const char *callback_name(void (*fn) (void *)) {
    if (fn == enumerate   ) return "enumerate";
    if (fn == transfer_zlp) return "transfer_zlp";
    if (fn == poll_ep1_in ) return "poll_ep1_in";
    printf("Calling unknown callback function\n");
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        printf("\n=> %u) New task, %s\n\n", task.guid, task_name(type)); // ~3 ms (sprintf was ~31 μs, ring_printf was 37 μs)
        switch (type) {

            case TASK_CONNECT: {
                static uint64_t last_attempt;

                // For now, ignore rapid device connects
                if (last_attempt && (time_us_64() - last_attempt < 1000000)) {
                    printf("Connections allowed only once every second\n");
                    break;
                }
                last_attempt = time_us_64();

                // Initialize dev0
                reset_device(0);
                dev0->state = DEVICE_ENUMERATING;
                dev0->speed = task.connect.speed;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // Start enumeration
                enumerate(dev0);

            }   break;

            case TASK_TRANSFER: {
                endpoint_t *ep  = task.transfer.ep;
                uint16_t    len = task.transfer.len;

                // Handle the transfer
                device_t *dev = get_device(ep->dev_addr);
                if (len && dev->state < DEVICE_READY) {
                    printf("Calling transfer_zlp\n");
                    transfer_zlp(ep);
                } else if (dev->state < DEVICE_ACTIVE) {
                    printf("Calling enumerate\n");
                    enumerate(dev);
                } else if (dev->state < DEVICE_READY) {
                    printf("Calling reset_piccolo\n");
                    reset_piccolo(dev);

                    // Once we're ready, start polling
                    if (dev->state == DEVICE_READY) {
                        debug_mode = 1;
                        queue_add_blocking(queue, &((task_t) {
                            .type         = TASK_CALLBACK,
                            .guid         = guid++,
                            .callback.fn  = poll_ep1_in,
                            .callback.arg = NULL,
                        }));
                    }

                } else {
                    printf("Transfer completed\n");
                }
           }   break;

            case TASK_CALLBACK: {
                printf("Calling %s\n", callback_name(task.callback.fn));
                task.callback.fn(task.callback.arg);
            }   break;

            default:
                printf("Unknown task queued\n");
                break;
        }
    }
}

// ==[ Interrupts ]=============================================================

// NOTE: This is how the *host* forces a bus reset
// #define USB_SIE_CTRL_RESET_BUS_RESET  _u(0x0)
// #define USB_SIE_CTRL_RESET_BUS_BITS   _u(0x00002000)
// #define USB_SIE_CTRL_RESET_BUS_MSB    _u(13)
// #define USB_SIE_CTRL_RESET_BUS_LSB    _u(13)
// #define USB_SIE_CTRL_RESET_BUS_ACCESS "SC"

// NOTE: Is this the interrupt to *detect* the bus reset? device only? host?
// #define USB_INTR_BUS_RESET_RESET  _u(0x0)
// #define USB_INTR_BUS_RESET_BITS   _u(0x00001000)
// #define USB_INTR_BUS_RESET_MSB    _u(12)
// #define USB_INTR_BUS_RESET_LSB    _u(12)
// #define USB_INTR_BUS_RESET_ACCESS "RO"

SDK_INLINE void printf_interrupts(uint32_t ints) {
    if (ints & USB_INTS_HOST_CONN_DIS_BITS   ) printf(", device"  );
    if (ints & USB_INTS_STALL_BITS           ) printf(", stall"   );
    if (ints & USB_INTS_BUFF_STATUS_BITS     ) printf(", buffer"  );
    if (ints & USB_INTS_BUFF_STATUS_BITS &&
        usb_hw->buf_status & ~1u             ) printf(", bulk"    );
    if (ints & USB_INTS_TRANS_COMPLETE_BITS  ) printf(", last"    );
    if (ints & USB_INTS_ERROR_RX_TIMEOUT_BITS) printf(", timeout" );
    if (ints & USB_INTS_ERROR_DATA_SEQ_BITS  ) printf(", dataseq" );
    if (ints & USB_INTS_HOST_RESUME_BITS     ) printf(", power"   );
}

// Interrupt handler
void isr_usbctrl() {
    uint32_t ints = usb_hw->ints; // Interrupt bits after masking and forcing

    // ==[ EPX related registers and variables ]==

    uint32_t dar  = usb_hw->dev_addr_ctrl;              // dev_addr/ep_num
    uint32_t ecr  = usbh_dpram->epx_ctrl;               // Endpoint control
    uint32_t bcr  = usbh_dpram->epx_buf_ctrl;           // Buffer control
    bool     dub  = ecr & EP_CTRL_DOUBLE_BUFFERED_BITS; // EPX double buffered

    // Fix RP2040-E4 by shifting buffer control registers for affected buffers
    if (!dub && (usb_hw->buf_cpu_should_handle & 1u)) bcr >>= 16;

    // Get device address and endpoint information
    uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;     // 7 bits (lowest)
    uint8_t ep_addr  = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >> // 4 bits (higher)
                              USB_ADDR_ENDP_ENDPOINT_LSB;
    endpoint_t *ep = get_endpoint(dev_addr, ep_addr);

    // Show system state
    printf( "\n=> %u) New ISR", guid++);
    printf_interrupts(ints);
    printf( "\n\n");
    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Interrupt Handler");
    show_endpoint(ep);
    printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    bindump("│INT", ints);
    bindump("│SSR", usb_hw->sie_status);
    bindump("│SCR", usb_hw->sie_ctrl);
    printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    bindump("│DAR", usb_hw->dev_addr_ctrl);
    bindump("│ECR", ecr);
    bindump("│BCR", bcr);
    bool flat = false; // For the last line of debug output

    // Connection (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed
        uint8_t speed = (usb_hw->sie_status & USB_SIE_STATUS_SPEED_BITS)
                                           >> USB_SIE_STATUS_SPEED_LSB;

        // Clear the interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;

        // Handle connect and disconnect
        if (speed) {

            // Show connection info
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│CONNECT│ %-4s │ %-35s │ Task #%-4u │\n", "", "New device connected", guid);

            queue_add_blocking(queue, &((task_t) { // ~20 μs
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
            }));
        } else {

            // Show disconnection
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│DISCONN│ %-4s │ %-35s │            │\n", "", "Device disconnected");

            reset_device(0);
            reset_epx(); // TODO: There's more to do here
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        printf("Stall detected\n");

        // // Queue the stalled transfer
        // queue_add_blocking(queue, &((task_t) {
        //     .type            = TASK_TRANSFER,
        //     .guid            = guid++,
        //     .transfer.ep     = ep,  // TODO: Need to flesh this out
        //     .transfer.len    = 999, // TODO: Need to flesh this out
        //     .transfer.status = TRANSFER_STALLED,
        // }));
    }

    // Buffer processing is needed
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Remove this soon also...
        bool bulk = usb_hw->buf_status & ~1u;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 1u;

        // Show single/double buffer status of EPX and which buffers are ready
        printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump(dub ? "│BUF/2" : "│BUF/1", bits);

        // Check EPX (bidirectional) and polled hardware endpoints (IN and OUT)
        for (uint8_t hwep = 0; hwep < MAX_ENDPOINTS && bits; hwep++) {
            for (uint8_t dir = 0; dir < 2; dir++) { // IN=evens, OUT=odds
                mask = 1 << (hwep * 2 + dir);
                if (bits &  mask) {
                    bits ^= mask;
                    usb_hw_clear->buf_status = mask;

                    char *str = (char[MAX_TEMP]) { 0 };
                    sprintf(str, "│ECR%u", hwep);
                    bindump(str, *eps[hwep].ecr);
                    sprintf(str, "│BCR%u", hwep);
                    bindump(str, *eps[hwep].bcr);

                    handle_buffers(&eps[hwep]);

                    // FIXME: Go nuclear trying to re-arm
                    *eps[hwep].ecr |=  EP_CTRL_ENABLE_BITS;
                    *eps[hwep].bcr &= ~USB_BUF_CTRL_LAST;
                    usb_hw_set->int_ep_ctrl = 1 << hwep;
                }
                if (!hwep) break;
            }
        }

        // Panic if we missed any buffers
        if (bits) panic("Unhandled buffer mask: %032b", bits);

        if (bulk) {
            eps[1].active = false; // HACK HACK HACK...

            queue_add_blocking(queue, &((task_t) {
                .type         = TASK_CALLBACK,
                .guid         = guid++,
                .callback.fn  = poll_ep1_in,
                .callback.arg = NULL,
            }));
        }
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;

        // Panic if the endpoint is not active
        if (!ep->active) panic("Endpoints must be active to be completed");

        // Get the transfer length (actual bytes transferred)
        uint16_t len = ep->bytes_done;

        // Debug output
        if (len) {
            printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            printf( "│XFER\t│ %4u │ Device %-28u   Task #%-4u │\n", len, ep->dev_addr, guid);
            hexdump("│Data", ep->user_buf, len, 1);
            flat = true;
        } else {
            printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│ZLP\t│ %-4s │ Device %-28u │ Task #%-4u │\n", ep_dir(ep), ep->dev_addr, guid);
        }

        // Clear the endpoint (since its complete)
        clear_endpoint(ep);

        // Queue the transfer task
        queue_add_blocking(queue, &((task_t) {
            .type            = TASK_TRANSFER,
            .guid            = guid++,
            .transfer.ep     = ep,
            .transfer.len    = len,
            .transfer.status = TRANSFER_SUCCESS, // TODO: Is this needed?
        }));
    }

    // Receive timeout (waited too long without seeing an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        printf("Receive timeout\n");

        panic("Timed out waiting for data");
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &  USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints ^= USB_INTS_ERROR_DATA_SEQ_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;

        panic("Data sequence error");
    }

    // Device resumed (device initiated)
    if (ints &  USB_INTS_HOST_RESUME_BITS) {
        ints ^= USB_INTS_HOST_RESUME_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;

        printf("Device initiated resume\n");
    }

    // Were any interrupts missed?
    if (ints) panic("Unhandled IRQ bitmask 0x%04x", ints);

    // TODO: How should we deal with NAKs seen in the SSR?
    // usb_hw_clear->sie_status = 1 << 28u; // Clear the NAK???

    printf("└───────┴──────┴─────────────────────────────────────%s────────────┘\n", flat ? "─" : "┴");
}

// ==[ Setup USB Host ]=========================================================

void setup_usb_host() {
    printf("USB host reset\n\n");

    // Reset controller
    reset_block       (RESETS_RESET_USBCTRL_BITS);
    unreset_block_wait(RESETS_RESET_USBCTRL_BITS);

    // Clear state
    memset(usb_hw    , 0, sizeof(*usb_hw    ));
    memset(usbh_dpram, 0, sizeof(*usbh_dpram));

    // Configure USB host controller
    usb_hw->muxing    = USB_USB_MUXING_TO_PHY_BITS       // Connect to USB Phy
                      | USB_USB_MUXING_SOFTCON_BITS;     // Soft connect
    usb_hw->pwr       = USB_USB_PWR_VBUS_DETECT_BITS     // Enable VBUS detect
                      | USB_USB_PWR_VBUS_DETECT_OVERRIDE_EN_BITS;
    usb_hw->main_ctrl = USB_MAIN_CTRL_CONTROLLER_EN_BITS // Enable controller
                      | USB_MAIN_CTRL_HOST_NDEVICE_BITS; // Enable USB Host
    usb_hw->sie_ctrl  = USB_SIE_CTRL_BASE;               // SIE_CTRL defaults
    usb_hw->inte      = USB_INTE_HOST_CONN_DIS_BITS      // Connect/disconnect
                      | USB_INTE_STALL_BITS              // Stall detected
                      | USB_INTE_BUFF_STATUS_BITS        // Buffer ready
                      | USB_INTE_TRANS_COMPLETE_BITS     // Transfer complete
                      | USB_INTE_HOST_RESUME_BITS        // Device wakes host
                      | USB_INTE_ERROR_DATA_SEQ_BITS     // DATA0/DATA1 wrong
                      | USB_INTE_ERROR_RX_TIMEOUT_BITS   // Receive timeout
                      | (0xffffffff ^ 0x00000004);       // NOTE: Debug all on

    irq_set_enabled(USBCTRL_IRQ, true);

    printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    reset_devices();
    reset_endpoints();
    bindump("│INT", usb_hw->inte);
    printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
}

// ==[ Main ]===================================================================

int main() {
    stdout_uart_init();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    queue_init(queue, sizeof(task_t), 64);
    setup_usb_host();

    debug_mode = 1;

    while (1) {
        usb_task();
    }
}
