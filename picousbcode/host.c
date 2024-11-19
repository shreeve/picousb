// =============================================================================
// PicoUSB - A USB host and device library for the rp2040 (Raspberry Pi Pico/W)
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 29, 2024
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
#include "types.h"
#include "chip/ch34x.h"           // CH34x USB to serial chip

// line coding:
// baud rate: 9600
// stop bits: 1
// parity: none
// data bits: 8

#define CFG_NUM_CDC_LINE_CODING_ON_ENUM_CH34X { 9600, 1, 0, 8 }

#include "chip/ch34x.h"

// ==[ PicoUSB ]================================================================

// User defined limits
#define USER_HUBS      0
#define USER_DEVICES   12 // Not including dev0
#define USER_ENDPOINTS 10 // Not including any EP0s

#define CFG_NUM_CDC 1
#define CFG_NUM_CDC_CH34X 1


#define USB_ENUMERATION_BUFFER_SIZE 256

enum {
    MAX_DEVICES   =   1 + USER_DEVICES,
    MAX_ENDPOINTS =   1 + USER_DEVICES + USER_ENDPOINTS,
    MAX_POLLED    =  15, // Maximum polled endpoints
    MAX_TEMP      = 255, // Must be 255 or less
};



// TODO: Add debug levels: 0=none, 1=errors, 2=info, 3=debug
// const char *box = "┌─┬┐"  // ╔═╦╗ // ┏━┳┓ // ╭─┬╮ // 0 1 2 3
//                   "│ ││"  // ║ ║║ // ┃ ┃┃ // │ ││ // 4 5 6 7
//                   "├─┼┤"  // ╠═╬╣ // ┣━╋┫ // ├─┼┤ // 8 9 a b
//                   "└─┴┘"; // ╚═╩╝ // ┗━┻┛ // ╰─┴╯ // c d e f

static uint8_t ctrl_buf[MAX_TEMP]; // Buffer for control transfers (shared)
static uint8_t temp_buf[MAX_TEMP]; // TODO: Where is this needed???
static cdch_interface_t cdch_data[CFG_NUM_CDC];





// == [ Drivers ] =============================================================

typedef struct {
  uint16_t const (*vid_pid_list)[2];
  uint16_t const vid_pid_count;
  bool (*const open)(uint8_t daddr, const usb_interface_descriptor_t *itf_desc, uint16_t max_len);
  void (*const process_set_config)(usb_transfer_t* xfer);
  bool (*const set_control_line_state)(cdch_interface_t* p_cdc, uint16_t line_state, usb_transfer_cb_t complete_cb, uintptr_t user_data);
  bool (*const set_baudrate)(cdch_interface_t* p_cdc, uint32_t baudrate, usb_transfer_cb_t complete_cb, uintptr_t user_data);
  bool (*const set_data_format)(cdch_interface_t* p_cdc, uint8_t stop_bits, uint8_t parity, uint8_t data_bits, usb_transfer_cb_t complete_cb, uintptr_t user_data);
  bool (*const set_line_coding)(cdch_interface_t* p_cdc, usb_cdc_line_coding_t const* line_coding, usb_transfer_cb_t complete_cb, uintptr_t user_data);
} cdch_serial_driver_t;

static uint16_t const ch34x_vid_pid_list[][2] = {CFG_TUH_CDC_CH34X_VID_PID_LIST};

static bool ch34x_open(uint8_t daddr, usb_interface_descriptor_t const* itf_desc, uint16_t max_len);
static void ch34x_process_config(usb_transfer_t* xfer);

static bool ch34x_set_baudrate(cdch_interface_t* p_cdc, uint32_t baudrate, usb_transfer_cb_t complete_cb, uintptr_t user_data);
static bool ch34x_set_data_format(cdch_interface_t* p_cdc, uint8_t stop_bits, uint8_t parity, uint8_t data_bits, usb_transfer_cb_t complete_cb, uintptr_t user_data);
static bool ch34x_set_line_coding(cdch_interface_t* p_cdc, usb_cdc_line_coding_t const* line_coding, usb_transfer_cb_t complete_cb, uintptr_t user_data);
static bool ch34x_set_modem_ctrl(cdch_interface_t* p_cdc, uint16_t line_state, usb_transfer_cb_t complete_cb, uintptr_t user_data);

// Note driver list must be in the same order as SERIAL_DRIVER enum
static const cdch_serial_driver_t serial_drivers[] = {

  {
      .vid_pid_list           = ch34x_vid_pid_list,
      .vid_pid_count          = TU_ARRAY_SIZE(ch34x_vid_pid_list),
      .open                   = ch34x_open,
      .process_set_config     = ch34x_process_config,
      .set_control_line_state = ch34x_set_modem_ctrl,
      .set_baudrate           = ch34x_set_baudrate,
      .set_data_format        = ch34x_set_data_format,
      .set_line_coding        = ch34x_set_line_coding
  },

};

// ==[ Prototypes ]==============================================================


void usb_task();


// drivers
// bool enable_drivers(endpoint_t *ep);

// ==[ Endpoint Functions ]======================================================                                                                                       

// ==[ Endpoints ]==============================================================

static endpoint_t eps[MAX_ENDPOINTS], *epx = eps;


SDK_INLINE const char *ep_dir(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN ? "IN" : "OUT";
}

SDK_INLINE bool ep_in(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN;
}

SDK_INLINE uint8_t ep_num(endpoint_t *ep) {
    return ep->ep_addr & ~USB_DIR_IN;
}

SDK_INLINE void show_endpoint(endpoint_t *ep) {
    printf(" │ %-3uEP%-2d%3s │\n", ep->dev_addr, ep_num(ep), ep_dir(ep));
}

SDK_INLINE void clear_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->data_pid   = 0;

    // Transfer state
    ep->setup      = false;
    ep->user_buf   = temp_buf;
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

// Get a device by its address
SDK_INLINE endpoint_t *get_endpoint(uint8_t ep_addr) {
    if (ep_addr < MAX_ENDPOINTS) return &eps[ep_addr];
    panic("Endpoint %u does not exist", ep_addr);
    return NULL;
}


void setup_endpoint(endpoint_t *ep, usb_endpoint_descriptor_t *usb,
                    uint8_t *user_buf) {

    // Populate the endpoint (clears all fields not present)
    *ep = (endpoint_t) {
        .dev_addr = ep->dev_addr,
        .ep_addr  = usb->bEndpointAddress,
        .type     = usb->bmAttributes,
        .maxsize  = usb->wMaxPacketSize,
        .interval = usb->bInterval,
        .user_buf = user_buf != NULL ? user_buf : temp_buf,
    };

    // Setup the necessary registers and data buffer pointer
    if (ep->interval) {
        if (!ep_num(ep)) panic("EP0 cannot be polled");
        uint8_t most = MIN(USER_ENDPOINTS, MAX_POLLED);
        for (uint8_t i = 0; i < most; i++) {
            if (usbh_dpram->int_ep_ctrl[i].ctrl) continue; // Skip if being used
            ep->ecr = &usbh_dpram->int_ep_ctrl       [i].ctrl;
            ep->bcr = &usbh_dpram->int_ep_buffer_ctrl[i].ctrl;
            ep->buf = &usbh_dpram->epx_data[(i + 2) * 64]; // Can't do ISO?
            break;
        }
        if (!ep->ecr) panic("No free polled endpoints remaining");
    } else {
        ep->ecr = &usbh_dpram->epx_ctrl;
        ep->bcr = &usbh_dpram->epx_buf_ctrl;
        ep->buf = &usbh_dpram->epx_data[0];
    }

    // Calculate the ECR
    uint32_t type   = ep->type;
    uint32_t ms     = ep->interval;
    uint32_t lsb    = EP_CTRL_HOST_INTERRUPT_INTERVAL_LSB;
    uint32_t offset = (uint32_t) ep->buf & 0x0fff;        // Offset from DSPRAM
    uint32_t style  = ep->interval                        // Polled endpoint?
                    ? EP_CTRL_INTERRUPT_PER_BUFFER        // Y: Single buffering
                    : EP_CTRL_DOUBLE_BUFFERED_BITS        // N: Double buffering
                    | EP_CTRL_INTERRUPT_PER_DOUBLE_BUFFER;// and an INT per pair
    uint32_t ecr    = EP_CTRL_ENABLE_BITS                 // Enable endpoint
                    | style                               // Set buffering style
                    | type << EP_CTRL_BUFFER_TYPE_LSB     // Set transfer type
                    | (ms ? ms - 1 : 0) << lsb            // Polling time in ms
                    | offset;                             // Data buffer offset

    // Set the ECR and mark this endpoint as configured
   *ep->ecr = ecr;
    ep->configured = true;
}

endpoint_t *find_endpoint(uint8_t dev_addr, uint8_t ep_addr) {
    bool want_ep0 = !(ep_addr & ~USB_DIR_IN);
    if (!dev_addr && want_ep0) return epx;

    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (ep->configured && ep->dev_addr == dev_addr) {
            if (want_ep0 && !(ep->ep_addr & ~USB_DIR_IN)) return ep;
            if (ep->ep_addr == ep_addr) return ep;
        }
    }
    panic("Invalid endpoint 0x%02x for device %u", ep_addr, dev_addr);
    return NULL;
}

endpoint_t *next_endpoint(uint8_t dev_addr, usb_endpoint_descriptor_t *usb,
                          uint8_t *user_buf) {

    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, usb, user_buf);
            return ep;
        }
    }
    panic("No free endpoints remaining");
    return NULL;
}

void reset_epx() {
    setup_endpoint(epx, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 8, // Default per USB 2.0 spec
        .bInterval        = 0,
    }), NULL);
}

// Clear out all endpoints
void reset_endpoints() {
    memclr(eps, sizeof(eps));
    reset_epx();
}

void dump_endpoint(endpoint_t *ep) {
    printf("Endpoint Information:\n");
    printf("---------------------\n");
    printf("Device Address   : %u\n", ep->dev_addr);
    printf("Endpoint Address : %u\n", ep->ep_addr);
    printf("Transfer Type    : %u\n", ep->type);
    switch(ep->type) {
        case USB_TRANSFER_TYPE_CONTROL:
            printf("CONTROL (%u)\n");
            break;
        case USB_TRANSFER_TYPE_ISOCHRONOUS:
            printf("ISOCHRONOUS (%u)\n");
            break;
        case USB_TRANSFER_TYPE_BULK:
            printf("BULK (%u)\n");
            break;
        case USB_TRANSFER_TYPE_INTERRUPT:
            printf("INTERRUPT (%u)\n");
            break;
        default:
            printf("UNKNOWN (%u)\n");
            break;
    }
    printf("Max Packet Size  : %u\n", ep->maxsize);
    printf("Polling Interval : %u ms\n", ep->interval);
    printf("Data PID         : %u\n", ep->data_pid);
    printf("Configured       : %s\n", ep->configured ? "Yes" : "No");
    printf("Active           : %s\n", ep->active ? "Yes" : "No");
    printf("Setup Packet     : %s\n", ep->setup ? "Yes" : "No");

    printf("\nHardware Registers:\n");
    printf("Endpoint Control Register : %p (%04x)\n", (void*)ep->ecr);
    printf("Buffer Control Register   : %p (%04x)\n", (void*)ep->bcr);

    printf("\nShared Application Data:\n");
    printf("User Buffer     : %p\n", (void*)ep->user_buf);
    printf("Bytes Left      : %u\n", ep->bytes_left);
    printf("Bytes Done      : %u\n", ep->bytes_done);
    printf("Callback        : %p\n", (void*)ep->cb);
    printf("\nData Buffer     :\n");

    hexdump("Data Buffer", ep->buf, 256, 1);

    printf("---------------------\n");
}

// ==[ Buffers ]================================================================

enum { // Used to mask availability in the BCR (enum resolves at compile time)
    UNAVAILABLE = ~(USB_BUF_CTRL_AVAIL << 16 | USB_BUF_CTRL_AVAIL)
};

// Read a buffer and return its length
uint16_t read_buffer(endpoint_t *ep, uint8_t buf_id, uint32_t bcr) {
    bool     in   = ep_in(ep);                   // Buffer is inbound
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Buffer is full (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // If we are reading data, copy it from the data buffer to the user buffer
    if (in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        memcpy(ptr, (void *) (ep->buf + buf_id * 64), len);
        hexdump(buf_id ? "│IN/2" : "│IN/1", ptr, len, 1); // ~7.5 ms
        ep->bytes_done += len;
    }

    // // Update byte counts
    // ep->bytes_left -= len;

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize) {
        ep->bytes_left = 0;
    }

    return len;
}

// Prepare a buffer and return its half of the BCR
uint16_t prep_buffer(endpoint_t *ep, uint8_t buf_id) {
    bool     in  = ep_in(ep);                         // Buffer is inbound
    bool     mas = ep->bytes_left > ep->maxsize;      // Any more packets?
    uint8_t  pid = ep->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(ep->maxsize, ep->bytes_left);  // Buffer length
    uint16_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (mas ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 |            USB_BUF_CTRL_AVAIL      // Buffer available now
                 | len;                               // Length of next buffer

    // Toggle DATA0/DATA1 pid
    ep->data_pid = pid ^ 1u;

    // If we are sending data, copy it from the user buffer to the data buffer
    if (!in && len) {
        uint8_t *ptr = &ep->user_buf[ep->bytes_done];
        memcpy((void *) (ep->buf + buf_id * 64), ptr, len);
        hexdump(buf_id ? "│OUT/2" : "│OUT/1", ptr, len, 1);
        ep->bytes_done += len;
    }

    // Update byte counts
    ep->bytes_left -= len;

    return bcr;
}

// Send buffer(s) immediately for active transfers, new ones still need SIE help
void send_buffers(endpoint_t *ep) {
    uint32_t ecr = *ep->ecr;
    uint32_t bcr = prep_buffer(ep, 0);

    // Set ECR and BCR based on whether the transfer should be double buffered
    if (~bcr & USB_BUF_CTRL_LAST) {
        ecr |= EP_CTRL_DOUBLE_BUFFERED_BITS;
        bcr |= prep_buffer(ep, 1) << 16;
    } else {
        ecr &= ~EP_CTRL_DOUBLE_BUFFERED_BITS;
    }

    // Update ECR and BCR (set BCR first so controller has time to settle)
    *ep->bcr = bcr & UNAVAILABLE;
    *ep->ecr = ecr;
    nop();
    nop();
    *ep->bcr = bcr;
}

// Processes buffers in ISR context
void handle_buffers(endpoint_t *ep) {
    if (!ep->active) show_endpoint(ep), panic("Halted");

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

    // Send next buffer(s)
    if (ep->bytes_left) send_buffers(ep);
}

// ==[ Devices ]================================================================



static device_t devices[MAX_DEVICES], *dev0 = devices;

// Get a device by its address
SDK_INLINE device_t *get_device(uint8_t dev_addr) {
    if (dev_addr < MAX_DEVICES) return &devices[dev_addr];
    panic("Device %u does not exist", dev_addr);
    return NULL;
}

device_t *device_with_vid_pid(uint16_t vid, uint16_t pid) {
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        device_t *dev = &devices[i];
        if (dev->vid == vid && dev->pid == pid) return dev;
    }
    return NULL;
}

// Find the next device address
uint8_t next_dev_addr() {
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        if (devices[i].state == DEVICE_DISCONNECTED) {
            devices[i].state =  DEVICE_ALLOCATED;
            return i;
        }
    }
    panic("No free devices remaining");
    return 0;
}

// Reset a device
void reset_device(uint8_t dev_addr) {
    device_t *dev = get_device(dev_addr);
    memclr(dev, sizeof(device_t));
    // TODO: Surely, there must be more work to do here?
    // memset(dev->itf2drv, TUSB_INDEX_INVALID_8, sizeof(dev->itf2drv));
    // memset(dev->ep2drv , TUSB_INDEX_INVALID_8, sizeof(dev->ep2drv ));
}

// Clear out all devices
void reset_devices() {
    memclr(devices, sizeof(devices));
}

// Print the the VID, PID, and device state of all devices
void show_devices() {
    printf("\n");
    printf( "┌───────┬──────┬──────────────────────────────────────────────────────┐\n");
    printf( "│Device │ VID  │ PID                                                  │\n");
    printf( "├───────┼──────┼──────────────────────────────────────────────────────┤\n");
    for (uint8_t i = 1; i < MAX_DEVICES; i++) {
        device_t *dev = &devices[i];
        if (dev->state == DEVICE_DISCONNECTED) continue;
        printf( "│ %5u │ %4x │ %4x                                                 │\n",
                i, dev->vid, dev->pid);
    }
    printf( "└───────┴──────┴──────────────────────────────────────────────────────┘\n");
}

// ==[ Transfers ]==============================================================



// TODO: Clear a stall and toggle data PID back to DATA0
// TODO: Abort a transfer if not yet started and return true on success

void transfer(endpoint_t *ep) {
    bool in = ep_in(ep);
    bool su = ep->setup && !ep->bytes_done; // Start of a SETUP packet

    // If there's no data phase, flip the endpoint direction
    if (!ep->bytes_left) {
        in = !in;
        ep->ep_addr ^= USB_DIR_IN;
    }

    // Calculate registers
    uint8_t  lsb = USB_ADDR_ENDP_ENDPOINT_LSB;       // LSB for the ep_num
    uint32_t dar = ep->dev_addr | ep_num(ep) << lsb; // Has dev_addr and ep_num
    uint32_t scr = USB_SIE_CTRL_BASE                 // SIE_CTRL defaults
 //   | (ls  ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS)    // Preamble (LS on FS hub)
      | (!su ? 0 : USB_SIE_CTRL_SEND_SETUP_BITS)     // Toggle SETUP packet
      | (in  ?     USB_SIE_CTRL_RECEIVE_DATA_BITS    // Receive bit means IN
                 : USB_SIE_CTRL_SEND_DATA_BITS)      // Send bit means OUT
      |            USB_SIE_CTRL_START_TRANS_BITS;    // Start the transfer now

    // Debug output
    // if (ep->setup || (*ep->bcr & 0x3f)) {
    //     printf("\n");
    //     printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    //     printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transfer started");
    //     show_endpoint(ep);
    //     printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    //     bindump("│DAR", usb_hw->dev_addr_ctrl);
    //     bindump("│SSR", usb_hw->sie_status);
    //     bindump("│SCR", usb_hw->sie_ctrl);
    //     bindump("│ECR", *ep->ecr);
    //     bindump("│BCR", *ep->bcr);
         if (ep->setup) {
            uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
    //         printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            hexdump("│SETUP", packet, sizeof(usb_urb_control_packet_t), 1);
         }
        //     printf( "└───────┴──────┴──────────────────────────────────────────────────┘\n");
        // } else {
        //     printf( "└───────┴──────┴─────────────────────────────────────┴────────────┘\n");
        // }
    //}

    // Mark the endpoint as active
    ep->active = true;

    // Perform the transfer (also gives SCR some time to settle)
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = scr & ~USB_SIE_CTRL_START_TRANS_BITS;
    send_buffers(ep); // ~20 μs
    usb_hw->sie_ctrl      = scr;
}

void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Send the ZLP transfer
    ep->data_pid = 1;
    transfer(ep);
}

void control_transfer(endpoint_t *ep, usb_urb_control_packet_t *setup) {
    if ( ep_num(ep))     panic("Control transfers must use EP0");
    if (!ep->configured) panic("Endpoint not configured");
    if ( ep->active)     panic("Control transfers per device must be serial");
    if ( ep->type)       panic("Control transfers require a control endpoint");

    //Copy the setup packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_urb_control_packet_t));

    // Send the control transfer
    ep->setup      = true;
    ep->data_pid   = 1;
    ep->ep_addr    = setup->bmRequestType & USB_DIR_IN;
    ep->bytes_left = setup->wLength;
    ep->bytes_done = 0;
    transfer(ep);
}


// ==[ Descriptors ]============================================================

SDK_INLINE void get_descriptor(endpoint_t *ep, uint8_t type, uint8_t len) {
    control_transfer(ep, &((usb_urb_control_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(type, 0),
        .wIndex        = 0,
        .wLength       = len,
    }));
}

void get_string_descriptor_blocking(endpoint_t *ep, uint8_t index) {
    control_transfer(ep, &((usb_urb_control_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_STRING, index),
        .wIndex        = 0,
        .wLength       = MAX_TEMP,
    }));

    do { usb_task(); } while (ep->active); // This transfer...
    do { usb_task(); } while (ep->active); // The ZLP...
}

void show_device_descriptor(void *ptr) {
    usb_device_descriptor_t *d = (usb_device_descriptor_t *) ptr;

    printf("Device Descriptor:\n");
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
        char *sp = d->bmAttributes & 0x40 ? "Self-powered"  : NULL;
        char *rw = d->bmAttributes & 0x20 ? "Remote wakeup" : NULL;

        if (sp && rw) printf("%s, %s\n", sp, rw);
        else if  (sp) printf("%s\n", sp);
        else if  (rw) printf("%s\n", rw);
        else          printf("None\n");
    }
    printf("  Max power:    %umA\n" , d->bMaxPower * 2);
    printf("\n");
}

void show_string_descriptor(usb_string_descriptor_t *d) {
    printf("String Descriptor:\n");
    printf("  bLength: %u\n", d->bLength);
    // printf("  bDescriptorType: %u\n", d->bDescriptorType);
    
    // Calculate the length of the Unicode string (UTF-16)
    uint8_t len = (d->bLength - 2) / 2;
    uint16_t *uni = d->bString;

    // Print the UTF-16 string (hex format)
    // printf("  UTF-16 String: ");
    // for (int i = 0; i < len; i++) {
    //     printf("0x%04x ", uni[i]);
    // }
    // printf("\n");

    // Convert the Unicode string (UTF-16) to a UTF-8 string
    char str[256] = {0}; // Assuming the UTF-8 result will fit within 256 bytes
    char *utf = str;
    while (len--) {
        uint16_t u = *uni++;
        if (u < 0x80) {
            *utf++ = (char) u;
        } else if (u < 0x800) {
            *utf++ = (char) (0xc0 | (u >> 6));
            *utf++ = (char) (0x80 | (u & 0x3f));
        } else {
            *utf++ = (char) (0xe0 | (u >> 12));
            *utf++ = (char) (0x80 | ((u >> 6) & 0x3f));
            *utf++ = (char) (0x80 | (u & 0x3f));
        }
    }
    *utf = '\0'; // Null-terminate the UTF-8 string

    // Print the converted UTF-8 string
    printf("  bString: %s\n", str);
}

void show_interface_descriptor(usb_interface_descriptor_t *d) {
    printf("Interface Descriptor:\n");
    printf("  bLength: %u (0x%x)\n", d->bLength, d->bLength);
    printf("  bDescriptorType: %u (0x%x)\n", d->bDescriptorType, d->bDescriptorType);
    printf("  bInterfaceNumber: %u\n", d->bInterfaceNumber);
    printf("  bAlternateSetting: %u\n", d->bAlternateSetting);
    printf("  bNumEndpoints: %u\n", d->bNumEndpoints);
    printf("  bInterfaceClass: %u (0x%x)\n", d->bInterfaceClass, d->bInterfaceClass);
    printf("  bInterfaceSubClass: %u (0x%x)\n", d->bInterfaceSubClass, d->bInterfaceSubClass);
    printf("  bInterfaceProtocol: %u (0x%x)\n", d->bInterfaceProtocol, d->bInterfaceProtocol);
    printf("  iInterface: %u\n", d->iInterface);
}

void show_endpoint_descriptor(usb_endpoint_descriptor_t *d) {
    printf("Endpoint Descriptor:\n");
    printf("  bLength: %u\n", d->bLength);
    printf("  bDescriptorType: %u (0x%x)\n", d->bDescriptorType, d->bDescriptorType);
    printf("  bEndpointAddress: %u (0x%x)\n", d->bEndpointAddress, d->bEndpointAddress);
    printf("  bmAttributes: %u (0x%x)\n", d->bmAttributes, d->bmAttributes);
    printf("  wMaxPacketSize: %u\n", d->wMaxPacketSize);
    printf("  bInterval: %u\n", d->bInterval);
}


// ==[ Enumeration ]============================================================


void get_device_descriptor(endpoint_t *ep) {
    printf("Get device descriptor\n");

    uint8_t len = ep->dev_addr ? sizeof(usb_device_descriptor_t) : 8;
    get_descriptor(ep, USB_DT_DEVICE, len);
}

void set_device_address(endpoint_t *ep) {
    printf("Set device address to %u\n", ep->dev_addr);

    // TODO: Allow devices to change their address (not just from zero)
    control_transfer(epx, &((usb_urb_control_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = ep->dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void get_configuration_descriptor(endpoint_t *ep, uint8_t len) {
    printf("Get configuration descriptor\n");

    get_descriptor(ep, USB_DT_CONFIG, len);
}

void set_configuration(endpoint_t *ep, uint16_t cfg) {
    printf("Set configuration to %u\n", cfg);

    control_transfer(ep, &((usb_urb_control_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_CONFIGURATION,
        .wValue        = cfg,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

///

static inline cdch_interface_t* get_itf(uint8_t idx) {
  cdch_interface_t* p_cdc = &cdch_data[idx];

  return (p_cdc->daddr != 0) ? p_cdc : NULL;
}

static inline uint8_t get_idx_by_ep_addr(uint8_t daddr, uint8_t ep_addr) {
  for(uint8_t i=0; i<CFG_NUM_CDC; i++) {
    cdch_interface_t* p_cdc = &cdch_data[i];
    if ( (p_cdc->daddr == daddr) &&
         (ep_addr == p_cdc->ep_notif || ep_addr == p_cdc->stream.rx.ep_addr || ep_addr == p_cdc->stream.tx.ep_addr)) {
      return i;
    }
  }

  return TUSB_INDEX_INVALID_8;
}

static cdch_interface_t* make_new_itf(uint8_t daddr, usb_interface_descriptor_t const *itf_desc) {
  for(uint8_t i=0; i<CFG_NUM_CDC; i++) {
    if (cdch_data[i].daddr == 0) {
      cdch_interface_t* p_cdc = &cdch_data[i];
      p_cdc->daddr              = daddr;
      p_cdc->bInterfaceNumber   = itf_desc->bInterfaceNumber;
      p_cdc->bInterfaceSubClass = itf_desc->bInterfaceSubClass;
      p_cdc->bInterfaceProtocol = itf_desc->bInterfaceProtocol;
      p_cdc->line_state         = 0;
      return p_cdc;
    }
  }

  return NULL;
}

static bool open_ep_stream_pair(cdch_interface_t* p_cdc , usb_endpoint_descriptor_t const *desc_ep);
static void set_config_complete(cdch_interface_t * p_cdc, uint8_t idx, uint8_t itf_num);
static void cdch_internal_control_complete(usb_transfer_t* xfer);

///

enum {
  CONFIG_CH34X_READ_VERSION = 0,
  CONFIG_CH34X_SERIAL_INIT,
  CONFIG_CH34X_SPECIAL_REG_WRITE,
  CONFIG_CH34X_FLOW_CONTROL,
  CONFIG_CH34X_MODEM_CONTROL,
  CONFIG_CH34X_COMPLETE
};










static inline cdch_interface_t* get_itf(uint8_t idx) {
  TU_ASSERT(idx < CFG_NUM_CDC, NULL);
  cdch_interface_t* p_cdc = &cdch_data[idx];

  return (p_cdc->daddr != 0) ? p_cdc : NULL;
}

static inline uint8_t get_idx_by_ep_addr(uint8_t daddr, uint8_t ep_addr) {
  for(uint8_t i=0; i<CFG_NUM_CDC; i++) {
    cdch_interface_t* p_cdc = &cdch_data[i];
    if ( (p_cdc->daddr == daddr) &&
         (ep_addr == p_cdc->ep_notif || ep_addr == p_cdc->stream.rx.ep_addr || ep_addr == p_cdc->stream.tx.ep_addr)) {
      return i;
    }
  }

  return TUSB_INDEX_INVALID_8;
}

static bool open_ep_stream_pair(cdch_interface_t* p_cdc , usb_endpoint_descriptor_t const *desc_ep);
static void set_config_complete(cdch_interface_t * p_cdc, uint8_t idx, uint8_t itf_num);
static void cdch_internal_control_complete(usb_transfer_t* xfer);

//--------------------------------------------------------------------+
// APPLICATION API
//--------------------------------------------------------------------+

uint8_t tuh_cdc_itf_get_index(uint8_t daddr, uint8_t itf_num) {
  for (uint8_t i = 0; i < CFG_NUM_CDC; i++) {
    const cdch_interface_t* p_cdc = &cdch_data[i];
    if (p_cdc->daddr == daddr && p_cdc->bInterfaceNumber == itf_num) return i;
  }

  return TUSB_INDEX_INVALID_8;
}

bool tuh_cdc_itf_get_info(uint8_t idx, tuh_itf_info_t* info) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc && info);

  info->daddr = p_cdc->daddr;

  // re-construct descriptor
  usb_interface_descriptor_t* desc = &info->desc;
  desc->bLength            = sizeof(usb_interface_descriptor_t);
  desc->bDescriptorType    = USB_DESCRIPTOR_TYPE_INTERFACE;

  desc->bInterfaceNumber   = p_cdc->bInterfaceNumber;
  desc->bAlternateSetting  = 0;
  desc->bNumEndpoints      = 2u + (p_cdc->ep_notif ? 1u : 0u);
  desc->bInterfaceClass    = USB_CLASS_CDC;
  desc->bInterfaceSubClass = p_cdc->bInterfaceSubClass;
  desc->bInterfaceProtocol = p_cdc->bInterfaceProtocol;
  desc->iInterface         = 0; // not used yet

  return true;
}

bool tuh_cdc_mounted(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);
  return p_cdc->mounted;
}

bool tuh_cdc_get_dtr(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return (p_cdc->line_state & CDC_CONTROL_LINE_STATE_DTR) ? true : false;
}

bool tuh_cdc_get_rts(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return (p_cdc->line_state & CDC_CONTROL_LINE_STATE_RTS) ? true : false;
}

bool tuh_cdc_get_local_line_coding(uint8_t idx, usb_cdc_line_coding_t* line_coding) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  *line_coding = p_cdc->line_coding;

  return true;
}

//--------------------------------------------------------------------+
// Write
//--------------------------------------------------------------------+

uint32_t tuh_cdc_write(uint8_t idx, void const* buffer, uint32_t bufsize) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return tu_edpt_stream_write(p_cdc->daddr, &p_cdc->stream.tx, buffer, bufsize);
}

uint32_t tuh_cdc_write_flush(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return tu_edpt_stream_write_xfer(p_cdc->daddr, &p_cdc->stream.tx);
}

bool tuh_cdc_write_clear(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return tu_edpt_stream_clear(&p_cdc->stream.tx);
}

uint32_t tuh_cdc_write_available(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return tu_edpt_stream_write_available(p_cdc->daddr, &p_cdc->stream.tx);
}

//--------------------------------------------------------------------+
// Read
//--------------------------------------------------------------------+

uint32_t tuh_cdc_read (uint8_t idx, void* buffer, uint32_t bufsize) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc);

  return tu_edpt_stream_read(p_cdc->daddr, &p_cdc->stream.rx, buffer, bufsize);
}

uint32_t tuh_cdc_read_available(uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);

  return tu_edpt_stream_read_available(&p_cdc->stream.rx);
}

bool tuh_cdc_peek(uint8_t idx, uint8_t* ch) {
  cdch_interface_t* p_cdc = get_itf(idx);

  return tu_edpt_stream_peek(&p_cdc->stream.rx, ch);
}

bool tuh_cdc_read_clear (uint8_t idx) {
  cdch_interface_t* p_cdc = get_itf(idx);

  bool ret = tu_edpt_stream_clear(&p_cdc->stream.rx);
  tu_edpt_stream_read_xfer(p_cdc->daddr, &p_cdc->stream.rx);
  return ret;
}

//--------------------------------------------------------------------+
// Control Endpoint API
//--------------------------------------------------------------------+

static void process_internal_control_complete(usb_transfer_t* xfer, uint8_t itf_num) {
  uint8_t idx = tuh_cdc_itf_get_index(xfer->daddr, itf_num);
  cdch_interface_t* p_cdc = get_itf(idx);

  uint16_t const value = tu_le16toh(xfer->setup->wValue);

  if (xfer->result == XFER_RESULT_SUCCESS) {
    switch (p_cdc->serial_drid) {
      case SERIAL_DRIVER_ACM:
        switch (xfer->setup->bRequest) {
          case CDC_REQUEST_SET_CONTROL_LINE_STATE:
            p_cdc->line_state = (uint8_t) value;
            break;

          case CDC_REQUEST_SET_LINE_CODING: {
            uint16_t const len = tu_min16(sizeof(usb_cdc_line_coding_t), tu_le16toh(xfer->setup->wLength));
            memcpy(&p_cdc->line_coding, xfer->buffer, len);
            break;
          }

          default: break;
        }
        break;

      #if CFG_NUM_CDC_FTDI
      case SERIAL_DRIVER_FTDI:
        switch (xfer->setup->bRequest) {
          case FTDI_SIO_MODEM_CTRL:
            p_cdc->line_state = (uint8_t) value;
            break;

          case FTDI_SIO_SET_BAUD_RATE:
            p_cdc->line_coding.bit_rate = p_cdc->requested_line_coding.bit_rate;
            break;

          default: break;
        }
        break;
      #endif

      #if CFG_NUM_CDC_CP210X
      case SERIAL_DRIVER_CP210X:
        switch(xfer->setup->bRequest) {
          case CP210X_SET_MHS:
            p_cdc->line_state = (uint8_t) value;
            break;

          case CP210X_SET_BAUDRATE: {
            uint32_t baudrate;
            memcpy(&baudrate, xfer->buffer, sizeof(uint32_t));
            p_cdc->line_coding.bit_rate = tu_le32toh(baudrate);
            break;
          }

          default: break;
        }
        break;
      #endif

      #if CFG_NUM_CDC_CH34X
      case SERIAL_DRIVER_CH34X:
        switch (xfer->setup->bRequest) {
          case CH34X_REQ_WRITE_REG:
            // register write request
            switch (value) {
              case CH34X_REG16_DIVISOR_PRESCALER:
                // baudrate
                p_cdc->line_coding.dwDTERate = p_cdc->requested_line_coding.dwDTERate;
                break;

              case CH32X_REG16_LCR2_LCR:
                // data format
                p_cdc->line_coding.bCharFormat = p_cdc->requested_line_coding.bCharFormat;
                p_cdc->line_coding.bParityType = p_cdc->requested_line_coding.bParityType;
                p_cdc->line_coding.bDataBits = p_cdc->requested_line_coding.bDataBits;
                break;

              default: break;
            }
            break;

          case CH34X_REQ_MODEM_CTRL: {
            // set modem controls RTS/DTR request. Note: signals are inverted
            uint16_t const modem_signal = ~value;
            if (modem_signal & CH34X_BIT_RTS) {
              p_cdc->line_state |= CDC_CONTROL_LINE_STATE_RTS;
            } else {
              p_cdc->line_state &= (uint8_t) ~CDC_CONTROL_LINE_STATE_RTS;
            }

            if (modem_signal & CH34X_BIT_DTR) {
              p_cdc->line_state |= CDC_CONTROL_LINE_STATE_DTR;
            } else {
              p_cdc->line_state &= (uint8_t) ~CDC_CONTROL_LINE_STATE_DTR;
            }
            break;
          }

          default: break;
        }
        break;
      #endif

      default: break;
    }
  }

  xfer->complete_cb = p_cdc->user_control_cb;
  if (xfer->complete_cb) {
    xfer->complete_cb(xfer);
  }
}

// internal control complete to update state such as line state, encoding
static void cdch_internal_control_complete(usb_transfer_t* xfer) {
  uint8_t const itf_num = (uint8_t) tu_le16toh(xfer->setup->wIndex);
  process_internal_control_complete(xfer, itf_num);
}

bool tuh_cdc_set_control_line_state(uint8_t idx, uint16_t line_state, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc && p_cdc->serial_drid < SERIAL_DRIVER_COUNT);
  cdch_serial_driver_t const* driver = &serial_drivers[p_cdc->serial_drid];

  if (complete_cb) {
    return driver->set_control_line_state(p_cdc, line_state, complete_cb, user_data);
  } else {
    // blocking
    usb_transfer_result_t result = XFER_RESULT_INVALID;
    bool ret = driver->set_control_line_state(p_cdc, line_state, complete_cb, (uintptr_t) &result);

    if (user_data) {
      // user_data is not NULL, return result via user_data
      *((usb_transfer_result_t*) user_data) = result;
    }

    TU_VERIFY(ret && result == XFER_RESULT_SUCCESS);
    p_cdc->line_state = (uint8_t) line_state;
    return true;
  }
}

bool tuh_cdc_set_baudrate(uint8_t idx, uint32_t baudrate, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc && p_cdc->serial_drid < SERIAL_DRIVER_COUNT);
  cdch_serial_driver_t const* driver = &serial_drivers[p_cdc->serial_drid];

  if (complete_cb) {
    return driver->set_baudrate(p_cdc, baudrate, complete_cb, user_data);
  } else {
    // blocking
    usb_transfer_result_t result = XFER_RESULT_INVALID;
    bool ret = driver->set_baudrate(p_cdc, baudrate, complete_cb, (uintptr_t) &result);

    if (user_data) {
      // user_data is not NULL, return result via user_data
      *((usb_transfer_result_t*) user_data) = result;
    }

    TU_VERIFY(ret && result == XFER_RESULT_SUCCESS);
    p_cdc->line_coding.dwDTERate = baudrate;
    return true;
  }
}

bool tuh_cdc_set_data_format(uint8_t idx, uint8_t stop_bits, uint8_t parity, uint8_t data_bits,
                             usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc && p_cdc->serial_drid < SERIAL_DRIVER_COUNT);
  cdch_serial_driver_t const* driver = &serial_drivers[p_cdc->serial_drid];

  if (complete_cb) {
    return driver->set_data_format(p_cdc, stop_bits, parity, data_bits, complete_cb, user_data);
  } else {
    // blocking
    usb_transfer_result_t result = XFER_RESULT_INVALID;
    bool ret = driver->set_data_format(p_cdc, stop_bits, parity, data_bits, complete_cb, (uintptr_t) &result);

    if (user_data) {
      // user_data is not NULL, return result via user_data
      *((usb_transfer_result_t*) user_data) = result;
    }

    TU_VERIFY(ret && result == XFER_RESULT_SUCCESS);
    p_cdc->line_coding.bCharFormat = stop_bits;
    p_cdc->line_coding.bParityType = parity;
    p_cdc->line_coding.bDataBits = data_bits;
    return true;
  }
}

bool tuh_cdc_set_line_coding(uint8_t idx, usb_cdc_line_coding_t const* line_coding, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  cdch_interface_t* p_cdc = get_itf(idx);
  TU_VERIFY(p_cdc && p_cdc->serial_drid < SERIAL_DRIVER_COUNT);
  cdch_serial_driver_t const* driver = &serial_drivers[p_cdc->serial_drid];

  if ( complete_cb ) {
    return driver->set_line_coding(p_cdc, line_coding, complete_cb, user_data);
  } else {
    // blocking
    usb_transfer_result_t result = XFER_RESULT_INVALID;
    bool ret = driver->set_line_coding(p_cdc, line_coding, complete_cb, (uintptr_t) &result);

    if (user_data) {
      // user_data is not NULL, return result via user_data
      *((usb_transfer_result_t*) user_data) = result;
    }

    TU_VERIFY(ret && result == XFER_RESULT_SUCCESS);
    p_cdc->line_coding = *line_coding;
    return true;
  }
}

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+

bool cdch_init(void) {
  TU_LOG_DRV("sizeof(cdch_interface_t) = %u\r\n", sizeof(cdch_interface_t));
  tu_memclr(cdch_data, sizeof(cdch_data));
  for (size_t i = 0; i < CFG_NUM_CDC; i++) {
    cdch_interface_t* p_cdc = &cdch_data[i];
    tu_edpt_stream_init(&p_cdc->stream.tx, true, true, false,
                        p_cdc->stream.tx_ff_buf, CFG_USB_CDC_RX_BUFSIZE,
                        p_cdc->stream.tx_ep_buf, CFG_USB_CDC_TX_EPSIZE);

    tu_edpt_stream_init(&p_cdc->stream.rx, true, false, false,
                        p_cdc->stream.rx_ff_buf, CFG_USB_CDC_RX_BUFSIZE,
                        p_cdc->stream.rx_ep_buf, CFG_USB_CDC_RX_EPSIZE);
  }

  return true;
}

bool cdch_deinit(void) {
  for (size_t i = 0; i < CFG_NUM_CDC; i++) {
    cdch_interface_t* p_cdc = &cdch_data[i];
    tu_edpt_stream_deinit(&p_cdc->stream.tx);
    tu_edpt_stream_deinit(&p_cdc->stream.rx);
  }
  return true;
}

void cdch_close(uint8_t daddr) {
  for (uint8_t idx = 0; idx < CFG_NUM_CDC; idx++) {
    cdch_interface_t* p_cdc = &cdch_data[idx];
    if (p_cdc->daddr == daddr) {
      TU_LOG_DRV("  CDCh close addr = %u index = %u\r\n", daddr, idx);

      // Invoke application callback
      if (tuh_cdc_umount_cb) tuh_cdc_umount_cb(idx);

      p_cdc->daddr = 0;
      p_cdc->bInterfaceNumber = 0;
      p_cdc->mounted = false;
      tu_edpt_stream_close(&p_cdc->stream.tx);
      tu_edpt_stream_close(&p_cdc->stream.rx);
    }
  }
}

bool cdch_xfer_cb(uint8_t daddr, uint8_t ep_addr, usb_transfer_result_t event, uint32_t xferred_bytes) {
  // TODO handle stall response, retry failed transfer ...
  TU_ASSERT(event == XFER_RESULT_SUCCESS);

  uint8_t const idx = get_idx_by_ep_addr(daddr, ep_addr);
  cdch_interface_t * p_cdc = get_itf(idx);
  TU_ASSERT(p_cdc);

  if ( ep_addr == p_cdc->stream.tx.ep_addr ) {
    // invoke tx complete callback to possibly refill tx fifo
    if (tuh_cdc_tx_complete_cb) tuh_cdc_tx_complete_cb(idx);

    if ( 0 == tu_edpt_stream_write_xfer(daddr, &p_cdc->stream.tx) ) {
      // If there is no data left, a ZLP should be sent if:
      // - xferred_bytes is multiple of EP Packet size and not zero
      tu_edpt_stream_write_zlp_if_needed(daddr, &p_cdc->stream.tx, xferred_bytes);
    }
  } else if ( ep_addr == p_cdc->stream.rx.ep_addr ) {
    #if CFG_NUM_CDC_FTDI
    if (p_cdc->serial_drid == SERIAL_DRIVER_FTDI) {
      // FTDI reserve 2 bytes for status
      // uint8_t status[2] = {p_cdc->stream.rx.ep_buf[0], p_cdc->stream.rx.ep_buf[1]};
      tu_edpt_stream_read_xfer_complete_offset(&p_cdc->stream.rx, xferred_bytes, 2);
    }else
    #endif
    {
      tu_edpt_stream_read_xfer_complete(&p_cdc->stream.rx, xferred_bytes);
    }

    // invoke receive callback
    if (tuh_cdc_rx_cb) tuh_cdc_rx_cb(idx);

    // prepare for next transfer if needed
    tu_edpt_stream_read_xfer(daddr, &p_cdc->stream.rx);
  }else if ( ep_addr == p_cdc->ep_notif ) {
    // TODO handle notification endpoint
  }else {
    TU_ASSERT(false);
  }

  return true;
}

//--------------------------------------------------------------------+
// Enumeration
//--------------------------------------------------------------------+

static bool open_ep_stream_pair(cdch_interface_t* p_cdc, usb_endpoint_descriptor_t const* desc_ep) {
  for (size_t i = 0; i < 2; i++) {
    tuh_edpt_open(p_cdc->daddr, desc_ep);

    if (tu_edpt_dir(desc_ep->bEndpointAddress) == USB_DIR_IN) {
      tu_edpt_stream_open(&p_cdc->stream.rx, desc_ep);
    } else {
      tu_edpt_stream_open(&p_cdc->stream.tx, desc_ep);
    }

    desc_ep = (usb_endpoint_descriptor_t const*) tu_desc_next(desc_ep);
  }

  return true;
}

bool cdch_open(uint8_t rhport, uint8_t daddr, usb_interface_descriptor_t const *itf_desc, uint16_t max_len) {
  (void) rhport;

  // For CDC: only support ACM subclass
  // Note: Protocol 0xFF can be RNDIS device
  if (USB_CLASS_CDC                           == itf_desc->bInterfaceClass &&
      USB_SUBCLASS_CDC_ABSTRACT_CONTROL_MODEL == itf_desc->bInterfaceSubClass) {
    return acm_open(daddr, itf_desc, max_len);
  }
  else if (SERIAL_DRIVER_COUNT > 1 &&
           USB_CLASS_VENDOR_SPECIFIC == itf_desc->bInterfaceClass) {
    uint16_t vid, pid;
    TU_VERIFY(tuh_vid_pid_get(daddr, &vid, &pid));

    for (size_t dr = 1; dr < SERIAL_DRIVER_COUNT; dr++) {
      cdch_serial_driver_t const* driver = &serial_drivers[dr];
      for (size_t i = 0; i < driver->vid_pid_count; i++) {
        if (driver->vid_pid_list[i][0] == vid && driver->vid_pid_list[i][1] == pid) {
          return driver->open(daddr, itf_desc, max_len);
        }
      }
    }
  }

  return false;
}

static void set_config_complete(cdch_interface_t * p_cdc, uint8_t idx, uint8_t itf_num) {
  TU_LOG_DRV("CDCh Set Configure complete\r\n");
  p_cdc->mounted = true;
  if (tuh_cdc_mount_cb) tuh_cdc_mount_cb(idx);

  // Prepare for incoming data
  tu_edpt_stream_read_xfer(p_cdc->daddr, &p_cdc->stream.rx);

  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(p_cdc->daddr, itf_num);
}

bool cdch_set_config(uint8_t daddr, uint8_t itf_num) {
  usb_urb_control_packet_t request;
  request.wIndex = tu_htole16((uint16_t) itf_num);

  // fake transfer to kick-off process
  usb_transfer_t xfer;
  xfer.daddr  = daddr;
  xfer.result = XFER_RESULT_SUCCESS;
  xfer.setup  = &request;
  xfer.user_data = 0; // initial state

  uint8_t const idx = tuh_cdc_itf_get_index(daddr, itf_num);
  cdch_interface_t * p_cdc = get_itf(idx);
  TU_ASSERT(p_cdc && p_cdc->serial_drid < SERIAL_DRIVER_COUNT);

  serial_drivers[p_cdc->serial_drid].process_set_config(&xfer);
  return true;
}













static bool ch34x_open(uint8_t daddr, usb_interface_descriptor_t const* itf_desc, uint16_t max_len) {
  // CH34x Interface includes 1 vendor interface + 2 bulk + 1 interrupt endpoints
  if(itf_desc->bNumEndpoints != 3) printf("***WARNING: CH34x Interface should have 3 endpoints, only %u found\r\n", itf_desc->bNumEndpoints);

  uint32_t min_memory = sizeof(usb_interface_descriptor_t) + 3 * sizeof(usb_endpoint_descriptor_t);

  if(min_memory > max_len) {
    printf("CH34x Interface requires %u bytes, only %u available\r\n", min_memory, max_len);
    printf("Size of interface descriptor: %u bytes\r\n", sizeof(usb_interface_descriptor_t));
    printf("Size of each endpoint: %u bytes\r\n", sizeof(usb_endpoint_descriptor_t));
    printf("Number of endpoints: %u bytes\r\n", itf_desc->bNumEndpoints);
    printf("Overhead: %u\r\n", "3 bytes");
    printf("Total: %u bytes\r\n", min_memory);
    printf("Max length: %u bytes\r\n", max_len);
    return false;
}

  cdch_interface_t* p_cdc = make_new_itf(daddr, itf_desc);

  printf ("CH34x opened\r\n");
  p_cdc->serial_drid = SERIAL_DRIVER_CH34X;

  usb_endpoint_descriptor_t const* desc_ep = (usb_endpoint_descriptor_t const*) tu_desc_next(itf_desc);

  // data endpoints expected to be in pairs
  open_ep_stream_pair(p_cdc, desc_ep);
  desc_ep += 2;

  // Interrupt endpoint: not used for now
//   TU_ASSERT(usb_ENDPOINT == tu_desc_descriptor_type(desc_ep) &&
//             TUSB_XFER_INTERRUPT == desc_ep->bmAttributes.xfer);
  tuh_edpt_open(daddr, desc_ep);
  p_cdc->ep_notif = desc_ep->bEndpointAddress;

  return true;
}

static void ch34x_process_config(usb_transfer_t* xfer) {
  // CH34x only has 1 interface and use wIndex as payload and not for bInterfaceNumber
  uint8_t const itf_num = 0;
  uint8_t const idx = tuh_cdc_itf_get_index(xfer->daddr, itf_num);
  cdch_interface_t* p_cdc = get_itf(idx);
  uintptr_t const state = xfer->user_data;
  uint8_t buffer[2]; // TODO remove

  switch (state) {
    case CONFIG_CH34X_READ_VERSION:
      printf("[%u] CDCh CH34x attempt to read Chip Version\r\n", p_cdc->daddr);
      ch34x_control_in(p_cdc, CH34X_REQ_READ_VERSION, 0, 0, buffer, 2, ch34x_process_config, CONFIG_CH34X_SERIAL_INIT);
      break;

    case CONFIG_CH34X_SERIAL_INIT: {
      // handle version read data, set CH34x line coding (incl. baudrate)
      uint8_t const version = xfer->buffer[0];
      printf("[%u] CDCh CH34x Chip Version = %02x\r\n", p_cdc->daddr, version);
      // only versions >= 0x30 are tested, below 0x30 seems having other programming, see drivers from WCH vendor, Linux kernel and FreeBSD

      // init CH34x with line coding
      usb_cdc_line_coding_t const line_coding = CFG_NUM_CDC_LINE_CODING_ON_ENUM_CH34X;
      uint16_t const div_ps = ch34x_get_divisor_prescaler(line_coding.bCharFormat);

      uint8_t const lcr = ch34x_get_lcr(line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

      ch34x_control_out(p_cdc, CH34X_REQ_SERIAL_INIT, TO_U16(lcr, 0x9c), div_ps, ch34x_process_config, CONFIG_CH34X_SPECIAL_REG_WRITE);
      break;
    }

    case CONFIG_CH34X_SPECIAL_REG_WRITE:
      // overtake line coding and do special reg write, purpose unknown, overtaken from WCH driver
      p_cdc->line_coding = ((usb_cdc_line_coding_t) CFG_NUM_CDC_LINE_CODING_ON_ENUM_CH34X);
      ch34x_write_reg(p_cdc, BYTES_TO_U16(CH341_REG_0x0F, CH341_REG_0x2C), 0x0007, ch34x_process_config, CONFIG_CH34X_FLOW_CONTROL);
      break;

    case CONFIG_CH34X_FLOW_CONTROL:
      // no hardware flow control
      ch34x_write_reg(p_cdc, BYTES_TO_U16(CH341_REG_0x27, CH341_REG_0x27), 0x0000, ch34x_process_config, CONFIG_CH34X_MODEM_CONTROL);
      break;

    case CONFIG_CH34X_MODEM_CONTROL:
      // !always! set modem controls RTS/DTR (CH34x has no reset state after CH34X_REQ_SERIAL_INIT)
      ch34x_set_modem_ctrl(p_cdc, CFG_USB_CDC_LINE_CONTROL_ON_ENUM, ch34x_process_config, CONFIG_CH34X_COMPLETE);
      break;

    case CONFIG_CH34X_COMPLETE:
      set_config_complete(p_cdc, idx, itf_num);
      break;

    default:
      break;
  }
}

static uint8_t ch34x_get_lcr(uint8_t stop_bits, uint8_t parity, uint8_t data_bits);
static uint16_t ch34x_get_divisor_prescaler(uint32_t baval);

//------------- control request -------------//

static bool ch34x_set_request(cdch_interface_t* p_cdc, uint8_t direction, uint8_t request, uint16_t value,
                              uint16_t index, uint8_t* buffer, uint16_t length, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
    //   .bmRequestType_bit = {
    //       .recipient = TUSB_REQ_RCPT_DEVICE,
    //       .type      = TUSB_REQ_TYPE_VENDOR,
    //       .direction = direction & 0x01u
    //   },
  usb_urb_control_packet_t const request_setup = {
      .bmRequestType = direction | USB_REQ_TYPE_VENDOR | USB_REQ_TYPE_RECIPIENT_DEVICE,
      .bRequest = request,
      .wValue   = tu_htole16 (value),
      .wIndex   = tu_htole16 (index),
      .wLength  = tu_htole16 (length)
  };

  // use usbh enum buf since application variable does not live long enough
  uint8_t* enum_buf = NULL;

  if (buffer && length > 0) {
    enum_buf = ctrl_buf;
    if (direction == USB_DIR_OUT) {
      //tu_memcpy_s(enum_buf, USB_ENUMERATION_BUFFER_SIZE, buffer, length);
      memcpy(enum_buf, buffer, length);
    }
  }

  usb_transfer_t xfer = {
      .daddr       = p_cdc->daddr,
      .ep_addr     = 0,
      .setup       = &request_setup,
      .buffer      = enum_buf,
      .complete_cb = complete_cb,
      .user_data   = user_data
  };

  return tuh_control_xfer(&xfer);
}

static inline bool ch34x_control_out(cdch_interface_t* p_cdc, uint8_t request, uint16_t value, uint16_t index,
                                     usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  return ch34x_set_request(p_cdc, USB_DIR_OUT, request, value, index, NULL, 0, complete_cb, user_data);
}

static inline bool ch34x_control_in(cdch_interface_t* p_cdc, uint8_t request, uint16_t value, uint16_t index,
                                    uint8_t* buffer, uint16_t buffersize, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  return ch34x_set_request(p_cdc, USB_DIR_IN, request, value, index, buffer, buffersize,
                           complete_cb, user_data);
}

static inline bool ch34x_write_reg(cdch_interface_t* p_cdc, uint16_t reg, uint16_t reg_value, usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  return ch34x_control_out(p_cdc, CH34X_REQ_WRITE_REG, reg, reg_value, complete_cb, user_data);
}

//static bool ch34x_read_reg_request ( cdch_interface_t* p_cdc, uint16_t reg,
//                                     uint8_t *buffer, uint16_t buffersize, usb_transfer_cb_t complete_cb, uintptr_t user_data )
//{
//  return ch34x_control_in ( p_cdc, CH34X_REQ_READ_REG, reg, 0, buffer, buffersize, complete_cb, user_data );
//}

static bool ch34x_write_reg_baudrate(cdch_interface_t* p_cdc, uint32_t baudrate,
                                     usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  uint16_t const div_ps = ch34x_get_divisor_prescaler(baudrate);

  ch34x_write_reg(p_cdc, CH34X_REG16_DIVISOR_PRESCALER, div_ps, complete_cb, user_data);
  return true;
}


// internal control complete to update state such as line state, encoding
static void ch34x_control_complete(usb_transfer_t* xfer) {
  // CH34x only has 1 interface and use wIndex as payload and not for bInterfaceNumber
  process_internal_control_complete(xfer, 0);
}

static bool ch34x_set_data_format(cdch_interface_t* p_cdc, uint8_t stop_bits, uint8_t parity, uint8_t data_bits,
                                usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  p_cdc->requested_line_coding.bCharFormat = stop_bits;
  p_cdc->requested_line_coding.bParityType = parity;
  p_cdc->requested_line_coding.bDataBits = data_bits;

  uint8_t const lcr = ch34x_get_lcr(stop_bits, parity, data_bits);

  ch34x_control_out(p_cdc, CH34X_REQ_WRITE_REG, CH32X_REG16_LCR2_LCR, lcr, complete_cb ? ch34x_control_complete : NULL, user_data);
  return true;
}

static bool ch34x_set_baudrate(cdch_interface_t* p_cdc, uint32_t baudrate,
                               usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  p_cdc->requested_line_coding.dwDTERate = baudrate;
  p_cdc->user_control_cb = complete_cb;
  ch34x_write_reg_baudrate(p_cdc, baudrate, complete_cb ? ch34x_control_complete : NULL, user_data);
  return true;
}

static void ch34x_set_line_coding_stage1_complete(usb_transfer_t* xfer) {
  // CH34x only has 1 interface and use wIndex as payload and not for bInterfaceNumber
  uint8_t const itf_num = 0;
  uint8_t const idx = tuh_cdc_itf_get_index(xfer->daddr, itf_num);
  cdch_interface_t* p_cdc = get_itf(idx);

  if (xfer->result == XFER_RESULT_SUCCESS) {
    // stage 1 success, continue to stage 2
    p_cdc->line_coding.dwDTERate = p_cdc->requested_line_coding.dwDTERate;
    ch34x_set_data_format(p_cdc, p_cdc->requested_line_coding.bCharFormat, p_cdc->requested_line_coding.bParityType,
                                    p_cdc->requested_line_coding.bDataBits, ch34x_control_complete, xfer->user_data);
  } else {
    // stage 1 failed, notify user
    xfer->complete_cb = p_cdc->user_control_cb;
    if (xfer->complete_cb) {
      xfer->complete_cb(xfer);
    }
  }
}

// 2 stages: set baudrate (stage1) + set data format (stage2)
static bool ch34x_set_line_coding(cdch_interface_t* p_cdc, usb_cdc_line_coding_t const* line_coding,
                                  usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  p_cdc->requested_line_coding = *line_coding;
  p_cdc->user_control_cb = complete_cb;

  if (complete_cb) {
    // stage 1 set baudrate
    ch34x_write_reg_baudrate(p_cdc, line_coding->dwDTERate, ch34x_set_line_coding_stage1_complete, user_data);
  } else {
    // sync call
    usb_transfer_result_t result;

    // stage 1 set baudrate
    ch34x_write_reg_baudrate(p_cdc, line_coding->dwDTERate, NULL, (uintptr_t) &result);
    // TU_VERIFY(result == XFER_RESULT_SUCCESS);
    p_cdc->line_coding.dwDTERate = line_coding->dwDTERate;

    // stage 2 set data format
    ch34x_set_data_format(p_cdc, line_coding->bCharFormat, line_coding->bParityType, line_coding->dwDTERate,
                                    NULL, (uintptr_t) &result);

    p_cdc->line_coding.bCharFormat = line_coding->bCharFormat;
    p_cdc->line_coding.bParityType = line_coding->bParityType;
    p_cdc->line_coding.dwDTERate = line_coding->dwDTERate;

    // update transfer result, user_data is expected to point to usb_transfer_result_t
    if (user_data) {
      *((usb_transfer_result_t*) user_data) = result;
    }
  }

  return true;
}

static bool ch34x_set_modem_ctrl(cdch_interface_t* p_cdc, uint16_t line_state,
                                 usb_transfer_cb_t complete_cb, uintptr_t user_data) {
  uint8_t control = 0;
  if (line_state & CDC_CONTROL_LINE_STATE_RTS) {
    control |= CH34X_BIT_RTS;
  }
  if (line_state & CDC_CONTROL_LINE_STATE_DTR) {
    control |= CH34X_BIT_DTR;
  }

  // CH34x signals are inverted
  control = ~control;

  p_cdc->user_control_cb = complete_cb;
  ch34x_control_out(p_cdc, CH34X_REQ_MODEM_CTRL, control, 0, complete_cb ? ch34x_control_complete : NULL, user_data);
  return true;
}


// -- [ Enumeration ] ----------------------------------------------------------

static bool ch34x_open(uint8_t daddr, usb_interface_descriptor_t const* itf_desc, uint16_t max_len) {
  // CH34x Interface includes 1 vendor interface + 2 bulk + 1 interrupt endpoints
  TU_VERIFY (itf_desc->bNumEndpoints == 3);
  TU_VERIFY (sizeof(usb_interface_descriptor_t) + 3 * sizeof(usb_endpoint_descriptor_t) <= max_len);

  cdch_interface_t* p_cdc = make_new_itf(daddr, itf_desc);
  TU_VERIFY (p_cdc);

  printf ("CH34x opened\r\n");
  p_cdc->serial_drid = SERIAL_DRIVER_CH34X;

  usb_endpoint_descriptor_t const* desc_ep = (usb_endpoint_descriptor_t const*) tu_desc_next(itf_desc);

  // data endpoints expected to be in pairs
  open_ep_stream_pair(p_cdc, desc_ep);
  desc_ep += 2;

  // Interrupt endpoint: not used for now
  tuh_edpt_open(daddr, desc_ep);
  p_cdc->ep_notif = desc_ep->bEndpointAddress;

  return true;
}

static void ch34x_process_config(usb_transfer_t* xfer) {
  // CH34x only has 1 interface and use wIndex as payload and not for bInterfaceNumber
  uint8_t const itf_num = 0;
  uint8_t const idx = tuh_cdc_itf_get_index(xfer->daddr, itf_num);
  cdch_interface_t* p_cdc = get_itf(idx);
  uintptr_t const state = xfer->user_data;
  uint8_t buffer[2]; // TODO remove

  switch (state) {
    case CONFIG_CH34X_READ_VERSION:
      printf("[%u] CDCh CH34x attempt to read Chip Version\r\n", p_cdc->daddr);
      ch34x_control_in(p_cdc, CH34X_REQ_READ_VERSION, 0, 0, buffer, 2, ch34x_process_config, CONFIG_CH34X_SERIAL_INIT);
      break;

    case CONFIG_CH34X_SERIAL_INIT: {
      // handle version read data, set CH34x line coding (incl. baudrate)
      uint8_t const version = xfer->buffer[0];
      printf("[%u] CDCh CH34x Chip Version = %02x\r\n", p_cdc->daddr, version);
      // only versions >= 0x30 are tested, below 0x30 seems having other programming, see drivers from WCH vendor, Linux kernel and FreeBSD

      // init CH34x with line coding
      usb_cdc_line_coding_t const line_coding = CFG_NUM_CDC_LINE_CODING_ON_ENUM_CH34X;
      uint16_t const div_ps = ch34x_get_divisor_prescaler(line_coding.dwDTERate);

      uint8_t const lcr = ch34x_get_lcr(line_coding.bCharFormat, line_coding.bParityType, line_coding.bDataBits);

      ch34x_control_out(p_cdc, CH34X_REQ_SERIAL_INIT, BYTES_TO_U16(lcr, 0x9c), div_ps,
                                   ch34x_process_config, CONFIG_CH34X_SPECIAL_REG_WRITE);
      break;
    }

    case CONFIG_CH34X_SPECIAL_REG_WRITE:
      // overtake line coding and do special reg write, purpose unknown, overtaken from WCH driver
      p_cdc->line_coding = ((usb_cdc_line_coding_t) CFG_NUM_CDC_LINE_CODING_ON_ENUM_CH34X);
      ch34x_write_reg(p_cdc, BYTES_TO_U16(CH341_REG_0x0F, CH341_REG_0x2C), 0x0007, ch34x_process_config, CONFIG_CH34X_FLOW_CONTROL);
      break;

    case CONFIG_CH34X_FLOW_CONTROL:
      // no hardware flow control
      ch34x_write_reg(p_cdc, BYTES_TO_U16(CH341_REG_0x27, CH341_REG_0x27), 0x0000, ch34x_process_config, CONFIG_CH34X_MODEM_CONTROL);
      break;

    case CONFIG_CH34X_MODEM_CONTROL:
      // !always! set modem controls RTS/DTR (CH34x has no reset state after CH34X_REQ_SERIAL_INIT)
      ch34x_set_modem_ctrl(p_cdc, CFG_USB_CDC_LINE_CONTROL_ON_ENUM, ch34x_process_config, CONFIG_CH34X_COMPLETE);
      break;

    case CONFIG_CH34X_COMPLETE:
      set_config_complete(p_cdc, idx, itf_num);
      break;

    default:
      break;
  }
}

void enumerate(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Prepare to advance the enumeration
    static uint8_t step;
    static uint8_t new_addr;

    if (!ep) step = ENUMERATION_START;

    // TODO: We need a way to ensure only one device enumerating at a time!

    switch (step++) {

         case ENUMERATION_START:
            printf("Enumeration started\n");


            get_device_descriptor(epx); // TODO: We need to make sure we snag the value right when it comes back
            break;

        case ENUMERATION_GET_MAXSIZE: {
            uint8_t maxsize0 =
                ((usb_device_descriptor_t *) epx->user_buf)->bMaxPacketSize0;

            // Allocate a new device
            new_addr      = next_dev_addr();
            device_t *dev = get_device(new_addr);
            dev->state    = DEVICE_ENUMERATING;
            dev->speed    = dev0->speed;

            // Allocate EP0 on the new device (uses the shared ctrl_buf buffer)
            endpoint_t *ep = next_endpoint(new_addr, &((usb_endpoint_descriptor_t) {
                .bLength          = sizeof(usb_endpoint_descriptor_t),
                .bDescriptorType  = USB_DT_ENDPOINT,
                .bEndpointAddress = 0,
                .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
                .wMaxPacketSize   = maxsize0,
                .bInterval        = 0,
            }), ctrl_buf);
            ep->dev_addr = new_addr;

            printf("Starting SET_ADDRESS\n");
            set_device_address(ep);
        }   break;

        case ENUMERATION_SET_ADDRESS: {
            endpoint_t *ep  = find_endpoint(new_addr, 0);
            device_t   *dev = get_device(ep->dev_addr);

            dev0->state = DEVICE_ALLOCATED;
            dev->state  = DEVICE_ADDRESSED;

            printf("Starting GET_DEVICE\n");
            get_device_descriptor(ep);
        }   break;

        case ENUMERATION_GET_DEVICE: {
            show_device_descriptor(ep->user_buf);
            uint8_t len = sizeof(usb_configuration_descriptor_t);

            printf("Starting GET_CONFIG_SHORT (%u bytes)\n", len);
            get_configuration_descriptor(ep, len);
        }   break;

        case ENUMERATION_GET_CONFIG_SHORT: {
            uint8_t len =
                ((usb_configuration_descriptor_t *) ep->user_buf)->wTotalLength;
            if (len > MAX_TEMP) {
                show_configuration_descriptor(ep->user_buf);
                panic("Configuration descriptor too large");
            }

            printf("Starting GET_CONFIG_FULL (%u bytes)\n", len);
            get_configuration_descriptor(ep, len);
        }   break;

        case ENUMERATION_GET_CONFIG_FULL: {
            show_configuration_descriptor(ep->user_buf);
            //enable_drivers(ep);

            printf("Starting SET_CONFIG\n");
            set_configuration(ep, 1);
        }   break;

        case ENUMERATION_SET_CONFIG:
            device_t *dev = get_device(ep->dev_addr);
            dev->state = DEVICE_ACTIVE;

            printf("Enumeration completed\n");

            show_string_blocking(ep, 1);
            show_string_blocking(ep, 2);
            show_string_blocking(ep, 3);

            break;
        case FTDI_RESET_REQUEST:
            printf("============> #8) Enumeration complete\n");

            // USB_DIR_OUT | USB_REQ_TYPE_TYPE_CLASS | USB_REQ_TYPE_RECIPIENT_INTERFACE,

            // usb_urb_control_packet_t setup_packet = {
            //     .bmRequestType = USB_DIR_OUT | USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_RECIPIENT_DEVICE,
            //     .bRequest      = FTDI_SIO_RESET_REQUEST,
            //     .wValue        = FTDI_SIO_RESET_SIO,
            //     .wIndex        = 0,
            //     .wLength       = 0,
            // };

            //control_transfer(ep, &setup_packet);

            // usb_urb_control_packet_t *setup_pkt;
            // *setup_pkt = (usb_urb_control_packet_t) {
            //     .bmRequestType = USB_DIR_OUT | USB_REQ_TYPE_TYPE_CLASS | USB_REQ_TYPE_RECIPIENT_INTERFACE,  // Host-to-device, Class request, Interface recipient
            //     .bRequest      = 0x22,   // SET_CONTROL_LINE_STATE (0x22)
            //     .wValue        = line_state,  // Line state: DTR and RTS (0x03 for both enabled)
            //     .wIndex        = interface,   // Interface number (usually 0 for the first interface)
            //     .wLength       = 0x0000       // No data in the data phase
            // };

            break;
    }
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

// ==[ Tasks ]==================================================================



static uint32_t guid = 1;

static queue_t *queue = &((queue_t) { 0 });

SDK_INLINE const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CALLBACK: return "TASK_CALLBACK";
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

SDK_INLINE const char *callback_name(void (*fn) (void *)) {
    if (fn == enumerate   ) return "enumerate";
    if (fn == transfer_zlp) return "transfer_zlp";
    printf("Calling unknown callback function\n");
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        printf("\n=> %u) New task, %s\n\n", task.guid, task_name(type)); // ~3 ms (sprintf was ~31 μs, ring_printf was 37 μs)
        switch (type) {

            case TASK_CALLBACK: {
                printf("Calling %s\n", callback_name(task.callback.fn));
                task.callback.fn(task.callback.arg);
            }   break;

            case TASK_CONNECT: {
                static uint64_t last_attempt;

                // For now, ignore rapid device connects
                if (last_attempt && (time_us_64() - last_attempt < 1000000)) {
                    printf("Connections allowed only once every second\n");
                    break;
                }
                last_attempt = time_us_64();

                // Initialize dev0
                reset_device(0); // TODO: Is this really necessary?
                dev0->state = DEVICE_ENUMERATING;
                dev0->speed = task.connect.speed;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // Start enumeration
                enumerate(NULL);

            }   break;

            case TASK_TRANSFER: {
                endpoint_t *ep  = task.transfer.ep;
                uint16_t    len = task.transfer.len;

                // Handle the transfer
                device_t *dev = get_device(ep->dev_addr);
                if (len) {
                    printf("Calling transfer_zlp\n");
                    transfer_zlp(ep);
                } else if (dev->state < DEVICE_ACTIVE) {
                    printf("Calling enumerate\n");
                    enumerate(ep);
                } else {
                    printf("Transfer completed\n");
                }
           }   break;

            default:
                printf("Unknown task queued\n");
                break;
        }
        // printf("=> %u) Finish task: %s\n", task.guid, task_name(type));
    }
}

// ==[ Interrupts ]=============================================================

SDK_INLINE void printf_interrupts(uint32_t ints) {
    if (ints & USB_INTS_HOST_CONN_DIS_BITS   ) printf(", device"  );
    if (ints & USB_INTS_STALL_BITS           ) printf(", stall"   );
    if (ints & USB_INTS_BUFF_STATUS_BITS     ) printf(", buffer"  );
    if (ints & USB_INTS_TRANS_COMPLETE_BITS  ) printf(", last"    );
    if (ints & USB_INTS_ERROR_RX_TIMEOUT_BITS) printf(", timeout" );
    if (ints & USB_INTS_ERROR_DATA_SEQ_BITS  ) printf(", dataseq" );
    if (ints & USB_INTS_HOST_RESUME_BITS     ) printf(", power"   );
}

// Interrupt handler
void isr_usbctrl() {
    task_t task;

    // Load some registers into local variables
    uint32_t ints = usb_hw->ints;
    uint32_t dar  = usb_hw->dev_addr_ctrl;              // dev_addr/ep_num
    uint32_t ecr  = usbh_dpram->epx_ctrl;               // Endpoint control
    uint32_t bcr  = usbh_dpram->epx_buf_ctrl;           // Buffer control
    bool     dub  = ecr & EP_CTRL_DOUBLE_BUFFERED_BITS; // EPX double buffered

    printf("\n----------------\nINTERRUPT\n---------------\n");

    // Fix RP2040-E4 by shifting buffer control registers for affected buffers
    if (!dub && (usb_hw->buf_cpu_should_handle & 1u)) bcr >>= 16; // Fix EPX
    // TODO: Add a similar fix for all polled endpoints

    // Get device address and endpoint information
    uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;
    uint8_t ep_addr  = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >>
                              USB_ADDR_ENDP_ENDPOINT_LSB;
    endpoint_t *ep = find_endpoint(dev_addr, ep_addr);

    // printf("Device Address: %u\n", dev_addr);
    // printf("Endpoint Address: %u\n", ep_addr);
    // printf("Endpoint (lookup): %p\n", ep);
    // dump_endpoint(ep);

    // Show system state
    printf( "\n=> %u) New ISR", guid++);
    // printf_interrupts(ints);
    // printf( "\n\n");
    // printf( "┌───────┬──────┬─────────────────────────────────────┬────────────┐\n");
    // printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Interrupt Handler");
    // show_endpoint(ep);
    // printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
    // bindump("│INTR", usb_hw->intr);
    // bindump("│INTS", ints);
    // bindump("│DAR" , dar);
    // bindump("│SSR" , usb_hw->sie_status);
    // bindump("│SCR" , usb_hw->sie_ctrl);
    // bindump("│ECR" , ecr);
    // bindump("│BCR" , bcr);
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
            // printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│CONNECT│ %-4s │ %-35s │ Task #%-4u │\n", "", "New device connected", guid);

            queue_add_blocking(queue, &((task_t) { // ~20 μs
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
            }));
        } else {
            reset_epx(); // TODO: There's more to do here
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        printf("Stall detected\n");

//         // Queue the stalled transfer
//         queue_add_blocking(queue, &((task_t) {
//             .type            = TASK_TRANSFER,
//             .guid            = guid++,
//             .transfer.ep     = ep,  // TODO: Need to flesh this out
//             .transfer.len    = 999, // TODO: Need to flesh this out
//             .transfer.status = TRANSFER_STALLED,
//         }));
    }

    // Buffer processing is needed
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 1u;

        // Show single/double buffer status of EPX and which buffers are ready
        // printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
        bindump(dub ? "│BUF/2" : "│BUF/1", bits);

        // Lookup the endpoint
        handle_buffers(ep);
        usb_hw_clear->buf_status = 0x1; bits ^= 0x1; // TODO: TOTAL HACK!

//         // Check the polled endpoints (IN and OUT)
//         for (uint8_t i = 0; i < MAX_ENDPOINTS && bits; i++) {
//             for (uint8_t j = 0; j < 2; j++) {
//                 mask = 1 << (i * 2 + j);
//                 if (bits &  mask) {
//                     bits ^= mask;
//                     handle_buffers(&eps[i]);
//                 }
//             }
//         }

        // Panic if we missed any buffers
        if (bits)
            panic("Unhandled buffer mask: %032b", bits);
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
            // printf( "├───────┼──────┼─────────────────────────────────────┴────────────┤\n");
            // printf( "│XFER\t│ %4u │ Device %-28u   Task #%-4u │\n", len, ep->dev_addr, guid);
//            hexdump("│Data", temp_buf, len, 1); // TODO: We should use the base buffer address for this endpoint
            hexdump("Data", ep->user_buf, len, 1);
            flat = true;
        } else {
            char *str = ep_in(ep) ? "IN" : "OUT";
            // printf( "├───────┼──────┼─────────────────────────────────────┼────────────┤\n");
            printf( "│ZLP\t│ %-4s │ Device %-28u │ Task #%-4u │\n", str, ep->dev_addr, guid);
        }


        // hexdump("temp_buf", temp_buf, len, 1);
        // hexdump("ep->user_buf", ep->user_buf, len, 1);

        bool marshal_error = false;
        
        uint8_t *descriptor_offset = ep->user_buf;
        while (descriptor_offset < ep->user_buf + len)
        {

            if(descriptor_offset[0] == 0)
            {
                printf("Malformed data: Descriptor length is 0\n");
                break;
            }

            switch(descriptor_offset[1])
            {

                case USB_DESCRIPTOR_TYPE_DEVICE:
                    marshal_error = sizeof(usb_device_descriptor_t) != descriptor_offset[0];
                    if (marshal_error)
                    {
                        printf("Malformed data: Descriptor type %02x should have a size of %u, got %u\n", descriptor_offset[1], sizeof(usb_device_descriptor_t), descriptor_offset[0]);
                        break;
                    }

                    usb_device_descriptor_t *device_descriptor = (usb_device_descriptor_t *)descriptor_offset;
                    show_device_descriptor(device_descriptor);
                    break;

                case USB_DESCRIPTOR_TYPE_STRING:
                    
                    usb_string_descriptor_t *string_descriptor = (usb_string_descriptor_t *)descriptor_offset;
                    show_string_descriptor(string_descriptor);
                    break;

                case USB_DESCRIPTOR_TYPE_CONFIG:
                    marshal_error = sizeof(usb_configuration_descriptor_t) != descriptor_offset[0];
                    if (marshal_error)
                    {
                        printf("Malformed data: Descriptor type %02x should have a size of %u, got %u\n", descriptor_offset[1], sizeof(usb_configuration_descriptor_t), descriptor_offset[0]);
                        break;
                    }

                    usb_configuration_descriptor_t *config_descriptor = (usb_configuration_descriptor_t *) descriptor_offset;
                    show_configuration_descriptor(config_descriptor);
                    break;

                case USB_DESCRIPTOR_TYPE_INTERFACE:

                    marshal_error = sizeof(usb_interface_descriptor_t) != descriptor_offset[0];
                    if (marshal_error)
                    {
                        printf("Malformed data: Descriptor type %02x should have a size of %u, got %u\n", descriptor_offset[1], sizeof(usb_interface_descriptor_t), descriptor_offset[0]);
                        break;
                    }

                    usb_interface_descriptor_t *interface_descriptor = (usb_interface_descriptor_t *) descriptor_offset;
                    show_interface_descriptor(interface_descriptor);
                    break;

                case USB_DESCRIPTOR_TYPE_ENDPOINT:

                    marshal_error = sizeof(usb_endpoint_descriptor_t) != descriptor_offset[0];
                    if (marshal_error)
                    {
                        printf("Malformed data: Descriptor type %02x should have a size of %u, got %u\n", descriptor_offset[1], sizeof(usb_endpoint_descriptor_t), descriptor_offset[0]);
                        break;
                    }

                    usb_endpoint_descriptor_t *endpoint_descriptor = (usb_endpoint_descriptor_t *) descriptor_offset;
                    show_endpoint_descriptor(endpoint_descriptor);
                    break;

                default:

                    printf("Unknown or unsupported descriptor type: %02x\n", descriptor_offset[1]);
                    break;
            }

            if (marshal_error)
            {
                printf("**ABORT** Can't reliably marshal remaining data\n");
                break;
            }

            descriptor_offset += descriptor_offset[0];

            printf("----------------\n");

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
    // usb_hw_clear->sie_status = 1 << 28u; // Clear the NAK???n

    printf("└───────┴──────┴─────────────────────────────────────%s────────────┘\n", flat ? "─" : "┴");
}

// ==[ Drivers ]================================================================



// ==[ CDC Host ]===============================================================

// CDC Driver State
typedef struct {
    uint8_t bulk_in_ep;
    uint8_t bulk_out_ep;
    uint8_t in_buffer[MAX_CDC_IN_BUFFER_SIZE + 1];  // +1 for null-terminator
    uint8_t out_buffer[MAX_CDC_OUT_BUFFER_SIZE];
} cdch_state_t;

static cdch_state_t cdch_state;

// prototype cdc functions
// CDC Driver Functions




// ==[ Main ]===================================================================

int main() {
    stdout_uart_init();
    printf("\033[2J\033[H\n==[ USB host example]==\n\n");
    setup_usb_host();

    queue_init(queue, sizeof(task_t), 64);

    // while (1) {
    //     usb_task();
    // }


    // Initialize drivers
    // for (int i = 0; i < DRIVER_COUNT; i++) {
    //     drivers[i].init();
    // }

    while (1) {
        usb_task();


    }
}

static uint16_t ch34x_get_divisor_prescaler(uint32_t baval) {
  uint8_t a;
  uint8_t b;
  uint32_t c;

  switch (baval) {
    case 921600:
      a = 0xf3;
      b = 7;
      break;

    case 307200:
      a = 0xd9;
      b = 7;
      break;

    default:
      if (baval > 6000000 / 255) {
        b = 3;
        c = 6000000;
      } else if (baval > 750000 / 255) {
        b = 2;
        c = 750000;
      } else if (baval > 93750 / 255) {
        b = 1;
        c = 93750;
      } else {
        b = 0;
        c = 11719;
      }
      a = (uint8_t) (c / baval);
      if (a == 0 || a == 0xFF) {
        return 0;
      }
      if ((c / a - baval) > (baval - c / (a + 1))) {
        a++;
      }
      a = (uint8_t) (256 - a);
      break;
  }

  // reg divisor = a, reg prescaler = b
  // According to linux code we need to set bit 7 of UCHCOM_REG_BPS_PRE,
  // otherwise the chip will buffer data.
  return (uint16_t) ((uint16_t)a << 8 | 0x80 | b);
}

// calculate lcr value from data coding
static uint8_t ch34x_get_lcr(uint8_t stop_bits, uint8_t parity, uint8_t data_bits) {
  uint8_t lcr = CH34X_LCR_ENABLE_RX | CH34X_LCR_ENABLE_TX;

  lcr |= (uint8_t) (data_bits - 5);

  switch(parity) {
    case CDC_LINE_CODING_PARITY_NONE:
      break;

    case CDC_LINE_CODING_PARITY_ODD:
      lcr |= CH34X_LCR_ENABLE_PAR;
      break;

    case CDC_LINE_CODING_PARITY_EVEN:
      lcr |= CH34X_LCR_ENABLE_PAR | CH34X_LCR_PAR_EVEN;
      break;

    case CDC_LINE_CODING_PARITY_MARK:
      lcr |= CH34X_LCR_ENABLE_PAR | CH34X_LCR_MARK_SPACE;
      break;

    case CDC_LINE_CODING_PARITY_SPACE:
      lcr |= CH34X_LCR_ENABLE_PAR | CH34X_LCR_MARK_SPACE | CH34X_LCR_PAR_EVEN;
      break;

    default: break;
  }

  // 1.5 stop bits not supported
  TU_VERIFY(stop_bits != CDC_LINE_CODING_STOP_BITS_1_5, 0);
  if (stop_bits == CDC_LINE_CODING_STOP_BITS_2) {
    lcr |= CH34X_LCR_STOP_BITS_2;
  }

  return lcr;
}
