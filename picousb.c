#include "picousb.h"

// =============================================================================
// PicoUSB - A smaller than tiny USB Host library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: November 20, 2024
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach for TinyUSB and https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for his https://github.com/Panda381/PicoLibSDK
// =============================================================================

// Globals for task handling
static uint32_t guid = 1;
static queue_t *queue = &((queue_t) { 0 });

// ==[ Hubs ]===================================================================

hub_t hubs[MAX_HUBS], *root = hubs;

// ==[ Devices ]================================================================

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

void clear_device(uint8_t dev_addr) {
    device_t *dev = get_device(dev_addr);
    memclr(dev, sizeof(device_t));
}

void clear_devices() {
    memclr(devices, sizeof(devices));
}

device_t devices[MAX_DEVICES], *dev0 = devices;

// ==[ Endpoints ]==============================================================

static uint8_t ctrl_buf[MAX_CTRL_BUF]; // Shared buffer for control transfers

SDK_INJECT const char *ep_dir(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN ? "IN" : "OUT";
}

SDK_INJECT bool ep_in(endpoint_t *ep) {
    return ep->ep_addr & USB_DIR_IN;
}

SDK_INJECT void show_endpoint(endpoint_t *ep) {
    printf(" │ %-3uEP%-2d%3s │\n", ep->dev_addr, ep->ep_addr & 0xf, ep_dir(ep));
}

void setup_endpoint(endpoint_t *ep, uint8_t epn, usb_endpoint_descriptor_t *usb,
                    uint8_t *user_buf) {

    // Populate the endpoint (clears all fields not present)
    *ep = (endpoint_t) {
        .dev_addr = ep->dev_addr,
        .ep_addr  = usb->bEndpointAddress, // Thus, 0x81 is EP1/IN
        .type     = usb->bmAttributes,
        .interval = usb->bInterval,
        .maxsize  = usb->wMaxPacketSize,
        .user_buf = user_buf,
    };

    // Panic if endpoint is not a control or bulk transfer type
    if (ep->type & 1u) panic("Interrupt and isochronous endpoints not allowed");

    // USB 2.0 max packet size is 64 bytes, but isochronous can be up to 1,023!
    if (ep->maxsize > 64) panic("Packet size is currently limited to 64 bytes");

    // Setup hardware registers and data buffers
    if (!epn) { // Shared epx
        ep->ecr = &usbh_dpram->epx_ctrl;
        ep->bcr = &usbh_dpram->epx_buf_ctrl;
        ep->buf = &usbh_dpram->epx_data[0];
    } else if (ep->ep_addr & 0xf) { // Polled hardware endpoint
        uint8_t i = epn - 1;
        ep->ecr = &usbh_dpram->int_ep_ctrl       [i].ctrl;
        ep->bcr = &usbh_dpram->int_ep_buffer_ctrl[i].ctrl;
        ep->buf = &usbh_dpram->epx_data         [(i + 2) * 64];

        // Setup polled hardware endpoint
        bool ls = false;
        bool in = ep_in(ep);
        usb_hw->int_ep_addr_ctrl[i] = \
              (ls ? 1 : 0)        << 26    // Preamble: LS on FS hub
            | (in ? 0 : 1)        << 25    // Direction (In=0, Out=1)
            | (ep->ep_addr & 0xf) << 16    // Endpoint number
            |  ep->dev_addr;               // Device address
        usb_hw->int_ep_ctrl |= (1 << epn); // Activate the endpoint
    } else {
        panic("EP0 cannot be polled");
    }

    // Set bcr first to prevent any issues when ecr is set
    *ep->bcr = 0; nop(); nop(); nop(); nop(); nop(); nop();

    // Setup shared epx endpoint and enable double buffering
    *ep->ecr = EP_CTRL_ENABLE_BITS             // Enable endpoint
             |  (epn ? SINGLE_BUFFER           // Non-epx are single buffered
                     : DOUBLE_BUFFER)          // And epx are double buffered
             |   ep->type << 26                // Set transfer type
             | ((ep->interval || 1) - 1) << 16 // Polling interval minus 1 ms
             | ((uint32_t) ep->buf) & 0xfc0;   // DSPRAM offset: 64-byte aligned

    // Control endpoints start with DATA0, otherwise start with DATA1
    ep->data_pid = 0;

    // NOTE: Devices should start with DATA0, but some start with DATA1.
    //
    // The easiest way to handle this is to simply hard code it, as Linux does.

    if (ep->dev_addr) {
        device_t *dev = get_device(ep->dev_addr);

        if (dev->vid == 0x0403 && dev->pid == 0xcd18) {
            ep->data_pid = 1;
        }
    }

    // Set as configured
    ep->configured = true;
}

SDK_INJECT void reset_endpoint(endpoint_t *ep) {
    ep->active     = false;
    ep->setup      = false;
    ep->bytes_left = 0;
    ep->bytes_done = 0;
}

void setup_epx() {
    setup_endpoint(epx, 0, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 0,
        .bInterval        = 0,
    }), ctrl_buf);
}

endpoint_t *get_endpoint(uint8_t dev_addr, uint8_t ep_num) {
    for (uint8_t i = 0; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (ep->configured) {
            if ((ep->dev_addr == dev_addr) && ((ep->ep_addr & 0xf) == ep_num))
                return ep;
        }
    }
    panic("No configured EP%u on device %u", ep_num, dev_addr);
    return NULL;
}

endpoint_t *next_endpoint(uint8_t dev_addr, usb_endpoint_descriptor_t *usb,
                          uint8_t *user_buf) {
    if (!(usb->bEndpointAddress & 0xf)) panic("EP0 cannot be requested");
    for (uint8_t i = 1; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];
        if (!ep->configured) {
            ep->dev_addr = dev_addr;
            setup_endpoint(ep, i, usb, user_buf);
            return ep;
        }
    }
    panic("No free endpoints remaining");
    return NULL;
}

void clear_endpoint(uint8_t dev_addr, uint8_t ep_num) {
    endpoint_t *ep = get_endpoint(dev_addr, ep_num);
    memclr(ep, sizeof(endpoint_t));
}

void clear_endpoints() {
    memclr(eps, sizeof(eps));
}

void print_endpoints() {
    printf("\nEndpoint Status:\n");
    printf("─────────────────────────────────────────────────────────────\n");

    for (uint8_t i = 0; i < MAX_ENDPOINTS; i++) {
        endpoint_t *ep = &eps[i];

        if (!ep->configured) {
            continue;
        }

        printf("EP%u%s:\n", ep->ep_addr & 0xf, ep_in(ep) ? " IN" : " OUT");
        printf("  Device Address: 0x%02x\n", ep->dev_addr);
        printf("  Endpoint Addr:  0x%02x\n", ep->ep_addr);
        printf("  Max Size:       0x%04x\n", ep->maxsize);
        printf("  Type:          %s\n",
            ep->type == USB_TRANSFER_TYPE_CONTROL ? "Control" :
            ep->type == USB_TRANSFER_TYPE_ISOCHRONOUS ? "Isochronous" :
            ep->type == USB_TRANSFER_TYPE_BULK ? "Bulk" :
            ep->type == USB_TRANSFER_TYPE_INTERRUPT ? "Interrupt" : "Unknown");
        printf("  Active:        %s\n", ep->active ? "Yes" : "No");
        printf("  Setup:         %s\n", ep->setup ? "Yes" : "No");
        printf("  Data PID:      %u\n", ep->data_pid);
        printf("  Bytes Left:    0x%04x\n", ep->bytes_left);
        printf("  Bytes Done:    0x%04x\n", ep->bytes_done);
        printf("─────────────────────────────────────────────────────────\n");
    }
}

endpoint_t eps[MAX_ENDPOINTS], *epx = eps;

// ==[ Buffers ]================================================================

uint16_t start_buffer(endpoint_t *ep, uint8_t buf_id) {
    bool     in  = ep_in(ep);                         // Inbound buffer?
    bool     run = ep->bytes_left > ep->maxsize;      // Continue to run?
    uint8_t  pid = ep->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(ep->bytes_left, ep->maxsize);  // Determine buffer length
    uint16_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (run ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 | len;                               // Set buffer length

    // Toggle DATA0/DATA1 pid
    ep->data_pid = pid ^ 1u;

    // OUT: Copy outbound data from the user buffer to the endpoint
    if (!in && len) {
        uint8_t *src = (uint8_t *) (&ep->user_buf[ep->bytes_done]);
        uint8_t *dst = (uint8_t *) (ep->buf + buf_id * 64);
        memcpy(dst, src, len);
        hexdump(buf_id ? "│OUT/2" : "│OUT/1", src, len, 1);
        ep->bytes_done += len;
    }

    // Update byte counts
    ep->bytes_left -= len;

    return bcr;
}

uint16_t finish_buffer(endpoint_t *ep, uint8_t buf_id, io_rw_32 bcr) {
    bool     in   = ep_in(ep);                   // Inbound buffer?
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Is buffer full? (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // IN: Copy inbound data from the endpoint to the user buffer
    if (in && len) {
        uint8_t *src = (uint8_t *) (ep->buf + buf_id * 64);
        uint8_t *dst = (uint8_t *) &ep->user_buf[ep->bytes_done];
        memcpy(dst, src, len);
        hexdump(buf_id ? "│IN/2" : "│IN/1", dst, len, 1);
        ep->bytes_done += len;
    }

    // Short packet (below maxsize) means the transfer is done
    if (len < ep->maxsize)
        ep->bytes_left = 0;

    return len;
}

// ==[ Transactions ]===========================================================

void start_transaction(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;
    io_rw_32 *ecr = ep->ecr;
    io_rw_32 *bcr = ep->bcr, hold;
    uint32_t fire = USB_BUF_CTRL_AVAIL;

    assert(ep->active);

    // Hold the value for bcr
    hold = start_buffer(ep, 0);

    // If using epx, update the buffering mode
    if (ep->ecr == epx->ecr) {
        if (hold & USB_BUF_CTRL_LAST) {        // For single buffering:
            *ecr &= ~DOUBLE_BUFFER;            //   Disable double-buffering
            *ecr |=  SINGLE_BUFFER;            //   Enable  single-buffering
        } else {                               // For double buffering:
            hold |= start_buffer(ep, 1) << 16; //   Merge bcr for buf1
            *ecr &= ~SINGLE_BUFFER;            //   Disable single-buffering
            *ecr |=  DOUBLE_BUFFER;            //   Enable  double-buffering
            fire |=  fire << 16;               //   Fire buffers together
        }
    }

    // Set bcr and allow time for it to settle (problems if CPU reads too soon!)
    *bcr = hold; nop(); nop(); nop(); nop(); nop(); nop();

    // Debug output
    if (!ep->bytes_done) {
        printf("\n");
        printf(DEBUG_ROW);
        printf( "│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transaction started");
        show_endpoint(ep);
        printf(DEBUG_ROW);
        bindump("│SIE", usb_hw->sie_ctrl);
        bindump("│SSR", usb_hw->sie_status);
        printf(DEBUG_ROW);
        bindump("│DAR", usb_hw->dev_addr_ctrl);
        bindump("│ECR", *ep->ecr);
        bindump("│BCR", hold | fire);
        if (ep->setup) {
            uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
            printf(DEBUG_ROW);
            hexdump("│SETUP", packet, sizeof(usb_setup_packet_t), 1);
            printf(DEBUG_ROW);
        } else if (!ep->bytes_left) {
            bool in = ep_in(ep);
            char *str = in ? "IN" : "OUT";
            printf(DEBUG_ROW);
            printf( "│ZLP\t│ %-4s │ Device %-28u │            │\n", str, ep->dev_addr);
            printf(DEBUG_ROW);
        } else {
            printf(DEBUG_ROW);
        }
    }

    // Fire off the transaction, which yields to the USB controller
    *bcr = hold | fire;
}

void finish_transaction(endpoint_t *ep) {
    io_rw_32 *ecr = ep->ecr;
    io_rw_32 *bcr = ep->bcr;

    assert(ep->active);

    // Finish based on if we're single or double buffered
    if (*ecr & EP_CTRL_DOUBLE_BUFFERED_BITS) {         // For double buffering:
        if (finish_buffer(ep, 0, *bcr) == ep->maxsize) //   Finish first buffer
            finish_buffer(ep, 1, *bcr >> 16);          //   Finish second buffer
    } else {                                           // For single buffering:
        uint32_t bch = usb_hw->buf_cpu_should_handle;  //   Workaround RP2040-E4
        uint32_t tmp = *bcr;                           //   First, get bcr
        if (bch & 1u) tmp >>= 16;                      //   Shift if needed
        finish_buffer(ep, 0, tmp);                     //   Then, finish buffer
    }
}

// ==[ Transfers ]==============================================================

const uint32_t USB_SIE_CTRL_BASE = USB_SIE_CTRL_PULLDOWN_EN_BITS   // Enable
                                 | USB_SIE_CTRL_VBUS_EN_BITS       // Allow VBUS
                                 | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Low speed
                                 | USB_SIE_CTRL_SOF_EN_BITS;       // Full speed

SDK_INJECT const char *transfer_type(uint8_t bits) {
    switch (bits & USB_TRANSFER_TYPE_BITS) {
        case USB_TRANSFER_TYPE_CONTROL:     return "Control"    ; break;
        case USB_TRANSFER_TYPE_ISOCHRONOUS: return "Isochronous"; break;
        case USB_TRANSFER_TYPE_BULK:        return "Bulk"       ; break;
        case USB_TRANSFER_TYPE_INTERRUPT:   return "Interrupt"  ; break;
        default:                            return "Unknown"    ; break;
    }
}

// TODO: Clear a stall and toggle data PID back to DATA0
// TODO: Abort a transfer if not yet started and return true on success

// Start a new transfer
void start_transfer(endpoint_t *ep) {
    if (!ep->user_buf) panic("Transfer has an invalid memory pointer");
    if ( ep->active  ) panic("Transfer already active on endpoint");
    ep->active = true;

    // Calculate registers
    uint32_t dar = (ep->ep_addr & 0xf) << 16 | ep->dev_addr;
    uint32_t sie = USB_SIE_CTRL_BASE;

    // Shared epx must be setup each transfer, epn's are already setup
    if (ep->ecr == epx->ecr) {
        bool ls = false;
        bool in = ep_in(ep);
        bool ss = ep->setup && !ep->bytes_done; // Start of a SETUP packet

        sie |= (!ls ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS ) // LS on a FS hub
            |  ( in ?     USB_SIE_CTRL_RECEIVE_DATA_BITS  // IN=Receive
                        : USB_SIE_CTRL_SEND_DATA_BITS   ) // OUT=Send
            |  (!ss ? 0 : USB_SIE_CTRL_SEND_SETUP_BITS  );// Toggle SETUP packet
    }

    // Set the registers
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = sie;

    // Get the transaction and buffers ready
    start_transaction(ep);

    // Initiate the transfer
    usb_hw->sie_ctrl      = sie | USB_SIE_CTRL_START_TRANS_BITS;
}

// Send a ZLP transfer out
void transfer_zlp(void *arg) {
    endpoint_t *ep = (endpoint_t *) arg;

    // Force direction to OUT
    ep->ep_addr &= ~USB_DIR_IN;
    ep->bytes_left = 0;

    start_transfer(ep);
}

// Send a control transfer using an existing setup packet
void control_transfer(device_t *dev, usb_setup_packet_t *setup) {
    printf("Control transfer: ");
    if (!epx->configured) print_endpoints();
    if ( epx->type)       panic("Not a control endpoint");

    // Copy the setup packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_setup_packet_t));

    epx->dev_addr   = dev->dev_addr;
    epx->ep_addr    = setup->bmRequestType & USB_DIR_IN; // Thus, 0x80 is EP0/IN
    epx->maxsize    = dev->maxsize0;
    epx->setup      = true;
    epx->data_pid   = 1; // SETUP uses DATA0, so this next packet will be DATA1
    epx->user_buf   = ctrl_buf;
    epx->bytes_left = setup->wLength;
    epx->bytes_done = 0;

    // Flip the endpoint direction if there is no data phase
    if (!epx->bytes_left) epx->ep_addr ^= USB_DIR_IN;

    start_transfer(epx);
}

// Send a control transfer using a newly constructed setup packet
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

// Send a bulk transfer and pass a data buffer
void bulk_transfer(endpoint_t *ep, uint8_t *ptr, uint16_t len) {
    printf("Bulk transfer");
    if (!ep->configured) print_endpoints();
    if ( ep->type != USB_TRANSFER_TYPE_BULK) panic("Not a bulk endpoint");

    ep->user_buf   = ptr;
    ep->bytes_left = len;
    ep->bytes_done = 0;

    start_transfer(ep);
}

// Finish a transfer
void finish_transfer(endpoint_t *ep) {

    // Panic if the endpoint is not active
    if (!ep->active) panic("Endpoints must be active to finish");

    uint16_t len = ep->bytes_done;

    // Debug output
    if (len) {
        printf(DEBUG_ROW);
        printf( "│XFER\t│ %4u │ Device %-28u   Task #%-4u │\n",
                    len, ep->dev_addr, guid);
        hexdump("│Data", ep->user_buf, len, 1);
    } else {
        printf(DEBUG_ROW);
        printf( "│ZLP\t│ %-4s │ Device %-28u │ Task #%-4u │\n",
                    ep_dir(ep), ep->dev_addr, guid);
    }

    // Create the transfer task
    task_t transfer_task = {
        .type              = TASK_TRANSFER,
        .guid              = guid++,
        .transfer.dev_addr = ep->dev_addr,      // Device address
        .transfer.ep_num   = ep->ep_addr & 0xf, // Endpoint number (EP0-EP15)
        .transfer.user_buf = ep->user_buf,      // User buffer
        .transfer.len      = ep->bytes_done,    // Buffer length
        .transfer.cb       = ep->cb,            // Callback fn
        .transfer.status   = TRANSFER_SUCCESS,  // Transfer status
    };

    // Reset the endpoint
    reset_endpoint(ep);

    // Queue the transfer task
    queue_add_blocking(queue, &transfer_task);
}

// ==[ Descriptors ]============================================================

SDK_INJECT void get_descriptor(device_t *dev, uint8_t type, uint8_t len) {
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

void get_device_descriptor(device_t *dev) {
    printf("Get device descriptor\n");

    uint8_t len = sizeof(usb_device_descriptor_t);

    if (!dev->maxsize0)
        len = dev->maxsize0 = 8; // Default per USB 2.0 spec

    get_descriptor(dev, USB_DT_DEVICE, len);
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

void get_configuration_descriptor(device_t *dev, uint8_t len) {
    printf("Get configuration descriptor\n");

    get_descriptor(dev, USB_DT_CONFIG, len);
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

void get_string_descriptor_blocking(device_t *dev, uint8_t index) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQ_TYPE_STANDARD
                       | USB_REQ_TYPE_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_STRING, index),
        .wIndex        = 0,
        .wLength       = MAX_CTRL_BUF,
    }));

    do { usb_task(); } while (epx->active); // This transfer...
    do { usb_task(); } while (epx->active); // The ZLP...
}

void show_string_blocking(device_t *dev, uint8_t index) {

    // Request a string and wait for it
    get_string_descriptor_blocking(dev, index);
    uint8_t *ptr = ctrl_buf;

    // Prepare to parse Unicode string
    uint8_t   len =              *ptr / 2 - 1;
    uint16_t *uni = (uint16_t *) (ptr + 2);

    // Convert string from Unicode to UTF-8
    char *utf = (char[MAX_CTRL_BUF]) { 0 };
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

// Resets an FTDI device and configures its baud rate and line settings
void reset_ftdi(device_t *dev) {
    static uint8_t (states[MAX_DEVICES]) = { 0 };
    uint8_t state = ++states[dev->dev_addr];

    printf("FTDI reset step %u\n", state);

    switch (state) {
        case 1: command(dev, 0x40,  0,  0    , 1, 0); break; // reset interface
        case 2: command(dev, 0x40,  9, 16    , 1, 0); break; // set latency=16ms
        case 3: command(dev, 0x40,  3, 0x4138, 1, 0); break; // set baud=9600
        case 4: command(dev, 0x40,  1, 0x0303, 1, 0); break; // enable DTR/RTS
        case 5: command(dev, 0x40,  2, 0x1311, 1, 0); break; // flow control on
        default:
            states[dev->dev_addr] = 0;
            dev->state = DEVICE_READY;
            usb_debug(1);
            printf("FTDI reset complete\n");
            break;
    }
}

static inline int write_ring(driver_t *driver, const uint8_t *data, uint16_t len) {
    if (!driver || !driver->rx_buffer)
        return -1;
    return ring_write_blocking(driver->rx_buffer, data, len);
}

static inline int read_ring(driver_t *driver, uint8_t *buffer, uint16_t len) {
    if (!driver || !driver->rx_buffer)
        return -1;
    bool available = ring_is_empty(driver->rx_buffer);
    if (!available) return 0;

    uint16_t to_read = (available < len) ? available : len;
    return ring_read_blocking(driver->rx_buffer, buffer, to_read);
}


static driver_t drivers[MAX_DRIVERS];
static uint8_t driver_count = 0;

driver_t* find_driver(uint8_t topclass, uint8_t subclass, uint8_t protocol) {
    for (uint8_t i = 0; i < driver_count; i++) {
        driver_t *driver = &drivers[i];
        if (
            driver->if_topclass== topclass &&
            driver->if_subclass == subclass &&
            driver->if_protocol == protocol) {
            return driver;
        }
    }
    return NULL;
}

const driver_t driver_templates[] = {
    {
        .name                = "CDC",
        .if_topclass         = 0xFF,
        .if_subclass         = 0xFF,
        .if_protocol         = 0xFF,
        .rx_endpoint         = NULL,
        .tx_endpoint         = NULL,
        .rx_buffer           = NULL,

        .open                = (bool (*)(void *, uint16_t))open_trampoline,
        .config              = (void (*)(void))config_trampoline,
        .close               = (void (*)(void))close_trampoline,
        .send_data           = (void (*)(const uint8_t *, uint16_t))send_data_trampoline,
        .read_ring           = (void (*)(void *, uint16_t))read_ring_trampoline,
        .write_ring          = (void (*)(void))write_ring_trampoline,

        .open_impl           = cdc_open,
        .config_impl         = reset_ftdi,
        .close_impl          = NULL,
        .send_data_impl      = cdc_send,
        .read_ring_impl      = read_ring,
        .write_ring_impl     = write_ring
    }
};

bool driver_init(driver_t *driver, char *driver_name, uint16_t bufsize) {
    if (driver_count >= MAX_DRIVERS) {
        printf("No more room for drivers\n");
        return false;
    }

    const driver_t *template = NULL;
    for (int i = 0; i < sizeof(driver_templates) / sizeof(driver_t); i++) {
        if (strcmp(driver_templates[i].name, driver_name) == 0) {
            template = &driver_templates[i];
            break;
        }
    }

    if (!template) {
        printf("Could not find driver template named '%s'\n", driver_name);
        return false;
    }

    ring_t *rx_buffer = ring_new(bufsize);
    if (!rx_buffer) {
        printf("Failed to create ring buffer\n");
        return false;
    }
    
    driver_t *new_driver = &drivers[driver_count++];
    memcpy(new_driver, template, sizeof(driver_t));
    new_driver->rx_buffer = rx_buffer;

    return true;
}

driver_t* driver_instance_from_endpoint(endpoint_t *ep) {
    for (uint8_t i = 0; i < driver_count; i++) {
        driver_t *driver = &drivers[i];
        if (driver->rx_endpoint == ep || driver->tx_endpoint == ep) {
            return driver;
        }
    }
    return NULL;
}

driver_t* driver_instance_for_interface(uint8_t topclass, uint8_t subclass, uint8_t protocol) {
    for (uint8_t i = 0; i < driver_count; i++) {
        driver_t *driver = &drivers[i];
        if (driver->if_topclass == topclass &&
            driver->if_subclass == subclass &&
            driver->if_protocol == protocol) {
            return driver;
        }
    }
    return NULL;
}

// ==[ Driver Implementation for CDC ]==========================================

bool cdc_open(driver_t *driver, void *ptr, uint16_t len) {
    usb_configuration_descriptor_t *config_desc = (usb_configuration_descriptor_t *)ptr;

    printf("CDC Open: Attempting to configure driver\n");
    printf("Total config length: %u\n", len);

    driver->rx_endpoint = NULL;
    driver->tx_endpoint = NULL;

    uint8_t *cur = (uint8_t *)ptr;
    uint8_t *end = cur + len;

    while (cur < end) {
        if (cur[1] == USB_DT_ENDPOINT) {
            usb_endpoint_descriptor_t *epd = (usb_endpoint_descriptor_t *)cur;
            show_endpoint_descriptor(epd);

            if (epd->bmAttributes == USB_TRANSFER_TYPE_BULK) {
                endpoint_t *ep = next_endpoint(epx->dev_addr, epd, NULL);
                if (epd->bEndpointAddress & USB_DIR_IN) {
                    driver->rx_endpoint = ep;
                    printf("Found Bulk IN endpoint\n");
                } else {
                    driver->tx_endpoint = ep;
                    printf("Found Bulk OUT endpoint\n");
                }
            }
        }
        cur += cur[0];
    }

    if (!driver->rx_endpoint || !driver->tx_endpoint) {
        printf("Failed to find Bulk IN or Bulk OUT endpoints\n");
        return false;
    }

    return true;
}

void cdc_send(driver_t *driver, const uint8_t *data, uint16_t len) {
    if (!driver->tx_endpoint) {
        printf("Bulk OUT endpoint not set for CDC driver\n");
        return;
    }

    bulk_transfer(driver->tx_endpoint, (uint8_t*)data, len);
}

bool open_trampoline(driver_t *self, void *ptr, uint16_t len) {
    return self->open_impl ? self->open_impl(self, ptr, len) : false;
}

void config_trampoline(driver_t *self) {
    if (self->config_impl) self->config_impl(self);
}

void close_trampoline(driver_t *self) {
    if (self->close_impl) self->close_impl(self);
}

void send_data_trampoline(driver_t *self, const uint8_t *data, uint16_t len) {
    if (self->send_data_impl) self->send_data_impl(self, data, len);
}

void read_ring_trampoline(driver_t *self, uint16_t bytes_done) {
    if (self->read_ring_impl) self->read_ring_impl(self, bytes_done);
}

void write_ring_trampoline(driver_t *self) {
    if (self->write_ring_impl) self->write_ring_impl(self);
}


// ==[ Setup Drivers ]============================================================

bool setup_drivers(void *ptr, device_t *dev) {
    usb_configuration_descriptor_t *cfd = (usb_configuration_descriptor_t *)ptr;
    uint8_t *cur = (uint8_t *)ptr;
    uint8_t *end = cur + cfd->wTotalLength;

    while (cur < end) {
        if (cur[0] == 0) {
            printf("Malformed data: Descriptor length is 0\n");
            break;
        }

        if (cur[1] == USB_DT_INTERFACE) {
            usb_interface_descriptor_t *ifd = (usb_interface_descriptor_t *)cur;
            driver_t *matched_driver = driver_instance_for_interface(
                ifd->bInterfaceClass,
                ifd->bInterfaceSubClass,
                ifd->bInterfaceProtocol
            );

            if (matched_driver) {
                matched_driver->open(ptr, cfd->wTotalLength);
            }
        }

        cur += cur[0];
    }
    return true;
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
            clear_device(0);

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
            if (size > MAX_CTRL_BUF) {
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
            on_device_active(dev);

            break;
    }
}

// ==[ Callbacks ]==============================================================

SDK_WEAK void on_device_active(device_t *dev) {
    printf("WEAK: Device %u is now active\n", dev->dev_addr);
}

// ==[ Tasks ]==================================================================

SDK_INJECT const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        case TASK_CALLBACK: return "TASK_CALLBACK";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

SDK_INJECT const char *callback_name(void (*fn) (void *)) {
    if (fn == enumerate        ) return "enumerate"        ;
    if (fn == start_transaction) return "start_transaction";
    if (fn == transfer_zlp     ) return "transfer_zlp"     ;
    return "user defined function";
}

SDK_INLINE void queue_callback(void (*fn)(void *), void *arg) {
    queue_add_blocking(queue, &((task_t) {
        .type         = TASK_CALLBACK,
        .guid         = guid++,
        .callback.fn  = fn,
        .callback.arg = arg,
    }));
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        printf("\n=> %u) New task, %s\n\n", task.guid, task_name(type));
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
                clear_device(0);
                dev0->state = DEVICE_ENUMERATING;
                dev0->speed = task.connect.speed;

                // Show the device connection and speed
                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                printf("Device connected (%s speed)\n", str);

                // Start enumeration
                printf("Start enumeration\n");
                enumerate(dev0);

            }   break;

            case TASK_TRANSFER: {
                uint8_t    dev_addr = task.transfer.dev_addr; // Device address
                uint8_t    ep_num   = task.transfer.ep_num;   // Endpoint number
                uint8_t   *user_buf = task.transfer.user_buf; // User buffer
                uint16_t   len      = task.transfer.len;      // Buffer length
                endpoint_c cb       = task.transfer.cb;       // Callback fn
                uint8_t    status   = task.transfer.status;   // Transfer status

                // Get the endpoint
                endpoint_t *ep = get_endpoint(dev_addr, ep_num);

                // Handle the transfer
                device_t *dev = get_device(ep->dev_addr);

                if (len && dev->state < DEVICE_READY) {
                    printf("Calling transfer_zlp\n");
                    transfer_zlp(ep);
                } else if (dev->state < DEVICE_ACTIVE) {
                    printf("Calling enumerate\n");
                    enumerate(dev);
                } else if (dev->state < DEVICE_READY) {
                    printf("Calling reset_ftdi\n");
                    reset_ftdi(dev);
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

SDK_INJECT void printf_interrupts(uint32_t ints) {
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
    io_rw_32 ints = usb_hw->ints; // Interrupt bits after masking and forcing

    // ==[ EPX related registers and variables ]==

    io_rw_32 dar  = usb_hw->dev_addr_ctrl;              // dev_addr/ep_num
    io_rw_32 ecr  = usbh_dpram->epx_ctrl;               // Endpoint control
    io_rw_32 bcr  = usbh_dpram->epx_buf_ctrl;           // Buffer control
    io_rw_32 sie  = usb_hw->sie_ctrl;                   // SIE control
    io_rw_32 ssr  = usb_hw->sie_status;                 // SIE status
    io_ro_32 sof  = usb_hw->sof_rd;                     // Frame number
    bool     dbl  = ecr & EP_CTRL_DOUBLE_BUFFERED_BITS; // EPX double buffered

    // Fix RP2040-E4 by shifting buffer control registers for affected buffers
    if (!dbl && (usb_hw->buf_cpu_should_handle & 1u)) bcr >>= 16;

    // Get device address and endpoint information
    uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;     // 7 bits (lowest)
    uint8_t ep_num   = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >> // 4 bits (higher)
                              USB_ADDR_ENDP_ENDPOINT_LSB;
    endpoint_t *ep = get_endpoint(dev_addr, ep_num);

    // Show system state
    printf( "\n=> %u) New ISR", guid++);
    printf_interrupts(ints);
    printf( "\n\n");
    printf(DEBUG_ROW);
    printf( "│Frame  │ %4u │ %-35s", sof, "Interrupt Handler");
    show_endpoint(ep);
    printf(DEBUG_ROW);
    bindump("│INT", ints);
    bindump("│SIE", sie);
    bindump("│SSR", ssr);
    printf(DEBUG_ROW);
    bindump("│DAR", dar);
    bindump("│ECR", ecr);
    bindump("│BCR", bcr);

    // Connection (attach or detach)
    if (ints &  USB_INTS_HOST_CONN_DIS_BITS) {
        ints ^= USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed
        uint8_t speed = (sie &  USB_SIE_STATUS_SPEED_BITS)
                             >> USB_SIE_STATUS_SPEED_LSB;

        // Clear the interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;

        // Handle connect and disconnect
        if (speed) {
            printf(DEBUG_ROW);
            printf( "│CONNECT│ %-4s │ %-35s │ Task #%-4u │\n", "",
                     "New device connected", guid);

            queue_add_blocking(queue, &((task_t) { // ~20 μs
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
            }));
        } else {
            printf(DEBUG_ROW);
            printf( "│DISCONN│ %-4s │ %-35s │            │\n", "",
                     "Device disconnected");

            clear_device(0);
        }
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &  USB_INTS_STALL_BITS) {
        ints ^= USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        panic("Stall detected");
    }

    // Buffer processing is needed
    if (ints &  USB_INTS_BUFF_STATUS_BITS) {
        ints ^= USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 0b11; // (2 bits at time, IN/OUT transfer together)

        // Show single/double buffer status of EPX and which buffers are ready
        printf(DEBUG_ROW);
        bindump(dbl ? "│BUF/2" : "│BUF/1", bits);

        // Finish transactions on each pending endpoint
        endpoint_t *epz;
        for (uint8_t epn = 0; epn < MAX_ENDPOINTS && bits; epn++, mask <<= 2) {
            if (bits &   mask) {
                bits &= ~mask;
                usb_hw_clear->buf_status = mask;

                // Get a handle to the correct endpoint
                epz = (!epn && ep->ecr == epx->ecr) ? ep : &eps[epn];

                // Show epn details
                char *str = (char[MAX_CTRL_BUF]) { 0 };
                sprintf(str, "│ECR%u", epn);
                bindump(str, *epz->ecr);
                sprintf(str, "│BCR%u", epn);
                bindump(str, *epz->bcr);

                // Finish the transaction
                finish_transaction(epz);

                // Start the next transaction or finish the transfer
                if (epz->bytes_left) {
                    queue_add_blocking(queue, &((task_t) {
                        .type         = TASK_CALLBACK,
                        .guid         = guid++,
                        .callback.fn  = start_transaction,
                        .callback.arg = (void *) epz,
                    }));
                } else {
                    finish_transfer(ep);
                }
            }
        }

        // Panic if we missed any buffers
        if (bits) panic("Unhandled buffer mask: %032b", bits);
    }

    // Transfer complete (last packet)
    if (ints &  USB_INTS_TRANS_COMPLETE_BITS) {
        ints ^= USB_INTS_TRANS_COMPLETE_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;

        driver_t *driver = driver_instance_from_endpoint(ep);

        if (driver) {
            driver->write_ring(ep->user_buf, ep->bytes_done);
        }
    }

    // Receive timeout (waited too long without seeing an ACK)
    if (ints &  USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints ^= USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        panic("Timeout waiting for data");
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
    if (ints) panic("Unhandled IRQ bitmask 0x%08x", ints);

    printf(DEBUG_ROW);
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

    clear_devices();
    clear_endpoints();
    setup_epx();

    printf(DEBUG_ROW);
    bindump("│INT", usb_hw->inte);
    printf(DEBUG_ROW);
}

void usb_init() {
    stdout_uart_init();
    printf("\033[2J\033[H\n==[ PicoUSB Host ]==\n\n");
    queue_init(queue, sizeof(task_t), 64);
    setup_usb_host();
}

// ==[ End ]====================================================================
