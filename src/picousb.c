#include "picousb.h"

// =============================================================================
// PicoUSB - A smaller than tiny USB Host library for the Raspberry Pi Pico/W
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
// Author: Babak Rasouli <b@bak.rasou.li>
//   Date: November 25, 2024
//  Legal: Same license as the Raspberry Pi Pico SDK
//
// Thanks to Ha Thach for TinyUSB and https://github.com/hathach/tinyusb
// Thanks to Miroslav Nemecek for his https://github.com/Panda381/PicoLibSDK
// =============================================================================

// ==[ Devices ]================================================================

device_t devices[MAX_DEVICES], *dev0 = devices;

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

// ==[ Pipes ]==================================================================

pipe_t pipes[MAX_PIPES], *ctrl = pipes;

static uint8_t ctrl_buf[MAX_CTRL_BUF]; // Shared control transfer buffer

SDK_INJECT const char *ep_dir(uint8_t in) {
    return in ? "IN" : "OUT";
}

SDK_INJECT void show_pipe(pipe_t *pp) {
    uint8_t in = pp->ep_in;
    if (pp->setup) { // For SETUP packets, we need to use the original direction
        usb_setup_packet_t *p = (usb_setup_packet_t *) usbh_dpram->setup_packet;
        in = p->bmRequestType & USB_DIR_IN ? 1 : 0;
    }
    debug(" │ D%-2uEP%-2d%-4s│\n", pp->dev_addr, pp->ep_num, ep_dir(in));
}

void setup_pipe(pipe_t *pp, uint8_t phe, usb_endpoint_descriptor_t *epd,
                    uint8_t *user_buf) {

    // Populate the pipe (clears all fields not present)
    *pp = (pipe_t) {
        .dev_addr = pp->dev_addr,
        .ep_num   = epd->bEndpointAddress & 0x0f,
        .ep_in    = epd->bEndpointAddress & USB_DIR_IN ? 1 : 0,
        .type     = epd->bmAttributes,
        .interval = epd->bInterval,
        .maxsize  = epd->wMaxPacketSize,
        .user_buf = user_buf,
    };

    // Panic if endpoint is not a control or bulk transfer type
    if (pp->type == 1u) panic("Isochronous endpoints not allowed");

    // USB 2.0 max packet size is 64 bytes, but isochronous can be up to 1,023!
    if (pp->maxsize > 64) panic("Packet size is currently limited to 64 bytes");

    // Setup hardware registers and data buffers
    if (!phe) { // If using epx (not a polled hardware endpoint)
        pp->ecr = &usbh_dpram->epx_ctrl;
        pp->bcr = &usbh_dpram->epx_buf_ctrl;
        pp->buf = &usbh_dpram->epx_data[0];
    } else { // Using a polled hardware endpoint
        uint8_t i = phe - 1; // these start at one, so adjust
        pp->ecr = &usbh_dpram->int_ep_ctrl       [i].ctrl;
        pp->bcr = &usbh_dpram->int_ep_buffer_ctrl[i].ctrl;
        pp->buf = &usbh_dpram->epx_data         [(i + 2) * 64];

        // Setup as a polled hardware endpoint
        bool ls = false;
        usb_hw->int_ep_addr_ctrl[i] =
              (ls        ? 1 : 0) << 26 // Preamble: LS on FS hub
            | (pp->ep_in ? 0 : 1) << 25 // Direction (In=0, Out=1)
            |  pp->ep_num         << 16 // Endpoint number
            |  pp->dev_addr;            // Device address
        usb_hw->int_ep_ctrl |= (1 << phe); // Activate the endpoint
    }

    // Set bcr first to prevent any issues when ecr gets in the next line
    *pp->bcr = 0; nop(); nop(); nop(); nop(); nop(); nop();

    // Setup the endpoint control register based on the type of endpoint
    *pp->ecr = EP_CTRL_ENABLE_BITS             // Enable this endpoint
             |  (phe ? SINGLE_BUFFER           // Non-epx are single buffered
                     : DOUBLE_BUFFER)          // And epx starts double buffered
             |   pp->type                << 26 // Set transfer type
             | ((pp->interval || 1) - 1) << 16 // Polling interval minus 1 ms
             |(((uint32_t) pp->buf) & 0xfc0);  // DSPRAM offset: 64-byte aligned

    // NOTE: Endpoints should start with DATA0. However, there are some devices
    // that are non-standard and start with DATA1. Linux just hard codes these
    // values, so we do too.

    pp->data_pid = 0;

    if (pp->dev_addr) {
        device_t *dev = get_device(pp->dev_addr);

        if (dev->vid == 0x0403 && dev->pid == 0xcd18) {
            pp->data_pid = 1;
        }
    }

    // Set as configured
    pp->status = ENDPOINT_CONFIGURED;
}

void setup_ctrl() {
    setup_pipe(ctrl, 0, &((usb_endpoint_descriptor_t) {
        .bLength          = sizeof(usb_endpoint_descriptor_t),
        .bDescriptorType  = USB_DT_ENDPOINT,
        .bEndpointAddress = 0,
        .bmAttributes     = USB_TRANSFER_TYPE_CONTROL,
        .wMaxPacketSize   = 0,
        .bInterval        = 0,
    }), ctrl_buf);
}

SDK_INJECT void reset_pipe(pipe_t *pp) {
    pp->status     = ENDPOINT_FINISHED;
    pp->setup      = false;
    pp->bytes_left = 0;
    pp->bytes_done = 0;
}

pipe_t *get_pipe(uint8_t dev_addr, uint8_t ep_num) {
    for (uint8_t i = 0; i < MAX_PIPES; i++) {
        pipe_t *pp = &pipes[i];
        if (pp->status) {
            if ((pp->dev_addr == dev_addr) && (pp->ep_num == ep_num))
                return pp;
        }
    }
    panic("No configured EP%u on device %u", ep_num, dev_addr);
    return NULL;
}

pipe_t *next_pipe(uint8_t dev_addr, usb_endpoint_descriptor_t *epd,
                          uint8_t *user_buf) {
    if (!(epd->bEndpointAddress & 0xf)) panic("EP0 cannot be requested");
    for (uint8_t i = 1; i < MAX_PIPES; i++) {
        pipe_t *pp = &pipes[i];
        if (!pp->status) {
            pp->dev_addr = dev_addr;
            setup_pipe(pp, i, epd, user_buf);
            return pp;
        }
    }
    panic("No free pipes remaining");
    return NULL;
}

void clear_pipe(uint8_t dev_addr, uint8_t ep_num) {
    pipe_t *pp = get_pipe(dev_addr, ep_num);
    memclr(pp, sizeof(pipe_t));
}

void clear_pipes() {
    memclr(pipes, sizeof(pipes));
}

// ==[ Buffers ]================================================================

uint16_t start_buffer(pipe_t *pp, uint8_t buf_id) {
    bool     in  = pp->ep_in;                         // Inbound buffer?
    bool     run = pp->bytes_left > pp->maxsize;      // Continue to run?
    uint8_t  pid = pp->data_pid;                      // Set DATA0/DATA1
    uint16_t len = MIN(pp->bytes_left, pp->maxsize);  // Determine buffer length
    uint16_t bcr = (in  ? 0 : USB_BUF_CTRL_FULL)      // IN/Recv=0, OUT/Send=1
                 | (run ? 0 : USB_BUF_CTRL_LAST)      // Trigger TRANS_COMPLETE
                 | (pid ?     USB_BUF_CTRL_DATA1_PID  // Use DATA1 if needed
                            : USB_BUF_CTRL_DATA0_PID) // Use DATA0 if needed
                 | len;                               // Set buffer length

    // Toggle DATA0/DATA1 pid
    pp->data_pid = pid ^ 1u;

    // OUT: Copy outbound data from the user buffer to the usb controller buffer
    if (!in && len) {
        uint8_t *src = (uint8_t *) (&pp->user_buf[pp->bytes_done]);
        uint8_t *dst = (uint8_t *) (pp->buf + buf_id * 64);
        memcpy(dst, src, len);
        _hex_(buf_id ? "│OUT/2" : "│OUT/1", src, len, 1);
        pp->bytes_done += len;
    }

    // Update byte counts
    pp->bytes_left -= len;

    return bcr;
}

uint16_t finish_buffer(pipe_t *pp, uint8_t buf_id, io_rw_32 bcr) {
    bool     in   = pp->ep_in;                   // Inbound buffer?
    bool     full = bcr & USB_BUF_CTRL_FULL;     // Is buffer full? (populated)
    uint16_t len  = bcr & USB_BUF_CTRL_LEN_MASK; // Buffer length

    // Inbound buffers must be full and outbound buffers must be empty
    assert(in == full);

    // IN: Copy inbound data from the usb controller buffer to the user buffer
    if (in && len) {
        uint8_t *src = (uint8_t *) (pp->buf + buf_id * 64);
        uint8_t *dst = (uint8_t *) &pp->user_buf[pp->bytes_done];
        memcpy(dst, src, len);
        _hex_(buf_id ? "│IN/2" : "│IN/1", dst, len, 1);
        pp->bytes_done += len;
    }

    // Short packet (below maxsize) means the transfer is done
    if (len < pp->maxsize)
        pp->bytes_left = 0;

    return len;
}

// ==[ Transactions ]===========================================================

void start_transaction(void *arg) {
    pipe_t *pp = (pipe_t *) arg;
    io_rw_32 *ecr = pp->ecr;
    io_rw_32 *bcr = pp->bcr, hold;
    uint32_t fire = USB_BUF_CTRL_AVAIL;

    assert(pp->status == ENDPOINT_STARTED);

    // Hold the value for bcr
    hold = start_buffer(pp, 0);

    // If using epx, update the buffering mode
    if (pp->ecr == ctrl->ecr) {
        if (hold & USB_BUF_CTRL_LAST) {        // For single buffering:
            *ecr &= ~DOUBLE_BUFFER;            //   Disable double-buffering
            *ecr |=  SINGLE_BUFFER;            //   Enable  single-buffering
        } else {                               // For double buffering:
            hold |= start_buffer(pp, 1) << 16; //   Overlay bcr for buf1
            *ecr &= ~SINGLE_BUFFER;            //   Disable single-buffering
            *ecr |=  DOUBLE_BUFFER;            //   Enable  double-buffering
            fire |=  fire << 16;               //   Fire buffers together
        }
    }

    // Set bcr and allow time for it to settle (problems if CPU reads too soon!)
    *bcr = hold; nop(); nop(); nop(); nop(); nop(); nop();

    // Log output
    if (!pp->bytes_done) {
        debug(LOG_ROW);
        debug("│Frame  │ %4u │ %-35s", usb_hw->sof_rd, "Transaction started");
        show_pipe(pp);
        debug(LOG_ROW);
        _bin_("│SIE", usb_hw->sie_ctrl);
        _bin_("│SSR", usb_hw->sie_status);
        debug(LOG_ROW);
        _bin_("│DAR", usb_hw->dev_addr_ctrl);
        _bin_("│ECR", *pp->ecr);
        _bin_("│BCR", hold | fire);
        if (pp->setup) {
            uint32_t *packet = (uint32_t *) usbh_dpram->setup_packet;
            debug(LOG_ROW);
            _hex_("│SETUP", packet, sizeof(usb_setup_packet_t), 1);
        } else if (!pp->bytes_left) {
            debug(LOG_ROW);
            debug("│ZLP\t│ %-4s │ Device %-28u │            │\n",
                ep_dir(pp->ep_in), pp->dev_addr);
        }
        debug(LOG_ROW);
    }

    // Fire off the transaction, which yields to the USB controller
    *bcr = hold | fire;
}

void finish_transaction(pipe_t *pp) {
    io_rw_32 *ecr = pp->ecr;
    io_rw_32 *bcr = pp->bcr;

    assert(pp->status == ENDPOINT_STARTED);

    // Finish based on if we're single or double buffered
    if (*ecr & EP_CTRL_DOUBLE_BUFFERED_BITS) {         // For double buffering:
        if (finish_buffer(pp, 0, *bcr) == pp->maxsize) //   Finish first buffer
            finish_buffer(pp, 1, *bcr >> 16);          //   Finish second buffer
    } else {                                           // For single buffering:
        uint32_t bch = usb_hw->buf_cpu_should_handle;  //   Workaround RP2040-E4
        uint32_t tmp = *bcr;                           //   First, get bcr
        if (bch & 1u) tmp >>= 16;                      //   Shift if needed
        finish_buffer(pp, 0, tmp);                     //   Then, finish buffer
    }
}

// ==[ Transfers ]==============================================================

// Globals for task handling
static uint32_t guid = 1;
static queue_t *queue = &((queue_t) { 0 });

// Helper variable for common bits
static const uint32_t USB_SIE_CTRL_BASE =
                      USB_SIE_CTRL_PULLDOWN_EN_BITS   // Enable
                    | USB_SIE_CTRL_VBUS_EN_BITS       // Allow VBUS
                    | USB_SIE_CTRL_KEEP_ALIVE_EN_BITS // Low speed
                    | USB_SIE_CTRL_SOF_EN_BITS;       // Full speed

SDK_INJECT const char *transfer_type(uint8_t bits) {
    switch (bits & USB_TRANSFER_TYPE_MASK) {
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
static void start_transfer(pipe_t *pp) {
    if (!pp->user_buf) panic("Transfer has an invalid memory pointer");
    if ( pp->status == ENDPOINT_STARTED) panic("Transfer already started");

    // Make the transfer active
    pp->status = ENDPOINT_STARTED;

    // Calculate registers
    uint32_t dar = pp->ep_num << 16 | pp->dev_addr;
    uint32_t sie = USB_SIE_CTRL_BASE;

    // Shared epx needs setup each transfer, polled hardware endpoints are ready
    if (pp->ecr == ctrl->ecr) {
        bool ls = false;
        bool in = pp->ep_in;
        bool ss = pp->setup && !pp->bytes_done; // Start of a SETUP packet

        sie |= (!ls ? 0 : USB_SIE_CTRL_PREAMBLE_EN_BITS ) // LS on a FS hub
            |  ( in ?     USB_SIE_CTRL_RECEIVE_DATA_BITS  // IN=Receive
                        : USB_SIE_CTRL_SEND_DATA_BITS   ) // OUT=Send
            |  (!ss ? 0 : USB_SIE_CTRL_SEND_SETUP_BITS  );// Toggle SETUP packet
    }

    // Set the registers
    usb_hw->dev_addr_ctrl = dar;
    usb_hw->sie_ctrl      = sie;

    // Get the transaction and buffers ready
    start_transaction(pp);

    // Initiate the transfer
    usb_hw->sie_ctrl      = sie | USB_SIE_CTRL_START_TRANS_BITS;
}

// Send a ZLP transfer
void transfer_zlp(void *arg) {
    pipe_t *pp = (pipe_t *) arg;

    pp->bytes_left = 0;

    // ZLP's for control transfers flip the direction and use DATA1
    if (!pp->type) {
        pp->ep_in    ^= 1;
        pp->data_pid  = 1;
    }

    start_transfer(pp);
}

// Helper to allow blocking until transfer is finished
void await_transfer(pipe_t *pp) {
    while (pp->status != ENDPOINT_FINISHED) usb_task();
}

// Send a control transfer
void control_transfer(device_t *dev, usb_setup_packet_t *setup) {
    if (!ctrl->status) panic("Control endpoint is not configured");
    if ( ctrl->type  ) panic("Not a control endpoint");

    // Copy the SETUP packet
    memcpy((void*) usbh_dpram->setup_packet, setup, sizeof(usb_setup_packet_t));

    // Configure the pipe
    ctrl->dev_addr   = dev->dev_addr;
    ctrl->ep_num     = 0; // Always EP0
    ctrl->ep_in      = setup->bmRequestType & USB_DIR_IN ? 1 : 0;
    ctrl->maxsize    = dev->maxsize0;
    ctrl->setup      = true;
    ctrl->data_pid   = 1; // SETUP uses DATA0, so this next packet will be DATA1
    ctrl->user_buf   = ctrl_buf;
    ctrl->bytes_left = setup->wLength;
    ctrl->bytes_done = 0;

    // Flip the endpoint direction if there is no data phase
    if (!ctrl->bytes_left) ctrl->ep_in ^= 1;

    start_transfer(ctrl);
}

// Send a control transfer using a newly constructed SETUP packet
void command(device_t *dev, uint8_t bmRequestType, uint8_t bRequest,
             uint16_t wValue, uint16_t wIndex, uint16_t wLength) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = bmRequestType,
        .bRequest      = bRequest,
        .wValue        = wValue,
        .wIndex        = wIndex,
        .wLength       = wLength,
    }));
    await_transfer(ctrl);
}

// Send a bulk transfer and pass a data buffer
void bulk_transfer(pipe_t *pp, uint8_t *ptr, uint16_t len) {
    if (!pp->status) panic("Endpoint is not configured");
    if ( pp->type != USB_TRANSFER_TYPE_BULK) panic("Not a bulk endpoint");

    pp->user_buf   = ptr;
    pp->bytes_left = len;
    pp->bytes_done = 0;

    start_transfer(pp);
}

// Finish a transfer
static void finish_transfer(pipe_t *pp) {

    // Panic if the pipe is not active
    if (pp->status != ENDPOINT_STARTED) panic("Pipes must be active to finish");

    // Get the transfer length (actual bytes transferred)
    uint16_t len = pp->bytes_done;

    // Log output
    if (len) {
        debug(LOG_ROW);
        debug("│XFER\t│ %4u │ Device %-28u   Task #%-4u │\n",
                len, pp->dev_addr, guid);
        _hex_("│Data", pp->user_buf, len, 1);
    } else {
        debug(LOG_ROW);
        debug("│ZLP\t│ %-4s │ Device %-28u │ Task #%-4u │\n",
                ep_dir(pp->ep_in), pp->dev_addr, guid);
    }

    // Create the transfer task
    task_t transfer_task = {
        .type              = TASK_TRANSFER,
        .guid              = guid++,
        .transfer.status   = TRANSFER_SUCCESS,  // Transfer status
        .transfer.dev_addr = pp->dev_addr,      // Device address
        .transfer.ep_num   = pp->ep_num,        // Endpoint number (EP0-EP15)
        .transfer.user_buf = pp->user_buf,      // User buffer
        .transfer.len      = pp->bytes_done,    // Buffer length
        .fn                = pp->fn,            // Callback function
        .arg               = pp->arg,           // Callback argument
    };

    // Reset the pipe
    reset_pipe(pp);

    // Control transfer callbacks are cleared each time
    if (!pp->ep_num && pp->fn) {
        pp->fn  = NULL;
        pp->arg = NULL;
    }

    // Queue the transfer task
    queue_add_blocking(queue, &transfer_task);
}

// ==[ Descriptors ]============================================================

SDK_INJECT void get_descriptor(device_t *dev, uint8_t type, uint8_t len) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQUEST_TYPE_STANDARD
                       | USB_REQUEST_RECIPIENT_DEVICE,
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

    flash("Connected Device:\n");
    flash("  Total Length:       %u\n"    , d->bLength);
    _bcd_("  USB Version:        "        , d->bcdUSB);
    flash("  Device Class:       %u\n"    , d->bDeviceClass);
    flash("    Subclass:         %u\n"    , d->bDeviceSubClass);
    flash("    Protocol:         %u\n"    , d->bDeviceProtocol);
    flash("  Packet Size:        %u\n"    , d->bMaxPacketSize0);
    flash("  Vendor Id:          0x%04x\n", d->idVendor);
    flash("  Product Id:         0x%04x\n", d->idProduct);
    _bcd_("  Version:            "        , d->bcdDevice);
    flash("  Manufacturer:       [#%u]\n" , d->iManufacturer);
    flash("  Product:            [#%u]\n" , d->iProduct);
    flash("  Serial:             [#%u]\n" , d->iSerialNumber);
    flash("\n");
}

void get_device_descriptor(device_t *dev) {
    debug("Get device descriptor\n");

    uint8_t len = sizeof(usb_device_descriptor_t);

    if (!dev->maxsize0)
        len = dev->maxsize0 = 8; // Default per USB 2.0 spec

    get_descriptor(dev, USB_DT_DEVICE, len);
}

void show_configuration_descriptor(void *ptr) {
    usb_configuration_descriptor_t *d = (usb_configuration_descriptor_t *) ptr;

    flash("Configuration Descriptor:\n");
    flash("  Total Length:       %u\n"   , d->wTotalLength);
    flash("  Interfaces:         %u\n"   , d->bNumInterfaces);
    flash("  Config Value:       %u\n"   , d->bConfigurationValue);
    flash("  Config Name:        [#%u]\n", d->iConfiguration);
    flash("  Attributes:         ");
    {
        char *sp = d->bmAttributes & 0x40 ? "Self Powered"  : NULL;
        char *rw = d->bmAttributes & 0x20 ? "Remote Wakeup" : NULL;

        if (sp && rw) flash("%s, %s\n", sp, rw);
        else if  (sp) flash("%s\n", sp);
        else if  (rw) flash("%s\n", rw);
        else          flash("None\n");
    }
    flash("  Max Power:          %u mA\n", d->bMaxPower * 2);
    flash("\n");
}

void show_interface_assoc_descriptor(void *ptr) {
    usb_interface_assoc_descriptor_t *d = (usb_interface_assoc_descriptor_t *)
                                                                            ptr;
    flash("Interface Association Descriptor:\n");
    flash("  Length:          %u\n"    , d->bLength);
    flash("  First Interface: %u\n"    , d->bFirstInterface);
    flash("  Interface Count: %u\n"    , d->bInterfaceCount);
    flash("  Class:           0x%02x\n", d->bFunctionClass);
    flash("  SubClass:        0x%02x\n", d->bFunctionSubClass);
    flash("  Protocol:        0x%02x\n", d->bFunctionProtocol);
    flash("  Function:        [#%u]\n" , d->iFunction);
    flash("\n");
}

void show_interface_descriptor(void *ptr) {
    usb_interface_descriptor_t *d = (usb_interface_descriptor_t *) ptr;

    flash("Interface Descriptor:\n");
    flash("  Interface:          %u\n"    , d->bInterfaceNumber);
    flash("  Alternate:          %u\n"    , d->bAlternateSetting);
    flash("  Endpoints:          %u\n"    , d->bNumEndpoints);
    flash("  Class:              0x%02x\n", d->bInterfaceClass);
    flash("  Subclass:           0x%02x\n", d->bInterfaceSubClass);
    flash("  Protocol:           0x%02x\n", d->bInterfaceProtocol);
    flash("  Name:               [#%u]\n" , d->iInterface);
    flash("\n");
}

void get_configuration_descriptor(device_t *dev, uint8_t len) {
    debug("Get configuration descriptor\n");

    get_descriptor(dev, USB_DT_CONFIGURATION, len);
}

void show_endpoint_descriptor(void *ptr) {
    usb_endpoint_descriptor_t *d = (usb_endpoint_descriptor_t *) ptr;
    uint8_t in = d->bEndpointAddress & USB_DIR_IN ? 1 : 0;

    flash("Endpoint Descriptor:\n");
    flash("  Length:             %u\n"   , d->bLength);
    flash("  Endpoint number:    EP%u\n" , d->bEndpointAddress & 0x0f);
    flash("  Endpoint direction: %s\n"   , ep_dir(in));
    flash("  Attributes:         0x%02x" , d->bmAttributes);
    flash(" (%s Transfer Type)\n"        , transfer_type(d->bmAttributes));
    flash("  Max Packet Size:    %u\n"   , d->wMaxPacketSize);
    flash("  Interval:           %u\n"   , d->bInterval);
    flash("\n");
}

void unicode_to_utf8(uint8_t *src, uint8_t *dst) {
    uint8_t   len =              *src / 2 - 1;
    uint16_t *uni = (uint16_t *) (src + 2);

    char *utf = (char *) dst;
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
}

void show_string(void *arg) {
    static uint8_t utf[MAX_CTRL_BUF] = { 0 };
    uint8_t index = (uint8_t) (uintptr_t) arg;
    unicode_to_utf8(ctrl_buf, utf);
    flash("[String #%u]: \"%s\"\n", index, utf);
}

void get_string_descriptor(device_t *dev, uint8_t index) {
    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_IN
                       | USB_REQUEST_TYPE_STANDARD
                       | USB_REQUEST_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_GET_DESCRIPTOR,
        .wValue        = MAKE_U16(USB_DT_STRING, index),
        .wIndex        = 0,
        .wLength       = MAX_CTRL_BUF,
    }));
}

void show_string_descriptor_blocking(device_t *dev, uint8_t index) {

    // Wait for get_string_descriptor() to finish
    get_string_descriptor(dev, index);
    await_transfer(ctrl);

    // // Set callback
    // ctrl->fn  = show_string;
    // ctrl->arg = (void *) (uintptr_t) index;

    // Wait for transfer_zlp() to finish
    transfer_zlp(ctrl);
    await_transfer(ctrl);

    // When done, show string
    show_string((void *) (uintptr_t) index);
}

// ==[ Drivers ]================================================================

driver_t drivers[MAX_DRIVERS];

static uint8_t driver_count = 0;

void register_driver(driver_t *driver) {
    if (driver_count < MAX_DRIVERS) {
        drivers[driver_count++] = *driver;
    } else {
        panic("No free driver slots remaining");
    }
}

bool enumerate_descriptors(void *ptr, device_t *dev) {
    usb_configuration_descriptor_t   *cfd; // Configuration descriptor
    usb_interface_assoc_descriptor_t *iad; // Interface association descriptor
    usb_interface_descriptor_t       *ifd; // Interface descriptor
    usb_endpoint_descriptor_t        *epd; // Endpoint descriptor

    uint8_t *cur = ptr; // Points to a list of descriptors
    uint8_t *end = cur + ((usb_configuration_descriptor_t *) cur)->wTotalLength;
    uint8_t  ias = 1; // Number of interface associations

    // Iterate through each descriptor
    for ( ; cur < end ; cur += *cur ) {
        if (!*cur) panic("Invalid descriptor");
        uint8_t type = cur[1];

        // Log output
        _hex_("Detail", cur, *cur, 1);
        debug("\n");

        switch (type) {

            case USB_DT_CONFIGURATION:
                cfd = (usb_configuration_descriptor_t *) cur;
                show_configuration_descriptor(cfd);
                break;

            case USB_DT_INTERFACE_ASSOCIATION:
                iad = (usb_interface_assoc_descriptor_t *) cur;
                ias = iad->bInterfaceCount;
                show_interface_assoc_descriptor(iad);
                break;

            case USB_DT_INTERFACE: {
                ifd = (usb_interface_descriptor_t *) cur;
                show_interface_descriptor(ifd);

                // Special case: CDC/ACM uses two interfaces (control and data)
                if (ias                     == 1    && // If only 1 interface,
                    ifd->bInterfaceClass    == 0x02 && // of CDC class, and
                    ifd->bInterfaceSubClass == 0x02)   // of ACM subclass,
                    ias = 2;                           // it uses 2 interfaces

                // Find a driver for this interface
                uint8_t i;
                for (i = 0; i < driver_count; i++) {
                    const driver_t *driver = &drivers[i];

                    if (driver->open(dev, ifd)) {
                        debug("Trying the %s driver\n\n", driver->name);

                        // Bind each interface association to the driver
                        // for (uint8_t j = 0; j < ias; j++) {
                        //     uint8_t k = ifd->bInterfaceNumber + j;
                        //     dev->itf2drv[k] = i;
                        // }

                        // Bind all endpoints to the driver
                        // endpoint_bind_driver(dev->ep2drv, ifd, len, i);

                        break;
                    }
                }
                if (i == driver_count) debug("No matching driver found");
            }   break;

            case USB_DT_ENDPOINT:
                epd = (usb_endpoint_descriptor_t *) cur;
                show_endpoint_descriptor(epd);
                next_pipe(dev->dev_addr, epd, NULL); // user_buf starts NULL
                break;

            default:
                debug("Unknown descriptor type: 0x%02x\n\n", type);
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
    ENUMERATION_FINISH,
};

void set_device_address(device_t *dev) {
    debug("Set device address to %u\n", dev->dev_addr);

    control_transfer(dev0, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQUEST_TYPE_STANDARD
                       | USB_REQUEST_RECIPIENT_DEVICE,
        .bRequest      = USB_REQUEST_SET_ADDRESS,
        .wValue        = dev->dev_addr,
        .wIndex        = 0,
        .wLength       = 0,
    }));
}

void set_configuration(device_t *dev, uint16_t cfg) {
    debug("Set configuration to %u\n", cfg);

    control_transfer(dev, &((usb_setup_packet_t) {
        .bmRequestType = USB_DIR_OUT
                       | USB_REQUEST_TYPE_STANDARD
                       | USB_REQUEST_RECIPIENT_DEVICE,
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
            debug("Enumeration started\n");
            step++;

            // Explicit fall through

        case ENUMERATION_GET_MAXSIZE:

            debug("Starting GET_MAXSIZE\n");
            get_device_descriptor(dev);
            break;

        case ENUMERATION_SET_ADDRESS: {
            device_t *dev = next_device();
            dev->state    = dev0->state;
            dev->speed    = dev0->speed;
            dev->maxsize0 = ((usb_device_descriptor_t *) ctrl_buf)
                                ->bMaxPacketSize0;

            debug("Starting SET_ADDRESS\n");
            new_addr = dev->dev_addr;
            set_device_address(dev);
        }   break;

        case ENUMERATION_GET_DEVICE: {
            device_t *dev = get_device(new_addr);
            dev->state    = DEVICE_ADDRESSED;
            clear_device(0);

            debug("Starting GET_DEVICE\n");
            get_device_descriptor(dev);
        }   break;

        case ENUMERATION_GET_CONFIG_SHORT: {
            load_device_descriptor(ctrl_buf, dev);
            show_device_descriptor(ctrl_buf);

            uint8_t len = sizeof(usb_configuration_descriptor_t);
            debug("Starting GET_CONFIG_SHORT (%u bytes)\n", len);
            get_configuration_descriptor(dev, len);
        }   break;

        case ENUMERATION_GET_CONFIG_FULL: {
            uint16_t size =
                ((usb_configuration_descriptor_t *) ctrl_buf)->wTotalLength;
            if (size > MAX_CTRL_BUF)
                panic("Configuration descriptor too large (%u bytes)", size);

            uint8_t len = (uint8_t) size;
            debug("Starting GET_CONFIG_FULL (%u bytes)\n", len);
            get_configuration_descriptor(dev, len);
        }   break;

        case ENUMERATION_SET_CONFIG: {
            enumerate_descriptors(ctrl_buf, dev);
            dev->state = DEVICE_ENUMERATED;
            on_device_enumerated(dev); // Notify that device is enumerated

            debug("Starting SET_CONFIG\n");
            set_configuration(dev, 1);
        }   break;

        case ENUMERATION_FINISH:
            dev->state = DEVICE_CONFIGURED;

            show_string_descriptor_blocking(dev, dev->manufacturer);
            show_string_descriptor_blocking(dev, dev->product     );
            show_string_descriptor_blocking(dev, dev->serial      );

            on_device_configured(dev); // Notify that device is configured
            break;

        default:
            panic("Spurious enumeration request");
            break;
    }
}

// ==[ Callbacks ]==============================================================

SDK_WEAK void on_device_enumerated(device_t *dev) {
    debug("Device %u is now enumerated\n", dev->dev_addr);
}

SDK_WEAK void on_device_configured(device_t *dev) {
    debug("Device %u is now configured\n", dev->dev_addr);

 // activate_drivers(dev);
}

// ==[ Tasks ]==================================================================

SDK_INJECT const char *task_name(uint8_t type) {
    switch (type) {
        case TASK_CALLBACK: return "TASK_CALLBACK";
        case TASK_CONNECT:  return "TASK_CONNECT";
        case TASK_TRANSFER: return "TASK_TRANSFER";
        default:            return "UNKNOWN";
    }
    panic("Unknown task queued");
}

SDK_INJECT const char *callback_name(callback_t fn) {
    if (fn == enumerate        ) return "enumerate"        ;
    if (fn == start_transaction) return "start_transaction";
    if (fn == transfer_zlp     ) return "transfer_zlp"     ;
    return "user defined function";
}

SDK_INLINE void queue_callback(callback_t fn, void *arg) {
    queue_add_blocking(queue, &((task_t) {
        .type         = TASK_CALLBACK,
        .guid         = guid++,
        .fn           = fn,
        .arg          = arg,
    }));
}

void usb_task() {
    task_t task;

    while (queue_try_remove(queue, &task)) {
        uint8_t type = task.type;
        debug("\n=> %u) New task, %s\n\n", task.guid, task_name(type));

        // NOTE: If a task has a callback, it will run after processing the task
        switch (type) {

            case TASK_CALLBACK: {
                debug("Calling %s\n", callback_name(task.fn));
            }   break;

            case TASK_CONNECT: {
                static uint64_t last_attempt;

                if (last_attempt && (time_us_64() - last_attempt < 1000000)) {
                    alert("Connections allowed only once every second\n");
                    break;
                }
                last_attempt = time_us_64();

                clear_device(0);
                dev0->state = DEVICE_DETECTED;
                dev0->speed = task.connect.speed;

                char *str = dev0->speed == LOW_SPEED ? "low" : "full";
                debug("Device connected (%s speed)\n", str);
            }   break;

            case TASK_TRANSFER: {
                uint8_t    status   = task.transfer.status;   // Transfer status
                uint8_t    dev_addr = task.transfer.dev_addr; // Device address
                uint8_t    ep_num   = task.transfer.ep_num;   // Endpoint number
                uint8_t   *user_buf = task.transfer.user_buf; // User buffer
                uint16_t   len      = task.transfer.len;      // Buffer length

                device_t *dev = get_device(dev_addr);
                pipe_t   *pp  = get_pipe(dev_addr, ep_num);

                if (dev->state < DEVICE_CONFIGURED) {
                    len ? transfer_zlp(pp) : enumerate(dev);
                } else if (ep_num) {
                    task.arg = &task.transfer;
                }

            }   break;

            default:
                panic("An unknown task type was queued\n");
                break;
        }

        // Finally, invoke an optional task callback
        if (task.fn) task.fn(task.arg);
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

SDK_INJECT void show_interrupts(uint32_t ints) {
    if (ints & USB_INTS_HOST_CONN_DIS_BITS   ) debug(", device"  );
    if (ints & USB_INTS_STALL_BITS           ) debug(", stall"   );
    if (ints & USB_INTS_BUFF_STATUS_BITS     ) debug(", buffer"  );
    if (ints & USB_INTS_BUFF_STATUS_BITS &&
               usb_hw->buf_status & ~1u      ) debug(", EPn"     );
    if (ints & USB_INTS_TRANS_COMPLETE_BITS  ) debug(", last"    );
    if (ints & USB_INTS_ERROR_RX_TIMEOUT_BITS) debug(", timeout" );
    if (ints & USB_INTS_ERROR_DATA_SEQ_BITS  ) debug(", dataseq" );
    if (ints & USB_INTS_HOST_RESUME_BITS     ) debug(", power"   );
}

// Interrupt handler
void isr_usbctrl() {
    io_rw_32 ints = usb_hw->ints; // Interrupt bits after masking and forcing

    // ==[ Registers and variables related to epx ]==

    io_rw_32 dar  = usb_hw->dev_addr_ctrl;              // dev_addr/ep_num
    io_rw_32 ecr  = usbh_dpram->epx_ctrl;               // Endpoint control
    io_rw_32 bcr  = usbh_dpram->epx_buf_ctrl;           // Buffer control
    io_rw_32 sie  = usb_hw->sie_ctrl;                   // SIE control
    io_rw_32 ssr  = usb_hw->sie_status;                 // SIE status
    io_ro_32 sof  = usb_hw->sof_rd;                     // Frame number
    bool     dbl  = ecr & EP_CTRL_DOUBLE_BUFFERED_BITS; // Double buffered epx?

    // Fix RP2040-E4 by shifting buffer control registers for affected buffers
    if (!dbl && (usb_hw->buf_cpu_should_handle & 1u)) bcr >>= 16;

    // Get device address and endpoint number
    uint8_t dev_addr =  dar & USB_ADDR_ENDP_ADDRESS_BITS;     // 7 bits (lowest)
    uint8_t ep_num   = (dar & USB_ADDR_ENDP_ENDPOINT_BITS) >> // 4 bits (higher)
                              USB_ADDR_ENDP_ENDPOINT_LSB;

    // Get the corresponding pipe
    pipe_t *pp = get_pipe(dev_addr, ep_num);

    // Show system state
    debug("\n=> %u) New ISR", guid++);
    show_interrupts(ints);
    debug("\n\n");
    debug(LOG_ROW);
    debug("│Frame  │ %4u │ %-35s", sof, "Interrupt Handler");
    show_pipe(pp);
    debug(LOG_ROW);
    _bin_("│INT", ints);
    _bin_("│SIE", sie);
    _bin_("│SSR", ssr);
    debug(LOG_ROW);
    _bin_("│DAR", dar);
    _bin_("│ECR", ecr);
    _bin_("│BCR", bcr);
    debug(LOG_ROW);

    // Connection (attach or detach)
    if (ints &   USB_INTS_HOST_CONN_DIS_BITS) {
        ints &= ~USB_INTS_HOST_CONN_DIS_BITS;

        // Get the device speed
        uint8_t speed = (sie &  USB_SIE_STATUS_SPEED_BITS)
                             >> USB_SIE_STATUS_SPEED_LSB;

        // Clear the interrupt
        usb_hw_clear->sie_status = USB_SIE_STATUS_SPEED_BITS;

        // Handle connect and disconnect
        if (speed) {
            debug("│CONNECT│ %-4s │ %-35s │ Task #%-4u │\n", "",
                   "New device connected", guid);

            queue_add_blocking(queue, &((task_t) { // ~20 μs
                .type          = TASK_CONNECT,
                .guid          = guid++,
                .connect.speed = speed,
                .fn            = enumerate,
                .arg           = (void *) dev0,
            }));
        } else {
            debug("│DISCONN│ %-4s │ %-35s │            │\n", "",
                   "Device disconnected");

            clear_device(0);
        }
        debug(LOG_ROW);
    }

    // Stall detected (higher priority than BUFF_STATUS and TRANS_COMPLETE)
    if (ints &   USB_INTS_STALL_BITS) {
        ints &= ~USB_INTS_STALL_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_STALL_REC_BITS;

        panic("Stall detected");
    }

    // Buffer processing is needed
    if (ints &   USB_INTS_BUFF_STATUS_BITS) {
        ints &= ~USB_INTS_BUFF_STATUS_BITS;

        // Find the buffer(s) that are ready
        uint32_t bits = usb_hw->buf_status;
        uint32_t mask = 0b11; // (2 bits at time, IN/OUT transfer together)

        // Show single/double buffer status of epx and which buffers are ready
        _bin_(dbl ? "│2BUF" : "│1BUF", bits);

        // Finish transactions on each pending pipe
        pipe_t *cur;
        for (uint8_t pipe = 0; pipe < MAX_PIPES && bits; pipe++, mask <<= 2) {
            if (bits &   mask) {
                bits &= ~mask;
                usb_hw_clear->buf_status = mask;

                // Get a handle to the correct pipe
                cur = &pipes[pipe];

                // Show registers for endpoint control and buffer control
                char *str = (char[16]) { 0 };
                sprintf(str, "│ECR%u", pipe);
                _bin_(str, *cur->ecr);
                sprintf(str, "│BCR%u", pipe);
                _bin_(str, *cur->bcr);

                finish_transaction(cur);

                cur->bytes_left ? start_transaction(cur) : finish_transfer(cur);
            }
        }
        debug(LOG_ROW);

        // Panic if we missed any buffers
        if (bits) panic("Unhandled buffer mask: %032b", bits);
    }

    // Transfer complete (last packet)
    if (ints &   USB_INTS_TRANS_COMPLETE_BITS) {
        ints &= ~USB_INTS_TRANS_COMPLETE_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_TRANS_COMPLETE_BITS;

        // TODO: Is there anything else we need to do or want to do here?
    }

    // Receive timeout (waited too long without seeing an ACK)
    if (ints &   USB_INTS_ERROR_RX_TIMEOUT_BITS) {
        ints &= ~USB_INTS_ERROR_RX_TIMEOUT_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RX_TIMEOUT_BITS;

        alert("Timeout waiting for data\n");
    }

    // Data error (IN packet from device has wrong data PID)
    if (ints &   USB_INTS_ERROR_DATA_SEQ_BITS) {
        ints &= ~USB_INTS_ERROR_DATA_SEQ_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_DATA_SEQ_ERROR_BITS;

        panic("Data sequence error");
    }

    // Device resumed (device initiated)
    if (ints &   USB_INTS_HOST_RESUME_BITS) {
        ints &= ~USB_INTS_HOST_RESUME_BITS;

        usb_hw_clear->sie_status = USB_SIE_STATUS_RESUME_BITS;

        alert("Device initiated resume\n");
    }

    // Were any interrupts missed?
    if (ints) panic("Unhandled IRQ bitmask 0x%08x", ints);
}

// ==[ Setup USB Host ]=========================================================

void setup_usb_host() {
    debug("USB host reset\n\n");

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
                      | (0xffffffff ^ 0x00000004);       // FIXME: Catches all!

    irq_set_enabled(USBCTRL_IRQ, true);

    clear_devices();
    clear_pipes();
    setup_ctrl();

    debug(LOG_ROW);
    _bin_("│INT", usb_hw->inte);
    debug(LOG_ROW);
}

void usb_init() {
    stdout_uart_init();
    flash("\033[2J\033[H\n==[ PicoUSB Host ]==\n\n");
    queue_init(queue, sizeof(task_t), 64);
    setup_usb_host();
}

// ==[ End ]====================================================================
