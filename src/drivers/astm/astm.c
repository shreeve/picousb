#include "picousb.h"

bool init() {
    return true;
}

bool open(device_t *dev, usb_interface_descriptor_t *ifd) {
    return true;
}

bool config(device_t *dev, uint8_t itf_num) {
    return true;
}

bool transfer(void *transfer) {
    return true;
}

bool close(device_t *dev) {
    return true;
}

bool quit() {
    return true;
}

static driver_t driver = {
    .name     = "ASTM",
    .init     = init,
    .open     = open,
    .config   = config,
    .transfer = transfer,
    .close    = close,
    .quit     = quit,
};

// Register the driver
static void autoregister() __attribute__((constructor)); // Runs before main()
static void autoregister() { register_driver(&driver); }
