#include "picousb.h"

char *buf = (char[1024]) { 0 };

void poll_ep1_in(void *arg) {
    endpoint_t *ep = &eps[1];
    uint16_t len = ep->maxsize;

    bulk_transfer(ep, buf, len);
}

int main() {
    usb_debug(1);
    usb_init();

    while (1) {
        usb_task();

        if (devices[1].state == DEVICE_READY) {
            static uint64_t last_attempt = 0;
            if(!last_attempt || ((time_us_64() - last_attempt) > 400000)) {
                last_attempt = time_us_64();
                queue_callback(poll_ep1_in, NULL);
            }
        }
    }
}
