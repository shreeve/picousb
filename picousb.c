#include "picousb.h"

void poll_ep1_in(void *arg) {
    endpoint_t *ep = &eps[1];
    uint16_t len = ep->maxsize;

    bulk_transfer(ep, REMOVE_THIS, len);
}

int main() {
    usb_debug(1);
    usb_init();

    while (1) {
        usb_task();

        // FIXME: Poor-man's polling...
        if (devices[1].state == DEVICE_READY) {
            static uint64_t last_attempt = 0;
            if ((time_us_64() - last_attempt) > 1000000) {
                last_attempt = time_us_64();
                queue_add_blocking(queue, &((task_t) {
                    .type         = TASK_CALLBACK,
                    .guid         = guid++,
                    .callback.fn  = poll_ep1_in,
                    .callback.arg = NULL,
                }));
            }
        }
    }
}
