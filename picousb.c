#include "picousb.h"

void poll_ep1_in(void *arg) {
    endpoint_t *ep = &eps[1];
    uint16_t len = ep->maxsize;

    bulk_transfer(ep, REMOVE_THIS, len);
}

bool poll_epx(repeating_timer_t *timer) {
    uint8_t dev_addr = (uint32_t)(uintptr_t) timer->user_data;

    if (devices[dev_addr].state == DEVICE_READY) {
        queue_add_blocking(queue, &((task_t) {
            .type         = TASK_CALLBACK,
            .guid         = guid++,
            .callback.fn  = poll_ep1_in,
            .callback.arg = NULL,
        }));
    }

    return true;
}

int main() {
    usb_debug = 1;

    usb_init();

    // Hard code a 0.5 sec polling interval
    repeating_timer_t timer;
    void *user_data = (void *) 1; // dev_addr
    add_repeating_timer_us(500000, poll_epx, user_data, &timer);

    while (1) {
        usb_task();
    }
}
