#include "picousb.h"

// ==[ Manage timer ticks ]==

volatile uint32_t timer_ticks = 0;

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

char *buf = (char[1024]) { 0 };

void poll_ep1_in(void *arg) {
    endpoint_t *ep = &eps[1];
    uint16_t len = ep->maxsize;

    bulk_transfer(ep, buf, len);
}

int main() {
    usb_debug(1);
    usb_init();

    // Create a repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(300, timer_callback, NULL, &timer);

    uint32_t last_ticks = 0;

    while (1) {
        usb_task();

        if (devices[1].state == DEVICE_READY) {
            if (last_ticks != timer_ticks) {
                last_ticks  = timer_ticks;
                queue_callback(poll_ep1_in, NULL);
            }
        }
    }
}
