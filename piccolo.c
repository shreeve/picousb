#include "picousb.h"

// ==[ Manage timer ticks ]==

volatile uint32_t timer_ticks = 0;

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

// ==[ Polling with a static buffer ]==

char *buf = (char[1024]) { 0 };

void poll_ep1_in(void *arg) {
    endpoint_t *ep = &eps[1];
    uint16_t len = ep->maxsize;

    bulk_transfer(ep, buf, len);
}

void enquire_ep2_out(void *arg) {
    endpoint_t *ep = &eps[2];
    buf[0] = 0x06; // ASTM enquire
    uint16_t len = 1;

    bulk_transfer(ep, buf, len);
}

void hello() {
    printf("STRONG function!!!\n");
}

int main() {
    usb_debug(0);
    usb_init();

    // Create a repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(2000, timer_callback, NULL, &timer);

    uint32_t last_ticks = 0;

    while (1) {
        usb_task();

        if (devices[1].state == DEVICE_READY) {
            if (last_ticks != timer_ticks) {
                last_ticks  = timer_ticks;
                if (last_ticks % 10) {
                    queue_callback(poll_ep1_in, NULL);
                } else {
                    queue_callback(enquire_ep2_out, NULL);
                }
            }
        }
    }
}
