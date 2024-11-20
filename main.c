
#include "picousb.h"
#include "ring.h"

ring_t *rx_ring;
driver_instance_t *cdc_instance;

// ==[ Manage timer ticks ]==

volatile uint32_t timer_ticks = 0;

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

// ==[ Polling with oa static buffer ]==

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

int main() {
    usb_debug(1);
    usb_init();

    rx_ring = ring_new(1024);
    if (!rx_ring) {
        printf("Failed to create ring buffer\n");
        return -1;
    }

    cdc_instance = driver_instance_register("CDC", rx_ring);
    if (!cdc_instance) {
        printf("Failed to register CDC driver instance\n");
        return -1;
    }

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
