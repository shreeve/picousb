#include "picousb.h"

// ==[ Manage timer ticks ]==

volatile uint32_t timer_ticks = 0;

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

// ==[ Application specific ]==

char *buf = (char[1024]) { 0 };

typedef struct {
    pipe_t *in;
    pipe_t *out;
} stream_t;

// return status and chr in buffer
int getchr(uint8_t *c) {
    // TODO: If we are polling and appending to a ring buffer, we can just look
    //       for c immediately.
}

// return status and chr sent
int putchr(uint8_t c) {
    // TODO: To know if c was sent, we need to know if the transfer completed,
    //       which will require some sort of callback when that transfer is
    //       done. How should we do that?
}


void poll_ep1_in(void *arg) {
    pipe_t *pp = &pipes[1];
    uint16_t len = pp->maxsize;

    if (!pp->active) {
        bulk_transfer(pp, buf, len);
    } else {
        debug("Why is EP1/IN still active???\n");
    }
}

void enquire_ep2_out(void *arg) {
    pipe_t *pp = &pipes[2];
    buf[0] = 0x06; // ASTM enquire
    uint16_t len = 1;

    if (!pp->active) {
        bulk_transfer(pp, buf, len);
    } else {
        debug("Why is EP2/OUT still active???\n");
    }
}

void on_device_configured(device_t *dev) {
    printf("STRONG: Device %u is configured\n", dev->dev_addr);

    // Reset FTDI
    if (dev->vid == 0x0403) {
        command(dev, 0x40,  0,  0    , 1, 0); await_transfer(ctrl);
        command(dev, 0x40,  9, 16    , 1, 0); await_transfer(ctrl);
        command(dev, 0x40,  3, 0x4138, 1, 0); await_transfer(ctrl);
        command(dev, 0x40,  1, 0x0303, 1, 0); await_transfer(ctrl);
        command(dev, 0x40,  2, 0x1311, 1, 0); await_transfer(ctrl);
    }

    debug("We're ready!\n");
    dev->state = DEVICE_READY;
}

void piccolo_task() {
    static uint32_t last_ticks = 0;

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

int main() {
    usb_log(LOG_DEBUG);
    usb_init();

    // Create a repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, timer_callback, NULL, &timer);

    while (1) {
        usb_task();
        piccolo_task();
    }
}
