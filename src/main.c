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
            usb_log(LOG_ALERT);
            printf("FTDI reset complete\n");
            break;
    }
}

void poll_ep1_in(void *arg) {
    pipe_t *pp = &pipes[1];
    uint16_t len = pp->maxsize;

    bulk_transfer(pp, buf, len);
}

void enquire_ep2_out(void *arg) {
    pipe_t *pp = &pipes[2];
    buf[0] = 0x06; // ASTM enquire
    uint16_t len = 1;

    bulk_transfer(pp, buf, len);
}

int main() {
    usb_log(LOG_DEBUG);
    usb_init();

    // Create a repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, timer_callback, NULL, &timer);
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
