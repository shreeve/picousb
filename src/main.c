#include "picousb.h"
#include "ring.h"

// ==[ Handle data ]============================================================

char buf[1024] = { 0 };

typedef struct {
    pipe_t *pipe_in;
    ring_t *ring_in;
    uint8_t size_in;

    pipe_t *pipe_out;
} stream_t;

stream_t s;

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

void poll_ep1_in(void *);
void enquire_ep2_out(void *);

void chaser(void *arg) {
    transfer_t *transfer = (transfer_t *) arg;

    if (transfer->len > 2) {
        uint8_t *tmp = transfer->user_buf + 2, *ptr = tmp;
        uint16_t len = transfer->len      - 2,  pos = 0  ;

        ring_write_blocking(s.ring_in, tmp, len);
        s.size_in += len;

        if (tmp[len - 1] == '\n') {
            ring_read_blocking(s.ring_in, buf, s.size_in);
            printf("%.*s\n", s.size_in - 7, buf + 1);
            s.size_in = 0;
            queue_callback(ep2_out_ack, NULL);
            queue_callback(ep1_in_poll, NULL);
        } else {
            void *chr4 = memchr(transfer->user_buf, 4, transfer->len);
            void *chr5 = memchr(transfer->user_buf, 5, transfer->len);

            if (chr4) {
                printf("[Piccolo is done with that transfer]\n");
            }
            if (chr5) {
                printf("\n[Piccolo wants to speak!]\n");
                queue_callback(ep2_out_ack, NULL);
                queue_callback(ep1_in_poll, NULL);
            }
        }
    }
}

void poll_ep1_in(void *arg) {
    pipe_t *pp = &pipes[1];
    uint16_t len = pp->maxsize;

    pp->fn  = chaser;

    bulk_transfer(pp, buf, len);
}

void enquire_ep2_out(void *arg) {
    pipe_t *pp = &pipes[2];
    buf[0] = 0x06; // ASTM enquire
    uint16_t len = 1;

    bulk_transfer(pp, buf, len);
}

// ==[ Polling ]================================================================

volatile uint32_t timer_ticks = 0;

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

void piccolo_task() {
    static uint32_t last_ticks = 0;

    if (devices[1].state == DEVICE_READY) {
        if (last_ticks != timer_ticks) {
            last_ticks  = timer_ticks;
            if (last_ticks % 3) {
                queue_callback(poll_ep1_in, NULL);
            } else {
                queue_callback(enquire_ep2_out, NULL);
            }
        }
    }
}

// ==[ On configure ]===========================================================

void on_device_configured(device_t *dev) {
    debug("Device %u is configured\n", dev->dev_addr);

    // Reset FTDI
    if (dev->vid == 0x0403) {
        command(dev, 0x40,  0,      0, 1, 0); await_transfer(ctrl); // reset all
        command(dev, 0x40,  9,    200, 1, 0); await_transfer(ctrl); // latency
        command(dev, 0x40,  3, 0x4138, 1, 0); await_transfer(ctrl); // 9600 baud
        command(dev, 0x40,  1, 0x0303, 1, 0); await_transfer(ctrl); // dtr/rts
        command(dev, 0x40,  2, 0x1311, 1, 0); await_transfer(ctrl); // xon/xoff
    }

    dev->state = DEVICE_READY;

    usb_log(LOG_FLASH);

    s.pipe_in  = &pipes[1];
    s.ring_in  = ring_new(1024);
    s.size_in  = 0;

    s.pipe_out = &pipes[2];
}

// ==[ Let 'er rip! ]===========================================================

int main() {
    usb_log(LOG_FLASH);
    usb_init();

    struct repeating_timer timer;
    add_repeating_timer_ms(325, timer_callback, NULL, &timer);

    while (1) {
        usb_task();
        piccolo_task();
    }
}
