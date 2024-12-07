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

// ==[ Polling ]================================================================

enum {
    TIMER_TICK = 10               , // Tick each 10ms
    TIMER_FAST = 10   / TIMER_TICK, // Fast mode checks every 10ms
    TIMER_SLOW = 1000 / TIMER_TICK, // Slow mode checks every 1 second
    TIMER_WAIT = 5000 / TIMER_TICK, // Slow mode begins after 5 seconds inactive
};

volatile uint32_t timer_ticks = 0;
volatile uint32_t timer_check = 0;

void ep1_in_poll(void *);
void ep2_out_ack(void *);

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
            timer_ticks = 0;
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

void ep1_in_poll(void *arg) {
    printf("•");

    pipe_t *pp = s.pipe_in; // &pipes[1];

    if (pp->status == ENDPOINT_STARTED) return;

    uint16_t len = pp->maxsize;

    pp->fn = chaser;
    bulk_transfer(pp, buf, len);
}

void ep2_out_ack(void *arg) {
    printf("≈");

    pipe_t *pp = s.pipe_out; // &pipes[2];

    if (pp->status == ENDPOINT_STARTED) return;

    buf[0] = 0x06; // ASTM ack
    uint16_t len = 1;

    pp->fn = ep1_in_poll;
    bulk_transfer(pp, buf, len);
}

void on_device_configured(device_t *dev) {
    flash("Device %u is configured\n", dev->dev_addr);

    // Reset FTDI
    if (dev->vid == 0x0403) {
        command(dev, 0x40,  0,      0, 1, 0); // reset all
        command(dev, 0x40,  9,     16, 1, 0); // latency
        command(dev, 0x40,  3, 0x4138, 1, 0); // 9600 baud
        command(dev, 0x40,  1, 0x0303, 1, 0); // dtr/rts
        command(dev, 0x40,  2, 0x1311, 1, 0); // xon/xoff
    }

    flash("FTDI device is setup\n");

    dev->state = DEVICE_READY;

    usb_log(LOG_FLASH);

    s.pipe_in  = &pipes[1];
    s.ring_in  = ring_new(1024);
    s.size_in  = 0;

    s.pipe_out = &pipes[2];
}

bool timer_callback(struct repeating_timer *t) {
    timer_ticks++;
    return true; // Keep the timer running
}

void piccolo_task() {
    static uint32_t last_ticks = 0;

    if (last_ticks != timer_ticks) {
        last_ticks  = timer_ticks;
        if (!timer_ticks) {
            timer_check = TIMER_FAST;
        }
    } else {
        return;
    }

    if (devices[1].state < DEVICE_READY)
        return;

    if ((timer_check == TIMER_FAST) && (timer_ticks > TIMER_WAIT)) {
        timer_ticks = 1;
        timer_check = TIMER_SLOW;
    }

    if ((timer_ticks % timer_check) == 0) {
        queue_callback(ep1_in_poll, NULL);
        // printf("%u\t%u\n", timer_ticks, timer_check);
    }
}

int main() {
    usb_log(LOG_FLASH);
    usb_init();

    struct repeating_timer timer;
    add_repeating_timer_ms(TIMER_TICK, timer_callback, NULL, &timer);

    while (1) {
        usb_task();
        piccolo_task();
    }
}
