#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/timer.h"

// States
typedef enum {
    NONE,
    IDLE,
    AWAKE,
    WAITING1,
    FRAME_RECEIVED,
    HAVE_DATA,
    DATA_TO_SEND,
    CONTENTION,
    WAITING2,
    NEXT_FRAME_SETUP,
    FRAME_READY,
    WAITING3,
    OLD_FRAME_SETUP,
    INTERRUPT_REQUESTED,
} state_t;

// Tokens
#define STX 0x02
#define ETX 0x03
#define EOT 0x04
#define ENQ 0x05
#define ACK 0x06
#define NAK 0x15
#define CR  0x0D
#define LF  0x0A

// Globals
state_t  state        = IDLE;
state_t  next_state   = NONE;
uint8_t  sequence     = 0;
uint8_t  retries      = 0;
uint8_t  frame_count  = 0;
int32_t  timer_left   = 0;
bool     timer_done   = false;
uint8_t  get_str[255] = { 0 };
uint8_t  put_str[255] = { 0 };

// Structs
struct repeating_timer timer; // TODO: Should this say " = { 0 };" ???

// Timer callback function
bool timer_callback(struct repeating_timer *t) {
    if (--timer_left < 0) {
        timer_done = true;
        if (next_state) {
            state = next_state;
        }
    }
    return true;
}

void set_timeout(int32_t seconds, state_t goto_state) {
    timer_left = seconds;
    timer_done = timer_left < 0;
    next_state = goto_state;
}

void put_chr(uint8_t data) {
    put_chr_raw(data);
}

uint8_t get_chr() {
    return get_chr_timeout_us(1000);
}

// void send_data(uint8_t *data, size_t length) {
//     if (length <= 5) {
//         // Direct UART write for small data
//         for (size_t i = 0; i < length; i++) {
//             uart_putc(uart0, data[i]);
//         }
//     } else {
//         // Use DMA for larger data
//         memcpy(put_str, data, length);
//         dma_channel_set_trans_count(dma_tx_channel, length, false);
//         dma_channel_start(dma_tx_channel);
//         while (dma_channel_is_busy(dma_tx_channel));
//     }
// }
//
// SDK_INLINE void send_byte(uint8_t byte) {
//     send_data((uint8_t *) &byte, 1);
// }

void state_machine() {
    uint8_t chr;

    switch (state) {
        case IDLE:
            if (*put_str) {
                state = DATA_TO_SEND;
            } else if (chr = get_chr()) {
                if (chr == ENQ) {
                    state = AWAKE;
                }
            }
            break;

        // ==[ Receiving device ]===============================================

        case AWAKE:
            if (false) { // TODO: If we're busy
                put_chr(NAK);
                state = IDLE;
            } else {
                put_chr(ACK);
                sequence = 1;
                set_timeout(30, IDLE);
                state = WAITING1;
            }
            break;

        case WAITING1:
            switch (chr = get_chr()) {
                case 0:
                    break;
                case EOT:
                    // TODO: Done receiving frames
                    state = IDLE;
                    break;
                case LF:
                    state = FRAME_RECEIVED;
                    break;
                // TODO: Add bad cases here (send NAK and timeout=30)
                // TODO: Then, default should accept all others
                default:
                    // add to frame...
                    break;
            }
            break;

        case FRAME_RECEIVED:
            if (true) { // TODO: Frame was good
                // TODO: Do something useful with the frame
                if (++sequence > 8) sequence = 1;
                put_chr(ACK);
            } else { // TODO: Frame was bad
                // TODO: Toss out this bad frame
                put_chr(NAK);
            }
            set_timeout(30, IDLE);
            state = WAITING1;
            break;

        case HAVE_DATA:
            if (false) {
                // TODO: Repeat frame
            } else {
                // TODO: New frame
                if (++sequence > 8) sequence = 1;
            }
            put_chr(EOT);
            set_timeout(30, IDLE);
            break;

        // ==[ Sending device ]=================================================

        case DATA_TO_SEND:
            if (false) { // TODO: If we're busy sending
                state = IDLE;
            } else {
                sequence = 1;
                put_chr(ENQ);
                set_timeout(15, WAITING2); // Timeout to same state
                state = WAITING2;
            }
            break;

        case WAITING2:
            if (timer_done) {
                put_chr(EOT);
                state = IDLE;
            } else {
                switch (chr = get_chr()) {
                    case ENQ:
                    case NAK:
                        state = IDLE;
                        break;
                    case ACK:
                        retries = 0;
                        state = FRAME_READY;
                        break;
                    default:
                        break;
                }
            }
            break;

        case FRAME_READY:
            if (!*put_str) {
                put_chr(EOT);
                state = IDLE;
            } else {
                put_str(???); // TODO: Send frame
                set_timeout(15, WAITING3);
                state = WAITING3;
            }
            break;

        case WAITING3:
            if (timer_done) {
                put_chr(EOT);
                state = IDLE;
            } else {
                switch (chr = get_chr()) {
                    case 0:
                        break;
                    case ACK:
                        retries = 0;
                        state = FRAME_READY;
                        break;
                    case EOT:
                        put_chr(EOT);
                        state = INTERRUPT_REQUESTED;
                        break;
                    case NAK:
                        retries++;
                        // NOTE: Allow fall through
                    default:
                        if (retries < 6) {
                            state = FRAME_READY;
                        } else {
                            put_chr(EOT);
                            state = IDLE;
                        }
                        break;
                }
            }
            break;

        case INTERRUPT_REQUESTED:
            if (false) { // TODO: Ignore the interrupt request
                retries = 0;
                if (++sequence > 8) sequence = 1;
                state = FRAME_READY;
            } else { // TODO: Accept the interrupt request
                put_chr(EOT);
                state = IDLE;
            }
            break;

        default:
            break;
    }
}

// Main function
int main() {
    stdio_init_all();
    add_repeating_timer_ms(1000, timer_callback, NULL, &timer); // 1-second timer

    while (true) {
        state_machine();
        sleep_ms(10); // Small delay for CPU efficiency
    }

    return 0;
}
