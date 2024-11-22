#ifndef RING_H
#define RING_H

// =============================================================================
// ring.h: Multi-core and IRQ safe ring buffer implementation. Data is an array
//         of uint8_t, so it can be used for any data type.
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: November 20, 2024
//  Legal: Same license as the Pico SDK
// Thanks: btstack_ring_buffer from the Raspberry Pi Pico SDK
// Thanks: rppicomidi has a nice ring buffer implementation
// =============================================================================

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    lock_core_t core;
    uint8_t    *data;
    uint16_t    size;
    uint16_t    wptr;
    uint16_t    rptr;
} ring_t;

ring_t  *ring_new(uint size);
void     ring_init_with_spin_lock(ring_t *r, uint size, uint spin_lock_num);
void     ring_reset(ring_t *r);
void     ring_destroy(ring_t *r);

uint16_t ring_used(ring_t *r);
uint16_t ring_free(ring_t *r);
bool     ring_is_empty(ring_t *r);
bool     ring_is_full(ring_t *r);

uint16_t ring_try_read(ring_t *r, void *ptr, uint16_t len);
uint16_t ring_try_write(ring_t *r, const void *ptr, uint16_t len);

uint16_t ring_read_blocking(ring_t *r, void *ptr, uint16_t len);
uint16_t ring_write_blocking(ring_t *r, const void *ptr, uint16_t len);

// ==[ Debugging: We should remove or improve this ]============================

#define RING_BUFFER_SIZE ((1 << 8) - 1)

extern char ring_buffer[RING_BUFFER_SIZE];

uint16_t ring_printf(ring_t *r, const char *fmt, ...);

#endif // RING_H
