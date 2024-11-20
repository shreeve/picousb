#ifndef RING_H
#define RING_H

#include <stdint.h>
#include <stdbool.h>
#include "pico.h"
#include "hardware/sync.h"
#include "pico/assert.h"
#include "pico/lock_core.h"

typedef struct {
    lock_core_t core;
    uint8_t    *data;
    uint16_t    size;
    uint16_t    wptr;
    uint16_t    rptr;
} ring_t;

void ring_init_with_spin_lock(ring_t *r, uint size, uint spin_lock_num);
ring_t *ring_new(uint size);
void ring_reset(ring_t *r);
void ring_destroy(ring_t *r);

uint16_t ring_used(ring_t *r);
uint16_t ring_free(ring_t *r);
bool ring_is_empty(ring_t *r);
bool ring_is_full(ring_t *r);

uint16_t ring_try_write(ring_t *r, const void *ptr, uint16_t len);
uint16_t ring_try_read(ring_t *r, void *ptr, uint16_t len);

uint16_t ring_write_blocking(ring_t *r, const void *ptr, uint16_t len);
uint16_t ring_read_blocking(ring_t *r, void *ptr, uint16_t len);

#define RING_BUFFER_SIZE ((1 << 8) - 1)
uint16_t ring_printf(ring_t *r, const char *fmt, ...);

#endif
