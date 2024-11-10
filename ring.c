// =============================================================================
// ring.c: Multi-core and IRQ safe ring buffer implementation. Data is an array
//         of uint8_t, so it can be used for any data type.
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: February 29, 2024
//  Legal: Same license as the Pico SDK
// Thanks: btstack_ring_buffer from the Raspberry Pi Pico SDK
// Thanks: rppicomidi has a nice ring buffer implementation
// =============================================================================

#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "pico.h"
#include "hardware/sync.h"
#include "pico/assert.h"
#include "pico/lock_core.h"

// ==[ Init, Reset, Destroy ]===================================================

typedef struct {
    lock_core_t core;
    uint8_t    *data;
    uint16_t    size;
    uint16_t    wptr;
    uint16_t    rptr;
} ring_t;

void ring_init_with_spin_lock(ring_t *r, uint size, uint spin_lock_num) {
    assert(r);
    assert(!r->data);
    lock_init(&r->core, spin_lock_num);
    r->data = (uint8_t *) calloc(size, 1);
    r->size = (uint16_t) size;
    r->wptr = 0;
    r->rptr = 0;
}

ring_t *ring_new(uint size) {
    ring_t *r = (ring_t *) calloc(sizeof(ring_t), 1);
    ring_init_with_spin_lock(r, size, next_striped_spin_lock_num());
    return r;
}

void ring_reset(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    r->wptr = 0;
    r->rptr = 0;
    spin_unlock(r->core.spin_lock, save);
}

void ring_destroy(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    free(r->data);
    free(r);
    spin_unlock(r->core.spin_lock, save);
}

// ==[ Used, Free, Empty, Full ]================================================

static inline uint16_t ring_used_unsafe(ring_t *r) {
    int32_t used = (int32_t) r->wptr - (int32_t) r->rptr;
    if (used < 0) used += r->size + 1;
    return (uint16_t) used;
}

static inline uint16_t ring_free_unsafe(ring_t *r) {
    return r->size - ring_used_unsafe(r);
}

static inline uint16_t ring_used(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    uint16_t used = ring_used_unsafe(r);
    spin_unlock(r->core.spin_lock, save);
    return used;
}

static inline uint16_t ring_free(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    uint16_t free = ring_free_unsafe(r);
    spin_unlock(r->core.spin_lock, save);
    return free;
}

static inline bool ring_is_empty(ring_t *r) {
    return ring_used(r) == 0;
}

static inline bool ring_is_full(ring_t *r) {
    return ring_free(r) == 0;
}

// ==[ Internal ]===============================================================

static uint16_t
ring_write_internal(ring_t *r, const void *ptr, uint16_t len, bool block) {
    do {
        uint32_t save = spin_lock_blocking(r->core.spin_lock);
        uint16_t cnt = ring_free_unsafe(r);
        if (cnt) {
            if (len = MIN(len, cnt)) {
                uint16_t sip = MIN(r->size - r->wptr, len);
                if (sip < len) {
                    memcpy(r->data + r->wptr, ptr, sip);
                    memcpy(r->data, ptr + sip, len - sip);
                    r->wptr = len - sip;
                } else {
                    memcpy(r->data + r->wptr, ptr, len);
                    r->wptr += len;
                }
            }
            lock_internal_spin_unlock_with_notify(&r->core, save);
            return len;
        }
        if (block) {
            lock_internal_spin_unlock_with_wait(&r->core, save);
        } else {
            spin_unlock(r->core.spin_lock, save);
            return 0;
        }
    } while (true);
}

static uint16_t
ring_read_internal(ring_t *r, const void *ptr, uint16_t len, bool block) {
    do {
        uint32_t save = spin_lock_blocking(r->core.spin_lock);
        uint16_t cnt = ring_used_unsafe(r);
        if (cnt) {
            if (len = MIN(len, cnt)) {
                uint16_t sip = MIN(r->size - r->rptr, len);
                if (sip < len) {
                    memcpy((void *) ptr, r->data + r->rptr, sip);
                    memcpy((void *) ptr + sip, r->data, len - sip);
                    r->rptr = len - sip;
                } else {
                    memcpy((void *) ptr, r->data + r->rptr, len);
                    r->rptr += len;
                }
            }
            lock_internal_spin_unlock_with_notify(&r->core, save);
            return len;
        }
        if (block) {
            lock_internal_spin_unlock_with_wait(&r->core, save);
        } else {
            spin_unlock(r->core.spin_lock, save);
            return 0;
        }
    } while (true);
}

// ==[ Try ]====================================================================

uint16_t ring_try_write(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, false);
}

uint16_t ring_try_read(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, false);
}

// ==[ Blocking ]===============================================================

uint16_t ring_write_blocking(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, true);
}

uint16_t ring_read_blocking(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, true);
}

// ==[ String printing ]========================================================

#if 1

#include <stdarg.h>

#define RING_BUFFER_SIZE ((1 << 8) - 1)

static char ring_buffer[RING_BUFFER_SIZE];

uint16_t ring_printf(ring_t *r, const char *fmt, ...) {
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf(ring_buffer, RING_BUFFER_SIZE, fmt, args);
    va_end(args);
    return ring_write_blocking(r, ring_buffer, len);
}

#endif

// =============================================================================
