// =============================================================================
// ring.c: Implementation for ring.h (see ring.h for documentation)
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 2, 2026
//  Legal: Same license as the Pico SDK
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "pico.h"
#include "pico/assert.h"

#include "ring.h"

// ==[ Init, Reset, Destroy ]===================================================

void ring_init_with_spin_lock(ring_t *r, uint size, uint spin_lock_num) {
    assert(r);
    assert(!r->data);
    lock_init(&r->core, spin_lock_num);
    r->data = (uint8_t *) calloc(size, 1);
    assert(r->data);  // Fail fast on allocation failure
    r->size = (uint16_t) size;
    r->wptr = 0;
    r->rptr = 0;
}

ring_t *ring_new(uint size) {
    ring_t *r = (ring_t *) calloc(sizeof(ring_t), 1);
    assert(r);  // Fail fast on allocation failure
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
    spin_lock_t *lock = r->core.spin_lock;
    uint32_t save = spin_lock_blocking(lock);
    free(r->data);
    free(r);
    spin_unlock(lock, save);
}

// ==[ Internal ]===============================================================

uint16_t
ring_write_internal(ring_t *r, const void *ptr, uint16_t len, bool block) {
    const uint8_t *src = (const uint8_t *) ptr;
    do {
        uint32_t save = spin_lock_blocking(r->core.spin_lock);
        uint16_t cnt = ring_free_unsafe(r);
        if (cnt) {
            len = MIN(len, cnt);
            if (len) {
                uint16_t sip = MIN(r->size - r->wptr, len);
                if (sip < len) {
                    memcpy(r->data + r->wptr, src, sip);
                    memcpy(r->data, src + sip, len - sip);
                    r->wptr = len - sip;
                } else {
                    memcpy(r->data + r->wptr, src, len);
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

uint16_t
ring_read_internal(ring_t *r, void *ptr, uint16_t len, bool block) {
    uint8_t *dst = (uint8_t *) ptr;
    do {
        uint32_t save = spin_lock_blocking(r->core.spin_lock);
        uint16_t cnt = ring_used_unsafe(r);
        if (cnt) {
            len = MIN(len, cnt);
            if (len) {
                uint16_t sip = MIN(r->size - r->rptr, len);
                if (sip < len) {
                    memcpy(dst, r->data + r->rptr, sip);
                    memcpy(dst + sip, r->data, len - sip);
                    r->rptr = len - sip;
                } else {
                    memcpy(dst, r->data + r->rptr, len);
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

// ==[ Debug convenience ]======================================================

#include <stdarg.h>

uint16_t ring_printf(ring_t *r, const char *fmt, ...) {
    char buf[128];  // Stack-allocated, thread-safe
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return ring_write_blocking(r, buf, MIN(len, sizeof(buf) - 1));
}
