// =============================================================================
// ring.c: Multi-core and IRQ safe ring buffer implementation. Data is an array
//         of uint8_t, so it can be used for any data type.
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: February 29, 2024
//  Legal: Same license as the Pico SDK
// Thanks: btstack_ring_buffer from the Raspberry Pi Pico SDK
// Thanks: rppicomidi has a nice ring buffer implementation
//
// ==[ Usage Notes ]============================================================
//
// THREAD SAFETY:
//   - Safe for multi-core and multi-producer/multi-consumer use
//   - Lock is held during memcpy (acceptable for small transfers)
//
// IRQ SAFETY:
//   - From ISR: Use ring_try_read() / ring_try_write() ONLY
//   - From main/tasks: Blocking versions are safe
//
// PERFORMANCE:
//   - For large transfers, consider chunking to reduce lock hold time
//   - Typical use: messages < 64 bytes, latency impact minimal
//
// LIMITATIONS (standard for spin-lock designs):
//   - Blocking callers may wake spuriously and re-check
//   - No fairness: under heavy load, writers may starve readers or vice versa
//
// =============================================================================

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <malloc.h>

#include "pico.h"
#include "pico/assert.h"
#include "pico/lock_core.h"

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

// ==[ Used, Free, Empty, Full ]================================================

// Returns bytes used. When wptr < rptr (wrapped), add size to get correct count.
// Example: size=10, wptr=2, rptr=8 → used = 2-8+10 = 4 bytes (indices 8,9,0,1)
inline uint16_t ring_used_unsafe(ring_t *r) {
    int32_t used = (int32_t) r->wptr - (int32_t) r->rptr;
    if (used < 0) used += r->size;
    return (uint16_t) used;
}

inline uint16_t ring_free_unsafe(ring_t *r) {
    return r->size - ring_used_unsafe(r);
}

inline uint16_t ring_used(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    uint16_t used = ring_used_unsafe(r);
    spin_unlock(r->core.spin_lock, save);
    return used;
}

inline uint16_t ring_free(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    uint16_t free = ring_free_unsafe(r);
    spin_unlock(r->core.spin_lock, save);
    return free;
}

inline bool ring_is_empty(ring_t *r) {
    return ring_used(r) == 0;
}

inline bool ring_is_full(ring_t *r) {
    return ring_free(r) == 0;
}

// ==[ Internal ]===============================================================

static uint16_t
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

static uint16_t
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

// ==[ Read and write ]=========================================================

inline uint16_t ring_try_read(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, false);
}

inline uint16_t ring_try_write(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, false);
}

inline uint16_t ring_read_blocking(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, true);
}

inline uint16_t ring_write_blocking(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, true);
}

// ==[ Debug convenience ]======================================================

#include <stdarg.h>

uint16_t ring_printf(ring_t *r, const char *fmt, ...) {
    char buf[128];  // Stack-allocated, thread-safe
    va_list args;
    va_start(args, fmt);
    uint16_t len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    return ring_write_blocking(r, buf, MIN(len, sizeof(buf)));
}
