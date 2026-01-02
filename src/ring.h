#ifndef RING_H
#define RING_H

// =============================================================================
// ring.h: Multi-core and IRQ safe ring buffer implementation. Data is an array
//         of uint8_t, so it can be used for any data type.
//
// Author: Steve Shreeve <steve.shreeve@gmail.com>
//   Date: January 2, 2026
//  Legal: Same license as the Pico SDK
// Thanks: btstack_ring_buffer from the Raspberry Pi Pico SDK
// Thanks: rppicomidi has a nice ring buffer implementation
//
// ==[ Capacity Rules ]=========================================================
//
//   - ring_new(N) allocates N bytes and capacity IS N (no wasted byte)
//   - Uses explicit 'used' counter to track bytes (true counted ring)
//   - Empty: used == 0
//   - Full:  used == size
//   - wptr/rptr wrap when they reach size
//
//   NOTE: This is a "counted" ring buffer with an explicit count field.
//   Pointer equality (wptr == rptr) is ambiguous; the 'used' field resolves it.
//
// ==[ Usage Notes ]============================================================
//
// THREAD SAFETY:
//   - Safe for multi-core and multi-producer/multi-consumer use
//   - Lock is held during memcpy (acceptable for small transfers)
//
// IRQ SAFETY:
//   - From ISR: Use ring_try_read() / ring_try_write() ONLY
//   - These are non-sleeping but may spin briefly if another core holds the lock
//   - From main/tasks: Blocking versions are safe
//
// PERFORMANCE:
//   - For large transfers, consider chunking to reduce lock hold time
//   - Typical use: messages < 64 bytes, latency impact minimal
//
// LIMITATIONS (standard for spin-lock designs):
//   - Blocking callers may wake spuriously and re-check
//   - No fairness: under heavy load, writers may starve readers or vice versa
//   - ring_destroy() is undefined if other cores are blocked in ring_*_blocking()
//     (caller must ensure all blocking operations complete before destroying)
//
// =============================================================================

#include <stdint.h>
#include <stdbool.h>
#include "pico/lock_core.h"

typedef struct {
    lock_core_t core;
    uint8_t    *data;     // Buffer of 'size' bytes
    uint16_t    size;     // Capacity in bytes (usable space = size)
    uint16_t    used;     // Bytes currently in buffer (disambiguates full/empty)
    uint16_t    wptr;     // Write index [0..size], wraps after reaching size
    uint16_t    rptr;     // Read index  [0..size], wraps after reaching size
} ring_t;

// ==[ Lifecycle ]==============================================================

ring_t  *ring_new(uint size);
void     ring_init_with_spin_lock(ring_t *r, uint size, uint spin_lock_num);
void     ring_reset(ring_t *r);
void     ring_destroy(ring_t *r);

// ==[ Internal (used by inline wrappers) ]=====================================

uint16_t ring_read_internal(ring_t *r, void *ptr, uint16_t len, bool block);
uint16_t ring_write_internal(ring_t *r, const void *ptr, uint16_t len, bool block);

// ==[ Inline accessors ]=======================================================

// Returns bytes used (no lock - caller must hold lock or accept race)
static inline uint16_t ring_used_unsafe(ring_t *r) {
    return r->used;
}

// Returns bytes free (no lock - caller must hold lock or accept race)
static inline uint16_t ring_free_unsafe(ring_t *r) {
    return r->size - r->used;
}

// Returns bytes used (thread-safe)
static inline uint16_t ring_used(ring_t *r) {
    uint32_t save = spin_lock_blocking(r->core.spin_lock);
    uint16_t used = ring_used_unsafe(r);
    spin_unlock(r->core.spin_lock, save);
    return used;
}

// Returns bytes free (thread-safe)
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

// ==[ Inline read/write wrappers ]=============================================

static inline uint16_t ring_try_read(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, false);
}

static inline uint16_t ring_try_write(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, false);
}

static inline uint16_t ring_read_blocking(ring_t *r, void *ptr, uint16_t len) {
    return ring_read_internal(r, ptr, len, true);
}

static inline uint16_t ring_write_blocking(ring_t *r, const void *ptr, uint16_t len) {
    return ring_write_internal(r, ptr, len, true);
}

// ==[ Debug convenience ]======================================================

uint16_t ring_printf(ring_t *r, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

#endif // RING_H
