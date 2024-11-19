#ifndef PICOUSB_HELPERS_H
#define PICOUSB_HELPERS_H

#include "picousb.h"

extern uint8_t usb_debug_level;
#define __real_printf(...) printf(__VA_ARGS__)
#define printf(...) (usb_debug_level ? __real_printf(__VA_ARGS__) : (void) 0)

void hexdump(const unsigned char *str, const void *data, size_t size, uint mode);
void bindump(uint8_t *str, uint32_t val);
void printb(const char *str, uint16_t val);

#endif // PICOUSB_HELPERS_H
