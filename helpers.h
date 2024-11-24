#ifndef PICOUSB_HELPERS_H
#define PICOUSB_HELPERS_H

#include "picousb.h"

extern uint8_t usb_log_level;
#define __printf(...) printf(__VA_ARGS__)
#define log(l, ...) (usb_log_level < (l) ? (void) 0 : __printf(__VA_ARGS__))

#define alert(...) log(1, __VA_ARGS__)
#define flash(...) log(2, __VA_ARGS__)
#define debug(...) log(3, __VA_ARGS__)

void hexdump(const unsigned char *str, const void *data, size_t size, uint mode);
void bindump(uint8_t *str, uint32_t val);
void tobcd(const char *str, uint16_t val);

#endif // PICOUSB_HELPERS_H
