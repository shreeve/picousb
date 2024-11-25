#ifndef PICOUSB_HELPERS_H
#define PICOUSB_HELPERS_H

#include "picousb.h"

extern uint8_t usb_log_level;

enum {
    LOG_NEVER, // No logging
    LOG_ALERT, // Warning messages
    LOG_FLASH, // Useful notifications
    LOG_DEBUG, // Lower level debug messages
};

#define log(l,...) (usb_log_level < (l) ? (void) 0 : printf(__VA_ARGS__))
#define alert(...) log(LOG_ALERT, __VA_ARGS__)
#define flash(...) log(LOG_FLASH, __VA_ARGS__)
#define debug(...) log(LOG_DEBUG, __VA_ARGS__)

void _hex_(const unsigned char *str, const void *data, size_t size, uint mode);
void _bin_(uint8_t *str, uint32_t val);
void _bcd_(const char *str, uint16_t val);

#endif // PICOUSB_HELPERS_H
