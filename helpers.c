#include "helpers.h"

// Hex dump (mode: 0 = hex; 1 = hex + ascii; 2 = hex + ascii + no newline)
void hexdump(const unsigned char *str, const void *data, size_t size, uint mode) {
    const unsigned char *byte = (const unsigned char *) data;
    size_t i, j;

    for (i = 0; i < size; i += 16) {
        printf("%s\t│ %04x │ ", str, i); // Print the offset

        // Print hex values
        for (j = 0; j < 16; j++) {
            if (i + j < size) {
                printf("%02x ", byte[i + j]);
            } else {
                printf("   "); // Pad if less than 16 bytes in the line
            }
        }

        printf(" │ ");

        // Print ASCII values
        if (mode == 1) {
            for (j = 0; j < 16; j++) {
                if (i + j < size) {
                    unsigned char ch = byte[i + j];
                    printf("%c", (ch >= 32 && ch <= 126) ? ch : '.');
                }
            }
        }

        printf("\n");
    }
}

// Binary dump
void bindump(uint8_t *str, uint32_t val) {
    uint32_t bit = 1 << 31u;
    size_t i;

    printf("%s\t│      │ ", str);

    for (i = 0; i < 32; i++) {
        printf("%c", val & bit ? '1' : '0');
        bit >>= 1u;
        if (i % 8 == 7) printf(" ");
    }

    printf("│ 0x%08x │\n", val);
}

// Print a BCD value
void printb(const char *str, uint16_t val) {
    uint8_t x = (val & 0x0f00) >> 8;
    uint8_t y = (val & 0x00f0) >> 4;
    uint8_t z = (val & 0x000f);

    z ? printf("%s%x.%x.%x\n", str, x, y, z) : printf("%s%x.%x\n", str, x, y);
}
