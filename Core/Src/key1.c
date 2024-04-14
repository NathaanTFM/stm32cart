#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "key1.h"
#include "key1buf.h"

#define bswap_32bit(x) __builtin_bswap32(x)

#define NO_INIT __attribute__ ((section(".noinit")))
static NO_INIT uint32_t keycode[3];
//uint32_t __attribute__((section(".keybuf"))) keybuf[0x412];
const uint32_t *const keybuf = (const uint32_t *)KEYBUF_ADDR;
uint32_t *keybuf_ptr;

static inline void encrypt_64bit(uint32_t* b0, uint32_t* b1) {
    uint32_t y = *b0;
    uint32_t x = *b1;

    for (int i = 0; i < 0x10; i++) {
        uint32_t z = keybuf_ptr[i] ^ x;
        x = keybuf_ptr[0x12 + ((z >> 24) & 0xFF)];
        x += keybuf_ptr[0x112 + ((z >> 16) & 0xFF)];
        x ^= keybuf_ptr[0x212 + ((z >> 8) & 0xFF)];
        x += keybuf_ptr[0x312 + (z & 0xFF)];
        x ^= y;
        y = z;
    }

    *b0 = x ^ keybuf_ptr[0x10];
    *b1 = y ^ keybuf_ptr[0x11];
}

static inline void decrypt_64bit(uint32_t* b0, uint32_t* b1) {
    uint32_t y = *b0;
    uint32_t x = *b1;

    for (int i = 0x11; i > 0x01; i--) {
        uint32_t z = keybuf[i] ^ x;
        x = keybuf[0x12 + ((z >> 24) & 0xFF)];
        x += keybuf[0x112 + ((z >> 16) & 0xFF)];
        x ^= keybuf[0x212 + ((z >> 8) & 0xFF)];
        x += keybuf[0x312 + (z & 0xFF)];
        x ^= y;
        y = z;
    }

    *b0 = x ^ keybuf[0x01];
    *b1 = y ^ keybuf[0x00];
}

static void apply_keycode() {
    encrypt_64bit(&keycode[1], &keycode[2]);
    encrypt_64bit(&keycode[0], &keycode[1]);

    uint32_t scratch[2] = {0, 0};
    for (int i = 0; i < 0x12; i++) {
    	keybuf_ptr[i] ^= bswap_32bit(keycode[i & 1]);
    }

    for (int i = 0; i < 0x412; i += 2) {
        encrypt_64bit(&scratch[0], &scratch[1]);
        keybuf_ptr[i] = scratch[1];
        keybuf_ptr[i+1] = scratch[0];
    }
}

void key1_init(uint32_t idcode) {
	memcpy(keybuf_ptr, ninkeybuf, 0x412 * sizeof(uint32_t));

    keycode[0] = idcode;
    keycode[1] = idcode / 2;
    keycode[2] = (idcode * 2);

    apply_keycode();
    apply_keycode();
}

void key1_decrypt_cmd(volatile uint32_t *cmd) {
    uint32_t b0 = bswap_32bit(cmd[1]);
    uint32_t b1 = bswap_32bit(cmd[0]);
    decrypt_64bit(&b0, &b1);
    // TODO: remove those to save time?
    cmd[1] = b0; //bswap_32bit(b0);
    cmd[0] = b1; //bswap_32bit(b1);
}
