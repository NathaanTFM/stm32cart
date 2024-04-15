#ifndef KEY1_H
#define KEY1_H

#include <inttypes.h>
#include <stm32g4xx_hal.h>

// sizeof(blowfish_nds)
#define KEYBUF_SIZE (((0x412 * 4 + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

extern const uint32_t keybuf_nds[KEYBUF_SIZE / 4];
extern const uint32_t keybuf_dsi[KEYBUF_SIZE / 4];
extern uint32_t *keybuf_ptr;

void key1_init(uint32_t idcode, const uint32_t *key);
void key1_decrypt_cmd(volatile uint32_t *cmd);

#endif
