#ifndef KEY1_H
#define KEY1_H

// FIXME: this address is an ugly magic constant
#define KEYBUF_ADDR (0x0801E000)

const extern uint32_t *const keybuf;
extern uint32_t *keybuf_ptr;

void key1_init(uint32_t idcode);
void key1_decrypt_cmd(volatile uint32_t *cmd);

#endif
