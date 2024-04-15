#ifndef CONFIG_H
#define CONFIG_H

#include <stdint.h>
#include <stm32g4xx_ll_bus.h>

#define CONFIG_SIZE (((sizeof(card_config_t) + FLASH_PAGE_SIZE - 1) / FLASH_PAGE_SIZE) * FLASH_PAGE_SIZE)

typedef struct {
	// ROM config
	/*     0h */ uint8_t chip_id[4];

	// SPI (Backup)
	/*     4h */ int spi_variant;
	/* 	   8h */ uint8_t spi_data[0x10000]; // 64K

	/* 10008h */ uint8_t _padding[0x7F8]; // Pad to page size

} card_config_t;

extern const card_config_t config;

#endif
