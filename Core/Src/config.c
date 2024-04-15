#include "config.h"
#include <assert.h>

__attribute__((aligned(FLASH_PAGE_SIZE)))
const card_config_t config = {
	.chip_id = {0xCA, 0xFE, 0x00, 0x00},
	.spi_variant = 1
};

static_assert((sizeof(config) & (FLASH_PAGE_SIZE-1)) == 0);
