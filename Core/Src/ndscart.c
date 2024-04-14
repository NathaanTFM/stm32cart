#include "main.h"
#include "key1.h"

#include <string.h>
#include <stdio.h>
#include <setjmp.h>

#include <stm32g4xx_ll_bus.h>

// define this for WIP 1000h bytes transfer support
#define TRANSFER_1000H
#define ENABLE_SPI2

static jmp_buf jmp;
void *stackPointer;

#define CLK_Pin (1<<12)
#define UNUSED_Pin (1<<13)
#define CS1_Pin (1<<14)
#define CS2_Pin (1<<15)

// NEXT_DATA macro
#define NEXT_DATA() (*(data++))

#define ALIGN(n) __attribute__ ((aligned (n)))
#define NO_INIT __attribute__ ((section(".noinit")))

// transfer buffer backup
static volatile uint16_t currentState = 0;

static ALIGN(4) const uint8_t chipId[4] = {0xAB, 0xCD, 0xEF, 0x00};

#define DATA_SPI1 (*(volatile uint8_t *)&SPI1->DR)
#define DATA_OUT (*(volatile uint8_t *)&GPIOA->ODR)

// data input/output buffer
// first command is 2000h bytes long
#define TRANSFER_BUFFER_SIZE (0x2200)

// NDS      c0  c1  c2  c3  c4  c5  c6  c7  d0  d1
// 		     ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓   ↓
//    CLK ‾‾‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾|_|‾‾‾‾
//             ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑   ↑
// GPIO TX    b0  b1  b2  b3  b4  b5  b6  b7  b8  b9

#define COMMAND_TRANSFER_DELTA (0)
#define DATA_TRANSFER_DELTA (17) // we write at b7 for d0 (see above), so it's 8-7=1 to have our dword alignment

// transferBuffer:
// 0
// ...
// COMMAND_TRANSFER_DELTA
// ...
// COMMAND_TRANSFER_DELTA + 8
// ...
// DATA_TRANSFER_DELTA
// ...
// DATA_TRANSFER_DELTA+TRANSFER_BUFFER_SIZE

static ALIGN(4) NO_INIT volatile uint8_t transferBufferInternal[TRANSFER_BUFFER_SIZE + DATA_TRANSFER_DELTA + 8]; // Adding 8 bytes because data and command could overlap
static volatile uint8_t * const transferBuffer = transferBufferInternal + DATA_TRANSFER_DELTA - 1 + 8; // 4-bytes aligned (because key2 is aligned)
static volatile uint8_t * const command = transferBufferInternal + COMMAND_TRANSFER_DELTA; // 4-bytes aligned (because key2 is aligned)
#define COMMAND_HI *(volatile uint32_t*)(transferBufferInternal)
#define COMMAND_LO *(volatile uint32_t*)(transferBufferInternal+4)

// macros for transfers
//#define COMMAND_BUF_SIZE (8)
#define COMMAND_BUF_SIZE (sizeof(transferBufferInternal) - COMMAND_TRANSFER_DELTA)

#define WAIT_FOR_BYTE_NO_CS(idx) \
	while (DMA1_Channel1->CNDTR >= (COMMAND_BUF_SIZE - (idx)));

#define WAIT_FOR_BYTE(idx) \
	while (DMA1_Channel1->CNDTR >= (COMMAND_BUF_SIZE - (idx)));

#define COMMAND_PROGRESS() \
	(COMMAND_BUF_SIZE - DMA1_Channel1->CNDTR)

#define DATA_PROGRESS() \
	((int32_t)((COMMAND_BUF_SIZE - 8) - DMA1_Channel1->CNDTR))

#define SEED_BUFFER_SIZE (0x4000)
#define SEED_MASK (SEED_BUFFER_SIZE-1)
static volatile ALIGN(0x4) NO_INIT uint8_t seedBuffer[SEED_BUFFER_SIZE];
static volatile uint16_t seedPosition = 0; // next starting position
static volatile int16_t seedAvailable = 0; // available bytes

#define KEY2_BEGIN() do { uint16_t _spos = (seedPosition); ((void)0)
#define KEY2_AT(x) (*(volatile uint8_t*)(&seedBuffer[(_spos + x) & SEED_MASK]))
#define KEY2_AT_32(x) (*(volatile uint32_t*)(&seedBuffer[(_spos + x) & SEED_MASK]))
#define KEY2_END() } while (0)

// key2 encryption variables
static NO_INIT volatile uint16_t gpioModeBuffer[8];
static NO_INIT volatile uint32_t dmamuxGeneratorEnable[2];
static NO_INIT volatile uint32_t dmamuxGeneratorDisable[2];

static volatile int cs1Enabled = 0, cs2Enabled = 0;

static ALIGN(4) NO_INIT volatile uint8_t spiBuffer[0x40];

#define SPI_AVAILABLE(sz) (int32_t)(sz - DMA2_Channel2->CNDTR)
//#define IS_SPI_AVAILABLE(sz, nb) ((uint32_t)DMA2_Channel2->CNDTR < (uint32_t)(sz-nb))
#define IS_SPI_AVAILABLE(sz, nb) (SPI_AVAILABLE(sz) > nb)
#define SPI_WAIT(sz, nb) while (!IS_SPI_AVAILABLE(sz, nb)); // wait for index to be available

// Those were inside jokes with friends. Please forget about them. I'm keeping them because I don't want to rebuild my 3DS homebrew
static const uint8_t flash_modecmd[8] = {'A','M','O','G','U','S',0,0};
static const uint8_t flash_flashcmd[8] = {'C','R','A','M','P','T','E','S'};
static const uint8_t flash_erasecmd[8] = {'A','P','A','G','N','A','N','!'};
static const uint8_t flash_pollcmd[8] = {'Q','C','O','U','B','E','H','?'};
static const uint8_t flash_debugcmd[8] = {'B','E','B','O','U','?','?','?'};

#ifdef ENABLE_SPI2
static volatile uint32_t spi2cr[2];
static volatile uint32_t gpiof_moder;

const uint8_t save[0x8000];

#endif

/* KEY 2 */
struct hilo {
    uint32_t hi;
    uint32_t lo;
};

struct key2 {
	struct hilo x;
	struct hilo y;
};

static volatile struct key2 key2;

static inline void setKey2Seed(volatile struct hilo *key2, uint32_t hi, uint32_t lo) {
	uint32_t tmp = __RBIT(lo);

    key2->hi = tmp >> 25;
    key2->lo = (tmp << 7) | (__RBIT(hi) >> 25);
}

static inline void initKey2Default(void) {
    setKey2Seed(&key2.x, 0x58, 0xC56DE0E8);
    setKey2Seed(&key2.y, 0x5C, 0x879B9B05);
}

static inline void initKey2(uint32_t value, uint32_t seed) {
	setKey2Seed(&key2.x, (value >> 17), (value << 15) | 0x6000 | seed);
    setKey2Seed(&key2.y, 0x5C, 0x879B9B05);
}


/* FAULT HANDLER */
void fault_handler(uint32_t magic) {
	__disable_irq();

	// Reset GPIOA (might allow us to reconnect)
	__HAL_RCC_GPIOA_FORCE_RESET();
	__NOP(); __NOP(); __NOP();
	__HAL_RCC_GPIOA_RELEASE_RESET();

	for (;;);
}

static void delay(uint32_t count) {
	volatile uint32_t x = count;
	while (x > 0)
		--x;
}

static void memcpy_v(volatile void *dest, volatile const void *src, size_t n) {
    volatile const char *src_c = src;
    volatile char *dest_c = dest;
    for (size_t i = 0; i < n; i++)
        dest_c[i] = src_c[i];
}

static int memcmp_v(volatile const void *buf1, volatile const void *buf2, size_t n) {
	volatile const char *buf1_c = buf1;
	volatile const char *buf2_c = buf2;

	for (size_t i = 0; i < n; i++) {
		if (buf1_c[i] != buf2_c[i])
			return 1; // maybe the opposite, i don't care!
	}
	return 0;
}

static inline void spiSelect() {
	GPIOB->BRR = (1<<6); // reset pin, drive it low
}

static inline void spiUnselect() {
	GPIOB->BSRR = (1<<6); // set pin, drive it high
}

static inline void spiStop() {
	// Disable SPI DMA
	DMA2_Channel1->CCR &= ~(DMA_CCR_EN | DMA_CCR_CIRC);
	DMA2_Channel2->CCR &= ~DMA_CCR_EN;

	if (SPI1->CR1 & SPI_CR1_SPE) {
		// Disable SPI DMA Request
		SPI1->CR2 &= ~(SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

		// Wait for FTLVL=0 and BSY=0
		while (SPI1->SR & (SPI_SR_FTLVL | SPI_SR_BSY));

		SPI1->CR1 &= ~SPI_CR1_SPE;

		// Wait for FRLVL=0
		while (SPI1->SR & SPI_SR_FRLVL) {
			volatile uint8_t tmpreg8 = *(__IO uint8_t *)&SPI1->DR;
			UNUSED(tmpreg8);
		}
	}

	spiUnselect();
}

static inline void spiPrepareEnable() {
	// Disable SPI DMA
	DMA2_Channel1->CCR &= ~(DMA_CCR_EN | DMA_CCR_CIRC);
	DMA2_Channel2->CCR &= ~DMA_CCR_EN;

	// Enable SPI
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}

static inline void spiInit() {
	SPI1->CR1 = (
		  (0 << SPI_CR1_BIDIMODE_Pos) // 2-line unidirectional data mode
		| (0 << SPI_CR1_CRCEN_Pos) // crc calculation disabled
		| (0 << SPI_CR1_RXONLY_Pos) // full-duplex
		| (1 << SPI_CR1_SSM_Pos) // software slave management enabled
		| (1 << SPI_CR1_SSI_Pos) // internal slave select enabled
		| (0 << SPI_CR1_LSBFIRST_Pos) // msb first
		| (0 << SPI_CR1_SPE_Pos) // peripheral disabled
		| (0 << SPI_CR1_BR_Pos) // fpclk/2
		| (1 << SPI_CR1_MSTR_Pos) // master configuration
		| (0 << SPI_CR1_CPOL_Pos) // clock to 0 when idle
		| (0 << SPI_CR1_CPHA_Pos) // first clock transition is data capture edge
	);

	SPI1->CR2 = (
		  (0 << SPI_CR2_LDMATX_Pos) // not an odd number
		| (0 << SPI_CR2_LDMARX_Pos) // not an odd number
		| (1 << SPI_CR2_FRXTH_Pos) // 8-bit event
		| (7 << SPI_CR2_DS_Pos) // 8-bit data size
		| (0 << SPI_CR2_TXEIE_Pos) // TXE interrupt masked
		| (0 << SPI_CR2_RXNEIE_Pos) // RXNE interrupt masked
		| (0 << SPI_CR2_ERRIE_Pos) // error interrupt maked
		| (0 << SPI_CR2_FRF_Pos) // SPI Motorola mode
		| (0 << SPI_CR2_NSSP_Pos) // no nss pulse
		| (0 << SPI_CR2_SSOE_Pos) // SS output disable
		| (0 << SPI_CR2_TXDMAEN_Pos) // Tx buffer DMA disabled
		| (0 << SPI_CR2_RXDMAEN_Pos) // Rx buffer DMA disabled
	);
}

static inline void spiInitDma() {
	// Initialize DMA for SPI
	DMA2_Channel1->CPAR = (uint32_t)&SPI1->DR;
	DMA2_Channel1->CMAR = (uint32_t)spiBuffer;
	DMA2_Channel1->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (2 << DMA_CCR_PL_Pos) // high priority
		| (0 << DMA_CCR_MSIZE_Pos) // 8 bits in memory
		| (0 << DMA_CCR_PSIZE_Pos) // 8 bits in peripheral (SPI1)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (0 << DMA_CCR_PINC_Pos) // do not increment peripheral
		| (0 << DMA_CCR_CIRC_Pos) // circular mode!
		| (1 << DMA_CCR_DIR_Pos) // read from memory
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel6->CCR = (
		(11 << DMAMUX_CxCR_DMAREQ_ID_Pos) // SPI1 TX
	);

	DMA2_Channel2->CPAR = (uint32_t)&SPI1->DR;
	DMA2_Channel2->CMAR = (uint32_t)spiBuffer;
	DMA2_Channel2->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (2 << DMA_CCR_PL_Pos) // high priority
		| (0 << DMA_CCR_MSIZE_Pos) // 8 bits in memory
		| (0 << DMA_CCR_PSIZE_Pos) // 8 bits in peripheral (SPI1)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (0 << DMA_CCR_PINC_Pos) // do not increment peripheral
		| (0 << DMA_CCR_CIRC_Pos) // no circular mode
		| (0 << DMA_CCR_DIR_Pos) // read from peripheral
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel7->CCR = (
		(10 << DMAMUX_CxCR_DMAREQ_ID_Pos) // SPI1 RX
	);
}

static inline void spiAbort() {
	spiUnselect();

	// Disable SPI DMA
	DMA2_Channel1->CCR &= ~(DMA_CCR_EN | DMA_CCR_CIRC);
	DMA2_Channel2->CCR &= ~DMA_CCR_EN;

	// Reset SPI
	__HAL_RCC_SPI1_FORCE_RESET();
	__HAL_RCC_SPI1_RELEASE_RESET();

	// Re-init SPI
	spiInit();

	// We need to enable it now (spiPrepareEnable)
	SPI1->CR1 |= SPI_CR1_SPE;
	SPI1->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
}

// high-level for spi commands
static void spiCommand(uint8_t *txbuf, uint16_t txlen, uint8_t *rxbuf, uint16_t rxlen) {
	// enable spi
	spiSelect();

	SPI1->CR1 |= SPI_CR1_SPE;

	// write txlen bytes from txbuf
	while (txlen) {
		while (!(SPI1->SR & SPI_SR_TXE));
		*(__IO uint8_t *)&SPI1->DR = *(txbuf++);

		while (!(SPI1->SR & SPI_SR_RXNE));
		volatile uint8_t x = *(__IO uint8_t *)&SPI1->DR;
		UNUSED(x);

		txlen--;
	}

	// read rxlen bytes into rxbuf
	while (rxlen) {
		while (!(SPI1->SR & SPI_SR_TXE));
		*(__IO uint8_t *)&SPI1->DR = 0; // dummy write

		while (!(SPI1->SR & SPI_SR_RXNE));
		*(rxbuf++) = *(__IO uint8_t *)&SPI1->DR;

		rxlen--;
	}

	spiStop();
}

static inline void spiCommandDMA(volatile uint8_t *txbuf, volatile uint8_t *rxbuf, uint16_t len) {
	spiSelect();

	// set addresses of DMA channels
	DMA2_Channel1->CMAR = (uint32_t)txbuf;
	DMA2_Channel2->CMAR = (uint32_t)rxbuf;

	// set lengths of dma channels
	DMA2_Channel1->CNDTR = len;
	DMA2_Channel2->CNDTR = len;

	// enable dma channels
	DMA2_Channel1->CCR |= DMA_CCR_EN;
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	// enable spi
	SPI1->CR1 |= SPI_CR1_SPE;

	// enable events
	SPI1->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);
	//SPI1->CR2 |= SPI_CR2_RXDMAEN;

	while (DMA2_Channel2->CNDTR != 0);
	spiStop();
}

static inline volatile uint8_t * spiDataRead(uint32_t address, uint16_t size, volatile uint8_t *dst) {
	// This borrows the transferBuffer!
	spiSelect();

	// set address of dma channel
	DMA2_Channel1->CMAR = (uint32_t)spiBuffer;
	DMA2_Channel2->CMAR = (uint32_t)dst;

	spiBuffer[0] = 0x0B; // read data
	spiBuffer[1] = 0; // dummy byte
	spiBuffer[2] = address >> 16; // 24-bit address
	spiBuffer[3] = address >> 8; // 24-bit address
	spiBuffer[4] = address >> 0; // 24-bit address

	// set lengths of dma channels
	DMA2_Channel1->CNDTR = 4+size;
	DMA2_Channel2->CNDTR = 4+size;

	// enable spi
	SPI1->CR1 |= SPI_CR1_SPE;
	//SPI1->CR2 |= SPI_CR2_RXDMAEN;

	// enable events
	SPI1->CR2 |= (SPI_CR2_TXDMAEN | SPI_CR2_RXDMAEN);

	// enable dma channels
	DMA2_Channel1->CCR |= DMA_CCR_EN;
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	//while (DMA2_Channel2->CNDTR >= 0x203);

	// N.B: make sure it's aligned (change spiBuffer in DMA2_Channel2->CMAR if it's not)
	// UPDATE: it might not be and we have no control over it - this method is used for general cases
	return &dst[5];
}

static void spiWriteEnable() {
	uint8_t cmd[1] = {0x06};
	spiCommand(cmd, 1, NULL, 0);
}

static void spiWriteDisable() {
	uint8_t cmd[1] = {0x04};
	spiCommand(cmd, 1, NULL, 0);
}

static uint8_t spiReadStatus() {
	uint8_t cmd[1] = {0x05};
	uint8_t byte[1];
	spiCommand(cmd, 1, byte, 1);
	return byte[0];
}

static void spiProgramPage(uint32_t address, volatile uint8_t *data) {
	while (spiReadStatus() & 1) delay(5000);
	spiWriteEnable();

	// useless memcpy, need to work on spiCommand
	// i'll let it here for now
	static NO_INIT volatile uint8_t cmd[0x104];
	memcpy_v(cmd, (volatile uint8_t[]){0x02, address >> 16, address >> 8, address >> 0}, 4);
	memcpy_v(cmd+4, data, 0x100);
	spiCommandDMA(cmd, cmd, 0x104);

	while (spiReadStatus() & 1) delay(5000);
	spiWriteDisable();
}

static void spiEraseSector(uint32_t address) {
	while (spiReadStatus() & 1) delay(5000);
	spiWriteEnable();

	uint8_t cmd[4] = {0x20, address >> 16, address >> 8, address >> 0};
	spiCommand(cmd, 4, NULL, 0);

	while (spiReadStatus() & 1) delay(5000);
	spiWriteDisable();
}

#ifdef ENABLE_SPI2

static inline void spi2Init() {
	// Initialize SPI2
	SPI2->CR1 = (
		(0 << SPI_CR1_BIDIMODE_Pos) /* 2-line unidirectional data mode*/
		| (0 << SPI_CR1_CRCEN_Pos) /* crc calculation disabled*/
		| (0 << SPI_CR1_RXONLY_Pos) /* full-duplex */
		| (1 << SPI_CR1_SSM_Pos) /* software slave management enabled */
		| (0 << SPI_CR1_SSI_Pos) /* internal slave select enabled*/
		| (0 << SPI_CR1_LSBFIRST_Pos) /* msb first*/
		| (0 << SPI_CR1_SPE_Pos) /* peripheral disabled*/
		| (0 << SPI_CR1_BR_Pos) /* fpclk/2*/
		| (0 << SPI_CR1_MSTR_Pos) /* slave configuration*/
		| (1 << SPI_CR1_CPOL_Pos) /* clock to 0 when idle*/
		| (1 << SPI_CR1_CPHA_Pos) /* first clock transition is data capture edge*/
	);

	SPI2->CR2 = (
		  (0 << SPI_CR2_LDMATX_Pos) // not an odd number
		| (0 << SPI_CR2_LDMARX_Pos) // not an odd number
		| (1 << SPI_CR2_FRXTH_Pos) // 8-bit event
		| (7 << SPI_CR2_DS_Pos) // 8-bit data size
		| (0 << SPI_CR2_TXEIE_Pos) // TXE interrupt masked
		| (0 << SPI_CR2_RXNEIE_Pos) // RXNE interrupt masked
		| (0 << SPI_CR2_ERRIE_Pos) // error interrupt maked
		| (0 << SPI_CR2_FRF_Pos) // SPI Motorola mode
		| (0 << SPI_CR2_NSSP_Pos) // no nss pulse
		| (0 << SPI_CR2_SSOE_Pos) // SS output disable
		| (0 << SPI_CR2_TXDMAEN_Pos) // Tx buffer DMA enabled
		| (0 << SPI_CR2_RXDMAEN_Pos) // Rx buffer DMA enabled
	);

	spi2cr[0] = SPI2->CR1 | SPI_CR1_SPE;
	spi2cr[1] = SPI2->CR2;
}

static inline void spi2Reset() {
	__HAL_RCC_SPI2_FORCE_RESET();
	__HAL_RCC_SPI2_RELEASE_RESET();

	spi2Init();
}

#endif


static inline void dmamuxInit() {
	// request generator 0: low clk
	DMAMUX1_RequestGenerator0->RGCR = (
		  (0  << DMAMUX_RGxCR_GNBREQ_Pos) // GNBREQ-1
		| (2  << DMAMUX_RGxCR_GPOL_Pos) // falling edge
		| (0  << DMAMUX_RGxCR_GE_Pos) // do not enable generator yet
		| (0  << DMAMUX_RGxCR_OIE_Pos) // disable overrun event
		| (12 << DMAMUX_RGxCR_SIG_ID_Pos) // EXTI12 (CLK)
	);

	// request generator 1: high clk
	DMAMUX1_RequestGenerator1->RGCR = (
		  (0  << DMAMUX_RGxCR_GNBREQ_Pos) // GNBREQ-1
		| (1  << DMAMUX_RGxCR_GPOL_Pos) // rising edge
		| (0  << DMAMUX_RGxCR_GE_Pos) // do not enable generator yet
		| (0  << DMAMUX_RGxCR_OIE_Pos) // disable overrun event
		| (12 << DMAMUX_RGxCR_SIG_ID_Pos) // EXTI12 (CLK)
	);

	// request generator 2: low cs1, 2 transfers
	DMAMUX1_RequestGenerator2->RGCR = (
		  (1  << DMAMUX_RGxCR_GNBREQ_Pos) // GNBREQ-1 (generate 2 transfers)
		| (2  << DMAMUX_RGxCR_GPOL_Pos) // falling edge
		| (1  << DMAMUX_RGxCR_GE_Pos) // enable generator
		| (0  << DMAMUX_RGxCR_OIE_Pos) // disable overrun event
		| (14 << DMAMUX_RGxCR_SIG_ID_Pos) // EXTI14 (CS1)
	);

#ifdef ENABLE_SPI2
	// request generator 3: low cs2
	DMAMUX1_RequestGenerator3->RGCR = (
		  (1  << DMAMUX_RGxCR_GNBREQ_Pos) // GNBREQ-1 (generate 2 transfers)
		| (2  << DMAMUX_RGxCR_GPOL_Pos) // falling edge
		| (1  << DMAMUX_RGxCR_GE_Pos) // enable generator
		| (0  << DMAMUX_RGxCR_OIE_Pos) // disable overrun event
		| (15 << DMAMUX_RGxCR_SIG_ID_Pos) // EXTI15 (CS2)
	);
#else
	// request generator 3: high cs1, 2 transfers
	DMAMUX1_RequestGenerator3->RGCR = (
		  (1  << DMAMUX_RGxCR_GNBREQ_Pos) // GNBREQ-1 (generate 2 transfers)
		| (1  << DMAMUX_RGxCR_GPOL_Pos) // rising edge
		| (0  << DMAMUX_RGxCR_GE_Pos) // enable generator
		| (0  << DMAMUX_RGxCR_OIE_Pos) // disable overrun event
		| (14 << DMAMUX_RGxCR_SIG_ID_Pos) // EXTI14 (CS1)
	);
#endif

	// Fill the dmamuxGeneratorEnable buffer
	dmamuxGeneratorEnable[0] = DMAMUX1_RequestGenerator0->RGCR | DMAMUX_RGxCR_GE;
	dmamuxGeneratorEnable[1] = DMAMUX1_RequestGenerator1->RGCR | DMAMUX_RGxCR_GE;
	dmamuxGeneratorDisable[0] = DMAMUX1_RequestGenerator0->RGCR & ~DMAMUX_RGxCR_GE;
	dmamuxGeneratorDisable[1] = DMAMUX1_RequestGenerator1->RGCR & ~DMAMUX_RGxCR_GE;
}

static inline void dmamuxClearFlags() {
	DMAMUX1_RequestGenStatus->RGCFR = DMAMUX1_RequestGenStatus->RGSR;
	DMAMUX1_ChannelStatus->CFR = DMAMUX1_ChannelStatus->CSR;
}

static inline void dmaInit() {
	// Command transfer
	DMA1_Channel1->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (2 << DMA_CCR_PL_Pos) // high priority
		| (0 << DMA_CCR_MSIZE_Pos) // 8 bits in memory
		| (0 << DMA_CCR_PSIZE_Pos) // 8 bits in peripheral (GPIO)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (0 << DMA_CCR_PINC_Pos) // do not increment peripheral
		| (0 << DMA_CCR_CIRC_Pos) // no circular mode
		| (0 << DMA_CCR_DIR_Pos) // read from peripheral
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel0->CCR = (
		(1 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 0
	);

	DMA1_Channel1->CPAR = (uint32_t)&GPIOA->IDR;
	DMA1_Channel1->CMAR = (uint32_t)transferBufferInternal+COMMAND_TRANSFER_DELTA;

	// Data transfer
	DMA1_Channel2->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (3 << DMA_CCR_PL_Pos) // *very* high priority
		| (0 << DMA_CCR_MSIZE_Pos) // 8 bits in memory
		| (0 << DMA_CCR_PSIZE_Pos) // 8 bits in peripheral (GPIO)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (0 << DMA_CCR_PINC_Pos) // do not increment peripheral
		| (0 << DMA_CCR_CIRC_Pos) // no circular mode
		| (1 << DMA_CCR_DIR_Pos) // read from memory
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel1->CCR = (
		(2 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 1
	);

	DMA1_Channel2->CPAR = (uint32_t)&GPIOA->ODR;
	DMA1_Channel2->CMAR = (uint32_t)transferBufferInternal+DATA_TRANSFER_DELTA;

	// GPIO mode transfer
	// This sets GPIO as output after 7 transfers
	DMA1_Channel3->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (2 << DMA_CCR_PL_Pos) // high priority
		| (1 << DMA_CCR_MSIZE_Pos) // 16 bits in memory
		| (1 << DMA_CCR_PSIZE_Pos) // 16 bits in peripheral (GPIO MODER)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (0 << DMA_CCR_PINC_Pos) // do not increment peripheral
		| (0 << DMA_CCR_CIRC_Pos) // no circular mode
		| (1 << DMA_CCR_DIR_Pos) // read from memory
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel2->CCR = (
		(2 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 1
	);

	DMA1_Channel3->CPAR = (uint32_t)&GPIOA->MODER;
	DMA1_Channel3->CMAR = (uint32_t)gpioModeBuffer;

	// CS transfer
	DMA1_Channel4->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (1 << DMA_CCR_PL_Pos) // normal priority
		| (2 << DMA_CCR_MSIZE_Pos) // 32 bits in memory
		| (2 << DMA_CCR_PSIZE_Pos) // 32 bits in peripheral (DMA)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (1 << DMA_CCR_PINC_Pos) // increment peripheral
		| (1 << DMA_CCR_CIRC_Pos) // circular mode
		| (1 << DMA_CCR_DIR_Pos) // read from memory
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel3->CCR = (
		(3 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 2
	);

	DMA1_Channel4->CPAR = (uint32_t)&DMAMUX1_RequestGenerator0->RGCR;
	DMA1_Channel4->CMAR = (uint32_t)dmamuxGeneratorEnable;
	DMA1_Channel4->CNDTR = sizeof(dmamuxGeneratorEnable) / sizeof(uint32_t);

#ifdef ENABLE_SPI2
	// CS transfer (CS2 variant)
	// The point here is to start the SPI channel transfer as soon as chip is selected
	DMA1_Channel5->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (1 << DMA_CCR_PL_Pos) // normal priority
		| (2 << DMA_CCR_MSIZE_Pos) // 32 bits in memory
		| (2 << DMA_CCR_PSIZE_Pos) // 32 bits in peripheral (DMA)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (1 << DMA_CCR_PINC_Pos) // increment peripheral
		| (1 << DMA_CCR_CIRC_Pos) // circular mode
		| (1 << DMA_CCR_DIR_Pos) // read from memory
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel4->CCR = (
		(4 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 3
	);

	DMA1_Channel5->CPAR = (uint32_t)&SPI2->CR1;
	DMA1_Channel5->CMAR = (uint32_t)spi2cr;
	DMA1_Channel5->CNDTR = sizeof(spi2cr) / sizeof(uint32_t);

	// CS2 transfer

#else
	// CS transfer for disabling
	DMA1_Channel5->CCR = (
		  (0 << DMA_CCR_MEM2MEM_Pos) // disable memory-to-memory
		| (1 << DMA_CCR_PL_Pos) // normal priority
		| (2 << DMA_CCR_MSIZE_Pos) // 32 bits in memory
		| (2 << DMA_CCR_PSIZE_Pos) // 32 bits in peripheral (DMA)
		| (1 << DMA_CCR_MINC_Pos) // increment memory
		| (1 << DMA_CCR_PINC_Pos) // increment peripheral
		| (1 << DMA_CCR_CIRC_Pos) // circular mode
		| (1 << DMA_CCR_DIR_Pos) // read from peripheral
		| (0 << DMA_CCR_TEIE_Pos) // disable transfer error interrupt
		| (0 << DMA_CCR_HTIE_Pos) // disable half transfer interrupt
		| (0 << DMA_CCR_TCIE_Pos) // disable transfer complete interrupt
		| (0 << DMA_CCR_EN_Pos) // do not enable channel yet
	);

	DMAMUX1_Channel4->CCR = (
		(4 << DMAMUX_CxCR_DMAREQ_ID_Pos) // Request Generator 3
	);

	DMA1_Channel5->CPAR = (uint32_t)&DMAMUX1_RequestGenerator0->RGCR;
	DMA1_Channel5->CMAR = (uint32_t)dmamuxGeneratorDisable;
	DMA1_Channel5->CNDTR = sizeof(dmamuxGeneratorDisable) / sizeof(uint32_t);
#endif
}

// we can abuse DMA to do this and reduce time. We need to find more dma abuse optimizations
static void dmaRestart() {
	// Disable all DMA channels
	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;

	// Disable generators
	DMAMUX1_RequestGenerator0->RGCR &= ~DMAMUX_RGxCR_GE;
	DMAMUX1_RequestGenerator1->RGCR &= ~DMAMUX_RGxCR_GE;
	//DMAMUX1_RequestGenerator2->RGCR &= ~DMAMUX_RGxCR_GE;

	// Re-configure dma channels
	DMA1_Channel1->CNDTR = COMMAND_BUF_SIZE; // read everything
	DMA1_Channel2->CNDTR = sizeof(transferBufferInternal)-DATA_TRANSFER_DELTA;
	DMA1_Channel3->CNDTR = sizeof(gpioModeBuffer) / 2;

	// Re-enable all DMA channels
	DMA1_Channel1->CCR |= DMA_CCR_EN;
	DMA1_Channel2->CCR |= DMA_CCR_EN;
	DMA1_Channel3->CCR |= DMA_CCR_EN;
}


static inline int16_t key2_step_4(int16_t savail, uint16_t spos) {
	uint32_t x_lo = key2.x.lo, x_hi = key2.x.hi;
    uint32_t x_tmp = (x_lo << 19) ^ (x_lo << 7) ^ (x_lo << 6) ^ ((x_lo >> 7) | (x_hi << 25));
    x_tmp ^= (x_tmp >> 13) ^ (x_tmp >> 25);

	uint32_t y_lo = key2.y.lo, y_hi = key2.y.hi;
    uint32_t y_tmp = (y_lo << 19) ^ (y_lo << 6) ^ (y_lo << 1) ^ ((y_lo >> 7) | (y_hi << 25));
    y_tmp ^= (y_tmp >> 13) ^ (y_tmp >> 31);

    __disable_irq();
    key2.x.hi = x_lo;
    key2.x.lo = x_tmp;
    key2.y.hi = y_lo;
    key2.y.lo = y_tmp;
    (*(volatile uint32_t *)(&seedBuffer[(spos+savail)&SEED_MASK])) = __builtin_bswap32(x_tmp ^ y_tmp);
    savail += 4;
    seedAvailable = savail;
    __enable_irq();
    return savail;
}

// TODO: try to remove the unrolling (it makes things slower, doesn't it?)
static inline int16_t key2_step(int16_t savail, uint16_t spos) {
//#pragma GCC unroll 16/4
	for (int i = 4; i != 0; --i) {
		savail = key2_step_4(savail, spos); //(volatile uint32_t *)(&seedBuffer[(spos+savail+i)&SEED_MASK]));
	}

	return savail;
}

static inline void key2_loop() {
	int16_t savail = seedAvailable;
	uint16_t spos = seedPosition;

	while (savail < 0x3FE0) {
		savail = key2_step(savail, spos);
	}
}

// TODO: test it
// could be faster, could be slower, who knows
static inline void key2_loop_test() {
    uint32_t x_lo = key2.x.lo, x_hi = key2.x.hi;
    uint32_t y_lo = key2.y.lo, y_hi = key2.y.hi;
    uint32_t x_tmp, y_tmp;
    
    uint32_t spos = seedPosition;
    uint32_t savail = seedAvailable;

    while (savail < 0x3FE0) {
        x_tmp = (x_lo << 19) ^ (x_lo << 7) ^ (x_lo << 6) ^ ((x_lo >> 7) | (x_hi << 25));
        x_tmp ^= (x_tmp >> 13) ^ (x_tmp >> 25);

        y_tmp = (y_lo << 19) ^ (y_lo << 6) ^ (y_lo << 1) ^ ((y_lo >> 7) | (y_hi << 25));
        y_tmp ^= (y_tmp >> 13) ^ (y_tmp >> 31);

        x_lo = (x_tmp << 19) ^ (x_tmp << 7) ^ (x_tmp << 6) ^ ((x_tmp >> 7) | (x_lo << 25));
        x_lo ^= (x_lo >> 13) ^ (x_lo >> 25);

        y_lo = (y_tmp << 19) ^ (y_tmp << 6) ^ (y_tmp << 1) ^ ((y_tmp >> 7) | (y_lo << 25));
        y_lo ^= (y_lo >> 13) ^ (y_lo >> 31);

        __disable_irq();
        key2.x.hi = x_tmp;
        key2.x.lo = x_lo;
        key2.y.hi = y_tmp;
        key2.y.lo = y_lo;
        (*(volatile uint32_t *)(&seedBuffer[(spos+savail)&SEED_MASK])) = __builtin_bswap32(x_tmp ^ y_tmp);
        (*(volatile uint32_t *)(&seedBuffer[(spos+savail+4)&SEED_MASK])) = __builtin_bswap32(x_lo ^ y_lo);
        savail += 8;
        seedAvailable = savail;
        __enable_irq();
    }
}

// this one ends if we reached the end of the buffer
#define CALCULATE_KEY2_WHILE(cond) \
	do { \
		int16_t savail = seedAvailable; \
		uint16_t spos = seedPosition; \
		if (savail < 0x3FE0) { \
			do { \
				savail = key2_step(savail, spos); \
			} while ((cond) && savail < 0x3FE0); \
		} \
	} while (0)

// this one ends when cond is false
#define DUMMY_KEY2_WHILE(cond) \
	do { \
		CALCULATE_KEY2_WHILE(cond); \
		while (cond); \
	} while (0)

static inline void initialize_key2(uint32_t seed) {
	initKey2(seed, 0xE8);

	// okay, we have calculated this mess, reset the positions
	seedPosition = 0;
	seedAvailable = 0;
}

// pre-init functions
static void preinit_command() {
	spiAbort();

	spiSelect();
	spiBuffer[3] = 0; // fourth command byte (a7-0)

	// set dma buf length
	DMA2_Channel1->CNDTR = 4; // command size (circular buffer) - first byte manual
	DMA2_Channel2->CNDTR = 0x1005;

	// set dma addresses
	DMA2_Channel1->CMAR = (uint32_t)spiBuffer+1; // txbuf
	DMA2_Channel2->CMAR = (uint32_t)transferBuffer-5; // rxbuf

	// enable RX dma channel (but disable TX dma channel)
	DMA2_Channel1->CCR &= ~DMA_CCR_EN;
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	// write the first command byte
	*(__IO uint8_t *)&SPI1->DR = 0x0B;
}

static void preinit_key1() {
	spiAbort();

	spiSelect();
	spiBuffer[3] = 0; // fourth command byte (a7-0)

	// set dma buf length
	DMA2_Channel1->CNDTR = 4; // command size (circular buffer) - first byte manual
	DMA2_Channel2->CNDTR = 0x1005;

	// set dma addresses
	DMA2_Channel1->CMAR = (uint32_t)spiBuffer+1; // txbuf
	DMA2_Channel2->CMAR = (uint32_t)transferBuffer+0x19B8-0x1000-5; // rxbuf - strategic choice

	// enable RX dma channel (but disable TX dma channel)
	DMA2_Channel1->CCR &= ~DMA_CCR_EN;
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	// write the first command byte
	*(__IO uint8_t *)&SPI1->DR = 0x0B;

}

static void preinit_key2() {
	spiAbort();

	spiSelect();
	spiBuffer[3] = 0; // fourth command byte (a7-0)

	// set dma buf length
#ifdef TRANSFER_1000H
	DMA2_Channel1->CNDTR = 5; // tx - the first command byte is written manually (command size)
	DMA2_Channel2->CNDTR = 0x1005;
#else
	DMA2_Channel1->CNDTR = 5; // tx - the first command byte is written manually (command size)
	DMA2_Channel2->CNDTR = 0x205;
#endif

	// set dma addresses
	DMA2_Channel1->CMAR = (uint32_t)spiBuffer+1; // txbuf
	DMA2_Channel2->CMAR = (uint32_t)transferBuffer-5; // rxbuf

	// enable RX dma channel (but disable TX dma channel)
	DMA2_Channel2->CCR |= DMA_CCR_EN;

	// write the first command byte
	*(__IO uint8_t*)&SPI1->DR = 0x0B;
}

static inline void spiDataRead_command(uint32_t address) {
	// specialized for normal mode (search for preinit_command)
	// write address
	*(volatile uint32_t *)&spiBuffer[0] = __builtin_bswap32(address);

	// enable tx dma channel
	DMA2_Channel1->CCR |= DMA_CCR_EN | DMA_CCR_CIRC;
}

static inline void spiDataRead_key1(uint32_t address) {
	spiSelect();
	*(volatile uint32_t *)&spiBuffer[0] = __builtin_bswap32(address);

	// enable tx dma channels
	DMA2_Channel1->CCR |= DMA_CCR_EN | DMA_CCR_CIRC;

	// nb: it's aligned
	//return &spiBuffer[12];
}

static inline volatile void spiDataRead_key2(uint32_t address) {
	// specialized for key2 (search for preinit_key2)
	// write address
	*(volatile uint32_t *)&spiBuffer[0] = __builtin_bswap32(address);

	// enable tx dma channel
	DMA2_Channel1->CCR |= DMA_CCR_EN | DMA_CCR_CIRC;
}

static inline void handle_command() {
	// when using preinit in chip select, we have already read 6 command bytes
	WAIT_FOR_BYTE(0);

	switch (command[0]) {
	case 0x00: {
		preinit_command();
		WAIT_FOR_BYTE(4);
		uint32_t address = (__builtin_bswap32(COMMAND_HI) << 8) | command[4];
		spiDataRead_command(address);

		SPI_WAIT(0x1000, 0);
		DATA_OUT = transferBuffer[0];
		SPI_WAIT(0x1000, 0xFFF); // wait till we receive it all because we're getting aborted
		break;
	}

	case 0x90: {
		DATA_OUT = transferBuffer[0] = chipId[0];
		transferBuffer[1] = chipId[1];
		transferBuffer[2] = chipId[2];
		transferBuffer[3] = chipId[3];

		for (int i = 4; i < 0x200; i += 4) {
			*(volatile uint32_t *)&transferBuffer[i] = *(uint32_t *)chipId;
		}
		break;
	}

	case 0x9F: {
		// dummy command
		// (nb: we are not fast enough to fill keybuf here)
		break;
	}

	case 0x3C: {
		// enable KEY1
		currentState = 3;
		break;
	}

	case 'A': {
		WAIT_FOR_BYTE(4);
		if (memcmp_v(command, flash_modecmd, 4) == 0) {
			// switch to flash mode
			currentState = 7;
			break;
		}

		// falls to default
	}

	default: {
		break;
	}
	}
}

static inline void handle_key1() {
	WAIT_FOR_BYTE(7); // we want the whole command
	key1_decrypt_cmd((volatile uint32_t *)command);

	// command_hi and command_lo are byte-reversed
	// encrypted input = {01, 02, 03, 04, 05, 06, 07, 08}
	// decrypted command = {04, 03, 02, 01, 08, 07, 06, 05}
	// command_hi = 0x01020304
	// command_lo = 0x05060708

	switch (command[3] >> 4) {
		case 0x1: {
			CALCULATE_KEY2_WHILE(DATA_PROGRESS() < (0x910 - 0x40));

			// 1lllliiijjjkkkkkh (914h) - 2nd Get ROM Chip ID / Get KEY2 Stream
			uint16_t pos = 0x910;
			KEY2_BEGIN();
			*(volatile uint32_t *)&transferBuffer[0x900] = __builtin_bswap32(seedAvailable);
			*(volatile uint32_t *)&transferBuffer[0x904] = __builtin_bswap32(seedPosition);

			while (pos < (0x910+0x200)) {
				*(volatile uint32_t *)&transferBuffer[pos] = *(uint32_t *)chipId ^ KEY2_AT_32(pos);
				pos += 4;
			}

			KEY2_END();
			break;
		}

		case 0x2: {
			// 2bbbbiiijjjkkkkkh (19B8h) - Get Secure Area Block (4KBytes)
			uint32_t address = ((COMMAND_HI >> 12) & 0xFFFF) * 0x1000;

			preinit_key1();
			spiDataRead_key1(address);

			CALCULATE_KEY2_WHILE(DATA_PROGRESS() < (0x910 - 0x40));

			KEY2_BEGIN();

			uint16_t pos = 0;
			uint16_t key2pos = 0x910;

			for (int j = 0; j < 8; j++) {
				// 0x200 + 0x18 dummy

				// Write 0x200 bytes!
				for (int i = 0; i < 0x200/0x80; i++) {
					DUMMY_KEY2_WHILE(!IS_SPI_AVAILABLE(0x1000, pos+0x7F));
					for (int k = 0; k < 0x80; k += 4) {
						*(volatile uint32_t *)&transferBuffer[key2pos+k] = *(volatile uint32_t *)&transferBuffer[0x19B8-0x1000+pos+k] ^ KEY2_AT_32(key2pos+k);
					}

					pos += 0x80;
					key2pos += 0x80;
				}

				key2pos += 0x18;

				// we wrote up to `key2pos`
				// continue when the console has caught up with us
				CALCULATE_KEY2_WHILE(DATA_PROGRESS() < (key2pos - 0x40));
			}

			KEY2_END();
			break;
		}

		case 0x4: {
			// 4llllmmmnnnkkkkkh (910h) - Activate KEY2 Encryption Mode
			// ^^^^^^^^--------
			uint32_t seed = ((COMMAND_HI << 12) | (COMMAND_LO >> 20)) & 0xFFFFFF;
			currentState = 5;
			initialize_key2(seed);
			break;
		}

		case 0xA: {
			// Alllliiijjjkkkkkh (910h) - Enter Main Data Mode
			// we don't care about transfer buffer or anything
			// state key2
			currentState = 4;
			break;
		}

		default: {
			break;
		}
	}
}

static inline void handle_key2() {
	KEY2_BEGIN();

	WAIT_FOR_BYTE(0);
	int cmd = command[0] ^ KEY2_AT(0);

	switch (cmd) {
	case 0xB7: {
		preinit_key2();
		WAIT_FOR_BYTE(3);
		uint32_t address = (__builtin_bswap32(COMMAND_HI ^ KEY2_AT_32(0)) << 8);// | (command[4] ^ KEY2_AT(4));
		address &= 0x7fffff;
		if (address < 0x8000) {
			address = 0x8000 + (address & 0x1FF);
		}

		spiDataRead_key2(address);
#ifdef TRANSFER_1000H
		SPI_WAIT(0x1000, 0);
#else
		SPI_WAIT(0x200, 0);
#endif
		DATA_OUT = (transferBuffer[0] ^= KEY2_AT(8));

		uint16_t pos = 1;
		int16_t savail = seedAvailable;

		// overall: 18 clocks per byte (not incl. the loop)
		// mathematically: uC is 150 mhz, spi is 75mhz, 9.3mhz for byte transfer,
		// which means we need 16 clocks for a single byte

		// thus we need 64 clocks for 4 bytes, but we are fine with 72.
		// (we need 128 clocks for 8 bytes)
		// first run is 156 clocks, the second run is 83 clocks (which is a bit better).

		// we need to optimize this way because compiler hates us

		// Update: (I wrote the comments above 6 months ago)
		// This snippet works for some reason ; it is synchronizing with the SPI
		// and it has been tested at various transfer rates.
		// It's not the best way to synchronize (it's dependant on the clock rate)
		// The TRANSFER_1000h variant doesn't depend on the CPU clock rate;
		// but the 200h first bytes are the hardest (because we need to provide
		// an immediate response)

		if (savail < 0x3CE0) {
			// do the first bytes "byte per byte"
			for (; pos < 0x20; pos++) {
				transferBuffer[pos] ^= KEY2_AT(8+pos);
				__NOP(); __NOP(); __NOP();
			}

			for (; pos < 0x200; pos += 4) {
				savail = key2_step_4(savail, _spos);
				__NOP(); __NOP(); __NOP(); // timings....
				__NOP(); __NOP(); __NOP();
				__NOP(); __NOP(); __NOP();
				*(volatile uint32_t *)&transferBuffer[pos] ^= KEY2_AT_32(8+pos);
			}

		} else {
			// only write byte per byte
			for (; pos < 0x200; pos++) {
				transferBuffer[pos] ^= KEY2_AT(8+pos);
				__NOP(); __NOP(); __NOP();
			}
		}

#ifdef TRANSFER_1000H
		do {
			CALCULATE_KEY2_WHILE(DATA_PROGRESS() < (pos - 0x20));

			uint16_t avail = SPI_AVAILABLE(0x1000) & ~3;
			if (avail - pos > 0x100) avail = pos + 0x100;
			for (; pos < avail; pos += 4) {
				*(volatile uint32_t *)&transferBuffer[pos] ^= KEY2_AT_32(8+pos);
			}
		} while (pos != 0x1000);
#endif

		break;

	}
	case 0xB8: {
		DATA_OUT = transferBuffer[0] = chipId[0] ^ KEY2_AT(8);
		transferBuffer[1] = chipId[1] ^ KEY2_AT(9);
		transferBuffer[2] = chipId[2] ^ KEY2_AT(10);
		transferBuffer[3] = chipId[3] ^ KEY2_AT(11);

		// Writing more means it gets interrupted by chip unselect and then chip select,
		// which gives us less time to handle the next command.
		break;
	}

	default: {
		break;
	}
	}

	KEY2_END();
}


static void start_ignoring() {
	__disable_irq();

	DMA1_Channel1->CCR &= ~DMA_CCR_EN;
	DMA1_Channel2->CCR &= ~DMA_CCR_EN;
	DMA1_Channel3->CCR &= ~DMA_CCR_EN;
	DMA1_Channel4->CCR &= ~DMA_CCR_EN;
	DMA1_Channel5->CCR &= ~DMA_CCR_EN;

	*(volatile uint16_t *)&GPIOA->MODER = 0x5555;
	DATA_OUT = 0xCC;
}

static void stop_ignoring() {
	*(volatile uint16_t *)&GPIOA->MODER = 0;

	// restart dma channel 4
	DMA1_Channel4->CCR |= DMA_CCR_EN;
	DMA1_Channel5->CCR |= DMA_CCR_EN;

	// restore irq (might never return)
	while (!(GPIOA->IDR & CS1_Pin)); // while it's selected
	__enable_irq();
}

static void preinit_flash() {
	gpioModeBuffer[7] = 0x5555;
}

// This whole flashing protocol is a mess
static void handle_flash() {
	WAIT_FOR_BYTE(3)
	if (memcmp_v(command, flash_pollcmd, 4) == 0) {
		// not checking the rest of the command for that one
		DATA_OUT = transferBuffer[0] = 'F';
		transferBuffer[1] = 'E';
		transferBuffer[2] = 'U';
		transferBuffer[3] = 'R';

	} else if (memcmp_v(command, flash_debugcmd, 4) == 0) {
		DATA_OUT = transferBuffer[0];
		//memcpy_v(transferBuffer, flashbuf, 0x200);

	} else {
		// nevermind, we read
		gpioModeBuffer[7] = 0;
		*(volatile uint16_t *)&GPIOA->MODER = 0;

		WAIT_FOR_BYTE(7);
		if (memcmp_v(command, flash_flashcmd, 8) == 0) {
			WAIT_FOR_BYTE(8+4+0x100-1);
			start_ignoring();

			// okay, we are free to do whatever we want
			uint32_t address = __builtin_bswap32(*(volatile uint32_t *)&command[8]);
			spiProgramPage(address, (command+12)); // NOTE: discarding volatile shouldn't be an issue

			if (address == 0) {
				// re-calculate flashbuf
				keybuf_ptr = (uint32_t *)seedBuffer;

				// command+12 is ROM header
				// gamecode is @ 0xC
				key1_init(*(uint32_t *)(command+12+0xC));

				// disabled because HAL
				FLASH_EraseInitTypeDef erase;
				erase.TypeErase = FLASH_TYPEERASE_PAGES;
				erase.Banks = FLASH_BANK_1; // i have a single bank
				erase.Page = (KEYBUF_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE; // erase our page
				erase.NbPages = (8<<10) / FLASH_PAGE_SIZE; // (erase 8 KB)
				uint32_t pageerror;

			    HAL_FLASH_Unlock();
			    HAL_FLASHEx_Erase(&erase, &pageerror);
			    for (int i = 0; i < 0x412; i += 2) {
			    	if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, KEYBUF_ADDR + i * 4, *(uint64_t *)&keybuf_ptr[i]) != HAL_OK) {
			    		DATA_OUT = 0xBF;
			    		for (;;);
			    	}
			    }

			    HAL_FLASH_Lock();
			}
			stop_ignoring();

		} else if (memcmp_v(command, flash_erasecmd, 8) == 0) {
			WAIT_FOR_BYTE(11); // we want command[8,9,10,11]
			start_ignoring();

			// okay, we are free to do whatever we want
			uint32_t address = __builtin_bswap32(*(volatile uint32_t *)&command[8]);
			spiEraseSector(address);
			stop_ignoring();

		} else {
			start_ignoring();

			DATA_OUT = 0xEE;
			delay(500000);

			stop_ignoring();
		}
	}
}


void irqhandler() {
	longjmp(jmp, 0);
}

// we don't really care about our registers when we enter irqhandler, so here's another way
void EXTI15_10_IRQHandler() {
	// stack contains r0, r1, r2, r3, r12, lr, pc, xPSR
	__asm__ (
		// __disable_irq
		"cpsid i\n"

		// EXTI->PR1 = (CS1_Pin | CS2_Pin)
		"mov.w r3, #0x0400\n" // address 0x40010400
		"movt r3, #0x4001\n"
		"mov.w r2, #0xC000\n" // (CS1_Pin | CS2_Pin)
		"str r2, [r3,#0x14]\n"

		//"pop {r1,r2,r3,r4,r5,r6,r7}\n"

		// Update PC
		"ldr r1, =irqhandler\n"
		"str r1, [sp,#24]\n"

		// Update xPSR
		"ldr r1, [sp,#28]\n"
		"bic r1, r1, #0xFE00FE00\n"
		"bic r1, r1, #0xFF0000\n"
		"str r1, [sp,#28]\n"

		//"push {r1,r2,r3,r4,r5,r6,r7}\n"
		"bx lr\n"
	);
}

static void consumeKey2(int32_t count) {
	if (count <= 0)
		return;

	seedPosition = (seedPosition + count) & SEED_MASK;
	seedAvailable -= count;
}

static void on_chip_select() {
	// chip selected
	cs1Enabled = 1;
	__enable_irq();

	switch (currentState) {
	case 0:
		handle_command();
		break;

	case 1:
		handle_key1();
		break;

	case 2:
		handle_key2();
		break;

	case 6:
		handle_flash();
		break;
	}
}

#ifdef ENABLE_SPI2
#define DATA_SPI2 (*(volatile uint8_t *)&SPI2->DR)
#define WAIT_RX_SPI2() do ; while ((SPI2->SR & SPI_SR_RXNE) == 0)
#define WAIT_TX_SPI2() do ; while ((SPI2->SR & SPI_SR_FTLVL) == SPI_SR_FTLVL)

static void on_spi_select() {
	cs2Enabled = 1;
	__enable_irq();

	// Can DMA be used here? It would most likely be very helpful
	// We've had some issues with DMA though

	// Enable DMA channels
	//DMA2_Channel3->CCR |= DMA_CCR_EN;
	//DMA2_Channel4->CCR |= DMA_CCR_EN;

	// wait for RX
	WAIT_RX_SPI2();
	uint8_t byte = DATA_SPI2;

	switch (byte) {
	case 0x05: // read status register
		DATA_SPI2 = 0;//(spi2_write_status == 0 ? 0 : 1) | (spi2_sector_progress << 1);
		break;

	case 0x01:
		WAIT_RX_SPI2();
		uint8_t sr = DATA_SPI2;
		UNUSED(sr);
		break;

	case 0x06:
		break;

	case 0x04:
		break;

	case 0x03: {
		//if (spi2_write_status != 0)
		//	break;

		uint32_t address = 0;
		WAIT_RX_SPI2();
		address |= DATA_SPI2 << 8;
		WAIT_RX_SPI2();
		address |= DATA_SPI2;

		//volatile uint8_t *buf = spiDataRead(address, 0x200, spi2_sector_backup);
		const uint8_t *buf = &save[address];

		int n = 0;
		for (;;) {
			//SPI_WAIT(0x200, n);
			WAIT_TX_SPI2();
			DATA_SPI2 = buf[n];
			n++;
		}

		break;
	}

	case 0x02: {
		WAIT_RX_SPI2();
		uint16_t address = DATA_SPI2 << 8;
		WAIT_RX_SPI2();
		address |= DATA_SPI2;

		/*
		__enable_irq();
		spi2_write_status = 1;
		spi2_write_address = 0xFF0000 | address;
		spi2_write_progress = 0;
		spi2_write_length = 0;
		spi2_sector_progress = 0;
		__disable_irq();
		*/

		for (;;) {
			WAIT_RX_SPI2();
			__disable_irq();
			//spi2_write_input[spi2_write_length++] = DATA_SPI2;
			//fake_save[address] = DATA_SPI2;
			__enable_irq();
			address++;
		}

		break;
	}

	default:
		for (;;) {
			WAIT_TX_SPI2();
			*(volatile uint8_t *)&SPI2->DR = 0xFF;
		}
	}

	// SPI2_MISO enable
	//GPIOB->MODER = (GPIOB->MODER & ~(3<<(14*2))) | (2<<(14*2));

	// Make sure the other pins won't cause any trouble (input mode)

	// Enable DMA channels and SPI

}

static inline uint8_t spi1_transfer(uint8_t value) {
	while (!(SPI1->SR & SPI_SR_TXE));
	DATA_SPI1 = value;

	while (!(SPI1->SR & SPI_SR_RXNE)); // wait to read the next coming byte
	return DATA_SPI1;
}

static inline uint8_t spi1_send_recv(uint8_t byte) {
	spiSelect();
	spi1_transfer(byte);
	uint8_t b = spi1_transfer(0);
	spiUnselect();
	return b;
}

static inline void spi1_send(uint8_t byte) {
	spiSelect();
	spi1_transfer(byte);
	spiUnselect();
	return;
}
#endif

static void on_chip_unselect() {
	// chip unselected
	// switch to read gpioa
	*(volatile uint16_t*)&GPIOA->MODER = 0;

	// restart dma
	int32_t transferred = DATA_PROGRESS();
	dmaRestart();

#ifdef ENABLE_SPI2
	if (cs2Enabled) {
		cs2Enabled = 0;
		spi2Reset();
	}
#endif

	if (cs1Enabled) {
		cs1Enabled = 0;
		//spiAbort();

		// state transition
		switch (currentState) {
		case 0:
			// normal command
			//preinit_command();
			break;

		case 1:
			// key1 command, advance by transferred
			consumeKey2(transferred);
			//preinit_key1();
			break;

		case 2:
			// key2 command, advance by transferred + command size
			consumeKey2(transferred+8);
			//preinit_key2();
			break;

		case 3:
			// switch from normal to key1
			currentState = 1;
			//preinit_key1();
			break;

		case 4:
			// switch from key1 to key2
			consumeKey2(transferred);
			currentState = 2;
			//preinit_key2();
			break;

		case 5:
			// re-set key2 seeds (key1-only command)
			currentState = 1;
			//preinit_key1();

			//consumeKey2(transferred-0x910);
			break;

		case 6:
			// flash mode, ignore
			preinit_flash();
			break;

		case 7:
			// switch to flash mode
			currentState = 6;
			preinit_flash();
			break;
		}

	}

	__enable_irq();
}

#define MODER_INPUT(n) (0<<(2*n))
#define MODER_OUTPUT(n) (1<<(2*n))
#define MODER_ALTERNATE(n) (2<<(2*n))
#define MODER_ANALOG(n) (3<<(2*n))

#define PUPDR_NOPULL(n) (0<<(2*n))
#define PUPDR_PULLUP(n) (1<<(2*n))
#define PUPDR_PULLDOWN(n) (2<<(2*n))

#define OSPEEDR_LOW(n) (0<<(2*n))
#define OSPEEDR_MEDIUM(n) (1<<(2*n))
#define OSPEEDR_HIGH(n) (2<<(2*n))
#define OSPEEDR_VERYHIGH(n) (3<<(2*n))

#define AFR0_SEL(n, v) ((v)<<(4*n))
#define AFR1_SEL(n, v) ((v)<<(4*(n-8)))

void ndscart_begin() {
	// disable systick, we don't need it
	SysTick->CTRL = 0;

	// enable peripherals
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_SPI2);
    LL_APB2_GRP1_EnableClock(LL_APB2_GRP1_PERIPH_SYSCFG|LL_APB2_GRP1_PERIPH_SPI1);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_DMA1|LL_AHB1_GRP1_PERIPH_DMA2|LL_AHB1_GRP1_PERIPH_DMAMUX1);
    LL_AHB2_GRP1_EnableClock(LL_AHB2_GRP1_PERIPH_GPIOA|LL_AHB2_GRP1_PERIPH_GPIOB|LL_AHB2_GRP1_PERIPH_GPIOF);

#ifdef ENABLE_SPI2
	// Configure GPIOF:
	// (1) SPI2_SCK		AF5 (?)

	GPIOF->AFR[0] = (
			AFR0_SEL(1, 5)
	);

	GPIOF->MODER = (
			MODER_ANALOG(0) | MODER_ALTERNATE(1)
	);

	GPIOF->PUPDR = (
			PUPDR_NOPULL(0) | PUPDR_NOPULL(1)
	);

	GPIOF->OSPEEDR = (
			OSPEEDR_LOW(0) | OSPEEDR_HIGH(1)
	);
#endif

	// Configure GPIOB:
	// (3) SPI1_SCK     AF5
	// (4) SPI1_MISO    AF5
	// (5) SPI1_MOSI    AF5
	// (6) SPI1_CS      GPIO output
	// (14) SPI2_MISO	AF5 (analog)

	GPIOB->AFR[0] = (
			AFR0_SEL(3, 5) | AFR0_SEL(4, 5) | AFR0_SEL(5, 5)
	);

	GPIOB->AFR[1] = (
			AFR1_SEL(14, 5)
	);

	GPIOB->MODER = (
			MODER_ANALOG(0) | MODER_ANALOG(1) | MODER_ANALOG(2) | MODER_ALTERNATE(3)
			| MODER_ALTERNATE(4) | MODER_ALTERNATE(5) | MODER_OUTPUT(6) | MODER_ANALOG(7)
			| MODER_ANALOG(8) | MODER_ANALOG(9) | MODER_ANALOG(10) | MODER_ANALOG(11)
			| MODER_ANALOG(12) | MODER_ANALOG(13) | MODER_ALTERNATE(14) | MODER_ANALOG(15)
	);

	GPIOB->PUPDR = (
			PUPDR_NOPULL(0) | PUPDR_NOPULL(1) | PUPDR_NOPULL(2) | PUPDR_NOPULL(3)
			| PUPDR_NOPULL(4) | PUPDR_NOPULL(5) | PUPDR_NOPULL(6) | PUPDR_NOPULL(7)
			| PUPDR_NOPULL(8) | PUPDR_NOPULL(9) | PUPDR_NOPULL(10) | PUPDR_NOPULL(11)
			| PUPDR_NOPULL(12) | PUPDR_NOPULL(13) | PUPDR_NOPULL(14) | PUPDR_NOPULL(15)
	);

	GPIOB->OSPEEDR = (
			OSPEEDR_LOW(0) | OSPEEDR_LOW(1) | OSPEEDR_LOW(2) | OSPEEDR_HIGH(3)
			| OSPEEDR_HIGH(4) | OSPEEDR_HIGH(5) | OSPEEDR_LOW(6) | OSPEEDR_LOW(7)
			| OSPEEDR_LOW(8) | OSPEEDR_LOW(9) | OSPEEDR_LOW(10) | OSPEEDR_LOW(11)
			| OSPEEDR_LOW(12) | OSPEEDR_LOW(13) | OSPEEDR_HIGH(14) | OSPEEDR_LOW(15)
	);


	// Configure GPIOA:
	// CS1 (14), CS2 (15): exti with rising/falling, high speed
	// CLK (12): exti (event) with rising/falling, high speed
	// I/O (0-7): input with high speed
	// SPI2_MOSI (11): AF5
	// for the rest, analog mode (3)

	GPIOA->AFR[1] = (
			AFR1_SEL(11, 5)
	);

	GPIOA->MODER = (
			MODER_INPUT(0) | MODER_INPUT(1) | MODER_INPUT(2) | MODER_INPUT(3)
			| MODER_INPUT(4) | MODER_INPUT(5) | MODER_INPUT(6) | MODER_INPUT(7)
			| MODER_ANALOG(8) | MODER_ANALOG(9) | MODER_ANALOG(10) | MODER_ALTERNATE(11)
			| MODER_INPUT(12) | MODER_ANALOG(13) | MODER_INPUT(14) | MODER_INPUT(15)
	);

	GPIOA->PUPDR = (
			PUPDR_NOPULL(0) | PUPDR_NOPULL(1) | PUPDR_NOPULL(2) | PUPDR_NOPULL(3)
			| PUPDR_NOPULL(4) | PUPDR_NOPULL(5) | PUPDR_NOPULL(6) | PUPDR_NOPULL(7)
			| PUPDR_NOPULL(8) | PUPDR_NOPULL(9) | PUPDR_NOPULL(10) | PUPDR_NOPULL(11)
			| PUPDR_PULLUP(12) | PUPDR_NOPULL(13) | PUPDR_PULLUP(14) | PUPDR_PULLUP(15)
	);

	GPIOA->OSPEEDR = (
			OSPEEDR_HIGH(0) | OSPEEDR_HIGH(1) | OSPEEDR_HIGH(2) | OSPEEDR_HIGH(3)
			| OSPEEDR_HIGH(4) | OSPEEDR_HIGH(5) | OSPEEDR_HIGH(6) | OSPEEDR_HIGH(7)
			| OSPEEDR_LOW(8) | OSPEEDR_LOW(9) | OSPEEDR_LOW(10) | OSPEEDR_HIGH(11)
			| OSPEEDR_HIGH(12) | OSPEEDR_LOW(13) | OSPEEDR_HIGH(14) | OSPEEDR_HIGH(15)
	);

	SYSCFG->EXTICR[3] = 0; // Port A for all EXTI12-15
	EXTI->RTSR1 = (1<<12) | (1<<14);// | (1<<15); // enable rising trigger for EXTI12, EXTI14 and EXTI15
	EXTI->FTSR1 = (1<<12) | (1<<14);// | (1<<15); // enable falling trigger for EXTI12, EXTI14 and EXTI15
	EXTI->EMR1 = (1<<12); // event generation for EXTI12
	EXTI->IMR1 = (1<<14);// | (1<<15); // enable interrupt generation for EXTI14 and EXTI15, disable for EXTI12

#ifdef ENABLE_SPI2
	EXTI->RTSR1 |= (1<<15);
	EXTI->FTSR1 |= (1<<15);
	EXTI->IMR1 |= (1<<15);
#endif

	initKey2Default();

	// Initialize gpioModeBuffer
	for (int i = 0; i < sizeof(gpioModeBuffer) / 2; i++) {
		if (i < 7) { // 0 to 6
			gpioModeBuffer[i] = 0;
		} else { // 7
			gpioModeBuffer[i] = 0x5555;
		}
	}

	*(volatile uint16_t*)&GPIOA->MODER = 0;

#ifdef ENABLE_SPI2
	// SPI2 init
	spi2Init();
#endif

	// Initialize DMA
	dmaInit();
	dmamuxInit();
	spiInit();
	spiInitDma();
	spiUnselect();

	// Initialize transfers
	__disable_irq();
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	dmaRestart();
	spiStop();
#ifdef ENABLE_SPI2
	//spi2Prepare();
#endif
	spiPrepareEnable();
	DMA1_Channel4->CCR |= DMA_CCR_EN; // CS1 low transfer
	DMA1_Channel5->CCR |= DMA_CCR_EN; // CS2 low/CS1 high transfer
	setjmp(jmp);

	// read GPIO, check CS status
	uint16_t idr = *(volatile uint16_t *)&GPIOA->IDR;

	if ((idr & CS1_Pin) == 0) {
		on_chip_select();
		spiAbort();

#ifdef ENABLE_SPI2
	} else if ((idr & CS2_Pin) == 0) {
		on_spi_select();
		spiAbort();
#endif

	} else {
		spiAbort();
		on_chip_unselect();
	}

	// done ; loop now
	key2_loop();
	for (;;) {
		HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
	}
}

