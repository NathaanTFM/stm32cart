# stm32cart

Cartridge for DS units using a STM32G431CBU6 micro-controller and a W25Q128JVSIQ NOR FLASH chip.

## Pinout

GPIO A
- PA0 (8): NDS I/O 0
- PA1 (9): NDS I/O 1
- PA2 (10): NDS I/O 2
- PA3 (11): NDS I/O 3
- PA4 (12): NDS I/O 4
- PA5 (13): NDS I/O 5
- PA6 (14): NDS I/O 6
- PA7 (15): NDS I/O 7
- PA11 (33): NDS I/O 7 (SPI2 MOSI)
- PA12 (34): NDS CLK
- PA13 (36): NDS UNUSED
- PA14 (37): NDS CS1
- PA15 (38): NDS CS2

GPIO B
- PB3 (41): W25Q CLK
- PB4 (42): W25Q DO/IO1
- PB5 (43): W25Q DI/IO0
- PB6 (44): W25Q CS
- PB14 (27): NDS I/O 6 (SPI2 MISO)

GPIO F
- PF1 (6): NDS CLK (SPI2 CLK)

GPIO G
- PG10-NRST (7): NDS RESET


## Implementation details

Most of the code is in a same file (for inlining ; but it will probably be split later).

Transfer is handled by DMA:
- DMA1 Channel1: reads the command from I/O pins when CLK gets low
- DMA1 Channel2: writes the data from the transferBuffer to I/O pins when CLK gets high
- DMA1 Channel3: switches GPIO to input mode after the last command byte
- DMA1 Channel4: enables the first two DMA channels on CS1 low
- DMA1 Channel5: if SPI is enabled, enables SPI2. Otherwise, stops the first two DMA1 channels on CS1 high.

DMA2 is used for SPI communication with the W25Q memory chip.

The flash mode commands (used to program the W25Q memory chip from a console) are temporary and shouldn't be documented until a future rework. (Also they're a joke)

**Interrupt handler:** most of the functions can be interrupted at any moment to ensure low latency with interrupts and fast loops. Let me explain.

A normal interrupt handler sets a flag and then resumes the code execution, which means that if I had a loop that wrote 1000h bytes to a buffer, I'd need to do something like this:

```c
volatile int interruptFlag;

for (int i = 0; i < 0x1000; i++) {
    transferBuffer[i] = (...);
    if (interruptFlag)
        break;
}
```

This is very slow: the function would have to return and cleanup everything when interruptFlag is set, and it has to check that flag every time it loops. This prevents writing low-latency code.

To increase performance, I've written another interrupt handler that immediately gives up on the current executing function and resumes in another function. Basically, it just calls longjmp. This allows me to write "while true" loops. Everything is cleaned up when CS1/CS2 goes low.

# Benchmark

Main data transfer: using GAP=12h, can do 750 consecutive reads with 1000h bytes length, or 6000 consecutive reads with 200h bytes.

Will need more benchmarking.

# To-do

- An actual flashcart with ROM patching and microSD card reading
- SPI NAND chip support 
- RP2040 version
- Code cleanup