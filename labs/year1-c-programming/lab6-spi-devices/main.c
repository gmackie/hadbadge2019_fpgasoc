#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * Lab 6: SPI Device Communication
 * ================================
 * The Serial Peripheral Interface (SPI) is a four-wire full-duplex bus
 * commonly used for flash memory, displays, and high-speed sensors.
 * Unlike I2C, SPI uses separate MOSI/MISO data lines plus a clock (SCK)
 * and per-device chip-select (CS#) lines.
 *
 * The badge has a memory-mapped SPI master peripheral.  You control it
 * by writing commands to the TX register, reading responses from RX,
 * and managing chip-select via a control register.
 *
 * EXERCISES:
 *
 * Exercise 1: Configure the SPI peripheral for Mode 0 (CPOL=0, CPHA=0)
 *             and set the clock divider for 1 MHz operation.
 *
 * Exercise 2: Read the JEDEC ID from an SPI flash chip.  Send command
 *             0x9F and read back three bytes (manufacturer, type, capacity).
 *
 * Exercise 3: Read data from flash at a given 24-bit address.  Send
 *             command 0x03 followed by a 3-byte address, then clock
 *             out N data bytes.
 *
 * Exercise 4: Write data to flash.  This requires WRITE ENABLE (0x06),
 *             SECTOR ERASE (0x20), polling STATUS until ready, then
 *             PAGE PROGRAM (0x02).
 *
 * Exercise 5 (challenge): Implement a simple SPI display driver that
 *             sends pixel data to an SPI-connected LCD.
 *
 * CONCEPTS:
 *   - SPI modes 0-3 (CPOL / CPHA clock polarity and phase)
 *   - Chip select management (directly via GPIO control register)
 *   - Full-duplex: every byte out clocks a byte in simultaneously
 *   - Flash memory commands: READ, WRITE ENABLE, SECTOR ERASE, READ STATUS
 *   - Busy-wait polling with status register
 */

/* ---- SPI peripheral registers (memory-mapped) ---- */
#define SPI_BASE        0x50020000
#define SPI_TX_DATA     (*(volatile uint32_t *)(SPI_BASE + 0x00))
#define SPI_RX_DATA     (*(volatile uint32_t *)(SPI_BASE + 0x04))
#define SPI_CTRL        (*(volatile uint32_t *)(SPI_BASE + 0x08))
#define SPI_STATUS      (*(volatile uint32_t *)(SPI_BASE + 0x0C))
#define SPI_CLK_DIV     (*(volatile uint32_t *)(SPI_BASE + 0x10))

/* CTRL register bits */
#define CTRL_CPOL       (1 << 0)   /* Clock polarity   */
#define CTRL_CPHA       (1 << 1)   /* Clock phase      */
#define CTRL_CS0        (1 << 8)   /* Chip select 0 (active low, directly driven) */
#define CTRL_CS1        (1 << 9)   /* Chip select 1    */
#define CTRL_ENABLE     (1 << 16)  /* Peripheral enable */

/* STATUS register bits */
#define STATUS_TX_READY (1 << 0)
#define STATUS_RX_AVAIL (1 << 1)
#define STATUS_BUSY     (1 << 2)

/* Common SPI flash commands */
#define FLASH_CMD_READ          0x03
#define FLASH_CMD_WRITE_EN      0x06
#define FLASH_CMD_WRITE_DIS     0x04
#define FLASH_CMD_READ_STATUS   0x05
#define FLASH_CMD_PAGE_PROGRAM  0x02
#define FLASH_CMD_SECTOR_ERASE  0x20
#define FLASH_CMD_JEDEC_ID      0x9F

/* Flash status register bits */
#define FLASH_STATUS_BUSY       (1 << 0)
#define FLASH_STATUS_WEL        (1 << 1)

static FILE *console;

/* ---- SPI helper functions ---- */

/* Transfer a single byte (full-duplex: sends and receives simultaneously) */
static uint8_t spi_transfer(uint8_t tx_byte) {
	while (!(SPI_STATUS & STATUS_TX_READY))
		;
	SPI_TX_DATA = tx_byte;
	while (SPI_STATUS & STATUS_BUSY)
		;
	return (uint8_t)SPI_RX_DATA;
}

/* Assert chip select N (active-low: clear the CS bit) */
static void spi_cs_assert(int n) {
	uint32_t ctrl = SPI_CTRL;
	ctrl &= ~(CTRL_CS0 << n);   /* CS lines idle high; clear = assert */
	SPI_CTRL = ctrl;
}

/* De-assert chip select (release: set the CS bit high) */
static void spi_cs_deassert(void) {
	SPI_CTRL |= (CTRL_CS0 | CTRL_CS1);
}

/* Wait until the flash's internal write/erase operation completes */
static void flash_wait_ready(void) {
	uint8_t status;
	do {
		spi_cs_assert(0);
		spi_transfer(FLASH_CMD_READ_STATUS);
		status = spi_transfer(0xFF);   /* dummy byte clocks out status */
		spi_cs_deassert();
	} while (status & FLASH_STATUS_BUSY);
}

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;
	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	fprintf(console, "\033C");
	fprintf(console, "Lab 6: SPI Devices\n\n");

	/* === Exercise 1: Configure SPI for Mode 0, 1 MHz ===
	 * Mode 0: CPOL=0, CPHA=0 (clock idles low, sample on rising edge).
	 * Clock divider: SCL = 48 MHz / (2 * (div + 1)).
	 * For 1 MHz: div = (48000000 / (2 * 1000000)) - 1 = 23.
	 * TODO: Write the control and clock divider registers.
	 */
	// SPI_CLK_DIV = 23;                                 /* 1 MHz */
	// SPI_CTRL = CTRL_ENABLE | CTRL_CS0 | CTRL_CS1;     /* Mode 0, CS idle high */
	// fprintf(console, "SPI: Mode 0, 1 MHz\n");
	//
	// /* Mode 3 would be: SPI_CTRL |= CTRL_CPOL | CTRL_CPHA; */

	/* === Exercise 2: Read JEDEC ID ===
	 * Transaction: assert CS, send 0x9F, read 3 bytes, deassert CS.
	 * Expected for W25Q128: manufacturer=0xEF, type=0x40, capacity=0x18.
	 * TODO: Read and display the JEDEC ID.
	 */
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_JEDEC_ID);
	// uint8_t mfr  = spi_transfer(0xFF);   /* dummy TX to clock in RX */
	// uint8_t type = spi_transfer(0xFF);
	// uint8_t cap  = spi_transfer(0xFF);
	// spi_cs_deassert();
	// fprintf(console, "JEDEC ID: %02X %02X %02X\n", mfr, type, cap);

	/* === Exercise 3: Read flash data at an address ===
	 * Send READ command (0x03) + 24-bit address + N dummy bytes.
	 * The flash returns one data byte per dummy clock.
	 * TODO: Read 16 bytes starting at address 0x000000.
	 */
	// uint32_t addr = 0x000000;
	// uint8_t buf[16];
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_READ);
	// spi_transfer((addr >> 16) & 0xFF);   /* A23..A16 */
	// spi_transfer((addr >> 8)  & 0xFF);   /* A15..A8  */
	// spi_transfer((addr >> 0)  & 0xFF);   /* A7..A0   */
	// for (int i = 0; i < 16; i++)
	//     buf[i] = spi_transfer(0xFF);
	// spi_cs_deassert();
	//
	// fprintf(console, "Data @0x%06lX:\n", (unsigned long)addr);
	// for (int i = 0; i < 16; i++)
	//     fprintf(console, "%02X ", buf[i]);
	// fprintf(console, "\n");

	/* === Exercise 4: Write data to flash ===
	 * Writing to flash requires: WRITE ENABLE -> SECTOR ERASE ->
	 * wait ready -> WRITE ENABLE -> PAGE PROGRAM -> wait ready.
	 * A sector is 4 KB; a page program writes up to 256 bytes.
	 * TODO: Erase sector at 0x010000, then write a test pattern.
	 */
	// /* Step 1: Enable writes */
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_WRITE_EN);
	// spi_cs_deassert();
	//
	// /* Step 2: Erase the 4 KB sector containing 0x010000 */
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_SECTOR_ERASE);
	// spi_transfer(0x01);   /* A23..A16 */
	// spi_transfer(0x00);   /* A15..A8  */
	// spi_transfer(0x00);   /* A7..A0   */
	// spi_cs_deassert();
	// flash_wait_ready();   /* Erase takes ~40 ms */
	// fprintf(console, "Sector erased\n");
	//
	// /* Step 3: Write enable again for page program */
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_WRITE_EN);
	// spi_cs_deassert();
	//
	// /* Step 4: Page program (write up to 256 bytes) */
	// spi_cs_assert(0);
	// spi_transfer(FLASH_CMD_PAGE_PROGRAM);
	// spi_transfer(0x01);   /* A23..A16 */
	// spi_transfer(0x00);   /* A15..A8  */
	// spi_transfer(0x00);   /* A7..A0   */
	// for (int i = 0; i < 16; i++)
	//     spi_transfer(0xA0 + i);   /* test pattern */
	// spi_cs_deassert();
	// flash_wait_ready();
	// fprintf(console, "Page programmed\n");

	/* === Exercise 5 (challenge): SPI display driver ===
	 * Many small LCDs use SPI with a Data/Command (DC) pin.
	 * To draw a pixel: set DC=0 for commands, DC=1 for data.
	 * TODO: Implement a function that sends a block of pixel data.
	 */
	// #define DC_CMD   0
	// #define DC_DATA  1
	// static void lcd_set_dc(int dc) {
	//     /* Toggle a GPIO pin for Data/Command select */
	//     if (dc) MISC_REG(MISC_LED_REG) |=  (1 << 8);
	//     else    MISC_REG(MISC_LED_REG) &= ~(1 << 8);
	// }
	//
	// static void lcd_send_cmd(uint8_t cmd) {
	//     lcd_set_dc(DC_CMD);
	//     spi_cs_assert(1);
	//     spi_transfer(cmd);
	//     spi_cs_deassert();
	// }
	//
	// static void lcd_send_pixels(uint16_t *pixels, int count) {
	//     lcd_set_dc(DC_DATA);
	//     spi_cs_assert(1);
	//     for (int i = 0; i < count; i++) {
	//         spi_transfer(pixels[i] >> 8);    /* high byte (RGB565) */
	//         spi_transfer(pixels[i] & 0xFF);  /* low byte           */
	//     }
	//     spi_cs_deassert();
	// }

	fprintf(console, "\n\nPress START to exit");
	while (!(MISC_REG(MISC_BTN_REG) & BUTTON_START))
		;
	wait_for_button_release();
}
