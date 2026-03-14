#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * I2C Test Program
 *
 * Scans the I2C bus for devices, then demonstrates read/write
 * operations. Connect an I2C device (e.g. EEPROM, sensor) to
 * the badge's SAO or PMOD connector.
 *
 * Controls:
 *   A      - Run I2C bus scan
 *   B      - Read from detected device
 *   START  - Exit
 */

static FILE *console;

static void i2c_init(uint16_t clkdiv) {
	I2C_REG(I2C_CLKDIV_REG) = clkdiv;
}

static void i2c_wait(void) {
	while (I2C_REG(I2C_CMD_REG) & I2C_STATUS_BUSY)
		;
}

static int i2c_start_write(uint8_t addr) {
	// Send START + address with write bit
	I2C_REG(I2C_DATA_REG) = (addr << 1) | 0;
	I2C_REG(I2C_CMD_REG) = I2C_CMD_START | I2C_CMD_WRITE;
	i2c_wait();
	return (I2C_REG(I2C_CMD_REG) & I2C_STATUS_NACK) ? -1 : 0;
}

static int i2c_start_read(uint8_t addr) {
	// Send START + address with read bit
	I2C_REG(I2C_DATA_REG) = (addr << 1) | 1;
	I2C_REG(I2C_CMD_REG) = I2C_CMD_START | I2C_CMD_WRITE;
	i2c_wait();
	return (I2C_REG(I2C_CMD_REG) & I2C_STATUS_NACK) ? -1 : 0;
}

static int i2c_write_byte(uint8_t data) {
	I2C_REG(I2C_DATA_REG) = data;
	I2C_REG(I2C_CMD_REG) = I2C_CMD_WRITE;
	i2c_wait();
	return (I2C_REG(I2C_CMD_REG) & I2C_STATUS_NACK) ? -1 : 0;
}

static uint8_t i2c_read_byte(int last) {
	// For the last byte, send NACK to signal end of read
	I2C_REG(I2C_CMD_REG) = I2C_CMD_READ | (last ? I2C_CMD_NACK : 0);
	i2c_wait();
	return I2C_REG(I2C_DATA_REG) & 0xFF;
}

static void i2c_stop(void) {
	I2C_REG(I2C_CMD_REG) = I2C_CMD_STOP;
	i2c_wait();
}

// Probe a single I2C address, returns 0 if device responds
static int i2c_probe(uint8_t addr) {
	int ret = i2c_start_write(addr);
	i2c_stop();
	return ret;
}

static uint8_t found_addrs[128];
static int found_count = 0;

static void i2c_scan(void) {
	found_count = 0;
	fprintf(console, "\033C"); // clear console
	fprintf(console, "\0330X\0330Y");
	fprintf(console, "I2C Bus Scan (100kHz)\n\n");
	fprintf(console, "     0  1  2  3  4  5  6  7");
	fprintf(console, "  8  9  A  B  C  D  E  F\n");

	for (int row = 0; row < 8; row++) {
		fprintf(console, "%02X: ", row << 4);
		for (int col = 0; col < 16; col++) {
			uint8_t addr = (row << 4) | col;

			// Skip reserved addresses
			if (addr < 0x08 || addr > 0x77) {
				fprintf(console, "   ");
				continue;
			}

			if (i2c_probe(addr) == 0) {
				fprintf(console, "%02X ", addr);
				if (found_count < 128)
					found_addrs[found_count++] = addr;
			} else {
				fprintf(console, "-- ");
			}
		}
		fprintf(console, "\n");
	}

	fprintf(console, "\nFound %d device(s)\n", found_count);
	if (found_count > 0) {
		fprintf(console, "Press B to read from 0x%02X\n", found_addrs[0]);
	}
	fprintf(console, "A=Rescan START=Exit\n");
}

static void i2c_read_demo(void) {
	if (found_count == 0) {
		fprintf(console, "No devices found!\n");
		return;
	}

	uint8_t addr = found_addrs[0];
	fprintf(console, "\nReading 16 bytes from 0x%02X:\n", addr);

	// Try to read register 0x00 onwards
	if (i2c_start_write(addr) < 0) {
		fprintf(console, "NACK on address\n");
		i2c_stop();
		return;
	}

	// Write register address 0x00
	i2c_write_byte(0x00);

	// Repeated start for read
	if (i2c_start_read(addr) < 0) {
		fprintf(console, "NACK on read\n");
		i2c_stop();
		return;
	}

	// Read 16 bytes
	for (int i = 0; i < 16; i++) {
		uint8_t data = i2c_read_byte(i == 15);
		fprintf(console, "%02X ", data);
		if (i == 7) fprintf(console, "\n");
	}
	fprintf(console, "\n");

	i2c_stop();
}

void main(int argc, char **argv) {
	printf("I2C Test: starting\n");

	// Set up console on tile layer A
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;

	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	// Initialize I2C at 100kHz (divider=119 at 48MHz)
	i2c_init(119);

	// Do initial scan
	i2c_scan();

	// Main loop
	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);

		if (btn & BUTTON_A) {
			wait_for_button_release();
			i2c_scan();
		}

		if (btn & BUTTON_B) {
			wait_for_button_release();
			i2c_read_demo();
		}

		if (btn & BUTTON_START) {
			break;
		}
	}

	wait_for_button_release();
	printf("I2C Test: done\n");
}
