#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * Lab 5: I2C Sensor Communication
 * ================================
 * The Inter-Integrated Circuit (I2C) bus is a two-wire serial protocol
 * used to communicate with sensors, EEPROMs, and other peripherals.
 * The badge has a memory-mapped I2C master peripheral that we control
 * by writing to command, data, and clock divider registers.
 *
 * I2C uses open-drain signaling on SDA (data) and SCL (clock).
 * The master always drives the clock. Devices are addressed with a
 * 7-bit address; the 8th bit selects read (1) or write (0).
 *
 * EXERCISES:
 *
 * Exercise 1: Configure the I2C clock divider for 100 kHz operation.
 *             The formula is: SCL = 48 MHz / (4 * (divider + 1)).
 *             Calculate the correct divider and write it to I2C_CLK_DIV.
 *
 * Exercise 2: Read the device ID (WHO_AM_I) register from a sensor
 *             at address 0x68.  Send START + addr(write), register
 *             number, repeated START + addr(read), read one byte, STOP.
 *
 * Exercise 3: Read a 16-bit temperature value from registers 0x41-0x42.
 *             This requires writing the register address, then reading
 *             two consecutive bytes (ACK after first, NACK after last).
 *
 * Exercise 4: Implement a multi-byte burst read function that reads
 *             N bytes starting from a given register address.
 *
 * Exercise 5 (challenge): Write an I2C bus scanner that probes all
 *             127 possible addresses and prints which ones ACK.
 *
 * CONCEPTS:
 *   - I2C protocol: START condition, STOP condition, ACK/NACK
 *   - Open-drain signaling (wired-AND, external pull-up resistors)
 *   - 7-bit addressing (address byte = addr<<1 | R/W bit)
 *   - Register-based sensor access (write reg addr, then read data)
 *   - Bus scanning to discover connected devices
 */

/* ---- I2C peripheral registers (memory-mapped) ---- */
#define I2C_BASE        0x50010000
#define I2C_CMD         (*(volatile uint32_t *)(I2C_BASE + 0x00))
#define I2C_STATUS      (*(volatile uint32_t *)(I2C_BASE + 0x04))
#define I2C_WRITE_DATA  (*(volatile uint32_t *)(I2C_BASE + 0x08))
#define I2C_READ_DATA   (*(volatile uint32_t *)(I2C_BASE + 0x0C))
#define I2C_CLK_DIV     (*(volatile uint32_t *)(I2C_BASE + 0x10))

/* Command bits */
#define CMD_START   (1 << 0)
#define CMD_STOP    (1 << 1)
#define CMD_WRITE   (1 << 2)
#define CMD_READ    (1 << 3)
#define CMD_ACK     (0 << 4)   /* Send ACK after read  */
#define CMD_NACK    (1 << 4)   /* Send NACK after read */

/* Status bits */
#define STATUS_BUSY (1 << 0)
#define STATUS_NACK (1 << 2)

/* Sensor address and register map (MPU-6050 style) */
#define SENSOR_ADDR     0x68
#define REG_WHO_AM_I    0x75
#define REG_TEMP_H      0x41
#define REG_TEMP_L      0x42
#define REG_ACCEL_XOUT  0x3B
#define REG_PWR_MGMT_1  0x6B

static FILE *console;

/* ---- Low-level I2C helpers ---- */

/* Spin until the peripheral finishes the current operation */
static void i2c_wait_busy(void) {
	while (I2C_STATUS & STATUS_BUSY)
		;
}

/* Send a START condition followed by one byte; return 0 on ACK */
static int i2c_start_write(uint8_t byte) {
	I2C_WRITE_DATA = byte;
	I2C_CMD = CMD_START | CMD_WRITE;
	i2c_wait_busy();
	return (I2C_STATUS & STATUS_NACK) ? -1 : 0;
}

/* Write one data byte (no START); return 0 on ACK */
static int i2c_write_byte(uint8_t byte) {
	I2C_WRITE_DATA = byte;
	I2C_CMD = CMD_WRITE;
	i2c_wait_busy();
	return (I2C_STATUS & STATUS_NACK) ? -1 : 0;
}

/* Read one byte and send ACK (more bytes follow) */
static uint8_t i2c_read_ack(void) {
	I2C_CMD = CMD_READ | CMD_ACK;
	i2c_wait_busy();
	return (uint8_t)I2C_READ_DATA;
}

/* Read one byte and send NACK (last byte in transfer) */
static uint8_t i2c_read_nack(void) {
	I2C_CMD = CMD_READ | CMD_NACK;
	i2c_wait_busy();
	return (uint8_t)I2C_READ_DATA;
}

/* Issue a STOP condition to release the bus */
static void i2c_stop(void) {
	I2C_CMD = CMD_STOP;
	i2c_wait_busy();
}

/* Read a single 8-bit register from a device */
static int i2c_read_reg(uint8_t dev, uint8_t reg, uint8_t *val) {
	/* Phase 1: write the register address */
	if (i2c_start_write((dev << 1) | 0)) { i2c_stop(); return -1; }
	if (i2c_write_byte(reg))              { i2c_stop(); return -1; }

	/* Phase 2: repeated START in read mode */
	if (i2c_start_write((dev << 1) | 1))  { i2c_stop(); return -1; }
	*val = i2c_read_nack();
	i2c_stop();
	return 0;
}

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;
	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	fprintf(console, "\033C");
	fprintf(console, "Lab 5: I2C Sensors\n\n");

	/* === Exercise 1: Configure I2C clock divider ===
	 * System clock = 48 MHz.  SCL = 48 MHz / (4 * (div + 1)).
	 * For 100 kHz standard mode: div = (48000000 / (4 * 100000)) - 1 = 119
	 * TODO: Calculate and write the correct divider value.
	 */
	// I2C_CLK_DIV = 119;   /* 100 kHz */
	// fprintf(console, "I2C clk div = %d (100 kHz)\n", 119);
	//
	// For 400 kHz fast mode the divider would be:
	// I2C_CLK_DIV = 29;

	/* === Exercise 2: Read WHO_AM_I register ===
	 * The MPU-6050 returns 0x68 from register 0x75.
	 * Bus transaction:
	 *   [S] [0x68<<1|0] [ACK] [0x75] [ACK] [Sr] [0x68<<1|1] [ACK] [data] [NACK] [P]
	 * TODO: Use i2c_read_reg() and print the result.
	 */
	// uint8_t who = 0;
	// if (i2c_read_reg(SENSOR_ADDR, REG_WHO_AM_I, &who) == 0) {
	//     fprintf(console, "WHO_AM_I = 0x%02X\n", who);
	// } else {
	//     fprintf(console, "No ACK from 0x%02X!\n", SENSOR_ADDR);
	// }

	/* === Exercise 3: Read 16-bit temperature ===
	 * Temperature is stored big-endian across REG_TEMP_H and REG_TEMP_L.
	 * ACK after the first byte, NACK after the second.
	 * TODO: Perform a two-byte read and combine the result.
	 */
	// if (i2c_start_write((SENSOR_ADDR << 1) | 0) == 0) {
	//     i2c_write_byte(REG_TEMP_H);
	//     i2c_start_write((SENSOR_ADDR << 1) | 1);
	//     uint8_t hi = i2c_read_ack();    /* ACK  - more data coming */
	//     uint8_t lo = i2c_read_nack();   /* NACK - last byte        */
	//     i2c_stop();
	//     int16_t raw = (int16_t)((hi << 8) | lo);
	//     /* MPU-6050 formula: Temp_C = raw / 340.0 + 36.53 */
	//     fprintf(console, "Temp raw=0x%04X\n", (unsigned)raw & 0xFFFF);
	// }

	/* === Exercise 4: Multi-byte burst read ===
	 * TODO: Write a function that reads N consecutive registers.
	 * Prototype: int i2c_burst_read(uint8_t dev, uint8_t start_reg,
	 *                               uint8_t *buf, int len);
	 */
	// int i2c_burst_read(uint8_t dev, uint8_t reg, uint8_t *buf, int n) {
	//     if (i2c_start_write((dev << 1) | 0)) { i2c_stop(); return -1; }
	//     i2c_write_byte(reg);
	//     if (i2c_start_write((dev << 1) | 1)) { i2c_stop(); return -1; }
	//     for (int i = 0; i < n; i++)
	//         buf[i] = (i < n - 1) ? i2c_read_ack() : i2c_read_nack();
	//     i2c_stop();
	//     return 0;
	// }
	//
	// /* Example: read 6 bytes of accelerometer data */
	// uint8_t accel[6];
	// if (i2c_burst_read(SENSOR_ADDR, REG_ACCEL_XOUT, accel, 6) == 0) {
	//     int16_t ax = (accel[0] << 8) | accel[1];
	//     int16_t ay = (accel[2] << 8) | accel[3];
	//     int16_t az = (accel[4] << 8) | accel[5];
	//     fprintf(console, "Accel X=%d Y=%d Z=%d\n", ax, ay, az);
	// }

	/* === Exercise 5 (challenge): I2C Bus Scanner ===
	 * Probe every 7-bit address (0x03 .. 0x77).  For each address
	 * send START + address(write).  If ACK, a device is present.
	 * Print a grid like the Linux i2cdetect tool.
	 * TODO: Implement the scanner loop.
	 */
	// fprintf(console, "\nI2C Bus Scan:\n");
	// fprintf(console, "     0  1  2  3  4  5  6  7"
	//                  "  8  9  A  B  C  D  E  F\n");
	// int found = 0;
	// for (int addr = 0; addr < 128; addr++) {
	//     if ((addr & 0x0F) == 0)
	//         fprintf(console, "%02X: ", addr);
	//     if (addr < 0x03 || addr > 0x77) {
	//         fprintf(console, "   ");
	//     } else {
	//         int ack = i2c_start_write((addr << 1) | 0);
	//         i2c_stop();
	//         if (ack == 0) {
	//             fprintf(console, "%02X ", addr);
	//             found++;
	//         } else {
	//             fprintf(console, "-- ");
	//         }
	//     }
	//     if ((addr & 0x0F) == 0x0F)
	//         fprintf(console, "\n");
	// }
	// fprintf(console, "Found %d device(s)\n", found);

	fprintf(console, "\n\nPress START to exit");
	while (!(MISC_REG(MISC_BTN_REG) & BUTTON_START))
		;
	wait_for_button_release();
}
