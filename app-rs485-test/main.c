#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * RS-485 Test Program
 *
 * Half-duplex RS-485 communication test. Connect an RS-485
 * transceiver (e.g. MAX485, SN75176) to the badge's PMOD
 * or SAO connector.
 *
 * The program acts as a simple terminal / echo tester:
 * - Sends incrementing test messages on button press
 * - Continuously polls for received data
 * - Displays TX/RX statistics
 *
 * Controls:
 *   A      - Send test message
 *   B      - Send ping (short message)
 *   UP     - Increase baud rate
 *   DOWN   - Decrease baud rate
 *   START  - Exit
 */

static FILE *console;
static int tx_count = 0;
static int rx_count = 0;
static uint8_t msg_seq = 0;

// Baud rate table
static const struct {
	uint32_t baud;
	uint16_t div;
} baud_table[] = {
	{   9600, 4998 },
	{  19200, 2498 },
	{  38400, 1248 },
	{  57600,  830 },
	{ 115200,  414 },
};
#define BAUD_COUNT (sizeof(baud_table) / sizeof(baud_table[0]))
static int baud_idx = 4;  // Default 115200

static void rs485_init(uint16_t div, uint8_t turnaround) {
	RS485_REG(RS485_CTRL_REG) = div;
	RS485_REG(RS485_CONFIG_REG) = turnaround;  // Active-high DE, 2 bit turnaround
}

static void rs485_send(const uint8_t *data, int len) {
	for (int i = 0; i < len; i++) {
		RS485_REG(RS485_DATA_REG) = data[i];
	}
}

static int rs485_recv(uint8_t *buf, int maxlen) {
	int count = 0;
	while (count < maxlen) {
		uint32_t val = RS485_REG(RS485_DATA_REG);
		if (val & (1u << 31))
			break;  // RX FIFO empty
		buf[count++] = val & 0xFF;
	}
	return count;
}

static void display_status(void) {
	fprintf(console, "\033C");
	fprintf(console, "\0330X\0330Y");
	fprintf(console, "RS-485 Test (%lu baud)\n\n",
		(unsigned long)baud_table[baud_idx].baud);

	uint32_t ctrl = RS485_REG(RS485_CTRL_REG);
	fprintf(console, "TX FIFO: %s  RX FIFO: %s\n",
		(ctrl & (1u << 29)) ? "empty" : "data",
		(ctrl & (1u << 31)) ? "empty" : "data");
	if (ctrl & (1u << 30))
		fprintf(console, "WARNING: RX overflow!\n");

	fprintf(console, "TX msgs: %d  RX bytes: %d\n", tx_count, rx_count);
	fprintf(console, "Seq: %d\n\n", msg_seq);

	fprintf(console, "A=Send msg  B=Ping\n");
	fprintf(console, "UP/DOWN=Baud  START=Exit\n");
}

// Receive buffer for display
static uint8_t rx_buf[64];

static void poll_rx(void) {
	int n = rs485_recv(rx_buf, sizeof(rx_buf));
	if (n > 0) {
		rx_count += n;
		fprintf(console, "\nRX(%d): ", n);
		for (int i = 0; i < n && i < 32; i++) {
			if (rx_buf[i] >= 0x20 && rx_buf[i] < 0x7F)
				fprintf(console, "%c", rx_buf[i]);
			else
				fprintf(console, "<%02X>", rx_buf[i]);
		}
		if (n > 32)
			fprintf(console, "...");
		fprintf(console, "\n");
	}
}

static void send_test_msg(void) {
	char msg[32];
	int len = snprintf(msg, sizeof(msg), "BADGE#%03d TEST\r\n", msg_seq);
	rs485_send((uint8_t *)msg, len);
	tx_count++;
	msg_seq++;
	fprintf(console, "\nTX: %s", msg);
}

static void send_ping(void) {
	const uint8_t ping[] = { 'P', 'I', 'N', 'G', '\r', '\n' };
	rs485_send(ping, sizeof(ping));
	tx_count++;
	fprintf(console, "\nTX: PING\n");
}

void main(int argc, char **argv) {
	printf("RS-485 Test: starting\n");

	GFX_REG(GFX_BGNDCOL_REG) = 0x181018;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;

	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	// Initialize RS-485 at 115200 baud, 2-bit turnaround, active-high DE
	rs485_init(baud_table[baud_idx].div, 2);

	display_status();

	while (1) {
		// Always check for incoming data
		poll_rx();

		uint32_t btn = MISC_REG(MISC_BTN_REG);

		if (btn & BUTTON_A) {
			wait_for_button_release();
			send_test_msg();
		}

		if (btn & BUTTON_B) {
			wait_for_button_release();
			send_ping();
		}

		if (btn & BUTTON_UP) {
			wait_for_button_release();
			if (baud_idx < (int)BAUD_COUNT - 1) {
				baud_idx++;
				rs485_init(baud_table[baud_idx].div, 2);
				display_status();
			}
		}

		if (btn & BUTTON_DOWN) {
			wait_for_button_release();
			if (baud_idx > 0) {
				baud_idx--;
				rs485_init(baud_table[baud_idx].div, 2);
				display_status();
			}
		}

		if (btn & BUTTON_START) {
			break;
		}
	}

	wait_for_button_release();
	printf("RS-485 Test: done\n");
}
