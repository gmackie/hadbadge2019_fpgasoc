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
 * CAN Bus Test Program
 *
 * Demonstrates CAN 2.0B transmit and receive. Connect a CAN
 * transceiver (e.g. MCP2551, SN65HVD230) to the badge's PMOD
 * or SAO connector.
 *
 * Controls:
 *   A      - Send a standard frame (11-bit ID)
 *   B      - Send an extended frame (29-bit ID)
 *   UP     - Increase TX data counter
 *   DOWN   - Check for received frames
 *   START  - Exit
 */

static FILE *console;
static uint8_t tx_counter = 0;
static int tx_count = 0;
static int rx_count = 0;

static void can_init(uint16_t prescaler, uint8_t tseg1, uint8_t tseg2) {
	CAN_REG(CAN_PRESCALER_REG) = prescaler;
	CAN_REG(CAN_TIMING_REG) = (tseg2 << 4) | tseg1;
}

static int can_tx_standard(uint16_t id, const uint8_t *data, uint8_t dlc) {
	// Wait for any previous TX to complete
	while (CAN_REG(CAN_CMD_REG) & CAN_STATUS_BUSY)
		;

	// Set up standard frame
	CAN_REG(CAN_TX_ID_REG) = id & 0x7FF;
	CAN_REG(CAN_TX_CTRL_REG) = dlc & 0x0F;  // Standard frame, no RTR

	// Pack data bytes (big-endian)
	uint32_t d0 = 0, d1 = 0;
	for (int i = 0; i < 4 && i < dlc; i++)
		d0 |= (uint32_t)data[i] << (24 - i * 8);
	for (int i = 4; i < 8 && i < dlc; i++)
		d1 |= (uint32_t)data[i] << (24 - (i - 4) * 8);

	CAN_REG(CAN_TX_DATA0_REG) = d0;
	CAN_REG(CAN_TX_DATA1_REG) = d1;

	// Start transmission
	CAN_REG(CAN_CMD_REG) = CAN_CMD_TX_START;

	// Wait for completion
	uint32_t status;
	do {
		status = CAN_REG(CAN_CMD_REG);
	} while (status & CAN_STATUS_BUSY);

	if (status & CAN_STATUS_ARBLOST)
		return -2;

	return 0;
}

static int can_tx_extended(uint32_t id, const uint8_t *data, uint8_t dlc) {
	while (CAN_REG(CAN_CMD_REG) & CAN_STATUS_BUSY)
		;

	CAN_REG(CAN_TX_ID_REG) = id & 0x1FFFFFFF;
	CAN_REG(CAN_TX_CTRL_REG) = (dlc & 0x0F) | CAN_CTRL_IDE;

	uint32_t d0 = 0, d1 = 0;
	for (int i = 0; i < 4 && i < dlc; i++)
		d0 |= (uint32_t)data[i] << (24 - i * 8);
	for (int i = 4; i < 8 && i < dlc; i++)
		d1 |= (uint32_t)data[i] << (24 - (i - 4) * 8);

	CAN_REG(CAN_TX_DATA0_REG) = d0;
	CAN_REG(CAN_TX_DATA1_REG) = d1;

	CAN_REG(CAN_CMD_REG) = CAN_CMD_TX_START;

	uint32_t status;
	do {
		status = CAN_REG(CAN_CMD_REG);
	} while (status & CAN_STATUS_BUSY);

	if (status & CAN_STATUS_ARBLOST)
		return -2;

	return 0;
}

static void display_status(void) {
	fprintf(console, "\033C");
	fprintf(console, "\0330X\0330Y");
	fprintf(console, "CAN Bus Test (100kbps)\n\n");

	uint32_t status = CAN_REG(CAN_CMD_REG);
	fprintf(console, "Status: %s%s\n",
		(status & CAN_STATUS_BUSY) ? "BUSY " : "IDLE ",
		(status & CAN_STATUS_BUS_OFF) ? "BUS-OFF" : "OK");
	fprintf(console, "TX sent: %d  RX recv: %d\n", tx_count, rx_count);
	fprintf(console, "TX counter: 0x%02X\n\n", tx_counter);

	fprintf(console, "A=TX Std  B=TX Ext\n");
	fprintf(console, "UP=Inc counter\n");
	fprintf(console, "DOWN=Check RX  START=Exit\n");
}

static void send_standard(void) {
	uint8_t data[8] = {tx_counter, 0xCA, 0xFE, 0xBA, 0xBE, 0x00, 0x00, tx_counter};

	fprintf(console, "\nTX Std ID=0x123 DLC=8: ");
	int ret = can_tx_standard(0x123, data, 8);
	if (ret == 0) {
		fprintf(console, "OK\n");
		tx_count++;
		tx_counter++;
	} else if (ret == -2) {
		fprintf(console, "ARB LOST\n");
	} else {
		fprintf(console, "ERROR\n");
	}
}

static void send_extended(void) {
	uint8_t data[4] = {tx_counter, 0xDE, 0xAD, tx_counter};

	fprintf(console, "\nTX Ext ID=0x1234ABCD DLC=4: ");
	int ret = can_tx_extended(0x1234ABCD, data, 4);
	if (ret == 0) {
		fprintf(console, "OK\n");
		tx_count++;
		tx_counter++;
	} else if (ret == -2) {
		fprintf(console, "ARB LOST\n");
	} else {
		fprintf(console, "ERROR\n");
	}
}

static void check_rx(void) {
	uint32_t status = CAN_REG(CAN_CMD_REG);

	if (!(status & CAN_STATUS_RX_VALID)) {
		fprintf(console, "\nNo RX frame\n");
		return;
	}

	uint32_t rx_id   = CAN_REG(CAN_RX_ID_REG);
	uint32_t rx_ctrl = CAN_REG(CAN_RX_CTRL_REG);
	uint32_t rx_d0   = CAN_REG(CAN_RX_DATA0_REG);
	uint32_t rx_d1   = CAN_REG(CAN_RX_DATA1_REG);
	uint8_t dlc = rx_ctrl & 0x0F;
	int ide = (rx_ctrl & CAN_CTRL_IDE) ? 1 : 0;
	int rtr = (rx_ctrl & CAN_CTRL_RTR) ? 1 : 0;

	rx_count++;
	fprintf(console, "\nRX #%d: ", rx_count);
	if (ide)
		fprintf(console, "EXT ID=0x%08lX", (unsigned long)rx_id);
	else
		fprintf(console, "STD ID=0x%03lX", (unsigned long)(rx_id & 0x7FF));

	fprintf(console, " DLC=%d%s\n", dlc, rtr ? " RTR" : "");

	if (dlc > 0 && !rtr) {
		fprintf(console, "Data: ");
		uint8_t bytes[8];
		bytes[0] = (rx_d0 >> 24) & 0xFF;
		bytes[1] = (rx_d0 >> 16) & 0xFF;
		bytes[2] = (rx_d0 >> 8)  & 0xFF;
		bytes[3] = rx_d0 & 0xFF;
		bytes[4] = (rx_d1 >> 24) & 0xFF;
		bytes[5] = (rx_d1 >> 16) & 0xFF;
		bytes[6] = (rx_d1 >> 8)  & 0xFF;
		bytes[7] = rx_d1 & 0xFF;
		for (int i = 0; i < dlc && i < 8; i++)
			fprintf(console, "%02X ", bytes[i]);
		fprintf(console, "\n");
	}

	if (status & CAN_STATUS_RX_CRC_ERR)
		fprintf(console, "WARNING: CRC error!\n");
}

void main(int argc, char **argv) {
	printf("CAN Test: starting\n");

	GFX_REG(GFX_BGNDCOL_REG) = 0x102010;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;

	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	// Initialize CAN at 100kbps
	// 48MHz / (48 * (1 + 5 + 4)) = 100kbps
	can_init(48, 5, 4);

	display_status();

	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);

		if (btn & BUTTON_A) {
			wait_for_button_release();
			send_standard();
		}

		if (btn & BUTTON_B) {
			wait_for_button_release();
			send_extended();
		}

		if (btn & BUTTON_UP) {
			wait_for_button_release();
			tx_counter++;
			display_status();
		}

		if (btn & BUTTON_DOWN) {
			wait_for_button_release();
			check_rx();
		}

		if (btn & BUTTON_START) {
			break;
		}
	}

	wait_for_button_release();
	printf("CAN Test: done\n");
}
