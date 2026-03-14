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
 * harness_display.c - LCD display companion for digital logic labs
 *
 * This program runs on the RISC-V and reads the student module's
 * state from the test harness status registers, then displays it
 * on the LCD as a logic analyzer-style view.
 *
 * Students don't need to modify this file. They write their Verilog
 * module and this program automatically visualizes its behavior.
 */

// The test harness status registers are mapped at 0xD0000000
// (Set this to match the actual harness address in the SoC)
#define HARNESS_OFFSET 0xD0000000
extern volatile uint32_t HARNESS[];
#define HARNESS_REG(i) HARNESS[(i)/4]

#define HARNESS_INPUTS_REG   0x00
#define HARNESS_OUTPUTS_REG  0x04
#define HARNESS_STATE_LO_REG 0x08
#define HARNESS_STATE_HI_REG 0x0C

static FILE *console;

static void print_binary(FILE *f, uint32_t val, int bits) {
	for (int i = bits - 1; i >= 0; i--) {
		fprintf(f, "%c", (val & (1u << i)) ? '1' : '0');
		if (i > 0 && i % 4 == 0) fprintf(f, " ");
	}
}

static void draw_signal_bar(FILE *f, const char *name, int val, int width) {
	fprintf(f, "%-8s ", name);
	for (int i = 0; i < width; i++) {
		fprintf(f, "%c", val ? '#' : '_');
	}
	fprintf(f, " %d\n", val);
}

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x000810;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;

	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	// Signal names (customize per lab)
	const char *input_names[] = {
		"UP", "DOWN", "LEFT", "RIGHT", "A", "B", "SEL", "START"
	};
	const char *output_names[] = {
		"OUT0", "OUT1", "OUT2", "OUT3", "OUT4", "OUT5", "OUT6", "OUT7"
	};

	uint32_t frame = 0;

	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);
		if ((btn & BUTTON_SELECT) && (btn & BUTTON_START))
			break;

		// Read harness status
		uint32_t inputs  = HARNESS_REG(HARNESS_INPUTS_REG);
		uint32_t outputs = HARNESS_REG(HARNESS_OUTPUTS_REG);
		uint32_t state_lo = HARNESS_REG(HARNESS_STATE_LO_REG);
		uint32_t state_hi = HARNESS_REG(HARNESS_STATE_HI_REG);

		// Update display every 4 frames (~15 fps text update)
		if ((frame & 3) == 0) {
			fprintf(console, "\033C");
			fprintf(console, " Digital Logic Test Harness\n");
			fprintf(console, " -------------------------\n\n");

			// Input signals
			fprintf(console, " INPUTS:  ");
			print_binary(console, inputs, 8);
			fprintf(console, "\n");
			for (int i = 0; i < 8; i++) {
				if (inputs & (1 << i))
					fprintf(console, "   %s", input_names[i]);
			}
			fprintf(console, "\n\n");

			// Output signals
			fprintf(console, " OUTPUTS: ");
			print_binary(console, outputs, 8);
			fprintf(console, "\n");
			for (int i = 0; i < 8; i++) {
				draw_signal_bar(console, output_names[i],
				               (outputs >> i) & 1, 6);
			}

			fprintf(console, "\n STATE: 0x%08lX %08lX\n",
			        (unsigned long)state_hi, (unsigned long)state_lo);

			fprintf(console, "\n SEL+START to exit");
		}

		// Sync to vblank
		uint32_t vbl = GFX_REG(GFX_VBLCTR_REG);
		while (GFX_REG(GFX_VBLCTR_REG) == vbl)
			;
		frame++;
	}

	wait_for_button_release();
}
