#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * Lab 2: Buttons and LEDs
 * =======================
 * Polling hardware, bitwise operations, and real-time I/O.
 *
 * The badge has 8 buttons and 9 LEDs, all accessed through
 * memory-mapped registers in the MISC peripheral block.
 *
 * EXERCISES:
 *
 * Exercise 1: Make the LEDs mirror the buttons - when you press
 *             UP, LED 0 lights up, DOWN lights LED 1, etc.
 *             (This is already implemented below as an example.)
 *
 * Exercise 2: Implement a binary counter on the LEDs. Each time
 *             you press button A, increment a counter and display
 *             it in binary on the LEDs.
 *
 * Exercise 3: Create a "knight rider" LED pattern - a single lit
 *             LED bouncing back and forth. Use a delay loop or
 *             the vblank counter (GFX_REG(GFX_VBLCTR_REG)) for timing.
 *
 * Exercise 4: Implement a simple combination lock:
 *             - Define a secret sequence (e.g., UP UP DOWN DOWN LEFT RIGHT)
 *             - Track button presses and check against the sequence
 *             - Flash all LEDs when the correct combo is entered
 *
 * Exercise 5 (challenge): Use the ADC (analog-to-digital converter)
 *             to read an analog voltage. Display the value on the LEDs
 *             as a bar graph. See MISC_ADC_CTL_REG and MISC_ADC_VAL_REG.
 *
 * CONCEPTS:
 *   - Polling vs interrupts
 *   - Bitwise AND, OR, XOR, shift operations
 *   - Debouncing (why wait_for_button_release exists)
 *   - Binary representation
 */

static FILE *console;

static const char *btn_names[] = {
	"UP", "DOWN", "LEFT", "RIGHT", "A", "B", "SELECT", "START"
};

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;
	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	fprintf(console, "\033C");
	fprintf(console, "Lab 2: Buttons & LEDs\n\n");
	fprintf(console, "Press buttons to light LEDs\n");
	fprintf(console, "SELECT+START to exit\n\n");

	// === Exercise 2 variables ===
	// uint8_t counter = 0;
	// uint8_t prev_a = 0;

	uint32_t prev_btn = 0;

	while (1) {
		// Read button state - each bit corresponds to a button
		uint32_t btn = MISC_REG(MISC_BTN_REG);

		// === Exercise 1: Mirror buttons to LEDs ===
		// The lower 8 bits of the LED register control LEDs 0-7.
		// Simply copying the button state makes LEDs mirror buttons.
		MISC_REG(MISC_LED_REG) = btn & 0xFF;

		// Print which buttons changed (edge detection)
		uint32_t pressed = btn & ~prev_btn;  // Newly pressed
		if (pressed) {
			for (int i = 0; i < 8; i++) {
				if (pressed & (1 << i)) {
					fprintf(console, "Pressed: %s\n", btn_names[i]);
				}
			}
		}
		prev_btn = btn;

		// === Exercise 2: Binary counter ===
		// Detect the rising edge of button A:
		// uint8_t cur_a = (btn & BUTTON_A) ? 1 : 0;
		// if (cur_a && !prev_a) {
		//     counter++;
		//     MISC_REG(MISC_LED_REG) = counter;
		//     fprintf(console, "Counter: %d (0b", counter);
		//     for (int i = 7; i >= 0; i--)
		//         fprintf(console, "%d", (counter >> i) & 1);
		//     fprintf(console, ")\n");
		// }
		// prev_a = cur_a;

		// Exit on SELECT+START
		if ((btn & BUTTON_SELECT) && (btn & BUTTON_START))
			break;
	}

	MISC_REG(MISC_LED_REG) = 0;
	wait_for_button_release();
}
