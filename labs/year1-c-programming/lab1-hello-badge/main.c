#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * Lab 1: Hello Badge
 * ==================
 * Your first program on real hardware! This lab introduces you to:
 * - The C compilation toolchain (cross-compiling for RISC-V)
 * - Memory-mapped I/O (how software talks to hardware)
 * - The badge's text console output
 *
 * EXERCISES:
 *
 * Exercise 1: Make it print your name instead of "World"
 *
 * Exercise 2: Print the SoC version by reading MISC_REG(MISC_SOC_VER).
 *             What value do you get? What does bit 16 mean?
 *
 * Exercise 3: Print which CPU number you're running on using
 *             MISC_REG(MISC_CPU_NO). The badge has 2 RISC-V cores!
 *
 * Exercise 4: Use a loop to print the numbers 1 to 20. Observe how
 *             printf works on embedded systems (it goes to UART).
 *
 * Exercise 5 (challenge): Read the MISC_RNG_REG register to get a
 *             random number. Print 10 random numbers.
 *
 * BUILDING:
 *   make          - Build the app
 *   make flash    - Upload to badge via USB
 *
 * CONCEPTS:
 *   - Cross compilation: your PC is x86, the badge is RISC-V
 *   - Memory-mapped I/O: hardware registers appear as memory addresses
 *   - volatile: tells the compiler not to optimize away hardware reads
 */

void main(int argc, char **argv) {
	// Set up the display - don't worry about this yet, we'll cover
	// graphics in Lab 3. For now, just know it gives us a text console.
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;

	FILE *console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	// This prints to the UART (serial port) - you can see it on your
	// computer with a terminal program at 115200 baud.
	printf("Lab 1: Hello Badge!\n");

	// This prints to the LCD screen via the tile-based text console.
	fprintf(console, "\033C");  // Clear screen (escape sequence)
	fprintf(console, "\0332X");  // Set X position to 2
	fprintf(console, "\0332Y");  // Set Y position to 2

	fprintf(console, "Hello, World!\n\n");

	// === Exercise 2: Read hardware registers ===
	// The MISC peripheral lives at address 0x20000000.
	// Each "register" is a 32-bit value at a specific offset.
	// MISC_REG(offset) reads or writes that register.
	//
	// Try uncommenting this:
	// uint32_t soc_version = MISC_REG(MISC_SOC_VER);
	// fprintf(console, "SoC version: 0x%08lX\n", (unsigned long)soc_version);

	// === Exercise 3: Which CPU am I? ===
	// uint32_t cpu_id = MISC_REG(MISC_CPU_NO);
	// fprintf(console, "Running on CPU %lu\n", (unsigned long)cpu_id);

	// === Exercise 5: Random numbers ===
	// The badge has a hardware random number generator!
	// for (int i = 0; i < 10; i++) {
	//     uint32_t rng = MISC_REG(MISC_RNG_REG);
	//     fprintf(console, "Random: 0x%08lX\n", (unsigned long)rng);
	// }

	fprintf(console, "\n\nPress START to exit");

	// Wait for START button
	while (!(MISC_REG(MISC_BTN_REG) & BUTTON_START))
		;
	wait_for_button_release();
}
