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
 * Lab 3: Graphics Introduction
 * ============================
 * Understanding the badge's graphics hardware:
 * - Framebuffer: a block of memory that maps directly to pixels
 * - Palette: a lookup table that maps byte values to RGBA colors
 * - Tiles: reusable 16x16 pixel blocks for efficient text/map rendering
 *
 * The graphics system has 4 composited layers:
 *   1. Framebuffer (direct pixel bitmap, 4-bit or 8-bit indexed)
 *   2. Tile Layer A (64x64 grid of 16x16 tiles)
 *   3. Tile Layer B (same)
 *   4. Sprite Layer (256 movable sprites from tile memory)
 *
 * EXERCISES:
 *
 * Exercise 1: Modify draw_pixel to draw in different colors.
 *             Change the palette entries to your favorite colors.
 *
 * Exercise 2: Write a draw_line function using Bresenham's algorithm.
 *             Draw lines between the corners of the screen.
 *
 * Exercise 3: Draw a filled rectangle and a filled circle.
 *             For the circle, use: (x-cx)^2 + (y-cy)^2 <= r^2
 *
 * Exercise 4: Animate! Move a shape across the screen by redrawing
 *             each frame. Use GFX_REG(GFX_VBLCTR_REG) to sync to
 *             the display refresh (60 Hz).
 *
 * Exercise 5 (challenge): Implement a simple drawing program where
 *             the D-pad moves a cursor and button A draws pixels.
 *
 * CONCEPTS:
 *   - Framebuffers and pixel addressing
 *   - Color palettes (indexed color)
 *   - Cache coherency (CPU cache vs DMA)
 *   - Double buffering (why we need cache_flush)
 */

// Framebuffer dimensions
#define FB_WIDTH 480
#define FB_HEIGHT 320

static uint8_t *fbmem;
static FILE *console;

// Set a pixel in the framebuffer
static void draw_pixel(int x, int y, uint8_t color) {
	if (x >= 0 && x < FB_WIDTH && y >= 0 && y < FB_HEIGHT) {
		fbmem[y * FB_WIDTH + x] = color;
	}
}

// Fill a rectangle
static void draw_rect(int x0, int y0, int w, int h, uint8_t color) {
	for (int y = y0; y < y0 + h && y < FB_HEIGHT; y++) {
		for (int x = x0; x < x0 + w && x < FB_WIDTH; x++) {
			draw_pixel(x, y, color);
		}
	}
}

// Clear the framebuffer
static void clear_fb(uint8_t color) {
	memset(fbmem, color, FB_WIDTH * FB_HEIGHT);
}

// Flush framebuffer to PSRAM so the GFX hardware can read it
static void fb_flush(void) {
	cache_flush(fbmem, fbmem + FB_WIDTH * FB_HEIGHT);
}

// Wait for vertical blank (next frame)
static void wait_vblank(void) {
	uint32_t vbl = GFX_REG(GFX_VBLCTR_REG);
	while (GFX_REG(GFX_VBLCTR_REG) == vbl)
		;
}

void main(int argc, char **argv) {
	printf("Lab 3: Graphics\n");

	// Allocate framebuffer memory
	fbmem = calloc(FB_WIDTH, FB_HEIGHT);

	// Configure the GFX hardware to use our framebuffer
	GFX_REG(GFX_FBADDR_REG) = (uint32_t)fbmem;
	GFX_REG(GFX_FBPITCH_REG) = (0 << GFX_FBPITCH_PAL_OFF) | (FB_WIDTH << GFX_FBPITCH_PITCH_OFF);
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_FB | GFX_LAYEREN_FB_8BIT;
	GFX_REG(GFX_BGNDCOL_REG) = 0x000000;

	// Set up a simple palette (256 colors)
	// Format: 0xAARRGGBB (alpha, red, green, blue)
	GFXPAL[0] = 0xFF000000;  // 0 = black
	GFXPAL[1] = 0xFFFF0000;  // 1 = red
	GFXPAL[2] = 0xFF00FF00;  // 2 = green
	GFXPAL[3] = 0xFF0000FF;  // 3 = blue
	GFXPAL[4] = 0xFFFFFF00;  // 4 = yellow
	GFXPAL[5] = 0xFFFF00FF;  // 5 = magenta
	GFXPAL[6] = 0xFF00FFFF;  // 6 = cyan
	GFXPAL[7] = 0xFFFFFFFF;  // 7 = white

	// Generate a gradient palette for colors 8-255
	for (int i = 8; i < 256; i++) {
		uint8_t r = (i * 3) & 0xFF;
		uint8_t g = (i * 5 + 100) & 0xFF;
		uint8_t b = (i * 7 + 50) & 0xFF;
		GFXPAL[i] = 0xFF000000 | (r << 16) | (g << 8) | b;
	}

	// Draw some shapes
	clear_fb(0);

	// Colored rectangles
	draw_rect(20, 20, 100, 60, 1);   // Red rectangle
	draw_rect(140, 20, 100, 60, 2);  // Green rectangle
	draw_rect(260, 20, 100, 60, 3);  // Blue rectangle

	// A gradient pattern
	for (int y = 100; y < 200; y++) {
		for (int x = 20; x < 460; x++) {
			draw_pixel(x, y, (x + y) & 0xFF);
		}
	}

	// Crosshair in the center
	for (int i = -20; i <= 20; i++) {
		draw_pixel(240 + i, 260, 7);  // Horizontal
		draw_pixel(240, 260 + i, 7);  // Vertical
	}

	fb_flush();

	// === Exercise 4: Animation ===
	// Uncomment this block for a moving square demo:
	// int bx = 0, by = 220, dx = 2, dy = 1;
	// while (!(MISC_REG(MISC_BTN_REG) & BUTTON_START)) {
	//     draw_rect(bx, by, 16, 16, 0);  // Erase old position
	//     bx += dx; by += dy;
	//     if (bx <= 0 || bx >= FB_WIDTH-16) dx = -dx;
	//     if (by <= 200 || by >= FB_HEIGHT-16) dy = -dy;
	//     draw_rect(bx, by, 16, 16, 4);  // Draw new position
	//     fb_flush();
	//     wait_vblank();
	// }

	// Wait for exit
	while (!(MISC_REG(MISC_BTN_REG) & BUTTON_START))
		;
	wait_for_button_release();
	free(fbmem);
}
