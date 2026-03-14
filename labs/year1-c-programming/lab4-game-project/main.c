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
 * Lab 4: Game Project
 * ===================
 * Capstone project for Year 1. Build a simple game using everything
 * you've learned: C programming, hardware I/O, and the graphics system.
 *
 * This skeleton provides a basic game loop with:
 * - 60fps frame timing via vblank sync
 * - Input handling with edge detection
 * - A player entity with position and movement
 * - Collision detection helper
 * - Score tracking and display
 *
 * PROJECT IDEAS:
 *   - Pong: paddle + bouncing ball
 *   - Snake: growing trail with food pickups
 *   - Breakout: paddle, ball, and bricks
 *   - Dodge: avoid falling objects
 *   - Your own idea!
 *
 * REQUIREMENTS:
 *   1. Player-controlled entity with D-pad input
 *   2. At least one other moving entity (enemy, ball, etc.)
 *   3. Collision detection between entities
 *   4. Score display
 *   5. Game over condition with restart option
 *
 * CONCEPTS:
 *   - Game loop architecture (input -> update -> render)
 *   - Fixed timestep and frame synchronization
 *   - State machines (menu, playing, game over)
 *   - Struct-based entity management
 */

#define FB_WIDTH 480
#define FB_HEIGHT 320

static uint8_t *fbmem;

// --- Palette setup ---
enum colors {
	COL_BLACK = 0,
	COL_WHITE = 1,
	COL_RED = 2,
	COL_GREEN = 3,
	COL_BLUE = 4,
	COL_YELLOW = 5,
	COL_GRAY = 6,
};

static void setup_palette(void) {
	GFXPAL[COL_BLACK]  = 0xFF000000;
	GFXPAL[COL_WHITE]  = 0xFFFFFFFF;
	GFXPAL[COL_RED]    = 0xFFFF2020;
	GFXPAL[COL_GREEN]  = 0xFF20FF20;
	GFXPAL[COL_BLUE]   = 0xFF2060FF;
	GFXPAL[COL_YELLOW] = 0xFFFFFF20;
	GFXPAL[COL_GRAY]   = 0xFF808080;
}

// --- Drawing primitives ---
static void draw_pixel(int x, int y, uint8_t c) {
	if (x >= 0 && x < FB_WIDTH && y >= 0 && y < FB_HEIGHT)
		fbmem[y * FB_WIDTH + x] = c;
}

static void draw_rect(int x0, int y0, int w, int h, uint8_t c) {
	for (int y = y0; y < y0 + h; y++)
		for (int x = x0; x < x0 + w; x++)
			draw_pixel(x, y, c);
}

static void clear_fb(void) {
	memset(fbmem, COL_BLACK, FB_WIDTH * FB_HEIGHT);
}

static void fb_flush(void) {
	cache_flush(fbmem, fbmem + FB_WIDTH * FB_HEIGHT);
}

// --- Simple entity ---
typedef struct {
	int x, y;
	int w, h;
	int dx, dy;
	uint8_t color;
	int active;
} entity_t;

// Axis-aligned bounding box collision
static int collides(entity_t *a, entity_t *b) {
	return a->active && b->active &&
	       a->x < b->x + b->w && a->x + a->w > b->x &&
	       a->y < b->y + b->h && a->y + a->h > b->y;
}

// --- Game state ---
typedef enum { STATE_PLAYING, STATE_GAMEOVER } game_state_t;

static entity_t player;
static entity_t ball;
static int score;
static game_state_t state;

static void game_init(void) {
	player = (entity_t){ .x = 220, .y = 280, .w = 40, .h = 8,
	                     .color = COL_WHITE, .active = 1 };
	ball = (entity_t){ .x = 240, .y = 160, .w = 6, .h = 6,
	                   .dx = 2, .dy = -2,
	                   .color = COL_YELLOW, .active = 1 };
	score = 0;
	state = STATE_PLAYING;
}

static void game_update(uint32_t btn) {
	if (state == STATE_GAMEOVER) {
		if (btn & BUTTON_A)
			game_init();
		return;
	}

	// Move player with D-pad
	int speed = 4;
	if (btn & BUTTON_LEFT)  player.x -= speed;
	if (btn & BUTTON_RIGHT) player.x += speed;

	// Clamp player to screen
	if (player.x < 0) player.x = 0;
	if (player.x > FB_WIDTH - player.w) player.x = FB_WIDTH - player.w;

	// Move ball
	ball.x += ball.dx;
	ball.y += ball.dy;

	// Ball bounces off walls
	if (ball.x <= 0 || ball.x >= FB_WIDTH - ball.w)
		ball.dx = -ball.dx;
	if (ball.y <= 0)
		ball.dy = -ball.dy;

	// Ball bounces off player paddle
	if (collides(&ball, &player)) {
		ball.dy = -ball.dy;
		ball.y = player.y - ball.h;
		score++;
	}

	// Ball fell off bottom = game over
	if (ball.y > FB_HEIGHT) {
		state = STATE_GAMEOVER;
	}
}

static void game_render(void) {
	clear_fb();

	// Draw player
	draw_rect(player.x, player.y, player.w, player.h, player.color);

	// Draw ball
	draw_rect(ball.x, ball.y, ball.w, ball.h, ball.color);

	// Draw score as a bar
	draw_rect(10, 5, score * 8, 4, COL_GREEN);

	if (state == STATE_GAMEOVER) {
		// Simple "game over" indicator: red bar across screen
		draw_rect(0, 155, FB_WIDTH, 10, COL_RED);
	}

	fb_flush();
}

void main(int argc, char **argv) {
	fbmem = calloc(FB_WIDTH, FB_HEIGHT);
	GFX_REG(GFX_FBADDR_REG) = (uint32_t)fbmem;
	GFX_REG(GFX_FBPITCH_REG) = (0 << GFX_FBPITCH_PAL_OFF) | (FB_WIDTH << GFX_FBPITCH_PITCH_OFF);
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_FB | GFX_LAYEREN_FB_8BIT;
	GFX_REG(GFX_BGNDCOL_REG) = 0x000000;

	setup_palette();
	game_init();

	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);

		if ((btn & BUTTON_SELECT) && (btn & BUTTON_START))
			break;

		game_update(btn);
		game_render();

		// Sync to 60fps
		uint32_t vbl = GFX_REG(GFX_VBLCTR_REG);
		while (GFX_REG(GFX_VBLCTR_REG) == vbl)
			;
	}

	free(fbmem);
	wait_for_button_release();
}
