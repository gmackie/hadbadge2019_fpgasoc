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
 * Lab 8: Data Acquisition and Signal Processing
 * ===============================================
 * In this lab you will sample the ADC continuously into a ring buffer,
 * compute statistics, and implement digital filters in C.  This is the
 * foundation of any real-time data-acquisition system.
 *
 * Sampling theory (Nyquist): to faithfully represent a signal of
 * frequency F you must sample at >= 2*F.  Sampling too slowly causes
 * aliasing -- high-frequency content folds into lower frequencies.
 *
 * EXERCISES:
 *
 * Exercise 1: Implement continuous ADC sampling into a 256-sample ring
 *             buffer at a fixed rate (one sample per vblank = 60 Hz).
 *
 * Exercise 2: Compute min, max, and average of the buffer contents.
 *             Display the statistics on the console.
 *
 * Exercise 3: Implement a moving-average filter with a configurable
 *             window size.  Compare raw and filtered values.
 *
 * Exercise 4: Implement a software FIR (Finite Impulse Response) filter
 *             using a coefficient array and a convolution loop.
 *
 * Exercise 5 (challenge): Display a real-time waveform on the LCD by
 *             writing to the framebuffer, scrolling as new samples arrive.
 *
 * CONCEPTS:
 *   - Sampling theory and the Nyquist frequency
 *   - Ring (circular) buffers for streaming data
 *   - Digital filtering: moving average and FIR
 *   - Fixed-point convolution (Q16.16)
 *   - Real-time constraints and aliasing
 */

/* ---- Ring buffer ---- */
#define BUF_SIZE   256                    /* must be a power of two */
#define BUF_MASK   (BUF_SIZE - 1)
static uint16_t sample_buf[BUF_SIZE];    /* raw ADC samples        */
static uint16_t filtered_buf[BUF_SIZE];  /* output of the filter   */
static int buf_head = 0;                 /* next write position     */

/* ---- Fixed-point helpers (Q16.16) ---- */
typedef int32_t fixed_t;
#define FP_SHIFT  16
#define FP_ONE    (1 << FP_SHIFT)
static inline fixed_t fp_mul(fixed_t a, fixed_t b) {
	return (fixed_t)(((int64_t)a * (int64_t)b) >> FP_SHIFT);
}

/* ---- FIR filter coefficients (low-pass, 7-tap) ----
 * Stored as Q16.16 values.  These approximate a simple low-pass
 * filter with a cutoff around Fs/8.  Coefficients must sum to 1.0.
 */
#define FIR_TAPS  7
static const fixed_t fir_coeff[FIR_TAPS] = {
	4681,    /* ~0.071 * 65536 */
	9830,    /* ~0.150         */
	14746,   /* ~0.225         */
	16384,   /* ~0.250  (center tap) */
	14746,   /* ~0.225         */
	9830,    /* ~0.150         */
	4681,    /* ~0.071         */
};

/* ---- Framebuffer for waveform display ---- */
#define FB_WIDTH   480
#define FB_HEIGHT  320
#define WAVE_Y0    60          /* top of waveform area   */
#define WAVE_H     200         /* height of waveform area */
static uint8_t *fbmem;

static FILE *console;

/* Wait for the next vertical blank (one sample period at 60 Hz) */
static void wait_vblank(void) {
	uint32_t vbl = GFX_REG(GFX_VBLCTR_REG);
	while (GFX_REG(GFX_VBLCTR_REG) == vbl)
		;
}

/* Draw a single pixel (bounds-checked) */
static void draw_pixel(int x, int y, uint8_t color) {
	if (x >= 0 && x < FB_WIDTH && y >= 0 && y < FB_HEIGHT)
		fbmem[y * FB_WIDTH + x] = color;
}

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;
	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	fprintf(console, "\033C");
	fprintf(console, "Lab 8: Data Acquisition\n\n");

	/* Enable the ADC */
	MISC_REG(MISC_ADC_CTL_REG) = MISC_ADC_CTL_ENA | MISC_ADC_CTL_DIV(4);

	/* Allocate framebuffer for Exercise 5 */
	fbmem = calloc(FB_WIDTH, FB_HEIGHT);

	/* Set up a simple palette for the waveform */
	GFXPAL[0] = 0xFF000000;   /* black  - background */
	GFXPAL[1] = 0xFF00FF00;   /* green  - raw signal */
	GFXPAL[2] = 0xFFFF4040;   /* red    - filtered   */
	GFXPAL[3] = 0xFF404040;   /* gray   - grid lines */
	GFXPAL[4] = 0xFFFFFF00;   /* yellow - text/info  */

	memset(sample_buf, 0, sizeof(sample_buf));
	memset(filtered_buf, 0, sizeof(filtered_buf));

	/* Moving-average window size (Exercise 3) */
	int ma_window = 8;

	uint32_t prev_btn = 0;
	int frame = 0;

	fprintf(console, "UP/DN: window size  START: exit\n\n");

	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);
		uint32_t pressed = btn & ~prev_btn;
		prev_btn = btn;

		if ((btn & BUTTON_SELECT) && (btn & BUTTON_START))
			break;

		/* Adjust moving-average window with UP/DOWN */
		if (pressed & BUTTON_UP)   { ma_window <<= 1; if (ma_window > 64) ma_window = 64; }
		if (pressed & BUTTON_DOWN) { ma_window >>= 1; if (ma_window < 1)  ma_window = 1;  }

		/* === Exercise 1: Sample ADC into ring buffer ===
		 * Read the 10-bit ADC value and store it at buf_head.
		 * Advance buf_head with wrap-around using BUF_MASK.
		 * TODO: Fill in the sampling code.
		 */
		// uint16_t raw = MISC_REG(MISC_ADC_VAL_REG) & 0x3FF;
		// sample_buf[buf_head] = raw;

		/* === Exercise 3: Moving-average filter ===
		 * Average the last ma_window samples to smooth the signal.
		 * TODO: Compute the moving average and store in filtered_buf.
		 */
		// uint32_t sum = 0;
		// for (int i = 0; i < ma_window; i++) {
		//     int idx = (buf_head - i) & BUF_MASK;
		//     sum += sample_buf[idx];
		// }
		// filtered_buf[buf_head] = (uint16_t)(sum / ma_window);

		/* === Exercise 4: FIR filter ===
		 * Convolve the last FIR_TAPS samples with fir_coeff[].
		 * Use fixed-point multiplication for each tap.
		 * TODO: Replace or augment the moving average with an FIR.
		 */
		// fixed_t acc = 0;
		// for (int t = 0; t < FIR_TAPS; t++) {
		//     int idx = (buf_head - t) & BUF_MASK;
		//     fixed_t sample_fp = (fixed_t)sample_buf[idx] << FP_SHIFT;
		//     acc += fp_mul(fir_coeff[t], sample_fp);
		// }
		// filtered_buf[buf_head] = (uint16_t)(acc >> FP_SHIFT);

		/* Advance ring buffer head */
		// buf_head = (buf_head + 1) & BUF_MASK;

		/* === Exercise 2: Compute statistics ===
		 * Walk the entire buffer and find min, max, and average.
		 * TODO: Calculate and display the statistics.
		 */
		// uint32_t s_min = 0xFFFF, s_max = 0, s_sum = 0;
		// for (int i = 0; i < BUF_SIZE; i++) {
		//     uint16_t s = sample_buf[i];
		//     if (s < s_min) s_min = s;
		//     if (s > s_max) s_max = s;
		//     s_sum += s;
		// }
		// uint32_t s_avg = s_sum / BUF_SIZE;
		//
		// if ((frame & 0x0F) == 0) {  /* update display ~4 Hz */
		//     fprintf(console, "\033""5Y");
		//     fprintf(console, "Min=%4lu  Max=%4lu  Avg=%4lu\n",
		//             (unsigned long)s_min, (unsigned long)s_max,
		//             (unsigned long)s_avg);
		//     fprintf(console, "Window=%d  Head=%d\n", ma_window, buf_head);
		// }

		/* === Exercise 5 (challenge): Real-time waveform display ===
		 * Draw the last FB_WIDTH samples as a scrolling waveform.
		 * Scale 10-bit ADC (0-1023) to WAVE_H pixels.
		 * Green = raw, Red = filtered.
		 * TODO: Enable the framebuffer and draw the waveform.
		 */
		// GFX_REG(GFX_FBADDR_REG) = (uint32_t)fbmem;
		// GFX_REG(GFX_FBPITCH_REG) = (0 << GFX_FBPITCH_PAL_OFF)
		//                           | (FB_WIDTH << GFX_FBPITCH_PITCH_OFF);
		// GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_FB
		//                           | GFX_LAYEREN_FB_8BIT
		//                           | GFX_LAYEREN_TILEA;
		//
		// /* Clear the waveform area */
		// for (int y = WAVE_Y0; y < WAVE_Y0 + WAVE_H; y++)
		//     memset(fbmem + y * FB_WIDTH, 0, FB_WIDTH);
		//
		// /* Draw horizontal grid lines every 64 ADC counts */
		// for (int g = 0; g < 1024; g += 64) {
		//     int gy = WAVE_Y0 + WAVE_H - 1 - (g * WAVE_H / 1023);
		//     for (int x = 0; x < FB_WIDTH; x += 4)
		//         draw_pixel(x, gy, 3);
		// }
		//
		// /* Draw raw (green) and filtered (red) waveforms */
		// for (int x = 0; x < FB_WIDTH && x < BUF_SIZE; x++) {
		//     int idx = (buf_head - FB_WIDTH + x) & BUF_MASK;
		//     int yr = WAVE_Y0 + WAVE_H - 1
		//            - (sample_buf[idx] * WAVE_H / 1023);
		//     int yf = WAVE_Y0 + WAVE_H - 1
		//            - (filtered_buf[idx] * WAVE_H / 1023);
		//     draw_pixel(x, yr, 1);  /* green = raw      */
		//     draw_pixel(x, yf, 2);  /* red   = filtered  */
		// }
		//
		// cache_flush(fbmem, fbmem + FB_WIDTH * FB_HEIGHT);

		frame++;
		wait_vblank();   /* One sample per frame = 60 Hz sample rate */
	}

	MISC_REG(MISC_ADC_CTL_REG) = 0;
	if (fbmem) free(fbmem);
	wait_for_button_release();
}
