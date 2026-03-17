#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "gfx_load.h"
#include "cache.h"
#include "badgetime.h"

/*
 * Lab 7: Embedded Control Systems
 * ================================
 * A Proportional-Integral-Derivative (PID) controller reads a sensor
 * (the ADC), compares the measurement to a desired setpoint, and
 * drives an actuator (PWM output) to minimize the error.
 *
 * Because the RISC-V core has no hardware floating-point unit we use
 * fixed-point Q16.16 arithmetic: the upper 16 bits are the integer
 * part and the lower 16 bits are the fractional part.  Multiplying
 * two Q16.16 values requires a 64-bit intermediate to avoid overflow.
 *
 * EXERCISES:
 *
 * Exercise 1: Implement a proportional-only controller.  Read the
 *             ADC value, compute error = setpoint - measured, and
 *             set the PWM output to Kp * error.
 *
 * Exercise 2: Add an integral term that accumulates error over time
 *             to eliminate steady-state offset.
 *
 * Exercise 3: Add a derivative term that reacts to the rate of
 *             change of the error to reduce overshoot and ringing.
 *
 * Exercise 4: Use the D-pad buttons to adjust Kp, Ki, and Kd in
 *             real time.  Display current gains and output on the
 *             console and LEDs.
 *
 * Exercise 5 (challenge): Implement anti-windup for the integral
 *             term and a low-pass filter on the derivative term.
 *
 * CONCEPTS:
 *   - Feedback control loops (measure -> compute -> actuate)
 *   - PID algorithm: output = Kp*e + Ki*integral(e) + Kd*d(e)/dt
 *   - Fixed-point Q16.16 arithmetic (no FPU on this core)
 *   - Integral windup and anti-windup clamping
 *   - Sampling rate, timing, and loop determinism
 */

/* ---- Fixed-point Q16.16 helpers ---- */
typedef int32_t  fixed_t;           /* Q16.16 */
#define FP_SHIFT   16
#define FP_ONE     (1 << FP_SHIFT)  /* 1.0 in Q16.16 = 0x00010000 */

/* Convert integer to fixed-point */
#define INT_TO_FP(x)   ((fixed_t)(x) << FP_SHIFT)

/* Convert fixed-point to integer (truncates) */
#define FP_TO_INT(x)   ((x) >> FP_SHIFT)

/* Multiply two Q16.16 values using 64-bit intermediate */
static inline fixed_t fp_mul(fixed_t a, fixed_t b) {
	return (fixed_t)(((int64_t)a * (int64_t)b) >> FP_SHIFT);
}

/* ---- Hardware register shortcuts ---- */
/* PWM output register (directly drives a PWM duty-cycle counter) */
#define PWM_BASE       0x50030000
#define PWM_DUTY       (*(volatile uint32_t *)(PWM_BASE + 0x00))
#define PWM_PERIOD     (*(volatile uint32_t *)(PWM_BASE + 0x04))

/* Maximum output value for the PWM (10-bit) */
#define PWM_MAX  1023

static FILE *console;

/* Clamp a value to [lo, hi] */
static inline int32_t clamp(int32_t v, int32_t lo, int32_t hi) {
	if (v < lo) return lo;
	if (v > hi) return hi;
	return v;
}

/* Wait for next vertical blank (used as a fixed ~16.7 ms time base) */
static void wait_vblank(void) {
	uint32_t vbl = GFX_REG(GFX_VBLCTR_REG);
	while (GFX_REG(GFX_VBLCTR_REG) == vbl)
		;
}

void main(int argc, char **argv) {
	GFX_REG(GFX_BGNDCOL_REG) = 0x101830;
	GFX_REG(GFX_LAYEREN_REG) = GFX_LAYEREN_TILEA;
	console = fopen("/dev/console", "w");
	setvbuf(console, NULL, _IONBF, 0);

	fprintf(console, "\033C");
	fprintf(console, "Lab 7: PID Control\n\n");

	/* ---- PID state variables (all Q16.16) ---- */
	fixed_t setpoint   = INT_TO_FP(512);  /* Target: mid-range of 10-bit ADC */
	fixed_t integral   = 0;
	fixed_t prev_error = 0;

	/* ---- Default gains (Q16.16) ---- */
	fixed_t Kp = FP_ONE / 4;       /* 0.25  */
	fixed_t Ki = FP_ONE / 64;      /* ~0.016 */
	fixed_t Kd = FP_ONE / 8;       /* 0.125  */

	/* Anti-windup limits for integral term */
	fixed_t integral_max = INT_TO_FP(PWM_MAX);
	fixed_t integral_min = INT_TO_FP(-PWM_MAX);

	/* Enable the ADC */
	MISC_REG(MISC_ADC_CTL_REG) = MISC_ADC_CTL_ENA | MISC_ADC_CTL_DIV(4);

	uint32_t prev_btn = 0;
	int frame = 0;

	fprintf(console, "UP/DN: setpoint  L/R: select gain\n");
	fprintf(console, "A: increase gain B: decrease gain\n");
	fprintf(console, "SELECT+START: exit\n\n");

	int gain_sel = 0;   /* 0=Kp, 1=Ki, 2=Kd */

	while (1) {
		uint32_t btn = MISC_REG(MISC_BTN_REG);
		uint32_t pressed = btn & ~prev_btn;

		/* Exit on SELECT+START */
		if ((btn & BUTTON_SELECT) && (btn & BUTTON_START))
			break;

		/* === Exercise 4: Adjust gains with buttons === */
		/* TODO: Use LEFT/RIGHT to cycle gain_sel, A/B to adjust. */
		// if (pressed & BUTTON_LEFT)  gain_sel = (gain_sel + 2) % 3;
		// if (pressed & BUTTON_RIGHT) gain_sel = (gain_sel + 1) % 3;
		//
		// fixed_t step = FP_ONE / 64;
		// fixed_t *sel = (gain_sel == 0) ? &Kp :
		//                (gain_sel == 1) ? &Ki : &Kd;
		// if (pressed & BUTTON_A) *sel += step;
		// if (pressed & BUTTON_B) *sel -= step;
		// if (*sel < 0) *sel = 0;

		/* Adjust setpoint with UP/DOWN */
		if (btn & BUTTON_UP)   setpoint += INT_TO_FP(2);
		if (btn & BUTTON_DOWN) setpoint -= INT_TO_FP(2);
		setpoint = clamp(setpoint, 0, INT_TO_FP(1023));

		prev_btn = btn;

		/* ---- Read the ADC ---- */
		uint32_t adc_raw = MISC_REG(MISC_ADC_VAL_REG) & 0x3FF; /* 10-bit */
		fixed_t measured = INT_TO_FP((int)adc_raw);

		/* ---- Compute PID ---- */
		fixed_t error = setpoint - measured;

		/* === Exercise 1: Proportional term ===
		 * TODO: Compute p_term = Kp * error
		 */
		fixed_t p_term = 0;
		// p_term = fp_mul(Kp, error);

		/* === Exercise 2: Integral term ===
		 * TODO: Accumulate error into integral; compute i_term = Ki * integral
		 */
		fixed_t i_term = 0;
		// integral += error;
		// integral = clamp(integral, integral_min, integral_max);
		// i_term = fp_mul(Ki, integral);

		/* === Exercise 3: Derivative term ===
		 * TODO: Compute d_term = Kd * (error - prev_error)
		 */
		fixed_t d_term = 0;
		// d_term = fp_mul(Kd, error - prev_error);
		prev_error = error;

		/* === Exercise 5 (challenge): Anti-windup ===
		 * If the output is saturated, stop accumulating integral.
		 */
		// fixed_t output_raw = p_term + i_term + d_term;
		// int saturated = (FP_TO_INT(output_raw) > PWM_MAX) ||
		//                 (FP_TO_INT(output_raw) < 0);
		// if (saturated) {
		//     /* Back-calculate: undo last integral accumulation */
		//     integral -= error;
		//     i_term = fp_mul(Ki, integral);
		// }
		//
		// /* Derivative low-pass filter (exponential moving average) */
		// static fixed_t d_filtered = 0;
		// fixed_t alpha = FP_ONE / 4;  /* filter coefficient 0.25 */
		// d_filtered = fp_mul(alpha, d_term) +
		//              fp_mul(FP_ONE - alpha, d_filtered);
		// d_term = d_filtered;

		/* ---- Sum and clamp output ---- */
		fixed_t output = p_term + i_term + d_term;
		int32_t pwm_val = clamp(FP_TO_INT(output), 0, PWM_MAX);

		/* Write to PWM hardware */
		PWM_DUTY = (uint32_t)pwm_val;

		/* Show on LEDs (top 8 bits of PWM as a bar) */
		uint8_t led_bar = (uint8_t)(pwm_val >> 2);
		MISC_REG(MISC_LED_REG) = led_bar;

		/* ---- Display status every 15 frames (~4 Hz) ---- */
		if ((frame++ & 0x0F) == 0) {
			fprintf(console, "\033""0Y");  /* cursor to row 7 */
			fprintf(console, "SP=%4ld  ADC=%4lu  PWM=%4ld\n",
			        (long)FP_TO_INT(setpoint),
			        (unsigned long)adc_raw,
			        (long)pwm_val);
			fprintf(console, "Kp=%ld.%03ld  Ki=%ld.%03ld  Kd=%ld.%03ld\n",
			        (long)FP_TO_INT(Kp),
			        (long)((Kp & 0xFFFF) * 1000L >> 16),
			        (long)FP_TO_INT(Ki),
			        (long)((Ki & 0xFFFF) * 1000L >> 16),
			        (long)FP_TO_INT(Kd),
			        (long)((Kd & 0xFFFF) * 1000L >> 16));
			const char *names[] = { "Kp", "Ki", "Kd" };
			fprintf(console, "Editing: %s\n", names[gain_sel]);
		}

		wait_vblank();   /* Fixed ~60 Hz sample rate */
	}

	MISC_REG(MISC_LED_REG) = 0;
	PWM_DUTY = 0;
	wait_for_button_release();
}
