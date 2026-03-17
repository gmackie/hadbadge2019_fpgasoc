/*
 * Lab 6: PWM Generator and Servo Control
 * =======================================
 * Pulse Width Modulation (PWM) is a fundamental technique in digital systems
 * used to control analog-like behaviour from a digital output. By rapidly
 * switching a signal on and off with a variable duty cycle, you can control
 * the average power delivered to a load. In this lab you will implement a
 * flexible PWM generator on the ECP5 FPGA badge (48 MHz system clock) that
 * supports LED dimming, H-bridge motor driving with dead-time insertion,
 * and RC hobby-servo positioning. The servo protocol requires 50 Hz pulses
 * whose width encodes the desired angle: 1 ms = 0 degrees, 1.5 ms = 90
 * degrees, 2 ms = 180 degrees.
 *
 * EXERCISES:
 * Exercise 1: Implement basic PWM generation. Use a free-running counter
 *             that counts from 0 to `period - 1`. Compare the counter
 *             against `duty` to produce the PWM output (high when counter
 *             < duty, low otherwise).
 * Exercise 2: Add dead-time generation for an H-bridge motor driver. From
 *             the base PWM signal, derive two complementary outputs
 *             (`pwm_out` and `pwm_n_out`) with a configurable gap
 *             (`dead_time`) inserted at each transition to prevent
 *             shoot-through.
 * Exercise 3: Implement RC servo control. Generate a 50 Hz base period
 *             (960 000 cycles at 48 MHz) and produce a pulse whose width
 *             varies from 1 ms (48 000 cycles) to 2 ms (96 000 cycles)
 *             based on a position register.
 * Exercise 4: Add smooth ramping using badge button inputs. Pressing
 *             buttons[0] increments the servo position, buttons[1]
 *             decrements it. Include acceleration for a natural feel.
 * Exercise 5 (Challenge): Implement multi-channel PWM with phase
 *             offsetting for RGB LED colour mixing. Three independent
 *             duty cycles with 120-degree phase offsets reduce peak
 *             current draw and supply ripple.
 *
 * CONCEPTS:
 *   - Pulse width modulation and duty cycle
 *   - Dead-time insertion for H-bridge safety
 *   - RC servo protocol (1-2 ms pulses at 50 Hz)
 *   - H-bridge motor control (complementary outputs)
 *   - LED dimming and gamma perception (perceived brightness is non-linear)
 *   - Phase-offset PWM for reduced supply ripple
 *
 * LED MAPPING:
 *   LED[0] = PWM output (directly observe duty cycle)
 *   LED[1] = H-bridge high-side output (pwm_out with dead time)
 *   LED[2] = H-bridge low-side output (pwm_n_out with dead time)
 *   LED[3] = Servo pulse (1-2 ms at 50 Hz)
 *   LED[4] = Ramp direction indicator
 *   LED[7:5] = Servo position upper bits (visual position gauge)
 */

`default_nettype none

module pwm_servo (
    input  wire        clk,         // 48 MHz system clock
    input  wire        rst,         // Synchronous reset, active high
    // PWM configuration
    input  wire [15:0] duty,        // Duty cycle threshold (0 = always off)
    input  wire [15:0] period,      // PWM period in clock cycles
    input  wire [7:0]  dead_time,   // Dead-time gap in clock cycles
    // Outputs
    output reg         pwm_out,     // PWM high-side output
    output reg         pwm_n_out,   // PWM low-side (complementary) output
    output reg         servo_out,   // RC servo pulse output
    // User interface
    input  wire [7:0]  buttons,     // Badge buttons (directly active-high)
    output reg  [7:0]  leds         // Badge LEDs
);

    // ================================================================
    // Internal Registers
    // ================================================================
    reg [15:0] pwm_counter;         // Free-running PWM counter
    reg        pwm_raw;             // Raw PWM before dead-time insertion
    reg        pwm_raw_prev;        // Previous cycle's pwm_raw (edge detect)

    // Dead-time state
    reg [7:0]  dt_counter;          // Dead-time countdown timer
    reg        dt_blanking;         // High during dead-time blanking interval
    reg        pwm_rising;          // Detected rising edge on pwm_raw
    reg        pwm_falling;         // Detected falling edge on pwm_raw

    // Servo
    reg [19:0] servo_counter;       // 20-bit counter for 50 Hz period
    reg [16:0] servo_pulse_width;   // Pulse width in clock cycles (48000..96000)
    reg [7:0]  servo_position;      // 0..255 maps to 1 ms..2 ms

    // Ramping
    reg [15:0] ramp_prescaler;      // Slows down button-driven position changes
    reg        ramp_direction;      // 0 = incrementing, 1 = decrementing

    // ================================================================
    // Servo Timing Parameters (48 MHz clock)
    // ================================================================
    localparam SERVO_PERIOD     = 20'd960000;  // 20 ms = 50 Hz
    localparam SERVO_MIN_WIDTH  = 17'd48000;   // 1.0 ms = 0 degrees
    localparam SERVO_MAX_WIDTH  = 17'd96000;   // 2.0 ms = 180 degrees
    localparam SERVO_RANGE      = 17'd48000;   // SERVO_MAX - SERVO_MIN
    localparam RAMP_RATE        = 16'd48000;   // ~1 ms between position steps

    // ================================================================
    // Exercise 1: Basic PWM Generator
    // ================================================================
    // TODO: Implement a free-running counter that resets at `period`.
    //       Compare the counter value against `duty` to produce the
    //       raw PWM signal.
    //
    // HINT:
    //   pwm_counter counts 0, 1, 2, ... period-1, 0, 1, ...
    //   pwm_raw = (pwm_counter < duty) ? 1 : 0;
    //
    // This creates a signal that is HIGH for `duty` clock cycles out of
    // every `period` cycles, giving a duty cycle of duty/period * 100%.

    always @(posedge clk) begin
        if (rst) begin
            pwm_counter <= 16'd0;
            pwm_raw     <= 1'b0;
        end else begin
            // TODO: Increment pwm_counter, wrapping at period
            if (pwm_counter >= period - 1'b1)
                pwm_counter <= 16'd0;
            else
                pwm_counter <= pwm_counter + 1'b1;  // <-- remove if starting fresh

            // TODO: Generate pwm_raw by comparing counter to duty
            pwm_raw <= 1'b0;  // <-- replace with (pwm_counter < duty)
        end
    end

    // ================================================================
    // Exercise 2: Dead-Time Generator for H-Bridge
    // ================================================================
    // TODO: From the raw PWM signal, generate two complementary outputs
    //       `pwm_out` and `pwm_n_out`. Insert a dead-time gap of
    //       `dead_time` clock cycles at every transition to prevent
    //       shoot-through (both transistors on simultaneously).
    //
    // APPROACH:
    //   1. Detect rising and falling edges of pwm_raw.
    //   2. On any edge, start a dead-time countdown. During the countdown,
    //      BOTH outputs are LOW (blanking interval).
    //   3. After the countdown, drive the appropriate output high.
    //
    // HINT: Edge detection: pwm_rising  = pwm_raw & ~pwm_raw_prev;
    //                       pwm_falling = ~pwm_raw & pwm_raw_prev;
    //
    // Without dead time, if both high-side and low-side FETs turn on
    // at the same instant, a short circuit occurs -- this can damage
    // the H-bridge hardware.

    always @(posedge clk) begin
        if (rst) begin
            pwm_raw_prev <= 1'b0;
            pwm_rising   <= 1'b0;
            pwm_falling  <= 1'b0;
        end else begin
            pwm_raw_prev <= pwm_raw;
            pwm_rising   <=  pwm_raw & ~pwm_raw_prev;
            pwm_falling  <= ~pwm_raw &  pwm_raw_prev;
        end
    end

    always @(posedge clk) begin
        if (rst) begin
            dt_counter  <= 8'd0;
            dt_blanking <= 1'b0;
            pwm_out     <= 1'b0;
            pwm_n_out   <= 1'b0;
        end else begin
            // TODO: On a rising or falling edge of pwm_raw, load dt_counter
            //       with dead_time and assert dt_blanking.
            // TODO: While dt_blanking, decrement dt_counter. When it reaches
            //       zero, clear dt_blanking and update outputs.
            // TODO: When not blanking, drive pwm_out = pwm_raw and
            //       pwm_n_out = ~pwm_raw.

            // Placeholder: pass through without dead time
            pwm_out   <= pwm_raw;
            pwm_n_out <= ~pwm_raw;
        end
    end

    // ================================================================
    // Exercise 3: RC Servo Pulse Generator
    // ================================================================
    // TODO: Generate a 50 Hz signal using `servo_counter` that counts
    //       from 0 to SERVO_PERIOD-1. Produce a HIGH pulse at the start
    //       of each period whose width is determined by `servo_position`:
    //
    //       servo_pulse_width = SERVO_MIN_WIDTH
    //                         + (servo_position * SERVO_RANGE) / 256
    //
    //       servo_out = (servo_counter < servo_pulse_width) ? 1 : 0;
    //
    // HINT: To avoid a hardware divider, you can approximate the
    //       multiplication:  servo_position * 188  (since 48000/256 ~ 188)
    //       and add SERVO_MIN_WIDTH.
    //
    //       At 48 MHz:  48000 cycles = 1.0 ms (0 degrees)
    //                   72000 cycles = 1.5 ms (90 degrees)
    //                   96000 cycles = 2.0 ms (180 degrees)

    always @(posedge clk) begin
        if (rst) begin
            servo_counter     <= 20'd0;
            servo_pulse_width <= SERVO_MIN_WIDTH;
            servo_out         <= 1'b0;
        end else begin
            // TODO: Implement servo counter (counts 0 .. SERVO_PERIOD-1)
            if (servo_counter >= SERVO_PERIOD - 1'b1)
                servo_counter <= 20'd0;
            else
                servo_counter <= servo_counter + 1'b1;

            // TODO: Calculate servo_pulse_width from servo_position
            //       servo_pulse_width <= SERVO_MIN_WIDTH + (servo_position * 188);
            servo_pulse_width <= SERVO_MIN_WIDTH;  // <-- replace with calculation

            // TODO: Generate servo_out pulse
            servo_out <= 1'b0;  // <-- replace with (servo_counter < servo_pulse_width)
        end
    end

    // ================================================================
    // Exercise 4: Button-Driven Position Ramping
    // ================================================================
    // TODO: Read buttons[0] (increment) and buttons[1] (decrement) to
    //       adjust `servo_position`. Use `ramp_prescaler` to slow the
    //       rate so the servo moves smoothly rather than jumping.
    //
    // HINT: Every RAMP_RATE clock cycles, check the buttons:
    //       - buttons[0] pressed and position < 255: position++
    //       - buttons[1] pressed and position > 0:   position--
    //       Record direction in `ramp_direction` for LED display.

    always @(posedge clk) begin
        if (rst) begin
            servo_position  <= 8'd128;   // Start at centre (90 degrees)
            ramp_prescaler  <= 16'd0;
            ramp_direction  <= 1'b0;
        end else begin
            // TODO: Implement prescaler-gated button reading
            // TODO: Increment / decrement servo_position with saturation
            ramp_prescaler <= ramp_prescaler + 1'b1;
        end
    end

    // ================================================================
    // Exercise 5 (Challenge): Multi-Channel Phase-Offset PWM
    // ================================================================
    // TODO: Create three independent PWM channels for RGB LED mixing.
    //       Each channel shares the same `period` but has its own duty
    //       cycle input and a phase offset of period/3 between channels.
    //
    //       channel_counter[i] starts at i * (period / 3).
    //       pwm_rgb[i] = (channel_counter[i] < duty_rgb[i]);
    //
    //       Phase offsetting spreads current draw across time, reducing
    //       peak supply current and visible flicker.
    //
    //       Suggested additional ports:
    //         input  wire [15:0] duty_r, duty_g, duty_b
    //         output reg         pwm_r, pwm_g, pwm_b
    //
    //       This exercise is left entirely for you to implement.

    // ================================================================
    // LED Output Mapping
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            leds <= 8'd0;
        end else begin
            leds[0] <= pwm_raw;
            leds[1] <= pwm_out;
            leds[2] <= pwm_n_out;
            leds[3] <= servo_out;
            leds[4] <= ramp_direction;
            leds[7:5] <= servo_position[7:5];
        end
    end

endmodule
