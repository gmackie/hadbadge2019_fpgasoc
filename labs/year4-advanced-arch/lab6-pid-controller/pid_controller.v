/*
 * Lab 6: Hardware PID Controller for Embedded Control
 * ====================================================
 * Build a hardware PID controller for real-time control applications such as
 * motor speed regulation, temperature control, or position servoing.
 *
 * All arithmetic uses Q16.16 signed fixed-point representation:
 *   Bit 31 = sign, Bits [30:16] = integer part, Bits [15:0] = fractional part
 *   Range: -32768.0 to +32767.99998 with resolution of ~0.0000153
 *
 * Control loop (executes once per sample_tick):
 *
 *   error = setpoint - measurement
 *   P = Kp * error
 *   I = I_prev + Ki * error    (with anti-windup clamping)
 *   D = Kd * (error - error_prev)   (with low-pass filter)
 *   output = clamp(P + I + D, output_min, output_max)
 *
 * EXERCISES:
 *
 * Exercise 1: Proportional term with configurable gain
 *   Implement the error calculation and proportional multiply.
 *   The multiply of two Q16.16 numbers produces a Q32.32 result;
 *   you must extract the Q16.16 portion (bits [47:16]) and handle
 *   overflow by saturating to INT32_MAX or INT32_MIN.
 *
 * Exercise 2: Integral term with anti-windup
 *   Add an accumulator for the integral term. Implement anti-windup:
 *   when the output is saturated (clamped), stop accumulating the
 *   integral to prevent massive overshoot when the error reverses.
 *   Also clamp the integrator itself to prevent overflow.
 *
 * Exercise 3: Derivative term with low-pass filtering
 *   Implement the derivative as the first difference of the error
 *   (or better: the first difference of the *measurement* to avoid
 *   "derivative kick" on setpoint changes). Apply a first-order IIR
 *   low-pass filter to reduce noise amplification:
 *     D_filtered = alpha * D_raw + (1 - alpha) * D_prev
 *   where alpha is a small fraction (e.g., 0.1 in Q16.16 = 0x0000199A).
 *
 * Exercise 4: Setpoint ramping (slew rate limiting)
 *   Instead of applying setpoint changes instantaneously, ramp toward
 *   the target setpoint at a configurable rate. This avoids step inputs
 *   that cause large transients.
 *
 * Exercise 5 (challenge): Auto-tuning via relay feedback
 *   Implement Ziegler-Nichols relay feedback auto-tuning:
 *   - Replace PID with a relay (bang-bang): output = +d if error > 0, else -d
 *   - Measure the resulting oscillation period (Tu) and amplitude (a)
 *   - Compute ultimate gain: Ku = 4*d / (pi*a)
 *   - Set PID gains: Kp = 0.6*Ku, Ki = 2*Kp/Tu, Kd = Kp*Tu/8
 *
 * CONCEPTS:
 *   - PID control theory (proportional, integral, derivative)
 *   - Fixed-point arithmetic (Q16.16 signed representation)
 *   - Integral windup and anti-windup strategies
 *   - Derivative kick and why to differentiate PV, not error
 *   - Sampling rate and control loop bandwidth
 *   - Stability margins and tuning methods
 *   - Ziegler-Nichols tuning rules
 */

`default_nettype none

module pid_controller (
    input  wire        clk,
    input  wire        rst,

    // Control signals
    input  wire        sample_tick,     // Pulse high for one cycle at sample rate
    input  wire        active,          // Enable the controller

    // Setpoint and measurement (Q16.16 signed fixed-point)
    input  wire [31:0] setpoint,        // Desired value
    input  wire [31:0] measurement,     // Actual measured value

    // Gain parameters (Q16.16 signed fixed-point)
    input  wire [31:0] kp,              // Proportional gain
    input  wire [31:0] ki,              // Integral gain
    input  wire [31:0] kd,              // Derivative gain

    // Output limits (Q16.16 signed fixed-point)
    input  wire [31:0] output_min,      // Minimum output value
    input  wire [31:0] output_max,      // Maximum output value

    // Output
    output reg  [31:0] output_val,      // PID output (Q16.16)
    output reg         saturated        // Output is at limit (for anti-windup feedback)
);

    // =========================================================================
    // Q16.16 fixed-point constants
    // =========================================================================
    localparam signed [31:0] Q16_ZERO = 32'sh0000_0000;
    localparam signed [31:0] Q16_ONE  = 32'sh0001_0000;   // 1.0
    localparam signed [31:0] Q16_MAX  = 32'sh7FFF_FFFF;   // +32767.99998
    localparam signed [31:0] Q16_MIN  = 32'sh8000_0000;   // -32768.0

    // Derivative filter coefficient (alpha ~ 0.1 in Q16.16)
    localparam signed [31:0] DERIV_ALPHA     = 32'sh0000_199A;  // ~0.1
    localparam signed [31:0] DERIV_ONE_MINUS = 32'sh0000_E666;  // ~0.9

    // =========================================================================
    // Internal registers
    // =========================================================================
    reg signed [31:0] error;                 // Current error
    reg signed [31:0] error_prev;            // Previous error (for derivative)
    reg signed [31:0] meas_prev;             // Previous measurement (for derivative kick avoidance)
    reg signed [31:0] integral_acc;          // Integral accumulator
    reg signed [31:0] deriv_filtered;        // Filtered derivative
    reg signed [31:0] setpoint_ramped;       // Ramped setpoint (Exercise 4)

    // Intermediate products (64-bit to hold Q16.16 * Q16.16 = Q32.32)
    reg signed [63:0] p_product;
    reg signed [63:0] i_product;
    reg signed [63:0] d_product;
    reg signed [63:0] deriv_alpha_prod;
    reg signed [63:0] deriv_beta_prod;

    // Extracted Q16.16 from products
    reg signed [31:0] p_term;
    reg signed [31:0] i_delta;
    reg signed [31:0] d_raw;
    reg signed [31:0] d_term;

    // Summation
    reg signed [31:0] pid_sum;
    reg signed [31:0] pid_clamped;

    // =========================================================================
    // FSM for pipelined PID computation
    // =========================================================================
    // The PID computation is pipelined across multiple cycles to meet timing:
    //   Cycle 0: Compute error, derivative raw value
    //   Cycle 1: Multiply (P, I, D products)
    //   Cycle 2: Extract Q16.16, filter derivative
    //   Cycle 3: Sum, clamp, output

    localparam [2:0] PID_IDLE = 3'd0,
                     PID_ERR  = 3'd1,
                     PID_MULT = 3'd2,
                     PID_SUM  = 3'd3,
                     PID_OUT  = 3'd4;

    reg [2:0] pid_state;

    // =========================================================================
    // Q16.16 multiply with saturation
    // =========================================================================
    // Multiplying two Q16.16 values gives a Q32.32 result in 64 bits.
    // The Q16.16 result is bits [47:16]. We must check bits [63:48] for overflow.
    function signed [31:0] q16_mul_extract;
        input signed [63:0] product;
        reg signed [31:0] result;
        reg overflow_pos, overflow_neg;
        begin
            result = product[47:16];
            // Check if upper bits indicate overflow
            // For positive result, bits [63:48] should all be 0
            // For negative result, bits [63:48] should all be 1
            overflow_pos = !product[63] && (product[63:47] != 17'd0);
            overflow_neg =  product[63] && (product[63:47] != {17{1'b1}});

            if (overflow_pos)
                q16_mul_extract = Q16_MAX;
            else if (overflow_neg)
                q16_mul_extract = Q16_MIN;
            else
                q16_mul_extract = result;
        end
    endfunction

    // =========================================================================
    // Saturating add for Q16.16
    // =========================================================================
    function signed [31:0] sat_add;
        input signed [31:0] a, b;
        reg signed [32:0] sum;
        begin
            sum = {a[31], a} + {b[31], b};
            if (sum > $signed({1'b0, Q16_MAX}))
                sat_add = Q16_MAX;
            else if (sum < $signed({1'b1, Q16_MIN[30:0]}))
                sat_add = Q16_MIN;
            else
                sat_add = sum[31:0];
        end
    endfunction

    // =========================================================================
    // Clamping function
    // =========================================================================
    function signed [31:0] clamp_val;
        input signed [31:0] val, lo, hi;
        begin
            if ($signed(val) < $signed(lo))
                clamp_val = lo;
            else if ($signed(val) > $signed(hi))
                clamp_val = hi;
            else
                clamp_val = val;
        end
    endfunction

    // =========================================================================
    // Main PID pipeline
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            pid_state      <= PID_IDLE;
            error          <= Q16_ZERO;
            error_prev     <= Q16_ZERO;
            meas_prev      <= Q16_ZERO;
            integral_acc   <= Q16_ZERO;
            deriv_filtered <= Q16_ZERO;
            setpoint_ramped <= Q16_ZERO;
            output_val     <= Q16_ZERO;
            saturated      <= 1'b0;
            p_term         <= Q16_ZERO;
            i_delta        <= Q16_ZERO;
            d_raw          <= Q16_ZERO;
            d_term         <= Q16_ZERO;
            pid_sum        <= Q16_ZERO;
        end else begin
            case (pid_state)

                // ---------------------------------------------------------
                // IDLE: Wait for sample tick
                // ---------------------------------------------------------
                PID_IDLE: begin
                    if (sample_tick && active) begin
                        pid_state <= PID_ERR;
                    end
                end

                // ---------------------------------------------------------
                // ERR: Compute error and raw derivative
                // ---------------------------------------------------------
                PID_ERR: begin
                    // TODO (Exercise 1): Compute error
                    //   error = setpoint_ramped - measurement
                    // (Use setpoint_ramped after Exercise 4, or setpoint directly at first)

                    error <= $signed(setpoint) - $signed(measurement);

                    // TODO (Exercise 3): Compute raw derivative
                    // Option A: Derivative of error (simple but has derivative kick)
                    //   d_raw = error - error_prev
                    // Option B (better): Derivative of measurement (avoids kick)
                    //   d_raw = -(measurement - meas_prev)

                    d_raw <= -($signed(measurement) - $signed(meas_prev));

                    pid_state <= PID_MULT;
                end

                // ---------------------------------------------------------
                // MULT: Compute P, I, D products
                // ---------------------------------------------------------
                PID_MULT: begin
                    // TODO (Exercise 1): Proportional term
                    //   p_product = kp * error  (64-bit signed multiply)
                    p_product <= $signed(kp) * $signed(error);

                    // TODO (Exercise 2): Integral delta
                    //   i_product = ki * error
                    i_product <= $signed(ki) * $signed(error);

                    // TODO (Exercise 3): Derivative with IIR filter
                    //   d_product = kd * d_raw
                    //   deriv_alpha_prod = DERIV_ALPHA * (kd * d_raw)   -- new term
                    //   deriv_beta_prod  = DERIV_ONE_MINUS * deriv_filtered  -- old filtered
                    d_product <= $signed(kd) * $signed(d_raw);

                    deriv_alpha_prod <= $signed(DERIV_ALPHA) * $signed(d_raw);
                    deriv_beta_prod  <= $signed(DERIV_ONE_MINUS) * $signed(deriv_filtered);

                    pid_state <= PID_SUM;
                end

                // ---------------------------------------------------------
                // SUM: Extract Q16.16, accumulate integral, sum terms
                // ---------------------------------------------------------
                PID_SUM: begin
                    // Extract P term
                    p_term <= q16_mul_extract(p_product);

                    // TODO (Exercise 2): Accumulate integral with anti-windup
                    //   Only accumulate if output is NOT saturated (anti-windup)
                    //   Also clamp the accumulator to prevent overflow
                    i_delta <= q16_mul_extract(i_product);

                    if (!saturated) begin
                        integral_acc <= clamp_val(
                            sat_add(integral_acc, q16_mul_extract(i_product)),
                            output_min, output_max
                        );
                    end
                    // else: freeze integrator (anti-windup)

                    // TODO (Exercise 3): Apply IIR filter to derivative
                    //   D_filtered = alpha * D_raw + (1 - alpha) * D_prev
                    d_term <= sat_add(
                        q16_mul_extract(deriv_alpha_prod),
                        q16_mul_extract(deriv_beta_prod)
                    );
                    deriv_filtered <= sat_add(
                        q16_mul_extract(deriv_alpha_prod),
                        q16_mul_extract(deriv_beta_prod)
                    );

                    pid_state <= PID_OUT;
                end

                // ---------------------------------------------------------
                // OUT: Final sum, clamp, and output
                // ---------------------------------------------------------
                PID_OUT: begin
                    // Sum all three terms
                    pid_sum <= sat_add(sat_add(p_term, integral_acc), d_term);

                    // Clamp to output limits
                    pid_clamped = clamp_val(
                        sat_add(sat_add(p_term, integral_acc), d_term),
                        output_min, output_max
                    );

                    output_val <= pid_clamped;

                    // Set saturated flag for anti-windup feedback
                    saturated <= (pid_clamped == output_min) || (pid_clamped == output_max);

                    // Save state for next iteration
                    error_prev <= error;
                    meas_prev  <= $signed(measurement);

                    pid_state <= PID_IDLE;
                end

                default: pid_state <= PID_IDLE;
            endcase
        end
    end

    // =========================================================================
    // TODO (Exercise 4): Setpoint ramping
    // =========================================================================
    //
    // Implement slew rate limiting on the setpoint to avoid step changes:
    //
    //   parameter SLEW_RATE = 32'sh0000_0100;  // Max change per sample
    //
    //   always @(posedge clk) begin
    //       if (rst) begin
    //           setpoint_ramped <= Q16_ZERO;
    //       end else if (sample_tick && active) begin
    //           if ($signed(setpoint) > $signed(setpoint_ramped + SLEW_RATE))
    //               setpoint_ramped <= sat_add(setpoint_ramped, SLEW_RATE);
    //           else if ($signed(setpoint) < $signed(setpoint_ramped - SLEW_RATE))
    //               setpoint_ramped <= sat_add(setpoint_ramped, -SLEW_RATE);
    //           else
    //               setpoint_ramped <= setpoint;
    //       end
    //   end
    //
    // Then change the error computation in PID_ERR to use setpoint_ramped.

    // =========================================================================
    // TODO (Exercise 5 - Challenge): Relay feedback auto-tuning
    // =========================================================================
    //
    // Implement Ziegler-Nichols relay feedback method:
    //
    // 1. Add an auto_tune input and relay_amplitude parameter
    // 2. When auto_tune is active, bypass PID and output:
    //      output = (error > 0) ? +relay_amplitude : -relay_amplitude
    // 3. Measure oscillation:
    //    - Detect zero-crossings of the error signal
    //    - Measure the period Tu between crossings (count sample ticks)
    //    - Measure the amplitude 'a' of the oscillation (track min/max of measurement)
    // 4. After N complete cycles (e.g., 10), compute:
    //    - Ku = 4 * relay_amplitude / (pi * a)   [pi ~ 3.14159, approximate as 3+1/7]
    //    - Set kp = 0.6 * Ku
    //    - Set ki = 2 * kp / Tu
    //    - Set kd = kp * Tu / 8
    // 5. Switch back to PID mode with computed gains
    //
    // NOTE: Division in hardware is complex. Consider using shift-based
    // approximations or a sequential divider.

endmodule
