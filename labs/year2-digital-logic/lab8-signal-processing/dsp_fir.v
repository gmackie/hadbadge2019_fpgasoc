/*
 * Lab 8: Digital Signal Processing - FIR Filter
 * ==============================================
 * In this lab you will build a hardware FIR (Finite Impulse Response) filter
 * on the ECP5 FPGA badge. FIR filters are the workhorses of digital signal
 * processing, used in audio equalisation, communication systems, image
 * processing, and sensor conditioning. The output of an N-tap FIR filter is
 * the convolution of the input samples with a set of fixed coefficients:
 *
 *   y[n] = sum_{k=0}^{N-1} h[k] * x[n-k]
 *
 * where h[k] are the filter coefficients and x[n-k] are delayed input
 * samples. You will implement the multiply-accumulate (MAC) datapath using
 * signed fixed-point arithmetic (Q1.15 format: 1 sign bit, 15 fractional
 * bits, range -1.0 to +0.99997). The ECP5 has dedicated DSP blocks that
 * the synthesis tool can infer from well-written Verilog multiply
 * operations. Your design will start with a 4-tap filter using hardcoded
 * coefficients and grow to an 8-tap version with runtime-configurable
 * coefficients loaded through a register interface.
 *
 * EXERCISES:
 * Exercise 1: Implement a single multiply-accumulate (MAC) unit. Given
 *             two signed 16-bit inputs, produce a 32-bit product and add
 *             it to a 36-bit accumulator. Ensure the synthesis tool infers
 *             an ECP5 DSP block.
 * Exercise 2: Implement a 4-tap FIR filter with hardcoded coefficients.
 *             Maintain a 4-sample delay line. For each new input sample,
 *             multiply all taps by their coefficients and sum the products.
 * Exercise 3: Extend to an 8-tap filter with configurable coefficients.
 *             Coefficients are loaded through the coeff_addr/coeff_data/
 *             coeff_wen interface. Store them in a small register file.
 * Exercise 4: Add a circular buffer in block RAM to hold the sample
 *             history. Use a write pointer that wraps around, eliminating
 *             the need to shift all samples every cycle.
 * Exercise 5 (Challenge): Implement an 8-point DFT (Discrete Fourier
 *             Transform) by time-multiplexing the MAC unit with twiddle
 *             factor coefficients. Compute one DFT output bin per 8
 *             MAC cycles.
 *
 * CONCEPTS:
 *   - FIR filter theory: convolution, impulse response, frequency response
 *   - Multiply-accumulate (MAC) operations
 *   - Fixed-point representation (Q1.15) and overflow handling
 *   - Circular buffers for efficient delay-line implementation
 *   - Filter coefficient design: low-pass, high-pass, band-pass
 *   - FPGA DSP block inference (ECP5 MULT18X18D)
 *   - Pipelining for throughput vs. latency trade-off
 *
 * LED MAPPING:
 *   LED[0]   = sample_valid input (blinks when samples arrive)
 *   LED[1]   = result_valid output
 *   LED[2]   = MAC busy indicator
 *   LED[3]   = Accumulator overflow warning
 *   LED[7:4] = Upper 4 bits of result magnitude (VU meter)
 */

`default_nettype none

module dsp_fir (
    input  wire               clk,          // 48 MHz system clock
    input  wire               rst,          // Synchronous reset, active high
    // Sample interface
    input  wire signed [15:0] sample_in,    // Input sample (Q1.15 signed)
    input  wire               sample_valid, // Pulse high when sample_in is valid
    output reg  signed [15:0] result_out,   // Filter output (Q1.15 signed)
    output reg                result_valid, // Pulses high when result_out is valid
    // Coefficient loading interface
    input  wire [2:0]         coeff_addr,   // Coefficient index (0..7)
    input  wire signed [15:0] coeff_data,   // Coefficient value (Q1.15 signed)
    input  wire               coeff_wen     // Write enable for coefficient
);

    // ================================================================
    // Parameters
    // ================================================================
    localparam NUM_TAPS = 8;

    // ================================================================
    // Coefficient Storage
    // ================================================================
    // 8 coefficients, each 16-bit signed (Q1.15).
    // Default values implement a simple low-pass FIR:
    //   h = [0.0625, 0.125, 0.1875, 0.25, 0.25, 0.1875, 0.125, 0.0625]
    // In Q1.15: multiply by 32768 and round to integer.
    reg signed [15:0] coeff [0:NUM_TAPS-1];

    // Default coefficient values (symmetric low-pass)
    localparam signed [15:0] COEFF_DEFAULT_0 = 16'sd2048;   // ~0.0625
    localparam signed [15:0] COEFF_DEFAULT_1 = 16'sd4096;   // ~0.125
    localparam signed [15:0] COEFF_DEFAULT_2 = 16'sd6144;   // ~0.1875
    localparam signed [15:0] COEFF_DEFAULT_3 = 16'sd8192;   // ~0.25
    localparam signed [15:0] COEFF_DEFAULT_4 = 16'sd8192;   // ~0.25
    localparam signed [15:0] COEFF_DEFAULT_5 = 16'sd6144;   // ~0.1875
    localparam signed [15:0] COEFF_DEFAULT_6 = 16'sd4096;   // ~0.125
    localparam signed [15:0] COEFF_DEFAULT_7 = 16'sd2048;   // ~0.0625

    // ================================================================
    // Sample Delay Line
    // ================================================================
    // Holds the most recent NUM_TAPS samples for convolution.
    // delay[0] = most recent sample, delay[N-1] = oldest.
    reg signed [15:0] delay [0:NUM_TAPS-1];

    // ================================================================
    // MAC Datapath
    // ================================================================
    reg signed [15:0] mac_a;          // Multiplicand (sample)
    reg signed [15:0] mac_b;          // Multiplier (coefficient)
    wire signed [31:0] mac_product;   // 32-bit product
    reg signed [35:0] accumulator;    // 36-bit accumulator (4 guard bits)
    reg               overflow;       // Accumulator overflow flag

    // ================================================================
    // Convolution Control FSM
    // ================================================================
    localparam CONV_IDLE    = 2'd0;   // Waiting for sample_valid
    localparam CONV_COMPUTE = 2'd1;   // Multiplying and accumulating taps
    localparam CONV_OUTPUT  = 2'd2;   // Extracting result

    reg [1:0] conv_state;
    reg [2:0] tap_index;              // Current tap being computed (0..7)

    // ================================================================
    // Exercise 1: Multiply-Accumulate (MAC) Unit
    // ================================================================
    // TODO: Implement the signed multiplier. The ECP5 will infer a
    //       DSP block (MULT18X18D) from a well-formed multiply:
    //
    //       assign mac_product = mac_a * mac_b;
    //
    //       The product of two Q1.15 numbers is Q2.30 (32 bits). To
    //       convert back to Q1.15 for the output, you will right-shift
    //       by 15 bits after accumulation.
    //
    // HINT: Declare the product as `signed` so Verilog uses signed
    //       multiplication. The accumulator needs extra bits (36) to
    //       prevent overflow when summing 8 products.

    // TODO: Replace this placeholder with signed multiply
    assign mac_product = 32'sd0;  // <-- replace with: mac_a * mac_b

    // ================================================================
    // Coefficient Initialisation and Runtime Loading (Exercise 3)
    // ================================================================
    integer c;
    always @(posedge clk) begin
        if (rst) begin
            coeff[0] <= COEFF_DEFAULT_0;
            coeff[1] <= COEFF_DEFAULT_1;
            coeff[2] <= COEFF_DEFAULT_2;
            coeff[3] <= COEFF_DEFAULT_3;
            coeff[4] <= COEFF_DEFAULT_4;
            coeff[5] <= COEFF_DEFAULT_5;
            coeff[6] <= COEFF_DEFAULT_6;
            coeff[7] <= COEFF_DEFAULT_7;
        end else if (coeff_wen) begin
            // TODO (Exercise 3): Write coeff_data to coeff[coeff_addr]
            // coeff[coeff_addr] <= coeff_data;
        end
    end

    // ================================================================
    // Sample Delay Line Management
    // ================================================================
    integer s;
    always @(posedge clk) begin
        if (rst) begin
            for (s = 0; s < NUM_TAPS; s = s + 1)
                delay[s] <= 16'sd0;
        end else if (sample_valid && conv_state == CONV_IDLE) begin
            // Shift delay line: newest sample enters at index 0
            // TODO (Exercise 2): Shift all samples down by one position
            //   delay[0] <= sample_in;
            //   for (s = 1; s < NUM_TAPS; s = s + 1)
            //       delay[s] <= delay[s-1];

            delay[0] <= sample_in;  // Provided: load newest sample
            // TODO: Shift delay[1] through delay[NUM_TAPS-1]
        end
    end

    // ================================================================
    // Exercise 2: Convolution Pipeline (4-tap, then extend to 8-tap)
    // ================================================================
    // TODO: Implement the convolution state machine. On each new sample:
    //   1. Clear accumulator to zero.
    //   2. For each tap (0 .. NUM_TAPS-1):
    //      a. Load mac_a = delay[tap_index]
    //      b. Load mac_b = coeff[tap_index]
    //      c. Wait one cycle for the multiply to complete.
    //      d. Add mac_product to accumulator.
    //   3. Extract result: result_out = accumulator[30:15] (Q2.30 -> Q1.15)
    //      with saturation on overflow.
    //   4. Pulse result_valid for one cycle.
    //
    // For the 4-tap version (Exercise 2), only iterate tap_index 0..3.
    // For the 8-tap version (Exercise 3), iterate tap_index 0..7.

    always @(posedge clk) begin
        if (rst) begin
            conv_state   <= CONV_IDLE;
            tap_index    <= 3'd0;
            accumulator  <= 36'sd0;
            mac_a        <= 16'sd0;
            mac_b        <= 16'sd0;
            result_out   <= 16'sd0;
            result_valid <= 1'b0;
            overflow     <= 1'b0;
        end else begin
            result_valid <= 1'b0;  // Default: single-cycle pulse

            case (conv_state)
                CONV_IDLE: begin
                    if (sample_valid) begin
                        // TODO: Clear accumulator and start computation
                        accumulator <= 36'sd0;
                        tap_index   <= 3'd0;
                        conv_state  <= CONV_COMPUTE;
                    end
                end

                CONV_COMPUTE: begin
                    // TODO: Feed delay[tap_index] and coeff[tap_index]
                    //       into the MAC, accumulate the product, and
                    //       advance tap_index.
                    //
                    // mac_a <= delay[tap_index];
                    // mac_b <= coeff[tap_index];
                    // accumulator <= accumulator + mac_product;
                    //
                    // When tap_index reaches NUM_TAPS-1, go to CONV_OUTPUT.

                    // Placeholder: skip directly to output
                    conv_state <= CONV_OUTPUT;
                end

                CONV_OUTPUT: begin
                    // TODO: Extract the Q1.15 result from the accumulator.
                    //       The accumulator holds a Q2.30 sum. To get Q1.15,
                    //       right-shift by 15: result = accumulator[30:15].
                    //
                    //       Add saturation: if the upper bits indicate
                    //       overflow, clamp to max positive or min negative.
                    //
                    // if (accumulator[35:30] != {6{accumulator[30]}})
                    //     // Overflow: saturate
                    //     result_out <= accumulator[35] ? -16'sd32768 : 16'sd32767;
                    // else
                    //     result_out <= accumulator[30:15];

                    result_out   <= 16'sd0;  // <-- replace with extraction logic
                    result_valid <= 1'b1;
                    conv_state   <= CONV_IDLE;
                end

                default: conv_state <= CONV_IDLE;
            endcase
        end
    end

    // ================================================================
    // Exercise 4: Circular Buffer in Block RAM
    // ================================================================
    // TODO: Replace the shift-register delay line with a circular buffer
    //       stored in ECP5 block RAM. This is more efficient for large
    //       tap counts.
    //
    //       Implementation sketch:
    //         - reg [15:0] sample_ram [0:NUM_TAPS-1]; // inferred BRAM
    //         - reg [2:0]  write_ptr;  // points to oldest sample
    //
    //       On new sample:
    //         sample_ram[write_ptr] <= sample_in;
    //         write_ptr <= write_ptr + 1;  // wraps naturally for power-of-2
    //
    //       During convolution, read samples at:
    //         read_addr = write_ptr - 1 - tap_index  (modular arithmetic)
    //
    //       This eliminates the costly shift of all delay line elements.

    // ================================================================
    // Exercise 5 (Challenge): 8-Point DFT
    // ================================================================
    // TODO: Implement a Discrete Fourier Transform using the same MAC
    //       unit. The DFT of 8 points is:
    //
    //         X[k] = sum_{n=0}^{7} x[n] * W_8^{nk}
    //
    //       where W_8^{nk} = cos(2*pi*n*k/8) - j*sin(2*pi*n*k/8).
    //
    //       Since W_8 twiddle factors for N=8 are drawn from the set
    //       {0, +/-1, +/-0.707}, they can be stored in a small ROM.
    //
    //       For each output bin k (0..7):
    //         1. Clear real and imaginary accumulators.
    //         2. For each sample n (0..7):
    //            real_acc += x[n] * twiddle_real[n*k mod 8]
    //            imag_acc += x[n] * twiddle_imag[n*k mod 8]
    //         3. Compute magnitude: |X[k]| ~ |real| + |imag| (approximate)
    //
    //       Suggested twiddle ROM (Q1.15 format):
    //         cos(0)=32767, cos(pi/4)=23170, cos(pi/2)=0,
    //         cos(3pi/4)=-23170, cos(pi)=-32768
    //
    //       This exercise is left entirely for you to implement.

    // ================================================================
    // Overflow Detection
    // ================================================================
    // Monitor the accumulator guard bits for overflow during computation.
    always @(posedge clk) begin
        if (rst) begin
            overflow <= 1'b0;
        end else if (conv_state == CONV_IDLE) begin
            overflow <= 1'b0;
        end else if (conv_state == CONV_COMPUTE) begin
            // Check if upper guard bits differ from the sign bit
            if (accumulator[35:31] != {5{accumulator[30]}})
                overflow <= 1'b1;
        end
    end

endmodule
