/*
 * Lab 7: DSP Accelerator - FFT Engine
 * =====================================
 * Build a hardware FFT (Fast Fourier Transform) accelerator for
 * spectral analysis. This module implements an 8-point radix-2
 * decimation-in-time (DIT) FFT using a single time-multiplexed
 * butterfly unit.
 *
 * The 8-point FFT has 3 stages (log2(8) = 3), each with 4 butterflies:
 *
 *   Stage 0: pairs (0,1), (2,3), (4,5), (6,7)   twiddle: W8^0 only
 *   Stage 1: pairs (0,2), (1,3), (4,6), (5,7)   twiddle: W8^0, W8^2
 *   Stage 2: pairs (0,4), (1,5), (2,6), (3,7)   twiddle: W8^0, W8^1, W8^2, W8^3
 *
 * Data format: Q1.15 fixed-point complex numbers
 *   Each sample is 32 bits: {real[15:0], imag[15:0]}
 *   Real and imaginary are signed Q1.15: range [-1.0, +0.99997]
 *   Bit 15 = sign, Bits [14:0] = fractional part
 *
 * EXERCISES:
 *
 * Exercise 1: Radix-2 butterfly unit
 *   Implement the butterfly operation:
 *     A' = A + W * B
 *     B' = A - W * B
 *   where W is the twiddle factor and all values are complex.
 *   Complex multiply: (a+jb)(c+jd) = (ac-bd) + j(ad+bc)
 *   This requires 4 real multiplies and 2 additions/subtractions.
 *
 * Exercise 2: Twiddle factor ROM
 *   Precompute the twiddle factors W8^k = e^(-j*2*pi*k/8) for k=0..3:
 *     W8^0 = ( 1.000,  0.000)  = (0x7FFF, 0x0000)
 *     W8^1 = ( 0.707, -0.707)  = (0x5A82, 0xA57E)
 *     W8^2 = ( 0.000, -1.000)  = (0x0000, 0x8001)
 *     W8^3 = (-0.707, -0.707)  = (0xA57E, 0xA57E)
 *
 * Exercise 3: Address generator
 *   Implement bit-reversal permutation for input ordering and the
 *   stage/butterfly indexing that selects which pairs to process
 *   and which twiddle factor to use at each step.
 *
 * Exercise 4: Complete 8-point FFT
 *   Wire everything together: sample RAM, butterfly unit, twiddle ROM,
 *   and address generator FSM to execute all 12 butterflies (3 stages x 4).
 *
 * Exercise 5 (challenge): Extend to 256-point FFT
 *   Scale up to N=256 (8 stages, 128 butterflies per stage).
 *   Use block RAM for sample storage and a 128-entry twiddle ROM.
 *   Consider pipelining the butterfly for higher throughput.
 *
 * CONCEPTS:
 *   - DFT and FFT algorithm (Cooley-Tukey)
 *   - Radix-2 decimation-in-time decomposition
 *   - Butterfly operations and signal flow graphs
 *   - Twiddle factors as roots of unity
 *   - Bit-reversal addressing for in-place computation
 *   - Complex arithmetic in fixed-point hardware
 *   - Fixed-point complex multiply: 4 real multiplies, 2 adds
 *   - Pipeline throughput vs latency tradeoffs
 *   - Spectral analysis applications (audio, vibration, comms)
 */

`default_nettype none

module dsp_accelerator (
    input  wire        clk,
    input  wire        rst,

    // Control
    input  wire        start,           // Pulse to begin FFT computation
    output reg         done,            // Pulse when FFT is complete
    output reg         busy,            // High during computation

    // Sample write interface (load input samples before start)
    input  wire [2:0]  sample_wr_addr,  // 0..7 for 8-point FFT
    input  wire [31:0] sample_wr_data,  // {real[15:0], imag[15:0]}
    input  wire        sample_wr_en,

    // Result read interface (read after done)
    input  wire [2:0]  result_rd_addr,
    output wire [31:0] result_rd_data,

    // Debug outputs
    output reg  [1:0]  fft_stage        // Current FFT stage (0, 1, 2)
);

    // =========================================================================
    // Parameters
    // =========================================================================
    localparam N       = 8;             // FFT size
    localparam LOG2N   = 3;             // log2(N) = number of stages
    localparam N_HALF  = N / 2;         // Butterflies per stage

    // =========================================================================
    // Q1.15 fixed-point constants
    // =========================================================================
    localparam signed [15:0] Q15_ONE     = 16'sh7FFF;   // +0.99997 ~ 1.0
    localparam signed [15:0] Q15_NEG_ONE = 16'sh8001;   // -0.99997 ~ -1.0
    localparam signed [15:0] Q15_ZERO    = 16'sh0000;
    localparam signed [15:0] Q15_SQRT2_2 = 16'sh5A82;   // +0.7071 ~ sqrt(2)/2
    localparam signed [15:0] Q15_NSQRT2  = 16'shA57E;   // -0.7071

    // =========================================================================
    // Twiddle factor ROM (Exercise 2)
    // =========================================================================
    // W8^k = cos(2*pi*k/8) - j*sin(2*pi*k/8) stored as {real, imag}
    //
    // TODO (Exercise 2): Verify these values and understand the derivation.
    //   W8^0 = e^(-j*0)       = ( 1.000,  0.000)
    //   W8^1 = e^(-j*pi/4)    = ( 0.707, -0.707)
    //   W8^2 = e^(-j*pi/2)    = ( 0.000, -1.000)
    //   W8^3 = e^(-j*3*pi/4)  = (-0.707, -0.707)

    reg signed [15:0] twiddle_real [0:3];
    reg signed [15:0] twiddle_imag [0:3];

    initial begin
        twiddle_real[0] = Q15_ONE;       twiddle_imag[0] = Q15_ZERO;      // W8^0
        twiddle_real[1] = Q15_SQRT2_2;   twiddle_imag[1] = Q15_NSQRT2;    // W8^1
        twiddle_real[2] = Q15_ZERO;      twiddle_imag[2] = Q15_NEG_ONE;   // W8^2
        twiddle_real[3] = Q15_NSQRT2;    twiddle_imag[3] = Q15_NSQRT2;    // W8^3
    end

    // =========================================================================
    // Sample / working RAM (dual-port: write from input or butterfly, read for butterfly)
    // =========================================================================
    reg [31:0] sample_ram [0:N-1];
    integer i;

    initial begin
        for (i = 0; i < N; i = i + 1)
            sample_ram[i] = 32'd0;
    end

    // Result read port
    assign result_rd_data = sample_ram[result_rd_addr];

    // =========================================================================
    // Bit-reversal function (Exercise 3)
    // =========================================================================
    // For 8-point FFT (3-bit indices), bit-reverse maps:
    //   000 -> 000  (0 -> 0)
    //   001 -> 100  (1 -> 4)
    //   010 -> 010  (2 -> 2)
    //   011 -> 110  (3 -> 6)
    //   100 -> 001  (4 -> 1)
    //   101 -> 101  (5 -> 5)
    //   110 -> 011  (6 -> 3)
    //   111 -> 111  (7 -> 7)

    function [2:0] bit_reverse;
        input [2:0] addr;
        bit_reverse = {addr[0], addr[1], addr[2]};
    endfunction

    // =========================================================================
    // Butterfly unit (Exercise 1)
    // =========================================================================
    // Inputs: A (complex), B (complex), W (twiddle factor, complex)
    // Outputs: A' = A + W*B,  B' = A - W*B
    //
    // Complex multiply W*B = (wr + j*wi)(br + j*bi)
    //                      = (wr*br - wi*bi) + j(wr*bi + wi*br)

    reg signed [15:0] bfly_ar, bfly_ai;     // Input A (real, imag)
    reg signed [15:0] bfly_br, bfly_bi;     // Input B (real, imag)
    reg signed [15:0] bfly_wr, bfly_wi;     // Twiddle W (real, imag)

    // Intermediate products (Q1.15 * Q1.15 = Q2.30, need 32 bits)
    wire signed [31:0] mul_wr_br, mul_wi_bi, mul_wr_bi, mul_wi_br;

    // TODO (Exercise 1): Implement the four real multiplications
    // The multiply of two Q1.15 numbers gives a Q2.30 result.
    // We shift right by 15 to get back to Q1.15 (with extra precision).
    assign mul_wr_br = bfly_wr * bfly_br;
    assign mul_wi_bi = bfly_wi * bfly_bi;
    assign mul_wr_bi = bfly_wr * bfly_bi;
    assign mul_wi_br = bfly_wi * bfly_br;

    // Complex product WB = (wr*br - wi*bi) + j(wr*bi + wi*br)
    // Shift right by 15 to convert Q2.30 back to Q1.15
    wire signed [15:0] wb_real, wb_imag;

    // TODO (Exercise 1): Extract Q1.15 from Q2.30 products
    // HINT: Take bits [30:15] of the 32-bit product for Q1.15 result
    assign wb_real = (mul_wr_br - mul_wi_bi) >>> 15;
    assign wb_imag = (mul_wr_bi + mul_wi_br) >>> 15;

    // Butterfly outputs: A' = A + WB, B' = A - WB
    wire signed [15:0] bfly_out_ar, bfly_out_ai;
    wire signed [15:0] bfly_out_br, bfly_out_bi;

    // TODO (Exercise 1): Compute butterfly outputs with saturation
    // NOTE: Addition of two Q1.15 values can overflow. For simplicity,
    // we use Q1.15 and accept minor clipping, or use an extra guard bit.
    assign bfly_out_ar = bfly_ar + wb_real;
    assign bfly_out_ai = bfly_ai + wb_imag;
    assign bfly_out_br = bfly_ar - wb_real;
    assign bfly_out_bi = bfly_ai - wb_imag;

    // =========================================================================
    // FFT control FSM (Exercises 3 & 4)
    // =========================================================================
    localparam [2:0] FFT_IDLE      = 3'd0,
                     FFT_BITREV    = 3'd1,  // Bit-reverse input permutation
                     FFT_LOAD      = 3'd2,  // Load butterfly inputs from RAM
                     FFT_COMPUTE   = 3'd3,  // Butterfly computation (1 cycle latency)
                     FFT_STORE     = 3'd4,  // Store butterfly outputs to RAM
                     FFT_NEXT      = 3'd5,  // Advance to next butterfly/stage
                     FFT_DONE      = 3'd6;

    reg [2:0]  fft_state;
    reg [1:0]  stage;            // Current stage (0, 1, 2)
    reg [1:0]  bfly_idx;         // Current butterfly within stage (0..3)

    // Address generation (Exercise 3)
    // For stage s, butterfly b:
    //   Block size = 2^(s+1)
    //   Half block = 2^s
    //   Group = b >> s            (which group of butterflies)
    //   pair_offset = b & (half-1)  (position within group)
    //   addr_a = group * block_size + pair_offset
    //   addr_b = addr_a + half_block
    //   twiddle_index = pair_offset * (N / block_size) = pair_offset << (LOG2N - 1 - s)

    reg [2:0] addr_a, addr_b;
    reg [1:0] tw_idx;

    // Bit-reversal permutation state
    reg [2:0] bitrev_idx;
    reg [31:0] bitrev_temp [0:N-1];

    // =========================================================================
    // Address generation logic
    // =========================================================================
    // TODO (Exercise 3): Implement the address generator
    // This is the heart of the FFT controller - it determines which
    // pairs of samples to process and which twiddle factor to use.

    always @(*) begin
        // Default
        addr_a = 3'd0;
        addr_b = 3'd0;
        tw_idx = 2'd0;

        case (stage)
            2'd0: begin
                // Stage 0: pairs separated by 1, block size 2
                // Butterflies: (0,1), (2,3), (4,5), (6,7)
                addr_a = {bfly_idx, 1'b0};      // 0, 2, 4, 6
                addr_b = {bfly_idx, 1'b1};      // 1, 3, 5, 7
                tw_idx = 2'd0;                   // Always W8^0 in stage 0
            end
            2'd1: begin
                // Stage 1: pairs separated by 2, block size 4
                // Butterflies: (0,2), (1,3), (4,6), (5,7)
                addr_a = {bfly_idx[1], 1'b0, bfly_idx[0]};  // 0, 1, 4, 5
                addr_b = {bfly_idx[1], 1'b1, bfly_idx[0]};  // 2, 3, 6, 7
                tw_idx = {1'b0, bfly_idx[0]} << 1;           // W8^0 or W8^2
            end
            2'd2: begin
                // Stage 2: pairs separated by 4, block size 8
                // Butterflies: (0,4), (1,5), (2,6), (3,7)
                addr_a = {1'b0, bfly_idx};       // 0, 1, 2, 3
                addr_b = {1'b1, bfly_idx};       // 4, 5, 6, 7
                tw_idx = bfly_idx;               // W8^0, W8^1, W8^2, W8^3
            end
            default: begin
                addr_a = 3'd0;
                addr_b = 3'd0;
                tw_idx = 2'd0;
            end
        endcase
    end

    // =========================================================================
    // Main FFT FSM
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            fft_state  <= FFT_IDLE;
            done       <= 1'b0;
            busy       <= 1'b0;
            stage      <= 2'd0;
            bfly_idx   <= 2'd0;
            fft_stage  <= 2'd0;
            bitrev_idx <= 3'd0;
            bfly_ar    <= 16'd0;
            bfly_ai    <= 16'd0;
            bfly_br    <= 16'd0;
            bfly_bi    <= 16'd0;
            bfly_wr    <= 16'd0;
            bfly_wi    <= 16'd0;
        end else begin
            done <= 1'b0;  // Default: pulse only

            case (fft_state)
                // ---------------------------------------------------------
                // IDLE: Wait for start signal
                // ---------------------------------------------------------
                FFT_IDLE: begin
                    if (start) begin
                        busy       <= 1'b1;
                        bitrev_idx <= 3'd0;
                        fft_state  <= FFT_BITREV;
                    end
                end

                // ---------------------------------------------------------
                // BITREV: Bit-reverse permutation of input samples
                // ---------------------------------------------------------
                // TODO (Exercise 3): Implement in-place bit-reversal
                // The DIT FFT expects input in bit-reversed order.
                // We copy all samples to a temp buffer in bit-reversed order,
                // then write them back. (A more efficient approach swaps pairs.)
                FFT_BITREV: begin
                    if (bitrev_idx < N) begin
                        // Read from natural order, store to bit-reversed position
                        bitrev_temp[bit_reverse(bitrev_idx)] <= sample_ram[bitrev_idx];
                        bitrev_idx <= bitrev_idx + 3'd1;
                    end else begin
                        // Write back bit-reversed data
                        for (i = 0; i < N; i = i + 1)
                            sample_ram[i] <= bitrev_temp[i];

                        stage     <= 2'd0;
                        bfly_idx  <= 2'd0;
                        fft_stage <= 2'd0;
                        fft_state <= FFT_LOAD;
                    end
                end

                // ---------------------------------------------------------
                // LOAD: Read butterfly inputs from RAM
                // ---------------------------------------------------------
                FFT_LOAD: begin
                    // TODO (Exercise 4): Load A and B samples, select twiddle
                    bfly_ar <= sample_ram[addr_a][31:16];  // A real
                    bfly_ai <= sample_ram[addr_a][15:0];   // A imag
                    bfly_br <= sample_ram[addr_b][31:16];  // B real
                    bfly_bi <= sample_ram[addr_b][15:0];   // B imag
                    bfly_wr <= twiddle_real[tw_idx];
                    bfly_wi <= twiddle_imag[tw_idx];

                    fft_state <= FFT_COMPUTE;
                end

                // ---------------------------------------------------------
                // COMPUTE: Wait one cycle for butterfly combinational logic
                // ---------------------------------------------------------
                FFT_COMPUTE: begin
                    // Butterfly outputs are ready on the combinational wires
                    fft_state <= FFT_STORE;
                end

                // ---------------------------------------------------------
                // STORE: Write butterfly outputs back to RAM
                // ---------------------------------------------------------
                FFT_STORE: begin
                    // TODO (Exercise 4): Write results back in-place
                    sample_ram[addr_a] <= {bfly_out_ar, bfly_out_ai};
                    sample_ram[addr_b] <= {bfly_out_br, bfly_out_bi};

                    fft_state <= FFT_NEXT;
                end

                // ---------------------------------------------------------
                // NEXT: Advance to next butterfly or next stage
                // ---------------------------------------------------------
                FFT_NEXT: begin
                    if (bfly_idx == 2'd3) begin
                        // All butterflies in this stage done
                        bfly_idx <= 2'd0;
                        if (stage == 2'd2) begin
                            // All stages done - FFT complete
                            fft_state <= FFT_DONE;
                        end else begin
                            stage     <= stage + 2'd1;
                            fft_stage <= stage + 2'd1;
                            fft_state <= FFT_LOAD;
                        end
                    end else begin
                        bfly_idx  <= bfly_idx + 2'd1;
                        fft_state <= FFT_LOAD;
                    end
                end

                // ---------------------------------------------------------
                // DONE: Signal completion
                // ---------------------------------------------------------
                FFT_DONE: begin
                    done      <= 1'b1;
                    busy      <= 1'b0;
                    fft_state <= FFT_IDLE;
                end

                default: fft_state <= FFT_IDLE;
            endcase

            // External sample write (only when idle)
            if (sample_wr_en && !busy) begin
                sample_ram[sample_wr_addr] <= sample_wr_data;
            end
        end
    end

    // =========================================================================
    // TODO (Exercise 5 - Challenge): 256-point FFT extension
    // =========================================================================
    //
    // To scale from 8-point to 256-point:
    //
    // 1. Change parameters:
    //      N = 256, LOG2N = 8, sample addresses become [7:0]
    //
    // 2. Replace sample_ram with block RAM (use (* ram_style = "block" *)):
    //      (* ram_style = "block" *) reg [31:0] sample_ram [0:255];
    //
    // 3. Expand twiddle ROM to 128 entries (N/2):
    //      W256^k for k = 0..127
    //      Consider generating with a script: W = round(32767 * e^(-j*2*pi*k/256))
    //      Or use CORDIC to compute twiddle factors on-the-fly.
    //
    // 4. Generalize address generator:
    //      For stage s (0..7), butterfly b (0..127):
    //        block_size = 1 << (s+1)
    //        half = 1 << s
    //        group = b / half
    //        offset = b % half
    //        addr_a = group * block_size + offset
    //        addr_b = addr_a + half
    //        tw_idx = offset * (N / block_size)
    //
    // 5. Pipeline the butterfly for higher throughput:
    //      Stage 1: Multiply (wr*br, wi*bi, wr*bi, wi*br)
    //      Stage 2: Add/subtract to get WB, compute A+WB and A-WB
    //      This allows starting a new butterfly every cycle.
    //
    // 6. Bit-reverse permutation for 8-bit addresses:
    //      function [7:0] bit_reverse_8;
    //          input [7:0] addr;
    //          bit_reverse_8 = {addr[0],addr[1],addr[2],addr[3],
    //                           addr[4],addr[5],addr[6],addr[7]};
    //      endfunction

endmodule
