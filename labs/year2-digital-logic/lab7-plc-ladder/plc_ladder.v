/*
 * Lab 7: PLC Ladder Logic Engine
 * ==============================
 * In this lab you will build a hardware Programmable Logic Controller (PLC)
 * that executes ladder logic programs directly in the FPGA fabric. Real
 * PLCs are the backbone of industrial automation, controlling everything
 * from conveyor belts to robotic assembly lines. A PLC repeatedly runs a
 * "scan cycle": read physical inputs into an image register, evaluate all
 * ladder rungs sequentially, then copy the output image register to
 * physical outputs. Each ladder rung is a series/parallel combination of
 * "contacts" (input tests) driving "coils" (output assignments). Your
 * engine will execute a small program stored in an instruction memory,
 * supporting normally-open/normally-closed contacts, output coils, timers
 * (TON/TOF), and counters (CTU/CTD). The badge buttons serve as field
 * inputs and the LEDs as field outputs, giving you a tangible feel for
 * industrial control.
 *
 * EXERCISES:
 * Exercise 1: Implement basic ladder rung evaluation. Each rung has a
 *             chain of series contacts (AND logic) and parallel contacts
 *             (OR logic) feeding a single coil. Evaluate left-to-right
 *             and assign the result to the coil address.
 * Exercise 2: Add timer instructions. Implement TON (Timer On-Delay):
 *             when the rung input is true, start accumulating time; when
 *             the accumulator reaches the preset, the timer's done bit
 *             is set. Implement TOF (Timer Off-Delay) similarly.
 * Exercise 3: Add counter instructions. CTU (Count Up) increments on
 *             each false-to-true transition of the rung input. CTD
 *             (Count Down) decrements. When the accumulator reaches the
 *             preset, the counter done bit is set.
 * Exercise 4: Implement a complete conveyor-belt ladder program using
 *             your engine: start button, stop button, emergency stop
 *             (N/C contact), sensor interlock (jam detection), and a
 *             motor-run output with indicator lamp.
 * Exercise 5 (Challenge): Add analog comparison instructions (GRT, LES,
 *             EQU) operating on 8-bit values, and a sequencer output
 *             (SQO) that steps through a table of output patterns on
 *             each trigger.
 *
 * CONCEPTS:
 *   - Ladder logic: normally-open (XIC) and normally-closed (XIO) contacts
 *   - Coils (OTE, OTL, OTU) for output control
 *   - PLC scan cycle: read inputs, execute program, write outputs
 *   - Timer (TON/TOF) and counter (CTU/CTD) instructions
 *   - Safety interlocking and emergency-stop circuits
 *   - IEC 61131-3 programming model
 *
 * LED MAPPING:
 *   LED[0] = Motor run output (conveyor belt motor)
 *   LED[1] = Motor run indicator lamp
 *   LED[2] = Jam sensor alarm
 *   LED[3] = Timer done indicator
 *   LED[4] = Counter done indicator
 *   LED[5] = E-stop active (active-low input, LED on when stopped)
 *   LED[6] = Scan cycle heartbeat (toggles each complete scan)
 *   LED[7] = Run mode indicator
 */

`default_nettype none

module plc_ladder (
    input  wire        clk,          // 48 MHz system clock
    input  wire        rst,          // Synchronous reset, active high
    input  wire [7:0]  inputs,       // Physical inputs (badge buttons)
    output reg  [7:0]  outputs,      // Physical outputs (badge LEDs)
    input  wire        run,          // Run/stop mode select
    output reg  [3:0]  current_rung  // Currently executing rung (debug)
);

    // ================================================================
    // Instruction Encoding
    // ================================================================
    // Each instruction is 16 bits wide:
    //   [15:12] = opcode (instruction type)
    //   [11:8]  = address (which bit/timer/counter to reference)
    //   [7:0]   = operand (preset value, comparison constant, etc.)
    //
    // Opcodes:
    localparam OP_NOP  = 4'h0;   // No operation (skip)
    localparam OP_XIC  = 4'h1;   // Examine If Closed (normally-open contact)
    localparam OP_XIO  = 4'h2;   // Examine If Open (normally-closed contact)
    localparam OP_OTE  = 4'h3;   // Output Energize (coil, follows rung state)
    localparam OP_OTL  = 4'h4;   // Output Latch (set on true, stays on)
    localparam OP_OTU  = 4'h5;   // Output Unlatch (reset on true)
    localparam OP_TON  = 4'h6;   // Timer On-Delay
    localparam OP_TOF  = 4'h7;   // Timer Off-Delay
    localparam OP_CTU  = 4'h8;   // Counter Up
    localparam OP_CTD  = 4'h9;   // Counter Down
    localparam OP_END  = 4'hF;   // End of program / end of rung

    // Address space:
    //   0-7   = input image register bits  (I:0/0 .. I:0/7)
    //   8-15  = output image register bits (O:0/0 .. O:0/7)
    //   16-19 = timer done bits            (T4:0 .. T4:3)
    //   20-23 = counter done bits          (C5:0 .. C5:3)

    // ================================================================
    // Instruction Memory (16 rungs x 4 instructions max)
    // ================================================================
    // Flattened to 64 instruction slots. Each rung is terminated by OP_END.
    // The program ends when an OP_END is encountered at a rung boundary.

    localparam NUM_SLOTS = 64;
    reg [15:0] prog_mem [0:NUM_SLOTS-1];

    // ================================================================
    // Image Registers and Internal State
    // ================================================================
    reg [7:0]  input_image;          // Latched copy of physical inputs
    reg [7:0]  output_image;         // Output state, written to LEDs
    reg        rung_result;          // Accumulated result for current rung

    // Timers (4 available: T4:0 .. T4:3)
    reg [23:0] timer_acc   [0:3];    // Timer accumulators (counts clock ticks)
    reg [23:0] timer_pre   [0:3];    // Timer presets (loaded from operand * scale)
    reg [3:0]  timer_dn;             // Timer done bits (one per timer)
    reg [3:0]  timer_en;             // Timer enable bits
    reg [3:0]  timer_prev_en;        // Previous enable (edge detection for TOF)

    // Counters (4 available: C5:0 .. C5:3)
    reg [15:0] counter_acc [0:3];    // Counter accumulators
    reg [15:0] counter_pre [0:3];    // Counter presets
    reg [3:0]  counter_dn;           // Counter done bits
    reg [3:0]  counter_prev_rung;    // Previous rung result per counter (edge detect)

    // Timer prescaler: divide 48 MHz down to ~100 Hz for readable timing
    // 48_000_000 / 480_000 = 100 ticks per second
    localparam TIMER_PRESCALE = 24'd480000;
    reg [23:0] timer_prescaler;
    reg        timer_tick;           // Pulses high once per 10 ms

    // ================================================================
    // Scan Cycle FSM
    // ================================================================
    localparam SCAN_READ    = 2'd0;  // Read physical inputs
    localparam SCAN_EXECUTE = 2'd1;  // Evaluate ladder program
    localparam SCAN_WRITE   = 2'd2;  // Write output image to physical outputs

    reg [1:0]  scan_state;
    reg [5:0]  pc;                   // Program counter (index into prog_mem)
    reg        scan_heartbeat;       // Toggles each complete scan cycle
    reg [15:0] current_instr;        // Instruction being executed
    wire [3:0] opcode  = current_instr[15:12];
    wire [3:0] address = current_instr[11:8];
    wire [7:0] operand = current_instr[7:0];

    // ================================================================
    // Helper: Read a Bit from the Address Space
    // ================================================================
    // Used by XIC and XIO to read a single bit.
    function read_bit;
        input [3:0] addr;
        begin
            if (addr < 4'd8)
                read_bit = input_image[addr[2:0]];
            else if (addr < 4'd16)
                read_bit = output_image[addr[2:0]];
            else
                read_bit = 1'b0;  // TODO: extend for timer/counter done bits
        end
    endfunction

    // ================================================================
    // Timer Prescaler (generates 100 Hz tick)
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            timer_prescaler <= 24'd0;
            timer_tick      <= 1'b0;
        end else begin
            if (timer_prescaler >= TIMER_PRESCALE - 1) begin
                timer_prescaler <= 24'd0;
                timer_tick      <= 1'b1;
            end else begin
                timer_prescaler <= timer_prescaler + 1'b1;
                timer_tick      <= 1'b0;
            end
        end
    end

    // ================================================================
    // Program Memory Initialisation
    // ================================================================
    // Default program: conveyor belt control
    //   Rung 0: XIC(I:0 btn0=START) -> XIC(~I:1 btn1=ESTOP N/C) -> OTL(O:0 motor)
    //   Rung 1: XIC(I:2 btn2=STOP) -> OTU(O:0 motor)
    //   Rung 2: XIC(O:0 motor) -> OTE(O:1 indicator lamp)
    //   Rung 3: END
    integer i;
    always @(posedge clk) begin
        if (rst) begin
            // Clear all program memory
            for (i = 0; i < NUM_SLOTS; i = i + 1)
                prog_mem[i] <= {OP_NOP, 4'd0, 8'd0};

            // Rung 0: START button AND NOT E-STOP => latch motor
            prog_mem[0]  <= {OP_XIC, 4'd0, 8'd0};   // XIC I:0/0 (start btn)
            prog_mem[1]  <= {OP_XIO, 4'd1, 8'd0};   // XIO I:0/1 (e-stop, N/C)
            prog_mem[2]  <= {OP_OTL, 4'd8, 8'd0};   // OTL O:0/0 (motor latch)
            prog_mem[3]  <= {OP_END, 4'd0, 8'd0};   // End of rung 0

            // Rung 1: STOP button => unlatch motor
            prog_mem[4]  <= {OP_XIC, 4'd2, 8'd0};   // XIC I:0/2 (stop btn)
            prog_mem[5]  <= {OP_OTU, 4'd8, 8'd0};   // OTU O:0/0 (motor unlatch)
            prog_mem[6]  <= {OP_END, 4'd0, 8'd0};   // End of rung 1

            // Rung 2: Motor running => indicator lamp
            prog_mem[7]  <= {OP_XIC, 4'd8, 8'd0};   // XIC O:0/0 (motor status)
            prog_mem[8]  <= {OP_OTE, 4'd9, 8'd0};   // OTE O:0/1 (lamp)
            prog_mem[9]  <= {OP_END, 4'd0, 8'd0};   // End of rung 2

            // Rung 3: E-stop active => alarm LED
            prog_mem[10] <= {OP_XIC, 4'd1, 8'd0};   // XIC I:0/1 (e-stop pressed)
            prog_mem[11] <= {OP_OTE, 4'd13, 8'd0};  // OTE O:0/5 (e-stop LED)
            prog_mem[12] <= {OP_END, 4'd0, 8'd0};   // End of rung 3

            // Program terminator
            prog_mem[13] <= {OP_END, 4'd0, 8'd0};   // End of program
        end
    end

    // ================================================================
    // Exercise 1: Scan Cycle and Rung Evaluation
    // ================================================================
    // TODO: Implement the three-phase scan cycle:
    //   SCAN_READ:    Latch `inputs` into `input_image`.
    //   SCAN_EXECUTE: Step through prog_mem using `pc`. For each instruction:
    //     - XIC: AND rung_result with read_bit(address)
    //     - XIO: AND rung_result with ~read_bit(address)
    //     - OTE: Write rung_result to output_image[address[2:0]]
    //     - OTL: If rung_result, SET output_image[address[2:0]]
    //     - OTU: If rung_result, CLEAR output_image[address[2:0]]
    //     - END: Move to next rung (reset rung_result to 1) or finish
    //   SCAN_WRITE:   Copy output_image to `outputs`, return to SCAN_READ.
    //
    // HINT: rung_result starts as 1 (true). Contacts AND/OR it down.
    //       When you encounter an END, check if the NEXT instruction is
    //       also END (= end of program, go to SCAN_WRITE) or not
    //       (= end of rung, reset rung_result and continue).

    always @(posedge clk) begin
        if (rst) begin
            scan_state     <= SCAN_READ;
            pc             <= 6'd0;
            current_rung   <= 4'd0;
            rung_result    <= 1'b1;
            input_image    <= 8'd0;
            output_image   <= 8'd0;
            outputs        <= 8'd0;
            scan_heartbeat <= 1'b0;
        end else if (run) begin
            case (scan_state)
                // ---- Phase 1: Read Inputs ----
                SCAN_READ: begin
                    // TODO: Latch physical inputs into input_image
                    input_image <= 8'd0;   // <-- replace with `inputs`
                    pc          <= 6'd0;
                    current_rung <= 4'd0;
                    rung_result <= 1'b1;
                    scan_state  <= SCAN_EXECUTE;
                end

                // ---- Phase 2: Execute Program ----
                SCAN_EXECUTE: begin
                    current_instr <= prog_mem[pc];
                    case (opcode)
                        OP_NOP: begin
                            // No operation, advance PC
                            pc <= pc + 1'b1;
                        end

                        OP_XIC: begin
                            // TODO: AND rung_result with the addressed bit
                            // rung_result <= rung_result & read_bit(address);
                            pc <= pc + 1'b1;
                        end

                        OP_XIO: begin
                            // TODO: AND rung_result with NOT of addressed bit
                            // rung_result <= rung_result & ~read_bit(address);
                            pc <= pc + 1'b1;
                        end

                        OP_OTE: begin
                            // TODO: Write rung_result to output bit
                            // output_image[address[2:0]] <= rung_result;
                            pc <= pc + 1'b1;
                        end

                        OP_OTL: begin
                            // TODO: If rung_result, SET output bit (latch)
                            // if (rung_result)
                            //     output_image[address[2:0]] <= 1'b1;
                            pc <= pc + 1'b1;
                        end

                        OP_OTU: begin
                            // TODO: If rung_result, CLEAR output bit (unlatch)
                            // if (rung_result)
                            //     output_image[address[2:0]] <= 1'b0;
                            pc <= pc + 1'b1;
                        end

                        OP_TON: begin
                            // TODO (Exercise 2): Timer On-Delay
                            // If rung_result, increment timer_acc on timer_tick
                            // When timer_acc >= preset (operand * scale), set timer_dn
                            pc <= pc + 1'b1;
                        end

                        OP_TOF: begin
                            // TODO (Exercise 2): Timer Off-Delay
                            // When rung_result goes FALSE, start timing
                            // After preset expires, clear timer_dn
                            pc <= pc + 1'b1;
                        end

                        OP_CTU: begin
                            // TODO (Exercise 3): Counter Up
                            // On rising edge of rung_result, increment counter_acc
                            // When counter_acc >= preset, set counter_dn
                            pc <= pc + 1'b1;
                        end

                        OP_CTD: begin
                            // TODO (Exercise 3): Counter Down
                            // On rising edge of rung_result, decrement counter_acc
                            // When counter_acc <= 0, set counter_dn
                            pc <= pc + 1'b1;
                        end

                        OP_END: begin
                            // Check if this is end-of-program or end-of-rung
                            // TODO: If next instruction is also END or pc at limit,
                            //       transition to SCAN_WRITE. Otherwise, reset
                            //       rung_result for the next rung.
                            scan_state <= SCAN_WRITE;  // <-- simplistic: always finish
                        end

                        default: begin
                            pc <= pc + 1'b1;
                        end
                    endcase
                end

                // ---- Phase 3: Write Outputs ----
                SCAN_WRITE: begin
                    // TODO: Copy output_image to physical outputs
                    outputs <= 8'd0;   // <-- replace with output_image
                    scan_heartbeat <= ~scan_heartbeat;
                    scan_state <= SCAN_READ;
                end

                default: scan_state <= SCAN_READ;
            endcase
        end
    end

    // ================================================================
    // Exercise 2: Timer Instructions (TON / TOF)
    // ================================================================
    // TODO: Implement timer accumulation logic. For each of the 4 timers:
    //
    //   TON (Timer On-Delay):
    //     - While enable (rung input) is TRUE, increment accumulator on
    //       each timer_tick.
    //     - When accumulator >= preset, set timer_dn bit.
    //     - When enable goes FALSE, reset accumulator and clear timer_dn.
    //
    //   TOF (Timer Off-Delay):
    //     - While enable is TRUE, the timer_dn bit is set and accumulator
    //       is held at zero.
    //     - When enable goes FALSE, start accumulating.
    //     - When accumulator >= preset, clear timer_dn.
    //
    // The operand field of the TON/TOF instruction provides the preset
    // in units of 10 ms (1 = 10 ms, 100 = 1 second, 255 = 2.55 seconds).
    //
    // HINT: You need to track timer_en (current enable) vs timer_prev_en
    //       (previous scan's enable) for edge detection.

    // ================================================================
    // Exercise 3: Counter Instructions (CTU / CTD)
    // ================================================================
    // TODO: Implement counter logic. For each of the 4 counters:
    //
    //   CTU (Count Up):
    //     - Detect false-to-true transition of rung_result.
    //     - On each rising edge, increment counter_acc.
    //     - When counter_acc >= counter_pre, set counter_dn.
    //
    //   CTD (Count Down):
    //     - On each rising edge of rung_result, decrement counter_acc.
    //     - When counter_acc reaches 0 (or underflows), set counter_dn.
    //
    // HINT: Store the previous rung_result for each counter in
    //       counter_prev_rung to detect edges.

    // ================================================================
    // Exercise 4: Conveyor Belt Program
    // ================================================================
    // TODO: Modify the program memory initialisation to implement:
    //
    //   Rung 0: [START btn] AND [NOT E-STOP] AND [NOT JAM] -> [LATCH Motor]
    //   Rung 1: [STOP btn] OR [E-STOP] OR [JAM] -> [UNLATCH Motor]
    //   Rung 2: [Motor running] -> [TON T4:0, preset=30 (300ms)]
    //   Rung 3: [T4:0 DN] -> [Indicator Lamp]
    //   Rung 4: [JAM sensor] -> [CTU C5:0, preset=3]
    //   Rung 5: [C5:0 DN] -> [Alarm LED] (3 jams = alarm)
    //
    // Button mapping:
    //   buttons[0] = START
    //   buttons[1] = E-STOP (normally closed -- XIO in normal operation)
    //   buttons[2] = STOP
    //   buttons[3] = JAM sensor (simulated)

    // ================================================================
    // Exercise 5 (Challenge): Analog Comparison and Sequencer
    // ================================================================
    // TODO: Add opcodes for:
    //   OP_GRT (4'hA): Compare input_image against operand, rung_result
    //                  AND= (input_image[address] > operand)
    //   OP_LES (4'hB): rung_result AND= (input_image[address] < operand)
    //   OP_EQU (4'hC): rung_result AND= (input_image[address] == operand)
    //   OP_SQO (4'hD): Sequencer Output. On rising edge of rung_result,
    //                  advance a step pointer through a table of 8-bit
    //                  patterns and write the current pattern to output_image.
    //
    // The sequencer table can be stored in a separate memory array.
    // This exercise is left entirely for you to implement.

    // ================================================================
    // Timer/Counter Initialisation
    // ================================================================
    integer t;
    always @(posedge clk) begin
        if (rst) begin
            timer_dn   <= 4'd0;
            timer_en   <= 4'd0;
            timer_prev_en <= 4'd0;
            counter_dn <= 4'd0;
            for (t = 0; t < 4; t = t + 1) begin
                timer_acc[t]   <= 24'd0;
                timer_pre[t]   <= 24'd0;
                counter_acc[t] <= 16'd0;
                counter_pre[t] <= 16'd0;
                counter_prev_rung[t] <= 1'b0;
            end
        end
    end

endmodule
