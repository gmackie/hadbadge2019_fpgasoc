/*
 * Lab 5: SPI Controller Design
 * ============================
 * In this lab you will build an SPI (Serial Peripheral Interface) master
 * controller from scratch in Verilog, targeting the ECP5 FPGA badge running
 * at 48 MHz. SPI is one of the most common serial protocols used to
 * communicate with sensors, displays, SD cards, and other peripherals. You
 * will implement clock generation, data shifting, and chip-select management
 * using a simple finite state machine. By the end of this lab you will have
 * a fully functional SPI master capable of full-duplex 8-bit transfers.
 *
 * EXERCISES:
 * Exercise 1: Implement the SCK clock divider. Use `clk_div` to divide the
 *             48 MHz system clock down to the desired SPI clock frequency.
 *             (PROVIDED as an example below.)
 * Exercise 2: Implement the MOSI shift register. On each SCK falling edge,
 *             shift the next bit of `tx_data` onto MOSI (MSB first).
 * Exercise 3: Implement the MISO sampling logic. On each SCK rising edge,
 *             shift the sampled MISO bit into the receive shift register.
 * Exercise 4: Add chip-select (CS) logic with configurable active polarity.
 *             CS should assert before the first SCK edge and deassert after
 *             the last bit with a half-clock hold time.
 * Exercise 5 (Challenge): Add multi-byte transfer support with automatic
 *             CS management. Accept a byte count and keep CS asserted while
 *             streaming successive bytes.
 *
 * CONCEPTS:
 *   - Serial communication protocols (SPI modes 0-3)
 *   - Shift registers for serialisation / deserialisation
 *   - Clock domain awareness and clock dividers
 *   - Full-duplex communication (simultaneous TX and RX)
 *   - Timing diagrams and setup/hold requirements
 *
 * LED MAPPING:
 *   LED[0] = SCK output (directly mirrors sck)
 *   LED[1] = MOSI output
 *   LED[2] = CS_N active (directly mirrors cs_n)
 *   LED[3] = busy flag
 *   LED[4] = done flag (directly mirrors done)
 *   LED[5] = bit_counter[0]
 *   LED[6] = bit_counter[1]
 *   LED[7] = bit_counter[2]
 */

`default_nettype none

module spi_controller (
    input  wire        clk,        // 48 MHz system clock
    input  wire        rst,        // Synchronous reset, active high
    // SPI bus signals
    output reg         sck,        // SPI clock output
    output reg         mosi,       // Master-Out Slave-In
    input  wire        miso,       // Master-In Slave-Out
    output reg         cs_n,       // Chip select, directly active-low
    // Parallel data interface
    input  wire [7:0]  tx_data,    // Byte to transmit
    output reg  [7:0]  rx_data,    // Byte received
    // Control / status
    input  wire        start,      // Pulse high for one cycle to begin
    output reg         busy,       // High while transfer in progress
    output reg         done,       // Pulses high for one cycle when complete
    // Configuration
    input  wire [7:0]  clk_div     // SCK half-period in system-clock cycles
);

    // ----------------------------------------------------------------
    // FSM state encoding
    // ----------------------------------------------------------------
    localparam STATE_IDLE     = 2'b00;
    localparam STATE_TRANSFER = 2'b01;
    localparam STATE_DONE     = 2'b10;

    reg [1:0] state, state_next;

    // ----------------------------------------------------------------
    // Internal registers
    // ----------------------------------------------------------------
    reg [7:0] clk_counter;     // Counts system clocks for SCK generation
    reg [7:0] shift_out;       // TX shift register (MSB shifted out first)
    reg [7:0] shift_in;        // RX shift register (MSB shifted in first)
    reg [2:0] bit_counter;     // Counts 0..7 for the 8 bits in a transfer
    reg       sck_internal;    // Internal SCK level (directly driven to sck)
    reg       sck_rising;      // Pulses high for one clk when SCK rises
    reg       sck_falling;     // Pulses high for one clk when SCK falls

    // ================================================================
    // Exercise 1 (EXAMPLE): SCK Clock Divider
    // ================================================================
    // The clock divider counts system clock cycles and toggles `sck_internal`
    // every `clk_div` cycles, producing a symmetric SPI clock.  Two helper
    // signals `sck_rising` and `sck_falling` are generated so the rest of
    // the design can react on the correct edges without worrying about
    // glitches.
    //
    // Example: clk_div = 8'd24 gives SCK period = 48 system clocks = 1 MHz.

    always @(posedge clk) begin
        if (rst || state == STATE_IDLE) begin
            clk_counter  <= 8'd0;
            sck_internal <= 1'b0;
            sck_rising   <= 1'b0;
            sck_falling  <= 1'b0;
        end else if (state == STATE_TRANSFER) begin
            sck_rising  <= 1'b0;
            sck_falling <= 1'b0;
            if (clk_counter == clk_div - 1'b1) begin
                clk_counter  <= 8'd0;
                sck_internal <= ~sck_internal;
                sck_rising   <= ~sck_internal;   // was low, now goes high
                sck_falling  <=  sck_internal;   // was high, now goes low
            end else begin
                clk_counter <= clk_counter + 1'b1;
            end
        end
    end

    // Drive the external SCK pin from the internal signal
    always @(posedge clk) begin
        sck <= sck_internal;
    end

    // ================================================================
    // Exercise 2: MOSI Shift Register (MSB first)
    // ================================================================
    // TODO: On the `start` pulse, load `tx_data` into `shift_out`.
    //       On every SCK *falling* edge (use `sck_falling`), shift
    //       `shift_out` left by one bit and drive `mosi` from the MSB.
    //
    // Hint: mosi should update on the falling edge of SCK so that it is
    //       stable well before the slave samples on the rising edge.
    //
    // Replace the placeholder below with your implementation.

    always @(posedge clk) begin
        if (rst) begin
            shift_out <= 8'd0;
            mosi      <= 1'b0;
        end else if (state == STATE_IDLE && start) begin
            // TODO: Load tx_data into shift_out and drive MSB onto mosi
            shift_out <= 8'd0;  // <-- replace with tx_data
            mosi      <= 1'b0;  // <-- replace with tx_data[7]
        end else if (state == STATE_TRANSFER && sck_falling) begin
            // TODO: Shift shift_out left by 1, drive new MSB onto mosi
            shift_out <= shift_out;  // <-- replace with {shift_out[6:0], 1'b0}
            mosi      <= 1'b0;      // <-- replace with shift_out[6]
        end
    end

    // ================================================================
    // Exercise 3: MISO Sampling
    // ================================================================
    // TODO: On every SCK *rising* edge (use `sck_rising`), sample the
    //       `miso` input and shift it into `shift_in` (MSB first).
    //       When the transfer is complete, copy `shift_in` to `rx_data`.
    //
    // Hint: shift_in = {shift_in[6:0], miso}  (shift left, new bit at LSB)
    //
    // Replace the placeholder below with your implementation.

    always @(posedge clk) begin
        if (rst) begin
            shift_in <= 8'd0;
            rx_data  <= 8'd0;
        end else if (state == STATE_TRANSFER && sck_rising) begin
            // TODO: Shift miso into shift_in from the right
            shift_in <= shift_in;  // <-- replace with {shift_in[6:0], miso}
        end else if (state == STATE_DONE) begin
            // TODO: Latch the received byte into rx_data
            rx_data <= rx_data;    // <-- replace with shift_in
        end
    end

    // ================================================================
    // Exercise 4: Chip Select Logic
    // ================================================================
    // TODO: Assert cs_n LOW when a transfer begins (on `start`) and keep
    //       it low throughout STATE_TRANSFER. De-assert (HIGH) when the
    //       transfer reaches STATE_DONE after a half-clock hold time.
    //
    // Bonus: Add a `cs_polarity` input so the active level is configurable.
    //
    // Replace the placeholder below with your implementation.

    always @(posedge clk) begin
        if (rst) begin
            cs_n <= 1'b1;  // De-asserted at reset
        end else begin
            // TODO: Drive cs_n based on state
            cs_n <= 1'b1;  // <-- replace with proper CS management
        end
    end

    // ================================================================
    // Bit Counter
    // ================================================================
    // Counts the bits transferred.  Increments on each SCK rising edge
    // (after MISO is sampled).  When bit_counter reaches 7 (all 8 bits
    // done), the FSM transitions to DONE.

    always @(posedge clk) begin
        if (rst || state == STATE_IDLE) begin
            bit_counter <= 3'd0;
        end else if (state == STATE_TRANSFER && sck_rising) begin
            bit_counter <= bit_counter + 3'd1;
        end
    end

    // ================================================================
    // FSM: Next-State Logic
    // ================================================================
    always @(*) begin
        state_next = state;
        case (state)
            STATE_IDLE: begin
                if (start)
                    state_next = STATE_TRANSFER;
            end
            STATE_TRANSFER: begin
                // After 8 rising edges (bit_counter wraps from 7 to 0)
                if (sck_rising && bit_counter == 3'd7)
                    state_next = STATE_DONE;
            end
            STATE_DONE: begin
                state_next = STATE_IDLE;
            end
            default: state_next = STATE_IDLE;
        endcase
    end

    // ================================================================
    // FSM: State Register and Outputs
    // ================================================================
    always @(posedge clk) begin
        if (rst) begin
            state <= STATE_IDLE;
            busy  <= 1'b0;
            done  <= 1'b0;
        end else begin
            state <= state_next;
            case (state_next)
                STATE_IDLE: begin
                    busy <= 1'b0;
                    done <= 1'b0;
                end
                STATE_TRANSFER: begin
                    busy <= 1'b1;
                    done <= 1'b0;
                end
                STATE_DONE: begin
                    busy <= 1'b0;
                    done <= 1'b1;
                end
                default: begin
                    busy <= 1'b0;
                    done <= 1'b0;
                end
            endcase
        end
    end

    // ================================================================
    // Exercise 5 (Challenge): Multi-Byte Transfer
    // ================================================================
    // TODO: Add a `byte_count[7:0]` input and a byte counter.  When
    //       `start` is pulsed, begin transferring `byte_count` bytes
    //       consecutively.  Keep CS asserted for the entire burst.
    //       After each byte, pulse a `byte_done` signal so the
    //       host can load the next tx_data.
    //
    //       Suggested additions:
    //         - input  wire [7:0] byte_count
    //         - output reg        byte_done
    //         - reg    [7:0]      bytes_remaining
    //         - New FSM state: STATE_BYTE_GAP (short pause between bytes)
    //
    //       This exercise is left entirely for you to implement.

endmodule
