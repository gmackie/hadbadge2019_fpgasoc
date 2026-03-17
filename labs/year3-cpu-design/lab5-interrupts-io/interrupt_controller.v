/*
 * Lab 5: Interrupt Controller and I/O Integration
 * =================================================
 * Build a RISC-V compatible interrupt controller inspired by the Platform-
 * Level Interrupt Controller (PLIC) and Core-Local Interruptor (CLINT).
 * The module accepts up to 32 external interrupt sources, provides priority
 * encoding to select the highest-priority pending interrupt, and implements
 * a claim/complete handshake so the CPU can service interrupts one at a
 * time.  A 64-bit machine-timer (mtime) with a 64-bit comparator
 * (mtimecmp) generates the standard RISC-V timer interrupt.
 *
 * EXERCISES:
 * Exercise 1: Implement interrupt pending/enable registers with priority
 *             encoding to select the highest-priority active interrupt.
 * Exercise 2: Implement the interrupt acknowledge (claim) and complete
 *             mechanism so the CPU can atomically accept and finish an
 *             interrupt.
 * Exercise 3: Add a 64-bit mtime counter and mtimecmp comparison to
 *             generate timer interrupts.
 * Exercise 4: Integrate a UART RX interrupt (directly wire irq_sources[1]
 *             as the UART data-available signal and verify it flows
 *             through the priority logic).
 * Exercise 5 (Challenge): Implement nested interrupts with configurable
 *             priority levels so a higher-priority interrupt can preempt
 *             a lower-priority handler.
 *
 * CONCEPTS:
 *   - Interrupt vs polling: trading CPU utilisation for latency
 *   - Interrupt vector table: mapping IRQ IDs to handler addresses
 *   - Interrupt latency: cycles from assertion to first handler insn
 *   - Priority arbitration: deterministic selection among simultaneous IRQs
 *   - Edge-triggered vs level-triggered interrupts
 *   - RISC-V privilege model: mstatus.MIE, mie, mip CSRs
 *   - Context save/restore on interrupt entry/exit
 */

`default_nettype none
`include "../common/riscv_defs.vh"

module interrupt_controller (
    input  wire        clk,
    input  wire        rst,

    // --- External interrupt sources (directly active-high level) ---
    input  wire [31:0] irq_sources,

    // --- Software-writable enable mask (active-high per source) ---
    input  wire [31:0] irq_enable,

    // --- Interrupt status to CPU ---
    output wire        irq_pending,       // At least one enabled IRQ active
    output wire [4:0]  irq_id,            // ID of highest-priority pending IRQ

    // --- Claim / complete handshake ---
    input  wire        irq_claim,         // CPU pulses to claim irq_id
    input  wire        irq_complete,      // CPU pulses when handler finishes

    // --- Timer interface ---
    input  wire        mtime_tick,        // Increment mtime (connect to 1 MHz)
    input  wire [63:0] mtimecmp,          // Software-writable compare value
    output wire        timer_irq          // mtime >= mtimecmp
);

    // ================================================================
    // Section 1 -- Pending / Enable Logic
    // ================================================================

    // The pending register latches any source that has fired but has not
    // yet been completed.  Sources are level-sensitive: as long as the
    // external line is high the pending bit remains set.
    reg [31:0] irq_pending_reg;

    // Effective pending = raw pending AND enable mask
    wire [31:0] irq_active;
    assign irq_active = irq_pending_reg & irq_enable;

    // ------------------------------------------------------------
    // TODO (Exercise 1): Update irq_pending_reg every cycle.
    //   - On reset, clear all bits.
    //   - Otherwise, OR in new irq_sources each cycle.
    //   - When irq_complete is pulsed, clear the bit for the
    //     currently-claimed IRQ ID (see Section 2).
    // Hint:  irq_pending_reg <= (irq_pending_reg | irq_sources) & ~clear_mask;
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            irq_pending_reg <= 32'b0;
        end else begin
            // TODO: Implement pending register update logic
            irq_pending_reg <= irq_pending_reg;  // placeholder
        end
    end

    // ================================================================
    // Section 2 -- Priority Encoder
    // ================================================================
    // Find the lowest-numbered (highest-priority) bit that is set in
    // irq_active.  IRQ 0 has the highest priority.

    reg [4:0]  highest_pri_id;
    reg        any_pending;

    // ------------------------------------------------------------
    // TODO (Exercise 1): Write a priority encoder.
    //   - Scan irq_active from bit 0 to bit 31.
    //   - Set highest_pri_id to the first set bit index.
    //   - Set any_pending if at least one bit is set.
    //
    // You may use a for-loop with a "found" flag, or use casez, or
    // use $clog2 tricks -- whichever you prefer.
    // ------------------------------------------------------------
    integer i;

    always @(*) begin
        highest_pri_id = 5'd0;
        any_pending    = 1'b0;

        // TODO: Priority encoder logic
        // for (i = 0; i < 32; i = i + 1) begin
        //     ...
        // end
    end

    assign irq_pending = any_pending;
    assign irq_id      = highest_pri_id;

    // ================================================================
    // Section 3 -- Claim / Complete State Machine
    // ================================================================
    // When the CPU asserts irq_claim, the controller latches the current
    // irq_id into claimed_id and masks that source from the priority
    // encoder until the CPU asserts irq_complete.

    localparam STATE_IDLE    = 2'd0;
    localparam STATE_CLAIMED = 2'd1;
    localparam STATE_WAIT    = 2'd2;  // extra state for nested IRQs

    reg [1:0] claim_state;
    reg [4:0] claimed_id;

    // ------------------------------------------------------------
    // TODO (Exercise 2): Implement the claim/complete FSM.
    //   STATE_IDLE:
    //     - If irq_claim && any_pending, latch irq_id into claimed_id,
    //       transition to STATE_CLAIMED.
    //   STATE_CLAIMED:
    //     - If irq_complete, clear the pending bit for claimed_id,
    //       transition back to STATE_IDLE.
    //   (Exercise 5 -- nested interrupts: use STATE_WAIT and a small
    //    stack to save preempted claimed_id values.)
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            claim_state <= STATE_IDLE;
            claimed_id  <= 5'd0;
        end else begin
            case (claim_state)
                STATE_IDLE: begin
                    // TODO: Implement claim logic
                end
                STATE_CLAIMED: begin
                    // TODO: Implement complete logic
                end
                default: begin
                    claim_state <= STATE_IDLE;
                end
            endcase
        end
    end

    // ================================================================
    // Section 4 -- 64-bit Machine Timer (mtime / mtimecmp)
    // ================================================================
    // The RISC-V spec defines a 64-bit real-time counter (mtime) and a
    // 64-bit compare register (mtimecmp).  When mtime >= mtimecmp the
    // timer interrupt is asserted and remains asserted until software
    // writes a new (future) value to mtimecmp.

    reg [63:0] mtime;

    // ------------------------------------------------------------
    // TODO (Exercise 3): Implement the mtime counter.
    //   - On reset, clear mtime to zero.
    //   - When mtime_tick is high, increment mtime by 1.
    // ------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            mtime <= 64'b0;
        end else begin
            // TODO: Increment mtime on tick
        end
    end

    // ------------------------------------------------------------
    // TODO (Exercise 3): Generate timer_irq.
    //   - timer_irq should be HIGH when mtime >= mtimecmp.
    //   - This is a level-sensitive signal; the CPU clears it by
    //     writing a future value into mtimecmp.
    // ------------------------------------------------------------
    assign timer_irq = 1'b0;  // TODO: Replace with comparison

    // ================================================================
    // Section 5 (Challenge) -- Nested Interrupt Support
    // ================================================================
    // For Exercise 5, add:
    //   - A small stack (depth 4) of claimed_id values so that a
    //     higher-priority interrupt can preempt the current handler.
    //   - A per-source priority register array (e.g. 3-bit priority).
    //   - Modified priority encoder that only considers sources whose
    //     priority exceeds the priority of the currently claimed IRQ.

    // TODO (Exercise 5): Declare priority registers and stack
    // reg [2:0] irq_priority [0:31];       // 8 priority levels per source
    // reg [4:0] nested_stack [0:3];        // stack of preempted IRQ IDs
    // reg [1:0] nested_sp;                 // stack pointer

    // ================================================================
    // Section 6 -- UART RX Interrupt Wiring (Exercise 4)
    // ================================================================
    // In a full SoC, irq_sources[1] would be connected to the UART RX
    // "data available" flag.  No extra logic is needed inside this
    // module -- the pending/enable/priority logic handles it generically.
    //
    // In your testbench, instantiate a UART model and connect:
    //   .irq_sources({30'b0, uart_rx_data_avail, 1'b0})
    //
    // Verify that irq_id == 5'd1 when the UART has data and the source
    // is enabled.

    // ================================================================
    // Debug / Verification Helpers
    // ================================================================
`ifdef FORMAL
    // If nothing is pending, irq_pending must be low
    always @(*) begin
        if (irq_active == 32'b0)
            assert(irq_pending == 1'b0);
    end

    // irq_id must be less than 32
    always @(*) begin
        assert(irq_id < 5'd32 || !irq_pending);
    end

    // claimed_id should only change during a claim event
    // (students can extend these properties)
`endif

endmodule
