/*
 * Lab 1: Out-of-Order Foundations - Reorder Buffer
 * =================================================
 * The reorder buffer (ROB) is the key structure that enables
 * out-of-order execution while maintaining the illusion of
 * in-order completion.
 *
 * The ROB:
 * - Allocates an entry for each dispatched instruction (in order)
 * - Tracks whether each instruction has completed execution
 * - Commits (retires) instructions in program order
 * - Enables precise exceptions and branch misprediction recovery
 *
 * ARCHITECTURE OVERVIEW:
 *   Fetch -> Decode -> DISPATCH -> Issue -> Execute -> Complete -> COMMIT
 *                        |                                          |
 *                   ROB allocate                               ROB retire
 *                   (in order)                                (in order)
 *               [execution can happen out of order in between]
 *
 * EXERCISES:
 *
 * Exercise 1: Implement ROB allocation
 *   When an instruction is dispatched, allocate the next ROB entry.
 *   Record the destination register and instruction type.
 *
 * Exercise 2: Implement ROB completion
 *   When an execution unit produces a result, mark the ROB entry
 *   as complete and store the result value.
 *
 * Exercise 3: Implement in-order commit
 *   The head of the ROB retires when its entry is complete.
 *   Write the result to the architectural register file.
 *
 * Exercise 4: Implement flush on misprediction
 *   When a branch misprediction is detected, flush all ROB entries
 *   after the mispredicted branch. Restore the rename table.
 *
 * CONCEPTS:
 *   - Circular buffer (head/tail pointers)
 *   - In-order dispatch, out-of-order execute, in-order commit
 *   - Precise exceptions
 *   - Speculative execution and recovery
 */

`default_nettype none

`include "../common/ooo_defs.vh"

module reorder_buffer (
	input  wire        clk,
	input  wire        rst,

	// Dispatch interface (allocate new entry)
	input  wire        dispatch_valid,
	input  wire [4:0]  dispatch_arch_rd,     // Architectural destination register
	input  wire [`PHYS_REG_BITS-1:0] dispatch_phys_rd, // Physical dest register
	input  wire [`PHYS_REG_BITS-1:0] dispatch_old_phys, // Previous physical mapping (for recovery)
	input  wire [6:0]  dispatch_opcode,
	input  wire [31:0] dispatch_pc,
	output wire [`ROB_IDX_BITS-1:0] dispatch_rob_idx,  // Allocated ROB index
	output wire        dispatch_ready,       // ROB has space

	// Complete interface (execution finished)
	input  wire        complete_valid,
	input  wire [`ROB_IDX_BITS-1:0] complete_rob_idx,
	input  wire [31:0] complete_value,
	input  wire        complete_exception,   // Execution caused exception

	// Commit interface (retire in order)
	output reg         commit_valid,
	output reg  [4:0]  commit_arch_rd,
	output reg  [`PHYS_REG_BITS-1:0] commit_phys_rd,
	output reg  [`PHYS_REG_BITS-1:0] commit_old_phys,
	output reg  [31:0] commit_value,
	output reg         commit_exception,

	// Flush interface (branch misprediction)
	input  wire        flush,
	input  wire [`ROB_IDX_BITS-1:0] flush_rob_idx,  // Flush entries after this

	// Status
	output wire        rob_empty,
	output wire        rob_full,
	output wire [`ROB_IDX_BITS-1:0] rob_head,
	output wire [`ROB_IDX_BITS-1:0] rob_tail
);

	// ROB entry fields
	reg        rob_valid     [0:`ROB_SIZE-1];
	reg        rob_complete  [0:`ROB_SIZE-1];
	reg [4:0]  rob_arch_rd   [0:`ROB_SIZE-1];
	reg [`PHYS_REG_BITS-1:0] rob_phys_rd [0:`ROB_SIZE-1];
	reg [`PHYS_REG_BITS-1:0] rob_old_phys [0:`ROB_SIZE-1];
	reg [6:0]  rob_opcode   [0:`ROB_SIZE-1];
	reg [31:0] rob_pc       [0:`ROB_SIZE-1];
	reg [31:0] rob_value    [0:`ROB_SIZE-1];
	reg        rob_exception [0:`ROB_SIZE-1];

	// Head and tail pointers
	reg [`ROB_IDX_BITS-1:0] head;
	reg [`ROB_IDX_BITS-1:0] tail;
	reg [`ROB_IDX_BITS:0]   count;  // Extra bit for full/empty distinction

	assign rob_head  = head;
	assign rob_tail  = tail;
	assign rob_empty = (count == 0);
	assign rob_full  = (count == `ROB_SIZE);

	assign dispatch_ready   = !rob_full;
	assign dispatch_rob_idx = tail;

	integer i;
	always @(posedge clk) begin
		if (rst) begin
			head  <= 0;
			tail  <= 0;
			count <= 0;
			commit_valid <= 0;
			for (i = 0; i < `ROB_SIZE; i = i + 1) begin
				rob_valid[i]     <= 0;
				rob_complete[i]  <= 0;
				rob_exception[i] <= 0;
			end
		end else if (flush) begin
			// Flush: invalidate entries from flush_rob_idx+1 to tail
			// Reset tail to flush_rob_idx+1
			tail <= flush_rob_idx + 1;
			// Recalculate count
			if (flush_rob_idx >= head)
				count <= flush_rob_idx - head + 1;
			else
				count <= `ROB_SIZE - head + flush_rob_idx + 1;

			// Invalidate flushed entries
			for (i = 0; i < `ROB_SIZE; i = i + 1) begin
				// Check if entry i is "after" flush_rob_idx
				// (This is simplified - proper circular buffer logic needed)
				if (rob_valid[i]) begin
					// Keep entries from head to flush_rob_idx, invalidate rest
				end
			end

			commit_valid <= 0;
		end else begin
			commit_valid <= 0;

			// === ALLOCATE: add entry at tail ===
			if (dispatch_valid && !rob_full) begin
				rob_valid[tail]     <= 1;
				rob_complete[tail]  <= 0;
				rob_arch_rd[tail]   <= dispatch_arch_rd;
				rob_phys_rd[tail]   <= dispatch_phys_rd;
				rob_old_phys[tail]  <= dispatch_old_phys;
				rob_opcode[tail]    <= dispatch_opcode;
				rob_pc[tail]        <= dispatch_pc;
				rob_exception[tail] <= 0;
				tail  <= tail + 1;
				count <= count + 1;
			end

			// === COMPLETE: mark entry as done ===
			if (complete_valid) begin
				rob_complete[complete_rob_idx]  <= 1;
				rob_value[complete_rob_idx]     <= complete_value;
				rob_exception[complete_rob_idx] <= complete_exception;
			end

			// === COMMIT: retire from head (in order) ===
			if (!rob_empty && rob_valid[head] && rob_complete[head]) begin
				commit_valid     <= 1;
				commit_arch_rd   <= rob_arch_rd[head];
				commit_phys_rd   <= rob_phys_rd[head];
				commit_old_phys  <= rob_old_phys[head];
				commit_value     <= rob_value[head];
				commit_exception <= rob_exception[head];

				rob_valid[head] <= 0;
				head  <= head + 1;
				count <= count - 1;

				// Adjust count if allocate happens simultaneously
				if (dispatch_valid && !rob_full)
					count <= count;  // +1 -1 = net zero change
			end
		end
	end

endmodule
