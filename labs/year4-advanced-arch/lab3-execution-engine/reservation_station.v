/*
 * Lab 3: Execution Engine - Reservation Stations
 * ================================================
 * Reservation stations hold instructions waiting for their
 * operands. When all operands are ready, the instruction "issues"
 * to an execution unit.
 *
 * This is Tomasulo's algorithm - the breakthrough that enabled
 * out-of-order execution in the IBM System/360 Model 91 (1967).
 *
 * FLOW:
 *   Dispatch -> Reservation Station -> Issue -> Execute -> Complete
 *                    |                           |
 *               Wait for operands          Result broadcast
 *               (snoop result bus)         (CDB: Common Data Bus)
 *
 * EXERCISES:
 *
 * Exercise 1: Implement RS allocation
 *   When an instruction is dispatched, allocate an RS entry.
 *   Record the operation, destination, and source operand tags.
 *   If a source is already ready, store its value directly.
 *
 * Exercise 2: Implement operand snooping (CDB wakeup)
 *   When a result appears on the Common Data Bus (CDB), check
 *   all RS entries. If any entry is waiting for that tag, capture
 *   the value and mark the operand as ready.
 *
 * Exercise 3: Implement issue logic
 *   An instruction can issue when ALL its operands are ready.
 *   If multiple instructions are ready, use oldest-first policy.
 *
 * Exercise 4 (challenge): Multiple execution units
 *   Add a second ALU and implement issue to both. Handle the
 *   case where both produce results on the same cycle.
 *
 * CONCEPTS:
 *   - Tomasulo's algorithm
 *   - Tag-based operand tracking
 *   - Common Data Bus (result broadcast)
 *   - Dynamic scheduling
 *   - Issue width vs execution unit count
 */

`default_nettype none

`include "../common/ooo_defs.vh"

module reservation_station (
	input  wire        clk,
	input  wire        rst,

	// Dispatch interface (load new instruction)
	input  wire        dispatch_valid,
	input  wire [3:0]  dispatch_alu_op,
	input  wire [`PHYS_REG_BITS-1:0] dispatch_phys_rd,
	input  wire [`ROB_IDX_BITS-1:0]  dispatch_rob_idx,

	// Source 1
	input  wire [`PHYS_REG_BITS-1:0] dispatch_src1_tag,
	input  wire [31:0]               dispatch_src1_val,
	input  wire                      dispatch_src1_ready,

	// Source 2
	input  wire [`PHYS_REG_BITS-1:0] dispatch_src2_tag,
	input  wire [31:0]               dispatch_src2_val,
	input  wire                      dispatch_src2_ready,

	output wire        dispatch_stall,  // RS is full

	// Issue interface (to execution unit)
	output reg         issue_valid,
	output reg  [3:0]  issue_alu_op,
	output reg  [31:0] issue_src1_val,
	output reg  [31:0] issue_src2_val,
	output reg  [`PHYS_REG_BITS-1:0] issue_phys_rd,
	output reg  [`ROB_IDX_BITS-1:0]  issue_rob_idx,

	// Common Data Bus (snoop for wakeup)
	input  wire        cdb_valid,
	input  wire [`PHYS_REG_BITS-1:0] cdb_tag,
	input  wire [31:0] cdb_value,

	// Flush
	input  wire        flush
);

	// RS entry storage
	reg        rs_valid     [0:`RS_SIZE-1];
	reg [3:0]  rs_alu_op    [0:`RS_SIZE-1];
	reg [`PHYS_REG_BITS-1:0] rs_phys_rd [0:`RS_SIZE-1];
	reg [`ROB_IDX_BITS-1:0]  rs_rob_idx [0:`RS_SIZE-1];

	// Source 1
	reg [`PHYS_REG_BITS-1:0] rs_src1_tag   [0:`RS_SIZE-1];
	reg [31:0]               rs_src1_val   [0:`RS_SIZE-1];
	reg                      rs_src1_ready [0:`RS_SIZE-1];

	// Source 2
	reg [`PHYS_REG_BITS-1:0] rs_src2_tag   [0:`RS_SIZE-1];
	reg [31:0]               rs_src2_val   [0:`RS_SIZE-1];
	reg                      rs_src2_ready [0:`RS_SIZE-1];

	// Find free slot
	reg [`RS_IDX_BITS-1:0] free_slot;
	reg has_free;

	// Find ready-to-issue slot (oldest first)
	reg [`RS_IDX_BITS-1:0] issue_slot;
	reg has_ready;

	integer i;

	// Combinational: find free slot and ready slot
	always @(*) begin
		has_free  = 0;
		free_slot = 0;
		has_ready = 0;
		issue_slot = 0;

		// Find first free slot
		for (i = `RS_SIZE - 1; i >= 0; i = i - 1) begin
			if (!rs_valid[i]) begin
				has_free  = 1;
				free_slot = i;
			end
		end

		// Find first ready slot (both operands ready)
		for (i = `RS_SIZE - 1; i >= 0; i = i - 1) begin
			if (rs_valid[i] && rs_src1_ready[i] && rs_src2_ready[i]) begin
				has_ready  = 1;
				issue_slot = i;
			end
		end
	end

	assign dispatch_stall = !has_free;

	always @(posedge clk) begin
		if (rst || flush) begin
			issue_valid <= 0;
			for (i = 0; i < `RS_SIZE; i = i + 1)
				rs_valid[i] <= 0;
		end else begin
			issue_valid <= 0;

			// === CDB Snoop: wake up waiting operands ===
			if (cdb_valid) begin
				for (i = 0; i < `RS_SIZE; i = i + 1) begin
					if (rs_valid[i]) begin
						if (!rs_src1_ready[i] && rs_src1_tag[i] == cdb_tag) begin
							rs_src1_ready[i] <= 1;
							rs_src1_val[i]   <= cdb_value;
						end
						if (!rs_src2_ready[i] && rs_src2_tag[i] == cdb_tag) begin
							rs_src2_ready[i] <= 1;
							rs_src2_val[i]   <= cdb_value;
						end
					end
				end
			end

			// === Dispatch: allocate new entry ===
			if (dispatch_valid && has_free) begin
				rs_valid[free_slot]      <= 1;
				rs_alu_op[free_slot]     <= dispatch_alu_op;
				rs_phys_rd[free_slot]    <= dispatch_phys_rd;
				rs_rob_idx[free_slot]    <= dispatch_rob_idx;

				// Source 1: check CDB bypass (result available THIS cycle)
				if (dispatch_src1_ready) begin
					rs_src1_ready[free_slot] <= 1;
					rs_src1_val[free_slot]   <= dispatch_src1_val;
				end else if (cdb_valid && cdb_tag == dispatch_src1_tag) begin
					rs_src1_ready[free_slot] <= 1;
					rs_src1_val[free_slot]   <= cdb_value;
				end else begin
					rs_src1_ready[free_slot] <= 0;
					rs_src1_tag[free_slot]   <= dispatch_src1_tag;
				end

				// Source 2: same logic
				if (dispatch_src2_ready) begin
					rs_src2_ready[free_slot] <= 1;
					rs_src2_val[free_slot]   <= dispatch_src2_val;
				end else if (cdb_valid && cdb_tag == dispatch_src2_tag) begin
					rs_src2_ready[free_slot] <= 1;
					rs_src2_val[free_slot]   <= cdb_value;
				end else begin
					rs_src2_ready[free_slot] <= 0;
					rs_src2_tag[free_slot]   <= dispatch_src2_tag;
				end
			end

			// === Issue: send ready instruction to execution ===
			if (has_ready) begin
				issue_valid    <= 1;
				issue_alu_op   <= rs_alu_op[issue_slot];
				issue_src1_val <= rs_src1_val[issue_slot];
				issue_src2_val <= rs_src2_val[issue_slot];
				issue_phys_rd  <= rs_phys_rd[issue_slot];
				issue_rob_idx  <= rs_rob_idx[issue_slot];

				// Deallocate RS entry
				rs_valid[issue_slot] <= 0;
			end
		end
	end

endmodule
