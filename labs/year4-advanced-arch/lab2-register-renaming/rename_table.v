/*
 * Lab 2: Register Renaming
 * ========================
 * Register renaming eliminates false dependencies (WAR, WAW)
 * by mapping architectural registers to a larger set of physical registers.
 *
 * Example of a false dependency:
 *   add  x1, x2, x3   # Writes x1
 *   sub  x4, x1, x5   # Reads x1 (TRUE dependency - RAW)
 *   add  x1, x6, x7   # Writes x1 (FALSE dependency - WAW with line 1)
 *   mul  x8, x1, x9   # Reads x1 from line 3, not line 1
 *
 * After renaming:
 *   add  p10, p2, p3   # x1 -> p10
 *   sub  p11, p10, p5  # x4 -> p11, reads p10 (forwarded)
 *   add  p12, p6, p7   # x1 -> p12 (new physical reg!)
 *   mul  p13, p12, p9  # x8 -> p13, reads p12
 *
 * Now lines 1 and 3 can execute in parallel - no conflict!
 *
 * EXERCISES:
 *
 * Exercise 1: Implement the rename table (RAT)
 *   The RAT maps each architectural register (x0-x31) to a
 *   physical register (p0-p63). Complete the lookup and update logic.
 *
 * Exercise 2: Implement the free list
 *   Physical registers not currently mapped are on the free list.
 *   Allocate from the free list during rename, return to it during commit.
 *
 * Exercise 3: Implement checkpoint/restore
 *   On branch misprediction, restore the RAT to its state at
 *   the branch point. This requires snapshotting the RAT.
 *
 * Exercise 4 (challenge): Multiple rename per cycle
 *   Support renaming 2 instructions per cycle. Handle the case
 *   where the second instruction reads a register that the first
 *   instruction renames.
 *
 * CONCEPTS:
 *   - WAR (Write After Read) and WAW (Write After Write) hazards
 *   - Physical vs architectural registers
 *   - Free list management
 *   - Speculative rename and recovery
 */

`default_nettype none

`include "../common/ooo_defs.vh"

module rename_table (
	input  wire        clk,
	input  wire        rst,

	// Rename request (from decode/dispatch)
	input  wire        rename_valid,
	input  wire [4:0]  rename_rs1,       // Source register 1
	input  wire [4:0]  rename_rs2,       // Source register 2
	input  wire [4:0]  rename_rd,        // Destination register
	input  wire        rename_rd_wen,    // Destination is written

	// Rename results
	output wire [`PHYS_REG_BITS-1:0] phys_rs1,   // Physical reg for rs1
	output wire [`PHYS_REG_BITS-1:0] phys_rs2,   // Physical reg for rs2
	output wire [`PHYS_REG_BITS-1:0] phys_rd,    // New physical reg for rd
	output wire [`PHYS_REG_BITS-1:0] phys_rd_old, // Previous mapping of rd (for ROB)
	output wire        rename_stall,               // Free list empty

	// Free list return (from commit stage)
	input  wire        free_valid,
	input  wire [`PHYS_REG_BITS-1:0] free_reg,

	// Checkpoint/restore for branch misprediction
	input  wire        checkpoint_save,
	input  wire        checkpoint_restore
);

	// Register Alias Table: maps arch reg -> phys reg
	reg [`PHYS_REG_BITS-1:0] rat [0:31];

	// Checkpoint copy for recovery
	reg [`PHYS_REG_BITS-1:0] rat_checkpoint [0:31];

	// Free list: circular buffer of available physical registers
	reg [`PHYS_REG_BITS-1:0] free_list [0:`NUM_PHYS_REGS-1];
	reg [`PHYS_REG_BITS:0]   free_head, free_tail;
	wire free_empty = (free_head == free_tail);

	// Source lookups are combinational (read current RAT)
	assign phys_rs1    = rat[rename_rs1];
	assign phys_rs2    = rat[rename_rs2];
	assign phys_rd_old = rat[rename_rd];

	// Allocate from free list
	assign phys_rd = free_list[free_head[`PHYS_REG_BITS-1:0]];
	assign rename_stall = rename_valid && rename_rd_wen && free_empty;

	integer i;
	always @(posedge clk) begin
		if (rst) begin
			// Initial mapping: arch reg i -> phys reg i
			for (i = 0; i < 32; i = i + 1) begin
				rat[i] <= i;
				rat_checkpoint[i] <= i;
			end

			// Free list starts with phys regs 32-63
			for (i = 0; i < `NUM_PHYS_REGS - 32; i = i + 1)
				free_list[i] <= i + 32;

			free_head <= 0;
			free_tail <= `NUM_PHYS_REGS - 32;

		end else if (checkpoint_restore) begin
			// Restore RAT from checkpoint
			for (i = 0; i < 32; i = i + 1)
				rat[i] <= rat_checkpoint[i];

		end else begin
			// Save checkpoint
			if (checkpoint_save) begin
				for (i = 0; i < 32; i = i + 1)
					rat_checkpoint[i] <= rat[i];
			end

			// Rename: update RAT and consume from free list
			if (rename_valid && rename_rd_wen && rename_rd != 0 && !free_empty) begin
				rat[rename_rd] <= phys_rd;
				free_head <= free_head + 1;
			end

			// Return freed register to free list (from commit)
			if (free_valid) begin
				free_list[free_tail[`PHYS_REG_BITS-1:0]] <= free_reg;
				free_tail <= free_tail + 1;
			end
		end
	end

endmodule
