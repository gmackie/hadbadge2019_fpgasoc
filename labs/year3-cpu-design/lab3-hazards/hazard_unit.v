/*
 * Lab 3: Pipeline Hazards
 * =======================
 * The pipelined CPU from Lab 2 produces wrong results when
 * instructions depend on each other! This lab fixes that.
 *
 * THREE TYPES OF HAZARDS:
 *
 * 1. DATA HAZARDS: Instruction reads a register that a prior
 *    instruction hasn't written back yet.
 *      addi x1, x0, 5     # WB writes x1 in cycle 5
 *      addi x2, x1, 3     # ID reads x1 in cycle 3 - STALE!
 *
 * 2. CONTROL HAZARDS: Branch target isn't known until EX stage,
 *    but we've already fetched 2 wrong instructions.
 *      beq x1, x2, target  # Resolved in EX (cycle 3)
 *      addi x3, x0, 1      # Already in ID - WRONG if taken!
 *      addi x4, x0, 2      # Already in IF - WRONG if taken!
 *
 * 3. STRUCTURAL HAZARDS: Two stages need the same resource.
 *    (Less common in RISC-V; our design avoids most of these.)
 *
 * EXERCISES:
 *
 * Exercise 1: Implement forwarding (bypassing)
 *   If the EX/MEM or MEM/WB pipeline register contains a result
 *   that the ID/EX stage needs, forward it directly instead of
 *   waiting for writeback. Complete the forwarding unit below.
 *
 * Exercise 2: Implement load-use stall
 *   Forwarding can't help when a load is immediately followed by
 *   a dependent instruction (the data isn't available until MEM).
 *   Insert a pipeline bubble (stall IF and ID, flush EX).
 *
 * Exercise 3: Handle control hazards
 *   Flush the pipeline (insert NOPs) when a branch is taken.
 *   Count the branch penalty (cycles wasted on wrong path).
 *
 * Exercise 4 (challenge): Branch prediction
 *   Implement a simple "predict not taken" scheme: always fetch
 *   the next sequential instruction, but flush if wrong.
 *   Measure the prediction accuracy on test programs.
 *
 * CONCEPTS:
 *   - Forwarding / bypassing
 *   - Pipeline stalls and bubbles
 *   - Branch penalty
 *   - CPI impact: ideal=1.0, with hazards it increases
 */

`default_nettype none

module hazard_unit (
	// EX stage source registers
	input  wire [4:0]  idex_rs1,
	input  wire [4:0]  idex_rs2,

	// EX/MEM destination
	input  wire [4:0]  exmem_rd,
	input  wire        exmem_reg_write,
	input  wire        exmem_mem_read,

	// MEM/WB destination
	input  wire [4:0]  memwb_rd,
	input  wire        memwb_reg_write,

	// Forwarding control (to muxes before ALU inputs)
	// 00 = no forwarding (use register file)
	// 01 = forward from EX/MEM
	// 10 = forward from MEM/WB
	output reg  [1:0]  forward_a,
	output reg  [1:0]  forward_b,

	// Stall and flush control
	output wire        stall_if,      // Freeze IF stage (hold PC)
	output wire        stall_id,      // Freeze ID stage (hold IF/ID reg)
	output wire        flush_ex       // Insert bubble in EX stage
);

	// ===== Forwarding Logic =====
	// TODO: Complete the forwarding conditions

	always @(*) begin
		// Default: no forwarding
		forward_a = 2'b00;
		forward_b = 2'b00;

		// Forward from EX/MEM (highest priority - most recent result)
		if (exmem_reg_write && exmem_rd != 0) begin
			if (exmem_rd == idex_rs1)
				forward_a = 2'b01;
			if (exmem_rd == idex_rs2)
				forward_b = 2'b01;
		end

		// Forward from MEM/WB (lower priority)
		if (memwb_reg_write && memwb_rd != 0) begin
			// Only forward if EX/MEM isn't already forwarding
			if (memwb_rd == idex_rs1 && !(exmem_reg_write && exmem_rd == idex_rs1))
				forward_a = 2'b10;
			if (memwb_rd == idex_rs2 && !(exmem_reg_write && exmem_rd == idex_rs2))
				forward_b = 2'b10;
		end
	end

	// ===== Load-Use Hazard Detection =====
	// A load in EX/MEM followed by a dependent instruction in ID/EX
	// requires a 1-cycle stall because the load data isn't available
	// until the END of the MEM stage.
	wire load_use_hazard;

	// TODO: Detect load-use hazard
	assign load_use_hazard = exmem_mem_read && exmem_rd != 0 &&
	                         (exmem_rd == idex_rs1 || exmem_rd == idex_rs2);

	assign stall_if = load_use_hazard;
	assign stall_id = load_use_hazard;
	assign flush_ex = load_use_hazard;

endmodule
