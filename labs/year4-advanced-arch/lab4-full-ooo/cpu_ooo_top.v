/*
 * Lab 4: Full Out-of-Order CPU
 * ============================
 * The capstone: integrate all OoO components into a working processor.
 *
 * This is the complete pipeline:
 *
 *  Fetch -> Decode -> Rename -> Dispatch -> Issue -> Execute -> Complete -> Commit
 *                       |          |                    |                     |
 *                      RAT        ROB                  CDB               Retire
 *                    Free List    RS/LSQ             Broadcast         Free phys regs
 *
 * EXERCISES:
 *
 * Exercise 1: Connect the pipeline
 *   Wire together the ROB, rename table, and reservation stations
 *   from the previous labs. Start with ALU instructions only.
 *
 * Exercise 2: Add the CDB (Common Data Bus)
 *   When an execution unit completes, broadcast the result tag
 *   and value. The ROB and reservation stations both snoop this.
 *
 * Exercise 3: Add load/store support
 *   Loads and stores must maintain memory ordering. Implement a
 *   load/store queue that enforces this constraint.
 *
 * Exercise 4: Performance analysis
 *   Run benchmarks comparing:
 *   - Single-cycle CPU (Year 3 Lab 1)
 *   - Pipelined CPU (Year 3 Lab 2)
 *   - Out-of-order CPU (this lab)
 *   Measure IPC, stall cycles, and CDB utilization.
 *
 * Exercise 5 (challenge): Superscalar
 *   Extend to 2-wide dispatch: rename, dispatch, and issue
 *   two instructions per cycle. This requires dual-ported
 *   structures and careful dependency checking between the
 *   two instructions in the same cycle.
 *
 * This module provides the top-level skeleton that students
 * fill in by connecting their Year 4 lab modules.
 *
 * CONCEPTS:
 *   - Full OoO pipeline integration
 *   - Performance counters and analysis
 *   - IPC (Instructions Per Cycle) measurement
 *   - The complexity/performance tradeoff
 *   - Why modern CPUs are so complex
 */

`default_nettype none

`include "../common/ooo_defs.vh"

module cpu_ooo_top (
	input  wire        clk,
	input  wire        rst,

	// Instruction memory
	output wire [31:0] imem_addr,
	input  wire [31:0] imem_data,

	// Data memory
	output wire [31:0] dmem_addr,
	output wire [31:0] dmem_wdata,
	input  wire [31:0] dmem_rdata,
	output wire [3:0]  dmem_wstrb,
	output wire        dmem_ren,
	input  wire        dmem_ready,

	// Performance counters (directly to harness for display)
	output reg  [31:0] perf_cycles,
	output reg  [31:0] perf_instructions,
	output reg  [31:0] perf_stall_cycles,
	output reg  [31:0] perf_branch_mispred
);

	// ==========================================================
	// Front End: Fetch + Decode
	// ==========================================================
	reg  [31:0] pc;
	wire [31:0] pc_plus4 = pc + 32'd4;
	wire        fetch_stall;
	wire        redirect_valid;
	wire [31:0] redirect_pc;

	assign imem_addr = pc;

	always @(posedge clk) begin
		if (rst)
			pc <= 32'h40000000;
		else if (redirect_valid)
			pc <= redirect_pc;
		else if (!fetch_stall)
			pc <= pc_plus4;
	end

	// Decode: extract instruction fields
	wire [31:0] inst = imem_data;
	wire [6:0]  opcode = inst[6:0];
	wire [4:0]  rd     = inst[11:7];
	wire [2:0]  funct3 = inst[14:12];
	wire [4:0]  rs1    = inst[19:15];
	wire [4:0]  rs2    = inst[24:20];
	wire        rd_wen = (opcode == `OP_ALU  || opcode == `OP_ALUI ||
	                      opcode == `OP_LOAD || opcode == `OP_LUI  ||
	                      opcode == `OP_AUIPC|| opcode == `OP_JAL  ||
	                      opcode == `OP_JALR);

	// ==========================================================
	// Rename Stage
	// ==========================================================
	wire [`PHYS_REG_BITS-1:0] phys_rs1, phys_rs2, phys_rd, phys_rd_old;
	wire rename_stall;

	// TODO: Instantiate your rename_table from Lab 2
	// rename_table rat_inst ( ... );

	// Placeholder signals until connected
	assign phys_rs1    = rs1;
	assign phys_rs2    = rs2;
	assign phys_rd     = rd;
	assign phys_rd_old = rd;
	assign rename_stall = 0;

	// ==========================================================
	// Dispatch to ROB + Reservation Station
	// ==========================================================
	wire [`ROB_IDX_BITS-1:0] rob_idx;
	wire rob_ready;
	wire rs_stall;

	// TODO: Instantiate your reorder_buffer from Lab 1
	// reorder_buffer rob_inst ( ... );

	// TODO: Instantiate your reservation_station from Lab 3
	// reservation_station rs_inst ( ... );

	// Dispatch stall: if ROB full or RS full or free list empty
	assign fetch_stall = rename_stall || !rob_ready || rs_stall;

	// ==========================================================
	// Execute: ALU (reused from Year 2!)
	// ==========================================================
	wire        exec_valid;
	wire [3:0]  exec_alu_op;
	wire [31:0] exec_src1, exec_src2;
	wire [`PHYS_REG_BITS-1:0] exec_phys_rd;
	wire [`ROB_IDX_BITS-1:0]  exec_rob_idx;

	wire [31:0] exec_result;
	wire exec_alu_zero;

	// TODO: Instantiate ALU from Year 2 Lab 4
	// alu exec_alu ( ... );

	// ==========================================================
	// Common Data Bus (CDB)
	// ==========================================================
	wire        cdb_valid = exec_valid;  // From execution unit
	wire [`PHYS_REG_BITS-1:0] cdb_tag = exec_phys_rd;
	wire [31:0] cdb_value = exec_result;

	// CDB broadcasts to:
	// 1. Reservation stations (operand wakeup)
	// 2. ROB (mark complete)
	// 3. Physical register file (write result)

	// ==========================================================
	// Commit Stage
	// ==========================================================
	wire        commit_valid;
	wire [4:0]  commit_arch_rd;
	wire [31:0] commit_value;

	// Redirect on branch misprediction
	assign redirect_valid = 0;  // TODO: detect misprediction
	assign redirect_pc    = 0;

	// ==========================================================
	// Memory (simplified: no LSQ yet)
	// ==========================================================
	assign dmem_addr  = 0;
	assign dmem_wdata = 0;
	assign dmem_wstrb = 0;
	assign dmem_ren   = 0;

	// ==========================================================
	// Performance Counters
	// ==========================================================
	always @(posedge clk) begin
		if (rst) begin
			perf_cycles        <= 0;
			perf_instructions  <= 0;
			perf_stall_cycles  <= 0;
			perf_branch_mispred <= 0;
		end else begin
			perf_cycles <= perf_cycles + 1;

			if (commit_valid)
				perf_instructions <= perf_instructions + 1;

			if (fetch_stall)
				perf_stall_cycles <= perf_stall_cycles + 1;

			if (redirect_valid)
				perf_branch_mispred <= perf_branch_mispred + 1;
		end
	end

	// IPC can be calculated: perf_instructions / perf_cycles
	// Display via the harness or compute in software

endmodule
