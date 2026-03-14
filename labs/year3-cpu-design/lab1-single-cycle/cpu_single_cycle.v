/*
 * Lab 1: Single-Cycle RISC-V Processor
 * =====================================
 * Build a processor that executes one instruction per clock cycle.
 * This is the simplest possible CPU architecture - every instruction
 * completes in exactly one cycle.
 *
 * ARCHITECTURE:
 *   Instruction Memory -> Decode -> Register Read -> ALU -> Memory -> Writeback
 *   All in ONE clock cycle. The clock must be slow enough for the
 *   longest path (typically a load instruction) to complete.
 *
 * EXERCISES:
 *
 * Exercise 1: Instruction Decoder
 *   Complete the decoder that extracts fields from the 32-bit
 *   instruction word: opcode, rd, rs1, rs2, funct3, funct7, immediates.
 *
 * Exercise 2: ALU Integration
 *   Connect the ALU from Year 2 Lab 4. Map funct3/funct7 to ALU ops.
 *
 * Exercise 3: Implement R-type instructions
 *   ADD, SUB, AND, OR, XOR, SLT, SLL, SRL, SRA
 *
 * Exercise 4: Implement I-type instructions
 *   ADDI, ANDI, ORI, XORI, SLTI, LUI, AUIPC, loads
 *
 * Exercise 5: Implement branches and jumps
 *   BEQ, BNE, BLT, BGE, JAL, JALR
 *   This requires modifying the PC update logic.
 *
 * Exercise 6: Implement stores and loads
 *   SW, SH, SB, LW, LH, LB, LHU, LBU
 *
 * TESTING:
 *   Load test programs into instruction memory and verify correct
 *   execution by checking register file contents.
 *
 * CONCEPTS:
 *   - Datapath vs control
 *   - Instruction encoding and decoding
 *   - Critical path determines max clock frequency
 *   - Why single-cycle is simple but slow
 */

`default_nettype none

`include "../common/riscv_defs.vh"

module cpu_single_cycle (
	input  wire        clk,
	input  wire        rst,

	// Instruction memory interface
	output wire [31:0] imem_addr,
	input  wire [31:0] imem_data,

	// Data memory interface
	output wire [31:0] dmem_addr,
	output wire [31:0] dmem_wdata,
	input  wire [31:0] dmem_rdata,
	output wire [3:0]  dmem_wstrb,   // Byte write strobes
	output wire        dmem_ren,

	// Debug outputs (directly to harness/LEDs)
	output wire [31:0] dbg_pc,
	output wire [31:0] dbg_inst,
	output wire [4:0]  dbg_rd_addr,
	output wire [31:0] dbg_rd_data
);

	// ===== Program Counter =====
	reg [31:0] pc;
	wire [31:0] pc_next;
	wire [31:0] pc_plus4;

	assign pc_plus4 = pc + 32'd4;
	assign imem_addr = pc;
	assign dbg_pc = pc;

	always @(posedge clk) begin
		if (rst)
			pc <= 32'h40000000;  // Reset vector (matches badge memory map)
		else
			pc <= pc_next;
	end

	// ===== Instruction Decode =====
	wire [31:0] inst = imem_data;
	assign dbg_inst = inst;

	// Extract instruction fields
	wire [6:0]  opcode = inst[6:0];
	wire [4:0]  rd     = inst[11:7];
	wire [2:0]  funct3 = inst[14:12];
	wire [4:0]  rs1    = inst[19:15];
	wire [4:0]  rs2    = inst[24:20];
	wire [6:0]  funct7 = inst[31:25];

	// Immediate generation
	// TODO: Complete all immediate types
	wire [31:0] imm_i = {{20{inst[31]}}, inst[31:20]};
	wire [31:0] imm_s = {{20{inst[31]}}, inst[31:25], inst[11:7]};
	wire [31:0] imm_b = {{19{inst[31]}}, inst[31], inst[7], inst[30:25], inst[11:8], 1'b0};
	wire [31:0] imm_u = {inst[31:12], 12'b0};
	wire [31:0] imm_j = {{11{inst[31]}}, inst[31], inst[19:12], inst[20], inst[30:21], 1'b0};

	// ===== Register File =====
	wire [31:0] rs1_data, rs2_data;
	reg         rd_wen;
	reg  [31:0] rd_data;

	assign dbg_rd_addr = rd;
	assign dbg_rd_data = rd_data;

	regfile regfile_inst (
		.clk(clk),
		.rst(rst),
		.rs1_addr(rs1),
		.rs1_data(rs1_data),
		.rs2_addr(rs2),
		.rs2_data(rs2_data),
		.rd_addr(rd),
		.rd_data(rd_data),
		.rd_wen(rd_wen)
	);

	// ===== ALU =====
	reg  [31:0] alu_a, alu_b;
	reg  [3:0]  alu_op;
	wire [31:0] alu_result;
	wire        alu_zero, alu_neg, alu_carry, alu_ovf;

	alu alu_inst (
		.operand_a(alu_a),
		.operand_b(alu_b),
		.alu_op(alu_op),
		.result(alu_result),
		.flag_zero(alu_zero),
		.flag_negative(alu_neg),
		.flag_carry(alu_carry),
		.flag_overflow(alu_ovf)
	);

	// ===== Control Logic =====
	// TODO: Generate control signals based on opcode
	reg        branch_taken;
	reg [31:0] branch_target;

	assign pc_next = branch_taken ? branch_target : pc_plus4;

	// Data memory signals
	assign dmem_addr  = alu_result;
	assign dmem_wdata = rs2_data;
	assign dmem_wstrb = (opcode == `OP_STORE) ?
	                    (funct3 == `F3_BYTE ? 4'b0001 :
	                     funct3 == `F3_HALF ? 4'b0011 : 4'b1111) : 4'b0000;
	assign dmem_ren   = (opcode == `OP_LOAD);

	// ===== Datapath Control =====
	always @(*) begin
		// Defaults
		alu_a        = rs1_data;
		alu_b        = rs2_data;
		alu_op       = `ALU_ADD;
		rd_wen       = 0;
		rd_data      = alu_result;
		branch_taken = 0;
		branch_target = pc_plus4;

		case (opcode)
			`OP_ALU: begin
				// R-type: register-register
				alu_a  = rs1_data;
				alu_b  = rs2_data;
				alu_op = {funct7[5], funct3};
				rd_wen = 1;
				rd_data = alu_result;
			end

			`OP_ALUI: begin
				// I-type: register-immediate
				alu_a  = rs1_data;
				alu_b  = imm_i;
				// For SRAI, funct7[5] distinguishes SRL from SRA
				alu_op = (funct3 == `F3_SRL_SRA) ? {funct7[5], funct3} :
				         (funct3 == `F3_ADD_SUB)  ? `ALU_ADD : {1'b0, funct3};
				rd_wen = 1;
				rd_data = alu_result;
			end

			`OP_LUI: begin
				rd_wen = 1;
				rd_data = imm_u;
			end

			`OP_AUIPC: begin
				rd_wen = 1;
				rd_data = pc + imm_u;
			end

			`OP_JAL: begin
				rd_wen = 1;
				rd_data = pc_plus4;
				branch_taken = 1;
				branch_target = pc + imm_j;
			end

			`OP_JALR: begin
				rd_wen = 1;
				rd_data = pc_plus4;
				branch_taken = 1;
				branch_target = (rs1_data + imm_i) & ~32'b1;
			end

			`OP_BRANCH: begin
				// TODO: Implement branch condition evaluation
				// Use ALU to subtract rs1 - rs2, then check funct3
				alu_a  = rs1_data;
				alu_b  = rs2_data;
				alu_op = `ALU_SUB;
				branch_target = pc + imm_b;

				case (funct3)
					`F3_BEQ:  branch_taken = alu_zero;
					`F3_BNE:  branch_taken = !alu_zero;
					`F3_BLT:  branch_taken = (alu_neg ^ alu_ovf);
					`F3_BGE:  branch_taken = !(alu_neg ^ alu_ovf);
					`F3_BLTU: branch_taken = !alu_carry;
					`F3_BGEU: branch_taken = alu_carry;
					default:  branch_taken = 0;
				endcase
			end

			`OP_LOAD: begin
				// Address = rs1 + imm_i, read from data memory
				alu_a  = rs1_data;
				alu_b  = imm_i;
				alu_op = `ALU_ADD;
				rd_wen = 1;
				// TODO: implement byte/halfword sign extension
				rd_data = dmem_rdata;
			end

			`OP_STORE: begin
				// Address = rs1 + imm_s
				alu_a  = rs1_data;
				alu_b  = imm_s;
				alu_op = `ALU_ADD;
			end

			default: begin
				// Unknown instruction - NOP
			end
		endcase
	end

endmodule
