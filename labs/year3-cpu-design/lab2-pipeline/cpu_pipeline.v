/*
 * Lab 2: 5-Stage Pipelined RISC-V Processor
 * ==========================================
 * Transform the single-cycle CPU into a pipelined design.
 * The pipeline has 5 stages:
 *
 *   IF (Instruction Fetch) -> ID (Instruction Decode) ->
 *   EX (Execute) -> MEM (Memory Access) -> WB (Write Back)
 *
 * Each stage does 1/5 of the work, so the clock can be 5x faster,
 * but each instruction still takes 5 cycles to complete. Throughput
 * increases because stages work in parallel on different instructions.
 *
 * EXERCISES:
 *
 * Exercise 1: Add pipeline registers
 *   Insert registers between each stage. Data flows through the
 *   pipeline registers on each clock edge. Start with the IF/ID
 *   and ID/EX boundaries (provided below as skeleton).
 *
 * Exercise 2: Verify R-type instructions flow through the pipeline
 *   Run the same test program from Lab 1. Results should appear
 *   5 cycles later but be identical.
 *
 * Exercise 3: Identify data hazards (Lab 3 will fix them)
 *   What happens when instruction N+1 reads a register that
 *   instruction N writes? Run this program and observe:
 *     addi x1, x0, 5
 *     addi x2, x1, 3    # x2 should be 8, but x1 hasn't been written yet!
 *
 * Exercise 4: Identify control hazards
 *   What happens when a branch is taken? The instructions after
 *   the branch have already entered the pipeline!
 *
 * CONCEPTS:
 *   - Pipelining: trading latency for throughput
 *   - Pipeline registers (stage boundaries)
 *   - Hazards: structural, data, control
 *   - CPI (clocks per instruction) - ideal is 1.0
 */

`default_nettype none

`include "../common/riscv_defs.vh"

module cpu_pipeline (
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

	// Debug
	output wire [31:0] dbg_pc,
	output wire [31:0] dbg_inst
);

	// ====================================================
	// Stage 1: IF (Instruction Fetch)
	// ====================================================
	reg [31:0] pc;
	wire [31:0] pc_plus4 = pc + 32'd4;

	// PC source selection (will be driven by branch logic in EX stage)
	wire        branch_taken;
	wire [31:0] branch_target;
	wire [31:0] pc_next = branch_taken ? branch_target : pc_plus4;

	always @(posedge clk) begin
		if (rst)
			pc <= 32'h40000000;
		else
			pc <= pc_next;
	end

	assign imem_addr = pc;
	assign dbg_pc = pc;
	assign dbg_inst = imem_data;

	// ====================================================
	// Pipeline Register: IF/ID
	// ====================================================
	reg [31:0] ifid_pc;
	reg [31:0] ifid_inst;
	reg        ifid_valid;

	always @(posedge clk) begin
		if (rst || branch_taken) begin
			ifid_pc    <= 32'b0;
			ifid_inst  <= 32'h00000013;  // NOP
			ifid_valid <= 0;
		end else begin
			ifid_pc    <= pc;
			ifid_inst  <= imem_data;
			ifid_valid <= 1;
		end
	end

	// ====================================================
	// Stage 2: ID (Instruction Decode)
	// ====================================================
	wire [6:0] id_opcode = ifid_inst[6:0];
	wire [4:0] id_rd     = ifid_inst[11:7];
	wire [2:0] id_funct3 = ifid_inst[14:12];
	wire [4:0] id_rs1    = ifid_inst[19:15];
	wire [4:0] id_rs2    = ifid_inst[24:20];
	wire [6:0] id_funct7 = ifid_inst[31:25];

	// Immediate decode
	wire [31:0] id_imm_i = {{20{ifid_inst[31]}}, ifid_inst[31:20]};
	wire [31:0] id_imm_s = {{20{ifid_inst[31]}}, ifid_inst[31:25], ifid_inst[11:7]};
	wire [31:0] id_imm_b = {{19{ifid_inst[31]}}, ifid_inst[31], ifid_inst[7],
	                         ifid_inst[30:25], ifid_inst[11:8], 1'b0};
	wire [31:0] id_imm_u = {ifid_inst[31:12], 12'b0};
	wire [31:0] id_imm_j = {{11{ifid_inst[31]}}, ifid_inst[31], ifid_inst[19:12],
	                         ifid_inst[20], ifid_inst[30:21], 1'b0};

	// Register file (reads in ID, writes in WB)
	wire [31:0] id_rs1_data, id_rs2_data;
	wire [4:0]  wb_rd_addr;
	wire [31:0] wb_rd_data;
	wire        wb_rd_wen;

	regfile regfile_inst (
		.clk(clk), .rst(rst),
		.rs1_addr(id_rs1), .rs1_data(id_rs1_data),
		.rs2_addr(id_rs2), .rs2_data(id_rs2_data),
		.rd_addr(wb_rd_addr), .rd_data(wb_rd_data), .rd_wen(wb_rd_wen)
	);

	// Control signal decode
	reg       id_alu_src;      // 0=rs2, 1=immediate
	reg [3:0] id_alu_op;
	reg       id_mem_read;
	reg       id_mem_write;
	reg       id_reg_write;
	reg       id_is_branch;
	reg       id_is_jal;
	reg       id_is_jalr;
	reg [31:0] id_imm;

	always @(*) begin
		id_alu_src   = 0;
		id_alu_op    = `ALU_ADD;
		id_mem_read  = 0;
		id_mem_write = 0;
		id_reg_write = 0;
		id_is_branch = 0;
		id_is_jal    = 0;
		id_is_jalr   = 0;
		id_imm       = id_imm_i;

		case (id_opcode)
			`OP_ALU: begin
				id_alu_op    = {id_funct7[5], id_funct3};
				id_reg_write = 1;
			end
			`OP_ALUI: begin
				id_alu_src   = 1;
				id_alu_op    = (id_funct3 == `F3_SRL_SRA) ? {id_funct7[5], id_funct3} :
				               (id_funct3 == `F3_ADD_SUB) ? `ALU_ADD : {1'b0, id_funct3};
				id_reg_write = 1;
				id_imm       = id_imm_i;
			end
			`OP_LOAD: begin
				id_alu_src   = 1;
				id_mem_read  = 1;
				id_reg_write = 1;
				id_imm       = id_imm_i;
			end
			`OP_STORE: begin
				id_alu_src   = 1;
				id_mem_write = 1;
				id_imm       = id_imm_s;
			end
			`OP_BRANCH: begin
				id_is_branch = 1;
				id_alu_op    = `ALU_SUB;
				id_imm       = id_imm_b;
			end
			`OP_JAL: begin
				id_is_jal    = 1;
				id_reg_write = 1;
				id_imm       = id_imm_j;
			end
			`OP_JALR: begin
				id_is_jalr   = 1;
				id_reg_write = 1;
				id_imm       = id_imm_i;
			end
			`OP_LUI: begin
				id_reg_write = 1;
				id_imm       = id_imm_u;
			end
			`OP_AUIPC: begin
				id_reg_write = 1;
				id_imm       = id_imm_u;
			end
			default: ;
		endcase
	end

	// ====================================================
	// Pipeline Register: ID/EX
	// ====================================================
	reg [31:0] idex_pc;
	reg [31:0] idex_rs1_data, idex_rs2_data;
	reg [4:0]  idex_rd;
	reg [4:0]  idex_rs1, idex_rs2;
	reg [2:0]  idex_funct3;
	reg [31:0] idex_imm;
	reg        idex_alu_src;
	reg [3:0]  idex_alu_op;
	reg        idex_mem_read, idex_mem_write;
	reg        idex_reg_write;
	reg        idex_is_branch, idex_is_jal, idex_is_jalr;
	reg [6:0]  idex_opcode;
	reg        idex_valid;

	always @(posedge clk) begin
		if (rst || branch_taken) begin
			idex_pc        <= 0;
			idex_rs1_data  <= 0;
			idex_rs2_data  <= 0;
			idex_rd        <= 0;
			idex_rs1       <= 0;
			idex_rs2       <= 0;
			idex_funct3    <= 0;
			idex_imm       <= 0;
			idex_alu_src   <= 0;
			idex_alu_op    <= 0;
			idex_mem_read  <= 0;
			idex_mem_write <= 0;
			idex_reg_write <= 0;
			idex_is_branch <= 0;
			idex_is_jal    <= 0;
			idex_is_jalr   <= 0;
			idex_opcode    <= 0;
			idex_valid     <= 0;
		end else begin
			idex_pc        <= ifid_pc;
			idex_rs1_data  <= id_rs1_data;
			idex_rs2_data  <= id_rs2_data;
			idex_rd        <= id_rd;
			idex_rs1       <= id_rs1;
			idex_rs2       <= id_rs2;
			idex_funct3    <= id_funct3;
			idex_imm       <= id_imm;
			idex_alu_src   <= id_alu_src;
			idex_alu_op    <= id_alu_op;
			idex_mem_read  <= id_mem_read;
			idex_mem_write <= id_mem_write;
			idex_reg_write <= id_reg_write;
			idex_is_branch <= id_is_branch;
			idex_is_jal    <= id_is_jal;
			idex_is_jalr   <= id_is_jalr;
			idex_opcode    <= id_opcode;
			idex_valid     <= ifid_valid;
		end
	end

	// ====================================================
	// Stage 3: EX (Execute)
	// ====================================================
	wire [31:0] ex_alu_b = idex_alu_src ? idex_imm : idex_rs2_data;

	wire [31:0] ex_alu_result;
	wire ex_alu_zero, ex_alu_neg, ex_alu_carry, ex_alu_ovf;

	alu alu_inst (
		.operand_a(idex_rs1_data),
		.operand_b(ex_alu_b),
		.alu_op(idex_alu_op),
		.result(ex_alu_result),
		.flag_zero(ex_alu_zero),
		.flag_negative(ex_alu_neg),
		.flag_carry(ex_alu_carry),
		.flag_overflow(ex_alu_ovf)
	);

	// Branch resolution
	reg ex_branch_taken;
	always @(*) begin
		ex_branch_taken = 0;
		if (idex_is_jal || idex_is_jalr)
			ex_branch_taken = 1;
		else if (idex_is_branch) begin
			case (idex_funct3)
				`F3_BEQ:  ex_branch_taken = ex_alu_zero;
				`F3_BNE:  ex_branch_taken = !ex_alu_zero;
				`F3_BLT:  ex_branch_taken = (ex_alu_neg ^ ex_alu_ovf);
				`F3_BGE:  ex_branch_taken = !(ex_alu_neg ^ ex_alu_ovf);
				`F3_BLTU: ex_branch_taken = !ex_alu_carry;
				`F3_BGEU: ex_branch_taken = ex_alu_carry;
				default:  ex_branch_taken = 0;
			endcase
		end
	end

	assign branch_taken = ex_branch_taken && idex_valid;
	assign branch_target = idex_is_jalr ? (idex_rs1_data + idex_imm) & ~32'b1 :
	                                      idex_pc + idex_imm;

	// EX stage result mux
	reg [31:0] ex_result;
	always @(*) begin
		case (idex_opcode)
			`OP_JAL, `OP_JALR: ex_result = idex_pc + 32'd4;
			`OP_LUI:           ex_result = idex_imm;
			`OP_AUIPC:         ex_result = idex_pc + idex_imm;
			default:           ex_result = ex_alu_result;
		endcase
	end

	// ====================================================
	// Pipeline Register: EX/MEM
	// ====================================================
	reg [31:0] exmem_alu_result;
	reg [31:0] exmem_rs2_data;
	reg [4:0]  exmem_rd;
	reg [2:0]  exmem_funct3;
	reg        exmem_mem_read, exmem_mem_write;
	reg        exmem_reg_write;
	reg [31:0] exmem_result;
	reg        exmem_valid;

	always @(posedge clk) begin
		if (rst) begin
			exmem_alu_result <= 0;
			exmem_rs2_data   <= 0;
			exmem_rd         <= 0;
			exmem_funct3     <= 0;
			exmem_mem_read   <= 0;
			exmem_mem_write  <= 0;
			exmem_reg_write  <= 0;
			exmem_result     <= 0;
			exmem_valid      <= 0;
		end else begin
			exmem_alu_result <= ex_alu_result;
			exmem_rs2_data   <= idex_rs2_data;
			exmem_rd         <= idex_rd;
			exmem_funct3     <= idex_funct3;
			exmem_mem_read   <= idex_mem_read && idex_valid;
			exmem_mem_write  <= idex_mem_write && idex_valid;
			exmem_reg_write  <= idex_reg_write && idex_valid;
			exmem_result     <= ex_result;
			exmem_valid      <= idex_valid;
		end
	end

	// ====================================================
	// Stage 4: MEM (Memory Access)
	// ====================================================
	assign dmem_addr  = exmem_alu_result;
	assign dmem_wdata = exmem_rs2_data;
	assign dmem_ren   = exmem_mem_read;
	assign dmem_wstrb = exmem_mem_write ?
	                    (exmem_funct3 == `F3_BYTE ? 4'b0001 :
	                     exmem_funct3 == `F3_HALF ? 4'b0011 : 4'b1111) : 4'b0000;

	wire [31:0] mem_result = exmem_mem_read ? dmem_rdata : exmem_result;

	// ====================================================
	// Pipeline Register: MEM/WB
	// ====================================================
	reg [31:0] memwb_result;
	reg [4:0]  memwb_rd;
	reg        memwb_reg_write;

	always @(posedge clk) begin
		if (rst) begin
			memwb_result    <= 0;
			memwb_rd        <= 0;
			memwb_reg_write <= 0;
		end else begin
			memwb_result    <= mem_result;
			memwb_rd        <= exmem_rd;
			memwb_reg_write <= exmem_reg_write;
		end
	end

	// ====================================================
	// Stage 5: WB (Write Back)
	// ====================================================
	assign wb_rd_addr = memwb_rd;
	assign wb_rd_data = memwb_result;
	assign wb_rd_wen  = memwb_reg_write;

endmodule
