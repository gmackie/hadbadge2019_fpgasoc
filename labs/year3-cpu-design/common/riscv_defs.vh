/*
 * riscv_defs.vh - RISC-V RV32I constant definitions
 *
 * Shared definitions for all CPU design labs.
 * These match the RISC-V ISA specification.
 */

`ifndef RISCV_DEFS_VH
`define RISCV_DEFS_VH

// Opcodes (inst[6:0])
`define OP_LUI     7'b0110111
`define OP_AUIPC   7'b0010111
`define OP_JAL     7'b1101111
`define OP_JALR    7'b1100111
`define OP_BRANCH  7'b1100011
`define OP_LOAD    7'b0000011
`define OP_STORE   7'b0100011
`define OP_ALUI    7'b0010011  // ALU immediate
`define OP_ALU     7'b0110011  // ALU register-register
`define OP_FENCE   7'b0001111
`define OP_SYSTEM  7'b1110011

// funct3 for ALU operations
`define F3_ADD_SUB 3'b000
`define F3_SLL     3'b001
`define F3_SLT     3'b010
`define F3_SLTU    3'b011
`define F3_XOR     3'b100
`define F3_SRL_SRA 3'b101
`define F3_OR      3'b110
`define F3_AND     3'b111

// funct3 for branch operations
`define F3_BEQ     3'b000
`define F3_BNE     3'b001
`define F3_BLT     3'b100
`define F3_BGE     3'b101
`define F3_BLTU    3'b110
`define F3_BGEU    3'b111

// funct3 for load/store
`define F3_BYTE    3'b000
`define F3_HALF    3'b001
`define F3_WORD    3'b010
`define F3_BYTEU   3'b100
`define F3_HALFU   3'b101

// ALU operation encoding
`define ALU_ADD    4'b0000
`define ALU_SUB    4'b1000
`define ALU_SLL    4'b0001
`define ALU_SLT    4'b0010
`define ALU_SLTU   4'b0011
`define ALU_XOR    4'b0100
`define ALU_SRL    4'b0101
`define ALU_SRA    4'b1101
`define ALU_OR     4'b0110
`define ALU_AND    4'b0111

// Pipeline stage constants
`define STAGE_IF   3'd0
`define STAGE_ID   3'd1
`define STAGE_EX   3'd2
`define STAGE_MEM  3'd3
`define STAGE_WB   3'd4

`endif
