/*
 * Lab 4: ALU Design
 * =================
 * Design the Arithmetic Logic Unit - the computational heart of a CPU.
 * This lab bridges digital logic design (Year 2) with CPU design (Year 3).
 *
 * Your ALU will be reused in Year 3 when you build a full processor!
 *
 * SPECIFICATION:
 *   - 32-bit data width (matches RISC-V)
 *   - Operations: ADD, SUB, AND, OR, XOR, SLT, SLL, SRL, SRA
 *   - Status flags: Zero, Negative, Carry, Overflow
 *
 * EXERCISES:
 *
 * Exercise 1: Implement ADD and SUB
 *   SUB is ADD with the second operand inverted plus 1 (two's complement).
 *   Set the carry and overflow flags correctly.
 *
 * Exercise 2: Implement logical operations (AND, OR, XOR)
 *   These are straightforward bitwise operations.
 *
 * Exercise 3: Implement SLT (Set Less Than)
 *   Output is 1 if operand_a < operand_b (signed comparison).
 *   This is critical for branch instructions in RISC-V.
 *
 * Exercise 4: Implement shifts (SLL, SRL, SRA)
 *   SLL = shift left logical (fill with zeros)
 *   SRL = shift right logical (fill with zeros)
 *   SRA = shift right arithmetic (fill with sign bit)
 *
 * Exercise 5 (challenge): Add a barrel shifter
 *   Implement SLL/SRL/SRA using a barrel shifter instead of
 *   sequential shifts. Compare the synthesis results.
 *
 * TESTING:
 *   An iverilog testbench is provided. Run: make test
 *
 * CONCEPTS:
 *   - Two's complement arithmetic
 *   - Carry vs overflow
 *   - Arithmetic vs logical shift
 *   - Critical path analysis (which operation is slowest?)
 */

`default_nettype none

module alu (
	input  wire [31:0] operand_a,
	input  wire [31:0] operand_b,
	input  wire [3:0]  alu_op,
	output reg  [31:0] result,
	output wire        flag_zero,
	output wire        flag_negative,
	output reg         flag_carry,
	output reg         flag_overflow
);

	// ALU operation encoding (matches RISC-V funct3 where possible)
	localparam OP_ADD  = 4'b0000;
	localparam OP_SUB  = 4'b1000;
	localparam OP_AND  = 4'b0111;
	localparam OP_OR   = 4'b0110;
	localparam OP_XOR  = 4'b0100;
	localparam OP_SLT  = 4'b0010;  // Set less than (signed)
	localparam OP_SLTU = 4'b0011;  // Set less than (unsigned)
	localparam OP_SLL  = 4'b0001;  // Shift left logical
	localparam OP_SRL  = 4'b0101;  // Shift right logical
	localparam OP_SRA  = 4'b1101;  // Shift right arithmetic

	// Internal signals for addition
	wire [32:0] add_result;
	wire [31:0] b_effective;
	wire        is_sub;

	// SUB is implemented as ADD with inverted B and carry-in of 1
	assign is_sub = alu_op[3];
	assign b_effective = is_sub ? ~operand_b : operand_b;
	assign add_result = {1'b0, operand_a} + {1'b0, b_effective} + {32'b0, is_sub};

	// Flags
	assign flag_zero     = (result == 32'b0);
	assign flag_negative = result[31];

	always @(*) begin
		flag_carry    = 0;
		flag_overflow = 0;

		case (alu_op)
			OP_ADD, OP_SUB: begin
				result = add_result[31:0];
				flag_carry = add_result[32];
				// Overflow: sign of inputs differs from sign of result
				flag_overflow = (operand_a[31] == b_effective[31]) &&
				                (result[31] != operand_a[31]);
			end

			OP_AND: begin
				result = operand_a & operand_b;
			end

			OP_OR: begin
				result = operand_a | operand_b;
			end

			OP_XOR: begin
				result = operand_a ^ operand_b;
			end

			OP_SLT: begin
				// Signed comparison using subtraction result
				// a < b when: (a-b) is negative XOR overflow occurred
				result = {31'b0, add_result[31] ^ flag_overflow};
				// TODO: fix overflow calculation for SLT
				// Hint: reuse the SUB path
			end

			OP_SLTU: begin
				// Unsigned comparison
				result = {31'b0, ~add_result[32]};
			end

			OP_SLL: begin
				// TODO: Implement shift left logical
				// result = operand_a << operand_b[4:0];
				result = 32'b0;
			end

			OP_SRL: begin
				// TODO: Implement shift right logical
				// result = operand_a >> operand_b[4:0];
				result = 32'b0;
			end

			OP_SRA: begin
				// TODO: Implement shift right arithmetic
				// result = $signed(operand_a) >>> operand_b[4:0];
				result = 32'b0;
			end

			default: begin
				result = 32'b0;
			end
		endcase
	end

endmodule
