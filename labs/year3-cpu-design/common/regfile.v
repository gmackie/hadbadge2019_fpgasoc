/*
 * regfile.v - RISC-V 32x32-bit Register File
 *
 * Standard RISC-V register file:
 * - 32 registers, each 32 bits wide
 * - x0 is hardwired to zero
 * - 2 read ports (for rs1 and rs2)
 * - 1 write port (for rd)
 * - Write occurs on clock edge; reads are combinational
 *
 * This module is provided complete - it's used by all CPU labs.
 */

`default_nettype none

module regfile (
	input  wire        clk,
	input  wire        rst,

	// Read port 1 (rs1)
	input  wire [4:0]  rs1_addr,
	output wire [31:0] rs1_data,

	// Read port 2 (rs2)
	input  wire [4:0]  rs2_addr,
	output wire [31:0] rs2_data,

	// Write port (rd)
	input  wire [4:0]  rd_addr,
	input  wire [31:0] rd_data,
	input  wire        rd_wen
);

	// 32 registers x 32 bits
	reg [31:0] regs [1:31];  // x0 is implicit zero

	// Read ports: x0 always returns 0
	assign rs1_data = (rs1_addr == 0) ? 32'b0 : regs[rs1_addr];
	assign rs2_data = (rs2_addr == 0) ? 32'b0 : regs[rs2_addr];

	// Write port: ignore writes to x0
	integer i;
	always @(posedge clk) begin
		if (rst) begin
			for (i = 1; i < 32; i = i + 1)
				regs[i] <= 32'b0;
		end else if (rd_wen && rd_addr != 0) begin
			regs[rd_addr] <= rd_data;
		end
	end

endmodule
