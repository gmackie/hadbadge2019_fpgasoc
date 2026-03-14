/*
 * cpu_tb.v - Single-cycle CPU testbench
 *
 * Provides instruction and data memory, loads a test program,
 * and checks register values after execution.
 *
 * Run: iverilog -I../common -o cpu_tb cpu_single_cycle.v \
 *      ../common/regfile.v ../../year2-digital-logic/lab4-alu-design/alu.v \
 *      cpu_tb.v && vvp cpu_tb
 */

`timescale 1ns/1ps

module cpu_tb;

	reg clk, rst;
	wire [31:0] imem_addr, dmem_addr, dmem_wdata;
	wire [31:0] imem_data, dmem_rdata;
	wire [3:0]  dmem_wstrb;
	wire        dmem_ren;
	wire [31:0] dbg_pc, dbg_inst;
	wire [4:0]  dbg_rd_addr;
	wire [31:0] dbg_rd_data;

	// Instruction memory (4KB)
	reg [31:0] imem [0:1023];

	// Data memory (4KB)
	reg [31:0] dmem [0:1023];

	// Memory read
	assign imem_data = imem[(imem_addr - 32'h40000000) >> 2];
	assign dmem_rdata = dmem[(dmem_addr - 32'h40000000) >> 2];

	// Memory write
	always @(posedge clk) begin
		if (|dmem_wstrb)
			dmem[(dmem_addr - 32'h40000000) >> 2] <= dmem_wdata;
	end

	cpu_single_cycle uut (
		.clk(clk), .rst(rst),
		.imem_addr(imem_addr), .imem_data(imem_data),
		.dmem_addr(dmem_addr), .dmem_wdata(dmem_wdata),
		.dmem_rdata(dmem_rdata), .dmem_wstrb(dmem_wstrb),
		.dmem_ren(dmem_ren),
		.dbg_pc(dbg_pc), .dbg_inst(dbg_inst),
		.dbg_rd_addr(dbg_rd_addr), .dbg_rd_data(dbg_rd_data)
	);

	// Clock generation
	always #5 clk = ~clk;

	integer i;
	initial begin
		$dumpfile("cpu_tb.vcd");
		$dumpvars(0, cpu_tb);

		clk = 0;
		rst = 1;

		// Clear memories
		for (i = 0; i < 1024; i = i + 1) begin
			imem[i] = 32'h00000013;  // NOP (addi x0, x0, 0)
			dmem[i] = 32'h00000000;
		end

		// Load test program
		// This program tests basic ALU operations:
		//   addi x1, x0, 5      # x1 = 5
		//   addi x2, x0, 3      # x2 = 3
		//   add  x3, x1, x2     # x3 = 8
		//   sub  x4, x1, x2     # x4 = 2
		//   and  x5, x1, x2     # x5 = 1
		//   or   x6, x1, x2     # x6 = 7
		//   slt  x7, x2, x1     # x7 = 1 (3 < 5)
		//   lui  x8, 0x12345    # x8 = 0x12345000
		//   jal  x0, 0          # infinite loop (halt)

		imem[0] = 32'h00500093;  // addi x1, x0, 5
		imem[1] = 32'h00300113;  // addi x2, x0, 3
		imem[2] = 32'h002081b3;  // add  x3, x1, x2
		imem[3] = 32'h40208233;  // sub  x4, x1, x2
		imem[4] = 32'h0020f2b3;  // and  x5, x1, x2
		imem[5] = 32'h0020e333;  // or   x6, x1, x2
		imem[6] = 32'h001123b3;  // slt  x7, x2, x1
		imem[7] = 32'h12345437;  // lui  x8, 0x12345
		imem[8] = 32'h0000006f;  // jal  x0, 0 (loop forever)

		// Release reset
		#20 rst = 0;

		// Run for enough cycles to execute all instructions
		#200;

		// Check results
		$display("\n=== CPU Single-Cycle Test Results ===");
		$display("PC = 0x%08x (should be 0x40000020 = halted)", dbg_pc);
		$display("x1 = %0d (expected 5)", uut.regfile_inst.regs[1]);
		$display("x2 = %0d (expected 3)", uut.regfile_inst.regs[2]);
		$display("x3 = %0d (expected 8)", uut.regfile_inst.regs[3]);
		$display("x4 = %0d (expected 2)", uut.regfile_inst.regs[4]);
		$display("x5 = %0d (expected 1)", uut.regfile_inst.regs[5]);
		$display("x6 = %0d (expected 7)", uut.regfile_inst.regs[6]);
		$display("x7 = %0d (expected 1)", uut.regfile_inst.regs[7]);
		$display("x8 = 0x%08x (expected 0x12345000)", uut.regfile_inst.regs[8]);

		$finish;
	end

endmodule
