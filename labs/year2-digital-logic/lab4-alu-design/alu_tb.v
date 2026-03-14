/*
 * alu_tb.v - Testbench for the ALU
 *
 * Run with: iverilog -o alu_tb alu.v alu_tb.v && vvp alu_tb
 */

`timescale 1ns/1ps

module alu_tb;

	reg  [31:0] a, b;
	reg  [3:0]  op;
	wire [31:0] result;
	wire        zero, negative, carry, overflow;

	alu uut (
		.operand_a(a),
		.operand_b(b),
		.alu_op(op),
		.result(result),
		.flag_zero(zero),
		.flag_negative(negative),
		.flag_carry(carry),
		.flag_overflow(overflow)
	);

	integer pass_count = 0;
	integer fail_count = 0;

	task check;
		input [31:0] expected;
		input [127:0] test_name;
		begin
			if (result !== expected) begin
				$display("FAIL: %0s: got 0x%08x, expected 0x%08x", test_name, result, expected);
				fail_count = fail_count + 1;
			end else begin
				$display("PASS: %0s: 0x%08x", test_name, result);
				pass_count = pass_count + 1;
			end
		end
	endtask

	initial begin
		$display("=== ALU Testbench ===\n");

		// --- ADD tests ---
		op = 4'b0000;

		a = 32'd5; b = 32'd3; #10;
		check(32'd8, "ADD 5+3");

		a = 32'd0; b = 32'd0; #10;
		check(32'd0, "ADD 0+0");
		if (!zero) begin
			$display("FAIL: Zero flag not set for 0+0");
			fail_count = fail_count + 1;
		end

		a = 32'h7FFFFFFF; b = 32'd1; #10;
		check(32'h80000000, "ADD overflow");
		if (!overflow) begin
			$display("FAIL: Overflow flag not set");
			fail_count = fail_count + 1;
		end

		// --- SUB tests ---
		op = 4'b1000;

		a = 32'd10; b = 32'd3; #10;
		check(32'd7, "SUB 10-3");

		a = 32'd3; b = 32'd10; #10;
		check(-32'd7, "SUB 3-10");
		if (!negative) begin
			$display("FAIL: Negative flag not set for 3-10");
			fail_count = fail_count + 1;
		end

		// --- AND tests ---
		op = 4'b0111;
		a = 32'hFF00FF00; b = 32'h0F0F0F0F; #10;
		check(32'h0F000F00, "AND");

		// --- OR tests ---
		op = 4'b0110;
		a = 32'hFF00FF00; b = 32'h0F0F0F0F; #10;
		check(32'hFF0FFF0F, "OR");

		// --- XOR tests ---
		op = 4'b0100;
		a = 32'hFF00FF00; b = 32'h0F0F0F0F; #10;
		check(32'hF00FF00F, "XOR");

		// --- SLT tests ---
		op = 4'b0010;
		a = 32'd5; b = 32'd10; #10;
		check(32'd1, "SLT 5<10");

		a = 32'd10; b = 32'd5; #10;
		check(32'd0, "SLT 10<5");

		// --- Summary ---
		$display("\n=== Results: %0d passed, %0d failed ===", pass_count, fail_count);
		if (fail_count == 0)
			$display("ALL TESTS PASSED!");
		else
			$display("SOME TESTS FAILED!");

		$finish;
	end

endmodule
