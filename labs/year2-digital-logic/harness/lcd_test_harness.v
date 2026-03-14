/*
 * lcd_test_harness.v - Visual test harness for digital logic labs
 *
 * This module wraps a student's digital logic design and displays its
 * inputs, outputs, and internal state on the badge's LCD via the
 * existing SoC graphics pipeline.
 *
 * The harness:
 * 1. Instantiates the full SoC (for LCD/graphics/button support)
 * 2. Maps badge buttons to the student module's inputs
 * 3. Maps student module's outputs to LEDs
 * 4. Provides a memory-mapped status region the SoC can read
 *    to display the student module's state on the LCD
 *
 * The student writes ONLY their digital logic module. The harness
 * handles all the I/O plumbing. A companion C program running on
 * the RISC-V reads the student module's state and renders it.
 *
 * Usage: students implement the `student_module` interface.
 * The harness instantiates it and connects everything.
 */

`default_nettype none

module lcd_test_harness #(
	parameter NUM_INPUTS  = 8,   // Number of input signals
	parameter NUM_OUTPUTS = 8,   // Number of output signals
	parameter NUM_STATE   = 32   // Bits of internal state to expose
)(
	input  wire        clk,
	input  wire        rst,

	// Badge buttons (directly mapped to student inputs)
	input  wire [7:0]  btn,

	// LEDs (directly driven by student outputs)
	output wire [8:0]  led,

	// Memory-mapped status interface (directly to SoC bus)
	input  wire [3:2]  status_addr,
	output reg  [31:0] status_rdata,
	input  wire        status_cyc,
	output reg         status_ack,

	// Student module I/O (directly exposed for custom wiring)
	output wire [NUM_INPUTS-1:0]  student_inputs,
	input  wire [NUM_OUTPUTS-1:0] student_outputs,
	input  wire [NUM_STATE-1:0]   student_state
);

	// Map buttons to student inputs
	assign student_inputs = btn[NUM_INPUTS-1:0];

	// Map student outputs to LEDs
	assign led = {1'b0, student_outputs[7:0]};

	// Memory-mapped status registers for the SoC to read
	// Address 0x00: student inputs  (what buttons are pressed)
	// Address 0x04: student outputs (what the module is driving)
	// Address 0x08: student state low 32 bits
	// Address 0x0C: student state high 32 bits (if >32 bits)
	always @(posedge clk) begin
		if (rst)
			status_ack <= 0;
		else if (status_ack)
			status_ack <= 0;
		else
			status_ack <= status_cyc;
	end

	always @(*) begin
		case (status_addr)
			2'b00: status_rdata = {{(32-NUM_INPUTS){1'b0}}, student_inputs};
			2'b01: status_rdata = {{(32-NUM_OUTPUTS){1'b0}}, student_outputs};
			2'b10: status_rdata = student_state[31:0];
			2'b11: status_rdata = (NUM_STATE > 32) ?
			                      student_state[NUM_STATE-1:32] : 32'h0;
			default: status_rdata = 32'h0;
		endcase
	end

endmodule
