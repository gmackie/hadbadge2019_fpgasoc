/*
 * Lab 1: Combinational Logic
 * ==========================
 * Your first Verilog module! This runs directly on the FPGA fabric,
 * not as software on the RISC-V processor.
 *
 * In this lab you will implement basic combinational circuits:
 * the building blocks of all digital systems.
 *
 * EXERCISES:
 *
 * Exercise 1: 4-to-1 Multiplexer (below)
 *   Complete the mux4 module. Use the 'sel' input to choose which
 *   of the 4 data inputs appears on the output.
 *
 * Exercise 2: 4-bit Adder
 *   Create a module `adder4` with inputs a[3:0], b[3:0], cin
 *   and outputs sum[3:0], cout. Test by connecting buttons to
 *   inputs and observing LEDs.
 *
 * Exercise 3: 7-segment Decoder
 *   Create a module that takes a 4-bit BCD input and produces
 *   the 7 segment enables. Display on LEDs (or create a
 *   virtual 7-seg using the LCD harness).
 *
 * Exercise 4: Priority Encoder
 *   8 inputs, 3-bit output indicating the highest active input.
 *   Include a 'valid' output that's 0 when no inputs are active.
 *
 * Exercise 5 (challenge): ALU Preview
 *   Build a 4-bit ALU with operations: ADD, SUB, AND, OR, XOR, NOT.
 *   Use 3 button inputs to select the operation.
 *
 * TESTING:
 *   The lcd_test_harness connects your module to the badge buttons
 *   (as inputs) and LEDs (as outputs). The companion display program
 *   shows the state on the LCD.
 *
 * CONCEPTS:
 *   - Combinational vs sequential logic
 *   - Verilog assign statements and always @(*) blocks
 *   - Bitwise operations in hardware
 *   - Propagation delay (all outputs change "instantly")
 */

`default_nettype none

module mux4 (
	input  wire [7:0] data_in,   // 4 x 2-bit inputs packed: {d3,d2,d1,d0}
	input  wire [1:0] sel,       // Select which input to route to output
	output wire [1:0] data_out   // Selected 2-bit output
);

	// TODO: Implement the multiplexer
	// Hint: You can use a case statement in an always block,
	// or the ternary operator (?:), or bit indexing.
	//
	// Example using conditional operator:
	//   assign data_out = (sel == 2'b00) ? data_in[1:0] :
	//                     (sel == 2'b01) ? data_in[3:2] :
	//                     (sel == 2'b10) ? data_in[5:4] :
	//                                     data_in[7:6];

	assign data_out = 2'b00;  // Replace this with your implementation

endmodule
