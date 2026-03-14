/*
 * Lab 2: Sequential Logic
 * =======================
 * Introduction to clocked circuits: flip-flops, registers, and counters.
 * Unlike combinational logic, sequential circuits have memory - their
 * outputs depend on BOTH current inputs AND previous state.
 *
 * EXERCISES:
 *
 * Exercise 1: 8-bit Counter (below)
 *   Complete the counter module. It should count up on each clock
 *   cycle, reset to 0 when rst is high, and hold its value when
 *   enable is low. Display the count on the LEDs.
 *
 * Exercise 2: Up/Down Counter
 *   Add a 'direction' input. When high, count up. When low, count down.
 *   Connect to buttons: A=enable, B=direction, UP=reset.
 *
 * Exercise 3: Shift Register
 *   Create an 8-bit shift register with serial input (from button A),
 *   parallel output (to LEDs), and shift clock (from button B).
 *   Add a load input to parallel-load from buttons.
 *
 * Exercise 4: Debouncer
 *   Badge buttons bounce! Create a debounce circuit that requires
 *   the input to be stable for N clock cycles before changing
 *   the output. Compare raw vs debounced button presses.
 *
 * Exercise 5 (challenge): PWM Generator
 *   Create a pulse-width modulation module. Use a counter and
 *   comparator to generate a variable-duty-cycle output.
 *   Connect to an LED to see it dim/brighten with button presses.
 *
 * CONCEPTS:
 *   - always @(posedge clk) - the clocked always block
 *   - Reset: synchronous vs asynchronous
 *   - Enable signals
 *   - Registers vs wires
 *   - The "nonblocking assignment" (<=) in sequential logic
 */

`default_nettype none

module counter #(
	parameter WIDTH = 8
)(
	input  wire             clk,
	input  wire             rst,      // Synchronous reset
	input  wire             enable,   // Count enable
	output reg  [WIDTH-1:0] count,    // Current count value
	output wire             overflow  // Pulses when count wraps
);

	// TODO: Implement the counter
	//
	// On each rising clock edge:
	//   - If rst is high, set count to 0
	//   - Else if enable is high, increment count by 1
	//   - Otherwise, hold current value
	//
	// The overflow output should be high for one cycle when the
	// counter wraps from its maximum value back to 0.

	assign overflow = 1'b0;  // Replace with your implementation

	always @(posedge clk) begin
		if (rst) begin
			count <= 0;
		end else if (enable) begin
			// TODO: increment count
		end
	end

	// For testing with the harness, expose internal state:
	// The harness reads 'count' directly via student_state

endmodule

// Exercise 4 starter: Debouncer
// module debounce #(
//     parameter THRESHOLD = 48000  // ~1ms at 48MHz
// )(
//     input  wire clk,
//     input  wire rst,
//     input  wire raw_in,
//     output reg  clean_out
// );
//     reg [15:0] stable_count;
//     reg        last_raw;
//
//     always @(posedge clk) begin
//         if (rst) begin
//             stable_count <= 0;
//             clean_out <= 0;
//             last_raw <= 0;
//         end else begin
//             if (raw_in != last_raw) begin
//                 stable_count <= 0;
//                 last_raw <= raw_in;
//             end else if (stable_count < THRESHOLD) begin
//                 stable_count <= stable_count + 1;
//             end else begin
//                 clean_out <= last_raw;
//             end
//         end
//     end
// endmodule
