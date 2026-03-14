/*
 * Lab 3: Finite State Machines - Traffic Light Controller
 * =======================================================
 * The classic digital logic project! Design a traffic light
 * controller as a finite state machine (FSM).
 *
 * This lab ties together combinational logic (output decoder),
 * sequential logic (state register), and state machine design.
 *
 * SPECIFICATION:
 *   - Two-way intersection (North-South and East-West)
 *   - Each direction has Red, Yellow, Green lights
 *   - Pedestrian walk request button
 *   - Emergency vehicle override
 *
 * TIMING (at 48MHz clock, prescaled):
 *   Green:  30 seconds (or until pedestrian request)
 *   Yellow:  5 seconds
 *   Red:     2 seconds (all-red clearance interval)
 *   Walk:   15 seconds
 *
 * EXERCISES:
 *
 * Exercise 1: Implement the basic 4-state FSM:
 *   NS_GREEN -> NS_YELLOW -> EW_GREEN -> EW_YELLOW -> (repeat)
 *   Map to LEDs: LED[2:0] = NS lights, LED[5:3] = EW lights
 *
 * Exercise 2: Add timer-based transitions
 *   Use a prescaler to divide the 48MHz clock to 1Hz,
 *   then count seconds in each state.
 *
 * Exercise 3: Add pedestrian crossing
 *   Button A = pedestrian request. When requested, insert a
 *   WALK state with flashing output. Clear request after walk.
 *
 * Exercise 4: Add emergency vehicle override
 *   Button B = emergency. Immediately go to all-red, then
 *   green in the emergency direction.
 *
 * Exercise 5 (challenge): Add turn arrows
 *   Add left-turn arrow phases. This requires additional states
 *   and careful safety analysis (no conflicting greens!).
 *
 * CONCEPTS:
 *   - State encoding (binary vs one-hot)
 *   - State transition diagrams
 *   - Output logic (Moore vs Mealy machines)
 *   - Timer/prescaler design
 *   - Safety-critical design (never two conflicting greens)
 *
 * LED MAPPING (directly visible on badge LEDs):
 *   LED[0] = NS Red
 *   LED[1] = NS Yellow
 *   LED[2] = NS Green
 *   LED[3] = EW Red
 *   LED[4] = EW Yellow
 *   LED[5] = EW Green
 *   LED[6] = Walk indicator
 *   LED[7] = Emergency active
 */

`default_nettype none

module traffic_light (
	input  wire       clk,
	input  wire       rst,
	input  wire       pedestrian_req,   // Button A: walk request
	input  wire       emergency,        // Button B: emergency override
	output reg  [7:0] lights,           // LED outputs
	output reg  [3:0] current_state     // Exposed to harness for display
);

	// Clock prescaler: 48MHz -> 1Hz tick
	localparam PRESCALE = 48_000_000;
	reg [25:0] prescaler;
	wire       tick;

	always @(posedge clk) begin
		if (rst || prescaler >= PRESCALE - 1)
			prescaler <= 0;
		else
			prescaler <= prescaler + 1;
	end
	assign tick = (prescaler == 0);

	// Timer: counts seconds in current state
	reg [5:0] timer;

	// State definitions
	localparam S_NS_GREEN   = 4'd0;
	localparam S_NS_YELLOW  = 4'd1;
	localparam S_ALL_RED_1  = 4'd2;
	localparam S_EW_GREEN   = 4'd3;
	localparam S_EW_YELLOW  = 4'd4;
	localparam S_ALL_RED_2  = 4'd5;
	localparam S_WALK       = 4'd6;
	localparam S_EMERGENCY  = 4'd7;

	// Timing constants (in seconds)
	localparam T_GREEN  = 6'd30;
	localparam T_YELLOW = 6'd5;
	localparam T_RED    = 6'd2;
	localparam T_WALK   = 6'd15;

	// Pedestrian request latch
	reg ped_latch;

	// State register
	reg [3:0] state;

	// TODO: Implement the FSM
	//
	// Start with Exercise 1: basic 4-state cycle
	// The state machine should transition on 'tick' pulses

	always @(posedge clk) begin
		if (rst) begin
			state     <= S_NS_GREEN;
			timer     <= 0;
			ped_latch <= 0;
		end else begin
			// Latch pedestrian request
			if (pedestrian_req)
				ped_latch <= 1;

			if (tick) begin
				timer <= timer + 1;

				case (state)
					S_NS_GREEN: begin
						// TODO: transition to NS_YELLOW after T_GREEN seconds
						// or when pedestrian is requested (min 5 seconds)
						if (timer >= T_GREEN - 1) begin
							state <= S_NS_YELLOW;
							timer <= 0;
						end
					end

					S_NS_YELLOW: begin
						if (timer >= T_YELLOW - 1) begin
							state <= S_ALL_RED_1;
							timer <= 0;
						end
					end

					S_ALL_RED_1: begin
						if (timer >= T_RED - 1) begin
							state <= S_EW_GREEN;
							timer <= 0;
						end
					end

					S_EW_GREEN: begin
						if (timer >= T_GREEN - 1) begin
							state <= S_EW_YELLOW;
							timer <= 0;
						end
					end

					S_EW_YELLOW: begin
						if (timer >= T_YELLOW - 1) begin
							state <= S_ALL_RED_2;
							timer <= 0;
						end
					end

					S_ALL_RED_2: begin
						if (timer >= T_RED - 1) begin
							// TODO (Exercise 3): check ped_latch,
							// go to S_WALK if set
							state <= S_NS_GREEN;
							timer <= 0;
						end
					end

					S_WALK: begin
						if (timer >= T_WALK - 1) begin
							ped_latch <= 0;
							state <= S_NS_GREEN;
							timer <= 0;
						end
					end

					default: begin
						state <= S_NS_GREEN;
						timer <= 0;
					end
				endcase
			end
		end
	end

	// Output logic (Moore machine: outputs depend only on state)
	always @(*) begin
		case (state)
			//                    Emerg Walk EW(RYG) NS(RYG)
			S_NS_GREEN:  lights = 8'b0_0_100_001;
			S_NS_YELLOW: lights = 8'b0_0_100_010;
			S_ALL_RED_1: lights = 8'b0_0_100_100;
			S_EW_GREEN:  lights = 8'b0_0_001_100;
			S_EW_YELLOW: lights = 8'b0_0_010_100;
			S_ALL_RED_2: lights = 8'b0_0_100_100;
			S_WALK:      lights = 8'b0_1_100_100;
			S_EMERGENCY: lights = 8'b1_0_100_100;
			default:     lights = 8'b0_0_100_100; // Safe default: all red
		endcase
	end

	always @(*) begin
		current_state = state;
	end

endmodule
