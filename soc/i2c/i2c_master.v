/*
 * i2c_master.v - I2C master controller core
 *
 * Implements an I2C master with support for standard mode (100kHz)
 * and fast mode (400kHz). Uses open-drain signaling via active-low
 * drive with tristate control.
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module i2c_master (
	input  wire        clk,
	input  wire        rst,

	// I2C physical interface (directly to pads)
	// Active-low open-drain: drive low to assert, tristate to release (pulled high externally)
	output wire        scl_o,   // SCL output (active low drive)
	output wire        scl_oe,  // SCL output enable (1 = driving low)
	input  wire        scl_i,   // SCL input (for clock stretching)
	output wire        sda_o,   // SDA output (active low drive)
	output wire        sda_oe,  // SDA output enable (1 = driving low)
	input  wire        sda_i,   // SDA input

	// Command interface
	input  wire        cmd_start,   // Issue START condition
	input  wire        cmd_stop,    // Issue STOP condition
	input  wire        cmd_write,   // Write a byte
	input  wire        cmd_read,    // Read a byte
	input  wire        cmd_ack_val, // ACK value to send after read (0=ACK, 1=NACK)
	input  wire [7:0]  write_data,  // Byte to write

	output reg  [7:0]  read_data,   // Byte read
	output reg         ack_received, // ACK bit received after write (0=ACK, 1=NACK)
	output reg         busy,        // Controller is busy
	output reg         done,        // Operation complete (1 cycle pulse)

	// Clock configuration
	input  wire [15:0] clk_div      // SCL period = clk_div * 4 clock cycles
);

	// State machine
	localparam ST_IDLE       = 4'd0;
	localparam ST_START1     = 4'd1;
	localparam ST_START2     = 4'd2;
	localparam ST_WRITE_BIT  = 4'd3;
	localparam ST_WRITE_CLK  = 4'd4;
	localparam ST_WRITE_DONE = 4'd5;
	localparam ST_READ_BIT   = 4'd6;
	localparam ST_READ_CLK   = 4'd7;
	localparam ST_READ_DONE  = 4'd8;
	localparam ST_ACK_SEND   = 4'd9;
	localparam ST_ACK_CLK    = 4'd10;
	localparam ST_STOP1      = 4'd11;
	localparam ST_STOP2      = 4'd12;
	localparam ST_STOP3      = 4'd13;

	reg [3:0]  state;
	reg [15:0] clk_cnt;
	reg [3:0]  bit_cnt;
	reg [7:0]  shift_reg;
	reg        scl_reg;
	reg        sda_reg;

	// Open-drain: output 0 when driving, tristate (OE=0) to release high
	assign scl_o  = 1'b0;
	assign scl_oe = ~scl_reg;  // OE=1 when scl_reg=0 (driving low)
	assign sda_o  = 1'b0;
	assign sda_oe = ~sda_reg;  // OE=1 when sda_reg=0 (driving low)

	// Clock divider tick
	wire clk_tick = (clk_cnt == 0);

	always @(posedge clk) begin
		if (rst) begin
			clk_cnt <= 0;
		end else if (state == ST_IDLE) begin
			clk_cnt <= clk_div;
		end else if (clk_cnt == 0) begin
			clk_cnt <= clk_div;
		end else begin
			clk_cnt <= clk_cnt - 1;
		end
	end

	always @(posedge clk) begin
		if (rst) begin
			state        <= ST_IDLE;
			scl_reg      <= 1;
			sda_reg      <= 1;
			busy         <= 0;
			done         <= 0;
			read_data    <= 0;
			ack_received <= 1;
			shift_reg    <= 0;
			bit_cnt      <= 0;
		end else begin
			done <= 0;

			case (state)
				ST_IDLE: begin
					if (cmd_start) begin
						// START: SDA goes low while SCL is high
						state   <= ST_START1;
						sda_reg <= 1;  // Ensure SDA high first
						scl_reg <= 1;  // Ensure SCL high first
						busy    <= 1;
					end else if (cmd_stop) begin
						state   <= ST_STOP1;
						sda_reg <= 0;  // SDA low first
						scl_reg <= 0;  // SCL low
						busy    <= 1;
					end else if (cmd_write) begin
						state     <= ST_WRITE_BIT;
						shift_reg <= write_data;
						bit_cnt   <= 0;
						scl_reg   <= 0;
						busy      <= 1;
					end else if (cmd_read) begin
						state     <= ST_READ_BIT;
						shift_reg <= 0;
						bit_cnt   <= 0;
						scl_reg   <= 0;
						sda_reg   <= 1;  // Release SDA for slave to drive
						busy      <= 1;
					end
				end

				// START condition: SDA falls while SCL is high
				ST_START1: begin
					if (clk_tick) begin
						sda_reg <= 0;  // Pull SDA low
						state   <= ST_START2;
					end
				end

				ST_START2: begin
					if (clk_tick) begin
						scl_reg <= 0;  // Pull SCL low
						busy    <= 0;
						done    <= 1;
						state   <= ST_IDLE;
					end
				end

				// WRITE: shift out MSB first
				ST_WRITE_BIT: begin
					if (clk_tick) begin
						sda_reg <= shift_reg[7];
						state   <= ST_WRITE_CLK;
					end
				end

				ST_WRITE_CLK: begin
					if (clk_tick) begin
						scl_reg <= 1;  // Rising edge
						state   <= ST_WRITE_DONE;
					end
				end

				ST_WRITE_DONE: begin
					// Wait for clock stretching (SCL must be high)
					if (clk_tick && scl_i) begin
						scl_reg   <= 0;  // Falling edge
						shift_reg <= {shift_reg[6:0], 1'b0};
						bit_cnt   <= bit_cnt + 1;
						if (bit_cnt == 7) begin
							// Read ACK bit
							sda_reg <= 1;  // Release SDA for ACK
							state   <= ST_READ_BIT;
							bit_cnt <= 8;  // Special: reading ACK
						end else begin
							state <= ST_WRITE_BIT;
						end
					end
				end

				// READ: sample SDA on SCL high
				ST_READ_BIT: begin
					if (clk_tick) begin
						scl_reg <= 1;  // Rising edge
						state   <= ST_READ_CLK;
					end
				end

				ST_READ_CLK: begin
					// Wait for clock stretching
					if (clk_tick && scl_i) begin
						if (bit_cnt == 8) begin
							// This is the ACK bit after a write
							ack_received <= sda_i;
							scl_reg      <= 0;
							busy         <= 0;
							done         <= 1;
							state        <= ST_IDLE;
						end else begin
							shift_reg <= {shift_reg[6:0], sda_i};
							scl_reg   <= 0;
							bit_cnt   <= bit_cnt + 1;
							if (bit_cnt == 7) begin
								// All 8 bits read, now send ACK/NACK
								state <= ST_READ_DONE;
							end else begin
								state <= ST_READ_BIT;
							end
						end
					end
				end

				ST_READ_DONE: begin
					// Send ACK or NACK
					read_data <= {shift_reg[6:0], sda_i};
					sda_reg   <= cmd_ack_val;  // 0 = ACK, 1 = NACK
					state     <= ST_ACK_SEND;
				end

				ST_ACK_SEND: begin
					if (clk_tick) begin
						scl_reg <= 1;  // Rising edge for ACK clock
						state   <= ST_ACK_CLK;
					end
				end

				ST_ACK_CLK: begin
					if (clk_tick && scl_i) begin
						scl_reg <= 0;
						sda_reg <= 1;  // Release SDA
						busy    <= 0;
						done    <= 1;
						state   <= ST_IDLE;
					end
				end

				// STOP condition: SDA rises while SCL is high
				ST_STOP1: begin
					if (clk_tick) begin
						scl_reg <= 1;  // SCL goes high first
						state   <= ST_STOP2;
					end
				end

				ST_STOP2: begin
					if (clk_tick && scl_i) begin
						sda_reg <= 1;  // SDA goes high = STOP
						state   <= ST_STOP3;
					end
				end

				ST_STOP3: begin
					if (clk_tick) begin
						busy <= 0;
						done <= 1;
						state <= ST_IDLE;
					end
				end

				default: begin
					state <= ST_IDLE;
				end
			endcase
		end
	end

endmodule
