/*
 * spi_master.v - SPI master core
 *
 * Configurable SPI master supporting modes 0-3 (CPOL/CPHA),
 * 8-bit full-duplex transfers, configurable clock divider,
 * and multiple chip selects.
 *
 * SCL frequency = clk / (2 * (clk_div + 1))
 * At 48MHz with clk_div=23: SCL = 1MHz
 * At 48MHz with clk_div=0:  SCL = 24MHz
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module spi_master (
	// SPI physical pins
	output reg         sck,
	output reg         mosi,
	input  wire        miso,
	output reg  [3:0]  cs_n,

	// Control interface
	input  wire [7:0]  tx_data,
	output reg  [7:0]  rx_data,
	input  wire        start,       // pulse to begin transfer
	output reg         busy,
	output reg         done,        // single-cycle pulse when transfer completes

	// Configuration
	input  wire [15:0] clk_div,     // SCL freq = clk / (2 * (clk_div + 1))
	input  wire        cpol,        // clock polarity
	input  wire        cpha,        // clock phase
	input  wire [3:0]  cs_mask,     // chip select mask (active bits pull CS low)

	// Clock / Reset
	input  wire        clk,
	input  wire        rst
);

	// State machine
	localparam S_IDLE = 2'd0;
	localparam S_TRANSFER = 2'd1;
	localparam S_DONE = 2'd2;

	reg [1:0]  state;
	reg [15:0] clk_cnt;        // clock divider counter
	reg [2:0]  bit_cnt;        // bit counter (0-7)
	reg [7:0]  shift_out;      // TX shift register
	reg [7:0]  shift_in;       // RX shift register
	reg        sck_internal;   // internal SCK phase tracking

	always @(posedge clk) begin
		if (rst) begin
			state        <= S_IDLE;
			sck          <= 1'b0;
			mosi         <= 1'b0;
			cs_n         <= 4'hF;
			rx_data      <= 8'h0;
			busy         <= 1'b0;
			done         <= 1'b0;
			clk_cnt      <= 16'h0;
			bit_cnt      <= 3'h0;
			shift_out    <= 8'h0;
			shift_in     <= 8'h0;
			sck_internal <= 1'b0;
		end else begin
			done <= 1'b0;

			case (state)
				S_IDLE: begin
					sck <= cpol;
					cs_n <= ~cs_mask;
					if (start) begin
						state        <= S_TRANSFER;
						busy         <= 1'b1;
						shift_out    <= tx_data;
						shift_in     <= 8'h0;
						bit_cnt      <= 3'h0;
						clk_cnt      <= 16'h0;
						sck_internal <= 1'b0;
						sck          <= cpol;
						// For CPHA=0, put first bit on MOSI immediately
						if (!cpha) begin
							mosi <= tx_data[7];
						end
					end
				end

				S_TRANSFER: begin
					if (clk_cnt == clk_div) begin
						clk_cnt <= 16'h0;
						sck_internal <= ~sck_internal;

						if (!sck_internal) begin
							// Leading edge of SCK
							sck <= ~cpol;
							if (cpha) begin
								// CPHA=1: shift out data on leading edge
								mosi <= shift_out[7];
							end else begin
								// CPHA=0: sample MISO on leading edge
								shift_in <= {shift_in[6:0], miso};
							end
						end else begin
							// Trailing edge of SCK
							sck <= cpol;
							if (cpha) begin
								// CPHA=1: sample MISO on trailing edge
								shift_in <= {shift_in[6:0], miso};
							end else begin
								// CPHA=0: shift out next bit on trailing edge
								shift_out <= {shift_out[6:0], 1'b0};
								mosi <= shift_out[6];
							end

							// One full bit period completed
							if (bit_cnt == 3'd7) begin
								state <= S_DONE;
							end else begin
								bit_cnt <= bit_cnt + 3'd1;
							end
						end
					end else begin
						clk_cnt <= clk_cnt + 16'd1;
					end
				end

				S_DONE: begin
					rx_data <= shift_in;
					busy    <= 1'b0;
					done    <= 1'b1;
					sck     <= cpol;
					state   <= S_IDLE;
				end

				default: begin
					state <= S_IDLE;
				end
			endcase
		end
	end

endmodule
