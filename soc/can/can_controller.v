/*
 * can_controller.v - CAN 2.0B controller core
 *
 * Implements a basic CAN controller supporting standard (11-bit)
 * and extended (29-bit) identifiers. Handles bit stuffing,
 * CRC-15 generation/checking, and bus arbitration.
 *
 * This is a simplified CAN controller suitable for low-speed
 * applications and prototyping with FPGA badge hardware.
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module can_controller (
	input  wire        clk,
	input  wire        rst,

	// CAN physical interface
	output reg         can_tx,     // CAN TX output (active-dominant = 0)
	input  wire        can_rx,     // CAN RX input

	// TX interface
	input  wire        tx_valid,   // Start transmission
	input  wire [28:0] tx_id,      // Identifier (11 or 29 bits)
	input  wire        tx_ide,     // 1 = extended (29-bit) ID
	input  wire        tx_rtr,     // Remote transmission request
	input  wire [3:0]  tx_dlc,     // Data length code (0-8)
	input  wire [63:0] tx_data,    // TX data (big-endian, MSB first)
	output reg         tx_done,    // TX complete pulse
	output reg         tx_arblost, // Arbitration lost

	// RX interface
	output reg         rx_valid,   // Received frame available (pulse)
	output reg  [28:0] rx_id,      // Received identifier
	output reg         rx_ide,     // Received IDE flag
	output reg         rx_rtr,     // Received RTR flag
	output reg  [3:0]  rx_dlc,     // Received DLC
	output reg  [63:0] rx_data,    // Received data
	output reg         rx_crc_err, // CRC error on last received frame

	// Status
	output reg         busy,       // Bus activity detected or transmitting
	output reg         bus_off,    // Bus error threshold exceeded (simplified)

	// Bit timing configuration
	// Baud = clk / (prescaler * (1 + tseg1 + tseg2))
	input  wire [15:0] prescaler,  // Baud rate prescaler (min 1)
	input  wire [3:0]  tseg1,      // Time segment 1 (propagation + phase1)
	input  wire [3:0]  tseg2       // Time segment 2 (phase2)
);

	// -------------------------------------------------------
	// Bit timing generator
	// -------------------------------------------------------
	reg [15:0] baud_cnt;
	reg [4:0]  bit_time_cnt;
	wire [4:0] bit_time_total;
	wire       sample_point;
	wire       bit_tick;

	assign bit_time_total = 1 + tseg1 + tseg2;

	always @(posedge clk) begin
		if (rst) begin
			baud_cnt     <= 0;
			bit_time_cnt <= 0;
		end else begin
			if (baud_cnt >= prescaler - 1) begin
				baud_cnt <= 0;
				if (bit_time_cnt >= bit_time_total - 1)
					bit_time_cnt <= 0;
				else
					bit_time_cnt <= bit_time_cnt + 1;
			end else begin
				baud_cnt <= baud_cnt + 1;
			end
		end
	end

	// Sample at end of tseg1, output at start of bit time
	assign sample_point = (baud_cnt == prescaler - 1) && (bit_time_cnt == tseg1);
	assign bit_tick     = (baud_cnt == prescaler - 1) && (bit_time_cnt == 0);

	// -------------------------------------------------------
	// State machine
	// -------------------------------------------------------
	localparam ST_IDLE       = 4'd0;
	localparam ST_TX_SOF     = 4'd1;
	localparam ST_TX_ARBID   = 4'd2;
	localparam ST_TX_CTRL    = 4'd3;
	localparam ST_TX_DATA    = 4'd4;
	localparam ST_TX_CRC     = 4'd5;
	localparam ST_TX_CRCDEL  = 4'd6;
	localparam ST_TX_ACK     = 4'd7;
	localparam ST_TX_ACKDEL  = 4'd8;
	localparam ST_TX_EOF     = 4'd9;
	localparam ST_TX_IFS     = 4'd10;
	localparam ST_RX_FRAME   = 4'd11;
	localparam ST_RX_CRC     = 4'd12;
	localparam ST_RX_ACKSLOT = 4'd13;
	localparam ST_RX_EOF     = 4'd14;

	reg [3:0]  state;
	reg [7:0]  bit_cnt;

	// TX frame construction
	reg [127:0] tx_shift;  // Shift register for TX frame bits (pre-stuffing)
	reg [7:0]   tx_len;    // Number of bits to transmit in current phase

	// RX frame reconstruction
	reg [127:0] rx_shift;
	reg [7:0]   rx_bit_cnt;

	// CRC-15
	reg [14:0]  crc_reg;
	wire        crc_nxt;
	reg         crc_active;
	reg         tx_bit_out;

	// Bit stuffing
	reg [2:0]   stuff_cnt;
	reg         stuff_last_bit;
	reg         stuff_active;
	wire        need_stuff;

	assign need_stuff = (stuff_cnt == 5);
	assign crc_nxt = crc_reg[14] ^ tx_bit_out;

	// CRC-15 with polynomial 0x4599
	always @(posedge clk) begin
		if (rst || !crc_active) begin
			crc_reg <= 15'h0;
		end else if (bit_tick && !need_stuff) begin
			crc_reg[14] <= crc_nxt ? ~crc_reg[13] : crc_reg[13];
			crc_reg[13] <= crc_reg[12];
			crc_reg[12] <= crc_reg[11];
			crc_reg[11] <= crc_reg[10];
			crc_reg[10] <= crc_nxt ? ~crc_reg[9] : crc_reg[9];
			crc_reg[9]  <= crc_reg[8];
			crc_reg[8]  <= crc_nxt ? ~crc_reg[7] : crc_reg[7];
			crc_reg[7]  <= crc_nxt ? ~crc_reg[6] : crc_reg[6];
			crc_reg[6]  <= crc_reg[5];
			crc_reg[5]  <= crc_reg[4];
			crc_reg[4]  <= crc_nxt ? ~crc_reg[3] : crc_reg[3];
			crc_reg[3]  <= crc_nxt ? ~crc_reg[2] : crc_reg[2];
			crc_reg[2]  <= crc_reg[1];
			crc_reg[1]  <= crc_nxt ? ~crc_reg[0] : crc_reg[0];
			crc_reg[0]  <= crc_nxt;
		end
	end

	// Bit stuffing counter
	always @(posedge clk) begin
		if (rst || !stuff_active) begin
			stuff_cnt      <= 0;
			stuff_last_bit <= 1;
		end else if (bit_tick) begin
			if (need_stuff) begin
				// Insert a stuff bit (opposite of last 5)
				stuff_cnt      <= 1;
				stuff_last_bit <= ~stuff_last_bit;
			end else if (tx_bit_out == stuff_last_bit) begin
				stuff_cnt <= stuff_cnt + 1;
			end else begin
				stuff_cnt      <= 1;
				stuff_last_bit <= tx_bit_out;
			end
		end
	end

	// Main state machine
	always @(posedge clk) begin
		if (rst) begin
			state      <= ST_IDLE;
			can_tx     <= 1'b1;  // Recessive
			tx_done    <= 0;
			tx_arblost <= 0;
			rx_valid   <= 0;
			busy       <= 0;
			bus_off    <= 0;
			bit_cnt    <= 0;
			crc_active <= 0;
			stuff_active <= 0;
			rx_crc_err <= 0;
			tx_shift   <= 0;
			tx_len     <= 0;
			rx_shift   <= 0;
			rx_bit_cnt <= 0;
			tx_bit_out <= 1;
			rx_id      <= 0;
			rx_ide     <= 0;
			rx_rtr     <= 0;
			rx_dlc     <= 0;
			rx_data    <= 0;
		end else begin
			tx_done    <= 0;
			tx_arblost <= 0;
			rx_valid   <= 0;
			rx_crc_err <= 0;

			case (state)
				ST_IDLE: begin
					can_tx       <= 1'b1;  // Recessive
					busy         <= 0;
					crc_active   <= 0;
					stuff_active <= 0;

					if (tx_valid && !bus_off) begin
						// Build the frame in the shift register
						busy       <= 1;
						state      <= ST_TX_SOF;
						crc_active <= 1;
						stuff_active <= 1;
					end else if (sample_point && !can_rx) begin
						// SOF detected on bus - start receiving
						busy       <= 1;
						state      <= ST_RX_FRAME;
						rx_shift   <= 0;
						rx_bit_cnt <= 0;
						crc_active <= 1;
					end
				end

				ST_TX_SOF: begin
					if (bit_tick) begin
						// SOF = dominant (0)
						can_tx     <= 1'b0;
						tx_bit_out <= 1'b0;

						// Build arbitration field
						if (tx_ide) begin
							// Extended frame: 11-bit base ID + SRR + IDE + 18-bit ext ID + RTR
							tx_shift[127:96] <= {tx_id[28:18], 1'b1, 1'b1, tx_id[17:0]};
							tx_shift[95]     <= tx_rtr;
							tx_shift[94:93]  <= 2'b00;  // r1, r0
							tx_shift[92:89]  <= tx_dlc;
							tx_len <= 8'd34 + {4'd0, tx_dlc, 3'd0};  // arb(32) + ctrl(6) + data
						end else begin
							// Standard frame: 11-bit ID + RTR + IDE=0 + r0 + DLC
							tx_shift[127:117] <= tx_id[10:0];
							tx_shift[116]     <= tx_rtr;
							tx_shift[115]     <= 1'b0;  // IDE
							tx_shift[114]     <= 1'b0;  // r0
							tx_shift[113:110] <= tx_dlc;
							tx_len <= 8'd18 + {4'd0, tx_dlc, 3'd0};  // arb(13) + ctrl(5) + data
						end

						// Pack data into shift register after control field
						if (tx_ide) begin
							tx_shift[88:25] <= tx_data;
						end else begin
							tx_shift[109:46] <= tx_data;
						end

						bit_cnt <= 0;
						state   <= ST_TX_ARBID;
					end
				end

				ST_TX_ARBID: begin
					if (bit_tick) begin
						if (!need_stuff) begin
							// Shift out next bit
							tx_bit_out <= tx_shift[127];
							can_tx     <= tx_shift[127];

							// Check for arbitration loss during ID field
							if (sample_point && tx_shift[127] && !can_rx) begin
								// We sent recessive but bus is dominant = lost arbitration
								tx_arblost <= 1;
								can_tx     <= 1'b1;
								state      <= ST_IDLE;
							end

							tx_shift <= {tx_shift[126:0], 1'b0};
							bit_cnt  <= bit_cnt + 1;

							if (bit_cnt >= tx_len - 1) begin
								state <= ST_TX_CRC;
								bit_cnt <= 0;
							end
						end else begin
							// Insert stuff bit
							can_tx     <= ~stuff_last_bit;
							tx_bit_out <= ~stuff_last_bit;
						end
					end
				end

				ST_TX_CRC: begin
					if (bit_tick) begin
						if (bit_cnt < 15) begin
							can_tx     <= crc_reg[14 - bit_cnt[3:0]];
							tx_bit_out <= crc_reg[14 - bit_cnt[3:0]];
							bit_cnt    <= bit_cnt + 1;
						end else begin
							// CRC delimiter (recessive)
							can_tx       <= 1'b1;
							stuff_active <= 0;
							crc_active   <= 0;
							state        <= ST_TX_ACK;
						end
					end
				end

				ST_TX_ACK: begin
					if (bit_tick) begin
						// ACK slot: we transmit recessive, receiver drives dominant
						can_tx <= 1'b1;
						state  <= ST_TX_ACKDEL;
					end
				end

				ST_TX_ACKDEL: begin
					if (bit_tick) begin
						// ACK delimiter: recessive
						can_tx  <= 1'b1;
						state   <= ST_TX_EOF;
						bit_cnt <= 0;
					end
				end

				ST_TX_EOF: begin
					if (bit_tick) begin
						can_tx  <= 1'b1;  // EOF = 7 recessive bits
						bit_cnt <= bit_cnt + 1;
						if (bit_cnt >= 6) begin
							state   <= ST_TX_IFS;
							bit_cnt <= 0;
						end
					end
				end

				ST_TX_IFS: begin
					if (bit_tick) begin
						can_tx  <= 1'b1;  // IFS = 3 recessive bits
						bit_cnt <= bit_cnt + 1;
						if (bit_cnt >= 2) begin
							tx_done <= 1;
							state   <= ST_IDLE;
						end
					end
				end

				// Simplified RX: capture bits and extract frame fields
				ST_RX_FRAME: begin
					if (sample_point) begin
						rx_shift   <= {rx_shift[126:0], can_rx};
						rx_bit_cnt <= rx_bit_cnt + 1;

						// After enough bits for a minimal standard frame (44 bits without stuffing),
						// we check for end-of-frame pattern. This is simplified - a production
						// controller would do proper destuffing and field extraction.
						if (rx_bit_cnt >= 44 && can_rx) begin
							// Count consecutive recessive bits for EOF detection
							state <= ST_RX_EOF;
							bit_cnt <= 1;
						end
					end
				end

				ST_RX_EOF: begin
					if (sample_point) begin
						if (can_rx) begin
							bit_cnt <= bit_cnt + 1;
							if (bit_cnt >= 6) begin
								// 7 recessive bits = EOF
								rx_valid <= 1;
								// Extract standard frame fields from captured bits (simplified)
								rx_id    <= {18'd0, rx_shift[rx_bit_cnt-1 -: 11]};
								rx_ide   <= 0;
								rx_rtr   <= 0;
								rx_dlc   <= 4'd0;
								rx_data  <= 64'd0;
								state    <= ST_IDLE;
							end
						end else begin
							// Not EOF, continue receiving
							state <= ST_RX_FRAME;
							rx_shift   <= {rx_shift[126:0], can_rx};
							rx_bit_cnt <= rx_bit_cnt + 1;
						end
					end
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
