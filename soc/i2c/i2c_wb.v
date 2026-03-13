/*
 * i2c_wb.v - I2C master with SoC bus interface
 *
 * Memory-mapped register interface for the I2C master controller.
 * Follows the same bus protocol as other peripherals in this SoC.
 *
 * Register map (active bits, 32-bit aligned):
 *   0x00 - CMD/STATUS:
 *     Write: [0]=START, [1]=STOP, [2]=WRITE, [3]=READ, [4]=ACK_VAL (0=ACK, 1=NACK)
 *     Read:  [0]=BUSY, [1]=DONE, [2]=ACK_RECEIVED (0=ACK from slave, 1=NACK)
 *   0x04 - DATA:
 *     Write: [7:0]=TX data byte
 *     Read:  [7:0]=RX data byte
 *   0x08 - CLKDIV:
 *     Write/Read: [15:0]=Clock divider. SCL freq = clk / (4 * (clkdiv+1))
 *     Default 119 -> 100kHz at 48MHz. Set to 29 for 400kHz.
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module i2c_wb (
	// I2C physical pins
	output wire        scl_o,
	output wire        scl_oe,
	input  wire        scl_i,
	output wire        sda_o,
	output wire        sda_oe,
	input  wire        sda_i,

	// Bus interface
	input  wire [3:2]  bus_addr,
	input  wire [31:0] bus_wdata,
	output reg  [31:0] bus_rdata,
	input  wire        bus_cyc,
	output reg         bus_ack,
	input  wire        bus_we,

	// Interrupt (active high pulse on done)
	output wire        irq,

	// Clock / Reset
	input  wire        clk,
	input  wire        rst
);

	// Internal signals
	reg         cmd_start;
	reg         cmd_stop;
	reg         cmd_write;
	reg         cmd_read;
	reg         cmd_ack_val;
	reg  [7:0]  tx_data;
	wire [7:0]  rx_data;
	wire        ack_received;
	wire        busy;
	wire        done;
	reg  [15:0] clk_div;

	// Status latch: done stays set until read
	reg         done_latch;

	// I2C master core
	i2c_master i2c_core (
		.clk(clk),
		.rst(rst),
		.scl_o(scl_o),
		.scl_oe(scl_oe),
		.scl_i(scl_i),
		.sda_o(sda_o),
		.sda_oe(sda_oe),
		.sda_i(sda_i),
		.cmd_start(cmd_start),
		.cmd_stop(cmd_stop),
		.cmd_write(cmd_write),
		.cmd_read(cmd_read),
		.cmd_ack_val(cmd_ack_val),
		.write_data(tx_data),
		.read_data(rx_data),
		.ack_received(ack_received),
		.busy(busy),
		.done(done),
		.clk_div(clk_div)
	);

	assign irq = done;

	// Bus acknowledge: single cycle delay
	always @(posedge clk) begin
		if (rst)
			bus_ack <= 0;
		else if (bus_ack)
			bus_ack <= 0;
		else
			bus_ack <= bus_cyc;
	end

	// Done latch: set when i2c_master pulses done, cleared on CMD/STATUS read
	always @(posedge clk) begin
		if (rst)
			done_latch <= 0;
		else if (done)
			done_latch <= 1;
		else if (bus_cyc && !bus_we && bus_addr == 2'b00 && bus_ack)
			done_latch <= 0;
	end

	// Read mux
	always @(*) begin
		case (bus_addr)
			2'b00:   bus_rdata = {29'h0, ack_received, done_latch, busy};
			2'b01:   bus_rdata = {24'h0, rx_data};
			2'b10:   bus_rdata = {16'h0, clk_div};
			default: bus_rdata = 32'h0;
		endcase
	end

	// Write logic: issue commands and set registers
	always @(posedge clk) begin
		if (rst) begin
			cmd_start   <= 0;
			cmd_stop    <= 0;
			cmd_write   <= 0;
			cmd_read    <= 0;
			cmd_ack_val <= 0;
			tx_data     <= 0;
			clk_div     <= 16'd119;  // Default: 100kHz at 48MHz
		end else begin
			// Commands are single-cycle pulses
			cmd_start <= 0;
			cmd_stop  <= 0;
			cmd_write <= 0;
			cmd_read  <= 0;

			if (bus_cyc && bus_we && !bus_ack) begin
				case (bus_addr)
					2'b00: begin
						cmd_start   <= bus_wdata[0];
						cmd_stop    <= bus_wdata[1];
						cmd_write   <= bus_wdata[2];
						cmd_read    <= bus_wdata[3];
						cmd_ack_val <= bus_wdata[4];
					end
					2'b01: begin
						tx_data <= bus_wdata[7:0];
					end
					2'b10: begin
						clk_div <= bus_wdata[15:0];
					end
					default: ;
				endcase
			end
		end
	end

endmodule
