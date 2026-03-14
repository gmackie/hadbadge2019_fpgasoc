/*
 * can_wb.v - CAN controller with SoC bus interface
 *
 * Memory-mapped register interface for the CAN controller.
 *
 * Register map (32-bit aligned):
 *   0x00 - CMD/STATUS:
 *     Write: [0]=TX_START
 *     Read:  [0]=BUSY, [1]=TX_DONE, [2]=TX_ARBLOST, [3]=RX_VALID,
 *            [4]=RX_CRC_ERR, [5]=BUS_OFF
 *   0x04 - TX_ID:
 *     Write/Read: [28:0]=Identifier
 *   0x08 - TX_CTRL:
 *     Write/Read: [3:0]=DLC, [4]=RTR, [5]=IDE
 *   0x0C - TX_DATA0:
 *     Write/Read: [31:0]=TX data bytes 0-3 (byte 0 = bits [31:24])
 *   0x10 - TX_DATA1:
 *     Write/Read: [31:0]=TX data bytes 4-7 (byte 4 = bits [31:24])
 *   0x14 - RX_ID:
 *     Read: [28:0]=Received identifier
 *   0x18 - RX_CTRL:
 *     Read: [3:0]=DLC, [4]=RTR, [5]=IDE
 *   0x1C - RX_DATA0:
 *     Read: [31:0]=RX data bytes 0-3
 *   0x20 - RX_DATA1:
 *     Read: [31:0]=RX data bytes 4-7
 *   0x24 - PRESCALER:
 *     Write/Read: [15:0]=Baud rate prescaler
 *   0x28 - TIMING:
 *     Write/Read: [3:0]=TSEG1, [7:4]=TSEG2
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module can_wb (
	// CAN physical pins
	output wire        can_tx,
	input  wire        can_rx,

	// Bus interface
	input  wire [5:2]  bus_addr,
	input  wire [31:0] bus_wdata,
	output reg  [31:0] bus_rdata,
	input  wire        bus_cyc,
	output reg         bus_ack,
	input  wire        bus_we,

	// Interrupts
	output wire        irq_tx,
	output wire        irq_rx,

	// Clock / Reset
	input  wire        clk,
	input  wire        rst
);

	// TX registers
	reg  [28:0] tx_id;
	reg         tx_ide;
	reg         tx_rtr;
	reg  [3:0]  tx_dlc;
	reg  [63:0] tx_data;
	reg         tx_start;

	// CAN core signals
	wire        tx_done;
	wire        tx_arblost;
	wire        rx_valid;
	wire [28:0] rx_id;
	wire        rx_ide;
	wire        rx_rtr;
	wire [3:0]  rx_dlc;
	wire [63:0] rx_data;
	wire        rx_crc_err;
	wire        busy;
	wire        bus_off;

	// Configuration
	reg  [15:0] prescaler;
	reg  [3:0]  tseg1;
	reg  [3:0]  tseg2;

	// Status latches
	reg         tx_done_latch;
	reg         tx_arblost_latch;
	reg         rx_valid_latch;
	reg         rx_crc_err_latch;

	// RX buffer
	reg  [28:0] rx_id_buf;
	reg         rx_ide_buf;
	reg         rx_rtr_buf;
	reg  [3:0]  rx_dlc_buf;
	reg  [63:0] rx_data_buf;

	can_controller can_core (
		.clk(clk),
		.rst(rst),
		.can_tx(can_tx),
		.can_rx(can_rx),
		.tx_valid(tx_start),
		.tx_id(tx_id),
		.tx_ide(tx_ide),
		.tx_rtr(tx_rtr),
		.tx_dlc(tx_dlc),
		.tx_data(tx_data),
		.tx_done(tx_done),
		.tx_arblost(tx_arblost),
		.rx_valid(rx_valid),
		.rx_id(rx_id),
		.rx_ide(rx_ide),
		.rx_rtr(rx_rtr),
		.rx_dlc(rx_dlc),
		.rx_data(rx_data),
		.rx_crc_err(rx_crc_err),
		.busy(busy),
		.bus_off(bus_off),
		.prescaler(prescaler),
		.tseg1(tseg1),
		.tseg2(tseg2)
	);

	assign irq_tx = tx_done;
	assign irq_rx = rx_valid;

	// Bus acknowledge
	always @(posedge clk) begin
		if (rst)
			bus_ack <= 0;
		else if (bus_ack)
			bus_ack <= 0;
		else
			bus_ack <= bus_cyc;
	end

	// Status latches
	always @(posedge clk) begin
		if (rst) begin
			tx_done_latch    <= 0;
			tx_arblost_latch <= 0;
			rx_valid_latch   <= 0;
			rx_crc_err_latch <= 0;
		end else begin
			if (tx_done)    tx_done_latch    <= 1;
			if (tx_arblost) tx_arblost_latch <= 1;
			if (rx_valid)   rx_valid_latch   <= 1;
			if (rx_crc_err) rx_crc_err_latch <= 1;
			// Clear on CMD/STATUS read
			if (bus_cyc && !bus_we && bus_addr == 4'b0000 && bus_ack) begin
				tx_done_latch    <= 0;
				tx_arblost_latch <= 0;
				rx_valid_latch   <= 0;
				rx_crc_err_latch <= 0;
			end
		end
	end

	// RX buffer: latch received frame
	always @(posedge clk) begin
		if (rst) begin
			rx_id_buf   <= 0;
			rx_ide_buf  <= 0;
			rx_rtr_buf  <= 0;
			rx_dlc_buf  <= 0;
			rx_data_buf <= 0;
		end else if (rx_valid) begin
			rx_id_buf   <= rx_id;
			rx_ide_buf  <= rx_ide;
			rx_rtr_buf  <= rx_rtr;
			rx_dlc_buf  <= rx_dlc;
			rx_data_buf <= rx_data;
		end
	end

	// Read mux
	always @(*) begin
		case (bus_addr)
			4'b0000: bus_rdata = {26'h0, bus_off, rx_crc_err_latch, rx_valid_latch,
			                      tx_arblost_latch, tx_done_latch, busy};
			4'b0001: bus_rdata = {3'h0, tx_id};
			4'b0010: bus_rdata = {26'h0, tx_ide, tx_rtr, tx_dlc};
			4'b0011: bus_rdata = tx_data[63:32];
			4'b0100: bus_rdata = tx_data[31:0];
			4'b0101: bus_rdata = {3'h0, rx_id_buf};
			4'b0110: bus_rdata = {26'h0, rx_ide_buf, rx_rtr_buf, rx_dlc_buf};
			4'b0111: bus_rdata = rx_data_buf[63:32];
			4'b1000: bus_rdata = rx_data_buf[31:0];
			4'b1001: bus_rdata = {16'h0, prescaler};
			4'b1010: bus_rdata = {24'h0, tseg2, tseg1};
			default: bus_rdata = 32'h0;
		endcase
	end

	// Write logic
	always @(posedge clk) begin
		if (rst) begin
			tx_start  <= 0;
			tx_id     <= 0;
			tx_ide    <= 0;
			tx_rtr    <= 0;
			tx_dlc    <= 0;
			tx_data   <= 0;
			prescaler <= 16'd48;   // Default: 1Mbps at 48MHz with tseg1=5,tseg2=4 -> 48/(48*(1+5+4))=100kbps
			tseg1     <= 4'd5;
			tseg2     <= 4'd4;
		end else begin
			tx_start <= 0;  // Single-cycle pulse

			if (bus_cyc && bus_we && !bus_ack) begin
				case (bus_addr)
					4'b0000: tx_start          <= bus_wdata[0];
					4'b0001: tx_id             <= bus_wdata[28:0];
					4'b0010: begin
						tx_dlc <= bus_wdata[3:0];
						tx_rtr <= bus_wdata[4];
						tx_ide <= bus_wdata[5];
					end
					4'b0011: tx_data[63:32]    <= bus_wdata;
					4'b0100: tx_data[31:0]     <= bus_wdata;
					4'b1001: prescaler         <= bus_wdata[15:0];
					4'b1010: begin
						tseg1 <= bus_wdata[3:0];
						tseg2 <= bus_wdata[7:4];
					end
					default: ;
				endcase
			end
		end
	end

endmodule
