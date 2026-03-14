/*
 * rs485_wb.v - RS-485 UART with SoC bus interface
 *
 * RS-485 half-duplex UART with automatic direction control.
 * Wraps the existing uart_tx and uart_rx cores with an RS-485
 * transceiver driver enable signal.
 *
 * The DE (driver enable) pin is automatically asserted during
 * transmission and deasserted after the last stop bit, with a
 * configurable turnaround delay.
 *
 * Register map (32-bit aligned):
 *   0x00 - DATA:
 *     Write: [7:0]=TX byte (starts transmission, blocks if FIFO full)
 *     Read:  [7:0]=RX byte, [31]=1 if RX FIFO was empty (no data)
 *   0x04 - CTRL/STATUS:
 *     Write: [DIV_WIDTH-1:0]=Baud divisor
 *     Read:  [DIV_WIDTH-1:0]=Baud divisor, [28]=TX FIFO full,
 *            [29]=TX FIFO empty, [30]=RX overflow, [31]=RX FIFO empty
 *   0x08 - CONFIG:
 *     Write/Read: [7:0]=Turnaround delay (in bit periods after TX complete),
 *                 [8]=DE polarity (0=active high, 1=active low)
 *
 * Baud rate = 48MHz / (DIV + 2)
 *
 * Copyright (C) 2019  Hackaday Supercon Badge Contributors
 * All rights reserved.
 *
 * BSD 3-clause, see LICENSE.bsd
 */

`default_nettype none

module rs485_wb #(
	parameter integer FIFO_DEPTH = 16,
	parameter integer DIV_WIDTH = 12
)(
	// RS-485 physical interface
	output wire        rs485_tx,     // UART TX data line -> to RS-485 DI
	input  wire        rs485_rx,     // UART RX data line <- from RS-485 RO
	output wire        rs485_de,     // Driver Enable -> to RS-485 DE pin

	// Bus interface
	input  wire [3:2]  bus_addr,
	input  wire [31:0] bus_wdata,
	output wire [31:0] bus_rdata,
	input  wire        bus_cyc,
	output wire        bus_ack,
	input  wire        bus_we,

	// Interrupt
	output wire        irq,

	// Clock / Reset
	input  wire        clk,
	input  wire        rst
);

	// -------------------------------------------------------
	// Internal signals
	// -------------------------------------------------------

	// RX FIFO
	wire [7:0]  urf_wdata;
	wire        urf_wren;
	wire        urf_full;
	wire [7:0]  urf_rdata;
	wire        urf_rden;
	wire        urf_empty;
	reg         urf_overflow;
	wire        urf_overflow_clr;

	// TX FIFO
	wire [7:0]  utf_wdata;
	wire        utf_wren;
	wire        utf_full;
	wire [7:0]  utf_rdata;
	wire        utf_rden;
	wire        utf_empty;

	// TX core
	wire [7:0]  uart_tx_data;
	wire        uart_tx_valid;
	wire        uart_tx_ack;

	// RX core
	wire [7:0]  uart_rx_data;
	wire        uart_rx_stb;

	// CSR
	reg [DIV_WIDTH-1:0] uart_div;
	reg [7:0]   turnaround_delay;
	reg         de_polarity;

	// Bus IF
	wire        ub_rdata_rst;
	reg [31:0]  ub_rdata;
	reg         ub_rd_data;
	reg         ub_rd_ctrl;
	reg         ub_wr_data;
	reg         ub_wr_div;
	reg         ub_wr_config;
	reg         ub_ack;

	// DE control
	reg         de_active;
	reg [7:0]   de_delay_cnt;
	reg         tx_was_active;

	// -------------------------------------------------------
	// TX Core (reuse existing uart_tx)
	// -------------------------------------------------------

	uart_tx #(
		.DIV_WIDTH(DIV_WIDTH)
	) uart_tx_I (
		.data(uart_tx_data),
		.valid(uart_tx_valid),
		.ack(uart_tx_ack),
		.tx(rs485_tx),
		.div(uart_div),
		.clk(clk),
		.rst(rst)
	);

	// TX FIFO
	fifo_sync_ram #(
		.DEPTH(FIFO_DEPTH),
		.WIDTH(8)
	) uart_tx_fifo_I (
		.wr_data(utf_wdata),
		.wr_ena(utf_wren),
		.wr_full(utf_full),
		.rd_data(utf_rdata),
		.rd_ena(utf_rden),
		.rd_empty(utf_empty),
		.clk(clk),
		.rst(rst)
	);

	assign uart_tx_data  = utf_rdata;
	assign uart_tx_valid = ~utf_empty;
	assign utf_rden      = uart_tx_ack;

	// -------------------------------------------------------
	// RX Core (reuse existing uart_rx)
	// -------------------------------------------------------

	uart_rx #(
		.DIV_WIDTH(DIV_WIDTH),
		.GLITCH_FILTER(2)
	) uart_rx_I (
		.rx(rs485_rx),
		.data(uart_rx_data),
		.stb(uart_rx_stb),
		.div(uart_div),
		.clk(clk),
		.rst(rst)
	);

	// RX FIFO
	fifo_sync_ram #(
		.DEPTH(FIFO_DEPTH),
		.WIDTH(8)
	) uart_rx_fifo_I (
		.wr_data(urf_wdata),
		.wr_ena(urf_wren),
		.wr_full(urf_full),
		.rd_data(urf_rdata),
		.rd_ena(urf_rden),
		.rd_empty(urf_empty),
		.clk(clk),
		.rst(rst)
	);

	assign urf_wdata = uart_rx_data;
	// Only accept RX data when DE is not active (half-duplex: ignore echo)
	assign urf_wren  = uart_rx_stb & ~urf_full & ~de_active;

	always @(posedge clk or posedge rst)
		if (rst)
			urf_overflow <= 1'b0;
		else
			urf_overflow <= (urf_overflow & ~urf_overflow_clr) | (uart_rx_stb & urf_full & ~de_active);

	// IRQ on RX data available
	assign irq = uart_rx_stb & ~de_active;

	// -------------------------------------------------------
	// RS-485 Driver Enable (DE) control
	// -------------------------------------------------------
	// DE is asserted when TX FIFO has data or TX is active.
	// After TX completes and FIFO empties, DE stays asserted
	// for turnaround_delay bit periods before deasserting.

	wire tx_active = ~utf_empty | uart_tx_valid;

	always @(posedge clk) begin
		if (rst) begin
			de_active    <= 0;
			de_delay_cnt <= 0;
			tx_was_active <= 0;
		end else begin
			tx_was_active <= tx_active;

			if (tx_active) begin
				de_active    <= 1;
				de_delay_cnt <= turnaround_delay;
			end else if (de_active && !tx_active) begin
				// TX finished, count down turnaround delay
				if (de_delay_cnt == 0) begin
					de_active <= 0;
				end else begin
					de_delay_cnt <= de_delay_cnt - 1;
				end
			end
		end
	end

	assign rs485_de = de_active ^ de_polarity;

	// -------------------------------------------------------
	// Bus interface (follows same pattern as uart_wb)
	// -------------------------------------------------------

	always @(posedge clk)
		if (ub_ack) begin
			ub_rd_data  <= 1'b0;
			ub_rd_ctrl  <= 1'b0;
			ub_wr_data  <= 1'b0;
			ub_wr_div   <= 1'b0;
			ub_wr_config <= 1'b0;
		end else begin
			ub_rd_data   <= ~bus_we & bus_cyc & (bus_addr == 2'b00);
			ub_rd_ctrl   <= ~bus_we & bus_cyc & (bus_addr == 2'b01);
			ub_wr_data   <=  bus_we & bus_cyc & (bus_addr == 2'b00) & ~utf_full;
			ub_wr_div    <=  bus_we & bus_cyc & (bus_addr == 2'b01);
			ub_wr_config <=  bus_we & bus_cyc & (bus_addr == 2'b10);
		end

	always @(posedge clk)
		if (ub_ack)
			ub_ack <= 1'b0;
		else
			ub_ack <= bus_cyc & (~bus_we | (bus_addr != 2'b00) | ~utf_full);

	assign ub_rdata_rst = ub_ack | bus_we | ~bus_cyc;

	always @(posedge clk)
		if (ub_rdata_rst)
			ub_rdata <= 32'b0;
		else begin
			case (bus_addr)
				2'b00: ub_rdata <= {urf_empty, {23{1'b0}}, urf_rdata};
				2'b01: ub_rdata <= {urf_empty, urf_overflow, utf_empty, utf_full,
				                    {(28-DIV_WIDTH){1'b0}}, uart_div};
				2'b10: ub_rdata <= {23'b0, de_polarity, turnaround_delay};
				default: ub_rdata <= 32'b0;
			endcase
		end

	// Write registers
	always @(posedge clk)
		if (rst) begin
			uart_div         <= 0;
			turnaround_delay <= 8'd2;
			de_polarity      <= 0;
		end else begin
			if (ub_wr_div)
				uart_div <= bus_wdata[DIV_WIDTH-1:0];
			if (ub_wr_config) begin
				turnaround_delay <= bus_wdata[7:0];
				de_polarity      <= bus_wdata[8];
			end
		end

	assign utf_wdata = bus_wdata[7:0];
	assign utf_wren  = ub_wr_data;

	assign urf_rden  = ub_rd_data & ~ub_rdata[31];
	assign urf_overflow_clr = ub_rd_ctrl & ub_rdata[30];

	assign bus_rdata = ub_rdata;
	assign bus_ack = ub_ack;

endmodule
