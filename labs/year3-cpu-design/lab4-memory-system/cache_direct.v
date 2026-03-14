/*
 * Lab 4: Memory System
 * ====================
 * Understand the memory hierarchy by building a cache.
 * The badge has 16MB PSRAM accessed through a QPI cache -
 * in this lab you'll build a simplified version.
 *
 * This lab connects your CPU to a realistic memory system:
 * - L1 cache: fast, small, SRAM-based (in FPGA block RAM)
 * - Main memory: slow, large (PSRAM with multi-cycle latency)
 *
 * EXERCISES:
 *
 * Exercise 1: Direct-mapped cache
 *   Complete the direct-mapped cache below. It should handle
 *   read hits, read misses (fetch from memory), and writes.
 *
 * Exercise 2: Cache performance analysis
 *   Run test programs with different access patterns:
 *   - Sequential access (good locality)
 *   - Stride access (varying strides)
 *   - Random access (poor locality)
 *   Count hits and misses. Calculate the hit rate.
 *
 * Exercise 3: Write policy
 *   Implement write-through and write-back policies.
 *   Compare the memory traffic for each.
 *
 * Exercise 4 (challenge): 2-way set-associative cache
 *   Extend to 2-way set associative with LRU replacement.
 *   Compare hit rates with direct-mapped on the same workloads.
 *
 * CONCEPTS:
 *   - Spatial and temporal locality
 *   - Tag, index, offset fields
 *   - Hit rate and miss penalty
 *   - Write-through vs write-back
 *   - Cache coherency (connects to the badge's cache_flush)
 */

`default_nettype none

module cache_direct #(
	parameter CACHE_SIZE_BYTES = 1024,   // Total cache size
	parameter LINE_SIZE_BYTES  = 16,     // Cache line size (4 words)
	parameter ADDR_WIDTH       = 32
)(
	input  wire                  clk,
	input  wire                  rst,

	// CPU interface
	input  wire [ADDR_WIDTH-1:0] cpu_addr,
	input  wire [31:0]           cpu_wdata,
	output reg  [31:0]           cpu_rdata,
	input  wire                  cpu_ren,
	input  wire [3:0]            cpu_wstrb,
	output reg                   cpu_ready,

	// Memory interface (slow, multi-cycle)
	output reg  [ADDR_WIDTH-1:0] mem_addr,
	output reg  [31:0]           mem_wdata,
	input  wire [31:0]           mem_rdata,
	output reg                   mem_ren,
	output reg  [3:0]            mem_wstrb,
	input  wire                  mem_ready,

	// Performance counters
	output reg  [31:0]           hit_count,
	output reg  [31:0]           miss_count
);

	// Cache geometry calculations
	localparam NUM_LINES     = CACHE_SIZE_BYTES / LINE_SIZE_BYTES;
	localparam WORDS_PER_LINE = LINE_SIZE_BYTES / 4;
	localparam OFFSET_BITS   = $clog2(LINE_SIZE_BYTES);
	localparam INDEX_BITS    = $clog2(NUM_LINES);
	localparam TAG_BITS      = ADDR_WIDTH - INDEX_BITS - OFFSET_BITS;

	// Extract address fields
	wire [OFFSET_BITS-1:0] addr_offset = cpu_addr[OFFSET_BITS-1:0];
	wire [INDEX_BITS-1:0]  addr_index  = cpu_addr[OFFSET_BITS +: INDEX_BITS];
	wire [TAG_BITS-1:0]    addr_tag    = cpu_addr[ADDR_WIDTH-1 -: TAG_BITS];
	wire [1:0]             word_sel    = addr_offset[OFFSET_BITS-1:2];

	// Cache storage
	reg [TAG_BITS-1:0] tag_mem   [0:NUM_LINES-1];
	reg                valid_mem [0:NUM_LINES-1];
	reg                dirty_mem [0:NUM_LINES-1];
	reg [31:0]         data_mem  [0:NUM_LINES*WORDS_PER_LINE-1];

	// Cache lookup
	wire tag_match = (tag_mem[addr_index] == addr_tag);
	wire cache_hit = valid_mem[addr_index] && tag_match;

	// State machine
	localparam ST_IDLE       = 3'd0;
	localparam ST_READ_HIT   = 3'd1;
	localparam ST_WRITEBACK  = 3'd2;
	localparam ST_FILL       = 3'd3;
	localparam ST_WRITE_HIT  = 3'd4;

	reg [2:0] state;
	reg [1:0] fill_word;  // Which word in the line we're filling

	integer i;
	always @(posedge clk) begin
		if (rst) begin
			state     <= ST_IDLE;
			cpu_ready <= 0;
			mem_ren   <= 0;
			mem_wstrb <= 0;
			hit_count <= 0;
			miss_count <= 0;
			fill_word <= 0;
			for (i = 0; i < NUM_LINES; i = i + 1) begin
				valid_mem[i] <= 0;
				dirty_mem[i] <= 0;
			end
		end else begin
			cpu_ready <= 0;
			mem_ren   <= 0;
			mem_wstrb <= 0;

			case (state)
				ST_IDLE: begin
					if (cpu_ren || |cpu_wstrb) begin
						if (cache_hit) begin
							if (cpu_ren) begin
								// TODO: Read hit - return data from cache
								cpu_rdata <= data_mem[addr_index * WORDS_PER_LINE + word_sel];
								cpu_ready <= 1;
								hit_count <= hit_count + 1;
							end else begin
								// TODO: Write hit - update cache line
								data_mem[addr_index * WORDS_PER_LINE + word_sel] <= cpu_wdata;
								dirty_mem[addr_index] <= 1;
								cpu_ready <= 1;
								hit_count <= hit_count + 1;
							end
						end else begin
							// Cache miss
							miss_count <= miss_count + 1;
							fill_word  <= 0;

							// Check if we need to write back dirty line first
							if (valid_mem[addr_index] && dirty_mem[addr_index]) begin
								state <= ST_WRITEBACK;
							end else begin
								state <= ST_FILL;
								// Start first word fetch
								mem_addr <= {cpu_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
								mem_ren  <= 1;
							end
						end
					end
				end

				ST_WRITEBACK: begin
					// TODO: Write dirty line back to memory
					// Write each word, advancing fill_word
					mem_addr  <= {tag_mem[addr_index], addr_index, fill_word, 2'b00};
					mem_wdata <= data_mem[addr_index * WORDS_PER_LINE + fill_word];
					mem_wstrb <= 4'b1111;

					if (mem_ready) begin
						if (fill_word == WORDS_PER_LINE - 1) begin
							fill_word <= 0;
							state <= ST_FILL;
							mem_addr <= {cpu_addr[ADDR_WIDTH-1:OFFSET_BITS], {OFFSET_BITS{1'b0}}};
							mem_ren  <= 1;
						end else begin
							fill_word <= fill_word + 1;
						end
					end
				end

				ST_FILL: begin
					// TODO: Fill cache line from memory
					mem_addr <= {cpu_addr[ADDR_WIDTH-1:OFFSET_BITS], fill_word, 2'b00};
					mem_ren  <= 1;

					if (mem_ready) begin
						data_mem[addr_index * WORDS_PER_LINE + fill_word] <= mem_rdata;

						if (fill_word == WORDS_PER_LINE - 1) begin
							// Line fill complete
							tag_mem[addr_index]   <= addr_tag;
							valid_mem[addr_index] <= 1;
							dirty_mem[addr_index] <= 0;
							state <= ST_IDLE;
							// The original request will retry next cycle
						end else begin
							fill_word <= fill_word + 1;
						end
					end
				end

				default: state <= ST_IDLE;
			endcase
		end
	end

endmodule
