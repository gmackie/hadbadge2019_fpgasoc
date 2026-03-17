/*
 * Lab 8: SoC Integration - Complete System-on-Chip
 * ==================================================
 * Wire together the CPU from previous labs with peripherals into a
 * complete System-on-Chip.  This is the capstone lab: you will build
 * an address decoder, connect instruction ROM and data RAM, and add
 * I/O peripherals (UART, GPIO, timer).
 *
 * Memory Map:
 *   0x00000000 - 0x0000FFFF   Instruction ROM  (64 KiB)
 *   0x10000000 - 0x1000FFFF   Data RAM         (64 KiB)
 *   0x20000000                 UART registers
 *   0x20001000                 GPIO registers
 *   0x20002000                 Timer registers
 *   0x20003000                 I2C  registers  (challenge)
 *   0x20004000                 SPI  registers  (challenge)
 *
 * EXERCISES:
 *
 * Exercise 1: Implement a simple bus decoder
 *   Decode the upper address bits to generate peripheral chip-select
 *   signals.  Only one peripheral may be selected at a time.
 *   Multiplex the read-data path so the CPU gets data from the
 *   correct source.
 *
 * Exercise 2: Connect the CPU to instruction ROM and data RAM
 *   Instantiate a single-port ROM for instructions and a
 *   read/write RAM for data.  Wire the CPU's fetch and memory
 *   interfaces to the correct memories through the bus decoder.
 *
 * Exercise 3: Add a UART peripheral for serial output
 *   Implement a minimal UART TX register: writing to the UART
 *   address transmits a byte.  Add a status register so the CPU
 *   can poll for TX-ready.
 *
 * Exercise 4: Add a GPIO peripheral for buttons and LEDs
 *   Implement a GPIO block with an output data register (directly
 *   drives gpio_out), an input register (samples gpio_in), and an
 *   output-enable register.
 *
 * Exercise 5 (Challenge): Add a bus bridge to I2C and SPI
 *   Connect the I2C and SPI peripherals you built in Year 2 to
 *   this SoC through the address decoder.  Design the register
 *   interface so the CPU can configure, send, and receive data.
 *
 * CONCEPTS:
 *   - System-on-Chip architecture: CPU + memory + I/O on one chip
 *   - Memory map design: partitioning the address space
 *   - Bus decoding: address-based peripheral selection
 *   - Peripheral integration: register-mapped I/O
 *   - Address space partitioning: ROM / RAM / I/O regions
 *   - Boot sequence: CPU fetches first instruction from ROM at 0x0
 *   - Real-world SoC design trade-offs (area, power, bandwidth)
 */

`default_nettype none
`include "../common/riscv_defs.vh"

module soc_simple (
    input  wire       clk,
    input  wire       rst,

    // --- UART pins ---
    output wire       uart_tx,
    input  wire       uart_rx,

    // --- GPIO pins ---
    input  wire [7:0] gpio_in,
    output wire [7:0] gpio_out,

    // --- SPI pins (Exercise 5) ---
    output wire       spi_sck,
    output wire       spi_mosi,
    input  wire       spi_miso,
    output wire       spi_cs_n,

    // --- I2C pins (Exercise 5) ---
    output wire       i2c_scl,
    inout  wire       i2c_sda
);

    // ================================================================
    // Parameters
    // ================================================================
    localparam ROM_ADDR_WIDTH = 14;  // 16K words = 64 KiB
    localparam RAM_ADDR_WIDTH = 14;  // 16K words = 64 KiB

    // ================================================================
    // Section 1 -- CPU Instance (from previous labs)
    // ================================================================
    // In a complete implementation you would instantiate your pipelined
    // CPU from Labs 1-3 here.  For this lab we define the CPU signals
    // and leave the instantiation as a TODO so students can plug in
    // their own core.

    // Instruction fetch interface
    wire [31:0] cpu_instr_addr;
    wire [31:0] cpu_instr_rdata;
    wire        cpu_instr_req;
    wire        cpu_instr_ready;

    // Data memory interface
    wire [31:0] cpu_data_addr;
    wire [31:0] cpu_data_wdata;
    wire [31:0] cpu_data_rdata;
    wire        cpu_data_ren;
    wire [3:0]  cpu_data_wstrb;
    wire        cpu_data_ready;

    // -----------------------------------------------------------------
    // TODO (Exercise 2): Instantiate your CPU here.
    //   Connect cpu_instr_* to the instruction fetch port and
    //   cpu_data_* to the data memory port.
    //
    //   Example:
    //   cpu_pipeline u_cpu (
    //       .clk           (clk),
    //       .rst           (rst),
    //       .instr_addr    (cpu_instr_addr),
    //       .instr_rdata   (cpu_instr_rdata),
    //       .instr_req     (cpu_instr_req),
    //       .instr_ready   (cpu_instr_ready),
    //       .data_addr     (cpu_data_addr),
    //       .data_wdata    (cpu_data_wdata),
    //       .data_rdata    (cpu_data_rdata),
    //       .data_ren      (cpu_data_ren),
    //       .data_wstrb    (cpu_data_wstrb),
    //       .data_ready    (cpu_data_ready)
    //   );
    // -----------------------------------------------------------------

    // Placeholder drives (remove when CPU is instantiated)
    assign cpu_instr_addr  = 32'b0;
    assign cpu_instr_req   = 1'b0;
    assign cpu_data_addr   = 32'b0;
    assign cpu_data_wdata  = 32'b0;
    assign cpu_data_ren    = 1'b0;
    assign cpu_data_wstrb  = 4'b0;

    // ================================================================
    // Section 2 -- Address Decoder
    // ================================================================
    // The address decoder examines the upper bits of cpu_data_addr
    // to determine which peripheral is being accessed.
    //
    // Region         | Address MSBs          | Select
    // -------------- | --------------------- | ------
    // Instr ROM      | 0x0000_xxxx           | sel_rom
    // Data RAM       | 0x1000_xxxx           | sel_ram
    // UART           | 0x2000_0xxx           | sel_uart
    // GPIO           | 0x2000_1xxx           | sel_gpio
    // Timer          | 0x2000_2xxx           | sel_timer
    // I2C            | 0x2000_3xxx           | sel_i2c
    // SPI            | 0x2000_4xxx           | sel_spi

    wire sel_rom;
    wire sel_ram;
    wire sel_uart;
    wire sel_gpio;
    wire sel_timer;
    wire sel_i2c;
    wire sel_spi;

    // -----------------------------------------------------------------
    // TODO (Exercise 1): Decode the address to generate select signals.
    //
    //   Hint: use the top nibble for the major region, then the
    //   next nibble for peripheral sub-selection within the I/O region.
    //
    //   assign sel_rom   = (cpu_data_addr[31:16] == 16'h0000);
    //   assign sel_ram   = (cpu_data_addr[31:16] == 16'h1000);
    //   assign sel_uart  = (cpu_data_addr[31:12] == 20'h20000);
    //   assign sel_gpio  = (cpu_data_addr[31:12] == 20'h20001);
    //   assign sel_timer = (cpu_data_addr[31:12] == 20'h20002);
    //   assign sel_i2c   = (cpu_data_addr[31:12] == 20'h20003);
    //   assign sel_spi   = (cpu_data_addr[31:12] == 20'h20004);
    // -----------------------------------------------------------------
    assign sel_rom   = 1'b0;  // TODO: Replace with decode logic
    assign sel_ram   = 1'b0;  // TODO: Replace with decode logic
    assign sel_uart  = 1'b0;  // TODO: Replace with decode logic
    assign sel_gpio  = 1'b0;  // TODO: Replace with decode logic
    assign sel_timer = 1'b0;  // TODO: Replace with decode logic
    assign sel_i2c   = 1'b0;  // TODO: Replace with decode logic
    assign sel_spi   = 1'b0;  // TODO: Replace with decode logic

    // Bus error: no peripheral selected (address hole)
    wire bus_error = (cpu_data_ren || |cpu_data_wstrb) &&
                     !(sel_rom || sel_ram || sel_uart || sel_gpio ||
                       sel_timer || sel_i2c || sel_spi);

    // ================================================================
    // Section 3 -- Instruction ROM
    // ================================================================
    // Single-port synchronous ROM.  In a real FPGA this would be
    // inferred as block RAM and initialized from a .hex file.

    reg [31:0] instr_rom [0:(1<<ROM_ADDR_WIDTH)-1];

    // Word-aligned address (drop bottom 2 bits)
    wire [ROM_ADDR_WIDTH-1:0] rom_word_addr = cpu_instr_addr[ROM_ADDR_WIDTH+1:2];

    reg [31:0] rom_rdata;
    reg        rom_ready;

    always @(posedge clk) begin
        if (rst) begin
            rom_ready <= 1'b0;
        end else begin
            rom_rdata <= instr_rom[rom_word_addr];
            rom_ready <= cpu_instr_req;
        end
    end

    assign cpu_instr_rdata = rom_rdata;
    assign cpu_instr_ready = rom_ready;

    // Initialize ROM contents (boot code)
    // In synthesis, replace with $readmemh("firmware.hex", instr_rom);
    initial begin
        // NOP sled as default
        integer ri;
        for (ri = 0; ri < (1 << ROM_ADDR_WIDTH); ri = ri + 1)
            instr_rom[ri] = 32'h00000013;  // addi x0, x0, 0 (NOP)
    end

    // ================================================================
    // Section 4 -- Data RAM
    // ================================================================
    // Byte-addressable synchronous RAM with byte-write strobes.

    reg [31:0] data_ram [0:(1<<RAM_ADDR_WIDTH)-1];

    wire [RAM_ADDR_WIDTH-1:0] ram_word_addr = cpu_data_addr[RAM_ADDR_WIDTH+1:2];

    reg [31:0] ram_rdata;
    reg        ram_ready;

    // -----------------------------------------------------------------
    // TODO (Exercise 2): Implement the data RAM with byte write strobes.
    //
    //   On a read (cpu_data_ren && sel_ram):
    //     - Register the read data on the next clock edge.
    //     - Assert ram_ready one cycle later.
    //
    //   On a write (|cpu_data_wstrb && sel_ram):
    //     - Use cpu_data_wstrb[0] to gate byte 0, [1] for byte 1, etc.
    //     - Assert ram_ready one cycle later.
    //
    //   Example byte-write:
    //     if (cpu_data_wstrb[0]) data_ram[addr][ 7: 0] <= cpu_data_wdata[ 7: 0];
    //     if (cpu_data_wstrb[1]) data_ram[addr][15: 8] <= cpu_data_wdata[15: 8];
    //     ...
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            ram_ready <= 1'b0;
        end else begin
            ram_ready <= 1'b0;

            if (sel_ram && cpu_data_ren) begin
                ram_rdata <= data_ram[ram_word_addr];
                ram_ready <= 1'b1;
            end

            if (sel_ram && |cpu_data_wstrb) begin
                // TODO: Byte-lane writes
                // if (cpu_data_wstrb[0]) data_ram[ram_word_addr][ 7: 0] <= cpu_data_wdata[ 7: 0];
                // if (cpu_data_wstrb[1]) data_ram[ram_word_addr][15: 8] <= cpu_data_wdata[15: 8];
                // if (cpu_data_wstrb[2]) data_ram[ram_word_addr][23:16] <= cpu_data_wdata[23:16];
                // if (cpu_data_wstrb[3]) data_ram[ram_word_addr][31:24] <= cpu_data_wdata[31:24];
                ram_ready <= 1'b1;
            end
        end
    end

    // ================================================================
    // Section 5 -- UART Peripheral
    // ================================================================
    // Minimal UART: register-mapped TX data and status.
    //
    // Register offsets (from UART base 0x20000000):
    //   0x00  TXDATA  [7:0] write=transmit byte, read=0
    //   0x04  TXSTAT  [0] TX ready (1=can accept byte, 0=busy)
    //   0x08  RXDATA  [7:0] read=received byte (clears rx_valid)
    //   0x0C  RXSTAT  [0] RX data available

    reg [7:0]  uart_tx_data;
    reg        uart_tx_valid;
    reg        uart_tx_ready;
    reg [7:0]  uart_rx_data;
    reg        uart_rx_valid;

    reg [31:0] uart_rdata;
    reg        uart_ready;

    // -----------------------------------------------------------------
    // TODO (Exercise 3): Implement the UART register interface.
    //
    //   On write to offset 0x00 (TXDATA):
    //     - Latch cpu_data_wdata[7:0] into uart_tx_data.
    //     - Set uart_tx_valid to begin transmission.
    //     - Clear uart_tx_ready until transmission completes.
    //
    //   On read from offset 0x04 (TXSTAT):
    //     - Return uart_tx_ready in bit 0.
    //
    //   On read from offset 0x08 (RXDATA):
    //     - Return uart_rx_data, clear uart_rx_valid.
    //
    //   For simulation, you can use $write to print characters:
    //     if (uart_tx_valid) $write("%c", uart_tx_data);
    // -----------------------------------------------------------------
    wire [3:0] uart_reg_offset = cpu_data_addr[3:0];

    always @(posedge clk) begin
        if (rst) begin
            uart_tx_data  <= 8'b0;
            uart_tx_valid <= 1'b0;
            uart_tx_ready <= 1'b1;
            uart_rx_data  <= 8'b0;
            uart_rx_valid <= 1'b0;
            uart_ready    <= 1'b0;
            uart_rdata    <= 32'b0;
        end else begin
            uart_ready    <= 1'b0;
            uart_tx_valid <= 1'b0;

            if (sel_uart) begin
                if (|cpu_data_wstrb) begin
                    // Write access
                    case (uart_reg_offset)
                        4'h0: begin  // TXDATA
                            // TODO: Latch TX byte and start transmission
                            uart_tx_data  <= cpu_data_wdata[7:0];
                            uart_tx_valid <= 1'b1;
                        end
                        default: ;
                    endcase
                    uart_ready <= 1'b1;
                end

                if (cpu_data_ren) begin
                    // Read access
                    case (uart_reg_offset)
                        4'h0: uart_rdata <= {24'b0, uart_tx_data};
                        4'h4: uart_rdata <= {31'b0, uart_tx_ready};
                        4'h8: begin
                            uart_rdata    <= {24'b0, uart_rx_data};
                            uart_rx_valid <= 1'b0;  // clear on read
                        end
                        4'hC: uart_rdata <= {31'b0, uart_rx_valid};
                        default: uart_rdata <= 32'b0;
                    endcase
                    uart_ready <= 1'b1;
                end
            end

            // Simulation helper: print TX characters
            `ifdef SIMULATION
            if (uart_tx_valid)
                $write("%c", uart_tx_data);
            `endif
        end
    end

    // Stub: drive uart_tx low (a real UART would serialize bits here)
    assign uart_tx = 1'b1;  // idle high

    // ================================================================
    // Section 6 -- GPIO Peripheral
    // ================================================================
    // Register offsets (from GPIO base 0x20001000):
    //   0x00  OUTPUT   [7:0] drive value for gpio_out pins
    //   0x04  INPUT    [7:0] current state of gpio_in pins (read-only)
    //   0x08  OE       [7:0] output enable (1=output, 0=input/hi-Z)

    reg [7:0] gpio_out_reg;
    reg [7:0] gpio_oe_reg;

    reg [31:0] gpio_rdata;
    reg        gpio_ready;

    // -----------------------------------------------------------------
    // TODO (Exercise 4): Implement the GPIO register interface.
    //
    //   Write to 0x00: update gpio_out_reg
    //   Write to 0x08: update gpio_oe_reg
    //   Read  from 0x00: return gpio_out_reg
    //   Read  from 0x04: return sampled gpio_in
    //   Read  from 0x08: return gpio_oe_reg
    // -----------------------------------------------------------------
    wire [3:0] gpio_reg_offset = cpu_data_addr[3:0];

    always @(posedge clk) begin
        if (rst) begin
            gpio_out_reg <= 8'b0;
            gpio_oe_reg  <= 8'b0;
            gpio_ready   <= 1'b0;
            gpio_rdata   <= 32'b0;
        end else begin
            gpio_ready <= 1'b0;

            if (sel_gpio) begin
                if (|cpu_data_wstrb) begin
                    case (gpio_reg_offset)
                        4'h0: gpio_out_reg <= cpu_data_wdata[7:0];
                        4'h8: gpio_oe_reg  <= cpu_data_wdata[7:0];
                        default: ;
                    endcase
                    gpio_ready <= 1'b1;
                end

                if (cpu_data_ren) begin
                    case (gpio_reg_offset)
                        4'h0: gpio_rdata <= {24'b0, gpio_out_reg};
                        4'h4: gpio_rdata <= {24'b0, gpio_in};
                        4'h8: gpio_rdata <= {24'b0, gpio_oe_reg};
                        default: gpio_rdata <= 32'b0;
                    endcase
                    gpio_ready <= 1'b1;
                end
            end
        end
    end

    // Drive GPIO output pins (active only when OE is set)
    assign gpio_out = gpio_out_reg & gpio_oe_reg;

    // ================================================================
    // Section 7 -- Timer Peripheral
    // ================================================================
    // Register offsets (from Timer base 0x20002000):
    //   0x00  MTIME_LO   [31:0]  low  32 bits of mtime (read/write)
    //   0x04  MTIME_HI   [31:0]  high 32 bits of mtime (read/write)
    //   0x08  MTCMP_LO   [31:0]  low  32 bits of mtimecmp
    //   0x0C  MTCMP_HI   [31:0]  high 32 bits of mtimecmp

    reg [63:0] timer_mtime;
    reg [63:0] timer_mtimecmp;
    reg [31:0] timer_rdata;
    reg        timer_ready;

    wire timer_irq_out = (timer_mtime >= timer_mtimecmp);

    wire [3:0] timer_reg_offset = cpu_data_addr[3:0];

    always @(posedge clk) begin
        if (rst) begin
            timer_mtime    <= 64'b0;
            timer_mtimecmp <= 64'hFFFFFFFF_FFFFFFFF;
            timer_ready    <= 1'b0;
            timer_rdata    <= 32'b0;
        end else begin
            // Free-running counter
            timer_mtime <= timer_mtime + 64'd1;
            timer_ready <= 1'b0;

            if (sel_timer) begin
                if (|cpu_data_wstrb) begin
                    case (timer_reg_offset)
                        4'h0: timer_mtime[31:0]     <= cpu_data_wdata;
                        4'h4: timer_mtime[63:32]    <= cpu_data_wdata;
                        4'h8: timer_mtimecmp[31:0]  <= cpu_data_wdata;
                        4'hC: timer_mtimecmp[63:32] <= cpu_data_wdata;
                        default: ;
                    endcase
                    timer_ready <= 1'b1;
                end

                if (cpu_data_ren) begin
                    case (timer_reg_offset)
                        4'h0: timer_rdata <= timer_mtime[31:0];
                        4'h4: timer_rdata <= timer_mtime[63:32];
                        4'h8: timer_rdata <= timer_mtimecmp[31:0];
                        4'hC: timer_rdata <= timer_mtimecmp[63:32];
                        default: timer_rdata <= 32'b0;
                    endcase
                    timer_ready <= 1'b1;
                end
            end
        end
    end

    // ================================================================
    // Section 8 -- Read Data Multiplexer
    // ================================================================
    // Route read data from the selected peripheral back to the CPU.

    // -----------------------------------------------------------------
    // TODO (Exercise 1): Multiplex the read data from all peripherals.
    //   The CPU should receive data from whichever peripheral is
    //   currently selected.  Use a priority mux (or one-hot mux
    //   since only one select is active at a time).
    //
    //   assign cpu_data_rdata = sel_ram   ? ram_rdata   :
    //                           sel_uart  ? uart_rdata  :
    //                           sel_gpio  ? gpio_rdata  :
    //                           sel_timer ? timer_rdata :
    //                           32'hDEAD_BEEF;  // unmapped
    //
    //   assign cpu_data_ready = sel_ram   ? ram_ready   :
    //                           sel_uart  ? uart_ready  :
    //                           sel_gpio  ? gpio_ready  :
    //                           sel_timer ? timer_ready :
    //                           1'b0;
    // -----------------------------------------------------------------
    assign cpu_data_rdata = 32'b0;   // TODO: Replace with data mux
    assign cpu_data_ready = 1'b0;    // TODO: Replace with ready mux

    // ================================================================
    // Section 9 (Challenge) -- I2C and SPI Bus Bridge
    // ================================================================
    // For Exercise 5, instantiate the I2C and SPI peripherals from
    // Year 2 labs and connect them to the address decoder.
    //
    // I2C register map (base 0x20003000):
    //   0x00  CTRL     [0] start, [1] stop, [2] ack
    //   0x04  STATUS   [0] busy, [1] ack_received
    //   0x08  TXDATA   [7:0] byte to transmit
    //   0x0C  RXDATA   [7:0] received byte
    //
    // SPI register map (base 0x20004000):
    //   0x00  CTRL     [0] enable, [7:4] clock divider
    //   0x04  STATUS   [0] busy, [1] tx_empty
    //   0x08  TXDATA   [7:0] byte to transmit
    //   0x0C  RXDATA   [7:0] received byte

    // TODO (Exercise 5): Instantiate I2C peripheral
    // i2c_master u_i2c (
    //     .clk      (clk),
    //     .rst      (rst),
    //     .scl      (i2c_scl),
    //     .sda      (i2c_sda),
    //     ...
    // );

    // TODO (Exercise 5): Instantiate SPI peripheral
    // spi_master u_spi (
    //     .clk      (clk),
    //     .rst      (rst),
    //     .sck      (spi_sck),
    //     .mosi     (spi_mosi),
    //     .miso     (spi_miso),
    //     .cs_n     (spi_cs_n),
    //     ...
    // );

    // Stub drives for unimplemented peripherals
    assign spi_sck  = 1'b0;
    assign spi_mosi = 1'b0;
    assign spi_cs_n = 1'b1;   // deselected
    assign i2c_scl  = 1'b1;   // idle high
    assign i2c_sda  = 1'bz;   // tri-state

    // ================================================================
    // Debug / Verification Helpers
    // ================================================================
`ifdef FORMAL
    // At most one peripheral selected at a time
    always @(*) begin
        assert($onehot0({sel_rom, sel_ram, sel_uart, sel_gpio,
                         sel_timer, sel_i2c, sel_spi}));
    end

    // Bus error should fire on unmapped access
    always @(*) begin
        if (cpu_data_ren && cpu_data_addr == 32'hFFFF_0000)
            assert(bus_error);
    end
`endif

`ifdef SIMULATION
    // Print a message on boot
    initial begin
        $display("[SoC] System-on-Chip starting up");
        $display("[SoC] ROM: 0x00000000 - 0x0000FFFF (%0d KiB)", (1<<ROM_ADDR_WIDTH)*4/1024);
        $display("[SoC] RAM: 0x10000000 - 0x1000FFFF (%0d KiB)", (1<<RAM_ADDR_WIDTH)*4/1024);
        $display("[SoC] UART:  0x20000000");
        $display("[SoC] GPIO:  0x20001000");
        $display("[SoC] Timer: 0x20002000");
    end
`endif

endmodule
