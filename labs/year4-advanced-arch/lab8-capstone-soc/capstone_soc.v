/*
 * Lab 8: Capstone - Full SoC with Advanced Peripherals
 * =====================================================
 * Integrate everything: OoO CPU, GPU, DSP, PID controller, and all
 * communication peripherals into a complete badge SoC. This is the
 * culmination of the entire 4-year curriculum.
 *
 * Memory Map:
 *   0x00000000 - 0x0000FFFF  Boot ROM (64KB)
 *   0x10000000 - 0x1001FFFF  SRAM (128KB)
 *   0x20000000 - 0x20000FFF  UART
 *   0x20001000 - 0x20001FFF  GPIO
 *   0x20002000 - 0x20002FFF  I2C
 *   0x20003000 - 0x20003FFF  SPI
 *   0x20004000 - 0x20004FFF  Timer
 *   0x30000000 - 0x3000FFFF  GPU command/status
 *   0x30010000 - 0x3001FFFF  DSP/FFT sample/result memory
 *   0x30020000 - 0x3002FFFF  PID controller registers
 *   0x40000000 - 0x43FFFFFF  External PSRAM (via cache)
 *
 * EXERCISES:
 *
 * Exercise 1: Top-level memory map with bus arbiter
 *   Implement the address decoder and a round-robin bus arbiter
 *   for CPU, GPU, and DSP masters. The arbiter grants one master
 *   access per cycle, with configurable priority for the CPU.
 *
 * Exercise 2: GPU pipeline integration
 *   Map the GPU pipeline as a memory-mapped peripheral at 0x30000000.
 *   Register layout:
 *     0x00: CMD_TYPE   (W) - command type (line/rect/triangle)
 *     0x04: X0         (W) - vertex 0 X
 *     0x08: Y0         (W) - vertex 0 Y
 *     0x0C: X1         (W) - vertex 1 X / width
 *     0x10: Y1         (W) - vertex 1 Y / height
 *     0x14: X2         (W) - vertex 2 X
 *     0x18: Y2         (W) - vertex 2 Y
 *     0x1C: COLOR      (W) - RGB565 color
 *     0x20: CONTROL    (W) - bit 0: start command
 *     0x24: STATUS     (R) - bit 0: busy, bit 1: cmd_ready
 *
 * Exercise 3: DSP/FFT accelerator integration
 *   Map the FFT engine at 0x30010000.
 *   Register layout:
 *     0x0000: CONTROL  (W) - bit 0: start FFT
 *     0x0004: STATUS   (R) - bit 0: busy, bit 1: done
 *     0x0100-0x011F: SAMPLE_MEM (W) - 8 x 32-bit input samples
 *     0x0200-0x021F: RESULT_MEM (R) - 8 x 32-bit output bins
 *
 * Exercise 4: PID controller integration
 *   Map the PID controller at 0x30020000.
 *   Register layout:
 *     0x00: SETPOINT    (RW)
 *     0x04: MEASUREMENT (R) - read from ADC
 *     0x08: OUTPUT      (R) - current PID output
 *     0x0C: KP          (RW)
 *     0x10: KI          (RW)
 *     0x14: KD          (RW)
 *     0x18: OUT_MIN     (RW)
 *     0x1C: OUT_MAX     (RW)
 *     0x20: CONTROL     (RW) - bit 0: active, bit 1: sample_tick (auto)
 *     0x24: STATUS      (R)  - bit 0: saturated
 *
 * Exercise 5 (challenge): Interrupt controller
 *   Add an interrupt controller that consolidates interrupts from all
 *   peripherals into a single CPU interrupt line:
 *     IRQ 0: UART RX ready
 *     IRQ 1: UART TX empty
 *     IRQ 2: GPIO edge detect
 *     IRQ 3: Timer overflow
 *     IRQ 4: GPU command done
 *     IRQ 5: FFT done
 *     IRQ 6: PID sample ready
 *     IRQ 7: SPI transfer complete
 *   Implement interrupt enable, pending, and clear registers.
 *
 * CONCEPTS:
 *   - System integration and multi-master bus architecture
 *   - Address decoding and peripheral mapping
 *   - Round-robin and priority bus arbitration
 *   - Interrupt consolidation and priority encoding
 *   - Hardware/software co-design
 *   - Design verification strategy for complex SoCs
 *   - Real-world SoC complexity and design tradeoffs
 */

`default_nettype none

module capstone_soc (
    input  wire        clk,
    input  wire        rst,

    // UART
    output wire        uart_tx,
    input  wire        uart_rx,

    // GPIO
    input  wire [7:0]  gpio_in,
    output wire [7:0]  gpio_out,

    // I2C
    output wire        i2c_scl,
    inout  wire        i2c_sda,

    // SPI
    output wire        spi_sck,
    output wire        spi_mosi,
    input  wire        spi_miso,
    output wire [3:0]  spi_cs_n,

    // HDMI / LCD display
    output wire        hdmi_clk,
    output wire [2:0]  hdmi_data,

    // Audio
    output wire        audio_out,

    // PSRAM
    output wire        psram_sck,
    output wire        psram_cs_n,
    inout  wire [3:0]  psram_data
);

    // =========================================================================
    // Address decode constants
    // =========================================================================
    localparam ADDR_BOOT_BASE   = 32'h0000_0000;
    localparam ADDR_SRAM_BASE   = 32'h1000_0000;
    localparam ADDR_UART_BASE   = 32'h2000_0000;
    localparam ADDR_GPIO_BASE   = 32'h2000_1000;
    localparam ADDR_I2C_BASE    = 32'h2000_2000;
    localparam ADDR_SPI_BASE    = 32'h2000_3000;
    localparam ADDR_TIMER_BASE  = 32'h2000_4000;
    localparam ADDR_GPU_BASE    = 32'h3000_0000;
    localparam ADDR_DSP_BASE    = 32'h3001_0000;
    localparam ADDR_PID_BASE    = 32'h3002_0000;
    localparam ADDR_PSRAM_BASE  = 32'h4000_0000;

    // =========================================================================
    // Bus master interface (simplified Wishbone-like)
    // =========================================================================
    // CPU master signals
    wire [31:0] cpu_addr;
    wire [31:0] cpu_wdata;
    wire [31:0] cpu_rdata;
    wire        cpu_we;
    wire        cpu_stb;
    wire        cpu_ack;

    // GPU master signals (for framebuffer DMA writes)
    wire [31:0] gpu_dma_addr;
    wire [31:0] gpu_dma_wdata;
    wire        gpu_dma_we;
    wire        gpu_dma_stb;
    wire        gpu_dma_ack;

    // DSP master signals (for sample DMA)
    wire [31:0] dsp_dma_addr;
    wire [31:0] dsp_dma_wdata;
    wire [31:0] dsp_dma_rdata;
    wire        dsp_dma_we;
    wire        dsp_dma_stb;
    wire        dsp_dma_ack;

    // Shared bus (output of arbiter)
    reg  [31:0] bus_addr;
    reg  [31:0] bus_wdata;
    wire [31:0] bus_rdata;
    reg         bus_we;
    reg         bus_stb;
    wire        bus_ack;

    // =========================================================================
    // Bus arbiter - Round-robin with CPU priority (Exercise 1)
    // =========================================================================
    // TODO (Exercise 1): Implement a multi-master bus arbiter.
    //
    // Three masters compete for the bus: CPU, GPU DMA, DSP DMA.
    // Policy: Round-robin, but CPU gets priority when multiple masters
    //         request simultaneously (CPU is the most latency-sensitive).
    //
    // The arbiter grants one master per transaction. A transaction is
    // complete when bus_ack is asserted.

    localparam [1:0] MASTER_CPU = 2'd0,
                     MASTER_GPU = 2'd1,
                     MASTER_DSP = 2'd2;

    reg [1:0] current_master;
    reg [1:0] last_grant;       // For round-robin fairness
    reg       grant_valid;

    // Arbitration request signals
    wire req_cpu = cpu_stb;
    wire req_gpu = gpu_dma_stb;
    wire req_dsp = dsp_dma_stb;

    // Round-robin arbiter with CPU priority boost
    always @(posedge clk) begin
        if (rst) begin
            current_master <= MASTER_CPU;
            last_grant     <= MASTER_CPU;
            grant_valid    <= 1'b0;
        end else if (!grant_valid || bus_ack) begin
            // Arbitrate next master
            // CPU always wins if it's requesting (priority)
            if (req_cpu) begin
                current_master <= MASTER_CPU;
                last_grant     <= MASTER_CPU;
                grant_valid    <= 1'b1;
            end else if (req_gpu && (last_grant != MASTER_GPU || !req_dsp)) begin
                current_master <= MASTER_GPU;
                last_grant     <= MASTER_GPU;
                grant_valid    <= 1'b1;
            end else if (req_dsp) begin
                current_master <= MASTER_DSP;
                last_grant     <= MASTER_DSP;
                grant_valid    <= 1'b1;
            end else begin
                grant_valid <= 1'b0;
            end
        end
    end

    // Mux master signals onto shared bus
    always @(*) begin
        bus_addr  = 32'd0;
        bus_wdata = 32'd0;
        bus_we    = 1'b0;
        bus_stb   = 1'b0;

        case (current_master)
            MASTER_CPU: begin
                bus_addr  = cpu_addr;
                bus_wdata = cpu_wdata;
                bus_we    = cpu_we;
                bus_stb   = cpu_stb & grant_valid;
            end
            MASTER_GPU: begin
                bus_addr  = gpu_dma_addr;
                bus_wdata = gpu_dma_wdata;
                bus_we    = gpu_dma_we;
                bus_stb   = gpu_dma_stb & grant_valid;
            end
            MASTER_DSP: begin
                bus_addr  = dsp_dma_addr;
                bus_wdata = dsp_dma_wdata;
                bus_we    = dsp_dma_we;
                bus_stb   = dsp_dma_stb & grant_valid;
            end
            default: begin
                bus_addr  = 32'd0;
                bus_wdata = 32'd0;
                bus_we    = 1'b0;
                bus_stb   = 1'b0;
            end
        endcase
    end

    // Route acknowledgments and read data back to masters
    assign cpu_ack      = bus_ack & grant_valid & (current_master == MASTER_CPU);
    assign cpu_rdata    = bus_rdata;
    assign gpu_dma_ack  = bus_ack & grant_valid & (current_master == MASTER_GPU);
    assign dsp_dma_ack  = bus_ack & grant_valid & (current_master == MASTER_DSP);
    assign dsp_dma_rdata = bus_rdata;

    // =========================================================================
    // Address decoder (Exercise 1)
    // =========================================================================
    // TODO (Exercise 1): Decode bus_addr to select the appropriate peripheral.
    // Generate chip-select signals based on the upper address bits.

    wire sel_boot  = (bus_addr[31:16] == 16'h0000);
    wire sel_sram  = (bus_addr[31:20] == 12'h100);
    wire sel_uart  = (bus_addr[31:12] == 20'h20000);
    wire sel_gpio  = (bus_addr[31:12] == 20'h20001);
    wire sel_i2c   = (bus_addr[31:12] == 20'h20002);
    wire sel_spi   = (bus_addr[31:12] == 20'h20003);
    wire sel_timer = (bus_addr[31:12] == 20'h20004);
    wire sel_gpu   = (bus_addr[31:16] == 16'h3000);
    wire sel_dsp   = (bus_addr[31:16] == 16'h3001);
    wire sel_pid   = (bus_addr[31:16] == 16'h3002);
    wire sel_psram = (bus_addr[31:26] == 6'h10);    // 0x40000000

    // =========================================================================
    // Boot ROM (64KB)
    // =========================================================================
    reg [31:0] boot_rom [0:16383];  // 16K x 32-bit = 64KB
    reg [31:0] boot_rdata;
    reg        boot_ack;

    initial begin
        // TODO: Load boot code from hex file
        // $readmemh("bootrom.hex", boot_rom);
    end

    always @(posedge clk) begin
        if (rst) begin
            boot_ack <= 1'b0;
        end else begin
            boot_ack <= sel_boot & bus_stb & !boot_ack;
            if (sel_boot & bus_stb)
                boot_rdata <= boot_rom[bus_addr[15:2]];
        end
    end

    // =========================================================================
    // SRAM (128KB)
    // =========================================================================
    reg [31:0] sram [0:32767];  // 32K x 32-bit = 128KB
    reg [31:0] sram_rdata;
    reg        sram_ack;

    always @(posedge clk) begin
        if (rst) begin
            sram_ack <= 1'b0;
        end else begin
            sram_ack <= sel_sram & bus_stb & !sram_ack;
            if (sel_sram & bus_stb) begin
                if (bus_we)
                    sram[bus_addr[16:2]] <= bus_wdata;
                else
                    sram_rdata <= sram[bus_addr[16:2]];
            end
        end
    end

    // =========================================================================
    // GPU peripheral registers (Exercise 2)
    // =========================================================================
    reg [2:0]  gpu_cmd_type;
    reg [15:0] gpu_x0, gpu_y0, gpu_x1, gpu_y1, gpu_x2, gpu_y2;
    reg [15:0] gpu_color;
    reg        gpu_start;
    wire       gpu_busy;
    wire       gpu_cmd_ready;
    reg [31:0] gpu_rdata;
    reg        gpu_ack;

    // GPU pixel output (directly drives display or framebuffer)
    wire [9:0]  gpu_pixel_x;
    wire [8:0]  gpu_pixel_y;
    wire [15:0] gpu_pixel_color;
    wire        gpu_pixel_valid;
    reg         gpu_pixel_ready;

    // TODO (Exercise 2): Implement GPU register read/write logic
    always @(posedge clk) begin
        if (rst) begin
            gpu_cmd_type <= 3'd0;
            gpu_x0 <= 16'd0; gpu_y0 <= 16'd0;
            gpu_x1 <= 16'd0; gpu_y1 <= 16'd0;
            gpu_x2 <= 16'd0; gpu_y2 <= 16'd0;
            gpu_color <= 16'd0;
            gpu_start <= 1'b0;
            gpu_ack   <= 1'b0;
        end else begin
            gpu_start <= 1'b0;  // Auto-clear
            gpu_ack   <= sel_gpu & bus_stb & !gpu_ack;

            if (sel_gpu & bus_stb & bus_we) begin
                case (bus_addr[7:2])
                    6'h00: gpu_cmd_type <= bus_wdata[2:0];
                    6'h01: gpu_x0      <= bus_wdata[15:0];
                    6'h02: gpu_y0      <= bus_wdata[15:0];
                    6'h03: gpu_x1      <= bus_wdata[15:0];
                    6'h04: gpu_y1      <= bus_wdata[15:0];
                    6'h05: gpu_x2      <= bus_wdata[15:0];
                    6'h06: gpu_y2      <= bus_wdata[15:0];
                    6'h07: gpu_color   <= bus_wdata[15:0];
                    6'h08: gpu_start   <= bus_wdata[0];
                    default: ;
                endcase
            end

            if (sel_gpu & bus_stb & !bus_we) begin
                case (bus_addr[7:2])
                    6'h09:   gpu_rdata <= {30'd0, gpu_cmd_ready, gpu_busy};
                    default: gpu_rdata <= 32'd0;
                endcase
            end
        end
    end

    // GPU pipeline instantiation
    gpu_pipeline u_gpu (
        .clk         (clk),
        .rst         (rst),
        .cmd_valid   (gpu_start),
        .cmd_type    (gpu_cmd_type),
        .x0          (gpu_x0),
        .y0          (gpu_y0),
        .x1          (gpu_x1),
        .y1          (gpu_y1),
        .x2          (gpu_x2),
        .y2          (gpu_y2),
        .color       (gpu_color),
        .cmd_ready   (gpu_cmd_ready),
        .pixel_x     (gpu_pixel_x),
        .pixel_y     (gpu_pixel_y),
        .pixel_color (gpu_pixel_color),
        .pixel_valid (gpu_pixel_valid),
        .pixel_ready (gpu_pixel_ready),
        .busy        (gpu_busy)
    );

    // =========================================================================
    // DSP/FFT peripheral registers (Exercise 3)
    // =========================================================================
    reg        dsp_start;
    wire       dsp_done, dsp_busy;
    wire [1:0] dsp_fft_stage;
    reg [2:0]  dsp_wr_addr;
    reg [31:0] dsp_wr_data;
    reg        dsp_wr_en;
    wire [31:0] dsp_rd_data;
    reg [31:0] dsp_rdata;
    reg        dsp_ack;

    // TODO (Exercise 3): Implement DSP register read/write logic
    always @(posedge clk) begin
        if (rst) begin
            dsp_start   <= 1'b0;
            dsp_wr_en   <= 1'b0;
            dsp_wr_addr <= 3'd0;
            dsp_wr_data <= 32'd0;
            dsp_ack     <= 1'b0;
        end else begin
            dsp_start <= 1'b0;
            dsp_wr_en <= 1'b0;
            dsp_ack   <= sel_dsp & bus_stb & !dsp_ack;

            if (sel_dsp & bus_stb & bus_we) begin
                case (bus_addr[11:8])
                    4'h0: begin
                        // Control register at offset 0x0000
                        if (bus_addr[7:2] == 6'd0)
                            dsp_start <= bus_wdata[0];
                    end
                    4'h1: begin
                        // Sample memory write at 0x0100-0x011F
                        dsp_wr_addr <= bus_addr[4:2];
                        dsp_wr_data <= bus_wdata;
                        dsp_wr_en   <= 1'b1;
                    end
                    default: ;
                endcase
            end

            if (sel_dsp & bus_stb & !bus_we) begin
                case (bus_addr[11:8])
                    4'h0: begin
                        // Status register at 0x0004
                        dsp_rdata <= {28'd0, dsp_fft_stage, dsp_done, dsp_busy};
                    end
                    4'h2: begin
                        // Result memory read at 0x0200-0x021F
                        dsp_rdata <= dsp_rd_data;
                    end
                    default: dsp_rdata <= 32'd0;
                endcase
            end
        end
    end

    // FFT accelerator instantiation
    dsp_accelerator u_dsp (
        .clk            (clk),
        .rst            (rst),
        .start          (dsp_start),
        .done           (dsp_done),
        .busy           (dsp_busy),
        .sample_wr_addr (dsp_wr_addr),
        .sample_wr_data (dsp_wr_data),
        .sample_wr_en   (dsp_wr_en),
        .result_rd_addr (bus_addr[4:2]),
        .result_rd_data (dsp_rd_data),
        .fft_stage      (dsp_fft_stage)
    );

    // =========================================================================
    // PID controller peripheral registers (Exercise 4)
    // =========================================================================
    reg [31:0] pid_setpoint, pid_kp, pid_ki, pid_kd;
    reg [31:0] pid_out_min, pid_out_max;
    reg        pid_active;
    wire [31:0] pid_output;
    wire        pid_saturated;
    reg [31:0] pid_rdata;
    reg        pid_ack;

    // Sample tick generation (configurable divider)
    reg [15:0] pid_tick_div;
    reg [15:0] pid_tick_cnt;
    wire       pid_sample_tick = (pid_tick_cnt == pid_tick_div) && pid_active;

    always @(posedge clk) begin
        if (rst) begin
            pid_tick_cnt <= 16'd0;
            pid_tick_div <= 16'd999;  // Default: divide by 1000
        end else if (pid_sample_tick || !pid_active) begin
            pid_tick_cnt <= 16'd0;
        end else begin
            pid_tick_cnt <= pid_tick_cnt + 16'd1;
        end
    end

    // TODO (Exercise 4): Implement PID register read/write logic
    always @(posedge clk) begin
        if (rst) begin
            pid_setpoint <= 32'd0;
            pid_kp       <= 32'd0;
            pid_ki       <= 32'd0;
            pid_kd       <= 32'd0;
            pid_out_min  <= 32'sh8000_0000;
            pid_out_max  <= 32'sh7FFF_FFFF;
            pid_active   <= 1'b0;
            pid_ack      <= 1'b0;
        end else begin
            pid_ack <= sel_pid & bus_stb & !pid_ack;

            if (sel_pid & bus_stb & bus_we) begin
                case (bus_addr[7:2])
                    6'h00: pid_setpoint <= bus_wdata;
                    6'h03: pid_kp       <= bus_wdata;
                    6'h04: pid_ki       <= bus_wdata;
                    6'h05: pid_kd       <= bus_wdata;
                    6'h06: pid_out_min  <= bus_wdata;
                    6'h07: pid_out_max  <= bus_wdata;
                    6'h08: pid_active   <= bus_wdata[0];
                    default: ;
                endcase
            end

            if (sel_pid & bus_stb & !bus_we) begin
                case (bus_addr[7:2])
                    6'h00: pid_rdata <= pid_setpoint;
                    6'h01: pid_rdata <= 32'd0;        // TODO: connect to ADC measurement
                    6'h02: pid_rdata <= pid_output;
                    6'h03: pid_rdata <= pid_kp;
                    6'h04: pid_rdata <= pid_ki;
                    6'h05: pid_rdata <= pid_kd;
                    6'h06: pid_rdata <= pid_out_min;
                    6'h07: pid_rdata <= pid_out_max;
                    6'h08: pid_rdata <= {31'd0, pid_active};
                    6'h09: pid_rdata <= {31'd0, pid_saturated};
                    default: pid_rdata <= 32'd0;
                endcase
            end
        end
    end

    // PID controller instantiation
    pid_controller u_pid (
        .clk         (clk),
        .rst         (rst),
        .sample_tick (pid_sample_tick),
        .active      (pid_active),
        .setpoint    (pid_setpoint),
        .measurement (32'd0),           // TODO: connect to actual sensor/ADC
        .kp          (pid_kp),
        .ki          (pid_ki),
        .kd          (pid_kd),
        .output_min  (pid_out_min),
        .output_max  (pid_out_max),
        .output_val  (pid_output),
        .saturated   (pid_saturated)
    );

    // =========================================================================
    // Peripheral stubs (UART, GPIO, I2C, SPI, Timer)
    // =========================================================================
    // These are placeholder implementations. In a full design, each would be
    // a separate module. Students can reuse peripherals from Year 3 labs.

    // --- UART stub ---
    reg [31:0] uart_rdata;
    reg        uart_ack;
    reg [7:0]  uart_tx_data;
    reg        uart_tx_valid;

    always @(posedge clk) begin
        if (rst) begin
            uart_ack <= 1'b0;
            uart_tx_valid <= 1'b0;
        end else begin
            uart_ack <= sel_uart & bus_stb & !uart_ack;
            uart_tx_valid <= 1'b0;
            if (sel_uart & bus_stb & bus_we && bus_addr[3:2] == 2'd0) begin
                uart_tx_data  <= bus_wdata[7:0];
                uart_tx_valid <= 1'b1;
            end
            if (sel_uart & bus_stb & !bus_we) begin
                case (bus_addr[3:2])
                    2'd0: uart_rdata <= {24'd0, uart_tx_data};
                    2'd1: uart_rdata <= {30'd0, 1'b1, 1'b0};  // TX ready, RX empty
                    default: uart_rdata <= 32'd0;
                endcase
            end
        end
    end
    assign uart_tx = 1'b1;  // TODO: connect to actual UART TX module

    // --- GPIO stub ---
    reg [7:0]  gpio_out_r;
    reg [31:0] gpio_rdata;
    reg        gpio_ack;

    always @(posedge clk) begin
        if (rst) begin
            gpio_out_r <= 8'd0;
            gpio_ack   <= 1'b0;
        end else begin
            gpio_ack <= sel_gpio & bus_stb & !gpio_ack;
            if (sel_gpio & bus_stb & bus_we && bus_addr[3:2] == 2'd0)
                gpio_out_r <= bus_wdata[7:0];
            if (sel_gpio & bus_stb & !bus_we) begin
                case (bus_addr[3:2])
                    2'd0: gpio_rdata <= {24'd0, gpio_out_r};
                    2'd1: gpio_rdata <= {24'd0, gpio_in};
                    default: gpio_rdata <= 32'd0;
                endcase
            end
        end
    end
    assign gpio_out = gpio_out_r;

    // --- I2C, SPI, Timer stubs ---
    reg        i2c_ack, spi_stub_ack, timer_ack;
    reg [31:0] i2c_rdata, spi_rdata, timer_rdata;

    always @(posedge clk) begin
        if (rst) begin
            i2c_ack      <= 1'b0;
            spi_stub_ack <= 1'b0;
            timer_ack    <= 1'b0;
        end else begin
            i2c_ack      <= sel_i2c   & bus_stb & !i2c_ack;
            spi_stub_ack <= sel_spi   & bus_stb & !spi_stub_ack;
            timer_ack    <= sel_timer & bus_stb & !timer_ack;
            // TODO: Implement full I2C, SPI, Timer peripherals
            i2c_rdata   <= 32'd0;
            spi_rdata   <= 32'd0;
            timer_rdata <= 32'd0;
        end
    end

    assign i2c_scl  = 1'b1;   // TODO: connect to I2C controller
    assign i2c_sda  = 1'bz;
    assign spi_sck  = 1'b0;   // TODO: connect to SPI controller
    assign spi_mosi = 1'b0;
    assign spi_cs_n = 4'b1111;

    // --- PSRAM stub ---
    reg        psram_ack;
    reg [31:0] psram_rdata;

    always @(posedge clk) begin
        if (rst)
            psram_ack <= 1'b0;
        else
            psram_ack <= sel_psram & bus_stb & !psram_ack;
        // TODO: Implement PSRAM controller with cache
        psram_rdata <= 32'd0;
    end

    assign psram_sck  = 1'b0;  // TODO: connect to PSRAM controller
    assign psram_cs_n = 1'b1;
    assign psram_data = 4'bz;

    // =========================================================================
    // Bus read data mux and acknowledge
    // =========================================================================
    assign bus_rdata = sel_boot  ? boot_rdata  :
                       sel_sram  ? sram_rdata  :
                       sel_uart  ? uart_rdata  :
                       sel_gpio  ? gpio_rdata  :
                       sel_i2c   ? i2c_rdata   :
                       sel_spi   ? spi_rdata   :
                       sel_timer ? timer_rdata :
                       sel_gpu   ? gpu_rdata   :
                       sel_dsp   ? dsp_rdata   :
                       sel_pid   ? pid_rdata   :
                       sel_psram ? psram_rdata :
                       32'hDEAD_BEEF;   // Bus error / unmapped

    assign bus_ack = sel_boot  ? boot_ack    :
                     sel_sram  ? sram_ack    :
                     sel_uart  ? uart_ack    :
                     sel_gpio  ? gpio_ack    :
                     sel_i2c   ? i2c_ack     :
                     sel_spi   ? spi_stub_ack :
                     sel_timer ? timer_ack   :
                     sel_gpu   ? gpu_ack     :
                     sel_dsp   ? dsp_ack     :
                     sel_pid   ? pid_ack     :
                     sel_psram ? psram_ack   :
                     1'b0;

    // =========================================================================
    // Interrupt controller (Exercise 5 - Challenge)
    // =========================================================================
    //
    // TODO (Exercise 5): Implement an interrupt controller
    //
    // Architecture:
    //   - 8 interrupt sources (IRQ 0..7)
    //   - Three registers:
    //       IRQ_ENABLE  (RW): bit mask to enable/disable each IRQ
    //       IRQ_PENDING (R):  shows which IRQs are asserted
    //       IRQ_CLEAR   (W):  write 1 to clear pending IRQ bits
    //   - Single output: cpu_irq = |(irq_pending & irq_enable)
    //
    // reg [7:0] irq_enable;
    // reg [7:0] irq_pending;
    // wire [7:0] irq_sources;
    //
    // assign irq_sources = {
    //     spi_irq,         // IRQ 7: SPI transfer complete
    //     pid_sample_tick,  // IRQ 6: PID sample ready
    //     dsp_done,        // IRQ 5: FFT done
    //     gpu_done,        // IRQ 4: GPU command done
    //     timer_overflow,  // IRQ 3: Timer overflow
    //     gpio_edge,       // IRQ 2: GPIO edge detect
    //     uart_tx_empty,   // IRQ 1: UART TX empty
    //     uart_rx_ready    // IRQ 0: UART RX ready
    // };
    //
    // // Edge-detect and latch interrupts
    // reg [7:0] irq_sources_prev;
    // always @(posedge clk) begin
    //     if (rst) begin
    //         irq_pending <= 8'd0;
    //         irq_sources_prev <= 8'd0;
    //     end else begin
    //         irq_sources_prev <= irq_sources;
    //         // Set on rising edge of source
    //         irq_pending <= (irq_pending | (irq_sources & ~irq_sources_prev))
    //                      & ~irq_clear_mask;
    //     end
    // end
    //
    // wire cpu_irq = |(irq_pending & irq_enable);

    // =========================================================================
    // Display output stub
    // =========================================================================
    // TODO: Connect GPU pixel output to HDMI/LCD display controller
    assign hdmi_clk  = 1'b0;
    assign hdmi_data = 3'b000;
    assign audio_out = 1'b0;

    always @(posedge clk) begin
        if (rst)
            gpu_pixel_ready <= 1'b0;
        else
            gpu_pixel_ready <= gpu_pixel_valid;  // Simple always-ready for now
    end

    // =========================================================================
    // CPU instantiation stub
    // =========================================================================
    // TODO: Instantiate the OoO CPU from Lab 4 as the main processor
    //
    // cpu_ooo_top u_cpu (
    //     .clk       (clk),
    //     .rst       (rst),
    //     .imem_addr (cpu_addr),
    //     .imem_data (cpu_rdata),
    //     .dmem_addr (cpu_addr),
    //     .dmem_wdata(cpu_wdata),
    //     .dmem_rdata(cpu_rdata),
    //     .dmem_we   (cpu_we),
    //     .dmem_stb  (cpu_stb),
    //     .dmem_ack  (cpu_ack)
    // );

    // Temporary: tie off CPU bus signals when CPU is not instantiated
    assign cpu_addr  = 32'd0;
    assign cpu_wdata = 32'd0;
    assign cpu_we    = 1'b0;
    assign cpu_stb   = 1'b0;

    // Temporary: tie off GPU and DSP DMA signals
    assign gpu_dma_addr  = 32'd0;
    assign gpu_dma_wdata = 32'd0;
    assign gpu_dma_we    = 1'b0;
    assign gpu_dma_stb   = 1'b0;
    assign dsp_dma_addr  = 32'd0;
    assign dsp_dma_wdata = 32'd0;
    assign dsp_dma_we    = 1'b0;
    assign dsp_dma_stb   = 1'b0;

endmodule
