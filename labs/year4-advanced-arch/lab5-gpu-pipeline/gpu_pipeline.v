/*
 * Lab 5: GPU Graphics Pipeline
 * ============================
 * Build a simple 2D graphics pipeline accelerator targeting the badge's
 * 480x320 LCD. This module accepts drawing commands (lines, rectangles,
 * triangles) via a command FIFO and outputs individual pixel writes to
 * a framebuffer or display controller.
 *
 * The pipeline processes one command at a time:
 *
 *   Command FIFO -> Decode -> Rasterize -> Pixel Output
 *                              |
 *                   Line (Bresenham) / Rect Fill / Triangle (Edge Func)
 *
 * EXERCISES:
 *
 * Exercise 1: Line drawing with Bresenham's algorithm
 *   Implement the Bresenham FSM that, given start (x0,y0) and end (x1,y1),
 *   emits every pixel along the line. Handle all octants correctly.
 *   The algorithm avoids floating-point by tracking an integer error term:
 *     dx = |x1 - x0|, dy = |y1 - y0|
 *     error starts at dx/2 (or -dy/2 depending on variant)
 *     Each step: emit pixel, advance major axis, update error,
 *     conditionally advance minor axis.
 *
 * Exercise 2: Rectangle fill engine
 *   Given top-left (x0,y0) and dimensions (width=x1, height=y1), fill
 *   all pixels in the rectangle. Use a simple row-major scan pattern.
 *   This exercises nested counter design.
 *
 * Exercise 3: Triangle rasterizer using edge functions
 *   Given three vertices (x0,y0), (x1,y1), (x2,y2), rasterize the
 *   triangle using the edge function method. For each candidate pixel (px,py),
 *   compute three edge functions:
 *     E01 = (px - x0)*(y1 - y0) - (py - y0)*(x1 - x0)
 *     E12 = (px - x1)*(y2 - y1) - (py - y1)*(x2 - x1)
 *     E20 = (px - x2)*(y0 - y2) - (py - y2)*(x0 - x2)
 *   The pixel is inside the triangle if all three are >= 0 (or all <= 0).
 *   Scan the bounding box of the triangle and test each pixel.
 *
 * Exercise 4: Framebuffer DMA writer
 *   Implement a pixel output interface that writes rasterized pixels into
 *   a memory-mapped framebuffer. Handle backpressure (pixel_ready).
 *
 * Exercise 5 (challenge): Texture mapping
 *   Add a small texture memory (e.g., 16x16 texels). For triangle pixels,
 *   use barycentric coordinates to interpolate UV texture coordinates,
 *   read the texel, and use it as the pixel color.
 *
 * CONCEPTS:
 *   - Graphics pipeline stages (command processing, rasterization, output)
 *   - Rasterization: converting geometric primitives to pixels
 *   - Bresenham's line algorithm (integer-only, all octants)
 *   - Edge functions for triangle rasterization
 *   - Barycentric interpolation for attribute mapping
 *   - Framebuffer and DMA concepts
 *   - Pixel clock domains and display timing
 *   - 2D vs 3D pipeline comparison (this is a simplified 2D version)
 *   - GPU parallelism: why real GPUs process many pixels simultaneously
 */

`default_nettype none
`include "../common/ooo_defs.vh"

module gpu_pipeline (
    input  wire        clk,
    input  wire        rst,

    // Command interface
    input  wire        cmd_valid,       // New command available
    input  wire [2:0]  cmd_type,        // 3'b000=NOP, 3'b001=LINE, 3'b010=RECT, 3'b011=TRIANGLE
    input  wire [15:0] x0,              // Vertex 0 / start X
    input  wire [15:0] y0,              // Vertex 0 / start Y
    input  wire [15:0] x1,              // Vertex 1 / end X  (or width for RECT)
    input  wire [15:0] y1,              // Vertex 1 / end Y  (or height for RECT)
    input  wire [15:0] x2,              // Vertex 2 X (triangle only)
    input  wire [15:0] y2,              // Vertex 2 Y (triangle only)
    input  wire [15:0] color,           // RGB565 pixel color
    output wire        cmd_ready,       // Pipeline ready for next command

    // Pixel output interface (to framebuffer / display controller)
    output reg  [9:0]  pixel_x,         // 0..479
    output reg  [8:0]  pixel_y,         // 0..319
    output reg  [15:0] pixel_color,     // RGB565
    output reg         pixel_valid,     // Pixel output strobe
    input  wire        pixel_ready,     // Downstream backpressure

    // Status
    output wire        busy             // Pipeline is processing a command
);

    // =========================================================================
    // Display parameters
    // =========================================================================
    localparam DISPLAY_W = 480;
    localparam DISPLAY_H = 320;

    // Command types
    localparam CMD_NOP      = 3'b000;
    localparam CMD_LINE     = 3'b001;
    localparam CMD_RECT     = 3'b010;
    localparam CMD_TRIANGLE = 3'b011;

    // =========================================================================
    // FSM states
    // =========================================================================
    localparam [3:0] S_IDLE      = 4'd0,
                     S_CMD_LOAD  = 4'd1,
                     S_LINE_INIT = 4'd2,
                     S_LINE_STEP = 4'd3,
                     S_RECT_INIT = 4'd4,
                     S_RECT_FILL = 4'd5,
                     S_TRI_INIT  = 4'd6,
                     S_TRI_BBOX  = 4'd7,
                     S_TRI_TEST  = 4'd8,
                     S_TRI_EMIT  = 4'd9,
                     S_DONE      = 4'd10;

    reg [3:0] state, state_next;

    // =========================================================================
    // Command registers (latched on accept)
    // =========================================================================
    reg [2:0]  cmd_type_r;
    reg signed [15:0] cx0, cy0, cx1, cy1, cx2, cy2;
    reg [15:0] cmd_color;

    // =========================================================================
    // Bresenham's line drawing registers
    // =========================================================================
    // Exercise 1: You will use these registers
    reg signed [15:0] line_x, line_y;       // Current pixel position
    reg signed [15:0] line_dx, line_dy;     // Absolute deltas
    reg signed [15:0] line_sx, line_sy;     // Step direction (+1 or -1)
    reg signed [16:0] line_err;             // Error accumulator (needs extra bit)
    reg signed [16:0] line_e2;              // 2 * error (temporary)
    reg        line_done;                    // Reached endpoint

    // =========================================================================
    // Rectangle fill registers
    // =========================================================================
    reg [15:0] rect_cur_x, rect_cur_y;
    reg [15:0] rect_end_x, rect_end_y;     // x0+width-1, y0+height-1

    // =========================================================================
    // Triangle rasterizer registers
    // =========================================================================
    // Bounding box
    reg signed [15:0] tri_min_x, tri_min_y;
    reg signed [15:0] tri_max_x, tri_max_y;
    // Scan position
    reg signed [15:0] tri_px, tri_py;
    // Edge function values (need more bits for multiply results)
    reg signed [31:0] edge01, edge12, edge20;
    // Precomputed edge deltas for incremental evaluation
    reg signed [15:0] dy01, dy12, dy20;     // y-deltas of edges
    reg signed [15:0] dx01, dx12, dx20;     // x-deltas of edges
    wire tri_inside;

    // =========================================================================
    // Status signals
    // =========================================================================
    assign busy      = (state != S_IDLE);
    assign cmd_ready = (state == S_IDLE);

    // Triangle inside test: pixel is inside if all edge functions have same sign
    // TODO (Exercise 3): Implement the inside test
    // HINT: The pixel is inside if all three edge values are non-negative,
    //       or all three are non-positive (handles CW and CCW winding).
    assign tri_inside = (edge01 >= 0 && edge12 >= 0 && edge20 >= 0) ||
                        (edge01 <= 0 && edge12 <= 0 && edge20 <= 0);

    // =========================================================================
    // Helper functions
    // =========================================================================
    function signed [15:0] abs_val;
        input signed [15:0] val;
        abs_val = (val < 0) ? -val : val;
    endfunction

    function signed [15:0] min3;
        input signed [15:0] a, b, c;
        min3 = (a < b) ? ((a < c) ? a : c) : ((b < c) ? b : c);
    endfunction

    function signed [15:0] max3;
        input signed [15:0] a, b, c;
        max3 = (a > b) ? ((a > c) ? a : c) : ((b > c) ? b : c);
    endfunction

    function signed [15:0] clamp;
        input signed [15:0] val, lo, hi;
        clamp = (val < lo) ? lo : ((val > hi) ? hi : val);
    endfunction

    // =========================================================================
    // Main FSM - Sequential
    // =========================================================================
    always @(posedge clk) begin
        if (rst) begin
            state       <= S_IDLE;
            pixel_valid <= 1'b0;
            pixel_x     <= 10'd0;
            pixel_y     <= 9'd0;
            pixel_color <= 16'd0;
        end else begin
            // Default: deassert pixel_valid when accepted
            if (pixel_valid && pixel_ready)
                pixel_valid <= 1'b0;

            case (state)
                // ---------------------------------------------------------
                // IDLE: Wait for a command
                // ---------------------------------------------------------
                S_IDLE: begin
                    if (cmd_valid) begin
                        // Latch command parameters
                        cmd_type_r <= cmd_type;
                        cx0 <= x0; cy0 <= y0;
                        cx1 <= x1; cy1 <= y1;
                        cx2 <= x2; cy2 <= y2;
                        cmd_color  <= color;
                        state      <= S_CMD_LOAD;
                    end
                end

                // ---------------------------------------------------------
                // CMD_LOAD: Decode command and branch to rasterizer
                // ---------------------------------------------------------
                S_CMD_LOAD: begin
                    case (cmd_type_r)
                        CMD_LINE:     state <= S_LINE_INIT;
                        CMD_RECT:     state <= S_RECT_INIT;
                        CMD_TRIANGLE: state <= S_TRI_INIT;
                        default:      state <= S_IDLE;  // NOP or unknown
                    endcase
                end

                // =========================================================
                // LINE DRAWING - Bresenham's Algorithm
                // =========================================================

                // ---------------------------------------------------------
                // LINE_INIT: Compute Bresenham parameters
                // ---------------------------------------------------------
                S_LINE_INIT: begin
                    // TODO (Exercise 1): Initialize Bresenham's algorithm
                    //
                    // Step 1: Compute absolute deltas
                    //   line_dx <= abs_val(cx1 - cx0)
                    //   line_dy <= abs_val(cy1 - cy0)  -- NOTE: store as NEGATIVE for the algorithm
                    //
                    // Step 2: Compute step directions
                    //   line_sx <= (cx0 < cx1) ? 1 : -1
                    //   line_sy <= (cy0 < cy1) ? 1 : -1
                    //
                    // Step 3: Initialize error term
                    //   line_err <= dx + dy   (where dy is negative)
                    //
                    // Step 4: Set starting position
                    //   line_x <= cx0
                    //   line_y <= cy0

                    line_dx  <= abs_val(cx1 - cx0);
                    line_dy  <= -abs_val(cy1 - cy0);    // Negative for algorithm
                    line_sx  <= (cx0 < cx1) ? 16'sd1 : -16'sd1;
                    line_sy  <= (cy0 < cy1) ? 16'sd1 : -16'sd1;
                    line_err <= $signed({1'b0, abs_val(cx1 - cx0)}) + $signed({abs_val(cy1 - cy0)[15], -abs_val(cy1 - cy0)});
                    line_x   <= cx0;
                    line_y   <= cy0;
                    line_done <= 1'b0;

                    state <= S_LINE_STEP;
                end

                // ---------------------------------------------------------
                // LINE_STEP: Emit pixel and step along line
                // ---------------------------------------------------------
                S_LINE_STEP: begin
                    if (line_done) begin
                        state <= S_IDLE;
                    end else if (!pixel_valid || pixel_ready) begin
                        // TODO (Exercise 1): Implement Bresenham stepping
                        //
                        // Step 1: Emit current pixel
                        //   pixel_x     <= line_x[9:0]
                        //   pixel_y     <= line_y[8:0]
                        //   pixel_color <= cmd_color
                        //   pixel_valid <= 1'b1
                        //
                        // Step 2: Check if we've reached the endpoint
                        //   if (line_x == cx1 && line_y == cy1) line_done <= 1
                        //
                        // Step 3: Compute e2 = 2 * line_err
                        //   If e2 >= line_dy: err += dy, x += sx
                        //   If e2 <= line_dx: err += dx, y += sy
                        //
                        // The beauty of Bresenham: only integer adds and compares!

                        // Emit current pixel
                        pixel_x     <= line_x[9:0];
                        pixel_y     <= line_y[8:0];
                        pixel_color <= cmd_color;
                        pixel_valid <= 1'b1;

                        // Check endpoint
                        if (line_x == cx1 && line_y == cy1) begin
                            line_done <= 1'b1;
                        end else begin
                            // Bresenham step
                            line_e2 = {line_err[16], line_err} <<< 1;  // e2 = 2*err

                            if (line_e2 >= {line_dy[15], line_dy[15:0]}) begin
                                line_err <= line_err + {line_dy[15], line_dy};
                                line_x   <= line_x + line_sx;
                            end

                            if (line_e2 <= {1'b0, line_dx}) begin
                                line_err <= line_err + {1'b0, line_dx};
                                line_y   <= line_y + line_sy;
                            end
                        end
                    end
                    // else: wait for pixel_ready (backpressure)
                end

                // =========================================================
                // RECTANGLE FILL
                // =========================================================

                // ---------------------------------------------------------
                // RECT_INIT: Set up rectangle scan bounds
                // ---------------------------------------------------------
                S_RECT_INIT: begin
                    // TODO (Exercise 2): Initialize rectangle fill
                    //
                    // cx0, cy0 = top-left corner
                    // cx1 = width, cy1 = height
                    //
                    // Set current position to top-left
                    // Compute end position: (cx0 + cx1 - 1, cy0 + cy1 - 1)
                    // Clamp to display bounds

                    rect_cur_x <= cx0;
                    rect_cur_y <= cy0;
                    rect_end_x <= clamp(cx0 + cx1 - 16'sd1, 16'sd0, DISPLAY_W - 1);
                    rect_end_y <= clamp(cy0 + cy1 - 16'sd1, 16'sd0, DISPLAY_H - 1);

                    state <= S_RECT_FILL;
                end

                // ---------------------------------------------------------
                // RECT_FILL: Scan row-major and emit pixels
                // ---------------------------------------------------------
                S_RECT_FILL: begin
                    if (!pixel_valid || pixel_ready) begin
                        // TODO (Exercise 2): Implement rectangle scanning
                        //
                        // Emit pixel at (rect_cur_x, rect_cur_y)
                        // Advance: x++; if x > end_x, x = start_x, y++
                        // Done when y > end_y

                        pixel_x     <= rect_cur_x[9:0];
                        pixel_y     <= rect_cur_y[8:0];
                        pixel_color <= cmd_color;
                        pixel_valid <= 1'b1;

                        if (rect_cur_x == rect_end_x) begin
                            rect_cur_x <= cx0;
                            if (rect_cur_y == rect_end_y) begin
                                state <= S_IDLE;
                            end else begin
                                rect_cur_y <= rect_cur_y + 16'd1;
                            end
                        end else begin
                            rect_cur_x <= rect_cur_x + 16'd1;
                        end
                    end
                end

                // =========================================================
                // TRIANGLE RASTERIZATION - Edge Function Method
                // =========================================================

                // ---------------------------------------------------------
                // TRI_INIT: Precompute edge deltas
                // ---------------------------------------------------------
                S_TRI_INIT: begin
                    // TODO (Exercise 3): Precompute edge parameters
                    //
                    // For incremental edge function evaluation, we need:
                    //   dy01 = cy1 - cy0    (y-delta of edge 0->1)
                    //   dx01 = cx1 - cx0    (x-delta of edge 0->1)
                    //   ... and similarly for edges 1->2 and 2->0
                    //
                    // These allow us to step the edge functions by adding
                    // dy when we step in x, or subtracting dx when we step in y.

                    dy01 <= cy1 - cy0;
                    dx01 <= cx1 - cx0;
                    dy12 <= cy2 - cy1;
                    dx12 <= cx2 - cx1;
                    dy20 <= cy0 - cy2;
                    dx20 <= cx0 - cx2;

                    state <= S_TRI_BBOX;
                end

                // ---------------------------------------------------------
                // TRI_BBOX: Compute bounding box and initial edge values
                // ---------------------------------------------------------
                S_TRI_BBOX: begin
                    // TODO (Exercise 3): Compute bounding box of triangle
                    //
                    // min_x = max(0, min(x0, x1, x2))
                    // min_y = max(0, min(y0, y1, y2))
                    // max_x = min(DISPLAY_W-1, max(x0, x1, x2))
                    // max_y = min(DISPLAY_H-1, max(y0, y1, y2))
                    //
                    // Then compute initial edge function values at (min_x, min_y):
                    //   E01 = (min_x - cx0) * dy01 - (min_y - cy0) * dx01
                    //   E12 = (min_x - cx1) * dy12 - (min_y - cy1) * dx12
                    //   E20 = (min_x - cx2) * dy20 - (min_y - cy2) * dx20

                    tri_min_x <= clamp(min3(cx0, cx1, cx2), 16'sd0, DISPLAY_W - 1);
                    tri_min_y <= clamp(min3(cy0, cy1, cy2), 16'sd0, DISPLAY_H - 1);
                    tri_max_x <= clamp(max3(cx0, cx1, cx2), 16'sd0, DISPLAY_W - 1);
                    tri_max_y <= clamp(max3(cy0, cy1, cy2), 16'sd0, DISPLAY_H - 1);

                    // Initial edge function values computed at (min_x, min_y) next cycle
                    tri_px <= clamp(min3(cx0, cx1, cx2), 16'sd0, DISPLAY_W - 1);
                    tri_py <= clamp(min3(cy0, cy1, cy2), 16'sd0, DISPLAY_H - 1);

                    state <= S_TRI_TEST;
                end

                // ---------------------------------------------------------
                // TRI_TEST: Evaluate edge functions for current pixel
                // ---------------------------------------------------------
                S_TRI_TEST: begin
                    // TODO (Exercise 3): Compute edge functions at (tri_px, tri_py)
                    //
                    // For full (non-incremental) evaluation each cycle:
                    //   edge01 = (tri_px - cx0) * dy01 - (tri_py - cy0) * dx01
                    //   edge12 = (tri_px - cx1) * dy12 - (tri_py - cy1) * dx12
                    //   edge20 = (tri_px - cx2) * dy20 - (tri_py - cy2) * dx20
                    //
                    // OPTIMIZATION: For incremental evaluation, you can step
                    // edge values by +dy when moving +1 in x, and -dx when
                    // moving +1 in y. This replaces multiplies with adds.

                    edge01 <= (tri_px - cx0) * dy01 - (tri_py - cy0) * dx01;
                    edge12 <= (tri_px - cx1) * dy12 - (tri_py - cy1) * dx12;
                    edge20 <= (tri_px - cx2) * dy20 - (tri_py - cy2) * dx20;

                    state <= S_TRI_EMIT;
                end

                // ---------------------------------------------------------
                // TRI_EMIT: Output pixel if inside, advance scan position
                // ---------------------------------------------------------
                S_TRI_EMIT: begin
                    if (!pixel_valid || pixel_ready) begin
                        // Emit pixel if inside triangle
                        if (tri_inside) begin
                            pixel_x     <= tri_px[9:0];
                            pixel_y     <= tri_py[8:0];
                            pixel_color <= cmd_color;
                            pixel_valid <= 1'b1;
                        end

                        // Advance scan position (row-major across bounding box)
                        if (tri_px == tri_max_x) begin
                            tri_px <= tri_min_x;
                            if (tri_py == tri_max_y) begin
                                state <= S_IDLE;    // Done
                            end else begin
                                tri_py <= tri_py + 16'sd1;
                                state  <= S_TRI_TEST;
                            end
                        end else begin
                            tri_px <= tri_px + 16'sd1;
                            state  <= S_TRI_TEST;
                        end
                    end
                    // else: wait for backpressure
                end

                // ---------------------------------------------------------
                // DONE: (reserved for multi-cycle completion)
                // ---------------------------------------------------------
                S_DONE: begin
                    state <= S_IDLE;
                end

                default: state <= S_IDLE;
            endcase
        end
    end

    // =========================================================================
    // Performance counters (for Exercise 4 / debug)
    // =========================================================================
    reg [31:0] perf_cmd_count;      // Total commands processed
    reg [31:0] perf_pixel_count;    // Total pixels emitted
    reg [31:0] perf_cycle_count;    // Total cycles spent busy
    reg [31:0] perf_stall_count;    // Cycles stalled on backpressure

    always @(posedge clk) begin
        if (rst) begin
            perf_cmd_count   <= 32'd0;
            perf_pixel_count <= 32'd0;
            perf_cycle_count <= 32'd0;
            perf_stall_count <= 32'd0;
        end else begin
            if (state == S_IDLE && cmd_valid && cmd_ready)
                perf_cmd_count <= perf_cmd_count + 32'd1;

            if (pixel_valid && pixel_ready)
                perf_pixel_count <= perf_pixel_count + 32'd1;

            if (busy)
                perf_cycle_count <= perf_cycle_count + 32'd1;

            if (pixel_valid && !pixel_ready)
                perf_stall_count <= perf_stall_count + 32'd1;
        end
    end

    // =========================================================================
    // TODO (Exercise 5 - Challenge): Texture mapping
    // =========================================================================
    //
    // Add a small texture memory and UV interpolation for triangles:
    //
    // 1. Declare a texture RAM (e.g., 16x16 texels, RGB565):
    //      reg [15:0] tex_mem [0:255];  // 16x16 texture
    //
    // 2. For each triangle vertex, accept UV coordinates (u0,v0), (u1,v1), (u2,v2)
    //
    // 3. At each interior pixel, compute barycentric weights from edge functions:
    //      w0 = edge12 / (edge01_at_v0)   -- normalized weight for vertex 0
    //      w1 = edge20 / ...               -- normalized weight for vertex 1
    //      w2 = edge01 / ...               -- normalized weight for vertex 2
    //
    // 4. Interpolate UV: u = w0*u0 + w1*u1 + w2*u2  (same for v)
    //
    // 5. Read texel: pixel_color = tex_mem[{v[3:0], u[3:0]}]
    //
    // NOTE: Division for normalization is expensive in hardware. Consider
    //       using the un-normalized edge values and a reciprocal LUT, or
    //       simply scaling by a power of 2.
    //
    // reg [15:0] tex_mem [0:255];
    // initial $readmemh("texture.hex", tex_mem);

endmodule
