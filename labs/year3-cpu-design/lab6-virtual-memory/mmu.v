/*
 * Lab 6: Virtual Memory - Memory Management Unit
 * ================================================
 * Implement a simple MMU with a TLB for RISC-V Sv32 paging.
 *
 * In Sv32 a 32-bit virtual address is split into:
 *   VPN[1] (bits 31:22) -- indexes the root page table
 *   VPN[0] (bits 21:12) -- indexes the second-level page table
 *   Offset  (bits 11:0) -- byte offset within a 4 KiB page
 *
 * A Page Table Entry (PTE) is 32 bits:
 *   PPN[1] (bits 31:20), PPN[0] (bits 19:10), RSW (bits 9:8),
 *   D (7), A (6), G (5), U (4), X (3), W (2), R (1), V (0)
 *
 * The TLB caches recent translations so that most accesses avoid
 * the two-level page table walk entirely.
 *
 * EXERCISES:
 *
 * Exercise 1: Implement a 16-entry fully-associative TLB
 *   Store VPN, PPN, ASID, permission flags, and a valid bit per
 *   entry.  Write the parallel comparison logic for hit detection.
 *
 * Exercise 2: Implement TLB lookup with hit/miss detection
 *   On a translation request, check all 16 entries in parallel.
 *   On a hit, return the physical address in the same cycle.
 *   On a miss, launch the page table walker.
 *
 * Exercise 3: Implement the page table walker (2-level Sv32)
 *   Walk from the root page table (address in satp.PPN) through
 *   up to two levels of page tables, reading PTEs from memory.
 *   Handle superpage detection (leaf at level 1).
 *
 * Exercise 4: Add TLB replacement policy
 *   Implement round-robin (simple) or pseudo-LRU replacement to
 *   choose which TLB entry to evict on a miss.
 *
 * Exercise 5 (Challenge): Permission checking and page faults
 *   Check R/W/X against access_type and U against privilege_mode.
 *   Assert page_fault when permissions are violated, the V bit is
 *   clear, or a PTE has W=1 but R=0 (reserved combination).
 *
 * CONCEPTS:
 *   - Virtual vs physical addresses and address spaces
 *   - Page tables: hierarchical translation structures
 *   - TLB (Translation Lookaside Buffer): caching translations
 *   - Page faults: hardware signals, software handles
 *   - Memory protection: per-page R/W/X and U/S permissions
 *   - Sv32 PTE format: PPN[1:0], RSW, D, A, G, U, X, W, R, V
 *   - ASID (Address Space Identifier): avoiding TLB flushes
 */

`default_nettype none
`include "../common/riscv_defs.vh"

module mmu (
    input  wire        clk,
    input  wire        rst,

    // --- CPU translation request ---
    input  wire [31:0] virt_addr,         // Virtual address from CPU
    output reg  [31:0] phys_addr,         // Translated physical address
    input  wire        translate_req,     // Pulse to start translation
    output reg         translate_done,    // High for one cycle when result ready
    output reg         page_fault,        // High for one cycle on fault

    // --- Access information ---
    input  wire [1:0]  access_type,       // 2'b00=read, 2'b01=write, 2'b10=exec
    input  wire        privilege_mode,    // 0=user, 1=supervisor

    // --- SATP register (from CSR file) ---
    input  wire [31:0] satp,             // satp: bit31=MODE, 30:22=ASID, 21:0=PPN

    // --- Memory interface (for page table walks) ---
    output reg         mem_req,           // Request a memory read
    output reg  [31:0] mem_addr,          // Address to read from memory
    input  wire [31:0] mem_data,          // Data returned from memory
    input  wire        mem_ready          // Memory read complete
);

    // ================================================================
    // SATP field extraction
    // ================================================================
    wire        satp_mode = satp[31];             // 0=bare (no translation), 1=Sv32
    wire [8:0]  satp_asid = satp[30:22];
    wire [21:0] satp_ppn  = satp[21:0];

    // Virtual address field extraction
    wire [9:0]  vpn1   = virt_addr[31:22];        // Level-1 VPN
    wire [9:0]  vpn0   = virt_addr[21:12];        // Level-0 VPN
    wire [11:0] pg_off = virt_addr[11:0];         // Page offset

    // PTE field extraction (from a fetched PTE word)
    wire [11:0] pte_ppn1  = mem_data[31:20];
    wire [9:0]  pte_ppn0  = mem_data[19:10];
    wire        pte_dirty = mem_data[7];
    wire        pte_acc   = mem_data[6];
    wire        pte_glob  = mem_data[5];
    wire        pte_user  = mem_data[4];
    wire        pte_x     = mem_data[3];
    wire        pte_w     = mem_data[2];
    wire        pte_r     = mem_data[1];
    wire        pte_v     = mem_data[0];

    // A PTE is a leaf if at least one of R, W, X is set
    wire pte_is_leaf = pte_r | pte_w | pte_x;

    // ================================================================
    // Section 1 -- TLB Storage (16 entries, fully associative)
    // ================================================================

    localparam TLB_ENTRIES = 16;

    reg [19:0] tlb_vpn   [0:TLB_ENTRIES-1]; // VPN[1] ++ VPN[0]
    reg [21:0] tlb_ppn   [0:TLB_ENTRIES-1]; // PPN[1] ++ PPN[0]
    reg [8:0]  tlb_asid  [0:TLB_ENTRIES-1];
    reg        tlb_valid [0:TLB_ENTRIES-1];
    reg        tlb_glob  [0:TLB_ENTRIES-1]; // Global page (matches any ASID)
    // Permission bits stored per entry
    reg        tlb_r     [0:TLB_ENTRIES-1];
    reg        tlb_w     [0:TLB_ENTRIES-1];
    reg        tlb_x     [0:TLB_ENTRIES-1];
    reg        tlb_u     [0:TLB_ENTRIES-1]; // User-accessible

    // Round-robin replacement pointer
    reg [3:0] tlb_replace_ptr;

    // ================================================================
    // Section 2 -- TLB Lookup (parallel comparison)
    // ================================================================

    wire [19:0] lookup_vpn = {vpn1, vpn0};
    reg         tlb_hit;
    reg  [3:0]  tlb_hit_idx;
    reg  [21:0] tlb_hit_ppn;
    reg         tlb_hit_r, tlb_hit_w, tlb_hit_x, tlb_hit_u;

    // ------------------------------------------------------------
    // TODO (Exercise 2): Implement TLB lookup.
    //   - Compare lookup_vpn against all 16 tlb_vpn entries.
    //   - Also check that tlb_valid is set and that the ASID
    //     matches satp_asid (or tlb_glob is set).
    //   - Set tlb_hit, tlb_hit_idx, tlb_hit_ppn, and the
    //     permission signals for the matching entry.
    //
    // Hint: Use a combinational always block with a for-loop.
    //   If multiple entries match, take the first one (lowest index).
    // ------------------------------------------------------------
    integer j;

    always @(*) begin
        tlb_hit     = 1'b0;
        tlb_hit_idx = 4'd0;
        tlb_hit_ppn = 22'd0;
        tlb_hit_r   = 1'b0;
        tlb_hit_w   = 1'b0;
        tlb_hit_x   = 1'b0;
        tlb_hit_u   = 1'b0;

        // TODO: Parallel TLB comparison logic
        // for (j = 0; j < TLB_ENTRIES; j = j + 1) begin
        //     if (!tlb_hit && tlb_valid[j] &&
        //         (tlb_vpn[j] == lookup_vpn) &&
        //         (tlb_glob[j] || tlb_asid[j] == satp_asid)) begin
        //         tlb_hit     = 1'b1;
        //         tlb_hit_idx = j[3:0];
        //         tlb_hit_ppn = tlb_ppn[j];
        //         tlb_hit_r   = tlb_r[j];
        //         tlb_hit_w   = tlb_w[j];
        //         tlb_hit_x   = tlb_x[j];
        //         tlb_hit_u   = tlb_u[j];
        //     end
        // end
    end

    // ================================================================
    // Section 3 -- Page Table Walker FSM
    // ================================================================

    localparam PTW_IDLE         = 3'd0;
    localparam PTW_LEVEL1_FETCH = 3'd1;  // Fetch PTE from root page table
    localparam PTW_LEVEL1_WAIT  = 3'd2;  // Wait for memory response
    localparam PTW_LEVEL0_FETCH = 3'd3;  // Fetch PTE from second-level table
    localparam PTW_LEVEL0_WAIT  = 3'd4;  // Wait for memory response
    localparam PTW_UPDATE_TLB   = 3'd5;  // Write result into TLB
    localparam PTW_FAULT        = 3'd6;  // Page fault detected

    reg [2:0] ptw_state;

    // Registers to hold intermediate walk results
    reg [31:0] fetched_pte;       // Last PTE fetched from memory
    reg [21:0] result_ppn;        // Physical page number from walk
    reg        result_r, result_w, result_x, result_u, result_g;
    reg        is_superpage;      // Leaf found at level 1 (4 MiB page)

    // Saved request information
    reg [31:0] saved_vaddr;
    reg [1:0]  saved_access;
    reg        saved_priv;

    integer k;

    always @(posedge clk) begin
        if (rst) begin
            ptw_state      <= PTW_IDLE;
            translate_done <= 1'b0;
            page_fault     <= 1'b0;
            mem_req        <= 1'b0;
            phys_addr      <= 32'b0;
            tlb_replace_ptr <= 4'd0;
            is_superpage   <= 1'b0;

            for (k = 0; k < TLB_ENTRIES; k = k + 1) begin
                tlb_valid[k] <= 1'b0;
            end
        end else begin
            // Default: deassert single-cycle signals
            translate_done <= 1'b0;
            page_fault     <= 1'b0;
            mem_req        <= 1'b0;

            case (ptw_state)
                // --------------------------------------------------
                PTW_IDLE: begin
                    if (translate_req) begin
                        // Save request context
                        saved_vaddr  <= virt_addr;
                        saved_access <= access_type;
                        saved_priv   <= privilege_mode;

                        // If translation is disabled (bare mode), pass through
                        if (!satp_mode) begin
                            phys_addr      <= virt_addr;
                            translate_done <= 1'b1;
                        end
                        // ------------------------------------------------
                        // TODO (Exercise 2): Check TLB first.
                        //   If tlb_hit, produce physical address
                        //   immediately and pulse translate_done.
                        //
                        //   phys_addr <= {tlb_hit_ppn, pg_off};
                        //
                        //   (Exercise 5): Also check permissions here.
                        //   If permissions fail, assert page_fault.
                        // ------------------------------------------------
                        else if (tlb_hit) begin
                            // TODO: Return cached translation
                            phys_addr      <= 32'b0; // placeholder
                            translate_done <= 1'b1;
                        end
                        // TLB miss -- start page table walk
                        else begin
                            ptw_state <= PTW_LEVEL1_FETCH;
                        end
                    end
                end

                // --------------------------------------------------
                // Level 1: fetch PTE from root page table
                // Address = satp.PPN * 4096 + VPN[1] * 4
                // --------------------------------------------------
                PTW_LEVEL1_FETCH: begin
                    // ----------------------------------------------------
                    // TODO (Exercise 3): Compute the level-1 PTE address
                    //   and issue a memory read request.
                    //
                    //   mem_addr <= {satp_ppn, 12'b0} + {vpn1_of_saved, 2'b00};
                    //   mem_req  <= 1'b1;
                    // ----------------------------------------------------
                    mem_addr <= {satp_ppn, 12'b0} + {20'b0, saved_vaddr[31:22], 2'b00};
                    mem_req  <= 1'b1;
                    ptw_state <= PTW_LEVEL1_WAIT;
                end

                PTW_LEVEL1_WAIT: begin
                    if (mem_ready) begin
                        fetched_pte <= mem_data;

                        // -------------------------------------------------
                        // TODO (Exercise 3): Interpret the level-1 PTE.
                        //   - If !pte_v → page fault.
                        //   - If pte_is_leaf → superpage; record PPN and
                        //     go to PTW_UPDATE_TLB.
                        //   - Else → pointer to level-0 table; go to
                        //     PTW_LEVEL0_FETCH.
                        //
                        // (Exercise 5): Also fault if pte_w && !pte_r.
                        // -------------------------------------------------
                        if (!pte_v) begin
                            ptw_state <= PTW_FAULT;
                        end else if (pte_is_leaf) begin
                            // Superpage (4 MiB)
                            result_ppn <= {pte_ppn1, pte_ppn0};
                            result_r   <= pte_r;
                            result_w   <= pte_w;
                            result_x   <= pte_x;
                            result_u   <= pte_user;
                            result_g   <= pte_glob;
                            is_superpage <= 1'b1;
                            ptw_state  <= PTW_UPDATE_TLB;
                        end else begin
                            is_superpage <= 1'b0;
                            ptw_state    <= PTW_LEVEL0_FETCH;
                        end
                    end
                end

                // --------------------------------------------------
                // Level 0: fetch PTE from second-level page table
                // Address = PTE.PPN * 4096 + VPN[0] * 4
                // --------------------------------------------------
                PTW_LEVEL0_FETCH: begin
                    // ----------------------------------------------------
                    // TODO (Exercise 3): Compute the level-0 PTE address.
                    //   Use the PPN from the level-1 PTE (fetched_pte).
                    //
                    //   mem_addr <= {fetched_pte[31:10], 12'b0}
                    //             + {vpn0_of_saved, 2'b00};
                    //   mem_req  <= 1'b1;
                    // ----------------------------------------------------
                    mem_addr <= {fetched_pte[31:10], 12'b0}
                              + {20'b0, saved_vaddr[21:12], 2'b00};
                    mem_req  <= 1'b1;
                    ptw_state <= PTW_LEVEL0_WAIT;
                end

                PTW_LEVEL0_WAIT: begin
                    if (mem_ready) begin
                        fetched_pte <= mem_data;

                        // -------------------------------------------------
                        // TODO (Exercise 3): Interpret the level-0 PTE.
                        //   - If !pte_v or !pte_is_leaf → page fault.
                        //   - Otherwise record PPN and go to PTW_UPDATE_TLB.
                        // -------------------------------------------------
                        if (!pte_v || !pte_is_leaf) begin
                            ptw_state <= PTW_FAULT;
                        end else begin
                            result_ppn <= {pte_ppn1, pte_ppn0};
                            result_r   <= pte_r;
                            result_w   <= pte_w;
                            result_x   <= pte_x;
                            result_u   <= pte_user;
                            result_g   <= pte_glob;
                            ptw_state  <= PTW_UPDATE_TLB;
                        end
                    end
                end

                // --------------------------------------------------
                // Update TLB with the walk result
                // --------------------------------------------------
                PTW_UPDATE_TLB: begin
                    // ------------------------------------------------
                    // TODO (Exercise 4): Write the result into the TLB
                    //   at position tlb_replace_ptr.
                    //
                    //   tlb_vpn  [tlb_replace_ptr] <= ...
                    //   tlb_ppn  [tlb_replace_ptr] <= result_ppn;
                    //   tlb_asid [tlb_replace_ptr] <= satp_asid;
                    //   tlb_valid[tlb_replace_ptr] <= 1'b1;
                    //   (and permission bits)
                    //
                    //   Advance tlb_replace_ptr (round-robin):
                    //   tlb_replace_ptr <= tlb_replace_ptr + 1;
                    // ------------------------------------------------
                    // TODO: Write TLB entry here

                    // ------------------------------------------------
                    // TODO (Exercise 5): Check permissions before
                    //   signalling translate_done.  If the access type
                    //   is not permitted, assert page_fault instead.
                    //
                    //   Read:  requires result_r
                    //   Write: requires result_w
                    //   Exec:  requires result_x
                    //   User mode accessing !result_u → fault
                    // ------------------------------------------------

                    // Produce physical address
                    if (is_superpage) begin
                        // Superpage: PPN[1] from PTE, VPN[0] passes through
                        phys_addr <= {result_ppn[21:10], saved_vaddr[21:0]};
                    end else begin
                        phys_addr <= {result_ppn, saved_vaddr[11:0]};
                    end

                    translate_done <= 1'b1;
                    ptw_state      <= PTW_IDLE;
                end

                // --------------------------------------------------
                // Page fault
                // --------------------------------------------------
                PTW_FAULT: begin
                    page_fault <= 1'b1;
                    phys_addr  <= 32'b0;
                    ptw_state  <= PTW_IDLE;
                end

                default: ptw_state <= PTW_IDLE;
            endcase
        end
    end

    // ================================================================
    // Debug / Verification Helpers
    // ================================================================
`ifdef FORMAL
    // Physical address must be stable when translate_done is high
    always @(posedge clk) begin
        if (translate_done && !page_fault)
            assert(phys_addr != 32'b0 || saved_vaddr[11:0] == 12'b0);
    end

    // translate_done and page_fault must not be asserted simultaneously
    always @(*) begin
        assert(!(translate_done && page_fault));
    end

    // TLB replace pointer must be in range
    always @(*) begin
        assert(tlb_replace_ptr < TLB_ENTRIES);
    end
`endif

endmodule
