/*
 * Lab 7: Multicore - Bus Interconnect and Cache Coherency
 * ========================================================
 * Build a bus interconnect for a dual-core system with snooping
 * cache coherency based on the MSI (Modified/Shared/Invalid)
 * protocol.
 *
 * Two CPU cores share a single memory bus.  A round-robin arbiter
 * decides which core gains access each cycle.  Every bus transaction
 * is visible to both cores; a snoop controller monitors addresses
 * and transitions its local cache-line state according to the MSI
 * protocol to keep memory coherent.
 *
 * EXERCISES:
 *
 * Exercise 1: Implement a round-robin bus arbiter for 2 masters
 *   The arbiter must grant the bus to one core at a time.  If both
 *   request simultaneously, alternate between them each grant.
 *
 * Exercise 2: Implement bus snooping logic
 *   Each core observes every bus transaction.  When the other core
 *   reads or writes an address that this core holds in its cache,
 *   the snoop controller must update the local cache-line state.
 *
 * Exercise 3: Implement the MSI cache coherency protocol
 *   Each tracked cache line has one of three states:
 *     INVALID  - not present in this cache
 *     SHARED   - clean copy, may also be in other caches
 *     MODIFIED - dirty copy, only valid copy in the system
 *   Implement the state transitions for BusRd, BusRdX (read-
 *   exclusive / write), and BusWB (write-back) transactions.
 *
 * Exercise 4: Add shared memory region detection
 *   Only snoop addresses in the shared region (0x10000000-0x1FFFFFFF).
 *   Private and I/O addresses bypass coherency for better performance.
 *
 * Exercise 5 (Challenge): Implement the MESI protocol
 *   Add an EXCLUSIVE state (clean, sole owner) that allows silent
 *   upgrade to MODIFIED without a bus transaction.  This reduces
 *   write-miss traffic for data not shared between cores.
 *
 * CONCEPTS:
 *   - Multicore challenges: shared mutable state
 *   - Bus arbitration: fair, starvation-free scheduling
 *   - Cache coherency problem: stale data in private caches
 *   - Snooping protocols: every cache watches the bus
 *   - MSI/MESI state machines per cache line
 *   - False sharing: two cores touch different words in same line
 *   - Memory consistency models: when are writes visible?
 *   - Bus bandwidth bottleneck: why snooping does not scale
 */

`default_nettype none
`include "../common/riscv_defs.vh"

module bus_interconnect (
    input  wire        clk,
    input  wire        rst,

    // --- Core 0 bus master interface ---
    input  wire        core0_req,
    input  wire [31:0] core0_addr,
    input  wire [31:0] core0_wdata,
    output reg  [31:0] core0_rdata,
    input  wire        core0_wen,       // 1=write, 0=read
    output reg         core0_grant,

    // --- Core 1 bus master interface ---
    input  wire        core1_req,
    input  wire [31:0] core1_addr,
    input  wire [31:0] core1_wdata,
    output reg  [31:0] core1_rdata,
    input  wire        core1_wen,
    output reg         core1_grant,

    // --- Shared memory interface ---
    output reg         mem_req,
    output reg  [31:0] mem_addr,
    output reg  [31:0] mem_wdata,
    input  wire [31:0] mem_rdata,
    output reg         mem_wen,
    input  wire        mem_ready,

    // --- Snoop status (directly observable for testing) ---
    output wire        snoop_hit,       // Snoop found a matching line
    output wire [1:0]  snoop_state      // MSI state of snooped line
);

    // ================================================================
    // MSI State Encoding
    // ================================================================
    localparam MSI_INVALID  = 2'd0;
    localparam MSI_SHARED   = 2'd1;
    localparam MSI_MODIFIED = 2'd2;
    // (Exercise 5) localparam MSI_EXCLUSIVE = 2'd3;  // for MESI

    // Bus transaction types (internal encoding)
    localparam BUS_NONE = 2'd0;
    localparam BUS_RD   = 2'd1;  // Read -- requester wants shared copy
    localparam BUS_RDX  = 2'd2;  // Read-exclusive -- requester will write
    localparam BUS_WB   = 2'd3;  // Write-back -- evicting modified line

    // ================================================================
    // Section 1 -- Round-Robin Bus Arbiter
    // ================================================================

    // Last-granted core (used to break ties fairly)
    reg last_grant;  // 0 = last grant to core0, 1 = core1

    // Active grant signals (active for the duration of a bus transaction)
    reg       bus_busy;
    reg       active_core;   // Which core currently owns the bus
    reg [1:0] bus_txn_type;  // Current transaction type

    // Derived signals for the current bus owner
    wire [31:0] active_addr  = active_core ? core1_addr  : core0_addr;
    wire [31:0] active_wdata = active_core ? core1_wdata : core0_wdata;
    wire        active_wen   = active_core ? core1_wen   : core0_wen;

    // -----------------------------------------------------------------
    // TODO (Exercise 1): Implement the round-robin arbiter.
    //
    //   When the bus is free (bus_busy == 0):
    //     - If only one core is requesting, grant it.
    //     - If both cores are requesting, grant the one that was NOT
    //       granted last time (use last_grant to decide).
    //     - Latch the granted core into active_core, set bus_busy.
    //
    //   When the bus is busy:
    //     - Keep the current grant until the memory responds
    //       (mem_ready) or the transaction completes.
    //     - On completion, release the bus (bus_busy <= 0) and
    //       update last_grant.
    // -----------------------------------------------------------------
    always @(posedge clk) begin
        if (rst) begin
            bus_busy    <= 1'b0;
            active_core <= 1'b0;
            last_grant  <= 1'b0;
            core0_grant <= 1'b0;
            core1_grant <= 1'b0;
            bus_txn_type <= BUS_NONE;
        end else begin
            core0_grant <= 1'b0;
            core1_grant <= 1'b0;

            if (!bus_busy) begin
                // -------------------------------------------------
                // TODO: Arbitration logic here
                //   Determine which core to grant based on requests
                //   and last_grant.
                //
                //   Example skeleton:
                //   if (core0_req && core1_req) begin
                //       active_core <= last_grant;  // round-robin
                //   end else if (core0_req) begin
                //       active_core <= 1'b0;
                //   end else if (core1_req) begin
                //       active_core <= 1'b1;
                //   end
                //
                //   if (core0_req || core1_req) begin
                //       bus_busy <= 1'b1;
                //       bus_txn_type <= (next_core_wen) ? BUS_RDX : BUS_RD;
                //       if (active_core == 0) core0_grant <= 1;
                //       else                  core1_grant <= 1;
                //   end
                // -------------------------------------------------
            end else begin
                // Bus transaction in progress
                if (mem_ready) begin
                    // -------------------------------------------------
                    // TODO: Complete the transaction
                    //   - Route mem_rdata to the requesting core
                    //   - Release the bus
                    //   - Update last_grant
                    // -------------------------------------------------
                    bus_busy <= 1'b0;  // placeholder
                end
            end
        end
    end

    // ================================================================
    // Section 2 -- Bus-to-Memory Routing
    // ================================================================
    // Route the active core's address/data to the memory port.

    always @(*) begin
        mem_req   = 1'b0;
        mem_addr  = 32'b0;
        mem_wdata = 32'b0;
        mem_wen   = 1'b0;

        if (bus_busy) begin
            mem_req   = 1'b1;
            mem_addr  = active_addr;
            mem_wdata = active_wdata;
            mem_wen   = active_wen;
        end
    end

    // Route read data back to the requesting core
    always @(*) begin
        core0_rdata = 32'b0;
        core1_rdata = 32'b0;

        if (bus_busy && mem_ready) begin
            if (active_core == 1'b0)
                core0_rdata = mem_rdata;
            else
                core1_rdata = mem_rdata;
        end
    end

    // ================================================================
    // Section 3 -- Snoop Controller and MSI State Machine
    // ================================================================
    // We track MSI state for a small number of cache lines per core.
    // For simplicity, we model a direct-mapped tag store (like a
    // cache tag array with MSI state instead of data).

    localparam SNOOP_LINES = 64;
    localparam SNOOP_IDX_BITS = 6;  // $clog2(64)

    // Tag + state arrays for core 0 and core 1
    reg [31:SNOOP_IDX_BITS+2] core0_tags  [0:SNOOP_LINES-1];
    reg [1:0]                  core0_msi   [0:SNOOP_LINES-1];
    reg                        core0_tvalid[0:SNOOP_LINES-1];

    reg [31:SNOOP_IDX_BITS+2] core1_tags  [0:SNOOP_LINES-1];
    reg [1:0]                  core1_msi   [0:SNOOP_LINES-1];
    reg                        core1_tvalid[0:SNOOP_LINES-1];

    // Snoop index from the active bus address
    wire [SNOOP_IDX_BITS-1:0] snoop_idx = active_addr[SNOOP_IDX_BITS+1:2];
    wire [31:SNOOP_IDX_BITS+2] snoop_tag = active_addr[31:SNOOP_IDX_BITS+2];

    // ---------------------------------------------------------------
    // Shared region detection (Exercise 4)
    // Only enforce coherency for addresses 0x10000000 - 0x1FFFFFFF
    // ---------------------------------------------------------------
    wire addr_is_shared = (active_addr[31:28] == 4'h1);

    // ---------------------------------------------------------------
    // Snoop lookup for the "other" core
    // When core0 is active, we snoop core1's tags and vice versa.
    // ---------------------------------------------------------------
    wire other_core = ~active_core;

    // Snoop hit detection (combinational)
    reg        snoop_hit_r;
    reg [1:0]  snoop_state_r;

    always @(*) begin
        snoop_hit_r   = 1'b0;
        snoop_state_r = MSI_INVALID;

        // ---------------------------------------------------------
        // TODO (Exercise 2): Check the other core's tag array at
        //   snoop_idx.  A hit occurs if the tag matches and the
        //   entry is valid and the address is in the shared region.
        //
        //   if (other_core == 1) begin
        //       // Active core is 0, snoop core 1
        //       if (core1_tvalid[snoop_idx] &&
        //           core1_tags[snoop_idx] == snoop_tag &&
        //           addr_is_shared) begin
        //           snoop_hit_r   = 1'b1;
        //           snoop_state_r = core1_msi[snoop_idx];
        //       end
        //   end else begin
        //       // Active core is 1, snoop core 0
        //       ...
        //   end
        // ---------------------------------------------------------
    end

    assign snoop_hit   = snoop_hit_r;
    assign snoop_state = snoop_state_r;

    // ---------------------------------------------------------------
    // MSI State Transition Logic
    // ---------------------------------------------------------------
    // On every completed bus transaction, update both the requester's
    // and the snooper's MSI state.
    //
    // MSI transitions (from perspective of the snooped cache):
    //
    //   Current State | BusRd      | BusRdX
    //   ------------- | ---------- | ------
    //   INVALID       | (no action)| (no action)
    //   SHARED        | stay SHARED| → INVALID
    //   MODIFIED      | → SHARED   | → INVALID  (+ flush data)
    //                   (supply data to bus)
    //
    // Requester transitions:
    //   BusRd  → SHARED
    //   BusRdX → MODIFIED
    //   BusWB  → INVALID (eviction)

    integer m;

    always @(posedge clk) begin
        if (rst) begin
            for (m = 0; m < SNOOP_LINES; m = m + 1) begin
                core0_tvalid[m] <= 1'b0;
                core0_msi[m]    <= MSI_INVALID;
                core1_tvalid[m] <= 1'b0;
                core1_msi[m]    <= MSI_INVALID;
            end
        end else if (bus_busy && mem_ready && addr_is_shared) begin
            // ---------------------------------------------------------
            // TODO (Exercise 3): Implement MSI state transitions.
            //
            //   Step A -- Update the REQUESTER's state:
            //     - BUS_RD  → entry becomes SHARED, tag is set, valid=1
            //     - BUS_RDX → entry becomes MODIFIED
            //     - BUS_WB  → entry becomes INVALID, valid=0
            //
            //   Step B -- Update the SNOOPED (other) core's state:
            //     Apply the snoop transitions from the table above.
            //     If the snooped line was MODIFIED and we see a BusRd,
            //     the data must be flushed (in a real system the snoop
            //     controller would supply data; here just update state).
            //
            //   Use active_core to decide which tag/msi arrays to
            //   write as requester vs snooped.
            //
            //   Example for requester (core 0 active, BUS_RD):
            //     core0_tags[snoop_idx]   <= snoop_tag;
            //     core0_tvalid[snoop_idx] <= 1'b1;
            //     core0_msi[snoop_idx]    <= MSI_SHARED;
            // ---------------------------------------------------------
        end
    end

    // ================================================================
    // Section 4 (Challenge) -- MESI Extension
    // ================================================================
    // For Exercise 5, extend the state encoding to include EXCLUSIVE:
    //
    //   EXCLUSIVE: Clean, sole owner.  BusRd on an EXCLUSIVE line by
    //     another core transitions it to SHARED (no writeback needed).
    //     A local write silently upgrades EXCLUSIVE → MODIFIED without
    //     a bus transaction, saving bandwidth.
    //
    // TODO (Exercise 5): Modify the state machine above to support
    //   4 states (MSI_INVALID, MSI_SHARED, MSI_EXCLUSIVE, MSI_MODIFIED).
    //   Key additional transitions:
    //     - BusRd when no other core has copy → EXCLUSIVE (not SHARED)
    //     - EXCLUSIVE + local write → MODIFIED (no bus transaction)
    //     - EXCLUSIVE + BusRd from other core → SHARED

    // ================================================================
    // Debug / Verification Helpers
    // ================================================================
`ifdef FORMAL
    // Mutual exclusion: both cores should never be granted simultaneously
    always @(*) begin
        assert(!(core0_grant && core1_grant));
    end

    // Only one core active on the bus at a time
    always @(posedge clk) begin
        if (bus_busy)
            assert(active_core == 1'b0 || active_core == 1'b1);
    end

    // MSI invariant: a line cannot be MODIFIED in two caches at once
    // (for lines with matching tags at the same index)
    genvar gi;
    generate
        for (gi = 0; gi < SNOOP_LINES; gi = gi + 1) begin : msi_check
            always @(*) begin
                if (core0_tvalid[gi] && core1_tvalid[gi] &&
                    core0_tags[gi] == core1_tags[gi]) begin
                    // Both valid with same tag: at most one can be MODIFIED
                    assert(!(core0_msi[gi] == MSI_MODIFIED &&
                             core1_msi[gi] == MSI_MODIFIED));
                    // MODIFIED in one means INVALID in the other
                    if (core0_msi[gi] == MSI_MODIFIED)
                        assert(core1_msi[gi] == MSI_INVALID);
                    if (core1_msi[gi] == MSI_MODIFIED)
                        assert(core0_msi[gi] == MSI_INVALID);
                end
            end
        end
    endgenerate
`endif

endmodule
