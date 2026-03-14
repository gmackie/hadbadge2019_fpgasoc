/*
 * ooo_defs.vh - Out-of-Order CPU definitions
 *
 * Shared parameters and types for the OoO CPU labs.
 */

`ifndef OOO_DEFS_VH
`define OOO_DEFS_VH

`include "../../year3-cpu-design/common/riscv_defs.vh"

// Physical register file size (architectural + rename)
`define NUM_ARCH_REGS   32
`define NUM_PHYS_REGS   64
`define PHYS_REG_BITS   6

// Reorder buffer
`define ROB_SIZE        16
`define ROB_IDX_BITS    4

// Reservation station
`define RS_SIZE         8
`define RS_IDX_BITS     3

// Issue queue
`define IQ_SIZE         8

// Load/Store queue
`define LSQ_SIZE        8

`endif
