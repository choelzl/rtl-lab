// Copyright 2022 OpenHW Group
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
/*
 * Description: OBI package — shared request/response struct types.
 *
 * Data-path widths are controlled by two compile-time macros:
 *   OBI_DATA_W  — data bus width in bits   (default 32)
 *   OBI_BE_W    — byte-enable width in bits (default 4, one bit per byte)
 * Override at elaboration time: -DOBI_DATA_W=<n> -DOBI_BE_W=<n/8>.
 * Both macros must be consistent: OBI_BE_W == OBI_DATA_W / 8.
 */

`ifndef INTERACE_OBI
`define INTERACE_OBI


`ifndef OBI_DATA_W
`define OBI_DATA_W 32
`endif
`ifndef OBI_BE_W
`define OBI_BE_W 4
`endif

package obi_pkg;

  typedef struct packed {
    logic                    req;
    logic                    we;
    logic [`OBI_BE_W-1:0]   be;
    logic [31:0]             addr;
    logic [`OBI_DATA_W-1:0] wdata;
  } obi_req_t;

  typedef struct packed {
    logic                    gnt;
    logic                    rvalid;
    logic [`OBI_DATA_W-1:0] rdata;
  } obi_resp_t;

endpackage

`endif
