// Copyright 2022 OpenHW Group
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1
//
// Description:
//   OBI-typed wrapper around xbar_varlat (logarithmic interconnect with
//   per-output round-robin arbitration).
//
//   Each of the XBAR_NMASTER masters presents an obi_req_t; the slave
//   selection index is extracted from addr[SEL_SLICE_START +: SEL_SLICE_LENGTH]
//   and forwarded to xbar_varlat as the bank-address field (add_i).  Conflicts
//   on the same slave are resolved by rr_arb_tree inside xbar_varlat.
//
//   OBI protocol constraints respected here:
//     - gnt is combinational (from rr_arb_tree → addr_dec_resp_mux_varlat).
//     - rvalid is registered inside addr_dec_resp_mux_varlat (1-cycle latency).
//     - No outstanding transactions: a port stalls until its in-flight
//       request completes (valid_inflight_q gate in addr_dec_resp_mux_varlat).
//
//   Parameters:
//     XBAR_NMASTER    — number of OBI master ports
//     XBAR_NSLAVE     — number of OBI slave ports
//     SEL_SLICE_START — LSB of the address slice used for slave selection
//     SEL_SLICE_LENGTH — width of that slice; must satisfy 2^SEL_SLICE_LENGTH == XBAR_NSLAVE

module xbar
  import obi_pkg::*;
#(
    parameter XBAR_NMASTER    = 3,
    parameter XBAR_NSLAVE     = 6,
    parameter SEL_SLICE_START  = 0,
    parameter SEL_SLICE_LENGTH = 2,
    localparam int unsigned IdxWidth = $clog2(XBAR_NSLAVE)
) (
    input logic clk_i,
    input logic rst_ni,

    // Unused by xbar_varlat when ExtPrio=0; kept for interface symmetry.
    input logic [IdxWidth-1:0] default_idx_i,

    input  obi_req_t  [XBAR_NMASTER-1:0] master_req_i,
    output obi_resp_t [XBAR_NMASTER-1:0] master_resp_o,

    output obi_req_t  [XBAR_NSLAVE-1:0] slave_req_o,
    input  obi_resp_t [XBAR_NSLAVE-1:0] slave_resp_i
);

  // --------------------------------------------------------------------------
  // Parameter sanity checks (elaboration time)
  // --------------------------------------------------------------------------
  initial begin : chk_params
    if (XBAR_NMASTER < 1)
      $fatal(1, "xbar: XBAR_NMASTER must be >= 1, got %0d", XBAR_NMASTER);
    if (XBAR_NSLAVE < 1)
      $fatal(1, "xbar: XBAR_NSLAVE must be >= 1, got %0d", XBAR_NSLAVE);
    if (XBAR_NSLAVE & (XBAR_NSLAVE - 1))
      $fatal(1, "xbar: XBAR_NSLAVE (%0d) must be a power of 2 for address decode", XBAR_NSLAVE);
    if (SEL_SLICE_START + SEL_SLICE_LENGTH > 32)
      $fatal(1, "xbar: address slice [%0d+:%0d] overflows 32-bit address",
             SEL_SLICE_START, SEL_SLICE_LENGTH);
    if ((1 << SEL_SLICE_LENGTH) != XBAR_NSLAVE)
      $fatal(1, "xbar: 2^SEL_SLICE_LENGTH=%0d but XBAR_NSLAVE=%0d; slice cannot reach all slaves",
             1 << SEL_SLICE_LENGTH, XBAR_NSLAVE);
  end

  localparam int unsigned LOG_XBAR_NMASTER = XBAR_NMASTER > 1 ? $clog2(XBAR_NMASTER) : 32'd1;
  localparam int unsigned LOG_XBAR_NSLAVE  = XBAR_NSLAVE  > 1 ? $clog2(XBAR_NSLAVE)  : 32'd1;

  // Aggregated request payload passed through xbar_varlat: {we, be, addr, wdata}.
  // Sized for 32-bit OBI (OBI_BE_W=4, OBI_DATA_W=32); update macros if widths change.
  localparam int unsigned REQ_AGG_DATA_WIDTH  = 1 + `OBI_BE_W + 32 + `OBI_DATA_W;
  localparam int unsigned RESP_AGG_DATA_WIDTH = `OBI_DATA_W;

  // Decoded slave-select index, one per master port.
  logic [XBAR_NMASTER-1:0][LOG_XBAR_NSLAVE-1:0] port_sel;

  logic [XBAR_NMASTER-1:0] master_req_req;
  logic [XBAR_NMASTER-1:0] master_resp_gnt;
  logic [XBAR_NMASTER-1:0] master_resp_rvalid;
  logic [XBAR_NMASTER-1:0][31:0] master_resp_rdata;

  logic [XBAR_NSLAVE-1:0] slave_req_req;
  logic [XBAR_NSLAVE-1:0] slave_resp_gnt;
  logic [XBAR_NSLAVE-1:0] slave_resp_rvalid;
  logic [XBAR_NSLAVE-1:0][31:0] slave_resp_rdata;


  logic [XBAR_NMASTER-1:0][REQ_AGG_DATA_WIDTH-1:0] master_req_data;
  logic [XBAR_NSLAVE-1:0][REQ_AGG_DATA_WIDTH-1:0] slave_req_out_data;

  // Extract the slave-select index from each master's address.
  for (genvar i = 0; unsigned'(i) < XBAR_NMASTER; i++) begin : gen_sel_signal
    assign port_sel[i] = master_req_i[i].addr[SEL_SLICE_START +: SEL_SLICE_LENGTH];
  end

  // Flatten OBI structs → scalar/vector signals consumed by xbar_varlat.
  for (genvar i = 0; unsigned'(i) < XBAR_NMASTER; i++) begin : gen_unroll_master
    assign master_req_req[i]  = master_req_i[i].req;
    assign master_req_data[i] = {
      master_req_i[i].we,
      master_req_i[i].be,
      master_req_i[i].addr,
      master_req_i[i].wdata
    };
    assign master_resp_o[i].gnt    = master_resp_gnt[i];
    assign master_resp_o[i].rdata  = master_resp_rdata[i];
    assign master_resp_o[i].rvalid = master_resp_rvalid[i];
  end

  for (genvar i = 0; i < XBAR_NSLAVE; i++) begin : gen_unroll_slave
    assign slave_req_o[i].req = slave_req_req[i];
    assign {slave_req_o[i].we, slave_req_o[i].be,
            slave_req_o[i].addr, slave_req_o[i].wdata} = slave_req_out_data[i];
    assign slave_resp_rdata[i]  = slave_resp_i[i].rdata;
    assign slave_resp_gnt[i]    = slave_resp_i[i].gnt;
    assign slave_resp_rvalid[i] = slave_resp_i[i].rvalid;
  end

  xbar_varlat #(
      // AggregateGnt=0: each master's gnt comes from its individually addressed slave,
      // not from OR-ing all slave grants. Required when this xbar is used as a sub-xbar
      // (its masters may already be shared outputs of an upstream arbiter).
      .AggregateGnt (0),
      .NumIn        (XBAR_NMASTER),
      .NumOut       (XBAR_NSLAVE),
      .ReqDataWidth (REQ_AGG_DATA_WIDTH),
      .RespDataWidth(RESP_AGG_DATA_WIDTH)
  ) i_xbar (
      .clk_i,
      .rst_ni,
      .req_i  (master_req_req),
      .add_i  (port_sel),
      .wdata_i(master_req_data),
      .gnt_o  (master_resp_gnt),
      .rdata_o(master_resp_rdata),
      .rr_i   ('0),           // ExtPrio=0 (default): RR state managed internally
      .vld_o  (master_resp_rvalid),
      .gnt_i  (slave_resp_gnt),
      .req_o  (slave_req_req),
      .vld_i  (slave_resp_rvalid),
      .wdata_o(slave_req_out_data),
      .rdata_i(slave_resp_rdata)
  );

endmodule : xbar
