// Copyright 2022 OpenHW Group
// Solderpad Hardware License, Version 2.1, see LICENSE.md for details.
// SPDX-License-Identifier: Apache-2.0 WITH SHL-2.1



module system_xbar
  import obi_pkg::*;
  import addr_map_rule_pkg::*;
  import core_v_mini_mcu_pkg::*;
#(
    parameter XBAR_NMASTER = 3,
    parameter XBAR_NSLAVE = 6,
    localparam int unsigned IdxWidth = cf_math_pkg::idx_width(XBAR_NSLAVE)
) (
    input logic clk_i,
    input logic rst_ni,

    // Address map
    input addr_map_rule_pkg::addr_map_rule_t [XBAR_NSLAVE-1:0] addr_map_i,

    // Default slave index
    input logic [IdxWidth-1:0] default_idx_i,

    input  obi_req_t  [XBAR_NMASTER-1:0] master_req_i,
    output obi_resp_t [XBAR_NMASTER-1:0] master_resp_o,

    output obi_req_t  [XBAR_NSLAVE-1:0] slave_req_o,
    input  obi_resp_t [XBAR_NSLAVE-1:0] slave_resp_i

);

  localparam int unsigned LOG_XBAR_NMASTER = XBAR_NMASTER > 1 ? $clog2(XBAR_NMASTER) : 32'd1;
  localparam int unsigned LOG_XBAR_NSLAVE = XBAR_NSLAVE > 1 ? $clog2(XBAR_NSLAVE) : 32'd1;

  //Aggregated Request Data (from Master -> slaves)
  //WE + BE + ADDR + WDATA
  localparam int unsigned REQ_AGG_DATA_WIDTH = 1 + 4 + 32 + 32;
  localparam int unsigned RESP_AGG_DATA_WIDTH = 32;

  //Address Decoder
  logic [XBAR_NMASTER-1:0][LOG_XBAR_NSLAVE-1:0] port_sel;

  // Neck crossbar
  obi_req_t neck_req;
  obi_resp_t neck_resp;

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
  obi_req_t [XBAR_NMASTER-1:0] master_req;

  begin : gen_addr_decoders_NtoM
    for (genvar i = 0; i < XBAR_NMASTER; i++) begin : gen_addr_decoders
      addr_decode #(
          /// Highest index which can happen in a rule.
          .NoIndices(XBAR_NSLAVE),
          .NoRules(XBAR_NSLAVE),
          .addr_t(logic [31:0]),
          .rule_t(addr_map_rule_pkg::addr_map_rule_t)
      ) addr_decode_i (
          .addr_i(master_req_i[i].addr),
          .addr_map_i,
          .idx_o(port_sel[i]),
          .dec_valid_o(),
          .dec_error_o(),
          .en_default_idx_i(1'b1),
          .default_idx_i
      );
    end
  end

  // Propagate interleaved address
  generate
    for (genvar i = 0; i < XBAR_NMASTER; i++) begin : gen_unroll_master
      assign master_req[i] = '{
        req: master_req_i[i].req,
        we: master_req_i[i].we,
        be: master_req_i[i].be,
        addr: master_req_i[i].addr,
        wdata: master_req_i[i].wdata
      };
    end
  endgenerate

  begin : gen_xbar_NtoM
    // Unroll OBI structs
    for (genvar i = 0; unsigned'(i) < XBAR_NMASTER; i++) begin: gen_unroll_master
      assign master_req_req[i] = master_req[i].req;
      assign master_req_data[i] = {
        master_req[i].we,
        master_req[i].be,
        master_req[i].addr,
        master_req[i].wdata
      };
      assign master_resp_o[i].gnt = master_resp_gnt[i];
      assign master_resp_o[i].rdata = master_resp_rdata[i];
      assign master_resp_o[i].rvalid = master_resp_rvalid[i];
    end

    for (genvar i = 0; i < XBAR_NSLAVE; i++) begin : gen_unroll_slave
      assign slave_req_o[i].req = slave_req_req[i];
      assign {slave_req_o[i].we, slave_req_o[i].be, slave_req_o[i].addr, slave_req_o[i].wdata} = slave_req_out_data[i];
      assign slave_resp_rdata[i] = slave_resp_i[i].rdata;
      assign slave_resp_gnt[i] = slave_resp_i[i].gnt;
      assign slave_resp_rvalid[i] = slave_resp_i[i].rvalid;
    end

    //Crossbar instantiation
    xbar_varlat #(
        .AggregateGnt(0),
        .NumIn(XBAR_NMASTER),
        .NumOut(XBAR_NSLAVE),
        .ReqDataWidth(REQ_AGG_DATA_WIDTH),
        .RespDataWidth(RESP_AGG_DATA_WIDTH)
    ) i_xbar (
        .clk_i,
        .rst_ni,
        .req_i  (master_req_req),
        .add_i  (port_sel),
        .wdata_i(master_req_data),
        .gnt_o  (master_resp_gnt),
        .rdata_o(master_resp_rdata),
        .rr_i   ('0),
        .vld_o  (master_resp_rvalid),
        .gnt_i  (slave_resp_gnt),
        .req_o  (slave_req_req),
        .vld_i  (slave_resp_rvalid),
        .wdata_o(slave_req_out_data),
        .rdata_i(slave_resp_rdata)
    );
  end

endmodule : system_xbar
