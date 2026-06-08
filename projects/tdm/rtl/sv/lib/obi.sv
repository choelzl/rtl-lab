// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   SystemVerilog interface for one base-OBI channel (single word, N=1).
//   Covers the full OBI signal set: req/addr/we/be/wdata (manager → subordinate)
//   and gnt/rvalid/rdata (subordinate → manager).  For the x-OBI multi-word
//   extension (N > 1) use xobi_if.  Parameters ADDR_W / DATA_W / BE_W set the
//   physical signal widths; BE_W defaults to DATA_W / 8.
//
//   Modports:
//     mgr — manager drives req/addr/we/be/wdata, receives gnt/rvalid/rdata
//     sub — subordinate receives req/addr/we/be/wdata, drives gnt/rvalid/rdata
//
//   Port-name conventions (module flat-port equivalent):
//     subordinate side   req_i, addr_i, we_i, be_i, wdata_i  (in)
//                        gnt_o, rvalid_o, rdata_o             (out)
//     manager side       req_o, addr_o, we_o, be_o, wdata_o  (out)
//                        gnt_i, rvalid_i, rdata_i             (in)
// -----------------------------------------------------------------------------

`ifndef LIB_OBI_SV
`define LIB_OBI_SV

interface obi_if #(
    parameter int ADDR_W = 32,
    parameter int DATA_W = 32,
    parameter int BE_W   = DATA_W / 8
);
    logic              req;
    logic [ADDR_W-1:0] addr;
    logic              we;
    logic [BE_W-1:0]   be;
    logic [DATA_W-1:0] wdata;
    logic              gnt;
    logic              rvalid;
    logic [DATA_W-1:0] rdata;

    modport mgr (
        output req, addr, we, be, wdata,
        input  gnt, rvalid, rdata
    );

    modport sub (
        input  req, addr, we, be, wdata,
        output gnt, rvalid, rdata
    );
endinterface

`endif // LIB_OBI_SV
