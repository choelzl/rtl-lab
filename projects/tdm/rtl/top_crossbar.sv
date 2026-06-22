// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Three-level interleaved crossbar connecting NUM_RAGU read AGUs and NUM_WAGU
//   write AGUs (NUM_REQ request ports each) to NUM_BANK*2 memory banks.
//
//   Level 1: per-AGU NUM_REQ x NUM_REQ crossbar — spreads an AGU's NUM_REQ ports
//            across the NUM_REQ level-2 groups.
//   Level 2: per-group NUM_RAGU x NUM_BANK_GRP crossbar — routes to the banks of
//            a group.
//   Level 3: per-bank 2 x 2 crossbar — merges the read and write paths onto the
//            even/odd physical bank pair.
//
//   A transaction is one bank row (BYTES_PER_ROW bytes) wide. Word-interleaved
//   address layout (defaults: BYTES_PER_ROW=16, NUM_REQ=4, NUM_BANK=32):
//     addr[3:0]   byte offset within a row
//     addr[5:4]   level-1 select
//     addr[8:6]   level-2 select
//     addr[9]     level-3 even/odd select
//     addr[19:10] bank-local row
//   The bank-select bits addr[9:4] are stripped before each bank, so a bank sees
//   a compact bank-local address (row directly above the byte offset).
// -----------------------------------------------------------------------------

`include "obi_pkg.sv"

module top_crossbar
    import obi_pkg::*;
#(
    parameter  int NUM_RAGU       = 7,
    parameter  int NUM_WAGU       = 6,
    parameter  int NUM_REQ        = 4,
    parameter  int NUM_BANK       = 32,
    parameter  int NUM_ROW        = 1024,
    parameter  int WORDS_PER_ROW  = 4,
    parameter  int BYTES_PER_WORD = 4,
    localparam int NUM_RD         = NUM_RAGU * NUM_REQ,
    localparam int NUM_WR         = NUM_WAGU * NUM_REQ,
    localparam int NUM_BANK_GRP   = NUM_BANK / NUM_REQ,
    localparam int BYTES_PER_ROW  = WORDS_PER_ROW * BYTES_PER_WORD,
    localparam int ROUTE_LSB      = $clog2(BYTES_PER_ROW),
    localparam int ROUTE_BITS     = $clog2(NUM_REQ) + $clog2(NUM_BANK_GRP) + 1
) (
    input  logic                   clk_i,
    input  logic                   rst_ni,
    input  obi_req_t  [NUM_RD-1:0] ragu_req_i,
    output obi_resp_t [NUM_RD-1:0] ragu_resp_o,
    input  obi_req_t  [NUM_WR-1:0] wagu_req_i,
    output obi_resp_t [NUM_WR-1:0] wagu_resp_o
);
    function automatic logic [31:0] addr_hash(input logic [31:0] addr);
        return {addr[31:9], 3'(addr[11:9] + addr[8:6]), addr[5:0]};
    endfunction

    // --------------------------------------------------------------------------
    // Pack flat AGU ports → obi_pkg structs (scrambled addr_hash)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_RD-1:0] ragu_obi_req;
    obi_resp_t [NUM_RD-1:0] ragu_obi_resp;

    for (genvar a = 0; a < NUM_RD; a++) begin : gen_ragu_pack
        assign ragu_obi_req[a] = '{
            req:   ragu_req_i[a].req,
            we:    ragu_req_i[a].we,
            be:    ragu_req_i[a].be,
            addr:  addr_hash(ragu_req_i[a].addr),
            wdata: ragu_req_i[a].wdata
        };
        assign ragu_resp_o[a].gnt    = ragu_obi_resp[a].gnt;
        assign ragu_resp_o[a].rvalid = ragu_obi_resp[a].rvalid;
        assign ragu_resp_o[a].rdata  = ragu_obi_resp[a].rdata;
    end

    // --------------------------------------------------------------------------
    // Pack flat WAGU ports → obi_pkg structs (scrambled addr_hash)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_WR-1:0] wagu_obi_req;
    obi_resp_t [NUM_WR-1:0] wagu_obi_resp;

    for (genvar a = 0; a < NUM_WR; a++) begin : gen_wagu_pack
        assign wagu_obi_req[a] = '{
            req:   wagu_req_i[a].req,
            we:    wagu_req_i[a].we,
            be:    wagu_req_i[a].be,
            addr:  addr_hash(wagu_req_i[a].addr),
            wdata: wagu_req_i[a].wdata
        };
        assign wagu_resp_o[a].gnt    = wagu_obi_resp[a].gnt;
        assign wagu_resp_o[a].rvalid = wagu_obi_resp[a].rvalid;
        assign wagu_resp_o[a].rdata  = wagu_obi_resp[a].rdata;
    end

    obi_req_t  [NUM_RAGU-1:0][NUM_REQ-1:0] rd_l1_l2_req;
    obi_resp_t [NUM_RAGU-1:0][NUM_REQ-1:0] rd_l1_l2_resp;

    for (genvar j = 0; j < NUM_RAGU; j++) begin : gen_read_level_1
        crossbar #(
            .XBAR_NMASTER    (NUM_REQ),
            .XBAR_NSLAVE     (NUM_REQ),
            .SEL_SLICE_START (ROUTE_LSB),
            .SEL_SLICE_LENGTH($clog2(NUM_REQ))
        ) crossbar_level_1_i (
            .clk_i,
            .rst_ni,
            .master_req_i (ragu_obi_req [j*NUM_REQ +: NUM_REQ]),
            .master_resp_o(ragu_obi_resp[j*NUM_REQ +: NUM_REQ]),
            .slave_req_o  (rd_l1_l2_req [j]),
            .slave_resp_i (rd_l1_l2_resp[j])
        );
    end

    obi_req_t  [NUM_WAGU-1:0][NUM_REQ-1:0] wr_l1_l2_req;
    obi_resp_t [NUM_WAGU-1:0][NUM_REQ-1:0] wr_l1_l2_resp;

    for (genvar j = 0; j < NUM_WAGU; j++) begin : gen_write_level_1
        crossbar #(
            .XBAR_NMASTER    (NUM_REQ),
            .XBAR_NSLAVE     (NUM_REQ),
            .SEL_SLICE_START (ROUTE_LSB),
            .SEL_SLICE_LENGTH($clog2(NUM_REQ))
        ) crossbar_level_1_i (
            .clk_i,
            .rst_ni,
            .master_req_i (wagu_obi_req [j*NUM_REQ +: NUM_REQ]),
            .master_resp_o(wagu_obi_resp[j*NUM_REQ +: NUM_REQ]),
            .slave_req_o  (wr_l1_l2_req [j]),
            .slave_resp_i (wr_l1_l2_resp[j])
        );
    end

    obi_req_t  [NUM_BANK-1:0] rd_l2_l3_req;
    obi_resp_t [NUM_BANK-1:0] rd_l2_l3_resp;

    for (genvar k = 0; k < NUM_REQ; k++) begin : gen_read_level_2
        obi_req_t  [NUM_RAGU-1:0] l2_master_req;
        obi_resp_t [NUM_RAGU-1:0] l2_master_resp;
        for (genvar j = 0; j < NUM_RAGU; j++) begin : gen_link
            assign l2_master_req[j]    = rd_l1_l2_req[j][k];
            assign rd_l1_l2_resp[j][k] = l2_master_resp[j];
        end
        crossbar #(
            .XBAR_NMASTER    (NUM_RAGU),
            .XBAR_NSLAVE     (NUM_BANK_GRP),
            .SEL_SLICE_START (ROUTE_LSB + $clog2(NUM_REQ)),
            .SEL_SLICE_LENGTH($clog2(NUM_BANK_GRP))
        ) crossbar_level_2_i (
            .clk_i,
            .rst_ni,
            .master_req_i (l2_master_req),
            .master_resp_o(l2_master_resp),
            .slave_req_o  (rd_l2_l3_req [k*NUM_BANK_GRP +: NUM_BANK_GRP]),
            .slave_resp_i (rd_l2_l3_resp[k*NUM_BANK_GRP +: NUM_BANK_GRP])
        );
    end

    obi_req_t  [NUM_BANK-1:0] wr_l2_l3_req;
    obi_resp_t [NUM_BANK-1:0] wr_l2_l3_resp;

    for (genvar k = 0; k < NUM_REQ; k++) begin : gen_write_level_2
        obi_req_t  [NUM_WAGU-1:0] l2_master_req;
        obi_resp_t [NUM_WAGU-1:0] l2_master_resp;
        for (genvar j = 0; j < NUM_WAGU; j++) begin : gen_link
            assign l2_master_req[j]    = wr_l1_l2_req[j][k];
            assign wr_l1_l2_resp[j][k] = l2_master_resp[j];
        end
        crossbar #(
            .XBAR_NMASTER    (NUM_WAGU),
            .XBAR_NSLAVE     (NUM_BANK_GRP),
            .SEL_SLICE_START (ROUTE_LSB + $clog2(NUM_REQ)),
            .SEL_SLICE_LENGTH($clog2(NUM_BANK_GRP))
        ) crossbar_level_2_i (
            .clk_i,
            .rst_ni,
            .master_req_i (l2_master_req),
            .master_resp_o(l2_master_resp),
            .slave_req_o  (wr_l2_l3_req [k*NUM_BANK_GRP +: NUM_BANK_GRP]),
            .slave_resp_i (wr_l2_l3_resp[k*NUM_BANK_GRP +: NUM_BANK_GRP])
        );
    end

    obi_req_t  [NUM_BANK*2-1:0] bank_req;
    obi_resp_t [NUM_BANK*2-1:0] bank_resp;

    for (genvar k = 0; k < NUM_BANK; k++) begin : gen_level_3
        obi_req_t  [1:0] l3_master_req;
        obi_resp_t [1:0] l3_master_resp;
        assign l3_master_req[0] = rd_l2_l3_req[k];
        assign rd_l2_l3_resp[k] = l3_master_resp[0];
        assign l3_master_req[1] = wr_l2_l3_req[k];
        assign wr_l2_l3_resp[k] = l3_master_resp[1];
        crossbar #(
            .XBAR_NMASTER    (2),
            .XBAR_NSLAVE     (2),
            .SEL_SLICE_START (ROUTE_LSB + $clog2(NUM_REQ) + $clog2(NUM_BANK_GRP)),
            .SEL_SLICE_LENGTH(1)
        ) crossbar_level_3_i (
            .clk_i,
            .rst_ni,
            .master_req_i (l3_master_req),
            .master_resp_o(l3_master_resp),
            .slave_req_o  (bank_req [k*2 +: 2]),
            .slave_resp_i (bank_resp[k*2 +: 2])
        );
    end

    for (genvar i = 0; i < NUM_BANK*2; i++) begin : gen_banks
        obi_req_t bank_local_req;
        assign bank_local_req = '{
            req:   bank_req[i].req,
            we:    bank_req[i].we,
            be:    bank_req[i].be,
            addr:  {{ROUTE_BITS{1'b0}},
                    bank_req[i].addr[31 : ROUTE_LSB+ROUTE_BITS],
                    bank_req[i].addr[ROUTE_LSB-1 : 0]},
            wdata: bank_req[i].wdata
        };
        bank #(
            .NUM_ROW       (NUM_ROW/2),
            .WORDS_PER_ROW (WORDS_PER_ROW),
            .BYTES_PER_WORD(BYTES_PER_WORD)
        ) bank_i (
            .clk_i,
            .rst_ni,
            .obi_req_i (bank_local_req),
            .obi_resp_o(bank_resp[i])
        );
    end

endmodule
