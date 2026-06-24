// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Three-level interleaved crossbar connecting NUM_RPORT read ports and NUM_WPORT
//   write ports to NUM_BANK*2 memory banks. Each port has NUM_REQ OBI buses.
//
//   Before L1/L2, adjacent port-group pairs are muxed down:
//     read : groups 2&4 → xbar port 2, groups 3&5 → xbar port 3  (9→7)
//     write: groups 2&4 → xbar port 2, groups 3&5 → xbar port 3  (8→6)
//   Selection is req-driven; sel is registered to steer rvalid one cycle later.
//   The two sources of each muxed pair are guaranteed mutually exclusive by stimulus.
//
//   Level 1: per-port NUM_REQ x NUM_REQ crossbar — spreads a port's NUM_REQ OBI
//            buses across the NUM_REQ level-2 groups.
//   Level 2: per-group NUM_RPORT_XBAR x NUM_BANK_GRP crossbar — routes to the banks
//            of a group.
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
    parameter  int NUM_RPORT      = 9,
    parameter  int NUM_WPORT      = 8,
    parameter  int NUM_REQ        = 4,
    parameter  int NUM_BANK       = 32,
    parameter  int NUM_ROW        = 1024,
    parameter  int WORDS_PER_ROW  = 4,
    parameter  int BYTES_PER_WORD = 4,
    localparam int NUM_RD         = NUM_RPORT * NUM_REQ,
    localparam int NUM_WR         = NUM_WPORT * NUM_REQ,
    localparam int NUM_RPORT_XBAR = NUM_RPORT - 2,
    localparam int NUM_WPORT_XBAR = NUM_WPORT - 2,
    localparam int NUM_RD_XBAR    = NUM_RPORT_XBAR * NUM_REQ,
    localparam int NUM_WR_XBAR    = NUM_WPORT_XBAR * NUM_REQ,
    localparam int NUM_BANK_GRP   = NUM_BANK / NUM_REQ,
    localparam int BYTES_PER_ROW  = WORDS_PER_ROW * BYTES_PER_WORD,
    localparam int ROUTE_LSB      = $clog2(BYTES_PER_ROW),
    localparam int ROUTE_BITS     = $clog2(NUM_REQ) + $clog2(NUM_BANK_GRP) + 1
) (
    input  logic                   clk_i,
    input  logic                   rst_ni,
    input  obi_req_t  [NUM_RD-1:0] rport_req_i,
    output obi_resp_t [NUM_RD-1:0] rport_resp_o,
    input  obi_req_t  [NUM_WR-1:0] wport_req_i,
    output obi_resp_t [NUM_WR-1:0] wport_resp_o
);
    function automatic logic [31:0] addr_hash(input logic [31:0] addr);
        return {addr[31:9], 3'(addr[11:9] + addr[8:6]), addr[5:0]};
    endfunction

    // --------------------------------------------------------------------------
    // Pack flat read-port OBI buses → obi_pkg structs (scrambled addr_hash)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_RD-1:0] rport_obi_req;
    obi_resp_t [NUM_RD-1:0] rport_obi_resp;

    for (genvar a = 0; a < NUM_RD; a++) begin : gen_rport_pack
        assign rport_obi_req[a] = '{
                req: rport_req_i[a].req,
                we: rport_req_i[a].we,
                be: rport_req_i[a].be,
                addr: addr_hash(rport_req_i[a].addr),
                wdata: rport_req_i[a].wdata
            };
        assign rport_resp_o[a].gnt = rport_obi_resp[a].gnt;
        assign rport_resp_o[a].rvalid = rport_obi_resp[a].rvalid;
        assign rport_resp_o[a].rdata = rport_obi_resp[a].rdata;
    end

    // --------------------------------------------------------------------------
    // Pack flat write-port OBI buses → obi_pkg structs (scrambled addr_hash)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_WR-1:0] wport_obi_req;
    obi_resp_t [NUM_WR-1:0] wport_obi_resp;

    for (genvar a = 0; a < NUM_WR; a++) begin : gen_wport_pack
        assign wport_obi_req[a] = '{
                req: wport_req_i[a].req,
                we: wport_req_i[a].we,
                be: wport_req_i[a].be,
                addr: addr_hash(wport_req_i[a].addr),
                wdata: wport_req_i[a].wdata
            };
        assign wport_resp_o[a].gnt = wport_obi_resp[a].gnt;
        assign wport_resp_o[a].rvalid = wport_obi_resp[a].rvalid;
        assign wport_resp_o[a].rdata = wport_obi_resp[a].rdata;
    end

    // --------------------------------------------------------------------------
    // Read-port mux: groups 2&4 → xbar 2, groups 3&5 → xbar 3  (9 → 7 ports)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_RD_XBAR-1:0] rd_mux_req;
    obi_resp_t [NUM_RD_XBAR-1:0] rd_mux_resp;

    // sel=1 when the B-side source (group 4 or 5) has any req asserted
    logic [NUM_REQ-1:0] grp4r_reqs, grp5r_reqs;
    logic sel_r24, sel_r35;
    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_rd_sel_req
        assign grp4r_reqs[r] = rport_obi_req[4*NUM_REQ+r].req;
        assign grp5r_reqs[r] = rport_obi_req[5*NUM_REQ+r].req;
    end
    assign sel_r24 = |grp4r_reqs;
    assign sel_r35 = |grp5r_reqs;

    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_rd_mux_req
        assign rd_mux_req[0*NUM_REQ+r] = rport_obi_req[0*NUM_REQ+r];  // RAGU_A[0]
        assign rd_mux_req[1*NUM_REQ+r] = rport_obi_req[1*NUM_REQ+r];  // RAGU_A[1]
        assign rd_mux_req[2*NUM_REQ+r] = sel_r24  // RAGU_A[2] | RAGU_B[0]
            ? rport_obi_req[4*NUM_REQ+r] : rport_obi_req[2*NUM_REQ+r];
        assign rd_mux_req[3*NUM_REQ+r] = sel_r35  // RAGU_A[3] | RAGU_B[1]
            ? rport_obi_req[5*NUM_REQ+r] : rport_obi_req[3*NUM_REQ+r];
        assign rd_mux_req[4*NUM_REQ+r] = rport_obi_req[6*NUM_REQ+r];  // RAGU_C
        assign rd_mux_req[5*NUM_REQ+r] = rport_obi_req[7*NUM_REQ+r];  // RAGU_D
        assign rd_mux_req[6*NUM_REQ+r] = rport_obi_req[8*NUM_REQ+r];  // RAGU_DMA
    end

    // Register sel at gnt time to steer rvalid (one cycle later)
    logic [NUM_REQ-1:0] mux2r_gnts, mux3r_gnts;
    logic sel_r24_q, sel_r35_q;
    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_rd_gnt_bits
        assign mux2r_gnts[r] = rd_mux_resp[2*NUM_REQ+r].gnt;
        assign mux3r_gnts[r] = rd_mux_resp[3*NUM_REQ+r].gnt;
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            sel_r24_q <= 1'b0;
            sel_r35_q <= 1'b0;
        end else begin
            if (|mux2r_gnts) sel_r24_q <= sel_r24;
            if (|mux3r_gnts) sel_r35_q <= sel_r35;
        end
    end

    // Demux responses back to original group indices
    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_rd_demux
        assign rport_obi_resp[0*NUM_REQ+r]        = rd_mux_resp[0*NUM_REQ+r];
        assign rport_obi_resp[1*NUM_REQ+r]        = rd_mux_resp[1*NUM_REQ+r];
        // group 2 (RAGU_A[2]): active when sel_r24=0
        assign rport_obi_resp[2*NUM_REQ+r].gnt    = rd_mux_resp[2*NUM_REQ+r].gnt & ~sel_r24;
        assign rport_obi_resp[2*NUM_REQ+r].rvalid = rd_mux_resp[2*NUM_REQ+r].rvalid & ~sel_r24_q;
        assign rport_obi_resp[2*NUM_REQ+r].rdata  = rd_mux_resp[2*NUM_REQ+r].rdata;
        // group 3 (RAGU_A[3]): active when sel_r35=0
        assign rport_obi_resp[3*NUM_REQ+r].gnt    = rd_mux_resp[3*NUM_REQ+r].gnt & ~sel_r35;
        assign rport_obi_resp[3*NUM_REQ+r].rvalid = rd_mux_resp[3*NUM_REQ+r].rvalid & ~sel_r35_q;
        assign rport_obi_resp[3*NUM_REQ+r].rdata  = rd_mux_resp[3*NUM_REQ+r].rdata;
        // group 4 (RAGU_B[0]): active when sel_r24=1
        assign rport_obi_resp[4*NUM_REQ+r].gnt    = rd_mux_resp[2*NUM_REQ+r].gnt & sel_r24;
        assign rport_obi_resp[4*NUM_REQ+r].rvalid = rd_mux_resp[2*NUM_REQ+r].rvalid & sel_r24_q;
        assign rport_obi_resp[4*NUM_REQ+r].rdata  = rd_mux_resp[2*NUM_REQ+r].rdata;
        // group 5 (RAGU_B[1]): active when sel_r35=1
        assign rport_obi_resp[5*NUM_REQ+r].gnt    = rd_mux_resp[3*NUM_REQ+r].gnt & sel_r35;
        assign rport_obi_resp[5*NUM_REQ+r].rvalid = rd_mux_resp[3*NUM_REQ+r].rvalid & sel_r35_q;
        assign rport_obi_resp[5*NUM_REQ+r].rdata  = rd_mux_resp[3*NUM_REQ+r].rdata;
        // groups 6,7,8 ← xbar 4,5,6
        assign rport_obi_resp[6*NUM_REQ+r]        = rd_mux_resp[4*NUM_REQ+r];
        assign rport_obi_resp[7*NUM_REQ+r]        = rd_mux_resp[5*NUM_REQ+r];
        assign rport_obi_resp[8*NUM_REQ+r]        = rd_mux_resp[6*NUM_REQ+r];
    end

    // --------------------------------------------------------------------------
    // Write-port mux: groups 2&4 → xbar 2, groups 3&5 → xbar 3  (8 → 6 ports)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_WR_XBAR-1:0] wr_mux_req;
    obi_resp_t [NUM_WR_XBAR-1:0] wr_mux_resp;

    logic [NUM_REQ-1:0] grp4w_reqs, grp5w_reqs;
    logic sel_w24, sel_w35;
    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_wr_sel_req
        assign grp4w_reqs[r] = wport_obi_req[4*NUM_REQ+r].req;
        assign grp5w_reqs[r] = wport_obi_req[5*NUM_REQ+r].req;
    end
    assign sel_w24 = |grp4w_reqs;
    assign sel_w35 = |grp5w_reqs;

    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_wr_mux_req
        assign wr_mux_req[0*NUM_REQ+r] = wport_obi_req[0*NUM_REQ+r];  // WAGU_A[0]
        assign wr_mux_req[1*NUM_REQ+r] = wport_obi_req[1*NUM_REQ+r];  // WAGU_A[1]
        assign wr_mux_req[2*NUM_REQ+r] = sel_w24  // WAGU_A[2] | WAGU_B[0]
            ? wport_obi_req[4*NUM_REQ+r] : wport_obi_req[2*NUM_REQ+r];
        assign wr_mux_req[3*NUM_REQ+r] = sel_w35  // WAGU_A[3] | WAGU_B[1]
            ? wport_obi_req[5*NUM_REQ+r] : wport_obi_req[3*NUM_REQ+r];
        assign wr_mux_req[4*NUM_REQ+r] = wport_obi_req[6*NUM_REQ+r];  // WAGU_D
        assign wr_mux_req[5*NUM_REQ+r] = wport_obi_req[7*NUM_REQ+r];  // WAGU_DMA
    end

    logic [NUM_REQ-1:0] mux2w_gnts, mux3w_gnts;
    logic sel_w24_q, sel_w35_q;
    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_wr_gnt_bits
        assign mux2w_gnts[r] = wr_mux_resp[2*NUM_REQ+r].gnt;
        assign mux3w_gnts[r] = wr_mux_resp[3*NUM_REQ+r].gnt;
    end
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            sel_w24_q <= 1'b0;
            sel_w35_q <= 1'b0;
        end else begin
            if (|mux2w_gnts) sel_w24_q <= sel_w24;
            if (|mux3w_gnts) sel_w35_q <= sel_w35;
        end
    end

    for (genvar r = 0; r < NUM_REQ; r++) begin : gen_wr_demux
        assign wport_obi_resp[0*NUM_REQ+r]        = wr_mux_resp[0*NUM_REQ+r];
        assign wport_obi_resp[1*NUM_REQ+r]        = wr_mux_resp[1*NUM_REQ+r];
        // group 2 (WAGU_A[2])
        assign wport_obi_resp[2*NUM_REQ+r].gnt    = wr_mux_resp[2*NUM_REQ+r].gnt & ~sel_w24;
        assign wport_obi_resp[2*NUM_REQ+r].rvalid = wr_mux_resp[2*NUM_REQ+r].rvalid & ~sel_w24_q;
        assign wport_obi_resp[2*NUM_REQ+r].rdata  = wr_mux_resp[2*NUM_REQ+r].rdata;
        // group 3 (WAGU_A[3])
        assign wport_obi_resp[3*NUM_REQ+r].gnt    = wr_mux_resp[3*NUM_REQ+r].gnt & ~sel_w35;
        assign wport_obi_resp[3*NUM_REQ+r].rvalid = wr_mux_resp[3*NUM_REQ+r].rvalid & ~sel_w35_q;
        assign wport_obi_resp[3*NUM_REQ+r].rdata  = wr_mux_resp[3*NUM_REQ+r].rdata;
        // group 4 (WAGU_B[0])
        assign wport_obi_resp[4*NUM_REQ+r].gnt    = wr_mux_resp[2*NUM_REQ+r].gnt & sel_w24;
        assign wport_obi_resp[4*NUM_REQ+r].rvalid = wr_mux_resp[2*NUM_REQ+r].rvalid & sel_w24_q;
        assign wport_obi_resp[4*NUM_REQ+r].rdata  = wr_mux_resp[2*NUM_REQ+r].rdata;
        // group 5 (WAGU_B[1])
        assign wport_obi_resp[5*NUM_REQ+r].gnt    = wr_mux_resp[3*NUM_REQ+r].gnt & sel_w35;
        assign wport_obi_resp[5*NUM_REQ+r].rvalid = wr_mux_resp[3*NUM_REQ+r].rvalid & sel_w35_q;
        assign wport_obi_resp[5*NUM_REQ+r].rdata  = wr_mux_resp[3*NUM_REQ+r].rdata;
        // groups 6,7 ← xbar 4,5
        assign wport_obi_resp[6*NUM_REQ+r]        = wr_mux_resp[4*NUM_REQ+r];
        assign wport_obi_resp[7*NUM_REQ+r]        = wr_mux_resp[5*NUM_REQ+r];
    end

    // --------------------------------------------------------------------------
    // Level 1: per-port NUM_REQ × NUM_REQ crossbar (operates on muxed port set)
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_RPORT_XBAR-1:0][NUM_REQ-1:0] rd_l1_l2_req;
    obi_resp_t [NUM_RPORT_XBAR-1:0][NUM_REQ-1:0] rd_l1_l2_resp;

    for (genvar j = 0; j < NUM_RPORT_XBAR; j++) begin : gen_read_level_1
        crossbar #(
            .XBAR_NMASTER    (NUM_REQ),
            .XBAR_NSLAVE     (NUM_REQ),
            .SEL_SLICE_START (ROUTE_LSB),
            .SEL_SLICE_LENGTH($clog2(NUM_REQ))
        ) crossbar_level_1_i (
            .clk_i,
            .rst_ni,
            .master_req_i (rd_mux_req[j*NUM_REQ+:NUM_REQ]),
            .master_resp_o(rd_mux_resp[j*NUM_REQ+:NUM_REQ]),
            .slave_req_o  (rd_l1_l2_req[j]),
            .slave_resp_i (rd_l1_l2_resp[j])
        );
    end

    obi_req_t  [NUM_WPORT_XBAR-1:0][NUM_REQ-1:0] wr_l1_l2_req;
    obi_resp_t [NUM_WPORT_XBAR-1:0][NUM_REQ-1:0] wr_l1_l2_resp;

    for (genvar j = 0; j < NUM_WPORT_XBAR; j++) begin : gen_write_level_1
        crossbar #(
            .XBAR_NMASTER    (NUM_REQ),
            .XBAR_NSLAVE     (NUM_REQ),
            .SEL_SLICE_START (ROUTE_LSB),
            .SEL_SLICE_LENGTH($clog2(NUM_REQ))
        ) crossbar_level_1_i (
            .clk_i,
            .rst_ni,
            .master_req_i (wr_mux_req[j*NUM_REQ+:NUM_REQ]),
            .master_resp_o(wr_mux_resp[j*NUM_REQ+:NUM_REQ]),
            .slave_req_o  (wr_l1_l2_req[j]),
            .slave_resp_i (wr_l1_l2_resp[j])
        );
    end

    // --------------------------------------------------------------------------
    // Level 2: per-group NUM_RPORT_XBAR × NUM_BANK_GRP crossbar
    // --------------------------------------------------------------------------
    obi_req_t  [NUM_BANK-1:0] rd_l2_l3_req;
    obi_resp_t [NUM_BANK-1:0] rd_l2_l3_resp;

    for (genvar k = 0; k < NUM_REQ; k++) begin : gen_read_level_2
        obi_req_t  [NUM_RPORT_XBAR-1:0] l2_master_req;
        obi_resp_t [NUM_RPORT_XBAR-1:0] l2_master_resp;
        for (genvar j = 0; j < NUM_RPORT_XBAR; j++) begin : gen_link
            assign l2_master_req[j]    = rd_l1_l2_req[j][k];
            assign rd_l1_l2_resp[j][k] = l2_master_resp[j];
        end
        crossbar #(
            .XBAR_NMASTER    (NUM_RPORT_XBAR),
            .XBAR_NSLAVE     (NUM_BANK_GRP),
            .SEL_SLICE_START (ROUTE_LSB + $clog2(NUM_REQ)),
            .SEL_SLICE_LENGTH($clog2(NUM_BANK_GRP))
        ) crossbar_level_2_i (
            .clk_i,
            .rst_ni,
            .master_req_i (l2_master_req),
            .master_resp_o(l2_master_resp),
            .slave_req_o  (rd_l2_l3_req[k*NUM_BANK_GRP+:NUM_BANK_GRP]),
            .slave_resp_i (rd_l2_l3_resp[k*NUM_BANK_GRP+:NUM_BANK_GRP])
        );
    end

    obi_req_t  [NUM_BANK-1:0] wr_l2_l3_req;
    obi_resp_t [NUM_BANK-1:0] wr_l2_l3_resp;

    for (genvar k = 0; k < NUM_REQ; k++) begin : gen_write_level_2
        obi_req_t  [NUM_WPORT_XBAR-1:0] l2_master_req;
        obi_resp_t [NUM_WPORT_XBAR-1:0] l2_master_resp;
        for (genvar j = 0; j < NUM_WPORT_XBAR; j++) begin : gen_link
            assign l2_master_req[j]    = wr_l1_l2_req[j][k];
            assign wr_l1_l2_resp[j][k] = l2_master_resp[j];
        end
        crossbar #(
            .XBAR_NMASTER    (NUM_WPORT_XBAR),
            .XBAR_NSLAVE     (NUM_BANK_GRP),
            .SEL_SLICE_START (ROUTE_LSB + $clog2(NUM_REQ)),
            .SEL_SLICE_LENGTH($clog2(NUM_BANK_GRP))
        ) crossbar_level_2_i (
            .clk_i,
            .rst_ni,
            .master_req_i (l2_master_req),
            .master_resp_o(l2_master_resp),
            .slave_req_o  (wr_l2_l3_req[k*NUM_BANK_GRP+:NUM_BANK_GRP]),
            .slave_resp_i (wr_l2_l3_resp[k*NUM_BANK_GRP+:NUM_BANK_GRP])
        );
    end

    // --------------------------------------------------------------------------
    // Level 3: per-bank 2 × 2 crossbar (read vs. write onto even/odd bank pair)
    // --------------------------------------------------------------------------
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
            .slave_req_o  (bank_req[k*2+:2]),
            .slave_resp_i (bank_resp[k*2+:2])
        );
    end

    for (genvar i = 0; i < NUM_BANK * 2; i++) begin : gen_banks
        obi_req_t bank_local_req;
        assign bank_local_req = '{
                req: bank_req[i].req,
                we: bank_req[i].we,
                be: bank_req[i].be,
                addr: {
                    {ROUTE_BITS{1'b0}},
                    bank_req[i].addr[31 : ROUTE_LSB+ROUTE_BITS],
                    bank_req[i].addr[ROUTE_LSB-1 : 0]
                },
                wdata: bank_req[i].wdata
            };
        bank #(
            .NUM_ROW       (NUM_ROW / 2),
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
