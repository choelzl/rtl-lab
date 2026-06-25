// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   TDM (Time-Division Multiplexed) alternative to top_crossbar.sv.
//
//   The crossbar hierarchy (L1/L2/L3 + arbiters) is replaced by a single
//   scheduler that grants exactly one manager per cycle and routes that
//   manager's request directly to the addressed bank.
//
//   Bank routing is driven externally: each read/write port declares its current
//   target bank via rport_bk_i / wport_bk_i; the row address for each bank is supplied
//   via bank_addr_i[NUM_BANK] as a bank-local byte address.  The scheduler
//   selection is exposed on sched_sel_o / grant_issued_o so that external
//   address-generation logic can advance its per-bank address pointers after
//   each grant.
//
//   All read and write ports share a single scheduler pool. Manager indices are
//   assigned as:
//     [0 .. NUM_RPORT-1]             — read ports
//     [NUM_RPORT .. NUM_MGRS-1]      — write ports
//
//   Per-manager prefetch buffer:
//     Each manager has an NUM_BANK × WORDS_PER_ROW word buffer.  The TDM fill
//     path writes each bank rvalid word into the entry for the in-flight bank.
//     The consumer path grants the manager immediately when the buffer holds
//     data for its requested bank, returning rvalid one cycle later at
//     1 word/cycle — hiding TDM/network latency from RR scheduling.
//
//   Throughput note:
//     A one-cycle inhibit is inserted after every grant so that no bank ever
//     sees a new req while the previous cycle's rvalid is still asserted
//     (OBI single-outstanding constraint).  Peak throughput is therefore one
//     granted request every two cycles.
//
//   Scheduling policy is selected at elaboration time via SCHED_POLICY:
//     0 — Round Robin (default); see tdm_sched.sv for the full list.
//
// Parameters (user-overridable):
//   NUM_RPORT, NUM_WPORT — read/write port counts.
//   NUM_REQ              — OBI buses per port.
//   NUM_BANK, NUM_ROW, WORDS_PER_ROW, BYTES_PER_WORD — same defaults as top_crossbar.
//   SCHED_POLICY   — forwarded to tdm_sched (0 = RR).
// -----------------------------------------------------------------------------

`include "obi_pkg.sv"

module top_tdm
    import obi_pkg::*;
#(
    parameter int NUM_RPORT      = 9,
    parameter int NUM_WPORT      = 8,
    parameter int NUM_REQ        = 4,
    parameter int NUM_BANK       = 32,
    parameter int NUM_ROW        = 1024,
    parameter int WORDS_PER_ROW  = 4,
    parameter int BYTES_PER_WORD = 4,
    parameter int SCHED_POLICY   = 0,

    localparam int NUM_RD         = NUM_RPORT * NUM_REQ,
    localparam int NUM_WR         = NUM_WPORT * NUM_REQ,
    localparam int NUM_MGRS       = NUM_RPORT + NUM_WPORT,
    localparam int NUM_BANK_GRP   = NUM_BANK / NUM_REQ,
    localparam int BYTES_PER_ROW  = WORDS_PER_ROW * BYTES_PER_WORD,
    localparam int MGR_SEL_W      = $clog2(NUM_MGRS),
    localparam int BANK_SEL_W     = $clog2(NUM_BANK),
    localparam int WRD_PTR_W      = $clog2(WORDS_PER_ROW + 1),
    localparam int WRD_IDX_W      = $clog2(WORDS_PER_ROW),
    localparam int BYTE_OFF_W     = $clog2(BYTES_PER_ROW),
    localparam int ROUTE_BITS     = $clog2(NUM_REQ) + $clog2(NUM_BANK_GRP) + 1,
    localparam int L1_SEL_START   = BYTE_OFF_W,
    localparam int L2_SEL_START   = BYTE_OFF_W + $clog2(NUM_REQ),
    localparam int BANK_ROW_START = BYTE_OFF_W + ROUTE_BITS
) (
    input  logic                   clk_i,
    input  logic                   rst_ni,
    input  obi_req_t  [NUM_RD-1:0] rport_req_i,
    output obi_resp_t [NUM_RD-1:0] rport_resp_o,
    input  obi_req_t  [NUM_WR-1:0] wport_req_i,
    output obi_resp_t [NUM_WR-1:0] wport_resp_o,

    // Per-manager bank selector and per-bank row address (bank-local format)
    input logic [BANK_SEL_W-1:0] rport_bk_i [NUM_RPORT],
    input logic [BANK_SEL_W-1:0] wport_bk_i [NUM_WPORT],
    input logic [          31:0] bank_addr_i[ NUM_BANK],

    // Scheduler outputs for external address-generation logic
    output logic [MGR_SEL_W-1:0] sched_sel_o,
    output logic                 grant_issued_o
);

    // --------------------------------------------------------------------------
    // Unified manager OBI array (one entry per read/write port)
    // --------------------------------------------------------------------------
    obi_req_t  [       NUM_MGRS-1:0] mgr_req;
    obi_resp_t [       NUM_MGRS-1:0] mgr_resp;

    // --------------------------------------------------------------------------
    // Prefetch buffer: one entry per (manager, bank), WORDS_PER_ROW words deep
    // --------------------------------------------------------------------------
    logic      [`OBI_DATA_WIDTH-1:0] pbuf_data  [NUM_MGRS] [NUM_BANK] [WORDS_PER_ROW];
    logic      [      WRD_PTR_W-1:0] pbuf_wr_ptr[NUM_MGRS] [NUM_BANK];
    logic      [      WRD_PTR_W-1:0] pbuf_rd_ptr[NUM_MGRS] [NUM_BANK];
    logic      [     BANK_SEL_W-1:0] pbuf_bk_if [NUM_MGRS];
    logic                            pbuf_rvalid[NUM_MGRS];
    logic      [`OBI_DATA_WIDTH-1:0] pbuf_rdata [NUM_MGRS];

    // --------------------------------------------------------------------------
    // Flatten read/write ports into unified mgr_req / mgr_resp
    // --------------------------------------------------------------------------
    for (genvar a = 0; a < NUM_RPORT; a++) begin : gen_rport_flat
        assign mgr_req[a] = '{
                req: rport_req_i[a*NUM_REQ].req,
                we: rport_req_i[a*NUM_REQ].we,
                be: rport_req_i[a*NUM_REQ].be,
                addr: bank_addr_i[rport_bk_i[a]],
                wdata: rport_req_i[a*NUM_REQ].wdata
            };
        for (genvar r = 0; r < NUM_REQ; r++) begin : gen_rport_resp
            assign rport_resp_o[a*NUM_REQ+r].gnt    = mgr_resp[a].gnt;
            assign rport_resp_o[a*NUM_REQ+r].rvalid = mgr_resp[a].rvalid;
            assign rport_resp_o[a*NUM_REQ+r].rdata  = mgr_resp[a].rdata;
        end
    end

    for (genvar a = 0; a < NUM_WPORT; a++) begin : gen_wport_flat
        assign mgr_req[NUM_RPORT+a] = '{
                req: wport_req_i[a*NUM_REQ].req,
                we: wport_req_i[a*NUM_REQ].we,
                be: wport_req_i[a*NUM_REQ].be,
                addr: bank_addr_i[wport_bk_i[a]],
                wdata: wport_req_i[a*NUM_REQ].wdata
            };
        for (genvar r = 0; r < NUM_REQ; r++) begin : gen_wport_resp
            assign wport_resp_o[a*NUM_REQ+r].gnt    = mgr_resp[NUM_RPORT+a].gnt;
            assign wport_resp_o[a*NUM_REQ+r].rvalid = mgr_resp[NUM_RPORT+a].rvalid;
            assign wport_resp_o[a*NUM_REQ+r].rdata  = mgr_resp[NUM_RPORT+a].rdata;
        end
    end

    // --------------------------------------------------------------------------
    // Scheduler
    // --------------------------------------------------------------------------
    logic [ NUM_MGRS-1:0] sched_req;
    logic [MGR_SEL_W-1:0] sched_sel;
    logic                 sched_valid;
    logic                 grant_issued;

    for (genvar m = 0; m < NUM_MGRS; m++) begin : gen_req_vec
        assign sched_req[m] = mgr_req[m].req;
    end

    // One-cycle global inhibit after any grant — ensures no bank receives a new
    // req while the rvalid from the previous grant is still asserted (OBI
    // single-outstanding).
    logic inhibit_q;
    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) inhibit_q <= 1'b0;
        else inhibit_q <= grant_issued;
    end

    assign grant_issued = sched_valid && !inhibit_q;

    // tdm_sched.sv — TODO: implement
    tdm_sched #(
        .N     (NUM_MGRS),
        .POLICY(SCHED_POLICY)
    ) u_sched (
        .clk_i,
        .rst_ni,
        .req_i      (sched_req),
        .grant_i    (grant_issued),
        .sel_o      (sched_sel),
        .sel_valid_o(sched_valid)
    );

    assign sched_sel_o    = sched_sel;
    assign grant_issued_o = grant_issued;

    // --------------------------------------------------------------------------
    // Per-manager bank selector
    // --------------------------------------------------------------------------
    logic [NUM_MGRS-1:0][BANK_SEL_W-1:0] mgr_bk;
    for (genvar m = 0; m < NUM_RPORT; m++) begin : gen_rport_bk
        assign mgr_bk[m] = rport_bk_i[m];
    end
    for (genvar m = 0; m < NUM_WPORT; m++) begin : gen_wport_bk
        assign mgr_bk[NUM_RPORT+m] = wport_bk_i[m];
    end

    // --------------------------------------------------------------------------
    // Prefetch buffer: per-manager response and drain/fill path
    // --------------------------------------------------------------------------
    for (genvar m = 0; m < NUM_MGRS; m++) begin : gen_pbuf
        logic pbuf_has_data;
        assign pbuf_has_data      = (pbuf_wr_ptr[m][mgr_bk[m]] != pbuf_rd_ptr[m][mgr_bk[m]]);
        assign mgr_resp[m].gnt    = mgr_req[m].req && pbuf_has_data;
        assign mgr_resp[m].rvalid = pbuf_rvalid[m];
        assign mgr_resp[m].rdata  = pbuf_rdata[m];

        always_ff @(posedge clk_i or negedge rst_ni) begin
            if (!rst_ni) begin
                pbuf_rvalid[m] <= 1'b0;
                pbuf_rdata[m]  <= '0;
                pbuf_bk_if[m]  <= '0;
                for (int b = 0; b < NUM_BANK; b++) begin
                    pbuf_wr_ptr[m][b] <= '0;
                    pbuf_rd_ptr[m][b] <= '0;
                end
            end else begin
                // Capture bank at TDM grant (one cycle before bank rvalid)
                if (grant_issued && (MGR_SEL_W'(m) == sched_sel)) pbuf_bk_if[m] <= mgr_bk[m];

                // Drain: deliver one word per cycle to the port, advance read pointer
                pbuf_rvalid[m] <= mgr_resp[m].gnt;
                if (mgr_resp[m].gnt) begin
                    pbuf_rdata[m]                    <= pbuf_data[m][mgr_bk[m]]
                                                         [pbuf_rd_ptr[m][mgr_bk[m]][WRD_IDX_W-1:0]];
                    pbuf_rd_ptr[m][mgr_bk[m]] <= pbuf_rd_ptr[m][mgr_bk[m]] + 1;
                end

                // TODO: fill path — on bank rvalid write into pbuf_data[m][pbuf_bk_if[m]]
                //       and advance pbuf_wr_ptr[m][pbuf_bk_if[m]].
            end
        end
    end

    // --------------------------------------------------------------------------
    // Bank routing: forward granted manager's request to the addressed bank
    // --------------------------------------------------------------------------
    logic      [BANK_SEL_W-1:0] bk_sel;
    obi_req_t  [  NUM_BANK-1:0] bk_req;
    obi_resp_t [  NUM_BANK-1:0] bk_resp;

    assign bk_sel = mgr_bk[sched_sel];

    for (genvar b = 0; b < NUM_BANK; b++) begin : gen_bk_req
        assign bk_req[b].req   = grant_issued && (BANK_SEL_W'(b) == bk_sel);
        assign bk_req[b].addr  = bank_addr_i[b];
        assign bk_req[b].we    = mgr_req[sched_sel].we;
        assign bk_req[b].be    = mgr_req[sched_sel].be;
        assign bk_req[b].wdata = mgr_req[sched_sel].wdata;
    end

    // --------------------------------------------------------------------------
    // Banks
    // --------------------------------------------------------------------------
    for (genvar i = 0; i < NUM_BANK; i++) begin : gen_banks
        bank #(
            .NUM_ROW       (NUM_ROW),
            .WORDS_PER_ROW (WORDS_PER_ROW),
            .BYTES_PER_WORD(BYTES_PER_WORD)
        ) u_bank (
            .clk_i,
            .rst_ni,
            .obi_req_i (bk_req[i]),
            .obi_resp_o(bk_resp[i])
        );
    end

endmodule : top_tdm
