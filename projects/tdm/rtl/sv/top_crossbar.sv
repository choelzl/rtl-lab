// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Two-level crossbar: 32 AGU inputs → 32 bank outputs.
//
//   Level 1: 8 × system_xbar(NtoM, 4-in, 4-out)
//     AGUs 4j..4j+3 → L1 xbar j → 4 inter-level links (one per L2 group)
//
//   Level 2: 4 × system_xbar(NtoM, 8-in, 8-out)
//     8 inter-level inputs (one per L1 xbar) → 8 banks
//
//   Inter-level: L1 xbar j, slave k  →  L2 xbar k, master j
//
//   Address routing (BANK_ROWS=1024 → BANK_BYTES=4096):
//     L1 map: group k covers [k * L2_NOUT * BANK_BYTES, (k+1) * L2_NOUT * BANK_BYTES)
//     L2 map: bank b in group k covers [(k*L2_NOUT+b)*BANK_BYTES, (k*L2_NOUT+b+1)*BANK_BYTES)
// -----------------------------------------------------------------------------

module top_crossbar
  import obi_pkg::*;
#(
    parameter int unsigned N_AGU     = 7,
    parameter int unsigned N_REQ     = 4,
    parameter int unsigned N_BANK    = 32,
    parameter int unsigned N_ROW     = 1024,
    // Derived — do not override
    localparam int unsigned NUM_AGU   = N_AGU * N_REQ,
    localparam int unsigned L1_NIN    = 4,
    localparam int unsigned L1_NOUT   = 4,
    localparam int unsigned L2_NIN    = 8,
    localparam int unsigned L2_NOUT   = 8,
    localparam int unsigned NUM_L1    = NUM_AGU  / L1_NIN,
    localparam int unsigned NUM_L2    = N_BANK / L2_NOUT,
    localparam int unsigned BANK_BYTES= N_ROW * 4
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic [NUM_AGU-1:0]                    agu_req_i,
    input  logic [NUM_AGU-1:0][31:0]              agu_addr_i,
    input  logic [NUM_AGU-1:0]                    agu_we_i,
    input  logic [NUM_AGU-1:0][`OBI_BE_W-1:0]   agu_be_i,
    input  logic [NUM_AGU-1:0][`OBI_DATA_W-1:0] agu_wdata_i,
    output logic [NUM_AGU-1:0]                    agu_gnt_o,
    output logic [NUM_AGU-1:0]                    agu_rvalid_o,
    output logic [NUM_AGU-1:0][`OBI_DATA_W-1:0] agu_rdata_o
);

  // --------------------------------------------------------------------------
  // Pack flat AGU ports → obi_pkg structs
  // --------------------------------------------------------------------------
  obi_req_t  [NUM_AGU-1:0] agu_obi_req;
  obi_resp_t [NUM_AGU-1:0] agu_obi_resp;

  for (genvar a = 0; a < NUM_AGU; a++) begin : gen_agu_pack
    assign agu_obi_req[a] = '{
      req:   agu_req_i  [a],
      we:    agu_we_i   [a],
      be:    agu_be_i   [a],
      addr:  agu_addr_i [a],
      wdata: agu_wdata_i[a]
    };
    assign agu_gnt_o   [a] = agu_obi_resp[a].gnt;
    assign agu_rvalid_o[a] = agu_obi_resp[a].rvalid;
    assign agu_rdata_o [a] = agu_obi_resp[a].rdata;
  end

  // --------------------------------------------------------------------------
  // Inter-level OBI wires: [L1-xbar-j][slave/L2-xbar-k]
  // --------------------------------------------------------------------------
  obi_req_t  [NUM_L1-1:0][L1_NOUT-1:0] il_req;
  obi_resp_t [NUM_L1-1:0][L1_NOUT-1:0] il_resp;

  // --------------------------------------------------------------------------
  // Level-1: 8 × system_xbar(NtoM, 4-in, 4-out)
  // --------------------------------------------------------------------------
  for (genvar j = 0; j < NUM_L1; j++) begin : gen_l1
    system_xbar #(
        .XBAR_NMASTER(L1_NIN),
        .XBAR_NSLAVE (L1_NOUT),
        .SEL_SLICE_START(0),
        .SEL_SLICE_LENGTH($clog2(L1_NOUT))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (agu_obi_req [j*L1_NIN +: L1_NIN]),
        .master_resp_o(agu_obi_resp[j*L1_NIN +: L1_NIN]),
        .slave_req_o  (il_req [j]),
        .slave_resp_i (il_resp[j])
    );
  end

  // --------------------------------------------------------------------------
  // Bank-side OBI signals
  // --------------------------------------------------------------------------
  obi_req_t  [N_BANK-1:0] bk_obi_req;
  obi_resp_t [N_BANK-1:0] bk_obi_resp;

  // --------------------------------------------------------------------------
  // Level-2: 4 × system_xbar(NtoM, 8-in, 8-out)
  // L2 xbar k, master j  ←  L1 xbar j, slave k
  // --------------------------------------------------------------------------
  for (genvar k = 0; k < NUM_L2; k++) begin : gen_l2
    obi_req_t  [L2_NIN-1:0]       l2_master_req;
    obi_resp_t [L2_NIN-1:0]       l2_master_resp;


    for (genvar j = 0; j < L2_NIN; j++) begin : gen_il
      assign l2_master_req[j]    = il_req        [j][k];
      assign il_resp      [j][k] = l2_master_resp[j];
    end

    system_xbar #(
        .XBAR_NMASTER(L2_NIN),
        .XBAR_NSLAVE (L2_NOUT),
        .SEL_SLICE_START($clog2(L1_NOUT)),
        .SEL_SLICE_LENGTH($clog2(L2_NOUT))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (l2_master_req),
        .master_resp_o(l2_master_resp),
        .slave_req_o  (bk_obi_req [k*L2_NOUT +: L2_NOUT]),
        .slave_resp_i (bk_obi_resp[k*L2_NOUT +: L2_NOUT])
    );
  end

  // --------------------------------------------------------------------------
  // Banks
  // --------------------------------------------------------------------------
  for (genvar i = 0; i < N_BANK; i++) begin : gen_banks
    bank #(
        .NUM_ROW       (N_ROW),
        .WORDS_PER_ROW (1),
        .BYTES_PER_WORD(4)
    ) u_bank (
        .clk_i,
        .rst_ni,
        .obi_req_i (bk_obi_req [i]),
        .obi_resp_o(bk_obi_resp[i])
    );
  end

endmodule
