// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Two-level crossbar: 32 AGU inputs → 32 bank outputs.
//
//   Level 1: 8 × xbar(NtoM, 4-in, 4-out)
//     AGUs 4j..4j+3 → L1 xbar j → 4 inter-level links (one per L2 group)
//
//   Level 2: 4 × xbar(NtoM, 8-in, 8-out)
//     8 inter-level inputs (one per L1 xbar) → 8 banks
//
//   Inter-level: L1 xbar j, slave k  →  L2 xbar k, master j
//
//   Address routing (BANK_ROWS=1024 → BANK_BYTES=4096):
//     L1 map: group k covers [k * L2_NOUT * BANK_BYTES, (k+1) * L2_NOUT * BANK_BYTES)
//     L2 map: bank b in group k covers [(k*L2_NOUT+b)*BANK_BYTES, (k*L2_NOUT+b+1)*BANK_BYTES)
//
//   Address scramble (applied before routing, see addr_hash):
//     XORs the lower routing/offset bits with the
//     next-higher nibble to spread stride patterns across different banks and
//     reduce conflict probability.
// -----------------------------------------------------------------------------

module top_crossbar
  import obi_pkg::*;
#(
    parameter int unsigned N_AGU     = 7,
    parameter int unsigned N_WAGU    = 6,
    parameter int unsigned N_REQ     = 4,
    parameter int unsigned N_BANK    = 32,
    parameter int unsigned N_ROW     = 1024,
    parameter int unsigned WORDS_PER_ROW  = 4,
    parameter int unsigned BYTES_PER_WORD = 4,
    // Derived — do not override
    localparam int unsigned NUM_AGU    = N_AGU * N_REQ,
    localparam int unsigned NUM_WAGU   = N_WAGU * N_REQ,
    localparam int unsigned NUM_BANK_G = N_BANK / N_REQ,
    localparam int unsigned BYTES_PER_ROW = WORDS_PER_ROW*BYTES_PER_WORD,
    localparam int unsigned BANK_BYTES = N_ROW * BYTES_PER_ROW
) (
    input  logic clk_i,
    input  logic rst_ni,
    input  logic [NUM_AGU-1:0]                    agu_req_i,
    input  logic [NUM_AGU-1:0][31:0]              agu_addr_i,
    input  logic [NUM_AGU-1:0]                    agu_we_i,
    input  logic [NUM_AGU-1:0][`OBI_BE_W-1:0]     agu_be_i,
    input  logic [NUM_AGU-1:0][`OBI_DATA_W-1:0]   agu_wdata_i,
    output logic [NUM_AGU-1:0]                    agu_gnt_o,
    output logic [NUM_AGU-1:0]                    agu_rvalid_o,
    output logic [NUM_AGU-1:0][`OBI_DATA_W-1:0]   agu_rdata_o,
    
    input  logic [NUM_WAGU-1:0]                    wagu_req_i,
    input  logic [NUM_WAGU-1:0][31:0]              wagu_addr_i,
    input  logic [NUM_WAGU-1:0]                    wagu_we_i,
    input  logic [NUM_WAGU-1:0][`OBI_BE_W-1:0]     wagu_be_i,
    input  logic [NUM_WAGU-1:0][`OBI_DATA_W-1:0]   wagu_wdata_i,
    output logic [NUM_WAGU-1:0]                    wagu_gnt_o,
    output logic [NUM_WAGU-1:0]                    wagu_rvalid_o,
    output logic [NUM_WAGU-1:0][`OBI_DATA_W-1:0]   wagu_rdata_o
);

  // --------------------------------------------------------------------------
  // Address scramble: XOR addr[5:2] with addr[8:5] to reduce bank conflicts.
  // Applied to every AGU and WAGU address before routing.
  // --------------------------------------------------------------------------
  function automatic logic [31:0] addr_hash(input logic [31:0] addr);
    return {addr[31:12],  addr[11:9] + addr[8:6], addr[5:0]};
  endfunction

  // --------------------------------------------------------------------------
  // Pack flat AGU ports → obi_pkg structs (scrambled addr_hash)
  // --------------------------------------------------------------------------
  obi_req_t  [NUM_AGU-1:0] agu_obi_req;
  obi_resp_t [NUM_AGU-1:0] agu_obi_resp;

  for (genvar a = 0; a < NUM_AGU; a++) begin : gen_agu_pack
    assign agu_obi_req[a] = '{
      req:   agu_req_i             [a],
      we:    agu_we_i              [a],
      be:    agu_be_i              [a],
      addr:  addr_hash(agu_addr_i[a]),
      wdata: agu_wdata_i           [a]
    };
    assign agu_gnt_o   [a] = agu_obi_resp[a].gnt;
    assign agu_rvalid_o[a] = agu_obi_resp[a].rvalid;
    assign agu_rdata_o [a] = agu_obi_resp[a].rdata;
  end

  // --------------------------------------------------------------------------
  // Pack flat WAGU ports → obi_pkg structs (scrambled addr_hash)
  // --------------------------------------------------------------------------
  obi_req_t  [NUM_WAGU-1:0] wagu_obi_req;
  obi_resp_t [NUM_WAGU-1:0] wagu_obi_resp;

  for (genvar a = 0; a < NUM_WAGU; a++) begin : gen_wagu_pack
    assign wagu_obi_req[a] = '{
      req:   wagu_req_i              [a],
      we:    wagu_we_i               [a],
      be:    wagu_be_i               [a],
      addr:  addr_hash(wagu_addr_i[a]),
      wdata: wagu_wdata_i            [a]
    };
    assign wagu_gnt_o   [a] = wagu_obi_resp[a].gnt;
    assign wagu_rvalid_o[a] = wagu_obi_resp[a].rvalid;
    assign wagu_rdata_o [a] = wagu_obi_resp[a].rdata;
  end

  // --------------------------------------------------------------------------
  // Inter-level OBI wires: [L1-xbar-j][slave/L2-xbar-k]
  // Reading Side
  // --------------------------------------------------------------------------
  obi_req_t  [N_AGU-1:0][N_REQ-1:0] L1_L2_req;
  obi_resp_t [N_AGU-1:0][N_REQ-1:0] L1_L2_resp;

  // --------------------------------------------------------------------------
  // Level-1: 8 × xbar(NtoM, 4-in, 4-out)
  // --------------------------------------------------------------------------
  for (genvar j = 0; j < N_AGU; j++) begin : gen_l1
    xbar #(
        .XBAR_NMASTER(N_REQ),
        .XBAR_NSLAVE (N_REQ),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)),
        .SEL_SLICE_LENGTH($clog2(N_REQ))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (agu_obi_req [j*N_REQ +: N_REQ]),
        .master_resp_o(agu_obi_resp[j*N_REQ +: N_REQ]),
        .slave_req_o  (L1_L2_req [j]),
        .slave_resp_i (L1_L2_resp[j])
    );
  end

  // --------------------------------------------------------------------------
  // Inter-level OBI wires: [L1-xbar-j][slave/L2-xbar-k]
  // Writing Side
  // --------------------------------------------------------------------------
  obi_req_t  [N_WAGU-1:0][N_REQ-1:0] wL1_L2_req;
  obi_resp_t [N_WAGU-1:0][N_REQ-1:0] wL1_L2_resp;

  // --------------------------------------------------------------------------
  // Level-1: AGU × xbar(NtoM, 4-in, 4-out)
  // --------------------------------------------------------------------------
  for (genvar j = 0; j < N_WAGU; j++) begin : gen_wl1
    xbar #(
        .XBAR_NMASTER(N_REQ),
        .XBAR_NSLAVE (N_REQ),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)),
        .SEL_SLICE_LENGTH($clog2(N_REQ))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (wagu_obi_req [j*N_REQ +: N_REQ]),
        .master_resp_o(wagu_obi_resp[j*N_REQ +: N_REQ]),
        .slave_req_o  (wL1_L2_req [j]),
        .slave_resp_i (wL1_L2_resp[j])
    );
  end

  // --------------------------------------------------------------------------
  // Inter-level OBI wires: [L2-xbar-j][L3-xbar-k]
  // Reading Side
  // --------------------------------------------------------------------------
  obi_req_t  [N_BANK-1:0] L2_L3_req;
  obi_resp_t [N_BANK-1:0] L2_L3_resp;

  // --------------------------------------------------------------------------
  // Level-2: 4 × xbar(NtoM, 8-in, 8-out)
  // --------------------------------------------------------------------------
  for (genvar k = 0; k < N_REQ; k++) begin : gen_l2
    obi_req_t  [N_AGU-1:0]       l2_master_req;
    obi_resp_t [N_AGU-1:0]       l2_master_resp;


    for (genvar j = 0; j < N_AGU; j++) begin : gen_il
      assign l2_master_req[j]    = L1_L2_req     [j][k];
      assign L1_L2_resp   [j][k] = l2_master_resp[j];
    end

    xbar #(
        .XBAR_NMASTER(N_AGU),
        .XBAR_NSLAVE (NUM_BANK_G),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)+$clog2(N_REQ)),
        .SEL_SLICE_LENGTH($clog2(NUM_BANK_G))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (l2_master_req),
        .master_resp_o(l2_master_resp),
        .slave_req_o  (L2_L3_req [k*NUM_BANK_G +: NUM_BANK_G]),
        .slave_resp_i (L2_L3_resp[k*NUM_BANK_G +: NUM_BANK_G])
    );
  end

  
  // --------------------------------------------------------------------------
  // Inter-level OBI wires: [L2-xbar-j][L3-xbar-k]
  // Writing Side
  // --------------------------------------------------------------------------
  obi_req_t  [N_BANK-1:0] wL2_L3_req;
  obi_resp_t [N_BANK-1:0] wL2_L3_resp;

  // --------------------------------------------------------------------------
  // Level-2: 4 × xbar(NtoM, 8-in, 8-out)
  // --------------------------------------------------------------------------
  for (genvar k = 0; k < N_REQ; k++) begin : gen_wl2
    obi_req_t  [N_WAGU-1:0]       l2_master_req;
    obi_resp_t [N_WAGU-1:0]       l2_master_resp;


    for (genvar j = 0; j < N_WAGU; j++) begin : gen_il
      assign l2_master_req[j]    = wL1_L2_req     [j][k];
      assign wL1_L2_resp   [j][k] = l2_master_resp[j];
    end

    xbar #(
        .XBAR_NMASTER(N_WAGU),
        .XBAR_NSLAVE (NUM_BANK_G),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)+$clog2(N_REQ)),
        .SEL_SLICE_LENGTH($clog2(NUM_BANK_G))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (l2_master_req),
        .master_resp_o(l2_master_resp),
        .slave_req_o  (wL2_L3_req [k*NUM_BANK_G +: NUM_BANK_G]),
        .slave_resp_i (wL2_L3_resp[k*NUM_BANK_G +: NUM_BANK_G])
    );
  end








  // --------------------------------------------------------------------------
  // Bank-side OBI signals
  // --------------------------------------------------------------------------
  obi_req_t  [(N_BANK*2)-1:0] bk_req;
  obi_resp_t [(N_BANK*2)-1:0] bk_resp;

  // --------------------------------------------------------------------------
  // Level-3: NBANK × xbar(NtoM, 2-in, 2-out)
  // Handles Even/Odd
  // --------------------------------------------------------------------------
  for (genvar k = 0; k < N_BANK; k++) begin : gen_l3
    obi_req_t  [1:0]       l3_master_req;
    obi_resp_t [1:0]       l3_master_resp;

    assign l3_master_req[0] = L2_L3_req     [k];
    assign L2_L3_resp   [k] = l3_master_resp[0];
    
    assign l3_master_req[1] = wL2_L3_req    [k];
    assign wL2_L3_resp  [k] = l3_master_resp[1];

    xbar #(
        .XBAR_NMASTER(2),
        .XBAR_NSLAVE (2),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)+$clog2(N_REQ)+$clog2(NUM_BANK_G)),
        .SEL_SLICE_LENGTH($clog2(2))
    ) u_xbar (
        .clk_i,
        .rst_ni,
        .default_idx_i('0),
        .master_req_i (l3_master_req),
        .master_resp_o(l3_master_resp),
        .slave_req_o  (bk_req [k*2 +: 2]),
        .slave_resp_i (bk_resp[k*2 +: 2])
    );
  end

  // --------------------------------------------------------------------------
  // Banks
  // --------------------------------------------------------------------------
  for (genvar i = 0; i < N_BANK*2; i++) begin : gen_banks
    bank #(
        .NUM_ROW       (N_ROW),
        .WORDS_PER_ROW (1),
        .BYTES_PER_WORD(4),
        .SEL_SLICE_START($clog2(BYTES_PER_ROW)+$clog2(N_REQ)+$clog2(NUM_BANK_G)+$clog2(2))
    ) u_bank (
        .clk_i,
        .rst_ni,
        .obi_req_i (bk_req [i]),
        .obi_resp_o(bk_resp[i])
    );
  end

endmodule
