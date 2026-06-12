// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   TDM (Time-Division Multiplexed) alternative to top_crossbar.sv.
//
//   External interface is identical to top_crossbar so the two modules are
//   interchangeable.  Internally the crossbar hierarchy (L1/L2/L3 + arbiters)
//   is replaced by a single scheduler that grants exactly one manager per
//   cycle and routes that manager's request directly to the addressed bank.
//
//   Memory layout is compatible with top_crossbar: the same addr_hash function
//   and bank-select bit field are used, so the same physical addresses reach
//   the same banks regardless of which top-level is instantiated.
//
//   All read-AGU and write-AGU manager ports share a single scheduler pool.
//   Manager indices are assigned as:
//     [0 .. NUM_AGU-1]        — read-AGU ports  (agu_*)
//     [NUM_AGU .. NUM_MGRS-1] — write-AGU ports (wagu_*)
//
//   Throughput note:
//     A one-cycle inhibit is inserted after every grant so that no bank ever
//     sees a new req while the previous cycle's rvalid is still asserted
//     (OBI single-outstanding constraint).  Peak throughput is therefore one
//     granted request every two cycles.  To recover the idle cycle for traffic
//     targeting a different bank, replace the global inhibit with per-bank
//     inflight tracking.
//
//   Scheduling policy is selected at elaboration time via SCHED_POLICY:
//     0 — Round Robin (default); see tdm_sched.sv for the full list.
//
// Parameters (user-overridable):
//   N_AGU, N_WAGU, N_REQ, N_BANK, N_ROW, WORDS_PER_ROW, BYTES_PER_WORD
//     — same meaning and defaults as top_crossbar.
//   SCHED_POLICY — forwarded to tdm_sched (0 = RR).
// -----------------------------------------------------------------------------

module top_tdm
  import obi_pkg::*;
#(
    parameter int unsigned N_AGU          = 7,
    parameter int unsigned N_WAGU         = 6,
    parameter int unsigned N_REQ          = 4,
    parameter int unsigned N_BANK         = 32,
    parameter int unsigned N_ROW          = 1024,
    parameter int unsigned WORDS_PER_ROW  = 4,
    parameter int unsigned BYTES_PER_WORD = 4,
    parameter int unsigned SCHED_POLICY   = 0,
    // Derived — do not override
    localparam int unsigned NUM_AGU       = N_AGU  * N_REQ,
    localparam int unsigned NUM_WAGU      = N_WAGU * N_REQ,
    localparam int unsigned NUM_MGRS      = NUM_AGU + NUM_WAGU,
    localparam int unsigned BYTES_PER_ROW = WORDS_PER_ROW * BYTES_PER_WORD,
    localparam int unsigned NUM_BANKS     = N_BANK * 2,
    localparam int unsigned BANK_BYTES    = N_ROW * BYTES_PER_ROW
) (
    input  logic clk_i,
    input  logic rst_ni,

    input  logic [NUM_AGU-1:0]                    agu_req_i,
    input  logic [NUM_AGU-1:0][31:0]              agu_addr_i,
    input  logic [NUM_AGU-1:0]                    agu_we_i,
    input  logic [NUM_AGU-1:0][`OBI_BE_W-1:0]    agu_be_i,
    input  logic [NUM_AGU-1:0][`OBI_DATA_W-1:0]  agu_wdata_i,
    output logic [NUM_AGU-1:0]                    agu_gnt_o,
    output logic [NUM_AGU-1:0]                    agu_rvalid_o,
    output logic [NUM_AGU-1:0][`OBI_DATA_W-1:0]  agu_rdata_o,

    input  logic [NUM_WAGU-1:0]                   wagu_req_i,
    input  logic [NUM_WAGU-1:0][31:0]             wagu_addr_i,
    input  logic [NUM_WAGU-1:0]                   wagu_we_i,
    input  logic [NUM_WAGU-1:0][`OBI_BE_W-1:0]   wagu_be_i,
    input  logic [NUM_WAGU-1:0][`OBI_DATA_W-1:0] wagu_wdata_i,
    output logic [NUM_WAGU-1:0]                   wagu_gnt_o,
    output logic [NUM_WAGU-1:0]                   wagu_rvalid_o,
    output logic [NUM_WAGU-1:0][`OBI_DATA_W-1:0] wagu_rdata_o
);

  // Width of the bank-select field in the hashed address and the manager index.
  localparam int unsigned BANK_SEL_W    = $clog2(NUM_BANKS);
  localparam int unsigned BANK_ROW_START = $clog2(BYTES_PER_ROW) + BANK_SEL_W;
  localparam int unsigned MGR_SEL_W     = NUM_MGRS > 1 ? $clog2(NUM_MGRS) : 1;

  // --------------------------------------------------------------------------
  // Parameter sanity checks (elaboration time)
  // --------------------------------------------------------------------------
  initial begin : chk_params
    if (N_AGU < 1 || N_WAGU < 1 || N_REQ < 1 || N_ROW < 1)
      $fatal(1, "top_tdm: N_AGU/N_WAGU/N_REQ/N_ROW must all be >= 1");
    if (N_BANK < 2)
      $fatal(1, "top_tdm: N_BANK must be >= 2, got %0d", N_BANK);
    if (N_BANK & (N_BANK - 1))
      $fatal(1, "top_tdm: N_BANK (%0d) must be a power of 2", N_BANK);
    if (BANK_ROW_START >= 32)
      $fatal(1, "top_tdm: routing consumes %0d address bits, leaving none for row index",
             BANK_ROW_START);
  end

  // --------------------------------------------------------------------------
  // Address hash — same function as top_crossbar for memory-layout compatibility.
  // addr[11:9] += addr[8:6] folds the bank-group bits into the L2 routing field
  // to spread stride patterns across banks.
  // --------------------------------------------------------------------------
  function automatic logic [31:0] addr_hash(input logic [31:0] addr);
    return {addr[31:0]};
  endfunction

  // --------------------------------------------------------------------------
  // Flatten ports into a unified manager OBI array.
  // --------------------------------------------------------------------------
  obi_req_t  [NUM_MGRS-1:0] mgr_req;
  obi_resp_t [NUM_MGRS-1:0] mgr_resp;

  for (genvar a = 0; a < NUM_AGU; a++) begin : gen_agu_flat
    assign mgr_req[a] = '{
      req:   agu_req_i  [a],
      we:    agu_we_i   [a],
      be:    agu_be_i   [a],
      addr:  addr_hash(agu_addr_i[a]),
      wdata: agu_wdata_i[a]
    };
    assign agu_gnt_o   [a] = mgr_resp[a].gnt;
    assign agu_rvalid_o[a] = mgr_resp[a].rvalid;
    assign agu_rdata_o [a] = mgr_resp[a].rdata;
  end

  for (genvar a = 0; a < NUM_WAGU; a++) begin : gen_wagu_flat
    assign mgr_req[NUM_AGU + a] = '{
      req:   wagu_req_i  [a],
      we:    wagu_we_i   [a],
      be:    wagu_be_i   [a],
      addr:  addr_hash(wagu_addr_i[a]),
      wdata: wagu_wdata_i[a]
    };
    assign wagu_gnt_o   [a] = mgr_resp[NUM_AGU + a].gnt;
    assign wagu_rvalid_o[a] = mgr_resp[NUM_AGU + a].rvalid;
    assign wagu_rdata_o [a] = mgr_resp[NUM_AGU + a].rdata;
  end

  // --------------------------------------------------------------------------
  // Scheduler
  // --------------------------------------------------------------------------
  logic [NUM_MGRS-1:0]  sched_req;
  logic [MGR_SEL_W-1:0] sched_sel;
  logic                  sched_valid;
  logic                  grant_issued;

  for (genvar m = 0; m < NUM_MGRS; m++) begin : gen_req_vec
    assign sched_req[m] = mgr_req[m].req;
  end

  // One-cycle global inhibit after any grant.  Ensures no bank receives a new
  // req while the rvalid from the previous grant is still asserted (OBI
  // single-outstanding).  Replace with per-bank inflight tracking to allow
  // back-to-back grants to distinct banks.
  logic inhibit_q;
  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) inhibit_q <= 1'b0;
    else         inhibit_q <= grant_issued;
  end

  assign grant_issued = sched_valid && !inhibit_q;

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

  // --------------------------------------------------------------------------
  // Grant: only the scheduler-selected manager receives gnt this cycle.
  // --------------------------------------------------------------------------
  for (genvar m = 0; m < NUM_MGRS; m++) begin : gen_gnt
    assign mgr_resp[m].gnt = grant_issued && (MGR_SEL_W'(m) == sched_sel);
  end

  // --------------------------------------------------------------------------
  // Bank routing: forward selected manager's request to the addressed bank.
  // Bank index = hashed_addr[$clog2(BYTES_PER_ROW) +: BANK_SEL_W].
  // --------------------------------------------------------------------------
  logic [BANK_SEL_W-1:0] bk_sel;
  assign bk_sel = mgr_req[sched_sel].addr[$clog2(BYTES_PER_ROW) +: BANK_SEL_W];

  obi_req_t  [NUM_BANKS-1:0] bk_req;
  obi_resp_t [NUM_BANKS-1:0] bk_resp;

  for (genvar b = 0; b < NUM_BANKS; b++) begin : gen_bk_req
    assign bk_req[b].req   = grant_issued && (BANK_SEL_W'(b) == bk_sel);
    assign bk_req[b].addr  = mgr_req[sched_sel].addr;
    assign bk_req[b].we    = mgr_req[sched_sel].we;
    assign bk_req[b].be    = mgr_req[sched_sel].be;
    assign bk_req[b].wdata = mgr_req[sched_sel].wdata;
  end

  // --------------------------------------------------------------------------
  // Response routing: register the granted manager and bank so rvalid/rdata
  // can be steered back to the right manager one cycle later.
  // --------------------------------------------------------------------------
  logic [MGR_SEL_W-1:0]  sel_q;
  logic                   granted_q;
  logic [BANK_SEL_W-1:0] bk_sel_q;

  always_ff @(posedge clk_i or negedge rst_ni) begin
    if (!rst_ni) begin
      sel_q     <= '0;
      granted_q <= 1'b0;
      bk_sel_q  <= '0;
    end else begin
      granted_q <= grant_issued;
      sel_q     <= sched_sel;
      bk_sel_q  <= bk_sel;
    end
  end

  for (genvar m = 0; m < NUM_MGRS; m++) begin : gen_rsp
    assign mgr_resp[m].rvalid = granted_q && (MGR_SEL_W'(m) == sel_q);
    assign mgr_resp[m].rdata  = (granted_q && (MGR_SEL_W'(m) == sel_q))
                                  ? bk_resp[bk_sel_q].rdata : '0;
  end

  // --------------------------------------------------------------------------
  // Banks — same configuration as top_crossbar for layout compatibility.
  // BANK_ROW_START strips the byte-offset and bank-select bits so the bank
  // sees only its local row address.
  // --------------------------------------------------------------------------
  for (genvar i = 0; i < NUM_BANKS; i++) begin : gen_banks
    bank #(
      .NUM_ROW       (N_ROW),
      .WORDS_PER_ROW (1),
      .BYTES_PER_WORD(4),
      .SEL_SLICE_START(BANK_ROW_START)
    ) u_bank (
      .clk_i,
      .rst_ni,
      .obi_req_i (bk_req [i]),
      .obi_resp_o(bk_resp[i])
    );
  end

endmodule : top_tdm
