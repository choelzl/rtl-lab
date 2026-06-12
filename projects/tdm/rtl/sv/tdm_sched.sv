// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Parameterized TDM scheduler.  Consumes a one-hot-capable request vector
//   and produces the index of the manager that should be granted in the
//   current cycle.  The caller must pulse grant_i whenever sel_o is actually
//   acted upon; the scheduler only advances its internal state on that pulse.
//
//   Scheduling policies (POLICY parameter):
//     0 — Round Robin (default)
//           Scans managers 0..N-1 in order starting from the manager after
//           the last one that was granted.  The first manager with req=1 in
//           that rotation wins.  Slot is advanced only on grant, so idle
//           managers are skipped without wasting a cycle.
//
//   Adding a new policy:
//     Add an `else if (POLICY == K) begin : gen_<name>` block that drives
//     sel_o and sel_valid_o.  The block receives req_i, clk_i, rst_ni, and
//     grant_i; it must not break sel_o / sel_valid_o semantics documented
//     above.
// -----------------------------------------------------------------------------

`ifndef TDM_SCHED_SV
`define TDM_SCHED_SV

module tdm_sched #(
    parameter  int unsigned N      = 8,
    parameter  int unsigned POLICY = 0,
    localparam int unsigned SEL_W  = N > 1 ? $clog2(N) : 1
) (
    input  logic             clk_i,
    input  logic             rst_ni,

    input  logic [N-1:0]     req_i,      // pending request per manager
    // Pulsed by the caller when sel_o is actually granted this cycle.
    // The scheduler advances its pointer only when this is asserted.
    input  logic             grant_i,

    output logic [SEL_W-1:0] sel_o,      // index of the selected manager
    output logic             sel_valid_o  // 1 when at least one req_i is set
);

  initial begin : chk_params
    if (N < 1)
      $fatal(1, "tdm_sched: N must be >= 1, got %0d", N);
  end

  if (POLICY == 0) begin : gen_rr

    logic [SEL_W-1:0] ptr_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
      if (!rst_ni) ptr_q <= '0;
      else if (grant_i)
        ptr_q <= (sel_o == SEL_W'(N - 1)) ? '0 : SEL_W'(sel_o + 1'b1);
    end

    // Combinational priority search: starting from ptr_q, find the next
    // manager with req=1, wrapping around the end of the list.
    always_comb begin
      sel_o       = ptr_q;
      sel_valid_o = 1'b0;
      for (int i = 0; i < N; i++) begin
        automatic int unsigned idx = unsigned'((int'(ptr_q) + i) % N);
        if (!sel_valid_o && req_i[idx]) begin
          sel_o       = SEL_W'(idx);
          sel_valid_o = 1'b1;
        end
      end
    end

  end else begin : gen_unknown_policy
    initial $fatal(1, "tdm_sched: unknown POLICY=%0d (only 0=RR is defined)", POLICY);
    assign sel_o       = '0;
    assign sel_valid_o = 1'b0;
  end

endmodule : tdm_sched

`endif // TDM_SCHED_SV
