// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single-port OBI subordinate memory bank.
//
//   gnt is combinational (= req); rvalid is registered one cycle after a granted
//   request. A read returns the addressed row, a write applies the byte-enables
//   and returns rdata = '0.
//
//   obi_req_i.addr is a bank-local byte address: the low
//   $clog2(WORDS_PER_ROW*BYTES_PER_WORD) bits are the byte offset within a row,
//   the next $clog2(NUM_ROW) bits select the row.
//
// Parameters:
//   NUM_ROW        — number of rows (depth)
//   WORDS_PER_ROW  — words per row
//   BYTES_PER_WORD — bytes per word
// -----------------------------------------------------------------------------

`include "obi_pkg.sv"

module bank
    import obi_pkg::*;
#(
    parameter int NUM_ROW        = 1024,
    parameter int WORDS_PER_ROW  = 4,
    parameter int BYTES_PER_WORD = 4
) (
    input  logic      clk_i,
    input  logic      rst_ni,
    input  obi_req_t  obi_req_i,
    output obi_resp_t obi_resp_o
);

    logic [WORDS_PER_ROW*BYTES_PER_WORD*8-1:0] mem      [NUM_ROW];

    logic [               $clog2(NUM_ROW)-1:0] row_idx;
    logic                                      rvalid_q;
    logic [               `OBI_DATA_WIDTH-1:0] rdata_q;

    assign row_idx = obi_req_i.addr[$clog2(WORDS_PER_ROW*BYTES_PER_WORD)+:$clog2(NUM_ROW)];

    assign obi_resp_o.gnt = obi_req_i.req;
    assign obi_resp_o.rvalid = rvalid_q;
    assign obi_resp_o.rdata = rdata_q;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            rvalid_q <= 1'b0;
            rdata_q  <= '0;
        end else begin
            rvalid_q <= obi_req_i.req;
            rdata_q  <= '0;
            if (obi_req_i.req) begin
                if (obi_req_i.we) begin
                    for (int b = 0; b < WORDS_PER_ROW * BYTES_PER_WORD; b++) begin
                        if (obi_req_i.be[b]) mem[row_idx][8*b+:8] <= obi_req_i.wdata[8*b+:8];
                    end
                end else begin
                    rdata_q <= mem[row_idx];
                end
            end
        end
    end

endmodule
