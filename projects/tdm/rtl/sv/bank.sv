// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single-port OBI subordinate memory bank.
//
//   Ports use obi_pkg::obi_req_t / obi_resp_t; data/BE widths track OBI_DATA_W / OBI_BE_W
//   (= BYTES_PER_ROW*8 / BYTES_PER_ROW). gnt is combinational; 1-cycle read latency.
//   obi_req_i.addr is a BANK-LOCAL byte address; the bank-select bits are stripped
//   by the upstream crossbar. Row decode: row = addr / (WORDS_PER_ROW * BYTES_PER_WORD).
//   Out-of-range row access triggers $fatal (simulation only).
//
// Parameters:
//   NUM_ROW        — number of rows (depth)
//   WORDS_PER_ROW  — words per row; must satisfy WORDS_PER_ROW * BYTES_PER_WORD == OBI_BE_W
//   BYTES_PER_WORD — bytes per word; must satisfy WORDS_PER_ROW * BYTES_PER_WORD * 8 == OBI_DATA_W
// -----------------------------------------------------------------------------

`ifndef BANK_SV
`define BANK_SV

`include "obi_pkg.sv"

module bank #(
    parameter int NUM_ROW        = 1024,
    parameter int WORDS_PER_ROW  = 4,
    parameter int BYTES_PER_WORD = 4
) (
    input  logic                clk_i,
    input  logic                rst_ni,
    input  obi_pkg::obi_req_t   obi_req_i,
    output obi_pkg::obi_resp_t  obi_resp_o
);

    localparam int ADDR_W = $bits(obi_req_i.addr);
    localparam int DATA_W = $bits(obi_req_i.wdata);
    localparam int BE_W   = $bits(obi_req_i.be);

    initial begin
        if (DATA_W != WORDS_PER_ROW * BYTES_PER_WORD * 8)
            $fatal(1, "bank: DATA_W (%0d) != WORDS_PER_ROW*BYTES_PER_WORD*8 (%0d)",
                   DATA_W, WORDS_PER_ROW * BYTES_PER_WORD * 8);
        if (BE_W != WORDS_PER_ROW * BYTES_PER_WORD)
            $fatal(1, "bank: BE_W (%0d) != WORDS_PER_ROW*BYTES_PER_WORD (%0d)",
                   BE_W, WORDS_PER_ROW * BYTES_PER_WORD);
        if (ADDR_W < $clog2(NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD))
            $fatal(1, "bank: ADDR_W (%0d) too narrow for bank size (need >= %0d bits for %0d bytes)",
                   ADDR_W, $clog2(NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD),
                   NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD);
    end

    logic [DATA_W-1:0] mem [0:NUM_ROW-1];

    logic [ADDR_W-1:0] row_idx;
    assign row_idx = obi_req_i.addr / ADDR_W'(WORDS_PER_ROW * BYTES_PER_WORD);

    assign obi_resp_o.gnt = obi_req_i.req;

    always_ff @(posedge clk_i or negedge rst_ni) begin
        if (!rst_ni) begin
            obi_resp_o.rvalid <= 1'b0;
            obi_resp_o.rdata  <= '0;
        end else begin
            obi_resp_o.rvalid <= obi_req_i.req;
            obi_resp_o.rdata  <= '0;
            if (obi_req_i.req) begin
                // synthesis translate_off
                if (row_idx >= ADDR_W'(NUM_ROW))
                    $fatal(1, "bank: access out of range: row %0d >= capacity %0d",
                           row_idx, NUM_ROW);
                // synthesis translate_on
                if (obi_req_i.we) begin
                    for (int b = 0; b < BE_W; b++) begin
                        if (obi_req_i.be[b])
                            mem[row_idx][8*b +: 8] <= obi_req_i.wdata[8*b +: 8];
                    end
                end else begin
                    obi_resp_o.rdata <= mem[row_idx];
                end
            end
        end
    end

endmodule

`endif // BANK_SV
