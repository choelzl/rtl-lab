// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single-port OBI subordinate memory bank. One OBI beat transfers one full
//   row (WORDS_PER_ROW words); the OBI interface must therefore be instantiated
//   with DATA_W = WORDS_PER_ROW * BYTES_PER_WORD * 8.
//
//   obi.gnt is combinational (follows obi.req). 1-cycle access latency.
//   Active-low reset (rst_ni). Byte-enable writes across the full row.
//   Out-of-range row index triggers $fatal (simulation only).
//
//   obi.addr is a BANK-LOCAL byte address — the bank-select field has already
//   been stripped by the upstream interconnect.
//   Row decode: row = addr / (WORDS_PER_ROW * BYTES_PER_WORD).
//
// Parameters (mapped from PARAMS macros via the instantiating top-level):
//   NUM_ROW        — depth in rows              (maps from N_ROW,    default 1024)
//   WORDS_PER_ROW  — words per row              (maps from N_REQ,    default 4)
//   BYTES_PER_WORD — bytes per data word         (maps from WORD_BYTES, default 4)
//
//   The instantiating module must set:
//     obi_if #(.DATA_W(WORDS_PER_ROW * BYTES_PER_WORD * 8)) obi_inst (...);
//   A mismatch is caught by an elaboration-time assertion.
// -----------------------------------------------------------------------------

`ifndef BANK_SV
`define BANK_SV

`include "obi.sv"

module bank #(
    parameter int NUM_ROW        = 1024,
    parameter int WORDS_PER_ROW  = 4,
    parameter int BYTES_PER_WORD = 4
) (
    input  logic   clk_i,
    input  logic   rst_ni,
    obi_if.sub     obi
);

    localparam int ADDR_W = $bits(obi.addr);
    localparam int DATA_W = $bits(obi.wdata);  // actual interface data width
    localparam int BE_W   = $bits(obi.be);

    initial begin
        if (DATA_W != WORDS_PER_ROW * BYTES_PER_WORD * 8)
            $fatal(1, "bank: obi DATA_W (%0d) != WORDS_PER_ROW*BYTES_PER_WORD*8 (%0d)",
                   DATA_W, WORDS_PER_ROW * BYTES_PER_WORD * 8);
        if (BE_W != WORDS_PER_ROW * BYTES_PER_WORD)
            $fatal(1, "bank: obi BE_W (%0d) != WORDS_PER_ROW*BYTES_PER_WORD (%0d)",
                   BE_W, WORDS_PER_ROW * BYTES_PER_WORD);
        if (ADDR_W < $clog2(NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD))
            $fatal(1, "bank: obi ADDR_W (%0d) too narrow for bank size (need >= %0d bits for %0d bytes)",
                   ADDR_W, $clog2(NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD),
                   NUM_ROW * WORDS_PER_ROW * BYTES_PER_WORD);
    end

    logic [DATA_W-1:0] mem [0:NUM_ROW-1];

    // Row index from bank-local byte address
    logic [ADDR_W-1:0] row_idx;
    assign row_idx = obi.addr / ADDR_W'(WORDS_PER_ROW * BYTES_PER_WORD);

    // Combinational grant — bank never back-pressures; contention resolved upstream
    assign obi.gnt = obi.req;

    always_ff @(posedge clk_i) begin
        if (!rst_ni) begin
            obi.rvalid <= 1'b0;
            obi.rdata  <= '0;
        end else begin
            obi.rvalid <= obi.req;
            obi.rdata  <= '0;
            if (obi.req) begin
                // synthesis translate_off
                if (row_idx >= ADDR_W'(NUM_ROW))
                    $fatal(1, "bank: OBI access out of range: row %0d >= capacity %0d",
                           row_idx, NUM_ROW);
                // synthesis translate_on
                if (obi.we) begin
                    for (int b = 0; b < BE_W; b++) begin
                        if (obi.be[b])
                            mem[row_idx][8*b +: 8] <= obi.wdata[8*b +: 8];
                    end
                end else begin
                    obi.rdata <= mem[row_idx];
                end
            end
        end
    end

endmodule

`endif // BANK_SV
