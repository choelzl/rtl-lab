// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Self-contained combinational signed multiplier used as the Verilated SV
//   DUT in the SystemC co-simulation demo (see tb/systemc/tb_mul_s.cpp).
//   Computes the signed product of two operands into an (IN_WIDTH_A +
//   IN_WIDTH_B)-bit result.
//
// Parameters:
//   IN_WIDTH_A - bit width of operand A
//   IN_WIDTH_B - bit width of operand B
// -----------------------------------------------------------------------------

`timescale 1 ns / 1 ps

module mul_s #(
    parameter int IN_WIDTH_A = 8,
    parameter int IN_WIDTH_B = 8,

    localparam int OUT_WIDTH = IN_WIDTH_A + IN_WIDTH_B
) (
    input  logic [IN_WIDTH_A-1:0] a_i,
    input  logic [IN_WIDTH_B-1:0] b_i,
    output logic [ OUT_WIDTH-1:0] out_o
);

    assign out_o = OUT_WIDTH'($signed(a_i) * $signed(b_i));

endmodule
