// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   SystemC design top for the co-simulation demo. Wraps the Verilated
//   SystemVerilog multiplier (mul_s) and the native SystemC accumulator (acc)
//   into a single structural module, wiring the multiplier's product to the
//   accumulator through an internal sc_signal. The harness (tb/systemc/
//   tb_mul_s.cpp) instantiates this top and only drives its external ports.
//
//     top_mul_s
//       |- Vmul_s dut   (Verilated SV combinational signed multiplier)
//       |- acc    accu  (native SystemC clocked accumulator)
//
//   Operand and product widths are compile-time; OUT_WIDTH must equal
//   IN_WIDTH_A + IN_WIDTH_B of the Verilated mul_s and is forwarded to the
//   accumulator for sign extension.
//
// Template parameters:
//   OUT_WIDTH - bit width of the multiplier product (IN_WIDTH_A + IN_WIDTH_B)
// -----------------------------------------------------------------------------

#ifndef TOP_MUL_S_HPP
#define TOP_MUL_S_HPP

#include <systemc.h>
#include <cstdint>

#include "Vmul_s.h"
#include "acc.hpp"

template <int OUT_WIDTH>
SC_MODULE(top_mul_s) {
    sc_in<bool>     clk_i;
    sc_in<bool>     rst_i;
    sc_in<uint32_t> a_i;
    sc_in<uint32_t> b_i;
    sc_out<int>     sum_o;

    sc_signal<uint32_t> prod;

    Vmul_s          dut;
    acc<OUT_WIDTH>  accu;

    SC_CTOR(top_mul_s) : dut("dut"), accu("acc") {
        dut.a_i  (a_i);
        dut.b_i  (b_i);
        dut.out_o(prod);

        accu.clk_i (clk_i);
        accu.rst_i (rst_i);
        accu.prod_i(prod);
        accu.sum_o (sum_o);
    }
};

#endif
