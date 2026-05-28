// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC accumulator model. On every rising clock edge it adds the
//   incoming product to a running signed sum (cleared while rst_i is high).
//   Used by the SystemC demo harness to show a hand-written SC module running
//   cycle-accurately alongside a Verilated SystemVerilog DUT (the mul_s
//   multiplier), connected through sc_signal nets under the SystemC kernel.
//
//   The product arrives as a raw PROD_WIDTH-bit value carried in Verilator's
//   default uint32_t SystemC port type, so it is sign-extended here before
//   being accumulated.
//
// Template parameters:
//   PROD_WIDTH - bit width of the incoming product (for sign extension)
// -----------------------------------------------------------------------------

#ifndef ACC_HPP
#define ACC_HPP

#include <systemc.h>
#include <cstdint>

template <int PROD_WIDTH>
SC_MODULE(acc) {
    sc_in<bool>     clk_i;
    sc_in<bool>     rst_i;
    sc_in<uint32_t> prod_i;
    sc_out<int>     sum_o;

    void step() {
        if (rst_i.read()) {
            sum_o.write(0);
            return;
        }
        const int m    = 1 << (PROD_WIDTH - 1);
        const int mask = (1 << PROD_WIDTH) - 1;
        const int prod = (static_cast<int>(prod_i.read() & mask) ^ m) - m;
        sum_o.write(sum_o.read() + prod);
    }

    SC_CTOR(acc) {
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
