// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC strict round-robin selector — a free-running counter over
//   NUM_AGU indices that advances EVERY clock cycle, independent of any request
//   or grant (0, 1, ..., NUM_AGU-1, 0, 1, ...). It has no data inputs; it is
//   purely a cyclic index generator.
//
//   Two registered index outputs:
//     - sel_req_o : the index selected THIS cycle.
//     - sel_rsp_o : the index selected the PREVIOUS cycle (sel_req_o delayed by
//                   one clock). Useful for a consumer whose return path lags its
//                   forward path by one cycle and must be steered with the prior
//                   selection.
//
//   sel_req_o starts at 0 on the first cycle after reset release; sel_rsp_o then
//   trails it by exactly one cycle (sel_rsp_o(T) = sel_req_o(T-1)). Reset is
//   active-low (rst_ni): while asserted both outputs are held at 0.
//
// Template parameters:
//   NUM_AGU - number of indices in the round-robin cycle (default 2)
// -----------------------------------------------------------------------------

#ifndef ARBITER_HPP
#define ARBITER_HPP

#include <systemc.h>

template <int NUM_AGU = 2> SC_MODULE(arbiter) {
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;
    sc_out<int> sel_req_o;
    sc_out<int> sel_rsp_o;

    static_assert(NUM_AGU >= 1, "NUM_AGU must be >= 1");

    int sel_;
    int prev_;

    void step() {
        if (!rst_ni.read()) {
            sel_  = 0;
            prev_ = 0;
            sel_req_o.write(0);
            sel_rsp_o.write(0);
            return;
        }

        sel_req_o.write(sel_);
        sel_rsp_o.write(prev_);
        prev_ = sel_;
        sel_  = (sel_ + 1) % NUM_AGU;
    }

    SC_CTOR(arbiter) {
        sel_  = 0;
        prev_ = 0;
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
