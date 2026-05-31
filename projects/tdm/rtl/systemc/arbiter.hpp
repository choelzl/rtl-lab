// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC round-robin arbiter for the TDM design. It is a free-running
//   scheduler — a strict round-robin counter over the NUM_AGU AGUs that advances
//   EVERY clock cycle regardless of whether any AGU has a request that cycle
//   (0, 1, ..., NUM_AGU-1, 0, 1, ...). It takes no req/gnt inputs; it is the
//   single owner of the time-slot schedule that the OBI mux/demux follow.
//
//   It exposes two registered selection outputs:
//     - sel_req_o : the AGU served THIS cycle — steers the request path
//                   (obi_mux selects buffer[sel_req] toward the mapping/banks).
//     - sel_rsp_o : sel_req delayed by ONE cycle — steers the response path
//                   (obi_mux routes the bank response back to buffer[sel_rsp]).
//
//   Why the response lag: a request issued at cycle T reaches the bank at T and
//   the bank answers at T+1 (1-cycle bank latency), by which point sel_req has
//   already advanced. Routing the response therefore needs the selection from
//   the previous cycle, hence sel_rsp = sel_req delayed one register.
//
//   TIMING ASSUMPTION: the 1-cycle response lag holds only while the datapath
//   between the mux and the banks (TDM mapping + conflicts checker) stays
//   combinational and bank latency is 1. If that path is pipelined deeper later,
//   sel_rsp's delay must grow to match.
//
//   sel_req_o counts from 0 on the first cycle after reset release. Reset is
//   active-low (rst_ni): while asserted both outputs are held at 0.
//
// Template parameters (NUM_AGU from the PARAMS macro N_AGU):
//   NUM_AGU - number of AGUs / round-robin slots (default 2)
// -----------------------------------------------------------------------------

#ifndef ARBITER_HPP
#define ARBITER_HPP

#include <systemc.h>

template <int NUM_AGU = 2>
SC_MODULE(arbiter) {
    sc_in<bool>  clk_i;
    sc_in<bool>  rst_ni;
    sc_out<int>  sel_req_o;
    sc_out<int>  sel_rsp_o;

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
