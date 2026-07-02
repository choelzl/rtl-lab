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
//   active-low (rst_ni): while asserted both outputs are held at seq_[0].
//
//   Programmable slot table: by default the counter walks the identity
//   sequence 0..NUM_AGU-1. set_sequence() replaces it with an arbitrary
//   client list — e.g. a system whose stimuli never drive one of the
//   NUM_AGU clients programs an (NUM_AGU-1)-entry table that skips it, so
//   the free-running rotation stops wasting one bus turn per revolution on
//   a client with nothing to send. Entries may repeat (weighted slots) and
//   order is arbitrary; the table is static configuration (a slot register
//   file in hardware terms), programmed once before reset release, not a
//   per-cycle input.
//
// Template parameters:
//   NUM_AGU - number of client indices (default 2); also the maximum
//             sequence table length
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

    int seq_[NUM_AGU]; // slot table: sequence of client indices to rotate over
    int seq_len_;      // number of valid entries in seq_
    int pos_;          // current position in seq_
    int prev_;         // previous cycle's selected client

    // Program the slot table (static configuration — call before reset
    // release). Each entry must be a valid client index in [0, NUM_AGU).
    void set_sequence(const int *seq, int len) {
        sc_assert(len >= 1 && len <= NUM_AGU);
        for (int i = 0; i < len; ++i) {
            sc_assert(seq[i] >= 0 && seq[i] < NUM_AGU);
            seq_[i] = seq[i];
        }
        seq_len_ = len;
        pos_     = 0;
        prev_    = seq_[0];
    }

    void step() {
        if (!rst_ni.read()) {
            pos_  = 0;
            prev_ = seq_[0];
            sel_req_o.write(seq_[0]);
            sel_rsp_o.write(seq_[0]);
            return;
        }

        sel_req_o.write(seq_[pos_]);
        sel_rsp_o.write(prev_);
        prev_ = seq_[pos_];
        pos_  = (pos_ + 1) % seq_len_;
    }

    SC_CTOR(arbiter) {
        for (int i = 0; i < NUM_AGU; ++i)
            seq_[i] = i;
        seq_len_ = NUM_AGU;
        pos_     = 0;
        prev_    = 0;
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
