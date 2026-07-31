// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Native SystemC strict round-robin selector — a free-running counter over
// NUM_AGU indices, advancing every clock cycle independent of request/grant
// (0,1,...,NUM_AGU-1,0,1,...). No data inputs, purely a cyclic generator.
//
// sel_req_o: index selected this cycle. sel_rsp_o: sel_req_o delayed one
// clock, for a consumer whose return path lags its forward path. Both start
// at seq_[0] after reset (active-low rst_ni).
//
// Programmable slot table (set_sequence()): by default walks 0..NUM_AGU-1;
// a deployment that never drives one client can program a shorter table
// skipping it, so the rotation stops wasting a turn on an idle client.
// Entries may repeat (weighted slots); static config, set before reset
// release, not a per-cycle input.
//
// Template param: NUM_AGU (client count, default 2; also max table length).
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
