// -----------------------------------------------------------------------------
// Request-aware round-robin selector, an alternative to arbiter.hpp's
// free-running counter (which visits all NUM_AGU indices every cycle
// unconditionally, wasting cycles when only one requester is active — the
// common case for top_tdm.hpp's 9 buffers). Keeps the same round-robin
// fairness among active requesters but skips indices with no pending
// request, closing that gap.
//
// Grant is COMBINATIONAL (priority encoder off the registered pointer over
// this cycle's req_i) — an earlier registered-scan version cost +1 dead
// cycle per window turnaround on every buffered read stream.
//
// req_i[NUM_AGU]: one per index, OR-reduced from that buffer's own req
// wires. sel_req_o: index granted this cycle, or -1 if none (arbiter.hpp
// never outputs -1, but every consumer already guards `sel >= 0`).
// sel_rsp_o: sel_req_o delayed one clock, same as arbiter.hpp. Reset
// (active-low) holds both outputs at -1 and resets the search pointer.
// -----------------------------------------------------------------------------

#ifndef ARBITER_ADAPTIVE_HPP
#define ARBITER_ADAPTIVE_HPP

#include <systemc.h>

template <int NUM_AGU = 2> SC_MODULE(arbiter_adaptive) {
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;
    sc_in<bool> req_i[NUM_AGU];
    sc_out<int> sel_req_o;
    sc_out<int> sel_rsp_o;

    static_assert(NUM_AGU >= 1, "NUM_AGU must be >= 1");

    // Round-robin search start — an sc_signal (not a plain member) so the
    // combinational grant re-evaluates when it advances.
    sc_signal<int> rr_ptr_q;

    // Priority-encode from `from` over this cycle's requests; -1 when idle.
    int scan(int from) const {
        for (int k = 0; k < NUM_AGU; ++k) {
            const int idx = (from + k) % NUM_AGU;
            if (req_i[idx].read())
                return idx;
        }
        return -1;
    }

    void comb() {
        sel_req_o.write(rst_ni.read() ? scan(rr_ptr_q.read()) : -1);
    }

    void seq() {
        if (!rst_ni.read()) {
            rr_ptr_q.write(0);
            sel_rsp_o.write(-1);
            return;
        }
        // Re-derive this (ending) cycle's winner from the same settled req_i
        // the combinational grant saw — that is who the response one cycle
        // from now belongs to, and who the fairness pointer must move past.
        const int winner = scan(rr_ptr_q.read());
        sel_rsp_o.write(winner);
        if (winner >= 0)
            rr_ptr_q.write((winner + 1) % NUM_AGU);
    }

    SC_CTOR(arbiter_adaptive) {
        SC_METHOD(comb);
        sensitive << rst_ni << rr_ptr_q;
        for (int i = 0; i < NUM_AGU; ++i)
            sensitive << req_i[i];
        SC_METHOD(seq);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
