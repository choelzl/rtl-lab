// -----------------------------------------------------------------------------
// Native SystemC request-aware round-robin selector — an alternative to
// arbiter.hpp's free-running counter. That counter advances through all
// NUM_AGU indices every clock cycle unconditionally, with no visibility into
// which index actually has pending work: when only one of NUM_AGU requesters
// is active (the common case for top_tdm.hpp's 9 buffers, since most stimuli
// only drive one or two of RAGU_A/B/C/D/E and WAGU_A/B/D/E at a time),
// that requester still only gets the shared TDM bus on 1 cycle out of every
// NUM_AGU, wasting the other NUM_AGU-1 cycles on buffers with nothing to
// send. This module keeps the same round-robin FAIRNESS policy among active
// requesters (advance a search pointer past whichever index wins, so one
// always-busy requester can't starve another) but SKIPS indices with no
// pending request entirely, closing that gap.
//
// The grant is COMBINATIONAL — a priority encoder from the registered
// round-robin pointer over this cycle's req_i, so a request is granted the
// same cycle it is raised. (An earlier version registered the whole scan,
// i.e. sampled req_i at the clock edge and granted one cycle later: after
// every idle gap the bus took an extra cycle to re-acquire, which showed up
// as exactly +1 dead cycle per 32-beat window turnaround on every buffered
// read stream — the window's refetch request rises, the grant lands a cycle
// later, and the whole fetch->rvalid->drain pipeline shifts by one. A
// combinational grant is also what "request-aware arbitration" means in
// hardware terms: the encoder sits in the request path's own cycle; only
// the fairness state advances on the clock.)
//
//   req_i[NUM_AGU] : one input per index — asserted when that index has ANY
//                    pending work this cycle (top_tdm.hpp feeds this from an
//                    OR-reduction of that buffer's own NUM_BANK req wires).
//   sel_req_o      : the index granted THIS cycle (combinational), or -1 if
//                    no req_i is asserted (arbiter.hpp never outputs -1;
//                    every consumer of arb_req_sel already guards with
//                    `sel >= 0` for other reasons, so -1 is handled without
//                    any further wiring changes — see top_tdm.hpp's
//                    mux_comb()/map_cfg_comb()).
//   sel_rsp_o      : sel_req_o delayed by one clock, same as arbiter.hpp —
//                    a consumer whose return path lags its forward path by
//                    one cycle steers with this delayed selection instead.
//
// Reset is active-low (rst_ni): while asserted, both outputs are held at -1
// and the search pointer resets to 0.
//
// Template parameters:
//   NUM_AGU - number of indices in the round-robin cycle (default 2)
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
