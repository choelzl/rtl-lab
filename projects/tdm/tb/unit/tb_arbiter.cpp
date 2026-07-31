// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for BOTH TDM-bus selectors, head to head:
//   arbiter<NUM_AGU=4>          — free-running counter (arbiter.hpp)
//   arbiter_adaptive<NUM_AGU=4> — request-aware round-robin (arbiter_adaptive.hpp)
//
// Until now these were only exercised indirectly (top_tdm integration plus
// stim_bank's phase-5 exact-cycle timing constants in both build variants) —
// a failure there says "some timing moved", not "the arbiter broke". This
// pins each module's own contract directly.
//
// Tests:
//   T01: free-running — reset holds both outputs at 0
//   T02: free-running — sel_req cycles 0,1,2,3,0,... every cycle, no inputs
//   T03: free-running — sel_rsp trails sel_req by exactly one cycle
//   T04: free-running — mid-run reset re-zeros and restarts the cycle
//   T05: adaptive — reset holds both outputs at -1
//   T06: adaptive — no requesters -> sel_req stays -1 (and sel_rsp trails)
//   T07: adaptive — a single requester wins EVERY cycle (no idle slots)
//   T08: adaptive — two requesters alternate strictly (fairness)
//   T09: adaptive — all four requesters rotate 0,1,2,3,0,... (matches the
//        free-running policy exactly when everyone is busy)
//   T10: adaptive — search starts PAST the last winner: after 1 wins, a
//        newly-arrived 2 beats a still-waiting 0
//   T11: adaptive — a requester dropping out mid-rotation is skipped, the
//        remaining ones keep alternating
//   T12: adaptive — sel_rsp trails sel_req by one cycle through arbitrary
//        request patterns, including the -1 idle gaps
// -----------------------------------------------------------------------------

#include "arbiter.hpp"
#include "arbiter_adaptive.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

static constexpr int kNumAgu = 4;

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // ---- free-running arbiter ----
    arbiter<kNumAgu> rr{"rr"};
    sc_signal<int>   rr_req{"rr_req"}, rr_rsp{"rr_rsp"};

    // ---- adaptive arbiter ----
    arbiter_adaptive<kNumAgu> ad{"ad"};
    sc_signal<bool>           ad_req_i[kNumAgu];
    sc_signal<int>            ad_req{"ad_req"}, ad_rsp{"ad_rsp"};

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        rr.clk_i(clk);
        rr.rst_ni(rst_n);
        rr.sel_req_o(rr_req);
        rr.sel_rsp_o(rr_rsp);

        ad.clk_i(clk);
        ad.rst_ni(rst_n);
        for (int i = 0; i < kNumAgu; ++i)
            ad.req_i[i](ad_req_i[i]);
        ad.sel_req_o(ad_req);
        ad.sel_rsp_o(ad_rsp);

        SC_THREAD(run);
    }

    void set_reqs(bool r0, bool r1, bool r2, bool r3) {
        ad_req_i[0].write(r0);
        ad_req_i[1].write(r1);
        ad_req_i[2].write(r2);
        ad_req_i[3].write(r3);
    }

    void do_reset() {
        rst_n.write(false);
        set_reqs(false, false, false, false);
        tick(clk);
        tick(clk);
        rst_n.write(true);
    }

    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01-T04: free-running arbiter ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(rr_req.read() == 0 && rr_rsp.read() == 0, "T01 reset holds sel_req=sel_rsp=0");

        // First live edge outputs 0, then increments each cycle.
        bool cycle_ok = true, trail_ok = true;
        int  prev = -1;
        for (int c = 0; c < 2 * kNumAgu + 1; ++c) {
            tick(clk);
            const int expect = c % kNumAgu;
            cycle_ok &= (rr_req.read() == expect);
            if (c > 0)
                trail_ok &= (rr_rsp.read() == prev);
            prev = rr_req.read();
        }
        CHECK(cycle_ok, "T02 sel_req cycles 0..3 wrapping, one step per cycle");
        CHECK(trail_ok, "T03 sel_rsp trails sel_req by exactly one cycle");

        rst_n.write(false);
        tick(clk);
        CHECK(rr_req.read() == 0 && rr_rsp.read() == 0, "T04a mid-run reset re-zeros outputs");
        rst_n.write(true);
        tick(clk);
        CHECK(rr_req.read() == 0, "T04b restart begins the cycle at 0 again");

        // -------------------------------------------------------------------
        std::puts("\n=== T05-T06: adaptive — reset & idle ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(ad_req.read() == -1 && ad_rsp.read() == -1, "T05 reset holds sel_req=sel_rsp=-1");
        bool idle_ok = true;
        for (int c = 0; c < 4; ++c) {
            tick(clk);
            idle_ok &= (ad_req.read() == -1);
        }
        CHECK(idle_ok, "T06 no requesters -> sel_req stays -1");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: adaptive — lone requester wins every cycle ===");
        // -------------------------------------------------------------------
        set_reqs(false, false, true, false); // only index 2
        bool lone_ok = true;
        for (int c = 0; c < 5; ++c) {
            tick(clk);
            lone_ok &= (ad_req.read() == 2);
        }
        CHECK(lone_ok, "T07 index 2 granted back-to-back, zero idle slots");

        // -------------------------------------------------------------------
        std::puts("\n=== T08: adaptive — two requesters alternate ===");
        // -------------------------------------------------------------------
        set_reqs(true, false, false, true); // 0 and 3
        tick(clk);
        const int first = ad_req.read();
        CHECK(first == 0 || first == 3, "T08a one of the two active wins first");
        bool alt_ok = true;
        int  last   = first;
        for (int c = 0; c < 6; ++c) {
            tick(clk);
            const int cur = ad_req.read();
            alt_ok &= (cur == 0 || cur == 3) && (cur != last);
            last = cur;
        }
        CHECK(alt_ok, "T08b strict 0/3 alternation — neither starves the other");

        // -------------------------------------------------------------------
        std::puts("\n=== T09: adaptive — all four rotate like the free-running one ===");
        // -------------------------------------------------------------------
        set_reqs(true, true, true, true);
        tick(clk);
        bool rot_ok = true;
        int  cur    = ad_req.read();
        for (int c = 0; c < 2 * kNumAgu; ++c) {
            tick(clk);
            const int nxt = ad_req.read();
            rot_ok &= (nxt == (cur + 1) % kNumAgu);
            cur = nxt;
        }
        CHECK(rot_ok, "T09 strict +1 rotation when everyone requests");

        // -------------------------------------------------------------------
        std::puts("\n=== T10: adaptive — search starts past the last winner ===");
        // -------------------------------------------------------------------
        // The grant is COMBINATIONAL (see arbiter_adaptive.hpp): a request is
        // granted the same cycle it is raised, and the fairness pointer moves
        // past the winner at the edge — so each cycle's winner is sampled
        // BEFORE its tick, not after.
        do_reset();
        set_reqs(false, true, false, false); // only 1
        wait(1, SC_NS);
        CHECK(ad_req.read() == 1, "T10a index 1 wins the same cycle it requests");
        tick(clk);                          // rr_ptr moves past 1
        set_reqs(true, false, true, false); // 0 and 2 both arrive
        wait(1, SC_NS);
        CHECK(ad_req.read() == 2, "T10b newly-arrived 2 beats still-waiting 0 (fair rotation)");
        tick(clk);
        wait(1, SC_NS);
        CHECK(ad_req.read() == 0, "T10c then 0 gets its turn");
        tick(clk);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: adaptive — dropout is skipped cleanly ===");
        // -------------------------------------------------------------------
        set_reqs(true, true, true, false);
        tick(clk);
        tick(clk);
        set_reqs(true, false, true, false); // 1 drops out
        bool skip_ok = true;
        for (int c = 0; c < 4; ++c) {
            tick(clk);
            const int g = ad_req.read();
            skip_ok &= (g == 0 || g == 2);
        }
        CHECK(skip_ok, "T11 dropped-out index never granted; the rest keep rotating");

        // -------------------------------------------------------------------
        std::puts("\n=== T12: adaptive — sel_rsp trails through arbitrary patterns ===");
        // -------------------------------------------------------------------
        do_reset();
        static const bool pat[][kNumAgu] = {
            {true, false, false, false}, {false, false, false, false}, {false, true, true, false},
            {true, true, true, true},    {false, false, false, true},  {false, false, false, false},
        };
        bool rsp_ok = true;
        prev        = -1; // reset value of sel_rsp
        for (const auto &p : pat) {
            set_reqs(p[0], p[1], p[2], p[3]);
            wait(1, SC_NS); // combinational grant settles within the cycle
            rsp_ok &= (ad_rsp.read() == prev);
            prev = ad_req.read();
            tick(clk);
        }
        CHECK(rsp_ok, "T12 sel_rsp(T) == sel_req(T-1), idle -1 gaps included");

        // -------------------------------------------------------------------
        std::puts("\n=== T13: adaptive — 3-active rotation skips the idle slot ===");
        // -------------------------------------------------------------------
        do_reset();
        set_reqs(true, false, true, true); // 1 idle
        {
            bool      rot       = true;
            const int expect[6] = {0, 2, 3, 0, 2, 3};
            for (int c = 0; c < 6; ++c) {
                wait(1, SC_NS);
                rot &= (ad_req.read() == expect[c]);
                tick(clk);
            }
            CHECK(rot, "T13 rotation is 0,2,3,0,2,3 — slot 1 never granted");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T14: adaptive — requester toggling every cycle ===");
        // -------------------------------------------------------------------
        do_reset();
        {
            bool ok = true;
            // 3 requests only on even cycles; 1 holds. Odd cycles must grant
            // 1 (the only requester); even cycles rotate fairly among {1,3}.
            for (int c = 0; c < 8; ++c) {
                const bool even = (c % 2) == 0;
                set_reqs(false, true, false, even);
                tick(clk);
                const int sel = ad_req.read();
                if (even) {
                    if (sel != 1 && sel != 3)
                        ok = false;
                } else if (sel != 1) {
                    ok = false;
                }
            }
            CHECK(ok, "T14a a toggling requester never steals an idle cycle");
            // and 3 was actually served on some even cycle (fairness held):
            set_reqs(false, true, false, true);
            bool saw3 = false;
            for (int c = 0; c < 4; ++c) {
                tick(clk);
                saw3 |= (ad_req.read() == 3);
            }
            CHECK(saw3, "T14b the toggler is served once it holds its request");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T15: adaptive — burst hand-off between two singletons ===");
        // -------------------------------------------------------------------
        do_reset();
        set_reqs(true, false, false, false);
        {
            bool solo0 = true;
            for (int c = 0; c < 3; ++c) {
                tick(clk);
                solo0 &= (ad_req.read() == 0);
            }
            CHECK(solo0, "T15a lone requester 0 wins every cycle");
            set_reqs(false, false, true, false); // instant hand-off
            bool solo2 = true;
            for (int c = 0; c < 3; ++c) {
                tick(clk);
                solo2 &= (ad_req.read() == 2);
            }
            CHECK(solo2, "T15b hand-off to the new lone requester with no dead cycle");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T16: adaptive — all-to-none-to-all recovery ===");
        // -------------------------------------------------------------------
        do_reset();
        set_reqs(true, true, true, true);
        tick(clk);
        tick(clk); // grants 0, then 1
        set_reqs(false, false, false, false);
        tick(clk);
        CHECK(ad_req.read() == -1, "T16a all requests dropped -> sel_req=-1");
        tick(clk);
        CHECK(ad_rsp.read() == -1, "T16b sel_rsp trails to -1 one cycle later");
        set_reqs(true, true, true, true);
        wait(1, SC_NS);
        CHECK(ad_req.read() == 2, "T16c resume continues the rotation past the last winner");
        tick(clk);

        // -------------------------------------------------------------------
        std::puts("\n=== T17: free-running arbiter — sequence resumes after mid-run reset ===");
        // -------------------------------------------------------------------
        do_reset();
        tick(clk);
        tick(clk);
        tick(clk); // rr_req = 2 now
        rst_n.write(false);
        tick(clk);
        CHECK(rr_req.read() == 0 && rr_rsp.read() == 0, "T17a mid-run reset re-pins both to 0");
        rst_n.write(true);
        {
            bool seq = true;
            for (int c = 0; c < kNumAgu; ++c) {
                tick(clk);
                seq &= (rr_req.read() == c % kNumAgu);
            }
            CHECK(seq, "T17b sequence restarts 0,1,2,3 after reset release");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T18: adaptive — one-cycle request pulse is served exactly once ===");
        // -------------------------------------------------------------------
        do_reset();
        set_reqs(false, false, true, false);
        tick(clk);
        CHECK(ad_req.read() == 2, "T18a the pulse cycle grants the requester");
        set_reqs(false, false, false, false);
        {
            bool gone = true;
            for (int c = 0; c < 3; ++c) {
                tick(clk);
                gone &= (ad_req.read() == -1);
            }
            CHECK(gone, "T18b no residual grants after the pulse");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T19: free-running — sel_rsp is always sel_req minus one (mod N) ===");
        // -------------------------------------------------------------------
        do_reset();
        tick(clk);
        {
            bool rel = true;
            for (int c = 0; c < 2 * kNumAgu; ++c) {
                const int before = rr_req.read();
                tick(clk);
                rel &= (rr_rsp.read() == before);
            }
            CHECK(rel, "T19 sel_rsp(T) == sel_req(T-1) across two full rotations");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T20: free-running — programmable slot table (set_sequence) ===");
        // -------------------------------------------------------------------
        // Drop client 1 from the rotation (the "skip an unused buffer" use
        // case): 3-entry table over a 4-client arbiter.
        {
            static const int seq3[] = {0, 2, 3};
            rr.set_sequence(seq3, 3);
            do_reset();
            CHECK(rr_req.read() == 0 && rr_rsp.read() == 0,
                  "T20a reset holds both outputs at seq[0]");
            bool walk = true, trail = true;
            int  prev = -1;
            for (int c = 0; c < 7; ++c) {
                tick(clk);
                walk &= (rr_req.read() == seq3[c % 3]);
                if (c > 0)
                    trail &= (rr_rsp.read() == prev);
                prev = rr_req.read();
            }
            CHECK(walk, "T20b rotation walks 0,2,3,0,... — client 1 never granted");
            CHECK(trail, "T20c sel_rsp still trails sel_req by one cycle");
        }
        // Table starting at a nonzero client, with a repeated (weighted) slot.
        {
            static const int seqw[] = {3, 3, 1};
            rr.set_sequence(seqw, 3);
            do_reset();
            CHECK(rr_req.read() == 3 && rr_rsp.read() == 3,
                  "T20d reset re-pins outputs to the new seq[0]");
            bool walk = true;
            for (int c = 0; c < 6; ++c) {
                tick(clk);
                walk &= (rr_req.read() == seqw[c % 3]);
            }
            CHECK(walk, "T20e weighted table 3,3,1 repeats verbatim (client 3 gets 2 of 3 slots)");
        }
        // Restore the identity table so any later-added tests see the default.
        {
            static const int id4[] = {0, 1, 2, 3};
            rr.set_sequence(id4, 4);
        }

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
