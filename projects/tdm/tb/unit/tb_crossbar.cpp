// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for crossbar<NUM_IN, NUM_OUT, BYTES_PER_ROW, SEL_START, SEL_LEN> —
// the word-interleaved primitive with per-bank round-robin arbitration, tested
// standalone (until now it was only exercised through top_crossbar/top_tdm).
//
// Two instantiations:
//   A) crossbar<4,4,16> (SEL_LEN=0, beat-interleaved decode) wired to four
//      real bank<16,16> instances — end-to-end: decode, arbitration, owner
//      steering, and data all through the production response path.
//   B) crossbar<2,4,16,4,2> (SEL_LEN=2 bit-field routing, as top_crossbar's
//      L1/L2/L3 use it) with the bank side driven manually — verifies the
//      routing-field mode and full-address passthrough in isolation.
//
// Build and run:
//   edaf unit TOP=crossbar   (PROJECT=tdm)
//
// Tests:
//   T01  Reset: all manager gnt/rvalid low, all bank req low
//   T02  Beat-interleaved decode: (bank,row) address lands on bank b with the
//        routing bits stripped (bank-local addr = row*BYTES_PER_ROW)
//   T03  gnt is combinational end to end (bank gnt=req, no clock edge needed)
//   T04  Write-then-read round trip through the crossbar, per manager
//   T05  Response steering: manager m reads bank b -> rvalid returns to m
//        only, one cycle later, with that bank's data
//   T06  Conflict-free concurrency: 4 managers to 4 distinct banks in one
//        cycle -> all granted together, all responses next cycle
//   T07  2-way same-bank conflict: exactly one grant per cycle, round-robin
//        order (0 then 1), both data correct
//   T08  4-way same-bank conflict: serialized over 4 cycles, each manager
//        granted exactly once, in rr order 0,1,2,3
//   T09  rr pointer advances past the winner: after manager 1 wins, a 0-vs-2
//        contest is won by 2 (search starts at 2)
//   T10  Loser is not granted while losing (no spurious gnt), and is granted
//        once the winner's request drops
//   T11  we/be/wdata passthrough: byte-enable partial write through the
//        crossbar modifies only the enabled bytes
//   T12  Back-to-back streaming: one manager, one bank, 4 consecutive reads
//        -> gnt every cycle, rvalid pipelined 1/cycle behind
//        (grant/return overlap during a conflict is pinned by T07c: manager
//        0's response returns on the same cycle manager 1 is being granted)
//   T14  SEL_LEN>0: routes by addr[SEL_START +: SEL_LEN], and the full
//        address passes through to the bank UNCHANGED (no strip)
//   T16  SEL_LEN>0: bank-side gnt passes back combinationally; rvalid/rdata
//        steered to the requesting manager one cycle later
//   T17  Crossing pair (m0->b1, m1->b0): both granted, responses not swapped
//   T18  Two independent 2-way conflicts on two banks arbitrate in parallel
//   T19  An idle bank's rr pointer stays frozen (no phantom advance)
//   T20  A spurious rvalid on an owner-less bank reaches no manager
//   T21  Back-to-back write streaming: gnt every cycle, all rows land
// -----------------------------------------------------------------------------

#include "bank.hpp"
#include "crossbar.hpp"
#include "obi_ports.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

static constexpr int kNumIn    = 4;
static constexpr int kNumOut   = 4;
static constexpr int kBytesRow = 16;
static constexpr int kNumRows  = 16;

using xbar_t  = crossbar<kNumIn, kNumOut, kBytesRow>;
using xbarf_t = crossbar<2, kNumOut, kBytesRow, 4, 2>; // routing field addr[5:4]
using bank_t  = bank<kNumRows, kBytesRow>;
using data_t  = xbar_t::data_t;

// (bank, row) -> beat-interleaved byte address for instantiation A
static uint64_t mk_addr(int b, int r) {
    return static_cast<uint64_t>(r * kNumOut + b) * kBytesRow;
}

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // ---- instantiation A: crossbar + real banks ----
    xbar_t                   *xb;
    bank_t                   *banks[kNumOut];
    obi_signal_bundle<data_t> mgr[kNumIn];  // tb -> crossbar manager side
    obi_signal_bundle<data_t> bnk[kNumOut]; // crossbar -> banks

    // ---- instantiation B: field-routed crossbar, bank side hand-driven ----
    xbarf_t                  *xf;
    obi_signal_bundle<data_t> fmgr[2];
    obi_signal_bundle<data_t> fbnk[kNumOut];

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        xb = new xbar_t("xb");
        xb->clk_i(clk);
        xb->rst_ni(rst_n);
        for (int m = 0; m < kNumIn; ++m)
            bind_obi(xb->m_ports[m], mgr[m]);
        for (int b = 0; b < kNumOut; ++b) {
            bind_obi(xb->b_ports[b], bnk[b]);
            char nmb[16];
            std::snprintf(nmb, sizeof(nmb), "bank%d", b);
            banks[b] = new bank_t(nmb);
            banks[b]->clk_i(clk);
            banks[b]->rst_ni(rst_n);
            bind_obi(banks[b]->obi, bnk[b]);
        }

        xf = new xbarf_t("xf");
        xf->clk_i(clk);
        xf->rst_ni(rst_n);
        for (int m = 0; m < 2; ++m)
            bind_obi(xf->m_ports[m], fmgr[m]);
        for (int b = 0; b < kNumOut; ++b)
            bind_obi(xf->b_ports[b], fbnk[b]);

        SC_THREAD(run);
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    void idle_mgr(obi_signal_bundle<data_t> & g) {
        g.req.write(false);
        g.addr.write(0);
        g.we.write(false);
        g.be.write(0);
        g.wdata.write(data_t(0));
    }

    void idle_all() {
        for (int m = 0; m < kNumIn; ++m)
            idle_mgr(mgr[m]);
        for (int m = 0; m < 2; ++m)
            idle_mgr(fmgr[m]);
        for (int b = 0; b < kNumOut; ++b) {
            fbnk[b].gnt.write(false);
            fbnk[b].rvalid.write(false);
            fbnk[b].rdata.write(data_t(0));
        }
    }

    void do_reset() {
        idle_all();
        rst_n.write(false);
        tick(clk);
        tick(clk);
        rst_n.write(true);
        tick(clk);
    }

    void drive_read(int m, uint64_t addr) {
        mgr[m].req.write(true);
        mgr[m].we.write(false);
        mgr[m].addr.write(addr);
        mgr[m].be.write(0xF);
    }

    void drive_write(int m, uint64_t addr, uint64_t data, uint32_t be = 0xF) {
        mgr[m].req.write(true);
        mgr[m].we.write(true);
        mgr[m].addr.write(addr);
        mgr[m].be.write(be);
        mgr[m].wdata.write(data_t(static_cast<unsigned long long>(data)));
    }

    // Single-manager write/read through the crossbar (assumes no contention:
    // gnt is immediate, response one tick later).
    void xwrite(int m, uint64_t addr, uint64_t data, uint32_t be = 0xF) {
        drive_write(m, addr, data, be);
        tick(clk);
        idle_mgr(mgr[m]);
    }

    uint64_t xread(int m, uint64_t addr) {
        drive_read(m, addr);
        tick(clk);
        idle_mgr(mgr[m]);
        const uint64_t v = mgr[m].rdata.read().to_uint64();
        return v;
    }

    // -----------------------------------------------------------------------
    void run() {
        do_reset();

        // -------------------------------------------------------------------
        std::printf("\n=== T01: Reset — all outputs quiet ===\n");
        rst_n.write(false);
        tick(clk);
        {
            bool quiet = true;
            for (int m = 0; m < kNumIn; ++m)
                quiet = quiet && !mgr[m].gnt.read() && !mgr[m].rvalid.read();
            for (int b = 0; b < kNumOut; ++b)
                quiet = quiet && !bnk[b].req.read();
            CHECK(quiet, "T01 all manager gnt/rvalid and bank req low in reset");
        }
        rst_n.write(true);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T02: beat-interleaved decode + routing bits stripped ===\n");
        {
            bool decode_ok = true, strip_ok = true;
            for (int b = 0; b < kNumOut && decode_ok && strip_ok; ++b) {
                const int r = b + 1; // arbitrary distinct row per bank
                drive_read(0, mk_addr(b, r));
                wait(1, SC_NS);
                for (int ob = 0; ob < kNumOut; ++ob) {
                    const bool expect = (ob == b);
                    if (bnk[ob].req.read() != expect)
                        decode_ok = false;
                }
                if (bnk[b].addr.read() != static_cast<uint64_t>(r) * kBytesRow)
                    strip_ok = false;
                idle_mgr(mgr[0]);
                tick(clk);
            }
            CHECK(decode_ok, "T02a each address requests exactly its decoded bank");
            CHECK(strip_ok, "T02b bank sees the bank-local address (row*BYTES_PER_ROW)");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T03: gnt combinational through crossbar + bank ===\n");
        drive_read(2, mk_addr(1, 0));
        wait(1, SC_NS);
        CHECK(mgr[2].gnt.read(), "T03 gnt reaches the manager without a clock edge");
        idle_mgr(mgr[2]);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T04: write-then-read round trip per manager ===\n");
        {
            bool ok = true;
            for (int m = 0; m < kNumIn; ++m) {
                const uint64_t a = mk_addr(m, 2 + m); // distinct (bank,row) per manager
                xwrite(m, a, 0x1000 + m);
                if (xread(m, a) != static_cast<uint64_t>(0x1000 + m))
                    ok = false;
            }
            CHECK(ok, "T04 every manager round-trips its own data");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T05: response steered to the requesting manager only ===\n");
        xwrite(0, mk_addr(1, 5), 0xBEEF);
        drive_read(2, mk_addr(1, 5));
        tick(clk); // grant edge; response next cycle
        idle_mgr(mgr[2]);
        {
            bool only2 = mgr[2].rvalid.read();
            for (int m = 0; m < kNumIn; ++m)
                if (m != 2 && mgr[m].rvalid.read())
                    only2 = false;
            CHECK(only2, "T05a rvalid returns to manager 2 and nobody else");
            CHECK(mgr[2].rdata.read().to_uint64() == 0xBEEF, "T05b steered data correct");
        }
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T06: 4 managers, 4 distinct banks, one cycle ===\n");
        {
            for (int m = 0; m < kNumIn; ++m)
                xwrite(m, mk_addr(m, 7), 0x2000 + m);
            for (int m = 0; m < kNumIn; ++m)
                drive_read(m, mk_addr(m, 7)); // manager m -> bank m: no conflicts
            wait(1, SC_NS);
            bool all_gnt = true;
            for (int m = 0; m < kNumIn; ++m)
                all_gnt = all_gnt && mgr[m].gnt.read();
            CHECK(all_gnt, "T06a all four granted in the same cycle");
            tick(clk);
            for (int m = 0; m < kNumIn; ++m)
                idle_mgr(mgr[m]);
            bool all_rsp = true;
            for (int m = 0; m < kNumIn; ++m)
                all_rsp = all_rsp && mgr[m].rvalid.read() &&
                          mgr[m].rdata.read().to_uint64() == static_cast<uint64_t>(0x2000 + m);
            CHECK(all_rsp, "T06b all four responses land together, each with its own data");
            tick(clk);
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T07: 2-way same-bank conflict — rr serializes 0 then 1 ===\n");
        xwrite(0, mk_addr(2, 8), 0xAA);
        xwrite(0, mk_addr(2, 9), 0xBB);
        do_reset(); // AFTER the setup writes: pins rr_ptr back to 0 (bank
                    // memory survives reset), so the order assertion is exact
        drive_read(0, mk_addr(2, 8));
        drive_read(1, mk_addr(2, 9));
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read() && !mgr[1].gnt.read(), "T07a cycle 1: manager 0 wins, 1 waits");
        tick(clk);
        idle_mgr(mgr[0]); // winner drops; loser keeps requesting
        wait(1, SC_NS);
        CHECK(mgr[1].gnt.read(), "T07b cycle 2: manager 1 wins");
        CHECK(mgr[0].rvalid.read() && mgr[0].rdata.read().to_uint64() == 0xAA,
              "T07c manager 0's data returns while 1 is being granted");
        tick(clk);
        idle_mgr(mgr[1]);
        CHECK(mgr[1].rvalid.read() && mgr[1].rdata.read().to_uint64() == 0xBB,
              "T07d manager 1's data returns one cycle later");
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T08: 4-way same-bank conflict — serialized in rr order ===\n");
        for (int m = 0; m < kNumIn; ++m)
            xwrite(0, mk_addr(3, 8 + m), 0x30 + m);
        do_reset(); // re-pin rr_ptr after the setup writes
        for (int m = 0; m < kNumIn; ++m)
            drive_read(m, mk_addr(3, 8 + m));
        {
            bool order_ok = true, once_ok = true;
            int  grants[kNumIn] = {0, 0, 0, 0};
            for (int c = 0; c < kNumIn; ++c) {
                wait(1, SC_NS);
                int winner = -1, n_gnt = 0;
                for (int m = 0; m < kNumIn; ++m)
                    if (mgr[m].gnt.read()) {
                        winner = m;
                        ++n_gnt;
                    }
                if (n_gnt != 1 || winner != c)
                    order_ok = false;
                if (winner >= 0) {
                    ++grants[winner];
                    tick(clk);
                    idle_mgr(mgr[winner]);
                } else {
                    tick(clk);
                }
            }
            for (int m = 0; m < kNumIn; ++m)
                once_ok = once_ok && grants[m] == 1;
            CHECK(order_ok, "T08a exactly one grant per cycle, in rr order 0,1,2,3");
            CHECK(once_ok, "T08b every manager granted exactly once");
            tick(clk);
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T09: rr pointer starts past the last winner ===\n");
        do_reset();
        // Make manager 1 the last winner on bank 0.
        drive_read(1, mk_addr(0, 3));
        tick(clk);
        idle_mgr(mgr[1]);
        tick(clk); // absorb the response
        // Now 0 and 2 contend for bank 0: search starts at 2 -> 2 wins first.
        drive_read(0, mk_addr(0, 4));
        drive_read(2, mk_addr(0, 5));
        wait(1, SC_NS);
        CHECK(mgr[2].gnt.read() && !mgr[0].gnt.read(),
              "T09 search resumes past the last winner: 2 beats 0");
        tick(clk);
        idle_mgr(mgr[2]);
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read(), "T09b manager 0 wins the following cycle");
        tick(clk);
        idle_mgr(mgr[0]);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T10: loser never granted while losing ===\n");
        do_reset();
        drive_read(0, mk_addr(1, 1));
        drive_read(3, mk_addr(1, 2));
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read() && !mgr[3].gnt.read(), "T10a winner 0 granted, loser 3 not");
        // Winner holds its request an extra cycle (re-requests): loser must
        // win the second cycle anyway - rr passed 0.
        tick(clk);
        wait(1, SC_NS);
        CHECK(mgr[3].gnt.read() && !mgr[0].gnt.read(),
              "T10b rr prevents starvation: 3 wins even though 0 still requests");
        tick(clk);
        idle_mgr(mgr[0]);
        idle_mgr(mgr[3]);
        tick(clk);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T11: byte-enable partial write through the crossbar ===\n");
        {
            const uint64_t a = mk_addr(2, 12);
            xwrite(0, a, 0x11223344, 0xF);
            xwrite(0, a, 0x000000AA, 0x1); // only byte 0
            const uint64_t v = xread(0, a);
            CHECK(v == 0x112233AA, "T11 only the enabled byte changed");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T12: back-to-back streaming, gnt every cycle ===\n");
        {
            for (int r = 0; r < 4; ++r)
                xwrite(1, mk_addr(0, 10 + r), 0x40 + r);
            bool gnt_ok = true, data_ok = true;
            int  seen = 0;
            for (int r = 0; r < 4; ++r) {
                drive_read(1, mk_addr(0, 10 + r));
                wait(1, SC_NS);
                if (!mgr[1].gnt.read())
                    gnt_ok = false;
                tick(clk);
                // read r's response is visible right after its accepting edge
                // (bank rvalid registers at that edge — same point T05/T06
                // sample), overlapping the NEXT read's request cycle
                if (mgr[1].rvalid.read()) {
                    if (mgr[1].rdata.read().to_uint64() != static_cast<uint64_t>(0x40 + r))
                        data_ok = false;
                    ++seen;
                }
            }
            idle_mgr(mgr[1]);
            tick(clk);
            CHECK(gnt_ok, "T12a granted on every streaming cycle");
            CHECK(data_ok && seen == 4, "T12b four responses pipelined 1/cycle, in order");
        }

        // ===================================================================
        // Instantiation B: SEL_LEN=2 field routing (addr[5:4]), manual banks
        // ===================================================================
        std::printf("\n=== T14: SEL_LEN>0 routes by addr[5:4] ===\n");
        {
            bool route_ok = true;
            for (int f = 0; f < kNumOut; ++f) {
                const uint64_t a = 0xA00 | (static_cast<uint64_t>(f) << 4) | 0x3;
                fmgr[0].req.write(true);
                fmgr[0].addr.write(a);
                fmgr[0].be.write(0xF);
                wait(1, SC_NS);
                for (int b = 0; b < kNumOut; ++b)
                    if (fbnk[b].req.read() != (b == f))
                        route_ok = false;
                // T15 folded in: address must pass through unchanged
                if (fbnk[f].addr.read() != a)
                    route_ok = false;
                fmgr[0].req.write(false);
                tick(clk);
            }
            CHECK(route_ok, "T14+T15 field-routed to addr[5:4], full address passed through");
        }

        std::printf("\n=== T16: SEL_LEN>0 gnt passthrough + response steering ===\n");
        {
            const uint64_t a = (2ull << 4); // field 2
            fmgr[1].req.write(true);
            fmgr[1].addr.write(a);
            fmgr[1].be.write(0xF);
            fbnk[2].gnt.write(true);
            wait(1, SC_NS);
            CHECK(fmgr[1].gnt.read(), "T16a bank gnt reaches the manager combinationally");
            tick(clk);
            fmgr[1].req.write(false);
            fbnk[2].gnt.write(false);
            fbnk[2].rvalid.write(true);
            fbnk[2].rdata.write(data_t(0xC0FFEEull));
            wait(1, SC_NS);
            CHECK(fmgr[1].rvalid.read() && !fmgr[0].rvalid.read() &&
                      fmgr[1].rdata.read().to_uint64() == 0xC0FFEE,
                  "T16b response steered to the owning manager with its data");
            fbnk[2].rvalid.write(false);
            tick(clk);
        }

        // ===================================================================
        // Back on instantiation A
        // ===================================================================
        std::printf("\n=== T17: crossing pair — m0->b1 and m1->b0 in one cycle ===\n");
        do_reset();
        xwrite(0, mk_addr(1, 3), 0xA1);
        xwrite(1, mk_addr(0, 3), 0xB0);
        drive_read(0, mk_addr(1, 3));
        drive_read(1, mk_addr(0, 3));
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read() && mgr[1].gnt.read(),
              "T17a both granted simultaneously (no false conflict on crossed routes)");
        tick(clk);
        idle_mgr(mgr[0]);
        idle_mgr(mgr[1]);
        CHECK(mgr[0].rvalid.read() && mgr[0].rdata.read().to_uint64() == 0xA1 &&
                  mgr[1].rvalid.read() && mgr[1].rdata.read().to_uint64() == 0xB0,
              "T17b crossed responses steered back without swapping");
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T18: two independent 2-way conflicts on two banks ===\n");
        do_reset();
        xwrite(0, mk_addr(0, 4), 0x18A);
        xwrite(0, mk_addr(0, 5), 0x18B);
        xwrite(0, mk_addr(1, 4), 0x18C);
        xwrite(0, mk_addr(1, 5), 0x18D);
        do_reset(); // re-pin rr after setup
        drive_read(0, mk_addr(0, 4));
        drive_read(1, mk_addr(0, 5)); // conflict pair on bank 0
        drive_read(2, mk_addr(1, 4));
        drive_read(3, mk_addr(1, 5)); // conflict pair on bank 1
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read() && mgr[2].gnt.read() && !mgr[1].gnt.read() && !mgr[3].gnt.read(),
              "T18a both banks arbitrate independently and in parallel (0 and 2 win)");
        tick(clk);
        idle_mgr(mgr[0]);
        idle_mgr(mgr[2]);
        wait(1, SC_NS);
        CHECK(mgr[1].gnt.read() && mgr[3].gnt.read(),
              "T18b both losers win their banks the next cycle");
        CHECK(mgr[0].rvalid.read() && mgr[0].rdata.read().to_uint64() == 0x18A &&
                  mgr[2].rvalid.read() && mgr[2].rdata.read().to_uint64() == 0x18C,
              "T18c first winners' data returns while the losers are granted");
        tick(clk);
        idle_mgr(mgr[1]);
        idle_mgr(mgr[3]);
        CHECK(mgr[1].rvalid.read() && mgr[1].rdata.read().to_uint64() == 0x18B &&
                  mgr[3].rvalid.read() && mgr[3].rdata.read().to_uint64() == 0x18D,
              "T18d losers' data returns one cycle later");
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T19: idle bank does not advance its rr pointer ===\n");
        do_reset();
        // Cycle bank 1's arbiter twice via manager 1 (rr_ptr[1] -> 2), while
        // bank 2 stays idle (rr_ptr[2] stays 0).
        drive_read(1, mk_addr(1, 6));
        tick(clk);
        idle_mgr(mgr[1]);
        tick(clk);
        // Contest bank 2 with managers 0 and 1: idle bank kept rr_ptr=0, so 0
        // must win — if idle cycles advanced the pointer this would flip.
        drive_read(0, mk_addr(2, 6));
        drive_read(1, mk_addr(2, 7));
        wait(1, SC_NS);
        CHECK(mgr[0].gnt.read() && !mgr[1].gnt.read(),
              "T19 idle bank's rr pointer is frozen (0 still wins bank 2)");
        tick(clk);
        idle_mgr(mgr[0]);
        tick(clk);
        idle_mgr(mgr[1]);
        tick(clk);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T20: spurious rvalid with no owner is dropped ===\n");
        // Use instantiation B (manual bank side): raise rvalid on a bank
        // whose owner register is -1 (nothing was granted there).
        for (int m2 = 0; m2 < 2; ++m2)
            idle_mgr(fmgr[m2]);
        tick(clk);
        tick(clk); // two idle edges: owner[3] = -1
        fbnk[3].rvalid.write(true);
        fbnk[3].rdata.write(data_t(0xBADull));
        wait(1, SC_NS);
        CHECK(!fmgr[0].rvalid.read() && !fmgr[1].rvalid.read(),
              "T20 ghost response reaches no manager");
        fbnk[3].rvalid.write(false);
        tick(clk);

        // -------------------------------------------------------------------
        std::printf("\n=== T21: back-to-back WRITE streaming through one bank ===\n");
        do_reset();
        {
            bool gnt_ok = true;
            for (int r = 0; r < 4; ++r) {
                drive_write(2, mk_addr(3, r), 0x210 + r);
                wait(1, SC_NS);
                if (!mgr[2].gnt.read())
                    gnt_ok = false;
                tick(clk);
            }
            idle_mgr(mgr[2]);
            CHECK(gnt_ok, "T21a write stream granted every cycle");
            bool data_ok = true;
            for (int r = 0; r < 4; ++r)
                if (xread(2, mk_addr(3, r)) != static_cast<uint64_t>(0x210 + r))
                    data_ok = false;
            CHECK(data_ok, "T21b all streamed writes landed in their rows");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T22: SEL_LEN>0 — 2-way conflict arbitrates in field mode too ===\n");
        {
            for (int m2 = 0; m2 < 2; ++m2)
                idle_mgr(fmgr[m2]);
            tick(clk);
            tick(clk);
            const uint64_t a0 = (1ull << 4) | 0x100; // field 1
            const uint64_t a1 = (1ull << 4) | 0x200; // field 1 — conflict
            fmgr[0].req.write(true);
            fmgr[0].addr.write(a0);
            fmgr[1].req.write(true);
            fmgr[1].addr.write(a1);
            fbnk[1].gnt.write(true);
            wait(1, SC_NS);
            CHECK(fmgr[0].gnt.read() && !fmgr[1].gnt.read(),
                  "T22a field-mode conflict: manager 0 wins first");
            CHECK(fbnk[1].addr.read() == a0, "T22b the winner's FULL address is presented");
            tick(clk);
            fmgr[0].req.write(false);
            wait(1, SC_NS);
            CHECK(fmgr[1].gnt.read() && fbnk[1].addr.read() == a1,
                  "T22c loser wins the next cycle with its own address");
            tick(clk);
            fmgr[1].req.write(false);
            fbnk[1].gnt.write(false);
            tick(clk);
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T23: WRITE conflict — payloads serialize uncorrupted ===\n");
        do_reset();
        drive_write(0, mk_addr(2, 14), 0x23A);
        drive_write(1, mk_addr(2, 15), 0x23B);
        tick(clk); // winner 0 accepted
        idle_mgr(mgr[0]);
        tick(clk); // winner 1 accepted
        idle_mgr(mgr[1]);
        tick(clk);
        CHECK(xread(0, mk_addr(2, 14)) == 0x23A && xread(0, mk_addr(2, 15)) == 0x23B,
              "T23 both conflicting writes landed with their own payloads");

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t("tb");
    sc_start();
    return report_and_exit();
}
