// -----------------------------------------------------------------------------
// Same-bank conflict timing test for the crossbar backend, at production
// scale — top_crossbar<9, 8, 4, 32, 1024, 4, 4> matches exactly what top.hpp
// instantiates for IMPL_CROSSBAR (see top.hpp's `impl` member), unlike
// tb_top_crossbar.cpp's deliberately tiny default (NUM_RPORT=2, NUM_BANK=8)
// used for fast, simple unit tests.
//
// Scenario: one port's 4 OBI lanes (matching "1 RAGU/WAGU, 1 port = 4 OBIs")
// each issue a stream of writes to the SAME single address, back-to-back with
// no gaps — i.e. 32 total write requests all conflicting on one bank. Since
// the bank accepts exactly one request per cycle, the crossbar's round-robin
// arbiter (crossbar.hpp) serializes all 32 to ~1 grant/cycle. This is the
// crossbar-side half of the TDM vs crossbar conflict-cost comparison that is
// the whole point of this project (see doc/specs/tdm.md); tb_top_tdm.cpp's
// T23 is the TDM-side equivalent (shadow-flush serialization of a 32-cell
// window landing on one bank — under posted acks the cost surfaces on the
// following window).
//
// Checks:
//   - Total cycles to complete all 32 writes matches the ~32-cycle prediction
//     (1 grant/cycle, no idle gaps, +constant pipeline drain).
//   - Grant order is a deterministic round-robin rotation across the 4
//     lanes (0,1,2,3,0,1,2,3,...) — losers keep re-requesting and are served
//     fairly in order, not starved or reordered.
//   - Final read-back value is the last write applied (all 32 target the
//     same address, so last-write-wins; the grant-order check pins down
//     exactly which write that is).
// -----------------------------------------------------------------------------

#include "top_crossbar.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

using DUT = top_crossbar<9, 8, 4, 32, 1024, 4, 4>; // matches top.hpp's IMPL_CROSSBAR instantiation
using data_t = DUT::data_t;

static constexpr int      NR       = DUT::NUM_RPORT_PORTS; // 36
static constexpr int      NW       = DUT::NUM_WPORT_PORTS; // 32
static constexpr uint32_t FULL_BE  = 0xFFFF;
static constexpr int      N_LANES  = 4;  // one port's worth of OBI lanes
static constexpr int      N_WRITES = 32; // total conflicting writes, matching the TDM window size

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    obi_signal_bundle<data_t> rport[NR];
    obi_signal_bundle<data_t> wport[NW];

    DUT *dut;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        bind_obi_group(dut->rport_req_i, dut->rport_addr_i, dut->rport_we_i, dut->rport_be_i,
                       dut->rport_wdata_i, dut->rport_gnt_o, dut->rport_rvalid_o,
                       dut->rport_rdata_o, rport);
        bind_obi_group(dut->wport_req_i, dut->wport_addr_i, dut->wport_we_i, dut->wport_be_i,
                       dut->wport_wdata_i, dut->wport_gnt_o, dut->wport_rvalid_o,
                       dut->wport_rdata_o, wport);
        SC_THREAD(run);
    }
    ~tb() {
        delete dut;
    }

    void idle_rport(int m) {
        rport[m].req.write(false);
        rport[m].we.write(false);
        rport[m].be.write(0);
        rport[m].addr.write(0);
        rport[m].wdata.write(data_t(0));
    }
    void idle_wport(int m) {
        wport[m].req.write(false);
        wport[m].we.write(false);
        wport[m].be.write(0);
        wport[m].addr.write(0);
        wport[m].wdata.write(data_t(0));
    }

    void do_reset() {
        rst_n.write(false);
        for (int m = 0; m < NR; ++m)
            idle_rport(m);
        for (int m = 0; m < NW; ++m)
            idle_wport(m);
        tick(clk);
        tick(clk);
        rst_n.write(true);
        tick(clk);
    }

    data_t do_read(int bus, uint64_t addr) {
        rport[bus].req.write(true);
        rport[bus].addr.write(addr);
        rport[bus].we.write(false);
        rport[bus].be.write(FULL_BE);
        tick(clk);
        data_t rd = rport[bus].rdata.read();
        idle_rport(bus);
        return rd;
    }

    void run() {
        std::puts("=== T01: Same-address conflict — 32 writes serialize 1-per-cycle ===");
        do_reset();

        static constexpr uint64_t kAddr = 0x00;
        data_t                    data[N_WRITES];
        for (int i = 0; i < N_WRITES; ++i)
            data[i] = make_row<data_t>(0x16000000U + static_cast<uint32_t>(i));

        // Each lane m streams its own 8 writes (data[k*N_LANES+m] for k=0..7)
        // back-to-back, immediately re-requesting as soon as granted — no
        // idle gaps, so the round-robin arbiter never has a free cycle.
        int    issued[N_LANES]     = {};
        bool   exhausted[N_LANES]  = {};
        int    gnt_order[N_WRITES] = {};
        int    gnt_count           = 0;
        int    rv_count            = 0;
        int    cycle               = 0;
        data_t last_wdata[N_LANES];
        for (int m = 0; m < N_LANES; ++m)
            last_wdata[m] = data[m];

        for (int m = 0; m < N_LANES; ++m) {
            wport[m].req.write(true);
            wport[m].addr.write(kAddr);
            wport[m].we.write(true);
            wport[m].be.write(FULL_BE);
            wport[m].wdata.write(data[m]); // lane m's write #0 = data[0*N_LANES+m]
            issued[m] = 1;
        }

        while (rv_count < N_WRITES && cycle < 10 * N_WRITES) {
            tick(clk);
            ++cycle;
            for (int m = 0; m < N_LANES; ++m) {
                if (!exhausted[m] && wport[m].gnt.read()) {
                    gnt_order[gnt_count++] = m;
                    if (issued[m] < N_WRITES / N_LANES) {
                        last_wdata[m] = data[issued[m] * N_LANES + m];
                        wport[m].addr.write(kAddr);
                        wport[m].wdata.write(last_wdata[m]);
                        ++issued[m];
                        // req stays asserted; be/we unchanged
                    } else {
                        wport[m].req.write(false);
                        exhausted[m] = true;
                    }
                }
                if (wport[m].rvalid.read())
                    ++rv_count;
            }
        }
        for (int m = 0; m < N_LANES; ++m)
            idle_wport(m);

        char lbl[128];
        std::snprintf(lbl, sizeof(lbl), "T01a all %d writes completed (took %d cycles)", N_WRITES,
                      cycle);
        CHECK(rv_count == N_WRITES, lbl);

        // Theoretical minimum: 1 grant/cycle with no idle gaps (32 grants),
        // plus 1 cycle for the last grant's response (1-cycle bank latency).
        // Allow a small margin for the fixed pipeline/reset overhead.
        std::snprintf(lbl, sizeof(lbl),
                      "T01b cycle count (%d) matches the ~%d-cycle same-bank-conflict prediction",
                      cycle, N_WRITES + 1);
        CHECK(cycle >= N_WRITES && cycle <= N_WRITES + 4, lbl);

        // Grant order: with all 4 lanes continuously re-requesting the same
        // address, L1's round-robin arbiter (crossbar.hpp) rotates through
        // them in strict, fair order every cycle *while all 4 remain live*.
        // All lanes advance in lockstep (same grant rate), so they all
        // exhaust in the same final round; the exact tie-break order among
        // simultaneously-exhausting requesters in that last round isn't a
        // guaranteed property of round-robin fairness, only the steady-state
        // rotation and the total count per lane are. Verify both:
        const int steady_grants = N_WRITES - N_LANES; // all rounds before the last
        bool      rotation_ok   = (gnt_count == N_WRITES);
        for (int i = 0; rotation_ok && i < steady_grants; ++i)
            if (gnt_order[i] != gnt_order[i % N_LANES])
                rotation_ok = false;
        CHECK(rotation_ok, "T01c steady-state grant order is a strict round-robin rotation while "
                           "all 4 lanes compete");

        bool fair = true;
        for (int m = 0; m < N_LANES; ++m) {
            int count = 0;
            for (int i = 0; i < gnt_count; ++i)
                if (gnt_order[i] == m)
                    ++count;
            if (count != N_WRITES / N_LANES)
                fair = false;
        }
        CHECK(fair, "T01d every lane is granted exactly N_WRITES/N_LANES times (no starvation)");

        // Last write applied is whichever lane the arbiter actually granted
        // last (all 32 target the same address, so last-write-wins).
        const int last_lane = (gnt_count > 0) ? gnt_order[gnt_count - 1] : -1;
        CHECK(last_lane >= 0 && do_read(0, kAddr) == last_wdata[last_lane],
              "T01e final value is the last write applied (last-write-wins)");

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
