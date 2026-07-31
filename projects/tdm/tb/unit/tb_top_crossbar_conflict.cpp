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
//
// T02/T03 exercise the two poles of the L1 lane-select behavior on the READ
// plane, one port (4 lanes) issuing 8 group-synchronized frames of 4 beats
// (the AGU's compute-frame shape — a frame retries until all 4 lanes are
// granted, then the next frame issues):
//   - T02: the 32 consecutive row addresses 0x000..0x1F0. Below 0x200 the
//     address hash is a no-op and rows n, n+1, n+2, n+3 carry distinct
//     addr[5:4], so every frame spreads over 4 distinct L1 outputs (and all
//     32 beats land on 32 distinct banks): 4 grants/cycle, 1 cycle/frame,
//     zero wait — the conflict-free ideal.
//   - T03: the same 8 frames with a 0x200 lane stride (lane m at
//     base + m*0x200). The four lanes agree in addr[5:4] (0x200 only flips
//     bit 9), so every frame folds 4:1 onto ONE L1 output and serializes at
//     1 grant/cycle — 100% of frames conflicted, 4 cycles/frame, and 3 of 4
//     beats (75%) wait: the first-granted lane of each frame never does.
//     The banks are all distinct (bit 9 = L3 select), proving the collision
//     is purely the L1 lane-select field, not the memory.
//   - T04: the same 8 frames with a 0x100 lane stride — the hash-PROOF fold.
//     0x100 only flips bit 8, invisible to the raw select, and the four
//     lanes (base + 0..0x300) stay inside one 1 KiB block, so addr[11:10]
//     is identical too: XBAR_HASH_L1 cannot split this class (it is the
//     0x100-delta residual measured on the production traces, Appendix A.8
//     of the report). Full 4:1 fold under BOTH builds — the expectations
//     carry no #ifdef — while the L2 hash still spreads the four beats to
//     four distinct banks.
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

        run_l1_frames_test();

        sc_stop();
    }

    // Drives N_FRAMES group-synchronized read frames on port 0's 4 lanes
    // (frame f, lane m reads addrs[f][m]; a frame's losers keep re-requesting
    // until all 4 are granted, then the next frame issues — the AGU's
    // compute-frame behavior). Returns per-frame cycle counts plus the
    // waited-beat and max-grants-per-cycle tallies the T02/T03 checks assert.
    static constexpr int N_FRAMES = 8;
    struct frame_stats_t {
        int  frame_cycles[N_FRAMES] = {};
        int  total_cycles           = 0;
        int  waited_beats           = 0; // lanes that saw >=1 req&&!gnt cycle
        int  max_gnt_per_cycle      = 0;
        int  rv_count               = 0;
        bool timed_out              = false;
    };
    frame_stats_t drive_read_frames(const uint64_t (&addrs)[N_FRAMES][N_LANES]) {
        frame_stats_t st;
        for (int f = 0; f < N_FRAMES && !st.timed_out; ++f) {
            bool granted[N_LANES] = {};
            for (int m = 0; m < N_LANES; ++m) {
                rport[m].req.write(true);
                rport[m].addr.write(addrs[f][m]);
                rport[m].we.write(false);
                rport[m].be.write(FULL_BE);
            }
            bool lane_waited[N_LANES] = {};
            int  done                 = 0;
            while (done < N_LANES) {
                tick(clk);
                ++st.frame_cycles[f];
                ++st.total_cycles;
                int gnt_now = 0;
                for (int m = 0; m < N_LANES; ++m) {
                    if (!granted[m]) {
                        if (rport[m].gnt.read()) {
                            granted[m] = true;
                            rport[m].req.write(false);
                            ++gnt_now;
                            ++done;
                        } else {
                            lane_waited[m] = true;
                        }
                    }
                    if (rport[m].rvalid.read())
                        ++st.rv_count;
                }
                if (gnt_now > st.max_gnt_per_cycle)
                    st.max_gnt_per_cycle = gnt_now;
                if (st.frame_cycles[f] > 8 * N_LANES) {
                    st.timed_out = true;
                    break;
                }
            }
            for (int m = 0; m < N_LANES; ++m)
                if (lane_waited[m])
                    ++st.waited_beats;
        }
        // Drain the last frame's 1-cycle-latency responses.
        for (int d = 0; d < 2; ++d) {
            tick(clk);
            for (int m = 0; m < N_LANES; ++m)
                if (rport[m].rvalid.read())
                    ++st.rv_count;
        }
        for (int m = 0; m < N_LANES; ++m)
            idle_rport(m);
        return st;
    }

    void run_l1_frames_test() {
        char lbl[160];

        std::puts("=== T02: 32 consecutive rows — 4 distinct L1 selects/frame, no conflict ===");
        uint64_t seq[N_FRAMES][N_LANES];
        for (int f = 0; f < N_FRAMES; ++f)
            for (int m = 0; m < N_LANES; ++m)
                seq[f][m] = static_cast<uint64_t>(f * N_LANES + m) * 0x10; // rows 0..31
        frame_stats_t s2 = drive_read_frames(seq);

        std::snprintf(lbl, sizeof(lbl), "T02a all %d reads completed (took %d cycles)",
                      N_FRAMES * N_LANES, s2.total_cycles);
        CHECK(!s2.timed_out && s2.rv_count == N_FRAMES * N_LANES, lbl);
        bool one_cycle_frames = true;
        for (int f = 0; f < N_FRAMES; ++f)
            if (s2.frame_cycles[f] != 1)
                one_cycle_frames = false;
        CHECK(one_cycle_frames,
              "T02b every frame completes in 1 cycle (all 4 lanes granted together)");
        std::snprintf(lbl, sizeof(lbl), "T02c zero waited beats (%d) — conflict-free",
                      s2.waited_beats);
        CHECK(s2.waited_beats == 0, lbl);

        std::puts("=== T03: 0x200 lane stride — every frame folds 4:1 onto one L1 output ===");
        uint64_t strided[N_FRAMES][N_LANES];
        for (int f = 0; f < N_FRAMES; ++f)
            for (int m = 0; m < N_LANES; ++m)
                strided[f][m] = static_cast<uint64_t>(f) * 0x10 +
                                static_cast<uint64_t>(m) * 0x200; // same addr[5:4] per frame
        frame_stats_t s3 = drive_read_frames(strided);

        // Under the original hash the four lanes (bit 9 apart) share one L1
        // output: full 4:1 fold. With XBAR_HASH_L1 the select also takes
        // addr[11:10], which splits lanes 0/1 (bit 10 clear) from lanes 2/3
        // (bit 10 set) into a 2+2 fold — half the serialization, still 100%
        // of frames conflicted. XBAR_HASH_L1_V2 is deliberately NOT covered
        // here: this test instantiates top_crossbar<> directly and never
        // wires the R/C/napa1 port-group signals that macro's dynamic
        // dispatch (top_crossbar.hpp's hash_rd_addr()/hash_wr_addr()) reads,
        // so a build with it defined fails at elaboration ("port not
        // bound") before any test runs — a known, accepted limitation (see
        // top_crossbar.hpp's adaptive_group_hash()/adaptive_vector_axis_hash()
        // for where that macro's actual logic now lives).
#if defined(XBAR_HASH_L1)
        constexpr int kFold = 2; // lanes per L1 output (2+2)
#else
        constexpr int kFold = 4; // all four lanes on one output
#endif
        std::snprintf(lbl, sizeof(lbl), "T03a all %d reads completed (took %d cycles)",
                      N_FRAMES * N_LANES, s3.total_cycles);
        CHECK(!s3.timed_out && s3.rv_count == N_FRAMES * N_LANES, lbl);
        bool serialized_frames = true;
        for (int f = 0; f < N_FRAMES; ++f)
            if (s3.frame_cycles[f] != kFold)
                serialized_frames = false;
        std::snprintf(lbl, sizeof(lbl),
                      "T03b every frame takes exactly %d cycles (%d:1 fold per L1 output) — 100%% "
                      "of frames conflicted",
                      kFold, kFold);
        CHECK(serialized_frames, lbl);
        std::snprintf(lbl, sizeof(lbl),
                      "T03c never more than %d grants/cycle (L1 self-collision bounds the frame's "
                      "parallelism)",
                      N_LANES / kFold);
        CHECK(s3.max_gnt_per_cycle == N_LANES / kFold, lbl);
        std::snprintf(lbl, sizeof(lbl),
                      "T03d %d of %d beats waited (first-granted lane per L1 output never waits)",
                      s3.waited_beats, N_FRAMES * N_LANES);
        CHECK(s3.waited_beats == N_FRAMES * (N_LANES - N_LANES / kFold), lbl);

        std::puts("=== T04: 0x100 lane stride — 4:1 L1 fold XBAR_HASH_L1 cannot repair ===");
        uint64_t s100[N_FRAMES][N_LANES];
        for (int f = 0; f < N_FRAMES; ++f)
            for (int m = 0; m < N_LANES; ++m)
                s100[f][m] = static_cast<uint64_t>(f) * 0x10 +
                             static_cast<uint64_t>(m) * 0x100; // bit 8 only: hash-blind
        frame_stats_t s4 = drive_read_frames(s100);

        // The four lanes share addr[5:4] AND addr[11:10], varying only in
        // addr[9:8] — XBAR_HASH_L1's single addr[11:10] fold term never
        // sees that bit range, so it's a full 4:1 fold with or without it.
        // (XBAR_HASH_L1_V2 would fully split this exact pattern via
        // l1_field_add()'s addr[9:8] term — see top_crossbar.hpp — but this
        // test can't build under that macro at all, see T03's comment, so
        // there's no build-dependent case to select here.)
        constexpr int kFold04 = 4; // all four lanes on one output
        std::snprintf(lbl, sizeof(lbl), "T04a all %d reads completed (took %d cycles)",
                      N_FRAMES * N_LANES, s4.total_cycles);
        CHECK(!s4.timed_out && s4.rv_count == N_FRAMES * N_LANES, lbl);
        bool full_fold = true;
        for (int f = 0; f < N_FRAMES; ++f)
            if (s4.frame_cycles[f] != kFold04)
                full_fold = false;
        std::snprintf(lbl, sizeof(lbl),
                      "T04b every frame takes exactly %d cycle(s) (%d:1 fold per L1 output)",
                      kFold04, kFold04);
        CHECK(full_fold, lbl);
        std::snprintf(lbl, sizeof(lbl), "T04c never more than %d grant(s)/cycle",
                      N_LANES / kFold04);
        CHECK(s4.max_gnt_per_cycle == N_LANES / kFold04, lbl);
        std::snprintf(lbl, sizeof(lbl), "T04d %d of %d beats waited (%s)", s4.waited_beats,
                      N_FRAMES * N_LANES,
                      kFold04 == 1 ? "zero — fully repaired"
                                   : "75% — the hash-proof residual class");
        CHECK(s4.waited_beats == N_FRAMES * (N_LANES - N_LANES / kFold04), lbl);
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
