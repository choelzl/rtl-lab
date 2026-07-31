// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer<> active_mode GEOMETRY: the mode-3 alias and the
// PORT_COUNT clamp. The main buffer suites are instantiated at PORT_COUNT=2,
// where ports_for_mode collapses (mode1/2/3 all give 2 ports) — so the
// mode2-vs-mode3 alias and the clamp-below-request paths were structurally
// untestable there and only integration-covered (top_tdm's PORT_COUNT=1 and
// PORT_COUNT=4 buffers). Two write-mode instances close that:
//
//   A) buffer<NUM_REQ=1, PORT_COUNT=4, 4, NUM_TDM=8, IS_WRITE=true>
//      NUM_IO=4, group width = ports_for_mode(m): mode0=1, mode1=2,
//      mode2=4, mode3=4 (the alias) — all four distinguishable.
//   B) buffer<NUM_REQ=4, PORT_COUNT=1, 4, NUM_TDM=8, IS_WRITE=true>
//      NUM_IO=4; every mode clamps to the 1 existing port -> 4 lanes.
//
// Write mode is used because its fill handshake exposes the group width
// directly on p_gnt_o with no lookahead/fetch plumbing; ports_for_mode is
// the same function both modes use.
//
// Tests:
//   T01  A/mode0: exactly lane 0 granted (1-lane groups)
//   T02  A/mode1: exactly lanes 0-1 granted, 2-3 never granted
//   T03  A/mode2: all four lanes granted together
//   T04  A/mode3: identical to mode2 (the documented alias) — grant span,
//        window fill count, and posted-ack group width all match
//   T05  A/mode2: full window end to end — 2 fills of 4, snapshot burst on
//        all 8 TDM lanes, acks stream back 4 lanes per cycle
//   T06  B/mode2 (asks for 4 ports, only 1 exists): clamped cleanly — all
//        4 lanes of the one port granted, full window completes, and the
//        behavior is identical to B/mode0 (clamp == floor at PORT_COUNT)
//   T07-T09  READ-mode twin (instance C): drain-group span per mode, exact
//        per-slot data, and the mode3 alias — the same geometry paths
//        exercised through the prefetch/drain protocol instead of the fill
//   T10  READ-mode clamp (instance D, 1 port): mode2 identical to mode0
// -----------------------------------------------------------------------------

#include "buffer.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

static constexpr int kBytes  = 4;
static constexpr int kNumTdm = 8;
static constexpr int kNumIO  = 4; // both instances: PORT_COUNT*NUM_REQ = 4

using data_t = obi_data<kBytes>;
using DUT_A  = buffer<1, 4, kBytes, kNumTdm, /*IS_WRITE=*/true>;  // 4 ports x 1 lane
using DUT_B  = buffer<4, 1, kBytes, kNumTdm, /*IS_WRITE=*/true>;  // 1 port  x 4 lanes
using DUT_C  = buffer<1, 4, kBytes, kNumTdm, /*IS_WRITE=*/false>; // read twin of A
using DUT_D  = buffer<4, 1, kBytes, kNumTdm, /*IS_WRITE=*/false>; // read twin of B

// One full set of harness signals for one DUT.
template <typename DUT> struct harness {
    sc_signal<bool>     rst_n;
    sc_signal<uint32_t> active_mode;
    // Port-facing OBI as wire bundles (one per lane)
    obi_signal_bundle<data_t> p_bus[kNumIO];
    // TDM-facing OBI as wire bundles (one per slot)
    obi_signal_bundle<data_t> m_bus[kNumTdm];
    sc_signal<uint64_t>       fetch_addr_i[kNumTdm];
    sc_signal<bool>           fetch_addr_valid_i;
    DUT                      *dut = nullptr;

    void bind(const char *nm, sc_clock &clk) {
        dut = new DUT(nm);
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->active_mode(active_mode);
        for (int i = 0; i < kNumIO; ++i) {
            bind_obi(dut->p[i], p_bus[i]);
        }
        for (int t = 0; t < kNumTdm; ++t) {
            bind_obi(dut->m[t], m_bus[t]);
            dut->fetch_addr_i[t](fetch_addr_i[t]);
        }
        dut->fetch_addr_valid_i(fetch_addr_valid_i);
    }

    void idle() {
        for (int i = 0; i < kNumIO; ++i) {
            p_bus[i].req.write(false);
            p_bus[i].addr.write(0);
            p_bus[i].be.write(0);
            p_bus[i].wdata.write(data_t(0));
        }
        for (int t = 0; t < kNumTdm; ++t) {
            m_bus[t].gnt.write(false);
            m_bus[t].rvalid.write(false);
            m_bus[t].rdata.write(data_t(0));
            fetch_addr_i[t].write(0);
        }
        fetch_addr_valid_i.write(false);
    }

    void drive_all_lanes(uint64_t base) {
        for (int i = 0; i < kNumIO; ++i) {
            p_bus[i].req.write(true);
            p_bus[i].addr.write(base + static_cast<uint64_t>(i) * kBytes);
            p_bus[i].be.write(0xF);
            p_bus[i].wdata.write(data_t(static_cast<unsigned long long>(0x50 + i)));
        }
    }

    // grant mask over the 4 lanes right now (comb)
    uint32_t gnt_mask() {
        uint32_t m = 0;
        for (int i = 0; i < kNumIO; ++i)
            if (p_bus[i].gnt.read())
                m |= 1u << i;
        return m;
    }

    uint32_t rvalid_mask() {
        uint32_t m = 0;
        for (int i = 0; i < kNumIO; ++i)
            if (p_bus[i].rvalid.read())
                m |= 1u << i;
        return m;
    }
};

SC_MODULE(tb) {
    sc_clock       clk{"clk", 10, SC_NS};
    harness<DUT_A> A;
    harness<DUT_B> B;
    harness<DUT_C> C;
    harness<DUT_D> D;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        A.bind("dutA", clk);
        B.bind("dutB", clk);
        C.bind("dutC", clk);
        D.bind("dutD", clk);
        SC_THREAD(run);
    }

    template <typename H> void reset(H & h) {
        h.idle();
        h.rst_n.write(false);
        tick(clk);
        tick(clk);
        h.rst_n.write(true);
        tick(clk);
    }

    // Drive all 4 lanes and return the grant mask the buffer answers with.
    template <typename H> uint32_t probe_gnt(H & h, uint32_t mode) {
        reset(h);
        h.active_mode.write(mode);
        tick(clk);
        h.drive_all_lanes(0x100);
        wait(1, SC_NS);
        const uint32_t m = h.gnt_mask();
        h.idle();
        tick(clk);
        return m;
    }

    // Fill a whole window in the given mode, serve the snapshot burst, and
    // report {fill groups, burst width, ack-group count, every-ack-full-width}
    // for structural checks. Acks are POSTED and stream one group per cycle
    // behind the snapshot, so they are collected over the drain, not sampled
    // at one instant.
    template <typename H>
    void run_window(H & h, uint32_t mode, int expect_lanes, int &fills, int &burst_lanes,
                    int &ack_groups, bool &acks_full_width) {
        reset(h);
        h.active_mode.write(mode);
        tick(clk);
        fills                    = 0;
        ack_groups               = 0;
        acks_full_width          = true;
        const uint32_t want_mask = (1u << expect_lanes) - 1;
        uint64_t       base      = 0x200;
        // fill until every TDM lane has latched (at most kNumTdm cycles)
        for (int c = 0; c < kNumTdm + 2 && fills * expect_lanes < kNumTdm; ++c) {
            h.drive_all_lanes(base);
            wait(1, SC_NS);
            if (h.gnt_mask() != 0)
                ++fills;
            if (h.rvalid_mask() != 0) { // acks may already overlap the fill
                ++ack_groups;
                if (h.rvalid_mask() != want_mask)
                    acks_full_width = false;
            }
            tick(clk);
            base += static_cast<uint64_t>(expect_lanes) * kBytes;
        }
        h.idle();
        wait(1, SC_NS);
        // snapshot has fired: the shadow burst is on the TDM side
        burst_lanes = 0;
        for (int t = 0; t < kNumTdm; ++t)
            if (h.m_bus[t].req.read())
                ++burst_lanes;
        // serve all grants and drain the posted ack stream
        for (int t = 0; t < kNumTdm; ++t)
            h.m_bus[t].gnt.write(true);
        for (int c = 0; c < kNumTdm; ++c) {
            wait(1, SC_NS);
            const uint32_t rm = h.rvalid_mask();
            if (rm != 0) {
                ++ack_groups;
                if (rm != want_mask)
                    acks_full_width = false;
            }
            tick(clk);
        }
        for (int t = 0; t < kNumTdm; ++t)
            h.m_bus[t].gnt.write(false);
        tick(clk);
    }

    // ---- read-mode helpers ------------------------------------------------
    // Emulated always-ready TDM bank side: gnt is held high; each cycle a
    // sampled request produces an rvalid the following cycle with
    // rdata = addr (an easy per-slot fingerprint).
    template <typename H> void serve_reads(H & h, int cycles) {
        bool     was_req[kNumTdm]  = {};
        uint64_t was_addr[kNumTdm] = {};
        for (int t = 0; t < kNumTdm; ++t)
            h.m_bus[t].gnt.write(true);
        for (int c = 0; c < cycles; ++c) {
            wait(1, SC_NS);
            for (int t = 0; t < kNumTdm; ++t) {
                h.m_bus[t].rvalid.write(was_req[t]);
                if (was_req[t])
                    h.m_bus[t].rdata.write(data_t(static_cast<unsigned long long>(was_addr[t])));
                was_req[t]  = h.m_bus[t].req.read();
                was_addr[t] = h.m_bus[t].addr.read();
            }
            tick(clk);
        }
        for (int t = 0; t < kNumTdm; ++t) {
            h.m_bus[t].gnt.write(false);
            h.m_bus[t].rvalid.write(false);
        }
    }

    // Prime a read buffer: expose a full window of fetch addresses, serve
    // the fetches, then drain with all 4 lanes requesting. Reports the
    // grant mask of the first drain and whether every drained beat's rdata
    // equals its slot's fetch address, over one full window.
    template <typename H>
    void run_read_window(H & h, uint32_t mode, int expect_lanes, uint32_t &first_mask,
                         bool &data_ok, int &drains) {
        reset(h);
        h.active_mode.write(mode);
        for (int t = 0; t < kNumTdm; ++t)
            h.fetch_addr_i[t].write(0x900 + 0x10 * static_cast<uint64_t>(t));
        h.fetch_addr_valid_i.write(true);
        tick(clk);
        serve_reads(h, kNumTdm + 6); // plenty to fetch the whole window
        data_ok            = true;
        drains             = 0;
        first_mask         = 0;
        const int n_groups = kNumTdm / expect_lanes;
        // p_rdata is REGISTERED: the grant fires in cycle g, the data is
        // presented in cycle g+1 — so run one extra iteration and check each
        // group's data one iteration after its grant.
        for (int g = 0; g <= n_groups; ++g) {
            if (g < n_groups)
                h.drive_all_lanes(0x0); // read side ignores addr/wdata on p
            else
                h.idle();
            wait(1, SC_NS);
            if (g < n_groups) {
                const uint32_t gm = h.gnt_mask();
                if (g == 0)
                    first_mask = gm;
                if (gm != 0)
                    ++drains;
            }
            if (g > 0) {
                for (int i = 0; i < expect_lanes; ++i) {
                    const uint64_t want =
                        0x900 + 0x10 * static_cast<uint64_t>((g - 1) * expect_lanes + i);
                    if (h.p_bus[i].rdata.read().to_uint64() != want)
                        data_ok = false;
                }
            }
            tick(clk);
        }
        h.idle();
        tick(clk);
    }

    void run() {
        // -------------------------------------------------------------------
        std::printf("\n=== T01-T03: instance A grant span per mode ===\n");
        CHECK(probe_gnt(A, 0) == 0x1, "T01 mode0 grants exactly lane 0");
        CHECK(probe_gnt(A, 1) == 0x3, "T02 mode1 grants exactly lanes 0-1");
        CHECK(probe_gnt(A, 2) == 0xF, "T03 mode2 grants all four lanes");

        // -------------------------------------------------------------------
        std::printf("\n=== T04: mode3 aliases mode2 ===\n");
        CHECK(probe_gnt(A, 3) == 0xF, "T04a mode3 grant span identical to mode2");
        {
            int  f2, f3, b2, b3, g2, g3;
            bool w2, w3;
            run_window(A, 2, 4, f2, b2, g2, w2);
            run_window(A, 3, 4, f3, b3, g3, w3);
            CHECK(f2 == 2 && f3 == 2, "T04b both modes fill the 8-window in 2 groups of 4");
            CHECK(b2 == b3 && g2 == g3 && w2 && w3,
                  "T04c snapshot burst and ack stream identical across the alias");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T05: A/mode2 full window end to end ===\n");
        {
            int  f, b, g;
            bool w;
            run_window(A, 2, 4, f, b, g, w);
            CHECK(f == 2, "T05a window filled in exactly 2 four-lane groups");
            CHECK(b == kNumTdm, "T05b snapshot bursts all 8 TDM lanes at once");
            CHECK(g == 2 && w, "T05c posted acks stream 2 full 4-lane groups");
            // burst content: beat k of the fill must sit in slot k with its
            // own address/data (run_window fills addr base+16k, data 0x50+lane)
            bool content = true;
            {
                int  fx, bx, gx;
                bool wx;
                // refill and inspect the burst before serving it
                reset(A);
                A.active_mode.write(2);
                tick(clk);
                uint64_t base2 = 0x600;
                for (int grp2 = 0; grp2 < 2; ++grp2) {
                    A.drive_all_lanes(base2);
                    wait(1, SC_NS);
                    tick(clk);
                    base2 += 4 * kBytes;
                }
                A.idle();
                wait(1, SC_NS);
                for (int t = 0; t < kNumTdm; ++t) {
                    const uint64_t want_a = 0x600 + static_cast<uint64_t>(t) * kBytes;
                    const uint64_t want_d = 0x50 + static_cast<uint64_t>(t % 4);
                    if (!A.m_bus[t].req.read() || A.m_bus[t].addr.read() != want_a ||
                        A.m_bus[t].wdata.read().to_uint64() != want_d)
                        content = false;
                }
                CHECK(content, "T05d burst slots carry exact per-beat addr/data");
                for (int t = 0; t < kNumTdm; ++t)
                    A.m_bus[t].gnt.write(true);
                tick(clk);
                for (int t = 0; t < kNumTdm; ++t)
                    A.m_bus[t].gnt.write(false);
                tick(clk);
                (void)fx;
                (void)bx;
                (void)gx;
                (void)wx;
            }
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T06: B clamps mode2 (4 ports requested, 1 exists) ===\n");
        CHECK(probe_gnt(B, 2) == 0xF,
              "T06a all 4 lanes of the single port granted under the clamp");
        {
            int  f0, f2, b0, b2, g0, g2;
            bool w0, w2;
            run_window(B, 0, 4, f0, b0, g0, w0);
            run_window(B, 2, 4, f2, b2, g2, w2);
            CHECK(f0 == 2 && f2 == 2 && b0 == b2 && g0 == g2 && w0 && w2,
                  "T06b mode2 behaves exactly like mode0 (clamped to the one port)");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T07-T09: READ-mode geometry on instance C (4 ports x 1 lane) ===\n");
        {
            uint32_t m0, m1, m2, m3;
            bool     d0, d1, d2, d3;
            int      n0, n1, n2, n3;
            run_read_window(C, 0, 1, m0, d0, n0);
            run_read_window(C, 1, 2, m1, d1, n1);
            run_read_window(C, 2, 4, m2, d2, n2);
            run_read_window(C, 3, 4, m3, d3, n3);
            CHECK(m0 == 0x1 && n0 == kNumTdm, "T07a mode0 drains lane 0 only, 8 groups");
            CHECK(d0, "T07b mode0 every beat returns its slot's fetched data");
            CHECK(m1 == 0x3 && n1 == kNumTdm / 2 && d1,
                  "T08 mode1 drains lanes 0-1 in 4 groups, data exact");
            CHECK(m2 == 0xF && n2 == kNumTdm / 4 && d2,
                  "T09a mode2 drains all four lanes in 2 groups, data exact");
            CHECK(m3 == m2 && n3 == n2 && d3, "T09b mode3 aliases mode2 on the read side too");
        }

        // -------------------------------------------------------------------
        std::printf("\n=== T10: READ-mode clamp on instance D (1 port x 4 lanes) ===\n");
        {
            uint32_t m0, m2;
            bool     d0, d2;
            int      n0, n2;
            run_read_window(D, 0, 4, m0, d0, n0);
            run_read_window(D, 2, 4, m2, d2, n2);
            CHECK(m0 == 0xF && n0 == kNumTdm / 4 && d0,
                  "T10a mode0 drains the single port's 4 lanes together");
            CHECK(m2 == m0 && n2 == n0 && d2,
                  "T10b mode2 clamps to the one port — identical to mode0");
        }

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t("tb");
    sc_start();
    return report_and_exit();
}
