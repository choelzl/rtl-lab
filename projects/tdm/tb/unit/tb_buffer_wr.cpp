// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer<NUM_REQ=1, PORT_COUNT=2, BYTES_PER_ROW=4, NUM_TDM=4,
//                        IS_WRITE=true>
//
//   NUM_IO = PORT_COUNT * NUM_REQ = 2
//   active_mode=0 → na=1  (4 fill groups of 1 cell each)
//   active_mode=1 → na=2  (2 fill groups of 2 cells each)
//
// Write-mode protocol (FILL → FLUSH → RESPOND):
//   FILL    — ports write na beats at a time; p_gnt_o fires when all na ports
//             assert p_req_i; fill-ptr advances; repeat until all NUM_TDM cells
//             hold latched port data.
//   FLUSH   — buffer asserts flush internally; all cells start TDM write
//             transactions simultaneously; buffer waits for every cell to reach
//             VALID (TDM write ack).
//   RESPOND — p_rvalid_o fired back to ports na beats at a time (same
//             group-by-group pattern as the READ drain); one group per cycle.
//
// Tests:
//   T01   Reset — all outputs deasserted
//   T02   No grant while idle (ports not requesting)
//   T03   FILL: p_gnt_o combinatorial when all na ports assert p_req_i
//   T04   FILL: partial request blocks grant (only port 0 of 2 in mode1)
//   T05   FILL: ports 0..na-1 only — p_gnt_o[na..] always 0 in mode0
//   T06   FILL: fill-ptr advances — second group accepted on next posedge
//   T07   FILL: p_rvalid_o=0 during fill phase (write ack not yet sent)
//   T08   FLUSH: m_req_o[*]=1 once buffer full; m_we_o[*]=1
//   T09   FLUSH: TDM signals stable while gnt=0
//   T10   FLUSH: after all TDM gnt+rvalid — m_req_o deasserts
//   T11   RESPOND: p_rvalid_o fires for group 0 after full TDM flush
//   T12   RESPOND: p_rvalid_o[1]=0 in mode0 (only 1 active port)
//   T13   RESPOND: p_rdata_o always 0 (write carries no read data)
//   T14   RESPOND: all groups get p_rvalid_o before window resets (mode0)
//   T15   RESPOND: resets to FILL after last group responds
//   T16   mode1 — both ports granted together; full 2-group fill
//   T17   mode1 — both ports get p_rvalid_o simultaneously in RESPOND
//   T18   TDM m_wdata_o / m_addr_o / m_be_o match latched port values
//   T19   Full round-trip mode0: fill 4 cells, flush, respond, refill
//   T20   Full round-trip mode1: fill 4 cells (2 groups of 2), flush, respond
//   T21   Staggered TDM grants: cells flush and respond only after all VALID
// -----------------------------------------------------------------------------

#include "buffer.hpp"
#include "obi_data.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <systemc.h>

static constexpr int kNumReq    = 1;
static constexpr int kPortCount = 2;
static constexpr int kBytes     = 4;
static constexpr int kNumTdm    = 4;
static constexpr int kNumIO     = kPortCount * kNumReq; // 2

using data_t = obi_data<kBytes>;
using DUT    = buffer<kNumReq, kPortCount, kBytes, kNumTdm, /*IS_WRITE=*/true>;

// ---------------------------------------------------------------------------
// Test accounting
// ---------------------------------------------------------------------------
static int g_pass = 0;
static int g_fail = 0;

static void CHECK(bool cond, const char *label) {
    if (cond) {
        ++g_pass;
        std::printf("  PASS  %s\n", label);
    } else {
        ++g_fail;
        std::printf("  FAIL  %s\n", label);
    }
}

static data_t make_data(uint32_t v) {
    return data_t(static_cast<unsigned long long>(v));
}

// ---------------------------------------------------------------------------
// Testbench module
// ---------------------------------------------------------------------------
SC_MODULE(tb) {
    sc_clock            clk{"clk", 10, SC_NS};
    sc_signal<bool>     rst_n{"rst_n"};
    sc_signal<uint32_t> active_mode{"active_mode"};

    // Port-facing OBI
    sc_signal<bool>     p_req_i[kNumIO];
    sc_signal<uint64_t> p_addr_i[kNumIO];
    sc_signal<uint32_t> p_be_i[kNumIO];
    sc_signal<data_t>   p_wdata_i[kNumIO];
    sc_signal<bool>     p_gnt_o[kNumIO];
    sc_signal<bool>     p_rvalid_o[kNumIO];
    sc_signal<data_t>   p_rdata_o[kNumIO];

    // TDM-facing OBI
    sc_signal<bool>     m_req_o[kNumTdm];
    sc_signal<uint64_t> m_addr_o[kNumTdm];
    sc_signal<bool>     m_we_o[kNumTdm];
    sc_signal<uint32_t> m_be_o[kNumTdm];
    sc_signal<data_t>   m_wdata_o[kNumTdm];
    sc_signal<bool>     m_gnt_i[kNumTdm];
    sc_signal<bool>     m_rvalid_i[kNumTdm];
    sc_signal<data_t>   m_rdata_i[kNumTdm];

    // Fetch address bus — unused in write mode; held at 0
    sc_signal<uint64_t> fetch_addr_i[kNumTdm];
    sc_signal<bool>     fetch_addr_valid_i{"fetch_addr_valid_i"};

    DUT *dut;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->active_mode(active_mode);

        for (int i = 0; i < kNumIO; ++i) {
            dut->p_req_i[i](p_req_i[i]);
            dut->p_addr_i[i](p_addr_i[i]);
            dut->p_be_i[i](p_be_i[i]);
            dut->p_wdata_i[i](p_wdata_i[i]);
            dut->p_gnt_o[i](p_gnt_o[i]);
            dut->p_rvalid_o[i](p_rvalid_o[i]);
            dut->p_rdata_o[i](p_rdata_o[i]);
        }
        for (int t = 0; t < kNumTdm; ++t) {
            dut->m_req_o[t](m_req_o[t]);
            dut->m_addr_o[t](m_addr_o[t]);
            dut->m_we_o[t](m_we_o[t]);
            dut->m_be_o[t](m_be_o[t]);
            dut->m_wdata_o[t](m_wdata_o[t]);
            dut->m_gnt_i[t](m_gnt_i[t]);
            dut->m_rvalid_i[t](m_rvalid_i[t]);
            dut->m_rdata_i[t](m_rdata_i[t]);
            dut->fetch_addr_i[t](fetch_addr_i[t]);
        }
        dut->fetch_addr_valid_i(fetch_addr_valid_i);

        SC_THREAD(run);
    }

    ~tb() {
        delete dut;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    void tick() {
        wait(clk.posedge_event());
        wait(1, SC_NS);
    }

    void do_reset() {
        rst_n.write(false);
        active_mode.write(0);
        fetch_addr_valid_i.write(false);
        for (int i = 0; i < kNumIO; ++i) {
            p_req_i[i].write(false);
            p_addr_i[i].write(0);
            p_be_i[i].write(0);
            p_wdata_i[i].write(data_t{0});
        }
        for (int t = 0; t < kNumTdm; ++t) {
            m_gnt_i[t].write(false);
            m_rvalid_i[t].write(false);
            m_rdata_i[t].write(data_t{0});
        }
        wait(clk.posedge_event());
        wait(clk.posedge_event());
        rst_n.write(true);
        tick();
    }

    // Present one group of write requests to ports 0..na-1.
    void issue_group(int na, uint64_t addrs[], data_t wdatas[], uint32_t be = 0xFu) {
        for (int i = 0; i < na; ++i) {
            p_req_i[i].write(true);
            p_addr_i[i].write(addrs[i]);
            p_wdata_i[i].write(wdatas[i]);
            p_be_i[i].write(be);
        }
    }

    void deassert_ports(int na) {
        for (int i = 0; i < na; ++i)
            p_req_i[i].write(false);
    }

    // Fill all NUM_TDM cells by issuing NUM_TDM/na groups of na port writes.
    // Waits for p_gnt_o after each group; returns after last grant.
    // On entry: active_mode must already be set; no port requests asserted.
    void fill_all_cells(int na, uint64_t addrs[kNumTdm], data_t wdatas[kNumTdm],
                        uint32_t be = 0xFu) {
        int n_groups = kNumTdm / na;
        for (int g = 0; g < n_groups; ++g) {
            issue_group(na, &addrs[g * na], &wdatas[g * na], be);
            wait(1, SC_NS); // let combinatorial grant settle
            // gnt fires when all na ports request
            tick(); // posedge: cells latch; fill-ptr advances
            deassert_ports(na);
        }
    }

    // Grant and ack all TDM cells simultaneously (simulated TDM that accepts
    // all requests in one cycle and responds one cycle later).
    void flush_all_cells() {
        // All cells start REQUESTING after flush_s is asserted.
        // Drive gnt to all, then rvalid one cycle later.
        for (int t = 0; t < kNumTdm; ++t)
            m_gnt_i[t].write(true);
        tick();
        for (int t = 0; t < kNumTdm; ++t)
            m_gnt_i[t].write(false);

        for (int t = 0; t < kNumTdm; ++t)
            m_rvalid_i[t].write(true);
        tick();
        for (int t = 0; t < kNumTdm; ++t)
            m_rvalid_i[t].write(false);
    }

    // -----------------------------------------------------------------------
    // Test thread
    // -----------------------------------------------------------------------
    void run() {
        // -----------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all outputs deasserted ===");
        // -----------------------------------------------------------------------
        do_reset();
        bool all_m_req_low = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_m_req_low &= !m_req_o[t].read();
        CHECK(all_m_req_low, "T01a m_req_o[*]=0 after reset");
        bool all_p_gnt_low = true;
        for (int i = 0; i < kNumIO; ++i)
            all_p_gnt_low &= !p_gnt_o[i].read();
        CHECK(all_p_gnt_low, "T01b p_gnt_o[*]=0 after reset");
        bool all_p_rvalid_low = true;
        for (int i = 0; i < kNumIO; ++i)
            all_p_rvalid_low &= !p_rvalid_o[i].read();
        CHECK(all_p_rvalid_low, "T01c p_rvalid_o[*]=0 after reset");
        bool all_m_we_low = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_m_we_low &= !m_we_o[t].read();
        CHECK(all_m_we_low, "T01d m_we_o[*]=0 after reset");

        // -----------------------------------------------------------------------
        std::puts("\n=== T02: No grant while idle (no port requests) ===");
        // -----------------------------------------------------------------------
        for (int i = 0; i < 3; ++i)
            tick();
        bool no_gnt = true;
        for (int i = 0; i < kNumIO; ++i)
            no_gnt &= !p_gnt_o[i].read();
        CHECK(no_gnt, "T02 p_gnt_o[*]=0 (no ports requesting)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T03: FILL — p_gnt_o combinatorial when all na ports request ===");
        // -----------------------------------------------------------------------
        // mode0: na=1; port 0 requesting → gnt[0] fires combinatorially
        do_reset();
        active_mode.write(0);
        p_req_i[0].write(true);
        p_addr_i[0].write(0x100);
        p_wdata_i[0].write(make_data(0xAABBCCDD));
        p_be_i[0].write(0xF);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T03a p_gnt_o[0]=1 (combinatorial, same cycle as req)");
        CHECK(!p_gnt_o[1].read(), "T03b p_gnt_o[1]=0 (only port 0 active)");
        tick();
        deassert_ports(1);

        // -----------------------------------------------------------------------
        std::puts("\n=== T04: FILL — partial request blocks grant (mode1) ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2
        // only port 0 requesting — port 1 not ready
        p_req_i[0].write(true);
        p_req_i[1].write(false);
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T04 p_gnt_o[0]=0 when p_req_i[1]=0 (partial req, na=2)");
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T05: FILL — p_gnt_o[1] always 0 in mode0 ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        p_req_i[0].write(true);
        p_req_i[1].write(true); // irrelevant — na=1
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T05a p_gnt_o[0]=1");
        CHECK(!p_gnt_o[1].read(), "T05b p_gnt_o[1]=0 (port 1 inactive in mode0)");
        tick();
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T06: FILL — fill-ptr advances; second group accepted ===");
        // -----------------------------------------------------------------------
        // After granting group 0, the next req (group 1) goes into cell 1.
        // Verify p_gnt_o fires again on the next request.
        do_reset();
        active_mode.write(0);
        // group 0
        p_req_i[0].write(true);
        p_addr_i[0].write(0x100);
        p_wdata_i[0].write(make_data(0x11111111));
        p_be_i[0].write(0xF);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T06a group 0: p_gnt_o[0]=1");
        tick();
        deassert_ports(1);
        // group 1
        p_req_i[0].write(true);
        p_addr_i[0].write(0x200);
        p_wdata_i[0].write(make_data(0x22222222));
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T06b group 1: p_gnt_o[0]=1 (fill-ptr advanced)");
        tick();
        deassert_ports(1);

        // -----------------------------------------------------------------------
        std::puts("\n=== T07: FILL — p_rvalid_o=0 during fill (no write ack yet) ===");
        // -----------------------------------------------------------------------
        // Continuing from above: still in fill phase
        bool no_rvalid = true;
        for (int i = 0; i < kNumIO; ++i)
            no_rvalid &= !p_rvalid_o[i].read();
        CHECK(no_rvalid, "T07 p_rvalid_o[*]=0 during fill (write not yet flushed to TDM)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T08: FLUSH — m_req_o[*]=1 and m_we_o[*]=1 once buffer full ===");
        // -----------------------------------------------------------------------
        // Fill remaining 2 groups (groups 2 and 3) to complete the buffer
        do_reset();
        active_mode.write(0);
        uint64_t addrs[kNumTdm]  = {0x100, 0x200, 0x300, 0x400};
        data_t   wdatas[kNumTdm] = {
            make_data(0x11111111),
            make_data(0x22222222),
            make_data(0x33333333),
            make_data(0x44444444),
        };
        fill_all_cells(1, addrs, wdatas);
        // After filling all 4 cells, buffer enters FLUSH — all cells start TDM write
        // Wait 1 cycle for the FLUSH phase to propagate (phase transition is registered)
        tick();
        bool all_req = true, all_we = true;
        for (int t = 0; t < kNumTdm; ++t) {
            all_req &= m_req_o[t].read();
            all_we &= m_we_o[t].read();
        }
        CHECK(all_req, "T08a m_req_o[*]=1 in FLUSH (all cells requesting TDM)");
        CHECK(all_we, "T08b m_we_o[*]=1 in FLUSH (all cells writing)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T09: FLUSH — TDM signals stable while gnt=0 ===");
        // -----------------------------------------------------------------------
        for (int c = 0; c < 3; ++c) {
            tick();
            bool still_req = true;
            for (int t = 0; t < kNumTdm; ++t)
                still_req &= m_req_o[t].read();
            CHECK(still_req, "T09 m_req_o[*]=1 stable while gnt=0");
        }

        // -----------------------------------------------------------------------
        std::puts("\n=== T10: FLUSH — after all TDM ack, m_req_o deasserts ===");
        // -----------------------------------------------------------------------
        flush_all_cells();
        bool all_req_low = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req_low &= !m_req_o[t].read();
        CHECK(all_req_low, "T10 m_req_o[*]=0 after all TDM acks (cells VALID)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T11: RESPOND — p_rvalid_o fires for group 0 ===");
        // -----------------------------------------------------------------------
        // One cycle after all cells VALID, RESPOND phase starts and first group fires
        tick();
        CHECK(p_rvalid_o[0].read(), "T11a p_rvalid_o[0]=1 (first respond group)");
        CHECK(!p_rvalid_o[1].read(), "T11b p_rvalid_o[1]=0 (port 1 inactive in mode0)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T12: RESPOND — p_rvalid_o[1]=0 in mode0 ===");
        // -----------------------------------------------------------------------
        // (Already verified above as part of T11b)
        CHECK(!p_rvalid_o[1].read(), "T12 p_rvalid_o[1]=0 (mode0 na=1)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T13: RESPOND — p_rdata_o always 0 (write carries no data) ===");
        // -----------------------------------------------------------------------
        for (int i = 0; i < kNumIO; ++i)
            CHECK(p_rdata_o[i].read() == data_t{0}, "T13 p_rdata_o[i]=0 (write mode)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T14: RESPOND — all 4 groups get p_rvalid_o (mode0) ===");
        // -----------------------------------------------------------------------
        // Continuing from T11 which already fired group 0.
        // Groups 1, 2, 3 fire on successive cycles.
        for (int g = 1; g < kNumTdm; ++g) {
            tick();
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "T14 group %d: p_rvalid_o[0]=1", g);
            CHECK(p_rvalid_o[0].read(), lbl);
        }

        // -----------------------------------------------------------------------
        std::puts("\n=== T15: RESPOND → FILL reset after last group ===");
        // -----------------------------------------------------------------------
        // After last respond group, buffer resets to FILL. No more grants.
        tick();
        bool gnt_low = true;
        for (int i = 0; i < kNumIO; ++i)
            gnt_low &= !p_gnt_o[i].read();
        CHECK(gnt_low, "T15a p_gnt_o[*]=0 (back in FILL, no port requests pending)");
        bool rvalid_low = true;
        for (int i = 0; i < kNumIO; ++i)
            rvalid_low &= !p_rvalid_o[i].read();
        CHECK(rvalid_low, "T15b p_rvalid_o[*]=0 (FILL phase, no respond pending)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T16: mode1 — both ports granted together (2-group fill) ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2: 2 groups of 2 cells
        uint64_t addrs2[kNumTdm]  = {0xA00, 0xB00, 0xC00, 0xD00};
        data_t   wdatas2[kNumTdm] = {
            make_data(0xAAAA0001),
            make_data(0xBBBB0002),
            make_data(0xCCCC0003),
            make_data(0xDDDD0004),
        };
        // group 0: ports 0+1 → cells 0+1
        p_req_i[0].write(true);
        p_addr_i[0].write(addrs2[0]);
        p_wdata_i[0].write(wdatas2[0]);
        p_be_i[0].write(0xF);
        p_req_i[1].write(true);
        p_addr_i[1].write(addrs2[1]);
        p_wdata_i[1].write(wdatas2[1]);
        p_be_i[1].write(0xF);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T16a group0: p_gnt_o[0]=1");
        CHECK(p_gnt_o[1].read(), "T16b group0: p_gnt_o[1]=1");
        tick();
        deassert_ports(2);
        // group 1: ports 0+1 → cells 2+3
        p_req_i[0].write(true);
        p_addr_i[0].write(addrs2[2]);
        p_wdata_i[0].write(wdatas2[2]);
        p_be_i[0].write(0xF);
        p_req_i[1].write(true);
        p_addr_i[1].write(addrs2[3]);
        p_wdata_i[1].write(wdatas2[3]);
        p_be_i[1].write(0xF);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T16c group1: p_gnt_o[0]=1");
        CHECK(p_gnt_o[1].read(), "T16d group1: p_gnt_o[1]=1");
        tick();
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T17: mode1 — both ports get p_rvalid_o simultaneously ===");
        // -----------------------------------------------------------------------
        tick(); // FLUSH phase starts
        flush_all_cells();
        tick(); // RESPOND group 0
        CHECK(p_rvalid_o[0].read(), "T17a p_rvalid_o[0]=1 (group0, mode1)");
        CHECK(p_rvalid_o[1].read(), "T17b p_rvalid_o[1]=1 (group0, mode1 — both ports active)");
        tick(); // RESPOND group 1
        CHECK(p_rvalid_o[0].read(), "T17c p_rvalid_o[0]=1 (group1)");
        CHECK(p_rvalid_o[1].read(), "T17d p_rvalid_o[1]=1 (group1)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T18: m_wdata_o / m_addr_o / m_be_o match latched port values ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        uint64_t chk_addrs[kNumTdm]  = {0x111, 0x222, 0x333, 0x444};
        data_t   chk_wdatas[kNumTdm] = {
            make_data(0xDEAD0001),
            make_data(0xDEAD0002),
            make_data(0xDEAD0003),
            make_data(0xDEAD0004),
        };
        uint32_t chk_be = 0xA; // non-trivial byte enable
        fill_all_cells(1, chk_addrs, chk_wdatas, chk_be);
        tick(); // FLUSH: cells start requesting TDM
        // Verify each cell drives the correct values on TDM
        for (int t = 0; t < kNumTdm; ++t) {
            char lbl[80];
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_addr_o=latched port addr", t);
            CHECK(m_addr_o[t].read() == chk_addrs[t], lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_wdata_o=latched port wdata", t);
            CHECK(m_wdata_o[t].read() == chk_wdatas[t], lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_be_o=latched port be", t);
            CHECK(m_be_o[t].read() == chk_be, lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_we_o=1 (write)", t);
            CHECK(m_we_o[t].read(), lbl);
        }
        flush_all_cells();

        // -----------------------------------------------------------------------
        std::puts("\n=== T19: Full round-trip mode0: fill→flush→respond→refill ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        uint64_t rt_addrs[kNumTdm]  = {0x10, 0x20, 0x30, 0x40};
        data_t   rt_wdatas[kNumTdm] = {
            make_data(0xF0000001),
            make_data(0xF0000002),
            make_data(0xF0000003),
            make_data(0xF0000004),
        };
        fill_all_cells(1, rt_addrs, rt_wdatas);
        tick();            // transition to FLUSH
        flush_all_cells(); // all cells → VALID
        // RESPOND: 4 groups × 1 port
        for (int g = 0; g < kNumTdm; ++g) {
            tick();
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "T19 respond group %d: p_rvalid_o[0]=1", g);
            CHECK(p_rvalid_o[0].read(), lbl);
        }
        // Back in FILL: a new write request should be granted
        tick();
        p_req_i[0].write(true);
        p_addr_i[0].write(0x50);
        p_wdata_i[0].write(make_data(0xF0000005));
        p_be_i[0].write(0xF);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T19 after full round-trip: p_gnt_o[0]=1 (new fill accepted)");
        tick();
        deassert_ports(1);

        // -----------------------------------------------------------------------
        std::puts("\n=== T20: Full round-trip mode1: 2-group fill→flush→respond ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2
        uint64_t rt2_addrs[kNumTdm]  = {0x1000, 0x2000, 0x3000, 0x4000};
        data_t   rt2_wdatas[kNumTdm] = {
            make_data(0xAA000001),
            make_data(0xAA000002),
            make_data(0xAA000003),
            make_data(0xAA000004),
        };
        fill_all_cells(2, rt2_addrs, rt2_wdatas);
        tick();
        flush_all_cells();
        // RESPOND: 2 groups × 2 ports
        for (int g = 0; g < kNumTdm / 2; ++g) {
            tick();
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "T20 respond group %d: p_rvalid_o[0]=1", g);
            CHECK(p_rvalid_o[0].read(), lbl);
            std::snprintf(lbl, sizeof(lbl), "T20 respond group %d: p_rvalid_o[1]=1", g);
            CHECK(p_rvalid_o[1].read(), lbl);
        }
        // Confirm reset to FILL
        tick();
        bool back_fill = !p_rvalid_o[0].read() && !p_rvalid_o[1].read();
        CHECK(back_fill, "T20 p_rvalid_o[*]=0 after RESPOND completes (back in FILL)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T21: Staggered TDM grants — RESPOND only after all VALID ===");
        // -----------------------------------------------------------------------
        // Fill buffer, then grant TDM cells one at a time.
        // RESPOND must not start until the LAST cell is acked.
        do_reset();
        active_mode.write(0);
        fill_all_cells(1, rt_addrs, rt_wdatas);
        tick(); // FLUSH: cells start TDM requests
        // Grant and ack cells one at a time (staggered, not all at once)
        for (int t = 0; t < kNumTdm; ++t) {
            m_gnt_i[t].write(true);
            tick();
            m_gnt_i[t].write(false);
            m_rvalid_i[t].write(true);
            tick();
            m_rvalid_i[t].write(false);

            // RESPOND must not start yet unless this is the last cell
            bool rvalid_premature = p_rvalid_o[0].read();
            if (t < kNumTdm - 1) {
                char lbl[80];
                std::snprintf(lbl, sizeof(lbl),
                              "T21 cell %d acked: p_rvalid_o[0]=0 (not all cells VALID yet)", t);
                CHECK(!rvalid_premature, lbl);
            }
        }
        // All cells now VALID → RESPOND starts
        tick();
        CHECK(p_rvalid_o[0].read(), "T21 all cells VALID: p_rvalid_o[0]=1 (RESPOND begins)");

        // Drain remaining respond groups
        for (int g = 1; g < kNumTdm; ++g)
            tick();

        // -----------------------------------------------------------------------
        std::puts("\n=== Summary ===");
        // -----------------------------------------------------------------------
        std::printf("  passed: %d\n  failed: %d\n", g_pass, g_fail);
        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();

    if (g_fail > 0) {
        std::fprintf(stderr, "\n%d test(s) FAILED\n", g_fail);
        return 1;
    }
    std::puts("\nAll tests passed.");
    return 0;
}
