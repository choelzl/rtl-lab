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
// Write-mode protocol (pipelined fill / snapshot / posted respond — see
// buffer.hpp's header):
//   fill     — ports write na beats at a time; p_gnt_o fires when all na
//              ports assert p_req_i; fill-ptr advances; repeat until all
//              NUM_TDM cells hold latched port data.
//   snapshot — a one-cycle pulse hands the window to the cells' shadow
//              engines, which fire their TDM writes simultaneously (each
//              shadow frees at its own grant) while the freed primaries
//              accept the next window's fill.
//   respond  — POSTED p_rvalid_o streamed back na beats at a time behind the
//              snapshot (same group-by-group pattern as the READ drain).
//
// Tests:
//   T01   Reset — all outputs deasserted
//   T02   No grant while idle (ports not requesting)
//   T03   fill: p_gnt_o combinatorial when all na ports assert p_req_i
//   T04   fill: partial request blocks grant (only port 0 of 2 in mode1)
//   T05   fill: ports 0..na-1 only — p_bus[na..].gnt always 0 in mode0
//   T06   fill: fill-ptr advances — second group accepted on next posedge
//   T07   fill: p_rvalid_o=0 during fill (write ack not yet sent)
//   T08   snapshot at the fill wrap — the shadow flush fires AND the posted
//         acks start together
//   T09   posted acks stream while the (un-served) flush still holds its bus
//   T10   respond completes; the flush completes independently of it
//   T11   everything idle afterward (clean post-window state)
//         (T12-T15 of the old serial-protocol plan are folded into
//          T08-T11/T19-T20 by the pipelined redesign)
//   T16   mode1 — both ports granted together; full 2-group fill
//   T17   mode1 — both ports get p_rvalid_o simultaneously in respond
//   T18   TDM m_wdata_o / m_addr_o / m_be_o match latched port values
//   T19   Full round-trip mode0: fill 4 cells, flush, respond, refill
//   T20   Full round-trip mode1: fill 4 cells (2 groups of 2), flush, respond
//   T21   Back-pressure: the NEXT window's snapshot waits for the previous
//         un-served burst, releasing at its last grant
// -----------------------------------------------------------------------------

#include "buffer.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <systemc.h>

static constexpr int kNumReq        = 1;
static constexpr int kPortCount     = 2;
static constexpr int kBytes         = 4;
static constexpr int kNumTdm        = 4;
static constexpr int kNumIO         = kPortCount * kNumReq; // 2
static constexpr int n_groups_mode0 = kNumTdm;              // mode0: 1 beat per group

using data_t = obi_data<kBytes>;
using DUT    = buffer<kNumReq, kPortCount, kBytes, kNumTdm, /*IS_WRITE=*/true>;

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

    // Port-facing OBI as wire bundles (one per lane)
    obi_signal_bundle<data_t> p_bus[kNumIO];
    // TDM-facing OBI

    // TDM-facing OBI as wire bundles (one per slot)
    obi_signal_bundle<data_t> m_bus[kNumTdm];
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
            bind_obi(dut->p[i], p_bus[i]);
        }
        for (int t = 0; t < kNumTdm; ++t) {
            bind_obi(dut->m[t], m_bus[t]);
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

    void do_reset() {
        rst_n.write(false);
        active_mode.write(0);
        fetch_addr_valid_i.write(false);
        for (int i = 0; i < kNumIO; ++i) {
            p_bus[i].req.write(false);
            p_bus[i].addr.write(0);
            p_bus[i].be.write(0);
            p_bus[i].wdata.write(data_t{0});
        }
        for (int t = 0; t < kNumTdm; ++t) {
            m_bus[t].gnt.write(false);
            m_bus[t].rvalid.write(false);
            m_bus[t].rdata.write(data_t{0});
        }
        wait(clk.posedge_event());
        wait(clk.posedge_event());
        rst_n.write(true);
        tick(clk);
    }

    // Present one group of write requests to ports 0..na-1.
    void issue_group(int na, uint64_t addrs[], data_t wdatas[], uint32_t be = 0xFu) {
        for (int i = 0; i < na; ++i) {
            p_bus[i].req.write(true);
            p_bus[i].addr.write(addrs[i]);
            p_bus[i].wdata.write(wdatas[i]);
            p_bus[i].be.write(be);
        }
    }

    void deassert_ports(int na) {
        for (int i = 0; i < na; ++i)
            p_bus[i].req.write(false);
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
            tick(clk); // posedge: cells latch; fill-ptr advances
            deassert_ports(na);
        }
    }

    // Grant and ack all TDM cells simultaneously (simulated TDM that accepts
    // all requests in one cycle and responds one cycle later).
    void flush_all_cells() {
        // All cells start REQUESTING after flush_s is asserted.
        // Drive gnt to all, then rvalid one cycle later.
        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].gnt.write(true);
        tick(clk);
        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].gnt.write(false);

        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].rvalid.write(true);
        tick(clk);
        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].rvalid.write(false);
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
            all_m_req_low &= !m_bus[t].req.read();
        CHECK(all_m_req_low, "T01a m_bus[*].req=0 after reset");
        bool all_p_gnt_low = true;
        for (int i = 0; i < kNumIO; ++i)
            all_p_gnt_low &= !p_bus[i].gnt.read();
        CHECK(all_p_gnt_low, "T01b p_bus[*].gnt=0 after reset");
        bool all_p_rvalid_low = true;
        for (int i = 0; i < kNumIO; ++i)
            all_p_rvalid_low &= !p_bus[i].rvalid.read();
        CHECK(all_p_rvalid_low, "T01c p_bus[*].rvalid=0 after reset");
        bool all_m_we_low = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_m_we_low &= !m_bus[t].we.read();
        CHECK(all_m_we_low, "T01d m_bus[*].we=0 after reset");

        // -----------------------------------------------------------------------
        std::puts("\n=== T02: No grant while idle (no port requests) ===");
        // -----------------------------------------------------------------------
        for (int i = 0; i < 3; ++i)
            tick(clk);
        bool no_gnt = true;
        for (int i = 0; i < kNumIO; ++i)
            no_gnt &= !p_bus[i].gnt.read();
        CHECK(no_gnt, "T02 p_bus[*].gnt=0 (no ports requesting)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T03: fill — p_gnt_o combinatorial when all na ports request ===");
        // -----------------------------------------------------------------------
        // mode0: na=1; port 0 requesting → gnt[0] fires combinatorially
        do_reset();
        active_mode.write(0);
        p_bus[0].req.write(true);
        p_bus[0].addr.write(0x100);
        p_bus[0].wdata.write(make_data(0xAABBCCDD));
        p_bus[0].be.write(0xF);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T03a p_bus[0].gnt=1 (combinatorial, same cycle as req)");
        CHECK(!p_bus[1].gnt.read(), "T03b p_bus[1].gnt=0 (only port 0 active)");
        tick(clk);
        deassert_ports(1);

        // -----------------------------------------------------------------------
        std::puts("\n=== T04: fill — partial request blocks grant (mode1) ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2
        // only port 0 requesting — port 1 not ready
        p_bus[0].req.write(true);
        p_bus[1].req.write(false);
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T04 p_bus[0].gnt=0 when p_bus[1].req=0 (partial req, na=2)");
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T05: fill — p_bus[1].gnt always 0 in mode0 ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true); // irrelevant — na=1
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T05a p_bus[0].gnt=1");
        CHECK(!p_bus[1].gnt.read(), "T05b p_bus[1].gnt=0 (port 1 inactive in mode0)");
        tick(clk);
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T06: fill — fill-ptr advances; second group accepted ===");
        // -----------------------------------------------------------------------
        // After granting group 0, the next req (group 1) goes into cell 1.
        // Verify p_gnt_o fires again on the next request.
        do_reset();
        active_mode.write(0);
        // group 0
        p_bus[0].req.write(true);
        p_bus[0].addr.write(0x100);
        p_bus[0].wdata.write(make_data(0x11111111));
        p_bus[0].be.write(0xF);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T06a group 0: p_bus[0].gnt=1");
        tick(clk);
        deassert_ports(1);
        // group 1
        p_bus[0].req.write(true);
        p_bus[0].addr.write(0x200);
        p_bus[0].wdata.write(make_data(0x22222222));
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T06b group 1: p_bus[0].gnt=1 (fill-ptr advanced)");
        tick(clk);
        deassert_ports(1);

        // -----------------------------------------------------------------------
        std::puts("\n=== T07: fill — p_rvalid_o=0 during fill (no write ack yet) ===");
        // -----------------------------------------------------------------------
        // Continuing from above: still in fill phase
        bool no_rvalid = true;
        for (int i = 0; i < kNumIO; ++i)
            no_rvalid &= !p_bus[i].rvalid.read();
        CHECK(no_rvalid, "T07 p_bus[*].rvalid=0 during fill (write not yet flushed to TDM)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T08: snapshot at the fill wrap — flush fires AND acks start ===");
        // -----------------------------------------------------------------------
        // Filling the whole window snapshots it into the cells' shadow flush
        // engines on the final fill edge: the TDM burst fires, and — acks
        // being POSTED — the first respond group streams out the very next
        // cycle, concurrent with the in-flight flush.
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
        tick(clk);
        bool all_req = true, all_we = true;
        for (int t = 0; t < kNumTdm; ++t) {
            all_req &= m_bus[t].req.read();
            all_we &= m_bus[t].we.read();
        }
        CHECK(all_req, "T08a m_bus[*].req=1 — whole-window TDM burst in flight");
        CHECK(all_we, "T08b m_bus[*].we=1 (all writes)");
        CHECK(p_bus[0].rvalid.read(), "T08c first posted ack already streaming (group 0)");
        CHECK(!p_bus[1].rvalid.read(), "T08d p_bus[1].rvalid=0 (port 1 inactive in mode0)");
        CHECK(p_bus[0].rdata.read() == data_t{0}, "T08e p_rdata_o=0 (write carries no data)");

        // -----------------------------------------------------------------------
        std::puts("\n=== T09: acks stream while the un-served flush holds its bus ===");
        // -----------------------------------------------------------------------
        for (int g = 1; g < kNumTdm; ++g) {
            tick(clk);
            bool still_req = true;
            for (int t = 0; t < kNumTdm; ++t)
                still_req &= m_bus[t].req.read();
            char lbl[96];
            std::snprintf(lbl, sizeof(lbl),
                          "T09 group %d: p_bus[0].rvalid=1 while m_bus[*].req still held", g);
            CHECK(p_bus[0].rvalid.read() && still_req, lbl);
        }

        // -----------------------------------------------------------------------
        std::puts("\n=== T10: respond completes; flush completes independently ===");
        // -----------------------------------------------------------------------
        tick(clk);
        bool rvalid_low = true;
        for (int i = 0; i < kNumIO; ++i)
            rvalid_low &= !p_bus[i].rvalid.read();
        CHECK(rvalid_low, "T10a p_bus[*].rvalid=0 after the window's 4 groups (no repeats)");
        flush_all_cells();
        bool all_req_low = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req_low &= !m_bus[t].req.read();
        CHECK(all_req_low, "T10b m_bus[*].req=0 after the banks ack the burst");

        // -----------------------------------------------------------------------
        std::puts("\n=== T11: everything idle afterward ===");
        // -----------------------------------------------------------------------
        tick(clk);
        bool gnt_low = true;
        rvalid_low   = true;
        for (int i = 0; i < kNumIO; ++i) {
            gnt_low &= !p_bus[i].gnt.read();
            rvalid_low &= !p_bus[i].rvalid.read();
        }
        CHECK(gnt_low && rvalid_low, "T11 no grants, no acks — cleanly idle");

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
        p_bus[0].req.write(true);
        p_bus[0].addr.write(addrs2[0]);
        p_bus[0].wdata.write(wdatas2[0]);
        p_bus[0].be.write(0xF);
        p_bus[1].req.write(true);
        p_bus[1].addr.write(addrs2[1]);
        p_bus[1].wdata.write(wdatas2[1]);
        p_bus[1].be.write(0xF);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T16a group0: p_bus[0].gnt=1");
        CHECK(p_bus[1].gnt.read(), "T16b group0: p_bus[1].gnt=1");
        tick(clk);
        deassert_ports(2);
        // group 1: ports 0+1 → cells 2+3
        p_bus[0].req.write(true);
        p_bus[0].addr.write(addrs2[2]);
        p_bus[0].wdata.write(wdatas2[2]);
        p_bus[0].be.write(0xF);
        p_bus[1].req.write(true);
        p_bus[1].addr.write(addrs2[3]);
        p_bus[1].wdata.write(wdatas2[3]);
        p_bus[1].be.write(0xF);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T16c group1: p_bus[0].gnt=1");
        CHECK(p_bus[1].gnt.read(), "T16d group1: p_bus[1].gnt=1");
        tick(clk);
        deassert_ports(2);

        // -----------------------------------------------------------------------
        std::puts("\n=== T17: mode1 — both ports get p_rvalid_o simultaneously ===");
        // -----------------------------------------------------------------------
        // T16's final fill group was the snapshot edge; the two 2-wide ack
        // groups stream on the following cycles (posted, flush not yet
        // served).
        tick(clk); // respond group 0
        CHECK(p_bus[0].rvalid.read(), "T17a p_bus[0].rvalid=1 (group0, mode1)");
        CHECK(p_bus[1].rvalid.read(), "T17b p_bus[1].rvalid=1 (group0, mode1 — both ports active)");
        tick(clk); // respond group 1
        CHECK(p_bus[0].rvalid.read(), "T17c p_bus[0].rvalid=1 (group1)");
        CHECK(p_bus[1].rvalid.read(), "T17d p_bus[1].rvalid=1 (group1)");
        flush_all_cells(); // serve the concurrent burst before the next test

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
        tick(clk); // flush: cells start requesting TDM
        // Verify each cell drives the correct values on TDM
        for (int t = 0; t < kNumTdm; ++t) {
            char lbl[80];
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_addr_o=latched port addr", t);
            CHECK(m_bus[t].addr.read() == chk_addrs[t], lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_wdata_o=latched port wdata", t);
            CHECK(m_bus[t].wdata.read() == chk_wdatas[t], lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_be_o=latched port be", t);
            CHECK(m_bus[t].be.read() == chk_be, lbl);
            std::snprintf(lbl, sizeof(lbl), "T18[%d] m_we_o=1 (write)", t);
            CHECK(m_bus[t].we.read(), lbl);
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
        // Posted acks: 4 respond groups stream right off the snapshot.
        for (int g = 0; g < kNumTdm; ++g) {
            tick(clk);
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "T19 respond group %d: p_bus[0].rvalid=1", g);
            CHECK(p_bus[0].rvalid.read(), lbl);
        }
        flush_all_cells(); // serve the concurrent TDM burst
        // A new fill is accepted immediately (primaries were freed by the
        // snapshot; in fact it would have been accepted even earlier).
        p_bus[0].req.write(true);
        p_bus[0].addr.write(0x50);
        p_bus[0].wdata.write(make_data(0xF0000005));
        p_bus[0].be.write(0xF);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T19 after full round-trip: p_bus[0].gnt=1 (new fill accepted)");
        tick(clk);
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
        // Posted acks: 2 groups × 2 ports stream right off the snapshot.
        for (int g = 0; g < kNumTdm / 2; ++g) {
            tick(clk);
            char lbl[64];
            std::snprintf(lbl, sizeof(lbl), "T20 respond group %d: p_bus[0].rvalid=1", g);
            CHECK(p_bus[0].rvalid.read(), lbl);
            std::snprintf(lbl, sizeof(lbl), "T20 respond group %d: p_bus[1].rvalid=1", g);
            CHECK(p_bus[1].rvalid.read(), lbl);
        }
        tick(clk);
        bool back_fill = !p_bus[0].rvalid.read() && !p_bus[1].rvalid.read();
        CHECK(back_fill, "T20 p_bus[*].rvalid=0 after the window's acks complete");
        flush_all_cells();

        // -----------------------------------------------------------------------
        std::puts("\n=== T21: back-pressure — the NEXT window's snapshot waits for "
                  "the un-served flush ===");
        // -----------------------------------------------------------------------
        // Posted acks moved the ordering guarantee: window A's acks never
        // wait for its banks, but window B's SNAPSHOT (and therefore B's
        // acks and B's TDM burst) must wait until A's shadows are all
        // served — that's where a slow bank now costs.
        do_reset();
        active_mode.write(0);
        fill_all_cells(1, rt_addrs, rt_wdatas); // window A; snapshot fires, burst holds
        // A's acks stream regardless of the banks never answering yet.
        int a_acks = 0;
        for (int g = 0; g < kNumTdm; ++g) {
            tick(clk);
            a_acks += p_bus[0].rvalid.read();
        }
        CHECK(a_acks == kNumTdm, "T21a window A fully acked while its burst is still un-served");
        // Window B fills fine (primaries were freed by A's snapshot)...
        uint64_t b_addrs[kNumTdm]  = {0x50, 0x60, 0x70, 0x80};
        data_t   b_wdatas[kNumTdm] = {
            make_data(0xB0000001),
            make_data(0xB0000002),
            make_data(0xB0000003),
            make_data(0xB0000004),
        };
        fill_all_cells(1, b_addrs, b_wdatas);
        // ...but B's snapshot is blocked: the TDM side must still show A.
        bool held_a = true;
        int  b_acks = 0;
        for (int i = 0; i < 4; ++i) {
            tick(clk);
            held_a &= m_bus[0].req.read() && (m_bus[0].addr.read() == rt_addrs[0]);
            b_acks += p_bus[0].rvalid.read();
        }
        CHECK(held_a, "T21b TDM side still holds window A's burst (B's snapshot blocked)");
        CHECK(b_acks == 0, "T21c and B gets NO acks until its snapshot can fire");
        // Serve A's burst staggered, one cell at a time. A shadow frees at
        // its GRANT (the bank samples the payload the edge after — see
        // buffer_cell.hpp), so B's snapshot releases the moment A's LAST
        // grant lands, mid-loop — count B's acks from here on.
        b_acks = 0;
        for (int t = 0; t < kNumTdm; ++t) {
            m_bus[t].gnt.write(true);
            tick(clk);
            m_bus[t].gnt.write(false);
            b_acks += p_bus[0].rvalid.read();
            m_bus[t].rvalid.write(true);
            tick(clk);
            m_bus[t].rvalid.write(false);
            b_acks += p_bus[0].rvalid.read();
        }
        for (int i = 0; i < kNumTdm + 2 && b_acks < kNumTdm; ++i) {
            tick(clk);
            b_acks += p_bus[0].rvalid.read();
        }
        CHECK(m_bus[0].req.read() && m_bus[0].addr.read() == b_addrs[0],
              "T21d window B's burst fires once A's last grant frees the shadows");
        CHECK(b_acks == kNumTdm, "T21e window B's acks stream off the released snapshot");
        flush_all_cells();

        // -----------------------------------------------------------------------
        // -----------------------------------------------------------------------
        std::puts("\n=== T22: fill order -> slot mapping is exact (mode0) ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        tick(clk);
        {
            uint64_t a[kNumTdm];
            data_t   d[kNumTdm];
            for (int t = 0; t < kNumTdm; ++t) {
                a[t] = 0x2200 + 0x10 * static_cast<uint64_t>(t);
                d[t] = make_data(0x22000000u + static_cast<uint32_t>(t));
            }
            fill_all_cells(1, a, d, 0x9); // note the non-trivial be
            wait(1, SC_NS);
            bool slots_ok = true;
            for (int t = 0; t < kNumTdm; ++t)
                slots_ok &= m_bus[t].req.read() && (m_bus[t].addr.read() == a[t]) &&
                            (m_bus[t].wdata.read() == d[t]) && (m_bus[t].be.read() == 0x9) &&
                            m_bus[t].we.read();
            CHECK(slots_ok,
                  "T22a beat k lands in slot k with its own addr/data and the group's be");
            flush_all_cells();
            bool quiet = true;
            for (int t = 0; t < kNumTdm; ++t)
                quiet &= !m_bus[t].req.read();
            CHECK(quiet, "T22b all shadows freed after their grants");
        }

        // -----------------------------------------------------------------------
        std::puts("\n=== T23: posted acks carry rdata=0 on every lane, whole round ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0);
        tick(clk);
        {
            uint64_t a[kNumTdm];
            data_t   d[kNumTdm];
            for (int t = 0; t < kNumTdm; ++t) {
                a[t] = 0x2600 + 0x10 * static_cast<uint64_t>(t);
                d[t] = make_data(0x2600u + static_cast<uint32_t>(t));
            }
            fill_all_cells(1, a, d);
            bool rdata_zero = true;
            int  acks       = 0;
            for (int c = 0; c < kNumTdm + 4; ++c) {
                for (int t = 0; t < kNumTdm; ++t)
                    m_bus[t].gnt.write(true);
                wait(1, SC_NS);
                if (p_bus[0].rvalid.read()) {
                    ++acks;
                    if (p_bus[0].rdata.read().to_uint64() != 0)
                        rdata_zero = false;
                }
                tick(clk);
            }
            for (int t = 0; t < kNumTdm; ++t)
                m_bus[t].gnt.write(false);
            CHECK(acks == kNumTdm, "T23a every group acked exactly once");
            CHECK(rdata_zero, "T23b p_rdata is 0 on every posted ack");
        }

        // -----------------------------------------------------------------------
        std::puts("\n=== T24: inactive lane stays silent through a whole round ===");
        // -----------------------------------------------------------------------
        do_reset();
        active_mode.write(0); // 1 active port of the 2
        tick(clk);
        {
            uint64_t a[kNumTdm];
            data_t   d[kNumTdm];
            for (int t = 0; t < kNumTdm; ++t) {
                a[t] = 0x2a00 + 0x10 * static_cast<uint64_t>(t);
                d[t] = make_data(0x2a00u + static_cast<uint32_t>(t));
            }
            // request on BOTH lanes throughout, but mode0 keeps lane 1 inactive
            bool lane1_quiet = true;
            for (int g = 0; g < n_groups_mode0; ++g) {
                p_bus[0].req.write(true);
                p_bus[0].addr.write(a[g]);
                p_bus[0].wdata.write(d[g]);
                p_bus[0].be.write(0xF);
                p_bus[1].req.write(true);
                p_bus[1].addr.write(0x3f00);
                p_bus[1].be.write(0xF);
                wait(1, SC_NS);
                lane1_quiet &= !p_bus[1].gnt.read();
                tick(clk);
                deassert_ports(2);
            }
            for (int c = 0; c < kNumTdm + 4; ++c) {
                for (int t = 0; t < kNumTdm; ++t)
                    m_bus[t].gnt.write(true);
                wait(1, SC_NS);
                lane1_quiet &= !p_bus[1].rvalid.read();
                tick(clk);
            }
            for (int t = 0; t < kNumTdm; ++t)
                m_bus[t].gnt.write(false);
            CHECK(lane1_quiet, "T24 inactive lane gets neither gnt nor rvalid, ever");
        }

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
