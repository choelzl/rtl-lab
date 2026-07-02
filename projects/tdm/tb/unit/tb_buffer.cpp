// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer<NUM_REQ=1, PORT_COUNT=2, BYTES_PER_ROW=4, NUM_TDM=4>
//
//   NUM_IO = PORT_COUNT * NUM_REQ = 2
//   active_mode=0 → na=1 (4 drain groups of 1 cell each)
//   active_mode=1 → na=2 (2 drain groups of 2 cells each)
//
// Build and run:
//   make -C projects/tdm/tb/unit
//
// Tests:
//   T01: Reset — TDM and port outputs all deasserted
//   T02: fetch_addr_valid_i — all cells start requesting on TDM
//   T03: TDM m_req_o stable until grant arrives
//   T04: No drain before cells VALID (group not ready)
//   T05: fill_all_cells — cells become VALID; m_req_o deasserts
//   T06: No drain when p_req_i=0 (group valid, request missing)
//   T07: p_gnt_o combinatorial when group valid + all_req
//   T08: p_rvalid_o registered one cycle after drain (OBI R-5 at system boundary)
//   T09: Correct data forwarded on first drain (cell 0)
//   T10: rd_ptr advances — second drain delivers cell 1's data
//   T11: Last-group drain; rd_ptr wraps to 0 — each cell already started its
//        own next fetch the instant it drained (fetch_addr_valid_i has been
//        held continuously since T02)
//   T12: mode0 — p_bus[1].gnt always 0 (port outside active count)
//   T13: mode1 — partial request (one of two ports) blocks drain
//   T14: mode1 — both ports granted and respond simultaneously
//   T15: Full 4-group round-trip (mode0): all drains then re-fetch
//   T16: Bank contention on cell 0's very first fetch — the group must not
//        drain (must not forward anything) until it actually lands
//   T17: Same contention, but on a STEADY-STATE refetch (not the first one)
//   T18: Multi-lane group (na=2) — ONE lane contended, the other not: the
//        whole group stalls, not just the contended lane
//   T19: Same multi-lane contention, specifically at the wraparound-landing
//        group (the position that would normally get the zero-bubble
//        is_fwd handoff) — confirms it isn't a special case
//   T20: Bootstrap window_reset pulse — the all-at-once bootstrap latch
//        consumes the staged window, so it pulses window_reset exactly like
//        a wraparound (the caller's "advance one window" contract)
//   T21: Pulse-driven caller across windows — every drain-triggered refetch
//        latches the NEXT window's addresses, and the next window's drain
//        returns ITS data (pins the one-window-late bug the bootstrap
//        pulse fixed)
//   T22: parked cells (drained while fetch_addr_valid_i was low — a task
//        fence) restart the very edge it returns, after a gap of any
//        length — no threshold, no reprime handshake — and the all-cells
//        restart pulses window_reset like the bootstrap it is
//   T23: mode change against a parked buffer — the restart edge re-latches
//        the window geometry from the caller's current active_mode
//   T24: mid-drain mode change — active_mode flipping while a window is
//        still draining does NOT reprime; the latched geometry finishes
//        the window and the wrap re-latches (the unfenced task-boundary
//        "ride the pipeline" contract)
//   T25: fenced mode-change resume — a long en gap AND a lane-count change
//        together: one reprime snaps mode, base and cells in one shot
//        (the production phase-fence shape)
//   T26: en held high through reset — the bootstrap latch and its pulse
//        fire on the very first live edge (tb_top_tdm's constant-true
//        fetch_addr_valid_i wiring)
// -----------------------------------------------------------------------------

#include "buffer.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
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
using DUT    = buffer<kNumReq, kPortCount, kBytes, kNumTdm>;

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
    // Fetch address bus
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
            fetch_addr_i[t].write(static_cast<uint64_t>(t) * 0x100);
        }
        wait(clk.posedge_event());
        wait(clk.posedge_event());
        rst_n.write(true);
        tick(clk);
    }

    // Latch addresses into all cells simultaneously. fetch_addr_valid_i is
    // left asserted afterward (never pulsed-then-dropped) — each cell now
    // starts its own next fetch the instant its own group drains
    // (all_valid_i), not on a separate trigger (see buffer_cell.hpp's own
    // header comment), so en_i has to be continuously held for that to ever
    // happen, matching how a real AGU would drive it in production.
    void fetch_addresses(uint64_t addrs[kNumTdm]) {
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrs[t]);
        fetch_addr_valid_i.write(true);
        tick(clk);
    }

    // Grant all cells simultaneously (one tick), then send rvalid+data (one tick).
    void fill_all_cells(data_t data[kNumTdm]) {
        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].gnt.write(true);
        tick(clk);
        for (int t = 0; t < kNumTdm; ++t)
            m_bus[t].gnt.write(false);

        for (int t = 0; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(true);
            m_bus[t].rdata.write(data[t]);
        }
        tick(clk);
        for (int t = 0; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(false);
            m_bus[t].rdata.write(data_t{0});
        }
    }

    // -----------------------------------------------------------------------
    // Test cases
    // -----------------------------------------------------------------------
    void run() {
        static uint64_t addrs[kNumTdm] = {0x100, 0x200, 0x300, 0x400};
        static data_t   data[kNumTdm]  = {
            make_data(0xAAAA'0001),
            make_data(0xBBBB'0002),
            make_data(0xCCCC'0003),
            make_data(0xDDDD'0004),
        };

        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all outputs deasserted ===");
        // -------------------------------------------------------------------
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

        // -------------------------------------------------------------------
        std::puts("\n=== T02: fetch_addr_valid_i — cells request on TDM ===");
        // -------------------------------------------------------------------
        fetch_addresses(addrs);
        bool all_req = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req &= m_bus[t].req.read();
        CHECK(all_req, "T02a all m_bus[t].req=1 after fetch_addr_valid_i");
        CHECK(m_bus[0].addr.read() == addrs[0], "T02b m_bus[0].addr=latched addr");
        CHECK(m_bus[3].addr.read() == addrs[3], "T02c m_bus[3].addr=latched addr");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: m_req_o stable while waiting for TDM grant ===");
        // -------------------------------------------------------------------
        for (int c = 0; c < 3; ++c)
            tick(clk);
        bool still_req = true;
        for (int t = 0; t < kNumTdm; ++t)
            still_req &= m_bus[t].req.read();
        CHECK(still_req, "T03 m_bus[*].req=1 stable while gnt=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: No drain before cells VALID ===");
        // -------------------------------------------------------------------
        // Cells are REQUESTING — group not valid yet
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T04 p_gnt_o=0 while cells still fetching");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T05: fill_all_cells — cells become VALID ===");
        // -------------------------------------------------------------------
        fill_all_cells(data);
        bool all_req_deasserted = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req_deasserted &= !m_bus[t].req.read();
        CHECK(all_req_deasserted, "T05 m_bus[*].req=0 after fill (cells VALID, not REQUESTING)");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: No drain when p_req_i=0 (group valid, no request) ===");
        // -------------------------------------------------------------------
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T06 p_gnt_o=0 when p_req_i=0 even though group valid");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: p_gnt_o combinatorial when group valid + all_req ===");
        // -------------------------------------------------------------------
        p_bus[0].addr.write(addrs[0]); // port must present the pre-fetched address
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T07 p_bus[0].gnt=1 (combinatorial, same cycle as drain cond)");

        // -------------------------------------------------------------------
        // T08/T09: p_rvalid_o registered — fires ONE cycle after drain.
        //
        // The buffer seq_proc captures cell data and drives p_rvalid_o on the
        // next posedge.  This satisfies OBI R-5: earliest rvalid is the cycle
        // after req+gnt were sampled high.  (Contrast with the cell itself,
        // which fires p_rvalid_o combinatorially — see tb_buffer_cell T10.)
        // -------------------------------------------------------------------
        std::puts("\n=== T08/T09: p_rvalid_o registered; correct data (cell 0) ===");
        // Still in drain cond: tick → seq_proc fires rvalid + advances rd_ptr
        tick(clk);
        CHECK(p_bus[0].rvalid.read(), "T08 p_bus[0].rvalid=1 one cycle after drain (R-5 met)");
        CHECK(!p_bus[1].rvalid.read(), "T08b p_bus[1].rvalid=0 (only port 0 active in mode0)");
        CHECK(p_bus[0].rdata.read() == data[0], "T09 p_bus[0].rdata=data[0] (cell 0)");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T10: rd_ptr advances — second drain delivers cell 1's data ===");
        // -------------------------------------------------------------------
        // rd_ptr=1 now; cell[1] is VALID
        p_bus[0].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T10a p_bus[0].gnt=1 for group 1");
        tick(clk);
        CHECK(p_bus[0].rvalid.read(), "T10b p_bus[0].rvalid=1");
        CHECK(p_bus[0].rdata.read() == data[1], "T10c p_bus[0].rdata=data[1] (cell 1)");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: Last-group drain — rd_ptr wraps to 0; cells already refetching ===");
        // -------------------------------------------------------------------
        // Drain group 2 silently to advance rd_ptr to 3
        p_bus[0].addr.write(addrs[2]);
        p_bus[0].req.write(true);
        tick(clk);
        p_bus[0].req.write(false);

        // Drain group 3 — this is the last group (rd_ptr+na == BUFFER_SIZE)
        p_bus[0].addr.write(addrs[3]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T11a p_bus[0].gnt=1 for last group");
        tick(clk);
        CHECK(p_bus[0].rvalid.read(), "T11b p_bus[0].rvalid=1 for last drain");
        CHECK(p_bus[0].rdata.read() == data[3], "T11c p_bus[0].rdata=data[3] (cell 3)");
        p_bus[0].req.write(false);

        // After last drain: rd_ptr=0, no grants available yet (cell 0's own
        // refetch — started back at T09 the instant IT drained, since
        // fetch_addr_valid_i has been held continuously since T02 — hasn't
        // landed yet)
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T11d p_gnt_o=0 (cell 0 not valid again yet)");

        // Every cell already started its own next fetch the instant its own
        // group drained above (T09/T10/T11), not via any separate "reset
        // window" trigger (see buffer_cell.hpp's own header comment) — this
        // just confirms all four are mid-fetch. (fetch_addresses() here is a
        // no-op re-assertion: valid was already continuously held, and the
        // addresses haven't changed.)
        fetch_addresses(addrs);
        bool cells_refetch = true;
        for (int t = 0; t < kNumTdm; ++t)
            cells_refetch &= m_bus[t].req.read();
        CHECK(cells_refetch,
              "T11e all cells already mid their own next fetch (self-triggered on drain)");

        // -------------------------------------------------------------------
        std::puts("\n=== T12: mode0 — p_bus[1].gnt always 0 (outside active count) ===");
        // -------------------------------------------------------------------
        // Bring all cells to VALID again from the current REQUESTING state
        fill_all_cells(data);
        // mode0: na=1 → only port 0 active; p_bus[1].gnt is always 0
        p_bus[0].addr.write(addrs[0]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T12a p_bus[0].gnt=1 (active port)");
        CHECK(!p_bus[1].gnt.read(), "T12b p_bus[1].gnt=0 (port 1 outside active count in mode0)");
        // Drain remaining groups to leave buffer clean
        tick(clk);
        p_bus[0].req.write(false);
        p_bus[0].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        tick(clk);
        p_bus[0].req.write(false);
        p_bus[0].addr.write(addrs[2]);
        p_bus[0].req.write(true);
        tick(clk);
        p_bus[0].req.write(false);
        p_bus[0].addr.write(addrs[3]);
        p_bus[0].req.write(true);
        tick(clk);
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T13: mode1 — partial request blocks drain ===");
        // -------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2: groups [0,1] and [2,3]
        fetch_addresses(addrs);
        fill_all_cells(data);

        // Only port 0 requesting — all_req=false
        p_bus[0].req.write(true);
        p_bus[1].req.write(false);
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T13 p_bus[0].gnt=0 when p_bus[1].req=0 (partial req, na=2)");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T14: mode1 — both ports granted and respond simultaneously ===");
        // -------------------------------------------------------------------
        // Both ports request — each must present the address of its pre-fetched cell
        p_bus[0].addr.write(addrs[0]);
        p_bus[1].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T14a p_bus[0].gnt=1");
        CHECK(p_bus[1].gnt.read(), "T14b p_bus[1].gnt=1");
        tick(clk);
        CHECK(p_bus[0].rvalid.read(), "T14c p_bus[0].rvalid=1");
        CHECK(p_bus[1].rvalid.read(), "T14d p_bus[1].rvalid=1");
        CHECK(p_bus[0].rdata.read() == data[0], "T14e p_bus[0].rdata=data[0]");
        CHECK(p_bus[1].rdata.read() == data[1], "T14f p_bus[1].rdata=data[1]");
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T15: Full 4-group round-trip (mode0) ===");
        // -------------------------------------------------------------------
        do_reset();
        active_mode.write(0); // na=1: 4 groups
        fetch_addresses(addrs);
        fill_all_cells(data);

        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrs[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read(), "T15 group drain: p_bus[0].gnt=1");
            tick(clk);
            char label[64];
            std::snprintf(label, sizeof(label), "T15 group %d: p_bus[0].rvalid=1", g);
            CHECK(p_bus[0].rvalid.read(), label);
            std::snprintf(label, sizeof(label), "T15 group %d: p_bus[0].rdata=data[%d]", g, g);
            CHECK(p_bus[0].rdata.read() == data[g], label);
            p_bus[0].req.write(false);
        }

        // After all 4 groups drained: cells reset, rd_ptr=0
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T15 p_gnt_o=0 after full window drain");

        // Second window: re-fetch and drain two groups to confirm rd_ptr reset
        data_t data2[kNumTdm] = {
            make_data(0x1111'0001),
            make_data(0x2222'0002),
            make_data(0x3333'0003),
            make_data(0x4444'0004),
        };
        fetch_addresses(addrs);
        fill_all_cells(data2);

        p_bus[0].addr.write(addrs[0]);
        p_bus[0].req.write(true);
        tick(clk);
        CHECK(p_bus[0].rdata.read() == data2[0], "T15 second window cell[0] data correct");
        p_bus[0].req.write(false);

        p_bus[0].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        tick(clk);
        CHECK(p_bus[0].rdata.read() == data2[1], "T15 second window cell[1] data correct");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T16: Bank contention — multi-cycle fetch, first window ===");
        // -------------------------------------------------------------------
        // Cell 0's grant is deliberately withheld (simulating an upstream
        // arbiter conflict with some other requester for the same physical
        // bank — see top_tdm.hpp's own same-bank-conflict tests) while every
        // other cell fetches normally. The group must not drain — must not
        // forward anything at all — until cell 0's own fetch actually lands.
        do_reset();
        active_mode.write(0); // na=1: isolates cell 0 in its own group
        fetch_addresses(addrs);
        // Grant + respond for cells 1-3 only; cell 0 stays REQUESTING.
        for (int t = 1; t < kNumTdm; ++t)
            m_bus[t].gnt.write(true);
        tick(clk);
        for (int t = 1; t < kNumTdm; ++t)
            m_bus[t].gnt.write(false);
        for (int t = 1; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(true);
            m_bus[t].rdata.write(data[t]);
        }
        tick(clk);
        for (int t = 1; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(false);
            m_bus[t].rdata.write(data_t{0});
        }
        CHECK(m_bus[0].req.read(), "T16a cell 0 still REQUESTING (contended, not yet granted)");

        p_bus[0].addr.write(addrs[0]);
        p_bus[0].req.write(true);
        for (int i = 0; i < 3; ++i) {
            wait(1, SC_NS);
            CHECK(!p_bus[0].gnt.read(),
                  "T16b p_bus[0].gnt=0 while cell 0's fetch is still contended");
            tick(clk);
        }
        // Contention resolves: the TDM-side grant arrives first (a separate
        // handshake from the port side; registers cell 0's own granted_q).
        m_bus[0].gnt.write(true);
        tick(clk);
        m_bus[0].gnt.write(false);
        // Now the bank's response (rvalid+data) arrives. is_fwd reacts to it
        // COMBINATIONALLY (see buffer_cell.hpp's own header comment) — since
        // p_bus[0].req has been asserted the whole time, the group's
        // can_drain/p_gnt_o become true THIS SAME cycle, before the next
        // clock edge — exactly the same "gnt is combinational, rvalid+data
        // register one edge later" idiom every other test in this file
        // already uses (see T07/T08 above), just with the TDM-side
        // grant+rvalid handshake standing in for what would normally be an
        // already-fetched, already-VALID cell.
        m_bus[0].rvalid.write(true);
        m_bus[0].rdata.write(data[0]);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T16c p_bus[0].gnt=1 now that contention has resolved");
        tick(clk);
        m_bus[0].rvalid.write(false);
        m_bus[0].rdata.write(data_t{0});
        CHECK(p_bus[0].rvalid.read(), "T16d p_bus[0].rvalid=1");
        CHECK(p_bus[0].rdata.read() == data[0],
              "T16e p_bus[0].rdata=data[0] (correct, not garbage)");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T17: Bank contention on a STEADY-STATE refetch (not the first) ===");
        // -------------------------------------------------------------------
        // Cell 0 just drained above, which (fetch_addr_valid_i has been held
        // continuously since T02) also started its OWN next fetch for this
        // window. Withhold ITS grant this time, drain groups 1-3 normally to
        // wrap rd_ptr back around to cell 0, and confirm the exact same
        // stall-then-resume behaviour holds for a refetch as it did for the
        // very first one — the mechanism doesn't get a free pass just
        // because it's not cell 0's maiden fetch.
        CHECK(m_bus[0].req.read(), "T17a cell 0 already mid its own next fetch (self-triggered)");

        for (int g = 1; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrs[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read(), "T17 group drain (unrelated to cell 0's contended refetch)");
            tick(clk);
            p_bus[0].req.write(false);
        }
        // rd_ptr has wrapped back to cell 0 (several cycles have passed since
        // cell 0's own refetch started — well past its normal 2-cycle round
        // trip, and still contended) — the group must still refuse to drain.
        p_bus[0].addr.write(addrs[0]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(!p_bus[0].gnt.read(), "T17b p_bus[0].gnt=0 — cell 0's refetch is STILL contended");
        // Contention resolves now, several cycles later than a normal round
        // trip would have taken — same TDM-grant-then-rvalid sequencing as
        // T16 above (see that block's own comment on the timing).
        m_bus[0].gnt.write(true);
        tick(clk);
        m_bus[0].gnt.write(false);
        m_bus[0].rvalid.write(true);
        m_bus[0].rdata.write(data[0]);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T17c p_bus[0].gnt=1 — resolved, group drains correctly");
        tick(clk);
        m_bus[0].rvalid.write(false);
        m_bus[0].rdata.write(data_t{0});
        CHECK(p_bus[0].rvalid.read(), "T17d p_bus[0].rvalid=1");
        CHECK(p_bus[0].rdata.read() == data[0],
              "T17e p_bus[0].rdata=data[0] (correct, not stale/garbage)");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T18: Multi-lane group — ONE lane contended, the other not ===");
        // -------------------------------------------------------------------
        // na=2 (mode1): group 0 = {cell 0, cell 1}. Cell 0 fetches normally;
        // cell 1's grant is deliberately withheld. The whole GROUP must
        // stall — not just cell 1's own lane — since can_drain requires
        // every cell in the group valid; a partial drain (or forwarding
        // cell 0 alone while cell 1 is still fetching) is exactly the class
        // of race the "structurally-NOP lane races ahead of its slower
        // groupmate" fix (see buffer_cell.hpp's own header comment, and
        // last_drained_base_q's in buffer.hpp) protects against.
        do_reset();
        active_mode.write(1); // na=2: groups {0,1} and {2,3}
        fetch_addresses(addrs);
        // Group 1 {2,3} fetches normally — uninvolved in this test.
        for (int t = 2; t < kNumTdm; ++t)
            m_bus[t].gnt.write(true);
        tick(clk);
        for (int t = 2; t < kNumTdm; ++t)
            m_bus[t].gnt.write(false);
        for (int t = 2; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(true);
            m_bus[t].rdata.write(data[t]);
        }
        tick(clk);
        for (int t = 2; t < kNumTdm; ++t) {
            m_bus[t].rvalid.write(false);
            m_bus[t].rdata.write(data_t{0});
        }
        // Cell 0 fetches normally; cell 1 stays contended (withheld).
        m_bus[0].gnt.write(true);
        tick(clk);
        m_bus[0].gnt.write(false);
        m_bus[0].rvalid.write(true);
        m_bus[0].rdata.write(data[0]);
        tick(clk);
        m_bus[0].rvalid.write(false);
        m_bus[0].rdata.write(data_t{0});
        CHECK(m_bus[1].req.read(), "T18 pre: cell 1 still REQUESTING (contended, not yet granted)");

        p_bus[0].addr.write(addrs[0]);
        p_bus[1].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        for (int i = 0; i < 3; ++i) {
            wait(1, SC_NS);
            CHECK(!p_bus[0].gnt.read(),
                  "T18a p_bus[0].gnt=0 — whole group stalls even though cell 0 alone is ready");
            CHECK(!p_bus[1].gnt.read(), "T18b p_bus[1].gnt=0 — cell 1 still contended");
            tick(clk);
        }
        // Cell 1's contention resolves.
        m_bus[1].gnt.write(true);
        tick(clk);
        m_bus[1].gnt.write(false);
        m_bus[1].rvalid.write(true);
        m_bus[1].rdata.write(data[1]);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T18c p_bus[0].gnt=1 — group ready now both lanes are valid");
        CHECK(p_bus[1].gnt.read(), "T18d p_bus[1].gnt=1");
        tick(clk);
        m_bus[1].rvalid.write(false);
        m_bus[1].rdata.write(data_t{0});
        CHECK(p_bus[0].rvalid.read(), "T18e p_bus[0].rvalid=1");
        CHECK(p_bus[1].rvalid.read(), "T18f p_bus[1].rvalid=1");
        CHECK(p_bus[0].rdata.read() == data[0],
              "T18g p_bus[0].rdata=data[0] (cell 0's own, not cell 1's)");
        CHECK(p_bus[1].rdata.read() == data[1],
              "T18h p_bus[1].rdata=data[1] (cell 1's own, not cell 0's)");
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T19: Same multi-lane contention, at the wraparound-landing group ===");
        // -------------------------------------------------------------------
        // Group 0 just drained above, which (fetch_addr_valid_i held
        // continuously) also started both cells' own next fetch. Drain
        // group 1 {2,3} normally to wrap rd_ptr back around to group 0 —
        // exactly the position that would normally get the zero-bubble
        // is_fwd handoff (see buffer.hpp's own header comment) — then
        // contend ONE lane of THIS wraparound-landing group specifically,
        // confirming the same whole-group stall (not a partial/garbage
        // forward) holds there too, not just for an ordinary mid-window group.
        const data_t new_data0 = make_data(0x5555'0005);
        const data_t new_data1 = make_data(0x6666'0006);

        p_bus[0].addr.write(addrs[2]);
        p_bus[1].addr.write(addrs[3]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T19a group 1 {2,3} drains normally (already valid)");
        tick(clk);
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);

        // rd_ptr has wrapped back to group 0 {0,1}. Both cells' own refetch
        // started when THEY drained in T18 — cell 0's completes normally,
        // cell 1's is contended.
        m_bus[0].gnt.write(true);
        tick(clk);
        m_bus[0].gnt.write(false);
        m_bus[0].rvalid.write(true);
        m_bus[0].rdata.write(new_data0);
        tick(clk);
        m_bus[0].rvalid.write(false);
        m_bus[0].rdata.write(data_t{0});

        p_bus[0].addr.write(addrs[0]);
        p_bus[1].addr.write(addrs[1]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        for (int i = 0; i < 3; ++i) {
            wait(1, SC_NS);
            CHECK(!p_bus[0].gnt.read(),
                  "T19b p_bus[0].gnt=0 — wraparound group still stalls (cell 1 contended)");
            CHECK(!p_bus[1].gnt.read(), "T19c p_bus[1].gnt=0");
            tick(clk);
        }
        m_bus[1].gnt.write(true);
        tick(clk);
        m_bus[1].gnt.write(false);
        m_bus[1].rvalid.write(true);
        m_bus[1].rdata.write(new_data1);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(),
              "T19d p_bus[0].gnt=1 — resolved, wraparound group drains correctly");
        CHECK(p_bus[1].gnt.read(), "T19e p_bus[1].gnt=1");
        tick(clk);
        m_bus[1].rvalid.write(false);
        m_bus[1].rdata.write(data_t{0});
        CHECK(p_bus[0].rvalid.read(), "T19f p_bus[0].rvalid=1");
        CHECK(p_bus[1].rvalid.read(), "T19g p_bus[1].rvalid=1");
        CHECK(p_bus[0].rdata.read() == new_data0, "T19h p_bus[0].rdata=new data (cell 0's own)");
        CHECK(p_bus[1].rdata.read() == new_data1,
              "T19i p_bus[1].rdata=new data (cell 1's own, not cell 0's)");
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T20: bootstrap window_reset pulse — the caller's 'staged "
                  "window consumed' contract ===");
        // -------------------------------------------------------------------
        // The all-at-once bootstrap latch consumes the entire staged window,
        // so it must pulse window_reset exactly like a drain-cycle wraparound
        // does (see buffer.hpp's boot_latch) — one pulse, on the latch
        // edge, then low again. Without it the caller's lookahead cursor runs
        // one window behind from every bootstrap onward.
        do_reset();
        CHECK(!dut->snapshot().window_reset, "T20a no pulse while en is still low after reset");
        static uint64_t addrsA[kNumTdm] = {0x1100, 0x1200, 0x1300, 0x1400};
        fetch_addresses(addrsA); // en rises; cells latch this edge
        CHECK(dut->snapshot().window_reset, "T20b pulse fires on the bootstrap latch edge");
        tick(clk);
        CHECK(!dut->snapshot().window_reset, "T20c pulse is one cycle wide");
        tick(clk);
        CHECK(!dut->snapshot().window_reset, "T20d no repeat while fetches are merely in flight");

        // -------------------------------------------------------------------
        std::puts("\n=== T21: pulse-driven caller across windows — refetches latch "
                  "the NEXT window's addresses ===");
        // -------------------------------------------------------------------
        // Emulates a production caller exactly: hold en, and advance the
        // fetch bus one window per observed window_reset pulse — nothing
        // else. Window A was staged and consumed at T20's bootstrap; per the
        // contract the bus must now hold window B, so each group's
        // drain-triggered refetch latches B's addresses, and window B's own
        // drain returns B's data. (Pre-fix, each refetch re-latched its OWN
        // window's addresses and window B's drain returned window A's data
        // again — the exact one-window-late bug this pins down.)
        static uint64_t addrsB[kNumTdm] = {0x2100, 0x2200, 0x2300, 0x2400};
        static uint64_t addrsC[kNumTdm] = {0x3100, 0x3200, 0x3300, 0x3400};
        static data_t   dataA[kNumTdm]  = {make_data(0xA000'0001), make_data(0xA000'0002),
                                           make_data(0xA000'0003), make_data(0xA000'0004)};
        static data_t   dataB[kNumTdm]  = {make_data(0xB000'0001), make_data(0xB000'0002),
                                           make_data(0xB000'0003), make_data(0xB000'0004)};
        // T20's pulse was observed → advance the bus to window B.
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsB[t]);
        fill_all_cells(dataA); // serve window A's bootstrap fetches
        active_mode.write(0);  // na=1: four 1-cell groups
        bool wrap_seen = false;
        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrsA[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read(), "T21a window A group drains");
            tick(clk);
            CHECK(p_bus[0].rdata.read() == dataA[g], "T21b window A data correct");
            p_bus[0].req.write(false);
            // The drain just started this cell's refetch — it must target
            // window B's address for this lane, not window A's again.
            CHECK(m_bus[g].req.read(), "T21c refetch started on drain");
            CHECK(m_bus[g].addr.read() == addrsB[g],
                  "T21d refetch targets the NEXT window's address (not its own again)");
            wrap_seen = dut->snapshot().window_reset;
        }
        CHECK(wrap_seen, "T21e wraparound pulse fired at the last drain");
        // Pulse observed → advance the bus to window C, per the contract.
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsC[t]);
        fill_all_cells(dataB); // serve window B's refetches
        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrsB[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read(), "T21f window B group drains");
            tick(clk);
            CHECK(p_bus[0].rdata.read() == dataB[g],
                  "T21g window B returns B's data (one-window-late bug would return A's)");
            p_bus[0].req.write(false);
            CHECK(m_bus[g].addr.read() == addrsC[g], "T21h next refetch targets window C");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T22: parked cells restart the moment fetch_addr_valid_i "
                  "returns ===");
        // -------------------------------------------------------------------
        // A caller fenced behind a future task holds en low; cells that
        // drain during that stretch park (!valid && !pending — no refetch
        // can start without en). The cell start rule (!pending && en &&
        // (all_valid || !valid)) restarts them the very edge en returns —
        // no gap threshold, no edge detection, no reprime handshake: a
        // start wipes nothing, so resuming is safe after a gap of ANY
        // length, and the all-cells restart pulses window_reset like a
        // bootstrap (which it is).
        do_reset();
        fetch_addresses(addrsA);
        fill_all_cells(dataA);
        fetch_addr_valid_i.write(false); // fence: en drops before any drain
        active_mode.write(0);
        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrsA[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            tick(clk);
            p_bus[0].req.write(false);
        }
        bool no_refetch = true;
        for (int t = 0; t < kNumTdm; ++t)
            no_refetch &= !m_bus[t].req.read();
        CHECK(no_refetch, "T22a cells park with en low — no refetch starts");
        // The final drain's wrap still pulsed (a window WAS consumed drain-
        // side; the production caller's cursor is already parked at the
        // fence and absorbs it as its fenced rollover retry) — but after
        // that one pulse, a parked buffer must stay silent.
        tick(clk);
        CHECK(!dut->snapshot().window_reset, "T22b no further pulses while parked");
        // Resume after only this SHORT gap: the next task's addresses are
        // staged and en returns — everything restarts immediately. (NOTE
        // the caller contract this relies on: en only ever drops at window
        // boundaries — lookahead_ready() flips exactly at cursor
        // rollovers — so a parked buffer is always WHOLLY parked; en must
        // not drop after a window's drain cycle has partially consumed the
        // staged bus, since a partial restart couldn't be reported as a
        // whole-window pulse.)
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsB[t]);
        fetch_addr_valid_i.write(true);
        tick(clk); // cells latch the staged window THIS edge
        CHECK(dut->snapshot().window_reset, "T22c bootstrap pulse fires on the resume edge");
        bool all_refetch = true, addrs_ok = true;
        for (int t = 0; t < kNumTdm; ++t) {
            all_refetch &= m_bus[t].req.read();
            addrs_ok &= (m_bus[t].addr.read() == addrsB[t]);
        }
        CHECK(all_refetch, "T22d all cells fetching again after the resume");
        CHECK(addrs_ok, "T22e resume fetches use the freshly staged addresses");
        fill_all_cells(dataB);
        // Drain the whole resumed window with en low again (per the same
        // contract: the next "task" is fenced too) — data must be intact,
        // and the buffer parks wholly once more.
        fetch_addr_valid_i.write(false);
        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrsB[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read(), "T22f post-resume window drains");
            tick(clk);
            CHECK(p_bus[0].rdata.read() == dataB[g], "T22g post-resume data correct");
            p_bus[0].req.write(false);
        }
        // Long-gap variant (a real fence is thousands of cycles): idle a
        // long stretch parked, then resume — identical single-edge restart.
        for (int i = 0; i < 40; ++i)
            tick(clk);
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsA[t]);
        fetch_addr_valid_i.write(true);
        tick(clk);
        CHECK(dut->snapshot().window_reset, "T22h long-gap resume: same single-edge restart");
        bool long_ok = true;
        for (int t = 0; t < kNumTdm; ++t)
            long_ok &= m_bus[t].req.read() && (m_bus[t].addr.read() == addrsA[t]);
        CHECK(long_ok, "T22i and the same fresh-address fetches");

        // -------------------------------------------------------------------
        std::puts("\n=== T23: mode change against a parked buffer — geometry snaps "
                  "at the restart ===");
        // -------------------------------------------------------------------
        // A caller that moves to a task with a DIFFERENT lane count while
        // the buffer sits parked: the bootstrap latch re-latches
        // window_mode_q from the caller's CURRENT active_mode at the same
        // edge the cells restart, so the new window drains under the new
        // grouping (see buffer.hpp's boot_latch comment).
        do_reset();
        active_mode.write(1); // na=2: groups {0,1} {2,3}
        fetch_addresses(addrsA);
        fill_all_cells(dataA);
        fetch_addr_valid_i.write(false); // park cells over the drains
        for (int g = 0; g < 2; ++g) {
            p_bus[0].addr.write(addrsA[g * 2]);
            p_bus[1].addr.write(addrsA[g * 2 + 1]);
            p_bus[0].req.write(true);
            p_bus[1].req.write(true);
            wait(1, SC_NS);
            CHECK(p_bus[0].gnt.read() && p_bus[1].gnt.read(), "T23a mode1 group drains");
            tick(clk);
            p_bus[0].req.write(false);
            p_bus[1].req.write(false);
        }
        // window_mode_q latched mode1 at that wrap. Switch the caller to
        // mode0 with fresh addresses, then resume.
        active_mode.write(0);
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsB[t]);
        fetch_addr_valid_i.write(true);
        tick(clk); // restart edge: cells latch, geometry snaps, pulse fires
        CHECK(dut->snapshot().window_reset, "T23b bootstrap pulse fires on the restart");
        bool stale_refetch = true, stale_addrs = true;
        for (int t = 0; t < kNumTdm; ++t) {
            stale_refetch &= m_bus[t].req.read();
            stale_addrs &= (m_bus[t].addr.read() == addrsB[t]);
        }
        CHECK(stale_refetch, "T23c all cells fetching again");
        CHECK(stale_addrs, "T23d fetches use the new task's addresses");
        fill_all_cells(dataB);
        // Drain under the NEW mode0 grouping — single-cell groups.
        p_bus[0].addr.write(addrsB[0]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read(), "T23e first mode0 group drains under the new geometry");
        CHECK(!p_bus[1].gnt.read(), "T23f port 1 idle — grouping really is mode0 now");
        tick(clk);
        CHECK(p_bus[0].rdata.read() == dataB[0], "T23g new-mode data correct");
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T24: mid-drain mode change — the latched window is "
                  "untouched ===");
        // -------------------------------------------------------------------
        // active_mode flipping while a window is STILL DRAINING must not
        // disturb it: mid-drain cells are valid or pending — never all
        // idle — so no bootstrap latch can fire, the in-flight window
        // completes under its latched geometry, and only the wrap
        // re-latches the new mode. This is exactly how an unfenced task
        // boundary rides the pipeline in production (active_mode is
        // la-synchronized and switches while the old task's last window
        // drains).
        do_reset();
        active_mode.write(1); // na=2: groups {0,1} {2,3}
        fetch_addresses(addrsA);
        fill_all_cells(dataA);
        // Drain group {0,1}, then flip active_mode BETWEEN the two groups.
        p_bus[0].addr.write(addrsA[0]);
        p_bus[1].addr.write(addrsA[1]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read() && p_bus[1].gnt.read(), "T24a group {0,1} drains under mode1");
        tick(clk);
        active_mode.write(0); // caller already on the next task's config
        p_bus[0].addr.write(addrsA[2]);
        p_bus[1].addr.write(addrsA[3]);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read() && p_bus[1].gnt.read(),
              "T24b group {2,3} STILL drains 2-wide — latched mode survives the flip");
        tick(clk);
        CHECK(p_bus[0].rdata.read() == dataA[2] && p_bus[1].rdata.read() == dataA[3],
              "T24c its data is intact (no reprime wiped it mid-window)");
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);
        // The wrap that just happened re-latched mode0; the refetches
        // started by those drains latched the bus (still addrsA) — serve
        // them so the next test starts clean, and confirm the grouping
        // really is mode0 now.
        fill_all_cells(dataB);
        p_bus[0].addr.write(addrsA[0]);
        p_bus[0].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read() && !p_bus[1].gnt.read(),
              "T24d after the wrap the grouping is mode0 (single-cell groups)");
        tick(clk);
        p_bus[0].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T25: fenced MODE-CHANGE resume — one restart edge snaps "
                  "the geometry ===");
        // -------------------------------------------------------------------
        // T22 covered same-mode fence resumes, T23 a short-gap mode change.
        // This is the production phase-fence shape since the stimuli crutch
        // tasks were removed: a LONG en gap AND a lane-count change together
        // — the restart edge must snap mode, base and cells in one shot.
        do_reset();
        active_mode.write(0); // task A: mode0
        fetch_addresses(addrsA);
        fill_all_cells(dataA);
        fetch_addr_valid_i.write(false); // fence begins
        for (int g = 0; g < kNumTdm; ++g) {
            p_bus[0].addr.write(addrsA[g]);
            p_bus[0].req.write(true);
            wait(1, SC_NS);
            tick(clk);
            p_bus[0].req.write(false);
        }
        // Fenced next task: DIFFERENT mode (1), fresh addresses.
        active_mode.write(1);
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsB[t]);
        for (int i = 0; i < 40; ++i)
            tick(clk);
        fetch_addr_valid_i.write(true);
        tick(clk); // restart edge: cells latch, geometry snaps, pulse fires
        CHECK(dut->snapshot().window_reset, "T25a bootstrap pulse on the fenced resume");
        bool res_fetch = true, res_addrs = true;
        for (int t = 0; t < kNumTdm; ++t) {
            res_fetch &= m_bus[t].req.read();
            res_addrs &= (m_bus[t].addr.read() == addrsB[t]);
        }
        CHECK(res_fetch && res_addrs, "T25b all cells fetch the new task's addresses");
        fill_all_cells(dataB);
        p_bus[0].addr.write(addrsB[0]);
        p_bus[1].addr.write(addrsB[1]);
        p_bus[0].req.write(true);
        p_bus[1].req.write(true);
        wait(1, SC_NS);
        CHECK(p_bus[0].gnt.read() && p_bus[1].gnt.read(),
              "T25c first drain is 2-wide — geometry snapped to mode1 in the same reprime");
        tick(clk);
        CHECK(p_bus[0].rdata.read() == dataB[0] && p_bus[1].rdata.read() == dataB[1],
              "T25d new task's data under the new grouping");
        p_bus[0].req.write(false);
        p_bus[1].req.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T26: en held high THROUGH reset — bootstrap still clean ===");
        // -------------------------------------------------------------------
        // tb_top_tdm's wiring holds fetch_addr_valid_i constantly true, so
        // reset releases straight into an en-high edge: the cells must latch
        // on that first live edge and the pulse must fire with it.
        do_reset();
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrsC[t]);
        fetch_addr_valid_i.write(true);
        rst_n.write(false); // re-enter reset with en still high
        tick(clk);
        rst_n.write(true);
        tick(clk); // first live edge: bootstrap latch
        CHECK(dut->snapshot().window_reset, "T26a pulse fires on the first live edge");
        bool boot_fetch = true;
        for (int t = 0; t < kNumTdm; ++t)
            boot_fetch &= m_bus[t].req.read() && (m_bus[t].addr.read() == addrsC[t]);
        CHECK(boot_fetch, "T26b all cells fetching the staged window immediately");
        tick(clk);
        CHECK(!dut->snapshot().window_reset, "T26c pulse is one cycle wide here too");

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
