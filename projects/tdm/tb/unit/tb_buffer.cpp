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
//   T11: Last-group drain fires reset_window; rd_ptr wraps to 0; cells → MISSING
//   T12: mode0 — p_gnt_o[1] always 0 (port outside active count)
//   T13: mode1 — partial request (one of two ports) blocks drain
//   T14: mode1 — both ports granted and respond simultaneously
//   T15: Full 4-group round-trip (mode0): all drains then re-fetch
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
using DUT    = buffer<kNumReq, kPortCount, kBytes, kNumTdm>;

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
    sc_signal<data_t>   m_wdata_o[kNumTdm]; // sink; unused in read-mode tests
    sc_signal<bool>     m_gnt_i[kNumTdm];
    sc_signal<bool>     m_rvalid_i[kNumTdm];
    sc_signal<data_t>   m_rdata_i[kNumTdm];

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
            fetch_addr_i[t].write(static_cast<uint64_t>(t) * 0x100);
        }
        wait(clk.posedge_event());
        wait(clk.posedge_event());
        rst_n.write(true);
        tick();
    }

    // Latch addresses into all cells simultaneously.
    void fetch_addresses(uint64_t addrs[kNumTdm]) {
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(addrs[t]);
        fetch_addr_valid_i.write(true);
        tick();
        fetch_addr_valid_i.write(false);
    }

    // Grant all cells simultaneously (one tick), then send rvalid+data (one tick).
    void fill_all_cells(data_t data[kNumTdm]) {
        for (int t = 0; t < kNumTdm; ++t)
            m_gnt_i[t].write(true);
        tick();
        for (int t = 0; t < kNumTdm; ++t)
            m_gnt_i[t].write(false);

        for (int t = 0; t < kNumTdm; ++t) {
            m_rvalid_i[t].write(true);
            m_rdata_i[t].write(data[t]);
        }
        tick();
        for (int t = 0; t < kNumTdm; ++t) {
            m_rvalid_i[t].write(false);
            m_rdata_i[t].write(data_t{0});
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

        // -------------------------------------------------------------------
        std::puts("\n=== T02: fetch_addr_valid_i — cells request on TDM ===");
        // -------------------------------------------------------------------
        fetch_addresses(addrs);
        bool all_req = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req &= m_req_o[t].read();
        CHECK(all_req, "T02a all m_req_o[t]=1 after fetch_addr_valid_i");
        CHECK(m_addr_o[0].read() == addrs[0], "T02b m_addr_o[0]=latched addr");
        CHECK(m_addr_o[3].read() == addrs[3], "T02c m_addr_o[3]=latched addr");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: m_req_o stable while waiting for TDM grant ===");
        // -------------------------------------------------------------------
        for (int c = 0; c < 3; ++c)
            tick();
        bool still_req = true;
        for (int t = 0; t < kNumTdm; ++t)
            still_req &= m_req_o[t].read();
        CHECK(still_req, "T03 m_req_o[*]=1 stable while gnt=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: No drain before cells VALID ===");
        // -------------------------------------------------------------------
        // Cells are REQUESTING — group not valid yet
        p_req_i[0].write(true);
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T04 p_gnt_o=0 while cells still fetching");
        p_req_i[0].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T05: fill_all_cells — cells become VALID ===");
        // -------------------------------------------------------------------
        fill_all_cells(data);
        bool all_req_deasserted = true;
        for (int t = 0; t < kNumTdm; ++t)
            all_req_deasserted &= !m_req_o[t].read();
        CHECK(all_req_deasserted, "T05 m_req_o[*]=0 after fill (cells VALID, not REQUESTING)");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: No drain when p_req_i=0 (group valid, no request) ===");
        // -------------------------------------------------------------------
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T06 p_gnt_o=0 when p_req_i=0 even though group valid");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: p_gnt_o combinatorial when group valid + all_req ===");
        // -------------------------------------------------------------------
        p_addr_i[0].write(addrs[0]); // port must present the pre-fetched address
        p_req_i[0].write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T07 p_gnt_o[0]=1 (combinatorial, same cycle as drain cond)");

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
        tick();
        CHECK(p_rvalid_o[0].read(), "T08 p_rvalid_o[0]=1 one cycle after drain (R-5 met)");
        CHECK(!p_rvalid_o[1].read(), "T08b p_rvalid_o[1]=0 (only port 0 active in mode0)");
        CHECK(p_rdata_o[0].read() == data[0], "T09 p_rdata_o[0]=data[0] (cell 0)");
        p_req_i[0].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T10: rd_ptr advances — second drain delivers cell 1's data ===");
        // -------------------------------------------------------------------
        // rd_ptr=1 now; cell[1] is VALID
        p_addr_i[0].write(addrs[1]);
        p_req_i[0].write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T10a p_gnt_o[0]=1 for group 1");
        tick();
        CHECK(p_rvalid_o[0].read(), "T10b p_rvalid_o[0]=1");
        CHECK(p_rdata_o[0].read() == data[1], "T10c p_rdata_o[0]=data[1] (cell 1)");
        p_req_i[0].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: Last-group drain — reset_window; rd_ptr→0; cells→MISSING ===");
        // -------------------------------------------------------------------
        // Drain group 2 silently to advance rd_ptr to 3
        p_addr_i[0].write(addrs[2]);
        p_req_i[0].write(true);
        tick();
        p_req_i[0].write(false);

        // Drain group 3 — this is the last group (rd_ptr+na == BUFFER_SIZE)
        p_addr_i[0].write(addrs[3]);
        p_req_i[0].write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T11a p_gnt_o[0]=1 for last group");
        tick();
        CHECK(p_rvalid_o[0].read(), "T11b p_rvalid_o[0]=1 for last drain");
        CHECK(p_rdata_o[0].read() == data[3], "T11c p_rdata_o[0]=data[3] (cell 3)");
        p_req_i[0].write(false);

        // After last drain: cells are MISSING/IDLE, rd_ptr=0, no grants available
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T11d p_gnt_o=0 (cells reset to MISSING after last drain)");

        // Re-fetch to confirm cells are back in IDLE (can latch new addresses)
        fetch_addresses(addrs);
        bool cells_refetch = true;
        for (int t = 0; t < kNumTdm; ++t)
            cells_refetch &= m_req_o[t].read();
        CHECK(cells_refetch, "T11e cells re-requesting after window reset (IDLE→REQUESTING)");

        // -------------------------------------------------------------------
        std::puts("\n=== T12: mode0 — p_gnt_o[1] always 0 (outside active count) ===");
        // -------------------------------------------------------------------
        // Bring all cells to VALID again from the current REQUESTING state
        fill_all_cells(data);
        // mode0: na=1 → only port 0 active; p_gnt_o[1] is always 0
        p_addr_i[0].write(addrs[0]);
        p_req_i[0].write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T12a p_gnt_o[0]=1 (active port)");
        CHECK(!p_gnt_o[1].read(), "T12b p_gnt_o[1]=0 (port 1 outside active count in mode0)");
        // Drain remaining groups to leave buffer clean
        tick();
        p_req_i[0].write(false);
        p_addr_i[0].write(addrs[1]);
        p_req_i[0].write(true);
        tick();
        p_req_i[0].write(false);
        p_addr_i[0].write(addrs[2]);
        p_req_i[0].write(true);
        tick();
        p_req_i[0].write(false);
        p_addr_i[0].write(addrs[3]);
        p_req_i[0].write(true);
        tick();
        p_req_i[0].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T13: mode1 — partial request blocks drain ===");
        // -------------------------------------------------------------------
        do_reset();
        active_mode.write(1); // na=2: groups [0,1] and [2,3]
        fetch_addresses(addrs);
        fill_all_cells(data);

        // Only port 0 requesting — all_req=false
        p_req_i[0].write(true);
        p_req_i[1].write(false);
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T13 p_gnt_o[0]=0 when p_req_i[1]=0 (partial req, na=2)");
        p_req_i[0].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T14: mode1 — both ports granted and respond simultaneously ===");
        // -------------------------------------------------------------------
        // Both ports request — each must present the address of its pre-fetched cell
        p_addr_i[0].write(addrs[0]);
        p_addr_i[1].write(addrs[1]);
        p_req_i[0].write(true);
        p_req_i[1].write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o[0].read(), "T14a p_gnt_o[0]=1");
        CHECK(p_gnt_o[1].read(), "T14b p_gnt_o[1]=1");
        tick();
        CHECK(p_rvalid_o[0].read(), "T14c p_rvalid_o[0]=1");
        CHECK(p_rvalid_o[1].read(), "T14d p_rvalid_o[1]=1");
        CHECK(p_rdata_o[0].read() == data[0], "T14e p_rdata_o[0]=data[0]");
        CHECK(p_rdata_o[1].read() == data[1], "T14f p_rdata_o[1]=data[1]");
        p_req_i[0].write(false);
        p_req_i[1].write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T15: Full 4-group round-trip (mode0) ===");
        // -------------------------------------------------------------------
        do_reset();
        active_mode.write(0); // na=1: 4 groups
        fetch_addresses(addrs);
        fill_all_cells(data);

        for (int g = 0; g < kNumTdm; ++g) {
            p_addr_i[0].write(addrs[g]);
            p_req_i[0].write(true);
            wait(1, SC_NS);
            CHECK(p_gnt_o[0].read(), "T15 group drain: p_gnt_o[0]=1");
            tick();
            char label[64];
            std::snprintf(label, sizeof(label), "T15 group %d: p_rvalid_o[0]=1", g);
            CHECK(p_rvalid_o[0].read(), label);
            std::snprintf(label, sizeof(label), "T15 group %d: p_rdata_o[0]=data[%d]", g, g);
            CHECK(p_rdata_o[0].read() == data[g], label);
            p_req_i[0].write(false);
        }

        // After all 4 groups drained: cells reset, rd_ptr=0
        wait(1, SC_NS);
        CHECK(!p_gnt_o[0].read(), "T15 p_gnt_o=0 after full window drain");

        // Second window: re-fetch and drain two groups to confirm rd_ptr reset
        data_t data2[kNumTdm] = {
            make_data(0x1111'0001),
            make_data(0x2222'0002),
            make_data(0x3333'0003),
            make_data(0x4444'0004),
        };
        fetch_addresses(addrs);
        fill_all_cells(data2);

        p_addr_i[0].write(addrs[0]);
        p_req_i[0].write(true);
        tick();
        CHECK(p_rdata_o[0].read() == data2[0], "T15 second window cell[0] data correct");
        p_req_i[0].write(false);

        p_addr_i[0].write(addrs[1]);
        p_req_i[0].write(true);
        tick();
        CHECK(p_rdata_o[0].read() == data2[1], "T15 second window cell[1] data correct");
        p_req_i[0].write(false);

        std::printf("\n  passed: %d\n  failed: %d\n", g_pass, g_fail);
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
