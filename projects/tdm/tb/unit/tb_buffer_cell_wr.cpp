// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer_cell<BYTES_PER_ROW, IS_WRITE=true>.
//
// Build and run:
//   make unit-test PROJECT=tdm TOP_LEVEL=buffer_cell_wr
//
// Write-mode FSM (new accumulate-then-flush design):
//   IDLE   → p_req_i          → LATCHED    (data latched; TDM write NOT started)
//   LATCHED → p_req_i (pulse) → REQUESTING (parent re-asserts p_req_i during FLUSH)
//   REQUESTING → m_gnt_i → GRANTED
//   GRANTED  → m_rvalid_i → VALID
//
// Tests:
//   T01   Reset — all outputs low
//   T02   IDLE — no p_req → m_req=0, m_we=0
//   T03   p_req → LATCHED: data latched; m_req stays 0 (TDM not yet started)
//   T04   LATCHED: stable while p_req=0; data not re-latched on port change
//   T05   p_req re-pulse → REQUESTING: m_req=1, m_we=1, m_addr/m_wdata/m_be driven
//   T06   REQUESTING: TDM signals stable while gnt=0
//   T07   REQUESTING: wdata/addr not re-latched when port changes them mid-flight
//   T08   gnt → GRANTED: m_req deasserts; m_addr/m_be/m_wdata zeroed
//   T09   TDM rvalid (write ack) → VALID: valid_o=1
//   T10   gnt+rvalid same cycle: impl-defined tolerance (TDM R-5 violation)
//   T11   VALID + all_valid_i=0: p_gnt/p_rvalid low; p_rdata always 0
//   T12   VALID + all_valid_i=1: p_gnt/p_rvalid assert; p_rdata stays 0
//   T13   VALID → INVALID when all_valid_i=1; invalid_o=1
//   T14   reset_window from LATCHED — returns to MISSING (m_req stays 0)
//   T15   reset_window from REQUESTING — m_req deasserts
//   T16   reset_window from VALID — valid_o clears
//   T17   reset_window from INVALID — invalid_o clears
//   T18   reset_window beats all_valid_i in same cycle
//   T19   After reset_window: new write can be latched immediately
//   T-OBI-A  Premature TDM gnt ignored (not in REQUESTING)
//   T-OBI-B  TDM rvalid before grant ignored (OBI R-5 enforcement)
//   T20   Batch 1: 32 sequential write transactions (unique addr/wdata/be)
//   T21   Batch 2: 32 more write transactions — confirms unlimited cell reuse
// -----------------------------------------------------------------------------

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include <cstdio>
#include <systemc.h>

static constexpr int      kBytes  = 4;
static constexpr uint32_t kBeFull = static_cast<uint32_t>((uint64_t{1} << kBytes) - 1u); // 0xF
using data_t                      = obi_data<kBytes>;
using DUT                         = buffer_cell<kBytes, true>;

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

// ---------------------------------------------------------------------------
// Testbench module
// ---------------------------------------------------------------------------
SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // DUT inputs
    sc_signal<uint64_t> p_addr_i{"p_addr_i"};
    sc_signal<data_t>   p_wdata_i{"p_wdata_i"};
    sc_signal<uint32_t> p_be_i{"p_be_i"};
    sc_signal<bool>     p_req_i{"p_req_i"};
    sc_signal<bool>     m_gnt_i{"m_gnt_i"};
    sc_signal<bool>     m_rvalid_i{"m_rvalid_i"};
    sc_signal<data_t>   m_rdata_i{"m_rdata_i"}; // always 0 — write mode returns no data
    sc_signal<bool>     all_valid_i{"all_valid_i"};
    sc_signal<bool>     reset_window_i{"reset_window_i"};

    // Unused read-mode inputs (held at 0)
    sc_signal<uint64_t> addr_i{"addr_i"};
    sc_signal<bool>     en_i{"en_i"};

    // DUT outputs
    sc_signal<bool>     m_req_o{"m_req_o"};
    sc_signal<uint64_t> m_addr_o{"m_addr_o"};
    sc_signal<bool>     m_we_o{"m_we_o"};
    sc_signal<uint32_t> m_be_o{"m_be_o"};
    sc_signal<data_t>   m_wdata_o{"m_wdata_o"};
    sc_signal<bool>     p_gnt_o{"p_gnt_o"};
    sc_signal<bool>     p_rvalid_o{"p_rvalid_o"};
    sc_signal<data_t>   p_rdata_o{"p_rdata_o"};
    sc_signal<bool>     valid_o{"valid_o"};
    sc_signal<bool>     invalid_o{"invalid_o"};

    DUT *dut;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->addr_i(addr_i);
        dut->en_i(en_i);
        dut->p_req_i(p_req_i);
        dut->p_addr_i(p_addr_i);
        dut->p_wdata_i(p_wdata_i);
        dut->p_be_i(p_be_i);
        dut->m_gnt_i(m_gnt_i);
        dut->m_rvalid_i(m_rvalid_i);
        dut->m_rdata_i(m_rdata_i);
        dut->all_valid_i(all_valid_i);
        dut->reset_window_i(reset_window_i);
        dut->m_req_o(m_req_o);
        dut->m_addr_o(m_addr_o);
        dut->m_we_o(m_we_o);
        dut->m_be_o(m_be_o);
        dut->m_wdata_o(m_wdata_o);
        dut->p_gnt_o(p_gnt_o);
        dut->p_rvalid_o(p_rvalid_o);
        dut->p_rdata_o(p_rdata_o);
        dut->valid_o(valid_o);
        dut->invalid_o(invalid_o);

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

    void do_reset(int cycles = 2) {
        rst_n.write(false);
        p_req_i.write(false);
        p_addr_i.write(0);
        p_wdata_i.write(data_t{0});
        p_be_i.write(0);
        m_gnt_i.write(false);
        m_rvalid_i.write(false);
        all_valid_i.write(false);
        reset_window_i.write(false);
        for (int i = 0; i < cycles; ++i)
            wait(clk.posedge_event());
        rst_n.write(true);
        tick();
    }

    // Present a write request; cell latches addr/wdata/be on the next posedge
    // and enters LATCHED (does NOT start TDM write yet).
    void issue_req(uint64_t addr, data_t wdata, uint32_t be = kBeFull) {
        p_req_i.write(true);
        p_addr_i.write(addr);
        p_wdata_i.write(wdata);
        p_be_i.write(be);
    }

    // Re-assert p_req_i for one cycle: LATCHED → REQUESTING.
    // (Mirrors what the parent buffer does during its FLUSH phase.)
    void do_flush() {
        p_req_i.write(true);
        tick();
        p_req_i.write(false);
    }

    void grant() {
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
    }

    void ack() { // TDM write response (rvalid, no data)
        m_rvalid_i.write(true);
        tick();
        m_rvalid_i.write(false);
    }

    // Full write transaction: IDLE → LATCHED → REQUESTING → GRANTED → VALID.
    // Returns after valid_o=1; does NOT drain or reset.
    void bring_to_valid(uint64_t addr, data_t wdata, uint32_t be = kBeFull) {
        issue_req(addr, wdata, be);
        tick();               // → LATCHED (data captured; m_req still 0)
        p_req_i.write(false); // deassert so cell stays LATCHED until explicit flush
        do_flush();           // re-pulse p_req_i → REQUESTING
        grant();
        ack();
    }

    // -----------------------------------------------------------------------
    // Test thread
    // -----------------------------------------------------------------------
    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all outputs low ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(!m_req_o.read(), "T01a m_req_o=0");
        CHECK(!m_we_o.read(), "T01b m_we_o=0");
        CHECK(!p_gnt_o.read(), "T01c p_gnt_o=0");
        CHECK(!p_rvalid_o.read(), "T01d p_rvalid_o=0");
        CHECK(!valid_o.read(), "T01e valid_o=0");
        CHECK(!invalid_o.read(), "T01f invalid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: IDLE — no p_req → m_req=0, m_we=0 ===");
        // -------------------------------------------------------------------
        for (int i = 0; i < 4; ++i)
            tick();
        CHECK(!m_req_o.read(), "T02a m_req_o=0 while IDLE (no port request)");
        CHECK(!m_we_o.read(), "T02b m_we_o=0 while IDLE");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: p_req → LATCHED: data latched; TDM write not started ===");
        // -------------------------------------------------------------------
        // Cell captures addr/wdata/be but stays silent on TDM (m_req=0)
        // until the parent buffer re-pulses p_req_i.
        do_reset();
        const data_t wd3 = data_t(static_cast<unsigned long long>(0xDEADBEEFU));
        issue_req(0xDEAD0000ULL, wd3, 0xA);
        tick();               // → LATCHED
        p_req_i.write(false); // deassert so cell stays LATCHED
        CHECK(!m_req_o.read(), "T03a m_req_o=0 in LATCHED (TDM write not yet started)");
        CHECK(!m_we_o.read(), "T03b m_we_o=0 in LATCHED");
        CHECK(!valid_o.read(), "T03c valid_o=0 in LATCHED");
        CHECK(p_rdata_o.read() == data_t{0}, "T03d p_rdata_o=0 (write carries no rdata)");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: LATCHED — stable while p_req=0; data not re-latched ===");
        // -------------------------------------------------------------------
        // Change port signals while in LATCHED — cell must keep latched values.
        p_wdata_i.write(data_t(static_cast<unsigned long long>(0xFFFFFFFFU)));
        p_addr_i.write(0xFFFFFFFFFFFFFFFFULL);
        for (int i = 0; i < 3; ++i) {
            tick();
            CHECK(!m_req_o.read(), "T04a m_req_o=0 stays low in LATCHED (p_req=0)");
        }
        // After flush: REQUESTING with original latched values (not the changed port values)
        do_flush(); // → REQUESTING
        CHECK(m_req_o.read(), "T04b m_req_o=1 after flush");
        CHECK(m_addr_o.read() == 0xDEAD0000ULL,
              "T04c m_addr_o=original latched addr (not port change)");
        CHECK(m_wdata_o.read() == wd3, "T04d m_wdata_o=original latched wdata");
        CHECK(m_be_o.read() == 0xAu, "T04e m_be_o=original latched be");
        // Clean up
        grant();
        ack();
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T05: p_req re-pulse → REQUESTING: TDM write signals driven ===");
        // -------------------------------------------------------------------
        do_reset();
        const data_t wd5 = data_t(static_cast<unsigned long long>(0xCAFEBABEU));
        issue_req(0xBEEF0000ULL, wd5, 0x5);
        tick();               // → LATCHED
        p_req_i.write(false); // deassert; stay LATCHED
        do_flush();           // re-pulse → REQUESTING
        CHECK(m_req_o.read(), "T05a m_req_o=1 in REQUESTING");
        CHECK(m_we_o.read(), "T05b m_we_o=1 (write)");
        CHECK(m_addr_o.read() == 0xBEEF0000ULL, "T05c m_addr_o=latched addr");
        CHECK(m_wdata_o.read() == wd5, "T05d m_wdata_o=latched wdata");
        CHECK(m_be_o.read() == 0x5u, "T05e m_be_o=latched be");
        grant();
        ack();
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T06: REQUESTING — TDM signals stable while gnt=0 ===");
        // -------------------------------------------------------------------
        do_reset();
        const data_t wd6 = data_t(static_cast<unsigned long long>(0x11223344U));
        issue_req(0xAAAA0000ULL, wd6);
        tick();     // LATCHED
        do_flush(); // REQUESTING
        for (int i = 0; i < 4; ++i) {
            tick();
            CHECK(m_req_o.read(), "T06 m_req_o stable while waiting for gnt");
            CHECK(m_we_o.read(), "T06 m_we_o stable while waiting for gnt");
            CHECK(m_wdata_o.read() == wd6, "T06 m_wdata_o stable");
        }
        grant();
        ack();
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T07: REQUESTING — wdata/addr not re-latched on port change ===");
        // -------------------------------------------------------------------
        do_reset();
        const data_t wd7 = data_t(static_cast<unsigned long long>(0xDEADBEEFU));
        issue_req(0xDEAD0000ULL, wd7, 0xA);
        tick();     // LATCHED
        do_flush(); // REQUESTING
        // Change port signals while REQUESTING — latched values must be kept
        p_wdata_i.write(data_t(static_cast<unsigned long long>(0xFFFFFFFFU)));
        p_addr_i.write(0xFFFFFFFFFFFFFFFFULL);
        tick();
        CHECK(m_wdata_o.read() == wd7, "T07a m_wdata_o unchanged after port change");
        CHECK(m_addr_o.read() == 0xDEAD0000ULL, "T07b m_addr_o unchanged after port change");
        grant();
        ack();
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T08: gnt → GRANTED: m_req deasserts; TDM A-channel zeroed ===");
        // -------------------------------------------------------------------
        do_reset();
        issue_req(0xDEAD0000ULL, wd3, 0xA);
        tick();
        do_flush();
        grant(); // also does tick()
        CHECK(!m_req_o.read(), "T08a m_req_o=0 after gnt");
        CHECK(!m_we_o.read(), "T08b m_we_o=0 after gnt");
        CHECK(m_addr_o.read() == 0, "T08c m_addr_o=0 outside address phase");
        CHECK(m_be_o.read() == 0, "T08d m_be_o=0 outside address phase");
        CHECK(m_wdata_o.read() == data_t{0}, "T08e m_wdata_o=0 outside address phase");
        ack();
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T09: TDM rvalid (write ack) → VALID: valid_o=1 ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x1000, data_t(static_cast<unsigned long long>(0x12340000U)));
        CHECK(valid_o.read(), "T09a valid_o=1 after write ack");
        CHECK(!invalid_o.read(), "T09b invalid_o=0");
        CHECK(!m_req_o.read(), "T09c m_req_o=0 in VALID");
        CHECK(!m_we_o.read(), "T09d m_we_o=0 in VALID");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        // T10: gnt + rvalid same cycle (TDM R-5 violation; impl-defined).
        // -------------------------------------------------------------------
        std::puts("\n=== T10: gnt+rvalid coincide (impl-defined; TDM R-5 violation) ===");
        do_reset();
        issue_req(0x1000, data_t(static_cast<unsigned long long>(0x12345678U)));
        tick();
        do_flush();
        m_gnt_i.write(true);
        m_rvalid_i.write(true);
        tick();
        m_gnt_i.write(false);
        m_rvalid_i.write(false);
        CHECK(valid_o.read(), "T10 cell reaches VALID despite same-cycle gnt+rvalid");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: VALID + all_valid_i=0 — AGU outputs silent ===");
        // -------------------------------------------------------------------
        // (still VALID from T10)
        CHECK(!p_gnt_o.read(), "T11a p_gnt_o=0 when all_valid_i=0");
        CHECK(!p_rvalid_o.read(), "T11b p_rvalid_o=0 when all_valid_i=0");
        CHECK(p_rdata_o.read() == data_t{0}, "T11c p_rdata_o=0 always (write cell)");

        // -------------------------------------------------------------------
        std::puts("\n=== T12: VALID + all_valid_i=1 — p_gnt/p_rvalid assert; p_rdata stays 0 ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o.read(), "T12a p_gnt_o=1");
        CHECK(p_rvalid_o.read(), "T12b p_rvalid_o=1 (write ack forwarded)");
        CHECK(p_rdata_o.read() == data_t{0}, "T12c p_rdata_o=0 (write carries no read data)");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T13: VALID → INVALID when all_valid_i=1 ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        tick();
        all_valid_i.write(false);
        CHECK(invalid_o.read(), "T13a invalid_o=1");
        CHECK(!valid_o.read(), "T13b valid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T14: reset_window from LATCHED — returns to MISSING ===");
        // -------------------------------------------------------------------
        do_reset();
        issue_req(0x4000, data_t(static_cast<unsigned long long>(0xCAFEBABEU)));
        tick();               // → LATCHED
        p_req_i.write(false); // deassert so cell stays LATCHED
        CHECK(!m_req_o.read(), "T14 pre: m_req=0 in LATCHED");
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!m_req_o.read(), "T14a m_req_o=0 after reset_window (was LATCHED)");
        CHECK(!valid_o.read(), "T14b valid_o=0");
        CHECK(!invalid_o.read(), "T14c invalid_o=0 (back to MISSING)");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T15: reset_window from REQUESTING — m_req deasserts ===");
        // -------------------------------------------------------------------
        do_reset();
        issue_req(0x4000, data_t(static_cast<unsigned long long>(0xCAFEBABEU)));
        tick();     // LATCHED
        do_flush(); // REQUESTING
        CHECK(m_req_o.read(), "T15 pre: req active in REQUESTING");
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!m_req_o.read(), "T15a m_req_o=0 after reset_window");
        CHECK(!m_we_o.read(), "T15b m_we_o=0 after reset_window");
        CHECK(!valid_o.read(), "T15c valid_o=0");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T16: reset_window from VALID — valid_o clears ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x8000, data_t(static_cast<unsigned long long>(0x0000DEADU)));
        CHECK(valid_o.read(), "T16 pre: VALID before reset");
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!valid_o.read(), "T16a valid_o=0 after reset_window from VALID");
        CHECK(!p_gnt_o.read(), "T16b p_gnt_o=0");
        CHECK(!m_req_o.read(), "T16c m_req_o=0 (IDLE, no pending request)");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T17: reset_window from INVALID — invalid_o clears ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x9000, data_t(static_cast<unsigned long long>(0x00001111U)));
        all_valid_i.write(true);
        tick();
        all_valid_i.write(false);
        CHECK(invalid_o.read(), "T17 pre: INVALID before reset");
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!invalid_o.read(), "T17a invalid_o=0 after reset_window from INVALID");
        CHECK(!valid_o.read(), "T17b valid_o=0");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T18: reset_window beats all_valid_i same cycle ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0xA000, data_t(static_cast<unsigned long long>(0x00002222U)));
        all_valid_i.write(true);
        reset_window_i.write(true);
        tick();
        all_valid_i.write(false);
        reset_window_i.write(false);
        CHECK(!invalid_o.read(), "T18a invalid_o=0 (reset wins over all_valid_i)");
        CHECK(!valid_o.read(), "T18b valid_o=0 (went to MISSING, not INVALID)");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T19: After reset_window — new write can be latched immediately ===");
        // -------------------------------------------------------------------
        // Still in MISSING/IDLE from T18
        const data_t wd19 = data_t(static_cast<unsigned long long>(0xBEEFBEEFU));
        issue_req(0xB000, wd19);
        tick(); // LATCHED
        CHECK(!m_req_o.read(), "T19a m_req_o=0 in LATCHED after reset_window");
        do_flush(); // REQUESTING
        CHECK(m_req_o.read(), "T19b m_req_o=1 after flush");
        CHECK(m_we_o.read(), "T19c m_we_o=1");
        CHECK(m_addr_o.read() == 0xB000, "T19d new addr latched");
        CHECK(m_wdata_o.read() == wd19, "T19e new wdata latched");
        p_req_i.write(false);
        grant();
        ack();

        // -------------------------------------------------------------------
        // T-OBI-A: premature TDM gnt (not in REQUESTING) is ignored.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-A: premature TDM gnt ignored ===");
        do_reset();
        // gnt in IDLE (no pending request)
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
        CHECK(!m_req_o.read(), "T-OBI-Aa gnt in IDLE: req still 0");
        CHECK(!valid_o.read(), "T-OBI-Ab gnt in IDLE: state still MISSING");
        // gnt in LATCHED (before flush)
        issue_req(0xD000, data_t(static_cast<unsigned long long>(0xABCDABCDU)));
        tick();               // → LATCHED
        p_req_i.write(false); // deassert so cell stays LATCHED
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
        CHECK(!m_req_o.read(), "T-OBI-Ac gnt in LATCHED: req still 0");
        CHECK(!valid_o.read(), "T-OBI-Ad gnt in LATCHED: still LATCHED");
        // Normal flush → REQUESTING → gnt → GRANTED; then a second gnt while GRANTED
        do_flush(); // REQUESTING
        grant();    // → GRANTED
        CHECK(!m_req_o.read(), "T-OBI-Ae in GRANTED after correct grant");
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
        CHECK(!valid_o.read(), "T-OBI-Af second gnt in GRANTED: still waiting for ack");
        ack();
        CHECK(valid_o.read(), "T-OBI-Ag VALID after legitimate ack");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        // T-OBI-B: TDM rvalid before grant violates OBI R-5; cell ignores it.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-B: TDM rvalid before grant ignored (OBI R-5) ===");
        do_reset();
        issue_req(0xE000, data_t(static_cast<unsigned long long>(0xBAD0BAD0U)));
        tick();     // LATCHED
        do_flush(); // REQUESTING
        // Premature rvalid while still REQUESTING (not granted yet)
        m_rvalid_i.write(true);
        tick();
        m_rvalid_i.write(false);
        CHECK(m_req_o.read(), "T-OBI-Ba req still asserted (not granted yet)");
        CHECK(!valid_o.read(), "T-OBI-Bb still not VALID (premature rvalid ignored)");
        grant();
        ack();
        CHECK(valid_o.read(), "T-OBI-Bc VALID after correct grant+ack sequence");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T20: Batch 1 — 32 sequential write transactions ===");
        // -------------------------------------------------------------------
        do_reset();

        for (int i = 0; i < 32; ++i) {
            uint64_t addr = static_cast<uint64_t>(i) * 0x10;
            data_t   wdata =
                data_t(static_cast<unsigned long long>(0xA0000000u | static_cast<unsigned>(i)));
            uint32_t be = kBeFull;
            char     lbl[80];

            // Latch into cell
            issue_req(addr, wdata, be);
            tick();               // → LATCHED
            p_req_i.write(false); // deassert so cell stays LATCHED

            std::snprintf(lbl, sizeof(lbl), "T20[%02d] m_req=0 in LATCHED (before flush)", i);
            CHECK(!m_req_o.read(), lbl);

            // Flush → REQUESTING
            do_flush();

            bool tdm_ok = m_req_o.read() && m_we_o.read() && (m_addr_o.read() == addr) &&
                          (m_wdata_o.read() == wdata) && (m_be_o.read() == be);
            std::snprintf(lbl, sizeof(lbl), "T20[%02d] TDM: m_req/we/addr/wdata/be correct", i);
            CHECK(tdm_ok, lbl);
            std::snprintf(lbl, sizeof(lbl), "T20[%02d] p_rdata_o=0 (write)", i);
            CHECK(p_rdata_o.read() == data_t{0}, lbl);

            grant();
            ack();

            std::snprintf(lbl, sizeof(lbl), "T20[%02d] valid_o=1 after write ack", i);
            CHECK(valid_o.read(), lbl);

            // Drain: VALID → INVALID
            all_valid_i.write(true);
            tick();
            all_valid_i.write(false);
            std::snprintf(lbl, sizeof(lbl), "T20[%02d] invalid_o=1 after drain", i);
            CHECK(invalid_o.read(), lbl);

            // Reset window: INVALID → MISSING
            reset_window_i.write(true);
            tick();
            reset_window_i.write(false);
            bool is_missing = !valid_o.read() && !invalid_o.read();
            std::snprintf(lbl, sizeof(lbl), "T20[%02d] back to MISSING after reset", i);
            CHECK(is_missing, lbl);

            p_req_i.write(false);
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T21: Batch 2 — 32 more write transactions (confirm reuse) ===");
        // -------------------------------------------------------------------
        for (int i = 0; i < 32; ++i) {
            uint64_t addr = 0x2000 + static_cast<uint64_t>(i) * 0x10;
            data_t   wdata =
                data_t(static_cast<unsigned long long>(0xB0000000u | static_cast<unsigned>(i)));
            uint32_t be = static_cast<uint32_t>(0x1u << (i % 4));
            char     lbl[80];

            issue_req(addr, wdata, be);
            tick(); // → LATCHED
            p_req_i.write(false);
            do_flush(); // → REQUESTING

            bool tdm_ok = m_req_o.read() && m_we_o.read() && (m_addr_o.read() == addr) &&
                          (m_wdata_o.read() == wdata) && (m_be_o.read() == be);
            std::snprintf(lbl, sizeof(lbl), "T21[%02d] TDM: m_req/we/addr/wdata/be correct", i);
            CHECK(tdm_ok, lbl);

            grant();
            ack();

            std::snprintf(lbl, sizeof(lbl), "T21[%02d] valid_o=1 after write ack", i);
            CHECK(valid_o.read(), lbl);

            all_valid_i.write(true);
            tick();
            all_valid_i.write(false);
            std::snprintf(lbl, sizeof(lbl), "T21[%02d] invalid_o=1 after drain", i);
            CHECK(invalid_o.read(), lbl);

            reset_window_i.write(true);
            tick();
            reset_window_i.write(false);
            bool is_missing = !valid_o.read() && !invalid_o.read();
            std::snprintf(lbl, sizeof(lbl), "T21[%02d] back to MISSING after reset", i);
            CHECK(is_missing, lbl);

            p_req_i.write(false);
        }

        // -------------------------------------------------------------------
        std::puts("\n=== Summary ===");
        // -------------------------------------------------------------------
        std::printf("  passed: %d\n", g_pass);
        std::printf("  failed: %d\n", g_fail);

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
