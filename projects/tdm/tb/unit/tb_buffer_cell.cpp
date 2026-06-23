// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer_cell<BYTES_PER_ROW>.
//
// Build and run:
//   make -C projects/tdm/tb/unit
//
// Tests:
//   T01      Reset: all outputs low after reset
//   T02      IDLE: m_req_o=0 before address is presented
//   T03      Address latch: en_i pulse → REQUESTING (req+addr+be asserted)
//   T04      REQUESTING: req stable across multiple cycles while gnt=0
//   T05      REQUESTING: addr not re-latched if en_i pulses again
//   T06      TDM gnt → GRANTED: m_req_o deasserts; addr/be zeroed
//   T07      TDM rvalid → VALID: data latched, valid_o=1
//   T08      gnt+rvalid same cycle: impl-defined tolerance (TDM R-5 violation)
//   T09      VALID + all_valid_i=0: AGU outputs silent
//   T10      VALID + all_valid_i=1: p_gnt_o+p_rvalid_o+p_rdata_o asserted
//            (p_rvalid_o fires same cycle as p_gnt_o; buffer registers it → R-5 met at system
//            boundary)
//   T11      p_gnt_o pre-asserted before p_req_i (OBI R-3.2.1)
//   T12      p_req_i=1 alone has no effect on p_gnt_o
//   T13      VALID → INVALID when all_valid_i=1 (next cycle); invalid_o=1
//   T14      INVALID: all AGU outputs 0
//   T15      reset_window from REQUESTING → IDLE (req deasserts)
//   T16      reset_window from VALID → MISSING/IDLE
//   T17      reset_window from INVALID → MISSING/IDLE
//   T18      reset_window beats all_valid_i in same cycle
//   T19      After reset_window: new address can be latched immediately
//   T-OBI-A  Premature TDM gnt (not in REQUESTING) is ignored
//   T-OBI-B  TDM rvalid before grant is ignored (OBI R-5 enforcement by cell)
//   T20      m_we_o always 0; m_be_o=0 outside address phase
//   T21      Full round-trip: IDLE→REQUESTING→VALID→INVALID→reset→repeat
// -----------------------------------------------------------------------------

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

// Use a compact 4-byte data word so values are easy to reason about.
static constexpr int kBytes = 4;
using data_t                = obi_data<kBytes>;
using DUT                   = buffer_cell<kBytes>;

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
    sc_signal<uint64_t> addr_i{"addr_i"};
    sc_signal<uint64_t> p_addr_s{"p_addr_s"}; // tracks last latched addr for p_addr_i cross-check
    sc_signal<bool>     en_i{"en_i"};
    sc_signal<bool>     m_gnt_i{"m_gnt_i"};
    sc_signal<bool>     m_rvalid_i{"m_rvalid_i"};
    sc_signal<data_t>   m_rdata_i{"m_rdata_i"};
    sc_signal<bool>     p_req_i{"p_req_i"};
    sc_signal<bool>     all_valid_i{"all_valid_i"};
    sc_signal<bool>     reset_window_i{"reset_window_i"};

    // DUT outputs
    sc_signal<bool>     m_req_o{"m_req_o"};
    sc_signal<uint64_t> m_addr_o{"m_addr_o"};
    sc_signal<bool>     m_we_o{"m_we_o"};
    sc_signal<uint32_t> m_be_o{"m_be_o"};
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
        dut->m_gnt_i(m_gnt_i);
        dut->m_rvalid_i(m_rvalid_i);
        dut->m_rdata_i(m_rdata_i);
        dut->p_req_i(p_req_i);
        dut->p_addr_i(p_addr_s); // tracks last latched addr; addr_i may change after latch
        dut->all_valid_i(all_valid_i);
        dut->reset_window_i(reset_window_i);
        dut->m_req_o(m_req_o);
        dut->m_addr_o(m_addr_o);
        dut->m_we_o(m_we_o);
        dut->m_be_o(m_be_o);
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

    // Assert reset and hold for n clocks, then release and settle.
    void do_reset(int cycles = 2) {
        rst_n.write(false);
        en_i.write(false);
        m_gnt_i.write(false);
        m_rvalid_i.write(false);
        m_rdata_i.write(data_t{0});
        p_req_i.write(false);
        all_valid_i.write(false);
        reset_window_i.write(false);
        addr_i.write(0);
        p_addr_s.write(0);
        for (int i = 0; i < cycles; ++i)
            wait(clk.posedge_event());
        rst_n.write(true);
        tick();
    }

    // Advance one clock and let all delta cycles from the posedge settle.
    // wait(1, SC_NS) (< half the 10 ns period) ensures every sc_signal
    // write in seq_proc has propagated through comb_proc to its outputs.
    void tick() {
        wait(clk.posedge_event());
        wait(1, SC_NS);
    }

    // Drive en_i for one clock to latch addr, then deassert.
    void latch_addr(uint64_t addr) {
        addr_i.write(addr);
        p_addr_s.write(addr); // keep cross-check signal in sync with what actually gets latched
        en_i.write(true);
        tick();
        en_i.write(false);
    }

    // Drive TDM grant for one cycle.
    void grant() {
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
    }

    // Drive TDM rvalid+data for one cycle.
    void rvalid(data_t d) {
        m_rvalid_i.write(true);
        m_rdata_i.write(d);
        tick();
        m_rvalid_i.write(false);
        m_rdata_i.write(data_t{0});
    }

    // Bring the cell from reset all the way to VALID with given addr and data.
    void bring_to_valid(uint64_t addr, data_t data) {
        latch_addr(addr);
        grant();
        rvalid(data);
    }

    // -----------------------------------------------------------------------
    // Test cases
    // -----------------------------------------------------------------------
    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all outputs low ===");
        // -------------------------------------------------------------------
        do_reset();
        CHECK(!m_req_o.read(), "T01a m_req_o=0");
        CHECK(!p_gnt_o.read(), "T01b p_gnt_o=0");
        CHECK(!p_rvalid_o.read(), "T01c p_rvalid_o=0");
        CHECK(!valid_o.read(), "T01d valid_o=0");
        CHECK(!invalid_o.read(), "T01e invalid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: IDLE — no req before address ===");
        // -------------------------------------------------------------------
        do_reset();
        // Hold idle for several cycles
        for (int i = 0; i < 4; ++i)
            tick();
        CHECK(!m_req_o.read(), "T02 m_req_o=0 while IDLE (no addr)");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: Address latch — IDLE→REQUESTING ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0xDEAD'BEEF'0000'0100ULL);
        // After tick: seq_proc latched addr, comb_proc drives req
        CHECK(m_req_o.read(), "T03a m_req_o=1");
        CHECK(m_addr_o.read() == 0xDEAD'BEEF'0000'0100ULL, "T03b m_addr_o=latched addr");
        CHECK(!m_we_o.read(), "T03c m_we_o=0 (read)");
        static constexpr uint32_t kBeFull = static_cast<uint32_t>((uint64_t{1} << kBytes) - 1u);
        CHECK(m_be_o.read() == kBeFull, "T03d m_be_o=all-ones");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: REQUESTING — req stable, no gnt ===");
        // -------------------------------------------------------------------
        for (int i = 0; i < 4; ++i) {
            tick();
            CHECK(m_req_o.read(), "T04 m_req_o stable while waiting for gnt");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T05: REQUESTING — addr not re-latched on second en_i ===");
        // -------------------------------------------------------------------
        addr_i.write(0xFFFF'FFFF'FFFF'FFFFULL);
        en_i.write(true);
        tick();
        en_i.write(false);
        CHECK(m_addr_o.read() == 0xDEAD'BEEF'0000'0100ULL,
              "T05 addr_q unchanged on second en_i (REQUESTING, not IDLE)");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: gnt → GRANTED: req deasserts ===");
        // -------------------------------------------------------------------
        grant();
        CHECK(!m_req_o.read(), "T06a m_req_o=0 after gnt");
        CHECK(m_addr_o.read() == 0, "T06b m_addr_o=0 outside address phase");
        CHECK(m_be_o.read() == 0, "T06c m_be_o=0 outside address phase");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: rvalid → VALID: data latched ===");
        // -------------------------------------------------------------------
        data_t d1{"0xCAFEBABE"};
        rvalid(d1);
        CHECK(valid_o.read(), "T07a valid_o=1");
        CHECK(!invalid_o.read(), "T07b invalid_o=0");
        CHECK(!m_req_o.read(), "T07c m_req_o=0 in VALID");

        // -------------------------------------------------------------------
        // T08: gnt + rvalid same cycle.
        //
        // OBI R-5 states the earliest rvalid can fire is the cycle AFTER
        // both req and gnt have been sampled high — so a TDM subordinate
        // asserting gnt and rvalid simultaneously is technically an R-5
        // violation.  The cell nonetheless handles it gracefully: seq_proc
        // evaluates REQUESTING→GRANTED then GRANTED→VALID in the same pass.
        // The test documents this implementation-defined tolerance, not a
        // required protocol behaviour.
        // -------------------------------------------------------------------
        std::puts("\n=== T08: gnt+rvalid coincide (impl-defined; TDM R-5 violation) ===");
        do_reset();
        latch_addr(0x1000);
        m_gnt_i.write(true);
        m_rvalid_i.write(true);
        m_rdata_i.write(data_t{"0xABCD1234"});
        tick();
        m_gnt_i.write(false);
        m_rvalid_i.write(false);
        m_rdata_i.write(data_t{0});
        CHECK(valid_o.read(), "T08 cell reaches VALID despite illegal same-cycle gnt+rvalid");

        // -------------------------------------------------------------------
        std::puts("\n=== T09: VALID + all_valid_i=0 — AGU outputs silent ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x2000, data_t{"0x12345678"});
        // all_valid_i is 0 by default
        CHECK(!p_gnt_o.read(), "T09a p_gnt_o=0 when all_valid_i=0");
        CHECK(!p_rvalid_o.read(), "T09b p_rvalid_o=0 when all_valid_i=0");
        CHECK(p_rdata_o.read() == data_t{0}, "T09c p_rdata_o=0 when all_valid_i=0");

        // -------------------------------------------------------------------
        // T10: p_gnt_o and p_rvalid_o both fire when window_ready.
        //
        // For the port-facing interface (cell is subordinate) OBI R-5 also
        // applies: rvalid should only appear in the cycle after req+gnt are
        // sampled.  The cell asserts p_rvalid_o combinatorially in the same
        // cycle as p_gnt_o, which would violate R-5 in isolation.  This is
        // intentional: the parent buffer seq_proc re-registers p_rvalid_o /
        // p_rdata_o before forwarding them to the AGU, satisfying R-5 at
        // the system boundary.  Here we verify the cell's internal behaviour.
        // -------------------------------------------------------------------
        std::puts("\n=== T10: VALID + all_valid_i=1 — AGU R-channel asserted ===");
        data_t d2{"0x12345678"};
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(p_gnt_o.read(), "T10a p_gnt_o=1");
        CHECK(p_rvalid_o.read(), "T10b p_rvalid_o=1 (same cycle as gnt; buffer will register)");
        CHECK(p_rdata_o.read() == d2, "T10c p_rdata_o=expected data");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T11: p_gnt_o pre-asserted before p_req_i (OBI R-3.2.1) ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        p_req_i.write(false);
        wait(1, SC_NS);
        CHECK(p_gnt_o.read(), "T11 p_gnt_o=1 even when p_req_i=0");
        all_valid_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T12: p_req_i alone has no effect on p_gnt_o ===");
        // -------------------------------------------------------------------
        p_req_i.write(true);
        all_valid_i.write(false);
        wait(1, SC_NS);
        CHECK(!p_gnt_o.read(), "T12 p_gnt_o=0 when all_valid_i=0 regardless of p_req_i");
        p_req_i.write(false);

        // -------------------------------------------------------------------
        std::puts("\n=== T13: VALID → INVALID when all_valid_i=1 ===");
        // -------------------------------------------------------------------
        all_valid_i.write(true);
        tick(); // posedge: seq_proc sees all_valid_i=1 while VALID → INVALID
        all_valid_i.write(false);
        CHECK(invalid_o.read(), "T13a invalid_o=1");
        CHECK(!valid_o.read(), "T13b valid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T14: INVALID — AGU outputs 0 ===");
        // -------------------------------------------------------------------
        CHECK(!p_gnt_o.read(), "T14a p_gnt_o=0 in INVALID");
        CHECK(!p_rvalid_o.read(), "T14b p_rvalid_o=0 in INVALID");
        CHECK(p_rdata_o.read() == data_t{0}, "T14c p_rdata_o=0 in INVALID");

        // -------------------------------------------------------------------
        std::puts("\n=== T15: reset_window from REQUESTING — req deasserts ===");
        // -------------------------------------------------------------------
        do_reset();
        latch_addr(0x4000);
        CHECK(m_req_o.read(), "T15 pre: req active in REQUESTING");
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!m_req_o.read(), "T15a m_req_o=0 after reset_window");
        CHECK(!valid_o.read(), "T15b valid_o=0 after reset_window");
        CHECK(!invalid_o.read(), "T15c invalid_o=0 after reset_window");

        // -------------------------------------------------------------------
        std::puts("\n=== T16: reset_window from VALID ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x8000, data_t{"0x0000DEAD"});
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!valid_o.read(), "T16a valid_o=0 after reset_window from VALID");
        CHECK(!p_gnt_o.read(), "T16b p_gnt_o=0");
        CHECK(!m_req_o.read(), "T16c m_req_o=0 (IDLE, no address yet)");

        // -------------------------------------------------------------------
        std::puts("\n=== T17: reset_window from INVALID ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0x9000, data_t{"0x00001111"});
        all_valid_i.write(true);
        tick();
        all_valid_i.write(false);
        // Now INVALID
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!invalid_o.read(), "T17a invalid_o=0 after reset_window from INVALID");
        CHECK(!valid_o.read(), "T17b valid_o=0");

        // -------------------------------------------------------------------
        std::puts("\n=== T18: reset_window beats all_valid_i same cycle ===");
        // -------------------------------------------------------------------
        do_reset();
        bring_to_valid(0xA000, data_t{"0x00002222"});
        // Both asserted simultaneously
        all_valid_i.write(true);
        reset_window_i.write(true);
        tick();
        all_valid_i.write(false);
        reset_window_i.write(false);
        // reset_window is in the outer if; all_valid_i check is in the else
        CHECK(!invalid_o.read(), "T18a invalid_o=0 (reset wins over all_valid_i)");
        CHECK(!valid_o.read(), "T18b valid_o=0 (went to MISSING, not INVALID)");

        // -------------------------------------------------------------------
        std::puts("\n=== T19: After reset_window, new address can be latched ===");
        // -------------------------------------------------------------------
        // Still in MISSING/IDLE from T18
        latch_addr(0xB000);
        CHECK(m_req_o.read(), "T19a req asserted after reset_window");
        CHECK(m_addr_o.read() == 0xB000, "T19b new addr latched");

        // -------------------------------------------------------------------
        // T-OBI-A: premature TDM gnt (arrives while cell is not REQUESTING).
        //
        // OBI R-3.2.1 allows a subordinate to assert gnt before req, but
        // the cell must not change state on early gnt: only a gnt received
        // while in REQUESTING phase is meaningful.  Test: gnt in IDLE and
        // in GRANTED states → cell stays put, req not driven spuriously.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-A: premature TDM gnt ignored (not in REQUESTING) ===");
        do_reset();
        // IDLE: gnt arrives before address is latched
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
        CHECK(!m_req_o.read(), "T-OBI-Aa gnt in IDLE: req still 0 (no addr)");
        CHECK(!valid_o.read(), "T-OBI-Ab gnt in IDLE: state still MISSING");
        // Latch addr → REQUESTING; grant correctly; land in GRANTED
        latch_addr(0xD000);
        grant();
        CHECK(!m_req_o.read(), "T-OBI-Ac in GRANTED after correct grant");
        // Second gnt while already GRANTED (req is deasserted)
        m_gnt_i.write(true);
        tick();
        m_gnt_i.write(false);
        CHECK(!valid_o.read(), "T-OBI-Ad second gnt in GRANTED: still waiting for rvalid");
        // Complete legitimately
        rvalid(data_t{"0x0000EEEE"});
        CHECK(valid_o.read(), "T-OBI-Ae VALID after legitimate rvalid");

        // -------------------------------------------------------------------
        // T-OBI-B: TDM rvalid before grant violates OBI R-5.
        //
        // "rvalid shall not start before req and gnt have been sampled high."
        // The cell enforces R-5 by only transitioning MISSING→VALID in the
        // GRANTED fetch phase.  A rvalid arriving while still REQUESTING
        // must be ignored — the cell stays in REQUESTING and keeps req high.
        // -------------------------------------------------------------------
        std::puts("\n=== T-OBI-B: TDM rvalid before grant ignored (OBI R-5 enforcement) ===");
        do_reset();
        latch_addr(0xE000);
        // Premature rvalid while still in REQUESTING (no grant yet)
        m_rvalid_i.write(true);
        m_rdata_i.write(data_t{"0x0000BAD0"});
        tick();
        m_rvalid_i.write(false);
        m_rdata_i.write(data_t{0});
        CHECK(m_req_o.read(), "T-OBI-Ba req still asserted (not granted yet)");
        CHECK(!valid_o.read(), "T-OBI-Bb still not VALID (premature rvalid ignored)");
        // Now grant legitimately and complete
        grant();
        rvalid(data_t{"0x0000600D"});
        CHECK(valid_o.read(), "T-OBI-Bc VALID after correct grant+rvalid sequence");

        // -------------------------------------------------------------------
        std::puts("\n=== T20: m_we_o always 0; m_be_o=0 outside address phase ===");
        // -------------------------------------------------------------------
        do_reset();
        // In IDLE: req=0 → be=0, we=0
        CHECK(!m_we_o.read(), "T20a m_we_o=0 in IDLE");
        CHECK(m_be_o.read() == 0, "T20b m_be_o=0 in IDLE");
        latch_addr(0xC000);
        CHECK(!m_we_o.read(), "T20c m_we_o=0 in REQUESTING");
        CHECK(m_be_o.read() == kBeFull, "T20d m_be_o=all-ones in REQUESTING");
        grant();
        CHECK(!m_we_o.read(), "T20e m_we_o=0 in GRANTED");
        CHECK(m_be_o.read() == 0, "T20f m_be_o=0 in GRANTED");

        // -------------------------------------------------------------------
        std::puts("\n=== T21: Full round-trip (two consecutive fetches) ===");
        // -------------------------------------------------------------------
        do_reset();
        // First fetch
        data_t rd1{"0xAAAAAAAA"};
        bring_to_valid(0x1000, rd1);
        CHECK(valid_o.read(), "T21a first fetch VALID");
        all_valid_i.write(true);
        tick();
        all_valid_i.write(false);
        CHECK(invalid_o.read(), "T21b INVALID after drain");
        // Reset window (as buffer would do)
        reset_window_i.write(true);
        tick();
        reset_window_i.write(false);
        CHECK(!invalid_o.read() && !valid_o.read(), "T21c back to MISSING");
        // Second fetch with different data
        data_t rd2{"0xBBBBBBBB"};
        bring_to_valid(0x2000, rd2);
        all_valid_i.write(true);
        wait(1, SC_NS);
        CHECK(p_rvalid_o.read(), "T21d second fetch: p_rvalid_o=1");
        CHECK(p_rdata_o.read() == rd2, "T21e second fetch: correct data");
        all_valid_i.write(false);

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
