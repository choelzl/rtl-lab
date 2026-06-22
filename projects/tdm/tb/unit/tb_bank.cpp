// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// Unit tests for bank<NUM_ROW, BYTES_PER_WORD>.
//
// Build and run:
//   make unit-test PROJECT=tdm TOP_LEVEL=bank
//
// Tests:
//   T01  Reset: rvalid_o=0, rdata_o=0 while rst_ni=0
//   T02  gnt_o is combinatorial: follows req_i without a clock edge
//   T03  gnt_o=0 when req_i=0 (combinatorial)
//   T04  Read zero-initialised memory returns 0
//   T05  rvalid_o=0 on the cycle after no request
//   T06  Write response: rvalid=1, rdata=0 (write does not return data)
//   T07  Write then read: round-trip data integrity
//   T08  Byte-enable partial write: only selected bytes are modified
//   T09  Multiple rows are independent (no aliasing)
//   T10  Back-to-back reads: two consecutive read cycles return correct data
//   T11  Overwrite: second write to same row takes effect
//   T12  Reset mid-operation: rvalid deasserts immediately
// -----------------------------------------------------------------------------

#include "bank.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

static constexpr int kNumRows   = 8;
static constexpr int kBytesWord = 4;
using DUT                       = bank<kNumRows, kBytesWord>;

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

    sc_signal<bool>     req_i{"req_i"};
    sc_signal<uint64_t> addr_i{"addr_i"};
    sc_signal<bool>     we_i{"we_i"};
    sc_signal<uint32_t> be_i{"be_i"};
    sc_signal<uint64_t> wdata_i{"wdata_i"};
    sc_signal<bool>     gnt_o{"gnt_o"};
    sc_signal<bool>     rvalid_o{"rvalid_o"};
    sc_signal<uint64_t> rdata_o{"rdata_o"};

    DUT *dut;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->req_i(req_i);
        dut->addr_i(addr_i);
        dut->we_i(we_i);
        dut->be_i(be_i);
        dut->wdata_i(wdata_i);
        dut->gnt_o(gnt_o);
        dut->rvalid_o(rvalid_o);
        dut->rdata_o(rdata_o);

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

    void idle_inputs() {
        req_i.write(false);
        addr_i.write(0);
        we_i.write(false);
        be_i.write(0);
        wdata_i.write(0ULL);
    }

    void do_reset() {
        idle_inputs();
        rst_n.write(false);
        tick();
        tick();
        rst_n.write(true);
        tick();
    }

    // Issue a write; returns after one tick with rvalid=1, rdata=0.
    void do_write(uint64_t addr, uint64_t data, uint32_t be = 0xF) {
        req_i.write(true);
        we_i.write(true);
        addr_i.write(addr);
        wdata_i.write(data);
        be_i.write(be);
        tick();
        idle_inputs();
    }

    // Issue a read; returns after one tick with rvalid=1, rdata=mem[addr].
    uint64_t do_read(uint64_t addr) {
        req_i.write(true);
        we_i.write(false);
        addr_i.write(addr);
        be_i.write(0xF);
        tick();
        uint64_t result = rdata_o.read();
        idle_inputs();
        return result;
    }

    // -----------------------------------------------------------------------
    // Test thread
    // -----------------------------------------------------------------------
    void run() {
        do_reset();

        // -------------------------------------------------------------------
        // T01 — Reset: outputs are silent while rst_ni=0
        // -------------------------------------------------------------------
        std::printf("\n=== T01: Reset outputs ===\n");
        idle_inputs();
        rst_n.write(false);
        tick();
        CHECK(!rvalid_o.read(), "T01 rvalid_o=0 during reset");
        CHECK(rdata_o.read() == 0ULL, "T01 rdata_o=0 during reset");
        rst_n.write(true);
        tick();

        // -------------------------------------------------------------------
        // T02 — gnt_o is combinatorial: high immediately when req_i asserted
        // -------------------------------------------------------------------
        std::printf("\n=== T02: gnt_o combinatorial (asserted) ===\n");
        req_i.write(true);
        wait(1, SC_NS);
        CHECK(gnt_o.read(), "T02 gnt_o=1 without clock edge");
        idle_inputs();
        tick();

        // -------------------------------------------------------------------
        // T03 — gnt_o=0 immediately when req_i deasserted
        // -------------------------------------------------------------------
        std::printf("\n=== T03: gnt_o combinatorial (deasserted) ===\n");
        req_i.write(true);
        wait(1, SC_NS);
        req_i.write(false);
        wait(1, SC_NS);
        CHECK(!gnt_o.read(), "T03 gnt_o=0 after req_i deasserted");
        tick();

        // -------------------------------------------------------------------
        // T04 — Read zero-initialised memory returns 0
        // -------------------------------------------------------------------
        std::printf("\n=== T04: Read zero-initialised memory ===\n");
        do_reset();
        req_i.write(true);
        addr_i.write(0);
        we_i.write(false);
        be_i.write(0xF);
        tick();
        CHECK(rvalid_o.read(), "T04 rvalid_o=1 on read response");
        CHECK(rdata_o.read() == 0ULL, "T04 rdata_o=0 from zero-init");
        idle_inputs();

        // -------------------------------------------------------------------
        // T05 — rvalid_o=0 on the cycle after no request
        // -------------------------------------------------------------------
        std::printf("\n=== T05: No rvalid when no prior request ===\n");
        idle_inputs();
        tick();
        CHECK(!rvalid_o.read(), "T05 rvalid_o=0 with no req previous cycle");

        // -------------------------------------------------------------------
        // T06 — Write response: rvalid=1, rdata=0
        // -------------------------------------------------------------------
        std::printf("\n=== T06: Write response ===\n");
        do_reset();
        req_i.write(true);
        we_i.write(true);
        addr_i.write(0);
        wdata_i.write(0xDEADBEEFULL);
        be_i.write(0xF);
        tick();
        CHECK(rvalid_o.read(), "T06 rvalid_o=1 on write response");
        CHECK(rdata_o.read() == 0ULL, "T06 rdata_o=0 on write (no read data)");
        idle_inputs();

        // -------------------------------------------------------------------
        // T07 — Write then read: round-trip data integrity
        // -------------------------------------------------------------------
        std::printf("\n=== T07: Write then read round-trip ===\n");
        do_reset();
        do_write(0, 0xCAFEBABEULL);
        tick(); // idle cycle: rvalid=0
        CHECK(!rvalid_o.read(), "T07 rvalid_o=0 on idle cycle after write");
        req_i.write(true);
        we_i.write(false);
        addr_i.write(0);
        be_i.write(0xF);
        tick();
        CHECK(rvalid_o.read(), "T07 rvalid_o=1 on read response");
        CHECK(rdata_o.read() == 0xCAFEBABEULL, "T07 rdata_o matches written value");
        idle_inputs();

        // -------------------------------------------------------------------
        // T08 — Byte-enable partial write
        // -------------------------------------------------------------------
        std::printf("\n=== T08: Byte-enable partial write ===\n");
        do_reset();
        // Write full word to word 1 (byte addr=4 with BYTES_PER_WORD=4)
        do_write(4, 0xDEADBEEFULL, 0xF);
        // Overwrite only bytes 0-1 (be=0x3): expect 0xDEAD_CAFE
        do_write(4, 0x0000CAFEULL, 0x3);
        CHECK(do_read(4) == 0xDEADCAFEULL, "T08 partial write: unselected bytes preserved");

        // Overwrite only bytes 2-3 (be=0xC): expect 0xBEEF_CAFE
        do_write(4, 0xBEEF0000ULL, 0xC);
        CHECK(do_read(4) == 0xBEEFCAFEULL, "T08 partial write: high bytes updated");

        // -------------------------------------------------------------------
        // T09 — Multiple rows are independent
        // -------------------------------------------------------------------
        std::printf("\n=== T09: Multiple rows independent ===\n");
        do_reset();
        do_write(0, 0x11111111ULL);
        do_write(4, 0x22222222ULL);
        do_write(8, 0x33333333ULL);
        do_write(12, 0x44444444ULL);

        CHECK(do_read(0) == 0x11111111ULL, "T09 word 0 unaffected by other writes");
        CHECK(do_read(4) == 0x22222222ULL, "T09 word 1 unaffected by other writes");
        CHECK(do_read(8) == 0x33333333ULL, "T09 word 2 unaffected by other writes");
        CHECK(do_read(12) == 0x44444444ULL, "T09 word 3 unaffected by other writes");

        // -------------------------------------------------------------------
        // T10 — Back-to-back reads
        // -------------------------------------------------------------------
        std::printf("\n=== T10: Back-to-back reads ===\n");
        do_reset();
        do_write(0, 0xAAAAAAAAULL);
        do_write(4, 0xBBBBBBBBULL);

        // Issue two reads without an idle cycle between them
        req_i.write(true);
        we_i.write(false);
        be_i.write(0xF);
        addr_i.write(0);
        tick();
        bool     rv0 = rvalid_o.read();
        uint64_t rd0 = rdata_o.read();

        addr_i.write(4); // keep req_i high, change address
        tick();
        bool     rv1 = rvalid_o.read();
        uint64_t rd1 = rdata_o.read();
        idle_inputs();

        CHECK(rv0, "T10 first read rvalid=1");
        CHECK(rd0 == 0xAAAAAAAAULL, "T10 first read data correct");
        CHECK(rv1, "T10 second read rvalid=1");
        CHECK(rd1 == 0xBBBBBBBBULL, "T10 second read data correct");

        // -------------------------------------------------------------------
        // T11 — Overwrite: second write to same row takes effect
        // -------------------------------------------------------------------
        std::printf("\n=== T11: Overwrite same row ===\n");
        do_reset();
        do_write(0, 0x11111111ULL);
        do_write(0, 0x99999999ULL);
        CHECK(do_read(0) == 0x99999999ULL, "T11 second write takes effect");

        // -------------------------------------------------------------------
        // T12 — Reset mid-operation: rvalid deasserts immediately
        // -------------------------------------------------------------------
        std::printf("\n=== T12: Reset clears rvalid ===\n");
        do_reset();
        // Issue a read — rvalid goes high in this tick
        req_i.write(true);
        we_i.write(false);
        addr_i.write(0);
        be_i.write(0xF);
        tick();
        CHECK(rvalid_o.read(), "T12 rvalid=1 before reset");
        idle_inputs();
        // Assert reset — step() runs at next posedge and clears rvalid
        rst_n.write(false);
        tick();
        CHECK(!rvalid_o.read(), "T12 rvalid=0 immediately after reset");
        rst_n.write(true);
        tick();

        // -------------------------------------------------------------------
        // Summary
        // -------------------------------------------------------------------
        std::printf("\n=== Summary ===\n");
        std::printf("  passed: %d\n", g_pass);
        std::printf("  failed: %d\n", g_fail);

        if (g_fail == 0)
            std::printf("\nAll tests passed.\n");
        else
            std::printf("\n%d test(s) FAILED.\n", g_fail);

        sc_stop();
    }
};

int sc_main(int, char *[]) {
    tb tb_inst("tb");
    sc_start();
    return (g_fail > 0) ? 1 : 0;
}
