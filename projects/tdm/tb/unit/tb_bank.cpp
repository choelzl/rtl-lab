// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
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
//   T13  Row index wraps modulo NUM_ROW instead of faulting out-of-range
//   T14  Walking single-byte enable: each byte lane lands in its own slot
//   T15  we=1/be=0 is a silent no-op that is still granted and acked
//   T16  Interleaved be patterns (0x5/0xA) compose without cross-lane bleed
//   T17  Full/partial/full overwrite sequence composes exactly
// -----------------------------------------------------------------------------

#include "bank.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <cstdlib>
#include <systemc.h>

static constexpr int kNumRows  = 8;
static constexpr int kBytesRow = 4;
using DUT                      = bank<kNumRows, kBytesRow>;
using data_t                   = DUT::data_t;

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
    sc_signal<data_t>   wdata_i{"wdata_i"};
    sc_signal<bool>     gnt_o{"gnt_o"};
    sc_signal<bool>     rvalid_o{"rvalid_o"};
    sc_signal<data_t>   rdata_o{"rdata_o"};

    DUT *dut;

    SC_HAS_PROCESS(tb);

    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->obi.req_i(req_i);
        dut->obi.addr_i(addr_i);
        dut->obi.we_i(we_i);
        dut->obi.be_i(be_i);
        dut->obi.wdata_i(wdata_i);
        dut->obi.gnt_o(gnt_o);
        dut->obi.rvalid_o(rvalid_o);
        dut->obi.rdata_o(rdata_o);

        SC_THREAD(run);
    }

    ~tb() {
        delete dut;
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------

    void idle_inputs() {
        req_i.write(false);
        addr_i.write(0);
        we_i.write(false);
        be_i.write(0);
        wdata_i.write(data_t(0));
    }

    void do_reset() {
        idle_inputs();
        rst_n.write(false);
        tick(clk);
        tick(clk);
        rst_n.write(true);
        tick(clk);
    }

    // Issue a write; returns after one tick with rvalid=1, rdata=0.
    void do_write(uint64_t addr, uint64_t data, uint32_t be = 0xF) {
        req_i.write(true);
        we_i.write(true);
        addr_i.write(addr);
        wdata_i.write(data_t(static_cast<unsigned long long>(data)));
        be_i.write(be);
        tick(clk);
        idle_inputs();
    }

    // Issue a read; returns after one tick with rvalid=1, rdata=mem[addr].
    uint64_t do_read(uint64_t addr) {
        req_i.write(true);
        we_i.write(false);
        addr_i.write(addr);
        be_i.write(0xF);
        tick(clk);
        uint64_t result = rdata_o.read().to_uint64();
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
        tick(clk);
        CHECK(!rvalid_o.read(), "T01 rvalid_o=0 during reset");
        CHECK(rdata_o.read().to_uint64() == 0ULL, "T01 rdata_o=0 during reset");
        rst_n.write(true);
        tick(clk);

        // -------------------------------------------------------------------
        // T02 — gnt_o is combinatorial: high immediately when req_i asserted
        // -------------------------------------------------------------------
        std::printf("\n=== T02: gnt_o combinatorial (asserted) ===\n");
        req_i.write(true);
        wait(1, SC_NS);
        CHECK(gnt_o.read(), "T02 gnt_o=1 without clock edge");
        idle_inputs();
        tick(clk);

        // -------------------------------------------------------------------
        // T03 — gnt_o=0 immediately when req_i deasserted
        // -------------------------------------------------------------------
        std::printf("\n=== T03: gnt_o combinatorial (deasserted) ===\n");
        req_i.write(true);
        wait(1, SC_NS);
        req_i.write(false);
        wait(1, SC_NS);
        CHECK(!gnt_o.read(), "T03 gnt_o=0 after req_i deasserted");
        tick(clk);

        // -------------------------------------------------------------------
        // T04 — Read zero-initialised memory returns 0
        // -------------------------------------------------------------------
        std::printf("\n=== T04: Read zero-initialised memory ===\n");
        do_reset();
        req_i.write(true);
        addr_i.write(0);
        we_i.write(false);
        be_i.write(0xF);
        tick(clk);
        CHECK(rvalid_o.read(), "T04 rvalid_o=1 on read response");
        CHECK(rdata_o.read().to_uint64() == 0ULL, "T04 rdata_o=0 from zero-init");
        idle_inputs();

        // -------------------------------------------------------------------
        // T05 — rvalid_o=0 on the cycle after no request
        // -------------------------------------------------------------------
        std::printf("\n=== T05: No rvalid when no prior request ===\n");
        idle_inputs();
        tick(clk);
        CHECK(!rvalid_o.read(), "T05 rvalid_o=0 with no req previous cycle");

        // -------------------------------------------------------------------
        // T06 — Write response: rvalid=1, rdata=0
        // -------------------------------------------------------------------
        std::printf("\n=== T06: Write response ===\n");
        do_reset();
        req_i.write(true);
        we_i.write(true);
        addr_i.write(0);
        wdata_i.write(data_t(0xDEADBEEFULL));
        be_i.write(0xF);
        tick(clk);
        CHECK(rvalid_o.read(), "T06 rvalid_o=1 on write response");
        CHECK(rdata_o.read().to_uint64() == 0ULL, "T06 rdata_o=0 on write (no read data)");
        idle_inputs();

        // -------------------------------------------------------------------
        // T07 — Write then read: round-trip data integrity
        // -------------------------------------------------------------------
        std::printf("\n=== T07: Write then read round-trip ===\n");
        do_reset();
        do_write(0, 0xCAFEBABEULL);
        tick(clk); // idle cycle: rvalid=0
        CHECK(!rvalid_o.read(), "T07 rvalid_o=0 on idle cycle after write");
        req_i.write(true);
        we_i.write(false);
        addr_i.write(0);
        be_i.write(0xF);
        tick(clk);
        CHECK(rvalid_o.read(), "T07 rvalid_o=1 on read response");
        CHECK(rdata_o.read().to_uint64() == 0xCAFEBABEULL, "T07 rdata_o matches written value");
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
        tick(clk);
        bool     rv0 = rvalid_o.read();
        uint64_t rd0 = rdata_o.read().to_uint64();

        addr_i.write(4); // keep req_i high, change address
        tick(clk);
        bool     rv1 = rvalid_o.read();
        uint64_t rd1 = rdata_o.read().to_uint64();
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
        tick(clk);
        CHECK(rvalid_o.read(), "T12 rvalid=1 before reset");
        idle_inputs();
        // Assert reset — step() runs at next posedge and clears rvalid
        rst_n.write(false);
        tick(clk);
        CHECK(!rvalid_o.read(), "T12 rvalid=0 immediately after reset");
        rst_n.write(true);
        tick(clk);

        // -------------------------------------------------------------------
        // T13 — Row index wraps modulo NUM_ROW instead of faulting
        // -------------------------------------------------------------------
        std::printf("\n=== T13: Row wraps modulo NUM_ROW ===\n");
        do_reset();
        do_write(0, 0x11111111ULL); // row 0
        // Address one full bank past row 0: row = (kNumRows*kBytesRow)/kBytesRow = kNumRows,
        // which wraps to row 0 (kNumRows % kNumRows == 0).
        const uint64_t wrap_addr = static_cast<uint64_t>(kNumRows) * kBytesRow;
        CHECK(do_read(wrap_addr) == 0x11111111ULL,
              "T13a out-of-range addr aliases back to row 0 (read)");
        do_write(wrap_addr, 0x22222222ULL);
        CHECK(do_read(0) == 0x22222222ULL, "T13b write through wrapped addr updates row 0");
        // Two banks past row 0 should alias the same way.
        const uint64_t wrap_addr2 = static_cast<uint64_t>(2 * kNumRows) * kBytesRow;
        CHECK(do_read(wrap_addr2) == 0x22222222ULL, "T13c wraps consistently at 2x capacity");

        // -------------------------------------------------------------------
        // T14 — Walking single-byte enable: each byte lane lands in its own
        // slot (T08's adjacent-pair masks can't distinguish a lane-index
        // error in apply_be's l*8 shift; a per-lane walk can)
        // -------------------------------------------------------------------
        std::printf("\n=== T14: Walking single-byte enable ===\n");
        do_reset();
        do_write(3 * kBytesRow, 0, 0xF); // clear row 3
        do_write(3 * kBytesRow, 0x000000AAULL, 0x1);
        do_write(3 * kBytesRow, 0x0000BB00ULL, 0x2);
        do_write(3 * kBytesRow, 0x00CC0000ULL, 0x4);
        do_write(3 * kBytesRow, 0xDD000000ULL, 0x8);
        CHECK(do_read(3 * kBytesRow) == 0xDDCCBBAAULL,
              "T14 four single-byte writes assemble 0xDDCCBBAA");

        // -------------------------------------------------------------------
        // T15 — we=1 with be=0 is a silent no-op that still ACKS: the request
        // is accepted (gnt), produces rvalid, and modifies nothing
        // -------------------------------------------------------------------
        std::printf("\n=== T15: be=0 write is an acknowledged no-op ===\n");
        do_write(4 * kBytesRow, 0xCAFECAFEULL, 0xF);
        req_i.write(true);
        we_i.write(true);
        addr_i.write(4 * kBytesRow);
        wdata_i.write(data_t(0xDEADBEEFull));
        be_i.write(0x0);
        wait(1, SC_NS);
        CHECK(gnt_o.read(), "T15a be=0 write is still granted");
        tick(clk);
        idle_inputs();
        CHECK(rvalid_o.read(), "T15b be=0 write is still acknowledged (rvalid)");
        CHECK(do_read(4 * kBytesRow) == 0xCAFECAFEULL, "T15c row content untouched");

        // -------------------------------------------------------------------
        // T16 — Interleaved (non-contiguous) byte-enable patterns: 0x5 and
        // 0xA select alternating lanes; together they must compose the full
        // word with no cross-lane bleed
        // -------------------------------------------------------------------
        std::printf("\n=== T16: interleaved byte-enable patterns ===\n");
        do_write(5 * kBytesRow, 0, 0xF);
        do_write(5 * kBytesRow, 0x00CC00AAULL, 0x5); // lanes 0 and 2
        CHECK(do_read(5 * kBytesRow) == 0x00CC00AAULL, "T16a be=0x5 writes lanes 0/2 only");
        do_write(5 * kBytesRow, 0xDD00BB00ULL, 0xA); // lanes 1 and 3
        CHECK(do_read(5 * kBytesRow) == 0xDDCCBBAAULL,
              "T16b be=0xA fills lanes 1/3 without disturbing 0/2");

        // -------------------------------------------------------------------
        // T17 — Overwrite sequence: full write, partial overwrite, full
        // overwrite — each step observes exactly the expected composition
        // -------------------------------------------------------------------
        std::printf("\n=== T17: overwrite sequence ===\n");
        do_write(6 * kBytesRow, 0x11111111ULL, 0xF);
        do_write(6 * kBytesRow, 0x00002222ULL, 0x3); // low half only
        CHECK(do_read(6 * kBytesRow) == 0x11112222ULL, "T17a partial overwrite low half");
        do_write(6 * kBytesRow, 0x33333333ULL, 0xF);
        CHECK(do_read(6 * kBytesRow) == 0x33333333ULL, "T17b full overwrite wins completely");
        do_write(6 * kBytesRow, 0x00440000ULL, 0x4); // one middle lane
        CHECK(do_read(6 * kBytesRow) == 0x33443333ULL, "T17c middle-lane overwrite composes");

        // -------------------------------------------------------------------
        // T18 — rvalid is a one-shot pulse: exactly one response per accepted
        // request, no repetition on the following idle cycles
        // -------------------------------------------------------------------
        std::printf("\n=== T18: rvalid one-shot ===\n");
        req_i.write(true);
        we_i.write(false);
        addr_i.write(6 * kBytesRow);
        be_i.write(0xF);
        tick(clk);
        idle_inputs();
        CHECK(rvalid_o.read(), "T18a response pulse on the cycle after acceptance");
        tick(clk);
        CHECK(!rvalid_o.read(), "T18b pulse is gone the next cycle with no new request");

        sc_stop();
    }
};

int sc_main(int, char *[]) {
    tb tb_inst("tb");
    sc_start();
    return report_and_exit();
}
