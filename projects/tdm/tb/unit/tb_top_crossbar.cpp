// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for top_crossbar<> (default parameters:
//   NUM_RPORT=2, NUM_WPORT=2, NUM_REQ=4, NUM_BANK=8, NUM_ROW=1024,
//   BYTES_PER_WORD=4, WORDS_PER_ROW=4  →  BYTES_PER_ROW=16, NUM_PHYS_BANKS=16)
//
// Build and run:
//   make unit-test PROJECT=tdm TOP_LEVEL=top_crossbar
//
// No-conflict address layout (addr_hash is a no-op for addr < 0x200 with
// addr[11:9]=0; routing bits are addr[7:4]):
//   rport[0] buses 0-3 → 0x00, 0x10, 0x20, 0x30  (L1_sel 0-3, L2_sel=0)
//   rport[1] buses 0-3 → 0x40, 0x50, 0x60, 0x70  (L1_sel 0-3, L2_sel=1)
// Each of the 8 buses maps to a distinct physical bank with no arbitration at
// any level of the crossbar hierarchy.
//
// Tests:
//   T01  Reset — all output ports deasserted
//   T02  Write then read — single wport write / single rport read, data integrity
//   T03  Full conflict — all NR rport buses read the same address; all eventually served
//   T04  No conflict — NR rport buses each target a unique bank; all served in one cycle
//   T05  64-address sequential write-then-read — data integrity and response ordering
//   T06  Overwrite — second write to same address overwrites first (last write wins)
//   T07  Alternating bit patterns (0xAA.../0x55...) — stuck-at-0/1 detection
//   T08  Cross-address independence — writes to set B do not corrupt set A
//   T09  Port response isolation — rvalid/gnt asserts only on the requesting port
//   T10  Simultaneous multi-port write then read — all NW wports and NR rports in parallel
//   T11  Byte-enable partial write — only selected bytes updated end-to-end
//   T12  addr=0 is a real, writable address (crossbar has no NOP sentinel, unlike TDM)
//   T13  Genuinely concurrent read + write traffic on independent ports
//   T14  Write — 1 real request + 7 idle ports (crossbar has no buffering/
//        window, so "NOP padding" here just means the other ports never
//        request at all — not all ports need to be used every cycle)
//   T15  Read — 1 real request + 7 idle ports
// -----------------------------------------------------------------------------

#include "top_crossbar.hpp"
#include "unit_test_common.hpp"
#include <algorithm>
#include <cstdio>
#include <systemc.h>

using DUT    = top_crossbar<>;
using data_t = DUT::data_t;

static constexpr int      NR      = DUT::NUM_RPORT_PORTS; // 8
static constexpr int      NW      = DUT::NUM_WPORT_PORTS; // 8
static constexpr uint32_t FULL_BE = 0xFFFF;               // all 16 byte-enables

// Addresses whose routing bits [7:4] map each rport bus to a unique bank.
static constexpr uint64_t ADDR_NC[NR] = {
    0x00, 0x10, 0x20, 0x30, // rport buses 0-3  (L1_sel 0-3, L2_sel 0)
    0x40, 0x50, 0x60, 0x70, // rport buses 4-7  (L1_sel 0-3, L2_sel 1)
};

// ─── test accounting ─────────────────────────────────────────────────────────

// Fill all four 32-bit words of a 16-byte row with the same 32-bit value.

// ─── testbench module ────────────────────────────────────────────────────────
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

    // ── helpers ──────────────────────────────────────────────────────────────

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

    // Write one row via wport bus `bus`; asserts after one tick that wport_rvalid fires.
    void do_write(int bus, uint64_t addr, data_t data, uint32_t be = FULL_BE) {
        wport[bus].req.write(true);
        wport[bus].addr.write(addr);
        wport[bus].we.write(true);
        wport[bus].be.write(be);
        wport[bus].wdata.write(data);
        tick(clk);
        CHECK(wport[bus].rvalid.read(), "do_write: wport_rvalid fired");
        idle_wport(bus);
    }

    // Read one row via rport bus `bus`; asserts rvalid and returns rdata.
    data_t do_read(int bus, uint64_t addr) {
        rport[bus].req.write(true);
        rport[bus].addr.write(addr);
        rport[bus].we.write(false);
        rport[bus].be.write(FULL_BE);
        tick(clk);
        CHECK(rport[bus].rvalid.read(), "do_read: rport_rvalid fired");
        data_t rd = rport[bus].rdata.read();
        idle_rport(bus);
        return rd;
    }

    // ── test thread ──────────────────────────────────────────────────────────

    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: Reset — all output ports deasserted ===");
        // -------------------------------------------------------------------
        do_reset();

        bool all_rport_gnt_low = true, all_rport_rv_low = true;
        bool all_wport_gnt_low = true, all_wport_rv_low = true;
        for (int m = 0; m < NR; ++m) {
            all_rport_gnt_low &= !rport[m].gnt.read();
            all_rport_rv_low &= !rport[m].rvalid.read();
        }
        for (int m = 0; m < NW; ++m) {
            all_wport_gnt_low &= !wport[m].gnt.read();
            all_wport_rv_low &= !wport[m].rvalid.read();
        }
        CHECK(all_rport_gnt_low, "T01a rport_gnt_o[*]=0 after reset");
        CHECK(all_rport_rv_low, "T01b rport_rvalid_o[*]=0 after reset");
        CHECK(all_wport_gnt_low, "T01c wport_gnt_o[*]=0 after reset");
        CHECK(all_wport_rv_low, "T01d wport_rvalid_o[*]=0 after reset");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: Write then read — data integrity ===");
        // -------------------------------------------------------------------
        do_reset();

        const data_t wr_data = make_row<data_t>(0xDEADBEEFU);
        do_write(0, 0x00, wr_data);

        data_t rd_data = do_read(0, 0x00);
        CHECK(rd_data == wr_data, "T02 rdata matches wdata written via wport[0]");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: Full conflict — all rport buses read the same address ===");
        // -------------------------------------------------------------------
        do_reset();

        const data_t conflict_data = make_row<data_t>(0xCAFEBABEU);
        do_write(0, 0x00, conflict_data);

        // Assert all NR requests to the same address.
        for (int m = 0; m < NR; ++m) {
            rport[m].req.write(true);
            rport[m].addr.write(0x00);
            rport[m].we.write(false);
            rport[m].be.write(FULL_BE);
        }

        // In the first tick, not all buses can be served (only the single
        // winner per bank makes it through).  Collect first-tick rvalids to
        // confirm serialisation is actually happening.
        tick(clk);
        int first_rvalid = 0;
        for (int m = 0; m < NR; ++m)
            if (rport[m].rvalid.read())
                ++first_rvalid;
        CHECK(first_rvalid < NR, "T03a first tick serves fewer than NR buses (serialised)");

        // Deassert those already served; keep remaining buses requesting.
        // One bus is served per cycle — 8 buses need ≤ NR+4 cycles total.
        bool done[NR] = {};
        int  n_done   = first_rvalid;
        for (int m = 0; m < NR; ++m) {
            if (rport[m].rvalid.read()) {
                CHECK(rport[m].rdata.read() == conflict_data, "T03b conflict read: rdata correct");
                done[m] = true;
                idle_rport(m);
            }
        }

        for (int iter = 0; iter < NR + 4 && n_done < NR; ++iter) {
            tick(clk);
            for (int m = 0; m < NR; ++m) {
                if (!done[m] && rport[m].rvalid.read()) {
                    CHECK(rport[m].rdata.read() == conflict_data,
                          "T03c conflict read: rdata correct (remaining buses)");
                    done[m] = true;
                    ++n_done;
                    idle_rport(m);
                }
            }
        }
        CHECK(n_done == NR, "T03d all NR conflict reads completed");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: No conflict — NR buses target unique banks ===");
        // -------------------------------------------------------------------
        do_reset();

        // Write unique data to each of the 8 bank-distinct addresses.
        data_t nc_data[NR];
        for (int m = 0; m < NR; ++m) {
            nc_data[m] = make_row<data_t>(0x10000000U * (m + 1));
            do_write(0, ADDR_NC[m], nc_data[m]);
        }

        // Assert all NR reads simultaneously.
        for (int m = 0; m < NR; ++m) {
            rport[m].req.write(true);
            rport[m].addr.write(ADDR_NC[m]);
            rport[m].we.write(false);
            rport[m].be.write(FULL_BE);
        }

        // Combinatorial grants: all NR buses should see gnt=1 at the same time.
        wait(1, SC_NS);
        int all_gnt = 0;
        for (int m = 0; m < NR; ++m)
            if (rport[m].gnt.read())
                ++all_gnt;
        CHECK(all_gnt == NR, "T04a all NR buses granted simultaneously (no conflict)");

        // One tick: all NR banks process in parallel → all rvalid together.
        tick(clk);
        int all_rv = 0;
        for (int m = 0; m < NR; ++m)
            if (rport[m].rvalid.read())
                ++all_rv;
        CHECK(all_rv == NR, "T04b all NR rvalid signals fired in the same cycle");

        bool data_ok = true;
        for (int m = 0; m < NR; ++m) {
            if (!(rport[m].rdata.read() == nc_data[m])) {
                data_ok = false;
                std::printf("  FAIL  T04c bus %d: rdata mismatch\n", m);
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (data_ok)
            std::puts("  PASS  T04c all NR read data values correct");

        for (int m = 0; m < NR; ++m)
            idle_rport(m);

        // -------------------------------------------------------------------
        std::puts("\n=== T05: Sequential write-then-read — 64 addresses, data and order ===");
        // -------------------------------------------------------------------
        // Write 64 distinct rows (address stride = BYTES_PER_ROW = 0x10) with a
        // unique per-address payload, then read them back in the same order and
        // verify both data correctness and address ordering.  This catches bank
        // aliasing, routing conflicts, and row-collision bugs that a single-address
        // write-read (T02) would miss.
        do_reset();

        static constexpr int      N_ADDRS   = 64;
        static constexpr uint64_t ADDR_STEP = 0x10;

        data_t t05_wdata[N_ADDRS];
        for (int i = 0; i < N_ADDRS; ++i) {
            // Fill all four 32-bit words with the same value so make_row<data_t>() applies.
            t05_wdata[i] = make_row<data_t>(0x01010101U * static_cast<uint32_t>(i + 1));
            do_write(0, static_cast<uint64_t>(i) * ADDR_STEP, t05_wdata[i]);
        }

        bool t05_order_ok = true;
        bool t05_data_ok  = true;
        for (int i = 0; i < N_ADDRS; ++i) {
            const uint64_t addr = static_cast<uint64_t>(i) * ADDR_STEP;
            rport[0].req.write(true);
            rport[0].addr.write(addr);
            rport[0].we.write(false);
            rport[0].be.write(FULL_BE);
            tick(clk);
            if (!rport[0].rvalid.read()) {
                t05_order_ok = false;
                std::printf("  FAIL  T05: no rvalid for addr 0x%03llx (i=%d)\n",
                            (unsigned long long)addr, i);
                ++g_fail;
            } else {
                const data_t rd = rport[0].rdata.read();
                if (!(rd == t05_wdata[i])) {
                    t05_data_ok = false;
                    std::printf("  FAIL  T05: data mismatch at addr 0x%03llx (i=%d)\n",
                                (unsigned long long)addr, i);
                    ++g_fail;
                } else {
                    ++g_pass;
                }
            }
            idle_rport(0);
        }
        if (t05_order_ok)
            std::puts("  PASS  T05a all 64 reads received rvalid in order");
        if (t05_data_ok)
            std::puts("  PASS  T05b all 64 read-back data values match written data");

        // -------------------------------------------------------------------
        std::puts("\n=== T06: Overwrite — last write wins at the same address ===");
        // -------------------------------------------------------------------
        // Write value A then value B to the same address.  A subsequent read
        // must return B; returning A means the second write was lost.
        do_reset();

        const data_t t06_first  = make_row<data_t>(0xDEADBEEFU);
        const data_t t06_second = make_row<data_t>(0x01234567U);
        do_write(0, 0x00, t06_first);
        do_write(0, 0x00, t06_second);
        CHECK(do_read(0, 0x00) == t06_second, "T06 overwrite: rdata equals second written value");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: Alternating bit patterns — stuck-at-0 and stuck-at-1 detection ===");
        // -------------------------------------------------------------------
        // Write 0xAAAAAAAA (alternating 1/0) to even-indexed addresses and
        // 0x55555555 (alternating 0/1) to odd-indexed addresses.  Read all back
        // and verify each pattern survives the round-trip intact.
        do_reset();

        static constexpr int N_PAT  = 8;
        const data_t         t07_aa = make_row<data_t>(0xAAAAAAAAU);
        const data_t         t07_55 = make_row<data_t>(0x55555555U);
        for (int i = 0; i < N_PAT; ++i) {
            const uint64_t addr = static_cast<uint64_t>(i) * ADDR_STEP;
            do_write(0, addr, (i % 2 == 0) ? t07_aa : t07_55);
        }
        bool t07_ok = true;
        for (int i = 0; i < N_PAT; ++i) {
            const uint64_t addr     = static_cast<uint64_t>(i) * ADDR_STEP;
            const data_t   expected = (i % 2 == 0) ? t07_aa : t07_55;
            if (!(do_read(0, addr) == expected)) {
                std::printf("  FAIL  T07: pattern mismatch at addr 0x%03llx (i=%d)\n",
                            (unsigned long long)addr, i);
                t07_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t07_ok)
            std::puts("  PASS  T07 all alternating-pattern reads match written data");

        // -------------------------------------------------------------------
        std::puts(
            "\n=== T08: Cross-address independence — writes to set B do not corrupt set A ===");
        // -------------------------------------------------------------------
        // Write set A (lower addresses) with payload_A, then write set B (higher
        // addresses) with payload_B.  Re-read set A: it must still hold payload_A.
        // This catches address aliasing and row-collision bugs.
        do_reset();

        static constexpr int N_AB   = 8;
        const uint64_t       BASE_A = 0x000;
        const uint64_t       BASE_B = 0x200; // well-separated to avoid accidental aliasing
        data_t               t08_A[N_AB], t08_B[N_AB];
        for (int i = 0; i < N_AB; ++i) {
            t08_A[i] = make_row<data_t>(0xA0000001U + static_cast<uint32_t>(i));
            t08_B[i] = make_row<data_t>(0xB0000001U + static_cast<uint32_t>(i));
            do_write(0, BASE_A + static_cast<uint64_t>(i) * ADDR_STEP, t08_A[i]);
        }
        for (int i = 0; i < N_AB; ++i)
            do_write(0, BASE_B + static_cast<uint64_t>(i) * ADDR_STEP, t08_B[i]);

        bool t08_ok = true;
        for (int i = 0; i < N_AB; ++i) {
            const uint64_t addr = BASE_A + static_cast<uint64_t>(i) * ADDR_STEP;
            if (!(do_read(0, addr) == t08_A[i])) {
                std::printf("  FAIL  T08: set-A addr 0x%03llx corrupted after set-B writes\n",
                            (unsigned long long)addr);
                t08_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t08_ok)
            std::puts("  PASS  T08 set-A data unchanged after set-B overwrites");

        // -------------------------------------------------------------------
        std::puts(
            "\n=== T09: Port response isolation — rvalid fires only on the requesting port ===");
        // -------------------------------------------------------------------
        // Assert a request on exactly one port.  No other port's rvalid/gnt should
        // assert as a side-effect (OBI: each response targets its own initiator).
        do_reset();

        // wport side: only wport[0] requests
        for (int m = 1; m < NW; ++m)
            idle_wport(m);
        wport[0].req.write(true);
        wport[0].addr.write(0x00);
        wport[0].we.write(true);
        wport[0].be.write(FULL_BE);
        wport[0].wdata.write(make_row<data_t>(0xFEDCBA98U));
        tick(clk);
        CHECK(wport[0].rvalid.read(), "T09a wport[0] rvalid fired");
        {
            bool no_leak = true;
            for (int m = 1; m < NW; ++m)
                no_leak &= !wport[m].rvalid.read() && !wport[m].gnt.read();
            CHECK(no_leak, "T09b no other wport rvalid/gnt asserted");
        }
        idle_wport(0);

        // rport side: only rport[0] requests
        for (int m = 1; m < NR; ++m)
            idle_rport(m);
        rport[0].req.write(true);
        rport[0].addr.write(0x00);
        rport[0].we.write(false);
        rport[0].be.write(FULL_BE);
        tick(clk);
        CHECK(rport[0].rvalid.read(), "T09c rport[0] rvalid fired");
        {
            bool no_leak = true;
            for (int m = 1; m < NR; ++m)
                no_leak &= !rport[m].rvalid.read() && !rport[m].gnt.read();
            CHECK(no_leak, "T09d no other rport rvalid/gnt asserted");
        }
        idle_rport(0);

        // -------------------------------------------------------------------
        std::puts("\n=== T10: Simultaneous multi-port write then read ===");
        // -------------------------------------------------------------------
        // Drive all NW wports to distinct addresses in the same cycle, then
        // drain rvalids (serialisation may spread them across cycles).  Read
        // all NR rports simultaneously and verify every read-back value matches
        // what was written.
        do_reset();

        data_t   t10_data[NW];
        uint64_t t10_addr[NW];
        for (int m = 0; m < NW; ++m) {
            t10_addr[m] = static_cast<uint64_t>(m) * ADDR_STEP;
            t10_data[m] = make_row<data_t>(0x10101010U * static_cast<uint32_t>(m + 1));
            wport[m].req.write(true);
            wport[m].addr.write(t10_addr[m]);
            wport[m].we.write(true);
            wport[m].be.write(FULL_BE);
            wport[m].wdata.write(t10_data[m]);
        }
        {
            bool w_done[NW] = {};
            int  w_got      = 0;
            for (int iter = 0; iter < NW * 4 && w_got < NW; ++iter) {
                tick(clk);
                for (int m = 0; m < NW; ++m) {
                    if (!w_done[m] && wport[m].rvalid.read()) {
                        w_done[m] = true;
                        ++w_got;
                        idle_wport(m);
                    }
                }
            }
            CHECK(w_got == NW, "T10a all NW parallel writes completed");
        }

        for (int m = 0; m < NR; ++m) {
            rport[m].req.write(true);
            rport[m].addr.write(t10_addr[m]);
            rport[m].we.write(false);
            rport[m].be.write(FULL_BE);
        }
        {
            bool r_done[NR]    = {};
            bool r_data_ok[NR] = {};
            int  r_got         = 0;
            for (int iter = 0; iter < NR * 4 && r_got < NR; ++iter) {
                tick(clk);
                for (int m = 0; m < NR; ++m) {
                    if (!r_done[m] && rport[m].rvalid.read()) {
                        r_data_ok[m] = (rport[m].rdata.read() == t10_data[m]);
                        r_done[m]    = true;
                        ++r_got;
                        idle_rport(m);
                    }
                }
            }
            CHECK(r_got == NR, "T10b all NR parallel reads completed");
            bool t10_all_ok = true;
            for (int m = 0; m < NR; ++m)
                t10_all_ok &= r_data_ok[m];
            CHECK(t10_all_ok, "T10c all parallel read data values match written data");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T11: Byte-enable partial write ===");
        // -------------------------------------------------------------------
        // Same scenario as top_tdm T12: full-BE baseline, then a partial-BE
        // overwrite (low 2 bytes only), verifying unselected bytes survive.
        do_reset();

        const data_t t11_base    = make_row<data_t>(0xAAAAAAAAU);
        const data_t t11_partial = make_row<data_t>(0x0000BBBBU);
        do_write(0, 0x00, t11_base);
        // be=0x0003 enables only the low 2 bytes of the 128-bit row (bits[15:0]).
        do_write(0, 0x00, t11_partial, 0x0003);
        {
            const data_t   rd   = do_read(0, 0x00);
            const uint32_t want = (t11_base.range(31, 0).to_uint() & 0xFFFF0000U) |
                                  (t11_partial.range(31, 0).to_uint() & 0x0000FFFFU);
            const uint32_t got = rd.range(31, 0).to_uint();
            CHECK(want == got, "T11 unselected bytes preserved, selected bytes updated");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T12: addr=0 is a real, writable address on crossbar ===");
        // -------------------------------------------------------------------
        // Unlike top_tdm (where addr=0 is a reserved NOP sentinel — see
        // buffer_cell.hpp/tdm.hpp), the crossbar backend has no windowing
        // layer needing to disambiguate "no request" from "real address 0",
        // so addr=0 must round-trip like any other address.
        do_reset();

        const data_t t12_data = make_row<data_t>(0x600DF00DU);
        do_write(0, 0x00, t12_data);
        CHECK(do_read(0, 0x00) == t12_data, "T12 addr=0 round-trips real data (no NOP semantics)");

        // -------------------------------------------------------------------
        std::puts("\n=== T13: Genuinely concurrent read + write traffic ===");
        // -------------------------------------------------------------------
        // T10 writes a full burst, then reads a full burst — the two phases
        // never overlap in time. Here a write burst and a read burst are
        // driven in the same cycles on independent ports/addresses to check
        // that concurrent traffic through the crossbar doesn't corrupt
        // either stream (mirrors top_tdm T14).
        do_reset();

        static constexpr uint64_t T13_RD_BASE = 0x000;
        static constexpr uint64_t T13_WR_BASE = 0x400;
        data_t                    t13_rd_frame[NW], t13_wr_frame[NW];
        for (int i = 0; i < NW; ++i) {
            t13_rd_frame[i] = make_row<data_t>(0x40000001U + static_cast<uint32_t>(i));
            t13_wr_frame[i] = make_row<data_t>(0x50000001U + static_cast<uint32_t>(i));
        }
        // Pre-populate the data the read side will fetch, before starting
        // the concurrent phase.
        for (int i = 0; i < NW; ++i)
            do_write(0, T13_RD_BASE + static_cast<uint64_t>(i) * ADDR_STEP, t13_rd_frame[i]);

        // Concurrent phase: drive an NW-wide write burst on wport[] while an
        // NR-wide read burst runs on rport[] in the same cycles.
        for (int m = 0; m < NW; ++m) {
            wport[m].req.write(true);
            wport[m].addr.write(T13_WR_BASE + static_cast<uint64_t>(m) * ADDR_STEP);
            wport[m].we.write(true);
            wport[m].be.write(FULL_BE);
            wport[m].wdata.write(t13_wr_frame[m]);
        }
        for (int m = 0; m < NR && m < NW; ++m) {
            rport[m].req.write(true);
            rport[m].addr.write(T13_RD_BASE + static_cast<uint64_t>(m) * ADDR_STEP);
            rport[m].we.write(false);
            rport[m].be.write(FULL_BE);
        }

        bool      w_done[NW] = {}, r_done[NR] = {};
        bool      r_data_ok[NR] = {};
        data_t    r_out[NR];
        int       w_got = 0, r_got = 0;
        const int r_targets = std::min(NR, NW);
        for (int iter = 0; iter < (NW + NR) * 4 && (w_got < NW || r_got < r_targets); ++iter) {
            tick(clk);
            for (int m = 0; m < NW; ++m) {
                if (!w_done[m] && wport[m].rvalid.read()) {
                    w_done[m] = true;
                    ++w_got;
                    idle_wport(m);
                }
            }
            for (int m = 0; m < r_targets; ++m) {
                if (!r_done[m] && rport[m].rvalid.read()) {
                    r_out[m]     = rport[m].rdata.read();
                    r_data_ok[m] = (r_out[m] == t13_rd_frame[m]);
                    r_done[m]    = true;
                    ++r_got;
                    idle_rport(m);
                }
            }
        }
        for (int m = 0; m < NW; ++m)
            idle_wport(m);
        for (int m = 0; m < r_targets; ++m)
            idle_rport(m);

        CHECK(w_got == NW, "T13a concurrent write burst completed");
        CHECK(r_got == r_targets, "T13b concurrent read burst completed");
        bool t13_r_ok = true;
        for (int m = 0; m < r_targets; ++m)
            t13_r_ok &= r_data_ok[m];
        CHECK(t13_r_ok,
              "T13c concurrent reads returned pre-written data (no corruption from writes)");

        // Verify the concurrently-written data landed correctly too.
        bool t13_w_ok = true;
        for (int i = 0; i < NW; ++i) {
            const data_t rd = do_read(0, T13_WR_BASE + static_cast<uint64_t>(i) * ADDR_STEP);
            if (!(rd == t13_wr_frame[i])) {
                std::printf("  FAIL  T13d: write mismatch i=%d\n", i);
                t13_w_ok = false;
                ++g_fail;
            } else {
                ++g_pass;
            }
        }
        if (t13_w_ok)
            std::puts("  PASS  T13d concurrently-written data correct after the fact");

        // -------------------------------------------------------------------
        std::puts("\n=== T14: Write — 1 real request + 7 idle ports ===");
        // -------------------------------------------------------------------
        // Unlike top_tdm's fixed 32-slot window (where unused slots must be
        // padded with the addr=0 NOP sentinel), the crossbar has no buffering
        // at all — a port that never asserts req_i simply never participates.
        // Only wport[REAL_WPORT] issues a real write this cycle; every other
        // wport is fully idle (not driving addr=0, just not requesting).
        do_reset();

        static constexpr int T14_REAL_WPORT = 5;
        const data_t         t14_data       = make_row<data_t>(0x7EA17EA1U);
        for (int m = 0; m < NW; ++m)
            idle_wport(m);
        wport[T14_REAL_WPORT].req.write(true);
        wport[T14_REAL_WPORT].addr.write(0x00);
        wport[T14_REAL_WPORT].we.write(true);
        wport[T14_REAL_WPORT].be.write(FULL_BE);
        wport[T14_REAL_WPORT].wdata.write(t14_data);
        tick(clk);
        CHECK(wport[T14_REAL_WPORT].rvalid.read(), "T14a the single real write completed");
        {
            bool no_leak = true;
            for (int m = 0; m < NW; ++m)
                if (m != T14_REAL_WPORT)
                    no_leak &= !wport[m].rvalid.read() && !wport[m].gnt.read();
            CHECK(no_leak, "T14b the 7 idle wports show no spurious gnt/rvalid");
        }
        idle_wport(T14_REAL_WPORT);
        CHECK(do_read(0, 0x00) == t14_data, "T14c written data reads back correctly");

        // -------------------------------------------------------------------
        std::puts("\n=== T15: Read — 1 real request + 7 idle ports ===");
        // -------------------------------------------------------------------
        do_reset();

        static constexpr int T15_REAL_RPORT = 6;
        const data_t         t15_data       = make_row<data_t>(0xF00DBEEFU);
        do_write(0, 0x00, t15_data);

        for (int m = 0; m < NR; ++m)
            idle_rport(m);
        rport[T15_REAL_RPORT].req.write(true);
        rport[T15_REAL_RPORT].addr.write(0x00);
        rport[T15_REAL_RPORT].we.write(false);
        rport[T15_REAL_RPORT].be.write(FULL_BE);
        tick(clk);
        CHECK(rport[T15_REAL_RPORT].rvalid.read(), "T15a the single real read completed");
        CHECK(rport[T15_REAL_RPORT].rdata.read() == t15_data,
              "T15b the single real read returns correct data");
        {
            bool no_leak = true;
            for (int m = 0; m < NR; ++m)
                if (m != T15_REAL_RPORT)
                    no_leak &= !rport[m].rvalid.read() && !rport[m].gnt.read();
            CHECK(no_leak, "T15c the 7 idle rports show no spurious gnt/rvalid");
        }
        idle_rport(T15_REAL_RPORT);

        // -------------------------------------------------------------------
        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
