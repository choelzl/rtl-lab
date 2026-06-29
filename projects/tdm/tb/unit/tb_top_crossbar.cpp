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
// -----------------------------------------------------------------------------

#include "top_crossbar.hpp"
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

// Fill all four 32-bit words of a 16-byte row with the same 32-bit value.
static data_t make_row(uint32_t v) {
    sc_bv<32> w(v);
    data_t    d;
    d.range(31, 0)   = w;
    d.range(63, 32)  = w;
    d.range(95, 64)  = w;
    d.range(127, 96) = w;
    return d;
}

// ─── testbench module ────────────────────────────────────────────────────────
SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    sc_signal<bool>     rport_req[NR];
    sc_signal<uint64_t> rport_addr[NR];
    sc_signal<bool>     rport_we[NR];
    sc_signal<uint32_t> rport_be[NR];
    sc_signal<data_t>   rport_wdata[NR];
    sc_signal<bool>     rport_gnt[NR];
    sc_signal<bool>     rport_rvalid[NR];
    sc_signal<data_t>   rport_rdata[NR];

    sc_signal<bool>     wport_req[NW];
    sc_signal<uint64_t> wport_addr[NW];
    sc_signal<bool>     wport_we[NW];
    sc_signal<uint32_t> wport_be[NW];
    sc_signal<data_t>   wport_wdata[NW];
    sc_signal<bool>     wport_gnt[NW];
    sc_signal<bool>     wport_rvalid[NW];
    sc_signal<data_t>   wport_rdata[NW];

    DUT *dut;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        for (int m = 0; m < NR; ++m) {
            dut->rport_req_i[m](rport_req[m]);
            dut->rport_addr_i[m](rport_addr[m]);
            dut->rport_we_i[m](rport_we[m]);
            dut->rport_be_i[m](rport_be[m]);
            dut->rport_wdata_i[m](rport_wdata[m]);
            dut->rport_gnt_o[m](rport_gnt[m]);
            dut->rport_rvalid_o[m](rport_rvalid[m]);
            dut->rport_rdata_o[m](rport_rdata[m]);
        }
        for (int m = 0; m < NW; ++m) {
            dut->wport_req_i[m](wport_req[m]);
            dut->wport_addr_i[m](wport_addr[m]);
            dut->wport_we_i[m](wport_we[m]);
            dut->wport_be_i[m](wport_be[m]);
            dut->wport_wdata_i[m](wport_wdata[m]);
            dut->wport_gnt_o[m](wport_gnt[m]);
            dut->wport_rvalid_o[m](wport_rvalid[m]);
            dut->wport_rdata_o[m](wport_rdata[m]);
        }
        SC_THREAD(run);
    }

    ~tb() {
        delete dut;
    }

    // ── helpers ──────────────────────────────────────────────────────────────

    void tick() {
        wait(clk.posedge_event());
        wait(1, SC_NS);
    }

    void idle_rport(int m) {
        rport_req[m].write(false);
        rport_we[m].write(false);
        rport_be[m].write(0);
        rport_addr[m].write(0);
        rport_wdata[m].write(data_t(0));
    }

    void idle_wport(int m) {
        wport_req[m].write(false);
        wport_we[m].write(false);
        wport_be[m].write(0);
        wport_addr[m].write(0);
        wport_wdata[m].write(data_t(0));
    }

    void do_reset() {
        rst_n.write(false);
        for (int m = 0; m < NR; ++m)
            idle_rport(m);
        for (int m = 0; m < NW; ++m)
            idle_wport(m);
        tick();
        tick();
        rst_n.write(true);
        tick();
    }

    // Write one row via wport bus `bus`; asserts after one tick that wport_rvalid fires.
    void do_write(int bus, uint64_t addr, data_t data) {
        wport_req[bus].write(true);
        wport_addr[bus].write(addr);
        wport_we[bus].write(true);
        wport_be[bus].write(FULL_BE);
        wport_wdata[bus].write(data);
        tick();
        CHECK(wport_rvalid[bus].read(), "do_write: wport_rvalid fired");
        idle_wport(bus);
    }

    // Read one row via rport bus `bus`; asserts rvalid and returns rdata.
    data_t do_read(int bus, uint64_t addr) {
        rport_req[bus].write(true);
        rport_addr[bus].write(addr);
        rport_we[bus].write(false);
        rport_be[bus].write(FULL_BE);
        tick();
        CHECK(rport_rvalid[bus].read(), "do_read: rport_rvalid fired");
        data_t rd = rport_rdata[bus].read();
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
            all_rport_gnt_low &= !rport_gnt[m].read();
            all_rport_rv_low &= !rport_rvalid[m].read();
        }
        for (int m = 0; m < NW; ++m) {
            all_wport_gnt_low &= !wport_gnt[m].read();
            all_wport_rv_low &= !wport_rvalid[m].read();
        }
        CHECK(all_rport_gnt_low, "T01a rport_gnt_o[*]=0 after reset");
        CHECK(all_rport_rv_low, "T01b rport_rvalid_o[*]=0 after reset");
        CHECK(all_wport_gnt_low, "T01c wport_gnt_o[*]=0 after reset");
        CHECK(all_wport_rv_low, "T01d wport_rvalid_o[*]=0 after reset");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: Write then read — data integrity ===");
        // -------------------------------------------------------------------
        do_reset();

        const data_t wr_data = make_row(0xDEADBEEFU);
        do_write(0, 0x00, wr_data);

        data_t rd_data = do_read(0, 0x00);
        CHECK(rd_data == wr_data, "T02 rdata matches wdata written via wport[0]");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: Full conflict — all rport buses read the same address ===");
        // -------------------------------------------------------------------
        do_reset();

        const data_t conflict_data = make_row(0xCAFEBABEU);
        do_write(0, 0x00, conflict_data);

        // Assert all NR requests to the same address.
        for (int m = 0; m < NR; ++m) {
            rport_req[m].write(true);
            rport_addr[m].write(0x00);
            rport_we[m].write(false);
            rport_be[m].write(FULL_BE);
        }

        // In the first tick, not all buses can be served (only the single
        // winner per bank makes it through).  Collect first-tick rvalids to
        // confirm serialisation is actually happening.
        tick();
        int first_rvalid = 0;
        for (int m = 0; m < NR; ++m)
            if (rport_rvalid[m].read())
                ++first_rvalid;
        CHECK(first_rvalid < NR, "T03a first tick serves fewer than NR buses (serialised)");

        // Deassert those already served; keep remaining buses requesting.
        // One bus is served per cycle — 8 buses need ≤ NR+4 cycles total.
        bool done[NR] = {};
        int  n_done   = first_rvalid;
        for (int m = 0; m < NR; ++m) {
            if (rport_rvalid[m].read()) {
                CHECK(rport_rdata[m].read() == conflict_data, "T03b conflict read: rdata correct");
                done[m] = true;
                idle_rport(m);
            }
        }

        for (int iter = 0; iter < NR + 4 && n_done < NR; ++iter) {
            tick();
            for (int m = 0; m < NR; ++m) {
                if (!done[m] && rport_rvalid[m].read()) {
                    CHECK(rport_rdata[m].read() == conflict_data,
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
            nc_data[m] = make_row(0x10000000U * (m + 1));
            do_write(0, ADDR_NC[m], nc_data[m]);
        }

        // Assert all NR reads simultaneously.
        for (int m = 0; m < NR; ++m) {
            rport_req[m].write(true);
            rport_addr[m].write(ADDR_NC[m]);
            rport_we[m].write(false);
            rport_be[m].write(FULL_BE);
        }

        // Combinatorial grants: all NR buses should see gnt=1 at the same time.
        wait(1, SC_NS);
        int all_gnt = 0;
        for (int m = 0; m < NR; ++m)
            if (rport_gnt[m].read())
                ++all_gnt;
        CHECK(all_gnt == NR, "T04a all NR buses granted simultaneously (no conflict)");

        // One tick: all NR banks process in parallel → all rvalid together.
        tick();
        int all_rv = 0;
        for (int m = 0; m < NR; ++m)
            if (rport_rvalid[m].read())
                ++all_rv;
        CHECK(all_rv == NR, "T04b all NR rvalid signals fired in the same cycle");

        bool data_ok = true;
        for (int m = 0; m < NR; ++m) {
            if (!(rport_rdata[m].read() == nc_data[m])) {
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
            // Fill all four 32-bit words with the same value so make_row() applies.
            t05_wdata[i] = make_row(0x01010101U * static_cast<uint32_t>(i + 1));
            do_write(0, static_cast<uint64_t>(i) * ADDR_STEP, t05_wdata[i]);
        }

        bool t05_order_ok = true;
        bool t05_data_ok  = true;
        for (int i = 0; i < N_ADDRS; ++i) {
            const uint64_t addr = static_cast<uint64_t>(i) * ADDR_STEP;
            rport_req[0].write(true);
            rport_addr[0].write(addr);
            rport_we[0].write(false);
            rport_be[0].write(FULL_BE);
            tick();
            if (!rport_rvalid[0].read()) {
                t05_order_ok = false;
                std::printf("  FAIL  T05: no rvalid for addr 0x%03llx (i=%d)\n",
                            (unsigned long long)addr, i);
                ++g_fail;
            } else {
                const data_t rd = rport_rdata[0].read();
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

        const data_t t06_first  = make_row(0xDEADBEEFU);
        const data_t t06_second = make_row(0x01234567U);
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

        static constexpr int N_PAT = 8;
        const data_t         t07_aa = make_row(0xAAAAAAAAU);
        const data_t         t07_55 = make_row(0x55555555U);
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
        std::puts("\n=== T08: Cross-address independence — writes to set B do not corrupt set A ===");
        // -------------------------------------------------------------------
        // Write set A (lower addresses) with payload_A, then write set B (higher
        // addresses) with payload_B.  Re-read set A: it must still hold payload_A.
        // This catches address aliasing and row-collision bugs.
        do_reset();

        static constexpr int N_AB    = 8;
        const uint64_t       BASE_A  = 0x000;
        const uint64_t       BASE_B  = 0x200;  // well-separated to avoid accidental aliasing
        data_t               t08_A[N_AB], t08_B[N_AB];
        for (int i = 0; i < N_AB; ++i) {
            t08_A[i] = make_row(0xA0000001U + static_cast<uint32_t>(i));
            t08_B[i] = make_row(0xB0000001U + static_cast<uint32_t>(i));
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
        std::puts("\n=== T09: Port response isolation — rvalid fires only on the requesting port ===");
        // -------------------------------------------------------------------
        // Assert a request on exactly one port.  No other port's rvalid/gnt should
        // assert as a side-effect (OBI: each response targets its own initiator).
        do_reset();

        // wport side: only wport[0] requests
        for (int m = 1; m < NW; ++m)
            idle_wport(m);
        wport_req[0].write(true);
        wport_addr[0].write(0x00);
        wport_we[0].write(true);
        wport_be[0].write(FULL_BE);
        wport_wdata[0].write(make_row(0xFEDCBA98U));
        tick();
        CHECK(wport_rvalid[0].read(), "T09a wport[0] rvalid fired");
        {
            bool no_leak = true;
            for (int m = 1; m < NW; ++m)
                no_leak &= !wport_rvalid[m].read() && !wport_gnt[m].read();
            CHECK(no_leak, "T09b no other wport rvalid/gnt asserted");
        }
        idle_wport(0);

        // rport side: only rport[0] requests
        for (int m = 1; m < NR; ++m)
            idle_rport(m);
        rport_req[0].write(true);
        rport_addr[0].write(0x00);
        rport_we[0].write(false);
        rport_be[0].write(FULL_BE);
        tick();
        CHECK(rport_rvalid[0].read(), "T09c rport[0] rvalid fired");
        {
            bool no_leak = true;
            for (int m = 1; m < NR; ++m)
                no_leak &= !rport_rvalid[m].read() && !rport_gnt[m].read();
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
            t10_data[m] = make_row(0x10101010U * static_cast<uint32_t>(m + 1));
            wport_req[m].write(true);
            wport_addr[m].write(t10_addr[m]);
            wport_we[m].write(true);
            wport_be[m].write(FULL_BE);
            wport_wdata[m].write(t10_data[m]);
        }
        {
            bool w_done[NW] = {};
            int  w_got      = 0;
            for (int iter = 0; iter < NW * 4 && w_got < NW; ++iter) {
                tick();
                for (int m = 0; m < NW; ++m) {
                    if (!w_done[m] && wport_rvalid[m].read()) {
                        w_done[m] = true;
                        ++w_got;
                        idle_wport(m);
                    }
                }
            }
            CHECK(w_got == NW, "T10a all NW parallel writes completed");
        }

        for (int m = 0; m < NR; ++m) {
            rport_req[m].write(true);
            rport_addr[m].write(t10_addr[m]);
            rport_we[m].write(false);
            rport_be[m].write(FULL_BE);
        }
        {
            bool r_done[NR]    = {};
            bool r_data_ok[NR] = {};
            int  r_got         = 0;
            for (int iter = 0; iter < NR * 4 && r_got < NR; ++iter) {
                tick();
                for (int m = 0; m < NR; ++m) {
                    if (!r_done[m] && rport_rvalid[m].read()) {
                        r_data_ok[m] = (rport_rdata[m].read() == t10_data[m]);
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
