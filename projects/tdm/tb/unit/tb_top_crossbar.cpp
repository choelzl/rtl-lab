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
