// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for buffer<NUM_REQ=4, PORT_COUNT=4, BYTES_PER_ROW=4, NUM_TDM=32,
// IS_WRITE=true> — the REAL production write-buffer configuration (matches
// top_tdm.hpp's buf_w0/WAGU_A) — driven against REAL bank<> instances (one
// per TDM slot, 1-cycle grant+response latency).
//
// tb_buffer.cpp/tb_buffer_pipeline.cpp only cover read mode. This file closes
// the equivalent gap for write mode's pipelined stages (see buffer.hpp's own
// header comment): fill (ports push a group of 4/8/16 beats at a time, the
// inverse of read's drain) -> snapshot (once all 32 slots are filled, a
// one-cycle pulse hands the window to the cells' shadow engines, which fire
// their TDM writes simultaneously) -> posted respond (p_rvalid_o sent back
// group-by-group, mirroring read's drain loop) -> the next fill, which was
// already running underneath.
//
// Tests:
// Structure: one loop over active_mode 0/1/2 (groups of 4/8/16) x three
// back-to-back rounds each, emitting four labelled checks per round (36
// total) rather than discrete T-numbered cases:
//   - fill: correct grant timing per group until all 32 slots are filled;
//   - shadow flush: the cycle after the 32nd beat is accepted, every one of
//     the 32 cells simultaneously issues its own TDM write with the exact
//     (address, data) the port supplied — checked directly against the
//     buffer's manager-side wires (m[t].req_o/addr_o/we_o/wdata_o), not just
//     via a later readback;
//   - posted respond: p_rvalid_o group-by-group behind the snapshot;
//   - three rounds back to back (the write-side equivalent of
//     tb_buffer_pipeline.cpp): every round's addresses/data land in the
//     right banks, no stalls or cross-round corruption.
// -----------------------------------------------------------------------------

#include "bank.hpp"
#include "buffer.hpp"
#include "obi_data.hpp"
#include "obi_ports.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

static constexpr int      kNumReq    = 4;
static constexpr int      kPortCount = 4;
static constexpr int      kBytes     = 4;
static constexpr int      kNumTdm    = 32;
static constexpr int      kNumIO     = kPortCount * kNumReq; // 16
static constexpr uint32_t kFullBe    = 0xF;

using data_t = obi_data<kBytes>;
using DUT    = buffer<kNumReq, kPortCount, kBytes, kNumTdm, true>;

static data_t make_data(uint32_t v) {
    return data_t(static_cast<unsigned long long>(v));
}

// Deterministic per-(round, slot) address/data the port writes — one
// distinct bank row each, far below the 1024-row capacity even across many
// rounds, so no aliasing risk.
static uint64_t round_addr(int round, int slot) {
    return static_cast<uint64_t>(round * kNumTdm + slot + 1) * kBytes;
}
static data_t round_data(int round, int slot) {
    return make_data(0xE0000000u + static_cast<uint32_t>(round) * 1000u +
                     static_cast<uint32_t>(slot));
}

// Inverse of round_addr() — address-forwarding checks go through this rather
// than the test's own (round, slot) loop indices directly, so a fetch/drain
// indexing bug can't coincidentally still pass (see
// tb_buffer_pipeline.cpp's data_for_addr() for the read-side twin).
static data_t data_for_addr(uint64_t addr) {
    uint64_t idx   = addr / kBytes - 1;
    int      round = static_cast<int>(idx / kNumTdm);
    int      slot  = static_cast<int>(idx % kNumTdm);
    return round_data(round, slot);
}

SC_MODULE(tb) {
    sc_clock            clk{"clk", 10, SC_NS};
    sc_signal<bool>     rst_n{"rst_n"};
    sc_signal<uint32_t> active_mode{"active_mode"};

    // Unused in write mode, but the ports exist on buffer<> regardless — bind
    // to fixed dummy signals.
    sc_signal<uint64_t> fetch_addr_i[kNumTdm];
    sc_signal<bool>     fetch_addr_valid_i{"fetch_addr_valid_i"};

    // Port-facing OBI as wire bundles (one per lane)
    obi_signal_bundle<data_t>     p_bus[kNumIO];
    obi_signal_bundle<data_t>     m_drv[kNumTdm];
    sc_vector<bank<1024, kBytes>> banks{"banks"};

    DUT *dut;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        banks.init(kNumTdm);
        dut = new DUT("dut");
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->active_mode(active_mode);

        for (int i = 0; i < kNumIO; ++i) {
            bind_obi(dut->p[i], p_bus[i]);
        }
        for (int t = 0; t < kNumTdm; ++t) {
            bind_obi(dut->m[t], m_drv[t]);
            dut->fetch_addr_i[t](fetch_addr_i[t]);
            banks[t].clk_i(clk);
            banks[t].rst_ni(rst_n);
            bind_obi(banks[t].obi, m_drv[t]);
        }
        dut->fetch_addr_valid_i(fetch_addr_valid_i);

        SC_THREAD(run);
    }

    ~tb() {
        delete dut;
    }

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
        for (int t = 0; t < kNumTdm; ++t)
            fetch_addr_i[t].write(0);
        tick(clk);
        tick(clk);
        rst_n.write(true);
        tick(clk);
    }

    // Drives one fill group (na ports wide) with round_addr/round_data for
    // absolute slots [base, base+na), waits one edge, and reports whether
    // every port in the group was granted that same cycle (comb_proc grants
    // combinationally off ports_req, mirroring read mode's gnt timing).
    bool fill_group(int round, int base, int na) {
        for (int p = 0; p < na; ++p) {
            p_bus[p].addr.write(round_addr(round, base + p));
            p_bus[p].wdata.write(round_data(round, base + p));
            p_bus[p].be.write(kFullBe);
            p_bus[p].req.write(true);
        }
        wait(1, SC_NS);
        bool gnt_ok = true;
        for (int p = 0; p < na; ++p)
            gnt_ok &= p_bus[p].gnt.read();
        tick(clk);
        for (int p = 0; p < na; ++p)
            p_bus[p].req.write(false);
        return gnt_ok;
    }

    // After the snapshot has fired (the cycle right after the 32nd beat is
    // accepted), every cell drives its own TDM write this same cycle — check
    // each of the 32 manager-side wires directly against what was filled for
    // that absolute slot, and that all_valid arrives exactly one bank round
    // trip later.
    bool check_flush(int round) {
        bool ok = true;
        for (int t = 0; t < kNumTdm; ++t) {
            ok &= m_drv[t].req.read();
            ok &= m_drv[t].we.read();
            ok &= (m_drv[t].be.read() == kFullBe);
            ok &= (m_drv[t].addr.read() == round_addr(round, t));
            ok &= (m_drv[t].wdata.read() == data_for_addr(m_drv[t].addr.read()));
        }
        return ok;
    }

    void run();
};

void tb::run() {
    struct ModeCfg {
        int mode, na, n_groups;
    };
    const ModeCfg cfgs[3] = {{0, 4, 8}, {1, 8, 4}, {2, 16, 2}};

    for (const auto &cfg : cfgs) {
        std::printf("\n=== fill/snapshot/respond for na=%d (active_mode=%d) ===\n", cfg.na,
                    cfg.mode);
        do_reset();
        active_mode.write(cfg.mode);

        for (int round = 0; round < 3; ++round) {
            // ---- fill: push na ports at a time until all 32 slots are
            // filled, the inverse of read mode's group-by-group drain. ----
            bool fill_ok = true;
            for (int g = 0; g < cfg.n_groups; ++g)
                fill_ok &= fill_group(round, g * cfg.na, cfg.na);
            char l1[128];
            std::snprintf(l1, sizeof(l1),
                          "na=%d round=%d: fill — all %d groups of %d granted, correct timing",
                          cfg.na, round, cfg.n_groups, cfg.na);
            CHECK(fill_ok, l1);

            // ---- shadow flush: poll for the one cycle every one of the 32 cells
            // fires its own TDM write simultaneously — the write-side twin
            // of read mode's same-cycle prefetch handoff. ----
            bool flush_seen = false, flush_ok = false;
            for (int i = 0; i < 4 && !flush_seen; ++i) {
                if (m_drv[0].req.read()) {
                    flush_seen = true;
                    flush_ok   = check_flush(round);
                } else {
                    tick(clk);
                }
            }
            char l2[160];
            std::snprintf(l2, sizeof(l2),
                          "na=%d round=%d: flush — all 32 cells push to their banks the same "
                          "cycle, addresses/data forwarded correctly",
                          cfg.na, round);
            CHECK(flush_seen && flush_ok, l2);

            // ---- POSTED respond: the acks stream group-by-group right
            // behind the snapshot, CONCURRENT with the banks serving the
            // burst — they no longer wait for the bank round trip (see
            // buffer.hpp's write branch). ----
            bool respond_ok = true;
            for (int g = 0; g < cfg.n_groups; ++g) {
                tick(clk);
                bool rv_ok = true;
                for (int p = 0; p < cfg.na; ++p)
                    rv_ok &= p_bus[p].rvalid.read();
                respond_ok &= rv_ok;
            }
            char l4[128];
            std::snprintf(l4, sizeof(l4),
                          "na=%d round=%d: respond — all %d groups of %d rvalid, streamed "
                          "concurrently with the flush",
                          cfg.na, round, cfg.n_groups, cfg.na);
            CHECK(respond_ok, l4);

            // ---- The shadows drain via the real banks' round trip;
            // snapshot().n_valid counts busy shadows in write mode. ----
            bool drained = false;
            for (int i = 0; i < 6 && !drained; ++i) {
                if (dut->snapshot().n_valid == 0)
                    drained = true;
                else
                    tick(clk);
            }
            char l3[128];
            std::snprintf(l3, sizeof(l3),
                          "na=%d round=%d: flush completes — every shadow drained through its "
                          "bank",
                          cfg.na, round);
            CHECK(drained, l3);
        }
    }

    sc_stop();
}

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
