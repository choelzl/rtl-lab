// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Unit tests for agu<> (tb/systemc/agu.hpp) — the testbench-side AGU driver.
// It's TB infrastructure, but its lookahead cursor / fence / window-rounding
// logic is the most intricate code that until now had no isolated coverage
// (every prior bug hunt in it went through full stim_bank integration runs) —
// this pins the parsing and cursor contracts directly.
//
// Configuration under test: agu<NUM_REQ=4, uint64_t, 4, N_PER_GROUP=1>,
// target=tdm, tdm_window=8 — small enough to hand-trace: a 2-lane task has
// 8/2 = 4 groups per window, a 4-lane task has 8/4 = 2.
//
// Tests:
//   T01: trace parsing — task headers (full + short descriptor), ports_used
//        scaling, CRL/has_crl, fence cycles, address lines
//   T02: n_groups window rounding — a task's real trace is padded up to a
//        whole TDM window's worth of groups
//   T03: lookahead accessors — ready flag, per-lane window slice with
//        zero-padding past the trace, la-synchronized config
//   T04: advance_lookahead_window — jumps a full window stride, then parks
//        (not ready) behind the next task's start_cycle fence
//   T05: retry_lookahead_fence — a no-op while fenced, rolls into the next
//        task once cycle_ passes the fence
//   T06: capture side end to end — groups drain in order against served
//        rvalid responses, NOP padding lanes request with addr=0 and are
//        never logged, the fenced second task completes, done_o rises, and
//        log_ holds exactly the real (addr, data) pairs in order
// -----------------------------------------------------------------------------

#include "agu.hpp"
#include "obi_ports.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <fstream>
#include <systemc.h>

static constexpr int         kNumReq    = 4;
static constexpr std::size_t kTdmWindow = 8;
static const char           *kTracePath = "/tmp/tb_agu_trace.txt";

using data_t = uint64_t;
using DUT    = agu<kNumReq, data_t, 4, /*N_PER_GROUP=*/1>;

// Task 0: 2 lanes, full descriptor, 6 addresses -> 3 real groups, rounded to
//         4 (one 8-cell window at 2 lanes/group).
// Task 1: 4 lanes, short descriptor (no CRL), fenced at cycle 50,
//         4 addresses -> 1 real group, rounded to 2.
static void write_trace() {
    std::ofstream f(kTracePath);
    f << "#0, 2, 4, 4, 8, 0\n"; // #cycle, napa, R, C, L, storemode
    f << "addr\n";              // optional header line, must be skipped
    for (int i = 0; i < 6; ++i)
        f << "0x" << std::hex << (0x100 + 4 * i) << std::dec << "\n";
    f << "#50, 4, 1\n";
    for (int i = 0; i < 4; ++i)
        f << "0x" << std::hex << (0x200 + 4 * i) << std::dec << "\n";
}

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};
    sc_signal<bool> done{"done"};

    obi_signal_bundle<data_t> obi[kNumReq];

    DUT *dut;

    // Second instance fed a nonexistent trace — pins the empty-trace
    // contract (immediate done, silent ports).
    sc_signal<bool>           done_empty{"done_empty"};
    obi_signal_bundle<data_t> obi_empty[kNumReq];
    DUT                      *dut_empty;

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        write_trace();
        dut = new DUT("dut", kTracePath, "", agu_target::tdm, kTdmWindow);
        dut->clk_i(clk);
        dut->rst_ni(rst_n);
        dut->done_o(done);
        for (int p = 0; p < kNumReq; ++p)
            bind_obi(dut->obi[p], obi[p]);
        dut_empty =
            new DUT("dut_empty", "/nonexistent/agu_trace.log", "", agu_target::tdm, kTdmWindow);
        dut_empty->clk_i(clk);
        dut_empty->rst_ni(rst_n);
        dut_empty->done_o(done_empty);
        for (int p = 0; p < kNumReq; ++p)
            bind_obi(dut_empty->obi[p], obi_empty[p]);
        SC_THREAD(run);
    }
    ~tb() {
        delete dut;
        delete dut_empty;
    }

    // Serve one captured group: assert rvalid+rdata on the given lanes for
    // one edge (the tdm capture path needs no gnt handshake — the buffer
    // grants upstream; this AGU only waits for responses).
    void serve(int lanes, uint64_t data_base) {
        for (int p = 0; p < lanes; ++p) {
            obi[p].rvalid.write(true);
            obi[p].rdata.write(data_base + static_cast<uint64_t>(p));
        }
        tick(clk);
        for (int p = 0; p < lanes; ++p) {
            obi[p].rvalid.write(false);
            obi[p].rdata.write(0);
        }
    }

    void run() {
        // -------------------------------------------------------------------
        std::puts("\n=== T01: trace parsing ===");
        // -------------------------------------------------------------------
        CHECK(dut->tasks_.size() == 2, "T01a two task descriptors parsed");
        CHECK(dut->tasks_[0].ports_used == 2,
              "T01b task0 ports_used = num_port_active * N_PER_GROUP");
        CHECK(dut->tasks_[0].has_crl && dut->tasks_[0].R == 4 && dut->tasks_[0].C == 4 &&
                  dut->tasks_[0].L == 8 && dut->tasks_[0].store_mode == 0,
              "T01c task0 full descriptor carries R/C/L/storemode");
        CHECK(dut->tasks_[0].trace.size() == 6, "T01d task0 has its 6 address lines");
        CHECK(dut->tasks_[0].trace[2].addr == 0x108, "T01e address lines parsed in order");
        CHECK(dut->tasks_[1].ports_used == 4 && !dut->tasks_[1].has_crl,
              "T01f task1 short descriptor: 4 lanes, no CRL");
        CHECK(dut->tasks_[1].start_cycle == 50, "T01g task1 fence cycle parsed");

        // -------------------------------------------------------------------
        std::puts("\n=== T02: n_groups window rounding ===");
        // -------------------------------------------------------------------
        CHECK(dut->tasks_[0].n_groups == 4,
              "T02a task0: 6 addrs / 2 lanes = 3 groups, rounded to a 4-group window");
        CHECK(dut->tasks_[1].n_groups == 2,
              "T02b task1: 4 addrs / 4 lanes = 1 group, rounded to a 2-group window");
        CHECK(dut->n_groups_ == 6, "T02c total groups across tasks");

        // -------------------------------------------------------------------
        std::puts("\n=== T03: lookahead accessors ===");
        // -------------------------------------------------------------------
        CHECK(dut->lookahead_ready(), "T03a ready at task0 window 0");
        bool slice_ok = true;
        for (int w = 0; w < 6; ++w)
            slice_ok &= (dut->lookahead_addr(w) == 0x100u + 4u * static_cast<unsigned>(w));
        CHECK(slice_ok, "T03b window slice = the next tdm_window addresses in trace order");
        CHECK(dut->lookahead_addr(6) == 0 && dut->lookahead_addr(7) == 0,
              "T03c zero-padded past the end of the trace");
        CHECK(dut->lookahead_ports_used() == 2 && dut->lookahead_C() == 4,
              "T03d la-synchronized config reflects task0");

        // -------------------------------------------------------------------
        std::puts("\n=== T04: advance_lookahead_window strides and parks at the fence ===");
        // -------------------------------------------------------------------
        dut->advance_lookahead_window(); // one window = 8/2 = 4 groups = all of task0
        CHECK(!dut->lookahead_ready(),
              "T04a cursor crossed task0 but task1's fence (cycle 50) blocks the rollover");
        CHECK(dut->lookahead_ports_used() == 2,
              "T04b la config still task0's while parked at the fence");

        // -------------------------------------------------------------------
        std::puts("\n=== T05: retry_lookahead_fence unblocks once the fence passes ===");
        // -------------------------------------------------------------------
        dut->retry_lookahead_fence();
        CHECK(!dut->lookahead_ready(), "T05a retry is a no-op while still fenced");
        rst_n.write(false);
        tick(clk);
        rst_n.write(true);               // cycle_ counts up from here (also resets la to task0)
        dut->advance_lookahead_window(); // re-park the cursor at the fence
        for (int i = 0; i < 55; ++i)
            tick(clk); // capture side stalls at task0 group 0 (no rvalid served) — harmless
        dut->retry_lookahead_fence();
        // task0 (2 lanes) -> task1 (4 lanes) changes the buffer geometry, so
        // the roll is additionally held until the capture side has finished
        // task0 — see agu.hpp's la_task_roll_gate_open_ comment and
        // tb_task_boundary.cpp for the production deadlock this prevents.
        CHECK(!dut->lookahead_ready(),
              "T05b geometry-changing roll held while capture is still mid-task");
        dut->group_ = dut->tasks_[0].n_groups; // capture completes task0 (simulated)
        dut->retry_lookahead_fence();
        CHECK(dut->lookahead_ready(), "T05c fence + capture done: cursor rolled into task1");
        CHECK(dut->lookahead_ports_used() == 4, "T05d la config now task1's");
        CHECK(dut->lookahead_addr(0) == 0x200, "T05e la window is task1's first slice");
        dut->group_ = 0; // hand capture back to T06 at task0 group 0

        // -------------------------------------------------------------------
        std::puts("\n=== T06: capture side end to end ===");
        // -------------------------------------------------------------------
        // The 55 idle ticks above left task0's capture waiting at group 0
        // with both real lanes requesting.
        CHECK(obi[0].req.read() && obi[1].req.read(), "T06a task0 lanes 0-1 requesting");
        CHECK(!obi[2].req.read() && !obi[3].req.read(), "T06b lanes beyond ports_used quiet");
        CHECK(obi[0].addr.read() == 0x100 && obi[1].addr.read() == 0x104,
              "T06c group 0 addresses on the bus");
        serve(2, 0xD000); // group 0
        serve(2, 0xD010); // group 1
        tick(clk);        // step() re-drives the bus for the group it advanced to
        CHECK(obi[0].addr.read() == 0x110 && obi[1].addr.read() == 0x114,
              "T06d group 2 (last real) addresses on the bus");
        serve(2, 0xD020); // group 2
        tick(clk);
        CHECK(obi[0].req.read() && obi[0].addr.read() == 0,
              "T06e padding group requests with addr=0 (NOP)");
        serve(2, 0xD030); // group 3 = padding
        // cycle_ is already past task1's fence (T05 ticked to ~57), so task1
        // starts immediately: 4 lanes, group 0 real, group 1 padding.
        tick(clk);
        CHECK(obi[3].req.read() && obi[3].addr.read() == 0x20c,
              "T06f task1 group 0 on all 4 lanes");
        serve(4, 0xE000);
        serve(4, 0xE010); // padding group
        tick(clk);
        CHECK(done.read(), "T06g done_o after the last task's last group");
        CHECK(dut->log_.size() == 10, "T06h log holds exactly the 10 REAL accesses (no padding)");
        bool log_ok = dut->log_.size() == 10;
        if (log_ok) {
            for (int i = 0; i < 6; ++i)
                log_ok &= (dut->log_[i].addr == 0x100u + 4u * static_cast<unsigned>(i)) &&
                          (dut->log_[i].data == 0xD000u + 0x10u * static_cast<unsigned>(i / 2) +
                                                    static_cast<unsigned>(i % 2)) &&
                          !dut->log_[i].we;
            for (int i = 0; i < 4; ++i)
                log_ok &= (dut->log_[6 + i].addr == 0x200u + 4u * static_cast<unsigned>(i)) &&
                          (dut->log_[6 + i].data == 0xE000u + static_cast<unsigned>(i));
        }
        CHECK(log_ok, "T06i log entries: right addresses, right data, in trace order");

        // -------------------------------------------------------------------
        std::puts("\n=== T07: empty/missing trace — immediately done, silent ===");
        // -------------------------------------------------------------------
        CHECK(dut_empty->tasks_.empty(), "T07a no tasks parsed from a missing file");
        CHECK(done_empty.read(), "T07b done_o is already high");
        {
            bool silent = true;
            for (int c = 0; c < 3; ++c) {
                tick(clk);
                for (int p2 = 0; p2 < kNumReq; ++p2)
                    silent &= !obi_empty[p2].req.read();
            }
            CHECK(silent, "T07c never issues a request");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T08: lookahead zero-pads with no trace at all ===");
        // -------------------------------------------------------------------
        {
            // The empty instance has nothing to expose: every lookahead slot
            // must read as the addr=0 NOP sentinel (the same zero-padding
            // that fills a real trace's tail).
            bool zeros = true;
            for (int w = 0; w < static_cast<int>(kTdmWindow); ++w)
                zeros &= (dut_empty->lookahead_addr(w) == 0);
            CHECK(zeros, "T08 empty trace exposes an all-NOP lookahead window");
        }

        // -------------------------------------------------------------------
        std::puts("\n=== T09: ports_used tracked across the task rollover ===");
        // -------------------------------------------------------------------
        CHECK(dut->ports_used_ == dut->tasks_[1].ports_used,
              "T09 public ports_used_ reflects the LAST task after rollover (4 lanes)");

        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
