// -----------------------------------------------------------------------------
// Task-boundary unit tests for the TDM read path (agu<> + buffer<> + top<>).
//
// Production stimuli (tb/stimuli/final/*) exposed a family of task-boundary
// deadlocks the synthetic stim_bank sweep never hits, because its phases pad
// every task to whole windows and fence generously. This suite reproduces
// those boundaries minimally and deterministically, so the failure mechanism
// is visible in one place and the fix is regression-locked:
//
//   T-width  : consecutive tasks with DIFFERENT port widths and a tight
//              fence — the boundary lands on/near a window wrap, where the
//              buffer re-latches its group width from active_mode at the
//              wrap edge while the window's CONTENT arrives separately over
//              the preceding drain (fed by the lookahead cursor). Geometry
//              and content straddling the boundary is a permanent
//              width-mismatch deadlock unless every geometry-changing
//              transition is forced through the buffer's atomic all-idle
//              boot latch (see agu.hpp's task-roll gating).
//   T-napa3 : a num_port_active=3 task — the buffer's active_mode encoding
//              has no 3-group state; agu.hpp must pad requests up to the
//              rounded 4-group width (rounded_ports_used()).
//   T-flush : a task whose trace ends mid-window followed by a fenced gap —
//              the buffer's prefetch leaves residual valid cells the AGU
//              never asks for; the between-tasks flush must drain them so
//              the next task can boot-latch.
//   T-same  : consecutive tasks with IDENTICAL geometry — these must keep
//              streaming seamlessly (no forced idle), guarding the fix
//              against over-serializing the common case.
//
// The whole scenario list is ONE ragu_a trace (one elaboration per process);
// verification is per-task: every trace address must appear in the AGU's
// response log exactly once, and the whole run must finish within a budget
// that any non-deadlocked execution meets with huge margin.
//
// Built with the ADAPTIVE arbiter (the production configuration — see
// doc/report §6): its same-cycle grants give the tightest boundary timing,
// which is exactly what these tests need to exercise.
// -----------------------------------------------------------------------------

#define IMPL_TDM
#define IMPL_ARB_ADAPTIVE

// top.hpp must precede constants.hpp — see tb_top.cpp for why.
#include "top.hpp"

#include <systemc.h>

#include "obi_data.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <unistd.h>
#include <vector>

#include "agu.hpp"
#include "constants.hpp"
#include "lane_agu.hpp"

using dut_t  = top<N_BANK, N_ROW, WORD_BYTES, WORDS_PER_ROW>;
using data_t = obi_data<BYTES_PER_ROW>;

#include "agu_bind_util.hpp"

static int  g_pass = 0, g_fail = 0;
static void CHECK(bool ok, const char *label) {
    std::printf("  %s  %s\n", ok ? "PASS" : "FAIL", label);
    (ok ? g_pass : g_fail)++;
}

// One task of the crafted scenario trace.
struct spec_t {
    const char *label;       // which boundary pattern this task's ENTRY exercises
    uint64_t    start_cycle; // fence
    int         napa;        // num_port_active (1..4; 3 exercises the rounding)
    uint64_t    C, R, L, sm; // map geometry
    int         lines;       // address lines
};

int sc_main(int, char *[]) {
    // ---- craft the scenario trace -----------------------------------------
    // Tight fences: each task starts shortly after the previous one's data
    // would naturally finish, so boundaries land inside the buffer's window
    // turnaround — the exact conditions the production traces hit.
    // Fence placement is the whole test. Two kinds of boundaries:
    //   start_cycle=1  : never gates — a pure lookahead-roll transition (la
    //                    crosses into the next task at a wrap DURING the
    //                    previous task's drain).
    //   start_cycle=6  : THE hazard — scanning every fence offset for the
    //                    warmup below showed exactly one deadlocking cycle,
    //                    the fence clearing precisely at the warmup's last
    //                    window-wrap edge (all other offsets complete). This
    //                    reproduces the production deadlock (doc/report A.6
    //                    Cause 2b) deterministically: the buffer re-latches
    //                    its group width at that wrap from active_mode —
    //                    which the harness, seeing the wrap only via the
    //                    registered window_reset one cycle later, cannot
    //                    possibly have updated yet — while the cells go on
    //                    to fetch the NEXT task's (narrower) content into
    //                    that wider-latched window.
    const std::vector<spec_t> specs = {
        {"warmup 4-wide steady state", 1, 4, 4, 4, 8, 0, 64},
        {"width 4->1, fence ON the wrap edge (hazard)", 6, 1, 4, 4, 8, 0, 8},
        {"width 1->4, back-to-back", 1, 4, 4, 4, 8, 0, 32},
        {"width 4->3 (napa=3 rounding), back-to-back", 1, 3, 4, 4, 8, 0, 24},
        {"width 3->2 with C/R/L change, back-to-back", 1, 2, 8, 8, 4, 1, 32},
        {"same geometry back-to-back (must stream)", 1, 2, 8, 8, 4, 1, 32},
        {"width 2->1 mid-window tail (flush), back-to-back", 1, 1, 4, 4, 8, 0, 4},
        {"width 1->4 after flush residue", 1, 4, 4, 4, 8, 0, 32},
    };

    const std::string dir = std::string("/tmp/tb_task_boundary_") + std::to_string(::getpid());
    std::filesystem::create_directories(dir);
    std::vector<std::set<uint64_t>> task_addrs(specs.size());
    {
        std::ofstream f(dir + "/ragu_a.log");
        for (std::size_t k = 0; k < specs.size(); ++k) {
            const spec_t &sp = specs[k];
            f << "#" << sp.start_cycle << "," << sp.napa << "," << sp.C << "," << sp.R << ","
              << sp.L << "," << sp.sm << "\n";
            for (int j = 0; j < sp.lines; ++j) {
                const uint64_t a = 0x100000ull * (k + 1) + static_cast<uint64_t>(j) * BYTES_PER_ROW;
                task_addrs[k].insert(a);
                f << "0x" << std::hex << a << std::dec << "\n";
            }
        }
    }

    // ---- system under test (mirrors tb_top.cpp's TDM wiring) ---------------
    sc_clock        clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool> rst_ni;

    dut_t dut("dut");
    dut.clk_i(clk);
    dut.rst_ni(rst_ni);

    obi_signal_bundle<data_t> ragu_a[dut_t::RAGU_A_PORTS];
    obi_signal_bundle<data_t> ragu_b[dut_t::RAGU_B_PORTS];
    obi_signal_bundle<data_t> ragu_c[dut_t::RAGU_C_PORTS];
    obi_signal_bundle<data_t> ragu_d[dut_t::RAGU_D_PORTS];
    obi_signal_bundle<data_t> ragu_e[dut_t::RAGU_E_PORTS];
    obi_signal_bundle<data_t> wagu_a[dut_t::WAGU_A_PORTS];
    obi_signal_bundle<data_t> wagu_b[dut_t::WAGU_B_PORTS];
    obi_signal_bundle<data_t> wagu_d[dut_t::WAGU_D_PORTS];
    obi_signal_bundle<data_t> wagu_e[dut_t::WAGU_E_PORTS];

    BIND_DUT_GROUP(dut, ragu_a, ragu_a, dut_t::RAGU_A_PORTS);
    BIND_DUT_GROUP(dut, ragu_b, ragu_b, dut_t::RAGU_B_PORTS);
    BIND_DUT_GROUP(dut, ragu_c, ragu_c, dut_t::RAGU_C_PORTS);
    BIND_DUT_GROUP(dut, ragu_d, ragu_d, dut_t::RAGU_D_PORTS);
    BIND_DUT_GROUP(dut, ragu_e, ragu_e, dut_t::RAGU_E_PORTS);
    BIND_DUT_GROUP(dut, wagu_a, wagu_a, dut_t::WAGU_A_PORTS);
    BIND_DUT_GROUP(dut, wagu_b, wagu_b, dut_t::WAGU_B_PORTS);
    BIND_DUT_GROUP(dut, wagu_d, wagu_d, dut_t::WAGU_D_PORTS);
    BIND_DUT_GROUP(dut, wagu_e, wagu_e, dut_t::WAGU_E_PORTS);

    sc_signal<bool> done[9];

    constexpr std::size_t                                            tdm_window = N_BANK;
    agu<dut_t::RPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_a_src(
        "ragu_a", dir + "/ragu_a.log", "", agu_target::tdm, tdm_window);
    agu<dut_t::RPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_b_src(
        "ragu_b", dir + "/ragu_b.log", "", agu_target::tdm, tdm_window);
    agu<dut_t::RPORT_C_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_c_src(
        "ragu_c", dir + "/ragu_c.log", "", agu_target::tdm, tdm_window);
    lane_agu<data_t, BYTES_PER_ROW> ragu_d_src("ragu_d", dir + "/ragu_d.log", "",
                                               lane_agu_dir::read, tdm_window);
    lane_agu<data_t, BYTES_PER_ROW> ragu_e_src("ragu_e", dir + "/ragu_e.log", "",
                                               lane_agu_dir::read, tdm_window);
    agu<dut_t::WPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> wagu_a_src(
        "wagu_a", dir + "/wagu_a.log", "", agu_target::crossbar, tdm_window);
    agu<dut_t::WPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> wagu_b_src(
        "wagu_b", dir + "/wagu_b.log", "", agu_target::crossbar, tdm_window);
    lane_agu<data_t, BYTES_PER_ROW> wagu_d_src("wagu_d", dir + "/wagu_d.log", "",
                                               lane_agu_dir::write, tdm_window);
    lane_agu<data_t, BYTES_PER_ROW> wagu_e_src("wagu_e", dir + "/wagu_e.log", "",
                                               lane_agu_dir::write, tdm_window);

    bind_agu(ragu_a_src, clk, rst_ni, done[0], ragu_a);
    bind_agu(ragu_b_src, clk, rst_ni, done[1], ragu_b);
    bind_agu(ragu_c_src, clk, rst_ni, done[2], ragu_c);
    bind_agu(ragu_d_src, clk, rst_ni, done[3], ragu_d);
    bind_agu(ragu_e_src, clk, rst_ni, done[4], ragu_e);
    bind_agu(wagu_a_src, clk, rst_ni, done[5], wagu_a);
    bind_agu(wagu_b_src, clk, rst_ni, done[6], wagu_b);
    bind_agu(wagu_d_src, clk, rst_ni, done[7], wagu_d);
    bind_agu(wagu_e_src, clk, rst_ni, done[8], wagu_e);

    sc_signal<uint64_t> rd0_lookahead[N_BANK], rd1_lookahead[N_BANK], rd2_lookahead[N_BANK],
        rd3_lookahead[N_BANK], rd4_lookahead[N_BANK];
    for (int w = 0; w < N_BANK; ++w) {
        dut.impl.rd0_lookahead_i[w](rd0_lookahead[w]);
        dut.impl.rd1_lookahead_i[w](rd1_lookahead[w]);
        dut.impl.rd2_lookahead_i[w](rd2_lookahead[w]);
        dut.impl.rd3_lookahead_i[w](rd3_lookahead[w]);
        dut.impl.rd4_lookahead_i[w](rd4_lookahead[w]);
    }
    sc_signal<bool> rd0_lookahead_valid, rd1_lookahead_valid, rd2_lookahead_valid,
        rd3_lookahead_valid;
    dut.impl.rd0_lookahead_valid_i(rd0_lookahead_valid);
    dut.impl.rd1_lookahead_valid_i(rd1_lookahead_valid);
    dut.impl.rd2_lookahead_valid_i(rd2_lookahead_valid);
    dut.impl.rd3_lookahead_valid_i(rd3_lookahead_valid);

    auto tdm_mode = [](int ports_used, int num_req) -> uint32_t {
        const int g = (num_req > 0 && ports_used > 0) ? ports_used / num_req : 1;
        return (g <= 1) ? 0u : (g <= 2) ? 1u : 2u;
    };

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    constexpr int kBudget = 30000; // any non-deadlocked run finishes in << this
    int           actual  = 0;
    while (actual < kBudget) {
        bool all = true;
        for (int a = 0; a < 9; ++a)
            all = all && done[a].read();
        if (all)
            break;

        // Same per-cycle sync tb_top.cpp's TDM block performs, ragu_a only
        // (the other groups are idle placeholders).
        if (dut.impl.buf_r0.snapshot().window_reset)
            ragu_a_src.advance_lookahead_window();
        // Task-transition idle gates (see agu.hpp): geometry-changing
        // boundaries are held until the buffer is observed idle.
        const bool idle_a                  = dut.impl.buf_r0.snapshot().n_valid == 0;
        ragu_a_src.la_task_roll_gate_open_ = idle_a;
        ragu_a_src.flush_hold_ =
            !ragu_a_src.all_tasks_done() &&
            ragu_a_src.group_ >= ragu_a_src.tasks_[ragu_a_src.task_idx_].n_groups && !idle_a;
        ragu_a_src.retry_lookahead_fence();

        dut.impl_buf_active_mode[0].write(
            tdm_mode(ragu_a_src.lookahead_ports_used(), dut_t::NUM_REQ));
        dut.impl_buf_map_r[0].write(ragu_a_src.lookahead_R());
        dut.impl_buf_map_c[0].write(ragu_a_src.lookahead_C());
        dut.impl_buf_map_l[0].write(ragu_a_src.lookahead_L());
        dut.impl_buf_map_store_mode[0].write(ragu_a_src.lookahead_store_mode());
        for (int i = 1; i < 9; ++i) {
            dut.impl_buf_active_mode[i].write(0);
            dut.impl_buf_map_r[i].write(4);
            dut.impl_buf_map_c[i].write(4);
            dut.impl_buf_map_l[i].write(8);
            dut.impl_buf_map_store_mode[i].write(0);
        }
        rd0_lookahead_valid.write(ragu_a_src.lookahead_ready());
        rd1_lookahead_valid.write(false);
        rd2_lookahead_valid.write(false);
        rd3_lookahead_valid.write(false);
        for (int w = 0; w < N_BANK; ++w) {
            rd0_lookahead[w].write(ragu_a_src.lookahead_addr(w));
            rd1_lookahead[w].write(0);
            rd2_lookahead[w].write(0);
            rd3_lookahead[w].write(0);
            rd4_lookahead[w].write(0);
        }

        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }

    std::printf("\n=== task-boundary sweep: %d cycles (budget %d) ===\n", actual, kBudget);
    CHECK(actual < kBudget, "whole scenario trace completes without deadlock");

    // Per-task coverage: every address of every task appears in the response
    // log exactly once — a boundary that wedges (or double-drains) shows up
    // as its own labeled failure, pointing straight at the offending pattern.
    std::map<uint64_t, int> seen;
    for (const auto &a : ragu_a_src.log_)
        if (!a.we)
            ++seen[a.addr];
    for (std::size_t k = 0; k < specs.size(); ++k) {
        bool ok = true;
        for (uint64_t a : task_addrs[k])
            if (seen[a] != 1) {
                ok = false;
                break;
            }
        char lbl[160];
        std::snprintf(lbl, sizeof(lbl), "task %zu (%s): all %d responses exactly once", k,
                      specs[k].label, specs[k].lines);
        CHECK(ok, lbl);
    }
    std::size_t total = 0;
    for (const auto &kv : seen)
        total += static_cast<std::size_t>(kv.second);
    std::size_t expect_total = 0;
    for (const auto &s : task_addrs)
        expect_total += s.size();
    CHECK(total == expect_total, "no spurious or duplicated responses overall");

    std::filesystem::remove_all(dir);

    std::printf("\n=== Summary ===\n  passed: %d\n  failed: %d\n\n%s\n", g_pass, g_fail,
                g_fail == 0 ? "All tests passed." : "SOME TESTS FAILED.");
    return g_fail == 0 ? 0 : 1;
}
