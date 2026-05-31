// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   sc_main harness for the crossbar design (make sim-sc). It instantiates the
//   DUT (top_crossbar = crossbar + N_BANK banks) and N_AGU AGUs, wires each
//   AGU's N_REQ OBI ports to the DUT manager ports, drives clock/reset, and
//   runs until every AGU has drained its trace. Each AGU dumps its completed
//   accesses to out_<i>.log; the harness then prints timing statistics.
//
//   Statistics:
//     - actual cycles : measured cycles (reset release -> all AGUs done).
//     - ideal cycles  : conflict-free analytical estimate. The AGU is pipelined
//                       and group-synchronized: with no conflict it issues one
//                       group of N_REQ requests every cycle (throughput 1
//                       group/cycle), plus a fixed PIPE_FILL = 2 cycles to drain
//                       the last group (issue -> grant -> response; measured,
//                       independent of N_REQ/N_BANK). A single AGU finishes in
//                       G + PIPE_FILL cycles; with no conflicts the AGUs overlap,
//                       so ideal = G_max + PIPE_FILL, G_a = ceil(len_a / N_REQ).
//     - delay penalty : 100 * (actual - ideal) / ideal  [%].
//
//   Paths are resolved from the environment so the binary works under the flow
//   (CWD = scripts/sim-sc) and standalone: input traces from
//   $CODE_HOME/rtl-lab/projects/$SEL_PROJECT/tb/traces/mem_<i>.log, output logs
//   into the run's output dir ($.../sim/$SEL_OUT_DIR/output/out_<i>.log).
//
//   Configuration via -D (the flow's PARAMS mechanism), defaults below. The
//   config knobs are ALL-CAPS macros (N_AGU, …); the design headers name their
//   template parameters in ALL-CAPS too but deliberately distinct from those
//   macros (counts NUM_*, e.g. NUM_AGU; other dims spelled out, e.g.
//   BYTES_PER_WORD), so a -D override can never rewrite a template declaration.
//   The macros pass straight through as positional template arguments below.
// -----------------------------------------------------------------------------

#include <systemc.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "top_crossbar.hpp"
#include "agu.hpp"

#ifndef N_AGU
#define N_AGU 2
#endif
#ifndef N_REQ
#define N_REQ 4
#endif
#ifndef N_BANK
#define N_BANK 8
#endif
#ifndef N_ROW
#define N_ROW 1024
#endif
#ifndef WORD_BYTES
#define WORD_BYTES 4
#endif
#ifndef CLK_PERIOD_NS
#define CLK_PERIOD_NS 10
#endif

static const int kCyclesPerGroup = 1;
static const int kPipeFill       = 2;

static std::string env_or(const char* key, const std::string& dflt) {
    const char* v = std::getenv(key);
    return v ? std::string(v) : dflt;
}

int sc_main(int, char*[]) {
    static const int kNumMgr = N_AGU * N_REQ;

    const std::string project = env_or("SEL_PROJECT", "tdm");
    const char* ch = std::getenv("CODE_HOME");
    const std::string proj_dir =
        ch ? (std::string(ch) + "/rtl-lab/projects/" + project)
           : ("projects/" + project);
    const std::string trace_dir = proj_dir + "/tb/traces";
    const char* od = std::getenv("SEL_OUT_DIR");
    const std::string out_dir =
        od ? (proj_dir + "/sim/" + od + "/output") : ".";

    sc_clock clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool> rst_ni;

    sc_signal<bool>     m_req[kNumMgr], m_we[kNumMgr], m_gnt[kNumMgr], m_rvalid[kNumMgr];
    sc_signal<uint64_t> m_addr[kNumMgr], m_wdata[kNumMgr], m_rdata[kNumMgr];
    sc_signal<uint32_t> m_be[kNumMgr];
    sc_signal<bool>     done[N_AGU];

    top_crossbar<N_AGU, N_REQ, N_BANK, N_ROW, WORD_BYTES> dut("dut");
    dut.clk_i(clk);
    dut.rst_ni(rst_ni);
    for (int m = 0; m < kNumMgr; ++m) {
        dut.m_req_i[m](m_req[m]);   dut.m_addr_i[m](m_addr[m]);
        dut.m_we_i[m](m_we[m]);     dut.m_be_i[m](m_be[m]);
        dut.m_wdata_i[m](m_wdata[m]);
        dut.m_gnt_o[m](m_gnt[m]);   dut.m_rvalid_o[m](m_rvalid[m]);
        dut.m_rdata_o[m](m_rdata[m]);
    }

    agu<N_REQ, WORD_BYTES>* agus[N_AGU];
    for (int a = 0; a < N_AGU; ++a) {
        const std::string nm    = "agu" + std::to_string(a);
        const std::string trace = trace_dir + "/mem_" + std::to_string(a) + ".log";
        const std::string out   = out_dir + "/out_" + std::to_string(a) + ".log";
        agus[a] = new agu<N_REQ, WORD_BYTES>(nm.c_str(), trace, out);
        agus[a]->clk_i(clk);
        agus[a]->rst_ni(rst_ni);
        agus[a]->done_o(done[a]);
        for (int p = 0; p < N_REQ; ++p) {
            const int m = a * N_REQ + p;
            agus[a]->req_o[p](m_req[m]);     agus[a]->addr_o[p](m_addr[m]);
            agus[a]->we_o[p](m_we[m]);       agus[a]->be_o[p](m_be[m]);
            agus[a]->wdata_o[p](m_wdata[m]);
            agus[a]->gnt_i[p](m_gnt[m]);     agus[a]->rvalid_i[p](m_rvalid[m]);
            agus[a]->rdata_i[p](m_rdata[m]);
        }
    }

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    const int kMaxCycles = 1000000;
    int actual = 0;
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < N_AGU; ++a) all = all && done[a].read();
        if (all) break;
        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }

    int g_max = 0;
    std::size_t total_acc = 0, total_rd = 0;
    for (int a = 0; a < N_AGU; ++a) {
        const std::size_t len = agus[a]->trace_.size();
        const int g = static_cast<int>((len + N_REQ - 1) / N_REQ);
        g_max = std::max(g_max, g);
        total_acc += agus[a]->log_.size();
        for (const auto& e : agus[a]->log_) if (!e.we) ++total_rd;
    }
    const int ideal = kCyclesPerGroup * g_max + kPipeFill;
    const double penalty = ideal > 0 ? 100.0 * (actual - ideal) / ideal : 0.0;

    printf("\n");
    printf("=========== crossbar statistics ===========\n");
    printf(" config       : N_AGU=%d N_REQ=%d N_BANK=%d N_ROW=%d WORD_BYTES=%d\n",
           N_AGU, N_REQ, N_BANK, N_ROW, WORD_BYTES);
    printf(" accesses     : %zu (%zu reads)\n", total_acc, total_rd);
    printf(" groups (max) : %d\n", g_max);
    printf(" actual cycles: %d\n", actual);
    printf(" ideal cycles : %d   (conflict-free: %d*%d + %d)\n",
           ideal, kCyclesPerGroup, g_max, kPipeFill);
    printf(" delay penalty: %.2f %%\n", penalty);
    printf("===========================================\n");

    sc_stop();
    for (int a = 0; a < N_AGU; ++a) delete agus[a];
    return 0;
}
