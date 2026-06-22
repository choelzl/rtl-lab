// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   sc_main harness for the crossbar design (make sim-sc). It instantiates the
//   DUT (top_crossbar = crossbar + N_BANK banks), N_RAGU read AGUs and N_WAGU
//   write AGUs, wires each AGU's N_REQ OBI ports to the DUT manager ports,
//   drives clock/reset, and runs until every AGU has drained its trace. Read
//   AGUs dump completed accesses to out_<i>.log, write AGUs to wout_<i>.log;
//   the harness then prints timing statistics.
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
//   (CWD = scripts/sim-sc) and standalone: read-AGU stimuli from
//   .../tb/stimuli/mem_<i>.log, write-AGU stimuli from wmem_<i>.log; output
//   logs into $.../sim/$SEL_OUT_DIR/output/out_<i>.log and wout_<i>.log.
//
//   Configuration via -D (the flow's PARAMS mechanism), defaults below. The
//   config knobs are ALL-CAPS macros (N_RAGU, …); the design headers name their
//   template parameters in ALL-CAPS too but deliberately distinct from those
//   macros (counts NUM_*, e.g. NUM_RAGU; other dims spelled out, e.g.
//   BYTES_PER_WORD), so a -D override can never rewrite a template declaration.
//   The macros pass straight through as positional template arguments below.
// -----------------------------------------------------------------------------

#include <systemc.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#ifdef SV
#include "top_crossbar_sv.hpp"
#else
#include "top_crossbar.hpp"
#endif
#include "agu_crossbar.hpp"
#include "constants.hpp"

// ---------------------------------------------------------------------------
// Lx crossbar Number
// ---------------------------------------------------------------------------
#ifndef NUM_L1
#define NUM_L1 N_RAGU
#endif
#ifndef NUM_L2
#define NUM_L2 4
#endif
// ---------------------------------------------------------------------------
// L1 crossbar  (NUM_L1 switches, each L1_NIN-in × L1_NOUT-out)
// ---------------------------------------------------------------------------
#ifndef L1_NIN
#define L1_NIN N_REQ
#endif
#ifndef L1_NOUT
#define L1_NOUT NUM_L2
#endif

// ---------------------------------------------------------------------------
// L2 crossbar  (NUM_L2 switches, each L2_NIN-in × L2_NOUT-out)
// ---------------------------------------------------------------------------

#ifndef L2_NIN
#define L2_NIN NUM_L1
#endif
#ifndef L2_NOUT
#define L2_NOUT N_BANK / NUM_L2
#endif

// ---------------------------------------------------------------------------
// Write crossbar (separate 2-layer path for N_WAGU write AGUs; not yet
// connected to banks — ports exist for future wiring only)
// ---------------------------------------------------------------------------
// WL1: NUM_WL1 switches, each WL1_NIN-in × WL1_NOUT-out
#ifndef NUM_WL1
#define NUM_WL1 N_WAGU
#endif
#ifndef NUM_WL2
#define NUM_WL2 4
#endif
#ifndef WL1_NIN
#define WL1_NIN N_REQ
#endif
#ifndef WL1_NOUT
#define WL1_NOUT NUM_WL2
#endif
// WL2: NUM_WL2 switches, each WL2_NIN-in × WL2_NOUT-out
#ifndef WL2_NIN
#define WL2_NIN NUM_WL1
#endif
#ifndef WL2_NOUT
#define WL2_NOUT N_BANK / NUM_WL2
#endif

static constexpr int kCyclesPerGroup = 1;
static constexpr int kPipeFill       = 2;

static std::string env_or(const char *key, const std::string &dflt) {
    const char *v = std::getenv(key);
    return v ? std::string(v) : dflt;
}

int sc_main(int, char *[]) {
    constexpr int kNumMgr  = N_RAGU * N_REQ;
    constexpr int kNumWMgr = N_WAGU * N_REQ;

    const std::string project = env_or("SEL_PROJECT", "tdm");
    const char       *ch      = std::getenv("RTL_LAB_HOME");
    const std::string proj_dir =
        ch ? (std::string(ch) + "/projects/" + project) : ("projects/" + project);
    const std::string in_dir   = env_or("SEL_IN_DIR", "");
    const std::string stim_dir = in_dir.empty() ? proj_dir + "/tb/stimuli/sample"
                                 : in_dir.find('/') != std::string::npos
                                     ? in_dir
                                     : proj_dir + "/tb/stimuli/" + in_dir;
    const char       *od       = std::getenv("SEL_OUT_DIR");
    const std::string out_dir  = od ? (proj_dir + "/sim/" + od + "/output") : ".";

    sc_clock        clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool> rst_ni;

    sc_signal<bool>     m_req[kNumMgr], m_we[kNumMgr], m_gnt[kNumMgr], m_rvalid[kNumMgr];
    sc_signal<uint64_t> m_addr[kNumMgr], m_wdata[kNumMgr], m_rdata[kNumMgr];
    sc_signal<uint32_t> m_be[kNumMgr];

    sc_signal<bool> m_w_req[kNumWMgr], m_w_we[kNumWMgr], m_w_gnt[kNumWMgr], m_w_rvalid[kNumWMgr];
    sc_signal<uint64_t> m_w_addr[kNumWMgr], m_w_wdata[kNumWMgr], m_w_rdata[kNumWMgr];
    sc_signal<uint32_t> m_w_be[kNumWMgr];

    sc_signal<bool> done[N_RAGU + N_WAGU];

#ifdef SV
    top_crossbar_sv<N_RAGU, N_WAGU, N_REQ, N_BANK, N_ROW> dut("dut");
#else
    top_crossbar<N_RAGU, N_WAGU, N_REQ, N_BANK, N_ROW, WORD_BYTES> dut("dut");
#endif
    dut.clk_i(clk);
    dut.rst_ni(rst_ni);
    for (int m = 0; m < kNumMgr; ++m) {
        dut.m_req_i[m](m_req[m]);
        dut.m_addr_i[m](m_addr[m]);
        dut.m_we_i[m](m_we[m]);
        dut.m_be_i[m](m_be[m]);
        dut.m_wdata_i[m](m_wdata[m]);
        dut.m_gnt_o[m](m_gnt[m]);
        dut.m_rvalid_o[m](m_rvalid[m]);
        dut.m_rdata_o[m](m_rdata[m]);
    }
    for (int m = 0; m < kNumWMgr; ++m) {
        dut.m_w_req_i[m](m_w_req[m]);
        dut.m_w_addr_i[m](m_w_addr[m]);
        dut.m_w_we_i[m](m_w_we[m]);
        dut.m_w_be_i[m](m_w_be[m]);
        dut.m_w_wdata_i[m](m_w_wdata[m]);
        dut.m_w_gnt_o[m](m_w_gnt[m]);
        dut.m_w_rvalid_o[m](m_w_rvalid[m]);
        dut.m_w_rdata_o[m](m_w_rdata[m]);
    }

    std::unique_ptr<agu_crossbar<N_REQ, WORD_BYTES>> agus[N_RAGU];
    for (int a = 0; a < N_RAGU; ++a) {
        const std::string nm   = "agu" + std::to_string(a);
        const std::string stim = stim_dir + "/mem_" + std::to_string(a) + ".log";
        const std::string out  = out_dir + "/out_" + std::to_string(a) + ".log";
        agus[a] = std::make_unique<agu_crossbar<N_REQ, WORD_BYTES>>(nm.c_str(), stim, out);
        agus[a]->clk_i(clk);
        agus[a]->rst_ni(rst_ni);
        agus[a]->done_o(done[a]);
        for (int p = 0; p < N_REQ; ++p) {
            const int m = a * N_REQ + p;
            agus[a]->req_o[p](m_req[m]);
            agus[a]->addr_o[p](m_addr[m]);
            agus[a]->we_o[p](m_we[m]);
            agus[a]->be_o[p](m_be[m]);
            agus[a]->wdata_o[p](m_wdata[m]);
            agus[a]->gnt_i[p](m_gnt[m]);
            agus[a]->rvalid_i[p](m_rvalid[m]);
            agus[a]->rdata_i[p](m_rdata[m]);
        }
    }

    std::unique_ptr<agu_crossbar<N_REQ, WORD_BYTES>> wagus[N_WAGU];
    for (int a = 0; a < N_WAGU; ++a) {
        const std::string nm   = "wagu" + std::to_string(a);
        const std::string stim = stim_dir + "/wmem_" + std::to_string(a) + ".log";
        const std::string out  = out_dir + "/wout_" + std::to_string(a) + ".log";
        wagus[a] = std::make_unique<agu_crossbar<N_REQ, WORD_BYTES>>(nm.c_str(), stim, out);
        wagus[a]->clk_i(clk);
        wagus[a]->rst_ni(rst_ni);
        wagus[a]->done_o(done[N_RAGU + a]);
        for (int p = 0; p < N_REQ; ++p) {
            const int m = a * N_REQ + p;
            wagus[a]->req_o[p](m_w_req[m]);
            wagus[a]->addr_o[p](m_w_addr[m]);
            wagus[a]->we_o[p](m_w_we[m]);
            wagus[a]->be_o[p](m_w_be[m]);
            wagus[a]->wdata_o[p](m_w_wdata[m]);
            wagus[a]->gnt_i[p](m_w_gnt[m]);
            wagus[a]->rvalid_i[p](m_w_rvalid[m]);
            wagus[a]->rdata_i[p](m_w_rdata[m]);
        }
    }

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    constexpr int kMaxCycles = 1000000;
    int           actual     = 0;
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < N_RAGU + N_WAGU; ++a)
            all = all && done[a].read();
        if (all)
            break;
        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }
    if (actual >= kMaxCycles)
        fprintf(stderr, "WARNING: simulation timed out after %d cycles — not all AGUs finished\n",
                kMaxCycles);

    int         g_max     = 0;
    std::size_t total_acc = 0, total_rd = 0;
    for (int a = 0; a < N_RAGU; ++a) {
        const std::size_t len = agus[a]->trace_.size();
        const int         g   = static_cast<int>((len + N_REQ - 1) / N_REQ);
        g_max                 = std::max(g_max, g);
        total_acc += agus[a]->log_.size();
        for (const auto &e : agus[a]->log_)
            if (!e.we)
                ++total_rd;
    }
    for (int a = 0; a < N_WAGU; ++a) {
        const std::size_t len = wagus[a]->trace_.size();
        const int         g   = static_cast<int>((len + N_REQ - 1) / N_REQ);
        g_max                 = std::max(g_max, g);
        total_acc += wagus[a]->log_.size();
        for (const auto &e : wagus[a]->log_)
            if (!e.we)
                ++total_rd;
    }
    const int    ideal   = kCyclesPerGroup * g_max + kPipeFill;
    const double penalty = ideal > 0 ? 100.0 * (actual - ideal) / ideal : 0.0;

    printf("\n");
    printf("=========== crossbar statistics ===========\n");
    printf(" config       : N_RAGU=%d N_WAGU=%d N_REQ=%d N_BANK=%d N_ROW=%d WORD_BYTES=%d "
           "WORDS_PER_ROW=%d\n",
           N_RAGU, N_WAGU, N_REQ, N_BANK, N_ROW, WORD_BYTES, WORDS_PER_ROW);
    printf(" accesses     : %zu (%zu reads)\n", total_acc, total_rd);
    printf(" groups (max) : %d\n", g_max);
    printf(" actual cycles: %d\n", actual);
    printf(" ideal cycles : %d   (conflict-free: %d*%d + %d)\n", ideal, kCyclesPerGroup, g_max,
           kPipeFill);
    printf(" delay penalty: %.2f %%\n", penalty);
    printf("===========================================\n");

    sc_stop();
    return 0;
}
