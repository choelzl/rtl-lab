// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   sc_main harness for the TDM design (make sim-sc). It instantiates the DUT
//   (top_tdm) and N_AGU TDM AGUs, wires each AGU's x-OBI group ports + stride to
//   the DUT's AGU-facing ports, drives clock/reset, and runs until every AGU has
//   drained its trace. Each AGU dumps its completed accesses to out_<i>.log; the
//   harness then prints timing statistics.
//
//   Statistics mirror the crossbar harness so the two are comparable:
//     - actual cycles : measured cycles (reset release -> all AGUs done).
//     - ideal cycles  : conflict-free analytical estimate, G_max + PIPE_FILL with
//                       G_a = ceil(len_a / N_REQ). PIPE_FILL for the TDM pipeline
//                       is provisional and must be CALIBRATED from a conflict-free
//                       run (the v1 consecutive mapping has no conflicts).
//     - delay penalty : 100 * (actual - ideal) / ideal  [%].
//
//   Paths are resolved from the environment like the crossbar harness.
// -----------------------------------------------------------------------------

#include <systemc.h>

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "agu_tdm.hpp"
#include "top_tdm.hpp"

#ifndef N_AGU
#define N_AGU 2
#endif
#ifndef N_REQ
#define N_REQ 4
#endif
#ifndef N_BANK
#define N_BANK 32
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
static const int kPipeFill       = 6;

static std::string env_or(const char *key, const std::string &dflt) {
    const char *v = std::getenv(key);
    return v ? std::string(v) : dflt;
}

int sc_main(int, char *[]) {
    static const int kNumMgr = N_AGU * N_REQ;

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

    sc_signal<bool>     m_req[kNumMgr], m_gnt[kNumMgr], m_rvalid[kNumMgr];
    sc_signal<uint64_t> m_wdata[kNumMgr], m_rdata[kNumMgr];
    sc_signal<uint64_t> m_addr[N_AGU];
    sc_signal<bool>     m_we[N_AGU];
    sc_signal<uint32_t> m_be[N_AGU];
    sc_signal<uint64_t> m_num_banks[N_AGU], m_bank_width[N_AGU];
    sc_signal<uint64_t> m_r[N_AGU], m_c[N_AGU], m_l[N_AGU], m_store_mode[N_AGU];
    sc_signal<bool>     done[N_AGU];

    top_tdm<N_AGU, N_REQ, N_BANK, N_ROW, WORD_BYTES> dut("dut");
    dut.clk_i(clk);
    dut.rst_ni(rst_ni);
    for (int m = 0; m < kNumMgr; ++m) {
        dut.a_req_i[m](m_req[m]);
        dut.a_wdata_i[m](m_wdata[m]);
        dut.a_gnt_o[m](m_gnt[m]);
        dut.a_rvalid_o[m](m_rvalid[m]);
        dut.a_rdata_o[m](m_rdata[m]);
    }
    for (int a = 0; a < N_AGU; ++a) {
        dut.a_addr_i[a](m_addr[a]);
        dut.a_we_i[a](m_we[a]);
        dut.a_be_i[a](m_be[a]);
        dut.a_num_banks_i[a](m_num_banks[a]);
        dut.a_bank_width_i[a](m_bank_width[a]);
        dut.a_r_i[a](m_r[a]);
        dut.a_c_i[a](m_c[a]);
        dut.a_l_i[a](m_l[a]);
        dut.a_store_mode_i[a](m_store_mode[a]);
    }

    agu_tdm<N_REQ, WORD_BYTES> *agus[N_AGU];
    for (int a = 0; a < N_AGU; ++a) {
        const std::string nm   = "agu" + std::to_string(a);
        const std::string stim = stim_dir + "/mem_" + std::to_string(a) + ".log";
        const std::string out  = out_dir + "/out_" + std::to_string(a) + ".log";
        agus[a]                = new agu_tdm<N_REQ, WORD_BYTES>(nm.c_str(), stim, out);
        agus[a]->clk_i(clk);
        agus[a]->rst_ni(rst_ni);
        agus[a]->done_o(done[a]);
        agus[a]->addr_o(m_addr[a]);
        agus[a]->we_o(m_we[a]);
        agus[a]->be_o(m_be[a]);
        agus[a]->num_banks_o(m_num_banks[a]);
        agus[a]->bank_width_o(m_bank_width[a]);
        agus[a]->r_o(m_r[a]);
        agus[a]->c_o(m_c[a]);
        agus[a]->l_o(m_l[a]);
        agus[a]->store_mode_o(m_store_mode[a]);
        for (int p = 0; p < N_REQ; ++p) {
            const int m = a * N_REQ + p;
            agus[a]->req_o[p](m_req[m]);
            agus[a]->wdata_o[p](m_wdata[m]);
            agus[a]->gnt_i[p](m_gnt[m]);
            agus[a]->rvalid_i[p](m_rvalid[m]);
            agus[a]->rdata_i[p](m_rdata[m]);
        }
    }

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    const int kMaxCycles = 1000000;
    int       actual     = 0;
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < N_AGU; ++a)
            all = all && done[a].read();
        if (all)
            break;
        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }

    int         g_max     = 0;
    std::size_t total_acc = 0, total_rd = 0;
    for (int a = 0; a < N_AGU; ++a) {
        const std::size_t len = agus[a]->trace_.size();
        const int         g   = static_cast<int>((len + N_REQ - 1) / N_REQ);
        g_max                 = std::max(g_max, g);
        total_acc += agus[a]->log_.size();
        for (const auto &e : agus[a]->log_)
            if (!e.we)
                ++total_rd;
    }
    const int    ideal   = kCyclesPerGroup * g_max + kPipeFill;
    const double penalty = ideal > 0 ? 100.0 * (actual - ideal) / ideal : 0.0;

    printf("\n");
    printf("============== tdm statistics ==============\n");
    printf(" config       : N_AGU=%d N_REQ=%d N_BANK=%d N_ROW=%d WORD_BYTES=%d\n", N_AGU, N_REQ,
           N_BANK, N_ROW, WORD_BYTES);
    printf(" accesses     : %zu (%zu reads)\n", total_acc, total_rd);
    printf(" groups (max) : %d\n", g_max);
    printf(" actual cycles: %d\n", actual);
    printf(" ideal cycles : %d   (conflict-free: %d*%d + %d)\n", ideal, kCyclesPerGroup, g_max,
           kPipeFill);
    printf(" delay penalty: %.2f %%\n", penalty);
    printf("============================================\n");

    sc_stop();
    for (int a = 0; a < N_AGU; ++a)
        delete agus[a];
    return 0;
}
