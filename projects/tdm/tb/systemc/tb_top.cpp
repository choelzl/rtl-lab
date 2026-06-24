// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// Description:
//   Unified SystemC harness for rtl/systemc/top.hpp.  It connects RAGU/WAGU trace drivers to
//   the fixed wrapper map:
//     read : RAGU_A trace drives RPORT_A[0..3], RAGU_B drives RPORT_B[0..1],
//            RAGU_C/RAGU_D/RAGU_DMA each drive one RPORT.
//     write: WAGU_A trace drives WPORT_A[0..3], WAGU_B drives WPORT_B[0..1],
//            WAGU_D/WAGU_DMA each drive one WPORT
//
//   Pass IMPL=tdm to select the native SC TDM implementation and match RAGU
//   trace targets / default stimuli directory. IMPL=tdm_sc remains a compatibility
//   alias.
// -----------------------------------------------------------------------------

// top.hpp must precede constants.hpp to avoid the BYTES_PER_ROW macro colliding
// with template parameter names in bank.hpp / buffer.hpp.
#include "top.hpp"

#include <systemc.h>

#include "obi_data.hpp"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <memory>
#include <string>

#include "agu.hpp"
#include "constants.hpp"

static constexpr int kPipeFill = 2;
using dut_t                    = top<N_BANK, N_ROW, WORD_BYTES, WORDS_PER_ROW>;
using data_t                   = obi_data<BYTES_PER_ROW>;

static std::string env_or(const char *key, const std::string &dflt) {
    const char *v = std::getenv(key);
    return v ? std::string(v) : dflt;
}

template <int N> struct obi_group_signals {
    sc_signal<bool>     req[N], we[N], gnt[N], rvalid[N];
    sc_signal<uint64_t> addr[N];
    sc_signal<uint32_t> be[N];
    sc_signal<data_t>   wdata[N], rdata[N];
};

template <int N, int NPG>
static void bind_agu(agu<N, data_t, BYTES_PER_ROW, NPG> &src, sc_clock &clk,
                     sc_signal<bool> &rst_ni, sc_signal<bool> &done, obi_group_signals<N> &bus) {
    src.clk_i(clk);
    src.rst_ni(rst_ni);
    src.done_o(done);
    for (int p = 0; p < N; ++p) {
        src.req_o[p](bus.req[p]);
        src.addr_o[p](bus.addr[p]);
        src.we_o[p](bus.we[p]);
        src.be_o[p](bus.be[p]);
        src.wdata_o[p](bus.wdata[p]);
        src.gnt_i[p](bus.gnt[p]);
        src.rvalid_i[p](bus.rvalid[p]);
        src.rdata_i[p](bus.rdata[p]);
    }
}

#define BIND_DUT_GROUP(dut, prefix, bus, N)                                                        \
    do {                                                                                           \
        for (int p = 0; p < (N); ++p) {                                                            \
            (dut).prefix##_req_i[p]((bus).req[p]);                                                 \
            (dut).prefix##_addr_i[p]((bus).addr[p]);                                               \
            (dut).prefix##_we_i[p]((bus).we[p]);                                                   \
            (dut).prefix##_be_i[p]((bus).be[p]);                                                   \
            (dut).prefix##_wdata_i[p]((bus).wdata[p]);                                             \
            (dut).prefix##_gnt_o[p]((bus).gnt[p]);                                                 \
            (dut).prefix##_rvalid_o[p]((bus).rvalid[p]);                                           \
            (dut).prefix##_rdata_o[p]((bus).rdata[p]);                                             \
        }                                                                                          \
    } while (0)

int sc_main(int, char *[]) {
    const std::string project = env_or("SEL_PROJECT", "tdm");
    const char       *ch      = std::getenv("RTL_LAB_HOME");
    const std::string proj_dir =
        ch ? (std::string(ch) + "/projects/" + project) : ("projects/" + project);
    const std::string in_dir   = env_or("SEL_IN_DIR", "");
    const std::string stim_dir = in_dir.empty() ? proj_dir + "/tb/stimuli/"
                                                             "sample"
                                 : in_dir.find('/') != std::string::npos
                                     ? in_dir
                                     : proj_dir + "/tb/stimuli/" + in_dir;
    const char       *od       = std::getenv("SEL_OUT_DIR");
    const std::string out_dir  = od ? (proj_dir + "/sim/" + od + "/output") : ".";

    sc_clock        clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool> rst_ni;

    dut_t dut("dut");
    dut.clk_i(clk);
    dut.rst_ni(rst_ni);

    obi_group_signals<dut_t::RAGU_A_PORTS>   ragu_a;
    obi_group_signals<dut_t::RAGU_B_PORTS>   ragu_b;
    obi_group_signals<dut_t::RAGU_C_PORTS>   ragu_c;
    obi_group_signals<dut_t::RAGU_D_PORTS>   ragu_d;
    obi_group_signals<dut_t::RAGU_DMA_PORTS> ragu_dma;
    obi_group_signals<dut_t::WAGU_A_PORTS>   wagu_a;
    obi_group_signals<dut_t::WAGU_B_PORTS>   wagu_b;
    obi_group_signals<dut_t::WAGU_D_PORTS>   wagu_d;
    obi_group_signals<dut_t::WAGU_DMA_PORTS> wagu_dma;

    BIND_DUT_GROUP(dut, ragu_a, ragu_a, dut_t::RAGU_A_PORTS);
    BIND_DUT_GROUP(dut, ragu_b, ragu_b, dut_t::RAGU_B_PORTS);
    BIND_DUT_GROUP(dut, ragu_c, ragu_c, dut_t::RAGU_C_PORTS);
    BIND_DUT_GROUP(dut, ragu_d, ragu_d, dut_t::RAGU_D_PORTS);
    BIND_DUT_GROUP(dut, ragu_dma, ragu_dma, dut_t::RAGU_DMA_PORTS);
    BIND_DUT_GROUP(dut, wagu_a, wagu_a, dut_t::WAGU_A_PORTS);
    BIND_DUT_GROUP(dut, wagu_b, wagu_b, dut_t::WAGU_B_PORTS);
    BIND_DUT_GROUP(dut, wagu_d, wagu_d, dut_t::WAGU_D_PORTS);
    BIND_DUT_GROUP(dut, wagu_dma, wagu_dma, dut_t::WAGU_DMA_PORTS);

    sc_signal<bool> done[9];

#if defined(IMPL_TDM) || defined(IMPL_TDM_SC)
    constexpr agu_target ragu_tgt = agu_target::tdm;
#else
    constexpr agu_target ragu_tgt = agu_target::crossbar;
#endif

    auto ragu_a_src =
        std::make_unique<agu<dut_t::RPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_a", stim_dir + "/ragu_a.log", out_dir + "/ragu_a.csv", ragu_tgt);
    auto ragu_b_src =
        std::make_unique<agu<dut_t::RPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_b", stim_dir + "/ragu_b.log", out_dir + "/ragu_b.csv", ragu_tgt);
    auto ragu_c_src =
        std::make_unique<agu<dut_t::RPORT_C_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_c", stim_dir + "/ragu_c.log", out_dir + "/ragu_c.csv", ragu_tgt);
    auto ragu_d_src =
        std::make_unique<agu<dut_t::RPORT_D_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_d", stim_dir + "/ragu_d.log", out_dir + "/ragu_d.csv", ragu_tgt);
    auto ragu_dma_src =
        std::make_unique<agu<dut_t::RPORT_DMA_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_dma", stim_dir + "/ragu_dma.log", out_dir + "/ragu_dma.csv", ragu_tgt);
    auto wagu_a_src =
        std::make_unique<agu<dut_t::WPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_a", stim_dir + "/wagu_a.log", out_dir + "/wagu_a.csv", agu_target::crossbar);
    auto wagu_b_src =
        std::make_unique<agu<dut_t::WPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_b", stim_dir + "/wagu_b.log", out_dir + "/wagu_b.csv", agu_target::crossbar);
    auto wagu_d_src =
        std::make_unique<agu<dut_t::WPORT_D_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_d", stim_dir + "/wagu_d.log", out_dir + "/wagu_d.csv", agu_target::crossbar);
    auto wagu_dma_src =
        std::make_unique<agu<dut_t::WPORT_DMA_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_dma", stim_dir + "/wagu_dma.log", out_dir + "/wagu_dma.csv",
            agu_target::crossbar);

    bind_agu(*ragu_a_src, clk, rst_ni, done[0], ragu_a);
    bind_agu(*ragu_b_src, clk, rst_ni, done[1], ragu_b);
    bind_agu(*ragu_c_src, clk, rst_ni, done[2], ragu_c);
    bind_agu(*ragu_d_src, clk, rst_ni, done[3], ragu_d);
    bind_agu(*ragu_dma_src, clk, rst_ni, done[4], ragu_dma);
    bind_agu(*wagu_a_src, clk, rst_ni, done[5], wagu_a);
    bind_agu(*wagu_b_src, clk, rst_ni, done[6], wagu_b);
    bind_agu(*wagu_d_src, clk, rst_ni, done[7], wagu_d);
    bind_agu(*wagu_dma_src, clk, rst_ni, done[8], wagu_dma);

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    constexpr int kMaxCycles = 1000000;
    int           actual     = 0;
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < 9; ++a)
            all = all && done[a].read();
        if (all)
            break;
        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }

    if (actual >= kMaxCycles)
        fprintf(stderr, "WARNING: simulation timed out after %d cycles\n", kMaxCycles);

    // ideal = pipeline fill + the slowest AGU (structural-conflict-free lower bound)
    const std::size_t max_groups = std::max({
        ragu_a_src->n_groups_,
        ragu_b_src->n_groups_,
        ragu_c_src->n_groups_,
        ragu_d_src->n_groups_,
        ragu_dma_src->n_groups_,
        wagu_a_src->n_groups_,
        wagu_b_src->n_groups_,
        wagu_d_src->n_groups_,
        wagu_dma_src->n_groups_,
    });
    const int         ideal      = kPipeFill + static_cast<int>(max_groups);
    const double      overhead   = ideal > 0 ? 100.0 * (actual - ideal) / ideal : 0.0;

    printf("\n=========== top statistics (%s) ===========\n",
#if defined(IMPL_TDM) || defined(IMPL_TDM_SC)
           "tdm"
#elif defined(IMPL_TOP_TDM)
           "top_tdm"
#elif defined(IMPL_TOP_CROSSBAR)
           "top_crossbar"
#else
           "crossbar"
#endif
    );
    printf(" groups     : RAGU_A=%zu RAGU_B=%zu RAGU_C=%zu RAGU_D=%zu RAGU_DMA=%zu\n",
           ragu_a_src->n_groups_, ragu_b_src->n_groups_, ragu_c_src->n_groups_,
           ragu_d_src->n_groups_, ragu_dma_src->n_groups_);
    printf("             WAGU_A=%zu WAGU_B=%zu WAGU_D=%zu WAGU_DMA=%zu\n", wagu_a_src->n_groups_,
           wagu_b_src->n_groups_, wagu_d_src->n_groups_, wagu_dma_src->n_groups_);
    printf(" actual     : %d cycles\n", actual);
    printf(" ideal      : %d cycles (pipeline=%d + max_groups=%zu)\n", ideal, kPipeFill,
           max_groups);
    printf(" overhead   : +%.1f%%\n", overhead);
    printf("==============================================\n\n");

    {
        std::ofstream sf(out_dir + "/stats.log");
        if (sf) {
            sf << "actual_cycles," << actual << "\n";
            sf << "ideal_cycles," << ideal << "\n";
            sf << "overhead_pct," << static_cast<int>(overhead) << "\n";
            sf << "timed_out," << (actual >= kMaxCycles ? 1 : 0) << "\n";
            sf << "ragu_a_groups," << ragu_a_src->n_groups_ << "\n";
            sf << "ragu_b_groups," << ragu_b_src->n_groups_ << "\n";
            sf << "wagu_a_groups," << wagu_a_src->n_groups_ << "\n";
            sf << "wagu_b_groups," << wagu_b_src->n_groups_ << "\n";
        }
    }

    sc_stop();
    return 0;
}

#undef BIND_DUT_GROUP
