// -----------------------------------------------------------------------------
// Shared OBI signal-group plumbing for wiring an agu<>/lane_agu<> trace driver
// to top<>'s flat per-group port arrays — byte-identical code previously
// duplicated between tb_top.cpp (the production `edaf sim` entry point) and
// tb/unit/system_stimuli_common.hpp (the system-level integration test
// harness), since both drive the same top<> wrapper the same way.
//
// Requires the includer to have already defined `data_t` and `BYTES_PER_ROW`
// (both current includers alias `data_t = obi_data<BYTES_PER_ROW>` from
// constants.hpp immediately before including this) — bind_agu()'s agu<>
// overload names them directly rather than taking them as template
// parameters, matching how agu<> itself is always instantiated in this
// project.
// -----------------------------------------------------------------------------

#ifndef AGU_BIND_UTIL_HPP
#define AGU_BIND_UTIL_HPP

#include "agu.hpp"
#include "lane_agu.hpp"
#include "obi_ports.hpp"
#include <systemc.h>

template <int N, int NPG>
static void bind_agu(agu<N, data_t, BYTES_PER_ROW, NPG> &src, sc_clock &clk,
                     sc_signal<bool> &rst_ni, sc_signal<bool> &done,
                     obi_signal_bundle<data_t> (&bus)[N]) {
    src.clk_i(clk);
    src.rst_ni(rst_ni);
    src.done_o(done);
    for (int p = 0; p < N; ++p)
        bind_obi(src.obi[p], bus[p]);
}

// Same port list as agu<>'s overload above — lane_agu is the dedicated driver
// for RAGU_E/WAGU_E's own stimuli format (see lane_agu.hpp), matched here
// by its own NUM_REQ rather than a caller-supplied N since DMA is always
// exactly 4 physical lanes.
template <typename DATA_T, int BYTES>
static void bind_agu(lane_agu<DATA_T, BYTES> &src, sc_clock &clk, sc_signal<bool> &rst_ni,
                     sc_signal<bool> &done,
                     obi_signal_bundle<DATA_T> (&bus)[lane_agu<DATA_T, BYTES>::NUM_REQ]) {
    src.clk_i(clk);
    src.rst_ni(rst_ni);
    src.done_o(done);
    for (int p = 0; p < lane_agu<DATA_T, BYTES>::NUM_REQ; ++p)
        bind_obi(src.obi[p], bus[p]);
}

#define BIND_DUT_GROUP(dut, prefix, bus, N)                                                        \
    do {                                                                                           \
        for (int p = 0; p < (N); ++p)                                                              \
            bind_obi((dut).prefix[p], (bus)[p]);                                                   \
    } while (0)

#endif // AGU_BIND_UTIL_HPP
