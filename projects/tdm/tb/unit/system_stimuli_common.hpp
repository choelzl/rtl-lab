// -----------------------------------------------------------------------------
// Shared harness for system-level stimuli integration tests (tb_system_*.cpp).
//
// Drives REAL stimuli files (the same ones used by production `edaf sim`
// runs) through the full top<> wrapper via the actual agu.hpp trace driver —
// not raw hand-wiggled OBI signals like tb_top_tdm.cpp — and checks
// end-to-end request/response data integrity, for EITHER backend.
//
// For every RAGU read recorded in an AGU's own access log (agu::log_), the
// returned data must match the most recent WAGU write to that address (or
// zero, if the address was never written) — a black-box, log-driven
// consistency check that exercises the full production path:
//   stimuli file -> agu.hpp parser -> AGU driver -> top.hpp wrapper ->
//   top_tdm/top_crossbar backend -> bank
//
// Includers must #define IMPL_TDM or IMPL_CROSSBAR before including this
// header (matching top.hpp's own selection macros), so the SAME stimuli set
// can be run through both backends and cross-checked: both must pass the
// SAME read-after-write correctness check derived from the SAME write log —
// the only expected difference between backends is cycle count (TDM's
// buffering adds pipeline fill/flush overhead), never correctness.
//
// One SystemC process can only run a single sc_start() lifecycle, so each
// (stimuli case, backend) pair gets its own tb_system_stimuli_<case>[_xbar].cpp
// driver file including this header, rather than looping over cases in one
// binary.
// -----------------------------------------------------------------------------

#ifndef SYSTEM_STIMULI_COMMON_HPP
#define SYSTEM_STIMULI_COMMON_HPP

#if !defined(IMPL_TDM) && !defined(IMPL_CROSSBAR)
#error "system_stimuli_common.hpp: define IMPL_TDM or IMPL_CROSSBAR before including"
#endif

// top.hpp must precede constants.hpp — see tb_top.cpp for why.
#include "top.hpp"

#include <systemc.h>

#include "agu.hpp"
#include "constants.hpp"
#include "lane_agu.hpp"
#include "obi_data.hpp"
#include "unit_test_common.hpp"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

using dut_t  = top<N_BANK, N_ROW, WORD_BYTES, WORDS_PER_ROW>;
using data_t = obi_data<BYTES_PER_ROW>;

// bind_agu() (both overloads) and BIND_DUT_GROUP come from agu_bind_util.hpp
// — shared with tb_top.cpp, which wires the same top<> wrapper the same way
// for the production `edaf sim` entry point.
#include "agu_bind_util.hpp"

// Per-address write history: every write to that address, sorted by cycle —
// NOT collapsed to "last write wins", so a read can be checked against
// whatever was true AT ITS OWN CYCLE rather than the final global state (see
// check_read_after_write()). Stimuli sets like multi_stim intentionally
// cross-reference groups — e.g. RAGU_D reads addresses that WAGU_E (not
// WAGU_D) wrote — so history must be built globally across all writers, not
// paired one WAGU per RAGU; ordering only means the right thing if writes
// from different groups are merged in the order they actually happened.
template <typename... WaguSrcs>
static std::map<uint64_t, std::vector<std::pair<uint64_t, data_t>>>
build_expected_map(const WaguSrcs &...wagus) {
    struct write_t {
        uint64_t cycle;
        uint64_t addr;
        data_t   data;
    };
    std::vector<write_t> writes;
    auto                 collect = [&](const auto &wagu) {
        for (const auto &a : wagu.log_)
            if (a.we)
                writes.push_back({a.cycle, a.addr, a.data});
    };
    (collect(wagus), ...);
    std::stable_sort(writes.begin(), writes.end(),
                     [](const write_t &a, const write_t &b) { return a.cycle < b.cycle; });

    std::map<uint64_t, std::vector<std::pair<uint64_t, data_t>>> history;
    for (const auto &w : writes)
        history[w.addr].push_back({w.cycle, w.data});
    return history;
}

// Check one RAGU group's recorded reads against the per-address write
// history (unwritten-as-of-that-cycle addresses default to zero). Each read
// is compared against the LATEST write strictly before its own cycle, not
// the final global state — a read racing ahead of that address's first-ever
// write (legitimately returning zero) is correct, not a bug, and a fixed
// global map can't tell the difference once ANY later write exists. Returns
// the number of reads that matched a REAL prior write (as opposed to
// defaulting to zero because nothing had written that address yet), so the
// caller can confirm the run actually exercised write-then-read data flow.
template <typename RaguSrc>
static int
check_read_after_write(const RaguSrc                                                      &ragu,
                       const std::map<uint64_t, std::vector<std::pair<uint64_t, data_t>>> &history,
                       const char                                                         *label) {
    int  n_checked = 0, n_real = 0;
    bool ok = true;
    for (const auto &a : ragu.log_) {
        if (a.we)
            continue; // reads only
        ++n_checked;
        const data_t *want_ptr = nullptr;
        auto          it       = history.find(a.addr);
        if (it != history.end()) {
            // Latest write with cycle < a.cycle: upper_bound on cycle, then
            // step back one (writes are sorted by cycle, ties keep insertion
            // order via stable_sort above).
            auto ub = std::upper_bound(
                it->second.begin(), it->second.end(), a.cycle,
                [](uint64_t c, const std::pair<uint64_t, data_t> &w) { return c < w.first; });
            if (ub != it->second.begin())
                want_ptr = &std::prev(ub)->second;
        }
        const bool   real = want_ptr != nullptr;
        const data_t want = real ? *want_ptr : data_t(0);
        if (real)
            ++n_real;
        if (!(a.data == want)) {
            ok = false;
            std::printf("  FAIL  %s: addr=0x%llx cycle=%llu got=0x%08x want=0x%08x\n", label,
                        static_cast<unsigned long long>(a.addr),
                        static_cast<unsigned long long>(a.cycle),
                        static_cast<uint32_t>(a.data.range(31, 0).to_uint()),
                        static_cast<uint32_t>(want.range(31, 0).to_uint()));
        }
    }
    if (n_checked == 0)
        return 0; // no stimuli for this group — nothing to verify
    char lbl[256];
    std::snprintf(lbl, sizeof(lbl),
                  "%s: %d reads checked (%d against real prior writes), all match expected data",
                  label, n_checked, n_real);
    CHECK(ok, lbl);
    return n_real;
}

// Runs the full 9-AGU-group system (mirrors tb_top.cpp) against one stimuli
// directory and checks read-after-write consistency for every RAGU/WAGU pair
// that has real stimuli data. Groups with no stimuli report "no stimuli" and
// are immediately done.
static bool run_stimuli_case(const std::string &stim_dir, const char *label) {
    char sysc_nm[64];
    std::snprintf(sysc_nm, sizeof(sysc_nm), "dut_%s", label);

    sc_clock        clk("clk", CLK_PERIOD_NS, SC_NS);
    sc_signal<bool> rst_ni;

    dut_t dut(sysc_nm);
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

    char nm[9][32];
    std::snprintf(nm[0], sizeof(nm[0]), "%s_ragu_a", label);
    std::snprintf(nm[1], sizeof(nm[1]), "%s_ragu_b", label);
    std::snprintf(nm[2], sizeof(nm[2]), "%s_ragu_c", label);
    std::snprintf(nm[3], sizeof(nm[3]), "%s_ragu_d", label);
    std::snprintf(nm[4], sizeof(nm[4]), "%s_ragu_e", label);
    std::snprintf(nm[5], sizeof(nm[5]), "%s_wagu_a", label);
    std::snprintf(nm[6], sizeof(nm[6]), "%s_wagu_b", label);
    std::snprintf(nm[7], sizeof(nm[7]), "%s_wagu_d", label);
    std::snprintf(nm[8], sizeof(nm[8]), "%s_wagu_e", label);

#if defined(IMPL_TDM)
    // TDM read buffers prefetch a whole N_BANK-cell window at once from the
    // AGU's lookahead_addr()/lookahead_ready() accessors (see agu.hpp and
    // this function's own lookahead wiring below) — a newly-set target
    // address only takes effect once the buffer's current window drains and
    // resets, same as before, but the whole window's worth of upcoming
    // groups can be prefetching at once instead of one group per drain.
    // TDM write buffers need a full N_BANK-cell window filled before the snapshot;
    // pad WAGU traces with addr=0 NOPs out to that boundary (see agu.hpp).
    constexpr agu_target      ragu_tgt   = agu_target::tdm;
    constexpr lane_agu_target dma_tgt    = lane_agu_target::tdm;
    constexpr std::size_t     tdm_window = N_BANK;
#else
    // Crossbar backend: no buffering/windowing on either path — plain
    // one-shot request/grant/response, same as tb_top_crossbar.cpp.
    constexpr agu_target      ragu_tgt   = agu_target::crossbar;
    constexpr lane_agu_target dma_tgt    = lane_agu_target::crossbar;
    constexpr std::size_t     tdm_window = 0;
#endif

    agu<dut_t::RPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_a_src(
        nm[0], stim_dir + "/ragu_a.log", "", ragu_tgt, tdm_window);
    agu<dut_t::RPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_b_src(
        nm[1], stim_dir + "/ragu_b.log", "", ragu_tgt, tdm_window);
    agu<dut_t::RPORT_C_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_c_src(
        nm[2], stim_dir + "/ragu_c.log", "", ragu_tgt, tdm_window);
    agu<dut_t::RPORT_D_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> ragu_d_src(
        nm[3], stim_dir + "/ragu_d.log", "", ragu_tgt, tdm_window);
    // ragu_e/wagu_e use the DMA sub-port stimuli format (see lane_agu.hpp)
    // — a genuinely different descriptor shape from RAGU_A/B/C/D and
    // WAGU_A/B/D, requiring the dedicated lane_agu driver rather than agu<>.
    lane_agu<data_t, BYTES_PER_ROW> ragu_e_src(nm[4], stim_dir + "/ragu_e.log", "",
                                               lane_agu_dir::read, tdm_window, dma_tgt);
    agu<dut_t::WPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> wagu_a_src(
        nm[5], stim_dir + "/wagu_a.log", "", agu_target::crossbar, tdm_window);
    agu<dut_t::WPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> wagu_b_src(
        nm[6], stim_dir + "/wagu_b.log", "", agu_target::crossbar, tdm_window);
    agu<dut_t::WPORT_D_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ> wagu_d_src(
        nm[7], stim_dir + "/wagu_d.log", "", agu_target::crossbar, tdm_window);
    // target (dma_tgt) matters for writes too, not just reads: it
    // gates whether idle lanes must keep asserting req forever (mandatory
    // for the TDM write buffer's group AND-gate) or go idle like a plain
    // crossbar requester — see drive_crossbar_requests()'s comment.
    lane_agu<data_t, BYTES_PER_ROW> wagu_e_src(nm[8], stim_dir + "/wagu_e.log", "",
                                               lane_agu_dir::write, tdm_window, dma_tgt);

    bind_agu(ragu_a_src, clk, rst_ni, done[0], ragu_a);
    bind_agu(ragu_b_src, clk, rst_ni, done[1], ragu_b);
    bind_agu(ragu_c_src, clk, rst_ni, done[2], ragu_c);
    bind_agu(ragu_d_src, clk, rst_ni, done[3], ragu_d);
    bind_agu(ragu_e_src, clk, rst_ni, done[4], ragu_e);
    bind_agu(wagu_a_src, clk, rst_ni, done[5], wagu_a);
    bind_agu(wagu_b_src, clk, rst_ni, done[6], wagu_b);
    bind_agu(wagu_d_src, clk, rst_ni, done[7], wagu_d);
    bind_agu(wagu_e_src, clk, rst_ni, done[8], wagu_e);

#if defined(IMPL_TDM)
    // Each read AGU's own lookahead_addr(w)/lookahead_ready() (its next
    // N_BANK addresses from its own pre-loaded trace, plus whether that
    // content is real yet) feeds the matching read buffer's N_BANK-wide
    // fetch bus through these intermediate signals, driven every cycle in
    // the polling loop below — see stim_bank_common.hpp's identical pattern
    // for the full rationale. dut.impl is top_tdm<>'s public instance (no
    // change to top.hpp's own external interface needed). ragu_e_src is a
    // lane_agu<>, which has no lookahead accessor — left at its default zero,
    // matching buf_r4's existing mostly-NOP placeholder role.
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
#endif

#if defined(IMPL_TDM)
    // Drive each buffer's active_mode/mapping geometry from its AGU descriptor
    // (mirrors tb_top.cpp's IMPL_TDM block). These DUT ports only exist for
    // the native TDM backend (see top.hpp's `#if defined(IMPL_TDM)` guard).
    auto tdm_mode = [](int ports_used, int num_req) -> uint32_t {
        const int g = (num_req > 0 && ports_used > 0) ? ports_used / num_req : 1;
        return (g <= 1) ? 0u : (g <= 2) ? 1u : 2u;
    };
    dut.impl_buf_active_mode[0].write(tdm_mode(ragu_a_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[1].write(tdm_mode(ragu_b_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[2].write(tdm_mode(ragu_c_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[3].write(tdm_mode(ragu_d_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[4].write(tdm_mode(ragu_e_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[5].write(tdm_mode(wagu_a_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[6].write(tdm_mode(wagu_b_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[7].write(tdm_mode(wagu_d_src.ports_used_, dut_t::NUM_REQ));
    dut.impl_buf_active_mode[8].write(tdm_mode(wagu_e_src.ports_used_, dut_t::NUM_REQ));

    constexpr uint64_t kDfltR = 4, kDfltC = 4, kDfltL = 8, kDfltSM = 0;
    auto               get_r = [](const auto &s, uint64_t d) { return s.p_has_crl_ ? s.p_R_ : d; };
    auto               get_c = [](const auto &s, uint64_t d) { return s.p_has_crl_ ? s.p_C_ : d; };
    auto               get_l = [](const auto &s, uint64_t d) { return s.p_has_crl_ ? s.p_L_ : d; };
    auto get_sm    = [](const auto &s, uint64_t d) { return s.p_has_crl_ ? s.p_store_mode_ : d; };
    auto write_map = [&](int idx, const auto &s) {
        dut.impl_buf_map_r[idx].write(get_r(s, kDfltR));
        dut.impl_buf_map_c[idx].write(get_c(s, kDfltC));
        dut.impl_buf_map_l[idx].write(get_l(s, kDfltL));
        dut.impl_buf_map_store_mode[idx].write(get_sm(s, kDfltSM));
    };
    write_map(0, ragu_a_src);
    write_map(1, ragu_b_src);
    write_map(2, ragu_c_src);
    write_map(3, ragu_d_src);
    write_map(4, ragu_e_src);
    write_map(5, wagu_a_src);
    write_map(6, wagu_b_src);
    write_map(7, wagu_d_src);
    write_map(8, wagu_e_src);
#endif

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    // final's real DMA trace has fence cycles up to ~111.5k on its own (see
    // ragu_e.log/wagu_e.log's own #cycle descriptors), before any
    // service-time overhead — 100000 was too tight for that regardless of
    // implementation speed, not a sign of a hang.
    constexpr int kMaxCycles = 300000;
    int           actual     = 0;
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < 9; ++a)
            all = all && done[a].read();
        if (all)
            break;

#if defined(IMPL_TDM)
        // Advance each read AGU's lookahead cursor directly off the matching
        // buffer's OWN observed window_reset pulse (see agu.hpp's
        // advance_lookahead_window() comment on why this can't be derived
        // from the AGU's task_idx_/group_ instead) — checked here, BEFORE
        // this edge, so the cursor already reflects the next window by the
        // time it's written below for the buffer to latch next edge.
        if (dut.impl.buf_r0.snapshot().window_reset)
            ragu_a_src.advance_lookahead_window();
        if (dut.impl.buf_r1.snapshot().window_reset)
            ragu_b_src.advance_lookahead_window();
        if (dut.impl.buf_r2.snapshot().window_reset)
            ragu_c_src.advance_lookahead_window();
        if (dut.impl.buf_r3.snapshot().window_reset)
            ragu_d_src.advance_lookahead_window();
        // Retried every cycle (not just on window_reset): fetch_addr_valid_i
        // being gated on lookahead_ready() means a buffer stuck behind a
        // fence never fetches, so it never produces another window_reset to
        // trigger the advance above — something has to notice the fence
        // clearing on a plain cycle tick instead.
        ragu_a_src.retry_lookahead_fence();
        ragu_b_src.retry_lookahead_fence();
        ragu_c_src.retry_lookahead_fence();
        ragu_d_src.retry_lookahead_fence();
        rd0_lookahead_valid.write(ragu_a_src.lookahead_ready());
        rd1_lookahead_valid.write(ragu_b_src.lookahead_ready());
        rd2_lookahead_valid.write(ragu_c_src.lookahead_ready());
        rd3_lookahead_valid.write(ragu_d_src.lookahead_ready());

        for (int w = 0; w < N_BANK; ++w) {
            rd0_lookahead[w].write(ragu_a_src.lookahead_addr(w));
            rd1_lookahead[w].write(ragu_b_src.lookahead_addr(w));
            rd2_lookahead[w].write(ragu_c_src.lookahead_addr(w));
            rd3_lookahead[w].write(ragu_d_src.lookahead_addr(w));
        }
#endif

        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;
    }

    char lbl[256];
    std::snprintf(lbl, sizeof(lbl), "%s: simulation completed within %d cycles (took %d)", label,
                  kMaxCycles, actual);
    const bool completed = actual < kMaxCycles;
    CHECK(completed, lbl);

    // Build ONE expected map from all writers combined (see
    // build_expected_map()'s comment — multi_stim intentionally has RAGU_D
    // read data that WAGU_E wrote, not WAGU_D), then check every RAGU
    // group against it.
    const auto expected = build_expected_map(wagu_a_src, wagu_b_src, wagu_d_src, wagu_e_src);
    int        n_real   = 0;
    n_real += check_read_after_write(ragu_a_src, expected, label);
    n_real += check_read_after_write(ragu_b_src, expected, label);
    n_real += check_read_after_write(ragu_c_src, expected, label);
    n_real += check_read_after_write(ragu_d_src, expected, label);
    n_real += check_read_after_write(ragu_e_src, expected, label);

    // Guard against a false-positive pass: if the whole datapath silently
    // returned zero for everything, reads of never-written addresses would
    // still "match" (they default to zero too). Require that at least one
    // read actually observed a real prior write's (necessarily non-zero, per
    // this project's stimuli) data, so a totally broken pipeline can't sneak
    // through as "correct" just because nothing was ever really checked.
    if (!expected.empty()) {
        std::snprintf(lbl, sizeof(lbl),
                      "%s: at least one read reflects a real prior write (%d such reads seen)",
                      label, n_real);
        CHECK(n_real > 0, lbl);
    }

    return completed;
}

#undef BIND_DUT_GROUP

static int run_and_report(const std::string &stim_dir, const char *label) {
    run_stimuli_case(stim_dir, label);
    return report_and_exit();
}

// Resolves SEL_IN_DIR (a bare name under tb/stimuli/, or a full path) to an
// actual stimuli directory — shared by tb_system_stimuli_tdm.cpp and
// tb_system_stimuli_xbar.cpp, whose sc_main()s are otherwise identical
// except for the IMPL_TDM/IMPL_CROSSBAR selection and report label.
static std::string resolve_stim_dir(const std::string &in_dir) {
    if (in_dir.find('/') != std::string::npos)
        return in_dir;
    namespace fs   = std::filesystem;
    const char *ch = std::getenv("RTL_LAB_HOME");
    fs::path    proj_dir;
    if (ch) {
        proj_dir = fs::path(ch) / "projects" / "tdm";
    } else {
        // Same CWD anchoring as stim_bank_common.hpp's temp_stim_dir():
        // never re-append projects/tdm when the CWD already is the project.
        const fs::path cwd = fs::current_path();
        proj_dir           = (cwd.filename() == "tdm" && cwd.parent_path().filename() == "projects")
                                 ? cwd
                                 : cwd / "projects" / "tdm";
    }
    return (proj_dir / "tb" / "stimuli" / in_dir).string();
}

#endif // SYSTEM_STIMULI_COMMON_HPP
