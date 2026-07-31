// -----------------------------------------------------------------------------
// Author: Cedric Hoelzl
//
// Description:
//   Unified SystemC harness for rtl/systemc/top.hpp.  It connects RAGU/WAGU trace drivers to
//   the fixed wrapper map:
//     read : RAGU_A trace drives RPORT_A[0..3], RAGU_B drives RPORT_B[0..1],
//            RAGU_C/RAGU_D/RAGU_E each drive one RPORT.
//     write: WAGU_A trace drives WPORT_A[0..3], WAGU_B drives WPORT_B[0..1],
//            WAGU_D/WAGU_E each drive one WPORT
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
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "agu.hpp"
#include "constants.hpp"
#include "lane_agu.hpp"
#include "obi_monitor.hpp"

static constexpr int kPipeFill = 2;
using dut_t                    = top<N_BANK, N_ROW, WORD_BYTES, WORDS_PER_ROW>;
using data_t                   = obi_data<BYTES_PER_ROW>;

// bind_agu() (both overloads) and BIND_DUT_GROUP come from agu_bind_util.hpp
// — shared with system_stimuli_common.hpp, which wires the same top<>
// wrapper the same way for the system-level integration tests.
#include "agu_bind_util.hpp"

static std::string env_or(const char *key, const std::string &dflt) {
    const char *v = std::getenv(key);
    return v ? std::string(v) : dflt;
}

template <int N>
static void bind_monitor(obi_monitor<N, BYTES_PER_ROW> &mon, sc_clock &clk, sc_signal<bool> &rst_ni,
                         obi_signal_bundle<data_t> (&bus)[N]) {
    mon.clk_i(clk);
    mon.rst_ni(rst_ni);
    for (int p = 0; p < N; ++p)
        bind_obi(mon.obi[p], bus[p]);
}

// SEL_VCD support: trace one OBI bundle array's 8 fields per lane under
// descriptive names ("<prefix>[i].req" etc.) — sc_signal's own auto-generated
// name is unique but not readable in a waveform viewer.
// Zero-pad a small non-negative index to 2 digits so scope/signal names sort
// lexically in waveform viewers (00,01,..,09,10,11 instead of 0,1,10,11,..,2).
static std::string z2(int v) {
    const std::string s = std::to_string(v);
    return s.size() < 2 ? std::string(2 - s.size(), '0') + s : s;
}

template <int N>
static void trace_group(sc_trace_file *tf, const char *prefix, obi_signal_bundle<data_t> (&bus)[N]) {
    for (int i = 0; i < N; ++i) {
        // <prefix>.<port>.<field> — prefix is e.g. "AGU_A.READ", so each port
        // 0..N-1 becomes a sub-scope holding the full OBI interface.
        const std::string base = std::string(prefix) + "." + z2(i) + ".";
        sc_trace(tf, bus[i].req, base + "req");
        sc_trace(tf, bus[i].gnt, base + "gnt");
        sc_trace(tf, bus[i].addr, base + "addr");
        sc_trace(tf, bus[i].be, base + "be");
        sc_trace(tf, bus[i].we, base + "we");
        sc_trace(tf, bus[i].wdata, base + "wdata");
        sc_trace(tf, bus[i].rvalid, base + "rvalid");
        sc_trace(tf, bus[i].rdata, base + "rdata");
    }
}

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
    // Skips per-event CSV logs (AGU response logs and OBI monitors) entirely
    // when set — useful for large sweeps where only the printed/stats.log
    // summary is needed, since these per-cycle logs dominate I/O time.
    const bool no_monitor = std::getenv("SEL_NO_MONITOR") != nullptr;
    // SEL_DESC_SYNC (crossbar only): per-descriptor barrier — the 5 agu groups
    // (ragu_a/b/c, wagu_a/b) synchronize at each descriptor boundary (fences
    // ignored, tasks NOT merged), so descriptor N is directly comparable across
    // builds. DMA (ragu_d/e, wagu_d/e) runs free as background traffic. Emits
    // out_dir/desc_sync.csv (drain cycles + L1/L2/L3 split per descriptor). See
    // doc/xbar_hash_l1_performance.md. Must NOT be combined with SEL_NO_FENCE,
    // whose task-merge would erase the descriptor boundaries this relies on.
    const bool desc_sync = std::getenv("SEL_DESC_SYNC") != nullptr;
    if (desc_sync && std::getenv("SEL_NO_FENCE"))
        fprintf(stderr, "WARNING: SEL_DESC_SYNC with SEL_NO_FENCE — the latter's "
                        "task-merge destroys descriptor boundaries; run SEL_DESC_SYNC alone\n");
    // SEL_BANK_TRACE: write one line per cycle with the number of banks
    // served (req&&gnt) that cycle — the per-cycle parallelism timeline
    // (doc/report §5.1). Cheap: one small integer per cycle.
    std::ofstream bank_trace;
    if (std::getenv("SEL_BANK_TRACE"))
        bank_trace.open(out_dir + "/bank_trace.csv");
    // SEL_VCD: dump a VCD waveform of every OBI request (all 9 AGU-facing
    // port groups) plus the bank-facing stage (IMPL_TDM: mux_tdm/xbar_bank
    // and the tdm mapping function's per-lane bank_id/row_id/conflict;
    // IMPL_CROSSBAR: l3_bank) to out_dir/wave.vcd — for interactive
    // inspection (gtkwave), not for any automated stat.
    const bool  sel_vcd = std::getenv("SEL_VCD") != nullptr;
    sc_trace_file *tf   = nullptr;
    // SEL_MAP_CONFLICT_LOG (IMPL_TDM only): one row per cycle/lane where the
    // mapping function's XOR hash landed on the same bank_id as another real
    // lane in the same group — the collision the downstream per-bank
    // arbiter must then serialize.
    std::ofstream map_conflict_log;
#if defined(IMPL_TDM) && !defined(IMPL_SV)
    if (std::getenv("SEL_MAP_CONFLICT_LOG")) {
        map_conflict_log.open(out_dir + "/map_conflicts.csv");
        if (map_conflict_log)
            map_conflict_log << "cycle,lane,bank_id,row_id\n";
    }
#endif
    auto log_path = [&](const std::string &name) -> std::string {
        return no_monitor ? std::string() : out_dir + "/" + name;
    };

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

#if defined(IMPL_TDM) || defined(IMPL_TDM_SC)
    constexpr agu_target ragu_tgt = agu_target::tdm;
    // TDM write buffers accumulate a full N_BANK-cell window before they can
    // flush; pad WAGU traces with addr=0 NOPs out to that window boundary so
    // a task's real trace ending mid-window doesn't stall fill forever.
    // TDM read buffers continuously prefetch from the AGU's own port
    // addresses, so a newly-set target address only takes effect after one
    // full window drains and resets — RAGU AGUs use the same window size to
    // drive that drain/capture/drain-remainder protocol (see agu.hpp).
    constexpr std::size_t     tdm_window = N_BANK;
    constexpr lane_agu_target dma_tgt    = lane_agu_target::tdm;
#else
    constexpr agu_target      ragu_tgt   = agu_target::crossbar;
    constexpr std::size_t     tdm_window = 0;
    constexpr lane_agu_target dma_tgt    = lane_agu_target::crossbar;
#endif

    auto ragu_a_src =
        std::make_unique<agu<dut_t::RPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_a", stim_dir + "/ragu_a.log", log_path("ragu_a.csv"), ragu_tgt, tdm_window);
    auto ragu_b_src =
        std::make_unique<agu<dut_t::RPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_b", stim_dir + "/ragu_b.log", log_path("ragu_b.csv"), ragu_tgt, tdm_window);
    auto ragu_c_src =
        std::make_unique<agu<dut_t::RPORT_C_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "ragu_c", stim_dir + "/ragu_c.log", log_path("ragu_c.csv"), ragu_tgt, tdm_window);
    // ragu_d/wagu_d and ragu_e/wagu_e use the DMA sub-port stimuli format
    // (see lane_agu.hpp) — a genuinely different descriptor shape from
    // RAGU_A/B/C and WAGU_A/B, requiring the dedicated lane_agu driver
    // rather than agu<>.
    auto ragu_d_src = std::make_unique<lane_agu<data_t, BYTES_PER_ROW>>(
        "ragu_d", stim_dir + "/ragu_d.log", log_path("ragu_d.csv"), lane_agu_dir::read, tdm_window,
        dma_tgt);
    auto ragu_e_src = std::make_unique<lane_agu<data_t, BYTES_PER_ROW>>(
        "ragu_e", stim_dir + "/ragu_e.log", log_path("ragu_e.csv"), lane_agu_dir::read, tdm_window,
        dma_tgt);
    auto wagu_a_src =
        std::make_unique<agu<dut_t::WPORT_A_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_a", stim_dir + "/wagu_a.log", log_path("wagu_a.csv"), agu_target::crossbar,
            tdm_window);
    auto wagu_b_src =
        std::make_unique<agu<dut_t::WPORT_B_PORTS, data_t, BYTES_PER_ROW, dut_t::NUM_REQ>>(
            "wagu_b", stim_dir + "/wagu_b.log", log_path("wagu_b.csv"), agu_target::crossbar,
            tdm_window);
    auto wagu_d_src = std::make_unique<lane_agu<data_t, BYTES_PER_ROW>>(
        "wagu_d", stim_dir + "/wagu_d.log", log_path("wagu_d.csv"), lane_agu_dir::write, tdm_window,
        dma_tgt);
    auto wagu_e_src = std::make_unique<lane_agu<data_t, BYTES_PER_ROW>>(
        "wagu_e", stim_dir + "/wagu_e.log", log_path("wagu_e.csv"), lane_agu_dir::write, tdm_window,
        dma_tgt);

    bind_agu(*ragu_a_src, clk, rst_ni, done[0], ragu_a);
    bind_agu(*ragu_b_src, clk, rst_ni, done[1], ragu_b);
    bind_agu(*ragu_c_src, clk, rst_ni, done[2], ragu_c);
    bind_agu(*ragu_d_src, clk, rst_ni, done[3], ragu_d);
    bind_agu(*ragu_e_src, clk, rst_ni, done[4], ragu_e);
    bind_agu(*wagu_a_src, clk, rst_ni, done[5], wagu_a);
    bind_agu(*wagu_b_src, clk, rst_ni, done[6], wagu_b);
    bind_agu(*wagu_d_src, clk, rst_ni, done[7], wagu_d);
    bind_agu(*wagu_e_src, clk, rst_ni, done[8], wagu_e);

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
    if (desc_sync) {
        // All 9 groups ignore start_cycle fences (barrier controls timing);
        // the 5 agu groups additionally hold at each descriptor boundary until
        // released together. DMA (d/e) has no barrier — it just runs free.
        ragu_a_src->ignore_fence_ = ragu_b_src->ignore_fence_ = ragu_c_src->ignore_fence_ = true;
        ragu_d_src->ignore_fence_ = ragu_e_src->ignore_fence_ = true;
        wagu_a_src->ignore_fence_ = wagu_b_src->ignore_fence_ = true;
        wagu_d_src->ignore_fence_ = wagu_e_src->ignore_fence_ = true;
        ragu_a_src->desc_barrier_hold_ = ragu_b_src->desc_barrier_hold_ =
            ragu_c_src->desc_barrier_hold_ = true;
        wagu_a_src->desc_barrier_hold_ = wagu_b_src->desc_barrier_hold_ = true;
    }
#endif

    // OBI monitors — one per AGU group; follow same path convention as AGU logs.
    auto mon_path = [&](const char *name) -> std::string {
        return log_path(std::string("obi_") + name + ".csv");
    };
    auto mon_ragu_a = std::make_unique<obi_monitor<dut_t::RAGU_A_PORTS, BYTES_PER_ROW>>(
        "mon_ragu_a", "RAGU_A", mon_path("ragu_a"));
    auto mon_ragu_b = std::make_unique<obi_monitor<dut_t::RAGU_B_PORTS, BYTES_PER_ROW>>(
        "mon_ragu_b", "RAGU_B", mon_path("ragu_b"));
    auto mon_ragu_c = std::make_unique<obi_monitor<dut_t::RAGU_C_PORTS, BYTES_PER_ROW>>(
        "mon_ragu_c", "RAGU_C", mon_path("ragu_c"));
    auto mon_ragu_d = std::make_unique<obi_monitor<dut_t::RAGU_D_PORTS, BYTES_PER_ROW>>(
        "mon_ragu_d", "RAGU_D", mon_path("ragu_d"));
    auto mon_ragu_e = std::make_unique<obi_monitor<dut_t::RAGU_E_PORTS, BYTES_PER_ROW>>(
        "mon_ragu_e", "RAGU_E", mon_path("ragu_e"));
    auto mon_wagu_a = std::make_unique<obi_monitor<dut_t::WAGU_A_PORTS, BYTES_PER_ROW>>(
        "mon_wagu_a", "WAGU_A", mon_path("wagu_a"));
    auto mon_wagu_b = std::make_unique<obi_monitor<dut_t::WAGU_B_PORTS, BYTES_PER_ROW>>(
        "mon_wagu_b", "WAGU_B", mon_path("wagu_b"));
    auto mon_wagu_d = std::make_unique<obi_monitor<dut_t::WAGU_D_PORTS, BYTES_PER_ROW>>(
        "mon_wagu_d", "WAGU_D", mon_path("wagu_d"));
    auto mon_wagu_e = std::make_unique<obi_monitor<dut_t::WAGU_E_PORTS, BYTES_PER_ROW>>(
        "mon_wagu_e", "WAGU_E", mon_path("wagu_e"));

    bind_monitor(*mon_ragu_a, clk, rst_ni, ragu_a);
    bind_monitor(*mon_ragu_b, clk, rst_ni, ragu_b);
    bind_monitor(*mon_ragu_c, clk, rst_ni, ragu_c);
    bind_monitor(*mon_ragu_d, clk, rst_ni, ragu_d);
    bind_monitor(*mon_ragu_e, clk, rst_ni, ragu_e);
    bind_monitor(*mon_wagu_a, clk, rst_ni, wagu_a);
    bind_monitor(*mon_wagu_b, clk, rst_ni, wagu_b);
    bind_monitor(*mon_wagu_d, clk, rst_ni, wagu_d);
    bind_monitor(*mon_wagu_e, clk, rst_ni, wagu_e);

#if defined(IMPL_TDM) && !defined(IMPL_SV)
    // Read buffers prefetch a whole window ahead of the port side (see
    // top_tdm.hpp's own header comment) — each read AGU exposes its next
    // N_BANK addresses via lookahead_addr(w), which must be wired to the
    // matching buffer's fetch_addr_i bus and re-driven every cycle (see
    // agu.hpp's own note: "tb_top should re-drive map_*_cfg each clock
    // cycle from these fields"). dut.impl is top_tdm<>'s public instance
    // (top.hpp has no private: sections), so this needs no change to
    // top.hpp's own external interface — mirrors stim_bank_common.hpp's/
    // system_stimuli_common.hpp's identical pattern for the unit and
    // system-integration suites. ragu_d_src/ragu_e_src are lane_agu<>,
    // whose window packs multiple tasks per sub-port directly (see
    // lane_agu.hpp) rather than exposing agu<>'s separate lookahead_ready()
    // gate — buf_r3/buf_r4's fetch_addr_valid_i is therefore just held
    // true for those two (lane_agu's content is always well-defined, real
    // or NOP; buf_r4 has no external _valid_i port at all, hardwired true
    // inside top_tdm.hpp itself).
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

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) && defined(XBAR_ROB)
    // Read-side reorder buffers (top_crossbar.hpp's XBAR_ROB experiment):
    // per-ROB-port fetch buses fed from each agu<>-driven read AGU's
    // lookahead cursor, group-granular — the crossbar analogue of the TDM
    // lookahead wiring above. ROB ports 0-3 = ragu_a, 4-5 = ragu_b,
    // 6 = ragu_c; ragu_d/e (lane_agu, sequential DMA-style traffic) keep
    // the plain fabric path.
    using impl_rob_t = std::remove_reference_t<decltype(dut.impl)>;
    constexpr int kRobPorts = impl_rob_t::ROB_PORTS;
    constexpr int kRobLanes = impl_rob_t::ROB_LANES;
    static_assert(kRobPorts == 7, "fetch wiring below assumes ragu_a(4)+ragu_b(2)+ragu_c(1)");
    sc_signal<uint64_t> rob_fetch_addr[kRobLanes];
    sc_signal<bool>     rob_fetch_valid[kRobPorts], rob_fetch_ready[kRobPorts],
        rob_fetch_ack[kRobPorts];
    sc_signal<uint64_t> rob_la_r[kRobPorts], rob_la_c[kRobPorts], rob_la_l[kRobPorts],
        rob_la_sm[kRobPorts], rob_la_napa[kRobPorts];
    for (int w = 0; w < kRobLanes; ++w)
        dut.impl.rob_.fetch_addr_i[w](rob_fetch_addr[w]);
    for (int j = 0; j < kRobPorts; ++j) {
        dut.impl.rob_.fetch_valid_i[j](rob_fetch_valid[j]);
        dut.impl.rob_.fetch_ready_o[j](rob_fetch_ready[j]);
        dut.impl.rob_.fetch_ack_o[j](rob_fetch_ack[j]);
        dut.impl.rob_.la_r_i[j](rob_la_r[j]);
        dut.impl.rob_.la_c_i[j](rob_la_c[j]);
        dut.impl.rob_.la_l_i[j](rob_la_l[j]);
        dut.impl.rob_.la_sm_i[j](rob_la_sm[j]);
        dut.impl.rob_.la_napa_i[j](rob_la_napa[j]);
    }
#endif

#if defined(IMPL_TDM) && !defined(IMPL_SV) && !defined(IMPL_ARB_ADAPTIVE)
    // Free-running RR slot table: the final/N stimuli never drive WAGU_B (no
    // wagu_b.log exists in any set), so its buffer never has work — drop it
    // from the rotation (9 -> 8 slots) so the shared bus stops spending one
    // turn per revolution on an always-idle client. SEL_ARB_FULL9 restores
    // the full identity rotation for A/B comparison. See top_tdm.hpp's
    // client-list comment; the adaptive arbiter needs no table (it skips
    // requestless clients cycle-by-cycle).
    if (!std::getenv("SEL_ARB_FULL9")) {
        using impl_t                = decltype(dut.impl);
        static const int kArbSeq8[] = {
            impl_t::BUF_RAGU_A, impl_t::BUF_RAGU_B, impl_t::BUF_RAGU_C, impl_t::BUF_RAGU_D,
            impl_t::BUF_RAGU_E, impl_t::BUF_WAGU_A, impl_t::BUF_WAGU_D, impl_t::BUF_WAGU_E,
        };
        dut.impl.set_arb_sequence(kArbSeq8, 8);
    }
#endif

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
    // Per-port L1 collision flags (one per physical RPORT/WPORT, i.e. one
    // per 4-lane L1 instance): true this cycle iff that port's own 4
    // concurrent sub-lanes had more real requesters than the L1 stage
    // forwarded to l1_l2_{rd,wr} — i.e. >=2 of its lanes hashed to the same
    // L1 output (addr[5:4] by default; addr[5:4] is unhashed unless built
    // with -DXBAR_HASH_L1, see top_crossbar.hpp's addr_hash()). Declared
    // here (before SEL_VCD tracing) and populated per-cycle in the main
    // loop's crossbar accounting block below.
    sc_signal<bool> l1_conflict_rd[9];
    sc_signal<bool> l1_conflict_wr[8];
    // SEL_VCD per-cycle conflict-wait view: number of requesters blocked *at*
    // each crossbar stage this cycle (the telescoped d[0..2] = L1/L2/L3, rd+wr)
    // plus the raw req && !gnt input-port counts. Written in the crossbar
    // accounting block below, traced under `wait.` — lets you watch conflict
    // spikes line up with the `<group>.desc` label in Surfer per descriptor.
    sc_signal<sc_uint<16>> wait_lvl_l1, wait_lvl_l2, wait_lvl_l3;
    sc_signal<sc_uint<16>> wait_cyc_rd, wait_cyc_wr;  // = cyc_wait_rd/wr (input req && !gnt)
    // Accumulated (running-sum) conflict wait-cycles per level — the same
    // lvl_rd[0..2]/lvl_wr[0..2] totals printed on the `conflict lvl` line. 32-bit
    // (the totals exceed 16-bit) traced as wait.lvl_{rd,wr}_l{1,2,3}.
    sc_signal<sc_uint<32>> lvl_rd_sig[3], lvl_wr_sig[3];
#endif

    // SEL_VCD descriptor label as ONE `<group>.desc` signal: a standard
    // bit-vector holding the current descriptor's label as ASCII bytes (char[0]
    // in the MS byte, space-padded). In Surfer, add the signal and set its value
    // format to ASCII to read it as text; the label mirrors the stimulus line
    // "#cycle,num_port_active,R,C,L,sm" exactly (see doc/specs/stimuli.md). A
    // plain bit-vector is used (not the VCD string extension) because that is
    // the one representation every VCD reader parses — underscores keep every
    // field separated without whitespace.
    static constexpr int        kDescChars = 48;          // ASCII chars in the label field
    static constexpr int        kDescBits  = kDescChars * 8;
    sc_signal<sc_bv<kDescBits>> desc_sig[5];
    std::vector<std::string>    desc_labels[5];
    auto desc_label_of = [](const auto &t, std::size_t i) {
        std::string s = "d" + std::to_string(i) + "_cyc" + std::to_string(t.start_cycle) +
                        "_npa" + std::to_string(t.num_port_active);
        if (t.has_crl)
            s += "_R" + std::to_string(t.R) + "_C" + std::to_string(t.C) + "_L" + std::to_string(t.L);
        return s + "_sm" + std::to_string(t.store_mode);
    };
    auto pack_ascii = [](const std::string &s) {
        sc_bv<kDescBits> v;
        for (int i = 0; i < kDescChars; ++i) {
            const int c  = i < static_cast<int>(s.size()) ? static_cast<unsigned char>(s[i]) : ' ';
            const int hi = kDescBits - 1 - i * 8; // char[0] -> most-significant byte
            v.range(hi, hi - 7) = c;
        }
        return v;
    };

    if (sel_vcd) {
        tf = sc_create_vcd_trace_file((out_dir + "/wave").c_str());
        sc_trace(tf, clk, "clk");
        sc_trace(tf, rst_ni, "rst_ni");
        // Group each AGU letter under one AGU_<L> scope, split READ (ragu_*) /
        // WRITE (wagu_*). AGU_C is read-only (no wagu_c in hardware).
        trace_group(tf, "AGU_A.READ", ragu_a);
        trace_group(tf, "AGU_A.WRITE", wagu_a);
        trace_group(tf, "AGU_B.READ", ragu_b);
        trace_group(tf, "AGU_B.WRITE", wagu_b);
        trace_group(tf, "AGU_C.READ", ragu_c);
        trace_group(tf, "AGU_D.READ", ragu_d);
        trace_group(tf, "AGU_D.WRITE", wagu_d);
        trace_group(tf, "AGU_E.READ", ragu_e);
        trace_group(tf, "AGU_E.WRITE", wagu_e);
        // One `<group>.desc` signal per barriered agu group: an ASCII bit-vector
        // holding the current descriptor's label (set format=ASCII in Surfer).
        // Labels are precomputed from each group's already-parsed tasks_; the
        // main loop packs the current one each cycle.
        {
            const char *strn[5] = {"ragu_a.desc", "ragu_b.desc", "ragu_c.desc",
                                   "wagu_a.desc", "wagu_b.desc"};
            auto setup = [&](int g, const auto &src) {
                for (std::size_t i = 0; i < src->tasks_.size(); ++i)
                    desc_labels[g].push_back(desc_label_of(src->tasks_[i], i));
                sc_trace(tf, desc_sig[g], strn[g]);
            };
            setup(0, ragu_a_src);
            setup(1, ragu_b_src);
            setup(2, ragu_c_src);
            setup(3, wagu_a_src);
            setup(4, wagu_b_src);
        }
#if defined(IMPL_TDM) && !defined(IMPL_SV)
        for (int w = 0; w < N_BANK; ++w) {
            // Group banks 8 per scope: banks.<lo>-<hi>.<w>.<field>
            const int lo = (w / 8) * 8;
            const std::string base = "banks." + z2(lo) + "-" + z2(lo + 7) + "." + z2(w) + ".";
            // Full OBI bank interface (xbar_bank = the bank's OBI port)
            sc_trace(tf, dut.impl.xbar_bank[w].req, base + "req");
            sc_trace(tf, dut.impl.xbar_bank[w].gnt, base + "gnt");
            sc_trace(tf, dut.impl.xbar_bank[w].addr, base + "addr");
            sc_trace(tf, dut.impl.xbar_bank[w].be, base + "be");
            sc_trace(tf, dut.impl.xbar_bank[w].we, base + "we");
            sc_trace(tf, dut.impl.xbar_bank[w].wdata, base + "wdata");
            sc_trace(tf, dut.impl.xbar_bank[w].rvalid, base + "rvalid");
            sc_trace(tf, dut.impl.xbar_bank[w].rdata, base + "rdata");
            // TDM-specific diagnostics (mux arbitration + address map)
            sc_trace(tf, dut.impl.mux_tdm[w].req, base + "mux_req");
            sc_trace(tf, dut.impl.mux_tdm[w].gnt, base + "mux_gnt");
            sc_trace(tf, dut.impl.mux_tdm[w].addr, base + "mux_addr");
            sc_trace(tf, dut.impl.mapf.bank_id_o[w], base + "map_bank_id");
            sc_trace(tf, dut.impl.mapf.row_id_o[w], base + "map_row_id");
            sc_trace(tf, dut.impl.mapf.conflict_o[w], base + "map_conflict");
        }
#elif defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
        for (int b = 0; b < decltype(dut.impl)::NUM_PHYS_BANKS; ++b) {
            // Group banks 8 per scope: banks.<lo>-<hi>.<b>.<field>
            const int lo = (b / 8) * 8;
            const std::string base = "banks." + z2(lo) + "-" + z2(lo + 7) + "." + z2(b) + ".";
            sc_trace(tf, dut.impl.l3_bank[b].req, base + "req");
            sc_trace(tf, dut.impl.l3_bank[b].gnt, base + "gnt");
            sc_trace(tf, dut.impl.l3_bank[b].addr, base + "addr");
            sc_trace(tf, dut.impl.l3_bank[b].be, base + "be");
            sc_trace(tf, dut.impl.l3_bank[b].we, base + "we");
            sc_trace(tf, dut.impl.l3_bank[b].wdata, base + "wdata");
            sc_trace(tf, dut.impl.l3_bank[b].rvalid, base + "rvalid");
            sc_trace(tf, dut.impl.l3_bank[b].rdata, base + "rdata");
        }
        // L1 stage (4x4 per physical port): l1_l2_rd/wr is L1's OUTPUT side
        // (post-arbitration, feeding L2) — flat-indexed j*NUM_REQ+m, the
        // same global lane numbering as the AGU-facing ragu_*/wagu_* arrays
        // traced above, so l1_l2_rd(i) lines up directly with e.g.
        // ragu_a(i) for i in RAGU_A's own lane range.
        // Grouped as L1_IF_OUT.<READ|WRITE>.<lo>-<hi>.<lane>.<field>, 4 lanes
        // per scope (one physical port's 4-lane L1 instance).
        for (int i = 0; i < decltype(dut.impl)::NUM_L1_L2_RD; ++i) {
            const int lo = (i / 4) * 4;
            const std::string base = "L1_IF_OUT.READ." + z2(lo) + "-" + z2(lo + 3) + "." + z2(i) + ".";
            sc_trace(tf, dut.impl.l1_l2_rd[i].req, base + "req");
            sc_trace(tf, dut.impl.l1_l2_rd[i].gnt, base + "gnt");
            sc_trace(tf, dut.impl.l1_l2_rd[i].addr, base + "addr");
            sc_trace(tf, dut.impl.l1_l2_rd[i].be, base + "be");
            sc_trace(tf, dut.impl.l1_l2_rd[i].we, base + "we");
            sc_trace(tf, dut.impl.l1_l2_rd[i].wdata, base + "wdata");
            sc_trace(tf, dut.impl.l1_l2_rd[i].rvalid, base + "rvalid");
            sc_trace(tf, dut.impl.l1_l2_rd[i].rdata, base + "rdata");
        }
        for (int i = 0; i < decltype(dut.impl)::NUM_L1_L2_WR; ++i) {
            const int lo = (i / 4) * 4;
            const std::string base = "L1_IF_OUT.WRITE." + z2(lo) + "-" + z2(lo + 3) + "." + z2(i) + ".";
            sc_trace(tf, dut.impl.l1_l2_wr[i].req, base + "req");
            sc_trace(tf, dut.impl.l1_l2_wr[i].gnt, base + "gnt");
            sc_trace(tf, dut.impl.l1_l2_wr[i].addr, base + "addr");
            sc_trace(tf, dut.impl.l1_l2_wr[i].be, base + "be");
            sc_trace(tf, dut.impl.l1_l2_wr[i].we, base + "we");
            sc_trace(tf, dut.impl.l1_l2_wr[i].wdata, base + "wdata");
            sc_trace(tf, dut.impl.l1_l2_wr[i].rvalid, base + "rvalid");
            sc_trace(tf, dut.impl.l1_l2_wr[i].rdata, base + "rdata");
        }
        for (int j = 0; j < 9; ++j)
            sc_trace(tf, l1_conflict_rd[j], "l1_conflict_rd(" + std::to_string(j) + ")");
        for (int j = 0; j < 8; ++j)
            sc_trace(tf, l1_conflict_wr[j], "l1_conflict_wr(" + std::to_string(j) + ")");
        // Per-cycle conflict-wait counts (see declaration), grouped under `wait.`
        sc_trace(tf, wait_cyc_rd, "wait.cyc_wait_rd");
        sc_trace(tf, wait_cyc_wr, "wait.cyc_wait_wr");
        sc_trace(tf, wait_lvl_l1, "wait.l1");
        sc_trace(tf, wait_lvl_l2, "wait.l2");
        sc_trace(tf, wait_lvl_l3, "wait.l3");
        {
            static const char *ln[3] = {"l1", "l2", "l3"};
            for (int k = 0; k < 3; ++k) {
                sc_trace(tf, lvl_rd_sig[k], std::string("wait.lvl_rd_") + ln[k]);
                sc_trace(tf, lvl_wr_sig[k], std::string("wait.lvl_wr_") + ln[k]);
            }
        }
#endif
    }

    rst_ni.write(false);
    sc_start(3 * CLK_PERIOD_NS + CLK_PERIOD_NS / 2, SC_NS);
    rst_ni.write(true);

    constexpr int kMaxCycles = 1000000;
    int           actual     = 0;

    // --- utilization / conflict counters, sampled once per simulated cycle ---
    // port_serve/port_wait: over all 68 flat input ports, beats accepted
    // (req&&gnt) vs cycles a raised request was not yet granted (req&&!gnt) —
    // the end-to-end wait the requesters actually see, whatever the internal
    // mechanism (bank conflict, crossbar-level arbitration, TDM bus turn or
    // window refetch). bank_busy/bank_stall: same tally at the physical-bank
    // bundles (TDM: 32 banks behind the beat crossbar; crossbar: 64 physical
    // banks) — busy bank-cycles give the memory-array utilization floor.
    uint64_t port_serve = 0, port_wait = 0;
    uint64_t bank_busy = 0, bank_stall = 0;
    // Per-beat conflict accounting, split per AGU group: a beat's delay is
    // the run of consecutive req&&!gnt cycles its port saw before the grant
    // landed. The zero-wait default (crossbar grants combinationally, a TDM
    // buffer with prefetched data grants immediately) means every delayed
    // beat was blocked by contention somewhere — delayed-beat count IS the
    // conflict count, and delay_sum is its total cycle cost. Only REAL
    // traffic counts: a request carrying addr 0 is NOP padding / a
    // between-tasks flush / fence parking, so its waits are protocol idle
    // time, not conflicts (its grants are tallied as nop_beats instead),
    // and rates are conflicts per real beat — comparable across backends.
    // active_cycles counts cycles where the group drove >=1 real-address
    // request. gnt->rvalid latency is tracked separately on read ports to
    // confirm the response stage itself is deterministic, i.e. all conflict
    // delay sits in the accept stage.
    // fill_* separates delayed beats whose wait run began from group-idle
    // (fence gap / task start — pipeline fill latency, not contention) from
    // in-stream stalls. episodes counts group-cycles in which >=1 waited
    // beat was granted: a whole group released by one window stall is one
    // episode, not PORT_COUNT "conflicts" — the event-level view that makes
    // TDM's batched stalls comparable to the crossbar's per-lane retries.
    struct grp_stat_t {
        const char *name;
        bool        is_rd;
        uint64_t    real_beats = 0, nop_beats = 0, delayed = 0, delay_sum = 0, delay_max = 0;
        uint64_t    active_cycles = 0;
        uint64_t    fill_delayed = 0, fill_delay_sum = 0;
        uint64_t    episodes = 0, stall_episodes = 0;
        // Wall-clock delay: cycles this unit had >=1 real-address request
        // waiting. Immune to parallel-lane multiplication (a window stall
        // that parks 16 lanes for 40 cycles costs 40 wait_cycles, not 640
        // summed lane-cycles) — the number that actually maps to latency.
        // serve_cycles is its complement (cycles delivering >=1 real beat):
        // active_cycles stretches beyond serve_cycles exactly by the
        // conflict wait, so activity and conflict cost correlate directly.
        // fill_wait_cycles: wait cycles where EVERY waiting lane was in a
        // fill run (task-start pipeline latency) — subtracted from
        // wait_cycles to get pure contention wait. A cycle with >=1
        // contention-run lane waiting counts as contention.
        uint64_t wait_cycles = 0, serve_cycles = 0, fill_wait_cycles = 0;
        // Method-1 (per-beat) level attribution, crossbar builds only (see
        // blocked_level[]/touched_lvl below): a delayed beat may touch more
        // than one level across its wait streak, so delayed_l1+l2+l3 can
        // exceed `delayed` — each counts "was this beat EVER blocked here",
        // not a partition of it.
        uint64_t delayed_l1 = 0, delayed_l2 = 0, delayed_l3 = 0;
        // Method-2 (per-AGU cycle-inflation), crossbar builds only (see
        // wait_lvl_cyc above): extra wait-CYCLES this group's own beats
        // spent blocked at each level — a true partition, always summing to
        // exactly delay_sum. Correct with multiple AGUs active at once,
        // unlike the global lvl_rd_l1/l2/l3 counters below (which are only
        // "this AGU's own" when it's the sole traffic source).
        uint64_t delay_l1_sum = 0, delay_l2_sum = 0, delay_l3_sum = 0;
        // M3 basis (cycle-inflation vs. a perfectly conflict-free execution
        // of this exact same traffic, wall-clock not per-lane).
        // arrival_cycles: distinct cycles in which >=1 lane's request is
        // genuinely NEW (wait_ctr[p]==0 the instant it's seen — the first
        // cycle of its req/gnt streak, whether granted same-cycle or forced
        // to wait), counted once per cycle regardless of how many lanes
        // arrive simultaneously. This is NOT lambda-based (an earlier
        // attempt bucketed real_beats by the task's configured
        // ports_used_/napa and divided by it — measures capacity-if-fully-
        // utilized, not this-traffic-if-uncontended, and gives a false 300%
        // "overhead" on the conflictfree fixture, which deliberately drives
        // only 1 of 4 lanes per group: napa says lambda=4, but the traffic
        // itself never asks for more than 1 beat/cycle, so lambda was never
        // the right denominator). active_cycles already excludes idle/NOP
        // time (see its own comment); in a genuinely conflict-free run every
        // arrival is granted its own cycle and nothing else stays pending,
        // so arrival_cycles == active_cycles exactly (M3=0%). Verified
        // directly: conflictfree (8 sequential, never-simultaneous arrivals)
        // gives arrival_cycles=8=active_cycles, M3=0%; fullconflict (4
        // simultaneous arrivals in 1 cycle, pigeonhole-serialized over 4)
        // gives arrival_cycles=1, active_cycles=4, M3=300%; l3conflict_2bank
        // (2 simultaneous, INDEPENDENT 1-cycle waits on different banks)
        // gives arrival_cycles=1, active_cycles=2, M3=100% for BOTH the
        // 1-bank and 2-bank case identically per bank pair — confirming
        // simultaneous-but-independent conflicts don't inflate M3 by lane
        // count the way delay_sum (M2) does.
        uint64_t arrival_cycles = 0;
    };
    grp_stat_t    gstat[9] = {{"ragu_a", true},  {"ragu_b", true},  {"ragu_c", true},
                              {"ragu_d", true},  {"ragu_e", true},  {"wagu_a", false},
                              {"wagu_b", false}, {"wagu_d", false}, {"wagu_e", false}};
    constexpr int kNRdFlat = dut_t::RAGU_A_PORTS + dut_t::RAGU_B_PORTS + dut_t::RAGU_C_PORTS +
                             dut_t::RAGU_D_PORTS + dut_t::RAGU_E_PORTS;
    constexpr int kNWrFlat =
        dut_t::WAGU_A_PORTS + dut_t::WAGU_B_PORTS + dut_t::WAGU_D_PORTS + dut_t::WAGU_E_PORTS;
    std::vector<int>             wait_ctr(kNRdFlat + kNWrFlat, 0);
    std::vector<uint8_t>         run_fill(kNRdFlat + kNWrFlat, 0);
    // Method-1 per-beat level attribution (crossbar builds only): bit 0/1/2
    // set if the beat currently accumulating in wait_ctr[p] has been
    // blocked at L1/L2/L3 on any cycle of its (still in progress) wait
    // streak. Same lifecycle as wait_ctr — cleared on grant or on an
    // abandoned (!rq) run — so it always reflects only the CURRENT streak.
    std::vector<uint8_t> touched_lvl(kNRdFlat + kNWrFlat, 0);
    // Method-2 per-beat, per-level CYCLE counts (crossbar builds only, same
    // lifecycle/indexing as touched_lvl): unlike tb_top.cpp's global
    // lvl_rd_l1/l2/l3 (summed across every active read port, so only valid
    // as "this AGU's own" when it's the only one with traffic), this is
    // indexed per flat port, so accumulating it into gs.delay_lN_sum below
    // stays correct even with multiple AGUs active simultaneously.
    // blocked_level[p] is exactly one level per cycle, so wait_lvl_cyc[0][p]
    // + [1][p] + [2][p] always sums to wait_ctr[p] exactly (no double count,
    // unlike touched_lvl's bitmask).
    std::vector<int> wait_lvl_cyc[3] = {
        std::vector<int>(kNRdFlat + kNWrFlat, 0),
        std::vector<int>(kNRdFlat + kNWrFlat, 0),
        std::vector<int>(kNRdFlat + kNWrFlat, 0),
    };
    bool                         prev_any_real[9] = {};
    std::vector<std::deque<int>> rd_gnt_q(kNRdFlat + kNWrFlat);
    uint64_t                     rsp_events = 0;
    int                          rsp_min = 1 << 30, rsp_max = -1;
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
    // Crossbar conflict level attribution. gnt chains back combinationally,
    // so a blocked request is visible as req&&!gnt at every boundary from
    // the input port down to the level whose arbitration it lost (beyond
    // that its req is not forwarded — the mux carries the winner). Per
    // cycle: inputs waiting W >= l1_l2 waiting A >= l2_l3 waiting B >=
    // l3_bank waiting C, so the per-level conflict-cycle counts are the
    // deltas: L1 = W-A, L2 = A-B, L3 = B-C, bank = C.
    uint64_t lvl_rd[4] = {0, 0, 0, 0}, lvl_wr[4] = {0, 0, 0, 0};
    // Distinct-cycle view: cycles in which >=1 request (rd or wr) was
    // blocked at that level — lvl_cyc[k]/actual is the fraction of runtime
    // that level was conflicting (bounded by 100%, unlike the summed
    // request-cycle counters above, which weight by how many requests were
    // blocked at once).
    uint64_t lvl_cyc[4] = {0, 0, 0, 0};

    // SEL_DESC_SYNC per-descriptor barrier state (see the barrier step at the
    // end of the main loop). desc_prev_lvl snapshots [l1_rd,l2_rd,l3_rd,
    // l1_wr,l2_wr,l3_wr] at the previous descriptor boundary.
    std::ofstream desc_csv;
    if (desc_sync) {
        desc_csv.open(out_dir + "/desc_sync.csv");
        if (desc_csv)
            desc_csv << "desc,boundary_cycle,drain,ideal,overhead,"
                        "l1_rd,l2_rd,l3_rd,l1_wr,l2_wr,l3_wr\n";
    }
    bool     desc_just_released = false;
    int      desc_idx           = 0;
    uint64_t desc_prev_cycle    = 0;
    uint64_t desc_prev_lvl[6]   = {0, 0, 0, 0, 0, 0};
    // Per-descriptor overhead accounting (1c): ideal = max over the 5 barriered
    // groups of the just-drained task's n_groups (conflict-free issue cycles);
    // overhead = drain - ideal. barrier_overhead collects the overhead of
    // zero-conflict descriptors (structural: last-beat response drain + any
    // cross-group load imbalance) so conflict_overhead = total - barrier is the
    // part actually attributable to counted L1/L2/L3 conflicts.
    uint64_t sum_ideal = 0, sum_drain = 0, sum_overhead = 0, barrier_overhead = 0;
    uint64_t agu_ideal[5] = {0, 0, 0, 0, 0};  // ragu_a,b,c, wagu_a,b : sum of n_groups
#endif
#if defined(IMPL_TDM) && !defined(IMPL_SV)
    // Shared-TDM-bus accounting: bus_busy = cycles the granted buffer had
    // pending work; bus_wasted = cycles some buffer wanted the bus but the
    // grant landed on one with nothing to send (the free-running RR's
    // idle-slot tax — structurally 0 under the adaptive arbiter);
    // bus_contention = cycles >=2 buffers wanted the bus at once (the true
    // sharing conflict: someone must wait even with perfect arbitration).
    uint64_t bus_busy = 0, bus_wasted = 0, bus_contention = 0;
    // Bus payload split: of the bank requests actually granted through the
    // shared bus (mux_tdm, the selected buffer's wires), how many carried a
    // real address vs addr-0 filler (window padding, napa rounding, flush)?
    // busy% counts a turn as "used" even when its beats are filler — this
    // pair says how much of the bus's bandwidth does real work.
    uint64_t bus_real_beats = 0, bus_nop_beats = 0;
    // Conflict RESOLUTION cost: cycles with >=1 blocked bank slot at the
    // mux->tdm boundary. Blocked losers all retry in parallel, so each such
    // cycle is exactly one extra serialization pass — 16 pairwise
    // collisions cost 1 cycle here, 32 hits on one bank cost 31 — which is
    // the number that maps to fetch latency (bank_stall, above it, counts
    // blocked slots summed and grows quadratically with pile-up depth).
    uint64_t bank_stall_cycles = 0;
#endif
    while (actual < kMaxCycles) {
        bool all = true;
        for (int a = 0; a < 9; ++a)
            all = all && done[a].read();
        if (all)
            break;

#if defined(IMPL_TDM) && !defined(IMPL_SV)
        // active_mode encoding: 0→1 group (4 beats), 1→2 groups (8 beats),
        // 2→4 groups (16 beats).
        auto tdm_mode = [](int ports_used, int num_req) -> uint32_t {
            const int g = (num_req > 0 && ports_used > 0) ? ports_used / num_req : 1;
            return (g <= 1) ? 0u : (g <= 2) ? 1u : 2u;
        };
        // DMA-style buffers (D/E) carry no CRL (p_has_crl_/lookahead
        // equivalents stay false) — fall back to safe defaults.
        constexpr uint64_t kDfltR = 4, kDfltC = 4, kDfltL = 8, kDfltSM = 0;

        // Advance each read AGU's lookahead cursor directly off the
        // matching buffer's OWN observed window_reset pulse (see
        // agu.hpp's advance_lookahead_window() comment on why this can't
        // be derived from the AGU's task_idx_/group_ instead) — checked
        // BEFORE this edge, so the cursor already reflects the next
        // window by the time it's written below for the buffer to latch
        // next edge.
        if (dut.impl.buf_r0.snapshot().window_reset)
            ragu_a_src->advance_lookahead_window();
        if (dut.impl.buf_r1.snapshot().window_reset)
            ragu_b_src->advance_lookahead_window();
        if (dut.impl.buf_r2.snapshot().window_reset)
            ragu_c_src->advance_lookahead_window();
        if (dut.impl.buf_r3.snapshot().window_reset)
            ragu_d_src->advance_lookahead_window();
        if (dut.impl.buf_r4.snapshot().window_reset)
            ragu_e_src->advance_lookahead_window();
        // Retried every cycle (not just on window_reset): fetch_addr_valid_i
        // being gated on lookahead_ready() means a buffer stuck behind a
        // fence never fetches, so it never produces another window_reset
        // to trigger the advance above — something has to notice the
        // fence clearing on a plain cycle tick instead. lane_agu (D/E) has
        // no such fence to retry.
        // Task-transition idle gates (see agu.hpp's la_task_roll_gate_open_
        // and flush_hold_): geometry-changing task boundaries must pass
        // through the buffer's atomic all-idle boot latch; same-geometry
        // boundaries stream seamlessly. Regression-locked by
        // tb_task_boundary.cpp.
        const auto between_tasks = [](const auto &src) {
            return !src->all_tasks_done() && src->group_ >= src->tasks_[src->task_idx_].n_groups;
        };
        const bool idle_a                   = dut.impl.buf_r0.snapshot().n_valid == 0;
        const bool idle_b                   = dut.impl.buf_r1.snapshot().n_valid == 0;
        const bool idle_c                   = dut.impl.buf_r2.snapshot().n_valid == 0;
        ragu_a_src->la_task_roll_gate_open_ = idle_a;
        ragu_b_src->la_task_roll_gate_open_ = idle_b;
        ragu_c_src->la_task_roll_gate_open_ = idle_c;
        ragu_a_src->flush_hold_             = between_tasks(ragu_a_src) && !idle_a;
        ragu_b_src->flush_hold_             = between_tasks(ragu_b_src) && !idle_b;
        ragu_c_src->flush_hold_             = between_tasks(ragu_c_src) && !idle_c;
        ragu_a_src->retry_lookahead_fence();
        ragu_b_src->retry_lookahead_fence();
        ragu_c_src->retry_lookahead_fence();

        // Mode/map config AFTER the cursor advances above, in the same
        // iteration — everything the buffer sees at the next edge
        // (fetch_addr_i, fetch_addr_valid_i, active_mode, TDM map) must
        // describe the SAME task (see agu.hpp's lookahead_ports_used()
        // comment for the desync a stale mode caused here previously).
        // A/B/C (agu<>) use their lookahead_*() accessors, which track
        // whichever window the buffer's hardware is actually processing;
        // D/E (lane_agu, fixed one 4-lane group, constant map geometry)
        // and all WAGU groups (writes never prefetch) use the plain
        // capture-side fields.
        dut.impl_buf_active_mode[0].write(
            tdm_mode(ragu_a_src->lookahead_ports_used(), dut_t::NUM_REQ));
        dut.impl_buf_map_r[0].write(ragu_a_src->lookahead_R());
        dut.impl_buf_map_c[0].write(ragu_a_src->lookahead_C());
        dut.impl_buf_map_l[0].write(ragu_a_src->lookahead_L());
        dut.impl_buf_map_store_mode[0].write(ragu_a_src->lookahead_store_mode());
        dut.impl_buf_active_mode[1].write(
            tdm_mode(ragu_b_src->lookahead_ports_used(), dut_t::NUM_REQ));
        dut.impl_buf_map_r[1].write(ragu_b_src->lookahead_R());
        dut.impl_buf_map_c[1].write(ragu_b_src->lookahead_C());
        dut.impl_buf_map_l[1].write(ragu_b_src->lookahead_L());
        dut.impl_buf_map_store_mode[1].write(ragu_b_src->lookahead_store_mode());
        dut.impl_buf_active_mode[2].write(
            tdm_mode(ragu_c_src->lookahead_ports_used(), dut_t::NUM_REQ));
        dut.impl_buf_map_r[2].write(ragu_c_src->lookahead_R());
        dut.impl_buf_map_c[2].write(ragu_c_src->lookahead_C());
        dut.impl_buf_map_l[2].write(ragu_c_src->lookahead_L());
        dut.impl_buf_map_store_mode[2].write(ragu_c_src->lookahead_store_mode());

        auto get_r  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_R_ : d; };
        auto get_c  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_C_ : d; };
        auto get_l  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_L_ : d; };
        auto get_sm = [](const auto &s, uint64_t d) {
            return s->p_has_crl_ ? s->p_store_mode_ : d;
        };
        auto write_map = [&](int idx, const auto &s) {
            dut.impl_buf_active_mode[idx].write(tdm_mode(s->ports_used_, dut_t::NUM_REQ));
            dut.impl_buf_map_r[idx].write(get_r(s, kDfltR));
            dut.impl_buf_map_c[idx].write(get_c(s, kDfltC));
            dut.impl_buf_map_l[idx].write(get_l(s, kDfltL));
            dut.impl_buf_map_store_mode[idx].write(get_sm(s, kDfltSM));
        };
        write_map(3, ragu_d_src);
        write_map(4, ragu_e_src);
        write_map(5, wagu_a_src);
        write_map(6, wagu_b_src);
        write_map(7, wagu_d_src);
        write_map(8, wagu_e_src);

        rd0_lookahead_valid.write(ragu_a_src->lookahead_ready());
        rd1_lookahead_valid.write(ragu_b_src->lookahead_ready());
        rd2_lookahead_valid.write(ragu_c_src->lookahead_ready());
        rd3_lookahead_valid.write(true); // lane_agu: always well-defined content

        for (int w = 0; w < N_BANK; ++w) {
            rd0_lookahead[w].write(ragu_a_src->lookahead_addr(w));
            rd1_lookahead[w].write(ragu_b_src->lookahead_addr(w));
            rd2_lookahead[w].write(ragu_c_src->lookahead_addr(w));
            rd3_lookahead[w].write(ragu_d_src->lookahead_addr(w));
            rd4_lookahead[w].write(ragu_e_src->lookahead_addr(w));
        }
#endif

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) &&                                                \
    (defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16) || defined(XBAR_HASH32) ||                \
     defined(XBAR_HASH_L1_V2) || defined(XBAR_HASH_L1_V3))
        // top_crossbar.hpp's dynamic-hash experiment: broadcast each AGU's
        // current task R/C/L/store_mode to every crossbar port-group index
        // that AGU drives. Crossbar mode has no prefetch buffer (unlike the
        // TDM wiring above), so the plain per-task fields suffice — no
        // lookahead accessors needed. Defaults mirror the TDM write_map
        // lambda above for groups with no CRL geometry (D/E lane_agu).
        {
            constexpr uint64_t kDfltR = 4, kDfltC = 4, kDfltL = 8, kDfltSM = 0;
            auto               get_r  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_R_ : d; };
            auto               get_c  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_C_ : d; };
            auto               get_l  = [](const auto &s, uint64_t d) { return s->p_has_crl_ ? s->p_L_ : d; };
            auto               get_sm = [](const auto &s, uint64_t d) {
                return s->p_has_crl_ ? s->p_store_mode_ : d;
            };
            auto write_rmap = [&](int base, int count, const auto &s, bool hi_bank) {
                const uint64_t r = get_r(s, kDfltR), c = get_c(s, kDfltC), l = get_l(s, kDfltL),
                               sm = get_sm(s, kDfltSM);
                for (int j = base; j < base + count; ++j) {
                    dut.impl_rport_map_r[j].write(r);
                    dut.impl_rport_map_c[j].write(c);
                    dut.impl_rport_map_l[j].write(l);
                    dut.impl_rport_map_store_mode[j].write(sm);
#if defined(XBAR_HASH_L1_V2)
                    dut.impl_rport_map_napa[j].write(s->ports_used_);
#endif
#if defined(XBAR_HASH16)
                    dut.impl_rport_map_hi_bank[j].write(hi_bank);
#endif
                }
            };
            auto write_wmap = [&](int base, int count, const auto &s, bool hi_bank) {
                const uint64_t r = get_r(s, kDfltR), c = get_c(s, kDfltC), l = get_l(s, kDfltL),
                               sm = get_sm(s, kDfltSM);
                for (int j = base; j < base + count; ++j) {
                    dut.impl_wport_map_r[j].write(r);
                    dut.impl_wport_map_c[j].write(c);
                    dut.impl_wport_map_l[j].write(l);
                    dut.impl_wport_map_store_mode[j].write(sm);
#if defined(XBAR_HASH_L1_V2)
                    dut.impl_wport_map_napa[j].write(s->ports_used_);
#endif
#if defined(XBAR_HASH16)
                    dut.impl_wport_map_hi_bank[j].write(hi_bank);
#endif
                }
            };
            // Static per-AGU bank-half assignment for XBAR_HASH16 (see
            // top_crossbar.hpp's rport_map_hi_bank_i comment): ragu_a is the
            // heaviest single contributor to residual conflict (Table
            // E11/E15), so it gets the low half (banks 0-15) to itself;
            // every other AGU shares the high half (banks 16-31). This is a
            // fixed assignment, not derived from live traffic.
            write_rmap(0, dut_t::NUM_RAGU_A, ragu_a_src, false);
            write_rmap(4, dut_t::NUM_RAGU_B, ragu_b_src, true);
            write_rmap(6, dut_t::NUM_RAGU_C, ragu_c_src, true);
            write_rmap(7, dut_t::NUM_RAGU_D, ragu_d_src, true);
            write_rmap(8, dut_t::NUM_RAGU_E, ragu_e_src, true);
            write_wmap(0, dut_t::NUM_WAGU_A, wagu_a_src, true);
            write_wmap(4, dut_t::NUM_WAGU_B, wagu_b_src, true);
            write_wmap(6, dut_t::NUM_WAGU_D, wagu_d_src, true);
            write_wmap(7, dut_t::NUM_WAGU_E, wagu_e_src, true);
        }
#endif

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) && defined(XBAR_ROB)
        {
            // ROB fetch driving, one block per agu<>-driven read AGU. Per
            // cycle: (1) if last cycle's ingest ack fired, the whole group
            // was consumed (all sibling ports ingest on the same edge —
            // valid is only raised when every sibling is ready), so advance
            // the group-granular lookahead cursor; (2) retry a fence-parked
            // task rollover; (3) re-drive the fetch buses and lookahead
            // geometry for whatever group the cursor now points at. Lanes
            // beyond the lookahead task's rounded width belong to the NEXT
            // group in lookahead_addr()'s flat indexing, so they are driven
            // as 0 (NOP holes) rather than leaking the next group early.
            auto drive_rob_fetch = [&](auto &src, int base_port, int n_ports) {
                if (rob_fetch_ack[base_port].read())
                    src->advance_lookahead_group_rob();
                src->retry_lookahead_fence_rob();
                bool ready_all = src->lookahead_ready();
                for (int jp = 0; jp < n_ports; ++jp)
                    ready_all = ready_all && rob_fetch_ready[base_port + jp].read();
                const int rw =
                    std::remove_reference_t<decltype(*src)>::rounded_width(
                        src->lookahead_ports_used());
                for (int jp = 0; jp < n_ports; ++jp) {
                    const int j = base_port + jp;
                    rob_fetch_valid[j].write(ready_all);
                    rob_la_r[j].write(src->lookahead_R());
                    rob_la_c[j].write(src->lookahead_C());
                    rob_la_l[j].write(src->lookahead_L());
                    rob_la_sm[j].write(src->lookahead_store_mode());
                    rob_la_napa[j].write(src->lookahead_ports_used());
                    for (int m = 0; m < dut_t::NUM_REQ; ++m) {
                        const int wla = jp * dut_t::NUM_REQ + m;
                        rob_fetch_addr[j * dut_t::NUM_REQ + m].write(
                            (ready_all && wla < rw) ? src->lookahead_addr(wla) : 0);
                    }
                }
            };
            drive_rob_fetch(ragu_a_src, 0, dut_t::NUM_RAGU_A);
            drive_rob_fetch(ragu_b_src, 4, dut_t::NUM_RAGU_B);
            drive_rob_fetch(ragu_c_src, 6, dut_t::NUM_RAGU_C);
        }
#endif

        sc_start(CLK_PERIOD_NS, SC_NS);
        ++actual;

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
        // Method-1 per-lane level attribution: walk the crossbar's own
        // winner registers (crossbar.hpp's win_[], public by convention) to
        // learn, per flat lane (same indexing as dut.impl.rport_req_i/
        // wport_req_i), whether it won L1 and L2 this cycle. Unlike the
        // aggregate a/b/c counts further below (anonymous totals at each
        // boundary — how MANY lanes are blocked where, not WHICH ones),
        // this preserves per-lane identity so tally_group() below can
        // attribute a delayed BEAT to the specific level(s) that blocked
        // it. A lane that's req&&!gnt and won L1 and L2 must be blocked at
        // L3 by elimination (gnt_o is the AND of all three), so L3 needs no
        // separate winner walk. Values: 0=blocked at L1, 1=at L2, 2=at L3,
        // 3=not blocked (irrelevant/unused for such lanes).
        std::vector<uint8_t> blocked_level(kNRdFlat + kNWrFlat, 3);
        {
            using impl_t = decltype(dut.impl);
            // --- read side ---
            {
                std::vector<uint8_t> won_l1(kNRdFlat, 0), won_l2(kNRdFlat, 0);
                for (int j = 0; j < dut_t::NUM_RPORT; ++j)
                    for (int m = 0; m < dut_t::NUM_REQ; ++m) {
                        const int w = dut.impl.l1_rd_[j].win_[m].read();
                        if (w >= 0)
                            won_l1[j * dut_t::NUM_REQ + w] = 1;
                    }
                for (int k = 0; k < dut_t::NUM_REQ; ++k)
                    for (int g = 0; g < impl_t::NUM_BANK_GRP; ++g) {
                        const int jw = dut.impl.l2_rd_[k].win_[g].read();
                        if (jw < 0)
                            continue;
                        const int orig = dut.impl.l1_rd_[jw].win_[k].read();
                        if (orig >= 0)
                            won_l2[jw * dut_t::NUM_REQ + orig] = 1;
                    }
                for (int p = 0; p < kNRdFlat; ++p) {
                    if (!dut.impl.rport_req_i[p].read() || dut.impl.rport_gnt_o[p].read())
                        continue; // not requesting, or fully granted -> not blocked
                    blocked_level[p] = !won_l1[p] ? 0 : !won_l2[p] ? 1 : 2;
                }
            }
            // --- write side ---
            {
                std::vector<uint8_t> won_l1(kNWrFlat, 0), won_l2(kNWrFlat, 0);
                for (int j = 0; j < dut_t::NUM_WPORT; ++j)
                    for (int m = 0; m < dut_t::NUM_REQ; ++m) {
                        const int w = dut.impl.l1_wr_[j].win_[m].read();
                        if (w >= 0)
                            won_l1[j * dut_t::NUM_REQ + w] = 1;
                    }
                for (int k = 0; k < dut_t::NUM_REQ; ++k)
                    for (int g = 0; g < impl_t::NUM_BANK_GRP; ++g) {
                        const int jw = dut.impl.l2_wr_[k].win_[g].read();
                        if (jw < 0)
                            continue;
                        const int orig = dut.impl.l1_wr_[jw].win_[k].read();
                        if (orig >= 0)
                            won_l2[jw * dut_t::NUM_REQ + orig] = 1;
                    }
                for (int p = 0; p < kNWrFlat; ++p) {
                    if (!dut.impl.wport_req_i[p].read() || dut.impl.wport_gnt_o[p].read())
                        continue;
                    blocked_level[kNRdFlat + p] = !won_l1[p] ? 0 : !won_l2[p] ? 1 : 2;
                }
            }
        }
#endif
        // SEL_VCD: pack each group's current descriptor label (ASCII) into its
        // `desc` signal (only emits a VCD change at descriptor boundaries).
        if (sel_vcd) {
            auto upd = [&](int g, const auto &src) {
                const std::size_t i = src->task_idx_;
                desc_sig[g].write(
                    pack_ascii(i < desc_labels[g].size() ? desc_labels[g][i] : std::string("done")));
            };
            upd(0, ragu_a_src);
            upd(1, ragu_b_src);
            upd(2, ragu_c_src);
            upd(3, wagu_a_src);
            upd(4, wagu_b_src);
        }

        // --- sample this cycle's settled req/gnt state into the counters ---
        int  cyc_wait_rd = 0, cyc_wait_wr = 0; // this cycle's waiting input ports
        int  cyc_stall_lanes = 0;              // real lanes blocked by contention (fill excluded)
        auto tally_group     = [&](const obi_signal_bundle<data_t> *g, int n, int base, int gi) {
            grp_stat_t &gs       = gstat[gi];
            bool        any_real = false, any_arrival = false;
            bool        ep = false, ep_stall = false, any_wait = false, any_serve = false;
            bool        any_stall_wait = false;
            for (int i = 0; i < n; ++i) {
                const int  p    = base + i;
                const bool rq   = g[i].req.read();
                const bool gt   = g[i].gnt.read();
                const bool real = rq && g[i].addr.read() != 0;
                any_real        = any_real || real;
                // Fresh arrival: wait_ctr[p] still holds last cycle's final
                // value here (unmodified so far this iteration) — ==0 means
                // this lane wasn't mid-wait, so if it's real this is the
                // first cycle of a new req/gnt streak, granted immediately
                // or not. See grp_stat_t::arrival_cycles for why this (not a
                // lambda/napa-based capacity guess) is the M3 basis.
                any_arrival     = any_arrival || (real && wait_ctr[p] == 0);
                if (!rq) {
                    wait_ctr[p] = 0; // abandoned run — a beat's delay is only
                                     // the wait immediately preceding its grant
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
                    touched_lvl[p] = 0;
                    wait_lvl_cyc[0][p] = wait_lvl_cyc[1][p] = wait_lvl_cyc[2][p] = 0;
#endif
                } else if (!gt) {
                    ++port_wait;
                    (gs.is_rd ? cyc_wait_rd : cyc_wait_wr) += 1;
                    if (real) {
                        if (wait_ctr[p] == 0) // run starts: from group-idle -> fill
                            run_fill[p] = !prev_any_real[gi];
                        ++wait_ctr[p];
                        any_wait = true;
                        if (!run_fill[p]) {
                            any_stall_wait = true;
                            ++cyc_stall_lanes;
                        }
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
                        touched_lvl[p] |= static_cast<uint8_t>(1u << blocked_level[p]);
                        ++wait_lvl_cyc[blocked_level[p]][p];
#endif
                    } else {
                        wait_ctr[p] = 0; // NOP/flush wait: idle time, not a conflict
                    }
                } else {
                    ++port_serve;
                    if (real) {
                        ++gs.real_beats;
                        any_serve = true;
                        if (wait_ctr[p] > 0) {
                            ++gs.delayed;
                            gs.delay_sum += wait_ctr[p];
                            gs.delay_max = std::max<uint64_t>(gs.delay_max, wait_ctr[p]);
                            if (run_fill[p]) {
                                ++gs.fill_delayed;
                                gs.fill_delay_sum += wait_ctr[p];
                            } else {
                                ep_stall = true;
                            }
                            ep = true;
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
                            // Method-1 per-level: this beat touched a level
                            // if any of its wait cycles were blocked there
                            // (see touched_lvl's own comment) — a beat can
                            // set more than one bit across its streak.
                            if (touched_lvl[p] & 1u)
                                ++gs.delayed_l1;
                            if (touched_lvl[p] & 2u)
                                ++gs.delayed_l2;
                            if (touched_lvl[p] & 4u)
                                ++gs.delayed_l3;
                            // Method-2 per-level: a true partition of this
                            // beat's wait_ctr[p] cycles (see wait_lvl_cyc's
                            // own comment) — always sums to gs.delay_sum.
                            gs.delay_l1_sum += wait_lvl_cyc[0][p];
                            gs.delay_l2_sum += wait_lvl_cyc[1][p];
                            gs.delay_l3_sum += wait_lvl_cyc[2][p];
#endif
                        }
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
                        touched_lvl[p] = 0;
                        wait_lvl_cyc[0][p] = wait_lvl_cyc[1][p] = wait_lvl_cyc[2][p] = 0;
#endif
                    } else {
                        ++gs.nop_beats;
                    }
                    wait_ctr[p] = 0;
                    if (gs.is_rd)
                        rd_gnt_q[p].push_back(actual);
                }
                if (gs.is_rd && g[i].rvalid.read() && !rd_gnt_q[p].empty()) {
                    const int d = actual - rd_gnt_q[p].front();
                    rd_gnt_q[p].pop_front();
                    ++rsp_events;
                    rsp_min = std::min(rsp_min, d);
                    rsp_max = std::max(rsp_max, d);
                }
            }
            if (any_real)
                ++gs.active_cycles;
            if (any_arrival)
                ++gs.arrival_cycles;
            gs.wait_cycles += any_wait;
            gs.fill_wait_cycles += any_wait && !any_stall_wait;
            gs.serve_cycles += any_serve;
            gs.episodes += ep;
            gs.stall_episodes += ep_stall;
            prev_any_real[gi] = any_real;
        };
        {
            int base = 0;
            tally_group(ragu_a, dut_t::RAGU_A_PORTS, base, 0);
            base += dut_t::RAGU_A_PORTS;
            tally_group(ragu_b, dut_t::RAGU_B_PORTS, base, 1);
            base += dut_t::RAGU_B_PORTS;
            tally_group(ragu_c, dut_t::RAGU_C_PORTS, base, 2);
            base += dut_t::RAGU_C_PORTS;
            tally_group(ragu_d, dut_t::RAGU_D_PORTS, base, 3);
            base += dut_t::RAGU_D_PORTS;
            tally_group(ragu_e, dut_t::RAGU_E_PORTS, base, 4);
            base += dut_t::RAGU_E_PORTS;
            tally_group(wagu_a, dut_t::WAGU_A_PORTS, base, 5);
            base += dut_t::WAGU_A_PORTS;
            tally_group(wagu_b, dut_t::WAGU_B_PORTS, base, 6);
            base += dut_t::WAGU_B_PORTS;
            tally_group(wagu_d, dut_t::WAGU_D_PORTS, base, 7);
            base += dut_t::WAGU_D_PORTS;
            tally_group(wagu_e, dut_t::WAGU_E_PORTS, base, 8);
        }
#if defined(IMPL_TDM) && !defined(IMPL_SV)
        {
            int served = 0;
            for (int b = 0; b < N_BANK; ++b) {
                if (dut.impl.xbar_bank[b].req.read() && dut.impl.xbar_bank[b].gnt.read())
                    ++served;
            }
            bank_busy += served;
            if (bank_trace.is_open())
                bank_trace << served << ',' << cyc_stall_lanes << "\n";
        }
        // TDM bank conflicts never reach xbar_bank (the losing beat is held
        // upstream): two window slots XOR-mapping to the same bank in the
        // same cycle stall at the mux->tdm bundles as req&&!gnt. Granted
        // beats split into real payload vs addr-0 filler.
        {
            bool any_blocked = false;
            for (int w = 0; w < N_BANK; ++w) {
                const bool rq = dut.impl.mux_tdm[w].req.read();
                const bool gt = dut.impl.mux_tdm[w].gnt.read();
                if (rq && !gt) {
                    ++bank_stall;
                    any_blocked = true;
                } else if (rq && gt) {
                    (dut.impl.mux_tdm[w].addr.read() != 0 ? bus_real_beats : bus_nop_beats) += 1;
                }
            }
            bank_stall_cycles += any_blocked;
        }
        if (map_conflict_log.is_open()) {
            for (int w = 0; w < N_BANK; ++w) {
                if (dut.impl.mapf.conflict_o[w].read())
                    map_conflict_log << actual << ',' << w << ',' << dut.impl.mapf.bank_id_o[w].read()
                                      << ',' << dut.impl.mapf.row_id_o[w].read() << "\n";
            }
        }
        {
            using impl_t       = decltype(dut.impl);
            const int sel      = dut.impl.arb_req_sel.read();
            int       n_active = 0;
            bool      sel_has  = false;
            for (int c = 0; c < impl_t::NUM_TOTAL_BUF; ++c) {
                bool any = false;
                for (int w = 0; w < N_BANK && !any; ++w)
                    any = dut.impl.buf_tdm[c * N_BANK + w].req.read();
                if (any) {
                    ++n_active;
                    if (c == sel)
                        sel_has = true;
                }
            }
            if (sel_has)
                ++bus_busy;
            else if (n_active > 0)
                ++bus_wasted;
            if (n_active > 1)
                ++bus_contention;
        }
#elif defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
        {
            using impl_t = decltype(dut.impl);
            // Per-port L1 collision detection: for each physical port's own
            // 4-lane L1 instance, more real requesters at the input than
            // distinct winners forwarded to l1_l2_{rd,wr} means >=2 lanes
            // hashed to the same L1 output this cycle (see l1_conflict_rd/wr
            // declaration above for why).
            auto l1_conflict_check = [&](const obi_signal_bundle<data_t> *in, int n_lanes,
                                         const obi_signal_bundle<data_t> *l1out, int sig_base,
                                         sc_signal<bool> *sig) {
                for (int p = 0; p * 4 < n_lanes; ++p) {
                    int in_req = 0, out_req = 0;
                    for (int k = 0; k < 4; ++k) {
                        if (in[p * 4 + k].req.read())
                            ++in_req;
                        if (l1out[p * 4 + k].req.read())
                            ++out_req;
                    }
                    sig[sig_base + p].write(in_req > out_req);
                }
            };
            {
                int rd_base = 0, sig_base = 0;
                l1_conflict_check(ragu_a, dut_t::RAGU_A_PORTS, dut.impl.l1_l2_rd + rd_base, sig_base,
                                  l1_conflict_rd);
                rd_base += dut_t::RAGU_A_PORTS;
                sig_base += dut_t::RAGU_A_PORTS / 4;
                l1_conflict_check(ragu_b, dut_t::RAGU_B_PORTS, dut.impl.l1_l2_rd + rd_base, sig_base,
                                  l1_conflict_rd);
                rd_base += dut_t::RAGU_B_PORTS;
                sig_base += dut_t::RAGU_B_PORTS / 4;
                l1_conflict_check(ragu_c, dut_t::RAGU_C_PORTS, dut.impl.l1_l2_rd + rd_base, sig_base,
                                  l1_conflict_rd);
                rd_base += dut_t::RAGU_C_PORTS;
                sig_base += dut_t::RAGU_C_PORTS / 4;
                l1_conflict_check(ragu_d, dut_t::RAGU_D_PORTS, dut.impl.l1_l2_rd + rd_base, sig_base,
                                  l1_conflict_rd);
                rd_base += dut_t::RAGU_D_PORTS;
                sig_base += dut_t::RAGU_D_PORTS / 4;
                l1_conflict_check(ragu_e, dut_t::RAGU_E_PORTS, dut.impl.l1_l2_rd + rd_base, sig_base,
                                  l1_conflict_rd);

                int wr_base = 0;
                sig_base    = 0;
                l1_conflict_check(wagu_a, dut_t::WAGU_A_PORTS, dut.impl.l1_l2_wr + wr_base, sig_base,
                                  l1_conflict_wr);
                wr_base += dut_t::WAGU_A_PORTS;
                sig_base += dut_t::WAGU_A_PORTS / 4;
                l1_conflict_check(wagu_b, dut_t::WAGU_B_PORTS, dut.impl.l1_l2_wr + wr_base, sig_base,
                                  l1_conflict_wr);
                wr_base += dut_t::WAGU_B_PORTS;
                sig_base += dut_t::WAGU_B_PORTS / 4;
                l1_conflict_check(wagu_d, dut_t::WAGU_D_PORTS, dut.impl.l1_l2_wr + wr_base, sig_base,
                                  l1_conflict_wr);
                wr_base += dut_t::WAGU_D_PORTS;
                sig_base += dut_t::WAGU_D_PORTS / 4;
                l1_conflict_check(wagu_e, dut_t::WAGU_E_PORTS, dut.impl.l1_l2_wr + wr_base, sig_base,
                                  l1_conflict_wr);
            }
            int a_rd = 0, a_wr = 0, b_rd = 0, b_wr = 0, c_rd = 0, c_wr = 0;
            for (int i = 0; i < impl_t::NUM_L1_L2_RD; ++i)
                if (dut.impl.l1_l2_rd[i].req.read() && !dut.impl.l1_l2_rd[i].gnt.read())
                    ++a_rd;
            for (int i = 0; i < impl_t::NUM_L1_L2_WR; ++i)
                if (dut.impl.l1_l2_wr[i].req.read() && !dut.impl.l1_l2_wr[i].gnt.read())
                    ++a_wr;
            for (int i = 0; i < impl_t::NUM_L2_L3; ++i) {
                if (dut.impl.l2_l3_rd[i].req.read() && !dut.impl.l2_l3_rd[i].gnt.read())
                    ++b_rd;
                if (dut.impl.l2_l3_wr[i].req.read() && !dut.impl.l2_l3_wr[i].gnt.read())
                    ++b_wr;
            }
            int served = 0;
            for (int b = 0; b < impl_t::NUM_PHYS_BANKS; ++b) {
                if (!dut.impl.l3_bank[b].req.read())
                    continue;
                if (dut.impl.l3_bank[b].gnt.read()) {
                    ++served;
                } else {
                    ++bank_stall;
                    (dut.impl.l3_bank[b].we.read() ? c_wr : c_rd) += 1;
                }
            }
            bank_busy += served;
            if (bank_trace.is_open())
                bank_trace << served << ',' << cyc_stall_lanes << "\n";
            const int d[4] = {std::max(0, cyc_wait_rd - a_rd) + std::max(0, cyc_wait_wr - a_wr),
                              std::max(0, a_rd - b_rd) + std::max(0, a_wr - b_wr),
                              std::max(0, b_rd - c_rd) + std::max(0, b_wr - c_wr), c_rd + c_wr};
            lvl_rd[0] += std::max(0, cyc_wait_rd - a_rd);
            lvl_rd[1] += std::max(0, a_rd - b_rd);
            lvl_rd[2] += std::max(0, b_rd - c_rd);
            lvl_rd[3] += c_rd;
            lvl_wr[0] += std::max(0, cyc_wait_wr - a_wr);
            lvl_wr[1] += std::max(0, a_wr - b_wr);
            lvl_wr[2] += std::max(0, b_wr - c_wr);
            lvl_wr[3] += c_wr;
            for (int k = 0; k < 4; ++k)
                lvl_cyc[k] += d[k] > 0;
            if (sel_vcd) {
                wait_cyc_rd.write(cyc_wait_rd);
                wait_cyc_wr.write(cyc_wait_wr);
                wait_lvl_l1.write(d[0]);
                wait_lvl_l2.write(d[1]);
                wait_lvl_l3.write(d[2]);
                for (int k = 0; k < 3; ++k) {
                    lvl_rd_sig[k].write(static_cast<unsigned>(lvl_rd[k]));
                    lvl_wr_sig[k].write(static_cast<unsigned>(lvl_wr[k]));
                }
            }
        }
#endif
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
        // --- SEL_DESC_SYNC per-descriptor barrier: the 5 agu groups hold at
        //     each descriptor boundary; when all are drained, log this
        //     descriptor's drain cycles + conflict-level split, then release
        //     all together for exactly one advance (one task each).
        if (desc_sync) {
            auto bnd  = [](const auto &s) { return s->all_tasks_done() || s->at_task_boundary(); };
            auto hold = [](auto &s, bool v) { s->desc_barrier_hold_ = v; };
            if (desc_just_released) {
                // cycle after release: the 5 groups advanced one task each — re-hold.
                hold(ragu_a_src, true); hold(ragu_b_src, true); hold(ragu_c_src, true);
                hold(wagu_a_src, true); hold(wagu_b_src, true);
                desc_just_released = false;
            } else {
                const bool all_bnd = bnd(ragu_a_src) && bnd(ragu_b_src) && bnd(ragu_c_src) &&
                                     bnd(wagu_a_src) && bnd(wagu_b_src);
                const bool all_done = ragu_a_src->all_tasks_done() && ragu_b_src->all_tasks_done() &&
                                      ragu_c_src->all_tasks_done() && wagu_a_src->all_tasks_done() &&
                                      wagu_b_src->all_tasks_done();
                if (all_bnd && !all_done) {
                    const uint64_t drain = actual - desc_prev_cycle;
                    auto ng = [](const auto &s) {
                        return s->all_tasks_done() ? std::size_t(0) : s->cur_task().n_groups;
                    };
                    const uint64_t ideal = std::max({ng(ragu_a_src), ng(ragu_b_src), ng(ragu_c_src),
                                                     ng(wagu_a_src), ng(wagu_b_src)});
                    const int64_t  ovh = static_cast<int64_t>(drain) - static_cast<int64_t>(ideal);
                    const uint64_t cl1 = (lvl_rd[0] - desc_prev_lvl[0]) + (lvl_wr[0] - desc_prev_lvl[3]);
                    const uint64_t cl2 = (lvl_rd[1] - desc_prev_lvl[1]) + (lvl_wr[1] - desc_prev_lvl[4]);
                    const uint64_t cl3 = (lvl_rd[2] - desc_prev_lvl[2]) + (lvl_wr[2] - desc_prev_lvl[5]);
                    if (desc_csv)
                        desc_csv << desc_idx << ',' << actual << ',' << drain << ',' << ideal << ','
                                 << ovh << ',' << (lvl_rd[0] - desc_prev_lvl[0]) << ','
                                 << (lvl_rd[1] - desc_prev_lvl[1]) << ','
                                 << (lvl_rd[2] - desc_prev_lvl[2]) << ','
                                 << (lvl_wr[0] - desc_prev_lvl[3]) << ','
                                 << (lvl_wr[1] - desc_prev_lvl[4]) << ','
                                 << (lvl_wr[2] - desc_prev_lvl[5]) << '\n';
                    const uint64_t ovh_pos = ovh > 0 ? static_cast<uint64_t>(ovh) : 0;
                    sum_drain += drain;
                    sum_ideal += ideal;
                    sum_overhead += ovh_pos;
                    if (cl1 + cl2 + cl3 == 0)
                        barrier_overhead += ovh_pos;
                    agu_ideal[0] += ng(ragu_a_src); agu_ideal[1] += ng(ragu_b_src);
                    agu_ideal[2] += ng(ragu_c_src); agu_ideal[3] += ng(wagu_a_src);
                    agu_ideal[4] += ng(wagu_b_src);
                    ++desc_idx;
                    desc_prev_cycle  = actual;
                    desc_prev_lvl[0] = lvl_rd[0]; desc_prev_lvl[1] = lvl_rd[1];
                    desc_prev_lvl[2] = lvl_rd[2]; desc_prev_lvl[3] = lvl_wr[0];
                    desc_prev_lvl[4] = lvl_wr[1]; desc_prev_lvl[5] = lvl_wr[2];
                    hold(ragu_a_src, false); hold(ragu_b_src, false); hold(ragu_c_src, false);
                    hold(wagu_a_src, false); hold(wagu_b_src, false);
                    desc_just_released = true;
                }
            }
        }
#endif
    }

    if (actual >= kMaxCycles)
        fprintf(stderr, "WARNING: simulation timed out after %d cycles\n", kMaxCycles);

    // ideal = pipeline fill + the slowest AGU (structural-conflict-free lower bound)
    const std::size_t max_groups = std::max({
        ragu_a_src->n_groups_,
        ragu_b_src->n_groups_,
        ragu_c_src->n_groups_,
        ragu_d_src->n_groups_,
        ragu_e_src->n_groups_,
        wagu_a_src->n_groups_,
        wagu_b_src->n_groups_,
        wagu_d_src->n_groups_,
        wagu_e_src->n_groups_,
    });
    const int         ideal      = kPipeFill + static_cast<int>(max_groups);
    const double      overhead   = ideal > 0 ? 100.0 * (actual - ideal) / ideal : 0.0;

    printf("\n=========== top statistics (%s) ===========\n",
#if defined(IMPL_TDM)
#if defined(IMPL_SV)
           "top_tdm"
#else
           "tdm"
#endif
#elif defined(IMPL_CROSSBAR)
#if defined(IMPL_SV)
           "top_crossbar"
#else
           "crossbar"
#endif
#endif
    );
    printf(" hw ports   : RAGU_A=%d RAGU_B=%d RAGU_C=%d RAGU_D=%d RAGU_E=%d\n", dut_t::NUM_RAGU_A,
           dut_t::NUM_RAGU_B, dut_t::NUM_RAGU_C, dut_t::NUM_RAGU_D, dut_t::NUM_RAGU_E);
    printf("              WAGU_A=%d WAGU_B=%d WAGU_D=%d WAGU_E=%d\n", dut_t::NUM_WAGU_A,
           dut_t::NUM_WAGU_B, dut_t::NUM_WAGU_D, dut_t::NUM_WAGU_E);
    printf(" stim groups: RAGU_A=%zu RAGU_B=%zu RAGU_C=%zu RAGU_D=%zu RAGU_E=%zu\n",
           ragu_a_src->n_groups_, ragu_b_src->n_groups_, ragu_c_src->n_groups_,
           ragu_d_src->n_groups_, ragu_e_src->n_groups_);
    printf("              WAGU_A=%zu WAGU_B=%zu WAGU_D=%zu WAGU_E=%zu\n", wagu_a_src->n_groups_,
           wagu_b_src->n_groups_, wagu_d_src->n_groups_, wagu_e_src->n_groups_);
    printf(" actual     : %d cycles\n", actual);
    printf(" ideal      : %d cycles (pipeline=%d + max_groups=%zu)\n", ideal, kPipeFill,
           max_groups);
    printf(" overhead   : +%.1f%%\n", overhead);
#if defined(IMPL_TDM) && !defined(IMPL_SV)
    constexpr int kNBanks = N_BANK;
#elif defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
    constexpr int kNBanks = decltype(dut.impl)::NUM_PHYS_BANKS;
#else
    constexpr int kNBanks = 0;
#endif
    if (kNBanks > 0 && actual > 0) {
        printf(" port serve : %llu beat-grants, wait %llu req-cycles (%.1f%% of req-cycles "
               "waiting)\n",
               (unsigned long long)port_serve, (unsigned long long)port_wait,
               port_serve + port_wait ? 100.0 * port_wait / (port_serve + port_wait) : 0.0);
        printf(" bank util  : %.1f%% of %d banks busy; %llu stalled bank-cycles\n",
               100.0 * bank_busy / ((double)actual * kNBanks), kNBanks,
               (unsigned long long)bank_stall);
        printf(" conflicts per AGU (real beats only; NOP/flush/fence waits excluded):\n");
        for (const auto &gs : gstat) {
            if (gs.real_beats == 0 && gs.nop_beats == 0)
                continue;
            printf("   %-7s : %llu conflicts / %llu real beats (%.1f%%, avg +%.2f, max +%llu); "
                   "%llu NOP beats; active %.1f%% of cycles\n",
                   gs.name, (unsigned long long)gs.delayed, (unsigned long long)gs.real_beats,
                   gs.real_beats ? 100.0 * gs.delayed / gs.real_beats : 0.0,
                   gs.delayed ? (double)gs.delay_sum / gs.delayed : 0.0,
                   (unsigned long long)gs.delay_max, (unsigned long long)gs.nop_beats,
                   100.0 * gs.active_cycles / actual);
        }
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
        printf(" conflict lvl: rd L1/L2/L3/bank = %llu/%llu/%llu/%llu wait-cycles; "
               "wr = %llu/%llu/%llu/%llu\n",
               (unsigned long long)lvl_rd[0], (unsigned long long)lvl_rd[1],
               (unsigned long long)lvl_rd[2], (unsigned long long)lvl_rd[3],
               (unsigned long long)lvl_wr[0], (unsigned long long)lvl_wr[1],
               (unsigned long long)lvl_wr[2], (unsigned long long)lvl_wr[3]);
#endif
        if (rsp_events)
            printf(" rd rsp lat : gnt->rvalid %d..%d cycles over %llu responses\n", rsp_min,
                   rsp_max, (unsigned long long)rsp_events);
#if defined(IMPL_TDM) && !defined(IMPL_SV)
        printf(" tdm bus    : busy %.1f%% / wasted %.1f%% of cycles; contention (>=2 clients) "
               "%.1f%%\n",
               100.0 * bus_busy / actual, 100.0 * bus_wasted / actual,
               100.0 * bus_contention / actual);
        printf(" bus payload: %llu real / %llu NOP beats (%.1f%% of bus beats are filler)\n",
               (unsigned long long)bus_real_beats, (unsigned long long)bus_nop_beats,
               bus_real_beats + bus_nop_beats
                   ? 100.0 * bus_nop_beats / (bus_real_beats + bus_nop_beats)
                   : 0.0);
#endif
    }
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
            sf << "port_serve," << port_serve << "\n";
            sf << "port_wait," << port_wait << "\n";
            for (const auto &gs : gstat) {
                sf << gs.name << "_real_beats," << gs.real_beats << "\n";
                sf << gs.name << "_nop_beats," << gs.nop_beats << "\n";
                sf << gs.name << "_conflicts," << gs.delayed << "\n";
                sf << gs.name << "_delay_sum," << gs.delay_sum << "\n";
                sf << gs.name << "_delay_max," << gs.delay_max << "\n";
                sf << gs.name << "_active_cycles," << gs.active_cycles << "\n";
                sf << gs.name << "_arrival_cycles," << gs.arrival_cycles << "\n";
                sf << gs.name << "_fill_delayed," << gs.fill_delayed << "\n";
                sf << gs.name << "_fill_delay_sum," << gs.fill_delay_sum << "\n";
                sf << gs.name << "_episodes," << gs.episodes << "\n";
                sf << gs.name << "_stall_episodes," << gs.stall_episodes << "\n";
                sf << gs.name << "_wait_cycles," << gs.wait_cycles << "\n";
                sf << gs.name << "_fill_wait_cycles," << gs.fill_wait_cycles << "\n";
                sf << gs.name << "_serve_cycles," << gs.serve_cycles << "\n";
                sf << gs.name << "_delayed_l1," << gs.delayed_l1 << "\n";
                sf << gs.name << "_delayed_l2," << gs.delayed_l2 << "\n";
                sf << gs.name << "_delayed_l3," << gs.delayed_l3 << "\n";
                sf << gs.name << "_delay_l1_sum," << gs.delay_l1_sum << "\n";
                sf << gs.name << "_delay_l2_sum," << gs.delay_l2_sum << "\n";
                sf << gs.name << "_delay_l3_sum," << gs.delay_l3_sum << "\n";
            }
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
            static const char *kLvl[4] = {"l1", "l2", "l3", "bank"};
            for (int l = 0; l < 4; ++l) {
                sf << "lvl_rd_" << kLvl[l] << "," << lvl_rd[l] << "\n";
                sf << "lvl_wr_" << kLvl[l] << "," << lvl_wr[l] << "\n";
                sf << "lvl_cyc_" << kLvl[l] << "," << lvl_cyc[l] << "\n";
            }
#endif
            sf << "rsp_events," << rsp_events << "\n";
            sf << "rsp_min," << (rsp_events ? rsp_min : 0) << "\n";
            sf << "rsp_max," << (rsp_events ? rsp_max : 0) << "\n";
            sf << "bank_busy," << bank_busy << "\n";
            sf << "bank_stall," << bank_stall << "\n";
            sf << "n_banks," << kNBanks << "\n";
#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV) && defined(XBAR_ROB)
            // Read-ROB experiment counters (see top_crossbar.hpp's XBAR_ROB
            // block). Note: per-LEVEL M1/M2 splits are not meaningful under
            // XBAR_ROB (external port waits are prefetch underruns, not
            // fabric-stage blocks); use the tot columns + these.
            sf << "rob_depth," << static_cast<int>(decltype(dut.impl)::ROB_DEPTH) << "\n";
            sf << "rob_underrun_wait," << dut.impl.rob_.underrun_wait << "\n";
            sf << "rob_fabric_hold," << dut.impl.rob_.fabric_hold << "\n";
            sf << "rob_sched_eligible," << dut.impl.rob_.sched_eligible << "\n";
            sf << "rob_sched_issued," << dut.impl.rob_.sched_issued << "\n";
            sf << "rob_ingest_groups," << dut.impl.rob_.ingest_groups << "\n";
            sf << "rob_mismatch," << dut.impl.rob_.mismatch_cnt << "\n";
            sf << "rob_sched_ooo," << dut.impl.rob_.sched_ooo << "\n";
#endif
#if defined(IMPL_TDM) && !defined(IMPL_SV)
            sf << "bus_busy," << bus_busy << "\n";
            sf << "bus_wasted," << bus_wasted << "\n";
            sf << "bus_contention," << bus_contention << "\n";
            sf << "bus_real_beats," << bus_real_beats << "\n";
            sf << "bus_nop_beats," << bus_nop_beats << "\n";
            sf << "bank_stall_cycles," << bank_stall_cycles << "\n";
#endif
        }
    }

#if defined(IMPL_CROSSBAR) && !defined(IMPL_SV)
    if (desc_sync) {
        // Per-descriptor overhead rollup (SEL_DESC_SYNC): overall drain vs ideal,
        // conflict vs barrier split (1c), per-level conflict wait-cycles, and
        // per-AGU (barriered groups) ideal + wall-clock contention stall.
        const uint64_t conflict_overhead =
            sum_overhead > barrier_overhead ? sum_overhead - barrier_overhead : 0;
        const uint64_t cw_l1 = lvl_rd[0] + lvl_wr[0];
        const uint64_t cw_l2 = lvl_rd[1] + lvl_wr[1];
        const uint64_t cw_l3 = lvl_rd[2] + lvl_wr[2];
        const double   ov_pct = sum_ideal ? 100.0 * sum_overhead / sum_ideal : 0.0;
        const int      bidx[5] = {0, 1, 2, 5, 6};  // gstat: ragu_a,b,c, wagu_a,b
        std::ofstream ds(out_dir + "/desc_sync_summary.txt");
        if (ds) {
            ds << "# per-descriptor overhead summary (SEL_DESC_SYNC)\n";
            ds << "descriptors," << desc_idx << "\n";
            ds << "sum_drain," << sum_drain << "\n";
            ds << "sum_ideal," << sum_ideal << "\n";
            ds << "total_overhead," << sum_overhead << "\n";
            ds << "conflict_overhead," << conflict_overhead << "\n";
            ds << "barrier_overhead," << barrier_overhead << "\n";
            ds << "overhead_pct," << ov_pct << "\n";
            ds << "conflict_wait_l1," << cw_l1 << "\n";
            ds << "conflict_wait_l2," << cw_l2 << "\n";
            ds << "conflict_wait_l3," << cw_l3 << "\n";
            ds << "# agu,name,real_beats,ideal_cyc,stall_cyc\n";
            for (int a = 0; a < 5; ++a) {
                const auto &gs = gstat[bidx[a]];
                ds << "agu," << gs.name << "," << gs.real_beats << "," << agu_ideal[a] << ","
                   << (gs.wait_cycles - gs.fill_wait_cycles) << "\n";
            }
        }
        printf(" desc-sync overhead: %d desc  drain=%llu ideal=%llu  overhead=%llu "
               "(conflict=%llu barrier=%llu, %.1f%%)  conflict-wait L1/L2/L3=%llu/%llu/%llu\n",
               desc_idx, (unsigned long long)sum_drain, (unsigned long long)sum_ideal,
               (unsigned long long)sum_overhead, (unsigned long long)conflict_overhead,
               (unsigned long long)barrier_overhead, ov_pct,
               (unsigned long long)cw_l1, (unsigned long long)cw_l2, (unsigned long long)cw_l3);
    }
#endif

    if (tf)
        sc_close_vcd_trace_file(tf);

    sc_stop();
    return 0;
}

#undef BIND_DUT_GROUP
