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
    // SEL_BANK_TRACE: write one line per cycle with the number of banks
    // served (req&&gnt) that cycle — the per-cycle parallelism timeline
    // (doc/report §5.1). Cheap: one small integer per cycle.
    std::ofstream bank_trace;
    if (std::getenv("SEL_BANK_TRACE"))
        bank_trace.open(out_dir + "/bank_trace.csv");
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
     defined(XBAR_HASH_L1_V2))
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

    sc_stop();
    return 0;
}

#undef BIND_DUT_GROUP
