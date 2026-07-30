// -----------------------------------------------------------------------------
// Unified native SystemC Address Generation Unit (AGU) testbench driver.
//
// The module exposes one black-box OBI-like manager interface and selects the
// access policy internally:
//   - agu_target::crossbar: one-shot request/grant/response per lane, driven
//     group-by-group. Used for all writes (regardless of backend) and for
//     reads against the crossbar backend. Pass tdm_window > 0 when this
//     drives a TDM write buffer, so this AGU pads a task's trace with addr=0
//     NOPs out to the buffer's window boundary (see step()/drive_requests()
//     and tdm.hpp/buffer_cell.hpp's addr=0 fast path) — otherwise a trace
//     ending mid-window would stall the buffer's fill stage forever. With
//     tdm_window == 0 (talking directly to the crossbar, no buffer window to
//     fill) drive_requests() drops padding lanes at the port instead — no
//     request at all — since a live addr=0 NOP would just contend for a real
//     bank slot (every NOP hashes to the same bank) against other buffers'
//     genuine traffic for no reason.
//   - agu_target::tdm: reads against a TDM read buffer. REQUIRES tdm_window
//     > 0 (enforced at construction). The read buffer's cells prefetch a
//     whole window's worth of addresses at once from lookahead_addr()/
//     lookahead_ready() (see their own comments below and
//     advance_lookahead_window()/retry_lookahead_fence()) — the caller wires
//     those into the buffer's fetch_addr_i[]/fetch_addr_valid_i every cycle
//     (see stim_bank_common.hpp). step_tdm_read() below just asserts the
//     current group's request and waits for its response.
//
// Stimuli file format (lines starting with '#' are task descriptors):
//
//   #cycle, num_port_active, R, C, L, storemode   <- full descriptor (RAGU/WAGU)
//   #cycle, num_port_active, storemode             <- short descriptor (no CRL)
//   0x00000000
//   0x00000010
//   ...
//   #next_cycle, ...                               <- next task (acts as fence)
//   0x00001000
//   ...
//
//   Fields:
//     cycle           : earliest start cycle for this task; the AGU waits until
//                       the previous task completes AND cycle_ >= this value (fence).
//     num_port_active : number of active port groups; ports_used = num_port_active
//                       * N_PER_GROUP.
//     storemode       : passed to the TDM mapping function.
//     C, R, L         : column/row/line geometry passed to the TDM mapping function.
//                       Omit entirely for a group with no TDM mapping geometry
//                       (has_crl = false). RAGU_E/WAGU_E used this short
//                       form historically, but now use their own dedicated
//                       driver (lane_agu.hpp) and format instead — this form
//                       remains a general agu<> feature, not DMA-specific.
//
//   Address lines follow each descriptor.  RAGU: addr only (implicit read).
//   WAGU: addr,data (implicit write).  The "addr" header line is optional and skipped.
//
// TDM mapping note: p_C_, p_R_, p_L_, p_store_mode_ always reflect the
// CURRENT task's parameters.  tb_top should re-drive map_*_cfg each clock
// cycle from these fields so the mapping function uses the correct geometry
// for whichever buffer is active.
// -----------------------------------------------------------------------------

#ifndef AGU_HPP
#define AGU_HPP

#include <systemc.h>

#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>
#include <vector>

#include "csv_parse_util.hpp"
#include "obi_data.hpp"
#include "obi_ports.hpp"

enum class agu_target { crossbar, tdm };

template <typename T> static inline T agu_data_from_u64(uint64_t v) {
    return T(static_cast<unsigned long long>(v));
}

template <> inline uint64_t agu_data_from_u64<uint64_t>(uint64_t v) {
    return v;
}

template <typename T> static inline std::string agu_data_hex(const T &v) {
    std::ostringstream os;
    os << v;
    return os.str();
}

template <> inline std::string agu_data_hex<uint64_t>(const uint64_t &v) {
    std::ostringstream os;
    os << "0x" << std::hex << std::setw(8) << std::setfill('0') << v;
    return os.str();
}

template <int NUM_REQ = 4, typename DATA_T = uint64_t, int BYTES_PER_BEAT = 4, int N_PER_GROUP = 1>
SC_MODULE(agu) {
    using data_t = DATA_T;

    SC_HAS_PROCESS(agu);

    sc_in<bool>               clk_i;
    sc_in<bool>               rst_ni;
    obi_manager_ports<data_t> obi[NUM_REQ];
    sc_out<bool>              done_o;

    struct access_t {
        uint64_t cycle;
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::vector<access_t> log_;

    static constexpr uint32_t kBeFull = (BYTES_PER_BEAT >= 32) ? ~0u : ((1u << BYTES_PER_BEAT) - 1);

    std::string out_path_;
    agu_target  target_;

    // Size (in cells) of the TDM buffer window this AGU is connected to, or 0
    // to disable all TDM-window-aware behaviour below (crossbar backend, no
    // windowing there). Used two different ways depending on which side of a
    // TDM buffer this AGU drives:
    //
    //   Write (target_==crossbar, e.g. WAGU): a TDM write buffer must fill
    //   its entire fixed-size window (accumulate-then-flush) before it can
    //   complete a task; if a task's real trace ends partway through a
    //   window, this AGU pads the remaining groups with addr=0 NOP writes
    //   (see buffer_cell.hpp/tdm.hpp addr=0 fast path) so fill can still
    //   reach the window boundary and flush. See load_trace()'s n_groups
    //   rounding and drive_requests()'s NOP-padding.
    //
    //   Read (target_==tdm, e.g. RAGU): a TDM read buffer's cells prefetch a
    //   whole window at once from lookahead_addr()/lookahead_ready() (see
    //   their own comments below) — this AGU's job on the read side is just
    //   to assert the current group's request and wait for its response
    //   (see step_tdm_read()).
    std::size_t tdm_window_;

    // Per-lane "has this group's response already been captured" latch,
    // used by step_tdm_read() to detect when every active lane's response
    // for the current group has arrived.
    bool tdm_cap_done_[NUM_REQ] = {};

    // -------------------------------------------------------------------------
    // Per-task state
    // -------------------------------------------------------------------------
    struct trace_entry_t {
        uint64_t addr;
        bool     we;
        data_t   data;
    };

    struct task_t {
        uint64_t                   start_cycle = 0;
        int                        ports_used  = NUM_REQ;
        uint64_t                   store_mode  = 0;
        uint64_t                   C = 4, R = 4, L = 8;
        bool                       has_crl = true;
        std::vector<trace_entry_t> trace;
        std::size_t                n_groups = 0;

        std::size_t trace_base(std::size_t g) const {
            return g * static_cast<std::size_t>(ports_used);
        }
        // Trace-entry offset served by flat lane p within a group.
        // Default (port-major): consecutive trace entries fill one port's
        // lanes before moving to the next port. SEL_PORT_INTERLEAVE
        // (cross-port distribution, matching hardware AGUs that deal
        // addresses across ports first): entry i goes to port i % napa,
        // lane i / napa — so flat lane p (port p/N, lane p%N) serves entry
        // (p % N_PER_GROUP) * napa + p / N_PER_GROUP. The choice changes
        // which addresses share one port's L1 switch on the crossbar and
        // one window slot group on TDM.
        static bool port_interleave() {
            static const bool v = std::getenv("SEL_PORT_INTERLEAVE") != nullptr;
            return v;
        }
        std::size_t lane_off(int p) const {
            if (!port_interleave())
                return static_cast<std::size_t>(p);
            const int napa = ports_used / N_PER_GROUP;
            if (napa <= 1)
                return static_cast<std::size_t>(p);
            return static_cast<std::size_t>((p % N_PER_GROUP) * napa + p / N_PER_GROUP);
        }
        bool has_row(std::size_t g, int p) const {
            return p < ports_used && trace_base(g) + lane_off(p) < trace.size();
        }
    };

    std::vector<task_t> tasks_;
    std::size_t         task_idx_;
    std::size_t         group_;
    uint64_t            cycle_;

    // -------------------------------------------------------------------------
    // Public fields — always reflect the CURRENT task; used by tb_top for
    // active_mode selection (ports_used_) and TDM map config (p_C_ etc.).
    // n_groups_ is the total across all tasks (used for ideal-cycle stats).
    // -------------------------------------------------------------------------
    int         ports_used_;
    uint64_t    start_cycle_;
    uint64_t    p_C_, p_R_, p_L_, p_store_mode_;
    bool        p_has_crl_;
    std::size_t n_groups_;

    bool granted_[NUM_REQ];

    // In-flight tracking for the crossbar-style path (writes, and non-TDM
    // reads): one request per lane, granted immediately, response arrives
    // later. TDM reads never take this path — see step_tdm_read().
    struct lane_rec_t {
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::deque<lane_rec_t> lane_inflight_[NUM_REQ];

    // -------------------------------------------------------------------------
    // Helpers — trim() comes from csv_parse_util.hpp (shared with lane_agu.hpp)
    // -------------------------------------------------------------------------
    static std::vector<uint64_t> parse_csv_u64(const std::string &line,
                                               std::size_t        skip_prefix = 0) {
        std::vector<uint64_t> v;
        std::size_t           pos = skip_prefix;
        while (pos <= line.size()) {
            const std::size_t c = line.find(',', pos);
            const std::string tok =
                trim(c == std::string::npos ? line.substr(pos) : line.substr(pos, c - pos));
            if (!tok.empty())
                v.push_back(std::strtoull(tok.c_str(), nullptr, 0));
            if (c == std::string::npos)
                break;
            pos = c + 1;
        }
        return v;
    }

    bool all_tasks_done() const {
        return task_idx_ >= tasks_.size();
    }

    const task_t &cur_task() const {
        return tasks_[task_idx_];
    }

    bool has_row(std::size_t g, int p) const {
        return !all_tasks_done() && cur_task().has_row(g, p);
    }

    // The TDM buffer's active_mode encoding has exactly three states — 1, 2,
    // or 4 port-groups (see buffer.hpp's ports_for_mode()/beats_for_mode()) —
    // so a task whose own ports_used isn't already one of those (e.g. 3
    // groups, num_port_active=3) gets rounded UP to the next one by whoever
    // drives active_mode from it (top_tdm.hpp callers all replicate this same
    // g<=1?0:g<=2?1:2 rule). Left unaddressed, the buffer then waits for
    // request on a WIDER group than this AGU ever asserts — a real,
    // permanent deadlock (the buffer's per-group "every lane requests" gate
    // can never close), not a slow path: confirmed directly against
    // production stimuli where a num_port_active=3 task exists in a build
    // targeting the TDM backend. This computes the same rounded width so the
    // request/grant/capture loops below can pad the gap with real (granted)
    // NOP requests instead of leaving those lanes permanently silent.
    static int rounded_width(int ports_used) {
        const int g  = (N_PER_GROUP > 0 && ports_used > 0) ? ports_used / N_PER_GROUP : 1;
        const int rg = (g <= 1) ? 1 : (g <= 2) ? 2 : 4;
        return rg * N_PER_GROUP;
    }

    int rounded_ports_used() const {
        return all_tasks_done() ? 0 : rounded_width(cur_task().ports_used);
    }

    // Loop bound record_grants()/advance_group_if_granted()/drive_requests()/
    // step_tdm_read() all agree on: the task's own port count for crossbar
    // targets (tdm_window_==0 — no downstream grouping constraint to round
    // for), or that count rounded up per rounded_ports_used() for TDM
    // targets. `real` (whether a given lane has actual trace data) is
    // always computed separately against the task's TRUE ports_used —
    // this only widens how many lanes get a request/grant/capture cycle.
    int effective_ports_used() const {
        if (all_tasks_done())
            return 0;
        return tdm_window_ > 0 ? rounded_ports_used() : cur_task().ports_used;
    }

    uint64_t request_addr(std::size_t g, int p) const {
        const task_t &t = cur_task();
        return t.trace[t.trace_base(g) + t.lane_off(p)].addr;
    }

    bool request_we(std::size_t g, int p) const {
        const task_t &t = cur_task();
        return t.trace[t.trace_base(g) + t.lane_off(p)].we;
    }

    data_t request_data(std::size_t g, int p) const {
        if (!request_we(g, p))
            return data_t(0);
        const task_t &t = cur_task();
        return t.trace[t.trace_base(g) + t.lane_off(p)].data;
    }

    bool has_inflight() const {
        for (int p = 0; p < NUM_REQ; ++p)
            if (!lane_inflight_[p].empty())
                return true;
        return false;
    }

    // Sync public compat fields to the current task
    void sync_public_fields() {
        if (all_tasks_done())
            return;
        const task_t &t = cur_task();
        ports_used_     = t.ports_used;
        start_cycle_    = t.start_cycle;
        p_C_            = t.C;
        p_R_            = t.R;
        p_L_            = t.L;
        p_store_mode_   = t.store_mode;
        p_has_crl_      = t.has_crl;
    }

    // -------------------------------------------------------------------------
    // Parsing
    // -------------------------------------------------------------------------
    static bool no_fence() {
        static const bool v = std::getenv("SEL_NO_FENCE") != nullptr;
        return v;
    }

    // Hidden lookahead: let the LOOKAHEAD cursor roll into a fenced task
    // SEL_LA_LEAD cycles before its start_cycle, so the buffer boot-latches
    // and prefetches the task's first window during the fence gap and the
    // data is already resident when the fence expires. The CAPTURE side
    // keeps the exact fence (step()/advance gates below), so the port
    // starts consuming at start_cycle sharp — the task-start pipeline fill
    // (doc/report §4) is paid inside the idle gap instead of after it.
    // Lead 0 (default) preserves the original behaviour. The lead must stay
    // under the producer's fence margin when fences order cross-stream
    // write->read dependencies.
    static uint64_t la_lead() {
        static const uint64_t v = [] {
            const char *e = std::getenv("SEL_LA_LEAD");
            return e ? std::strtoull(e, nullptr, 0) : uint64_t{0};
        }();
        return v;
    }

    // Parse a task header: #cycle, num_port_active, R, C, L, storemode (full)
    // or #cycle, num_port_active, storemode (short, no map geometry)
    task_t parse_task_header(const std::string &line) const {
        const std::vector<uint64_t> v = parse_csv_u64(line, 1); // skip '#'
        if (v.size() < 3)
            SC_REPORT_FATAL(name(),
                            "task descriptor needs at least: #cycle, num_port_active, storemode");
        task_t t;
        t.start_cycle = v[0];
        t.ports_used  = static_cast<int>(v[1]) * N_PER_GROUP;
        if (v.size() >= 6) { // full descriptor: #cycle, napa, R, C, L, storemode
            t.R          = v[2];
            t.C          = v[3];
            t.L          = v[4];
            t.store_mode = v[5];
            t.has_crl    = true;
        } else { // short descriptor: #cycle, napa, storemode (no map geometry)
            t.store_mode = v[2];
            t.has_crl    = false;
        }
        if (t.ports_used < N_PER_GROUP || t.ports_used > NUM_REQ || t.ports_used % N_PER_GROUP != 0)
            SC_REPORT_FATAL(name(),
                            "num_port_active out of range: must be 1..NUM_REQ/N_PER_GROUP groups");
        return t;
    }

    // "addr" (RAGU) or "addr,data" (WAGU), optionally followed by more
    // comma-separated fields this driver doesn't use (e.g. a trailing
    // ",# N" comment some trace exports carry) — only the first two fields
    // are ever meaningful, so split_csv() + index access ignores the rest
    // rather than the field-2-is-write test misreading a comment as data.
    trace_entry_t parse_addr_line(const std::string &line) const {
        const std::vector<std::string> f = split_csv(line);
        trace_entry_t                  e;
        e.addr = std::strtoull(f[0].c_str(), nullptr, 0);
        if (f.size() < 2 || f[1].empty()) {
            e.we   = false;
            e.data = data_t(0);
        } else {
            // "addr,data,..." → implicit write
            e.we   = true;
            e.data = agu_data_from_u64<data_t>(std::strtoull(f[1].c_str(), nullptr, 0));
        }
        return e;
    }

    void load_trace(const std::string &path_in) {
        const std::string path = resolve_stim_path(path_in);
        std::ifstream     f(path.c_str());
        if (!f) {
            SC_REPORT_INFO(name(), ("no stimuli (" + path + "), will be idle").c_str());
            return;
        }

        // Collect non-empty lines
        std::vector<std::string> lines;
        std::string              ln;
        while (std::getline(f, ln)) {
            ln = trim(ln);
            if (!ln.empty())
                lines.push_back(ln);
        }

        // '#' lines open a new task; other (non-"addr") lines are addresses
        // belonging to the most recently opened task.
        task_t *cur = nullptr;
        for (const auto &l : lines) {
            if (l[0] == '#') {
                tasks_.push_back(parse_task_header(l));
                cur = &tasks_.back();
            } else if (l != "addr") {
                if (!cur)
                    SC_REPORT_FATAL(name(), "address line before first '#' descriptor");
                cur->trace.push_back(parse_addr_line(l));
            }
        }

        // SEL_NO_FENCE (throughput-bound harness mode): remove every
        // fence, including the stream's own first one — the buffer is
        // idle from reset, so task 0 doesn't need lookahead-based fill
        // hiding the way a mid-stream task does (there's no prior task's
        // residue to roll seamlessly past); it can boot-latch its first
        // window at cycle 0 exactly like any other from-idle start
        // elsewhere in this codebase. Left un-zeroed, task 0's ORIGINAL
        // start_cycle (copied straight from the fenced schedule) can be
        // tens of thousands of cycles on real production stimuli — not a
        // small pipeline-fill bubble, the dominant term in the reported
        // "unfenced" total (measured directly: 23271 of a 31431-cycle
        // total on one real set, i.e. 74% of what this mode reports as
        // "throughput-bound" was actually just this one un-removed fence).
        // Every later task's fence is removed the same way, and — only
        // when there's more than one task to begin with — consecutive
        // SAME-geometry tasks are also merged into one (trace
        // concatenation): the window rounding below then pads to the TDM
        // window boundary once per merged run instead of once per task,
        // removing the addr-0 filler beats that otherwise occupy the
        // shared bus between every pair of adjacent tasks. Geometry must
        // match exactly (ports_used + C/R/L/store_mode) — the same
        // condition as the buffer's seamless task-roll path
        // (la_same_geometry()).
        if (no_fence())
            for (auto &t : tasks_)
                t.start_cycle = 0;
        if (no_fence() && tasks_.size() > 1) {
            std::vector<task_t> packed;
            packed.push_back(std::move(tasks_.front()));
            for (std::size_t i = 1; i < tasks_.size(); ++i) {
                task_t    &prev      = packed.back();
                task_t    &t         = tasks_[i];
                const bool same_geom = t.ports_used == prev.ports_used && t.C == prev.C &&
                                       t.R == prev.R && t.L == prev.L &&
                                       t.store_mode == prev.store_mode && t.has_crl == prev.has_crl;
                if (same_geom)
                    prev.trace.insert(prev.trace.end(), std::make_move_iterator(t.trace.begin()),
                                      std::make_move_iterator(t.trace.end()));
                else
                    packed.push_back(std::move(t));
            }
            tasks_.swap(packed);
        }

        // Compute n_groups per task
        for (auto &t : tasks_) {
            t.n_groups = t.ports_used > 0 ? (t.trace.size() + t.ports_used - 1) /
                                                static_cast<std::size_t>(t.ports_used)
                                          : 0;
            // Round up to a full TDM buffer window: tdm_window_ /
            // rounded_width(ports_used) groups fill exactly one window (the
            // ROUNDED width is what the buffer's active_mode encoding
            // actually drains/fills at — see rounded_width()); the remainder
            // past the real trace is driven as addr=0 NOP beats (see
            // drive_requests()/record_grants()/step_tdm_read()).
            if (tdm_window_ > 0 && t.ports_used > 0) {
                const std::size_t groups_per_window =
                    static_cast<std::size_t>(tdm_window_) /
                    static_cast<std::size_t>(rounded_width(t.ports_used));
                if (groups_per_window > 0) {
                    const std::size_t rem = t.n_groups % groups_per_window;
                    if (rem != 0)
                        t.n_groups += groups_per_window - rem;
                }
            }
        }

        // Total groups across all tasks (used for ideal-cycle stats in tb_top)
        n_groups_ = 0;
        for (const auto &t : tasks_)
            n_groups_ += t.n_groups;
    }

    // -------------------------------------------------------------------------
    // Simulation
    // -------------------------------------------------------------------------
    void reset_state() {
        for (int p = 0; p < NUM_REQ; ++p) {
            obi[p].req_o.write(false);
            obi[p].addr_o.write(0);
            obi[p].we_o.write(false);
            obi[p].be_o.write(0);
            obi[p].wdata_o.write(0);
            granted_[p] = false;
            lane_inflight_[p].clear();
        }
        task_idx_    = 0;
        group_       = 0;
        la_task_idx_ = 0;
        la_group_    = 0;
        cycle_       = 0;
        done_o.write(false);
        log_.clear();
        for (int p = 0; p < NUM_REQ; ++p)
            tdm_cap_done_[p] = false;
        sync_public_fields();
    }

    // Lookahead window cursor — deliberately SEPARATE from task_idx_/group_
    // above. task_idx_/group_ track this AGU's own
    // CAPTURE/log-side progress, which only advances once rvalid_i (a
    // registered OBI response) confirms a group; the buffer's cells, by
    // contrast, prefetch a whole window autonomously as soon as it resets,
    // gated only by cell_valid (ports_req is unconditionally true once a
    // task is active — see buffer.hpp's eval_group()). So the buffer's own
    // window-reset pace and this AGU's group_ are NOT a fixed one-cycle
    // apart; they can drift by an arbitrary amount. Deriving the lookahead
    // window from group_ (even "group_+1") is therefore unsound — it must
    // instead be driven directly off the buffer's observed reset pulse (see
    // advance_lookahead_window(), called by stim_bank_common.hpp once per
    // window_reset it sees on the buffer side).
    std::size_t la_task_idx_ = 0;
    std::size_t la_group_    = 0;

    // True once la_task_idx_/la_group_ point at real, in-range trace content
    // — false while a pending task rollover is stuck behind a start_cycle
    // fence (see retry_lookahead_fence()). stim_bank_common.hpp gates each
    // read buffer's fetch_addr_valid_i on this so cells hold IDLE (no fetch
    // at all) instead of latching stale/zero content while blocked — see
    // that wiring's comment in top_tdm.hpp for why an unconditional fetch
    // would otherwise permanently desync this cursor from task_idx_/group_.
    bool lookahead_ready() const {
        return la_task_idx_ < tasks_.size() && la_group_ < tasks_[la_task_idx_].n_groups;
    }

    // la_-synchronized counterparts to ports_used_/p_C_/p_R_/p_L_/
    // p_store_mode_ (sync_public_fields() above): those track task_idx_,
    // which only advances once this AGU's OWN (slower, response-gated)
    // capture confirms a task is done — reflecting whichever window the
    // BUFFER's hardware is actually fetching/draining RIGHT NOW requires
    // la_task_idx_ instead. Driving active_mode (and the TDM map geometry)
    // from the task_idx_-based fields instead of these is what let a
    // buffer-side latency optimization (fetching a window's first group the
    // same edge as its reset, instead of one edge later) briefly drain a
    // NEW task's data using the OLD task's group width — active_mode hadn't
    // switched yet because task_idx_ hadn't caught up, even though the
    // buffer's own cells (fed by la_) already had the new task's addresses.
    // Fixed by having the caller (stim_bank_common.hpp) drive active_mode
    // from these accessors for read buffers instead.
    //
    // NOTE (production-stimuli deadlock, see doc/report A.6 Cause 2): a
    // one-window-delayed exposure (mirroring lane_agu.hpp's window_queue_)
    // was prototyped here and reverted — it does not fix the underlying
    // race and introduced a NEW regression (a previously-clean dataset
    // deadlocked under the round-robin arbiter with the delay in place).
    // The real gap is a registered-signal causality issue in buffer.hpp:
    // window_reset is only OBSERVABLE to this caller one cycle after
    // window_mode_q has already re-latched (on the wrap edge itself) from
    // whatever active_mode held AT THAT EDGE — so any external, per-cycle
    // active_mode driver is structurally one edge too late whenever a task
    // boundary changes ports_used, independent of how that driver computes
    // its value. Closing this needs an RTL-side fix (e.g. a combinational
    // pre-wrap signal exposed from buffer.hpp), not a testbench change.
    int lookahead_ports_used() const {
        return la_task_idx_ < tasks_.size() ? tasks_[la_task_idx_].ports_used : ports_used_;
    }
    uint64_t lookahead_C() const {
        return la_task_idx_ < tasks_.size() ? tasks_[la_task_idx_].C : p_C_;
    }
    uint64_t lookahead_R() const {
        return la_task_idx_ < tasks_.size() ? tasks_[la_task_idx_].R : p_R_;
    }
    uint64_t lookahead_L() const {
        return la_task_idx_ < tasks_.size() ? tasks_[la_task_idx_].L : p_L_;
    }
    uint64_t lookahead_store_mode() const {
        return la_task_idx_ < tasks_.size() ? tasks_[la_task_idx_].store_mode : p_store_mode_;
    }

    // Retries a task rollover that's pending behind a start_cycle fence.
    // Called every cycle by the caller (not just on a window_reset), since
    // fetch_addr_valid_i being gated on lookahead_ready() means the buffer
    // never fetches — and so never completes another window_reset — while
    // blocked; something has to re-check the fence on a plain cycle tick
    // instead of waiting for an event that can't happen yet. No-op if
    // nothing is pending.
    void retry_lookahead_fence() {
        if (la_task_idx_ >= tasks_.size() || la_group_ < tasks_[la_task_idx_].n_groups)
            return;
        const std::size_t next = la_task_idx_ + 1;
        if (next >= tasks_.size()) {
            la_task_idx_ = tasks_.size();
            return;
        }
        if (cycle_ + la_lead() < tasks_[next].start_cycle)
            return; // still fenced (la_lead() lets prefetch start early)
        // Geometry-changing task transitions must go through the buffer's
        // atomic all-cells-idle boot latch — see la_task_roll_gate_open_'s
        // comment for the deadlock this prevents. Same-geometry transitions
        // stream seamlessly (nothing the buffer latches per window changes,
        // so there is no atomicity to protect) — EXCEPT when the roll is
        // EARLY (la_lead(), still ahead of the fence): rolling before the
        // capture side has finished the outgoing task disarms the
        // between-tasks flush (whose stale-residue test is exactly "la
        // parked at this task"), stranding residue the flush would have
        // drained. An early roll therefore always takes the clean-boot
        // path: capture done, residue flushed, buffer idle — the prefetch
        // then fills the fence gap and the data is resident at start_cycle
        // whenever the gap allows; otherwise behaviour degrades gracefully
        // to the original at-fence roll.
        const bool early = cycle_ < tasks_[next].start_cycle;
        if (early || !la_same_geometry(tasks_[la_task_idx_], tasks_[next])) {
            const bool capture_done =
                task_idx_ > la_task_idx_ ||
                (task_idx_ == la_task_idx_ && group_ >= tasks_[la_task_idx_].n_groups);
            if (!capture_done || !la_task_roll_gate_open_)
                return;
        }
        la_task_idx_ = next;
        la_group_    = 0;
    }

    // Two consecutive tasks are geometry-equivalent when everything the
    // buffer/map latches or muxes per window is identical: the ROUNDED
    // active_mode group width (a num_port_active=3 task and a 4 task both
    // run 16-wide — see rounded_ports_used()) and the TDM map parameters.
    static bool la_same_geometry(const task_t &a, const task_t &b) {
        const auto mode_of = [](int pu) {
            const int g = (N_PER_GROUP > 0 && pu > 0) ? pu / N_PER_GROUP : 1;
            return (g <= 1) ? 0 : (g <= 2) ? 1 : 2;
        };
        return mode_of(a.ports_used) == mode_of(b.ports_used) && a.C == b.C && a.R == b.R &&
               a.L == b.L && a.store_mode == b.store_mode;
    }

    // TDM read-target only, caller-driven from the matching buffer's own
    // observed occupancy (snapshot().n_valid == 0). Geometry (active_mode,
    // from lookahead_ports_used()) and content (fetch_addr_i, from
    // lookahead_addr()) are only latched ATOMICALLY by the buffer on its
    // all-cells-idle boot restart; the steady-state wrap relatch instead
    // samples live active_mode at the wrap edge — one cycle before this
    // AGU's caller can even observe that wrap (window_reset is registered).
    // A geometry-changing task transition mid-stream therefore latches one
    // task's width for another task's content: a permanent width-mismatch
    // deadlock, reproduced deterministically by tb_task_boundary.cpp's
    // fence-on-the-wrap-edge case and confirmed against production stimuli
    // (doc/report A.6 Cause 2b). retry_lookahead_fence() above holds such
    // transitions until (a) this AGU's own capture has finished the
    // outgoing task AND (b) this gate reports the buffer idle — the
    // drained cells park (lookahead_ready() is false while the roll is
    // held, so nothing refetches), the buffer reaches all-idle, and the
    // new task's geometry and content then arrive together on the restart
    // edge. Defaults to true so callers that never drive it see the old
    // behavior; the between-tasks flush (step_tdm_read()) is what
    // guarantees the residue actually drains while the roll is held.
    bool la_task_roll_gate_open_ = true;

    // Called once per observed buffer window_reset pulse, BEFORE the next
    // edge — advances straight to the next window, then delegates to
    // retry_lookahead_fence() to roll into the next task if that crossed
    // n_groups (respecting the same start_cycle fence as
    // advance_task_if_ready_tdm_read()).
    void advance_lookahead_window() {
        if (la_task_idx_ >= tasks_.size())
            return;
        // One window_reset completes a whole window's worth of groups
        // (tdm_window_ / ports_used), not a single group — la_group_ must
        // jump by that stride, not by 1, or a task whose n_groups equals
        // exactly one window (the common case, thanks to load_trace()'s own
        // window-rounding) would need N window_reset pulses to ever cross
        // into the next task instead of the one it actually gets.
        const task_t     &cur = tasks_[la_task_idx_];
        const std::size_t groups_per_window =
            static_cast<std::size_t>(tdm_window_) /
            static_cast<std::size_t>(rounded_width(cur.ports_used));
        la_group_ += groups_per_window > 0 ? groups_per_window : 1;
        retry_lookahead_fence();
    }

    // TDM read-target only (see step_tdm_read()): the w-th address of the
    // window la_task_idx_/la_group_ currently point at, taken straight from
    // this AGU's own pre-loaded trace (this AGU
    // always knows its own future, unlike a real hardware AGU that
    // generates addresses on the fly) — zero past the end of the trace,
    // matching every other NOP-padding convention in this file. trace is
    // already ordered group-then-port (see task_t's own comment), so "the
    // next N addresses" is a direct slice at trace_base(la_group_), not a
    // per-port walk. A plain accessor rather than an sc_out port: agu<> is
    // instantiated for write groups too (which have no use for this), and
    // an sc_out port would need binding on every one of them regardless —
    // the caller (e.g. stim_bank_common.hpp) polls this once per cycle to
    // drive the read buffer's own lookahead input signals directly.
    uint64_t lookahead_addr(int w) const {
        if (la_task_idx_ >= tasks_.size())
            return 0;
        const task_t &t = tasks_[la_task_idx_];
        // Window slots are laid out at the ROUNDED group width (the width
        // the buffer actually drains at — see rounded_width()); the lanes
        // between a task's real ports_used and that rounded width are NOP
        // holes. For the encodable widths (1/2/4 groups) rounded == real
        // and this reduces exactly to the original flat slice; only an
        // unencodable width (num_port_active=3) gets the holes — without
        // them the flat 12-wide slice and the buffer's 16-wide drain
        // grouping shear apart, delivering group g+1's rows inside group
        // g's drain.
        const int rw = rounded_width(t.ports_used);
        if (rw <= 0)
            return 0;
        const std::size_t g = la_group_ + static_cast<std::size_t>(w) / rw;
        const int         p = w % rw;
        if (p >= t.ports_used)
            return 0; // NOP hole
        const std::size_t idx = t.trace_base(g) + t.lane_off(p);
        return idx < t.trace.size() ? t.trace[idx].addr : uint64_t{0};
    }

    void collect_crossbar_responses() {
        for (int p = 0; p < NUM_REQ; ++p) {
            if (!lane_inflight_[p].empty() && obi[p].rvalid_i.read()) {
                const lane_rec_t rec = lane_inflight_[p].front();
                lane_inflight_[p].pop_front();
                log_.push_back(
                    {cycle_, rec.addr, rec.we, rec.we ? rec.data : obi[p].rdata_i.read()});
            }
        }
    }

    void record_grants() {
        if (all_tasks_done() || cycle_ < cur_task().start_cycle)
            return;
        const task_t &t  = cur_task();
        const int     ew = effective_ports_used();
        for (int p = 0; p < ew; ++p) {
            // Grants apply to every active lane, real row or NOP pad alike —
            // the write buffer's fill stage needs a real gnt for padding
            // beats too before its fill-ptr can advance. Pad beats get an
            // inflight entry TOO (addr=0): the buffer acks pad groups like
            // any other, and with its pipelined write path a task's acks
            // stream concurrently with the NEXT task's grants — a pad ack
            // popping against a one-sided FIFO would consume the next
            // task's entry instead, mis-stamping every later response's
            // cycle (caught as phantom +1 spans in the timing report).
            if (obi[p].req_o.read() && obi[p].gnt_i.read()) {
                granted_[p]     = true;
                const bool real = t.has_row(group_, p);
                lane_inflight_[p].push_back({real ? request_addr(group_, p) : uint64_t{0},
                                             real ? request_we(group_, p) : true,
                                             real ? request_data(group_, p) : data_t(0)});
            }
        }
    }

    void advance_group_if_granted() {
        if (all_tasks_done())
            return;
        const task_t &t = cur_task();
        if (group_ >= t.n_groups)
            return;

        // Every active lane (real row, NOP pad, or a rounded-mode padding
        // lane — see effective_ports_used()) must be granted before the
        // group advances — has_row() alone would be vacuously "all granted"
        // during padding groups, racing ahead of the buffer's actual state.
        bool all_granted = true;
        for (int p = 0; p < effective_ports_used(); ++p) {
            if (!granted_[p]) {
                all_granted = false;
                break;
            }
        }
        if (!all_granted)
            return;

        ++group_;
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;
    }

    // Fence: advance to the next task once the current one is fully complete
    // (all groups issued and all in-flight responses received) and the next
    // task's start cycle has been reached.
    void advance_task_if_ready() {
        if (all_tasks_done())
            return;
        if (group_ < cur_task().n_groups || has_inflight())
            return;
        // Current task done. Find next task respecting its fence cycle.
        const std::size_t next = task_idx_ + 1;
        if (next >= tasks_.size()) {
            task_idx_ = tasks_.size(); // sentinel: all done
            return;
        }
        if (cycle_ < tasks_[next].start_cycle)
            return; // fence: wait
        task_idx_ = next;
        group_    = 0;
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;
        sync_public_fields();
    }

    void drive_requests() {
        for (int p = 0; p < NUM_REQ; ++p) {
            const bool active = !all_tasks_done() && cycle_ >= cur_task().start_cycle &&
                                group_ < cur_task().n_groups && p < effective_ports_used() &&
                                !granted_[p];
            if (active) {
                const bool real = has_row(group_, p);
                // A row is a NOP either from out-of-bounds padding (!real)
                // or because an in-bounds trace row's own address is the
                // literal 0x0 filler sentinel — has_row() only checks index
                // bounds, not content, so an explicit addr=0 stimuli line
                // (e.g. the padded/patroklos sources' filler rows) would
                // otherwise be driven as a live request. Both must be
                // dropped identically for the crossbar backend: every NOP
                // hashes to the same bank (top_crossbar.hpp's
                // addr_hash(0)==0), so a live one would just steal a real
                // grant slot from whichever other buffer's genuine traffic
                // lands there too (regression test:
                // doc/report/tools/check_stats_invariants.sh's
                // nop_regression case).
                const bool nop = !real || request_addr(group_, p) == 0;
                if (nop && tdm_window_ == 0) {
                    // Drop it at the port instead — no request at all — and
                    // treat it as trivially granted so
                    // advance_group_if_granted() doesn't wait on a grant that
                    // will never come. TDM write buffers (tdm_window_ > 0)
                    // still need the real NOP request below: their fill stage
                    // requires an actual gnt per beat to know the window is
                    // complete (see this file's header comment).
                    obi[p].addr_o.write(0);
                    obi[p].we_o.write(false);
                    obi[p].be_o.write(0);
                    obi[p].wdata_o.write(0);
                    obi[p].req_o.write(false);
                    granted_[p] = true;
                    continue;
                }
                // Real trace row, or (tdm_window_ > 0 only) a NOP pad
                // (addr=0) filling out the rest of a TDM write buffer's
                // window past the real trace.
                obi[p].addr_o.write(real ? request_addr(group_, p) : uint64_t{0});
                obi[p].we_o.write(real ? request_we(group_, p) : false);
                obi[p].be_o.write(kBeFull);
                obi[p].wdata_o.write(real ? request_data(group_, p) : data_t(0));
                obi[p].req_o.write(true);
            } else {
                obi[p].addr_o.write(0);
                obi[p].we_o.write(false);
                obi[p].be_o.write(0);
                obi[p].wdata_o.write(0);
                obi[p].req_o.write(false);
            }
        }
    }

    // TDM read-target only, caller-driven alongside la_task_roll_gate_open_:
    // while true, the transition into the next task is deferred even once
    // its fence has passed, keeping the between-tasks flush (old-width NOP
    // requests — see step_tdm_read()) alive so the buffer's residue drains
    // at the width its window was latched with. Without it, this AGU would
    // switch its request width to the NEW task while the buffer still holds
    // an OLD-width window — the same mismatch the la roll gate guards, from
    // the other side. Defaults false (no behavior change for callers that
    // don't drive it).
    bool flush_hold_ = false;

    // Fence for the TDM read-drain protocol: identical intent to
    // advance_task_if_ready(), but called from step_tdm_read() once the
    // current group's response has been fully captured, rather than from
    // the crossbar-style grant machinery.
    void advance_task_if_ready_tdm_read() {
        if (all_tasks_done())
            return;
        if (group_ < cur_task().n_groups)
            return;
        const std::size_t next = task_idx_ + 1;
        if (next >= tasks_.size()) {
            task_idx_ = tasks_.size();
            return;
        }
        if (cycle_ < tasks_[next].start_cycle)
            return; // fence: wait (retried every cycle from step_tdm_read())
        // flush_hold_ only applies while the lookahead cursor is still
        // parked at THIS task's end (the gated, geometry-changing path,
        // where the buffer's residue is stale and must flush out). If la
        // has already rolled on (seamless same-geometry path), the residue
        // IS the next task's prefetched content — advance immediately and
        // capture it.
        if (flush_hold_ && la_task_idx_ == task_idx_)
            return;
        task_idx_ = next;
        group_    = 0;
        sync_public_fields();
    }

    // TDM read-target only (tdm_window_ > 0): the read buffer's cells
    // prefetch a whole window at once from lookahead_addr()/
    // lookahead_ready() (see their own comments above) — this AGU's job here
    // is just to assert the current group's request and wait for its
    // ports_used lanes to come valid (fast, since the window's initial fill
    // already prefetched the data ahead of time), capture, advance group_,
    // repeat. No window-position bookkeeping needed on this side at all —
    // that's entirely the lookahead cursor's job.
    void step_tdm_read() {
        advance_task_if_ready_tdm_read();

        // BETWEEN tasks (current task's groups all captured, next task still
        // behind its start_cycle fence), keep requesting — at the FINISHED
        // task's own width, with NOP addresses, capturing nothing — instead
        // of dropping req. The buffer can be left holding a partially-drained
        // window here: its cells prefetch ahead of this AGU's capture pace,
        // so the trace's last real window can be followed by residual
        // prefetched content this AGU never asks for. Dropping req strands
        // those cells VALID forever — the buffer then never reaches the
        // all-cells-idle state its boot latch needs, its window geometry
        // stays latched at the OLD task's width, and the NEXT task (if its
        // width differs) meets a half-drained window it can never satisfy: a
        // permanent deadlock, confirmed against production stimuli (doc/
        // report A.6 Cause 2). Holding req through the gap lets the residual
        // window drain at full speed (the phantom rvalids are consumed here
        // without being logged), the drained cells park (their refetch is
        // gated off by fetch_addr_valid_i once the lookahead has nothing
        // real to offer), and the buffer reaches all-idle — so the next
        // task boot-latches cleanly under its own geometry. This is the
        // same hard rule lane_agu.hpp documents for its own TDM targets
        // ("always assert req, real-or-NOP, until done") — agu<>'s read
        // path previously only honored it WITHIN a task.
        // The flush must NOT engage when the lookahead has already rolled
        // into the next task (seamless same-geometry boundary): the
        // buffer's remaining content is then the NEXT task's legitimately
        // prefetched window, not stale residue — flushing it would drain
        // real beats uncaptured and starve the next task. la parked at
        // this task's end (la_task_idx_ == task_idx_) is what identifies
        // genuinely stale residue.
        if (!all_tasks_done() && cycle_ >= cur_task().start_cycle &&
            group_ >= cur_task().n_groups && la_task_idx_ == task_idx_) {
            const int ew = effective_ports_used(); // finished task's own width
            for (int p = 0; p < NUM_REQ; ++p) {
                obi[p].req_o.write(p < ew);
                obi[p].addr_o.write(0);
                obi[p].we_o.write(false);
                obi[p].be_o.write(p < ew ? kBeFull : 0u);
                obi[p].wdata_o.write(data_t(0));
            }
            return;
        }

        // Capture finished this task but la has already rolled on (early
        // roll, la_lead()): park quietly until advance_task_if_ready_
        // tdm_read() moves task_idx_ up. Falling through to the main drive
        // branch here would request at the FINISHED task's width against
        // the NEXT task's prefetched window — draining real beats
        // uncaptured and de-syncing the lookahead stride (the la==task
        // stale case is the flush branch above; this is its rolled-on
        // complement).
        if (all_tasks_done() || cycle_ < cur_task().start_cycle || group_ >= cur_task().n_groups) {
            for (int p = 0; p < NUM_REQ; ++p) {
                obi[p].req_o.write(false);
                obi[p].addr_o.write(0);
                obi[p].we_o.write(false);
                obi[p].be_o.write(0);
                obi[p].wdata_o.write(0);
            }
            return;
        }

        const int ew =
            effective_ports_used(); // rounded up to 1/2/4 port-groups — see its own comment
        for (int p = 0; p < NUM_REQ; ++p) {
            if (p < ew) {
                const bool real = has_row(group_, p); // false for padding lanes (p >= t.ports_used)
                obi[p].req_o.write(true);
                obi[p].addr_o.write(real ? request_addr(group_, p) : uint64_t{0});
                obi[p].we_o.write(false);
                obi[p].be_o.write(kBeFull);
                obi[p].wdata_o.write(data_t(0));
            } else {
                obi[p].req_o.write(false);
                obi[p].addr_o.write(0);
                obi[p].we_o.write(false);
                obi[p].be_o.write(0);
                obi[p].wdata_o.write(0);
            }
        }

        bool all_captured = true;
        for (int p = 0; p < ew; ++p) {
            if (!tdm_cap_done_[p] && obi[p].rvalid_i.read()) {
                if (has_row(group_, p))
                    log_.push_back({cycle_, request_addr(group_, p), false, obi[p].rdata_i.read()});
                tdm_cap_done_[p] = true;
            }
            if (!tdm_cap_done_[p])
                all_captured = false;
        }
        if (all_captured) {
            ++group_;
            for (int p = 0; p < NUM_REQ; ++p)
                tdm_cap_done_[p] = false;
            // NOTE: deliberately NOT rolling task_idx_/group_ into the next
            // task within this same edge (that was tried and reverted — see
            // git history). ports_used_ (and thus the buffer's active_mode,
            // re-driven every cycle by sync_map_cfg()) is derived from
            // task_idx_; advancing task_idx_ even one edge early switches
            // active_mode while the buffer's CURRENT window (still using the
            // OLD width) hasn't actually finished draining, corrupting that
            // window's own is_last/bounds check whenever consecutive tasks
            // use different port counts — a permanent stall, not a one-cycle
            // glitch. The next step()'s advance_task_if_ready_tdm_read()
            // call (top of this function) already handles the transition
            // the following edge, which is what lookahead_addr() (driven
            // independently off the buffer's own window_reset — see
            // advance_lookahead_window()) actually needs.
        }
    }

    void step() {
        if (!rst_ni.read()) {
            reset_state();
            return;
        }

        ++cycle_;
        if (target_ == agu_target::tdm) {
            // TDM read buffers are always windowed — tdm_window_ > 0 is a
            // constructor precondition, enforced below.
            step_tdm_read();
            done_o.write(all_tasks_done());
            return;
        }

        collect_crossbar_responses();
        record_grants();
        advance_group_if_granted();
        advance_task_if_ready();
        drive_requests();
        done_o.write(all_tasks_done() && !has_inflight());
    }

    void end_of_simulation() override {
        if (out_path_.empty())
            return;
        std::ofstream f(out_path_.c_str());
        if (!f) {
            SC_REPORT_WARNING(name(), ("cannot write log: " + out_path_).c_str());
            return;
        }
        f << "cycle,addr,we,data\n";
        for (const access_t &a : log_) {
            f << a.cycle << ",0x" << std::hex << std::setw(8) << std::setfill('0') << a.addr << ","
              << std::dec << (a.we ? 1 : 0) << "," << agu_data_hex(a.data) << "\n";
        }
    }

    agu(sc_core::sc_module_name nm, const std::string &trace_path,
        const std::string &out_path = std::string(), agu_target target = agu_target::crossbar,
        std::size_t tdm_window = 0)
        : sc_module(nm), out_path_(out_path), target_(target), tdm_window_(tdm_window),
          task_idx_(0), group_(0), cycle_(0), ports_used_(NUM_REQ), start_cycle_(0), p_C_(4),
          p_R_(4), p_L_(8), p_store_mode_(0), p_has_crl_(true), n_groups_(0) {
        if (target_ == agu_target::tdm && tdm_window_ == 0)
            SC_REPORT_FATAL(
                name(),
                "agu: target=tdm requires tdm_window > 0 (TDM read buffers are always windowed)");
        for (int p = 0; p < NUM_REQ; ++p)
            granted_[p] = false;
        load_trace(trace_path);
        sync_public_fields();

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
