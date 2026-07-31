// -----------------------------------------------------------------------------
// DMA-specific Address Generation Unit (AGU) testbench driver.
//
// DMA's stimuli format and access pattern are different enough from the
// group-based RAGU_A/B/C/D / WAGU_A/B/D format (see agu.hpp) that this is a
// standalone class rather than an extension of agu<> — see "Why a separate
// class" below. It exposes the SAME external OBI port interface as agu<>
// (clk_i/rst_ni/obi[4] — see obi_ports.hpp's obi_manager_ports — /done_o/
// log_), so it is a drop-in replacement wherever an agu<> instance currently
// drives a RAGU_E/WAGU_E group — nothing here requires touching
// top.hpp/top_tdm.hpp/tb_top.cpp/system_stimuli_common.hpp; a caller only
// needs a small bind_agu() overload (same port shape, different class) if/
// when it wants to use this instead.
//
// -----------------------------------------------------------------------------
// Stimuli format (tb/stimuli/final/{ragu,wagu}_e.log)
// -----------------------------------------------------------------------------
//
// Task descriptor: #cycle,rate,sub_port_id,store_mode,C,R,L  (7 fields)
//   cycle       : earliest start cycle for the address line(s) that follow,
//                 same fencing semantics as agu.hpp's task descriptors.
//   rate        : UNUSED. Verified (by diffing the raw interleaved file) to
//                 just be cycle[k]-cycle[k-1] from the previous descriptor
//                 line — not an address-line count, not a rate to enforce.
//   sub_port_id : 0 or 1. DMA's 4 physical OBI lanes are split into two
//                 independent, asynchronously-progressing sub-ports —
//                 sub-port 0 owns lanes {0,1}, sub-port 1 owns lanes {2,3}.
//                 Descriptors for both sub-ports are interleaved in the same
//                 file; each sub-port's own task queue is built by filtering
//                 on this field and parsed/consumed completely independently
//                 of the other.
//   store_mode,C,R,L : present but not meaningful for DMA (same as agu.hpp's
//                 existing has_crl=false fallback) — never threaded through.
//
// Unlike the initial assumption, each descriptor is followed by a VARIABLE
// number of address lines (1, 2, 20, 42, 66... observed) — parsed "until the
// next '#'", exactly like agu.hpp's existing group parsing, just without
// grouping: every address line is its own independent transfer for that
// sub-port, consumed in order as fast as that sub-port's own lane(s) allow.
//
// Address line format:
//   RAGU (read):  addr,width
//   WAGU (write): addr,data,width
// `width` is in BYTES (not bits), and must be in (0, 2*BYTES_PER_ROW]:
//   width <= BYTES_PER_ROW       : single beat on the sub-port's PRIMARY
//                                  lane only; be_o enables just the low
//                                  `width` bytes of that beat (partial byte
//                                  enable — verified real data: width=8
//                                  writes carry exactly 16 hex chars = 8
//                                  bytes). The secondary lane sits idle
//                                  (NOP) for the whole transfer.
//   width >  BYTES_PER_ROW       : two beats — PRIMARY lane at `addr`,
//                                  always a full beat, and SECONDARY lane at
//                                  `addr + kDmaWideOffset` covering the
//                                  remaining `width - BYTES_PER_ROW` bytes
//                                  (partial byte-enable via be_for_width(),
//                                  full only when width == 2*BYTES_PER_ROW
//                                  exactly). See kDmaWideOffset's comment for
//                                  why this specific offset. For WAGU, the
//                                  data hex string is split the same way:
//                                  the last BYTES_PER_ROW*2 hex chars are the
//                                  primary lane's (full) payload, whatever
//                                  precedes that is the secondary lane's
//                                  (possibly partial) payload.
// Any width outside (0, 2*BYTES_PER_ROW] is rejected (SC_REPORT_FATAL)
// rather than silently mishandled.
//
// -----------------------------------------------------------------------------
// Why the hardware forces "always assert req on all 4 lanes, real-or-NOP" —
// for TDM-buffered targets only (NOT the crossbar backend, see below)
// -----------------------------------------------------------------------------
//
// DMA buffers (buf_r4/buf_w3 in top_tdm.hpp) are PORT_COUNT=1, NUM_REQ=4,
// always active_mode=0 — i.e. exactly one 4-lane group, always. Reading
// buffer.hpp directly: both the write fill stage's `ports_req` and the read
// side's `eval_group().ports_req` are a single AND-gate spanning ALL 4
// physical lanes simultaneously (`for (i=0;i<n_beats;++i) if (!p_req_i[i])
// ports_req=false;`, n_beats==4 always for DMA) — there is no partial/
// per-lane grant. This means:
//   - If EITHER sub-port's lane(s) ever stop asserting req_o while waiting
//     for their own next task, the ENTIRE group's ports_req goes false —
//     stalling BOTH sub-ports (the OTHER sub-port's real, ready work
//     included), not just the idle one.
//   - An EXHAUSTED sub-port (no more tasks) must keep asserting req_o=true
//     with addr_o=0 (NOP — buffer_cell.hpp's addr=0 fast path accepts it
//     instantly, touching nothing real) for the rest of the simulation, or
//     it permanently deadlocks the sibling sub-port too.
// This module's pins therefore ALWAYS assert req=true on all 4 lanes, every
// cycle, for the module's entire lifetime (until its OWN done_o fires) —
// never gated on this sub-port's own readiness — WHENEVER target_==tdm
// (see lane_agu_target below). This is not a stylistic choice for that case;
// violating it deadlocks the buffer. See drive_crossbar_requests()/
// step_read().
//
// The crossbar backend has NO such shared group at all — each of the 4
// lanes is routed and arbitrated completely independently, per bank (see
// crossbar.hpp). There, asserting req=true forever on an idle/exhausted
// lane is not just unnecessary but actively harmful: an addr=0 NOP still
// competes for whichever bank addr 0 maps to, creating permanent phantom
// contention that can starve real traffic from OTHER groups sharing that
// bank (found via a stimuli set with no DMA activity at all timing out once
// this was made unconditional). So under target_==crossbar, an idle lane
// goes fully idle (req=false), matching agu<>'s existing crossbar-target
// behavior — see drive_crossbar_requests()'s `must_hold_req` gate.
//
// On the read side there's one more subtlety, from buffer_cell.hpp's
// read-mode fetch engine: `addr_i` is latched only once, the instant a
// cell is idle (i.e. right after a window reset) — changing addr_o mid-window
// has zero effect until the NEXT reset. So each lane's target must be frozen
// for a whole window's lifetime, decided once at window-start
// (latch_window_targets()), not re-polled mid-drain; a sub-port's task
// becoming ready mid-window is simply picked up at the start of the next
// window (acceptable — DMA reads aren't latency-sensitive within one
// window's ~tdm_window-cycle drain). A window packs up to
// tdm_window/NUM_REQ consecutive ready tasks per sub-port (8 per lane on
// the production 32-cell buffer), so a deep task queue keeps every window
// slot real rather than one task per sub-port per window.
//
// -----------------------------------------------------------------------------
// Why a separate class instead of extending agu<>
// -----------------------------------------------------------------------------
//
// agu<>'s task_t/parsing assumes ONE linear task list advancing group-by-
// group in lockstep across all active lanes. DMA needs TWO independent task
// queues (one per sub-port) that can be at completely different points in
// their own trace at any given moment, plus width-driven variable lane
// usage and a from-scratch hex/data parser (agu<>::parse_addr_line() calls
// strtoull() on the whole data string, which SILENTLY TRUNCATES a
// width=32 transfer's 64-hex-char/256-bit payload — confirmed by reading
// agu.hpp; this alone rules out reusing that parser). Shoehorning both
// models into one class would make both harder to follow.
// -----------------------------------------------------------------------------

#ifndef DMA_AGU_HPP
#define DMA_AGU_HPP

#include <systemc.h>

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "csv_parse_util.hpp"
#include "obi_data.hpp"
#include "obi_ports.hpp"

enum class lane_agu_dir { read, write };

// Mirrors agu.hpp's agu_target — says whether the OTHER side of this OBI
// link is a TDM buffer (windowed, shared-group) or plain crossbar
// (independent per-lane arbitration). Matters for BOTH directions:
//   crossbar: one-shot request/grant/response per lane, independent progress
//             — the crossbar backend has no DMA-specific buffering at all.
//             Idle lanes go fully quiet (req=false); see
//             drive_crossbar_requests()'s `must_hold_req` gate.
//   tdm:      the buffer on the other end (buf_r4 for reads, buf_w3 for
//             writes) requires every lane to assert req=true real-or-NOP
//             forever, or its shared 4-lane group deadlocks — see the
//             class-level "always assert req" comment above. Reads
//             additionally go through step_read()'s windowed drain/capture/
//             drain-remainder protocol, which REQUIRES tdm_window > 0.
// Both directions use the SAME crossbar-style request/grant/response
// machinery (drive_crossbar_requests() etc.) for target_==crossbar and for
// ALL writes; only dir_==read && target_==tdm takes the windowed step_read()
// path instead.
enum class lane_agu_target { crossbar, tdm };

template <typename DATA_T = uint64_t, int BYTES_PER_BEAT = 16> SC_MODULE(lane_agu) {
    using data_t = DATA_T;

    static constexpr int NUM_REQ           = 4; // DMA is always exactly 4 physical lanes
    static constexpr int NUM_SUBPORT       = 2; // sub_port_id in {0,1}
    static constexpr int LANES_PER_SUBPORT = NUM_REQ / NUM_SUBPORT; // 2

    static constexpr uint32_t kBeFull = (BYTES_PER_BEAT >= 32) ? ~0u : ((1u << BYTES_PER_BEAT) - 1);

    // Address offset for a width>BYTES_PER_BEAT ("wide") transfer's second
    // beat, relative to the address given in the trace. Verified against the
    // FULL real trace (tb/stimuli/final/wagu_e.log): every exactly-wide
    // (width=32) entry is packed exactly 0x20 = 2*BYTES_PER_BEAT bytes apart
    // from the next one with zero gaps/exceptions (e.g. 0x254c0 -> 0x254e0 ->
    // 0x25500, each delta 0x20) — so `addr + BYTES_PER_BEAT` lands exactly at
    // the midpoint before the next real transfer begins, with no risk of
    // aliasing real data. This offset is correct for ANY width > BYTES_PER_BEAT,
    // not just exactly 2*BYTES_PER_BEAT: the primary lane is always a full
    // beat, so the remaining bytes always start right after it regardless of
    // how many of them are real — a PARTIAL second beat only narrows the
    // byte-enable mask (be_for_width(width-BYTES_PER_BEAT)), it never moves
    // where that beat starts, so there's no aliasing risk to re-verify.
    static constexpr uint64_t kDmaWideOffset = static_cast<uint64_t>(BYTES_PER_BEAT);

    SC_HAS_PROCESS(lane_agu);

    // ---- External OBI port interface — matches agu<>'s port list exactly ----
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

    // Public fields mirroring agu<>'s, so any caller reading them (e.g.
    // tb_top.cpp's active_mode/mapping-geometry code) keeps working
    // unchanged if this is substituted for an agu<> instance. DMA always
    // uses all 4 lanes as one group and carries no real CRL geometry.
    int         ports_used_ = NUM_REQ;
    bool        p_has_crl_  = false;
    uint64_t    p_C_ = 4, p_R_ = 4, p_L_ = 8, p_store_mode_ = 0;
    std::size_t n_groups_ = 0;

    // -------------------------------------------------------------------
    // Per-sub-port task representation and state
    // -------------------------------------------------------------------
    struct dma_task_t {
        uint64_t start_cycle = 0;
        uint64_t addr        = 0;
        int      width       = 0; // bytes; in (0, 2*BYTES_PER_BEAT]
        bool     we          = false;
        data_t   data_lo{}; // primary-lane payload (write only)
        data_t   data_hi{}; // secondary-lane payload (write, wide only)
    };

    struct subport_state_t {
        std::deque<dma_task_t> tasks;
        std::size_t            idx = 0;
        bool                   done() const { return idx >= tasks.size(); }
    };
    std::array<subport_state_t, NUM_SUBPORT> subports_;

    bool all_subports_done() const {
        for (const auto &s : subports_)
            if (!s.done())
                return false;
        return true;
    }

    // What's currently being presented on each of the 4 lanes — either a
    // real (sub-)transfer, or a NOP (addr=0) filling in for a lane whose
    // sub-port has nothing ready right now. Same struct used by both the
    // write path (content_, re-decided each time the prior content is
    // accepted) and the read path (window_win_, a full window's groups
    // frozen at latch time — see the class-level comment on latch timing).
    struct lane_content_t {
        bool     real = false;
        uint64_t addr = 0;
        data_t   data{};
        int      subport = -1;
    };

    // ---- Crossbar-style path state ----
    lane_content_t content_[NUM_REQ];
    // Per-SUB-PORT "currently presenting a task, not yet fully accepted"
    // flag, plus a per-LANE latch for whether THIS lane's own current
    // request has already been granted. Two independent lanes (and, for a
    // sub-port's own primary/secondary on a wide transfer, two lanes within
    // the SAME sub-port) can be granted on completely different cycles under
    // the crossbar backend, where each lane is arbitrated independently
    // per-bank (see crossbar.hpp) — unlike the TDM write buffer, where all 4
    // physical lanes are accepted together in one atomic fill beat. Tracking
    // a single flag across all 4 lanes (as an earlier version of this code
    // did) silently treated a lane as "accepted" whenever ANY other lane got
    // its own grant, abandoning the real in-flight request before the bank
    // ever saw it while still logging it as a success — a real, hard-to-spot
    // corruption bug found via `final`'s real trace data under contention.
    bool sp_active_[NUM_SUBPORT]                          = {};
    bool sp_lane_granted_[NUM_SUBPORT][LANES_PER_SUBPORT] = {};

    // SEL_DESC_SYNC (tb_top, crossbar only): when true, start_cycle fences are a
    // no-op so DMA runs free/back-to-back as background traffic (DMA is excluded
    // from the descriptor barrier). Defaults false => zero behavior change.
    bool ignore_fence_ = false;
    struct lane_rec_t {
        uint64_t addr;
        bool     we;
        data_t   data;
    };
    std::deque<lane_rec_t> lane_inflight_[NUM_REQ]; // grant->rvalid, mirrors agu.hpp

    // ---- Read-side state (generalizes agu.hpp::step_tdm_read() to 2
    //      independent sub-ports sharing one physical 4-lane window) ----
    // A window packs up to tdm_window_/NUM_REQ CONSECUTIVE ready tasks per
    // sub-port — on the production 32-cell buffer that is 8 tasks per
    // sub-port, i.e. all 32 window slots real when both queues are deep
    // enough (8 beats per physical lane per window, matching agu.hpp's
    // fully-packed windows). An earlier version froze ONE task per sub-port
    // per window (4 real beats, 7 groups of structural NOP padding), which
    // capped an E stream at 4 beats per ~window-drain — a ~4x driver-side
    // throughput ceiling that showed up as the DMA read outlier in the
    // stim_bank phase-8 spans, not anything in the RTL. The window's groups
    // are walked in drain order: each group's 4 lanes are presented while
    // its own rvalid pulse is awaited; a group whose sub-port had nothing
    // ready when the window was decided is a NOP group and drains as
    // padding exactly like before (its rvalid pulse still paces the walk,
    // which is what the old DRAIN_REMAINDER phase existed to do).
    std::vector<lane_content_t> window_win_;            // full window, frozen at latch
    std::size_t                 win_grp_           = 0; // group being drained/captured
    bool                        window_valid_      = false;
    bool                        cap_done_[NUM_REQ] = {};

    lane_agu_dir    dir_;
    lane_agu_target target_ =
        lane_agu_target::tdm; // gates must_hold_req/step_read() — see enum comment
    std::size_t tdm_window_;  // NUM_BANK; required >0 for dir_==read && target_==tdm
    uint64_t    cycle_ = 0;
    std::string out_path_;

    // -------------------------------------------------------------------
    // Parsing helpers — trim()/split_csv()/parse_hex_u64()/strip_0x() come
    // from csv_parse_util.hpp (shared with agu.hpp).
    // -------------------------------------------------------------------

    // Builds a data_t from a (possibly short) hex digit string — zero-extends
    // naturally via sc_bv's string constructor, which is why callers must
    // NOT go through strtoull (that silently truncates anything over 64
    // bits, e.g. a width=32 transfer's 256-bit payload).
    static data_t data_from_hex_digits(const std::string &hex_digits) {
        return data_t(("0x" + hex_digits).c_str());
    }

    // #cycle,rate,sub_port_id,store_mode,C,R,L — rate/store_mode/C/R/L are
    // parsed only to validate field count; they carry no meaning for DMA
    // (see the class-level format comment).
    static int parse_descriptor(const std::string &line, uint64_t &cycle_out) {
        const std::vector<std::string> f = split_csv(line.substr(1)); // skip '#'
        if (f.size() != 7)
            SC_REPORT_FATAL("lane_agu", "DMA task descriptor needs exactly 7 fields: "
                                        "#cycle,rate,sub_port_id,store_mode,C,R,L");
        cycle_out          = parse_hex_u64(f[0]);
        const int sub_port = static_cast<int>(parse_hex_u64(f[2]));
        if (sub_port != 0 && sub_port != 1)
            SC_REPORT_FATAL("lane_agu", "sub_port_id must be 0 or 1");
        return sub_port;
    }

    // SEL_NO_FENCE: same harness knob as agu.hpp — drop start_cycle fences
    // so tasks run back-to-back (throughput-bound instead of schedule-bound).
    static bool no_fence() {
        static const bool v = std::getenv("SEL_NO_FENCE") != nullptr;
        return v;
    }

    // Parses one address line into a task, given the fence cycle and
    // direction (RAGU: "addr,width" — WAGU: "addr,data,width").
    dma_task_t parse_addr_line(const std::string &line, uint64_t start_cycle) const {
        const std::vector<std::string> f = split_csv(line);
        dma_task_t                     t;
        t.start_cycle = start_cycle;
        t.we          = (dir_ == lane_agu_dir::write);

        if (dir_ == lane_agu_dir::read) {
            if (f.size() != 2)
                SC_REPORT_FATAL("lane_agu", "RAGU_E address line needs: addr,width");
            t.addr  = parse_hex_u64(f[0]);
            t.width = static_cast<int>(parse_hex_u64(f[1]));
        } else {
            if (f.size() != 3)
                SC_REPORT_FATAL("lane_agu", "WAGU_E address line needs: addr,data,width");
            t.addr                = parse_hex_u64(f[0]);
            t.width               = static_cast<int>(parse_hex_u64(f[2]));
            const std::string hex = strip_0x(f[1]);
            if (t.width > BYTES_PER_BEAT) {
                if (t.width > 2 * BYTES_PER_BEAT)
                    SC_REPORT_FATAL("lane_agu", "WAGU_E width must be in (0, 2*BYTES_PER_BEAT]");
                if (hex.size() != static_cast<std::size_t>(t.width) * 2)
                    SC_REPORT_FATAL("lane_agu",
                                    "WAGU_E data length does not match width*2 hex chars");
                // Primary lane is always a full beat, so the data string's
                // LAST BYTES_PER_BEAT*2 chars are data_lo; whatever precedes
                // that (width-BYTES_PER_BEAT bytes, partial unless width==
                // 2*BYTES_PER_BEAT) is data_hi for the secondary lane.
                // data_from_hex_digits zero-extends a short string, so a
                // partial data_hi lands in its own low bytes — matching
                // be_for_width(width-BYTES_PER_BEAT)'s low-byte enable mask.
                const std::size_t lo_chars = static_cast<std::size_t>(BYTES_PER_BEAT) * 2;
                const std::size_t hi_chars = hex.size() - lo_chars;
                t.data_hi                  = data_from_hex_digits(hex.substr(0, hi_chars));
                t.data_lo                  = data_from_hex_digits(hex.substr(hi_chars, lo_chars));
            } else if (t.width > 0 && t.width <= BYTES_PER_BEAT) {
                t.data_lo = data_from_hex_digits(hex);
            } else {
                SC_REPORT_FATAL("lane_agu", "WAGU_E width must be in (0, 2*BYTES_PER_BEAT]");
            }
        }
        if (dir_ == lane_agu_dir::read && !(t.width > 0 && t.width <= 2 * BYTES_PER_BEAT))
            SC_REPORT_FATAL("lane_agu", "RAGU_E width must be in (0, 2*BYTES_PER_BEAT]");
        return t;
    }

    void load_trace(const std::string &path_in) {
        const std::string path = resolve_stim_path(path_in);
        std::ifstream     f(path.c_str());
        if (!f) {
            SC_REPORT_INFO(name(), ("no stimuli (" + path + "), will be idle").c_str());
            return;
        }
        std::vector<std::string> lines;
        std::string              ln;
        while (std::getline(f, ln)) {
            ln = trim(ln);
            if (!ln.empty())
                lines.push_back(ln);
        }

        int      cur_sub   = -1;
        uint64_t cur_cycle = 0;
        for (const auto &l : lines) {
            if (l[0] == '#') {
                cur_sub = parse_descriptor(l, cur_cycle);
            } else {
                if (cur_sub < 0)
                    SC_REPORT_FATAL(name(), "address line before first '#' descriptor");
                subports_[cur_sub].tasks.push_back(parse_addr_line(l, cur_cycle));
            }
        }
        // SEL_NO_FENCE: keep only each sub-port's initial fence; later
        // tasks run back-to-back (see agu.hpp's identical harness knob).
        if (no_fence())
            for (auto &sp : subports_)
                for (std::size_t i = 1; i < sp.tasks.size(); ++i)
                    sp.tasks[i].start_cycle = 0;
        for (const auto &s : subports_)
            n_groups_ += s.tasks.size();
    }

    // -------------------------------------------------------------------
    // Crossbar-style path — writes (always), and reads when
    // target_==lane_agu_target::crossbar. One-shot request/grant/response per
    // lane, each sub-port progressing independently via lane_inflight_; no
    // window state machine (see step_read() below for the TDM-windowed read
    // path, used only when dir_==read && target_==tdm).
    // -------------------------------------------------------------------
    bool has_inflight() const {
        for (const auto &q : lane_inflight_)
            if (!q.empty())
                return true;
        return false;
    }

    // Decides what ONE sub-port's own 2 lanes should present: its current
    // task if one is ready now (fence passed), else NOP on both lanes. See
    // the class-level comment for why NOP lanes are mandatory whenever a
    // sub-port has nothing ready. Only called for a sub-port that isn't
    // already mid-task (sp_active_[sp]==false) — see drive_crossbar_requests().
    void decide_content_for(int sp) {
        auto      &s        = subports_[sp];
        const bool ready    = !s.done() && (ignore_fence_ || cycle_ >= s.tasks[s.idx].start_cycle);
        const dma_task_t *t = ready ? &s.tasks[s.idx] : nullptr;
        const int         primary   = sp * LANES_PER_SUBPORT;
        const int         secondary = primary + 1;

        if (t) {
            content_[primary] = {true, t->addr, t->data_lo, sp};
            if (t->width > BYTES_PER_BEAT)
                content_[secondary] = {true, t->addr + kDmaWideOffset, t->data_hi, sp};
            else
                content_[secondary] = {false, 0, data_t{}, sp};
            sp_active_[sp] = true;
        } else {
            content_[primary]   = {false, 0, data_t{}, sp};
            content_[secondary] = {false, 0, data_t{}, sp};
            sp_active_[sp]      = false; // nothing ready yet — retry next cycle
        }
        sp_lane_granted_[sp][0] = false;
        sp_lane_granted_[sp][1] = false;
    }

    // Independent-readiness-check fallback for latch_window_targets() when
    // window_queue_ is empty — see that function's own comment for when
    // this applies.
    void decide_content() {
        for (int sp = 0; sp < NUM_SUBPORT; ++sp)
            decide_content_for(sp);
    }

    static uint32_t be_for_width(int width) {
        if (width >= BYTES_PER_BEAT)
            return kBeFull;
        return (width > 0) ? ((1u << width) - 1u) : 0u;
    }

    // Presents each of the 4 lanes independently: a lane whose own request
    // has already been granted (sp_lane_granted_) goes idle/NOP until its
    // sub-port's OTHER lane (if any) also completes and the whole task can
    // advance — see record_crossbar_grants(). This is what makes the two
    // lanes of a wide transfer, and the two independent sub-ports, safe
    // under the crossbar backend's independent per-bank arbitration (see
    // sp_active_/sp_lane_granted_'s class-level comment).
    //
    // req_o's "always assert real-or-NOP" rule (class-level comment) is a
    // TDM WRITE BUFFER requirement (its ports_req AND-gate spans all 4
    // lanes) — NOT a crossbar one. Under target_==crossbar there is no
    // shared group to keep alive, so a lane with nothing pending goes fully
    // idle (req=false), matching agu<>'s existing crossbar-target behavior.
    // Asserting req=true forever there was actively harmful: it created
    // permanent phantom contention for whichever bank addr=0 routes to,
    // starving real traffic from OTHER groups sharing that bank (found via
    // `tiny`'s crossbar run, which has no DMA stimuli at all and should be
    // fully idle, timing out instead once this was unconditional).
    void drive_crossbar_requests() {
        const bool is_write      = (dir_ == lane_agu_dir::write);
        const bool must_hold_req = (target_ == lane_agu_target::tdm);
        for (int sp = 0; sp < NUM_SUBPORT; ++sp)
            if (!sp_active_[sp])
                decide_content_for(sp);

        for (int p = 0; p < NUM_REQ; ++p) {
            const int   sp       = p / LANES_PER_SUBPORT;
            const int   lane_idx = p % LANES_PER_SUBPORT;
            const auto &c        = content_[p];
            const bool  pending  = c.real && !sp_lane_granted_[sp][lane_idx];
            obi[p].req_o.write(pending || must_hold_req);
            obi[p].addr_o.write(pending ? c.addr : uint64_t{0});
            obi[p].we_o.write(pending && is_write);
            // Width only matters for a write's byte-enable — the primary
            // lane's own width (saturates to full at BYTES_PER_BEAT via
            // be_for_width) or, for a >BYTES_PER_BEAT transfer, the
            // secondary lane's REMAINING width (partial unless the transfer
            // is exactly 2*BYTES_PER_BEAT). Reads always use full
            // byte-enable regardless of width.
            const bool  primary = (lane_idx == 0);
            const auto &s       = subports_[sp];
            uint32_t    be      = kBeFull;
            if (pending && is_write && !s.done()) {
                const int w = s.tasks[s.idx].width;
                be = primary ? be_for_width(w) : be_for_width(std::max(0, w - BYTES_PER_BEAT));
            }
            obi[p].be_o.write(pending ? be : uint32_t{0});
            obi[p].wdata_o.write(pending && is_write ? c.data : data_t{});
        }
    }

    void record_crossbar_grants() {
        const bool is_write = (dir_ == lane_agu_dir::write);
        for (int p = 0; p < NUM_REQ; ++p) {
            const int sp       = p / LANES_PER_SUBPORT;
            const int lane_idx = p % LANES_PER_SUBPORT;
            if (content_[p].real && !sp_lane_granted_[sp][lane_idx] && obi[p].gnt_i.read()) {
                lane_inflight_[p].push_back({content_[p].addr, is_write, content_[p].data});
                sp_lane_granted_[sp][lane_idx] = true;
            }
        }
        // A sub-port's task is complete once every lane it actually used
        // (primary always; secondary too for a wide transfer) has its OWN
        // grant recorded — NOP lanes (content_[p].real==false) trivially
        // count as already satisfied.
        for (int sp = 0; sp < NUM_SUBPORT; ++sp) {
            if (!sp_active_[sp])
                continue;
            const int  primary      = sp * LANES_PER_SUBPORT;
            const int  secondary    = primary + 1;
            const bool primary_ok   = !content_[primary].real || sp_lane_granted_[sp][0];
            const bool secondary_ok = !content_[secondary].real || sp_lane_granted_[sp][1];
            if (primary_ok && secondary_ok) {
                ++subports_[sp].idx;
                sp_active_[sp] = false; // next drive_crossbar_requests() decides the next task
            }
        }
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

    // -------------------------------------------------------------------
    // Read-side — see class-level comment for why targets must be latched
    // once per window rather than re-polled mid-drain.
    //
    // window_win_ is filled from window_queue_ (pushed by
    // advance_lookahead_window(), one entry per window it decided) rather
    // than re-deciding readiness independently here. Both this function and
    // fill_subport_window() check the SAME fence (cycle_ >= start_cycle)
    // on the SAME sub-port task, but at DIFFERENT edges — la_'s fires
    // exactly when the buffer's own window_reset does, while this one only
    // fires once this module's own (registered-rvalid-lagged) window_valid_
    // catches up. When a sub-port's fence clears while BOTH cursors happen
    // to be waiting on the very same task (idx unchanged from before), each
    // independently re-checking "is it ready NOW" can answer true on two
    // DIFFERENT physical windows — this module's own catches up late and
    // ends up logging the FETCH side's real address against the STALE
    // (still-NOP) data the cell was actually holding. Queuing instead means
    // whatever window's content la_ decided is the SAME content this
    // function later attributes to that window, however many edges later
    // it catches up.
    void latch_window_targets() {
        if (window_queue_.empty()) {
            // No caller has been driving this instance's lookahead cursor
            // via advance_lookahead_window() (e.g. a RAGU_E/WAGU_E
            // group a given harness never wires up for real traffic, left
            // as a mostly-NOP placeholder) — fall back to deciding
            // readiness directly from the capture-side sub-port cursors
            // (which THIS path's own group walk advances), matching this
            // module's behavior before the lookahead queue existed. Safe
            // precisely because such an instance has no buffer-side
            // prefetch staged to race against.
            window_win_.assign(groups_per_window() * NUM_REQ, lane_content_t{});
            for (int sp = 0; sp < NUM_SUBPORT; ++sp)
                fill_subport_window(window_win_, sp, subports_[sp].idx);
        } else {
            window_win_ = window_queue_.front();
            window_queue_.pop_front();
        }
        win_grp_ = 0;
        for (int p = 0; p < NUM_REQ; ++p)
            cap_done_[p] = false;
    }

    // -------------------------------------------------------------------
    // Lookahead cursor (read + target_==tdm only) — mirrors agu.hpp's
    // la_task_idx_/la_group_: subports_[sp].idx only advances once
    // step_read()'s CAPTURE phase confirms a window's real lanes were read
    // back, which lags the buffer's own window-reset pace (the buffer
    // resets and starts refetching as soon as its cells drain, independent
    // of how fast THIS module happens to notice via registered rvalid_i).
    // la_idx_ instead advances directly off the buffer's own observed
    // window_reset (see advance_lookahead_window(), called by the harness),
    // so fetch_addr_i can be staged with the NEXT window's targets ahead of
    // time — the same fix agu.hpp's lookahead_addr()/advance_lookahead_
    // window() apply for RAGU_A/B/C/D.
    //
    // Unlike agu.hpp's single task queue, an idle/not-yet-fenced sub-port
    // has a well-defined answer (NOP on both its lanes) rather than an
    // ambiguous "don't know yet" state, so there's no lookahead_ready()
    // gate to add here — fill_subport_window() always produces valid
    // content, real or NOP, for whatever cycle_ currently is.
    std::array<std::size_t, NUM_SUBPORT> la_idx_{};
    std::vector<lane_content_t>          la_win_; // full window's staged content

    // Every window's content, in fetch order — pushed once per
    // advance_lookahead_window() call, popped once per
    // latch_window_targets() call (see its own comment for why this queue
    // exists instead of window_win_ being independently re-decided).
    std::deque<std::vector<lane_content_t>> window_queue_;

    std::size_t groups_per_window() const {
        return tdm_window_ >= static_cast<std::size_t>(NUM_REQ)
                   ? tdm_window_ / static_cast<std::size_t>(NUM_REQ)
                   : 1;
    }

    // Packs sub-port sp's CONSECUTIVE ready tasks, starting at from_idx,
    // into `win`'s groups — group g's slots [g*NUM_REQ + sp*LANES_PER_
    // SUBPORT (+1)] — stopping at the first not-yet-ready task (task order
    // is preserved; a fenced task never lets a later one jump the queue,
    // it just leaves the window's remaining groups as NOPs for this
    // sub-port). Shared by the lookahead decide (from la_idx_) and
    // latch_window_targets()'s self-contained fallback (from the capture-
    // side subports_[sp].idx).
    void fill_subport_window(std::vector<lane_content_t> & win, int sp, std::size_t from_idx) {
        const auto       &s      = subports_[sp];
        const std::size_t groups = win.size() / static_cast<std::size_t>(NUM_REQ);
        for (std::size_t g = 0; g < groups; ++g) {
            const std::size_t idx = from_idx + g;
            if (idx >= s.tasks.size() || cycle_ < s.tasks[idx].start_cycle)
                break; // remaining groups stay NOP for this sub-port
            const dma_task_t &t         = s.tasks[idx];
            const std::size_t primary   = g * NUM_REQ + sp * LANES_PER_SUBPORT;
            const std::size_t secondary = primary + 1;
            win[primary]                = {true, t.addr, t.data_lo, sp};
            if (t.width > BYTES_PER_BEAT)
                win[secondary] = {true, t.addr + kDmaWideOffset, t.data_hi, sp};
        }
    }

    void decide_la_content() {
        la_win_.assign(groups_per_window() * NUM_REQ, lane_content_t{});
        for (int sp = 0; sp < NUM_SUBPORT; ++sp)
            fill_subport_window(la_win_, sp, la_idx_[sp]);
    }

    // Pushes the CURRENT la_win_ onto window_queue_ for latch_window_
    // targets() to pop later — see window_queue_'s own comment.
    void push_la_window() {
        window_queue_.push_back(la_win_);
    }

    // Called once per observed buffer window_reset pulse, BEFORE the next
    // edge — the OUTGOING window (whatever la_win_ currently holds, from
    // the last call) is what's about to be consumed, so advance each
    // sub-port past every real task it carried, then decide the NEW
    // window's content.
    void advance_lookahead_window() {
        for (int sp = 0; sp < NUM_SUBPORT; ++sp) {
            std::size_t consumed = 0;
            for (std::size_t g = 0; g * NUM_REQ < la_win_.size(); ++g)
                if (la_win_[g * NUM_REQ + sp * LANES_PER_SUBPORT].real)
                    ++consumed;
            la_idx_[sp] += consumed;
        }
        decide_la_content();
        push_la_window();
    }

    // TDM read-target only: the w-th window slot's address for whichever
    // window la_idx_ currently points at — zero (NOP) for w outside the
    // window or a slot with nothing real, matching every other NOP-padding
    // convention in this file. A plain accessor, not an sc_out port — see
    // agu.hpp's lookahead_addr() comment for why (this module is a drop-in
    // replacement for agu<> and shouldn't need every caller's port list
    // touched to gain a feature only the DMA read buffer uses).
    uint64_t lookahead_addr(int w) const {
        if (w < 0 || static_cast<std::size_t>(w) >= la_win_.size())
            return 0;
        return la_win_[static_cast<std::size_t>(w)].real ? la_win_[static_cast<std::size_t>(w)].addr
                                                         : 0;
    }

    // The read buffer's cells prefetch a whole window at once from
    // lookahead_addr() (see its own comment above) — CAPTURE below only
    // needs to wait for the first (real) group's own rvalid, now that
    // lookahead has already staged this window's content ahead of the
    // reset — but DRAIN_REMAINDER is still required to skip the structural
    // NOP group(s) padding out the rest of the window before the next
    // window's content can safely be decided (see this module's own
    // class-level comment on why, unlike agu.hpp, this can't just be
    // dropped).
    //
    // Formerly a known bug (tb_lane_agu's T03b/T04b): one task's own (addr,
    // data) pair was silently dropped and every subsequent logged entry
    // shifted by one task, permanently, first observed right at/before the
    // one wide (2-lane) task in the stream. Root-caused via direct tracing to
    // buffer_cell.hpp's own "safe" condition (`!valid || all_valid_i`): the
    // bare !valid escape let an already-drained, already-primed cell commit
    // its NEXT fetch's value the instant that fetch resolved, without
    // waiting for its own group to actually drain (all_valid_i) — fine for a
    // cell's first-ever (never-primed) commit, but for a structurally-always-
    // NOP lane (e.g. a narrow task's unused secondary lane, sharing this
    // group with a slower REAL lane — see LANES_PER_SUBPORT) it meant the NOP
    // lane's own next value got swept into the PRECEDING task's drain (both
    // read back as 0 regardless, so numerically invisible there), leaving it
    // !valid again by the time the group was revisited for ITS OWN task —
    // forcing the group to wait on a fresh re-fetch while the slower real
    // lane's already-committed data sat unread until the FOLLOWING task's own
    // fetch silently superseded it. Fixed in buffer_cell.hpp: safe's !valid
    // escape now applies only pre-primed (the genuine boot case); once
    // primed, a cell always waits for its own group's all_valid_i, even if
    // presently !valid, so a fast NOP sibling can no longer race ahead of a
    // slower real lane in the same group. (The equivalent combinational
    // is_fwd path in comb_proc_read() was deliberately NOT changed to match —
    // doing so creates a real combinational cycle, all_valid_i -> can_drain
    // -> cells_valid -> is_fwd -> all_valid_i, and caused an actual deadlock
    // in testing; is_fwd's laxer escape is harmless since the parent's own
    // cells_valid still waits on every other cell in the group regardless.)
    //
    // The once-"remaining" gap (a real beat reading back 0 at a stream
    // boundary — historically seen on a wide task's secondary lane, later on
    // the final window's group-1 beat) is now root-caused and FIXED in the
    // RTL: the parent buffer's one-cycle post-drain echo pulse used to share
    // the cell's all_valid_i wire, where it could (a) START a fetch that
    // latched whatever the lookahead bus held one cycle past the window
    // hand-off — a NOP at end-of-stream, whose staged zero then superseded
    // the real committed beat at its own drain via is_fwd precedence — and
    // (b) CLEAR a beat committed at the drain edge but not yet delivered.
    // The echo now arrives on its own commit_ok_i wire and can only unlock a
    // LATE COMMIT or clean up an already-delivered forward-and-commit; it
    // never starts a fetch (see buffer_cell.hpp's commit_ok_i and the clear
    // branch's comment). tb_lane_agu's T03b/T04b verify the exact
    // (addr,data) sets end to end.
    void step_read() {
        if (!window_valid_) {
            latch_window_targets();
            window_valid_ = true;
        }
        // Present the CURRENT group's 4 lanes and wait for that group's own
        // rvalid pulse; each pulse (real content or NOP padding alike)
        // advances the walk one group, so the walk stays in lockstep with
        // the buffer's own drain pointer whether or not a group carried
        // anything real — the same pacing the old CAPTURE+DRAIN_REMAINDER
        // pair provided for the one-real-group window shape.
        const std::size_t base = win_grp_ * static_cast<std::size_t>(NUM_REQ);
        for (int p = 0; p < NUM_REQ; ++p) {
            const auto &c = window_win_[base + p];
            obi[p].req_o.write(true); // ALWAYS asserted — see class-level comment
            obi[p].addr_o.write(c.real ? c.addr : uint64_t{0});
            obi[p].we_o.write(false);
            obi[p].be_o.write(kBeFull);
            obi[p].wdata_o.write(data_t{});
        }

        bool all_captured = true;
        for (int p = 0; p < NUM_REQ; ++p) {
            if (!cap_done_[p] && obi[p].rvalid_i.read()) {
                const auto &c = window_win_[base + p];
                if (c.real)
                    log_.push_back({cycle_, c.addr, false, obi[p].rdata_i.read()});
                cap_done_[p] = true;
            }
            if (!cap_done_[p])
                all_captured = false;
        }
        if (all_captured) {
            for (int sp = 0; sp < NUM_SUBPORT; ++sp)
                if (window_win_[base + sp * LANES_PER_SUBPORT].real)
                    ++subports_[sp].idx;
            ++win_grp_;
            for (int p = 0; p < NUM_REQ; ++p)
                cap_done_[p] = false;
            if (win_grp_ * static_cast<std::size_t>(NUM_REQ) >= window_win_.size())
                window_valid_ = false; // next step_read() latches a fresh window
        }
    }

    // -------------------------------------------------------------------
    // Top-level step / reset
    // -------------------------------------------------------------------
    void reset_state() {
        for (int p = 0; p < NUM_REQ; ++p) {
            obi[p].req_o.write(false);
            obi[p].addr_o.write(0);
            obi[p].we_o.write(false);
            obi[p].be_o.write(0);
            obi[p].wdata_o.write(data_t{});
            lane_inflight_[p].clear();
            cap_done_[p] = false;
        }
        for (auto &s : subports_)
            s.idx = 0;
        for (int sp = 0; sp < NUM_SUBPORT; ++sp) {
            sp_active_[sp]          = false;
            sp_lane_granted_[sp][0] = false;
            sp_lane_granted_[sp][1] = false;
        }
        window_valid_ = false;
        win_grp_      = 0;
        window_win_.assign(groups_per_window() * NUM_REQ, lane_content_t{});
        cycle_ = 0;
        done_o.write(false);
        log_.clear();
        la_idx_.fill(0);
        window_queue_.clear();
        decide_la_content();
        push_la_window(); // the very first window's content
    }

    void step() {
        if (!rst_ni.read()) {
            reset_state();
            return;
        }
        ++cycle_;
        if (dir_ == lane_agu_dir::read && target_ == lane_agu_target::tdm) {
            step_read();
            done_o.write(all_subports_done());
        } else {
            collect_crossbar_responses();
            record_crossbar_grants();
            drive_crossbar_requests();
            done_o.write(all_subports_done() && !has_inflight());
        }
    }

    void end_of_simulation() override {
        if (out_path_.empty())
            return;
        std::ofstream f(out_path_.c_str());
        if (!f)
            return;
        f << "cycle,addr,we,data\n";
        for (const auto &a : log_) {
            std::ostringstream ds;
            ds << a.data;
            f << a.cycle << ",0x" << std::hex << std::setw(8) << std::setfill('0') << a.addr << ","
              << std::dec << (a.we ? 1 : 0) << "," << ds.str() << "\n";
        }
    }

    lane_agu(sc_core::sc_module_name nm, const std::string &trace_path, const std::string &out_path,
             lane_agu_dir dir, std::size_t tdm_window,
             lane_agu_target target = lane_agu_target::tdm)
        : sc_module(nm), dir_(dir), target_(target), tdm_window_(tdm_window), out_path_(out_path) {
        if (dir_ == lane_agu_dir::read && target_ == lane_agu_target::tdm && tdm_window_ == 0)
            SC_REPORT_FATAL(
                name(),
                "lane_agu: read direction with target=tdm requires tdm_window > 0 (TDM read "
                "buffers are always windowed)");
        load_trace(trace_path);

        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif // DMA_AGU_HPP
