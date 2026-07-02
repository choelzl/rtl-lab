// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Prefetch / write buffer — one instance per port group.
//   Composed of NUM_TDM buffer_cell instances (one per TDM slot), plus
//   combinatorial logic to orchestrate drain windows and drive port outputs.
//
//   Template parameters:
//     NUM_REQ       — OBI beats per port
//     PORT_COUNT    — maximum ports/groups connected to this buffer
//     BYTES_PER_ROW — data width in bytes
//     NUM_TDM       — TDM lanes and slots in one fetch window
//     IS_WRITE      — false (default): read-prefetch buffer; true: write buffer
//
//   Window drain protocol (read mode) — pipelined, no window-transition gap:
//     Cells are drained sequentially in groups of beats_for_mode(window_mode_q)
//     = ports_for_mode(window_mode_q) * NUM_REQ — the LATCHED window geometry,
//     so a mid-stream active_mode change never tears a window in flight.
//     rd_ptr_q tracks the current group's base slot.  Each cell
//     starts fetching its OWN next value the instant its OWN group drains
//     (see buffer_cell.hpp's header comment) — there is no single
//     whole-window "prefetch trigger" position to time: a plain per-cell
//     mux between "present what's stored" and "forward what just arrived"
//     (is_fwd), gated on that cell's own all_valid_i. This is safe by
//     construction and needs no special-casing: a group can only drain once
//     every cell in it is valid (not pending), so a cell's own refetch can
//     never start while an earlier one for it is still in flight, and this
//     exact group position isn't needed again until every OTHER group in
//     the window has had its own turn — n_groups-1 cycles of slack, which
//     covers the fetch's 2-cycle round trip (1 arbiter grant + 1 bank
//     response) with room to spare for any window with 3+ groups. A 1- or
//     2-group window has too little slack to fully hide the round trip
//     (0 or 1 cycles respectively) and keeps a small, unavoidable bubble —
//     not expected to matter for this buffer's actual configurations, which
//     all have several groups per window.
//
//     Because there's no separate signal to time, window_mode_q (this
//     window's own frozen geometry — see that signal's own comment) simply
//     tracks active_mode continuously until primed, then re-latches once
//     at each real wraparound.
//
//     Restart from idle (after reset, a fence, or a parked task switch) is
//     the same path, not a special case: when every cell reports idle and
//     the fetch bus is enabled, a boot latch snaps the staged window under
//     the CURRENT geometry and pulses the same window_reset a wraparound
//     does — one pulse, one meaning ("advance the lookahead one window"),
//     for the caller and the cells alike (see seq_proc's boot_latch).
//
//   Read mode (IS_WRITE=false):
//     Cells proactively fetch TDM data when fetch_addr_valid_i fires.  Group
//     drains when all cells are VALID and all active ports have asserted p_req_i.
//     p_rdata_o carries the fetched data.
//
//   Write mode (IS_WRITE=true) — pipelined fill / snapshot / posted respond
//   (the write-side twin of the read pipelining above; see the write branch
//   in comb_proc/seq_proc for the full picture):
//     Fill     : ports write one group (fill_beats) at a time; p_gnt_o fires
//                when all active lanes request AND the group's cells have
//                free primary latches. The fill pointer advances one group
//                per cycle until the whole window (NUM_TDM slots) is
//                latched — including straight through window boundaries.
//     Snapshot : a one-cycle reset_window_i pulse hands the filled window
//                to the cells' shadow flush engines, which burst all
//                NUM_TDM TDM writes together (one atomic window burst) and
//                drain to their banks independently — while the ports are
//                already filling the NEXT window into the freed primaries.
//     Respond  : POSTED port acks — one group of p_rvalid_o per cycle
//                right behind the snapshot, meaning "the buffer holds the
//                write and its burst is in flight", NOT "the bank
//                committed it". Uncontended, windows ack back to back at
//                the crossbar's exact pacing; a slow/conflicted burst
//                instead back-pressures the NEXT window's snapshot (its
//                fill parks as full_q until the shadows and acks drain).
//                p_rdata_o is always 0.
//     fetch_addr_i and fetch_addr_valid_i are present but unused in write mode.
//
//   active_mode encoding (matches buffer.sv):
//     0   → 1 active port
//     1   → 2 active ports
//     2/3 → 4 active ports (clamped to PORT_COUNT)
// -----------------------------------------------------------------------------

#ifndef BUFFER_HPP
#define BUFFER_HPP

#include <systemc.h>

#include "buffer_cell.hpp"
#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <algorithm>
#include <cstdint>
#include <cstdio>

template <int NUM_REQ = 4, int PORT_COUNT = 4, int BYTES_PER_ROW = 4 * 4, int NUM_TDM = 32,
          bool IS_WRITE = false>
SC_MODULE(buffer) {
    static constexpr int BUFFER_SIZE = NUM_TDM;
    static constexpr int NUM_IO      = PORT_COUNT * NUM_REQ;
    using data_t                     = obi_data<BYTES_PER_ROW>;
    using cell_t                     = buffer_cell<BYTES_PER_ROW, IS_WRITE>;

    static_assert(NUM_REQ >= 1, "NUM_REQ must be >= 1");
    static_assert(PORT_COUNT >= 1 && PORT_COUNT <= 4, "PORT_COUNT must be in [1, 4]");
    static_assert(NUM_TDM >= NUM_IO && NUM_TDM % NUM_IO == 0,
                  "NUM_TDM must be a positive multiple of NUM_IO");
    static_assert(BYTES_PER_ROW >= 1 && BYTES_PER_ROW <= 32, "BYTES_PER_ROW must be in [1, 32]");

    // -----------------------------------------------------------------------
    // Clock / reset / mode
    // -----------------------------------------------------------------------
    sc_in<bool>     clk_i;
    sc_in<bool>     rst_ni;
    sc_in<uint32_t> active_mode;

    // -----------------------------------------------------------------------
    // Port-facing OBI  (NUM_IO = PORT_COUNT * NUM_REQ ports) — full
    // subordinate bundles. p[i].we_i is wired for interface completeness but
    // ignored: direction is fixed per instance by IS_WRITE (read buffers
    // only read, write buffers always write).
    // -----------------------------------------------------------------------
    obi_subordinate_ports<data_t> p[NUM_IO];

    // -----------------------------------------------------------------------
    // TDM-facing OBI  (NUM_TDM-wide; one full OBI port per slot) — m[t].wdata_o
    // driven in write mode, 0 in read mode; m[t].rdata_i used in read mode,
    // ignored in write mode.
    // -----------------------------------------------------------------------
    obi_manager_ports<data_t> m[NUM_TDM];

    // Per-slot fetch addresses — used in read mode; present but unused in write mode
    sc_in<uint64_t> fetch_addr_i[NUM_TDM];
    sc_in<bool>     fetch_addr_valid_i;

    // =======================================================================
    // Internal
    // =======================================================================
  private:
    // -----------------------------------------------------------------------
    // Cell array
    // -----------------------------------------------------------------------
    cell_t *cells[NUM_TDM];

    // -----------------------------------------------------------------------
    // Interconnect signals (buffer → cells)
    // -----------------------------------------------------------------------
    sc_signal<bool> cell_all_valid_s[NUM_TDM];
    // Read mode: the post-drain commit echo, split from cell_all_valid_s so
    // a cell can tell "your group is draining NOW" (drain + refetch trigger)
    // apart from "your group drained LAST cycle" (late-commit permission
    // only) — see buffer_cell.hpp's commit_ok_i comment for the wipe this
    // split fixes. Write mode holds it at 0.
    sc_signal<bool> cell_commit_ok_s[NUM_TDM];
    // One wire bundle per cell's port side (req/addr/wdata/be routed to the
    // active cell by comb_proc; rdata read back for the drain; gnt/rvalid/we
    // exist only to complete the bundle — the buffer drives port gnt/rvalid
    // itself and we is fixed by IS_WRITE).
    obi_signal_bundle<data_t> cell_p_s[NUM_TDM];
    // Read mode: pulses AT this window's last-group drain (wrap_now) or on
    // the all-cells-idle boot latch — both mean "advance the caller's
    // lookahead cursor one window" (see seq_proc; cells start their own
    // refetches off all_valid_i, not off this). Write mode: the one-cycle
    // SNAPSHOT pulse (see comb_proc), handing every cell's primary latch to
    // its shadow flush engine atomically.
    sc_signal<bool> cell_reset_window_s;

    // -----------------------------------------------------------------------
    // Interconnect signals (cells → buffer)
    // -----------------------------------------------------------------------
    sc_signal<bool> cell_valid_s[NUM_TDM];

    // Read mode: each cell's idle/parked flag (invalid_o = !valid_q &&
    // !pending_q) — ANDed in seq_proc to detect the all-cells bootstrap
    // latch (see its comment there). Unused in write mode.
    sc_signal<bool> cell_invalid_s[NUM_TDM];

    // -----------------------------------------------------------------------
    // Pointer register (drain pointer in read mode; respond pointer in write
    // mode — the fill side has its own fill_ptr_q)
    // -----------------------------------------------------------------------
    sc_signal<int> rd_ptr_q;

    // Read mode only: the group that drained on the IMMEDIATELY PRECEDING
    // cycle (-1 if none) and how wide it was, so comb_proc can give that
    // group's cells exactly one extra all_valid_i pulse the cycle after
    // they're no longer at rd_ptr. Without this, a group's cells only ever
    // see all_valid_i while rd_ptr is actually parked on them — for a window
    // with few enough groups that this same position gets revisited again
    // very soon (2 groups is the tightest real case), their still-valid_q
    // register from the drain that just happened hasn't had a chance to
    // register-clear yet, so the next visit sees "still valid" and can
    // drain the exact same (already-delivered) data a second time under a
    // different label. One extra all_valid_i pulse immediately after the
    // drain is enough: buffer_cell.hpp's own commit logic (unchanged) either
    // promotes a same-cycle-ready new fetch (is_fwd) or, if not ready yet,
    // takes the ordinary "plain drained" branch and clears valid_q — either
    // way the cell can no longer look "still valid" by the time this same
    // position comes up again. This is purely a buffer.hpp-level fix (no
    // buffer_cell.hpp changes) specifically so it can't let one cell in a
    // multi-lane group race ahead of its groupmates the way gating this on
    // a per-cell "already delivered" escape inside safe's own definition
    // did (tried and reverted — see git history: broke DMA's wide,
    // two-lane task by letting the primary lane commit independently of
    // the secondary).
    sc_signal<int> last_drained_base_q;
    sc_signal<int> last_drained_n_beats_q;

    // -----------------------------------------------------------------------
    // Read mode only: this window's own latched geometry. Each cell starts
    // its own refetch off its own all_valid_i (see buffer_cell.hpp's header
    // comment), so — unlike a design with one shared "prefetch trigger" —
    // there is no single early moment where the incoming window's config
    // needs to be captured ahead of time. window_mode_q just tracks
    // active_mode continuously until primed_q (bootstrap: nothing has
    // drained yet, nothing to protect), then re-latches active_mode once
    // per window, exactly at the real wraparound (this window's actual
    // last group) — so it stays stable for the whole of a window's drain,
    // and group boundaries are never misclassified mid-drain even if
    // active_mode changes right underneath. Write mode doesn't need any of
    // this: it has no lookahead prefetch to race against, so it just reads
    // active_mode live (see active_ports()).
    // -----------------------------------------------------------------------
    sc_signal<uint32_t> window_mode_q;
    sc_signal<bool>     primed_q;

    // -----------------------------------------------------------------------
    // Write mode only — the pipelined fill/snapshot/respond stages (the
    // write-side twin of the read pipelining; see the header comment):
    //   fill_ptr_q     — next group to accept from the ports (chases the
    //                    respond/snapshot machinery window by window)
    //   full_q         — window fully latched, snapshot pending (only when
    //                    the previous window's flush/respond isn't done yet)
    //   resp_pending_q — a snapshotted window still owes port acks;
    //                    rd_ptr_q doubles as its respond pointer
    //   resp_mode_q    — the geometry the responding window was FILLED
    //                    with (fill always uses live active_mode; a task
    //                    boundary may change it while the previous
    //                    window's acks are still streaming)
    sc_signal<int>      fill_ptr_q;
    sc_signal<bool>     full_q;
    sc_signal<bool>     resp_pending_q;
    sc_signal<uint32_t> resp_mode_q;
    sc_signal<uint32_t> pend_mode_q; // geometry of a window parked as full_q

    SC_HAS_PROCESS(buffer);

  public:
    explicit buffer(sc_module_name nm) : sc_module(nm) {
        for (int t = 0; t < NUM_TDM; ++t) {
            char name[32];
            std::snprintf(name, sizeof(name), "cell_%d", t);
            cells[t] = new cell_t(name);

            cells[t]->clk_i(clk_i);
            cells[t]->rst_ni(rst_ni);

            cells[t]->addr_i(fetch_addr_i[t]);
            cells[t]->en_i(fetch_addr_valid_i);

            bind_obi(cells[t]->m, m[t]);

            bind_obi(cells[t]->p, cell_p_s[t]);
            cells[t]->all_valid_i(cell_all_valid_s[t]);
            cells[t]->commit_ok_i(cell_commit_ok_s[t]);
            cells[t]->reset_window_i(cell_reset_window_s);

            cells[t]->valid_o(cell_valid_s[t]);
            cells[t]->invalid_o(cell_invalid_s[t]);
        }

        SC_METHOD(comb_proc);
        sensitive << active_mode << rd_ptr_q;
        if constexpr (IS_WRITE) {
            sensitive << fill_ptr_q << full_q << resp_pending_q << resp_mode_q;
            for (int t = 0; t < NUM_TDM; ++t)
                sensitive << cell_invalid_s[t];
        }
        if constexpr (!IS_WRITE) {
            // last_drained_*_q matter here: comb_proc derives the one-cycle
            // all_valid_i echo from them (see last_drained_base_q's own
            // comment) — without them in this list, the echo's CLEAR (the
            // -1 written the edge after a drain) never re-triggers
            // comb_proc, leaving the drained group's cell_all_valid_s stuck
            // high until some unrelated input happens to wiggle. A caller
            // that re-drives p_addr_i every cycle (every AGU harness) masks
            // that completely; a quiet caller sees the stuck echo let a
            // parked cell spuriously restart a fetch with whatever stale
            // address is still on the bus (caught by tb_buffer T22).
            sensitive << window_mode_q << last_drained_base_q << last_drained_n_beats_q;
        }
        for (int t = 0; t < NUM_TDM; ++t)
            sensitive << cell_valid_s[t];
        for (int i = 0; i < NUM_IO; ++i) {
            sensitive << p[i].req_i << p[i].addr_i;
            if constexpr (IS_WRITE)
                sensitive << p[i].wdata_i << p[i].be_i;
        }

        SC_THREAD(seq_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);
    }

    ~buffer() {
        for (int t = 0; t < NUM_TDM; ++t)
            delete cells[t];
    }

  public:
    struct snapshot_t {
        int  rd_ptr;
        int  n_valid;
        int  n_beats;
        bool window_reset;
    };

    snapshot_t snapshot() const {
        int n = 0;
        for (int t = 0; t < NUM_TDM; ++t)
            if (cell_valid_s[t].read())
                ++n;
        int n_beats = IS_WRITE ? active_beats() : beats_for_mode(window_mode_q.read());
        return {rd_ptr_q.read(), n, n_beats, cell_reset_window_s.read()};
    }

  private:
    static int ports_for_mode(uint32_t mode) {
        switch (mode & 0x3u) {
        case 0:
            return std::min(PORT_COUNT, 1);
        case 1:
            return std::min(PORT_COUNT, 2);
        default:
            return std::min(PORT_COUNT, 4);
        }
    }

    static int beats_for_mode(uint32_t mode) {
        return ports_for_mode(mode) * NUM_REQ;
    }

    // Write mode only (and snapshot()'s write-mode branch): reads the live
    // active_mode directly — see window_mode_q's own comment for why write
    // mode doesn't need to latch anything.
    int active_ports() const {
        return ports_for_mode(active_mode.read());
    }

    int active_beats() const {
        return active_ports() * NUM_REQ;
    }

    struct GroupState {
        bool ports_req;
        bool cells_valid;
        bool can_drain;
        bool is_last;
    };

    GroupState eval_group(int base, int n_beats) const {
        bool ports_req = true;
        for (int i = 0; i < n_beats; ++i)
            if (!p[i].req_i.read()) {
                ports_req = false;
                break;
            }

        bool cells_valid = (base + n_beats <= BUFFER_SIZE);
        for (int i = 0; cells_valid && i < n_beats; ++i)
            cells_valid = cell_valid_s[base + i].read();

        return {ports_req, cells_valid, ports_req && cells_valid, base + n_beats == BUFFER_SIZE};
    }

    // -----------------------------------------------------------------------
    // Combinational process — drives cell controls and port grant
    // -----------------------------------------------------------------------
    void comb_proc() {
        int base = rd_ptr_q.read();

        if constexpr (!IS_WRITE) {
            // ------------------------------------------------------------------
            // READ mode: TDM pre-filled → drain to ports group-by-group,
            // pipelined with the next window's prefetch (see header comment).
            // ------------------------------------------------------------------
            int  n_beats = beats_for_mode(window_mode_q.read());
            auto grp     = eval_group(base, n_beats);

            // cell_reset_window_s itself is registered from seq_proc for
            // read mode now (see that process's own comment) — not driven
            // here, so an external observer polling snapshot() right after
            // an edge can't catch a same-cycle glitch from p_req_i still
            // reflecting whichever group it was last written for.

            int last_drained_base    = last_drained_base_q.read();
            int last_drained_n_beats = last_drained_n_beats_q.read();

            for (int t = 0; t < NUM_TDM; ++t) {
                bool in_grp = (t >= base) && (t < base + n_beats);
                int  lane   = t - base;
                // See last_drained_base_q's own comment: a one-cycle echo
                // of all_valid_i for whichever group drained last cycle,
                // so its cells get a chance to clear/recommit before this
                // exact position can ever be revisited again. (Removing
                // this was re-tried after the bootstrap window_reset pulse
                // fix landed, on the theory it only compensated for the old
                // one-window-behind lag: still 12/60/22 failures across
                // tb_buffer/stim_bank_tdm/top_tdm — it is genuinely
                // load-bearing for narrow-window revisits, not a
                // compensation artifact.)
                bool just_drained = last_drained_base >= 0 && t >= last_drained_base &&
                                    t < last_drained_base + last_drained_n_beats;
                cell_all_valid_s[t].write(grp.can_drain && in_grp);
                cell_commit_ok_s[t].write(just_drained && !in_grp);
                cell_p_s[t].req.write(grp.ports_req && in_grp);
                cell_p_s[t].addr.write(in_grp ? p[lane].addr_i.read() : uint64_t{0});
            }

            for (int i = 0; i < NUM_IO; ++i)
                p[i].gnt_o.write(grp.can_drain && (i < n_beats));

        } else {
            // ------------------------------------------------------------------
            // WRITE mode, pipelined: window k's flush+respond overlap window
            // k+1's fill (see the header comment). Three concurrent pieces:
            //
            //   Fill    — accept one group of port writes at fill_ptr_q when
            //             every lane requests AND every cell in the group has
            //             a free primary latch (cell_invalid_s — the natural
            //             chase: window k+1's group g frees the instant the
            //             snapshot fires, not when a global phase ends).
            //   Snapshot — the one-cycle reset_window_i pulse handing a fully
            //             latched window to the cells' shadow flush engines
            //             (they burst the whole window to TDM together,
            //             exactly as before). Fires the same cycle the
            //             window's last fill group is granted when nothing
            //             blocks it, else as soon as the previous window's
            //             respond finishes and its shadows are free.
            //   Respond — port acks for the snapshotted window, one group
            //             per cycle at rd_ptr_q under the geometry that
            //             window was filled with. POSTED: an ack means "the
            //             buffer holds the write and its TDM burst is in
            //             flight", not "the bank committed it" — that is
            //             what lets the acks stream concurrently with the
            //             next fill instead of serializing behind the bank
            //             round trip (the read-back checks in every
            //             integration suite verify the data does land).
            // ------------------------------------------------------------------
            int fbase      = fill_ptr_q.read();
            int fill_beats = active_beats();
            int resp_beats = beats_for_mode(resp_mode_q.read());

            // Fill side.
            bool fill_free = true;
            for (int i = 0; fill_free && i < fill_beats; ++i)
                fill_free = (fbase + i < BUFFER_SIZE) && cell_invalid_s[fbase + i].read();
            bool ports_req = true;
            for (int i = 0; ports_req && i < fill_beats; ++i)
                ports_req = p[i].req_i.read();
            bool fill_ok   = ports_req && fill_free && !full_q.read();
            bool fill_wrap = fill_ok && (fbase + fill_beats == BUFFER_SIZE);

            // Snapshot decision — mirrored in seq_proc (same inputs). A
            // respond that WRAPS this very edge counts as done: the next
            // window's snapshot may fire alongside its last ack group, so
            // the ack stream runs back-to-back across windows with no gap
            // (matching the crossbar's continuous pacing).
            bool shadows_free = true;
            for (int t = 0; shadows_free && t < NUM_TDM; ++t)
                shadows_free = !cell_valid_s[t].read(); // write: valid_o = shadow busy
            bool resp_now   = resp_pending_q.read();
            bool resp_wraps = resp_now && (base + resp_beats >= BUFFER_SIZE);
            bool snapshot_now =
                (fill_wrap || full_q.read()) && shadows_free && (!resp_now || resp_wraps);
            cell_reset_window_s.write(snapshot_now);

            for (int t = 0; t < NUM_TDM; ++t) {
                bool in_fill = (t >= fbase) && (t < fbase + fill_beats);
                int  lane    = t - fbase;
                cell_p_s[t].req.write(fill_ok && in_fill);
                cell_p_s[t].addr.write(in_fill ? p[lane].addr_i.read() : uint64_t{0});
                cell_p_s[t].wdata.write(in_fill ? p[lane].wdata_i.read() : data_t{0});
                cell_p_s[t].be.write(in_fill ? p[lane].be_i.read() : uint32_t{0});
                cell_all_valid_s[t].write(false); // unused by write cells
                cell_commit_ok_s[t].write(false); // unused by write cells
            }
            for (int i = 0; i < NUM_IO; ++i)
                p[i].gnt_o.write(fill_ok && (i < fill_beats));
        }
    }

    // -----------------------------------------------------------------------
    // Sequential process — advances rd_ptr and registers port response
    // -----------------------------------------------------------------------
    void seq_proc() {
        rd_ptr_q.write(0);
        if constexpr (!IS_WRITE) {
            window_mode_q.write(0);
            primed_q.write(false);
            last_drained_base_q.write(-1);
            last_drained_n_beats_q.write(0);
            cell_reset_window_s.write(false);
        }
        if constexpr (IS_WRITE) {
            fill_ptr_q.write(0);
            full_q.write(false);
            resp_pending_q.write(false);
            resp_mode_q.write(0);
            pend_mode_q.write(0);
        }
        for (int i = 0; i < NUM_IO; ++i) {
            p[i].rvalid_o.write(false);
            p[i].rdata_o.write(data_t{0});
        }
        wait();

        while (true) {
            int base = rd_ptr_q.read();

            bool   rvalid_next[NUM_IO] = {};
            data_t rdata_next[NUM_IO]  = {};

            if constexpr (!IS_WRITE) {
                // -------------------------------------------------------
                // READ mode: drain TDM-prefilled cells to ports, pipelined
                // with the next window's prefetch (see header comment).
                // -------------------------------------------------------
                bool     primed   = primed_q.read();
                uint32_t cur_mode = window_mode_q.read();
                bool     en_now   = fetch_addr_valid_i.read();

                // ---- Bootstrap latch: every cell restarts at once ----
                //
                // A cell starts a fetch whenever it holds nothing and en is
                // high (see buffer_cell.hpp's start rule) — so the moment
                // en_now is high while EVERY cell is idle (parked:
                // !valid_q && !pending_q, read off invalid_o), all of them
                // latch the staged bus together this very edge. That
                // consumes the entire staged window in one go, exactly like
                // a full drain cycle does group by group — so it must pulse
                // window_reset ("advance your cursor one window") just like
                // a wraparound, and it re-latches the window geometry from
                // whatever the caller has active_mode set to NOW (base is
                // snapped to 0 for self-consistency; a fully-parked buffer
                // has no meaningful drain position).
                //
                // All-cells-idle can only happen three ways — after reset,
                // after a fence (the caller held en low while the last
                // window drained, so no cell could restart), and after a
                // caller-side task/mode switch against a parked buffer —
                // and the response is correct for all three: fetch the
                // staged window under the caller's current geometry. It
                // can NEVER fire mid-window: with en high, a drained cell
                // restarts the same edge it drains (so it's pending, not
                // idle), and a still-presenting cell is valid — so a brief
                // en dip, a mid-drain mode change, or in-flight fetches
                // all leave at least one cell non-idle and the latched
                // window undisturbed. Since a start wipes nothing (see the
                // cell's own comment), no gap threshold or edge detection
                // is needed: a short en_i dip simply resumes where things
                // stood.
                bool all_idle = true;
                for (int t = 0; all_idle && t < NUM_TDM; ++t)
                    all_idle = cell_invalid_s[t].read();
                bool boot_latch_now = en_now && all_idle;
                if (boot_latch_now) {
                    cur_mode = active_mode.read();
                    base     = 0;
                }

                int  n_beats  = beats_for_mode(cur_mode);
                auto grp      = eval_group(base, n_beats);
                bool wrap_now = grp.can_drain && grp.is_last;

                // Registered here (using this invocation's stable, pre-advance
                // base/grp) rather than left as a raw combinational signal in
                // comb_proc — an external observer (e.g. an AGU's lookahead
                // cursor) polling snapshot() right after an edge would
                // otherwise be able to catch comb_proc reacting to the
                // freshly-advanced rd_ptr with whichever p_req_i values
                // happen to still be on the (reused) lane wires from the
                // group that just drained, before anything's had a chance to
                // update them for the new position — a same-cycle glitch,
                // not a real second wrap. Registering it here means it only
                // ever reflects what THIS edge's own seq_proc pass actually
                // decided, once, synchronized to the real clock.
                //
                // Pulsed on boot_latch_now as well as wrap_now: both events
                // mean "the staged window's addresses have all been
                // latched by cells — advance the lookahead cursor", the
                // wrap because window N's drain-triggered refetches
                // consumed window N+1's slices group by group, the
                // bootstrap because every cell latched its slice at once.
                // One unified caller contract: one pulse = advance one
                // window. Without the bootstrap pulse the caller's cursor
                // runs one window behind from every bootstrap onward:
                // during window N's drain the bus still holds N's own
                // addresses, every refetch re-latches the window it came
                // from, and every window's drain delivers the PREVIOUS
                // window's data for the rest of the task. The two can't
                // fire together — a bootstrap edge has no valid cells, so
                // can_drain (hence wrap_now) is false.
                cell_reset_window_s.write(wrap_now || boot_latch_now);

                if (grp.can_drain) {
                    for (int i = 0; i < n_beats; ++i) {
                        rvalid_next[i] = true;
                        rdata_next[i]  = cell_p_s[base + i].rdata.read();
                    }
                    // Remember this exact group (base, width) so comb_proc
                    // can give its cells one extra all_valid_i pulse next
                    // cycle — see last_drained_base_q's own comment.
                    last_drained_base_q.write(base);
                    last_drained_n_beats_q.write(n_beats);
                    base = grp.is_last ? 0 : base + n_beats;
                } else {
                    last_drained_base_q.write(-1);
                }

                // window_mode_q: continuously track active_mode until
                // primed (bootstrap, nothing drained yet to protect), then
                // re-latch it once per window exactly at the real
                // wraparound, or on a bootstrap latch's own geometry snap
                // above — see that signal's own comment.
                if (!primed || wrap_now || boot_latch_now)
                    window_mode_q.write(active_mode.read());
                primed_q.write(primed || wrap_now);

                rd_ptr_q.write(base);

            } else {
                // -------------------------------------------------------
                // WRITE mode: pipelined fill / snapshot / posted respond —
                // see comb_proc's write branch for the full picture. This
                // side re-derives the same fill_ok / snapshot_now
                // decisions from the same pre-edge state and advances the
                // pointers/flags accordingly.
                // -------------------------------------------------------
                int  fbase      = fill_ptr_q.read();
                bool full       = full_q.read();
                bool resp       = resp_pending_q.read();
                int  fill_beats = active_beats();
                int  resp_beats = beats_for_mode(resp_mode_q.read());

                bool fill_free = true;
                for (int i = 0; fill_free && i < fill_beats; ++i)
                    fill_free = (fbase + i < BUFFER_SIZE) && cell_invalid_s[fbase + i].read();
                bool ports_req = true;
                for (int i = 0; ports_req && i < fill_beats; ++i)
                    ports_req = p[i].req_i.read();
                bool fill_ok   = ports_req && fill_free && !full;
                bool fill_wrap = fill_ok && (fbase + fill_beats == BUFFER_SIZE);

                bool shadows_free = true;
                for (int t = 0; shadows_free && t < NUM_TDM; ++t)
                    shadows_free = !cell_valid_s[t].read();
                bool resp_wraps   = resp && (base + resp_beats >= BUFFER_SIZE);
                bool snapshot_now = (fill_wrap || full) && shadows_free && (!resp || resp_wraps);

                // Posted respond: one group of port acks per cycle for the
                // most recently snapshotted window, under the geometry it
                // was FILLED with.
                if (resp) {
                    for (int i = 0; i < resp_beats; ++i)
                        rvalid_next[i] = true;
                    base += resp_beats;
                    if (base >= BUFFER_SIZE) {
                        base = 0;
                        resp = false;
                    }
                }

                // Snapshot: hand the latched window to the shadows (the
                // cells see cell_reset_window_s this same edge, driven by
                // comb_proc from this same condition) and start its acks.
                if (snapshot_now) {
                    resp = true;
                    base = 0;
                    resp_mode_q.write(fill_wrap ? active_mode.read() : pend_mode_q.read());
                    full = false;
                }

                // Fill advance. A window completing while the previous
                // one's flush or acks are still running parks as `full`
                // (its cells' primaries stay latched, which also blocks
                // further fill grants) until the snapshot above releases
                // it; its geometry is remembered in pend_mode_q since the
                // caller may already be moving to a different task.
                if (fill_ok) {
                    fbase += fill_beats;
                    if (fbase >= BUFFER_SIZE) {
                        fbase = 0;
                        if (!snapshot_now) {
                            full = true;
                            pend_mode_q.write(active_mode.read());
                        }
                    }
                }

                fill_ptr_q.write(fbase);
                full_q.write(full);
                resp_pending_q.write(resp);
                rd_ptr_q.write(base);
            }

            for (int i = 0; i < NUM_IO; ++i) {
                p[i].rvalid_o.write(rvalid_next[i]);
                p[i].rdata_o.write(rdata_next[i]); // always 0 in write mode
            }

            wait();
        }
    }
};

#endif // BUFFER_HPP
