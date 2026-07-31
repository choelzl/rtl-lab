// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Prefetch/write buffer, one instance per port group: NUM_TDM buffer_cell
// instances (one per TDM slot) plus comb logic orchestrating drain windows
// and port I/O. NUM_REQ: OBI beats/port. PORT_COUNT: ports/groups. NUM_TDM:
// lanes+slots per window. IS_WRITE: false=read-prefetch, true=write.
// -----------------------------------------------------------------------------
//
// Read mode: cells proactively fetch on fetch_addr_valid_i. Drained group-
// by-group (beats_for_mode(window_mode_q), the LATCHED geometry so a
// mid-window active_mode change can't tear a window). Each cell starts its
// own next fetch the instant its own group drains (buffer_cell.hpp), so
// there's no shared "prefetch trigger" to time — a fetch's 2-cycle round
// trip is hidden by the n_groups-1 cycles before that position is needed
// again (zero-bubble for 3+ groups). Restart from idle (reset/fence/task
// switch) reuses the same path: a boot latch fires when every cell is idle,
// snapping the staged window under current geometry.
//
// Write mode: pipelined fill/snapshot/posted-respond (see comb_proc/
// seq_proc write branches). Fill accepts one group/cycle when ports request
// and cells are free. Snapshot hands a full window to the cells' shadow
// flush engines (one atomic burst) while ports fill the next window.
// Respond posts acks right behind snapshot — "write is in flight", not
// "bank committed" — so a slow burst back-pressures only the next
// snapshot, not the acks already streaming.
//
// active_mode (matches buffer.sv): 0->1 port, 1->2 ports, 2/3->4 ports
// (clamped to PORT_COUNT).
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

    // Read mode only: the group that drained last cycle (-1 if none) and
    // its width, giving those cells one extra all_valid_i pulse the cycle
    // after they leave rd_ptr — needed for narrow windows (2 groups is the
    // tightest case) where the same position is revisited before valid_q
    // has cleared, which would otherwise re-drain stale data. A per-cell
    // fix (inside buffer_cell.hpp's safe) was tried and reverted: it let
    // one lane in a multi-lane group commit ahead of its groupmates.
    sc_signal<int> last_drained_base_q;
    sc_signal<int> last_drained_n_beats_q;

    // Read mode only: this window's latched geometry. Tracks active_mode
    // live until primed_q, then re-latches once per window exactly at the
    // real wraparound — so group boundaries stay stable for a whole drain
    // even if active_mode changes mid-drain. Write mode has no lookahead
    // to race against, so it just reads active_mode live (active_ports()).
    sc_signal<uint32_t> window_mode_q;
    sc_signal<bool>     primed_q;

    // Write mode: pipelined fill/snapshot/respond stages (see header).
    // fill_ptr_q: next group to accept. full_q: window latched, snapshot
    // pending. resp_pending_q: a snapshotted window still owes acks
    // (rd_ptr_q doubles as its pointer). resp_mode_q: geometry the
    // responding window was FILLED with (may differ from live active_mode).
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
            // last_drained_*_q needed: without them, the echo's CLEAR (-1
            // the edge after a drain) never re-triggers comb_proc, leaving
            // cell_all_valid_s stuck high until an unrelated input wiggles
            // (caught by tb_buffer T22).
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

            // cell_reset_window_s is registered from seq_proc for read mode
            // (not driven here), so snapshot() can't catch a same-cycle
            // glitch from stale p_req_i wiring.

            int last_drained_base    = last_drained_base_q.read();
            int last_drained_n_beats = last_drained_n_beats_q.read();

            for (int t = 0; t < NUM_TDM; ++t) {
                bool in_grp = (t >= base) && (t < base + n_beats);
                int  lane   = t - base;
                // One-cycle echo of all_valid_i for whichever group drained
                // last cycle (see last_drained_base_q) — confirmed
                // load-bearing, not a compensation artifact: removing it
                // still fails 12/60/22 tests across tb_buffer/stim_bank_tdm/
                // top_tdm even with the bootstrap window_reset fix in place.
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
            // Write mode, pipelined (see header): Fill accepts one group at
            // fill_ptr_q once every lane requests and every cell is free.
            // Snapshot (one-cycle reset_window_i) hands a full window to the
            // shadows once the previous window's respond/shadows are clear.
            // Respond posts one ack group/cycle right behind snapshot —
            // POSTED, meaning in-flight not bank-committed, so it can stream
            // concurrently with the next fill.
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

                // Bootstrap latch: if EVERY cell is idle (parked) while
                // en_now is high, all of them latch the staged bus together
                // this edge — consuming the whole window at once, so it
                // must pulse window_reset like a wraparound and re-latch
                // geometry from active_mode now. Can only trigger after
                // reset, a fence, or a parked task switch — never
                // mid-window, since a drained cell is pending (not idle)
                // and a presenting one is valid.
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

                // Registered here (this invocation's stable pre-advance
                // base/grp), not left combinational in comb_proc, so an
                // observer polling snapshot() right after an edge can't
                // catch a same-cycle glitch from the freshly-advanced
                // rd_ptr against stale lane wires.
                //
                // Pulsed on wrap_now OR boot_latch_now: both mean "advance
                // the lookahead cursor one window". Without the bootstrap
                // pulse the caller's cursor runs one window behind forever
                // after any bootstrap. The two can't fire together (a
                // bootstrap edge has no valid cells, so can_drain is false).
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
