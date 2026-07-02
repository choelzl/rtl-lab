// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Single read/write-buffer slot.
//
//   Template parameters:
//     BYTES_PER_ROW  — data width in bytes
//     IS_WRITE       — false (default): read-prefetch cell; true: write cell
//
//   Read mode (IS_WRITE=false) state — no combined FSM; a handful of
//   independent, orthogonal bits instead, so "old data still being drained"
//   and "new data being fetched" can never collide into a single state that
//   has to special-case their overlap:
//     valid_q   — data_q is presentable to the port right now.
//     pending_q — a fetch (for whatever this cell's NEXT value is) is in
//                 flight. Independent of valid_q on purpose: this cell
//                 starts its next fetch off its own all_valid_i drain pulse
//                 (the zero-bubble handoff — see the start rule below), so
//                 the fetch runs while the cell is still valid_q and
//                 presenting its CURRENT data.
//     granted_q — meaningful only while pending_q: have we seen m.gnt_i yet
//                 (stop asserting req, wait for m.rvalid_i)? Irrelevant for
//                 an addr=0 NOP fetch, which never touches the bus at all.
//     fetched_q — sticky: the in-flight fetch's response has arrived (NOP,
//                 or bank rvalid) and is staged in data_pend_q, waiting for
//                 a safe moment to promote into data_q. m.rvalid_i is a
//                 one-shot, non-repeating pulse (see bank.hpp) — if it lands
//                 before this cell is safe to overwrite (current data not
//                 yet drained), there is no second chance to see it. The
//                 all_valid_i drain pulse normally lands ready and
//                 safe together so this latches for zero extra cycles, but
//                 under bank contention a fetch or a drain can each be
//                 delayed independently, landing apart — fetched_q/
//                 data_pend_q make that harmless instead of a lost response.
//     primed_q  — has this cell ever committed a fetch? Only gates the
//                 commit-side safe check's boot escape (see below); once
//                 true, stays true.
//     addr_q    — latched target address for the current/pending fetch
//                 (0 = NOP padding, no real TDM transaction needed).
//     data_q    — the presentable data once valid_q.
//
//   Fetch start: `!pending_q && en_i && (all_valid_i || !valid_q)`. The
//   `all_valid_i` disjunct is the zero-bubble handoff — this cell's own
//   group is draining this exact edge, and the commit in the same pass has
//   just freed pending_q and re-set valid_q (so `!valid_q` alone can't see
//   it). The `!valid_q` disjunct covers every "holding nothing" case with
//   one rule: the boot fetch after reset, AND a parked cell (drained while
//   en_i was low — e.g. the caller fenced behind a future task) restarting
//   the moment en_i returns. Nothing is ever wiped or overwritten by a
//   start: pending blocks it while a fetch is in flight, valid blocks it
//   while real data is still being presented — which is why en_i needs no
//   edge detection or thresholds around gaps of any
//   length. This is also collision-free by construction: all_valid_i for a
//   group requires every cell in it to already be valid (see buffer.hpp's
//   eval_group/can_drain), only possible once pending_q has gone false.
//   There is deliberately no separate "remember a pulse that arrived while
//   busy" latch: that scenario is unreachable here (it can only arise if
//   the trigger is decoupled from the cell's own readiness, which this
//   design avoids entirely).
//
//   Fetch completion vs. commit are two separate steps. A pending fetch
//   (real or NOP) is captured into data_pend_q / fetched_q the instant
//   `addr_q==0` (NOP, instant) or `granted_q && m.rvalid_i` (bank's
//   one-shot response arrived) — regardless of whether it's currently safe
//   to show. It's promoted into data_q/valid_q — "commits" — the moment
//   fetched_q is set AND "safe": `!valid_q` (already drained, nothing to
//   protect) or `all_valid_i` (this cell's current data is being drained
//   this exact cycle, so overwriting it now is equivalent to overwriting it
//   the instant after). Since the fetch already only ever starts at
//   all_valid_i, and this cell's own position isn't needed again until
//   every OTHER group in the window has had its own turn, a real fetch's
//   2-cycle round trip (arbiter grant + bank response) has that many cycles
//   of slack to land — zero-bubble for any window with 3+ groups, by
//   construction, with no precisely-timed "early trigger" required (see
//   buffer.hpp's own header comment). fetched_q/data_pend_q exist for when
//   bank contention delays the round trip past even that: the response
//   still isn't lost, it just waits until safe catches up. "fetched &&
//   safe" is also computed combinationally (is_fwd) so the parent sees this
//   cell as valid — and can drain the new window's first group — the same
//   cycle the promotion is legal, without waiting an extra edge for the
//   register.
//
//   The commit check and the plain "drained, nothing new ready yet"
//   transition (valid_q && all_valid_i alone) are mutually exclusive
//   (else-if): all_valid_i is driven by the parent's comb_proc, which
//   itself reacts to this cell's own is_fwd — so the same cycle a commit
//   happens, all_valid_i can already reflect it. Checking both in the same
//   pass would let valid_q flip on then immediately back off within one
//   edge, without ever durably holding VALID for a cycle that a real read
//   could observe it.
//
//   TDM OBI A-channel: req asserted while `pending_q && addr_q!=0 &&
//   !granted_q`; an addr=0 NOP never touches the bus — "pending" for a NOP
//   is purely "waiting for the safe moment to commit".
//
//   Write mode (IS_WRITE=true) — accumulate-then-flush, PIPELINED across
//   windows: the port-facing latch and the TDM-facing flush are two
//   independent stages so window k's flush and port acks overlap window
//   k+1's fill (the write-side twin of the read pipelining above; without
//   it, each window would serialize its whole fill, TDM burst, and port
//   acks instead of overlapping them).
//
//     Primary latch (port side):
//       valid_q          — holding a port write not yet handed to the flush
//       addr_q/data_q/be_q — the latched write (addr==0 = NOP padding)
//     Shadow flush engine (TDM side) — loaded from the primary by the
//     parent's snapshot pulse (reset_window_i, fired once per filled
//     window), freeing the primary for the next window's fill the same
//     edge; the whole window's shadows fire their TDM writes together
//     (one atomic window burst, exactly as before). A shadow's job ends
//     at its GRANT: the bank fabric samples the payload the edge after,
//     so no rvalid tracking is needed (a stray rvalid finds nothing busy
//     and is ignored) and the shadow is reusable one window-fill later
//     even for the shortest (2-group) windows:
//       pending_q        — TDM write awaiting its gnt (a NOP never
//                          starts one)
//       sh_addr_q/data_pend_q/sh_be_q — the in-flight write's own copy
//     all_valid_i        — unused in write mode: port acks (respond) are
//                          POSTED by the parent per snapshotted window
//                          (see buffer.hpp's write branch), not gated on
//                          this cell's bank ack.
//
//   Port OBI (this cell is subordinate) — the full obi_subordinate_ports
//     bundle `p`. p.gnt_o and p.rvalid_o are sunk in buffer.hpp (the parent
//     drives port gnt/rvalid directly); p.we_i is wired for interface
//     completeness but ignored — write mode always drives m.we_o=1.
// -----------------------------------------------------------------------------

#ifndef BUFFER_CELL_HPP
#define BUFFER_CELL_HPP

#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <cstdint>
#include <systemc.h>

template <int BYTES_PER_ROW = 16, bool IS_WRITE = false> SC_MODULE(buffer_cell) {
    static_assert(BYTES_PER_ROW >= 1 && BYTES_PER_ROW <= 32, "BYTES_PER_ROW must be in [1, 32]");

    using data_t = obi_data<BYTES_PER_ROW>;

    // -----------------------------------------------------------------------
    // Clock / reset
    // -----------------------------------------------------------------------
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    // -----------------------------------------------------------------------
    // TDM OBI (cell → TDM) — m.wdata_o driven in write mode, 0 in read mode;
    // m.rdata_i used in read mode, ignored in write mode.
    // -----------------------------------------------------------------------
    obi_manager_ports<data_t> m;

    // -----------------------------------------------------------------------
    // Port OBI — p.req_i is the write-mode accepted fill beat (latched when
    // the primary is free — the parent's grant gating guarantees that;
    // unused in read mode); p.addr_i/wdata_i/be_i are latched with it.
    // p.we_i is ignored (see header). p.gnt_o/p.rvalid_o are sunk in
    // buffer.hpp; p.rdata_o carries fetched data in read mode, 0 in write.
    // -----------------------------------------------------------------------
    obi_subordinate_ports<data_t> p;

    // -----------------------------------------------------------------------
    // Address input — read mode: latched when en_i=1 and a fetch may start
    // -----------------------------------------------------------------------
    sc_in<uint64_t> addr_i;
    sc_in<bool>     en_i;

    // -----------------------------------------------------------------------
    // Window control
    // -----------------------------------------------------------------------
    // Read mode: the one-cycle post-drain echo from the parent (see
    // buffer.hpp's last_drained_base_q). It lets a LATE-resolving fetch
    // commit (safe) — it must NEVER clear or start anything: a beat that
    // committed AT the drain edge (promote-over-drain) is still undelivered
    // when this pulse arrives, and an echo-triggered clear wipes it (found
    // via lane_agu: the final window's group-1 beat read back 0 — mid-stream
    // the wipe was masked by an echo-triggered redundant refetch of the same
    // address, at the stream boundary that refetch latched a NOP instead).
    // Unused in write mode.
    sc_in<bool> commit_ok_i;
    sc_in<bool> all_valid_i;    // read: this cell's group is draining;
                                // write: unused (port acks are posted by
                                // the parent, not gated per cell)
    sc_in<bool> reset_window_i; // write mode only: the snapshot pulse —
                                // hand the primary latch to the shadow
                                // flush engine and free it (see header
                                // comment). Read mode starts its next
                                // fetch off all_valid_i directly and does
                                // not use this signal at all.

    // -----------------------------------------------------------------------
    // State outputs (combinatorial)
    // -----------------------------------------------------------------------
    sc_out<bool> valid_o;   // read: presentable now; write: shadow acked
    sc_out<bool> invalid_o; // read: idle/parked (!valid && !pending);
                            // write: primary latch free (can accept a fill)

    // =======================================================================
    // Internal
    // =======================================================================
  private:
    sc_signal<uint32_t> be_q; // write mode: latched byte-enable; unused in read mode

    // ---- Orthogonal flags, one set per mode (see header comment) ----
    // Read mode:  valid/pending/granted/fetched as documented up top.
    // Write mode: valid_q = primary latch full; pending_q = shadow TDM
    //             write in flight (freed AT its gnt — the returning rvalid
    //             is not tracked). granted_q/fetched_q are read-mode-only
    //             and inert here.
    sc_signal<bool>   valid_q;
    sc_signal<bool>   pending_q;
    sc_signal<bool>   granted_q;   // read mode only
    sc_signal<bool>   fetched_q;   // read mode only
    sc_signal<bool>   primed_q;    // read mode only
    sc_signal<data_t> data_pend_q; // read: staged fetch result; write: shadow wdata

    // Write mode only: the shadow flush engine's own address/byte-enable
    // copy — the primary regs are freed for the next window's fill the
    // same edge the snapshot fires, so the in-flight TDM write needs its
    // own stable copy until the bank acks it.
    sc_signal<uint64_t> sh_addr_q;
    sc_signal<uint32_t> sh_be_q;

    // Shared by both modes.
    sc_signal<uint64_t> addr_q;
    sc_signal<data_t>   data_q; // read mode: TDM rdata; write mode: port wdata to forward to TDM

    SC_HAS_PROCESS(buffer_cell);

  public:
    explicit buffer_cell(sc_module_name nm) : sc_module(nm) {
        if constexpr (IS_WRITE) {
            SC_METHOD(comb_proc_write);
            sensitive << valid_q << pending_q << data_pend_q << sh_addr_q << sh_be_q << m.gnt_i;
        } else {
            SC_METHOD(comb_proc_read);
            // data_q included even though every change to it currently
            // co-occurs with a fetched_q flip (the commit) — relying on that
            // co-occurrence is exactly how buffer.hpp's comb_proc ended up
            // with a stuck all_valid echo (missing last_drained_*_q, caught
            // by tb_buffer T22); list what's read, not what happens to
            // co-trigger.
            sensitive << valid_q << pending_q << granted_q << fetched_q << data_pend_q << addr_q
                      << commit_ok_i << data_q << all_valid_i << m.rvalid_i << m.rdata_i;
        }

        SC_THREAD(seq_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);
    }

  private:
    void comb_proc_write() {
        bool lat     = valid_q.read();   // primary latch holds a port write
        bool sh_busy = pending_q.read(); // shadow TDM write awaiting its gnt

        // Write mode's state wires to the parent:
        //   valid_o   — shadow busy. The payload is consumed the edge AFTER
        //               the grant (the bank fabric samples it then), so the
        //               shadow is reusable from the grant on — and a grant
        //               landing THIS cycle already counts (the same
        //               same-cycle preview trick as read mode's is_fwd):
        //               the parent may snapshot the next window into this
        //               shadow at the very edge the bank samples the old
        //               payload, which is safe by register semantics (the
        //               bank reads pre-edge values). This is what lets a
        //               2-group window (ports=4) run windows back to back
        //               at crossbar pacing instead of waiting out the bank
        //               round trip.
        //   invalid_o — primary latch free; the parent gates a fill group's
        //               grant on every cell in it having this high, which is
        //               what makes window k+1's fill naturally chase window
        //               k's snapshot instead of a global phase.
        valid_o.write(sh_busy && !m.gnt_i.read());
        invalid_o.write(!lat);

        bool m_req = sh_busy;
        m.req_o.write(m_req);
        m.addr_o.write(m_req ? sh_addr_q.read() : uint64_t{0});
        m.we_o.write(m_req);
        m.be_o.write(m_req ? sh_be_q.read() : uint32_t{0});
        m.wdata_o.write(m_req ? data_pend_q.read() : data_t{0});

        // Port gnt/rvalid are driven by the parent buffer (fill grants and
        // posted respond acks) — these standalone outputs stay low.
        p.gnt_o.write(false);
        p.rvalid_o.write(false);
        p.rdata_o.write(data_t{0});
    }

    void comb_proc_read() {
        bool     valid   = valid_q.read();
        bool     pending = pending_q.read();
        bool     granted = granted_q.read();
        bool     fetched = fetched_q.read();
        uint64_t addr    = addr_q.read();

        // Has the in-flight fetch's response arrived — already staged
        // (fetched_q), or arriving this exact cycle (NOP, or bank rvalid)?
        bool fetched_now = fetched || (pending && (addr == 0 || (granted && m.rvalid_i.read())));
        // Is it safe to promote — either nothing live to protect, or this
        // cell's current data is being drained this same cycle (see header
        // comment for why this normally lands together with fetched_now for
        // a real fetch's timing; fetched_q covers the cases where it doesn't).
        // NOTE: deliberately NOT matching seq_proc's own stricter safe (see
        // its comment) — is_fwd's job is to let the PARENT see this cell as
        // valid the same cycle promotion becomes legal, without an extra
        // register edge; tying it to all_valid_i unconditionally (even once
        // primed) makes it circular (is_fwd -> cells_valid -> can_drain ->
        // all_valid_i -> is_fwd) since all_valid_i is itself derived from
        // this cell's own valid_o — tried, caused a real deadlock (T03a).
        // The register-level fix alone (seq_proc) is sufficient: it stops
        // valid_q from committing early, which is what let a structurally-
        // NOP lane's stale valid_q wrongly satisfy this cell's own PLAIN
        // DRAINED branch too soon — is_fwd's laxer !valid escape here is
        // harmless because it only ever feeds the parent's cells_valid
        // check, which still waits on every OTHER cell in the group too.
        bool safe   = !valid || all_valid_i.read() || commit_ok_i.read();
        bool is_fwd = fetched_now && safe;

        valid_o.write(valid || is_fwd);
        // Read mode's "idle" wire: holding nothing and fetching nothing —
        // i.e. parked. The parent ANDs this across all cells to detect the
        // all-at-once restart en_i triggers on parked cells (see start's
        // comment in seq_proc and buffer.hpp's boot_latch): that restart
        // consumes the whole staged window in one go, so the parent must
        // pulse its window_reset ("advance your cursor") exactly like a
        // wraparound. Write mode drives this as "primary latch free"
        // instead (see comb_proc_write).
        invalid_o.write(!valid && !pending);

        bool m_req = pending && (addr != 0) && !granted;
        m.req_o.write(m_req);
        m.addr_o.write(m_req ? addr : uint64_t{0});
        m.we_o.write(false);
        m.be_o.write(m_req ? static_cast<uint32_t>((uint64_t{1} << BYTES_PER_ROW) - 1u) : 0u);
        m.wdata_o.write(data_t{0});

        bool window_ready = valid && all_valid_i.read();
        p.gnt_o.write(window_ready);
        p.rvalid_o.write(window_ready);
        data_t staged = fetched ? data_pend_q.read() : (addr == 0 ? data_t{0} : m.rdata_i.read());
        p.rdata_o.write(is_fwd ? staged : (window_ready ? data_q.read() : data_t{0}));
    }

    void seq_proc() {
        be_q.write(0);
        valid_q.write(false);
        pending_q.write(false);
        granted_q.write(false);
        fetched_q.write(false);
        primed_q.write(false);
        addr_q.write(0);
        data_q.write(data_t{0});
        data_pend_q.write(data_t{0});
        sh_addr_q.write(0);
        sh_be_q.write(0);
        wait();

        while (true) {
            if constexpr (IS_WRITE) {
                // ---------------------------------------------------------
                // Write mode: primary latch + shadow flush engine — see
                // header comment.
                // ---------------------------------------------------------
                bool     lat     = valid_q.read();
                bool     sh_busy = pending_q.read();
                uint64_t addr    = addr_q.read();
                data_t   dat     = data_q.read();
                uint32_t be      = be_q.read();
                uint64_t sh_addr = sh_addr_q.read();
                data_t   sh_dat  = data_pend_q.read();
                uint32_t sh_be   = sh_be_q.read();

                // Shadow TDM handshake FIRST (pre-snapshot state): the
                // grant ends this shadow's job — the bank fabric samples
                // the payload the edge after the grant, so nothing needs
                // to wait for the rvalid (a stray one finds nothing busy
                // and is ignored). Running this before the snapshot below
                // lets a snapshot land on this very edge, overwriting the
                // payload exactly as the bank consumes it (safe: the bank
                // reads pre-edge values).
                if (sh_busy && m.gnt_i.read())
                    sh_busy = false;

                // Port latch (fill): the window's final fill group is
                // granted the same edge its snapshot fires — the fresh
                // beat must be in the primary before the snapshot below
                // reads it. The parent only grants a group whose cells'
                // primaries are all free, so a p_req with a free latch is
                // an accepted beat.
                if (!lat && p.req_i.read()) {
                    addr = p.addr_i.read();
                    dat  = p.wdata_i.read();
                    be   = p.be_i.read();
                    lat  = true;
                }

                // Snapshot pulse: the whole window is latched and the
                // previous window's shadows are done — hand the primary to
                // the shadow (an addr==0 NOP never touches the bus at all)
                // and free the primary for the next window's fill.
                if (reset_window_i.read() && lat) {
                    if (addr != 0) {
                        sh_addr = addr;
                        sh_dat  = dat;
                        sh_be   = be;
                        sh_busy = true;
                    }
                    lat = false;
                }

                valid_q.write(lat);
                pending_q.write(sh_busy);
                addr_q.write(addr);
                data_q.write(dat);
                be_q.write(be);
                sh_addr_q.write(sh_addr);
                data_pend_q.write(sh_dat);
                sh_be_q.write(sh_be);
            } else {
                // ---------------------------------------------------------
                // Read mode: orthogonal flags — see header comment.
                // ---------------------------------------------------------
                bool     valid         = valid_q.read();
                bool     pending       = pending_q.read();
                bool     granted       = granted_q.read();
                bool     fetched       = fetched_q.read();
                bool     primed        = primed_q.read();
                uint64_t addr          = addr_q.read();
                data_t   dat           = data_q.read();
                data_t   dpend         = data_pend_q.read();
                bool     all_valid_now = all_valid_i.read();

                // Progress on whatever's ALREADY pending, entering this
                // cycle — must happen before the commit/start checks below,
                // which react to what this progress just established. Real
                // fetch's bus handshake — a NOP (addr==0) never touches the
                // bus at all.
                if (pending && addr != 0 && !granted && m.gnt_i.read())
                    granted = true;

                // Stage the response the instant it arrives — see
                // fetched_q's header comment for why this can't just be
                // folded into the commit check below.
                if (pending && !fetched) {
                    if (addr == 0) {
                        dpend   = data_t{0};
                        fetched = true;
                    } else if (granted && m.rvalid_i.read()) {
                        dpend   = m.rdata_i.read();
                        fetched = true;
                    }
                }

                // !valid alone (nothing currently presented) is only safe as
                // an ESCAPE for a cell that has never committed anything yet
                // (the boot path, primed==false) — once primed, always wait
                // for this cell's own group to actually drain (all_valid_i),
                // even if this specific cell is presently !valid. Found via
                // lane_agu: a structurally-always-NOP lane (e.g. a narrow
                // task's unused secondary lane) sits !valid between uses, so
                // its OWN commit would otherwise fire instantly via the bare
                // !valid escape — before the group's real lane(s) are even
                // close to ready, and before the group has actually drained.
                // That NOP lane then gets swept into the PRECEDING drain
                // instead of its own group's, goes !valid again, and is gone
                // by the time the slow real lane finally commits and the
                // group is revisited — permanently dropping that real
                // lane's own response (see lane_agu.hpp's own step_read()
                // comment for the full trace).
                bool safe = (!valid && !primed) || all_valid_now || commit_ok_i.read();

                // Commit and "plain drained" are mutually exclusive
                // (else-if) — see header comment for why.
                if (fetched && safe) {
                    dat     = dpend;
                    valid   = true;
                    pending = false;
                    fetched = false;
                    primed  = true;
                } else if (valid && (all_valid_now || commit_ok_i.read())) {
                    // This cell's own group drained (all_valid_now), or the
                    // post-drain echo is cleaning up a forward-and-commit:
                    // a fetch that resolved AT the drain edge was already
                    // DELIVERED via is_fwd (is_fwd masks data_q on the port,
                    // so the staged beat is what went out) and the commit
                    // above stored that same, now-consumed beat — the echo
                    // clears it. A beat committed AT the echo itself (late
                    // resolve) takes the commit branch instead and survives;
                    // the echo deliberately does NOT reach `start` below —
                    // an echo-started fetch latches whatever the lookahead
                    // bus happens to hold one cycle past the window hand-
                    // off (stale or NOP), and its staged data would then
                    // supersede the real beat at the next drain via the
                    // same is_fwd precedence (found via lane_agu: the last
                    // window's group-1 beat read back 0).
                    valid = false;
                }

                // Start a fetch whenever there's nothing in flight and
                // either this cell's own group is draining right now
                // (all_valid_i — the zero-bubble handoff: the commit above
                // just freed `pending` and set `valid` in this same pass,
                // so `!valid` alone can't see it) or the cell is simply
                // holding nothing (`!valid` — the boot path after reset,
                // AND a parked cell restarting the moment en_i returns
                // after a fence gap; see buffer.hpp's boot_latch for how
                // the parent turns that all-cells restart into the
                // caller's cursor-advance pulse). `pending` is checked
                // AFTER the commit above (not this cycle's entry value) —
                // without that, the zero-bubble case would slip a cycle to
                // the echo pulse, by which point an external cursor
                // tracking window_reset may already have moved on. `start`
                // can never collide with a fetch in flight (pending), and
                // never fires while real data is still being presented
                // (valid, not currently draining) — so nothing is ever
                // overwritten or wiped, which is also why no threshold is
                // needed around en_i gaps: a short dip simply resumes
                // wherever things stood.
                //
                // NOTE (see doc/report §4, phase-7 patterns): a window-batched
                // launch (re-arm all cells on reset_window_i) was
                // prototyped here and parked — it costs single-stream read
                // parity (+2/task via the AGU group-sync interplay). It
                // also turned out unnecessary for the case that motivated
                // it: the shared-bus read slowdown was (a) the stimulus
                // reusing banks between a window's two halves and (b) the
                // adaptive arbiter's then-registered grant losing one bus
                // re-acquisition cycle per window turnaround; with
                // window-distinct addresses and the combinational grant
                // (arbiter_adaptive.hpp) this per-group launch sustains
                // exactly n/lanes.
                bool start = !pending && en_i.read() && (all_valid_now || !valid);
                if (start) {
                    addr    = addr_i.read();
                    pending = true;
                    granted = false;
                    fetched = false;
                }

                valid_q.write(valid);
                pending_q.write(pending);
                granted_q.write(granted);
                fetched_q.write(fetched);
                primed_q.write(primed);
                addr_q.write(addr);
                data_q.write(dat);
                data_pend_q.write(dpend);
            }

            wait();
        }
    }
};
#endif // BUFFER_CELL_HPP
