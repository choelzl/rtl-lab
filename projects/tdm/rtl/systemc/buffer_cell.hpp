// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Single read/write-buffer slot. BYTES_PER_ROW: data width in bytes.
// IS_WRITE=false: read-prefetch cell (fetch ahead, present on drain).
// IS_WRITE=true: accumulate-then-flush write cell, pipelined across windows
// via a primary latch + shadow flush engine.
// -----------------------------------------------------------------------------
//
// Read-mode state (independent flags, not one combined FSM):
//   valid_q/pending_q — presentable now / fetch in flight. Independent on
//   purpose: the next fetch starts off this cell's own drain pulse, so it
//   overlaps with still presenting the CURRENT data (valid_q).
//   granted_q/fetched_q — bus grant seen / response staged in data_pend_q,
//   awaiting a safe promote (m.rvalid_i is one-shot — must be caught here).
//   primed_q — has ever committed (gates the boot escape in `safe` below).
//
// Fetch start: !pending_q && en_i && (all_valid_i || !valid_q). all_valid_i
// is the zero-bubble handoff (this group drains this exact edge); !valid_q
// covers boot and a re-armed parked cell. Never overwrites live data or an
// in-flight fetch, so en_i needs no edge detection.
//
// Completion (staged in data_pend_q/fetched_q) and commit (promoted to
// data_q/valid_q, when fetched_q && safe) are separate steps: under bank
// contention a fetch can complete before it's safe to show. is_fwd (comb)
// lets the parent see the promotion the same cycle, without an extra edge.
//
// Commit and the plain "drained, nothing new" transition are mutually
// exclusive (else-if) — checking both the same pass could flip valid_q on
// then off within one edge, never durably holding VALID for an observable
// cycle.
//
// TDM OBI: req asserted while pending_q && addr_q!=0 && !granted_q; an
// addr=0 NOP never touches the bus.
//
// Write mode: primary latch (port-facing: valid_q/addr_q/data_q/be_q) and
// shadow flush engine (TDM-facing: pending_q/sh_addr_q/data_pend_q/sh_be_q),
// loaded from the primary by reset_window_i so window k's flush overlaps
// window k+1's fill. A shadow's job ends at its grant (bank samples the
// payload the edge after; a stray rvalid finds nothing busy). all_valid_i
// is unused — port acks are posted per snapshotted window, not gated on
// this cell's bank ack.
//
// Port OBI (this cell is subordinate, bundle `p`): p.gnt_o/p.rvalid_o are
// sunk here (buffer.hpp drives them directly); p.we_i is ignored (write
// mode always drives m.we_o=1).
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
    // Read mode: parent's post-drain echo (see buffer.hpp's
    // last_drained_base_q), lets a late-resolving fetch commit safely.
    // Must never clear/start anything — a beat that committed at the drain
    // edge is still undelivered when this arrives. Unused in write mode.
    sc_in<bool> commit_ok_i;
    sc_in<bool> all_valid_i;    // read: this cell's group is draining;
                                // write: unused (acks posted per window)
    sc_in<bool> reset_window_i; // write mode: snapshot pulse, hands primary
                                // to shadow (see header). Unused in read
                                // mode (starts next fetch off all_valid_i).

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
            // data_q included even though its changes co-occur with a
            // fetched_q flip (the commit) — list what's read, not what
            // happens to co-trigger (a missed signal here caused a stuck
            // echo bug in buffer.hpp's comb_proc, tb_buffer T22).
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

        // valid_o: shadow busy. Payload is consumed the edge AFTER the
        // grant, so the shadow is reusable from the grant cycle on —
        // lets back-to-back windows run at crossbar pacing instead of
        // waiting out the bank round trip.
        // invalid_o: primary free; parent gates a fill group's grant on
        // every cell in it being free, so window k+1's fill naturally
        // chases window k's snapshot.
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

        // Has the response arrived — staged (fetched_q), or arriving this
        // exact cycle (NOP, or bank rvalid)?
        bool fetched_now = fetched || (pending && (addr == 0 || (granted && m.rvalid_i.read())));
        // Safe to promote: nothing live to protect, or this cycle drains it.
        // Deliberately NOT seq_proc's stricter safe — tying is_fwd to
        // all_valid_i unconditionally is circular (is_fwd -> cells_valid ->
        // all_valid_i -> is_fwd, a real deadlock, T03a); the register-level
        // fix in seq_proc is what actually stops early commits, so is_fwd's
        // laxer escape here only ever feeds the parent's cells_valid check.
        bool safe   = !valid || all_valid_i.read() || commit_ok_i.read();
        bool is_fwd = fetched_now && safe;

        valid_o.write(valid || is_fwd);
        // Idle: holding and fetching nothing (parked). Parent ANDs this
        // across all cells to detect an all-at-once restart, which needs
        // its own window-reset pulse just like a wraparound (see
        // buffer.hpp's boot_latch). Write mode: "primary latch free" instead.
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

                // Shadow handshake runs before the snapshot below: ends the
                // shadow's job at grant (bank samples the payload the edge
                // after, so a stray rvalid finds nothing busy), letting a
                // same-edge snapshot safely overwrite the payload as the
                // bank consumes it.
                if (sh_busy && m.gnt_i.read())
                    sh_busy = false;

                // Fill: the window's final group is granted the same edge
                // its snapshot fires, so the fresh beat must land in the
                // primary before the snapshot below reads it.
                if (!lat && p.req_i.read()) {
                    addr = p.addr_i.read();
                    dat  = p.wdata_i.read();
                    be   = p.be_i.read();
                    lat  = true;
                }

                // Snapshot: window fully latched, prior shadows done — hand
                // primary to shadow (addr==0 NOP skips the bus) and free
                // primary for the next window's fill.
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

                // Advance an already-in-flight fetch before the commit/start
                // checks below react to it. NOP (addr==0) never touches the bus.
                if (pending && addr != 0 && !granted && m.gnt_i.read())
                    granted = true;

                // Stage the response the instant it arrives (see fetched_q
                // in the header comment).
                if (pending && !fetched) {
                    if (addr == 0) {
                        dpend   = data_t{0};
                        fetched = true;
                    } else if (granted && m.rvalid_i.read()) {
                        dpend   = m.rdata_i.read();
                        fetched = true;
                    }
                }

                // !valid alone is only a safe escape pre-primed (boot); once
                // primed, always wait for this cell's own group to drain
                // (all_valid_i). A structurally-always-NOP lane would
                // otherwise commit instantly via the bare !valid escape,
                // stealing its own group's drain before the real lane is
                // ready (lane_agu bug: dropped the real lane's response).
                bool safe = (!valid && !primed) || all_valid_now || commit_ok_i.read();

                // Commit and "plain drained" are mutually exclusive (else-if).
                if (fetched && safe) {
                    dat     = dpend;
                    valid   = true;
                    pending = false;
                    fetched = false;
                    primed  = true;
                } else if (valid && (all_valid_now || commit_ok_i.read())) {
                    // Group drained, or the post-drain echo cleans up a
                    // forward-and-commit already delivered via is_fwd (the
                    // commit above stored that same consumed beat). The
                    // echo never reaches `start` below — an echo-started
                    // fetch would latch stale/NOP data and supersede the
                    // real beat at the next drain (lane_agu bug).
                    valid = false;
                }

                // Start: pending is checked AFTER the commit above, or the
                // zero-bubble case slips a cycle behind the echo pulse. A
                // window-batched launch variant was tried and dropped (cost
                // single-stream read parity, doc/report §4).
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
