// -----------------------------------------------------------------------------
// Unit tests for lane_agu<> driving a standalone buffer<4,1,...> instance
// directly (not the full top_tdm wrapper) — focused on lane_agu's own
// correctness: independent per-sub-port progress, width-driven lane usage,
// and the "always assert req on all 4 lanes" invariant that keeps one
// sub-port's idle/exhausted lanes from deadlocking the other.
//
// Test cases push dma_task_t entries directly into lane_agu::subports_
// (a public member — see lane_agu.hpp), bypassing file I/O so cases don't
// need fragile on-disk trace files.
//
// The write buffer's TDM-facing side is wired to real bank<> instances (one
// per TDM slot, no address-based multiplexing — this test isn't about bank
// routing/conflicts, just about lane_agu presenting the right addr/data/be to
// the right physical lanes). Each of the 4 real lanes maps to its own
// dedicated bank instance across the whole test, so pre-loading known data
// at a lane's bank (for the read test) naturally supports that sub-port
// issuing many different addresses over time — different addresses just
// land on different rows within the same bank instance.
// -----------------------------------------------------------------------------

#include "bank.hpp"
#include "buffer.hpp"
#include "lane_agu.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <systemc.h>

static constexpr int      NUM_TDM = 8; // small window (multiple of NUM_IO=4) for fast tests
static constexpr uint32_t FULL_BE = 0xFFFF;
using data_t                      = obi_data<16>;
using dma_t                       = lane_agu<data_t, 16>;
using task_t                      = dma_t::dma_task_t;

struct expected_entry_t {
    uint64_t addr;
    data_t   data;
};

// The exact (addr,data) set build_case() below is expected to produce for
// one dense+sparse case, independent of which sub-port plays which role —
// shared by both write and read verification so the two can't drift apart.
static std::vector<expected_entry_t> expected_entries_for_case(bool dense_is_sp0) {
    std::vector<expected_entry_t> out;
    const uint64_t                dense_base  = dense_is_sp0 ? 0x1000 : 0x9000;
    const uint64_t                sparse_base = dense_is_sp0 ? 0x9000 : 0x1000;
    for (int i = 0; i < 20; ++i) {
        const uint64_t base = dense_base + static_cast<uint64_t>(i) * 0x20;
        out.push_back({base, make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i))});
        // i==10: width=32 (full second beat); i==15: width=24 (PARTIAL second
        // beat — regression-locks the width>BYTES_PER_BEAT-but-not-exactly-
        // 2*BYTES_PER_BEAT case added alongside the DMA width fix).
        if (i == 10 || i == 15)
            out.push_back(
                {base + 16, make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i) + 0x100u)});
    }
    out.push_back({sparse_base, make_row<data_t>(0xE0000000u)});
    out.push_back({sparse_base + 0x20, make_row<data_t>(0xE0000001u)});
    out.push_back({sparse_base + 0x20 + 16, make_row<data_t>(0xE0000101u)});
    return out;
}

// Strict verification of a lane_agu log against the exact expected set:
//   - every expected (addr,data) pair appears EXACTLY once (catches both
//     missing entries and duplicate/double-logged entries)
//   - the log's total we-matching entry count equals the expected count
//     exactly (catches spurious extra entries not caught by the pair check)
//   - every we-matching entry's address falls inside one of the two known
//     fingerprint windows (catches wrong-lane/wrong-bank routing leaks that
//     happen to collide with a real address elsewhere)
template <typename LogT>
static bool verify_log_exact(const LogT &log, bool we_expected,
                             const std::vector<expected_entry_t> &expected) {
    bool ok = true;
    for (const auto &e : expected) {
        int cnt = 0;
        for (const auto &a : log)
            if (a.we == we_expected && a.addr == e.addr && a.data == e.data)
                ++cnt;
        if (cnt != 1) {
            std::printf("    mismatch: addr=0x%llx expected exactly 1 occurrence, found %d\n",
                        static_cast<unsigned long long>(e.addr), cnt);
            ok = false;
        }
    }
    std::size_t match_count = 0;
    for (const auto &a : log) {
        if (a.we != we_expected)
            continue;
        ++match_count;
        const bool in_sp0 = a.addr >= 0x1000 && a.addr < 0x2000;
        const bool in_sp1 = a.addr >= 0x9000 && a.addr < 0xA000;
        if (!in_sp0 && !in_sp1)
            std::printf("    entry at addr=0x%llx falls outside expected fingerprint ranges\n",
                        static_cast<unsigned long long>(a.addr));
        ok &= (in_sp0 || in_sp1);
    }
    if (match_count != expected.size()) {
        std::printf("    log has %zu matching entries, expected exactly %zu\n", match_count,
                    expected.size());
        ok = false;
    }
    return ok;
}

// Confirms the delayed sparse task (start_cycle=500) was not serviced before
// its own fence — a real regression here would mean decide_content()/
// step_read() ignored start_cycle and let a not-yet-ready task through.
template <typename LogT>
static bool verify_not_early(const LogT &log, bool we_expected, uint64_t addr, uint64_t min_cycle) {
    for (const auto &a : log)
        if (a.we == we_expected && a.addr == addr)
            return a.cycle >= min_cycle;
    return false; // not found at all — also a failure, reported by verify_log_exact
}

// Plain signal group for one N-lane pre-load control (no ports, no
// submodule — just what the test pokes directly via preload_bank() below).
// Reused for both the TDM read test (N=NUM_TDM) and the crossbar read test
// (N=4); which one is irrelevant to the signals themselves.
template <int N> struct preload_ctrl_t {
    // Deliberately unnamed (SystemC auto-generates unique names) — this
    // struct is instantiated twice (r_preload, rx_preload) under the same
    // parent module, so a hardcoded name here would collide between them.
    sc_signal<bool>     active;
    sc_signal<bool>     req[N];
    sc_signal<uint64_t> addr[N];
    sc_signal<data_t>   wdata[N];
};

// One lane's "pre-load vs real driver" decision — the actual logic shared by
// mux_read_banks() and mux_xbar_banks() below, so it exists in exactly one
// place regardless of which real component (buf_r, or dma_rx directly) sits
// on the driver side. While pre-loading, the bank sees the test's
// preload_req/addr/wdata instead of the real driver, and the real driver
// sees gnt/rvalid/rdata held low/blank so it can't mistake pre-load traffic
// for its own request being serviced.
struct mux_lane_t {
    bool     bank_req, bank_we;
    uint64_t bank_addr;
    uint32_t bank_be;
    data_t   bank_wdata;
    bool     drv_gnt, drv_rvalid;
    data_t   drv_rdata;
};

static mux_lane_t mux_lane(bool pre, bool preload_req, uint64_t preload_addr, data_t preload_wdata,
                           bool drv_req, uint64_t drv_addr, bool drv_we, uint32_t drv_be,
                           data_t drv_wdata, bool bank_gnt, bool bank_rvalid, data_t bank_rdata) {
    mux_lane_t o;
    if (pre) {
        o.bank_req   = preload_req;
        o.bank_addr  = preload_addr;
        o.bank_we    = true;
        o.bank_be    = FULL_BE;
        o.bank_wdata = preload_wdata;
    } else {
        o.bank_req   = drv_req;
        o.bank_addr  = drv_addr;
        o.bank_we    = drv_we;
        o.bank_be    = drv_be;
        o.bank_wdata = drv_wdata;
    }
    o.drv_gnt    = pre ? false : bank_gnt;
    o.drv_rvalid = pre ? false : bank_rvalid;
    o.drv_rdata  = pre ? data_t{} : bank_rdata;
    return o;
}

SC_MODULE(tb) {
    sc_clock        clk{"clk", 10, SC_NS};
    sc_signal<bool> rst_n{"rst_n"};

    // ---- write-side scaffolding: buffer<4,1,16,NUM_TDM,true> + lane_agu ----
    buffer<4, 1, 16, NUM_TDM, true> buf_w{"buf_w"};
    dma_t                           dma_w{"dma_w", "", "", lane_agu_dir::write, NUM_TDM};

    sc_signal<uint32_t>       w_active_mode{"w_active_mode"};
    sc_signal<bool>           w_fetch_valid{"w_fetch_valid"};
    sc_signal<uint64_t>       w_fetch_addr[NUM_TDM];
    sc_signal<bool>           w_done{"w_done"};
    obi_signal_bundle<data_t> w[4];
    obi_signal_bundle<data_t> w_m[NUM_TDM];
    sc_vector<bank<1024, 16>> w_banks{"w_banks"};

    // ---- read-side scaffolding: buffer<4,1,16,NUM_TDM,false> + lane_agu ----
    buffer<4, 1, 16, NUM_TDM, false> buf_r{"buf_r"};
    dma_t                            dma_r{"dma_r", "", "", lane_agu_dir::read, NUM_TDM};

    sc_signal<bool>     r_done{"r_done"};
    sc_signal<uint32_t> r_active_mode{"r_active_mode"};
    sc_signal<bool>     r_fetch_valid{"r_fetch_valid"};
    // All NUM_TDM slots driven every cycle from dma_r.lookahead_addr(w)
    // (see run_read()): with multi-group window packing the driver stages
    // up to NUM_TDM/4 tasks per sub-port per window, so slots past [0..3]
    // carry the later groups' addresses too.
    sc_signal<uint64_t> r_fetch_addr[NUM_TDM];
    // r_obi.we/be/wdata are dma_r's always-false/don't-care read-mode outputs,
    // still need a bound sink since lane_agu writes them every cycle.
    obi_signal_bundle<data_t> r_obi[4];
    // Bank-facing signals (final, muxed) — what r_banks[] actually see —
    // and buf_r's own TDM-facing drive, which goes to "_drv" signals, NOT
    // directly to the bank-facing ones (mux_read_banks() combines them; see
    // mux_lane()'s comment for why a pre-load can't just share the driver's
    // own signal).
    obi_signal_bundle<data_t> r_bank[NUM_TDM];
    obi_signal_bundle<data_t> r_drv[NUM_TDM];
    preload_ctrl_t<NUM_TDM>   r_preload;
    sc_vector<bank<1024, 16>> r_banks{"r_banks"};

    // ---- crossbar-read scaffolding: lane_agu(target=crossbar) driving 4
    // bank<> instances DIRECTLY (no buffer<> in between) — mirrors how the
    // crossbar backend actually wires DMA (no TDM-specific buffering at
    // all), unlike buf_r above which is TDM-only. dma_rx plays the "driver"
    // role itself (no buf_r in front of it), so its req/addr/we/be/wdata
    // outputs (always we=false/don't-care be,wdata in read mode) feed the
    // mux directly as rx_drv_*.
    dma_t dma_rx{"dma_rx", "", "", lane_agu_dir::read, /*tdm_window=*/0, lane_agu_target::crossbar};
    sc_signal<bool>           rx_done{"rx_done"};
    obi_signal_bundle<data_t> rx_bank[4];
    obi_signal_bundle<data_t> rx_drv[4];
    preload_ctrl_t<4>         rx_preload;
    sc_vector<bank<1024, 16>> xbar_banks{"xbar_banks"};

    SC_HAS_PROCESS(tb);
    tb(sc_module_name nm) : sc_module(nm) {
        w_banks.init(NUM_TDM);
        r_banks.init(NUM_TDM);

        // ---- write buffer wiring ----
        buf_w.clk_i(clk);
        buf_w.rst_ni(rst_n);
        buf_w.active_mode(w_active_mode);
        buf_w.fetch_addr_valid_i(w_fetch_valid);
        for (int p = 0; p < 4; ++p) {
            bind_obi(dma_w.obi[p], w[p]);

            bind_obi(buf_w.p[p], w[p]);
        }
        for (int t = 0; t < NUM_TDM; ++t)
            buf_w.fetch_addr_i[t](w_fetch_addr[t]); // unused in write mode
        dma_w.clk_i(clk);
        dma_w.rst_ni(rst_n);
        dma_w.done_o(w_done);

        for (int t = 0; t < NUM_TDM; ++t) {
            bind_obi(buf_w.m[t], w_m[t]);

            w_banks[t].clk_i(clk);
            w_banks[t].rst_ni(rst_n);
            bind_obi(w_banks[t].obi, w_m[t]);
        }

        // ---- read buffer wiring ----
        buf_r.clk_i(clk);
        buf_r.rst_ni(rst_n);
        buf_r.active_mode(r_active_mode);
        buf_r.fetch_addr_valid_i(r_fetch_valid);
        for (int p = 0; p < 4; ++p) {
            bind_obi(dma_r.obi[p], r_obi[p]);

            // r_obi is a full bundle; the addr/be/wdata members double as
            // the previously-dedicated dead wiring for read mode
            bind_obi(buf_r.p[p], r_obi[p]);
        }
        for (int t = 0; t < NUM_TDM; ++t)
            buf_r.fetch_addr_i[t](r_fetch_addr[t]); // [0..3] real, [4..7] tied to 0
        dma_r.clk_i(clk);
        dma_r.rst_ni(rst_n);
        dma_r.done_o(r_done);

        // buf_r's own drive goes to the "_drv" signals, NOT directly to the
        // bank-facing "_bank" wires — see mux_read_banks()/mux_lane().
        for (int t = 0; t < NUM_TDM; ++t) {
            bind_obi(buf_r.m[t], r_drv[t]);

            r_banks[t].clk_i(clk);
            r_banks[t].rst_ni(rst_n);
            bind_obi(r_banks[t].obi, r_bank[t]);
        }
        SC_METHOD(mux_read_banks);
        for (int t = 0; t < NUM_TDM; ++t) {
            sensitive << r_preload.active << r_preload.req[t] << r_preload.addr[t]
                      << r_preload.wdata[t] << r_drv[t].req << r_drv[t].addr << r_drv[t].we
                      << r_drv[t].be << r_drv[t].wdata << r_bank[t].gnt << r_bank[t].rvalid
                      << r_bank[t].rdata;
        }

        // ---- crossbar-read wiring: dma_rx drives 4 banks DIRECTLY (no
        // buffer<> in between), matching the crossbar backend's actual DMA
        // wiring. Same pre-load isolation as the TDM read side (mux_lane()).
        xbar_banks.init(4);
        dma_rx.clk_i(clk);
        dma_rx.rst_ni(rst_n);
        dma_rx.done_o(rx_done);
        for (int p = 0; p < 4; ++p) {
            bind_obi(dma_rx.obi[p], rx_drv[p]);

            xbar_banks[p].clk_i(clk);
            xbar_banks[p].rst_ni(rst_n);
            bind_obi(xbar_banks[p].obi, rx_bank[p]);
        }
        SC_METHOD(mux_xbar_banks);
        for (int p = 0; p < 4; ++p) {
            sensitive << rx_preload.active << rx_preload.req[p] << rx_preload.addr[p]
                      << rx_preload.wdata[p] << rx_drv[p].req << rx_drv[p].addr << rx_drv[p].we
                      << rx_drv[p].be << rx_drv[p].wdata << rx_bank[p].gnt << rx_bank[p].rvalid
                      << rx_bank[p].rdata;
        }

        SC_THREAD(run);
    }

    // Combinational mux for the TDM read test — see mux_lane()'s comment for
    // the actual logic; this just plumbs signals in and out per lane.
    void mux_read_banks() {
        const bool pre = r_preload.active.read();
        for (int t = 0; t < NUM_TDM; ++t) {
            const auto o =
                mux_lane(pre, r_preload.req[t].read(), r_preload.addr[t].read(),
                         r_preload.wdata[t].read(), r_drv[t].req.read(), r_drv[t].addr.read(),
                         r_drv[t].we.read(), r_drv[t].be.read(), r_drv[t].wdata.read(),
                         r_bank[t].gnt.read(), r_bank[t].rvalid.read(), r_bank[t].rdata.read());
            r_bank[t].req.write(o.bank_req);
            r_bank[t].addr.write(o.bank_addr);
            r_bank[t].we.write(o.bank_we);
            r_bank[t].be.write(o.bank_be);
            r_bank[t].wdata.write(o.bank_wdata);
            r_drv[t].gnt.write(o.drv_gnt);
            r_drv[t].rvalid.write(o.drv_rvalid);
            r_drv[t].rdata.write(o.drv_rdata);
        }
    }

    // Same as mux_read_banks(), for the direct-to-bank crossbar-read
    // scaffolding (dma_rx/xbar_banks).
    void mux_xbar_banks() {
        const bool pre = rx_preload.active.read();
        for (int p = 0; p < 4; ++p) {
            const auto o =
                mux_lane(pre, rx_preload.req[p].read(), rx_preload.addr[p].read(),
                         rx_preload.wdata[p].read(), rx_drv[p].req.read(), rx_drv[p].addr.read(),
                         rx_drv[p].we.read(), rx_drv[p].be.read(), rx_drv[p].wdata.read(),
                         rx_bank[p].gnt.read(), rx_bank[p].rvalid.read(), rx_bank[p].rdata.read());
            rx_bank[p].req.write(o.bank_req);
            rx_bank[p].addr.write(o.bank_addr);
            rx_bank[p].we.write(o.bank_we);
            rx_bank[p].be.write(o.bank_be);
            rx_bank[p].wdata.write(o.bank_wdata);
            rx_drv[p].gnt.write(o.drv_gnt);
            rx_drv[p].rvalid.write(o.drv_rvalid);
            rx_drv[p].rdata.write(o.drv_rdata);
        }
    }

    // Replaces every bare tick(clk) in this file: dma_r's own step() (and so
    // buf_r's window cycling) runs on EVERY clock edge regardless of which
    // run_*()/preload helper happens to be "in charge" right now — dma_r
    // always holds req high on all 4 lanes (a hard TDM-buffer requirement,
    // see lane_agu.hpp's class-level comment), so buf_r's window keeps
    // resetting and refetching (as fast as NOP resolves) even during
    // do_reset()/preload_case(), well before run_read() itself starts. If
    // the lookahead cursor only started listening once run_read() began, it
    // would catch a reset already in progress for unrelated reasons and
    // desync its bookkeeping from the very first cycle it looked. Tracking
    // this from the first tick onward (matching stim_bank_common.hpp's
    // pattern, which starts its own polling loop from right after reset
    // too) keeps la_idx_ correctly aligned to every REAL window transition,
    // not just the ones that happen to occur while run_read() is running.
    void tick_r() {
        if (buf_r.snapshot().window_reset)
            dma_r.advance_lookahead_window();
        for (int w = 0; w < NUM_TDM; ++w)
            r_fetch_addr[w].write(dma_r.lookahead_addr(w));
        tick(clk);
    }

    void do_reset() {
        rst_n.write(false);
        w_active_mode.write(0);
        w_fetch_valid.write(false);
        r_active_mode.write(0);
        r_fetch_valid.write(true); // read cells always continuously prefetch
        for (int t = 0; t < NUM_TDM; ++t) {
            w_fetch_addr[t].write(0);
            r_fetch_addr[t].write(0); // re-driven from dma_r.lookahead_addr() by run_read()
        }
        tick_r();
        tick_r();
        rst_n.write(true);
        tick_r();
    }

    // Pre-loads one bank's row at `addr` with `data` — must only be called
    // while that preload_ctrl_t's `active` is set, so the write reaches the
    // bank without ever touching a signal the real driver (buf_r, or
    // dma_rx) itself drives. Shared by both the TDM read test (ctrl=
    // r_preload) and the crossbar read test (ctrl=rx_preload).
    template <int N>
    void preload_bank(preload_ctrl_t<N> & ctrl, int bank_idx, uint64_t addr, data_t data) {
        ctrl.req[bank_idx].write(true);
        ctrl.addr[bank_idx].write(addr);
        ctrl.wdata[bank_idx].write(data);
        tick_r();
        ctrl.req[bank_idx].write(false);
    }

    // Pre-loads both banks for whichever sub-port is dense and whichever is
    // sparse, deriving each lane's bank index from its sub-port number
    // (primary = sub_port*2, secondary = sub_port*2+1) rather than a
    // hand-picked literal — this is precisely the class of mistake that
    // previously broke T03 (a hardcoded bank index pointed at the wrong
    // sub-port's lane), so computing it structurally rules that class out.
    // With multi-group window packing (see lane_agu.hpp), a sub-port's task
    // fetches through slot g*4 + lane for whichever group g the packer put
    // it in — which depends on runtime fence/readiness dynamics. Each TDM
    // slot has its own bank instance in this scaffolding, so the preload
    // replicates every task's row into ALL bank instances its lane can map
    // to (one per group) instead of assuming group 0.
    template <int N>
    void preload_lane_all_groups(preload_ctrl_t<N> & ctrl, int lane, uint64_t addr,
                                 const data_t &val) {
        for (int g = 0; g * 4 < N; ++g)
            preload_bank(ctrl, g * 4 + lane, addr, val);
    }

    template <int N> void preload_case(preload_ctrl_t<N> & ctrl, bool dense_is_sp0) {
        const int      dense_sp    = dense_is_sp0 ? 0 : 1;
        const int      sparse_sp   = dense_is_sp0 ? 1 : 0;
        const int      dense_pri   = dense_sp * 2;
        const int      dense_sec   = dense_pri + 1;
        const int      sparse_pri  = sparse_sp * 2;
        const int      sparse_sec  = sparse_pri + 1;
        const uint64_t dense_base  = dense_is_sp0 ? 0x1000 : 0x9000;
        const uint64_t sparse_base = dense_is_sp0 ? 0x9000 : 0x1000;

        ctrl.active.write(true);
        for (int i = 0; i < 20; ++i) {
            const uint64_t addr = dense_base + static_cast<uint64_t>(i) * 0x20;
            preload_lane_all_groups(ctrl, dense_pri, addr,
                                    make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i)));
            if (i == 10 || i == 15)
                preload_lane_all_groups(
                    ctrl, dense_sec, addr + 16,
                    make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i) + 0x100u));
        }
        preload_lane_all_groups(ctrl, sparse_pri, sparse_base, make_row<data_t>(0xE0000000u));
        preload_lane_all_groups(ctrl, sparse_pri, sparse_base + 0x20,
                                make_row<data_t>(0xE0000001u));
        preload_lane_all_groups(ctrl, sparse_sec, sparse_base + 0x20 + 16,
                                make_row<data_t>(0xE0000101u));
        ctrl.active.write(false);
        tick_r();
    }

    // Runs dma_w against buf_w until done_o (or a cycle budget is exhausted).
    // Returns the number of cycles taken; -1 if it never finished.
    int run_write(int max_cycles) {
        for (int c = 0; c < max_cycles; ++c) {
            tick_r();
            if (w_done.read())
                return c + 1;
        }
        return -1;
    }

    int run_read(int max_cycles) {
        for (int c = 0; c < max_cycles; ++c) {
            tick_r();
            if (r_done.read())
                return c + 1;
        }
        return -1;
    }

    int run_rx(int max_cycles) {
        for (int c = 0; c < max_cycles; ++c) {
            tick_r();
            if (rx_done.read())
                return c + 1;
        }
        return -1;
    }

    // Builds Case A (sub_port0 dense, sub_port1 sparse) or Case B (reversed)
    // task lists directly into dma.subports_[]. Sub-port 0's addresses live
    // in [0x1000,0x2000), sub-port 1's in [0x9000,0xA000) — non-overlapping
    // and easy to fingerprint, so a wrong-lane routing bug shows up as an
    // address/data mismatch rather than an accidental pass. Each dense
    // sub-port gets one width=32 (wide) task among its narrow ones.
    static void build_case(dma_t & dma, bool write, bool sp0_dense) {
        // dma_w/dma_r are reused across multiple test cases (reset_state()
        // only zeroes idx — by design, a real reset restarts a DMA transfer
        // from scratch without needing a fresh trace load — it does NOT
        // clear tasks). build_case() must therefore start from a clean slate
        // itself, or a second call would append onto the previous case's
        // stale (now unconsumed-again) tasks and replay them alongside the
        // new case.
        for (auto &s : dma.subports_) {
            s.tasks.clear();
            s.idx = 0;
        }
        const int      dense_sp    = sp0_dense ? 0 : 1;
        const int      sparse_sp   = sp0_dense ? 1 : 0;
        const uint64_t dense_base  = dense_sp == 0 ? 0x1000 : 0x9000;
        const uint64_t sparse_base = sparse_sp == 0 ? 0x1000 : 0x9000;

        // Dense sub-port: 20 narrow (width=16) tasks at consecutive cycles,
        // plus one wide (width=32, full second beat) task and one PARTIAL
        // wide (width=24, second beat only half-real) task partway through —
        // the latter regression-locks the width>BYTES_PER_BEAT-but-not-
        // exactly-2*BYTES_PER_BEAT case.
        for (int i = 0; i < 20; ++i) {
            task_t t;
            t.start_cycle = static_cast<uint64_t>(i);
            t.addr        = dense_base + static_cast<uint64_t>(i) * 0x20;
            t.width       = (i == 10) ? 32 : (i == 15) ? 24 : 16;
            t.we          = write;
            if (write) {
                t.data_lo = make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i));
                if (t.width > 16)
                    t.data_hi = make_row<data_t>(0xD0000000u + static_cast<uint32_t>(i) + 0x100u);
            }
            dma.subports_[dense_sp].tasks.push_back(t);
        }
        // Sparse sub-port: 2 tasks, one immediate, one far in the future —
        // exercises "the other sub-port must not stall waiting for this one".
        {
            task_t t0;
            t0.start_cycle = 0;
            t0.addr        = sparse_base;
            t0.width       = 16;
            t0.we          = write;
            if (write)
                t0.data_lo = make_row<data_t>(0xE0000000u);
            dma.subports_[sparse_sp].tasks.push_back(t0);

            task_t t1;
            t1.start_cycle = 500; // far after the dense sub-port would finish
            t1.addr        = sparse_base + 0x20;
            t1.width       = 32;
            t1.we          = write;
            if (write) {
                t1.data_lo = make_row<data_t>(0xE0000001u);
                t1.data_hi = make_row<data_t>(0xE0000101u);
            }
            dma.subports_[sparse_sp].tasks.push_back(t1);
        }
    }

    void run() {
        // ═══════════════════════════════════════════════════════════════
        // Write-side: Case A (sub_port0 dense, sub_port1 sparse)
        // ═══════════════════════════════════════════════════════════════
        std::puts("\n=== T01: Write — sub_port0 dense, sub_port1 sparse ===");
        // build_case() before do_reset(): do_reset()'s own final tick runs
        // one full step() with rst_ni already true, one cycle before this
        // call would otherwise repopulate subports_ — if that tick still saw
        // a PREVIOUS case's stale tasks (idx freshly zeroed by reset_state(),
        // tasks not yet cleared), a phantom grant against old data could
        // fire before the new case is even loaded. Loading tasks first means
        // subports_ already holds the correct case throughout the reset
        // pulse, so no such window exists.
        build_case(dma_w, /*write=*/true, /*sp0_dense=*/true);
        do_reset();

        const int w_cycles_a = run_write(2000);
        CHECK(w_cycles_a > 0, "T01a write completes without deadlock (dense+sparse)");

        // Expected (addr,data) pairs: 19 narrow + 1 wide(2 entries) for
        // sub_port0, 1 narrow + 1 wide(2 entries) for sub_port1 = 24 total.
        // verify_log_exact demands each appear EXACTLY once, the log's total
        // write-entry count matches exactly (no spurious extras), and every
        // entry's address falls in a known fingerprint window.
        const auto t01_expected = expected_entries_for_case(/*dense_is_sp0=*/true);
        CHECK(verify_log_exact(dma_w.log_, /*we_expected=*/true, t01_expected),
              "T01b all expected (addr,data) pairs present exactly once, no extras/leaks");
        CHECK(
            verify_not_early(dma_w.log_, /*we_expected=*/true, /*addr=*/0x9020, /*min_cycle=*/500),
            "T01b2 delayed sparse task (start_cycle=500) not serviced early");

        // Ordering: sub_port0's own PRIMARY-lane entries (addr aligned to
        // dense_base + k*0x20, i.e. excluding a wide task's secondary-lane
        // half) must appear in non-decreasing address order — this is what
        // the design actually guarantees (each lane's own inflight queue is
        // FIFO). The secondary lane is a separate, independently-progressing
        // queue (only ever carries one entry per wide task), so its entry's
        // completion relative to LATER primary-lane entries isn't ordered
        // by address — that's not a correctness bug, just two independent
        // lanes; T01b already confirms both halves land with the right data.
        {
            uint64_t last     = 0;
            bool     order_ok = true;
            for (const auto &a : dma_w.log_)
                if (a.we && a.addr >= 0x1000 && a.addr < 0x2000 && (a.addr - 0x1000) % 0x20 == 0) {
                    if (a.addr < last)
                        order_ok = false;
                    last = a.addr;
                }
            CHECK(order_ok, "T01c sub_port0's own writes preserve task order");
        }
        std::printf("  (info: write took %d cycles)\n", w_cycles_a);

        // ═══════════════════════════════════════════════════════════════
        // Write-side: Case B (reversed — sub_port1 dense, sub_port0 sparse)
        // ═══════════════════════════════════════════════════════════════
        std::puts("\n=== T02: Write — sub_port1 dense, sub_port0 sparse (reversed) ===");
        build_case(dma_w, /*write=*/true, /*sp0_dense=*/false);
        do_reset();

        const int w_cycles_b = run_write(2000);
        CHECK(w_cycles_b > 0, "T02a write completes without deadlock (reversed)");

        const auto t02_expected = expected_entries_for_case(/*dense_is_sp0=*/false);
        CHECK(verify_log_exact(dma_w.log_, /*we_expected=*/true, t02_expected),
              "T02b all expected pairs present exactly once, no extras/leaks (reversed roles)");
        CHECK(
            verify_not_early(dma_w.log_, /*we_expected=*/true, /*addr=*/0x1020, /*min_cycle=*/500),
            "T02b2 delayed sparse task (start_cycle=500) not serviced early (reversed roles)");

        // ═══════════════════════════════════════════════════════════════
        // Read-side: Case A (sub_port0 dense, sub_port1 sparse)
        // ═══════════════════════════════════════════════════════════════
        std::puts("\n=== T03: Read — sub_port0 dense, sub_port1 sparse ===");
        // build_case() before do_reset() for the same reason as T01/T02
        // above; preload_case() must stay AFTER do_reset() since it needs
        // the banks (and buf_r's mux) out of reset to accept writes.
        build_case(dma_r, /*write=*/false, /*sp0_dense=*/true);
        do_reset();
        preload_case(r_preload, /*dense_is_sp0=*/true);

        const int r_cycles_a = run_read(4000);
        CHECK(r_cycles_a > 0, "T03a read completes without deadlock (dense+sparse)");

        const auto t03_expected = expected_entries_for_case(/*dense_is_sp0=*/true);
        CHECK(verify_log_exact(dma_r.log_, /*we_expected=*/false, t03_expected),
              "T03b all expected read pairs present exactly once, no extras/leaks");
        std::printf("  (info: read took %d cycles)\n", r_cycles_a);

        // ═══════════════════════════════════════════════════════════════
        // Read-side: Case B (reversed — sub_port1 dense, sub_port0 sparse)
        // ═══════════════════════════════════════════════════════════════
        std::puts("\n=== T04: Read — sub_port1 dense, sub_port0 sparse (reversed) ===");
        build_case(dma_r, /*write=*/false, /*sp0_dense=*/false);
        do_reset();
        preload_case(r_preload, /*dense_is_sp0=*/false);

        const int r_cycles_b = run_read(4000);
        CHECK(r_cycles_b > 0, "T04a read completes without deadlock (reversed)");

        const auto t04_expected = expected_entries_for_case(/*dense_is_sp0=*/false);
        CHECK(
            verify_log_exact(dma_r.log_, /*we_expected=*/false, t04_expected),
            "T04b all expected read pairs present exactly once, no extras/leaks (reversed roles)");
        std::printf("  (info: read took %d cycles)\n", r_cycles_b);

        // ═══════════════════════════════════════════════════════════════
        // Read-side, crossbar target: sub_port0 dense, sub_port1 sparse.
        // dma_rx drives 4 bank<> instances DIRECTLY (no buffer<>/windowing)
        // — exercises the one-shot crossbar read path added alongside the
        // system-harness wiring (system_stimuli_common.hpp's IMPL_CROSSBAR
        // backend has no DMA-specific buffering at all).
        // ═══════════════════════════════════════════════════════════════
        std::puts("\n=== T05: Read (crossbar target) — sub_port0 dense, sub_port1 sparse ===");
        build_case(dma_rx, /*write=*/false, /*sp0_dense=*/true);
        do_reset();
        preload_case(rx_preload, /*dense_is_sp0=*/true);

        const int rx_cycles = run_rx(4000);
        CHECK(rx_cycles > 0, "T05a crossbar read completes without deadlock (dense+sparse)");

        const auto t05_expected = expected_entries_for_case(/*dense_is_sp0=*/true);
        CHECK(verify_log_exact(dma_rx.log_, /*we_expected=*/false, t05_expected),
              "T05b all expected crossbar read pairs present exactly once, no extras/leaks");
        std::printf("  (info: crossbar read took %d cycles)\n", rx_cycles);

        // ═══════════════════════════════════════════════════════════════
        sc_stop();
    }
};

int sc_main(int, char **) {
    tb t{"tb"};
    sc_start();
    return report_and_exit();
}
