// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Native SystemC design top (DUT) for the TDM read/write path.
//
// Pipeline: RPORT groups -> buf_r{0..4} -+
//           WPORT groups -> buf_w{0..3} -+-> mux (arbiter sel) -> tdm -> crossbar -> bank[NUM_BANK]
//
// 5 read buffers (PORT_COUNT 4,2,1,1,1 = RAGU_A..E) + 4 write buffers
// (PORT_COUNT 4,2,1,1 = WAGU_A,B,D,E), all NUM_TDM=NUM_BANK. active_mode
// per buffer (0/1/2 -> 1/2/4 active port groups) comes from stimuli's
// ports_used_groups field; buffer indices 0..4=r, 5..8=w (enum buf_client).
// -----------------------------------------------------------------------------
//
// Each read buffer's fetch_addr_i/fetch_addr_valid_i is wired to its own
// AGU's lookahead bus (agu.hpp's lookahead_addr/lookahead_ready), which
// already knows its next NUM_BANK addresses — so a whole window can
// prefetch at once and only the first group after a window reset pays the
// TDM round trip. Write buffers ignore fetch_addr_i.
//
// The arbiter round-robins across all 9 buffers; mux_comb routes the
// selected buffer's OBI to/from the TDM module and steers gnt/rvalid/rdata
// back. IMPL_ARB_ADAPTIVE (off by default) swaps in arbiter_adaptive.hpp,
// which skips requestless clients instead of visiting all 9 every cycle —
// same sel_req_o/sel_rsp_o interface either way.
//
// Template params: NUM_RPORT (>=9, 4+2+1+1+1 split), NUM_WPORT (>=8,
// 4+2+1+1 split), NUM_REQ (OBI buses/port), NUM_BANK (TDM width/window
// size), NUM_ROW, BYTES_PER_WORD, WORDS_PER_ROW.
// -----------------------------------------------------------------------------

#ifndef TOP_TDM_HPP
#define TOP_TDM_HPP

#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <systemc.h>

#if defined(IMPL_ARB_ADAPTIVE)
#include "arbiter_adaptive.hpp"
#else
#include "arbiter.hpp"
#endif
#include "bank.hpp"
#include "buffer.hpp"
#include "crossbar.hpp"
#include "obi_data.hpp"
#include "obi_monitor.hpp"
#include "obi_ports.hpp"
#include "tdm.hpp"

template <int NUM_RPORT = 9, int NUM_WPORT = 8, int NUM_REQ = 4, int NUM_BANK = 32,
          int NUM_ROW = 1024, int BYTES_PER_WORD = 4, int WORDS_PER_ROW = 4>
SC_MODULE(top_tdm) {
    static_assert(NUM_RPORT >= 9, "top_tdm maps 9 read port groups (4+2+1+1+1)");
    static_assert(NUM_WPORT >= 8, "top_tdm maps 8 write port groups (4+2+1+1)");

    static constexpr int NUM_RD_BUF    = 5;
    static constexpr int NUM_WR_BUF    = 4;
    static constexpr int NUM_TOTAL_BUF = NUM_RD_BUF + NUM_WR_BUF;

    // Arbiter client indices (buffer order: reads then writes)
    enum buf_client {
        BUF_RAGU_A = 0,
        BUF_RAGU_B = 1,
        BUF_RAGU_C = 2,
        BUF_RAGU_D = 3,
        BUF_RAGU_E = 4,
        BUF_WAGU_A = 5,
        BUF_WAGU_B = 6,
        BUF_WAGU_D = 7,
        BUF_WAGU_E = 8,
    };
    static constexpr int NUM_OBI_REQ     = NUM_REQ;
    static constexpr int NUM_RPORT_PORTS = NUM_RPORT * NUM_REQ;
    static constexpr int NUM_WPORT_PORTS = NUM_WPORT * NUM_REQ;
    static constexpr int BYTES_PER_ROW   = WORDS_PER_ROW * BYTES_PER_WORD;
    static constexpr int NUM_BUF_TDM     = NUM_TOTAL_BUF * NUM_BANK;

    // Flat OBI port counts per group (PORT_COUNT * NUM_REQ each)
    static constexpr int RD0_PORTS = 4 * NUM_REQ; // RAGU_A
    static constexpr int RD1_PORTS = 2 * NUM_REQ; // RAGU_B
    static constexpr int RD2_PORTS = NUM_REQ;     // RAGU_C
    static constexpr int RD3_PORTS = NUM_REQ;     // RAGU_D
    static constexpr int RD4_PORTS = NUM_REQ;     // RAGU_E

    static constexpr int WR0_PORTS = 4 * NUM_REQ; // WAGU_A
    static constexpr int WR1_PORTS = 2 * NUM_REQ; // WAGU_B
    static constexpr int WR2_PORTS = NUM_REQ;     // WAGU_D
    static constexpr int WR3_PORTS = NUM_REQ;     // WAGU_E

    // Rport base offsets
    static constexpr int RD1_BASE = RD0_PORTS;
    static constexpr int RD2_BASE = RD0_PORTS + RD1_PORTS;
    static constexpr int RD3_BASE = RD0_PORTS + RD1_PORTS + RD2_PORTS;
    static constexpr int RD4_BASE = RD0_PORTS + RD1_PORTS + RD2_PORTS + RD3_PORTS;

    // Wport base offsets
    static constexpr int WR1_BASE = WR0_PORTS;
    static constexpr int WR2_BASE = WR0_PORTS + WR1_PORTS;
    static constexpr int WR3_BASE = WR0_PORTS + WR1_PORTS + WR2_PORTS;

    using data_t = obi_data<BYTES_PER_ROW>;

    // -----------------------------------------------------------------------
    // External ports
    // -----------------------------------------------------------------------
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>     rport_req_i[NUM_RPORT_PORTS];
    sc_in<uint64_t> rport_addr_i[NUM_RPORT_PORTS];
    sc_in<bool>     rport_we_i[NUM_RPORT_PORTS];
    sc_in<uint32_t> rport_be_i[NUM_RPORT_PORTS];
    sc_in<data_t>   rport_wdata_i[NUM_RPORT_PORTS];
    sc_out<bool>    rport_gnt_o[NUM_RPORT_PORTS];
    sc_out<bool>    rport_rvalid_o[NUM_RPORT_PORTS];
    sc_out<data_t>  rport_rdata_o[NUM_RPORT_PORTS];

    sc_in<bool>     wport_req_i[NUM_WPORT_PORTS];
    sc_in<uint64_t> wport_addr_i[NUM_WPORT_PORTS];
    sc_in<bool>     wport_we_i[NUM_WPORT_PORTS];
    sc_in<uint32_t> wport_be_i[NUM_WPORT_PORTS];
    sc_in<data_t>   wport_wdata_i[NUM_WPORT_PORTS];
    sc_out<bool>    wport_gnt_o[NUM_WPORT_PORTS];
    sc_out<bool>    wport_rvalid_o[NUM_WPORT_PORTS];
    sc_out<data_t>  wport_rdata_o[NUM_WPORT_PORTS];

    // One NUM_BANK-wide lookahead bus per read buffer — each driven by that
    // buffer's own AGU (agu.hpp's lookahead_addr() accessor), wired straight
    // to fetch_addr_i[].
    sc_in<uint64_t> rd0_lookahead_i[NUM_BANK];
    sc_in<uint64_t> rd1_lookahead_i[NUM_BANK];
    sc_in<uint64_t> rd2_lookahead_i[NUM_BANK];
    sc_in<uint64_t> rd3_lookahead_i[NUM_BANK];
    sc_in<uint64_t> rd4_lookahead_i[NUM_BANK];

    // Per-buffer fetch_addr_valid_i (buf_r0..r3 only — buf_r4/DMA has no
    // lookahead source, see stim_bank_common.hpp). Gates fetch on the AGU's
    // own readiness (lookahead_ready()) so a fence-stalled task (e.g. a
    // write-then-read boundary) holds cells IDLE instead of latching stale
    // lookahead content, which would otherwise desync the lookahead cursor
    // from the AGU's own capture-side state for the rest of the run.
    sc_in<bool> rd0_lookahead_valid_i;
    sc_in<bool> rd1_lookahead_valid_i;
    sc_in<bool> rd2_lookahead_valid_i;
    sc_in<bool> rd3_lookahead_valid_i;

    // -----------------------------------------------------------------------
    // Internal: buffer TDM-side <-> mux  [buf * NUM_BANK + slot]
    //   Read buffers  : indices 0..NUM_RD_BUF-1
    //   Write buffers : indices NUM_RD_BUF..NUM_TOTAL_BUF-1
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> buf_tdm[NUM_BUF_TDM];

    // -----------------------------------------------------------------------
    // Internal: mux output -> tdm
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> mux_tdm[NUM_BANK];

    // -----------------------------------------------------------------------
    // Internal: tdm <-> crossbar
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> tdm_xbar[NUM_BANK];

    // -----------------------------------------------------------------------
    // Internal: crossbar <-> banks
    // -----------------------------------------------------------------------
    obi_signal_bundle<data_t> xbar_bank[NUM_BANK];

    sc_signal<int>      arb_req_sel, arb_rsp_sel;
    sc_signal<bool>     fetch_valid_const;
    sc_signal<uint64_t> zero_addr_const;

    // active_mode per buffer driven by the testbench (from stimuli ports_used_groups):
    //   buf_r0..r4 → indices 0..4,  buf_w0..w3 → indices 5..8
    sc_in<uint32_t> buf_active_mode_i[NUM_TOTAL_BUF];

    // Per-buffer TDM mapping parameters (muxed by arb_req_sel via map_cfg_comb)
    sc_in<uint64_t> buf_map_r_i[NUM_TOTAL_BUF];
    sc_in<uint64_t> buf_map_c_i[NUM_TOTAL_BUF];
    sc_in<uint64_t> buf_map_l_i[NUM_TOTAL_BUF];
    sc_in<uint64_t> buf_map_store_mode_i[NUM_TOTAL_BUF];

    sc_signal<uint64_t> map_num_banks_cfg, map_bank_width_cfg, map_r_cfg, map_c_cfg;
    sc_signal<uint64_t> map_l_cfg, map_store_mode_cfg;

    // -----------------------------------------------------------------------
    // Submodules
    // -----------------------------------------------------------------------
    // Read buffers
    buffer<NUM_REQ, 4, BYTES_PER_ROW, NUM_BANK, false> buf_r0; // RAGU_A
    buffer<NUM_REQ, 2, BYTES_PER_ROW, NUM_BANK, false> buf_r1; // RAGU_B
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, false> buf_r2; // RAGU_C
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, false> buf_r3; // RAGU_D
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, false> buf_r4; // RAGU_E

    // Write buffers
    buffer<NUM_REQ, 4, BYTES_PER_ROW, NUM_BANK, true> buf_w0; // WAGU_A
    buffer<NUM_REQ, 2, BYTES_PER_ROW, NUM_BANK, true> buf_w1; // WAGU_B
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, true> buf_w2; // WAGU_D
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, true> buf_w3; // WAGU_E

#if defined(IMPL_ARB_ADAPTIVE)
    arbiter_adaptive<NUM_TOTAL_BUF> arb;
    // One per buffer: OR-reduction of that buffer's own NUM_BANK buf_tdm[]
    // req wires, feeding arb.req_i[] so the adaptive arbiter can skip
    // buffers with nothing pending instead of blindly rotating through them
    // (see arbiter_adaptive.hpp and buf_req_any_comb() below).
    sc_signal<bool> buf_req_any[NUM_TOTAL_BUF];
#else
    arbiter<NUM_TOTAL_BUF> arb;

    // Program the free-running arbiter's rotation table (see the client-list
    // comment in the header block). Static configuration — call during
    // elaboration/reset, before releasing rst_ni.
    void set_arb_sequence(const int *seq, int len) {
        arb.set_sequence(seq, len);
    }
#endif
    tdm<NUM_BANK, NUM_BANK, BYTES_PER_ROW>      mapf;
    crossbar<NUM_BANK, NUM_BANK, BYTES_PER_ROW> xbar;
    sc_vector<bank<NUM_ROW, BYTES_PER_ROW>>     banks;
    obi_monitor<NUM_BANK, BYTES_PER_ROW>       *bank_mon = nullptr;

#if defined(IMPL_ARB_ADAPTIVE)
    void buf_req_any_comb() {
        for (int b = 0; b < NUM_TOTAL_BUF; ++b) {
            bool any = false;
            for (int w = 0; w < NUM_BANK; ++w)
                any = any || buf_tdm[b * NUM_BANK + w].req.read();
            buf_req_any[b].write(any);
        }
    }
#endif

    // -----------------------------------------------------------------------
    // Inline mux: route arb_req_sel buffer's TDM bus to tdm module;
    //             return gnt via arb_req_sel, rvalid/rdata via arb_rsp_sel.
    // -----------------------------------------------------------------------
    void mux_comb() {
        const int sr = arb_req_sel.read();
        const int sp = arb_rsp_sel.read();

        for (int w = 0; w < NUM_BANK; ++w) {
            mux_tdm[w].req.write(false);
            mux_tdm[w].addr.write(0);
            mux_tdm[w].we.write(false);
            mux_tdm[w].be.write(0);
            mux_tdm[w].wdata.write(data_t{});
        }
        for (int i = 0; i < NUM_BUF_TDM; ++i) {
            buf_tdm[i].gnt.write(false);
            buf_tdm[i].rvalid.write(false);
            buf_tdm[i].rdata.write(data_t{});
        }

        if (sr >= 0 && sr < NUM_TOTAL_BUF) {
            for (int w = 0; w < NUM_BANK; ++w) {
                const int i = sr * NUM_BANK + w;
                mux_tdm[w].req.write(buf_tdm[i].req.read());
                mux_tdm[w].addr.write(buf_tdm[i].addr.read());
                mux_tdm[w].we.write(buf_tdm[i].we.read());
                mux_tdm[w].be.write(buf_tdm[i].be.read());
                mux_tdm[w].wdata.write(buf_tdm[i].wdata.read());
                buf_tdm[i].gnt.write(mux_tdm[w].gnt.read());
            }
        }

        if (sp >= 0 && sp < NUM_TOTAL_BUF) {
            for (int w = 0; w < NUM_BANK; ++w) {
                const int i = sp * NUM_BANK + w;
                buf_tdm[i].rvalid.write(mux_tdm[w].rvalid.read());
                buf_tdm[i].rdata.write(mux_tdm[w].rdata.read());
            }
        }
    }

    void map_cfg_comb() {
        const int sel = arb_req_sel.read();
        if (sel >= 0 && sel < NUM_TOTAL_BUF) {
            map_r_cfg.write(buf_map_r_i[sel].read());
            map_c_cfg.write(buf_map_c_i[sel].read());
            map_l_cfg.write(buf_map_l_i[sel].read());
            map_store_mode_cfg.write(buf_map_store_mode_i[sel].read());
        }
    }

    void monitor_proc() {
        // SEL_NO_MONITOR skips this debug CSV entirely — useful for large
        // sweeps where only the caller's own printed/stats summary is
        // needed, since this per-cycle log dominates I/O time otherwise.
        if (std::getenv("SEL_NO_MONITOR")) {
            wait();
            while (true)
                wait();
        }
        const char       *ch    = std::getenv("RTL_LAB_HOME");
        const char       *proj  = std::getenv("SEL_PROJECT");
        const char       *od    = std::getenv("SEL_OUT_DIR");
        const std::string pname = proj ? proj : "tdm";
        const std::string pdir  = ch ? std::string(ch) + "/projects/" + pname : "projects/" + pname;
        // Falls back to sim/unit/output (not ".") when SEL_OUT_DIR is unset
        // (e.g. under `edaf unit`) so debug CSVs land under the already
        // gitignored projects/*/sim tree instead of the current working
        // directory, wherever that happens to be.
        const std::string out_dir = pdir + "/sim/" + (od ? std::string(od) : "unit") + "/output";
        std::filesystem::create_directories(out_dir);

        std::ofstream log(out_dir + "/tdm_state.csv");
        log << "cycle,arb_req,arb_rsp,tdm_slots_req,"
               "r0_rd_ptr,r0_valid,r0_drain,"
               "r1_rd_ptr,r1_valid,r1_drain,"
               "r2_rd_ptr,r2_valid,r2_drain,"
               "r3_rd_ptr,r3_valid,r3_drain,"
               "r4_rd_ptr,r4_valid,r4_drain"
            << std::endl;

        uint64_t cycle = 0;
        wait();
        while (true) {
            wait();
            if (!rst_ni.read()) {
                cycle = 0;
                continue;
            }
            ++cycle;

            const auto s0 = buf_r0.snapshot();
            const auto s1 = buf_r1.snapshot();
            const auto s2 = buf_r2.snapshot();
            const auto s3 = buf_r3.snapshot();
            const auto s4 = buf_r4.snapshot();

            int tdm_slots = 0;
            for (int w = 0; w < NUM_BANK; ++w)
                if (mux_tdm[w].req.read())
                    ++tdm_slots;

            const bool r0_drain = rport_gnt_o[0].read();
            const bool r1_drain = rport_gnt_o[RD1_BASE].read();
            const bool r2_drain = rport_gnt_o[RD2_BASE].read();
            const bool r3_drain = rport_gnt_o[RD3_BASE].read();
            const bool r4_drain = rport_gnt_o[RD4_BASE].read();

            log << cycle << "," << arb_req_sel.read() << "," << arb_rsp_sel.read() << ","
                << tdm_slots << "," << s0.rd_ptr << "," << s0.n_valid << "," << r0_drain << ","
                << s1.rd_ptr << "," << s1.n_valid << "," << r1_drain << "," << s2.rd_ptr << ","
                << s2.n_valid << "," << r2_drain << "," << s3.rd_ptr << "," << s3.n_valid << ","
                << r3_drain << "," << s4.rd_ptr << "," << s4.n_valid << "," << r4_drain
                << std::endl;
        }
    }

    ~top_tdm() {
        delete bank_mon;
    }

    SC_CTOR(top_tdm)
        : buf_r0("buf_r0"), buf_r1("buf_r1"), buf_r2("buf_r2"), buf_r3("buf_r3"), buf_r4("buf_r4"),
          buf_w0("buf_w0"), buf_w1("buf_w1"), buf_w2("buf_w2"), buf_w3("buf_w3"), arb("arb"),
          mapf("tdm"), xbar("xbar"), banks("bank") {

        banks.init(NUM_BANK);

        fetch_valid_const.write(true);
        zero_addr_const.write(0);

        map_num_banks_cfg.write(NUM_BANK);
        map_bank_width_cfg.write(BYTES_PER_ROW);

        SC_METHOD(map_cfg_comb);
        sensitive << arb_req_sel;
        for (int i = 0; i < NUM_TOTAL_BUF; ++i)
            sensitive << buf_map_r_i[i] << buf_map_c_i[i] << buf_map_l_i[i]
                      << buf_map_store_mode_i[i];

        SC_THREAD(monitor_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);

        arb.clk_i(clk_i);
        arb.rst_ni(rst_ni);
        arb.sel_req_o(arb_req_sel);
        arb.sel_rsp_o(arb_rsp_sel);
#if defined(IMPL_ARB_ADAPTIVE)
        for (int b = 0; b < NUM_TOTAL_BUF; ++b)
            arb.req_i[b](buf_req_any[b]);

        SC_METHOD(buf_req_any_comb);
        for (int i = 0; i < NUM_BUF_TDM; ++i)
            sensitive << buf_tdm[i].req;
#endif

        SC_METHOD(mux_comb);
        sensitive << arb_req_sel << arb_rsp_sel;
        for (int i = 0; i < NUM_BUF_TDM; ++i)
            sensitive << buf_tdm[i].req << buf_tdm[i].addr << buf_tdm[i].we << buf_tdm[i].be
                      << buf_tdm[i].wdata;
        for (int w = 0; w < NUM_BANK; ++w)
            sensitive << mux_tdm[w].gnt << mux_tdm[w].rvalid << mux_tdm[w].rdata;

        // ---- Helper: bind TDM-side signals for a buffer at slot buf_idx ----
        // (port-side OBI and fetch-address inputs are bound inline per buffer
        // below, since each group's signal arrays differ)
        auto bind_tdm = [&](auto &buf, int buf_idx) {
            for (int w = 0; w < NUM_BANK; ++w)
                bind_obi(buf.m[w], buf_tdm[buf_idx * NUM_BANK + w]);
        };

        // ---- buf_r0 : RAGU_A, PORT_COUNT=4 ------------------------------------
        buf_r0.clk_i(clk_i);
        buf_r0.rst_ni(rst_ni);
        buf_r0.active_mode(buf_active_mode_i[0]);
        buf_r0.fetch_addr_valid_i(rd0_lookahead_valid_i);
        for (int p = 0; p < RD0_PORTS; ++p) {
            buf_r0.p[p].req_i(rport_req_i[p]);
            buf_r0.p[p].addr_i(rport_addr_i[p]);
            buf_r0.p[p].we_i(rport_we_i[p]);
            buf_r0.p[p].be_i(rport_be_i[p]);
            buf_r0.p[p].wdata_i(rport_wdata_i[p]);
            buf_r0.p[p].gnt_o(rport_gnt_o[p]);
            buf_r0.p[p].rvalid_o(rport_rvalid_o[p]);
            buf_r0.p[p].rdata_o(rport_rdata_o[p]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_r0.fetch_addr_i[w](rd0_lookahead_i[w]);
        bind_tdm(buf_r0, 0);

        // ---- buf_r1 : RAGU_B, PORT_COUNT=2 ------------------------------------
        buf_r1.clk_i(clk_i);
        buf_r1.rst_ni(rst_ni);
        buf_r1.active_mode(buf_active_mode_i[1]);
        buf_r1.fetch_addr_valid_i(rd1_lookahead_valid_i);
        for (int p = 0; p < RD1_PORTS; ++p) {
            const int rp = RD1_BASE + p;
            buf_r1.p[p].req_i(rport_req_i[rp]);
            buf_r1.p[p].addr_i(rport_addr_i[rp]);
            buf_r1.p[p].we_i(rport_we_i[rp]);
            buf_r1.p[p].be_i(rport_be_i[rp]);
            buf_r1.p[p].wdata_i(rport_wdata_i[rp]);
            buf_r1.p[p].gnt_o(rport_gnt_o[rp]);
            buf_r1.p[p].rvalid_o(rport_rvalid_o[rp]);
            buf_r1.p[p].rdata_o(rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_r1.fetch_addr_i[w](rd1_lookahead_i[w]);
        bind_tdm(buf_r1, 1);

        // ---- buf_r2 : RAGU_C, PORT_COUNT=1 ------------------------------------
        buf_r2.clk_i(clk_i);
        buf_r2.rst_ni(rst_ni);
        buf_r2.active_mode(buf_active_mode_i[2]);
        buf_r2.fetch_addr_valid_i(rd2_lookahead_valid_i);
        for (int p = 0; p < RD2_PORTS; ++p) {
            const int rp = RD2_BASE + p;
            buf_r2.p[p].req_i(rport_req_i[rp]);
            buf_r2.p[p].addr_i(rport_addr_i[rp]);
            buf_r2.p[p].we_i(rport_we_i[rp]);
            buf_r2.p[p].be_i(rport_be_i[rp]);
            buf_r2.p[p].wdata_i(rport_wdata_i[rp]);
            buf_r2.p[p].gnt_o(rport_gnt_o[rp]);
            buf_r2.p[p].rvalid_o(rport_rvalid_o[rp]);
            buf_r2.p[p].rdata_o(rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_r2.fetch_addr_i[w](rd2_lookahead_i[w]);
        bind_tdm(buf_r2, 2);

        // ---- buf_r3 : RAGU_D, PORT_COUNT=1 ------------------------------------
        buf_r3.clk_i(clk_i);
        buf_r3.rst_ni(rst_ni);
        buf_r3.active_mode(buf_active_mode_i[3]);
        buf_r3.fetch_addr_valid_i(rd3_lookahead_valid_i);
        for (int p = 0; p < RD3_PORTS; ++p) {
            const int rp = RD3_BASE + p;
            buf_r3.p[p].req_i(rport_req_i[rp]);
            buf_r3.p[p].addr_i(rport_addr_i[rp]);
            buf_r3.p[p].we_i(rport_we_i[rp]);
            buf_r3.p[p].be_i(rport_be_i[rp]);
            buf_r3.p[p].wdata_i(rport_wdata_i[rp]);
            buf_r3.p[p].gnt_o(rport_gnt_o[rp]);
            buf_r3.p[p].rvalid_o(rport_rvalid_o[rp]);
            buf_r3.p[p].rdata_o(rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_r3.fetch_addr_i[w](rd3_lookahead_i[w]);
        bind_tdm(buf_r3, 3);

        // ---- buf_r4 : RAGU_E, PORT_COUNT=1 ----------------------------------
        // No dedicated valid gate: lane_agu<> (RAGU_E's driver) has no
        // lookahead source (see stim_bank_common.hpp's comment on why), so
        // this stays tied to the always-true constant and just drains
        // whatever its unconnected/zero lookahead bus feeds it (NOP).
        buf_r4.clk_i(clk_i);
        buf_r4.rst_ni(rst_ni);
        buf_r4.active_mode(buf_active_mode_i[4]);
        buf_r4.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD4_PORTS; ++p) {
            const int rp = RD4_BASE + p;
            buf_r4.p[p].req_i(rport_req_i[rp]);
            buf_r4.p[p].addr_i(rport_addr_i[rp]);
            buf_r4.p[p].we_i(rport_we_i[rp]);
            buf_r4.p[p].be_i(rport_be_i[rp]);
            buf_r4.p[p].wdata_i(rport_wdata_i[rp]);
            buf_r4.p[p].gnt_o(rport_gnt_o[rp]);
            buf_r4.p[p].rvalid_o(rport_rvalid_o[rp]);
            buf_r4.p[p].rdata_o(rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_r4.fetch_addr_i[w](rd4_lookahead_i[w]);
        bind_tdm(buf_r4, 4);

        // ---- buf_w0 : WAGU_A, PORT_COUNT=4, IS_WRITE=true ---------------------
        buf_w0.clk_i(clk_i);
        buf_w0.rst_ni(rst_ni);
        buf_w0.active_mode(buf_active_mode_i[5]);
        buf_w0.fetch_addr_valid_i(fetch_valid_const); // unused in write mode
        for (int p = 0; p < WR0_PORTS; ++p) {
            buf_w0.p[p].req_i(wport_req_i[p]);
            buf_w0.p[p].addr_i(wport_addr_i[p]);
            buf_w0.p[p].we_i(wport_we_i[p]);
            buf_w0.p[p].be_i(wport_be_i[p]);
            buf_w0.p[p].wdata_i(wport_wdata_i[p]);
            buf_w0.p[p].gnt_o(wport_gnt_o[p]);
            buf_w0.p[p].rvalid_o(wport_rvalid_o[p]);
            buf_w0.p[p].rdata_o(wport_rdata_o[p]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w0.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w0, NUM_RD_BUF + 0);

        // ---- buf_w1 : WAGU_B, PORT_COUNT=2, IS_WRITE=true ---------------------
        buf_w1.clk_i(clk_i);
        buf_w1.rst_ni(rst_ni);
        buf_w1.active_mode(buf_active_mode_i[6]);
        buf_w1.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < WR1_PORTS; ++p) {
            const int wp = WR1_BASE + p;
            buf_w1.p[p].req_i(wport_req_i[wp]);
            buf_w1.p[p].addr_i(wport_addr_i[wp]);
            buf_w1.p[p].we_i(wport_we_i[wp]);
            buf_w1.p[p].be_i(wport_be_i[wp]);
            buf_w1.p[p].wdata_i(wport_wdata_i[wp]);
            buf_w1.p[p].gnt_o(wport_gnt_o[wp]);
            buf_w1.p[p].rvalid_o(wport_rvalid_o[wp]);
            buf_w1.p[p].rdata_o(wport_rdata_o[wp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w1.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w1, NUM_RD_BUF + 1);

        // ---- buf_w2 : WAGU_D, PORT_COUNT=1, IS_WRITE=true ---------------------
        buf_w2.clk_i(clk_i);
        buf_w2.rst_ni(rst_ni);
        buf_w2.active_mode(buf_active_mode_i[7]);
        buf_w2.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < WR2_PORTS; ++p) {
            const int wp = WR2_BASE + p;
            buf_w2.p[p].req_i(wport_req_i[wp]);
            buf_w2.p[p].addr_i(wport_addr_i[wp]);
            buf_w2.p[p].we_i(wport_we_i[wp]);
            buf_w2.p[p].be_i(wport_be_i[wp]);
            buf_w2.p[p].wdata_i(wport_wdata_i[wp]);
            buf_w2.p[p].gnt_o(wport_gnt_o[wp]);
            buf_w2.p[p].rvalid_o(wport_rvalid_o[wp]);
            buf_w2.p[p].rdata_o(wport_rdata_o[wp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w2.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w2, NUM_RD_BUF + 2);

        // ---- buf_w3 : WAGU_E, PORT_COUNT=1, IS_WRITE=true -------------------
        buf_w3.clk_i(clk_i);
        buf_w3.rst_ni(rst_ni);
        buf_w3.active_mode(buf_active_mode_i[8]);
        buf_w3.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < WR3_PORTS; ++p) {
            const int wp = WR3_BASE + p;
            buf_w3.p[p].req_i(wport_req_i[wp]);
            buf_w3.p[p].addr_i(wport_addr_i[wp]);
            buf_w3.p[p].we_i(wport_we_i[wp]);
            buf_w3.p[p].be_i(wport_be_i[wp]);
            buf_w3.p[p].wdata_i(wport_wdata_i[wp]);
            buf_w3.p[p].gnt_o(wport_gnt_o[wp]);
            buf_w3.p[p].rvalid_o(wport_rvalid_o[wp]);
            buf_w3.p[p].rdata_o(wport_rdata_o[wp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w3.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w3, NUM_RD_BUF + 3);

        // ---- TDM ---------------------------------------------------------------
        mapf.num_banks_i(map_num_banks_cfg);
        mapf.bank_width_i(map_bank_width_cfg);
        mapf.r_i(map_r_cfg);
        mapf.c_i(map_c_cfg);
        mapf.l_i(map_l_cfg);
        mapf.store_mode_i(map_store_mode_cfg);
        for (int w = 0; w < NUM_BANK; ++w) {
            bind_obi(mapf.g[w], mux_tdm[w]);
            bind_obi(mapf.c[w], tdm_xbar[w]);
        }

        // ---- Crossbar ----------------------------------------------------------
        xbar.clk_i(clk_i);
        xbar.rst_ni(rst_ni);
        for (int m = 0; m < NUM_BANK; ++m)
            bind_obi(xbar.m_ports[m], tdm_xbar[m]);
        for (int b = 0; b < NUM_BANK; ++b) {
            bind_obi(xbar.b_ports[b], xbar_bank[b]);

            banks[b].clk_i(clk_i);
            banks[b].rst_ni(rst_ni);
            bind_obi(banks[b].obi, xbar_bank[b]);
        }

        // ---- Bank-side OBI monitor ---------------------------------------------
        // SEL_NO_MONITOR skips this CSV too (obi_monitor no-ops on an empty
        // path) — see monitor_proc()'s matching comment.
        if (!std::getenv("SEL_NO_MONITOR")) {
            const char       *ch    = std::getenv("RTL_LAB_HOME");
            const char       *proj  = std::getenv("SEL_PROJECT");
            const char       *od    = std::getenv("SEL_OUT_DIR");
            const std::string pname = proj ? proj : "tdm";
            const std::string pdir =
                ch ? std::string(ch) + "/projects/" + pname : "projects/" + pname;
            // See monitor_proc()'s matching comment: falls back to
            // sim/unit/output, never ".", so this never writes into whatever
            // directory the binary happens to be run from.
            const std::string out_dir =
                pdir + "/sim/" + (od ? std::string(od) : "unit") + "/output";
            std::filesystem::create_directories(out_dir);

            bank_mon = new obi_monitor<NUM_BANK, BYTES_PER_ROW>("bank_mon", "xbar_bank",
                                                                out_dir + "/bank_obi.csv");
            bank_mon->clk_i(clk_i);
            bank_mon->rst_ni(rst_ni);
            for (int b = 0; b < NUM_BANK; ++b)
                bind_obi(bank_mon->obi[b], xbar_bank[b]);
        }
    }
};

#endif
