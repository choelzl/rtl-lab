// -----------------------------------------------------------------------------
// Author: Cedric Hölzl
//
// Description:
//   Native SystemC design top (DUT) for the TDM read/write path.
//
//   Pipeline:
//     RPORT groups --> buf_r{0..4} -+
//     WPORT groups --> buf_w{0..3} -+--> mux (arbiter sel) --> tdm --> crossbar --> bank[NUM_BANK]
//
//   Read buffers (IS_WRITE=false):
//     buf_r0 : PORT_COUNT=4, NUM_TDM=NUM_BANK  (RAGU_A)
//     buf_r1 : PORT_COUNT=2, NUM_TDM=NUM_BANK  (RAGU_B)
//     buf_r2 : PORT_COUNT=1, NUM_TDM=NUM_BANK  (RAGU_C)
//     buf_r3 : PORT_COUNT=1, NUM_TDM=NUM_BANK  (RAGU_D)
//     buf_r4 : PORT_COUNT=1, NUM_TDM=NUM_BANK  (RAGU_DMA)
//
//   Write buffers (IS_WRITE=true):
//     buf_w0 : PORT_COUNT=4, NUM_TDM=NUM_BANK  (WAGU_A)
//     buf_w1 : PORT_COUNT=2, NUM_TDM=NUM_BANK  (WAGU_B)
//     buf_w2 : PORT_COUNT=1, NUM_TDM=NUM_BANK  (WAGU_D)
//     buf_w3 : PORT_COUNT=1, NUM_TDM=NUM_BANK  (WAGU_DMA)
//
//   active_mode encoding per buffer (driven from buf_active_mode_i[0..NUM_TOTAL_BUF-1]):
//     0 → 1 active port group  (active_beats = NUM_REQ)
//     1 → 2 active port groups (active_beats = 2*NUM_REQ)
//     2 → 4 active port groups (active_beats = 4*NUM_REQ, clamped to PORT_COUNT)
//   Set from stimuli ports_used_groups field; indices: buf_r0..r4=0..4, buf_w0..w3=5..8.
//
//   Each read buffer's fetch_addr_i[0..N-1] is wired from the corresponding
//   rport_addr_i group (N = PORT_COUNT * NUM_REQ).  Slots [N..NUM_BANK-1] are
//   tied to 0 as a placeholder; a lookahead queue module feeding future groups
//   is a TODO.  Write buffers ignore fetch_addr_i entirely.
//
//   The arbiter round-robins across all 9 buffers.  mux_comb routes the
//   selected buffer's full NUM_BANK-wide OBI (including wdata) to/from the TDM
//   module and steers gnt/rvalid/rdata back to the correct buffer.
//
// Template parameters:
//   NUM_RPORT       -- read port groups; must be >= 9 (4+2+1+1+1 split)
//   NUM_WPORT       -- write port groups; must be >= 8 (4+2+1+1 split)
//   NUM_REQ         -- OBI buses per port (default 4)
//   NUM_BANK        -- TDM bus width / buffer window size (default 32)
//   NUM_ROW         -- rows per bank (default 1024)
//   BYTES_PER_WORD  -- bytes per word (default 4)
//   WORDS_PER_ROW   -- words per bank row (default 4)
// -----------------------------------------------------------------------------

#ifndef TOP_TDM_HPP
#define TOP_TDM_HPP

#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <string>
#include <systemc.h>

#include "arbiter.hpp"
#include "bank.hpp"
#include "buffer.hpp"
#include "crossbar.hpp"
#include "obi_data.hpp"
#include "obi_monitor.hpp"
#include "tdm.hpp"

template <int NUM_RPORT = 9, int NUM_WPORT = 8, int NUM_REQ = 4, int NUM_BANK = 32,
          int NUM_ROW = 1024, int BYTES_PER_WORD = 4, int WORDS_PER_ROW = 4>
SC_MODULE(top_tdm) {
    static_assert(NUM_RPORT >= 9, "top_tdm maps 9 read port groups (4+2+1+1+1)");
    static_assert(NUM_WPORT >= 8, "top_tdm maps 8 write port groups (4+2+1+1)");

    static constexpr int NUM_RD_BUF      = 5;
    static constexpr int NUM_WR_BUF      = 4;
    static constexpr int NUM_TOTAL_BUF   = NUM_RD_BUF + NUM_WR_BUF;
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
    static constexpr int RD4_PORTS = NUM_REQ;     // RAGU_DMA

    static constexpr int WR0_PORTS = 4 * NUM_REQ; // WAGU_A
    static constexpr int WR1_PORTS = 2 * NUM_REQ; // WAGU_B
    static constexpr int WR2_PORTS = NUM_REQ;     // WAGU_D
    static constexpr int WR3_PORTS = NUM_REQ;     // WAGU_DMA

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

    // -----------------------------------------------------------------------
    // Internal: buffer TDM-side <-> mux  [buf * NUM_BANK + slot]
    //   Read buffers  : indices 0..NUM_RD_BUF-1
    //   Write buffers : indices NUM_RD_BUF..NUM_TOTAL_BUF-1
    // -----------------------------------------------------------------------
    sc_signal<bool>     buf_tdm_req[NUM_BUF_TDM];
    sc_signal<uint64_t> buf_tdm_addr[NUM_BUF_TDM];
    sc_signal<bool>     buf_tdm_we[NUM_BUF_TDM];
    sc_signal<uint32_t> buf_tdm_be[NUM_BUF_TDM];
    sc_signal<data_t>   buf_tdm_wdata[NUM_BUF_TDM];
    sc_signal<bool>     buf_tdm_gnt[NUM_BUF_TDM];
    sc_signal<bool>     buf_tdm_rvalid[NUM_BUF_TDM];
    sc_signal<data_t>   buf_tdm_rdata[NUM_BUF_TDM];

    // -----------------------------------------------------------------------
    // Internal: mux output -> tdm
    // -----------------------------------------------------------------------
    sc_signal<bool>     mux_tdm_req[NUM_BANK];
    sc_signal<uint64_t> mux_tdm_addr[NUM_BANK];
    sc_signal<bool>     mux_tdm_we[NUM_BANK];
    sc_signal<uint32_t> mux_tdm_be[NUM_BANK];
    sc_signal<data_t>   mux_tdm_wdata[NUM_BANK];
    sc_signal<bool>     mux_tdm_gnt[NUM_BANK];
    sc_signal<bool>     mux_tdm_rvalid[NUM_BANK];
    sc_signal<data_t>   mux_tdm_rdata[NUM_BANK];

    // -----------------------------------------------------------------------
    // Internal: tdm <-> crossbar
    // -----------------------------------------------------------------------
    sc_signal<bool>     tdm_xbar_req[NUM_BANK];
    sc_signal<uint64_t> tdm_xbar_addr[NUM_BANK];
    sc_signal<bool>     tdm_xbar_we[NUM_BANK];
    sc_signal<uint32_t> tdm_xbar_be[NUM_BANK];
    sc_signal<data_t>   tdm_xbar_wdata[NUM_BANK];
    sc_signal<bool>     tdm_xbar_gnt[NUM_BANK];
    sc_signal<bool>     tdm_xbar_rvalid[NUM_BANK];
    sc_signal<data_t>   tdm_xbar_rdata[NUM_BANK];

    // -----------------------------------------------------------------------
    // Internal: crossbar <-> banks
    // -----------------------------------------------------------------------
    sc_signal<bool>     xbar_bank_req[NUM_BANK];
    sc_signal<uint64_t> xbar_bank_addr[NUM_BANK];
    sc_signal<bool>     xbar_bank_we[NUM_BANK];
    sc_signal<uint32_t> xbar_bank_be[NUM_BANK];
    sc_signal<data_t>   xbar_bank_wdata[NUM_BANK];
    sc_signal<bool>     xbar_bank_gnt[NUM_BANK];
    sc_signal<bool>     xbar_bank_rvalid[NUM_BANK];
    sc_signal<data_t>   xbar_bank_rdata[NUM_BANK];

    sc_signal<int>      arb_req_sel, arb_rsp_sel;
    sc_signal<bool>     fetch_valid_const;
    sc_signal<uint64_t> zero_addr_const;

    // active_mode per buffer driven by the testbench (from stimuli ports_used_groups):
    //   buf_r0..r4 → indices 0..4,  buf_w0..w3 → indices 5..8
    sc_in<uint32_t> buf_active_mode_i[NUM_TOTAL_BUF];

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
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, false> buf_r4; // RAGU_DMA

    // Write buffers
    buffer<NUM_REQ, 4, BYTES_PER_ROW, NUM_BANK, true> buf_w0; // WAGU_A
    buffer<NUM_REQ, 2, BYTES_PER_ROW, NUM_BANK, true> buf_w1; // WAGU_B
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, true> buf_w2; // WAGU_D
    buffer<NUM_REQ, 1, BYTES_PER_ROW, NUM_BANK, true> buf_w3; // WAGU_DMA

    arbiter<NUM_TOTAL_BUF>                      arb;
    tdm<NUM_BANK, NUM_BANK, BYTES_PER_ROW>      mapf;
    crossbar<NUM_BANK, NUM_BANK, BYTES_PER_ROW> xbar;
    sc_vector<bank<NUM_ROW, BYTES_PER_ROW>>     banks;
    obi_monitor<NUM_BANK, BYTES_PER_ROW>       *bank_mon = nullptr;

    // -----------------------------------------------------------------------
    // Inline mux: route arb_req_sel buffer's TDM bus to tdm module;
    //             return gnt via arb_req_sel, rvalid/rdata via arb_rsp_sel.
    // -----------------------------------------------------------------------
    void mux_comb() {
        const int sr = arb_req_sel.read();
        const int sp = arb_rsp_sel.read();

        for (int w = 0; w < NUM_BANK; ++w) {
            mux_tdm_req[w].write(false);
            mux_tdm_addr[w].write(0);
            mux_tdm_we[w].write(false);
            mux_tdm_be[w].write(0);
            mux_tdm_wdata[w].write(data_t{});
        }
        for (int i = 0; i < NUM_BUF_TDM; ++i) {
            buf_tdm_gnt[i].write(false);
            buf_tdm_rvalid[i].write(false);
            buf_tdm_rdata[i].write(data_t{});
        }

        if (sr >= 0 && sr < NUM_TOTAL_BUF) {
            for (int w = 0; w < NUM_BANK; ++w) {
                const int i = sr * NUM_BANK + w;
                mux_tdm_req[w].write(buf_tdm_req[i].read());
                mux_tdm_addr[w].write(buf_tdm_addr[i].read());
                mux_tdm_we[w].write(buf_tdm_we[i].read());
                mux_tdm_be[w].write(buf_tdm_be[i].read());
                mux_tdm_wdata[w].write(buf_tdm_wdata[i].read());
                buf_tdm_gnt[i].write(mux_tdm_gnt[w].read());
            }
        }

        if (sp >= 0 && sp < NUM_TOTAL_BUF) {
            for (int w = 0; w < NUM_BANK; ++w) {
                const int i = sp * NUM_BANK + w;
                buf_tdm_rvalid[i].write(mux_tdm_rvalid[w].read());
                buf_tdm_rdata[i].write(mux_tdm_rdata[w].read());
            }
        }
    }

    void monitor_proc() {
        const char       *ch    = std::getenv("RTL_LAB_HOME");
        const char       *proj  = std::getenv("SEL_PROJECT");
        const char       *od    = std::getenv("SEL_OUT_DIR");
        const std::string pname = proj ? proj : "tdm";
        const std::string pdir  = ch ? std::string(ch) + "/projects/" + pname : "projects/" + pname;
        const std::string out_dir = od ? pdir + "/sim/" + od + "/output" : ".";

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
                if (mux_tdm_req[w].read())
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

    ~top_tdm() { delete bank_mon; }

    SC_CTOR(top_tdm)
        : buf_r0("buf_r0"), buf_r1("buf_r1"), buf_r2("buf_r2"), buf_r3("buf_r3"), buf_r4("buf_r4"),
          buf_w0("buf_w0"), buf_w1("buf_w1"), buf_w2("buf_w2"), buf_w3("buf_w3"), arb("arb"),
          mapf("tdm"), xbar("xbar"), banks("bank") {

        banks.init(NUM_BANK);

        fetch_valid_const.write(true);
        zero_addr_const.write(0);

        map_num_banks_cfg.write(NUM_BANK);
        map_bank_width_cfg.write(BYTES_PER_ROW);
        map_r_cfg.write(4);
        map_c_cfg.write(4);
        map_l_cfg.write(8);
        map_store_mode_cfg.write(0);

        SC_THREAD(monitor_proc);
        sensitive << clk_i.pos();
        async_reset_signal_is(rst_ni, false);

        arb.clk_i(clk_i);
        arb.rst_ni(rst_ni);
        arb.sel_req_o(arb_req_sel);
        arb.sel_rsp_o(arb_rsp_sel);

        SC_METHOD(mux_comb);
        sensitive << arb_req_sel << arb_rsp_sel;
        for (int i = 0; i < NUM_BUF_TDM; ++i)
            sensitive << buf_tdm_req[i] << buf_tdm_addr[i] << buf_tdm_we[i] << buf_tdm_be[i]
                      << buf_tdm_wdata[i];
        for (int w = 0; w < NUM_BANK; ++w)
            sensitive << mux_tdm_gnt[w] << mux_tdm_rvalid[w] << mux_tdm_rdata[w];

        // -----------------------------------------------------------------------
        // Helper lambda: bind a buffer's port-side OBI, TDM-side OBI,
        // and fetch-address inputs.
        //
        //   buf_idx    : arbiter / buf_tdm slice index
        //   port_base  : first index into rport_*/wport_* signal arrays
        //   num_ports  : PORT_COUNT * NUM_REQ (number of OBI buses for this group)
        //   r_req / r_addr / r_be / r_wdata / r_gnt / r_rvalid / r_rdata :
        //                flat signal arrays for the port-facing OBI
        //   fetch_base : first rport_addr_i index used as fetch address (read mode)
        //                for read buffers; ignored (tied zero) for write buffers
        // -----------------------------------------------------------------------

        // ---- Helper: bind TDM-side signals for a buffer at slot buf_idx ----
        auto bind_tdm = [&](auto &buf, int buf_idx) {
            for (int w = 0; w < NUM_BANK; ++w) {
                const int i = buf_idx * NUM_BANK + w;
                buf.m_req_o[w](buf_tdm_req[i]);
                buf.m_addr_o[w](buf_tdm_addr[i]);
                buf.m_we_o[w](buf_tdm_we[i]);
                buf.m_be_o[w](buf_tdm_be[i]);
                buf.m_wdata_o[w](buf_tdm_wdata[i]);
                buf.m_gnt_i[w](buf_tdm_gnt[i]);
                buf.m_rvalid_i[w](buf_tdm_rvalid[i]);
                buf.m_rdata_i[w](buf_tdm_rdata[i]);
            }
        };

        // ---- buf_r0 : RAGU_A, PORT_COUNT=4 ------------------------------------
        buf_r0.clk_i(clk_i);
        buf_r0.rst_ni(rst_ni);
        buf_r0.active_mode(buf_active_mode_i[0]);
        buf_r0.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD0_PORTS; ++p) {
            buf_r0.p_req_i[p](rport_req_i[p]);
            buf_r0.p_addr_i[p](rport_addr_i[p]);
            buf_r0.p_be_i[p](rport_be_i[p]);
            buf_r0.p_wdata_i[p](rport_wdata_i[p]);
            buf_r0.p_gnt_o[p](rport_gnt_o[p]);
            buf_r0.p_rvalid_o[p](rport_rvalid_o[p]);
            buf_r0.p_rdata_o[p](rport_rdata_o[p]);
        }
        // fetch_addr_i: first RD0_PORTS slots from port addresses; rest tied zero
        for (int w = 0; w < NUM_BANK; ++w) {
            if (w < RD0_PORTS) buf_r0.fetch_addr_i[w](rport_addr_i[w]);
            else                buf_r0.fetch_addr_i[w](zero_addr_const);
        }
        bind_tdm(buf_r0, 0);

        // ---- buf_r1 : RAGU_B, PORT_COUNT=2 ------------------------------------
        buf_r1.clk_i(clk_i);
        buf_r1.rst_ni(rst_ni);
        buf_r1.active_mode(buf_active_mode_i[1]);
        buf_r1.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD1_PORTS; ++p) {
            const int rp = RD1_BASE + p;
            buf_r1.p_req_i[p](rport_req_i[rp]);
            buf_r1.p_addr_i[p](rport_addr_i[rp]);
            buf_r1.p_be_i[p](rport_be_i[rp]);
            buf_r1.p_wdata_i[p](rport_wdata_i[rp]);
            buf_r1.p_gnt_o[p](rport_gnt_o[rp]);
            buf_r1.p_rvalid_o[p](rport_rvalid_o[rp]);
            buf_r1.p_rdata_o[p](rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w) {
            if (w < RD1_PORTS) buf_r1.fetch_addr_i[w](rport_addr_i[RD1_BASE + w]);
            else                buf_r1.fetch_addr_i[w](zero_addr_const);
        }
        bind_tdm(buf_r1, 1);

        // ---- buf_r2 : RAGU_C, PORT_COUNT=1 ------------------------------------
        buf_r2.clk_i(clk_i);
        buf_r2.rst_ni(rst_ni);
        buf_r2.active_mode(buf_active_mode_i[2]);
        buf_r2.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD2_PORTS; ++p) {
            const int rp = RD2_BASE + p;
            buf_r2.p_req_i[p](rport_req_i[rp]);
            buf_r2.p_addr_i[p](rport_addr_i[rp]);
            buf_r2.p_be_i[p](rport_be_i[rp]);
            buf_r2.p_wdata_i[p](rport_wdata_i[rp]);
            buf_r2.p_gnt_o[p](rport_gnt_o[rp]);
            buf_r2.p_rvalid_o[p](rport_rvalid_o[rp]);
            buf_r2.p_rdata_o[p](rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w) {
            if (w < RD2_PORTS) buf_r2.fetch_addr_i[w](rport_addr_i[RD2_BASE + w]);
            else                buf_r2.fetch_addr_i[w](zero_addr_const);
        }
        bind_tdm(buf_r2, 2);

        // ---- buf_r3 : RAGU_D, PORT_COUNT=1 ------------------------------------
        buf_r3.clk_i(clk_i);
        buf_r3.rst_ni(rst_ni);
        buf_r3.active_mode(buf_active_mode_i[3]);
        buf_r3.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD3_PORTS; ++p) {
            const int rp = RD3_BASE + p;
            buf_r3.p_req_i[p](rport_req_i[rp]);
            buf_r3.p_addr_i[p](rport_addr_i[rp]);
            buf_r3.p_be_i[p](rport_be_i[rp]);
            buf_r3.p_wdata_i[p](rport_wdata_i[rp]);
            buf_r3.p_gnt_o[p](rport_gnt_o[rp]);
            buf_r3.p_rvalid_o[p](rport_rvalid_o[rp]);
            buf_r3.p_rdata_o[p](rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w) {
            if (w < RD3_PORTS) buf_r3.fetch_addr_i[w](rport_addr_i[RD3_BASE + w]);
            else                buf_r3.fetch_addr_i[w](zero_addr_const);
        }
        bind_tdm(buf_r3, 3);

        // ---- buf_r4 : RAGU_DMA, PORT_COUNT=1 ----------------------------------
        buf_r4.clk_i(clk_i);
        buf_r4.rst_ni(rst_ni);
        buf_r4.active_mode(buf_active_mode_i[4]);
        buf_r4.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < RD4_PORTS; ++p) {
            const int rp = RD4_BASE + p;
            buf_r4.p_req_i[p](rport_req_i[rp]);
            buf_r4.p_addr_i[p](rport_addr_i[rp]);
            buf_r4.p_be_i[p](rport_be_i[rp]);
            buf_r4.p_wdata_i[p](rport_wdata_i[rp]);
            buf_r4.p_gnt_o[p](rport_gnt_o[rp]);
            buf_r4.p_rvalid_o[p](rport_rvalid_o[rp]);
            buf_r4.p_rdata_o[p](rport_rdata_o[rp]);
        }
        for (int w = 0; w < NUM_BANK; ++w) {
            if (w < RD4_PORTS) buf_r4.fetch_addr_i[w](rport_addr_i[RD4_BASE + w]);
            else                buf_r4.fetch_addr_i[w](zero_addr_const);
        }
        bind_tdm(buf_r4, 4);

        // ---- buf_w0 : WAGU_A, PORT_COUNT=4, IS_WRITE=true ---------------------
        buf_w0.clk_i(clk_i);
        buf_w0.rst_ni(rst_ni);
        buf_w0.active_mode(buf_active_mode_i[5]);
        buf_w0.fetch_addr_valid_i(fetch_valid_const); // unused in write mode
        for (int p = 0; p < WR0_PORTS; ++p) {
            buf_w0.p_req_i[p](wport_req_i[p]);
            buf_w0.p_addr_i[p](wport_addr_i[p]);
            buf_w0.p_be_i[p](wport_be_i[p]);
            buf_w0.p_wdata_i[p](wport_wdata_i[p]);
            buf_w0.p_gnt_o[p](wport_gnt_o[p]);
            buf_w0.p_rvalid_o[p](wport_rvalid_o[p]);
            buf_w0.p_rdata_o[p](wport_rdata_o[p]);
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
            buf_w1.p_req_i[p](wport_req_i[wp]);
            buf_w1.p_addr_i[p](wport_addr_i[wp]);
            buf_w1.p_be_i[p](wport_be_i[wp]);
            buf_w1.p_wdata_i[p](wport_wdata_i[wp]);
            buf_w1.p_gnt_o[p](wport_gnt_o[wp]);
            buf_w1.p_rvalid_o[p](wport_rvalid_o[wp]);
            buf_w1.p_rdata_o[p](wport_rdata_o[wp]);
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
            buf_w2.p_req_i[p](wport_req_i[wp]);
            buf_w2.p_addr_i[p](wport_addr_i[wp]);
            buf_w2.p_be_i[p](wport_be_i[wp]);
            buf_w2.p_wdata_i[p](wport_wdata_i[wp]);
            buf_w2.p_gnt_o[p](wport_gnt_o[wp]);
            buf_w2.p_rvalid_o[p](wport_rvalid_o[wp]);
            buf_w2.p_rdata_o[p](wport_rdata_o[wp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w2.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w2, NUM_RD_BUF + 2);

        // ---- buf_w3 : WAGU_DMA, PORT_COUNT=1, IS_WRITE=true -------------------
        buf_w3.clk_i(clk_i);
        buf_w3.rst_ni(rst_ni);
        buf_w3.active_mode(buf_active_mode_i[8]);
        buf_w3.fetch_addr_valid_i(fetch_valid_const);
        for (int p = 0; p < WR3_PORTS; ++p) {
            const int wp = WR3_BASE + p;
            buf_w3.p_req_i[p](wport_req_i[wp]);
            buf_w3.p_addr_i[p](wport_addr_i[wp]);
            buf_w3.p_be_i[p](wport_be_i[wp]);
            buf_w3.p_wdata_i[p](wport_wdata_i[wp]);
            buf_w3.p_gnt_o[p](wport_gnt_o[wp]);
            buf_w3.p_rvalid_o[p](wport_rvalid_o[wp]);
            buf_w3.p_rdata_o[p](wport_rdata_o[wp]);
        }
        for (int w = 0; w < NUM_BANK; ++w)
            buf_w3.fetch_addr_i[w](zero_addr_const);
        bind_tdm(buf_w3, NUM_RD_BUF + 3);

        // ---- TDM ---------------------------------------------------------------
        // TODO: tdm.hpp needs per-slot g_addr_i[NUM_BANK]; mux_tdm_addr[0] is a
        //       placeholder until that update lands.
        mapf.g_addr_i(mux_tdm_addr[0]);
        mapf.g_we_i(mux_tdm_we[0]);
        mapf.g_be_i(mux_tdm_be[0]);
        mapf.num_banks_i(map_num_banks_cfg);
        mapf.bank_width_i(map_bank_width_cfg);
        mapf.r_i(map_r_cfg);
        mapf.c_i(map_c_cfg);
        mapf.l_i(map_l_cfg);
        mapf.store_mode_i(map_store_mode_cfg);
        for (int w = 0; w < NUM_BANK; ++w) {
            mapf.g_req_i[w](mux_tdm_req[w]);
            mapf.g_wdata_i[w](mux_tdm_wdata[w]);
            mapf.g_gnt_o[w](mux_tdm_gnt[w]);
            mapf.g_rvalid_o[w](mux_tdm_rvalid[w]);
            mapf.g_rdata_o[w](mux_tdm_rdata[w]);
            mapf.c_req_o[w](tdm_xbar_req[w]);
            mapf.c_addr_o[w](tdm_xbar_addr[w]);
            mapf.c_we_o[w](tdm_xbar_we[w]);
            mapf.c_be_o[w](tdm_xbar_be[w]);
            mapf.c_wdata_o[w](tdm_xbar_wdata[w]);
            mapf.c_gnt_i[w](tdm_xbar_gnt[w]);
            mapf.c_rvalid_i[w](tdm_xbar_rvalid[w]);
            mapf.c_rdata_i[w](tdm_xbar_rdata[w]);
        }

        // ---- Crossbar ----------------------------------------------------------
        xbar.clk_i(clk_i);
        xbar.rst_ni(rst_ni);
        for (int m = 0; m < NUM_BANK; ++m) {
            xbar.m_req_i[m](tdm_xbar_req[m]);
            xbar.m_addr_i[m](tdm_xbar_addr[m]);
            xbar.m_we_i[m](tdm_xbar_we[m]);
            xbar.m_be_i[m](tdm_xbar_be[m]);
            xbar.m_wdata_i[m](tdm_xbar_wdata[m]);
            xbar.m_gnt_o[m](tdm_xbar_gnt[m]);
            xbar.m_rvalid_o[m](tdm_xbar_rvalid[m]);
            xbar.m_rdata_o[m](tdm_xbar_rdata[m]);
        }
        for (int b = 0; b < NUM_BANK; ++b) {
            xbar.b_req_o[b](xbar_bank_req[b]);
            xbar.b_addr_o[b](xbar_bank_addr[b]);
            xbar.b_we_o[b](xbar_bank_we[b]);
            xbar.b_be_o[b](xbar_bank_be[b]);
            xbar.b_wdata_o[b](xbar_bank_wdata[b]);
            xbar.b_gnt_i[b](xbar_bank_gnt[b]);
            xbar.b_rvalid_i[b](xbar_bank_rvalid[b]);
            xbar.b_rdata_i[b](xbar_bank_rdata[b]);

            banks[b].clk_i(clk_i);
            banks[b].rst_ni(rst_ni);
            banks[b].req_i(xbar_bank_req[b]);
            banks[b].addr_i(xbar_bank_addr[b]);
            banks[b].we_i(xbar_bank_we[b]);
            banks[b].be_i(xbar_bank_be[b]);
            banks[b].wdata_i(xbar_bank_wdata[b]);
            banks[b].gnt_o(xbar_bank_gnt[b]);
            banks[b].rvalid_o(xbar_bank_rvalid[b]);
            banks[b].rdata_o(xbar_bank_rdata[b]);
        }

        // ---- Bank-side OBI monitor ---------------------------------------------
        {
            const char       *ch      = std::getenv("RTL_LAB_HOME");
            const char       *proj    = std::getenv("SEL_PROJECT");
            const char       *od      = std::getenv("SEL_OUT_DIR");
            const std::string pname   = proj ? proj : "tdm";
            const std::string pdir    = ch ? std::string(ch) + "/projects/" + pname
                                           : "projects/" + pname;
            const std::string out_dir = od ? pdir + "/sim/" + od + "/output" : ".";

            bank_mon = new obi_monitor<NUM_BANK, BYTES_PER_ROW>(
                "bank_mon", "xbar_bank", out_dir + "/bank_obi.csv");
            bank_mon->clk_i(clk_i);
            bank_mon->rst_ni(rst_ni);
            for (int b = 0; b < NUM_BANK; ++b) {
                bank_mon->req_i[b](xbar_bank_req[b]);
                bank_mon->addr_i[b](xbar_bank_addr[b]);
                bank_mon->we_i[b](xbar_bank_we[b]);
                bank_mon->be_i[b](xbar_bank_be[b]);
                bank_mon->wdata_i[b](xbar_bank_wdata[b]);
                bank_mon->gnt_i[b](xbar_bank_gnt[b]);
                bank_mon->rvalid_i[b](xbar_bank_rvalid[b]);
                bank_mon->rdata_i[b](xbar_bank_rdata[b]);
            }
        }
    }
};

#endif
