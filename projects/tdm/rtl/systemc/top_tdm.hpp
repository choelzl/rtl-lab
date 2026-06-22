// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC design top (DUT) for the TDM design. It instantiates the full
//   TDM datapath and the memory banks:
//
//     AGU(ext) --x-OBI--> buf[NUM_AGU] --> x_obi_mux --> tdm --> crossbar --> bank[NUM_BANK]
//                              ^ arbiter (sel_req/sel_rsp)   ^ tdm_mux (map params)
//
//   - buf[a]            : per-AGU x-OBI buffer / 4<->8 width converter.
//   - arbiter           : round-robin slot scheduler (sel_req + delayed sel_rsp).
//   - x_obi_mux         : x-OBI mux+demux time-sharing the 8-wide port.
//   - tdm_mux           : selects the active AGU's mapping parameters.
//   - tdm               : maps the group's base + parameters to 8 bank/row
//                         placements, emitted as 8 word addresses.
//   - crossbar + bank[] : REUSED from the crossbar design — bank/row decode and
//                         per-bank round-robin arbitration (the conflict resolver).
//
//   External ports are the AGU-facing x-OBI group channels (one base/we/be per
//   AGU, NUM_REQ per-word req/wdata/gnt/rvalid/rdata) plus the per-AGU mapping
//   parameters, which the harness (tb/systemc/tb_top_tdm.cpp) connects to the AGUs.
//
//   Sizes: NUM_WORD = NUM_AGU*NUM_REQ words per slot = crossbar managers; the
//   crossbar is NUM_WORD x NUM_BANK.
//
//   Reset is active-low (rst_ni).
//
// Template parameters (set from the flow's PARAMS macros N_AGU, N_REQ, N_BANK,
// N_ROW, WORD_BYTES):
//   NUM_AGU        - number of AGUs (default 2)
//   NUM_REQ        - AGU-side group width (default 4)
//   NUM_BANK       - number of memory banks (default 32; the mapping needs >= 32)
//   NUM_ROW        - rows (words) per bank (default 1024)
//   BYTES_PER_WORD - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef TOP_TDM_HPP
#define TOP_TDM_HPP

#include <systemc.h>

#include <cstdint>

#include "arbiter.hpp"
#include "bank.hpp"
#include "buf.hpp"
#include "crossbar.hpp"
#include "tdm.hpp"
#include "tdm_mux.hpp"
#include "x_obi_mux.hpp"

template <int NUM_AGU = 2, int NUM_REQ = 4, int NUM_BANK = 32, int NUM_ROW = 1024,
          int BYTES_PER_WORD = 4>
SC_MODULE(top_tdm) {
    static constexpr int NUM_WORD = NUM_AGU * NUM_REQ;
    static constexpr int NUM_MGR  = NUM_WORD;
    static constexpr int NUM_REQT = NUM_AGU * NUM_REQ;
    static constexpr int NUM_IN   = NUM_AGU * NUM_WORD;

    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>      a_req_i[NUM_REQT];
    sc_in<uint64_t>  a_wdata_i[NUM_REQT];
    sc_out<bool>     a_gnt_o[NUM_REQT];
    sc_out<bool>     a_rvalid_o[NUM_REQT];
    sc_out<uint64_t> a_rdata_o[NUM_REQT];
    sc_in<uint64_t>  a_addr_i[NUM_AGU];
    sc_in<bool>      a_we_i[NUM_AGU];
    sc_in<uint32_t>  a_be_i[NUM_AGU];
    sc_in<uint64_t>  a_num_banks_i[NUM_AGU];
    sc_in<uint64_t>  a_bank_width_i[NUM_AGU];
    sc_in<uint64_t>  a_r_i[NUM_AGU];
    sc_in<uint64_t>  a_c_i[NUM_AGU];
    sc_in<uint64_t>  a_l_i[NUM_AGU];
    sc_in<uint64_t>  a_store_mode_i[NUM_AGU];

    sc_signal<bool>     bx_req[NUM_IN], bx_gnt[NUM_IN], bx_rvalid[NUM_IN];
    sc_signal<uint64_t> bx_wdata[NUM_IN], bx_rdata[NUM_IN];
    sc_signal<uint64_t> bx_addr[NUM_AGU];
    sc_signal<bool>     bx_we[NUM_AGU];
    sc_signal<uint32_t> bx_be[NUM_AGU];

    sc_signal<bool>     xt_req[NUM_WORD], xt_gnt[NUM_WORD], xt_rvalid[NUM_WORD];
    sc_signal<uint64_t> xt_wdata[NUM_WORD], xt_rdata[NUM_WORD];
    sc_signal<uint64_t> xt_addr;
    sc_signal<bool>     xt_we;
    sc_signal<uint32_t> xt_be;

    sc_signal<bool>     tc_req[NUM_MGR], tc_we[NUM_MGR], tc_gnt[NUM_MGR], tc_rvalid[NUM_MGR];
    sc_signal<uint64_t> tc_addr[NUM_MGR], tc_wdata[NUM_MGR], tc_rdata[NUM_MGR];
    sc_signal<uint32_t> tc_be[NUM_MGR];

    sc_signal<bool>     cb_req[NUM_BANK], cb_we[NUM_BANK], cb_gnt[NUM_BANK], cb_rvalid[NUM_BANK];
    sc_signal<uint64_t> cb_addr[NUM_BANK], cb_wdata[NUM_BANK], cb_rdata[NUM_BANK];
    sc_signal<uint32_t> cb_be[NUM_BANK];

    sc_signal<int>      sel_req, sel_rsp;
    sc_signal<uint64_t> map_num_banks, map_bank_width, map_r, map_c, map_l, map_store_mode;

    sc_vector<buf<NUM_REQ, NUM_WORD>>           bufs;
    arbiter<NUM_AGU>                            arb;
    x_obi_mux<NUM_AGU, NUM_WORD>                xmux;
    tdm_mux<NUM_AGU>                            tmux;
    tdm<NUM_WORD, NUM_BANK, BYTES_PER_WORD>     mapf;
    crossbar<NUM_MGR, NUM_BANK, BYTES_PER_WORD> xbar;
    sc_vector<bank<NUM_ROW, BYTES_PER_WORD>>    banks;

    SC_CTOR(top_tdm)
        : bufs("buf"), arb("arb"), xmux("xmux"), tmux("tmux"), mapf("tdm"), xbar("xbar"),
          banks("bank") {
        bufs.init(NUM_AGU);
        banks.init(NUM_BANK);

        arb.clk_i(clk_i);
        arb.rst_ni(rst_ni);
        arb.sel_req_o(sel_req);
        arb.sel_rsp_o(sel_rsp);

        for (int a = 0; a < NUM_AGU; ++a) {
            bufs[a].clk_i(clk_i);
            bufs[a].rst_ni(rst_ni);
            bufs[a].a_addr_i(a_addr_i[a]);
            bufs[a].a_we_i(a_we_i[a]);
            bufs[a].a_be_i(a_be_i[a]);
            for (int p = 0; p < NUM_REQ; ++p) {
                const int m = a * NUM_REQ + p;
                bufs[a].a_req_i[p](a_req_i[m]);
                bufs[a].a_wdata_i[p](a_wdata_i[m]);
                bufs[a].a_gnt_o[p](a_gnt_o[m]);
                bufs[a].a_rvalid_o[p](a_rvalid_o[m]);
                bufs[a].a_rdata_o[p](a_rdata_o[m]);
            }
            bufs[a].m_addr_o(bx_addr[a]);
            bufs[a].m_we_o(bx_we[a]);
            bufs[a].m_be_o(bx_be[a]);
            for (int w = 0; w < NUM_WORD; ++w) {
                const int i = a * NUM_WORD + w;
                bufs[a].m_req_o[w](bx_req[i]);
                bufs[a].m_wdata_o[w](bx_wdata[i]);
                bufs[a].m_gnt_i[w](bx_gnt[i]);
                bufs[a].m_rvalid_i[w](bx_rvalid[i]);
                bufs[a].m_rdata_i[w](bx_rdata[i]);
            }
        }

        xmux.sel_req_i(sel_req);
        xmux.sel_rsp_i(sel_rsp);
        for (int a = 0; a < NUM_AGU; ++a) {
            xmux.m_addr_i[a](bx_addr[a]);
            xmux.m_we_i[a](bx_we[a]);
            xmux.m_be_i[a](bx_be[a]);
        }
        for (int i = 0; i < NUM_IN; ++i) {
            xmux.m_req_i[i](bx_req[i]);
            xmux.m_wdata_i[i](bx_wdata[i]);
            xmux.m_gnt_o[i](bx_gnt[i]);
            xmux.m_rvalid_o[i](bx_rvalid[i]);
            xmux.m_rdata_o[i](bx_rdata[i]);
        }
        xmux.s_addr_o(xt_addr);
        xmux.s_we_o(xt_we);
        xmux.s_be_o(xt_be);
        for (int w = 0; w < NUM_WORD; ++w) {
            xmux.s_req_o[w](xt_req[w]);
            xmux.s_wdata_o[w](xt_wdata[w]);
            xmux.s_gnt_i[w](xt_gnt[w]);
            xmux.s_rvalid_i[w](xt_rvalid[w]);
            xmux.s_rdata_i[w](xt_rdata[w]);
        }

        tmux.sel_i(sel_req);
        for (int a = 0; a < NUM_AGU; ++a) {
            tmux.num_banks_i[a](a_num_banks_i[a]);
            tmux.bank_width_i[a](a_bank_width_i[a]);
            tmux.r_i[a](a_r_i[a]);
            tmux.c_i[a](a_c_i[a]);
            tmux.l_i[a](a_l_i[a]);
            tmux.store_mode_i[a](a_store_mode_i[a]);
        }
        tmux.num_banks_o(map_num_banks);
        tmux.bank_width_o(map_bank_width);
        tmux.r_o(map_r);
        tmux.c_o(map_c);
        tmux.l_o(map_l);
        tmux.store_mode_o(map_store_mode);

        mapf.g_addr_i(xt_addr);
        mapf.g_we_i(xt_we);
        mapf.g_be_i(xt_be);
        mapf.num_banks_i(map_num_banks);
        mapf.bank_width_i(map_bank_width);
        mapf.r_i(map_r);
        mapf.c_i(map_c);
        mapf.l_i(map_l);
        mapf.store_mode_i(map_store_mode);
        for (int w = 0; w < NUM_WORD; ++w) {
            mapf.g_req_i[w](xt_req[w]);
            mapf.g_wdata_i[w](xt_wdata[w]);
            mapf.g_gnt_o[w](xt_gnt[w]);
            mapf.g_rvalid_o[w](xt_rvalid[w]);
            mapf.g_rdata_o[w](xt_rdata[w]);
            mapf.c_req_o[w](tc_req[w]);
            mapf.c_addr_o[w](tc_addr[w]);
            mapf.c_we_o[w](tc_we[w]);
            mapf.c_be_o[w](tc_be[w]);
            mapf.c_wdata_o[w](tc_wdata[w]);
            mapf.c_gnt_i[w](tc_gnt[w]);
            mapf.c_rvalid_i[w](tc_rvalid[w]);
            mapf.c_rdata_i[w](tc_rdata[w]);
        }

        xbar.clk_i(clk_i);
        xbar.rst_ni(rst_ni);
        for (int m = 0; m < NUM_MGR; ++m) {
            xbar.m_req_i[m](tc_req[m]);
            xbar.m_addr_i[m](tc_addr[m]);
            xbar.m_we_i[m](tc_we[m]);
            xbar.m_be_i[m](tc_be[m]);
            xbar.m_wdata_i[m](tc_wdata[m]);
            xbar.m_gnt_o[m](tc_gnt[m]);
            xbar.m_rvalid_o[m](tc_rvalid[m]);
            xbar.m_rdata_o[m](tc_rdata[m]);
        }
        for (int b = 0; b < NUM_BANK; ++b) {
            xbar.b_req_o[b](cb_req[b]);
            xbar.b_addr_o[b](cb_addr[b]);
            xbar.b_we_o[b](cb_we[b]);
            xbar.b_be_o[b](cb_be[b]);
            xbar.b_wdata_o[b](cb_wdata[b]);
            xbar.b_gnt_i[b](cb_gnt[b]);
            xbar.b_rvalid_i[b](cb_rvalid[b]);
            xbar.b_rdata_i[b](cb_rdata[b]);

            banks[b].clk_i(clk_i);
            banks[b].rst_ni(rst_ni);
            banks[b].req_i(cb_req[b]);
            banks[b].addr_i(cb_addr[b]);
            banks[b].we_i(cb_we[b]);
            banks[b].be_i(cb_be[b]);
            banks[b].wdata_i(cb_wdata[b]);
            banks[b].gnt_o(cb_gnt[b]);
            banks[b].rvalid_o(cb_rvalid[b]);
            banks[b].rdata_o(cb_rdata[b]);
        }
    }
};

#endif
