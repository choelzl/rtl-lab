// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC design top (DUT) for the crossbar design. It is the
//   synthesizable half: it instantiates the crossbar interconnect and the
//   N_BANK memory banks and wires the crossbar's bank side to the banks through
//   internal sc_signals. Its external ports are the manager-side OBI channels
//   (N_MGR = N_AGU * N_REQ of them), which the harness (tb/systemc/
//   tb_top_crossbar.cpp) connects to the AGUs.
//
//     top_crossbar
//       |- crossbar<N_MGR,N_BANK,WORD_BYTES> xbar
//       |- bank<N_ROW,WORD_BYTES>            banks[N_BANK]
//
//   Manager-side ports (forwarded 1:1 to the crossbar):
//     in : m_req_i, m_addr_i, m_we_i, m_be_i, m_wdata_i [N_MGR]
//     out: m_gnt_o, m_rvalid_o, m_rdata_o               [N_MGR]
//
//   Reset is active-low (rst_ni).
//
// Template parameters:
//   N_AGU      - number of AGUs (managers)      (default 2)
//   N_REQ      - request ports per AGU          (default 4)
//   N_BANK     - number of memory banks         (default 8)
//   N_ROW      - rows (words) per bank          (default 1024)
//   WORD_BYTES - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef TOP_CROSSBAR_HPP
#define TOP_CROSSBAR_HPP

#include <systemc.h>

#include <cstdint>

#include "bank.hpp"
#include "crossbar.hpp"

template <int N_AGU = 2, int N_REQ = 4, int N_BANK = 8, int N_ROW = 1024,
          int WORD_BYTES = 4>
SC_MODULE(top_crossbar) {
    static constexpr int N_MGR = N_AGU * N_REQ;

    sc_in<bool>         clk_i;
    sc_in<bool>         rst_ni;

    sc_in<bool>         m_req_i[N_MGR];
    sc_in<uint64_t>     m_addr_i[N_MGR];
    sc_in<bool>         m_we_i[N_MGR];
    sc_in<uint32_t>     m_be_i[N_MGR];
    sc_in<uint64_t>     m_wdata_i[N_MGR];
    sc_out<bool>        m_gnt_o[N_MGR];
    sc_out<bool>        m_rvalid_o[N_MGR];
    sc_out<uint64_t>    m_rdata_o[N_MGR];

    sc_signal<bool>     b_req[N_BANK], b_we[N_BANK], b_gnt[N_BANK], b_rvalid[N_BANK];
    sc_signal<uint64_t> b_addr[N_BANK], b_wdata[N_BANK], b_rdata[N_BANK];
    sc_signal<uint32_t> b_be[N_BANK];

    crossbar<N_MGR, N_BANK, WORD_BYTES> xbar;
    sc_vector<bank<N_ROW, WORD_BYTES>>  banks;

    SC_CTOR(top_crossbar) : xbar("xbar"), banks("bank") {
        banks.init(N_BANK);

        xbar.clk_i(clk_i);
        xbar.rst_ni(rst_ni);

        for (int m = 0; m < N_MGR; ++m) {
            xbar.m_req_i[m](m_req_i[m]);
            xbar.m_addr_i[m](m_addr_i[m]);
            xbar.m_we_i[m](m_we_i[m]);
            xbar.m_be_i[m](m_be_i[m]);
            xbar.m_wdata_i[m](m_wdata_i[m]);
            xbar.m_gnt_o[m](m_gnt_o[m]);
            xbar.m_rvalid_o[m](m_rvalid_o[m]);
            xbar.m_rdata_o[m](m_rdata_o[m]);
        }

        for (int b = 0; b < N_BANK; ++b) {
            xbar.b_req_o[b](b_req[b]);
            xbar.b_addr_o[b](b_addr[b]);
            xbar.b_we_o[b](b_we[b]);
            xbar.b_be_o[b](b_be[b]);
            xbar.b_wdata_o[b](b_wdata[b]);
            xbar.b_gnt_i[b](b_gnt[b]);
            xbar.b_rvalid_i[b](b_rvalid[b]);
            xbar.b_rdata_i[b](b_rdata[b]);

            banks[b].clk_i(clk_i);
            banks[b].rst_ni(rst_ni);
            banks[b].req_i(b_req[b]);
            banks[b].addr_i(b_addr[b]);
            banks[b].we_i(b_we[b]);
            banks[b].be_i(b_be[b]);
            banks[b].wdata_i(b_wdata[b]);
            banks[b].gnt_o(b_gnt[b]);
            banks[b].rvalid_o(b_rvalid[b]);
            banks[b].rdata_o(b_rdata[b]);
        }
    }
};

#endif
