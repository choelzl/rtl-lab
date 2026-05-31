// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC design top (DUT) for the crossbar design. It is the
//   synthesizable half: it instantiates the crossbar interconnect and the
//   NUM_BANK memory banks and wires the crossbar's bank side to the banks through
//   internal sc_signals. Its external ports are the manager-side OBI channels
//   (NUM_MGR = NUM_AGU * NUM_REQ of them), which the harness (tb/systemc/
//   tb_top_crossbar.cpp) connects to the AGUs.
//
//     top_crossbar
//       |- crossbar<NUM_MGR,NUM_BANK,BYTES_PER_WORD> xbar
//       |- bank<NUM_ROW,BYTES_PER_WORD>            banks[NUM_BANK]
//
//   Manager-side ports (forwarded 1:1 to the crossbar):
//     in : m_req_i, m_addr_i, m_we_i, m_be_i, m_wdata_i [NUM_MGR]
//     out: m_gnt_o, m_rvalid_o, m_rdata_o               [NUM_MGR]
//
//   Reset is active-low (rst_ni).
//
// Template parameters (set from the flow's PARAMS macros N_AGU, N_REQ, N_BANK,
// N_ROW, WORD_BYTES):
//   NUM_AGU    - number of AGUs (managers) (default 2)
//   NUM_REQ    - request ports per AGU (default 4)
//   NUM_BANK   - number of memory banks (default 8)
//   NUM_ROW    - rows (words) per bank (default 1024)
//   BYTES_PER_WORD - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef TOP_CROSSBAR_HPP
#define TOP_CROSSBAR_HPP

#include <systemc.h>

#include <cstdint>

#include "bank.hpp"
#include "crossbar.hpp"

template <int NUM_AGU = 2, int NUM_REQ = 4, int NUM_BANK = 8, int NUM_ROW = 1024,
          int BYTES_PER_WORD = 4>
SC_MODULE(top_crossbar) {
    static constexpr int NUM_MGR = NUM_AGU * NUM_REQ;

    sc_in<bool>         clk_i;
    sc_in<bool>         rst_ni;

    sc_in<bool>         m_req_i[NUM_MGR];
    sc_in<uint64_t>     m_addr_i[NUM_MGR];
    sc_in<bool>         m_we_i[NUM_MGR];
    sc_in<uint32_t>     m_be_i[NUM_MGR];
    sc_in<uint64_t>     m_wdata_i[NUM_MGR];
    sc_out<bool>        m_gnt_o[NUM_MGR];
    sc_out<bool>        m_rvalid_o[NUM_MGR];
    sc_out<uint64_t>    m_rdata_o[NUM_MGR];

    sc_signal<bool>     b_req[NUM_BANK], b_we[NUM_BANK], b_gnt[NUM_BANK], b_rvalid[NUM_BANK];
    sc_signal<uint64_t> b_addr[NUM_BANK], b_wdata[NUM_BANK], b_rdata[NUM_BANK];
    sc_signal<uint32_t> b_be[NUM_BANK];

    crossbar<NUM_MGR, NUM_BANK, BYTES_PER_WORD> xbar;
    sc_vector<bank<NUM_ROW, BYTES_PER_WORD>>  banks;

    SC_CTOR(top_crossbar) : xbar("xbar"), banks("bank") {
        banks.init(NUM_BANK);

        xbar.clk_i(clk_i);
        xbar.rst_ni(rst_ni);

        for (int m = 0; m < NUM_MGR; ++m) {
            xbar.m_req_i[m](m_req_i[m]);
            xbar.m_addr_i[m](m_addr_i[m]);
            xbar.m_we_i[m](m_we_i[m]);
            xbar.m_be_i[m](m_be_i[m]);
            xbar.m_wdata_i[m](m_wdata_i[m]);
            xbar.m_gnt_o[m](m_gnt_o[m]);
            xbar.m_rvalid_o[m](m_rvalid_o[m]);
            xbar.m_rdata_o[m](m_rdata_o[m]);
        }

        for (int b = 0; b < NUM_BANK; ++b) {
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
