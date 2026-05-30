// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC crossbar interconnect for the crossbar design — an
//   N_MGR x N_BANK switch between OBI managers (AGU request ports) and OBI
//   subordinate banks. Implements the simplified single-channel OBI protocol
//   (see doc/specs/obi.md) with word-interleaved banking and a per-bank
//   round-robin arbiter.
//
//   Manager side (the crossbar is the subordinate seen by each manager):
//     in : m_req_i, m_addr_i (global byte addr), m_we_i, m_be_i, m_wdata_i  [N_MGR]
//     out: m_gnt_o, m_rvalid_o, m_rdata_o                                   [N_MGR]
//   Bank side (the crossbar is the manager seen by each bank):
//     out: b_req_o, b_addr_o (bank-local byte addr), b_we_o, b_be_o, b_wdata_o [N_BANK]
//     in : b_gnt_i, b_rvalid_i, b_rdata_i                                      [N_BANK]
//
//   Address decode (word-interleaved, see doc/specs/crossbar.md):
//     word = addr / WORD_BYTES ; bank = word % N_BANK ; row = word / N_BANK
//   The bank receives the bank-local byte address row * WORD_BYTES.
//
//   Routing & arbitration (the data path is combinational — the crossbar adds
//   no pipeline register; only small per-bank control state is registered):
//     - Each cycle, for every bank, the requesters targeting it are arbitrated
//       round-robin (rr_ptr advances past the winner). The winner's request is
//       forwarded to the bank and its grant returned; losing managers keep
//       their request asserted (the AGU holds it) until their turn — the source
//       of the conflict penalty.
//     - Connection / response routing: the bank's response arrives one cycle
//       after its grant (1-cycle bank latency), so the manager that won bank b
//       is remembered in a per-bank owner register. When the bank raises rvalid
//       the next cycle, its response is steered back to that owner. No
//       transaction IDs are needed.
//
//   Reset is active-low (rst_ni). Banks are always-ready (gnt = req), so the
//   arbiter winner is effectively granted every cycle.
//
// Template parameters:
//   N_MGR      - number of manager request ports (default 8)
//   N_BANK     - number of memory banks          (default 8)
//   WORD_BYTES - bytes per word / OBI data beat  (default 4)
// -----------------------------------------------------------------------------

#ifndef CROSSBAR_HPP
#define CROSSBAR_HPP

#include <systemc.h>

#include <cstdint>

template <int N_MGR = 8, int N_BANK = 8, int WORD_BYTES = 4>
SC_MODULE(crossbar) {
    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;

    sc_in<bool>      m_req_i[N_MGR];
    sc_in<uint64_t>  m_addr_i[N_MGR];
    sc_in<bool>      m_we_i[N_MGR];
    sc_in<uint32_t>  m_be_i[N_MGR];
    sc_in<uint64_t>  m_wdata_i[N_MGR];
    sc_out<bool>     m_gnt_o[N_MGR];
    sc_out<bool>     m_rvalid_o[N_MGR];
    sc_out<uint64_t> m_rdata_o[N_MGR];

    sc_out<bool>     b_req_o[N_BANK];
    sc_out<uint64_t> b_addr_o[N_BANK];
    sc_out<bool>     b_we_o[N_BANK];
    sc_out<uint32_t> b_be_o[N_BANK];
    sc_out<uint64_t> b_wdata_o[N_BANK];
    sc_in<bool>      b_gnt_i[N_BANK];
    sc_in<bool>      b_rvalid_i[N_BANK];
    sc_in<uint64_t>  b_rdata_i[N_BANK];

    sc_signal<int>   rr_ptr[N_BANK];    // round-robin start index
    sc_signal<int>   win_[N_BANK];      // this cycle's winner (-1 = none)
    sc_signal<int>   owner[N_BANK];     // last cycle's winner (grant->response)

    static int bank_of(uint64_t a) {
        return static_cast<int>((a / WORD_BYTES) % N_BANK);
    }
    static uint64_t local_of(uint64_t a) {
        return (a / WORD_BYTES / N_BANK) * static_cast<uint64_t>(WORD_BYTES);
    }

    void comb() {
        for (int m = 0; m < N_MGR; ++m) {
            m_gnt_o[m].write(false);
            m_rvalid_o[m].write(false);
            m_rdata_o[m].write(0);
        }
        for (int b = 0; b < N_BANK; ++b) {
            b_req_o[b].write(false);
            b_addr_o[b].write(0);
            b_we_o[b].write(false);
            b_be_o[b].write(0);
            b_wdata_o[b].write(0);
        }

        for (int b = 0; b < N_BANK; ++b) {
            int winner = -1;
            const int start = rr_ptr[b].read();
            for (int k = 0; k < N_MGR; ++k) {
                const int m = (start + k) % N_MGR;
                if (m_req_i[m].read() && bank_of(m_addr_i[m].read()) == b) {
                    winner = m;
                    break;
                }
            }
            win_[b].write(winner);

            if (winner >= 0) {
                b_req_o[b].write(true);
                b_addr_o[b].write(local_of(m_addr_i[winner].read()));
                b_we_o[b].write(m_we_i[winner].read());
                b_be_o[b].write(m_be_i[winner].read());
                b_wdata_o[b].write(m_wdata_i[winner].read());
                m_gnt_o[winner].write(b_gnt_i[b].read());
            }

            const int ow = owner[b].read();
            if (ow >= 0 && b_rvalid_i[b].read()) {
                m_rvalid_o[ow].write(true);
                m_rdata_o[ow].write(b_rdata_i[b].read());
            }
        }
    }

    void seq() {
        if (!rst_ni.read()) {
            for (int b = 0; b < N_BANK; ++b) {
                rr_ptr[b].write(0);
                owner[b].write(-1);
            }
            return;
        }
        for (int b = 0; b < N_BANK; ++b) {
            const int w = win_[b].read();
            owner[b].write(w);                             // 1-cycle grant->resp
            if (w >= 0) rr_ptr[b].write((w + 1) % N_MGR);
        }
    }

    SC_CTOR(crossbar) {
        SC_METHOD(comb);
        for (int m = 0; m < N_MGR; ++m)
            sensitive << m_req_i[m] << m_addr_i[m] << m_we_i[m] << m_be_i[m]
                      << m_wdata_i[m];
        for (int b = 0; b < N_BANK; ++b)
            sensitive << b_gnt_i[b] << b_rvalid_i[b] << b_rdata_i[b] << rr_ptr[b]
                      << owner[b];

        SC_METHOD(seq);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
