// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC crossbar interconnect for the crossbar design — an
//   NUM_IN x NUM_OUT switch between OBI managers (AGU request ports) and OBI
//   subordinate banks. Implements the simplified single-channel OBI protocol
//   (see doc/specs/obi.md) with word-interleaved banking and a per-bank
//   round-robin arbiter.
//
//   Manager side (the crossbar is the subordinate seen by each manager):
//     in : m_req_i, m_addr_i (global byte addr), m_we_i, m_be_i, m_wdata_i  [NUM_IN]
//     out: m_gnt_o, m_rvalid_o, m_rdata_o                                   [NUM_IN]
//   Bank side (the crossbar is the manager seen by each bank):
//     out: b_req_o, b_addr_o (bank-local byte addr), b_we_o, b_be_o, b_wdata_o [NUM_OUT]
//     in : b_gnt_i, b_rvalid_i, b_rdata_i                                      [NUM_OUT]
//
//   Address decode (word-interleaved, see doc/specs/crossbar.md):
//     word = addr / BYTES_PER_WORD ; bank = word % NUM_OUT ; row = word / NUM_OUT
//   The bank receives the bank-local byte address row * BYTES_PER_WORD.
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
// Template parameters (NUM_IN = N_AGU*N_REQ; NUM_OUT, BYTES_PER_WORD from PARAMS):
//   NUM_IN        - number of manager request ports (default 8)
//   NUM_OUT       - number of slave ports (default 8)
//   BYTES_PER_WORD - bytes per word; used only when SEL_LEN == 0 (default 4)
//   SEL_START      - LSB of routing address slice; ignored when SEL_LEN == 0 (default 0)
//   SEL_LEN        - slice width; 0 = use legacy word-interleaved routing (default 0)
//
//   When SEL_LEN == 0 (default): routes by (addr/BYTES_PER_WORD) % NUM_OUT and
//   delivers the bank-local address to the slave (routing bits stripped).
//   When SEL_LEN > 0: routes by (addr >> SEL_START) & mask and passes the full
//   address through to the slave unchanged (caller strips routing bits).
// -----------------------------------------------------------------------------

#ifndef CROSSBAR_HPP
#define CROSSBAR_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_IN = 8, int NUM_OUT = 8, int BYTES_PER_WORD = 4, int SEL_START = 0,
          int SEL_LEN = 0>
SC_MODULE(crossbar) {
    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    sc_in<bool>      m_req_i[NUM_IN];
    sc_in<uint64_t>  m_addr_i[NUM_IN];
    sc_in<bool>      m_we_i[NUM_IN];
    sc_in<uint32_t>  m_be_i[NUM_IN];
    sc_in<uint64_t>  m_wdata_i[NUM_IN];
    sc_out<bool>     m_gnt_o[NUM_IN];
    sc_out<bool>     m_rvalid_o[NUM_IN];
    sc_out<uint64_t> m_rdata_o[NUM_IN];

    sc_out<bool>     b_req_o[NUM_OUT];
    sc_out<uint64_t> b_addr_o[NUM_OUT];
    sc_out<bool>     b_we_o[NUM_OUT];
    sc_out<uint32_t> b_be_o[NUM_OUT];
    sc_out<uint64_t> b_wdata_o[NUM_OUT];
    sc_in<bool>      b_gnt_i[NUM_OUT];
    sc_in<bool>      b_rvalid_i[NUM_OUT];
    sc_in<uint64_t>  b_rdata_i[NUM_OUT];

    sc_signal<int> rr_ptr[NUM_OUT];
    sc_signal<int> win_[NUM_OUT];
    sc_signal<int> owner[NUM_OUT];

    static int bank_of(uint64_t a) {
        if constexpr (SEL_LEN > 0)
            return static_cast<int>((a >> SEL_START) & ((1 << SEL_LEN) - 1));
        else
            return static_cast<int>((a / BYTES_PER_WORD) % NUM_OUT);
    }
    static uint64_t slave_addr(uint64_t a) {
        if constexpr (SEL_LEN > 0)
            return a;
        else
            return (a / BYTES_PER_WORD / NUM_OUT) * static_cast<uint64_t>(BYTES_PER_WORD);
    }

    void comb() {
        for (int m = 0; m < NUM_IN; ++m) {
            m_gnt_o[m].write(false);
            m_rvalid_o[m].write(false);
            m_rdata_o[m].write(0);
        }
        for (int b = 0; b < NUM_OUT; ++b) {
            b_req_o[b].write(false);
            b_addr_o[b].write(0);
            b_we_o[b].write(false);
            b_be_o[b].write(0);
            b_wdata_o[b].write(0);
        }

        for (int b = 0; b < NUM_OUT; ++b) {
            int       winner = -1;
            const int start  = rr_ptr[b].read();
            for (int k = 0; k < NUM_IN; ++k) {
                const int m = (start + k) % NUM_IN;
                if (m_req_i[m].read() && bank_of(m_addr_i[m].read()) == b) {
                    winner = m;
                    break;
                }
            }
            win_[b].write(winner);

            if (winner >= 0) {
                b_req_o[b].write(true);
                b_addr_o[b].write(slave_addr(m_addr_i[winner].read()));
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
            for (int b = 0; b < NUM_OUT; ++b) {
                rr_ptr[b].write(0);
                owner[b].write(-1);
            }
            return;
        }
        for (int b = 0; b < NUM_OUT; ++b) {
            const int w = win_[b].read();
            owner[b].write(w);
            if (w >= 0)
                rr_ptr[b].write((w + 1) % NUM_IN);
        }
    }

    SC_CTOR(crossbar) {
        SC_METHOD(comb);
        for (int m = 0; m < NUM_IN; ++m)
            sensitive << m_req_i[m] << m_addr_i[m] << m_we_i[m] << m_be_i[m] << m_wdata_i[m];
        for (int b = 0; b < NUM_OUT; ++b)
            sensitive << b_gnt_i[b] << b_rvalid_i[b] << b_rdata_i[b] << rr_ptr[b] << owner[b];

        SC_METHOD(seq);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
