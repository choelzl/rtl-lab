// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// Description:
//   Native SystemC crossbar interconnect for the crossbar design — an
//   NUM_IN x NUM_OUT switch between OBI manager ports and OBI
//   subordinate banks. Implements the simplified single-channel OBI protocol
//   (see doc/specs/obi.md) with word-interleaved banking and a per-bank
//   round-robin arbiter.
//
//   Manager side (the crossbar is the subordinate seen by each manager) —
//   m_ports[NUM_IN], see obi_ports.hpp's obi_subordinate_ports:
//     in : req_i, addr_i (global byte addr), we_i, be_i, wdata_i
//     out: gnt_o, rvalid_o, rdata_o
//   Bank side (the crossbar is the manager seen by each bank) —
//   b_ports[NUM_OUT], see obi_ports.hpp's obi_manager_ports:
//     out: req_o, addr_o (bank-local byte addr), we_o, be_o, wdata_o
//     in : gnt_i, rvalid_i, rdata_i
//
//   Address decode (beat-interleaved — this primitive's default SEL_LEN==0
//   mode, reused by the TDM backend; see doc/specs/map_func.md's "Use in
//   the TDM design". The three-level crossbar BACKEND in doc/specs/
//   crossbar.md instead composes this primitive with SEL_LEN>0 bit-field
//   routing at each level):
//     beat = addr / BYTES_PER_ROW ; bank = beat % NUM_OUT ; row = beat / NUM_OUT
//   The bank receives the bank-local byte address row * BYTES_PER_ROW.
//
//   Routing & arbitration (the data path is combinational — the crossbar adds
//   no pipeline register; only small per-bank control state is registered):
//     - Each cycle, for every bank, the requesters targeting it are arbitrated
//       round-robin (rr_ptr advances past the winner). The winner's request is
//       forwarded to the bank and its grant returned; losing managers keep
//       their request asserted until their turn — the source
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
// Template parameters (NUM_IN = port_count*NUM_REQ; NUM_OUT, BYTES_PER_ROW from PARAMS):
//   NUM_IN        - number of manager request ports (default 8)
//   NUM_OUT       - number of slave ports (default 8)
//   BYTES_PER_ROW - bytes per OBI data beat = WORDS_PER_ROW*BYTES_PER_WORD; used
//                   when SEL_LEN == 0 to compute bank index and slave address (default 16)
//   SEL_START      - LSB of routing address slice; ignored when SEL_LEN == 0 (default 0)
//   SEL_LEN        - slice width; 0 = use legacy word-interleaved routing (default 0)
//
//   When SEL_LEN == 0 (default): routes by (addr/BYTES_PER_ROW) % NUM_OUT and
//   delivers the bank-local address to the slave (routing bits stripped).
//   When SEL_LEN > 0: routes by (addr >> SEL_START) & mask and passes the full
//   address through to the slave unchanged (caller strips routing bits).
// -----------------------------------------------------------------------------

#ifndef CROSSBAR_HPP
#define CROSSBAR_HPP

#include <systemc.h>

#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <cstdint>

template <int NUM_IN = 8, int NUM_OUT = 8, int BYTES_PER_ROW = 4 * 4, int SEL_START = 0,
          int SEL_LEN = 0>
SC_MODULE(crossbar) {
    using data_t = obi_data<BYTES_PER_ROW>;

    // A SEL_LEN-wide field can address 2^SEL_LEN outputs; if that exceeds
    // NUM_OUT, an in-range address could decode to a bank that doesn't
    // exist and the request would silently never be served (the manager
    // hangs). All of top_crossbar.hpp's levels keep 2^SEL_LEN == NUM_OUT.
    static_assert(SEL_LEN == 0 || (1 << SEL_LEN) <= NUM_OUT,
                  "SEL_LEN routing field must not address more banks than NUM_OUT");

    sc_in<bool> clk_i;
    sc_in<bool> rst_ni;

    // m_ports (manager-facing, subordinate role) / b_ports (bank-facing,
    // manager role) — named to avoid colliding with the `m`/`b` loop
    // variables used throughout comb()/seq() below.
    obi_subordinate_ports<data_t> m_ports[NUM_IN];
    obi_manager_ports<data_t>     b_ports[NUM_OUT];

    sc_signal<int> rr_ptr[NUM_OUT];
    sc_signal<int> win_[NUM_OUT];
    sc_signal<int> owner[NUM_OUT];

    static int bank_of(uint64_t a) {
        if constexpr (SEL_LEN > 0)
            return static_cast<int>((a >> SEL_START) & ((1 << SEL_LEN) - 1));
        else
            return static_cast<int>((a / BYTES_PER_ROW) % NUM_OUT);
    }
    static uint64_t slave_addr(uint64_t a) {
        if constexpr (SEL_LEN > 0)
            return a;
        else
            return (a / BYTES_PER_ROW / NUM_OUT) * static_cast<uint64_t>(BYTES_PER_ROW);
    }

    void comb() {
        for (int m = 0; m < NUM_IN; ++m) {
            m_ports[m].gnt_o.write(false);
            m_ports[m].rvalid_o.write(false);
            m_ports[m].rdata_o.write(0);
        }
        for (int b = 0; b < NUM_OUT; ++b) {
            b_ports[b].req_o.write(false);
            b_ports[b].addr_o.write(0);
            b_ports[b].we_o.write(false);
            b_ports[b].be_o.write(0);
            b_ports[b].wdata_o.write(0);
        }

        for (int b = 0; b < NUM_OUT; ++b) {
            int       winner = -1;
            const int start  = rr_ptr[b].read();
            for (int k = 0; k < NUM_IN; ++k) {
                const int m = (start + k) % NUM_IN;
                if (m_ports[m].req_i.read() && bank_of(m_ports[m].addr_i.read()) == b) {
                    winner = m;
                    break;
                }
            }
            win_[b].write(winner);

            if (winner >= 0) {
                b_ports[b].req_o.write(true);
                b_ports[b].addr_o.write(slave_addr(m_ports[winner].addr_i.read()));
                b_ports[b].we_o.write(m_ports[winner].we_i.read());
                b_ports[b].be_o.write(m_ports[winner].be_i.read());
                b_ports[b].wdata_o.write(m_ports[winner].wdata_i.read());
                m_ports[winner].gnt_o.write(b_ports[b].gnt_i.read());
            }

            const int ow = owner[b].read();
            if (ow >= 0 && b_ports[b].rvalid_i.read()) {
                m_ports[ow].rvalid_o.write(true);
                m_ports[ow].rdata_o.write(b_ports[b].rdata_i.read());
            }
        }
    }

    // NOTE (contract): owner/rr_ptr advance unconditionally on a winner —
    // this assumes the bank side always grants (bank.hpp: gnt follows req
    // combinationally, stated in this file's header). A back-pressuring
    // subordinate (gnt_i=0 for a presented winner) would desynchronize the
    // owner register from the actual transaction; do not attach one.
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
            sensitive << m_ports[m].req_i << m_ports[m].addr_i << m_ports[m].we_i << m_ports[m].be_i
                      << m_ports[m].wdata_i;
        for (int b = 0; b < NUM_OUT; ++b)
            sensitive << b_ports[b].gnt_i << b_ports[b].rvalid_i << b_ports[b].rdata_i << rr_ptr[b]
                      << owner[b];

        SC_METHOD(seq);
        sensitive << clk_i.pos();
        dont_initialize();
    }
};

#endif
