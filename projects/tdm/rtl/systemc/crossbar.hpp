// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// NUM_IN x NUM_OUT OBI crossbar (doc/specs/obi.md), word-interleaved banking
// with a per-bank round-robin arbiter. m_ports[NUM_IN] (crossbar =
// subordinate) <-> b_ports[NUM_OUT] (crossbar = manager). Combinational data
// path; only rr_ptr/owner are registered per bank.
//
// SEL_LEN==0 (default, beat-interleaved): bank = (addr/BYTES_PER_ROW) %
// NUM_OUT, bank-local addr = row*BYTES_PER_ROW (routing bits stripped).
// SEL_LEN>0 (used by the 3-level crossbar backend, doc/specs/crossbar.md):
// bank = (addr>>SEL_START) & mask, full address passed through unchanged.
//
// Arbitration: round-robin winner per bank forwarded to the bank; losers
// hold req until their turn. Response routing: bank latency is 1 cycle, so
// the winning manager is latched in a per-bank owner register and steered
// the response when rvalid arrives next cycle — no transaction IDs needed.
// Banks are always-ready (gnt=req), so the arbiter winner is granted every
// cycle; reset is active-low.
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
