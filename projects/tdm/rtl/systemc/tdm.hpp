// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC TDM mapping function (v1, simple). Given an x-OBI group's
//   single base address (see doc/specs/x_obi.md) and a STRIDE (from the TDM
//   config mux, tdm_mux.hpp), it generates NUM_WORD word addresses
//
//       addr[w] = base + w * stride        (w = 0 .. NUM_WORD-1)
//
//   and adapts the group's per-word x-OBI signals to NUM_WORD single-word OBI
//   manager ports for a downstream interconnect (the crossbar) that decodes
//   bank/row and arbitrates. The scalar group we/be are broadcast to every port;
//   per-word req/wdata pass straight through, as do the returning per-word
//   gnt/rvalid/rdata. Purely combinational, no state. (With stride = WORD_BYTES
//   the addresses are consecutive words.)
//
//   This v1 is a placeholder: only the address generation here is the "mapping".
//   The real conflict-minimizing mapping (arbitrary per-word placement, driven
//   by the AGU's access pattern) replaces this address computation later.
//
// Template parameters:
//   NUM_WORD - words per group / manager ports out (default 8)
// -----------------------------------------------------------------------------

#ifndef TDM_HPP
#define TDM_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_WORD = 8>
SC_MODULE(tdm) {
    static_assert(NUM_WORD >= 1, "NUM_WORD must be >= 1");

    sc_in<bool>      g_req_i[NUM_WORD];
    sc_in<uint64_t>  g_addr_i;
    sc_in<bool>      g_we_i;
    sc_in<uint32_t>  g_be_i;
    sc_in<uint64_t>  g_wdata_i[NUM_WORD];
    sc_in<uint64_t>  stride_i;
    sc_out<bool>     g_gnt_o[NUM_WORD];
    sc_out<bool>     g_rvalid_o[NUM_WORD];
    sc_out<uint64_t> g_rdata_o[NUM_WORD];

    sc_out<bool>     c_req_o[NUM_WORD];
    sc_out<uint64_t> c_addr_o[NUM_WORD];
    sc_out<bool>     c_we_o[NUM_WORD];
    sc_out<uint32_t> c_be_o[NUM_WORD];
    sc_out<uint64_t> c_wdata_o[NUM_WORD];
    sc_in<bool>      c_gnt_i[NUM_WORD];
    sc_in<bool>      c_rvalid_i[NUM_WORD];
    sc_in<uint64_t>  c_rdata_i[NUM_WORD];

    void comb() {
        const uint64_t base   = g_addr_i.read();
        const uint64_t stride = stride_i.read();
        const bool     we     = g_we_i.read();
        const uint32_t be     = g_be_i.read();
        for (int w = 0; w < NUM_WORD; ++w) {
            c_req_o[w].write(g_req_i[w].read());
            c_addr_o[w].write(base + static_cast<uint64_t>(w) * stride);
            c_we_o[w].write(we);
            c_be_o[w].write(be);
            c_wdata_o[w].write(g_wdata_i[w].read());

            g_gnt_o[w].write(c_gnt_i[w].read());
            g_rvalid_o[w].write(c_rvalid_i[w].read());
            g_rdata_o[w].write(c_rdata_i[w].read());
        }
    }

    SC_CTOR(tdm) {
        SC_METHOD(comb);
        sensitive << g_addr_i << g_we_i << g_be_i << stride_i;
        for (int w = 0; w < NUM_WORD; ++w)
            sensitive << g_req_i[w] << g_wdata_i[w] << c_gnt_i[w] << c_rvalid_i[w]
                      << c_rdata_i[w];
    }
};

#endif
