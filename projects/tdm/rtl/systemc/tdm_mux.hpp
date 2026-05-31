// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC config selector for the TDM mapping function. It selects one
//   of NUM_AGU per-AGU configuration values by index (sel_i) and forwards it to
//   the mapping function. It is NOT an OBI path — just a combinational mux on the
//   config carried alongside the request.
//
//   v1: the only config is the address STRIDE (bytes between consecutive words
//   of a group); the mapping computes addr[w] = base + w*stride. More config
//   fields can be added as the mapping grows.
//
// Template parameters:
//   NUM_AGU - number of config sources to select among (default 2)
// -----------------------------------------------------------------------------

#ifndef TDM_MUX_HPP
#define TDM_MUX_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_AGU = 2>
SC_MODULE(tdm_mux) {
    static_assert(NUM_AGU >= 1, "NUM_AGU must be >= 1");

    sc_in<int>       sel_i;
    sc_in<uint64_t>  stride_i[NUM_AGU];
    sc_out<uint64_t> stride_o;

    void comb() {
        const int s = sel_i.read();
        stride_o.write((s >= 0 && s < NUM_AGU) ? stride_i[s].read() : 0);
    }

    SC_CTOR(tdm_mux) {
        SC_METHOD(comb);
        sensitive << sel_i;
        for (int a = 0; a < NUM_AGU; ++a) sensitive << stride_i[a];
    }
};

#endif
