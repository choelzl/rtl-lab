// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC config selector for the TDM mapping function. It selects one
//   of NUM_AGU per-source configuration sets by index (sel_i) and forwards it to
//   the mapping function. It is NOT an OBI path — just a combinational mux on the
//   config carried alongside the request.
//
//   The config set is the kernel-wide mapping parameters that decide the access
//   pattern: num_banks, bank_width, R, C, L, store_mode. Each source presents its
//   own set; sel_i picks the active one (an out-of-range index yields zeros).
//
// Template parameters:
//   NUM_AGU - number of config sources to select among (default 2)
// -----------------------------------------------------------------------------

#ifndef TDM_MUX_HPP
#define TDM_MUX_HPP

#include <systemc.h>

#include <cstdint>

template <int NUM_AGU = 2> SC_MODULE(tdm_mux) {
    static_assert(NUM_AGU >= 1, "NUM_AGU must be >= 1");

    sc_in<int>      sel_i;
    sc_in<uint64_t> num_banks_i[NUM_AGU];
    sc_in<uint64_t> bank_width_i[NUM_AGU];
    sc_in<uint64_t> r_i[NUM_AGU];
    sc_in<uint64_t> c_i[NUM_AGU];
    sc_in<uint64_t> l_i[NUM_AGU];
    sc_in<uint64_t> store_mode_i[NUM_AGU];

    sc_out<uint64_t> num_banks_o;
    sc_out<uint64_t> bank_width_o;
    sc_out<uint64_t> r_o;
    sc_out<uint64_t> c_o;
    sc_out<uint64_t> l_o;
    sc_out<uint64_t> store_mode_o;

    void comb() {
        const int  s  = sel_i.read();
        const bool ok = (s >= 0 && s < NUM_AGU);
        num_banks_o.write(ok ? num_banks_i[s].read() : 0);
        bank_width_o.write(ok ? bank_width_i[s].read() : 0);
        r_o.write(ok ? r_i[s].read() : 0);
        c_o.write(ok ? c_i[s].read() : 0);
        l_o.write(ok ? l_i[s].read() : 0);
        store_mode_o.write(ok ? store_mode_i[s].read() : 0);
    }

    SC_CTOR(tdm_mux) {
        SC_METHOD(comb);
        sensitive << sel_i;
        for (int a = 0; a < NUM_AGU; ++a)
            sensitive << num_banks_i[a] << bank_width_i[a] << r_i[a] << c_i[a] << l_i[a]
                      << store_mode_i[a];
    }
};

#endif
