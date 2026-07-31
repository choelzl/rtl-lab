// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// Native SystemC single-port memory bank — an OBI subordinate (doc/specs/
// obi.md) wrapping a word-addressable RAM array, exposed as `obi`
// (obi_ports.hpp). gnt_o follows req_i combinationally (no back-pressure;
// contention is resolved upstream, one request/cycle to this port). 1-cycle
// latency: a request accepted at T produces rvalid_o/rdata_o at T+1. Reads
// return mem[word]; writes apply be_i byte lanes. Array is zero-init.
//
// addr_i is bank-local (bank-select already stripped upstream): word =
// addr_i/BYTES_PER_WORD, row = word % NUM_ROW (wraps rather than faulting).
// Reset is active-low (rst_ni). Template params (from PARAMS N_ROW,
// BYTES_PER_ROW): NUM_ROW (rows/bank), BYTES_PER_ROW (bytes/OBI beat).
// -----------------------------------------------------------------------------

#ifndef BANK_HPP
#define BANK_HPP

#include <systemc.h>

#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <cstdint>
#include <vector>

template <int NUM_ROW = 1024, int BYTES_PER_ROW = 4 * 4> SC_MODULE(bank) {
    using data_t = obi_data<BYTES_PER_ROW>;

    sc_in<bool>                   clk_i;
    sc_in<bool>                   rst_ni;
    obi_subordinate_ports<data_t> obi;

    static constexpr int kDepthRows = NUM_ROW;

    static_assert(BYTES_PER_ROW >= 1, "BYTES_PER_ROW must be >= 1");

    std::vector<data_t> mem;

    // A we=1 request with be=0 is deliberately a no-op that is still
    // granted and acknowledged like any accepted request — the row is
    // untouched but rvalid fires (pinned by tb_bank T15).
    static data_t apply_be(data_t out, const data_t &new_w, uint32_t be) {
        for (int l = 0; l < BYTES_PER_ROW; ++l)
            if (be & (1u << l))
                out.range(l * 8 + 7, l * 8) = new_w.range(l * 8 + 7, l * 8);
        return out;
    }

    void step() {
        if (!rst_ni.read()) {
            obi.rvalid_o.write(false);
            obi.rdata_o.write(0);
            return;
        }

        bool   rv = false;
        data_t rd = 0;
        if (obi.req_i.read()) {
            const uint64_t row =
                (obi.addr_i.read() / BYTES_PER_ROW) % static_cast<uint64_t>(kDepthRows);
            if (obi.we_i.read()) {
                mem[row] = apply_be(mem[row], obi.wdata_i.read(), obi.be_i.read());
            } else {
                rd = mem[row];
            }
            rv = true;
        }
        obi.rvalid_o.write(rv);
        obi.rdata_o.write(rd);
    }

    void comb_gnt() {
        obi.gnt_o.write(obi.req_i.read());
    }

    SC_CTOR(bank) : mem(kDepthRows, 0) {
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();

        SC_METHOD(comb_gnt);
        sensitive << obi.req_i;
    }
};

#endif
