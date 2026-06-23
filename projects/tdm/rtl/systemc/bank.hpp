// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// Description:
//   Native SystemC single-port memory bank — an OBI subordinate wrapping a
//   word-addressable RAM array. It implements the simplified single-channel OBI
//   protocol (see doc/specs/obi.md):
//
//     request  (manager -> bank) : req_i, addr_i, we_i, be_i, wdata_i
//     response (bank -> manager) : gnt_o, rvalid_o, rdata_o
//
//   Behaviour (all sampled / driven on the rising edge of clk_i):
//     - gnt_o follows req_i combinationally: the bank accepts whenever a
//       request is present and never back-pressures. Any contention for the
//       bank is resolved by the upstream interconnect, which presents at most
//       one request to this single port per cycle.
//     - 1-cycle access latency: a request accepted at cycle T (req_i & gnt_o)
//       produces its response (rvalid_o, and rdata_o on reads) at cycle T+1.
//       One request is accepted per cycle (no outstanding/pipelined depth > 1).
//     - Reads return mem[word]; writes update the byte lanes selected by be_i.
//     - The array is zero-initialised at construction.
//
//   Addressing: addr_i is a BANK-LOCAL byte address — the bank-select field has
//   already been stripped upstream, so word = addr_i / BYTES_PER_WORD indexes
//   directly into this bank's array (capacity NUM_ROW words, one word per row).
//   An access outside that range is a fatal error (SC_REPORT_FATAL).
//
//   Reset is active-low (rst_ni), matching OBI reset_n directly.
//
// Template parameters (set from PARAMS macros N_ROW, BYTES_PER_ROW):
//   NUM_ROW      - rows per bank (default 1024)
//   BYTES_PER_ROW - bytes per OBI data beat = WORDS_PER_ROW * BYTES_PER_WORD (default 16)
// -----------------------------------------------------------------------------

#ifndef BANK_HPP
#define BANK_HPP

#include <systemc.h>

#include "obi_data.hpp"
#include <cstdint>
#include <sstream>
#include <vector>

template <int NUM_ROW = 1024, int BYTES_PER_ROW = 4 * 4> SC_MODULE(bank) {
    using data_t = obi_data<BYTES_PER_ROW>;

    sc_in<bool>     clk_i;
    sc_in<bool>     rst_ni;
    sc_in<bool>     req_i;
    sc_in<uint64_t> addr_i;
    sc_in<bool>     we_i;
    sc_in<uint32_t> be_i;
    sc_in<data_t>   wdata_i;
    sc_out<bool>    gnt_o;
    sc_out<bool>    rvalid_o;
    sc_out<data_t>  rdata_o;

    static constexpr int kDepthRows = NUM_ROW;

    static_assert(BYTES_PER_ROW >= 1, "BYTES_PER_ROW must be >= 1");

    std::vector<data_t> mem;

    static data_t apply_be(data_t out, const data_t &new_w, uint32_t be) {
        for (int l = 0; l < BYTES_PER_ROW; ++l)
            if (be & (1u << l))
                out.range(l * 8 + 7, l * 8) = new_w.range(l * 8 + 7, l * 8);
        return out;
    }

    void step() {
        if (!rst_ni.read()) {
            rvalid_o.write(false);
            rdata_o.write(0);
            return;
        }

        bool   rv = false;
        data_t rd = 0;
        if (req_i.read()) {
            const uint64_t row = addr_i.read() / BYTES_PER_ROW;
            if (row >= static_cast<uint64_t>(kDepthRows)) {
                std::ostringstream os;
                os << "OBI access out of range: bank-local row " << row << " >= capacity "
                   << kDepthRows;
                SC_REPORT_FATAL(name(), os.str().c_str());
            }
            if (we_i.read()) {
                mem[row] = apply_be(mem[row], wdata_i.read(), be_i.read());
            } else {
                rd = mem[row];
            }
            rv = true;
        }
        rvalid_o.write(rv);
        rdata_o.write(rd);
    }

    void comb_gnt() {
        gnt_o.write(req_i.read());
    }

    SC_CTOR(bank) : mem(kDepthRows, 0) {
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();

        SC_METHOD(comb_gnt);
        sensitive << req_i;
    }
};

#endif
