// -----------------------------------------------------------------------------
// Author: Simone Machetti
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
// Template parameters (set from PARAMS macros N_ROW, WORD_BYTES):
//   NUM_ROW    - rows (words) per bank (default 1024)
//   BYTES_PER_WORD - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef BANK_HPP
#define BANK_HPP

#include <systemc.h>

#include <cstdint>
#include <sstream>
#include <vector>

template <int NUM_ROW = 1024, int BYTES_PER_WORD = 4> SC_MODULE(bank) {
    sc_in<bool>      clk_i;
    sc_in<bool>      rst_ni;
    sc_in<bool>      req_i;
    sc_in<uint64_t>  addr_i;
    sc_in<bool>      we_i;
    sc_in<uint32_t>  be_i;
    sc_in<uint64_t>  wdata_i;
    sc_out<bool>     gnt_o;
    sc_out<bool>     rvalid_o;
    sc_out<uint64_t> rdata_o;

    static constexpr int kDepthWords = NUM_ROW;

    static_assert(BYTES_PER_WORD >= 1, "BYTES_PER_WORD must be >= 1");

    std::vector<uint64_t> mem;

    static uint64_t apply_be(uint64_t old_w, uint64_t new_w, uint32_t be) {
        uint64_t out = old_w;
        for (int l = 0; l < BYTES_PER_WORD; ++l) {
            if (be & (1u << l)) {
                const uint64_t m = static_cast<uint64_t>(0xFF) << (8 * l);
                out              = (out & ~m) | (new_w & m);
            }
        }
        return out;
    }

    void step() {
        if (!rst_ni.read()) {
            rvalid_o.write(false);
            rdata_o.write(0);
            return;
        }

        bool     rv = false;
        uint64_t rd = 0;
        if (req_i.read()) {
            const uint64_t word = addr_i.read() / BYTES_PER_WORD;
            if (word >= static_cast<uint64_t>(kDepthWords)) {
                std::ostringstream os;
                os << "OBI access out of range: bank-local word " << word << " >= capacity "
                   << kDepthWords;
                SC_REPORT_FATAL(name(), os.str().c_str());
            }
            if (we_i.read()) {
                mem[word] = apply_be(mem[word], wdata_i.read(), be_i.read());
            } else {
                rd = mem[word];
            }
            rv = true;
        }
        rvalid_o.write(rv);
        rdata_o.write(rd);
    }

    void comb_gnt() {
        gnt_o.write(req_i.read());
    }

    SC_CTOR(bank) : mem(kDepthWords, 0) {
        SC_METHOD(step);
        sensitive << clk_i.pos();
        dont_initialize();

        SC_METHOD(comb_gnt);
        sensitive << req_i;
    }
};

#endif
