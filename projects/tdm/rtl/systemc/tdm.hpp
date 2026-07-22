// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// Description:
//   Native SystemC TDM mapping function. Given a request group's base
//   address (a group = NUM_WORD parallel OBI lanes, one mapped word each,
//   sharing scalar we/be — see doc/specs/obi.md) and kernel-wide mapping parameters
//   (num_banks, bank_width, R, C, L, store_mode), it places each of the NUM_WORD
//   words of the group (logical address base + w*BYTES_PER_WORD) into a
//   (bank_id, row_id) location with an XOR-skewed banking scheme, then emits
//   one OBI request per lane for a downstream beat-interleaved interconnect
//   (the word exists only in the placement math — the emitted requests are
//   ordinary full-beat OBI, routed by the reused crossbar as bank = beat %
//   NUM_BANK).
//
//   The placement scheme itself (get_k, map_one, the con/str/l split, and the
//   5-bit bank-id XOR matrix) lives in map_func.hpp, shared with other
//   backends that want the same placement (e.g. top_crossbar.hpp's
//   XBAR_HASH_DYNAMIC experiment). Parameter meaning, the get_k boundaries,
//   the address split into con/str/l, and the XOR matrix are specified in
//   doc/specs/map_func.md. The emitted OBI byte address re-encodes the placement
//   as (row_id * NUM_BANK + bank_id) * BYTES_PER_ROW, so a downstream decode of
//   bank = beat % NUM_BANK and row = beat / NUM_BANK recovers exactly that
//   (bank_id, row_id); a bank collision (>=2 words sharing a bank) is left for
//   the interconnect's per-bank arbiter to serialize.
//
//   The scalar group we/be are broadcast to every emitted port; per-word
//   req/wdata pass straight through, as do the returning per-word gnt/rvalid/
//   rdata. Purely combinational, no state. bank_id is a 5-bit value (0..31); if
//   it reaches NUM_BANK the build is too small for the mapping (NUM_BANK must be
//   >= 32) and the module reports a fatal error.
//
// Template parameters (NUM_BANK, BYTES_PER_ROW from PARAMS N_BANK, BYTES_PER_ROW):
//   NUM_WORD      - words per group / manager ports out (default 8)
//   NUM_BANK      - number of banks the re-encoded address targets (default 32)
//   BYTES_PER_ROW - bytes per OBI data beat = WORDS_PER_ROW*BYTES_PER_WORD (default 16)
// -----------------------------------------------------------------------------

#ifndef TDM_HPP
#define TDM_HPP

#include <systemc.h>

#include "map_func.hpp"
#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <algorithm>
#include <cstdint>
#include <utility>

template <int NUM_WORD = 8, int NUM_BANK = 32, int BYTES_PER_ROW = 4 * 4> SC_MODULE(tdm) {
    static_assert(NUM_WORD >= 1, "NUM_WORD must be >= 1");
    using data_t = obi_data<BYTES_PER_ROW>;

    // g[w] : group-facing OBI (subordinate — receives the request to map)
    // c[w] : bank-facing OBI (manager — issues the mapped request)
    obi_subordinate_ports<data_t> g[NUM_WORD];
    obi_manager_ports<data_t>     c[NUM_WORD];

    sc_in<uint64_t> num_banks_i;
    sc_in<uint64_t> bank_width_i;
    sc_in<uint64_t> r_i;
    sc_in<uint64_t> c_i;
    sc_in<uint64_t> l_i;
    sc_in<uint64_t> store_mode_i;

    // Thin wrapper over map_func::map_one (rtl/systemc/map_func.hpp) — kept
    // static so callers that need the placement without a live tdm instance
    // (e.g. the crossbar-build testbench computing what the TDM map WOULD do
    // with an address) can use it directly.
    static void map_one(uint64_t addr, uint64_t nb, uint64_t bw, uint64_t R, uint64_t C, uint64_t L,
                        tdm_stor_mode mode, uint64_t &bank_id, uint64_t &row_id) {
        map_func::map_one(addr, nb, bw, R, C, L, mode, bank_id, row_id);
    }

    void comb() {
        const uint64_t      nb   = num_banks_i.read();
        const uint64_t      bw   = bank_width_i.read();
        const uint64_t      R    = r_i.read();
        const uint64_t      C    = c_i.read();
        const uint64_t      L    = l_i.read();
        const tdm_stor_mode mode = static_cast<tdm_stor_mode>(store_mode_i.read());

        for (int w = 0; w < NUM_WORD; ++w) {
            const bool     req  = g[w].req_i.read();
            const uint64_t addr = g[w].addr_i.read();

            if (addr == 0) {
                // addr=0 is the NOP sentinel: suppress the bank request and
                // grant immediately. Do NOT touch g[w].rvalid_o — fall through
                // to always pass c[w].rvalid_i so a concurrent response for
                // another slot (arb_rsp_sel) is never clobbered.
                c[w].req_o.write(false);
                c[w].addr_o.write(0);
                c[w].we_o.write(false);
                c[w].be_o.write(0);
                c[w].wdata_o.write(data_t{});
                g[w].gnt_o.write(req);
            } else {
                uint64_t bank_id = 0, row_id = 0;
                map_one(addr, nb, bw, R, C, L, mode, bank_id, row_id);
                if (bank_id >= static_cast<uint64_t>(NUM_BANK))
                    SC_REPORT_FATAL("tdm", "bank_id >= NUM_BANK (build N_BANK too small; "
                                           "the mapping needs N_BANK >= 32)");
                const uint64_t word_index = row_id * static_cast<uint64_t>(NUM_BANK) + bank_id;

                c[w].req_o.write(req);
                c[w].addr_o.write(word_index * static_cast<uint64_t>(BYTES_PER_ROW));
                c[w].we_o.write(g[w].we_i.read());
                c[w].be_o.write(g[w].be_i.read());
                c[w].wdata_o.write(g[w].wdata_i.read());
                g[w].gnt_o.write(c[w].gnt_i.read());
            }

            // Always pass the bank rvalid through — this must happen regardless
            // of the addr=0 path so responses in-flight for the previous TDM
            // slot (arb_rsp_sel) are not lost.
            g[w].rvalid_o.write(c[w].rvalid_i.read());
            g[w].rdata_o.write(c[w].rdata_i.read());
        }
    }

    SC_CTOR(tdm) {
        SC_METHOD(comb);
        sensitive << num_banks_i << bank_width_i << r_i << c_i << l_i << store_mode_i;
        for (int w = 0; w < NUM_WORD; ++w)
            sensitive << g[w].addr_i << g[w].req_i << g[w].we_i << g[w].be_i << g[w].wdata_i
                      << c[w].gnt_i << c[w].rvalid_i << c[w].rdata_i;
    }
};

#endif
