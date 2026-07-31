// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// Native SystemC TDM mapping function. Given a request group's base address
// (NUM_WORD parallel OBI lanes sharing scalar we/be, doc/specs/obi.md) and
// kernel mapping params (num_banks, bank_width, R, C, L, store_mode), places
// each word (base + w*BYTES_PER_WORD) into a (bank_id, row_id) via map_func.hpp's
// XOR-skewed scheme (spec: doc/specs/map_func.md), then emits one ordinary
// full-beat OBI request per lane, re-encoded as (row_id*NUM_BANK+bank_id)*
// BYTES_PER_ROW so a downstream bank=beat%NUM_BANK decode recovers it. Bank
// collisions across words are left to the interconnect's per-bank arbiter.
//
// Purely combinational, no state. bank_id is 5-bit (0..31); NUM_BANK must be
// >= 32 or the module reports fatal. Template params: NUM_WORD (words per
// group), NUM_BANK, BYTES_PER_ROW (both from PARAMS N_BANK/BYTES_PER_ROW).
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

    // Debug/trace outputs (not consumed by any downstream module — for
    // waveform inspection and conflict detection only): the placement this
    // cycle's group computed, and whether it collided with another real
    // lane's bank_id in the same group. addr==0 (NOP) lanes report 0/0/false.
    sc_signal<uint64_t> bank_id_o[NUM_WORD];
    sc_signal<uint64_t> row_id_o[NUM_WORD];
    sc_signal<bool>     conflict_o[NUM_WORD];

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

        // Pass 1: compute this cycle's placement for every lane so pass 2 can
        // compare lanes against each other before anything is driven.
        bool     real[NUM_WORD];
        uint64_t bank_id[NUM_WORD], row_id[NUM_WORD];
        for (int w = 0; w < NUM_WORD; ++w) {
            const uint64_t addr = g[w].addr_i.read();
            real[w]             = g[w].req_i.read() && addr != 0;
            if (!real[w]) {
                bank_id[w] = 0;
                row_id[w]  = 0;
                continue;
            }
            map_one(addr, nb, bw, R, C, L, mode, bank_id[w], row_id[w]);
            if (bank_id[w] >= static_cast<uint64_t>(NUM_BANK))
                SC_REPORT_FATAL("tdm", "bank_id >= NUM_BANK (build N_BANK too small; "
                                       "the mapping needs N_BANK >= 32)");
        }

        // Pass 2: a lane conflicts iff it's real and shares its bank_id with
        // at least one other real lane this cycle (a genuine collision the
        // downstream per-bank arbiter must serialize).
        for (int w = 0; w < NUM_WORD; ++w) {
            bool conflict = false;
            if (real[w])
                for (int w2 = 0; w2 < NUM_WORD && !conflict; ++w2)
                    conflict = w2 != w && real[w2] && bank_id[w2] == bank_id[w];
            bank_id_o[w].write(bank_id[w]);
            row_id_o[w].write(row_id[w]);
            conflict_o[w].write(conflict);
        }

        for (int w = 0; w < NUM_WORD; ++w) {
            const bool req = g[w].req_i.read();

            if (!real[w]) {
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
                const uint64_t word_index =
                    row_id[w] * static_cast<uint64_t>(NUM_BANK) + bank_id[w];

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
