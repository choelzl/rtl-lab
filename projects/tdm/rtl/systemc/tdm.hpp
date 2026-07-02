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
//   The placement scheme (parameter meaning, the get_k boundaries, the address
//   split into con/str/l, and the 5-bit bank-id XOR matrix) is specified in
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

#include "obi_data.hpp"
#include "obi_ports.hpp"
#include <algorithm>
#include <cstdint>
#include <utility>

enum class tdm_stor_mode {
    Loop_Row_Col   = 0,
    Loop_Col_Row   = 1,
    Row_Col_Loop   = 2,
    Col_Row_Loop   = 3,
    Row_Loop_Col   = 4,
    Col_Loop_Row   = 5,
    Loop_2x2_H     = 6,
    Loop_2x2_V     = 7,
    Loop_4x4_H     = 8,
    Loop_4x4_V     = 9,
    Loop_Row       = 10,
    Row_Loop       = 11,
    Loop_Row_Space = 12,
    Loop_2i        = 13,
    Loop_3i        = 14,
    Loop_4i        = 15
};

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

    static uint32_t ilog2(uint64_t v) {
        uint32_t r = 0;
        while (v > 1) {
            v >>= 1;
            ++r;
        }
        return r;
    }

    static uint32_t tzeros(uint64_t v) {
        if (v == 0)
            return 0;
        uint32_t r = 0;
        while (((v >> r) & 1ull) == 0)
            ++r;
        return r;
    }

    static std::pair<uint32_t, uint32_t> get_k(tdm_stor_mode mode, uint32_t e, uint32_t tzR,
                                               uint32_t tzC, uint32_t tzL) {
        const std::pair<uint32_t, uint32_t> k = get_k_raw(mode, e, tzR, tzC, tzL);
#ifdef TDM_GETK_GUARD
        // Degenerate-split guard (experimental; doc/report Appendix A.8): when the
        // mode's LEADING dimension has no trailing zeros (e.g. C = 1 under
        // Loop_Row), k1 collapses onto e, the con field is zero-width, every
        // con term of the bank-id XOR matrix drops out, and some window
        // layouts fold pairwise onto half the banks. Borrow up to two bits
        // from the bottom of str so con is never empty — bank_id stays a
        // bijection per routing field, row_id untouched. NOTE: the guard is
        // necessarily pattern-blind (get_k sees only the geometry, not the
        // window layout); measured statically it repairs the napa=4
        // offender but can introduce collisions on other layouts sharing
        // the same split — hence opt-in, see the report for the evaluation.
        using M = tdm_stor_mode;
        const uint32_t lead =
            (mode == M::Loop_Row_Col || mode == M::Loop_Row || mode == M::Row_Loop_Col ||
             mode == M::Loop_2x2_H || mode == M::Loop_4x4_H)
                ? tzC
            : (mode == M::Row_Col_Loop || mode == M::Row_Loop || mode == M::Col_Row_Loop) ? tzL
                                                                                          : tzR;
        if (lead == 0 && k.second > e)
            return {std::min(e + 2, k.second), k.second};
#endif
        return k;
    }

    static std::pair<uint32_t, uint32_t> get_k_raw(tdm_stor_mode mode, uint32_t e, uint32_t tzR,
                                                   uint32_t tzC, uint32_t tzL) {
        using M = tdm_stor_mode;
        switch (mode) {
        case M::Loop_Row_Col:
        case M::Loop_Row:
            return {std::max(tzC, e), std::max(tzR + tzC, e)};
        case M::Loop_Col_Row:
        case M::Loop_Row_Space:
            return {std::max(tzR, e), std::max(tzC + tzR, e)};
        case M::Row_Col_Loop:
        case M::Row_Loop:
            return {std::max(tzL, e), std::max(tzL + tzC, e)};
        case M::Col_Row_Loop:
            return {std::max(tzL, e), std::max(tzR + tzL, e)};
        case M::Row_Loop_Col:
            return {std::max(tzC, e), std::max(tzL + tzC, e)};
        case M::Col_Loop_Row:
            return {std::max(tzR, e), std::max(tzL + tzR, e)};
        case M::Loop_2x2_H:
            return {std::max(tzC + 1, e), std::max(tzR + tzC, e)};
        case M::Loop_2x2_V:
        case M::Loop_2i:
            return {std::max(tzR + 1, e), std::max(tzC + tzR, e)};
        case M::Loop_4x4_H:
            return {std::max(tzC + 2, e), std::max(tzR + tzC, e)};
        case M::Loop_4x4_V:
        case M::Loop_4i:
            return {std::max(tzR + 2, e), std::max(tzC + tzR, e)};
        default:
            SC_REPORT_ERROR("tdm", "unsupported store_mode");
            return {e, e};
        }
    }

    // Pure function of its arguments (all mapping parameters are passed in,
    // no module state) — static so callers that need the placement without a
    // live tdm instance (e.g. the crossbar-build testbench computing what the
    // TDM map WOULD do with an address) can use it directly.
    static void map_one(uint64_t addr, uint64_t nb, uint64_t bw, uint64_t R, uint64_t C, uint64_t L,
                        tdm_stor_mode mode, uint64_t &bank_id, uint64_t &row_id) {
        const uint32_t                      b  = ilog2(nb);
        const uint32_t                      e  = ilog2(bw);
        const std::pair<uint32_t, uint32_t> k  = get_k(mode, e, tzeros(R), tzeros(C), tzeros(L));
        const uint32_t                      k1 = k.first;
        const uint32_t                      k2 = k.second;

        const uint64_t bmask = (b >= 64) ? ~0ull : ((1ull << b) - 1);
        const uint64_t con   = (addr >> e) & ((1ull << (k1 - e)) - 1) & bmask;
        const uint64_t str   = (addr >> k1) & ((1ull << (k2 - k1)) - 1) & bmask;
        const uint64_t l     = (addr >> k2) & bmask;

        auto bit = [](uint64_t v, uint32_t i) -> uint64_t { return (v >> i) & 1ull; };

        bank_id = ((bit(str, 1) ^ bit(con, 2) ^ bit(l, 1) ^ bit(l, 2)) << 0) |
                  ((bit(str, 2) ^ bit(con, 1) ^ bit(l, 1)) << 1) |
                  ((bit(str, 0) ^ bit(str, 4) ^ bit(con, 0) ^ bit(con, 1) ^ bit(l, 4)) << 2) |
                  ((bit(str, 0) ^ bit(con, 4) ^ bit(l, 0)) << 3) |
                  ((bit(str, 1) ^ bit(str, 3) ^ bit(con, 0) ^ bit(con, 3) ^ bit(l, 3)) << 4);

        row_id = addr >> (e + b);
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
