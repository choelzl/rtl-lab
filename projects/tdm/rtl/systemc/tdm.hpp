// -----------------------------------------------------------------------------
// Author: Simone Machetti
//
// Description:
//   Native SystemC TDM mapping function. Given an x-OBI group's single base
//   address (see doc/specs/x_obi.md) and a set of kernel-wide mapping parameters
//   (num_banks, bank_width, R, C, L, store_mode), it places each of the NUM_WORD
//   words of the group (logical address base + w*BYTES_PER_WORD) into a
//   (bank_id, row_id) location with an XOR-skewed banking scheme, then emits
//   NUM_WORD single-word OBI requests for a downstream word-interleaved
//   interconnect.
//
//   The placement scheme (parameter meaning, the get_k boundaries, the address
//   split into con/str/l, and the 5-bit bank-id XOR matrix) is specified in
//   doc/specs/map_func.md. The emitted OBI byte address re-encodes the placement
//   as (row_id * NUM_BANK + bank_id) * BYTES_PER_WORD, so a downstream decode of
//   bank = word % NUM_BANK and row = word / NUM_BANK recovers exactly that
//   (bank_id, row_id); a bank collision (>=2 words sharing a bank) is left for
//   the interconnect's per-bank arbiter to serialize.
//
//   The scalar group we/be are broadcast to every emitted port; per-word
//   req/wdata pass straight through, as do the returning per-word gnt/rvalid/
//   rdata. Purely combinational, no state. bank_id is a 5-bit value (0..31); if
//   it reaches NUM_BANK the build is too small for the mapping (NUM_BANK must be
//   >= 32) and the module reports a fatal error.
//
// Template parameters (NUM_BANK, BYTES_PER_WORD from PARAMS N_BANK, WORD_BYTES):
//   NUM_WORD       - words per group / manager ports out (default 8)
//   NUM_BANK       - number of banks the re-encoded address targets (default 32)
//   BYTES_PER_WORD - bytes per word / OBI data beat (default 4)
// -----------------------------------------------------------------------------

#ifndef TDM_HPP
#define TDM_HPP

#include <systemc.h>

#include <algorithm>
#include <cstdint>
#include <utility>

enum class tdm_stor_mode {
    Loop_Row_Col,
    Loop_Row,
    Loop_Col_Row,
    Loop_Row_Space,
    Row_Col_Loop,
    Row_Loop,
    Col_Row_Loop,
    Row_Loop_Col,
    Col_Loop_Row,
    Loop_2x2_H,
    Loop_2x2_V,
    Loop_2i,
    Loop_4x4_H,
    Loop_4x4_V,
    Loop_4i
};

template <int NUM_WORD = 8, int NUM_BANK = 32, int BYTES_PER_WORD = 4> SC_MODULE(tdm) {
    static_assert(NUM_WORD >= 1, "NUM_WORD must be >= 1");

    sc_in<bool>      g_req_i[NUM_WORD];
    sc_in<uint64_t>  g_addr_i;
    sc_in<bool>      g_we_i;
    sc_in<uint32_t>  g_be_i;
    sc_in<uint64_t>  g_wdata_i[NUM_WORD];
    sc_out<bool>     g_gnt_o[NUM_WORD];
    sc_out<bool>     g_rvalid_o[NUM_WORD];
    sc_out<uint64_t> g_rdata_o[NUM_WORD];

    sc_in<uint64_t> num_banks_i;
    sc_in<uint64_t> bank_width_i;
    sc_in<uint64_t> r_i;
    sc_in<uint64_t> c_i;
    sc_in<uint64_t> l_i;
    sc_in<uint64_t> store_mode_i;

    sc_out<bool>     c_req_o[NUM_WORD];
    sc_out<uint64_t> c_addr_o[NUM_WORD];
    sc_out<bool>     c_we_o[NUM_WORD];
    sc_out<uint32_t> c_be_o[NUM_WORD];
    sc_out<uint64_t> c_wdata_o[NUM_WORD];
    sc_in<bool>      c_gnt_i[NUM_WORD];
    sc_in<bool>      c_rvalid_i[NUM_WORD];
    sc_in<uint64_t>  c_rdata_i[NUM_WORD];

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

    void map_one(uint64_t addr, uint64_t nb, uint64_t bw, uint64_t R, uint64_t C, uint64_t L,
                 tdm_stor_mode mode, uint64_t &bank_id, uint64_t &row_id) const {
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
        const uint64_t      base = g_addr_i.read();
        const bool          we   = g_we_i.read();
        const uint32_t      be   = g_be_i.read();
        const uint64_t      nb   = num_banks_i.read();
        const uint64_t      bw   = bank_width_i.read();
        const uint64_t      R    = r_i.read();
        const uint64_t      C    = c_i.read();
        const uint64_t      L    = l_i.read();
        const tdm_stor_mode mode = static_cast<tdm_stor_mode>(store_mode_i.read());

        for (int w = 0; w < NUM_WORD; ++w) {
            uint64_t bank_id = 0, row_id = 0;
            map_one(base + static_cast<uint64_t>(w) * BYTES_PER_WORD, nb, bw, R, C, L, mode,
                    bank_id, row_id);
            if (bank_id >= static_cast<uint64_t>(NUM_BANK))
                SC_REPORT_FATAL("tdm", "bank_id >= NUM_BANK (build N_BANK too small; "
                                       "the mapping needs N_BANK >= 32)");
            const uint64_t word_index = row_id * static_cast<uint64_t>(NUM_BANK) + bank_id;

            c_req_o[w].write(g_req_i[w].read());
            c_addr_o[w].write(word_index * static_cast<uint64_t>(BYTES_PER_WORD));
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
        sensitive << g_addr_i << g_we_i << g_be_i << num_banks_i << bank_width_i << r_i << c_i
                  << l_i << store_mode_i;
        for (int w = 0; w < NUM_WORD; ++w)
            sensitive << g_req_i[w] << g_wdata_i[w] << c_gnt_i[w] << c_rvalid_i[w] << c_rdata_i[w];
    }
};

#endif
