// -----------------------------------------------------------------------------
// Bank-hash / address-scrambling formulas for top_crossbar.hpp's L1/L2
// routing fields, split out so that file stays connection-only. Every
// function here is a pure function of an address (plus, for the dynamic
// schemes, R/C/L/store_mode) — none touch a port, signal, or submodule.
//
// Template params mirror top_crossbar.hpp's derived constants: ROUTE_LSB
// (routing field LSB), LOG_REQ (L1-select width), LOG_BANK_GRP (L2-select
// width), NUM_BANK/BYTES_PER_ROW (XBAR_HASH_DYNAMIC only).
// -----------------------------------------------------------------------------

#ifndef ADDR_HASH_HPP
#define ADDR_HASH_HPP

#include <cstdint>
#include <systemc.h>

#include "map_func.hpp"

template <int ROUTE_LSB, int LOG_REQ, int LOG_BANK_GRP, int NUM_BANK, int BYTES_PER_ROW>
struct addr_hash_ops {
    static uint64_t addr_hash(uint64_t a) {
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        const uint64_t sum = (hi + mid) & 0x7;
        uint64_t       r   = (a & ~(static_cast<uint64_t>(0x7) << 6)) | (sum << 6);
#if defined(XBAR_HASH_L1_V2)
        // 4-term additive fold of the non-overlapping windows addr[5:4]/
        // [7:6]/[9:8]/[11:10] — repairs strided R/C column-walks
        // XBAR_HASH_L1's single term misses (doc/report Appendix A.8).
        // Tiles bits[4:11] with no gap: a stride that only ever touches
        // bit 10 (the old window boundary at addr[12:11] skipped it) was a
        // blind spot the fold couldn't see at all. Per-address only (no
        // cross-lane state: bank.hpp needs same address -> same field).
        uint64_t l1 = (a >> 4) & 0x3;
        l1          = (l1 + ((a >> 6) & 0x3)) & 0x3;
        l1          = (l1 + ((a >> 8) & 0x3)) & 0x3;
        l1          = (l1 + ((a >> 10) & 0x3)) & 0x3;
        r           = (r & ~(static_cast<uint64_t>(0x3) << 4)) | (l1 << 4);
#elif defined(XBAR_HASH_L1)
        // Single-term fold: addr[5:4] with addr[11:10].
        const uint64_t l1 = ((a >> 4) + (a >> 10)) & 0x3;
        r                 = (r & ~(static_cast<uint64_t>(0x3) << 4)) | (l1 << 4);
#endif
        return r;
    }

    // Inverse of addr_hash()'s unconditional L2 fold only. Used solely as a
    // sanity check (XBAR_HASH_L2_COMPOSE): addr_hash(addr_hash_inv(x)) == x
    // is a mathematical identity — see tb_addr_hash_roundtrip.cpp.
    static uint64_t addr_hash_inv(uint64_t h) {
        const uint64_t hi  = (h >> 9) & 0x7;
        const uint64_t sum = (h >> 6) & 0x7;
        const uint64_t mid = (sum - hi) & 0x7; // unsigned wraparound == mod 8
        return (h & ~(static_cast<uint64_t>(0x7) << 6)) | (mid << 6);
    }

#if defined(XBAR_HASH_L1_V2)
    // Repair for the residual XBAR_HASH_L1_V2's fold handles poorly: a
    // degenerate "vector" axis (R==1 or C==1), split by napa below.
    static bool is_vector_geometry(uint64_t R, uint64_t C) {
        return R == 1 || C == 1;
    }

    // napa==1: applies addr_hash()'s own L2 fold (hi+mid into bits[8:6]) —
    // a vector task is otherwise the only path that left L2 completely raw,
    // which is a real gap: a short/degenerate axis often leaves addr[8:6]
    // literally constant across every lane, guaranteeing an L2 collision.
    // L1 is then XOR-folded (not additive: vector strides can vary across
    // two bit positions with equal weight, which addition can't distinguish
    // once they land in the same output bit) over the same non-overlapping
    // windows addr[5:4]/[7:6]/[9:8]/[11:10] as the non-vector fold above.
    static uint64_t vector_axis_fold_addr(uint64_t a) {
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        uint64_t       r   = (a & ~(static_cast<uint64_t>(0x7) << 6)) | (((hi + mid) & 0x7) << 6);

        uint64_t v = 0;
        for (int s : {4, 6, 8, 10})
            v ^= (a >> s) & 0x3;
        const uint64_t field_mask = (1ull << LOG_REQ) - 1;
        return (r & ~(field_mask << ROUTE_LSB)) | (v << ROUTE_LSB);
    }

    // napa>1: folds the FULL 5-bit L1+L2 field with addr[11:7]/[16:12] — a
    // short vector stride often leaves L2 constant, so L1 alone can't avoid
    // a pigeonhole collision across many simultaneous requesters. The
    // overlap with the base window (bits 7-8) was tried as a "clean,
    // non-overlapping" tiling instead (addr[13:9]/[18:14]) and measured
    // WORSE on real traffic (12.5%->17.8% conflict rate, ragu_a-isolation
    // sweep): these vector tasks are confined to small buffers, so the
    // wider windows mostly read constant/unused high address bits instead
    // of the real entropy the overlapping window happens to capture.
    static uint64_t vector_multiport_addr(uint64_t a) {
        uint64_t v = (a >> ROUTE_LSB) & 0x1F;
        for (int s : {7, 12})
            v = (v + ((a >> s) & 0x1F)) & 0x1F;
        const uint64_t field_mask = (1ull << (LOG_REQ + LOG_BANK_GRP)) - 1;
        return (a & ~(field_mask << ROUTE_LSB)) | (v << ROUTE_LSB);
    }
#endif

#if defined(XBAR_HASH_DYNAMIC) || defined(XBAR_HASH16)
    // Picks the L1+L2 bank field from the task's R/C/L/store_mode via
    // map_func.hpp's TDM placement, instead of addr_hash()'s fixed bit-mix.
    // L3 and everything above stay raw address bits.
    static uint64_t addr_hash_dynamic(uint64_t a, uint64_t R, uint64_t C, uint64_t L,
                                      uint64_t store_mode) {
        uint64_t bank_id = 0, row_id = 0;
        map_func::map_one(a, static_cast<uint64_t>(NUM_BANK), static_cast<uint64_t>(BYTES_PER_ROW), R,
                          C, L, static_cast<tdm_stor_mode>(store_mode), bank_id, row_id);
        const uint64_t field_mask = (1ull << (LOG_REQ + LOG_BANK_GRP)) - 1;
        return (a & ~(field_mask << ROUTE_LSB)) | (bank_id << ROUTE_LSB);
    }
#endif

#if defined(XBAR_HASH16) || defined(XBAR_HASH32)
    // Stride-XOR construction shared by XBAR_HASH16 (4-bit id) and
    // XBAR_HASH32 (5-bit id): both derive stride coords s2/s1/s0 from
    // R/C/L/store_mode, word-granular (addr_hash16/32 convert first).
    static void hash16_get_strides(tdm_stor_mode mode, uint64_t R, uint64_t C, uint64_t L,
                                   uint64_t bank_width, uint64_t &str2, uint64_t &str1,
                                   uint64_t &str0) {
        str0 = bank_width;
        switch (mode) {
        case tdm_stor_mode::Loop_Row_Col: str2 = R * C; str1 = C; return;
        case tdm_stor_mode::Loop_Col_Row: str2 = R * C; str1 = R; return;
        case tdm_stor_mode::Row_Col_Loop: str2 = C * L; str1 = L; return;
        case tdm_stor_mode::Col_Row_Loop: str2 = R * L; str1 = L; return;
        case tdm_stor_mode::Row_Loop_Col: str2 = L * C; str1 = C; return;
        case tdm_stor_mode::Col_Loop_Row: str2 = L * R; str1 = R; return;
        case tdm_stor_mode::Loop_2x2_H:   str2 = R * C; str1 = C * 2; return;
        case tdm_stor_mode::Loop_2x2_V:   str2 = R * C; str1 = R * 2; return;
        case tdm_stor_mode::Loop_4x4_H:   str2 = R * C; str1 = C * 4; return;
        case tdm_stor_mode::Loop_4x4_V:   str2 = R * C; str1 = R * 4; return;
        case tdm_stor_mode::Loop_Row:     str2 = R * C * L; str1 = R * C; return;
        case tdm_stor_mode::Row_Loop:     str2 = R * C * L; str1 = L; return;
        default:
            // No case for the remaining store_modes — hard error rather
            // than silently routing garbage.
            SC_REPORT_FATAL("addr_hash", "XBAR_HASH16/XBAR_HASH32: unsupported store_mode");
            str2 = str1 = bank_width;
        }
    }

    static uint64_t hash16_bit(uint64_t v, int i) {
        return (v >> i) & 0x1ull;
    }
#endif

#if defined(XBAR_HASH16)
    // 4-bit bank id (0..15). addr_hash16() ORs in a fixed per-AGU "hi_bank"
    // bit for AGU-vs-AGU bank segregation; this alone does NOT make the
    // 4-bit output a bijection. s2/s1/s0 are the stride coords, bit(v,i)
    // reads bit i of each.
    static uint64_t hash16_combine(uint64_t s2, uint64_t s1, uint64_t s0) {
        const uint64_t g0 = hash16_bit(s0, 1) ^ hash16_bit(s1, 0) ^ hash16_bit(s2, 0);
        const uint64_t g1 =
            hash16_bit(s0, 0) ^ hash16_bit(s1, 0) ^ hash16_bit(s1, 1) ^ hash16_bit(s2, 1);
        const uint64_t g2 = hash16_bit(s0, 0) ^ hash16_bit(s0, 2) ^ hash16_bit(s1, 1) ^
                            hash16_bit(s1, 2) ^ hash16_bit(s2, 2);
        const uint64_t g3 = hash16_bit(s0, 0) ^ hash16_bit(s0, 3) ^ hash16_bit(s1, 3) ^
                            hash16_bit(s2, 0) ^ hash16_bit(s2, 3);
        return (g0 << 0) | (g1 << 1) | (g2 << 2) | (g3 << 3);
    }

    static uint64_t addr_hash16(uint64_t a, uint64_t R, uint64_t C, uint64_t L, uint64_t store_mode,
                                bool hi_bank) {
        static constexpr uint64_t kBankWidth = 4;
        uint64_t                  str2 = 0, str1 = 0, str0 = 0;
        hash16_get_strides(static_cast<tdm_stor_mode>(store_mode), R, C, L, kBankWidth, str2, str1,
                           str0);
        // R/C/L can read 0 on the very first delta cycle — clamp to avoid
        // divide-by-zero; a real task always overwrites these first.
        if (str2 == 0)
            str2 = 1;
        if (str1 == 0)
            str1 = 1;
        // Each task's buffer sits on a fixed 64KiB boundary — mask to that
        // window before the stride math, or s2/s1/s0 depend on the buffer's
        // placement instead of the task's own (row,col,loop) coordinates.
        const uint64_t a_word  = (a & 0xFFFFull) / kBankWidth;
        const uint64_t s2      = a_word / str2;
        const uint64_t s1      = (a_word % str2) / str1;
        const uint64_t s0      = (a_word % str1) / str0;
        const uint64_t bank_id = hash16_combine(s2, s1, s0) | (hi_bank ? 0x10ull : 0); // 0..31
        const uint64_t field_mask = (1ull << (LOG_REQ + LOG_BANK_GRP)) - 1;
        return (a & ~(field_mask << ROUTE_LSB)) | (bank_id << ROUTE_LSB);
    }
#endif

#if defined(XBAR_HASH32)
    // Same stride coords as XBAR_HASH16, combined into the full 5-bit
    // L1+L2 field directly (no hi_bank needed). NOT a proven bijection:
    // dimensionally capable (5-bit output, 5-bit field) but this XOR
    // formula isn't full-rank over every task's varying bits (~13%
    // residual aliasing measured against real traffic — see doc/report).
    static uint64_t hash32_combine(uint64_t s2, uint64_t s1, uint64_t s0) {
        const uint64_t g0 = hash16_bit(s0, 1) ^ hash16_bit(s1, 0) ^ hash16_bit(s1, 3) ^
                            hash16_bit(s2, 0) ^ hash16_bit(s2, 3);
        const uint64_t g1 =
            hash16_bit(s0, 0) ^ hash16_bit(s1, 1) ^ hash16_bit(s2, 0) ^ hash16_bit(s2, 1);
        const uint64_t g2 = hash16_bit(s0, 2) ^ hash16_bit(s1, 0) ^ hash16_bit(s1, 1) ^
                            hash16_bit(s1, 4) ^ hash16_bit(s2, 2);
        const uint64_t g3 =
            hash16_bit(s0, 2) ^ hash16_bit(s0, 4) ^ hash16_bit(s1, 2) ^ hash16_bit(s2, 0);
        const uint64_t g4 =
            hash16_bit(s0, 3) ^ hash16_bit(s1, 0) ^ hash16_bit(s2, 1) ^ hash16_bit(s2, 4);
        return (g0 << 0) | (g1 << 1) | (g2 << 2) | (g3 << 3) | (g4 << 4);
    }

    static uint64_t addr_hash32(uint64_t a, uint64_t R, uint64_t C, uint64_t L, uint64_t store_mode) {
        static constexpr uint64_t kBankWidth = 4;
        uint64_t                  str2 = 0, str1 = 0, str0 = 0;
        hash16_get_strides(static_cast<tdm_stor_mode>(store_mode), R, C, L, kBankWidth, str2, str1,
                           str0);
        if (str2 == 0)
            str2 = 1;
        if (str1 == 0)
            str1 = 1;
        // Buffer-local 64KiB mask — see addr_hash16()'s comment.
        const uint64_t a_word     = (a & 0xFFFFull) / kBankWidth;
        const uint64_t s2         = a_word / str2;
        const uint64_t s1         = (a_word % str2) / str1;
        const uint64_t s0         = (a_word % str1) / str0;
        const uint64_t bank_id    = hash32_combine(s2, s1, s0); // 0..31, full field directly
        const uint64_t field_mask = (1ull << (LOG_REQ + LOG_BANK_GRP)) - 1;
        return (a & ~(field_mask << ROUTE_LSB)) | (bank_id << ROUTE_LSB);
    }
#endif
};

#endif
