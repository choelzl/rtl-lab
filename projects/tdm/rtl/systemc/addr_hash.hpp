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
        // 5-term additive fold of the non-overlapping windows addr[5:4]/
        // [7:6]/[9:8]/[11:10]/[13:12] — repairs strided R/C column-walks
        // XBAR_HASH_L1's single term misses (doc/report Appendix A.8).
        // This is also the fold every geometry uses now (see
        // is_vector_geometry() below): a dedicated fold for napa>1 vector
        // tasks was tried and measured WORSE (these tasks are confined to
        // small buffers, so a wider dedicated window mostly read constant
        // bits). Also tried and measured no better (same sweep): a 6th
        // term (addr[15:14]); XOR instead of additive here; a 3rd L2 term
        // (addr[14:12], L1_V2-only); extending vector_axis_fold_addr()'s
        // own XOR fold to 5 terms. This is this fold family's practical
        // floor on real traffic, well short of HASH16/32's <1%, which
        // comes from knowing the task's actual R/C/L stride instead of
        // guessing at fixed bit windows. Per-address only (no
        // cross-lane state: bank.hpp needs same address -> same field).
        uint64_t l1 = (a >> 4) & 0x3;
        l1          = (l1 + ((a >> 6) & 0x3)) & 0x3;
        l1          = (l1 + ((a >> 8) & 0x3)) & 0x3;
        l1          = (l1 + ((a >> 10) & 0x3)) & 0x3;
        l1          = (l1 + ((a >> 12) & 0x3)) & 0x3;
        r           = (r & ~(static_cast<uint64_t>(0x3) << 4)) | (l1 << 4);
#elif defined(XBAR_HASH_L1_V3S)
        // Selector-free (L1,L2) fold pair — the best UNCONDITIONAL single
        // formula a joint L1+L2 offline search found on the patroklos2
        // sweep. L1 = addr[5:4]^[7:6]^[9:8]^[10:9]: the overlapping
        // [9:8]/[10:9] window pair fixes the 2-D lane grids (e.g. lanes
        // {base,+0x100,+0x400,+0x500}) that no aligned-window fold
        // separates — their XOR contributes bit9^bit10 to the high output
        // bit and bit8^bit9 to the low one, gathering one bit from each
        // grid stride. L2 = addr[7:5]^[10:8]^[12:10] REPLACES the hi+mid
        // ADD fold above (overwritten below): co-optimizing L2 with this L1
        // matters because the L1 field selects which L2 instance a beat
        // traverses, so L1 and L2 conflict rates are coupled (joint static
        // 7.2% vs 9.6% keeping the stock L2). Purely address-keyed: unlike
        // L1_V2/V3 this can NEVER route the same address to different banks
        // across a buffer's lifetime — no descriptor-selector risk at all.
        uint64_t l1 = (a >> 4) & 0x3;
        l1 ^= (a >> 6) & 0x3;
        l1 ^= (a >> 8) & 0x3;
        l1 ^= (a >> 9) & 0x3;
        uint64_t l2 = (a >> 5) & 0x7;
        l2 ^= (a >> 8) & 0x7;
        l2 ^= (a >> 10) & 0x7;
        r = (a & ~(static_cast<uint64_t>(0x1f) << 4)) | (l2 << 6) | (l1 << 4);
#elif defined(XBAR_HASH_POLY)
        // GF(2^5) polynomial-skew bank interleaving (classic Rau-style):
        // bank[4:0] = q[4:0] ^ (x^1 * row mod p), p = x^5 + x^2 + 1
        // (primitive), q = a>>4 (one 16B row per bank granule), row = q>>5;
        // 7 row bits folded = a[15:9], the buffer-local 64 KiB. kG[i] =
        // x^(1+i) mod p. Selector-free / address-only like V3S. Writes the
        // WHOLE 5-bit field a[8:4] (overriding the unconditional hi+mid L2
        // fold above). Measured caveat for THIS fabric: the primitive
        // polynomial guarantees distinct BANKS for power-of-2 strides, but
        // the fabric structurally splits bank[4:0] into L1=bank[1:0] and
        // L2=bank[4:2] — and the fold only reaches bank[1:0] when row bits
        // 0/4/5 flip, so 2-D lane grids (e.g. {0,+0x100,+0x400,+0x500} ->
        // banks {0,16,4,20}) land on 4 DISTINCT banks that share one L1
        // field and still serialize in the port's 4x4 switch.
        static const uint64_t kG[7] = {2, 4, 8, 16, 5, 10, 20};
        const uint64_t q    = a >> 4;
        const uint64_t row  = q >> 5;
        uint64_t       fold = 0;
        for (int i = 0; i < 7; ++i)
            if ((row >> i) & 1)
                fold ^= kG[i];
        r = (a & ~(static_cast<uint64_t>(0x1f) << 4)) | (((q & 0x1f) ^ fold) << 4);
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
    // Picks the L1 fold's own LENGTH (still ADD, still the same
    // non-overlapping addr[5:4]/[7:6]/[9:8]/[11:10]/[13:12] windows) per
    // port-group from a fixed R/C/napa rule — promoted from an earlier
    // "XBAR_HASH_L1_V2_ALT" comparison scheme to the deployed
    // XBAR_HASH_L1_V2 formula once it measured at least as good as the old
    // fixed two-path dispatch (is_vector_geometry()+napa1 selecting a
    // dedicated 4-term XOR vs. the ordinary 5-term ADD) and a small real
    // edge under full-traffic contention (doc/report §4.4). Mined from an
    // offline oracle sweep: with ragu_a's own descriptors broken out by
    // (R,C,L,store_mode,napa), the BEST achievable per-shape choice among
    // {no fold, 2-term ADD, 3-term ADD, 5-term ADD} reaches 5.5% weighted M1
    // L1% — but that ceiling assumes an oracle that already knows the right
    // answer per shape. This rule is the best *practically expressible*
    // R/C/napa-keyed selector found by search (a 2-level threshold split,
    // footprint = R*C): it only reaches ~7.8%, because the winning fold
    // doesn't correlate cleanly enough with these fields for a static rule
    // to recover most of the oracle's gap (a single huge-volume napa=8
    // shape needs the 5-term fold specifically while other napa=8 shapes of
    // similar volume need 2/3-term — no threshold separates them).
    //
    // CORRECTNESS CAVEAT: R/C/napa are properties of the task descriptor
    // CURRENTLY active on a port, not of the address. Real buffers in this
    // traffic are commonly touched by 10+ distinct (napa,R,C,L,store_mode)
    // combinations over their lifetime, so this rule can route the SAME
    // address to a DIFFERENT bank at different points in that buffer's
    // life — a live correctness risk this session flagged and explicitly
    // deferred (the actual fix is a software-assigned, buffer-stable
    // formula ID, analogous to XBAR_HASH16's existing hi_bank mechanism,
    // not something re-derived from a task's transient R/C/L view). The
    // production testbench performs no read-after-write verification, so
    // none of this scheme's timing numbers are evidence of correctness
    // either way.
    static uint64_t addr_hash_l1_v2(uint64_t a, uint64_t R, uint64_t C, uint64_t napa) {
        const uint64_t footprint = R * C;
        int            n_terms;
        if (napa <= 4) {
            n_terms = (footprint <= 2048) ? 3 : 2;
        } else if (footprint <= 1024) {
            n_terms = 5;
        } else {
            return a; // "baseline" leg of the rule: no fold at all, raw passthrough
        }
        const uint64_t hi  = (a >> 9) & 0x7;
        const uint64_t mid = (a >> 6) & 0x7;
        uint64_t       r   = (a & ~(static_cast<uint64_t>(0x7) << 6)) | (((hi + mid) & 0x7) << 6);
        uint64_t       l1  = (a >> 4) & 0x3;
        if (n_terms >= 2)
            l1 = (l1 + ((a >> 6) & 0x3)) & 0x3;
        if (n_terms >= 3)
            l1 = (l1 + ((a >> 8) & 0x3)) & 0x3;
        if (n_terms >= 4)
            l1 = (l1 + ((a >> 10) & 0x3)) & 0x3;
        if (n_terms >= 5)
            l1 = (l1 + ((a >> 12) & 0x3)) & 0x3;
        r = (r & ~(static_cast<uint64_t>(0x3) << 4)) | (l1 << 4);
        return r;
    }
#endif

#if defined(XBAR_HASH_L1_V3)
    // Two L1 XOR folds keyed on the task's C via a single compare (no R,
    // no L, no napa — deliberately the narrowest descriptor surface the
    // offline search could reach; see doc/report §4.4), over ONE shared
    // address-only L2 ADD fold that replaces addr_hash()'s stock hi+mid.
    // C<=96 tasks use XBAR_HASH_L1_V3S's selector-free L1 fold;
    // very-wide-column tasks (C>96) push the L1 windows up (their lane
    // strides live in higher bits). Chosen by a JOINT L1+L2 static model —
    // an L1-only search first found folds that halved L1 conflicts but
    // gave most of the win back at L2, because the L1 field selects which
    // L2 instance a beat traverses, so the two stages' conflict rates are
    // coupled (measured: M1 L2 3.6%->6.5% under the L1-only pair); the
    // shared L2 was then re-searched jointly under both L1 legs (the legs
    // themselves re-verified optimal under the sharing constraint).
    // Measured M1 tot 6.4%/3.4% (unpadded/padded32, ragu_a isolated) vs
    // 11.0%/8.7% for XBAR_HASH_L1_V2. Variants measured: per-leg L2
    // (each C leg with its own L2 fold) trades the other way, 7.3%/2.5% —
    // better padded32, worse unpadded, second formula surface; 3 L1 legs
    // (C<=34/66) 6.3%/1.6%; 4 legs 5.3%/1.6% — padded32 saturates near
    // 1.6% for every rule-based selector tried. Exact zero is NOT
    // reachable with
    // fold formulas: a perfect per-address lookup provably exists for
    // every shape (graph-coloring feasible, no duplicate addresses share a
    // cycle), but six small R1_C16/SM10 shapes (~0.6% of padded32 traffic)
    // resist every fold/gather candidate swept (~4k L1 x ~1k L2). Carries
    // the SAME correctness caveat as addr_hash_l1_v2() below: C is
    // transient task-descriptor state, so the same address can route to a
    // different bank across a buffer's life — reduced surface (one field
    // instead of three), not eliminated. XBAR_HASH_L1_V3S eliminates it
    // entirely.
    static uint64_t addr_hash_l1_v3(uint64_t a, uint64_t C) {
        // L2 is ONE shared address-only fold for both legs (only L1 muxes
        // on C): 3-term ADD over a[8:6]/[9:7]/[11:9].
        const uint64_t l2 = (((a >> 6) & 0x7) + ((a >> 7) & 0x7) + ((a >> 9) & 0x7)) & 0x7;
        uint64_t       l1 = (a >> 4) & 0x3;
        if (C <= 96) {
            l1 ^= ((a >> 6) ^ (a >> 8) ^ (a >> 9)) & 0x3;
        } else { // very wide columns: strides live in higher bits
            l1 ^= ((a >> 8) ^ (a >> 10) ^ (a >> 11)) & 0x3;
        }
        return (a & ~(static_cast<uint64_t>(0x1f) << 4)) | (l2 << 6) | (l1 << 4);
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
