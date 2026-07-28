// -----------------------------------------------------------------------------
// Author: Simone Machetti, Cedric Hölzl
//
// TDM address-mapping function, shared. Given a byte address and kernel
// mapping params (num_banks, bank_width, R, C, L, store_mode), places it
// into a (bank_id, row_id) via an XOR-skewed banking scheme. Pure functions,
// no state — see doc/specs/map_func.md for the full spec. Extracted from
// tdm.hpp so other backends (e.g. top_crossbar.hpp's XBAR_HASH_DYNAMIC) can
// reuse the same placement scheme.
// -----------------------------------------------------------------------------

#ifndef MAP_FUNC_HPP
#define MAP_FUNC_HPP

#include <algorithm>
#include <cstdint>
#include <systemc.h>
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

namespace map_func {

inline uint32_t ilog2(uint64_t v) {
    uint32_t r = 0;
    while (v > 1) {
        v >>= 1;
        ++r;
    }
    return r;
}

inline uint32_t tzeros(uint64_t v) {
    if (v == 0)
        return 0;
    uint32_t r = 0;
    while (((v >> r) & 1ull) == 0)
        ++r;
    return r;
}

inline std::pair<uint32_t, uint32_t> get_k_raw(tdm_stor_mode mode, uint32_t e, uint32_t tzR,
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
        SC_REPORT_ERROR("map_func", "unsupported store_mode");
        return {e, e};
    }
}

inline std::pair<uint32_t, uint32_t> get_k(tdm_stor_mode mode, uint32_t e, uint32_t tzR,
                                           uint32_t tzC, uint32_t tzL) {
    const std::pair<uint32_t, uint32_t> k = get_k_raw(mode, e, tzR, tzC, tzL);
#ifdef TDM_GETK_GUARD
    // Degenerate-split guard (experimental, opt-in; doc/report Appendix
    // A.8): when the mode's leading dimension has no trailing zeros (e.g.
    // C=1 under Loop_Row), con collapses to zero-width and some window
    // layouts fold onto half the banks. Borrows up to two bits from str's
    // bottom so con is never empty; pattern-blind, so it can also
    // introduce collisions elsewhere — see the report for the evaluation.
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

// Pure function of its arguments (no module state) — places one byte address
// into a (bank_id, row_id) location. bank_id is a 5-bit value (0..31); callers
// targeting a physical bank count that isn't exactly 32 must fold/guard it
// themselves (see doc/specs/map_func.md's "32 banks" note).
inline void map_one(uint64_t addr, uint64_t nb, uint64_t bw, uint64_t R, uint64_t C, uint64_t L,
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

// -----------------------------------------------------------------------
// "Stride XOR" bank hash — alternate candidate mapping, ported 1:1 from
// projects/tdm/pythonXOR_mapfun.py (input_layer.py's StorMode/get_addr).
// Unlike map_one() (bit-shift fields, power-of-two R/C/L only), this uses
// plain dimension products for the strides, so it also covers non-power-
// of-two geometries — but only the 6 simple permutation modes (0-5) and
// Loop_4x4 (8,9); STRIDE_XOR_N_BANKS/WIDTH (8 banks, 3-bit XOR) match the
// python reference, not this project's N_BANK=32 target. addr is in
// ELEMENTS, not bytes like map_one().
// -----------------------------------------------------------------------

constexpr int STRIDE_XOR_N_BANKS    = 8;
constexpr int STRIDE_XOR_BANK_WIDTH = 4;

// (str2, str1) such that addr = s2*str2 + s1*str1 + s0*bank_width + offset —
// ports get_strides(). str1 = size(innermost axis), str2 = size(middle
// axis) * str1 (see the file-level comment above for why the per-mode
// inner/middle pick needs no runtime address sampling).
inline void stride_get_strides(tdm_stor_mode mode, uint64_t R, uint64_t C, uint64_t L,
                               uint64_t &str2, uint64_t &str1) {
    using M = tdm_stor_mode;
    if (mode == M::Loop_4x4_H || mode == M::Loop_4x4_V) {
        const uint64_t ceil_C4 = (C + 3) / 4, ceil_R4 = (R + 3) / 4;
        str2 = 16 * ceil_C4 * ceil_R4;
        str1 = 16 * (mode == M::Loop_4x4_H ? ceil_C4 : ceil_R4);
        return;
    }

    uint64_t inner, middle;
    switch (mode) {
    case M::Loop_Row_Col:
        inner = C;
        middle = R;
        break;
    case M::Loop_Col_Row:
        inner = R;
        middle = C;
        break;
    case M::Row_Col_Loop:
        inner = L;
        middle = C;
        break;
    case M::Col_Row_Loop:
        inner = L;
        middle = R;
        break;
    case M::Row_Loop_Col:
        inner = C;
        middle = L;
        break;
    case M::Col_Loop_Row:
        inner = R;
        middle = L;
        break;
    default:
        SC_REPORT_FATAL("map_func", "stride_get_strides: unsupported store_mode");
        str2 = str1 = 0;
        return;
    }
    str1 = inner;
    str2 = middle * inner;
}

// Ports stride_xor_bank(): a fixed 3-bit XOR of hand-picked (s0,s1,s2) bit
// positions, where s2/s1/s0 come from decomposing addr against (str2,str1,
// bank_width) — see the file-level comment above for scope/limitations.
inline uint32_t stride_xor_bank(uint64_t addr, uint64_t str2, uint64_t str1,
                                uint64_t bank_width = STRIDE_XOR_BANK_WIDTH) {
    const uint64_t s2 = addr / str2;
    const uint64_t r2 = addr % str2;
    const uint64_t s1 = r2 / str1;
    const uint64_t r1 = r2 % str1;
    const uint64_t s0 = r1 / bank_width;

    auto bit = [](uint64_t v, uint32_t i) -> uint64_t { return (v >> i) & 1ull; };

    const uint32_t bit0 = static_cast<uint32_t>(bit(s0, 0) ^ bit(s1, 2) ^ bit(s2, 0) ^ bit(s2, 2));
    const uint32_t bit1 = static_cast<uint32_t>(bit(s0, 2) ^ bit(s1, 0) ^ bit(s2, 0));
    const uint32_t bit2 = static_cast<uint32_t>(bit(s0, 1) ^ bit(s1, 1) ^ bit(s2, 0) ^ bit(s2, 1));
    return bit0 | (bit1 << 1) | (bit2 << 2);
}

// Ports make_bank_fn()+stride_xor_bank(): addr (in elements) -> bank (0..7)
// for one (mode, R, C, L) kernel geometry.
inline uint32_t stride_xor_bank_of(tdm_stor_mode mode, uint64_t R, uint64_t C, uint64_t L,
                                   uint64_t addr) {
    uint64_t str2 = 0, str1 = 0;
    stride_get_strides(mode, R, C, L, str2, str1);
    return stride_xor_bank(addr, str2, str1);
}

} // namespace map_func

#endif
