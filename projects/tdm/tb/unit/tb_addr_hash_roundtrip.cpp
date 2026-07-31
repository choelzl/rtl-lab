// -----------------------------------------------------------------------------
// Checks addr_hash_inv() (top_crossbar.hpp) both for correctness (it is a
// genuine two-sided inverse of addr_hash()'s unconditional L2 fold, i.e. of
// "hash11" in this session's terminology) and for existence (it is not a
// degenerate stand-in for the identity function). No SC_MODULE instantiation
// needed: addr_hash()/addr_hash_inv()/addr_hash16()/addr_hash32() are all
// static, pure functions of their arguments.
//
// Confirms directly, by construction rather than simulation, that
// hash11(hash11_inv(hash16(addr))) == hash16(addr) for every case tested —
// i.e. composing addr_hash_inv() then addr_hash() onto XBAR_HASH16/32's
// output is provably a no-op, matching the algebraic argument (f(f^-1(x))=x
// for any bijection f) made when this was proposed.
// -----------------------------------------------------------------------------
// "hash11" specifically means addr_hash()'s unconditional L2 fold with
// NEITHER of these layered on top — force that regardless of what an
// external PARAMS= might also define, since _unit.sh applies PARAMS globally
// to every tb_*.cpp (see run_hash_sweep.sh's own comment on this), and
// addr_hash_inv() only ever undoes the L2 fold, not an L1/L1_V2 addition.
#undef XBAR_HASH_L1
#undef XBAR_HASH_L1_V2
#define XBAR_HASH16
#define XBAR_HASH32
#include "top_crossbar.hpp"
#include "unit_test_common.hpp"
#include <cstdio>
#include <random>

// Production-shaped (5-bit L1+L2 field): required by XBAR_HASH16/32's own
// static_assert, unrelated to whether a live crossbar instance is built.
using DUT      = top_crossbar<9, 8, 4, 32, 1024, 4, 4>;
using hash_ops = DUT::hash_ops;

int sc_main(int, char *[]) {
    std::mt19937_64                         rng(0xC0FFEE);
    std::uniform_int_distribution<uint64_t> addr_dist(0, (1ull << 20) - 1);
    std::uniform_int_distribution<int>      rc_dist(1, 8); // small, power-of-two-friendly
    static constexpr uint64_t               kStoreModes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};

    // --- Correctness: addr_hash()/addr_hash_inv() are true two-sided
    // inverses of each other, for arbitrary addresses (not just hash16/32
    // outputs) ---
    int checked_general = 0;
    for (int i = 0; i < 20000; ++i) {
        const uint64_t a            = addr_dist(rng);
        const uint64_t fwd_then_inv = hash_ops::addr_hash_inv(hash_ops::addr_hash(a));
        const uint64_t inv_then_fwd = hash_ops::addr_hash(hash_ops::addr_hash_inv(a));
        if (fwd_then_inv != a || inv_then_fwd != a) {
            char lbl[160];
            std::snprintf(
                lbl, sizeof(lbl),
                "addr_hash_inv(addr_hash(0x%llx))==a and addr_hash(addr_hash_inv(0x%llx))==a",
                (unsigned long long)a, (unsigned long long)a);
            CHECK(false, lbl);
        }
        ++checked_general;
    }
    CHECK(checked_general == 20000, "ran all 20000 general round-trip cases");

    // --- Existence: addr_hash_inv() is not secretly the identity function
    // (a degenerate "inverse" that never changes anything) ---
    int differed = 0;
    for (int i = 0; i < 20000; ++i) {
        const uint64_t a = addr_dist(rng);
        if (hash_ops::addr_hash_inv(a) != a)
            ++differed;
    }
    CHECK(differed > 15000,
          "addr_hash_inv() is a real, non-identity function (differs on most inputs)");

    // --- The actual claim under test: hash11(hash11_inv(hash16/32(addr)))
    // == hash16/32(addr) exactly, across a spread of task descriptors and
    // addresses (napa/hi_bank included for hash16 specifically) ---
    int mismatches16 = 0, mismatches32 = 0, cases = 0;
    for (int i = 0; i < 5000; ++i) {
        const uint64_t a       = addr_dist(rng);
        const uint64_t R       = static_cast<uint64_t>(rc_dist(rng));
        const uint64_t C       = static_cast<uint64_t>(rc_dist(rng));
        const uint64_t L       = static_cast<uint64_t>(rc_dist(rng)) * 4;
        const uint64_t sm      = kStoreModes[i % 12];
        const bool     hi_bank = (i % 2) == 0;

        const uint64_t h16         = hash_ops::addr_hash16(a, R, C, L, sm, hi_bank);
        const uint64_t roundtrip16 = hash_ops::addr_hash(hash_ops::addr_hash_inv(h16));
        if (roundtrip16 != h16)
            ++mismatches16;

        const uint64_t h32         = hash_ops::addr_hash32(a, R, C, L, sm);
        const uint64_t roundtrip32 = hash_ops::addr_hash(hash_ops::addr_hash_inv(h32));
        if (roundtrip32 != h32)
            ++mismatches32;

        ++cases;
    }
    CHECK(cases == 5000, "ran all 5000 hash16/32 composition cases");
    CHECK(mismatches16 == 0,
          "hash11(hash11_inv(hash16(addr))) == hash16(addr) for every case (0 mismatches)");
    CHECK(mismatches32 == 0,
          "hash11(hash11_inv(hash32(addr))) == hash32(addr) for every case (0 mismatches)");

    return report_and_exit();
}
