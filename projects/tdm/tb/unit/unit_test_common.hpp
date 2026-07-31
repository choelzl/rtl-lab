// -----------------------------------------------------------------------------
// Shared bookkeeping/helpers duplicated identically across every tb_*.cpp
// unit test in this project (verified byte-for-byte before consolidating
// here) — g_pass/g_fail/CHECK() for PASS/FAIL reporting, and make_row() for
// the standard "32-bit pattern replicated across a 128-bit row" test
// payload. Keeping these in one place means a change to the summary format
// or fail-detection logic only has to happen once, instead of drifting
// independently across N copies.
//
// Each tb_*.cpp is its own standalone SystemC binary (compiled and run
// separately, never linked with another tb_*.cpp), so plain `inline`
// variables/functions here are sufficient — no risk of cross-binary ODR
// issues, only the ordinary single-definition-per-translation-unit rule
// C++17 inline already covers.
// -----------------------------------------------------------------------------

#ifndef UNIT_TEST_COMMON_HPP
#define UNIT_TEST_COMMON_HPP

#include <cstdint>
#include <cstdio>
#include <systemc.h>

inline int g_pass = 0;
inline int g_fail = 0;

inline void CHECK(bool cond, const char *label) {
    if (cond) {
        ++g_pass;
        std::printf("  PASS  %s\n", label);
    } else {
        ++g_fail;
        std::printf("  FAIL  %s\n", label);
    }
}

// Replicates a 32-bit pattern across all 128 bits of a data_t row — the
// standard fingerprintable write/read payload used throughout these tests.
template <typename DATA_T> DATA_T make_row(uint32_t v) {
    sc_bv<32> w(v);
    DATA_T    d;
    d.range(31, 0)   = w;
    d.range(63, 32)  = w;
    d.range(95, 64)  = w;
    d.range(127, 96) = w;
    return d;
}

// Advances exactly one clock edge plus a small settle margin — the standard
// "drive this cycle, sample next" idiom every testbench here uses. Takes the
// clock explicitly (rather than assuming a member named `clk`) so it works
// the same whether called from a testbench's own thread or a free helper.
inline void tick(sc_clock &clk) {
    wait(clk.posedge_event());
    wait(1, SC_NS);
}

// Prints the standard PASS/FAIL summary — the "  passed: N" / "  failed: N"
// lines are what scripts/_unit.sh and check_all_stimuli.sh grep from each
// binary's stdout to tally counts across the whole suite, so the exact
// "passed:"/"failed:" label text matters, not just the numbers — then
// returns the process exit code every tb_*.cpp's sc_main() should return
// after sc_start() completes.
inline int report_and_exit() {
    std::printf("\n=== Summary ===\n  passed: %d\n  failed: %d\n", g_pass, g_fail);
    if (g_fail > 0) {
        std::fprintf(stderr, "\n%d test(s) FAILED\n", g_fail);
        return 1;
    }
    std::puts("\nAll tests passed.");
    return 0;
}

#endif // UNIT_TEST_COMMON_HPP
