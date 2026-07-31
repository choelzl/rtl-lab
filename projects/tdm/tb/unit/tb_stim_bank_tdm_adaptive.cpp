// Bank-routing integration test, native TDM backend with the
// request-aware adaptive arbiter (IMPL_ARB_ADAPTIVE) instead of the
// default free-running round-robin — see arbiter_adaptive.hpp and
// top_tdm.hpp's toggle comment. Identical sweep and checks to
// tb_stim_bank_tdm.cpp; only the phase-5 exact-cycle timing constants
// differ (see stim_bank_common.hpp's kExpected adaptive branch). Exists
// as its own suite so the adaptive arbiter build is exercised by the
// regular unit-test run rather than only when someone remembers to
// compile with the define by hand.
#define IMPL_TDM
#define IMPL_ARB_ADAPTIVE
#include "stim_bank_common.hpp"

int sc_main(int, char **) {
    std::printf("=== Bank-routing check (IMPL=tdm, adaptive arbiter) ===\n");
    run_bank_check("tdm-adaptive");
    return report_and_exit();
}
