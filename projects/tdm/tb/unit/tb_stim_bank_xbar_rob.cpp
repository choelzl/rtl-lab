// Bank-routing integration test, crossbar backend with the XBAR_ROB
// read-side reorder buffers (top_crossbar.hpp) — the ROB's correctness
// suite: same 8-phase sweep, same routing + read-data checks as
// tb_stim_bank_xbar.cpp, exercised through the ROB's prefetch/schedule/
// deliver path instead of direct fabric requests. Depth 4 (the smallest
// depth that sustains 1 group/cycle through the refill pipeline — see
// top_crossbar.hpp's ROB_DEPTH comment); baseline hash (no XBAR_HASH_*
// macro), so the suite's xbar_field_addr() routing predictions apply
// unchanged. Timing expectations differ from the plain crossbar where
// conflicts are dissolved by prefetch — see the XBAR_ROB rows in
// stim_bank_common.hpp's per-backend span tables.
#define IMPL_CROSSBAR
#define XBAR_ROB
#define XBAR_ROB_DEPTH 4
#include "stim_bank_common.hpp"

int sc_main(int, char **) {
    std::printf("=== Bank-routing check (IMPL=crossbar+rob) ===\n");
    run_bank_check("crossbar+rob");
    return report_and_exit();
}
