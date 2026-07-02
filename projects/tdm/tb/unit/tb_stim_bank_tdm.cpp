// Bank-routing integration test, native TDM backend — see
// stim_bank_common.hpp for what this checks and why (the incremental
// port-count/request-count sweep, phase structure, and routing-check design).
// Same sweep and same checks as tb_stim_bank_xbar.cpp; only the routing
// formula differs per backend.
#define IMPL_TDM
#include "stim_bank_common.hpp"

int sc_main(int, char **) {
    std::printf("=== Bank-routing check (IMPL=tdm) ===\n");
    run_bank_check("tdm");
    return report_and_exit();
}
