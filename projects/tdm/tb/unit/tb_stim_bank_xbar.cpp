// Bank-routing integration test, crossbar backend — see
// tb_stim_bank_tdm.cpp / stim_bank_common.hpp for the native-TDM counterpart
// and usage notes. Same sweep, same checks; only the routing formula differs.
#define IMPL_CROSSBAR
#include "stim_bank_common.hpp"

int sc_main(int, char **) {
    std::printf("=== Bank-routing check (IMPL=crossbar) ===\n");
    run_bank_check("crossbar");
    return report_and_exit();
}
