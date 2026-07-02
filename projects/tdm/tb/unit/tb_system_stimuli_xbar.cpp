// System-level integration test, crossbar backend — see
// tb_system_stimuli_tdm.cpp for the native-TDM counterpart and usage notes.
// Same stimuli set, same read-after-write correctness check; only cycle
// count is expected to differ between the two backends.
#define IMPL_CROSSBAR
#include "system_stimuli_common.hpp"

int sc_main(int, char **) {
    const char       *env_in = std::getenv("SEL_IN_DIR");
    const std::string in_dir = env_in ? env_in : "sample";

    std::printf("=== System stimuli: %s (IMPL=crossbar) ===\n", in_dir.c_str());
    return run_and_report(resolve_stim_dir(in_dir), (in_dir + "_on_xbar").c_str());
}
