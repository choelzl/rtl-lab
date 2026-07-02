// System-level integration test, native TDM backend: drives ONE stimuli
// directory (selected at runtime via SEL_IN_DIR, matching tb_top.cpp's own
// convention) through the full production path and checks read-after-write
// correctness. See system_stimuli_common.hpp for what this checks and why.
//
// This replaced one .cpp file per stimuli set; to check every stimuli
// directory under tb/stimuli/ (each identified by having a ragu_*.log —
// its "result file"), run scripts/check_all_stimuli.sh, which builds this
// binary (and its crossbar counterpart, tb_system_stimuli_xbar.cpp) once and
// re-invokes them with SEL_IN_DIR set to each discovered directory in turn.
#define IMPL_TDM
#include "system_stimuli_common.hpp"

int sc_main(int, char **) {
    const char       *env_in = std::getenv("SEL_IN_DIR");
    const std::string in_dir = env_in ? env_in : "sample";

    std::printf("=== System stimuli: %s (IMPL=tdm) ===\n", in_dir.c_str());
    return run_and_report(resolve_stim_dir(in_dir), (in_dir + "_on_tdm").c_str());
}
