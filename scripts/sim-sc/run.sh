#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
#
# SystemC simulation flow. Verilates the SV DUT named by SEL_TOP_LEVEL into an
# sc_module and links it against the per-top-level C++/SystemC harness
# (tb/systemc/tb_<top>.cpp), which becomes sc_main. The harness wires the
# Verilated DUT to native SystemC modules and drives the simulation under the
# SystemC kernel. Simulation only -- the SystemC parts are never synthesized.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${CODE_HOME}/rtl-lab/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
cflags="-std=c++17 -I${PROJ}/tb/systemc -DCLK_PERIOD_NS=${SEL_CLK_PERIOD_NS}"

if [ "${SEL_PARAMS}" != "none" ]; then
    for param in ${SEL_PARAMS}; do
        g_flags+=("-G${param}")
        cflags="${cflags} -D${param}"
    done
fi

if [ "${SEL_TB_DEFS}" != "none" ]; then
    for def in ${SEL_TB_DEFS}; do
        cflags="${cflags} -D${def}"
    done
fi

verilator \
    --sc \
    --exe \
    --build \
    -sv \
    --trace \
    --trace-max-array 0 \
    --trace-max-width 0 \
    -Wall \
    -Wno-fatal \
    -CFLAGS "${cflags}" \
    "${g_flags[@]}" \
    -I"${PROJ}/rtl" \
    --top-module "${SEL_TOP_LEVEL}" \
    "${PROJ}/rtl/${SEL_TOP_LEVEL}.sv" \
    "${PROJ}/tb/systemc/tb_${SEL_TOP_LEVEL}.cpp" \
    -Mdir "${SIM}/build/obj_dir" \
    -o "${SIM}/build/simv" \
    | tee "${SIM}/output/compile.log"

exec "${SIM}/build/simv" "$@" \
    | tee "${SIM}/output/run.log"
