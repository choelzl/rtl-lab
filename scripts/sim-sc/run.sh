#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
#
# SystemC simulation flow. Verilates the project's SV sources (rtl/*.sv) into
# sc_modules and links them against the C++/SystemC harness named by
# SEL_TOP_LEVEL (tb/systemc/tb_<top>.cpp), which becomes sc_main. SEL_TOP_LEVEL
# names the SystemC design top (rtl/systemc/<top>.hpp) that the harness
# instantiates; that top wires the Verilated SV DUT(s) to native SystemC design
# modules (rtl/systemc/). Verilator picks the SV top automatically from the
# given sources. Simulation only -- the SystemC parts are never synthesized.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${CODE_HOME}/rtl-lab/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
cflags="-std=c++17 -I${PROJ}/tb/systemc -I${PROJ}/rtl/systemc -DCLK_PERIOD_NS=${SEL_CLK_PERIOD_NS}"

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

rtl_files=("${PROJ}"/rtl/*.sv)

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
    "${rtl_files[@]}" \
    "${PROJ}/tb/systemc/tb_${SEL_TOP_LEVEL}.cpp" \
    -Mdir "${SIM}/build/obj_dir" \
    -o "${SIM}/build/simv" \
    | tee "${SIM}/output/compile.log"

exec "${SIM}/build/simv" "$@" \
    | tee "${SIM}/output/run.log"
