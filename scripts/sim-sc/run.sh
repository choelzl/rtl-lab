#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
#
# SystemC simulation flow. If the project has SV sources (rtl/*.sv), they are
# Verilated into sc_modules and linked against the C++/SystemC harness named by
# SEL_TOP_LEVEL (tb/systemc/tb_<top>.cpp), which becomes sc_main; that harness
# instantiates the SystemC design top (rtl/systemc/<top>.hpp) wiring the
# Verilated SV DUT(s) to native SystemC modules. If the project has no SV (a
# pure-SystemC design), the harness is instead built directly with g++ against
# the SystemC library. Either way it is simulation only -- never synthesized.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${RTL_LAB_HOME}/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
cflags="-std=c++17 -I${PROJ}/tb/systemc -I${PROJ}/rtl/systemc -DCLK_PERIOD_NS=${SEL_CLK_PERIOD_NS}"

# Verilated SV DUT: TOP_LEVEL=V<module> selects the Verilated backend.
# Strip the leading V to get the SV module name and the sc_main harness name.
verilator_top_flags=()
harness_top="${SEL_TOP_LEVEL}"
if [[ "${SEL_TOP_LEVEL}" == V* ]]; then
    sv_top="${SEL_TOP_LEVEL#V}"
    harness_top="${sv_top}"
    cflags="${cflags} -DUSE_SV_DUT"
    verilator_top_flags=("--top" "${sv_top}")
fi

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

shopt -s nullglob globstar
rtl_files=("${PROJ}"/rtl/*.sv "${PROJ}"/rtl/sv/**/*.sv)
shopt -u nullglob globstar

# Collect -I flags for any lib subdirectory trees that exist
inc_flags=(-I"${PROJ}/rtl" -I"${PROJ}/rtl/sv")
for d in "${PROJ}"/rtl/sv/lib "${PROJ}"/rtl/sv/lib/*/; do
    [ -d "$d" ] && inc_flags+=(-I"$d")
done

if [[ "${SEL_TOP_LEVEL}" == V* ]]; then
    verilator \
        --sc \
        --exe \
        -sv \
        --trace \
        --trace-max-array 0 \
        --trace-max-width 0 \
        -Wall \
        -Wno-fatal \
        -Wno-UNUSEDPARAM \
        "${verilator_top_flags[@]}" \
        -CFLAGS "${cflags}" \
        "${g_flags[@]}" \
        "${inc_flags[@]}" \
        "${rtl_files[@]}" \
        "${PROJ}/tb/systemc/tb_${harness_top}.cpp" \
        -Mdir "${SIM}/build/obj_dir" \
        -o "${SIM}/build/simv" \
        2>&1 | tee "${SIM}/output/compile.log"
    MAKEFLAGS= make -C "${SIM}/build/obj_dir" \
        -f "V${harness_top}.mk" \
        2>&1 | tee -a "${SIM}/output/compile.log"
else
    # shellcheck disable=SC2086
    g++ ${cflags} \
        -I"${SYSTEMC_INCLUDE}" \
        "${PROJ}/tb/systemc/tb_${harness_top}.cpp" \
        -o "${SIM}/build/simv" \
        -L"${SYSTEMC_LIBDIR}" -Wl,-rpath,"${SYSTEMC_LIBDIR}" -lsystemc -pthread \
        | tee "${SIM}/output/compile.log"
fi

exec "${SIM}/build/simv" "$@" \
    | tee "${SIM}/output/run.log"
