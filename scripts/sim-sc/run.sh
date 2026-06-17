#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
#
# SystemC simulation flow. If the project has SV sources (any .sv under rtl/),
# they are Verilated into sc_modules and linked against the C++/SystemC harness
# named by SEL_TOP_LEVEL (tb/systemc/tb_<top>.cpp), which becomes sc_main; that
# harness instantiates the SystemC design top (rtl/systemc/<top>.hpp) wiring the
# Verilated SV DUT(s) to native SystemC modules. If the project has no SV (a
# pure-SystemC design), the harness is instead built directly with g++ against
# the SystemC library. Either way it is simulation only -- never synthesized.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${RTL_LAB_HOME}/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
cflags="-std=c++17 -I${PROJ}/tb/systemc -I${PROJ}/rtl/systemc -DCLK_PERIOD_NS=${SEL_CLK_PERIOD_NS}"

if [[ "${SEL_TOP_LEVEL}" == V* ]]; then
    SEL_TOP_LEVEL="${SEL_TOP_LEVEL#V}"
    cflags="${cflags} -DSV"
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
rtl_files=("${PROJ}"/rtl/**/*.sv)
shopt -u nullglob globstar

inc_flags=()
while IFS= read -r d; do
    inc_flags+=(-I"$d")
done < <(find "${PROJ}/rtl" -type d | sort)

vlt_files=()
while IFS= read -r f; do
    vlt_files+=("$f")
done < <(find "${PROJ}/rtl" -name "*.vlt" | sort)

top_flags=()
if [ -n "$(find "${PROJ}/rtl" -name "${SEL_TOP_LEVEL}.sv" -print -quit)" ]; then
    top_flags=(--top-module "${SEL_TOP_LEVEL}")
fi

if [ "${#rtl_files[@]}" -gt 0 ]; then
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
        "${top_flags[@]}" \
        -CFLAGS "${cflags}" \
        "${g_flags[@]}" \
        "${inc_flags[@]}" \
        "${vlt_files[@]}" \
        "${rtl_files[@]}" \
        "${PROJ}/tb/systemc/tb_${SEL_TOP_LEVEL}.cpp" \
        -Mdir "${SIM}/build/obj_dir" \
        -o "${SIM}/build/simv" \
        | tee "${SIM}/output/compile.log"
else
    # shellcheck disable=SC2086
    g++ ${cflags} \
        -I"${SYSTEMC_INCLUDE}" \
        "${PROJ}/tb/systemc/tb_${SEL_TOP_LEVEL}.cpp" \
        -o "${SIM}/build/simv" \
        -L"${SYSTEMC_LIBDIR}" -Wl,-rpath,"${SYSTEMC_LIBDIR}" -lsystemc -pthread \
        | tee "${SIM}/output/compile.log"
fi

exec "${SIM}/build/simv" "$@" \
    | tee "${SIM}/output/run.log"
