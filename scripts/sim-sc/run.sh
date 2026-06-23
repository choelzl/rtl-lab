#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hölzl
#
# SystemC simulation flow.  Testbench: tb/systemc/tb_${TOP_LEVEL}.cpp.
#
# Build mode:
#   SV files present  — Verilator (--sc --exe).
#   No SV files       — g++ directly against the SystemC library.
#
# IMPL (optional) is uppercased and forwarded as -DIMPL_<UPPER> so project
# headers can respond with #ifdef guards.  Its meaning is project-defined.
# Native SystemC IMPL values can coexist with SV helper files in rtl/.
# Only IMPL values that name top-level SV backends should drive Verilator's
# --top-module; native backends leave the SV top unspecified unless TOP_LEVEL
# itself names an SV module.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${RTL_LAB_HOME}/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
cflags="-std=c++17 -I${PROJ}/tb/systemc -I${PROJ}/rtl/systemc -DCLK_PERIOD_NS=${SEL_CLK_PERIOD_NS}"

if [ "${SEL_IMPL:-none}" != "none" ]; then
    impl_flag=$(echo "${SEL_IMPL}" | tr '[:lower:]' '[:upper:]' | tr '-' '_')
    cflags="${cflags} -DIMPL_${impl_flag}"
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

# The TDM project has both native SystemC and SV-backed implementations in one
# rtl tree.  Native IMPL values should build directly with g++; top_* values use
# Verilator and select the matching SV top.
if [ "${SEL_PROJECT}" = "tdm" ]; then
    case "${SEL_IMPL:-none}" in
        none|crossbar|tdm|tdm_sc)
            rtl_files=()
            ;;
    esac
fi

inc_flags=()
while IFS= read -r d; do
    inc_flags+=(-I"$d")
done < <(find "${PROJ}/rtl" -type d | sort)

vlt_files=()
while IFS= read -r f; do
    vlt_files+=("$f")
done < <(find "${PROJ}/rtl" -name "*.vlt" | sort)

top_flags=()
case "${SEL_IMPL:-none}" in
    top_*)
        if [ -n "$(find "${PROJ}/rtl" -name "${SEL_IMPL}.sv" -print -quit)" ]; then
            top_flags=(--top-module "${SEL_IMPL}")
        fi
        ;;
    none)
        if [ -n "$(find "${PROJ}/rtl" -name "${SEL_TOP_LEVEL}.sv" -print -quit)" ]; then
            top_flags=(--top-module "${SEL_TOP_LEVEL}")
        fi
        ;;
esac

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
