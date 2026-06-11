#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${RTL_LAB_HOME}/projects/${SEL_PROJECT}"
SIM="${PROJ}/sim/${SEL_OUT_DIR}"

g_flags=()
if [ "${SEL_PARAMS}" != "none" ]; then
    for param in ${SEL_PARAMS}; do
        g_flags+=("-G${param}")
    done
fi

# Collect -I flags including any lib subdirectory trees that exist
inc_flags=(-I"${PROJ}/rtl" -I"${PROJ}/rtl/sv")
for d in "${PROJ}"/rtl/sv/lib "${PROJ}"/rtl/sv/lib/*/; do
    [ -d "$d" ] && inc_flags+=(-I"$d")
done

verilator \
    -sv \
    --binary \
    --timing \
    --trace \
    --trace-max-array 0 \
    --trace-max-width 0 \
    -Wall \
    -Wno-fatal \
    -Wno-UNUSEDPARAM \
    -DVCD \
    -DCLK_PERIOD_NS="${SEL_CLK_PERIOD_NS}" \
    "${g_flags[@]}" \
    "${inc_flags[@]}" \
    --top-module "tb_${SEL_TOP_LEVEL}" \
    "${PROJ}/tb/tb_${SEL_TOP_LEVEL}.sv" \
    -Mdir "${SIM}/build/obj_dir" \
    -o "${SIM}/build/simv" \
    | tee "${SIM}/output/compile.log"

exec "${RTL_LAB_HOME}/projects/${SEL_PROJECT}/sim/${SEL_OUT_DIR}/build/simv" "$@" \
    | tee "${RTL_LAB_HOME}/projects/${SEL_PROJECT}/sim/${SEL_OUT_DIR}/output/run.log"
