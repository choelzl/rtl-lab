#!/usr/bin/env bash
# Cross-checks every stimuli directory under tb/stimuli/ against BOTH
# backends (native TDM and crossbar), via tb_system_stimuli_tdm.cpp /
# tb_system_stimuli_xbar.cpp (see tb/unit/system_stimuli_common.hpp).
#
# A directory counts as a stimuli set if it has at least one ragu_*.log or
# wagu_*.log ("result file"). For each discovered set, both backend binaries
# are run once with SEL_IN_DIR=<set>; both must independently pass the same
# read-after-write correctness check — the only expected difference between
# backends is cycle count (TDM's buffering adds pipeline overhead).
#
# Usage: PROJECT=tdm bash scripts/check_all_stimuli.sh
set -euo pipefail

: "${RTL_LAB_HOME:?RTL_LAB_HOME not set — source sourceme.sh}"
: "${SYSTEMC_INCLUDE:?SYSTEMC_INCLUDE not set}"
: "${SYSTEMC_LIB:?SYSTEMC_LIB not set}"
PROJECT="${PROJECT:-tdm}"
PROJ="${RTL_LAB_HOME}/projects/${PROJECT}"

export SC_COPYRIGHT_MESSAGE=DISABLE

BUILD="${PROJ}/sim/unit/build"
BIN_DIR="${PROJ}/sim/unit"
mkdir -p "${BUILD}" "${BIN_DIR}"

cflags="-std=c++17 -O0 -g -Wall -Wextra -Wno-cpp"
cflags+=" -I${PROJ}/rtl/systemc -I${PROJ}/tb/systemc -I${SYSTEMC_INCLUDE}"
ldflags="-L${SYSTEMC_LIB} -Wl,-rpath,${SYSTEMC_LIB} -lsystemc -pthread"

for name in tb_system_stimuli_tdm tb_system_stimuli_xbar; do
    echo "==> Building ${name} ..."
    # shellcheck disable=SC2086
    g++ $cflags "${PROJ}/tb/unit/${name}.cpp" -o "${BIN_DIR}/${name}" ${ldflags} \
        2>&1 | tee "${BUILD}/${name}.build.log"
done

# Discover stimuli sets: any subdirectory of tb/stimuli/ with a ragu_*.log or
# wagu_*.log result file.
sets=()
for d in "${PROJ}"/tb/stimuli/*/; do
    [[ -n "$(find "$d" -maxdepth 1 \( -name 'ragu_*.log' -o -name 'wagu_*.log' \) -print -quit)" ]] \
        && sets+=("$(basename "$d")")
done
[[ "${#sets[@]}" -gt 0 ]] || { echo "check_all_stimuli: no stimuli sets found under ${PROJ}/tb/stimuli/" >&2; exit 1; }

echo ""
echo "Discovered stimuli sets: ${sets[*]}"

pass=0; fail=0; t_pass=0; t_fail=0

for set in "${sets[@]}"; do
    for name in tb_system_stimuli_tdm tb_system_stimuli_xbar; do
        echo ""
        echo "==> Running ${name} SEL_IN_DIR=${set} ..."
        tmpout=$(mktemp)
        if SEL_IN_DIR="${set}" "${BIN_DIR}/${name}" | tee "${tmpout}"; then
            pass=$((pass + 1))
        else
            echo "  [FAIL] ${name} (${set}) exited non-zero"
            fail=$((fail + 1))
        fi
        t_pass=$((t_pass + $(grep -E '^ *passed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)))
        t_fail=$((t_fail + $(grep -E '^ *failed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)))
        rm -f "${tmpout}"
    done
done

total=$((t_pass + t_fail))
echo ""
echo "=============================================="
printf " Runs:  %d total  |  %d passed  |  %d failed\n" $((pass + fail)) "${pass}" "${fail}"
printf " Tests: %d total  |  %d passed  |  %d failed\n" "${total}" "${t_pass}" "${t_fail}"
echo "=============================================="
[[ "${fail}" -eq 0 ]]
