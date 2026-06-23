#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Cedric Hölzl
#
# Unit test flow.
#
# Discovers and builds every tb_*.cpp under PROJ/tb/unit/. Each binary is run
# in turn; the script exits non-zero if any suite fails or fails to compile.
#
# SV dependencies (optional):
#   Place any .sv files that unit tests need to co-simulate with under
#   PROJ/tb/unit/sv/.  They are Verilated once (--sc --build) before the
#   C++ build step; the resulting headers and objects are linked into every
#   test binary in that project.  Tests with no SV dependency work unchanged.
# -----------------------------------------------------------------------------

set -euo pipefail

PROJ="${RTL_LAB_HOME}/projects/${SEL_PROJECT}"
UNIT_SRC="${PROJ}/tb/unit"
BUILD="${PROJ}/sim/unit/build"
BIN_DIR="${PROJ}/sim/unit"

mkdir -p "${BUILD}" "${BIN_DIR}"

cflags="-std=c++17 -O0 -g -Wall -Wextra -Wno-cpp"
cflags="${cflags} -I${PROJ}/rtl/systemc"
cflags="${cflags} -I${PROJ}/tb/systemc"
cflags="${cflags} -I${SYSTEMC_INCLUDE}"
ldflags="-L${SYSTEMC_LIBDIR} -Wl,-rpath,${SYSTEMC_LIBDIR} -lsystemc -pthread"

if [ "${SEL_PARAMS:-none}" != "none" ]; then
    for param in ${SEL_PARAMS}; do
        cflags="${cflags} -D${param}"
    done
fi

# ---- Optional: Verilate SV stubs placed in tb/unit/sv/ ----
extra_inc=""
extra_objs=()

sv_dir="${UNIT_SRC}/sv"
if [ -d "${sv_dir}" ]; then
    shopt -s nullglob
    sv_files=("${sv_dir}"/*.sv)
    shopt -u nullglob

    if [ "${#sv_files[@]}" -gt 0 ]; then
        obj_dir="${BUILD}/obj_dir"
        echo "==> Verilating SV sources in tb/unit/sv/ ..."
        verilator --sc --build -sv -Wall -Wno-fatal \
            "${sv_files[@]}" \
            -Mdir "${obj_dir}" \
            2>&1 | tee "${BIN_DIR}/verilate.log"

        extra_inc="-I${obj_dir}"
        while IFS= read -r o; do
            extra_objs+=("$o")
        done < <(find "${obj_dir}" -name "*.o" | sort)
    fi
fi

# ---- Build and run each tb_*.cpp ----
pass=0; fail=0
total_tests_pass=0; total_tests_fail=0

if [ -n "${SEL_TOP_LEVEL:-}" ]; then
    srcs=("${UNIT_SRC}/tb_${SEL_TOP_LEVEL}.cpp")
    if [ ! -f "${srcs[0]}" ]; then
        echo "No test source found: ${srcs[0]}"
        exit 1
    fi
else
    shopt -s nullglob
    srcs=("${UNIT_SRC}"/tb_*.cpp)
    shopt -u nullglob
    if [ "${#srcs[@]}" -eq 0 ]; then
        echo "No unit test sources found in ${UNIT_SRC}"
        exit 1
    fi
fi

for src in "${srcs[@]}"; do
    name=$(basename "${src}" .cpp)
    bin="${BIN_DIR}/${name}"
    tmpout=$(mktemp)

    echo ""
    echo "==> Building ${name} ..."
    # shellcheck disable=SC2086
    if g++ ${cflags} ${extra_inc} "${src}" \
            ${extra_objs:+"${extra_objs[@]}"} \
            -o "${bin}" ${ldflags} \
            2>&1 | tee "${BUILD}/${name}.build.log"; then
        echo "==> Running ${name} ..."
        if "${bin}" | tee "${tmpout}"; then
            pass=$((pass + 1))
        else
            echo "  [FAIL] ${name} exited non-zero"
            fail=$((fail + 1))
        fi
        t_pass=$(grep -E '^ *passed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)
        t_fail=$(grep -E '^ *failed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)
        total_tests_pass=$((total_tests_pass + t_pass))
        total_tests_fail=$((total_tests_fail + t_fail))
    else
        echo "  [FAIL] ${name} failed to compile (see ${BUILD}/${name}.build.log)"
        fail=$((fail + 1))
    fi

    rm -f "${tmpout}"
done

total_tests=$((total_tests_pass + total_tests_fail))
echo ""
echo "=============================================="
printf " Suites: %d total  |  %d passed  |  %d failed\n" \
    $((pass + fail)) "${pass}" "${fail}"
printf " Tests:  %d total  |  %d passed  |  %d failed\n" \
    "${total_tests}" "${total_tests_pass}" "${total_tests_fail}"
echo "=============================================="

[ "${fail}" -eq 0 ]
