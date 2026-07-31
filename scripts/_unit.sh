#!/usr/bin/env bash
# Internal helper — invoked by `edaf unit`. Not meant to be called directly.
#
# Discovers and builds every tb_*.cpp under $PROJ/tb/unit/.
# Each binary is run in turn; exits non-zero if any suite fails to compile or run.
#
# SV stubs (optional):
#   Place .sv files that unit tests need to co-simulate with under tb/unit/sv/.
#   They are Verilated once (--sc --build) before the C++ build; the resulting
#   headers and objects are linked into every test binary in that project.
set -euo pipefail

export SC_COPYRIGHT_MESSAGE=DISABLE

UNIT_SRC="${PROJ}/tb/unit"
BUILD="${PROJ}/sim/unit/build"
BIN_DIR="${PROJ}/sim/unit"
mkdir -p "${BUILD}" "${BIN_DIR}"

cflags="-std=c++17 -O0 -g -Wall -Wextra -Wno-cpp"
cflags+=" -I${PROJ}/rtl/systemc"
cflags+=" -I${PROJ}/tb/systemc"
cflags+=" -I${SYSTEMC_INCLUDE}"
ldflags="-L${SYSTEMC_LIB} -Wl,-rpath,${SYSTEMC_LIB} -lsystemc -pthread"

# PARAMS: FLAG → -DFLAG, K=V → -DK=V (no -G in unit tests)
if [[ -n "${PARAMS:-}" ]]; then
    IFS=',' read -ra _ps <<< "$PARAMS"
    for p in "${_ps[@]}"; do cflags+=" -D${p}"; done
fi

# ── Optional: Verilate SV stubs under tb/unit/sv/ ────────────────────────────
extra_inc="" extra_objs=()
sv_dir="${UNIT_SRC}/sv"
if [[ -d "$sv_dir" ]]; then
    shopt -s nullglob
    sv_files=("${sv_dir}"/*.sv)
    shopt -u nullglob
    if [[ "${#sv_files[@]}" -gt 0 ]]; then
        obj_dir="${BUILD}/obj_dir"
        echo "==> Verilating SV stubs in tb/unit/sv/ ..."
        verilator --sc --build -sv -Wall -Wno-fatal \
            "${sv_files[@]}" \
            -Mdir "${obj_dir}" \
            2>&1 | tee "${BIN_DIR}/verilate.log"
        extra_inc="-I${obj_dir}"
        while IFS= read -r o; do extra_objs+=("$o"); done \
            < <(find "${obj_dir}" -name "*.o" | sort)
    fi
fi

# ── Select test sources ───────────────────────────────────────────────────────
if [[ -n "${TOP:-}" ]]; then
    srcs=("${UNIT_SRC}/tb_${TOP}.cpp")
    [[ -f "${srcs[0]}" ]] || { echo "rtlf unit: no test source: ${srcs[0]}" >&2; exit 1; }
else
    shopt -s nullglob
    srcs=("${UNIT_SRC}"/tb_*.cpp)
    shopt -u nullglob
    [[ "${#srcs[@]}" -gt 0 ]] || { echo "rtlf unit: no tb_*.cpp found in ${UNIT_SRC}" >&2; exit 1; }
fi

# ── Build and run ─────────────────────────────────────────────────────────────
pass=0; fail=0; t_pass=0; t_fail=0

for src in "${srcs[@]}"; do
    name=$(basename "${src}" .cpp)
    bin="${BIN_DIR}/${name}"
    echo ""
    echo "==> Building ${name} ..."
    # shellcheck disable=SC2086
    if g++ $cflags ${extra_inc} "${src}" \
            ${extra_objs:+"${extra_objs[@]}"} \
            -o "${bin}" ${ldflags} \
            2>&1 | tee "${BUILD}/${name}.build.log"; then
        echo "==> Running ${name} ..."
        tmpout=$(mktemp)
        if "${bin}" | tee "${tmpout}"; then
            pass=$((pass + 1))
        else
            echo "  [FAIL] ${name} exited non-zero"
            fail=$((fail + 1))
        fi
        t_pass=$((t_pass + $(grep -E '^ *passed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)))
        t_fail=$((t_fail + $(grep -E '^ *failed:' "${tmpout}" | tail -1 | grep -oE '[0-9]+' || echo 0)))
        rm -f "${tmpout}"
    else
        echo "  [FAIL] ${name} failed to compile (see ${BUILD}/${name}.build.log)"
        fail=$((fail + 1))
    fi
done

total=$((t_pass + t_fail))
echo ""
echo "=============================================="
printf " Suites: %d total  |  %d passed  |  %d failed\n" $((pass + fail)) "${pass}" "${fail}"
printf " Tests:  %d total  |  %d passed  |  %d failed\n" "${total}" "${t_pass}" "${t_fail}"
echo "=============================================="
[[ "${fail}" -eq 0 ]]
