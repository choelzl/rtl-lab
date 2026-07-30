#!/bin/bash
# Reruns report Table E7's ragu_a-isolated crossbar-hash-scheme sweep against
# tb/stimuli/final/patroklos2. Isolation: for each set, only ragu_a's own
# trace is copied into a scratch stimuli dir (ragu_b/ragu_c/wagu_a are absent
# there, same "no stimuli, will be idle" path already used for
# ragu_d/e/wagu_b/d/e, which this suite never populates either) -- so
# ragu_a's own per-group stats.log fields are its self-collision behavior
# alone, with no cross-AGU or read/write contention. Both fenced and
# SEL_NO_FENCE=1 sweeps run in one pass (only the fenced one currently has
# its own report table; the unfenced numbers are cited inline in prose).
# XBAR_HASH_L2_COMPOSE variants are not built here: they're a standing
# regression check on addr_hash_inv(), provably identical to the plain
# HASH16/32 row -- redundant for this sweep, not worth the build/run cost.
#
# Usage (from projects/tdm/, environment sourced -- see sourceme.sh):
#   bash doc/report/tools/run_ragu_a_isolation_sweep.sh [OUT_DIR]
set -e
OUT=${1:-/tmp/tdm_ragu_a_isolation}
PROJ=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$PROJ"

CFLAGS="-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I$SYSTEMC_INCLUDE"
LFLAGS="-L$SYSTEMC_LIB -Wl,-rpath,$SYSTEMC_LIB -lsystemc -pthread"
BIN=$OUT/bin
mkdir -p "$BIN"

# Binary tag -> extra -D flags beyond -DIMPL_CROSSBAR. One binary per hash
# scheme; l1v2 is reused for both its unpadded and padded32 rows below (the
# scheme doesn't change, only which stimuli it's pointed at) rather than
# rebuilt identically twice. XBAR_HASH_L1_V2 is the R/C/napa-keyed
# fold-length rule (formerly tracked separately as XBAR_HASH_L1_V2_ALT until
# it measured at least as good as the old fixed two-path dispatch — see
# addr_hash.hpp's own comment).
declare -A VARIANT_FLAGS=(
  [baseline]=""
  [l1]="-DXBAR_HASH_L1"
  [l1v2]="-DXBAR_HASH_L1_V2"
  [hash16]="-DXBAR_HASH16"
  [hash32]="-DXBAR_HASH32"
)
# Variant tag (one row in the output table) -> (binary tag, stimuli sub-dir).
# baseline_p32 reuses the baseline binary against padded32 (same convention as
# l1v2_p32: same scheme, different stimuli, not a rebuild).
declare -A VARIANT_BIN=(
  [baseline]=baseline [baseline_p32]=baseline [l1]=l1 [l1v2]=l1v2 [l1v2_p32]=l1v2
  [hash16]=hash16 [hash32]=hash32
)
declare -A VARIANT_SRC=(
  [baseline]=unpadded [baseline_p32]=padded32 [l1]=unpadded [l1v2]=unpadded [l1v2_p32]=padded32
  [hash16]=padded16
  [hash32]=padded32
)
SRC_ROOT="$PROJ/tb/stimuli/final/patroklos2"
declare -A SRC_DIR=(
  [unpadded]="$SRC_ROOT/stimuli_2026_07_28_unpadded"
  [padded16]="$SRC_ROOT/stimuli_2026_07_28_padded16"
  [padded32]="$SRC_ROOT/stimuli_2026_07_28_padded32"
)

for tag in "${!VARIANT_FLAGS[@]}"; do
  g++ $CFLAGS -DIMPL_CROSSBAR ${VARIANT_FLAGS[$tag]} tb/systemc/tb_top.cpp \
    -o "$BIN/$tag" $LFLAGS &
done
wait

export RTL_LAB_HOME=${RTL_LAB_HOME:-$(cd "$PROJ/../.." && pwd)}
export SEL_NO_MONITOR=1

# Isolated per-set stimuli: only ragu_a's own trace, copied once per (source,
# set) -- shared across variants that use the same source (baseline/l1/l1v2
# all read "unpadded"), not regenerated per variant. Extension varies by
# export (.csv or .log; agu.hpp's resolve_stim_path() tries both either way)
# so this tries both rather than hardcoding one.
ISO=$OUT/iso
mkdir -p "$ISO"
for src in "${!SRC_DIR[@]}"; do
  for s in $(seq 0 19); do
    d="$ISO/${src}_${s}"
    mkdir -p "$d"
    srcfile="${SRC_DIR[$src]}/$s/ragu_a.csv"
    [ -f "$srcfile" ] || srcfile="${SRC_DIR[$src]}/$s/ragu_a.log"
    cp "$srcfile" "$d/ragu_a.csv"
  done
done

run_one() {
  variant=$1; bin=$2; src=$3; s=$4; mode=$5
  d="$OUT/${variant}_${mode}_${s}"
  mkdir -p "$d"; cd "$d"
  if [ "$mode" = "nofence" ]; then export SEL_NO_FENCE=1; else unset SEL_NO_FENCE; fi
  SEL_IN_DIR="$ISO/${src}_${s}" "$BIN/$bin" > out.log 2>&1
  echo "done $variant $mode $s"
}
export -f run_one
export OUT BIN ISO

for tag in "${!VARIANT_SRC[@]}"; do
  bin=${VARIANT_BIN[$tag]}
  src=${VARIANT_SRC[$tag]}
  for s in $(seq 0 19); do
    echo "$tag $bin $src $s fenced"
    echo "$tag $bin $src $s nofence"
  done
done | xargs -P "$(( $(nproc) > 12 ? 10 : $(nproc) - 2 ))" -n 5 bash -c 'run_one "$@"' _

python3 "$PROJ/doc/report/tools/extract_ragu_a_isolation.py" "$OUT"
echo "ragu_a-isolation sweep CSVs written to doc/report/data/"
