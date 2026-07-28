#!/bin/bash
# Reruns report Tables E8/E10's ragu_a-isolated crossbar-hash-scheme sweep
# against tb/stimuli/final/patroklos1 (the corrected 20-set patroklos export)
# instead of the older tb/stimuli/final/patroklos. Isolation: for each set,
# only ragu_a.csv is copied into a scratch stimuli dir (ragu_b/ragu_c/wagu_a
# are absent there, same "no stimuli, will be idle" path already used for
# ragu_d/e/wagu_b/d/e, which patroklos/patroklos1 never populate either) --
# so ragu_a's own per-group stats.log fields are its self-collision behavior
# alone, with no cross-AGU or read/write contention. Both fenced (E8) and
# SEL_NO_FENCE=1 (E10) sweeps run in one pass.
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

# variant tag -> (extra -D flags beyond -DIMPL_CROSSBAR, stimuli sub-dir)
declare -A VARIANT_FLAGS=(
  [baseline]=""
  [l1]="-DXBAR_HASH_L1"
  [l1v2]="-DXBAR_HASH_L1_V2"
  [hash16]="-DXBAR_HASH16"
  [hash16c]="-DXBAR_HASH16 -DXBAR_HASH_L2_COMPOSE"
  [hash32]="-DXBAR_HASH32"
  [hash32c]="-DXBAR_HASH32 -DXBAR_HASH_L2_COMPOSE"
)
declare -A VARIANT_SRC=(
  [baseline]=unpadded [l1]=unpadded [l1v2]=unpadded
  [hash16]=padded16 [hash16c]=padded16
  [hash32]=padded32 [hash32c]=padded32
)
SRC_ROOT="$PROJ/tb/stimuli/final/patroklos1"
declare -A SRC_DIR=(
  [unpadded]="$SRC_ROOT/stimuli_2026_07_27_unpadded"
  [padded16]="$SRC_ROOT/stimuli_2026_07_27_padded16"
  [padded32]="$SRC_ROOT/stimuli_2026_07_27_padded32"
)

for tag in "${!VARIANT_FLAGS[@]}"; do
  g++ $CFLAGS -DIMPL_CROSSBAR ${VARIANT_FLAGS[$tag]} tb/systemc/tb_top.cpp \
    -o "$BIN/$tag" $LFLAGS &
done
wait

export RTL_LAB_HOME=${RTL_LAB_HOME:-$(cd "$PROJ/../.." && pwd)}
export SEL_NO_MONITOR=1

# Isolated per-set stimuli: only ragu_a.csv, copied once per (source, set) --
# shared across variants that use the same source (baseline/l1/l1v2 all read
# "unpadded"), not regenerated per variant.
ISO=$OUT/iso
mkdir -p "$ISO"
for src in "${!SRC_DIR[@]}"; do
  for s in $(seq 0 19); do
    d="$ISO/${src}_${s}"
    mkdir -p "$d"
    cp "${SRC_DIR[$src]}/$s/ragu_a.csv" "$d/ragu_a.csv"
  done
done

run_one() {
  variant=$1; src=$2; s=$3; mode=$4
  d="$OUT/${variant}_${mode}_${s}"
  mkdir -p "$d"; cd "$d"
  if [ "$mode" = "nofence" ]; then export SEL_NO_FENCE=1; else unset SEL_NO_FENCE; fi
  SEL_IN_DIR="$ISO/${src}_${s}" "$BIN/$variant" > out.log 2>&1
  echo "done $variant $mode $s"
}
export -f run_one
export OUT BIN ISO

for tag in "${!VARIANT_FLAGS[@]}"; do
  src=${VARIANT_SRC[$tag]}
  for s in $(seq 0 19); do
    echo "$tag $src $s fenced"
    echo "$tag $src $s nofence"
  done
done | xargs -P "$(( $(nproc) > 12 ? 10 : $(nproc) - 2 ))" -n 4 bash -c 'run_one "$@"' _

python3 "$PROJ/doc/report/tools/extract_ragu_a_isolation.py" "$OUT"
echo "ragu_a-isolation sweep CSVs written to doc/report/data/"
