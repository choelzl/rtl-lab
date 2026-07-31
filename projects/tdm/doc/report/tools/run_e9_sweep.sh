#!/bin/bash
# Reruns report Table E8's full-traffic ragu_a hash-scheme comparison, and
# (SEL_NO_FENCE=1, feeding the new Table E9) the same full-traffic mix run
# throughput-bound instead of on the descriptors' own schedule. This
# environment has no proprietary tb/stimuli/final/{0..19} export, so every
# row substitutes patroklos2 instead -- each scheme against its own matching
# set, same convention as Table E7 (unpadded for
# baseline/XBAR_HASH_L1/XBAR_HASH_L1_V2, padded16 for XBAR_HASH16, padded32
# for XBAR_HASH32) -- but run with ALL of that set's stimuli present (ragu_a,
# ragu_b, ragu_c, wagu_a together, real cross-AGU/read-vs-write contention),
# not ragu_a-isolated like Table E7.
#
# Usage (from projects/tdm/, environment sourced -- see sourceme.sh):
#   bash doc/report/tools/run_e9_sweep.sh [OUT_DIR]
set -e
OUT=${1:-/tmp/tdm_e9_sweep}
PROJ=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$PROJ"

CFLAGS="-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I$SYSTEMC_INCLUDE"
LFLAGS="-L$SYSTEMC_LIB -Wl,-rpath,$SYSTEMC_LIB -lsystemc -pthread"
BIN=$OUT/bin
mkdir -p "$BIN"

# Binary tag -> extra -D flags. l1v2 is reused for both its unpadded and
# padded32 rows below (same scheme, different stimuli) rather than rebuilt
# identically twice -- see run_ragu_a_isolation_sweep.sh's VARIANT_BIN.
declare -A VARIANT_FLAGS=(
  [baseline]=""
  [l1]="-DXBAR_HASH_L1"
  [l1v2]="-DXBAR_HASH_L1_V2"
  [l1v3]="-DXBAR_HASH_L1_V3"
  [rob_d2]="-DXBAR_ROB -DXBAR_ROB_DEPTH=2 -DXBAR_HASH_L1_V3"
  [rob_d4]="-DXBAR_ROB -DXBAR_ROB_DEPTH=4 -DXBAR_HASH_L1_V3"
  [poly]="-DXBAR_HASH_POLY"
  [rob_poly_d4]="-DXBAR_ROB -DXBAR_ROB_DEPTH=4 -DXBAR_HASH_POLY"
  [hash16]="-DXBAR_HASH16"
  [hash32]="-DXBAR_HASH32"
)
declare -A VARIANT_BIN=(
  [baseline]=baseline [l1]=l1 [l1v2]=l1v2 [l1v3]=l1v3
  [rob_d2]=rob_d2 [rob_d4]=rob_d4
  [poly]=poly [rob_poly_d4]=rob_poly_d4
  [hash16]=hash16 [hash32]=hash32
)
declare -A VARIANT_SRC=(
  [baseline]=unpadded [l1]=unpadded [l1v2]=unpadded [l1v3]=unpadded
  [rob_d2]=unpadded [rob_d4]=unpadded
  [poly]=unpadded [rob_poly_d4]=unpadded
  [hash16]=padded16 [hash32]=padded32
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
# See run_ragu_a_isolation_sweep.sh: lead 16 is the evaluation convention;
# only the XBAR_ROB variants consume it.
export SEL_LA_LEAD=16

# Full-traffic stimuli per source: every file that set has (ragu_a, ragu_b,
# ragu_c, wagu_a -- patroklos2 never populates ragu_d/e or wagu_b/d/e),
# copied once per (source, set) and shared across variants that read the
# same source. Extension varies by export (.csv or .log; agu.hpp's
# resolve_stim_path() tries both either way).
ISO=$OUT/iso
mkdir -p "$ISO"
for src in "${!SRC_DIR[@]}"; do
  for s in $(seq 0 19); do
    d="$ISO/${src}_${s}"
    mkdir -p "$d"
    for f in ragu_a ragu_b ragu_c wagu_a; do
      srcfile="${SRC_DIR[$src]}/$s/$f.csv"
      [ -f "$srcfile" ] || srcfile="${SRC_DIR[$src]}/$s/$f.log"
      [ -f "$srcfile" ] && cp "$srcfile" "$d/$f.csv"
    done
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

python3 "$PROJ/doc/report/tools/extract_e9.py" "$OUT"
echo "E9 sweep CSV written to doc/report/data/"
