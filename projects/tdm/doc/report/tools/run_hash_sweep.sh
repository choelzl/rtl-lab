#!/bin/bash
# Runs the crossbar-hash-variant comparison (report §5.4-5.6): four crossbar
# builds -- default (no macro), XBAR_HASH_L1, XBAR_HASH_L1_V2, XBAR_HASH16 --
# each against three 20-set stimuli sources (final, final_tst/pad,
# final_tst/unpad), then extract_hash_sweep.py turns the resulting stats.log
# files into doc/report/data/eval_xbar_hash_sweep*.csv.
#
# Usage (from projects/tdm/, environment sourced -- see sourceme.sh):
#   bash doc/report/tools/run_hash_sweep.sh [OUT_DIR]
#
# Binaries are compiled once per variant (not once per stimuli set) and run
# from a fixed path: running `make sim-crossbar` concurrently for different
# PARAMS races on the same simv binary path ("Text file busy"), which is why
# this compiles four distinct binaries up front instead.
set -e
OUT=${1:-/tmp/tdm_hash_sweep}
PROJ=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$PROJ"

CFLAGS="-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I$SYSTEMC_INCLUDE"
LFLAGS="-L$SYSTEMC_LIB -Wl,-rpath,$SYSTEMC_LIB -lsystemc -pthread"
BIN=$OUT/bin
mkdir -p "$BIN"

# variant tag -> extra -D flags (beyond -DIMPL_CROSSBAR, always on)
declare -A VARIANTS=(
  [default]=""
  [l1]="-DXBAR_HASH_L1"
  [l1v2]="-DXBAR_HASH_L1_V2"
  [hash16]="-DXBAR_HASH16"
  [hash32]="-DXBAR_HASH32"
)
for tag in "${!VARIANTS[@]}"; do
  g++ $CFLAGS -DIMPL_CROSSBAR ${VARIANTS[$tag]} tb/systemc/tb_top.cpp \
    -o "$BIN/$tag" $LFLAGS &
done
wait

# source tag -> stimuli root (each has numbered 0..19 subdirs)
declare -A SOURCES=(
  [final]="$PROJ/tb/stimuli/final"
  [pad]="$PROJ/tb/stimuli/final_tst/pad"
  [unpad]="$PROJ/tb/stimuli/final_tst/unpad"
)

export RTL_LAB_HOME=${RTL_LAB_HOME:-$(cd "$PROJ/../.." && pwd)}
export SEL_NO_MONITOR=1

run_one() {
  variant=$1; source_tag=$2; source_dir=$3; set_i=$4
  d="$OUT/${variant}_${source_tag}_${set_i}"
  mkdir -p "$d"; cd "$d"
  SEL_IN_DIR="$source_dir/$set_i" "$BIN/$variant" > out.log 2>&1
  echo "done $variant $source_tag $set_i"
}
export -f run_one
export OUT BIN

for tag in "${!VARIANTS[@]}"; do
  for src in "${!SOURCES[@]}"; do
    for s in $(seq 0 19); do
      echo "$tag $src ${SOURCES[$src]} $s"
    done
  done
done | xargs -P "$(( $(nproc) > 12 ? 10 : $(nproc) - 2 ))" -n 4 bash -c 'run_one "$@"' _

python3 "$PROJ/doc/report/tools/extract_hash_sweep.py" "$OUT"
echo "hash-sweep CSVs written to doc/report/data/"
