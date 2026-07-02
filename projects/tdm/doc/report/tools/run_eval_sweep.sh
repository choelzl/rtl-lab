#!/bin/bash
# Runs the full §5 evaluation sweep: final/0-19 x {crossbar, TDM-adaptive,
# TDM-RR(8-slot)} x {fenced, unfenced}, then extract_eval.py turns the
# stats.log files into doc/report/data/eval_*.csv.
#
# Usage (from projects/tdm/, environment sourced):
#   bash doc/report/tools/run_eval_sweep.sh [OUT_DIR]
#
# The final/N stimuli are proprietary and not in the repository — place them
# under tb/stimuli/final/{0..19}/ first (see tb/stimuli/final/.gitignore).
set -e
OUT=${1:-/tmp/tdm_eval_sweep}
PROJ=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$PROJ"

CFLAGS="-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I$SYSTEMC_INCLUDE"
LFLAGS="-L$SYSTEMC_LIB -Wl,-rpath,$SYSTEMC_LIB -lsystemc -pthread"
BIN=$OUT/bin
mkdir -p "$BIN"
# The evaluation crossbar includes the L1 hash repair (report §2.1 / Appendix A.8).
g++ $CFLAGS -DIMPL_CROSSBAR -DXBAR_HASH_L1     tb/systemc/tb_top.cpp -o "$BIN/cb"   $LFLAGS &
g++ $CFLAGS -DIMPL_TDM -DIMPL_ARB_ADAPTIVE     tb/systemc/tb_top.cpp -o "$BIN/adap" $LFLAGS &
g++ $CFLAGS -DIMPL_TDM                         tb/systemc/tb_top.cpp -o "$BIN/rr"   $LFLAGS &
wait

# Evaluation configuration (report Appendix A.7): no per-cycle CSV logs,
# hidden-lookahead lead 16; the RR build programs its 8-slot table itself.
export RTL_LAB_HOME=${RTL_LAB_HOME:-$(cd "$PROJ/../.." && pwd)}
export SEL_NO_MONITOR=1 SEL_LA_LEAD=16

run_one() {
  set=$1; bin=$2; tag=$3; nofence=$4
  d=$OUT/${tag}_${set}
  mkdir -p "$d"; cd "$d"
  if [ "$nofence" = "1" ]; then export SEL_NO_FENCE=1; else unset SEL_NO_FENCE; fi
  SEL_IN_DIR="$PROJ/tb/stimuli/final/$set" "$BIN/$bin" > out.log 2>&1
  echo "done $tag $set"
}
export -f run_one; export OUT BIN PROJ
for s in $(seq 0 19); do
  echo "$s cb cb 0";   echo "$s adap adap 0";   echo "$s rr rr8 0"
  echo "$s cb cbnf 1"; echo "$s adap adapnf 1"; echo "$s rr rr8nf 1"
done | xargs -P "$(( $(nproc) > 12 ? 10 : $(nproc) - 2 ))" -n 4 bash -c 'run_one "$@"' _

python3 "$PROJ/doc/report/tools/extract_eval.py" "$OUT"
echo "eval CSVs written to doc/report/data/; regenerate with python3 doc/report/gen_report.py"
