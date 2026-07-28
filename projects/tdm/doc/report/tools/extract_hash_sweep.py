#!/usr/bin/env python3
"""Turns a crossbar-hash-variant sweep's stats.log files (run_hash_sweep.sh)
into doc/report/data/eval_xbar_hash_sweep*.csv and a printed summary table.
Usage:
    python3 extract_hash_sweep.py <sweep-output-dir>
Expects <dir>/{variant}_{source}_{set}/stats.log for variant in
{default,l1,l1v2,hash16,hash32}, source in {final,pad,unpad}, set in 0..19."""
import csv
import sys
from pathlib import Path

V = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tdm_hash_sweep")
DATA = Path(__file__).resolve().parent.parent / "data"

VARIANTS = ["default", "l1", "l1v2", "hash16", "hash32"]
SOURCES = ["final", "pad", "unpad"]
NSETS = 20


def stats(variant, source, s):
    d = {}
    path = V / f"{variant}_{source}_{s}" / "stats.log"
    for line in path.read_text().splitlines():
        k, _, v = line.partition(",")
        d[k] = int(v)
    return d


# Per-(variant, source, set) row: total wait-cycles (port_wait, matching the
# "wait N req-cycles" printed stat) and the per-level Method-2 breakdown
# (lvl_rd_X + lvl_wr_X, same convention as extract_eval.py / report §5.5).
rows = []
missing = []
for variant in VARIANTS:
    for source in SOURCES:
        for s in range(NSETS):
            try:
                d = stats(variant, source, s)
            except FileNotFoundError:
                missing.append(f"{variant}_{source}_{s}")
                continue
            l1 = d.get("lvl_rd_l1", 0) + d.get("lvl_wr_l1", 0)
            l2 = d.get("lvl_rd_l2", 0) + d.get("lvl_wr_l2", 0)
            l3 = d.get("lvl_rd_l3", 0) + d.get("lvl_wr_l3", 0)
            rows.append({
                "variant": variant, "source": source, "set": s,
                "actual_cycles": d["actual_cycles"],
                "wait_cycles": d["port_wait"],
                "lvl_l1": l1, "lvl_l2": l2, "lvl_l3": l3,
                "timed_out": d.get("timed_out", 0),
            })

if missing:
    print(f"WARNING: {len(missing)} missing stats.log (sweep incomplete or still running):")
    for m in missing[:10]:
        print(f"  {m}")
    if len(missing) > 10:
        print(f"  ... and {len(missing) - 10} more")

with open(DATA / "eval_xbar_hash_sweep.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

# Aggregate: total wait-cycles per (variant, source) across all 20 sets, plus
# % reduction vs. the "default" (no-macro) baseline for the same source.
totals = {(v, s): 0 for v in VARIANTS for s in SOURCES}
for r in rows:
    totals[(r["variant"], r["source"])] += r["wait_cycles"]

summary = []
for source in SOURCES:
    base = totals[("default", source)]
    for variant in VARIANTS:
        t = totals[(variant, source)]
        pct = round(100 * (1 - t / base), 1) if base else 0.0
        summary.append({
            "source": source, "variant": variant,
            "total_wait_cycles": t,
            "pct_reduction_vs_default": pct,
        })

with open(DATA / "eval_xbar_hash_sweep_summary.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(summary[0].keys()))
    w.writeheader()
    w.writerows(summary)

# Printed pivot: rows = source, columns = variant.
print(f"{'source':<8}" + "".join(f"{v:>16}" for v in VARIANTS))
for source in SOURCES:
    cells = []
    for variant in VARIANTS:
        t = totals[(variant, source)]
        base = totals[("default", source)]
        pct = round(100 * (1 - t / base), 1) if base else 0.0
        cells.append(f"{t:>9} ({pct:+.1f}%)")
    print(f"{source:<8}" + "".join(f"{c:>16}" for c in cells))

print()
print(f"wrote {DATA / 'eval_xbar_hash_sweep.csv'}")
print(f"wrote {DATA / 'eval_xbar_hash_sweep_summary.csv'}")
