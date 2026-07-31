#!/usr/bin/env python3
"""Turns run_ragu_a_isolation_sweep.sh's stats.log files into
doc/report/data/eval_ragu_a_isolation.csv (report Tables E8/E10, patroklos2
source). One row per (variant, mode) summed across all 20 sets -- M1/M2 are
rates, so numerators/denominators are summed first and the ratio taken once,
not averaged per-set. XBAR_HASH_L2_COMPOSE variants are not included -- see
run_ragu_a_isolation_sweep.sh's header for why.

M2 uses tb_top.cpp's per-AGU delay_lN_sum fields (not the crossbar-wide
lvl_rd_l1/l2/l3 counters), same source as Table E9 -- numerically identical
to lvl_rd_lN here since ragu_a is isolated (verified in
check_stats_invariants.sh), but using the same per-AGU mechanism
everywhere means this script would still be correct if ever pointed at a
non-isolated sweep.

Usage: python3 extract_ragu_a_isolation.py <sweep-output-dir>
Expects <dir>/{variant}_{mode}_{set}/stats.log for variant in VARIANTS,
mode in {fenced,nofence}, set in 0..19.
"""
import sys
from pathlib import Path

V = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tdm_ragu_a_isolation")
DATA = Path(__file__).resolve().parent.parent / "data"

VARIANTS = ["baseline", "l1", "l1v2", "l1v3", "poly", "rob_d2", "rob_d4", "rob_poly_d4", "hash16", "hash32"]
SRC_LABEL = {
    "baseline": "unpadded", "l1": "unpadded", "l1v2": "unpadded", "l1v3": "unpadded",
    "rob_d2": "unpadded", "rob_d4": "unpadded",
    "poly": "unpadded", "rob_poly_d4": "unpadded",
    "hash16": "padded16", "hash32": "padded32",
}
SCENARIO_LABEL = {
    "baseline": "Hash11 (baseline)", "l1": "XBAR_HASH_L1",
    "l1v2": "XBAR_HASH_L1_V2", "l1v3": "XBAR_HASH_L1_V3",
    "rob_d2": "XBAR_ROB d2 + L1_V3", "rob_d4": "XBAR_ROB d4 + L1_V3",
    "poly": "XBAR_HASH_POLY", "rob_poly_d4": "XBAR_ROB d4 + POLY",
    "hash16": "XBAR_HASH16", "hash32": "XBAR_HASH32",
}
MODES = ["fenced", "nofence"]
NSETS = 20


def stats(variant, mode, s):
    d = {}
    path = V / f"{variant}_{mode}_{s}" / "stats.log"
    for line in path.read_text().splitlines():
        k, _, val = line.partition(",")
        d[k] = int(val)
    return d


rows = []
missing = []
for variant in VARIANTS:
    for mode in MODES:
        agg = dict(real_beats=0, conflicts=0, delayed_l1=0, delayed_l2=0, fill_delayed=0,
                    active_cycles=0, arrival_cycles=0, delay_l1_sum=0, delay_l2_sum=0,
                    delay_l3_sum=0, actual_cycles=0)
        for s in range(NSETS):
            try:
                d = stats(variant, mode, s)
            except FileNotFoundError:
                missing.append(f"{variant}_{mode}_{s}")
                continue
            agg["real_beats"] += d["ragu_a_real_beats"]
            agg["conflicts"] += d["ragu_a_conflicts"]
            agg["delayed_l1"] += d["ragu_a_delayed_l1"]
            agg["delayed_l2"] += d["ragu_a_delayed_l2"]
            agg["fill_delayed"] += d["ragu_a_fill_delayed"]
            agg["active_cycles"] += d["ragu_a_active_cycles"]
            agg["arrival_cycles"] += d.get("ragu_a_arrival_cycles", 0)
            agg["delay_l1_sum"] += d.get("ragu_a_delay_l1_sum", 0)
            agg["delay_l2_sum"] += d.get("ragu_a_delay_l2_sum", 0)
            agg["delay_l3_sum"] += d.get("ragu_a_delay_l3_sum", 0)
            agg["actual_cycles"] += d["actual_cycles"]

        rb = agg["real_beats"]
        row = {
            "variant": variant, "mode": mode, "source": SRC_LABEL[variant],
            "scenario": SCENARIO_LABEL[variant],
            "real_beats": rb,
            "m1_l1_pct": round(100 * agg["delayed_l1"] / rb, 1) if rb else 0.0,
            "m1_l2_pct": round(100 * agg["delayed_l2"] / rb, 1) if rb else 0.0,
            "m1_tot_pct": round(100 * agg["conflicts"] / rb, 1) if rb else 0.0,
            "m1_gen_pct": round(100 * (agg["conflicts"] - agg["fill_delayed"]) / rb, 1) if rb else 0.0,
            "m2_l1_pct": round(100 * agg["delay_l1_sum"] / rb, 1) if rb else 0.0,
            "m2_l2_pct": round(100 * agg["delay_l2_sum"] / rb, 1) if rb else 0.0,
            "m2_tot_pct": round(100 * (agg["delay_l1_sum"] + agg["delay_l2_sum"] + agg["delay_l3_sum"]) / rb, 1) if rb else 0.0,
            "m3_pct": round(100 * (agg["active_cycles"] / agg["arrival_cycles"] - 1), 1) if agg["arrival_cycles"] else 0.0,
            "beat_per_ac": round(rb / agg["active_cycles"], 3) if agg["active_cycles"] else 0.0,
            "active_pct": round(100 * agg["active_cycles"] / agg["actual_cycles"], 1) if agg["actual_cycles"] else 0.0,
            "actual_cycles": agg["actual_cycles"],
        }
        rows.append(row)

if missing:
    print(f"WARNING: {len(missing)} missing stats.log (sweep incomplete):")
    for m in missing[:10]:
        print(f"  {m}")
    if len(missing) > 10:
        print(f"  ... and {len(missing) - 10} more")
    sys.exit(1)

import csv
with open(DATA / "eval_ragu_a_isolation_patroklos2.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

for mode in MODES:
    print(f"\n=== mode={mode} ===")
    print(f"{'scenario':<32}{'M1 L1%':>8}{'M1 L2%':>8}{'M1 tot%':>9}{'M1 gen%':>9}"
          f"{'M2 L1%':>8}{'M2 L2%':>8}{'M2 tot%':>9}{'M3%':>9}{'beat/ac':>9}{'active%':>9}{'actual_cyc':>12}")
    for r in rows:
        if r["mode"] != mode:
            continue
        print(f"{r['scenario']:<32}{r['m1_l1_pct']:>8.1f}{r['m1_l2_pct']:>8.1f}{r['m1_tot_pct']:>9.1f}"
              f"{r['m1_gen_pct']:>9.1f}{r['m2_l1_pct']:>8.1f}{r['m2_l2_pct']:>8.1f}{r['m2_tot_pct']:>9.1f}"
              f"{r['m3_pct']:>9.1f}{r['beat_per_ac']:>9.3f}{r['active_pct']:>9.1f}{r['actual_cycles']:>12}")

print(f"\nwrote {DATA / 'eval_ragu_a_isolation_patroklos2.csv'}")
