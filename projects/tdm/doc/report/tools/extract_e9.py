#!/usr/bin/env python3
"""Turns run_e9_sweep.sh's stats.log files into doc/report/data/eval_e9.csv
(report Table E8, mode=fenced, and Table E9, mode=nofence). One row per
(variant, mode) summed across all 20 sets -- M1/M2 are rates, so
numerators/denominators are summed first and the ratio taken once, not
averaged per-set. Reads ragu_a's own per-group stats.log fields: ragu_a's
experience under patroklos2's full real traffic (see run_e9_sweep.sh's
header), each scheme against its own matching set, unlike E7 not isolated.

M2 here (delay_lN_sum) is tb_top.cpp's per-AGU, per-level wait-CYCLE sum --
NOT the crossbar-wide lvl_rd_l1/l2/l3 counters (those are summed across
every active read port, so under this table's full traffic they'd silently
include ragu_b/ragu_c's own wait too). delay_lN_sum is indexed per flat port
inside tally_group(), so it stays correctly ragu_a-specific with multiple
AGUs active -- verified to match lvl_rd_lN exactly under isolation
(doc/report/tools/check_stats_invariants.sh). L3 also gets its own column,
unlike Table E7: isolating removes all write traffic (L3 is exactly 0%
there by construction), but this table's real wagu_a traffic makes L3 a
genuine, often large, contributor to both M1 and M2.

m3_pct is the existing per-AGU ratio (ragu_a's own active_cycles vs.
arrival_cycles) -- a rate, immune to idle time by construction, so it reads
the same in fenced and nofence mode (see tb_top.cpp's grp_stat_t comment).
It answers "how much did ragu_a's own beats get stretched by contention,"
not "how long did the whole run take."

m3_sys_pct/actual_cycles answer that second question instead, and only
make sense in mode=nofence: actual_cycles is the run's real total cycle
count, and ideal_cycles (tb_top.cpp's pre-existing structural lower bound,
pipeline-fill + the slowest AGU's own group count -- computed straight from
the stimuli files, independent of SEL_NO_FENCE) is what that same traffic
would take with zero contention. In fenced mode actual_cycles is dominated
by the descriptors' own schedule gaps, not contention, so actual/ideal
there mixes schedule idle into what should be a pure contention number --
exactly why Table E7/E8 use the arrival-based per-AGU m3_pct instead of
this. In nofence mode (throughput-bound, no schedule gaps once the
NO_FENCE fence-zeroing bug was fixed) actual_cycles is a genuine wall-clock
figure, so 100*(actual_cycles/ideal_cycles - 1) is the real whole-run
cycle-inflation this report was asked for -- reported alongside the raw
actual_cycles cycle count itself, not just the percentage.

Usage: python3 extract_e9.py <sweep-output-dir>
Expects <dir>/{variant}_{mode}_{set}/stats.log for variant in VARIANTS,
mode in {fenced,nofence}, set in 0..19.
"""
import sys
from pathlib import Path

V = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tdm_e9_sweep")
DATA = Path(__file__).resolve().parent.parent / "data"

VARIANTS = ["baseline", "baseline_p32", "l1", "l1v2", "l1v2_p32", "hash16", "hash32"]
SRC_LABEL = {
    "baseline": "unpadded (full traffic)", "baseline_p32": "padded32 (full traffic)",
    "l1": "unpadded (full traffic)",
    "l1v2": "unpadded (full traffic)", "l1v2_p32": "padded32 (full traffic)",
    "hash16": "padded16 (full traffic)", "hash32": "padded32 (full traffic)",
}
SCENARIO_LABEL = {
    "baseline": "Hash11 (baseline)", "baseline_p32": "Hash11 (baseline)", "l1": "XBAR_HASH_L1",
    "l1v2": "XBAR_HASH_L1_V2", "l1v2_p32": "XBAR_HASH_L1_V2",
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
        agg = dict(real_beats=0, conflicts=0, delayed_l1=0, delayed_l2=0, delayed_l3=0,
                    fill_delayed=0, active_cycles=0, arrival_cycles=0, actual_cycles=0,
                    ideal_cycles=0, delay_l1_sum=0, delay_l2_sum=0, delay_l3_sum=0)
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
            agg["delayed_l3"] += d["ragu_a_delayed_l3"]
            agg["fill_delayed"] += d["ragu_a_fill_delayed"]
            agg["active_cycles"] += d["ragu_a_active_cycles"]
            agg["arrival_cycles"] += d.get("ragu_a_arrival_cycles", 0)
            agg["actual_cycles"] += d["actual_cycles"]
            agg["ideal_cycles"] += d["ideal_cycles"]
            agg["delay_l1_sum"] += d["ragu_a_delay_l1_sum"]
            agg["delay_l2_sum"] += d["ragu_a_delay_l2_sum"]
            agg["delay_l3_sum"] += d["ragu_a_delay_l3_sum"]

        rb = agg["real_beats"]
        row = {
            "variant": variant, "mode": mode, "source": SRC_LABEL[variant],
            "scenario": SCENARIO_LABEL[variant],
            "real_beats": rb,
            "m1_l1_pct": round(100 * agg["delayed_l1"] / rb, 1) if rb else 0.0,
            "m1_l2_pct": round(100 * agg["delayed_l2"] / rb, 1) if rb else 0.0,
            "m1_l3_pct": round(100 * agg["delayed_l3"] / rb, 1) if rb else 0.0,
            "m1_tot_pct": round(100 * agg["conflicts"] / rb, 1) if rb else 0.0,
            "m1_gen_pct": round(100 * (agg["conflicts"] - agg["fill_delayed"]) / rb, 1) if rb else 0.0,
            "m2_l1_pct": round(100 * agg["delay_l1_sum"] / rb, 1) if rb else 0.0,
            "m2_l2_pct": round(100 * agg["delay_l2_sum"] / rb, 1) if rb else 0.0,
            "m2_l3_pct": round(100 * agg["delay_l3_sum"] / rb, 1) if rb else 0.0,
            "m2_tot_pct": round(100 * (agg["delay_l1_sum"] + agg["delay_l2_sum"] + agg["delay_l3_sum"]) / rb, 1) if rb else 0.0,
            "m3_pct": round(100 * (agg["active_cycles"] / agg["arrival_cycles"] - 1), 1) if agg["arrival_cycles"] else 0.0,
            "beat_per_ac": round(rb / agg["active_cycles"], 3) if agg["active_cycles"] else 0.0,
            "active_pct": round(100 * agg["active_cycles"] / agg["actual_cycles"], 1) if agg["actual_cycles"] else 0.0,
            "actual_cycles": agg["actual_cycles"],
            "ideal_cycles": agg["ideal_cycles"],
            "m3_sys_pct": round(100 * (agg["actual_cycles"] / agg["ideal_cycles"] - 1), 1) if agg["ideal_cycles"] else 0.0,
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
with open(DATA / "eval_e9.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
    w.writeheader()
    w.writerows(rows)

for mode in MODES:
    print(f"\n=== mode={mode} ===")
    print(f"{'scenario':<24}{'source':<28}{'M1 L1%':>8}{'M1 L2%':>8}{'M1 L3%':>8}{'M1 tot%':>9}"
          f"{'M1 gen%':>9}{'M2 L1%':>8}{'M2 L2%':>8}{'M2 L3%':>8}{'M2 tot%':>9}{'M3%':>9}{'beat/ac':>9}"
          f"{'active%':>9}{'actual_cyc':>12}{'ideal_cyc':>11}{'sysM3%':>9}")
    for r in rows:
        if r["mode"] != mode:
            continue
        print(f"{r['scenario']:<24}{r['source']:<28}{r['m1_l1_pct']:>8.1f}{r['m1_l2_pct']:>8.1f}"
              f"{r['m1_l3_pct']:>8.1f}{r['m1_tot_pct']:>9.1f}{r['m1_gen_pct']:>9.1f}"
              f"{r['m2_l1_pct']:>8.1f}{r['m2_l2_pct']:>8.1f}{r['m2_l3_pct']:>8.1f}{r['m2_tot_pct']:>9.1f}"
              f"{r['m3_pct']:>9.1f}{r['beat_per_ac']:>9.3f}{r['active_pct']:>9.1f}"
              f"{r['actual_cycles']:>12}{r['ideal_cycles']:>11}{r['m3_sys_pct']:>9.1f}")

print(f"\nwrote {DATA / 'eval_e9.csv'}")
