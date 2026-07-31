#!/usr/bin/env python3
"""Turns an evaluation sweep's stats.log files (run_eval_sweep.sh) into the
doc/report/data/eval_*.csv snapshots gen_report.py renders. Usage:
    python3 extract_eval.py <sweep-output-dir>
Expects <dir>/{cb,adap,rr8,cbnf,adapnf,rr8nf}_{0..19}/stats.log."""
import csv
import sys
from pathlib import Path

V = Path(sys.argv[1]) if len(sys.argv) > 1 else Path("/tmp/tdm_eval_sweep")
DATA = Path(__file__).resolve().parent.parent / "data"
GROUPS = ["ragu_a", "ragu_b", "ragu_c", "ragu_d", "ragu_e",
          "wagu_a", "wagu_b", "wagu_d", "wagu_e"]
GKEYS = ["serve_cycles", "fill_wait_cycles", "real_beats", "nop_beats", "conflicts", "delay_sum", "delay_max",
         "active_cycles", "fill_delayed", "fill_delay_sum", "episodes",
         "stall_episodes", "wait_cycles"]

# Legacy 9-slot-RR column: carried over from the existing CSV when present
# (it is no longer re-measured; see report §5 / Appendix A.7).
rr9 = {}
try:
    with open(DATA / "eval_final.csv") as f:
        for row in csv.DictReader(f):
            rr9[int(row["dataset"])] = int(row.get("tdm_rr9_cycles", 0))
except FileNotFoundError:
    pass

def stats(tag, s):
    d = {}
    for line in (V / f"{tag}_{s}" / "stats.log").read_text().splitlines():
        k, _, v = line.partition(",")
        d[k] = int(v)
    return d

def gsum(d, key):
    return sum(d[f"{g}_{key}"] for g in GROUPS)

rows, util, unf = [], [], []
agu = {b: {g: {k: 0 for k in GKEYS} for g in GROUPS} for b in ("cb", "ad")}
tot_cyc = {"cb": 0, "ad": 0}

for s in range(20):
    cb, ad, r8 = stats("cb", s), stats("adap", s), stats("rr8", s)
    cbn, adn, r8n = stats("cbnf", s), stats("adapnf", s), stats("rr8nf", s)
    for d in (cb, ad, r8, cbn, adn, r8n):
        assert d["timed_out"] == 0, s
    rows.append((s, cb["actual_cycles"], ad["actual_cycles"], r8["actual_cycles"], rr9.get(s, r8["actual_cycles"])))
    cyc_cb, cyc_ad, cyc_r8 = cb["actual_cycles"], ad["actual_cycles"], r8["actual_cycles"]
    tot_cyc["cb"] += cyc_cb
    tot_cyc["ad"] += cyc_ad
    for g in GROUPS:
        for k in GKEYS:
            if k == "delay_max":
                agu["cb"][g][k] = max(agu["cb"][g][k], cb[f"{g}_{k}"])
                agu["ad"][g][k] = max(agu["ad"][g][k], ad[f"{g}_{k}"])
            else:
                agu["cb"][g][k] += cb[f"{g}_{k}"]
                agu["ad"][g][k] += ad[f"{g}_{k}"]

    lvl = [cb[f"lvl_rd_{l}"] + cb[f"lvl_wr_{l}"] for l in ("l1", "l2", "l3", "bank")]
    lvl_tot = max(1, sum(lvl))
    util.append({
        "dataset": s,
        "cb_bank_util_pct": round(100 * cb["bank_busy"] / (cyc_cb * cb["n_banks"]), 2),
        "cb_beats": cb["bank_busy"],
        "cb_port_wait_pct": round(100 * cb["port_wait"] / max(1, cb["port_wait"] + cb["port_serve"]), 1),
        "ad_bank_util_pct": round(100 * ad["bank_busy"] / (cyc_ad * ad["n_banks"]), 2),
        "ad_beats": ad["bank_busy"],
        "ad_bus_busy_pct": round(100 * ad["bus_busy"] / cyc_ad, 1),
        "ad_bus_wasted_pct": round(100 * ad["bus_wasted"] / cyc_ad, 1),
        "ad_contention_pct": round(100 * ad["bus_contention"] / cyc_ad, 1),
        "ad_bank_stall": ad["bank_stall"],
        "r8_bus_busy_pct": round(100 * r8["bus_busy"] / cyc_r8, 1),
        "r8_bus_wasted_pct": round(100 * r8["bus_wasted"] / cyc_r8, 1),
        "r8_contention_pct": round(100 * r8["bus_contention"] / cyc_r8, 1),
        # per-beat view (kept as secondary)
        "cb_conflicts": gsum(cb, "conflicts"),
        "cb_conf_pct": round(100 * gsum(cb, "conflicts") / max(1, gsum(cb, "real_beats")), 1),
        "cb_avg_delay": round(gsum(cb, "delay_sum") / max(1, gsum(cb, "conflicts")), 2),
        "ad_conflicts": gsum(ad, "conflicts"),
        "ad_conf_pct": round(100 * gsum(ad, "conflicts") / max(1, gsum(ad, "real_beats")), 1),
        # wall-clock / event view (headline): contention only — wait cycles
        # in runs that began from group idle (task-start fill) are excluded
        # on BOTH backends by the same rule
        "cb_wait_cycles": gsum(cb, "wait_cycles") - gsum(cb, "fill_wait_cycles"),
        "cb_episodes": gsum(cb, "stall_episodes"),
        "ad_wait_cycles": gsum(ad, "wait_cycles") - gsum(ad, "fill_wait_cycles"),
        "ad_episodes": gsum(ad, "stall_episodes"),
        "cb_fill_wait": gsum(cb, "fill_wait_cycles"),
        "ad_fill_wait": gsum(ad, "fill_wait_cycles"),
        "ad_fill_pct": round(100 * gsum(ad, "fill_delayed") / max(1, gsum(ad, "conflicts")), 1),
        # per-level conflict rates: % of cycles with >=1 request blocked at
        # that level (1 conflicting cycle per 100 cycles = 1%)
        "cb_lvl_l1_rate": round(100 * cb["lvl_cyc_l1"] / cyc_cb, 1),
        "cb_lvl_l2_rate": round(100 * cb["lvl_cyc_l2"] / cyc_cb, 1),
        "cb_lvl_l3_rate": round(100 * cb["lvl_cyc_l3"] / cyc_cb, 1),
        # per-request view: conflict (blocked-request) cycles per 100 requests,
        # split by resolving level; TDM bank column uses the same
        # normalization, and ad_resolve is the serialization depth — extra
        # resolution passes (cycles with >=1 blocked slot) per 32-request
        # window's worth of traffic
        "cb_l1_per_req": round(100 * lvl[0] / max(1, gsum(cb, "real_beats")), 1),
        "cb_l2_per_req": round(100 * lvl[1] / max(1, gsum(cb, "real_beats")), 1),
        "cb_l3_per_req": round(100 * lvl[2] / max(1, gsum(cb, "real_beats")), 1),
        "ad_bank_per_req": round(100 * ad["bank_stall"] / max(1, ad["bus_real_beats"]), 1),
        "ad_resolve_per_win": round(32.0 * ad["bank_stall_cycles"] / max(1, ad["bus_real_beats"]), 2),
        "cb_lvl_l1_pct": round(100 * lvl[0] / lvl_tot),
        "cb_lvl_l2_pct": round(100 * lvl[1] / lvl_tot),
        "cb_lvl_l3_pct": round(100 * lvl[2] / lvl_tot),
        # activity decomposition: active = serve + wait (per-cycle group view)
        "cb_serve_cycles": gsum(cb, "serve_cycles"),
        "ad_serve_cycles": gsum(ad, "serve_cycles"),
        # bus width utilization: real beats per busy bus turn (of 32 possible)
        "ad_beats_per_turn": round(ad["bus_real_beats"] / max(1, ad["bus_busy"]), 2),
        "rsp_fixed": int(cb["rsp_min"] == cb["rsp_max"] == ad["rsp_min"] == ad["rsp_max"] == 1),
    })
    unf.append({
        "dataset": s,
        "cb_nf_cycles": cbn["actual_cycles"],
        "ad_nf_cycles": adn["actual_cycles"],
        "r8_nf_cycles": r8n["actual_cycles"],
        "ratio_ad_cb": round(adn["actual_cycles"] / cbn["actual_cycles"], 2),
        "ad_nf_bus_busy_pct": round(100 * adn["bus_busy"] / adn["actual_cycles"], 1),
        "ad_nf_contention_pct": round(100 * adn["bus_contention"] / adn["actual_cycles"], 1),
        "ad_nf_beats_per_turn": round(adn["bus_real_beats"] / max(1, adn["bus_busy"]), 2),
        "cb_nf_beats_per_cycle": round(cbn["bank_busy"] / max(1, cbn["actual_cycles"]), 2),
        "ideal_cycles": cbn["ideal_cycles"],
    })

with open(DATA / "eval_final.csv", "w") as f:
    f.write("dataset,crossbar_cycles,tdm_adaptive_cycles,tdm_rr8_cycles,tdm_rr9_cycles\n")
    for r in rows:
        f.write(",".join(map(str, r)) + "\n")
for name, data in (("eval_util.csv", util), ("eval_unfenced.csv", unf)):
    with open(DATA / name, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(data[0].keys()))
        w.writeheader()
        w.writerows(data)

# SET x AGU contention-wait matrix (fill excluded), TDM adaptive and crossbar
with open(DATA / "eval_matrix.csv", "w") as f:
    f.write("dataset,cycles," + ",".join(f"{g}_ad,{g}_cb" for g in GROUPS) + "\n")
    for s_ in range(20):
        cb, ad = stats("cb", s_), stats("adap", s_)
        cells = []
        for g in GROUPS:
            cells.append(str(ad[f"{g}_wait_cycles"] - ad[f"{g}_fill_wait_cycles"]))
            cells.append(str(cb[f"{g}_wait_cycles"] - cb[f"{g}_fill_wait_cycles"]))
        f.write(f"{s_},{cb['actual_cycles']}," + ",".join(cells) + "\n")

with open(DATA / "eval_agu.csv", "w") as f:
    f.write("agu,cb_real_beats,cb_wait_cycles,cb_episodes,cb_fill_wait,cb_active_pct,"
            "ad_wait_cycles,ad_episodes,ad_fill_wait,ad_active_pct\n")
    for g in GROUPS:
        c, a = agu["cb"][g], agu["ad"][g]
        f.write(f"{g.upper()},{c['real_beats']},"
                f"{c['wait_cycles'] - c['fill_wait_cycles']},{c['stall_episodes']},"
                f"{c['fill_wait_cycles']},"
                f"{round(100*c['active_cycles']/tot_cyc['cb'],1)},"
                f"{a['wait_cycles'] - a['fill_wait_cycles']},{a['stall_episodes']},"
                f"{a['fill_wait_cycles']},"
                f"{round(100*a['active_cycles']/tot_cyc['ad'],1)}\n")

print(f"{'set':>3} | fenced: {'cbWait':>7} {'cbEpis':>7} {'adWait':>7} {'adEpis':>7} {'fill%':>5} | "
      f"unfenced: {'cb':>6} {'adap':>6} {'rr8':>6} {'ad/cb':>5} {'bus%':>5}")
for u, n in zip(util, unf):
    print(f"{u['dataset']:>3} |         {u['cb_wait_cycles']:>7} {u['cb_episodes']:>7} "
          f"{u['ad_wait_cycles']:>7} {u['ad_episodes']:>7} {u['ad_fill_pct']:>5} |          "
          f"{n['cb_nf_cycles']:>6} {n['ad_nf_cycles']:>6} {n['r8_nf_cycles']:>6} "
          f"{n['ratio_ad_cb']:>5} {n['ad_nf_bus_busy_pct']:>5}")
print()
print((DATA / "eval_agu.csv").read_text())
print("fenced cycles unchanged:",
      all(r[1] == 116275 or r[0] == 6 for r in rows))
