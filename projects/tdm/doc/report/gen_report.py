#!/usr/bin/env python3
# -----------------------------------------------------------------------------
# gen_report.py — regenerate the TDM vs. crossbar design report (report.html).
#
# Inputs (all under data/, see README.md for how to refresh them):
#   timing_xbar.log          stim_bank suite, IMPL_CROSSBAR,     -DSTIM_TIMING_REPORT
#   timing_tdm_rr.log        stim_bank suite, IMPL_TDM,          -DSTIM_TIMING_REPORT
#   timing_tdm_adaptive.log  stim_bank suite, IMPL_TDM+ADAPTIVE, -DSTIM_TIMING_REPORT
#   sweep.txt                one line: "suite: N | suite: N | ..." from the full sweep
#
# Output: report.html next to this script (override with argv[1]).
#
# The prose and figures are maintained HERE, in this file; the tables are
# recomputed from the logs on every run, so a re-measured design only needs
# fresh logs — the report text only changes when the architecture does.
# -----------------------------------------------------------------------------
import html
import re
import sys
from collections import defaultdict
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA = HERE / "data"
OUT = Path(sys.argv[1]) if len(sys.argv) > 1 else HERE / "report.html"

NUM_REQ = 4  # OBI lanes per port group (matches rtl PARAMS)

# ---------------------------------------------------------------- parse inputs
BACKENDS = [
    ("crossbar", "timing_xbar.log"),
    ("tdm", "timing_tdm_rr.log"),
    ("tdm-adaptive", "timing_tdm_adaptive.log"),
]
BK_LABEL = {"crossbar": "Crossbar", "tdm": "TDM·RR", "tdm-adaptive": "TDM·adaptive"}

# spans[(phase, ports, n_data, note)][backend] = span
spans = defaultdict(dict)
for bk, fname in BACKENDS:
    for line in (DATA / fname).read_text(errors="replace").splitlines():
        m = re.match(r"\[timing\] ([\w-]+),(\w+),(\d+),(\d+),([^,]*),(\d+)", line)
        if m:
            _, phase, ports, n, note, span = m.groups()
            spans[(phase, int(ports), int(n), note)][bk] = int(span)

suites = []
for part in (DATA / "sweep.txt").read_text().split("|"):
    name, _, count = part.strip().partition(":")
    if name:
        suites.append((name.strip(), int(count)))
n_tests = sum(c for _, c in suites)

# --- §Evaluation: final/0-19 production-representative sweep ----------------
eval_rows = []  # (dataset, crossbar, tdm_adaptive, tdm_rr8, tdm_rr9)
_eval_path = DATA / "eval_final.csv"
if _eval_path.exists():
    for line in _eval_path.read_text().splitlines()[1:]:
        if not line.strip():
            continue
        ds, cb, ad, r8, r9 = line.split(",")
        eval_rows.append((int(ds), int(cb), int(ad), int(r8), int(r9)))
eval_n = len(eval_rows)
eval_exact_parity = sum(1 for _, cb, ad, _r8, _r9 in eval_rows if ad == cb)
eval_rr8_parity = sum(1 for _, cb, _ad, r8, _r9 in eval_rows if r8 == cb)

eval_util = []  # per-set utilization/conflict metrics (dict per row)
_util_path = DATA / "eval_util.csv"
if _util_path.exists():
    _lines = _util_path.read_text().splitlines()
    _keys = _lines[0].split(",")
    for line in _lines[1:]:
        if not line.strip():
            continue
        eval_util.append(dict(zip(_keys, line.split(","))))

PHASES = [
    ("phase1_read", "Phase 1 — single-group reads (RAGU_A, 1 port)"),
    ("phase2_read", "Phase 2 — multi-port reads (RAGU_A, 1/2/4 ports)"),
    ("phase3_read", "Phase 3 — reads interleaved with writes"),
    ("phase3_write", "Phase 3 — writes (WAGU_A, 1/2/4 ports)"),
    ("phase4_read", "Phase 4 — mixed-requester reads"),
    ("phase4_write", "Phase 4 — mixed-requester writes"),
]

def ceil_div(a, b):
    return -(-a // b)

# fill-latency rows: first-response-cycle minus the task's own start_cycle
# fence, for the one genuinely idle-buffer moment in the whole sweep (the
# front() task of each phase-1..4 group — see stim_bank_common.hpp's own
# fill_latency block). label -> (kind, description)
FILL_POINTS = [
    ("phase1_read_cold", "read", "RAGU_A, cold boot from reset"),
    ("phase2_read_cold", "read", "RAGU_B, cold boot from reset"),
    ("phase3_write_cold", "write", "WAGU_A, cold boot from reset"),
    ("phase4_write_cold", "write", "WAGU_B, cold boot from reset"),
    ("phase3_read_idle", "read", "RAGU_A, restart from idle"),
    ("phase4_read_idle", "read", "RAGU_B, restart from idle"),
]
fill_rows = []
for label, kind, desc in FILL_POINTS:
    bks = spans.get(("fill_latency", 0, 0, label), {})
    fill_rows.append((desc, kind, bks.get("crossbar"), bks.get("tdm"), bks.get("tdm-adaptive")))

# conflict-free rows: (phase, ports, n, expect, xb, rr, ad)
cf_rows = defaultdict(list)
for (phase, ports, n, note), bks in sorted(spans.items()):
    if (phase == "fill_latency" or phase == "phase5_conflict" or phase.startswith("phase6")
            or phase.startswith("phase7") or phase.startswith("phase8")):
        continue
    lanes = ports * NUM_REQ
    cf_rows[phase].append(
        (ports, n, lanes, ceil_div(n, lanes), bks["crossbar"], bks["tdm"], bks["tdm-adaptive"])
    )

# phase 6 same-bank streaming, own-map flavor only (each backend vs its own
# worst case; the cross-mapped flavor is folded into the class tables below):
# (direction, ports, n, xb, rr, ad)
sb_rows = []
for (phase, ports, n, note), bks in sorted(spans.items()):
    if phase in ("phase6_write", "phase6_read") and note == "same_bank":
        direction = "write" if phase.endswith("write") else "read"
        sb_rows.append((direction, ports, n, bks["crossbar"], bks["tdm"], bks["tdm-adaptive"]))
sb_rows.sort(key=lambda r: (r[0] == "read", r[1]))  # writes first, then by ports
sb_total = {
    "crossbar": sum(r[3] for r in sb_rows),
    "tdm": sum(r[4] for r in sb_rows),
    "tdm-adaptive": sum(r[5] for r in sb_rows),
}

# --- cross-mapped pattern classes -------------------------------------------
# A conflict pattern is only adversarial relative to ONE mapping, and each
# build constructs its patterns against a chosen map: notes without a suffix
# are same-bank under the build's OWN routing; "_xbarhash"/"_tdmmap" notes
# are same-bank under the SIBLING's. Pairing them gives each pattern class
# measured on all three backends. (Caveat: the concrete addresses differ per
# build — the searches consume different candidates — so rows compare the
# structural class, not a byte-identical stream.)
def sp(bk, phase, ports, n, note):
    return spans.get((phase, ports, n, note), {}).get(bk)

p5_class_rows = []
for ports in (1, 2, 4):
    n = ports * NUM_REQ * 8
    p5_class_rows.append(("same-bank under crossbar hash", ports, n,
                          sp("crossbar", "phase5_conflict", ports, n, "conflict=full"),
                          sp("tdm", "phase5_conflict", ports, n, "conflict=full_xbarhash"),
                          sp("tdm-adaptive", "phase5_conflict", ports, n, "conflict=full_xbarhash")))
for ports in (1, 2, 4):
    n = ports * NUM_REQ * 8
    p5_class_rows.append(("same-bank under TDM map", ports, n,
                          sp("crossbar", "phase5_conflict", ports, n, "conflict=full_tdmmap"),
                          sp("tdm", "phase5_conflict", ports, n, "conflict=full"),
                          sp("tdm-adaptive", "phase5_conflict", ports, n, "conflict=full")))


# phase 7 crossbar-structural rows: (stimulus, direction, n, xb, rr, ad)
P7_KINDS = [("free", "conflict-free"), ("intra_port", "L1 / intra-port (4:1)"),
            ("inter_port", "L2 / inter-port (4:1)"), ("rw_bank", "L3 / R-W same-target"),
            ("noise", "random noise")]
p7_rows = []
for note, label in P7_KINDS:
    n = 4096 if note == "noise" else 256
    for d, ph in (("read", "phase7_read"), ("write", "phase7_write")):
        xb = sp("crossbar", ph, 4, n, note)
        rr = sp("tdm", ph, 4, n, note)
        ad = sp("tdm-adaptive", ph, 4, n, note)
        if xb is None:
            continue
        p7_rows.append((label, d, n, xb, rr, ad, f"{xb / ad:.1f}×"))

# phase 8 full-parallel rows: (agu, dir, ports, xb, rr, ad)
P8_AGUS = [("ragu_a", "read", 2), ("ragu_b", "read", 2), ("ragu_c", "read", 1),
           ("ragu_d", "read", 1), ("ragu_e", "read", 1),
           ("wagu_a", "write", 2), ("wagu_b", "write", 2), ("wagu_d", "write", 1),
           ("wagu_e", "write", 1)]
P8_N = 4096
p8_rows = []
for agu, d, ports in P8_AGUS:
    ph = f"phase8_{d}"
    xb = sp("crossbar", ph, ports, P8_N, agu)
    if xb is None:
        continue
    p8_rows.append((agu.replace("_", "\u2009"), d, ports,
                    xb, sp("tdm", ph, ports, P8_N, agu), sp("tdm-adaptive", ph, ports, P8_N, agu)))
p8_overall = {bk: sp(bk, "phase8_total", 9, 9 * P8_N, "all")
              for bk in ("crossbar", "tdm", "tdm-adaptive")}

conflict_rows = []  # own-map flavor only: (ports, n, kind, xb, rr, ad)
for (phase, ports, n, note), bks in sorted(spans.items()):
    if phase == "phase5_conflict":
        kind = note.split("=")[1]
        if "_" in kind:  # cross-mapped flavor — handled by the class tables
            continue
        conflict_rows.append((ports, n, kind, bks["crossbar"], bks["tdm"], bks["tdm-adaptive"]))
kind_order = {"none": 0, "partial": 1, "full": 2}
conflict_rows.sort(key=lambda r: (r[0], kind_order[r[2]]))

cf_total = {bk: 0 for bk, _ in BACKENDS}
for rows in cf_rows.values():
    for _, _, _, _, xb, rr, ad in rows:
        cf_total["crossbar"] += xb
        cf_total["tdm"] += rr
        cf_total["tdm-adaptive"] += ad
c_total = {"crossbar": 0, "tdm": 0, "tdm-adaptive": 0}
for _, _, _, xb, rr, ad in conflict_rows:
    c_total["crossbar"] += xb
    c_total["tdm"] += rr
    c_total["tdm-adaptive"] += ad
n_cf_tasks = sum(len(v) for v in cf_rows.values())
parity_ok = all(
    ad == exp and xb == exp
    for rows in cf_rows.values()
    for _, _, _, exp, xb, _, ad in rows
)

SUITE_FOCUS = {
    "agu": "Trace-driven bundled AGU: grant/ack accounting, lookahead exposure and zero-padding, in-flight ordering, empty-trace and rollover edges",
    "arbiter": "Both arbiters: RR sequence/lag/reset; adaptive same-cycle combinational grant, skip-idle, fairness rotation, all-to-none recovery",
    "tdm": "Mapping module interface contract: per-lane independence, we/be broadcast vs per-lane payloads, NOP lanes, concurrent returns (placement math excluded by design)",
    "bank": "Bank model: OBI R-5 timing, one-shot rvalid, walking and interleaved byte-enables, overwrite composition, row wrap",
    "crossbar": "Word-interleaved crossbar primitive: both decode modes, per-bank round-robin arbitration, owner-register response steering",
    "buffer_modes": "Buffer geometry on 4-port and 1-port instances, read AND write: active_mode-3 alias, PORT_COUNT clamp, per-slot data",
    "buffer": "Read buffer windows: priming, drain order, wrap, boot restart, mode re-latch, drain echo",
    "buffer_cell": "Read cell: fetch engine, forward mux, idle-restart rule, NOP fetches, en-gap immunity, idle-report lifecycle",
    "buffer_cell_wr": "Write cell: primary latch, snapshot hand-off, shadow free-at-grant, NOP primaries, be passthrough, flush-only we",
    "buffer_wr": "Write buffer: fill/snapshot/posted-respond protocol, back-pressure, exact slot mapping, rdata=0 acks, inactive-lane silence",
    "buffer_pipeline": "Read window pipelining: zero-bubble drain across window boundaries",
    "buffer_pipeline_write": "Write window pipelining: back-to-back fills across snapshots",
    "lane_agu": "Per-lane (unbundled) AGU driver for the E groups (today's DMA): trace playback, 2 sub-ports, multi-task window packing",
    "stim_bank_tdm": "Full-system stimuli on TDM·RR: routing + data checks",
    "stim_bank_tdm_adaptive": "Full-system stimuli on TDM·adaptive: routing + data + exact-span assertions",
    "stim_bank_xbar": "Full-system stimuli on crossbar: routing + data + exact-span assertions",
    "system_stimuli_tdm": "End-to-end system smoke (TDM)",
    "system_stimuli_xbar": "End-to-end system smoke (crossbar)",
    "top_crossbar": "Crossbar top: address hash and L1/L2/L3 routing",
    "top_crossbar_conflict": "Crossbar serialization under deliberate bank conflicts",
    "top_tdm": "TDM top integration: buffers + arbiter + map + banks, posted-ack ordering",
}

# ------------------------------------------------------------------ svg helpers
def esc(s):
    return html.escape(str(s), quote=True)

def box(x, y, w, h, lines, fill="#eceff4", stroke="#1b1b1f", fs=11.5, mono_last=False):
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="{fill}" stroke="{stroke}" stroke-width="1.1"/>']
    n = len(lines)
    for i, ln in enumerate(lines):
        ty = y + h / 2 + (i - (n - 1) / 2) * (fs + 3) + fs * 0.35
        cls = "fm" if (mono_last and i == n - 1) else "ff"
        w_attr = ' font-weight="600"' if i == 0 and n > 1 else ""
        out.append(
            f'<text x="{x + w / 2}" y="{ty:.1f}" class="{cls}" font-size="{fs if i == 0 else fs - 1.5}"'
            f'{w_attr} text-anchor="middle">{esc(ln)}</text>'
        )
    return "\n".join(out)

def arrow(x1, y1, x2, y2, dashed=False, color="#1b1b1f"):
    d = ' stroke-dasharray="4 3"' if dashed else ""
    return (
        f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" stroke-width="1.1"{d} '
        f'marker-end="url(#ah)"/>'
    )

def txt(x, y, s, fs=10, anchor="middle", cls="ff", color="#575862", style=""):
    return (
        f'<text x="{x}" y="{y}" class="{cls}" font-size="{fs}" text-anchor="{anchor}" '
        f'fill="{color}" style="{style}">{esc(s)}</text>'
    )

SVG_DEFS = (
    '<defs><marker id="ah" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" '
    'markerHeight="7" orient="auto-start-reverse">'
    '<path d="M 0 1 L 9 5 L 0 9 z" fill="#1b1b1f"/></marker></defs>'
)

def fig_crossbar_wiring():
    p = [SVG_DEFS]
    # columns: AGUs | addr_hash | L1 | L2 | L3 | banks; rows: read (top) / write (bottom)
    yr, yw, hh = 36, 140, 70
    p.append(box(8, yr, 92, hh, ["read groups ×5", "RAGU A·B·C·D·E", "9 ports · 36 lanes", "(= Fig. 2)"], "#eceff4", fs=10))
    p.append(box(8, yw, 92, hh, ["write groups ×4", "WAGU A·B·D·E", "8 ports · 32 lanes", "(= Fig. 2)"], "#eceff4", fs=10))
    p.append(box(132, yr, 86, hh + (yw - yr), ["addr_hash", "per lane, comb", "addr[8:6] +=", "addr[11:9]"], "#f4ece0", fs=10.5))
    p.append(box(250, yr, 100, hh, ["L1 read ×9", "4×4 xbar / port", "sel addr[4+:2]"], "#f4ece0", fs=10.5))
    p.append(box(250, yw, 100, hh, ["L1 write ×8", "4×4 xbar / port", "sel addr[4+:2]"], "#f4ece0", fs=10.5))
    p.append(box(382, yr, 100, hh, ["L2 read ×4", "9-in 8-out / lane", "sel addr[6+:3]"], "#f4ece0", fs=10.5))
    p.append(box(382, yw, 100, hh, ["L2 write ×4", "8-in 8-out / lane", "sel addr[6+:3]"], "#f4ece0", fs=10.5))
    p.append(box(514, 88, 64, 100, ["L3 ×32", "2×2 R/W", "merge, sel", "addr[9]"], "#f4ece0", fs=10.5))
    p.append(box(604, 66, 70, 144, ["bank ×64", "512 rows", "× 128 b", "= 32 logical", "× even/odd", "(same module", "as Fig. 2)"], "#eceff4", fs=10))
    # forward wiring, read then write
    p.append(arrow(100, yr + hh / 2, 132, yr + hh / 2))
    p.append(arrow(100, yw + hh / 2, 132, yw + hh / 2))
    p.append(arrow(218, yr + hh / 2, 250, yr + hh / 2))
    p.append(arrow(218, yw + hh / 2, 250, yw + hh / 2))
    p.append(arrow(350, yr + hh / 2, 382, yr + hh / 2))
    p.append(arrow(350, yw + hh / 2, 382, yw + hh / 2))
    p.append(arrow(482, yr + hh / 2, 514, 108))
    p.append(arrow(482, yw + hh / 2, 514, 168))
    p.append(arrow(578, 138, 604, 138))
    p.append(txt(116, yr + hh / 2 - 8, "36×", 9))
    p.append(txt(116, yw + hh / 2 - 8, "32×", 9))
    p.append(txt(234, yr + hh / 2 - 8, "36×", 9))
    p.append(txt(234, yw + hh / 2 - 8, "32×", 9))
    p.append(txt(366, yr + hh / 2 - 8, "36×", 9))
    p.append(txt(366, yw + hh / 2 - 8, "32×", 9))
    p.append(txt(591, 130, "64×", 9))
    # return path (dashed, right to left along the bottom)
    p.append(arrow(624, 214, 54, 214, dashed=True, color="#575862"))
    p.append(arrow(54, 214, 54, yw + hh, dashed=True, color="#575862"))
    p.append(txt(341, 228, "return: gnt · rvalid · rdata (128 b), 1-cycle bank latency, steered back through the same levels", 9.5))
    p.append(txt(341, 258, "every wire is one OBI lane (addr 32 b · wdata/rdata 128 b · be · we); the addr[4+:6] routing field is stripped before the bank", 9.5))
    return f'<svg viewBox="0 0 682 272" role="img" aria-label="Crossbar backend wiring">{"".join(p)}</svg>'

def fig_tdm_wiring():
    p = [SVG_DEFS]
    # lookahead / window_reset annotations above the boxes
    p.append(txt(96, 26, "lookahead: fetch_addr[32] + valid, per read buffer (from each AGU's own trace)", 9.5, "start"))
    p.append(arrow(150, 32, 190, 56, dashed=True, color="#575862"))
    p.append(txt(96, 44, "window_reset back: “advance lookahead one window”", 9.5, "start"))
    p.append(arrow(140, 50, 76, 68, dashed=True, color="#575862"))
    # main columns
    p.append(box(8, 70, 96, 84, ["read groups ×5", "RAGU A·B·C·D·E", "9 ports · 36 lanes", "(= Fig. 1)"], "#eceff4", fs=10))
    p.append(box(8, 210, 96, 84, ["write groups ×4", "WAGU A·B·D·E", "8 ports · 32 lanes", "(= Fig. 1)"], "#eceff4", fs=10))
    p.append(box(124, 56, 124, 112, ["read buffers ×5", "buf_r0..r4", "32 cells each", "prefetch window"], "#e3ecf7", fs=10.5))
    p.append(box(124, 196, 124, 112, ["write buffers ×4", "buf_w0..w3", "32 cells each", "fill + shadow"], "#e3ecf7", fs=10.5))
    p.append(box(282, 126, 58, 108, ["mux", "9 → 1", "comb"], "#e3ecf7", fs=10.5))
    p.append(box(374, 126, 96, 108, ["TDM map", "XOR-skew", "word→(bank,row)", "re-enc addr =", "(row·32+b)·16 B"], "#e3ecf7", fs=10.5))
    p.append(box(504, 126, 66, 108, ["bank", "router", "(xbar 32→32,", "as Fig. 1)", "bank =", "beat % 32"], "#e3ecf7", fs=9.5))
    p.append(box(600, 100, 74, 160, ["bank ×32", "1024 rows", "× 128 b", "(same module", "as Fig. 1)"], "#eceff4", fs=10))
    # forward wiring
    p.append(arrow(104, 112, 124, 112))
    p.append(arrow(104, 252, 124, 252))
    p.append(txt(56, 166, "16/8/4/4/4 lanes", 9))
    p.append(txt(56, 306, "16/8/4/4 lanes", 9))
    p.append(arrow(248, 112, 282, 150))
    p.append(arrow(248, 252, 282, 210))
    p.append(arrow(340, 180, 374, 180))
    p.append(arrow(470, 180, 504, 180))
    p.append(arrow(570, 180, 600, 180))
    p.append(txt(263, 122, "9 ×", 9))
    p.append(txt(263, 240, "32-lane", 9))
    p.append(txt(357, 172, "32×", 9))
    p.append(txt(487, 172, "32×", 9))
    p.append(txt(585, 172, "32×", 9))
    # arbiter + map-cfg row below, dashed control wiring
    p.append(box(240, 316, 132, 56, ["arbiter (1-of-9)", "RR: free-run counter", "adaptive: skip idle"], "#e3ecf7", fs=10))
    p.append(box(396, 316, 150, 56, ["per-buffer map cfg", "R · C · L · store_mode", "muxed by sel_req"], "#eceff4", fs=10))
    p.append(arrow(298, 316, 306, 234, dashed=True, color="#575862"))
    p.append(arrow(330, 316, 322, 234, dashed=True, color="#575862"))
    p.append(txt(306, 386, "sel_req (fwd) · sel_rsp (return, 1-cy late)", 9, "middle"))
    p.append(arrow(452, 316, 430, 234, dashed=True, color="#575862"))
    p.append(arrow(240, 344, 190, 308, dashed=True, color="#575862"))
    p.append(txt(60, 330, "req_any[9] (adaptive)", 9, "start"))
    # return path: below the map/xbar row, up into the mux (which steers it
    # back to the owning buffer via sel_rsp — the caption explains the rest)
    p.append(arrow(630, 268, 296, 268, dashed=True, color="#575862"))
    p.append(arrow(290, 268, 290, 234, dashed=True, color="#575862"))
    p.append(txt(470, 282, "return: gnt · rvalid · rdata — steered back to the owning buffer by sel_rsp", 9.5))
    p.append(txt(341, 402, "each lane logically carries one 32 b kernel word, but the plumbing never touches words:", 9.5))
    p.append(txt(341, 415, "the router is the same crossbar primitive as Fig. 1, moving whole 128 b OBI beats (bank = beat % 32)", 9.5))
    return f'<svg viewBox="0 0 682 428" role="img" aria-label="TDM backend wiring">{"".join(p)}</svg>'

def fig_buffer_block():
    p = [SVG_DEFS]
    # ---- left column: external interface
    p.append(box(8, 30, 122, 128, ["port side", "p[NUM_IO] bundles", "req · addr · be ·", "wdata → · ← gnt ·", "rvalid · rdata", "NUM_IO = ports·4"], "#eceff4", fs=9.5, ))
    p.append(box(8, 186, 122, 66, ["lookahead (read)", "fetch_addr_i[32]", "fetch_addr_valid_i"], "#e3ecf7", fs=9.5))
    p.append(box(8, 280, 122, 54, ["config", "active_mode", "clk_i · rst_ni"], "#eceff4", fs=9.5))
    # ---- routing (comb) and control (seq)
    p.append(box(172, 30, 148, 128,
                 ["port⇄group routing", "(comb)", "read: drain group", "rd_ptr_q, is_fwd mux", "write: fill group", "fill_ptr_q, p_gnt=fill_ok"],
                 "#f4ece0", fs=9.5))
    p.append(box(172, 210, 148, 150,
                 ["control (seq)", "rd_ptr_q · primed_q", "window_mode_q · full_q", "fill_ptr_q · resp_*_q", "→ cell_reset_window_s", "(one broadcast pulse)", "← valid/invalid[w]", "collectors"],
                 "#f4ece0", fs=9))
    # ---- cell array
    p.append(f'<rect x="364" y="24" width="196" height="342" fill="none" stroke="#575862" stroke-width="1" stroke-dasharray="2 3"/>')
    p.append(txt(462, 18, "cells[0..31]  (buffer_cell ×32, one per TDM slot)", 9.5))
    rows = [("cell 0", 34), ("cell 1", 62), ("cell 2", 90), ("cell 3", 118),
            ("cell 4", 158), ("cell 5", 186), ("⋮", 232), ("cell 30", 278), ("cell 31", 306)]
    for lbl, y in rows:
        if lbl == "⋮":
            p.append(txt(462, y + 14, "⋮", 13))
            continue
        p.append(box(380, y, 164, 22, [lbl], "#e3ecf7", fs=9.5))
        p.append(arrow(544, y + 11, 596, y + 11))
        p.append(arrow(596, y + 17, 544, y + 17, color="#575862"))
    p.append(txt(400, 152, "group 0 (λ cells) ─", 8.5, "start"))
    p.append(txt(400, 340, "group 7 ─", 8.5, "start"))
    # ---- TDM side bracket
    p.append(box(600, 30, 74, 340, ["TDM side", "m[0..31]", "", "req · addr", "we · be", "wdata →", "", "← gnt", "← rvalid", "← rdata"], "#eceff4", fs=9.5))
    p.append(txt(674, 384, "each cell w owns its manager bundle m[w]", 8.5, "end"))
    # ---- wiring: datapath (solid), returns (grey), control (dashed)
    p.append(arrow(130, 80, 172, 80))
    p.append(txt(151, 72, "λ", 9))
    p.append(arrow(172, 110, 130, 110, color="#575862"))
    p.append(arrow(320, 60, 380, 60))
    p.append(txt(348, 44, "cell_p_s[w]", 8, "middle", "fm"))
    p.append(txt(348, 53, "req/addr/wdata/be", 8, "middle", "fm"))
    p.append(arrow(380, 130, 320, 130, color="#575862"))
    p.append(txt(350, 122, "cell_p_s[w].rdata", 8, "middle", "fm"))
    # lookahead → cells, routed through the free corridor between the boxes
    p.append(arrow(130, 200, 380, 200, dashed=True, color="#575862"))
    p.append(txt(216, 194, "fetch_addr_i[w] → addr_i · en", 8, "start", "fm"))
    # routing ptrs from control, reset pulse + collectors between control and cells
    p.append(arrow(200, 210, 200, 158))
    p.append(txt(208, 178, "ptrs", 8, "start"))
    p.append(arrow(320, 262, 364, 262, dashed=True))
    p.append(arrow(364, 306, 320, 306, dashed=True, color="#575862"))
    # window_reset exported to the caller
    p.append(arrow(246, 360, 246, 376, dashed=True, color="#575862"))
    p.append(txt(246, 388, "window_reset → caller: “advance lookahead one window”", 8.5))
    return f'<svg viewBox="0 0 682 396" role="img" aria-label="Buffer block diagram with wiring">{"".join(p)}</svg>'

def fig_buffer_timeline():
    p = [SVG_DEFS]
    LBL_W, CW, RH, X0 = 118, 38, 20, 126
    NCYC = 14

    def grid(y0, rows_labels):
        for c in range(NCYC):
            p.append(txt(X0 + c * CW + CW / 2, y0 - 4, str(c), 8, "middle", "fm"))
        for i, lbl in enumerate(rows_labels):
            p.append(txt(X0 - 8, y0 + i * RH + RH / 2 + 3, lbl, 8.5, "end"))

    def cell(y0, row, c, label, fill="#e3ecf7", span=1):
        x = X0 + c * CW
        y = y0 + row * RH + 2
        p.append(f'<rect x="{x + 1}" y="{y}" width="{CW * span - 2}" height="{RH - 4}" fill="{fill}" stroke="#1b1b1f" stroke-width="0.7"/>')
        p.append(txt(x + CW * span / 2, y + RH / 2 + 1.5, label, 8, "middle", "fm"))

    # ---- (a) read steady state, λ=4 → 8 groups per window
    p.append(txt(8, 14, "(a) read window, λ=4 (8 groups) — drain, per-cell refetch, and return, zero bubbles", 10, "start", "ff", "#1b1b1f", "font-weight:600"))
    ya = 34
    grid(ya, ["port drain", "bus refetch", "data back"])
    names = ["g0", "g1", "g2", "g3", "g4", "g5", "g6", "g7", "g0’", "g1’", "g2’", "g3’", "g4’", "g5’"]
    for c in range(NCYC):
        cell(ya, 0, c, names[c])
        cell(ya, 1, c, names[c], "#f4ece0")
        if c >= 2:
            cell(ya, 2, c, names[c - 2], "#eceff4")
    p.append(txt(X0 + 7 * CW + CW / 2, ya + 3 * RH + 14,
                 "slot g0 is next needed 8 cycles after its drain; its refetch lands after 2 — 6 cycles of slack", 8.5))
    # ---- (b) write pipeline
    yb = 140
    p.append(txt(8, yb - 20, "(b) write windows, λ=4 — fill, snapshot, posted respond, and shadow flush overlap", 10, "start", "ff", "#1b1b1f", "font-weight:600"))
    grid(yb, ["port fill", "snapshot", "posted ack", "shadow flush"])
    fills = ["f0", "f1", "f2", "f3", "f4", "f5", "f6", "f7", "f0’", "f1’", "f2’", "f3’", "f4’", "f5’"]
    for c in range(NCYC):
        cell(yb, 0, c, fills[c])
    cell(yb, 1, 8, "S", "#f4ece0")
    for c in range(9, NCYC):
        cell(yb, 2, c, f"r{c - 9}", "#eceff4")
    cell(yb, 3, 9, "window burst → banks (shadows free at their grant)", "#e3ecf7", span=4)
    p.append(txt(X0 + 4 * CW, yb + 4 * RH + 14,
                 "the snapshot frees the primaries the same edge — f0’ fills at cycle 8 with no gap;", 8.5, "start"))
    p.append(txt(X0 + 4 * CW, yb + 4 * RH + 26,
                 "acks r0.. mean “burst in flight”, and window period = max(fill, flush) = fill", 8.5, "start"))
    return f'<svg viewBox="0 0 682 260" role="img" aria-label="Buffer read and write cycle-level schedules">{"".join(p)}</svg>'

def fig_results_chart():
    """Grouped horizontal bars: workload categories and phase-7 patterns,
    absolute cycles per backend. Reads the module-level totals computed
    above (called from the page template after parsing)."""
    BK = [("Crossbar", "#c9a97e"), ("TDM·RR", "#9a9ba3"), ("TDM·adaptive", "#7d9fc7")]
    p = [SVG_DEFS]

    def bar_group(x0, y0, w, title, rows):
        out = [txt(x0, y0, title, 10.5, "start", "ff", "#1b1b1f", "font-weight:600")]
        vmax = max(v for _, vals in rows for v in vals)
        y = y0 + 12
        for label, vals in rows:
            out.append(txt(x0 + 108, y + 12, label, 9, "end"))
            for k, v in enumerate(vals):
                bw = max(2, v / vmax * (w - 190))
                out.append(f'<rect x="{x0 + 116}" y="{y + k * 9}" width="{bw:.1f}" height="7" '
                           f'fill="{BK[k][1]}" stroke="#1b1b1f" stroke-width="0.4"/>')
                out.append(txt(x0 + 120 + bw, y + k * 9 + 6, str(v), 7.5, "start", "fm"))
            y += 36
        return "".join(out), y

    cats = [
        ("conflict-free (75 tasks)", [cf_total["crossbar"], cf_total["tdm"], cf_total["tdm-adaptive"]]),
        ("bank conflicts (own-map)", [c_total["crossbar"], c_total["tdm"], c_total["tdm-adaptive"]]),
        ("same-bank streams (own-map)", [sb_total["crossbar"], sb_total["tdm"], sb_total["tdm-adaptive"]]),
        ("structural classes (phase 7)", [sum(r[3] for r in p7_rows), sum(r[4] for r in p7_rows),
                                           sum(r[5] for r in p7_rows)]),
        ("full parallel, 9 AGUs (phase 8)", [p8_total["crossbar"], p8_total["tdm"],
                                             p8_total["tdm-adaptive"]]),
    ]
    g1, y_end1 = bar_group(8, 18, 666, "(a) total cycles by workload category", cats)
    p.append(g1)

    # phase 7 patterns, read+write summed per backend
    by_pat = {}
    for label, d, n, xb, rr, ad, sp in p7_rows:
        e = by_pat.setdefault(label, [0, 0, 0])
        e[0] += xb; e[1] += rr; e[2] += ad
    pats = [(k, v) for k, v in by_pat.items()]
    g2, y_end2 = bar_group(8, y_end1 + 16, 666, "(b) phase 7 structural patterns (read + write cycles)", pats)
    p.append(g2)

    # legend
    lx = 480
    for k, (name, color) in enumerate(BK):
        p.append(f'<rect x="{lx}" y="{6 + k * 11}" width="10" height="7" fill="{color}" '
                 f'stroke="#1b1b1f" stroke-width="0.4"/>')
        p.append(txt(lx + 14, 12 + k * 11, name, 8.5, "start"))
    h = y_end2 + 8
    return f'<svg viewBox="0 0 682 {h}" role="img" aria-label="Timing results overview">{"".join(p)}</svg>'

def fig_buffer_cells():
    p = [SVG_DEFS]
    # ---- (a) read cell
    p.append(txt(10, 18, "(a) Read cell (prefetch)", 12.5, "start", "ff", "#1b1b1f", "font-weight:600"))
    p.append(box(10, 32, 300, 46, ["fetch engine", "req → gnt → data  (2-cycle round trip)"], "#e3ecf7"))
    p.append(box(10, 106, 300, 46, ["stored value", "data_q, valid"], "#eceff4"))
    p.append(box(10, 180, 300, 46, ["port drain", "is_fwd mux: stored / just-arrived"], "#eceff4"))
    p.append(arrow(160, 78, 160, 106))
    p.append(arrow(160, 152, 160, 180))
    # restart loop, right side
    p.append(
        '<path d="M 310 203 C 344 203 344 55 310 55" fill="none" stroke="#575862" '
        'stroke-width="1.1" stroke-dasharray="4 3" marker-end="url(#ah)"/>'
    )
    p.append(txt(330, 126, "restart:", 9.5, "start"))
    p.append(txt(330, 139, "!pending ∧ en", 9.5, "start", "fm"))
    p.append(txt(330, 152, "∧ (all_valid", 9.5, "start", "fm"))
    p.append(txt(330, 165, "  ∨ !valid)", 9.5, "start", "fm"))
    p.append(txt(160, 250, "each cell refetches the instant its own group drains — no window-level trigger", 9.5))
    # ---- (b) write cell
    xo = 440
    p.append(txt(xo, 18, "(b) Write cell (fill + shadow)", 12.5, "start", "ff", "#1b1b1f", "font-weight:600"))
    p.append(box(xo, 32, 232, 46, ["primary latch", "addr / data / be from port lane"], "#eceff4"))
    p.append(box(xo, 122, 232, 46, ["shadow flush engine", "req until gnt; freed AT the grant"], "#e3ecf7"))
    p.append(box(xo, 196, 232, 46, ["bank", "samples payload the edge after gnt"], "#eceff4"))
    p.append(arrow(xo + 30, 78, xo + 30, 122))
    p.append(txt(xo + 44, 96, "snapshot (1-cycle pulse): window →", 9.5, "start"))
    p.append(txt(xo + 44, 108, "shadows, primaries freed same edge", 9.5, "start"))
    p.append(arrow(xo + 116, 168, xo + 116, 196))
    p.append(txt(xo + 116, 264, "posted ack: p_rvalid streams one group per", 9.5))
    p.append(txt(xo + 116, 276, "cycle behind the snapshot (burst in flight)", 9.5))
    return f'<svg viewBox="0 0 682 292" role="img" aria-label="Read and write buffer cell internals">{"".join(p)}</svg>'

# --------------------------------------------------------------- table helpers
def booktabs(headers, rows, caption, num, aligns=None, foot=None):
    aligns = aligns or ["l"] + ["r"] * (len(headers) - 1)
    th = "".join(f'<th class="a{a}">{h}</th>' for h, a in zip(headers, aligns))
    body = []
    for r in rows:
        tds = "".join(f'<td class="a{a}">{c}</td>' for c, a in zip(r, aligns))
        body.append(f"<tr>{tds}</tr>")
    tf = ""
    if foot:
        tds = "".join(f'<td class="a{a}">{c}</td>' for c, a in zip(foot, aligns))
        tf = f"<tfoot><tr>{tds}</tr></tfoot>"
    return (
        f'<figure class="tbl"><figcaption><span class="cap-label">Table {num}:</span> {caption}</figcaption>'
        f'<div class="scroll"><table><thead><tr>{th}</tr></thead><tbody>{"".join(body)}</tbody>{tf}</table></div></figure>'
    )

def phase_table(phase, title, num):
    rows = []
    for ports, n, lanes, exp, xb, rr, ad in cf_rows[phase]:
        mark = "✓" if (xb == exp and ad == exp) else "✗"
        rows.append((ports, n, lanes, exp, xb, rr, ad, mark))
    return booktabs(
        ["ports", "n<sub>data</sub>", "λ", "⌈n/λ⌉", "Crossbar", "TDM·RR", "TDM·adaptive", "parity"],
        rows, title, num,
        aligns=["r", "r", "r", "r", "r", "r", "r", "c"],
    )

# --------------------------------------------------------------------- the page
def S(x):  # section text shortcut: keeps f-strings below readable
    return x

phase_summary_rows = []
for phase, title in PHASES:
    rows = cf_rows[phase]
    xb = sum(r[4] for r in rows)
    rr = sum(r[5] for r in rows)
    ad = sum(r[6] for r in rows)
    ok = all(r[4] == r[3] and r[6] == r[3] for r in rows)
    phase_summary_rows.append((title.split("—")[1].strip(), len(rows), xb, rr, ad, "✓" if ok else "✗"))
phase_summary_rows.append(("<em>total conflict-free</em>", n_cf_tasks,
                           cf_total["crossbar"], cf_total["tdm"], cf_total["tdm-adaptive"],
                           "✓" if parity_ok else "✗"))

conflict_tbl_rows = [
    (p, n, k, xb, rr, ad, f"{xb / ad:.1f}×") for (p, n, k, xb, rr, ad) in conflict_rows
]
suite_rows = [(f"<code>{name}</code>", cnt, SUITE_FOCUS.get(name, "")) for name, cnt in suites]

appendix_tables = "\n".join(
    phase_table(phase, title, i + 10) for i, (phase, title) in enumerate(PHASES)
)

p5_class_tbl = [(c, p, n, xb, rr, ad) for (c, p, n, xb, rr, ad) in p5_class_rows]

# phase 6 own-map: port-side span next to bank-side occupancy, per build
p6_bank_rows = []
for d, ph, phb in (("write", "phase6_write", "phase6_write_bank"),
                   ("read", "phase6_read", "phase6_read_bank")):
    for ports in (1, 2, 4):
        p6_bank_rows.append((d, ports,
                             sp("crossbar", ph, ports, 128, "same_bank"),
                             sp("crossbar", phb, ports, 128, "same_bank"),
                             sp("tdm", ph, ports, 128, "same_bank"),
                             sp("tdm", phb, ports, 128, "same_bank"),
                             sp("tdm-adaptive", ph, ports, 128, "same_bank"),
                             sp("tdm-adaptive", phb, ports, 128, "same_bank")))
p7_total = {"crossbar": sum(r[3] for r in p7_rows), "tdm": sum(r[4] for r in p7_rows),
            "tdm-adaptive": sum(r[5] for r in p7_rows)}
p8_total = {"crossbar": sum(r[3] for r in p8_rows), "tdm": sum(r[4] for r in p8_rows),
            "tdm-adaptive": sum(r[5] for r in p8_rows)}
grand = {bk: cf_total[bk] + c_total[bk] + sb_total[bk] + p7_total[bk] + p8_total[bk]
         for bk in cf_total}

eval_tbl_rows = []
for ds, cb, ad, r8, _r9 in eval_rows:
    eval_tbl_rows.append((ds, cb, ad, r8, f"{cb / ad:.3f}&times;"))

eval_util_rows = []
for u in eval_util:
    eval_util_rows.append((
        u["dataset"], f'{u["cb_bank_util_pct"]}%', f'{u["ad_bank_util_pct"]}%',
        f'{u["ad_bus_busy_pct"]}%', f'{u["ad_contention_pct"]}%'))


eval_unf = []
_unf_path = DATA / "eval_unfenced.csv"
if _unf_path.exists():
    _lines = _unf_path.read_text().splitlines()
    _keys = _lines[0].split(",")
    for line in _lines[1:]:
        if not line.strip():
            continue
        eval_unf.append(dict(zip(_keys, line.split(","))))
eval_unf_rows = [
    (n["dataset"], n["cb_nf_cycles"], n["ad_nf_cycles"], n["r8_nf_cycles"],
     f'{n["ratio_ad_cb"]}&times;')
    for n in eval_unf]

def _rng(key, cast=float):
    vals = [cast(u[key]) for u in eval_util]
    return (min(vals), max(vals)) if vals else (0, 0)

util_cb_lo = util_cb_hi = busy_lo = busy_hi = 0.0
lvl1_cyc_lo = lvl1_cyc_hi = lvl2_cyc_lo = lvl2_cyc_hi = lvl3_cyc_hi = 0.0
if eval_util:
    util_cb_lo, util_cb_hi = _rng("cb_bank_util_pct")
    busy_lo, busy_hi = _rng("ad_bus_busy_pct")
    lvl1_cyc_lo, lvl1_cyc_hi = _rng("cb_lvl_l1_rate")
    lvl2_cyc_lo, lvl2_cyc_hi = _rng("cb_lvl_l2_rate")
    lvl3_cyc_hi = _rng("cb_lvl_l3_rate")[1]
wait_ratio_lo = wait_ratio_hi = epis_ratio_lo = epis_ratio_hi = 0.0
if eval_util:
    _wr = [int(u["ad_wait_cycles"]) / max(1, int(u["cb_wait_cycles"])) for u in eval_util]
    _er = [int(u["cb_episodes"]) / max(1, int(u["ad_episodes"])) for u in eval_util]
    wait_ratio_lo, wait_ratio_hi = min(_wr), max(_wr)
    epis_ratio_lo, epis_ratio_hi = min(_er), max(_er)
unf_ratio_lo = unf_ratio_hi = 0.0
unf_bus_lo = unf_bus_hi = 0.0
unf_bpt_lo = unf_bpt_hi = 0.0
if eval_unf:
    _ur = [float(n["ratio_ad_cb"]) for n in eval_unf]
    unf_ratio_lo, unf_ratio_hi = min(_ur), max(_ur)
    _ub = [float(n["ad_nf_bus_busy_pct"]) for n in eval_unf]
    unf_bus_lo, unf_bus_hi = min(_ub), max(_ub)
    _bt = [float(n["ad_nf_beats_per_turn"]) for n in eval_unf]
    unf_bpt_lo, unf_bpt_hi = min(_bt), max(_bt)

conf_cb_pct_lo, conf_cb_pct_hi = _rng("cb_conf_pct")
conf_ad_pct_lo, conf_ad_pct_hi = _rng("ad_conf_pct")
conf_lvl_l1_lo, conf_lvl_l1_hi = _rng("cb_lvl_l1_pct")

eval_set_conf_rows = []
for u in eval_util:
    _cb_tot = round(float(u["cb_l1_per_req"]) + float(u["cb_l2_per_req"])
                    + float(u["cb_l3_per_req"]), 1)
    eval_set_conf_rows.append((
        u["dataset"],
        f'{u["cb_l1_per_req"]}%', f'{u["cb_l2_per_req"]}%', f'{u["cb_l3_per_req"]}%',
        f'<strong>{_cb_tot}%</strong>',
        f'<strong>{u["ad_bank_per_req"]}%</strong>', u["ad_resolve_per_win"]))

def xt_table(groups, rows, caption, num):
    """booktabs-styled table with a two-row header: (label, span) groups on
    top, X/T sub-labels under every span-2 group. Row cells are flat, in
    group order (X before T within a pair)."""
    h1 = "".join(
        f'<th class="ac" rowspan="2">{g}</th>' if n == 1 else f'<th class="ac" colspan="2">{g}</th>'
        for g, n in groups)
    h2 = "".join('<th class="ac xt">X</th><th class="ac xt">T</th>' for g, n in groups if n == 2)
    body = []
    for r in rows:
        tds = [f'<td class="al">{r[0]}</td>'] + [f'<td class="ar">{c}</td>' for c in r[1:]]
        body.append(f"<tr>{''.join(tds)}</tr>")
    return (
        f'<figure class="tbl"><figcaption><span class="cap-label">Table {num}:</span> {caption}</figcaption>'
        f'<div class="scroll"><table><thead><tr>{h1}</tr><tr>{h2}</tr></thead>'
        f'<tbody>{"".join(body)}</tbody></table></div></figure>'
    )

def fig_bank_timeline():
    """Per-set small multiples: filled areas = per-bucket peak bank
    utilization (worst cycle in the bucket, % of each fabric's own bank
    count: crossbar/64, TDM/32); lines = share of cycles with >=1 real
    request stalled by contention. One 0-100% axis for both."""
    path = DATA / "eval_timeline.csv"
    if not path.exists():
        return ""
    per = {}
    for line in path.read_text().splitlines()[1:]:
        st, b, csp, asp, cpk, apk = line.split(",")
        per.setdefault(int(st), []).append(
            (float(csp), float(asp), 100.0 * int(cpk) / 64.0, 100.0 * int(apk) / 32.0))
    COLS, W, H, PX, PY = 5, 132, 64, 8, 14
    rowsn = (len(per) + COLS - 1) // COLS
    TW, TH = COLS * (W + PX) + 34, rowsn * (H + PY + 14) + 38
    CB, AD = "#8a8a93", "#26518f"
    p = [f'<svg viewBox="0 0 {TW} {TH}" style="max-width:100%;height:auto" role="img" '
         f'aria-label="bank utilization demand and stall pressure over time, per set">']
    p.append(f'<text x="34" y="10" font-size="9" fill="#575862">filled: peak bank utilization '
             f'per bucket (worst cycle, % of own banks — crossbar/64, TDM/32) &#183; '
             f'lines: share of cycles with &#8805;1 contention-stalled request (%)</text>')
    p.append(f'<text x="34" y="21" font-size="9" fill="#575862">'
             f'<tspan fill="{CB}">&#9644;</tspan> crossbar '
             f'<tspan fill="{AD}">&#9644;</tspan> TDM&#183;adaptive</text>')
    for st in sorted(per):
        r, c = divmod(st, COLS)
        ox, oy = 34 + c * (W + PX), 32 + r * (H + PY + 14)
        rows = per[st]
        n = len(rows)
        def xy(i, v):
            return f"{ox + i*(W-2)/(n-1):.1f},{oy + H - min(v,100.0)/100.0*H:.1f}"
        def line_pts(sel):
            return " ".join(xy(i, rows[i][sel]) for i in range(n))
        def area_pts(sel):
            top = " ".join(xy(i, rows[i][sel]) for i in range(n))
            return f"{ox},{oy+H} {top} {ox+W-2},{oy+H}"
        p.append(f'<rect x="{ox}" y="{oy}" width="{W-2}" height="{H}" fill="none" '
                 f'stroke="#d9d9de" stroke-width="0.6"/>')
        p.append(f'<line x1="{ox}" y1="{oy+H/2:.0f}" x2="{ox+W-2}" y2="{oy+H/2:.0f}" '
                 f'stroke="#ececea" stroke-width="0.6"/>')
        p.append(f'<polygon points="{area_pts(2)}" fill="{CB}" fill-opacity="0.20" stroke="none"/>')
        p.append(f'<polygon points="{area_pts(3)}" fill="{AD}" fill-opacity="0.16" stroke="none"/>')
        p.append(f'<polyline points="{line_pts(0)}" fill="none" stroke="{CB}" stroke-width="1"/>')
        p.append(f'<polyline points="{line_pts(1)}" fill="none" stroke="{AD}" stroke-width="1.1"/>')
        p.append(f'<text x="{ox}" y="{oy + H + 10}" font-size="8.5" fill="#575862">set {st}</text>')
        if c == 0:
            p.append(f'<text x="{ox-4}" y="{oy+7}" font-size="7.5" fill="#a5a5ad" '
                     f'text-anchor="end">100</text>')
            p.append(f'<text x="{ox-4}" y="{oy+H}" font-size="7.5" fill="#a5a5ad" '
                     f'text-anchor="end">0</text>')
    p.append("</svg>")
    return "".join(p)

def heatmap_matrix(caption, num):
    """SET x AGU contention-wait heatmap: per unit a TDM|xbar sub-cell pair,
    shaded on a shared log scale (sequential blue ramp), compact labels."""
    _mx_path = DATA / "eval_matrix.csv"
    if not _mx_path.exists():
        return ""
    _lines = _mx_path.read_text().splitlines()
    _keys = _lines[0].split(",")
    # The three organically-loaded units get their own columns; the five
    # near-idle ones (RAGU_C/D/E, WAGU_D/E — all &le; a few hundred cycles,
    # WAGU_B zero) fold into one "others" pair so the matrix fits the page.
    groups = ["ragu_a", "ragu_b", "wagu_a", "others"]
    rest   = ["ragu_c", "ragu_d", "ragu_e", "wagu_d", "wagu_e"]
    data = []  # per set: (dataset, cycles, {g: (ad, cb)})
    for line in _lines[1:]:
        if not line.strip():
            continue
        m = dict(zip(_keys, line.split(",")))
        d = {g: (int(m[f"{g}_ad"]), int(m[f"{g}_cb"])) for g in ["ragu_a", "ragu_b", "wagu_a"]}
        d["others"] = (sum(int(m[f"{g}_ad"]) for g in rest),
                       sum(int(m[f"{g}_cb"]) for g in rest))
        data.append((m["dataset"], int(m["cycles"]), d))
    tot = {g: [sum(d[g][k] for _, _, d in data) for k in (0, 1)] for g in groups}
    tot_cycles = sum(c for _, c, _ in data)

    # Cell = conflict cycle overhead: the unit's contention-wait as a share
    # of that run's total cycles (absolute cycles on hover).
    def cell(v, cycles):
        if v == 0:
            return '<td class="hm hm0">&mdash;</td>'
        pct = 100.0 * v / cycles
        txt = f"{pct:.2f}%" if pct < 1 else f"{pct:.1f}%"
        return f'<td class="hm" title="{v} wait cycles of {cycles}">{txt}</td>'

    def gname(g):
        return "others (5 units)" if g == "others" else g.upper().replace("_", "&#8202;")
    h1 = '<th class="hmh"></th>' + "".join(
        f'<th class="hmh" colspan="2">{gname(g)}</th>' for g in groups)
    h2 = '<th class="hms">set</th>' + "".join(
        '<th class="hms">X</th><th class="hms">T</th>' for _ in groups)
    body = []
    for ds, cyc, d in data:
        tds = "".join(cell(d[g][1], cyc) + cell(d[g][0], cyc) for g in groups)
        body.append(f'<tr><td class="hml">{ds}</td>{tds}</tr>')
    tf = '<tr><td class="hml"><em>total</em></td>' + "".join(
        cell(tot[g][1], tot_cycles) + cell(tot[g][0], tot_cycles) for g in groups) + "</tr>"
    def gain(g):
        ad, cb = tot[g]
        if ad == 0 and cb == 0:
            return '<td class="hm hm0" colspan="2">&mdash;</td>'
        if ad == 0:
            return '<td class="hm" colspan="2" style="text-align:center"><strong>&infin;</strong></td>'
        r = cb / ad
        txt = f"{r:.1f}&times;" if r < 10 else f"{r:.0f}&times;"
        return f'<td class="hm" colspan="2" style="text-align:center" title="{cb} / {ad} wait cycles"><strong>{txt}</strong></td>'
    tf += ('<tr><td class="hml"><em>TDM gain</em></td>' +
           "".join(gain(g) for g in groups) + "</tr>")
    return (
        f'<figure class="tbl"><figcaption><span class="cap-label">Table {num}:</span> {caption}</figcaption>'
        f'<div class="scroll"><table class="hmt"><thead><tr>{h1}</tr><tr>{h2}</tr></thead>'
        f'<tbody>{"".join(body)}</tbody><tfoot>{tf}</tfoot></table></div></figure>'
    )

eval_bymode_rows = []
_bm_path = DATA / "eval_bymode.csv"
if _bm_path.exists():
    _lines = _bm_path.read_text().splitlines()
    _keys = _lines[0].split(",")
    for line in _lines[1:]:
        if not line.strip():
            continue
        m = dict(zip(_keys, line.split(",")))
        eval_bymode_rows.append((
            m["mode"], m["windows"], f'{m["coll_pct"]}%', m["extra_cycles"],
            m["extra_per_window"], m["note"] or "&mdash;"))

eval_agu_rows = []
_agu_path = DATA / "eval_agu.csv"
if _agu_path.exists():
    _lines = _agu_path.read_text().splitlines()
    _keys = _lines[0].split(",")
    for line in _lines[1:]:
        if not line.strip():
            continue
        a = dict(zip(_keys, line.split(",")))
        _tc_cb = sum(cb for _, cb, _ad, _r8, _r9 in eval_rows) or 1
        _tc_ad = sum(ad for _, _cb, ad, _r8, _r9 in eval_rows) or 1
        def _k(v):
            v = int(v)
            return f"{v/1000:.0f}k" if v >= 10000 else (f"{v/1000:.1f}k" if v >= 1000 else str(v))
        def _pct(v, tot):
            p = 100.0 * int(v) / tot
            if int(v) == 0:
                return "&mdash;"
            if p < 0.01:
                return "&lt;0.01%"
            return f"{p:.2f}%" if p < 1 else f"{p:.1f}%"
        eval_agu_rows.append((
            a["agu"].replace("_", "&#8202;"),
            _k(a["cb_real_beats"]),
            _pct(a["cb_wait_cycles"], _tc_cb), _pct(a["ad_wait_cycles"], _tc_ad),
            _k(a["cb_episodes"]), _k(a["ad_episodes"]),
            _pct(a["cb_fill_wait"], _tc_cb), _pct(a["ad_fill_wait"], _tc_ad),
            f'{a["cb_active_pct"]}%', f'{a["ad_active_pct"]}%'))

page = f"""<title>A Windowed TDM Interconnect with Crossbar-Parity Timing</title>
<style>
  :root {{ color-scheme: light; }}
  body {{ background: #e8e8e5; margin: 0; }}
  .sheet {{
    max-width: 46rem; margin: 2.5rem auto; padding: 4rem 4.5rem 5rem;
    background: #fdfdfb; color: #1b1b1f;
    box-shadow: 0 1px 3px rgba(27,27,31,.18), 0 8px 28px rgba(27,27,31,.12);
    font: 1.02rem/1.62 Charter, "Bitstream Charter", "Sitka Text", Cambria, Georgia, serif;
  }}
  @media (max-width: 52rem) {{ .sheet {{ margin: 0; padding: 2rem 1.4rem 3rem; box-shadow: none; }} }}
  .ff {{ font-family: Charter, "Bitstream Charter", "Sitka Text", Cambria, Georgia, serif; }}
  .fm, code, pre {{ font-family: ui-monospace, "Cascadia Mono", Menlo, Consolas, monospace; }}
  code {{ font-size: .82em; background: #f0f0ee; padding: .05em .3em; border-radius: 2px; }}
  pre {{ font-size: .78rem; line-height: 1.5; background: #f5f5f2; border: 1px solid #d9d9de;
        padding: .8rem 1rem; overflow-x: auto; }}
  pre code {{ background: none; padding: 0; }}
  .hmt {{ border-collapse: collapse; font-size: .85rem; margin: 0 auto; }}
  .hmt th, .hmt td {{ padding: .22rem .55rem; }}
  .hmt .hm {{ text-align: right; font-variant-numeric: tabular-nums; }}
  .hmt .hm0 {{ color: #a5a5ad; }}
  .hmt .hml {{ text-align: right; }}
  .hmt thead tr:first-child th {{ border-top: 2px solid #1b1b1f; font-weight: 600;
                                  font-style: italic; text-align: center; }}
  .hmt .hms {{ font-weight: 600; font-style: italic; text-align: center;
               border-bottom: 1px solid #1b1b1f; font-size: .75rem; color: #575862; }}
  .hmt tfoot td {{ border-top: 1px solid #1b1b1f; border-bottom: 2px solid #1b1b1f; font-weight: 600; }}
  .xt {{ font-size: .75rem; color: #575862; }}
  h1 {{ font-size: 1.55rem; line-height: 1.25; text-align: center; font-weight: 700;
       margin: 0 0 .4rem; text-wrap: balance; }}
  .byline {{ text-align: center; font-variant: small-caps; letter-spacing: .06em;
            color: #575862; margin: 0 0 2.2rem; }}
  .abstract {{ margin: 0 2.2rem 2.4rem; font-size: .93rem; line-height: 1.55; }}
  .abstract b {{ font-variant: small-caps; letter-spacing: .05em; font-weight: 600; }}
  h2 {{ font-size: 1.14rem; margin: 2.4rem 0 .7rem; }}
  h3 {{ font-size: 1rem; margin: 1.7rem 0 .5rem; font-style: italic; font-weight: 600; }}
  p {{ margin: 0 0 .85rem; text-align: justify; hyphens: auto; }}
  ul {{ margin: 0 0 .85rem; padding-left: 1.4rem; }}
  li {{ margin-bottom: .3rem; text-align: justify; hyphens: auto; }}
  em.term {{ font-style: italic; }}
  figure {{ margin: 1.6rem 0 1.8rem; }}
  figure svg {{ width: 100%; height: auto; display: block; }}
  figcaption {{ font-size: .85rem; color: #575862; text-align: center; margin-top: .55rem;
               line-height: 1.45; }}
  .tbl figcaption {{ margin: 0 0 .5rem; }}
  .cap-label {{ font-weight: 600; color: #1b1b1f; }}
  .scroll {{ overflow-x: auto; }}
  table {{ border-collapse: collapse; margin: 0 auto; font-size: .85rem;
          font-variant-numeric: tabular-nums; line-height: 1.4; }}
  th, td {{ padding: .28rem .7rem; }}
  thead th {{ border-top: 2px solid #1b1b1f; border-bottom: 1px solid #1b1b1f;
             font-weight: 600; font-style: italic; }}
  tbody tr:last-child td {{ border-bottom: 2px solid #1b1b1f; }}
  tfoot td {{ border-bottom: 2px solid #1b1b1f; border-top: 1px solid #1b1b1f; font-weight: 600; }}
  tbody + tfoot {{ position: relative; }}
  table:has(tfoot) tbody tr:last-child td {{ border-bottom: none; }}
  .al {{ text-align: left; }} .ar {{ text-align: right; }} .ac {{ text-align: center; }}
  a {{ color: #26518f; text-decoration: none; }}
  a:focus-visible {{ outline: 2px solid #26518f; outline-offset: 2px; }}
  .fnote {{ font-size: .84rem; color: #575862; border-top: 1px solid #d9d9de;
           margin-top: 2.8rem; padding-top: .8rem; }}
</style>
<div class="sheet">
<h1>A Windowed TDM Memory Interconnect with Crossbar-Parity Timing</h1>
<p class="byline">rtl-lab / projects / tdm · SystemC reference model · {html.escape("July 8, 2026")}</p>

<div class="abstract">
<p><b>Abstract.</b> This report closes out the evaluation of a multi-requester banked-memory
subsystem modeled in SystemC, comparing two interconnect backends behind one common top-level
interface: a three-level <em>crossbar</em> (64 physical banks, 68&times;64 switch fabric) and a
<em>time-division-multiplexed bus</em> (one 32-wide bus, 32 banks) fronted by per-requester
prefetch and write-combining buffers. On a synthetic sweep of {n_cf_tasks} conflict-free tasks
the TDM datapath matches the crossbar cycle-for-cycle (every span equals the closed form
⌈n<sub>data</sub>/λ⌉) and pulls ahead — up to 4.9&times; — wherever a workload turns
adversarial. On 20 production-representative stimulus sets it finishes in <em>exactly</em> the
crossbar's cycle count on 19 and beats it on the heaviest, at 16&ndash;27% single-bus load
where the crossbar idles at ~2% bank utilization, with 2&ndash;15&times; less contention
wait than the crossbar and — via a hidden-lookahead prefetch — essentially zero task-start latency,
both absorbed inside the schedule's own fence gaps. The evaluation additionally surfaced
and resolved three task-boundary deadlock classes (now regression-locked among
{n_tests}&thinsp;assertions across {len(suites)} suites) and one stimulus-format error,
diagnosed through the anomalous conflict rates reported by the per-beat accounting. Even with the schedule stripped, the single bus stays within 8% of the crossbar on the
light sets and leads by up to 1.27&times; on the heavy ones — against a crossbar that
already includes the L1 hash repair this evaluation itself motivated. Recommendation: the TDM backend with the request-aware (adaptive) arbiter and
lookahead lead 16.</p>
</div>

<h2>1&emsp;System overview</h2>
<p>The system serves nine address-generation units (AGUs): five read groups
(RAGU_A with 4 ports, RAGU_B with 2, RAGU_C, RAGU_D and RAGU_E with 1 each) and four write
groups (WAGU_A with 4 ports, WAGU_B with 2, WAGU_D and WAGU_E with 1 each). The E groups are
driven per-lane by <code>lane_agu</code> (today's DMA engines; the driver is unit-agnostic). Every
port carries NUM_REQ&thinsp;=&thinsp;4 independent OBI request lanes, so the fabric sees 36 read
and 32 write lanes. Storage is 32 logical banks of 1024 rows, 16 bytes per row. A lane issues a
16-byte beat per request; a group of ports working one task therefore moves
λ&thinsp;=&thinsp;ports&thinsp;×&thinsp;4 beats per cycle when nothing stalls, which makes
⌈n<sub>data</sub>/λ⌉ the ideal span for a task of n<sub>data</sub> beats — the yardstick used
throughout §4.</p>
<p>Both backends implement the same OBI contract (req/gnt forward, rvalid/rdata return, one-cycle
bank latency) behind an identical <code>top&lt;&gt;</code> wrapper, so testbenches, stimuli, and
the AGU drivers are shared verbatim and any timing difference is attributable to the
interconnect alone. Both ends of the datapath are common, and only the middle differs: the
requester side — the five read and four write AGU groups with their 36&thinsp;+&thinsp;32 OBI
lanes — is the same top-level interface wired to either backend (the boxes labeled
“read/write groups” in Figures 1 and 2 are the same ports), and the storage is literally
shared code: both tops instantiate the same single-port <code>bank&lt;&gt;</code> module
(128-bit rows, combinational grant, one-cycle response). The crossbar splits each logical bank into two half-depth physical instances —
64&thinsp;×&thinsp;512 rows — so its L3 stage can serve a read and a write concurrently on the
even/odd halves, while the TDM top keeps the 32 full-depth instances —
32&thinsp;×&thinsp;1024 rows. Capacity is identical either way:
32&thinsp;×&thinsp;1024&thinsp;×&thinsp;16&thinsp;B&thinsp;=&thinsp;512&thinsp;KiB. Figures 1 and 2
show the full wiring of each backend.</p>

<h2>2&emsp;The two architectures</h2>
<h3>2.1&emsp;Crossbar backend</h3>
<figure>
{fig_crossbar_wiring()}
<figcaption><span class="cap-label">Figure 1:</span> Crossbar backend wiring. Solid arrows are
OBI request bundles (req, addr, we, be, wdata); the dashed path is the shared return
(gnt, rvalid, rdata). Read and write traffic have private hash/L1/L2 planes and meet only in
the 32 L3 merge stages and the 64 physical banks.</figcaption>
</figure>
<p>The crossbar is a three-level switch. An address hash first scrambles the L2-select bits
(adding the overlapping <code>addr[11:9]</code> field into <code>addr[8:6]</code>) to spread
strided traffic; the evaluation of §5 additionally enables an L1 extension of the same hash
(<code>XBAR_HASH_L1</code>, folding <code>addr[11:10]</code> into the L1-select bits), a
repair this study itself motivated — §5.2 reports the measured effect and Appendix A.8 the diagnosis,
while the synthetic sweep of §4 uses the original hash it was pinned against. L1 is one
4×4 crossbar per port that routes the port's four lanes by <code>addr[4+:2]</code>; L2 routes across bank groups by <code>addr[6+:3]</code>; L3 is a 2×2
stage per logical bank that merges the separate read and write paths onto even/odd physical
banks by <code>addr[9]</code>. The routing field is stripped before the address reaches the
bank. Read and write traffic have private L1/L2 planes and share only L3 and the 64 physical
banks, so the backend sustains full read and write bandwidth simultaneously — at the cost of
(36&thinsp;+&thinsp;32) lane-wide switching fabric.</p>

<h3>2.2&emsp;TDM backend</h3>
<figure>
{fig_tdm_wiring()}
<figcaption><span class="cap-label">Figure 2:</span> TDM backend wiring. One buffer's 32-lane
OBI owns the bus per cycle (<code>sel_req</code>); the arbiter's one-cycle-delayed
<code>sel_rsp</code> steers grants and read data back to the owning buffer. The mapping
parameters follow the bus ownership through the same select.</figcaption>
</figure>
<p>The TDM backend replaces the switch fabric with one 32-lane bus granted to a single
requester group per cycle. Each group owns a <em>buffer</em> of 32 cells (one per TDM slot)
that decouples the group's port-side pacing from its bus turns: read buffers prefetch a full
window of upcoming addresses supplied by their AGU's lookahead bus, and write buffers
accumulate a full window before bursting it. An arbiter selects which buffer owns the bus each
cycle — either a free-running round-robin counter (<em>TDM·RR</em>) or a request-aware scanner
(<em>TDM·adaptive</em>) that skips idle buffers while staying round-robin fair among active
ones. The granted window then passes through the <em>TDM map</em>, a purely combinational
XOR-skewed banking function that places each 4-byte word of the group into a
(bank,&thinsp;row) location such that common strides hit 32 distinct banks; collisions that do
remain are serialized by the per-bank arbiters downstream. One naming nuance is worth pinning
down: the “word” lives only inside the map — each lane logically carries one 32-bit kernel
word, and the map decides that word's (bank,&thinsp;row). The plumbing after it never touches
words: the map re-encodes each lane to an ordinary byte address
((row·32&thinsp;+&thinsp;bank)·16&thinsp;B) and the bank router — the same crossbar primitive
the crossbar backend builds its levels from — routes the whole 128-bit OBI beat by plain
beat-interleaved decode (bank&thinsp;=&thinsp;beat&thinsp;mod&thinsp;32), onto exactly the
same wires and bank module the crossbar backend uses.</p>
<p>The buffers give the caller exactly one cursor contract: a one-cycle
<code>window_reset</code> pulse means “advance the lookahead one window.” The same pulse serves
both the steady-state wrap and the return from idle, so the AGU side needs no special cases.</p>

<h2>3&emsp;Verification</h2>
<p>Every module has a hand-driven unit suite, and the full system is exercised by a shared
stimulus program (eight phases: single-group reads, multi-port reads, read/write interleave,
mixed requesters, deliberate bank-conflict patterns, same-bank streaming — 128 writes
routed entirely to one bank, then read back — structural conflict classes — intra-port,
inter-port, and read/write-same-target, realized against each backend's own routing — and a
full-parallel finale driving all nine AGUs at once) run against all three builds with
routing, data-integrity, and timing checks. The conflict phase additionally runs cross-mapped
— the pattern adversarial to the sibling backend's routing (§4) — since “same bank” is only
defined relative to a mapping function; the structural-class phase sequences each class's
read and write streams so every span prices one direction's own conflicts (only the
R/W-same-target class keeps both concurrent — their interaction is its definition). The two conflict-free-deterministic builds
(crossbar and TDM·adaptive) additionally assert every task's measured span
<em>exactly</em> equals ⌈n<sub>data</sub>/λ⌉ in-suite, so the parity result of §4 is a
regression-checked invariant, not a one-off measurement. Two suites exist specifically to
close structural blind spots: <code>crossbar</code> exercises the beat-interleaved primitive
standalone (both decode modes, per-bank round-robin order, owner-register response steering,
ghost-response rejection — previously only reachable through the tops), and
<code>buffer_modes</code> instantiates 4-port and 1-port buffers to pin the
<code>active_mode</code>-3 alias and the PORT_COUNT clamp, geometry paths the PORT_COUNT=2
main suites structurally cannot distinguish.</p>
{booktabs(["suite", "tests", "focus"], suite_rows,
          f"Test suites — {n_tests} assertions total, all passing.", 1,
          aligns=["l", "r", "l"],
          foot=("<em>total</em>", n_tests, ""))}

<h2>4&emsp;Timing results: synthetic sweep</h2>
<p>Every span in this section — Table 2's and all that follow — is measured PORT-side, first
response cycle to last response cycle inclusive, using each driver's own logged response
timestamps. That interval starts only once data is already flowing: it excludes whatever
happened between a task becoming eligible and its first beat coming back, which for TDM reads
is the first window's prefetch round trip and for TDM writes (posted acks) is fill and
snapshot before the first ack. Table 3 measures that excluded interval directly, so it is not
silently absorbed into the "TDM matches the crossbar" headline below — see its own discussion
before that headline is taken at face value. Table 2 reports, per stimulus phase, the summed
port-side spans over all conflict-free tasks; Tables 4–5 break out the conflict phase and
Table 6 the same-bank phase. The principal result: the crossbar and TDM·adaptive columns are
identical on every one of the {n_cf_tasks} conflict-free tasks —
{"confirmed" if parity_ok else "NOT confirmed"} against the closed form — while under full
same-bank conflicts the XOR-skewed TDM backend is up to
{max(xb / ad for _, _, k, xb, _, ad in conflict_rows if k == "full"):.1f}× faster than the
crossbar (Table 4), and never slower than
{min(xb / ad for _, _, k, xb, _, ad in conflict_rows if k == "full"):.1f}× at any port count.
Per-task listings are in Appendix A.</p>
{booktabs(["phase", "tasks", "Crossbar", "TDM·RR", "TDM·adaptive", "= ⌈n/λ⌉"],
          phase_summary_rows,
          "Conflict-free spans by phase (summed cycles). The parity column checks both the crossbar and TDM·adaptive against the closed form.", 2,
          aligns=["l", "r", "r", "r", "r", "c"])}
{booktabs(["scenario", "direction", "Crossbar", "TDM·RR", "TDM·adaptive"],
          fill_rows,
          "Pipeline fill latency — first-response cycle minus the task's own eligibility cycle, for the one genuinely idle-buffer moment in the whole sweep (cycles; not summed into any other table). This is the interval Table 2's port-side spans exclude.", 3,
          aligns=["l", "l", "r", "r", "r"])}
<p>The fill cost is real but small on the read side — 2–3 cycles for the crossbar's one bank
round trip, 4 for a TDM read (buffer boot-latch detection plus its own 2-cycle fetch) — and it
is paid <em>once</em> per idle-to-active transition, not once per beat, so it vanishes into
the noise for any task with n<sub>data</sub>&thinsp;≫&thinsp;λ. It is not negligible on the
TDM write side: 10 cycles for a cold WAGU boot, because the very first ack cannot be posted
until fill, snapshot, <em>and</em> the shadow's own bus round trip have all happened in
sequence — a real, one-time latency floor the posted-ack model does not hide; it simply does
not appear inside a port-to-port span once acks are already streaming. The idle-restart reads
(phase 3/4) show the one place TDM·RR pays extra for something TDM·adaptive does not: 7 cycles
vs 4, because the free-running arbiter's 1-in-9 rotation can make even a single restart wait
out other buffers' idle slots before the request is ever seen.</p>
{booktabs(["ports", "n<sub>data</sub>", "conflict", "Crossbar", "TDM·RR", "TDM·adaptive", "speed-up"],
          conflict_tbl_rows,
          "Phase 5 — deliberate bank-conflict patterns, own-mapped: each backend runs the pattern adversarial to its own routing (spans in cycles; speed-up is crossbar / TDM·adaptive). “Full” = every lane of a group targets one bank, but each of the 8 groups picks a fresh bank — up to 8 banks still work in parallel, which is why spans can undercut n<sub>data</sub> here (the one-bank extreme is phase 6). On the TDM builds every pick that is <em>meant</em> to be conflict-free (none-kind lanes, partial fillers, each group's fresh bank) is additionally enforced bank-distinct across the whole 32-beat fetch window, so cross-cycle batching in the buffer cannot add accidental conflicts on top of the intended per-group ones (verified: the guard never rejects a pick — the sequential allocator under the XOR map already rotates through all 32 banks).", 4,
          aligns=["r", "r", "l", "r", "r", "r", "r"],
          foot=("", "", "<em>total</em>", c_total["crossbar"], c_total["tdm"], c_total["tdm-adaptive"],
                f"{c_total['crossbar'] / c_total['tdm-adaptive']:.1f}×"))}
<p>Table 4 needs a methodological caveat, which Table 5 resolves: “full conflict” is only
defined <em>relative to a mapping</em>. The two backends hash addresses differently, so each
build's conflict set is constructed against its own routing — the crossbar's 128 and
TDM·adaptive's 56 in the last row of Table 4 are both worst cases, but not the same address
pattern. For an equal-footing comparison every conflict pattern class therefore also runs
<em>cross-mapped</em>: the set that is same-bank under the crossbar's hash is fed to the TDM
builds, and the set that is same-bank under the TDM map is fed to the crossbar. (The concrete
addresses still differ per build — the pattern searches consume different candidates — so these
rows compare the structural class, not a byte-identical stream.)</p>
{booktabs(["pattern class", "ports", "n<sub>data</sub>", "Crossbar", "TDM·RR", "TDM·adaptive"],
          p5_class_tbl,
          "Phase 5, cross-mapped — each full-conflict pattern class measured on all three backends; the informative cells are off-diagonal, where a backend faces the pattern adversarial to the other's routing.", 5,
          aligns=["l", "r", "r", "r", "r", "r"])}
<p>Phase 6 is the degenerate extreme of phase 5: not a few contending lanes inside otherwise
well-spread groups, but an entire task serialized through one bank's arbiter. On its own
worst case, port count cannot help — every beat waits on the same bank — so each build's spans
are essentially flat, and what separates the columns is the per-beat cost of reaching that
bank. The crossbar takes exactly n<sub>data</sub>&thinsp;=&thinsp;128 cycles: one beat per
cycle, with the first response arriving immediately, so the span sees the full serialization.
TDM·adaptive lands <em>below</em> 128 on its own worst case — the bank is no faster, but spans
measure first-to-last response and the windowed pipeline moves serialization out of that
interval on both sides: a read's whole first window is prefetched before the first response
exists, and a write's posted acks track snapshots rather than bank commits. TDM·RR additionally pays the free-running rotation's idle-slot penalty on every
serialized beat.</p>
<p>The sub-n<sub>data</sub> figure warrants scrutiny — n<sub>data</sub> beats through
one single-port bank cannot take fewer than n<sub>data</sub> cycles of <em>bank</em> time. The
invariant holds — at the bank rather than at the port. Table 6 puts both metrics side by
side, measured from the per-cycle bank-wire samples: the bank-side occupancy (first to last
accepted beat at the bank) is ≥&thinsp;128 on every backend, every task — asserted in-suite as
a physical-invariant check — while the port-side span is what a requester actually experiences
once the pipeline is flowing. The two coincide on the crossbar (its first response arrives
immediately, so the port sees the whole serialization); they diverge on TDM exactly by the
window the pipeline hides.</p>
{booktabs(["", "ports", "Crossbar port", "bank", "TDM·RR port", "bank", "TDM·adaptive port", "bank"],
          p6_bank_rows,
          "Phase 6 — same-bank streaming (128 beats written through one bank of the build's OWN routing, then read back): port-side span vs. bank-side occupancy (cycles). Bank-side occupancy is pinned to its closed form: exactly 128 on crossbar and TDM·adaptive (zero inter-window bubbles), exactly 1144 = 127·9+1 on TDM·RR. The cross-mapped same-bank flavor was dropped from the suite: a set that is same-bank under the OTHER backend's routing just scatters into ordinary conflict-free traffic here, which the free phases already measure.", 6,
          aligns=["l", "r", "r", "r", "r", "r", "r", "r"])}
<p>The cross-mapped rows answer the fairness question directly, and asymmetrically: the TDM
map <em>defuses</em> the crossbar's worst case almost completely — a stream fully serialized
on the crossbar spreads under the XOR skew to near-conflict-free speed on TDM·adaptive — while
the crossbar's hash only partially defuses the TDM-adversarial sets (same-bank-under-the-map
addresses still cluster onto a handful of crossbar banks, landing between its best and worst
case). So on every pattern class in Table 5, TDM·adaptive is at least as fast as the
crossbar, usually much faster — the conclusion the own-mapped tables alone could not support.
All cross-mapped spans are pinned as exact per-build regression constants, like everything
else in this section.</p>
<p>Phase 7 measures <em>structural</em> conflict classes, defined backend-agnostically and —
like phase 5 — realized against each backend's <em>own</em> routing, so neither column ever
coasts through the other's worst case as accidentally-free traffic. Three classes plus a
conflict-free baseline and noise, each a 4-port read stream and a 4-port write stream (256
beats each; noise is 4096 pseudo-random beats each). The two directions run <em>sequenced</em>
(the write stream fenced behind its read sibling) so each span measures that direction's own
conflict cost — on the crossbar the separate read/write networks never interleave anyway,
while on TDM's single bus two concurrent conflicted streams merely time-share, roughly
doubling both spans without saying anything new about either. The one exception is <em>R/W
same-target</em>, which keeps both streams concurrent: their interaction <em>is</em> that
class's definition. The classes: <em>intra-port</em> — a port
conflicts with itself, its four lanes sharing one route (crossbar: one L1 output, 4:1 inside
the port's own L1 switch; TDM: one bank per port, 4:1 at the bank); <em>inter-port</em> — the
four ports collide with each other on one route per lane (crossbar: one L2 group, 4:1 in L2;
TDM: one bank per lane index — its single bus + map play the role of L1+L2 combined, so both
port-conflict classes reach it as the same thing: a 4:1 bank conflict); and <em>R/W
same-target</em> — the read and write streams, each internally conflict-free, aimed at the
same 16 banks (crossbar: every beat meets the other stream in the L3 merge, 2:1; TDM: there
is no merge to meet — the two buffers already own different bus turns). The crossbar build
constructs exact post-hash L1/L2/L3 field targets; the TDM builds construct exact
<code>map_one()</code> bank targets with the same search machinery phases 5–6 use.</p>
{booktabs(["pattern", "", "n<sub>data</sub>", "Crossbar", "TDM·RR", "TDM·adaptive", "speed-up"],
          p7_rows,
          "Phase 7 — structural conflict classes, each realized against the backend's OWN routing; per class a 4-port read stream then its 4-port write stream, sequenced (R/W same-target alone runs both concurrently — the interaction is its definition). Spans in cycles; speed-up is crossbar / TDM·adaptive.", 7,
          aligns=["l", "l", "r", "r", "r", "r", "r"])}
<p>The results follow directly from each backend's structure. Two of the crossbar's
values merit explicit derivation. <em>Free</em> is
16: sixteen frames, one response cycle each, and span counts first-to-last inclusive.
<em>L3</em> is 31, not 32: the merge serves each physical bank's read and write in strict
alternation, so one stream's 16 responses land every <em>other</em> cycle —
2·(16−1)+1&thinsp;=&thinsp;31 for that stream's own first-to-last window, while the read/write
<em>pair</em> does occupy the full 2·16&thinsp;=&thinsp;32 cycles of bank time. (Its noise
spans also eased from ~851 to ~808–810 with the sequencing: the two random 4096-beat sets
used to collide with each other at the physical banks on top of their own self-conflicts.)</p>
<p>The TDM columns now say something real about TDM, and every class lands at or ahead of
the crossbar. <em>Conflict-free</em> is exact parity: 16 read and 16 write —
n/λ, the closed form, on a shared bus. The two <em>port-conflict</em> classes come in
<em>under</em> the crossbar's 64: both are the same genuine 4:1 bank conflict to the map
(intra and inter are indistinguishable to it — the rows match at 58–60), and the 64 cycles of
bank serialization are partly hidden by the window prefetch, which overlaps the first window's
conflicts with the pipeline fill the span never sees; the crossbar's switch stages serve the
same 4:1 at exactly 64, all of it inside the span. <em>R/W same-target</em> stays the
designed-for headline: TDM·adaptive reads 20 / writes 16 against the crossbar's 31/31 — the
two streams aim at the very same 16 banks, and where every crossbar beat pays the L3 merge's
2:1 alternation, the TDM buffers simply own different bus turns (the read's 20 vs its solo 16
is the residual cost of sharing the bus with the concurrent write stream — one exposed slot
per window). Random noise inverts the old statistical parity: 492/469 vs the crossbar's
808/810, because the whole accumulated 32-beat window attacks the banks at once and the XOR
map resolves a random window's collisions in fewer bus turns than the crossbar's per-frame
switch conflicts resolve in cycles.</p>
<p>The conflict-free case took a two-stage diagnostic detour worth recording, because each
stage removed a different real defect. It first measured 35/26 — 2× the intuitive model of
“the bus fetches a whole 32-slot window per turn, the port drains 16 per cycle, 16 cycles for
256 beats” — and a per-cycle bank-beat profile showed every window arriving as two 16-beat
halves. Stage one was the <em>stimulus</em>: “conflict-free” had been generated per 16-beat
frame, so each 32-slot fetch window hit every bank twice and the window's accumulated
32-request batch was halved at the banks. Window-distinct generation brought writes to
exactly 16 but left reads at 23 — one leftover cycle per window. Stage two was the
<em>arbiter</em>: the request-aware selector registered its whole scan, sampling requests at
the clock edge and granting a cycle later, so each window turnaround (where the lone buffer's
request drops for one cycle) paid an extra bus re-acquisition cycle. Rebuilt as a
combinational priority encoder over the registered fairness pointer — which is what
request-aware arbitration means in hardware anyway — the grant lands in the request's own
cycle and <strong>reads and writes both measure exactly n/λ&thinsp;=&thinsp;16</strong>. The
same fix is what moved phase 5's 4-port conflict-free run from 11 to its ideal 8. Kept from
the detour: the <code>STIM_P7_PROFILE</code> diagnostic, and the note at the read cell's
start rule recording the parked window-batched-launch prototype (no longer needed for this
case — per-group launch plus the same-cycle grant already sustains n/λ). All ten spans are
pinned per build (thirty across the three builds).</p>
<p>Phase 8 closes the sweep at the opposite extreme from every phase before it: <em>all
nine</em> AGUs at once, 4096 beats each — RAGU_A/B and WAGU_A/B in 2-port mode, RAGU_C/D and
WAGU_D in 1-port mode, and RAGU_E/WAGU_E driven through their <code>lane_agu</code> traces
(1024 wide 2-beat tasks per sub-port), the first real traffic through the E-group buffers in
this suite. Each stream is conflict-free <em>within itself against its own build's routing</em>
— like phases 5–7, the crossbar build constructs distinct L1/L2/L3 routes per frame and the
TDM builds walk all 32 map banks per window — with per-stream offsets so the only contention
is the genuine cross-stream kind. (At this scale nine mutually-fresh streams would need more
physical slots than the memory has, so the phase — deliberately the last — is row-partitioned
instead: reads in the low half of every bank's rows, writes in the high half, reads free to
alias one another; every beat is still routing-verified and every content check accounts for
what its folded slot actually holds.) The E streams get the same discipline on all four of
their lanes: every 32-beat window is one aligned run of 32 distinct banks, and
<code>lane_agu</code> packs up to eight tasks per sub-port per window (an earlier version
froze ONE task per sub-port per window — 4 real beats amid 28 NOPs — a ~4× driver-side
ceiling that had nothing to do with the fabric). Because per-stream spans cannot see
cross-stream cost, the phase's headline metric is its <strong>overall wall clock</strong> —
first response of any stream to last response of any stream, so every conflict, arbitration
decision and skew is priced in. The result: 36864 beats over a bus that moves at most 32
beats per turn have a hard 1152-cycle floor, and <strong>both TDM builds run the whole phase
in 1161–1162 cycles — 99.2% sustained bus utilization</strong> — with all nine streams
finishing within ~10 cycles of one another (the time-slicing is exactly fair, and RR ties
adaptive: at saturation there are no idle slots to skip, so the arbiter gap seen everywhere
else in this report vanishes). The crossbar's 68-lane fabric finishes the same workload in
the <em>same</em> 1161 overall — its aggregate bandwidth advantage is bounded by its slowest
stream (the 4-lane E groups, 1161/1151), while its other streams finish unevenly at
977–1057. One bus, at full load, matches the full switch fabric wall-clock for wall-clock —
which is the design's entire premise.</p>
{booktabs(["AGU", "", "ports", "Crossbar", "TDM·RR", "TDM·adaptive"],
          p8_rows,
          "Phase 8 — all nine AGUs in parallel, 4096 beats per stream (36864 total), each stream conflict-free against its own build's routing (per-stream spans in cycles). The footer is the phase's OVERALL wall clock — first response of any stream to last response of any stream — so cross-stream conflicts, arbitration and skew are all inside it. The TDM bus's hard floor for 36864 beats is 1152 turns.", 8,
          aligns=["l", "l", "r", "r", "r", "r"],
          foot=("<em>overall wall clock</em>", "", "", p8_overall["crossbar"], p8_overall["tdm"],
                p8_overall["tdm-adaptive"]))}
{booktabs(["", "Crossbar", "TDM·RR", "TDM·adaptive"],
          [("conflict-free", cf_total["crossbar"], cf_total["tdm"], cf_total["tdm-adaptive"]),
           ("conflict phase (own-mapped)", c_total["crossbar"], c_total["tdm"], c_total["tdm-adaptive"]),
           ("same-bank phase (own-mapped)", sb_total["crossbar"], sb_total["tdm"], sb_total["tdm-adaptive"]),
           ("structural-class phase (own-mapped)", p7_total["crossbar"], p7_total["tdm"],
            p7_total["tdm-adaptive"]),
           ("full-parallel phase (all 9 AGUs)", p8_total["crossbar"], p8_total["tdm"],
            p8_total["tdm-adaptive"])],
          "Grand totals over the full stimulus program (cycles; own-mapped rows — the cross-mapped classes of Table 5 are not summed here since each build runs a different concrete workload). The full-parallel row sums per-stream spans; the phase's wall clock is Table 8's footer.", 9,
          aligns=["l", "r", "r", "r"],
          foot=("<em>total</em>", grand["crossbar"], grand["tdm"], grand["tdm-adaptive"]))}
<p>Three design decisions carry the parity result. First, read windows pipeline because refetch
is per-cell and self-timed, so no cycle is spent on window transitions. Second, write windows
pipeline because the snapshot frees the primaries in one cycle and shadows free at their grant,
so the bus round trip never lengthens the window period. Third, the adaptive grant is
combinational, so the window turnaround's one-cycle request gap costs no bus re-acquisition
cycle. The residual TDM·RR gap is purely arbitration — idle bus turns — and vanishes exactly
when several buffers are active at once, which is why TDM·RR approaches the other columns in
the mixed phases of Appendix A.</p>

<figure>
{fig_results_chart()}
<figcaption><span class="cap-label">Figure 3:</span> Summary of the synthetic sweep
(cycles, lower is better). TDM·adaptive matches the crossbar wherever the crossbar is at its
best, and wins — by a widening margin — everywhere the workload turns adversarial, including
the crossbar's own structural conflict classes; TDM·RR carries the free-running arbiter's
idle-slot penalty whenever fewer than all buffers are active, and converges at saturation.</figcaption>
</figure>

<h2>5&emsp;Evaluation: production-representative stimuli</h2>
<p>Everything above is a synthetic, hand-constructed sweep: every stimulus is built to a known
structural class (conflict-free, single-bank, intra-port, ...) precisely so its result has a
closed form to check against. This section instead runs <code>tb_top.cpp</code> — the full
9-AGU system harness, not a per-phase unit test — against {eval_n} independently-generated,
externally-sourced stimuli sets (<code>tb/stimuli/final/0</code>&ndash;<code>19</code>), each
mixing RAGU_A/B/C and WAGU_A traffic of organic shape and volume over a shared, constant
RAGU_D/E and WAGU_D/E background load (confirmed byte-identical across all {eval_n} sets).
Nothing here is pinned to a closed form; the question is simply whether each backend finishes,
and how many cycles it takes when it does.</p>
<p>This sweep initially completed on only 11 of {eval_n} sets under TDM — a family of
task-boundary deadlocks the synthetic suite never provokes. All three root causes were found,
fixed in <code>agu.hpp</code>/the harness protocol, and regression-locked by a dedicated unit
suite (<code>tb_task_boundary.cpp</code>, Table 1) that reproduces each mechanism
deterministically — including a fence placed on the exact window-wrap edge, found by scanning
every fence offset. <strong>Both arbiters now complete all {eval_n} sets</strong>, and the
adaptive arbiter reaches <strong>exact crossbar cycle parity ({eval_exact_parity} of {eval_n}
sets to the beat), beating the crossbar outright on the remaining one</strong> (set 6, the
heaviest workload: 134824 vs 134857).</p>
<p>With the deadlocks resolved, the arbiter comparison is unambiguous:
<strong>the adaptive arbiter is the recommended TDM configuration</strong>. Under it, this
production-representative sweep matches the crossbar — identical cycle counts on
{eval_exact_parity} of {eval_n} sets and a win on the other — and §4 already showed the same
configuration matching or beating the crossbar on every synthetic class. For deployments
that keep the free-running round-robin, its rotation is now a <em>programmable slot
table</em> (<code>arbiter.hpp</code>'s <code>set_sequence()</code>, exposed as
<code>top_tdm.hpp::set_arb_sequence()</code> over the named client list
BUF_RAGU_A/B/C/D/E, BUF_WAGU_A/B/D/E): a deployment whose workload never uses a client
drops it from the table so the bus stops spending one turn per revolution on an idle
buffer. The final/N stimuli never drive WAGU_B, so the evaluation programs the 8-slot
sequence RAGU_A&rarr;B&rarr;C&rarr;D&rarr;E&rarr;WAGU_A&rarr;D&rarr;E, removing the dead
turn the 9-slot rotation spent per revolution — with it, even the free-running RR holds
exact parity on {eval_rr8_parity} of {eval_n} sets (Table E1).</p>
{booktabs(["set", "Crossbar", "TDM·adaptive", "TDM·RR (8-slot)", "speed-up"],
          eval_tbl_rows,
          f"Evaluation sweep — final/0&ndash;19 through the full system harness, all task-boundary fixes applied (actual cycles to completion; speed-up is crossbar / TDM·adaptive). Both arbiters complete all {eval_n} sets; the adaptive arbiter matches the crossbar exactly on {eval_exact_parity} and beats it on set 6. The RR column uses the programmed 8-slot table that skips the never-driven WAGU_B (Appendix A.7).", "E1",
          aligns=["r", "r", "r", "r", "r"])}

<h3>5.1&emsp;Utilization: why the completion times coincide</h3>
<p>Identical completion times mean neither fabric is the bottleneck, not that they behave
identically. The stimuli are <em>schedule-driven</em>: tasks carry absolute
<code>start_cycle</code> fences, and the last fence in every set sits ~2 cycles before the
finish (116275 on 19 sets, ~134.8k on set 6) — total runtime is fence-bound, and a backend only posts a longer
total when it falls behind schedule (exactly what RR does on the hot sets). Table E2 shows
how hard each fabric works to hold that schedule: both serve the identical beat count per
set, the crossbar's 64 banks idle at {util_cb_lo:g}&ndash;{util_cb_hi:g}% utilization,
and TDM carries the same beats over one shared bus at {busy_lo:g}&ndash;{busy_hi:g}%
occupancy with zero wasted turns under the adaptive arbiter (the free-running RR wastes
~40&ndash;50% of its turns on requestless buffers, yet still holds the schedule on 19 of
20 sets with the 8-slot table).</p>
{booktabs(["set", "xbar bank util", "TDM bank util", "TDM bus busy", "bus contention"],
          eval_util_rows,
          "Fenced utilization per set. Bank util = busy bank-cycles / (cycles &times; banks); the crossbar has 64 physical banks, TDM 32, so identical served beats give TDM exactly 2&times; the crossbar's figure. Bus busy = share of cycles the granted buffer had &ge;1 pending request; contention = share of cycles &ge;2 buffers competed (TDM·adaptive).", "E2",
          aligns=["r"] * 5)}

<figure>
{fig_bank_timeline()}
<figcaption><span class="cap-label">Figure 4:</span> Throughput demand vs fabric
pressure over time, one panel per evaluation set (160 buckets per run, one 0&ndash;100%
axis, 50% guide line). Filled areas show the <em>throughput requirement</em>: each
bucket's worst-cycle bank utilization, normalized to the fabric's own bank count
(crossbar grey&thinsp;/&thinsp;64, TDM blue&thinsp;/&thinsp;32) — the demand envelope the
interconnect must sustain instant by instant. Lines show <em>stall pressure</em>: the
share of the bucket's cycles with at least one real request blocked by contention
(task-start fill excluded). A fabric is a bottleneck only where its line approaches 100%
for a sustained stretch. The two fabrics carry the same beats in opposite shapes: the crossbar's demand is a
thin trickle (worst-cycle utilization median 6%, never above 56% of its 64 banks), while
TDM's window drains concentrate the same throughput into wide bursts that routinely fill
all 32 banks in a single cycle (median bucket peak 50%, max 100%) — time-division
batching made visible. The bottleneck verdict comes from the lines, not the areas: TDM's
pressure hugs zero outside short bursts even while its bursts saturate the bank array,
and the crossbar's broad contention plateaus on the hot sets occur at single-digit
utilization — pressure driven by conflicts, not capacity.</figcaption>
</figure>

<h3>5.2&emsp;Conflicts: cost, count, location</h3>
<p>A <em>conflict</em> is a real-address beat whose grant arrived late: both fabrics accept
a request with zero wait in the conflict-free case, and the response stage after the grant
is deterministic (gnt&rarr;rvalid is exactly 1 cycle for every response, both backends,
all sets), so every late grant is contention. Requests carrying addr&thinsp;0 (NOP
padding, flush, fence parking) are protocol idle time and excluded. Cost is reported as
<em>wait cycles</em> — wall-clock cycles a unit had &ge;1 real request pending, so 16
lanes parked in parallel count once, not sixteen times — and count as <em>episodes</em>,
group-cycles releasing &ge;1 waited beat. Wait that begins from group idle is <em>fill</em>
— the task-start pipeline latency of §4, paid once per idle&rarr;active transition and
hidden in practice by the timed lookahead prefetch (Appendix A.7) — so it is not a
conflict: fill cycles are excluded from the wait and episode columns on both backends by
the same rule and reported separately.</p>
<p>This accounting proved diagnostic before it produced Tables E3 and E4: the first sweep showed
70+% of RAGU_A/B beats delayed on <em>both</em> backends, with the crossbar losing three
quarters of its conflict cycles at L1 — its own sub-lanes colliding — and every TDM fetch
window containing same-bank pairs no mapping function could possibly separate. That
pattern (self-collision, same row, immune to both fabrics' scrambling) turned out to be a
<strong>stimulus format error</strong>: the A/B/C and WAGU trace files are row-indexed,
and the harness had been reading them as byte addresses, folding every 16 consecutive
"addresses" into one 16-byte row. With the traces corrected (&times;16), the static
same-row-collision predictor drops from 77.7% of beats to exactly 0. A second, smaller
descriptor correction followed (the R/C field order and the store-mode numbering); the
TDM sweeps were re-run — completion cycles are unchanged and the conflict metrics shift
only marginally, since only the map's k-split boundaries moved.</p>
<p>On the corrected traces, three results (Tables E3 per set, E4 per requester).
<strong>TDM waits far less than the crossbar</strong>:
{conf_ad_pct_lo:g}&ndash;{conf_ad_pct_hi:g}% of TDM beats see any delay (vs
{conf_cb_pct_lo:g}&ndash;{conf_cb_pct_hi:g}% on the crossbar), and summed wall-clock
contention wait is 2&ndash;15&times; lower per set and 5.5&times; lower in aggregate —
even though the crossbar baseline already includes the L1 hash repair this study
motivated. The crossbar's remaining conflicts are dominated by L1 self-collisions
(blocked in {lvl1_cyc_lo:g}&ndash;{lvl1_cyc_hi:g}% of cycles, vs
&le;{lvl2_cyc_hi:g}% at L2 and &le;{lvl3_cyc_hi:g}% at L3): its L1 stage routes each
lane by two unhashed address bits, so aligned lane strides fold a port onto one output —
a structural property shown in Appendix A.8 to be independent of lane-assignment policy
and irreducible by scheduling (&le;6% under the compute-frame constraint), which is what
justified folding those bits into the hash (<code>XBAR_HASH_L1</code>, &minus;87% L1
wait on set&thinsp;0, adopted as the baseline).
<strong>Task-start fill is gone</strong>: the hidden-lookahead prefetch (lead 16,
Appendix A.7) leaves ~100 residual fill-wait cycles per hot unit across the entire
20-set sweep.
<strong>Conflicts remain concentrated</strong> in the organically-loaded RAGU_A/B and
WAGU_A (Table E5 gives the full set&times;requester matrix); the sequentially-allocated
D/E lanes are structurally clean on both fabrics, and TDM's residual collisions trace to
a single task-geometry class — non-power-of-two dimensions degenerating the map's field
split — that collides only pairwise, costing the &lt;1 extra cycle per window of
Table E3. Appendix A.8 bounds every remedy evaluated for this class. No unit's duty
cycle exceeds ~10%, which is why none of this cost reaches the fence-bound finish
line.</p>
{booktabs(["set", "xbar L1", "xbar L2", "xbar L3", "xbar total", "TDM total", "TDM extra cyc/win"],
          eval_set_conf_rows,
          "Conflict rates per set, normalized to requests. Crossbar columns: blocked-request cycles resolved at each level per 100 requests (a request waiting 2 cycles at L1 counts twice — rate &times; average depth); xbar total = L1+L2+L3. TDM total uses the identical normalization (blocked window-slot cycles per 100 bus requests) — the two bold columns compare directly. TDM extra cyc/win is the latency view: extra serialization passes per 32-request window, averaged over ALL windows — a colliding window needs &ge;1 extra pass (16 pairwise collisions resolve in 1, 32 hits on one bank need 31), so a value of 0.4 means roughly 40% of windows pay one extra cycle and the rest pay none (collisions in these traces are almost always pairwise, see §5.2). Adaptive arbiter, fenced runs, real traffic only.", "E3",
          aligns=["r"] * 7)}

{xt_table([("AGU", 1), ("real beats", 1), ("conflict overhead", 2), ("episodes", 2),
           ("fill overhead", 2), ("active", 2)],
          eval_agu_rows,
          "Per-requester conflict summary, aggregated over all 20 sets (real traffic only). In every pair X = crossbar, T = TDM·adaptive — same convention as Table E5. conflict overhead = the unit's wall-clock contention-wait as a share of the sweep's total cycles (the same metric as E5's total row); episodes = number of stall events; fill overhead = the excluded task-start fill, same normalization; active = duty cycle (share of cycles driving &ge;1 real request). RAGU_C is driven by only 3 of 20 sets; WAGU_B by none.", "E4")}

{heatmap_matrix("Conflict cycle overhead per (set, requester): the unit's contention-wait cycles as a share of that run's total cycles — how much of the runtime the unit spent blocked by conflicts. The three organically-loaded units plus the five near-idle ones summed as <em>others</em> (RAGU_C/D/E, WAGU_D/E; WAGU_B is zero everywhere). Each unit shows a crossbar (X) and TDM·adaptive (T) column pair; T below X means less conflict overhead on TDM — which holds almost everywhere. Task-start fill excluded; &mdash; = zero. Absolute cycle counts are archived in eval_matrix.csv (shown on hover in the HTML version); the total row is the 20-set aggregate, and TDM gain below it is the improvement factor X&thinsp;/&thinsp;T on those totals — how many times less conflict overhead TDM accumulates on that unit.", "E5")}

<p>Table E6 attributes TDM's map-level collisions to their <em>storage modes</em>, from a
static replay of every fetch window through the exact map. AGU_D and AGU_E are kept as
their own rows — they carry no pattern geometry and currently run on fixed default map
parameters (Loop_Row_Col, C&thinsp;=&thinsp;R&thinsp;=&thinsp;4, L&thinsp;=&thinsp;8) — and
are natural candidates for dedicated storage modes of their own. Evaluating that option:
a per-unit split search and a per-unit free-matrix search bound the gain — AGU_E (no
same-row floor) improves 45&rarr;15 (reads) and 502&rarr;341 (writes) extra cycles per
set under a dedicated matrix, while AGU_D is dominated by a same-row floor (single-beat
tasks walking one 16-byte row: 1425 of its 1893 per-set extra cycles) that no bank
mapping can address — only same-row coalescing in the buffer would. The mode rows show
the same two offender classes as the shape analysis — Loop_Row/Row_Loop and the
thin-split Loop_Row_Col/Row_Col_Loop geometries — while the block modes never collide.</p>
{booktabs(["storage mode", "windows", "coll. windows", "extra cyc", "extra / window", "note"],
          eval_bymode_rows,
          "TDM map-level collision rate by storage mode (static window replay, full 20-set sweep). extra cyc = summed serialization passes; extra / window = per-fetch-window average. AGU_D/E rows model lane_agu's window packing in lockstep, which upper-bounds their collisions (their measured per-set bank-stall contribution is ~1000 cycles combined); they are listed separately from the pattern modes as candidates for dedicated storage modes. The AGU_D floor is same-row traffic no bank mapping can separate.", "E6",
          aligns=["l", "r", "r", "r", "r", "l"])}

<h3>5.3&emsp;Removing the schedule: throughput-bound</h3>
<p>To measure what the fabrics can actually <em>do</em>, the harness knob
<code>SEL_NO_FENCE</code> keeps only each stream's initial fence and runs every later task
back-to-back (same-geometry neighbors are merged so no padding is inserted between them).
Against the repaired crossbar the two fabrics are close to parity (Table E7):
<strong>the TDM bus leads by up to 1.27&times; on the heavy sets (6, 9, 10, 14), ties
set&thinsp;0, and trails by only 4&ndash;8% on the light ones</strong>. The bus runs
{unf_bus_lo:g}&ndash;{unf_bus_hi:g}% occupied moving {unf_bpt_lo:g}&ndash;{unf_bpt_hi:g}
real beats per turn; the crossbar sustains a similar 5.8&ndash;12.2 beats/cycle but pays
its residual L1 self-collisions, which is exactly where the heavy sets tip to TDM.
(Addr-0 filler plays no role: its fast path never issues bank requests, and the measured
NOP share of bus payload is zero.) The free-running RR remains the one clear loser at
saturation — 1.3&ndash;2&times; the adaptive cycles even with the 8-slot table — since
wasted turns become pure lost bandwidth. The two operating modes are consistent: under the production schedule the fabrics tie
with the TDM bus three-quarters idle, and even without a schedule the single bus gives up
at most single-digit percentages on the lightest sets.</p>
{booktabs(["set", "Crossbar", "TDM·adaptive", "TDM·RR (8-slot)", "TDM/xbar"],
          eval_unf_rows,
          "Unfenced sweep — identical stimuli with all but the initial fences removed, so total cycles measure fabric throughput. TDM/xbar = adaptive / crossbar cycles. The TDM·adaptive bus runs 95&ndash;100% occupied in this mode (vs 16&ndash;27% fenced).", "E7",
          aligns=["r"] * 5)}

<h2>6&emsp;Conclusions</h2>
<p>Under its intended, schedule-driven workload the windowed TDM interconnect is a full
substitute for the three-level crossbar, and on the corrected production stimuli it is
the stronger fabric on every measured axis. With the adaptive arbiter it finishes every one of the 20
production-representative sets in exactly the crossbar's cycle count (beating it on the
heaviest), holds the schedule at 16&ndash;27% utilization of one 32-wide bus where the
crossbar idles its 68&times;64 fabric at ~2% bank utilization, accumulates a small fraction (2&ndash;15&times; less per set) of the crossbar's
contention wait (the XOR map spreads distinct rows; the
crossbar keeps paying L1 self-collisions on strided patterns), and — with the hidden
lookahead — pays no visible task-start latency. Stripped of the schedule entirely, it
stays within 8% of the repaired crossbar on light sets and leads by up to 1.27&times; on
the heavy ones. The recommended configuration is <strong>TDM with the
adaptive arbiter and lookahead lead 16</strong>; deployments keeping the free-running RR
should program the slot table to their live client list and expect it to fall behind
only when the bus saturates. The evaluation also demonstrated the value of
first-principles metrics twice over: per-beat response-delay accounting is what exposed
both the task-boundary deadlock family and the row-vs-byte stimulus format error.</p>

<p><em>Limitations.</em> All results come from cycle-level SystemC models, not synthesized
RTL: the wiring-cost comparison is architectural (port counts and switch fan-in), and
area, frequency, and power remain future work. Timing is measured at the OBI ports;
pipeline-fill latency is characterized separately (Table 3) and hidden by the lookahead
prefetch rather than eliminated. The production evaluation covers one workload family —
twenty schedule-driven stimulus sets from a single generator — so the throughput-bound
comparison (§5.3) brackets rather than maps the design space; the synthetic sweep (§4)
provides the structural coverage the organic sets cannot. Fences are treated as timing
constraints only: read data is not checked against a cross-stream producer schedule, so
the lookahead lead's write&rarr;read ordering margin (Appendix A.7) is asserted, not
verified, in this harness. Both backends are driven by the same in-order AGU issuing
beats in trace order; §5.2 shows this is not a confound — with reordering bounded by the
compute frame, at most 6% of the crossbar's L1 overhead is schedulable away, and TDM's
results are issue-order-invariant. Finally, the free-running-arbiter results depend on the
programmed slot table matching the live client set; a mismatched table reintroduces the
idle-turn cost measured in §5.1.</p>

<h2>Appendix A&emsp;Implementation and design choices</h2>
<p>Everything in this appendix is reference material: how the modules are built, the design
decisions taken along the way, and what it takes to regenerate the numbers. Nothing here is
needed to follow the main text.</p>

<h3>A.1&emsp;Buffer organization and wiring</h3>
<figure>
{fig_buffer_block()}
<figcaption><span class="cap-label">Figure A1:</span> The buffer, wired. 32 identical cells —
one per TDM slot, each owning its manager bundle <code>m[w]</code> outright — sit between a
combinational port⇄group router and the sequential control registers. Solid arrows are
datapath, dashed are control; grey reverse arrows are return paths.</figcaption>
</figure>
<p>The buffer is deliberately thin glue around its cells. All per-slot state — fetched data,
pending fetches, primary latches, shadow engines — lives <em>inside</em> the 32
<code>buffer_cell</code> instances; the buffer itself owns only window bookkeeping: two group
pointers (<code>rd_ptr_q</code> for the draining/responding group, <code>fill_ptr_q</code> for
the filling group), the frozen window geometry (<code>window_mode_q</code> /
<code>pend_mode_q</code>, latched from <code>active_mode</code> so a mid-stream mode change
never tears a window), and a handful of flags (<code>primed_q</code>, <code>full_q</code>,
<code>resp_pending_q</code>). The router is pure combinational fan-in/fan-out: it connects the
λ&thinsp;=&thinsp;active_ports·4 port lanes to the current group of λ cells
(<code>cell_p_*_s[w]</code> going in, <code>cell_p_rdata_s[w]</code> coming out), and derives
the port handshakes (<code>p_gnt_o</code>&thinsp;=&thinsp;fill_ok on the write side,
<code>p_rvalid_o</code> from the drain/respond group on the read/write sides).</p>
<p>Control flows through exactly three kinds of wires. <em>Downstream</em>, one broadcast
pulse: <code>cell_reset_window_s</code>, whose single meaning is “a window boundary happened” —
the read wrap, the read boot-from-idle, and the write snapshot all drive the same wire, so a
cell has one boundary behavior to implement and the caller has one cursor contract to honor
(the pulse is also what tells the AGU to advance its lookahead window). <em>Upstream</em>, two
per-cell status collectors: <code>valid_o[w]</code> (read: presentable; write: shadow done) and
<code>invalid_o[w]</code> (read: idle; write: primary free), which the buffer AND-reduces into
the three decisions it ever makes — is this group drain-ready, are all shadows free, is the
whole array idle (boot). <em>Per-group</em>, <code>cell_all_valid_s[w]</code> tells a cell its
own group is draining this cycle, which is what arms the per-cell refetch and the
<code>is_fwd</code> forward path.</p>
<figure>
{fig_buffer_timeline()}
<figcaption><span class="cap-label">Figure A2:</span> Cycle-level schedules (λ=4, conflict-free,
illustrative). (a) Reads drain one group per cycle while each drained group's refetch is
already in flight — the two-cycle bus round trip sits well inside the 8-cycle reuse distance.
(b) Writes fill one group per cycle straight through the snapshot: the pulse hands the full
window to the shadows and frees the primaries on the same edge, the posted acks stream one
group per cycle behind it, and the whole burst drains to the banks while the next window
fills.</figcaption>
</figure>

<h3>A.2&emsp;Read buffer: per-cell prefetch, zero-bubble drain</h3>
<figure>
{fig_buffer_cells()}
<figcaption><span class="cap-label">Figure A3:</span> Inside one cell. (a) A read cell
refetches autonomously the moment its own group drains. (b) A write cell hands its latched beat
to a shadow engine on the snapshot pulse and is immediately ready for the next window.</figcaption>
</figure>
<p>Each read cell runs a private two-cycle fetch (arbiter grant, then bank response) and holds
one value. A group of λ cells drains to the ports when every cell in it is valid and all active
lanes request; a per-cell <code>is_fwd</code> mux lets a value that arrives on the drain edge be
forwarded directly, so a drained group's refetch overlaps the other groups' turns. Since a
group's slot is not needed again for (window/λ&thinsp;−&thinsp;1) cycles, the two-cycle round
trip hides completely for every real configuration and windows stream back to back with no
transition gap. The restart rule is a single expression —
<code>!pending&thinsp;∧&thinsp;en&thinsp;∧&thinsp;(all_valid&thinsp;∨&thinsp;!valid)</code> —
covering both steady-state refetch and cold restart from idle; the buffer detects the all-idle
case and emits the same <code>window_reset</code> pulse the wrap path uses, snapshotting the
window geometry (<code>window_mode_q</code>) so a mid-stream mode change never tears a window.</p>

<h3>A.3&emsp;Write buffer: fill, snapshot, posted respond</h3>
<p>The write path is the read path's twin, pipelined in three stages that all run
concurrently:</p>
<ul>
<li><em class="term">Fill.</em> Ports latch one group per cycle into the cells' primary latches,
advancing straight through window boundaries whenever primaries are free.</li>
<li><em class="term">Snapshot.</em> When the window is full (and any previous burst has cleared),
a one-cycle pulse copies every primary into its cell's shadow engine atomically and frees the
primaries — the next window's fill begins the very next cycle.</li>
<li><em class="term">Respond.</em> Port acks are <em>posted</em>: <code>p_rvalid</code> streams
one group per cycle right behind the snapshot, meaning “the burst is in flight,” not “the bank
committed.” A slow or conflicted burst back-pressures the <em>next</em> window's snapshot
instead of stalling the ports beat by beat.</li>
</ul>
<p>Shadow engines drain independently and free themselves <em>at the grant</em>: the bank fabric
samples the payload on the edge after the grant, so nothing downstream ever reads a freed
shadow, and the returning <code>rvalid</code> needs no tracking at all. A grant-live preview
(<code>valid_o&thinsp;=&thinsp;busy&thinsp;∧&thinsp;¬gnt</code>) lets the parent snapshot on the
exact edge the last grant lands. This is what closed the final timing gap (§4): the effective
bus round trip never exceeds the two-cycle fill time, so window period&thinsp;=
max(fill,&thinsp;round-trip)&thinsp;= fill at every port count.</p>
<p>One consequence is deliberate: posted acks relocate conflict cost onto the following write
window, and read-after-write ordering across different buffers requires a fence — which the
production phase structure already provides at thousands-of-cycles granularity.</p>

<h3>A.4&emsp;Arbiters</h3>
<p>The RR arbiter is a free-running counter over the nine buffers, advancing every cycle with no
data inputs; a registered <code>sel_rsp</code> trails one cycle to steer the lagging return
path. Its cost model is simple but harsh: an idle slot still consumes a bus turn, so a lone
active buffer gets one turn in nine. The adaptive arbiter priority-encodes, from a registered
round-robin fairness pointer, the next buffer with a pending request and skips the rest; the
grant itself is <em>combinational</em>, so a request is served the same cycle it is raised
(an earlier version registered the whole scan, and the one-cycle bus re-acquisition after
every idle gap cost exactly one dead cycle per 32-beat window turnaround on every buffered
read stream — §4's conflict-free detour). Identical interface to the RR arbiter, and the
source of most of the gap between the two TDM columns in §4.</p>

<h3>A.5&emsp;Mapping function</h3>
<p>The TDM map is stateless: it splits each word address into contiguous/stride/linear fields
and produces a 5-bit bank id through an XOR matrix (spec in
<code>doc/specs/map_func.md</code>), re-encoding the placement as an ordinary byte address for
the beat-interleaved fabric. The placement scheme is expected to evolve, so the test suite
deliberately pins only its <em>interface</em> contract — OBI pass-through, we/be broadcast, NOP
lanes — never the placement arithmetic itself.</p>


<h3>A.6&emsp;The task-boundary deadlocks and their fixes</h3>
<p>The production evaluation (§5) initially deadlocked on 9 of 20 sets. Three root causes
were isolated, fixed, and regression-locked by <code>tb_task_boundary.cpp</code>, which
reproduces each mechanism deterministically (including a fence placed on the exact
window-wrap edge, found by scanning every fence offset):</p>
<p><em>Cause 1: no <code>active_mode</code> encoding for a 3-port-group task.</em> The
buffer's group width has exactly three states — 1, 2, or 4 port-groups; a
<code>num_port_active&thinsp;=&thinsp;3</code> task gets rounded up to the 4-group mode, but
the AGU only requested on its 12 real lanes, so the 16-wide group's request gate could never
close. Fixed by <code>rounded_ports_used()</code>: requests, grants and captures all pad to
the rounded width with NOP lanes, and the lookahead window is laid out at the rounded stride
(NOP holes) so the slice matches the buffer's drain grouping.</p>
<p><em>Cause 2: residual prefetch stranded at task boundaries.</em> The buffer's cells
prefetch ahead of the AGU's capture pace, so a task's end can leave residual valid cells the
AGU never asks for; dropping req stranded them forever, blocking the all-idle restart the
next task needs. Fixed by the between-tasks flush: the AGU keeps requesting at the finished
task's width with NOP addresses (the same "always assert req until done" rule
<code>lane_agu.hpp</code> documents) until the residue drains — but only when the residue is
genuinely stale (see Cause 3's seamless path, which this must not destroy).</p>
<p><em>Cause 3: window geometry and window content latched non-atomically across task
boundaries.</em> The buffer re-latches its group width from <code>active_mode</code> at the
wrap edge itself, one cycle before an external driver can even observe that wrap
(<code>window_reset</code> is registered) — so a geometry-changing task transition mid-stream
latches one task's width for another task's content: a permanent width-mismatch deadlock,
reproduced deterministically by the fence-on-the-wrap-edge unit test. Fixed by making the
transition protocol geometry-aware: the lookahead cursor may only roll into a task with
DIFFERENT geometry (rounded width or C/R/L/store-mode) once the capture side has finished the
outgoing task and the buffer is observed idle — every geometry change then enters through the
buffer's atomic all-cells-idle boot latch, where mode and addresses are latched together on
one edge. Same-geometry boundaries keep streaming seamlessly (their prefetched next-task
content is real and must not be flushed), so the common case pays nothing; the synthetic
suite's only measurable shifts were +1 to +67 cycles on the handful of phase-5/6 tasks whose
port-width sweeps now pass through the idle transition, re-pinned as exact constants.</p>

<h3>A.7&emsp;Harness knobs and the hidden lookahead</h3>
<p>Three run-time knobs shape the evaluation. <em>Arbiter slot table</em>
(<code>arbiter.hpp::set_sequence()</code>, exposed as
<code>top_tdm.hpp::set_arb_sequence()</code>): the free-running RR's rotation is a
programmable client list; the evaluation programs the 8-slot sequence that skips the
never-driven WAGU_B — measured on the pre-correction stimuli it was worth exact parity on
16 of 20 sets and 7&ndash;12% on the hot ones versus the naive 9-slot rotation; on the
corrected traces RR-8 holds parity on 19 of 20 (Table E1). <em>Hidden lookahead</em> (<code>SEL_LA_LEAD</code>, agu.hpp):
the lookahead cursor may roll into a fenced task up to N cycles before its
<code>start_cycle</code>, so the buffer boot-latches and prefetches the first window inside
the fence gap and the port starts consuming at <code>start_cycle</code> sharp. An early roll
always takes the clean-boot path (capture done, residue flushed, buffer idle) — rolling
seamlessly ahead of the fence disarms the between-tasks flush and re-creates the Cause-2
strand, which is why the naive version deadlocked; with the gate, lead&thinsp;16 hid 83%
of RAGU_A's task-start fill on the pre-correction stimuli (8040&rarr;1385 wait cycles on
set 0; lead&thinsp;32: 93%) at unchanged completion times, and on the corrected traces it
leaves ~100 residual fill-wait cycles per hot unit across the whole sweep (Table E4). The lead must stay under the producer's fence margin
where fences order cross-stream write&rarr;read dependencies; the evaluation uses 16.
<em>Fences off</em> (<code>SEL_NO_FENCE</code>, agu.hpp/lane_agu.hpp): keeps each stream's
initial fence only and merges same-geometry back-to-back tasks (window padding is then
inserted once per merged run, not once per task) — the throughput-bound mode of §5.3.
<em>Lane distribution</em> (<code>SEL_PORT_INTERLEAVE</code>, agu.hpp): selects how a
group's trace entries map to port lanes — port-major (default, used by the evaluation of
§5 and assumed by the synthetic suites' pattern constructions) or cross-port dealing
(entry i to port i&thinsp;mod&thinsp;napa, lane i&thinsp;/&thinsp;napa). Completion
cycles are identical under both; only the crossbar's L1 conflict profile changes, for the
worse under cross-port dealing (A.8).</p>
<h3>A.8&emsp;Conflict remedy analysis</h3>
<p><em>Crossbar L1.</em> The L1 stage routes each lane by raw addr[5:4] — the two bits
directly above the row offset, untouched by the hash — so any access pattern whose
within-port lane stride is a multiple of 64&thinsp;B (four rows, e.g. the workload's
R&thinsp;=&thinsp;64 column walks) leaves those bits identical on all four lanes and
folds the port onto a single L1 output: a 4:1 self-collision no downstream stage can
undo. Three analyses established it as structural before it was repaired. The AGU's
lane-distribution policy only selects the victim class: the evaluation's port-major
assignment exposes strided walks; dealing entries across ports instead (Appendix A.7)
swaps the victims to the dominant row-sequential walks and measures ~45% <em>worse</em>
(set-0 L1 wait 103k&rarr;149k cycles), while TDM is bit-identical under both, since its
window content and bank map are lane-order-invariant. Nor is issue order the cause:
reordering freedom is bounded by the compute frame (one num_port_active-group feeds one
compute cycle), and an ideal within-frame scheduler recovers only 6% of the L1 overhead
— the dominant shapes' frames carry a single L1 key, so no permutation helps. The
repair is the same construction the L2 stage already uses: <code>XBAR_HASH_L1</code>
extends <code>addr_hash</code> to scramble addr[5:4] with addr[11:10] (bijective per
routing field). It removes 87% of set-0's L1 read wait (79.9k&rarr;10.2k cycles) and 28%
of the crossbar's aggregate contention wait, with some collisions redistributing to
L2/L3; it is adopted as the crossbar configuration for all of §5.</p>
<p><em>TDM map.</em> The residual TDM collisions trace to task geometries whose leading
dimension has no trailing zeros (e.g. C&thinsp;=&thinsp;1 under Loop_Row): k<sub>1</sub>
collapses onto the row boundary, the <code>con</code> field vanishes from the bank-id
XOR matrix, and windows fold exactly pairwise. Every remedy in reach was evaluated and
bounded. A <code>get_k</code> guard (<code>TDM_GETK_GUARD</code>: borrow two
<code>str</code> bits into <code>con</code> when the leading dimension is odd) is
implemented but left opt-in — <code>get_k</code> sees only the geometry, not the window
layout, and the same split serves layouts the guard helps (set&thinsp;6: bank-stall
cycles &minus;21%) and layouts it mildly degrades (set-0 writes). An exhaustive sweep of
every field split (k<sub>1</sub>, k<sub>2</sub>&thinsp;&isin;&thinsp;[e,&thinsp;12])
shows many colliding shapes — all with non-power-of-two dimensions — admit no
conflict-free split at all, and a weighted stochastic search over
invertibility-preserving 5&times;14 GF(2) bank matrices found none better than 11.8k
extra cycles against the current formula's 11.0k: the tz-parameterized splits are a
shape-adaptive matrix family a fixed matrix cannot beat. Per-shape <em>optimal</em>
splits found by offline search do cut the irregular shapes' cost roughly in half (49% on
held-out windows, several shapes to zero), but no closed-form arithmetic reproduces them
— an exact search over 1000+ formula candidates (trailing-zeros, floor/ceil-log2,
odd-part decompositions, dimension products, gcd combinations, additive offsets)
plateaus at 17% total reduction against the per-shape optimum's 62%, because the optimal
split depends on the walk's wrap phase against the matrix rows, information no scalar
function of (R,&thinsp;C,&thinsp;L) carries. Exploiting the full margin therefore
requires profile-guided mapping — the searched split carried in the task descriptor or
an exception table. Finally, layout-only zero-padding of irregular dimensions to powers
of two was evaluated and rejected: re-pitching leaves a sparse traversal whose wrap
points align, measuring 27% <em>worse</em> at 1.3&ndash;2.4&times; the address footprint
— the conflict-free power-of-two shapes owe their cleanliness to dense traversal, which
padding cannot retrofit without issuing the padding beats. The zero-hardware remedy
remains choosing power-of-two dimensions at generation time.</p>
<h3>A.9&emsp;Reproducing this report</h3>
<p>The report is generated by <code>doc/report/gen_report.py</code> from the timing logs and the
sweep summary checked in under <code>doc/report/data/</code>; see
<code>doc/report/README.md</code> for the three build commands that refresh the logs. In short:</p>
<pre><code>cd projects/tdm
# 1. rebuild the three timing binaries (-DSTIM_TIMING_REPORT) and rerun them
# 2. rerun the full unit sweep and update data/sweep.txt
python3 doc/report/gen_report.py          # → doc/report/report.html</code></pre>


<h2>Appendix B&emsp;Per-task spans</h2>
<p>λ&thinsp;=&thinsp;ports&thinsp;×&thinsp;4 lanes; the parity mark checks Crossbar and
TDM·adaptive against ⌈n/λ⌉. TDM·RR is listed for reference and is not expected to match.</p>
{appendix_tables}

<p class="fnote">Generated by <code>projects/tdm/doc/report/gen_report.py</code> from
<code>data/timing_*.log</code> and <code>data/sweep.txt</code>. SystemC models under
<code>projects/tdm/rtl/systemc/</code>; suites under <code>projects/tdm/tb/unit/</code>.</p>
</div>
"""

OUT.write_text(page)
print(f"wrote {OUT} ({len(page)} bytes); {n_cf_tasks} conflict-free tasks, "
      f"parity={'OK' if parity_ok else 'FAIL'}, {n_tests} tests in {len(suites)} suites")
