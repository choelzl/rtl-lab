#!/bin/bash
# Regression test for tb_top.cpp's own M1/M2 conflict-metric accounting
# (gs.conflicts/delayed_l1/l2/l3, lvl_rd/lvl_wr) — NOT a test of any hash
# formula or RTL routing policy. It exists because this session spent a lot
# of effort reasoning about what M1/M2 numbers "should" mean (fill vs.
# genuine collisions, the addr=0 NOP-request bug, etc.) with no automated
# check that the stats-gathering code itself stays correct across future
# edits to tb_top.cpp/agu.hpp.
#
# Hand-built, checked-in fixtures under tb/stimuli/metrics_selftest/*/0/, run
# against a plain (no hash macro) crossbar build, with EXACT expected values.
# Every address pair below was checked against the routing bits it actually
# needs to agree/disagree on (addr[5:4] decides the L1 target; the baseline
# addr_hash() L2 fold is applied unconditionally but is a pure function of the
# address, so identical inputs always land identically without needing to
# hand-derive the fold's numeric output) — an earlier draft of the
# partialconflict fixture used addresses that happened to share addr[5:4]=0
# and silently produced a full 4-way conflict instead of the intended 2-way
# one, which is exactly the kind of mistake these fixtures guard against.
#
#   conflictfree: a single real request per 4-lane group (the other 3 lanes
#     are addr=0 filler, now correctly dropped at the port — see agu.hpp's
#     drive_requests()), 8 groups, no two real requests ever simultaneous.
#     Expect M1=0%, M2=0% exactly: an uncontested request should see a
#     combinational grant, no wait at all.
#
#   fullconflict: one group, all 4 lanes request the IDENTICAL address (same
#     bank, same row) simultaneously. Only one can be granted per cycle by
#     construction, so exactly 3 of 4 beats must wait — M1=75% is a
#     pigeonhole guarantee independent of arbitration policy. If service is
#     maximally efficient (one new grant per cycle, no gaps), the total wait
#     is 0+1+2+3=6 cycles over 4 beats — M2=150% exactly.
#
#   l2conflict: napa=2 (two independent physical port-groups, each with its
#     own dedicated L1 arbiter), one real lane per group with the IDENTICAL
#     address, other 6 lanes filler. Neither lane's own L1 stage is
#     contested, so the conflict can only be observed once both L1 winners
#     reach L2 wanting the same bank — delayed_l1 must be 0 and delayed_l2
#     exactly 1, distinguishing this from fullconflict's L1-only contention.
#
#   wr_fullconflict: fullconflict's exact mirror on the write side (wagu_a),
#     checking that the separate won_l1/won_l2 write-side accounting block
#     in tb_top.cpp produces the identical 75%/150% result.
#
#   partialconflict: one group, lanes 0/1 share an identical address (must
#     conflict), lanes 2/3 use addresses with distinct addr[5:4] from lane
#     0/1 and each other (must NOT conflict with anything). Confirms M1
#     unions rather than over-counts: exactly 1 of 4 beats blocked, not 2
#     (both members of the pair) or 4 (the whole group).
#
#   fillstall: two groups back-to-back in one task. Group 1 (the very first
#     group, previous cycle genuinely idle) has a 2-way conflict; group 2
#     (starts immediately after group 1's last grant, no idle gap) has its
#     own independent 2-way conflict at different addresses. Confirms
#     fill_delayed/fill_delay_sum count only group 1's conflict (1, not 2) —
#     the run_fill flag correctly requires an idle previous cycle, so a
#     back-to-back group's own genuine contention is never misattributed to
#     "fill".
#
#   l3conflict: a real read (ragu_a) and a real write (wagu_a) to the
#     IDENTICAL address, same cycle. Reverse-engineered directly from
#     top_crossbar.hpp's bind_l3_and_banks()/bind_l2_read()/bind_l2_write():
#     l3_[b]'s two master ports are bound to l2_l3_rd[b] and l2_l3_wr[b] —
#     i.e. L3 is strictly a read-vs-write arbiter for one already-resolved
#     (L1,L2) target, never a many-reads arbiter (that's already resolved
#     upstream: L2 instance k only ever passes one winner per bank-group per
#     cycle). This came from reading the actual port bindings after two
#     guessed models of the L1/L2 relationship (same local lane-index; same
#     addr_hash L2-fold value) both failed empirically — recorded here so a
#     future change doesn't reintroduce a guess in place of the verified
#     wiring. Expect the read to win immediately (0 conflict) and the write
#     to lose, attributed specifically to delayed_l3/lvl_wr_l3 — confirming
#     read-over-write priority at this level as a locked-in, checked fact
#     rather than an assumption.
#
#   nop_regression: a direct regression guard for the addr=0 NOP-request bug
#     fixed this session in agu.hpp's drive_requests() (has_row() is an
#     index-bounds check, not a value check, so an in-bounds trace row whose
#     own address is literally 0x0 used to be driven as a live request
#     instead of being dropped at the port). One real request (0x1000,
#     chosen to match literal 0x0's routing on BOTH the L1-select bits
#     addr[5:4] and the unconditional addr_hash() L2 fold, so it would
#     genuinely contend if the filler asserted req) placed AFTER three 0x0
#     filler lines in lane order (arbitration favors lower lane index, so
#     the real request only loses if the fillers actually assert req — a
#     real request placed BEFORE the filler lines doesn't exercise the bug
#     at all, since it would win immediately regardless). Checked directly
#     against a rebuilt pre-fix copy of agu.hpp: this exact fixture shows
#     conflicts=1, delay_sum=3 on the old code and conflicts=0, delay_sum=0
#     on the fixed code — confirmed reproduction, not just a plausible guess.
#
# Usage (from projects/tdm/, environment sourced -- see sourceme.sh):
#   bash doc/report/tools/check_stats_invariants.sh
set -e
PROJ=$(cd "$(dirname "$0")/../../.." && pwd)
cd "$PROJ"

OUT=${1:-/tmp/tdm_metrics_selftest}
BIN="$OUT/bin"
mkdir -p "$BIN"

CFLAGS="-std=c++17 -O2 -Wno-cpp -Irtl/systemc -Itb/systemc -I$SYSTEMC_INCLUDE"
LFLAGS="-L$SYSTEMC_LIB -Wl,-rpath,$SYSTEMC_LIB -lsystemc -pthread"
g++ $CFLAGS -DIMPL_CROSSBAR tb/systemc/tb_top.cpp -o "$BIN/default" $LFLAGS

get_stat() { # get_stat <stats.log path> <key>
    awk -F, -v k="$2" '$1==k{print $2}' "$1"
}

pass=0
fail=0
check_eq() { # check_eq <label> <actual> <expected>
    if [ "$2" = "$3" ]; then
        echo "  PASS  $1 ($2)"
        pass=$((pass + 1))
    else
        echo "  FAIL  $1 (got $2, expected $3)"
        fail=$((fail + 1))
    fi
}

run_case() { # run_case <name>
    local name=$1
    local d="$OUT/run_$name"
    mkdir -p "$d"
    (cd "$d" && SEL_NO_MONITOR=1 SEL_IN_DIR="$PROJ/tb/stimuli/metrics_selftest/$name/0" "$BIN/default" >out.log 2>&1)
    echo "$d/stats.log"
}

echo "=== conflict-free: single isolated request per group, 8 groups ==="
s=$(run_case conflictfree)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  8
check_eq "conflicts (M1 numerator)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "delay_sum (M2 numerator)" "$(get_stat "$s" ragu_a_delay_sum)" 0
check_eq "delayed_l1"      "$(get_stat "$s" ragu_a_delayed_l1)"   0
check_eq "delayed_l2"      "$(get_stat "$s" ragu_a_delayed_l2)"   0
check_eq "nop_beats (filler must never be granted)" "$(get_stat "$s" ragu_a_nop_beats)" 0
check_eq "active_cycles (one per group, no waiting)" "$(get_stat "$s" ragu_a_active_cycles)" 8
check_eq "episodes (no delay -> no episodes)" "$(get_stat "$s" ragu_a_episodes)" 0
check_eq "rsp_min (fixed gnt->rvalid latency)" "$(get_stat "$s" rsp_min)" 1
check_eq "rsp_max (fixed gnt->rvalid latency)" "$(get_stat "$s" rsp_max)" 1
echo

echo "=== full conflict: 4 lanes, identical address, one group ==="
s=$(run_case fullconflict)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4)" "$(get_stat "$s" ragu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" ragu_a_delay_sum)" 6
check_eq "nop_beats" "$(get_stat "$s" ragu_a_nop_beats)" 0
check_eq "active_cycles (4-cycle drain of one group)" "$(get_stat "$s" ragu_a_active_cycles)" 4
check_eq "episodes (one per delayed beat)" "$(get_stat "$s" ragu_a_episodes)" 3
check_eq "fill_delayed (group's own first cycle -> all 3 are fill)" "$(get_stat "$s" ragu_a_fill_delayed)" 3
check_eq "stall_episodes (none - all delay is fill, not genuine stall)" "$(get_stat "$s" ragu_a_stall_episodes)" 0
check_eq "rsp_min (response latency independent of grant delay)" "$(get_stat "$s" rsp_min)" 1
check_eq "rsp_max (response latency independent of grant delay)" "$(get_stat "$s" rsp_max)" 1
echo

echo "=== L2 conflict: napa=2, matching address across two port-groups ==="
s=$(run_case l2conflict)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  2
check_eq "conflicts"       "$(get_stat "$s" ragu_a_conflicts)"   1
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   1
check_eq "delayed_l1 (must be 0 - contention is at L2)" "$(get_stat "$s" ragu_a_delayed_l1)" 0
check_eq "delayed_l2 (must be 1 - contention is at L2)" "$(get_stat "$s" ragu_a_delayed_l2)" 1
check_eq "nop_beats" "$(get_stat "$s" ragu_a_nop_beats)" 0
check_eq "active_cycles" "$(get_stat "$s" ragu_a_active_cycles)" 2
echo

echo "=== L3 conflict: simultaneous read+write to the identical address ==="
s=$(run_case l3conflict)
check_eq "read real_beats"  "$(get_stat "$s" ragu_a_real_beats)"  1
check_eq "read conflicts (wins immediately)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "read delayed_l3"  "$(get_stat "$s" ragu_a_delayed_l3)"  0
check_eq "write real_beats" "$(get_stat "$s" wagu_a_real_beats)"  1
check_eq "write conflicts (loses to the read)" "$(get_stat "$s" wagu_a_conflicts)" 1
check_eq "write delay_sum"  "$(get_stat "$s" wagu_a_delay_sum)"   1
check_eq "write delayed_l1 (must be 0 - contention is at L3)" "$(get_stat "$s" wagu_a_delayed_l1)" 0
check_eq "write delayed_l2 (must be 0 - contention is at L3)" "$(get_stat "$s" wagu_a_delayed_l2)" 0
check_eq "write delayed_l3 (must be 1 - contention is at L3)" "$(get_stat "$s" wagu_a_delayed_l3)" 1
check_eq "lvl_rd_l3 (read side pays nothing)" "$(get_stat "$s" lvl_rd_l3)" 0
check_eq "lvl_wr_l3 (write side pays the wait)" "$(get_stat "$s" lvl_wr_l3)" 1
echo

echo "=== write-side full conflict: 4 lanes, identical address, one group ==="
s=$(run_case wr_fullconflict)
check_eq "real_beats"      "$(get_stat "$s" wagu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4)" "$(get_stat "$s" wagu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" wagu_a_delay_sum)" 6
echo

echo "=== partial conflict: 2 of 4 lanes conflict, 2 are genuinely free ==="
s=$(run_case partialconflict)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  4
check_eq "conflicts (exactly 1 of 4, not 2 or 4)" "$(get_stat "$s" ragu_a_conflicts)" 1
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   1
echo

echo "=== fill vs. genuine stall: group1 (from idle) + group2 (back-to-back) ==="
s=$(run_case fillstall)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  8
check_eq "conflicts (one per group)" "$(get_stat "$s" ragu_a_conflicts)" 2
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   2
check_eq "fill_delayed (only group1's, not group2's)" "$(get_stat "$s" ragu_a_fill_delayed)" 1
check_eq "fill_delay_sum" "$(get_stat "$s" ragu_a_fill_delay_sum)" 1
check_eq "episodes (one per group's own conflict)" "$(get_stat "$s" ragu_a_episodes)" 2
check_eq "stall_episodes (only group2's — an independent counter from fill_delayed, should still agree)" "$(get_stat "$s" ragu_a_stall_episodes)" 1
echo

echo "=== NOP regression: real request behind addr=0 filler in lane order ==="
s=$(run_case nop_regression)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  1
check_eq "conflicts (0 - filler must not assert req)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   0
echo

echo "=== Summary: $pass passed, $fail failed ==="
if [ "$fail" -gt 0 ]; then
    exit 1
fi
echo "All stats-invariant checks passed."
