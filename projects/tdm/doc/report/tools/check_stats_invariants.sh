#!/bin/bash
# Regression test for tb_top.cpp's own M1/M2/M3 conflict-metric accounting
# (gs.conflicts/delayed_l1/l2/l3, lvl_rd/lvl_wr, arrival_cycles) — NOT a test
# of any hash formula or RTL routing policy. It exists because this session
# spent a lot of effort reasoning about what these numbers "should" mean
# (fill vs. genuine collisions, the addr=0 NOP-request bug, etc.) with no
# automated check that the stats-gathering code itself stays correct across
# future edits to tb_top.cpp/agu.hpp.
#
# M3 (cycle-inflation vs. a perfectly conflict-free execution of this SAME
# traffic, computed downstream as 100*(active_cycles/arrival_cycles - 1), not
# in tb_top.cpp itself) is wall-clock, not per-lane: arrival_cycles counts
# distinct cycles with >=1 genuinely NEW request (see grp_stat_t's own
# comment for why an earlier lambda/napa-based attempt was wrong — it
# assumed every configured port-group lane is simultaneously real, which
# gives a false 300% reading on deliberately-sparse fixtures like
# conflictfree). The fixtures below double as M3's own regression check:
# every *_arrival_cycles value asserted was computed by hand from the
# fixture's own construction (which cycle each lane's request first becomes
# real), not reverse-engineered from a run's output.
#
# M3 is meant to answer "what's the TOTAL time impact", so it must be
# aggregated as sum(active_cycles)/sum(arrival_cycles) across every real
# beat in scope (a whole sweep, not one episode) — never as an average of
# per-episode M3 percentages, which would weight a 2-beat episode the same
# as a 200-beat one. multi_episode exists specifically to regress that: 3
# temporally-separated conflicts (2-way, 3-way, and a clean zero-conflict
# beat, each its own task with a genuine idle gap before the next) on one
# AGU, checking that active_cycles/arrival_cycles/delay_sum/conflicts all
# SUM correctly across episodes rather than only being correct in isolation
# — every fixture above this point tests exactly one conflict event; this
# is the only one that tests accumulation over time, and the only one that
# exercises a 3-way (not just 2- or 4-way) pigeonhole.
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
#   l2conflict_2agu_napa1: same L2 target as l2conflict, but the two
#     contending requesters are on TWO SEPARATE AGUs (ragu_a, ragu_b), each
#     napa=1 (their own single port-group), rather than one AGU's own two
#     port-groups. l2conflict alone only exercises 2 of l2_rd_[k]'s NUM_RPORT
#     (9) inputs, both from the same physical AGU (same l1_rd_[j] instance
#     twice over j); this fixture instead crosses two entirely separate
#     l1_rd_[j] instances (RAGU_A occupies j=0-3, RAGU_B j=4-5) to confirm L2
#     arbitration behaves the same across AGU boundaries as within one AGU's
#     own ports. Checked empirically (not assumed): ragu_a (lower port index)
#     wins immediately, ragu_b loses — the same lower-index-wins convention
#     already established at L1 (nop_regression) holds at L2 too.
#
#   l2conflict_1agu_napa4: l2conflict's own-AGU case pushed to RAGU_A's full
#     napa=4 (all 4 of its port-groups, one real lane each, identical
#     address) instead of napa=2 — a 4-way pigeonhole at L2 exactly mirroring
#     fullconflict's 4-way L1 case (conflicts=3, delay_sum=6) but confirming
#     the same union/exact-sum invariants hold when the contention point is
#     L2 rather than L1.
#
#   l2conflict_2agu_napa2: the 4-way pigeonhole split two-and-two across two
#     AGUs instead of one (ragu_a napa=2 + ragu_b napa=2, identical L2
#     target) — checks that a multi-AGU, multi-port mix still sums to the
#     same pigeonhole floor (conflicts=3, delay_sum=6 combined) and that
#     per-AGU attribution follows port-index order throughout (ragu_a's two
#     ports serviced first: delay_sum=1; ragu_b's two serviced after:
#     delay_sum=5), not just the two extremes (all-one-AGU / one-port-each).
#
#   l2conflict_wr_2agu_napa1 / l2conflict_wr_1agu_napa4 / l2conflict_wr_2agu_napa2:
#     the exact write-side mirror of the three l2conflict_* fixtures above
#     (wagu_a/wagu_b in place of ragu_a/ragu_b) — L1 has both a read
#     (fullconflict) and write (wr_fullconflict) mirror already; L2 had
#     neither before these three, so the write-side won_l1/won_l2
#     accounting block was previously untested for L2-level contention at
#     all, single-AGU or cross-AGU.
#
#   l3conflict_2bank: two INDEPENDENT read+write L3 collisions in the same
#     cycle, at two different banks (ragu_a/wagu_a both napa=2, group 0 at
#     0x100, group 1 at 0x140 — different addr[8:6] targets, so genuinely
#     different bank groups, not the same collision counted twice). Confirms
#     l3conflict's single-pair result generalizes: per-bank L3 arbiters are
#     independent and their conflicts sum correctly (delayed_l3=2,
#     delay_l3_sum=2, matching lvl_wr_l3=2) rather than interfering or being
#     merged into one event. l3conflict alone can't distinguish "L3 handles
#     multiple simultaneous collisions correctly" from "L3 only ever sees one
#     collision at a time" since it only ever drives one bank.
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
check_eq "arrival_cycles (M3 basis: 8 sequential, never-simultaneous arrivals -> M3=0%)" "$(get_stat "$s" ragu_a_arrival_cycles)" 8
check_eq "episodes (no delay -> no episodes)" "$(get_stat "$s" ragu_a_episodes)" 0
check_eq "rsp_min (fixed gnt->rvalid latency)" "$(get_stat "$s" rsp_min)" 1
check_eq "rsp_max (fixed gnt->rvalid latency)" "$(get_stat "$s" rsp_max)" 1
echo

echo "=== full conflict: 4 lanes, identical address, one group ==="
s=$(run_case fullconflict)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4)" "$(get_stat "$s" ragu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" ragu_a_delay_sum)" 6
check_eq "delay_l1_sum (per-AGU M2, all contention is at L1)" "$(get_stat "$s" ragu_a_delay_l1_sum)" 6
check_eq "delay_l2_sum (none - contention is at L1)" "$(get_stat "$s" ragu_a_delay_l2_sum)" 0
check_eq "delay_l3_sum (none - contention is at L1)" "$(get_stat "$s" ragu_a_delay_l3_sum)" 0
check_eq "nop_beats" "$(get_stat "$s" ragu_a_nop_beats)" 0
check_eq "active_cycles (4-cycle drain of one group)" "$(get_stat "$s" ragu_a_active_cycles)" 4
check_eq "arrival_cycles (M3 basis: all 4 arrive in 1 cycle, serialized over 4 -> M3=300%)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
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
check_eq "delay_l1_sum (per-AGU M2, none - contention is at L2)" "$(get_stat "$s" ragu_a_delay_l1_sum)" 0
check_eq "delay_l2_sum (per-AGU M2, must equal delay_sum here)" "$(get_stat "$s" ragu_a_delay_l2_sum)" 1
check_eq "delay_l2_sum matches crossbar-wide lvl_rd_l2 (sole traffic source)" "$(get_stat "$s" ragu_a_delay_l2_sum)" "$(get_stat "$s" lvl_rd_l2)"
check_eq "nop_beats" "$(get_stat "$s" ragu_a_nop_beats)" 0
check_eq "active_cycles" "$(get_stat "$s" ragu_a_active_cycles)" 2
check_eq "arrival_cycles (M3 basis: both arrive together -> M3=100%)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
echo

echo "=== L2 conflict, cross-AGU: ragu_a napa=1 + ragu_b napa=1, same L2 target ==="
s=$(run_case l2conflict_2agu_napa1)
check_eq "ragu_a real_beats"  "$(get_stat "$s" ragu_a_real_beats)"  1
check_eq "ragu_a conflicts (wins immediately - lower port index)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "ragu_a delay_sum"   "$(get_stat "$s" ragu_a_delay_sum)"   0
check_eq "ragu_b real_beats"  "$(get_stat "$s" ragu_b_real_beats)"  1
check_eq "ragu_b conflicts (loses to ragu_a)" "$(get_stat "$s" ragu_b_conflicts)" 1
check_eq "ragu_b delay_sum"   "$(get_stat "$s" ragu_b_delay_sum)"   1
check_eq "ragu_a delayed_l1 (must be 0 - own port-group uncontested)" "$(get_stat "$s" ragu_a_delayed_l1)" 0
check_eq "ragu_b delayed_l1 (must be 0 - own port-group uncontested)" "$(get_stat "$s" ragu_b_delayed_l1)" 0
check_eq "ragu_b delayed_l2 (contention is cross-AGU, at L2)" "$(get_stat "$s" ragu_b_delayed_l2)" 1
check_eq "ragu_b delay_l2_sum" "$(get_stat "$s" ragu_b_delay_l2_sum)" 1
check_eq "combined delay_l2_sum matches crossbar-wide lvl_rd_l2" "$(( $(get_stat "$s" ragu_a_delay_l2_sum) + $(get_stat "$s" ragu_b_delay_l2_sum) ))" "$(get_stat "$s" lvl_rd_l2)"
check_eq "ragu_a arrival_cycles (M3=0%, wins immediately)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
check_eq "ragu_b arrival_cycles (M3=100%, both arrive same cycle, ragu_b waits 1)" "$(get_stat "$s" ragu_b_arrival_cycles)" 1
echo

echo "=== L2 conflict, single AGU, napa=4: RAGU_A's full port count, one address ==="
s=$(run_case l2conflict_1agu_napa4)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4, pigeonhole floor)" "$(get_stat "$s" ragu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" ragu_a_delay_sum)" 6
check_eq "delayed_l1 (must be 0 - contention is at L2, not within any one port-group)" "$(get_stat "$s" ragu_a_delayed_l1)" 0
check_eq "delayed_l2" "$(get_stat "$s" ragu_a_delayed_l2)" 3
check_eq "delay_l2_sum" "$(get_stat "$s" ragu_a_delay_l2_sum)" 6
check_eq "delay_l2_sum matches crossbar-wide lvl_rd_l2 (sole traffic source)" "$(get_stat "$s" ragu_a_delay_l2_sum)" "$(get_stat "$s" lvl_rd_l2)"
check_eq "arrival_cycles (M3 basis: all 4 arrive in 1 cycle, serialized over 4 -> M3=300%, matching fullconflict's L1 case)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
echo

echo "=== L2 conflict, two AGUs, napa=2 each: 4-way pigeonhole split across AGUs ==="
s=$(run_case l2conflict_2agu_napa2)
check_eq "ragu_a real_beats" "$(get_stat "$s" ragu_a_real_beats)" 2
check_eq "ragu_b real_beats" "$(get_stat "$s" ragu_b_real_beats)" 2
check_eq "ragu_a conflicts (its 2 ports serviced first)" "$(get_stat "$s" ragu_a_conflicts)" 1
check_eq "ragu_b conflicts (its 2 ports serviced after ragu_a's)" "$(get_stat "$s" ragu_b_conflicts)" 2
check_eq "ragu_a delay_sum (waits 0+1)" "$(get_stat "$s" ragu_a_delay_sum)" 1
check_eq "ragu_b delay_sum (waits 2+3)" "$(get_stat "$s" ragu_b_delay_sum)" 5
check_eq "combined delay_sum matches the same 4-way pigeonhole floor as l2conflict_1agu_napa4" "$(( $(get_stat "$s" ragu_a_delay_sum) + $(get_stat "$s" ragu_b_delay_sum) ))" 6
check_eq "ragu_a delayed_l1 (must be 0 - contention is cross-port, at L2)" "$(get_stat "$s" ragu_a_delayed_l1)" 0
check_eq "ragu_b delayed_l1 (must be 0 - contention is cross-port, at L2)" "$(get_stat "$s" ragu_b_delayed_l1)" 0
check_eq "combined delay_l2_sum matches crossbar-wide lvl_rd_l2" "$(( $(get_stat "$s" ragu_a_delay_l2_sum) + $(get_stat "$s" ragu_b_delay_l2_sum) ))" "$(get_stat "$s" lvl_rd_l2)"
check_eq "ragu_a arrival_cycles (M3=100%, its 2 ports both arrive together)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
check_eq "ragu_b arrival_cycles (M3=300%, all 4 across both AGUs arrive in the same cycle)" "$(get_stat "$s" ragu_b_arrival_cycles)" 1
echo

echo "=== L2 conflict, write side, cross-AGU: wagu_a napa=1 + wagu_b napa=1 ==="
s=$(run_case l2conflict_wr_2agu_napa1)
check_eq "wagu_a real_beats"  "$(get_stat "$s" wagu_a_real_beats)"  1
check_eq "wagu_a conflicts (wins immediately - lower port index)" "$(get_stat "$s" wagu_a_conflicts)" 0
check_eq "wagu_a delay_sum"   "$(get_stat "$s" wagu_a_delay_sum)"   0
check_eq "wagu_b real_beats"  "$(get_stat "$s" wagu_b_real_beats)"  1
check_eq "wagu_b conflicts (loses to wagu_a)" "$(get_stat "$s" wagu_b_conflicts)" 1
check_eq "wagu_b delay_sum"   "$(get_stat "$s" wagu_b_delay_sum)"   1
check_eq "wagu_a delayed_l1 (must be 0 - own port-group uncontested)" "$(get_stat "$s" wagu_a_delayed_l1)" 0
check_eq "wagu_b delayed_l1 (must be 0 - own port-group uncontested)" "$(get_stat "$s" wagu_b_delayed_l1)" 0
check_eq "wagu_b delayed_l2 (contention is cross-AGU, at L2)" "$(get_stat "$s" wagu_b_delayed_l2)" 1
check_eq "wagu_b delay_l2_sum" "$(get_stat "$s" wagu_b_delay_l2_sum)" 1
check_eq "combined delay_l2_sum matches crossbar-wide lvl_wr_l2" "$(( $(get_stat "$s" wagu_a_delay_l2_sum) + $(get_stat "$s" wagu_b_delay_l2_sum) ))" "$(get_stat "$s" lvl_wr_l2)"
check_eq "wagu_a arrival_cycles (M3=0%, wins immediately)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
check_eq "wagu_b arrival_cycles (M3=100%, both arrive same cycle, wagu_b waits 1)" "$(get_stat "$s" wagu_b_arrival_cycles)" 1
echo

echo "=== L2 conflict, write side, single AGU napa=4: WAGU_A's full port count ==="
s=$(run_case l2conflict_wr_1agu_napa4)
check_eq "real_beats"      "$(get_stat "$s" wagu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4, pigeonhole floor)" "$(get_stat "$s" wagu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" wagu_a_delay_sum)" 6
check_eq "delayed_l1 (must be 0 - contention is at L2)" "$(get_stat "$s" wagu_a_delayed_l1)" 0
check_eq "delayed_l2" "$(get_stat "$s" wagu_a_delayed_l2)" 3
check_eq "delay_l2_sum" "$(get_stat "$s" wagu_a_delay_l2_sum)" 6
check_eq "delay_l2_sum matches crossbar-wide lvl_wr_l2 (sole traffic source)" "$(get_stat "$s" wagu_a_delay_l2_sum)" "$(get_stat "$s" lvl_wr_l2)"
check_eq "arrival_cycles (M3 basis: all 4 arrive in 1 cycle -> M3=300%)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
echo

echo "=== L2 conflict, write side, two AGUs napa=2 each: pigeonhole split across AGUs ==="
s=$(run_case l2conflict_wr_2agu_napa2)
check_eq "wagu_a real_beats" "$(get_stat "$s" wagu_a_real_beats)" 2
check_eq "wagu_b real_beats" "$(get_stat "$s" wagu_b_real_beats)" 2
check_eq "wagu_a conflicts (its 2 ports serviced first)" "$(get_stat "$s" wagu_a_conflicts)" 1
check_eq "wagu_b conflicts (its 2 ports serviced after wagu_a's)" "$(get_stat "$s" wagu_b_conflicts)" 2
check_eq "wagu_a delay_sum (waits 0+1)" "$(get_stat "$s" wagu_a_delay_sum)" 1
check_eq "wagu_b delay_sum (waits 2+3)" "$(get_stat "$s" wagu_b_delay_sum)" 5
check_eq "combined delay_sum matches the same 4-way pigeonhole floor" "$(( $(get_stat "$s" wagu_a_delay_sum) + $(get_stat "$s" wagu_b_delay_sum) ))" 6
check_eq "wagu_a delayed_l1 (must be 0 - contention is cross-port, at L2)" "$(get_stat "$s" wagu_a_delayed_l1)" 0
check_eq "wagu_b delayed_l1 (must be 0 - contention is cross-port, at L2)" "$(get_stat "$s" wagu_b_delayed_l1)" 0
check_eq "combined delay_l2_sum matches crossbar-wide lvl_wr_l2" "$(( $(get_stat "$s" wagu_a_delay_l2_sum) + $(get_stat "$s" wagu_b_delay_l2_sum) ))" "$(get_stat "$s" lvl_wr_l2)"
check_eq "wagu_a arrival_cycles (M3=100%, its 2 ports both arrive together)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
check_eq "wagu_b arrival_cycles (M3=300%, all 4 across both AGUs arrive in the same cycle)" "$(get_stat "$s" wagu_b_arrival_cycles)" 1
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
check_eq "write delay_l3_sum (per-AGU M2, must equal delay_sum here)" "$(get_stat "$s" wagu_a_delay_l3_sum)" 1
check_eq "write delay_l3_sum matches crossbar-wide lvl_wr_l3 (sole write traffic)" "$(get_stat "$s" wagu_a_delay_l3_sum)" "$(get_stat "$s" lvl_wr_l3)"
check_eq "lvl_rd_l3 (read side pays nothing)" "$(get_stat "$s" lvl_rd_l3)" 0
check_eq "lvl_wr_l3 (write side pays the wait)" "$(get_stat "$s" lvl_wr_l3)" 1
check_eq "read arrival_cycles (M3=0%)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
check_eq "write arrival_cycles (M3=100%: 1 conflict resolved over 2 wall-clock cycles)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
echo

echo "=== L3 conflict, two independent banks: napa=2 read+write pairs at 0x100/0x140 ==="
s=$(run_case l3conflict_2bank)
check_eq "read real_beats"  "$(get_stat "$s" ragu_a_real_beats)"  2
check_eq "read conflicts (both banks' reads win immediately)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "read delayed_l3" "$(get_stat "$s" ragu_a_delayed_l3)" 0
check_eq "write real_beats" "$(get_stat "$s" wagu_a_real_beats)" 2
check_eq "write conflicts (both banks' writes lose to their own read)" "$(get_stat "$s" wagu_a_conflicts)" 2
check_eq "write delay_sum (1 cycle per bank, independently)" "$(get_stat "$s" wagu_a_delay_sum)" 2
check_eq "write delayed_l3 (both collisions attributed to L3)" "$(get_stat "$s" wagu_a_delayed_l3)" 2
check_eq "write delay_l3_sum" "$(get_stat "$s" wagu_a_delay_l3_sum)" 2
check_eq "write delay_l3_sum matches crossbar-wide lvl_wr_l3" "$(get_stat "$s" wagu_a_delay_l3_sum)" "$(get_stat "$s" lvl_wr_l3)"
check_eq "lvl_rd_l3 (read side pays nothing on either bank)" "$(get_stat "$s" lvl_rd_l3)" 0
check_eq "read arrival_cycles (M3=0%, both banks' reads win immediately)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
check_eq "write arrival_cycles (M3=100% — SAME as the single-bank l3conflict above: 2 simultaneous, INDEPENDENT 1-cycle conflicts don't inflate M3 by conflict count the way delay_sum/M2 does)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
echo

echo "=== write-side full conflict: 4 lanes, identical address, one group ==="
s=$(run_case wr_fullconflict)
check_eq "real_beats"      "$(get_stat "$s" wagu_a_real_beats)"  4
check_eq "conflicts (M1 = 75% of 4)" "$(get_stat "$s" wagu_a_conflicts)" 3
check_eq "delay_sum (M2 = 0+1+2+3 over 4)" "$(get_stat "$s" wagu_a_delay_sum)" 6
check_eq "delay_l1_sum (per-AGU M2, all contention is at L1)" "$(get_stat "$s" wagu_a_delay_l1_sum)" 6
check_eq "arrival_cycles (M3 basis: all 4 arrive in 1 cycle -> M3=300%, mirroring fullconflict)" "$(get_stat "$s" wagu_a_arrival_cycles)" 1
echo

echo "=== partial conflict: 2 of 4 lanes conflict, 2 are genuinely free ==="
s=$(run_case partialconflict)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  4
check_eq "conflicts (exactly 1 of 4, not 2 or 4)" "$(get_stat "$s" ragu_a_conflicts)" 1
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   1
check_eq "arrival_cycles (M3=100%: all 4 arrive together, 2 free lanes served same cycle, 1 of the colliding pair waits)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
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
check_eq "arrival_cycles (M3=100%: 2 distinct arrival cycles, one per group, 4 total active cycles)" "$(get_stat "$s" ragu_a_arrival_cycles)" 2
echo

echo "=== NOP regression: real request behind addr=0 filler in lane order ==="
s=$(run_case nop_regression)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  1
check_eq "conflicts (0 - filler must not assert req)" "$(get_stat "$s" ragu_a_conflicts)" 0
check_eq "delay_sum"       "$(get_stat "$s" ragu_a_delay_sum)"   0
check_eq "arrival_cycles (M3=0%: no conflict, arrives and is granted the same cycle)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
echo

echo "=== L1-then-L2: one beat blocked at L1, THEN (after winning L1) blocked at L2 too ==="
# napa=2: group0 lanes0,1 = 0x100 (L1 conflict within group0); group1 lane0 =
# 0x100 too (no L1 contention in group1 alone, but same L2 target as
# group0's address). Reconstructed from the actual run (L2 arbitration is
# fairness/rotation-based, not fixed-priority, discovered here): cycle0,
# group0 lane0 wins its own L1, group1 lane0 has none to contest — both
# reach L2 same cycle, group0-lane0 wins, group1-lane0 waits. cycle1,
# group0-lane1 (freed from L1) reaches L2 fresh; group1-lane0 (2nd L2
# attempt) now wins the rotation — group0-lane1 loses AT L2 too, having
# already waited at L1 in cycle0. cycle2, group0-lane1 alone, granted.
# group0-lane1's single 2-cycle wait streak touches BOTH levels (1 at L1,
# 1 at L2) — this is the case M1's "delayed_l1+l2 can exceed conflicts"
# comment describes and no other fixture exercises.
s=$(run_case l1_then_l2)
check_eq "real_beats"      "$(get_stat "$s" ragu_a_real_beats)"  3
check_eq "conflicts (2 of 3 beats delayed)" "$(get_stat "$s" ragu_a_conflicts)" 2
check_eq "delay_sum" "$(get_stat "$s" ragu_a_delay_sum)" 3
check_eq "delayed_l1 (only group0-lane1 ever touches L1)" "$(get_stat "$s" ragu_a_delayed_l1)" 1
check_eq "delayed_l2 (group0-lane1 AND group1-lane0 both touch L2 -- delayed_l1+delayed_l2=3 > conflicts=2, BY DESIGN: union, not a partition)" "$(get_stat "$s" ragu_a_delayed_l2)" 2
check_eq "delay_l1_sum (exactly group0-lane1's 1 L1-cycle)" "$(get_stat "$s" ragu_a_delay_l1_sum)" 1
check_eq "delay_l2_sum (group0-lane1's 1 L2-cycle + group1-lane0's 1 L2-cycle)" "$(get_stat "$s" ragu_a_delay_l2_sum)" 2
check_eq "delay_l1_sum + delay_l2_sum equals delay_sum exactly (M2 IS a true partition, unlike M1)" "$(( $(get_stat "$s" ragu_a_delay_l1_sum) + $(get_stat "$s" ragu_a_delay_l2_sum) ))" "$(get_stat "$s" ragu_a_delay_sum)"
check_eq "active_cycles (3 distinct wall-clock cycles to fully drain)" "$(get_stat "$s" ragu_a_active_cycles)" 3
check_eq "arrival_cycles (all 3 requests arrive together -> M3=200%, level-agnostic)" "$(get_stat "$s" ragu_a_arrival_cycles)" 1
check_eq "delay_l1_sum matches crossbar-wide lvl_rd_l1 (sole traffic source)" "$(get_stat "$s" ragu_a_delay_l1_sum)" "$(get_stat "$s" lvl_rd_l1)"
check_eq "delay_l2_sum matches crossbar-wide lvl_rd_l2 (sole traffic source)" "$(get_stat "$s" ragu_a_delay_l2_sum)" "$(get_stat "$s" lvl_rd_l2)"
echo

echo "=== Multi-episode: 3 temporally-separated conflicts (2-way, 3-way, clean) on one AGU ==="
s=$(run_case multi_episode)
check_eq "real_beats (4+4+1 across the 3 episodes)" "$(get_stat "$s" ragu_a_real_beats)" 9
check_eq "conflicts (1 from the 2-way + 2 from the 3-way + 0 clean)" "$(get_stat "$s" ragu_a_conflicts)" 3
check_eq "delay_sum (1 + [1+2] + 0)" "$(get_stat "$s" ragu_a_delay_sum)" 4
check_eq "active_cycles (2 + 3 + 1, summed across episodes, not averaged)" "$(get_stat "$s" ragu_a_active_cycles)" 6
check_eq "arrival_cycles (1 per episode - each starts from a genuine idle gap)" "$(get_stat "$s" ragu_a_arrival_cycles)" 3
check_eq "fill_delayed (all 3 episodes start from idle, so every conflict is fill-classified)" "$(get_stat "$s" ragu_a_fill_delayed)" 3
check_eq "aggregate M3 = 100*(6/3-1) = 100% (the TOTAL time impact across all 3 episodes combined)" "$(( 100 * ( $(get_stat "$s" ragu_a_active_cycles) - $(get_stat "$s" ragu_a_arrival_cycles) ) / $(get_stat "$s" ragu_a_arrival_cycles) ))" 100
echo

echo "=== Summary: $pass passed, $fail failed ==="
if [ "$fail" -gt 0 ]; then
    exit 1
fi
echo "All stats-invariant checks passed."
