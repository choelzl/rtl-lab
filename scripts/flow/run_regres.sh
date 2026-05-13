#!/usr/bin/env bash

# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

set -uo pipefail

cd "$CODE_HOME/ai-core" || exit 1

PASS=0
FAIL=0
RESULTS=()

run() {
    local desc="$1"
    shift
    if make "$@" > /dev/null 2>&1; then
        RESULTS+=("  PASS  $desc")
        ((PASS++))
    else
        RESULTS+=("  FAIL  $desc")
        ((FAIL++))
    fi
}

CLK=1.35

# -----------------------------------------------------------------------------
# Baseline 4x8
# -----------------------------------------------------------------------------
run "sim          Baseline 4x8"  sim          TOP_LEVEL=top_bas_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_sim                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Baseline 4x8"  syn          TOP_LEVEL=top_bas_4x8                      OUT_DIR=bas_4x8_syn                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0"
run "post-syn-sim Baseline 4x8"  post-syn-sim TOP_LEVEL=top_bas_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_post_syn_sim  NETLIST_DIR=bas_4x8_syn  PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Baseline 4x8"  post-syn-sta TOP_LEVEL=top_bas_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_post_syn_sta  NETLIST_DIR=bas_4x8_syn
run "post-syn-dpa Baseline 4x8"  post-syn-dpa TOP_LEVEL=top_bas_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_post_syn_dpa  NETLIST_DIR=bas_4x8_syn  VCD_DIR=bas_4x8_post_syn_sim

# -----------------------------------------------------------------------------
# Baseline 8x8
# -----------------------------------------------------------------------------
run "sim          Baseline 8x8"  sim          TOP_LEVEL=top_bas_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_8x8_sim                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=8 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Baseline 8x8"  syn          TOP_LEVEL=top_bas_8x8                      OUT_DIR=bas_8x8_syn                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0"
run "post-syn-sim Baseline 8x8"  post-syn-sim TOP_LEVEL=top_bas_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_8x8_post_syn_sim  NETLIST_DIR=bas_8x8_syn  PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=8 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Baseline 8x8"  post-syn-sta TOP_LEVEL=top_bas_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_8x8_post_syn_sta  NETLIST_DIR=bas_8x8_syn
run "post-syn-dpa Baseline 8x8"  post-syn-dpa TOP_LEVEL=top_bas_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_8x8_post_syn_dpa  NETLIST_DIR=bas_8x8_syn  VCD_DIR=bas_8x8_post_syn_sim

# -----------------------------------------------------------------------------
# Baseline 4x8 SC
# -----------------------------------------------------------------------------
run "sim          Baseline 4x8 SC"  sim          TOP_LEVEL=top_bas_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_sc_sim                                       PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Baseline 4x8 SC"  syn          TOP_LEVEL=top_bas_4x8_sc                      OUT_DIR=bas_4x8_sc_syn                                       PARAMS="IS_PIPELINED=1 MULT_TYPE=0"
run "post-syn-sim Baseline 4x8 SC"  post-syn-sim TOP_LEVEL=top_bas_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_sc_post_syn_sim  NETLIST_DIR=bas_4x8_sc_syn  PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Baseline 4x8 SC"  post-syn-sta TOP_LEVEL=top_bas_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_sc_post_syn_sta  NETLIST_DIR=bas_4x8_sc_syn
run "post-syn-dpa Baseline 4x8 SC"  post-syn-dpa TOP_LEVEL=top_bas_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=bas_4x8_sc_post_syn_dpa  NETLIST_DIR=bas_4x8_sc_syn  VCD_DIR=bas_4x8_sc_post_syn_sim

# -----------------------------------------------------------------------------
# Winograd 4x8
# -----------------------------------------------------------------------------
run "sim          Winograd 4x8"  sim          TOP_LEVEL=top_win_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_sim                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Winograd 4x8"  syn          TOP_LEVEL=top_win_4x8                      OUT_DIR=win_4x8_syn                                    PARAMS="IS_PIPELINED=1 MULT_TYPE=0"
run "post-syn-sim Winograd 4x8"  post-syn-sim TOP_LEVEL=top_win_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_post_syn_sim  NETLIST_DIR=win_4x8_syn  PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Winograd 4x8"  post-syn-sta TOP_LEVEL=top_win_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_post_syn_sta  NETLIST_DIR=win_4x8_syn
run "post-syn-dpa Winograd 4x8"  post-syn-dpa TOP_LEVEL=top_win_4x8  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_post_syn_dpa  NETLIST_DIR=win_4x8_syn  VCD_DIR=win_4x8_post_syn_sim

# -----------------------------------------------------------------------------
# Winograd 4x8 SC
# -----------------------------------------------------------------------------
run "sim          Winograd 4x8 SC"  sim          TOP_LEVEL=top_win_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_sc_sim                                       PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Winograd 4x8 SC"  syn          TOP_LEVEL=top_win_4x8_sc                      OUT_DIR=win_4x8_sc_syn                                       PARAMS="IS_PIPELINED=1 MULT_TYPE=0"
run "post-syn-sim Winograd 4x8 SC"  post-syn-sim TOP_LEVEL=top_win_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_sc_post_syn_sim  NETLIST_DIR=win_4x8_sc_syn  PARAMS="IS_PIPELINED=1 MULT_TYPE=0 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Winograd 4x8 SC"  post-syn-sta TOP_LEVEL=top_win_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_sc_post_syn_sta  NETLIST_DIR=win_4x8_sc_syn
run "post-syn-dpa Winograd 4x8 SC"  post-syn-dpa TOP_LEVEL=top_win_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=win_4x8_sc_post_syn_dpa  NETLIST_DIR=win_4x8_sc_syn  VCD_DIR=win_4x8_sc_post_syn_sim

# -----------------------------------------------------------------------------
# Square 4x8 SC
# -----------------------------------------------------------------------------
run "sim          Square 4x8 SC"  sim          TOP_LEVEL=top_sqr_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_sc_sim                                       PARAMS="IS_PIPELINED=1 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Square 4x8 SC"  syn          TOP_LEVEL=top_sqr_4x8_sc                      OUT_DIR=sqr_4x8_sc_syn                                       PARAMS="IS_PIPELINED=1"
run "post-syn-sim Square 4x8 SC"  post-syn-sim TOP_LEVEL=top_sqr_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_sc_post_syn_sim  NETLIST_DIR=sqr_4x8_sc_syn  PARAMS="IS_PIPELINED=1 DYN_RANGE_A=4 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Square 4x8 SC"  post-syn-sta TOP_LEVEL=top_sqr_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_sc_post_syn_sta  NETLIST_DIR=sqr_4x8_sc_syn
run "post-syn-dpa Square 4x8 SC"  post-syn-dpa TOP_LEVEL=top_sqr_4x8_sc  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_sc_post_syn_dpa  NETLIST_DIR=sqr_4x8_sc_syn  VCD_DIR=sqr_4x8_sc_post_syn_sim

# -----------------------------------------------------------------------------
# Square 8x8
# -----------------------------------------------------------------------------
run "sim          Square 8x8"  sim          TOP_LEVEL=top_sqr_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_sim                                    PARAMS="IS_PIPELINED=1 DYN_RANGE_A=8 DYN_RANGE_B=8 DIST_TYPE=0"
run "syn          Square 8x8"  syn          TOP_LEVEL=top_sqr_8x8                      OUT_DIR=sqr_8x8_syn                                    PARAMS="IS_PIPELINED=1"
run "post-syn-sim Square 8x8"  post-syn-sim TOP_LEVEL=top_sqr_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_post_syn_sim  NETLIST_DIR=sqr_8x8_syn  PARAMS="IS_PIPELINED=1 DYN_RANGE_A=8 DYN_RANGE_B=8 DIST_TYPE=0"
run "post-syn-sta Square 8x8"  post-syn-sta TOP_LEVEL=top_sqr_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_post_syn_sta  NETLIST_DIR=sqr_8x8_syn
run "post-syn-dpa Square 8x8"  post-syn-dpa TOP_LEVEL=top_sqr_8x8  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_post_syn_dpa  NETLIST_DIR=sqr_8x8_syn  VCD_DIR=sqr_8x8_post_syn_sim

# -----------------------------------------------------------------------------
# Square 4x8 Alpha
# -----------------------------------------------------------------------------
run "sim          Alpha"  sim          TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_sim                                          PARAMS="IS_PIPELINED=1 IS_SQUARE=0 DYN_RANGE_A=4 DIST_TYPE=0"
run "syn          Alpha"  syn          TOP_LEVEL=top_sqr_4x8_sc_alpha                      OUT_DIR=sqr_4x8_alpha_syn                                          PARAMS="IS_PIPELINED=1 IS_SQUARE=0"
run "post-syn-sim Alpha"  post-syn-sim TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_post_syn_sim  NETLIST_DIR=sqr_4x8_alpha_syn  PARAMS="IS_PIPELINED=1 IS_SQUARE=0 DYN_RANGE_A=4 DIST_TYPE=0"
run "post-syn-sta Alpha"  post-syn-sta TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_post_syn_sta  NETLIST_DIR=sqr_4x8_alpha_syn
run "post-syn-dpa Alpha"  post-syn-dpa TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_post_syn_dpa  NETLIST_DIR=sqr_4x8_alpha_syn  VCD_DIR=sqr_4x8_alpha_post_syn_sim

# -----------------------------------------------------------------------------
# Square 4x8 Alpha Squared
# -----------------------------------------------------------------------------
run "sim          Alpha Squared"  sim          TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_sqr_sim                                              PARAMS="IS_PIPELINED=1 IS_SQUARE=1 DYN_RANGE_A=4 DIST_TYPE=0"
run "syn          Alpha Squared"  syn          TOP_LEVEL=top_sqr_4x8_sc_alpha                      OUT_DIR=sqr_4x8_alpha_sqr_syn                                              PARAMS="IS_PIPELINED=1 IS_SQUARE=1"
run "post-syn-sim Alpha Squared"  post-syn-sim TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_sqr_post_syn_sim  NETLIST_DIR=sqr_4x8_alpha_sqr_syn  PARAMS="IS_PIPELINED=1 IS_SQUARE=1 DYN_RANGE_A=4 DIST_TYPE=0"
run "post-syn-sta Alpha Squared"  post-syn-sta TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_sqr_post_syn_sta  NETLIST_DIR=sqr_4x8_alpha_sqr_syn
run "post-syn-dpa Alpha Squared"  post-syn-dpa TOP_LEVEL=top_sqr_4x8_sc_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_4x8_alpha_sqr_post_syn_dpa  NETLIST_DIR=sqr_4x8_alpha_sqr_syn  VCD_DIR=sqr_4x8_alpha_sqr_post_syn_sim

# -----------------------------------------------------------------------------
# Square 8x8 Alpha
# -----------------------------------------------------------------------------
run "sim          Square 8x8 Alpha"  sim          TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_sim                                          PARAMS="IS_PIPELINED=1 IS_SQUARE=0 DYN_RANGE_A=8 DIST_TYPE=0"
run "syn          Square 8x8 Alpha"  syn          TOP_LEVEL=top_sqr_8x8_alpha                      OUT_DIR=sqr_8x8_alpha_syn                                          PARAMS="IS_PIPELINED=1 IS_SQUARE=0"
run "post-syn-sim Square 8x8 Alpha"  post-syn-sim TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_post_syn_sim  NETLIST_DIR=sqr_8x8_alpha_syn  PARAMS="IS_PIPELINED=1 IS_SQUARE=0 DYN_RANGE_A=8 DIST_TYPE=0"
run "post-syn-sta Square 8x8 Alpha"  post-syn-sta TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_post_syn_sta  NETLIST_DIR=sqr_8x8_alpha_syn
run "post-syn-dpa Square 8x8 Alpha"  post-syn-dpa TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_post_syn_dpa  NETLIST_DIR=sqr_8x8_alpha_syn  VCD_DIR=sqr_8x8_alpha_post_syn_sim

# -----------------------------------------------------------------------------
# Square 8x8 Alpha Squared
# -----------------------------------------------------------------------------
run "sim          Square 8x8 Alpha Squared"  sim          TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_sqr_sim                                              PARAMS="IS_PIPELINED=1 IS_SQUARE=1 DYN_RANGE_A=8 DIST_TYPE=0"
run "syn          Square 8x8 Alpha Squared"  syn          TOP_LEVEL=top_sqr_8x8_alpha                      OUT_DIR=sqr_8x8_alpha_sqr_syn                                              PARAMS="IS_PIPELINED=1 IS_SQUARE=1"
run "post-syn-sim Square 8x8 Alpha Squared"  post-syn-sim TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_sqr_post_syn_sim  NETLIST_DIR=sqr_8x8_alpha_sqr_syn  PARAMS="IS_PIPELINED=1 IS_SQUARE=1 DYN_RANGE_A=8 DIST_TYPE=0"
run "post-syn-sta Square 8x8 Alpha Squared"  post-syn-sta TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_sqr_post_syn_sta  NETLIST_DIR=sqr_8x8_alpha_sqr_syn
run "post-syn-dpa Square 8x8 Alpha Squared"  post-syn-dpa TOP_LEVEL=top_sqr_8x8_alpha  CLK_PERIOD_NS=$CLK  OUT_DIR=sqr_8x8_alpha_sqr_post_syn_dpa  NETLIST_DIR=sqr_8x8_alpha_sqr_syn  VCD_DIR=sqr_8x8_alpha_sqr_post_syn_sim

# -----------------------------------------------------------------------------
# Summary
# -----------------------------------------------------------------------------
echo ""
echo "Regression results:"
echo "-------------------"
for r in "${RESULTS[@]}"; do
    echo "$r"
done
echo "-------------------"
echo "PASS: $PASS  FAIL: $FAIL  TOTAL: $((PASS + FAIL))"
echo ""

[ $FAIL -eq 0 ]
