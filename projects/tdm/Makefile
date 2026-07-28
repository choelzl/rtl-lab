# projects/tdm/Makefile — sim shortcuts for the tdm project
#
# Thin wrapper around `edaf sim` (see scripts/Makefile / `edaf help`).
# Does NOT require sourceme.sh to be sourced first: run_edaf (below) bootstraps
# edaf itself — via `nix develop` when nix is available and edaf isn't already
# on PATH, else by adding scripts/ to PATH directly.
# `edaf` infers PROJECT=tdm from this directory, so PROJECT= is never needed here.

MODE   ?= sc
CLK    ?= 1
PARAMS ?=

# A literal comma to use inside $(call ...) arguments below — $(call) splits
# its argument text on top-level commas before substituting $(...), so a bare
# "crossbar,sv" would be split into two call arguments and silently dropped.
comma := ,

# Repo root, computed from this Makefile's own location (projects/tdm/Makefile
# -> repo root is two levels up) — works regardless of the caller's cwd or
# whether RTL_LAB_HOME is already exported.
RTL_LAB_HOME ?= $(abspath $(dir $(abspath $(lastword $(MAKEFILE_LIST))))../..)

# run_edaf: bootstrap edaf onto PATH before invoking it.
#   - nix available and edaf NOT already on PATH -> run it inside `nix develop`
#     (flake.nix's devShell puts scripts/ on PATH and sets RTL_LAB_HOME itself).
#   - otherwise -> add scripts/ to PATH (and export RTL_LAB_HOME, which edaf
#     requires) directly; a no-op if edaf was already reachable.
define run_edaf
if command -v nix >/dev/null 2>&1 && ! command -v edaf >/dev/null 2>&1; then \
	nix develop $(RTL_LAB_HOME) --command edaf $(1); \
else \
	PATH="$(RTL_LAB_HOME)/scripts:$$PATH" RTL_LAB_HOME="$(RTL_LAB_HOME)" edaf $(1); \
fi
endef

# run_in_env: bootstrap SYSTEMC_INCLUDE/SYSTEMC_LIB via `nix develop` before
# running a doc/report/tools/*.sh script, the same way run_edaf bootstraps
# edaf itself. Those scripts compile tb_top.cpp directly with raw g++
# (several -D variants built concurrently, to sidestep edaf/`make sim-*`
# racing on one shared simv binary path — see run_hash_sweep.sh's own
# comment), so they need the devShell's SystemC env vars but not edaf.
# No-op passthrough if SYSTEMC_INCLUDE is already set (environment already
# sourced, or already inside `nix develop`).
define run_in_env
if [ -z "$$SYSTEMC_INCLUDE" ] && command -v nix >/dev/null 2>&1; then \
	nix develop $(RTL_LAB_HOME) --command bash -c "$(1)"; \
else \
	bash -c "$(1)"; \
fi
endef

.PHONY: sim-crossbar sim-tdm sim-crossbar-sv sim-tdm-sv unit check-metrics check hash-sweep eval-sweep clean help

# Every recipe's lines run in a single shell invocation instead of Make's
# default one-shell-per-line — without this, help's multi-line `cat <<'EOF'`
# heredoc body gets split into separate shell invocations and each text line
# is executed as its own bogus command (pre-existing bug, confirmed present
# before this Makefile's other changes too: `make help` failed with
# "execvp: projects/tdm: Permission denied" on the very first heredoc body
# line). Harmless for every other target here since they're already
# single-line recipes.
.ONESHELL:

sim-crossbar:
	$(call run_edaf,sim IMPL=crossbar MODE=$(MODE) CLK=$(CLK) PARAMS="$(PARAMS)")

sim-tdm:
	$(call run_edaf,sim IMPL=tdm MODE=$(MODE) CLK=$(CLK) PARAMS="$(PARAMS)")

sim-crossbar-sv:
	$(call run_edaf,sim IMPL=crossbar$(comma)sv MODE=$(MODE) SV_MODS=top_crossbar CLK=$(CLK) PARAMS="$(PARAMS)")

sim-tdm-sv:
	$(call run_edaf,sim IMPL=tdm$(comma)sv MODE=$(MODE) SV_MODS=top_tdm CLK=$(CLK) PARAMS="$(PARAMS)")

unit:
	$(call run_edaf,unit TOP=$(TOP))

# check-metrics: regression suite for tb_top.cpp's own M1/M2 conflict-metric
# accounting (gs.conflicts/delayed_l1/l2/l3, lvl_rd/lvl_wr, fill vs. genuine
# stall, NOP-request handling) — hand-built fixtures under
# tb/stimuli/metrics_selftest/ with exact, hand-verified expected values.
# Complements `unit` (which tests RTL correctness) by testing the
# testbench's OWN measurement code instead — see
# doc/report/tools/check_stats_invariants.sh's header for the full rationale
# and what each fixture checks.
check-metrics:
	$(call run_in_env,doc/report/tools/check_stats_invariants.sh $(OUT))

# check: everything correctness-related in one shot (RTL unit tests + the
# stats-measurement self-tests) — the target to run before trusting any
# M1/M2 numbers coming out of hash-sweep/eval-sweep below.
check: unit check-metrics

# hash-sweep: compares crossbar bank-hash schemes (baseline/XBAR_HASH_L1/
# XBAR_HASH_L1_V2/XBAR_HASH16/XBAR_HASH32, + XBAR_HASH_L2_COMPOSE) across the
# stimuli sources configured in the script, writing
# doc/report/data/eval_xbar_hash_sweep*.csv. OUT_DIR defaults to
# /tmp/tdm_hash_sweep; override with OUT=path.
# See doc/report/tools/run_hash_sweep.sh for the full variant/source matrix.
hash-sweep:
	$(call run_in_env,doc/report/tools/run_hash_sweep.sh $(OUT))

# eval-sweep: the full §5 evaluation sweep (crossbar vs. TDM-adaptive vs.
# TDM-RR, fenced and unfenced) across final/0-19, writing
# doc/report/data/eval_*.csv. Needs the proprietary final/ stimuli in place
# first — see tb/stimuli/final/.gitignore. OUT_DIR defaults to
# /tmp/tdm_eval_sweep; override with OUT=path.
eval-sweep:
	$(call run_in_env,doc/report/tools/run_eval_sweep.sh $(OUT))

clean:
	$(call run_edaf,clean TYPE=all)

# Note: .ONESHELL strips ALL leading whitespace AND drops blank lines from
# every recipe line, not just the one recipe-prefix tab — confirmed
# directly (isolated test Makefile), not assumed. A line starting with '-'
# is worse: Make reads that as its own per-recipe-line "ignore errors"
# modifier and swallows the whole line (also confirmed directly — an
# earlier draft used a dashed divider line and it silently vanished, not an
# assumption either). So this heredoc's text is written flush-left with
# inline separators instead of leading-space indentation, and uses "===="
# divider lines (not blank lines, not '-'-led lines) for section breaks —
# deliberate, permanent choices for this target, not something to "fix" by
# re-adding source-level indentation/blank lines/dashes later.
help:
	@cat <<'EOF'
	projects/tdm -- sim shortcuts
	============================
	Targets:
	sim-crossbar    | native SystemC crossbar   (edaf sim IMPL=crossbar)
	sim-tdm         | native SystemC TDM        (edaf sim IMPL=tdm)
	sim-crossbar-sv | verilated SV crossbar     (edaf sim IMPL=crossbar,sv SV_MODS=top_crossbar)
	sim-tdm-sv      | verilated SV TDM          (edaf sim IMPL=tdm,sv SV_MODS=top_tdm; needs top_tdm_sv.hpp)
	unit            | RTL unit tests under tb/unit/     (edaf unit; TOP=name to run one suite)
	check-metrics   | testbench M1/M2 stats self-test   (doc/report/tools/check_stats_invariants.sh)
	check           | unit + check-metrics -- run both before trusting any M1/M2 numbers
	hash-sweep      | compare crossbar bank-hash schemes (doc/report/tools/run_hash_sweep.sh)
	eval-sweep      | full section-5 crossbar-vs-TDM eval sweep (doc/report/tools/run_eval_sweep.sh;
	                  needs proprietary final/ stimuli, see tb/stimuli/final/.gitignore)
	clean           | remove sim/ and imp/       (edaf clean TYPE=all)
	====
	Variables (forwarded to edaf):
	MODE=sc|sv    -- simulation mode (default: sc)
	CLK=ns        -- clock period in ns (default: 1)
	PARAMS=K=V,.. -- design params, e.g. PARAMS=N_BANK=16,N_ROW=1024
	====
	Variables (check-metrics / hash-sweep / eval-sweep only):
	OUT=path -- output directory for compiled binaries + run artifacts
	            (each target has its own /tmp/tdm_* default -- see its
	            doc/report/tools/*.sh header)
	====
	Stimulus selection and harness knobs (SEL_IN_DIR, SEL_NO_MONITOR, ...) are
	plain env vars read by the testbench -- set them ahead of the make command:
	SEL_IN_DIR=sample_tdm make sim-tdm
	EOF
