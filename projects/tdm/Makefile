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

.PHONY: sim-crossbar sim-tdm sim-crossbar-sv sim-tdm-sv unit clean help

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

clean:
	$(call run_edaf,clean TYPE=all)

help:
	@cat <<'EOF'
	projects/tdm — sim shortcuts

	Targets:
	  sim-crossbar     Native SystemC crossbar   (edaf sim IMPL=crossbar)
	  sim-tdm          Native SystemC TDM        (edaf sim IMPL=tdm)
	  sim-crossbar-sv  Verilated SV crossbar      (edaf sim IMPL=crossbar,sv SV_MODS=top_crossbar)
	  sim-tdm-sv       Verilated SV TDM           (edaf sim IMPL=tdm,sv SV_MODS=top_tdm; needs top_tdm_sv.hpp)
	  unit             Unit tests under tb/unit/  (edaf unit; TOP=name to run one suite)
	  clean            Remove sim/ and imp/       (edaf clean TYPE=all)

	Variables (forwarded to edaf):
	  MODE=sc|sv    Simulation mode (default: sc)
	  CLK=ns        Clock period in ns (default: 1)
	  PARAMS=K=V,.. Design params, e.g. PARAMS=N_BANK=16,N_ROW=1024

	Stimulus selection and harness knobs (SEL_IN_DIR, SEL_NO_MONITOR, ...) are
	plain env vars read by the testbench — set them ahead of the make command:
	  SEL_IN_DIR=sample_tdm make sim-tdm
	EOF
