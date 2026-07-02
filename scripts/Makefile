# EDA & RTL Flow Makefile — scripts/Makefile
#
# Invoked via `edaf` (on PATH after sourcing sourceme.sh or via direnv).
# Usage: edaf <target> [KEY=VALUE ...]   (run `edaf help` for full reference)

SHELL         := /bin/bash
.SHELLFLAGS   := -euo pipefail -c
.ONESHELL:
.PHONY: sim syn sta post-sim dpa unit clean fmt fmt-sv fmt-sc help

# ── Sanity ────────────────────────────────────────────────────────────────────
ifeq ($(origin RTL_LAB_HOME),undefined)
  $(error RTL_LAB_HOME not set — source sourceme.sh)
endif

# ── User variables ────────────────────────────────────────────────────────────
# TOP: no global default — sim falls back to "top" (target-specific below);
#      all other targets require it explicitly via _require_top.
IMPL           ?=
PARAMS         ?=
SV_MODS        ?=
MODE           ?=
CLK            ?= 1
OUT            ?=
NETLIST        ?=
VCD            ?=
KEEP_HIERARCHY ?= 0
TYPE           ?= all

# Project: explicit > env > CURDIR path
PROJECT ?= $(shell echo "$(CURDIR)" | grep -oP '(?<=/projects/)[^/]+(?=/|$$)' || true)

# ── Make-time computed values ─────────────────────────────────────────────────
_comma  := ,
_impl1  := $(firstword $(subst $(_comma), ,$(IMPL)))
# Effective OUT: honour explicit OUT, else derive from TOP + first IMPL token
_eff_out = $(if $(OUT),$(OUT),$(if $(IMPL),$(TOP)_$(_impl1),$(TOP)))

PROJ := $(RTL_LAB_HOME)/projects/$(PROJECT)

# ── Shared shell snippets (expanded into recipes via $(X), Make vars inline) ──

# IMPL=a,b,c → impl_dflags=(-DIMPL_A -DIMPL_B -DIMPL_C)
define _parse_impl
impl_dflags=()
if [[ -n "$(IMPL)" ]]; then
  IFS=',' read -ra _is <<< "$(IMPL)"
  for _i in "$${_is[@]}"; do
    _up=$$(printf '%s' "$$_i" | tr '[:lower:]' '[:upper:]' | tr '-' '_')
    impl_dflags+=("-DIMPL_$${_up}")
  done
fi
endef

# PARAMS=K=V,FLAG → g_flags=(-GK=V ...) c_dflags=(-DK=V -DFLAG ...)
define _parse_params
g_flags=(); c_dflags=()
if [[ -n "$(PARAMS)" ]]; then
  IFS=',' read -ra _ps <<< "$(PARAMS)"
  for _p in "$${_ps[@]}"; do
    c_dflags+=("-D$$_p")
    [[ "$$_p" == *=* ]] && g_flags+=("-G$$_p")
  done
fi
endef

# inc_flags from rtl/ include dirs; vlt_files from *.vlt suppressions
define _collect_inc_vlt
inc_flags=()
while IFS= read -r _d; do inc_flags+=("-I$$_d"); done \
  < <(find "$(PROJ)/rtl" -type d | sort)
vlt_files=()
while IFS= read -r _f; do vlt_files+=("$$_f"); done \
  < <(find "$(PROJ)/rtl" -name "*.vlt" | sort)
endef

define _collect_inc_sv
sv_files=()
while IFS= read -r _f; do sv_files+=("$$_f"); done \
  < <(find "$(PROJ)/rtl" -name "*.sv" | sort)
endef

define _require_project
[[ -n "$(PROJECT)" ]] || { echo "edaf: PROJECT not set and not inside projects/<name>/" >&2; exit 1; }
[[ -d "$(PROJ)" ]]    || { echo "edaf: project directory not found: $(PROJ)" >&2; exit 1; }
endef

define _require_top
[[ -n "$(TOP)" ]] || { echo "edaf: TOP not set" >&2; exit 1; }
endef

define _require_netlist
[[ -n "$(NETLIST)" ]] || { echo "edaf: NETLIST not set" >&2; exit 1; }
endef

define _require_vcd
[[ -n "$(VCD)" ]] || { echo "edaf: VCD not set" >&2; exit 1; }
endef

# EFF_OUT + imp dir creation — shared by syn, sta, dpa
define _setup_imp
EFF_OUT="$(_eff_out)"
mkdir -p "$(PROJ)/imp/$$EFF_OUT/output" "$(PROJ)/imp/$$EFF_OUT/report"
endef

# ── sim ───────────────────────────────────────────────────────────────────────
sim: TOP ?= top
sim:
	$(_require_project)
	$(_require_top)
	$(_parse_impl)
	$(_parse_params)
	$(_collect_inc_vlt)
	$(_collect_inc_sv)
	EFF_OUT="$(_eff_out)"
	SIM_DIR="$(PROJ)/sim/$$EFF_OUT"
	mkdir -p "$$SIM_DIR/build" "$$SIM_DIR/output"
	HAS_SC=false; HAS_SV=false
	[[ -f "$(PROJ)/tb/systemc/tb_$(TOP).cpp" ]] && HAS_SC=true || true
	[[ -f "$(PROJ)/tb/tb_$(TOP).sv" ]]          && HAS_SV=true || true
	RMODE="$(MODE)"
	if [[ -z "$$RMODE" ]]; then
	  if $$HAS_SC; then RMODE=sc; elif $$HAS_SV; then RMODE=sv
	  else echo "edaf sim: no testbench found for TOP=$(TOP)" >&2; exit 1; fi
	fi
	if [[ "$$RMODE" == sv ]]; then
	  verilator \
	    -sv --binary --timing --trace \
	    --trace-max-array 0 --trace-max-width 0 \
	    -Wall -Wno-fatal \
	    -DVCD "-DCLK_PERIOD_NS=$(CLK)" \
	    "$${impl_dflags[@]+"$${impl_dflags[@]}"}" \
	    "$${c_dflags[@]+"$${c_dflags[@]}"}" \
	    "$${g_flags[@]+"$${g_flags[@]}"}" \
	    "$${inc_flags[@]+"$${inc_flags[@]}"}" \
	    "$${vlt_files[@]+"$${vlt_files[@]}"}" \
		"$${sv_files[@]+"$${sv_files[@]}"}" \
	    --top-module "tb_$(TOP)" \
	    "$(PROJ)/tb/tb_$(TOP).sv" \
	    -Mdir "$$SIM_DIR/build/obj_dir" \
	    -o    "$$SIM_DIR/build/simv" \
	    | tee "$$SIM_DIR/output/compile.log"
	  (cd "$$SIM_DIR/output" && exec "$$SIM_DIR/build/simv") \
	    | tee "$$SIM_DIR/output/run.log"
	elif [[ "$$RMODE" == sc ]]; then
	  export SC_COPYRIGHT_MESSAGE=DISABLE
	  SC_CFLAGS="-std=c++17"
	  SC_CFLAGS+=" -I$(PROJ)/tb/systemc -I$(PROJ)/rtl/systemc"
	  SC_CFLAGS+=" -DCLK_PERIOD_NS=$(CLK)"
	  for _f in "$${impl_dflags[@]+"$${impl_dflags[@]}"}"; do SC_CFLAGS+=" $$_f"; done
	  for _f in "$${c_dflags[@]+"$${c_dflags[@]}"}";   do SC_CFLAGS+=" $$_f"; done
	  if [[ -n "$(SV_MODS)" ]]; then
	    VROOT=$$(verilator --getenv VERILATOR_ROOT)
	    sv_mod_incs=(); sv_mod_archives=()
	    IFS=',' read -ra _mods <<< "$(SV_MODS)"
	    for _mod in "$${_mods[@]}"; do
	      _mod_dir="$$SIM_DIR/build/obj_$${_mod}"
	      _sv_file=$$(find "$(PROJ)/rtl" -name "$${_mod}.sv" -print -quit)
	      [[ -n "$$_sv_file" ]] || \
	        { echo "edaf: SV_MODS: cannot find $${_mod}.sv under $(PROJ)/rtl/" >&2; exit 1; }
	      echo "==> Verilating $${_mod} ..."
	      verilator --sc --build -sv \
	        -Wall -Wno-fatal \
	        "$${inc_flags[@]+"$${inc_flags[@]}"}" \
	        "$${g_flags[@]+"$${g_flags[@]}"}" \
	        "$${vlt_files[@]+"$${vlt_files[@]}"}" \
	        "$${sv_files[@]+"$${sv_files[@]}"}" \
	        --top-module "$${_mod}" \
	        -Mdir "$$_mod_dir" \
	        | tee "$$SIM_DIR/output/verilate_$${_mod}.log"
	      sv_mod_incs+=("-I$$_mod_dir")
	      while IFS= read -r _a; do sv_mod_archives+=("$$_a"); done \
	        < <(find "$$_mod_dir" -name "*.a" | sort)
	    done
	    VINC="$$VROOT/include"
	    g++ $$SC_CFLAGS \
	      "$${sv_mod_incs[@]}" \
	      "-I$$VINC" \
	      "-I$(SYSTEMC_INCLUDE)" \
	      "$(PROJ)/tb/systemc/tb_$(TOP).cpp" \
	      "$$VINC/verilated.cpp" \
	      "$${sv_mod_archives[@]+"$${sv_mod_archives[@]}"}" \
	      -o "$$SIM_DIR/build/simv" \
	      "-L$(SYSTEMC_LIB)" "-Wl,-rpath,$(SYSTEMC_LIB)" -lsystemc -pthread \
	      | tee "$$SIM_DIR/output/compile.log"
	  else
	    g++ $$SC_CFLAGS \
	      "-I$(SYSTEMC_INCLUDE)" \
	      "$(PROJ)/tb/systemc/tb_$(TOP).cpp" \
	      -o "$$SIM_DIR/build/simv" \
	      "-L$(SYSTEMC_LIB)" "-Wl,-rpath,$(SYSTEMC_LIB)" -lsystemc -pthread \
	      | tee "$$SIM_DIR/output/compile.log"
	  fi
	  (cd "$$SIM_DIR/output" && exec "$$SIM_DIR/build/simv") \
	    | tee "$$SIM_DIR/output/run.log"
	else
	  echo "edaf sim: MODE must be sv or sc, got '$$RMODE'" >&2; exit 1
	fi

# ── syn ───────────────────────────────────────────────────────────────────────
syn:
	$(_require_project)
	$(_require_top)
	$(_setup_imp)
	export PROJECT="$(PROJECT)" TOP="$(TOP)" OUT="$$EFF_OUT" \
	       CLK="$(CLK)" PARAMS="$(PARAMS)" KEEP_HIERARCHY="$(KEEP_HIERARCHY)"
	yosys -l "$(PROJ)/imp/$$EFF_OUT/output/yosys.log" \
	    -c "$(RTL_LAB_HOME)/scripts/syn.tcl"

# ── sta ───────────────────────────────────────────────────────────────────────
sta:
	$(_require_project)
	$(_require_top)
	$(_require_netlist)
	$(_setup_imp)
	export PROJECT="$(PROJECT)" TOP="$(TOP)" OUT="$$EFF_OUT" \
	       CLK="$(CLK)" NETLIST="$(NETLIST)"
	sta -no_splash -exit "$(RTL_LAB_HOME)/scripts/sta.tcl" \
	    | tee "$(PROJ)/imp/$$EFF_OUT/output/opensta.log"

# ── dpa ───────────────────────────────────────────────────────────────────────
dpa:
	$(_require_project)
	$(_require_top)
	$(_require_netlist)
	$(_require_vcd)
	$(_setup_imp)
	export PROJECT="$(PROJECT)" TOP="$(TOP)" OUT="$$EFF_OUT" \
	       CLK="$(CLK)" NETLIST="$(NETLIST)" VCD="$(VCD)" \
	       KEEP_HIERARCHY="$(KEEP_HIERARCHY)"
	sta -no_splash -exit "$(RTL_LAB_HOME)/scripts/dpa.tcl" \
	    | tee "$(PROJ)/imp/$$EFF_OUT/output/opensta.log"

# ── post-sim ──────────────────────────────────────────────────────────────────
post-sim:
	$(_require_project)
	$(_require_top)
	$(_require_netlist)
	$(_parse_params)
	EFF_OUT="$(_eff_out)"
	SIM_DIR="$(PROJ)/sim/$$EFF_OUT"
	export PROJECT="$(PROJECT)" NETLIST="$(NETLIST)"
	mkdir -p "$$SIM_DIR/build" "$$SIM_DIR/output"
	verilator \
	  -sv --binary --timing --trace \
	  --trace-underscore \
	  --trace-max-array 0 --trace-max-width 0 \
	  -Wall -Wno-fatal \
	  -Wno-SPECIFYIGN -Wno-DECLFILENAME -Wno-UNUSEDSIGNAL \
	  -DPOST_SYN_SIM -DVCD -DPOST_SYNTH=1 "-DCLK_PERIOD_NS=$(CLK)" \
	  --x-initial fast --x-assign fast \
	  "$${g_flags[@]+"$${g_flags[@]}"}" \
	  --top-module "tb_$(TOP)" \
	  -f "$(RTL_LAB_HOME)/scripts/post_sim.f" \
	  "$(PROJ)/tb/tb_$(TOP).sv" \
	  -Mdir "$$SIM_DIR/build/obj_dir" \
	  -o    "$$SIM_DIR/build/simv" \
	  | tee "$$SIM_DIR/output/compile.log"
	(cd "$$SIM_DIR/output" && exec "$$SIM_DIR/build/simv") \
	  | tee "$$SIM_DIR/output/run.log"

# ── unit ──────────────────────────────────────────────────────────────────────
unit:
	$(_require_project)
	export PROJECT="$(PROJECT)" PROJ="$(PROJ)" TOP="$(TOP)" PARAMS="$(PARAMS)"
	bash "$(RTL_LAB_HOME)/scripts/_unit.sh"

# ── clean ─────────────────────────────────────────────────────────────────────
clean:
	$(_require_project)
	EFF_OUT="$(_eff_out)"
	case "$(TYPE)" in
	  sim)
	    [[ -n "$(TOP)" ]] || { echo "edaf clean TYPE=sim: TOP not set" >&2; exit 1; }
	    echo "Removing $(PROJ)/sim/$$EFF_OUT"
	    rm -rf "$(PROJ)/sim/$$EFF_OUT"
	    ;;
	  syn|imp)
	    [[ -n "$(TOP)" ]] || { echo "edaf clean TYPE=syn: TOP not set" >&2; exit 1; }
	    echo "Removing $(PROJ)/imp/$$EFF_OUT"
	    rm -rf "$(PROJ)/imp/$$EFF_OUT"
	    ;;
	  all)
	    echo "Removing $(PROJ)/sim and $(PROJ)/imp"
	    rm -rf "$(PROJ)/sim" "$(PROJ)/imp"
	    ;;
	  *)
	    echo "edaf clean: TYPE must be sim, syn, or all (got '$(TYPE)')" >&2; exit 1
	    ;;
	esac

# ── fmt ───────────────────────────────────────────────────────────────────────
_SV_FLAGS := --indentation_spaces=4 --wrap_spaces=4 --column_limit=100 \
             --assignment_statement_alignment=align \
             --case_items_alignment=align \
             --class_member_variable_alignment=align \
             --distribution_items_alignment=align \
             --enum_assignment_statement_alignment=align \
             --formal_parameters_alignment=align \
             --module_net_variable_alignment=align \
             --named_parameter_alignment=align \
             --named_port_alignment=align \
             --port_declarations_alignment=align \
             --port_declarations_right_align_packed_dimensions=true \
             --port_declarations_right_align_unpacked_dimensions=true \
             --struct_union_members_alignment=align

_SC_STYLE := {BasedOnStyle: LLVM, IndentWidth: 4, ContinuationIndentWidth: 4, \
              TabWidth: 4, UseTab: Never, ColumnLimit: 100, \
              AlignConsecutiveAssignments: true, AlignConsecutiveDeclarations: true, \
              AlignConsecutiveMacros: true, AlignTrailingComments: true, \
              AllowShortFunctionsOnASingleLine: Inline}

_FMT_ROOT     := $(if $(PROJECT),$(PROJ),$(CURDIR))
_FMT_PATHSPEC := $(if $(filter $(RTL_LAB_HOME),$(_FMT_ROOT)),.,$(patsubst $(RTL_LAB_HOME)/%,%,$(_FMT_ROOT)))
_FMT_FILES    := $(shell git -C $(RTL_LAB_HOME) ls-files -co --exclude-standard -- "$(_FMT_PATHSPEC)")
_SC_SOURCES   := $(foreach f,$(filter %.cpp %.h %.hpp,$(_FMT_FILES)),$(if $(wildcard $(RTL_LAB_HOME)/$(f)),$(RTL_LAB_HOME)/$(f)))
_SV_SOURCES   := $(foreach f,$(filter %.sv %.v,$(_FMT_FILES)),$(if $(wildcard $(RTL_LAB_HOME)/$(f)),$(RTL_LAB_HOME)/$(f)))

fmt: fmt-sv fmt-sc

fmt-sv:
	@echo "Formatting SystemVerilog under $(_FMT_ROOT) ..."
	$(if $(_SV_SOURCES),verible-verilog-format $(_SV_FLAGS) --inplace $(_SV_SOURCES),echo "No SV files.")

fmt-sc:
	@echo "Formatting SystemC under $(_FMT_ROOT) ..."
	$(if $(_SC_SOURCES),clang-format -style="$(_SC_STYLE)" -i $(_SC_SOURCES),echo "No SC files.")

# ── help ──────────────────────────────────────────────────────────────────────
help:
	@cat <<'EOF'
	edaf — EDA flow

	Usage:
	  edaf <target> [KEY=VALUE ...]

	Targets:
	  sim        Simulate — auto-detects SV or SC mode from testbench presence
	  syn        Logic synthesis (Yosys + yosys-slang)
	  sta        Post-synthesis static timing analysis (OpenSTA)
	  post-sim   Post-synthesis gate-level simulation
	  dpa        Post-synthesis dynamic power analysis (OpenSTA + VCD)
	  unit       Build and run unit tests under tb/unit/
	  clean      Remove build outputs (see TYPE=)
	  fmt        Format SV and C++ sources under PROJECT, else current directory
	  help       Show this message

	Variables:
	  PROJECT=name        Project (default: detected from CWD under projects/<name>/)
	  TOP=module          Top-level module name
	  IMPL=a[,b,c]        Implementation tags  →  -DIMPL_A -DIMPL_B -DIMPL_C
	  PARAMS=K=V[,FLAG]   Design params and defines:
	                        K=V  → -GK=V (SV generic) + -DK=V (C++ define)
	                        FLAG → -DFLAG (C++ define only, no -G)
	  SV_MODS=a[,b]       (sim sc) Verilate each SV module independently;
	                       each becomes its own Verilated SC class
	  MODE=sv|sc          Force simulation mode (default: auto from testbench)
	  CLK=ns              Clock period in ns (default: 1)
	  OUT=dir             Output dir (default: <TOP> or <TOP>_<first_impl>)
	  NETLIST=dir         Netlist dir for post-sim / dpa
	  VCD=dir             VCD source dir for dpa
	  KEEP_HIERARCHY=0|1  Keep hierarchy in synthesis (default: 0)
	  TYPE=sim|syn|all    Scope for clean (default: all)

	Examples:
	  edaf sim  TOP=top_tdm
	  edaf sim  TOP=top_tdm IMPL=crossbar,buf CLK=2
	  edaf sim  TOP=top_tdm SV_MODS=top_tdm
	  edaf sim  TOP=top_tdm SV_MODS=mod_a,mod_b PARAMS=N_BANK=4,VERBOSE
	  edaf syn  TOP=top_tdm KEEP_HIERARCHY=1
	  edaf sta  TOP=top_tdm NETLIST=top_tdm
	  edaf dpa  TOP=top_tdm NETLIST=top_tdm VCD=top_tdm
	  edaf unit TOP=buffer
	  edaf clean TYPE=sim TOP=top_tdm
	  edaf clean TYPE=all
	EOF
