# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

PROJECT        ?=
TOP_LEVEL      ?=
OUT_DIR        ?= no_name
NETLIST_DIR    ?= no_name
VCD_DIR        ?= no_name
CLK_PERIOD_NS  ?= 1
PARAMS         ?= none
TB_DEFS        ?= none
KEEP_HIERARCHY ?= 0
EXP            ?=

PROJ_DIR := $(RTL_LAB_HOME)/projects/$(PROJECT)

export SEL_PROJECT        := $(PROJECT)
export SEL_TOP_LEVEL      := $(TOP_LEVEL)
export SEL_OUT_DIR        := $(OUT_DIR)
export SEL_NETLIST_DIR    := $(NETLIST_DIR)
export SEL_VCD_DIR        := $(VCD_DIR)
export SEL_CLK_PERIOD_NS  := $(CLK_PERIOD_NS)
export SEL_PARAMS         := $(PARAMS)
export SEL_TB_DEFS        := $(TB_DEFS)
export SEL_KEEP_HIERARCHY := $(KEEP_HIERARCHY)

.PHONY: _check_project _check_exp init flow-list flow-run flow-ext flow-gen

_check_project:
	@if [ -z "$(PROJECT)" ]; then \
		echo "error: PROJECT is not set. Pass PROJECT=<name>, e.g. 'make $(MAKECMDGOALS) PROJECT=ai-core'."; \
		echo "       Projects currently in your checkout:"; \
		ls -1 $(RTL_LAB_HOME)/projects 2>/dev/null | sed 's/^/         - /' || true; \
		exit 1; \
	fi; \
	if [ ! -d "$(PROJ_DIR)" ]; then \
		echo "error: project '$(PROJECT)' is not in your checkout."; \
		echo "       Add it with: git sparse-checkout add projects/$(PROJECT)"; \
		exit 1; \
	fi

init: _check_project
	mkdir -p $(PROJ_DIR)/sim
	mkdir -p $(PROJ_DIR)/imp

sim: _check_project clean-sim
	cd $(RTL_LAB_HOME)/scripts/sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/sim/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

sim-sc: _check_project clean-sim
	cd $(RTL_LAB_HOME)/scripts/sim-sc && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/sim-sc/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/sim-sc/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

syn: _check_project clean-imp
	cd $(RTL_LAB_HOME)/scripts/syn && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	yosys -l $(PROJ_DIR)/imp/$(OUT_DIR)/output/yosys.log -c $(RTL_LAB_HOME)/scripts/syn/run.tcl

post-syn-sta: _check_project clean-imp
	cd $(RTL_LAB_HOME)/scripts/post-syn-sta && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(RTL_LAB_HOME)/scripts/post-syn-sta/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

post-syn-sim: _check_project clean-sim
	cd $(RTL_LAB_HOME)/scripts/post-syn-sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/post-syn-sim/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/post-syn-sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

post-syn-dpa: _check_project clean-imp
	cd $(RTL_LAB_HOME)/scripts/post-syn-dpa && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(RTL_LAB_HOME)/scripts/post-syn-dpa/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

flow-list: _check_project
	@echo "Experiments in project '$(PROJECT)':"
	@ls -1 $(PROJ_DIR)/scripts/flow 2>/dev/null | sed 's/^/  - /' || true

_check_exp: _check_project
	@if [ -z "$(EXP)" ]; then \
		echo "error: EXP is not set. Pass EXP=<experiment>, e.g. 'make $(MAKECMDGOALS) EXP=<experiment> PROJECT=$(PROJECT)'."; \
		echo "       Experiments available in project '$(PROJECT)':"; \
		ls -1 $(PROJ_DIR)/scripts/flow 2>/dev/null | sed 's/^/         - /' || true; \
		exit 1; \
	fi; \
	if [ ! -d "$(PROJ_DIR)/scripts/flow/$(EXP)" ]; then \
		echo "error: experiment '$(EXP)' does not exist in project '$(PROJECT)'."; \
		echo "       Experiments available:"; \
		ls -1 $(PROJ_DIR)/scripts/flow 2>/dev/null | sed 's/^/         - /' || true; \
		exit 1; \
	fi

flow-run: _check_exp
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/run.py

flow-ext: _check_exp
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/ext.py

flow-gen: _check_exp
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/gen.py

clean-all: _check_project
	rm -rf $(PROJ_DIR)/sim
	rm -rf $(PROJ_DIR)/imp

clean-sim: _check_project
	rm -rf $(PROJ_DIR)/sim/$(OUT_DIR)

clean-imp: _check_project
	rm -rf $(PROJ_DIR)/imp/$(OUT_DIR)
