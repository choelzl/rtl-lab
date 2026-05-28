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
KEEP_HIERARCHY ?= 0

PROJ_DIR := $(CODE_HOME)/rtl-lab/projects/$(PROJECT)

export SEL_PROJECT        := $(PROJECT)
export SEL_TOP_LEVEL      := $(TOP_LEVEL)
export SEL_OUT_DIR        := $(OUT_DIR)
export SEL_NETLIST_DIR    := $(NETLIST_DIR)
export SEL_VCD_DIR        := $(VCD_DIR)
export SEL_CLK_PERIOD_NS  := $(CLK_PERIOD_NS)
export SEL_PARAMS         := $(PARAMS)
export SEL_KEEP_HIERARCHY := $(KEEP_HIERARCHY)

.PHONY: _check_project init

_check_project:
	@if [ -z "$(PROJECT)" ]; then \
		echo "error: PROJECT is not set. Pass PROJECT=<name>, e.g. 'make $(MAKECMDGOALS) PROJECT=ai-core'."; \
		echo "       Projects currently in your checkout:"; \
		ls -1 $(CODE_HOME)/rtl-lab/projects 2>/dev/null | sed 's/^/         - /' || true; \
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
	cd $(CODE_HOME)/rtl-lab/scripts/sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(CODE_HOME)/rtl-lab/scripts/sim/activity.vcd ]; then \
	mv $(CODE_HOME)/rtl-lab/scripts/sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

syn: _check_project clean-imp
	cd $(CODE_HOME)/rtl-lab/scripts/syn && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	yosys -l $(PROJ_DIR)/imp/$(OUT_DIR)/output/yosys.log -c $(CODE_HOME)/rtl-lab/scripts/syn/run.tcl

post-syn-sta: _check_project clean-imp
	cd $(CODE_HOME)/rtl-lab/scripts/post-syn-sta && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(CODE_HOME)/rtl-lab/scripts/post-syn-sta/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

post-syn-sim: _check_project clean-sim
	cd $(CODE_HOME)/rtl-lab/scripts/post-syn-sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(CODE_HOME)/rtl-lab/scripts/post-syn-sim/activity.vcd ]; then \
	mv $(CODE_HOME)/rtl-lab/scripts/post-syn-sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

post-syn-dpa: _check_project clean-imp
	cd $(CODE_HOME)/rtl-lab/scripts/post-syn-dpa && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(CODE_HOME)/rtl-lab/scripts/post-syn-dpa/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

run-regres: _check_project
	python3 $(PROJ_DIR)/scripts/flow/regres/run.py

ext-results: _check_project
	python3 $(PROJ_DIR)/scripts/flow/regres/ext.py

gen-charts: _check_project
	python3 $(PROJ_DIR)/scripts/flow/regres/gen.py

clean-all: _check_project
	rm -rf $(PROJ_DIR)/sim
	rm -rf $(PROJ_DIR)/imp

clean-sim: _check_project
	rm -rf $(PROJ_DIR)/sim/$(OUT_DIR)

clean-imp: _check_project
	rm -rf $(PROJ_DIR)/imp/$(OUT_DIR)
