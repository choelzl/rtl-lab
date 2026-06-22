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
IN_DIR         ?=
EXP            ?=
VENDOR_ARGS    ?=

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
export SEL_IN_DIR         := $(if $(findstring /,$(IN_DIR)),$(abspath $(IN_DIR)),$(IN_DIR))

.PHONY: init flow-list flow-run flow-ext flow-gen unit-test

unit-test:
	cd $(RTL_LAB_HOME)/scripts/unit-test && \
	mkdir -p $(PROJ_DIR)/sim/unit && \
	./run.sh

init:
	mkdir -p $(PROJ_DIR)/sim
	mkdir -p $(PROJ_DIR)/imp

sim: clean-sim
	cd $(RTL_LAB_HOME)/scripts/sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/sim/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

sim-sc: clean-sim
	cd $(RTL_LAB_HOME)/scripts/sim-sc && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/sim-sc/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/sim-sc/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

syn: clean-imp
	cd $(RTL_LAB_HOME)/scripts/syn && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	yosys -l $(PROJ_DIR)/imp/$(OUT_DIR)/output/yosys.log -c $(RTL_LAB_HOME)/scripts/syn/run.tcl

post-syn-sta: clean-imp
	cd $(RTL_LAB_HOME)/scripts/post-syn-sta && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(RTL_LAB_HOME)/scripts/post-syn-sta/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

post-syn-sim: clean-sim
	cd $(RTL_LAB_HOME)/scripts/post-syn-sim && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/build && \
	mkdir -p $(PROJ_DIR)/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(RTL_LAB_HOME)/scripts/post-syn-sim/activity.vcd ]; then \
	mv $(RTL_LAB_HOME)/scripts/post-syn-sim/activity.vcd $(PROJ_DIR)/sim/$(OUT_DIR)/output; \
	fi

post-syn-dpa: clean-imp
	cd $(RTL_LAB_HOME)/scripts/post-syn-dpa && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR) && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/report && \
	mkdir -p $(PROJ_DIR)/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(RTL_LAB_HOME)/scripts/post-syn-dpa/run.tcl | tee $(PROJ_DIR)/imp/$(OUT_DIR)/output/opensta.log

flow-run:
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/run.py

flow-ext:
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/ext.py

flow-gen:
	python3 $(PROJ_DIR)/scripts/flow/$(EXP)/gen.py

clean-all:
	rm -rf $(PROJ_DIR)/sim
	rm -rf $(PROJ_DIR)/imp

clean-sim:
	rm -rf $(PROJ_DIR)/sim/$(OUT_DIR)

clean-imp:
	rm -rf $(PROJ_DIR)/imp/$(OUT_DIR)
