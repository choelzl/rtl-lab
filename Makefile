# -----------------------------------------------------------------------------
# Author: Simone Machetti
# -----------------------------------------------------------------------------

TOP_LEVEL        ?= pe_top
OUT_DIR          ?= no_name
NETLIST_DIR      ?= no_name
VCD_DIR          ?= no_name
CLK_PERIOD_NS    ?= 1
PARAMS           ?= none
KEEP_HIERARCHY   ?= 0

export SEL_TOP_LEVEL       := $(TOP_LEVEL)
export SEL_OUT_DIR         := $(OUT_DIR)
export SEL_NETLIST_DIR     := $(NETLIST_DIR)
export SEL_VCD_DIR         := $(VCD_DIR)
export SEL_CLK_PERIOD_NS   := $(CLK_PERIOD_NS)
export SEL_PARAMS          := $(PARAMS)
export SEL_KEEP_HIERARCHY  := $(KEEP_HIERARCHY)

.PHONY: init

init:
	mkdir -p $(CODE_HOME)/ai-core/sim
	mkdir -p $(CODE_HOME)/ai-core/imp

sim: clean-sim
	cd $(CODE_HOME)/ai-core/scripts/sim && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR) && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/build && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(CODE_HOME)/ai-core/scripts/sim/activity.vcd ]; then \
	mv $(CODE_HOME)/ai-core/scripts/sim/activity.vcd $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/output; \
	fi

syn: clean-imp
	cd $(CODE_HOME)/ai-core/scripts/syn && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR) && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/report && \
	yosys -l $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output/yosys.log -c $(CODE_HOME)/ai-core/scripts/syn/run.tcl

post-syn-sta: clean-imp
	cd $(CODE_HOME)/ai-core/scripts/post-syn-sta && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR) && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/report && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(CODE_HOME)/ai-core/scripts/post-syn-sta/run.tcl | tee $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output/opensta.log

post-syn-sim: clean-sim
	cd $(CODE_HOME)/ai-core/scripts/post-syn-sim && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR) && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/build && \
	mkdir -p $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/output && \
	./run.sh && \
	if [ -f $(CODE_HOME)/ai-core/scripts/post-syn-sim/activity.vcd ]; then \
	mv $(CODE_HOME)/ai-core/scripts/post-syn-sim/activity.vcd $(CODE_HOME)/ai-core/sim/$(OUT_DIR)/output; \
	fi

post-syn-dpa: clean-imp
	cd $(CODE_HOME)/ai-core/scripts/post-syn-dpa && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR) && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/report && \
	mkdir -p $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output && \
	sta -no_splash -exit $(CODE_HOME)/ai-core/scripts/post-syn-dpa/run.tcl | tee $(CODE_HOME)/ai-core/imp/$(OUT_DIR)/output/opensta.log

run-regres:
	bash $(CODE_HOME)/ai-core/scripts/flow/run_regres.sh

ext-results:
	bash $(CODE_HOME)/ai-core/scripts/flow/ext_results.sh

gen-charts:
	bash $(CODE_HOME)/ai-core/scripts/flow/gen_charts.sh

clean-all:
	rm -rf $(CODE_HOME)/ai-core/sim
	rm -rf $(CODE_HOME)/ai-core/imp

clean-sim:
	rm -rf $(CODE_HOME)/ai-core/sim/$(OUT_DIR)

clean-imp:
	rm -rf $(CODE_HOME)/ai-core/imp/$(OUT_DIR)
