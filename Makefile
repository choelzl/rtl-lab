# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hölzl
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

.PHONY: init vendor flow-list flow-run flow-ext flow-gen unit-test format format-sv format-sc

unit-test:
	cd $(RTL_LAB_HOME)/scripts/unit-test && \
	mkdir -p $(PROJ_DIR)/sim/unit && \
	./run.sh

init:
	mkdir -p $(PROJ_DIR)/sim
	mkdir -p $(PROJ_DIR)/imp

vendor:
	cd $(PROJ_DIR)/rtl/vendor && \
	for desc in *.vendor.hjson; do \
	python3 vendor.py $(VENDOR_ARGS) $$desc || exit 1; \
	done

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

SC_GIT_LIST := $(shell git ls-files -co --exclude-standard "*.cpp" "*.h" "*.hpp")
SV_GIT_LIST := $(shell git ls-files -co --exclude-standard "*.sv" "*.v")
SC_SOURCES := $(foreach f,$(SC_GIT_LIST),$(if $(wildcard $(f)),$(f)))
SV_SOURCES := $(foreach f,$(SV_GIT_LIST),$(if $(wildcard $(f)),$(f)))
SV_FLAGS := --indentation_spaces=4 --wrap_spaces=4 --column_limit=100 \
            --assignment_statement_alignment=align --named_port_alignment=align \
            --module_net_variable_alignment=align --port_declarations_alignment=align \
            --struct_union_members_alignment=align

SC_STYLE := "{BasedOnStyle: LLVM, IndentWidth: 4, ContinuationIndentWidth: 4, \
              TabWidth: 4, UseTab: Never, ColumnLimit: 100, \
              AlignConsecutiveAssignments: true, AlignConsecutiveDeclarations: true, \
              AlignConsecutiveMacros: true, AlignTrailingComments: true, \
              AllowShortFunctionsOnASingleLine: Inline}"
format: format-sv format-sc

format-sv:
	@echo "Formatting SystemVerilog ..."
	@$(if $(SV_SOURCES),verible-verilog-format $(SV_FLAGS) --inplace $(SV_SOURCES),echo "No SV files.")

format-sc:
	@echo "Formatting SystemC ..."
	@$(if $(SC_SOURCES),clang-format -style=$(SC_STYLE) -i $(SC_SOURCES),echo "No SC files.")