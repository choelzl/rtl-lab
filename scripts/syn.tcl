# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hölzl
# Synthesis flow — invoked by `edaf syn`
# -----------------------------------------------------------------------------

yosys "design -reset"

# ── Yosys-Slang plugin ────────────────────────────────────────────────────────
yosys "plugin -i $env(YOSYS_SLANG_HOME)/bin/slang.so"

# ── Read libraries ────────────────────────────────────────────────────────────
yosys "read_liberty -lib $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"
yosys "read_liberty -lib $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib"
yosys "read_liberty -lib $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib"
yosys "read_liberty -lib $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib"
yosys "read_liberty -lib $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib"

# ── Read SystemVerilog sources ─────────────────────────────────────────────────
set rtl_dir "$env(RTL_LAB_HOME)/projects/$env(PROJECT)/rtl"
set rtl_files [lsort [split [exec find $rtl_dir -name "*.sv"] "\n"]]

set inc_flags ""
foreach dir [lsort [split [exec find $rtl_dir -type d] "\n"]] {
    append inc_flags " -I $dir"
}

# PARAMS=K=V,FLAG,...  Only K=V entries become -G overrides.
set g_flags ""
if {[info exists env(PARAMS)] && $env(PARAMS) ne ""} {
    foreach param [split $env(PARAMS) ","] {
        if {[string match "*=*" $param]} {
            append g_flags " -G $param"
        }
    }
}

set kh_flag ""
if {[info exists env(KEEP_HIERARCHY)] && $env(KEEP_HIERARCHY) eq "1"} {
    set kh_flag " --keep-hierarchy"
}

yosys "read_slang --single-unit [join $rtl_files]$inc_flags --top $env(TOP)$g_flags$kh_flag"

# ── Elaboration ───────────────────────────────────────────────────────────────
yosys "hierarchy -check -top $env(TOP)"
yosys "check"

# ── Synthesis & optimizations ─────────────────────────────────────────────────
yosys "proc"
yosys "opt"
yosys "fsm"
yosys "opt"
yosys "memory"
yosys "opt"
yosys "techmap"
yosys "opt"

# ── Technology mapping ────────────────────────────────────────────────────────
yosys "dfflibmap -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib"
yosys "opt"

yosys "abc \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib \
    -script  $env(RTL_LAB_HOME)/scripts/syn_abc.tcl"

yosys "opt"
yosys "clean"

# ── Area report ───────────────────────────────────────────────────────────────
yosys "tee -o $env(RTL_LAB_HOME)/projects/$env(PROJECT)/imp/$env(OUT)/report/area.rpt stat -hierarchy \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib \
    -liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib"

# ── Flatten & write netlist ───────────────────────────────────────────────────
if {![info exists env(KEEP_HIERARCHY)] || $env(KEEP_HIERARCHY) eq "0"} {
    yosys "flatten"
    yosys "opt_clean"
    yosys "rename -hide"
}

yosys "write_verilog -noattr -noexpr -nodec $env(RTL_LAB_HOME)/projects/$env(PROJECT)/imp/$env(OUT)/output/netlist.v"
