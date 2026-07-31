# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hölzl
# Post-synthesis DPA — invoked by `edaf dpa`
# -----------------------------------------------------------------------------

set REPORT_DIR $env(RTL_LAB_HOME)/projects/$env(PROJECT)/imp/$env(OUT)/report
file mkdir $REPORT_DIR

# ── Libraries ─────────────────────────────────────────────────────────────────
read_liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SEQ_RVT_TT_nldm_220123.lib
read_liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_SIMPLE_RVT_TT_nldm_211120.lib
read_liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_INVBUF_RVT_TT_nldm_220122.lib
read_liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_AO_RVT_TT_nldm_211120.lib
read_liberty $env(ASAP7_HOME)/lib/NLDM/asap7sc7p5t_OA_RVT_TT_nldm_211120.lib

# ── Netlist ───────────────────────────────────────────────────────────────────
read_verilog $env(RTL_LAB_HOME)/projects/$env(PROJECT)/imp/$env(NETLIST)/output/netlist.v
link_design $env(TOP)

# ── Clock ─────────────────────────────────────────────────────────────────────
set CLK_PERIOD_PS [expr {$env(CLK) * 1000}]
create_clock -name clk_i -period $CLK_PERIOD_PS [get_ports clk_i]

# ── VCD switching activity ────────────────────────────────────────────────────
set vcd_file "$env(RTL_LAB_HOME)/projects/$env(PROJECT)/sim/$env(VCD)/output/activity.vcd"
read_vcd -scope tb_$env(TOP)/$env(TOP)_i $vcd_file

report_activity_annotation -report_annotated   > $REPORT_DIR/vcd_annotated.rpt
report_activity_annotation -report_unannotated > $REPORT_DIR/vcd_unannotated.rpt

# ── Power reports ─────────────────────────────────────────────────────────────
report_power > $REPORT_DIR/power_summary.rpt

if {[info exists env(KEEP_HIERARCHY)] && $env(KEEP_HIERARCHY) eq "1"} {
    report_power -instances [get_cells -hierarchical *] > $REPORT_DIR/power_hierarchy.rpt
}
