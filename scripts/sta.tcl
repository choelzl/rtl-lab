# -----------------------------------------------------------------------------
# Author: Simone Machetti, Cedric Hölzl
# Post-synthesis STA — invoked by `edaf sta`
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
link_design  $env(TOP)

# ── Clock ─────────────────────────────────────────────────────────────────────
set CLK_PERIOD_PS [expr {$env(CLK) * 1000}]
create_clock -name clk_i -period $CLK_PERIOD_PS [get_ports clk_i]

# ── Reports ───────────────────────────────────────────────────────────────────
report_checks -unconstrained > $REPORT_DIR/unconstrained.rpt
report_checks \
    -path_delay max \
    -fields {slew cap input_pins} \
    -digits 4 \
    -group_count 10 \
    > $REPORT_DIR/critical_paths.rpt
report_wns > $REPORT_DIR/wns.rpt
report_tns > $REPORT_DIR/tns.rpt
