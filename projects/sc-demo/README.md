# Sc-Demo

Minimal worked example of the SystemC simulation mode (`make sim-sc`), where a Verilated SystemVerilog module and a native SystemC module simulate together under the SystemC kernel.

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `sc-demo`.

## Quick start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

make sim-sc PROJECT=sc-demo TOP_LEVEL=mul_s CLK_PERIOD_NS=1.0 OUT_DIR=sc_demo
```

Requires the SystemC library (see the root README "Environment setup" for install steps). Run output goes to `sim/sc_demo/output/` (`compile.log`, `run.log`, `activity.vcd`); `run.log` reports `[tb_mul_s] PASS: 1000 products ...` on success.

## What the demo shows

The simulation top is C++ `sc_main`, which wires three peers together with `sc_signal` nets and runs them under the SystemC kernel:

```
sc_main
  |- sc_clock clk
  |- driver   (SystemC testbench: stimulus + checker)
  |- Vmul_s   (Verilated SV combinational signed multiplier)
  |- acc      (native SystemC clocked accumulator)
```

The driver feeds 1000 random signed operand pairs to the multiplier; the multiplier's product feeds the SystemC accumulator each cycle; the driver checks the accumulator's running sum against an independent reference and reports PASS/FAIL, then calls `sc_stop()`. This exercises a Verilated SV DUT and a hand-written SystemC module running cycle-accurately in the same simulation.

## Files

- `rtl/mul_s.sv` — self-contained combinational signed multiplier (the SV DUT, Verilated into an `sc_module`).
- `tb/systemc/tb_mul_s.cpp` — the harness; defines `sc_main` and the SystemC `driver` (stimulus + checker).
- `tb/systemc/models/acc.hpp` — header-only clocked SystemC accumulator (`SC_MODULE`), the native-SC half of the demo.

## Top-level modules

| Top level | Harness                      | Description                                                       |
| --------- | ---------------------------- | ----------------------------------------------------------------- |
| `mul_s`   | `tb/systemc/tb_mul_s.cpp`    | Signed multiplier feeding a SystemC accumulator; sum self-checked |

## RTL elaboration parameters

| Key          | Applies to | Values | Description                 |
| ------------ | ---------- | ------ | --------------------------- |
| `IN_WIDTH_A` | `mul_s`    | int    | Operand A width (default 8) |
| `IN_WIDTH_B` | `mul_s`    | int    | Operand B width (default 8) |

`PARAMS` reaches **both** the DUT (as Verilator `-G`) and the harness (mirrored as a `-D` define), so a single knob keeps the two in sync — no need to hand-edit the cpp:

```bash
make sim-sc PROJECT=sc-demo TOP_LEVEL=mul_s CLK_PERIOD_NS=1.0 OUT_DIR=sc_demo \
    PARAMS="IN_WIDTH_A=10 IN_WIDTH_B=12"
```

Widths are still bound at build time (SystemC port widths are compile-time), and `make sim-sc` rebuilds on every run, so this just works. Keep `IN_WIDTH_A + IN_WIDTH_B ≤ 32` so the ports stay in Verilator's `uint32_t` band that the harness signals assume; wider operands need the harness signal/sum types widened.

### Harness-only knobs (`TB_DEFS`)

Testbench settings that are not DUT parameters are passed as compile-time defines via `TB_DEFS` (forwarded to the harness as `-D`), again with defaults baked into the cpp:

| Define    | Default | Description                  |
| --------- | ------- | ---------------------------- |
| `N_TESTS` | `1000`  | Number of random test vectors |
| `SEED`    | `1`     | PRNG seed for the stimulus    |

```bash
make sim-sc PROJECT=sc-demo TOP_LEVEL=mul_s CLK_PERIOD_NS=1.0 OUT_DIR=sc_demo \
    PARAMS="IN_WIDTH_A=10 IN_WIDTH_B=12" TB_DEFS="N_TESTS=200 SEED=3"
```
