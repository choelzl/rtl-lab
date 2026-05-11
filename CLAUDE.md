# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Environment Setup

Before running any commands, source the environment script to set tool paths and `CODE_HOME`:

```bash
source sourceme.sh
```

This sets up Verilator, Yosys, Yosys-Slang, OpenSTA, OpenROAD, and `CODE_HOME=/home/simone/work/my_code`.

## Commands

**Pre-synthesis simulation** (Verilator):
```bash
make sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> [PARAMS="KEY=VAL ..."]
```

**Logic synthesis** (Yosys + ABC, targeting ASAP7):
```bash
make syn TOP_LEVEL=<top_level> OUT_DIR=<name> [PARAMS="KEY=VAL ..."] [KEEP_HIERARCHY=1]
```

Set `KEEP_HIERARCHY=1` to preserve module boundaries in the output netlist (skips `flatten`). Default is `0` (fully flattened netlist).

`TOP_LEVEL` can be any module in the hierarchy (e.g. `cpr_tree_4x8`, `mult_array`), not only PE top-levels. Module parameters are passed via `PARAMS` as a space-separated list of `KEY=VALUE` pairs (e.g. `PARAMS="MULT_TYPE=1 PP_SIZE=32"`).

**Post-synthesis static timing analysis** (OpenSTA):
```bash
make post-syn-sta TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir>
```

**Post-synthesis gate-level simulation**:
```bash
make post-syn-sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> [PARAMS="KEY=VAL ..."]
```

**Post-synthesis dynamic power analysis**:
```bash
make post-syn-dpa TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> VCD_DIR=<vcd_dir> [KEEP_HIERARCHY=1]
```

Set `KEEP_HIERARCHY=1` (requires a hierarchical netlist from `make syn ... KEEP_HIERARCHY=1`) to also generate `power_hierarchy.rpt` with a per-instance power breakdown.

**Cleanup**:
```bash
make clean-sim OUT_DIR=<name>   # remove one sim run
make clean-imp OUT_DIR=<name>   # remove one imp run
make clean-all                  # remove all sim/ and imp/
```

Outputs go to `sim/<OUT_DIR>/` (simulation) or `imp/<OUT_DIR>/` (synthesis/STA/DPA).

## Architecture

This project implements **Processing Elements (PEs)** for AI/ML inference, specifically fixed-point multiply-accumulate arrays, written in SystemVerilog.

### PE Variants (top-level modules in `rtl/`)

| Module                 | Algorithm                                           | Accumulators | Array                   |
|------------------------|-----------------------------------------------------|--------------|-------------------------|
| `top_bas_4x8`          | Baseline (extended)                                 | 1            | —                       |
| `top_bas_8x8`          | Baseline 8-bit × 8-bit                              | 1            | 32× (8-bit A × 8-bit B) |
| `top_bas_4x8_sc`       | Baseline Booth Radix-4/8, split-cell (4×4 sub-muls) | 1            | 64× (4-bit A × 8-bit B) |
| `top_win_4x8`          | Winograd                                            | 3            | —                       |
| `top_win_4x8_sc`       | Winograd, split-cell (4×4 sub-muls)                 | 3            | 64× (4-bit A × 8-bit B) |
| `top_sqr_4x8_sc`       | Squaring, split-cell (4×4 sub-muls)                 | 3            | 64× (4-bit A × 8-bit B) |
| `top_sqr_8x8`          | Squaring 8-bit × 8-bit                              | 3            | 32× (8-bit A × 8-bit B) |
| `top_sqr_4x8_sc_alpha` | Squaring reduced variant, 32 lanes, no accumulators | 0            | 32× (4-bit A)           |
| `top_sqr_8x8_alpha`    | Squaring 8-bit, no accumulator                      | 0            | 16× (8-bit A)           |

All variants share the same 3-stage pipeline:
```
Input FFs (ff_n) → Partial Product Generator → Compression Tree → Output FF
```

### Key RTL Modules

- **`booth_r4.sv` / `booth_r8.sv`** — Radix-4/8 Booth encoder cells; selected via `MULT_TYPE` parameter (0 = R4, 1 = R8)
- **`mult_array.sv`** — Instantiates the correct Booth encoder array
- **`bas_4x8.sv` / `bas_8x8.sv` / `bas_4x8_sc.sv` / `win_4x8_sc.sv` / `add_sqr_s_5_bit_array.sv` / `add_sqr_s_9_bit_array.sv` / `sqr_s_8_bit_alpha_array.sv`** — Partial product generators for each PE variant
- **`cpr_tree_4x8.sv`** — 3-stage 4-to-2 compression tree for 4×8 PEs
- **`cpr_tree_8x8.sv`** — 2-stage 4-to-2 compression tree for 8×8 PEs
- **`cpr_tree_4x8_alpha.sv`** — 3-stage compression tree for `top_sqr_4x8_sc_alpha` (no accumulators)
- **`cpr_tree_8x8_alpha.sv`** — 2-stage compression tree for `top_sqr_8x8_alpha` (no accumulators)
- **`cpr_n_2.sv` → `cpr_4_2.sv` → `cpr_4_2_bit.sv`** — Hierarchical 4-to-2 compressor building blocks
- **`ff.sv` / `ff_n.sv`** — Pipeline registers; `ff_n` is an array of N flip-flops
- **`fa.sv` / `ha.sv`** — Full adder / half adder primitives

### Parameter Conventions

- `MULT_TYPE`: 0 = Booth Radix-4, 1 = Booth Radix-8 (BAS and WIN top-levels)
- `IS_PIPELINED`: 0 = 2-cycle latency, 1 = 3-cycle latency (all top-levels)
- `IS_SQUARE`: 0 = passthrough sum `Σ(a[i])`, 1 = squaring `Σ(a[i]²)` (`top_sqr_4x8_sc_alpha` and `top_sqr_8x8_alpha`)
- `IN_SIZE`: number of multiply-accumulate lanes (typically 64)
- `IN_WIDTH_A` / `IN_WIDTH_B`: bit widths of operands A and B
- `ACC_SIZE`: number of accumulator inputs to the compression tree
- `ACC_WIDTH` / `OUT_WIDTH`: output precision (typically 48 bits)

### Testbenches

Each PE has a matching testbench at `tb/tb_<top_level>.sv`. Both the pre-synthesis (`make sim`) and post-synthesis (`make post-syn-sim`) flows generate a VCD activity trace (`activity.vcd`). The post-synthesis VCD is the recommended input for `make post-syn-dpa`.

### Automation Scripts

```bash
bash scripts/flow/run_regres.sh   # full flow across all PE variants
bash scripts/flow/ext_results.sh  # extract results into doc/data/
bash scripts/flow/gen_charts.sh   # generate comparison charts in doc/charts/
```
