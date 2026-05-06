# AI Core

Fixed-point multiply-accumulate Processing Elements (PEs) for AI/ML inference, implemented in SystemVerilog. The project explores several algorithmic variants — Baseline, Winograd, and Squaring — each available in standard and split-cell configurations, and characterizes them through simulation, logic synthesis, static timing analysis, and dynamic power analysis.

## Quick start

```bash
source sourceme.sh

# Pre-synthesis simulation
make sim TOP_LEVEL=top_bas_4x8 CLK_PERIOD_NS=1.0 OUT_DIR=bas_4x8

# Logic synthesis
make syn TOP_LEVEL=top_bas_4x8 OUT_DIR=bas_4x8

# Post-synthesis static timing analysis
make post-syn-sta TOP_LEVEL=top_bas_4x8 CLK_PERIOD_NS=1.0 OUT_DIR=bas_4x8_sta NETLIST_DIR=imp/bas_4x8

# Post-synthesis dynamic power analysis
make post-syn-dpa TOP_LEVEL=top_bas_4x8 CLK_PERIOD_NS=1.0 OUT_DIR=bas_4x8_dpa NETLIST_DIR=imp/bas_4x8 VCD_DIR=sim/bas_4x8
```

## Repository structure

```
.
├── rtl/                        SystemVerilog source modules
├── tb/                         Verilator testbenches
├── scripts/                    EDA flow scripts
│   ├── sim/                    Pre-synthesis simulation flow
│   │   └── run.sh              Verilator compile and run script
│   ├── syn/                    Logic synthesis flow
│   │   ├── run.tcl             Yosys top-level synthesis script (ASAP7)
│   │   ├── compile.tcl         RTL read and elaboration script
│   │   └── abc.tcl             ABC technology mapping script
│   ├── post-syn-sta/           Post-synthesis static timing analysis flow
│   │   └── run.tcl             OpenSTA timing analysis script
│   ├── post-syn-sim/           Post-synthesis gate-level simulation flow
│   │   ├── run.sh              Verilator compile and run script
│   │   └── filelist.f          Gate-level netlist and cell library filelist
│   ├── post-syn-dpa/           Post-synthesis dynamic power analysis flow
│   │   └── run.tcl             OpenSTA power analysis script
│   └── flow/                   End-to-end automation scripts
│       ├── run_regres.sh       Full regression runner across all PE variants
│       ├── ext_results.sh      Result extraction from synthesis/STA/DPA reports
│       └── gen_charts.sh       Chart generation from extracted results
├── doc/                        Documentation and results
|   |── diagrams/               Block diagrams
|   |── formulas/               Mathematical formulas
│   ├── charts/                 Comparison charts
│   │   ├── area/               Area charts (Python scripts + PNG outputs)
│   │   ├── freq/               Maximum frequency charts
│   │   └── power/              Dynamic power charts
│   └── data/                   Extracted results in tabular form (generated)
│       ├── area/               area.csv, area.md
│       ├── freq/               freq.csv, freq.md
│       └── power/              power.csv, power.md
├── sim/                        Simulation outputs (generated)
├── imp/                        Synthesis/STA/DPA outputs (generated)
├── Makefile                    Build system entry point
├── sourceme.sh                 Environment setup (tool paths, CODE_HOME)
└── CLAUDE.md                   AI assistant guidance for this repository
```

## Environment setup

Source the environment script once before running any command. It sets tool paths for Verilator, Yosys, Yosys-Slang, OpenSTA, OpenROAD, and sets `CODE_HOME`:

```bash
source sourceme.sh
```

## Typical workflow

The make targets form a pipeline where earlier steps produce artifacts consumed by later ones:

1. `make sim` — functional verification; produces `activity.vcd` for debugging purpose.
2. `make syn` — logic synthesis; produces the netlist consumed by all post-synthesis flows.
3. `make post-syn-sta` — static timing analysis from the synthesized netlist.
4. `make post-syn-sim` — gate-level functional verification; produces `activity.vcd` for debugging purpose.
5. `make post-syn-dpa` — power estimation using the synthesized netlist and the `activity.vcd` from `make post-syn-sim`.

To characterize all PE variants at once, use the automation scripts in `scripts/flow/`.

## Commands

### Pre-synthesis simulation (Verilator)

```bash
make sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> [PARAMS="KEY=VAL ..."]
```

**Parameters:**


| Parameter       | Required | Description                            |
| --------------- | -------- | -------------------------------------- |
| `TOP_LEVEL`     | yes      | RTL module to simulate                 |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds            |
| `OUT_DIR`       | yes      | Output subdirectory under `sim/`       |
| `PARAMS`        | no       | RTL elaboration parameters (see below) |


**Available `TOP_LEVEL`:**


| Top level              | Testbench                 | Verified formula                                 |
| ---------------------- | ------------------------- | ------------------------------------------------ |
| `top_bas_4x8`          | `tb_top_bas_4x8`          | `Σ(a[i]×b[i]) + acc[0]`                          |
| `top_bas_4x8_sc`       | `tb_top_bas_4x8_sc`       | `Σ(a[i]×b[i]) + acc[0]`                          |
| `top_win_4x8`          | `tb_top_win_4x8`          | `Σ[(a[i+1]+b[i])×(a[i]+b[i+1])] + Σacc`          |
| `top_win_4x8_sc`       | `tb_top_win_4x8_sc`       | Winograd with B sub-lane split + Σacc            |
| `top_sqr_4x8_sc`       | `tb_top_sqr_4x8_sc`       | `Σ[(a[k]+b_lo[k])² + 16×(a[k]+b_hi[k])²] + Σacc` |
| `top_sqr_4x8_sc_alpha` | `tb_top_sqr_4x8_sc_alpha` | `Σ(a[i]²)` or `Σ(a[i])` depending on `IS_SQUARE` |


**Accepted `PARAMS`:**


| Key            | Values   | Description                                                       |
| -------------- | -------- | ----------------------------------------------------------------- |
| `MULT_TYPE`    | `0`, `1` | Booth Radix-4 (`0`) or Radix-8 (`1`)                              |
| `IS_PIPELINED` | `0`, `1` | 2-cycle (`0`) or 3-cycle (`1`) latency                            |
| `IS_SQUARE`    | `0`, `1` | Squaring (`1`) or passthrough (`0`) — `top_sqr_4x8_sc_alpha` only |


**Testbench structure:** all testbenches share the same pattern — clock/reset generation with a configurable period, 1000 iterations of randomized inputs and accumulator values, corner cases (max-positive, min-negative, mixed-sign, zero), and self-checking via a software reference model that calls `$fatal` on any mismatch. Outputs go to `sim/<OUT_DIR>/`. A `activity.vcd` waveform is produced for debugging purpose.

### Logic synthesis (Yosys + ABC, ASAP7 target)

```bash
make syn TOP_LEVEL=<top_level> OUT_DIR=<name> [PARAMS="KEY=VAL ..."] [KEEP_HIERARCHY=1]
```

**Parameters:**


| Parameter        | Required        | Description                                                                                      |
| ---------------- | --------------- | ------------------------------------------------------------------------------------------------ |
| `TOP_LEVEL`      | yes             | RTL module to synthesize; can be any module in the hierarchy                                     |
| `OUT_DIR`        | yes             | Output subdirectory under `imp/`                                                                 |
| `PARAMS`         | no              | RTL elaboration parameters: `MULT_TYPE`, `IS_PIPELINED`, `IS_SQUARE` (same values as `make sim`) |
| `KEEP_HIERARCHY` | no (default: 0) | Preserve module boundaries in the netlist (skips `flatten`)                                      |


Outputs go to `imp/<OUT_DIR>/`.

### Post-synthesis static timing analysis (OpenSTA)

```bash
make post-syn-sta TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir>
```

**Parameters:**


| Parameter       | Required | Description                                                  |
| --------------- | -------- | ------------------------------------------------------------ |
| `TOP_LEVEL`     | yes      | RTL module name                                              |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds                                  |
| `OUT_DIR`       | yes      | Output subdirectory under `imp/`                             |
| `NETLIST_DIR`   | yes      | Directory containing the synthesized netlist from `make syn` |


Outputs go to `imp/<OUT_DIR>/`.

### Post-synthesis gate-level simulation

```bash
make post-syn-sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> [PARAMS="KEY=VAL ..."]
```

**Parameters:**


| Parameter       | Required | Description                                                                                      |
| --------------- | -------- | ------------------------------------------------------------------------------------------------ |
| `TOP_LEVEL`     | yes      | RTL module to simulate                                                                           |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds                                                                      |
| `OUT_DIR`       | yes      | Output subdirectory under `sim/`                                                                 |
| `NETLIST_DIR`   | yes      | Directory containing the synthesized netlist from `make syn`                                     |
| `PARAMS`        | no       | RTL elaboration parameters: `MULT_TYPE`, `IS_PIPELINED`, `IS_SQUARE` (same values as `make sim`) |


Outputs go to `sim/<OUT_DIR>/`. Compiles the testbench with ``define POST_SYNTH` to instantiate the flattened gate-level netlist instead of the RTL.

### Post-synthesis dynamic power analysis (OpenSTA)

```bash
make post-syn-dpa TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> VCD_DIR=<vcd_dir> [KEEP_HIERARCHY=1]
```

**Parameters:**


| Parameter        | Required        | Description                                                                                     |
| ---------------- | --------------- | ----------------------------------------------------------------------------------------------- |
| `TOP_LEVEL`      | yes             | RTL module name                                                                                 |
| `CLK_PERIOD_NS`  | yes             | Clock period in nanoseconds                                                                     |
| `OUT_DIR`        | yes             | Output subdirectory under `imp/`                                                                |
| `NETLIST_DIR`    | yes             | Directory containing the synthesized netlist from `make syn`                                    |
| `VCD_DIR`        | yes             | Directory containing `activity.vcd` from `make sim`                                             |
| `KEEP_HIERARCHY` | no (default: 0) | Also generate `power_hierarchy.rpt` with per-instance breakdown (requires hierarchical netlist) |


Outputs go to `imp/<OUT_DIR>/`.

### Automation scripts

```bash
bash scripts/flow/run_regres.sh    # full flow across all PE variants
bash scripts/flow/ext_results.sh   # extract results into doc/data/
bash scripts/flow/gen_charts.sh    # generate comparison charts in doc/charts/
```

- `run_regres.sh` — runs `make sim`, `make syn`, `make post-syn-sta`, and `make post-syn-dpa` for every PE variant and some parameter combination, reporting pass/fail per run.
- `ext_results.sh` — parses synthesis, STA, and DPA reports from `imp/` and writes area, frequency, and power tables to `doc/data/` in CSV and Markdown format.
- `gen_charts.sh` — runs the Python scripts in `doc/charts/` to generate PNG comparison charts from the data in `doc/data/`.

### Cleanup

```bash
make clean-sim OUT_DIR=<name>   # remove one simulation run
make clean-imp OUT_DIR=<name>   # remove one synthesis/STA/DPA run
make clean-all                  # remove all sim/ and imp/ directories
```

### Command parameters reference

**Make-level parameters:**


| Parameter        | Make targets                                       | Values                          | Description                                                      |
| ---------------- | -------------------------------------------------- | ------------------------------- | ---------------------------------------------------------------- |
| `TOP_LEVEL`      | sim, syn, post-syn-sta, post-syn-sim, post-syn-dpa | module name                     | RTL module to build/simulate; can be any module in the hierarchy |
| `CLK_PERIOD_NS`  | sim, post-syn-sta, post-syn-sim, post-syn-dpa      | e.g. `1.0`                      | Clock period in nanoseconds                                      |
| `OUT_DIR`        | all except clean-all                               | directory name                  | Output subdirectory under `sim/` or `imp/`                       |
| `NETLIST_DIR`    | post-syn-sta, post-syn-sim, post-syn-dpa           | e.g. `top_bas_4x8_syn`          | Directory containing the synthesized netlist from `make syn`     |
| `VCD_DIR`        | post-syn-dpa                                       | e.g. `top_bas_4x8_post-syn-sim` | Directory containing `activity.vcd` from `make post-syn-sim`     |
| `PARAMS`         | sim, syn, post-syn-sim                             | `"KEY=VAL ..."`                 | RTL elaboration parameters (see below)                           |
| `KEEP_HIERARCHY` | syn, post-syn-dpa                                  | `0` (default), `1`              | Preserve module boundaries in the netlist                        |


**RTL elaboration parameters (passed via `PARAMS="..."`):**


| Key            | Applies to                  | Values             | Description                                |
| -------------- | --------------------------- | ------------------ | ------------------------------------------ |
| `MULT_TYPE`    | BAS and WIN top-levels      | `0` (R4), `1` (R8) | Booth encoding radix                       |
| `IS_PIPELINED` | all top-levels              | `0`, `1`           | 2-cycle (`0`) or 3-cycle (`1`) latency     |
| `IS_SQUARE`    | `top_sqr_4x8_sc_alpha` only | `0`, `1`           | Squaring (`1`) or passthrough (`0`) inputs |


## PE architectures

### Common pipeline

All PE variants share the same 3-stage pipeline:

```
Input FFs (ff_n) → Partial Product Generator → Compression Tree (cpr_tree) → Output FF
```

- **Stage 1**: `ff_n` registers the `a_i` and `b_i` input arrays.
- **Stage 2**: the partial product generator produces compressed partial sums; `cpr_tree` begins reduction in stage 0 and optionally stores the result in a pipeline register (`IS_PIPELINED=1`).
- **Stage 3**: `cpr_tree` completes the reduction; `ff` registers the final 48-bit output.

With `IS_PIPELINED=1` the latency is 3 clock cycles; with `IS_PIPELINED=0` it is 2 clock cycles.

### Baseline 4x8

**Formula:** `out = Σ(a[i] × b[i]) + acc[0]`

Directly multiplies 64 pairs of 4-bit A and 8-bit B operands using a Booth multiplier array and sums the products with one 48-bit accumulator.

**Booth encoding** — selected at elaboration time by `MULT_TYPE`:


| `MULT_TYPE` | Encoding | Partial products per multiplier | Operations               |
| ----------- | -------- | ------------------------------- | ------------------------ |
| `0`         | Radix-4  | `(IN_WIDTH_A + 1) / 2`          | `{0, ±B, ±2B}`           |
| `1`         | Radix-8  | `(IN_WIDTH_A + 2) / 3`          | `{0, ±B, ±2B, ±3B, ±4B}` |


Radix-8 produces fewer partial products (faster compression) at the cost of a wider encoding table (the `±3B` term).

**Compression tree** — `cpr_tree` with 1 accumulator reduces all partial products to a 48-bit output in three stages:

```
Stage 0: 8 groups × (PP_SIZE/8) inputs → 8 groups × 2 outputs  [pipeline FF here if IS_PIPELINED=1]
Stage 1: 4 groups × 4 inputs           → 4 groups × 2 outputs
Stage 2: 2 groups × 4 inputs           → 2 groups × 2 outputs
Final:   4 outputs + 1 accumulator     → cpr_n_2 → add_n → 48-bit result
```

Between stages, `ext_n` conditionally sign/zero-extends and optionally left-shifts the compressor outputs to grow the bit width.

### Baseline 4x8 Split-Cell

**Formula:** `out = Σ(a[i] × b[i]) + acc[0]`

Split-cell variant of `top_bas_4x8`. Decomposes the 8-bit B operand as `B = B_lo + 16 × B_hi`, processing each 4-bit half with a separate narrower Booth array. This reduces the critical path compared to the full 8-bit Booth array.

**Booth encoding** — same R4/R8 selection via `MULT_TYPE` as `top_bas_4x8`, applied independently to the B_lo and B_hi halves.

**Compression tree** — `cpr_tree` with 1 accumulator, same stage structure as `top_bas_4x8`.

### Winograd 4x8

**Formula:** `out = Σ[(a[i+1]+b[i]) × (a[i]+b[i+1])] + Σacc`

Exploits the identity `(a+b)(c+d)` to pair adjacent inputs and compute a single multiply per pair, halving the multiplier count compared to BAS. Requires 3 accumulator inputs to `cpr_tree` to account for the reformulated sum.

**Booth encoding** — same R4/R8 selection via `MULT_TYPE` as BAS, but the Booth array operates on pre-summed inputs `(a[i+1]+b[i])` and `(a[i]+b[i+1])` rather than raw operands.

**Compression tree** — `cpr_tree` with 3 accumulators:

```
Final: 4 outputs + 3 accumulators → cpr_n_2 → add_n → 48-bit result
```

### Winograd 4x8 Split-Cell

**Formula:** `out = Σ[(a[i+1]+b[i]) × (a[i]+b[i+1])] + Σacc`

Split-cell variant of `top_win_4x8`. Applies the same B decomposition (`B = B_lo + 16 × B_hi`) on top of the Winograd pairing, combining both optimizations.

**Booth encoding** — same R4/R8 selection via `MULT_TYPE`, applied to each B half after Winograd pre-summation.

**Compression tree** — `cpr_tree` with 3 accumulators, same stage structure as `top_win_4x8`.

### Square 4x8 Split-Cell

**Formula:** `out = Σ[(a[k]+b_lo[k])² + 16×(a[k]+b_hi[k])²] + Σacc`

Replaces Booth multiplication with squaring via the identity `a×b = [(a+b)² − a² − b²] / 2`, decomposing B as `B_lo + 16×B_hi`. Uses dedicated `sqr_s_5_bit` squaring cells instead of a Booth multiplier array, which are more area-efficient for this computation. No `MULT_TYPE` parameter. Operates over 64 lanes.

**Compression tree** — `cpr_tree` with 3 accumulators, same stage structure as `top_win_4x8`.

### Square 4x8 Split-Cell Alpha

A reduced squaring variant with 32 input lanes and a dedicated `cpr_tree_alpha` that carries no accumulator inputs. Uses `sqr_s_4_bit` cells (signed 4-bit squarer) instead of the 5-bit cells in `top_sqr_4x8_sc`.

The `IS_SQUARE` parameter selects the operation:


| `IS_SQUARE` | Operation       | Formula    |
| ----------- | --------------- | ---------- |
| `1`         | Squaring        | `Σ(a[i]²)` |
| `0`         | Passthrough sum | `Σ(a[i])`  |

## RTL modules reference

### Primitives


| Module      | Description                                                     |
| ----------- | --------------------------------------------------------------- |
| `fa`        | 1-bit full adder                                                |
| `ha`        | 1-bit half adder                                                |
| `ff`        | WIDTH-bit D flip-flop with active-low asynchronous reset        |
| `ff_n`      | Array of SIZE D flip-flops, WIDTH bits each                     |
| `sign_ext`  | Sign extension from IN_WIDTH to OUT_WIDTH bits                  |
| `shifter_n` | Static barrel shifter for an array of values                    |
| `ext_n`     | Runtime-controlled sign/zero extension with optional left shift |
| `add_n`     | Signed (IN_WIDTH+1)-bit adder                                   |
| `adder_n`   | Signed SIZE-bit adder                                           |


### Compressor hierarchy


| Module           | Description                                                     |
| ---------------- | --------------------------------------------------------------- |
| `cpr_4_2_bit`    | 1-bit 4:2 compressor cell (two cascaded full adders)            |
| `cpr_4_2`        | Multi-bit 4:2 compressor with sign extension                    |
| `cpr_n_2`        | Tree of 4:2 compressors reducing N inputs to sum + carry        |
| `cpr_tree`       | Full 3-stage compression tree with accumulators and pipeline FF |
| `cpr_tree_alpha` | Compression tree variant for `top_sqr_4x8_sc_alpha`             |


### Booth multipliers


| Module          | Description                                               |
| --------------- | --------------------------------------------------------- |
| `booth_r4_cell` | Radix-4 Booth encoder cell: `{0, ±B, ±2B}`                |
| `booth_r4`      | Radix-4 Booth multiplier (`PP_SIZE = (A_width+1)/2`)      |
| `booth_r8_cell` | Radix-8 Booth encoder cell: `{0, ±B, ±2B, ±3B, ±4B}`      |
| `booth_r8`      | Radix-8 Booth multiplier (`PP_SIZE = (A_width+2)/3`)      |
| `mult_array`    | Array of parallel Booth multipliers (R4 or R8 selectable) |


### Squaring units


| Module            | Description                                                     |
| ----------------- | --------------------------------------------------------------- |
| `sqr_u_3_bit`     | Unsigned 3-bit squarer (combinational truth-table logic)        |
| `sqr_u_4_bit`     | Unsigned 4-bit squarer (combinational truth-table logic)        |
| `sqr_s_4_bit`     | Signed 4-bit squarer (2's complement → magnitude + sqr_u_3_bit) |
| `sqr_s_5_bit`     | Signed 5-bit squarer (2's complement → magnitude + sqr_u_4_bit) |
| `sqr_alpha_array` | Array: `a[i]²` or `a[i]` passthrough, selected by `IS_SQUARE`   |
| `add_sqr_array`   | Array: `pp[i] = (a[i]+b[i])²` using `sqr_s_5_bit`               |


### Partial product generators


| Module           | Description                                                |
| ---------------- | ---------------------------------------------------------- |
| `bas_4x8`        | Baseline 4×8 PP generator (full 8-bit B)                   |
| `bas_4x8_sc`     | Baseline split-cell (B split into B_lo and B_hi halves)    |
| `add_mult_array` | Winograd pairing: `(a[i+1]+b[i]) × (a[i]+b[i+1])` per pair |
| `win_4x8`        | Winograd 4×8 PP generator                                  |
| `win_4x8_sc`     | Winograd split-cell                                        |
| `sqr_4x8_sc`     | Squaring split-cell: `(a+b_lo)² + 16*(a+b_hi)²` per lane   |
