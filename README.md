# RTL Lab

Multi-project sandbox for prototyping RTL designs. The flow (Verilator simulation, Yosys synthesis, OpenSTA timing & dynamic power) is project-agnostic and lives at the repository root; each design sits under `projects/<name>/`.

Projects:

- [`ai-core`](projects/ai-core/README.md) — fixed-point multiply-accumulate Processing Elements (PEs) for AI/ML inference. The reference project and the default.

This README documents the shared EDA flow: the `make` targets, their parameters, and the typical pipeline. For a project's designs, top-levels, RTL parameters, and experiments, see that project's own README.

## Quick start

```bash
source sourceme.sh

# Pre-synthesis simulation
make sim PROJECT=ai-core TOP_LEVEL=<top_level> CLK_PERIOD_NS=1.0 OUT_DIR=<name>

# Logic synthesis
make syn PROJECT=ai-core TOP_LEVEL=<top_level> OUT_DIR=<name>

# Post-synthesis gate-level simulation
make post-syn-sim PROJECT=ai-core TOP_LEVEL=<top_level> CLK_PERIOD_NS=1.0 OUT_DIR=<name> NETLIST_DIR=<name>

# Post-synthesis static timing analysis
make post-syn-sta PROJECT=ai-core TOP_LEVEL=<top_level> CLK_PERIOD_NS=1.0 OUT_DIR=<name> NETLIST_DIR=<name>

# Post-synthesis dynamic power analysis
make post-syn-dpa PROJECT=ai-core TOP_LEVEL=<top_level> CLK_PERIOD_NS=1.0 OUT_DIR=<name> NETLIST_DIR=<name> VCD_DIR=<name>
```

`PROJECT` defaults to `ai-core` and can be omitted. See [projects/ai-core/README.md](projects/ai-core/README.md) for the available `TOP_LEVEL` values and runnable examples.

## Repository structure

```
.
├── scripts/              # Project-agnostic EDA flow scripts
│   ├── sim/              # Pre-synthesis simulation flow
│   │   └── run.sh        # Verilator compile and run script
│   ├── syn/              # Logic synthesis flow
│   │   ├── run.tcl       # Yosys top-level synthesis script (ASAP7)
│   │   ├── compile.tcl   # RTL read and elaboration script
│   │   └── abc.tcl       # ABC technology mapping script
│   ├── post-syn-sta/     # Post-synthesis static timing analysis flow
│   │   └── run.tcl       # OpenSTA timing analysis script
│   ├── post-syn-sim/     # Post-synthesis gate-level simulation flow
│   │   ├── run.sh        # Verilator compile and run script
│   │   └── filelist.f    # Gate-level netlist and cell library filelist
│   └── post-syn-dpa/     # Post-synthesis dynamic power analysis flow
│       └── run.tcl       # OpenSTA power analysis script
├── projects/             # One subfolder per RTL project
│   └── ai-core/          # Reference project (see its README.md)
│       ├── README.md     # Project-specific documentation
│       ├── rtl/          # SystemVerilog source modules
│       ├── tb/           # Verilator testbenches
│       ├── scripts/      # Project-specific scripts
│       │   └── flow/     # End-to-end automation (one subfolder per experiment)
│       ├── doc/          # Documentation and results
│       │   ├── diagrams/ # Block diagrams
│       │   ├── formulas/ # Mathematical formulas
│       │   ├── charts/   # Comparison charts (<exp>/<chart>.png, generated)
│       │   └── data/     # Extracted results (<exp>/results.xlsx, generated)
│       ├── sim/          # Simulation outputs (generated)
│       └── imp/          # Synthesis/STA/DPA outputs (generated)
├── Makefile              # Build system entry point (PROJECT=<name> selects project)
├── sourceme.sh           # Environment setup (tool paths, CODE_HOME)
└── CLAUDE.md             # AI assistant guidance for this repository
```

All `make` targets accept `PROJECT=<name>` (default `ai-core`) to select the project they operate on. The flow scripts in `scripts/` resolve project-specific paths through the `SEL_PROJECT` env var exported by the Makefile.

## Environment setup

Source the environment script once before running any command. It sets tool paths for Verilator, Yosys, Yosys-Slang, OpenSTA, OpenROAD, and sets `CODE_HOME`:

```bash
source sourceme.sh
```

## Typical workflow

The make targets form a pipeline where earlier steps produce artifacts consumed by later ones:

1. `make sim` — functional verification; produces `activity.vcd`.
2. `make syn` — logic synthesis; produces the netlist consumed by all post-synthesis flows.
3. `make post-syn-sim` — gate-level functional verification; produces `activity.vcd` consumed by `make post-syn-dpa`.
4. `make post-syn-sta` — static timing analysis from the synthesized netlist.
5. `make post-syn-dpa` — power estimation using the synthesized netlist and the `activity.vcd` from `make post-syn-sim`.

## Commands

The `TOP_LEVEL` values and `PARAMS` keys are project-specific; the syntax below is the shared interface. See the project README for the available top-levels and elaboration parameters.

### Pre-synthesis simulation (Verilator)

```bash
make sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> [PARAMS="KEY=VAL ..."]
```

| Parameter       | Required | Description                                       |
| --------------- | -------- | ------------------------------------------------- |
| `TOP_LEVEL`     | yes      | RTL module to simulate                            |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds                       |
| `OUT_DIR`       | yes      | Output subdirectory under `sim/`                  |
| `PARAMS`        | no       | Project-specific RTL elaboration parameters       |

Outputs go to `projects/<PROJECT>/sim/<OUT_DIR>/`, including an `activity.vcd` waveform.

### Logic synthesis (Yosys + ABC, ASAP7 target)

```bash
make syn TOP_LEVEL=<top_level> OUT_DIR=<name> [PARAMS="KEY=VAL ..."] [KEEP_HIERARCHY=1]
```

| Parameter        | Required        | Description                                                  |
| ---------------- | --------------- | ------------------------------------------------------------ |
| `TOP_LEVEL`      | yes             | RTL module to synthesize; can be any module in the hierarchy |
| `OUT_DIR`        | yes             | Output subdirectory under `imp/`                             |
| `PARAMS`         | no              | Project-specific RTL elaboration parameters                  |
| `KEEP_HIERARCHY` | no (default: 0) | Preserve module boundaries in the netlist (skips `flatten`)  |

Outputs go to `projects/<PROJECT>/imp/<OUT_DIR>/`.

### Post-synthesis static timing analysis (OpenSTA)

```bash
make post-syn-sta TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir>
```

| Parameter       | Required | Description                                                  |
| --------------- | -------- | ------------------------------------------------------------ |
| `TOP_LEVEL`     | yes      | RTL module name                                              |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds                                  |
| `OUT_DIR`       | yes      | Output subdirectory under `imp/`                             |
| `NETLIST_DIR`   | yes      | Directory containing the synthesized netlist from `make syn` |

Outputs go to `projects/<PROJECT>/imp/<OUT_DIR>/`.

### Post-synthesis gate-level simulation

```bash
make post-syn-sim TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> [PARAMS="KEY=VAL ..."]
```

| Parameter       | Required | Description                                                  |
| --------------- | -------- | ------------------------------------------------------------ |
| `TOP_LEVEL`     | yes      | RTL module to simulate                                       |
| `CLK_PERIOD_NS` | yes      | Clock period in nanoseconds                                  |
| `OUT_DIR`       | yes      | Output subdirectory under `sim/`                             |
| `NETLIST_DIR`   | yes      | Directory containing the synthesized netlist from `make syn` |
| `PARAMS`        | no       | Project-specific RTL elaboration parameters                  |

Outputs go to `projects/<PROJECT>/sim/<OUT_DIR>/`. Compiles the testbench with the `POST_SYNTH` compile-time flag to instantiate the flattened gate-level netlist instead of the RTL.

### Post-synthesis dynamic power analysis (OpenSTA)

```bash
make post-syn-dpa TOP_LEVEL=<top_level> CLK_PERIOD_NS=<val> OUT_DIR=<name> NETLIST_DIR=<netlist_dir> VCD_DIR=<vcd_dir> [KEEP_HIERARCHY=1]
```

| Parameter        | Required        | Description                                                                                     |
| ---------------- | --------------- | ----------------------------------------------------------------------------------------------- |
| `TOP_LEVEL`      | yes             | RTL module name                                                                                 |
| `CLK_PERIOD_NS`  | yes             | Clock period in nanoseconds                                                                     |
| `OUT_DIR`        | yes             | Output subdirectory under `imp/`                                                                |
| `NETLIST_DIR`    | yes             | Directory containing the synthesized netlist from `make syn`                                    |
| `VCD_DIR`        | yes             | Directory containing `activity.vcd` from `make post-syn-sim`                                    |
| `KEEP_HIERARCHY` | no (default: 0) | Also generate `power_hierarchy.rpt` with per-instance breakdown (requires hierarchical netlist) |

Outputs go to `projects/<PROJECT>/imp/<OUT_DIR>/`.

### Cleanup

```bash
make clean-sim OUT_DIR=<name> # remove one simulation run
make clean-imp OUT_DIR=<name> # remove one synthesis/STA/DPA run
make clean-all                # remove all sim/ and imp/ directories
```

### Make-level parameters reference

| Parameter        | Make targets                                       | Values                          | Description                                                      |
| ---------------- | -------------------------------------------------- | ------------------------------- | ---------------------------------------------------------------- |
| `PROJECT`        | all                                                | project name                    | Project under `projects/` to operate on (default `ai-core`)      |
| `TOP_LEVEL`      | sim, syn, post-syn-sta, post-syn-sim, post-syn-dpa | module name                     | RTL module to build/simulate; can be any module in the hierarchy |
| `CLK_PERIOD_NS`  | sim, post-syn-sta, post-syn-sim, post-syn-dpa      | e.g. `1.0`                      | Clock period in nanoseconds                                      |
| `OUT_DIR`        | all except clean-all                               | directory name                  | Output subdirectory under `sim/` or `imp/`                       |
| `NETLIST_DIR`    | post-syn-sta, post-syn-sim, post-syn-dpa           | e.g. `top_bas_4x8_syn`          | Directory containing the synthesized netlist from `make syn`     |
| `VCD_DIR`        | post-syn-dpa                                       | e.g. `top_bas_4x8_post-syn-sim` | Directory containing `activity.vcd` from `make post-syn-sim`     |
| `PARAMS`         | sim, syn, post-syn-sim                             | `"KEY=VAL ..."`                 | Project-specific RTL elaboration parameters                      |
| `KEEP_HIERARCHY` | syn, post-syn-dpa                                  | `0` (default), `1`              | Preserve module boundaries in the netlist                        |
