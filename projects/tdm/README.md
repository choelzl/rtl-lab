# TDM

Memory-interconnect designs that compare a full **crossbar** against a
**time-division-multiplexed (TDM)** scheme, measuring the delay penalty that
bank conflicts cost each architecture.

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `tdm`. Full design notes are in [doc/specs/crossbar.md](doc/specs/crossbar.md), [doc/specs/tdm.md](doc/specs/tdm.md), and the OBI protocol subset is documented in [doc/specs/obi.md](doc/specs/obi.md).

The SystemC simulation flow uses one unified project top:

- `top.hpp` is the fixed SystemC wrapper in [rtl/systemc/top.hpp](rtl/systemc/top.hpp).
- `IMPL=<name>` selects the implementation behind that wrapper.
- `IMPL=<base>,sv SV_MODS=<sv_top>` additionally selects a Verilated SV backend.
- The unified harness is [tb/systemc/tb_top.cpp](tb/systemc/tb_top.cpp).

## Quick Start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

# Native SystemC backends
edaf sim IMPL=crossbar MODE=sc
edaf sim IMPL=tdm      MODE=sc

# Verilated SV backends
edaf sim IMPL=crossbar,sv MODE=sc SV_MODS=top_crossbar
edaf sim IMPL=tdm,sv      MODE=sc SV_MODS=top_tdm
```

All commands are run from inside `projects/tdm/` or any subdirectory; `PROJECT` is inferred from the working directory.

Override the design size with `PARAMS`:

```bash
edaf sim IMPL=crossbar MODE=sc PARAMS="N_BANK=16,N_ROW=1024,WORD_BYTES=4,WORDS_PER_ROW=4"
```

For SV-backed implementations, `PARAMS` values are forwarded to Verilator as `-G` parameters.

## Implementation Selector

| `IMPL`          | Extra flags             | Backend                                                               |
| --------------- | ----------------------- | --------------------------------------------------------------------- |
| `crossbar`      | —                       | Native SystemC crossbar ([rtl/systemc/top_crossbar.hpp](rtl/systemc/top_crossbar.hpp)) |
| `crossbar,sv`   | `SV_MODS=top_crossbar`  | Verilated SV crossbar ([rtl/top_crossbar.sv](rtl/top_crossbar.sv)) wrapped in SystemC |
| `tdm`           | —                       | Native SystemC TDM ([rtl/systemc/top_tdm.hpp](rtl/systemc/top_tdm.hpp)) |
| `tdm,sv`        | `SV_MODS=top_tdm`       | Verilated SV TDM ([rtl/top_tdm.sv](rtl/top_tdm.sv)) wrapped in SystemC |

`IMPL` values are split on commas and passed to C++ as preprocessor defines (`crossbar` → `-DIMPL_CROSSBAR`, `sv` → `-DIMPL_SV`). SV variants use Verilator to compile the named `SV_MODS` top and link the resulting archive with the SystemC harness.

## Unified Top And Harness

| Component | File | Description |
| --------- | ---- | ----------- |
| `top` | [rtl/systemc/top.hpp](rtl/systemc/top.hpp) | Fixed SystemC wrapper around the selected backend. Packs the RAGU/WAGU port groups onto the flat port arrays used by the implementation tops. |
| `tb_top` | [tb/systemc/tb_top.cpp](tb/systemc/tb_top.cpp) | `sc_main`: instantiates `top`, connects the RAGU/WAGU trace drivers, runs to completion, and prints/writes timing statistics. |

The wrapper exposes this fixed port map:

| Driver | Ports | Flat OBI buses |
| ------ | ----- | -------------- |
| `RAGU_A` | 4 | 16 |
| `RAGU_B` | 2 | 8 |
| `RAGU_C` | 1 | 4 |
| `RAGU_D` | 1 | 4 |
| `RAGU_DMA` | 1 | 4 |
| `WAGU_A` | 4 | 16 |
| `WAGU_B` | 2 | 8 |
| `WAGU_D` | 1 | 4 |
| `WAGU_DMA` | 1 | 4 |

Each driver owns `NUM_REQ=4` independent OBI buses per port. The backends receive 9 read ports and 8 write ports through the same wrapper shape.

DUT submodules include [crossbar.hpp](rtl/systemc/crossbar.hpp) / [crossbar.sv](rtl/crossbar.sv), [bank.hpp](rtl/systemc/bank.hpp) / [bank.sv](rtl/bank.sv), and the TDM mapping/buffer blocks under `rtl/systemc/` and `rtl/`.

## RTL Elaboration Parameters

Passed via `PARAMS="NAME=VALUE ..."`; defaults below.

| Parameter       | Meaning                        | Default |
| --------------- | ------------------------------ | ------- |
| `N_BANK`        | number of memory banks         | 32      |
| `N_ROW`         | rows per bank                  | 1024    |
| `WORD_BYTES`    | bytes per word / OBI data beat | 4       |
| `WORDS_PER_ROW` | words per bank row             | 4       |

The unified top fixes the architectural RPORT/WPORT grouping described above and derives the flat read/write port counts from that grouping. Total bank capacity is `N_BANK * N_ROW * WORDS_PER_ROW * WORD_BYTES` bytes.

## Stimuli And Outputs

**Stimuli** live under [tb/stimuli/](tb/stimuli/). The unified harness expects one log per RAGU/WAGU trace driver: `ragu_a.log`, `ragu_b.log`, `ragu_c.log`, `ragu_d.log`, `ragu_dma.log`, `wagu_a.log`, `wagu_b.log`, `wagu_d.log`, `wagu_dma.log`. Each driver is connected to its matching RPORT/WPORT group as shown in the table above (see [tb/systemc/tb_top.cpp](tb/systemc/tb_top.cpp)).

Each stimulus file is CSV-like. The **first line** is a descriptor:

```text
start_cycle,ports_used_groups,C,R,L,store_mode
```

**WAGU access rows** (one per address group):

```text
addr,data
```

**RAGU access rows** (one per address group):

```text
addr
```

where `addr` is a hex byte address and `data` is the 128-bit write value in hex. TDM-mode RAGUs also parse the descriptor header to drive the TDM mapping; crossbar-mode drivers use it only for timing (`start_cycle`).

Select stimuli with `SEL_IN_DIR`:

```bash
SEL_IN_DIR=sample     edaf sim IMPL=crossbar MODE=sc
SEL_IN_DIR=sample_tdm edaf sim IMPL=tdm      MODE=sc
```

A bare `SEL_IN_DIR` name is resolved under `tb/stimuli/`. A value containing `/` is treated as a filesystem path. If unset, the harness defaults to `sample`.

**Outputs** are written to `sim/<SEL_OUT_DIR>/output/` (defaults to `.`):

- `compile.log` and `run.log`
- one completed-access CSV per AGU, such as `ragu_a.csv`
- `stats.log` with `actual_cycles`, `ideal_cycles`, `overhead_pct`, and per-group counts

## Statistics And Conflict Metric

The harness reports:

- **actual cycles**: measured cycles from reset release until every AGU has drained its trace.
- **ideal cycles**: the conflict-free analytical estimate.
- **overhead**: `100 * (actual - ideal) / ideal` %, i.e. the cost of structural conflicts relative to the ideal.

### Ideal Cycles

The AGU is pipelined and group-synchronized: it issues a group of `NUM_REQ` requests and advances to the next group as soon as all ports in that group are granted. It does not wait for responses, which return one cycle later and overlap the next group's address phase.

With no conflict every port is granted each cycle, so an AGU streams one group per cycle. A fixed 2-cycle pipeline fill drains the last group:

```text
ideal = max_groups + 2
```

where `max_groups` is the largest group count across all connected AGUs. Any cycles beyond `ideal` are arbitration or backend stalls and show up as overhead.

## Experiments

_None yet. Add experiment subfolders under `scripts/flow/`._
