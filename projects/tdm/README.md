# TDM

Memory-interconnect designs that compare a full **crossbar** against a
**time-division-multiplexed (TDM)** scheme, measuring the delay penalty that
bank conflicts cost each architecture.

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `tdm`. Full design notes are in [doc/specs/crossbar.md](doc/specs/crossbar.md), [doc/specs/tdm.md](doc/specs/tdm.md), and the OBI protocol subset is documented in [doc/specs/obi.md](doc/specs/obi.md).

The SystemC simulation flow now uses one project top:

- `TOP_LEVEL=top` selects the unified SystemC wrapper in [rtl/systemc/top.hpp](rtl/systemc/top.hpp).
- `IMPL=<name>` selects the implementation behind that wrapper.
- The unified harness is [tb/systemc/tb_top.cpp](tb/systemc/tb_top.cpp).

## Quick Start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=crossbar OUT_DIR=run_crossbar
make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=tdm      OUT_DIR=run_tdm
```

`PROJECT=tdm` selects this project, `TOP_LEVEL=top` selects the single SystemC testbench/wrapper pair, and `IMPL` selects the backend/blackbox used by the wrapper.

Override the design size with `PARAMS`:

```bash
make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=top_crossbar OUT_DIR=big \
    PARAMS="N_BANK=16 N_ROW=1024 WORD_BYTES=4 WORDS_PER_ROW=4"
```

For SV-backed implementations, `PARAMS` is also forwarded to Verilator as `-G` parameters. Keep the parameter set complete enough for the selected SV top.

## Implementation Selector

| `IMPL`         | Backend selected by `top`                              | Notes                                                                 |
| -------------- | ------------------------------------------------------- | --------------------------------------------------------------------- |
| `crossbar`     | Native SystemC crossbar in [rtl/systemc/top_crossbar.hpp](rtl/systemc/top_crossbar.hpp) | Default-style reference backend.                                      |
| `top_crossbar` | Verilated SV crossbar in [rtl/top_crossbar.sv](rtl/top_crossbar.sv) | Uses the same harness through the SystemC wrapper.                    |
| `tdm`          | Native SystemC TDM in [rtl/systemc/top_tdm.hpp](rtl/systemc/top_tdm.hpp) | Uses TDM-mode RAGU stimulus parsing and defaults to `sample_tdm`.     |
| `top_tdm`      | Verilated SV TDM in [rtl/top_tdm.sv](rtl/top_tdm.sv) | SV-backed TDM backend selected through the same `TOP_LEVEL=top` flow. |

The `sim-sc` script uppercases `IMPL` and passes it to C++ as `-DIMPL_<NAME>`. The `top_*` implementations are SV-backed and use the matching `rtl/<IMPL>.sv` as the Verilator `--top-module`; native `crossbar` and `tdm` build directly with `g++`.

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

Each stimulus file is CSV-like. The first line carries the TDM mapping parameters:

```text
num_banks,bank_width,C,R,L,store_mode
```

Access rows follow as:

```text
addr,we,data
```

where `addr` is a hex byte address, `we` is `1` for write and `0` for read, and `data` is the write value or empty on reads. Crossbar-mode AGUs skip the first metadata line; TDM-mode RAGUs parse and validate it.

Select stimuli with `IN_DIR`:

```bash
make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=crossbar IN_DIR=sample OUT_DIR=run_sample
make sim-sc PROJECT=tdm TOP_LEVEL=top IMPL=tdm      IN_DIR=sample_tdm OUT_DIR=run_sample_tdm
```

A bare `IN_DIR` name is resolved under `tb/stimuli/`. A value containing `/` is treated as a filesystem path, relative to where you run `make` unless absolute. If unset, the harness defaults to `sample` for crossbar mode and `sample_tdm` for TDM mode.

**Outputs** are written to `sim/<OUT_DIR>/output/`:

- `compile.log` and `run.log`
- one completed-access CSV per AGU, such as `ragu_a.csv`
- `stats.log` with `actual_cycles`, `ideal_cycles`, `overhead_pct`, and per-group counts
- `activity.vcd` when produced by the flow

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
