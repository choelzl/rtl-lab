# Tdm

Memory-interconnect designs that compare a full **crossbar** against a
**time-division-multiplexed (TDM)** scheme, measuring the delay penalty that
bank conflicts cost each architecture.

This project plugs into the repository-level EDA flow. See the [root README](../../README.md) for the `make` targets, their generic parameters, and the typical pipeline. This document covers the parts specific to `tdm`. The crossbar architecture is the one implemented so far — full design notes are in [doc/specs/crossbar.md](doc/specs/crossbar.md) (protocol: [doc/specs/obi.md](doc/specs/obi.md)).

The design is **pure SystemC** (no SV): the synthesizable DUT lives under `rtl/systemc/`, the verification drivers + harness under `tb/systemc/`. It runs through the `make sim-sc` flow, which builds a pure-SystemC project directly with `g++`.

## Quick start

```bash
source ../../sourceme.sh   # or: source sourceme.sh from the repository root

make sim-sc PROJECT=tdm TOP_LEVEL=top_crossbar OUT_DIR=run0
```

This builds [tb/systemc/tb_top_crossbar.cpp](tb/systemc/tb_top_crossbar.cpp), runs it on the stimuli in [tb/stimuli/](tb/stimuli/), and prints timing statistics. `tdm` is selected with `PROJECT=tdm` on every `make` command.

To run the Verilated SV DUT instead, prefix the top-level with `V` and pass the full parameter set (Verilator needs them as `-G` flags, which `PARAMS` provides):

```bash
make sim-sc PROJECT=tdm TOP_LEVEL=Vtop_crossbar OUT_DIR=run0 \
    PARAMS="N_RAGU=7 N_REQ=4 N_BANK=32 N_ROW=1024"
```

Override the design size with `PARAMS` (each becomes a `-D`):

```bash
make sim-sc PROJECT=tdm TOP_LEVEL=top_crossbar OUT_DIR=big \
    PARAMS="N_BANK=16 N_REQ=8"
```

## Top-level modules

| Top-level      | File                                                         | Description                                                                                                                                    |
| -------------- | ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------------------------------------- |
| `top_crossbar`  | [rtl/systemc/top_crossbar.hpp](rtl/systemc/top_crossbar.hpp) | DUT: the `N_MGR × N_BANK` crossbar interconnect plus `N_BANK` memory banks. Exposes the manager-side OBI ports; the harness attaches the AGUs. |
| `Vtop_crossbar` | [rtl/top_crossbar.sv](rtl/top_crossbar.sv)             | Alternative DUT: the two-level SV crossbar (Verilated into a SystemC module). Use `TOP_LEVEL=Vtop_crossbar` to select it. The same harness and AGUs are used; pass the full parameter set via `PARAMS` (see below). |

DUT submodules: [crossbar.hpp](rtl/systemc/crossbar.hpp) (round-robin per-bank arbiter, word-interleaved routing) and [bank.hpp](rtl/systemc/bank.hpp) (single-port OBI RAM, 1-cycle latency).

## RTL elaboration parameters

Passed via `PARAMS="NAME=VALUE …"` (forwarded as `-DNAME=VALUE`); defaults below.

| Parameter    | Meaning                        | Default |
| ------------ | ------------------------------ | ------- |
| `N_RAGU`     | number of AGUs (managers)      | 2       |
| `N_WAGU`     | number of AGUs (managers)      | 2       |
| `N_REQ`      | request ports per AGU          | 4       |
| `N_BANK`     | number of memory banks         | 32      |
| `N_ROW`      | rows (words) per bank          | 1024    |
| `WORD_BYTES` | bytes per word / OBI data beat | 4       |

`N_WAGU · N_REQ` request ports; total capacity `N_BANK · N_ROW · WORD_BYTES` bytes. See [doc/specs/crossbar.md](doc/specs/crossbar.md#configuration-parameters) for the address decode and full semantics.

## Testbenches

| Testbench         | File                                                             | Drives                                                                                                                       |
| ----------------- | ---------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| `tb_top_crossbar` | [tb/systemc/tb_top_crossbar.cpp](tb/systemc/tb_top_crossbar.cpp) | `sc_main`: instantiates `top_crossbar` (native SC) or `Vtop_crossbar` (Verilated SV, selected by `TOP_LEVEL=Vtop_crossbar`) + `N_AGU` [AGUs](tb/systemc/agu_crossbar.hpp), runs to completion, prints statistics. The Verilated wrapper is in [tb/systemc/top_crossbar_sv.hpp](tb/systemc/top_crossbar_sv.hpp). |

**Stimuli** ([tb/stimuli/](tb/stimuli/)): `mem_<i>.log`, one CSV per AGU. The **first line** carries the TDM mapping parameters (`num_banks,bank_width,C,R,L,store_mode`; consumed by the TDM AGU, skipped by the crossbar AGU); the access rows follow as `addr,we,data` (hex byte address, 1=write/0=read, write value or empty on reads). The `sample` stimuli write random data to a set of addresses then read them back. The stimuli folder is selected with `IN_DIR`: a bare name (e.g. `IN_DIR=sample`) is a subfolder of `tb/stimuli/`, while a value containing a `/` (e.g. `IN_DIR=/data/my_stim` or `IN_DIR=./cases/run1`) is used as a filesystem path (relative paths resolve from where you run `make`). Unset, it defaults to `tb/stimuli/sample`.

**Outputs** (in `sim/<OUT_DIR>/output/`): `out_<i>.log` (per-AGU completed accesses `cycle,addr,we,data` for inspection — a read should return the value of its matching write), plus `compile.log` / `run.log`.

## Statistics & the conflict metric

The harness reports:

- **actual cycles** — measured cycles from reset release until every AGU has drained its trace.
- **ideal cycles** — the conflict-free analytical estimate (derivation below).
- **delay penalty** — `100 · (actual − ideal) / ideal` %, i.e. the cost of bank conflicts.

### How the ideal (conflict-free) cycles are computed

The AGU is **pipelined and group-synchronized**: it issues a group of `N_REQ` requests and advances to the next group as soon as all `N_REQ` ports are **granted** — it does *not* wait for the responses, which return one cycle later and overlap the next group's address phase. With no bank conflict every port is granted each cycle, so it issues **one group per cycle** (throughput 1 group/cycle).

A single AGU with `G` groups therefore finishes in `G + 2` cycles: `G` to stream the groups (1/cycle), plus a fixed 2-cycle pipeline fill to drain the last group (issue → grant → response). Measured exactly: G=2→4, G=4→6, G=8→10.

In the conflict-free ideal the AGUs never interfere, so they run fully in parallel and the run ends with the slowest one:

```
ideal = G_max + 2,   G_a = ceil(len_a / N_REQ),  G_max = max_a G_a
```

Any cycles beyond `ideal` are arbitration stalls — the conflict penalty. (The `+2` fill is a property of the module timing; it is a named constant in the harness and must be updated if that timing changes.)

## Experiments (automation scripts)

_None yet. Add experiment subfolders under `scripts/flow/`._
